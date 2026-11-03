// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// HeadingBridge.h - heading/roll fall-back (coast) module for dual-antenna GNSS
//
// WHY IT EXISTS (2026-09-21 test drives; measurement details in the concept doc)
//
// The UM982 does not go SILENT when its dual-antenna attitude degrades. It keeps
// sending $GNHPR at 10 Hz with the whole attitude field zeroed:
//
//     $GNHPR,182639.40,000.0000,000.0000,000.0000,0,0,0.00,0000*6E
//             time      heading   pitch     roll    QF sat
//
// QF = 0, sat = 0, heading = 0, roll = 0 - while GGA keeps reporting RTK fixed
// (fixQ = 4) and VTG keeps reporting a valid course over ground. Measured event
// length: 90 ... 910 ms under trees, 600 ms and 2400 ms on a road.
//
// The second failure mode is worse because nothing flags it: 1.8 ... 2.9 % of
// the frames carrying QF = 4/5 hold a GARBAGE attitude - heading steps of
// 8 ... 62 deg per 100 ms plus roll spikes to +-47 deg in the same frame, while
// the trajectory (from the position stream) stays straight and the course over
// ground stays within 1 ... 2 deg. Those frames pass the existing
// headingQualityMask and reach AgOpenGPS unchanged.
//
// CONSEQUENCES IN THE CURRENT FIRMWARE
//   * gpsData.dualHeading/dualRoll are 0 during a zeroed frame and
//     NAVProcessor::formatPAOGIMessage() copies both into PAOGI without any
//     validity test -> AOG computes fixHeading = 0 + dualHeadingOffset = 90 deg
//     (Position.designer.cs:701, UDPComm.Designer.cs:85) while the machine
//     really travels at e.g. 165 deg; pivot/steer axle positions rotate by that
//     error (Position.designer.cs:1220 ff) - the steering kick.
//   * dualRoll = 0 in the same frames makes AOG's roll correction
//     (sin(roll) * antennaHeight) jump by up to ~1 m for single fixes.
//
// WHAT THIS MODULE DOES
//   It turns the raw attitude into a QUALIFIED attitude (plausibility gate) and
//   keeps heading and roll alive from the sources that were measured to survive
//   the attitude loss:
//
//     source              measured accuracy                      during loss
//     ----------------    -------------------------------------  -----------
//     dual heading (HPR)  p50 0.5...0.8 deg/100 ms, but 1.8...    no (it IS
//                         2.9 % garbage frames                    the fault)
//     course over ground  road: mean|d| 0.76, p90 1.85 deg        yes
//     (VTG)               field: mean|d| 6.5, p90 15 deg
//     fix-to-fix from     road: 1...3 deg, field: 3...11 deg      yes
//     the position stream (per 0.5 m of travel)
//     kinematic model     R2 0.89 (1 s, outlier gated),           yes
//     v*tan(delta)/L      residual RMS 4.7 deg
//
//   They are combined in a 2-state Kalman filter with the heading and the
//   EFFECTIVE WHEEL-ANGLE BIAS as states. The bias state is not cosmetic:
//   integrating v*tan(delta)/L open-loop drifts 60...80 deg within a minute
//   (measured in both data sets), because a residual wheel-angle error of
//   1...2 deg is 1.5...8 deg/s of false yaw rate at 2.5...10 m/s. With the bias
//   state fed by COG/fix-to-fix, the same data gives p50 2.8...6.0 deg over the
//   whole 65...102 s run without any attitude measurement at all.
//
// NOT A REPLACEMENT FOR AN IMU - a fall-back. It buys the seconds the receiver
// drops and never claims more than the state machine allows:
//
//   LIVE     attitude plausible and fresh  -> raw values, no bridging
//   BRIDGE   attitude missing/rejected     -> fused estimate, budget counted
//   COAST    beyond bridgeMaxMs/bridgeMaxM -> fused estimate, marked degraded
//   TIMEOUT  beyond timeoutMs/timeoutMaxM  -> configurable action towards AOG
//
// RAM: a few hundred bytes only. RAM1 leaves 15008 bytes for local variables
// (linker report 2026-09-21) and HeadingMonitor already holds 12.5 KB of ring
// buffer, so this class keeps two positions and no history of its own.

#ifndef HEADING_BRIDGE_H
#define HEADING_BRIDGE_H

#include "Arduino.h"
#include <stdint.h>

class HeadingBridge {
public:
    enum class State : uint8_t {
        INIT = 0,     // no trusted attitude seen yet
        LIVE,         // attitude valid and trusted
        BRIDGE,       // attitude missing/rejected, position supported, short
        COAST,        // attitude missing/rejected, position supported, longer
        DEADRECKON,   // neither attitude nor position/COG usable
        TIMEOUT       // beyond the dead-reckoning budget -> TimeoutAction
    };

    // What the firmware should do towards AgOpenGPS when TIMEOUT is reached.
    enum class TimeoutAction : uint8_t {
        KEEP_BRIDGED = 0,    // keep sending the estimate
        LAST_VALID_QF0,      // send the last valid heading, fixQuality = 0
        STOP_PAOGI           // send nothing -> AOG's own GPS watchdog reacts
    };

    // Why the last attitude frame was not trusted (logging / web UI).
    enum class RejectReason : uint8_t {
        NONE = 0, GAP, QF, SENTENCE_ZEROED, ROLL_ENVELOPE, ROLL_STEP,
        HEADING_STEP, COG_CROSSCHECK, BASELINE_LENGTH, SETTLING, NOT_YET
    };


    struct Config {
        // ACTIVE by default (operator decision 2026-09-22): the plausibility gates
        // filter the attitude and the fall-back publishes heading/roll for
        // AgOpenGPS. Set to false for a pure measurement run (gates and estimates
        // still computed and logged, nothing downstream follows them).
        bool enabled = true;

        // ---- Geometry --------------------------------------------------------
        // AOG's wheelbase (vehicle.xml setVehicle_wheelbase). The caller keeps
        // this up to date from the VWAS config via setWheelbaseM(); it is the
        // geometric length L in omega = v * tan(steerScale * delta + bias) / L.
        float wheelbaseM = 2.50f;
        // Ratio between the angle the VWAS reports and the angle that actually
        // produces the measured curvature. 1.0 = "trust the AOG/VWAS scale".
        //
        // MEASURED on the 2026-09-21 logs (1 s intervals, outlier gated, TLS fit):
        // the machine turned only 0.71 of the geometric yaw rate with the reported
        // wheel angle, i.e. this factor is ~1.38 (2.14 m wheelbase -> 2.95 m
        // effective wheelbase). Causes: VWAS scale error, tyre slip/understeer and
        // the non-linear part of the steering geometry that AOG's ackermanFix does
        // not cover. With active COG/fix-to-fix measurements the filter corrects a
        // wrong value continuously; it only matters during a real GNSS outage.
        // The measured recommendation is reported live (status.steerScaleEstimate).
        float steerScale = 1.0f;
        float steerScaleLimit = 2.0f;     // plausibility clamp
        float steerScaleMin = 0.6f;       // estimator clamp (see applyMeasurement)
        // Minimum accumulated geometric turn (deg) before the scale estimator
        // accepts a sample. Below that the innovation is dominated by noise and the
        // estimator would chase it. The turn is COLLECTED over several attitude
        // measurements (one 100 ms step only produces 1...2 deg, see applyMeasurement)
        // and capped by steerScaleWindowMaxMs so the window cannot span an unbounded
        // stretch of driving.
        float steerScaleMinSignalDeg = 5.0f;
        uint32_t steerScaleWindowMaxMs = 3000;
        // Self-calibration. The scale is a MACHINE CONSTANT (tyres, steering
        // geometry, load distribution), so the module learns it, applies it, and
        // stores it in the EEPROM - there is nothing for the operator to enter, and
        // nothing that changes from day to day.
        //   * samples: accepted estimator updates (curved, attitude trusted) before
        //     the estimate counts as learned,
        //   * spread: the estimate must have stopped moving (deg/deg per update),
        //   * applyTolerance: how far the learned value must be from the active one
        //     before it is written back and persisted.
        uint16_t steerScaleLearnMinSamples = 300;
        float steerScaleLearnMaxSpread = 0.02f;
        float steerScaleApplyTolerance = 0.03f;
        // ---- Applying learned values: PASSIVE by default ---------------------
        // Operator decision 2026-09-23. The estimators keep running and report
        // everywhere (HR columns, HB1/HB2, web UI) - but the kinematic model keeps
        // using the configured values until the estimates have been reviewed against
        // real measurements. Reason: with the fix-signal repair (V4, 2026-09-23) the
        // estimators get ~10 updates/s instead of ~1 per drive, so the steering-scale
        // learner would reach its 300-sample threshold within seconds and could
        // commit a value it never had the data to earn. A wrong scale silently
        // corrupts every fall-back heading, and it would be written to the EEPROM.
        bool applyLearnedSteerScale = false;
        bool applyLearnedSpeedScale = false;
        // Speed scale learning (member comment in the private section): uses the
        // projection's along-track residual - exactly the "did we travel as far as
        // we thought" question. A measurement, not a machine constant, so it is NOT
        // persisted; it re-learns within seconds of driving.
        bool  learnSpeedScale = true;
        float speedScaleAlpha = 0.02f;     // ~50 fixes at 10 Hz
        float speedScaleLimit = 0.10f;     // +-10 %, beyond that the VTG is broken
        uint16_t speedScaleMinSamples = 50;
        float speedScaleMinPathM = 0.5f;   // needs a real interval

        // ---- Direction of travel (forward / reverse) -------------------------
        // OPERATOR RULE (2026-09-23): the published heading is the vehicle FRONT
        // axis. Driving forward or backwards along a straight line does NOT change
        // it - the front axis does not flip when the driver reverses.
        // Consequence: the attitude heading is correct as it comes (the antennas
        // are bolted to the front axis), while the fall-back sources measure the
        // DIRECTION OF TRAVEL (COG, fix-to-fix, projection) and have to be mirrored
        // by 180 deg while reversing.
        // AgOpenGPS arrives at the same direction from the other side: its own
        // reverse detection feeds on "direction of travel vs. heading"
        // (GPS/Forms/Position.designer.cs:738-754 dual antenna, reverse when the
        // difference exceeds 2 rad = 114.6 deg; :473-523 without IMU, 0.2/0.8
        // filtered, 1.57 rad = 90 deg, and "changing direction" pauses the
        // decision entirely), and in the single-antenna path it adds 180 deg to the
        // travel direction ("newGPSHeading += Math.PI") so that everything
        // downstream always works with the front axis.
        float minSpeedForDirection = 1.0f;  // m/s, "is it moving fast enough for a position delta to mean anything"
        // --- standstill re-seed (2026-10-03) ----------------------------------
        // Below this speed the machine counts as standing. A standstill lasting
        // standstillMinMs arms the re-seed: the next evaluation of the direction decision
        // starts from the CURRENT measurement instead of ramping the 0.2 EMA up, which is
        // what a short slow K-turn needs (see the note at standstillSinceMs in the private
        // section). 0.15 m/s is well under a walking pace, so normal driving never arms it.
        float    standstillSpeedMps = 0.15f;
        uint32_t standstillMinMs = 300;
        // Distance-based travel direction, ported from AgOpenGPS
        // (Position.designer.cs:738-754: the travelled distance since the last check
        // must exceed `dualReverseDetectionDistance`, 0.25 m). AOG hard codes that
        // value because it assumes RTK; measured 2026-09-23 (hall logs) the COG
        // version of this test inverted the direction for ~6...12 s on every hall
        // entry, because the VTG course is unreliable at walking pace under a roof.
        // The position delta is the better source - but only if it is long enough to
        // stand out of the POSITION NOISE:
        //   required distance = max(dirMinDistanceM, dirSigmaFactor * sigma_pos),
        // with sigma_pos taken from the receiver's own standard deviations when it
        // reports them (INSPVA/INSPVAXA) and from the fix type otherwise. That keeps
        // a 0.25 m step meaningful for RTK fixed and demands metres while the
        // receiver is on a float/degraded solution.
        float dirMinDistanceM = 0.25f;
        // ... but it MUST stay short enough to catch tight turns: an operator note
        // (2026-09-23) says the second hall log contains turning manoeuvres in a
        // confined space, and 0.5 m is the most that may be demanded before the
        // direction decision is allowed to react. So the sigma term is capped:
        //   need = clamp(5 * sigma, 0.25 m, 0.5 m)
        // RTK fixed -> 0.25 m (AOG's own value), RTK float -> 0.5 m (the cap), and a
        // degraded fix cannot push it further - there the COG cross-check and the
        // "changing direction" pause carry the decision instead.
        float dirSigmaFactor = 5.0f;
        float dirMaxDistanceM = 0.5f;
        // A position delta is only a direction if it also matches the SPEED: in a hall
        // the fix jumps (multipath) and then the delta points anywhere - measured
        // 2026-09-24 in the hall log: 22 frames declared REVERSE while the machine
        // turned forward in a half circle, because the raw fix was jumping.
        // The check is |distance - v*dt| <= dirSpeedTolFrac*v*dt + dirSpeedTolM.
        float dirSpeedTolFrac = 0.5f;
        float dirSpeedTolM = 0.4f;
        // One bad evaluation must not flip the direction: the decision has to repeat.
        uint8_t dirVotesNeeded = 2;
        // Cross-check against the COG: they measure the same thing, so a large
        // disagreement means one of them is lying (multipath, spike). The decision is
        // then HELD instead of being made on rubbish (AOG's "goto byPass").
        float dirCogCrossCheckDeg = 45.0f;
        float reverseDetectDeg = 90.0f;     // filtered deviation above this = reverse
        float directionChangeDeg = 28.6f;   // |filtered - raw| above this = changing (AOG 0.5 rad)
        float directionFilterAlpha = 0.2f;  // AOG's 0.2/0.8 EMA
        // --- Operator correction 2026-09-29 ("AOG zeigt Rückwärts, obwohl ich vorwärts
        // fuhr", "die Richtung stimmt nur kurz nicht, wenn der Empfang zurückkommt") ----
        // Two hardware-independent guards, because ONE bogus evaluation was enough to
        // mirror everything for 50 s in the 06:31 log (rev 0->1 at 0.47 m/s, 0.25 m
        // baseline, straight afterwards the filter swung up to 124 deg away from a
        // TRUSTED dual heading):
        //   * the raw DUAL ANTENNA is used as an independent witness - it is body
        //     referenced and is never mirrored by the firmware, so it cannot confirm its
        //     own mirroring. REVERSE is only accepted if it agrees with the decision;
        //   * clearing back to FORWARD is allowed on its own (an anti-lock exit), so a
        //     wrong reverse can never hold itself in place through the mirrored inputs.
        float reverseConfirmDeg = 120.0f;   // attitude <-> travel must agree this closely
        float reverseClearDeg   = 60.0f;    // ... to clear reverse immediately
        // If the fused heading sits more than this far from a TRUSTED attitude, the
        // STATE is wrong (not the measurement): re-seed it instead of integrating the
        // 180 deg back around. Measured 2026-09-29: psi walked 124 deg away from a
        // steady, qf=4 antenna heading within 1.2 s, and AOG latched its own reverse
        // detection while the module published a heading that pointed backwards.
        // 2026-10-06: 30 -> 45. Die 30 feuerte bei einer normalen engen Kurve, weil das Modell
        // nachhinkte (Log 20-38: +31,1 und +31,5 Grad hart, zweimal in 2 s, bei perfektem
        // Empfaenger) - der ZUSTAND war aber nicht falsch, das Modell war zu kurz. Mit
        // effWheelbaseFac = 1.0 verschwindet das Nachhinken; 45 laesst trotzdem keinen echten
        // Zustandsfehler stehen (180-Grad-Klasse) und gibt Luft fuer enge Kurven.
        float reseedBeyondDeg = 45.0f;
        // B (2026-10-07): der Re-Seed setzt psi bisher HART auf die Messung (und umgeht den
        // Ratenbegrenzer). Gemessen: 82 und 94 Grad in EINEM Frame -> Betreiber-Symptom
        // "Traktor steht schraeg zur Spur". Statt dessen ein begrenzter Schritt je Tick
        // (10 Hz: 6 Grad = 60 Grad/s -> 31 Grad in 0,5 s, 94 Grad in 1,6 s).
        float reseedRampDegPerTick = 6.0f;
        // ---- Single-antenna / IMU-anchored heading (operator decision 2026-09-29) -----
        // "Wir lassen die AOG-Schematik" (PANDA + AOG's own fusion), but the VWAS needs a
        // good heading, and without a dual antenna the bridge must stay usable: the IMU
        // heading is the fast signal, the GPS path/COG are the slow absolute anchor - the
        // same construct AgOpenGPS uses (imuGPS_Offset learned against the GPS heading,
        // Position.designer.cs:439). The offset may therefore also be learned while a
        // PATH/COG measurement is accepted, not only while a dual attitude is trusted.
        uint32_t imuAnchorHoldMs = 400;      // a path/COG measurement counts as anchor this long
        float    imuAnchorMinSpeedMps = 1.0f;// below this neither the path nor the COG says much
        // The reversal witness: the dual antenna when there is one, otherwise the
        // direction of the NET DISPLACEMENT from the positions (>= travelWitnessMinM over a
        // straight window). The position witness needs no heading source at all and cannot
        // be mirrored, so it also protects a single-antenna machine.
        float    travelWitnessMinM = 1.0f;   // displacement for one witness decision
        float    travelWitnessMaxTurnDeg = 30.0f;  // window counts as straight below this
        float    travelWitnessSpeedMps = 1.0f;     // and only while actually moving
        // An IMU-heading measurement may re-seed the state only when the position witness
        // agrees with it (a wrong IMU OFFSET must not be able to pull the heading).
        float    reseedImuConfirmDeg = 45.0f;
        // A reversal must be re-confirmed by a SOLID travel measurement within this long
        // while such a measurement exists. Measured on the 09-29 v-r log: one bogus
        // evaluation at 0.47 m/s (0.25 m baseline) held REVERSE for 50 s. While the machine
        // crawls there is no solid measurement, so the decision is simply kept - a genuine
        // slow reverse manoeuvre is never cut short.
        uint32_t revReconfirmMs = 5000;
        // The state machine must not chatter when the fall-back sources flicker
        // (measured: BRIDGE/COAST <-> DEADRECKON every 100 ms while slow under a roof,
        // because cogValid/pathValid dip in and out around their speed limits).
        uint32_t fallbackHoldMs = 300;      // keep the last state this long
        // Learning gates. A machine constant may only be learned from clean data:
        // enough satellites, a trusted attitude throughout the window, no direction
        // change, and a bounded innovation (a hall exit jumped psi by 244 deg once -
        // that single event would have written a bogus scale into the EEPROM).
        // Careful with the satellite count (operator note 2026-09-23): NMEA GGA/GNS
        // caps the field at 12, an F9P never reports more there (more only in UBX),
        // and a healthy fix delivers 9...12 - so the count is the WEAKEST quality
        // measure we have. It is kept only as a floor that catches a total loss, while
        // the real quality test for learning is the attitude itself:
        //   * fixQuality >= 4 (RTK fixed position),
        //   * the solved baseline length inside baselineTolPct (a drifting baseline is
        //     the classic dual-antenna sign of multipath/obstruction),
        //   * the ADAPTIVE attitude measurement noise below maxAttRForLearn - that is
        //     the filter's own live estimate of how much the attitude scatters, i.e.
        //     exactly the quantity a scale must not be learned from.
        uint8_t  minSatsForLearn = 6;
        float    maxAttRForLearn = 3.0f;    // deg, effectiveMeasurementR(0)
        float    maxLearnInnovDeg = 30.0f;
        // Plausibility window for applying the learned scale (see
        // applyLearnedSteerScale): outside this range the value is not a scale error
        // but a broken measurement, and it is only reported, never applied.
        float    steerScaleApplyMin = 0.80f;
        float    steerScaleApplyMax = 1.30f;

        // ---- Attitude plausibility gates (all measured) -----------------------
        // QF whitelist, same idea as WheelAngleFusion::headingQualityMask - but
        // NOT sufficient on its own, because the garbage frames carry QF 4/5.
        // --- Trim aus dem VWAS-Beobachter (2026-10-11, B7.21) --------------------
    // Der Nullpunkt gehoert dem WAS/VWAS (Betreiber-Entscheidung 09-27, B8.39) -
    // der VWAS setzt ihn aber NUR, wenn keine Guidance laeuft (ZO_BLOCKED, B7.15).
    // Waehrend Autosteer ist der gemeldete Lenkwinkel deshalb um den aktuellen
    // Fehler verschoben (gemessen: 10...25 Grad in der Akquise, 5,5 Grad in 7-01).
    // Der Beobachter KENNT den Fehler (`residualDeg`) - die Bridge darf ihn fuer ihr
    // MODELL benutzen: sie schreibt nichts zurueck, speichert nichts, gibt nichts an
    // AOG oder das VWAS. Vorzeichen (aus WheelAngleFusion.cpp:1348 + Anwendung):
    //     res = est - offset,  Nullpunkt wirkt als raw - offset
    //     => wirksamer Winkel = delta - res
    // Angewandt wird er ratenbegrenzt (kein Sprung) und nur, wenn die Beobachter-
    // Bedingungen erfuellt sind (der eigene Gate des Beobachters: >= 0.8 m/s).
    bool  vwasTrimInModel      = true;   // auf false = reine Beobachtung (PASSIV)
    float vwasTrimMaxAbsDeg    = 30.0f;  // darueber ist der Beobachter selbst Unsinn
    float vwasTrimMaxRateDegPerS = 5.0f; // Nachfuehr-Rate des angewandten Trims
    float vwasTrimMinSpeedMps  = 0.8f;   // = ZO_MIN_SPEED_MPS des Beobachters
    // -------------------------------------------------------------------------
    uint16_t headingQualityMask = (1u << 1) | (1u << 2) | (1u << 4) |
                                      (1u << 5) | (1u << 8) | (1u << 9);
        // A tractor does not roll 25 deg. |roll| p99 was 36.5 deg in the road
        // data set where the body was demonstrably level, so the envelope catches
        // the receiver's attitude blow-ups without touching real side slopes.
        //
        // DECOUPLED from the HEADING on 2026-09-30 (operator request "P1a"): the
        // roll is an attitude OUTPUT. The heading of the same sentence is a separate
        // solution, so the roll must not condemn it by itself. The 2026-09-30 tree
        // log has 30 roll-envelope runs: 13 with a heading that matched the VTG
        // course within 0.3...9 deg (one of them 76 frames with |roll| 66 deg,
        // 26 satellites, deviation 0.4 deg - a good attitude thrown away) and 17
        // that were 12...135 deg off. Therefore: out of envelope -> the heading is
        // kept IF an independent source confirms it (rollOutlierMaxDevDeg /
        // rollOutlierRefSpeedMps below), otherwise rejected as before. The roll
        // itself is never published from such a frame (updateRoll holds it - its
        // own condition already tests |dualRoll| <= maxRollAbsDeg).
        float maxRollAbsDeg = 25.0f;
        // How far the vehicle heading (baseline azimuth + dualHeadingOffsetDeg) may
        // differ from the VTG course before a frame with an out-of-envelope roll is
        // rejected. The two describe BODY and PATH, so the side slip sits in here:
        // measured p90 4.5 deg on the road, up to ~10 deg in soft ground.
        float rollOutlierMaxDevDeg = 15.0f;
        // Below this speed the VTG course is no reference (see minSpeedForCogUse).
        float rollOutlierRefSpeedMps = 2.0f;
        // No real vehicle changes its roll by 8 deg within 100 ms at these
        // speeds. Measured roll steps: p95 5 deg, p99 19.6 deg, max 58.9 deg.
        //
        // Also speed proportional, and that is OBSERVED, not assumed: the roll step
        // p99 was 19.6 deg on the 10 m/s road data against 5 deg p95 on the 2.5 m/s
        // field data - same machine, same kind of bumpy ground, the excitation
        // frequency scales with speed. A quadratic term is wrong again: the roll
        // RATE from a given bump scales with v, not with v^2 (the vertical
        // acceleration scales with v^2, the angle does not).
        float maxRollStepDegPer100ms = 8.0f;
        float rollStepPerSpeedDegPer100ms = 2.0f;   // deg per m/s, per 100 ms
        // The physical heading-step limit is omega_max = v*tan(delta_max)/L_eff,
        // hence a fixed part plus a SPEED PROPORTIONAL part. With delta_max ~28 deg
        // and L = 2.5 m that is 12 deg/s per m/s, i.e. the constant below - a
        // quadratic term would be wrong: the vehicle can turn faster, not
        // quadratically faster, the faster it goes. What does NOT scale with speed
        // is the measurement noise, so the gate also has to stay open by a few
        // sigma of the CURRENT measurement noise (see headingStepNoiseSigma).
        float maxHeadingStepDeg = 5.0f;
        float headingStepPerSpeedDegS = 12.0f;   // deg per (m/s * 0.1 s)
        float headingStepNoiseSigma = 3.0f;      // x effectiveMeasurementR(0)
        // Cross-check against the independent course over ground. Measured
        // |attitude heading - COG| on the road: mean 1.3 deg, p90 4.5 deg.
        float cogCrossCheckDeg = 25.0f;
        float minSpeedForCogCheck = 1.5f;        // m/s
        // Course over ground is only a fall-back above walking pace: measured in the
        // hall logs (2026-09-23) the VTG course was inverted by ~180 deg at 0.5...1.5
        // m/s under a roof, which then flipped the direction detection. Below this
        // the estimate is HELD - dead reckoning for a short while is better than a
        // measurement that is known to be wrong.
        float minSpeedForCogUse = 1.0f;          // m/s
        // ---- Course over ground (VTG) as a HEADING measurement (P4.3, 2026-09-30) ----
        // The COG is the velocity direction; its direction error grows roughly as 1/v
        // (measured on the 2026-09-30 logs, |cog - position chord| p50/p90:
        //   0.3...1.0 m/s 4.0/9.0   1.0...1.5  4.4/10.4   1.5...2.0  3.5/10.3
        //   2...3 m/s     3.2/11.1  3...5      1.3/6.4     >5 m/s    0.6/2.5 deg).
        // Below cogRefSpeedMps the measurement noise is therefore multiplied by
        // cogRefSpeedMps/v, capped by cogMaxNoiseFactor: the source stays "cog" (no
        // dead-reckoning penalty, it still feeds the slow side-slip estimate) but its
        // pull on psi shrinks - the jitter the operator saw after the bridge came
        // exactly from COG-driven ticks at 1.1...1.3 m/s.
        // 2026-09-30 (second round): the SHAPE analysis (tools/heading_bridge/
        // spike_shape.ps1 + az_peaks_by_src.ps1) found that in the slow phases the
        // single-frame peaks came mostly from THIS fall-back: `cog` drove 10 of 10
        // peaks in the start log (v p50 1.2 m/s, up to 5.4 deg per 100 ms) and 12 of
        // 22 under the trees. cogRefSpeedMps is therefore 3.0, not 2.0: at 1.5 m/s the
        // factor is now 2.0 (gain /4) instead of 1.33 (gain /1.8).
        float cogRefSpeedMps = 3.0f;
        float cogMaxNoiseFactor = 4.0f;

        // ---- P2: physical RATE LIMIT for the measurement correction --------------
        // Operator decision 2026-09-30 ("beides einbauen"), and the operator's own
        // physical argument: a slow machine with a small wheel angle CANNOT change its
        // heading fast, so a large step cannot be real - the old value must be
        // weighted heavily. Implemented as a bound on the rate at which a measurement
        // may pull psi (NOT as an average - see below):
        //     allowed = v * tan(|wheel| + rateLimitWheelSlackDeg) / L
        //               + rateLimitMarginDegS                                [deg/s]
        // raised to  rateLimitPathFactor * |path turn rate| + margin  while an ACCEPTED
        // fix-to-fix window shows the machine IS turning (otherwise a turn START is
        // delayed: the wheel is still straight while the machine already turns).
        // The applied correction is clipped to allowed*dt.
        //
        // Why a limit and not a moving average (measured, Temp\filter_compare.ps1,
        // TWITCH = frames per 1000 straight frames with a > 1.5 deg step per 100 ms):
        //   Bäume   base 58.4 | average 121.8 | wheel-weighted average 125.5 | limit 16.1
        //   Halle   base 44.0 | average  55.0 | wheel-weighted average  38.5 | limit 13.8
        //   Start   base 108.2| average  40.9 | wheel-weighted average  43.9 | limit 11.7
        // Averaging is WORSE than doing nothing: it lags, and the lag appears as a big
        // step at every curve-to-straight transition. A rate limit clips exactly the
        // peak - and the peaks are SINGLE FRAME events (spike_shape.ps1: run length 1
        // in 21 of 22 peaks of az_out), so clipping one frame ends the event.
        // A 10...15 % error in cpd/max steer angle is harmless here: only the MAGNITUDE
        // of the wheel angle enters, plus a slack of rateLimitWheelSlackDeg.
        bool  rateLimitEnabled = true;
        float rateLimitMarginDegS = 2.0f;      // allowed on top of the kinematics
        float rateLimitWheelSlackDeg = 2.0f;   // the wheel angle is not exact
        float rateLimitPathFactor = 1.25f;     // weight of an ACCEPTED path turn
        // R4 (2026-10-02): the receiver's OWN measured yaw rate. It is a MEASUREMENT of
        // what the machine really does and it is the only bound that is right in a tight
        // manoeuvre: v*tan(wheel)/L ignores the Ackermann offset atan(L/R), so in a K-turn
        // at 1 m/s the machine turns far faster than the formula says. Measured on
        // 2026-10-02 16-27 ("drives diagonally after the K-turn"): |psi - attitude| reached
        // p50 23.5 deg / max 61.7 deg DURING the reverse phase although qf was 4 and the
        // attitude was the active source - psi could not follow the truth because the
        // limiter was TIGHTER than the truth, and the heading AgOpenGPS got then no longer
        // matched the vehicle's front axis.
        // rateLimitAttFactor = how much a measurement counts next to the model (1.25: the
        // measurement wins once it is >25 % higher, which covers the normal 1...2 deg
        // measurement noise without letting a single spike through),
        // rateLimitAttMinDegS = noise floor, below it the measurement is ignored.
        // ONLY a TRUSTED attitude frame may open the bound. With qf 0/5 (the 2026-10-01
        // 20-15 case) this term is zero and the behaviour is exactly as before.
        float rateLimitAttFactor = 1.25f;
        float rateLimitAttMinDegS = 1.0f;
        // R8 (2026-10-02): the kinematic model must not work AGAINST a trusted attitude.
        // Measured on 2026-10-02 16-27 ("the tractor drives diagonally after the K-turn"):
        // the model rate pointed opposite to the machine's real rotation (+18.0 deg/s while
        // the attitude rotated -25 deg/s). Both stayed inside their own rate limit (gain
        // 0.09, rl_clip 1), but their SUM left psi at -0.3 deg per 100 ms instead of
        // -25 deg/s, so |psi - attitude| grew to 23...62 deg and the body heading
        // AgOpenGPS received stopped matching the vehicle's front axis.
        // Root cause: the model has no direction sign (updateProjection uses dirSign, the
        // prediction does not) and the reverse flag is DELIBERATELY not set inside a turning
        // manoeuvre - the position witness needs a straight 1 m window at >= 1 m/s
        // (Config::travelWitnessMaxTurnDeg). So the flag cannot be the source; the attitude
        // is the only per-epoch witness that is never mirrored.
        // modelAgainstAttMinInnovDeg separates the real case from the innovation noise of
        // normal driving. Measured on the 2026-10-02 logs (tools/analysis/r8_preview.ps1):
        // at 3 deg the guard also fired in normal driving (16-30: 6 straight frames,
        // 16-23: 28) - too broad. At 10 deg it fires 0 times in 16-30 (the log that "runs
        // well"), 2 times in 16-23, and 53 times in the K-turn of 16-27 where the opposed
        // model is exactly the defect (17...31 deg innovation). In the 2026-10-01 evening
        // logs it fires often, and that is CORRECT: there the model was the error source
        // (bias runaway / IMU rate), so letting the measurement win is the point of R1+R8.
        bool  modelAgainstAttGuard = true;
        float modelAgainstAttMinInnovDeg = 10.0f;
        // While the wheel angle is untrustworthy (quality < wheelQualityLowPct) the
        // kinematics must not be trusted either - a generous fallback keeps the heading
        // alive while the VWAS learns its zero (the path bound still applies).
        float rateLimitWheelFallbackDeg = 15.0f;
        // PERSISTENCE (safety against a genuinely wrong STATE): if the measurement keeps
        // asking for more than the bound in the SAME direction for this long, the bound
        // is no longer describing noise - it is describing a state that must catch up
        // (a long dead-reckoning phase, a wrong seed). It is then relaxed by
        // rateLimitPersistRelax, so a 20 deg error converges in ~2 s instead of ~8 s.
        // Measured (Temp\filter_compare.ps1): 0.4 s is TOO SHORT (it lets the 3...5 frame
        // receiver bursts through, TWITCH 112 instead of 16), 1 s keeps them out.
        uint32_t rateLimitPersistMs = 1000;
        float    rateLimitPersistRelax = 4.0f;
        // ---- F4 (2026-10-01): break the "stuck state" deadlock -------------------
        // Measured in the 13-25 log: 60 s of DEAD-RECKONING while a valid dual attitude
        // was present (qf 4, 20 sats) - the wrongly latched REVERSE made the cogX gate
        // reject every one of those frames, so nothing could pull the 20...34 deg error
        // back and psi drifted at 1.5 deg/s. If NO measurement has been accepted for
        // this long while the receiver's own quality says the frame is good, the STATE
        // is the suspicious one: the frame is accepted with its R multiplied by
        // stuckRescueRFactor, so the estimate is pulled in gently (about 1 s).
        uint32_t stuckRescueAfterMs = 3000;
        float    stuckRescueRFactor = 3.0f;
        uint32_t maxCogAgeMs = 500;
    // Use the forward projection as a heading measurement even while the attitude is trusted
    // (2026-10-05, "Step A" of knowledge/heading_recovery_concept_2026-10-04.md). Measured on the
    // 2026-10-04 13-53 log: on the frames where |travel - psi| = 9.40 deg the projection's own
    // residual says 0.70 deg - it SEES the error, but the old rule
    // (allowMeasurement = !status.attitudeTrusted) admitted it only while the attitude was LOST,
    // so it was never used (src=proj 0 % of 1294 frames). Set to false for the old behaviour.
    bool projAlwaysMeasure = true;
        // Same idea for the position derived direction: the attitude heading
        // must not contradict the direction the machine actually travelled.
        float fixToFixCrossCheckDeg = 30.0f;
        float minDisplacementForFixToFixM = 0.5f;
        uint32_t maxPositionAgeMs = 500;
        // ---- Position quality gates for the fix-to-fix window ----------------
        // A fix-to-fix heading inherits every position fault, so the position
        // stream is checked before anything is derived from it.
        // Step test: displacement since the previous fix must be <=
        // maxPosStepMargin * v * dt + maxPosStepNoiseM.
        float maxPosStepMargin = 2.0f;
        float maxPosStepNoiseM = 0.15f;
        // HDOP envelope; 0 disables the test.
        float maxHdopForPath = 3.0f;
        // Chord/arc ratio of the window. 0.9997 for 0.5 m on a 6 m radius turn,
        // so a small margin is enough to catch spikes and noise-dominated windows.
        float minPathStraightness = 0.90f;
    // Step C (2026-10-05): take a fix-to-fix measurement from the position RING at every fix,
    // over a constant exact baseline, in addition to the forward projection. The old window
    // (updatePositionWindow) re-bases after every accepted window, so a measurement exists only
    // near the end of each window and its baseline varies from 0 to ~0.6 m. Measured 2026-10-04:
    // proj_upd 6.3/s against fix_cnt 10.0/s, and the 9 deg skew lives in the phases where the
    // attitude is trusted - i.e. exactly where nothing path-anchored was admitted.
    // Set to false to restore the old behaviour.
    bool ringPathMeasurement = true;
// --- B-Kette (2026-10-13, B10.34/B10.35): der RINGPUFFER-KURS als Traeger ---------
// Warum: in Zustand B (Lage nicht vertrauenswuerdig, Positionen gut) war die Quelle
// `travelWitness` - eine Netto-Verschiebung ueber >= 1 m, die nur gilt, wenn das
// Fenster GERADE war (travelWitnessMaxTurnDeg). In der Kurve wird sie ungueltig, der
// Ausgang wird GEHALTEN und springt beim Wiedereinsetzen (12-38: Jitter p99 83 Grad/s).
// Der Ringpuffer-Kurs existiert auch in der Kurve (kein Gerade-Test) und wird um den
// Sehnen-Nachlauf 0,5*omega*T korrigiert (GRUNDZIELE 2.1); die Gierrate liefert das
// Modell (Lenkwinkel trim-korrigiert, v, L - dieselbe Zahl wie status.yawRateDegS).
// Gemessen 12-38/12-30 (offline, gleiche Methode): Jitter p99 83 -> 22 Grad/s, Rate auf
// Geraden p99 64 -> 16 Grad/s, Band p50 1,58 -> 0,79 Grad; in Zustand D 175 -> 10 Grad.
// Gemessen wird er NUR als Kurs (kein Side-Slip-Abzug): als Fahrtrichung ist er die
// Gegenprobe zur Koerperrichtung, genau wie der Witness es war.
bool     pathChordEnable = true;
float    pathChordMinM = 2.0f;      // Bogenlaenge (kurz = aktuell; gemessen besser als 5 m)
// B10.53 (2026-10-25): pathChordMaxAgeMs ist NUR NOCH ein Sicherheitsdeckel gegen
// "Minuten zurueckgegriffen". Vorher standen hier 3000 ms - und weil findChordSample nach
// DISTANZ zuruecklaeuft, war das in Wahrheit eine TEMPOGRENZE (Spanne B/v <= 3 s =>
// v >= B/3 s, bei B = 2 m also 2,4 km/h). Gemessen 10-25: damit fehlte die Sehne in 55,8 %
// der Frames von 8-08 - nicht wegen der Qualitaet, sondern wegen der Geschwindigkeit.
// Die Veraltungsfrage beantwortet jetzt die Frischepruefung am NEUESTEN Fix
// (kNewestFixMaxAgeMs in HeadingBridge.cpp), die Laenge begrenzt der Halbssehnen-Test.
// Historie: 1500 ms blockten 10-13 bereits 86 % der B-Frames.
uint32_t pathChordMaxAgeMs = 8000u; // aelter darf die Sehne nicht sein (wie der Ring) // aelter darf die Sehne nicht sein (wie der Ring;
                                     // 1500 ms verlangten >=1,33 m/s fuer eine 2-m-Sehne
                                     // und blockten 86 % der B-Frames, gemessen 10-13)
bool     pathChordLagCorr = true;   // 0,5*omega*T gegen den Sehnen-Nachlauf
    // ADAPTIVE Bogenlaenge (Betreiber-Entscheidung 10-13: "adaptiv"). Die KURZE Sehne ist
    // der Standard, weil in der Kurve der NACHLAUF begrenzt und nicht das Rauschen (B10.34:
    // 2 m schlug 5 m in B UND in A). Die lange Sehne wird nur genommen, wenn der eigene Weg
    // GERADE ist (arc/chord >= pathChordStraightRatio) UND die lange die kurze Richtung
    // bestaetigt (|delta| <= pathChordDisagreeDeg) - sie hat weniger Rauschen (2*sigma/B).
    float    pathChordLongM = 2.0f;          // die lange Sehne (nur auf der Geraden)
    // GEMESSEN 10-13 (Sim-Replay 12-38, identische Frames, sonst unveraendert): adaptiv mit
    // lang=5 m brachte in Zustand B KEINEN Gewinn und war dort UND in D leicht schlechter -
    // Rate-Geraden p99 50 statt 38 Grad/s, D-Band p50 3,05 statt 2,32 Grad, B-Band
    // 3,10 statt 3,14 (unentschieden). Deshalb steht die lange Sehne per VORGABE AUS
    // (lang == min, der Zweig wird uebersprungen); zum Einschalten pathChordLongM = 5.0f.
    // Die kurze Sehne gewinnt bei diesen Geschwindigkeiten, weil der NACHLAUF begrenzt
    // (B10.34) - die adaptive Logik selbst ist eingebaut und geprueft (pc_len im Log).
    float    pathChordStraightRatio = 0.98f; // arc/chord der eigenen Fixes fuer "gerade"
    float    pathChordDisagreeDeg = 10.0f;   // so weit darf die lange von der kurzen abweichen
    float    pathChordSpeedTol = 1.0f;       // Sehne <= v*T*(1+tol) + maxPosStepNoiseM
    // --- Schleichfahrt (2026-10-15, Betreiber: "Schleichfahrt fix") ------------------
    // Unter 1 m/s gibt es keinen frischen Positionskurs (Sehne und Witness haben beide die
    // 1,0-m/s-Schwelle; gemessen 10-14: in 18-59 war pc_ok=1 genau dann, wenn v >= 1,0,
    // 49 % der B-Frames hielten deshalb). Ein eingefrorenes Heading springt, sobald die
    // Sehne wiederkommt. Deshalb laeuft die Ausgabe im HALTEN-Fall mit der MODELLRATE
    // weiter - aber nur, wenn sie vertrauenswuerdig ist und LANGSAM ist (sonst ist sie
    // Rauschen): Betrag <= holdRateMaxDegS, kein yawRateZeroed, nicht rueckwaerts (dort
    // rechnet das Modell mit |v| und haette das falsche Vorzeichen).
    float    holdRateMaxDegS = 3.0f;
    // --- 2026-10-18: Quellen-Bestaetigung (Heading) und Roll-Reinigung --------------
    // Betreiber: "1 s Uebergang zwischen den Quellen ist ok, aber zuerst sicher sein,
    // dass die neuen Daten stimmen - die ersten Werte nach Rueckkehr vom Empfang sind
    // meist nicht gut." Und: der Haltewert des Rolls darf die letzten Frames vor einem
    // Ausfall nicht enthalten (die sind schon verdorben).
    uint32_t sourceConfirmMs = 1000u;    // so lange muss ein Quellenwechsel bestaetigt sein
    uint32_t rollConfirmMs = 300u;       // so alt muss ein Roll-Sample sein (Haltewert). 10-18 gemessen: 1 s macht den Haltewert nur STAERKER veraltet (4,50 statt 3,96 Grad Abweichung) - 0,3 s ist der bessere Kompromiss.
    float    rollRateLimitDegPerS = 10.0f; // = 1,0 Grad/100 ms (Holperstrecke max 0,89)
    // --- Gleitender Durchschnitt auf der Heading-QUELLE (2026-10-18, Vergleichsreihe) ---
    // Der Betreiber will den MA gegen das EMA/Qualitaetstor messen: ein Boxcar ueber
    // outMaMs (0 = AUS) vor der bestehenden Kette. Ein Boxcar der Breite T laesst die
    // Werte um T/2 nachlaufen - das wird mit der Modellrate zurueckgerechnet
    // (outMaLead), genau wie der EMA-Vorhalt. Das Mittel wird zirkulaer gebildet
    // (sin/cos), sonst springt es am 0/360-Uebergang.
    uint32_t outMaMs = 300u;   // 10-18 gemessen: 300 ms + Lenkwinkel-Daempfung ist der beste Kompromiss (0 = AUS)
    bool     outMaLead = true;
    // 2026-10-18 (Betreiber: "MA variabel mit Lenkwinkel als Daempfer"): das Fenster
    // wird mit dem EINSCHLAG ausgeblendet - bei Geradeauslauf voll (outMaMs), ab
    // outMaWheelHiDeg gar nicht mehr. Dazwischen linear. So bleibt die Ruhe auf der
    // Geraden (wo die Unruhe sichtbar ist) und die Kurve bekommt KEINEN Nachlauf.
    // Der Winkel ist der wirksame (trim-korrigiert), wie im Modell.
    float    outMaWheelLoDeg = 3.0f;   // bis hier: volles Fenster
    float    outMaWheelHiDeg = 10.0f;  // ab hier: aus
    // --- Geschwindigkeit aus WEG/ZEIT (2026-10-14, Betreiber-Auftrag) ---------------
    // Warum: das Feld-Log 18-47 (die Sitzung mit der "nicht gefahrenen" 90-Grad-Kurve)
    // hatte eine Empfaenger-Geschwindigkeit, die bis 2,94 m/s von Weg/Zeit abwich
    // (Verhaeltnis 1,45); in 18-53/18-59 war sie genau (0,99/1,00). Ein falsches v
    // trifft ALLE Schwellen: Schritt-Tor (maxPosStepAllowedM), Sehnen-Endpunkt und
    // -Alter, Modellrate (57,3*v*tan(delta)/L) und die Richtungstore.
    // Messung 10-14, Fehler p50 gegen Weg/Zeit ueber 2 s:
    //     Empfaenger 0,02...2,94 m/s  |  Weg/Zeit (1 s) 0,01...0,41 m/s
    // Weg/Zeit ist also NIE schlechter, bei kaputtem Doppler um Groessenordnungen
    // besser; es ist nur rauer (Schleichfahrt: Rausch p50 0,57 statt 0,37) und braucht
    // daher eine Glaettung.
    // Regel (Betreiber 10-14: "nach aussen roh bei gutem Empfang, bei schlechtem
    // glaetten"): INNEN rechnet die Bridge mit vFused (Weg/Zeit + EMA), NACH AUSSEN
    // (PAOGI Feld 11) bleibt der Empfaengerwert ROH, solange er zu vFused passt - nur
    // im Zweifel geht der geglaettete Wert raus. AOG hat einen eigenen Filter.
    bool     speedFuseEnabled = true;
    float    speedFuseWindowS = 1.0f;    // Weg/Zeit ueber dieses Fenster (s)
    float    speedFuseEmaS = 0.5f;       // Glaettung danach (0 = aus)
    float    speedFuseDoubtMps = 0.3f;   // |v_empf - vFused| darueber = Zweifel
    uint32_t speedFuseDoubtMs = 1000u;   // ... und das mindestens so lange
    float    speedFuseMinDistM = 0.10f;  // darunter gilt Weg/Zeit als 0 (Stillstand)
    // Relative position noise (1 sigma, m) for a POSITION-DERIVED direction (2026-10-05).
    // Operator rule: "ohne rtk muss das genauso gut sein - die fix-to-fix Punkte liegen in
    // RELATIVER Lage gleich, nur die absolute Lage ist ohne rtk falsch." Measured on the
    // 2026-10-04 logs as the perpendicular distance of the middle fix from the chord through its
    // neighbours, straight frames: p50 0.5 cm, RMS 2.0 ... 3.7 cm - and the SAME with RTK
    // (13-53: RMS 3.7 cm) as without (20-35: RMS 3.2 cm, absolute sigma 1.2 m).
    // A direction over a baseline b therefore carries ~ 2*posRelativeNoiseM/b of uncertainty
    // (4.5 deg at 0.5 m, 2.2 deg at 1.0 m) - never the metres of the absolute sigma.
    float posRelativeNoiseM = 0.02f;
        // Position noise (1 sigma) used to derive the fix-to-fix measurement noise
        // from the window length: sigma_dir = atan(sigma_pos / range), so a longer
        // window is automatically trusted more (and the 0.5 m minimum is what keeps
        // it below ~3 deg with RTK). An entered/learned value would be better, but
        // the position stream has no covariance in the NMEA path.
        float positionSigmaM = 0.03f;
        // COG and fix-to-fix must describe the same path within this.
        float cogPathDisagreeDeg = 30.0f;
        // Innovation allowance against the fused state: base + per second without
        // any measurement (after a long outage the state IS wrong and the
        // measurement must be allowed to pull it back).
        float pathInnovationBaseDeg = 60.0f;
        float pathInnovationPerS = 30.0f;

        // ---- Timing / budget -------------------------------------------------
        // Minimum gap between two attitude sentences before the frame counts as
        // missing. The value is only the FLOOR: the module measures the actual
        // frame interval (UM982 can run at 20 Hz, F9P typically at 10 Hz) and
        // uses 3 x measured interval, clamped to [hprGapMinMs, 600 ms].
        uint32_t hprGapMinMs = 150;       // == 3 frames at 20 Hz
        // Short bridge: attitude gone, but the position and the course over
        // ground are still valid -> the estimate is a real measurement of the
        // path, not dead reckoning. Operator requirement (2026-09-22): construct
        // values as long as possible, a complete drop-out is the bad case.
        uint32_t bridgeMaxMs = 2000;
        float bridgeMaxM = 15.0f;
        // Position-supported phase. While the position stream keeps producing a
        // trustworthy direction there is no reason to stop: the heading stays a
        // measurement (path direction), so the budget is effectively unlimited
        // and the module merely reports "degraded". Set to 0 to disable the limit
        // entirely, which is the default.
        uint32_t coastMaxMs = 0;          // 0 = no limit while the path is valid
        float coastMaxM = 0.0f;           // 0 = no limit
        // True dead reckoning (no attitude, no position, no COG). Here the only
        // sources left are wheel angle, speed and the bias state, so a limit is
        // mandatory: after it the module stops claiming a valid heading and the
        // configured TimeoutAction applies.
        uint32_t deadReckonMaxMs = 10000;
        float deadReckonMaxM = 50.0f;
        TimeoutAction timeoutAction = TimeoutAction::LAST_VALID_QF0;

        // ---- Baseline length plausibility (F9P manual quality test) ----------
        // Reference for the solved baseline length. 0 = learn it from the receiver
        // itself: while the attitude is plausible and the machine is moving, the
        // median of the reported length IS the mechanical length (a healthy F9P
        // solves it correctly), so the check can run without anyone measuring the
        // antenna spacing with a tape measure. A value entered here overrides the
        // learned one (use it when the learned value itself is wrong, e.g. because
        // the receiver was already degraded during the learning phase).
        //
        // Why not from vehicle.xml: AgOpenGPS does not store an antenna spacing.
        // Its dual-antenna settings hold only the heading offset
        // (setGPS_dualHeadingOffset) and the imported vehicle model has
        // AntennaHeight/AntennaPivot/AntennaOffset/Wheelbase/TrackWidth - no
        // baseline. The UM982 does not need the test at all (measured: its
        // attitude failure is an immediate 0/QF=0 jump, not a drift).
        // 0 keeps LEARNING; anything >= 0.1 m overrides the learned value. Entering the
        // measured value is strongly preferred: the learned reference is a running mean
        // over ONE 100-frame window with no revision path anywhere, and on 2026-10-03
        // that window ran while the receiver was degraded, so it froze at 0.852 m
        // (+18.3 % of the true 0.711 m) and the gate then rejected 99.7 % of the frames
        // the receiver itself called clean.
        // Persisted in the EEPROM (1600-1699, bytes 5..9), applied at boot in
        // HeadingBridge::init(), changed at runtime with setPhysicalBaselineM().
        // Measured on this machine, 15 logs / 13 065 RTK-fixed frames: p50 = 0.711 m,
        // per-session medians 0.706 ... 0.719 m. 0.711 m is also the pooled optimum of
        // the window itself (87.8 % of RTK-fixed frames inside +-5 %, vs 87.7 % at
        // 0.710 and 86.8 % at 0.720).
        float physicalBaselineM = 0.0f;
        float baselineTolPct = 5.0f;      // reject outside +-5 % of the reference
        uint16_t baselineLearnFrames = 100;   // ~10 s at 10 Hz before judging

        // Machine constant shipped with the build: the antenna spacing measured on THIS
        // vehicle. Used when the EEPROM holds no value, so the test is active without
        // anyone entering anything. Measured over 15 logs / 13 065 RTK-fixed frames:
        // p50 0.711 m, per-session medians 0.706 ... 0.719 m, and 0.711 m is also the
        // pooled optimum of the +-5 % window (87.8 % of RTK-fixed frames inside it).
        static constexpr float physicalBaselineDefaultM = 0.711f;

        // SAFETY NET - the reason shipping a built-in default is acceptable at all. A wrong
        // reference (learned OR entered, e.g. this firmware in another vehicle) must never be
        // able to silence the heading. Over each window of baselineGuardWindow JUDGED frames
        // the test counts its own rejects; above baselineGuardRejectPct it declares the
        // REFERENCE wrong, disables itself and says so loudly.
        // Evidence it is needed: 2026-10-03 12-52, a learned 0.852 m (+18.3 % off) rejected
        // 99.7 % of the receiver-clean frames for the whole session.
        // This is deliberately NOT a re-learn: the wrong reference is never adopted. The test
        // is switched off and the behaviour returns to what it was before the test existed -
        // measured, a re-learn instead picks up the bad stretch and stays wrong (88 % loss).
        uint16_t baselineGuardWindow = 300;    // ~30 s at 10 Hz
        float    baselineGuardRejectPct = 90.0f;

        // ACTIVE REFERENCE CHECK (operator request 2026-10-11). The safety net above is
        // correct but LATE (300 frames ~ 30 s) and until then a wrong reference shows only
        // as silently rejected frames - a field log cannot tell that from bad reception.
        // This check measures what the receiver actually SOLVES - independent of every
        // reference, so it also runs while a FIXED reference is set (the learn path below
        // deliberately does not) - and compares it against the reference once
        // baselineSuggestFrames clean frames are in. A mismatch is REPORTED loudly and
        // flagged for the web page (Status::baselineRefSuspect) so the operator can correct
        // the value BEFORE the safety net mutes the test. The reference itself is NEVER
        // changed here: only the operator changes it (field entry, or "use measured
        // reference" on the bridge geometry page).
        // Evidence (2026-10-03 12-52): a reference of 0.852 m (+18.3 % of the true 0.711 m)
        // rejected 99.7 % of the receiver-clean frames for the whole session; the mismatch
        // was measurable after ~4 s, while the firmware stayed silent for ~30 s.
        uint16_t baselineSuggestFrames = 40;   // ~4 s at 10 Hz

        // ---- IMU yaw rate (operator request 2026-09-24) -----------------------
        // The kinematic model omega = v*tan(delta)/L is blind at creeping speed (at
        // 0.2 km/h it predicts ~nothing) and it inherits every error of wheel angle,
        // wheelbase and cpd. A gyro measures the rotation DIRECTLY - which is exactly
        // what a hall entry or a bridge needs.
        // What a gyro cannot give is an absolute heading: it drifts. So the bias is
        // learned whenever the attitude is trusted - then the filter's own heading
        // change IS the truth - which is the same trick AgOpenGPS uses with its
        // imuGPS_Offset (Position.designer.cs:439-469).
        // The MAGNETOMETER heading is deliberately NOT used: steel, engine and cables
        // make it useless on a tractor. Only roll/pitch would be usable from it, and
        // those are a gravity reference.
        // Works with both supported sensors (BNO085, TM171) because IMUProcessor
        // already applies the mounting/axis configuration that AgOpenGPS sends
        // (PGN 251 bit 3, ConfigManager::isUseYAxis).
        bool   useImuRate = true;           // ON by default (operator 2026-09-24); it
                                            // only acts while a valid IMU reports, and
                                            // an implausible bias disables it by itself
        float  imuRateMaxDegS = 90.0f;      // plausibility gate
        bool   imuRateInvert = false;       // sign; verify once, see the bias warning
        float  yawBiasAlpha = 0.005f;       // bias learning per 100 ms frame
        uint8_t imuRateMinQuality = 1;      // IMUData::quality at least this
        float  imuQHeadingDegS = 1.5f;      // process noise while the IMU carries it
        float  imuBiasWarnDegS = 5.0f;      // |bias| above this: sign is probably wrong

        // ---- IMU heading, offset-corrected (operator request 2026-09-24) ------
        // The TM171 delivers a very good heading (operator). AgOpenGPS uses an IMU
        // heading RELATIVELY, never as an absolute: it keeps imuGPS_Offset up to date
        // against the GNSS heading while that is good
        // (Position.designer.cs:439: imuGPS_Offset += ((imuHeading+offset)-gpsHeading)
        // * 0.0005) and then takes imuCorrected = imuHeading + imuGPS_Offset as
        // fixHeading. Nothing about the magnetometer is trusted - only the fact that
        // the sensor tracks rotation well once its offset is known.
        // Same here: the offset is learned while the dual-antenna attitude is trusted
        // (that is our reference), and afterwards the offset-corrected IMU heading is
        // a fall-back measurement whose noise grows with the time since the offset was
        // last refreshed (a servo/AHRS heading drifts).
        bool   useImuHeading = true;       // effective only while a valid IMU reports
        float  imuHdgOffsetAlpha = 0.0005f; // per 100 Hz tick - AOG's own value
        float  imuHeadingR0Deg = 4.0f;     // measurement noise right after a loss
        float  imuHeadingDriftDegPerS = 0.05f;  // noise growth per second blind
        float  imuHeadingMaxRdeg = 20.0f;  // beyond this it is not worth using
        uint32_t imuHeadingMaxAgeMs = 500; // sensor staleness gate
        uint8_t imuHeadingMinQuality = 1;

        // ---- Re-entry ramp ---------------------------------------------------

        // After a blind phase the first attitude frame would snap psi to the receiver
        // in ONE step (measured 2026-09-23: 244 deg in a single 100 ms frame at a hall
        // exit). The correction is wanted - the attitude is the truth - but not as a
        // step: the first frames are applied with an inflated measurement noise
        // (reentryRampGain x, decaying linearly to 1 over reentryRampMs), which
        // distributes the same correction over a few frames (10 Hz -> 5 frames in
        // 500 ms). AgOpenGPS then never sees a heading step.
        uint32_t reentryRampMs = 500;
        float    reentryRampGain = 30.0f;   // R multiplier at the start of the ramp
        // Before ANY of those frames is used, the receiver's attitude has to settle:
        // measured 2026-09-23/24 (hall logs), the first 0.5...2 s after a re-lock
        // carried roll values of 10...19 deg on a machine driving straight - the
        // baseline solution is not up yet, and those frames were accepted as "none"
        // by the roll envelope and pushed psi around. Operator request: 1 s.
        uint32_t reentrySettleMs = 1000;      // criteria must hold this long
        float    reentrySettleRollDeg = 8.0f;
        float    reentrySettleRollStepDeg = 3.0f;
        float    reentrySettleAttStepDeg = 5.0f;
        // Safety net: if the attitude never settles (heavy terrain), it must not stay
        // unused forever - after this the frames are used again with an inflated R.
        uint32_t reentrySettleTimeoutMs = 3000;
        // ---- Position fall-back (operator request 2026-09-24) -----------------
        // The forward projection already integrates the POSITION (not only the
        // heading). It is now fed to AgOpenGPS when the receiver's fix is unusable,
        // so a bridge or a tree does not throw the cross-track off:
        //   * the fix gate rejects a position that is physically impossible
        //     (ds > posSpikeMargin * v * dt + posSpikeNoiseM), a fix below RTK fixed
        //     (fixQuality < 4) or a stale one, and the projection keeps running
        //     instead of snapping to it,
        //   * while that lasts the projected position is published, bounded by
        //     posFallbackMaxMs/posFallbackMaxM.
        // Budget reasoning for the operator's "we work slowly" case (2026-09-24): the
        // TIME limit must not cut a creeping machine short - at 0.2 km/h (0.056 m/s)
        // seven seconds are 0.4 m of travel, which is useless under a tree. What
        // actually has to be bounded is the POSITION ERROR, and that is
        // travel * sin(heading sigma), i.e. the distance, not the time. So:
        //   * posFallbackMaxM (operator's 20 m) is the hard ceiling,
        //   * it is tightened by posFallbackMaxDriftM / sin(sigma_psi): a well-known
        //     heading allows a long extrapolation (sigma 0.5 deg -> 57 m, capped at
        //     20 m), a shaky one only a short one (5 deg -> 5.7 m),
        //   * the time limit only applies WHILE MOVING and is generous (60 s): at
        //     1 m/s the distance limit dominates anyway (20 m = 20 s), at creeping
        //     speed the distance is tiny and the time is irrelevant,
        //   * standing still (v < 0.1 m/s) the extrapolation does not move at all, so
        //     no time limit is applied - it holds the last good position.
        uint32_t posFallbackMaxMs = 120000;
        float    posFallbackMaxM = 40.0f;
        float    posFallbackMaxDriftM = 1.0f;   // allowed position error of the fake
        // RTK recovery (operator note 2026-09-24: "the time the GPS needs to recover is
        // the problem"). A bridge is only ~10 m, but the receiver needs seconds to get
        // its ambiguity fix back. As soon as fixQuality 4 is back AND has held for this
        // long, that position IS the truth - the projection is re-synchronised to it
        // even if it jumped away from our extrapolation. Without this the fall-back
        // would defend its own guess against a recovered fix until the budget ran out.
        uint32_t posRtkRecoverMs = 1000;
        // Position ramp (operator request 2026-09-24): when the fall-back hands the
        // position back (RTK recovered or budget spent) the extrapolated position is
        // blended onto the fix over this long instead of jumping - AgOpenGPS then sees
        // a continuous cross-track. The ramp takes ~95 % of the distance in this time.
        uint32_t posRampMs = 500;
        float    posSpikeMargin = 0.5f;
        // TIGHTENED 2026-09-24: the projection is resynchronised at EVERY fix, so after
        // 100 ms it can only be a few centimetres away from the truth (at 5 m/s the
        // machine covers 0.5 m). A deviation beyond ~0.5 m is therefore already a bad
        // fix - the original 3 * v * dt + 0.5 m allowed 2 m at 5 m/s and let the bridge
        // spikes of the 06:20 log through untouched.

        float    posSpikeNoiseM = 0.4f;

        // --- Operator correction 2026-09-29 ("der Traktor springt auf der Karte", bridge
        // log 06:34: 4 fixes of 1.4...2.9 m reached AgOpenGPS) -------------------------
        // The published position is the RECEIVER's own unless the fall-back is running,
        // so an accepted-but-wrong fix is visible 1:1. The gate above only decides
        // "fall-back or accept" - it does not limit an accepted step. With these two the
        // projection is also used as a CATCH-UP: a residual larger than what may be
        // covered in posRampMs is blended over a longer ramp (bounded by posRampMaxMs),
        // i.e. AgOpenGPS sees a continuous track instead of a jump. 0 disables it.
        float    posMaxCatchUpMps = 1.5f;   // m/s the published point may move on top of the model
        uint32_t posRampMaxMs = 3000;       // ceiling for the residual-based ramp time

        // ---- Filter tuning ---------------------------------------------------
        // Heading process noise in deg/s: 4.7 is the measured residual RMS of the
        // kinematic model over 1 s intervals on the bumpy track. The 2026-09-23
        // field logs (poor reception, 2.5 m/s and 8 m/s) give a worse fit, but the
        // prediction is only the bridge between two 10 Hz measurements, so the
        // value stays conservative.
        float qHeadingDegS = 4.7f;
        // Process noise while the WHEEL ANGLE is UNTRUSTWORTHY - VWAS quality below
        // wheelQualityLowPct (operator request "P1b", 2026-09-30). Then the kinematic
        // model is not the truth but the error source, so its process noise has to say
        // "unknown". Two effects, both wanted: sigma becomes honest for the consumers
        // (VWAS/NAVProcessor/log - AOG gets no sigma in PAOGI and trusts the heading
        // blindly), and the KF gain rises so psi follows the ACCEPTED measurements
        // instead of the invented rate.
        // Measured motivation, 2026-09-30 "Halle 2x": the VWAS had not learned its zero
        // yet (quality 0...13 %), the wheel angle read 34 deg while driving straight at
        // 7.6 m/s, the model therefore produced 86 deg/s and psi ran 52 deg away from
        // the direction of travel - while publishing sigma = 4.2 deg. Offline replay of
        // that log with yaw = 0 and this q: |psi - travel| p50 1.6 deg instead of 15.3,
        // p90 5.9 instead of 53.1. The start log (2026-09-30 6-23) stays neutral
        // (1.5 -> 1.6 deg p50) because its psi was carried by the COG fall-back, which
        // the existing gates had already validated - the fix must not trust a raw
        // measurement blindly, and it does not: the gates stay in force.
        float qHeadingWheelBadDegS = 25.0f;
        // Walk of the wheel-angle bias state in deg/s: tiny, the bias is a
        // calibration constant that may only follow slow changes.
        float qSteerBiasDegS = 0.02f;
        // Measurement noise, per frame. These are the values used when
        // adaptiveMeasurementNoise is off. MEASURED on the 2026-09-23 logs
        // (|step| per 100 ms, p90): dual heading 2.5 deg under trees (1.7 on the
        // 2026-09-21 road data), COG 6 ... 9 deg, so the old defaults (1.5/6)
        // were optimistic by 2-3x and the filter followed the noise instead of
        // averaging it (HB1 vs HB2 in the logs: psi p90 8.7 deg against a raw
        // azimuth p90 of 9.0 deg over the same intervals - pass-through).
        float rDualHeadingDeg = 4.0f;
        float rCogDeg = 8.0f;
        float rFixToFixDeg = 4.0f;
        // Learn R from the innovation statistics instead of trusting the numbers
        // above: R_used = clamp(EMA(innovation^2) - P_pred, lo, hi). Costs three
        // floats, no transcendental functions, and adapts to the reception of the
        // day (the difference between the 09-21 road and the 09-23 field data is
        // a factor 3 in the same machine).
        bool adaptiveMeasurementNoise = true;
        float rAdaptAlpha = 0.01f;        // ~100 frame time constant at 10 Hz
        // The adaptive estimate alone is too slow to react (operator report
        // 2026-09-27: "the heading flickers with bad reception, the autosteer then
        // steers wildly"). Measured over five drives: the per-frame heading step grew
        // from 0.10...0.26 deg (p50) to 0.60...0.75 deg and reached 9.5 deg (p99),
        // while sigma stayed at 2.4 deg - wrong by a factor 5...10. So a DIRECT factor
        // is applied on top of the adaptive value:
        //   * the attitude is not trusted (fall-back running) or reception is poor,
        //   * headingQuality below RTK fixed (4 fixed, 5 float) - THIS is the criterion,
        //   * and, only where a receiver is known to report a MEANINGFUL count, fewer
        //     satellites than rQualitySats.
        // Operator correction 2026-09-27: the satellite COUNT is not a quality measure.
        // Under trees it stays at 14...20 while the solution quality is what degrades,
        // so a count gate discriminates nothing - it only fires close to a total loss.
        // (Measured over the 2026-09-27 drives the count covered 5...34, so a threshold
        // of 15 was nearly always satisfied and did nothing either way.) Default 0 =
        // DISABLED, i.e. the heading quality decides.
        // This acts immediately instead of after 10 s.
        float rQualityDegraded = 3.0f;
        float rQualityMarginal = 1.8f;
        uint8_t rQualitySats = 0;
        // MODEL-PLAUSIBILITY GATE (operator request 2026-09-27: "Bei Änderung des
        // Lenkwinkels müsste die Ableitung der Heading-Änderung zur Änderung des
        // Lenkwinkels gleich sein. Dann bliebe alles berücksichtigt.").
        // The measurement is compared against the heading change the kinematics predict
        // over a SHORT window - not frame by frame:
        //   model change M = steerScale*X + bias*Q      over gateWindowMs
        //     X = integral 57.3*v*tan(delta)/L dt   (the model heading change)
        //     Q = integral 57.3*v*sec^2(delta)/L dt (the operator's DERIVATIVE column)
        //   dev = |measured change - M|
        //   fire when dev > max(gateFloorDeg, gateBandPct*|M|)
        // Limits derived from the 2026-09-27 drives (gate_sweep replay):
        //   0.3 s window: dev on RTK-fixed windows p50 0.5...1.5 deg, on non-RTK
        //   1.2...5.7 deg -> floor 3.0 deg fires on 2 % (hall) / 4 % (trees) of the good
        //   windows and catches 65 % (hall) / 16 % (trees) of the bad ones. A 1 s window
        //   does NOT separate any more (dev on good windows grows to 1.3...5.2 deg,
        //   because model errors integrate) - so the window stays short.
        // The tree figure is low by nature: there the receiver does NOT flag the
        // corruption, which is exactly the case this gate exists for.
        // It DAMPENS (R * gateRFactor for gateHoldMs) and NEVER discards: a wrong
        // scale/offset/wheelbase or slip must not be able to reject a real curve.
        bool     modelGateEnabled = true;
        uint32_t gateWindowMs = 300;
        float    gateFloorDeg = 3.0f;
        float    gateBandPct = 0.30f;
        float    gateRFactor = 2.0f;
        uint32_t gateHoldMs = 300;
        // WHEEL ANGLE RELIABILITY (operator decision 2026-09-28). The steering zero is
        // owned by the WAS/VWAS, but on this machine the Keya motor sets it at POWER-UP to
        // wherever the wheels stand ("der verwendete Keya Motor setzt 0 immer in der
        // Position, in der er steht, wenn der Strom angelegt wird"), so right after every
        // start the wheel angle is wrong by up to tens of degrees and has to be learned
        // quickly FROM DRIVING - which needs a MEASURED heading. The VWAS therefore
        // reports a DOP-like reliability (0...100, WheelAngleFusion::getAngleQuality) and
        // the bridge switches its behaviour. NOTE (operator, 2026-09-28): this number is
        // only meaningful for the VIRTUAL WAS - a real physical WAS (analog input or a
        // CAN WAS) has its own hardware zero, so the caller reports 100 for it from the
        // first cycle and the bridge never enters the fallback below:
        //   below wheelQualityLowPct:  model gate OFF, bias adaptation ON - the bias is
        //                              the only thing that can absorb an unknown zero
        //                              quickly (bounded by steerBiasLimitDeg)
        //   above wheelQualityHighPct: gate ON, bias OFF (the VWAS owns the zero)
        // A bias left over from the learning phase is ramped to zero again, so that the
        // two mechanisms never correct the same offset at the same time. Without the
        // gate the measurements rule the heading, the VWAS gets a measured heading and
        // its zero observer can actually learn - with the gate permanently firing
        // instead, the measurement noise is doubled, the heading follows the wrong model
        // and the learning is blocked (measured 2026-09-28: heading rotating although
        // both receivers were RTK fixed).
        // -1 (nobody reports a quality) keeps the previous behaviour: gate on, bias off.
        float    wheelQualityLowPct = 30.0f;   // below -> unusable
        float    wheelQualityHighPct = 40.0f;  // hysteresis: usable again above this
        // A heading that is ONLY the model (no attitude, no position support, i.e.
        // DEADRECKON) is not claimed as valid towards AOG after this long (operator
        // decision: fixQuality = 0 after 2 s of model operation, before that the value is
        // passed through).
        uint32_t drQf0AfterMs = 2000;
        // Output smoothing for the heading that goes to AgOpenGPS (operator request:
        // "smooth it at higher speed too, the trace in AOG then is calmer"). A
        // first-order filter alone LAGS by tau*omega while the machine turns steadily -
        // at 30 deg/s and 200 ms that is 6 deg, far too much for a steering system at
        // creeping speed. The filter therefore runs with a RATE LEAD: the current yaw
        // rate is added back, so a smooth real rotation passes through without delay
        // and only the fast jitter is damped. Time constant in ms; 0 disables.
        uint32_t outSmoothMs = 200;
        // ---- S-B (2026-10-10): GESTAFFELTE Ausgangsglaettung (Betreiber-Vorgabe) ----------
        // Messung (10-09-Logs): der feste 200-ms-LPF senkt das Heading-Rauschen um Faktor
        // 3,0-5,6, laesst aber die Abweichung zur Wahrheit unveraendert - er ist eine
        // ANZEIGE-Massnahme. Die Staffelung nutzt, dass Rauschen nur bei ruhiger Fahrt
        // stoert: 80/20 (alpha=0,2 neu) bei |Lenkwinkel|<=4 Grad UND v<=2 m/s,
        // 70/30 bei <=6 Grad UND <=3 m/s, sonst 50/50. Der Modellraten-Vorhalt bleibt
        // unveraendert (er darf NICHT aus der Ableitung des geglaetteten Signals kommen -
        // das hat in der Simulation den Gewinn vollstaendig aufgehoben).
        bool stagedOutFilter = true;
        float stage1WheelDeg = 4.0f;
        float stage1SpeedMps = 2.0f;
        float stage1Alpha = 0.2f;
        float stage2WheelDeg = 6.0f;
        float stage2SpeedMps = 3.0f;
        float stage2Alpha = 0.3f;
        float stage3Alpha = 0.5f;
        // ---- A10 (2026-10-11): Glattungsstaerke nach dem GEMESSENEN RAUSCHEN ----------
        // Messung (B10.18, 10-11): p50|dAtt| ueber 5 Frames trennt gute und schlechte
        // Frames deutlich (rauschstaerkstes Viertel 2,2x / 5,5x Fehler gegen das
        // ruhigste), waehrend Tempo und Lenkwinkel das nur indirekt tun: eine schnelle
        // Gerade im Freien ist ruhig, eine langsame unter dem Baum nicht.
        // Gemessen wird |d Eingangsheading| (der ROHE Empfaengerwert, nicht der eigene
        // Ausgang). Das kann echte Drehung nicht von Rauschen unterscheiden - tut aber
        // genau das Richtige: schnelle Aenderung (echte Kurve ODER Rauschen) -> wenig
        // Glaettung, ruhiger Verlauf -> viel Glaettung.
        // Kombination mit der Physik-Regel: alpha = MIN(stage, noise), die Regel bleibt
        // also die OBERGRENZE der Glaettung (in der Kurve weiterhin 50/50). Ein Wechsel
        // kann die bestehende Staffelung damit nie verschlechtern.
        // Der Vorhalt benutzt jetzt das WIRKSAME tau (dt/alpha), begrenzt auf outSmoothMs.
        bool    outNoiseAdaptive = false;        // Standard AUS: Wirkung noch nicht belegt
                                                 // (Messung 10-11: an/aus fast identisch,
                                                 // weil der Eingang meist GEHALTEN wird und
                                                 // damit kein Rauschen sichtbar ist)
        uint32_t outNoiseWindowMs = 500;         // Median-Fenster in ZEIT (5 Frames bei 10 Hz)
        float   outNoiseQuietDeg = 0.6f;         // <= das: ruhig  -> alpha = stage1Alpha
        float   outNoiseBusyDeg = 3.0f;          // >= das: unruhig -> alpha = stage3Alpha
        float   outNoiseAlphaSlewPerS = 1.0f;    // Traegheit der Stufe gegen Flackern
        // ---- A10-Schicht 1 (2026-10-11): PHYSIKALISCHER ZAUN auf der Ausgabe -----------
        // Betreiber-Vorschlag: die Spike-Leitplanke direkt aus dem KORRIGIERTEN Lenkwinkel
        // bilden (kinematicRateBoundDegS() rechnet schon mit trim + Slack + L_eff), dann
        // sind Kurven automatisch moeglich: der Zaun ist ein PRODUKT mit tan(delta_eff),
        // also ~0 auf der Geraden und weit in der Kurve, und er waechst mit v - "je
        // langsamer, desto ruhiger" faellt damit aus derselben Physik heraus.
        // Gemessen (10-11, Feldlogs): auf |Lenkw| <= 2 Grad liegt der Zaun bei 0,45...0,70
        // Grad/s, die TATSAECHLICHEN Ausgabeschritte aber bei 5...8 Grad/s (p90) - 53...78 %
        // aller Schritte sind physikalisch unmoeglich. Genau die soll der Zaun abfangen.
        // Der Zaun deckt EMA-Schritt, Snap UND RATE-LEAD gemeinsam ab (der Vorhalt wurde
        // bisher ungefenced addiert - ein eigener Jitter-Kanal bei falschem Nullpunkt).
        // outFenceK = 0 => AUS (Standard). Der Wert wird aus der Messung festgelegt:
        //   Treue gegen das 1-s-Mittel <= 0,5 Grad, Kurvenlatenz < 150 ms, p99 Schritt
        //   <= 0,5 Grad. Er ist als Faktor gedacht (Grip/Schlupf: das Modell unterschaetzt
        //   bis ~1,4x, B8.43).
        float   outFenceK = 0.0f;              // 0 = Zaun aus (Instrument, k folgt aus Messung)
        float   outFenceFloorDegS = 1.0f;      // Boden: bei Stillstand nicht einfrieren
        uint32_t outFenceValveMs = 1000;       // so lange begrenzt = kein Artefakt => Rampe
        // A10/10-11 (Betreiber-Auftrag): der Ratenvorhalt ist eine MODELL-Groesse - er darf
        // nur laufen, wenn der Lenkwinkel, aus dem das Modell gerechnet wird, wirklich
        // vertrauenswuerdig ist (Status::wheelObsTrusted, dieselben Gates wie der Trim).
        // Messung 10-11: auf Frames, auf denen der PFAD gerade sagt (<= 1,5 Grad/s) und der
        // rohe Lenkwinkel 25...28 Grad stand (falscher Nullpunkt, B7.20), setzte der Vorhalt
        // ~7,8 Grad daneben - das waren die 2,0-Grad-Schritte in az_out. Ohne Vorhalt folgt
        // die Ausgabe dem (richtigen) Lage-Mittel; die KURVENANTWORT bleibt, weil dort der
        // Lenkwinkel vertrauenswuerdig ist und der Vorhalt wieder an ist.
        bool    outLeadNeedsWheelTrust = true;
        // ---- Attitude QUALITY GATE for the AOG output (2026-10-02) ---------------
        // Operator request: the data sent to AgOpenGPS must be free of jumps but must
        // still show the current state without delay - and under the trees the raw
        // UM982 attitude carries 2...20 deg single-frame artifacts (measured 2026-10-01/02).
        //
        // The receiver's OWN UNIHEADINGA standard deviations are the per-EPOCH quality
        // figure for that (uniHdgStdDevDeg): unlike the adaptive R (an EMA that needs
        // ~10 s) they say "this epoch is bad" immediately. Measured separation on the
        // 2026-10-01/02 logs: uni_hdgsd > 1.5 deg detected the heading spikes with 92 %
        // and 10 % false alarm, the baseline deviation with 90 % / 12 % - both far
        // better than roll (38 % / 4.2 %), and both are already in GNSSData but were
        // never read by the fusion (tools/heading_bridge/README.md).
        //
        // COMPATIBILITY (operator note: "the code works well with F9P + IMU"): an F9P
        // does not send UNIHEADINGA, so uniHdgStdDevDeg stays 0 -> "unknown" -> the
        // gate is OPEN and the output path is bit-identical to the version before this
        // gate. Only a Unicore receiver with UNIHEADINGA enabled is affected.
        //
        // What the gate does: while the epoch is clean the smoothing residual may SNAP
        // (no lag at all). While the epoch is bad the residual is RATE-LIMITED instead
        // of snapped, so a receiver artifact becomes a bounded ramp piece instead of a
        // step - see Config::outSnapDeg and the output section in update().
        bool  outQualityGateEnabled = true;
        float uniHdgSdGoodDeg = 1.0f;   // <= this: clean epoch, snap allowed
        float uniHdgSdBadDeg  = 2.5f;   // >= this: bad epoch, snap suppressed
        // The baseline deviation test below uses Config::baselineTolPct - the SAME knob as
        // the reject gate in evaluateAttitudeFrame(). There used to be a second field
        // (uniBaseTolPct = 5.0) with the same value, i.e. two knobs for one decision; merged
        // 2026-10-03 on the operator's decision: one number, one meaning.
        //
        // It deliberately stays a SHARED knob rather than two: whatever the right value is,
        // it is right for both consequences. Beyond the tolerance the measurement is not
        // trustworthy - so it may neither be DISCARDED nor allowed to SNAP, and both gates
        // now agree on that.
        // ZEITBASIERT seit 2026-10-12 (GRUNDZIELE 1.0: Ziele in Grad/s; Logs mit 5/10/20 Hz).
        // Der Schwellwert ist als RATE angegeben und wird je Frame mit dt umgerechnet:
        // 30 Grad/s = 3 Grad bei 10 Hz (der alte Wert), 6 Grad bei 5 Hz, 1,5 Grad bei 20 Hz.
        float outSnapRateDegS = 30.0f;   // Residual-Rate, ab der der Ausgang "snappen" darf
        // ---- Confirmation of a snap on a DEGRADED epoch (2026-10-02) -------------
        // A hard rate limit CANNOT separate a receiver artifact from real motion at
        // speed: at 8 m/s the machine may legitimately yaw ~13 deg/s = 1.3 deg per
        // 100 ms frame, so the bound has to be that wide and the artifact fits through.
        // The instrument that does separate them is TIME: measured on the 2026-09-30
        // drive, 21 of 22 az_out peaks were SINGLE-FRAME (they come and go), while a
        // real change - a direction change, a re-solution, the start of a tight turn -
        // persists. So on a degraded epoch a big residual is held until it is confirmed
        // by a second frame, and a single-frame artifact therefore never reaches AOG.
        // Cost: a REAL step is delayed by one confirmation frame (~100 ms) instead of
        // 0 - that is the smallest delay that can tell the two apart. The hold is
        // capped by outHoldMaxMs so a chain of alternating artifacts cannot mask a real
        // state change for longer.
        // 0 = no confirmation (snap immediately, i.e. the behaviour before 2026-10-02).
        uint32_t outSnapConfirmMs = 100;   // Bestaetigungsdauer in ms (1 Frame bei 10 Hz war
                                           // der alte Wert - als Frame waere das bei 5 Hz
                                           // 200 ms und bei 20 Hz 50 ms)
        uint32_t outHoldMaxMs = 300;
        // Rate bound for the output residual while the epoch is bad, expressed as a
        // MULTIPLE of the physical turn rate (the same kinematics the measurement
        // limiter uses). 1.0 = exactly what the machine can physically yaw: this is a
        // SECONDARY guard for the small residuals, not the spike defence (that is the
        // confirmation above). 0 disables it.
        float outResidRateFactor = 1.0f;
        float rAttMinDeg = 1.0f,  rAttMaxDeg = 12.0f;
        float rPathMinDeg = 2.0f, rPathMaxDeg = 20.0f;
        // Circular average over the last N COG samples before it is used as a
        // heading measurement. The COG is the noisy one (p90 6 ... 9 deg) and the
        // bridge only uses it while the attitude is gone, where 0.3 s of lag does
        // not matter - the attitude path stays unfiltered, because a delayed
        // heading moves AgOpenGPS' pivot point (0.1 s = 0.25 m at 2.5 m/s).
        uint8_t cogAverageSamples = 3;
        // Below this speed neither the COG nor the position window says anything
        // about the heading, and a standing machine does not yaw: hold the
        // estimate and do NOT inflate the covariance (otherwise the return to LIVE
        // would be slow for no reason).
        float minSpeedForMeasurement = 0.30f;
        // Side slip / path-vs-body offset. COG and fix-to-fix describe the PATH,
        // the estimate describes the BODY; on a side slope they differ by a few
        // degrees, which shows up as a step every time the source changes. The
        // offset is measured while the attitude is trusted and removed from the
        // falls back to the path measurement from the mount offset, one level down.
        bool estimateSideSlip = true;
        float sideSlipAlpha = 0.002f;     // ~5 s time constant at 10 Hz
        float sideSlipLimitDeg = 15.0f;
        // Wheel angle bias state of the model: delta_eff = steerScale*delta + bias.
        // OFF since the operator decision 2026-09-27: "den 0 punkt der lenkung wird von
        // der lenkung VWas festgelegt. wenn ein echter was verbaut ist, wird der von der
        // Hardware einmalig 0 gesetzt." The zero belongs to the WAS/VWAS, so the bridge
        // must NOT learn it - two learners on one quantity fight each other, which is
        // exactly what happened on the 14-12 drive (state 5.1 -> +15.0 at its clamp while
        // the VWAS was still learning its own trim). The state and its limit stay for a
        // machine whose WAS zero really is not trustworthy; the angle is then taken from
        // the WAS as it is.
        // Diagnostic note: the VWAS encoder offset moved by tens of degrees BETWEEN the
        // drives of 2026-09-27 (-38.81 / -28.97 / +25.46) although a real WAS is fitted -
        // that is a finding for the steering task, not something the bridge can absorb.
        bool estimateSteerBias = false;
        float steerBiasLimitDeg = 15.0f;
        // Which of the two surviving measurements may correct the estimate.
        // Both are on by default; they can be separated for A/B test drives.
        bool enablePathMeasurement = true;  // fix-to-fix, preferred
        bool enableCogMeasurement = true;   // VTG course over ground, fallback

        // ---- Roll channel ----------------------------------------------------
        // MEASURED on the 2026-09-23 logs: the dual-GPS roll steps by p50 0.76 deg
        // and p90 7.2 deg per 100 ms. AgOpenGPS turns the roll into a POSITION
        // correction (sin(roll) * antennaHeight): at 3 m antenna height a 7 deg step
        // shifts the position by 0.37 m - visibly a jumping tractor. 0.35 s of LPF
        // still passed ~30 % of every step; 0.8 s passes ~12 % (0.06 m) and the
        // sidehill draft term does not care about 0.8 s of lag.
        float rollLpfTauS = 0.25f;         // 2026-10-10: 0.8 s war die eigentliche Latenz fuer GUTE Daten;
                                           // der Roll-Halt (roll_src=4) schuetzt die schlechten Phasen.
        float rollHoldMs = 1000;           // (alt) hold the last value this long
        float rollFadeTauS = 0.5f;         // Fade nach 0: tau 0,5 s = nach ~2 s unsichtbar
        bool rollFadeToZero = true;
        // ---- Roll HALTEN bei Lage-Ausfall (Betreiber-Entscheid 2026-10-06) ----------
        // Nicht sofort und nicht auf 0: gehalten wird der MITTELWERT der letzten GUTEN
        // Meter (distanzgewichtetes Fenster rollGoodAvgWinM, nur validierte Epochen).
        // Begruendung des Betreibers: der Roll ist bei schlechtem Empfang der ERSTE, der
        // falsch wird, und braucht nach dem Ausfall Sekunden, bis er wieder gut ist. Und
        // beide Verbraucher brauchen ihn: AgOpenGPS' Hangkompensation (sin(roll)*H) und
        // unser k*roll-Term. Fade erst, wenn 20 m gefahren ODER 45 s vergangen sind.
        float rollGoodAvgWinM = 5.0f;      // Fenster des "guten" Mittels (2...5 m)
        float rollHoldM = 20.0f;           // so lange (in Metern) wird der Mittelwert gehalten
        uint32_t rollHoldMaxMs = 45000;    // Zeitdeckel (0 = aus), greift bei Schleichfahrt
        // ---- IMU-Roll LIVE (Betreiber 2026-10-06) -----------------------------------
        // "die IMU braucht eigentlich keinen Roll-Filter, die stimmt eigentlich immer,
        // auch bei schlechtem Empfang." Also: ist die IMU-Roll als PLAUSIBEL erkannt
        // (Abweichung zum sauberen Dual-Roll klein - in den 10-04-Logs 2...3 Grad), wird
        // sie live und ungefiltert durchgereicht - kein Halten, kein Faden. Weicht sie
        // dauerhaft ab (Achse/Einheiten verdreht; in 4 Logs gemessen 68...177 Grad), wird
        // sie als unbrauchbar markiert und es bleibt beim Halten/Faden.
        bool  imuRollLive = true;
        float imuRollPlausDevDeg = 5.0f;   // erlaubte Abweichung Dual<->IMU (EMA)
        float imuRollPlausTauS = 5.0f;     // EMA-Zeitkonstante der Abweichung
        float imuRollLpfTauS = 0.1f;       // nur gegen Einzelframe-Ausreisser (10 Hz)
        // ---- Sidehill/Draft im MODELL (2026-10-06) ---------------------------------
        // AgOpenGPS addiert fuer die Hangdrift Lenkwinkel auf den SOLL-Winkel. Dieser
        // Anteil erzeugt KEINE Gierrate - er wird im Reifenschlaupf verbraucht -, also muss
        // er aus dem Winkel heraus, den das Modell in omega = v*tan(delta)/L einsetzt.
        // Nach aussen (az_out/PAOGI) melden wir unveraendert wie bisher.
        bool  sideHillInModel = true;
        float sideHillGainDegPerDeg = 0.12f;   // = AgOpenGPS "Side Hill Draft Gain"
        bool  sideHillGainSignFlip = true;     // 2026-10-06 abends: k-Selbsttest im
                                               // Sidehill-Log (Roll p50 13,2 Grad) gibt
                                               // Rest-k = -0,126 -> der Term muss ADDieren,
                                               // nicht abziehen (sonst verdoppelt er den
                                               // Fehler: psi 11,6 -> 16,7 Grad gemessen).
        // ---- M1: psi an den Empfaenger ankern (2026-10-06) --------------------------
        // Gemessen in 6-48: bei gutem Empfang ist der Empfaenger 1,7 Grad genau, waehrend
        // psi 11,6 Grad neben ihm laeuft - und im Ausfall uebernimmt dieses falsche psi.
        // Solange der Empfaenger sauber ist, darf er psi deshalb STAERKER nachfuehren:
        // R der Attitude-Messung (kind 0) x psiAnchorRScale. Kein harter Zwang, keine
        // neue Rampe - der Filter zieht in ~1 s nach, sichtbar ist davon nichts.
        bool  psiAnchorEnabled = true;
        // 2026-10-07: verschaerft. Belegt: psi driftete +8,8 Grad vom (einwandfreien)
        // Empfaenger weg (E5a, Log 6-13) und im Neustart-Modus (Modell wirkungslos) lag
        // psi bei 2,1 Grad statt 9,1. Also die Nachfuehrung deutlich staerker und auch bei
        // mittlerer Empfaenger-Qualitaet aktiv (Baeume/Bruecke bleiben oft unter 1,5 Grad).
        float psiAnchorSdDeg = 1.5f;
        float psiAnchorRScale = 0.3f;    // 0,3 = gut dreifache Nachfuehrung

        // ---- Heading conversion (identical to AgOpenGPS) ----------------------
        // ONE offset, with the SAME sign convention as AgOpenGPS:
        //
        //     AgOpenGPS : body = received + dualHeadingOffset   (UDPComm.Designer.cs:85)
        //     firmware  : body = az_recv  + dualHeadingOffset    <-- same number,
        //                                                            same formula
        //     published : psi - dualHeadingOffset  (= the received azimuth itself in
        //                                            normal operation, see below)
        //
        // That is the whole calibration. There is no second, internal constant any
        // more: a machine-specific correction applied only in the firmware would
        // make the two sides disagree, and a correction hidden here cannot be seen
        // or adjusted by the operator.
        //
        // Value: +90 or -90 depending on which way round the two antennas sit on
        // the roof - swapping them changes the SIGN (they are 180 deg + the
        // mechanical error apart, i.e. no small trim). MEASURED:
        //   2026-09-23 12:16  mnt = az - direction = +92 ... +95  -> offset -92
        //   2026-09-23 19:57  mnt = az - direction = -86 (p50)     -> offset +86
        // The 19:57 value is the CORRECT mounting (operator: antennas back in the
        // original order).
        //
        // 2026-10-03: the magnitude is now the MEASURED machine value instead of the generic
        // 90. tools/analysis/offset_check.ps1 (travel direction from the position chord vs
        // the receiver azimuth, straight clean driving) gives 88.3 deg over three logs /
        // 520 samples with sd 0.9; the firmware's own `mount_deg` reads -89.0 ... -89.3, and
        // AgOpenGPS is set to +88 by the operator.
        // The generic 90 was not harmless: the error is active in EVERY frame, because
        //   body      = az_recv + dualHeadingOffsetDeg   (line ~460)
        //   published = psi      - dualHeadingOffsetDeg  (line ~2065)
        // so 90 instead of 88.3 puts both poles 1.7 deg off. 88.3 makes the two sides agree
        // within 0.3 deg.
        // Persisted in the EEPROM (1600-1699 bytes 11..16) and settable in the web UI
        // (/vwas, "Antennen-Geometrie"); this literal is only the fallback for a board that
        // has nothing stored.
        // Antennen-Vorlauf: Hebel vom Drehpunkt (Gelenk beim Knicklenker,
        // Hinterachse beim Schlepper) bis zur Antenne, in Metern.
        //
        // 0.91 m ist der GEMESSENE Wert dieser Maschine: Stiga 0,45 m vor der
        // Vorderachse plus 0,46 m halber Radstand = 0,91 m ab dem Gelenk
        // (B10.68/B10.70). Er wird benutzt, wenn im EEPROM nichts steht - ohne
        // diesen Fallback war die Korrektur still auf 0, und die Sehne lief der
        // Koerperrichtung in der Kurve nach (gemessen 7-52: skew p50 +5,9 /
        // p90 18,8 Grad).
        //
        // Dieselbe Regel wie physicalBaselineDefaultM: nichts gespeichert ->
        // Maschinenkonstante aus dem Build; ein eingetragener Wert gewinnt immer.
        static constexpr float antennaPivotDefaultM = 0.91f;

        static constexpr float dualHeadingOffsetDefaultDeg = 88.3f;
        float dualHeadingOffsetDeg = dualHeadingOffsetDefaultDeg;
    // ---------------------------------------------------------------------------
    // Effektive Wheelbase fuer das MODELL (operator decision 2026-10-05).
    //   omega = v * tan(steerScale*delta + bias) / (wheelbaseM * effWheelbaseFac)
    // AgOpenGPS's Vehicle-Wheelbase bleibt UNVERAENDERT - hier steht nur, um wieviel
    // laenger die Strecke ist, die die Maschine fuer dieselbe Lenkwinkel-Aenderung
    // wirklich braucht (Reifenschlupf, Untersteuern, VWAS-Skala; im Code unten als
    // "2,14 m -> 2,95 m effective wheelbase" beschrieben, Faktor ~1,38).
    // GEMESSEN 2026-10-05 (per Frame, Geradeaus, wheel 0,09...0,88 Grad): das Modell
    // dreht 1,2...1,4x zu viel - der Ueberschuss (+0,5...+0,9 Grad/s) baut in ~10 s
    // einen 9-Grad-Offset auf, weil die Korrektur ~0 ist. VORLAEUFIG 1,3
    // (= 2,14 m -> 2,78 m). 1.0 = altes Verhalten.
    // ---------------------------------------------------------------------------
    // 2026-10-06 abends zurueck auf 1.0: Die 1.30 liessen das Modell 23 % zu wenig Gierrate
    // rechnen. Belegt im Log 20-38: bei einer echten Kurve (~33 Grad/s, att 204,9 -> 221,2 in
    // 0,5 s, qf=4, 15 Sats, sigma 0,02 m) hinkte psi 29 Grad nach und die 30-Grad-Notbremse
    // setzte ihn zweimal hart um +31 Grad (42,0 s und 43,8 s) - das sind genau die Spruenge
    // und der kurze Richtungswechsel, die der Betreiber gesehen hat. Der Empfaenger war
    // einwandfrei; falsch war das MODELL. 1.0 = richtige Gierrate -> kein Nachhinken mehr.
    float effWheelbaseFac = 1.0f;
    // Deckel fuer R der Attitude. Der adaptive Wert fuellt sich aus dem ZUSTANDSfehler
    // und schuetzt einen bestehenden Offset (gemessen: meas_r 3,95 -> 6,27, gain 0,016).
    // Mit dem Deckel ist K1 ~0,09 statt 0,016 - die verifizierte Attitude holt ~6x
    // schneller zurueck. Wirkt nur, wenn der Empfaenger NICHT selbst ein groesseres
    // 1-sigma meldet (uniHdgStdDevDeg) - schlechter Empfang hebt ihn auf.
    float attRCapDeg = 1.5f;
    // ---- LIVE-Gate (2026-10-05, Betreiber-Entscheidung) ----------------------------
    // Solange der Empfaenger nachweislich sauber ist, wird sein EIGENES Azimut LIVE
    // durchgereicht (genau wie heute beim Kaltstart vor dem ersten vertrauenswuerdigen
    // Frame) statt den Filterzustand psi zu senden. Begruendung aus den Logs vom
    // 2026-10-05: auf der Fahrt mit 9,6 Grad Schraeglage war der Empfaenger die ganze
    // Zeit sauber (uni_hdgsd p50 0,43 / p90 0,70 Grad, uni_ptchsd p50 0,83, |baseline_dev|
    // p50 0,9 %, |roll| <= 2,4 Grad) -> das Gate waere 95,1 % der Geradeaus-Frames offen,
    // und der Empfaengerwert ist dort die Wahrheit (att - cog stabil -88,2 Grad), waehrend
    // psi den 9,6-Grad-Versatz stehen laesst.
    // Hysterese (rein bei <= InSd, raus erst bei > OutSd), damit das Gate bei
    // schwankender Qualitaet nicht flattert. Ein Zappeln waere unkritisch (die
    // Ausgangsstufe bestaetigt grosse Residuen, bevor sie sie durchlaesst), aber es
    // kostet Durchlass.
    // Im Log ist der Gate-Zustand direkt erkennbar: offen  -> az_out == att_hdg_deg.
    bool  liveGateEnabled        = true;
    float liveGateInSdDeg        = 0.8f;   // uniHdgStdDevDeg: rein
    float liveGateOutSdDeg       = 2.0f;   // uniHdgStdDevDeg: raus
    float liveGateInPtchSdDeg    = 1.5f;   // uniPtchStdDevDeg (= ROLL, laeuft zuerst weg)
    float liveGateOutPtchSdDeg   = 3.0f;   // dito: raus
    float liveGateBaselineTolPct = 5.0f;   // |baselineDevPct| Grenze des Gates
    // ---- harter Rampen-Start (gemessen 2026-10-05 abends) -------------------------
    // An den Wiedereintritten reichen die Qualitaetszahlen NICHT: gemessen meldete der
    // Empfaenger dort sd 0,4...0,8 Grad, Baseline-Abweichung ~0 % und qf=4 - und das
    // Heading lag trotzdem 5...15 Grad (extrem 70...100 Grad) neben der Projektion.
    // Deshalb: Start erst nach reentryGoodFrames guten Epochen UND Einigkeit mit der
    // Projektion (travel-Witness), und der Wert wird ueber reentryRampMs gerampt.
    // Ebenso gemessen: die Haelfte der guten Phasen ist kuerzer als 1 s und viele nur
    // 1 Epoche -> ohne diese beiden Bedingungen flackert das Gate.
    uint16_t reentryGoodFrames = 5;      // 0,5 s anhaltend gute Qualitaet
    uint16_t gateBadFrames     = 3;      // Schliessen erst nach 3 schlechten Epochen
    float    reentryAgreeDeg   = 5.0f;   // Einigkeit mit der Projektion (0 = Bedingung aus)
    uint16_t liveGateRampMs     = 800;    // Rampe beim Oeffnen des Gates
    float    liveRampDegPerS   = 20.0f;  // maximale Nachfuehr-Geschwindigkeit der Rampe
    // ---- Roll-Qualitaet -----------------------------------------------------------
    // uniPtchStdDevDeg ist das eigene 1-sigma des Baseline-Pitch = ROLL (die Baseline
    // liegt quer). Gemessen: 0,86...0,93 Grad bei gutem Empfang, 159 Grad bei schlechtem.
    // Ein solcher Frame kann 30...60 Grad daneben liegen und schiebt ueber die LPF einen
    // Roll-Bias in AOG (sin(roll) * Antennenhoehe). Hysterese 3/6 Grad.
    float rollQualitySdDeg          = 3.0f;
    float rollQualitySdOutDeg       = 6.0f;
    float rollQualityBaselineTolPct = 5.0f;
        // Online measurement of the mounting (az_recv - direction of travel). Its
        // negative value is the number for dualHeadingOffsetDeg, i.e. the value for
        // AgOpenGPS. Diagnostic + calibration source, never applied silently.
        float mountOffsetMinSpeed = 1.5f;   // m/s, needs real travel
        float mountOffsetTauS = 30.0f;      // slow: a mounting constant
        // ACTIVE by default (operator decision 2026-09-22): the module publishes
        // the qualified heading/roll and the plausibility gates are in the path.
        // Set to false for a pure measurement run.
        bool exposeToAog = true;
        bool logEvents = true;

        // ---- Raw frame log (10 Hz, direct to USB serial) ----------------------
        // One CSV line per GNSS fix with EVERYTHING needed to analyse the bridge
        // offline - position, VTG speed/course, HPR attitude, wheel angle, the
        // bridged heading with its uncertainty, the projection residuals and the
        // calibration estimates. Written with Serial.write() (no EventLogger, no
        // token bucket) and only while the USB TX buffer has room, so it can never
        // stall the 100 Hz loop.
        //
        // Port load: ~200 characters at 10 Hz = 2 kB/s. The Teensy 4.1 USB CDC is
        // 12 Mbit/s (full speed), i.e. ~1.2 MB/s - the raw line uses ~0.2 % of it.
        // CPU: ~50 ... 80 us per line (float formatting) = 0.08 % at 10 Hz.
        bool logRawFrames = true;
        // 2026-11-01: zurueck auf 10 Hz. Die 500 ms (2 Hz) vom 30.10. waren fuer eine lange
        // Fehlersuche gedacht, haben aber die Dynamik aliast: die 60-Grad/s-Spitzen und die
        // 49 Spruenge > 5 Grad waren noch sichtbar, ihre FORM nicht. Alle Referenz-Baselines
        // (70_feldprobe §9) sind 10 Hz - ohne die ist kein Vergleich moeglich.
        uint16_t rawLogIntervalMs = 100;   // 10 Hz, unabhaengig vom EventLogger
    };

    // Where the published heading comes from right now. This is the value the
    // web UI and the log show, and it is what tells the operator whether a
    // number is a measurement or an estimate.
    enum class Source : uint8_t {
        NONE = 0,        // nothing valid yet
        ATTITUDE,        // dual-antenna attitude (the normal case)
        PATH_PROJECTED,  // forward-projected position residual (preferred fallback)
        PATH_FIX2FIX,    // position derived direction, >= 0.5 m window
        PATH_COG,        // VTG course over ground
        PATH_IMU_HEADING,// IMU heading, offset-corrected against the attitude
        DEADRECKON       // wheel angle + speed only
    };

    struct Status {
        State state = State::INIT;
        RejectReason lastReject = RejectReason::NOT_YET;
        Source source = Source::NONE;

        float vehicleHeadingDeg = 0.0f;   // fused body heading
        float azimuthDeg = 0.0f;
        // azSrc (10-13, B10.38): WOHER die AUSGABE kommt - 0 Lage, 1 Ringpuffer-Kurs,
        // 2 travelWitness/Fahrtrichung, 3 letzter Wert (gehalten). Die Spalte `src`
        // zeigt nur die zuletzt ANGEWENDETE Messung und hat genau diesen Punkt verdeckt.
        uint8_t azSrc = 0;          // what PAOGI field 12 must carry
        // --- Geschwindigkeit aus Weg/Zeit (2026-10-14) --------------------------
        // speedPosMps = Weg/Zeit roh (HR-Spalte v_pos), speedFusedMps = das, was die
        // Bridge INNEN benutzt (Weg/Zeit + EMA), speedRatio = v_pos / v_empfaenger
        // (1.00 = einig; 1.45 = Empfaenger zu langsam, gemessen 18-47),
        // speedDoubt = 1 solange der Empfaengerwert unplausibel ist (dann geht der
        // geglaettete Wert nach AOG).
        float speedPosMps = 0.0f;
        float speedFusedMps = 0.0f;
        float speedRatio = 1.0f;
        uint8_t speedDoubt = 0;
        // Diagnose der Weg/Zeit-Messung: welches Paar wurde benutzt (Zeit/Strecke).
        float speedWinDtS = 0.0f;
        float speedWinDistM = 0.0f;
        float rollDeg = 0.0f;
        float sigmaDeg = 0.0f;            // 1 sigma of the heading estimate
        float steerBiasDeg = 0.0f;
        float yawRateDegS = 0.0f;
        // --- P1 transparency (operator decisions 2026-09-30) -------------------
        // rollSuspect: the frame was accepted although |roll| was outside the envelope,
        // because an independent source confirmed the heading. The roll output holds
        // (rollSource becomes 2/3), so this flag tells the two cases apart.
        // yawRateZeroed: the kinematic yaw rate was NOT used because the wheel angle is
        // untrustworthy (quality < Config::wheelQualityLowPct) - psi then holds/follows
        // the measurements instead of an invented rotation.
        bool rollSuspect = false;
        bool yawRateZeroed = false;
        // --- P2 transparency (operator decision 2026-09-30) --------------------
        // rateLimitAllowedDegS: the rate bound the limiter used this frame (deg/s),
        // rateLimitPathDegS: the part of it that came from an accepted path turn,
        // rateLimitClipped: 1 when the correction had to be clipped to that bound,
        // lastMeasRDeg: the measurement noise R actually used by the applied
        // measurement (shows the COG 1/v damping and the adaptive estimate at work).
        float rateLimitAllowedDegS = 0.0f;
        float rateLimitPathDegS = 0.0f;
        float lastMeasRDeg = 0.0f;
        uint8_t rateLimitClipped = 0;
        // --- F1/F4 transparency (operator decision 2026-10-01) -----------------
        // stuckRescues: frames accepted although a plausibility gate rejected them,
        //   because NO measurement had been accepted for stuckRescueAfterMs (the
        //   state, not the measurement, was the suspicious one).
        // reverseClears: a wrongly latched REVERSE that the POSITION witness cleared
        //   immediately (front axis vs direction of travel from the positions).
        uint32_t stuckRescues = 0;
        uint32_t reverseClears = 0;

        uint32_t attitudeAgeMs = 0;       // since the last TRUSTED frame
        float bridgeDistanceM = 0.0f;     // travel since then
        float lastEventMs = 0.0f;
        float lastEventDistanceM = 0.0f;
        // True dead reckoning: since the last measurement of ANY kind.
        uint32_t deadReckonMs = 0;
        float deadReckonM = 0.0f;

        // Measured mounting offset (azimuth - path direction) and the solved
        // baseline length, both for the web UI / the AOG setting.
        float mountOffsetMeasuredDeg = 0.0f;
        uint32_t mountOffsetSamples = 0;
        // Configured offset and measurement must have OPPOSITE signs (offset = -mnt).
        // False = the antennas sit the other way round than configured, i.e. the
        // heading sent to AOG points backwards.
        bool mountOffsetSignOk = true;
        float baselineLengthM = 0.0f;     // as reported by the receiver (0 = n/a)
        float baselineDevPct = 0.0f;      // deviation from the physical value
        float baselineRefM = 0.0f;        // learned or configured reference length
        uint32_t baselineLearnCount = 0;  // frames used for the learned reference
        // Safety net state (Config::baselineGuardWindow / baselineGuardRejectPct). A
        // reference that rejects nearly everything is WRONG, so the test switches itself
        // off instead of silencing the heading. Set by the net, cleared by
        // setPhysicalBaselineM() or a reboot.
        bool     baselineTestDisabled = false;
        uint16_t baselineGuardCount = 0;     // judged frames in the current window
        uint16_t baselineGuardRejects = 0;   // of those, rejected by BASELINE_LENGTH
        // Active reference check (Config::baselineSuggestFrames): the length the receiver
        // solves on the first clean frames with NO reference involved. Reported as the
        // suggested value (web page can adopt it with one click) and used to flag a wrong
        // FIXED reference early - see baselineRefSuspect.
        float    baselineSuggestionM = 0.0f;   // running mean while collecting
        uint16_t baselineSuggestCount = 0;     // clean frames collected so far
        bool     baselineRefSuspect = false;   // suggestion outside +-baselineTolPct of
                                               // the FIXED reference (reported, never applied)
        // Forward projection: cross/along-track residual against the newest fix and
        // the resulting heading measurement (diagnostics + the R it was given).
        float projCtkM = 0.0f;
        float projAlongM = 0.0f;
        float projDistM = 0.0f;
        float projRmsM = 0.0f;
        float projHeadingDeg = 0.0f;
        float projRdeg = 0.0f;
        uint32_t projUpdates = 0;
        uint32_t pathProjUpdates = 0;
        // Self-calibration state: how many estimator samples are in, how much the
        // estimate still moves, whether it is trusted, and whether it was stored.
        uint32_t steerScaleLearnSamples = 0;
        float    steerScaleSpread = 0.0f;
        bool     steerScaleReady = false;
        bool     steerScaleStored = false;
        // Recommendation for Config::steerScale, estimated from the data (measured vs
        // predicted curvature). SCALE ONLY since 2026-09-27: the offset term was dropped
        // as ill-conditioned, and the zero belongs to the WAS/VWAS [B8.103].
        // Diagnostic only - the model keeps using Config::steerScale.
        float steerScaleEstimate = 1.0f;
        // The wheel angle offset the model APPLIES (delta_eff = steerScale*delta + this).
        // Zero by design since 2026-09-27 - the WAS/VWAS owns that zero - so a non-zero
        // value is a FAULT indicator, not a feature.
        float steerOffsetEstimateDeg = 0.0f;
        // Model-plausibility gate state, logged so a drive can be checked against the
        // replay that derived the limits (see Config::gateFloorDeg).
        bool  modelGateActive = false;
        float modelDevDeg = 0.0f;       // last evaluated window deviation
        float modelChangeDeg = 0.0f;    // model heading change of that window
        // Wheel angle reliability that currently drives the behaviour (see
        // Config::wheelQualityLowPct) and whether the heading is model-only.
        // --- VWAS-Beobachter (2026-10-11, B7.17/B7.21): Werte, die die Bridge fuer Log und
    // Modell braucht. Sie kommen ALLE aus WheelAngleFusion (getZeroObserverStatus,
    // getRawCounts, getEncoderAngle, getGPSAngle, getAog*) - die Bridge rechnet nichts
    // davon selbst nach (ein Eigentuemer, zwei Verbraucher).
    // WICHTIG: der VWAS-Trim ist waehrend aktiver Guidance BLOCKIERT (ZO_BLOCKED, B7.15)
    // -> der GEMELDETE Lenkwinkel traegt seinen Fehler weiter (10...25 Grad in der
    // Akquise, ~5 Grad in 7-01; gemessen B7.20), und genau dieser Wert (residualDeg) ist
    // es, den das Modell intern nutzen darf - ohne Schreiben, ohne EEPROM, ohne Rueckgabe.
    struct WheelObs {
        int32_t encRaw       = 0;     // roher Encoderzaehler (der VWAS-Eingang)
        float   encoderDeg   = 0.0f;  // Winkel VOR dem Offset
        float   offDeg       = 0.0f;  // angewandter encoderOffset (= trimDeg)
        float   residualDeg  = 0.0f;  // beobachteter Rest = der aktuelle Lenkwinkelfehler
        float   gainDegPerDeg = 0.0f; // gelernter/reportierter Sidehill-Gain
        float   gpsAngleDeg  = 0.0f;  // VWAS-eigene Kruemmung atan(curvature*wheelbase)
        float   cpdAog       = 0.0f;  // PGN 252 countsPerDegree von AOG
        float   aogWasOff    = 0.0f;  // PGN 252 wasOffset von AOG (der Nullpunkt, den AOG besitzt)
        float   rollMeanDeg  = 0.0f;  // Roll, auf den der Gain wirkt
        float   acquireRate  = 0.0f;  // aktuelle Trimm-Rate (Grad/s)
        uint8_t zoState      = 0;     // 0 IDLE / 1 ACQUIRE / 2 HOLD / 3 BLOCKED
        uint8_t fits         = 0;     // abgeschlossene Fits dieser Sitzung
        bool    aogZero      = false; // AOG besitzt den Nullpunkt (Smart WAS)
        bool    valid        = false; // mindestens einmal gepusht (sonst sind alle Werte 0)
    };
    WheelObs wheelObs;
    // Der Trim, den das MODELL gerade anwendet (delta_eff = wheel - dieser Wert).
    // Eigene Spalte, damit der Effekt messbar ist; sc_b bleibt der FEHLERINDIKATOR
    // (0 = alles gut, B8.51) und wird davon nicht beruehrt.
    float vwasTrimAppliedDeg = 0.0f;

    float wheelAngleQuality = -1.0f;   // -1 = not reported by the VWAS
        bool  wheelQualityOk = true;
        bool  headingModelOnly = false;

        // Direction of travel (see the operator rule in the config): true while the
        // machine moves backwards, i.e. while the front axis points against the
        // motion. The published heading does NOT flip with it - only the fall-back
        // sources, which measure the direction of travel, get mirrored.
        bool  reverseDirection = false;
        bool  dirChanging = false;        // direction change in progress (AOG's byPass)
        float dirDevDeg = 0.0f;           // filtered angle heading <-> direction of travel
        float dirTravelDeg = 0.0f;        // direction of travel from the position delta
        float dirRefDistM = 0.0f;         // distance the decision was based on
        float dirSigmaM = 0.0f;           // position sigma the threshold was derived from
        // --- Diagnosis of the 2026-09-29 corrections (HR columns 60...68) -------------
        // psiAttDevDeg is the pass-through check: in normal operation psi IS the
        // receiver's own azimuth plus the configured offset, so the difference must stay
        // constant. A swing here is exactly what AgOpenGPS turns into "reverse while
        // driving forward" (its own detection compares the travel direction with the
        // heading it receives, Position.designer.cs:738).
        float   psiAttDevDeg = 0.0f;      // wrap180(psi - attitude body heading)
        float   lastInnovDeg = 0.0f;      // innovation applied this tick (signed, deg)
        float   lastGainK = 0.0f;         // Kalman gain used this tick
        uint8_t lastMeasKind = 0;         // 0=none 1=att 2=path 3=cog 4=imu 5=projected
        uint16_t headingReseeds = 0;      // heading re-seeded because the state was wrong
        uint16_t dirVetoes = 0;           // reverse flips rejected by the attitude witness
        float   posStepRawM = 0.0f;       // fix-to-fix step (m) - what the receiver delivered
        float   posStepPubM = 0.0f;       // step of the PUBLISHED point (m) - what AOG saw
        bool    posLimited = false;       // 1 = the published position was ramped/limited
        // --- Single-antenna support (operator decision 2026-09-29) --------------------
        // The state machine used to hang off "a dual-antenna frame was ever trusted", which
        // kept a single-antenna machine in INIT forever: the VWAS then used the raw COG and
        // NAVProcessor published the receiver's (empty) heading. haveAnyHeading is set by
        // the first ACCEPTED measurement of any kind, and hasBodyAttitude still tells the
        // difference between a real body attitude (dual/INS) and a heading that is
        // anchored to the GPS path.
        bool    haveAnyHeading = false;   // a usable heading exists (any source)
        float   travelWitnessDeg = 0.0f;  // direction of travel from the POSITIONS
        bool    travelWitnessValid = false;
        // B-Kette (2026-10-13, B10.34): Ringpuffer-KURS (Bogen pathChordMinM), mit dem
        // Sehnen-Nachlauf 0,5*yawRate*T korrigiert - Schaetzung der AKTUELLEN Tangente.
        float   pathChordDeg = 0.0f;
        bool    pathChordValid = false;
        float   pathChordLagDeg = 0.0f;    // the 0,5*omega*T that was added (diagnostics)
        float   pathChordRateDegS = 0.0f;  // retroactive course rate over the chord
        // Adaptiv + geprueft (10-13, B10.40): pc_len = die WIRKLICH benutzte Bogenlaenge
        // (0 = kein Kurs), pc_step = der fix-to-fix-Schritt, den die Pruefung gemessen hat.
        float   pathChordLenM = 0.0f;
        float   pathChordStepM = 0.0f;
        bool    pathChordSpike = false;    // der Schritt war zu gross -> GEHALTEN
        // B10.47 (10-18): Sprungtor der Sehne (updatePathChord, Schritt 3b).
        // pathChordJumpDeg = die verworfene Richtungsaenderung in Grad (Diagnose),
        // pathChordJumpRejected = dieser Frame hat wegen des Sprungs gehalten.
        float   pathChordJumpDeg = 0.0f;
        bool    pathChordJumpRejected = false;
        uint8_t rollSource = 0;           // 0 = none, 1 = dual antenna, 2 = IMU, 3 = held
        uint8_t imuFlags = 0;             // bit0 ok/fresh, bit1 heading used, bit2 rate used,
                                          // bit3 rate NOT supported by this IMU (e.g. TM171)
        float   imuRollDeg = 0.0f;        // raw IMU roll (before the bridge LPF)
        uint8_t vwasHeadingSource = 255;  // what the VWAS used last: 0 bridge 1 dual 2 COG
        // Position fall-back (projection published instead of a bad fix) and the
        // attitude settling phase after a reception gap.
        bool     posFallbackActive = false;
        bool     posRampActive = false;   // blending the extrapolation onto the fix
        uint32_t posFallbackMs = 0;       // how long it has been running
        float    posFallbackM = 0.0f;     // how far the machine moved in that time
        uint32_t posFallbackUpdates = 0;  // frames the projection has been published for
        bool     reentrySettling = false; // attitude not yet usable after a gap
        uint32_t reentrySettlingMs = 0;   // how long it has been settling
        // IMU yaw rate support (see Config::useImuRate).
        float    imuRateDegS = 0.0f;      // raw rate used this tick (sign applied)
        float    imuYawBiasDegS = 0.0f;   // learned gyro bias
        bool     imuRateUsed = false;     // true while the rate drives psi
        // IMU heading support (see Config::useImuHeading).
        float    imuHeadingDeg = 0.0f;    // offset-corrected, as used
        float    imuHdgOffsetDeg = 0.0f;  // learned offset (AOG's imuGPS_Offset)
        bool     imuHeadingUsed = false;  // true while it feeds the filter
        uint32_t imuHeadingAgeMs = 0;     // since the offset was last refreshed
        // Output smoothing (see Config::outSmoothMs) - active only in poor phases.
        bool     outSmoothing = false;
        float    outSmoothDeg = 0.0f;
        // ---- Output quality gate (2026-10-02, Config::outQualityGateEnabled) ------
        // attQualGood   = this epoch passed the UNIHEADINGA quality gate (1 = clean,
        //                 0 = degraded, -1 = no per-epoch figure available, i.e. the
        //                 receiver does not report one -> gate open, behaviour unchanged),
        // out_snap      = which path the smoothing took this frame: 0 = rate-limited
        //                 residual (no jump reaches AOG), 1 = snapped (clean epoch, the
        //                 value is passed through without delay), 2 = no residual at all.
        int8_t   attQualGood = -1;
        uint8_t  outSnapSrc = 2;
        float    outResidLimDegS = 0.0f;   // bound applied to the output residual (deg/s)
        // A10 (2026-10-11): gemessenes Rauschen des EINGANGS und das alpha, das diese
        // Frame tatsaechlich benutzt hat - damit die Stufe im Log/Web belegbar ist.
        float    outNoiseDeg = 0.0f;       // Median |d Eingangsheading| ueber outNoiseSamples
        float    outAlphaUsed = 0.0f;      // wirksames alpha der Ausgangsstufe
        // A10-Schicht 1: der physikalische Zaun auf der Ausgabe.
        float    outFenceAllowedDegS = 0.0f;  // Zaunbreite dieser Frame (Grad/s)
        uint8_t  outFenceClip = 0;            // 1 = diese Frame wurde begrenzt
        uint16_t outFenceHoldMs = 0;          // wie lange schon ununterbrochen begrenzt
        // A10/10-11 (Betreiber-Auftrag): ist der LENKWINKEL, aus dem das Modell rechnet,
        // wirklich vertrauenswuerdig? Dieselben Beobachter-Bedingungen wie der Trim
        // (B7.21/B7.34) - EIN Fakt, den Trim UND Ratenvorhalt lesen.
        bool     wheelObsTrusted = false;
        uint8_t  outLimClip = 0;           // 1 when the output residual had to be clipped
        // R1 (2026-10-02): the NET rate clamp on psi (prediction + measurement together).
        bool     psiNetClip = false;       // 1 while it is clipping this tick
        uint16_t psiNetClips = 0;          // counter for the log
        // R8 (2026-10-02): the model wanted to move psi AGAINST a trusted attitude, so its
        // contribution was suppressed this tick. modelSuppressedDegS = the rate that was
        // dropped (0 while the model runs normally).
        bool     modelOpposedAtt = false;
        float    modelSuppressedDegS = 0.0f;
        // W2 (2026-10-02): distance between GNSSData::latitude/longitude (what the raw
        // log carries) and latitudeNMEA/longitudeNMEA (what PAOGI ACTUALLY transmits,
        // NAVProcessor.cpp:288) in metres, or -1 while no NMEA coordinate is cached.
        // Both are written by different parser paths (GNSSProcessor.cpp:660/1447/1538),
        // so a divergence between them reaches AgOpenGPS as a position jump while the log
        // looks perfect - that is exactly the "Kink in der Spur" report of 2026-10-02.
        float    nmeaDiffM = -1.0f;
        // Steering telemetry fed in by the caller (AutosteerProcessor), so the raw log
        // can show the whole chain heading -> AOG target angle -> PWM. Added
        // 2026-09-27 after the operator reported "wild steering" with flickering
        // headings: without these three the complaint stays subjective.
        float    aogTargetDeg = 0.0f;
        int16_t  pwmOut = 0;
        bool     guidanceActive = false;

        uint32_t framesSeen = 0;
        uint32_t framesRejected = 0;
        uint32_t events = 0;
        uint32_t cogUpdates = 0;
        uint32_t fixToFixUpdates = 0;
        float frameIntervalMs = 0.0f;     // measured attitude sentence period
        uint32_t gapThresholdMs = 0;      // 3 x the measured period, clamped
        uint32_t posFaults = 0;           // spikes / HDOP / innovation rejects
        float posStepM = 0.0f;            // last step that failed the test
        // P4.1 (2026-09-30): the RECEIVER's own fix-to-fix step is discontinuous. Set by
        // the position window's spike test (independent of the projection), read by the
        // position limiter so the catch-up RAMP cannot blind it.
        bool posSpikeFix = false;
        uint32_t posRampAborts = 0;       // catch-up ramps aborted by an impossible fix
        float pathRangeM = 0.0f;          // net displacement of the window
        float pathStraightness = 1.0f;    // net / arc of the window
        // Last COG / fix-to-fix values, reported even while stale, so the log
        // line always shows what the fall-back sources last said.
        float cogLastDeg = 0.0f;
        float pathLastDeg = 0.0f;

        bool attitudeTrusted = false;
        // True once the filter has been seeded from a REAL measurement (the first
        // trusted attitude frame). Before that the heading is a boot default, and
        // AgOpenGPS must not be fed that constant: measured on the 2026-09-27 14-12
        // drive, 46 s of INIT published 270 deg while the machine was already driving.
        bool everTrusted = false;
    };

    void init();

    // Geometry from AgOpenGPS (vehicle.xml via the VWAS config). Called by the
    // owner of the VWAS so there is exactly ONE wheelbase in the system - a
    // second copy here would silently disagree after a vehicle import.
    void setWheelbaseM(float m) {
        if (m > 0.5f && m < 20.0f) config.wheelbaseM = m;
    }

    // ---- Antenna spacing for the baseline-length plausibility test -------------
    // The measured distance between the two antenna phase centres. Sets the value at
    // runtime AND persists it to the EEPROM, so it survives a reboot and becomes the
    // reference of the baseline-length test from the first frame on.
    //
    // Passing 0 (or anything outside 0.1 ... 5 m) CLEARS it, which returns to learning
    // the reference from the receiver - the fall-back behaviour, not the wanted one.
    //
    // This is the single entry point for the web UI field. Measured on this machine:
    // 0.711 m (15 logs, 13 065 RTK-fixed frames, per-session medians 0.706 ... 0.719 m).
    void setPhysicalBaselineM(float m);
    float getPhysicalBaselineM() const { return config.physicalBaselineM; }
    bool  isPhysicalBaselineSet() const { return config.physicalBaselineM > 0.1f; }

    // ---- Dual-antenna heading offset, the SAME number AgOpenGPS carries --------
    // Sets the value at runtime AND persists it in the EEPROM, so it survives a reboot.
    // The firmware uses it in both directions (body = azimuth + offset, published =
    // psi - offset), which is why the two sides have to use the SAME value - the page says
    // so as well.
    // Accepts -180 ... +180 deg (0 is a legal offset). Anything outside is rejected and the
    // current value is kept.
    // Measured on this machine: 88.3 deg (offset_check.ps1, 3 logs / 520 samples, sd 0.9).
    void  setDualHeadingOffsetDeg(float deg);
    float getDualHeadingOffsetDeg() const { return config.dualHeadingOffsetDeg; }

    // Called from the 100 Hz autosteer loop. `wheelAngleDeg` is the same angle
    // the VWAS/AgOpenGPS work with (already AOG referenced). `speedMps` may be
    // left at its default: a negative value means "use the GNSS velocity", which
    // is what this class reads for everything else anyway. Pass a CAN or wheel
    // speed explicitly to use a different source.
    void update(float dtS, float wheelAngleDeg, float speedMps = -1.0f);

    // ---- outputs ------------------------------------------------------------
    // Body heading of the vehicle (0..360, true north).
    float getVehicleHeadingDeg() const { return status.vehicleHeadingDeg; }
    // --- Position fall-back (see Config::posFallbackMaxMs) ---
    // True while the published position comes from the forward projection instead of
    // the receiver, because the fix is physically impossible, below RTK fixed or
    // stale. The caller (NAVProcessor::formatPAOGIMessage) then sends this position;
    // the fix quality and all other fields stay the receiver's own.
    bool isPositionFallbackActive() const { return status.posFallbackActive; }
    // Steering telemetry for the raw log (the chain heading -> AOG target -> PWM).
    void setSteerTelemetry(float targetDeg, int16_t pwm, bool guidance) {
        status.aogTargetDeg = targetDeg;
        status.pwmOut = pwm;
        status.guidanceActive = guidance;
    }
    // Wheel angle reliability from the VWAS (0...100, -1 = not reported). Called by
    // AutosteerProcessor on the same tick as update(). See Config::wheelQualityLowPct
    // for what the bridge does with it: below the low threshold the model gate is off
    // and the wheel angle bias state is allowed to absorb an unknown zero (the Keya
    // sets its zero at power-up), above the high threshold the VWAS owns the zero and
    // the gate protects the model.
    void setWheelAngleQuality(float q) { pendingWheelQuality = q; }
    // VWAS-Beobachter durchreichen (10-11): der Aufrufer (AutosteerProcessor) hat den
    // Zugriff, die Bridge nicht - dasselbe Muster wie setWheelAngleQuality().
    void setWheelObserver(const Status::WheelObs& w) { status.wheelObs = w; }
    // What the VWAS used as its heading source this cycle: 0 = bridge, 1 = dual attitude,
    // 2 = course over ground (log-only, HR column vwas_hsrc). Set by WheelAngleFusion.
    void setVwasHeadingSource(uint8_t s) { status.vwasHeadingSource = s; }
    bool getFallbackPosition(double& lat, double& lon) const {
        // Also serves the ramp: while the extrapolation is blended onto a recovered
        // fix the projected position is still the one that must be published.
        if ((!status.posFallbackActive && !status.posRampActive) || !projValid) return false;
        lat = projLat; lon = projLon;
        return true;
    }

    // The value that belongs into PAOGI field 12: the inverter of the conversion
    // above, i.e. what AgOpenGPS has to receive for its own "dual heading offset"
    // to restore the body heading. In normal operation this is the receiver's own
    // azimuth - see update().
    float getAzimuthDeg() const { return status.azimuthDeg; }
    // Geschwindigkeit fuer PAOGI Feld 11 (2026-10-14): roh, solange der Empfaengerwert
    // zu Weg/Zeit passt - im Zweifel der geglaettete Wert (Betreiber-Regel "nach aussen
    // roh bei gutem Empfang"). Knoten, weil PAOGI Knoten traegt.
    float getSpeedKnotsForOutput(float rawKnots) const {
        if (!config.enabled || !config.exposeToAog || !status.speedDoubt) return rawKnots;
        return status.speedFusedMps / 0.514444f;
    }
    float getRollDeg() const { return status.rollDeg; }
    // Current 1 sigma position uncertainty (1 sigma, metres). The SAME figure the direction
    // of travel threshold is derived from (HeadingBridge::positionSigmaM, the fix-type table),
    // exposed so PositionHistoryService can store it without duplicating that table - one
    // formula, one owner (2026-10-04).
    float getPositionSigmaM() const { return positionSigmaM(); }
    // 1 sigma of the current heading estimate. Consumers able to weight
    // (WheelAngleFusion) should use it instead of a hard gate.
    float getHeadingSigmaDeg() const { return status.sigmaDeg; }
    // Side slip the module currently removes from the path derived measurements
    // (COG / fix-to-fix) to turn them into body headings.
    float getSideSlipDeg() const { return sideSlipDeg; }
    // Speed scale learned from the projection's along-track residual (1.0 = the
    // receiver's speed is taken as it is). Applied to the kinematic MODEL only.
    float getSpeedScale() const { return vScale; }
    // Measurement noise currently used for the attitude (adaptive estimator).
    float getEffectiveRAttDeg() { return effectiveMeasurementR(0); }
    bool isHeadingUsable() const { return status.state != State::TIMEOUT; }
    bool isAttitudeTrusted() const { return status.attitudeTrusted; }
    bool isBridging() const {
        return status.state == State::BRIDGE || status.state == State::COAST;
    }
    State getState() const { return status.state; }
    const Status& getStatus() const { return status; }

    Config& getConfig() { return config; }
    const Config& getConfig() const { return config; }

    static const char* stateName(State s);
    static const char* sourceName(Source s);
    static const char* reasonName(RejectReason r);

    // B10.54 (2026-10-25): die Zusatzwerte, die NUR im Log stehen (kein Status-/Config-Feld -
    // ein Feld dort hat im Sim die Config-Defaults zerstoert, s. B10.50). Damit kann der
    // Simulator (tools/sim/hr_writer.h) GENAU dieselben Spalten schreiben wie die Firmware.
    struct LogExtra {
        uint8_t pcRej;    // warum keine Sehne (0 ok ... 8 Sprungtor)
        float   pcDev;    // Abweichung des Sprungtors
        float   antLead;  // angewandter Antennen-Vorlauf
        float   artHalf;  // angewandte Knick-Korrektur gamma/2
        uint8_t vehType;  // 0 = Ackermann, 2 = Knicklenker
        float   yrRaw;    // ROHE Modellrate (vor dem P1b-Gate)
    // B10.58 (2026-10-25): rms-Residuum der Sehnenausgleichung in Grad (0 = nicht messbar).
    // Positionsqualitaet, die der Empfaenger nicht selbst meldet (B10.57) - reine Diagnose.
    float   crResid;
    };
    LogExtra logExtra();

    // Suppresses the 2 Hz diagnostic line for this module instance.
    void setQuiet(bool q) { quiet = q; }

private:
    Config config;
    Status status;
    bool   quiet = false;

    // Attitude frame detection
    uint32_t lastHprCount = 0;
    uint32_t lastTrustedMs = 0;
    bool     haveTrusted = false;
    float    lastTrustedVehHeading = 0.0f;  // vehicle heading of the last trusted frame
    float    lastRoll = 0.0f;
    bool     haveLastRoll = false;
    uint32_t lastFrameMs = 0;
    // Hysteresis: one bad frame is absorbed by the filter (its measurement is
    // simply not applied), only the second consecutive one switches the state
    // machine. Otherwise a single 100 ms artefact would toggle BRIDGE/LIVE.
    uint8_t  consecutiveRejects = 0;

    // Cross-check inputs, refreshed by update() from the newest GNSS data
    float    lastSpeedMps = 0.0f;
    // Geschwindigkeit aus Weg/Zeit (2026-10-14): EMA-Zustand + Zweifels-Timer.
    float    speedEmaState = 0.0f;
    bool     speedEmaInit = false;
    uint32_t speedEmaMs = 0;
    uint32_t speedDoubtSinceMs = 0;
    // Roll-Bestaetigung (2026-10-18): 40 Slots = 0,4 s bei 100 Hz; gesucht wird der
    // JUENGSTE Sample mit Alter >= rollConfirmMs - der Haltewert sieht die letzten
    // 300 ms damit nicht, die Ausgabe bleibt live.
    static const uint8_t RollPendN = 40;
    float    rollPendVal[RollPendN] = {0};
    bool     rollPendOk[RollPendN] = {false};
    uint32_t rollPendMs[RollPendN] = {0};
    uint8_t  rollPendIdx = 0;
    // Quellen-Bestaetigung (Heading): laufender Kandidat + Startzeit.
    uint32_t srcPendingMs = 0;
    // MA-Ring (2026-10-18): 64 Ticks zu 10 ms = 0,64 s, genug fuer 500 ms Fenster.
    static const uint8_t OutMaN = 64;
    float    outMaBuf[OutMaN] = {0};
    uint8_t  outMaIdx = 0;
    uint16_t outMaCount = 0;
    // Roll-Halten (2026-10-06): Mittelwert der letzten GUTEN Meter + die dafuer
    // gefahrenen Meter. Kein Ring noetig - ein distanzgewichtetes Fenster wirkt wie
    // "Durchschnitt der letzten rollGoodAvgWinM Meter" und zieht beim Wiedereintritt
    // von selbst weich nach (kein eigener Blend).
    float    rollGoodAvgDeg = 0.0f;
    bool     rollGoodAvgInit = false;
    float    rollLossM = 0.0f;
    // M1-Anker: das Step-Gate darf NICHT mit dem verkleinerten R rechnen (siehe
    // effectiveMeasurementR), sonst verwirft es die Frames, mit denen psi nachziehen soll.
    bool     rAnchorSuppress = false;
    // IMU-Roll-Plausibilitaet (2026-10-06): EMA der Abweichung Dual<->IMU + Verdict.
    float    imuRollDevDeg = 0.0f;
    bool     imuRollDevInit = false;
    bool     imuRollPlausible = false;
    // P2 (2026-09-30): the tick length and the wheel angle of the current call, so
    // applyMeasurement() can bound its correction (it is called from several places
    // inside update() and has no access to the parameters).
    float    lastDtS = 0.01f;
    float    lastWheelAngleDeg = 0.0f;
    // ---- P2b (2026-10-02): the ATTITUDE's own period, measured from the DtS of the
    // ticks that actually carried a trusted attitude frame (frameIntervalEmaMs only
    // tracks HPR ARRIVAL, not acceptance). It is the time base the rate limiter must
    // use for an attitude measurement; the 100 Hz tick (lastDtS = 0.01 s) is 10x too
    // short and made the limiter 10x tighter than documented (see applyMeasurement).
    // -1 = no attitude measurement seen yet -> fall back to frameIntervalEmaMs.
    float    attMeasPeriodS = -1.0f;
    // Timestamp of the last applied ATTITUDE measurement - the interval between two of
    // them is the rate limiter's time base for kind 0 (see applyMeasurement).
    // Interval base per measurement kind (2026-10-05, "D2"): [0] attitude, [1] path/fix-to-fix,
    // [2] COG, [3] IMU heading. applyMeasurement() takes the interval since the LAST APPLICATION
    // OF THAT KIND from here, because every kind is applied at the rate of its own source, never
    // on the 100 Hz tick. Replaces the single lastAttMeasMs.
    uint32_t lastMeasMsByKind[4] = {0, 0, 0, 0};
    // R4: the receiver's measured yaw rate between two TRUSTED attitude frames (deg/s) and
    // whether it is currently usable (the interval must be 0.02...1 s).
    float    attMeasRateDegS = 0.0f;
    bool     attMeasRateValid = false;
    // R1: psi was deliberately JUMPED this tick (cold start or re-seed) - the net rate
    // clamp must not undo that.
    bool     psiJumpedThisTick = false;
    // Output smoothing state (2026-10-02): the residual the rate limiter is applied to.
    float    outResidDeg = 0.0f;
    // Snap confirmation on a degraded epoch (Config::outSnapConfirmFrames): the sign and
    // size of the residual that is still waiting to be confirmed, how many consecutive
    // frames it has been asking, and since when it is being held.
    int8_t   outPendSign = 0;
    float    outPendMag = 0.0f;
    uint8_t  outPendCount = 0;
    uint32_t outHoldSinceMs = 0;
    // Turn rate of the last ACCEPTED fix-to-fix window (deg/s), decayed towards 0 so
    // a stale window cannot keep the rate limit open. Feeds Config::rateLimitPathFactor.
    float    pathRateDegS = 0.0f;
    float    lastPathMeasDeg = 0.0f;
    uint32_t lastPathMeasMs = 0;
    // P2 persistence: since when (and in which direction) the measurement has been
    // asking for more than the bound - see Config::rateLimitPersistMs.
    uint32_t rateLimitDevSinceMs = 0;
    float    rateLimitDevSign = 0.0f;
    // F4: set for the frame that was rescued by the stuck detector; effectiveMeasurementR
    // multiplies the attitude R by Config::stuckRescueRFactor while it is set.
    bool     stuckRescueActive = false;
    bool     cogValid = false;
    float    cogMeas = 0.0f;
    bool     pathValid = false;
    float    pathMeas = 0.0f;      // direction of travel from the position stream

    // Filter state (heading + effective wheel-angle bias)
    float    psi = 0.0f;
    float    steerBiasDeg = 0.0f;
    float    p11 = 0.0f, p12 = 0.0f, p22 = 0.0f;

    // Position history for fix-to-fix (exactly two points plus the last seen fix
    // for the step test)
    double   refLat = 0.0, refLon = 0.0;
    uint32_t refMs = 0;
    bool     haveRef = false;
    float    refArcM = 0.0f;          // accumulated |step| since the reference
    double   lastPosLat = 0.0, lastPosLon = 0.0;
    uint32_t lastPosMs = 0;
    // TWO independent fix stamps (2026-09-23): updatePositionWindow() and
    // updateProjection() both consume position fixes, and while they shared ONE
    // lastPosStamp the first one consumed it and the second one never saw a new
    // fix - measured: one resync per drive instead of 10x/s, plus 248 m intervals
    // when one slipped through. They compare gnssProcessor.getPositionCount() with
    // their own copy now, which cannot be consumed.
    // Last wheelbase announced in the log. The start-up line in init() prints the
    // configured value, but the real one (VWAS/AOG) only arrives with the first
    // VWAS tick - without this the log reads as if the module kept calculating with
    // the default (measured 2026-09-23: "HeadingBridge ENABLED: L=2.50m" while the
    // VWAS config held 2.14 m).
    float    loggedWheelbaseM = -1.0f;

    uint32_t lastWindowFixCount = 0;  // used by updatePositionWindow()
    uint32_t lastProjFixCount = 0;    // used by updateProjection()
    uint32_t lastRingFixCount = 0;    // Step C: one ring path measurement per fix
    bool     haveLastPos = false;
    // Direction-of-travel filter state (AOG's 0.2/0.8 EMA on the angle between the
    // heading and the path). dirDevInit keeps the first sample from being averaged
    // against a zero start, which would look like a direction change.
    float dirDevEma = 0.0f;
    bool  dirDevInit = false;
    void updateDirectionState(uint32_t now);
    // Steering-scale collection window: the accumulated geometric turn and the
    // heading it started from, so the estimator compares over a window that is long
    // enough to carry a signal (see applyMeasurement).
    bool     scaleCollecting = false;
    float    scaleStartPsi = 0.0f;
    uint32_t scaleStartMs = 0;
    // Reference point of the distance-based travel direction (AOG's
    // dualReverseDetectionDistance idea): the decision is only made once the machine
    // has moved far enough for the position delta to mean something.
    double   dirRefLat = 0.0;
    double   dirRefLon = 0.0;
    bool     dirRefValid = false;
    uint32_t dirRefMs = 0;            // when the reference point was taken
    uint8_t  revVotes = 0;            // consecutive evaluations asking for a switch
    // --- standstill re-seed for the direction decision (2026-10-03) --------------
    // A K-turn is slow and short. Measured 2026-10-03: the real reversal ran at 0.93 m/s for
    // 2.0 ... 3.0 s, so for most of it the machine was BELOW Config::minSpeedForDirection and
    // the whole direction logic returned early - and where it did run, the chain needs ~11
    // evaluations (the 0.2 EMA needs 9 steps before dirChanging clears, plus 2 votes) while
    // the manoeuvre offers only ~5. The reversal was never detected, and the state stayed
    // exposed to the mirrored path measurements: measured |psi - attitude| up to 68 deg and
    // 46 s until it settled again.
    // A stop is the one moment where the verdict can be re-derived from scratch, and that
    // branch already exists for the power-on case (!dirDevInit). standstillSinceMs is when the
    // machine last dropped below Config::standstillSpeedMps; once it has stood for
    // Config::standstillMinMs, the next evaluation re-seeds instead of ramping.
    // Simulated on 16 logs before it was built (tools/analysis/revfix.ps1, which models the
    // emergency clear and the travelSolid veto for BOTH variants so the comparison is fair):
    // detected truth-reverse frames 120 -> 282, missed 308 -> 146, false 233 -> 142, and
    // ZERO false reversals in the 11 reversal-free logs. The false windows that remain are
    // RELEASE delays of 0.4 ... 5.0 s right after a real manoeuvre - the old behaviour has
    // them too - and not spurious latches during normal driving.
    uint32_t standstillSinceMs = 0;
    bool     dirReseedPending = false;
    bool     scaleStartRev = false;   // direction at window start, see applyMeasurement
    // Keeps the state machine from following a flickering fall-back source.
    bool     pathUsableLatch = false;
    uint32_t pathUsableFalseMs = 0;
    // Re-entry ramp: true while the attitude was trusted at the end of the last tick.
    bool     prevTrustedLatch = false;
    uint32_t reentryUntilMs = 0;
    // Attitude settling after a gap (see Config::reentrySettleMs): how long the frame
    // sequence has been consistent, and the last values it is compared against.
    uint32_t settleGoodMs = 0;
    uint32_t settleStartMs = 0;
    uint32_t settleGoodSinceMs = 0;
    float    settleLastRoll = 0.0f;
    float    settleLastAtt = 0.0f;
    bool     settleHaveLast = false;
    // Position fall-back bookkeeping (see Config::posFallbackMaxMs).
    uint32_t lastProjFixMs = 0;
    uint32_t posFallbackStartMs = 0;
    float    posFallbackStartOdoM = 0.0f;
    uint32_t posRtkGoodMs = 0;      // how long fixQuality 4 has been holding
    uint32_t posRampStartMs = 0;    // start of the position ramp
    // Position continuity (operator correction 2026-09-29): the point that was actually
    // published for AgOpenGPS, so the next fix can be judged against what AOG saw - not
    // against the projection, which may already be ahead of it.
    double   pubLat = 0.0, pubLon = 0.0;
    bool     pubValid = false;
    // (2026-11-02) lastFixLat/lastFixLon/lastFixValid sind entfallen: der
    // Fix-zu-Fix-Schritt (raw_step) wird jetzt in logRawFrame() aus d.latitude/
    // longitude gegen den Stand der vorigen Log-Zeile gebildet.
    // Ramp time actually used by the running catch-up (see posMaxCatchUpMps).
    uint32_t posRampMsActive = 500;
    // Attitude witness for the direction decision (never mirrored): the body heading the
    // dual antenna reported in the last trusted frame.
    float    attWitnessDeg = 0.0f;
    bool     attWitnessValid = false;
    bool     outSmoothInit = false; // output smoothing state
    // A10 (2026-10-11): Ringpuffer der letzten |d Eingangsheading| (Median) und die
    // traege nachgefuehrte Stufe. 8 Werte = 32 B, kein Heap, keine Latenz im Ausgang.
    float    outNoiseHist[8] = {0};
    uint8_t  outNoiseCount = 0;
    uint8_t  outNoiseIdx = 0;
    float    outNoisePrevAz = 0.0f;
    bool     outNoiseInit = false;
    float    outNoiseAlpha = 0.5f;   // wirksame Stufe, wandert begrenzt schnell
    // A10-Schicht 1: Vorhalt der letzten Frame (der Zaun begrenzt seine AENDERUNG).
    float    outLeadPrevDeg = 0.0f;
    float    receptionQualityFactor();
    // Ramp time for a position residual, chosen so the blend's INITIAL rate equals the
    // allowed catch-up speed (Config::posMaxCatchUpMps) and never a step (see the note
    // there, operator correction 2026-09-29).
    uint32_t catchUpRampMs(float residualM) const;
    // --- single-antenna support (operator decision 2026-09-29) -------------------
    // Latched when any measurement was accepted; drives updateState()'s INIT gate.
    bool     haveAnyHeadingLatch = false;
    // When a PATH/COG measurement was last accepted - the anchor the IMU heading offset
    // may be learned against when there is no dual attitude (see Config::imuAnchorHoldMs).
    uint32_t imuAnchorMs = 0;
    // Direction-of-travel witness from the POSITIONS (independent of every heading
    // source). Window bookkeeping: the reference point, its time and the heading there.
    double   twRefLat = 0.0, twRefLon = 0.0;
    uint32_t twRefMs = 0;
    float    twRefHeadingDeg = 0.0f;
    bool     twRefValid = false;
    // B-Kette (2026-10-13, B10.34): letzter Ringpuffer-Kurs fuer die rueckwirkende Rate.
    float    pathChordPrevDeg = 0.0f;
    uint32_t pathChordPrevMs = 0;
    bool     pathChordPrevValid = false;
    // B10.47: Referenz des Sehnen-Sprungtors (zuletzt UEBERNOMMENE Richtung, Body-Frame)
    float    chordGateDeg = 0.0f;
    uint32_t chordGateMs = 0;
    bool     chordGateValid = false;
    uint32_t chordGateRejectMs = 0;   // erste Ablehnung der laufenden Sprungserie
    // Update the position based travel witness (called from updateDirectionState).
    void updateTravelWitness(uint32_t now);
    // When the current direction verdict was last re-confirmed by a solid measurement
    // (see Config::revReconfirmMs).
    uint32_t revConfirmedMs = 0;
    // IMU yaw-rate state: the learned gyro bias and the heading it is learned from.
    float    yawBiasDegS = 0.0f;
    float    psiPrevForBias = 0.0f;
    bool     psiPrevValid = false;
    bool     yawBiasWarned = false;
    // IMU heading state (see Config::useImuHeading).
    float    imuHdgOffsetDeg = 0.0f;
    bool     imuHdgOffsetInit = false;
    uint32_t imuHdgFreshMs = 0;       // last time the offset was refreshed
    bool     imuHdgHaveFresh = false;
    float    imuHeadingMeasDeg = 0.0f;
    float    imuHeadingMeasR = 0.0f;
    float    positionSigmaM() const;

    // Measured mounting offset (circular mean)
    float    mountOffsetSin = 0.0f;
    float    mountOffsetCos = 1.0f;

    // Diagnostic steerScale estimator. It estimates the SCALE ALONE, solved through the
    // origin (s = 1 + sumY1/scaleAccXX), because that is all the data supports:
    //   * the wheel angle ZERO belongs to the WAS/VWAS (operator decision 2026-09-27),
    //     so the bridge must not estimate it,
    //   * and the earlier two-parameter form (scale AND offset via the operator's
    //     derivative column Q) was ILL-CONDITIONED: X and Q are strongly correlated, so
    //     the solution is a difference of two nearly equal large numbers and it walked to
    //     its clamps (measured 14-17: sc 2.000 / sc_b +6.00).
    // sc_b (the offset the model APPLIES) therefore stays 0 by design and is a FAULT
    // indicator, not a feature. [B8.103]
    float    scaleAccX = 0.0f;      // geometric heading change since the last measurement
    float    scaleStartRawDeg = 0.0f; // RAW attitude at the window start (not psi)
    // DOUBLE on purpose: the float sums of a long drive lose the difference the
    // solution consists of (measured 2026-09-27: the estimate walked to its clamp).
    double   scaleAccXX = 0.0;      // sum X*X   (with forgetting)
    double   scaleY1 = 0.0;         // sum X*innov

    // Model-plausibility gate: its own short window (GateWindowMs), accumulated from
    // the same per-frame increments but reset on its own schedule.
    float    gateAccX = 0.0f;
    float    gateAccQ = 0.0f;
    float    gateStartRawDeg = 0.0f;   // RAW attitude at the window start, not psi
    uint32_t gateStartMs = 0;
    uint32_t modelGateUntilMs = 0;     // the gate holds R elevated until here
    // Wheel angle reliability from the caller (see setWheelAngleQuality) and the state
    // it drives. -1 = not reported -> behave as before (gate on, bias off).
    float    pendingWheelQuality = -1.0f;
    bool     wheelQualityOkState = true;

    // Adaptive measurement noise (innovation variance per source)
    float    rAttEmaSq = 25.0f;     // start at (5 deg)^2
    float    rPathEmaSq = 64.0f;    // start at (8 deg)^2

    // COG circular average history
    float    cogHistSin[8] = {0};
    float    cogHistCos[8] = {0};
    uint8_t  cogHistIdx = 0;
    uint8_t  cogHistCount = 0;

    // ---- forward projection (position predicted from psi + wheel angle) -----
    // See updateProjection(): the predicted position is integrated at 100 Hz with
    // the SAME kinematics that drives psi, so the wheel angle is in it. The
    // cross-track residual against the next fix is a heading measurement that has
    // no chord lag (unlike fix-to-fix) and no dependence on the course over
    // ground (unlike the COG).
    double   projLat = 0.0, projLon = 0.0;      // running predicted position
    double   projRefLat = 0.0, projRefLon = 0.0; // position at the last resync
    float    projPathM = 0.0f;                  // path length since the last resync
    bool     projValid = false;
    float    projCtkM = 0.0f;                   // last cross-track residual
    float    projLongM = 0.0f;                  // last along-track residual
    float    projDistM = 0.0f;                  // distance of the last interval
    uint32_t projSamples = 0;
    float    projRmsCtkM = 0.0f;                // EMA of the residual, diagnostics

        // Side slip (path vs body) estimate, in degrees
    float    sideSlipDeg = 0.0f;
    uint32_t sideSlipSamples = 0;

    // Speed scale from the projection's ALONG-track residual. The projection
    // integrates v*dt along psi; if what it integrates is longer than the path the
    // receiver actually measured, the speed it used was too high. That matters for
    // accuracy because the kinematic yaw rate is proportional to v (omega =
    // v*tan(delta)/L): a 3 % speed error is a 3 % heading-rate error, i.e. it shows
    // up directly in psi during every bridging phase. Applied to the MODEL only
    // (prediction, projection, odometer) - the speed sent to AgOpenGPS in PAOGI
    // stays the receiver's own value, untouched.
    float    vScaleEma = 1.0f;
    float    vScale = 1.0f;
    uint32_t vScaleSamples = 0;

    // Time of the last usable measurement of ANY kind (attitude/path/COG)
    uint32_t lastMeasurementMs = 0;
    float    lastMeasurementOdometerM = 0.0f;

    // Raw frame log pacing
    uint32_t lastRawLogMs = 0;

    // Diagnostic log pacing / event forcing. status.posFaults is a cumulative
    // counter, so comparing it with the value at the last log line is what makes
    // "a NEW position fault happened" visible instead of firing on the stale one
    // forever (the 2026-09-23 logs show HB2 on every single HB1 because of that).
    uint32_t lastLoggedPosFaults = 0;
    bool     forceLog = false;

    // Measured attitude sentence period -> dynamic gap threshold (UM982 can run
    // at 20 Hz, F9P at 10 Hz; a fixed 250 ms would flag every second frame as a
    // gap at 20 Hz and would be too slow to notice a gap at 5 Hz).
    float    frameIntervalEmaMs = 100.0f;
    uint32_t gapThresholdMs = 250;

    // Roll channel
    float    rollF = 0.0f;
    uint32_t rollLossMs = 0;
    bool     rollInit = false;

    // Event bookkeeping
    bool     inEvent = false;
    uint32_t eventStartMs = 0;
    float    eventStartOdometerM = 0.0f;
    float    lastTrustedOdometerM = 0.0f;
    float    odometerM = 0.0f;
    bool     timeoutLogged = false;
    uint32_t lastLogMs = 0;

    void evaluateAttitudeFrame(uint32_t now);
    // kind: 0 = attitude measurement, 1 = path-derived (COG / fix-to-fix),
    //       2 = forward-projection residual
    // dtRefS: the INTERVAL this measurement belongs to, used as the time base of the
    // rate limiter. It MUST match the update rate of the source: ~0.1 s for the 10 Hz
    // attitude, the 100 Hz tick (0.01 s) for COG/fix-to-fix/projection. Passing the
    // tick length for the attitude was the defect found on 2026-10-02 (R2). 0 = derive
    // (the attitude interval is measured inside applyMeasurement from the last update).
    void applyMeasurement(float measDeg, float rDeg, uint8_t kind = 0, float dtRefS = 0.0f);
    float effectiveMeasurementR(uint8_t kind);
    // Per-epoch attitude quality from the receiver's own UNIHEADINGA figures
    // (0 = clean, 1 = degraded, 2 = no figure available -> treated as clean).
    uint8_t attitudeQualityNow() const;
    // Yaw rate the machine can physically produce this tick (deg/s) - the bound the
    // MEASUREMENT limiter and the OUTPUT limiter both use (kinematics + margin,
    // raised by an accepted fix-to-fix turn).
    float kinematicRateBoundDegS(bool includeRaises = true) const;
    void updateRoll(float dtS, uint32_t now, bool attitudeTrusted);
    // Sidehill/Draft im MODELL (2026-10-06, Betreiber: "wir nutzen den gleichen Hebel
    // anders herum"). AgOpenGPS addiert fuer die Hangdrift Lenkwinkel auf den SOLL-Winkel;
    // dieser Anteil erzeugt keine Gierrate (er wird im Reifenschlaupf verbraucht). Fuer
    // omega = v*tan(delta)/L muss er also heraus - aber NUR hier: der nach aussen
    // gemeldete Radwinkel (PAOGI/Anzeige) bleibt unveraendert.
    // Der Roll ist der bridged Roll (status.rollDeg, mit Halten/Fade), also derselbe,
    // den auch WheelAngleFusion/AOG sehen -> eine Konvention fuer alle.
    float sideHillWheelDeg(float wheelAngleDeg) const {
        if (!config.sideHillInModel) return wheelAngleDeg;
        const float k = config.sideHillGainDegPerDeg;
        const float r = status.rollDeg;
        const float t = (config.sideHillGainSignFlip ? -k : k) * r;
        // Betrag ist per Konstruktion <= |k*roll|; keine weitere Klemme noetig.
        return wheelAngleDeg - t;
    }
    void updatePositionWindow(uint32_t now, float speedMps);
    // Returns true when the projection supplied the measurement for this tick
    // (then the caller must not run selectFallbackMeasurement as well).
    bool updateProjection(uint32_t now, float speedMps, float dtS, bool allowMeasurement);
    // Step C (2026-10-05): fix-to-fix measurement from the position ring, once per fix.
    bool updatePathFromRing(uint32_t now, float speedMps);

// B-Kette (2026-10-13, B10.34): Kurs aus dem Ringpuffer ueber eine Bogenlaenge, mit
// dem Sehnen-Nachlauf 0,5*yawRate*T korrigiert. Quelle fuer Zustand B (Lage nicht
// vertrauenswuerdig, Positionen gut) - ersetzt dort den Gerade-Te"st des Witness.
void updatePathChord(uint32_t now, float speedMps, float yawRateDegS);
    // Die EINE Schrittregel (Betreiber 10-13: "gleiche Schwellen") - Fenster UND B-Kette.
    float maxPosStepAllowedM(float speedMps, float dtS) const;
    // Geschwindigkeit aus Weg/Zeit + EMA (2026-10-14). Setzt status.speedPosMps/
    // speedFusedMps/speedRatio/speedDoubt und gibt den INNEN benutzten Wert zurueck.
    float updateFusedSpeed(uint32_t now, float rawMps);
    // Die erste (NEUESTE) Ringprobe >= minDistM, gueltig und jung genug. backOut = Abstand
    // in Fixes (1 = der vorige Fix), distM/arcM = Sehne und Weglaenge des eigenen Wegs.
    bool findChordSample(uint32_t now, double lat, double lon, float minDistM,
                         uint16_t& backOut, float& distM, float& arcM) const;
    void selectFallbackMeasurement(uint32_t now);
    void updateMountOffset(uint32_t now, float speedMps);
    void updateState(uint32_t now);
    void updateSteerScaleCalibration(uint32_t now);
    void logDebugLine(uint32_t now, float speedMps);
    void logRawFrame(uint32_t now, float speedMps, float wheelAngleDeg);
};

extern HeadingBridge headingBridge;

#endif // HEADING_BRIDGE_H
