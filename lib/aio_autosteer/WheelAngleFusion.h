// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// WheelAngleFusion.h - Virtual WAS (VWAS) using sensor fusion
#ifndef WHEEL_ANGLE_FUSION_H
#define WHEEL_ANGLE_FUSION_H

#include <Arduino.h>
#include "EventLogger.h"

// Math constants are already defined in Arduino.h
// DEG_TO_RAD and RAD_TO_DEG

// Forward declarations
class MotorDriverInterface;
class GNSSProcessor;
class IMUProcessor;

/**
 * WheelAngleFusion - Virtual Wheel Angle Sensor (VWAS)
 * 
 * Creates a virtual WAS by combining multiple sensor inputs to estimate 
 * steering angle. Uses an adaptive Kalman filter to fuse motor encoder 
 * data with GPS/INS heading rate.
 * 
 * This provides a software-based wheel angle sensor using existing hardware.
 * Based on proven algorithm from AOG_Teensy_UM98X project.
 */
class WheelAngleFusion {
public:
    // Configuration structure
    struct Config {
        // Vehicle parameters
        // 2026-11-07 (Betreiber, NEUE Vorgabe): Default = "normale Traktor-Werte".
        // ACHTUNG - das kehrt die Entscheidung vom 2026-11-02 um (siehe unten), bewusst:
        // die Fahrzeugseite /vwas ist seit 2026-11-07 auch OHNE Motor erreichbar
        // (AutosteerProcessor laesst das VWAS-Objekt am Leben), also koennen die Werte der
        // Maschine eingegeben bzw. per vehicle.xml importiert werden. Der Maeher BRAUCHT
        // 92 cm / Typ 2 / 75 Grad - das sind seine WERTE, nicht die eines frischen Boards.
        //
        // Der alte Text (2026-11-02, gleicher Betreiber) lautete: "in ALLEN Faellen muessen
        // brauchbare Defaults hinterlegt sein" -> damals 0.92 m, weil eine geleerte Firmware
        // ohne Import mit 2,50 m die Kruemmung 2,7-fach zu klein rechnet und die Bridge dann
        // die eigenen richtigen Messungen verwirft (sieht aus wie ein Sensorproblem).
        // Diese Gefahr besteht weiter, sie ist jetzt durch die erreichbare Seite abgefangen.
        float wheelbase = 2.50f;

        // Vehicle kinematic type - matches AgOpenGPS's own setVehicle_vehicleType
        // values (confirmed from real exported vehicle.xml files):
        //   0 = Standard (Ackermann front-wheel steering)
        //   2 = Articulated (center-pivot / "Knicklenker")
        // Other AgOpenGPS vehicleType values (e.g. Harvester, Trike) are not
        // handled here yet and fall back to the Standard (Ackermann) model.
        // 2026-11-07 (Betreiber): Default 0 = Standard/Ackermann. Die Maschine dieser
        // Firmware ist der Knicklenker (Typ 2) und stellt das auf der Fahrzeugseite ein;
        // eine geleerte Firmware soll dagegen NICHT die Maehwerks-Geometrie annehmen.
        // Siehe die Anmerkung bei wheelbase (Entscheidung 2026-11-02 aufgehoben).
        uint8_t vehicleType = 0;
        
        // Motor calibration
        //
        // MEASUREMENT, NOT THE ACTIVE SCALE ANY MORE (Plan 1, AOG owns counts).
        //
        // The value actually used to convert encoder counts into degrees is
        // AgOpenGPS's PGN 252 countsPerDegree - see setAogSteerScaling() below.
        // This field holds the cpd MEASURED by the endstop calibration, i.e. it
        // is the answer to "what should the operator type into AOG's Counts per
        // degree slider?". It is kept because
        //   * the web UI needs it as the recommendation / cross-check,
        //   * getEffectiveCountsPerDegree() falls back to it if AOG has never
        //     supplied a usable counts value (division-by-zero guard).
        //
        // Rationale for the handover: with a real analog WAS the operator tunes
        // AOG's "Counts per degree" until the angle at full lock matches the
        // maximum angle - the slider is sent down in PGN 252 and the FIRMWARE
        // does the division. The VWAS therefore scales with the same number, so
        // the two paths behave identically and there is exactly one scale
        // factor in the system instead of two that can disagree.
        float countsPerDegree = 100.0f;   // MEASURED encoder counts per steering degree
        // 2026-11-01: KONSTANT true - kein UI-Feld mehr. Das ist der Wert, der auf der Maschine
    // live war (Boot-Log 01.11.: "encoder invert=1, AgOpenGPS invertWAS=0 -> reported angle
    // is INVERTED (a RIGHT turn reads NEGATIVE)"). Es ist KEINE Doppelung zu AgOpenGPS:
    // invertEncoder ist der gemessene HARDWARE-Fakt (welcher Anschlag die Counts hochzaehlt),
    // invertWAS die Bediener-Aussage zur Verdrahtung. effectiveInvertEncoder() verknuepft
    // beide per XOR - AOGs Schalter greift also weiterhin und hat das letzte Wort.
    // Das Feld bleibt im Struct, weil der Blob als Rohblock im EEPROM liegt; der Default
    // wird durch die neue Magic (0xF1) einmal neu geschrieben.
        static constexpr bool invertEncoder = true;   // KONSTANT (0 Byte im Blob), s. Kommentar oben

        // Maximum steering angle (degrees, ONE side - i.e. half the
        // mechanical lock-to-lock swing). AgOpenGPS does NOT transmit this in
        // any PGN (verified against 6.8.4: neither PGN 251 nor PGN 252 carries
        // maxSteerAngle), so it has to be kept in sync with AOG's own
        // "maxSteerAngle" by hand (web UI / vehicle.xml import). Only used for
        // the -127..+128 display mapping in getScaledAngle(); the actual wire
        // format of PGN 253 is degrees * 100, so a mismatch here does not
        // corrupt the steering angle itself.
        // 2026-11-07 (Betreiber): Default 40 Grad (Traktor). Der Maeher hat 75 Grad -
        // GEMESSEN (B10.72: vehicle.xml sagt 45, real 75) und wird auf der Fahrzeugseite
        // eingetragen. Der Wert klemmt die Ausgabe in updateKalmanFilter: mit 40 waere
        // beim Maeher jede Kurve ueber 40 Grad abgeschnitten.
        float maxSteeringAngle = 40.0f;

        // --- Zero-point observer (Stufe 2) ---------------------------------
        // Sidehill gain: degrees of steering angle per degree of roll, i.e.
        // AgOpenGPS's own "Side Hill Comp" (CGuidance.cs:98 applies
        // steerAngleGO += imuRoll * -sideHillCompFactor). AgOpenGPS never sends
        // it in a PGN, so it has to be configured here.
        //
        // MAGNITUDE ONLY, always positive, entered in AgOpenGPS's convention.
        // The operator's slider is positive-only and 0.10..0.15 works on his
        // machine (his words). The SIGN is DERIVED, never entered:
        //
        //   internalGain = +/- magnitude * (signFlip ? -1 : +1)
        //                            * (effectiveInvertEncoder() ? -1 : +1)
        //
        // The invert factor is a necessity, not a guess: the residual model is
        //     residual = reportedAngle - gain * roll
        // and reportedAngle flips sign with the invert state, so the gain that
        // makes the roll term cancel has to flip with it. With the operator's
        // encoder calibration (invert=1 from the endstop run) and AgOpenGPS's
        // "Invert WAS" off, reportedAngle is INVERTED relative to AgOpenGPS -
        // which is why the internal value is NEGATIVE while the slider is
        // positive. getEffectiveSideHillGain() is what actually gets used, and
        // it is printed at every boot and in the web UI so there is no doubt.
        //
        // Field evidence for the magnitude: regressing errEst against rollMean
        // over the three 2026-09-21 logs with a wide roll range puts the true
        // constant at -0.07 .. -0.20 deg/deg (the wide-range, low-R^2 ones; the
        // narrow-range logs fit the slow time drift instead and are unusable).
        // That brackets AgOpenGPS's 0.12..0.15. The automated estimator may NOT
        // be used to decide this value: 31 of its candidates scattered with
        // sd 0.53 around a true value of ~0.125, i.e. it is noise-dominated (see
        // ZO_GAIN_MEDIAN_N). It now only REPORTS what it measures.
        // 2026-11-01: Default 0.125 -> 0.12 (Betreiber). 0.12 ist der Wert, der auf dem
        // Stiga eingetragen war; die 0.125 stammten aus der Messung und liegen ohnehin
        // innerhalb der Streuung (der Schaetzer ist rauschdominiert, s. Kommentar unten).
        float sideHillGainDegPerDeg = 0.12f;

        // Field escape hatch: flips the derived internal sign without a rebuild,
        // in case the derived sign turns out wrong. Test from the next log: the
        // slope of errEst vs rollMean must go to 0 with the correct sign and
        // doubles with the wrong one.
        static constexpr bool sideHillGainSignFlip = false;  // KONSTANT: Vorzeichen-Umkehr war ein Notausgang

        // Master switch for the zero-point trimmer. The observer always runs (it
        // is pure measurement and feeds the diagnostics); this only controls
        // whether the residual it measures is allowed to move encoderOffset.
        bool zeroTrimEnable = true;

        // NO Ackermann factor lives here (removed by decision 1b). AgOpenGPS's
        // PGN 252 ackermanFix byte is the single owner of the left/right
        // asymmetry; it is applied in AutosteerProcessor::process() to negative
        // (left) angles only.
        //
        // History: an endstop-measured factor (rightHalf / leftHalf) used to be
        // computed here, validated, persisted to EEPROM and shown in the web
        // UI - but getAckermanFix() had zero callers, so it was never applied.
        // A factor that is displayed but does nothing is worse than none: it
        // invites the operator to trust a number that has no effect. Applying
        // it *and* AOG's byte would double-correct instead. Hence: removed.

        // Set to true only by a successful endstop calibration (or by an
        // explicit manual cpd entry in the web UI). countsPerDegree cannot be
        // derived from the vehicle - any factory default is essentially always
        // wrong, and a wrong cpd makes the steering loop rail into the
        // mechanical stop. Persisted so the web UI / boot log can warn instead
        // of silently driving with a guessed value.
        bool cpdCalibrated = false;
        
        // Kalman filter parameters
        float processNoise = 0.1f;        // Q - process noise covariance
        float measurementNoise = 1.0f;    // R - measurement noise covariance
        float initialUncertainty = 10.0f; // P - initial error covariance
        
        // Fusion parameters
        float minSpeedForGPS = 0.5f;      // Minimum speed for GPS fusion (m/s) - kept for now per user request
        float maxHeadingRate = 50.0f;     // Maximum valid heading rate (deg/s)
        uint16_t varianceBufferSize = 50; // Size of variance calculation buffer

        // --- Encoder-dominant drift correction (replaces per-cycle Kalman blend) ---
        // Design follows the proven community approach (Cerea, AOG_Teensy_UM98X,
        // 87yj/AgOpen_Keya_VirtualWAS): the encoder is ALWAYS the primary,
        // instantaneous angle source. GPS/dual-heading is only ever used to
        // slowly trim a long-term offset (encoderOffset) - never to directly
        // replace or blend into the instantaneous angle. This avoids injecting
        // noisy single-sample heading-rate measurements straight into the
        // steering command.
        // 2026-11-01: UNGENUTZT - die Checkbox ist aus dem Web-IO entfernt, die Fusion nimmt
    // die Dual-Lage immer, wenn sie da und sauber ist (WheelAngleFusion.cpp:440). Das Feld
    // bleibt nur, weil die Config als ROHER STRUCT-BLOB im EEPROM liegt - Loeschen wuerde
    // alle folgenden Werte verschieben.
        static constexpr bool preferDualHeading = true;      // KONSTANT: Dual-Lage wird immer genommen
                                           // instead of single-antenna course-over-ground
                                           // (gpsData.headingTrue) whenever available. Dual
                                           // heading is a true compass reading and does not
                                           // require vehicle motion to be accurate.
        // Accepted HPR / fix quality values (Unicore "QF"), as a bitmask.
        //
        // The QF scale is NOT ordered by quality, so a ">= min" threshold cannot
        // express the intent. Per the Unicore manual:
        //   0 = fix invalid      1 = single point    2 = DGPS
        //   4 = RTK fix          5 = RTK float       6 = DEAD RECKONING
        //   7 = manual/fixed     8 = extra wide lane 9 = SBAS
        //
        // WIDER SET, deliberately: accept every solution that is actually MEASURED,
        // reject only 0 (invalid), 6 (dead reckoning) and 7 (operator-set/manual) -
        // those are the receiver's way of saying "I am not measuring this", which is
        // exactly what must never be used to trim the WAS offset.
        //
        // The previous value was (1<<4)|(1<<5), i.e. RTK only. That also excluded
        // single point, DGPS, extra-wide-lane and SBAS, so a perfectly working
        // non-RTK installation fell back to COG for no physical reason: 8 is a real
        // ambiguity-resolved solution, 9 and 1/2 are real measurements. Requiring RTK
        // is a POSITION accuracy requirement, not a heading validity requirement -
        // conflating the two is what the operator flagged, and RTK costs money so
        // not every machine has it. Dead reckoning stays excluded, which was the
        // original reason the mask exists.
        uint16_t headingQualityMask = (1u << 1) | (1u << 2) | (1u << 4) |
                                      (1u << 5) | (1u << 8) | (1u << 9);
                                           // before trusting the heading-derived curvature.
                                           // Curvature is computed as headingDelta/distance
                                           // (geometric, speed-independent) rather than
                                           // headingRate/speed - this removes a second,
                                           // independently-noisy division term.
                                           // heading-derived angle before it's used to trim
                                           // the encoder offset.
                                           // how fast GPS can ever pull the reported angle,
                                           // regardless of how large the disagreement is.
                                           // disagrees with the encoder by more than this many
                                           // degrees, treat it as an outlier/real manoeuvre and
                                           // skip the drift correction entirely this cycle.
                                           // Tightened from 15 deg: the 2026-09-08 log showed
                                           // the trim moving ~2 deg while cornering, i.e. 15 deg
                                           // let genuine-turn innovations through.
                                           // looked-up history sample implies a speed above this,
                                           // treat it as a GPS jump/multipath glitch and reject
                                           // the sample (guards against noisy low-cost GPS
                                           // occasionally reporting a spurious position jump).

        // --- Phase 4: validity gates for the drift trim ---
        // Straight-drive gate. The heading-derived angle is a *curvature*
        // angle; during a turn the encoder angle and the heading-derived
        // angle describe different things (different reference points, plus
        // steering/tyre dynamics), so a disagreement there is not drift.
        // The 2026-09-08 log showed the trim moving ~2 deg while cornering
        // even with driftInnovationRejectThreshold tightened to 5 deg, so an
        // explicit "vehicle is straight" condition is required.
        float straightDriveMaxAngle = 3.0f; // deg of reported angle around straight

        // Maximum age of a lookback sample. The lookback searches for the
        // sample ~minDistanceForCurvature back in the ring buffer. While
        // creeping (or stationary) that index walks arbitrarily far back, so
        // the heading delta would describe a manoeuvre from long ago. At the
        // ~0.04 m/s creep speed a 10 cm lookback is ~2.5 s, hence 3 s.
        float maxGPSSampleAge = 3.0f;      // s

        // Sensor selection
        bool useIMUHeadingRate = false;   // Use IMU instead of GPS for heading rate

        // Firmware-side zero-point trim (encoderOffset).
        //
        // Default OFF: AgOpenGPS' "Smart WAS" does the same job better and is
        // the single owner of the zero point. It samples the PGN 254 SETPOINT
        // (not our reported angle), needs >=2 km/h / <=500 mm XTE / |angle|
        // <=15 deg and 200..2000 samples, takes a MEDIAN and gates on
        // confidence (CSmartWAS.cs, FormSteer.cs), then writes wasOffset via
        // PGN 252 - which AutosteerProcessor::process() already subtracts for
        // the VWAS path.
        //
        // Running both regulators at once is a double integrator on the same
        // physical quantity (the zero point): this one is invisible to AOG
        // because it sits *inside* the reported angle, so neither can converge.
        // The 2026-09-08 log shows exactly that oscillation (encoderOffset
        // wandering to +2.0 deg).
        //
        // PERMANENTLY OFF - confirmed by operator decision. Do NOT enable this
        // as a "fix" for anything: it is not a missing feature, it is a second
        // integrator on a quantity AgOpenGPS already regulates. The firmware's
        // job on the VWAS path is the encoder scale (countsPerDegree), the
        // direction (invertEncoder) and the reported angle; the zero point and
        // its long-term trim belong to AOG. Re-enabling it requires switching
        // AOG's Smart WAS / Zero-WAS off first, and that is a deliberate
        // system-level choice, not a tuning tweak.
        bool enableDriftCompensation = false;
    };
    
    // Constructor
    WheelAngleFusion();
    ~WheelAngleFusion() = default;
    
    // Initialization
    // motor: any MotorDriverInterface with hasPositionFeedback()==true
    //        (TractorCANDriver with Keya, or KeyaSerialDriver)
    bool init(MotorDriverInterface* motor, GNSSProcessor* gnss, IMUProcessor* imu);
    
    // Configuration
    void setConfig(const Config& cfg) { config = cfg; }
    Config& getConfig() { return config; }
    const Config& getConfig() const { return config; }
    
    // Main update function - call at 100Hz
    void update(float dt);
    
    // Get fusion results
    float getFusedAngle() const { return fusedAngle; }
    float getEncoderOffset() const { return encoderOffset; }
    float getPredictedAngle() const { return predictedAngle; }
    float getGPSAngle() const { return gpsAngle; }
    // Wheel-angle reliability as a MODEL input, 0...100 (operator request 2026-09-28:
    // "evtl im VWAS Zweig eine Zahl einbauen, die die Verlaesslichkeit des Lenkwinkel
    // anzeigt, z.B. wie DOP"). The HeadingBridge consumes it through
    // AutosteerProcessor::setWheelAngleQuality().
    float getAngleQuality() const { return computeAngleQuality(); }
    bool  isZeroLearned() const { return (zoFits > 0) && (zoState == ZO_HOLD); }
    float getEncoderAngle() const { return encoderAngle; }

    /// GPS-derived vehicle speed in m/s, smoothed with the same 80/20 EMA that
    /// AgOpenGPS uses. Deliberately exposed so the commissioning log can print
    /// it NEXT TO the PGN 254 speed: those are two different values (this one
    /// is filtered here, the PGN one is quantised to 0.1 km/h by AOG) and
    /// confusing them is what makes standstill-vs-creep diagnosis hard.
    float getSpeedMps() const { return vehicleSpeed; }

    /// Net straight-line displacement over STANDSTILL_WINDOW_S, from the GPS ring
    /// buffer. This is the ONLY reliable way to separate GPS noise from real
    /// creep: no low-pass filter can, because a steady bias (multipath, RTK
    /// drift, a standing satellite constellation) survives averaging and reads
    /// as a constant "0.1 km/h" while the vehicle does not move. Position
    /// geometry can: noise is a random walk (net -> 0), motion is directed (net
    /// grows linearly with the window).
    float getNetDisplacementM() const { return standstillNetM; }

    /// True when a long enough window of position history exists. When false the
    /// caller must fall back to speed alone instead of assuming standstill.
    bool hasStandstillAssessment() const { return standstillValid; }

    /// True only when a fully populated window exists AND shows no net travel.
    bool isStandstill() const
    {
        return standstillValid && (standstillNetM < STANDSTILL_NET_M);
    }

    /// Get scaled angle for telemetry / web interface (-127..128 scale).
    /// int16_t (not int8_t) because the positive limit (+128) does not fit in
    /// a signed byte. Display-only: PGN 253 still carries the real angle as
    /// int16 degrees*100.
    int16_t getScaledAngle() const;

    /// Roll in degrees from the source that currently provides the heading:
    /// dual-antenna baseline roll (gpsData.dualRoll) when dual heading is in
    /// use, otherwise INS roll (gpsData.insRoll) if available.
    ///
    /// Reported only - never applied by the firmware. AgOpenGPS does its own
    /// side-hill compensation on the setpoint (CABLine.cs: steerAngleAB +=
    /// imuRoll * -sideHillCompFactor), so applying it here as well would
    /// double-correct. Exposed for the web UI / diagnostics so the operator
    /// can see whether the dual baseline is actually level.
    float getRollDeg() const { return rollDeg; }
    bool hasRoll() const { return rollValid; }

    /// Raw accumulated encoder counts (driver-native sign, no invert applied)
    /// and the count value captured with the wheels straight. These are the
    /// ground truth for calibration: countsPerDegree is derived from the count
    /// span between the two endstops, so changing it never rescales history.
    int32_t getRawCounts() const { return rawCounts; }
    int32_t getCenterCounts() const { return centerCounts; }
    
    // Get quality metrics
    float getUncertainty() const { return uncertainty; }
    float getMeasurementVariance() const { return measurementVariance; }
    float getKalmanGain() const { return kalmanGain; }
    
    // Health and status
    bool isHealthy() const;
    bool hasValidGPSAngle() const { return gpsAngleValid; }
    uint32_t getLastUpdateTime() const { return lastUpdateTime; }
    
    // Calibration
    void setEncoderCenter(); // Set current position as center (0 degrees)

    // Set encoderOffset directly to a specific value (degrees, in raw
    // encoderAngle scale) rather than "current position". Used by the
    // lock-to-lock endstop calibration sweep, which computes the midpoint
    // between two recorded extremes without physically returning the
    // motor to that midpoint first.
    void setEncoderOffsetDirect(float offsetDegrees) { encoderOffset = offsetDegrees; }

    // 2026-11-02 entfernt: recalibrateCountsPerDegree(). Sie skalierte
    // countsPerDegree aus einem gemessenen Anschlag-Sweep nach - der Weg ist mit
    // der Kalibrierung entfallen, und AgOpenGPS fuehrt den Wert (PGN 252).
    // Die fruehere Begruendung bleibt zur Nachvollziehbarkeit:
    // measuredSwingDegrees = |rightAngle - leftAngle| bei den Anschlaegen,
    // trueSwingDegrees = der mechanische Vollausschlag.

    // --- Raw-count calibration API (endstop calibration / "set straight") ---
    // Set the straight-ahead reference directly (raw counts) and clear the
    // long-term GPS drift offset. Does NOT move the motor.
    void setCenterCounts(int32_t c);

    /// Call ONCE at boot, after the stored config has been loaded.
    ///
    /// The straight-ahead reference cannot be restored from EEPROM, because the
    /// value it is expressed in does not survive a power cycle: rawCounts is an
    /// accumulator that starts at 0 on every Teensy boot, and the Keya's own
    /// position counter restarts with the motor (measured 2026-09-21: position
    /// ~65232 before a motor power interruption, ~495 after it, wheels unmoved).
    /// Pairing a stored centreCounts with a fresh accumulator produced -54.6 deg
    /// (clamped to the -45 deg limit) while the wheels were roughly 15 deg right
    /// - see the 15-28 log. That is an arbitrary angle, not a small offset.
    ///
    /// So the only honest start is a DEFINED one: treat the position at power-up
    /// as straight ahead. That is a policy, not a measurement - it is logged as
    /// such, and the GND button, the web "Geradeausstellung" button or an
    /// AgOpenGPS zero click correct it. A wrong-but-defined zero is strictly
    /// better than an arbitrary one.
    void adoptBootZero();

    // --- Zero-point observer / trimmer (Stufe 2) ----------------------------
    //
    // WHY THIS EXISTS AND WHY IT IS NOT updateDriftCompensation()
    //
    // The zero point has to be re-acquired after every power cycle, because both
    // the mechanical reference (a count value) and AgOpenGPS's wasOffset (counts
    // on the same counter) are invalidated by it. Neither AOG's Smart WAS (needs
    // >= 2 km/h, an open settings window and a button press) nor the endstop
    // calibration (needs the operator to turn the wheel to both stops) can do that
    // unattended.
    //
    // This observer uses the ONE relation that holds whenever the path is
    // straight:
    //
    //     reportedAngle = actualAngle + residual,  actualAngle ~= roll * gain
    //
    // because on a straight path the only steering that is needed is the sidehill
    // / implement-draft term - and that term is what AOG itself models as
    // roll * sideHillCompFactor (speed-INDEPENDENT, unlike the naive kinematic
    // atan(g*L*sin(roll)/v^2), which assumes a sliding vehicle and would demand
    // 26 deg at 5 km/h and 3 deg of roll).
    //
    // Regressing reportedAngle against roll over a window therefore yields BOTH
    // the gain (slope) and the residual zero error (intercept). The roll spread
    // requirement is what makes them separable.
    //
    // This is deliberately NOT the old GPS-heading drift compensator (which stays
    // permanently disabled, see config.enableDriftCompensation): it needs no
    // heading rate, no division by speed and no GPS curvature chain - only the
    // "path is straight" gate comes from the existing distance-based gpsAngle.
    //
    // ANTI-DOUBLE-INTEGRATOR RULES (operator requirement):
    //   * the residual is TRACKED at a low rate (acquire), then strongly damped
    //     once it is stable (hold) - a second integrator must not run against
    //     AgOpenGPS's PID integral;
    //   * the trim is BLOCKED entirely while guidance is engaged (AOG's integral
    //     owns the quantity then) and while AOG's own zero offset is active;
    //   * a per-session budget caps how far the trim may ever move the angle.
    void setZeroTrimPermissions(bool guidanceActive, bool aogZeroActive);

    struct ZeroObserverStatus {
        float    residualDeg;      // intercept of the last fit (= what the trim nulls)
        float    gainDegPerDeg;    // slope of the last fit / learned sidehill gain
        float    trimDeg;          // encoderOffset currently applied
        float    rollSpreadDeg;    // roll range covered by the last fit
        float    rollMeanDeg;      // mean roll of the last fit (the value the gain acts on)
        uint16_t fitSamples;       // samples in the current window
        uint8_t  fitBins;          // non-empty roll bins in the last fit
        uint8_t  fits;             // completed fits this session
        uint8_t  state;            // see ZeroObserverState
        uint32_t trims;            // trim steps applied this session
        float    acquireRateDegPerSec; // current quality/spread-scaled rate
        // Wheel-angle RELIABILITY (0...100), DOP-style. This is what the HeadingBridge
        // needs to decide whether its kinematic model may be trusted at all:
        // 0 = not usable as a model input (the zero was never learned - the Keya motor
        // sets its zero at power-up to wherever the wheels stand!), >= 80 = locked.
        float    angleQuality;
        bool     zeroLearned;      // at least one stable fit and the observer is in HOLD
    };
    enum ZeroObserverState : uint8_t {
        ZO_IDLE     = 0,   // not enough data yet
        ZO_ACQUIRE  = 1,   // correcting at the acquire rate
        ZO_HOLD     = 2,   // residual stable - heavily damped
        ZO_BLOCKED  = 3    // guidance engaged or AOG owns the zero: observe only
    };
    ZeroObserverStatus getZeroObserverStatus() const;

    // Set counts-per-degree (mechanical ratio) directly. Persisted by the
    // caller (EEPROM); never rescales already-accumulated raw counts.
    void setCountsPerDegree(float cpd) { if (cpd != 0.0f) config.countsPerDegree = cpd; }
    // 2026-11-01: invertEncoder ist eine Konstante - der Setter bleibt als No-op, damit die
    void setInvertEncoder(bool) { /* invertEncoder ist fest true (WheelAngleFusion.h) */ }

    // --- AgOpenGPS-owned steering scale and zero offset (Plan 1) ------------
    //
    // AgOpenGPS is the single owner of countsPerDegree, wasOffset and the
    // Ackermann factor. For the VWAS the encoder plays the role of the analog
    // ADC raw value and the straight-ahead reference the role of the ADC
    // centre, so the conversion becomes literally the same expression as the
    // analog path in ADProcessor:
    //
    //     position = rawCounts - centerCounts        (signed by invertEncoder)
    //     angle    = (position + wasOffset) / countsPerDegree
    //
    // which matches NG-V6 (include/Autosteer.h:418-427) and, crucially, gives
    // d(angle)/d(wasOffset) = +1/countsPerDegree in BOTH invert branches. That
    // positive slope is what AgOpenGPS's Zero-WAS button needs to converge:
    // it applies
    //     wasOffset += countsPerDegree * -actualSteerAngleDegrees
    // (FormSteer.cs:903/911), so subtracting the offset in the firmware would
    // double the error on every click instead of nulling it.
    //
    // These two values are NOT persisted here - they belong to AOG. The caller
    // (AutosteerProcessor) pushes them in after a PGN 252 receipt and after
    // loadSteerSettings(), and re-pushes them on change from process().
    void setAogSteerScaling(uint8_t countsPerDegree, int16_t wasOffset, bool invertWas);

    uint8_t getAogCountsPerDegree() const { return aogCountsPerDegree; }
    int16_t getAogWasOffset() const { return aogWasOffset; }
    bool    getAogInvertWas() const { return aogInvertWas; }

    /// The sign convention actually applied to the reported angle: the mechanically
    /// measured encoder direction XOR AgOpenGPS's own "Invert WAS" (PGN 251).
    ///
    /// Both flags are needed and they mean different things. invertEncoder is a
    /// HARDWARE fact measured by the endstop calibration (which stop makes the counts
    /// rise) and it is chosen so that a right turn reports a positive angle.
    /// AgOpenGPS's invertWAS is the OPERATOR's statement about the wiring, and the
    /// analog path in this firmware already honours it (ADProcessor::getWASAngle
    /// negates the angle when it is set). Until the VWAS did the same, an operator who
    /// had ticked "Invert WAS" in AgOpenGPS got the opposite sign from the virtual WAS
    /// than from a real one - which is a runaway-steering situation, not a cosmetic
    /// one, because AgOpenGPS compares the reported angle against its own setpoint.
    bool effectiveInvertEncoder() const {
        return (config.invertEncoder != aogInvertWas);
    }

    /// The countsPerDegree actually used for the encoder->degree conversion:
    /// AOG's value when AOG has supplied a usable one, otherwise the measured
    /// config.countsPerDegree. A zero AOG value would otherwise be a division
    /// by zero and would pin the reported angle, so the fallback is mandatory.
    float getEffectiveCountsPerDegree() const;

    /// The sidehill gain actually used in `residual = reportedAngle - gain * roll`,
    /// in deg of steering per deg of roll. Derived from
    /// Config::sideHillGainDegPerDeg (magnitude, AgOpenGPS convention) and the
    /// effective invert state - see the comment on that field. Public because the
    /// boot log, the web UI and the fit diagnostics all have to print the same
    /// number the residual is computed with, otherwise a wrong sign is invisible.
    float effectiveSideHillGain() const;

    /// Median of the estimator's recent candidates, in the SAME sign convention as
    /// effectiveSideHillGain(). DIAGNOSTIC ONLY - it is never applied. Printed next
    /// to the configured value so a wrong config or a wrong derived sign shows up in
    /// one log instead of by trial and error.
    float getReportedSideHillGain() const { return zoGainReportedDegPerDeg; }

    bool isCpdCalibrated() const { return config.cpdCalibrated; }

    // 2026-11-02 entfernt: computeHalfSpans(), computeCountsPerDegree() und
    // setCpdCalibrated(). Sie gehoerten zur dreistufigen Anschlag-Kalibrierung,
    // die mit ihren Web-Endpunkten auch den Aufrufer verloren hat. countsPerDegree
    // kommt heute aus AgOpenGPS (PGN 252); cpdCalibrated wird weiterhin GELESEN
    // (AutosteerProcessor, Web-UI) und vom manuellen cpd-Eintrag gesetzt, nur der
    // Setter fuer die Kalibrierung ist weg. CPD_MIN/CPD_MAX bleiben - die
    // Plausibilitaetspruefung des manuellen Eintrags braucht sie.

    // Plausibility window for countsPerDegree.
    // The operator's requirement was a 20..500 window; the floor was lowered to
    // 8 (operator decision) because the 2026-09-08 log yields ~17..21
    // counts/deg for this vehicle. A floor of 20 would sit right on top of the
    // real value and reject a *correct* calibration under normal capture noise,
    // leaving no path to a valid setup at all. 8 still rejects garbage (e.g. an
    // encoder that only produced noise), and the span / centring checks are the
    // sharper discriminators anyway.
    static constexpr float CPD_MIN = 8.0f;
    static constexpr float CPD_MAX = 500.0f;

    // Minimum total count span between the two mechanical stops. Below this
    // the capture is worthless regardless of the implied cpd (e.g. the wheel
    // was never turned, or the encoder is not responding).
    static constexpr int32_t MIN_SPAN_COUNTS = 200;
    
    // Reset and recovery
    void reset();
    void resetDriftCompensation();
    
private:
    // Sensor interfaces
    MotorDriverInterface* keyaDriver;  // name kept for minimal diff; any driver with position feedback
    GNSSProcessor* gnssProcessor;
    IMUProcessor* imuProcessor;
    
    // Configuration
    Config config;

    // Apply the +/- maxSteeringAngle limit to fusedAngle. Shared by the normal
    // fusion cycle and by the two setters that recompute the angle immediately
    // (centre capture, AOG scaling push): if only updateKalmanFilter() clamped,
    // a setter running right after it would publish an unclamped angle for one
    // cycle, and a too-small maxSteeringAngle would then be invisible again -
    // which is the exact failure this clamp exists to expose.
    void applyFusedAngleClamp();

    // --- Zero-point observer internals --------------------------------------
    //
    // The fit is a weighted least squares over roll BINS rather than over raw
    // samples: a 10 Hz stream over a whole field would otherwise be dominated by
    // whichever roll value happened to occur most often, and per-bin means also
    // suppress single-sample outliers for free.
    static constexpr uint8_t  ZO_BINS            = 33;    // -8.0 .. +8.0 deg
    static constexpr float    ZO_BIN_WIDTH_DEG   = 0.5f;
    static constexpr float    ZO_MAX_ROLL_DEG    = 8.0f;
    // 200 samples at the 100 Hz fusion rate = a 2 s window (the fit timestamps in
    // the 16-13 log are exactly 2.00 s apart). Short enough that a large error is
    // corrected promptly - the limit was never the window, it was the rate.
    static constexpr uint16_t ZO_WINDOW_SAMPLES  = 200;
    // --- wheel angle reliability, see computeAngleQuality() -----------------
    // How many completed fits count as "the zero is known" and how much fit deviation
    // (|raw estimate - filtered estimate|) is still acceptable.
    static constexpr uint8_t  ZO_Q_FITS_FULL   = 12;
    static constexpr float    ZO_Q_DEV_BAD_DEG = 5.0f;
    static constexpr uint8_t  ZO_MIN_BINS        = 5;

    // "The path is straight" gate, in degrees of the distance-based curvature angle.
    //
    // 2.5 deg, not 1.5: the curvature is computed over a 0.10 m lookback (tuned for
    // creep speed), so a 0.2 deg dual-antenna heading step is already 2 deg of
    // curvature angle - the measurement is noisy by construction, and a tighter
    // gate would reject most of a genuinely straight drive as well as bias the
    // accepted samples toward noise minima. Accuracy is meant to come from the
    // 200-sample window and the consistency guards, not from a tight per-sample gate.
    static constexpr float    ZO_STRAIGHT_GATE   = 2.5f;
    static constexpr float    ZO_MIN_SPEED_MPS   = 0.8f;  // ~2.9 km/h, dual-antenna heading
    // COG (headingTrue, from VTG) only means something once the vehicle is moving,
    // so the single-receiver + IMU case gets a higher floor.
    static constexpr float    ZO_MIN_SPEED_COG_MPS = 1.5f; // ~5.4 km/h

    // --- long-baseline straightness gate (operator request 2026-09-21) ----------
    //
    // Replaces the 0.10 m curvature angle as the observer's straightness test.
    //
    // WHY LONGER IS BETTER, AND WHY THE OPERATOR STEERING DURING THE BASELINE IS NOT
    // A PROBLEM:
    //   * a TRANSIENT correction - one the operator counteracts - is zero-mean over
    //     the 2 s window, so the regression absorbs it as scatter, not as a shift of
    //     the intercept. It does not need to be "computed out";
    //   * what DOES bias the intercept is a SUSTAINED steering input, i.e. a genuinely
    //     curved path, because then the mean actual angle is no longer the sidehill
    //     term. A long baseline measures exactly that: the NET heading change over
    //     0.5 m of real travel. A correction inside the baseline bends the path and
    //     the sample is rejected - which is the desired behaviour, not an artefact;
    //   * the 0.10 m curvature angle cannot do that job at all: at 0.10 m a 0.2 deg
    //     heading step already reads as ~2 deg of steering angle, so it is mostly
    //     random rejection that neither confirms nor denies straightness.
    //
    // Resolution: 0.2 deg of heading noise over 0.5 m resolves about 0.9 deg of
    // equivalent steering angle, over 1 m about 0.4 deg - against a 3 deg gate, i.e.
    // 3-7 sigma. 0.5 m is chosen because at the 0.8 m/s floor it spans 0.63 s of
    // travel; 1 m would double the resolution but take 1.25 s, long enough that
    // "straight right now" stops being a fair description. The operator's rule of
    // thumb ("50 cm or 1 m, longer is unrealistic because he will have steered in the
    // meantime") matches this: the limit is set by how long a straight stretch the
    // operator can guarantee, not by the arithmetic.
    static constexpr float    ZO_STRAIGHT_BASELINE_M = 0.5f;
    static constexpr float    ZO_STRAIGHT_GATE_LONG  = 3.0f; // deg equivalent steering
    static constexpr float    ZO_BASELINE_MAX_AGE_S  = 3.0f; // older = a past manoeuvre

    // --- standstill detection (operator requirement 2026-09-21) ----------------
    // "The motor must be free at standstill = autosteer off. We should steer from
    //  120 m/h upwards, below that not. We have to RECOGNISE the noise."
    //
    // The thresholds come out of the geometry, they are not tuned:
    //   120 m/h = 0.0333 m/s  ->  10.0 cm of net travel in 3 s
    //   100 m/h = 0.0278 m/s  ->   8.3 cm
    //   stationary F9P bias   ->   0-2 cm, and it does NOT grow with the window
    // 5 cm therefore sits 2-5x above the position noise floor and at half the
    // 120 m/h requirement: it vetoes a fake "0.1 km/h" at standstill without
    // touching real creep. 3 s because the window has to be long enough for the
    // random walk to average out - that is what makes the test work at all.
    static constexpr float STANDSTILL_WINDOW_S = 3.0f;
    static constexpr float STANDSTILL_NET_M    = 0.05f;
    // A window shorter than this cannot separate the two cases (120 m/h over
    // 2.25 s is only 7.5 cm, i.e. barely above the noise floor), so a partially
    // filled history is reported as "no assessment" rather than as movement.
    static constexpr float STANDSTILL_MIN_FRACTION = 0.75f;

    // --- plausibility limit for the implied speed (2026-09-27) ---------------
    // Config::maxPlausibleSpeed was an ABSOLUTE bound (5 m/s = 18 km/h) meant to
    // catch a GPS jump / multipath glitch. Measured on the 2026-09-27 logs it did
    // the opposite of its job on a transport drive: gpsAngleValid dropped to 0 the
    // moment the speed passed 5 m/s and stayed 0 until it fell back below, so the
    // zero observer accepted NOTHING on the road (win frozen at 12/200 for 38 s,
    // nacc -> 0) - on a drive that ran at 23...37 km/h. The operator explicitly
    // wants the zero learned on the drive to the field, so the bound has to scale.
    //
    // The bound is now relative with an absolute floor:
    //     limit = max(FACTOR * filteredSpeed, maxPlausibleSpeed)
    // Measured support (position per sample / filtered speed, HBUF data, five logs,
    // 2026-09-27): p50 = 1.00, p99 = 1.05 ... 1.46 in four of them, but 2.95 in the
    // fifth, and single-sample maxima of 2.1 / 6.2 / 8.4. So:
    //   * 2.0 would reject a measurable share of GOOD samples in the worst log;
    //   * 3.0 sits above every observed p99 and still catches the 6...8x glitches;
    //   * the observer averages 200 samples, so one leaked glitch costs 1/200 of a
    //     window - BLOCKING good driving is the expensive failure, not admitting a
    //     rare bad sample. The factor is therefore set to be permissive.
    // The same measurement answers the EMA-lag worry: p50 = 1.00 means the filtered
    // speed tracks the positions, so the relative test does not misfire on
    // acceleration.
    static constexpr float PLAUS_SPEED_FACTOR = 3.0f;

    /// Effective implausibility limit for distance/timeSpan, in m/s.
    float plausSpeedLimit() const;

    // NO upper limit on |fusedAngle| here. The first field run (2026-09-21 16-02)
    // proved why: the boot zero was 28.7 deg off (power-up with the wheels turned),
    // and a 20 deg sanity gate rejected EVERY sample - i.e. it blocked exactly the
    // situation the observer exists for. The straight-path gate is the real
    // selector; the angle magnitude only reflects the error being measured.

    // --- acquire rate, scaled by how far the measurement can be trusted --------
    //
    // Operator requirement (2026-09-21): a 20 deg boot error has to be gone in
    // SECONDS when the heading is good. A tractor parked at full lock would
    // otherwise need minutes at a fixed conservative rate, and the boot zero is
    // taken from the power-up position - so a large error is the normal case, not
    // the exception.
    //
    // Two independent things decide how fast we dare go:
    //
    //  1. VALIDITY - only 0 (invalid), 6 (dead reckoning) and 7 (manual/operator-set)
    //     are rejected. Every actually measured solution is allowed to contribute,
    //     so the mechanism works on a plain non-RTK installation.
    //  2. RECEPTION - the satellite count is the "pure signal quality" term and it
    //     moves the rate more than the solution type does.
    //  3. SOLUTION TYPE - carrier-phase solutions are weighted higher than code
    //     solutions, but none of them is zero: RTK is a POSITION accuracy advantage,
    //     not a precondition for measuring the heading well enough to trim a zero.
    //  4. SOURCE - the straight-path gate comes from the dual-antenna heading when
    //     there is one (a true compass reading, valid at standstill) and from VTG
    //     course-over-ground otherwise, which is the single-receiver + IMU case: COG
    //     needs motion and is noisier, so it is weighted lower and needs more speed.
    //  5. HOW WELL THE FIT IS SUPPORTED BY ROLL VARIATION - the 16-13 log shows the
    //     gain swinging from -1.27 to +4.07 deg/deg within one session when the roll
    //     spread was only ~2.5 deg, so a narrow window estimates the intercept
    //     loosely. Scaling by the spread means the trim only moves fast when the
    //     measurement deserves it.
    static constexpr float ZO_ACQUIRE_RATE_MAX   = 5.0f;   // deg/s, full trust
    static constexpr uint8_t ZO_MIN_SATELLITES   = 6;
    static constexpr uint8_t ZO_SAT_FULL_COUNT   = 15;     // full reception weight here
    static constexpr float   ZO_SAT_FACTOR_MIN   = 0.40f;  // at ZO_MIN_SATELLITES
    static constexpr float   ZO_SAT_FACTOR_MAX   = 1.00f;
    static constexpr float ZO_SOL_FACTOR_RTK     = 1.00f;  // 4 RTK fix, 5 RTK float
    static constexpr float ZO_SOL_FACTOR_XWL     = 0.90f;  // 8 extra wide lane
    static constexpr float ZO_SOL_FACTOR_SBAS    = 0.90f;  // 9 SBAS
    static constexpr float ZO_SOL_FACTOR_DGPS    = 0.80f;  // 2 DGPS
    static constexpr float ZO_SOL_FACTOR_SINGLE  = 0.60f;  // 1 single point
    static constexpr float ZO_SRC_FACTOR_DUAL    = 1.00f;  // dual-antenna heading
    static constexpr float ZO_SRC_FACTOR_COG     = 0.60f;  // single receiver + IMU
    // ZO_FIT_SPREAD_REF_DEG (5 deg, "spread that earns the full rate") was REMOVED
    // 2026-09-24 together with its use in zeroAcquireRate(): it only existed to
    // protect the intercept while the GAIN was estimated from the same window. Since
    // the gain is a configuration value (P2) the intercept is valid at any spread, so
    // the factor only throttled the correction - measured cost in the 2026-09-24 log:
    // 0.71 deg/s instead of 5.0. The roll spread is still REPORTED (zoLastFitSpreadDeg,
    // in the fit line) because it says how well-supported a fit was.
    // --- hold phase: a RATE, not a remaining budget (P3) ----------------------
    // The field logs (2026-09-21 holperstrecke) show a genuine slow zero drift of
    // 0.5..1.4 deg/min once the roll term is subtracted - i.e. the mechanical
    // zero really does move while driving. A budget that gets CONSUMED cannot
    // serve that: at ~1 deg/min the old 60 deg was gone in under an hour and the
    // correction then stopped silently (nothing in the logs said so). So the hold
    // phase now corrects indefinitely, slowly.
    //
    // 0.04 deg/s = 2.4 deg/min is about TWICE the fastest drift measured in the
    // field. Deliberately not faster: the filtered short-term estimate carries
    // about +-1 deg of scatter, and a hold rate well above the drift would start
    // tracking that scatter as an oscillation of the zero. Deliberately not
    // slower: below the drift rate the residual would grow until RE-OPEN fires,
    // which is the 0 -> 2.5 deg sawtooth the operator saw in the BÃ¤umen log.
    static constexpr float    ZO_HOLD_RATE_DEG_PER_S = 0.04f;
    static constexpr float ZO_LOCK_TOLERANCE  = 0.25f; // deg, residual stability
    static constexpr uint8_t  ZO_LOCK_FITS       = 3;
    static constexpr float ZO_REOPEN_DEG      = 2.0f;  // verified drift: unlock
    static constexpr uint8_t  ZO_REOPEN_FITS     = 3;
    // --- safety limit: a POSITION clamp, not a consumed budget (P3) -----------
    // The old pair (60 deg acquire / 0.5 deg hold, both "per session") is gone.
    // What actually needs bounding is how far the zero may ever sit from the
    // mechanical reference, and that bound must be available again on the next
    // correction instead of having been used up. 60 deg covers the full steering
    // range, so a boot with the wheels at full lock is still correctable, while a
    // runaway (a bad measurement walking the zero away) is still stopped.
    // NOTE the clamp is on |encoderOffset| - the POSITION - not on the travelled
    // distance zoTrimAppliedDeg, which keeps accumulating both ways and is only
    // reported.
    static constexpr float ZO_TRIM_ABS_MAX_DEG = 60.0f;
    // While the trim is RUNNING, a fit that disagrees with what the model predicts
    // stops the trim immediately (and re-arms it). The prediction is essential: the
    // residual is defined as (mechanical error - encoderOffset), so it MUST change by
    // the amount that was trimmed in the meantime. Comparing the raw residuals would
    // therefore flag every single window as a disagreement as soon as the fast rate
    // is active (5 deg/s x a 2 s window = 10 deg), i.e. the check would cancel
    // exactly the behaviour it is meant to protect. What is compared is the
    // DEVIATION from the prediction, which is ~0 while the measurement stays valid
    // and grows as soon as it stops describing reality.
    // Calibrated from the 16-13 log, where the compensated deviation between
    // consecutive fits was 0.06 / 0.41 / 0.05 / 1.55 deg - and 3.36 deg once. The
    // scatter is the fits' own error, not a fault, so the threshold has to sit above
    // it or the trim would keep stopping and re-arming (each re-arm costs two
    // windows). A genuine breakdown - a leaked straight gate, a bad roll, a moved
    // wheel - deviates by the size of the error it introduces, i.e. 5-20 deg, so
    // 3.5 still detects it at once.
    //
    // Raised to 6.0 deg and re-based on the raw-vs-FILTERED deviation: the 17-0 log
    // shows window-to-window scatter of +-4 deg during ordinary straight driving (the
    // straight gate admits up to 3 deg of genuine steering, which lands in the
    // residual), so a 3.5 deg threshold was tripping on normal behaviour.
    static constexpr float ZO_DISAGREE_STOP_DEG  = 6.0f;

    // Physically plausible range for the sidehill gain. AgOpenGPS's own "Side Hill
    // Comp" slider lives around 0..1 deg/deg; the 16-13 log produced +4.074 with only
    // a 2.77 deg roll spread, which is not a vehicle property but fit noise. Left
    // unclamped that single number moves the INTERCEPT - the value actually applied -
    // by gain_error * mean_roll, which is precisely the +-2 deg scatter seen between
    // consecutive fits. When the fit lands outside the window the slope is pinned to
    // the limit and the intercept is recomputed with it, so the outlier cannot leak
    // into the correction.
    static constexpr float ZO_GAIN_MIN = -1.5f;
    static constexpr float ZO_GAIN_MAX =  1.5f;

    // --- gain estimator: DIAGNOSTIC ONLY (P2) ---------------------------------
    // The gain is a CONFIG value now (Config::sideHillGainDegPerDeg), because the
    // estimator cannot decide it: 31 candidates collected over the 2026-09-21 logs
    // scattered with sd 0.53 around a true value of ~0.125, so even the median of 9
    // moves by about +-0.18 - larger than the value itself. A gain error of 0.2 with
    // +-5 deg of roll injects up to +-1 deg into every residual, i.e. the order of
    // the entire zero-point error being measured. The 19-43 log is the proof: 57 %
    // of its apparent errEst drift was roll-correlated and disappeared once that
    // slope was taken out - the drift was a gain error, not mechanics.
    //
    // The machinery below is kept because it REPORTS what it measures: comparing
    // the reported median against the configured value is how a wrong config - or a
    // wrong derived sign - is recognised from a log instead of by trial and error.
    // It never touches the value used in the residual.
    static constexpr float ZO_GAIN_LEARN_SPREAD = 4.0f;  // deg, before a candidate is attempted
    static constexpr uint8_t ZO_GAIN_MEDIAN_N    = 9;    // candidates behind the reported median
    // Plausibility window for the CONFIGURED magnitude (AgOpenGPS convention,
    // positive). AgOpenGPS's slider lives around 0..1 and the true value on this
    // machine is 0.12..0.15. Outside the window the value is logged as implausible
    // and clamped, so a typo cannot swing the zero estimate by
    // magnitude * ZO_MAX_ROLL_DEG.
    static constexpr float ZO_GAIN_CFG_MIN       = 0.01f;
    static constexpr float ZO_GAIN_CFG_MAX       = 0.50f;
    // Only log a gain line when the reported median moved by this much.
    static constexpr float ZO_GAIN_LOG_STEP      = 0.02f;

    // A single deviating fit must not stop a converging trimmer: the 17-0 log shows
    // consecutive residuals scattering by +-4 deg, so a one-shot threshold trips on
    // noise and costs two windows of re-arming every time. Two consecutive deviations
    // are required, and the per-residual grant caps what one extra window can cost.
    static constexpr uint8_t ZO_STOP_CONSECUTIVE = 2;

    // How many windows the zero-error estimate is averaged over. Window-to-window
    // scatter is NORMAL and not a fault: the straight gate admits up to
    // ZO_STRAIGHT_GATE_LONG (3 deg) of genuine steering, and whatever the driver does
    // inside a 2 s window lands in the residual - the 17-0 log shows +-4 deg between
    // consecutive fits while the underlying error stayed within +-1.5 deg. A single
    // window is therefore not a measurement of the zero to better than a few degrees,
    // and acting on it directly made the trim follow noise. Averaging over N windows
    // reduces that scatter by sqrt(N); 8 windows is ~16 s of straight driving.
    static constexpr uint8_t ZO_RESIDUAL_AVG_FITS = 8;

    // Proportional gain of the trim: step = residual * gain, then rate-capped.
    //
    // NOTE the gain is almost never the binding term: the rate cap (ZO_ACQUIRE_RATE_MAX
    // * dt at 100 Hz) is smaller than residual * gain for every residual above ~0.07 deg,
    // so the convergence is RATE-limited and the time is essentially |error| / rate.
    // The gain only shapes the last fraction of a degree. Do not "tune" it expecting
    // a time-constant change - set the rate instead.
    static constexpr float ZO_TRIM_GAIN          = 0.35f;

    // Window duration the residual is refreshed over. Needed because the trim runs
    // continuously while the residual is only updated once per window: at 5 deg/s a
    // whole 10 deg can be travelled on a stale value while the true error is already
    // 8 deg, so the trim would overshoot and the next fit would see the opposite sign
    // - an oscillation that the STOP guard would then keep interrupting. Capping the
    // per-second step at |residual| / window makes the approach asymptotic (the
    // residual shrinks by at most 100 % per window) instead of oscillating, which is
    // what makes the fast rates usable at all.
    static constexpr float ZO_WINDOW_SECONDS      = 2.0f;
    // The residual is only refreshed once per window, so it must not stay valid
    // forever either: if the straight gate closes and no new fit arrives, a residual
    // from minutes ago would keep driving the trim. Backstop only - the binding limit
    // is the per-residual grant in updateZeroObserver().
    static constexpr float ZO_RESIDUAL_MAX_AGE_S  = 4.0f;

    /// Current acquire rate: ZO_ACQUIRE_RATE_MAX scaled by heading quality and by
    /// the roll spread of the last completed fit.
    float zeroAcquireRate() const;

    float    zoBinSum[ZO_BINS];        // Î£ (reported angle âˆ’ bA) per roll bin
    uint16_t zoBinCount[ZO_BINS];      // samples per roll bin
    // Window-level sum of the GPS-implied angle that was SUBTRACTED sample by
    // sample (P1). Logged as bAmean: it is the diagnostic that separates "the
    // residual is the zero error" from "the residual is the route's mean
    // curvature leaking in through the old |bA| <= 3 deg gate".
    float    zoBASum;
    uint32_t zoBACount;
    uint16_t zoSamples;                // samples in the current window
    float    zoRollMin, zoRollMax;     // spread tracking for the window

    float    zoResidualDeg;            // last fitted intercept
    float    zoGainDegPerDeg;          // last fitted slope
    // Ring of recent per-window gain candidates; zoGainDegPerDeg is their MEDIAN.
    // Kept as a ring rather than a running average so that one wild window cannot
    // move the value at all, while ZO_GAIN_MEDIAN_N windows still replace it completely.
    float    zoGainCandDegPerDeg[ZO_GAIN_MEDIAN_N];
    // Median of that ring (P2): REPORTED, never applied. The web UI and the fit
    // line print it next to the configured value so a wrong config - or a wrong
    // derived sign - is visible from one log instead of by trial and error.
    float    zoGainReportedDegPerDeg;
    uint8_t  zoGainCandHead;
    uint8_t  zoGainCandCount;
    float    zoRollMeanDeg;            // mean roll of the LAST COMPLETED fit (logged)
    uint8_t  zoFits;                   // completed fits this session
    uint8_t  zoStableFits;             // consecutive fits with |residual| <= tolerance
    uint8_t  zoReopenFits;             // consecutive fits with a large residual
    uint8_t  zoStopFits;               // consecutive fits deviating from the prediction
    // Filtered, TRIM-INVARIANT estimate of the mechanical zero error:
    //     errEst = measured residual + encoderOffset
    // which does not move while the trim works, so a long average of it converges
    // instead of chasing its own correction. zoResidualDeg is then the remaining error
    // (errEst - encoderOffset), i.e. what the trim actually has to null.
    float    zoErrorEstDeg;
    float    zoFitDeviationDeg;        // |raw estimate - filtered estimate| of the last fit
    uint8_t  zoLastFitBins;            // non-empty bins in the LAST COMPLETED fit
    float    zoLastFitSpreadDeg;       // roll spread of the LAST COMPLETED fit

    uint8_t  zoState;
    bool     zoGuidanceActive;         // pushed in by AutosteerProcessor
    bool     zoAogZeroActive;          // pushed in by AutosteerProcessor
    // Quality signals, cached once per cycle by updateZeroObserver(): zeroAcquireRate()
    // is const and is also called from the const status getter, so it must not reach
    // into the GNSS processor itself.
    uint8_t  zoSatsUsed;
    uint8_t  zoQualityUsed;            // HPR QF when the dual heading is in use, else GGA
    bool     zoUsedDualHeading;

    // Long-baseline straightness, computed in updateGPSAngle() from the GPS ring
    // buffer (the short 0.10 m gpsAngle is reported alongside for comparison).
    float    zoStraightBaselineAngleDeg; // net path bend over ZO_STRAIGHT_BASELINE_M
    float    zoStraightBaselineDistM;
    bool     zoStraightBaselineValid;

    // Standstill detection, computed in updateGPSAngle() from the same ring
    // buffer. Read by the steering gate in AutosteerProcessor to keep the motor
    // free at standstill WITHOUT a speed gate that real creep could not pass.
    float    standstillNetM;           // net displacement over the window (m)
    bool     standstillValid;          // a long enough window exists
    float    zoTrimAppliedDeg;         // total |trim| applied since the last mechanical zero
    float    zoTrimSinceFitDeg;        // |trim| applied since the residual was last refreshed
    float    zoPrevResidualDeg;        // previous fit, for the consistency check
    float    zoOffsetAtPrevFit;        // encoderOffset (SIGNED accumulated trim) at that fit
    bool     zoArmed;                  // two consecutive fits agreed -> allowed to trim
    uint32_t zoTrims;                  // number of trim steps
    uint32_t zoLastTrimMs;
    uint32_t zoLastFitMs;              // when the residual was last refreshed
    // Timestamp of the FIRST accepted sample of the current window, so the periodic
    // status line can report the accepted-sample RATE (nacc). That number is what
    // says whether a window fills in 2 s (100 Hz accepted) or in 5 s (40 Hz accepted,
    // i.e. the straight gate closed most of the time) - the fit line alone cannot
    // show it, because it only reports the count once the window is full anyway.
    uint32_t zoWindowStartMs;

    /// Collect one sample and run a fit when the window is full.
    void updateZeroObserver();
    /// Weighted least squares over the roll bins. Returns false when the window
    /// does not carry enough data or enough roll spread to separate gain from
    /// offset (in which case nothing may be changed).
    bool fitZeroObserverWindow();
    void resetZeroObserverWindow();
    
    // Kalman filter state
    float fusedAngle;        // X - Current angle estimate
    float predictedAngle;    // Xp - Predicted angle
    float uncertainty;       // P - Estimation error covariance
    float kalmanGain;        // K - Kalman gain
    
    // Sensor angles
    float encoderAngle;      // Angle from motor encoder
    float gpsAngle;          // Angle from GPS heading rate
    bool gpsAngleValid;      // Is GPS angle valid this update
    
    // Motor position tracking
    int32_t lastEncoderPosition;
    float encoderOffset;      // Long-term GPS drift offset (degrees). Was int32_t - a bug,
                               // since it silently truncated fractional-degree offsets.
    // Raw encoder count accumulator (driver-native sign, invert applied only
    // when the angle is computed) and the value captured at wheels-straight.
    // encoderAngle is always derived from these, so countsPerDegree can change
    // at any time without corrupting the reported angle.
    int32_t rawCounts;
    int32_t centerCounts;
    // Persisted separately from Config (see AutosteerProcessor::saveVWASConfig):
    // Config is pushed to AOG's settings on save, so the straight-ahead
    // reference lives outside it. Without this the reference was silently 0
    // after every boot - the 2026-09-21 log shows centerCounts=62 in the session
    // that captured it and ctr=0 after the restart, i.e. a ~4 deg phantom angle
    // while the wheels were straight.

    // AgOpenGPS-owned steering scale and zero offset (PGN 252). Not persisted:
    // AOG is the owner, ConfigManager reloads them from EEPROM at boot and
    // handleSteerSettings() refreshes them on every receipt.
    uint8_t aogCountsPerDegree;
    int16_t aogWasOffset;
    bool    aogInvertWas;         // AgOpenGPS's PGN 251 "Invert WAS"
    
    // GPS angle calculation
    float vehicleSpeed;      // Current speed (m/s), smoothed 80/20 EMA (matches AgOpenGPS)
    bool vehicleSpeedInit;
    float headingRate;       // Rate of heading change (deg/s) - diagnostic only now
    float lastHeading;       // Previous heading for rate calculation
    // Ring buffer of recent GPS samples (position + heading + time), used to
    // look BACK for a sample far enough away (>= minDistanceForCurvature)
    // instead of waiting forward for enough new distance to accumulate.
    // This gives a fresh candidate on every GPS update instead of only
    // once every ~minDistanceForCurvature/speed seconds - important at
    // very low speed (~0.04 m/s => ~2.5s between updates otherwise).
    static const uint16_t GPS_HISTORY_SIZE = 300; // ~15-30s of history at 10-20Hz
    struct GPSHistorySample {
        double lat;
        double lon;
        float heading;
        uint32_t timeMs;
    };
    GPSHistorySample gpsHistory[GPS_HISTORY_SIZE];
    uint16_t gpsHistoryHead;   // next write index
    uint16_t gpsHistoryCount;  // number of valid entries (caps at GPS_HISTORY_SIZE)
    float lastDistanceTraveled;  // Distance (m) since last curvature sample - diagnostic
    bool usingDualHeadingNow;    // Diagnostic: which heading source was used last cycle
    uint8_t lastHeadingQualityUsed; // Diagnostic
    float rollDeg = 0.0f;        // Roll (deg) from the heading source in use - diagnostic,
                                 // reported via getRollDeg() but never applied (AOG owns
                                 // side-hill compensation). Inline-initialized so it is
                                 // always defined even before the first GPS update.
    bool rollValid = false;      // Is rollDeg meaningful this cycle
    float gpsAngleSmoothed;      // EMA-smoothed heading-derived angle
    bool gpsAngleSmoothedInit;
    float gpsIntegratedAngle;    // Articulated mode only: currentHeading minus
                                  // referenceHeadingArticulated (wrapped), NOT an
                                  // incremental sum - see referenceHeadingArticulated.
    bool gpsIntegratedAngleInit;
    float referenceHeadingArticulated; // Front-unit heading captured once at
                                  // calibration/first-fix, used as the articulated
                                  // "straight ahead" reference. Computing
                                  // (currentHeading - this) directly, rather than
                                  // summing many small deltas, avoids double-
                                  // counting when history lookback windows overlap.
    bool haveReferenceHeadingArticulated;
    uint32_t lastDriftCorrectionTime; // For rate-limiting using actual elapsed
                                  // time between GPS-valid samples, not the
                                  // fixed per-cycle dt (GPS updates can be much
                                  // rarer than the 100Hz fusion loop, especially
                                  // at very low speed / large minDistanceForCurvature)
    
    // Adaptive variance calculation
    float measurementVariance;
    float* varianceBuffer;   // Circular buffer for variance calculation
    uint16_t varianceIndex;
    uint16_t varianceCount;
    
    // Drift compensation
    float encoderDrift;      // Accumulated drift (degrees)
    float driftRate;         // Drift rate (degrees/second)
    uint32_t driftStartTime;
    
    // Timing
    uint32_t lastUpdateTime;
    uint32_t lastGPSTime;
    
    // Private methods
    void updateEncoderAngle();
    void updateGPSAngle();
    // Reliability of the wheel angle as a model input (0...100), pure function of the
    // zero observer state - no time decay on purpose: once learned the zero is a
    // MACHINE CONSTANT, only "never learned" and unstable fits make it unusable.
    float computeAngleQuality() const;
    void updateKalmanFilter(float dt);
    void updateVariance();
    void updateDriftCompensation(float dt);
    float calculateGPSAngleFromHeadingRate(float headingRate, float speed);
    bool isValidGPSConditions() const;
};

// Global instance pointer for external access
extern WheelAngleFusion* wheelAngleFusionPtr;

#endif // WHEEL_ANGLE_FUSION_H
