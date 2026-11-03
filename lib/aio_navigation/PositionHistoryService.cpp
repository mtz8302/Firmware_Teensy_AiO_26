#include "PositionHistoryService.h"
#include "GNSSProcessor.h"
#include "IMUProcessor.h"
#include "HeadingBridge.h"

// =================================================================================
// Compile-time guard (2026-10-04). The record's source tags MUST stay equal to
// HeadingBridge::Source, or the log's src column silently changes meaning. The first draft
// of the enum had 6 entries with PROJ=5, which would have logged a projected heading as
// "imuhdg". If the bridge ever reorders its enum, the build fails HERE instead of in a field
// log three weeks later.
// =================================================================================
static_assert((int)PositionHistory::HDG_NONE       == (int)HeadingBridge::Source::NONE,           "hdgSrc: NONE drift");
static_assert((int)PositionHistory::HDG_ATTITUDE   == (int)HeadingBridge::Source::ATTITUDE,       "hdgSrc: ATTITUDE drift");
static_assert((int)PositionHistory::HDG_PROJECTED  == (int)HeadingBridge::Source::PATH_PROJECTED, "hdgSrc: PATH_PROJECTED drift");
static_assert((int)PositionHistory::HDG_FIX2FIX    == (int)HeadingBridge::Source::PATH_FIX2FIX,   "hdgSrc: PATH_FIX2FIX drift");
static_assert((int)PositionHistory::HDG_COG        == (int)HeadingBridge::Source::PATH_COG,       "hdgSrc: PATH_COG drift");
static_assert((int)PositionHistory::HDG_IMU        == (int)HeadingBridge::Source::PATH_IMU_HEADING, "hdgSrc: PATH_IMU_HEADING drift");
static_assert((int)PositionHistory::HDG_DEADRECKON == (int)HeadingBridge::Source::DEADRECKON,     "hdgSrc: DEADRECKON drift");
// rollSrc is a plain uint8_t in the bridge (HeadingBridge.h: "0 = none, 1 = dual antenna,
// 2 = IMU, 3 = held"), so it cannot be asserted - PositionHistory::RollSrc uses those numbers.
static_assert((int)PositionHistory::ROLL_NONE == 0 && (int)PositionHistory::ROLL_DUAL == 1 &&
              (int)PositionHistory::ROLL_IMU  == 2 && (int)PositionHistory::ROLL_HELD == 3,
              "rollSrc: bridge contract is 0=none 1=dual 2=IMU 3=held");

// =================================================================================
// R1b (2026-10-18, B10.47) - nur PLAUSIBLE Positionen kommen in den Ring.
// ---------------------------------------------------------------------------------
// Anlass: Fahrt "2026-10-18 14-59 ... flip bei Bruecke". Unter der Bruecke brach die
// Empfaengerloesung zusammen (fixQ 4->1, uni_hdgsd 70...100 Grad, baseline_dev 195 %,
// Positionen sprangen 0,43...0,61 m bei 1,2 m/s). Die Bridge verliess LIVE korrekt,
// nahm dann aber den RING-KURS (az_src = 1), und der war aus den gesprungenen Fixes
// gebaut: 143 Grad falsch, 4,4 s lang (groesster az_out-Schritt 148 Grad in 0,5 s).
//
// Deshalb: ein Fix kommt nur in den Ring, wenn der Schritt vom zuletzt AUFGENOMMENEN
// Fix plausibel ist - dieselbe Formel wie HeadingBridge::maxPosStepAllowedM, aber mit
// einer Standstill-Untergrenze.
//
// WARUM die Untergrenze (Betreiberfrage 10-18: "was passiert, wenn sich beim Einschalten
// die Qualitaet verbessert und die Position deshalb springt?"): beim Einschalten wird
// float -> RTK-fixed und die Position springt 0,3...2 m. Das ist ein GUTER Fix. Mit
// 2*v*dt + 0,15 allein waere er bei v = 0 (0,15 m) verworfen - und weil jeder Folgefix
// 0,8 m vom alten Anker entfernt bleibt, waehrend die Schwelle nur mit 2*v*dt waechst,
// waere der Ring DAUERHAFT eingefroren (jede ringbasierte Logik saehe nur noch Alter und
// die Bridge bliebe in HALTEN stehen).
//
// Ein Sprung wird deshalb nur VERWORFEN, solange er nicht belegt ist:
//   * DAUERHAFTIGKEIT (empfaengerunabhaengig, gilt auch fuer 2x F9P und F9P + IMU):
//     die neue Stelle bleibt kRingReanchorMs lang stabil (<= kRingReanchorM) -> sie ist
//     die neue Wahrheit, nicht eine Spitze,
//   * STICHENTSCHEID QUALITAET (nur wo der Empfaenger eine Basislinie liefert):
//     fixQ >= 4 UND |baselineDevPct| <= 3 * baselineTolPct -> die Loesung ist gut, also
//     war es ein RTK-Einschaltsprung. Bei F9P + IMU ist baselineLengthM = 0 und
//     baselineDevPct bleibt 0 -> das Kriterium feuert dort nie (und muss es nicht, weil
//     die Dauerhaftigkeit empfaengerunabhaengig greift).
//
// Wird NEU VERANKERT, ist die ALTE Historie wertlos (der Kurs liefe ueber den Sprung)
// -> positionHistory.reset() (dieselbe Funktion wie bei Link-Verlust). Die Sehne startet
// dann aus den neuen, guten Fixes; bis dahin haelt die Bridge (az_src = 3 bzw. die
// 1-s-Bestaetigung der Quelle).
//
// Bewusst NICHT hier:
//   * eine Kurs-Plausibilitaet - die sitzt in updatePathChord (Sprungtor, HeadingBridge),
//   * ein Positions-Fallback - die VEROEFFENTLICHTE Position folgt immer dem neuesten
//     Fix; ein Offset waere schlimmer als ein Sprung (Betreiber-Regel 10-18).
// =================================================================================
static constexpr float    kRingStepFloorM   = 0.50f;  // Standstill-Untergrenze (m)
static constexpr float    kRingStepVDouble  = 2.0f;   // wie maxPosStepAllowedM
static constexpr float    kRingStepSlackM   = 0.15f;  // wie maxPosStepAllowedM
static constexpr float    kRingReanchorM    = 0.25f;  // "dieselbe neue Stelle" (m)
static constexpr uint32_t kRingReanchorMs   = 300u;   // ... so lange stabil (3 Epochen @10 Hz)
static constexpr float    kRingStandstillMps = 0.30f; // darunter gilt die Untergrenze

static bool     s_suspectValid = false;   // laeuft gerade eine Sprungserie?
static double   s_suspectLat   = 0.0;
static double   s_suspectLon   = 0.0;
static uint32_t s_suspectMs    = 0;
static uint32_t s_ringRejected = 0;       // Fixes, die NICHT in den Ring kamen
static uint32_t s_ringReanchor = 0;       // Neu-Anker (reset + uebernehmen)

// Meter-Differenz - dieselbe Naeherung wie HeadingBridge::localDeltaMeters.
static inline void ringDeltaMeters(double lat1, double lon1, double lat2, double lon2,
                                   double& dN, double& dE) {
    dN = (lat2 - lat1) * 111320.0;
    dE = (lon2 - lon1) * 111320.0 * cos(lat1 * 0.017453292519943295);
}

// ---------------------------------------------------------------------------------
// The ring's own log line - one per STORED record.
//
// Same discipline as HeadingBridge::logRawFrame(): a plain Serial.printf that never blocks
// (usb_serial_write() waits up to 120 ms when the TX buffer is full, so the line is only
// produced while the driver reports room, and with no host attached it pauses silently).
// Gate: the bridge's EXISTING raw-log switch, so the operator keeps ONE place for diagnostic
// logging instead of a new EEPROM field.
//
// Prefix "PH," is deliberate: every existing analysis script filters on "HR," and keeps
// working unchanged. Rate = one line per FIX = the same 10 Hz as the HR line, so the two can
// be aligned on t_ms.
//
// Columns (CSV, no header):
//   t_ms, lat, lon, v_mps, wheel_deg, wheel_q, psi_deg, hdg_src, roll_deg, roll_src,
//   dual_az, cog_raw, imu_hdg, imu_roll, imu_age_ms, fixQ, dualHdgQ, pos_sigma_cm,
//   flags, ring_n, pos_cnt, ring_push, ring_dup, hdg_sd_deg, base_cm, ring_rej, ring_rean
// ring_rej / ring_rean (B10.47): Fixes, die das Plausibilitaets-Tor verworfen hat, und
// Neu-Anker (Ring geleert + Fix uebernommen) - siehe Kopf dieser Datei.
// The last two are the per-epoch ATTITUDE quality the record carries (2026-10-05): the
// receiver's heading 1 sigma in degrees and the solved baseline in cm, both 0 = not provided.
// hdg_src is TEXT - the same string the HR src column carries (same values by construction,
// see the static_asserts); roll_src is numeric: 0 none, 1 dual, 2 IMU, 3 held; flags is
// 0x.... with the bit meanings from PositionHistory.h.
// ring_push/ring_dup are the ring counters, so the 100 Hz call rate becomes visible in the
// log (expect ~9 duplicates per fix) instead of being a claim in a comment.
// ---------------------------------------------------------------------------------
static void FLASHMEM logRingLine(const PositionHistory::FixSample& s) {
#if defined(AIO_HR_LOG) && ((AIO_HR_LOG == 0) || (AIO_HR_LOG == 1))
    // B10.49 (2026-10-19, Betreiber): PH-Zeile AUS (Schalter in platformio.ini).
    //   AIO_HR_LOG 0 = alles aus (Produktiv), 1 = nur HR, 2 = nur PH, 3 = beides.
    (void)s;
    return;
#endif
    if (!headingBridge.getConfig().logRawFrames) return;
    if (Serial.availableForWrite() < 256) return;   // never wait for the host
    // B10.49: siehe logRawFrame - erst in einen Puffer, dann in EINEM Stueck schreiben,
    // damit ein voller USB-Puffer die Zeile nicht mittendrin blockiert.
    DMAMEM static char phBuf[512];   // DMAMEM = RAM2, damit RAM1 (Stack) frei bleibt
    const int phN = snprintf(phBuf, sizeof(phBuf),
                  "PH,%lu,%.7f,%.7f,%.3f,%.2f,%u,%.2f,%s,%.2f,%u,"
                  "%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,0x%04X,%u,%u,%lu,%lu,%.2f,%u,%lu,%lu\r\n",
                  (unsigned long)s.tMs, s.lat, s.lon, (double)s.speedMps,
                  (double)s.wheelDeg, (unsigned)s.wheelQPct,
                  (double)s.bodyHdgDeg,
                  HeadingBridge::sourceName((HeadingBridge::Source)s.hdgSrc),
                  (double)s.rollDeg, (unsigned)s.rollSrc,
                  (double)s.dualAzDeg, (double)s.cogRawDeg,
                  (double)s.imuHdgDeg, (double)s.imuRollDeg,
                  (unsigned)s.imuAgeMs, (unsigned)s.fixQ, (unsigned)s.dualHdgQ,
                  (unsigned)s.posSigmaCm, (unsigned)s.flags,
                  (unsigned)positionHistory.count(), (unsigned)s.posCount16,
                  (unsigned long)positionHistory.pushes(),
                  (unsigned long)positionHistory.duplicatesSkipped(),
                  (double)(s.hdgSdX100 * 0.1f), (unsigned)s.baselineCm,
                  (unsigned long)s_ringRejected, (unsigned long)s_ringReanchor);
    if (phN <= 0 || phN >= (int)sizeof(phBuf)) return;
    if (Serial.availableForWrite() < phN + 16) return;
    Serial.write((const uint8_t*)phBuf, (size_t)phN);
}

// =================================================================================
// The record is assembled from all four live sources at once; this file is the only place
// that knows them together, so no consumer has to know about the others.
//
// Ordering (the whole point): called AFTER headingBridge.update() from
// AutosteerProcessor::process(), which means
//   * bodyHdgDeg == the psi the bridge just published == the log's psi_deg column,
//   * wheelDeg   == currentAngle == the angle the bridge model used this tick,
//   * the record is created on the FIRST tick that sees the new fix, so attitudeTrusted /
//     rollSource / direction in it belong to that fix.
// See knowledge/position_ring_buffer_architecture.md section 7.
// =================================================================================
bool FLASHMEM positionHistoryService(uint32_t now, float wheelDeg, float wheelQualPct) {
    const GNSSProcessor::GNSSData& d  = gnssProcessor.getData();
    const HeadingBridge::Status&   bs = headingBridge.getStatus();
    const IMUData imu = imuProcessor.getCurrentData();

    PositionHistory::FixSample s = {};   // value-init: every field incl. pad is defined

    // --- position and time ------------------------------------------------------
    s.lat = d.latitude;              // double end to end - the input path is double already,
    s.lon = d.longitude;             // and float32 position is 0.64 m resolution (unusable)
    s.tMs = now;
    s.posCount16 = (uint16_t)gnssProcessor.getPositionCount();

    // --- raw measurements, one field per source ---------------------------------
    s.dualAzDeg  = d.dualHeading;              // RAW baseline azimuth (the log's att_hdg_deg
                                               // is the offset-corrected one)
    s.cogRawDeg  = d.headingTrue;              // RAW VTG course (the log's cog_deg is FILTERED)
    s.imuHdgDeg  = imu.heading;                // RAW, offset free
    s.imuRollDeg = imu.roll;                   // RAW (cross-check, or the only roll on an F9P)
    s.speedMps   = d.speedKnots * 0.514444f;   // the factor PWMProcessor/WheelAngleFusion use
    const float sigmaM = headingBridge.getPositionSigmaM();
    s.posSigmaCm = (sigmaM > 0.0f)
                   ? (uint16_t)((sigmaM > 655.35f) ? 65535.0f : sigmaM * 100.0f)
                   : 0;                        // 0 = unknown
    const uint32_t imuAge = imuProcessor.getTimeSinceLastPacket();
    s.imuAgeMs = (imuAge > 65535u) ? (uint16_t)65535u : (uint16_t)imuAge;

    // --- used values, each with its source tag ----------------------------------
    s.bodyHdgDeg = headingBridge.getVehicleHeadingDeg();
    s.hdgSrc     = (uint8_t)bs.source;
    s.rollDeg    = bs.rollDeg;
    s.rollSrc    = bs.rollSource;
    s.wheelDeg   = wheelDeg;
    const int wq = (int)(wheelQualPct + 0.5f);
    s.wheelQPct  = (wheelQualPct < 0.0f) ? 255u
                 : (uint8_t)((wq < 0) ? 0 : ((wq > 100) ? 100 : wq));

    // --- quality ----------------------------------------------------------------
    s.fixQ     = d.fixQuality;
    s.dualHdgQ = d.headingQuality;
    // Per-epoch ATTITUDE quality, RAW from the receiver (see PositionHistory.h). 0 = "not
    // provided" - the same convention GNSSData::baselineLengthM uses - so an F9P without a dual
    // baseline reports 0 and every consumer knows it is unknown rather than "perfect".
    s.hdgSdX100 = (d.uniHdgStdDevDeg >= 25.5f) ? 255u
                : (uint8_t)(d.uniHdgStdDevDeg * 10.0f + 0.5f);
    const float baseCm = d.baselineLengthM * 100.0f;
    s.baselineCm = (baseCm <= 0.0f) ? 0u
                 : ((baseCm >= 65535.0f) ? 65535u : (uint16_t)(baseCm + 0.5f));

    // --- status flags -----------------------------------------------------------
    const HeadingBridge::Config& bc = headingBridge.getConfig();
    uint16_t f = PositionHistory::F_VALID;
    if (d.hasPosition && d.fixQuality > 0) f |= PositionHistory::F_POS_OK;
    if (d.fixQuality == 4)                 f |= PositionHistory::F_POS_RTK;
    // Dual attitude: EXACTLY the gate the bridge applies to an HPR/UNIHEADINGA frame, so
    // F_DUAL_ATT_OK can never disagree with the filter about what "usable attitude" means.
    const uint8_t qf = d.headingQuality;
    if ((d.hasDualHeading || d.hasINS) && qf < 16 &&
        (bc.headingQualityMask & (1u << qf)) != 0) {
        f |= PositionHistory::F_DUAL_ATT_OK;
    }
    // COG: the bridge's own validity rule (fresh VTG + fix + speed). A frozen COG behind a
    // stopped VTG is exactly the trap this ring exists to make visible.
    if (gnssProcessor.getVtgAgeMs() <= bc.maxCogAgeMs &&
        d.fixQuality > 0 && s.speedMps > bc.minSpeedForCogUse) {
        f |= PositionHistory::F_COG_OK;
    }
    // IMU: one packet carries heading AND roll, so both raw bits are the same freshness test
    // (the 100 ms IMUProcessor::isActive() uses). The USED roll has its own tag + F_ROLL_OK.
    if (imu.isValid && s.imuAgeMs <= 100) {
        f |= PositionHistory::F_IMU_HDG_OK | PositionHistory::F_IMU_ROLL_OK;
    }
    if (bs.rollSource != 0 && !bs.rollSuspect) f |= PositionHistory::F_ROLL_OK;
    if (bs.reverseDirection) f |= PositionHistory::F_REVERSING;
    if (bs.dirChanging)      f |= PositionHistory::F_DIR_CHANGING;
    if (bs.attitudeTrusted)  f |= PositionHistory::F_ATT_TRUSTED;
    // Step 2 (2026-10-13, B10.40): the bridge's own step test for the B-chain chord runs
    // BEFORE this push on the SAME tick (HeadingBridge first, this service last - see the
    // ordering note above), so its verdict belongs to THIS fix. A too-large fix-to-fix step
    // marks the record: consumers can skip it without repeating the test, and the value
    // that failed is in the HR column pc_step (pc_ok 0 = held, see updatePathChord).
    if (bs.pathChordSpike)   f |= PositionHistory::F_SPIKE_REJECTED;
    // Deliberately NOT set here:
    //   F_POS_STALE      - a record is only created ON a fix, so it cannot be stale,
    //   F_STRAIGHT       - a property of the CHORD through this sample (consumers, Step 2).
    s.flags = f;

    // One record per REAL fix, as the note requires: the epoch counter advances even without a
    // position (a no-fix GGA carries the same time of day), and the 20-35 log showed what that
    // caused - records with lat/lon 0 filling the ring. The gap now stays visible as a jump in
    // tMs instead of as empty records, which is also what lookback() expects.
    if ((f & (PositionHistory::F_VALID | PositionHistory::F_POS_OK)) !=
        (PositionHistory::F_VALID | PositionHistory::F_POS_OK)) {
        return false;
    }

    // --- R1b: Plausibilitaets-Tor (Begruendung im Kopf dieser Datei) -------------
    // Nur ein Fix, der vom zuletzt AUFGENOMMENEN Fix aus plausibel ist, kommt in den
    // Ring. Ein belegter Sprung (dauerhaft ODER mit guter Loesung) wird uebernommen und
    // ankert den Ring neu - sonst wuerde ein RTK-Einschaltsprung den Ring einfrieren.
    const PositionHistory::FixSample* newestStored = positionHistory.newest();
    if (newestStored != nullptr) {
        double dN = 0.0, dE = 0.0;
        ringDeltaMeters(newestStored->lat, newestStored->lon, s.lat, s.lon, dN, dE);
        const float stepM = (float)sqrt(dN * dN + dE * dE);
        const float dtS   = (float)(now - newestStored->tMs) * 0.001f;
        float allowedStepM = (kRingStepVDouble * s.speedMps * dtS) + kRingStepSlackM;
        // Die Untergrenze gilt NUR im Stillstand: dort ist 2*v*dt+0,15 = 0,15 m und der
        // RTK-Einschaltsprung (0,3...2 m) waere verworfen. In FAHRT ist sie schaedlich:
        // unter der Bruecke sprangen die Fixes 0,43...0,61 m bei 1,2 m/s - mit 0,50 m
        // Untergrenze kamen genau diese Fixes durch und der Kurs drehte 143 Grad
        // (Replay-Messung 10-18: Tor wirkungslos).
        if (s.speedMps < kRingStandstillMps && allowedStepM < kRingStepFloorM) {
            allowedStepM = kRingStepFloorM;
        }
        if (dtS <= 0.0f || dtS >= 10.0f) allowedStepM = kRingStepFloorM;  // Zeit unbekannt
        if (stepM > allowedStepM) {
            bool sustained = false;
            if (s_suspectValid) {
                double sN = 0.0, sE = 0.0;
                ringDeltaMeters(s_suspectLat, s_suspectLon, s.lat, s.lon, sN, sE);
                sustained = ((float)sqrt(sN * sN + sE * sE) <= kRingReanchorM) &&
                            ((uint32_t)(now - s_suspectMs) >= kRingReanchorMs);
                if (!sustained) {              // andere Stelle: Verdacht neu beginnen
                    s_suspectLat = s.lat;
                    s_suspectLon = s.lon;
                    s_suspectMs  = now;
                }
            } else {
                s_suspectValid = true;
                s_suspectLat = s.lat;
                s_suspectLon = s.lon;
                s_suspectMs  = now;
            }
            // Stichentscheid: gute Loesung => RTK-Einschaltsprung, kein Empfaengermuell.
            const bool qualGood = (d.fixQuality >= 4) &&
                                  (fabsf(bs.baselineDevPct) <= (3.0f * bc.baselineTolPct));
            if (!(sustained || qualGood)) {
                s_ringRejected++;
                return false;                  // dieser Fix kommt NICHT in den Ring
            }
            positionHistory.reset();           // alte Historie ist wertlos
            s_ringReanchor++;
            s_suspectValid = false;
        } else {
            s_suspectValid = false;            // normaler Fix: Verdacht verfaellt
        }
    }

    const bool stored = positionHistory.push(s);
    // The line is produced HERE and not by a consumer, because this is the only place that
    // knows a record was actually stored - and it makes the rate one line per FIX (10 Hz)
    // without a throttle. See logRingLine() below.
    if (stored) logRingLine(s);
    return stored;
}
