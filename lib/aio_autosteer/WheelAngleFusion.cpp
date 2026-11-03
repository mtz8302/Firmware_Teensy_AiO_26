// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// WheelAngleFusion.cpp - Implementation of sensor fusion for wheel angle
#include "WheelAngleFusion.h"
#include "MotorDriverInterface.h"
#include "GNSSProcessor.h"
#include "IMUProcessor.h"
#include "HeadingBridge.h"   // qualified heading/roll (fall-back module)
#include <cmath>
#include <algorithm> // For constrain, fmax, fmin

// ---------------------------------------------------------------------------
// B10.73 (2026-11-01, Betreiber): DRIFT-REGLER ALS KONSTANTEN.
// Diese fuenf Werte standen frueher im Web-IO (Abschnitt "Drift-Korrektur").
// Sie sind NICHT optional - sie erzeugen gpsAngle ("gps_a"), und das ist seit
// B10.66 die Knickwinkel-Quelle des Knicklenkers. Werte unveraendert uebernommen,
// nur nicht mehr verstellbar.
// ---------------------------------------------------------------------------
static constexpr float kMinDistanceForCurvature      = 0.10f;   // m zwischen zwei Samples
static constexpr float kGpsAngleSmoothingTau         = 1.00f;   // s, EMA des GPS-Winkels
static constexpr float kMaxDriftCorrectionRate       = 0.50f;   // Grad/s Offset-Nachfuehrung
static constexpr float kDriftInnovationRejectThreshold = 5.0f;  // Grad Ausreisser-Schwelle
static constexpr float kMaxPlausibleSpeed            = 5.00f;   // m/s Untergrenze der Schranke


// Global instance pointer
WheelAngleFusion* wheelAngleFusionPtr = nullptr;

// External sensor instances
extern GNSSProcessor* gnssProcessorPtr;
extern IMUProcessor imuProcessor;

// One character for the HeadingBridge state, for the periodic status line.
//
// The bridge is IN this class's heading and roll path (see the block in
// updateGPSAngle that replaces currentHeading/rollDeg), and the zero-point observer
// deliberately keeps SAMPLING while the heading is synthesised - it then runs on COG
// assumptions (usingDualHeadingNow = false, so the higher COG speed floor applies and
// zoQualityUsed becomes the GGA fix quality). So a zero estimate obtained during a
// bridge is built on a modelled heading, and that has to be visible in the same line
// as the estimate itself rather than inferred from a second log.
//
// L/LIVE is the normal case: a measured attitude.
static char headingBridgeStateChar() {
    switch (headingBridge.getStatus().state) {
        case HeadingBridge::State::LIVE:       return 'L';
        case HeadingBridge::State::BRIDGE:     return 'B';
        case HeadingBridge::State::COAST:      return 'C';
        case HeadingBridge::State::DEADRECKON: return 'D';
        case HeadingBridge::State::TIMEOUT:    return 'T';
        default:                               return 'I';   // INIT
    }
}


static float normalizeAngle(float a) { // Helper function for angle normalization
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

WheelAngleFusion::WheelAngleFusion() :
    keyaDriver(nullptr),
    gnssProcessor(nullptr),
    imuProcessor(nullptr),
    fusedAngle(0.0f),
    predictedAngle(0.0f),
    uncertainty(10.0f),
    kalmanGain(0.0f),
    encoderAngle(0.0f),
    gpsAngle(0.0f),
    gpsAngleValid(false),
    lastEncoderPosition(0),
    encoderOffset(0.0f),
    rawCounts(0),
    centerCounts(0),
    aogCountsPerDegree(0),
    aogWasOffset(0),
    aogInvertWas(false),
    vehicleSpeed(0.0f),
    vehicleSpeedInit(false),
    headingRate(0.0f),
    lastHeading(0.0f),
    gpsHistoryHead(0),
    gpsHistoryCount(0),
    lastDistanceTraveled(0.0f),
    usingDualHeadingNow(false),
    lastHeadingQualityUsed(0),
    gpsAngleSmoothed(0.0f),
    gpsAngleSmoothedInit(false),
    gpsIntegratedAngle(0.0f),
    gpsIntegratedAngleInit(false),
    referenceHeadingArticulated(0.0f),
    haveReferenceHeadingArticulated(false),
    lastDriftCorrectionTime(0),
    measurementVariance(1.0f),
    varianceBuffer(nullptr),
    varianceIndex(0),
    varianceCount(0),
    encoderDrift(0.0f),
    driftRate(0.0f),
    driftStartTime(0),
    lastUpdateTime(0),
    lastGPSTime(0)
{
    // Set global pointer
    wheelAngleFusionPtr = this;

    // Zero-point observer: start from a clean, explicitly-initialised state.
    // Done in the body rather than the member init list so the declaration order
    // of these (they sit between `config` and `fusedAngle`) cannot produce a
    // -Wreorder surprise.
    resetZeroObserverWindow();
    zoResidualDeg = 0.0f;
    zoGainDegPerDeg = 0.0f;
    zoGainCandHead = 0;
    zoGainCandCount = 0;
    for (uint8_t i = 0; i < ZO_GAIN_MEDIAN_N; i++) zoGainCandDegPerDeg[i] = 0.0f;
    zoGainReportedDegPerDeg = 0.0f;
    zoRollMeanDeg = 0.0f;
    zoFits = 0;
    zoStableFits = 0;
    zoReopenFits = 0;
    zoStopFits = 0;
    zoErrorEstDeg = 0.0f;
    zoFitDeviationDeg = 0.0f;
    zoLastFitBins = 0;
    zoLastFitSpreadDeg = 0.0f;
    zoState = ZO_IDLE;
    zoGuidanceActive = false;
    zoAogZeroActive = false;
    zoSatsUsed = 0;
    zoQualityUsed = 0;
    zoUsedDualHeading = false;
    zoStraightBaselineAngleDeg = 0.0f;
    zoStraightBaselineDistM = 0.0f;
    zoStraightBaselineValid = false;
    standstillNetM = 0.0f;
    standstillValid = false;
    zoTrimAppliedDeg = 0.0f;
    zoPrevResidualDeg = 0.0f;
    zoOffsetAtPrevFit = 0.0f;
    zoArmed = false;
    zoTrimSinceFitDeg = 0.0f;
    zoTrims = 0;
    zoLastTrimMs = 0;
    zoLastFitMs = 0;
}

bool WheelAngleFusion::init(MotorDriverInterface* motor, GNSSProcessor* gnss, IMUProcessor* imu) {
    LOG_INFO(EventSource::AUTOSTEER, "Initializing Virtual WAS (VWAS)");
    
    // Store sensor interfaces
    keyaDriver = motor;
    gnssProcessor = gnss;
    imuProcessor = imu;
    
    // Validate interfaces
    if (!keyaDriver) {
        LOG_ERROR(EventSource::AUTOSTEER, "VWAS: No motor driver provided");
        return false;
    }

    if (!keyaDriver->hasPositionFeedback()) {
        LOG_ERROR(EventSource::AUTOSTEER, "VWAS: Motor driver (%s) has no position feedback",
                  keyaDriver->getTypeName());
        return false;
    }
    
    if (!gnssProcessor && !imuProcessor) {
        LOG_ERROR(EventSource::AUTOSTEER, "VWAS: No heading rate source (GPS or IMU)");
        return false;
    }
    
    // Allocate variance buffer
    if (varianceBuffer) {
        delete[] varianceBuffer;
    }
    varianceBuffer = new float[config.varianceBufferSize];
    if (!varianceBuffer) {
        LOG_ERROR(EventSource::AUTOSTEER, "VWAS: Failed to allocate variance buffer");
        return false;
    }
    
    // Initialize variance buffer
    for (uint16_t i = 0; i < config.varianceBufferSize; i++) {
        varianceBuffer[i] = 0.0f;
    }
    
    // Initialize Kalman filter state
    fusedAngle = 0.0f;
    predictedAngle = 0.0f;
    encoderAngle = 0.0f;
    gpsAngle = 0.0f;
    uncertainty = config.initialUncertainty;
    
    // Initialize timing
    lastUpdateTime = millis();
    lastGPSTime = millis();
    driftStartTime = millis();
    
    LOG_INFO(EventSource::AUTOSTEER, "Virtual WAS initialized successfully");
    LOG_INFO(EventSource::AUTOSTEER, "  Wheelbase: %.2f m", config.wheelbase);
    // Plan 1: this is the MEASURED value (the recommendation for AOG's "Counts
    // per degree" slider). The scale actually in use comes from AOG's PGN 252
    // and is reported by AutosteerProcessor's startup check.
    LOG_INFO(EventSource::AUTOSTEER, "  Counts/degree (measured): %.2f", config.countsPerDegree);
    LOG_INFO(EventSource::AUTOSTEER, "  Min GPS speed: %.1f m/s", config.minSpeedForGPS);
    
    return true;
}

void WheelAngleFusion::update(float dt) {
    // Update timing
    uint32_t now = millis();
    lastUpdateTime = now;
    
    // Update sensor angles
    updateEncoderAngle();
    updateGPSAngle();
    
    // Run Kalman filter
    updateKalmanFilter(dt);
    
    // Update adaptive parameters.
    // NOTE (Phase 4 audit): this gate is effectively dead code today -
    // updateVariance() unconditionally sets measurementVariance = 1.0f and
    // ignores vehicleSpeed entirely, and kalmanGain is no longer used to
    // blend fusedAngle (see updateKalmanFilter). isValidGPSConditions()
    // therefore only costs a comparison per cycle. It still carries the
    // obsolete minSpeedForGPS term ("kept for now per user request" in the
    // config struct), so do not read it as an active speed gate on the
    // fusion path - the distance/plausibility gates in updateGPSAngle() are
    // the real ones. Left in place rather than removed so the adaptive-
    // variance work can be resumed without a second refactor.
    if (isValidGPSConditions()) {
        updateVariance();
    }
    
    // Update drift compensation
    if (config.enableDriftCompensation) {
        updateDriftCompensation(dt);
    }

    // Zero-point observer (Stufe 2). Every cycle: pure measurement unless the trim
    // is released, and it has to run before the diagnostic block below so that the
    // values it reports belong to this cycle.
    updateZeroObserver();

    // Comprehensive diagnostic log: every input/output of the GPS+encoder
    // fusion in one line, rate-limited so it doesn't flood the log.
    // Compare lastHeading (heading used in the previous cycle's delta) is
    // not stored separately from currentHeading, so we log currentHeading
    // (== gpsData.headingTrue at the time updateGPSAngle() ran this cycle).
    {
        static uint32_t lastDiagLogTime = 0;
        // 0.5 Hz (operator decision 2026-09-23): the 10 Hz raw frame line from
        // HeadingBridge now carries the full data record, so this line only has to
        // keep the VWAS state visible in the session log - and every INFO line it
        // consumes competes with the real events for the 10/s token bucket
        // (EventLogger.h:79).
        if (now - lastDiagLogTime > 2000) {
            lastDiagLogTime = now;
            uint8_t fixQ = gnssProcessor ? gnssProcessor->getData().fixQuality : 0;
            LOG_INFO(EventSource::AUTOSTEER,
                "VWAS diag: type=%d heading=%.2f\u00B0(%s q=%d) roll=%.2f%s "
                "speed=%.3fm/s fixQ=%d gpsHist=%d/%d dist=%.2fm "
                "gpsValid=%d gpsAngle=%.2f\u00B0 enc=%.2f\u00B0 off=%.2f\u00B0 pred=%.2f\u00B0 "
                "fused=%.2f\u00B0 raw=%d ctr=%d cpd=%.2f aogCpd=%u aogOff=%d cal=%d",
                config.vehicleType, lastHeading, usingDualHeadingNow ? "dual" : "COG",
                lastHeadingQualityUsed, rollDeg, rollValid ? "" : "(n/a)",
                vehicleSpeed, fixQ,
                gpsHistoryCount, GPS_HISTORY_SIZE, lastDistanceTraveled,
                gpsAngleValid ? 1 : 0, gpsAngle, encoderAngle, encoderOffset, predictedAngle,
                fusedAngle,
                // Commissioning data (Punkt 11): the raw count accumulator and
                // the straight-ahead reference. Together with cpd these are the
                // ground truth for the endstop calibration - without them a
                // field log cannot be checked for "did the calibration actually
                // produce a sane scale?", which was the original failure.
                (int)rawCounts, (int)centerCounts, config.countsPerDegree,
                (unsigned)aogCountsPerDegree, (int)aogWasOffset,
                config.cpdCalibrated ? 1 : 0);
            // headingRate was dropped from this line on purpose: the log writer cuts
            // every line at 296 characters (measured in the 2026-09-21 16-02 log,
            // where the line ended mid-field at "zRe"), and with the observer added
            // the line had reached that cap. The rate is diagnostic-only and is
            // still recorded in the HeadingMonitor CSV.

            // The zero-point observer gets its OWN line at 0.5 Hz for the same
            // reason: appended to the line above, its fields never reached the log.
            // "win" filling up (0/200 -> 200/200) is the direct evidence that the
            // sampling gate is letting samples through; a permanently 0 window is
            // what the 16-02 run showed.
            static uint32_t lastZeroLogTime = 0;
            if (now - lastZeroLogTime > 2000) {
                lastZeroLogTime = now;
                LOG_INFO(EventSource::AUTOSTEER,
                         "VWAS zero: res=%+.2f gain=%.3f trim=%+.2f used=%.2f gr=%.1f armed=%d "
                         "st=%u fits=%u win=%u/%u nacc=%.0f bst=%c src=%u qf=%u sat=%u rate=%.2f "
                         "roll=%.2f gpsA=%.2f bA=%.2f bD=%.2f spd=%.2f wq=%.0f",
                         zoResidualDeg, zoGainDegPerDeg, encoderOffset, zoTrimAppliedDeg,
                         zoTrimSinceFitDeg,
                         zoArmed ? 1 : 0, (unsigned)zoState, (unsigned)zoFits,
                         (unsigned)zoSamples, (unsigned)ZO_WINDOW_SAMPLES,
                         // Accepted samples per second since the window started. This is
                         // the field that says whether a 200-sample window fills in 2 s
                         // (straight driving available, ~100 Hz accepted) or in 5 s (the
                         // straight gate rejects most samples) - the fit line cannot show
                         // it, because by then the window is full by definition.
                         (zoSamples > 0 && zoWindowStartMs != 0 && now > zoWindowStartMs)
                             ? (float)zoSamples * 1000.0f / (float)(now - zoWindowStartMs)
                             : 0.0f,
                         // Heading source actually used for the samples in this window:
                         // L = measured attitude, B/C/D = synthesised (see
                         // headingBridgeStateChar). The observer keeps sampling during a
                         // bridge, so the reader has to be able to see it.
                         headingBridgeStateChar(),
                         (unsigned)zoUsedDualHeading, (unsigned)zoQualityUsed,
                         (unsigned)zoSatsUsed, zeroAcquireRate(),
                         rollDeg, gpsAngle, zoStraightBaselineAngleDeg,
                         zoStraightBaselineDistM, vehicleSpeed,
                         // Wheel angle reliability for the HeadingBridge (0 = the zero was
                         // never learned, >= 80 = locked). Printed here because this is the
                         // line that already carries the observer's evidence.
                         computeAngleQuality());
            }
        }
    }
}

void WheelAngleFusion::updateEncoderAngle() {
    if (!keyaDriver) {
        encoderAngle = 0.0f;
        return;
    }
    
    // Get position delta from motor (already rollover-safe: the driver
    // reports (int16_t)(pos - lastPos), so a 16-bit counter wrap is handled
    // there and never reaches us as a huge jump).
    int32_t deltaPosition = keyaDriver->getPositionDelta();
    
    // Accumulate RAW counts. This is the ground truth: countsPerDegree is
    // applied only when deriving the angle below, so changing cpd (e.g.
    // during endstop calibration) never rescales or corrupts history.
    rawCounts += deltaPosition;
    
    // Derive the angle from raw counts relative to the straight-ahead
    // reference. invertEncoder flips the sign here (not on the accumulator),
    // so the raw counts stay in the driver's native sign for calibration.
    //
    // UNIFIED WAS CONVERSION (Plan 1). AgOpenGPS owns countsPerDegree and
    // wasOffset; the encoder is treated exactly like the analog ADC reading,
    // which makes this the same expression as ADProcessor::getWASAngle():
    //
    //     position = rawCounts - centerCounts        (signed by invertEncoder)
    //     angle    = (position + wasOffset) / countsPerDegree
    //
    // The '+' is required, not cosmetic: AOG's Zero-WAS does
    //     wasOffset += countsPerDegree * -actualSteerAngleDegrees
    // (FormSteer.cs:903/911) and only converges if the firmware ADDS the
    // offset. With the '-' the error would double on every click.
    float counts = (float)(rawCounts - centerCounts);
    if (effectiveInvertEncoder()) {
        counts = -counts;
    }
    counts += (float)aogWasOffset;
    encoderAngle = counts / getEffectiveCountsPerDegree();
    
    // Log every raw movement (rate-limited) so direction/scale can be verified by hand-turning
    // the wheel with the motor disabled: rawDelta sign/magnitude should match the physical turn,
    // and encoderAngle should flip sign if invertEncoder is wrong.
    if (deltaPosition != 0) {
        static uint32_t lastLogTime = 0;
        static int32_t  lastLoggedCounts = 0;
        uint32_t now = millis();
        int32_t countsSinceLog = rawCounts - lastLoggedCounts;
        if (countsSinceLog < 0) countsSinceLog = -countsSinceLog;
        // Change-gated IN ADDITION to the time floor. The former 100 ms spacing
        // alone fired 10x/s whenever the wheel moved at all, which on its own
        // consumes the entire 10/s INFO token bucket (EventLogger.h:79). With
        // "VWAS diag" (5/s) and "VWAS ctrl" (1/s) the INFO budget was
        // over-subscribed, so messages were silently dropped - preferentially
        // while turning, i.e. exactly when the data matters. Only logging a
        // meaningfully different angle keeps the diagnostic value and leaves
        // budget for everything else.
        // 0.5 Hz time floor (operator decision 2026-09-23): the 10 Hz raw frame line
        // from HeadingBridge carries the wheel angle together with everything else,
        // so this line only has to show that the encoder is alive and counting in
        // the right direction. The change gate (>= 5 counts) stays - that is what
        // makes hand-turning the wheel visible at all.
        if ((now - lastLogTime > 2000) || (countsSinceLog >= 5)) {
            lastLoggedCounts = rawCounts;
            LOG_INFO(EventSource::AUTOSTEER,
                      "VWAS encoder: rawDelta=%d rawCounts=%d center=%d invert=%d(aog=%d) "
                      "cpd=%.1f(aog=%u) aogOff=%d -> encoderAngle=%.2f\u00B0",
                      deltaPosition, rawCounts, centerCounts, effectiveInvertEncoder() ? 1 : 0,
                      config.invertEncoder ? 1 : 0,
                      getEffectiveCountsPerDegree(), (unsigned)aogCountsPerDegree,
                      (int)aogWasOffset, encoderAngle);
            lastLogTime = now;
        }
    }
}

void WheelAngleFusion::updateGPSAngle() {
    // Invalidate the long-baseline straightness FIRST: every early return below
    // (no GNSS, no position) has to leave it invalid rather than let the observer act
    // on the previous cycle's verdict.
    zoStraightBaselineValid = false;
    // Same reasoning for the standstill window: no position means no geometry, and
    // the steering gate must not be handed last cycle's verdict.
    standstillValid = false;

    if (!gnssProcessor) {
        gpsAngleValid = false;
        return;
    }

    const auto& gpsData = gnssProcessor->getData();

    // Vehicle speed - smoothed with the same 80% old / 20% new EMA that
    // AgOpenGPS itself applies to its VTG-derived speed. Our core distance-
    // based curvature calc doesn't actually divide by speed (that's the
    // whole point of the distance-gate rework), so this mainly benefits
    // the diagnostic display and the optional IMU-heading-rate path (which
    // still does divide by speed).
    if (gpsData.hasVelocity) {
        float rawSpeed = gpsData.speedKnots * 0.514444f;  // knots -> m/s
        vehicleSpeed = (vehicleSpeedInit) ? (0.8f * vehicleSpeed + 0.2f * rawSpeed) : rawSpeed;
        vehicleSpeedInit = true;
    } else {
        vehicleSpeed = 0.0f;
    }

    // --- Choose heading source ---
    // Prefer the UM982 dual-antenna heading (gpsData.dualHeading): a true
    // compass reading from the antenna baseline, unlike course-over-ground
    // (gpsData.headingTrue, from VTG) which requires the vehicle to be
    // moving to be accurate.
    // The HPR "QF" value is the receiver's own statement about the baseline
    // solution. A bitmask, not a threshold - see the note on headingQualityMask
    // in the header: the QF scale is not ordered by quality, so ">= n" would
    // accept dead reckoning (6) and SBAS (8/9). The "< 16" guard keeps a
    // garbage/out-of-range QF from shifting out of the mask's range.
    bool headingQualityOk = gpsData.headingQuality < 16 &&
                            (config.headingQualityMask &
                             (1u << gpsData.headingQuality)) != 0;

    // 2026-11-01 (Betreiber): die Checkbox "Dual-Heading bevorzugen" ist entfernt.
    // Sie war NUR im Ersatzfall wirksam (Bridge nicht nutzbar, `bridgeUsable` unten false)
    // und dort ohnehin die richtige Wahl. Die echte Absicherung ist die QF-Maske oben
    // (B8.1: der UM982 sendet bei degradierter Loesung eine GENULLTE Lage mit QF 0).
    // Auf einer Single-Antennen-Maschine ist der Zweig unerreichbar, weil
    // `gpsData.hasDualHeading` false bleibt.
    // Das Config-Feld `preferDualHeading` bleibt ungenutzt stehen: die VWAS-Config liegt
    // als ROHER STRUCT-BLOB im EEPROM (VWAS_CONFIG_ADDR) - ein Feld weniger wuerde alle
    // folgenden Werte verschieben und die gespeicherte Fahrzeug-Geometrie unlesbar machen.
    bool haveGoodDualHeading = gpsData.hasDualHeading &&
                               headingQualityOk;

    // --- Heading/roll fall-back (HeadingBridge) -----------------------------
    // The VWAS uses the heading as a DIFFERENCE over a distance window
    // (dPsi = currentHeading - gpsHistory[idx].heading below) to derive the
    // wheel angle from the curvature. A garbage attitude frame therefore becomes
    // a tens-of-degrees error in the reported angle, and a zeroed sentence
    // (000.0000 / QF 0, measured on every dropout) becomes a 0 ... 180 deg
    // artefact. When the bridge is active its qualified heading is used instead -
    // and while the attitude is gone the bridge supplies the position derived
    // direction, which is a genuine measurement of the path.
    // The mount offset cancels in a difference, so body heading and azimuth are
    // equivalent here; the body heading is used because that is the frame the
    // model state lives in.
    extern HeadingBridge headingBridge;
    const HeadingBridge::Config& bcfg = headingBridge.getConfig();
    const HeadingBridge::Status& bst = headingBridge.getStatus();
    const bool bridgeUsable = bcfg.enabled &&
                              bst.state != HeadingBridge::State::INIT &&
                              bst.state != HeadingBridge::State::TIMEOUT;

    float currentHeading;
    if (bridgeUsable) {
        currentHeading = headingBridge.getVehicleHeadingDeg();
        usingDualHeadingNow = headingBridge.isAttitudeTrusted();
        // Log-only (operator request 2026-09-29): which heading the VWAS actually used. With
        // a single-antenna machine the bridge is usable now (the state machine left INIT),
        // so the fused heading replaces the raw COG - this flag makes that visible in the HR
        // log, because the wheel-angle learning depends on it.
        headingBridge.setVwasHeadingSource(0);
        lastHeadingQualityUsed = headingQualityOk ? gpsData.headingQuality : 0;
        // The bridge always has a roll: live from the LPF, held (then faded)
        // while the attitude is missing - that is the point of the roll channel.
        rollDeg = headingBridge.getRollDeg();
        rollValid = true;
    } else if (haveGoodDualHeading) {
        currentHeading = gpsData.dualHeading;
        usingDualHeadingNow = true;
        lastHeadingQualityUsed = gpsData.headingQuality;
        // The dual-antenna baseline roll is the physically correct roll for
        // this heading, so report it together with it.
        rollDeg = gpsData.dualRoll;
        rollValid = true;
        headingBridge.setVwasHeadingSource(1);
    } else {
        currentHeading = gpsData.headingTrue;
        usingDualHeadingNow = false;
        lastHeadingQualityUsed = 0;
        headingBridge.setVwasHeadingSource(2);
        // Fall back to INS roll if present. COG itself carries no roll, so if
        // neither dual heading nor INS is available there is nothing valid to
        // report (rollValid stays false rather than publishing a stale value).
        if (gpsData.hasINS) {
            rollDeg = gpsData.insRoll;
            rollValid = true;
        } else {
            rollValid = false;
        }
    }

    uint32_t now = millis();

    // Diagnostic-only instantaneous heading rate (not used for the angle
    // calculation any more, only for logging / the optional IMU path).
    //if (lastGPSTime > 0 && now - lastGPSTime < 500) {
    //    float dtRate = (now - lastGPSTime) / 1000.0f;
    //    float headingDeltaForRate = currentHeading - lastHeading;
    //    if (headingDeltaForRate > 180.0f) headingDeltaForRate -= 360.0f;
    //    if (headingDeltaForRate < -180.0f) headingDeltaForRate += 360.0f;
    //    headingRate = (dtRate > 0.0f) ? (headingDeltaForRate / dtRate) : 0.0f;
    //}
    // FIX: Use smoothed GPS angle for rate calculation (avoid raw GPS noise)
    static float lastSmoothedForRate = 0.0f;
    static uint32_t lastRateCalcTime = 0;
    float dtRate = (lastRateCalcTime > 0) ? (now - lastRateCalcTime) / 1000.0f : 0.1f;
    if (dtRate > 0.001f && dtRate < 5.0f) {
        float hDelta = gpsAngleSmoothed - lastSmoothedForRate;
        hDelta = normalizeAngle(hDelta);
        headingRate = hDelta / dtRate;
        lastSmoothedForRate = gpsAngleSmoothed;
        lastRateCalcTime = now;
    }
    lastHeading = currentHeading;
    lastGPSTime = now;

    gpsAngle = 0.0f;  // Default: no correction this cycle
    bool haveHeadingDelta = false;
    float headingDelta = 0.0f;

    // --- direction of travel for the curvature SIGN (steering task fix, 2026-09-29) -----
    // Operator decision: "ja, den steuer task noch fixen". The lookback DISTANCE is
    // unsigned, but the same wheel angle turns the vehicle the OTHER way while it moves
    // backwards: omega = v*tan(delta)/L with v and the distance taken as magnitudes gives
    // curvature = +/- tan(delta)/L, and the sign is the sign of v - which no receiver
    // reports (VTG speed is always positive). The bridge's reversal verdict is the
    // available substitute: it compares the fused heading with the position derived
    // direction of travel (and since 2026-09-29 it is itself guarded by the position
    // witness and the attitude witness). Every heading-delta -> angle conversion below is
    // sign-corrected with it - otherwise the derived wheel angle has the WRONG SIGN in
    // reverse, and the zero-point observer (which uses zoStraightBaselineAngleDeg as its
    // model value, see line ~1509) would learn a corrupted zero.
    // While a direction change is in progress (AOG's byPass) no verdict exists, so the
    // affected cycle is skipped instead of being used with an unknown sign.
    const bool reversing = bst.reverseDirection;
    const bool dirUncertain = bst.dirChanging;
    float distance = 0.0f;

    if (gpsData.hasPosition && gpsData.fixQuality > 0) {
        // --- Ring-buffer lookback ---
        // Instead of waiting forward for minDistanceForCurvature of new
        // travel (which at ~0.04 m/s takes ~2.5s per update), search
        // backward through recent history for the OLDEST sample that is
        // still >= minDistanceForCurvature away from the current position.
        // This gives a fresh candidate on every GPS fix, at any speed,
        // while still requiring a real, adequately-sized baseline.
        constexpr double metersPerDegreeLat = 111320.0;
        int16_t foundIdx = -1;
        float foundDistance = 0.0f;
        // Second, LONGER baseline for the zero observer's straightness gate. The loop
        // must no longer stop at the 0.10 m sample, otherwise the 0.5 m sample is never
        // reached. It runs to the end of the history instead - this is called per GPS
        // fix (10 Hz), so 300 iterations is nothing.
        int16_t longIdx = -1;
        float longDistance = 0.0f;
        // Standstill window: the sample whose age is closest to STANDSTILL_WINDOW_S,
        // but at least STANDSTILL_MIN_FRACTION of it - a shorter baseline cannot
        // separate noise from creep. The iteration is oldest-first, so the age
        // DECREASES with i; that is why the closest match is tracked instead of the
        // first one found.
        int16_t winIdx = -1;
        float   winDistance = 0.0f;
        float   winAgeError = 1.0e9f;

        for (uint16_t i = 0; i < gpsHistoryCount; i++) {
            // Iterate oldest-first: index arithmetic on the circular buffer.
            uint16_t idx = (gpsHistoryHead + GPS_HISTORY_SIZE - gpsHistoryCount + i)
                           % GPS_HISTORY_SIZE;
            // FIX: Clamp latitude for cos() to avoid NaN near poles
            double latClamped = fmax(-89.9, fmin(89.9, gpsData.latitude * DEG_TO_RAD));
            double dNorth = (gpsData.latitude - gpsHistory[idx].lat) * metersPerDegreeLat;
            double dEast = (gpsData.longitude - gpsHistory[idx].lon) * metersPerDegreeLat * cos(latClamped);
            float d = (float)sqrt(dNorth * dNorth + dEast * dEast);
            if (foundIdx < 0 && d >= kMinDistanceForCurvature) {
                foundIdx = (int16_t)idx;
                foundDistance = d;
            }
            // Keep OVERWRITING: oldest-first iteration walks from the farthest sample
            // towards the newest, so the last one that still qualifies is the SHORTEST
            // baseline >= ZO_STRAIGHT_BASELINE_M. That is what is wanted - an
            // unbounded baseline would let the sample age run to the 3 s cap and
            // describe a manoeuvre from long ago, which is exactly the case the
            // operator warned about ("he will have steered in the meantime").
            if (d >= ZO_STRAIGHT_BASELINE_M) {
                longIdx = (int16_t)idx;
                longDistance = d;
            }
            // Standstill candidate: net displacement over the window. `d` is
            // already the straight-line distance from the CURRENT position, i.e.
            // exactly the net travel over that sample's age - no extra maths.
            const float ageS = (now - gpsHistory[idx].timeMs) / 1000.0f;
            if (ageS >= STANDSTILL_WINDOW_S * STANDSTILL_MIN_FRACTION) {
                const float ageErr = fabsf(ageS - STANDSTILL_WINDOW_S);
                if (ageErr < winAgeError) {
                    winAgeError = ageErr;
                    winIdx = (int16_t)idx;
                    winDistance = d;
                }
            }
        }

        // --- standstill window (see STANDSTILL_* in the header) ---
        // Only set when a long enough window exists; otherwise standstillValid
        // stays false and the steering gate falls back to speed alone.
        if (winIdx >= 0) {
            standstillNetM = winDistance;
            standstillValid = true;
        }

        // --- long-baseline straightness for the zero-point observer ---
        // The equivalent steering angle of the path bend over the baseline. NOTE: this IS
        // used as a VALUE by the zero observer (`const float bA = zoStraightBaselineAngleDeg`,
        // see the ZO state machine further down), not only as a magnitude gate - an earlier
        // comment here claimed the opposite and was wrong. The sign convention therefore
        // matters and is corrected for the direction of travel below.
        zoStraightBaselineValid = false;
        if (longIdx >= 0 && longDistance > 0.01f) {
            const float timeSpanL = (now - gpsHistory[longIdx].timeMs) / 1000.0f;
            const float impliedSpeedL = (timeSpanL > 0.001f) ? (longDistance / timeSpanL)
                                                             : 999.0f;
            // Same relative limit as the short baseline (PLAUS_SPEED_FACTOR). An
            // absolute 5 m/s here stopped the LONG baseline too - and with it
            // zoStraightBaselineValid, i.e. the straightness gate the observer needs -
            // above 18 km/h.
            if (timeSpanL <= ZO_BASELINE_MAX_AGE_S &&
                impliedSpeedL <= plausSpeedLimit() && !dirUncertain) {
                float dPsi = currentHeading - gpsHistory[longIdx].heading;
                if (dPsi > 180.0f) dPsi -= 360.0f;
                if (dPsi < -180.0f) dPsi += 360.0f;
                if (reversing) dPsi = -dPsi;      // see the note at the top (unsigned distance)
                const float curvature = (dPsi * (float)DEG_TO_RAD) / longDistance; // rad/m
                zoStraightBaselineAngleDeg =
                    atanf(curvature * config.wheelbase) * (float)RAD_TO_DEG;
                zoStraightBaselineDistM = longDistance;
                zoStraightBaselineValid = true;
            }
        }

        if (foundIdx >= 0) {
            distance = foundDistance;
            lastDistanceTraveled = distance;

            // Plausibility check: reject an implausibly large implied speed
            // (GPS jump / multipath glitch) instead of requiring a minimum speed.
            // The limit is RELATIVE with an absolute floor - see PLAUS_SPEED_FACTOR:
            // the old ABSOLUTE 5 m/s bound switched this whole GPS-angle path off
            // above 18 km/h, which is exactly the drive-to-the-field case the operator
            // wants the zero learned in (measured: gpsValid 1 -> 0 at 5 m/s, win frozen
            // at 12/200 for 38 s).
            float timeSpan = (now - gpsHistory[foundIdx].timeMs) / 1000.0f;
            float impliedSpeed = (timeSpan > 0.001f) ? (distance / timeSpan) : 999.0f;
            const float speedLimit = plausSpeedLimit();

            // Phase 4: reject a lookback sample that is simply too old.
            // While creeping the 10 cm lookback is ~2.5 s away, and while
            // stationary the search can walk arbitrarily far back, so the
            // heading delta would describe a manoeuvre from long ago. Note
            // the current sample is still pushed into the ring buffer below
            // (no early return), otherwise the buffer would develop a hole.
            if (timeSpan > config.maxGPSSampleAge) {
                // too old -> haveHeadingDelta stays false for this cycle
            }
            else if (impliedSpeed <= speedLimit) {
                headingDelta = currentHeading - gpsHistory[foundIdx].heading;
                if (headingDelta > 180.0f) headingDelta -= 360.0f;
                if (headingDelta < -180.0f) headingDelta += 360.0f;
                // Sign correction for the direction of travel (steering task fix
                // 2026-09-29, see the long note at the top of the block): while reversing,
                // the same wheel angle turns the heading the other way relative to the
                // travelled (unsigned) path.
                if (reversing) headingDelta = -headingDelta;
                // A direction change in progress means no verdict exists - skip the cycle
                // instead of converting a heading delta with an unknown sign.
                haveHeadingDelta = !dirUncertain;
            }
            else {
                // Glitch: skip this cycle entirely (haveHeadingDelta stays false)
                // rather than feeding a corrupted sample downstream.
                //
                // Reported, rate-limited: the implied/filtered RATIO is the only way
                // to judge whether PLAUS_SPEED_FACTOR is right, and a silently
                // rejected candidate is indistinguishable from "no straight driving"
                // in every other log line.
                static uint32_t lastGlitchLog = 0;
                if (now - lastGlitchLog > 5000) {
                    lastGlitchLog = now;
                    LOG_WARNING(EventSource::AUTOSTEER,
                                "VWAS: lookback rejected as implausible - implied %.1f m/s vs "
                                "limit %.1f m/s (filtered speed %.2f, ratio %.1f), %.2f m in "
                                "%.2f s. Firing continuously while driving normally means "
                                "PLAUS_SPEED_FACTOR is too tight.",
                                impliedSpeed, speedLimit, vehicleSpeed,
                                (vehicleSpeed > 0.01f) ? (impliedSpeed / vehicleSpeed) : 0.0f,
                                distance, timeSpan);
                }
            }
        }

        // Push the current sample into the ring buffer for future lookups.
        gpsHistory[gpsHistoryHead].lat = gpsData.latitude;
        gpsHistory[gpsHistoryHead].lon = gpsData.longitude;
        gpsHistory[gpsHistoryHead].heading = currentHeading;
        gpsHistory[gpsHistoryHead].timeMs = now;
        gpsHistoryHead = (gpsHistoryHead + 1) % GPS_HISTORY_SIZE;
        if (gpsHistoryCount < GPS_HISTORY_SIZE) gpsHistoryCount++;
    }

    // --- Vehicle-type-specific raw angle candidate ---
    bool haveCandidate = false;
    float rawAngleCandidate = 0.0f;

    if (config.vehicleType == 2) {
        // Articulated ("Knicklenker"): B10.52 (2026-10-25) - HIER STAND EIN PHYSIKFEHLER.
        // Frueher wurde die Aenderung des VORDERRAHMEN-Headings gegen eine Festreferenz als
        // Knickwinkel ausgegeben ("rear-frame yaw rate is comparatively small"). Das ist
        // falsch: bei mittigem Pivot gilt psi_F = psi_M + gamma/2, in einer GLEICHMAESSIGEN
        // Kurve ist gamma konstant, also omega_F = omega_R = omega_M - beide Rahmen drehen
        // gleich schnell. Die Heading-Aenderung seit der Referenz ist damit der KURSWECHSEL
        // der Fahrt und nicht gamma; nach ~90 Grad Kurvenfahrt stand der Wert am Anschlag
        // (maxSteeringAngle) und blieb dort, weil die Referenz nur bei reset() neu gesetzt
        // wird. Richtig ist die Umkehrung der Gelenk-Kinematik, und die braucht KEINEN
        // Gelenksensor - nur die Kruemmung, die hier ohnehin berechnet wird:
        //     omega_F = v/(L/2) * sin(gamma/2)   =>   sin(gamma/2) = kappa * (L/2)
        //     gamma   = 2*asin(kappa * L/2)
        // Das ist das exakte Gegenstueck zum Ackermann-Zweig unten (delta = atan(kappa*L)).
        // Kein Zirkelbezug: geschaetzt wird aus dem EMPFAENGER-Heading, nicht aus az_out.
        if (haveHeadingDelta) {
            const float halfWb = 0.5f * config.wheelbase;
            if ((halfWb > 0.2f) && (distance > 0.01f)) {
                const float curvature = (headingDelta * DEG_TO_RAD) / distance;   // 1/m
                float s = curvature * halfWb;                                    // sin(gamma/2)
                if (s > 1.0f) s = 1.0f;
                if (s < -1.0f) s = -1.0f;
                float gamma = 2.0f * asinf(s) * RAD_TO_DEG;
                // Rueckwaerts dreht derselbe Knick den Vorderrahmen andersherum (wie im
                // Ackermann-Zweig): Vorzeichen umdrehen.
                if (reversing) gamma = -gamma;
                if (fabsf(gamma) > config.maxSteeringAngle * 1.2f) {
                    gpsAngleValid = false;
                    return;
                }
                rawAngleCandidate = constrain(gamma, -config.maxSteeringAngle, config.maxSteeringAngle);
                haveCandidate = true;
            }
        }
    } else {
        // Standard (Ackermann front-wheel steering, vehicleType 0 and
        // default fallback for any other/unhandled type).
        // curvature = headingDelta / distanceTravelled = 1/turnRadius.
        // Safe to recompute freely from overlapping lookback windows since
        // this feeds an EMA filter below, not a running sum.
        if (haveHeadingDelta) {
            float headingDeltaRad = headingDelta * DEG_TO_RAD;
            float curvature = headingDeltaRad / distance;  // 1/m
            float angleRad = atan(curvature * config.wheelbase);
            rawAngleCandidate = angleRad * RAD_TO_DEG;

        //    if (rawAngleCandidate > config.maxSteeringAngle) {
        //        rawAngleCandidate = config.maxSteeringAngle;
        //    } else if (rawAngleCandidate < -config.maxSteeringAngle) {
        //        rawAngleCandidate = -config.maxSteeringAngle;
        //    }
        // FIX: Plausibility reject instead of hard clip (40Â° saturation bug)
            if (abs(rawAngleCandidate) > config.maxSteeringAngle * 1.2f) {
                LOG_DEBUG(EventSource::AUTOSTEER,
                          "GPS candidate rejected: |%.2fÂ°| > %.2fÂ° threshold",
                          rawAngleCandidate, config.maxSteeringAngle * 1.2f);
                gpsAngleValid = false;
                return;
            }
            rawAngleCandidate = constrain(rawAngleCandidate, -config.maxSteeringAngle, config.maxSteeringAngle);
            haveCandidate = true;
        }

        // Optional IMU yaw-rate path (unchanged, still time-rate based -
        // only used if explicitly enabled).
        if (config.useIMUHeadingRate && imuProcessor) {
            const auto& imuData = imuProcessor->getCurrentData();
            if (imuData.isValid) {
                headingRate = imuData.yawRate;
                rawAngleCandidate = calculateGPSAngleFromHeadingRate(headingRate, vehicleSpeed);
                haveCandidate = (vehicleSpeed >= config.minSpeedForGPS);
            }
        }
    }

    if (!haveCandidate) {
        gpsAngleValid = false;
        return;
    }

    // --- EMA smoothing ---
    // A single moving-average/low-pass stage on the heading-derived angle
    // (rather than a second Kalman layer) - simpler to reason about and
    // tune, and avoids a Kalman-on-Kalman noise interaction.
    float dtSmooth = 0.1f;  // approx cycle time; fusion runs at 100Hz driven externally
    float alpha = dtSmooth / (kGpsAngleSmoothingTau + dtSmooth);
    if (!gpsAngleSmoothedInit) {
        gpsAngleSmoothed = rawAngleCandidate;
        gpsAngleSmoothedInit = true;
    } else {
        gpsAngleSmoothed = gpsAngleSmoothed + alpha * (rawAngleCandidate - gpsAngleSmoothed);
    }
    gpsAngle = gpsAngleSmoothed;

    // Validity gate: the distance/plausibility gate above is now the real
    // protection - no additional minimum-speed requirement, so this works
    // correctly at your ~0.04 m/s work speed as well as at road-transport
    // speed.
    gpsAngleValid = haveCandidate;

    if (gpsAngleValid) {
        LOG_DEBUG(EventSource::AUTOSTEER,
                  "GPS angle: %.2fÂ° (raw %.2fÂ°, type=%d, source=%s q=%d, speed: %.2f m/s, dist: %.2fm)",
                  gpsAngle, rawAngleCandidate, config.vehicleType,
                  usingDualHeadingNow ? "dual" : "COG",
                  lastHeadingQualityUsed, vehicleSpeed, lastDistanceTraveled);
    }
}


void WheelAngleFusion::updateKalmanFilter(float dt) {
    // --- Encoder-dominant fusion ---
    // fusedAngle is ALWAYS the encoder-derived angle. GPS/dual-heading never
    // directly replaces or blends into the instantaneous angle - it only
    // ever nudges encoderOffset, slowly and rate-limited, in
    // updateDriftCompensation() below. This matches the proven approach
    // used by Cerea, AOG_Teensy_UM98X and 87yj/AgOpen_Keya_VirtualWAS:
    // "encoder dominates during rapid changes, heading only compensates
    // for drift" - rather than a symmetric Kalman blend that lets a single
    // noisy heading sample swing the reported angle by many degrees.
    predictedAngle = encoderAngle - encoderOffset;
    fusedAngle = predictedAngle;

    // uncertainty/kalmanGain are kept only as diagnostic/health indicators
    // now (see isHealthy()), not as active blend weights.
    uncertainty = min(uncertainty + config.processNoise * dt, config.initialUncertainty);
    kalmanGain = 0.0f;  // No longer used to blend fusedAngle; see updateDriftCompensation()

    applyFusedAngleClamp();

    if (uncertainty < 0.001f) {
        uncertainty = 0.001f;
    }
}

void WheelAngleFusion::applyFusedAngleClamp() {
    // Constrain final angle to reasonable limits - but ONLY when the limit is
    // usable. With maxSteeringAngle == 0 this clamp evaluates to
    // "fusedAngle > 0 -> 0" and "fusedAngle < 0 -> 0", i.e. it silently pins the
    // reported steering angle to zero whatever the encoder says. That failure is
    // invisible: the firmware looks healthy, the web UI shows a plausible cpd,
    // and AgOpenGPS just receives a constant 0 deg (which is exactly what a
    // mis-shaped VWAS config blob produced in the field).
    //
    // A visibly wrong angle is strictly safer than an invisibly wrong one, so the
    // clamp is skipped and a loud error is raised instead.
    if (config.maxSteeringAngle >= 1.0f) {
        if (fusedAngle > config.maxSteeringAngle) {
            fusedAngle = config.maxSteeringAngle;
        } else if (fusedAngle < -config.maxSteeringAngle) {
            fusedAngle = -config.maxSteeringAngle;
        }
    } else {
        // Rate-limited: this runs at 100 Hz (and from the setters that
        // recompute the angle immediately).
        static uint32_t lastLimitWarnTime = 0;
        if (millis() - lastLimitWarnTime > 5000) {
            lastLimitWarnTime = millis();
            LOG_ERROR(EventSource::AUTOSTEER,
                      "VWAS: maxSteeringAngle=%.2f unusable - angle clamp DISABLED, reporting "
                      "the raw fused angle (%.2f deg) so the fault stays visible. Set "
                      "maxSteeringAngle in the VWAS web page.",
                      config.maxSteeringAngle, fusedAngle);
        }
    }
}

void WheelAngleFusion::updateVariance() {
    // Using fixed variance - adaptive calculation not yet implemented
    measurementVariance = 1.0f;
}

void WheelAngleFusion::updateDriftCompensation(float dt) {
    // Slowly trim encoderOffset toward agreement with the smoothed
    // GPS/dual-heading-derived angle. This never touches fusedAngle
    // directly (that stays encoder-only, see updateKalmanFilter) - it only
    // adjusts the long-term zero-point, and only by a small, rate-limited
    // amount per correction event.
    if (!gpsAngleValid) {
        driftRate = 0.0f;
        return;
    }

    // Rate-limit using the ACTUAL elapsed time since the last correction,
    // not the fusion loop's per-cycle dt. gpsAngleValid only pulses true
    // once per distance-gated GPS sample - at your ~0.04 m/s work speed
    // that's roughly every 2.5s, far longer than the 10ms fusion cycle.
    // Using the fixed 10ms dt here would make maxDriftCorrectionRate
    // apply ~250x too small a step per real correction event.
    uint32_t now = millis();
    float elapsed = (lastDriftCorrectionTime > 0) ?
                     (now - lastDriftCorrectionTime) / 1000.0f : dt;
    lastDriftCorrectionTime = now;
    // Guard against an implausibly long gap (e.g. after being stationary
    // for a while) turning into one huge correction step.
    if (elapsed > 5.0f) elapsed = 5.0f;

    float currentAngle = encoderAngle - encoderOffset;

    // --- Straight-drive gate (Phase 4) ---
    // Only trim while the vehicle is genuinely going straight. The
    // heading-derived angle is a curvature angle and is only comparable to
    // the encoder angle on a straight: in a turn the two describe different
    // quantities, so any disagreement is manoeuvre, not drift. Without this
    // gate the 2026-09-08 log showed the trim walking ~2 deg during corners.
    if (abs(currentAngle) > config.straightDriveMaxAngle) {
        driftRate = 0.0f;
        return;
    }

    float innovation = gpsAngle - currentAngle;

    // Outlier rejection: a large disagreement most likely means a genuine
    // manoeuvre, bad GPS, or the encoder scale/sign being wrong - not slow
    // drift. Skip the correction entirely rather than partially applying
    // it (partial application of bad data was the original bug).
    if (abs(innovation) > kDriftInnovationRejectThreshold) {
        return;
    }

    // Proportional correction, hard-capped in degrees/second so GPS can
    // never yank the reported angle quickly regardless of tuning mistakes
    // elsewhere.
    float desiredCorrection = innovation;  // degrees we'd like to close
    float maxStep = kMaxDriftCorrectionRate * elapsed;
    float step = constrain(desiredCorrection * 0.1f, -maxStep, maxStep);

    // encoderOffset is defined such that fusedAngle = encoderAngle - encoderOffset.
    // To move currentAngle toward gpsAngle (i.e. increase currentAngle when
    // innovation > 0), we decrease encoderOffset by `step`.
    encoderOffset -= step;
    driftRate = step / elapsed;
    encoderDrift = encoderOffset;  // kept for external getters/telemetry
}

float WheelAngleFusion::calculateGPSAngleFromHeadingRate(float headingRate, float speed) {
    // Ackermann steering geometry
    // wheel_angle = atan(heading_rate * wheelbase / speed) * RAD_TO_DEG
    
    if (speed < config.minSpeedForGPS) {
        return 0.0f;
    }
    
    float headingRateRad = headingRate * DEG_TO_RAD;
    float angleRad = atan(headingRateRad * config.wheelbase / speed);
    float angleDeg = angleRad * RAD_TO_DEG;
    
    // Sanity check
    if (angleDeg > config.maxSteeringAngle) {
        angleDeg = config.maxSteeringAngle;
    } else if (angleDeg < -config.maxSteeringAngle) {
        angleDeg = -config.maxSteeringAngle;
    }
    
    return angleDeg;
}

bool WheelAngleFusion::isValidGPSConditions() const {
    // Check if conditions are suitable for GPS angle calculation
    return (vehicleSpeed >= config.minSpeedForGPS &&
            abs(headingRate) < config.maxHeadingRate &&
            abs(fusedAngle) < 30.0f);  // Relatively straight driving
}

bool WheelAngleFusion::isHealthy() const {
    // Check overall system health.
    //
    // IMPORTANT: this deliberately does NOT require GPS/heading-rate
    // validity or a minimum speed. GPS fusion only *improves* long-term
    // drift correction while driving in a straight-ish line; the
    // encoder-derived estimate (predictedAngle / fusedAngle in the
    // no-GPS branch of updateKalmanFilter) is still a meaningful angle
    // at standstill or at low speed. If this returned false whenever GPS
    // wasn't aiding, AutosteerProcessor falls back to the raw analog WAS
    // input - which, on a machine with NO physical WAS sensor wired up,
    // is a floating ADC pin producing noise. That fallback is strictly
    // worse than the encoder-only estimate, so we do not gate on it here.
    uint32_t now = millis();
    
    if (!keyaDriver) {
        return false;  // Never initialized / no position feedback
    }
    
    // Check if we're getting updates
    if (now - lastUpdateTime > 1000) {
        return false;  // No updates for 1 second
    }
    
    // Check if uncertainty is reasonable
    if (uncertainty > 50.0f) {
        return false;  // Too uncertain (e.g. long time without GPS aiding)
    }
    
    // Check if angle is reasonable
    if (abs(fusedAngle) > config.maxSteeringAngle * 1.5f) {
        return false;  // Angle out of bounds - something is very wrong
    }
    
    return true;
}

void WheelAngleFusion::setEncoderCenter() {
    if (!keyaDriver) {
        LOG_ERROR(EventSource::AUTOSTEER, "Cannot set encoder center - no motor driver");
        return;
    }
    
    // Capture the current raw count as the straight-ahead reference and
    // clear the long-term GPS drift offset. Call this with the wheels
    // physically straight. The raw accumulator is NOT zeroed - only the
    // reference moves - so the reported angle becomes 0 without any
    // rescaling or sign inversion (the old code set encoderOffset=encoderAngle
    // AND encoderAngle=0, which made predictedAngle = -encoderAngle).
    setCenterCounts(rawCounts);
    
    // Reset the delta tracking in the driver so the next getPositionDelta()
    // call doesn't report a large jump.
    keyaDriver->getPositionDelta();
    
    LOG_INFO(EventSource::AUTOSTEER, "Encoder center set (centerCounts=%d, rawCounts=%d)",
             centerCounts, rawCounts);
}

void WheelAngleFusion::setAogSteerScaling(uint8_t countsPerDegree, int16_t wasOffset,
                                         bool invertWas) {
    // Change-gated logging only: this is pushed from process() as a cheap
    // "keep in sync" call, so an unconditional log would flood the INFO bucket.
    // A slider drag in AOG's steer settings deliberately produces one line per
    // value - that is exactly the trace that was missing when the 2026-09-21
    // drive went off the line (aogOff=-134 / aogCpd=16 was nowhere in the log).
    const bool changed = (countsPerDegree != aogCountsPerDegree) ||
                         (wasOffset != aogWasOffset) ||
                         (invertWas != aogInvertWas);
    if (changed) {
        LOG_INFO(EventSource::AUTOSTEER,
                 "VWAS: AOG steer scaling -> counts/deg=%u wasOffset=%d invertWAS=%d "
                 "=> reported sign %s (encoder invert=%d, measured cpd=%.2f%s)",
                 (unsigned)countsPerDegree, (int)wasOffset, invertWas ? 1 : 0,
                 (config.invertEncoder != invertWas) ? "INVERTED" : "normal",
                 config.invertEncoder ? 1 : 0, config.countsPerDegree,
                 (countsPerDegree == 0) ? " - AOG counts unusable, measured value in use" : "");
    }
    aogCountsPerDegree = countsPerDegree;
    aogWasOffset = wasOffset;
    aogInvertWas = invertWas;

    // Only recompute on an actual change: this is called from the 100 Hz
    // process() loop, and rewriting fusedAngle every 10 ms would fight the
    // clamp/uncertainty bookkeeping in updateKalmanFilter() for no gain.
    if (!changed) {
        return;
    }

    // Recompute immediately. The calibration buttons read getEncoderAngle()
    // right after a capture, so waiting for the next updateEncoderAngle() cycle
    // would report the angle with the previous scale.
    float counts = (float)(rawCounts - centerCounts);
    if (effectiveInvertEncoder()) counts = -counts;
    counts += (float)aogWasOffset;
    encoderAngle = counts / getEffectiveCountsPerDegree();
    predictedAngle = encoderAngle - encoderOffset;
    fusedAngle = predictedAngle;
    applyFusedAngleClamp();
}

float WheelAngleFusion::getEffectiveCountsPerDegree() const {
    if (aogCountsPerDegree != 0) {
        return (float)aogCountsPerDegree;
    }

    // AOG has never supplied a usable value (blank EEPROM before the first
    // PGN 252). Dividing by zero would make the reported angle +/-inf and the
    // steering loop would rail into the mechanical stop, so fall back to the
    // last measured value and say so.
    static uint32_t lastWarnTime = 0;
    if (millis() - lastWarnTime > 5000) {
        lastWarnTime = millis();
        LOG_WARNING(EventSource::AUTOSTEER,
                    "VWAS: AOG countsPerDegree is 0 - falling back to the measured "
                    "%.2f counts/deg. Set 'Counts per degree' in AgOpenGPS.",
                    config.countsPerDegree);
    }
    return (config.countsPerDegree != 0.0f) ? config.countsPerDegree : 1.0f;
}

float WheelAngleFusion::effectiveSideHillGain() const {
    // Magnitude only, as entered in AgOpenGPS's convention (his slider is
    // positive-only, 0.10..0.15 on his machine).
    float mag = config.sideHillGainDegPerDeg;
    if (!isfinite(mag)) mag = 0.0f;
    if (mag < 0.0f) mag = -mag;

    static uint32_t lastWarnMs = 0;
    if (mag > 0.0f && (mag < ZO_GAIN_CFG_MIN || mag > ZO_GAIN_CFG_MAX)) {
        if (millis() - lastWarnMs > 10000) {
            lastWarnMs = millis();
            LOG_WARNING(EventSource::AUTOSTEER,
                        "VWAS: sidehill gain %.3f deg/deg is outside the plausible window "
                        "%.2f..%.2f - clamping. The value is AgOpenGPS's \"Side Hill Draft "
                        "Gain\" (positive, typically 0.12..0.15).",
                        mag, ZO_GAIN_CFG_MIN, ZO_GAIN_CFG_MAX);
        }
        mag = constrain(mag, ZO_GAIN_CFG_MIN, ZO_GAIN_CFG_MAX);
    }

    // SIGN. The residual model is `residual = reportedAngle - gain * roll`, and the
    // invert state flips the sign of reportedAngle - so the gain that makes the roll
    // term cancel has to flip with it. That is a necessity, not a tuning choice.
    //
    // NOTE the extra minus AgOpenGPS carries ITSELF:
    //     CGuidance.cs:98   steerAngleGu += imuRoll * -sideHillCompFactor
    // AOG's own slope in its convention is therefore -S, not +S. Combining the two:
    //     internalGain = -S * s,     s = +1 normal / -1 inverted
    // With the operator's calibration (invert=1, reportedAngle INVERTED relative to
    // AOG) that gives +S: the internal gain is POSITIVE while the slider is positive.
    //
    // The first version of this function omitted AOG's minus and produced -S. The
    // 2026-09-24 log caught it: the estimator reported +0.690 / +0.375 / +0.136
    // deg/deg (its own sign, independent of this value) while the derived gain was
    // negative, and the machine only worked with signFlip enabled to cancel the
    // error. With AOG's minus included the flip is no longer needed.
    float sign = effectiveInvertEncoder() ? 1.0f : -1.0f;
    if (config.sideHillGainSignFlip) sign = -sign;

    return sign * mag;
}

float WheelAngleFusion::plausSpeedLimit() const {
    // See the constant's comment in the header for the field measurement this comes
    // from. vehicleSpeed is the 80/20-EMA filtered GPS speed, which the HBUF data
    // shows tracks the positions (p50 of position/speed = 1.00), so scaling by it is
    // safe; maxPlausibleSpeed stays as the floor so low-speed behaviour is unchanged
    // (below 1.67 m/s the absolute 5 m/s bound is the larger of the two).
    float limit = PLAUS_SPEED_FACTOR * vehicleSpeed;
    if (limit < kMaxPlausibleSpeed) limit = kMaxPlausibleSpeed;
    return limit;
}

void WheelAngleFusion::setZeroTrimPermissions(bool guidanceActive, bool aogZeroActive) {
    const bool wasBlocked = (zoGuidanceActive || zoAogZeroActive);
    zoGuidanceActive = guidanceActive;
    zoAogZeroActive  = aogZeroActive;
    const bool isBlocked = (zoGuidanceActive || zoAogZeroActive);

    if (isBlocked == wasBlocked) {
        return;
    }

    // Logged because "why did the trim stop / start" must be answerable from the
    // log alone - the whole point of the ownership rules is that the operator can
    // see which regulator owns the zero point right now.
    if (isBlocked) {
        LOG_INFO(EventSource::AUTOSTEER,
                 "VWAS zero-trim: BLOCKED (%s) - observing only. A second integrator on "
                 "the same quantity as AgOpenGPS's PID integral cannot converge; the "
                 "measured residual stays visible in the diagnostics.",
                 zoGuidanceActive ? "guidance engaged" : "AgOpenGPS owns the zero");
    } else {
        LOG_INFO(EventSource::AUTOSTEER,
                 "VWAS zero-trim: released (manual driving, AOG offset inactive) - "
                 "residual=%+.2f deg gain=%.3f deg/deg trim=%+.2f deg state=%u",
                 zoResidualDeg, zoGainDegPerDeg, encoderOffset, (unsigned)zoState);
    }
}

void WheelAngleFusion::resetZeroObserverWindow() {
    for (uint8_t i = 0; i < ZO_BINS; i++) {
        zoBinSum[i] = 0.0f;
        zoBinCount[i] = 0;
    }
    zoSamples = 0;
    zoRollMin = 999.0f;
    zoRollMax = -999.0f;
    zoBASum = 0.0f;
    zoBACount = 0;
    zoWindowStartMs = 0;
}

bool WheelAngleFusion::fitZeroObserverWindow() {
    // Weighted least squares over the roll-bin means:
    //     reportedAngle = gain * roll + residual
    // Weighting by the sample count keeps a long straight at one roll value from
    // drowning out the bins that carry the slope information.
    float sc = 0.0f, scRoll = 0.0f, scAng = 0.0f, scRoll2 = 0.0f, scRollAng = 0.0f;
    uint8_t bins = 0;

    for (uint8_t i = 0; i < ZO_BINS; i++) {
        if (zoBinCount[i] == 0) continue;
        bins++;
        const float roll = ((float)i - (float)(ZO_BINS / 2)) * ZO_BIN_WIDTH_DEG;
        const float c = (float)zoBinCount[i];
        const float ang = zoBinSum[i] / c;
        sc        += c;
        scRoll    += c * roll;
        scAng     += c * ang;
        scRoll2   += c * roll * roll;
        scRollAng += c * roll * ang;
    }

    const float spread = (zoRollMax > zoRollMin) ? (zoRollMax - zoRollMin) : 0.0f;
    zoLastFitBins = bins;
    zoLastFitSpreadDeg = spread;

    if (bins < 1 || sc < 1.0f) {
        LOG_INFO(EventSource::AUTOSTEER,
                 "VWAS zero-trim: window empty (bins=%u samples=%u) - nothing changed.",
                 (unsigned)bins, (unsigned)zoSamples);
        return false;
    }

    const float rollMean = scRoll / sc;
    const float angMean  = scAng / sc;

    // --- the INTERCEPT is estimated with the gain HELD FIXED ----------------
    //
    // This is the fix for the 17-0 log. Letting both parameters float makes the
    // intercept absorb the gain's own noise through gain_error * mean_roll: with the
    // gain jumping between -0.845 and +1.431 in consecutive windows and a mean roll of
    // a few degrees, the intercept scattered by about +-4 deg. That scatter then sat
    // ABOVE the disagreement-stop threshold, so the trimmer kept tripping over its own
    // noise, re-arming for two windows each time, and the operator saw exactly what he
    // reported: straight driving changing almost nothing per unit time.
    //
    // The gain is a vehicle constant (AgOpenGPS's "Side Hill Comp"), so it does not
    // need re-estimating every 2 s. Holding it fixed removes the coupling completely
    // and, on top of that, makes the roll SPREAD unnecessary for accepting a window -
    // which matters on flat ground, where the spread never reaches the learning
    // threshold.
    // P2: the gain is the CONFIGURED value, not a learned one. Deriving the
    // intercept with a FIXED gain is what removed the +-4 deg intercept scatter of
    // the 17-0 log, and the config value is by construction stabler than anything a
    // single 2 s window can produce. See Config::sideHillGainDegPerDeg and
    // effectiveSideHillGain() for the sign derivation.
    const float gain = effectiveSideHillGain();
    zoGainDegPerDeg = gain;
    float residual = angMean - gain * rollMean;
    if (!isfinite(residual)) {
        return false;
    }

    // --- the gain is the MEDIAN of the recent candidates --------------------
    //
    // Every window with real roll variation produces a candidate slope, and a single
    // candidate scatters by about +-0.8 deg/deg: the ten candidates of the 17-24 log
    // ran 0.015 / 0.79 / 1.50 / 0.277 / -0.508 / -0.195 / 0.633 / -0.637 / 1.449 / 0.109
    // (mean 0.34) while the value implied by the drift itself was 0.68. One window is
    // therefore NOT a value.
    //
    // The previous implementation took the FIRST candidate as one, froze it, and then
    // rejected every later candidate more than ZO_GAIN_MAX_STEP away - so one window's
    // noise became a session-long constant, and all ten of those candidates above were
    // thrown away. The cost is visible in the same log: the gain sat at -1.311 where
    // this vehicle wants ~-0.15 (AgOpenGPS "Side Hill Comp" 0.12..0.15 deg per deg of
    // roll), so every residual carried a bias of
    //     (gain_used - gain_true) * roll = (1.311 - 0.15) * roll
    // i.e. up to +-6 deg of zero-point error walked in with the roll. That - not sensor
    // noise - was the drift under investigation.
    //
    // A median fixes both halves at once: one wild window cannot move the value at all,
    // and a wrong seed is completely replaced after ZO_GAIN_MEDIAN_N qualifying windows -
    // about 18 s of driving with 4 deg of roll variation, not minutes.
    bool gainUpdated = false;
    const float denom = sc * scRoll2 - scRoll * scRoll;
    if (spread >= ZO_GAIN_LEARN_SPREAD && bins >= ZO_MIN_BINS && fabsf(denom) > 1e-6f) {
        float cand = (sc * scRollAng - scRoll * scAng) / denom;
        if (isfinite(cand)) {
            if (cand < ZO_GAIN_MIN) cand = ZO_GAIN_MIN;
            if (cand > ZO_GAIN_MAX) cand = ZO_GAIN_MAX;

            zoGainCandDegPerDeg[zoGainCandHead] = cand;
            zoGainCandHead = (uint8_t)((zoGainCandHead + 1) % ZO_GAIN_MEDIAN_N);
            if (zoGainCandCount < ZO_GAIN_MEDIAN_N) zoGainCandCount++;

            // Median of the ring. Insertion sort - nine elements, once per 2 s window,
            // against a 600 MHz core: the simplicity is worth more than an algorithm.
            float sorted[ZO_GAIN_MEDIAN_N];
            for (uint8_t i = 0; i < zoGainCandCount; i++) sorted[i] = zoGainCandDegPerDeg[i];
            for (uint8_t i = 1; i < zoGainCandCount; i++) {
                const float v = sorted[i];
                int8_t j = (int8_t)i - 1;
                while (j >= 0 && sorted[j] > v) {
                    sorted[j + 1] = sorted[j];
                    j--;
                }
                sorted[j + 1] = v;
            }
            const uint8_t mid = (uint8_t)(zoGainCandCount / 2);
            zoGainReportedDegPerDeg = (zoGainCandCount & 1u)
                                          ? sorted[mid]
                                          : 0.5f * (sorted[mid - 1] + sorted[mid]);
            // REPORTED, not applied (P2). The number is printed in AgOpenGPS's
            // convention (negated) because that is what the operator can compare
            // against his slider; a median far from the configured value, or of the
            // wrong sign, is the fault this diagnostic exists to expose.
            static float lastReported = 0.0f;
            gainUpdated = (fabsf(zoGainReportedDegPerDeg - lastReported) > ZO_GAIN_LOG_STEP);
            if (gainUpdated) {
                lastReported = zoGainReportedDegPerDeg;
                LOG_INFO(EventSource::AUTOSTEER,
                         "VWAS zero-trim: gain REPORTED %.3f deg/deg (candidate %.3f, %u in "
                         "the median, roll spread %.1f deg) = AgOpenGPS convention %.2f; "
                         "configured %.2f. Diagnostic only - the residual uses the "
                         "configured value.",
                         zoGainReportedDegPerDeg, cand, (unsigned)zoGainCandCount, spread,
                         -zoGainReportedDegPerDeg, -gain);
            }
        }
    }

    // --- average the TRIM-INVARIANT error estimate --------------------------
    // errEst = measured residual + encoderOffset. The trim moves encoderOffset, so
    // errEst stays put while the correction runs - which is what makes a long average
    // of it a valid estimate rather than a value that chases its own correction. A
    // trim-based prediction would have to model the filter's lag and would flag every
    // window.
    const float errEstRaw = residual + encoderOffset;
    if (zoFits == 0) {
        zoErrorEstDeg = errEstRaw;
        zoFitDeviationDeg = 0.0f;
    } else {
        zoFitDeviationDeg = fabsf(errEstRaw - zoErrorEstDeg);
        zoErrorEstDeg += (errEstRaw - zoErrorEstDeg) / (float)ZO_RESIDUAL_AVG_FITS;
    }
    // What the trim still has to null.
    zoResidualDeg = zoErrorEstDeg - encoderOffset;

    zoLastFitMs = millis();   // start the age clock for the trim
    zoTrimSinceFitDeg = 0.0f; // a new residual grants a fresh correction allowance
    if (zoFits < 255) zoFits++;

    zoRollMeanDeg = rollMean;
    // bAmean is the diagnostic that proves P1 works: it is the mean of the
    // GPS-implied wheel angle that was subtracted sample by sample. Before P1 the
    // straight gate admitted |bA| <= 3 deg and the mean of those admitted samples
    // was NOT zero, so that mean leaked straight into the residual and looked like
    // a zero drift. After P1 the residual is b + (gain_true - gain_used) * roll
    // regardless of route, and bAmean only has to be small enough to show the
    // window was not a manoeuvre.
    const float bAMean = (zoBACount > 0) ? (zoBASum / (float)zoBACount) : 0.0f;
    LOG_INFO(EventSource::AUTOSTEER,
             "VWAS zero-trim: fit #%u raw=%+.2f errEst=%+.2f (dev %.2f) residual=%+.2f "
             "gain=%+.3f%s rep=%+.3f (bins=%u spread=%.2f rollMean=%+.2f bAmean=%+.2f "
             "samples=%u trim=%+.2f)",
             (unsigned)zoFits, residual, zoErrorEstDeg, zoFitDeviationDeg, zoResidualDeg,
             gain, gainUpdated ? " [rep moved]" : "",
             zoGainReportedDegPerDeg,
             (unsigned)bins, spread, rollMean, bAMean, (unsigned)zoSamples, encoderOffset);
    return true;
}

float WheelAngleFusion::zeroAcquireRate() const {
    // 1) SOLUTION TYPE. Unordered codes (see headingQualityMask). Only the ones that
    //    are not measurements cancel the correction outright. A single-point or DGPS
    //    solution is NOT zero: RTK is a position-accuracy advantage, not a
    //    precondition for measuring a heading well enough to trim a zero - which is
    //    the whole point of the operator's requirement that non-RTK installations
    //    work too.
    float sol;
    switch (zoQualityUsed) {
        case 4:
        case 5:  sol = ZO_SOL_FACTOR_RTK;    break;   // RTK fix / RTK float
        case 8:  sol = ZO_SOL_FACTOR_XWL;    break;   // extra wide lane
        case 9:  sol = ZO_SOL_FACTOR_SBAS;   break;   // SBAS
        case 2:  sol = ZO_SOL_FACTOR_DGPS;   break;   // DGPS
        case 1:  sol = ZO_SOL_FACTOR_SINGLE; break;   // single point
        default: return 0.0f;                          // 0/6/7 = not a measurement
    }

    // 2) RECEPTION - the pure signal-quality term and the one that moves the rate
    //    most. A dual-antenna baseline with many satellites gives a good heading even
    //    without carrier-phase RTK, and that is what has to decide how fast the trim
    //    may move.
    if (zoSatsUsed < ZO_MIN_SATELLITES) {
        return 0.0f;
    }
    float sat = ZO_SAT_FACTOR_MAX;
    if (zoSatsUsed < ZO_SAT_FULL_COUNT && ZO_SAT_FULL_COUNT > ZO_MIN_SATELLITES) {
        const float f = (float)(zoSatsUsed - ZO_MIN_SATELLITES) /
                        (float)(ZO_SAT_FULL_COUNT - ZO_MIN_SATELLITES);
        sat = ZO_SAT_FACTOR_MIN + f * (ZO_SAT_FACTOR_MAX - ZO_SAT_FACTOR_MIN);
    }

    // 3) SOURCE - course over ground needs the vehicle to be moving and is noisier
    //    than a dual-antenna compass reading, which is the single-receiver + IMU case.
    const float src = zoUsedDualHeading ? ZO_SRC_FACTOR_DUAL : ZO_SRC_FACTOR_COG;

    // 4) The roll-spread factor is GONE (removed 2026-09-24).
    //
    //    Its purpose was to keep the fast path honest while the GAIN was estimated
    //    from the same window: with a 2.5 deg spread the slope - and through it the
    //    intercept - carried a degree or two of uncertainty, so the rate was scaled
    //    by spread / ZO_FIT_SPREAD_REF_DEG (5 deg).
    //
    //    Since P2 the gain comes from the configuration, and the intercept
    //    `mean(reported - bA) - gain * mean(roll)` is valid at ANY spread, so the
    //    factor no longer protects anything - it only throttled the correction. The
    //    2026-09-24 log measures the cost: the trimmer armed at 0.71 deg/s instead
    //    of 5.0 (spread 0.71 deg -> factor 0.142, and 5.0 * 1.0 * 1.0 * 1.0 * 0.142 =
    //    0.71 exactly), so a 13.7 deg correction took 38 s instead of about 3 - and
    //    it got WORSE the smoother the ground was, which is backwards.
    //
    //    Removed rather than retuned: there is nothing left for it to protect. What
    //    still scales the rate is the measurement quality itself (solution type,
    //    satellite count, heading source), i.e. only factors that describe how well
    //    the heading was MEASURED.
    return ZO_ACQUIRE_RATE_MAX * sol * sat * src;
}

void WheelAngleFusion::updateZeroObserver() {
    const uint32_t now = millis();

    // --- sample --------------------------------------------------------------
    // The gate is "the PATH is straight", taken from the existing distance-based
    // curvature angle (gpsAngle = headingDelta/distance), which is speed
    // independent and needs no heading RATE. It is used as a GATE only, never as
    // the reference, so its own noise does not enter the result.
    //
    // This deliberately works whether or not guidance is engaged:
    //   * manual driving (e.g. the drive to the field) - the operator holds a line
    //     by hand, which is the most robust observation phase because no AOG
    //     regulator is involved at all. The operator compensates the slope by hand
    //     exactly like the controller would, so the relation
    //     reportedAngle = roll * gain + residual still holds;
    //   * under guidance the same condition is simply "AOG is holding the line".
    // Only the TRIM is restricted to the manual phase, not the measurement.
    //
    // NOTE the absence of an upper limit on |fusedAngle|. The 16-02 run proves why
    // it must not be there: the boot zero was 28.7 deg off and a 20 deg gate
    // rejected every single sample, i.e. it blocked precisely the error it was
    // supposed to measure. The reported angle's magnitude IS the error under
    // investigation here, so gating on it is circular. The straight-path gate and
    // the consistency guards are what make a sample trustworthy.
    //
    // Cache the quality signals for this cycle (see the member comment).
    if (gnssProcessor) {
        const auto& q = gnssProcessor->getData();
        zoSatsUsed = q.numSatellites;
        // The dual-antenna heading is described by the HPR quality field; with a
        // single receiver there is no HPR at all, so the GGA fix quality is what says
        // whether the position - and therefore the COG track - is worth anything.
        zoQualityUsed = usingDualHeadingNow ? q.headingQuality : q.fixQuality;
        zoUsedDualHeading = usingDualHeadingNow;
    }

    // Only solutions that are actually measured may contribute. 0 = invalid,
    // 6 = dead reckoning, 7 = manual/operator-set: none of those is a measurement.
    const bool qfUsable = (zoQualityUsed != 0 && zoQualityUsed != 6 && zoQualityUsed != 7);

    // Source-aware minimum speed: COG (single receiver + IMU) only means anything
    // once the vehicle is moving, the dual-antenna heading does not.
    const float minSpeed = zoUsedDualHeading ? ZO_MIN_SPEED_MPS : ZO_MIN_SPEED_COG_MPS;

    // The straightness test is the LONG baseline (0.5 m of real travel), not the
    // 0.10 m curvature angle - see the note on ZO_STRAIGHT_BASELINE_M. gpsAngleValid
    // is kept only as a "the GPS pipeline produced a candidate this cycle" sanity
    // check; its VALUE is not used any more.
    //
    // ROLE OF THE GATE SINCE P1: bA is now SUBTRACTED per sample, so the gate no
    // longer has to protect the estimate from a BIAS - it only has to keep genuine
    // manoeuvres out, because bA describes the last 0.5 m and therefore lags the
    // current wheel angle. The 3 deg limit is deliberately left unchanged for this
    // build so the next log stays comparable with the previous ones; it can be
    // widened once bAmean confirms the subtraction behaves.
    // --- the samples must come from a MEASURED heading ------------------------
    // The circular path the operator asked about (2026-09-27) is real:
    // HeadingBridge's prediction is psi += v*tan(delta)/L * dt with delta = the wheel
    // angle WE report, and its steerBiasDeg state absorbs the very wheel-angle bias
    // this observer estimates. bA is a heading DIFFERENCE, so while the bridge
    // synthesises the heading the observer would be measuring a remainder of its own
    // output. During LIVE the heading is the measured attitude, so bA is a genuine
    // measurement of the path.
    //
    // Until now that exclusion only happened BY ACCIDENT: the dead-reckoning stretches
    // in the 2026-09-27 logs coincide with sat = 0, which the satellite gate below
    // rejects anyway. Measured over the same logs: bias_deg stayed inside +-0.5 deg
    // and did NOT track the applied trim (-0.85 deg) - so the bridge is NOT currently
    // absorbing the zero error, but nothing PREVENTS it. Hence an explicit gate.
    //
    // The bridge supplies the heading under exactly the same conditions as the
    // selection in updateGPSAngle(), so the test mirrors them instead of inventing a
    // second rule.
    const HeadingBridge::Config& zbCfg = headingBridge.getConfig();
    const HeadingBridge::Status& zbSt  = headingBridge.getStatus();
    const bool bridgeSuppliesHeading =
        zbCfg.enabled &&
        zbSt.state != HeadingBridge::State::INIT &&
        zbSt.state != HeadingBridge::State::TIMEOUT;
    // ... AND the heading must be a MEASUREMENT, not a model prediction. LIVE alone is
    // not enough since 2026-09-27: the bridge's own model gate raises its measurement
    // noise while it holds, so psi leans on the kinematic model for up to ~600 ms even
    // though the state stays LIVE. During those frames the observer would measure a
    // remainder of the model it feeds itself - the same circular path, one level deeper.
    const bool headingIsMeasured = (!bridgeSuppliesHeading ||
                                    zbSt.state == HeadingBridge::State::LIVE) &&
                                   !zbSt.modelGateActive;

    if (rollValid && gpsAngleValid && zoStraightBaselineValid && qfUsable &&
        headingIsMeasured &&
        zoSatsUsed >= ZO_MIN_SATELLITES &&
        fabsf(zoStraightBaselineAngleDeg) <= ZO_STRAIGHT_GATE_LONG &&
        vehicleSpeed >= minSpeed &&
        fabsf(rollDeg) <= ZO_MAX_ROLL_DEG) {

        const int32_t idx = lroundf(rollDeg / ZO_BIN_WIDTH_DEG) + (int32_t)(ZO_BINS / 2);
        if (idx >= 0 && idx < (int32_t)ZO_BINS) {
            // --- P1: subtract the GPS-implied wheel angle PER SAMPLE -------------
            // zoStraightBaselineAngleDeg = atan(curvature * wheelbase) is an
            // independent measurement of the wheel angle that the measured path
            // curvature implies, so `reported - bA` isolates the sideslip term plus
            // the zero error:
            //     reported - bA = gain_true * roll + b
            // That identity holds for ANY path, not only a straight one - which is
            // the point. Before, the gate admitted |bA| <= 3 deg and then assumed the
            // admitted mean was 0; it is not, because up to 3 deg of genuine steering
            // passes. That mean landed straight in the residual, and a route with
            // more curve one way than the other produced a slow apparent drift that
            // no amount of averaging could remove - it was a sampling bias, not
            // mechanics.
            const float bA = zoStraightBaselineAngleDeg;
            zoBinSum[idx] += (fusedAngle - bA);
            if (zoBinCount[idx] < 60000) zoBinCount[idx]++;
            if (rollDeg < zoRollMin) zoRollMin = rollDeg;
            if (rollDeg > zoRollMax) zoRollMax = rollDeg;
            // Start the fill-rate clock on the first accepted sample of the window
            // (nacc in the status line).
            if (zoSamples == 0) zoWindowStartMs = now;
            if (zoSamples < 60000) zoSamples++;
            zoBASum += bA;
            if (zoBACount < 60000) zoBACount++;
        }
    }

    // --- fit when the window is full -----------------------------------------
    if (zoSamples >= ZO_WINDOW_SAMPLES) {
        if (fitZeroObserverWindow()) {
            if (fabsf(zoResidualDeg) <= ZO_LOCK_TOLERANCE) {
                if (zoStableFits < 255) zoStableFits++;
                zoReopenFits = 0;
            } else if (fabsf(zoResidualDeg) >= ZO_REOPEN_DEG) {
                if (zoReopenFits < 255) zoReopenFits++;
                zoStableFits = 0;
            } else {
                zoStableFits = 0;
                zoReopenFits = 0;
            }

            // --- breakdown detection -----------------------------------------
            // Breakdown detector: does this window still describe the picture the
            // FILTER has built? The comparison is raw-vs-filtered, not raw-vs-prediction:
            // a trim-based prediction would have to model the filter's lag and would
            // flag every window once the trim is running.
            const float dRes = zoFitDeviationDeg;

            if (zoFits <= 1) {
                zoArmed = false;
            } else if (!zoArmed) {
                // The residual FILTER replaced the old "two consecutive fits must agree"
                // rule. One bad window now moves the average by 1/8 of its deviation
                // instead of being acted upon, so there is nothing left for a separate
                // arming check to protect against - while the check itself was tripping
                // on normal scatter ("not armed ... disagree 5.1" in the 17-0 log) and
                // costing two windows of re-arming every time.
                zoArmed = true;
                LOG_INFO(EventSource::AUTOSTEER,
                         "VWAS zero-trim: ARMED - errEst %+.2f deg (average of up to %u "
                         "windows), trim=%+.2f. Correcting at %.2f deg/s.",
                         zoErrorEstDeg, (unsigned)ZO_RESIDUAL_AVG_FITS, encoderOffset,
                         zeroAcquireRate());
            } else if (dRes > ZO_DISAGREE_STOP_DEG) {
                // Require ZO_STOP_CONSECUTIVE such fits before actually stopping: one
                // deviating window is normal scatter, and stopping costs two windows of
                // re-arming. A genuine breakdown - a leaked straight gate, a bad roll, a
                // moved wheel - deviates by the size of the error it introduces and
                // repeats.
                if (zoStopFits < 255) zoStopFits++;
                if (zoStopFits >= ZO_STOP_CONSECUTIVE) {
                    zoStopFits = 0;
                    zoArmed = false;
                    LOG_WARNING(EventSource::AUTOSTEER,
                                "VWAS zero-trim: STOPPED - %u consecutive windows deviated more "
                                "than %.1f deg from the filtered estimate (last raw %+.2f vs "
                                "filtered %+.2f, %.1f off), so the measurement no longer "
                                "describes the angle. Trim paused at %+.2f deg, re-arming on the "
                                "filtered estimate. Heading q=%u, roll spread %.2f deg.",
                                (unsigned)ZO_STOP_CONSECUTIVE, ZO_DISAGREE_STOP_DEG,
                                zoErrorEstDeg + dRes, zoErrorEstDeg, dRes, encoderOffset,
                                (unsigned)lastHeadingQualityUsed, zoLastFitSpreadDeg);
                } else {
                    LOG_INFO(EventSource::AUTOSTEER,
                             "VWAS zero-trim: deviation %.1f deg (> %.1f) - first of %u, "
                             "waiting for confirmation before stopping.",
                             dRes, ZO_DISAGREE_STOP_DEG, (unsigned)ZO_STOP_CONSECUTIVE);
                }
            } else {
                zoStopFits = 0;
            }

            const bool blocked = (zoGuidanceActive || zoAogZeroActive);
            if (blocked) {
                zoState = ZO_BLOCKED;
            } else {
                // Released (or never blocked): acquire again, because the mechanical
                // zero may have moved while guidance was engaged.
                if (zoState == ZO_BLOCKED || zoState == ZO_IDLE) {
                    zoState = ZO_ACQUIRE;
                }
                if (zoState == ZO_ACQUIRE && zoStableFits >= ZO_LOCK_FITS) {
                    zoState = ZO_HOLD;
                    // Acquisition is done. From here the trim keeps running, but only at
                    // the slow hold RATE - there is no budget left to spend.
                    LOG_INFO(EventSource::AUTOSTEER,
                             "VWAS zero-trim: LOCKED after %u stable fits - residual %+.2f deg "
                             "(trim %+.2f deg). Correction CONTINUES at %.2f deg/min with no "
                             "budget: the field logs show a real mechanical drift of 0.5..1.4 "
                             "deg/min, which a consumed budget could not serve for a working "
                             "day. The only remaining limits are |trim| <= %.0f deg and the "
                             "block while AgOpenGPS guidance is engaged.",
                             (unsigned)zoFits, zoResidualDeg, encoderOffset,
                             ZO_HOLD_RATE_DEG_PER_S * 60.0f, ZO_TRIM_ABS_MAX_DEG);
                }
                // Verified drift reopens a locked trimmer. A single noisy fit must
                // not: that is exactly how a second integrator starts hunting.
                if (zoState == ZO_HOLD && zoReopenFits >= ZO_REOPEN_FITS) {
                    zoState = ZO_ACQUIRE;
                    zoStableFits = 0;
                    LOG_WARNING(EventSource::AUTOSTEER,
                                "VWAS zero-trim: RE-OPENED - residual %+.2f deg exceeded %.1f deg "
                                "in %u consecutive fits, so the zero really moved (mechanical "
                                "shift or an encoder counter reset). Correcting again.",
                                zoResidualDeg, ZO_REOPEN_DEG, (unsigned)zoReopenFits);
                }
            }
        }
        resetZeroObserverWindow();
    }

    // --- trim (continuous, rate limited) -------------------------------------
    // zoArmed instead of zoFits > 0: two consistent fits are required before the
    // angle may be moved at all (see the arming block above).
    // A residual is only valid until the next window is due. If the straight gate
    // closes and no new fit arrives, the old value must not keep driving the trim: the
    // 16-41 log shows exactly that failure - a stale residual applied at 3.2 deg/s for
    // far longer than a window drove the trim from -21 to -42 deg (14 deg PAST the
    // target) and burned 60 deg of the budget, after which nothing was left to close
    // the remaining ~5 deg. The disagreement guard CANNOT catch this: the overshoot is
    // precisely what the model predicts, so it is a control problem, not a measurement
    // problem.
    const float residualAgeS = (zoLastFitMs == 0) ? 1.0e6f : (now - zoLastFitMs) / 1000.0f;

    if ((zoState == ZO_ACQUIRE || zoState == ZO_HOLD) && zoArmed &&
        config.zeroTrimEnable && residualAgeS <= ZO_RESIDUAL_MAX_AGE_S) {

        float dt = (zoLastTrimMs == 0) ? 0.0f : (now - zoLastTrimMs) / 1000.0f;
        if (dt > 0.5f) dt = 0.5f;          // guard after a long stall
        zoLastTrimMs = now;

        // --- per-residual grant --------------------------------------------------
        // A measured residual authorises AT MOST |residual| of correction. That is
        // the fix for the 16-57 log: the trim there raced to +39.14 deg against a
        // target of +28.9 because it kept applying a residual measured 14 s earlier
        // (the straight gate was closed during the operator's fast right-left
        // manoeuvres, so no new fit arrived), overshot by 9 deg, had to come back and
        // burned the travel for a 29 deg correction. With the grant, travel can never
        // exceed the correction, so overshoot is impossible by construction.
        const float grantRemaining = fabsf(zoResidualDeg) - zoTrimSinceFitDeg;

        // --- safety limit: POSITION, not consumption (P3) ------------------------
        // The old per-session budgets are gone. What is bounded now is how far the
        // zero may sit from the mechanical reference AT ANY MOMENT - and that bound
        // is available again on the next correction instead of having been spent.
        // The clamp is on |encoderOffset|, the POSITION: clamping the travelled
        // distance (zoTrimAppliedDeg) instead would stop a correction that merely
        // oscillated once even with a perfect zero. 60 deg covers the full steering
        // range, so a boot with the wheels at full lock (28.7 deg in the 16-02 log)
        // is still correctable, while a runaway measurement is still stopped.
        const float headroom = ZO_TRIM_ABS_MAX_DEG - fabsf(encoderOffset);
        float allowed = (grantRemaining < headroom) ? grantRemaining : headroom;
        if (allowed < 0.0f) allowed = 0.0f;

        if (dt > 0.0f && allowed > 0.0005f) {
            // Two SPEEDS, not two budgets: acquisition chases a boot error in seconds
            // (quality- and spread-scaled), holding only tracks the slow mechanical
            // drift at the fixed ZO_HOLD_RATE_DEG_PER_S. Deliberately NOT
            // acqRate * damping any more: that tied the hold rate to the heading
            // quality, so a weak-but-usable heading switched the drift correction off
            // exactly when the measurement was already the limiting factor.
            const float acqRate = zeroAcquireRate();
            const float rate = (zoState == ZO_HOLD) ? ZO_HOLD_RATE_DEG_PER_S : acqRate;

            // Cap the step so that at most the residual ITSELF is spent per elapsed
            // time since that residual was measured. With a nominal 2 s window this is
            // the same "at most 100 % per window" limit as before, but when a fit is
            // late the cap shrinks with the growing age instead of continuing at full
            // rate - the correction then converges exactly to the target and stops.
            float capPerSec = fabsf(zoResidualDeg) / fmaxf(residualAgeS, ZO_WINDOW_SECONDS);
            if (capPerSec > rate) capPerSec = rate;

            // encoderOffset is defined such that fusedAngle = encoderAngle - encoderOffset,
            // so a POSITIVE residual (the reported angle reads too large) is nulled by
            // RAISING the offset.
            float step = constrain(zoResidualDeg * ZO_TRIM_GAIN,
                                   -capPerSec * dt, capPerSec * dt);

            if (step >  allowed) step =  allowed;
            if (step < -allowed) step = -allowed;

            if (fabsf(step) > 0.0005f) {
                encoderOffset += step;
                encoderDrift = encoderOffset;   // kept for the external getters
                zoTrimAppliedDeg += fabsf(step);
                zoTrimSinceFitDeg += fabsf(step);
                zoTrims++;
            }
        }
    } else {
        zoLastTrimMs = now;
    }
}

WheelAngleFusion::ZeroObserverStatus WheelAngleFusion::getZeroObserverStatus() const {
    ZeroObserverStatus s;
    s.residualDeg   = zoResidualDeg;
    s.gainDegPerDeg = zoGainDegPerDeg;
    s.trimDeg       = encoderOffset;
    s.rollSpreadDeg = zoLastFitSpreadDeg;
    s.rollMeanDeg   = zoRollMeanDeg;
    s.fitSamples    = zoSamples;
    s.fitBins       = zoLastFitBins;
    s.fits          = zoFits;
    s.state         = zoState;
    s.trims         = zoTrims;
    s.acquireRateDegPerSec = zeroAcquireRate();
    s.angleQuality  = computeAngleQuality();
    s.zeroLearned   = isZeroLearned();
    return s;
}

// ---------------------------------------------------------------------------
// Wheel angle reliability as a MODEL input (0...100), operator request 2026-09-28.
// ---------------------------------------------------------------------------
// The HeadingBridge integrates v*tan(delta)/L. With a wrong zero that integration is a
// false yaw rate - measured 2026-09-28: a Keya zero that is set at power-up to wherever
// the wheels happen to stand gave -31 deg "steering" while the machine drove straight,
// i.e. 50 deg/s of invented rotation, and with the bridge's model gate damping the
// measurements the heading then rotated although both receivers were RTK fixed.
//
// So the bridge needs ONE number that says whether the angle may be used at all. It is
// derived from the zero observer only (fits, fit stability, armed, state) and it has NO
// time decay on purpose: once the zero is learned it is a machine constant, and a
// drive without straight sections must not make the model look unusable.
float WheelAngleFusion::computeAngleQuality() const {
    if (zoFits == 0) return 0.0f;                 // never learned -> unusable
    float q = 100.0f;
    float f = (float)zoFits / (float)ZO_Q_FITS_FULL;
    if (f > 1.0f) f = 1.0f;
    q *= f;
    float d = 1.0f - zoFitDeviationDeg / ZO_Q_DEV_BAD_DEG;
    if (d < 0.0f) d = 0.0f;
    q *= d;
    if (!zoArmed) q *= 0.5f;                      // the fits do not agree yet
    if (zoState != ZO_HOLD) q *= 0.7f;            // still acquiring / not stable
    if (zoState == ZO_BLOCKED) q *= 0.8f;         // observe-only phase
    if (q < 0.0f) q = 0.0f;
    if (q > 100.0f) q = 100.0f;
    return q;
}

void WheelAngleFusion::adoptBootZero() {


    // See the declaration in the header for the field evidence. The short
    // version: a straight-ahead reference is a *count value*, and the counter it
    // refers to (WheelAngleFusion::rawCounts, an accumulator fed by
    // KeyaSerialDriver::getPositionDelta()) starts at 0 on every Teensy boot and
    // the Keya's own counter restarts with the motor. So the stored value
    // describes an origin that no longer exists.
    const int32_t storedReference = centerCounts;

    // rawCounts is already 0 at this point - assigning it makes the policy
    // explicit instead of relying on the constructor, and it is what the
    // operator's "park straight" rule refers to.
    centerCounts = 0;
    encoderOffset = 0.0f;
    encoderDrift = 0.0f;
    driftRate = 0.0f;
    lastDriftCorrectionTime = 0;

    // The observer starts from scratch as well: its trim belonged to the previous
    // power cycle. The GAIN, by contrast, is a vehicle constant that now comes from
    // the CONFIG (P2) - it is neither learned nor persisted by a fit any more.
    //
    // Why the estimator may not decide it: 31 candidates collected over the
    // 2026-09-21 logs scattered with sd 0.53 around a true value of ~0.125, so even
    // a median of 9 moves by about +-0.18 - larger than the value being estimated.
    // It now only REPORTS (see fitZeroObserverWindow). The config is in AgOpenGPS's
    // convention (positive, 0.10..0.15 on this machine) and the SIGN is derived from
    // the invert state, so it is printed here - a wrong derivation has to be visible
    // from the boot log instead of being diagnosed on the field.
    zoGainDegPerDeg = effectiveSideHillGain();
    LOG_INFO(EventSource::AUTOSTEER,
             "VWAS sidehill gain: config %.3f (AgOpenGPS \"Side Hill Draft Gain\", positive)%s "
             "-> internal %+.3f deg/deg (reportedAngle is %s relative to AgOpenGPS). The "
             "estimator runs as a diagnostic and does not change this value.",
             fabsf(config.sideHillGainDegPerDeg),
             config.sideHillGainSignFlip ? ", signFlip ON" : "",
             zoGainDegPerDeg,
             effectiveInvertEncoder() ? "INVERTED" : "normal");
    if (config.sideHillGainSignFlip) {
        // The flag is a FIELD escape hatch only. Since the derivation was corrected
        // (2026-09-24: AgOpenGPS's own minus in its sidehill term is now included),
        // leaving it set flips the sign a SECOND time and re-creates exactly the fault
        // it was added to cure. Warned at every boot, because a stale flag in the EEPROM
        // is otherwise invisible - and the boot line would look self-consistent.
        LOG_WARNING(EventSource::AUTOSTEER,
                    "VWAS: sidehill gain signFlip is ON. It is normally OFF - the derived "
                    "sign already contains AgOpenGPS's own minus (steerAngleGu += imuRoll * "
                    "-sideHillCompFactor), so the flip inverts the roll term a second time. "
                    "Internal gain is now %+.3f deg/deg. Untick it unless the estimator's "
                    "REPORTED value clearly says otherwise.",
                    zoGainDegPerDeg);
    }
    if (fabsf(config.sideHillGainDegPerDeg) == 0.0f) {
        LOG_WARNING(EventSource::AUTOSTEER,
                    "VWAS: sidehill gain is 0 - the roll term is not compensated at all. On a "
                    "sidehill the residual then carries gain_true * roll (about %.1f deg at "
                    "%.0f deg of roll) and that lands in the zero estimate. Set it to the "
                    "AgOpenGPS value (0.12..0.15).",
                    0.15f * ZO_MAX_ROLL_DEG, ZO_MAX_ROLL_DEG);
    }
    // The candidate ring and the reported median belong to the session, not to the
    // vehicle, so they always start empty.
    zoGainCandHead = 0;
    zoGainCandCount = 0;
    zoGainReportedDegPerDeg = 0.0f;
    for (uint8_t i = 0; i < ZO_GAIN_MEDIAN_N; i++) zoGainCandDegPerDeg[i] = 0.0f;
    zoRollMeanDeg = 0.0f;
    zoTrimAppliedDeg = 0.0f;
    zoPrevResidualDeg = 0.0f;
    zoOffsetAtPrevFit = 0.0f;
    zoArmed = false;
    zoTrimSinceFitDeg = 0.0f;
    zoFits = 0;
    zoStableFits = 0;
    zoReopenFits = 0;
    zoResidualDeg = 0.0f;
    zoState = ZO_IDLE;
    resetZeroObserverWindow();

    float counts = (float)(rawCounts - centerCounts);
    if (effectiveInvertEncoder()) counts = -counts;
    counts += (float)aogWasOffset;
    encoderAngle = counts / getEffectiveCountsPerDegree();
    predictedAngle = encoderAngle - encoderOffset;
    fusedAngle = predictedAngle;
    applyFusedAngleClamp();

    LOG_WARNING(EventSource::AUTOSTEER,
                "VWAS BOOT-ZERO: power-up position taken as straight ahead "
                "(centerCounts=0). The stored reference (%ld counts) is deliberately "
                "NOT applied - the Keya position counter restarts with the motor, so no "
                "stored count value can be valid after a power cycle. Park straight, or "
                "correct with the GND button / web button / an AgOpenGPS zero click.",
                (long)storedReference);
}

void WheelAngleFusion::setCenterCounts(int32_t c) {
    centerCounts = c;
    encoderOffset = 0.0f;   // clear GPS drift trim; new zero reference
    encoderDrift = 0.0f;
    driftRate = 0.0f;
    lastDriftCorrectionTime = 0;

    // A new mechanical zero invalidates the observer's correction too: its trim
    // moved encoderOffset toward a residual that was measured against the OLD
    // reference, and the per-session budget must not stay partly spent on a
    // correction that no longer applies. The learned sidehill gain is a vehicle
    // constant and is deliberately kept.
    zoTrimAppliedDeg = 0.0f;
    zoPrevResidualDeg = 0.0f;
    zoOffsetAtPrevFit = 0.0f;
    zoArmed = false;
    zoTrimSinceFitDeg = 0.0f;
    zoFits = 0;
    zoStableFits = 0;
    zoReopenFits = 0;
    zoResidualDeg = 0.0f;
    zoState = ZO_IDLE;
    resetZeroObserverWindow();
    // Recompute the reported angle immediately so callers see 0 without
    // waiting for the next update() cycle.
    //
    // Note the AOG offset term: with the wheels physically straight the reported
    // angle is wasOffset/countsPerDegree, NOT 0, whenever AgOpenGPS is holding a
    // non-zero offset. That is not a bug - it is exactly how a real analog WAS
    // behaves (at the ADC centre the angle is also offset/cpd), and the operator
    // clears it in AOG (Zero-WAS / set the WAS Offset slider to 0). It is logged
    // below because "I pressed 'set straight' and it still shows 3 deg" is
    // otherwise indistinguishable from a broken capture.
    float counts = (float)(rawCounts - centerCounts);
    if (effectiveInvertEncoder()) counts = -counts;
    counts += (float)aogWasOffset;
    encoderAngle = counts / getEffectiveCountsPerDegree();
    predictedAngle = encoderAngle - encoderOffset;
    fusedAngle = predictedAngle;
    applyFusedAngleClamp();

    if (aogWasOffset != 0) {
        LOG_WARNING(EventSource::AUTOSTEER,
                    "VWAS: centre saved, but AgOpenGPS holds wasOffset=%d counts "
                    "(= %.2f deg at %u counts/deg), so the reported angle is %.2f deg "
                    "with the wheels straight. Clear the WAS offset in AOG.",
                    (int)aogWasOffset, (float)aogWasOffset / getEffectiveCountsPerDegree(),
                    (unsigned)aogCountsPerDegree, encoderAngle);
    }
}



int16_t WheelAngleFusion::getScaledAngle() const {
    // Display-only mapping of the real angle onto the -127..+128 scale used
    // by the web UI. int16_t because +128 does not fit in a signed byte.
    if (config.maxSteeringAngle < 0.01f) return 0;
    float scaled = (fusedAngle / config.maxSteeringAngle) * 128.0f;
    if (scaled > 128.0f) scaled = 128.0f;
    if (scaled < -127.0f) scaled = -127.0f;
    return (int16_t)lroundf(scaled);
}

void WheelAngleFusion::reset() {
    LOG_INFO(EventSource::AUTOSTEER, "Resetting wheel angle fusion");
    
    // Reset Kalman filter
    fusedAngle = 0.0f;
    predictedAngle = 0.0f;
    uncertainty = config.initialUncertainty;
    kalmanGain = 0.0f;
    
    // Reset sensor angles
    encoderAngle = 0.0f;
    gpsAngle = 0.0f;
    gpsAngleValid = false;
    rawCounts = 0;
    centerCounts = 0;
    
    // Reset drift compensation
    encoderDrift = 0.0f;
    driftRate = 0.0f;
    driftStartTime = millis();

    // Reset distance-based curvature / integration / smoothing state
    gpsHistoryHead = 0;
    gpsHistoryCount = 0;
    lastDistanceTraveled = 0.0f;
    gpsAngleSmoothed = 0.0f;
    gpsAngleSmoothedInit = false;
    gpsIntegratedAngle = 0.0f;
    gpsIntegratedAngleInit = false;
    referenceHeadingArticulated = 0.0f;
    haveReferenceHeadingArticulated = false;
    vehicleSpeedInit = false;
    encoderOffset = 0.0f;
    lastDriftCorrectionTime = 0;
    
    // Clear variance buffer
    for (uint16_t i = 0; i < config.varianceBufferSize; i++) {
        varianceBuffer[i] = 0.0f;
    }
    varianceIndex = 0;
    varianceCount = 0;
    measurementVariance = 1.0f;
}

void WheelAngleFusion::resetDriftCompensation() {
    LOG_INFO(EventSource::AUTOSTEER, "Resetting drift compensation");
    encoderDrift = 0.0f;
    driftRate = 0.0f;
    driftStartTime = millis();
}
