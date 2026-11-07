// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#include "AutosteerProcessor.h"
#include "PGNProcessor.h"
#include "ADProcessor.h"
#include "EncoderProcessor.h"
#include "MotorDriverInterface.h"
#include "TractorCANDriver.h"
#include "ConfigManager.h"
#include "LEDManagerFSM.h"
#include "EventLogger.h"
#include "QNetworkBase.h"
#include "HardwareManager.h"
#include "WheelAngleFusion.h"
#include "VehicleSetupBlob.h"          // Schritt 4b: gemeinsamer Fahrzeug-Setup-Block
#include "HeadingMonitor.h"
#include "HeadingBridge.h"             // heading/roll fall-back (coast) module
#include "GNSSProcessor.h"             // hasDualHeading/hasINS fuer die Meldungsfilterung
#include "PositionHistory.h"           // the central per-fix ring (Step 1 of the migration)
#include "PositionHistoryService.h"    // fills it once per fix - see that header
#include "MotorDriverManager.h"
#include "KickoutMonitor.h"
#include "MessageBuilder.h"
#include <cmath>  // For sin() function

// External network function
extern void sendUDPbytes(uint8_t* data, int len);

// PGN 221: Transport fuer Hardware-Meldungen (Definition weiter unten) - 2026-11-06g
static void aogHwTx(const uint8_t* d, int len);

// External objects and pointers
extern ConfigManager configManager;
// extern LEDManager ledManager; // Using FSM version now
extern ADProcessor adProcessor;
extern MotorDriverInterface* motorPTR;
extern WheelAngleFusion* wheelAngleFusionPtr;

// No longer need external network config - using ConfigManager
// extern struct NetworkConfig netConfig;

// Global pointer definition
AutosteerProcessor* autosteerPTR = nullptr;

// Vorwaertsdeklaration: definiert weiter unten im VWAS-Persistenz-Block (gleicher
// unnamed namespace). Wird schon in initializeFusion() als Hook gesetzt, also
// bevor die Definition im Quelltext steht.
namespace { void vehicleSetupSaveTrampoline(); }

// Singleton instance
AutosteerProcessor* AutosteerProcessor::instance = nullptr;

// Helper function: Get brand-specific valve/motor not ready message
static const char* getTractorValveMessage(TractorBrand brand) {
    switch (brand) {
        case TractorBrand::VALTRA_MASSEY:
            return "Valtra/MF steering valve not ready! Check CAN connection.";
        case TractorBrand::FENDT:
        case TractorBrand::FENDT_ONE:
            return "Fendt steering valve not ready! Check CAN connection.";
        case TractorBrand::CASEIH_NH:
            return "Case IH/NH steering valve not ready! Check CAN connection.";
        case TractorBrand::CLAAS:
            return "CLAAS steering valve not ready! Check CAN connection.";
        case TractorBrand::JCB:
            return "JCB steering valve not ready! Check CAN connection.";
        case TractorBrand::CAT_MT:
            return "CAT MT steering valve not ready! Check CAN connection.";
        case TractorBrand::LINDNER:
            return "Lindner steering valve not ready! Check CAN connection.";
        case TractorBrand::GENERIC:
            return "Keya motor not responding! Check CAN connection.";
        default:
            return "Tractor steering not ready! Check CAN connection.";
    }
}

AutosteerProcessor::AutosteerProcessor() {
}

AutosteerProcessor* AutosteerProcessor::getInstance() {
    if (instance == nullptr) {
        instance = new AutosteerProcessor();
    }
    return instance;
}

bool AutosteerProcessor::init() {
    // Check if already initialized to prevent duplicate PGN registrations
    static bool initialized = false;
    if (initialized) {
        LOG_DEBUG(EventSource::AUTOSTEER, "AutosteerProcessor already initialized, updating VWAS only");
        
        // Just update VWAS if needed
        if (configManager.getINSUseFusion() && !wheelAngleFusionPtr) {
            initializeFusion();
        }
        return true;
    }
    
    LOG_INFO(EventSource::AUTOSTEER, "Initializing AutosteerProcessor");

    // PGN 221: Hardware-Meldungen (2026-11-06g). Transport ist der vorhandene UDP-Weg.
    // Sprache = dieselbe Einstellung wie das Web-UI (WEB_CONFIG_ADDR: 0=EN, 1=DE, 2=LOCAL).
    // Direkt aus dem EEPROM gelesen, damit der Startzustand unabhaengig davon stimmt, ob
    // das Web-UI schon lief. 255/unbekannt -> DE.
    {
        const uint8_t lg = EEPROM.read(WEB_CONFIG_ADDR);
        hardwareMsg.begin(aogHwTx, (lg <= 2u) ? static_cast<AogMsgLang>(lg) : AogMsgLang::DE);
    }
    // LOCAL-Texte aus dem EEPROM (Block 1300). Leeres Feld -> EN als Rueckfall.
    (void)aogMsgStoreLoad(hardwareMsg);

    // The central per-fix position/state ring (knowledge/position_ring_buffer_architecture.md).
    // init() only clears head/counters: the 21.6 KB of storage is a DMAMEM array in RAM2 at
    // link time, so there is nothing to allocate, nothing to free and no heap involved.
    positionHistory.init();
    
    // Make sure instance is set
    instance = this;
    autosteerPTR = this;  // Also set global pointer
    
    // Initialize motor config tracking from EEPROM values
    previousMotorConfig = configManager.getMotorDriverConfig();
    previousCytronDriver = configManager.getCytronDriver() ? 1 : 0;
    motorConfigInitialized = true;
    LOG_INFO(EventSource::AUTOSTEER, "Motor config tracking initialized: Config=0x%02X, Cytron=%d", 
             previousMotorConfig, previousCytronDriver);
    
    // Initialize Virtual WAS if enabled
    if (configManager.getINSUseFusion()) {
        initializeFusion();
    }
    
    // Initialize button pin
    pinMode(2, INPUT_PULLUP);
    LOG_DEBUG(EventSource::AUTOSTEER, "Button pin 2 configured as INPUT_PULLUP");
    
    // Initialize LOCK output pin (SLEEP_PIN = 4)
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);  // Start with LOCK OFF (like NG-V6)
    LOG_DEBUG(EventSource::AUTOSTEER, "LOCK output pin 4 configured as OUTPUT, initially LOW");
    
    // Load steer settings from EEPROM
    configManager.loadSteerSettings();

    // Update ADProcessor with loaded values
    adProcessor.setWASOffset(configManager.getWasOffset());
    adProcessor.setWASCountsPerDegree(configManager.getSteerSensorCounts());

    // Load and apply soft start settings
    softStartDurationMs = configManager.getSoftStartDurationMs();
    softAccelDurationMs = softStartDurationMs / 2;  // Direction changes use half duration

    LOG_INFO(EventSource::AUTOSTEER, "Loaded steer settings from EEPROM: offset=%d, CPD=%d, highPWM=%d",
             configManager.getWasOffset(), configManager.getSteerSensorCounts(), configManager.getHighPWM());
    LOG_INFO(EventSource::AUTOSTEER, "Soft start: duration=%dms (accel=%dms)",
             softStartDurationMs, softAccelDurationMs);
    
    // PID functionality is now integrated directly in updateMotorControl()
    
    // Register PGN handlers with PGNProcessor
    if (PGNProcessor::instance) {
        LOG_DEBUG(EventSource::AUTOSTEER, "Registering PGN callbacks...");
        
        // Register for broadcast messages (PGN 200, 202)
        bool regBroadcast = PGNProcessor::instance->registerBroadcastCallback(handlePGNStatic, "AutosteerHandler");
        
        // Register for PGN 251 (Steer Config)
        bool reg251 = PGNProcessor::instance->registerCallback(251, handlePGNStatic, "AutosteerHandler");
        
        // Register for PGN 252 (Steer Settings)
        bool reg252 = PGNProcessor::instance->registerCallback(252, handlePGNStatic, "AutosteerHandler");
        
        // Register for PGN 254 (Steer Data with button)
        bool reg254 = PGNProcessor::instance->registerCallback(254, handlePGNStatic, "AutosteerHandler");
            
        LOG_DEBUG(EventSource::AUTOSTEER, "PGN registrations: Broadcast=%d, 251=%d, 252=%d, 254=%d", regBroadcast, reg251, reg252, reg254);
    } else {
        LOG_ERROR(EventSource::AUTOSTEER, "PGNProcessor not initialized!");
        return false;
    }
    
    // Initialize KickoutMonitor
    kickoutMonitor = KickoutMonitor::getInstance();
    if (kickoutMonitor) {
        kickoutMonitor->init(motorPTR);
        LOG_INFO(EventSource::AUTOSTEER, "KickoutMonitor initialized");
    } else {
        LOG_ERROR(EventSource::AUTOSTEER, "Failed to initialize KickoutMonitor");
    }
    
    // Heading health measurement for the UM982 HPR loss investigation. Purely
    // diagnostic - see HeadingMonitor.h. Initialised here so its banner (buffer
    // size, rate, gap threshold) is in the log from boot.
    headingMonitor.init();

    // Heading/roll fall-back module. Starts DISABLED (measuring only) so that a
    // test drive can validate the plausibility gates against the real receiver
    // before anything downstream is allowed to follow the bridged values - see
    // HeadingBridge.h and the implementation plan.
    headingBridge.init();

    LOG_INFO(EventSource::AUTOSTEER, "AutosteerProcessor initialized successfully");
    initialized = true;  // Mark as initialized to prevent duplicate PGN registrations
    return true;
}

void AutosteerProcessor::initializeFusion() {
    LOG_INFO(EventSource::AUTOSTEER, "Virtual WAS enabled - initializing VWAS system");
    
    // Create fusion instance if not already created
    if (!wheelAngleFusionPtr) {
        wheelAngleFusionPtr = new WheelAngleFusion();
    }
    
    // Initialize with sensor interfaces
    // Any motor driver that reports position feedback can feed VWAS -
    // this covers TractorCANDriver-with-Keya as well as KeyaSerialDriver
    // (RS232 Keya). hasPositionFeedback() already encodes the correct
    // per-driver logic (e.g. TractorCANDriver only returns true if the
    // configured tractor type is actually a Keya-based one).
    MotorDriverInterface* encoderDriver = nullptr;
    if (motorPTR && motorPTR->hasPositionFeedback()) {
        encoderDriver = motorPTR;
    }
    
    extern GNSSProcessor* gnssProcessorPtr;
    extern IMUProcessor imuProcessor;  // Global instance, not pointer
    
    if (wheelAngleFusionPtr->init(encoderDriver, gnssProcessorPtr, &imuProcessor)) {
        LOG_INFO(EventSource::AUTOSTEER, "Virtual WAS (VWAS) initialized successfully");

        // Vehicle geometry and encoder calibration are deliberately NOT set
        // here. The Config struct in WheelAngleFusion.h carries the documented
        // defaults, and anything the user entered in the web UI (manual entry
        // or vehicle.xml import) is restored from EEPROM by loadVehicleSetup().
        // Re-assigning literals here would be a no-op today but would silently
        // clobber future struct defaults and hide the fact that a fresh board
        // is running on uncalibrated values - that is what the validation
        // block below is for.
        loadVehicleSetup();

        // Ab jetzt darf ConfigManager melden, dass sich ein Bridge-Feld geaendert
        // hat. Erst hier ist die VWAS-Config geladen - frueher (z.B. aus
        // ConfigManager::init()) wuerde ein Schreiben den Block mit den Defaults
        // ueberschreiben statt mit dem geladenen Stand.
        g_vehicleSetupSaveHook = &vehicleSetupSaveTrampoline;

        // Acquire a DEFINED zero before anything can be reported. The stored
        // reference cannot be trusted after a power cycle (see
        // adoptBootZero()), so the alternative is an arbitrary angle - in the
        // 15-28 log that was -54.6 deg, clamped to the -45 deg limit, sent to
        // AgOpenGPS with the wheels roughly 15 deg right.
        wheelAngleFusionPtr->adoptBootZero();
        MessageBuilder::sendHardwarePopup(
            "VWAS: Nullpunkt = Stellung beim Einschalten. Geradeaus parken oder GND-Taster.",
            8, 0);

        // --- Startup validation (Phase 5) ---
        // countsPerDegree cannot be derived from the vehicle geometry, so any
        // factory default is essentially always wrong. The previously
        // hardcoded 100.0 turned out to be ~4.7x too large (the 2026-09-08
        // log implies ~21.5 counts/deg): the reported angle was far too small,
        // so the PID railed the motor into the mechanical stop. That is the
        // exact failure the operator reported. Make the uncalibrated state
        // loud instead of driving on a value that merely looks plausible.
        const WheelAngleFusion::Config& vwasCfg = wheelAngleFusionPtr->getConfig();

        if (!wheelAngleFusionPtr->isCpdCalibrated()) {
            // Name the value AND where it came from. The previous wording called
            // it a "default" while printing whatever was loaded from EEPROM,
            // which is exactly how a corrupt blob (cpd 2.54) reads as an
            // intentional built-in value.
            const bool cpdUsable = (vwasCfg.countsPerDegree >= WheelAngleFusion::CPD_MIN &&
                                    vwasCfg.countsPerDegree <= WheelAngleFusion::CPD_MAX);
            LOG_ERROR(EventSource::AUTOSTEER,
                      "VWAS NOT CALIBRATED: countsPerDegree=%.1f (from EEPROM/defaults, %s). "
                      "Run the manual endstop calibration in the web UI (/vwas) BEFORE driving. "
                      "A wrong countsPerDegree rails the steering into the mechanical stop.",
                      vwasCfg.countsPerDegree,
                      cpdUsable ? "inside the usable range" : "OUTSIDE the usable range - do not steer");
            MessageBuilder::sendHardwarePopup(
                "VWAS: Encoder NICHT kalibriert! Bitte Anschlag-Kalibrierung im Web-UI ausfuehren.", 8, 1);
        } else {
            LOG_INFO(EventSource::AUTOSTEER,
                     "VWAS cfg OK: cpd=%.2f invert=%d wheelbase=%.2f type=%d maxAngle=%.1f",
                     vwasCfg.countsPerDegree, vwasCfg.invertEncoder ? 1 : 0,
                     vwasCfg.wheelbase, vwasCfg.vehicleType, vwasCfg.maxSteeringAngle);
        }

        // --- Plan 1 consistency check: AOG's counts vs the measured value ---
        // AgOpenGPS's PGN 252 countsPerDegree now drives the VWAS conversion, so
        // the value AOG sends has to match what the calibration measured. A
        // mismatch is a pure gain error on the reported angle and AOG cannot
        // detect it - AOG only ever sees the angle, never the scale. This is the
        // check that was missing when AOG held counts=16 against a measured 20.4.
        //
        // loadSteerSettings() has already restored AOG's last known value, so
        // the warning is valid from the first boot line onwards.
        {
            const uint8_t aogCpd = configManager.getSteerSensorCounts();
            if (aogCpd == 0) {
                LOG_WARNING(EventSource::AUTOSTEER,
                            "VWAS: AgOpenGPS has sent no countsPerDegree yet - the measured "
                            "%.2f counts/deg is in use until it does.",
                            vwasCfg.countsPerDegree);
            } else if (vwasCfg.countsPerDegree > 0.01f) {
                const float relErrPct = 100.0f *
                    (fabsf((float)aogCpd - vwasCfg.countsPerDegree) / vwasCfg.countsPerDegree);
                if (relErrPct > 10.0f) {
                    LOG_WARNING(EventSource::AUTOSTEER,
                                "VWAS: AgOpenGPS counts=%u vs measured %.2f counts/deg "
                                "(%.0f%% off). AOG's value is authoritative, so the reported "
                                "angle is scaled by %.2f - set 'Counts per degree' in "
                                "AgOpenGPS to %.0f.",
                                (unsigned)aogCpd, vwasCfg.countsPerDegree, relErrPct,
                                (float)aogCpd / vwasCfg.countsPerDegree,
                                vwasCfg.countsPerDegree);
                    MessageBuilder::sendHardwarePopup(
                        "VWAS: AOG Counts per degree passt nicht zur Messung - siehe Log", 8, 1);
                } else {
                    LOG_INFO(EventSource::AUTOSTEER,
                             "VWAS: AgOpenGPS counts=%u matches the measured %.2f counts/deg "
                             "(%.1f%% off).",
                             (unsigned)aogCpd, vwasCfg.countsPerDegree, relErrPct);
                }
            }
        }

        // --- sign convention: mechanical direction XOR AgOpenGPS's invertWAS -------
        // Logged at boot because a wrong sign is not cosmetic: AgOpenGPS compares the
        // reported angle against its own setpoint, so an inverted WAS makes the steering
        // run away. Until the VWAS honoured PGN 251's invertWAS it disagreed with the
        // analog path for every operator who had ticked "Invert WAS" in AgOpenGPS -
        // which is the situation the 2026-09-21 logs show.
        {
            const bool encInv = vwasCfg.invertEncoder;
            const bool aogInv = configManager.getInvertWAS();
            LOG_INFO(EventSource::AUTOSTEER,
                     "VWAS sign: encoder invert=%d, AgOpenGPS invertWAS=%d -> reported angle is %s "
                     "(a RIGHT turn reads %s). measured cpd=%.2f",
                     encInv ? 1 : 0, aogInv ? 1 : 0,
                     (encInv != aogInv) ? "INVERTED" : "normal",
                     (encInv != aogInv) ? "NEGATIVE" : "POSITIVE",
                     vwasCfg.countsPerDegree);
            if (encInv != aogInv) {
                LOG_WARNING(EventSource::AUTOSTEER,
                            "VWAS sign: AgOpenGPS's Invert WAS and the measured encoder direction "
                            "differ, so the reported angle is flipped on purpose. If the steering "
                            "reacts the wrong way, untick Invert WAS in AgOpenGPS - or re-run the "
                            "endstop calibration if it is the calibration that is wrong.");
            }
        }

        // maxSteeringAngle is never transmitted by AgOpenGPS (verified against
        // 6.8.4: neither PGN 251 nor PGN 252 carries it), so it can only be
        // kept in sync by hand via the web UI / vehicle.xml import. A value of
        // 0 would collapse getScaledAngle() and the Ackermann/endstop maths,
        // so flag anything outside a physically sane window.
        if (vwasCfg.maxSteeringAngle <= 1.0f || vwasCfg.maxSteeringAngle > 90.0f) {
            LOG_WARNING(EventSource::AUTOSTEER,
                        "VWAS: maxSteeringAngle=%.1f looks wrong (expected ~20..60). "
                        "Import the AgOpenGPS vehicle.xml in the web UI (/vwas).",
                        vwasCfg.maxSteeringAngle);
        }
    } else {
        LOG_ERROR(EventSource::AUTOSTEER, "Failed to initialize Virtual WAS");
        configManager.setINSUseFusion(false);  // Disable VWAS
        delete wheelAngleFusionPtr;
        wheelAngleFusionPtr = nullptr;
    }
}

void AutosteerProcessor::process() {
    // === 100Hz AUTOSTEER LOOP (called by SimpleScheduler) ===
    
    // Track link state for down detection
    static bool previousLinkState = true;
    bool currentLinkState = QNetworkBase::isConnected();
    
    if (previousLinkState && !currentLinkState) {
        // Link just went DOWN
        LOG_WARNING(EventSource::AUTOSTEER, "Motor disabled - ethernet link down");
        linkWasDown = true;  // Set flag for handleSteerData
    }
    previousLinkState = currentLinkState;

    // Update Virtual WAS if enabled. Must keep running even while a manual
    // calibration session is open, because the capture buttons read
    // getRawCounts()/getEncoderAngle() live.
    if (wheelAngleFusionPtr && configManager.getINSUseFusion()) {
        float dt = 10.0f / 1000.0f;  // 10ms = 0.01 seconds (100Hz from SimpleScheduler)
        wheelAngleFusionPtr->update(dt);
    }

    // THE wheel angle for this cycle - selected ONCE, right after the fusion update, so
    // that the bridge model, the HeadingMonitor record, the PGN 253 sent to AgOpenGPS and
    // the steering control all work with the same number and the same source (see the
    // long note at selectSteerAngle()). Until 2026-09-28 the bridge had its own
    // "fusion ? fused : analog" selection here, which ignored isHealthy() and therefore
    // kept modelling with an angle the firmware itself had declared invalid.
    selectSteerAngle();

    // --- Heading health measurement (test-drive logging) ---
    // Runs regardless of the VWAS state on purpose: the receiver's HPR behaviour
    // is independent of it, and the buffer has to record what happens while the
    // steering is INACTIVE too - the encoder is polled at 50 Hz independent of
    // whether the motor is enabled (KeyaSerialDriver::process()), so the wheel
    // angle is valid even when nobody is steering.
    {
        const float steerAngleDeg = currentAngle;   // selected by selectSteerAngle() above
        // Wheel angle reliability for the MODEL decision (operator, 2026-09-28): it only
        // applies to the VIRTUAL WAS. A real physical WAS (analog input or a CAN WAS)
        // has its own hardware zero and is trustworthy from the first cycle, so it
        // reports 100 and the bridge keeps its normal behaviour. Only the VWAS - whose
        // zero has to be learned from driving, because the Keya motor sets it at
        // power-up to wherever the wheels stand - reports a number that starts at 0.
        // Comes from the SAME selection as the angle above: source and reliability can
        // never disagree.
        const float wheelQuality = steerAngleOutQuality;
        // targetAngle is AOG's commanded angle in degrees (handleSteerData()
        // stores angleRaw / 100.0f). Logging it alongside the measured angle is
        // what lets the side-slope draft term be fitted offline.
        headingMonitor.update(steerAngleDeg, targetAngle,
                              motorPWM, steerState, guidanceActive);

        // Heading/roll fall-back (coast) module: same 100 Hz tick, same wheel
        // angle the VWAS/AOG work with. The module reads the GNSS velocity
        // itself; a wheel/CAN speed can be passed instead once available.
        // The wheelbase is pushed from the VWAS config, which is filled from
        // AgOpenGPS' vehicle.xml - one single source of truth for L.
        if (wheelAngleFusionPtr && configManager.getINSUseFusion()) {
            headingBridge.setWheelbaseM(wheelAngleFusionPtr->getConfig().wheelbase);
        }
        // Wheel angle reliability for the model decision (operator request 2026-09-28:
        // "evtl im VWAS Zweig eine Zahl einbauen, die die Verlaesslichkeit des Lenkwinkel
        // anzeigt, z.B. wie DOP"). 100 for every physical WAS (analog/CAN, its zero is set
        // by hardware); 0...100 from the VWAS otherwise: 0 = the zero was never learned
        // (the Keya sets it at power-up), >= 80 = locked. The bridge uses it to choose
        // between "gate on / bias off" and "gate off / bias on" - without it the model
        // gate damps the measurements during the learning phase and the heading follows
        // the wrong wheel angle.
        headingBridge.setWheelAngleQuality(wheelQuality);
        // VWAS-Beobachter ins Log (2026-10-11, B7.17/B7.21) - reine VERDRAHTUNG: die Bridge
        // rechnet nichts davon selbst nach (ein Eigentuemer, zwei Verbraucher). enc_cnt ist der
        // VWAS-EINGANG (Grundlage fuer einen Sim-Replay), zo_res der AKTUELLE Lenkwinkelfehler
        // (Trim) und gps_a die attitude-freie Kruemmung. Spalten 103...115 der HR-Zeile.
        if (wheelAngleFusionPtr) {
            HeadingBridge::Status::WheelObs w;
            const WheelAngleFusion::ZeroObserverStatus zs = wheelAngleFusionPtr->getZeroObserverStatus();
            w.encRaw        = wheelAngleFusionPtr->getRawCounts();
            w.encoderDeg    = wheelAngleFusionPtr->getEncoderAngle();
            w.offDeg        = wheelAngleFusionPtr->getEncoderOffset();
            w.residualDeg   = zs.residualDeg;
            w.gainDegPerDeg = zs.gainDegPerDeg;
            w.gpsAngleDeg   = wheelAngleFusionPtr->getGPSAngle();
            w.cpdAog        = (float)wheelAngleFusionPtr->getAogCountsPerDegree();
            w.aogWasOff     = (float)wheelAngleFusionPtr->getAogWasOffset();
            w.rollMeanDeg   = zs.rollMeanDeg;
            w.acquireRate   = zs.acquireRateDegPerSec;
            w.zoState       = zs.state;
            w.fits          = zs.fits;
            w.aogZero       = vwasAogZeroActive;
            w.valid         = true;
            headingBridge.setWheelObserver(w);
        }
        // Steering telemetry for the raw log: the chain heading -> AOG target -> PWM.
        // Without it a reported "wild steering" cannot be told from a heading problem.
        headingBridge.setSteerTelemetry(targetAngle, motorPWM, guidanceActive);
        headingBridge.update(0.01f, steerAngleDeg);
    }

    // One record per GNSS fix for the central ring - AFTER the bridge on purpose, so
    // bodyHdgDeg is the psi that was just published (and logged) and wheelDeg the angle the
    // model used this tick. The fix gate lives inside push(): the nine of ten calls that see
    // no new fix are dropped and counted (duplicatesSkipped), which turns the 100 Hz call
    // rate into a measured number instead of a comment. wheelQuality is the SAME number the
    // bridge was just given (steerAngleOutQuality) - no second derivation of reliability.
    positionHistoryService(millis(), currentAngle, steerAngleOutQuality);

    // Drain any pending CSV dump. No-op when idle, and non-blocking by
    // construction (it only writes what the USB TX buffer has room for).
    headingMonitor.pumpDump();

    // === BUTTON/SWITCH LOGIC ===
    // Static variable for Massey/Fendt/CaseIH button state tracking (needs to persist across cycles)
    static bool lastMasseyEngageState = false;
    static bool lastFendtButtonState = false;
    static bool lastCaseIHEngageState = false;
    static bool lastCATMTEngageState = false;
    static bool lastClaasEngageState = false;
    static bool lastJcbEngageState = false;
    static bool lastLindnerEngageState = false;

    // Debug: log button/switch config periodically
    static uint32_t lastConfigLog = 0;
    if (millis() - lastConfigLog > 5000) {
        lastConfigLog = millis();
        LOG_DEBUG(EventSource::AUTOSTEER, "Button config: button=%d, switch=%d",
                  configManager.getSteerButton(), configManager.getSteerSwitch());
    }

    if (configManager.getSteerButton() || configManager.getSteerSwitch()) {
        if (configManager.getSteerButton()) {
            // BUTTON MODE - Toggle on press
            static bool lastButtonReading = HIGH;
            bool buttonReading = adProcessor.isSteerSwitchOn() ? LOW : HIGH;  // Convert to active low

            // Check CAN engage buttons via unified API or legacy per-brand flags
            bool canEngagePressed = false;
            const char* canEngageLabel = "button";

            if (motorPTR && motorPTR->getType() == MotorDriverType::TRACTOR_CAN) {
                TractorCANDriver* tractorCAN = static_cast<TractorCANDriver*>(motorPTR);

                // Unified path: protocol engine handles edge detection internally
                if (tractorCAN->checkEngageEvent()) {
                    canEngagePressed = true;
                    canEngageLabel = tractorCAN->getEngageLabel();
                }

                // Legacy fallback: check brand-specific flags when protocol engine not active
                if (!canEngagePressed) {
                    // Check Massey button (falling edge)
                    bool currentMasseyEngage = tractorCAN->isEngageButtonPressed();
                    if (!currentMasseyEngage && lastMasseyEngageState) {
                        canEngagePressed = true;
                        canEngageLabel = "Massey K_Bus button";
                    }
                    lastMasseyEngageState = currentMasseyEngage;

                    // Check Fendt button (falling edge)
                    bool currentFendtButton = tractorCAN->isFendtButtonPressed();
                    if (!currentFendtButton && lastFendtButtonState) {
                        canEngagePressed = true;
                        canEngageLabel = "Fendt armrest button";
                    }
                    lastFendtButtonState = currentFendtButton;

                    // Check Case IH (rising edge)
                    bool currentCaseIHEngage = tractorCAN->isCaseIHEngaged();
                    if (currentCaseIHEngage && !lastCaseIHEngageState) {
                        canEngagePressed = true;
                        canEngageLabel = "Case IH engage";
                    }
                    lastCaseIHEngageState = currentCaseIHEngage;

                    // Check CAT MT (rising edge)
                    bool currentCATMTEngage = tractorCAN->isCATMTEngaged();
                    if (currentCATMTEngage && !lastCATMTEngageState) {
                        canEngagePressed = true;
                        canEngageLabel = "CAT MT engage";
                    }
                    lastCATMTEngageState = currentCATMTEngage;

                    // Check CLAAS (rising edge)
                    bool currentClaasEngage = tractorCAN->isClaasEngaged();
                    if (currentClaasEngage && !lastClaasEngageState) {
                        canEngagePressed = true;
                        canEngageLabel = "CLAAS engage";
                    }
                    lastClaasEngageState = currentClaasEngage;

                    // Check JCB (rising edge)
                    bool currentJcbEngage = tractorCAN->isJcbEngaged();
                    if (currentJcbEngage && !lastJcbEngageState) {
                        canEngagePressed = true;
                        canEngageLabel = "JCB engage";
                    }
                    lastJcbEngageState = currentJcbEngage;

                    // Check Lindner (rising edge)
                    bool currentLindnerEngage = tractorCAN->isLindnerEngaged();
                    if (currentLindnerEngage && !lastLindnerEngageState) {
                        canEngagePressed = true;
                        canEngageLabel = "Lindner engage";
                    }
                    lastLindnerEngageState = currentLindnerEngage;
                }
            }

            // Check if any button was pressed (physical or CAN)
            if ((buttonReading == LOW && lastButtonReading == HIGH) || canEngagePressed) {

                // SAFETY CHECK: Verify motor/valve ready before engagement
                // Only check for TractorCAN drivers (not PWM or Keya Serial)
                if (motorPTR && motorPTR->getType() == MotorDriverType::TRACTOR_CAN) {
                    TractorCANDriver* tractorCAN = static_cast<TractorCANDriver*>(motorPTR);

                    if (tractorCAN) {
                        TractorBrand brand = tractorCAN->getCurrentBrand();
                        bool motorReady = false;

                        // Check readiness based on brand
                        if (brand == TractorBrand::GENERIC) {
                            // Keya CAN - check heartbeat
                            motorReady = tractorCAN->isHeartbeatValid();
                        } else {
                            // Tractor brands - check valve ready
                            motorReady = tractorCAN->isValveReady();
                        }

                        // Block engagement if not ready
                        if (!motorReady) {
                            const char* message = getTractorValveMessage(brand);
                            MessageBuilder::sendHardwarePopup(message, 5, 1);  // 5 sec, warning color

                            LOG_WARNING(EventSource::AUTOSTEER,
                                "Autosteer engagement blocked - motor/valve not ready (brand: %d)",
                                static_cast<int>(brand));

                            // Don't toggle state - exit early
                            lastButtonReading = buttonReading;
                            return;
                        }
                    }
                }
                // else: PWM_MOTOR or KEYA_SERIAL - no check needed, proceed normally

                // Button was just pressed - toggle state
                steerState = !steerState;
                LOG_INFO(EventSource::AUTOSTEER, "Autosteer %s via %s press",
                         steerState == 0 ? "ARMED" : "DISARMED",
                         canEngagePressed ? canEngageLabel : "button");

                // Reset encoder count when autosteer is armed
                if (steerState == 0 && EncoderProcessor::getInstance() && EncoderProcessor::getInstance()->isEnabled()) {
                    EncoderProcessor::getInstance()->resetPulseCount();
                    LOG_INFO(EventSource::AUTOSTEER, "Encoder count reset for new engagement");
                }

                // Pulse blue LED for button press
                ledManagerFSM.pulseButton();
            }
            lastButtonReading = buttonReading;
        } else {
            // SWITCH MODE - Follow switch position
            bool switchOn = adProcessor.isSteerSwitchOn();
            static bool lastSwitchState = false;

            if (switchOn != lastSwitchState) {
                // SAFETY CHECK: Verify motor/valve ready before engagement (only when arming)
                if (switchOn) {  // Only check when switching ON (arming)
                    if (motorPTR && motorPTR->getType() == MotorDriverType::TRACTOR_CAN) {
                        TractorCANDriver* tractorCAN = static_cast<TractorCANDriver*>(motorPTR);

                        if (tractorCAN) {
                            TractorBrand brand = tractorCAN->getCurrentBrand();
                            bool motorReady = false;

                            // Check readiness based on brand
                            if (brand == TractorBrand::GENERIC) {
                                motorReady = tractorCAN->isHeartbeatValid();
                            } else {
                                motorReady = tractorCAN->isValveReady();
                            }

                            // Block engagement if not ready
                            if (!motorReady) {
                                const char* message = getTractorValveMessage(brand);
                                MessageBuilder::sendHardwarePopup(message, 5, 1);

                                LOG_WARNING(EventSource::AUTOSTEER,
                                    "Autosteer engagement blocked via switch - motor/valve not ready (brand: %d)",
                                    static_cast<int>(brand));

                                // Don't update switch state - keep it disarmed
                                return;
                            }
                        }
                    }
                }

                // Switch state changed
                steerState = switchOn ? 0 : 1;  // 0 = armed, 1 = disarmed
                LOG_INFO(EventSource::AUTOSTEER, "Autosteer %s via switch", 
                         steerState == 0 ? "ARMED" : "DISARMED");
                
                // Reset encoder count when autosteer is armed
                if (steerState == 0 && EncoderProcessor::getInstance() && EncoderProcessor::getInstance()->isEnabled()) {
                    EncoderProcessor::getInstance()->resetPulseCount();
                    LOG_INFO(EventSource::AUTOSTEER, "Encoder count reset for new engagement");
                }
                
                lastSwitchState = switchOn;
            }
        }
    }
    
    // Handle deferred disarm (AOG OSB handshake)
    // After briefly arming for a valve-not-ready rejection, disarm once AOG has had
    // time to see the armed status and toggle the OSB off.
    if (pendingDisarm && (millis() - pendingDisarmTime >= DISARM_HANDSHAKE_MS)) {
        pendingDisarm = false;
        steerState = 1;
        sendPGN253();
        LOG_INFO(EventSource::AUTOSTEER, "Deferred disarm complete - motor/valve was not ready");
    }

    // Check if guidance status changed from AgOpenGPS
    if (guidanceStatusChanged) {
        LOG_INFO(EventSource::AUTOSTEER, "Guidance status changed: %s (steerState=%d, hasKickout=%d)",
                 guidanceActive ? "ACTIVE" : "INACTIVE", steerState,
                 kickoutMonitor ? kickoutMonitor->hasKickout() : 0);

        if (guidanceActive) {
            // SAFETY CHECK: Verify motor/valve ready before engagement
            if (motorPTR && motorPTR->getType() == MotorDriverType::TRACTOR_CAN) {
                TractorCANDriver* tractorCAN = static_cast<TractorCANDriver*>(motorPTR);
                if (tractorCAN) {
                    TractorBrand brand = tractorCAN->getCurrentBrand();
                    bool motorReady = false;

                    if (brand == TractorBrand::GENERIC) {
                        // Keya motor - check heartbeat validity
                        motorReady = tractorCAN->isHeartbeatValid();
                    } else {
                        // Tractor CAN valve - check valve ready status
                        motorReady = tractorCAN->isValveReady();
                    }

                    // If not ready, briefly arm so AOG sees the state change
                    // (needed for OSB handshake), then schedule a deferred disarm
                    if (!motorReady) {
                        const char* message = getTractorValveMessage(brand);
                        MessageBuilder::sendHardwarePopup(message, 5, 1);

                        LOG_WARNING(EventSource::AUTOSTEER,
                            "Motor/valve not ready (brand: %d) - arming briefly for AOG handshake",
                            static_cast<int>(brand));

                        steerState = 0;  // Arm temporarily
                        sendPGN253();    // AOG sees armed status immediately
                        pendingDisarm = true;
                        pendingDisarmTime = millis();
                        guidanceStatusChanged = false;
                        return;
                    }
                }
            }

            // Guidance turned ON in AgOpenGPS
            steerState = 0;  // Activate steering
            LOG_INFO(EventSource::AUTOSTEER, "Autosteer ARMED via AgOpenGPS (Guidance ON)");

            // If there's a kickout active, clear it
            if (kickoutMonitor && kickoutMonitor->hasKickout()) {
                kickoutMonitor->clearKickout();
                LOG_INFO(EventSource::AUTOSTEER, "KICKOUT: Cleared via AgOpenGPS (Guidance ON)");
            }
            
            // Reset encoder count when autosteer engages
            if (EncoderProcessor::getInstance() && EncoderProcessor::getInstance()->isEnabled()) {
                EncoderProcessor::getInstance()->resetPulseCount();
                LOG_INFO(EventSource::AUTOSTEER, "Encoder count reset for new engagement");
            }
        }
        guidanceStatusChanged = false;  // Clear flag
    }
    
    // If AgOpenGPS has stopped steering, turn off after delay
    // BUT only if not using a physical switch in switch mode OR button mode
    static int switchCounter = 0;
    bool physicalSwitchActive = configManager.getSteerSwitch() && adProcessor.isSteerSwitchOn();
    bool buttonModeActive = configManager.getSteerButton();

    if (steerState == 0 && !guidanceActive && !physicalSwitchActive && !buttonModeActive) {
        if (switchCounter++ > 30) {  // 30 * 10ms = 300ms delay
            steerState = 1;
            switchCounter = 0;
            LOG_INFO(EventSource::AUTOSTEER, "Autosteer DISARMED - guidance inactive");
            sendPGN253();  // Immediate feedback to AgOpenGPS
        }
    } else {
        switchCounter = 0;
    }

    // Check for valve/motor lost during active steering
    // If steering is armed and the tractor CAN valve goes not-ready, disarm immediately
    if (steerState == 0 && motorPTR && motorPTR->getType() == MotorDriverType::TRACTOR_CAN) {
        TractorCANDriver* tractorCAN = static_cast<TractorCANDriver*>(motorPTR);
        if (tractorCAN) {
            TractorBrand brand = tractorCAN->getCurrentBrand();
            bool motorReady = (brand == TractorBrand::GENERIC)
                ? tractorCAN->isHeartbeatValid()
                : tractorCAN->isValveReady();

            if (!motorReady) {
                steerState = 1;  // Disarm
                emergencyStop();
                const char* message = getTractorValveMessage(brand);
                MessageBuilder::sendHardwarePopup(message, 5, 1);
                LOG_WARNING(EventSource::AUTOSTEER, "Autosteer DISARMED - valve/motor not ready during steering");
            }
        }
    }

    // Check for work switch changes and log them
    static bool lastWorkState = false;
    bool currentWorkState = adProcessor.isWorkSwitchOn();
    if (currentWorkState != lastWorkState) {
        LOG_INFO(EventSource::AUTOSTEER, "Work switch %s", 
                 currentWorkState ? "ON (sections active)" : "OFF (sections inactive)");
        lastWorkState = currentWorkState;
    }
    
    // Pressure sensor kickout is now handled by KickoutMonitor
    static bool lastPressureSensorState = false;
    bool currentPressureSensorState = configManager.getPressureSensor();
    if (currentPressureSensorState != lastPressureSensorState) {
        LOG_INFO(EventSource::AUTOSTEER, "Pressure sensor kickout %s", 
                 currentPressureSensorState ? "ENABLED" : "DISABLED");
        lastPressureSensorState = currentPressureSensorState;
    }
    
    // Check motor status for errors (including CAN connection loss)
    if (motorPTR) {
        MotorStatus motorStatus = motorPTR->getStatus();
        if (motorStatus.hasError && steerState == 0 && guidanceActive) {
            LOG_WARNING(EventSource::AUTOSTEER, "KICKOUT: Motor error detected");
            emergencyStop();
            return;  // Skip the rest of this cycle
        }
        
        // Keya motor slip (TractorCAN+Keya) is detected via KickoutMonitor below.
    }
    
    // Track when button/switch is pressed while in kickout
    static uint32_t kickoutButtonPressTime = 0;
    static bool kickoutButtonPressed = false;
    
    // Process motor driver (for serial communication)
    if (motorPTR) motorPTR->process();
    
    // Process kickout monitoring
    if (kickoutMonitor) {
        kickoutMonitor->process();

        // Check if kickout is active while steering is armed
        if (kickoutMonitor->hasKickout() && steerState == 0) {
            // Disarm steering - this will stop the motor
            steerState = 1;  // Disarmed
            emergencyStop();
            LOG_WARNING(EventSource::AUTOSTEER, "KICKOUT: %s - steering disarmed", kickoutMonitor->getReasonString());
            kickoutButtonPressTime = millis();  // Start grace period for button clear
            kickoutButtonPressed = false;
            // Don't clear kickout here - let KickoutMonitor auto-clear when conditions return to normal
        }

        // Grace period after kickout - allow button/switch to clear it
        if (kickoutMonitor->hasKickout() && steerState == 0 && !kickoutButtonPressed &&
            millis() - kickoutButtonPressTime < 5000) {  // 5 second grace period
            // User pressed button/switch to re-arm during grace period - clear kickout
            kickoutMonitor->clearKickout();
            kickoutButtonPressed = true;
            LOG_INFO(EventSource::AUTOSTEER, "KICKOUT: Cleared via button/switch during grace period");

            // Reset encoder count
            if (EncoderProcessor::getInstance() && EncoderProcessor::getInstance()->isEnabled()) {
                EncoderProcessor::getInstance()->resetPulseCount();
                LOG_INFO(EventSource::AUTOSTEER, "Encoder count reset for new engagement");
            }
        }

        // Check for OSB re-engagement during kickout
        // When in kickout (steerState=1) and guidance is active, check if user is trying to re-engage
        // The OSB doesn't change any bits, but we can detect repeated clicks by watching for
        // guidance going off then on again quickly
        static uint32_t lastGuidanceOffTime = 0;
        static bool waitingForGuidanceOn = false;

        if (kickoutMonitor->hasKickout() && steerState == 1) {
            if (!guidanceActive && prevGuidanceStatus) {
                // Guidance just went OFF - user might have clicked OSB
                lastGuidanceOffTime = millis();
                waitingForGuidanceOn = true;
            }
            else if (guidanceActive && !prevGuidanceStatus && waitingForGuidanceOn &&
                     (millis() - lastGuidanceOffTime < 1000)) {
                // Guidance went back ON within 1 second - this is an OSB toggle
                waitingForGuidanceOn = false;

                LOG_INFO(EventSource::AUTOSTEER, "Guidance toggle detected during kickout - clearing kickout");

                // Clear kickout and re-arm
                kickoutMonitor->clearKickout();
                steerState = 0;  // Re-arm
                LOG_INFO(EventSource::AUTOSTEER, "KICKOUT: Cleared via guidance toggle");

                // Reset encoder count
                if (EncoderProcessor::getInstance() && EncoderProcessor::getInstance()->isEnabled()) {
                    EncoderProcessor::getInstance()->resetPulseCount();
                    LOG_INFO(EventSource::AUTOSTEER, "Encoder count reset for new engagement");
                }
            }
        }
    }

    // Always update current angle reading (needed for PGN253 even when autosteer is off)
    // The selection itself happens ONCE at the top of this cycle in selectSteerAngle() -
    // including AOG's wasOffset for the CAN-curve source and the reliability number. This
    // call only refreshes the scaling AgOpenGPS has sent; it runs a tick after the
    // selection, which is irrelevant for a value that changes when the operator edits the
    // vehicle settings.
    syncVwasSteerScaling();

    // Tell the zero-point observer WHO may correct the zero right now (operator
    // requirement). The measurement always runs; only the trim is gated:
    //   * guidance active or steering engaged -> blocked. AgOpenGPS's PID
    //     integral integrates the same quantity, and two integrators on one
    //     quantity cannot converge - they hunt each other.
    //   * AgOpenGPS owns the zero (operator pressed Zero-WAS) -> blocked, for the
    //     same reason seen from the firmware's side.
    // The trimming phase is therefore manual driving (e.g. the drive to the
    // field), which is also the most robust observation phase because no
    // controller is involved at all.
    if (wheelAngleFusionPtr) {
        wheelAngleFusionPtr->setZeroTrimPermissions(guidanceActive || (steerState == 0),
                                                   vwasAogZeroActive);
    }

    // THE wheel angle (currentAngle) and its reliability (steerAngleOutQuality) were
    // selected at the TOP of this cycle by selectSteerAngle() - see the long note there.
    // The PGN 253 (via actualAngle), the steering control, the HeadingMonitor record and
    // the HeadingBridge model therefore run on the same value with the same source. This
    // block used to hold the SECOND copy of that selection; the copy is gone because the
    // two could disagree (keyaWasSource == 1, wasOffset != 0, an unhealthy fusion).
    // The wasOffset for the CAN-curve source is applied inside selectSteerAngle() now
    // (analog WAS and virtual WAS fold it into their own raw->degree conversion).
    // The diagnostics ("Wheel angle source: ..." plus the loud fall-back error) are logged
    // inside selectSteerAngle() as well, so a source change is reported once per cycle -
    // i.e. from the selection that really produced the angle.
    
    // The Ackermann fix (AOG's PGN 252 byte, applied to negative = left angles) is applied
    // inside selectSteerAngle() now, so that the bridge model and the PGN 253 run on the
    // very same number. Verified in the AOG source that the PC does not correct the
    // received angle itself - see the note there. actualAngle stays as the name of the
    // published value (it is what leaves the module in PGN 253).
    actualAngle = currentAngle;
    
    // Update motor control
    updateMotorControl();
    
    // Note: LOCK output is handled by motor driver enable pin (dual-purpose)
    
    // Update LED status - simple motor state tracking
    bool motorActive = (motorState != MotorState::DISABLED);  // Check actual motor state

    // Map motor state directly to LED state
    LEDManagerFSM::SteerState ledState;
    if (motorActive) {
        ledState = LEDManagerFSM::STEER_ENGAGED;     // Green - motor running
    } else {
        ledState = LEDManagerFSM::STEER_READY;       // Amber - motor not running
    }

    // Debug logging for LED state
    static LEDManagerFSM::SteerState lastLedState = LEDManagerFSM::STEER_READY;
    if (ledState != lastLedState) {
        LOG_INFO(EventSource::AUTOSTEER, "LED state change: motor=%s -> %s",
                 motorActive ? "ACTIVE" : "DISABLED",
                 ledState == LEDManagerFSM::STEER_READY ? "AMBER" : "GREEN");
        lastLedState = ledState;
    }
    ledManagerFSM.transitionSteerState(ledState);
}

void AutosteerProcessor::handleBroadcastPGN(uint8_t pgn, const uint8_t* data, size_t len) {
    // Check if this is a Hello PGN
    if (pgn == 200) {
        // PGN 200 - Hello from AgIO, send reply
        sendHelloReply();
    }
    // Check if this is a Scan Request PGN
    else if (pgn == 202) {
        // PGN 202 - Scan request from AgIO
        bool newFormat = PGNProcessor::instance ? PGNProcessor::instance->lastScanWasNewFormat : false;
        sendScanReply(newFormat);
    }
}

void AutosteerProcessor::sendHelloReply() {
    // Hello from AutoSteer - PGN 126 (0x7E)
    // Format: {header, source, pgn, length, angleLo, angleHi, countsLo, countsHi, switches, checksum}
    
    uint8_t helloFromSteer[] = {
        128, 129,       // Header (0x80, 0x81)
        126,            // Source: Steer module (0x7E)
        126,            // PGN: Steer reply (0x7E)
        5,              // Length
        0, 0,           // Angle (0 for now)
        0, 0,           // Counts (0 for now)
        0,              // Switch byte (0 for now)
        71              // Checksum (hardcoded like NG-V6)
    };
    
    // Send via UDP
    sendUDPbytes(helloFromSteer, sizeof(helloFromSteer));
}

void AutosteerProcessor::sendScanReply(bool extended) {
    // Scan reply from AutoSteer - PGN 203 (0xCB)
    // classic  (extended=false): 13-byte reply (AgIO < 5.8 compatibility)
    // extended (extended=true) : 149-byte reply with shortname / longname / description

    uint8_t ip[4];
    configManager.getIPAddress(ip);

    if (!extended) {
        // Classic 13-byte PGN 203 for old AgIO compatibility
        uint8_t scanReply[] = {
            0x80, 0x81,                    // Header
            0x7E,                          // Source: Steer module
            0xCB,                          // PGN: 203 Scan reply
            0x07,                          // Length (data only)
            ip[0],        // IP octet 1
            ip[1],        // IP octet 2
            ip[2],        // IP octet 3
            ip[3],        // IP octet 4
            ip[0],        // Subnet octet 1 (repeat IP)
            ip[1],        // Subnet octet 2 (repeat IP)
            ip[2],        // Subnet octet 3 (repeat IP)
            0                              // CRC placeholder
        };
        uint8_t crc = 0;
        for (size_t i = 2; i < sizeof(scanReply) - 1; i++) crc += scanReply[i];
        scanReply[sizeof(scanReply) - 1] = crc;
        sendUDPbytes(scanReply, sizeof(scanReply));
        // Also send extended so Bridge/PANDA module tables work with old AgIO
    }
    // Extended 149-byte PGN 203 (always sent)
    {
        uint8_t reply[149];
        memset(reply, 0, sizeof(reply));
        reply[0] = 0x80;
        reply[1] = 0x81;
        reply[2] = 0x7E;  // Source: Steer module
        reply[3] = 0xCB;  // PGN 203
        reply[4] = 143;   // dataLen
        reply[5] = ip[0];
        reply[6] = ip[1];
        reply[7] = ip[2];
        reply[8] = ip[3];
        // bytes 9-15: zeros already
        strncpy((char*)&reply[16], configManager.getModuleShortname(),   11); reply[27]  = '\0';
        strncpy((char*)&reply[28], configManager.getModuleLongname(),    19); reply[47]  = '\0';
        strncpy((char*)&reply[48], configManager.getModuleDescription(), 99); reply[147] = '\0';
        uint8_t crc = 0;
        for (int i = 2; i <= 147; i++) crc += reply[i];
        reply[148] = crc;
        sendUDPbytes(reply, sizeof(reply));
    }
}

void AutosteerProcessor::handleSteerConfig(uint8_t pgn, const uint8_t* data, size_t len) {
    // PGN 251 - Steer Config
    // Expected length is 14 bytes total, but we get data after header
    
    LOG_DEBUG(EventSource::AUTOSTEER, "PGN 251 (Steer Config) received, %d bytes", len);
    
    // Clear any active kickout when steer config is received
    // This happens when user clicks the test button in AgOpenGPS
    if (kickoutMonitor && kickoutMonitor->hasKickout()) {
        kickoutMonitor->clearKickout();
        LOG_INFO(EventSource::AUTOSTEER, "KICKOUT: Cleared via steer config update");
        
        // Reset encoder count
        if (EncoderProcessor::getInstance() && EncoderProcessor::getInstance()->isEnabled()) {
            EncoderProcessor::getInstance()->resetPulseCount();
            LOG_INFO(EventSource::AUTOSTEER, "Encoder count reset via steer config");
        }
    }
    
    // Always show debug info first, regardless of packet validity
    char debugMsg[256];
    snprintf(debugMsg, sizeof(debugMsg), "Raw PGN 251 data:");
    for (int i = 0; i < len && strlen(debugMsg) < 200; i++) {
        char buf[20];
        snprintf(buf, sizeof(buf), " [%d]=0x%02X(%d)", i, data[i], data[i]);
        strncat(debugMsg, buf, sizeof(debugMsg) - strlen(debugMsg) - 1);
    }
    LOG_DEBUG(EventSource::AUTOSTEER, "%s", debugMsg);
    
    if (len < 4) {  // Minimum needed for basic config
        LOG_ERROR(EventSource::AUTOSTEER, "PGN 251 too short! Got %d bytes", len);
        return;
    }
    
    // Data array indices (after header/length removal):
    // Per PGN.md: set0, pulseCount, minSpeed, sett1, ***, ***, ***, ***
    // [0] = setting0 byte
    // [1] = pulseCountMax
    // [2] = minSpeed
    // [3] = setting1 byte
    // [4-7] = reserved/unused
    
    uint8_t sett0 = data[0];
    bool invertWAS = bitRead(sett0, 0);
    bool motorDriveDirection = bitRead(sett0, 2);
    bool cytronDriver = bitRead(sett0, 4);
    bool steerSwitch = bitRead(sett0, 5);
    bool steerButton = bitRead(sett0, 6);
    bool shaftEncoder = bitRead(sett0, 7);
    
    uint8_t pulseCountMax = data[1];  // Fixed: was data[2]
    uint8_t minSpeed = data[2];       // Fixed: was data[3]
    
    uint8_t sett1 = data[3];              // Fixed: was data[4]
    bool isDanfoss = bitRead(sett1, 0);
    bool pressureSensor = bitRead(sett1, 1);
    bool currentSensor = bitRead(sett1, 2);
    bool isUseYAxis = bitRead(sett1, 3);
    
    // When current sensor is enabled, data[1] (pulseCountMax) is repurposed as current threshold
    if (currentSensor) {
        uint8_t currentThreshold = data[1];
        LOG_INFO(EventSource::AUTOSTEER, "Current sensor enabled - threshold=%d (%.1f%%)", 
                  currentThreshold, (currentThreshold * 100.0f) / 255.0f);
        configManager.setCurrentThreshold(currentThreshold);
    } else if (pressureSensor) {
        // When pressure sensor is enabled, data[1] might be pressure threshold
        uint8_t pressureThreshold = data[1];
        LOG_INFO(EventSource::AUTOSTEER, "Pressure sensor enabled - threshold=%d (%.1f%%)", 
                  pressureThreshold, (pressureThreshold * 100.0f) / 255.0f);
        configManager.setPressureThreshold(pressureThreshold);
    } else if (shaftEncoder) {
        // When encoder is enabled, data[1] is pulse count max
        LOG_INFO(EventSource::AUTOSTEER, "Shaft encoder enabled - pulseCountMax=%d", 
                  pulseCountMax);
    }
    
    // Read motor driver configuration from byte 8 of the message (data array index 3)
    // Message structure: Header(5) + Data(8) + CRC(1) = 14 bytes total
    // Byte 8 of the message = data[3] (since data array starts after 5-byte header)
    uint8_t motorDriverConfig = data[3];
    
    // Workaround: Clear Cytron bit when Danfoss is selected
    // AgOpenGPS doesn't always clear this bit when switching to Danfoss
    if (isDanfoss || (motorDriverConfig & 0x01)) {
        cytronDriver = false;
    }
    
    // Update motor driver manager with new configuration
    MotorDriverManager::getInstance()->updateMotorConfig(motorDriverConfig);
    
    LOG_DEBUG(EventSource::AUTOSTEER, "InvertWAS: %d", invertWAS);
    LOG_DEBUG(EventSource::AUTOSTEER, "MotorDriveDirection: %d", motorDriveDirection);
    LOG_DEBUG(EventSource::AUTOSTEER, "SteerSwitch: %d", steerSwitch);
    LOG_DEBUG(EventSource::AUTOSTEER, "SteerButton: %d", steerButton);
    LOG_DEBUG(EventSource::AUTOSTEER, "PulseCountMax: %d", pulseCountMax);
    LOG_DEBUG(EventSource::AUTOSTEER, "MinSpeed: %d", minSpeed);
    
    // Determine motor type from config
    const char* motorType = "Unknown";
    switch (motorDriverConfig) {
        case 0x00: motorType = cytronDriver ? "Cytron IBT2" : "DRV8701"; break;
        case 0x01: motorType = "Danfoss"; break;
        case 0x02: motorType = cytronDriver ? "Cytron IBT2" : "DRV8701"; break;
        case 0x03: motorType = "Danfoss"; break;
        case 0x04: motorType = cytronDriver ? "Cytron IBT2" : "DRV8701"; break;
        default: motorType = "Unknown"; break;
    }
    
    // Determine steer enable type
    const char* steerType = "None";
    if (steerButton) {
        steerType = "Button";
    } else if (steerSwitch) {
        steerType = "Switch";
    }
    
    // Log all settings at INFO level in a single message so users see everything
    LOG_INFO(EventSource::AUTOSTEER, "Steer config: WAS=%s Motor=%s MinSpeed=%d Steer=%s Encoder=%s Pressure=%s Current=%s (max=%d) MotorType=%s", 
             invertWAS ? "Inv" : "Norm",
             motorDriveDirection ? "Rev" : "Norm",
             minSpeed,
             steerType,
             shaftEncoder ? "Yes" : "No",
             pressureSensor ? "Yes" : "No",
             currentSensor ? "Yes" : "No",
             pulseCountMax,
             motorType);
             
    // Additional debug for encoder configuration
    LOG_DEBUG(EventSource::AUTOSTEER, "Encoder Debug: ShaftEncoder=%d, IsDanfoss=%d, MotorConfig=0x%02X, MotorType=%s",
             shaftEncoder, isDanfoss, motorDriverConfig, motorType);
    
    
    // Save config to EEPROM
    configManager.setInvertWAS(invertWAS);
    configManager.setMotorDriveDirection(motorDriveDirection);
    configManager.setCytronDriver(cytronDriver);
    configManager.setSteerSwitch(steerSwitch);
    configManager.setSteerButton(steerButton);
    configManager.setShaftEncoder(shaftEncoder);
    configManager.setPressureSensor(pressureSensor);
    configManager.setCurrentSensor(currentSensor);
    configManager.setPulseCountMax(pulseCountMax);
    configManager.setMinSpeed(minSpeed);
    configManager.setMotorDriverConfig(motorDriverConfig);
    configManager.setIsUseYAxis(isUseYAxis);  // Save Y-axis swap setting for IMU
    
    // Note: Sensor configuration updates would go here if we want dynamic changes
    // For now, sensor changes require reboot to ensure clean state
    
    // Check for motor type changes
    bool motorTypeChanged = false;
    int8_t currentCytronDriver = cytronDriver ? 1 : 0;
    
    // Only check for changes if we've initialized from EEPROM
    if (motorConfigInitialized) {
        LOG_DEBUG(EventSource::AUTOSTEER, "Current motor state: Config=0x%02X, Cytron=%d (previous: Config=0x%02X, Cytron=%d)",
                  motorDriverConfig, currentCytronDriver,
                  previousMotorConfig, previousCytronDriver);
        
        // Only check motor-relevant bits (bit 0 = Danfoss, CytronDriver is separate)
        // Ignore sensor bits (bits 1-2) when checking for motor changes
        uint8_t previousMotorBits = previousMotorConfig & 0x01;  // Danfoss bit only
        uint8_t currentMotorBits = motorDriverConfig & 0x01;
        
        if (previousMotorBits != currentMotorBits ||
            previousCytronDriver != currentCytronDriver) {
            LOG_INFO(EventSource::AUTOSTEER, "Motor change detected: Danfoss %d->%d, Cytron %d->%d",
                     previousMotorBits, currentMotorBits,
                     previousCytronDriver, currentCytronDriver);
            motorTypeChanged = true;
        }
    } else {
        // First PGN 251 received, but we should already be initialized from init()
        LOG_WARNING(EventSource::AUTOSTEER, "Motor config not initialized - this shouldn't happen!");
    }
    
    // Update tracked values
    previousMotorConfig = motorDriverConfig;
    previousCytronDriver = currentCytronDriver;
    
    configManager.saveSteerConfig();
    configManager.saveTurnSensorConfig();  // Also save turn sensor config (includes current threshold)
    LOG_INFO(EventSource::AUTOSTEER, "Steer config saved to EEPROM");

    // Apply encoder settings if changed
    if (EncoderProcessor::getInstance()) {
        bool currentEncoderEnabled = EncoderProcessor::getInstance()->isEnabled();

        // Update encoder enable state if changed
        if (currentEncoderEnabled != shaftEncoder) {
            EncoderProcessor::getInstance()->updateConfig(
                (EncoderType)configManager.getEncoderType(), shaftEncoder);
            LOG_INFO(EventSource::AUTOSTEER, "Encoder %s via PGN 251", shaftEncoder ? "enabled" : "disabled");
        }
    }


    if (motorTypeChanged) {
        LOG_WARNING(EventSource::AUTOSTEER, "Motor type changed - rebooting in 2 seconds...");
        delay(2000);
        SCB_AIRCR = 0x05FA0004; // Teensy Reset
    }
}

void AutosteerProcessor::handleSteerSettings(uint8_t pgn, const uint8_t* data, size_t len) {
    // PGN 252 - Steer Settings
    // Expected length is 14 bytes total, but we get data after header
    
    LOG_DEBUG(EventSource::AUTOSTEER, "PGN 252 (Steer Settings) received, %d bytes", len);
    
    if (len < 8) {
        LOG_ERROR(EventSource::AUTOSTEER, "PGN 252 too short!");
        return;
    }
    
    // Parse PGN 252 data directly to local variables
    uint8_t kp = data[0];
    uint8_t highPWM = data[1];
    // data[2] is lowPWM â€” sent by AgOpenGPS but unused in steering logic
    uint8_t minPWM = data[3];
    uint8_t steerSensorCounts = data[4];
    int16_t wasOffset = data[5] | (data[6] << 8);
    float ackermanFix = (float)data[7] * 0.01f;

    LOG_INFO(EventSource::AUTOSTEER, "Steer settings: Kp=%d PWM min=%d high=%d WAS_offset=%d counts=%d Ackerman=%.2f",
             kp, minPWM, highPWM, wasOffset, steerSensorCounts, ackermanFix);

    // --- AgOpenGPS zero-point ownership (see the header note) --------------
    //
    // AOG re-sends its stored wasOffset continuously, so only a CHANGE means the
    // operator acted (Zero-WAS button, Smart-WAS apply, or the offset slider).
    // The first receipt of a session is the stored value, never an action - and it
    // must not be applied, because it is expressed in encoder counts on a counter
    // that restarted at the last Keya power cycle.
    if (!vwasAogZeroBaselineSeen) {
        vwasAogZeroBaseline = wasOffset;
        vwasAogZeroBaselineSeen = true;
        LOG_INFO(EventSource::AUTOSTEER,
                 "VWAS: AgOpenGPS wasOffset baseline for this session = %d counts. "
                 "NOT applied yet - press Zero-WAS in AOG (or move the offset slider) to "
                 "make AOG the zero-point owner.",
                 (int)wasOffset);
    } else if (wasOffset != vwasAogZeroBaseline) {
        const int16_t delta = (int16_t)(wasOffset - vwasAogZeroBaseline);
        vwasAogZeroBaseline = wasOffset;
        vwasAogZeroLastChangeMs = millis();

        // AOG's Zero-WAS arithmetic is wasOffset += countsPerDegree * -angle
        // (FormSteer.cs:903/911), so a deliberate zeroing produces a change of
        // about counts * -reportedAngle. Comparing the two tells a real button
        // press apart from a dragged slider; it is reported, not enforced, so a
        // legitimate action is never swallowed by a stale expectation.
        const float reported = wheelAngleFusionPtr ? wheelAngleFusionPtr->getFusedAngle() : 0.0f;
        const float cpdNow = (float)configManager.getSteerSensorCounts();
        const float predicted = -cpdNow * reported;
        const bool matches = (fabsf((float)delta - predicted) <= 0.5f * cpdNow + 1.0f);

        LOG_WARNING(EventSource::AUTOSTEER,
                    "VWAS: AgOpenGPS changed wasOffset to %d counts (delta=%d, expected "
                    "about %.0f for a Zero-WAS click at %.2f deg%s)",
                    (int)wasOffset, (int)delta, predicted, reported,
                    matches ? ", matches" : " - MISMATCH (slider?)");

        if (!vwasAogZeroActive) {
            vwasAogZeroActive = true;
            syncVwasSteerScaling();   // apply immediately so the check below is real

            // AOG's button is defined to make the reported angle read 0 at the
            // position where it was pressed. Verify that it did: if not, the
            // mechanical zero and AOG's trim disagree, which is exactly the state
            // that produced the stacked -54.6 deg in the 15-28 drive.
            const float after = wheelAngleFusionPtr ? wheelAngleFusionPtr->getFusedAngle() : 0.0f;
            if (fabsf(after) > 1.0f) {
                LOG_WARNING(EventSource::AUTOSTEER,
                            "VWAS: after applying AgOpenGPS's wasOffset the reported angle is "
                            "%.2f deg instead of ~0 - AOG's trim does not match the mechanical "
                            "zero. Re-run the endstop calibration or press Zero-WAS again.",
                            after);
            } else if (cpdNow != 0.0f) {
                LOG_INFO(EventSource::AUTOSTEER,
                         "VWAS: AgOpenGPS now owns the zero point (wasOffset=%d counts = "
                         "%.2f deg)",
                         (int)wasOffset, (float)wasOffset / cpdNow);
            }
        }
    }

    adProcessor.setWASOffset(wasOffset);
    adProcessor.setWASCountsPerDegree(steerSensorCounts);

    // Save steer settings to ConfigManager
    configManager.setKp(kp);
    configManager.setHighPWM(highPWM);
    configManager.setMinPWM(minPWM);
    configManager.setSteerSensorCounts(steerSensorCounts);
    configManager.setWasOffset(wasOffset);
    configManager.setAckermanFix(ackermanFix);
    configManager.saveSteerSettings();
    LOG_INFO(EventSource::AUTOSTEER, "Steer settings saved to EEPROM");

    // Log confirmation that settings are now active
    LOG_INFO(EventSource::AUTOSTEER, "Settings now active - no reboot required. Motor will use new PWM values immediately.");
}

void AutosteerProcessor::handleSteerData(uint8_t pgn, const uint8_t* data, size_t len) {
    // PGN 254 - Steer Data (comes at 10Hz from AgOpenGPS)
    // For now, we only care about the autosteer enable bit
    
    if (len < 3) {
        LOG_DEBUG(EventSource::AUTOSTEER, "PGN 254 too short, ignoring");
        return;  // Too short, ignore
    }
    
    // Check if we're recovering from a link down event
    static uint32_t linkUpTime = 0;
    static bool waitingForStableLink = false;
    
    if (linkWasDown) {
        // Link was down, now we're receiving PGN 254 again
        linkUpTime = millis();
        waitingForStableLink = true;
        linkWasDown = false;  // Clear the flag
    }
    
    // Wait 3 seconds for network negotiation after link restoration
    if (waitingForStableLink && (millis() - linkUpTime > 3000)) {
        LOG_INFO(EventSource::AUTOSTEER, "Communication restored - motor under AOG control");
        waitingForStableLink = false;
    }
    
    lastPGN254Time = millis();
    lastCommandTime = millis();  // Update watchdog timer
    
    // Debug: Log raw PGN 254 data
    // Data format (from PGNProcessor we get data starting at speed):
    // [0-1] = Speed (uint16, 0.1 km/h resolution)
    // [2] = Status byte
    //       Bit 0: Guidance active
    //       Bit 6: Autosteer enable (this is what we need for button)
    // [3-4] = Steer angle setpoint (int16, divide by 100 for degrees)
    // [5] = XTE (cross track error)
    // [6] = Machine sections 1-8
    // [7] = Machine sections 9-16
    
    // Extract speed
    vehicleSpeed = (uint16_t)(data[1] << 8 | data[0]) * 0.1f; // Convert to km/h
    
    // Extract status
    uint8_t status = data[2];
    // Track guidance status changes
    static bool firstBroadcast = true;
    bool newGuidanceActive = (status & 0x01) != 0;   // Bit 0 is guidance active
    
    if (firstBroadcast) {
        // On first broadcast, just set the status without triggering change
        guidanceActive = newGuidanceActive;
        prevGuidanceStatus = newGuidanceActive;
        guidanceStatusChanged = false;
        firstBroadcast = false;
    } else {
        // Normal operation - detect changes
        prevGuidanceStatus = guidanceActive;
        guidanceActive = newGuidanceActive;
        guidanceStatusChanged = (guidanceActive != prevGuidanceStatus);
    }
    // Extract steer angle
    int16_t angleRaw = (int16_t)(data[4] << 8 | data[3]);
    targetAngle = angleRaw / 100.0f;
    
    // Debug log for AgIO test mode
    if (targetAngle != 0.0f || autosteerEnabled) {
        LOG_DEBUG(EventSource::AUTOSTEER, "PGN254: speed=%.1f km/h, target=%.1fÂ°, enabled=%d, guidance=%d", 
                  vehicleSpeed, targetAngle, autosteerEnabled, guidanceActive);
    }
    
    // Extract XTE
    crossTrackError = (int8_t)data[5];
    
    // Extract sections
    uint8_t sections1_8 = data[6];
    uint8_t sections9_16 = data[7];
    machineSections = (uint16_t)(sections9_16 << 8 | sections1_8);

    autosteerEnabled = (status & 0x40) != 0;  // Bit 6 is autosteer enable

    // Send PGN 253 status 1:1 with each PGN254 received from AgOpenGPS
    sendPGN253();
}

// Static callback wrapper
void AutosteerProcessor::handlePGNStatic(uint8_t pgn, const uint8_t* data, size_t len) {
    if (instance) {
        // Don't print for PGN 254, 200, or 202 since they come frequently
        if (pgn != 254 && pgn != 200 && pgn != 202) {
            LOG_DEBUG(EventSource::AUTOSTEER, "Received PGN %d", pgn);
        }
        
        // Handle broadcast PGNs
        if (pgn == 200 || pgn == 202) {
            instance->handleBroadcastPGN(pgn, data, len);
        }
        else if (pgn == 251) {
            instance->handleSteerConfig(pgn, data, len);
        }
        else if (pgn == 252) {
            instance->handleSteerSettings(pgn, data, len);
        }
        else if (pgn == 254) {
            instance->handleSteerData(pgn, data, len);
        }
        // We'll add other PGNs one at a time as needed
    } else {
        LOG_ERROR(EventSource::AUTOSTEER, "No instance!");
    }
}

// ---------------------------------------------------------------------------
// THE wheel angle of this firmware - one source of truth per cycle
// ---------------------------------------------------------------------------
// Operator, 2026-09-28: "AOG bekommt den lenkwinkel per PGN aus dem teensy modul. AOG
// sieht die Quelle nicht, es ist die Aufgabe der firmware den lenkwinkel zu liefern, egal
// woher. Es gibt aber auf anderen Fahrzeugen jede Kombination: Keya mit analogem WAS oder
// mit CAN WAS." PAOGI/PANDA carry position, heading, roll/pitch and yaw rate - never a
// wheel angle, so the angle reaches AgOpenGPS through PGN 253 ONLY.
//
// This selection therefore happens ONCE per cycle and every consumer uses the result:
//   * the PGN 253 wheel angle (with the Ackermann convention, applied where it always was),
//   * the steering control,
//   * the HeadingMonitor record,
//   * the HeadingBridge model (omega = v*tan(delta)/L).
// Until 2026-09-28 there were TWO selections in process(): a simple one for the bridge
// (fusion ? fused : analog) and this one for AOG. They differed whenever
//   * keyaWasSource == 1 (CAN curve): AOG got the CAN angle, the bridge the analog one,
//   * AOG's wasOffset != 0: only the delivered value carried the correction,
//   * the fusion became UNHEALTHY: only this block falls back (with the loud error below,
//     because the analog input is not wired on this board) - the bridge kept modelling
//     with the UNHEALTHY fused angle.
// The reliability number for the model decision is derived right here, from the source
// that was actually selected: 100 for every physical WAS (analog input or CAN - its zero
// is set by hardware and valid from the first cycle), the VWAS's own DOP-like number for
// the virtual one (0 = the zero was never learned, because the Keya sets it at power-up
// to wherever the wheels stand). One addition: when a VIRTUAL WAS is configured but the
// fusion is unhealthy, the selection falls back to the analog input, which is NOT wired
// on this board - that fallback reports 0 as well, so the bridge never models with a
// floating-pin artefact.
void AutosteerProcessor::selectSteerAngle() {
    // Keya's CAN "curve" value is motor-side, not a real WAS, so for TractorCAN+Keya the
    // wheel-angle source is an explicit user choice (Analog/CAN/Virtual, see keyaWasSource)
    // instead of an automatic priority chain. Other CAN valve tractors do provide a real
    // WAS over CAN and keep using it automatically whenever available.
    uint8_t angleSource = 0;  // 0=analog, 1=fusion, 2=can-curve
    float selectedAngle = 0.0f;
    float wheelQuality = 100.0f;

    bool isTractorCANKeya = false;
    bool valveDataReceived = false;
    uint8_t keyaWasSource = 0;  // 0=Analog, 1=CAN curve, 2=Virtual (fusion) - only used when isTractorCANKeya
    if (motorPTR && motorPTR->getType() == MotorDriverType::TRACTOR_CAN) {
        TractorCANDriver* tractorCAN = static_cast<TractorCANDriver*>(motorPTR);
        isTractorCANKeya = (tractorCAN && tractorCAN->hasKeyaMotor());
        valveDataReceived = (tractorCAN && tractorCAN->isValveDataReceived());
        CANSteerConfig canConfig = configManager.getCANSteerConfig();
        keyaWasSource = canConfig.reserved[0];

        if (isTractorCANKeya) {
            if (keyaWasSource == 2 && configManager.getINSUseFusion() && wheelAngleFusionPtr && wheelAngleFusionPtr->isHealthy()) {
                selectedAngle = wheelAngleFusionPtr->getFusedAngle();
                wheelQuality = wheelAngleFusionPtr->getAngleQuality();
                angleSource = 1;
            } else if (keyaWasSource == 1 && valveDataReceived) {
                int16_t canCurve = tractorCAN->getActualCurve();
                float scale = tractorCAN->getCurveScale();
                selectedAngle = (float)canCurve / scale;
                angleSource = 2;
            } else {
                selectedAngle = adProcessor.getWASAngle();
                // keyaWasSource == 0 (operator picked the analog WAS) means a real physical
                // sensor with a hardware zero -> 100. Any other value here means he picked
                // the VIRTUAL WAS but the fusion is unhealthy, in which case this analog
                // number comes from an input that is not wired on this board (floating ADC)
                // and must be reported as unusable, otherwise the bridge would model with it.
                wheelQuality = (keyaWasSource == 0) ? 100.0f : 0.0f;
                angleSource = 0;
            }
        } else if (tractorCAN && valveDataReceived) {
            // Non-Keya tractor valve systems provide real WAS feedback over CAN
            int16_t canCurve = tractorCAN->getActualCurve();
            float scale = tractorCAN->getCurveScale();
            selectedAngle = (float)canCurve / scale;
            angleSource = 2;
        } else if (configManager.getINSUseFusion() && wheelAngleFusionPtr && wheelAngleFusionPtr->isHealthy()) {
            selectedAngle = wheelAngleFusionPtr->getFusedAngle();
            wheelQuality = wheelAngleFusionPtr->getAngleQuality();
            angleSource = 1;
        } else {
            selectedAngle = adProcessor.getWASAngle();
            // No fusion configured at all -> a real analog WAS, hardware zero, wq=100.
            // Fusion configured but unhealthy -> the analog input is unwired here, so the
            // number is unusable: report 0 so the bridge gates its model (see the note in
            // the Keya branch above).
            wheelQuality = configManager.getINSUseFusion() ? 0.0f : 100.0f;
            angleSource = 0;
        }
    } else if (configManager.getINSUseFusion() && wheelAngleFusionPtr && wheelAngleFusionPtr->isHealthy()) {
        selectedAngle = wheelAngleFusionPtr->getFusedAngle();
        wheelQuality = wheelAngleFusionPtr->getAngleQuality();
        angleSource = 1;
    } else {
        selectedAngle = adProcessor.getWASAngle();
        // Same rule as above: a real analog WAS reports 100, the unwired input that the
        // unhealthy fusion falls back to reports 0.
        wheelQuality = configManager.getINSUseFusion() ? 0.0f : 100.0f;
        angleSource = 0;
    }

    // AgOpenGPS's Zero-WAS button and Smart WAS auto-calibration are
    // angle-source-agnostic on the PC side - they only nudge wasOffset /
    // countsPerDegree (PGN 252) until the reported angle reads ~0 while driving
    // straight. The offset therefore has to be applied inside whichever source
    // owns the raw -> degree conversion:
    //
    //   analog WAS   ADProcessor::getWASAngle()  (raw - 2048 - wasOffset) / cpd
    //   Virtual WAS  WheelAngleFusion (plan 1)   (signedCounts + wasOffset) / cpd
    //   CAN curve    no raw -> degree conversion exists in the firmware for this
    //                source, so the same correction is added here, in degrees.
    //
    // The VWAS used to be handled by this block as well (currentAngle +=
    // wasOffset/cpd). That is deliberately gone: with the offset folded into the
    // encoder formula there is exactly ONE place per source where the zero point
    // is applied, which is what makes AOG's counts slider and its Zero-WAS/Zero
    // button behave on the VWAS identically to a real WAS - and it removes the
    // path that silently added -8.4 deg (wasOffset=-134 / counts=16) to every
    // angle in the 2026-09-21 test run.
    //
    // Sign is '+' for the same reason as in the fusion: AOG's Zero-WAS does
    // wasOffset += cpd * -actualSteerAngleDegrees (FormSteer.cs:903/911), which
    // only converges if the firmware adds the offset.
    if (angleSource == 2) {
        float cpd = (float)configManager.getSteerSensorCounts();
        if (cpd != 0.0f) {
            selectedAngle += (float)configManager.getWasOffset() / cpd;
        }
    }

    static uint8_t lastAngleSource = 255;
    if (angleSource != lastAngleSource) {
        const uint8_t previousSource = lastAngleSource;
        lastAngleSource = angleSource;
        const char* sourceName = (angleSource == 2) ? "CAN curve" :
                                 (angleSource == 1) ? "Fusion" : "Analog WAS";

        // A fallback from the fusion to the ANALOG input is not a benign
        // re-route on this board: there is no physical WAS, the ADC pin is left
        // floating (ADProcessor sets INPUT_DISABLE), and the reported angle
        // becomes (floatingRaw - 2048 - wasOffset)/counts - which is a large
        // constant, not a steering angle. It happens whenever isHealthy() turns
        // false, and under Plan 1 one of those conditions is
        // |fusedAngle| > 1.5 * maxSteeringAngle, i.e. a too-small AOG counts
        // value can push us into it. Make that visible as an error instead of a
        // neutral info line, so the operator does not chase the wrong fault.
        if (angleSource == 0 && previousSource == 1 && configManager.getINSUseFusion()) {
            LOG_ERROR(EventSource::AUTOSTEER,
                      "VWAS: fusion became UNHEALTHY - wheel angle fell back to the analog WAS "
                      "input, which is not wired up on this board (floating ADC). Treat the "
                      "reported angle as invalid. fusedAngle=%.2f maxSteeringAngle=%.1f "
                      "AOG counts=%u wasOffset=%d. Check the AOG 'Counts per degree' value and "
                      "the encoder wiring.",
                      wheelAngleFusionPtr ? wheelAngleFusionPtr->getFusedAngle() : 0.0f,
                      wheelAngleFusionPtr ? wheelAngleFusionPtr->getConfig().maxSteeringAngle : 0.0f,
                      (unsigned)configManager.getSteerSensorCounts(),
                      (int)configManager.getWasOffset());
        }

        LOG_INFO(EventSource::AUTOSTEER,
                 "Wheel angle source: %s (tractorCANKeya=%d, valveData=%d, keyaWasSource=%d) "
                 "wq=%.0f",
                 sourceName,
                 isTractorCANKeya ? 1 : 0,
                 valveDataReceived ? 1 : 0,
                 keyaWasSource,
                 (double)wheelQuality);
    }

    // AgOpenGPS's Ackermann factor (PGN 252 byte 8, `setAS_ackerman`, AOG default 100)
    // belongs to this pipeline, because it defines the value AgOpenGPS receives: verified
    // in the AOG source (GPS/Forms/UDPComm.Designer.cs, PGN 253 case) the PC takes the
    // angle EXACTLY as it arrives - `mc.actualSteerAngleDegrees = (Int16)((data[6]<<8)+
    // data[5]) * 0.01` - without any correction of its own. So applying it here is the
    // only application, there is no double correction, and the bridge model runs on the
    // same number with which AOG's own steer model works (AOG derives the byte in its
    // steer wizard, so it has to be set empirically rather than trusted blindly).
    const float ackermanFix = configManager.getAckermanFix();
    if (selectedAngle < 0.0f) {
        const float preAckerman = selectedAngle;
        selectedAngle *= ackermanFix;

        // Log the application periodically (left = negative angle only).
        static uint32_t lastAckermanLog = 0;
        if (millis() - lastAckermanLog > 5000 && abs(selectedAngle) > 1.0f) {
            lastAckermanLog = millis();
            LOG_DEBUG(EventSource::AUTOSTEER, "Ackerman fix applied: %.2fÂ° * %.2f = %.2fÂ°",
                      preAckerman, ackermanFix, selectedAngle);
        }
    }

    // Publish: from here on this is THE wheel angle of the firmware for this cycle. The
    // PGN 253 (via actualAngle), the steering control, the HeadingMonitor record and the
    // HeadingBridge model all read currentAngle / steerAngleOutQuality - none of them
    // selects a source or scales the angle again.
    currentAngle = selectedAngle;
    steerAngleOutQuality = wheelQuality;

}

// ---------------------------------------------------------------------------
// PGN 221: Hardware-Meldungen nach AgOpenGPS (2026-11-06g)
// ---------------------------------------------------------------------------
// Format/Katalog: lib/aio_system/aog_hwmsg.h. In AOG muss dafuer
// Config -> Display -> "Hardware Messages" eingeschaltet sein, sonst blendet AOG
// das Label nicht ein (GUI.Designer.cs: isHardwareMessages = false).
static void aogHwTx(const uint8_t* d, int len) {
    sendUDPbytes(const_cast<uint8_t*>(d), len);
}

bool AutosteerProcessor::sendHardwareMessage(const char* text, uint8_t seconds, uint8_t colour,
                                             uint8_t prio, uint32_t minGapMs) {
    return hardwareMsg.sendText(text, seconds, colour, prio, minGapMs, millis());
}

bool AutosteerProcessor::sendHardwareMessage(AogMsgId id) {
    return hardwareMsg.send(id, millis());
}

// LOCAL-Texte dauerhaft ablegen (EEPROM-Block 1300). NUR vom Web-UI beim Speichern
// aufrufen: Teensy-EEPROM ist flash-emuliert, jeder write kopiert einen 4-kB-Sektor um.
void AutosteerProcessor::saveLocalHardwareMessageTexts() {
    aogMsgStoreSave(hardwareMsg);
}

// Warnungen aus dem Bridge-Zustand. Ein ZUSTAND wird erst nach 2 s gemeldet (sonst flattert
// die Anzeige bei jedem schlechten Epoch), ein EREIGNIS (Positionssprung) sofort. Abstaende
// und Prioritaeten stehen im Katalog, damit sich Meldungen nicht gegenseitig verdraengen.
void AutosteerProcessor::updateHardwareMessages() {
    if (!hardwareMsg.isEnabled()) { return; }   // Hauptschalter aus (Web-UI)
    const uint32_t now = millis();
    if (hwMsgLastMs != 0u && (uint32_t)(now - hwMsgLastMs) < 100u) { return; }
    hwMsgLastMs = (now == 0u) ? 1u : now;
    const HeadingBridge::Status& st = headingBridge.getStatus();
    // HARDWARE-ABHAENGIGKEIT (Betreiber-Frage 2026-11-06): die beiden Bridge-Meldungen setzen
    // eine KOERPER-Lage voraus (Dual-Antenne oder INS). Ohne sie wird `attitudeTrusted` nie
    // wahr und der Zustand bleibt laut Zustandsautomat fuer immer in BRIDGE/COAST - die
    // Meldung wuerde dann STEHEND leuchten statt zu warnen. Sie werden deshalb nur gesendet,
    // wenn eine Koerper-Lage existiert. Kurs-/Positionsmeldungen (NoCourse, PosJump) haengen
    // nur an den Positionen und laufen auf jeder Hardware.
    const GNSSProcessor::GNSSData& hwGnss = gnssProcessor.getData();
    const bool haveAttitude = hwGnss.hasDualHeading || hwGnss.hasINS;
    if (!st.attitudeTrusted) { if (hwWeakT0 == 0u) { hwWeakT0 = now; } } else { hwWeakT0 = 0u; }
    const bool noCourse = (!st.pathChordValid && !st.travelWitnessValid && (st.speedFusedMps > 1.0f));
    if (noCourse) { if (hwNoCourseT0 == 0u) { hwNoCourseT0 = now; } } else { hwNoCourseT0 = 0u; }
    if (st.pathChordSpike) { (void)sendHardwareMessage(AogMsgId::PosJump); }
    if (haveAttitude &&
        ((st.state == HeadingBridge::State::BRIDGE) || (st.state == HeadingBridge::State::COAST))) {
        (void)sendHardwareMessage(AogMsgId::Bridge);
    }
    if (haveAttitude && (hwWeakT0 != 0u) && ((uint32_t)(now - hwWeakT0) >= 2000u)) {
        (void)sendHardwareMessage(AogMsgId::WeakFix);
    }
    if ((hwNoCourseT0 != 0u) && ((uint32_t)(now - hwNoCourseT0) >= 2000u)) {
        (void)sendHardwareMessage(AogMsgId::NoCourse);
    }
}

void AutosteerProcessor::sendPGN253() {
    // PGN 221 (Hardware-Meldungen): hier mitlaufen lassen, sendPGN253() ist der regulaere
    // Takt zum PC (2026-11-06g). Eigene Entprellung innen (10 Hz).
    updateHardwareMessages();
    // PGN 253 - Status data TO AgOpenGPS
    // Format: {header, source, pgn, length, 
    //          steerAngleLo, steerAngleHi,    // bytes 5-6: actual steer angle
    //          headingLo, headingHi,           // bytes 7-8: IMU heading (deprecated)
    //          rollLo, rollHi,                 // bytes 9-10: roll (deprecated)
    //          switchByte,                     // byte 11: switch states
    //          pwmDisplay,                     // byte 12: PWM value
    //          checksum}
    
    // Get actual values - use actualAngle which includes Ackerman correction
    // 2026-11-06: Ohne Lenkwinkel-Messung (Knicklenker-Testmaeher) geht der GESCHAETZTE
    // Knickwinkel raus - AgOpenGPS knickt die Maschine damit (Darstellung + Section Coverage)
    // und halbiert ihn selbst (CVehicle.cs: modelSteerAngle = 0.5 * actualSteerAngleDegrees).
    // Mit echtem Sensor bleibt alles wie bisher (Qualitaet > 0).
    float angleForAog = actualAngle;
    if ((steerAngleOutQuality <= 0.0f) && headingBridge.hasArticulationEstimate()) {
        angleForAog = headingBridge.getArticulationEstimateDeg();
    }
    int16_t actualSteerAngle = (int16_t)(angleForAog * 100.0f);  // Actual angle * 100
    int16_t heading = 0;            // Deprecated - sent by GNSS
    int16_t roll = 0;               // Deprecated - sent by GNSS  
    uint8_t livePwmDisplay = (uint8_t)abs(motorPWM);  // Already in 0-255 range
    bool kickoutActiveNow = (kickoutMonitor && kickoutMonitor->hasKickout());

    // Freeze PWM telemetry at kickout so AgOpenGPS can display the causal value briefly.
    if (kickoutActiveNow && !lastKickoutStateForTelemetry) {
        frozenPwmDisplay = livePwmDisplay;
        pwmFreezeStartTime = millis();
    }
    lastKickoutStateForTelemetry = kickoutActiveNow;

    bool inPostKickoutWindow = (pwmFreezeStartTime != 0) &&
                               ((uint32_t)(millis() - pwmFreezeStartTime) < POST_KICKOUT_TELEMETRY_MS);
    uint8_t pwmDisplay = (kickoutActiveNow || inPostKickoutWindow) ? frozenPwmDisplay : livePwmDisplay;
    
    // Build switch byte
    // Bit 0: work switch (inverted)
    // Bit 1: steer switch (inverted steerState)
    // Bit 2: remote/kickout input
    // Bit 3: unused
    // Bit 4: unused
    // Bit 5: fusion active (1 = fusion, 0 = WAS)
    uint8_t switchByte = 0;
    switchByte |= (0 << 2);        // No remote/kickout for now
    switchByte |= (steerState << 1);  // Steer state in bit 1
    switchByte |= !adProcessor.isWorkSwitchOn();  // Work switch state (inverted) in bit 0
    
    uint8_t pgn253[] = {
        0x80, 0x81,                    // Header
        0x7E,                          // Source: Steer module (126)
        0xFD,                          // PGN: 253
        8,                             // Length
        (uint8_t)(actualSteerAngle & 0xFF),      // Steer angle low
        (uint8_t)(actualSteerAngle >> 8),        // Steer angle high
        (uint8_t)(heading & 0xFF),                // Heading low
        (uint8_t)(heading >> 8),                  // Heading high
        (uint8_t)(roll & 0xFF),                   // Roll low
        (uint8_t)(roll >> 8),                     // Roll high
        switchByte,                               // Switch byte
        pwmDisplay,                               // PWM display
        0                                         // CRC placeholder
    };
    
    // Calculate CRC
    uint8_t crc = 0;
    for (int i = 2; i < sizeof(pgn253) - 1; i++) {
        crc += pgn253[i];
    }
    pgn253[sizeof(pgn253) - 1] = crc;
    
    // Send via UDP
    sendUDPbytes(pgn253, sizeof(pgn253));
}

void AutosteerProcessor::updateMotorControl() {
    // Current angle is now updated in process() before this function is called
    
    // Check if steering should be active
    bool shouldBeActive = shouldSteerBeActive();
    
    // Ramp length by speed: unchanged at/above RAMP_SLOW_KMH, linearly doubling
    // down to RAMP_FULL_KMH, capped below that. At creep speed the GPS/heading
    // noise outweighs the vehicle motion, so a slower build-up stops the motor
    // from chasing noise. The factor is computed once per cycle and applied at
    // the moment a ramp starts (inside the transitions below), never while a
    // ramp is running - otherwise a speed wobble around 3 km/h would jump the
    // ramp progress.
    // Speed source is the same one the steering speed gate uses (GPS ring, EMA
    // smoothed), never the 0.1 km/h-quantised PGN 254 value. A NaN or missing
    // speed leaves the factor at 1.0, i.e. today's behaviour.
    // Note: this is a stand-in for the data quality (see 01_ziele.md D6 - filter
    // by data quality, not by speed); speed is only the proxy for it.
    const float speedKmhNow = (wheelAngleFusionPtr ? wheelAngleFusionPtr->getSpeedMps()
                                                  : vehicleSpeed / 3.6f) * 3.6f;
    float rampFactor = 1.0f;
    if (speedKmhNow < RAMP_SLOW_KMH) {
        rampFactor = (speedKmhNow <= RAMP_FULL_KMH)
                   ? RAMP_MAX_FACTOR
                   : 1.0f + (RAMP_SLOW_KMH - speedKmhNow) / (RAMP_SLOW_KMH - RAMP_FULL_KMH)
                            * (RAMP_MAX_FACTOR - 1.0f);
    }

    // Handle state transitions
    if (shouldBeActive && motorState == MotorState::DISABLED) {
        // Transition: Start soft-start sequence
        motorState = MotorState::SOFT_START;
        rampDurationActiveMs = (uint16_t)(softStartDurationMs * rampFactor);  // speed-scaled, frozen here
        softStartBeginTime = millis();
        softStartRampValue = 0.0f;
        filteredCommandPwm = 0.0f;  // Reset filter so it doesn't start from a stale value
        LOG_INFO(EventSource::AUTOSTEER, "Motor STARTING - soft-start sequence (%dms, %.1f km/h)",
                 rampDurationActiveMs, speedKmhNow);
        // Update LED immediately
        ledManagerFSM.transitionSteerState(LEDManagerFSM::STEER_ENGAGED);
        LOG_INFO(EventSource::AUTOSTEER, "LED -> GREEN (motor starting)");
    } 
    else if (!shouldBeActive && motorState != MotorState::DISABLED) {
        // Transition: Disable motor
        motorState = MotorState::DISABLED;
        lastPwmDrive = 0;
        motorPWM = 0;
        filteredCommandPwm = 0.0f;
        if (motorPTR) {
            motorPTR->enable(false);
            motorPTR->setPWM(0);

            // LOCK output control
            if (motorPTR->getType() == MotorDriverType::KEYA_CAN) {
                // Directly control LOCK output for Keya motor
                digitalWrite(4, LOW);  // SLEEP_PIN = 4, LOW = LOCK OFF
                LOG_INFO(EventSource::AUTOSTEER, "LOCK output: INACTIVE (pin 4 LOW for Keya motor)");
            } else {
                // For PWM motors, pin 4 is controlled by the motor driver
                LOG_INFO(EventSource::AUTOSTEER, "LOCK output: INACTIVE (motor disabled)");
            }
        }
        // Update LED immediately when motor disabled
        ledManagerFSM.transitionSteerState(LEDManagerFSM::STEER_READY);
        LOG_INFO(EventSource::AUTOSTEER, "LED -> AMBER (motor disabled)");

        // Give more specific disable reason
        if (!QNetworkBase::isConnected()) {
            // Already logged in link state change detection above
        } else if (vehicleSpeed <= 0.1f) {
            LOG_INFO(EventSource::AUTOSTEER, "Motor disabled - speed too low (%.1f km/h)", vehicleSpeed);
        } else if (!guidanceActive) {
            LOG_INFO(EventSource::AUTOSTEER, "Motor disabled - guidance inactive");
        } else if (steerState != 0) {
            LOG_INFO(EventSource::AUTOSTEER, "Motor disabled - steer switch off");
        } else if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
            LOG_INFO(EventSource::AUTOSTEER, "Motor disabled - communication timeout");
        } else {
            LOG_INFO(EventSource::AUTOSTEER, "Motor disabled");
        }
        return;
    }
    else if (!shouldBeActive) {
        // Already disabled, nothing to do
        return;
    }
    
    // Ackerman fix is now applied in process() before this function is called
    
    // Calculate angle error
    float angleError = actualAngle - targetAngle;

    // Get PWM settings from ConfigManager (cached for performance)
    uint8_t kp = configManager.getKp();
    uint8_t highPWM = configManager.getHighPWM();
    uint8_t minPWM = configManager.getMinPWM();

    // Debug log to verify settings are being read
    static uint32_t lastSettingsVerifyLog = 0;
    static uint8_t lastKp = 0;
    static uint8_t lastHighPWM = 0;
    if (millis() - lastSettingsVerifyLog > 5000 || kp != lastKp || highPWM != lastHighPWM) {
        lastSettingsVerifyLog = millis();
        lastKp = kp;
        lastHighPWM = highPWM;
        LOG_INFO(EventSource::AUTOSTEER, "Active PWM settings: Kp=%d, highPWM=%d, minPWM=%d", kp, highPWM, minPWM);
    }
    
    // Calculate base PWM output (error * Kp)
    int16_t pValue = kp * angleError;
    
    // Apply PWM calculation similar to V6
    if (highPWM > 0) {  // Check if we have valid settings
        // Start with base P value
        int16_t pwmDrive = pValue;
        
        // Add min throttle factor so no delay from motor resistance
        if (pwmDrive < 0) {
            pwmDrive -= minPWM;
        } else if (pwmDrive > 0) {
            pwmDrive += minPWM;
        } else {
            // Dead zone - set pwmDrive to 0
            pwmDrive = 0;
        }
        
        // Limit the PWM drive to highPWM setting
        if (pwmDrive > highPWM) {
            pwmDrive = highPWM;
        } else if (pwmDrive < -highPWM) {
            pwmDrive = -highPWM;
        }

        // Check for hard acceleration - soften if needed
        // Use lastPwmDrive (pre-ramp, pre-inversion) so the check is not corrupted by the ramp value or direction inversion
        uint8_t accelThreshold = (uint8_t)((highPWM - minPWM) * ACCEL_THRESHOLD_RATIO);
        if ((abs(pwmDrive) > (highPWM - accelThreshold)) &&
            (abs(lastPwmDrive) < (minPWM + accelThreshold)) &&
            (motorState == MotorState::NORMAL_CONTROL)) {
            motorState = MotorState::SOFT_ACCEL;
            rampDurationActiveMs = (uint16_t)(softAccelDurationMs * rampFactor);  // speed-scaled
            softStartBeginTime = millis();
            LOG_DEBUG(EventSource::AUTOSTEER, "Hard acceleration detected - entering soft accel mode (%dms, %.1f km/h)",
                      rampDurationActiveMs, speedKmhNow);
        }

        // Check for direction change - if so, enter soft-start again
        // Use lastPwmDrive (pre-ramp, pre-inversion) to avoid false triggers from ramp or motorDriveDirection inversion
        if (((lastPwmDrive > 0 && pwmDrive < 0) || (lastPwmDrive < 0 && pwmDrive > 0)) &&
            (abs(pwmDrive) > (minPWM + DIRECTION_CHANGE_THRESHOLD)) &&
            (motorState == MotorState::NORMAL_CONTROL)) {
            motorState = MotorState::SOFT_ACCEL;
            rampDurationActiveMs = (uint16_t)(softAccelDurationMs * rampFactor);  // speed-scaled
            softStartBeginTime = millis();
            filteredCommandPwm = 0.0f;  // Reset filter on direction change
            LOG_DEBUG(EventSource::AUTOSTEER, "Direction change detected - entering soft accel mode (%dms, %.1f km/h)",
                      rampDurationActiveMs, speedKmhNow);
        }

        // Store final PWM value and save raw pwmDrive for next cycle's direction/accel checks
        // lastPwmDrive is saved BEFORE ramp and BEFORE motorDriveDirection inversion
        lastPwmDrive = pwmDrive;
        motorPWM = pwmDrive;
        
        // Log the PWM calculation periodically
        static uint32_t lastPWMCalcLog = 0;
        if (millis() - lastPWMCalcLog > 5000) {  // Every 5 seconds
            lastPWMCalcLog = millis();
            LOG_DEBUG(EventSource::AUTOSTEER, "PWM calc: actual=%.1fÂ° - target=%.1fÂ° = error=%.1fÂ° * Kp=%d = %d, +minPWM=%d, limit=%d, final=%d", 
                     actualAngle, targetAngle, angleError, kp, pValue, minPWM, highPWM, pwmDrive);
        }
        
        // Debug log final motor PWM periodically
        static uint32_t lastMotorPWMLog = 0;
        if (millis() - lastMotorPWMLog > 1000 && abs(motorPWM) > 10) {
            lastMotorPWMLog = millis();
            LOG_DEBUG(EventSource::AUTOSTEER, "Motor PWM: %d (highPWM=%d)", 
                      motorPWM, highPWM);
        }
        
        // Apply soft ramp if active (soft-start or soft-accel)
        if (motorState == MotorState::SOFT_START || motorState == MotorState::SOFT_ACCEL) {
            uint32_t elapsed = millis() - softStartBeginTime;
            // Length was frozen at ramp entry (speed-scaled); clamp it once here so
            // a future wider config range cannot produce a runaway ramp.
            uint16_t durationMs = rampDurationActiveMs;
            if (durationMs > RAMP_MAX_MS) durationMs = RAMP_MAX_MS;
            const char* modeName = (motorState == MotorState::SOFT_START) ?
                                  "Soft-start" : "Soft-accel";

            if (elapsed >= durationMs) {
                // Ramp complete, transition to normal
                motorState = MotorState::NORMAL_CONTROL;
                LOG_INFO(EventSource::AUTOSTEER, "%s complete - normal steering control", modeName);
            } else {
                // Calculate ramp progress (0.0 to 1.0)
                float rampProgress = (float)elapsed / (float)durationMs;

                // Apply ramp function based on configuration
                float rampValue;
                if (useSineRamp) {
                    // Use sine curve for smooth acceleration (slow-fast-slow)
                    rampValue = sin(rampProgress * PI / 2.0f);
                } else {
                    // Use linear ramp
                    rampValue = rampProgress;
                }

                // Apply ramp to motor PWM
                int16_t originalPWM = motorPWM;
                motorPWM = (int16_t)((float)motorPWM * rampValue);
                // Note: no minPWM clamping here - pwmDrive already includes minPWM,
                // so the ramp naturally passes through minPWM proportionally.
                // Clamping would bypass the smooth ramp from 0.

                softStartRampValue = rampValue;

                // Debug logging every 50ms during ramp
                static uint32_t lastRampDebug = 0;
                if (millis() - lastRampDebug > 50) {
                    lastRampDebug = millis();
                    LOG_DEBUG(EventSource::AUTOSTEER, "%s: elapsed=%dms, progress=%.2f, ramp=%.2f, pwm=%d->%d",
                              modeName, elapsed, rampProgress, rampValue, originalPWM, motorPWM);
                }
            }
        }
    } else {
        // No valid PWM config
        motorPWM = 0;
        LOG_ERROR(EventSource::AUTOSTEER, "Invalid PWM configuration");
    }
    
    // Apply motor direction from config
    if (configManager.getMotorDriveDirection()) {
        motorPWM = -motorPWM;  // Invert if configured
    }

    int16_t filteredMotorPwm = motorPWM;
    if (motorState == MotorState::NORMAL_CONTROL) {
        // Apply low-pass filter and minimum threshold only in normal control
        float alpha = configManager.getPwmFilterAlpha() / 100.0f;
        if (alpha > 0.97f) alpha = 0.97f;
        filteredCommandPwm = alpha * filteredCommandPwm + (1.0f - alpha) * (float)motorPWM;
        float threshold = (float)configManager.getMinPWM() * configManager.getPwmMinThresholdPct() / 100.0f;
        if (fabsf(filteredCommandPwm) < threshold) {
            filteredMotorPwm = 0;
        } else {
            filteredMotorPwm = (int16_t)constrain((int)filteredCommandPwm, -255, 255);
        }
    } else {
        // During soft-start / soft-accel: bypass filter, use ramp PWM directly
        // and keep filteredCommandPwm in sync so normal control starts smoothly
        filteredCommandPwm = (float)motorPWM;
    }
    
    // Send to motor
    if (motorPTR) {
        // Only enable motor if not in disabled state
        if (motorState != MotorState::DISABLED) {
            motorPTR->enable(true);
            motorPTR->setPWM(filteredMotorPwm);
            
            // Debug log to confirm PWM is being sent
            static uint32_t lastMotorCmdLog = 0;
            if (millis() - lastMotorCmdLog > 1000) {
                lastMotorCmdLog = millis();
                LOG_DEBUG(EventSource::AUTOSTEER, "Sending to motor: rawPWM=%d filteredPWM=%d State=%d", 
                          motorPWM, filteredMotorPwm, (int)motorState);
            }
        }
        
        // LOCK output control
        // For PWM motors, pin 4 is controlled by the motor driver
        // For Keya/CAN motors, we need to control pin 4 directly
        if (motorPTR->getType() == MotorDriverType::KEYA_CAN) {
            // Directly control LOCK output for Keya motor
            digitalWrite(4, HIGH);  // SLEEP_PIN = 4, HIGH = LOCK ON
            
            static bool lockLogged = false;
            if (motorState == MotorState::NORMAL_CONTROL && !lockLogged) {
                LOG_INFO(EventSource::AUTOSTEER, "LOCK output: ACTIVE (pin 4 HIGH for Keya motor)");
                lockLogged = true;
            } else if (motorState == MotorState::DISABLED) {
                lockLogged = false;
            }
        }
    }
    
}

bool AutosteerProcessor::shouldSteerBeActive() const {
    // Check kickout cooldown
    if (kickoutTime > 0 && (millis() - kickoutTime < KICKOUT_COOLDOWN_MS)) {
        return false;  // Still in cooldown
    }
    
    // Check ethernet link
    if (!QNetworkBase::isConnected()) {
        return false;  // No ethernet link
    }
    
    // Check watchdog timeout
    if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
        return false;  // No recent commands
    }
    
    // Check all enable conditions
    // Note: We use guidanceActive (bit 0) and steerState instead of autosteerEnabled (bit 6)
    // because AgOpenGPS may not set bit 6 until it receives confirmation from us
    //
    // --- steering speed gate (operator requirement 2026-09-21) -----------------
    // "The motor must be free at standstill = autosteer off. We should steer from
    //  120 m/h upwards, below that not at all. We have to RECOGNISE the noise."
    //
    // Two independent guards, because they fail in different ways:
    //
    //  1. SPEED, on the GPS speed - NOT on the PGN 254 byte. AOG sends
    //     (int)(avgSpeed_kmh * 10.0), i.e. 100 m/h per LSB and TRUNCATED: 120 m/h
    //     arrives as byte 1, which is read back as 0.1 km/h = 100 m/h, i.e. BELOW
    //     the floor. A byte-based 120 m/h gate could therefore never open. The byte
    //     is kept only as a fallback when the fusion reports nothing.
    //     Latching hysteresis (open above the floor, release below 75 %) plus a
    //     500 ms rising-edge debounce: one GPS speed spike at standstill would
    //     otherwise hold the motor on for the entire 2 s closing grace period.
    //
    //  2. GEOMETRY, the standstill veto from WheelAngleFusion. No low-pass can
    //     separate a steady "0.1 km/h" bias from real creep; net displacement over
    //     3 s can (see STANDSTILL_* there). It acts immediately and also cancels
    //     the closing grace, because 3 s without net travel IS the measurement.
    //
    // AOG's own "Min Steer Speed" may ask for a HIGHER floor and is honoured when
    // it does; the firmware floor covers the range AOG cannot express.
    static bool     speedGateOpen = false;
    static uint32_t aboveSince    = 0;
    static uint32_t lastAboveTime = 0;

    float speedMps = wheelAngleFusionPtr ? wheelAngleFusionPtr->getSpeedMps()
                                         : (vehicleSpeed / 3.6f);
    const float aogFloorMps     = configManager.getMinSpeed() / 36.0f;  // 0.1 km/h -> m/s
    const float speedFloorMps   = (aogFloorMps > STEER_MIN_SPEED_MPS) ? aogFloorMps
                                                                      : STEER_MIN_SPEED_MPS;
    const float speedReleaseMps = speedFloorMps * 0.75f;

    bool standstillVeto = false;
    if (wheelAngleFusionPtr && wheelAngleFusionPtr->hasStandstillAssessment()) {
        standstillVeto = wheelAngleFusionPtr->isStandstill();
    }

    bool above = speedGateOpen ? (speedMps > speedReleaseMps)
                               : (speedMps > speedFloorMps);
    if (standstillVeto) {
        above = false;
        speedGateOpen = false;
        aboveSince = 0;
        lastAboveTime = 0;  // cancels the closing grace as well
    } else if (above) {
        if (aboveSince == 0) aboveSince = millis();
        if ((millis() - aboveSince) >= SPEED_OPEN_DEBOUNCE_MS) {
            speedGateOpen = true;
            lastAboveTime = millis();
        }
    } else {
        aboveSince = 0;
        speedGateOpen = false;
    }

    bool speedOk = speedGateOpen ||
                   (!standstillVeto && lastAboveTime != 0 &&
                    (millis() - lastAboveTime) < SPEED_GRACE_MS);

    bool active = guidanceActive &&           // Guidance line active (bit 0 from PGN 254)
                  (steerState == 0) &&        // Our button/OSB state (0=active)
                  speedOk;                    // Moving (speed gate + standstill veto)
    
    // Debug logging for the engagement decision.
    //   pgnSpd  - PGN 254, quantised to 0.1 km/h by AOG (it sends
    //             (int)(avgSpeed*10.0), i.e. TRUNCATED). Reported for comparison
    //             only - the gate above runs on gpsSpd, see the note there.
    //   gpsSpd  - the 80/20-EMA-filtered GPS speed inside WheelAngleFusion. This is
    //             what the speed gate actually uses.
    //   floor   - the effective speed floor: max(AOG's Min Steer Speed, 120 m/h).
    //   net/std - net displacement over the 3 s standstill window and the resulting
    //             veto. Without these two the log cannot distinguish "AOG says
    //             0.1 km/h and we believed it" from "we recognised it as noise".
    // wasRaw is the raw analog WAS reading, which is also the GND-button
    // threshold detector - logging it shows whether that threshold is sane.
    static uint32_t lastDebugTime = 0;
    // 0.5 Hz (operator decision 2026-09-23): the 10 Hz raw frame line from
    // HeadingBridge carries speed, wheel angle and steering state, so this control
    // line only has to show that the control path is alive.
    if (millis() - lastDebugTime > 2000) {
        lastDebugTime = millis();
        LOG_INFO(EventSource::AUTOSTEER,
                 "VWAS ctrl: pgnSpd=%.1fkm/h(byte=%d) gpsSpd=%.3fm/s wasRaw=%d minSpd=%d "
                 "floor=%.4fm/s net=%.3fm std=%d guid=%d sState=%d ko=%d pwm=%d "
                 "act=%.2f tgt=%.2f -> %s",
                 vehicleSpeed, (int)(vehicleSpeed * 10.0f),
                 speedMps,
                 adProcessor.getWASRaw(), configManager.getMinSpeed(),
                 speedFloorMps,
                 wheelAngleFusionPtr && wheelAngleFusionPtr->hasStandstillAssessment()
                     ? wheelAngleFusionPtr->getNetDisplacementM() : -1.0f,
                 standstillVeto ? 1 : 0,
                 guidanceActive ? 1 : 0, steerState,
                 (kickoutMonitor && kickoutMonitor->hasKickout()) ? 1 : 0,
                 motorPWM, actualAngle, targetAngle, active ? "YES" : "NO");
    }
    
    return active;
}

void AutosteerProcessor::emergencyStop() {
    LOG_WARNING(EventSource::AUTOSTEER, "EMERGENCY STOP");
    
    // Reset motor state
    motorState = MotorState::DISABLED;
    filteredCommandPwm = 0.0f;
    lastPwmDrive = 0;   // upstream 26.6.x: without this the next engage compares against a stale raw drive
    
    // Disable motor immediately
    motorPWM = 0;
    if (motorPTR) {
        motorPTR->setPWM(0);
        motorPTR->enable(false);
        
        // Ensure LOCK is OFF on kickout
        if (motorPTR->getType() == MotorDriverType::KEYA_CAN) {
            digitalWrite(4, LOW);  // SLEEP_PIN = 4, LOW = LOCK OFF
        }
    }
    
    // Set inactive state
    steerState = 1;
    
    // Start kickout cooldown
    kickoutTime = millis();
}

// ============================================================================
// VWAS manual calibration (web UI buttons)
// ============================================================================
// The USER turns the steering wheel by hand. The firmware only records raw
// encoder counts and computes countsPerDegree from them. The motor is never
// driven, so there is no current draw and no "drive into the stop until the
// overload trips" failure mode, and no timeout. The straight-ahead reference
// (centreCounts) is session-only; countsPerDegree/invert are persisted.

// ============================================================================
// Analog-WAS GND button -> "Geradeausstellung setzen" (Punkt 2a)
// ============================================================================
// The AiO board's protected analog WAS input is read by the Teensy ADC in
// ADProcessor::updateWAS() (12-bit, centre 2048, full scale 4095) and refreshed
// every ADProcessor::process() cycle. With a pull-up on that input, a button to
// GND collapses the reading to ~0 counts. A real WAS potentiometer sits at
// ~2048 at centre and does not approach 0 even at full lock, so "very low" is
// an unambiguous press detection rather than a threshold on a live signal.
//
// Deliberately reuses adProcessor.getWASRaw() instead of calling analogRead()
// here: a second ADC read would reset the ADC1 configuration that
// PWMMotorDriver/ADProcessor depend on (see the warning in PWMMotorDriver.cpp).







// ============================================================================
// VWAS config EEPROM persistence
// ============================================================================
// Stores the whole WheelAngleFusion::Config struct as one block, guarded by
// a magic byte so we don't load garbage from a blank/uninitialized EEPROM.
namespace {
    // Guard values for the VWAS config blob at VWAS_CONFIG_ADDR.
    //
    // The whole WheelAngleFusion::Config struct is stored as one raw block, so
    // adding, removing or reordering ANY field silently reinterprets every value
    // that was already stored. The failure is invisible and looks like broken
    // hardware - in the field it appeared as maxSteeringAngle == 0, which:
    //   * zeroed fusedAngle completely, because the clamp in
    //     WheelAngleFusion::updateKalmanFilter treats 0 as a valid limit
    //     ("Constrain final angle to reasonable limits"), so AgOpenGPS received
    //     a constant 0 deg while the web UI showed a plausible cpd of 2.54,
    //   * made the endstop calibration impossible, because
    //     computeCountsPerDegree() rejects maxSteeringAngleDeg < 1.0 - with the
    //     misleading message "straight-ahead reference not between the stops".
    //
    // History: 0xEE was current while Config still carried trackWidth and
    // centerPosition. Those two were removed (decision 1b / Phase 5), which
    // shifted every field after them - so the blob written by the previous
    // firmware is misread by this one. 0xEF invalidated it.
    //
    // 0xF0: the straight-ahead reference (centerCounts) is now written into the
    // blob as well. It used to live ONLY in WheelAngleFusion as a runtime member
    // that nothing persisted, so the reference was 0 after every reboot while
    // cpd and the calibrated flag survived - producing a phantom angle with the
    // wheels straight (2026-09-21 11-59 log: ctr=62 in the capturing session,
    // ctr=0 after the restart, i.e. -4.06 deg at straight ahead).
    //
    // VWAS_CONFIG_LAYOUT is stored and compared as well, so the blob invalidates
    // itself automatically when the layout changes size even if somebody forgets
    // to bump the magic. (Size alone does not catch a same-size reorder - for
    // that the magic must still be bumped by hand.)
    // 0xF1 (2026-11-01): invertEncoder steht jetzt fest auf true (Wert der Maschine, siehe
    // WheelAngleFusion.h) - die Anschlag-Kalibrierung, die ihn frueher setzte, ist aus dem
    // Web-IO entfernt. Damit dieser neue Default auch in einem GERAET ankommt, in dem schon
    // ein 0xF0-Blob liegt, wird die Magic einmalig hochgezaehlt: der Block wird verworfen
    // und frisch geschrieben. Die Fahrzeug-Geometrie muss danach EINMAL neu gesetzt werden
    // (vehicle.xml-Import im Web-IO).
    // ------------------------------------------------------------------------
    // 2026-11-02 (Schritt 4b): Der alte Block an dieser Stelle ist ABGELOEST.
    // Bisher lagen hier Magic 0xF1 + Layout-Groesse + Config + centerCounts
    // einzeln hintereinander. Jetzt liegt bei VEHICLE_SETUP_ADDR EIN Block:
    // magic(2) layoutVer(2) payloadLen(2) reserviert(2) crc32(4) + Nutzlast
    // VehicleSetupBlob (VWAS-Config + centerCounts + Bridge).
    //
    // Was der neue Guard kann und der alte nicht:
    //   * der alte prueft nur, ob die GROESSE stimmt - ein gekipptes Bit oder
    //     ein gleich grosses Umsortieren kam durch,
    //   * die crc32 deckt die Nutzlast ab und erkennt beides,
    //   * payloadLen im Kopf laesst einen gewachsenen Config-Struct auffallen,
    //     statt ihn halb zu lesen,
    //   * VWAS und Bridge haben EINEN Guard statt Magic + Marker-PAAR.
    //
    // Der alte 0xF1-Block wird NICHT mehr gelesen: das erste Speichern
    // ueberschreibt ihn. Die Fahrzeug-Geometrie kommt danach aus dem
    // vehicle.xml-Import (Betreiber-Entscheidung 2026-11-02: der Verlust ist
    // hinnehmbar, weil in allen Faellen Defaults hinterlegt sind).
    // ------------------------------------------------------------------------

    // Trampolin fuer g_vehicleSetupSaveHook (VehicleSetupBlob.h). ConfigManager
    // meldet ueber saveHeadingBridgeConfig(), dass sich ein Bridge-Feld geaendert
    // hat - die VWAS-Config kennt er nicht, also schreibt dieser hier den Block.
    //
    // getInstance() ist hier unbedenklich: ist die Instanz noch nicht da oder
    // wheelAngleFusionPtr noch nullptr, kehrt saveVehicleSetup() sofort zurueck.
    void vehicleSetupSaveTrampoline() {
        AutosteerProcessor* as = AutosteerProcessor::getInstance();
        if (as) as->saveVehicleSetup();
    }
}

void AutosteerProcessor::syncVwasSteerScaling() {
    if (!wheelAngleFusionPtr) return;

    // The SCALE always comes from AgOpenGPS: countsPerDegree is a ratio, so it
    // survives a power cycle and legitimately belongs to AOG (plan 1).
    //
    // The OFFSET only while AOG is the current owner of the zero point. That is
    // not "ignoring AOG": wasOffset is expressed in encoder counts on a counter
    // that restarts with the motor, so a value that came out of EEPROM cannot be
    // valid now - applying it adds an error of its own magnitude, which is the
    // stacked -11.4 deg seen in the 15-28 logs.
    const int16_t wasOffsetToApply = vwasAogZeroActive ? configManager.getWasOffset() : 0;
    wheelAngleFusionPtr->setAogSteerScaling(configManager.getSteerSensorCounts(),
                                           wasOffsetToApply,
                                           configManager.getInvertWAS());
}


void AutosteerProcessor::saveVehicleSetup() {
    if (!wheelAngleFusionPtr) return;

    // Nutzlast zusammenstellen. blob{} nullt auch den Rest der 32-Byte-Bridge-
    // Region: ohne das haenge die CRC32 von nicht initialisiertem Stack ab und
    // der Block waere beim naechsten Lesen ungueltig.
    VehicleSetupBlob blob{};
    blob.vwas         = wheelAngleFusionPtr->getConfig();
    blob.centerCounts = wheelAngleFusionPtr->getCenterCounts();

    const uint16_t bridgeLen =
        configManager.serializeBridgePayload(blob.bridge, VEHICLE_SETUP_BRIDGE_BYTES);
    if (bridgeLen == 0) {
        // Puffer zu klein - dann NICHT halb schreiben. Ein Block mit
        // abgeschnittener Bridge waere beim naechsten Boot ungueltig, und der
        // Betreiber haette die Geometrie ohne sichtbaren Grund verloren.
        LOG_WARNING(EventSource::AUTOSTEER,
                    "Fahrzeug-Setup NICHT gespeichert: Bridge-Nutzlast passt nicht in %u Byte",
                    (unsigned)VEHICLE_SETUP_BRIDGE_BYTES);
        return;
    }

    const uint16_t payloadLen = (uint16_t)sizeof(VehicleSetupBlob);
    const uint32_t crc        = vehicleSetupCrc32(&blob, payloadLen);

    uint16_t addr = VEHICLE_SETUP_ADDR;
    const uint16_t magic = (uint16_t)VEHICLE_SETUP_MAGIC;
    const uint16_t ver   = (uint16_t)VEHICLE_SETUP_LAYOUT_VER;
    const uint16_t rsv   = 0;
    EEPROM.put(addr, magic);      addr += sizeof(magic);
    EEPROM.put(addr, ver);        addr += sizeof(ver);
    EEPROM.put(addr, payloadLen); addr += sizeof(payloadLen);
    EEPROM.put(addr, rsv);        addr += sizeof(rsv);
    EEPROM.put(addr, crc);        addr += sizeof(crc);
    EEPROM.put(addr, blob);

    LOG_INFO(EventSource::AUTOSTEER,
             "Fahrzeug-Setup gespeichert (magic=0x%04X ver=%u, %u Byte Nutzlast, "
             "crc=0x%08lX, bridge=%u Byte, centerCounts=%ld)",
             (unsigned)magic, (unsigned)ver, (unsigned)payloadLen,
             (unsigned long)crc, (unsigned)bridgeLen,
             (long)blob.centerCounts);
}

void AutosteerProcessor::loadVehicleSetup() {
    if (!wheelAngleFusionPtr) return;

    uint16_t addr = VEHICLE_SETUP_ADDR;
    uint16_t magic = 0, ver = 0, payloadLen = 0, rsv = 0;
    uint32_t crc = 0;
    EEPROM.get(addr, magic);      addr += sizeof(magic);
    EEPROM.get(addr, ver);        addr += sizeof(ver);
    EEPROM.get(addr, payloadLen); addr += sizeof(payloadLen);
    EEPROM.get(addr, rsv);        addr += sizeof(rsv);
    EEPROM.get(addr, crc);        addr += sizeof(crc);

    // ERST den Kopf pruefen, DANN die Nutzlast lesen. Bei einem falschen
    // payloadLen wuerde EEPROM.get(addr, blob) immer sizeof(blob) Bytes lesen -
    // also ueber den Block hinaus in fremde Bereiche.
    const char* reject = nullptr;
    VehicleSetupBlob blob{};
    if (magic != (uint16_t)VEHICLE_SETUP_MAGIC) {
        reject = "kein Block (leer oder altes EEPROM)";
    } else if (ver != (uint16_t)VEHICLE_SETUP_LAYOUT_VER) {
        reject = "Layout-Version";
    } else if (payloadLen != (uint16_t)sizeof(VehicleSetupBlob)) {
        reject = "Nutzlast-Laenge";
    } else {
        EEPROM.get(addr, blob);
        if (vehicleSetupCrc32(&blob, payloadLen) != crc) reject = "crc32";
    }

    if (reject != nullptr) {
        // Die Defaults stehen in den Strukturen selbst (WheelAngleFusion::Config,
        // HeadingBridge::Config) und sind die Maschinenwerte. Sie werden SOFORT
        // geschrieben: sonst wiederholt sich die Warnung bei jedem Start, und ein
        // Betreiber lernt, sie zu ignorieren - genau das darf eine Warnung nicht
        // erreichen.
        LOG_WARNING(EventSource::AUTOSTEER,
                    "Fahrzeug-Setup verworfen (%s: magic=0x%04X ver=%u len=%u crc=0x%08lX) "
                    "- Defaults werden geschrieben. Danach vehicle.xml importieren.",
                    reject, (unsigned)magic, (unsigned)ver,
                    (unsigned)payloadLen, (unsigned long)crc);
        saveVehicleSetup();
        return;
    }

    wheelAngleFusionPtr->setConfig(blob.vwas);
    wheelAngleFusionPtr->setCenterCounts(blob.centerCounts);

    if (!configManager.deserializeBridgePayload(blob.bridge,
                                                (uint16_t)sizeof(BridgePayload))) {
        LOG_WARNING(EventSource::AUTOSTEER,
                    "Fahrzeug-Setup: Bridge-Nutzlast nicht uebernommen (%u Byte erwartet) "
                    "- Bridge bleibt auf den Defaults",
                    (unsigned)sizeof(BridgePayload));
    }

    const WheelAngleFusion::Config& cfg = wheelAngleFusionPtr->getConfig();
    LOG_INFO(EventSource::AUTOSTEER,
             "Fahrzeug-Setup geladen: wheelbase=%.2f type=%d cpd=%.1f maxAngle=%.1f "
             "sideHill=%.3f centerCounts=%ld",
             cfg.wheelbase, (int)cfg.vehicleType, cfg.countsPerDegree,
             cfg.maxSteeringAngle, cfg.sideHillGainDegPerDeg,
             (long)blob.centerCounts);
}

AutosteerProcessor::VehicleSetupState AutosteerProcessor::getVehicleSetupState() const {
    VehicleSetupState st{};
    st.expectedLen = (uint16_t)sizeof(VehicleSetupBlob);

    uint16_t addr = VEHICLE_SETUP_ADDR;
    uint16_t rsv = 0;
    uint32_t crc = 0;
    EEPROM.get(addr, st.magic);      addr += sizeof(st.magic);
    EEPROM.get(addr, st.layoutVer);  addr += sizeof(st.layoutVer);
    EEPROM.get(addr, st.payloadLen); addr += sizeof(st.payloadLen);
    EEPROM.get(addr, rsv);           addr += sizeof(rsv);
    EEPROM.get(addr, crc);           addr += sizeof(crc);

    // Die Nutzlast nur bei passendem Kopf lesen: sonst wuerde hier ueber den
    // Block hinaus gelesen (dieselbe Begruendung wie in loadVehicleSetup - nur
    // ist der Schaden hier kleiner, weil nichts angewendet wird, aber ein
    // Lesen in fremde Bereiche bleibt ein Lesen in fremde Bereiche).
    st.crcOk = false;
    if (st.magic == (uint16_t)VEHICLE_SETUP_MAGIC && st.payloadLen == st.expectedLen) {
        VehicleSetupBlob blob{};
        EEPROM.get(addr, blob);
        st.crcOk = (vehicleSetupCrc32(&blob, st.payloadLen) == crc);
    }
    st.valid = st.crcOk && (st.layoutVer == (uint16_t)VEHICLE_SETUP_LAYOUT_VER);
    return st;
}
