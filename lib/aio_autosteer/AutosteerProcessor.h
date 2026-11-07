// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef AUTOSTEER_PROCESSOR_H
#define AUTOSTEER_PROCESSOR_H

#include <Arduino.h>
#include "aog_hwmsg.h"        // PGN 221: Hardware-Meldungen nach AgOpenGPS (mehrsprachig)
#include "aog_hwmsg_store.h"  // LOCAL-Texte im EEPROM (die dritte Sprache)
// External pointers
class ADProcessor;
extern ADProcessor adProcessor;

class KickoutMonitor;

// PGN data is parsed directly to ConfigManager
// No intermediate structs needed

class AutosteerProcessor {
private:
    static AutosteerProcessor* instance;

    // Private constructor for singleton
    AutosteerProcessor();
    
    // Configuration storage
    
    // State tracking
    bool autosteerEnabled = false;
    float targetAngle = 0.0f;
    uint32_t lastPGN254Time = 0;
    // THE wheel angle of this firmware. Selected ONCE per cycle in selectSteerAngle() and
    // used by every consumer: the PGN 253 towards AgOpenGPS (via actualAngle), the
    // steering control, the HeadingMonitor record and the HeadingBridge model. See the
    // note at the definition (operator, 2026-09-28: "es ist die Aufgabe der firmware den
    // lenkwinkel zu liefern, egal woher" - PAOGI/PANDA never carry a wheel angle, only
    // the steer PGN does).
    float   steerAngleOutQuality = 100.0f;  // 0...100: 100 for every physical WAS
    
    // PGN 254 data
    float vehicleSpeed = 0.0f;      // km/h
    bool guidanceActive = false;     // Guidance line active
    int8_t crossTrackError = 0;      // cm off line
    uint16_t machineSections = 0;    // 16 section states
    
    // Button state tracking
    bool physicalButtonState = false;     // Current physical button state
    bool lastPhysicalButtonState = false; // Previous physical button state
    uint32_t lastButtonDebounceTime = 0;  // Debounce timer
    static constexpr uint32_t DEBOUNCE_DELAY = 50; // 50ms debounce
    
    // Autosteer loop timing - Now handled by SimpleScheduler at 100Hz
    
    // Steer state management
    uint8_t steerState = 1;              // 0 = steering active, 1 = steering inactive
    bool prevGuidanceStatus = false;     // Previous guidance status from AgOpenGPS
    bool guidanceStatusChanged = false;  // Flag for guidance status change
    
    // Motor control
    float currentAngle = 0.0f;           // THE wheel angle of this cycle (selectSteerAngle)
    float actualAngle = 0.0f;            // published angle (Ackermann applied in selectSteerAngle)
    int16_t motorPWM = 0;                // Current motor PWM command (-255 to +255)
    
    // Watchdog
    uint32_t lastCommandTime = 0;        // Last time we received PGN 254
    static constexpr uint32_t WATCHDOG_TIMEOUT = 2000; // 2 seconds

    // Steering speed floor (operator requirement 2026-09-21): "the motor must be
    // free at standstill = autosteer off; we should steer from 120 m/h upwards,
    // below that not." Deliberately NOT taken from the PGN 254 speed byte: AOG
    // sends (int)(avgSpeed_kmh * 10.0), i.e. 100 m/h per LSB and TRUNCATED, so
    // 120 m/h arrives as byte 1 and is read back as 0.1 km/h = 100 m/h - below the
    // threshold. The floor is simply not representable in that byte, which is why
    // the gate runs on the GPS speed plus the geometric standstill test.
    static constexpr float    STEER_MIN_SPEED_MPS     = 0.0333f; // 120 m/h
    static constexpr uint32_t SPEED_OPEN_DEBOUNCE_MS  = 500;     // rising edge only
    static constexpr uint32_t SPEED_GRACE_MS          = 2000;    // falling edge
    
    // Kickout
    uint32_t kickoutTime = 0;            // Time of last kickout
    static constexpr uint32_t KICKOUT_COOLDOWN_MS = 2000; // 2 second cooldown
    KickoutMonitor* kickoutMonitor = nullptr;
    bool lastKickoutStateForTelemetry = false;
    uint32_t pwmFreezeStartTime = 0;
    uint8_t frozenPwmDisplay = 0;
    static constexpr uint32_t POST_KICKOUT_TELEMETRY_MS = 3000;
    
    // Soft-start motor control
    enum class MotorState {
        DISABLED,
        SOFT_START,
        SOFT_ACCEL,
        NORMAL_CONTROL
    };

    MotorState motorState = MotorState::DISABLED;
    uint32_t softStartBeginTime = 0;
    float softStartRampValue = 0.0f;

    // Soft-start configuration constants
    static constexpr uint8_t DIRECTION_CHANGE_THRESHOLD = 20;  // PWM threshold for direction change detection
    static constexpr float ACCEL_THRESHOLD_RATIO = 0.33f;      // 1/3 of PWM range for acceleration detection

    // Ramp length by speed (operator requirement 2026-11-03): the ramp acts at
    // motor start, on a direction change and on a hard acceleration step. At
    // creep speed the GPS/heading noise is larger than the vehicle motion, so the
    // motor ends up following noise. Therefore the run time is unchanged at and
    // above RAMP_SLOW_KMH, rises linearly down to RAMP_FULL_KMH and is capped at
    // RAMP_MAX_FACTOR below that.
    static constexpr float    RAMP_SLOW_KMH    = 5.0f;   // at/above: unchanged
    static constexpr float    RAMP_FULL_KMH    = 3.0f;   // at/below: full factor
    static constexpr float    RAMP_MAX_FACTOR  = 2.0f;   // cap
    static constexpr uint16_t RAMP_MAX_MS      = 2000;   // hard ceiling on the effective ramp

    // Soft-start parameters (configurable via Web UI)
    uint16_t softStartDurationMs = 500;     // Duration of soft-start ramp (400-1000ms range)
    uint16_t softAccelDurationMs = 250;     // Duration of soft-acceleration ramp (150-500ms range)
    uint16_t rampDurationActiveMs = 500;   // Frozen at ramp entry: base * speed factor
    bool useSineRamp = false;                // Use sine curve (true) or linear ramp (false)
    int16_t lastPwmDrive = 0;              // Last raw pwmDrive before ramp/inversion, for direction tracking
    float filteredCommandPwm = 0.0f;       // Global filtered PWM command applied to all motor drivers
    
    // Deferred disarm for AOG OSB handshake
    // When valve/motor not ready, we briefly arm so AOG sees the state change,
    // then disarm after 200ms so AOG can properly toggle the OSB off.
    bool pendingDisarm = false;
    uint32_t pendingDisarmTime = 0;
    static constexpr uint32_t DISARM_HANDSHAKE_MS = 200;

    // Link state tracking
    bool linkWasDown = false;               // Track if link was down
    
    // Motor config change tracking
    uint8_t previousMotorConfig = 0xFF;     // Previous motor config byte
    int8_t previousCytronDriver = -1;       // Previous Cytron bit state
    bool motorConfigInitialized = false;    // Track if we've initialized from EEPROM

    // --- Zero-point ownership: "whoever acted last wins" --------------------
    //
    // AgOpenGPS re-sends PGN 252 with its stored wasOffset continuously, so the
    // mere PRESENCE of a value says nothing about whether the operator wants it
    // applied. What is meaningful is a CHANGE: that is the Zero-WAS button, the
    // Smart-WAS apply button or the offset slider being moved.
    //
    // And "apply it" is only meaningful from a known starting point, because
    // wasOffset is expressed in ENCODER COUNTS - a value on the same counter that
    // restarts at every Keya power cycle (see WheelAngleFusion::adoptBootZero()).
    // A wasOffset that survived in EEPROM across a power cycle therefore cannot be
    // valid, and applying it together with a freshly acquired mechanical zero is
    // exactly the double correction seen in the 15-28 logs: AOG had zeroed against
    // the missing centre (ctr=0) at wasOffset=-206 counts (-11.4 deg), then a
    // mechanical centre (-281) appeared on top of it and the two added up to
    // -54.6 deg with the wheels at roughly 15 deg right.
    //
    // The rule that removes that failure mode, in both directions:
    //   * a mechanical zero change (boot zero, GND button, centre capture,
    //     endstop calibration) invalidates AOG's offset until AOG acts again;
    //   * an AOG offset change claims the zero for AOG.
    // Exactly one owner at any time - the same anti-double-integrator argument
    // that keeps the firmware's own drift compensator disabled.
    bool    vwasAogZeroActive = false;   // AOG's wasOffset is currently applied
    bool    vwasAogZeroBaselineSeen = false; // first receipt of this session seen
    int16_t vwasAogZeroBaseline = 0;     // value from that first receipt
    uint32_t vwasAogZeroLastChangeMs = 0;

    // --- Nullpunkt: keine Invalidation mehr noetig --------------------------
    // vwasInvalidateAogZero() ist 2026-11-02 entfallen. Sie existierte, um
    // AgOpenGPS' wasOffset zu verwerfen, wenn sich der mechanische Nullpunkt
    // MITTEN in der Sitzung verschiebt (GND-Taster, Zentrum setzen,
    // Anschlag-Kalibrierung). Alle drei sind entfernt; die einzige verbliebene
    // Verschiebung ist adoptBootZero() beim Start, und die liegt VOR dem ersten
    // PGN-252-Empfang: dort merkt sich die Baseline-Logik (process(),
    // "if (!vwasAogZeroBaselineSeen)") AOGs gespeicherten Wert nur, statt ihn
    // anzuwenden. vwasAogZeroActive startet false, es gibt also nichts zu
    // verwerfen.

    // 2026-11-02 entfernt (Betreiber): der GND-Taster am analogen WAS-Eingang
    // ("Geradeausstellung aus der Kabine") und die zugehoerige Kalibrier-Taste
    // vwasSetCenterNow(). Damit gibt es im Betrieb keinen Weg mehr, den
    // Nullpunkt von Hand zu setzen: er kommt aus adoptBootZero() beim Start oder
    // aus AgOpenGPS' eigenem Zero-WAS (PGN 252 wasOffset), das unveraendert
    // funktioniert. Auch die dreistufige Anschlag-Kalibrierung ist weg -
    // countsPerDegree liefert AgOpenGPS.

    // --- PGN 221: Hardware-Meldungen nach AgOpenGPS (2026-11-06g) --------------------
    // Eigenes Zeitfenster, damit die Meldungen unabhaengig vom PGN-253-Takt bewertet werden.
    AogHwMsg  hardwareMsg;
    uint32_t  hwMsgLastMs  = 0;
    uint32_t  hwWeakT0     = 0;
    uint32_t  hwNoCourseT0 = 0;
    void updateHardwareMessages();

public:
    // Singleton access
    static AutosteerProcessor* getInstance();
    
    // Initialization
    bool init();
    void process();
    void initializeFusion();  // Initialize sensor fusion separately

    // --- PGN 221: Hardware-Meldungen nach AgOpenGPS (2026-11-06g) ---------------------
    // Erscheinen in AOG als eingeblendetes Label (Config -> Display -> "Hardware Messages"
    // muss dort EIN sein). Sprache passt zu WebLanguage (ENGLISH=0, GERMAN=1).
    void setHardwareMessageLanguage(AogMsgLang lang) { hardwareMsg.setLang(lang); }
    // Freitext (z. B. Uhrzeit); Anzeigedauer/Farbe/Prioritaet/Abstand gibt der Aufrufer vor.
    bool sendHardwareMessage(const char* text, uint8_t seconds, uint8_t colour,
                             uint8_t prio, uint32_t minGapMs);
    // Katalogmeldung in der eingestellten Sprache (Texte siehe aog_hwmsg.h).
    bool sendHardwareMessage(AogMsgId id);
    
    // PGN handlers
    void handleBroadcastPGN(uint8_t pgn, const uint8_t* data, size_t len);
    void handleSteerConfig(uint8_t pgn, const uint8_t* data, size_t len);
    void handleSteerSettings(uint8_t pgn, const uint8_t* data, size_t len);
    void handleSteerData(uint8_t pgn, const uint8_t* data, size_t len);
    
    // Send replies
    void sendHelloReply();
    void sendScanReply(bool extended);
    
    // Send PGN 253 status to AgOpenGPS
    void sendPGN253();
    
    // Motor control
    void updateMotorControl();
    // Selects THE wheel angle of this firmware - one source of truth per cycle. Called at
    // the top of process(); every consumer (PGN 253 via actualAngle, steering control,
    // HeadingMonitor, HeadingBridge) uses currentAngle / steerAngleOutQuality.
    void selectSteerAngle();
    void emergencyStop();
    bool shouldSteerBeActive() const;

    // --- VWAS config persistence ------------------------------------------------
    // 2026-11-02: beide schreiben/lesen den GEMEINSAMEN Fahrzeug-Setup-Block
    // (EEPROM 1500-1755, VehicleSetupBlob = VWAS-Config + centerCounts + Bridge).
    void saveVehicleSetup();
    void loadVehicleSetup();

    /// Zustand des Fahrzeug-Setup-Blocks im EEPROM - fuer die Diagnose im Web-IO.
    ///
    /// Ohne serielle Konsole gab es bisher keine Moeglichkeit zu sehen, ob der
    /// Block gueltig ist, verworfen wurde oder nie geschrieben wurde. Genau das
    /// ist die Frage direkt nach einem Update, und ein Log, das man nicht
    /// mitgeschnitten hat, beantwortet sie nicht.
    ///
    /// Erwartete Werte nach erfolgreichem Schreiben:
    ///   magic = 42257 (0xA511), layoutVer = 1, payloadLen = 96, crcOk = true.
    struct VehicleSetupState {
        uint16_t magic;       ///< Rohwert; 0xFFFF = leer, 0xA511 = unser Block
        uint16_t layoutVer;   ///< VEHICLE_SETUP_LAYOUT_VER
        uint16_t payloadLen;  ///< gespeicherte Nutzlast-Laenge
        uint16_t expectedLen; ///< sizeof(VehicleSetupBlob) dieser Firmware
        bool crcOk;           ///< Pruefsumme passt zur Nutzlast
        bool valid;           ///< Kopf passt UND crcOk (was loadVehicleSetup akzeptiert)
    };
    VehicleSetupState getVehicleSetupState() const;

    /// Push AgOpenGPS's PGN 252 steering scale and zero offset into the VWAS.
    ///
    /// Plan 1: AOG owns countsPerDegree and wasOffset for the VWAS exactly as it
    /// does for an analog WAS, so the fused angle is computed as
    /// (encoderCounts - centre + wasOffset) / countsPerDegree. Called from
    /// process() (change-gated inside the fusion) so the fusion can never run on
    /// a stale value no matter which path updated ConfigManager.
    void syncVwasSteerScaling();
    
    // Static callback wrapper for PGN registration
    static void handlePGNStatic(uint8_t pgn, const uint8_t* data, size_t len);
    
    
    // Public getters for state
    bool isEnabled() const { return autosteerEnabled; }
    float getTargetAngle() const { return targetAngle; }

    // Public getter for PGN254 vehicle speed 
    float getVehicleSpeed() const { return vehicleSpeed; }

    // Soft-start configuration
    uint16_t getSoftStartDuration() const { return softStartDurationMs; }
    void setSoftStartDuration(uint16_t durationMs) {
        softStartDurationMs = constrain(durationMs, 0, 1000); // Max 1000ms
    }

    uint16_t getSoftAccelDuration() const { return softAccelDurationMs; }
    void setSoftAccelDuration(uint16_t durationMs) {
        softAccelDurationMs = constrain(durationMs, 0, 500); // Max 500ms
    }

    bool getUseSineRamp() const { return useSineRamp; }
    void setUseSineRamp(bool useSine) { useSineRamp = useSine; }
};

// Global instance
extern AutosteerProcessor autosteerProcessor;

#endif // AUTOSTEER_PROCESSOR_H