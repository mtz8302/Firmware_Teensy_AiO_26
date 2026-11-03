// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#include "NAVProcessor.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include "EventLogger.h"
#include "ConfigManager.h"
#include "NMEAMessageBuilder.h"
#include "HeadingBridge.h"

// External processor instances from main.cpp
extern GNSSProcessor gnssProcessor;
extern IMUProcessor imuProcessor;

// External UDP send function
extern void sendUDPbytes(uint8_t *message, int msgLen);

// Static instance
NAVProcessor* NAVProcessor::instance = nullptr;

NAVProcessor::NAVProcessor() {
    instance = this;

    // Initialize timing
    lastGPSMessageTime = 0;

    // Initialize latency monitoring
    latencyDisplayEnabled = false;
    latencySum = 0;
    latencyCount = 0;
    latencyMin = UINT32_MAX;
    latencyMax = 0;
    lastLatencyReportTime = 0;

    // Clear message buffer
    memset(messageBuffer, 0, BUFFER_SIZE);

    LOG_INFO(EventSource::GNSS, "NAVProcessor initialized");
}

NAVProcessor::~NAVProcessor() {
    instance = nullptr;
}

NAVProcessor* NAVProcessor::getInstance() {
    if (instance == nullptr) {
        instance = new NAVProcessor();
    }
    return instance;
}

void NAVProcessor::init() {
    if (instance == nullptr) {
        instance = new NAVProcessor();
    }
}

NavMessageType NAVProcessor::selectMessageType() {
    const auto& gnssData = gnssProcessor.getData();

    // For dual/INS systems, send PAOGI even without fix (for INS_ALIGNING state)
    if (gnssData.hasDualHeading || gnssData.hasINS) {
        return NavMessageType::PAOGI;
    }

    // For single GPS, require fix
    if (!gnssProcessor.hasFix()) {
        return NavMessageType::NONE;
    }

    return NavMessageType::PANDA;
}

void NAVProcessor::convertToNMEACoordinates(double decimalDegrees, bool isLongitude,
                                           double& nmeaValue, char& direction) {
    // Determine direction
    if (isLongitude) {
        direction = (decimalDegrees < 0) ? 'W' : 'E';
    } else {
        direction = (decimalDegrees < 0) ? 'S' : 'N';
    }

    // Work with absolute value
    double absDegrees = fabs(decimalDegrees);

    // Extract degrees and minutes
    int degrees = (int)absDegrees;
    double minutes = (absDegrees - degrees) * 60.0;

    // Format as DDDMM.MMMMM or DDMM.MMMMM
    nmeaValue = degrees * 100.0 + minutes;
}

uint8_t NAVProcessor::calculateNMEAChecksum(const char* sentence) {
    uint8_t checksum = 0;

    // Skip the $ and calculate XOR up to * or end
    const char* p = sentence + 1;
    while (*p && *p != '*') {
        checksum ^= *p++;
    }

    return checksum;
}

float NAVProcessor::convertGPStoUTC(uint16_t gpsWeek, float gpsSeconds) {
    // GPS epoch (January 6, 1980) to Unix epoch (January 1, 1970) offset
    const uint32_t GPS_EPOCH_OFFSET = 315964800UL;

    // Current leap seconds as of 2024
    const uint8_t LEAP_SECONDS = 18;

    // Calculate total seconds since GPS epoch
    uint32_t totalGPSSeconds = (uint32_t)gpsWeek * 7 * 24 * 60 * 60 + (uint32_t)gpsSeconds;

    // Convert to Unix time and adjust for leap seconds
    uint32_t unixTime = GPS_EPOCH_OFFSET + totalGPSSeconds - LEAP_SECONDS;

    // Extract UTC time components
    uint32_t secondsToday = unixTime % (24 * 60 * 60);
    uint8_t hours = secondsToday / 3600;
    uint8_t minutes = (secondsToday % 3600) / 60;
    uint8_t seconds = secondsToday % 60;

    // Get milliseconds from the fractional part of gpsSeconds
    float fractionalSeconds = gpsSeconds - (uint32_t)gpsSeconds;

    // Return as HHMMSS.S format
    return hours * 10000.0f + minutes * 100.0f + seconds + fractionalSeconds;
}

bool NAVProcessor::formatPANDAMessage() {

    const auto& gnssData = gnssProcessor.getData();

    // B10.48 (2026-10-18, Betreiber): 0/0 ist KEINE Position. Der Satz geht trotzdem
    // raus, aber mit LEEREN Feldern lat/N/lon/E/alt - so legt AgOpenGPS daraus keinen
    // Feld-Ursprung bei 0/0 an (der Fehler waere 5488 km, gemessen in allen " 0 "-Logs).
    // Heading/Roll/Tempo bleiben nutzbar, fixQ bleibt der des Empfaengers.
    const bool posOk = gnssProcessor.hasGPS() &&
                       (gnssData.latitude != 0.0 || gnssData.longitude != 0.0);

    // Use cached NMEA coordinates - no conversion needed
    double latNMEA = gnssData.latitudeNMEA;
    double lonNMEA = gnssData.longitudeNMEA;
    char latDir = gnssData.latDir;
    char lonDir = gnssData.lonDir;

    // Get IMU data if available
    char imuHeading[10] = "65535";  // Default "no IMU" value
    char imuRoll[10] = "0";
    char imuPitch[10] = "0";
    char imuYawRate[10] = "0";

    if (imuProcessor.hasValidData()) {
        const auto& imuData = imuProcessor.getCurrentData();
        snprintf(imuHeading, sizeof(imuHeading), "%d", (int)(imuData.heading * 10.0));
        snprintf(imuRoll, sizeof(imuRoll), "%d", (int)round(imuData.roll));
        snprintf(imuPitch, sizeof(imuPitch), "%d", (int)round(imuData.pitch));
        snprintf(imuYawRate, sizeof(imuYawRate), "%.2f", imuData.yawRate);
    }
    // NOTE: no dual-antenna fallback for the roll belongs here. selectMessageType()
    // returns PAOGI whenever hasDualHeading is true, and formatPAOGIMessage() already
    // takes its roll from gnssData.dualRoll - so this PANDA branch only runs when there
    // is NO dual heading, and a fallback reading dualRoll from here would be dead code.
    // The IMU failure on the 2026-09-21 logs therefore does NOT affect the roll that
    // AgOpenGPS receives. What it does affect is the PITCH (and yaw rate) in PAOGI:
    // with a dual antenna there is no independent pitch source, so a dead IMU leaves
    // AgOpenGPS with pitch = 0, disabling its fore/aft antenna swing correction.

    // Format time (HHMMSS.S)
    float timeFloat = gnssData.fixTime + gnssData.fixTimeFractional;

    // Build PANDA message using MessageBuilder
    NMEAMessageBuilder builder(messageBuffer);

    builder.addString("$PANDA");
    builder.addComma();
    builder.addFloat(timeFloat, 1);
    builder.addComma();
    if (posOk) { builder.addLatitude(latNMEA); }
    builder.addComma();
    if (posOk) { builder.addChar(latDir); }
    builder.addComma();
    if (posOk) { builder.addLongitude(lonNMEA); }
    builder.addComma();
    if (posOk) { builder.addChar(lonDir); }
    builder.addComma();
    builder.addInt(gnssData.fixQuality);
    builder.addComma();
    builder.addInt(gnssData.numSatellites);
    builder.addComma();
    builder.addFloat(gnssData.hdop, 1);
    builder.addComma();
    if (posOk) { builder.addFloat(gnssData.altitude, 3); }
    builder.addComma();
    builder.addFloat((float)gnssData.ageDGPS, 1);
    builder.addComma();
    builder.addFloat(gnssData.speedKnots, 3);
    builder.addComma();
    builder.addString(imuHeading);
    builder.addComma();
    builder.addString(imuRoll);
    builder.addComma();
    builder.addString(imuPitch);
    builder.addComma();
    builder.addString(imuYawRate);
    builder.addChecksum();
    builder.terminate();

    return true;
}

bool NAVProcessor::formatPAOGIMessage() {
    const auto& gnssData = gnssProcessor.getData();

    // B10.48 (2026-10-18, Betreiber): wie in formatPANDAMessage - ohne Position leere
    // Felder lat/N/lon/E/alt. posOk wird vom Positions-Fallback unten auf true gesetzt.
    bool posOk = gnssProcessor.hasGPS() &&
                 (gnssData.latitude != 0.0 || gnssData.longitude != 0.0);

    if (!gnssData.hasDualHeading) {
        return false;
    }

    // ---- Heading/roll fall-back (HeadingBridge) ----------------------------
    // The bridge works in the RECEIVER's convention (baseline azimuth = body
    // heading + mount offset), so what it returns here can replace dualHeading
    // 1:1 - AgOpenGPS adds its own dualHeadingOffset on top of it either way
    // (UDPComm.Designer.cs:85). Without this substitution a zeroed HPR sentence
    // (000.0000 with QF=0, measured on every dropout) and the 2-3 % garbage
    // attitude frames reach AgOpenGPS unchecked.
    float headingOut = gnssData.dualHeading;
    float rollOut = gnssData.dualRoll;
    {
        const HeadingBridge::Config& bcfg = headingBridge.getConfig();
        const HeadingBridge::Status& bs = headingBridge.getStatus();
        if (bcfg.enabled && bcfg.exposeToAog) {
            if (bs.state == HeadingBridge::State::TIMEOUT) {
                // Budget spent: the module no longer claims a valid heading.
                switch (bcfg.timeoutAction) {
                    case HeadingBridge::TimeoutAction::STOP_PAOGI:
                        // Send nothing at all - AgOpenGPS' own GPS watchdog takes
                        // over (it stops steering and reports "no GPS").
                        return false;
                    case HeadingBridge::TimeoutAction::LAST_VALID_QF0:
                        // Keep the position usable but mark the fix quality as
                        // invalid, which is the only field in PAOGI that tells AOG
                        // something is wrong. NOTE: AOG also gates section control
                        // on this - deliberate, it is the honest signal.
                        headingOut = bs.azimuthDeg;
                        rollOut = bs.rollDeg;
                        break;
                    case HeadingBridge::TimeoutAction::KEEP_BRIDGED:
                    default:
                        headingOut = bs.azimuthDeg;
                        rollOut = bs.rollDeg;
                        break;
                }
            } else if (!bs.haveAnyHeading) {
                // The filter has never produced a usable heading of ANY kind, so its state is
                // still the boot default (psi = 0 -> 270 deg for a +90 offset). Forward the
                // RECEIVER's own values instead of that constant.
                // Operator decision 2026-09-29: this branch used to key off "a dual-antenna
                // frame was ever trusted", which kept a single-antenna machine (F9P/UM981 +
                // IMU, GGA+VTG only) here FOREVER - publishing the receiver's heading/roll,
                // which are EMPTY without a dual antenna. haveAnyHeading is set by the first
                // accepted measurement of any source, so such a machine now gets the bridged
                // heading (and the roll, which the IMU supplies - see updateRoll()).
                headingOut = gnssData.dualHeading;
                rollOut = gnssData.dualRoll;
            } else {
                headingOut = bs.azimuthDeg;
                rollOut = bs.rollDeg;
            }
        }
        // Sentinel for the fix-quality override, applied in the builder below.
        // TWO reasons to report "not valid" to AOG (operator decisions):
        //   * TIMEOUT + LAST_VALID_QF0, and
        //   * the heading is model-only for longer than Config::drQf0AfterMs
        //     ("erst nach 2 s Modellbetrieb qf=0, vorher weiterreichen", 2026-09-28) - a
        //     heading that no sensor produced must not be claimed as a valid fix.
        paogiFixQualityOverride =
            (bcfg.enabled && bcfg.exposeToAog &&
             ((bs.state == HeadingBridge::State::TIMEOUT &&
               bcfg.timeoutAction == HeadingBridge::TimeoutAction::LAST_VALID_QF0) ||
              bs.headingModelOnly))
            ? 0 : -1;
    }

    // Use cached NMEA coordinates - no conversion needed
    double latNMEA = gnssData.latitudeNMEA;
    double lonNMEA = gnssData.longitudeNMEA;
    char latDir = gnssData.latDir;
    char lonDir = gnssData.lonDir;
    // Position fall-back (operator request 2026-09-24): while the receiver's fix is
    // unusable (bridge/trees -> physically impossible step, or fixQuality below RTK
    // fixed) the forward-projected position is published instead of it. Only the
    // POSITION is substituted - fix quality, satellite count, speed and heading stay
    // the receiver's/filter's own, so AgOpenGPS' own lost-RTK handling stays intact and
    // the module never claims a better fix than there is.
    {
        double fbLat = 0.0, fbLon = 0.0;
        if (headingBridge.getFallbackPosition(fbLat, fbLon)) {
            posOk = true;   // projizierte Position ist eine gueltige Position
            char fbLatDir = 'N', fbLonDir = 'E';
            convertToNMEACoordinates(fbLat, false, latNMEA, fbLatDir);
            convertToNMEACoordinates(fbLon, true, lonNMEA, fbLonDir);
            latDir = fbLatDir;
            lonDir = fbLonDir;
        }
    }



    // Get IMU data if available (for pitch and yaw rate)
    int16_t pitch = 0;
    float yawRate = 0.0;

    // Prefer INS pitch if available (from UM981)
    if (gnssData.hasINS) {
        pitch = (int16_t)round(gnssData.insPitch);
    } else if (imuProcessor.hasValidData()) {
        const auto& imuData = imuProcessor.getCurrentData();
        pitch = (int16_t)round(imuData.pitch);
        yawRate = imuData.yawRate;
    }

    // Use dual GPS roll (from KSXT pitch field)
    float roll = rollOut;

    // Format time - use UTC from GPS week/seconds if available
    float timeFloat;
    if (gnssData.gpsWeek > 0 && gnssData.gpsSeconds > 0) {
        timeFloat = convertGPStoUTC(gnssData.gpsWeek, gnssData.gpsSeconds);
    } else {
        timeFloat = gnssData.fixTime + gnssData.fixTimeFractional;
    }

    // Build PAOGI message using MessageBuilder
    NMEAMessageBuilder builder(messageBuffer);

    builder.addString("$PAOGI");
    builder.addComma();
    builder.addFloat(timeFloat, 1);
    builder.addComma();
    if (posOk) { builder.addLatitude(latNMEA); }
    builder.addComma();
    if (posOk) { builder.addChar(latDir); }
    builder.addComma();
    if (posOk) { builder.addLongitude(lonNMEA); }
    builder.addComma();
    if (posOk) { builder.addChar(lonDir); }
    builder.addComma();
    builder.addInt(paogiFixQualityOverride >= 0 ? paogiFixQualityOverride
                                                : (int)gnssData.fixQuality);
    builder.addComma();
    builder.addInt(gnssData.numSatellites);
    builder.addComma();
    builder.addFloat(gnssData.hdop, 1);
    builder.addComma();
    if (posOk) { builder.addFloat(gnssData.altitude, 3); }
    builder.addComma();
    builder.addFloat((float)gnssData.ageDGPS, 1);
    builder.addComma();
    // Geschwindigkeit (2026-10-14, Betreiber-Regel): ROH, solange der Empfaengerwert
    // zu Weg/Zeit passt; im Zweifel (Doppler unplausibel - gemessen 18-47 bis 2,9 m/s
    // daneben) der geglaettete Wert aus Weg/Zeit. AOG hat einen eigenen Filter, deshalb
    // bleibt der Normalfall unveraendert.
    builder.addFloat(headingBridge.getSpeedKnotsForOutput(gnssData.speedKnots), 3);
    builder.addComma();
    builder.addFloat(headingOut, 1);
    builder.addComma();
    builder.addFloat(roll, 2);
    builder.addComma();
    builder.addInt(pitch);
    builder.addComma();
    builder.addFloat(yawRate, 2);
    builder.addChecksum();
    builder.terminate();

    return true;
}

void NAVProcessor::sendMessage(const char* message) {
    // NMEA messages must end with CR+LF
    char buffer[BUFFER_SIZE + 3];
    snprintf(buffer, sizeof(buffer), "%s\r\n", message);

    // Send via UDP to AgIO
    sendUDPbytes((uint8_t*)buffer, strlen(buffer));
}

void NAVProcessor::sendImmediately() {
    // Start timing for latency measurement
    uint32_t startMicros = micros();

    // Check if UDP passthrough is enabled - if so, don't send PANDA/PAOGI
    extern ConfigManager configManager;
    if (configManager.getGPSPassThrough()) {
        return;
    }

    // Select and format appropriate message type
    NavMessageType msgType = selectMessageType();
    if (msgType == NavMessageType::NONE) {
        return;
    }

    bool success = false;

    switch (msgType) {
        case NavMessageType::PANDA:
            success = formatPANDAMessage();
            break;
        case NavMessageType::PAOGI:
            success = formatPAOGIMessage();
            break;
        default:
            break;
    }

    if (success) {
        sendMessage(messageBuffer);
        lastGPSMessageTime = millis();

        // Measure and track latency
        if (latencyDisplayEnabled) {
            uint32_t latencyMicros = micros() - startMicros;
            latencySum += latencyMicros;
            latencyCount++;
            if (latencyMicros < latencyMin) latencyMin = latencyMicros;
            if (latencyMicros > latencyMax) latencyMax = latencyMicros;

            // Report every second
            uint32_t now = millis();
            if (now - lastLatencyReportTime >= 1000) {
                if (latencyCount > 0) {
                    uint32_t avgLatency = latencySum / latencyCount;
                    Serial.printf("[GPS->UDP] %lu msgs, latency: avg=%luus, min=%luus, max=%luus\r\n",
                                  latencyCount, avgLatency, latencyMin, latencyMax);
                }
                // Reset for next period
                latencySum = 0;
                latencyCount = 0;
                latencyMin = UINT32_MAX;
                latencyMax = 0;
                lastLatencyReportTime = now;
            }
        }
    }
}

NavMessageType NAVProcessor::getCurrentMessageType() {
    return selectMessageType();
}

void NAVProcessor::printStatus() {
    LOG_INFO(EventSource::GNSS, "=== NAVProcessor Status ===");

    NavMessageType currentType = getCurrentMessageType();
    LOG_INFO(EventSource::GNSS, "Current mode: %s",
        currentType == NavMessageType::PANDA ? "PANDA (Single GPS)" :
        currentType == NavMessageType::PAOGI ? "PAOGI (Dual GPS)" : "NONE");

    if (lastGPSMessageTime > 0) {
        LOG_INFO(EventSource::GNSS, "Time since last GPS message: %lu ms",
            millis() - lastGPSMessageTime);
    }

    // Show data sources
    LOG_INFO(EventSource::GNSS, "Data sources:");
    if (gnssProcessor.isValid()) {
        const auto& gnssData = gnssProcessor.getData();
        LOG_INFO(EventSource::GNSS, "  GPS: Valid (Fix=%d, Sats=%d)",
            gnssData.fixQuality, gnssData.numSatellites);
        if (gnssData.hasDualHeading) {
            LOG_INFO(EventSource::GNSS, "  Dual GPS: Active (Quality=%d)",
                gnssData.headingQuality);
        }
    } else {
        LOG_INFO(EventSource::GNSS, "  GPS: No valid fix");
    }

    if (imuProcessor.hasValidData()) {
        LOG_INFO(EventSource::GNSS, "  IMU: %s connected", imuProcessor.getIMUTypeName());
    } else {
        LOG_INFO(EventSource::GNSS, "  IMU: Not detected");
    }
}

void NAVProcessor::toggleLatencyDisplay() {
    latencyDisplayEnabled = !latencyDisplayEnabled;

    if (latencyDisplayEnabled) {
        // Reset stats when enabling
        latencySum = 0;
        latencyCount = 0;
        latencyMin = UINT32_MAX;
        latencyMax = 0;
        lastLatencyReportTime = millis();
        Serial.println("\r\nGPS->UDP latency display ENABLED (reports every second)");
    } else {
        Serial.println("\r\nGPS->UDP latency display DISABLED");
    }
}
