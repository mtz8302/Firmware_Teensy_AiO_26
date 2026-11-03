// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// Firmware_Teensy_AiO-NG-v6 is copyright 2025 by the AOG Group
// GNSSProcessor - Pure NMEA parsing to structured data

#ifndef GNSS_PROCESSOR_H
#define GNSS_PROCESSOR_H

#include "Arduino.h"
#include <stdint.h>
#include "PGNProcessor.h"
#include "EventLogger.h"

// PGN Constants for GPS module
constexpr uint8_t GPS_SOURCE_ID = 0x78;     // 120 decimal - GPS source address (from PGN.md GPS Reply)
constexpr uint8_t GPS_PGN_DATA = 0xD6;      // 214 decimal - GPS data PGN
constexpr uint8_t GPS_HELLO_REPLY = 0x78;   // 120 decimal - GPS hello reply

// Callback type for position message notification
typedef void (*PositionCallback)();

// Forward declaration
class UBX_Parser;

class GNSSProcessor
{
public:
    // Clean GPS data structure
    struct GNSSData
    {
        // Position data
        double latitude;  // decimal degrees
        double longitude; // decimal degrees
        float altitude;   // meters
        uint32_t fixTime; // HHMMSS as integer
        float fixTimeFractional; // Fractional seconds (0.0-0.999)
        
        // NMEA format cached coordinates (to avoid repeated conversions)
        double latitudeNMEA;  // DDMM.MMMM format
        double longitudeNMEA; // DDDMM.MMMM format
        char latDir;          // 'N' or 'S'
        char lonDir;          // 'E' or 'W'
        
        // GPS time data (for UTC conversion)
        uint16_t gpsWeek;    // GPS week number
        float gpsSeconds;    // Seconds of week

        // Quality indicators
        uint8_t fixQuality; // 0=invalid, 1=GPS, 2=DGPS, 4=RTK, 5=Float
        uint8_t numSatellites;
        float hdop;
        uint16_t ageDGPS; // seconds since last DGPS update

        // Velocity data
        float speedKnots;
        float headingTrue; // degrees

        // Dual GPS data (from HPR messages)
        float dualHeading;      // degrees
        float dualRoll;         // degrees
        uint8_t headingQuality; // dual GPS solution quality
        // Baseline length the RECEIVER reports for the master->slave vector.
        //
        // This is the receiver-agnostic attitude health indicator the u-blox
        // F9P integration manual recommends for RELPOSNED: compare the length of
        // the baseline it solved for with the length that was physically
        // measured. A degraded solution (multipath, disturbed antenna, blocked
        // sky view) moves the solved length away from the mechanical one, and
        // that deviation is visible BEFORE the heading degrades - unlike QF,
        // which stays at 4/5 through the 1.8...2.9 % garbage frames measured on
        // the 2026-09-21 test drives.
        //
        // Sources:
        //   * F9P / RELPOSNED  -> UBXParser ubxData.baseRelL (set in
        //     parseNAVRELPOSNED handling, already parsed today),
        //   * Unicore UM98x    -> NOT available in $GNHPR (the sentence has no
        //     length field); it needs UNIHEADINGA (or equivalent) to be enabled
        //     and parsed. Stays 0 = "unknown" until that exists, and 0 disables
        //     the check in HeadingBridge on purpose.
        float baselineLengthM;
        // ---- UNIHEADINGA (Unicore N4 manual, table 7-169), added 2026-10-01 ------
        // The receiver's OWN statement about the baseline solution: length, heading,
        // pitch and the standard deviations of both. Logged only (not used in the
        // fusion yet) - the scale of hdgstddev has to be compared with the observed
        // noise on a drive first. `length` is what fills baselineLengthM on a Unicore
        // receiver, whose $GNHPR has no length field (see the comment above).
        float uniLengthM = 0.0f;
        float uniHeadingDeg = 0.0f;
        float uniPitchDeg = 0.0f;        // baseline elevation = body ROLL (crosswise mount)
        float uniHdgStdDevDeg = 0.0f;
        float uniPtchStdDevDeg = 0.0f;
        bool  uniSolValid = false;       // false while the receiver says INSUFFICIENT_OBS
        
        // INS data (from INSPVAA/INSPVAXA messages)
        float insPitch;         // degrees
        float insRoll;          // degrees  
        float insHeading;       // degrees (azimuth)
        float northVelocity;    // m/s
        float eastVelocity;     // m/s
        float upVelocity;       // m/s
        uint32_t insStatus;     // INS status word
        uint8_t posType;        // Position type (for INS quality)
        uint8_t insAlignmentStatus; // INS alignment status (0=inactive, 3=good, 7=aligning)
        
        // Extended INS data (from INSPVAXA)
        float posStdDevLat;     // Position std dev latitude (m)
        float posStdDevLon;     // Position std dev longitude (m)
        float posStdDevAlt;     // Position std dev altitude (m)
        float velStdDevNorth;   // Velocity std dev north (m/s)
        float velStdDevEast;    // Velocity std dev east (m/s)
        float velStdDevUp;      // Velocity std dev up (m/s)
        uint16_t extSolStatus;  // Extended solution status
        uint32_t timeSinceUpdate; // Time since last ZUPT or position update (seconds)

        // Status flags
        uint32_t lastUpdateTime;
        bool isValid;           // Deprecated - use hasFix instead
        bool hasPosition;       // Has lat/lon data with good fix
        bool hasVelocity;
        bool hasDualHeading;
        bool hasINS;            // Has INS data from INSPVAA/INSPVAXA
        
        // Message tracking (bit mask)
        // Bit 0: GGA, Bit 1: VTG, Bit 2: GNS
        // Bit 3: RELPOSNED, Bit 4: PVT
        // Bit 5: HPR, Bit 6: KSXT
        // Bit 7: INSPVA/INSPVAXA
        uint8_t messageTypeMask;
    };

    // UDP passthrough control
    void setUDPPassthrough(bool enabled) { 
        udpPassthroughEnabled = enabled; 
        LOG_DEBUG(EventSource::GNSS, "UDP Passthrough %s", enabled ? "ENABLED" : "DISABLED");
    }
    bool isUDPPassthroughEnabled() const { return udpPassthroughEnabled; }

    // --- HPR heading health ---
    // Why these exist: "did an HPR sentence arrive at all?" CANNOT be answered
    // from dualHeading/headingQuality. A receiver that keeps sending the
    // sentence with a zeroed heading field produces exactly the same downstream
    // state as a healthy one, and a receiver that stops sending entirely leaves
    // the last values frozen and looking valid. Counting sentences is the only
    // way to distinguish a GAP from a BAD VALUE - and those two need completely
    // different handling. See headingQualityMask in WheelAngleFusion.h.
    uint32_t getLastHprMillis() const { return lastHprMillis; }
    uint32_t getHprCount() const { return hprCount; }
    const char* getLastHprRaw() const { return lastHprRaw; }
    // Age of the newest HPR sentence; UINT32_MAX until the first one arrives.
    uint32_t getHprAgeMs() const {
        return (hprCount > 0) ? (millis() - lastHprMillis) : 0xFFFFFFFFu;
    }

    // Same instrumentation for the Unicore UNIHEADINGA log (see parseUniHeadingZeroCopy).
    uint32_t getUniCount() const { return uniCount; }
    uint32_t getUniAgeMs() const {
        return (uniCount > 0) ? (millis() - lastUniMillis) : 0xFFFFFFFFu;
    }
    // Newest raw UNIHEADINGA line, incl. the leading '#' and the trailing "*crc".
    // Used by HeadingMonitor exactly like the raw HPR sentence: the only place that
    // shows whether the receiver went silent or kept talking with INSUFFICIENT_OBS.
    const char* getLastUniRaw() const { return lastUniRaw; }

    // --- VTG (course over ground) arrival ---
    // Same reasoning as the HPR counters above, for the OTHER sensor that has to
    // carry the heading when the attitude goes: the bridge must know whether the
    // course over ground it is about to trust is a live measurement or a value
    // that has been sitting in gpsData since the last VTG. The 2026-09-21 logs
    // show VTG staying valid across every HPR loss (COG within 0.8 deg mean /
    // 1.9 deg p90 of the position derived direction at 10 m/s), but only if the
    // sentence actually keeps arriving.
    uint32_t getLastVtgMillis() const { return lastVtgMillis; }
    uint32_t getVtgCount() const { return vtgCount; }
    uint32_t getVtgAgeMs() const {
        return (vtgCount > 0) ? (millis() - lastVtgMillis) : 0xFFFFFFFFu;
    }

    // --- Position fix arrival -------------------------------------------------
    // A THIRD counter, and the reason it is not enough to use gpsData.lastUpdateTime
    // (found 2026-09-23): several sentences carry the SAME position - GGA and GNS
    // both set lastUpdateTime to millis(), so that field moves 2...4 times per
    // satellite epoch and per definition cannot mean "a new fix arrived". Measured
    // consequence: HeadingBridge::updatePositionWindow() consumed the stamp first
    // and updateProjection() never saw a new fix - the projection resynchronised
    // once per drive instead of 10x/s (one interval was 248 m long, giving an 86 m
    // cross-track residual).
    // This counter is deduplicated over the GPS time of day (gpsData.fixTime +
    // fixTimeFractional), which is the same in every sentence of one epoch, so
    //   * one satellite epoch = exactly one count, no matter how many sentences,
    //   * it is independent of the receive jitter that sits on millis(),
    //   * any number of consumers can read it WITHOUT consuming it.
    uint32_t getPositionCount() const { return positionCount; }
    uint32_t getPositionAgeMs() const {
        return (positionCount > 0) ? (millis() - lastPositionMillis) : 0xFFFFFFFFu;
    }
    // Called by every parser that carries a position, right after it updated
    // gpsData. Uses gpsData.fixTime, so no argument is needed.
    void notePositionFix();

private:
    // NMEA parsing state machine
    enum ParseState
    {
        WAIT_START,
        READ_DATA,
        READ_CHECKSUM
    };

    // Parse buffer and state
    char parseBuffer[300];  // Increased for INSPVAXA messages
    uint16_t bufferIndex;  // Changed from uint8_t to support messages > 255 bytes
    ParseState state;
    uint8_t calculatedChecksum;
    uint8_t receivedChecksum;
    uint32_t receivedChecksum32;  // For Unicore 32-bit CRC
    uint8_t checksumIndex;
    bool isUnicoreMessage;        // Track if current message starts with #

    // Message type enum for fast detection
    enum MessageType {
        MSG_UNKNOWN = 0,
        MSG_GGA,
        MSG_GNS,
        MSG_VTG,
        MSG_RMC,
        MSG_HPR,
        MSG_KSXT,
        MSG_INSPVAA,
        MSG_INSPVAXA,
        MSG_BESTGNSSPOS,
        MSG_AVR,
        MSG_UNIHEADINGA
    };

    // Field parsing - zero-copy approach
    struct FieldRef {
        const char* start;
        uint8_t length;
    };
    FieldRef fieldRefs[35];  // References to fields in parseBuffer
    uint8_t fieldCount;
    

    // Data storage
    GNSSData gpsData;

    // Configuration
    bool enableNoiseFilter;
    
    // GGA duplicate detection
    double lastGGALatitude;
    double lastGGALongitude;
    bool enableDebug;
    
    // UBX parser for GPS2 RELPOSNED
    UBX_Parser* ubxParser;
    
    // UDP passthrough
    bool udpPassthroughEnabled;

    // --- HPR heading health instrumentation ---
    // Written by parseHPRZeroCopy() on every recognised HPR sentence. Exposed
    // through getHprAgeMs()/getHprCount()/getLastHprRaw(); see the note on those
    // getters for why arrival must be counted separately from the values.
    uint32_t lastHprMillis = 0;     // millis() of the newest HPR sentence
    uint32_t hprCount = 0;          // HPR sentences since boot
    char lastHprRaw[96] = {0};      // newest raw HPR sentence, incl. "$" and "*cs"

    // --- UNIHEADINGA arrival instrumentation (same reasoning as HPR above) ---
    uint32_t lastUniMillis = 0;
    uint32_t uniCount = 0;
    // The raw sentence is the answer to "silent receiver or unusable solution?" and
    // is only observable at this moment - so keep the newest one (see the header of
    // parseUniHeadingZeroCopy). 160 B, not the 96 the shorter HPR sentence uses: a
    // UNIHEADINGA line is ~144 characters up to the CRC32, so 96 would cut off
    // exactly the two stddev fields this instrument exists for. A still longer line
    // (more satellites in the sig masks) is truncated at the tail, which is harmless.
    char lastUniRaw[160] = {0};

    // --- VTG arrival instrumentation ---
    // Written by parseVTGZeroCopy() on every recognised VTG sentence; see the
    // getters for why the age of the COG matters to the heading bridge.
    uint32_t lastVtgMillis = 0;
    uint32_t vtgCount = 0;

    // --- Position fix arrival instrumentation ---
    // Written by notePositionFix() from every parser that carries a position; see
    // the getters for why lastUpdateTime could not be used for this.
    uint32_t lastPositionMillis = 0;
    uint32_t positionCount = 0;
    uint32_t lastFixKey = 0;        // gpsData.fixTime*1000 + ms, for deduplication

    
    // Processing control
    bool processingPaused;

    // Position callback for async notification
    PositionCallback onPositionMessage;

    // Internal parsing methods
    void resetParser();
    bool validateChecksum();
    void parseFieldsZeroCopy();  // New zero-copy field parser
    bool processMessage();

    // Message handlers
    bool parseGGAZeroCopy();
    bool parseGNSZeroCopy();
    bool parseVTGZeroCopy();
    bool parseHPRZeroCopy();
    // Unicore "#UNIHEADINGA" log (table 7-169): the receiver's own baseline
    // length/heading/pitch plus their standard deviations. See the implementation.
    bool parseUniHeadingZeroCopy();
    bool parseKSXT();
    bool parseINSPVAA();
    bool parseINSPVAXA();
    
    // UDP passthrough
    void sendCompleteNMEA();

    // Field parsing utilities
    double parseLatitude(const char *lat, const char *ns);
    double parseLongitude(const char *lon, const char *ew);
    float parseFloat(const char *str);
    uint32_t parseTime(const char *str);
    uint8_t parseFixQuality(const char *str, bool isGNS = false);
    uint8_t hexToInt(char c);
    bool isHex(char c);
    void logDebug(const char *msg);
    
    // Fast message type detection
    MessageType detectMessageType(const char* msgType);
    
    // Convert decimal degrees to NMEA format and cache
    void cacheNMEACoordinates(double lat, double lon);
    
    // Zero-copy string utilities
    float parseFloatZeroCopy(const FieldRef& field);
    double parseDoubleZeroCopy(const FieldRef& field);
    int parseIntZeroCopy(const FieldRef& field);
    bool fieldEquals(const FieldRef& field, const char* str);
    bool fieldStartsWith(const FieldRef& field, const char* prefix);
    
    // Zero-copy coordinate parsers
    double parseLatitudeZeroCopy(const FieldRef& lat, const FieldRef& ns);
    double parseLongitudeZeroCopy(const FieldRef& lon, const FieldRef& ew);

public:
    GNSSProcessor();
    ~GNSSProcessor();

    // Initialization and setup
    bool init();
    bool setup(bool enableDebug = false, bool enableNoiseFilter = true);

    // Configuration
    void setNoiseFilter(bool enable) { enableNoiseFilter = enable; }
    void setDebug(bool enable) { enableDebug = enable; }

    // Main processing - single character input
    bool processNMEAChar(char c);
    bool processUBXByte(uint8_t b);

    // Batch processing
    uint16_t processNMEAStream(const char *data, uint16_t length);

    // Data access
    const GNSSData &getData() const { return gpsData; }
    bool isValid() const { return gpsData.isValid; }  // Deprecated - use hasFix()
    bool hasGPS() const { return gpsData.messageTypeMask > 0 && (millis() - gpsData.lastUpdateTime < 5000); }
    bool hasFix() const { return gpsData.hasPosition && gpsData.fixQuality > 0; }
    bool hasPosition() const { return gpsData.hasPosition; }
    bool hasVelocity() const { return gpsData.hasVelocity; }
    bool hasDualHeading() const { return gpsData.hasDualHeading; }
    bool hasINS() const { return gpsData.hasINS; }

    // Status checking
    uint32_t getDataAge() const;
    bool isDataFresh(uint32_t maxAgeMs = 2000) const;


    // Debug output
    void printData() const;
    
    // PGN support
    // registerPGNCallbacks removed - broadcast PGNs handled automatically
    void sendGPSData();  // Send PGN 214 (0xD6) - for future use
    
    // Static callback for PGN Hello (200)
    static void handleBroadcastPGN(uint8_t pgn, const uint8_t* data, size_t len);
    
    // Processing control - for UM98x configuration
    void pauseProcessing() { processingPaused = true; }
    void resumeProcessing() { processingPaused = false; }
    bool isProcessingPaused() const { return processingPaused; }

    // Position callback registration - called immediately when position message parsed
    void setPositionCallback(PositionCallback cb) { onPositionMessage = cb; }

    // Periodic status logging (called by coordinator)
    void logPeriodicStatus();
};

// Global instance following established pattern
extern GNSSProcessor gnssProcessor;

#endif // GNSS_PROCESSOR_H
