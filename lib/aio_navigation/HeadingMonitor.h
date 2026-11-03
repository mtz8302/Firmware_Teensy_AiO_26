// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// HeadingMonitor.h - measurement of UM982 HPR heading loss (test-drive logging)
//
// THIS CLASS IS MEASUREMENT ONLY. It does not alter the navigation path, does
// not touch dualHeading/headingQuality, and is not consulted by NAVProcessor or
// WheelAngleFusion. The heading bridge/projection will be designed from the data
// this collects - deliberately not before it.
//
// WHY IT EXISTS
// The UM982 keeps sending GGA and VTG but stops delivering a usable HPR heading
// under poor reception (e.g. under trees). Today that is invisible on the
// firmware side:
//   * nothing ever clears GNSSData::hasDualHeading (set in 5 places, cleared in
//     none), so NAVProcessor::selectMessageType() keeps choosing PAOGI,
//   * nothing invalidates GNSSData::headingQuality, so the quality gate in
//     WheelAngleFusion keeps seeing e.g. 4 = RTK-fixed,
//   * NAVProcessor::formatPAOGIMessage() does not even call hasGPS(), unlike
//     formatPANDAMessage() - so PAOGI keeps flowing with a frozen heading even
//     if the receiver dies outright.
// Result: AOG receives a heading that never changes while the position keeps
// moving, and the VWAS builds its straight-ahead reference from it.
//
// TWO THINGS MUST BE MEASURED BEFORE ANY BRIDGE CAN BE DESIGNED
//
// 1. What the receiver actually does when the heading goes. Three cases are
//    indistinguishable downstream but need completely different detectors:
//      (a) HPR stops entirely                    -> gap detection
//      (b) HPR continues with field zeroed/QF=0  -> value detection
//      (c) HPR continues with a frozen value     -> only visible via roll/speed
//    Distinguishing these is the whole point of counting HPR ARRIVAL separately
//    from reading its values (see the note on GNSSProcessor::getHprAgeMs()).
//
// 2. How good a kinematic projection would actually be. The proposed bridge is
//    omega = v * tan(delta) / L, using the VWAS angle and GPS speed - both of
//    which keep working under a tree, unlike the heading. Its error is
//    E(T) ~= |omega| * T * eps, where eps is the relative error of the
//    EFFECTIVE wheelbase (which absorbs the VWAS scale error, tyre slip and
//    side-slope drift). eps is unknown and must be fitted from real data, so
//    every sample stores everything needed for that fit: heading truth, roll,
//    speed, course-over-ground, VWAS angle, AOG's commanded angle and the
//    position delta.
//
// OUTPUT
//   * Event lines on the shared logger (NOTICE severity - see the note in the
//     .cpp on why NOTICE and not INFO: the INFO token bucket is already
//     over-subscribed and heading events must not be starved by it).
//   * A CSV dump of the ring buffer, emitted in non-blocking chunks, on every
//     event end and on manual request from the web UI.
//   * The raw HPR sentence at each transition - that single line is what
//     answers question 1 above.

#ifndef HEADING_MONITOR_H
#define HEADING_MONITOR_H

#include "Arduino.h"
#include <stdint.h>

class HeadingMonitor
{
public:
    // 10 Hz for 32 s. A sample is 30 bytes, so this costs 9.6 KB of the RAM1 area
    // that is free for local variables (the linker reports the figure; RSS was
    // 12.5 KB before the two HeadingBridge columns were added and 480 samples were
    // kept). Shortened from 48 s to 32 s on purpose: the new columns are worth more
    // than the extra 16 s of history, and the 10 Hz raw frame log (HR,...) now
    // carries the continuous record anyway.
    static constexpr uint16_t BUF_SAMPLES = 320;
    static constexpr uint16_t SAMPLE_INTERVAL_MS = 100;   // 10 Hz, matches HPR

    // An HPR sentence is expected every 100 ms (LOG HPR ONTIME 0.1). 250 ms =
    // 2.5 missed sentences is treated as a gap: long enough not to fire on
    // ordinary jitter or a single dropped sentence, short enough to catch a
    // short dropout before the heading has drifted far.
    static constexpr uint32_t HPR_GAP_MS = 250;

    // --- UNIHEADINGA instrument (2026-10-01) ---------------------------------
    // A UNIHEADINGA older than this counts as "not arriving". The receiver is set to
    // 5 or 10 Hz, so 2000 ms means 10...20 missing sentences - a real stop, not
    // jitter.
    static constexpr uint32_t UNI_STALE_MS = 2000;
    // Heartbeat for the SHORT status line (no raw sentence). Events and state changes
    // log immediately; this only keeps a fixed record for the "nothing is wrong"
    // case. 30 s = 120 lines/h = ~9 kB/h, negligible next to the 10 Hz HR log, and it
    // cannot flood the INFO token bucket that WheelAngleFusion/HeadingMonitor share.
    static constexpr uint32_t UNI_HEARTBEAT_MS = 30000;

    // One packed sample. Everything is in integer units to keep it small (26
    // bytes); the CSV dump prints raw integers plus a scale header line.
    struct Sample
    {
        uint16_t dtMs;       // ms since the previous sample (100 = nominal)
        uint16_t hdg100;     // dual heading, deg*100 (0..36000)
        int16_t  roll100;    // HPR field 4 = vehicle roll, deg*100 (baseline athwart)
        uint8_t  qf;         // HPR field 6 quality indicator
        uint8_t  fixQ;       // GGA fix quality (position, independent of heading)
        uint8_t  sats;
        uint8_t  hdop10;     // hdop*10
        uint16_t spdCmS;     // speed over ground, cm/s
        uint16_t cog100;     // course over ground (VTG), deg*100
        int16_t  steer100;   // VWAS fused steering angle, deg*100
        int16_t  cmd100;     // AOG commanded steering angle, deg*100
        int16_t  dNcm;       // position delta north since previous sample, cm
        int16_t  dEcm;       // position delta east since previous sample, cm
        // Operating state, so that a 48 s window can actually be interpreted
        // offline. Without these, a stretch of MANUAL driving is indistinguishable
        // from AOG-driven steering: when AOG stops sending PGN 254 the commanded
        // angle retains its last value instead of dropping to zero, so cmd100
        // alone cannot tell the two apart.
        int16_t  motorPwm;   // motor output, -255..255
        uint8_t  steerState; // 0 = steering active, 1 = steering inactive
        uint8_t  guidance;   // 1 = guidance line active
        // --- HeadingBridge (fall-back module) ---------------------------------
        // The bridged body heading and the cross-track residual of its forward
        // position projection. The raw attitude above (hdg100) is what the receiver
        // said; psi100 is what the bridge PUBLISHED to AgOpenGPS, and the two are
        // supposed to differ exactly where the gates rejected a frame. ctk10 is the
        // measured heading error of the projection - the quantity that makes the
        // projection a measurement, so a dump can be used to tune it offline.
        uint16_t psi100;     // bridged body heading, deg*100 (0..35999)
        int16_t  ctk10;      // projection cross-track residual, dm (0.1 m)
    };
    static_assert(sizeof(Sample) == 30, "Sample layout changed - update the RAM note");

    void init();

    // Called from the 100 Hz autosteer loop. `fusedAngleDeg` is the VWAS angle,
    // `cmdAngleDeg` is AOG's commanded angle (AutosteerProcessor::targetAngle).
    // Internally rate-limited to SAMPLE_INTERVAL_MS.
    void update(float fusedAngleDeg, float cmdAngleDeg,
                int16_t motorPwm, uint8_t steerState, bool guidanceActive);

    // Emits queued dump lines without ever blocking. Call once per loop tick.
    void pumpDump();

    // Manual trigger (web UI button).
    void requestDump(const char* reason);

    // Status for the web UI / event lines
    bool     isEventActive() const { return inEvent; }
    uint32_t getEventCount() const { return eventCount; }
    uint32_t getLastEventDurationMs() const { return lastEventDurationMs; }
    uint32_t getLastGapMs() const { return lastGapMs; }
    uint32_t getSamplesTaken() const { return samplesTaken; }
    bool     isDumpPending() const { return dumpIdx >= 0; }

private:
    void takeSample(float fusedAngleDeg, float cmdAngleDeg,
                    int16_t motorPwm, uint8_t steerState, bool guidanceActive);
    void emitDumpLine(uint16_t logicalIndex);
    // One short status line for the UNIHEADINGA instrument, plus the raw sentence as
    // its OWN line when `withRaw` (the sentence is ~144 chars - appended to the
    // status line it would break the <=120 char rule the log viewer needs).
    void logUniState(const char* tag, bool withRaw);

    Sample   buf[BUF_SAMPLES];
    uint16_t head = 0;        // next write position
    uint16_t count = 0;       // valid samples (<= BUF_SAMPLES)

    uint32_t lastSampleMs = 0;
    uint32_t lastSampleHprCount = 0;
    uint32_t samplesTaken = 0;

    // Position tracking for the per-sample deltas
    double   lastLat = 0.0;
    double   lastLon = 0.0;
    bool     havePosition = false;

    // Event state
    bool     inEvent = false;
    uint32_t eventStartMs = 0;
    uint32_t eventStartHprCount = 0;
    uint32_t eventFramesSeen = 0;      // HPR frames during the event (0 = pure gap)
    float    eventStartHeading = 0.0f; // last heading before the loss
    uint8_t  eventStartQf = 0;
    uint32_t eventCount = 0;
    uint32_t lastEventDurationMs = 0;
    uint32_t lastGapMs = 0;

    // UNIHEADINGA logging state (see UNI_HEARTBEAT_MS / logUniState)
    uint32_t lastUniLogMs = 0;
    bool     lastUniUsable = false;
    bool     uniStateKnown = false;    // false until the first line has been logged

    // Dump state. dumpIdx < 0 means idle; the ring is written oldest-first.
    int32_t  dumpIdx = -1;
    uint16_t dumpTotal = 0;
    bool     dumpHeaderSent = false;
    uint32_t dumpStartMs = 0;
};

extern HeadingMonitor headingMonitor;

#endif // HEADING_MONITOR_H
