// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// HeadingMonitor.cpp - see HeadingMonitor.h for why this exists and what the
// two questions are that it has to answer.

#include "HeadingMonitor.h"
#include "GNSSProcessor.h"
#include "HeadingBridge.h"
#include "EventLogger.h"
#include <math.h>

// RAM1-Entlastung 2026-10-29: der Monitor ist reine DIAGNOSTIK (Ringpuffer fuer Dumps, siehe
// HeadingMonitor.h) und belegte 9688 Byte im DTCM. RAM1 besteht aus ITCM (Code, in 32-KB-
// Bloecken) + DTCM (Variablen) und war mit -928 Byte UEBERvoll; Code zu verschieben half
// nichts, weil er im selben 32-KB-Block blieb (padding wuchs mit). DMAMEM legt den Puffer
// nach RAM2 (dort 337 KB frei). Kein Verhalten, keine Timing-Abhaengigkeit: der Ring wird
// in init()/update() nur geschrieben und beim Dump gelesen.
DMAMEM HeadingMonitor headingMonitor;

// ---------------------------------------------------------------------------
// Why NOTICE and not INFO for the event lines
// ---------------------------------------------------------------------------
// EventLogger rate-limits per severity with a token bucket (EventLogger.h:72),
// so every severity has its OWN bucket. The INFO bucket allows 10 messages per
// second and is already over-subscribed in normal operation:
//
//      5/s   WheelAngleFusion  "VWAS diag"     (LOG_INFO, 200 ms)
//      1/s   AutosteerProcessor "VWAS ctrl"    (LOG_INFO, 1000 ms)
//   up to 10/s WheelAngleFusion "VWAS encoder"  (LOG_INFO, 100 ms while turning)
//   ---------------------------------------------------------------------
//   up to 16/s against a budget of 10/s
//
// So INFO messages ARE being dropped today, and preferentially while turning -
// the exact moment the data matters most. Putting the heading events in the same
// bucket would make them lose that race too. NOTICE has a separate bucket, and
// it passes the same severity filter: NOTICE=5 <= INFO=6, so any configuration
// that shows INFO also shows NOTICE.
//
// The bulk CSV dump does not go through the logger at all - see pumpDump().
// ---------------------------------------------------------------------------

// A HEADING LOSS is the receiver saying "I am not measuring the heading", which is
// QF 0 (invalid), 6 (dead reckoning) or 7 (manual/operator-set). It is NOT the same
// as losing RTK.
//
// This used to be `qf == 4 || qf == 5`, i.e. RTK only, on the theory that a
// diagnostic should flag degradation even when the fusion is configured
// differently. That conflated two unrelated things: a machine without RTK (QF 1
// single point, 2 DGPS, 9 SBAS) has a perfectly valid dual-antenna heading, so the
// whole session would have been recorded as one continuous "heading loss" - the
// operator's own point, that pure reception quality and RTK are not the same
// question. The events that motivated this monitor were QF=0 with sats=0 and a
// zeroed heading, which the widened set still catches.
//
// Kept separate from WheelAngleFusion::headingQualityMask on purpose: the fusion's
// mask decides what may be USED, this decides what counts as a FAULT, and they are
// allowed to differ.
static inline bool monitorQfAcceptable(uint8_t qf) {
    return qf != 0 && qf != 6 && qf != 7;
}

static inline float monitorAngleDiff(float a, float b) {
    float d = a - b;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Angles are CLAMPED rather than wrapped: a value beyond +-327 deg means
// something upstream is wrong, and clamping keeps that visible in the data
// instead of hiding it behind a wrap.
static inline int16_t monitorClampAngle100(float deg) {
    float v = deg * 100.0f;
    if (v > 32767.0f)  v = 32767.0f;
    if (v < -32767.0f) v = -32767.0f;
    return (int16_t)lroundf(v);
}

// Position deltas are clamped to +-327 m between two 100 ms samples, which is
// already absurdly fast (3270 m/s) - so a clamp only ever fires on garbage.
static inline int16_t monitorClampCm(double cm) {
    if (cm > 32767.0)  cm = 32767.0;
    if (cm < -32767.0) cm = -32767.0;
    return (int16_t)lround(cm);
}

void HeadingMonitor::init() {
    head = 0;
    count = 0;
    lastSampleMs = 0;
    lastSampleHprCount = 0;
    samplesTaken = 0;
    havePosition = false;
    inEvent = false;
    eventCount = 0;
    lastEventDurationMs = 0;
    lastGapMs = 0;
    lastUniLogMs = 0;
    lastUniUsable = false;
    uniStateKnown = false;
    dumpIdx = -1;
    dumpTotal = 0;
    dumpHeaderSent = false;

    LOG_NOTICE(EventSource::GNSS,
               "HPR monitor: %u samples x %u B = %u B RAM, %u Hz, gap>%u ms",
               (unsigned)BUF_SAMPLES, (unsigned)sizeof(Sample),
               (unsigned)(BUF_SAMPLES * sizeof(Sample)),
               (unsigned)(1000 / SAMPLE_INTERVAL_MS), (unsigned)HPR_GAP_MS);
}

void HeadingMonitor::update(float fusedAngleDeg, float cmdAngleDeg,
                            int16_t motorPwm, uint8_t steerState, bool guidanceActive) {
    const auto& d = gnssProcessor.getData();
    const uint32_t now = millis();
    const uint32_t hprCount = gnssProcessor.getHprCount();
    const uint32_t hprAge = gnssProcessor.getHprAgeMs();

    // --- Event detection runs at the full caller rate (100 Hz), NOT at the
    // 10 Hz sample rate. A 250 ms gap has to be caught within 250 ms; the
    // sampler would only notice at its next tick and add up to 100 ms of blind
    // time to the very quantity being measured. ---
    const bool gap = (hprAge > HPR_GAP_MS);
    const bool badValue = (!gap && !monitorQfAcceptable(d.headingQuality));
    const char* reason = gap ? "GAP" : (badValue ? "BADQF" : nullptr);

    bool eventEdge = false;

    if (!inEvent && reason != nullptr) {
        eventEdge = true;
        inEvent = true;
        eventStartMs = now;
        eventStartHprCount = hprCount;
        eventStartHeading = d.dualHeading;
        eventStartQf = d.headingQuality;
        eventCount++;

        // This is the line the whole investigation hinges on. It carries the RAW
        // sentence, which is the only place that shows whether the receiver went
        // SILENT or kept TALKING with a zeroed/garbage field. Those two cases
        // leave identical state behind (dualHeading and headingQuality simply
        // keep their last values, since nothing ever clears them) but need
        // completely different detectors.
        LOG_NOTICE(EventSource::GNSS,
                   "HPR LOSS[%s]: age=%lums qf=%u hdg=%.4f roll=%.4f sats=%u fixQ=%u raw=\"%s\"",
                   reason, (unsigned long)hprAge, (unsigned)d.headingQuality,
                   (double)d.dualHeading, (double)d.dualRoll,
                   (unsigned)d.numSatellites, (unsigned)d.fixQuality,
                   gnssProcessor.getLastHprRaw());
    } else if (inEvent && reason == nullptr) {
        eventEdge = true;
        inEvent = false;
        lastEventDurationMs = now - eventStartMs;
        eventFramesSeen = hprCount - eventStartHprCount;
        // frames==0 means nothing arrived at all (a true gap); frames>0 means the
        // receiver kept sending while the quality stayed unusable.
        lastGapMs = (eventFramesSeen == 0) ? lastEventDurationMs : 0;

        LOG_NOTICE(EventSource::GNSS,
                   "HPR BACK: dur=%lums frames=%lu qf=%u hdg=%.4f jump=%.3f deg",
                   (unsigned long)lastEventDurationMs, (unsigned long)eventFramesSeen,
                   (unsigned)d.headingQuality, (double)d.dualHeading,
                   (double)monitorAngleDiff(d.dualHeading, eventStartHeading));

        // The recovery dump is the payload: 48 s of truth spanning the outage,
        // which is what the projection model will be fitted and validated on.
        requestDump("HPR recovery");
    }

    // --- UNIHEADINGA (2026-10-01) ---------------------------------------------
    // The receiver's own statement about the dual-antenna baseline, logged right next
    // to the HPR event so the two can be compared: if $GNHPR shows no solution while
    // UNIHEADINGA keeps reporting a length and stddevs, the loss is in that sentence,
    // not in the receiver. Emitted
    //   * on every HPR event EDGE (loss and recovery) - the moment of interest,
    //   * when the state CHANGES (usable <-> stale/unusable) - that change is itself
    //     an event, and it covers the case "INSUFFICIENT_OBS stays after the K-turn"
    //     that the 2026-10-01 investigation ran into,
    //   * otherwise every UNI_HEARTBEAT_MS as a SHORT status line (no raw sentence),
    //     so a healthy drive still leaves a record without cost.
    {
        const uint32_t uniAge = gnssProcessor.getUniAgeMs();
        const bool uniUsable = (uniAge != 0xFFFFFFFFu) && (uniAge <= UNI_STALE_MS) &&
                               d.uniSolValid;
        const bool uniEdge = (!uniStateKnown) || (uniUsable != lastUniUsable);
        if (uniEdge || eventEdge || (now - lastUniLogMs) >= UNI_HEARTBEAT_MS) {
            lastUniLogMs = now;
            lastUniUsable = uniUsable;
            uniStateKnown = true;
            // The raw sentence only when something happened. At the heartbeat it would
            // double the line length for no gain - the five values are already in the
            // status line.
            // The raw sentence only when something happened AND at least one line ever
            // arrived. At the very first tick nothing has arrived yet and a raw line
            // would print as "UNI RAW: " with an empty string - indistinguishable from
            // a truncated sentence in the log, so it is suppressed.
            logUniState(eventEdge ? "EVENT" : (uniEdge ? "CHANGE" : "HEARTBEAT"),
                        (eventEdge || uniEdge) && (gnssProcessor.getUniCount() > 0));
        }
    }

    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
        takeSample(fusedAngleDeg, cmdAngleDeg, motorPwm, steerState, guidanceActive);
    }
}

void HeadingMonitor::takeSample(float fusedAngleDeg, float cmdAngleDeg,
                                int16_t motorPwm, uint8_t steerState, bool guidanceActive) {
    const auto& d = gnssProcessor.getData();
    const uint32_t now = millis();

    Sample& s = buf[head];

    // The first sample has no predecessor, so there is no meaningful dt. Zero
    // rather than the uptime, which the naive subtraction would have produced.
    if (samplesTaken == 0) {
        s.dtMs = 0;
    } else {
        const uint32_t dt = now - lastSampleMs;
        s.dtMs = (dt > 65000u) ? 65000u : (uint16_t)dt;
    }

    // Heading is WRAPPED (it is an angle, and 359.9 -> 0.1 is a real thing).
    // A malformed sentence must not produce a negative number that then wraps
    // into the uint16 and reads as 655 deg.
    float hdg = fmodf(d.dualHeading, 360.0f);
    if (hdg < 0.0f) hdg += 360.0f;
    s.hdg100 = (uint16_t)(lroundf(hdg * 100.0f) % 36000L);

    s.roll100 = monitorClampAngle100(d.dualRoll);   // HPR field 4 = vehicle roll

    s.qf   = d.headingQuality;
    s.fixQ = d.fixQuality;                          // position quality, independent
    s.sats = (d.numSatellites > 255) ? 255 : (uint8_t)d.numSatellites;

    float hdop = d.hdop;
    if (!(hdop > 0.0f)) hdop = 0.0f;                // also catches NaN
    if (hdop > 25.5f)   hdop = 25.5f;
    s.hdop10 = (uint8_t)lroundf(hdop * 10.0f);

    // Speed: GNSSData::speedKnots really is knots - NAVProcessor relies on that,
    // because AOG multiplies PAOGI field 11 by 1.852 to get km/h. The GPS speed
    // is recorded rather than the fused one on purpose: it is the independent
    // measurement the projection model would have to work from during an outage.
    float cmS = d.speedKnots * 51.4444f;            // 1 knot = 51.4444 cm/s
    if (!(cmS > 0.0f)) cmS = 0.0f;
    if (cmS > 6000.0f) cmS = 6000.0f;               // clamp at 60 m/s
    s.spdCmS = (uint16_t)lroundf(cmS);

    float cog = fmodf(d.headingTrue, 360.0f);       // course over ground (VTG)
    if (cog < 0.0f) cog += 360.0f;
    s.cog100 = (uint16_t)(lroundf(cog * 100.0f) % 36000L);

    // The two steering angles. Their difference is exactly what the side-slope
    // question needs: AOG adds roll * sideHillDraftGain to the angle it commands,
    // so regressing (commanded - actual) against roll over this data recovers the
    // effective draft term without needing AOG's gain value at all.
    s.steer100 = monitorClampAngle100(fusedAngleDeg);
    s.cmd100   = monitorClampAngle100(cmdAngleDeg);

    // Operating state. Recorded so a 48 s window can be interpreted offline:
    // without it, a stretch of manual driving is indistinguishable from
    // AOG-driven steering, because targetAngle keeps its last value when AOG
    // stops sending PGN 254 instead of dropping to zero.
    s.motorPwm   = motorPwm;
    s.steerState = steerState;
    s.guidance   = guidanceActive ? 1 : 0;

    // HeadingBridge: the bridged heading and the projection residual. Clamped the
    // same way as the other angles; the cross-track residual is in decimetres, so
    // the int16 covers +-327 m which is already absurd for a 10 Hz interval.
    {
        const HeadingBridge::Status& bs = headingBridge.getStatus();
        float psiDeg = fmodf(bs.vehicleHeadingDeg, 360.0f);
        if (psiDeg < 0.0f) psiDeg += 360.0f;
        s.psi100 = (uint16_t)(lroundf(psiDeg * 100.0f) % 36000L);
        float ctkDm = bs.projCtkM * 10.0f;
        if (ctkDm > 32767.0f) ctkDm = 32767.0f;
        if (ctkDm < -32767.0f) ctkDm = -32767.0f;
        s.ctk10 = (int16_t)lroundf(ctkDm);
    }

    // Position delta, so the offline fit can verify whether the vehicle really
    // travelled the arc a projection would claim - and so that the "did it
    // actually move at all?" question stays answerable at creep speed.
    if (havePosition) {
        const double dLat = d.latitude  - lastLat;
        const double dLon = d.longitude - lastLon;
        const double mPerDegLat = 111320.0;
        const double mPerDegLon = 111320.0 * cos(d.latitude * 0.017453292519943295);
        s.dNcm = monitorClampCm(dLat * mPerDegLat * 100.0);
        s.dEcm = monitorClampCm(dLon * mPerDegLon * 100.0);
    } else {
        s.dNcm = 0;
        s.dEcm = 0;
    }
    lastLat = d.latitude;
    lastLon = d.longitude;
    havePosition = true;

    head = (uint16_t)((head + 1) % BUF_SAMPLES);
    if (count < BUF_SAMPLES) count++;
    lastSampleMs = now;
    samplesTaken++;
}

void HeadingMonitor::requestDump(const char* reason) {
    if (count == 0) {
        LOG_WARNING(EventSource::GNSS, "HPR buffer dump requested but empty (%s)", reason);
        return;
    }

    dumpIdx = 0;
    dumpTotal = count;
    dumpHeaderSent = false;
    dumpStartMs = millis();

    // The absolute time is reported here rather than per line, because storing a
    // 4-byte uptime in every sample would cost 1.9 KB of RAM for information
    // that reconstructs exactly: the LAST emitted line is this timestamp, and
    // each earlier one is that minus the dtMs values in between.
    LOG_NOTICE(EventSource::GNSS,
               "HPR buffer dump starting: %u lines, last sample at t=%lums (%s)",
               (unsigned)count, (unsigned long)lastSampleMs, reason);
}

// The UNIHEADINGA instrument (2026-10-01). It exists for two reasons: the baseline
// LENGTH only exists in this log (the $GNHPR sentence has no such field, so
// baselineLengthM had no source on this receiver at all), and the receiver's own
// hdgstddev/ptchstddev are its only independent statement about how good its
// attitude really is - the quantity the adaptive R should eventually be checked
// against instead of the innovation it feeds on.
//
// One line, <= ~90 chars, so it fits the same log viewer rules as the HB1/HB2 lines.
// The raw sentence goes on its OWN line (it is ~144 chars) and only when `withRaw`:
// appending it here would produce a 240 char line, which is exactly what the two
// line split in logDebugLine was introduced to avoid.
void FLASHMEM HeadingMonitor::logUniState(const char* tag, bool withRaw) {
    const auto& d = gnssProcessor.getData();
    const uint32_t age = gnssProcessor.getUniAgeMs();
    // n=0 tells "never arrived" apart from "arrived 0 ms ago" without a magic age.
    LOG_NOTICE(EventSource::GNSS,
               "UNI[%s]: n=%lu age=%lums sol=%u len=%.4f hdg=%.3f pitch=%.3f sd=%.3f/%.3f",
               tag, (unsigned long)gnssProcessor.getUniCount(),
               (unsigned long)((age == 0xFFFFFFFFu) ? 0u : age),
               (unsigned)(d.uniSolValid ? 1 : 0),
               (double)d.uniLengthM, (double)d.uniHeadingDeg, (double)d.uniPitchDeg,
               (double)d.uniHdgStdDevDeg, (double)d.uniPtchStdDevDeg);

    if (withRaw) {
        // NOT via LOG_NOTICE on purpose: measured 2026-10-01, the extra NOTICE call
        // site cost ~1 kB of RAM1 (the scarce region) in this build, and a raw
        // diagnostic must not be droppable by the INFO token bucket that
        // WheelAngleFusion/HeadingMonitor already oversubscribe. Serial only.
        Serial.printf("UNI RAW: %s\n", gnssProcessor.getLastUniRaw());
    }
}

void HeadingMonitor::pumpDump() {
    if (dumpIdx < 0) return;

    // Abandon a dump that cannot complete. If no host is attached to the USB
    // serial port, availableForWrite() stays at 0 forever and the monitor would
    // otherwise refuse to dump for the rest of the session.
    if (millis() - dumpStartMs > 30000u) {
        LOG_WARNING(EventSource::GNSS,
                    "HPR buffer dump ABANDONED after 30 s - serial port not draining");
        dumpIdx = -1;
        return;
    }

    // NEVER BLOCK. Serial.printf() on a full Teensy USB TX buffer can wait up to
    // TX_TIMEOUT_MSEC = 120 ms per call (usb_serial.c:281), and a 480-line burst
    // is ~41 KB - emitting it in one go would stall the 100 Hz steering loop
    // while the vehicle is moving. So each tick emits only while the driver
    // reports plenty of free space, capped at 6 lines (600 lines/s at 100 Hz, so
    // a full buffer drains in 0.8 s).
    //
    // The margin is 400 bytes against a ~88 byte line (4.5x). The Teensy USB
    // serial reports the sum of TX_NUM-1 = 3 inactive buffers of TX_SIZE = 2048
    // (usb_serial.c:70-71), so this threshold is satisfied in normal operation
    // but stops the dump long before the buffer is tight enough for a write to
    // have to wait. With no host attached usb_serial_write() returns immediately
    // (usb_serial.c:328), so the 30 s abandon timer above is what ends it.
    //
    // This deliberately bypasses the EventLogger: the bulk data must not consume
    // the shared token bucket (see the note at the top of this file), and the
    // lines are meant to be pasted into a spreadsheet, so the timestamp/severity
    // prefix would only get in the way.
    uint8_t lines = 0;
    while (dumpIdx < (int32_t)dumpTotal && lines < 6 && Serial.availableForWrite() > 400) {
        if (!dumpHeaderSent) {
            dumpHeaderSent = true;
            // Scales: hdg100/roll100/cog100/steer100/cmd100 are deg*100,
            // spdCmS is cm/s, hdop10 is hdop*10, dNcm/dEcm are cm per sample.
            Serial.print(F("HBUF_HDR,dtMs,hdg100,roll100,qf,fixQ,sats,hdop10,spdCmS,cog100,steer100,cmd100,dNcm,dEcm,motorPwm,steerState,guidance,psi100,ctk10\r\n"));
            continue;
        }
        emitDumpLine((uint16_t)dumpIdx);
        dumpIdx++;
        lines++;
    }

    if (dumpIdx >= (int32_t)dumpTotal) {
        const int32_t n = dumpIdx;
        dumpIdx = -1;
        LOG_NOTICE(EventSource::GNSS, "HPR buffer dump complete (%ld lines)", (long)n);
    }
}

void HeadingMonitor::emitDumpLine(uint16_t logicalIndex) {
    // The ring is written oldest-first, so logical index 0 is the oldest sample
    // still held. This is what makes the dump readable as a time series.
    const uint16_t start = (uint16_t)((head + BUF_SAMPLES - count) % BUF_SAMPLES);
    const Sample& s = buf[(uint16_t)((start + logicalIndex) % BUF_SAMPLES)];

    Serial.printf("HBUF,%u,%u,%d,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%u,%u,%u,%d\r\n",
                  (unsigned)s.dtMs, (unsigned)s.hdg100, (int)s.roll100,
                  (unsigned)s.qf, (unsigned)s.fixQ, (unsigned)s.sats,
                  (unsigned)s.hdop10, (unsigned)s.spdCmS, (unsigned)s.cog100,
                  (int)s.steer100, (int)s.cmd100, (int)s.dNcm, (int)s.dEcm,
                  (int)s.motorPwm, (unsigned)s.steerState, (unsigned)s.guidance,
                  (unsigned)s.psi100, (int)s.ctk10);
}
