#include "PositionHistory.h"

// ---------------------------------------------------------------------------------
// Storage: 300 x 72 B = 21.6 KB and it MUST live in RAM2.
// RAM1 is the FlexRAM that ITCM and DTCM share: after the 2026-10-03 baseline fix it has
// variables 158496, code 359572, padding 876, free for local variables 5344 - a 21.6 KB
// static array there would break the build immediately. DMAMEM puts the block into OCRAM
// (RAM2, 368 KB free for malloc/new) at LINK time, so an oversize is reported by
// teensy_size instead of failing at runtime.
// ---------------------------------------------------------------------------------
static DMAMEM PositionHistory::FixSample s_ring[PositionHistory::CAPACITY];

PositionHistory positionHistory;

// ---------------------------------------------------------------------------------
// Local helpers - deliberately file-static and small. FLASHMEM is NOT optional: only the
// member functions in this file carry it, and without it here distToPoint landed in ITCM
// with 144 bytes (measured with nm after the 2026-10-04 build: 0x00045e48) while RAM1 has
// 348 bytes of padding left before the next 32 KB DTCM bank.
// ---------------------------------------------------------------------------------
static double FLASHMEM distToPoint(double lat, double lon, const PositionHistory::FixSample& s) {
    const double mPerDegLat = 111320.0;
    double latClamped = s.lat;
    if (latClamped > 85.0) latClamped = 85.0;
    if (latClamped < -85.0) latClamped = -85.0;
    const double mPerDegLon = mPerDegLat * cos(latClamped * 0.017453292519943295);
    const double dN = (lat - s.lat) * mPerDegLat;
    const double dE = (lon - s.lon) * mPerDegLon;
    return sqrt(dN * dN + dE * dE);
}

// Ring index of the sample `back` records before the newest (0 = newest).
static inline uint16_t ringBack(uint16_t head, uint16_t count, uint16_t back) {
    (void)count;
    const int32_t idx = ((int32_t)head - 1 - (int32_t)back) % (int32_t)PositionHistory::CAPACITY;
    return (uint16_t)((idx + (int32_t)PositionHistory::CAPACITY) % (int32_t)PositionHistory::CAPACITY);
}

void FLASHMEM PositionHistory::init() {
    m_head = 0;
    m_count = 0;
    m_pushes = 0;
    m_dups = 0;
    m_ready = true;
}

void FLASHMEM PositionHistory::reset() {
    m_head = 0;
    m_count = 0;
    m_pushes = 0;
    m_dups = 0;
}

uint16_t FLASHMEM PositionHistory::count() const { return m_count; }

uint32_t FLASHMEM PositionHistory::pushes() const { return m_pushes; }

uint32_t FLASHMEM PositionHistory::duplicatesSkipped() const { return m_dups; }

const PositionHistory::FixSample* FLASHMEM PositionHistory::newest() const {
    if (m_count == 0) return nullptr;
    return &s_ring[ringBack(m_head, m_count, 0)];
}

const PositionHistory::FixSample* FLASHMEM PositionHistory::at(uint16_t back) const {
    if (back >= m_count) return nullptr;
    return &s_ring[ringBack(m_head, m_count, back)];
}

bool FLASHMEM PositionHistory::takeNew(uint16_t& lastSeen) const {
    const FixSample* s = newest();
    if (s == nullptr) return false;
    if (s->posCount16 == lastSeen) return false;
    lastSeen = s->posCount16;
    return true;
}

double FLASHMEM PositionHistory::distanceM(const FixSample& a, const FixSample& b) {
    return distToPoint(b.lat, b.lon, a);
}

// ---------------------------------------------------------------------------------
// One record per FIX. The marker is the POSITION COUNTER, not the wall clock: GNS/KSXT/
// INSPVA sentences set gpsData.lastUpdateTime for the SAME fix, which is what let the
// fix-to-fix window eat the projection's fix on 2026-09-23.
//
// posCount16 is a truncated copy - two fixes 65536 apart would compare equal, which at
// 10 Hz is 1.8 h and therefore impossible inside a 300 sample ring.
// ---------------------------------------------------------------------------------
bool FLASHMEM PositionHistory::push(const FixSample& s) {
    if (!m_ready) return false;
    if (m_count > 0) {
        const FixSample* prev = newest();
        if (prev != nullptr && prev->posCount16 == s.posCount16) {
            m_dups++;
            return false;
        }
    }
    s_ring[m_head] = s;
    m_head = (uint16_t)((m_head + 1) % CAPACITY);
    if (m_count < CAPACITY) m_count++;
    m_pushes++;
    return true;
}

// ---------------------------------------------------------------------------------
// The lookback everybody needs: the OLDEST sample at least minDistM away (and not further
// than maxDistM), not older than maxAgeMs. Oldest-first = the longest baseline still inside
// the limits, which is what makes a direction estimate accurate; the arc length comes back
// too because the straightness test needs chord/arc (range/arcM).
//
// The current position is a PARAMETER, not newest(): the consumers run before this fix is
// pushed (WheelAngleFusion first, HeadingBridge second, the push last), so newest() is the
// PREVIOUS fix at that moment - which is exactly the value the VWAS uses as "now" today,
// because headingBridge.getVehicleHeadingDeg() is also the previous tick's psi.
//
// Cost: one cos + one sqrt per candidate sample (<= 300) and one modulo per access. At the
// 10 Hz fix rate that is a fraction of a percent of the CPU, so it is written for clarity -
// hoist the cosine and walk the ring with a pointer only if a profile ever asks for it.
// ---------------------------------------------------------------------------------
bool FLASHMEM PositionHistory::lookback(double lat, double lon, uint32_t nowMs,
                                        float minDistM, float maxDistM, uint32_t maxAgeMs,
                                        const FixSample*& out, float& distM, float& arcM) const {
    out = nullptr;
    distM = 0.0f;
    arcM = 0.0f;
    if (m_count < 2) return false;

    int32_t found = -1;
    for (uint16_t back = (uint16_t)(m_count - 1); back > 0; back--) {
        const FixSample* s = at(back);
        if (s == nullptr) continue;
        if ((s->flags & (F_VALID | F_POS_OK)) != (F_VALID | F_POS_OK)) continue;
        if ((uint32_t)(nowMs - s->tMs) > maxAgeMs) continue;   // wrap-safe subtraction
        const double d = distToPoint(lat, lon, *s);
        if (d < (double)minDistM) continue;
        if (maxDistM > 0.0f && d > (double)maxDistM) continue;
        found = (int32_t)back;
        distM = (float)d;
        break;                        // oldest-first: the first hit is the longest baseline
    }
    if (found < 0) return false;

    // Arc length from the found sample to the current position, along the ring.
    float arc = 0.0f;
    for (int32_t back = found; back > 0; back--) {
        const FixSample* a = at((uint16_t)back);
        const FixSample* b = at((uint16_t)(back - 1));
        if (a == nullptr || b == nullptr) break;
        arc += (float)distanceM(*a, *b);
    }
    const FixSample* newestS = at(0);
    if (newestS != nullptr) arc += (float)distToPoint(lat, lon, *newestS);
    arcM = arc;
    out = at((uint16_t)found);
    return out != nullptr;
}
