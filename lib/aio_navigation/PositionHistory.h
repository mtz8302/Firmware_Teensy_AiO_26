#pragma once
#include <Arduino.h>

// =================================================================================
// PositionHistory - the CENTRAL per-fix position/state ring
// ---------------------------------------------------------------------------------
// Design and rationale: knowledge/position_ring_buffer_architecture.md.
// One record per GPS FIX, pushed once per fix by AutosteerProcessor::update() and read
// by WheelAngleFusion, HeadingBridge and the log/snapshot writer.
//
// Why it exists: the VWAS kept its own ring (pushed every 100 Hz tick -> a 3 s span with
// 90 % duplicate coordinates) and the HeadingBridge kept no history at all (a
// forward-looking 0.5 m window, re-based after every accepted window). This ring is that
// question asked once, for everybody.
//
// RULES (see the note):
//   * one record per FIX, gated inside push() on the position counter;
//   * RAW measurements keep their source in the name (dualAzDeg, cogRawDeg, imuHdgDeg,
//     imuRollDeg); the USED/fused values carry one field plus a *Src tag (bodyHdgDeg +
//     hdgSrc, rollDeg + rollSrc);
//   * bodyHdgDeg must NEVER be read back into the heading fusion - it is the migration
//     bridge for the VWAS, not an input;
//   * inputs only: no az_out / gain / innov / state, and no derived values (the offline
//     tools compute travel / proj_meas / psi_att themselves);
//   * source neutral: every source has a validity bit, so the same record works on a
//     UM982, an F9P + IMU and an F9P alone. Never read dualHdgQ without F_DUAL_ATT_OK.
//
// MEMORY: 400 x 72 B = 28.8 KB in RAM2 (DMAMEM - RAM1 has only ~5.3 KB of stack left and
// 876 B of ITCM code headroom before the next 32 KB DTCM bank). Every function in the
// .cpp is FLASHMEM for that reason; keep this header free of real code.
// =================================================================================

class PositionHistory {
public:
    // 400 (2026-10-13, Betreiber-Entscheidung): das Kriterium ist die Ringreichweite in
    // METERN, nicht die Zahl der Eintraege. Bei 1 km/h und 20 Hz sind 300 Eintraege nur
    // 4,2 m, 400 sind 5,6 m (11,1 m bei 10 Hz) - das deckt die 5-m-Vetosehne UND die
    // 2-m-Ausgabesehne bei JEDER Empfaengerrate (5/10/20 Hz); die 10-m-Feinpruefung ist
    // im Schleichgang bewusst nicht verfuegbar. 400 x 72 B = 28,8 KB (RAM2: ~346 KB frei).
    static const uint16_t CAPACITY = 400;      // 400 fixes = ~43 s at 9.3 Hz, 28.8 KB

    // --- validity / status bits of FixSample::flags ---------------------------
    enum : uint16_t {
        F_VALID          = 1u << 0,   // record valid (position and time present)
        F_POS_OK         = 1u << 1,   // position usable (fixQ > 0, not step-rejected)
        F_POS_RTK        = 1u << 2,   // fixQ == 4
        F_DUAL_ATT_OK    = 1u << 3,   // raw dual attitude present AND dualHdgQ allowed
        F_COG_OK         = 1u << 4,   // VTG course valid (speed above threshold)
        F_IMU_HDG_OK     = 1u << 5,   // IMU heading fresh and plausible
        F_IMU_ROLL_OK    = 1u << 6,   // IMU roll valid
        F_RESERVED7      = 1u << 7,   // (the IMU yaw rate field was dropped)
        F_ROLL_OK        = 1u << 8,   // rollDeg usable (|dualRoll| <= maxRollAbsDeg)
        F_REVERSING      = 1u << 9,   // reversing at sample time
        F_DIR_CHANGING   = 1u << 10,  // direction change in progress
        F_ATT_TRUSTED    = 1u << 11,  // the bridge rated the attitude as trusted
        F_SPIKE_REJECTED = 1u << 12,  // step test dropped this position (record kept)
        F_POS_STALE      = 1u << 13,  // position older than allowed
        F_STRAIGHT       = 1u << 14,  // the chord through this sample was straight
        F_RESERVED15     = 1u << 15
    };

    // Which source the USED values came from.
    //
    // The VALUES ARE HeadingBridge::Source, value for value - verified 2026-10-04 against
    // HeadingBridge.h:1017 (7 entries, PATH_PROJECTED=2 ... DEADRECKON=6), so
    //   HeadingBridge::sourceName((HeadingBridge::Source)hdgSrc)
    // prints the same text the log's src column carries and the offline tools keep working.
    // The FIRST draft of this enum had 6 entries with PROJ=5 - that would have logged a
    // projected heading as "imuhdg" without any error. Keep in sync if the bridge grows a
    // source; StaticSrcCheck in PositionHistoryService.cpp fails the build if it drifts.
    enum HeadingSrc : uint8_t {
        HDG_NONE = 0, HDG_ATTITUDE = 1, HDG_PROJECTED = 2, HDG_FIX2FIX = 3,
        HDG_COG = 4, HDG_IMU = 5, HDG_DEADRECKON = 6
    };
    enum RollSrc : uint8_t {
        ROLL_NONE = 0, ROLL_DUAL = 1, ROLL_IMU = 2, ROLL_HELD = 3
    };

    // --- the record: 72 bytes, 8-byte aligned (because of the doubles) --------
    struct FixSample {
        double   lat, lon;          //  0,  8  RAW position (double - see the note)
        uint32_t tMs;               // 16      millis() at the fix
        float    bodyHdgDeg;        // 20      USED body heading          + hdgSrc
        float    rollDeg;           // 24      USED roll                   + rollSrc
        float    speedMps;          // 28      RAW receiver speed
        float    wheelDeg;          // 32      the wheel angle the model used + wheelQPct
        float    dualAzDeg;         // 36      RAW baseline azimuth
        float    cogRawDeg;         // 40      RAW VTG course (cog_deg in the log is FILTERED)
        float    imuHdgDeg;         // 44      RAW IMU heading, offset free
        float    imuRollDeg;        // 48      RAW IMU roll (cross-check / F9P only roll)
        uint16_t posSigmaCm;        // 52
        uint16_t imuAgeMs;          // 54
        uint16_t flags;             // 56      F_* bits above
        uint16_t posCount16;        // 58      truncated getPositionCount()
        uint8_t  fixQ;              // 60
        uint8_t  dualHdgQ;          // 61      only meaningful with F_DUAL_ATT_OK
        uint8_t  wheelQPct;         // 62      0...100, 255 = not reported
        uint8_t  hdgSrc;            // 63      HeadingSrc
        uint8_t  rollSrc;           // 64      RollSrc
        // --- per-epoch quality of the ATTITUDE, raw from the receiver (2026-10-05) ----------
        // These two live in the three bytes that used to be padding, so the record stays 72 B.
        // They are RAW receiver figures, not the bridge's verdict, so every consumer can apply
        // its own thresholds - that is the "quality criteria per measurement" the operator asked
        // for, and it keeps the ring free of policy (the filters compute quality from facts).
        // Why exactly these two: they are the only per-epoch attitude-quality numbers that are
        // NOT derivable from the position/heading values already in the record. Measured on the
        // 2026-10-04 logs: the receiver's heading sigma is live (p50 0.44 ... 0.57 deg, p90 up to
        // 3.51 in a degraded drive) and correlates with the actual attitude error (r = 0.24 ...
        // 0.53), and the solved baseline (p50 0.711 m, stable) is the early warning the note
        // describes - a deviation from the learned reference appears BEFORE the heading degrades.
        uint8_t  hdgSdX100;         // 65      receiver heading 1 sigma, 0.1 deg steps, 0 = n/a
        uint16_t baselineCm;        // 66, 67  solved baseline length in cm, 0 = n/a
                                    //         (UNIHEADINGA length / RELPOSNED baseRelL)
    };
    static_assert(sizeof(FixSample) == 72, "FixSample must stay 72 bytes (ring sizing)");

    // --- lifecycle and writing ------------------------------------------------
    void init();                              // once from setup(); allocates nothing
    bool push(const FixSample& s);             // true when a NEW fix was stored
    void reset();                             // forget everything (link loss, config change)

    // --- reading --------------------------------------------------------------
    uint16_t count() const;
    static constexpr uint16_t capacity() { return CAPACITY; }
    static constexpr uint32_t byteSize() { return (uint32_t)CAPACITY * sizeof(FixSample); }
    const FixSample* newest() const;           // nullptr while empty
    const FixSample* at(uint16_t back) const;  // 0 = newest, nullptr when unavailable

    // Consumer-side fix gate: true when the newest sample differs from *lastSeen, and
    // updates *lastSeen. Every consumer keeps its own small counter - no shared state.
    bool takeNew(uint16_t& lastSeen) const;

    // The lookback everybody needs: the OLDEST sample that is at least minDistM away from
    // (lat, lon) but not further than maxDistM, and not older than maxAgeMs. Returns the
    // sample, the straight-line distance and the ARC length along the ring (needed for the
    // straightness test). false when no such sample exists.
    //
    // The current position is passed in instead of being taken from newest(), because the
    // consumers run before the push of the current fix (WheelAngleFusion runs first,
    // HeadingBridge second, the push last - see the note).
    bool lookback(double lat, double lon, uint32_t nowMs,
                  float minDistM, float maxDistM, uint32_t maxAgeMs,
                  const FixSample*& out, float& distM, float& arcM) const;

    // Straight-line distance between two records, in metres (equirectangular, the same
    // maths the rest of the navigation code uses over these few metres).
    static double distanceM(const FixSample& a, const FixSample& b);

    // --- diagnostics (web / snapshot) -----------------------------------------
    uint32_t pushes() const;                   // records stored since boot
    uint32_t duplicatesSkipped() const;        // pushes that were the same fix

private:
    uint16_t m_head;                           // next write index
    uint16_t m_count;                          // valid records (caps at CAPACITY)
    bool     m_ready;
    uint32_t m_pushes;
    uint32_t m_dups;
};

extern PositionHistory positionHistory;
