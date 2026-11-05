# Central position/state ring buffer - architecture (2026-10-04)

Status: proposal for review, no code written. Operator decision 2026-10-04: the architecture
comes first, and the field list is fixed before anything is built.

Companion documents: `heading_recovery_concept_2026-10-04.md` (the measurements that led
here: `psi` 6 ... 9 deg off on clean straight RTK frames while both the attitude and the
position-anchored projection are correct to 0.15 ... 0.53 deg), `MEMORY_BASELINE.md`
(the ITCM bank cliff and the RAM budget).

## 1. Why a central ring at all

Three consumers already keep position state, each their own way:

| consumer | what it needs | today |
|---|---|---|
| `WheelAngleFusion` (VWAS) | curvature lookback -> GPS wheel angle -> zero learning | own `gpsHistory[300]`, `{double lat, lon; float heading; uint32 timeMs}` = 24 B/sample = 7.2 KB, static -> **RAM1**, 3 windows (0.10 m curvature, 0.5 m straightness, standstill). **Pushed on every 100 Hz tick**: `update()` calls `updateGPSAngle()` unconditionally and the push sits in the `hasPosition && fixQuality > 0` block, so the ring spans **3.0 s**, not the "15 ... 30 s at 10-20 Hz" its header comment claims - 9 of 10 samples are identical coordinates |
| `HeadingBridge` | fix-to-fix chord and the forward projection | **no history at all** - "keeps two positions and no history of its own" (`HeadingBridge.h:72`), "exactly two points" (`:1435`), re-based after every accepted window |
| `HeadingMonitor` | the HBUF diagnostic dump | own ring (log only) |

The bridge's forward-looking 0.5 m window and the VWAS's backward-looking ring are the same
question asked twice, and the bridge's projection resyncs onto the fix (`proj_upd` 4.1 ...
8.0/s against `fix_cnt` 10.0/s), so a measurement only exists after ~0.5 m of travel.

## 2. The finding that shapes the design: today's ring is circular

`WheelAngleFusion.cpp:465`: `currentHeading = headingBridge.getVehicleHeadingDeg();` - the
VWAS ring stores **psi, the bridge's own fused output**. The VWAS derives the wheel angle
from the difference of these samples, and the bridge's model then consumes that wheel angle.
A central ring that stores psi would cement that loop. It must carry the **raw**
measurements, with the fused values clearly separated.

## 3. Design rules

1. **One record per FIX**, triggered by `gnssProcessor.getPositionCount()`. No fix, no
   record; the gap is visible in `tMs`.
2. **Raw vs used, separated.** Raw measurements are per source and named after it. The
   "used" (fused) values carry one field plus a `*Src` tag. **`bodyHdgDeg` must never be
   read back into the fusion** - it is there as the migration bridge for the VWAS, not as
   an input.
3. **Inputs only.** `az_out`, `gain`, `innov`, `sigma_deg`, `slip_deg`, `state` are outputs
   of the fusion and stay out. Derived quantities (`travel`, `proj_meas`, `psi_att`,
   `body - att - 88.3`) are computed by the consumers/offline tools, not stored
   (operator decision 2026-10-04).
4. **Source neutral.** Every source has a validity bit, so the same record works on a UM982,
   an F9P + IMU and an F9P alone. Never read `dualHdgQ` without `dualAttOk`.
5. **Store `double`, compute `float`.** The M7 has a single-precision FPU; double arithmetic
   is software. Subtract a local base first (`localDeltaMeters()` already does this).
6. **Naming rule:** raw fields name their source (`dualAzDeg`, `dualRollDeg`-style,
   `imuHdgDeg`, `imuRollDeg`, `cogRawDeg`); used fields carry a `*Src` tag
   (`bodyHdgDeg`+`hdgSrc`, `rollDeg`+`rollSrc`). The tag values are the bridge's existing
   `Source` / `rollSource` enums - no new vocabulary.

## 4. The record

```cpp
struct FixSample {                 // 72 B, 8-byte aligned (because of the doubles)
    double   lat, lon;             //  0,  8  raw position
    uint32_t tMs;                  // 16     millis() at the fix
    float    bodyHdgDeg;           // 20     USED body heading   + hdgSrc   (today psi)
    float    rollDeg;              // 24     USED roll          + rollSrc
    float    speedMps;             // 28     raw receiver speed
    float    wheelDeg;             // 32     the wheel angle the model used + wheelQPct
    float    dualAzDeg;            // 36     RAW baseline azimuth
    float    cogRawDeg;            // 40     RAW VTG course (the HR column cog_deg is filtered!)
    float    imuHdgDeg;            // 44     RAW IMU heading, offset free
    float    imuRollDeg;           // 48     RAW IMU roll (cross-check / F9P only roll)
    uint16_t posSigmaCm;           // 52
    uint16_t imuAgeMs;             // 54
    uint16_t flags;                // 56     see 5.
    uint16_t posCount16;           // 58     truncated copy of getPositionCount()
    uint8_t  fixQ, dualHdgQ, wheelQPct, hdgSrc, rollSrc;   // 60..64
    uint8_t  pad[3];               // 65..67  -> 72 B
};
```

Selection of the types (operator questions 2026-10-04):

* **lat/lon as `double`.** double resolves 1.1e-14 deg (1 nm); float32 only 5.7e-6 deg =
  **0.64 m** and is unusable; int32 at 1e-7 deg is 1.11 cm and would save 2.4 KB but adds
  +-0.56 cm per endpoint, i.e. **+0.52 deg** of direction uncertainty on a 0.5 m chord
  against ~1.7 deg of position noise - 3 % worse, in a pool with 368 KB free. The input
  path already is double (`gpsData.latitude/longitude`, parsed by `parseLatitudeZeroCopy`),
  so nothing is gained by degrading it.
* **`tMs` as `uint32`.** Every consumer only needs differences, and wrap-safe uint16 is
  mathematically fine as long as every interval and age stays below 65.5 s. That is not
  guaranteed: `getPositionCount()` only advances on a VALID position, so under trees the fix
  rate drops (4 Hz over a 300 sample window spans 75 s), and a fix gap can exceed 65 s.
  uint16 would save 600 B (2.5 %). If the 600 B are ever needed, make the ring
  **time-bounded** ("the last 30 s") instead of sample-bounded - then the span is bounded by
  construction and uint16 is provably safe.
* **`posCount16`** is a truncated copy: a consumer only needs "did this change since my last
  look" (wrap-safe comparison), not global uniqueness.

Mapping to the HR log columns, so a snapshot writer can emit either and the existing tools
(`HrLog.ps1` maps by NAME) keep working:

| ring field | HR column | note |
|---|---|---|
| `lat`, `lon`, `tMs` | `lat`, `lon`, `t_ms` | the log prints `%.7f` = 1.11 cm; the double in RAM is far finer |
| `bodyHdgDeg` | `psi_deg` | fused body heading |
| `rollDeg` | `roll_bridge_deg` | the roll the VWAS/AOG consume |
| `speedMps` | `v_mps` | |
| `wheelDeg` | `wheel_deg` | |
| `dualAzDeg` | `att_hdg_deg` | raw `d.dualHeading` |
| `cogRawDeg` | `cog_deg` is `cogMeas` (circular average of up to 8 samples) | store the raw VTG |
| `imuHdgDeg` | `imu_hdg_raw` | |
| `imuRollDeg` | `imu_roll` | |
| `posSigmaCm` | `pos_sigma_m` x100 | |
| `imuAgeMs` | `imu_age_ms` | |
| `flags` | `rev`, `dirchg`, `cog_ok`, `pf`, ... | packed |
| `posCount16` | `fix_cnt` | truncated |
| `fixQ`, `dualHdgQ`, `wheelQPct` | `fixQ`, `qf`, `wq` | |
| `hdgSrc`, `rollSrc` | `src`, `roll_src` | the bridge's existing enums |

### Why `dualAzDeg` and `cogRawDeg` are two fields and not one

They are different physical quantities in different frames:

| | `dualAzDeg` | `cogRawDeg` |
|---|---|---|
| quantity | direction of the antenna baseline | course over ground (velocity vector) |
| frame | vehicle body (the baseline is bolted to it) | the motion path |
| valid at standstill | yes | no |
| available with | dual antenna only | any antenna, incl. F9P |
| on this machine | about `travel - 88.6` deg (baseline mounted laterally) | about `travel +- 0.2` deg |

They are ~89 deg apart here. `body = dualAz + 88.3` and **`crab = cog - body`** - the
quantity under investigation IS their difference. "Either/or" applies when USING them
(`hdgSrc`), not when storing them.

### Why `rollDeg` + `rollSrc` instead of two roll fields

The dual roll and the IMU roll measure the SAME quantity (roll about the longitudinal axis),
so one used value plus a source tag is right - exactly what the bridge already keeps
(`status.rollSource`, the log column `roll_src`). `imuRollDeg` stays as the single extra raw
field: it is the independent reference when the dual roll blows up (measured -79 ... +67 deg
under trees) and it is the only roll on an F9P machine.

`pitchDeg` and `imuYawRateDegS` were dropped (operator decision 2026-10-04): on the UM982
the baseline elevation is identical to the dual roll in every log, and the yaw rate is the
difference of the stored IMU headings.

## 5. `flags` (16 bit)

```
bit  0  valid          record valid (position and time present)
bit  1  posOk          position usable (fixQ > 0, not rejected by the step test)
bit  2  posRTK         fixQ == 4
bit  3  dualAttOk      raw dual attitude present AND dualHdgQ allowed
bit  4  cogOk          VTG course valid (speed above threshold)
bit  5  imuHdgOk       IMU heading fresh and plausible
bit  6  imuRollOk      IMU roll valid
bit  7  reserve       (the IMU yaw rate field was dropped - see 4.)
bit  8  rollOk         rollDeg usable (|dualRoll| <= maxRollAbsDeg)
bit  9  reversing      reversing at sample time
bit 10  dirChanging    direction change in progress
bit 11  attTrusted     the bridge rated the attitude as trusted
bit 12  spikeRejected  the step test dropped this position (record kept, posOk = 0)
bit 13  posStale       position older than allowed (only the time is fresh)
bit 14  straight       the chord through this sample was straight
bit 15  reserve
```

## 6. Sizing, RAM, placement

* **300 samples** (operator decision) = 32 s at 9.3 Hz. From the ring sizing: at 300 m/h a
  0.5 m gate needs 6.0 s (56 fixes) and a 1.0 m gate 12 s (112 fixes); 300 samples reach
  2.69 m at 300 m/h, i.e. both gates down to ~100 ... 150 m/h. 512 samples = 55 s would be
  the reserve option (36.9 KB).
* **72 B x 300 = 21.6 KB.**
* **Must live in RAM2** (`DMAMEM` array or a one-time malloc). RAM1 today: `variables 158 496`,
  `code 359 572`, `padding 876`, `free for local variables 5 344` - a 21.6 KB static array
  would break the build.
* **All methods `FLASHMEM`** - only 876 bytes of ITCM headroom before the 32 KB DTCM bank
  cliff (`MEMORY_BASELINE.md`).
* **Bonus:** once the VWAS reads this ring, its own 7.2 KB (RAM1, static) goes away - a net
  +7.2 KB of stack.
* Access rate 10 Hz, so the slower RAM2 costs nothing; the 100 Hz integration stays live
  state, not history.

## 7. Who pushes, and where

`AutosteerProcessor::update()` ticks the VWAS first (line 364), the bridge second (line 418).
The push needs the GNSS data AND the wheel angle, so: a **free service** called once per fix
from `AutosteerProcessor::update()`, gated on `getPositionCount()`.

* Not `HeadingBridge` as owner - the ring would become "the bridge's" and every consumer
  would depend on the bridge.
* Not the VWAS - it runs before the bridge and reads the bridge's output; that would cement
  the loop of section 2.
* Not `GNSSProcessor` - it does not know the wheel angle.

The ordering is then harmless: the VWAS reads the previous fix, which is correct because
the record holds raw measurements.

## 8. API sketch

```cpp
bool     push(const GNSSData& d, uint32_t now, float wheelDeg, int wheelQualPct);
uint16_t count() const;
const FixSample* newest() const;
const FixSample* at(uint16_t back) const;          // 0 = newest
// the lookback that matters: the newest sample at least minDistM away, plus the arc length
bool     lookback(float minDistM, float maxDistM, uint32_t maxAgeMs,
                  const FixSample*& s, float& distM, float& arcM) const;
bool     window(...);                              // quality gated, for the LSQ idea
void     reset();
```

Consumers cache whatever they computed, keyed on a change of `posCount16`; the ring does no
per-sample work beyond the copy.

## 9. Migration (one step, but NOT behaviour neutral - it corrects an error)

**The finding (2026-10-04).** The VWAS was written and tuned for a **per-fix (10 Hz)** cycle
but is called at **100 Hz**:

* the ring is pushed on every tick -> a 3.0 s span with 90 % duplicate coordinates, while its
  header comment says "~15-30s of history at 10-20Hz" and the lookback comment says "called
  per GPS fix (10 Hz)" (`WheelAngleFusion.cpp:562`);
* the curvature EMA (`:808`) hardcodes `dtSmooth = 0.1f` ("approx cycle time") and computes
  `alpha = dtSmooth / (gpsAngleSmoothingTau + dtSmooth)`. `gpsAngleSmoothingTau` is
  **1.0 s**, so the intent is a 1 s time constant - but applied every 10 ms the real one is
  **~0.105 s**: the smoothing is **10x too weak**;
* the standstill window (`STANDSTILL_WINDOW_S = 3.0 s`, `STANDSTILL_MIN_FRACTION = 0.75`)
  is exactly as long as the whole ring -> no headroom.

So the migration is **not** "identical inputs, no behaviour change". Moving the VWAS to the
per-fix ring **restores the intended design** (the hardcoded `dtSmooth = 0.1f` becomes
correct, the standstill window gets 30 s of history, and the duplicate samples disappear) -
which means the GPS-derived wheel angle **will get calmer**. That is a visible change on the
machine and belongs in the field test, not in a "should be identical" assumption.

Order of work:

* **Step 1:** `PositionHistory` + push service + the log written from the ring. The bridge
  and the VWAS still use their own state - no behaviour change at all.
* **Step 2:** `HeadingBridge` reads the ring for its look-back window (the availability fix).
* **Step 3:** the VWAS reads the ring instead of its own buffer: `bodyHdgDeg` for the
  differences, `dualAzDeg`/`cogRawDeg` for its own source selection, `hdgSrc` per sample so
  a window can be constrained to ONE heading source (today `currentHeading` may switch
  frames inside a window - psi is a body heading, `dualHeading` is a baseline azimuth ~88 deg
  away - and a difference across such a switch is wrong; `usingDualHeadingNow` is logged for
  exactly this reason). Verify the WAS angle and the zero learning before/after.
* **Step 4:** only then do the raw columns open the real improvements (position anchored
  direction measurement, model-vs-geometry residual).

## 10. The log going away - the ring becomes the interface

* Add a **snapshot endpoint** (`GET /api/ring/snapshot?format=csv|bin`): the 300 records,
  ~22 KB over Ethernet.
* **No derived values in the ring** (operator decision, point 5): the tools compute
  `travel`, `proj_meas`, `psi_att` themselves - `projqc.ps1` already reconstructs
  `proj_meas` from `proj_ctk_m`/`proj_rng_m`, and `skew.ps1` computes the chord. Storing them
  would duplicate logic and go stale.
* Column names follow the mapping table, so `HrLog.ps1` (maps by NAME) and the existing
  tools re-point with minimal change.
* The web UI carries the live essentials (skew, proj vs att, clip share, state/src) so no
  serial console is needed.

## 11. Open items

* Ring capacity 300 (21.6 KB) vs 512 (36.9 KB)?
* Time-bounded ring (uint16 time, saves 600 B) - only if RAM ever matters.
* The bridge's window re-base rule (`maxWindowM`) - a separate decision, see
  `heading_recovery_concept_2026-10-04.md`.
* A second, small ring for the 100 Hz IMU rate - only if a lookback ever needs it. Not now.

## 12. Step 1 build report (2026-10-04, measured, not assumed)

PlatformIO `teensy41`, `AiO_v26.5.1.hex` (1986 KB), build SUCCESS.

| | before Step 1 | after Step 1 | delta |
|---|---|---|---|
| **RAM2 variables** | 156 000 | **177 600** | **+21 600 = 300 x 72** |
| FLASH code | 382 584 | 384 308 | +1 724 |
| RAM1 code (ITCM) | 359 764 | 359 972 | +208 |
| RAM1 padding | 684 | 476 | -208 |
| free for local variables | 4 320 | 4 288 | **-32** |

* `s_ring` sits at **0x20226160** (OCRAM = RAM2) with size 0x5460 = 21 600 B - verified with
  `arm-none-eabi-nm`, not assumed. `positionHistory` itself is 16 B in DTCM (0x200255AC).
* The 32 B of stack is the whole steady-state cost; the ring does not touch RAM1.
* `RAM1 padding` is only alignment slack (the 524 288 B total is fixed). The number that
  matters is `free for local variables`.

### Two traps found while building (both would have gone unnoticed)

1. **`hdgSrc` did not match `HeadingBridge::Source`.** The first draft of the enum had 6
   entries (`HDG_PROJ = 5`); the bridge has 7 (`PATH_PROJECTED = 2 ... DEADRECKON = 6`). The
   log column would have shown a projected heading as `imuhdg`, with no error anywhere.
   Fixed, and `PositionHistoryService.cpp` now carries one `static_assert` per entry - it
   fails the BUILD if the bridge ever reorders its enum.
2. **`FLASHMEM` on the member functions is not enough.** The file-static helper
   `distToPoint()` landed in **ITCM with 144 bytes** (nm: 0x00045E48) while only 684 B of
   padding were left - exactly the mistake section 6 warns about. Fixed (now 0x600069E8).
   Rule: every function in these .cpp files, including `static` helpers, needs `FLASHMEM`.

### The push service

`AutosteerProcessor::process()` calls it directly after `headingBridge.update()`:

```cpp
positionHistoryService(millis(), currentAngle, steerAngleOutQuality);
```

* `positionHistory.init()` runs once from `AutosteerProcessor::init()`.
* The **fix gate is inside `push()`** (position counter vs. the newest record), so the service
  keeps no state of its own, 9 of 10 calls are dropped duplicates, and `duplicatesSkipped()`
  turns the call rate into a measured number instead of a comment. Both counters are in the
  new log line.
* It is the only place that reads GNSS + bridge + IMU + the wheel angle together.
* `posSigmaCm` comes from the new const getter `headingBridge.getPositionSigmaM()` - the
  fix-type table stays in one place.

### The log line

`PH,` - one line per STORED record (10 Hz, the same rate as `HR,`, so the two align on
`t_ms`), emitted by the service itself because that is the only place that knows a record was
stored. The prefix is distinct so that every existing analysis script (all of them filter on
`HR,`) keeps working unchanged. Gate: the bridge's existing `logRawFrames` switch, so no new
EEPROM field was needed.

Columns: `t_ms, lat, lon, v_mps, wheel_deg, wheel_q, psi_deg, hdg_src, roll_deg, roll_src,
dual_az, cog_raw, imu_hdg, imu_roll, imu_age_ms, fixQ, dualHdgQ, pos_sigma_cm, flags, ring_n,
pos_cnt, ring_push, ring_dup`.

### Still open in Step 1

* The `PH` columns are the **raw/used inputs only**; the bridge's own outputs (innov, gain,
  `az_out`, rate limiter state) cannot come from the ring by design (section 3: "inputs
  only"). Replacing the `HR` line is therefore a separate decision that would break the 15
  analysis scripts - deliberately NOT done.
* Not yet verified on hardware: the first field log with `PH,` should show ~9 `ring_dup` per
  fix and `ring_n` reaching 300 after ~32 s.

