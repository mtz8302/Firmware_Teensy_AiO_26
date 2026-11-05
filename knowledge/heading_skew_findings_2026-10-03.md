# Heading skew ("Tractor läuft schräg") - findings 2026-10-03 evening

Logs: `2026-10-03 16-00 k wende bÄume`, `16-04 0 bach bäume k bäume`,
`16-08 k bach bäume autosteer schleichfahrt` (build with the baseline default 0.711 m).
Tools: `tools/analysis/askew.ps1` (heading vs travel, split by state), `azscan.ps1`
(chunk scan), `offset_check.ps1`, `azcheck.ps1`, `rej_hist.ps1`, `_devchk.ps1`.

## 1. The baseline fix works

```
_devchk:  ref samples 747:  p05 0,710  p50 0,711  p95 0,712      (was 0.852, frozen)
rej_hist: baseLen 1,5 ... 8,6 %  of the frames                     (was ~100 %)
```

The receiver again solves 0.711 m and the gate no longer discards the solution. This is
what made the heading "viel stabiler".

## 2. The remaining rejects are a different set

| log | none | settle | rollEnv | qf | baseLen | rollStep | cogX |
|---|---|---|---|---|---|---|---|
| 16-00 | 80.4 % | 2.5 | 2.6 | 10.9 | 2.1 | - | 1.4 |
| 16-04 | 58.4 % | **14.5** | **7.7** | 9.9 | 8.6 | - | 0.8 |
| 16-08 | 69.6 % | **12.0** | **14.0** | 2.7 | 1.5 | 0.2 | - |

`settle` and `rollEnv`/`rollStep` are now the dominant losses (up to 15 % each) - the
attitude gate and the roll envelope, not the length.

## 3. The skew, measured

**The receiver is not the problem.** On straight clean driving (qf=fixQ=4, |wheel| < 5):

```
travel - att = 88.3 deg, sd 0.9    (3 logs, 520 samples, R = 0.93 ... 0.98)
```

and `att_hdg_deg == uni_hdg` exactly (the two sentences agree), qf = 4, 26...31 sats,
`uni_hdgsd` = 0.38 ... 0.55 deg. So the mechanical offset really is **88.3 deg**, not 90.

**What AOG receives is not the attitude.** `az_out` follows `psi - 90`
(`psi` = `status.vehicleHeadingDeg`, the bridge's FUSED body heading), not `att`:

| log | travel - az_out | travel - att | skew if AOG offset = 88 |
|---|---|---|---|
| 16-00 | **75.8** | 88.5 | **-12.2 deg** |
| 16-04 | **85.2** | 88.4 | -2.6 deg |
| 16-08 | **102.9** | 88.2 | **+14.9 deg** |

The offset AgOpenGPS would have to carry is **different in every drive**, so no single AOG
setting can be right - and that is exactly what a diagonally running tractor looks like.

Raw proof that `psi` is the wrong one, not `att` (16-08, t = 219.8 s, straight, wheel 2.9):

```
att = 212.8   att+90 = 302.8   cog = 304.2   <-- the attitude is CORRECT
psi = 253.7   az_out = 163.1                 <-- published, 49 deg off
```

## 4. It is NOT a regression

Old logs, before any of the 2026-10-03 firmware changes show the same scatter:

| log | travel - az_out (straight) | R |
|---|---|---|
| 2026-10-02 16-23 | 88.9 | 1.00 |
| 2026-10-02 16-27 | 89.2 | 1.00 |
| 2026-10-02 16-30 | 88.3 | 1.00 |
| 2026-10-01 20-11 | **75.3** | 0.90 |
| 2026-10-01 20-15 | **55.5** | 0.79 |
| 2026-10-01 20-18 | 84.9 | 1.00 |

55 ... 89 deg on the old logs, 76 ... 103 on the new ones. Same defect, same size.

## 5. Mechanism

`psi` is a filter state (kinematic model fused with measurements). It

* **lags during and after turns** - up to 50 ... 60 deg wrong for several seconds (16-08
  t = 218.9: wheel 25.4 deg, cog 299.7, att+90 286.0, psi 241.0), and
* **sits systematically off on straight driving** - 3 ... 15 deg, a different value per
  drive.

The mount offset is NOT the cause: `mount_deg` measures -86.9 / -88.0 / -88.2 against a
true 88.3, i.e. within 1.4 deg - while `psi` is off by 12.2 / 2.6 / 14.4. Side slip does
not explain it either (slip p50 = 8.4 / 11.9 / -0.5 against deviations -12.2 / -2.6 / +14.4).

**Candidate fix:** in state LIVE with source `att`, publish the heading derived from the
ATTITUDE (`att` + the offset) instead of the fused `psi` - true pass-through, zero lag.
`heading_bridge_concept.md` claims this already happens ("in normal operation is the
receiver's own azimuth (pass-through)"); the measurement says `az_out != att`, so it does
not. Not implemented yet - it is a behaviour change in the output path and needs its own
measurement before/after.

## 6. Open questions for the operator

1. What is AgOpenGPS' **"Dual Heading Offset"** set to? The mechanical value is 88.3 deg.
   The firmware's own value could not be read: the boot line is cut off in these logs.
2. Does it run diagonally **continuously** or **after turns / K-turns**? The data shows
   both, so the answer decides which of the two mechanisms dominates in practice.
3. Is the skew **constant within one drive** (then it is an offset/configuration problem)
   or does it **change during the drive** (then it is the fusion state)?
4. Is only the **display** in AOG skewed, or does the machine also **steer badly**?
5. A log of **pure straight autosteer driving** (no turns, ~30 s) would separate the
   steady skew from the turn lag cleanly.

## 7. Answers from the operator, and what they mean

1. **AgOpenGPS "Dual Heading Offset" = +88 deg.** The mechanical measurement is 88.3, so AOG
   is right and the 12...15 deg skew is NOT an offset-configuration problem.
2. The first attempt was skewed **from the start**; that log no longer exists.
3. **"It sometimes jumps back to reality and then grows more skewed again."** This is the
   signature of a state that is walked away by a weak prediction and pulled back by
   occasional strong measurements - a fusion state problem, not a constant offset.
4. Both the display and the steering are affected, but **during autosteer the wrong angle was
   not that large** - consistent with the model error shrinking at higher speed / smaller
   steering angles.

## 8. The boot lines (16-04 contains them)

```
HeadingBridge ENABLED: L=2.50m scale=1.00 dualHdgOff=+90.0deg (AgOpenGPS value) ... base=0.000m
HeadingBridge config: useImuHeading=1 useImuRate=1 imuRateInvert=0 estimateSideSlip=1
                      imuAnchor=400ms/1.0mps travelWitness=1.0m/30deg reseedBeyond=120deg
HeadingBridge physicalBaselineM: not in the EEPROM, using the built-in default 0.711 m
VWAS config loaded from EEPROM: wheelbase=2.14 type=0 cpd=18.1 maxAngle=40.0 dual=1 invert=1
```

Three facts fall out of this:

* **The firmware runs `dualHeadingOffsetDeg = +90.0` while AgOpenGPS uses +88 and the
  mechanics are 88.3.** In pass-through that costs nothing (AOG applies its own value), but
  every frame whose heading comes from the path/model is converted with 90 and then has AOG's
  88 added back - a 2 deg inconsistency exactly in the non-passthrough states.
  This value has **no EEPROM slot and no web field** - the same class of gap the baseline had.
* **The IMU is dead in this log**: `imurate`, `yawbias`, `imu_q`, `imu_flags`, `imu_age_ms`
  are all 0 and `psi_imu = wrap(psi - 0)`. `useImuHeading=1 useImuRate=1` is configured but
  no IMU data arrives, so the IMU is ruled out as the source of the drift - and it means the
  configured IMU support is doing nothing.
* `estimateSideSlip=1` is active; side slip reached 8...12 deg in the 16-00/16-04 logs.

## 9. The straight stretch in 16-04 (the operator's hint) - read at t = 23.9 ... 36.8 s

```
    k    t_s     v  wheel    cog    f2f    att    psi az_out  mount   src  state
   72   23.9  3.56  -24.5  303.0  304.0  217.9  306.4  216.0  -89.3  att  LIVE
  111   27.8  3.88  -24.8  305.0  305.1  217.1  306.9  216.9  -89.2  att  LIVE
  132   29.9  3.39  -15.5  304.4  302.2  216.9  307.3  217.6  -89.2  att  LIVE
  165   33.2  5.96   -0.4  303.5  303.4  214.9  304.9  214.8  -89.1  att  LIVE
  201   36.8  6.19    1.7  303.6  304.0  216.3  306.4  216.4  -89.0  att  LIVE
```

* the **wheel angle finds 0 cleanly** (from -25 to 0 and holds) - the WAS zero is fine
* **`f2f` == `cog`** within 1 deg - the fix-to-fix PATH heading is uniform and correct
* **`az_out` == `att`** - pass-through works, so AOG gets the receiver's azimuth
* but the offset that comes out of it changes: `travel - az_out` = **87.0 at t=23.9** and
  **88.7 at t=33.2**, while `cog` is nailed at 303-305 and only `att` moves (217.9 -> 214.9).

So on this stretch the *only* moving quantity is the receiver's azimuth, and it moves by
~1.7 deg within 10 s. The path heading and COG agree with each other throughout.

## 10. Where the drift most likely comes from (to be confirmed)

Ranked by the evidence, and each needs its own measurement:

1. **`steerScale = 1.00`** with `applyLearnedSteerScale = 0`: the kinematic model
   `omega = v*tan(scale*delta)/L` under-predicts the yaw rate if the true scale is higher,
   so `psi` falls behind in every turn and is pulled back later - "grows more skewed again,
   then jumps back". `heading_bridge_concept.md` warns that the historical 1.38 is the
   ESTIMATOR's structural bias, so this must be measured, not guessed.
2. **The receiver's azimuth itself is not steady**: on a straight stretch it moved 1.7 deg in
   10 s and up to 3.5 deg against `f2f`/`cog`, which agree with each other to 1 deg.
3. **The firmware/AOG offset mismatch (90 vs 88 vs 88.3 mechanics)** - small, but it is
   active in exactly the non-passthrough states that survive during `settle`/`rollEnv`.

## 11. Fixed while analysing

`HeadingBridge::init()` applied the baseline default AFTER printing the
`HeadingBridge ENABLED: ... base=%.3fm` line, so the boot log showed **`base=0.000m`** while
the effective reference was already 0.711 m - the very number a field test reads first. The
block now runs before the log. Build re-verified: RAM1 unchanged (`code 359572`,
`padding 876`, `free for local variables 5344`).


## 12. A - done: `dualHeadingOffsetDeg` is now a real setting (build 18:13)

The value existed only as a compile-time literal (`Config::dualHeadingOffsetDeg = 90.0f`) and was
used in TWO places, so the 1.7 deg error was NOT harmless:

```
HeadingBridge.cpp:460   body      = az_recv + dualHeadingOffsetDeg   (the attitude measurement)
HeadingBridge.cpp:2065  published = psi      - dualHeadingOffsetDeg  (every non-boot frame)
```

The operator enters the same number in AgOpenGPS (+88). The firmware is now **88.3** (measured:
`offset_check.ps1`, 3 logs / 520 samples, sd 0.9; the firmware's own `mount_deg` reads
-89.0 ... -89.3).

Changed:

| file | what |
|---|---|
| `ConfigManager.h/.cpp` | `bridgeDualHeadingOffsetDeg` + marker pair in the EEPROM, bytes 11..16 of 1600-1699, same rule as the antenna spacing (nothing stored -> build default) |
| `HeadingBridge.h` | `dualHeadingOffsetDefaultDeg = 88.3f`, `dualHeadingOffsetDeg` initialised from it; `setDualHeadingOffsetDeg()` + getter |
| `HeadingBridge.cpp` | `init()` applies the EEPROM value BEFORE the summary line (so `dualHdgOff=%.1fdeg` reports what is in use); `setDualHeadingOffsetDeg()` (`FLASHMEM`) persists it |
| `SimpleWebManager.h/.cpp` | `/api/bridge/baseline` became **`/api/bridge/geometry`** (GET both values + the firmware's own `mnt` measurement, POST accepts `meters` and/or `offsetDeg`); `bridgeDualHeadingOffsetDeg` added to `/api/vwas/status` |
| `VWASConfigPage.h` | section renamed to "Antennen-Geometrie (Basislinie)", second field for the offset, one save button; the readout compares the offset against the firmware's own `mnt` and warns above 3 deg |

Build: **SUCCESS**, `AiO_v26.5.1.hex` 18:13:35. RAM1 `variables 159520`, `code 359764`,
`padding 684` (still 11 ITCM banks), `free for local variables 4320` - under the cliff.

## 13. B - why the K-turn reversal is MISSED (analysis, NOT built)

The truth is known (`kturn.ps1`, offset 88.3): 16-04 has a real 3.0 s reversal at t = 84.8...87.8 s
(v p50 = 0.93 m/s), 16-08 a 2.0 s one at t = 213.6...215.6 s (1.04 m/s). In BOTH the firmware flag
`rev` was **never** set (MISSED 31 and 21 frames, 0 false).

The acceptance chain in `updateDirection()` requires ALL of:

```
1  !dirChanging            |dirDevEma - raw| <= directionChangeDeg (28.6)
2  reverse                 dirDevEma > reverseDetectDeg (90)
3  attitude witness        attDev >= reverseConfirmDeg (120)   <- else VETO
4  solid travel vector     !travelWitnessValid || travelSolid  <- else VETO
5  dirVotesNeeded (2)      the same verdict must repeat
```

Two of these are TIMING blockers, and the arithmetic is decisive:

* `dirDevEma` is an EMA with `directionFilterAlpha = 0.2` (AgOpenGPS's own 0.2/0.8). Walking it
  from 0 to a settled 180 deg takes **9 evaluations** before `|ema - raw|` finally drops below
  28.6 and `dirChanging` clears - and until then line 2555 RETURNS. Then 2 more votes are needed:
  **~11 evaluations in total.**
* one evaluation needs `needM = clamp(5 * sigma_pos, 0.25, 0.5) m` of real displacement, i.e.
  ~0.5 m at best.
* the K-turn offers `v p50 0.93 m/s * 3.0 s = 2.8 m` -> **~5-6 evaluations**. 16-08 is worse:
  `1.04 * 2.0 = 2.1 m` -> ~4.

So the manoeuvre is simply **too short** for the current chain: by the time the EMA would have
settled, the machine is driving forward again, `raw` falls back and `revVotes` is reset
(`revVotes = 0` in the else branch). Nothing is ever accepted.

### Proposal (minimal, and the operator's idea is the right lever)

Use a **standstill as positive evidence**, not as a gate that blocks: a K-turn ALWAYS contains a
stop, and a stop is the one moment where the old verdict may be re-derived. The code already has
exactly this mechanism for the power-on case (`!dirDevInit`, line 2537):

```cpp
dirDevEma = raw;                       // seed, do not ramp from 0
status.reverseDirection = witnessForward ? false
                         : (witnessReverse ? true : (raw > config.reverseDetectDeg));
return;                                // "the witness still has the last word"
```

So: track how long the machine has been standing (`|v| < ~0.15 m/s` for `>= ~300 ms`), and when it
starts moving again, run that same seed-and-decide branch ONCE. That removes blocker 1 entirely
(the 9-evaluation EMA ramp is the dominant part) and leaves blocker 5 (2 votes) as the remaining
budget - which 2.8 m of travel provides.

**Why this is safe:**

* the witness conditions (3) and (4) are NOT touched: a reversal still needs `attDev >= 120` or no
  attitude, so the re-seed cannot create a false reversal - it only shortens the delay;
* it fires only after a genuine standstill, so it cannot trigger during normal driving - the
  working F9P + IMU path stays bit-identical;
* it is not a new mechanism: it is the boot branch, just also armed by a standstill.

Open for the build: the exact `standstillSpeedMps` / `standstillMinMs` values, and whether
`minSpeedForDirection = 1.0 m/s` (currently only used by the emergency CLEAR at line 2511) should
also apply to the fresh decision - at 0.93 m/s the K-turn was below it.

**Not built.** Analysis only, per the operator's instruction to go slowly.


## 14. B - simulated on the logs, then BUILT (build 18:22)

`tools/analysis/revfix.ps1` replays `updateDirectionState()` frame by frame on the log and
compares BASE against the standstill re-seed. Truth is the independent definition from
`kturn.ps1` (0.5 s position chord vs `att + 88.3`, reverse above 120 deg).

### The real blocker is even simpler than the EMA chain

```
HeadingBridge.cpp:2438  if (!status.attitudeTrusted || status.posFallbackActive ||
                            lastSpeedMps < config.minSpeedForDirection ||   <-- 1.0 m/s
                            ...) { dirRefValid = false; return; }
```

The K-turn ran at **v p50 = 0.93 m/s** - **below** that gate, so the whole direction logic
returned early and did nothing at all for most of the manoeuvre. Where it did run, the chain
needs ~11 evaluations (`needM = clamp(5*sigma, 0.25, 0.5) m`, so one per 0.25 m) while the
manoeuvre only offers 2.8 m / 3.0 s -> ~5.

### Simulation result - and a correction

The FIRST version of the tool did not model the emergency clear (firmware 2510-2527) - and that
is exactly what releases a stale REVERSE in the real firmware. So BASE looked far worse than it
is. The tool now models `updateTravelWitness()`, the emergency clear and the `travelSolid` veto
for BOTH variants. Honest numbers over 16 logs:

| | detected (tp) | missed (fn) | false (fp) |
|---|---|---|---|
| BASE | 120 | 308 | 233 |
| **FIX** | **282** (x2.3) | **146** (-53 %) | **142** (-39 %) |

| log | truth | BASE tp/fn/fp | FIX tp/fn/fp | BASE sets | FIX sets |
|---|---|---|---|---|---|
| 03.10 12-55 | 57 | 10 / 47 / 29 | 46 / 11 / 14 | 1.7 s / never | 0.6 s / 0.5 s |
| 03.10 16-00 | 53 | 15 / 38 / 94 | 27 / 26 / 8 | 3.8 s | 2.6 s |
| 03.10 16-04 | 31 | **0 / 31 / 0** | 17 / 14 / 5 | **never** | **1.4 s** |
| 03.10 16-08 | 21 | **0 / 21 / 0** | 14 / 7 / 6 | **never** | **0.7 s** |
| 02.10 16-27 | 55 | 0 / 55 / 0 | 29 / 26 / 12 | never | 0.4 s (1 of 2) |
| 01.10 13-20 | 124 | 76 / 48 / 51 | 113 / 11 / 68 | 2.3 s | 0.4 s |
| 01.10 20-06 | 42 | 19 / 23 / 59 | 36 / 6 / 29 | 2.3 s | 0.6 s |
| 01.10 20-11 | 24 | **0 / 24 / 0** | **0 / 24 / 0** | never | never |
| 01.10 20-18 | 21 | **0 / 21 / 0** | **0 / 21 / 0** | never | never |

**Safety: in the 11 reversal-free logs of the 16, BOTH variants are 0/0/0.** The false windows
that remain are RELEASE delays of 0.4 ... 5.0 s right after a real manoeuvre - the old behaviour
has them too - and not spurious latches during driving.

### Crawl speed (the operator's question)

The change touches ONLY `status.reverseDirection`. The heading, the kinematic model, the VAS
zero-point observer and the output path are untouched. Measured speed profiles:

| log | `< 1.0 m/s` (direction logic IDLE) | `< 0.3 m/s` |
|---|---|---|
| 12-52 | 10.4 % | 8.4 % |
| 12-55 | 23.9 % | 14.0 % |
| 16-04 | 11.5 % | 7.8 % |
| 16-08 (crawl autosteer) | **34.0 %** | **31.3 %** |

* Below 1.0 m/s the direction logic is IDLE - pre-existing (`minSpeedForDirection`) and NOT
  changed by this fix.
* In the crawl-autosteer log 16-08 the FIX produced exactly ONE wrong window: **0.5 s**, at
  0.97 m/s, **1.7 s from the real manoeuvre**. BASE had none there - but BASE also never
  detected the real reversal in that log.
* **Still not detected:** a reversal performed ENTIRELY below 1.0 m/s (02.10 16-27 at 0.88,
  01.10 20-11 at 0.83, 01.10 20-18 at 0.80). Lowering `minSpeedForDirection` is the lever for
  that and was deliberately NOT touched here.

### Built

| file | change |
|---|---|
| `HeadingBridge.h` | `Config::standstillSpeedMps = 0.15f`, `Config::standstillMinMs = 300`; state `standstillSinceMs`, `dirReseedPending` |
| `HeadingBridge.cpp` | standstill tracking at the TOP of `updateDirectionState()` (before the speed gate, which is the whole point); the existing `!dirDevInit` seed branch became `(!dirDevInit \|\| reseed)`, plus a `LOG_NOTICE` when the re-seed changes the verdict |

Nothing else was touched: the witness conditions (`attDev >= 120` or no attitude) and the
`travelSolid` veto are unchanged, so the re-seed can only shorten the delay, never create a
verdict that the witness would not allow. `updateDirectionState()` is `FLASHMEM`, so RAM1 is
unchanged: `variables 159520`, `code 359764`, `padding 684`, `free for local variables 4320`.

`AiO_v26.5.1.hex` 18:22:58. In the field, a K-turn now shows
`HeadingBridge direction: REVERSE ... after a standstill`.

