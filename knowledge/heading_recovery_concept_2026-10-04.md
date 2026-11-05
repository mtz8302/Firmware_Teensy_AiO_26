# Heading skew 2026-10-04: the limiter, not the projection

Logs (firmware with `physicalBaselineM = 0.711`, `dualHeadingOffsetDeg = 88.3`):
`2026-10-04 5-45 0 hof bach bäume leicht schräg`, `5-49 k Bäume bach k`,
`5-59 0 Bäume Schleichfahrt autosteer`, `6-06 0 bach brücke brücke schleichfahrt`.
New tool: `tools/analysis/skew.ps1` (takes over from `askew.ps1`, see the caveat at the end).

## 1. The receiver and the offset are NOT the problem

On clean, straight, fully RTK-fixed frames (v >= 1.5 m/s, no reverse, |wheel| <= 5 deg,
11 consecutive frames qf = fixQ = 4, turn rate over that 1 s window < 2 deg):

| log | travel - att (p50) | p10 | p90 |
|---|---|---|---|
| 5-45 | 88.58 | 87.26 | 89.99 |
| 5-49 | 88.88 | 87.84 | 89.78 |
| 5-59 | 89.17 | 87.49 | 90.39 |
| 6-06 | 88.84 | 88.20 | 89.58 |

Constant within +-1 deg over four drives and two days. The mechanical offset really is
88.3 deg, the receiver's attitude is correct, and the machine does **not** crab: if it
crabbed by the measured skew, `travel - att` would have to be 88.3 + skew.

## 2. `psi` is the wrong one, and it knows it

`az_out = psi - dualHeadingOffsetDeg` is what AgOpenGPS receives and adds 88.3 back to, so
every degree of `psi` error is a degree the tractor runs diagonally. On the same clean
frames:

| log | travel - psi (p50) | p10 | psi_att (fw) p50 | gain p50 | rl_allowed p50 |
|---|---|---|---|---|---|
| 5-45 | -2.69 | -64.78 | +2.80 | 0.118 | 11.7 |
| 5-49 | +1.63 | -27.49 | -0.90 | 0.333 | 11.2 |
| 5-59 | -8.65 | -10.54 | +8.60 | 0.826 | 19.9 |
| 6-06 | -1.56 | -9.85 | +1.70 | 0.311 | 11.1 |

`psi_att` is the firmware's own pass-through check `psi - (att + offset)` = the innovation
of a **correct** measurement that arrives every 100 ms. It is as large as the skew: the
filter is told about its own error and does not use it.

## 3. The smoking gun: `rl_clip`

`applyMeasurement()` clips how far ONE measurement may pull `psi` to the physically
possible yaw rate (`kinematicRateBoundDegS()` = v*tan(|wheel|+slack)/L + 2 deg/s, raised by
an accepted path turn and by the receiver's own attitude rate). Splitting the clean frames
by that clip flag:

| log | clipped | skew when CLIPPED | skew when free |
|---|---|---|---|
| 5-45 | 10 % | **-71.23** (n 18) | -2.11 (n 157) |
| 5-49 | 46 % | **+4.11** (n 71) | +1.10 (n 82) |
| 5-59 | 33 % | **-9.67** (n 8) | -2.06 (n 16) |
| 6-06 | 17 % | **-7.81** (n 40) | **-0.10** (n 190) |

Every log: the skew lives in the clipped frames, and the un-clipped frames are at
-2.1 / +1.1 / -2.1 / -0.1 deg, i.e. essentially zero. This is a feedback loop - a large
error requests a large correction, the correction is clipped, the error stays, the next
frame is clipped again. Examples straight out of the log (6-06, v 2.5 m/s, wheel 1.4 deg,
roll 1.9 deg, wq 92):

```
 t     v  wheel  roll   yr    wq |   psi     att  travel  psi_att  gain
67.5  2.5    1.5   1.9   0.0   92 |   93.2   353.5    81.8     11.4  0.06
67.7  2.5    1.4   1.7   0.0   92 |   93.0   353.7    81.8     11.1  0.11
68.4  2.5    1.4   2.0   0.0   85 |   92.5   353.9    81.2     10.4  0.17
```

`att + 88.3 = 81.8 = travel` (correct), `psi` sits at 93 and moves ~0.8 deg/s while the
measurement asks for 10 deg. The model yaw rate is 0, so the model is not turning it - the
state simply cannot catch up.

**Causality note:** the clip is a SYMPTOM, not the cause. A large error requests a large
correction, the clip keeps the error alive, the next frame is clipped again. The cause is
that the two measurements which DO see the error - the attitude and the position-anchored
projection - are not allowed to move psi (sections 6 and 7).

## 4. Where the error is injected

Split by the model yaw rate on the same clean frames:

| log | |yr| <= 1 deg/s | |yr| > 1 deg/s |
|---|---|---|---|
| 5-45 | -0.77 (n 57) | -3.27 (n 118) |
| 5-49 | -27.01 (n 40) | +3.66 (n 113) |
| 5-59 | -8.65 (n 23) | +0.50 (n 1) |
| 6-06 | +0.16 (n 144) | -5.66 (n 86) |

The error appears together with the kinematic model, and `wq` (wheel angle reliability) is
2 ... 86 across these logs with p10 down to 2 and 9 - so in several phases the model runs
on a wheel angle that is not trustworthy. P1b disables the model below
`wheelQualityLowPct = 30`, but it stops the *injection*, it does not remove the error that
was already injected - that is left to the (clipped) correction.

`reseedBeyondDeg = 120` is the only escape hatch and a 10 deg state error never reaches it.
`rateLimitPersistMs = 1000` / `rateLimitPersistRelax = 4.0` is a first, weak version of the
same idea.

## 5. The roll is a separate, smaller issue

`roll_src` says the published roll is the dual-antenna value (1009 / 1641 / 2915 / 1665
frames) with a hold in 372 / 471 / 384 / 523 frames. The RAW dual roll blows up to
-79 ... +67 deg under trees, the published one stays inside -7.7 ... +11.3 deg, and

* published roll step p50 = 0.0 ... 0.1 deg, p99 = 0.5 ... 0.9 deg,
* the worst published steps (2.2 ... 11.3 deg) happen almost exclusively at
  **v = 0.00 ... 0.20 m/s**, i.e. standing, or in the frame of a `baseLen` rejection,
* while driving, |roll| is p50 1.2 ... 4.2 and p90 2.8 ... 5.1 deg where the operator says
  the ground only has 2 ... 3 deg.

At 2.2 m antenna height, `rollCorrectionDistance = sin(roll)*h` turns a 2 deg roll error
into 7.7 cm and a 4 deg error into 15 cm of lateral position shift - a **constant offset on
a slope, not a heading error**. So the roll is not what makes the tractor run diagonally;
it is a bounded second-order item (and one 43 cm event, standing, in log 5-45).

## 6. CORRECTED: the projection IS the better source (measured, `projqc.ps1`)

The first version of this document claimed "improving the measurement side does not remove
the skew". That was wrong, and it was an untested claim. `updateProjection()` logs its own
cross-track residual, so its heading measurement can be reconstructed exactly as the
firmware computes it (HeadingBridge.cpp:3157):

```
proj_meas = psi - atan2(proj_ctk_m, proj_rng_m)
```

Compared against the truth (the 0.5 s position chord), all driving, curves included:

| log | err_psi = travel-psi | err_att = travel-(att+88.3) | **err_proj = travel-proj_meas** |
|---|---|---|---|
| 5-45 | -1.45  (p10 -97.93, p90 17.03) | 0.33  (p10 -13.32, p90 7.83) | **0.13  (p10 -2.34, p90 2.14)** |
| 5-49 | -2.59  (p10 -27.76, p90 3.01)  | 0.85  (p10 -5.10,  p90 3.83) | **-0.20 (p10 -2.22, p90 1.61)** |
| 6-06 | -3.48  (p10 -10.23, p90 1.10)  | 0.42  (p10 -7.04,  p90 3.15) | **0.33  (p10 -1.89, p90 1.96)** |

And the decisive subset - only the frames where `psi` is more than 4 deg wrong:

| log | \|err_psi\| p50 | -> | \|err_proj\| p50 | \|err_att\| p50 | n |
|---|---|---|---|---|---|
| 5-45 | 6.02 | | **0.31** | -0.11 | 87 |
| 5-49 | 6.49 | | **0.15** | 1.09 | 93 |
| 6-06 | 9.44 | | **0.53** | 0.42 | 224 |

The projection is the best of the three, in curves as well as on straights, because it is
position anchored: a bad attitude frame does not move it. `|proj_ctk|` p50 23 ... 52 mm over
`proj_rng` p50 0.36 ... 0.66 m is exactly the measured skew (atan(23/570) = 2.3 deg,
atan(52/360) = 8.2 deg) - so it does SEE the error and reports the right number.

### Why it does not reach psi - two defects

**D1, architectural.** `updateProjection(now, speedMps, dtS, !status.attitudeTrusted)`:
the measurement is only handed to `applyMeasurement()` while the attitude is NOT trusted,
and in `update()` the attitude branch explicitly skips the fall-back (`else if (!projUsed)`).
In 5-49 and 6-06 the projection produced 272 and 378 good measurements and **0 % of them
were used** - `src = proj` is 0 % of the frames in both logs (13 % in 5-45, where the
attitude was lost often).

**D2, mechanical.** In `applyMeasurement()` the rate limiter uses

```
float dtUse = (dtRefS > 0) ? dtRefS : ((lastDtS > 0) ? lastDtS : 0.01f);
if (kind == 0) { dtUse = <measured interval between two ATTITUDE frames>; }
```

So the ATTITUDE (kind 0) gets its real interval (~0.1 s at 10 Hz), while the path sources
(kind 1/2/3 - COG, projection, IMU heading) keep `lastDtS` = the 100 Hz tick = 0.01 s. Their
allowed correction is therefore `allowed * 0.01` instead of `allowed * 0.1`, i.e. **10x too
small**: at `rl_allowed` = 11 deg/s a path measurement may move psi 0.11 deg per application
(1.1 deg/s) while the attitude may move 1.1 deg (11 deg/s). A 9 deg error needs 8 s through
the path, 0.8 s through the attitude.

### The ring buffer belongs to this item

`proj_rng` p50 = 0.36 ... 0.66 m because the projection resyncs onto the fix every time and
`projPathM` restarts at 0, so a measurement only exists once ~0.5 m has been travelled: every
2nd fix at 2.5 m/s, every 5th at 1 m/s, never at crawl. `proj_upd` 4.1 ... 8.0/s against
`fix_cnt` 10.0/s is the same observation from the counter side (the README says the two have
to match). A position look-back (ring buffer) makes the window available at EVERY fix with an
exact 0.5 m baseline, and `maxWindowM` (re-base only after the window has grown past the
limit) is the zero-RAM variant.

## 7. Concept: make the projection the permanent, position-anchored reference

Goal (operator framing, now supported by the measurement): `psi` must be anchored to the
**direction of travel** - that is the quantity AgOpenGPS steers on - and the attitude/IMU
become the high-bandwidth supplement instead of the master. Then "what comes from the
projection and what from live data" is a **weighting** question (the R of each
measurement), which is exactly what the Kalman filter is there for.

### Step A - let the projection measure while the attitude is trusted (D1)

`updateProjection(..., allowMeasurement = !status.attitudeTrusted)` and the
`else if (!projUsed) selectFallbackMeasurement(now)` in `update()` turn a measurement that
is accurate to 0.15 ... 0.53 deg exactly where psi is 6 ... 9 deg wrong into a pure
fall-back. Make it a permanent measurement.

Risk to manage, not a reason to keep it out: the projection integrates the MODEL, so in a
curve its residual carries the model's over-turn (measured 1.4 ... 2.6x on the wheel-angle
chain). The measurement is self-correcting (it measures the path error), but its R must say
so - the code already inflates R by the along-track residual
(`rUse *= 1 + |projLongM|/projPathM`), and the innovation gate plus the rate limiter stay.
The offline replay below has to show that the turn tracking does not get worse.

### Step B - give the path measurements their own time base (D2)

`applyMeasurement()` measures the interval between two ATTITUDE frames for kind 0 but
leaves `lastDtS` (0.01 s) for kind 1/2/3, although these are applied at the fix rate
(0.1 s). That makes their allowed correction 10x too small. Give every kind the interval
since the last application of THAT kind. Three lines, and it is the single most direct
reason a correct path measurement cannot remove a state error.

### Step C - a look-back window so the measurement exists at every fix

`proj_rng` p50 0.36 ... 0.66 m: the projection resyncs onto the fix and restarts
`projPathM` at 0, so a measurement exists only after ~0.5 m - every 2nd fix at 2.5 m/s,
every 5th at 1 m/s, never at crawl, and `proj_upd` runs at 4.1 ... 8.0/s against
`fix_cnt` 10.0/s. A position ring buffer (300 samples at 9.3 Hz = 32 s, see the ring-buffer
sizing note) or the zero-RAM `maxWindowM` re-base rule makes the window available at every
fix with an exact 0.5 m (RTK) / 1.0 m (float) baseline. At 300 m/h that is 2.69 m of
history - the buffer is never the limit for a 0.5 m gate.

### Step D - the residual case, and the log

Where no path measurement exists (fix loss, standstill, crawl below the buffer) the
attitude is the only source and a state error must still be removable - that is where the
rate-limit / recovery idea of the first version belongs, as a *secondary* item. To be able
to see all of this directly, log `proj_meas` (reconstructible today from `proj_ctk_m` and
`proj_rng_m`), `proj_innov` and the `dtUse` actually used by the limiter per kind.

### Plan

1. `tools/analysis/fuse.ps1`: offline replay of a fusion with BOTH measurements (attitude +
   reconstructed projection) over the 2026-10-03/04 logs, judged by the AgOpenGPS-relevant
   metrics that `filter_compare.ps1` already uses - skew p50/p90, TWITCH (steps > 1.5 deg
   per 100 ms on straight driving), FOLLOW (turn rate ratio), SWALLOW - plus the single
   decisive check: does the skew go away. No build.
2. If confirmed: build Step A + Step B (small, both config-gated), with the new log columns.
3. Then Step C (ring buffer look-back) for crawl availability and `proj_upd` == `fix_cnt`.
4. Separately: the roll item (section 5) - bound what reaches AOG while standing / after a
   `baseLen` reject.

## IMPLEMENTED 2026-10-05: Step B + Step A + the two ring quality fields

Build SUCCESS (`RAM1 code 359 988`, stack 4 288 free, `RAM2 177 600` unchanged - the record is
still 72 B, `static_assert` holds).

**Step B - the time base of the rate limiter, per measurement kind** (`HeadingBridge::applyMeasurement`).
`dtUse` now comes from `lastMeasMsByKind[kind]` (attitude 0, fix-to-fix/COG 1, projection 2, IMU 3),
i.e. the interval since THAT kind was last applied, instead of the 100 Hz tick. Verified: every
call site omits `dtRefS`, so the change takes effect for all kinds:
```
  896: (.., 0) attitude     3186: (.., 2) projection   3352: (..,1) fix-to-fix
 3372: (.., 3) IMU          3397: (..,1) COG
```
Consequence: the allowed correction for the path kinds grows by ~10x (rl_allowed ~13 deg/s against
the ~1.7 deg/frame the filter asks for) - the measured reason psi never converged although the
projection was 0.70 deg accurate where psi was 9.40 deg off. Attitude behaviour is unchanged
(0.02 ... 0.5 s window); the path kinds accept up to 1.0 s.

**Step A - the projection as a permanent measurement** (`Config::projAlwaysMeasure = true`,
set to false for the old behaviour): the call site is now
`updateProjection(now, speedMps, dtS, config.projAlwaysMeasure || !status.attitudeTrusted)`.
Its R stays adaptive and grows with the along-track residual in curves.

**Step C - the ring gives a path-anchored measurement at EVERY fix**
(`HeadingBridge::updatePathFromRing()`, `Config::ringPathMeasurement = true`).
`updatePositionWindow()` re-bases after every accepted window (`refArcM = 0`), so a path
measurement exists only near the end of each window and its baseline drifts from 0 to ~0.6 m.
The new function takes the SAME geometry from the ring - `positionHistory.lookback()` returns the
oldest sample at least `minDistM` away - so every fix gets a measurement over a CONSTANT, exact
baseline: 0.5 m on a fixed solution, 1.0 m on float (the heading uncertainty is sigma_pos /
baseline), baseline age <= 3 s, plus a speed witness (`|chord - v*age| <= 0.5*v*age + 0.4`).
Everything else reuses the existing conventions: `atan2f(E, N)`, the side-slip offset, the
180 deg mirror while reversing, the physical innovation limit, and the adaptive path R inflated
by `2*sigma_pos/baseline`.
**It runs only while the attitude is trusted** - the degraded phases belong to
`selectFallbackMeasurement()` (which has its own window); two measurements of the same geometry in
one tick would over-weight it.
Cost: +640 B FLASH, ITCM/RAM unchanged (the function is FLASHMEM, the ring accessors already are).
Visible in the log: `src` now mixes `att` and `fix2fix` on good frames (it names the measurement
that moved psi last), and `fixToFixUpdates` grows.

**Ring quality fields (0 byte):** `FixSample` gained `hdgSdX100` (receiver heading 1 sigma, 0.1 deg
steps) and `baselineCm` (solved baseline, cm) in the three bytes that were padding; both are RAW
receiver figures (0 = not provided, like `GNSSData::baselineLengthM`). Filled in
`PositionHistoryService` and appended to the `PH,` line as `hdg_sd_deg, base_cm`.
Measured justification (2026-10-04 logs): `uni_hdgsd` is live (p50 0.44 ... 0.57, p90 3.51) and
correlates with the actual attitude error (r = 0.24 ... 0.53); the solved baseline (p50 0.711 m)
is the early warning. Everything else the filters need (per-source noise, the delta-roll gate, the
availability) is DERIVABLE from the raw values already in the record - so no further fields.

### Corrections after the 20-35 field log (no RTK), 2026-10-05

Operator rule: "ohne rtk muss das genauso gut sein - die fix-to-fix Punkte liegen in RELATIVER
Lage gleich, nur die absolute Lage ist ohne rtk falsch." **Measured and confirmed** as the
perpendicular distance of the middle fix from the chord through its neighbours (straight frames):

| log | fixQ | n | p50 | p90 | RMS |
|---|---|---|---|---|---|
| 13-53 | 4 (RTK) | 561 | 0.5 cm | 1.4 cm | 3.7 cm |
| 17-28 | 4 | 859 | 0.5 cm | 1.6 cm | 2.0 cm |
| 20-35 | **1 (no RTK)** | 843 | **0.5 cm** | **1.8 cm** | **3.2 cm** |

The RELATIVE noise is the same with and without RTK; only the ABSOLUTE sigma differs (1.2 m in
20-35). Built in as a consequence:

* `Config::posRelativeNoiseM = 0.02f` replaces the absolute sigma in the R of the ring path
  measurement (`2*rel/baseline` = 4.5 deg at 0.5 m, 2.2 deg at 1.0 m). The old
  `2*sigma_abs/baseline` gave **137 deg** at a 1.0 m baseline and silently disabled Step C
  without RTK.
* one baseline length for every fix type (`minDisplacementForFixToFixM`), no doubling for float.
* safety net: `rGeom > 20 deg` -> no measurement and no source claim.
* `PositionHistoryService` pushes only records with `F_POS_OK` (in 20-35 the epoch counter
  advanced without a position and filled the ring with lat/lon 0 records).
* the 1 Hz raw-IMU line is a `Serial.printf` in the FLASHMEM method
  `IMUProcessor::logRawImuLine()`: columns `t_ms, yaw, pitch, roll, appliedRoll, appliedPitch,
  swapXY, yawRate`. `LOG_DEBUG` never produced it (boot lines did), and the inline printf cost
  192 B of ITCM - now 0.

Result: `RAM1 code 359 988`, `padding 460`, stack 4 288 free, `RAM2 177 600` - all unchanged;
FLASH +1.3 KB.

**What the 20-35 log already showed** (new build, no RTK, UM982 + BNO): `err_psi p50 = -0.18 deg`
against **-9.02 deg** on 13-53 (old build, RTK), while the ATTITUDE was broken in the same frames
(`err_att p10 = -87 deg`) - the path-anchored sources carried the heading. `src` mix: projx 25 %,
fix2fix 11 %, cog 21 %, att 12 %, deadreck 30 %. **Still to validate with RTK:** `skew.ps1` needs
`qf = 4` AND `fixQ = 4` frames, so the no-RTK log cannot confirm the skew numerically.
Also observed there: the UM982 machine runs with the AXIS SWAP ON (`currentData.roll` = the BNO's
pitch, boot line `Pitch=-27.4, Roll=-140.2`), which explains the old "slope 9.5 between IMU and
receiver roll" - a pitch-against-roll regression. Operator is checking that setting.


`status.lastMeasKind = (kind==0)?1 : (kind==1)?2 : (kind==3)?4 : 3` - a PROJECTION measurement
(kind 2) is therefore logged as **3 = "COG"** in the `meas` column, while the column comment
promises 5 = projected. Do not judge Step A from `meas`: use `src=proj` (from `status.source`)
and `proj_upd`. One line to fix, deliberately left untouched in this build so the acceptance run
has no second change under it.

### Acceptance test for the next field log

1. `projqc.ps1` on the new logs: `src=proj > 0 %` (was 0 %), `err_psi p50` from -9.0 deg to < 2 deg,
   `err_proj` stays < 1 deg.
2. `skew.ps1`: `travel - psi` p50 collapses; the clipped/not-clipped split loses its meaning
   (nothing is clipped 10x too tight any more).
3. `PH,` line now carries `hdg_sd_deg, base_cm`; `uni_hdgsd`/`baseDev` per fix are in the ring for
   the upcoming R policy.


`askew.ps1` reports the **circular mean** of `travel - az_out`, which the early-log outlier
(5-45: -94 deg in the first 90 s) drags from 85.4 to 79.0. `skew.ps1` uses percentiles and
splits by the clip flag, so use `skew.ps1` for the skew and `askew.ps1` only for the
`travel - att` offset.
