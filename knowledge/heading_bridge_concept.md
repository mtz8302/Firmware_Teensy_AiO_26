2 neue # Heading/Roll fall-back (coast) concept - dual-antenna GNSS without IMU

Status: analysis complete, module implemented, integrated and ACTIVE
(`HeadingBridge::Config::enabled = true`, `exposeToAog = true`);
`pio run -e teensy41` builds clean.
Data basis: the 2026-09-21 test drives (HBUF ring-buffer dumps, 10 Hz, with
attitude, COG, position deltas, VWAS angle, AOG commanded angle).

## 1. Baseline fact that everything else depends on

The antennas are mounted LEFT/RIGHT, therefore the UM982 `$GNHPR` heading is the
antenna BASELINE azimuth, not the vehicle heading:

| data set | n | `hdg(azimuth) - fix2fix(path)` | `fix2fix - COG` |
|---|---|---|---|
| road, 10 m/s | 922 | mean **-89.07 deg**, mean abs 89.07 | mean **+0.85 deg** |
| field, 2.5 m/s | 546 | mean ~-89 deg | ~-5 deg |

AgOpenGPS computes the body heading as `received + dualHeadingOffset`
(`GPS/Forms/UDPComm.Designer.cs:85`, applied to `headingTrueDual`, used as
`fixHeading` in `GPS/Forms/Position.designer.cs:701`). Since 2026-09-23 the
firmware uses **the same formula with the same value** (see §12), so both sides
agree for one number the operator enters in both places:

* the firmware publishes `body - dualHeadingOffset`, which in normal operation is
  the receiver's own azimuth (pass-through) - the bridge only replaces it while
  the attitude is missing,
* the ~1-2 deg residual (measured 89...92 instead of 90) is the mechanical
  mounting error plus side slip; it is what `dualHeadingOffsetDeg` (and AOG) is
  trimmed to.

## 2. Failure mode A: the receiver zeroes the attitude (the drop-out)

The UM982 does **not** go silent. It keeps sending at 10 Hz with the whole
attitude field zeroed while the position keeps its RTK fix:

```
$GNHPR,182639.40,000.0000,000.0000,000.0000,0,0,0.00,0000*6E
         time      heading   pitch     roll    QF sat
```

Raw lines captured (HeadingMonitor event lines in
`Log virtual WAS 2026-09-21 19-50 holperstrecke unter Bäumen AugustHild.txt`):

| time | duration | frames | QF before -> after | HPR raw (tail) |
|---|---|---|---|---|
| 00:38:46.192 | 900 ms | 9 | 4 | `182639.40,000.0000,000.0000,000.0000,0,0,0.00,0000*6E` |
| 00:38:48.002 | 90 ms | 1 | 4 | `182641.20,...,0,0,0.00,0000*67` |
| 00:38:55.592 | 410 ms | 4 | 5 | `182648.80,...,0,0,0.00,0000*64` |
| 00:38:57.592 | 410 ms | 4 | 5 | `182650.80,...,0,0,0.00,0000*6D` |
| 00:39:01.792 | 300 ms | 3 | 5 | `182655.00,...,0,0,0.00,0000*60` |
| 00:39:02.392 | 610 ms | 6 | 5 | `182655.60,...,0,0,0.00,0000*66` |
| 00:39:03.592 | 500 ms | 5 | 5 | `182656.80,...,0,0,0.00,0000*6B` |

Also seen on the road (`straße heading Feuerwehr a.txt`): one 600 ms and one
2400 ms event at 4.8 ... 10.2 m/s.

Frame counts per log (HBUF samples merged on their absolute timestamps):

| log | samples | qf=0 frames | loss events | loss time | travel during loss |
|---|---|---|---|---|---|
| holperstrecke unter Bäumen (19:50) | 650 (65 s) | 41 (6.3 %) | 8 | 4.10 s | 0.32 ... 2.32 m each |
| straße heading Feuerwehr | 960 (102 s) | 30 (3.1 %) | 2 | 3.00 s | 6.2 m and 12.2 m |
| 2x durch Halle (19:43) | 952 (106 s) | 340 (**35.7 %**) | 5 | - | indoors the attitude is unusable a third of the time |
| start in Halle (18:57) | 431 | 431 (100 %) | 1 | - | dump holds the boot state only |

During every one of these events `fixQuality` stayed 4 (RTK) and VTG kept
delivering a valid COG - the position and the course over ground survive the
attitude loss, which is the basis of the bridge.

### 2.1 UM982 jumps, F9P drifts (operator observation, confirmed in the data)

The last frames before a dropout are healthy and calm, then the sentence is
hard zero - no fade-out:

```
t        dPsi    qf fixQ sats hdop  hdg      roll
2325202  +1.15   5   4    8    0.7   76.75   -11.38
2325302  -1.26   5   4    6    0.7   75.49    -5.42
2325402 -75.49   0   4    0    0.9    0.00     0.00   <- hard zero
```

The 40 ... 60 deg artefacts are equally unannounced (three preceding frames with
|dPsi| 0.5 ... 6.9 deg, then the jump) and arrive in clusters (multipath
episodes): 2325002/2325102, 2338102/2338202/2338302, 2340502/2340602/2340802.

Consequence for the design: **on the UM982 the QF/zero test is sufficient** - the
plausibility gates exist for the value outliers that carry QF 4/5. The
baseline-length test (u-blox F9P manual) is aimed at the **F9P** moving-baseline
setup, where the operator reports drifting, implausible values at QF 4/5. $GNHPR
has no length field, so that test cannot run on the UM982 - and does not need to.

Effect in the firmware today: `dualHeading = 0` and `dualRoll = 0` go into PAOGI
unvalidated -> AOG's `fixHeading` = 90 deg while the machine travels at ~165 deg,
and AOG's roll correction (`sin(roll) * antennaHeight`) jumps by up to ~1 m.


## 3. Failure mode B: garbage attitude frames that NOTHING flags

1.8 ... 2.9 % of the frames **carrying QF 4/5** hold a broken attitude while the
trajectory stays perfectly straight:

| data set | |dPsi| > 5 deg / 100 ms | largest step | roll in the same frame |
|---|---|---|---|
| road (10 m/s) | 27 of 927 (2.9 %) | 44.6 deg | p99 19.6 deg step, max 58.9 deg step |
| field (2.5 m/s) | 12 of 650 (1.8 %) | 62.1 deg | +/-47 ... 62 deg absolute |

Example (`holperstrecke unter Bäumen`, HPR buffer):

```
t=2340602  dt=100  dPsi=+62.13  v=2.23 m/s  cog=150.08  hdg_az=117.47  roll=+11.09  traj=146.3
t=2340802  dt=100  dPsi=-57.66  v=2.57 m/s  cog=126.38  hdg_az= 52.43  roll=+25.03  traj=131.8
```

The trajectory direction (from the position stream) stays at 146/132 deg and the
COG at 150/126 deg, so the heading really did not move - the receiver's baseline
solution did. Heading and roll move TOGETHER because they come out of the same
broken solution; that correlation makes a roll test a free heading test.

These frames pass `WheelAngleFusion::headingQualityMask`, feed the VWAS heading
rate term and are published to AOG. They are the "Heading und Roll schwanken
stark" symptom.

## 4. What survives an attitude loss (measured)

`fix2fix` = direction from the position deltas over a +/-0.5 s window;
body = HPR azimuth + 90 deg.

| comparison | data set | mean | mean abs | p90 abs | during loss |
|---|---|---|---|---|---|
| COG - path | road | -0.05 | **0.76** | 1.85 | 1.17 / 2.63 |
| COG - path | field | 0.38 | 6.53 | 15.17 | 10.52 / 20.45 |
| body - path | road | 1.20 | 2.82 | 3.65 | - |
| body - path | field | 6.15 | 7.07 | 11.94 | - |

Kinematic model `omega = v * tan(delta) / L`, fitted on outlier-gated intervals
(field data set, real turning, 2.5 m/s):

| T | n | correlation | R2 | OLS L_eff | TLS L_eff | residual RMS |
|---|---|---|---|---|---|---|
| 1 s | 310 | 0.943 | 0.890 | 3.01 m | 2.89 m | 4.66 deg |
| 2 s | 325 | 0.954 | 0.911 | 3.07 m | 2.97 m | 7.49 deg |

`L_eff = 2.9 ... 3.0 m` against a nominal wheelbase of 2.14 m means the model
tracks only 0.71 of the geometric yaw rate with the current VWAS angle - that
scale has to be calibrated, and the bias state absorbs the rest.

**Open-loop dead reckoning is not usable** (both data sets, no correction):

| model | field | road |
|---|---|---|
| kinematic only (65 ... 102 s) | RMS 85.8, p50 63.3 deg | RMS 99.8, p50 82.3 deg |
| COG only | RMS 22.2, p50 5.75, p90 18.1 | RMS 6.6, p50 1.82, p90 4.2 |
| FUSED (kinematic + COG + bias state) | RMS 5.8, p50 3.39, p90 8.4 | RMS 6.6, p50 2.17, p90 4.7 |
| FUSED, no attitude measurement at all | p50 5.98 (p90 78 = gate losses) | RMS 7.0, p50 2.79, p90 5.7 |

A residual wheel-angle error of 1 ... 2 deg is 1.5 ... 8 deg/s of false yaw rate,
so the bias state is not optional - it is the difference between "works for
minutes" and "drifts away in 30 s".

## 5. Model verdict (decision basis)

| model | short term (<= 0.5 s) | long term | verdict |
|---|---|---|---|
| freeze / hold | p50 1.3 deg, p90 12 deg at 0.5 s (16.5 deg of real heading change in one 0.9 s event) | drifts with yaw | roll only, last fallback |
| kinematic v*tan(d)/L | 1.5 ... 8 deg/s without a bias fix | unusable alone | prediction half of the filter |
| COG (VTG) | p50 1.8 (road) / 5.8 (field) | no drift | correction half of the filter |
| fix-to-fix | p90 3.7 deg road, 3 ... 11 deg field, needs >= 0.5 m travel | no drift | preferred correction while moving |
| **fusion (kin + COG/fix2fix + bias)** | **p50 2.2 ... 3.4 deg** | **no drift over 65 ... 102 s** | **the bridge** |


## 6. Thresholds derived from the data

| quantity | threshold | basis |
|---|---|---|
| QF whitelist | 1,2,4,5,8,9 | 0/6/7 are "not a measurement" |
| all-zero sentence | hdg==0 && roll==0 && qf==0 && sat==0 | observed verbatim |
| roll envelope | abs(roll) <= 25 deg | p99 36.5 deg measured on a level body |
| roll step | <= 8 deg / 100 ms | p95 5, p99 19.6, max 58.9 |
| heading step | <= 5 deg + 12 deg per (m/s * blind time) | omega_max = v*tan(delta_max)/L_eff |
| COG cross-check | disagreement <= 25 deg when v > 1.5 m/s | measured mean 1.3, p90 4.5 deg |
| baseline length | abs(solved - reference) <= 5 % | F9P drift test; reference learned from the receiver (Config::physicalBaselineM = 0) or entered |
| HPR gap | max(150 ms, 3 x measured sentence period) | rate independent (10 Hz F9P, 20 Hz UM982) |
| BRIDGE budget | 2000 ms / 15 m | worst measured 2400 ms / 12.2 m |
| COAST budget | 0 / 0 = unlimited while the position is valid | operator requirement 2026-09-22 |
| DEADRECKON budget | 10000 ms / 50 m | the only phase that can drift |
| state hysteresis | 2 consecutive rejected frames | one outlier must not toggle LIVE/BRIDGE |
| position step | <= 2.0 * v * dt + 0.15 m | everything else is a position spike |
| window straightness | net / arc >= 0.90 | 0.9997 for 0.5 m on a 6 m radius turn |
| HDOP for the window | <= 3.0 | 0 disables the test |
| wheelbase L | from vehicle.xml (setVehicle_wheelbase), default 2.50 m | one source of truth, pushed via setWheelbaseM() |
| steerScale | default 1.0, measured recommendation 1.38 | L_eff = L * steerScale; reported, never auto-applied |
| log line length | <= ~115 characters incl. prefix | log viewer / USB console / syslog readability |

### 6.1 What the gates do on the real data (replayed offline)

Frame is called an "artefact" when the vehicle heading jumps by more than
5 deg / 100 ms. `tools/heading_bridge/validate_gate.ps1`.

| data set | artefacts | rejected by the gates | of those: real artefacts detected | missed | rejected with a preceding QF fault |
|---|---|---|---|---|---|
| field (2.5 m/s) | 37 / 650 | 84 | **36 (97 %)** | 1 | 41 |
| road (10 m/s) | 31 / 960 | 73 | **28 (90 %)** | 3 | 30 |

The value gates (roll envelope, roll step, heading step, COG cross-check) reject
34 frames in the field set and 43 on the road BEYOND what the QF rule already
catches - and those frames are visibly broken, not random:

| data set | rejected frames | kept frames |
|---|---|---|
| | abs(dPsi) p50 / abs(roll) p50 | abs(dPsi) p50 / abs(roll) p50 |
| field | 12.6 deg / 13.5 deg | 0.76 deg / 1.96 deg |
| road | 5.7 deg / **28.9 deg** | 0.46 deg / 2.73 deg |

A 28.9 deg median roll while driving straight on a road is a receiver attitude
blow-up, and it carries QF = 4/5 - i.e. today it reaches AgOpenGPS.

### 6.2 Position quality gate (operator requirement 2026-09-22)

`tools/heading_bridge/validate_position_gate.ps1` replays the step test and the
window test. Spikes found: 0 in the field set, 0 on the road, **15 of 950 steps in
the hall set** (the hall also has 36 % invalid attitude, so the position stream is
disturbed there). Effect on the direction error against the independent COG:

| data set | single-step direction (ungated) | 0.5 m window with the gates |
|---|---|---|
| road | p50 0.38 deg, p90 1.66 deg, max **154.5 deg** | p50 0.54, p90 1.99, max **8.4 deg** |
| field | p50 2.37, p90 6.46, max 21.4 | p50 4.59, p90 15.99 (chord bias while turning) |

The window direction is the chord over its length, so on a curving bumpy track it
lags the instantaneous COG by a few degrees - that is geometry, not a fault. What
matters is the maximum: the 154 deg single-step glitch disappears. Implemented
details that the data forced:

* a failed step test drops the *reference*, not only the sample - otherwise the
  spike stays in the window and bends the direction until it ages out,
* the arc accumulator must sum the STEP lengths, not the (growing) range to the
  reference; summing the range gives net/arc ~ 2/(N+1) and would reject every
  window (caught while validating, fixed),
* after a fix-quality gap the last seen fix is kept as the reference for the step
  test, so the first fix after the gap is not the one unchecked step (this is
  exactly where the 164 deg hall glitch appeared).


### 6.1 What the gates do on the real data (replayed offline)

Frame is called an "artefact" when the vehicle heading jumps by more than
5 deg / 100 ms. `tools/heading_bridge/validate_gate.ps1`.

| data set | artefacts | rejected by the gates | of those: real artefacts detected | missed | rejected with a preceding QF fault |
|---|---|---|---|---|---|
| field (2.5 m/s) | 37 / 650 | 84 | **36 (97 %)** | 1 | 41 |
| road (10 m/s) | 31 / 960 | 73 | **28 (90 %)** | 3 | 30 |

The value gates (roll envelope, roll step, heading step, COG cross-check) reject
34 frames in the field set and 43 on the road BEYOND what the QF rule already
catches - and those frames are visibly broken, not random:

| data set | rejected frames | kept frames |
|---|---|---|
| | abs(dPsi) p50 / abs(roll) p50 | abs(dPsi) p50 / abs(roll) p50 |
| field | 12.6 deg / 13.5 deg | 0.76 deg / 1.96 deg |
| road | 5.7 deg / **28.9 deg** | 0.46 deg / 2.73 deg |

A 28.9 deg median roll while driving straight on a road is a receiver attitude
blow-up, and it carries QF = 4/5 - i.e. today it reaches AgOpenGPS.

Note on the return to LIVE: there is no blend ramp and none is wanted. The filter
runs at 100 Hz on the wheel angle, so its state is already where the vehicle is
when the next valid frame arrives; the measurement update (K = P / (P + R)) turns
the return into a correction instead of a step. The roll channel is the one place
that needs explicit smoothing (LPF while live, hold then fade while bridged),
because a roll step moves AOG's position by up to 1 m.

One edge case found by tracing the state machine against the data: after a
rejection, the step gates must NOT be applied to the first recovery frame -
otherwise the restored heading looks like a jump and the gate would never
re-open. The implementation therefore skips both step tests while
`consecutiveRejects > 0` and measures the heading-step interval from the last
TRUSTED frame.

## 7. Open points / decisions needed

1. **Timeout action towards AOG** - PAOGI has no free "heading invalid" field.
   Implemented options: keep the estimate, set `fixQuality = 0` (AOG's own
   invalid logic, but it also gates section control), or stop sending PAOGI
   (AOG's GPS watchdog). Default is `LAST_VALID_QF0`.
2. **L_eff calibration** - 2.95 m is fitted from this machine and these tyres. It
   has to be re-checked and stored per machine; a full-lock circle is the cheapest
   measurement.
3. **CAN/wheel speed** - the bridge currently uses the GNSS velocity. A wheel or
   CAN speed would let it survive a total GNSS outage (the module already accepts
   one as a parameter).
4. **Web UI / EEPROM** - the config lives in the module only. Enable flag, L_eff,
   mount offset and the timeouts belong in the device settings page and the
   config store.

## 8. Implementation plan (files)

| step | file | change | risk |
|---|---|---|---|
| 1 | `lib/aio_navigation/HeadingBridge.h/.cpp` | new module (done) | none, `enabled=false` |
| 2 | `lib/aio_navigation/GNSSProcessor.h/.cpp` | VTG arrival counter `getVtgAgeMs()`; `GNSSData::baselineLengthM` fed from RELPOSNED `baseRelL` (done) | none |
| 3 | `lib/aio_autosteer/AutosteerProcessor.cpp` | `headingBridge.init()` in `init()`, `headingBridge.update(0.01f, steerAngleDeg)` in the 100 Hz `process()` next to `headingMonitor.update()` (done) | none while disabled |
| 4 | `lib/aio_navigation/NAVProcessor.cpp` | `formatPAOGIMessage()`: heading = `headingBridge.getAzimuthDeg()`, roll = `headingBridge.getRollDeg()` when `exposeToAog`; plus the timeout action (skip the message / force fixQuality 0) | BEHAVIOUR CHANGE - validate on a test drive |
| 5 | `lib/aio_autosteer/WheelAngleFusion.cpp` | use the bridged vehicle heading instead of `gpsData.dualHeading` when enabled, and weight the straight-path gate with `getHeadingSigmaDeg()` instead of the binary `headingQualityMask` | behaviour change, keep the mask as the second gate |
| 6 | `lib/aio_system/SimpleWebManager.cpp` + `web_pages/VWASConfigPage.h` | `/api/vwas/status` carries the bridge diagnostics and the VWAS page shows them, including the measured antenna mount offset to enter in AgOpenGPS (done) | UI only; JSON capacity raised 1600 -> 2400 |
| 7 | `lib/aio_system/...` config store | expose enable / L_eff / mount offset / budgets / physical baseline in the settings page and EEPROM | UI only |
| 8 | `tools/heading_bridge/*.ps1` | HBUF extraction, L_eff fit, model comparison, attitude-gate replay, position-gate replay | none |

Verification during step 4: the `HBRIDGE` NOTICE line prints state/source, sigma,
bias, age, distance, the raw attitude, COG, fix-to-fix, range, straightness, the
reject reason, position faults, the measured mount offset and the baseline
deviation at 2 Hz, so a test drive under trees reproduces exactly the tables of
sections 2, 3, 6.1 and 6.2.

## 9. Open items

1. **steerScale calibration** - the module measures and reports it (RLS estimator,
   `status.steerScaleEstimate`, web UI + `sc=` in the HB2 log line) but does not
   apply it automatically. After a test drive with real turns the value can be
   entered; 1.38 is the value measured on this machine.
2. **Wheel/CAN speed** - `update()` already accepts it as a parameter; wiring it
   would let the bridge survive a complete GNSS outage in the dead-reckoning
   phase.
3. **UM982 baseline length** - not needed (its failure is a hard zero + QF 0), and
   not available in $GNHPR. If the F9P path is ever used with a Unicore receiver,
   a sentence carrying the length has to be added.
4. **Sigma weighting in the VWAS** - the VWAS now takes the bridged heading, but
   still gates it with the binary `headingQualityMask`; `getHeadingSigmaDeg()`
   would allow a continuous weighting of the zero observer.

## 12. Heading conversion: ONE offset, identical to AgOpenGPS (2026-09-23)

The 2026-09-23 logs exposed a 180 deg error, and the first attempt to fix it
introduced a second constant that the operator could not see. Final design -
one number, same formula on both sides:

```
AgOpenGPS : body      = received + dualHeadingOffset    (UDPComm.Designer.cs:85)
firmware  : body      = az_recv  + dualHeadingOffset    <-- the same value
firmware  : published = psi      - dualHeadingOffset
```

In normal operation (`psi = az_recv + offset`) the published value is exactly the
receiver's own azimuth - a pass-through; the bridge only replaces it while the
attitude is missing. Nothing is corrected silently:

* `dualHeadingOffsetDeg = +90` is the **generic** athwart mounting (magnitude), and
  the SIGN is the mounting itself: swapping the two antennas changes it (+90 <-> -90,
  i.e. 180 deg apart, not a small trim). MEASURED on this machine: +92...+95 with the
  antennas in one order (12:16 log, offset -92), -86 after swapping them back
  (19:57 log, offset +86) - so the 19:57 mounting is the one the machine keeps, and
  the value belongs at +90 there (and in AgOpenGPS). Deliberately not trimmed to
  86: the real error differs per machine and stays a measurement (`mnt` field, web
  UI), not a hidden constant.
* Symptom of a wrong SIGN (measured 19:57 log, 1024 of 1782 frames): the heading
  AOG receives points backwards, so AOG's own reverse detection
  (`Position.designer.cs:738-754`: direction of travel vs. `fixHeading`, `isReverse`
  when the difference exceeds 2 rad = 114.6 deg) reports REVERSE while driving
  forward, and the bridge's own cross-check throws the attitude away 1024 times
  (`cogX`), turning LIVE into COAST for more than half the drive.
* The web page shows both: the value in use and the measurement's recommendation,
  and flags it when they differ by more than 3 deg.

Signed convention behind it: `mnt` measures `az_recv - direction of travel` and is
therefore the *negative* of the AOG offset. On this machine AOG's `+90` was only
a compensation of the old 180 deg double-application (`-92 + 180 = +88`, which is
why `+88` felt better than `+90`); with the corrected formula both sides belong at
the measured value.


Measured on the 23-09 logs: raw azimuth noise p90 2.5 deg/100 ms (3x the 21-09
road data), COG p90 6 ... 9 deg/100 ms (the noisiest source, used 20 % of the
time), psi followed the raw value almost 1:1 (p50 1.00/p90 8.7 deg per 0.5 s
against 1.00/9.0 raw) - the filter was passing the measurement noise through
because R was optimistic by 2-3x.

| change | why / expected effect |
|---|---|
| adaptive R from the innovation variance, nominal values 4 deg (attitude) / 8 deg (path) | jitter ~K*sigma; K drops from 0.62 to ~0.3, factor 3 difference between days handled automatically |
| COG circular average over 3 samples | the fall-back source with p90 6 ... 9 deg; only used while the attitude is gone, so 0.3 s lag is free - sqrt(3) less noise |
| side-slip estimate (EMA of body - COG while LIVE) removed from COG/fix-to-fix | no step when the source switches; path vs body is a real few degrees |
| standstill hold (< 0.3 m/s, small q) | a standing machine does not yaw; the COG moves up to 20 deg/0.5 s at rest |
| heading/roll step gates speed PROPORTIONAL + noise term | physics is omega = v*tan(delta)/L (linear in v); roll step p99 19.6 deg at 10 m/s against p95 5 deg at 2.5 m/s - observed, not assumed. Quadratic would be wrong |
| fix-to-fix R from window length (sigma_pos/range) | longer window trusted more, automatically |
| FORWARD PROJECTION of the position (100 Hz, with the wheel angle) | cross-track residual = heading error without chord lag; along-track residual = speed/scale error. Preferred fall-back (`Source::PATH_PROJECTED`) |
| speed scale `vScale` learned from the along-track residual, applied to the MODEL only | omega is proportional to v, so a 3 % speed error is a 3 % heading-rate error; PAOGI keeps the receiver's own speed |
| projection R scaled by (1 + along/path) | a large along-track residual means the model does not describe the interval, so the cross-track part is less trustworthy |
| steerScale: self-learned, applied, stored in EEPROM (block 1600, marker 0xA5) | it is a MACHINE CONSTANT (tyres/geometry/load); operator decision: no manual calibration |
| roll LPF tau 0.35 -> 0.8 s | AOG turns roll into a POSITION shift (sin(roll)*antennaHeight): a 7 deg step at 3 m = 0.37 m of jumping tractor |
| 10 Hz raw frame log `HR,...` direct to USB (never blocking) | full data record at 10 Hz = 2 kB/s = 0.2 % of the USB CDC capacity |
| HB1/HB2, VWAS diag/zero/encoder/ctrl -> 0.5 Hz | the raw line carries the data now; INFO bucket freed for real events |
| HeadingMonitor ring buffer +psi100 +ctk10, BUF_SAMPLES 480 -> 320 | dumps show the published heading and the projection residual; RAM1 free for locals 9856 -> 12736 B |


Step 1-6 of section 8 are DONE, the module is **active**
(`Config::enabled = true`, `Config::exposeToAog = true`):

* PAOGI publishes `headingBridge.getAzimuthDeg()` / `getRollDeg()`, and the
  `TimeoutAction` is implemented (`STOP_PAOGI` = no message at all,
  `LAST_VALID_QF0` = fix quality forced to 0, `KEEP_BRIDGED`).
* `WheelAngleFusion` uses the bridged body heading for its curvature estimate and
  the bridged roll, instead of the raw attitude.
* The wheelbase is pushed from the VWAS config (vehicle.xml import) at 100 Hz,
  so L has exactly one source.
* Diagnostics: `HB1` (state) at 2 Hz, `HB2` (evidence) only while degraded; both
  <= ~115 characters. Web UI: bridge state/source, sigma, bias, mount offset
  (+ the value to enter in AOG), baseline length/reference/deviation, position
  faults, gap threshold, steer-scale recommendation.
* Build: `pio run -e teensy41` SUCCESS, RAM1 free for local variables 10880 B.

## 13. The fix signal: one shared stamp, two consumers (fixed 2026-09-23)

`gpsData.lastUpdateTime = millis()` is written by EVERY sentence that carries a
position - GGA (GNSSProcessor.cpp) and GNS on this machine, KSXT and INSPVA/INSPVAXA
on other receivers. With two or more of them enabled the field moves 2...4 times per
satellite epoch, so it can never mean "a new fix arrived".

HeadingBridge used it as exactly that, in a single member `lastPosStamp` shared by
two consumers:

```
update()                                    HeadingBridge.cpp
  676:  updatePositionWindow(now, speedMps);   // runs FIRST
            914:  if (d.lastUpdateTime == lastPosStamp) return;
            978:  lastPosStamp = d.lastUpdateTime;      <-- fix consumed
  687:  updateProjection(now, speedMps, dtS);  // runs SECOND
           1028:  const bool newFix = (d.lastUpdateTime != lastPosStamp);
                  -> always FALSE, the stamp was just set
```

Measured in the 2026-09-23 `HR` logs: over 160-170 s of driving, `proj_ctk`,
`proj_rng`, `proj_along`, `sc` hold ONE distinct value, while the position keeps
arriving (lat changes 1662x) and the vehicle turns. The projection resynchronised
~once per drive; when one slipped through after a position gap, the interval was
248 m long and the cross-track residual 86 m.

Consequences beyond the projection itself: `sc` (steer scale) sat at its 0.60 clamp,
`vScale` stayed exactly 1.000, and the adaptive measurement noise worked on a
freeze-frame. The 2026-09-23 "sc is stuck" observation is therefore not a latency
artefact - the estimator never received a sample at all.

Fix (V4): `GNSSProcessor::notePositionFix()` counts satellite epochs, deduplicated
over the GPS time of day (`gpsData.fixTime` + `fixTimeFractional`, identical in all
sentences of one epoch):

* exposed as `getPositionCount()` / `getPositionAgeMs()`,
* READ by consumers, never consumed - the window and the projection keep their own
  copy (`lastWindowFixCount`, `lastProjFixCount`),
* independent of the receive jitter that sits on `millis()`.

Verified: `v4_dedup_check.ps1` replays 2 s at 10 Hz with 4 sentences per epoch -
old counting 40/s, new counting 10/s.

Because the estimators now receive ~10 samples/s instead of ~1 per drive, applying
their results would change the machine's model within seconds. They are therefore
PASSIVE first (`Config::applyLearnedSteerScale` / `applyLearnedSpeedScale` = false):
the estimates are computed, logged and shown, the model and the EEPROM keep the
configured values. `HR` gained `proj_upd, fix_cnt, pos_age_ms, sc_n, sc_spread,
vsc_n` so the next drive can be scored directly.

## 14. Direction of travel: forward / reverse (operator rule, 2026-09-23)

OPERATOR RULE, verbatim: "bei Rueckwaertsfahrt muss das heading trotzdem nach vorn
am Fahrzeug zeigen. wenn ich gerade vor und zurueck fahre aendert sich heading
nicht." The heading that leaves the module is the vehicle FRONT axis - always.

Consequences, and they are not symmetric:

* the ATTITUDE heading satisfies the rule by construction (the antennas are bolted
  to the front axis, the baseline does not turn around when the driver reverses),
* the FALL-BACK sources do NOT: COG, fix-to-fix and the forward projection measure
  where the machine MOVED. They are mirrored by 180 deg while reversing, which is
  exactly what AgOpenGPS does in its single-antenna path
  (`Position.designer.cs:500`, `newGPSHeading += Math.PI`) so that everything
  downstream works with the front axis.

AgOpenGPS does its own reverse detection and needs no "R" from us - but it needs a
heading that is the front axis, otherwise its decision inverts. Its two forms:

| source | file:line | rule |
|---|---|---|
| dual antenna | `Position.designer.cs:738-754` | travel distance >= `dualReverseDetectionDistance` (0.25 m), `delta = |pi - ||travel - heading| - pi|`, `isReverse = delta > 2 rad` (114.6 deg) |
| single antenna, no IMU | `:473-523` | same delta, EMA 0.2/0.8, `\|filtered - raw\| > 0.5 rad` (28.6 deg) => `isChangingDirection` => `goto byPass` (decision paused), then `isReverse = filtered > 1.57 rad` (90 deg), and `newGPSHeading += pi` |

The firmware ports the conservative (second) form: `minSpeedForDirection`,
`reverseDetectDeg` 90, `directionChangeDeg` 28.6, `directionFilterAlpha` 0.2, with
the pause while the angle sweeps through 90 deg. What the 180 deg mirroring affects:

* `selectFallbackMeasurement()`: COG and fix-to-fix get `+ 180 deg` while reversing
  (after the side-slip correction, so the switch stays step-free),
* `updateProjection()`: the forward projection integrates the displacement against
  `psi` with the sign of the travel direction; `projPathM` stays a travelled LENGTH,
* `updateMountOffset()`: the reference is the FRONT axis (path + 180 deg while
  reversing). Without that the measured mounting offset would swing by 180 deg on
  every reverse manoeuvre; with it, `mnt` is a constant and the AOG value is `-mnt`,
* `evaluateAttitudeFrame()` gate 6: the COG cross-check is now direction aware. A
  one-sided `|heading - cog| < 25 deg` test rejects the attitude on EVERY reverse
  frame - measured 2026-09-23 19:57: 1024 of 1782 frames thrown away as `cogX`
  (LIVE -> COAST for most of the drive). It now measures against the side the
  current direction implies, and during a direction change it accepts whichever
  side is nearer (AOG's byPass in gate form).

Diagnostics for the field test: `HR` columns `rev` (1 = reversing), `dirchg` (1 =
direction change in progress), `dirdev` (filtered angle heading <-> travel); web UI
shows the direction, the angle and a mounting-sign check. The sign check compares
the configured offset with the measurement (`offset` must be `-mnt`): equal signs
mean the antennas sit the other way round than configured, i.e. the heading sent to
AOG points backwards - which shows up in the field as "AOG displays reverse while I
drive forward" and is reported as a warning, never corrected silently.

Measured origin of the rule (2026-09-23 19:57 log, antennas swapped back):
`mnt` p50 = -84.6 deg (offset must be +86...+90, configured was -90), 0 of 1782
frames with v < -0.3 m/s - the machine never actually drove backwards, the
"reverse" on the display was the inverted heading.

Correction to that note (2026-09-23 20:19 log): `v` in `HR` is the SPEED
MAGNITUDE (VTG has no sign), so "v never negative" says nothing about the
direction. With `rev` logged properly the same drive shows 281 reverse frames
(`dirdev` p90 = 174 deg, `rev` p50-speed 0.41 m/s - slow reversing), 0 heading
jumps across the forward<->reverse transitions, and 0 missed / 3 wrong direction
decisions.

## 15. Steering-scale estimator: window instead of single step (2026-09-23)

`HR` showed `sc_n = 0` over the whole 20:19 log (244 s of driving) - the estimator
never received one sample. Cause: the sample condition required an accumulated
geometric turn of > 5 deg, but the accumulator was RESET at every attitude
measurement (10 Hz), and one 100 ms step only produces
`57.3 * v * tan(delta) / L * 0.1 s` = 1.0...1.5 deg even in a hard turn at 1.3 m/s.
The threshold was unreachable by construction.

Fix: the turn is now COLLECTED across measurements (`scaleCollecting`,
`scaleStartPsi`, cap `steerScaleWindowMaxMs` = 3 s) and compared against the
heading movement over that whole window - the same 1 s window the offline fit used
to produce the original 1.38 factor. Path measurements end a running window, so
only trusted attitude data feeds the estimate.

## 16. Steering angle from the driven circle - measured, then parked (2026-09-23)

Tool: `tools/heading_bridge/steering_angle_from_circle.ps1` (steady turns only, LIVE
frames, >= 1 m/s, physical floor `L/tan(maxAngle)`, `-ForwardOnly` to skip reversing).
It needs no calibration drive - it uses the tightest circle the machine actually
drove: `R = v / omega`, `delta = atan(L / R)`, and compares that with the wheel angle
the VWAS reported at the same moment. The ratio is exactly what a counts-per-degree
error looks like.

Results with the real wheelbase 2.14 m (operator 2026-09-23):

| log | R_min | delta geometric | VWAS reported | ratio |
|---|---|---|---|---|
| `12-16 autosteer 88` | 2.54 m | 40.10 deg | 40.24 deg | **0.997** (cpd correct) |
| `20-19` | 2.14 m | 44.98 deg | 42.00 deg | **1.071** (cpd ~6.6 % too large) |
| `19-57` | artefacts 0.8...1.4 m | - | - | unusable, that drive had the inverted offset |

Cross-check: the VWAS config holds `maxAngle = 44.0` and `cpd = 18.12`; the tightest
circle at L = 2.14 m asks for 45.0 deg, so the calibrated maximum is right and only
the reported angle was 6.6 % low on that drive.

Operator decision: NOT pursued (AgOpenGPS' own wizard produces the maximum steering
angle, and the correction is 6.6 %). No live display was built. The method stays
available offline - if the ratio ever reads far from 1.000 again, it is one command.

Also verified while doing this: the wheelbase actually reaches the module. The
start-up line `HeadingBridge ENABLED: L=2.50m` is the config default printed by
init(); the live value is pushed in from the VWAS/AOG config in the 100 Hz tick
(`AutosteerProcessor.cpp:392`). `update()` now reports the change explicitly
(`HeadingBridge wheelbase 2.50 -> 2.14 m (VWAS/AOG config)`), so the log cannot be
misread again.

## 17. Open points (2026-09-23, evening)

* **Wheel angle in the projection**: it IS in there, but indirectly - the projection
  integrates the position along psi, and psi is advanced with
  `omega = v*tan(steerScale*delta + bias)/L` (see the note above
  `HeadingBridge::updateProjection`). So a wheel-angle error (cpd, steerScale,
  wheelbase) bends the projected path in turns and shows up as a cross-track
  residual, which is why steerScale must not self-learn while the cpd value is
  uncertain.
* `steerScaleEstimate` / `HR` column `sc_n`: the estimator collects a window now
  (fix of §15), but no drive has been logged since - it is expected to move off 0,
  and the first value has to be reviewed before `applyLearnedSteerScale` may be
  enabled.
* `vScale` stays passive for the same reason.
* `proj_upd` is structurally low while the attitude is healthy: the projection only
  runs when the attitude is NOT trusted (94 % LIVE on the 20-19 drive). That is by
  design - the attitude is the better measurement. Running it in LIVE as a pure
  plausibility check is an option, not a defect.
* Reversing detection uses the speed gate (`minSpeedForDirection`); AgOpenGPS uses
  the travelled distance (0.25 m). For slow, long reversing manoeuvres the distance
  criterion would react sooner. Not implemented.
* The ~1.2 deg between the measured `mnt` (-88.8) and the configured offset (+90) is
  intentionally left alone.
* HB1/HB2 do not print `rev` yet; direction changes appear as NOTICE lines only.

## 18. Hall drives (2026-09-23 evening): what the two logs showed, and the gates

Two drives, each with circles outside, a pass under trees and a half circle inside a
hall. Operator report: log 1 (`20-43 Halle.txt`) had the direction "spin" inside the
hall and correct itself only when reception returned; log 2 (`20-47 Halle besser.txt`)
had the direction correct all through the hall and span only afterwards, already
outside. Both reproduce in the `HR` columns.

| | log 1 | log 2 |
|---|---|---|
| hall phase | t = 136...162 s | t = 321...331 s (+ 367...393 s exit) |
| reception | `qf` 0, sats 0 -> BRIDGE/COAST/DEADRECKON | `qf` 5, sats 9...25 -> stays LIVE/att |
| heading there | follows the COG and drifts (psi 112 -> 77 -> 15 -> 10 deg) | correct (psi = att + 89 deg) |
| direction | **rev=1 for ~12 s, dev 173...179 deg** | **rev=1 for ~6 s, dev 167...175 deg** |
| on reception | psi jumps 122.6 -> 6.2 deg in ONE frame (~244 deg) | psi jumps 319.2 -> 350.0 (31 deg) |

Root cause: the **VTG course over ground is inverted under a roof at walking pace**
(multipath), while the position stays RTK-good. The old decision compared that COG
with the heading (`raw = |psi - cog|`), so `dev` went to ~180 deg and the module
declared REVERSE while the machine drove forward. In log 2 the attitude stayed
trusted, which is why only the direction lied and the heading did not.

Two secondary findings in the same logs:
* `mnt` was pulled from -88.8 deg (clean drive) to -83.0/-84.7 deg, because the
  offset measurement took the (inverted) COG as its reference.
* the state machine flipped **COAST <-> DEADRECKON every 100 ms** (log 1, 142...148 s)
  because `cogValid`/`pathValid` dip in and out around their own limits
  (`coastMaxMs = 0` = "no limit" is correct here, the flicker was the cause).
* `steerScaleEstimate` reached **1.544** (n=300, spread 0.0045) on the clean circles of
  log 2 and was reported as "LEARNED (passive)" - far from the 1.07 the circle
  measurement gives. Concrete, no tyre slip (concrete floor, good grip), so it is a
  contaminated estimate, not a machine constant.

Gates added the same evening (all of them defensive, nothing changes on a clean drive):

1. `minSpeedForCogUse` 0.5 -> **1.0 m/s**: below walking pace the COG is not used at
   all - holding the estimate is better than a measurement that is known to be wrong.
2. **Direction of travel from the POSITION DELTA** (AOG's `dualReverseDetectionDistance`
   scheme, `Position.designer.cs:738-754`), with the required distance
   `max(0.25 m, 10 * sigma_pos)` - sigma from the receiver's own standard deviations
   when reported, otherwise from the fix type (RTK fixed 2 cm, float 25 cm, single
   1.5 m x HDOP). AOG can hard code 0.25 m only because it assumes RTK; with this
   threshold a degraded fix cannot fake a direction out of noise.
3. **COG cross-check** for that decision: if COG and position delta disagree by more
   than 45 deg, one of them is lying and the previous decision is held (AOG's
   "goto byPass").
4. `fallbackHoldMs` = **300 ms**: the state no longer follows the flicker of
   `cogValid`/`pathValid` (no more COAST <-> DEADRECKON chatter).
5. **Learning gates** for the steering scale: at least 12 satellites, no direction
   change during the window, and a bounded window innovation (30 deg) - a hall exit
   snapped psi by 244 deg once and that alone would have been enough to store a bogus
   constant. Plus a plausibility window 0.80...1.30 for APPLYING it (outside that the
   value is reported as IMPLAUSIBLE and never written).
6. `mnt` (mounting offset) now uses the position window when it is valid and the COG
   only above 1.5 m/s, so an inverted COG can no longer fake a mounting error.

Operator corrections applied the same evening:

* **Re-entry ramp** (explicit request): after a blind phase the first attitude frame
  is applied with `reentryRampGain` x inflated measurement noise, decaying linearly to
  1 over `reentryRampMs` (500 ms = 5 frames at 10 Hz). The same correction is applied,
  only spread out - AgOpenGPS no longer sees a step (the 244 deg jump becomes ~5
  frames of ~50 deg). The very first lock after boot is exempt, so a cold start still
  snaps in immediately. Log line: `HeadingBridge re-entry: ramping the attitude
  correction for 500ms`.
* **The satellite count is the weakest quality measure we have.** Operator note: NMEA
  caps it at 12, an F9P never reports more there (more only in UBX), and a healthy fix
  delivers 9...12 - so a threshold of 9 is almost always true and tells nothing about
  MULTIPATH, which was exactly the hall problem (26...33 satellites and still an
  inverted COG). It is now only a floor (`minSatsForLearn = 6`, and `0` = "receiver
  sends no count" does not block). The learning gate tests quality where it matters:
  * `fixQuality >= 4` (RTK fixed position),
  * `|baselineDevPct| <= baselineTolPct` - the solved dual-antenna baseline length
    drifts first when the sky view is obstructed,
  * `effectiveMeasurementR(0) <= maxAttRForLearn` (3 deg) - the filter's OWN live
    estimate of how much the attitude scatters. That is the quantity a machine constant
    must not be learned from, and it is available continuously, unlike a satellite
    count.
  Other quality figures that exist in `GNSSData` and are already used elsewhere:
  `headingQuality` (attitude QF, gates the whole bridge), `posStdDevLat/Lon` and
  `velStdDev*` (INSPVAXA, used for the direction threshold's sigma), `hdop` (GGA),
  `ageDGPS` (not logged yet).

* **The demanded distance must stay short** - the second hall log contains turning
  manoeuvres in a confined space, and a long threshold would miss them. The sigma
  term is therefore capped: `need = clamp(5 * sigma, 0.25 m, 0.5 m)`. Verified against
  that log: 0.25 m are travelled in p50 0.20 s / p90 0.40 s, 0.50 m in p50 0.40 s /
  p90 0.60 s, and the real direction switches happened while 0.95...2.66 m were covered
  within +-1 s - so the switch reacts inside about half a second. (The original 10*sigma
  would have asked for 2.5 m on an RTK float fix: far too slow for a turn.)
* **NMEA caps the satellite count at 12** (historical; an F9P never reports more there,
  more only in UBX). A learning gate of ">= 12" would have been a coin flip, so it is
  now **9**, and a receiver that sends no count at all (0) does not block the estimator.
  Note that the UM982 in the hall logs reported 26...33, i.e. it does not use that cap.

`HR` gained `dir_ref_m` (distance the direction decision was based on) and
`pos_sigma_m` (the sigma the threshold came from), so the next log shows directly
whether the decision had enough displacement to be trusted.

Note on RAM: these additions pushed RAM1 (ITCM) over the limit by 21 kB of alignment
padding (free -21088 B). `logRawFrame()` and `logDebugLine()` are `FLASHMEM` now
(diagnostic output is not timing critical), which brought it back to 11680 B free.

## 19. Position fall-back and attitude settling (operator requests 2026-09-24)

Two requests from the bridge/hall drive of 2026-09-24 (`6-20 Aichhof - WG` under a
bridge, `6-36 Reut-Eichgraben-3xHalle` three times through a hall):

**(a) "the position jumps, fake it too" - the projection already integrates the
POSITION** (`projLat/projLon`, 100 Hz, along psi, which carries the wheel angle).
What was missing: it only ran while the attitude was untrusted, it was never
published, and it snapped to every fix - including impossible ones. Measured in the
bridge log: 3 spikes in 355 s, one of 2.23 m at 5.28 m/s (allowed 2.08 m) exactly on
the bridge crossing with `state=BRIDGE`, one of 1.29 m while crawling, and one
outlier of 5.5 Mm while standing (invalid coordinates). Now:

* the projection runs on EVERY 100 Hz tick; it acts as a heading measurement only
  while the attitude is untrusted,
* a **fix gate** rejects physically impossible steps
  (`step > posSpikeMargin * v * dt + posSpikeNoiseM`) and missing fixes, and the
  projection keeps integrating instead of snapping to them,
* while that lasts, `NAVProcessor::formatPAOGIMessage()` publishes the PROJECTED
  position; `getFallbackPosition()` feeds it. Fix quality, satellites, speed and
  heading stay the receiver's own, so AOG's lost-RTK handling remains intact,
* budget `posFallbackMaxMs`/`posFallbackMaxM` = **7 s / 20 m** (operator choice).
  At 1 m/s the time limit dominates (7 m travelled, far inside 20 m), at >= 2.9 m/s
  the distance limit does. Position error of a 7 m extrapolation =
  travel * sin(heading sigma): 2 deg -> 0.24 m, 5 deg -> 0.61 m. That is why a long
  time limit is safe at working speed and self-limiting on the road,
* a merely inaccurate fix (RTK float) with a plausible step is still ACCEPTED - it is
  better than extrapolating for seconds; `fixQuality >= 4` is required for
  MEASUREMENTS (direction, learning), not for following the position,
* `HR` columns `pfb` (fallback active) and `pfb_m` (distance extrapolated).

**(b) "the first values after reception returns are bad" - confirmed.** In the hall
log, the first 0.5...2 s after every re-lock carried roll values of 10...19 deg on a
machine driving straight (gap #4: roll +13.8, +18.5, +17.9, +15.7, +19.3 ...), and
those frames passed every gate (`rollEnv` 25 deg, `rollStep` only large jumps) and
pulled psi around (psi 127 -> 117 in one second). The baseline solution simply is not
there yet, and the UM982 reports no baseline length (`base`/`bdev` = 0), so the
baseline test cannot help.

New **settling gate** (operator parameters): after a gap the frames are observed but
not USED until the sequence has been consistent for `reentrySettleMs` = **1 s**:
|roll| <= 8 deg, per-frame change of roll <= 3 deg and of azimuth <= 5 deg. Until
then the fall-back sources keep the estimate alive (`RejectReason::SETTLING`, visible
in the log as `rej=settle`), then the re-entry ramp (R x 30 over 500 ms) takes over.
`reentrySettleTimeoutMs` = 3 s is the safety net for rough ground, where a perfectly
still sequence may never appear.

RAM note: these additions tipped RAM1 over again (-22112 B padding). Moving
`updateSteerScaleCalibration`, `updatePositionWindow`, `updateMountOffset` and
`selectFallbackMeasurement` to `FLASHMEM` (1...10 Hz each, none of them in the
100 Hz hot path) brought it back to 10656 B free.

**Budget follows the OPERATOR question of 2026-09-24: "at 0.2 km/h the time is too
short to pass under a tree".** Correct - 7 s at 0.056 m/s is 0.39 m. What has to be
bounded is the POSITION ERROR, which is `travel * sin(sigma_psi)`, i.e. a DISTANCE,
not a time. So:

* `posFallbackMaxM` = 20 m stays the hard ceiling and is tightened by
  `posFallbackMaxDriftM / sin(sigma_psi)` (0.5 m allowed error): sigma 0.5...1 deg ->
  the full 20 m, 2 deg -> 14.3 m, 5 deg -> 5.7 m. At creeping speed with a good
  heading that is 20 m of travel - 357 s at 0.2 km/h,
* the time limit is generous now (60 s) and applies only WHILE MOVING: at 1 m/s the
  distance limit rules anyway (20 m = 20 s), and standing still the extrapolation does
  not move at all (no drift), so it simply holds the last good position.

## 20. IMU support: yaw rate and offset-corrected heading (operator request 2026-09-24)

Operator: "the TM171 delivers very good heading results - check how AOG does it."
Checked, and AOG's scheme is exactly the operator's own idea:

**How AgOpenGPS uses an IMU heading** (`GPS/Forms/Position.designer.cs`, several
branches: 252-281, 433-464, 561-563, 623-626):

```csharp
if (ahrs.imuHeading != 99999)                    // 99999 = no IMU
{
    double imuHeading  = toRadians(ahrs.imuHeading);
    double gyroDelta   = (imuHeading + imuGPS_Offset) - gpsHeading;
    imuGPS_Offset     += gyroDelta * 0.0005;     // continuous calibration
    imuCorrected       = imuHeading + imuGPS_Offset;
    fixHeading         = imuCorrected;           // the IMU heading BECOMES the heading
}
```
So the IMU heading is used **relatively**: an offset against the GNSS heading is kept
up to date while the GNSS heading is good, and afterwards the corrected IMU heading is
the heading - all the way into the guidance (`fixHeading`). The magnetometer is never
trusted as an absolute. Where the value comes from: `UDPComm.Designer.cs:136-137`
reads it from PGN 211 (`imuHead *= 0.1`) - which is the PGN our own IMUProcessor sends
(`sendIMUData()`), plus `:89` "dual as IMU" and `:196` the 99999 sentinel.

**Ported into the bridge** (both sensors, BNO085 and TM171, because IMUProcessor
already applies the mounting/axis configuration AOG sends via PGN 251 bit 3,
`ConfigManager::isUseYAxis`):

* **yaw rate** (`Config::useImuRate`): `psi += (yawRate - yawBias) * dt` replaces the
  wheel-angle model, which is blind at creeping speed (`v*tan(delta)/L` ~ 0 at
  0.2 km/h) and inherits every error of cpd/wheelbase/slip. `yawBias` is learned
  while the attitude is trusted (the filter's own heading change is the truth then),
  clamped to +-30 deg/s, and a bias beyond `imuBiasWarnDegS` (5 deg/s) means the SIGN
  or the mounting is wrong: the gyro is then not used at all and a warning is logged,
  so a wrong sign can never steer the machine. Process noise drops from 4.7 to
  1.5 deg/s while the gyro carries the rate.
* **heading** (`Config::useImuHeading`): `imuHdgOffsetDeg` is learned with AOG's own
  alpha (0.0005 per 100 Hz tick) while the dual-antenna attitude is trusted, and the
  corrected heading is then a fall-back MEASUREMENT (`src=imuhdg`) with noise starting
  at 4 deg and growing 0.05 deg/s of "blind" time (an AHRS heading drifts), capped at
  20 deg. It sits after the geometrically exact fix-to-fix window and BEFORE the COG -
  precisely because the COG is worst where the IMU heading is strong (slow driving).
* `HR` gained `imurate, yawbias, imuhdg, imuhdgoff`; the web page and the JSON report
  the same, including whether the gyro is currently used.

Measured limitation: the TM171 path in this project parses only RPY packets
(`TM171AiOParser::parseRPYPacket()`, 20 byte payload) and explicitly reports no yaw
rate (`IMUProcessor.cpp:297`). For a TM171 the HEADING path above is therefore the one
that acts; the BNO085 provides both. Drifting the TM171 yaw to fake a rate was not
done - an AHRS yaw is magnetic/fused and too jumpy to differentiate into a usable
rate.



* Operator decision 2026-09-24: `useImuRate` is ON by default (it only acts while a
  valid IMU reports, and the bias safety gate disables it by itself if the sign or the
  mounting is wrong). `useImuHeading` is on by default as well. With a UM982 (no IMU)
  nothing changes at all.
* NOT built: "roll from the IMU". Where it would matter: (a) a single-antenna setup has
  no dual roll at all, so the bridge's own roll (`status.rollDeg`, used by the VWAS
  side-slope zero regulator and reported to AOG) would need it - AOG itself already
  gets an IMU roll in its own NMEA field (`NAVProcessor.cpp:159`); (b) with a dual
  antenna it is not a source but the ideal CROSS-CHECK: the receiver roll is exactly
  what jumped to 19 deg on the hall re-locks while the machine drove straight - a
  gravity-referenced IMU roll would expose that in ONE frame instead of after the 1 s

## 21. Log check 2026-09-24: the drive predates the fixes (and what that showed)

Operator reported three symptoms from two logs (`6-20 Aichhof - WG`, `6-36
Reut-Eichgraben-3xHalle`). First finding: **those logs do not contain the fixes**.

* `HR` has **42 fields** = 41 columns + tag, i.e. the state after "re-entry ramp +
  distance-based direction" - the later work is provably absent:
  `position fall-back` lines: **0**, `attitude settled` lines: **0**. The current build
  emits 48 fields.
* So "position still jumps on the bridge" and "wrong headings after the hall" are
  EXPECTED for that build: the position fall-back and the settling gate were not in it
  yet (only the 500 ms ramp was, and its `re-entry: ramping` lines are present).

What the log DID show, and what was tightened as a result:

* direction: **22 frames declared REVERSE with an angle < 90 deg** and 3 missed
  switches, while the machine drove a half circle forward. Cause: in the hall the
  attitude is gone (psi comes from the path sources then, so comparing psi with the
  path is meaningless) and/or the raw fix jumps, so the position delta pointed
  anywhere. Countermeasures, all built:

## 22. Log check 2026-09-27 (five drives: bridge, hall, 49 HR fields)

The five logs `2026-09-27 7-38 … 7-56` carry **49 HR fields** (48 columns) - the
current column set including `rollbridged` from the parallel task - so the firmware
was up to date in that sense. What they show:

* **working as designed**: the settling gate fires (`rej=settle` 20/50/39/81/32
  frames) and the position fall-back runs (114/9/0/0/279 frames, max 12.3 m
  extrapolated). Log 1 begins with **111 s of INIT** (qf 0, sats 0 - no fix, no
  attitude at all) and the first attitude after that is held back by `settle`, exactly
  the intended behaviour.
* **the bridge spikes are still there because the gate was still too loose**: the two
  bridge logs (`7-47`, `7-51`) have `position fall-back frames: 0` - with
  `3 * v * dt + 0.5 m` a 2 m error at 5 m/s passes unopposed. The tightened gate
  (0.5 * v * dt + 0.4 m) was built AFTER these logs.
* **`rev` was decided during DEADRECKON** (log 5): the two reverse phases start at
  `state=DEADRECKON, qf=0, sats=0, src=deadreck, dev~149 deg` and end when the
  attitude returns (`dev~28 deg`). That is the direction logic acting on a heading that
  was dead-reckoned and on a path it could not measure - the `attitudeTrusted`
  precondition plus the speed-consistency check were built after these logs too.
* **`mnt` (mounting offset) was poisoned**: -78.5 deg and -76.6 deg in the two hall

### 22b. The real bridge problem: the receiver's RECOVERY time (2026-09-24)

Operator: "the bridge crossing is only 10 m or less - the time the GPS needs to
recover seems to be the problem". That exposed a hole in the fall-back logic: it had
no way BACK. The projection was compared against every fix and, once the extrapolation
had drifted, the deviation stayed large - so the fall-back would have defended its own
estimate against a perfectly recovered RTK fix until the budget ran out (and the
"wrong values after the hall" the operator reported are exactly that window).

* **Hand-over through a RAMP** (operator request 2026-09-24): when the fall-back ends -
  whether because RTK recovered or the budget is spent - the extrapolated position is

## 23. Heading flicker with bad reception (operator report 2026-09-27)

Operator: "the heading flickers with bad reception, and the autosteer then steers
wildly - strengthen/smooth the heading? Also filter the PWM with a moving average
while the projection is active? I would prefer a better/more stable heading."

**Measured** over five drives (27.09. 08:49...09:04, `heading_jitter.ps1`), split into
good (LIVE, qf 4, >= 15 satellites, moving) and bad reception frames:

| phase | \|dPsi\| per 100 ms p50 | p90 | p99 | sigma p50 |
|---|---|---|---|---|
| good | 0.10...0.26 deg | 0.5...1.6 deg | 1.9...4.0 deg | 1.9...2.6 deg |
| bad | **0.60...0.75 deg** | 2.6...3.8 deg | **5.6...9.5 deg** | 1.4...2.5 deg |

So the flicker is real (5...6x the step size, up to 9.5 deg in single frames) - and the
essential finding is that **sigma does not move**: the adaptive R estimate is far too
optimistic exactly when it matters. Its time constant is alpha 0.01 per frame at 10 Hz
= ~10 s, so a phase under trees is over long before the estimate reacts.

Two countermeasures, both built:

1. **A direct reception quality factor on the measurement noise**
   (`receptionQualityFactor()`): 3x while the attitude is not trusted, 1.8x when the
   attitude quality is below RTK fixed.
   It is applied to EVERY source and to the self-learning gates (they use the same
   effective R), so a bad phase both damps the heading and stops the estimators.
   The satellite count is NOT part of this any more (operator correction 2026-09-27:
   *"die Zahl der Satelliten ist kein Maß"* - it is not a quality measure; measured over
   the 2026-09-27 drives it covered 5...34 and stayed at 14...20 under trees while the
   solution quality is what degrades). `rQualitySats` stays as an option and now defaults
   to **0 = off**; the heading quality (0 none / 4 RTK fixed / 5 float) decides.
2. **Output smoothing, poor phases only** (`outSmoothMs` = 200 ms): the heading that
   goes into PAOGI is passed through a first-order filter ONLY while the estimate is
   not trustworthy. In the good case the value stays exactly as measured - a filter in
   the steering loop costs phase margin, so it must not be active when it is not
   needed. This acts directly on what AOG steers by, without touching psi.

**No PWM filtering** (operator question): a low-pass on the motor PWM is downstream of
the problem and adds latency where phase margin is scarcest - it can turn "wild
steering" into oscillation. The wild steering comes from the heading flicker, so it is
fixed there. (The firmware already has a PWM ramp and an existing
`filteredCommandPwm` from the parallel task; those stay untouched.) If any roughness
remains afterwards, the right place is the steering-controller response, not the PWM
signal.

  blended onto the fix over `posRampMs` (500 ms, ~95 % of the distance) instead of
  snapping. The projected position stays the published one until the ramp finished
  (residual < 5 cm or 3x the ramp time), and no measurements or vScale learning happen

### 23b. Output smoothing with a rate lead (2026-09-27, evening)

Operator: "you can smooth at higher speed too, the trace in AOG then is calmer - are
200 ms also ok at creeping speed?"

Measured answer: a pure low-pass of time constant tau lags a steady rotation by
`tau * omega`:

| turn rate | lag at tau = 200 ms |
|---|---|
| 5 deg/s | 1.0 deg |
| 15 deg/s | 3.0 deg |
| 30 deg/s | **6.0 deg** |
| 45 deg/s | 9.0 deg |

At creeping speed, turns of 20...40 deg/s are normal, so 4...8 deg of lag would have
been a permanently wrong steering error - too much. Therefore the filter runs with a
**RATE LEAD**: `az_out = filtered + yawRate * tau`. A smooth real rotation passes
through undelayed, only the fast jitter is damped, and a **residual cap of 3 deg**
pulls the filter hard when it would fall further behind (direction change, re-lock).

Consequences:

* the smoothing is now **always** active (good reception included) - the operator
  wanted a calmer trace in AOG. Internally psi is NOT touched; this is only the value
  that leaves the module.
* `HR` gained a final column **`az_out`** (the heading actually sent) so the effect can
  be verified against `att` (raw) and `psi` (internal) in the log.
* `outSmoothMs` = 200 is one number to tune; 0 disables the filter.

### 23c. The steering chain in the raw log (2026-09-27)

### 23d. Result after the smoothing: the heading is calm, the PWM is not (2026-09-27 evening)

Operator: "overall much better … but the steering is still very unsteady with bad
reception; we must stabilize the heading."

**Heading, before -> after** (all five drives, `heading_jitter.ps1`):

| | 08:49...09:04 | 09:28 (schleichfahrt) | 09:37 |
|---|---|---|---|
| good p50 / p90 | 0.10...0.26 / 0.5...1.6 deg | 0.12 / 0.49 | 0.10 / 0.33 |
| bad p50 | 0.60...0.75 | **0.26** | **0.40** |
| bad p90 | 2.6...3.8 | **1.93** | **1.71** |
| bad p99 | 5.6...9.5 | **3.49** | **3.44** |
| steps > 5 deg | 10...19 | **0** | **2** |

That is the R quality factor plus the output smoothing working as intended (note:
`psi` in the log is the UNsmoothed internal value, so this improvement comes from the
measurement weighting).

**Speed at bad reception** (operator question "I have no idea how good the speed signal
is with bad reception"): the per-100 ms jitter of `v` is
* good: p50 0.021 / p90 0.082 / p99 0.201 m/s
* bad:  p50 0.049 / p90 0.134 / p99 0.283 m/s
i.e. only 2...3 % at 1.6 m/s, with no bias (median 1.55 vs 1.60). The VTG speed comes
from the receiver's DOPPLER solution, which does not depend on the position fix - so
the kinematic model `omega = v*tan(delta)/L` stays a valid rate reference even under
trees. A model-based rate gate (operator idea) is therefore feasible.

**And now the important part - the steering chain** (`steer_chain.ps1`, 53-column logs,
guidance engaged):

| log | frames guided | \|d(AOG target)\| p50/p90/p99 | \|dPWM\| p50/p90/p99 |
|---|---|---|---|
| 9-28 schleichfahrt | 1512 | 0.20 / 0.90 / 2.50 deg | **13 / 42 / 95** |
| 9-37 | 1181 | 0.30 / 1.00 / 2.80 deg | **14 / 43 / 95** |

The commanded angle is smooth (0.2...0.3 deg per 100 ms typical) while the PWM moves
13 counts (median!) and up to 95 of 199 in single steps. **The unsteadiness the
operator feels is therefore no longer coming from the heading or from AOG's steering
request - it is the steering controller / motor loop.** Only 7...9 % of the frames show
a target step > 1 deg together with a PWM step > 3, so the target explains a small
fraction. This belongs to the parallel steering task, not to the heading bridge.

Tooling note: the first run of `steer_chain.ps1` reported nonsense (|aog_tgt| 213 deg)
because the column indices were off by one (column 48 is `roll_bridge_deg`, `az_out`
is 49) - a script error, the firmware was correct.


Operator question: "do you want to log the PWM as well?" - yes, because otherwise
"wild steering" stays a subjective impression. The raw log now carries the whole
chain, three columns added:

| column | meaning |
|---|---|
| `az_out` | the heading that leaves the module (offset applied, output-smoothed) |
| `aog_tgt_deg` | the wheel angle AgOpenGPS asks for |
| `pwm` | the PWM actually sent to the motor/valve |
| `guid` | 1 while guidance is engaged |

Fed in by `AutosteerProcessor::process()` via `headingBridge.setSteerTelemetry()`, so
the bridge does not need access to the motor driver. With these a flicker can be
followed end to end (heading step -> target step -> PWM step, guidance engaged only),
which is exactly what is needed to prove whether the output smoothing helps.


  during it, because the residuals would measure the ramp instead of the model.
  Log: `… - ramping onto the fix (residual 0.62m) over 500ms` then
  `position ramp finished (residual 0.04m after 470ms)`.

Added:

* **RTK recovery detection**: as soon as `fixQuality >= 4` has held for
  `posRtkRecoverMs` (1 s), the position is taken as the truth again - the projection is
  re-synchronised to it even if that means accepting a jump, and the fall-back ends
  (`position fall-back off: RTK recovered after Xms / Y m (jump … taken over)`).
* **Budgets for a real recovery**: time **120 s** (only while moving, unlimited when
  standing), distance **40 m**, allowed extrapolation error **1.0 m**
  (`posFallbackMaxDriftM / sin(sigma_psi)` still tightens it: 2 deg -> 28.6 m).
  A 10 m bridge at working speed plus a few seconds of re-fix fits in easily now,
  and the drift bound keeps a long extrapolation from becoming a guess.

  logs against -88 deg in the others, because the 30 s EMA kept eating phases in which
  the path sources (not the attitude) defined the reference. Fixed: `updateMountOffset`
  now requires `status.attitudeTrusted`.

  1. no direction decision unless `status.attitudeTrusted` and not while the position
     fall-back is active,
  2. the position delta must match the SPEED (`|dist - v*dt| <= 0.5*v*dt + 0.4 m`),
     otherwise the evaluation is discarded and the reference is re-based,
  3. a switch needs `dirVotesNeeded` = 2 consecutive agreeing evaluations,
* position gate **tightened**: `posSpikeMargin` 3.0 -> **0.5** and `posSpikeNoiseM`
  0.5 -> **0.4 m**. The projection is resynchronised at every fix, so after 100 ms it
  is only centimetres away from the truth; the old allowance let ~2 m of bridge error
  through unopposed (measured in the 06:20 log: 2.23 m at 5.28 m/s).

  settling window.

---

## 24. Wheel angle scale/offset and the model-rate gate (operator correction 2026-09-27)

Operator: *"Vielleicht ist Dein Ansatz falsch. Wenn ein Winkelfehler im Lenkwinkel
vorliegt ist das Verhältnis von Lenkwinkel zu Headingänderung gleich. Bei Änderung des
Lenkwinkels müsste die Ableitung der Heading-Änderung zur Änderung des Lenkwinkels gleich
sein. Dann bliebe alles berücksichtigt. Es gibt keine weitere Quelle für Geschwindigkeit."*

He is right, and the measurement below shows it is not a theoretical concern. The earlier
plan was an **absolute** model-rate gate with a fixed tolerance (2 deg + 30 %). That
assumes the model's own scale is right. It is not, and - his point - it does not have to
be:

```
omega = v * tan(delta_eff) / L          delta_eff = k * delta_reported + b
kappa_model = tan(delta_eff) / L        kappa_meas = omega_meas / v     (needs no v!)
```

* `kappa_model` is **v-free**, and `k` (wheel angle scale) and `b` (wheel angle offset)
  are constant machine parameters. A gate may therefore only judge the **deviation from
  the learned ratio**, never an absolute rate; otherwise a constant scale error is a
  permanent, systematic false rejection.
* `b` is real and it MOVES: the VWAS re-learns its zero point during a drive
  (`VWAS zero: … trim=`) and applies it (`encoderOffset` in the fused angle). The
  existing estimator solves **through the origin** (`s = 1 + sumXY/sumXX`,
  HeadingBridge.cpp:619), so that offset is mixed into the estimated scale. This is the
  concrete defect the operator's differential form removes.
* `v` enters only as a **factor**, so a speed error of +p % is indistinguishable from a
  scale error of -p %. With VTG as the only speed source (operator: "es gibt keine
  weitere Quelle") the geometry can identify the **product** `k/vScale`, not `k` alone -
  which is why the bridge learns `vScale` from the along-track position residual and the
  scale only from the curvature ratio. The two observables must not be mixed.



### 24a. Measurement (tool `steering_angle_from_circle.ps1`, extended)

The tool now (1) filters **reception quality over the whole window** (the reference yaw
rate is the receiver's dual heading, so a single bad frame fakes a wrong circle),
(2) prints `L_implied = R * tan(delta)` per wheel-angle bin, which separates the two
possible error shapes (constant => scale error, rising with the angle => offset/play),
and (3) fits `omega = v*tan(k*delta + b)/L` by a **bounded grid search on the median
residual** - a free Gauss-Newton step walks into the tan pole (it returned k = 75.8 with
a *worse* residual than k = 1) and least squares follows the bimodal half of the
"steady" candidates.

Four drives of 2026-09-27, `L = 2.14 m` (confirmed in the logs: `wheelbase 2.50 -> 2.14 m
(VWAS/AOG config)`), fit over the informed range `|wheel| 8...45 deg`, sign consistent:

| log | condition | frames in fit | k | b [deg] | median residual k=1 -> fitted |
|---|---|---|---|---|---|
| 9-42 | trees, R 2.67 m @ 42.8 deg | 78 | **0.900** | +1.0 | 7.58 -> **1.40** deg/s |
| 9-47 | hall, R 2.15 m @ 40.6 deg | 39 | **1.100** | -3.0 | 2.10 -> **1.22** deg/s |
| 9-28 | trees, crawl, R 6.76 m @ 34.9 deg | 22 | **0.600** | -4.0 | 21.79 -> **1.32** deg/s |
| 9-37 | trees, slow | 8 of 31 | 0.750 | +2.0 | 0.54 -> 0.53 (no signal) |

Three findings:

1. **The model FORM is confirmed**: the residual collapses by a factor 5...16 in every
   drive that contains a real turn (7.6 -> 1.4 deg/s against a 35 deg/s signal). The tan
   relation with a scaled and offset angle describes the machine.
2. **The scale is NOT a constant that may be assumed**: k = 0.60 ... 1.10 over four
   drives of the same machine (implied wheelbase 3.6 m ... 1.95 m against a configured
   2.14 m). The two best-quality drives bracket 1.0 (0.90 / 1.10); the two worst (trees,
   slow, almost no quality frames) sit below it - the *reference* degrades under trees,
   so the absolute identification is limited by exactly the reception the gate is meant
   to help with. A fixed assumed scale is therefore wrong, and the fixed tolerance has to
   stay generous (>= ~20 % plus a floor).
   The stored recommendation **1.38** in sections 4/9 is inconsistent with all four
   measurements (it would make the model 38 % too strong) - do not enable
   `applyLearnedSteerScale` before it is re-measured with this fit.
3. **Three alternative explanations were tested and rejected** - no need to re-litigate:
   * the heading is **not** held/stale between receiver fixes: during turns only 0.0 %
     (0.0 % / 1.6 % in the slow log) of the 100 ms heading steps are exactly zero, p50
     1.6 deg per 100 ms, so a window chord cannot be biased that way
     (`hdg_update_rate.ps1`);
   * the machine is **not** articulated (`VWAS diag: type=0`, Ackermann), so `tan` is the
     right form and the `sin` variant is not needed;
   * there is **no IMU**: `imuRateDegS` and `imuHeadingDeg` are 0.0 in every frame of
     every 2026-09-27 log, so a gyro as an independent noise-free rate reference is not
     available on this machine today.

### 24b. Consequence - the gate as the operator describes it

1. Compare **curvatures**, not rates: `kappa_model = tan(steerScale*delta + bias)/L`
   against `kappa_meas = yawRateMeasured / v`. The model side needs no speed at all.
2. Gate on the **relative** deviation from the learned ratio
   (`|kappa_meas - r*kappa_model| > max(kappa_floor, band * |r*kappa_model|)`), where `r`
   is the machine's learned curvature ratio. **Damp (raise R), never discard** - a wrong
   `r`, a wrong `L` or a wheel angle offset must not be able to reject a real curve.
3. **Floor instead of a tight tolerance at small angles**: below ~8 deg wheel angle the
   offset and the steering play dominate (the 0...5 deg bin shows `L_implied` 5.6 m in the
   tree log against 2.6 m at 30...45 deg). There the model is not a reference at all and
   must not gate.
4. **Learn with two parameters.** Replace the through-the-origin solve by the 2x2 normal
   equations - the second column IS the operator's derivative
   (`d(omega)/d(delta) = v*k/L * sec^2(delta_eff)`):

   ```
   X += 57.3 * v * tan(delta)   / L * dt      (exists: scaleAccX)
   Q += 57.3 * v * sec^2(delta) / L * dt      (ADD: the derivative column)
   innovWin = a * X + c * Q                   with a = k - 1, c = b in rad
   A = [[sum XX, sum XQ], [sum XQ, sum QQ]]   (with forgetting)
   y = [sum X*innov, sum Q*innov]
   k = 1 + (y1*A22 - y2*A12)/det      b = (A11*y2 - A12*y1)/det
   ```

   A wheel angle offset - including the VWAS's own re-trim - then cancels out of `k`
   instead of biasing it, exactly as the operator argues. Both stay PASSIVE + reported,
   like `steerScale` today.
5. The filter's own `steerBiasDeg` state is a *second* estimator of the same offset (it
   even sat on its -15.00 deg clamp in one 2026-09-27 log). Before anything is applied,
   decide which of the two is authoritative - applying both would double-correct.

### 24c. Tooling

`tools/heading_bridge/steering_angle_from_circle.ps1` gained `-MinHeadingQ` (alias
`-MinFixQ`), `-MinSats`, `-MaxSigmaDeg` (quality gate, 0 disables), the `L_implied` table,
the median-ratio read-out and the two-parameter fit on plain `double[]` (a pipeline
`Sort-Object` per grid point took it over 30 s per log). `Temp\aiolog\hdg_update_rate.ps1`
counts frozen heading
frames during turns - the check that rejected the stale-reference hypothesis.


### 24d. Operator correction 2026-09-27: the satellite count is not a quality measure

*"die zahl der sateliten ist kein Maß, da nmea maximal 12 meldet"*

Checked against the logs of the day: the count in column 9 covered 5...34 (9-42: up to 34,
9-28: up to 28), so on this receiver the NMEA field is not capped at 12 - but the count is
still the wrong criterion, and for the same reason the operator gives: under trees it is
14...20, i.e. saturated and uninformative, while what degrades is the *solution* quality
(col 8 `headingQuality`: 0 no solution / 4 RTK fixed / 5 float, and col 10 `fixQuality`
for the position). A count threshold therefore does not separate good from bad phases; it
only fires close to a total loss.

And his remark uncovered a real trap in that gate: `GNSSProcessor.cpp:976` sets
`numSatellites = 12` **as a constant** on the INS/INSPVAX path ("typical for INS
solution"). With `rQualitySats = 15` a receiver on that path would have been classified
"marginal" in **every single frame**, i.e. the heading would have been permanently
damped by 1.8x - calm, but laggy, and with the self-learning gates permanently tightened.
This is exactly the class of error the change removes.


### 24e. Built (operator decision 2026-09-27: "beides umsetzen")

**Two-parameter estimator** (`applyMeasurement`): the through-the-origin solve is gone.
The window now accumulates a second regressor
`Q = integral 57.3*v*sec^2(delta)/L dt` (= the operator's derivative, `d(omega)/d(delta)`)
next to `X`, and the 2x2 normal equations are solved with the same 0.99 forgetting:
`innov = (k-1)*X + b_rad*Q`. `k` is clamped to `steerScaleMin/Limit`, `b` to the new
`steerOffsetLimitDeg` (6 deg). Both are reported (`sc`, new `sc_b` column, web UI) and
still PASSIVE - the model keeps `Config::steerScale`.

**Model-plausibility gate** (`update`/`applyMeasurement`/`receptionQualityFactor`): a
short window of its own (`gateWindowMs` = 300 ms) compares the RAW attitude heading
change against `steerScale*X + bias*Q`. If `dev > max(gateFloorDeg, gateBandPct*|M|)`
it raises R by `gateRFactor` (2.0) for `gateHoldMs` (300 ms). It never discards, it only
fires with a trusted attitude, and - because it goes through the same
`receptionQualityFactor()` - it also pauses the self-learning while it holds. Logged as
`mdev`/`mgate`, shown on the VWAS page.

**The limits, derived from the 2026-09-27 drives** (`gate_sweep.ps1` replay, 0.3 s
window, applied model k=1.0/b=0.0, which is what the firmware uses today):

| floor | 9-42 Bäume: FA / DET | 9-47 Halle: FA / DET |
|---|---|---|
| 2.0 deg | 16.6 % / 35 % | 2.9 % / 69 % |
| **3.0 deg** | **7.1 % / 16 %** | **0.7 % / 69 %** |
| 4.0 deg | 3.4 % / 7 % | 0.0 % / 65 % |
| 6.0 deg | 2.0 % / 4 % | 0.0 % / 39 % |

FA = fires although every heading in the window was RTK fixed; DET = catches a window in
which the heading quality had dropped. Chosen: **floor 3.0 deg, band 30 %** - the hall
drive is essentially untouched (0.7 %) while the gate catches 69 % of its bad windows;
the tree drive fires on 7 % of its good windows, which is the price for the case the
receiver does not flag at all (its DET label is weak by nature: under the trees qf stays
4 while the heading degrades). If a tree drive ever feels over-damped, floor 4.0 deg is
the next step (3.4 %), and applying the learned scale (~0.9) would lower the tree FA to
~4 % by itself.

Sensitivity that must be kept in mind: the deviation on RTK-fixed windows grows with the
window (p50 1.26 deg at 0.3 s, 1.27...5.18 at 1 s depending on the drive), so the window

### 25. Afternoon logs 2026-09-27 (14-12 ... 14-30): what the operator saw

Operator: *"unter Brücke war es subjektiv gut. unter den Bäumen eher gleich, also
verbesserungsfähig. beim ersten log hatte ich das Gefühl, dass der Traktor schräg fährt,
das heading also ganz am Anfang nicht gestimmt hat. Hier war dann vermutlich auch die 0
des Lenkwinkels noch falsch. Zur Unruhe unter den Bäumen hatte ich den Eindruck, dass
heading teilweise um 1° gedreht hat, obwohl die Lenkung ruhig war."*

Six drives: 14-12 Halle/Bäume (140 s, guidance OFF), 14-17 Bäume Aichhof (273 s, guided
56 %), 14-22 Brücke, 14-24 Brücke langsam, 14-28 Bach Schleichfahrt, 14-30 Halle.

**a) "the tractor drives diagonally" - measured, and it is NOT a fault.**
The direction of travel was computed from the POSITIONS (5 s bins, >= 2 m) and compared
with the three candidates:

| log | `cog` - travel | `psi` - travel | `az_out` - travel |
|---|---|---|---|
| 14-12 | +1.3 deg | **+3.9 deg** | -85.3 deg |
| 14-17 | -1.3 deg | **+2.2 deg** | -87.0 deg |
| 14-22 (the good one) | +1.1 deg | **+1.9 deg** | -87.2 deg |

`psi` (the internal body heading) follows the direction of travel within 2...4 deg - the
filter is right. `az_out` is that value minus the offset in EVERY log, and that is the
contract: `HeadingBridge.cpp:1261` states it ("AOG does body = received + offset, so the
firmware has to publish received = body - offset") and `applyAttitude()` uses the same
formula at line 258. **The doc/README wording ("az_out = the heading that leaves the
module, offset applied") was wrong and misled this analysis** - fixed in the tooling
README and in the log comments. AOG must have the same offset configured (operator rule).

**b) "the heading did not fit at the very beginning, the wheel angle zero was wrong" -
confirmed, two real defects.** On 14-12 the published heading was garbage for the first
~40 s:
* 0...20 s the module published the BOOT CONSTANT 270 deg (`psi = 0` minus the 90 offset)
  while the machine was already being driven. New: `Status::everTrusted`; until the filter
  has been seeded from a real measurement, NAVProcessor now forwards the receiver's own
  `dualHeading`/`dualRoll` - which is exactly what `az_out` becomes after the seed, so
  there is no jump and AOG is never fed a value no sensor produced.
* 20...40 s the heading was dragged around while the *wheel angle bias state*
  (`steerBiasDeg`) ramped 0 -> +15.0 (its clamp!) and back, because it was adapted in
  EVERY frame - including the INIT phase in which the VWAS was still learning its own zero
  ("VWAS zero: ... trim="). Two adaptive systems fighting each other. New: the bias state
  is only corrected in a clean, trusted window (`attitudeTrusted && fixQuality >= 4 &&
  !dirChanging && !modelGateActive`). Measured on 14-12: `bias` 5.10 -> 15.00 (clamp) ->
  13.7 -> 6.9 -> 2.5 -> 0.9 -> -0.2 deg.

**c) the model gate works as predicted.** 14-17 (trees): gate active in **9.7 %** of the
frames (offline prediction with floor 3.0 deg: 7 %), `mdev` p50 0.94 / p90 3.30 deg; on
the 14-12 drive 22.6 % (that drive had more bad reception). No frame was discarded - it
only damps.

**d) "the heading turned ~1 deg although the steering was calm" - the source is the
RECEIVER, not the bridge.** 107 such steps on the 14-17 drive (>= 0.7 deg/100 ms with the
wheel still): at **84 %** of them the raw measurement stepped too (`|d att|` p50 2.11,
p90 3.73 deg per 100 ms under the trees), and `headingQuality` stayed **4** in most of
them - so neither the reception factor nor the output smoothing engaged, which is exactly
why the drive felt "the same as before". 16 % are the filter's own corrections (some of
them the re-entry ramps after a gap).
Consequence (built): the model gate now counts as a poor phase for the OUTPUT SMOOTHING
(`status.outSmoothing`), so the one thing that the operator actually steers by is damped
exactly when the model disagrees - while `psi` stays responsive.

**e) The two-parameter estimator ran away: `sc` = 2.000 and `sc_b` = +6.00 deg (both
clamps) on the 14-17 drive.** Cause: X and Q are strongly correlated
(`X ~ Q*sin(delta)cos(delta)`), so `det = A11*A22 - A12^2` is a difference of two large,
nearly equal numbers - in float32 that is catastrophic cancellation, and the solution
wanders. Three fixes: the sums are `double` now; a **condition guard** falls back to the
offset-corrected scalar solve (`a = (sum Y1 - b_rad*sum XQ)/sum XX`, which cannot run
away) when `det <= 0.05 * A11 * A22`; and the estimator no longer learns from a window the
gate flagged. It also takes its window heading change from the RAW attitude now instead of
the filtered `psi` (psi contains the filter's own corrections, i.e. the estimator was
partly fed its own output).

**f) New tooling** (`tools/heading_bridge/`): `newlogs_check2.ps1` (gate statistics, the
start-up table heading-vs-position-bearing, the calm-steering step analysis with the
"did the measurement step too" split) and `traces.ps1` (time-binned state trace + the
event lines around each heading step).

stays SHORT - a 1 s window fires on 70 % of the good windows and is useless.

**Correction to 24a**: the fitted per-drive scale is an estimate from a handful of turns,
and judged over a whole drive the configuration is better than the fit in the hall log
(dev p90 1.43 deg with k=1.0 against 2.33 deg with the fitted k=1.10 - i.e. 1.10 was

### 25b. The real source under the trees: the WHEEL ANGLE the bridge is given (steering task)

Operator: *"heading teilweise um 1° gedreht hat, obwohl die Lenkung ruhig war"* and
*"Hier war dann vermutlich auch die 0 des Lenkwinkels noch falsch"* - both confirmed, and
both come from the steering chain, not from the receiver:

**a) The VWAS zero moves a lot, and differently per drive** (`VWAS zero: ... trim=`):

| drive | trim at the end | fits | state |
|---|---|---|---|
| 9-42 morning | -38.81 | 117 | LOCKED |
| 14-12 (first log) | -36.15 | **20** | **still learning** |
| 14-17 trees | -28.97 | 66 | LOCKED |
| 14-22 bridge | **+25.46** | 53 | LOCKED |

That is a spread of ~55 deg between drives, and in the first log the zero was still being
learned while the operator drove (`fits` 6 -> 20). The bridge sees `fused = enc - trim`,
so a wrong trim is a pure OFFSET on the wheel angle - up to ~30 deg, while the bridge's
own `steerBiasDeg` state can only absorb `steerBiasLimitDeg` = 15 and needs tens of
seconds. Measured on 14-12: the state ran 5.1 -> +15.0 (clamp) and back.

**b) What a wheel angle offset does to the heading** - it is not small: a 9 deg offset
yaws the model at `57.3*v*tan(9)/L` = 4.2 deg/s per m/s = **8.4 deg/s at 2 m/s**. And
without an IMU `status.yawRateDegS` IS that model rate, which is (a) integrated while the
attitude is missing and (b) added to the published heading as the SMOOTHING RATE LEAD
(`+ yawRateDegS*tau`). So a wrong zero appears as a permanent heading offset of
rate*0.2 s and, when the wheel angle jitters, as a wobble of about 1 deg - exactly the
"1 deg while the steering was calm" report. Built: the lead is dropped while the model
gate is active.

**c) The measurement that proves the wheel angle was wrong in the afternoon.** On steady
plateaus (wheel quiet >= 1 s, |wheel| >= 10 deg, forward, RTK fixed) the measured rate
relative to `57.3*v*tan(delta)/L` is:
* 9-42 morning: **0.80** (raw attitude) / 0.79 (psi) - consistent with the circle fit
  (0.86...1.10) and with the reported wheel angle;
* 14-17 afternoon trees: **0.15** (p10 ~ 0.0) - the machine turned 6x LESS than the
  reported wheel angle predicts. At 25...30 deg the circle tool's implied wheelbase is
  **15.8 m** in that same log, against 2.4 m at 10...15 deg: the reported angle and the
  geometry do not belong together on that drive.

**d) The window estimator has ALWAYS read high** - `k` = 1.4...2.0 on every drive
(morning and afternoon, forward-only and steady-only, scalar, offset-corrected and
2-parameter), while the steady-state circle measurement reads 0.86...1.10 and the
raw-attitude/psi rates agree with each other (0.803 vs 0.790). So the historical
**1.38 is this estimator's structural bias, not a machine constant** - `applyLearnedSteerScale`
must stay OFF and the value must not be believed. The estimator is a diagnostic; the
machine constant comes from the offline circle fit. What was fixed on the firmware side:
double-precision sums, the condition guard, no learning from gated windows, the raw
attitude as the window reference, and the offset (if any) must come from the filter's own
bias state - one offset estimator, not two.

Reported to the steering task (it owns the VWAS): the zero learner must not be allowed to
move the reference by tens of degrees between drives, and the bridge needs a way to know
whether the wheel angle it receives is currently trustworthy (`VWAS zero-trim: ARMED /
LOCKED`) - until then the bridge can only bound the damage (see 25a/25b above).

overfitted), while the tree log still prefers ~0.9 (2.88 deg against 1.79). So the
machine constant is good to ~10 % at best from field data, which is another reason why
the gate judges RELATIVE deviations and only damps.

RAM: the new state is ~20 floats and stays inside the alignment padding
(RAM1 free 9632 B before and after, build SUCCESS).

Changed:

* `Config::rQualitySats` default 15 -> **0 = disabled** (HeadingBridge.h). With 0 both
  uses - `receptionQualityFactor()` and the `outSmoothing` flag in `update()` - fall back

### 25c. Circular reference check (operator: "der task der VWAS soll darauf achten, dass es keinen Zirkelbezug gibt")

The loop is real and it is documented on the VWAS side (`WheelAngleFusion.cpp:1444`):

```
   VWAS wheel angle  ->  bridge model  psi += v*tan(delta)/L*dt
                     ->  psi  =  headingBridge.getVehicleHeadingDeg()   (WheelAngleFusion.cpp:461)
                     ->  VWAS GPS-angle / zero observer (bA = heading change over 0.5 m)
                     ->  VWAS wheel angle  ->  ...
```

Checked every path of MY changes against that (2026-09-27):

1. **`estimateSteerBias = false` BREAKS the bridge's half of the loop.** The VWAS comment
   says it explicitly: *"its steerBiasDeg state absorbs the very wheel-angle bias this
   observer estimates ... nothing PREVENTS it. Hence an explicit gate."* With the bias
   state off (operator rule: the zero belongs to the WAS/VWAS) the bridge can no longer
   absorb what the observer is measuring. The bias state and its limit stay in the code
   for a machine whose WAS zero really is untrustworthy, but they are off by default.
2. **The model gate is the one thing that could re-open the loop**: it fires INSIDE
   `state == LIVE` (it needs a trusted attitude), keeps R elevated for up to ~600 ms and
   makes psi lean on the model - while the VWAS gate only checked the state. Closed:
   `headingIsMeasured` in `WheelAngleFusion.cpp` now also requires
   `!zbSt.modelGateActive`, so the zero observer never measures a model-driven heading.
3. **Remaining path (VWAS side, reported not changed)**: `updateGPSAngle()` derives a
   wheel angle from the bridge heading as well. That is acceptable while psi comes from
   the *positions* (BRIDGE/COAST - a genuine path measurement, as their comment argues),
   but during a gate-active LIVE frame psi is partly the kinematic model. If the steering
   task wants that path closed too, the same one-liner applies there (fall back to the
   encoder while `modelGateActive`).
4. Consequence of (1) that must be watched: with the bias OFF the model is *wrong*
   whenever the VWAS zero is wrong (the logs show ±30 deg between drives), so the model
   gate will fire more often and damp more. That is the intended trade: bounded damping
   instead of a model that silently learns a wrong zero. If the future logs show the gate
   permanently firing, the fault is the VWAS zero, not the gate.

  to the heading quality alone; the option stays for a receiver whose count IS meaningful.
* The geometry tool gates on `-MinHeadingQ` (col 8) instead of a satellite count, and
  `-MinSats` defaults to 0.
* Consequence to keep in mind for the 4x/1.8x damping: with the count gate off, the
  factors now depend on `headingQuality` alone, which is exactly the signal that went
  bad under the trees in the 9-42 drive - so the 1.8x/3x behaviour measured on
  2026-09-27 stays reproducible.

### 26. Reception states, wheel-angle reliability and the behaviour in each (2026-09-28)

Operator decision after the 09-28 drive ("der Traktor in AOG dreht sich im Kreis, obwohl ich
gerade aus fahre" / "die meiste Zeit müssten beide Antennen vollen Empfang gehabt haben und
RTK fix" / "der verwendete Keya Motor setzt 0 immer in der Position, in der er steht, wenn
der Strom angelegt wird"; thresholds: WQ < 30 unusable, > 80 locked; bias clamp stays
+-15 deg; DEADRECKON -> fixQuality 0 only after 2 s of model operation).

**The measured chain (28.9. logs):** at straight driving (|d cog| < 0.6 deg in 0.2 s,
v > 1.5 m/s, LIVE, forward) the wheel angle reported was **-32.8 / -31.2 deg** (n = 15/16
frames), the VWAS showing `off = 0.00` (zero never trimmed) and `fits = 0, armed = 0,
st = 0 (IDLE)` in BOTH drives - the Keya zero was never learned. The model therefore
invented `57.3*v*tan(31)/L` = **50 deg/s** of yaw, the model gate fired permanently
(`mdev` p50 17.6 deg, `mgate` 24 % / 48 % of the frames), R doubled, psi leaned on the
model and rotated (-7 deg/s while LIVE, -58 deg/s in DEADRECKON) although both receivers
were RTK fixed. Additional amplifiers: `rev` flipping to 1 while driving forward (180 deg
mirroring of the path sources, `rej = cogX`) and `lastReject = cogX` throwing the bridge
out of LIVE (measured mount offset -103.5 deg against the configured +90, with unchanged
antennas).

**Decision criteria** (all from the code):
| Signal | Criterion |
|---|---|
| heading quality | `headingQuality == 4` (else 1.8x, not trusted 3x on R) |
| attitude plausibility | no zeroed sentence, roll envelope/step 8 deg/100 ms + 2 deg/(m/s), heading step 5 deg + 12 deg/(m/s*0.1 s) + 3 sigma, baseline +-5 %, COG cross-check, fresh within `gapThresholdMs` (~250 ms) |
| path (fix to fix) | fixQuality > 0, age <= 500 ms, HDOP <= 3.0, step <= 0.5*v*dt + 0.4 m, net >= 0.5 m, straightness >= 0.90 |
| COG (VTG) | age <= 500 ms, fixQuality > 0, v >= 1.0 m/s, 3 sample circular average |
| **wheel angle reliability WQ** | VWAS `getAngleQuality()` 0...100: `fits`, fit deviation, `armed`, `ZO_*` state; **0 = zero never learned**, >= 80 = locked. NOTE (operator 2026-09-28): this only applies to the VIRTUAL WAS - a real physical WAS (analog input or a CAN WAS) has its own hardware zero and reports **100 from the first cycle**, so the bridge never enters the learning fallback on such a machine |

**The three reception cases**
| Case | Criteria | State | What drives the heading | Model needed? | WQ critical? |
|---|---|---|---|---|---|
| 1 good | dual heading qf 4, gates pass, RTK fix | LIVE | `att` (10 Hz) + cross-checks | only between measurements | no - the measurement rules |
| 2 medium | no HPR/HDR (or rejected), position fix with scatter, HDOP <= 3 | BRIDGE then COAST | `path` (positions!) and/or `cog` | **yes - bridges the fix gaps** | **yes - a wrong wheel angle drags it** |
| 3 bad | no heading, fix scatters/jumps, no COG | DEADRECKON -> TIMEOUT | **the model only** | everything | **yes - 100 % model = 50 deg/s error** |

**Behaviour matrix (built)**
| | WQ good (zero learned) | **WQ bad (freshly powered up)** |
|---|---|---|
| LIVE | gate ON (relative, damping only), bias OFF, output smoothing in poor phases only | **gate OFF** (it would damp the very measurements the VWAS needs), **bias ON** (absorbs the unknown zero, clamp +-15 deg) - the heading stays a measurement, so the VWAS can learn |
| BRIDGE/COAST | model bridges, gate ON, VWAS gets psi (position based = genuine measurement) | model bridges WITH bias, gate OFF, the VWAS may still learn (psi is position based here) |
| DEADRECKON | heading is model only -> fixQuality 0 towards AOG after `drQf0AfterMs` (2 s), before that passed through; never to the VWAS (it only samples LIVE) | same, but the bias keeps the error bounded |
| TIMEOUT | TimeoutAction (LAST_VALID_QF0) | same |

Crossing from bad to good the bias **decays to zero** (0.995 per tick) so that the two
mechanisms never correct the same offset at once.

Measured separation of the two populations with the new number: fits = 0 -> **WQ 0**
(28.9., both drives) against fits 66/117 with 0.68/0.96 deg fit deviation -> **WQ 86/81**
(27.9.) - the thresholds 30/80 need no tuning.

New in the raw log (columns 57/58): `yr` = the yaw rate the MODEL used (deg/s - the direct
signature of a wrong wheel angle: 50 deg/s while driving straight is exactly the "tractor
turns in circles in AOG" of 2026-09-28) and `wq` = the wheel angle reliability
(-1 = not reported, 100 = physical WAS, 0 = VWAS zero never learned). `sc_b` is now the
bias the model APPLIES (0 with a good WQ, up to +-15 deg while learning).
`VWAS zero:` gained `wq=`, and the VWAS page shows the number plus what the bridge does
with it.

### 27. One wheel angle for everything (operator correction 2026-09-28, steering task)

**The correction that started it:** "POGi / panda enthalten keinen Lenkwinkel, nur GPS Daten
aus GGA und VTG, sowie IMU daten oder aus Dual Antennen. Der Lenkwinkel sollte nur in der
steer PGN sein" - plus "AOG bekommt den lenkwinkel per PGN aus dem teensy modul. AOG sieht
die Quelle nicht, es ist die Aufgabe der firmware den lenkwinkel zu liefern, egal woher. Es
gibt aber auf anderen Fahrzeugen jede Kombination: Keya mit analogem WAS oder mit CAN WAS."
And the request: "Damit könntest Du für die Projektion den Lenkwinkel verwenden, der auch an
AOG gesendet wird. gibt das Sinn?"

Answer: yes - and it fixed a real defect on the way. PAOGI/PANDA carry position, heading,
roll/pitch and yaw rate, **never a wheel angle**; the angle reaches AgOpenGPS in PGN 253
only. So the pair AOG steers with is (heading from PAOGI) + (wheel angle from PGN 253), and
both have to describe the **same** machine: a bridge model fed by a different angle than the
one the firmware delivers is a model of a machine that AOG never saw.

**The defect (two selections in `AutosteerProcessor::process()`).** Until 2026-09-28 the
wheel angle was selected twice per cycle:
* a simple one for the bridge and the monitor at the top of the cycle
  (`fusion ? getFusedAngle() : adProcessor.getWASAngle()`) - it did **not** check
  `isHealthy()`, so it kept modelling with an angle the firmware itself had declared invalid,
  and it ignored `keyaWasSource` (Keya on CAN with source = CAN curve got the CAN angle in
  PGN 253 while the model ran on the analog input);
* the full one near the end (TractorCAN/Keya detection, `keyaWasSource` 0/1/2, `wasOffset`)
  which fed `actualAngle` -> PGN 253 and the steering control.
The two differed whenever `keyaWasSource == 1`, whenever AOG's `wasOffset != 0`, and - the
dangerous case - whenever the fusion turned unhealthy: the delivered angle fell back to the
(unwired, floating) analog input while the bridge kept integrating `v*tan(delta_fused)/L`.

**Built: `AutosteerProcessor::selectSteerAngle()`, called once per cycle** directly after the
VWAS update, i.e. on the same sensor data the two old blocks saw. It selects the source
(Analog / Virtual / CAN curve, TractorCAN + Keya + `keyaWasSource` included), applies AOG's
`wasOffset` for the CAN-curve source, derives the reliability `WQ` **from the source that was
actually selected** and publishes the result in the members `currentAngle` /
`steerAngleOutQuality`. Every consumer now reads those two:
| Consumer | Before | Now |
|---|---|---|
| PGN 253 -> AOG (`actualAngle * 100`) | second selection | `currentAngle` |
| steering control (`angleError = actualAngle - targetAngle`) | second selection | `currentAngle` |
| HeadingMonitor record (`wa`) | first selection | `currentAngle` |
| HeadingBridge model (`v*tan(delta)/L`) | first selection, `isHealthy()` ignored | `currentAngle` |
| `setWheelAngleQuality()` / model gate + bias | first selection | `steerAngleOutQuality` |
The "Wheel angle source: ..." info and the loud unhealthy-fusion error moved into the
selection, so a source change is reported once per cycle by the code that produced the angle.

**WQ rule (final):** 100 for anything physical - analog WAS or CAN WAS (hardware zero, valid
from the first cycle), the VWAS's own DOP-like number for the virtual one, and **0** for the
analog fall-back that only happens when a *virtual* WAS is configured but unhealthy (that
input is unwired here; reporting 100 would make the bridge trust a floating pin). The lone
exception is `keyaWasSource == 0`, where the operator explicitly chose the analog WAS - that
is a real sensor again and reports 100.

**Ackermann moved into the same pipeline (verified against the AOG source).** The factor
(PGN 252 byte 8, `setAS_ackerman`, AOG default 100 = 1.0) used to be applied to `actualAngle`
*after* the bridge had already been fed the raw angle. It is now applied inside
`selectSteerAngle()` to negative (left) angles, before publishing. Verified in
`SourceCode_AgOpenGPS/GPS/Forms/UDPComm.Designer.cs` (PGN 253 case) that the PC takes the
angle exactly as it arrives - `mc.actualSteerAngleDegrees = (Int16)((data[6] << 8) +
data[5]) * 0.01` - with **no** correction of its own (`HScrollBar` `setAS_ackerman` is only
sent to the module; the `AckermannAngles()` in CVehicle.cs is drawing code). So there is no
double correction, the module is the only place the factor is applied, and the bridge model
now runs on the very number AOG's own steer model works with. Consequence to remember when
reading logs: `wa` (HR) and the model input are the *published* angle, i.e. left turns are
scaled by `setAS_ackerman / 100` - with the AOG default 100 they are identical to the sensor
angle.

**What to watch on the next drive:** `wq` should move 0 -> >= 40 -> >= 80 as soon as the
VWAS learns its zero (analog/CAN machines: 100 from the first frame), `sc_b` shows the
learning bias and then decays, `yr` shows the model's yaw rate (50 deg/s while driving
straight = a wrong wheel angle), `mgate` fires only after the switch. If `wq` stays 0 and
`sc_b` sits at its +-15 deg clamp, the VWAS learning chain is the fault (its next step is
using the raw receiver heading instead of `psi` - see 25c), not the gate.

### 28. The 2026-09-29 logs: jumps, "reverse while driving forward", new log columns (built)

Operator report on the three 09-29 logs (bäume, v-r mixed, brücke 2x): "das heading läuft
überwiegend ruhig", jumps in AOG under the trees and under the bridge, and "AOG zeigt
Rückwärts, obwohl ich vorwärts fuhr". After the discussion: "Traktor springt auf der Karte,
richtung stimmt - wenn Richtung nicht stimmt, dann nur im kurzen Moment, wenn der Empfang
zurückkommt, bereits nach der Brücke."

Measured with the new tool `tools/heading_bridge/jump_diagnosis.ps1` (one run per log; it
separates POSITION from HEADING independently of the firmware, rebuilds the travel direction
from the positions and reads out the new columns 60...69):

| | bäume (06:24) | v-r (06:31) | brücke (06:34) |
|---|---|---|---|
| position jumps (step > v*dt + 0.5 m) | **0** | 9, 4 of them reached AOG | 10, **4 reached AOG** (max 2.86 m at 5.03 m/s) |
| heading jumps (> yaw-rate allowance) | 60, all inside one real 6 s turn | 76 (44 measurement-supported) | 16 (10 measurement-supported) |
| filter-only steps (raw dual heading frozen) | 21, max 4.0 deg (COG phases, qf 0) | 37, max 63 deg (deadreck) / 15.5 deg | 12, max 4.2 deg (COG phases) |
| source switch att->cog step | 3.4 deg | 11.1 deg | 4.1 deg |
| step when the first attitude arrived (t=17.8 s) | 66 deg | 119 deg | 119 deg |

So: **under the trees it is the HEADING** (the 1...4 deg steps happen exactly when the dual
heading drops out - qf 0, sats 0 - and the bridge follows the VTG course, which under a
canopy is 10...25 deg off; measured cog deviation from the true travel direction p50 9.2 /
p90 14.2 / p99 25.8 deg). **Under the bridge it is the POSITION** (2...3 m steps that
reached AOG because the published point is the receiver's own unless the fall-back runs).

**The "reverse while driving forward" chain (v-r log).** `rev` was 1 for 50 s, and on
straight stretches at v >= 1.5 m/s 71 % of the 1 s windows showed |psi - travel| > 150 deg,
i.e. the machine really WAS reversing - the DECISION was mostly right. Broken was what
happened around it:
* the flip happened at **0.47 m/s** with a 0.25 m evaluation baseline, where a 0.4...0.7 m
  multipath step still passes the speed check (measured in the log);
* while `rev=1` the fused heading walked up to **124 deg away from a rock-steady qf=4
  antenna heading within 1.2 s** (trace 132.4...133.6 s: `att` constant 206 deg, `yr` only
  0.3...1.1 deg/s, IMU columns 0, `sc_b` 0, `mgate` 0) - psi rotated monotonically at
  ~140 deg/s and took the LONG way around the 180 deg ambiguity;
* the published heading therefore pointed backwards for seconds, and AgOpenGPS' own reverse
  detection (`Position.designer.cs:738`, travel vs heading, 2 rad = 114.6 deg, distance
  triggered and sticky) latched it - that is the "reverse while driving forward", and it
  clears only when reception returns and the two agree again (exactly the moment the
  operator described).

**Built (HeadingBridge, 2026-09-29):**
1. **Heading re-seed** (`applyMeasurement`): an ATTITUDE measurement more than
   `reseedBeyondDeg` (120) away from psi means the STATE is wrong - psi is re-seeded
   (p11 = 4) instead of integrating the 180 deg back around the long way, loudly logged and
   counted. kind 1/2 (path/COG) may be 180 deg off by definition and never re-seed.
2. **Direction witness** (`updateDirectionState`): the reversal is additionally checked
   against the ONLY heading that is never mirrored - the raw dual antenna
   (`attWitnessDeg`, the body heading of the last trusted frame). REVERSE is only accepted
   if the witness agrees (> `reverseConfirmDeg` 120 deg), and it is CLEARED immediately when
   the witness says forward (< `reverseClearDeg` 60 deg) - forward always wins, so a wrong
   reverse cannot hold itself in place. Without a dual heading nothing changes.
   `minSpeedForDirection` 0.3 -> **1.0 m/s** (below that a 0.25 m position delta says
   nothing at all), and votes no longer carry over between unrelated frames.
3. **Position continuity** (`updateProjection` + new `catchUpRampMs`): the published point
   must never jump.
   * The re-seed after `projValid == false` (fix quality 0 / position too old) used to snap
     onto the fix - it now starts from the PUBLISHED point and ramps onto the fix.
   * An accepted fix farther from the published point than the catch-up limit allows is
     ramped instead of resynced; the ramp time comes from the residual
     (T = 3*residual/`posMaxCatchUpMps`, capped by `posRampMaxMs`), so the blend's INITIAL
     rate is exactly the allowed catch-up speed (1.5 m/s) - a 2.9 m jump becomes ~1.9 s of
     smooth travel instead of ~29 m/s in one 100 ms frame. The existing hand-over ramps
     (RTK recovered, budget spent, fix usable again) use the same formula now.
   * Fix quality, satellite count and speed stay the receiver's own, so AgOpenGPS' own
     lost-RTK handling is unchanged (as documented for the fall-back).
4. **Cold start** (`update`): before the FIRST trusted attitude frame the module published
   dead-reckoned psi - the 66...119 deg step at t = 17.8 s in ALL three logs. While a usable
   receiver azimuth exists it is passed through instead (which is what normal operation
   publishes anyway), so the step is gone by construction.
5. **Log columns 60...69** (operator request "ergänze das Log um die Felder"), HR now has
   69 columns and HR_HDR is updated: `psi_att` (psi - attitude-derived body heading: must
   stay CONSTANT; a swing is what AOG turns into "reverse"), `innov` (innovation applied
   this tick, signed deg), `gain` (the Kalman K1 applied to it), `meas` (which measurement
   fed the filter: 0 none, 1 attitude, 2 path, 3 COG, 4 IMU, 5 projected), `travel` (the
   direction of travel the decision worked with), `raw_step` (fix-to-fix step of the
   receiver, m), `pub_step` (step of the PUBLISHED position - the jump AOG actually sees),
   `publim` (1 = the projection is published this tick), `hdg_res` (heading re-seeds),
   `dir_veto` (reversals refused by the witness). HB2 gained `psi_att/res/veto`.
   `tools/heading_bridge/jump_diagnosis.ps1` evaluates all ten and still reads old logs.
6. **RAM1 note:** the two functions that grew (`applyMeasurement`, `updateProjection`) are
   `FLASHMEM` now - the ~1 kB of extra ITCM code had pushed `.text.itcm` over a 32 kB
   boundary, which turned into 31.9 kB of alignment padding and a failing link ("program
   exceeds memory space"). After that: RAM1 free 8608 B (9632 B before), FLASH 373528.

**Still open (NOT touched - steering task's file):** `WheelAngleFusion` derives the wheel
angle from `headingDelta / distance` with an UNSIGNED distance (lookback over
`gpsHistory`). While reversing, the same wheel angle produces the opposite heading change,
so the derived angle has the WRONG SIGN - and its zero learner then sees a model that does
not apply. The bridge already knows the direction
(`HeadingBridge::Status::reverseDirection`), so the fix is small, but it changes the wheel
angle that reaches AgOpenGPS - needs the operator's go-ahead.

### 29. Single-antenna support: IMU-anchored heading, roll, log columns (built 2026-09-29)

Operator decisions of the same day: "der BNO ist im RVC modus, da das Magnetometer schlechte
Ergebnisse lieferte", "AOG nimmt das heading aus VTG als langsames Signal, kurzzeitig das
IMU als Differenz - so wird das heading reaktiv und über GPS langfristig richtig. Kannst Du
das verifizieren?" -> "PANDA lassen (AOG fusioniert die rohe BNO selbst), unsere Fusion nur
für VWAS", "der TM171 bleibt wie er ist (sonst funktioniert er nicht gut)", "Jetzt umbauen,
aber die capability nicht umsetzen".

**Verified against the AOG source** (`GPS/Forms/Position.designer.cs`), three modes selected
by "Heading from which source":
| mode | slow/absolute signal | what is USED | offset learning |
|---|---|---|---|
| Fix | position fix-to-fix (`atan2(dE,dN)`, step >= `minHeadingStepDist`, default 0.5 m) | `fixHeading = imuCorrected` | `imuGPS_Offset += gyroDelta * fusionWeight` (`:439-458`) |
| VTG | VTG course (only `avgSpeed > 1`) | same | same |
| Dual | our delivered dual heading (`:701`) | - | - |
The "slow + fast" model is literally in the code: when the position has not advanced far
enough the block is skipped (`goto byPass`) **and `fixHeading = imuCorrected` is still set**
- the comment says `//use imu as heading when going slow` (`:560-569`). Two more details that
matter: AOG corrects the POSITION with `tan(roll) * AntennaHeight` along the heading
(`:304/:345`), so it needs roll; and while `isReverseWithIMU` the offset learning is slowed
to 0.02 (`:455`) so a wrong reversal cannot corrupt it.

**What was built (PANDA stays - AOG keeps fusing its own IMU/GPS):**
1. **INIT is no longer dual-antenna-only** (`updateState`): `haveAnyHeadingLatch` is set by
   the first ACCEPTED measurement of any source (set inside `applyMeasurement`), so a
   GGA+VTG machine (F9P, UM981 single) reaches BRIDGE/COAST/DEADRECKON instead of sitting in
   INIT forever. LIVE still requires a real BODY attitude (dual antenna / INS);
   `Status.hasBodyAttitude` mirrors that for the consumers.
2. **`NAVProcessor`** published `gnssData.dualHeading/dualRoll` (both EMPTY without a dual
   antenna) as long as `!everTrusted` - i.e. forever for a single-antenna machine. The branch
   now keys off `!haveAnyHeading`, so the bridged heading reaches AOG as soon as the filter
   has one.
3. **The VWAS gets the fused heading**: with the state leaving INIT, `bridgeUsable` is true
   and `WheelAngleFusion` stops falling back to the raw COG (`headingTrue`). Which source it
   used is now logged (see 5). This was the main motivation - the wheel-angle learning lives
   off that heading.
4. **The IMU heading is anchored to the GPS** (`update`): its offset was only learned while a
   dual attitude was trusted. Now a PATH/COG measurement counts as an anchor too
   (`Config::imuAnchorHoldMs`/`imuAnchorMinSpeedMps`, set where such a measurement is
   accepted), and the yaw-rate bias learning uses the same condition. That is AOG's
   `imuGPS_Offset` construct - only the *change* of the magnetometer-assisted RVC yaw between
   two GPS anchors is used, so its absolute error is absorbed by the offset.
5. **Roll from the IMU** (`updateRoll`): source chain dual -> IMU (fresh/valid/quality) ->
   hold/fade, with `Status::rollSource` in the log. Without a dual antenna the roll used to
   fade to zero, which cost the VWAS its roll-binned zero learning and left AOG with roll 0.
   The IMU's mounting/axis handling stays as it is: IMUProcessor applies `isUseYAxis`
   (PGN 251 bit 3 -> ConfigManager) to the BNO ONLY - the TM171 is deliberately untouched
   (operator: "sonst funktioniert er nicht gut"), and the misleading comment in the header
   that claimed it applies to both sensors is corrected.
6. **The TM171 trap is closed** (`useImuRate`): the TM171 has NO yaw rate (IMUProcessor sets
   `yawRate = 0`), while `useImuRate` defaults to ON - the bridge would have integrated a
   constant fake rate `(0 - bias)` into psi until the `|bias| > 5` warning disabled it. The
   rate path now checks the IMU TYPE (`IMUProcessor::getIMUType()`), sets `imuFlags` bit 3
   and ignores the request.
7. **The reversal decision got a position witness** (`updateTravelWitness`): the direction of
   the NET DISPLACEMENT over >= 1 m at speed, on a straight window. It cannot say forward or
   reverse (both heading and travel can be mirrored), but it says whether the travel VECTOR
   an evaluation is based on deserves to be believed - the 06:31 flip was decided on a 0.25 m
   baseline at 0.47 m/s. A reversal (and its re-confirmation, `Config::revReconfirmMs`) now
   requires a solid travel measurement whenever one exists; while the machine crawls the
   verdict is kept, so a genuine slow reverse is never cut short.
8. **Heading re-seed extended** to the IMU heading (kind 3), but ONLY when the position
   witness agrees with it (`Config::reseedImuConfirmDeg`) - a wrong IMU offset must never be
   able to pull the state.
9. **HR log: 81 columns** (was 69). New: `psi_imu` (psi - offset-corrected IMU heading: the
   same pass-through check as `psi_att`), `imu_flags` (1 ok/fresh, 2 heading fed the filter,
   4 rate drives the model, 8 rate NOT supported by this IMU type), `imu_age_ms`, `imu_q`,
   `imu_hdg_raw` (what AOG receives in PANDA), `imu_roll`, `vwas_hsrc` (0 bridge/1 dual/2 COG
   - what the VWAS used), `roll_src`, `cog_ok`, `travel_wit` (the position witness), `nav`
   (0 NONE/1 PANDA/2 PAOGI) and `gps_flags` (dual/INS/GPS/fix). The printed `HR_HDR` had
   been missing the six names of the 2026-09-27/28 additions (`sc_b,mdev,mgate,yr,wq,hdop`) -
   fixed, so the CSV header now matches the data line (80 values + "HR").
   A second startup line prints the whole heading-fusion configuration
   (`useImuHeading/useImuRate/imuRateInvert/estimateSideSlip/imuAnchor/travelWitness/
   reseedBeyond/revReconfirm`), and IMUProcessor already prints "IMU Type: ..." - so a log
   file documents its own setup.

**Not built on purpose** (operator): the IMU **capability block**
(`hasHeading/hasRoll/hasYawRate/...`) - the type check in (6) covers today's need; the AOG
interface stays PANDA (AOG fuses its own IMU/GPS); the axis/sign handling stays BNO-only.
Note on the dual case: `haveAnyHeading` is set by the ATTITUDE measurement too, so a
dual-antenna machine behaves exactly as before (the first trusted HPR frame still switches
INIT -> LIVE, and NAVProcessor's fallback branch is left in the same tick). The only
difference: if the machine drives before the first HPR frame is trusted, a validated
PATH/COG measurement may publish the bridged heading instead of the receiver's raw one -
which is the point of the change (measured 2026-09-27: 46 s of INIT published a constant
270 deg).
Verified: `HR_HDR` carries 80 names and the data line 80 format specifiers (81 columns with
"HR"); `tools/heading_bridge/jump_diagnosis.ps1` was tested against an 81-column file and
reads every new field (nav/VWAS source/roll source/IMU flags/psi_imu/travel witness), while
old 59-column logs still work.
Still open from §25c: `WheelAngleFusion`'s curvature uses an unsigned distance, so the
derived wheel angle has the wrong sign while reversing (steering task's file).

### 29b. Steering task fixed: the sign of the GPS-derived wheel angle (2026-09-29)

Operator: "ja, den steuer task noch fixen."

**The defect** (`WheelAngleFusion`, §25c): the GPS-derived wheel angle is
`atan(curvature * wheelbase)` with `curvature = headingDelta / distance`, and `distance` is
the UNSIGNED lookback length. The same wheel angle turns the vehicle the OTHER way while it
moves backwards (`omega = v*tan(delta)/L`, v taken as a magnitude) - so the derived angle has
the wrong SIGN in reverse. It is used in three places:
* the Ackermann candidate (`headingDelta`, the `vehicleType != 2` branch),
* the articulated candidate (`totalDelta` against a fixed reference - same physics, no
  distance involved),
* **the zero-point observer's model value**: `zoStraightBaselineAngleDeg` is `bA` in the ZO
  state machine (`WheelAngleFusion.cpp:~1509`). The comment there claimed "only the magnitude
  is used (as a gate)" - that was wrong (the magnitude is *also* a gate at `ZO_STRAIGHT_GATE_LONG`,
  but the VALUE drives the zero learning), so a reverse manoeuvre fed the observer a model
  with the wrong sign. That is the part that would have corrupted the learned zero.

**Built:** the bridge's reversal verdict is the available substitute for the sign of `v` (no
receiver reports it, VTG speed is always positive). One snapshot per cycle
(`bst.reverseDirection` / `bst.dirChanging`) and then
* `if (reversing) dPsi = -dPsi;` for the ZO baseline,
* `if (reversing) headingDelta = -headingDelta;` for the Ackermann candidate,
* `if (reversing) totalDelta = -totalDelta;` for the articulated candidate,
* and while a direction change is in progress (AOG's byPass, `dirChanging`) the affected
  cycle is SKIPPED (`haveHeadingDelta = !dirUncertain`) instead of converting a heading delta
  with an unknown sign.

**Limits (honest):** the verdict comes from the bridge, so it is only as good as the
direction detection (§28 hardened it with the attitude witness, the position witness and the
re-confirmation). For a machine with ONLY GPS (no IMU, no dual antenna) there is no
independent witness at all: `psi` is built purely from travel-direction measurements, so
`|psi - travel|` stays ~0 and REVERSE is never declared - which also means this sign
correction has no source there. In that configuration AOG owns the direction (it detects the
reversal from the path itself), and the VWAS would need an encoder-based detector
(`sign(dpsi_gps) vs sign(encoderAngle)`), which is NOT built.

---

## 30. The 2026-09-30 logs: "heading schräg nach dem Start", trees, bridge (P1 built)

Operator report in the morning of 2026-09-30, four observations:
1. after every start the heading is oblique and it takes a while until the tractor drives
   straight on the screen ("the UM982 heading should be right almost immediately"),
2. under the trees the steering wobbles strongly - also without autosteer,
3. under the bridge the reception drops, the projection is medium good, the heading wobbles
   briefly and the POSITION jumped,
4. after the bridge, while the reception comes back, there are jumps and wobbles in the heading
   although the drive was steady.

Positive: the wheel-angle zero is reached quickly, and in the hall the output is reasonably
stable given the bad reception.

### 30a. What the logs measure (4 logs, ~7 500 HR frames)

| Observation | Measured cause |
|---|---|
| (1) heading oblique after start | `2026-09-30 6-23 Start in Halle`: for t = 15…92 s there is NO fix (`fixQ` 0/1/2) and the UM982 sends a zeroed HPR (`headingQuality` = 0, `att` = 0.0000). The bridge has nothing, so `NAVProcessor` forwarded exactly that 0.0000 (branch `!haveAnyHeading`) and AOG added its own dual offset → the icon showed ~90 deg while the machine stood at ~275 deg (the later accepted value gives `veh=274.51`). Then t = 93…127 s: the HPR became valid (`QF=5`) with a STABLE heading (94.8…95.8 deg) but a `roll` of **-35.5 deg**, and the roll envelope rejected the WHOLE attitude **311 times** → 30 s of COG-only heading (sigma ~19 deg) before the first trusted attitude at t = 127 s. |
| (2) trees | The dual heading itself moves: 42 heading jumps, 30 of them with the raw attitude moving (up to 5…6 deg/100 ms = 50…60 deg/s at 2.5 m/s straight ahead) and our gain follows (up to 0.76). In straight, fast windows `psi - travel` is p50 **+2…+4 deg with sd up to 12 deg** → NOISE, not a bias (an earlier "10 deg bias" reading was an artefact of curvy windows: the antenna lever-arm term `atan(omega*a/v)`). Under the trees there was also a `wq`-driven model error (see 5). |
| (3) bridge | 5-6 POSITION jumps of 0.9…3.7 m in 100 ms; **2 caught by the fall-back, 3-4 REACHED AOG**. Cause: `fixUsable = status.posRampActive || …` (position limiter) - **the spike test is disabled while the hand-over ramp runs**. Evidence `Halle 2x` t = 146.4 s: `d_pos` 3.66 m at 1.96 m/s (allowed 0.20 m), `posfall=0`, `fixQ` 4→5→2. The published psi itself was stable under the bridge (`att=61` garbage, `psi=266.8`, `az_out=177.0`, `hdQ=1.5`). |
| (4) after the bridge | 8 heading jumps were FILTER-ONLY (`d_att = 0.00`, attitude frozen/invalid) up to 9.4 deg/100 ms, all with `src=cog` at 1.1…1.3 m/s: the COG is worthless at that speed (in `Halle 2x` psi followed it by 220 deg at ~1 m/s). Our own doing, not the receiver's. |
| (5) not reported, but the biggest single error | `Halle 2x` t = 20…75 s: `wheel = +34 deg` with **`wq = 0…13 %`** (the VWAS zero was not learned yet) while driving STRAIGHT at 7.65 m/s → the model invented **86 deg/s** (`yr` up to 100) → `psi` was **52 deg** off the direction of travel (which the COG and travel witness reported correctly) while publishing `sigma = 4.2 deg`. Mechanism: with `wq < wheelQualityLowPct` the model gate is switched OFF on purpose (§26 - it would damp exactly the measurements the VWAS needs), but the measurement weights stay small, so the (wrong) model wins: stationary offset ≈ (model yaw per 100 ms)/gain ≈ 8.6/0.3 ≈ 29…52 deg. The invented rate also entered `az_out` as the rate lead (86 deg/s * 0.2 s ≈ +17 deg). |

Confirmed positively: the wheel-angle zero is reached (wq 0 → 65 → 86 %), and in the hall with a
valid attitude `psi - travel` is +0.2 deg.



### 30b. Offline verification BEFORE the change (operator request)

`tools\heading_bridge\p1_whatif.ps1` replays the P1 rules on the raw log columns
(`-Log <log> -Offset 90`). `|psi - travel|` in straight, fast windows (position chord as the
truth, `|dCOG| <= 4 deg`, v >= 2 m/s):

| Log | LOG p50 | LOG p90 | SIM p50 | SIM p90 |
|---|---|---|---|---|
| 6-44 Halle 2x | 15.3 | 53.1 | **1.6** | **5.9** |
| 6-23 Start in Halle | 1.5 | 4.4 | 1.6 | 6.0 |
| 6-36 Bäume | 1.6 | 5.1 | **1.5** | **3.7** |
| 6-41 Brücke | 2.5 | 10.6 | **1.7** | **3.7** |

Mechanism per phase (`wq < 30`): the model invented p50 16…44 deg/s (max 100), while the
measurement the EXISTING gates had already accepted was p50 0.9…1.9 deg accurate - including the
start log, where the raw attitude was 11 deg p50 / 88 deg p90 off but the log's `src=cog` was
1.8 deg. **That is why the fix is "no invented rotation", not "trust the measurement blindly"** -
the first, naive replay (following the raw attitude) made the start log WORSE (p90 4.9 → 84.9).

Roll-envelope runs (`P1a`), judged per run against the VTG course and the position chord:
`6-23` 1 run, heading 74 deg off (7 sats, `fixQ` 1-2) → **rejecting was right**;
`6-36` 30 runs: **13 with a heading within 0.3…9 deg** (one 76 frames with `|roll|` 66 deg,
26 sats, deviation 0.4 deg - a good attitude thrown away) and 17 that were 12…135 deg off;
`6-41` 2 runs, both 74…85 deg off. So the roll must not condemn the heading, but the heading
must be verified by an independent source.

### 30c. Built (P1, operator decision "P1 umsetzen")

1. **P1a - roll envelope decoupled from the heading** (`HeadingBridge::evaluateAttitudeFrame`,
   reject step 3): `|roll| > maxRollAbsDeg` no longer rejects the frame by itself. The frame is
   accepted when an INDEPENDENT source confirms the heading - the same BODY-vs-PATH cross-check
   as step 6, with the same reverse/undecided handling - `cogValid && v > rollOutlierRefSpeedMps
   (2.0)` and `|vehicleHeading - course| <= rollOutlierMaxDevDeg (15)`. Otherwise (slow,
   standing, no fix, or the check fails) it stays `ROLL_ENVELOPE` - exactly the runs the logs
   judged "heading BAD". The roll is NOT published from such a frame: `updateRoll()` has its own
   `|dualRoll| <= maxRollAbsDeg` test and holds/fades (visible as `roll_src = 2/3`). The re-entry
   settle test accepts a verified outlier (`|| rollOutlier`) but still rejects a JUMPING roll.
   One NOTICE per episode.
2. **P1b - the model must not invent rotation** (`update()`): while the wheel angle is
   untrustworthy (`quality < wheelQualityLowPct`) the kinematic yaw rate is not used
   (`yawRateZeroed`), the steerScale/gate accumulators are not fed, and the process noise becomes
   `qHeadingWheelBadDegS = 25` instead of `qHeadingDegS = 4.7`. Two effects: **sigma becomes
   honest** (AOG gets no sigma in PAOGI and trusts the heading blindly, so this matters for the
   consumers and the log) and **the KF gain rises**, i.e. psi follows the accepted measurements
   instead of the invented rate. Fully reversible - each accepted measurement shrinks `p11`
   again. The VWAS zero learning is NOT affected (it works from encoder + GPS angle, not psi).
3. **Two new HR columns 82/83** (`roll_susp`, `yaw_zeroed`) and the matching block in
   `tools\heading_bridge\jump_diagnosis.ps1` (old logs still parse).

Not built in this step (still open, next priorities): the POSITION limiter hole of 30a (3)
(the spike test must not be suspended while the ramp runs), the fake heading of 30a (1)
(`NAVProcessor` must not forward a receiver heading that the receiver itself flags invalid),
the COG speed gate of 30a (4), and the `UNIHEADINGA` parser (`length`, `hdgstddev`, `sol stat`,
`#solnSVs` - the baseline length monitor is DEAD with the UM982: `baselineLengthM` is only fed
by the u-blox UBX path, so column 25 reads 0.000 in every UM982 log).

### 30d. Vehicle facts confirmed by the operator (2026-09-30)

* The two antennas are mounted **left/right (crosswise)**, which is why the measured mounting
  offset is ~-89 deg (`az - front axis`). AOG is set to **+88**, the module to **+90**
  (`dualHdgOff=+90.0deg (AgOpenGPS value)` in the start-up log) → the value AOG finally uses is
  `psi + (88 - 90) = psi - 2 deg`, a permanent 2 deg bias that is easy to remove by setting both
  to the same number. AOG's roll display is correct with this mounting (checked by the operator).
* **No IMU is installed at the moment** (`No IMU detected` / `IMUProcessor FAILED` in all four
  logs, `imu_flags = 0`, `imu_age = 0`). The "sensor fusion" for (2) therefore has no gyro: the
  fastest source is the wheel-angle model, which is exactly why P1b couples its trust to `wq`.

### 30e. What to check on the next drive

* `yaw_zeroed` (col 83) should be 1 exactly while `wq` is low and 0 afterwards; `yr` must read 0
  in those frames (the invented rotation is gone) and `sigma_deg` must RISE there (honest) -
  previously sigma stayed at ~4 deg while psi was 50 deg off.
* `roll_susp` (col 82) should appear only with a `roll_src` of 2/3 and a heading that matches
  `cog_ok`/`travel_wit`; a NOTICE "roll envelope: heading KEPT" in the log marks each episode.
* `|psi - travel|` in straight fast windows should stay in the single-digit range in the
  `wq`-low phases (was p50 27.8 deg in `Halle 2x`).
* **P2 (build 2026-09-30):** `rl_clip` (col 85) should be 1 exactly in the frames where a
  single-frame peak used to arrive, and `|d psi|` per 100 ms must stay near `rl_allowed`/10 there
  (`jump_diagnosis.ps1` prints both). `rl_allowed` (col 84) must FALL with speed and with the wheel
  angle; `rl_path` (col 86) should rise only in accepted turns (it is what keeps the turn start
  from lagging). `meas_r` (col 87) shows the strengthened COG damping: a COG measurement at
  ~1.5 m/s must now carry R ~ 3.8 deg (factor 2 on top of the base R) instead of ~2.5 deg.
  Expected effect on the metrics: TWITCH/1000 should drop from 58 to ~16 (Bäume) and from 44 to
  ~14 (`Halle 2x`) - measure with `straight_twitch.ps1` and compare against the table in §31k.


---

## 31. Follow-up questions of 2026-09-30 (4.1...4.4): limiter gap, "send nothing?",
## COG speed gate, UNIHEADINGA vs heuristic

Tooling for all four: `tools\heading_bridge\p4_analysis.ps1 -Log <log>` (4.2 blind phases, 4.3 COG
quality and jitter per speed bin, 4.4 confusion matrix of the bad-heading heuristic).

### 31a. 4.1 - the position-limiter gap is CLOSED (built)

The hole: the fall-back judged the new fix against what WE published (the projection), and the
catch-up RAMP legitimately differs there - so `fixUsable = status.posRampActive || (...)` skipped
the spike test while the ramp ran. Measured consequence in "Halle 2x": 1.4/1.5/1.9/3.2/3.7 m steps
at 1.9...2.2 m/s reached AgOpenGPS with `posfall=0` (the operator's "Sprung in der Position").

Built: the position window's spike test (fix-to-fix, `maxPosStepMargin * v * dt +
maxPosStepNoiseM`) now sets `Status::posSpikeFix` independently of what is published, and the
limiter reads it: `fixUsable = !posSpikeFix && (ramp || (fixQ > 0 && residual <= allowed))`. If the
ramp was running, it is ABORTED (new episode of the fall-back, counter `posRampAborts`, warning
"ramp ABORTED after %.2fm receiver step"), so the projection is published again instead of
following the impossible point. The ramp has moved at most `posMaxCatchUpMps` * one fix interval
before the abort, so the published track stays continuous. The residual test keeps its ramp
escape - otherwise the release would restart in a loop.

### 31b. 4.2 - "send nothing at all when we have no heading?" - measured answer: NO

| Log | fixQ <= 0 until | HPR QF < 3 until | movement-based heading (src=cog/path) from | COG/path exists how much earlier than the HPR |
|---|---|---|---|---|
| 6-23 Start | 93.3 s | 95.3 s | 103.1 s | - |
| 6-36 Bäume | 16.7 s | 209.2 s | 23.8 s | **185 s** |
| 6-41 Brücke | 16.8 s | 112.0 s | 66.3 s (attitude was valid until ~105 s) | **46 s** |
| 6-44 Halle 2x | 17.1 s | 155.6 s | 95.5 s | **60 s** |

* While `fixQ <= 0` **there is no position either**, so AgOpenGPS is blind regardless of what we
  send - withholding the sentence would not gain a heading, it would only take away the position
  (map, section control) and AOG's own lost-RTK logic. Recommendation: **keep sending the
  sentence.**
* A heading from MOVEMENT already exists and is what the operator asked for: the fall-back
  measurements (`src=cog`, the VTG course, and `src=path`, the fix-to-fix window) are accepted from
  the first metre of travel - measured 46...185 s BEFORE the receiver's own dual heading is valid.
  Nothing new has to be invented; §31c only has to make the low-speed case honest (damping instead
  of dropping), and the travel witness (net displacement >= 1 m, valid in 100 % of the moving
  frames of all four logs) is the natural seed if an even earlier start is wanted (NOT built).
* The remaining cosmetic defect on a cold start is the `!haveAnyHeading` branch of NAVProcessor,
  which forwards the receiver's value even when the receiver flags it invalid (0.0000 -> AOG adds
  its offset -> the icon shows ~90 deg while the machine stands at ~275 deg). "Hold the last valid
  value" cannot help there: this branch only runs on a COLD start, i.e. when no value was ever
  published. What can be added later is a seed from the travel witness (~0.5 s after moving instead
  of ~8 s), which turns "garbage" into "+-10 deg with an honest sigma".


### 31c. 4.3 - the COG speed gate is BUILT (damped, not dropped)

Measured VTG-course error against the position chord, p50/p90 in deg:

| v | 6-23 Start | 6-36 Bäume | 6-41 Brücke | 6-44 Halle 2x |
|---|---|---|---|---|
| 0.3...1.0 | 2.2/5.4 | 2.9/7.7 | 4.0/9.0 | 5.2/13.0 |
| 1.0...1.5 | 3.3/10.8 | 1.8/4.6 | 2.2/10.9 | 4.4/10.4 |
| 1.5...2.0 | 2.4/7.7 | 1.8/5.0 | 3.2/6.9 | 3.5/10.3 |
| 2...3 | - | 2.4/7.5 | 1.4/6.6 | 3.2/11.1 |
| 3...5 | - | 2.0/4.6 | 1.3/6.4 | 2.2/5.6 |
| > 5 | - | 1.2/2.9 | 0.6/2.5 | 0.6/3.0 |

The error grows roughly as 1/v, so the COG is not wrong below 2 m/s, it is badly scaled. Built
accordingly (`selectFallbackMeasurement`): below `cogRefSpeedMps = 2.0` the COG's measurement noise
is multiplied by `2.0/v`, capped at `cogMaxNoiseFactor = 4.0`. The source stays "cog" (no
dead-reckoning penalty, the slow side-slip estimate keeps its input), but the KF gain shrinks with
the SQUARE of that factor - so the COG-driven jitter goes away, and the geometrically exact
fix-to-fix PATH measurement (0.5 m of travel) carries that speed range.

Effect measured as the jitter that is attributable to the COG (|d_psi| per 100 ms, frames with
that source): `Halle 2x` 1.0...1.5 m/s has 77 steps > 1 deg, 27 > 3 deg, 10 > 5 deg at 75 % src=cog
-> damped by 4...16x these become ~1...1.5 deg, i.e. gone. `Brücke` post-bridge (1.0...1.3 m/s,
src=cog) shows the same signature (51 steps > 1 deg). Honest limit: jitter that comes from the
ATTITUDE (trees; 1.5...2.0 m/s in the bridge log: 45 steps > 3 deg with src=cog 0 %) is NOT
touched by this - that is the robust-gain topic of §30c/P2.

### 31d. 4.4 - how good is the bad-heading heuristic? (measured, UNIHEADINGA can wait)

Truth = the position-derived direction of travel (independent); BAD = the receiver's attitude
heading (att + offset) is more than 15 deg away from it, or the HPR is invalid/zeroed. Judged only
where a truth exists (moving + straight = exactly where a bad heading hurts):

| Log | judged | BAD | sensitivity | specificity | precision | missed | missed deviation p50/p90 |
|---|---|---|---|---|---|---|---|
| 6-23 Start | 727 | 248 | 100.0 % | 97.9 % | 96.1 % | 0 | - |
| 6-36 Bäume | 1882 | 327 | 89.6 % | 82.8 % | 52.2 % | 34 | 16.0 / 17.4 deg |
| 6-41 Brücke | 1070 | 203 | 76.8 % | 98.4 % | 91.8 % | 47 | 16.8 / 19.6 deg |
| 6-44 Halle 2x | 1303 | 393 | 86.5 % | 99.6 % | 98.8 % | 53 | 17.1 / 17.7 deg |

Reasons that caught them: `qf` (invalid HPR), `rollEnv`, `settle`, `cogX`, `rollStep`.

**Conclusion:** for DETECTION the heuristic is good enough - 77...100 % of the bad headings are
caught, only 0.4...17 % of the good ones are thrown away, and **every single miss is just above
the 15 deg threshold (p50 16.0...17.1 deg)**, i.e. never a gross error; part of that band is even
legitimate side slip (body vs path is 1...10 deg by definition). The trees log's weak specificity
(82.8 %) is exactly what §30c/P1a repairs (13 of its 30 roll-envelope runs had a verifiably good
heading). **UNIHEADINGA is therefore NOT needed to detect a bad heading.** What it would add and no
heuristic can: (a) the BASELINE LENGTH - the only way to see a wrong integer fix that still reports
QF 4/5 with a plausible heading and roll; note the existing baseline monitor is completely dead on
this receiver (`baselineLengthM` is only fed by the u-blox UBX path, column 25 reads 0.000 in every
UM982 log), and (b) a per-epoch quality (`hdgstddev`) for the WEIGHTING - the adaptive R needs
~10 s to notice a degraded phase, `hdgstddev` would say it immediately.
Recommendation: defer it (data/tooling first if a drive ever shows a missed gross error).

### 31e. What to check on the next drive (additions to §30e)

* `posfall` (col 42) must go to 1 for a fix that steps more than `2*v*dt + 0.4 m` - and the new
  warning "ramp ABORTED after %.2fm receiver step" marks every aborted catch-up ramp, so the
  operator's "Sprung in der Position" should no longer reach AOG (`pub_step`, col 66, stays small).
* COG damping: at 1...2 m/s the COG-driven `|d_psi|` per 100 ms should drop from > 3 deg to
  ~1 deg, and `src=cog` should increasingly yield to `src=path` in that range.


### 31f. Operator follow-up: "did any of the implemented measures reduce the jitter under the TREES?"

**Answer: no - and the first version of this section was WRONG.** It classified "|d_psi| > 3 deg per
100 ms" as jitter. Verified against the positions, the biggest such burst (tree log, t = 82.6...84.7 s)
is a REAL ~100 deg turn: the position chord turns by -24...-48 deg/s, the VTG course by
-66...-108 deg/s, and `att+90`/psi follow the chord within ~16 deg. |d_psi| per 100 ms alone cannot
distinguish jitter from a turn.

Corrected metric (tool `jitter_path_ref.ps1`): **jitter = a heading ROTATION the PATH does not show**,
i.e. `|headRate - pathRate|`, where the path rate is the turn rate of the position chord (0.5 s
before vs 0.5 s after). A rate, so the antenna lever-arm term `atan(omega*a/v)` cancels.

| log | class | n | med |jit| | p90 |jit| | jit > 20 deg/s | jit > 40 deg/s |
|---|---|---|---|---|---|---|
| 6-36 Bäume | att, wq>=30 (attitude, the trees case) | 1036 | **2.1** | **7.7** | 35 (3.4 %) | **0** |
| 6-36 Bäume | att, wq<30 (P1b case) | 236 | 1.7 | 10.4 | 6 | 2 |
| 6-36 Bäume | **cog/path, v<2 (4.3 case)** | 324 | **7.9** | **21.8** | **41 (12.7 %)** | 3 |
| 6-36 Bäume | cog/path, v>=2 | 209 | 4.3 | 17.0 | 16 | 1 |
| 6-41 Brücke | att, wq>=30 | 574 | 1.2 | 14.9 | 24 | 0 |
| 6-41 Brücke | att, wq<30 (P1b) | 285 | 1.2 | 3.7 | 2 | 1 |
| 6-41 Brücke | **cog/path, v<2 (4.3)** | 92 | **10.2** | **23.6** | 16 | 3 |
| 6-41 Brücke | cog/path, v>=2 | 59 | 17.2 | 43.9 | 25 | 8 |

And what AgOpenGPS actually RECEIVES (it gets `az_out` = psi - offset + smoothing + rate lead, not
psi!): `|d az_out|` per 100 ms is p50 0.2 / p90 0.9 deg in the tree log and p50 0.1 / p90 1.2 in the
bridge log - i.e. **smoother than psi itself** (p50 0.3 / p90 1.6). The lead is a STATE, not jitter:
`|(psi - offset) - az_out|` p50 0.4 / p90 2.6 / max 12.6 deg (trees).

**Conclusion:** under the trees there was no significant attitude-driven heading jitter to reduce
(p50 2.1 deg/s, p90 7.7 deg/s, never above 40 deg/s). The measured jitter owner in the low-speed
phases is the **COG/path** class (p50 7.9...10.2 deg/s, 12.7 % of its frames above 20 deg/s) - which
is exactly what 4.3 now damps - and the model-invented rotation in the `wq<30` phases (Halle 2x:
p90 53 -> 5.9 deg, §30b), which P1b removes. So the implemented measures do help the SLOW phases
(after the bridge, in the yard, hall), not the trees. If a wobble under the trees is still seen after
this build, the next suspects are the steering chain (VWAS wheel angle, wq 42...62 there) and AgOG's
own loop - neither is a heading-filter topic.

### 31g. The twitch the operator means (2026-09-30, evening): located, three causes

Operator: "the heading wobbled by only 1...2 deg, but the wheel angle must be steady - that is too
much for AOG when the steering does not move; check the part WITHOUT autosteer, there no
self-amplifying loop can exist."

Located with `find_stops_turns.ps1` (stops at 16.7...22.6 s, 90.2...101.1 s, 106.5...120.0 s,
242...338 s; the tree log): the westbound straight at v = 2.8...8.3 m/s is **t = 30...70 s with
`guid = 0`**, the half circle is t = 81...86 s, the stop 90.2...101.1 s, and after the second stop
the drive back **eastbound with autosteer is t = 140...175 s (`guid = 1`, v ~1.8 m/s, `wq = 42`)** -
exactly the "5...10 s after turning" the operator describes.

`straight_twitch.ps1` (only straight driving: |path turn rate| <= 5 deg/s, |wheel| <= 5 deg, i.e.
the wheel standing still) over the 100 ms steps of `az_out` - the value AgOpenGPS actually
integrates:

| group | n | d_az p50 | p90 | max | exc p90 | exc max | d_att p90 | d_lead p90 |
|---|---|---|---|---|---|---|---|---|
| tree log, guid = 0, wq >= 30 | 355 | 0.00 | 0.50 | **3.20** | 0.40 | **2.83** | 7.02 | 0.66 |
| tree log, guid = 0, wq < 30 | 122 | -0.10 | 0.40 | 2.90 | 0.36 | 2.59 | 0.75 | 0.44 |
| **tree log, guid = 1 (drive back)** | 916 | 0.00 | 0.40 | **4.30** | 0.48 | **3.98** | 4.55 | 0.78 |
| bridge log, guid = 0, wq >= 30 | 487 | 0.00 | 0.10 | 9.40 | 0.19 | 9.09 | 0.53 | 0.26 |
| Halle 2x, guid = 0, wq >= 30 | 296 | 0.20 | 0.80 | 4.40 | 0.65 | 4.27 | 0.80 | 0.91 |

So: on straight driving with the wheel still, `az_out` (what AOG sees) jumps by up to 3.2...4.3 deg
per 100 ms - the operator's 1...2 deg with peaks beyond. Three DIFFERENT causes, all measured:

1. **The receiver's HPR noise.** `d_att` reaches 3.1 / 6.4 / 6.7 deg per 100 ms at 1.8 m/s
   (t = 146.7 / 163.0 / 161.1 s) with `hdQ` 4...5 - i.e. the measurement CLAIMS to be good. Through
   the gain (~0.5) that is 2.5...3.9 deg in psi and the same in az_out. This is the trees case of
   §31f. Only a robust gain (§30c/P2: cap the step per update, require confirmation) reduces it.
2. **The output smoothing's residual cap** (`HeadingBridge.cpp`, `if |resid| > 3.0 ->
   outSmoothDeg = rawAz`): any measurement step larger than 3 deg is not smoothed but SNAPPED -
   so az_out reproduces the jump instead of damping it. Evidence t = 152.9 s: `d_az` -2.90 deg while
   `d_psi` was only 0.18 deg and `yr` 0.5 deg/s; the whole movement came from
   `d_lead` = +3.08 deg (the lead = az_out - (psi - offset)). The wheel did not move (wheel 0.7 deg).
3. **The rate lead from the kinematic model** (lead = yawRate * 200 ms): it follows `yr`, and `yr`
   is only as good as the wheel angle. In these phases `wq` = 42 (above `wheelQualityLowPct`, so
   P1b does NOT stop the model) while the model is still learning (`mdev` up to 10.5 deg, `mgate` 0)
   - a wrong zero point then makes the lead wobble by +-1.4 deg and more on a straight line
   (frames t = 171.9: `d_lead` -2.55 with `mdev` 10.5).
4. **The COG fall-back moves psi while the attitude is rejected** (`hdQ = 0`, `src = cog/COAST`):
   `d_psi` 2.8...3.3 deg per 100 ms with `d_att` = 0.00 (t = 146.2, 152.7, 65.1 s). The 4.3 damping
   only gives a factor of 2.0/1.8 = 1.1 there, i.e. almost nothing.

And the operator's argument is confirmed by the data: the westbound straight (`guid = 0`, i.e. no
autosteer) shows the same 2.8...3.2 deg twitches as the drive back with autosteer - so it is NOT a
control loop, it is the heading chain itself.

### 31h. Operator question: "why always straight ahead? The wheel angle and the yaw rate are PROPORTIONAL - the model predicts, HPR delivers, they fit or get smoothed."

**The operator is right about the method, and my "straight ahead" wording was a limitation of my
MEASUREMENT, not of the system.** Two separate things, both now measured (`Temp\curve_robust.ps1`):

**(1) The model does cover curves** - the code integrates `omega = v * tan(delta_eff) / L` every
cycle (HeadingBridge.cpp, prediction step) and compares it with whatever HPR/COG/path delivers. No
straight-line assumption anywhere in the filter.

**(2) Why my truth checks had to use straight windows:** the independent reference (position chord,
VTG course) is a PATH direction, and in a curve the ANTENNA's body heading differs from its path
direction by a kinematic term `atan(omega * a / v)` - not an error. Measured in curves
(|path turn rate| >= 6 deg/s, trusted frames only): `attitude - chord` p50 = **15.4 deg** (tree log),
16.7 (Halle 2x), 4.2 (bridge) - against p50 1.4...2.8 deg on straight sections. A lever-arm sweep
(`att - chord - atan(omega*a/v)`) puts the minimum at **a ~ -1.7...-1.9 m** and drops the residual sd
from 3.6/6.8 deg to **2.1/2.7 deg** - so the 15 deg IS the arm and what is left (2...3 deg) is side
slip plus real error. On a straight line the arm term vanishes, which is why a straight section is
the only place where the path direction can judge the body heading without corrections.

**(3) The finding that actually matters: the proportionality constant is wrong right now.**
`k = omega_path / (v * tan(wheel) / L)` on trusted curved frames:

| log | k p10 | k p50 | k p90 | abs(path)/abs(yr) p50 | abs(wheel) p50 | steer scale sc |
|---|---|---|---|---|---|---|
| 6-36 Bäume | 0.33 | **0.42** | 0.52 | 0.42 | 40.3 deg | 1.51 |
| 6-41 Brücke | 0.35 | **0.69** | 0.87 | 0.69 | 20.9 deg | 2.00 |
| 6-44 Halle 2x | 0.31 | **0.38** | 0.46 | 0.38 | 42.9 deg | 1.00 |

The model over-predicts the yaw rate by **1.4...2.6x**. Cross-check: at 2.5 m/s and a reported 40 deg
wheel angle the geometry demands ~56 deg/s of yaw, the positions show **13 deg/s**. A physical wheel
angle of ~19 deg would explain the 13 deg/s (L = 2.14 m), or a wheelbase of 9 m - so what the model
USES is 2...2.6x too large. The cause is the wheel-angle chain (VWAS zero/scale, the steerScale
estimate sitting at 1.5...2.0, `sc_b` bias -15 in Halle 2x) - NOT the heading filter.

**Consequence for the operator's proposal** ("model predicts, HPR delivers, they fit or get
smoothed"): exactly right, but it can only judge the HPR where the MODEL is trustworthy. With a
2.4x over-turning model the innovation in a curve is dominated by the model error, so the fit test
would blame the receiver for the model's fault - which is why the model gate (`mdev`/`mgate`) fires
so rarely (51 of 3269 frames in the tree log) and why the "is the HPR good" question could only be
answered on straight sections so far.

Two independent ways forward:
* **Fix the wheel-angle proportionality** (VWAS zero + scale + wheelbase); then the model gate
  becomes usable in curves and is the cheapest quality signal we have. Now a concrete measurable
  task: k must come out as 1.00.
* **Add a model-INDEPENDENT quality number for the HPR** - which is exactly `hdgstddev`/`sol stat`
  from `UNIHEADINGA`. Nuance to §31d: the heuristic is sufficient for DETECTING garbage, but in
  curves the model cannot help judging the receiver, so UNIHEADINGA is the cleanest source for
  WEIGHTING there.
### 31i. CPD / max steering angle: the real numbers (2026-09-30, operator question)

Operator: "the real wheelbase is 2.14 m, so probably the MAXIMUM STEERING ANGLE is wrong and therefore
the counts per degree the user set. Are the values enough to set the right parameters? What is the
right cpd? Also check whether the code uses 2.50 m or the real 2.14 m from the web interface."

**A. Wheelbase in the code: the real value is used - with one caveat.**
`HeadingBridge.h:116` defaults to `wheelbaseM = 2.50f`, and `AutosteerProcessor.cpp:403` pushes the
real value:
`if (wheelAngleFusionPtr && configManager.getINSUseFusion()) headingBridge.setWheelbaseM(
wheelAngleFusionPtr->getConfig().wheelbase);` -> the VWAS/AOG (vehicle.xml) value. The boot log
confirms it: `HeadingBridge wheelbase 2.50 -> 2.14 m (VWAS/AOG config)`, and the change is logged
whenever it changes. **Caveat: that call sits inside `if (... getINSUseFusion())`. With fusion OFF
(VWAS disabled) the bridge silently keeps 2.50 m** - worth fixing, because the model then runs 17 %
too weakly.

**B. CPD: AOG owns it, the VWAS only converts.** `WheelAngleFusion.cpp:353-358`:
`counts = rawCounts - centerCounts` (+ AOG's `wasOffset`), `encoderAngle = counts /
getEffectiveCountsPerDegree()`. The logs show `cpd=18.12 aogCpd=18 cal=1`, i.e. AOG's 18 is in
effect, refined locally to 18.12.

**C. My earlier "model over-turns 2.4x" was WRONG - corrected.** That number came from
`curve_proportionality.ps1`, which selected frames by |path turn rate| alone, without requiring a
STEADY turn: transients (wheel just moved, yaw not built up yet, counter-steering) bias it LOW. The
validated tool (`steering_angle_from_circle.ps1`, circle fit on steady frames, headingQuality >= 4,
forward only) says something very different:

| log | tightest R | v | yaw | VWAS wheel | delta_geo | scale VWAS->geometry | cpd should be | max abs(wheel) |
|---|---|---|---|---|---|---|---|---|
| 6-36 Bäume | 2.72 m | 2.41 m/s | -50.6 deg/s | -41.63 deg | 38.15 deg | 0.916 | **109.1 %** (19.8) | 43.3 deg |
| 6-44 Halle 2x | 2.64 m | 1.98 m/s | -43.0 deg/s | -43.47 deg | 39.03 deg | 0.898 | **111.4 %** (20.2) | 44.0 deg |
| 6-41 Brücke | 2.78 m | 1.68 m/s | -34.7 deg/s | -40.68 deg | 37.60 deg | 0.924 | **108.2 %** (19.6) | 40.7 deg |

So at (near) full lock the VWAS angle is **8...11 % too large**; the two-parameter fit in the tree log
gives `k = 0.850, b = +1.00 deg` (residual 12.56 -> 1.76 deg/s), i.e. up to ~18 % in the scale plus a
small offset. **Answer: the right cpd is about 20 (range 19.6...21.3) instead of 18.12, i.e.
+8...18 %, and the physical maximum steering angle is ~38...39 deg.** If AOG's "max steer angle" is set
to 45 deg (which yields exactly cpd ~18 at ~800 counts), that IS the error the operator suspected: the
setting is ~15 % too high, so every reported angle is ~15 % too large.

**D. But the SMALL angles are dominated by the ZERO, not by cpd.** `L_implied = R*tan(wheel)` per
wheel-angle bin: 0...5 deg -> **0.95 m** (Halle 2x) / 1.69 m (Bäume) / 8.08 m (Brücke), while
40...45 deg -> 2.83 / 2.72 / 2.57 m. A constant L_implied means a pure scale error; THIS pattern
(far too small at small angles, converging to ~L at large ones) is the signature of an OFFSET. That
offset is the VWAS zero, and in Halle 2x it was still being learned early (`wq` 0...13 %) - which is
exactly the +52 deg error of §30a (5), NOT a cpd problem.

**E. Recommendation (concrete).**
1. Set AOG's **max steer angle to the measured ~38...39 deg** (three logs agree within 1.5 deg).
2. Then check the resulting cpd; it should come out ~20. If AOG keeps 18, correct it so that the
   wheel angle at full lock reads ~39 deg instead of ~42...44 deg.
3. Verify with one dedicated calibration circle (full lock, both directions, flat ground, RTK fixed)
   and re-run `steering_angle_from_circle.ps1`: it prints "countsPerDegree should be X % of the
   current value" directly. Two minutes of driving settle it to ~1 %.
### 31j. Operator question: "if cpd and max steer angle are right, do the filters then grip and suppress the heading noise under the trees?"

**Answer: NO - not by themselves.** Measured with `Temp\gate_wouldfire.ps1`, which replays the
firmware's own gate criterion (`fire when |measChange - modelChange| > max(gateFloorDeg,
gateBandPct*|M|)`, window 300 ms, `gateFloorDeg` 3.0, `gateBandPct` 0.30) on the tree log for three
wheel-angle variants - A as logged (cpd 18.12), B with the corrected cpd (wheel 9.4 % smaller),
C additionally with the remaining scale from the circle fit:

| variant | noise windows | FIRE in noise | real turns | FIRE in turns |
|---|---|---|---|---|
| A as logged (cpd 18.1) | 660 | 337 (**51.1 %**) | 94 | 17 (18.1 %) |
| B cpd corrected (x0.91) | 660 | 336 (**50.9 %**) | 94 | 11 (**11.7 %**) |
| C B + scale 0.92 | 660 | 332 (50.3 %) | 94 | 11 (11.7 %) |

* **Under the trees the cpd correction changes NOTHING** (51.1 -> 50.9 %): on a straight line the
  model predicts ~0 turn in EVERY variant, so the gate's disagreement is entirely the receiver's
  noise (dev p50 3.10 -> 3.02 deg). A correct cpd cannot help where there is no curve.
* **In real turns it matters a lot** (18.1 -> 11.7 %): today the model over-turns by ~18 %, so the
  gate fires in genuine turns and damps GOOD measurements. The correction stops that - i.e. it makes
  the gate trustworthy, which is the prerequisite for using it as a quality signal.

**Why the gate does not grip under the trees even though it fires** (firmware's own columns, tree
log): `mdev > 3` in **209 of 3269 frames (6.4 %)**, but `mgate = 1` in only **51 (1.6 %)** - the
window/hold timing (evaluate every 300 ms, re-open after 900 ms, hold 300 ms) lets the damped state
land on few frames. And when it does engage, `gateRFactor = 2.0` only doubles R: with
sigma = sqrt(p11) ~ 1.86 deg (p11 ~ 3.5) and R = 1.9 the gain is `K = 3.5/(3.5+1.9^2) = 0.49`
(log: gain p50 0.47-0.50), with R x 2 it is 0.19 - that halves a 2.5 deg noise step to 0.5 deg, it
does not remove it.

**What would grip, with the numbers:**
1. **`gateRFactor` 2 -> 4 and `gateHoldMs` 300 -> 500`** (firmware, small): in gated frames
   `K = 3.5/(3.5+7.6^2) = 0.056` instead of 0.19 - a 2.5 deg noise step contributes **0.14 deg**
   instead of 0.5 deg. On a straight line nothing of value is lost (the gated sample IS noise), and
   the false-firing in curves is gone after the cpd correction. Honest limit: the gate needs a
   300 ms window before it knows, so the FIRST 300 ms of an excursion still passes.
2. **Robust gain (P2)**: cap the per-update step and require two consistent measurements - reacts
   within 100 ms instead of 300 ms. More invasive, but model- and cpd-independent.
3. **UNIHEADINGA `hdgstddev`**: weight each epoch by its own reported accuracy - the only source
   that notices a degraded phase immediately (the adaptive R needs ~10 s).

**Recommended order:** (a) set cpd/max steer angle (no firmware risk, prerequisite), (b) then
`gateRFactor`/`gateHoldMs`, verify offline with `gate_wouldfire.ps1` + `straight_twitch.ps1`, (c)
UNIHEADINGA only if a drive still shows passes.
4. Separately: give the VWAS a proper zero (stand still on level ground until `wq` >= 80) - that is
   the larger error at small angles.
### 31k. Operator counter-proposal: "moving average with a weight from the wheel angle - and also from the SPEED" (simulated head-to-head, 2026-09-30)

The operator's physical argument (correct, and better than my gate wording): **a slow vehicle cannot
change its heading quickly, and a small steering angle cannot change it much either - so a large
deviation cannot be real, and the old values must be weighted heavily there.** Simulated on all four
logs with `Temp\cmp2.ps1` (metric set below), always filtering the SAME input (the receiver's own
attitude heading `att + offset`), judged against the position chord:

| variant | TWITCH/1000 Bäume | Brücke | Halle 2x | Start |
|---|---|---|---|---|
| base (the current chain) | 58.4 | 29.1 | 44.0 | 108.2 |
| avgFixed (plain average, tau 1 s) | 121.8 | 84.7 | 55.0 | 40.9 |
| oper (tau ramp from the wheel angle) | 125.5 | 68.8 | 38.5 | 43.9 |
| **lim2 (physical RATE LIMIT: v*tan(wheel+2deg)/L + 2 deg/s)** | **16.1** | 31.8 | **13.8** | 11.7 |
| **limW2 (lim2, and the accepted PATH turn raises the bound)** | **16.1** | 29.1 | **13.8** | **8.8** |
| limP2 (lim2 + persistence 0.4 s) | 112.3 | 78.0 | 31.6 | 52.6 |
| gate (firmware style, my earlier proposal) | 43.0 | 35.7 | 61.9 | 0.0 |

Metric: **TWITCH = frames per 1000 in which ONE 100 ms step moves the heading by > 1.5 deg while the
path is straight** - exactly the operator's complaint. The script also reports FOLLOW (median
`|d psi| / |path rate|` in real turns; 1.0 = keeps up) and SWALLOW (per 1000 turn frames with the
estimate standing still).

**Six findings, all measured:**
1. **The AVERAGING form of the proposal makes it WORSE** (twitch 122...125 vs. 58 in the tree log):
   any average lags, and the lag appears as a big step at every curve-to-straight transition. That
   refutes the "gleitender Durchschnitt" form - however the weight is chosen.
2. **The physical-bound form is a clear win**: limiting the rate at which the estimate may follow the
   measurement to `v * tan(|wheel| + 2 deg) / L + margin` cuts the twitch count by **3.6x (Bäume),
   3.2x (Halle 2x), 12x (Start)**; unchanged only in the bridge log (there the jumps come from a
   position discontinuity, not from a rate).
3. **The speed enters exactly as the operator says** - it is the same formula: v in the numerator, so
   a slow machine gets a tiny allowance. A 10...15 % cpd error is irrelevant: only the MAGNITUDE of
   the wheel angle is used, plus 2 deg of slack.
4. **Persistence must NOT be short** (`limP2` with 0.4 s is as bad as averaging, 112): a receiver
   noise burst lasts 3...5 frames, so a 0.4 s relaxation lets it through. Either none, or >= 1 s.
5. **The PATH as a second bound source helps** (SWALLOW 53 -> 32 in the tree log, Start log down to
   8.8): when the accepted fix-to-fix/path measurement shows a turn, the allowance must follow -
   otherwise turn ENTRY is delayed (the wheel is still straight while the machine already turns).
6. **My earlier gate proposal is NOT better on straight driving** (43 vs 16 in the tree log, 62 vs 14
   in Halle 2x) - it needs a 300 ms window before it reacts, and it costs turn-following
   (FOLLOW 0.26...0.71). It stays useful as a MODEL check, not as the noise filter.

**Recommendation - BUILT 2026-09-30 (operator decision "beides einbauen"):**
1. **Rate limit** in `HeadingBridge::applyMeasurement()` (the `limW2` variant): the applied
   correction `K*innov` is clipped to `allowedRate * dt`, with
   `allowedRate = v*tan(|wheel| + rateLimitWheelSlackDeg)/L + rateLimitMarginDegS`, raised by
   `rateLimitPathFactor*|path turn rate| + margin` while an ACCEPTED fix-to-fix window is turning.
   Config: `rateLimitEnabled=true`, `rateLimitMarginDegS=2.0`, `rateLimitWheelSlackDeg=2.0`,
   `rateLimitPathFactor=1.25`, `rateLimitWheelFallbackDeg=15.0` (used while the wheel quality is low,
   so a learning VWAS cannot freeze the heading). The path rate is the turn rate between two
   accepted fix-to-fix windows, decayed with 0.5 s so a stale window cannot keep the bound open.
   `status.lastGainK` now reports the EFFECTIVE fraction (raw K1 vs applied differ once it clips).
2. **COG damping strengthened**: `cogRefSpeedMps` 2 -> **3.0** (at 1.5 m/s the noise factor is now
   2.0, i.e. gain/4, instead of 1.33 = gain/1.8) - justified by the peak-source measurement in 31l
   (`cog` drove 10 of 10 slow-phase peaks in the start log).
3. **Four new HR columns 84...87** for verification: `rl_allowed` (deg/s), `rl_clip` (0/1),
   `rl_path` (deg/s), `meas_r` (deg, the R of the applied measurement). `jump_diagnosis.ps1` prints
   a P2 block (clipped share, allowed rate percentiles, path-bound frames, meas_r, and the heading
   step of clipped vs unclipped frames) and still reads older logs.
4. **Persistence against a genuinely wrong state** (`rateLimitPersistMs = 1000`,
   `rateLimitPersistRelax = 4`): the bound would otherwise correct a 20 deg error at 2 m/s only in
   5.2 s. A deviation that keeps asking in the SAME direction for 1 s is not noise but a state that
   must catch up; the bound is then relaxed 4x (20 deg in 1.3 s). 0.4 s would be too short - it lets
   the 3...5 frame receiver bursts through (measured: TWITCH 112 instead of 16). The re-seed at
   `|innov| > reseedBeyondDeg` (120 deg) bypasses the limiter entirely.

The resulting characteristic (`tools\heading_bridge\limiter_check.ps1`):

| case | allowed | per 100 ms | a 2 deg peak passes as |
|---|---|---|---|
| straight, 1.5 m/s, wheel 0 | 3.40 deg/s | 0.34 deg | 0.34 deg |
| straight, 2 m/s, wheel 0 | 3.87 deg/s | 0.39 deg | 0.39 deg |
| straight, 8 m/s, wheel 0 | 9.48 deg/s | 0.95 deg | 0.95 deg |
| straight, 8 m/s, wheel 2 deg | 16.98 deg/s | 1.70 deg | 1.70 deg |
| turn 20 deg, 2 m/s, no path | 23.63 deg/s | 2.36 deg | 2.00 deg (nothing lost) |
| turn 20 deg, 2 m/s, path 20 deg/s | 27.00 deg/s | 2.70 deg | 2.00 deg |
| wheel quality < 30 (fallback), 2 m/s | 16.35 deg/s | 1.63 deg | 1.63 deg |

So a single-frame 2...5 deg peak reaches psi as 0.34...0.95 deg (5...6x less) while a real turn
passes unclipped - and the limit falls with speed and with the wheel angle exactly as the operator
### 32. 2026-10-01 logs: "the tractor drove SIDEWAYS" after v/r/v - the chain, measured

Operator: the wheel-angle zero is quickly there (good), but the forward/reverse detection struggles. In
the last log he drove v/r/v; the tractor then drove sideways for a long time, and only when the
reception became really bad was the direction correct at once. "Sideways" also happened before,
without v+r.

**Log `2026-10-01 13-25 2x Bäume v+r autosteer` (2260 frames, 87 columns = the P2 build).**
Summary: `psi` deviates from the travel direction by p50 2.3 deg (good!), but p90 25.5 and p99 40.4 -
with 3 heading RE-SEEDS (|psi-attitude| > 120 deg). `az_out` is 88.4 deg off the travel direction,
which is BY DESIGN (receiver convention: az_out = psi - 90, verified frame by frame: psi 206.1 ->
az_out 116.1). So the "sideways" is NOT a constant offset - it is the excursions.

The chain, frame by frame (`Temp\zoom2.ps1`, `jump_diagnosis.ps1`):

1. **t = 659.1...660.4 s, v ramps 0.14 -> 1.32 m/s, wheel 0 -> -39 deg (the v/r/v manoeuvre):**
   `att+90` (the true body heading) rises 206.2 -> 214.6 (6.5 deg/s) while **psi FALLS 206.1 -> 194.3**
   - a growing 20 deg error in 1.3 s. `rl_clip = 1` in 10 of these 14 frames, `rl_allowed` ramping
   2.3 -> 30.8 deg/s, and `meas_r` inflated to 4...8 deg (the adaptive R).
   The model at that moment predicts 57.3*1.3*tan(-39deg)/2.14 = **-33 deg/s**, the truth is **+6.5
   deg/s** - a factor 5. So the prediction drags psi the wrong way; the measurement (which was RIGHT)
   is damped because the adaptive R sees the resulting large innovations and inflates - a positive
   feedback: a wrong model makes the filter trust the model even more.
2. **t = 661 s: `direction: REVERSE` is declared** ("heading-to-path 120 deg, attitude witness 159 deg,
   v 1.18") - a 120 deg mismatch is NOT a reversal, it is the symptom of the runaway. From here the
   PATH sources are mirrored by 180 deg, which pulls psi further wrong.
3. **t = 663 s: RE-SEED by +151 deg** (psi 137 -> attitude 288) - the state is snapped back, ~4 s
   after the deviation started. The first excursion (t = 559.4, -122 deg) took the same route.
4. t = 670/671: `cogX` rejects the attitude again while the machine drives forward with `rev = 1`
   still latched - the mirrored path keeps fighting the (correct) attitude.

**Weaknesses, in the order they act:**
* **W1 the kinematic model over-turns at large wheel angles and low speed.** Measured here 5x
  (33 vs 6.5 deg/s); 1.4...2.6x in normal driving (31h/31i). At low speed with a big angle the
  Ackermann relation does not hold (the machine pivots/skids). And `wq` was 50, i.e. ABOVE
  `wheelQualityLowPct`, so P1b does not stop the model. Concrete fixes: (a) below ~2 m/s do not use
  the wheel angle as a fast yaw rate at all (only as a slow bias), (b) apply the measured
  `steerScale` (0.85 fitted / 0.90...0.92 circle fit) instead of 1.0, (c) the cpd correction of 31i.
* **W2 the adaptive measurement noise (31c/§15) is a positive feedback when the MODEL is wrong**: it
  measures the innovation variance, which a wrong model inflates, so R grows and the measurement -
  the only truth in that moment - is damped. It should be driven by the disagreement with the PATH /
  travel witness, not by the raw innovation.
* **W3 the direction logic declares REVERSE on a 120 deg heading-to-path mismatch** and then mirrors
  the path sources by 180 deg, which amplifies the excursion. It must require: mismatch close to 180
  (>= 150 deg), a TRUSTED attitude, and the travel witness (positions) confirming the reversal - and
  it must never mirror path measurements while a reversal is merely suspected.
* **W4 the re-seed threshold (120 deg) fires too late**: seconds of "sideways" reach AOG first. A
  trusted attitude with a large `sigma` (the state is uncertain) should re-seed MUCH earlier, e.g.
  beyond 60...90 deg.
* **W5 my own P2 rate limiter clips during such a manoeuvre (10 of 14 frames)** and delays the
  recovery from a wrong state. It must step aside when the MODEL is the suspicious one: bypass or
  relax when the model gate is active (`mgate`), when `sigma` is large, and for the first
  measurement after a long rejection/dead-reckoning phase.

**Verification of the good parts in the same log:** `rl_allowed` rises with v and the wheel angle
exactly as designed (2.3 -> 30.8 deg/s), `rl_path` 0 on a straight line, position steps p50 0.18 m
(ratio 1.01), posFallback 1 frame only, `publim` 2.4 % - the P2/P0.1 mechanisms behaved.
CPD note: the log still shows `CPD=18` in the EEPROM and `L=2.50 -> 2.14`, so the 31i settings are
NOT corrected yet - W1(c) is still open.
argued.
Verification of the parser without a drive: `tools\heading_bridge\make_synthetic_p2.ps1` builds a
synthetic 87-column log from a real one; the tool then reported exactly the injected pattern
### 32b. Confirmed with the operator + the sustained error located (tool Temp\oblique.ps1)

Operator: the sideways happens 2x in the 13-25 log, the pattern is a K-turn (forward, full lock,
completely the other lock, back, the other lock again, forward); then a LONG leg driving obliquely
west; at the end the same K-turn and eastbound, partly with autosteer, where the tractor drives
40 cm beside the line because it is oblique. On the screen about 45 deg rotated. The K-turn is the
suspected trigger. AOG's dual-heading offset = 88 deg. When the reception got really bad the icon was
immediately straight (end of the log).

`Temp\oblique.ps1` (|psi - travel direction| per 50 s bin, straight windows only):

| t bin | n | dev p50 | dev p90 | >15 deg | >30 deg | psi-att | sigma | rev latched | sources |
|---|---|---|---|---|---|---|---|---|---|
| 550 | 261 | 0.9 | 2.8 | 0 % | 0 % | 2.2 | 3.4 | 0 % | att |
| 600 | 394 | 0.9 | 2.3 | 0 % | 0 % | 1.9 | 2.4 | 0 % | att/cog |
| 650 | 335 | 6.6 | 15.2 | **11 %** | 1 % | 7.1 | 12.1 | **76 %** | **deadreck 326** |
| 700 | 388 | **22.2** | **34.2** | **72 %** | **23 %** | 22.0 | 25.1 | **62 %** | **deadreck 308** |
| 750 | 85 | 1.5 | 2.8 | 0 % | 0 % | 2.6 | 3.3 | 0 % | att |

So the operator's "45 deg rotated / 40 cm beside the line" is a **sustained 22 deg (p50) / 34 deg
(p90) heading error over ~100 s** - and the numbers say why it held: **76/62 % of that time the
REVERSE flag was latched, and 80 % of the frames were DEADRECKON (no measurement at all)**, with
sigma 12...25 deg (the filter KNEW it was uncertain). It only came straight at the end (t >= 750 and
when the reception broke down).

**The deadlock (this is the real defect, sharper than 32's "excursion" reading):**
1. The K-turn leaves psi 22...34 deg wrong (32, W1: the model over-turns at low speed / big wheel
   angle while `wq` is above `wheelQualityLowPct`, so P1b does not stop it).
2. From that mismatch the **direction logic declares REVERSE** (threshold hit) and latches it
   (`revReconfirm` = 5000 ms).
3. **The wrong REVERSE flag then breaks the cogX cross-check**: with `reverseDirection` true the gate
   compares the attitude against `cog + 180`, so every frame with a GOOD attitude is rejected as
   `cogX`. Measured: reject mix `cogX = 584` frames (26 % of the log).
4. No measurement is accepted -> DEADRECKON -> nothing can pull the state back; and the mirrored path
   measurements pull the other way.
5. The re-seed cannot help either: it needs 120 deg, the error is only 22...34 deg. **The state is
   stuck between "too small for a re-seed" and "all measurements rejected".**

**Fixes, in the order they act (all small):**
* **F1 (the key one): the direction decision must not be able to break the measurement gates.**
  (a) Require the mismatch to be close to 180 deg (>= 150 deg) instead of the current threshold;
  (b) require the POSITION travel witness to confirm the reversal (it exists, `dirVeto` never fired -
  measured dir_veto = 0 in this log); (c) require a TRUSTED attitude; (d) **the cogX gate must not use
  the reverse hypothesis as long as the reversal is only *assumed*** - that is circular (the flag is
  derived from the same mismatch), and (e) 5 s re-confirmation is too slow when the attitude and the
  witness both disagree.
* **F2 (break the deadlock): a stuck detector.** If dead-reckoning persists for > X s while a valid
  attitude is present (qf 4/5, 20+ sats, roll plausible), accept that attitude and let the filter
  re-converge - the state is then the suspicious one, not the measurement. This is what the operator
  observed as "when the reception got really bad it was immediately straight": the false trust
  collapsed and the truth came back.
* **F3: the offsets.** AOG is 88 deg, the module sends psi - 90, so everything is 2 deg off (the
  measured mount offset is -89.5). Set BOTH to 90 (or both to 89) - a settings change, no firmware.
### 32c. F1...F4 BUILT (operator decision 2026-10-01, "alle 4"; F1 "sofort", F4 "weich")

1. **F1 - REVERSE emergency clear (sofort).** In `updateDirectionState()`, before anything else:
   if the flag is REVERSE and the POSITION witness says the machine moves along its FRONT AXIS
   (`|travelWitnessDeg - attWitnessDeg| <= reverseClearDeg`, speed >= minSpeedForDirection), the flag
   is cleared AT ONCE (counter `status.reverseClears`, warning "REVERSE cleared at once"). This test
   uses only the positions and the (never-mirrored) dual-antenna front axis, so it cannot be fooled by
   the mirrored sources - it is the test that the 13-25 log needed for 60 s.
2. **F2 - the travel witness is no longer gated by psi.** `updateTravelWitness()` keeps its last VALID
   value while the window is short (no re-basing on psi) and uses the CHANGE OF THE TRAVEL DIRECTION
   itself as the straightness test; `twRefHeadingDeg` now carries the travel direction, not psi.
   Before: `travel_wit = -1` for 60 s while the positions were unambiguous (the circular reference the
   operator warned about in the 25c discussion).
3. **F3 - the cogX gate can no longer condemn a good attitude with the reverse hypothesis.** If the
   POSITION witness agrees with the attitude (`<= cogCrossCheckDeg`), the frame is accepted even when
   the (possibly wrong, possibly mirrored) COG comparison fails.
4. **F4 - stuck-state rescue (soft, operator-approved).** If a frame is rejected ONLY by a
   plausibility gate (cogX / rollEnv / settle / rollStep) while the receiver's own quality is good and
   NO measurement has been accepted for `stuckRescueAfterMs` (3000), the frame is accepted with its R
   multiplied by `stuckRescueRFactor` (3.0) - the stale state is pulled in over ~1 s instead of
   drifting forever (counter `status.stuckRescues`, warning "frame RESCUED after %lums").
   The >120 deg case keeps its hard re-seed.

Build: SUCCESS, RAM1 free 7584 B (unchanged - the first version of the two new log texts cost 1 KB of
RAM1, they were shortened to respect the ~115 character rule), FLASH free 7 419 908 B.
Verification plan: `jump_diagnosis.ps1` (the log must show the new warnings instead of `cogX` for
60 s and `rev` must fall within ~1 s of driving forward again) + `Temp\oblique.ps1` (the
deviation-per-bin table must stop showing the 22...34 deg bins) + `straight_twitch.ps1`.

### 33. UNIHEADINGA parser (2026-10-01, operator request)

The Unicore UM982 does not put the baseline length in `$GNHPR`, so `baselineLengthM` (and with
it the fusion's baseline plausibility check) had no source on this receiver. The receiver
*does* report it - in the **UNIHEADINGA** log (Unicore N4 manual, table 7-169), which is a `#`
log, not an NMEA sentence:

```
#UNIHEADINGA,<header...>;<solstat>,<postype>,<length>,<heading>,<pitch>,<reserved>,
    <hdgstddev>,<ptchstddev>,"<stn id>",<#SVs>,...*<crc32>
```

Three details that make it different from every other sentence in `GNSSProcessor`:

1. **The header before the `;` is field 1 of the manual's numbering**, so the payload indexes
   are one off from the table: payload 0 = sol stat (field 2), 2 = length (field 4),
   3 = heading (field 5), 4 = pitch (field 6), 6 = hdgstddev (field 8), 7 = ptchstddev
   (field 9). The parser therefore uses **payload-relative** indexes and does **not** use
   `fieldRefs[]` - the header's own commas would shift every index.
2. **`stn id` is quoted** and may contain a comma, so quoted fields are skipped as a whole.
3. The checksum is the Unicore `*<8 hex>` CRC32 (already handled by `validateChecksum`).

Implementation: `GNSSProcessor::parseUniHeadingZeroCopy()` (standalone, same "arrival
instrumentation" idea as the HPR path: `uniCount` + `lastUniMillis` via `getUniCount()` /
`getUniAgeMs()`, so a silent receiver can be told from one that sends unusable values).
Detection: `detectMessageType()` strips a leading `#` (the caller passes `fieldRefs[0]`, which
is `#UNIHEADINGA` up to the first comma) and matches `UNIHEADINGA`.

**Logged, not fused.** HR columns 88...92 are `uni_len, uni_hdg, uni_pitch, uni_hdgsd,
uni_ptchsd`. Nothing in the fusion reads them yet, on purpose: the scale of `hdgstddev`
(1 sigma in degrees? a variance? an internal quality number?) has to be compared against the
observed heading spread on a drive first - the same mistake as trusting `QF` blindly would be
to feed an unverified "standard deviation" into R. `gpsData.baselineLengthM` is set from
`uni_len`, but only for `fieldIndex >= 8` (a truncated line must not pollute it).

**To see it in the field the log must be ENABLED on the receiver** (N4 ASCII command on the
port that feeds the module, e.g. `UNIHEADINGA COM1 1`; rate: 5 or 10 Hz is plenty), and the
module's own serial input must be that port. If instead every frame shows `uni_len = 0` with
`uni_hdgsd = 0`, either the log is off or the receiver answers `INSUFFICIENT_OBS` - the two
are told apart by `uniSolValid`/the all-zero pattern *plus* whether `solstat` reads
`INSUFFICIENT_OBS` or `SOL_COMPUTED`, i.e. by the raw line itself (`HeadingMonitor` prints the
last raw HPR line; the UNIHEADINGA line has no such dump yet - add `lastUniRaw` if needed).

Verification: `Temp\uni_parse_test.ps1` is a literal port of the algorithm (same index
arithmetic) and was run against the operator's sample
(`INSUFFICIENT_OBS,...,"",...` -> rejected as unusable, all zeros), a good `NARROW_INT` line
(2.1486 / 214.5513 / -1.2345 / 0.1842 / 0.2013 - field by field as in the table), a line with a
comma inside the quoted `stn id` (no index shift) and a truncated line (rejected).

**Raw sentence dump + frequency policy (operator question "wie oft?").** The newest raw
UNIHEADINGA line is kept in `GNSSProcessor::lastUniRaw[160]` (160, not the 96 of the shorter
`$GNHPR`: the sentence is ~144 characters up to the CRC32, so 96 would cut off exactly the two
stddev fields) and exposed as `getLastUniRaw()`. `HeadingMonitor::logUniState(tag, withRaw)`
emits it, and the cadence is:

| Trigger | Line | Rate on a normal drive |
| --- | --- | --- |
| every HPR event EDGE (loss and recovery) | `UNI[EVENT]` + `UNI RAW:` | ~2 lines per heading loss |
| usable <-> stale/unusable change, incl. the first line ever | `UNI[CHANGE]` + `UNI RAW:` | rare (a few per drive) |
| otherwise (heartbeat) | `UNI[HEARTBEAT]`, NO raw | 2 lines/min = 120 lines/h ~ 9 kB/h |

Reasoning: the raw sentence is a ~144 character line, so printing it at a fix rate would both
break the <=120 character rule the two-line split in `logDebugLine` exists for and flood the
INFO token bucket that `WheelAngleFusion`/`HeadingMonitor` already oversubscribe. The moments
that matter are the edges - that is where "did the receiver go silent, or did it keep talking
with INSUFFICIENT_OBS?" has to be answered - and the 30 s heartbeat exists only so a healthy
drive still leaves a record. `UNI_STALE_MS = 2000` (the receiver is set to 5...10 Hz, so 2 s =
10-20 missing sentences). The raw line goes to `Serial` directly (`Serial.printf`), not through
`LOG_NOTICE`: a raw diagnostic must not be droppable by the bucket. Cost measured 2026-10-01:
160 B static + ~200 B of code (kept out of ITCM with `FLASHMEM`); the reported RAM1 free value
moves in ~1 kB steps (`padding` swings with it), so it is not a proportional cost - the same
build with and without the LOG_NOTICE call site printed the identical number.
* **F4: W1 (the trigger)** - do not use the wheel angle as a fast yaw rate below ~2 m/s, and correct
  cpd / max steer angle (31i: cpd ~20, max ~38...39 deg; the log still shows CPD=18).
(20.0 % clipped, allowed p50 4.19/p90 14.33 deg/s, meas_r p50 1.90/p90 3.40).
### 31l. Operator follow-up: "you add 2 deg - so does a 2 deg peak pass immediately? And: did the peaks have a rise and fall, or are they singular?"

**(1) Wording correction first: the 2 is 2 deg/s, NOT 2 deg per step.** The bound is a RATE, and the
applied step is clipped to `allowedRate * dt`. Numerically on a straight line (wheel ~0, 100 ms step):
`allowedRate = (v*tan(0..2 deg)/L) + 2 deg/s` -> at 2 m/s and a straight wheel: **0.2 deg may pass
per 100 ms**; a reported 2 deg peak in ONE step is 20 deg/s, i.e. the excess is 18 deg/s and the step
is clipped from 2.0 deg to 0.2...0.7 deg. So a peak passes at most ~1/4 to 1/10 - which is exactly
why the twitch count fell 3.6x in the simulation.

**(2) Shape of the peaks - measured, and the answer is SINGULAR.** `Temp\spike_shape.ps1` (shape of
each peak: the step size, the two frames before/after, the level shift from before to after, and the
run length) plus `Temp\az_peaks_by_src.ps1` (source split). Peaks = |step| >= 1.5 deg per 100 ms while
the PATH is straight (|path turn rate| < 3 deg/s), i.e. exactly the jitter:

| log | straight frames | peaks | per 1000 | abs(step) p50 | v p50 | run length 1 | source split |
|---|---|---|---|---|---|---|---|
| 6-23 Start | 342 | 10 | 29.2 | 5.40 deg | 1.2 m/s | **10 of 10** | **cog = 10** |
| 6-36 Bäume | 1371 | 22 | 16.0 | 3.80 deg | 1.8 m/s | **21 of 22** | cog = 12, att = 10 |
| 6-41 Brücke | 756 | 3 | 4.0 | 3.80 deg | 1.9 m/s | 3 of 3 | att = 2, cog = 1 |
| 6-44 Halle 2x | 727 | 35 | 48.1 | 3.50 deg | 4.6 m/s | **28 of 35** | **att = 30**, cog = 5 |

* **They are single-frame events** (run length 1 in 21/22, 10/10, 28/35 cases): the value jumps in
  ONE 100 ms sample and is back on the next (sometimes as an up-down pair over 2...3 frames). **No
  rise-and-fall ramp, no slow build-up** - so a rate limiter is the right instrument: it clips the
  first frame and the event is over. A moving average would spread it over a second instead.
* **The dominant source in the slow phases is OUR OWN COG fall-back**, not the UM982: `cog` drives 10
  of 10 peaks in the Start log (v p50 **1.2 m/s**) and 12 of 22 under the trees (1.8 m/s), with peak
  sizes 3.8...5.4 deg. The 4.3 damping is far too weak there (at 1.2 m/s the factor is only 2.0/1.2 =
  1.67, i.e. the gain is reduced by 1/1.67^2 = 0.36). This is the operator's "1...2 deg Zittern" in
  the yard/after the stop - our own doing.
* **Only in `Halle 2x` is the raw ATTITUDE the dominant source** (att = 30 of 35) - and there at
  v p50 4.6 m/s, where the UM982's own heading noise is what arrives. That is the case the rate
  limiter handles as well.
* The bridge log is the cleanest (4 peaks/1000) - consistent with 31g (its jitter comes from position
  discontinuities, not from rates).

**Consequence for the design (both are small and independent):**
1. **Rate limiter `limW2`** (31k): cuts the single-frame peaks of BOTH sources - the receiver's and
   the COG's - because it works on the measured heading rate, not on the source name.
2. **Strengthen the COG damping** (4.3 follow-up): with the peak source now known to be `cog` at
   1...2 m/s, `cogRefSpeedMps` 2 -> 3 (or the factor squared) cuts those 3.8...5.4 deg single-frame
   peaks further at the source. Cheap, no new logic.