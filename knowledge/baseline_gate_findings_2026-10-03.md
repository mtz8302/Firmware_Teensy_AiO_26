# Baseline gate findings - 2026-10-03

Status: analysis complete, **no firmware touched**. All numbers come from the raw HR logs
by replaying `HeadingBridge.cpp:580-603` offline.

Tools: `tools/analysis/lenqc.ps1` (yardstick scoring), `tools/analysis/basfix.ps1`
(candidate simulator), `tools/analysis/_devchk.ps1` (reference reconstruction).

## 0. The question

Two candidates were on the table for judging the quality of a dual-antenna attitude
solution:

* the **solved baseline length** compared against the mechanically known antenna spacing
  (~0.72 m), and
* the **receiver's own `hdgstddev`** field from `$GNHPR` / `$UNIHEADINGA`.

## 1. Finding: the length MEASUREMENT is good, the learned REFERENCE was the defect

The 2026-10-03 wobble was blamed on the baseline-length gate. That is correct, but the
causal chain is one step longer than "the length is a bad yardstick":

| log | frames with a length | p50 solved length | error vs 0.72 m |
|---|---|---|---|
| 2026-10-03 12-52 | 565 | **0.719 m** | -0.1 % |
| 2026-10-03 12-55 | 1096 | **0.711 m** | -1.3 % |
| 2026-10-02 16-23 | 1446 | 0.718 m | -0.3 % |
| 2026-10-01 20-03 | 1288 | 0.712 m | -1.1 % |
| 2026-10-01 20-21 | 912 | 0.709 m | -1.5 % |

The receiver solves the length to within ~1 % of the physical 0.72 m **on every session**.
The measurement was never the problem.

The problem is the reference it is compared against, and that reference is recovered
exactly from the log because the firmware logs the deviation itself
(`devPct = 100*(len-ref)/ref  ->  ref = len/(1+devPct/100)`):

```
2026-10-03 12-52, frames 0..161 (the healthy stretch, before the trees)
   k     t_s    baseline_m   dev_pct   implied ref
   0    86.6      0.715      -16.1       0.852
  16    88.2      0.717      -15.9       0.853
  48    91.4      0.714      -16.2       0.852
  96    96.2      0.715      -16.0       0.851
 128    99.4      0.704      -17.4       0.852
 160   102.6      0.714      -16.2       0.852
```

* the reference is **0.852 m = +18.3 %** above the true 0.72 m,
* it is already there on the **first logged frame** and does not move for 162 frames,
* `|devPct| > 5 %` for **99.5 %** of all frames,

so the gate rejects frames whose length is spot-on - it is rejecting the *reference*, not
the measurement.

### Why it is necessarily a LEARNED reference

`Config::physicalBaselineM` exists (`HeadingBridge.h:543`, default 0) but has **no setter,
no web-UI field and no EEPROM slot**, so it is always 0, and the only writer of
`baselineRefM` is the running mean at `HeadingBridge.cpp:583/588`. There is no re-learn
path and no physical bound anywhere. The 0.852 m was therefore averaged in during the
`baselineLearnFrames = 100` window, which the firmware reached **before** the HR log
began.

### How wrong can the reference be? (all sessions)

`_devchk` / `lenqc` recover the reference per session and count how many
receiver-CLEAN frames (`fixQuality = 4`, `0 < hdgstddev <= 1` - i.e. frames the receiver
itself calls good) the gate would throw away:

| log | reference | off true | clean frames | gate-thrown-away |
|---|---|---|---|---|
| 2026-10-03 12-52 | 0.852 | **+18.3 %** | 335 | **99.7 %** |
| 2026-10-03 12-55 | 0.852 | **+18.3 %** | 868 | **99.8 %** |
| 2026-10-02 16-23 | 0.713 | -1.0 % | 760 | 12.9 % |
| 2026-10-02 16-27 | 0.713 | -1.0 % | 913 | 5.8 % |
| 2026-10-01 20-03 | 0.731 | +1.5 % | 897 | 26.0 % |
| 2026-10-01 20-06 | 0.731 | +1.5 % | 1425 | 15.9 % |
| 2026-10-01 20-15 | 0.746 | +3.6 % | 861 | ~30 % |
| 2026-10-01 20-18 | 0.746 | +3.6 % | 597 | 61.1 % |
| 2026-10-01 20-21 | 0.705 | -2.0 % | 832 | 0.5 % |
| 2026-10-01 20-24 | 0.705 | -2.0 % | 471 | 1.1 % |

**The poisoning is not a 2026-10-03 accident.** In 8 of 10 sessions the learned reference
sat 1-4 % off, and that alone cost 6-61 % of the frames the receiver called clean. Only
two sessions (20-21, 20-24) had a reference good enough to behave.

Also note: the reference is **not monotonic across sessions** (0.705 ... 0.852), which is
itself proof that it is re-learned per boot and never persisted - a reboot resets it.

## 2. Which yardstick is better? Both, because they measure different failures

`lenqc.ps1` scores each candidate signal against two *independent* truth proxies by
Mann-Whitney AUC (1.0 = perfect ranking, 0.5 = coin flip):

* **truth #1** `|dAtt|/dt > 5 deg/s` - "the heading is noisy/steppy". This is what a
  wobble looks like, and `hdgstddev` exists exactly to describe it. It is **contaminated
  by real fast turns**, so it is a *noise* proxy.
* **truth #2** the signed `attitude - COG` difference jumps more than 5 deg away from its
  own median while driving faster than 1.5 m/s - "the heading is **wrong**". It is
  completely independent of the dual-antenna solution, and a constant att-vs-COG offset
  (mount, sidehill, crab) is subtracted out first, so it is a *bias* proxy.

### vs truth #1, "is it noisy"

| log | length err vs 0.72 | receiver hdgstddev | gate abs(devPct) |
|---|---|---|---|
| 2026-10-03 12-52 | 0.742 | **0.805** | 0.521 |
| 2026-10-03 12-55 | 0.519 | **0.647** | 0.461 |
| 2026-10-02 16-23 | 0.542 | **0.628** | 0.604 |
| 2026-10-02 16-27 | 0.527 | **0.686** | 0.561 |
| 2026-10-01 20-03 | 0.482 | **0.656** | 0.503 |
| 2026-10-01 20-06 | 0.494 | **0.772** | 0.473 |
| 2026-10-01 20-11 | 0.514 | **0.654** | 0.464 |
| 2026-10-01 20-15 | 0.633 | **0.744** | 0.637 |
| 2026-10-01 20-18 | 0.623 | **0.778** | 0.520 |
| 2026-10-01 20-21 | 0.395 | 0.492 | 0.491 |
| 2026-10-01 20-24 | 0.515 | **0.702** | 0.207 |
| **mean** | **0.53** | **0.69** | 0.49 |

`hdgstddev` wins 10 of 11 and beats the length error decisively.

### vs truth #2, "is it WRONG"

| log | length err vs 0.72 | receiver hdgstddev | gate abs(devPct) |
|---|---|---|---|
| 2026-10-03 12-52 | 0.787 | 0.554 | 0.769 |
| 2026-10-03 12-55 | 0.501 | 0.441 | 0.462 |
| 2026-10-02 16-23 | **0.825** | 0.661 | 0.830 |
| 2026-10-02 16-27 | **0.684** | 0.656 | 0.724 |
| 2026-10-01 20-03 | **0.754** | 0.512 | 0.522 |
| 2026-10-01 20-06 | **0.646** | 0.515 | 0.352 |
| 2026-10-01 20-11 | **0.673** | 0.536 | 0.708 |
| 2026-10-01 20-15 | **0.836** | 0.395 | 0.795 |
| 2026-10-01 20-18 | **0.896** | 0.415 | 0.816 |
| 2026-10-01 20-21 | **0.452** | 0.374 | 0.437 |
| 2026-10-01 20-24 | **0.616** | 0.410 | 0.623 |
| **mean** | **0.70** | 0.49 | 0.64 |

The ranking **inverts**: against "is the heading wrong", the length error wins 10 of 11 and
`hdgstddev` is at chance level.

### Why that is not a contradiction

* `hdgstddev` is the receiver's **precision** estimate. A solution can be perfectly
  *precise* and still *wrong by 15 deg* - a consistent bias produces a small standard
  deviation, so `hdgstddev` cannot see it. On truth #2 it scores 0.49, i.e. it is blind
  to exactly the error that makes the tractor drive crooked.
* the baseline length is a **geometric consistency check** against a known physical
  constant. A wrong length and a wrong heading come out of the same broken carrier-phase
  solution, so the length tracks bias as well as noise.
* **only the length has an absolute anchor.** `hdgstddev` has no way of knowing that
  0.72 m is the correct spacing; the length does, because the operator measured it.

### The decisive case for keeping the length

Where the receiver is *not* RTK-fixed, the solved length collapses into nonsense while
`hdgstddev` stays unremarkable - the length is the only one of the two that notices:

| log | fixQ 4 p50 length | fixQ 1-3 p50 length |
|---|---|---|
| 2026-10-02 16-23 | 0.717 m | **1.558 m (+116 %)** |
| 2026-10-01 20-03 | 0.709 m | **3.237 m (+350 %)** |
| 2026-10-01 20-06 | 0.706 m | **1.476 m (+105 %)** |
| 2026-10-01 20-24 | 0.710 m | **4.084 m (+467 %)** |

And the price of an absolute length gate is small: among the frames the receiver calls
clean, only **1.2 - 5.3 %** sit outside +/-5 % of 0.72 m. The gate is not what destroys
good frames - the *reference* is.


## 3. Candidate simulation (`basfix.ps1`)

`basfix.ps1` replays the `HeadingBridge.cpp:580-603` learn/judge block over a log and
scores each candidate. Two details make it a faithful reproduction:

* the learning window is fed by `fixQuality >= 4` frames only (the learn gate at
  `HeadingBridge.cpp:1051`), and `baselineLengthM > 0.01` is the entry guard of the block,
* the **carried-in reference** is recovered from the log's own `baseline_dev_pct` column
  (median over the whole log of `len/(1+devPct/100)`), because on 2026-10-03 the poisoning
  happened *before* the HR log started. Without this the simulator would re-learn a fresh,
  healthy reference and would show a harmless 2.7 % loss instead of the real failure.

**Validation:** `cont5` (today's logic, carried-in reference) predicts a loss of
**99.7 % / 99.8 %** on 12-52 / 12-55; measured from the logs it is **99.7 % / 99.8 %**.

`loss%` = share of receiver-CLEAN frames (`fixQ = 4`, `0 < hdgstddev <= 1`) thrown away by
the gate. That is the number that must be small. `keepP50/90` = |len-0.72|/0.72 over the
frames that survive.

| candidate | what it is |
|---|---|
| `base5` | today, but starting a fresh boot inside the log (control) |
| `cont5` | **today, carrying the reference learned before the log** |
| `cont15` / `cont20` | candidate **D**: same, `baselineTolPct` raised to 15 / 20 % |
| `med100` | robust learn: **median** of the first 100 instead of a running mean |
| `contrl5/10` | median learn + re-learn after 600 **consecutive** rejects |
| `contrc5/10` | median learn + re-learn when 90 % of the last 300 judged frames were rejected |
| `abs5` / `abs15` | candidate **C1**: reference **bound to `physicalBaselineM` = 0.72 m**, no learning |

### 2026-10-03 12-52 and 12-55 (the failure case, reference 0.852 m = +18.3 %)

| candidate | 12-52 rej% | 12-52 loss% | 12-55 rej% | 12-55 loss% | keepP90 |
|---|---|---|---|---|---|
| cont5 (today) | 98.8 | **99.7** | 99.3 | **99.8** | 17.4 / 19.3 % |
| cont15 (D) | 82.8 | 84.8 | 86.7 | 89.6 | 29.6 / 10.1 % |
| cont20 (D) | 22.7 | 1.5 | 5.7 | 2.4 | 5.0 / 4.2 % |
| contrl5 | 98.8 | 99.7 | 99.3 | 98.2 | 17.4 / 19.3 % |
| contrc5 | 99.4 | 88.4 | 99.1 | 87.2 | 15.0 / 19.3 % |
| abs5 (C1) | 30.6 | **2.7** | 10.1 | **4.5** | 2.4 / 3.8 % |
| abs15 (C1) | 26.5 | 0.3 | 0.5 | 0.0 | 3.3 / 5.0 % |

### and on the sessions that looked "healthy"

| log | today (cont5) loss% | cont20 loss% | contrc5 loss% | abs5 loss% | abs15 loss% |
|---|---|---|---|---|---|
| 2026-10-02 16-23 | 5.1 | 0.1 | 5.1 | 5.3 | 0.3 |
| 2026-10-02 16-27 | 2.1 | 0.0 | 2.1 | 2.7 | 0.0 |
| 2026-10-01 20-03 | 13.0 | 0.0 | 13.0 | 4.6 | 0.0 |
| 2026-10-01 20-06 | 8.6 | 0.0 | 8.6 | 1.8 | 0.0 |
| 2026-10-01 20-15 | 30.3 | 0.2 | 30.3 | 3.4 | 0.3 |
| 2026-10-01 20-18 | 55.9 | 0.0 | 55.9 | 2.5 | 0.0 |
| 2026-10-01 20-21 | 0.5 | 0.0 | 0.5 | 1.2 | 0.0 |
| 2026-10-01 20-24 | 0.8 | 0.0 | 0.8 | 3.2 | 0.0 |
| **worst case** | **99.8** | 2.4 | 88.4 | **5.3** | **0.3** |

### What the simulation rules out

* **D (raise `baselineTolPct`) is a band-aid, not a fix.** `cont15` still loses 84.8 % on
  12-52 - the error is 18.3 %, so nothing below ~20 % helps. `cont20` does work on these
  logs, but its margin is exactly the coincidence that the poisoning happened to be
  +18.3 %; a +25 % reference would defeat it again. It also loosens the gate on every
  healthy session (keepP90 4-8 % instead of 2-4 %), i.e. it pays for the bug with
  permanently worse data.
* **Re-learn does not heal, it makes it worse.** `contrl5` never fires at all: 600
  *consecutive* rejects never happen because a stray accepted frame resets the counter.
  `contrc5` fires - **and then re-learns from the very bad stretch that caused the reject
  rate**, whose median length is ~0.95 m, so it adopts a *second* wrong reference and the
  loss stays at 88 %. Self-healing re-learn is unsafe by construction here.
* **`med100` is not robustness, it is a reboot.** On 03.10 it scores like `base5` (2.7 %
  loss) *because the log is already healthy* - the median simply re-derives the correct
  0.72 m. It does nothing for the observed failure, which is a reference that was already
  frozen before the log began.
* **`abs5` / `abs15` are the only candidates that are correct on every log**, because they
  remove the free parameter that can be wrong. They are also the best *quality* result:
  keepP50 0.8 - 1.9 %, keepP90 2.4 - 3.9 %, the tightest of all candidates.

## 4. Recommendation

1. **Bind the reference to the physical spacing - candidate C1.** `Config::physicalBaselineM`
   already exists (`HeadingBridge.h:543`) and the read is already wired
   (`HeadingBridge.cpp:582/597`); what is missing is the **setter, the web-UI field and the
   EEPROM slot**. With `physicalBaselineM = 0.72` and `baselineTolPct = 5` the simulated
   worst-case loss over 10 logs is **5.3 %** and every kept frame is within 2.4 % (p50) /
   3.9 % (p90) of the true length. Learning can stay as a fallback for
   `physicalBaselineM = 0`, but the operator should never have to rely on it.
2. **Keep `hdgstddev` as the adaptive quality signal, not as a hard reject.** It is the
   best ranker of heading *jitter* (mean AUC 0.69 vs 0.53) and it is already read by
   `attitudeQualityNow()`; use it to scale R / degrade the frame. As a hard gate it is
   unusable - even at its best cut it has 13-18 % false alarms, and it is blind to the
   systematic bias (AUC 0.49) that the length catches.
3. **Do not add a re-learn path.** Both re-learn variants are worse than the bug.
4. **Do not change the parser** (`GNSSProcessor.cpp:1799-1858`,
   `baselineLengthM = uniLengthM` regardless of `postype`) before the reference is fixed -
   candidate A (a `postype` gate) was not simulatable because `postype` is discarded and
   never logged. On the evidence above it is also not the binding constraint: 03.10 12-52
   is 100 % `fixQ = 4` and still fails.

### Open items

* `physicalBaselineM` needs a setter + web field + EEPROM entry before C1 can ship.
* `postype` is dropped in the parser and absent from the logs; adding it to the HR header
  would make candidate A measurable.
* `hdgstddev` is only present from the 2026-10-01 20:03 build onward (92-column logs); the
  87-column logs have neither `baseline_m` nor `uni_hdgsd`.
* the simulator only models the length gate. It does not model the interaction with
  `attitudeQualityNow()`, the R scaling or the COG cross-check.


### Notes for whoever runs these tools next

```
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\analysis\lenqc.ps1  -Filter '2026-10-03*'
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\analysis\basfix.ps1 -Filter '2026-10-03*'
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\analysis\_devchk.ps1
```

* PowerShell 5.1 rounds on a cast: `[int](1.5)` is **2**, not 1. A binary search written
  as `[int](($lo+$hi)/2)` therefore produced `$m == $hi` and looped forever, which is what
  made the first version of `lenqc.ps1` hang with no output. Use `[int][math]::Floor(...)`.
* `Write-Host` output is buffered when the call is piped to `Out-String`, so a hang shows
  **nothing** rather than partial progress. A trace file written with `Add-Content` survives
  the kill.
* argument lists matched with `Get-ChildItem -Filter` are 30 s budget: keep a batch to
  <= 3 logs. `Read-HrLog` borrows a header by column count, so 87/92/94/102-column logs all
  map correctly, but the 87-column builds simply do not contain `baseline_m` or `uni_hdgsd`.


## 5. The measured antenna spacing, and the EEPROM field (implemented 2026-10-03)

### 5.1 The measurement (`tools/analysis/baselen.ps1`)

Only `fixQuality = 4` frames are used - those are the RTK-fixed epochs where the receiver
actually solves the dual-antenna baseline. Frames with `fixQ 1-3` carry a meaningless
length (p50 1.5 - 4.1 m), so they must not enter the estimate.

15 logs, **13 065 RTK-fixed frames**, `2026-10-01 20-03` ... `2026-10-03 12-55`:

| log | n | baseline_m p05 | p50 | p95 |
|---|---|---|---|---|
| 2026-10-01 20-03 | 1001 | 0.682 | 0.709 | 0.726 |
| 2026-10-01 20-06 | 1542 | 0.688 | 0.706 | 0.728 |
| 2026-10-01 20-11 | 757 | 0.701 | 0.710 | 0.722 |
| 2026-10-01 20-15 | 966 | 0.687 | 0.716 | 0.744 |
| 2026-10-01 20-18 | 925 | 0.687 | 0.711 | 2.675 |
| 2026-10-01 20-21 | 859 | 0.692 | 0.708 | 0.724 |
| 2026-10-01 20-24 | 482 | 0.687 | 0.710 | 0.725 |
| 2026-10-02 11-27 | 735 | 0.683 | 0.713 | 2.576 |
| 2026-10-02 16-23 | 1221 | 0.684 | 0.717 | 1.375 |
| 2026-10-02 16-27 | 1160 | 0.691 | 0.711 | 0.993 |
| 2026-10-02 16-30 | 1050 | 0.677 | 0.715 | 0.857 |
| 2026-10-02 17-38 | 425 | 0.689 | 0.710 | 0.733 |
| 2026-10-02 17-40 | 281 | 0.692 | 0.712 | 0.741 |
| 2026-10-03 12-52 | 565 | 0.668 | 0.719 | 1.347 |
| 2026-10-03 12-55 | 1096 | 0.678 | 0.711 | 0.746 |

```
median of medians:  0.7110 m      mean of medians: 0.7119 m
per-session range:  0.706 ... 0.719 m
pooled median:      0.7110 m      pooled mean: 0.7530 m   <- dragged up by the multipath tail
receiver's own field uni_len, pooled median: 0.7110 m     <- identical, baseline_m is a faithful copy
```

The histogram has a single clean peak at **0.710 m** (2302 frames in that 5 mm bin), with an
asymmetric tail to the high side - multipath inflates the solved length, it never shrinks
it. So the median and the mode agree.

**The value to enter is 0.711 m.** The 0.719 m quoted earlier was the single highest session
(2026-10-03 12-52) and must not be used as the machine constant - it is 1.1 % high.

### 5.2 0.711 m is also the best anchor

Share of the 13 065 pooled RTK-fixed frames that fall inside the resulting +-5 % window:

| anchor | accepted | rejected low | rejected high |
|---|---|---|---|
| 0.700 m | 86.4 % | 1.4 % | 12.1 % |
| 0.705 m | 87.6 % | 1.7 % | 10.7 % |
| **0.711 m** | **87.8 %** | 2.4 % | 9.8 % |
| 0.715 m | 87.4 % | 3.2 % | 9.4 % |
| 0.720 m | 86.8 % | 4.3 % | 9.0 % |

The curve is flat, so a 1 cm error in the entered value costs well under 0.5 %. The ~10 %
that stay rejected at the high side are the multipath frames - physically correct rejects.
Because tolerance is a *percentage*, 0.71 m is stored, not 0.711 m.


### 5.3 What was implemented (build verified: `pio run -e teensy41` SUCCESS)

`Config::physicalBaselineM` existed but had **no writer** - not from the web, not from the
EEPROM, nowhere. It was therefore always 0 and the bridge always learned. Four files:

| file | change |
|---|---|
| `lib/aio_config/EEPROMLayout.h` | comment only - block 1600-1699 now holds both values |
| `lib/aio_config/ConfigManager.h` | members `bridgePhysicalBaselineM` / `...Valid` + getter/setter/isValid |
| `lib/aio_config/ConfigManager.cpp` | defaults in the constructor block; `save-`/`loadHeadingBridgeConfig()` extended |
| `lib/aio_navigation/HeadingBridge.h` | `setPhysicalBaselineM(float)`, `getPhysicalBaselineM()`, `isPhysicalBaselineSet()`; `Config` comment with the measurement |
| `lib/aio_navigation/HeadingBridge.cpp` | EEPROM read in `init()`, `setPhysicalBaselineM()` implementation, `base=%.3fm` in the boot log |

EEPROM layout inside 1600-1699 (was: 5 of 100 bytes used):

```
byte 0       scale marker        0xA5 = learned scale valid
bytes 1..4   float steerScale
bytes 5..6   baseline marker PAIR 0xA5 0x5A = antenna spacing valid   <- NEW
bytes 7..10  float physicalBaselineM                                 <- NEW
```

The **pair** of marker bytes is deliberate: the region from byte 5 on was never used before
2026-10-03, so on an existing board it can hold leftovers of an older layout. A single marker
byte would accept such stale data with a 1-in-256 chance; the pair makes that negligible.

Both values carry their **own** marker, so a blank or foreign EEPROM cannot inject a bogus
geometry into the plausibility test - the same reasoning that already protected the scale.
Load validates the window `0.1 ... 5.0 m`; a blank EEPROM reads 0xFF in both fields and
fails the marker test, so the result is 0.0 = "not entered" = learning, exactly as before.

**Behaviour is unchanged until a value is entered.** With `physicalBaselineM = 0` the code
path is bit-identical to before, so this cannot regress an existing machine.

With a value entered, `HeadingBridge.cpp:582` takes the `physicalBaselineM > 0.1f` branch:
the reference is that constant and the running mean at `:588` is never executed, and
`judged` at `:597` is true from the first frame, so there is no 100-frame blind window
either. This is the candidate `abs5`/`abs15` of the simulation in section 3 - the only one
that was correct on every log.

### 5.4 Runtime path and what is still missing

```
headingBridge.setPhysicalBaselineM(0.71f);   // sets, validates, persists, logs
headingBridge.getPhysicalBaselineM();        // 0.0 when not set
headingBridge.isPhysicalBaselineSet();       // for a UI warning
```

The setter is the single entry point the web UI will call; it writes the EEPROM itself, so
the page does not have to know about `ConfigManager`.

Still to do for the web field (was planned as item 7 in `heading_bridge_concept.md:308`):

* `SimpleWebManager::handleVWASConfig()` - parse the value, call `setPhysicalBaselineM()`,
  answer with the stored value. `saveVWASConfig()` is NOT the place: this value belongs to
  the bridge, not to the VWAS.
* `SimpleWebManager.cpp:2231` - add `bridgePhysicalBaselineM` and
  `bridgePhysicalBaselineSet` next to the existing `bridgeSteerScale*` keys of
  `/api/vwas/status`, so the page can show "entered 0.71 m" vs "learned 0.852 m".
* `web_pages/VWASConfigPage.h` - one input field, plus the warning when the value is 0 and
  `status.baselineLengthM` and `status.baselineRefM` disagree by more than the tolerance,
  which is the visible symptom of a wrongly learned reference.
* `/api/vwas/status` should also expose `status.baselineRefM` - it is the only field that
  reveals a poisoned reference, and it is already in `status`.

### 5.5 Suggested additional log fields

The HR line is already 102 columns and each column costs flash and CPU per frame, so these
are ranked - the first one is the important one:

1. **`ref_m` - the reference the gate actually used** (`status.baselineRefM`). Today it has
   to be reconstructed as `len/(1+devPct/100)`, which is only valid once the learn window is
   over and silently wrong inside it (`_devchk.ps1` shows 3.227 m where the truth was
   0.713 m). One column turns every future diagnosis of this gate into a single read.
2. `postype` - the u-blox carrier solution type, dropped by the parser
   (`GNSSProcessor.cpp:1799-1858`). Without it the `postype` gate (candidate A) cannot be
   measured at all, and `baselineLengthM = uniLengthM` is applied to FLOAT/NARROW_FLOAT
   frames as if they were NARROW_INT.
3. `base_src` - one character: `E` = entered/EEPROM, `L` = learned, `-` = not set. Makes it
   obvious from the log whether the fix is actually active.
4. the **learn counter** (`baselineLearnCount`) during the first 100 frames only, or simply
   the frozen/frozen-time - today the only way to see that learning is still running is
   `judged == false`, which is not logged.
5. `rej` already distinguishes the reason; nothing to add there.


### 5.6 The baseline deviation: THREE users, ONE tolerance

The same number `status.baselineDevPct = 100*(baselineLengthM - baselineRefM)/baselineRefM`
drives three different decisions:

| | learn gate | reject gate | output quality gate |
|---|---|---|---|
| code | `HeadingBridge.cpp:1168` - which frames may feed the learned reference | `evaluateAttitudeFrame()`, `RejectReason::BASELINE_LENGTH` | `attitudeQualityNow()` |
| tolerance | `Config::baselineTolPct` = 5.0 % | `Config::baselineTolPct` = 5.0 % | `Config::baselineTolPct` = 5.0 % (had its own field `uniBaseTolPct` until 2026-10-03) |
| effect | the frame is left out of the running mean | the attitude measurement is **DISCARDED**; the bridge falls back to COG / dead reckoning - this is what the operator sees as the wobble | the epoch is only marked **DEGRADED**: `att_q = 0`, the output may not SNAP and is rate-limited instead. The attitude is still used |

The two duplicate tolerances were merged on the operator's decision (2026-10-03). Behaviour is
unchanged (both were 5.0) and one number now has one meaning: beyond the tolerance the
measurement is not trustworthy, so it may neither be discarded *nor* snapped, and it may not
train the reference either.

Diagram of the shared state:

```
baselineRefM  <-- learned (running mean) OR Config::physicalBaselineM OR the build default
      |
      +--> baselineDevPct --+--> |dev| > baselineTolPct -> does NOT feed the learned reference
                            +--> |dev| > baselineTolPct -> DISCARD the measurement
                            +--> |dev| > baselineTolPct -> suppress the snap, rate limit
```

So a wrong reference hits **all three** paths, and one correct EEPROM value fixes **all three**.
Note the learn gate cannot protect itself against a bad learning window: while the reference is
still the running mean, `baselineDevPct` is near 0 for every incoming frame, so that test always
passes and each bad frame is simply averaged in. That is precisely what happened on 2026-10-03.

`baselineTolPct` itself is a **percentage of the reference**, so at 0.711 m it is +-3.6 cm. The
same field is also printed in the boot log as `gradev=%.0f%%` (a historical name - it is the
baseline deviation tolerance, not a "grade").

### 5.7 Second round: built-in default + safety net (build verified, +296 B flash)

Answer to "should 0.71 m be compiled in": **yes** - the field was useless until someone entered
it, and the web page comes later.

* `Config::physicalBaselineDefaultM = 0.711f` (`static constexpr`). `HeadingBridge::init()`
  now applies, in this order:
  1. a value in the EEPROM (operator override) - `0.0` there means "switched OFF, learn again";
  2. otherwise the build default 0.711 m, logged explicitly.
  So the fix is **active without anyone configuring anything**, and the EEPROM can still
  override or disable it.
* EEPROM load gained a third state so "switched off" is distinguishable from "nothing stored".

**The safety net, which is what makes a compiled-in constant acceptable.** A wrong reference -
learned *or* entered, e.g. this board moved to another vehicle with a different antenna spacing -
must never be able to silence the heading. Over each window of `baselineGuardWindow` (300, ~30 s)
judged frames the test counts its own rejects; above `baselineGuardRejectPct` (90 %) it declares
the **reference** wrong, disables itself and logs a warning naming the reference value and the
0.711 m to enter.

This is deliberately **not** the re-learn of section 3: the wrong reference is never adopted. The
test switches off and the behaviour returns to what it was before the test existed. The
simulation showed why the difference matters - a re-learn picks up the bad stretch that caused
the reject rate and stays wrong (88 % loss), while switching off is always safe.

Both gates respect it (`!status.baselineTestDisabled`), because the output quality gate would
otherwise keep rate-limiting the output for the same wrong reason. `setPhysicalBaselineM()`
clears the net, so entering a value re-arms the test.

With this, every session ends up in one of three states, and the boot log says which:

| boot log | meaning |
|---|---|
| `physicalBaselineM from EEPROM: 0.711 m ... FIXED reference` | operator value, no learning |
| `physicalBaselineM: not in the EEPROM, using the built-in default 0.711 m` | factory state |
| `physicalBaselineM from EEPROM: 0 (switched OFF)` | gate deliberately off, learning as before |
| `baseline test DISABLED: ... the REFERENCE ... is wrong` | safety net fired: value does not fit this machine |

