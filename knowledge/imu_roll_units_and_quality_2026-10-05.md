# IMU roll: units settled, quality is the open item (2026-10-05)

## 1. Units: the IMU roll/pitch are DEGREES, factor 1 - settled

Settled by three independent checks, after a wrong "factor 10" conclusion of mine:

1. **Field:** the operator (MTZ8302, author of the 2026-05-07 fix in `BNOAiOParser.cpp`) confirms
   the roll reaches AgOpenGPS correctly, and asked for the value to stay unchanged.
2. **AgIO:** `AgIO/NMEA.Designer.cs:38` `varRoll = 0.1f` is **not** a scaling - its only use is
   line 858 `G = Pc / (Pc + varRoll)`, a filter gain. And `ParsePANDA()` documents its own fields:
   "(12) Heading in degrees, (13) Roll angle in degrees (positive roll = right leaning),
   (14) Pitch angle in degrees". So the PANDA roll field is **degrees**, no hidden scaling.
3. **Geometry (the yard test, F9P 17-09):** log started in the yard, tilt 0 ... 3 deg,
   <= 5 ... 8 deg if a mounting error is included; level until after the second 90 deg turn.

| t [s] | v [m/s] | imu_roll p10 / p50 / p90 | max |
|---|---|---|---|
| 0 | 0.00 | -3.9 / -3.5 / -3.3 | 4.5 |
| 5 | 0.00 | -5.1 / -4.8 / -4.0 | 5.2 |
| 10 | 0.00 | -4.8 / -3.6 / -3.3 | 5.1 |
| 15 | 0.00 | -5.4 / -4.9 / -3.9 | 5.5 |
| 20 | 0.02 | -5.2 / -4.2 / -3.4 | 5.4 |

**3.2 ... 5.5 deg while standing in a yard with 0 ... 3 deg plus mounting - factor 1, correct.**
=> no scale change; my `* 0.1f` "restore" and the IMU-roll envelope were both reverted.
=> the in-code units note in `BNOAiOParser.cpp` (`yaw` field 0.01 deg -> scaled; `pitch`/`roll`
fields 0.1 deg -> taken as is; `getYaw()/getPitch()/getRoll()` return degrees) matches this.

## 2. What the same log shows instead: the roll is only usable at a STANDSTILL

Same drive, 10 s bins of |imu_roll|:

| t [s] | v p50 | p50 / p90 / max | > 15 deg | > 25 deg (AOG clamps to 0) |
|---|---|---|---|---|
| 0 ... 25 (standing) | 0.00 | 3.9 / 5.0 / 5.2 | **0 %** | **0 %** |
| 30 | 3.22 | 6.9 / 36.3 / 42.9 | 33 % | 23 % |
| 40 | 3.51 | 35.9 / 49.7 / 59.2 | 88 % | 77 % |
| 60 ... 70 | 5.9 ... 10.8 | 22 / 32 ... 33 / 42 | 76 ... 80 % | 37 ... 38 % |
| 130 ... 150 | 1.5 | 12 ... 15 / 16 ... 23 / 26 | 20 ... 59 % | 0 ... 3 % |
| 170 (stopped again) | 0.02 | 36.0 / 37.9 / 37.9 | **100 %** | **100 %** |

17-18 recovers the same way: t 70 ... 90 (v -> 0) back to **2.0 ... 4.0 deg**.

So: standing = correct, driving = up to 79 deg, and it re-converges after a stop - but it also
FAILS at a standstill at times (17-09 t=170). That is a fusion/sensor quality signature (BNO085
RVC roll under machine vibration / disturbed calibration), **not** a scaling problem. Nothing
should be "fixed" in the scale for it.

## 3. Architectural finding: on a single-antenna machine AOG's roll is the RAW IMU value

`NAVProcessor` (PANDA, the message a non-dual machine sends) fills field 13 from
`imuData.roll` **directly** - not from the bridge's roll channel:

* F9P (no dual): AOG roll = `imuData.roll` (raw IMU), filtered only by AOG's own `rollFilter`
  and offset by `rollZero`. `HeadingBridge::updateRoll()` (LPF, hold, fade) is bypassed for AOG;
  it only feeds the VWAS.
* UM982 (dual): PAOGI is sent instead and takes `rollOut = bs.rollDeg` (the bridge channel), so
  the receiver's roll - or the IMU roll when the receiver degrades.

Consequence: damping the IMU roll for AOG on an F9P machine must happen in the IMU/PANDA path
(or by making PANDA use `bs.rollDeg`), not in `updateRoll()` alone. A behaviour change - ask first.

## 4. Open questions for the operator

1. Is the wandering roll while driving visible/acceptable in AgOpenGPS (does the roll needle move
   around), or does the `rollFilter` hide it? It also drives the side-hill draft and AOG's
   `tan(roll) * -antennaHeight` POSITION correction, so it is not cosmetic.
2. Do we log the raw BNO triple (`yaw/pitch/roll`, 1-2 Hz)? That would show whether the *pitch*
   swings with the roll (vibration / whole-fusion problem) or only the roll (axis/mounting).
3. `isUseYAxis` on the F9P machine: the axes are described as nearly cleanly aligned, so it should
   be OFF - worth confirming, because that option swaps roll and pitch in `IMUProcessor.cpp:252`.

## 6. Axis configuration (operator 2026-10-05: "es wird x genutzt")

The AOG setting behind `ConfigManager::isUseYAxis` (PGN 251 bit 3, read in
`AutosteerProcessor.cpp:1075`, applied in `IMUProcessor.cpp:252`) is OFF on the F9P machine: the X
axis is used, `currentData.roll = getRoll()`, no swap of roll and pitch. That matches the yard
measurement in section 1 and the operator's statement that the board is firmly mounted with the
axes nearly cleanly aligned.

Open: the ~9.5x slope between `imu_roll` and the RECEIVER roll on the UM982 drives (17-28: 9.72
at r 0.77, 17-31: 9.14 at r 0.83) is still unexplained. If that machine has the swap ON, the
column holds the PITCH there and the comparison was a pitch-against-roll regression. Flagged as a
machine-configuration question - the UM982 uses the receiver roll in ~85 % of the frames anyway.

## 7. Roll and position must be aligned in TIME (operator warning 2026-10-05)

> "in AOG wird die Position mittels roll und Antennenhöhe korrigiert. Roll muss zeitlich direkt
> zur Position passen. Im Code müsste eine kurze Verzögerung drin sein, da der BNO schneller ist
> als der f9p. Im PANDA sind die Daten dann synchronisiert übertragen."

Checked - the warning is correct, and the code does NOT do it yet:

* `Position.designer.cs` applies `rollCorrectionDistance = tan(roll) * -AntennaHeight` to all
  three `stepFixPts`, so the roll must belong to the SAME epoch as the position AOG corrects.
* `NAVProcessor::formatPANDAMessage()` (line 157) and `formatPAOGIMessage()` (line 319) both take
  `imuProcessor.getCurrentData()` - the FRESHEST IMU sample - while the position comes from the
  last parsed fix. No delay, no buffer, no timestamp pairing: the only uses of `imu.timestamp`
  are freshness checks (`imuAge = millis() - imu.timestamp`) in `HeadingBridge`.
* The BNO is faster than the F9P (100 Hz packets vs a 10 Hz fix that additionally carries the
  receiver's own computation and transmission latency), so the roll LEADS the position it is
  attached to. With the measured roll swing while driving (section 2: 20 ... 40 deg within 1 s)
  that lead lands directly in `tan(roll) * 3 m`.

=> Requirement (behaviour change - ask first): pair the roll with the fix epoch, either by
delaying the IMU contribution by the MEASURED GNSS latency, or by timestamping both and taking
the IMU sample nearest the fix time. The delay constant must be measured, not guessed.

### 7b. Checked on request: there is NO IMU ring / delay buffer in the firmware

Searched at three levels:

1. `IMUProcessor` (private members): a single `IMUData currentData`, plus `timeSinceLastPacket`
   (elapsedMillis), `lastSerialDataTime`, `serialDataReceived`, the parser pointers and the
   detected type. **One sample, no history.**
2. `BNOAiOParser` (private members): `state`, `buffer[PACKET_SIZE]` + `bufferIndex` (the byte
   buffer for ONE incoming 19-byte packet), `yawX10/pitchX10/rollX10/yawX100` (the last parsed
   values), `prevYaw` (only to difference the yaw for the angular velocity), `angVel/angCounter`,
   `lastValidTime`, `dataValid`, `isSwapXY`. **No sample history either.**
3. The only ring-like structure in `lib/aio_navigation` is the NMEA character buffer in
   `GNSSProcessor` (`parseBuffer`/`bufferIndex`) - a sentence parser, not an IMU history.

`NAVProcessor::formatPANDAMessage()` (line 157) and `formatPAOGIMessage()` (line 319) therefore
always use the FRESHEST IMU sample. The delay the operator describes ("da der BNO schneller ist
als der f9p, im PANDA sind die Daten dann synchronisiert") does **not** exist yet - it has to be
BUILT, and it is a behaviour change.

How it would be built (for discussion, nothing implemented):

* a small IMU sample ring in `IMUProcessor` (e.g. 8 ... 16 samples with `timestamp`), and
  `getAtMs(now - gnssLatencyMs)` / `getNearest(fixEpochMs)`; or
* the simplest variant: keep a short delay of the roll (and pitch) by a constant that is
  **measured**, not guessed.
* The constant needs the receiver latency: the fix carries a time of day
  (`gpsData.fixTime + fixTimeFractional`), and `gpsData.lastUpdateTime` is the local millis when
  it was parsed. With a millis<->GPS-time anchor the latency is computable - a diagnostic column
  (or a short measurement run) is missing for that today.

firmware - it is dead code, which is exactly why the 10x in its `rollX10` conversion never
mattered. The roll reaches AOG only through PANDA on a single-antenna machine, i.e. in the same
sentence as the position - which is what makes a time alignment possible at all.

## 8. Implemented 2026-10-05 (no behaviour change)

* `IMUProcessor::processBNO085Data()` calls `bnoParser->printDebug()` at ~1 Hz (rate limited,
  `Serial.availableForWrite()` gated):
  `BNO: yaw=.. pitch=.. roll=.. rate=.. valid=.. swapXY=.. x10=yaw/pitch/roll`
  Degrees plus the stored 0.1 deg fields, so axis and unit questions are answerable from any log.
* `printDebug()` is FLASHMEM (verified in the ELF: 0x60006938, 168 B) - ITCM grew by 16 B only.
* NOT in the ring buffer (operator: the raw triple "ist eigentlich nicht Gegenstand" of the
  position/state ring) - a sensor diagnostic line, not a per-fix record.


The 2026-10-05 mistake came from two unverified assumptions: "the boot line is a measurement"
(it is the FIRST packet, ~10 ms after init, fusion not converged) and "the route has <= 5 deg"
(used as a terrain bound on a column whose meaning depends on the axis configuration). Both are
now written down at the code site (`BNOAiOParser.cpp`) so they cannot repeat.
