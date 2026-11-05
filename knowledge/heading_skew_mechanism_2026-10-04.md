# Heading-Skew: der Mechanismus, gemessen (2026-10-04)

Logs: `2026-10-04 20-59 0 Hof bach bruecke IMU ohne RTK.txt` und
`2026-10-04 21-03 0 bruecke bach halle imu.txt` (beide "schraeg" laut Operator).
Werkzeuge: `tools/analysis/_skewmap.ps1` (Orts-/Zeitkarte) und
`tools/analysis/_whybin.ps1` (alle HR-Spalten eines Zeitfensters + abgeleitete Groessen
**pro Frame**, nicht p50 aus gemischten Frames).
Operator-Angaben: AOG steht **immer auf +88 Grad** dualHeadingOffset; die 0 des
Lenkwinkels wird "trotzdem erkannt und staendig verbessert, auch wenn der Traktor im AOG
schraeg steht".

## 1. Die Identitaet: was der Operator sieht

`az_out = psi - dualHeadingOffsetDeg` (HeadingBridge.cpp:2135/3619), AOG rechnet
`body = empfangene Richtung + 88`. Also

    sichtbare Schraeglage = travel - (az_out + 88)
                          = travel - psi + (dualHeadingOffsetDeg - 88)

In beiden Logs ist `dualHeadingOffsetDeg` = 88,13 bzw. 88,25 -> **Restfehler 0,1...0,4 Grad**.
Bestaetigt durch die Kanalprobe `az_out - (psi + mount)` p50 = -0,17...-1,65 (Publish-Pfad
ist wirklich psi minus Offset).
=> **Die +88 in AOG sind NICHT die Ursache. Die sichtbare Schraeglage IST der psi-Fehler**
(plus 0,2 Grad).

## 2. Empfaenger, Positionen und COG sind in JEDEM Fenster sauber

`travel` = 0,5-s-Positionssehne (relativ, RTK-unabhaengig), geradeaus, vorwaerts.

| Log | Fenster | travel-psi (= Schraeglage) | travel-att | travel-cog |
|---|---|---|---|---|
| 21-03 (RTK) | 45..75 s | -0,58 | 88,23 | 0,09 |
| 21-03 (RTK) | 80..90 s | +2,86 | 88,55 | 1,64 |
| 21-03 (RTK) | 90..100 s | +4,78 | 88,25 | 2,17 |
| 21-03 (RTK) | 100..111 s | +11,25 | 89,52 | -1,10 |
| 20-59 (o. RTK) | 10..29 s | **+81,97** | 89,20 | 0,41 |
| 20-59 (o. RTK) | 40..60 s | -8,81 | 88,26 | -1,06 |
| 20-59 (o. RTK) | 60..91 s | -9,56 | 89,15 | 0,27 |

`travel-att` bleibt **87,2...89,5 Grad in allen Fenstern**, `travel-cog` **-1,1...+2,2**.
Die Dual-Attitude, der COG und die Positionen sind also *gegenseitig* konsistent - auch im
82-Grad-Startfenster ohne RTK. **Kein Antennen-, Multipath-, Sprung- oder qf-Thema.**
Die Schraeglage ist rein der psi-Fehler (Reihe 1).

## 3. psi bleibt abschnittsweise stehen - und zwar mit der Lage als Zeuge

`psi_att` = psi minus lageabgeleiteter Karosseriekurs (HeadingBridge.cpp:854), "muss
konstant bleiben":

| Fenster | psi_att | damit psi gegen travel |
|---|---|---|
| RTK 45..75 | +0,50 | 0 (gut) |
| RTK 80..90 | -2,30 | +2,9 |
| RTK 90..100 | -4,10 | +4,8 |
| RTK 100..111 | -9,50 | +11,3 (dort att_q -> 0, 46 Prozent qf-Rejects) |
| o. RTK 10..29 | **-80,00** | **+82** (erste 25 s!) |
| o. RTK 40..60 | +9,60 | -8,8 |
| o. RTK 60..91 | +10,30 | -9,6 |

Zusatz ohne RTK: `mount_deg` = -90,1 statt -87,8 (RTK) -> 2,3 Grad systematisch mehr
(die Messung `azRaw - refDir` steht ohne RTK anders).

## 4. Mechanismus: psi wird totgerechnet, die Lage zieht nicht

Direkt gemessen aus psi[k]-psi[k-1] ("realized"), gegen die Modellgierrate `yr`:

| Fenster | yr (Vorzeichen-Mittel) | realized dpsi | Befund |
|---|---|---|---|
| RTK 45..75 | +1,20 | -0,15 | beide ~0 (Fahrt war gerade) |
| RTK 80..90 | **-12,24** | **-10,76** | psi folgt dem MODELL |
| RTK 90..100 | -11,77 | -7,57 | psi folgt dem MODELL |
| o. RTK 10..29 | -3,76 | **-0,12** | psi steht 20 s still bei 80 Grad Fehler |
| o. RTK 40..60 | +8,72 | +5,69 | psi folgt dem MODELL |
| o. RTK 60..91 | +3,66 | **+0,01** | psi steht still |

Die Spalten `innov/gain/meas` behaupten im Startfenster ohne RTK eine Korrektur von
**+110 Grad/s** - realisiert werden **-0,12 Grad/s**. psi wird also praktisch nur von der
Praediktion bewegt; die Attitude-Korrektur kommt nicht durch (gain 0,02...0,23).
Ein **stehender** Offset bedeutet: eine falsche Modellrate wird von einer winzigen
Korrektur gerade gehalten - genau der Fall, den der Code selbst beschreibt
(HeadingBridge.cpp:2227: "a wrong wheel angle ZERO becomes a permanent heading offset
(rate*tau)"). Deshalb kann keine Filterlaufzeit den Offset entfernen.

## 5. Zwei Log-Defekte, die die bisherige Analyse in die Irre gefuehrt haben

1. `innov`, `gain`, `meas` werden **nur** geschrieben, wenn eine Messung greift
   (cpp:1124/1126, 1141/1142, 1362) und **nie** auf 0 zurueckgesetzt. Die Spalten sind
   damit **veraltet** (bis Minuten alt) und sehen trotzdem aktuell aus. `meas = 0 "none"`
   ist unerreichbar. Gleiches gilt fuer `rej` (letzter Ablehnungsgrund, nicht "diese
   Zeile").
2. `swapXY` in der 1-Hz-IMU-Zeile ist ein Etikett, kein Zustand
   (`bnoParser->getSwapXY()` wird nie gesetzt) - siehe BNOAiOParser-Hinweis.

## 6. Offen (zu besprechen, bevor gebaut wird)

Zwei moegliche Ursachen fuer die falsche Rate - messbar, nicht raten:

* **Modellseite**: Radwinkel-Nullpunkt / Radwinkel-Skala / Radstand. Die falsche Rate
  wechselt das Vorzeichen zwischen den Fahrten (-12 Grad/s mit RTK bei ~5 m/s,
  +3,7...+8,7 Grad/s ohne RTK) und der Offset folgt ihr in Groesse und Richtung. Passt zur
  Operator-Beobachtung, dass sich die Lenkwinkel-Null bewegt.
* **Fusionsseite**: die Attitude wird zu schwach durchgesetzt (R/gain) oder der
  Rate-Limit/Persistenz-Pfad laesst sie nicht durch, obwohl sie die belegte Wahrheit ist.

Entscheidender Test: `psi`-Aenderung pro Tick in **Modellanteil** und **Messanteil**
aufteilen und beide Spalten loggen (heute unmoeglich: `yr` ist nur die Rate,
`applied` wird nur als letzter Wert behalten). Danach ist mit einer Fahrt eindeutig
entscheidbar, wer den Offset haelt.

## 7. Nachtrag 2026-10-05 (Operator-Idee 1-m-fix-to-fix): die Injektoren

Werkzeug: `tools/analysis/_hsources.ps1` - Wahrheit = 1-m-Chord aus lat/lon, unabhaengig
von Attitude, IMU, Radwinkel und Modell. **Der Fehler wird in SPRUENGEN injiziert und
bleibt danach stehen.**

### 7a. Spaeter Sprung (RTK-Log 21-03, t = 80,8...82,4 s, "unter der Bruecke")

| t_s | dpsi | psi-f2f vor | nach | yr | wheel | v | state/src |
|---|---|---|---|---|---|---|---|
| 80,8 | +3,95 | 2,0 | 8,4 | +56,9 | -17,9 | 3,9 | LIVE/att |
| 81,1 | -3,56 | 9,3 | 7,5 | -33,7 | -17,9 | 3,9 | LIVE/att |
| 81,5 | -3,35 | 6,2 | 5,3 | -32,0 | -17,7 | 3,7 | LIVE/att |
| 82,4 | -3,14 | 2,7 | 3,9 | -30,0 | -14,5 | 4,3 | LIVE/att |

dpsi **-3,3 Grad pro Frame = -33 Grad/s**, Zustand LIVE/att (Attitude galt). Die
Bins davor standen bei psi-f2f = +7...+10, danach +1,6 - der Sprung hat den Offset neu
gesetzt. `wheel = -17,9 Grad` war echt (der VWAS-Beobachter bestaetigt gpsAngle -14,19
gegen enc -16,11), also ein sauberer Kurvenfall - hier zaehlt die Modell-SKALA.

### 7b. FRUEHE Injektionen (Operator: "die Fahrt war schon viel frueher schraeg")

Zeitachse t = 11...30 s (RTK-Log): `psi-f2f` steht **ab t = 11,5 s schon auf -8...-11,5**
- die Maschine war von Anfang an ~10 Grad schraeg. Zwei Ursachen:

* **t = 13,5...15,0 s: die IMU-Gierrate geht ins MODELL.** `imurate` rampt 13,8 -> 27,2
  -> 49,4 -> 71,3 -> **89,4 Grad/s**, und `yr` folgt ihr (85,9 Grad/s) - bei
  `wheel = 1,47 Grad` und v = 2,15 m/s. Die Kinematik haette dort **1,5 Grad/s**.
* **t = 25,5...29,7 s: `att = 0,00`** (Dual-Attitude weg) -> BRIDGE/COAST mit `src = cog`;
  psi schwingt in 4 s ueber +12,5 -> +120,8(!) -> -3,3 -> +18,4 -> -45,9.

### 7c. Wie oft? (imu_flags Bit 4 = "its yaw rate drives the model")

| Log | Frames mit IMU-Rate im Modell | \|imurate\| p50 | max | \|yr\| dort p50 |
|---|---|---|---|---|
| 21-03 (RTK) | 96 von 1143 (**8,4 %**) | **14,0 Grad/s** | 89,4 | 14,0 Grad/s |
| 20-59 (o. RTK) | 54 von 1047 (5,2 %) | 13,1 Grad/s | 34,7 | 13,1 Grad/s |
| 20-35 (o. RTK) | 62 von 1326 (4,7 %) | 0,0 Grad/s | 88,6 | 1,7 Grad/s |

**14 Grad/s p50 = 1,4 Grad Kopfdrehung pro 0,1-s-Frame, waehrend die Maschine geradeaus
faehrt** - physikalisch unmoeglich, von der Attitude widerlegt. Der Yaw-Bias-Lerner
(`yawbias` = 2,7 bzw. 7,8) frisst den Muell mit. Der bestehende Schutz greift nicht:
`imu_flags` Bit 8 deckt nur einen IMU-TYP ohne Gierrate ab (TM171) - ein BNO, dessen
Yaw-Signal selbst Muell ist, kommt durch.

### 7d. Warum die Injektionen bleiben: die gute Attitude darf nicht korrigieren

* `uni_hdgsd` (**eigene 1-sigma des Empfaengers je Epoche**) = **0,40...0,60 Grad**.
* `meas_r` (das R, das der Filter benutzt) = **4,0...5,6 Grad**.
* `effectiveMeasurementR(0)` = clamp(sqrt(EMA(innov^2) - p11), rAttMinDeg = 1 Grad,
  rAttMaxDeg = 12) * Empfangsqualitaet (HeadingBridge.cpp:910-930). Die Selbstschaetzung
  fuellt sich aus dem **Zustandsfehler**: ein bestehender Offset vergroessert R und
  senkt den Gewinn - der Offset schuetzt sich selbst.
* Folge: `gain` 0,02...0,23, Korrektur ~0,2 Grad/Tick = 2 Grad/s gegen eine Injektion
  von 14...33 Grad/s. Ein 10-Grad-Offset braucht Minuten - so lange faehrt der Traktor
  schraeg.

### 7e. Der Radwinkel ist NICHT schuld (Operator-Hinweis bestaetigt)

`VWAS zero:` steht im Log: `trim = -1,63 Grad` **konstant** ueber die ganze Fahrt,
`res` pendelt um 0 (-0,01...-0,66), `pred = enc + 1,63` liegt 0,2...0,4 Grad bei
`gpsAngle`. Die Null wird also erkannt und nachgefuehrt - der Hinweis des Operators war
richtig. Restabweichung: `mount_deg = -90,1` (ohne RTK) gegen -87,8 (RTK), also ~2 Grad
systematisch, und ohne RTK zusaetzlich ~7...10 Grad aus 7b/7d.

### 7f. UM982-only (Nachtrag 2026-10-05, Operator: "normalerweise nur UM982, die IMU ist nur zur Information")

Ohne IMU faellt Injektor 7b/7c weg. Die Logs als **UM982-only-Replay** gerechnet
(`_modelcheck.ps1 -SkipImuRate`, alle Frames mit `imu_flags` Bit 4 ausgeschlossen) - der
Modelltakt ist dann **die einzige Praediktion**:

| Log | Band 10...40 Grad wheel: Modell/Attitude | Signal vs Rauschen |
|---|---|---|
| 21-03 (RTK) | **1,54x** (Modell dreht zu viel) | 110,9 Grad vs +-1,7 -> belastbar |
| 2026-10-04 13-53 | **1,40x** | 127,8 vs +-2,6 -> belastbar |
| 2026-10-01 20-21 kreise | 0,80x | -168,7 vs +-2,6 -> belastbar |
| 2026-10-03 16-04 | 0,92x | -60,6 vs +-2,5 -> belastbar |
| Band 2...5 Grad | 0,84 ... 2,02 ... -4,88 | teils zu wenig Signal |

Ergebnis: **kein konstanter Skalenfehler, aber in engen Kurven dreht das Modell
systematisch zu weit (1,4...1,7x)** - genau der Faktor ~1,38, den
`HeadingBridge.h:122` selbst nennt ("2,14 m wheelbase -> 2,95 m effective wheelbase",
Ursachen: VWAS-Skala, Reifenschlupf/Untersteuern). Der **Nullpunkt ist es nicht**
(VWAS-Beobachter `trim = -1,63` konstant, `res ~ 0`) - und dieser Beobachter kann einen
Wheelbase-Fehler grundsaetzlich nicht sehen, weil er `gpsAngle` und `enc` mit DERSELBEN
2,14 m rechnet. Das Firmware-eigene Lernverfahren kennt den Faktor laengst
(`steerScaleEstimate` = `sc` im Log: 1,00 im guten, 1,88 im schlechten Abschnitt), aber
`Config::applyLearnedSteerScale = false` - **der gelernte Wert wird nur berichtet, nie
angewendet**.

### 7g. Rangfolge fuer die reale Maschine (UM982 only)

1. **Modell dreht in Kurven zu weit** (1,1...1,7x, in engen Kurven ~1,5x): jede Kurve
   injiziert Heading (Ueberschuss = (ratio-1) * Kurvenwinkel), und der Effekt hat das
   Vorzeichen der Kurve - passt zu "an vielen Stellen schraeg" und zum Vorzeichenwechsel
   zwischen den Abschnitten.
2. **Die verifizierte Attitude kann nichts zurueckholen**: R = 4,0...5,6 Grad, obwohl der
   Empfaenger sein eigenes 1-sigma `uni_hdgsd` = 0,40...0,60 Grad mitliefert -> gain 0,02
   -> 2 Grad/s gegen eine Injektion von 14...33 Grad/s. **Mit R = 0,5 Grad waere K1 = 0,47
   -> 47 Grad/s -> jeder Offset waere in 0,2...0,5 s weg.** Das ist der Hebel mit der
   groessten Wirkung gegen ALLE Injektoren (auch die noch unbekannten).
3. **COG/Pfad-Fallback schwingt**, wenn die Dual-Attitude wegfaellt (`att = 0`):
   +12,5 -> +120,8 -> -3,3 -> +18,4 -> -45,9 in 4 s (t = 25,5...29,7 s). IMU-unabhaengig,
   also auch auf der realen Maschine vorhanden (Baeume/Bruecke).
4. **IMU-Gierrate** nur noch defensiv absichern, falls doch eine verbaut wird (13...14
   Grad/s p50, bis 89 Grad/s, 5...8 % der Frames - und Roll -183 Grad).


## 9. Korrektur 2026-10-05 (Werkzeug-Bug) und die GERADEAUSFAHRT

`_straight.ps1` und `_hsources.ps1` rechneten den Positionskurs mit `cos(lat[0])`, und
`lat[0] = 0.0000000` (erste Zeilen vor dem ersten Fix) -> cos = 1 statt 0,657 -> die
Ostkomponente war 1,52x zu gross -> **jeder Positionskurs war ~7,5 Grad nach Osten
gedreht**. Behoben (cos pro Frame). Damit stimmen Positionskurs und COG ueberein
(`cog-psi` +0,15 gegen `psi-crs` +0,01), der Montageversatz liest sich wieder als
`att-cog = -88,2...-88,9`, und der fruehere Befund "auf Geraden +7...+10 Grad" im
RTK-Log war ein Artefakt. Die **Slam-Erkennung** (ueber dpsi und `yr`, ohne Position)
war davon nicht betroffen.

### 9a. Geradeausfahrt (`_straight.ps1`): Geradeaus aus den POSITIONEN

Kriterium: Kursaenderung pro Sekunde aus dem 1-s-Positionskurs < 1,5 Grad, v >= 2 m/s,
|Lenkwinkel| <= 1,5 Grad. Aussagekraeftig sind `cog-psi` (die sichtbare Schraeglage,
cog = eigene Doppler-Richtung des Empfaengers) und `psi-crs` (Kontrolle gegen den
Positionskurs). `psi_att` ist nur auf gueltigen Attitude-Frames frisch.

| Log | cog-psi auf bestaetigten Geraden | Verhalten |
|---|---|---|
| 21-03 UM982+RTK | p50 **-0,91 Grad**, max 3 | gut |
| 13-53 UM982 | p50 -9,15 | t=35,7: **+0,6** -> t=48,3...72,6: **-9,1...-10,0** (35 s) - Sprung +9,3 Grad zwischen zwei Geraden |
| 16-04 UM982 | p50 -5,53 | -1,7 -> -5,5 -> **-35,1** -> -12,5 -> -10,9 -> -0,4 -> +1,7 |
| 17-28 UM982 | p50 **-9,49**, sd **0,17** | 45 s lang konstant 9,5 Grad |
| 20-59 UM982, ohne RTK | **+97,5** (t=20 s) und -10,9 (t=90) | |
| 17-09 **F9P+IMU** (kein Dual) | p50 **-67,5**, sd 21,8 | att% = 0, die COG/IMU-Quelle allein sitzt nicht |

`att-cog` bleibt in ALLEN Zeilen stabil bei -88,2...-88,9 Grad: die Dual-Attitude ist die
Wahrheit, und psi folgt ihr nicht. **Der Fehler entsteht nicht beim Geradeausfahren,
sondern zwischen den Geraden (Kurve/Bruecke) und bleibt dann stehen** - die
Geradeausfahrt zeigt nur, dass er da ist.

### 9b. Wie das VWAS "geradeaus" erkennt (Operator-Frage 2026-10-05)

`WheelAngleFusion.cpp:923-985` - drei Gates, kein Positionskurs:
1. `abs(currentAngle) > config.straightDriveMaxAngle` -> keine Trimmung,
2. `isValidGPSConditions()`: `speed >= config.minSpeedForGPS`, `abs(headingRate) <
   config.maxHeadingRate`, `abs(fusedAngle) < 30`,
3. Ausreisser `abs(gpsAngle - currentAngle) > config.driftInnovationRejectThreshold`
   wird KOMPLETT verworfen (keine Teilkorrektur), sonst nur ratenbegrenzt
   (`step = innovation*0.1`, hart gedeckelt durch `maxDriftCorrectionRate`).

`gpsAngle = atan(headingRate * wheelbase / speed)` (Zeile 959-979), also mit DERSELBEN
Wheelbase - deshalb funktioniert die Null auch ohne RTK (nur relative Richtungsaenderung
noetig) und deshalb sieht sie einen Wheelbase-Fehler nicht. **Das Muster ist
"Witness + Ausreisser verwerfen + anhaltendes Signal gewinnt"** - genau das fehlt beim
Heading: dort ist die Attitude heute ein weiches Messrauschen (R = 4...5,6 Grad) statt
ein Zeuge, und ein 10-Grad-Offset ueber 35 s Geradeausfahrt bleibt stehen.


Reihenfolge nach Wirkung fuer die **reale Maschine (nur UM982)** - siehe 7g:

## 10. Vorschlag (Stand 2026-10-05, nach der Geradeaus-Messung)

* **Attitude als ZEUGE behandeln statt als weiches Messrauschen** - genau das VWAS-Muster
  (9b): Ausreisser verwerfen statt teilweise anwenden, eine ANHALTENDE Abweichung
  ratenbegrenzt nachfuehren (nicht mit K1 = 0,02), und ab ~20...40 Grad neu seeden
  (heute `reseedBeyondDeg = 120`). Dann kann kein 10-Grad-Offset 35 s Geradeausfahrt
  stehenbleiben.
* **Einheitliche Basis fuer Heading und VWAS** (Operator-Ziel): der 1-s-Positionskurs
  (bzw. der COG, wenn gueltig) als gemeinsamer Bezug. Er funktioniert auf F9P+IMU,
  2xF9P und UM982, weil er nur die RELATIVE Geometrie der Fixes braucht - und er ist
  die Groesse, gegen die das VWAS seinen `gpsAngle` ohnehin bildet. Der Roll ist der
  zweite Teil dieser Basis (bei gutem Empfang gut, kippt bei schlechtem weg - 9a/7g).
* **Vor dem Bauen simulieren**: psi aus den vorhandenen HR-Logs offline nachrechnen
  (Attitude als Zeuge, R aus `uniHdgStdDevDeg`, Re-Seed-Schwelle variieren) und mit
  `cog-psi` auf den Geradeaus-Segmenten messen. Die Logs dafuer sind da (13-53, 16-04,
  17-28, 20-59, 21-03, 17-09).

Danach die frueher notierten Punkte:

## 11. EINBAU-PLAN (vom Operator beschlossen 2026-10-05)

Beides in EINEN Build, dann Testfahrt mit HR-Log. Reihenfolge = Abarbeitungsreihenfolge.

### A. Faktor 1,3 als EEPROM-Wert "effektive Wheelbase" (Muster: dualHeadingOffsetDeg)

1. `lib/aio_config/EEPROMLayout.h` - im Bridge-Block (1600...1699, wo dualHeadingOffsetDeg
   in Byte 11...16 liegt) den NAECHSTEN freien 4-Byte-Float belegen, z. B. Byte 17...20:
   `effWheelbaseFac` (Default 1.0, gueltig 0.6...1.6). Vor dem Schreiben die tatsaechliche
   Belegung im Block pruefen - die Bytes muessen innerhalb 1600...1699 bleiben.
2. `lib/aio_config/ConfigManager.{h,cpp}` - `getBridgeEffWheelbaseFac()` /
   `setBridgeEffWheelbaseFac(float, bool save)`, Default 1.0, Klemme 0.6...1.6,
   dazu die Init-/Ladezeile wie bei getBridgeDualHeadingOffsetDeg().
3. `lib/aio_navigation/HeadingBridge.{h,cpp}`
   * `Config` bekommt `float effWheelbaseFac = 1.0f;`
   * `init()`: laden wie cpp:124-133 (`storedOff`-Muster) + LOG_INFO mit dem Wert.
   * **Anwenden an BEIDEN Stellen, an denen L gebildet wird:**
     - cpp:950 `kinematicRateBoundDegS()`: `const float L = ((config.wheelbaseM > 0.5f) ? config.wheelbaseM : 2.50f) * config.effWheelbaseFac;`
     - cpp:1855 (Praediktion): dieselbe Zeile.
     - cpp:1991/1994 (`incX`/`incQ`, Regressoren des steerScale-Schaetzers): ebenfalls mit
       L_eff, sonst aendert sich die Bedeutung des gelernten `steerScaleEstimate`.
   * `setEffWheelbaseFac(float)` mit Bereichspruefung + EEPROM-Speichern + LOG_NOTICE
     ("enter the SAME value in AgOpenGPS? NEIN - hier ist es die effektive Laenge fuer
     das Modell, AgOpenGPS's Vehicle-Wheelbase bleibt unveraendert").
4. `lib/aio_system/SimpleWebManager.{cpp,h}` - den bestehenden Endpunkt
   `/api/bridge/geometry` (mit dualHeadingOffsetDeg + Antennenabstand) um
   `effWheelbaseFac` (GET + POST mit Klemme) erweitern; Anzeige im Web-UI daneben.

### B. Attitude als Zeuge (R-Deckel + Re-Seed)

5. `HeadingBridge.cpp` `effectiveMeasurementR()`, nach der Qualitaetsmultiplikation
   (cpp:926), fuer `kind == 0`:
   ```cpp
   // Deckel: der Empfaenger liefert sein eigenes 1-sigma je Epoche (uniHdgStdDevDeg).
   // Der adaptive Wert fuellt sich aus dem ZUSTANDSfehler und schuetzt den Offset
   // (gemessen 2026-10-05: meas_r 3,95 -> 6,27, gain 0,016). Untergrenze 1,5 Grad,
   // damit Rauschen weiter gemittelt wird (Attitude-Streuung 0,2 Grad/Frame).
   const float sd = gnssProcessor.getData().uniHdgStdDevDeg;
   float cap = config.attRCapDeg;                 // Default 1.5f
   if (sd > cap) cap = sd;                        // schlechter Empfang -> Deckel weg
   if (q > cap) q = cap;
   ```
   Sinn: K1 = p11/(p11+R^2) = 0,22/(0,22+2,25) = **0,09 statt 0,016** - die verifizierte
   Attitude holt rund **6x schneller** zurueck, ohne dass ein schlechter Empfang sie
   hineinzieht (dann greift der adaptive/qualitaetsbewertete Wert wieder).
6. `HeadingBridge.h:255` `reseedBeyondDeg = 30.0f` (statt 120): ein anhaltender
   20...40-Grad-Fehler gegen eine gueltige Attitude ist ein falscher ZUSTAND
   (belegt: 80 Grad im Startfenster ohne RTK blieben stehen, `psi_att = -80`).
   Der Re-Seed-Pfad verlangt bereits `kind == 0` (Attitude) - der Positions-Witness
   fuer kind 3 bleibt unveraendert.

### C. Pruefen nach der Fahrt
7. HR-Log mit `_straight.ps1` (cog-psi auf den Geradeaus-Segmenten; Erwartung:
   |cog-psi| p50 < 2 Grad in ALLEN Logs) und `_whybin.ps1` (meas_r / gain / psi_att).
8. `_psisim.ps1` ist fertig und mit denselben Parametern (Fac 1,3 / RCap 1,5 /
   Reseed 30) lauffaehig - es braucht nur einmalig das Vorab-Extrahieren der
   Spalten in `double[]` (PowerShell-Funktionsaufrufe pro Frame kosten hier > 30 s).


1. **Attitude-Gewicht ehrlich machen** (groesster Hebel): R der Attitude nach dem
   Empfaenger-eigenen 1-sigma `uniHdgStdDevDeg` (0,4...0,6 Grad, ist schon geparst und
   wird von `attitudeQualityNow()` benutzt) begrenzen bzw. einen harten Deckel setzen.
   Damit raeumt die verifizierte Attitude JEDE Injektion in ~0,5 s ab, egal woher sie
   kommt. Heute: R = 4...5,6 Grad -> K1 = 0,02 -> 2 Grad/s.
2. **Effektive Wheelbase / steerScale anwenden**: das Modell dreht in engen Kurven
   1,4...1,7x zu weit (`HeadingBridge.h:122`: 2,14 m -> effektiv 2,95 m). Das
   Lernverfahren existiert (`steerScaleEstimate`, im Log `sc` = 1,00...1,88), wird aber
   wegen `applyLearnedSteerScale = false` **nicht angewendet**. Vorschlag: den Faktor
   gedrosselt/geklammert anwenden (oder die effektive Wheelbase als EEPROM-Wert, analog
   `dualHeadingOffsetDeg`).
3. **COG/Pfad-Fallback zaehmen**: bei fehlender Dual-Attitude schwingt psi in 4 s um
   +-28 Grad (t = 25,5...29,7 s im RTK-Log). Nur mit Positions-/Attitude-Witness
   nachfuehren, sonst halten.
4. `reseedBeyondDeg = 120` senken: 80 Grad Fehler im Startfenster ohne RTK blieben stehen
   (`psi_att = -80`). Mit dem Positions-Witness als Beleg sollten auch 20...40 Grad neu
   seeden.
5. **Defensiv** (falls doch eine IMU verbaut wird): IMU-Gierrate plausibilisieren
   (13...14 Grad/s p50, bis 89 Grad/s, 5...8 % der Frames) - der Roll ist ein separates
   Thema (knowledge/imu_roll_units_and_quality_2026-10-05.md).
6. Log ehrlich machen: `innov/gain/meas/rej` sind nur die **letzten** Werte einer
   greifenden Messung (nie auf 0 gesetzt, `meas = 0` unerreichbar), `swapXY` in der
   1-Hz-IMU-Zeile ist ein Etikett, `imu_flags` Bit 4 fehlt als Zahl in den
   Analyseskripten.
