# HeadingBridge – konsolidierter Analysebericht (Stand 2026-10-02)

Nur Analyse, kein Produktivcode geändert. Basis: `knowledge/heading_bridge_concept.md`,
Logs `2026-10-01 *` (18 Logs, inkl. Abend-Logs mit BNO085) und
`2026-10-02 11-27 halle Bäume IMU roll am Ende.txt` (92 Spalten, 1181 Frames, 118 s).
Werkzeuge: `tools/analysis/HrLog.ps1` + Ad-hoc-Skripte (gelöscht nach Gebrauch, Befunde
unten vollständig mit Rohwerten belegt).

---

## 0. Neuer Befund zuerst (Operator-Priorität): Heading-Runaway OHNE Bäume, OHNE GNSS-Fehler

**Log 2026-10-02 11-27, t = 90,0–93,4 s.** Fahrsituation laut Betreiber: lang gezogene
Rechtskurve, kaum Lenkbewegung nötig, Geschwindigkeit sehr gleichförmig. Das ist durch die
Daten exakt bestätigt:

- `v_mps`: 8,0–8,4 m/s, keine Sprünge.
- `wheel_deg`: 0,6° → 2,0° über die ganze Phase – minimale, stetige Lenkbewegung.
- Bahn-Azimut aus Positions-Chords (1-s-Fenster): dreht sauber und monoton 297°→334° über
  ca. 20 s – eine langgezogene Kurve, kein Zick-Zack, kein Richtungswechsel.
- `qf` durchgehend 4/5, `uni_hdgsd` 0,9–1,3° (sauber), `baseline_m` 0,67–0,72 m (passt zur
  gelernten Referenz dieses Logs), kein einziges `rej`≠`none` in diesem Fenster.

Trotzdem läuft `psi` in 3,4 s von der Dual-Attitude weg: `|psi − (att+90°)|` steigt von 0,0°
auf 95,0° (t=90,0→91,93s) und schwingt danach in einer Art Fehlregelung bis t≈94s zurück
(Werte: 0.9/12.0/27.0/48.0/72.3/90.1/95.0° Fehler über die Sequenz).

**Ursache ist im Code eindeutig nachweisbar, nicht hypothetisch:** `imu_flags` kippt bei
t=90,23s von 1 auf 5 (Bit 2 = „IMU-Rate treibt das Modell“, `HeadingBridge.cpp:1541`
`imuRateActive = true`), und `imurate` liefert in diesem Moment 78,7°/s, fallend über die
Episode bis ~55°/s. Bei einem stehenden/gleichförmig fahrenden Fahrzeug ist das physikalisch
falsch – es ist exakt der in Befund E beschriebene **BNO085-RVC-Sägezahn**: RVC liefert keinen
echten Gyrokanal, `BNOAiOParser::parsePacket()` akkumuliert die Yaw-Differenz über 20 Pakete
und setzt sie dann hart auf 0 zurück (`BNOAiOParser.cpp:130-141`, `angCounter`/`angVel`). Die
`yawbias`-Lernschleife in `HeadingBridge.cpp:1519` (`yawBiasDegS += yawBiasAlpha*(r−dPsi)`)
läuft diesem Sägezahn hinterher und bleibt am Clamp ±30°/s hängen (siehe Logwerte: `yawbias`
geht von −22,93° über −4,x° auf +30,00° und verharrt dort).

**Unterschied zur bisherigen Dokumentation von Befund E:** Dort war der Sägezahn als
Gierraten-*Rauschen* und `imu_flags`-Fehlmarkierung belegt (35–103 Frames je Log). Hier zeigt
sich zum ersten Mal, dass dieser Sägezahn **für sich allein** – ohne Bäume, ohne GNSS-Ausfall,
ohne Radwinkel-Bias (siehe unten) – einen vollen psi-Runaway von bis zu 95° auslösen kann,
weil `imuRateActive` das kinematische Modell (das stabile `v·tan(δ)/L`) temporär komplett
ersetzt (`HeadingBridge.cpp:1563-1567`: `if (imuRateActive || …) { yawRateDegS = imuRateActive
? yawRateDegS : …; psi = wrap360(psi + yawRateDegS*dtS); }`).

**Abgrenzung von Befund R1 (Radwinkel-Bias):** Ausdrücklich NICHT die Ursache hier. `wq`
(Wheel-Quality) liegt die gesamte Phase bei 12, weit unter der Sperrschwelle
`wheelQualityLowPct=30`. Das Radwinkelmodell ist korrekt deaktiviert
(`yaw_zeroed=1` außerhalb der IMU-Episode, `bias_runaway.ps1` meldet für dieses Log: „wq never
crossed 40 upward“, `max|bias|=15,0°` nur als Lernartefakt ohne Wirkung). Der hier gefundene
Fehlerpfad ist komplett unabhängig von R1 und bisher NICHT in den Abschnitten R1–R3
dokumentiert.

**Relevanz für die Steuerung – Betreiber-Korrektur beachtet:** Schlechtes Autosteer-Verhalten
braucht keine großen Sprünge; 1–2° reichen bereits, weil der Lenkregelkreis jede Änderung von
`az_out` direkt als Kurskorrektur verwendet. Der hier gemessene 95°-Ausreißer ist also ein
Extremfall dieses Mechanismus, aber derselbe Pfad (`imuRateActive` mit verrauschter Rate)
erzeugt auch bei kleineren Sägezahn-Amplituden kontinuierlichen Low-Level-Jitter, sobald
`imu_flags` Bit 2 gesetzt ist (47 von 1181 Frames in diesem Log, 4 %). Zur Einordnung: Auf den
ruhigen COAST-Abschnitten *ohne* IMU-Rate-Einfluss war der `az_out`-Jitter sehr klein (p50
0,20°, p90 0,40°, max 0,70° pro 100-ms-Frame, n=44, Fenster 96–113 s) – die IMU-Rate-Injektion
ist hier die mit Abstand dominante Störquelle für dieses Log, nicht die übliche Rezeptions-
qualität.

**Operator-Aussage „IMU wird vermutlich nie zusammen mit Dual verwendet“:** Das deckt sich mit
dem Befund – solange `useImuRate=true` per Default aktiv ist UND ein BNO085 angeschlossen ist,
kann dieser Pfad auch bei intaktem Dual-Antennen-Empfang zuschlagen. Die einfachste Abhilfe
(Vorschlag, noch nicht umgesetzt) ist in Abschnitt E/Stufenplan benannt.

---

## 1a. Neigetest BNO085 (Log 2026-10-02 11-27, Ende, t≈113–135 s)

Zwei auswertbare Phasen:

**Phase 1 – Rollausschlag während langsamer Fahrt/Ausrollen (t=113–117s, v 5,8→0,5 m/s):**
Dual-Roll (`roll_deg`, Wahrheit) läuft von ~0° auf −78° und wieder zurück auf ~0°; in exakt
diesem Fenster war `base_dev`/`baseline_m` instabil (0,67 m → 2,6 m → zurück), `uni_hdgsd`
schoss auf >1000–1800° (Ausreißer-Kennzeichnung der UM982 selbst) – die Firmware verwirft
diese Frames korrekt (`rej=rollEnv`, `state=COAST`). Das ist eine Artefaktphase der Dual-
Lösung selbst (vermutlich kurzer GNSS-Verlust durch Abschattung unter Bäumen am Fahrtende,
nicht Gegenstand des Neigetests).

**Phase 2 – bewusster Neigetest im Stillstand (t≈118–135s, v≈0,00–0,11 m/s):**
- `roll_deg` (Dual-Attitude, Wahrheit): bleibt flach bei −0,3° bis −3,4° – **keine reale
  Neigungsantwort sichtbar**, nur Sensorrauschen im Bereich der üblichen Attitude-Streuung.
- `imu_roll` (BNO085 roh): driftet **monoton** von −53,7° bei t=117,6s auf −122,6° bei
  t=134,6s, ohne Umkehr, ohne Korrelation zum tatsächlichen Neigungsverlauf.
- `uni_pitch` (UNIHEADINGA-Spalte) ist über die ganze Phase numerisch identisch mit
  `roll_deg` (z. B. t=117,63: roll=−0,32, uni_pitch=−0,32) – das ist eine reine
  Spaltenzuordnungs-Übereinstimmung des Logformats (`uni_pitch` wird hier als Pitch-Spalte
  der UNIHEADINGA-Sentence geführt, de facto gleichauf mit dem Dual-Roll-Kanal dieses
  Datensatzes), kein BNO085-Bezug.

**Interpretation:** Der BNO085-Rollkanal zeigt in der Stillstandsphase **kein** Ansprechen auf
die physische Neigung, sondern einen über >15 s linear laufenden Drift von knapp 70°. Das ist
kein Vorzeichenfehler und keine gestauchte Amplitude (beides würde eine – wenn auch falsch
skalierte/invertierte – Korrelation zur Neigungsrichtung zeigen). Es ist eine **freilaufende
Integrationsdrift ohne Ankerpunkt**: Pitch/Roll des BNO085-RVC-Pakets sind nach Sensor-
Datenblatt intrinsisch absolute, nicht integrierte Größen, ABER der Parser-Code enthält einen
verdächtigen Eingriff:

```
// BNOAiOParser.cpp:144-151
// Convert to degrees x 10 (original values are in 0.01 degree units)
// So divide by 10 to get 0.1 degree units
yawX10 = (int16_t)(rawYaw * 0.1f);
if (yawX10 < 0) yawX10 += 3600;
// pitchX10 = (int16_t)(rawPitch * 0.1f);
// rollX10 = (int16_t)(rawRoll * 0.1f);
pitchX10 = (int16_t)(rawPitch);//bugfix MTZ8302 2026-05-07
rollX10 = (int16_t)(rawRoll);
```

Yaw wird korrekt mit `*0.1f` skaliert (Hundertstelgrad → Zehntelgrad-Einheiten, wie
`getYaw()` mit `/10.0f` erwartet). Für Pitch/Roll wurde genau diese Skalierung durch den
„bugfix MTZ8302“ **entfernt** – `getRoll()`/`getPitch()` dividieren aber weiterhin durch 10
(`BNOAiOParser.h:69-70`). Wenn das BNO08x-RVC-Protokoll Pitch/Roll im selben 0,01°-Raster wie
Yaw liefert (branchenüblich für dieses Protokoll), bedeutet das **einen Faktor-10-Fehler**:
`imu_roll` wäre dann de facto um den Faktor 10 zu groß/klein, abhängig davon, was genau als
„bugfix“ korrigiert werden sollte. Das ist eine **Hypothese, die noch nicht am Signal
verifiziert ist** (dafür bräuchte es eine bekannte, für längere Zeit konstante Neigung mit
Winkelmessgerät als Referenz) – aber es erklärt ein freilaufendes, nicht an die Erdschwere
gebundenes Signal besser als ein reiner Achstausch, weil ein Achstausch die Yaw-Drehung
(Richtungswechsel der Fahrt, die im Log klar sichtbar ist, siehe `att_hdg_deg` 211°→243° in
derselben Zeit) im Rollkanal sichtbar machen würde, ein Skalierungsfehler aber nicht
zwangsläufig.

**Konkrete Aussage zur gestellten Frage:** `imu_roll` zeigt **keine** auswertbare Reaktion auf
die physische Neigung in diesem Log; es läuft stattdessen unabhängig davon weg. Ob das
Vorzeichen stimmt, Achse vertauscht ist oder die Skalierung falsch ist, kann aus diesem Log
NICHT entschieden werden, weil schon die Grundvoraussetzung (ein der Neigung folgendes Signal)
fehlt. Der Parser-Code liefert einen plausiblen Verdacht (Skalierungsfehler durch den
MTZ8302-Patch), aber keinen Beweis.

**Einordnung gemäß Operator-Vorgabe:** Da IMU und Dual-Antenne laut Betreiber vermutlich nie
gemeinsam produktiv eingesetzt werden, hat dieser Befund **keine Priorität für die
Dual-Betriebsart** – er ist nur insofern relevant, als `useImuRate` per Default aktiv ist und
(Abschnitt 0) bereits bei aktivem Dual-Empfang in den Heading-Filter eingreifen kann.

## 1b. roll_src = 2 (IMU-Roll an AOG) in diesem Log

Ja, durchgehend und in großem Umfang: `roll_src=2` liegt in weiten Teilen des Logs an
(z. B. während der gesamten Bäume-Episode 94–117s UND in der gesamten Stillstandsphase
118–135s), weil der Dual-Roll außerhalb des Envelope (`|roll|>maxRollAbsDeg=25°`) lag bzw. weil
nach `qf=0`-Lücken der Re-Entry-Settle noch lief. Der tatsächlich **publizierte** Wert
(`roll_bridge_deg`, Spalte „roll_pub“) folgt in dieser Phase nicht dem (unbrauchbaren) rohen
`imu_roll`, sondern einem LPF-geglätteten Pfad davon – sichtbar an einem deutlich trägeren
Verlauf (z. B. t=110,03–117,03s: `roll_pub` bewegt sich glatt von −118,3° auf −62,1°, während
`imu_roll` in Sprüngen von −118° auf −59° läuft). Das bedeutet: Die Firmware **glättet** das
kaputte BNO-Rollsignal, aber sie **validiert es nicht gegen den Dual-Roll**, bevor es an AOG
geht. Eine Kreuzprüfung hätte hier geholfen: Im selben Zeitraum, in dem `roll_pub` auf
Werte um −100° bis −120° läuft, meldet der Dual-Kanal (wo verfügbar, z. B. t=117,03s) nur
−2,48°. Eine einfache Plausibilitätsprüfung „IMU-Roll vs. letzter bekannter Dual-Roll,
max. Abweichung X°“ hätte diesen konkreten Fall erkannt und die Fehlerweitergabe an AOG
verhindert bzw. auf „hold“ statt „IMU-Wert“ umgeschaltet.

## 1c. Befunde B, C, D auch in diesem Log?

**B (Rückwärtsfahrt/K-Wende):** `kturn.ps1` meldet für dieses Log 0 echte Rückwärtsphasen
(„true reversal segments: 0“) – die Fahrt war durchgehend vorwärts, das Problem ist hier
nicht triggerbar. Die Modell-Gierraten-Prüfung läuft also leer; das widerlegt Befund B nicht,
bestätigt ihn aber auch nicht neu (kein Testfall in diesem Log).

**C (Heading-/Roll-Spikes der UM982):** `spikes.ps1` findet 22 Heading-Spikes in 571 gültigen
Frames (38,5 je 1000, deutlich höher als der historische Schnitt von 0,8 % = 8 je 1000 – aber
dieses Log hat auch wesentlich mehr Baumabschnitte). Größe p50 4,1°, p90 5,9°, max 17,4° –
im bekannten Rahmen. Bemerkenswert: `az_out` minus sauberer Attitude liegt in Spike-Frames bei
p50 26,4°, p90 46,6°, max 100° – deutlich schlechter als die historischen 0–5° aus Befund C.
Das deckt sich aber mit Abschnitt 0: Der dominante Fehler in diesem Log ist der IMU-Rate-
Runaway, nicht die reinen Attitude-Spikes der UM982, und die beiden Fehlerquellen überlagern
sich in der `az_out`-Messung.

**D (psi-Drift bei guter Attitude, Re-Seed):** Teilweise bestätigt, aber mit einer WICHTIGEN
Präzisierung: Der in Abschnitt 0 beschriebene Runaway (t=90–93s) ist strukturell derselbe
Fehlertyp wie Befund D (psi driftet trotz guter Attitude weg), aber die Ursache ist hier
NICHT R1 (Bias) oder R2 (Ratenlimiter-Zeitbasis) – das Radwinkelmodell war korrekt gesperrt.
Es ist ein **vierter, bisher nicht dokumentierter Pfad**: IMU-Rate-Einspeisung bei aktivem
Dual-Empfang. `azsteps.ps1` zählt für dieses Log 17 `az_out`-Sprünge >5°, 3 davon >10°, davon
nur 2 mit gleichzeitigem Rohattitude-Sprung >5° – auch das passt zum Bild „der Filter/die
IMU-Rate macht die meisten Sprünge, nicht der rohe Empfang“ (konsistent mit Befund C). Ein
Re-Seed-Ereignis trat in diesem Log nicht auf (0), d. h. der 95°-Fehler wurde NICHT per
Re-Seed korrigiert, sondern lief über die normale Modellkorrektur zurück (3–4 s Nachlauf,
t≈93,4–94s bis der Fehler wieder <5° war, dann folgte unmittelbar die Bäume-Episode).

---

## 2. Architektur (Abschnitt A)

Module, die an Heading/Position/Lenkwinkel beteiligt sind:

| Modul | Datei | Rolle |
|---|---|---|
| GNSSProcessor | `lib/aio_navigation/GNSSProcessor.*` | Parst UM982-NMEA (GGA/VTG/HPR/UNIHEADINGA), hält `GNSSData` (Position, Geschwindigkeit, dualHeading, dualRoll, headingQuality, Baseline-Länge, Sat-Zahl, HDOP) |
| HeadingBridge | `lib/aio_navigation/HeadingBridge.*` | Zentrales Filter-/Fallback-Modul: Plausibilitätsgates, 2-Zustands-Kalmanfilter (Heading + Radwinkel-Bias), Ratenlimiter, Richtungs-/Rückwärtserkennung, Roll-Kanal (Hold/Fade/IMU), Positions-Fallback/-Projektion, Baseline-Lernen, IMU-Integration, Logging (`HR,...`-Zeilen) |
| IMUProcessor | `lib/aio_navigation/IMUProcessor.*` | Erkennt BNO085 (RVC) oder TM171, liefert `IMUData{heading,roll,pitch,yawRate,quality}` |
| BNOAiOParser | `lib/BNOAiOParser/*` | Parst BNO085-RVC-Pakete (19 Byte, Prüfsumme), berechnet `yawRate` durch Differenzsummierung über 20 Pakete |
| WheelAngleFusion | `lib/aio_autosteer/WheelAngleFusion.*` | Virtueller Lenkwinkel aus Keya-Encoder, Nullpunktlernen (Zero-Offset-State-Machine), liefert `getAngleQuality()` (0–100, „wie DOP“) |
| AutosteerProcessor | `lib/aio_autosteer/AutosteerProcessor.*` | 100-Hz-Takt, ruft `headingBridge.update(dtS, steerAngleDeg)`, reicht Wheel-Quality und Steer-Telemetrie durch |
| NAVProcessor | `lib/aio_navigation/NAVProcessor.*` | Baut die an AgOpenGPS gesendete PAOGI-Nachricht, wählt `headingOut`/`rollOut` aus HeadingBridge-Status |

## 3. Datenfluss (Abschnitt B)

```
UM982 (GGA/VTG/HPR/UNIHEADINGA, 10/20 Hz) ──► GNSSProcessor ──► GNSSData
BNO085 RVC (115200 Baud, 10 Hz Pakete)     ──► BNOAiOParser ──► IMUProcessor ──► IMUData
Keya-Encoder                                ──► WheelAngleFusion ──► steerAngleDeg, Quality

AutosteerProcessor.process() [100 Hz]
   └─ headingBridge.setWheelAngleQuality(quality)
   └─ headingBridge.update(0.01f, steerAngleDeg)
        1. HPR-Gate (evaluateAttitudeFrame): QF/Zeroed/RollEnvelope/RollStep/Settle/BaseLen
        2. Prädiktion: psi += v_model * tan(scale*wheel+bias)/L * dt   ODER   psi += imurate*dt
           (IMU-Pfad ersetzt den kinematischen Pfad komplett, wenn imuRateActive)
        3. Messungs-Update (Kalman, Zustände psi + steerBiasDeg):
             Attitude (vertrauenswürdig) > COG (VTG) > fix-to-fix (Position)
        4. Ratenlimiter auf die angewandte Korrektur (allowed * dtS)
        5. Rollkanal: Dual (LPF) > IMU (fresh/valid) > Hold/Fade
        6. Zustandsmaschine LIVE/COAST/BRIDGE/TIMEOUT, Positions-Fallback/-Projektion
   └─ NAVProcessor: wählt headingOut/rollOut aus HeadingBridge::Status
   └─ PAOGI an AgOpenGPS: Dual-Heading (+Offset dort), Dual-Roll
```

**Was Position und Heading am Ende tatsächlich bestimmt:**
- Heading (`az_out` = `psi − dualHeadingOffsetDeg`): Kalman-Zustand `psi`, getrieben von
  (a) der Attitude-Messung wenn vertrauenswürdig, (b) dem kinematischen oder IMU-Rate-Modell
  dazwischen, (c) dem Ratenlimiter, (d) dem Re-Seed-Mechanismus bei >120° Abweichung.
- Roll: Dual-Roll (LPF) wenn im Envelope, sonst IMU-Roll (falls „usable“), sonst Hold/Fade auf
  0.
- Position: GNSS-Fix direkt, bei Ausfall (`posFallbackActive`) Vorwärtsprojektion aus
  `v, psi, dirSign` – diese Projektion nutzt `dirSign` aus `status.reverseDirection`, während
  die Heading-Prädiktion selbst (`psi += v*tan(...)/L*dt`, `HeadingBridge.cpp:1565` bzw. 1567)
  **ungerichtete** Geschwindigkeit verwendet (siehe Befund B – weiterhin bestätigt, hier nicht
  erneut hergeleitet).

## 4. Mathematik (Abschnitt C)

### 4.1 Zustandsvektor und Messmodell

Zustand: `x = [psi, steerBiasDeg]ᵀ`, Kovarianz `P = [[p11,p12],[p12,p22]]`.

Prädiktion (zeitkontinuierlich integriert, 100 Hz):
```
deltaEff = steerScale * wheelAngleDeg + steerBiasDeg
yawRateDegS = imuRateActive ? (imu.yawRate*invert − yawBiasDegS)
                            : 57.2958 * v_model * tan(deltaEff°) / L
psi(k+1) = psi(k) + yawRateDegS * dt            (dt = 0.01s, 100 Hz)
P(k+1) = F P(k) Fᵀ + Q      (F = [[1, dfdb],[0,1]], dfdb = Ableitung von yawRate nach bias)
```
Messupdate (skalar, H = [1,0], Kalman-Gain-Form, bei jedem akzeptierten Messwert):
```
innov = wrap180(meas − psi)
S = p11 + R²
K1 = p11/S     (Heading-Gain)
K2 = p12/S     (Bias-Gain, nur aktiv wenn estimateSteerBias || !wheelQualityOk)
psi += K1*innov  (nach Ratenlimiter-Clipping, siehe unten)
steerBiasDeg += K2*innov   (nur im „bias window“: attitudeTrusted, fixQ>=4, !dirChanging, !modelGateActive)
P ← (I − KH) P
```
Das ist ein echter 2-Zustands-linearer-Kalman-Filter mit Zustandskopplung über `p12`
(Heading-Fehler und Bias-Fehler sind korreliert, weil beide über dieselbe Messung beobachtet
werden – klassischer Fall von partieller Beobachtbarkeit: **nur die Summe** `deltaEff` ist aus
einer einzelnen Headingmessung schätzbar, Skalierung und Bias sind nur über die Zeit und
unterschiedliche Fahrsituationen (Geradeausfahrt vs. Kurve) trennbar). Das erklärt, warum ein
in der Lernphase falsch gesetzter Bias (R1) das System systematisch in die falsche Richtung
zieht, sobald das Modell wieder eingeschaltet wird: der Fehler sitzt im Zustand, nicht in
einer einzelnen Messung, und eine einzelne nachfolgende Attitude-Messung korrigiert ihn nur
mit Gain `K1<1`, nie sofort.

### 4.2 Adaptive Messrauschen R

`effectiveMeasurementR(kind)` skaliert R multiplikativ:
- schlechte Qualität (`headingQuality != 4`) → `rQualityMarginal`
- wenig Satelliten → `rQualityMarginal`
- Modellgatter aktiv (`modelGateActive`) → `× gateRFactor` (2,0)
- langsame Fahrt für COG (`v < cogRefSpeedMps=3.0`) → `× (3.0/v)`, gedeckelt bei 4,0

Das ist ein pragmatisches, aber korrektes Prinzip für ein Kalman-Filter (R wächst mit
abnehmender Messqualität), mit einer Einschränkung: Das Modellgatter wirkt NICHT auf die
Prädiktion selbst (die läuft immer mit vollem `yawRateDegS`), sondern nur auf die nachfolgende
Messkorrektur – das erklärt, warum der IMU-Rate-Runaway in Abschnitt 0 nicht durch das
Modellgatter gebremst wurde: Das Gatter vergleicht Messänderung gegen Modelländerung in einem
300-ms-Fenster, aber die Fehlerquelle WAR hier selbst die „Modelländerung“ (IMU-Rate), gegen
die sich kein Gatter wehrt, weil das Gatter das Modell als Referenz nutzt, nicht als
Fehlerkandidat.

### 4.3 Ratenlimiter

```
allowed = 57.2958 * |v| * tan(wheelUse°) / L + rateLimitMarginDegS   [deg/s]
maxStep = allowed * persistFactor * dt          (dt = lastDtS, SOLLTE 0.1s bei 10-Hz-Messung sein,
                                                  IST aber 0.01s, siehe Befund R2 – hier erneut
                                                  bestätigt: lastDtS wird am Ende von update() aus
                                                  dem 100-Hz-Aufrufintervall gesetzt, nicht aus dem
                                                  10-Hz-Messintervall)
```
`persistFactor` öffnet nach 1 s anhaltender Abweichung in dieselbe Richtung um Faktor 4
(„Schutz gegen einen echten Zustandsfehler“). Das ist konzeptionell richtig, aber die
Zeitbasis-Verwechslung (R2) bedeutet: Im Normalbetrieb ist der Limiter 10x enger als im Design
vorgesehen, und erst nach der 1-s-Öffnung wirkt er wie geplant. Das erklärt gemeinsam mit der
Betreiber-Korrektur (1–2° reichen schon für schlechtes Verhalten) eine Doppelwirkung: Normale
Attitude-Messungen werden zu langsam eingearbeitet (Trägheit, ggf. sichtbares Nachlaufen des
Headings in Kurven), WÄHREND ein echter Zustandsfehler (wie der IMU-Runaway) nach 1 s durch
die Persist-Relaxation ungebremst durchläuft – die Limiter-Logik schützt also in der Praxis
weder konsequent vor kleinem Jitter noch wirksam vor einem Runaway, der genau in der
Übergangsphase (<1 s) passiert, wie hier gesehen (die Episode dauerte nur 1,9 s bis zum
95°-Maximum).

### 4.4 Bewertung: welches Verfahren für diese Anwendung?

Kriterien: Rechenaufwand Teensy 4.1 (reichlich Reserve bei diesem simplen Modell), Robustheit,
Tuningaufwand, Verhalten ohne IMU / mit unterschiedlichen IMUs/Lenkwinkelquellen.

- **Aktueller Ansatz (handgeschriebener 2-Zustands-KF):** Rechnerisch trivial (3 Floats, keine
  Matrixbibliothek), aber mit genau den Schwächen, die die Befunde R1–R3 und der neue Befund
  aus Abschnitt 0 zeigen: Zustandskopplung macht die Bias-Freigabe zum Schlüsselparameter
  (R1), die Ratenlimiter-Logik hat einen Implementierungsfehler (R2), und es gibt keinen
  Mechanismus, der eine „Modelländerung“ selbst als fehlerhaft erkennt, wenn die Fehlerquelle
  IM Modell liegt (IMU-Rate-Sägezahn, Abschnitt 0). Für eine Anwendung mit mehreren
  möglichen Ratenquellen (Radwinkel ODER IMU) ist der aktuelle Entwurf im Kern richtig
  (Messungs-Updates aus unabhängigen Quellen, adaptive R), aber die Implementierungsdetails
  (Zeitbasis, Gate-Reihenfolge, Bias-Fenster) sind fehleranfällig, weil sie an mehreren
  Stellen im Code verteilt sind und schwer gemeinsam zu testen sind.
- **Voller EKF/UKF mit mehr Zuständen (z. B. Lenkwinkel-Skalierung als dritter Zustand):**
  Würde die Beobachtbarkeitsproblematik (4.1) nicht lösen, nur verschieben – mehr Zustände
  brauchen MEHR unterschiedliche Anregung (Geradeausfahrt UND Kurven UND Geschwindigkeits-
  variation), um trennbar zu sein. Für ein Feldfahrzeug mit viel Geradeausfahrt ist das
  unrealistisch. Rechenaufwand wäre auf dem Teensy 4.1 kein Hindernis, aber der
  Tuningaufwand (Prozessrauschen für 3+ Zustände) würde deutlich steigen, ohne das
  Kernproblem (eine fehlerhafte Rate-Quelle wird als Wahrheit behandelt) zu lösen.
- **Komplementärfilter (statt Kalman):** Würde die adaptive R-Gewichtung aufgeben – bei einem
  Feld mit wechselnder Signalqualität (Bäume, Brücken) ist adaptives R aber genau das, was
  hier gebraucht wird. Nicht empfohlen.
- **Zustandsbeobachter (deterministisch, z. B. Luenberger) statt Kalman:** Würde keinen
  Vorteil bringen, da das eigentliche Problem nicht Beobachterentwurf, sondern Quellenauswahl
  und Zeitbasis ist.

**Empfehlung:** Den 2-Zustands-KF-Ansatz BEIBEHALTEN (er ist für diese Rechenplattform und
Anwendung angemessen), aber:
1. die Quellenauswahl härten (IMU-Rate darf das kinematische Modell nicht ersetzen, sondern
   sollte NUR als zusätzliche, eigenständig geratete Messung einfließen – nicht als
   Prädiktions-Ersatz, s. Abschnitt 0/E),
2. die Zeitbasis des Ratenlimiters korrigieren (R2),
3. die Bias-Freigabebedingungen schärfen (R1),
4. die Baseline-Referenzlernphase robuster machen (R3).

## 5. Fehleranalyse (Abschnitt D) – nach Evidenzgrad sortiert

**Im Code nachgewiesen (nicht interpretiert, direkt aus Quelltext ablesbar):**
- R1: Bias-Lernen/-Freigabe bei `wq`-Übergang (`HeadingBridge.cpp:1035-1046`).
- R2: `lastDtS` wird aus dem 100-Hz-Aufrufintervall gesetzt, nicht aus dem 10-Hz-Messintervall
  (`HeadingBridge.cpp:1272`, verwendet in `990`/`1003`).
- B: `psi`-Prädiktion nutzt ungerichtete Geschwindigkeit (`HeadingBridge.cpp:1565/1567`),
  während die Positionsprojektion `dirSign` nutzt (`HeadingBridge.cpp:2395`).
- NEU (Abschnitt 0): IMU-Rate ersetzt bei `imuRateActive` die kinematische Prädiktion
  komplett (`HeadingBridge.cpp:1563-1567`), ohne eigene Plausibilitätsprüfung gegen die
  kinematisch erwartete Rate (nur eine Bias-Lernschleife mit ±30°/s-Clamp als einzige
  Schutzmaßnahme, `1519-1521`).
- BNOAiOParser: Skalierungsdifferenz Yaw (`*0.1f`) vs. Pitch/Roll (keine Skalierung, Kommentar
  „bugfix MTZ8302“) bei identischem `/10.0f` in den Gettern (`BNOAiOParser.cpp:144-151`,
  `BNOAiOParser.h:69-70`).

**Durch Logdaten belegt:**
- R3: Baseline-Referenz-Problematik (in diesem neuen Log nicht erneut quantifiziert, aber die
  hier gelernte Referenz 0,67-0,72 m liegt im „0 Ablehnungen“-Band aus der ursprünglichen
  Analyse).
- C: Spikes auch in diesem Log (22 in 571 Frames), aber mit ungewöhnlich hohem `az_out`-Fehler
  in den Spike-Frames (p50 26°) – das ist NICHT die übliche Spike-Signatur, sondern
  Überlagerung mit dem IMU-Rate-Runaway (Abschnitt 0).
- Abschnitt 0 (IMU-Rate-Runaway ohne Bäume/GNSS-Fehler): vollständig aus Logdaten belegt
  (qf, uni_hdgsd, baseline_m, wq, imu_flags, imurate, yawbias, psi – alle in Abschnitt 0
  zitiert).
- Neigetest: `imu_roll` zeigt keine Korrelation zur realen Neigung, läuft stattdessen
  monoton weg (Abschnitt 1a).

**Plausible Hypothese (nicht bewiesen):**
- Der Skalierungsfehler in `BNOAiOParser.cpp` als ERKLÄRUNG für die Pitch/Roll-Drift – ohne
  eine kontrollierte Referenzmessung (bekannter, konstanter Neigungswinkel über Zeit) nicht
  zu verifizieren, da auch das rohe BNO085-Ausgangssignal selbst (ohne GPS-Anker) für Pitch/
  Roll in RVC typischerweise schon eine gewisse Drift zeigt (keine Magnetometer-Stützung für
  Rollwinkel wie bei Yaw).

**Ungeklärt:**
- Ob `imuRateInvert`/Achszuordnung für `imu_roll` korrekt ist, kann aus den vorliegenden Logs
  nicht entschieden werden (siehe 1a).
- Aufgabe 2 (Segmentierung Brücke/kurzer GNSS-Ausfall über 6 Logs) wurde auf Wunsch des
  Betreibers VERSCHOBEN zugunsten der Analyse des heutigen Logs; die zugehörige
  Quantifizierung (Zeit bis Projektionsumschaltung, Positions-/Heading-Sprung bei Rückkehr,
  proj_ctk/proj_along/proj_rng) steht noch aus.

## 6. Verbesserungsvorschläge (Abschnitt E) – noch NICHT umgesetzt, zur Freigabe

| # | Vorschlag | Löst | Nutzt welche Daten | Modell | Nebenwirkungen | Verifikation |
|---|---|---|---|---|---|---|
| E1 | IMU-Rate NIE als Ersatz der kinematischen Prädiktion verwenden, sondern (a) nur als zusätzliche geratete Messung mit eigenem R, oder (b) komplett deaktivieren solange Dual-Heading verfügbar ist (`attitudeTrusted`) | Abschnitt-0-Runaway (95°), künftige kleinere IMU-Sägezahn-Jitter | `imu_flags`, `attitudeTrusted`, `imurate` | Prädiktion bleibt `v*tan(δ)/L`; IMU-Rate wird zu einer dritten Messquelle mit `effectiveMeasurementR(kind=IMU)` statt Prädiktionsersatz | Bei einem echten Radwinkel-Totalausfall UND aktivem Dual-Heading wäre die IMU-Rate als Fallback dann auch nicht mehr verfügbar – aber Dual-Heading selbst deckt diesen Fall bereits ab | Offline-Replay des 11-27-Logs mit „IMU-Rate aus, solange `attitudeTrusted`“ – Erfolg: `|psi−att|` bleibt <5° in t=90-93s |
| E2 | Bias-Freigabe (R1): Bias erst NACH einer Mindestzeit stabiler `wq>=high` UND kleinem `|bias|` freigeben, oder beim Übergang auf 0 zurücksetzen | R1 | `wq`, `bias_deg`, Übergangszeitpunkt | kein Modelländerung, nur Freigabebedingung | Verlangsamt Zeit bis Modell nutzbar ist (Operator-Priorität: schnelles Finden des VWAS-Nullpunkts beachten – NICHT verschlechtern) | `bias_runaway.ps1` auf allen 15 Referenzlogs, Erfolgskriterium: max|psi-att| nach Freigabe <20° in allen Logs |
| E3 | Ratenlimiter-Zeitbasis (R2): `lastDtS` aus dem tatsächlichen HPR-Intervall statt dem 100-Hz-Aufrufintervall setzen | R2 | Messintervall der Attitude (bereits in `frameIntervalEmaMs` vorhanden) | `maxStep = allowed * persistFactor * frameIntervalEmaMs` statt `lastDtS` | Limiter öffnet 10x weiter – mehr kurzfristige Korrektur erlaubt, ggf. mehr sichtbarer Jitter bei Spikes, die bisher durch die enge Grenze zufällig gedämpft wurden | `runaway.ps1` + `filter_compare.ps1` Seite an Seite auf denselben Logs, Vergleich Sprungzahl/Amplitude |
| E4 | Gate-Referenz/Settling (R3): erste ~100 Lernframes der Baseline-Referenz zusätzlich auf `qf=4 & uni_hdgsd<1°` filtern | R3 | `uni_hdgsd`, `qf` während der Lernphase | kein Modelländerung, nur Lernfilter | Lernphase dauert ggf. länger (mehr Frames nötig, falls die ersten Sekunden unsauber sind) | `baseref.ps1` auf allen Logs, Erfolgskriterium: 0 Ablehnungen wegen `baseLen` bei sauberer Fahrt |
| E5 | Fahrtrichtung im Modell (B): `psi`-Prädiktion mit `dirSign` versehen wie die Positionsprojektion | B | `status.reverseDirection` | `psi += dirSign * v*tan(δ)/L*dt` | Erfordert sorgfältige Prüfung der Zirkelbezug-Regel (Lenkwinkel-Nullpunkt lernt über Encoder+GPS, nicht über psi) – muss gegen `reverse_sign_check.ps1` laufen | `kturn.ps1` auf den 9 bekannten Rückwärtsphasen, Ziel: Vorzeichen-Trefferquote ≥90 % (siehe Aufgabe 6c) |
| E6 | IMU-Roll-Absicherung (E/1b): Kreuzprüfung `imu_roll` gegen letzten bekannten Dual-Roll (max. Abweichung X°) vor Publikation | 1b | `imu_roll`, letzter `roll_deg` bei `attitudeTrusted` | Zusätzliches Gate vor `roll_src=2` | Könnte bei echtem, großem Neigungswechsel (Hang) den IMU-Roll fälschlich verwerfen, wenn kein aktueller Dual-Anker existiert | `rollout.ps1` + neue Vergleichsmetrik auf allen IMU-Logs |
| E7 | BNOAiOParser-Skalierung: Pitch/Roll-Skalierung mit Referenzmessung verifizieren, ggf. `*0.1f` wiederherstellen | 1a (Hypothese) | Neigetest mit bekanntem Referenzwinkel | reine Parser-Korrektur | Bricht ggf. mit dem `MTZ8302`-Fix verbundene, bisher unbekannte andere Korrektur – braucht Git-History/Kontext zu diesem Commit, der hier NICHT vorlag | Kontrollierter Neigetest mit Winkelmesser, Vergleich `imu_roll` vs. bekannter Winkel vor/nach Änderung |
| E8 | GNSS-Ausfall/kontrollierte Rückkehr (2/3) | noch zu quantifizieren | Segmentierungsskript (noch zu schreiben) | — | — | — (siehe Abschnitt 5 ungeklärt) |

---

## 7. Offene Fragen an den Betreiber

1. **E7 (BNOAiOParser-Skalierung):** Ich habe keinen Zugriff auf die Commit-Historie/den
   Kontext des „bugfix MTZ8302 2026-05-07“-Kommentars. Welches konkrete Symptom sollte dieser
   Fix beheben? Falls bekannt, lässt sich daraus ableiten, ob die fehlende `*0.1f`-Skalierung
   beabsichtigt war (z. B. weil Pitch/Roll im RVC-Paket tatsächlich in einem anderen Raster als
   Yaw kodiert sind) oder ein Versehen ist.
2. **Aufgabe 2 (Segmentierung Brücke/GNSS-Ausfall):** Auf Wunsch zurückgestellt – soll ich das
   Segmentierungsskript jetzt nachliefern (6 Logs: 17-46, 20-15, 20-11, 20-06, 17-41,
   2026-09-30 6-41), oder bleibt das vorerst zurückgestellt?
3. **AgOpenGPS-Quellcode-Abschnitt (Aufgabe 3):** Bereits vom Betreiber direkt beantwortet
   (AOG liefert keine Werte zurück, verwendet Dual-Heading direkt, nur Versatz wird
   verrechnet) – dieser Teil ist damit erledigt und braucht keine Code-Lektüre mehr.
4. **E1 (IMU-Rate deaktivieren solange Dual verfügbar):** Soll das eine KONFIGURIERBARE Option
   werden (z. B. `useImuRate` bleibt, aber zusätzlich `imuRateOnlyWithoutDual=true` als neuer
   Schalter), oder soll `imuRateActive` grundsätzlich an `!attitudeTrusted` gekoppelt werden
   (hartcodiert)? Das betrifft auch zukünftige Single-Antenna-Betriebsart, wo die IMU-Rate laut
   Konzept-Dokument explizit gewünscht ist.

Rückfragen und Korrekturen ausdrücklich erwünscht – insbesondere zu E7, da mir hierzu der
Code-Kontext (Commit-Historie) fehlt.
