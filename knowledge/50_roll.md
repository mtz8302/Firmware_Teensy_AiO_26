# 50 Roll (R)

**Version 1.0.0 · 2026-10-11 · gilt fuer Logs 2026-09-07 bis 2026-10-11**
Aufgeteilt aus dem frueheren Register `WISSENSSTAND.md` (10-11, danach geloescht). **Die B-Nummern sind global und unveraendert** -
sie bleiben in allen Verweisen, Commits und Code-Kommentaren gueltig. Zugeordnet nach
HAUPThema der Zeile (`<!-- aus ... -->` nennt den Ursprungsabschnitt); Querthemen stehen
im Text. Einstiege: `00_ueberblick.md` (2 Minuten) und `symptome.md` (je Betreiberbericht).

| # | Befund | Beleg / Begruendung |
|---|---|---|
<!-- aus 1. Fundament (Wahrheiten) -->
| 1.13 | rollLpfTauS 0,8 s war die eigentliche Latenz; die Ausgangsstufe ist phasenneutral (0...100 ms) | 10-10 §2.10 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.1 | **Deshalb existiert das Modul** (09-21): der UM982 **schweigt nicht**, wenn die Dual-Lage degradiert - er sendet weiter 10 Hz `$GNHPR` mit **komplett genullter Lage** (`heading=000.0000, roll=000.0000, QF=0, sat=0`), waehrend GGA weiter **RTK fixed** meldet | 09-21 Testfahrten, Beispielsatz im Kommentar |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.11 | **Roll-Envelope vom Heading ENTKOPPELT** (09-30, Betreiber-Wunsch "P1a"): der Roll ist eine Ausgabe, das Heading desselben Satzes eine **eigene Loesung** - Roll darf es nicht allein verwerfen | Baum-Log 09-30, 30 Roll-Envelope-Laeufe: **13** mit passendem Heading (Abweichung 0,3...9 Grad; einer 76 Frames, \|roll\| 66 Grad, 26 Sats, 0,4 Grad Abweichung = **gute Lage weggeworfen**), **17** waren 12...135 Grad daneben |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.24 | **Die Lage muss sich nach dem Wieder-Einrasten erst SETZEN** (09-23/24): in den ersten 0,5...2 s nach einem Re-Lock war die Loesung noch nicht da | gemessen in den Hallen-Logs: **Roll 10...19 Grad bei GERADEAUSFAHRT**; diese Frames passierten den Roll-Envelope als "none" und schoben psi herum. Betreiber-Wunsch: **1 s** Setzzeit |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.40 | **Roll-Kanal**: AOG macht daraus einen Positionsversatz `sin(roll)*Antennenhoehe` | 09-23: Dual-Roll springt **p50 0,76 / p90 7,2 Grad je 100 ms**; bei 3 m = **0,37 m** (springender Traktor). 0,35 s LPF liess ~30 % jedes Sprungs durch, **0,8 s ~12 % (0,06 m)** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.41 | **Roll HALTEN bei Lageausfall** (Betreiber 10-06): Mittelwert der letzten GUTEN Meter (distanzgewichtet, nur validierte Epochen); Fade erst nach 20 m ODER 45 s | Betreiber: der Roll ist bei schlechtem Empfang der **ERSTE**, der falsch wird, und beide Verbraucher brauchen ihn (AOG-Hangkompensation + unser k*roll) |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.42 | **IMU-Roll LIVE** (Betreiber 10-06: *"die IMU braucht eigentlich keinen Roll-Filter, die stimmt eigentlich immer"*): plausibel (Abweichung 2...3 Grad in den 10-04-Logs) -> live und ungefiltert (kein Halten, kein Faden); dauerhaft abweichend (**68...177 Grad** gemessen, Achse/Einheiten verdreht) -> unbrauchbar markiert | Betreiber-Entscheidung 10-06 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.44 | **LIVE-Gate** (Betreiber-Entscheidung 10-05): solange der Empfaenger nachweislich sauber ist, wird sein EIGENES Azimut **live durchgereicht** (wie beim Kaltstart) statt den Filterzustand psi zu senden | Begruendung aus den 10-05-Logs: auf der Fahrt mit **9,6 Grad Schraeglage** war der Empfaenger die ganze Zeit sauber (`uni_hdgsd` p50 **0,43** / p90 0,70 Grad, `uni_ptchsd` p50 0,83, \|baseline_dev\| p50 0,9 %, \|roll\| <=2,4 Grad) -> **das Gate waere 95,1 % der Geraden-Frames offen**, und der Empfaengerwert IST dort die Wahrheit (`att - cog` stabil **-88,2 Grad**), waehrend psi den 9,6-Grad-Versatz stehenlaesst |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.46 | **P1-Transparenz** (Betreiber-Entscheidungen 09-30): `rollSuspect` = der Frame wurde akzeptiert, obwohl \|roll\| ausserhalb des Envelopes lag, weil eine unabhaengige Quelle die Richtung bestaetigte (der Roll-Ausgang haelt dann, `rollSource` wird 2/3); `yawRateZeroed` = die kinematische Gierrate wurde NICHT benutzt, weil der Lenkwinkel unzuverlaessig ist -> psi haelt/folgt den Messungen statt einer erfundenen Drehung | Spalten im HR-Log, Werkzeuge `_recon2.ps1`/`_l1010.ps1` |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.68 | **Sidehill/Draft im MODELL** (10-06, Betreiber: *"wir nutzen den gleichen Hebel anders herum"*): AOG addiert fuer die Hangdrift Lenkwinkel auf den SOLL-Winkel; dieser Anteil erzeugt KEINE Gierrate (er wird im Reifenschlupf verbraucht) - fuer `omega = v*tan(delta)/L` muss er also heraus, **aber NUR hier**: der nach aussen gemeldete Radwinkel (PAOGI/Anzeige) bleibt unveraendert. Verwendet wird der GEBRIDGTE Roll (mit Halten/Fade), also derselbe, den VWAS und AOG sehen -> eine Konvention fuer alle | Betreiber-Entscheidung 10-06 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.72 | **Spalte `roll_bridge`** (09-24): die ROHE Dual-Roll gegen das, was das VWAS wirklich konsumiert (WheelAngleFusion nimmt `rollDeg` vom Bridge) - beide muessen im Log stehen, um einen rohen Lage-Fehler von einem nur vom Bridge gelieferten Roll zu unterscheiden | Betreiber-Wunsch |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.76 | **`psi_imu` + die `imu_flags`-Bits** (09-29): `psi_imu` ist dieselbe Durchreich-Probe fuer die IMU; bit0 = ok/frisch, bit1 = Heading speiste den Filter, **bit2 = Rate treibt das Modell**, bit3 = dieser IMU-Typ hat keine Rate (TM171). `imu_hdg_raw` ist das, was AOG in PANDA bekommt; `vwas_hsrc` (0 Bridge / 1 Dual / 2 COG) sagt, welche Quelle das VWAS benutzte; `roll_src` (0 none / 1 dual / 2 IMU / 3 gehalten); `cog_ok` | Spaltenkommentare; Werkzeug `imu_presence.ps1` |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.80 | **Setzen nach einer Empfangsluecke** (Betreiber-Wunsch 09-24): der Empfaenger braucht nach dem Re-Lock einen Moment, bis seine Baseline-Loesung eine Loesung ist - **und diese Frames passierten JEDES Tor** (Roll-Envelope 25 Grad, Roll-Step nur grosse Spruenge) | gemessen in den Hallen-Logs: die ersten **0,5...2 s trugen Roll 10...19 Grad bei GERADEAUSFAHRT**, das Heading wanderte mit. Sie werden deshalb **beobachtet, aber nicht BENUTZT**, bis die Sequenz `reentrySettleMs` lang konsistent war (\|roll\| klein, Roll und Azimut stabil) |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.84 | **P1a (09-30)**: der Roll-Zustand DIESES Frames wird gemeldet, akzeptiert oder nicht - damit der Log zeigt, wann eine Richtung gehalten wurde, obwohl der Roll unbrauchbar war | Betreiber-Wunsch |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.105 | **Die IMU-Roll-Huelle wurde am 10-05 eingebaut und am SELBEN TAG wieder zurueckgenommen** (siehe B9.2): sie beruhte auf der falschen Annahme, die IMU-Roll komme 10x zu gross an | "Sie tut es nicht" - eine Huelle wuerde KORREKTE Werte ueber 25 Grad verwerfen; die Dual-Roll-Huelle bleibt unangetastet |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.106 | **Roll-Qualitaet (10-05)**: der Dual-Roll wurde bisher nur ueber "Lage vertraut && \|roll\| <= Huelle" angenommen, ohne Blick darauf, ob DIESE Epoch praezise ist. `uniPtchStdDevDeg` ist das eigene 1-sigma des Baseline-**Pitch = ROLL** (die Baseline liegt quer) | gemessen **0,86...0,93 Grad bei gutem Empfang und 159 Grad bei schlechtem**; ein 159-Grad-sigma-Frame kann 30...60 Grad danebenliegen und schiebt ueber den LPF einen Roll-Bias in AOG (`sin(roll)*2,2 m` Antennenhoehe) |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.107 | **IMU-Roll-Plausibilitaet (10-06)**: nur wenn der Dual-Roll in DIESER Epoch sauber ist, ist er die Wahrheit - dann wird die IMU daran gemessen. Bleibt die Abweichung klein, ist die IMU ein vollwertiger, empfaengsunabhaengiger Roll und darf live gehen | Hysterese **1x / 1,6x** der Grenze |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.108 | **Ein Roll-Re-Init darf nicht HART setzen** (Betreiber-Hinweis 10-05): AOG rechnet Roll in einen Positions-Querversatz um (`sin(roll)*Antennenhoehe`) | gemessen im Log 6-19: der GESENDETE Roll springt mit **p99 13,2 Grad und max 25,0 Grad pro 100 ms**, obwohl \|roll\| selbst <=4,2 Grad bleibt - das sind genau diese Re-Seeds, und sie ergeben **0,4...2,1 m AOG-Positionssprung** (bei H = 1...5 m). Nur der ALLERERSTE Init (direkt nach dem Boot, da faehrt nichts) setzt hart |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.109 | **Die "guten Meter"-Pflege gibt es auch im DUAL-Zweig** (10-06): ohne sie bleibt `rollGoodAvgDeg` ungefuellt und das Roll-Halten hat im Dual-Fall (Hauptpfad der Maschine) nichts zu halten | gemessen: **Log 20-41 hatte kein einziges `roll_src = 4`, obwohl 307 Epochen gefadet wurden** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.110 | **Single-Antennen-Roll-Rueckfall** (Betreiber-Entscheidung 09-29): ohne Dual-Antenne hielt der Rollkanal und fadete dann auf **NULL** - das kostete dem VWAS sein **roll-gebinnetes Nullpunkt-Lernen** (WheelAngleFusion binnt die Nullpunkt-Beobachtungen nach Roll) und liess AOG ohne Roll. Die IMU-Roll ist eine Schwerkraft-Referenz (kein Magnetometer) und damit der richtige Ersatz | Achsen/Montage werden im IMUProcessor angewandt - **nur BNO, der TM171 bleibt absichtlich unangetastet** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.117 | **PATH/COG als "letzter absoluter Anker"** (Betreiber-Entscheidung 09-29): ohne Dual-Antenne ist das die einzige Wahrheit, an die das IMU-Heading gebunden werden darf (dieselbe Rolle, die AOG ihrem GPS-Heading gibt) - **der IMU-Offset darf daher auch ueber eine akzeptierte PATH/COG-Messung gelernt werden** | Betreiber-Entscheidung |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.130 | **`fixQuality` ist KEIN Kriterium mehr** (Betreiber 10-05: *"gute Daten koennen auch float oder nur GPS sein, denn gutes Heading und Roll kann auch ohne RTK klappen"* = relative Geometrie) | Betreiber-Entscheidung |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.1 | **Die drei RVC-Felder teilen sich KEINE Einheit**: `yaw` = 0,01 Grad (hier mit 0,1 skaliert -> `yawX10` ist in 0,1-Grad), `pitch` = 0,1 Grad (**ROH** nach `pitchX10`), `roll` = 0,1 Grad (**ROH** nach `rollX10`). `getYaw/getPitch/getRoll` liefern also **Grad**, und alles stromabwaerts auch (`IMUProcessor::currentData.roll/pitch`, `HeadingBridge::Status.imuRollDeg`, der Rollkanal) | `BNOAiOParser.cpp:143ff` (geprueft 10-05) |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.2 | **"DO NOT FIX THIS AGAIN"**: `pitchX10 = rawPitch; rollX10 = rawRoll;` (Bugfix MTZ8302, 2026-05-07). Mit dem ROHWERT sind Roll und Pitch, die AgOpenGPS erreichen, **RICHTIG** - der Roll wurde in AOG wiederholt auf der F9P-Maschine bestaetigt (Board fest montiert, Achsen fast sauber ausgerichtet, Montageversatz <=5 Grad) | `BNOAiOParser.cpp:163ff`. Frueheres "Reparieren" scheiterte an **"impossible"-Pruefungen** (ein unkonvergiertes ERSTES Paket sieht genau so aus) und an einem Gelände-Grenzwert ("die Strecke hat <=5 Grad Neigung"), der aus einer Spalte gemessen wurde, die nicht dieser Winkel sein muss (**Achsenkonfiguration!**). Deshalb war die IMU-Roll-Huelle am 10-05 (B8.105) zu Recht wieder weg |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.16 | **Rohe IMU-Daten ~1 Hz ins Log** (Betreiber-Entscheidung 10-05): der Roll ist nur im STAND vertrauenswuerdig (gemessen **3...5 Grad im Stand gegen bis zu 79 Grad bei Fahrt auf demselben ebenen Grund**), und "schwingt der PITCH mit" (Vibration/Fusion) gegen "nur der Roll" (Achse/Montage) laesst sich nur an Rohwerten entscheiden. **Nicht in den Ring** - Sensor-Diagnose, keine Groesse je Fix | `IMUProcessor.cpp:288` (Spannung zur Korrektur in §0 jener Datei) |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.17 | **`get*X10` liefert die gespeicherte 0,1-Grad-Darstellung**, `getYaw/getPitch/getRoll` liefern GRAD; `swapXY` sagt, ob die Firmware die zwei Neigungsachsen tauscht - **Betreiber 10-05: die F9P-Maschine nutzt "x", der Tausch ist AUS, `currentData.roll` IST dieser Roll** | `BNOAiOParser.cpp:194` |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.18 | **Im PANDA-Zweig gehoert KEIN Dual-Roll-Rueckfall hin** (waere toter Code): `selectMessageType()` liefert PAOGI, sobald `hasDualHeading` gilt, und PAOGI nimmt den Roll ohnehin aus `gnssData.dualRoll` -> der PANDA-Zweig laeuft nur OHNE Dual-Heading. **Der IMU-Ausfall der 09-21-Logs betrifft den Roll, der AOG erreicht, also NICHT** | `NAVProcessor.cpp:163` |


## Messung 2026-11-09 - der Roll schwingt auch, wenn das Fahrzeug STEHT

Acht Logs (3x Knicklenker, 5x Fendt). Betreiber: ausdruecklich OHNE Wellen im PWM des Autosteer nachgeprueft - siehe B11.13.

| # | Befund | Beleg |
|---|---|---|
| B11.11 | **Der Roll schwingt IM STAND, also ist er kein Fahrwerkseffekt** | bei `v < 0,05 m/s` auf ebener Flaeche: `abs(roll_bridge)` max 3,8...7,0 Grad, sd 0,8...2,0 in 6 von 8 Logs (Fahrt bei v > 1 m/s: max 6,3...15,4, sd 1,1...5,0 - dieselbe Groessenordnung). Ein stehender Traktor kann nicht wanken -> die Welle ist ein **Artefakt der Dual-Loesung**, nicht die Strasse |
| B11.12 | **Groesse in Metern / Betreiber-Bestaetigung** | Ausgabe-Spanne -6,5...+7,2 Grad = bis **47,7 cm** Positionsversatz bei 2 m Antenne; an der Stelle des Betreibers ("roll -4 bis +4") sind es `sin(4 Grad) * 2 m` = **14 cm** - seine 15 cm sind damit erklaert |
| B11.13 | **Die Welle kommt NICHT von der Lenkung** (Betreiber-Hypothese widerlegt) | Roll-Streuung bei stehender Lenkung (`abs(pwm) < 3`) sd 2,14 Grad gegen 2,33 Grad bei aktiver (`abs(pwm) > 15`); Kreuzkorrelation Roll gegen `pwm` max r = 0,16 (Lag -0,8 s), gegen `aog_tgt` 0,32 ohne klaren Lag (alle Lags 0,21...0,32 = gemeinsamer Streckentrend). Periode der Welle p50 4,8 s (p10 1,2 s) |
| B11.14 | **Der Roh-Roll ist unbrauchbar, die Ausgabe ist nur gedeckelt** | roh -66,1...+50,8 Grad; ausgegeben max **1,000 Grad/Frame** (harter Deckel, 10 Hz = 10 Grad/s), Deckel greift in **5...10 %** der Fahreframes -> der interne Roll will staendig schneller als er darf. `roll_src` 1 (dual) 82 %, 4 (gehalten) 18 % |

**Folge fuer die Ausgabe:** AOG rechnet `sin(roll) * Antennenhoehe` als Querversatz. Ein
Roll-Artefakt von +-4 Grad bei 2 m ist ein **14-cm-Positionszittern**, das AOG als Querfehler
sieht - unabhaengig von der Heading-Kette. Der Roll ist damit nach dem Heading der **zweite**
Eintrag in die unruhige Lenkung (in der Ursachenzuordnung der 11-09-Logs fielen 0...26 % der
Lenk-Zappel-Frames mit einem Roll-Sprung zusammen, 43...95 % mit einem Heading-Sprung).
### Nachtrag 2026-11-09: die URSACHE des Roll-Zitterns (B11.15)

| # | Befund | Beleg |
|---|---|---|
| B11.15a | **Der Roll kommt aus dem Baseline-PITCH** der quer montierten Dual-Antenne | `GNSSProcessor.cpp:1914-1922` (Kommentar: "THIS IS THE VEHICLE ROLL ... Do not fix this to read field 5") |
| B11.15b | **Die Roll-Messung bricht bei LANGSAMER FAHRT zusammen** | dual-Roll-Streuung nach Tempo (PP-Log): Stand 1,83 Grad, 0,05...0,5 m/s **10,33**, 0,5...1 m/s 7,46, 1...2 m/s **12,23**, ueber 2 m/s **1,64**. Parallel `uni_ptchsd` p90 = 26,8 / 214,9 / 3,76 / 1,26 Grad - der Empfaenger SAGT es also selbst |
| B11.15c | **Die Baseline-Laenge ist der Zeuge** | `uni_len` springt 0,25 ... 8,26 m (p50 2,76) statt konstant zu sein; `baseline_dev_pct` ist im Log **leer** (NaN, n = 0) - die Spalte traegt nichts mehr |
| B11.15d | **... und die vorhandene Qualitaetspruefung steuert nur den HALTEWERT, nicht die Ausgabe** | `HeadingBridge.cpp:1883-1896`: `rollQualOk` aus `uniPtchStdDevDeg <= 3,0` (Hysterese 6,0) und `baselineDevPct <= 5 %`. Kommentar `:1902`: **"die AUSGABE bleibt live, nur der Haltewert wird gereinigt"**. Ein Frame mit 215 Grad Pitch-sigma laeuft damit live durch den LPF |
| B11.15e | **Keine IMU als Ersatz im Fahrzeug** | `imu_roll` ist im ganzen Log **exakt konstant** (sd 0,00 im Stand UND in Fahrt) -> kein IMU-Roll vorhanden, B8.42/B8.110 greifen hier nicht |

**Konsequenz (noch NICHT gebaut):** die Roll-Qualitaet (`uniPtchStdDevDeg`, `uni_len`) muss die
AUSGABE steuern - bei schlechter Epoche halten bzw. die Rampen-Rueckkehr nutzen -, nicht nur den
Haltewert. Das ist eine Aenderung an der Ursache; "im Stand halten" waere nur ein Symptom.