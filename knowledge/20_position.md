# 20 Position & Kurs (P)

**Version 1.0.0 · 2026-10-11 · gilt fuer Logs 2026-09-07 bis 2026-10-11**
Aufgeteilt aus dem frueheren Register `WISSENSSTAND.md` (10-11, danach geloescht). **Die B-Nummern sind global und unveraendert** -
sie bleiben in allen Verweisen, Commits und Code-Kommentaren gueltig. Zugeordnet nach
HAUPThema der Zeile (`<!-- aus ... -->` nennt den Ursprungsabschnitt); Querthemen stehen
im Text. Einstiege: `00_ueberblick.md` (2 Minuten) und `symptome.md` (je Betreiberbericht).

| # | Befund | Beleg / Begruendung |
|---|---|---|
<!-- aus 1. Fundament (Wahrheiten) -->
| 1.3 | Positionskurs ist RELATIV RTK-unabhaengig: Querabweichung p50 0,5 cm / p90 1,4...1,8 cm / RMS 2,0...3,7 cm, mit und ohne RTK gleich (abs. Sigma ohne RTK 1,2 m) | recovery (10-05) |
<!-- aus 1. Fundament (Wahrheiten) -->
| 1.4 | Positionskurs ueber 5 m: 0,10...1,11 Grad gegen Modell 13,9...29 Grad (Luecken 11...29 s) | 10-10 §2.1 |
<!-- aus 1. Fundament (Wahrheiten) -->
| 1.5 | Eigene publizierte Projektion als Kursquelle: 5,22 statt 0,10 Grad => nie verwenden | 10-10 §2.2 |
<!-- aus 1. Fundament (Wahrheiten) -->
| 1.8 | Empfaenger-Trust-Kriterien untauglich: att_q / uni_hdgsd / Baseline lassen 24...75 % der >2-Grad-falschen Epoch durch (Fehlablehnung nur 0,2...4,7 %) => Trust gegen den Kurs messen | 10-10 §2.5 |
<!-- aus 2. Mechanismen (das Herzstueck) -->
| 2.4 | Die Projektion IST die bessere Quelle (projqc), erreichte psi nur nicht => Ringpuffer + Steps A...D | recovery §6/§7 |
<!-- aus 2. Mechanismen (das Herzstueck) -->
| 2.6 | 90-Grad-Fall 10-09: Lage drehte 176 Grad, publiziert -95 Grad, out_lim = 0 => die QUELLE fiel auf psi zurueck | 10-10 §1 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.25 | **Positions-Rueckfall** (Betreiber-Wunsch 09-24): die Vorwaertsprojektion integriert auch die POSITION und wird an AOG gegeben, wenn der Fix unbrauchbar ist - damit Bruecke/Baum den Querversatz nicht wegwerfen. Das Fix-Tor verwirft eine physikalisch unmoegliche Position (`ds > posSpikeMargin*v*dt + posSpikeNoiseM`), alles unter RTK fixed und veraltete Fixes; die Projektion laeuft weiter, statt darauf zu springen; veroeffentlicht wird sie nur begrenzt (`posFallbackMaxMs`/`posFallbackMaxM`) | Betreiber-Wunsch 09-24 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.26 | **RTK-Erholung** (Betreiber-Hinweis 09-24: *"die Zeit, die das GPS braucht, um sich zu erholen, ist das Problem"*): eine Bruecke ist nur ~10 m, der Empfaenger braucht aber Sekunden fuer die Ambiguitaet. Sobald fixQuality 4 zurueck ist UND so lange gehalten hat, IST diese Position die Wahrheit - die Projektion wird darauf **neu synchronisiert, auch wenn sie von unserer Extrapolation weggesprungen ist** | "Ohne das wuerde der Rueckfall seine eigene Schaetzung gegen einen erholten Fix verteidigen, bis das Budget aufgebraucht ist" |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.27 | **Positions-Rampe** (Betreiber-Wunsch 09-24): wenn der Rueckfall die Position zurueckgibt (RTK erholt oder Budget verbraucht), wird die extrapolierte Position ueber diese Zeit auf den Fix geblendet statt zu springen - AOG sieht einen stetigen Querversatz (95 % der Strecke in dieser Zeit) | Betreiber-Wunsch |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.28 | **Die 0,5-m-Schranke fuer die Projektion** (verschaerft 09-24): weil die Projektion bei JEDEM Fix neu synchronisiert wird, kann sie nach 100 ms nur wenige cm von der Wahrheit weg sein (bei 5 m/s sind es 0,5 m). **Eine Abweichung ueber ~0,5 m ist also schon ein schlechter Fix** | Die alten `3*v*dt + 0,5 m` erlaubten bei 5 m/s **2 m** und liessen die Bruecken-Spikes des 06:20-Logs unangetastet durch |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.29 | **Betreiber-Korrektur 09-29** (*"der Traktor springt auf der Karte"*, Bruecken-Log 06:34: **4 Fixes von 1,4...2,9 m erreichten AOG**): die veroeffentlichte Position ist die des EMPFAENGERS, solange kein Rueckfall laeuft - ein akzeptierter, aber falscher Fix ist also 1:1 sichtbar. Das obige Tor entscheidet nur "Rueckfall oder akzeptieren", es begrenzt keinen akzeptierten Schritt | Zusatz: die Projektion wird auch als **Aufholer** benutzt (Rest groesser als in `posRampMs` schaffbar -> laengere Rampe bis `posRampMaxMs`), damit AOG eine stetige Spur sieht |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.37 | **Ein Knopf, eine Entscheidung** (10-03): `baselineTolPct` gilt fuer Verwerfen UND Baseline-Test; `uniBaseTolPct = 5,0` wurde zusammengelegt | ueber der Toleranz ist die Messung nicht vertrauenswuerdig: sie darf **weder verworfen noch springen** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.45 | **Harter Rampen-Start an Wiedereintritten** (gemessen 10-05 abends): **an den Wiedereintritten reichen die Qualitaetszahlen NICHT** | gemessen meldete der Empfaenger dort **sd 0,4...0,8 Grad, Baseline-Abweichung ~0 %, qf = 4 - und das Heading lag trotzdem 5...15 Grad (extrem 70...100 Grad) neben der Projektion**. Daher: Start erst nach `reentryGoodFrames` guten Epochen UND Einigkeit mit der Projektion (travel-Witness), dazu Rampe. **Ebenso gemessen: die Haelfte der guten Phasen ist kuerzer als 1 s, viele nur 1 Epoche** -> ohne beide Bedingungen flackert das Gate. *(Direkt relevant fuer den 10-11-Verdacht "LIVE-Gate reicht schlechte Epoch durch".)* |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.57 | **W2 (10-02) - die Log-Falle**: die Distanz zwischen `GNSSData::latitude/longitude` (**was das Rohlog traegt**) und `latitudeNMEA/longitudeNMEA` (**was PAOGI wirklich sendet**, NAVProcessor.cpp:288), in Metern. Beide werden von VERSCHIEDENEN Parser-Pfaden geschrieben (GNSSProcessor.cpp:660/1447/1538) | **Eine Divergenz erreicht AOG als Positionssprung, waehrend das Log perfekt aussieht** - genau der Betreiber-Bericht "Knick in der Spur" (10-02). Kolumne pruefen: bei `W2 != 0` ist der Log-Spaltenwert NICHT die gesendete Position |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.62 | **Positions-Kontinuitaet** (Betreiber-Korrektur 09-29): es wird der Punkt gehalten, der WIRKLICH an AOG ging, damit der naechste Fix gegen das beurteilt wird, **was AOG gesehen hat** - nicht gegen die Projektion, die schon voraus sein kann | Betreiber-Korrektur |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.63 | **Rampenzeit eines Positions-Restes**: so gewaehlt, dass die ANFANGS-Rate der Blendung der erlaubten Aufholgeschwindigkeit `posMaxCatchUpMps` entspricht - nie ein Schritt | Betreiber-Korrektur 09-29 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.67 | **Vertrag von `applyMeasurement`**: `kind` 0 = Lage, 1 = pfadbasiert (COG/Fix-zu-Fix), 2 = Projektions-Rest; `dtRefS` MUSS die Update-Rate der Quelle sein (~0,1 s fuer die 10-Hz-Lage, 0,01 s fuer COG/Fix-zu-Fix/Projektion) | **Den Tick fuer die Lage durchzureichen war der Defekt vom 10-02 (R2)** - siehe B8.58 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.69 | **Step C (10-05)**: Fix-zu-Fix-Messung aus dem Positionsring, **einmal je Fix** | Betreiber-Entscheidung |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.74 | **Steuerkette im Log** (09-27): AOG-Zielwinkel, PWM, "Guidance aktiv" - eingebaut, weil der Betreiber **"wild steering" bei flackerndem Heading** meldete; ohne diese drei bleibt die Beschwerde subjektiv | Betreiber-Bericht |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.79 | **W1/W2 (10-02): `pub_lat/pub_lon` = die PUBLIZIERTE Position** (Projektion solange `publim = 1`, sonst die des Empfaengers) - bis dahin trug das Rohlog nur die Empfaengerposition, und die gezeichnete Spur ist keine von beiden, wenn der Bridge seine Projektion einsetzt | "Ein Knick in AOGs Spur war aus keinem Log rekonstruierbar" - der Grund, warum die Spalten 98/99 existieren |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.87 | **Wiedereintritt-Rampe** (Betreiber-Wunsch 09-23): war die Lage am Ende des vorherigen Ticks NICHT vertraut, ist dies der Frame, der psi in EINEM Schritt auf den Empfaenger ziehen wuerde | gemessen **244 Grad in einem einzigen 100-ms-Frame** an einem Hallen-Ausgang -> solche Frames werden mit erhoehtem R angewandt, wodurch dieselbe Korrektur ueber mehrere Frames verteilt wird (B8.23) |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.91 | **Ein von F4 geretteter Frame wird SANFT vertraut**: sein R wird vervielfacht, damit der veraltete Zustand ueber etwa eine Sekunde herangezogen wird statt zu springen | Betreiber-Entscheidung 10-01, siehe B8.85 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.96 | **Rampenzeit eines Positions-Restes aus der Aufhol-GESCHWINDIGKEIT**: eine Blendung erster Ordnung schafft in der ersten Sekunde `3*Rest/T`, also `T = 3*Rest/vmax` | **Ein 2,9-m-"Brueckensprung" braucht dann ~5,8 s bei vmax = 1,5 m/s, statt als ~29 m/s in EINEM 100-ms-Frame anzukommen - genau das hat der Betreiber als "der Traktor springt auf der Karte" gesehen** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.125 | **Das Positions-Kontinuum**: der Aufrufer setzt die Projektion nur ein, solange Rueckfall/Rampe laufen - sonst wird die Empfaengerposition veroeffentlicht. Genau diesen Punkt mitzufuehren macht den Schritt zwischen zwei veroeffentlichten Punkten messbar (`publim`/`pub_step`), damit der naechste Log beweisen kann, dass kein Sprung an AOG ging | Betreiber-Korrektur 09-29 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.136 | **LIVE-Rampe (10-05)**: beim OEFFNEN des Gates darf der Empfaengerwert nicht als Sprung ausgehen - beim Wiedereintritt weicht das Empfaenger-Heading **5...15 Grad (extrem 70...100 Grad)** vom weitergefuehrten Projektionskurs ab, **obwohl seine eigenen Qualitaetszahlen sauber sind** -> Schritt auf `liveRampDegPerS` begrenzt, die Projektion fuehrt in der Zwischenzeit weiter | Messung 10-05 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.147 | **Die Projektion sprang frueher OHNE Tor auf den Fix** -> nach jeder Unterbrechung wurde der naechste Fix 1:1 veroeffentlicht, egal wie weit er sprang (**gemessen auf den Brueckenfahrten: 1,4...2,9 m**). Jetzt startet sie von dem, was schon veroeffentlicht war | Betreiber-Korrektur 09-29 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.149 | **Fix-Tor**: ein physikalisch unmoeglicher, ein unter RTK-fixed liegender oder ein veralteter Fix darf die Projektion (und AOG) nicht herumziehen | gemessen: **2,2 m Positionsschritt bei 5,3 m/s auf einer Brueckenfahrt, und ein 5,5-Mm-Ausreisser im Stand (ungueltige Koordinaten)** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.151 | **Waehrend die Positionsrampe laeuft, werden die Reste NICHT als Messungen benutzt** - "sie wuerden die Rampe messen, nicht das Modell" | Betreiber-Wunsch 09-24 |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.11 | **Rohe Attitude-Qualitaet JE EPOCH im Ring** (10-05): die zwei Werte liegen in Bytes, die vorher Padding waren (Record bleibt **72 B**). Es sind ROHE Empfaengerzahlen, nicht das Urteil der Bridge -> **jeder Verbraucher darf eigene Schwellen anlegen**; "das sind die Qualitaetskriterien je Messung, die der Betreiber wollte, und der Ring bleibt politikfrei" | `PositionHistory.h:95` |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.12 | **Der Ring (300 x 72 B = 21,6 KB) MUSS in RAM2 liegen**: RAM1 ist das FlexRAM, das ITCM und DTCM teilen - nach dem Baseline-Fix vom 10-03 hat es **nur 5344 B frei fuer lokale Variablen**; ein 21,6-KB-Array dort wuerde den Build sofort brechen. `DMAMEM` legt den Block zu LINK-Zeit nach OCRAM (RAM2, 368 KB frei) | `PositionHistory.cpp:3` |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.14 | **Ein Record je FIX, Marker ist der POSITIONSZAEHLER, nicht die Uhr**: GNS/KSXT/INSPVA setzen `lastUpdateTime` fuer DENSELBEN Fix - **genau das liess am 09-23 das Fix-zu-Fix-Fenster den Fix der Projektion auffressen**. `posCount16` ist eine abgeschnittene Kopie (zwei Fixes 65536 auseinander waeren gleich) | `PositionHistory.cpp:82` |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.25 | **Der Positions-Ankunftszaehler ist noetig, weil `gpsData.lastUpdateTime` es NICHT kann**: GGA und GNS setzen beide `millis()`, das Feld bewegt sich also **2...4x je Satellitenepoche** und kann per Definition nicht "ein neuer Fix ist da" bedeuten (gefunden 09-23) | `GNSSProcessor.h:182` |

