# 60 Empfaenger, NMEA, Log-Spalten & Werkzeuge (E)

**Version 1.0.0 · 2026-10-11 · gilt fuer Logs 2026-09-07 bis 2026-10-11**
Aufgeteilt aus dem frueheren Register `WISSENSSTAND.md` (10-11, danach geloescht). **Die B-Nummern sind global und unveraendert** -
sie bleiben in allen Verweisen, Commits und Code-Kommentaren gueltig. Zugeordnet nach
HAUPThema der Zeile (`<!-- aus ... -->` nennt den Ursprungsabschnitt); Querthemen stehen
im Text. Einstiege: `00_ueberblick.md` (2 Minuten) und `symptome.md` (je Betreiberbericht).

| # | Befund | Beleg / Begruendung |
|---|---|---|
<!-- aus 1. Fundament (Wahrheiten) -->
| 1.1 | travel-att = 88,58 / 88,88 / 89,17 / 88,84 (4 Fahrten, 2 Tage) => Antenne, Versatz und Empfaengerlage stimmen, die Maschine crabbt NICHT | recovery §1 |
<!-- aus 1. Fundament (Wahrheiten) -->
| 1.9 | uni_hdgsd = 0 heisst "keine Loesung", nicht "perfekt" (Werte bis 1099,8) | 10-10 §2.6 |
<!-- aus 1. Fundament (Wahrheiten) -->
| 1.12 | Dual-Heading jittert 2...6 Grad/Frame, Empfaenger meldet 0,32...0,55 (Faktor 10 optimistisch) => gleitender Durchschnitt gerechtfertigt | 10-10 §2.9 |
<!-- aus 2. Mechanismen (das Herzstueck) -->
| 2.8 | Baseline-Gate: die Laengenmessung ist gut, die gelernte REFERENZ war der Defekt (0,852 m = +18,3 %) | baseline §1/§2 |
<!-- aus 2b. Nachtrag aus der Chronik (September bis 10-02) - schon einmal geloeste Probleme -->
| 1.17 | **Satellitenzahl ist KEIN Qualitaetsmass** (14...20 unter Baeumen, waehrend die Loesung degradierte - Betreiber-Korrektur 09-27); ebenso wenig die eigene `sigma`-Spalte (bewusst aufgeblaeht). Gate auf `headingQuality` | dito |
<!-- aus 2b. Nachtrag aus der Chronik (September bis 10-02) - schon einmal geloeste Probleme -->
| 2.11 | **BNO085-RVC hat keinen echten Gyrokanal**: der Parser summiert die Yaw-Differenz ueber 20 Pakete und resettet hart (Saegezahn). Bei aktivem IMU-Ratenpfad **ersetzt** das die Kinematik komplett => **95 Grad psi-Runaway in 3,4 s ohne Baeume, ohne GNSS-Fehler** (Log 10-02 11-27, t=90,0...93,4 s, v 8 m/s, Kurve sauber, qf 4/5); die `yawbias`-Schleife laeuft dem Saegezahn nach und klebt am +-30-Grad/s-Clamp | report 10-02 §0 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.7 | **`reseedBeyondDeg` 30 -> 45** (10-06): die 30 feuerte in einer **normalen engen Kurve** (+31,1 und +31,5 Grad hart, 2x in 2 s, bei perfektem Empfaenger) - der ZUSTAND war nicht falsch, das **Modell war zu kurz** | Mit `effWheelbaseFac = 1.0` verschwindet das Nachhinken; 45 laesst weiterhin keinen echten Zustandsfehler stehen (180-Grad-Klasse) |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.19 | **Baseline-Referenz wird vom Empfaenger selbst gelernt** (0 = lernen): solange die Lage plausibel ist und die Maschine faehrt, IST der Median der gemeldeten Laenge die mechanische Laenge. Ein eingetragener Wert uebersteuert. **Nicht aus vehicle.xml** (AOG speichert keinen Antennenabstand) | F9P-Handbuch-Test; Log 10-03 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.20 | **Sicherheitsnetz des Baseline-Gates** (statt Re-Learn): ueber ein Fenster beurteilter Frames zaehlt der Test seine eigenen Verwerfungen; ueber der Schwelle erklaert er die REFERENZ fuer falsch, **schaltet sich selbst ab und sagt es laut** | Beleg, dass es noetig ist: **10-03 12-52 verwarf eine gelernte 0,852 m (+18,3 % daneben) 99,7 % der empfanger-sauberen Frames der ganzen Fahrt**. Bewusst KEIN Re-Learn: "gemessen - ein Re-Learn greift den schlechten Abschnitt auf und bleibt falsch (88 % Verlust)" |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.36 | **Attitude-Qualitaetstor fuer die AOG-Ausgabe** (10-02): unter Baeumen traegt die rohe UM982-Lage **2...20 Grad Einzelframe-Artefakte**; genutzt werden die EIGENEN UNIHEADINGA-Standardabweichungen als Qualitaet JE EPOCH (der adaptive R braucht ~10 s) | `uni_hdgsd > 1,5 Grad` erkannte die Spikes mit **92 % Treffer / 10 % Fehlalarm**, die Baseline-Abweichung 90 % / 12 % |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.47 | **Zug der Lage verstaerkt** (10-05/06): die Nachfuehrung wurde deutlich kraeftiger und **auch bei mittlerer Empfaengerqualitaet aktiv**, weil Baeume/Bruecke oft unter 1,5 Grad bleiben | gemessen: mit Empfaenger weg (E5a, Log 6-13) und im Neustart-Modus (Modell wirkungslos) lag psi bei **2,1 Grad statt 9,1** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.48 | **P2-Transparenz** (Betreiber-Entscheidung 09-30): der Ausgangsbegrenzer schreibt mit, welche Schranke er benutzte (`rl_allowed`), welcher Teil davon aus einem akzeptierten Pfad-Dreh kam (`rl_path`), ob geclippt wurde (`rl_clip`) und welches R die angewandte Messung wirklich hatte (`meas_r`) | das sind genau die HR-Spalten 84...87, mit denen wir den Begrenzer jagen - der Betreiber hat sie einbauen lassen, weil "ohne diese drei bleibt die Beschwerde subjektiv" |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.54 | **Ausgangs-Qualitaetstor** (10-02): `att_q` = 1 sauber / 0 degradiert / **-1 = der Empfaenger meldet je Epoch keine Zahl -> Tor OFFEN, Verhalten unveraendert**; `out_snap` = 0 ratenbegrenzter Rest (kein Sprung erreicht AOG) / 1 gesprungen (saubere Epoch, verzoegerungsfrei durchgereicht) / 2 kein Rest vorhanden | Spalten des HR-Logs |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.70 | **Antennenabstand wird in `init()` GANZ OBEN angewandt** (10-03): wenn je eingetragen, ist er ab dem ERSTEN Frame die Referenz des Baseline-Tests und der Lernpfad wird ganz uebersprungen. Laeuft absichtlich VOR der "HeadingBridge ENABLED: ... base=" Zeile | Fix fuer den 10-03-Fehler: die gelernte Referenz stand auf **0,852 m = +18,3 % der echten 0,711 m, eingefroren VOR dem Log** -> das Tor verwarf **99,7 %** der empfanger-sauberen Frames der ganzen Fahrt, **ohne Revisionsweg** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.77 | **`att_q` = −1 heisst "Tor OFFEN"**: der Empfaenger meldet je Epoch keine Zahl (F9P oder UNIHEADINGA nicht aktiviert) -> **der Ausgangspfad verhaelt sich exakt wie vor diesem Tor** | Spaltenkommentar 10-02 ("Stufe 2") |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.81 | **M1-Entkopplung (10-06)**: das Step-Gate muss mit dem VOLLEN R rechnen - sonst schaerft der psi-Anker das Gate mit und verwirft genau die Frames, mit denen psi dem Empfaenger nachziehen soll | gemessen: der Anker wirkte nur **11,6 -> 9,6 Grad statt < 3 Grad** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.85 | **F4 (Betreiber-Entscheidung 10-01)**: wenn `stuckRescueAfterMs` lang keine Messung akzeptiert wurde und der Frame NUR von einem PLAUSIBILITAETStor verworfen wird (**nie von der Empfaengerqualitaet**), ist der ZUSTAND verdaechtig - der Frame wird mit **R * `stuckRescueRFactor`** akzeptiert, damit die Schaetzung sanft herangezogen wird statt nie | gemessen im 13-25-Log: **60 s DEAD RECKONING bei gueltiger Dual-Lage (qf 4, 20 Sats)**; das falsch eingerastete REVERSE liess `cogX` alles verwerfen, psi driftete **30 Grad mit 1,5 Grad/s**, und AOG sah "den Traktor schraeg fahren" |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.89 | **M1 (10-06)**: solange der Empfaenger sauber ist, IST sein Azimut die Wahrheit - dann darf er psi mit KLEINEREM R (also staerker) nachfuehren, statt psi frei davon driften zu lassen. Wirkt **nur auf `kind 0` (Lage)**, alles andere bleibt unveraendert | gemessen 10-06: **11,6 Grad Abstand bei 1,7 Grad Empfaengergenauigkeit** |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.92 | **R-Deckel der Attitude** (Betreiber-Entscheidung 10-05): der adaptive Wert (EMA der Innovation^2 minus p11) fuellt sich aus dem **ZUSTANDSfehler** - ein bestehender Offset ERHOEHT R und SENKT den Gewinn: **der Offset schuetzt sich selbst**. Deshalb der Deckel; die Untergrenze sorgt dafuer, dass Rauschen weiter gemittelt wird | gemessen 10-04/05 (Logs 13-53, 16-04, 17-28): R lief **3,95 -> 6,27 Grad, K1 0,023 -> 0,010**, und ein **10-Grad-Offset stand 35 s Geradeausfahrt still**, obwohl die Lage die belegte Wahrheit war (Versatz `att-cog` stabil **-88,2 Grad**). Der Empfaenger liefert sein eigenes 1-sigma je Epoch (`uni_hdgsd` **0,40...0,60 Grad** bei gutem Fix) |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.93 | **R4 (10-02)**: die EIGENE gemessene Gierrate des Empfaengers ist der Term, der die Schranke im engen Manoever richtig macht - sie verlangt eine VERTRAUTE Lage | in den degradierten Phasen der Fahrt 10-01 20-15 traegt sie daher NICHTS bei und die Schranke bleibt rein kinematisch |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.94 | **Ein falscher Baseline-Wert trifft ZWEI Pfade**: dieselbe Referenz, dieselbe Abweichung und (seit 10-03) dieselbe Toleranz `baselineTolPct` benutzen das Ausgangs-Qualitaetstor UND das Verwerf-Tor - nur die Folge unterscheidet sich: hier wird die Epoch als **degradiert** markiert (`att_q = 0`, Ausgang ratenbegrenzt) und die Lage trotzdem benutzt, dort wird die Messung **verworfen** | Deshalb schaltet das Sicherheitsnetz BEIDE ab, und deshalb repariert EIN korrekter EEPROM-Wert beide |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.129 | **LIVE-Gate (10-05)**: bis dahin galt das Durchreichen des Empfaenger-Azimuts NUR vor dem ersten vertrauenswuerdigen Frame (Kaltstart); danach wurde immer der Filterzustand psi gesendet - **auch wenn der Empfaenger die besseren Zahlen liefert. Genau das ist der gemessene 9,6-Grad-Versatz auf den Geraden** | Betreiber-Entscheidung 10-05 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.131 | **Auch beim SCHLIESSEN des LIVE-Gates wird umbasierend gerampt** (10-07): vorher ging der Wert hart vom Empfaenger-Azimut auf psi | gemessen **34 Grad in EINEM Frame** (\|az_out-att\| p90 = 34,3 im Log 6-13) |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.135 | **"Stufe 2" (10-02): der harte Sprung ist QUALITAETSGEGATET.** Vorher wurde JEDER Rest ueber 3 Grad in EINEM Frame in die Ausgabe kopiert ("der geglaettete Wert darf einer echten Kurve nie nachhinken"). Unter Baeumen traegt die rohe Lage aber **2...20 Grad Einzelframe-Artefakte, waehrend qf 4 bleibt** -> jeder davon erreichte AOG als Sprung: **die "Spruenge in den AOG-Daten" des Betreibers** | gemessen 09-30...10-02 |
<!-- aus 8. HeadingBridge: gemessene Entscheidungen aus den Code-Kommentaren (H) -->
| B8.138 | **Das Modell-Gate erkennt die Not DIREKT**, also auch wenn der Empfaenger weiter Qualitaet 4 meldet (Betreiber-Bericht 09-27: *"unter den Baeumen drehte das Heading um etwa 1 Grad, obwohl die Lenkung ruhig war"*) | in dieser Baumfahrt kamen **84 % dieser Schritte aus der ROHEN Lage (p50 2,1 Grad je 100 ms), waehrend qf 4 blieb** |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.15 | **EEPROM 1600...1699**: Byte 0 = Skalen-Gueltigkeit (0xA5), 1...4 = float Skala, 5...6 = **Marker-PAAR** der Baseline (0xA5 0x5A), 7...10 = float Antennenabstand, 11...12 = Marker-Paar des Offsets, 13...16 = float Montage-Offset. **Die Paare existieren, weil der Bereich vor dem 10-03 nie benutzt wurde** - ein Byte wuerde Alt-Daten mit 1:256 Wahrscheinlichkeit akzeptieren | `ConfigManager.cpp:269/290` |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.22 | **Die gemeldete Baseline-Laenge ist der empfaengerunabhaengige Attitude-Gesundheitsindikator**, den das u-blox-F9P-Integrationshandbuch fuer RELPOSNED empfiehlt: die geloeste Laenge gegen die physisch gemessene vergleichen | `GNSSProcessor.h:66` |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.23 | **UNIHEADINGA (Unicore N4, Tabelle 7-169), eingebaut 10-01**: die EIGENE Aussage des Empfaengers ueber die Baseline (Laenge, Heading, Pitch + beide Streuungen) - **zunaechst NUR geloggt** ("die Skala von `hdgstddev` muss erst mit dem auf einer Fahrt beobachteten Rauschen verglichen werden"). `length` fuellt auf einem Unicore `baselineLengthM` | `GNSSProcessor.h:85` |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.26 | **UNIHEADINGA wird direkt neben dem HPR-Ereignis geloggt, damit man sie vergleichen kann**: zeigt `$GNHPR` KEINE Loesung, waehrend UNIHEADINGA weiter Laenge und Streuungen meldet, **liegt der Verlust in jenem Satz, nicht im Empfaenger** | `HeadingMonitor.cpp:168` |
<!-- aus 9. Weitere Module - Einheiten, Fallen und "nicht nochmal" -->
| B9.27 | **Wozu das UNIHEADINGA-Instrument da ist**: die Baseline-**LAENGE** existiert NUR in diesem Log (der `$GNHPR`-Satz hat kein solches Feld - `baselineLengthM` hatte auf diesem Empfaenger ueberhaupt keine Quelle), und die eigenen `hdgstddev`/`ptchstddev` sind seine einzige unabhaengige Aussage darueber, wie gut die Lage wirklich ist - **die Groesse, gegen die das adaptive R irgendwann geprueft werden soll** | `HeadingMonitor.cpp:325` |

