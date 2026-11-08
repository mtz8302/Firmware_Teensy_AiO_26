# Ueberblick: was wir wissen (Einstieg)

**Version 1.0.0 · 2026-10-11 · gilt fuer Logs 2026-09-07 bis 2026-10-11**
**2-Minuten-Einstieg**: Stand, Architektur-Entscheidungen, was WIDERLEGT ist, die Messbasis
und die offenen Luecken. Die Befunde selbst liegen in den Themendateien (IDs global).

**Aeltere Verweise** wie "Register §1" oder "Register §2" (in `symptome.md`, `archive/GRUNDZIELE_2026-10-11.md`,
`WORKFLOW.md`, Code-Kommentaren) meinen die **heutigen Themendateien**: die Zeilen sind nach
Thema verteilt, die **IDs sind unveraendert**. Weitere B-Nummern zaehlen global weiter.
Die frueheren Abschnitte 0/3/4/5/6 stehen weiter hier im Ueberblick.

## Themen-Karte (welche Frage wohin)

| Frage / Symptom | Themendatei | typische IDs |
|---|---|---|
| "faehrt schraeg", "Kurve wird nicht gefahren", "Heading verwackelt", Ausfaelle/Wiedereintritt | `10_heading.md` | B1.x, B2.x, B8.x (Richtung/Ausgang/Gates) |
| Positionssprung, publizierte Position, Projektion, Kursfenster, Ring | `20_position.md` | B1.3...B1.7, B8.x (Projektion/Ring/Fix-Tor) |
| "AOG zeigt rueckwaerts", K-Turn, Richtung kippt, Stand/Anfahren, Zeugen/Veto | `30_richtung.md` | B8.5, B8.6, B8.82, B8.83, B8.141...B8.144 |
| Lenkwinkel falsch, Nullpunkt, cpd/Skala, Keya, Sidehill-Gain, Qualitaet | `40_vwas.md` | B7.x, B9.3...B9.9 |
| Roll springt, Hangfahrt, IMU-Roll, Roll-Halten | `50_roll.md` | B8.40...B8.42, B8.105...B8.110, B9.16...B9.18 |
| Empfaengerqualitaet, QF/UNIHEADINGA, Baseline, Log-Spalten, Werkzeug-Fallen | `60_empfaenger_logs.md` | B1.8, B1.9, B1.17, B9.22...B9.28 |
| Betreiberbericht zuordnen, "hatten wir das nicht schon?" | `symptome.md` | S1...S8 + Chronik |
| Ablauf, Invarianten, Git-Regeln, Kennzahlen-Panel | `WORKFLOW.md` | I1...I11 |
| **Naechste Fahrt**: was fahren, welche Spalten auswerten, Zahl->Entscheidung | `70_feldprobe.md` | - |
| Ziele, Messlatte (was "gut" heisst) | `01_ziele.md` | - |
| Meldungen im Fuehrerhaus (PGN 221), Sprachen, Schalter, Meldungs-EEPROM | `90_aog_messages.md` | B10.80 |
| **Web-UI ueber LAN, ESP32-Bridge, Seiten der Module (`sc.aog`, Modul-Proxy, DNS/mDNS)** | `95_webui_und_bridge.md` (Modulseite selbst: im jeweiligen Modulprojekt) | B11.1...B11.9 |
| Werkzeuge finden | `tools/analysis/README.md`, `tools/heading_bridge/README.md`, `tools/sim/README.md` | - |

## 0. STAND (diesen Block bei jeder Sitzung zuerst lesen und aktualisieren)

* **FAHRDYNAMIK-SITZUNG 2026-11-06 (Branch `feat/hold-att-rate`)**: die Kennzahl
  "ruckelig" (Ausgabe steht < 0,2 Grad/100 ms ODER springt > 3x Lageschritt, in
  Kurvenframes der Lage > 2 Grad/100 ms) fiel in fuenf Schritten **23 % -> 18 -> 11 -> 7 ->
  4 -> 3 %**; stehende Frames **77 -> 6**; Standrauschen p50 0,2 / p90 1,0 / p99 2,1-2,5
  Grad. Die fuenf Ursachen stehen als **B10.73...B10.77** in `10_heading.md` (MA-Fenster
  nach Drehrate, Halten-Zweig in vier Teilen, gamma/2 ratenbegrenzt, LIVE-Rampe folgt der
  Lagrate, Snap-Einfrieren entfernt). **B10.78** = Halte-Deckel aus der Fahrzeuggeometrie
  (Knicklenker/Ackermann, deckungsgleich mit B10.33/B10.46), **B10.79** = was die Metrik
  nicht sieht. Neu: **`90_aog_messages.md`** (PGN 221 Hardware-Meldungen im Fuehrerhaus).
  Offene Punkte: `80_plan.md` §4.

* **ZIEL GESCHAERFT (10-12, 01_ziele.md §1.0)**: das Heading soll auf Geraden und in leichten
  Kurven nur **±0,5°** schwanken und sich mit **≤ 1 °/s** ändern — **in Grad und Grad/s, nicht
  je Frame** (Logs kommen mit 5/10/20 Hz; die alte Zeile „sd je 0,1 s (10 Hz)" war
  rate-abhängig ✗). Begründung: 0,5° = Eigenrauschen der Dual-Antenne (0,71 m, `uni_hdgsd`
  0,4–0,6°); 1 °/s = reale Drehrate bei 2–4 m/s mit 0,5° Lenkwinkel. **Kurven**:
  `max(1 °/s, k·ω_phys)` mit dem trim-korrigierten Winkel, **keine** Bestätigungs-Verzögerung.
  **Gilt auch bei schlechtem Empfang** (Betreiber: sonst schaukelt sich der Traktor auf) ⇒
  die **Rate ist die harte** Forderung (nie springen — AOG lenkt danach), das **Band die
  Qualitäts**forderung; in schlechten Phasen nahe am letzten vertrauten Kurs bleiben, per Rampe
  nachführen. **Wahrheit = Positionskurs als 2-s-Fit** (die Einzelframe-Ableitung `travel` ist
  selbst zu verrauscht: p90 14–21 °/s ✗). **Offene Umsetzung**: `outSnapDeg`,
  `outSnapConfirmFrames`, `outNoiseSamples` sind noch **frame-basiert** (3° = 15–60 °/s, je
  nach Rate ✗).

* **Letzte Aenderung**: 10-11 · **Feldprobe vorbereitet** (`70_feldprobe.md`, v1.0.0) plus die
  Kette B10.19...B10.24: die EEPROM-Referenz der Baseline-Laenge analysiert (drei Zustaende,
  fuenf Nutzer) und **aktiv pruefbar** gemacht (fruehe Warnung + Web-Knopf „gemessene Laenge
  uebernehmen"); A10 als **Instrument** gebaut (**Zaun `outFenceK` Standard AUS**, Vorhalts-
  Vertrauenspruefung an, Rausch-alpha aus); und die **Jitter-Quelle GEMESSEN**: nicht der Filter,
  sondern der **VWAS-Nullpunkt 2...6° daneben** (B10.24, drei unabhaengige Messungen) — dazu ein
  **Sim-Treuefix** (`wheel_deg` = der *benutzte* Winkel, wie in der Firmware). **Naechster Schritt:
  die Fahrt nach `70_feldprobe.md`** — sie liefert die Spalten 103...116 UND den Guidance-Fall.
* **10-11 (Struktur + Workflow)**: **Struktur + Workflow festgeschrieben** (v1.0 der drei Dateien).
  Geltungsbereich = **alle Subsysteme** (H/P/V/R/L/N, VWAS ausdruecklich dabei, weil gekoppelt);
  ID-Schema vereinheitlicht (`B`/`S`/`I`, 34 Verweise migriert); Invarianten I10 (Wheel-Quality
  => Modell schweigt) und I11 (kein Heading in die VWAS-Nullpunktbildung) ergaenzt; S6 auf VWAS
  erweitert. **Chronik begonnen** (14 Zeilen, Sept bis 11.10.) aus den September-Werkzeugen und
  dem Bericht 10-02 (W1.16...1.19, B2.9...2.13). Negative Diagnose: **keine IMU** in den drei
  10-11-Logs (B2.11 damit ausgeschlossen). Firmware unveraendert (HEAD `713fdb6`), Sim gruen.
* **Versionierung auf v1.0.0** umgestellt (dreistellig; **PATCH +1 vor jedem Log-Lauf**),
  **Referenz-Log-Satz (7 Logs)** und **Lauf-Register** angelegt. **Quellenlage geprueft**:
  Anfang der Arbeit = **2026-09-07** (Tippfehler-Datei umbenannt), kein Juni-August-Material,
  Git-Zeitleiste erst ab 10-04, drei Log-Familien (WAS/heading/HR) dokumentiert.
* **VWAS-Chronik begonnen** (B1.20...B1.23 aus den 20 Event-Logs `Log virtual WAS …`):
  am 09-08 saettigt `gpsAngle` bei +-40 Grad bei Schritttempo und der Encoder hat keinen
  Nullpunkt (-> Nullpunkt-Lernzustand gebaut); am 09-21 war der WAS-Nullpunkt **20...23 Grad
  falsch** (`enc -23,28` gegen `gpsAngle +0,81` bei 1,34 m/s, RTK) und der Lernschritt zu klein
  (`res -21,02`, `used 1,02`, `gain 0,053`; spaeter `gain -1,274`). Werkzeugfehler gefixt:
  `_vwas.ps1` las **1 statt 320** Zeilen (Event-Log hat nach dem Trenner ein echtes CRLF).
* **Code-Kommentare als Quelle ERSCHLOSSEN** (10-11, auf Betreiber-Hinweis "vwas funktioniert
  gut"): die datierten Kommentare sind die einzige Geschichtsquelle fuer September.
  **`HeadingBridge.h` 90/90 Bloecke, `HeadingBridge.cpp` 128/128 Bloecke**, dazu
  `WheelAngleFusion.{h,cpp}` und die "DO NOT FIX"-Notiz in `BNOAiOParser.cpp`
  -> **172 Registerzeilen (B7.1...B7.14, B8.1...B8.160, B9.1...B9.2)**.
  Kernfunde: AOG besitzt Nullpunkt UND Skala (zwei Lerner kaempfen), die Zeitbasis des
  Begrenzers war der 100-Hz-Tick statt der Quell-Epoche (10x zu eng), der Radstand-Widerspruch
  ist beidseitig datiert (B8.3/B8.43 = **Nichtlinearitaet**), der Zirkelpfad ist real und per
  `headingIsMeasured` abgeriegelt, "LIVE allein genuegt nicht" (Modell-Gate hebt bis 600 ms),
  und die "IMU-Roll kommt 10x zu gross" ist widerlegt (B9.1/B9.2).
* **Weitere Module ERSCHLOSSEN** (10-11): `AutosteerProcessor.cpp` (10), `BNOAiOParser.cpp` (3),
  `NAVProcessor.cpp` (4), `GNSSProcessor.h` (4), `HeadingMonitor.cpp` (3), `PositionHistory.{h,cpp}`
  (2+3), `ConfigManager.cpp` (2), `IMUProcessor.cpp` (1) -> **§9 (B9.1...B9.28)**.
  Kernfunde: **`countsPerDegree` 100 war ~4,7x zu gross** (der 09-08-Log impliziert ~21,5, der PID
  fuhr in den Anschlag); `invertWAS`-Vorzeichen (die 09-21-Logs zeigen genau diesen Fall);
  **EIN Lenkwinkel je Zyklus** seit 09-28 (vorher modellierte die Bridge mit einem Winkel, den die
  Firmware selbst fuer ungueltig erklaert hatte); der Ring MUSS in RAM2 (RAM1 hat nur 5344 B frei);
  `gpsData.lastUpdateTime` ist KEIN Fix-Zaehler (GGA+GNS, 2...4x je Epoche) - deshalb die eigenen
  Zaehler; EEPROM-Marker-PAARE gegen Alt-Daten (1:256).
* **Damit sind ALLE datierten Code-Kommentare ausgewertet** (HeadingBridge, WheelAngleFusion,
  Autosteer/NAV/GNSS/Monitor/PositionHistory/Config/IMU/BNOAiOParser).
* **FIX 1.2a/1.1 gebaut und im Sim verifiziert** (10-11). Zwei Aenderungen in `HeadingBridge.cpp`:
  (a) das Kinematik-Tor vergleicht die Lage gegen die **vorige Lage** statt gegen den
  publizierten Wert (das war ein Selbstbezug: mit `psi` als Ausgabe sah JEDE plausible Lage wie
  ein Sprung aus - B10.10), (b) **`psi` faellt als Ausgabe weg** (letzter Wert wird gehalten).
  **Sim gegen Firmware:** 6-57 errTrack p90 **28,3 -> 13,3** (schlechte Epochen 19,4 -> 5,7 %),
  6-52 p90 **5,70 -> 4,10** (>10 km/h p90 5,70 -> 3,30), 7-01 Band 1...2 km/h **17,1 -> 6,70**;
  Positionsspruenge unveraendert.
  **Offen:** `psiDrift` explodiert (p50 148 statt 14 - der Anker/Re-Seed muss beobachtet werden,
  B10.13), ein besseres Abnahmekriterium fuer Schleichfahrt, dann **1.2b**, dann Commit + Baseline.
* **Der Sim kann jetzt schnell rechnen**: Argument `nofeed` (oder `SIM_NOFEED=1`) schaltet den
  AOG-Feed ab -> Replay ~16x schneller (~10 s je 155-s-Log). Damit ist die Regression billig.
* **Log-Block "VWAS-Beobachter" gebaut** (10-11, **Spalten 103...115**): `enc_cnt, enc_deg,
  enc_off, zo_res, zo_st, zo_fit, zo_gain, zo_roll, zo_rate, gps_a, aog_zero, aog_wasoff,
  cpd_aog`. Alle Werte **existierten schon** im Code (getRawCounts/getEncoderAngle/
  getEncoderOffset/getZeroObserverStatus/getGPSAngle/getAog*) -> **reine Verdrahtung, kein
  neues Verhalten**, Firmware-Build gruen, **RAM1 unveraendert**. Zweck: Sim-Replay einer echten
  Fahrt (enc_cnt = VWAS-Eingang), der Trim (zo_res) und die attitude-freie Kreuzprobe (gps_a).
  **OFFEN:** der Simulator gibt die neuen Spalten noch nicht aus (Ursache offen, B7.24) - das ist
  die Voraussetzung fuer den naechsten Schritt (`WheelAngleFusion` + Stub-Encoder in den Sim).
* **Sim-VWAS laeuft und ist VERIFIZIERT** (10-11, Register B7.25...B7.32): `WheelAngleFusion` +
  Stub-Encoder im Sim, Boot-/Nullpunkt-Erkennung aus dem Log (`enc_off` uebernehmen bzw.
  `adoptBootZero`), Feld-`wq`/`guid` je Satz eingespeist. **7-01 mit VWAS**: `state=LIVE`, `src=att`,
  `travel=267,5` (wie die Firmware), errTrack p50 **5,60** (vorher 6,30), **`zo_st=1` (ACQUIRE)**,
  `zo_fit=3`, `gps_a` echte Kruemmung. **Der tagelange "VWAS macht die Bridge kaputt"-Verdacht war
  eine ODR-Verletzung in den Sim-Stubs** (zwei Definitionen von `GNSSProcessor`; nur eine hatte
  `insPitch`/`insRoll` = genau die 8 Byte, um die die Bridge `headingQuality` daneben las -> Wert
  72 = 'H'). **Regel: EINE Definition je Typ, `*_stub.h` inkludiert nur** (README tools/sim).
* **TRIM-FIX gebaut + ABGENOMMEN** (10-11, Register B7.34): die Bridge liest `residualDeg` aus
  dem VWAS-Beobachter und rechnet ihr Modell mit `delta_eff = delta - res` - **nur intern**
  (kein Schreiben, kein EEPROM, keine Rueckgabe; `sc_b` bleibt Fehlerindikator, neue Spalte
  `vwas_trim`, abschaltbar per `vwasTrimInModel`). Im Sim mit eingespeister Wanderung
  (+12 Grad, Guidance erzwungen): `mdev` p50 **3,96 -> 0,18** (Referenz ohne Fehler 0,00),
  `mgate` **764 -> 9** (Referenz 3), angewandter Trim **12,29** = genau der Fehler.
  **WICHTIG (B7.35)**: **alle 10-11-Logs haben `guid = 0`** - die Fahrten liefen OHNE
  Autosteer, der VWAS war also nicht blockiert und trimmt selbst. **Der Ziel-Fall des Fixes
  (Guidance aktiv) fehlt im Feld** => fuer den Nachweis eine Fahrt MIT Autosteer fahren.
  Und: `errTrack`/`proj_ctk` zeigen den Modell-Fix NICHT, das Mass ist `mdev`/`mgate`.
* **A10: DIE JITTER-QUELLE IST GEMESSEN (10-11, Register B10.22)** — sie ist **nicht** der Filter,
  sondern ein **falscher Lenkwinkel-Nullpunkt** ✗: auf den auffälligen Frames sagt der **Pfad
  „gerade" (≤1,5 °/s)** und der **Lenkwinkel 25–28°** ⇒ die Modellrate (~39 °/s) und damit der
  Vorhalt (~7,8°) sind falsch ⇒ die 2,0°-Schritte. (Meine erste Auswertung war **falsch
  klassifiziert** — sie benutzte die *fusionierte* HR-Spalte `wheel_deg`, die dort ≤2° stand:
  genau das VWAS-Residuum aus B7.20.) ⇒ **Der Zaun kann das nicht heilen, er erbt den falschen
  Lenkwinkel.** Was er kann (pfadbasiert klassifiziert): die **p99-Schritte überall senken**
  (652: 4,50→2,10 · 5,10→2,30; 701: 6,00→2,10 · 6,50→2,40 ✓✓) und in Kurven neutral bleiben ✓.
  ⇒ Rangfolge: **(1) vertrauenswürdiger Lenkwinkel (VWAS-Nullpunkt/Trim) zuerst**, (2) dann Zaun
  mit k≈3, (3) relativer Slack für die ±0,5°-Zielzahl. Zaun bleibt Standard AUS ✓.
* **A10 (Glättstärke nach Datenlage) = ZWISCHENSTAND, nur als INSTRUMENT (Register B10.21)**.
  Dein Ansatz ist **geprüft und richtig**: der kinematische Zaun existiert schon **mit dem
  trim-korrigierten Lenkwinkel** (`kinematicRateBoundDegS()`, Code:1046) und wird für
  Messungs-Begrenzer und psi benutzt — nur **nicht** für die Ausgabe. Messung an den Feldlogs:
  auf Geraden erlaubt der Zaun **0,45–0,70 °/s**, die echten Ausgabeschritte sind aber
  **5–8 °/s** (p90) ⇒ **53–78 % physikalisch unmöglich**; in Kurven öffnet er bis **94 °/s**
  (dort keine Verzögerung nötig ✓). Gebaut als Instrument mit **Standard AUS** (`outFenceK=0`)
  + Boden bei Stillstand + Ventil (Rampe statt Hängen).
  **Ehrliches Messergebnis:** auf Geraden verschlechtert der Zaun den p90-Schritt
  (0,40 → **2,00°**) ✗ — der Mechanismus ist **noch offen** (nächster Schritt: instrumentieren,
  nicht raten); in Kurven ist er neutral bis besser (701 p99 **88,8 → 3,9°**, Anker-Sprung
  gerampt ✓). **Struktureller Befund:** wo der Lenkwinkel nicht vertrauenswürdig ist, greift der
  15°-Rückfall (~**42 °/s**) ⇒ wirkungslos ✗ — die Geraden-Wirkung hängt also an einem
  vertrauenswürdigen Lenkwinkel (VWAS/Trim) ✓. Der Rausch-alpha-Teil ist ebenfalls Standard AUS.
* **AKTIVE REFERENZPRUEFUNG gebaut + im Sim belegt (10-11, Register B10.20)**: die Bridge mittelt
  die vom Empfaenger GELOESTE Baseline-Laenge ueber die ersten 40 sauberen Frames (~4 s) - auch
  bei gesetzter FESTER Referenz, also unabhaengig von jeder Referenz - und vergleicht sie einmalig.
  Abweichung > 5 % => **Firmware-Warnung mit Handlungsanweisung** + Flag; zusaetzlich ein
  **Web-Knopf "Gemessene Laenge als Referenz uebernehmen"** (schreibt ins EEPROM, nur per Klick).
  Nachweis: injizierte 0,852 m gegen die feste 0,711 m => Warnung bei Frame 40 (+19,8 %), danach
  das Sicherheitsnetz bei 300 Frames; echte 0,714/0,719 m => "passt" (kein Fehlalarm).
  **Der Sim kann jetzt Firmware-Logzeilen zeigen** (`SIM_LOG=1`; vorher waren LOG_* No-Ops) - damit
  sind kuenftige Warnungen im Sim pruefbar, ohne die Firmware anzufassen. Neuer Schalter
  `SIM_BASE_M` fuer die Baseline-Fehlerinjektion. **Offen: die Referenz-Verbesserungen sind im
  Feld noch nicht erprobt** - sie brauchen nur eine Fahrt mit gueltigem Empfang.
  **Verhaeltnis** Lenkwinkel -> Gierrate (= SKALA; erklaert den Widerspruch B8.3 gegen B8.43)
  und der **hydrostatische Lenkungs-Schlupf** verschiebt die **NULL** (B8.137). Zwei Knoepfe,
  zwei Eigentuemer: `steerScale` (Betreiber/AOG) und `vwas_trim` (Beobachter misst, Bridge
  wendet an). Grenze: der Bridge-Trim ist NUR ein Offset - Grip/Skala kann er nicht heilen.
* **Commit-Stand (Ende dieser Sitzung, alles gebaut und gruen)**: `0b4c70c` Hygiene (Wegwerf-Logs
  + Log-Rename) · `83f42af` **Fix 1.2a/1.1** + Beobachter-Spalten [B10.10/B10.12/B7.23] ·
  `65d078f` Autosteer-Push [B7.23] · `832ecad` Kommentar-Korrektur [B8.103] · `b218bad`
  **Sim-VWAS** [B7.25/B7.24] · `0aa71bb` **ODR-Fix in den Stubs** [B7.31] · `4d5a9b7` .gitignore ·
  `77d600d` Werkzeuge · `e9cdd79` Wissensbasis · **`72fb2d4` TRIM-FIX** (VWAS-Trim im Modell +
  `vwas_trim`-Spalte) · **`aefe4b3` Sim** (Beobachter-Push + Fehler-Einspeisung) · **`9671295`**
  Physik-Hinweis (Grip/Hydro-Schlupf) + Abnahme. **Firmware unveraendert bis auf die zwei
  Bridge-Zeilen + Log-Block + Trim; alle Fixes sind im Sim verifiziert, keiner im Feld.**
  **Fuer den Feldnachweis fehlt eine Fahrt MIT Autosteer** (alle 10-11-Logs haben `guid = 0`,
  B7.35) - und das Mass dafuer ist `mdev`/`mgate`, nicht `errTrack`.
**Reihenfolge (Betreiber 10-11)**: erst Struktur/Workflow - **erledigt** -, dann Inhalt:
Chronik weiter **inkl. VWAS** -> Symptomtabellen schaerfen -> Baseline/Regression bauen ->
erst dann der erste Fix. **Chronik-Rueckmarsch-Grenze: ab Juni 2026** (Betreiber-Angabe);
aeltere Dokumente nur, wenn sie eine heute noch geltende Entscheidung tragen - dann mit Datum markiert.
**Referenz-Log-Satz (Baseline, Betreiber-Freigabe 10-11)** - deckt Kurve, Bruecke, Schleichfahrt,
20 km/h und Stand ab:
1. `2026-10-11 6-52 0 r hof bach brücke 90 failed Gerade 21kmh verwackelt.txt`
2. `2026-10-11 6-57 k brücke 6kmh heading lost Positionssprung.txt`
3. `2026-10-11 7-01 brücke 1-5kmh heading medium LW 5-6 off.txt`
4. `2026-10-10 12-31 0 hof bach Brücke baum besser aber 90 kurve klappt nicht.txt`
5. `2026-10-10 12-36 k brücke baum k baum brücke heading teils sehr schlecht positionssprung.txt`
6. ein Log vom 10-09 (Hof/Bach/Bruecke) - Auswahl beim ersten Baseline-Lauf
7. `2026-10-06 7-07 Bäume schleichfahrt autosteer heading lost und schrägfahrt.txt` (Nachtrag Betreiber)
**Regel: jeder neue Feldlog kommt dazu** - der Satz waechst mit den Fahrten.

**Lauf-Register** (PATCH-Version je Log-Lauf, Sim oder Feld):
| Version | Datum | Log(s) | Art | Zweck/Ergebnis |
|---|---|---|---|---|
| v1.0.0 | 10-11 | keiner | Doku | Struktur/Workflow erstellt, kein Lauf |

**Quellenlage (geprueft 10-11 - wichtig, damit wir nicht vergeblich suchen)**
* Fahrten: **128 datierte Logs vom 2026-09-08 bis 2026-10-11**, in drei Familien:
  1. `Log virtual WAS 2026-09-08` + 28x `Log virtual WAS 2026-09-21 …` (Muster `0-re-li-0`,
     `20-re-li-0`, `AnschlagKali`, `Halle`, `Holperstrecke`) = **VWAS-/Lenkwinkel-Kalibrierung**.
  2. `Log virtual heading 2026-09-23 …` + `log 2026-09-23 …` = **Beginn der Heading-Arbeit**.
  3. `2026-09-27 … 2026-10-11 …` = die **HR-Logs** (102 Dateien, 10 Hz, 94 Spalten).
* Keine Fahrten: `_b*.txt`, `be.txt`, `bo.txt`, `_sim*.txt`, `_virt*.txt`, `clean.txt` usw. sind
  Analyse- und Simulator-Dumps. `2027-09-7 7-51 0 eichgraben - brücke.txt` war ein
  **Jahres-Tippfehler** (Betreiber-Bestaetigung 10-11: 2026-09-07) und damit der **aelteste Beleg**;
  umbenannt in `2026-09-07 7-51 0 eichgraben - brücke.txt`, damit sie in `2026-*`-Filtern auftaucht.
* **Kein Material von Juni bis August 2026 vorhanden** (keine Datei mit 2026-06/07/08 im Namen).
  Betreiber-Bestaetigung 10-11: *"wenn sie hier nicht ist, habe ich erst dann angefangen"* -
  der Anfang der Arbeit ist also **2026-09-07**. **Nicht weiter zurueck suchen.**
* **Git-Zeitleiste beginnt erst am 2026-10-04** (19 Commits, `ringpuffer1` … `simulator 2`).
  Alles davor existiert nur als Logs, Dokumente und Skript-Kopfkommentare.
* `Temp/*.ps1` (09-30/10-01: `hr_timeline`, `hr_reject`, `hr_bias`, `zoom2`, `oblique`,
  `dir_check`, `uni_parse_test`) ist ein **Archiv** aelterer Ad-hoc-Werkzeuge, nicht im Index.
* **Offene Widersprueche (blockieren Commits zu diesen Groessen)**:
  1. Modellskala: 1,30 (10-04) gegen 1,00 (10-06) - siehe Abschnitt 6 Punkt 9.
  2. 10-11: Ausgabe haengt zu 92 % an `att`, obwohl 10-05 die pfadverankerten Quellen als
     Traeger belegt hat - ist das Rueckfall oder bewusste LIVE-Entscheidung? (zu pruefen)
  3. Mittelwert gegen Ratengrenze: B2.9 (Messung 09-30) gegen Betreiber-Wunsch 10-11
     ("gleitender Durchschnitt ist gut") - beide gelten fuer verschiedene Stoerungen, muss in
     der Filterentscheidung (D6) aufgeloest werden.
* **Naechster Schritt (Chronik, chronologisch ab dem aeltesten Beleg)**:
  1. **VWAS/Lenkwinkel**: `Log virtual WAS 2026-09-08` + die 28 `Log virtual WAS 2026-09-21 …`
     (Muster `0-re-li-0`, `20-re-li-0`, `AnschlagKali`, `Halle`, `Holperstrecke`) zusammen mit
     `_vwas.ps1`, `steering_angle_from_circle.ps1`, `WheelAngleFusion.*` und den `keya_*`-
     Dokumenten => Chronik-Zeilen fuer S6 (Nullpunktfindung, Anschlag-Kalibrierung, Hallentest).
  2. **Heading-Beginn**: `Log virtual heading 2026-09-23 …` + `log 2026-09-23 …` +
     `tools/heading_bridge/README.md` (29 Skripte; die Ergebniszeilen teilweise schon eingearbeitet).
  3. **HR-Logs 09-27 bis 10-06**: `heading_bridge_concept.md` §1...§9 (aelteste Quelle),
     `heading_bridge_report_2026-10-02.md` §1/§5/§6, `baseline_gate_findings` §3/§4,
     `skewfind` §7...§13, `skewmech` §7a...§7g, `ring` §5/§6/§9/§12, `f9p` §1/§2/§6, `imu_roll`.
  4. **10-07 bis 10-11** (die 10-10-Verweise stehen in 80_plan.md §4).
  5. Dann `tools/sim/regression.ps1` + `regression_baseline.json` ueber den Referenz-Log-Satz.
* **Bekannte Fallen**: Skripte nur mit `-ExecutionPolicy Bypass`; Sprungzahlen nur nach der
  Werkzeugkorrektur vom 10-11 verwenden; `_l1009.ps1` nicht mehr benutzen.


Quellen-Kuerzel: `10-10` = heading_findings_2026-10-10.md · `recovery` =
heading_recovery_concept_2026-10-04.md · `skewmech` = heading_skew_mechanism_2026-10-04.md ·
`baseline` = baseline_gate_findings_2026-10-03.md · `skewfind` = heading_skew_findings_2026-10-03.md ·
`konzept` = heading_bridge_concept.md · `ring` = position_ring_buffer_architecture.md ·
`imu_roll` = imu_roll_units_and_quality_2026-10-05.md · `f9p` = f9p_vs_um982_2026-10-04.md ·
`10-11` = eigene Messung, Abschnitt 5.


## 3. Konzept- und Architektur-Entscheidungen

| # | Entscheidung | Beleg |
|---|---|---|
| 3.1 | EIN Offset, identisch zu AgOpenGPS; dualHeadingOffsetDeg als echte Einstellung (Default 88,3) | konzept §12 |
| 3.2 | Ringpuffer als zentrale Positions-/Kursquelle (Records, Flags, 300 Fixes ~ 32 s) | ring |
| 3.3 | Bei Single-Antenne gilt die AOG-Schematik: IMU-Rate kurzfristig, Kurs absolut | konzept |
| 3.4 | Im Ring relatives Rauschen statt absolutem (posRelativeNoiseM 0,02; vorher 137 Grad bei 1 m ohne RTK => Schritt C still deaktiviert) | recovery (10-05) |
| 3.5 | Roll-Einheiten sind Grad (Faktor 1); "Roll nur im Stand nutzbar" wurde vom Betreiber korrigiert; Roll und Position muessen zeitlich ausgerichtet sein | imu_roll §1/§2/§6/§7 |
| 3.6 | F9P- und UM982-Regime sind NICHT gleichwertig | f9p §3 |
| 3.7 | Betreiber-Regeln 10-11: gute Datenlage = treu+schnell, sonst stetig/halten; keine Zirkelbezuege (Heading<->VWAS); IMU kein Zwang; WAS-Nullpunkt bei Keya bei jedem Start finden | 01_ziele.md §1.1 / WORKFLOW.md §8.1 |
| 3.8 | Der **Host-Simulator** baut die ECHTEN Quellen (HeadingBridge/PositionHistory-Kopien) gegen Stubs, faehrt echte Logs im Log-Zeittakt und sendet `$PAOGI` 10 Hz live an AgIO:9999 (8888 wird nicht gebunden). Er ist das Verifikationswerkzeug fuer JEDEN Schritt | simulator_concept §13...§17, `tools/sim/README.md` |


## 4. Widerlegt (nicht wiederholen)

* Festes 5-m-Fenster (Laenge muss dynamisch sein) - 10-10 §3
* Glaetten von Lenkwinkel UND Heading mit Staffel - 10-10 §3
* Naiver Vorhalt aus der Ableitung des geglaetteten Signals (Gewinn 1,00) - 10-10 §3
* Trust ueber uni_hdgsd / att_q / Baseline - 10-10 §3, §1.8
* Konstanter Montage-Offset - 10-10 §3, §1.10
* psi_att (Log-Spalte) als Wahrheit (friert ein) - 10-10 §3
* "Die Lage war eingefroren" (att lief sauber) - 10-10 §3
* "out_lim = 0 heisst Deckel 0" (0 = kein Deckel aktiv) - 10-10 §3
* "1,8...2,3 Grad Lenkwinkel-Rauschen" (Auswahl-Artefakt) - 10-10 §3, §1.11
* "IMU-Roll unbrauchbar" (vom Betreiber korrigiert) - imu_roll §0
* **Neu 10-11 (Werkzeugfehler)**: die Sprungzahl aus `_l1010.ps1` Sektion 3 war falsch skaliert
  (`allow = max(0,5*v*dt+0,15; 0,45)` => bei 21 km/h 43,8 % "Spruenge"). Korrigiert auf
  `max(1,5*v*dt+0,15; 0,40)`. Zahlen davor sind unbrauchbar (betrifft nur Sektion 3).
* **Neu 10-11 (aus den Code-Kommentaren)**: "die IMU-Roll kommt 10x zu gross an" ist
  **widerlegt** (B9.1/B9.2 - die RVC-Einheiten sind gemischt, der Rohwert ist richtig; die
  Huelle wurde am 10-05 am selben Tag zurueckgenommen). Ebenso widerlegt: eine **zweite
  Skalen-Regel** neben AOG (`countsPerDegree`/Ackermann/Nullpunkt gehoeren AOG bzw. dem VWAS,
  B7.1/B7.3/B8.39) - "zwei Lerner auf einer Groesse kaempfen".


## 5. Messbasis 10-11 (neu, `_l1010.ps1 -Filter '2026-10-11*.txt'`)

| Log | errTrack (Heading gegen Fahrtrichtung) | psiDrift | schlechte Epoch |
|---|---|---|---|
| 6-52 (n=1316) | p50 **1,40** / p90 5,70 | p50 10,2 / p90 85,7 | 7,6 % -> dort 10,0 statt 1,3 |
| 6-57 (n=566) | p50 2,30 / p90 **28,3** | p50 29,3 / p90 52,2 | 19,4 % -> dort 24,7 statt 1,6 |
| 7-01 (n=1526) | p50 **9,20** / p90 **50,4** | p50 14,2 / p90 30,4 | 24,8 % -> dort 14,6 statt 4,6 |

* **7-01 im Band 1...2 km/h (n=1188): errTrack p50 17,1 / p90 51,0** => der Betreiber-Befund
  "bei 1,5 km/h unter Baum/Bruecke schlechtes Heading + Schraegfahrt" ist in Zahlen belegt
  (17 Grad mittlere Schraeglage!). Dies ist die schaerfste Einzelzahl der drei Logs.
* 6-52 im Band 5...10 km/h (n=232): psiDrift p50 78,4 / p90 126,9 => im Arbeitsband laeuft psi weg.
* 6-52 > 10 km/h (n=849): errTrack p50 1,20 / p90 5,70 => die 21-km/h-Gerade ist im Mittel gut,
  die Verwackelung steckt in p90 und in den Spitzen (Band 3-5 km/h: p90 3,8; 5-10: p90 7,4).
* Quelle/Zustand 6-52: LIVE 92,1 % (src att 92,1 / cog 5,9 / deadreck 2,0), COAST 4,6 %, BRIDGE 2,2 %;
  6-57: LIVE 85,7 %, COAST 10,9 %. **Die Ausgabe haengt also wieder fast vollstaendig an `att`** -
  die 10-05-Architektur (projx/fix2fix/cog als Traeger) ist hier nicht mehr dominant. Pruefen:
  reicht das LIVE-Gate auch schlechte Epoch durch? (Sektion 1 zeigt: schlechte Epoch sind
  deutlich schlechter => die Qualitaetsgrenze des LIVE-Gates greift nicht hart genug.)
* Positionen (Sprungzahl NACH der Korrektur aus §4): 6-52 **4** Spruenge (0,3 %, max 1,67 m),
  6-57 2 (max 1,14 m), 7-01 2 (max **3,17 m**). Der 7-01-Sprung faellt mit `uni_hdgsd 93,69`
  (= keine Loesung) und `att_q = 0` zusammen => der Betreiber-Positionssprung ist ein
  Empfaengerausfall, kein Projektionsfehler. Der Schritt p50 in 7-01 ist 1,37 m bei 1,5 km/h,
  d.h. die publizierte Position zappelt dauerhaft ~1 m um den wahren Fix.
* Roll: 6-52 roll_src dual 1327 / 4=71 / 3=43, |roll_bridge-raw| p90 3,61 Grad; 7-01 dual 1211 / 4=339.
* Fixqualitaet 6-52: fixQ 4 = 587 von 628 *vorher* gezaehlten "Spruengen" => die waren normales
  schnelles Fahren, kein Empfaengerfehler (Werkzeugfehler, siehe §4).
* **Keine IMU in allen drei 10-11-Logs** (`imu_flags = 0`, `imurate = 0,00` in allen Frames,
  `imu_presence.ps1`) => der BNO-Saegezahn-Pfad (B2.11) ist fuer die 10-11-Symptome
  **AUSGESCHLOSSEN**; die Maschine faehrt Dual (UM982) ohne IMU.
* Es begann also eine Differentialdiagnose: S4-Kandidat (a) "LIVE-Gate reicht schlechte Epoch
  durch" bleibt - (c) IMU-Ratenpfad ist erledigt.


## 6. Noch NICHT in dieses Register eingearbeitet (offene Arbeit, nach Nutzen)

1. `konzept` (176 kB): §2/§3 Fehlermodi (Attitude-Rueckstellung, unbemerkte Garbage-Frames),
   §4 was einen Attitude-Verlust ueberlebt, §5/§6 Schwellen und Gate-Replays, §7/§9 offene Punkte.
2. `heading_bridge_report_2026-10-02.md`: §0 Heading-Runaway ohne Baeume, §1 BNO-Neigetest,
   §5 Fehleranalyse nach Evidenzgrad, §6 Verbesserungsvorschlaege.
3. `baseline_gate_findings_2026-10-03.md`: §3 Kandidaten-Simulation, §4 Empfehlung.
4. `skewfind`: §7...§13 (Betreiberantworten, Boot-Zeilen, K-Turn-Analyse).
5. `ring`: §5 Flags, §6 Sizing/RAM, §9 Migration, §12 Buildbericht.
6. `skewmech` §7a...§7g: Injektoren, "Radwinkel ist nicht schuld", UM982-only, Rangfolge.
7. `f9p` §1/§2/§6: Wahrheitstabelle, Rollverarbeitung in AOG.
8. **ERLEDIGT 10-11**: die Kopfkommentare der Werkzeuge und die **Code-Kommentare** sind
   eingearbeitet - `WheelAngleFusion.{h,cpp}` (B7.1...B7.14), **`HeadingBridge.h` vollstaendig**
   (90 Bloecke -> B8.1...B8.69) und **`HeadingBridge.cpp` zu 26 von 128 Bloecken**
   (B8.70...B8.93). Offen bleiben die restlichen ~102 `.cpp`-Bloecke (Tor-Implementierungen
   F*/R*/P*, Projektion/Resync) sowie die Werkzeug-Kommentare der aelteren `tools/analysis`-
   Skripte.
9. Widerspruch, der noch zu klaeren ist: Modellskala 1,30 (10-04) gegen 1,00 (10-06) - siehe
   80_plan.md §4. Bis das geklaert ist, gilt keine der beiden Zahlen als gesichert.
   **Nachtrag 10-11**: beide Messungen sind jetzt datiert belegt (**B8.3** aus 09-21 und
   **B8.43** aus 10-05) - die Aufloesung ist vermutlich **Nichtlinearitaet** (kleine Winkel:
   Verstaerkung zu hoch, grosse Winkel: zu niedrig).


## 7. Datei-Index (was in welcher Datei steht)

Die Themenkarte oben beantwortet "welche Frage wohin"; hier steht die Gegenrichtung –
welche Datei was beantwortet. Ziele und Zahlen: `01_ziele.md`. Plan und offene Punkte:
`80_plan.md`. Ablauf und Regeln: `WORKFLOW.md`.

| Datei | Beantwortet |
|---|---|
| `01_ziele.md` | Ziele, Akzeptanzkriterien, Messlatte je Pfad (H/P/R/VWAS) |
| `90_aog_messages.md` | PGN 221 (Hardware-Messages): Format, Katalog, Flankentriggerung, Sprachen (EN/DE im Flash, LOCAL im EEPROM), EEPROM-Block 2000, Seite `/hwmsg` |
| `95_webui_und_bridge.md` | Web-UI der AiO ueber LAN und ESP32-Bridge per UART: Alias `sc.aog`/IP-Weg, DNS/mDNS, Virtual Host, Modul-Proxy, Modulliste (PGN 203). Die Modulseite selbst ist im Modulprojekt dokumentiert (Section Control: `AOG_Section_Control_ESP32/docs/WEBSEITE_UND_NETZWERK.md`) |
| `archive/GRUNDZIELE_2026-10-11.md` | Original der Ziele-/Regel-/Plan-Sammlung (Archiv, ersetzt durch 01_ziele / 80_plan / WORKFLOW §8) |
| `heading_bridge_concept.md` (10-01) | Gesamtkonzept, Fusion, Gates, Modellgleichung, Historie |
| `heading_bridge_report_2026-10-02.md` | Messbericht Kurve/Brücke, Rauschzahlen, kritische Sektion |
| `baseline_gate_findings_2026-10-03.md` | Baseline-/Qualitäts-Gate, Kaltstart, Empfängerfelder |
| `heading_skew_findings_2026-10-03.md` | Schräglage: Identität (AOG +88), psi-Fehler = Schräglage |
| `heading_skew_mechanism_2026-10-04.md` | **psi folgt dem MODELL, die Attitude zieht nicht** (80 Grad / 20 s), R-Deckel, Re-Seed |
| `heading_recovery_concept_2026-10-04.md` | Recovery-Konzept: Projektion als Träger, Zustände |
| `position_ring_buffer_architecture.md` (10-04) | Ringpuffer, Kurslängen, Verfügbarkeit |
| `f9p_vs_um982_2026-10-04.md` | Empfänger-Vergleich, Qualitätsfelder |
| `imu_roll_units_and_quality_2026-10-05.md` | Roll-Einheiten und Roll-Qualität |
| `heading_findings_2026-10-10.md` | Arbeitsstand 10-10: Positionskurs als Wahrheit, Trust-ROC, `rl_clip` frisst Kurven |
| `simulator_concept_2026-10-11.md` | Host-Simulator (S1/S2), Replay-Messung, UDP nach AOG |
| `10_heading.md` … `70_feldprobe.md` | Themenreihe mit Befunden B… und Beleg je Zeile |
| `WORKFLOW.md` | Ablauf gegen das Im-Kreis-Drehen: 5 Tore, Invarianten, Git-Regeln, Arbeitsregeln §8 |
| `80_plan.md` | Phasenplan 0–6, offene Punkte/Widersprüche, Fix 1, Entscheidungen D1–D7 |
| `symptome.md` | Symptom → bekannte Ursachen → Prüfung: erster Ort für jeden Feldbericht |

## 8. Werkzeuge

Die Auswertewerkzeuge liegen **im Arbeits-Repository des Betreibers** (`tools/`), nicht in
diesem Firmware-Repository. Wer sie braucht, fragt nach dem Arbeitsstand; die Konzepte und
Ergebnisse sind hier dokumentiert.

| Verzeichnis | Index | Wofür |
|---|---|---|
| `tools/analysis` | `tools/analysis/README.md` | HR-Rohlogs (10 Hz): Fahrten, Sprünge, Schräglage, Modell |
| `tools/heading_bridge` | `tools/heading_bridge/README.md` | HBUF-Dumps: Gate/Filter/Modell, Kurven-Proportionalität |
| `tools/sim` | `tools/sim/README.md` (+ Konzept `simulator_concept_2026-10-11.md`) | Host-Simulator: echte Quellen gegen Logs, `$PAOGI` live nach AgIO (9999) |

Standardaufruf (Pfade relativ zum Arbeits-Repository, Skripte immer mit
`-ExecutionPolicy Bypass`, weil LocalMachine auf dem Betreiber-Rechner `Restricted` ist):

```powershell
. .\tools\analysis\HrLog.ps1
$d = Read-HrLog '2026-10-11 6-52 0 r hof bach brücke 90 failed Gerade 21kmh verwackelt.txt'
powershell -NoProfile -ExecutionPolicy Bypass -File tools\analysis\_l1010.ps1 -Filter '2026-10-11*.txt'
powershell -NoProfile -ExecutionPolicy Bypass -File tools\heading_bridge\curve_proportionality.ps1 -Log '<log>'
```

Neue Fahrt: **`-Filter` des vorhandenen Werkzeugs** – keine neue Dateikopie (die alten
`_l1009.ps1`/`_l1010.ps1` waren genau solche Kopien, `_l1009.ps1` ist ersetzt).

## 9. Datenlage: Logs und Betreiber-Befunde

**Quellenlage (geprüft 10-11)**: 128 datierte Fahrten vom **2026-09-08 bis 10-11** in drei
Familien – VWAS-Kalibrierung (`Log virtual WAS …`, 09-08 und 09-21), Heading-Beginn
(`Log virtual heading …` / `log 2026-09-23 …`), HR-Logs ab 09-27. **Kein Material von Juni bis
August 2026**, Git-Zeitleiste erst ab 10-04. Details: §0 oben.

| Log | Befund des Betreibers |
|---|---|
| 10-09 (3 Logs) | schräges Heading, leichte Schrägfahrt, Sprünge bei schlechtem Empfang |
| 10-10 12-31 | 90-Grad-Kurve klappt nicht, unter Baum/Brücke schlecht |
| 10-10 12-36 | Heading teils sehr schlecht, Positionssprung |
| 10-10 15-46 | Heading lost (Hof/Bach/Brücke) |
| **10-11 6-52** | 90-Grad-Kurve FALSCH + Sprünge am Kurvenende; Gerade 21 km/h verwackelt; Brücke total off (Ende fast im Stand: Sprung) |
| **10-11 6-57** | Brücke 6 km/h: Heading verloren + Positionssprung |
| **10-11 7-01** | Baum/Brücke 1,5 km/h: Heading schlecht, Schrägfahrt, **Lenkwinkel 5,6 Grad off** |

