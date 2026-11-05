> **ARCHIV (2026-11-03).** Diese Datei ist die unveraenderte Originalsammlung "Grundziele und
> Arbeitsweise" vom 2026-10-11. Sie ist aufgeteilt in `01_ziele.md` (Ziele/Messlatte),
> `80_plan.md` (Phasen, offene Punkte, D1-D7), `WORKFLOW.md` §8 (Arbeitsregeln, kein
> Zirkelbezug, vier Regime) und `00_ueberblick.md` §7-§9 (Datei-Index, Werkzeuge, Datenlage).
> Nur noch als Beleg lesen - gepflegt werden die neuen Dateien.

# Grundziele und Arbeitsweise (AiO Heading/Position-Pfad)

**Diese Datei zuerst lesen.** Sie sagt, was gebaut wird, woran gemessen wird, welche
Werkzeuge und Logs es gibt und welche Fragen schon beantwortet sind.
Stand 2026-10-11. Ergaenzungen nur mit Datum + Log-Bezug.

## 1. Ziel des Ganzen

Die AiO-Box liefert AgOpenGPS (AOG) **Position und Fahrtrichtung**, AOG lenkt damit.
Es gibt keine IMU als Richtungsquelle: die Dual-Antenne (UM982) ist die absolute
Richtung, der Positionskurs die absolute Gegenprobe, der Lenkwinkel das kurzfristige Modell.

Akzeptanz ist der Feldbericht des Betreibers, in Zahlen uebersetzt:

| Betreiber sagt | Messgroesse | Zielwert |
|---|---|---|
| "faehrt schraeg" | sichtbare Schraeglage = `travel - (az_out+88)` = psi-Fehler | p90 < 2 Grad auf Geraden |
| "Heading verwackelt" (Gerade) | `p99 \|dHeading\| / dt` | **<= 1 Grad/s** - jede GPS-Rate, auch bei schlechtem Empfang |
| "Heading verwackelt" (Treue) | `p95 \|Heading - Positionskurs (2-s-Fit)\|` | **<= 0,5 Grad** (gute Datenlage) |
| "Kurve wird nicht gefahren" | im Heading verlorene Grad je Kurve | < 5 Grad bei 90-Grad-Kurve |
| "Positionssprung" | Schritt der publizierten Position gegen `v*dt` | kein Schritt > 0,15 m im Fahren |
| "Roll springt" | Sprung in `roll_bridge_deg` | < 1 Grad je Frame |
| "Heading verloren/falsch" | `|az_out+88-travel|` | < 5 Grad, Dauer < 1 s |

Arbeitsbereich: **0,2...20 km/h** (Arbeitsfall 5...10 km/h), Daten können bis **60 km/h**
kommen, enge 90-Grad-Kurven, Rueckwaerts/K-Turn, Stand und Schleichfahrt.
Diese Bereiche sind die Prueffaelle, nicht die Ausnahme.

### 1.0 Ziel in physikalischen Einheiten (2026-10-12, GPS-Rate-unabhaengig)

Alle Ziele gelten in **Grad pro Sekunde bzw. Grad**, NICHT pro Frame: die Logs kommen mit
**5, 10 oder 20 Hz**, ein "je Frame"-Kriterium verschiebt sich dadurch um Faktor 2 oder 4.
(Die alte Zeile "sd je 0,1 s (10 Hz)" war genau das - ersetzt 10-12.)

**Warum diese Zahlen physikalisch sind**
* **0,5 Grad** ist das Eigenrauschen der Dual-Antenne (0,71 m Basis, `uni_hdgsd` gemessen
  0,4...0,6 Grad) - das Ziel heisst: *nicht schlechter als die Hardware*.
* **1 Grad/s** ist die reale Drehrate eines GERADE fahrenden Traktors bei 2...4 m/s mit
  ~0,5 Grad Lenkwinkel (`57,3*v*tan(delta)/L` = 0,4...0,8 Grad/s) - "gerade" heisst
  physikalisch fast keine Drehung.

**Kurven-Regel** (Betreiber 10-12: "wie gehen wir in Kurven damit um"): die Grenze ist KEINE
feste 1 Grad/s, sondern
`zulaessig = max(1 Grad/s, k * omega_phys)`, `omega_phys = 57,3*v*tan(|delta_eff|+slack)/L`
mit dem **trim-korrigierten** Lenkwinkel (B7.21/B10.24). Auf der Geraden greift der Boden
1 Grad/s, in der Kurve oeffnet die Physik (bei 15 Grad Lenkwinkel und 4 m/s: ~25 Grad/s).
**Keine Bestaetigungs-Verzoegerung in Kurven** - dort ist die Aenderung durch die Physik
gedeckt und darf sofort durch; eine Verzoegerung macht aus der Kurve ein Nachlauf-Problem.

**Schlechter Empfang** (Betreiber 10-12: "das Ziel muss auch bei schlechtem Empfang gelten,
sonst schaukelt sich der Traktor auf"): das BAND (0,5 Grad) ist dort NICHT haltbar - die
Quelle selbst ist mehrere Grad falsch. Deshalb ist die **Rate die HARTE** und das **Band die
QUALITAETS**-Forderung:
* **HART (immer, auch bei Ausfall)**: die Ausgabe springt nie - `p99 |dHeading|/dt` <= 1 Grad/s
  (Gerade) bzw. <= der Kurven-Grenze. Begruendung aus der Messung 10-12: bei `att_q = 0`
  Sprunge bis 60 Grad, dazu AOG-Zielwinkel sd 6,2 Grad - das ist das Aufschaukeln.
* **QUALITAET (gute Datenlage)**: `p95 |Heading - Positionskurs (2-s-Fit)|` <= 0,5 Grad.
* **In schlechten Phasen**: nicht dem Unsinn folgen, sondern **nahe am letzten vertrauten Kurs
  bleiben** (driftet hoechstens mit der Rate), und beim Wiedereintritt **als Rampe** nachfuehren
  (Rate <= dieselbe Grenze, nie ein Sprung).

**Wahrheit fuer die Kennzahl**: der **Positionskurs als 2-s-Fit** ueber `lat/lon` (Prinzip
siehe unten, "Positionskurs = absolute Wahrheit"). Die Einzelframe-Ableitung `travel` taugt
NICHT als Wahrheit - sie ist selbst zu verrauscht (gemessen 10-12: p90 14...21 Grad/s).

**Umsetzungs-Konsequenz (GPS-Rate)**: alle Schwellen kuenftig in **Grad/s und ms** speichern
und mit `dt` umrechnen. Heute noch FRAME-basiert und damit rate-abhaengig:
`Config::outSnapDeg` (3 Grad - bei 5 Hz sind das 15 Grad/s, bei 20 Hz 60 Grad/s!),
`Config::outSnapConfirmFrames` (1 Frame = 100 ms bei 10 Hz, 200 ms bei 5 Hz),
`Config::outNoiseSamples` (5 Frames = 0,5 s bei 10 Hz). Schon zeitbasiert: `outSmoothMs`,
`outHoldMaxMs`, `liveRampDegPerS`, `outResidRateFactor`.

### 1.1 Betreiber-Festlegungen (2026-10-11) - verbindliche Messlatte

**Vorrang bei Zielkonflikt** (woertlich): *"treu, wenn die Datenlage sicher ist = guter Empfang,
Empfänger eingeschwungen, sonst stetige Werte, keine Sprünge bei Roll, Heading und Position
(wobei die Position bei schlechtem Empfang meist noch gut ist), möglichst verzögerungsfrei vom
GPS zu AOG. Bei Ausfall eine robuste Überbrückung, bis GPS wieder da ist."*

Also kurz: **gute Datenlage -> treu UND schnell. Schlechte Datenlage -> stetig, halten, nie
springen.** Treue ist ein Vorrecht guter Daten, nicht eine Filtereigenschaft.

**Weitere Festlegungen:**

* **Tempo**: Arbeitsfall **5...10 km/h**, Band 0,2...20 km/h, Daten bis 60 km/h.
* **IMU**: bei **Single** (F9P + IMU) fast immer vorhanden; bei **Dual** (2x F9P oder UM982)
  selten, aber möglich. Der Kernpfad muss **ohne** IMU funktionieren und **mit** IMU besser werden.
* **Lenkwinkel**: echter Sensor (analog), Traktor-CAN, oder **virtuell aus dem Keya-Lenkradmotor
  (eingebauter Encoder)**. Der Keya-Encoder startet bei **jedem Neustart bei 0**, egal wie die
  Raeder stehen -> **die Null muss gefahren/gefunden werden**.
* **Ausgabe-Ideal**: *"sauber lenken, aber nahe an den GPS-Daten bleiben, wenn diese gut sind.
  Sonst kommt es zu Schwingungen. Im Idealfall GPS live weitergeben."* Fuer das Heading ist ein
  **gleitender Durchschnitt** gut, weil die UM982 Heading-Rauschen mitbringt.
* Umbau: Patches **und** groessere Schritte sind erlaubt; Plan gemeinsam besprechen.
* "Fertig" definiert sich ueber den Plan; danach faehrt der Betreiber Probe und entscheidet.


## 2. Wahrheiten (die Messlatte)

1. **Positionskurs = absolute Wahrheit.** 0,5...5 m Sehne aus `lat/lon`
   (krummungskorrigiert um den Chord-Nachlauf `0,5*w*T`), unabhaengig von Attitude, IMU,
   Lenkwinkel und Modell. Gemessen 10-10: 0,10...1,11 Grad gegen Modell 13,9...29 Grad
   ueber 11...29 s Ausfallluecken.
2. **Attitude (`att_hdg_deg`, Dual) = Karosserie-Zeuge.** Wird nie gespiegelt und nie von
   uns erfunden; ihr Zuwachs beweist eine Drehung. Der COG ist der zweite, unabhaengige Zeuge.
3. **`psi` ist ZUSTAND, keine Messung.** Folgt psi dem Modell und zieht die Attitude nicht,
   ist psi falsch - direkt messbar als `psi_att` (muss konstant bleiben).
4. **Die eigene publizierte (Projektions-)Position ist NIE Kursquelle** (5,22 statt 0,10 Grad).
5. **Konvention**: `az_out` verlaesst das Modul, AOG addiert den Montage-Offset (+88) selbst
   -> die in AOG sichtbare Richtung ist `az_out+88`. `psi`, `travel`, `cog`, `att`
   sind AOG-Frame-Groessen.

## 3. Arbeitsregeln

* Erst messen, dann behaupten. Jede Aussage mit Log, Zeitstempel, Spalte.
* **Vorhandene Werkzeuge benutzen und erweitern** (Karte in §5), kein neues Skript pro Fahrt:
  neue Fahrt = `-Filter` des vorhandenen Auswertewerkzeugs.
* Ergebnis in `knowledge/` mit Datum; Firmware-Entscheidung als Messbeleg im Code-Kommentar.
* Widerspruch zwischen zwei Messungen wird benannt und aufgeloest, nicht geglaettet (§7).
* Firmware bleibt baubar (`pio run -e teensy41`); RAM1/RAM2 im Blick.
* **Jede Aenderung wird im Simulator verifiziert** (`tools/sim`, echte Quellen, live nach AgIO
  ueber `$PAOGI`): messen -> aendern -> denselben Log erneut fahren -> virtuelle Ausgabe **und**
  AOG-Anzeige vergleichen -> dann Feldprobe. Ablauf: `tools/sim/README.md`.

### 3.1 Harte Architektur-Regel: KEIN Zirkelbezug

Betreiber-Festlegung 10-11: *"wir muessen aufpassen, dass es keinen Zirkelbezug aus Heading fuer
VWAS und VWAS -> simuliertes Heading/Projektion gibt."* Daraus vier Regeln:

1. **`psi` (unser Filterzustand) ist NIE die veroeffentlichte Quelle**, wenn eine Messung
   vorliegt. Veroeffentlicht wird die **lagegefuehrte** Koerperrichtung (gute Datenlage) oder
   der **positionskursgefuehrte** Wert (schlechte Datenlage).
2. **Kursquelle ist nur der Rohfix** (`lat/lon`), NIE die eigene publizierte Projektion.
3. **Der WAS-Nullpunkt wird aus Rohpositionen gelernt** (Kruemmung gegen Lenkwinkel), nie aus
   Heading oder `az_out`.
4. **Der Lenkwinkel ist nie die einzige Quelle des veroeffentlichten Headings.** Genau das
   passiert, wenn `psi` rausgeht: WAS -> psi -> Heading -> (AOG lenkt) -> WAS schliesst sich
   zum Kreis und eine 5,6-Grad-Schieflage wird zu einer dauerhaften Drehung.

### 3.2 Vier Regime statt einer Kette

| Datenlage | Traeger (absolut) | Kurzzeit | Ausgabe |
|---|---|---|---|
| Empfaenger gut und **eingeschwungen** | Attitude | Attitude | **live**, nur kurz gemittelt |
| Lage zweifelhaft, Positionen gut (v > 0,5 m/s) | **Positionskurs** (Ring 2...5 m, krummungskorrigiert) | Modell (nur Gegenprobe) | kursgefuehrt, stetig |
| Positionen gut, sehr langsam (5 m dauern > 10 s) | letzter guter Kurs | Modell | halten / rampen |
| alles schlecht (Bruecke, Ausfall) | letzter Kurs + Modellrate, auf die physikalische Rate begrenzt | Modell | halten, keine Spruenge |

Begruendung aus den Logs: Positionskurs 0,10...1,11 Grad, Modell 13,9...29 Grad ueber 11...29 s
Ausfallluecken (10-10); Dual-Heading jittert 2...6 Grad/Frame, der Empfaenger meldet 0,32...0,55
(Faktor 10 zu optimistisch) -> gleitender Durchschnitt im Gutfall, Rate-Limit bei Spikes.


## 4. Wissenskarte - was schon beantwortet ist

| Datei | Beantwortet |
|---|---|
| `heading_bridge_concept.md` (10-01) | Gesamtkonzept, Fusion, Gates, Modellgleichung, Historie |
| `heading_bridge_report_2026-10-02.md` | Messbericht Kurve/Bruecke, Rauschzahlen, Kritische Sektion |
| `baseline_gate_findings_2026-10-03.md` | Baseline-/Qualitaets-Gate, Kaltstart, Empfaengerfelder |
| `heading_skew_findings_2026-10-03.md` | Schraeglage: Identitaet (AOG +88), psi-Fehler = Schraeglage |
| `heading_skew_mechanism_2026-10-04.md` | **psi folgt dem MODELL, die Attitude zieht nicht** (80 Grad / 20 s), R-Deckel, Re-Seed |
| `heading_recovery_concept_2026-10-04.md` | Recovery-Konzept: Projektion als Traeger, Zustaende |
| `position_ring_buffer_architecture.md` (10-04) | Ringpuffer, Kurslaengen, Verfuegbarkeit |
| `f9p_vs_um982_2026-10-04.md` | Empfaengervergleich, Qualitaetsfelder |
| `imu_roll_units_and_quality_2026-10-05.md` | Roll-Einheiten und Roll-Qualitaet |
| `heading_findings_2026-10-10.md` | **Arbeitsstand 10-10**: Positionskurs als Wahrheit, Trust-ROC, `rl_clip` frisst Kurven |
| `simulator_concept_2026-10-11.md` | Host-Simulator (S1/S2), Replay-Messung, UDP nach AOG |
| **`00_ueberblick.md`** | **Register aller Befunde mit Beleg + Status** (BEWIESEN/WIDERLEGT/OFFEN) - verhindert Kreisverkehr; oben ein STAND-Block |
| **`WORKFLOW.md`** | **Ablauf gegen das Im-Kreis-Drehen**: 5 Tore, Invarianten, Git-Regeln, Kennzahlen-Panel |
| **`symptome.md`** | **Symptom -> bekannte Ursachen -> Pruefung**: erster Ort fuer jeden Feldbericht |
| `GRUNDZIELE.md` (diese Datei) | Ziele, Messlatte, Regeln, Werkzeug-/Log-Karte |

## 5. Werkzeugkarte

| Verzeichnis | Verzeichnis-Index | Wofuer |
|---|---|---|
| `tools/analysis` | `tools/analysis/README.md` | HR-Rohlogs (10 Hz): Fahrten, Spruenge, Schraeglage, Modell |
| `tools/heading_bridge` | `tools/heading_bridge/README.md` | HBUF-Dumps: Gate/Filter/Modell, Kurven-Proportionalitaet |
| `tools/sim` | `tools/sim/README.md` (+ Konzept `knowledge/simulator_concept_2026-10-11.md`) | Host-Simulator: echte Quellen gegen Logs, `$PAOGI` live nach AgIO (9999) |

Standards:
```powershell
. .\tools\analysis\HrLog.ps1
$d = Read-HrLog '2026-10-11 6-52 0 r hof bach brücke 90 failed Gerade 21kmh verwackelt.txt'
# Skripte immer mit -ExecutionPolicy Bypass (LocalMachine ist Restricted auf diesem Rechner)
powershell -NoProfile -ExecutionPolicy Bypass -File tools\analysis\_l1010.ps1 -Filter '2026-10-11*.txt'
powershell -NoProfile -ExecutionPolicy Bypass -File tools\heading_bridge\curve_proportionality.ps1 -Log '<log>'
```
Neue Fahrt: **`-Filter` des vorhandenen Werkzeugs** - keine neue Dateikopie (die alten
`_l1009.ps1`/`_l1010.ps1` waren genau solche Kopien, `_l1009.ps1` ist ersetzt).

## 6. Logs und Betreiber-Befunde

**Quellenlage (geprueft 10-11)**: 128 datierte Fahrten vom **2026-09-08 bis 10-11** in drei
Familien - VWAS-Kalibrierung (`Log virtual WAS …`, 09-08 und 09-21), Heading-Beginn
(`Log virtual heading …` / `log 2026-09-23 …`), HR-Logs ab 09-27. **Kein Material von Juni bis
August 2026** im Repository, Git-Zeitleiste erst ab 10-04. Details: `00_ueberblick.md` §0.

| Log | Befund des Betreibers |
|---|---|
| 10-09 (3 Logs) | schraeges Heading, leichte Schraegfahrt, Spruenge bei schlechtem Empfang |
| 10-10 12-31 | 90-Grad-Kurve klappt nicht, unter Baum/Bruecke schlecht |
| 10-10 12-36 | Heading teils sehr schlecht, Positionssprung |
| 10-10 15-46 | Heading lost (Hof/Bach/Bruecke) |
| **10-11 6-52** | 90-Grad-Kurve FALSCH + Spruenge am Kurvenende; Gerade 21 km/h verwackelt; Bruecke total off (Ende fast im Stand: Sprung) |
| **10-11 6-57** | Bruecke 6 km/h: Heading verloren + Positionssprung |
| **10-11 7-01** | Baum/Bruecke 1,5 km/h: Heading schlecht, Schraegfahrt, **Lenkwinkel 5,6 Grad off** |

## 7. Offene Punkte und Widersprueche

1. **90-Grad-Kurve** - teilgefixt, zwei getrennte Ursachen:
   * *Ursache 1 (gefixt)*: `rawAz` fiel auf `psi` zurueck. Im 10-09-Log drehte die Lage 176 Grad,
     publiziert wurden -95 Grad, bei `out_lim = 0` (also ohne aktiven Deckel). Fix A+B2:
     `rawAz` nimmt IMMER die Dual-Lage, wenn `hasDualHeading && qfOk`.
   * *Ursache 2 (offen)*: der Pfad-Begrenzer frisst echte Kurven. 10-10 gemessen:
     141...151 Grad verloren, in genau diesen Episoden `rl_clip` 24...51.
     Vorschlag 1b: `limit = max(Boden, 1,5*|w_modell|)`, auf den Kurs gegated
     (Gegenrechnung: verlorene Grad p50 1,8 / max 4,2 statt 151; Kosten 3,78 % Fehlausnahmen).
2. **Widerspruch Modellskala** (ungeloest, muss zuerst geklaert werden):
   * 10-04: "Modell dreht in engen Kurven 1,4...1,7x zu weit" -> L_eff 2,14 m -> 2,95 m (Fac 1,30).
   * 10-06: "mit Fac 1,30 rechnet das Modell 23 % zu WENIG Gierrate, psi hinkt 29 Grad nach,
     die 30-Grad-Notbremse feuert 2x" -> zurueck auf `effWheelbaseFac = 1.00` (so steht es heute im Code).
   Beide Messungen sind dokumentiert. Vermutung: verschiedene Wahrheit (path/COG vs. `att`)
   oder verschiedene Epochen (Roll/Schlupf). Naechster Schritt: eine Messung, eine Wahrheit,
   neues Log mit korrekt gefahrenen 90-Grad-Kurven.
3. **Lenkwinkel-Versatz 5,6 Grad** (Betreiber 10-11 7-01): bei 0,42 m/s ergibt das
   `v*tan(5,6 Grad)/L_eff` ~0,9 Grad/s falsche Gierrate -> 19 Grad in 20 s. Passt genau zum
   "Heading unter der Bruecke falsch" bei 1,5 km/h. Zur Messung: p50 des Lenkwinkels auf
   geraden, sauberen Epochen (Soll 0) und `yr` gegen die Kursrate.
4. Aus dem Arbeitsstand 10-10 offen: **1a** psi nie als Ausgangsquelle (Fallback = Positionskurs),
   **2** Trust = Lage gegen 5-m-Kurs, **3** Offset laufend schaetzen statt festschreiben.

## 8. Vorgehensplan (Entwurf 2026-10-11, zur Besprechung)

Leitsatz: **Messen -> Regel -> kleiner Fix -> im Sim pruefen -> Feldprobe -> festschreiben.**
Jeder Schritt nennt Beleg, **vorhandenes** Werkzeug und ein pruefbares Abnahmekriterium.
Die Phasen sind nach Feldnutzen geordnet, nicht nach Eleganz.

### Phase 0 - Messen (kein Code, sofort)
Die drei 10-11-Logs auswerten und die sechs Befunde in Zahlen festhalten: errTrack (Heading
gegen Fahrtrichtung), Kurvenverlust in Grad, Positionsspruenge, Fehler je Geschwindigkeitsband,
Lenkwinkel-p50 auf geraden sauberen Epochen (Soll 0), Roll-Weg.
Werkzeug: `_l1010.ps1 -Filter '2026-10-11*.txt'`, `_hsources.ps1`, `_straight.ps1`, `_vbins.ps1`.
Abnahme: Tabelle "Betreiber-Befund -> Zahl" als Baseline. **Ohne diese Zahlen kein Fix.**

### Phase 1 - Zirkelbezug brechen (klein, Architektur nach 3.1)
* Ausgabe fuehrt die **Lage**, wenn sie vertrauenswuerdig ist; sonst der **Positionskurs**.
  `psi` geht nie mehr ungeprueft raus (A+B2 ist der Anfang, der Kurs-Fallback fehlt noch).
* Kursquelle = Rohfix; Ringpuffer darf nie die publizierte Projektion fuettern.
* WAS-Nullpunkt-Berechnung nur aus Rohpositionen.
Werkzeug: Code-Audit + Sim-Szenario "WAS-Nullpunkt 5,6 Grad falsch" (Heading bleibt richtig,
weil die Lage fuehrt), `_rec*`-Spuren im Log.
Abnahme: Audit-Liste "wer darf wen beeinflussen" ohne Rueckkante; Sim-Szenario besteht.

### Phase 2 - Kurven nicht mehr fressen (groesster Einzelhebel)
Manoeverterm am Ausgangsbegrenzer: `limit = max(Boden, 1,5*|w_modell|)`, **nur freigegeben,
wenn der Kurs die Kurve bestaetigt** (sonst Fehlausnahmen). Vorher `rl_allowed`-Semantik pruefen
(0 = kein Limit?) - die alte Verwechslung hat schon einmal in die Irre gefuehrt.
Beleg: 10-10 141...151 Grad verloren, in genau diesen Episoden `rl_clip` 24...51;
Gegenrechnung p50 1,8 / max 4,2 verlorene Grad, Kosten 3,78 % Fehlausnahmen.
Werkzeug: `_sim_manoeuvre.ps1` (vorher), `_l1010.ps1` (nachher), `_oldturns.ps1`.
Abnahme: 90-Grad-Kurve < 5 Grad verloren (Sim und Log), Fehlausnahmen < 5 % der Frames.

### Phase 3 - Ausfall robust ueberbruecken (Bruecke/Baum, keine Spruenge)
Positionskurs als Traeger, Modell nur als Kurzzeitrate; Rampen beim Re-Seed und beim
Gate-Schliessen; Guard gegen Wegdrehen (das Modell darf nicht gegen einen bestaetigenden Kurs
drehen); Roll halten statt springen.
Werkzeug: `_spin.ps1`, `_reentry.ps1`, `_gapdump.ps1`, `fuse.ps1`, `_jumps.ps1`.
Abnahme: 11...29 s Ausfall -> Drift < 5 Grad; Wiedereintritt ohne Schritt > 2 Grad;
kein Positionsschritt > 0,15 m im Fahren.

### Phase 4 - Gute Datenlage schnell und treu (Verwackeln und Latenz)
Trust-Praedikat "Empfaenger eingeschwungen" aus **Lage gegen 5-m-Kurs** (gerade Epochen,
Hysterese, Mindestdauer) -> dann Ausgabe = lagegefuehrte Koerperrichtung mit **kurzem**
gleitendem Durchschnitt, dessen Bandbreite an die **physikalisch moegliche Rate** gekoppelt ist
(gerade = stark mitteln, Kurve = nachfuehren). Damit ist "treu wenn gut" und "stetig" kein
Widerspruch. Die heute nach Tempo/Lenkwinkel gestaffelte Filterstaerke wird auf **Datenlage**
umgestellt (Betreiber: gleitender Durchschnitt, weil UM982 Rauschen bringt).
Werkzeug: `_sim_trust.ps1`, `_noise.ps1`, `_band57.ps1`, `_lag.ps1`, `filter_compare.ps1`.
Abnahme (2026-10-12 in physikalische Einheiten gefasst, siehe 1.0): auf Geraden
`p99 |dHeading|/dt <= 1 Grad/s` (jede GPS-Rate, auch bei schlechtem Empfang) und
`p95 |Heading - Positionskurs (2-s-Fit)| <= 0,5 Grad` bei guter Datenlage; in Kurven
`p99 |dHeading|/dt <= max(1, k*omega_phys)`; Latenz < 150 ms (Kreuzkorrelation); im Gutfall
folgt die Ausgabe der Lage live. Werkzeuge dafuer: `_band57.ps1`, `_lag.ps1`, `_noise.ps1`.

### Phase 5 - Roll und Lenkwinkel-Nullpunkt (Keya)
Roll getrennt gaten (keine Spruenge > 1 Grad/Frame, `roll_susp` bleibt); Keya-Nullpunkt aus
Rohpositions-Kruemmung lernen, Plausibilitaetsband, **nicht im Stand** lernen (im Stand gibt es
keine Information).
Werkzeug: `roll_check.ps1`, `rollout.ps1`, `steering_angle_from_circle.ps1`, `_kfit.ps1`, `circles.ps1`.
Abnahme: Lenkwinkel p50 auf Geraden < 1 Grad; `L_implied` ueber die Winkelbins konstant.

### Phase 6 - Abnahme
Sim mit je einem Szenario pro Regime (Ausfall, 90-Grad-Kurve, 21-km/h-Gerade, Bruecke,
Schleichfahrt) -> Feldprobe durch den Betreiber -> Ergebnis in `knowledge/` festschreiben und
die Schwellen nachziehen.

### Was wir NICHT machen
* keine neue Hardware voraussetzen; **kein IMU-Zwang** im Kernpfad (Single: IMU nutzen, Dual: ohne);
* die eigene Projektion nie als Kursquelle, das Heading nie als WAS-Nullpunkt-Quelle;
* keine ungemessene "Verbesserung"; jede Aenderung mit Zahl vorher/nachher;
* kein grosser Umbau vor der ersten Feldprobe (kleine, pruefbare Schritte);
* kein neues Werkzeug je Fahrt - `-Filter` des vorhandenen.

## 10. Fix 1 (beschlossen 10-11) - Ausgangsquelle und Trust

**Ziel als Zahlen. Die "heute"-Spalte ist gemessen, die Zielspalten sind die VORHERSAGE
(Workflow Tor 2: kein Fix ohne Vorhersage).**

**Reihenfolge korrigiert (Code-Lesung 10-11, siehe B10.9):** die Kette
`att -> Kurs -> psi -> letzter Wert` IST bereits implementiert; der wirksame Hebel ist das
**KRITERIUM**, nicht die Reihenfolge. Daher:
**1.2a** zuerst (die Lage fuer die AUSGABE gegen den PFAD pruefen, statt nur ueber die
Empfaenger-Flags `qfOk`/`baseLen`/`settle`), dann **1.2b** (dasselbe Kriterium fuer die
ANNAHME/den Anker), dann **1.1** (den `psi`-Schritt entfernen bzw. absichern).

| Kennzahl (10-11-Logs) | heute | Ziel nach 1.2a | Ziel nach 1.2b | Ziel nach 1.1 |
|---|---|---|---|---|
| 7-01 errTrack p50 (1...2 km/h, Bruecke) | **17,1** | < 5 | < 3 | < 3 |
| 6-57 errTrack p90 (6 km/h, Bruecke) | 28,3 | < 15 | < 10 | < 10 |
| 6-52 >10 km/h p50 / p90 | 1,2 / 5,7 | unveraendert | unveraendert | unveraendert |
| Kurvenverlust \|p50\| (drei Logs) | 0,19 / 2,42 / 0,18 | unveraendert | unveraendert | unveraendert |
| Positionsschritte > 0,15 m (drei Logs) | 4 / 2 / 2 | unveraendert | unveraendert | unveraendert |
| Frames "\|Lage als Wahrheit unbrauchbar" (6-52) | 1232 | soll fallen | soll fallen | soll fallen |

**1.2a Das KRITERIUM fuer die Ausgabe (erster Schritt)**: statt `attOk = hasDualHeading && qfOk`
gilt `attOk = hasDualHeading && !attJumped && lagePasstZumPfad`, mit
`lagePasstZumPfad = |wrap(att + Offset - Pfad)| <= attPathTolDeg` (Vorschlag 25 Grad) und
`Pfad` = `travelWitnessDeg` (Positionen) wenn gueltig, sonst der Fix-zu-Fix-Kurs; ist KEIN Pfad
verfuegbar (Schleichfahrt unter 0,3 m/s), bleibt `qfOk` das Kriterium wie heute.
Das harte kinematische Tor (`attJumped`, A+B4) bleibt unveraendert.
**1.2b Dasselbe Kriterium fuer die ANNAHME** (Anker/Gate in `evaluateAttitudeFrame`).
**1.1 `psi` aus der Ausgabe entfernen** (letzter Wert halten bzw. Kurs) - nach 1.2a/1.2b pruefen,
ob der Schritt noch gebraucht wird.

**Ausgangsquelle heute** (A+B2, `HeadingBridge.cpp:2364-2376`):
`rawAz = attOk ? att : (travelWitness gueltig ? Kurs : (psiOk ? psi : letzter Wert))`.


**1.1 Ausgangsquelle - `psi` faellt als Ausgabe weg** (Prioritaetskette, ersetzt das "sonst psi"
aus A+B2):
1. vertraute Lage (`att + Offset`) -> live durchreichen (wie heute),
2. Lage vorhanden, aber gerade verworfen (`baseLen`, `settle`, genullt) -> **letzte GUTE Lage
   halten** (ratenbegrenzt, mit den vorhandenen Rampen) - **nicht** `psi`,
3. keine Lage (Single-Antenne, Kaltstart) -> **Kurs aus den Positionen** (`travelWitnessDeg`),
4. nur wenn auch das fehlt: der letzte publizierte Wert (wie heute).

Belege: B10.3 (bei 0,3...0,6 m/s lag `att+offset` -1,8...+3,1 Grad auf dem Pfad, `psi` 11...21
Grad daneben, und AOG bekam `psi`), B10.4 (die Lage wurde als `baseLen`/`settle` verworfen,
obwohl sie passte), B10.8 (in 1232 von 1440 Frames war die Lage als Wahrheit unbrauchbar ->
der Rueckfall muss die GEHALTENE LAGE sein, nicht der Zustand).

**1.2 Trust gegen den PFAD**:
`lageVertraut = |wrap(att + Offset - Pfad)| <= 20 Grad` ueber ein kurzes Fenster, mit Hysterese;
der Pfad kommt aus den **Positionen** (Fix-zu-Fix bzw. 2...5 m Kurs) und ist damit vom Zustand
unabhaengig. Das ersetzt/ergaenzt `att_q`, `uni_hdgsd`, `baseLen` und `settle`.
Beleg: B10.5 (24 % / 42 % / 92 % falsche Epochen durchgelassen UND eine gute Lage in 7-01
verworfen).

**Blast-Radius, der nicht brechen darf:** Kaltstart/INIT, Single-Antenne (kein Dual),
Rueckwaerts-Spiegelung (Vorderachse!), Roll, Position, die >10-km/h-Geraden und die Kurvenantwort.

**Ablauf:** 1.1 implementieren -> Sim + die drei Referenz-Logs messen -> Commit `[B10.3]`;
danach 1.2 genauso -> Commit `[B10.5]`; dann `tools/sim/regression.ps1` + Baseline ueber den
Referenz-Log-Satz, damit diese Zahlen ab dann automatisch geprueft werden.


| # | Frage | meine Empfehlung |
|---|---|---|
| D1 | Vorrang-Regel so richtig: gute Datenlage = treu+schnell, schlechte = stetig+halten? | ja, genau so umsetzen |
| D2 | Darf im Gutfall die **Attitude** die Ausgabe fuehren (statt `psi`)? | ja - das ist der Bruch des Zirkelbezugs |
| D3 | Reihenfolge: erst Kurven (Phase 2) oder erst Live/Ruhe (Phase 4)? | erst Kurven, dann Ausfall, dann Live |
| D4 | Kurslaenge dynamisch 2...5 m, krummungskorrigiert, aus Rohfixes? | ja |
| D5 | WAS-Nullpunkt aus Rohpositionen (Keya-Start bei 0) - wie dringend? | Phase 5, nach Kurven/Ausfall |
| D6 | Filterstaerke nach **Datenlage** statt nach Tempo/Lenkwinkel staffeln? | ja (Betreiber: gleitender Durchschnitt ist gut) |
| D7 | Gilt "verzoegerungsfrei" auch fuer die Position (Projektion nur als Notnagel)? | ja, Projektion <= 0,5 m begrenzt |



