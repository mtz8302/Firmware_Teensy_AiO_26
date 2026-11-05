# 80 Plan: Phasen, offene Punkte, Entscheidungen

> **Summary (EN).** The working plan, explicitly marked as a **plan, not as truth**.
> Guiding rule: measure → change one thing → verify in the simulator → field test → write it
> down. The phases are ordered by field benefit, not by elegance: Phase 0 measure, 1 break the
> circular reference, 2 stop eating curves, 3 bridge outages, 4 good data fast and faithful,
> 5 roll and wheel-angle zero, 6 acceptance. Below that: the open points and contradictions
> that are still unresolved, the accepted "Fix 1" with its predicted numbers, and the
> operator decisions D1–D7. The goals and numbers live in `01_ziele.md`; the rules of work in
> `WORKFLOW.md`. Status 2026-11-03: content unchanged, split out of
> `archive/GRUNDZIELE_2026-10-11.md`.

Diese Datei ist ein **Plan** (Entwurf 2026-10-11, zur Besprechung) – keine Messung. Sobald ein
Schritt gefahren ist, wandert das Ergebnis in `10_heading.md` / `20_position.md` / `50_roll.md`
und der Schritt wird hier als erledigt markiert. Ziele und Zahlen: `01_ziele.md`.

## 1. Leitsatz

**Messen → Regel → kleiner Fix → im Sim prüfen → Feldprobe → festschreiben.**
Jeder Schritt nennt Beleg, **vorhandenes** Werkzeug und ein prüfbares Abnahmekriterium.
Die Phasen sind nach Feldnutzen geordnet, nicht nach Eleganz.

## 2. Phasen

### Phase 0 – Messen (kein Code, sofort)

Die drei 10-11-Logs auswerten und die sechs Befunde in Zahlen festhalten: errTrack (Heading
gegen Fahrtrichtung), Kurvenverlust in Grad, Positionssprünge, Fehler je Geschwindigkeitsband,
Lenkwinkel-p50 auf geraden sauberen Epochen (Soll 0), Roll-Weg.
Werkzeug: `_l1010.ps1 -Filter '2026-10-11*.txt'`, `_hsources.ps1`, `_straight.ps1`, `_vbins.ps1`.
Abnahme: Tabelle "Betreiber-Befund → Zahl" als Baseline. **Ohne diese Zahlen kein Fix.**

### Phase 1 – Zirkelbezug brechen (klein, Architektur nach `WORKFLOW.md` §8.1)

* Ausgabe führt die **Lage**, wenn sie vertrauenswürdig ist; sonst der **Positionskurs**.
  `psi` geht nie mehr ungeprüft raus (A+B2 ist der Anfang, der Kurs-Fallback fehlt noch).
* Kursquelle = Rohfix; Ringpuffer darf nie die publizierte Projektion füttern.
* WAS-Nullpunkt-Berechnung nur aus Rohpositionen.

Werkzeug: Code-Audit + Sim-Szenario "WAS-Nullpunkt 5,6 Grad falsch" (Heading bleibt richtig,
weil die Lage führt), `_rec*`-Spuren im Log.
Abnahme: Audit-Liste "wer darf wen beeinflussen" ohne Rückkante; Sim-Szenario besteht.

### Phase 2 – Kurven nicht mehr fressen (größter Einzelhebel)

Manöverterm am Ausgangsbegrenzer: `limit = max(Boden, 1,5*|w_modell|)`, **nur freigegeben,
wenn der Kurs die Kurve bestätigt** (sonst Fehlausnahmen). Vorher `rl_allowed`-Semantik prüfen
(0 = kein Limit?) – die alte Verwechslung hat schon einmal in die Irre geführt.
Beleg: 10-10 141…151 Grad verloren, in genau diesen Episoden `rl_clip` 24…51;
Gegenrechnung p50 1,8 / max 4,2 verlorene Grad, Kosten 3,78 % Fehlausnahmen.
Werkzeug: `_sim_manoeuvre.ps1` (vorher), `_l1010.ps1` (nachher), `_oldturns.ps1`.
Abnahme: 90-Grad-Kurve < 5 Grad verloren (Sim und Log), Fehlausnahmen < 5 % der Frames.

### Phase 3 – Ausfall robust überbrücken (Brücke/Baum, keine Sprünge)

Positionskurs als Träger, Modell nur als Kurzzeitrate; Rampen beim Re-Seed und beim
Gate-Schließen; Guard gegen Wegdrehen (das Modell darf nicht gegen einen bestätigenden Kurs
drehen); Roll halten statt springen.
Werkzeug: `_spin.ps1`, `_reentry.ps1`, `_gapdump.ps1`, `fuse.ps1`, `_jumps.ps1`.
Abnahme: 11…29 s Ausfall → Drift < 5 Grad; Wiedereintritt ohne Schritt > 2 Grad;
kein Positionsschritt > 0,15 m im Fahren.

### Phase 4 – Gute Datenlage schnell und treu (Verwackeln und Latenz)

Trust-Prädikat "Empfänger eingeschwungen" aus **Lage gegen 5-m-Kurs** (gerade Epochen,
Hysterese, Mindestdauer) → dann Ausgabe = lagegeführte Körperrichtung mit **kurzem**
gleitendem Durchschnitt, dessen Bandbreite an die **physikalisch mögliche Rate** gekoppelt ist
(gerade = stark mitteln, Kurve = nachführen). Damit ist "treu wenn gut" und "stetig" kein
Widerspruch. Die heute nach Tempo/Lenkwinkel gestaffelte Filterstärke wird auf **Datenlage**
umgestellt (Betreiber: gleitender Durchschnitt, weil UM982 Rauschen bringt).
Werkzeug: `_sim_trust.ps1`, `_noise.ps1`, `_band57.ps1`, `_lag.ps1`, `filter_compare.ps1`.
Abnahme (2026-10-12 in physikalische Einheiten gefasst, siehe `01_ziele.md` §1.0): auf Geraden
`p99 |dHeading|/dt <= 1 Grad/s` (jede GPS-Rate, auch bei schlechtem Empfang) und
`p95 |Heading − Positionskurs (2-s-Fit)| <= 0,5 Grad` bei guter Datenlage; in Kurven
`p99 |dHeading|/dt <= max(1, k*omega_phys)`; Latenz < 150 ms (Kreuzkorrelation); im Gutfall
folgt die Ausgabe der Lage live.

### Phase 5 – Roll und Lenkwinkel-Nullpunkt (Keya)

Roll getrennt gaten (keine Sprünge > 1 Grad/Frame, `roll_susp` bleibt); Keya-Nullpunkt aus
Rohpositions-Krümmung lernen, Plausibilitätsband, **nicht im Stand** lernen (im Stand gibt es
keine Information).
Werkzeug: `roll_check.ps1`, `rollout.ps1`, `steering_angle_from_circle.ps1`, `_kfit.ps1`, `circles.ps1`.
Abnahme: Lenkwinkel p50 auf Geraden < 1 Grad; `L_implied` über die Winkelbins konstant.

### Phase 6 – Abnahme

Sim mit je einem Szenario pro Regime (Ausfall, 90-Grad-Kurve, 21-km/h-Gerade, Brücke,
Schleichfahrt) → Feldprobe durch den Betreiber → Ergebnis in `knowledge/` festschreiben und
die Schwellen nachziehen.

## 3. Was wir NICHT machen

* keine neue Hardware voraussetzen; **kein IMU-Zwang** im Kernpfad (Single: IMU nutzen, Dual: ohne);
* die eigene Projektion nie als Kursquelle, das Heading nie als WAS-Nullpunkt-Quelle;
* keine ungemessene "Verbesserung"; jede Änderung mit Zahl vorher/nachher;
* kein großer Umbau vor der ersten Feldprobe (kleine, prüfbare Schritte);
* kein neues Werkzeug je Fahrt – `-Filter` des vorhandenen.


## 4. Offene Punkte und Widersprüche

Erledigte Punkte werden hier gestrichen und im jeweiligen Thema-Dokument belegt.

1. **90-Grad-Kurve** – teilgefixt, zwei getrennte Ursachen:
   * *Ursache 1 (gefixt)*: `rawAz` fiel auf `psi` zurück. Im 10-09-Log drehte die Lage 176 Grad,
     publiziert wurden −95 Grad, bei `out_lim = 0` (also ohne aktiven Deckel). Fix A+B2:
     `rawAz` nimmt IMMER die Dual-Lage, wenn `hasDualHeading && qfOk`.
   * *Ursache 2 (offen)*: der Pfad-Begrenzer frisst echte Kurven. 10-10 gemessen:
     141…151 Grad verloren, in genau diesen Episoden `rl_clip` 24…51.
     Vorschlag 1b: `limit = max(Boden, 1,5*|w_modell|)`, auf den Kurs gegated
     (Gegenrechnung: verlorene Grad p50 1,8 / max 4,2 statt 151; Kosten 3,78 % Fehlausnahmen).
2. **Widerspruch Modellskala** (ungelöst, muss zuerst geklärt werden):
   * 10-04: "Modell dreht in engen Kurven 1,4…1,7x zu weit" → L_eff 2,14 m → 2,95 m (Fac 1,30).
   * 10-06: "mit Fac 1,30 rechnet das Modell 23 % zu WENIG Gierrate, psi hinkt 29 Grad nach,

**1.2a Das KRITERIUM für die Ausgabe (erster Schritt)**: statt `attOk = hasDualHeading && qfOk`
gilt `attOk = hasDualHeading && !attJumped && lagePasstZumPfad`, mit
`lagePasstZumPfad = |wrap(att + Offset - Pfad)| <= attPathTolDeg` (Vorschlag 25 Grad) und
`Pfad` = `travelWitnessDeg` (Positionen) wenn gültig, sonst der Fix-zu-Fix-Kurs; ist KEIN Pfad
verfügbar (Schleichfahrt unter 0,3 m/s), bleibt `qfOk` das Kriterium wie heute.
Das harte kinematische Tor (`attJumped`, A+B4) bleibt unverändert.
**1.2b Dasselbe Kriterium für die ANNAHME** (Anker/Gate in `evaluateAttitudeFrame`).
**1.1 `psi` aus der Ausgabe entfernen** (letzter Wert halten bzw. Kurs) – nach 1.2a/1.2b prüfen,
ob der Schritt noch gebraucht wird.

**Ausgangsquelle heute** (A+B2, `HeadingBridge.cpp:2364-2376`):
`rawAz = attOk ? att : (travelWitness gültig ? Kurs : (psiOk ? psi : letzter Wert))`.

**1.1 Ausgangsquelle – `psi` fällt als Ausgabe weg** (Prioritätskette, ersetzt das "sonst psi"
aus A+B2):
1. vertraute Lage (`att + Offset`) → live durchreichen (wie heute),
2. Lage vorhanden, aber gerade verworfen (`baseLen`, `settle`, genullt) → **letzte GUTE Lage
   halten** (ratenbegrenzt, mit den vorhandenen Rampen) – **nicht** `psi`,
3. keine Lage (Single-Antenne, Kaltstart) → **Kurs aus den Positionen** (`travelWitnessDeg`),
4. nur wenn auch das fehlt: der letzte publizierte Wert (wie heute).

Belege: B10.3 (bei 0,3…0,6 m/s lag `att+offset` −1,8…+3,1 Grad auf dem Pfad, `psi` 11…21
Grad daneben, und AOG bekam `psi`), B10.4 (die Lage wurde als `baseLen`/`settle` verworfen,
obwohl sie passte), B10.8 (in 1232 von 1440 Frames war die Lage als Wahrheit unbrauchbar →
der Rückfall muss die GEHALTENE LAGE sein, nicht der Zustand).

**1.2 Trust gegen den PFAD**:
`lageVertraut = |wrap(att + Offset - Pfad)| <= 20 Grad` über ein kurzes Fenster, mit Hysterese;
der Pfad kommt aus den **Positionen** (Fix-zu-Fix bzw. 2…5 m Kurs) und ist damit vom Zustand
unabhängig. Das ersetzt/ergänzt `att_q`, `uni_hdgsd`, `baseLen` und `settle`.
Beleg: B10.5 (24 % / 42 % / 92 % falsche Epochen durchgelassen UND eine gute Lage in 7-01
verworfen).

**Blast-Radius, der nicht brechen darf:** Kaltstart/INIT, Single-Antenne (kein Dual),
Rückwärts-Spiegelung (Vorderachse!), Roll, Position, die >10-km/h-Geraden und die Kurvenantwort.

**Ablauf:** 1.1 implementieren → Sim + die drei Referenz-Logs messen → Commit `[B10.3]`;
danach 1.2 genauso → Commit `[B10.5]`; dann `tools/sim/regression.ps1` + Baseline über den
Referenz-Log-Satz, damit diese Zahlen ab dann automatisch geprüft werden.

## 6. Entscheidungen D1–D7 (zur Bestätigung)

| # | Frage | Empfehlung |
|---|---|---|
| D1 | Vorrang-Regel so richtig: gute Datenlage = treu+schnell, schlechte = stetig+halten? | ja, genau so umsetzen |
| D2 | Darf im Gutfall die **Attitude** die Ausgabe führen (statt `psi`)? | ja – das ist der Bruch des Zirkelbezugs |
| D3 | Reihenfolge: erst Kurven (Phase 2) oder erst Live/Ruhe (Phase 4)? | erst Kurven, dann Ausfall, dann Live |
| D4 | Kurslänge dynamisch 2…5 m, krümmungskorrigiert, aus Rohfixes? | ja |
| D5 | WAS-Nullpunkt aus Rohpositionen (Keya-Start bei 0) – wie dringend? | Phase 5, nach Kurven/Ausfall |
| D6 | Filterstärke nach **Datenlage** statt nach Tempo/Lenkwinkel staffeln? | ja (Betreiber: gleitender Durchschnitt ist gut) |
| D7 | Gilt "verzögerungsfrei" auch für die Position (Projektion nur als Notnagel)? | ja, Projektion <= 0,5 m begrenzt |

**Status:** D2, D4, D6 und D7 sind im Code umgesetzt, D3 hat die Phasenreihenfolge festgelegt,
D5 bleibt bis Phase 5 offen.

     die 30-Grad-Notbremse feuert 2x" → zurück auf `effWheelbaseFac = 1.00` (so steht es heute im Code).
   Beide Messungen sind dokumentiert. Vermutung: verschiedene Wahrheit (path/COG vs. `att`)
   oder verschiedene Epochen (Roll/Schlupf). Nächster Schritt: eine Messung, eine Wahrheit,
   neues Log mit korrekt gefahrenen 90-Grad-Kurven.
3. **Lenkwinkel-Versatz 5,6 Grad** (Betreiber 10-11 7-01): bei 0,42 m/s ergibt das
   `v*tan(5,6 Grad)/L_eff` ~0,9 Grad/s falsche Gierrate → 19 Grad in 20 s. Passt genau zum
   "Heading unter der Brücke falsch" bei 1,5 km/h. Zur Messung: p50 des Lenkwinkels auf
   geraden, sauberen Epochen (Soll 0) und `yr` gegen die Kursrate.
4. Aus dem Arbeitsstand 10-10 offen: **1a** psi nie als Ausgangsquelle (Fallback = Positionskurs),
   **2** Trust = Lage gegen 5-m-Kurs, **3** Offset laufend schätzen statt festschreiben.

## 5. Fix 1 (beschlossen 10-11) – Ausgangsquelle und Trust

**Ziel als Zahlen. Die "heute"-Spalte ist gemessen, die Zielspalten sind die VORHERSAGE
(Workflow Tor 2: kein Fix ohne Vorhersage).**

**Reihenfolge korrigiert (Code-Lesung 10-11, siehe B10.9):** die Kette
`att -> Kurs -> psi -> letzter Wert` IST bereits implementiert; der wirksame Hebel ist das
**KRITERIUM**, nicht die Reihenfolge. Daher:
**1.2a** zuerst (die Lage für die AUSGABE gegen den PFAD prüfen, statt nur über die
Empfänger-Flags `qfOk`/`baseLen`/`settle`), dann **1.2b** (dasselbe Kriterium für die
ANNAHME/den Anker), dann **1.1** (den `psi`-Schritt entfernen bzw. absichern).

| Kennzahl (10-11-Logs) | heute | Ziel nach 1.2a | Ziel nach 1.2b | Ziel nach 1.1 |
|---|---|---|---|---|
| 7-01 errTrack p50 (1…2 km/h, Brücke) | **17,1** | < 5 | < 3 | < 3 |
| 6-57 errTrack p90 (6 km/h, Brücke) | 28,3 | < 15 | < 10 | < 10 |
| 6-52 >10 km/h p50 / p90 | 1,2 / 5,7 | unverändert | unverändert | unverändert |
| Kurvenverlust \|p50\| (drei Logs) | 0,19 / 2,42 / 0,18 | unverändert | unverändert | unverändert |
| Positionsschritte > 0,15 m (drei Logs) | 4 / 2 / 2 | unverändert | unverändert | unverändert |
| Frames "\|Lage als Wahrheit unbrauchbar" (6-52) | 1232 | soll fallen | soll fallen | soll fallen |
