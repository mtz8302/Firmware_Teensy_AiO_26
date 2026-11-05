# Heading-Analyse 2026-10-09 / 10-10 — Messungen, Simulationen, Entscheidungen

Diese Sitzung hat den Heading-Pfad (Dual/UM982, keine IMU) **gemessen statt geraten**.
Alles hier ist aus den Logs vom 10-09 und 10-10 (und den Vorgängersitzungen) berechnet.
Werkzeuge siehe Abschnitt 7.

## 1. Gemessene Kernzahlen (10-09 vor A+B, 10-10 mit A+B+S-B)

| Groesse | 10-09 | 10-10 |
|---|---|---|
| psi-Fehler zur Fahrtrichtung (p50, gerade/sauber/schnell) | +10,7 / +24,4 / +5,9 Grad | **-1,45 / +5,58 Grad** |
| Heading-Rauschen roh -> gestaffelt gefiltert | 4,5-6,9 -> 1,2-1,5 Grad (x3,0-5,6) | (S-B war noch nicht gefahren) |
| Positionssprünge (alle bei `publim 1->0`, fixQ=4!) | 1,2-30,0 m | **2,4 / 44,1 / 46,3 m** |
| Verlorene Grad in echten Kurven (p50 / max) | 4,3 / 170 Grad | **0,85-2,6 / 88-151 Grad** |

Nachweis "psi driftet, Lage nicht": im 90-Grad-Fall drehte die **Lage 176 Grad**, publiziert
wurden **-95 Grad** — bei `out_lim = 0`, d.h. **kein Begrenzer war aktiv**. Ursache war die
QUELLE (`rawAz` fiel auf `psi` zurueck), nicht die Ratenbegrenzung.

## 2. Die wichtigsten Befunde

1. **Der Positionskurs ist der beste Zeuge.** Ueberbrueckung echter Ausfaelle: **Positionskurs
   0,10-1,11 Grad** gegen **Modell (Lenkwinkel) 13,9-29 Grad** (Luecken 11 s bzw. 29 s).
   Basis: Ringpuffer, 300 Fixes = ~32 s; relative Positionsgenauigkeit ~1 cm (RMS 3,2 cm ohne RTK).
2. **Unsere eigene publizierte Position verdirbt sie** (5,22 Grad statt 0,10 Grad) — Projektions-
   Weglaeufe. **Die Projektion darf sich nie selbst ankern.**
3. **Chord-Nachlauf in Kurven = (B/2)*tan(delta)/L** (bis 12 Grad bei B=10 m, delta=10 Grad).
   Mit diesem Term ist **kein Geradeaus-Tor** noetig — der Anker darf in Kurven laufen.
   Restfehler des Bogens ~kappa^2*B^3/24 = 0,002 Grad bei R=40 m, B=5 m.
4. **Die Positions-Gierrate ist kurzzeitig zu verrauscht** (sigma_w = 2*sigma_pos/(B*dt):
   B=2 m, dt=1 s -> ~0,6 Grad/s). Das **Modell** ist kurzfristig besser (0,1 Grad
   Lenkwinkelfehler -> 0,002 Grad/s), aber **systematisch falsch** (s. 1.). Arbeitsteilung:
   Position = Traeger/absolut, Modell = Kurzzeit/Plausibilitaet.
5. **Die Vertrauens-Kriterien des Empfaengers sind untauglich.** ROC ueber beide Logs:
   `att_q=1`, `uni_hdgsd <= {0,5/0,75/1,0/1,5}`, Baseline-Toleranzen, Einfrier-Erkennung —
   **alle** lassen 24 % (gutes Log) bis 75 % (schlechtes Log) der "guten" Epochen durch, deren
   Lage >2 Grad falsch ist. Falsche Ablehnungen sind ueberall klein (0,2-4,7 %).
   => Trust muss **gegen den Positionskurs** gemessen werden.
6. **`uni_hdgsd = 0` heisst "keine Loesung"**, nicht "perfekt"; Werte bis 1099,8 kommen vor
   (bereits in `tools/analysis/uni_quality.ps1` so behandelt).
7. **Der Montageversatz ist nicht konstant**: p10..p90 = 3,2 / 7,6 Grad Streuung, Drift
   innerhalb eines Logs 0,16 bzw. 2,1 Grad. Sidehill/Roll scheidet aus (Roll klein, Betreiber).
   => Offset laufend schaetzen (Median Lage-gegen-Kurs auf geraden Epochen), nicht festnageln.
8. **Der Lenkwinkel ist sauber** (sigma 0,04-0,11 Grad). Fruehere "1,8-2,3 Grad" waren ein
   Auswahl-Artefakt (nur v>1 m/s betrachtet -> zweite Differenz misst echte Kruemmung).
   **Lenkwinkel glaetten bringt nichts** (Faktor 0,77-1,41, kostet 100 ms Phase).
9. **Das Dual-Heading jittert 2-6 Grad von Frame zu Frame**, waehrend der Empfaenger
   0,32-0,55 Grad meldet (Faktor 10 zu optimistisch).
10. **`rollLpfTauS = 0,8 s` ist die eigentliche Latenz fuer gute Daten** (Ausgangsstufe selbst ist
    phasenneutral: att->az_out 0-100 ms, gemessen per Kreuzkorrelation).
11. **`gap_ms` ist die Gap-SCHWELLE** (3 Satzperioden ~300 ms), keine Verzoegerung.
12. **Echte Kurven wurden "gefressen"**: 141-151 Grad verloren, in genau diesen Episoden
    `rl_clip` 24-51 (Pfad-Begrenzer). Gegenrechnung mit `limit = max(5, 1,5*|w_modell|)`:
    verlorene Grad **p50 1,8 / max 4,2** statt 151. Kostenseite: 3,78 % der Frames zeigt das
    Modell >15 Grad/s waehrend die Wahrheit steht => Ausnahme **muss auf den Kurs gegated** werden.
13. **Das Modell ist in schnellen Kurven ~30 % zu niedrig** (27,5 statt 39,5 Grad/s) —
    Verdacht: VWAS glaettet den schnellen Lenkwinkelsprung.

## 3. Verworfene Ansaetze (nicht wiederholen)

- **Festes 5-m-Fenster** -> falsch; Laenge muss dynamisch sein (Qualitaet unten, Kruemmung oben).
- **Glaetten von Lenkwinkel UND Heading mit Staffel** -> Lenkwinkel bringt nichts (s. 2.8).
- **Naiver Vorhalt aus der Ableitung des geglaetteten Signals** -> hebt den Gewinn vollstaendig
  auf (Faktor 1,00). Vorhalt MUSS aus Modell/IMU kommen.
- **Trust ueber `uni_hdgsd`/`att_q`/Baseline** -> ROC widerlegt (s. 2.5).
- **Konstanter Montage-Offset** -> Frame ist zwar konsistent (-88,2 bzw. -89,4 Grad), aber
  streuend (s. 2.7).
- **`psi_att` (HR-Spalte 58) als Wahrheit** -> Spalte friert ein, wenn kein Mess-Update laeuft.
- **"Die Lage war eingefroren"** -> falsch: `att` lief sauber (20,11 -> 2,94 Grad = -17,2 Grad);
  eingefroren war die *Log-Spalte* `psi_att`.
- **"`out_lim = 0` heisst Deckel 0"** -> falsch: `outResidLimDegS` wird mit 0 initialisiert und nur
  gesetzt, wenn der Ratenpfad eingreift. 0 = **kein Deckel aktiv**.
- **"1,8-2,3 Grad Lenkwinkel-Rauschen"** -> Auswahl-Artefakt (s. 2.8).

## 4. Architektur-Entscheidung (aus den Zahlen)

AgOpenGPS macht es vor (Betreiber-Hinweis): **Gierrate kurzfristig + fix-to-fix-Heading absolut**
— bei Single/BNO liefert die BNO die Rate kurzfristig, AOG macht sie ueber fix-to-fix (0,5-1 m)
absolut "genau und reaktiv". Ohne IMU ist unser "Gyro" das **Modell** — und das ist systematisch
falsch (13,9-29 Grad gegen 0,10-1,11 Grad beim Positionskurs). Deshalb:

| Rolle | Quelle | gemessen |
|---|---|---|
| Traeger der Projektion (absolut) | **Positionskurs aus dem Ring** (2-5 m, kruemmungskorrigiert) | 0,10-1,11 Grad / 11-29 s |
| Kurzzeitrate / Plausibilitaet | Modell (Lenkwinkel+Radstand) | 1-4 Grad/s Abweichung -> nur Gegenprobe |
| Trust | **Lage gegen Positionskurs** (gerade Epochen, Hysterese) | sigma-Kriterien versagen |
| Nie | publizierte (Projektions-)Position als Kursquelle | 5,22 statt 0,10 Grad |

Konsequenz aus AOGs 0,5-1 m: **unser eigenes Positionsrauschen wird dort 1:1 zum Heading-Fehler**
(1 cm auf 1 m = 0,57 Grad; 0,5 m Sprung = 30 Grad) — der Schraeglauf entsteht zum Teil in AOG
selbst, aus unserer Position. Und: **unser Kurs darf laenger sein als AOGs Kurzbasis** (2-5 m mit
Kruemmungskorrektur ist besser als 0,5-1 m, weil das Rauschen mit 1/B faellt).

## 5. Build-Stand (10-10, verifiziert gruen)

- **A**: `psiAnchorSdDeg` 0,8 -> **1,5**, `psiAnchorRScale` 0,5 -> **0,3** (psi-Fehler x7 besser) ✓
- **B**: `reseedRampDegPerTick = 6` (Re-Seed gerampt = 60 Grad/s) · Gate-Schliessen gerampt ✓
- **S-B**: `stagedOutFilter` — alpha 0,2 bei |Lenkwinkel|<=4 Grad und v<=2 m/s, 0,3 bei <=6/<=3,
  sonst 0,5; **Modellraten-Vorhalt unveraendert** ✓
- **A+B2**: `rawAz` nimmt **immer** die Dual-Lage, wenn `d.hasDualHeading && qfOk` — nie mehr psi,
  wenn das Gate geschlossen ist. Beleg: 176 Grad Drehung -> nur 95 Grad publiziert bei `out_lim=0`.
  Restrisiko: eine eingefrorene Lage wuerde publiziert -> Kurs-Tor (Punkt 2) folgt.
- RAM1 free for local variables **4256 B** (unveraendert), RAM2 346 KB frei.

## 6. Offen (Reihenfolge)

1. **1a**: psi nie als Ausgangsquelle; Fallback = **Positionskurs**; Projektion **<=0,5 m** an den
   Empfaenger gebunden; Freigabe beim ersten nachweislich guten Fix
   (behebt die 44-46 m-Spruenge und "total schraeg" unter der Bruecke).
2. **1b**: Manoever-Term im **Pfad**-Begrenzer (`rl_clip` 24-51), `limit = max(Boden, 1,5*|w_modell|)`,
   auf den Kurs gegated. **Vorher `rl_allowed`-Semantik pruefen** (0 = kein Limit?).
3. **2**: Trust = Kurs-Vergleich (Lage gegen 5-m-Kurs, nur gerade Epochen wegen Schlupf).
4. **3**: Offset laufend schaetzen (2.7) — zuerst pruefen, ob die Streuung nur die Rauschgroesse der
   *Bezugsbasis* war (R gegen den 5-m-Kurs neu messen, nicht gegen `travel`).
5. **4**: Zustandsmaschine (3+1 Zustaende) als Klammer · `attRCapDeg` dynamisch (Kurven-Ausnahme) ·
   `rollLpfTauS` im Gutfall 0,1 s · **S1 Ratenzustand `[psi, omega, b_delta]`** (Beleg: "Lenkwinkel
   gehalten" 0,3 Grad vs "roh" 14 Grad ueber 8 s) · **S2 eine Positions-Stimme pro Fix**
   (cog/fix2fix/proj sind korreliert) · E (Rueckwaerts) · Trim-Block · `steerScale`.
6. Werkzeuge richten: `raw_step`/`pub_step` defekt (alles aus `pub_lat/lon` rechnen) ·
   `HrLog.ps1`/`Wrap*` (Infinity-Haenger, while-Schleifen).
7. **Log-Spalten 1/2 sind in 10-09/10-10 nicht die live Position** — 98/99 nutzen.

## 7. Werkzeuge dieser Sitzung (`tools/analysis/`)

`_offset_and_turns.ps1` (psi-Wahrheit + echte Kurven mit Clip-Zaehlern) · `_sim_yaw.ps1`
(Schranken |w_pos-w_model| + Tragefaehigkeit echter Luecken, `-PosSrc 1/98`) · `_sim_manoeuvre.ps1`
(Manoever-Gegenrechnung + Kostenseite) · `_sim_trust.ps1` (Trust-ROC + Offset-Diagnose) ·
`_sim_stage.ps1` (Staffelfilter auf Heading+Lenkwinkel) · `_sc2.ps1` (Fensterlaenge: Verfuegbarkeit
+ Streuung, Ring = 32 s) · `_sim_filters.ps1` (0,7/0,3 + Vorhalt) · `_noise.ps1` ·
`_whythemodel.ps1` (warum die Vorhersage ueberstimmt wird) · `_band57.ps1` (Hauptbereich 5-7 km/h) ·
`_gapdump.ps1` (Lueckenspur im Detail) · `_jump1010.ps1` / `_l1010.ps1` (10-10-Auswertung) ·
`_l1009.ps1` · `_jumproll.ps1` · `_l1009jump.ps1`.

**Stolperstein fuer alle kuenftigen Skripte:** PowerShell ist **case-insensitiv** — `$b`/`$B` und
`$gb`/`$gB` sind DIESELBE Variable. Zwei Messlaeufe lieferten dadurch "0 %"-Ergebnisse, weil die
Band-Schleife die Bandgrenzen ueberschrieb und die Ergebnisliste zum Boolean wurde. Immer
unterschiedliche Namen verwenden und jedes Ergebnis gegenpruefen (Diagnose-Lauf mit echten Werten).
