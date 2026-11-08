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

### Stand 2026-11-07 (Feldtest Maeher, Branch `feat/hold-att-rate`)

**Erledigt** (Belege: **B10.81...B10.85** in `10_heading.md`):

* **VWAS-Geometrie ohne Motor nutzbar** (B10.81): das Objekt bleibt am Leben, `s_vwasRunning`
  trennt „gewuenscht" von „laeuft". Damit ist die Fahrzeugseite immer erreichbar, die
  Geometrie kommt aus dem EEPROM, und der offene ADC-Eingang wird als Qualitaet 0 gemeldet.
* **Plausibilitaetsnetz fuer den analogen Geber** (B10.82): Bereich + 10-s-Bewegungstest,
  Grenze empirisch aus vier Logs (91 Grad ohne Geber gegen max. 44 Grad mit).
* **Chord-Gate entkoppelt** (B10.84): pc_ok im Sim 51 -> 67 %, im Feld-Replay 15,6 -> 20,6 %,
  rl_clip 84,7 -> 27,9 %.
* **gamma/2**: einfrieren -> 1 s Karenz -> Rampe; PGN 253 sendet den ANGEWANDTEN Knick.
* **Defaults** auf Traktor-Werte + Offset 90 Grad (B10.85, mit dem dokumentierten Widerspruch
  zur Entscheidung vom 2026-11-02).
* **Web-Texte ohne fahrzeugspezifische Beispielwerte** (Betreiber-Vorgabe 2026-11-07):
  aus `VWASConfigPage.h` sind „Measured: 71,1 cm (15 Fahrten, 13 065 RTK-Frames)",
  „Measured: 88,3 Grad … ein generisches 90 kostet 1,7 Grad", „gemessen am 01.11.: 37
  eingetragen, echt 75" und die beiden Warnsatz-Zusaetze entfernt; ersetzt durch „measure it
  on your machine". Geblieben sind die **Laufzeit-Anzeigen** („measured: …" in der
  Live-Status-Box, „Self-measured") und generische Plausibilitaetsgrenzen („typical > 70 cm") —
  die gelten fuer jede Maschine. Gleiche Aenderung in der aelteren Sim-Kopie der Seite.
  **Hinweis**: `physicalBaselineDefaultM = 0.711 m` steht weiter als Build-Default im Code
  (Begruendung B10.91: die geloeste Laenge dieser Maschine ist 0,83 m → +17 % → Default
  drosselt). **Offene Entscheidung**: Default auf 0 („lernen") setzen?
* **Hauptseite und Dateinamen** (Betreiber 2026-11-07): das Uhr-Symbol bei „Vehicle Settings"
  ist ein **Traktor als Profil-Silhouette** (Kabine hoch → Haube → schmaler Auspuff → Haube →
  Front → unten horizontal zurueck; die Raeder liegen hinter dem Koerper). Der Abstand zur
  Unterzeile war nicht loesbar, weil `.nav-menu a` **`display:flex`** ist — dort ist ein `<br>`
  wirkungslos (deshalb half das Leerzeichen nicht); am Ende sind die **Unterzeilen ganz
  entfallen**: nur „Vehicle Settings" und „Messages & Languages", jedes Wort gross.
  Dateien auf die TouchFriendly-Konvention gezogen: `VWASConfigPage.h`, `HwMsgPage.h`,
  `DragDropCANConfigPage.h`, `CANConfigUploadPage.h` → `TouchFriendly…` (Symbol, Guard,
  Include und extern mitgezogen; Routen und Methodennamen unveraendert).
  **Nebenbefund**: der Include von `SimpleDeviceSettingsNoReplace.h` war TOT (Symbol nirgends
  benutzt, `/device` nimmt `TOUCH_FRIENDLY_DEVICE_SETTINGS_PAGE`) → entfernt; damit lag eine
  komplette Seite ungenutzt im Flash. Ungenutzte Altlasten bleiben ausserdem
  `SimpleAnalogWorkSwitchPage.h`, `SimpleEventLoggerPage.h`, `SimpleHomePage.h`,
  `SimpleNetworkPage.h`, `SimpleOTAPageFixed.h`, `SimpleUM98xConfigPage.h` und
  `TouchFriendlyCANConfigPage.h` (werden nirgends included).
* **Stufe 2 der Geber-Erkennung** (B10.93): der BERECHNETE Knick (gamma aus Kurs + Lage) ist
  jetzt ein zweiter Zeuge — netto > 10 Grad im Fenster bei stehendem Geber.
* **`kOutHardRateDegS = 200 °/s` rechnerisch abgesichert** (B10.94): klemmt erst oberhalb von
  **32 km/h bei 40 Grad** Vollausschlag — Punkt geschlossen.

* **Feldtest 2026-11-08 ausgewertet** (B10.95, `70_feldprobe.md` §11): **2 von 5** Kriterien
  erreicht - `pc_ok` **62,6 %** (Ziel >=20) und Kurvenverhaeltnis **0,89** (Ziel >0,8).
  `veh_type` 0 und `rl_clip` 83 % scheitern an der NICHT eingetragenen Geometrie; dazu
  `rej=baseLen` 24 % (Basislinie 0,710 gegen 0,824). Drei Korrekturen aus dem Test (Commit
  `a3d614c`): PGN 253 sendet bei Qualitaet 0 **0** statt des rohen Analogen (das war AOGs
  -90 Grad), Knick-Schaetzer auch ohne RTK (`fixQuality >= 1`, weil er Kurs + Lage benutzt),
  und der Hinweis "Antenna ahead of pivot" auf **-10...+300 cm** korrigiert.
**Offen, priorisiert:**

1. **NAECHSTE FAHRT: die Geometrie muss ins GEPLAETTE EEPROM** (Feldtest 11-08, B10.95):
   `veh_type` war **0** in allen 1251 Frames, `art_half` 0, `L` = 2,50 m, Basislinie 0,710.
   Erst Basislinie **0,85** (gemessen) oder 0 = lernen, dann `/vwas`: **0,92 / Typ 2 / 75 /
   0,41** (halber Radstand folgt). Danach im Boot-Log `wheelbase=0.92 type=2 maxAngle=75`
   gegenpruefen und messen: `rl_clip` 83 -> ~25 %, `rej=baseLen` 300 -> 0, und in AOG
   **kein -90 Grad** mehr (bzw. der berechnete Knick statt 0).2. **pc_ok bleibt begrenzt** (Feld 15,6/20,6 %): die Restursachen `pc_rej=4` (unter 0,2 m/s)
   und `=7` (keine Sehnenprobe) sind laut **B10.53 gemessen** und duerfen NICHT einfach
   aufgeweitet werden (Verfuegbarkeit 74 %, skew p90 4,3 -> 19,1). Wer das verbessern will,
   braucht einen anderen Ansatz als B(v) - z. B. eine zweite, unabhaengige Probe.
3. **IMU-Rate in BRIDGE/COAST** (B10.83): im IMU-Log 0 % trotz `imu_q = 10` und 4...6 m/s.
   Betrifft nur Fahrzeuge MIT IMU; bewusst nicht weiter verfolgt (Maeher hat keine).
4. **AOG-Fahrtrichtung beobachten** (Gegenprobe zu `a3d614c`): die vermutete Ursache war der
   dauerhaft -90 Grad grosse Lenkwinkel. Nach der naechsten Fahrt pruefen, ob AOG die Richtung
   jetzt stabil erkennt - und ob `veh_type`/`art_half` dann 2 bzw. != 0 sind.
5. **Entscheidung offen**: `physicalBaselineDefaultM` von 0.711 m auf **0 (lernen)** setzen?
   Mit 0,711 als Build-Default drosselt jede Maschine, deren Antennenabstand davon abweicht
   (11-07: geloest 0,83 m = +17 %, Toleranz 5 %) - siehe B10.91/B10.92.
6. **Entscheidung offen**: sollen die ~12 untracked Feld-Logs (ca. 10 MB, inkl. 11-07) ins
   Sim-Repo? Es versioniert bereits 224 Logs.

**Geschlossen bzw. widerlegt am 2026-11-07** (Belege in `10_heading.md`): Modellskala-Widerspruch
(B10.86, Fac bleibt 1,0); "Ursache 2 = Pfad-Begrenzer frisst Kurven" (B10.84, `rl_path` in 0 von
3638 Frames aktiv); 1.2a/1.2b (B10.89 - die Ausgabe ist spaet bzw. ratebegrenzt, nicht falsch
ausgewaehlt); `outSnapConfirmMs` (B10.88, war wirkungslos); Epochen-Bewertung (B10.92,
ableitbar mit `att_why.ps1`, keine neue Spalte); Analog-Erkennung Stufe 2 (B10.93);
`kOutHardRateDegS` 200 Grad/s (B10.94, klemmt erst ueber 32 km/h bei Vollausschlag);
Sim-Repo-Hygiene (115 Build-Artefakte aus dem Index, `.gitignore`); Web-Texte ohne
fahrzeugspezifische Beispielwerte; Hauptseite (Traktor-Symbol, echter Abstand); Datei
`TouchFriendlyVWASConfigPage.h`.
### Stand bis 2026-11-06 (ueberholt - nur noch als Verlauf)

**Erledigt** (Belege: **B10.73...B10.79** in `10_heading.md`, neu `90_aog_messages.md`):

* MA-Fenster nach Drehrate (23 -> 18 %), Halten-Zweig in vier Teilen (-> 11 %),
  gamma/2 ratenbegrenzt (-> 7 %), LIVE-Rampe folgt der Lagrate + kein Snap-Einfrieren
  (-> 4 %), Anwendungsrate 25 Grad/s (-> **3 %**), stehende Frames 77 -> 6.
* Halte-Deckel aus der Fahrzeuggeometrie statt Maehwerks-Werten - fuer Ackermann
  deckungsgleich mit der Tempo-Obergrenze aus B10.46 (19 Grad/s bei 1 m/s).
* PGN 221 Hardware-Meldungen komplett: Katalog, Sprachen (EN/DE im Flash, LOCAL im EEPROM),
  Hauptschalter, Seite `/hwmsg`, **flankengetriggert** (1x je Auftreten), Hardware-Filter
  fuer die zwei Bridge-Meldungen.
* EEPROM-Karte aufgeraeumt: Feld-Layout (1...1299, 3000...4283) unberuehrt, Zusaetze
  1300...2999 mit Reserve, Meldungstexte bei 2000 (500 reserviert).

**Offen, priorisiert:**
> **ACHTUNG: diese Liste ist der Stand bis 2026-11-06 und in Teilen ueberholt.** Verbindlich ist
> der Abschnitt **„Stand 2026-11-07"** weiter unten. Dort sind erledigt: Modellskala (B10.86,
> Fac bleibt 1,0), Ursache 2 (B10.84, widerlegt), Snap-Karenz (B10.88), 1.2a/1.2b (B10.89,
> verworfen), Sim-Repo-Hygiene (erledigt), Epochen-Bewertung (B10.92, ableitbar). Offen bleiben
> nur: Feldprobe, `pc_ok`-Rest, Analog-Erkennung Stufe 2, `kOutHardRateDegS` fuer Ackermann und
> die Entscheidung zu den untracked Feld-Logs.



1. **gamma/2-Schaetzung ruhiger machen** - die letzten 3 % sind Bewegung des SCHAETZERS,
   kein Limiter-Thema: `artEstTauS` 0,6 -> 1,2 s testen, oder gamma hilfsweise aus der
   Pfadkruemmung stuetzen (beta nur als Rueckfall).
2. **Ursache 2 der 90-Grad-Kurve (aus §4.1, unveraendert offen)**: der Pfad-Begrenzer
   frisst echte Kurven (141...151 Grad verloren, `rl_clip` 24...51). Vorschlag 1b liegt
   vor. In unserer Messung erscheint dieselbe Wurzel als **`pc_rej = 8` in 92 % der
   Halten-Kurvenframes** (Sprungtor blockt den Kurs, weil die LAGE schlecht ist) - der
   Halten-Zweig ist nur das Symptom.
3. **Modellskala-Widerspruch (aus §4.2)**: L_eff 2,14 vs 2,95 m, Fac 1,30 - ungeloest.
4. **1.2a/1.2b nicht gebaut**: `attPathTol` / `lagePasstZumPfad` existiert im Code nicht.
   Wuerde Punkt 2 entschaerfen (Kriterium `attOk = hasDualHeading && !attJumped &&
   |att+Offset - Pfad| <= 25 Grad`).
5. **Snap-Mechanik aufraeumen**: seit B10.77 (kein Einfrieren) ist `outSnapConfirmMs` fast
   wirkungslos - entweder richtig oder ganz weg; `outSnapRateDegS` neu bewerten.
6. **`kOutHardRateDegS` 200 Grad/s** ist fuer den Ackermann-Fall nie geprueft worden.
7. **FELDPROBE fehlt** (der eigentliche naechste Schritt): alles oben ist Sim/Replay gegen
   EIN Log. Zu pruefen: 25 Grad/s Knickwinkel in der AOG-Darstellung, Verhalten der
   Meldungen im Einsatz, der Hauptschalter.
8. **Sim-Repo-Hygiene**: 175 Commits vor `origin/main`, dazu Build-Artefakte
   (`SourceCode_AgOpenGPS/**/obj/`) als "modified" - `.gitignore` ergaenzen.

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

> **1.2a/1.2b VERWORFEN 2026-11-07 (B10.89)** — mit Messung begründet, nicht aus Zeitmangel:
> * geradeaus (Lage dreht < 2 °/s) liegt die **Ausgabe** nur **1 °** schlechter als die Lage
>   (p90 3,8 gegen 2,8 °; Referenz-Log 6,0 gegen 4,8 °) → die Auswahl ist dort gut;
> * der Mehraufwand der Ausgabe entsteht **fast nur in Kurven** = **Nachlauf** der
>   Ratenbegrenzung/Halte-Logik (gewollt, B10.53/B10.84), kein Auswahlfehler;
> * der `travelWitness` ist **per Konstruktion in Kurven ungültig** und war nur in
>   **6,6 % der Frames** gültig → 1.2a würde genau dort filtern, wo der Fehler am kleinsten
>   ist, und könnte dort nicht wirken, wo der Betreiber das Problem sieht;
> * in Kurven läuft auch der **COG 12,7 °** vom Witness weg — die Sehne ist dort selbst keine
>   Wahrheit (man würde Nachlauf doppelt zählen);
> * der Nachlauf ist **direkt gemessen**: Verschiebung Ausgabe gegen Lage → Minimum bei
>   **+3 Frames = 300 ms** (Feld-Log 11‑07), **200 ms** (Referenz-Log), Restabweichung dann nur
>   2,9 bzw. 3,5 °. Die Ausgabe ist **spät, nicht falsch** — ein Auswahlkriterium repariert
>   keinen Nachlauf von 200…300 ms.
>
> Werkzeug für die Nachprüfung: `tools/analysis/out_vs_path.ps1` (`-MaxRateDegS 2` = geradeaus).
> Wer Kurven verbessern will, muss am **Nachlauf der Ausgabe** ansetzen, nicht an der Auswahl.
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
