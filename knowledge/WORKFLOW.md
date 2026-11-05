# Workflow gegen das Im-Kreis-Drehen (ENTWURF 2026-10-11, zur Besprechung)

> **Status: verbindlich seit 2026-10-11.** Betreiber-Entscheidung: "eher streng" (A + B + C),
> drei Dateien, Version + Datum zur Zuordnung zu den Logs. Offen bleiben nur die **fachlichen**
> Entscheidungen D1...D7 (`80_plan.md` §6), der **Referenz-Log-Satz** fuer die Baseline und
> die zwei noch zu bauenden Artefakte (unten als *geplant* markiert).

**Version 1.0.0 · 2026-10-11 · verbindlich fuer alle Logs ab 2026-09-20**
Aenderungen: v1.0.0 (10-11) Erstanlage (Struktur, 5 Tore, Invarianten I1...I11, Git-Regeln,
Kennzahlen-Panel). Versionierung: MAJOR Struktur, MINOR Inhalt, **PATCH vor jedem Log-Lauf**.

## Struktur (fest fuer alle kuenftigen Arbeiten)

**Drei Dateien tragen das Gedaechtnis - mehr nicht.**

| Datei | Was hineingehoert | Was NICHT hineingehoert |
|---|---|---|
| `WORKFLOW.md` | Ablauf, Invarianten, Git-Regeln, Kennzahlen-Panel, Struktur/IDs | Befunde, Zahlen |
| `00_ueberblick.md` | Befunde mit Beleg + Status, STAND-Block, Widersprueche | Verfahrensregeln |
| `symptome.md` | je Symptom: Ursachen + Pruefung + **Chronik (Probleme/Loesungen)** | Theorie, Verfahren |

Dazu unveraendert: `01_ziele.md` (Ziele/Messlatte), die drei Werkzeug-READMEs
(`tools/analysis`, `tools/heading_bridge`, `tools/sim`) und die datierten Fachdokumente
(`heading_*`, `baseline_*`, `skew*`, `ring`, `f9p`, `imu_roll`, ...). Die Fachdokumente werden
**nicht umgeschrieben**: ihre Befunde werden in die drei Dateien eingesammelt, das Dokument
bleibt als Beleg stehen. **Kein neues Dokument ohne Platz in der Karte.**

**Versionierung (dreistellig, Betreiber-Entscheidung 10-11)**: `vMAJOR.MINOR.PATCH · Datum ·
gilt fuer Logs a..b` oben in jeder der drei Dateien, plus Aenderungszeile.
* **MAJOR +1**: Struktur/Workflow geaendert.
* **MINOR +1**: neue Befunde/Abschnitte (Inhalt).
* **PATCH +1 vor JEDEM Log-Lauf (Sim oder Feld)** - damit ist jeder Log einer Dokumentversion
  zuordenbar. Der STAND-Block nennt zusaetzlich Commit-Hash + Firmware-Version.
Jeder Lauf wird im **Lauf-Register** eingetragen (im STAND-Block von `00_ueberblick.md`):
`Version | Datum | Log | Art (Sim/Feld) | Zweck/Ergebnis`.

**IDs** (eine Sprache, ueberall gleich):
* `B<Abschnitt>.<Nr>` = Zeile im Befund-Register (`00_ueberblick.md`), z.B. `B1.4`, `B2.7`, `B3.7`.
* `S<Nummer>` = Symptom in `symptome.md` (S1...S8).
* `I<Nummer>` = Invariante in dieser Datei (I1...I11).
Dieselbe ID steht im Register, in `symptome.md`, in der Commit-Nachricht und als Kommentar an der
Firmware-Konstante. Damit ist jede Zahl per `grep` auffindbar (Zeitleiste statt Gedaechtnis).
Subsystem-Zuordnung steht im Text der Zeile (H Heading/Fusion, P Position/Kurs, V VWAS/Lenkwinkel,
R Roll, L Lenkung, N Netz/Log) - sie ist Beschreibung, kein Nummernraum.

**Reihenfolge der Arbeit**: erst Struktur/Workflow, dann Inhalt. Inhalt immer in dieser Folge:
Chronik fuellen -> Symptomtabelle schaerfen -> Baseline/Regression bauen -> dann fixen.

## Geltungsbereich

Gilt fuer **alle Subsysteme der Box**, nicht nur Heading: Heading/Fusion (H), Position/Ring (P),
**VWAS/Lenkwinkel (V)**, Roll (R), Lenkung/Autosteer (L), Netz/Log/Web (N).
VWAS gehoert ausdruecklich dazu, weil es mit dem Heading **gekoppelt** ist: der virtuelle
Lenkwinkel speist das Heading-Modell, und das Heading darf umgekehrt **nicht** in die
VWAS-Nullpunktbildung eingehen (Zirkelbezug, siehe Invarianten I7/I10).



## 0. Warum wir uns im Kreis drehen (Ursachen, belegt aus unserem Verlauf)

1. Das Wissen liegt in **Dokumenten, nicht in Pruefungen**: "Fac 1,30 war falsch" steht als
   Kommentar im Code - aber **nichts schlaegt fehl**, wenn es wieder auf 1,30 gesetzt wird.
2. Jeder Fix wurde an **einem** Symptom geprueft, nie gegen die **anderen** akzeptierten Zahlen.
   (`effWheelbaseFac` 1,00 -> 1,30 -> 1,00 ist genau dieser Kreis.)
3. Es wurde eine **Konstante** geaendert, ohne den Mechanismus zu kennen - der Fix brach ein
   anderes Regime (Kurve, Schleichfahrt, 21 km/h).
4. Es gibt keinen Eintrag, **welche Befunde** eine Aenderung beruehrt (Blast-Radius unbekannt).
5. **Widersprueche ueberleben**: 1,30 (10-04) gegen 1,00 (10-06) stehen beide als "gemessen"
   da; ein Widerspruch ist ein offener Auftrag, kein Detail.
6. Die **Git-Historie ist als Gedaechtnis unbrauchbar** ("simulator 2", "simulator 1",
   "heading schleichfahrt1") - man kann nicht nachschauen, welcher Fix was bewirkt hat.
7. Zwischen den Sitzungen habe **ich kein Gedaechtnis**. Das einzige dauerhafte Gedaechtnis ist
   das Repo. Also muss **das Repo pruefen** - nicht ich mich erinnern.

## 1. Grundregel

**Kein Fix ohne belegte Ursache. Kein "gruen" ohne alle alten Kennzahlen.**

Gedaechtnis = `00_ueberblick.md` (Befunde mit Beleg) + `regression_baseline.json` (Zahlen, die
nicht schlechter werden duerfen) + Commit-Historie mit Befund-IDs `[B2.7]`.

## 2. Die vier Artefakte

| Artefakt | Inhalt | Pflege |
|---|---|---|
| `knowledge/00_ueberblick.md` | Befunde + Status (BEWIESEN/WIDERLEGT/OFFEN) + Widersprueche | bei jedem Fix |
| `knowledge/symptome.md` | **Symptom -> bekannte Ursachen -> Pruefwerkzeug -> Zahl** | bei jedem Feldbericht |
| `knowledge/regression_baseline.json` | je Referenzlog das Kennzahlen-Panel (Sollwerte) | nur wenn nichts schlechter wird (**geplant**) |
| `tools/sim/regression.ps1` | faehrt alle Referenzlogs im Sim, vergleicht, druckt GRUEN/ROT | ein Befehl, immer (**geplant**) |

## 3. Der Ablauf (5 Tore)

**Tor 0 - Sitzungsstart (2 Minuten, immer):**
`git log -3` · `00_ueberblick.md` "Stand"-Block lesen · **`tools/sim/regression.ps1`** (GRUEN/ROT
gegen die Baseline) · bei Firmwareaenderung `pio run -e teensy41`.
Ohne Tor 0 beginnt keine Arbeit - sonst wissen wir nicht, wo wir stehen.

**Tor 1 - Symptom aufnehmen (kein Code):**
Betreiberbericht -> Eintrag in `symptome.md` (Log, Zeitfenster). Dann die **bekannten Ursachen
dieses Symptoms** der Reihe nach pruefen, je Ursache ein Werkzeug + eine Zahl. Erst eine belegte
Ursache erlaubt einen Fix. (Das ist die Antwort auf "wir loesen dasselbe Problem wieder": das
Symptom hat eine Tabelle, keinen Blindflug.)

**Tor 2 - Aenderungsantrag (10 Zeilen, schriftlich):**
* Befund-ID(s), auf die sich die Aenderung stuetzt,
* **Blast-Radius**: welche Kennzahl darf sich NICHT verschlechtern, welches Regime darf nicht brechen,
* **Vorhersage als Zahl** (z.B. "verlorene Grad je 90-Grad-Kurve: 151 -> < 5"),
* Regime, in dem die Aenderung nicht schaden darf.
Ist die Vorhersage danach falsch -> **zuruecknehmen**, nicht nachtrimmen.

**Tor 3 - Umsetzung + Mess-Gate (Sim):**
Firmware aendern, `tools/sim/src/*` nachziehen (Kopie wird per Hash geprueft), dann
`regression.ps1 -Baseline` **plus** den neuen Log: Tabelle alt/neu je Kennzahl.
**ROT = nicht weiter.** Zusaetzlich die Invarianten (§4) automatisch pruefen und die AOG-Sicht
live ansehen.

**Tor 4 - Festschreiben:**
Register ergaenzen (Zahl + Beleg, alte Befunde ggf. WIDERLEGT), Baseline aktualisieren (nur
wenn nichts schlechter), **ein Commit je Fix** mit `[W-IDs]` und den Zahlen.
Git-Muster: `kurve: rl_clip-Manoeverterm -> verlorene Grad 151 -> 3,4 [B2.7]`.

**Tor 5 - Feldprobe mit einem Zweck:**
Vorher aufschreiben, welche Kennzahl geprueft wird; danach das Log durch Tor 0/1 schicken
(sind die alten Kennzahlen noch gruen?). Ein Feldtest ohne Zweck ist ein verlorener Tag.

## 4. Invarianten (Maschinenpruefung, aus dem Register)

| # | Invariante | Herkunft |
|---|---|---|
| I1 | `az_out == att` im Passthrough/INIT | `azcheck.ps1` |
| I2 | `psi_att` bleibt im Gutfall konstant | skewmech §3 |
| I3 | `sc_b = 0` (der WAS besitzt die Null, nicht das Modell) | Spaltenkommentar |
| I4 | `publim = 1` => Projektion <= 0,5 m vom Fix | 10-10 §6.1 |
| I5 | kein Positionsschritt > 0,15 m im Fahren | 01_ziele.md §1 |
| I6 | Kursquelle ist nie `proj` | 10-10 §2.2 |
| I7 | WAS-Nullpunkt nur aus Rohfix, nie aus Heading | WORKFLOW.md §8.1 |
| I8 | kein Sprung > 2 Grad/Frame im Heading | 01_ziele.md §1 |
| I9 | kein `src=deadreck` laenger als 2 s im Gutfall | `_l1010.ps1` |
| I10 | Wheel-Quality unter `wheelQualityLowPct` => `yaw_zeroed = 1` und `yr = 0` (das Modell darf dann nicht fuehren) | report 10-02 §0, `yaw_zeroed`-Spalte |
| I11 | Heading/`az_out` geht NIE in die VWAS-Nullpunktbildung ein; Nullpunkt nur aus Rohfix | WORKFLOW.md §8.1, I7 |

## 5. Git-Regeln (das Gedaechtnis bezahlbar machen)

* **Ein Fix = ein Commit.** Nachricht: `<Klasse>: <Symptom> -> <Kennzahl vorher/nachher> [W-IDs]`.
* Kein Commit, solange ein **offener Widerspruch** die geaenderte Groesse betrifft.
* Firmware-Konstanten tragen im Kommentar die Befund-ID, z.B. `// [B2.7] gemessen 10-10 ...`.
* `tools/sim/src/*` sind Kopien - nie "mal schnell" dort aendern; Hash-Pruefung im Build/Werkzeug.

## 6. Kennzahlen-Panel (je Log, aus vorhandenen Werkzeugen)

1 errTrack p50/p90 · 2 psiDrift p50/p90 · 3 **verlorene Grad je echter Kurve** ·
4 max Positionsschritt · 5 Heading-sd je 0,1 s · 6 Lenkwinkel p50 auf Geraden ·
7 max Roll-Sprung · 8 Fehler in schlechten Epoch.
Diese acht Zahlen sind die "Gesundheit" eines Logs; jede Aenderung muss alle acht nennen.

## 7. Warum das den Kreis bricht (konkret)

Feldbericht "90-Grad-Kurve wieder falsch" -> Symptomtabelle nennt drei bekannte Ursachen
(psi als Quelle [B2.6], rl_clip [B2.7], Modellskala [Widerspruch]). Also **zuerst** drei Zahlen
aus einem Log, dann **ein** Fix, dann **alle acht** Kennzahlen gegen die Baseline, dann Commit
mit IDs. Bricht die Kurve spaeter erneut, sagt Tor 0 sofort **welche** Kennzahl sich seit
welchem Commit verschlechtert hat - statt einer neuen Woche Ursachensuche.

## 8. Arbeitsregeln (verbindlich)

Diese Regeln gelten für jede Sitzung. Sie standen früher in `GRUNDZIELE.md` und gehören zum
Ablauf, nicht zu den Zielen (Zahlen und Messlatte: `01_ziele.md`).

* Erst messen, dann behaupten. Jede Aussage mit Log, Zeitstempel, Spalte.
* **Vorhandene Werkzeuge benutzen und erweitern** (Karte in `00_ueberblick.md`), kein neues
  Skript pro Fahrt: neue Fahrt = `-Filter` des vorhandenen Auswertewerkzeugs.
* Ergebnis in `knowledge/` mit Datum; Firmware-Entscheidung als Messbeleg im Code-Kommentar.
* Widerspruch zwischen zwei Messungen wird benannt und aufgelöst, nicht geglättet
  (Liste in `80_plan.md`).
* Firmware bleibt baubar (`pio run -e teensy41`); RAM1/RAM2 im Blick.
* **Jede Änderung wird im Simulator verifiziert** (`tools/sim`, echte Quellen, live nach AgIO
  über `$PAOGI`): messen → ändern → denselben Log erneut fahren → virtuelle Ausgabe **und**
  AOG-Anzeige vergleichen → dann Feldprobe.

### 8.1 Harte Architektur-Regel: KEIN Zirkelbezug

Betreiber-Festlegung 10-11: *"wir müssen aufpassen, dass es keinen Zirkelbezug aus Heading für
VWAS und VWAS → simuliertes Heading/Projektion gibt."* Daraus vier Regeln:

1. **`psi` (unser Filterzustand) ist NIE die veröffentlichte Quelle**, wenn eine Messung
   vorliegt. Veröffentlicht wird die **lagegeführte** Körperrichtung (gute Datenlage) oder
   der **positionskursgeführte** Wert (schlechte Datenlage).
2. **Kursquelle ist nur der Rohfix** (`lat/lon`), NIE die eigene publizierte Projektion.
3. **Der WAS-Nullpunkt wird aus Rohpositionen gelernt** (Krümmung gegen Lenkwinkel), nie aus
   Heading oder `az_out`.
4. **Der Lenkwinkel ist nie die einzige Quelle des veröffentlichten Headings.** Genau das
   passiert, wenn `psi` rausgeht: WAS → psi → Heading → (AOG lenkt) → WAS schließt sich
   zum Kreis und eine 5,6-Grad-Schieflage wird zu einer dauerhaften Drehung.

### 8.2 Vier Regime statt einer Kette

| Datenlage | Träger (absolut) | Kurzzeit | Ausgabe |
|---|---|---|---|
| Empfänger gut und **eingeschwungen** | Attitude | Attitude | **live**, nur kurz gemittelt |
| Lage zweifelhaft, Positionen gut (v > 0,5 m/s) | **Positionskurs** (Ring 2…5 m, krümmungskorrigiert) | Modell (nur Gegenprobe) | kursgeführt, stetig |
| Positionen gut, sehr langsam (5 m dauern > 10 s) | letzter guter Kurs | Modell | halten / rampen |
| alles schlecht (Brücke, Ausfall) | letzter Kurs + Modellrate, auf die physikalische Rate begrenzt | Modell | halten, keine Sprünge |

Begründung aus den Logs: Positionskurs 0,10…1,11 Grad, Modell 13,9…29 Grad über 11…29 s
Ausfallücken (10-10); Dual-Heading jittert 2…6 Grad/Frame, der Empfänger meldet 0,32…0,55
(Faktor 10 zu optimistisch) → gleitender Durchschnitt im Gutfall, Rate-Limit bei Spikes.

