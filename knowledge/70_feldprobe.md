# 70 · Feldprobe: Checkliste und Auswertung

**Version 1.0.0 · 2026-10-11 · fuer die NAECHSTE Fahrt (Fix-Stand: B7.21/B7.34, B10.20-B10.24)**
Aenderungen: v1.0.0 Erstanlage. Versionierung: MAJOR Struktur, MINOR Inhalt, **PATCH vor jedem Log-Lauf**.

## 0. Warum diese Fahrt (drei Luecken auf einmal)
1. **Die Beobachter-Spalten fehlen im Feld**: die 10-11-Logs haben **101 Spalten**, der Block
   (103...116: `enc_cnt, enc_deg, enc_off, zo_res, zo_st, zo_fit, zo_gain, zo_roll, zo_rate,
   gps_a, aog_zero, aog_wasoff, cpd_aog, vwas_trim`) entstand erst am 10-11 ⇒ **ohne diese Fahrt
   ist keine Feldpruefung von Trim, Nullpunkt und Vertrauen moeglich**.
2. **Der Ziel-Fall des Trim-Fixes fehlt**: alle 10-11-Logs haben `guid = 0` (ohne Autosteer) ⇒
   der VWAS war nie `ZO_BLOCKED` (B7.35) ⇒ genau der Fall, den der Fix adressiert, fehlt.
3. **Der VWAS-Nullpunkt war 2...6 Grad daneben** (gemessen, B10.24) — die Quelle des Jitters,
   unabhaengig von jeder Filterfrage.

## 1. Vorbereitung (vor dem Losfahren)
- [ ] Firmware flashen (aktueller Stand). **Die HR-Zeilen kommen ueber `Serial.printf`** ⇒ sie
  erscheinen nur in der **seriellen Ausgabe** — die wie bei den 10-11-Fahrten mitschreiben.
- [ ] VWAS-Konfiguration: `countsPerDegree` (Anschlag-Kalibrierung), `invertEncoder`,
  `wheelbase`, `maxSteeringAngle`, `vehicleType`.
- [ ] Bridge-Geometrie (Web `/api/bridge/geometry` bzw. VWAS-Seite): `meters` = **0** (lernen)
  oder **0,711**; `offsetDeg` = derselbe Wert wie in AgOpenGPS.
- [ ] Bootmeldungen pruefen: `physicalBaselineM ...` (EEPROM oder Build-Wert) und
  `HeadingBridge ENABLED: ... base=0.711m`.
- [ ] **Wissensbasis-Versionen notieren** (PATCH-Nummern der Dateien) — sie gehoeren zum Log.
- [ ] Log starten und **bis zum ersten `HR,`-Satz** laufen lassen (Selbsttest).

## 2. Was fahren (Segmente >= 30 s, Reihenfolge, Uhrzeiten notieren)
| # | Manoever | Dauer | Autosteer | Was es beweist |
|---|---|---|---|---|
| A | **Stillstand**, Motor an, Raeder gerade | 30 s | aus | Beobachter laeuft NICHT (`zo_st = 0`), Winkel konstant, `vwas_trim` bewegt sich nicht |
| B | **Geradeaus 2...6 km/h**, fester Untergrund | 60 s | **EIN** | Ziel-Fall: `guid=1`, `zo_st=3` (BLOCKED), `zo_res` ≠ 0, `vwas_trim` FOLGT, `mdev`/`mgate` niedrig |
| C | **Geradeaus 10...20 km/h** | 60 s | EIN | Zielzahl ±0,5°: `p99 \|Δaz_out\|`, Band gegen den Pfad |
| D | **90°-Kurven** links und rechts (je 3) | je ~10 s | EIN | Kurvenantwort ohne Verzoegerung; der Zaun muss dort weit sein |
| E | **Schleichfahrt 1...2 km/h + Wenden** | 60 s | aus UND ein | der bekannte harte Fall (B10.13) |
| F | **Unter Baeumen / Bruecke** (die bekannte Stelle) | 30 s | EIN | Trust/Reception + die aktive Referenzpruefung (B10.20) |
| G | **Rueckwaerts**, kurz | 10 s | aus | Richtungsentscheidung |
| H | **Keya/Motor aus und wieder ein** (Raeder bleiben GERADE) | — | aus | B7.7: Nullpunkt = Stellung beim Einschalten |
| (I) | optional **Anschlag links/rechts** | — | aus | cpd-Kontrolle |

## 3. Waehrend der Fahrt (die wichtigste Messung: der Betreiber)
- [ ] AOG-Anzeige: **zittert das Heading?** Wenn ja: bei welcher Geschwindigkeit, geradeaus oder
  in der Kurve? (Das ist die Hauptbeobachtung — Zahl dazu schreiben, z. B. „bei 12 km/h ±1°“.)
- [ ] Zieht es schraeg oder wandert es dauerhaft? (Richtung + Groesse)
- [ ] Web-Seite: `bridgeBaselineRefSuspect` gesetzt? `bridgeBaselineSuggestionM`?

## 4. Auswertung (der Reihe nach)
- [ ] **Spalten pruefen**: die HR-Zeile hat **>= 116** Werte; die Namen in
  `tools/analysis/HR_HDR.current.txt` sind die Referenz. **Falle**: 101 gegen 115 Namen —
  ein Skript, das nach Namen sucht, findet fehlende Spalten nicht von selbst.
- [ ] `guid=1`-Anteil (existiert der Ziel-Fall?)
- [ ] `zo_st`-Verteilung (3 = BLOCKED waehrend Guidance) und `zo_res` gegen `vwas_trim`
  (der angewandte Trim muss `zo_res` folgen, ratenbegrenzt 5°/s)
- [ ] **`mdev`/`mgate`** = Modellguete (die Abnahme des Trim-Fixes; **NICHT** `errTrack`, B7.35)
- [ ] Gerade + schnell: `wheel_deg` p50/p90 = **Nullpunkt-Qualitaet** (heute 2,3°/6,0°) und
  `p99 |Δaz_out|` + Band gegen das 1-s-Mittel des Pfads (Ziel **±0,5°**)
- [ ] Kurven: Latenz (Kreuzkorrelation gegen den Pfad) **< 150 ms**
- [ ] Spruenge (`_jumps.ps1`) und Positionsspruenge — unveraendert?
- [ ] Baseline: `uni_len` gegen die Referenz, `baseline_dev_pct`, die Suggestion
- [ ] Log in den **Referenz-Satz** aufnehmen (`00_ueberblick.md`) + PATCH-Versionen erhoehen

## 5. Zahl -> Entscheidung
| Ergebnis | Entscheidung |
|---|---|
| `mdev`/`mgate` mit Guidance niedrig | Trim-Fix **abgenommen**, Standard lassen |
| `mdev`/`mgate` hoch | Trim korrigiert falsch ⇒ Vorzeichen/Bereich pruefen (B10.24) |
| `wheel_deg` auf Geraden **<= 1°** | Nullpunkt gut ⇒ **Zaun mit k≈3 + relativem Slack einschalten** |
| `wheel_deg` auf Geraden **> 2°** | Nullpunkt bleibt die Wurzel ⇒ Anschlag-Kalibrierung/VWAS |
| `zo_res` klein (< 5°) bei `zo_st=2` | Vertrauensschwelle 5° traegt (B10.23) |
| `zo_res` 10...30° bei `zo_st=3` | Ziel-Fall bestaetigt ⇒ der Trim ist die Bruecke |
| `p99 \|Δaz_out\| <= 0,5°` | Ziel des Betreibers erreicht |
| Kurvenlatenz > 150 ms | Zaun zu eng ⇒ k erhoehen |

## 6. Werkzeuge
`tools/analysis/_l1010.ps1` (Kennzahlen-Panel) · `_jumps.ps1` (Spruenge) · `_vwas.ps1`
(VWAS-Ereignisse) · `HrLog.ps1` (Spaltenuebersicht) · `%TEMP%\_fence*.ps1` (Zaun/Treue,
pfadbasiert klassifiziert) · `_fieldwheel.ps1` (Nullpunkt auf Geraden).

**Neu 2026-11-07** (alle im Sim-Repo unter `tools/analysis/`):
* `att_why.ps1` — leitet ab, WARUM eine Epoche gedrosselt war (`qf`, `uni_hdgsd`,
  `baseline_dev_pct`); warnt, wenn ein Frame ohne bekannte Klausel gedrosselt ist (B10.92).
* `out_vs_path.ps1` — Ausgabe gegen einen unabhaengigen Kurs; mit `-MaxRateDegS 2` nur
  Geradeaus, das trennt Nachlauf von echtem Fehler (B10.89).
* `L_eff_bins.ps1` — effektiver Radstand je Log, zwei Wahrheiten, mit Signalschwelle (B10.86).
* `snap_compare.ps1` — Vorher/Nachher zweier Sim-Laeufe (Spruenge, `out_snap`, Klemmen) (B10.88).

**Sim-A/B-Schalter** (Umgebungsvariablen, `tools/sim/main.cpp`): `SIM_HR_OUT` (Szenario-Fahrt
loggen), `SIM_OUT_RESID_FACTOR`, `SIM_OUT_MA_MS`, `SIM_VEH_TYPE`, `SIM_ANT_PIVOT`,
`SIM_ART_K`, `SIM_ART_BOUND`, `SIM_ART_APPLY`.


## 7. Feldprobe B10.47 / B10.48 (2026-10-18)

**Zwei Fragen, zwei Antworten:**

| Frage | Antwort im Log/in AOG |
|---|---|
| Kippt die Ausgabe unter der Bruecke noch? | kein `az_out`-Schritt > 90 Grad in 0,5 s (ausser einer ECHTEN Wende: dann ist `az_out = att`) |
| Legt AOG den Ursprung noch bei 0/0? | AgIO-NMEA-Monitor zeigt beim Kaltstart `$PAOGI,<zeit>,,,,,<fixQ>,...` (leere Felder), AOG-Karte steht sofort am Hof, kein Versatz |

**Vorbereitung**
- [ ] **Ganzes Board** kalt starten (GPS-Strom weg, nicht nur Teensy) - nur so kommen die 0/0-Frames
- [ ] Log-Name mit **" 0 "** (z. B. `2026-10-xx hh-mm 0 Bruecke Feldprobe`) - daran erkennt die Auswertung die Kaltstart-Faelle
- [ ] Im Webinterface: Basislinie steht auf dem mechanischen Wert (0,711 m) - Warnung `baselineRefSuspect` darf **nicht** mehr kommen

**Was fahren (je >= 30 s, Uhrzeiten notieren)**
- [ ] **Bruecken-Strecke wie 14-59** (Bach/Bruecke/Baum), 2x, einmal langsam (< 2 m/s)
- [ ] **"heading is a mess"-Strecke wie 15-03** (Baum/Bruecke langsam)
- [ ] **15-12-Strecke** (Baeume, unruhig) - Vergleichsfahrt zum Referenz-Satz
- [ ] optional: **Kaltstart in der Halle** (12-51-Fall: 26 s ohne Fix) - die Ausgabe darf nicht 0/0 senden

**Sofort-Checks (waehrend der Fahrt)**
- [ ] AOG zeigt nach dem Kaltstart **keine** Position bei 0/0 und keinen Sprung "weit weg"
- [ ] Heading unter der Bruecke: bleibt **stehen** statt zu kippen; nach der Bruecke folgt es der echten Richtung (auch wenn das ein grosser Schritt ist - das ist die Wahrheit, nicht der Fehler)
- [ ] Kein "heading flip"-Zucken mehr in AOGs Heading-Anzeige

**Abbruch / Rueckfall**
- AOG-Ursprung trotzdem falsch (leere Felder werden als 0 gelesen) ⇒ in `NAVProcessor.cpp` die Formatter bei `!posOk` **gar nicht senden** (`return false`) - eine Zeile je Funktion
- Ausgabe friert unter der Bruecke laenger als ~6 s ein ⇒ `kRingReanchorMs` (300 ms) / `kRingStepFloorM` (0,50 m) pruefen

**Auswertung (mache ich aus dem Log)**
- `az_out`-Schritt max und p99 je Frame und je 0,5 s (Ziel: < 90 Grad, ausser echte Wende)
- Halten-Streaks (`az_src = 3`) bei v > 0,5 m/s (Ziel: <= 6 s, nur bei echter Stoerung)
- `ring_rej` / `ring_rean` aus den `PH,`-Zeilen (Verhaeltnis: Verwerfungen nur bei Stoerungen)
- `bd` (`baseline_dev_pct`): gute Phase +-2 %, Stoerung ~195 %; `baselineRefSuspect`-Warnung darf fehlen
- `pos_age_ms` = 4294967295 solange kein Fix (Beweis: 0/0 kam nicht in die Kette)

## 8. Log-Aufzeichnung: der Schalter `AIO_HR_LOG` (seit 2026-10-19)

In `platformio.ini` (`[env:teensy41]`), **vor dem Flashen** setzen:

| Wert | Was wird geschrieben | Wann |
|---|---|---|
| **0** | nichts | **Produktivbetrieb** |
| 1 | `HR,` (Heading/Roll; 121 Namen = 120 Werte, ~700 B je Fix) | nur Heading-/Roll-Fragen |
| 2 | `PH,` (Fix/Ring/VWAS; 27 Werte) | nur Positions-/Ring-/VWAS-Fragen |
| **3** | **beide** | **Feldprobe** (Standard fuer die Auswertung) |

**Regeln (aus B10.49, teuer gelernt):**
- Beide Zeilen sind **blockiersicher**: sie werden in einen `DMAMEM`-Puffer gebaut und nur geschrieben, wenn die **ganze** Zeile in den USB-Puffer passt (`availableForWrite() >= Laenge + 16`). Ein halber Satz darf nie auf die Leitung.
- Die **HR-Zeile wird in vier Haeppchen zu je 30 Argumenten** gebaut. 120 Argumente in **einem** `snprintf` kosteten ~1 KB Stack (RAM1 hat nur 3,2 KB frei) und liessen das Board **neu starten, bevor die erste Zeile fertig war**. Symptom: `HR_HDR` kommt noch, dann Bootloop - und **nur mit angeschlossenem Terminal** (ohne Host lief es).
- Der **Sim sieht diese Fehlerklasse nicht** (kein Stack, kein USB, keine Taktung) - Log-Aenderungen immer am Feld pruefen.
- Zwei Zeilen bei 10 Hz sind ~8 KB/s auf der seriellen Schnittstelle: Aufzeichnung mitlaufen lassen, aber nach der Fahrt `AIO_HR_LOG` wieder auf **0** und neu flashen.
- Auswertung: `tools/analysis/HrLog.ps1` (Spalten ueber `HR_HDR.current.txt`), Einstieg `tools/analysis/_l1010.ps1`.



## 9. REFERENZSATZ 2026-10-26 (Standard-Logs ab jetzt)

**Beschluss 10-26 (Betreiber):** die sechs Logs vom 26.10. sind der **Standard-Referenzsatz**.
Begruendung: sie haben **alle 127 Spalten** (inkl. `yr_raw` und `cr_resid`), decken viele
Fahrten **und** alle Empfangszustaende ab, und sind EINE Sitzung (t laeuft 16,7 s -> 1449,8 s).

| Log | was es abdeckt |
|---|---|
| `2026-10-26 16-00 0 hof bach bruecke baum reboot seitl Sprung bruecke` | Kaltstart, **Reboot bei t=81,5 s waehrend der Fahrt (5,8 m/s)**, Bruecke mit Positions-Fallback |
| `2026-10-26 16-03 baum bruecke positionssprung` | Baum/Bruecke, echter Positionssprung (3,16 m), cr_resid 18 Grad |
| `2026-10-26 16-07 bach baeume 2ms gut leichte schraegfahrt leichtes heading rauschen` | gute Lage (fixQ 4 durchgehend, uni_hdgsd 0,66) |
| `2026-10-26 16-11 baeume autosteer leichte schraegfahrt` | Autosteer, Schraegfahrt 12-23 Grad durch EINGEFRORENE f2f-Quelle |
| `2026-10-26 16-13 k vr falsch` | K-Wende, **442 Re-Seeds 133-154 Grad**, Roh-Lage driftet bei hdgsd 0,6 |
| `2026-10-26 16-32 Baeume Schleichfahrt` | Schleichfahrt 524 s (v p50 0,24), 91 % unter 0,6 m/s |

**Baselines (nur Geraden |yr|<1 Grad/s und v>1 m/s, Werkzeug `_n1026e.ps1`):**

| Log | Rate p99 gut / schlecht | Band p90 gut / schlecht | att_q=0 | cr_resid p90 |
|---|---|---|---|---|
| 16-00 | 9,2 / **324,2** | 3,6 / 5,7 | 33 % | 0,9 |
| 16-03 | (zu wenig Geraden) | - | 37 % | **18,1** |
| 16-07 | 6,8 / 6,0 | 3,9 / 6,9 | 33 % | **0,4** |
| 16-11 | 16,8 / 5,8 | 6,3 / 22,3 | 48 % | 1,0 |
| 16-13 | 35,8 / - | 23,7 / - | 1 % | **22,5** |
| 16-32 | **4,0** / - | **2,9** / - | 13 % | 0,3 |

**Ziele bleiben: Rate p99 <= 1 Grad/s, Band p90 <= 2 Grad.** Kein Log erreicht sie;
der beste (16-32) liegt bei 4,0 / 2,9.

**Vorher-Vergleich** (gleiche Metrik, 10-18-Logs mit 120 Spalten): Rate p99 gut 4,8-33,2 und
schlecht 13,8-30,4 Grad/s, Band p90 3,5-12,5 Grad. Also: **schlechter Empfang deutlich besser
durch die 10-2x-Fixes, gute Lage nur leicht besser.**

### 9.1 Diese Logs im Sim replayen (verifiziert 10-26)

`tools/sim/hr_reader.h` liest sie ohne Aenderung: es sucht `">HR,"` bis zum naechsten `<`, und
die Datensaetze sind durch den LITERALEN Text `<\r><\n>` getrennt (endet auf `>`) - es gibt
**keine** echten Zeilenumbrueche im Dump.

```
pio run -e sim
.pio\build\sim\program.exe "<log>.txt" "<out>.txt" 127.0.0.1 nofeed
```
**Falle:** den Pfad MIT Anfuehrungszeichen uebergeben (Leerzeichen im Dateinamen!). Ohne sie
bekommt der Sim `...\2026-10-26` als Datei, faellt auf die synthetische Szene zurueck und
schreibt gar nichts.

**Treffer-Nachweis 16-13:** Feld 537 Saetze / Sim 536; |az_out Sim-Feld| p50 **0,1 Grad**,
p90 1,8; Rate p99 35,4 gegen 35,4 Grad/s; Quellen 0:100 % in beiden. Der Sim bildet den
Fehlerfall also 1:1 ab - **P1 ist damit offline messbar**.

### 9.2 Spaltenindizes (am 127-Spalten-Log verifiziert)

Wertindex = Zeilennummer in `HR_HDR.current.txt` minus 1. Geprueft: `c[24]=baseline_m 0,727`,
`c[86]=uni_len 0,716`, `c[87]=uni_hdg`, `c[89]=uni_hdgsd 0,53`, `c[92-1]=att_q`.
`cr_resid` = c[126], `yr_raw` = c[125].
## 10. FELDPROBE 2026-11-08 — die erste Fahrt mit der VWAS-Reparatur (B10.81…B10.85)

Diese Fahrt hat EINEN Zweck: pruefen, ob die vier Feldbefunde vom 11-07 wirklich weg sind.
Alles andere (Zaun, Trim, Latenz) ist heute nicht dran — nicht ueberladen.

### 10.1 Vor dem Losfahren (5 Minuten, in dieser Reihenfolge)
1. [ ] Flashen. **WARNUNG**: die Defaults sind jetzt **Traktor** (Radstand **2,50 m**, Typ 0,
   40 Grad, Offset 90) — nicht mehr die Maschine. Ohne die Werte unten rechnet die Firmware
   die Kruemmung **2,7-fach falsch**.
2. [ ] **ZUERST die Basislinie** (B10.91/B10.92): am 11-07 waren **24 % der Frames
   gedrosselt** (`att_q = 0`; Achtung: `att_q = 1` heisst SAUBER), davon **542** wegen der
   Basislinien-Abweichung. Die Referenz steht im Boot-Log: `physicalBaselineM: ... using the
   built-in default 0.711 m`. Auf der Bruecken-Geometrie-Seite die **gemessene** Laenge
   eintragen (11-07 geloest: **0,83 m**) oder **0 = lernen**. Danach im Log pruefen:
   `|baseline_dev_pct| < 5 %` — **sonst sind alle anderen Zahlen hier Messwerte aus dem
   gedrosselten Zweig.** Auswerten mit `tools/analysis/att_why.ps1` (leitet den Grund aus
   `qf`, `uni_hdgsd` und `baseline_dev_pct` ab).

3. [ ] `/vwas` oeffnen. **Die Seite MUSS jetzt laden** (vorher 503). Wenn sie leer bleibt, ist
   die Reparatur nicht im Binary → nicht weiterfahren, Firmware-Stand pruefen.
4. [ ] Eintragen und **speichern**: Radstand **0,92** · Antenne vor Achse **0,41** ·
   halber Radstand **0,84** · Typ **2 (Knicklenker)** · max. Lenkwinkel **75** ·
   Dual-Offset **88,3**. Danach **Seite neu laden** und pruefen, ob die Werte stehen
   geblieben sind (das war der eigentliche Fehler: nichts liess sich speichern).
5. [ ] Einmal **Strom aus/ein**. Werte muessen aus dem EEPROM zurueckkommen.
6. [ ] Log starten (seriell, `AIO_HR_LOG`), **10 s Stillstand** als Referenzblock.

### 10.2 Was fahren (kurz halten, 6 Minuten)

| # | Manoever | Dauer | Was es beweist |
|---|---|---|---|
| A | Stillstand, Raeder gerade | 20 s | `veh_type=2`, `wq=0` (nicht 100), `az_out` ≈ 0 |
| B | Geradeaus 2…4 km/h | 60 s | `pc_ok` > 15 %, `rl_clip` deutlich unter 88 %, kein −91 Grad |
| C | 2…3 **scharfe** Wendungen (Hand) | 30 s | Knick wird publiziert; `|Δaz|/|Δatt|` nahe 1 in der Wendung |
| D | Optional: unter Baeumen | 30 s | Zustandswechsel ohne Sprung im Knick |

### 10.3 Abnahme (drei Zahlen, alle im Log, alle vorher/nachher bekannt)
| Groesse | Vorher (11-07) | Erwartung | Wenn nicht erreicht |
|---|---|---|---|
| `veh_type` | **0 in allen Frames** | **2** in allen Frames | Geometrie kommt nicht an → Save-Hook/EEPROM pruefen (B10.81) |
| `wq` + `wheel_deg` | **100 / −91,0 Grad** | **0** und kein −91 mehr | Plausibilitaetsnetz greift nicht → `maxSteeringAngle` im EEPROM? (B10.82) |
| `rl_clip` | **88 %** (Replay 84,7 → 27,9) | **≈ 25…30 %** | Halte-Deckel rechnet noch mit falschem L → Geometrie (B10.81) |
| `pc_ok` | **15,6 %** (Replay 20,6) | **≥ 20 %** | erwartbar; mehr geht laut B10.53 nicht ohne neuen Ansatz |
| `|Δaz|/|Δatt|` in Wendungen | **40 %** | **> 80 %** | Deckel/Rampe pruefen — aber **nur** mit gespeicherter Geometrie messen |

### 10.4 Was diese Fahrt NICHT klaeren kann
* **`guid=0`**: solange AOG nicht lenkt, ist jede Aussage ueber „Autosteer-Qualitaet"
  ungueltig (am 11-07 war `guid` in **allen** Frames 0 — die Fahrt war handgelenkt).
* **Kurvenverlust**: 18,6 % der Kurvendrehung wurden nicht publiziert — davon 221 Grad in
  **9 INIT-Frames** (Anlauf) und der Rest als **0,5 Grad/Frame** ueber LIVE verteilt, ohne
  Deckel (Ratenklassen breit verteilt). `rl_path` war in **0 von 3638** Frames aktiv, d. h.
  die alte Vermutung „Pfad-Begrenzer frisst echte Kurven" ist **widerlegt**. Offen bleibt nur
  die normale Filter-Nachlaufzeit — kein Fehler, kein Handlungsbedarf.
