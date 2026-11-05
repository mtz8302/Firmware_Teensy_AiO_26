# Symptom-Register: Bericht → bekannte Ursachen → Prüfung

**Version 1.0.0 · 2026-10-11 · gilt fuer Logs 2026-09-20 bis 2026-10-11**
Aenderungen: v1.0.0 (10-11) Erstanlage aus den Betreiberberichten 10-09 bis 10-11 +
Chronik-Rueckmarsch (September bis 10-02).
Versionierung: MAJOR Struktur, MINOR Inhalt, **PATCH vor jedem Log-Lauf (Sim oder Feld)**.

**Erster Ort für jeden Feldbericht.** Verhindert, dass eine Ursache neu gesucht wird, die schon
belegt (oder schon gefixt) ist. Ablauf: Symptom hier nachschlagen → die gelisteten Prüfungen
mit *einem* Log und *wenigen* Zahlen fahren → erst dann fixen (Workflow Tor 1).

## S1 „Der Traktor fährt schräg zur Spur" (Schrägfahrt)
| # | Ursache | Prüfung | Stand |
|---|---|---|---|
| a | `psi` ist falsch (Hauptursache: sichtbare Schräglage = psi-Fehler) | `_l1010.ps1` errTrack p50/p90; `skew.ps1` travel−psi | B1.2, B2.1 |
| b | `rl_clip` hält den Fehler am Leben (Rückkopplung) | `skew.ps1` Split geclippt/un-geclippt, `rl_clip`-Spalte | B2.2 |
| c | Modell dreht in Kurven 1,1–1,7× zu weit | `curve_proportionality.ps1`, `_scaleopt.ps1` | B2.3 + offener Widerspruch |
| d | Montageversatz falsch/gewandert | `skew.ps1` travel−att (Soll 88,3 ±1) | B1.1, B1.10 |
| e | Roll falsch/verspätet (AOG macht daraus Querversatz) | `roll_check.ps1`, `rollout.ps1` | B3.5 |
| f | **WAS-Nullpunkt falsch** (Keya startet bei 0) → Modell dreht dauerhaft | Lenkwinkel-p50 auf geraden Epochen (Soll 0), `steering_angle_from_circle.ps1` | B3.7, S6 |

## S2 „Die Kurve wird nicht gefahren"
| a | Ausgabe kam aus `psi`, während die Lage drehte (176° vs. −95°) | `_l1010.ps1` + `att`/`az_out`/`out_lim` | B2.6, Fix A+B2 drin |
| b | Pfad-Begrenzer frisst die Kurve (`rl_clip` 24…51) | `_sim_manoeuvre.ps1`, `_oldturns.ps1` | B2.7 — **OFFEN (Phase 2)** |
| c | Modellskala (1,00 gegen 1,30) | Widerspruch in `00_ueberblick.md` §6.9 | offen |
| d | Richtungsentscheidung spiegelt (rev) | `kturn.ps1`, `reverse_sign_check.ps1` | |

## S3 „Positionssprung"
| a | Empfängerausfall (`fixQ` weg, `uni_hdgsd` 93,7, `att_q` 0) | `_jumps.ps1`, `_steps.ps1` (`raw_step` gegen `pub_step`) | 10-11 7-01 belegt |
| b | Projektionsumschaltung (`publim` 1→0) | `projqc.ps1`, `_jumps.ps1` | B1.15 |
| c | Roll-Sprung → Querversatz in AOG | `_rollstep.ps1` (Höhe·sin(roll)) | B3.5 |
| d | Werkzeug-Fehlsignal (falsch skalierte Sprungerkennung) | `_l1010.ps1` §3 | gefixt 10-11 |

## S4 „Unter der Brücke / unter dem Baum ist es total off" (Wegdrehen, Heading verloren)
| a | Attitude schlecht UND die Ausgabe folgt ihr (LIVE-Gate zu weich) | `_l1010.ps1` §1: Fehler in schlechten Epoch vs. gute; `src`/`state`-Zählung | 10-11 Verdacht |
| b | Quelle fällt auf `psi`/Modell zurück und dreht weg | `_spin.ps1`, `_gapdump.ps1` | B2.6 |
| c | Kein Kurs-Träger (Ring) in der Lücke | `_sim_yaw.ps1`, `_fixlen.ps1`, `_hsources.ps1` | B1.4 |
| d | Modell dreht mit falschem WAS-Nullpunkt (5,6° → 0,9 °/s) | S1f | B3.7 |

## S5 „Heading verwackelt" (Gerade 20 km/h)
| a | Dual-Heading jittert 2–6°/Frame (Empfänger meldet 0,32–0,55) | `_noise.ps1`, `spike_shape.ps1`, `_band57.ps1` | B1.12 |
| b | Latenz der Kette (`rollLpfTauS`) | `_lag.ps1` (Kreuzkorrelation) | B1.13 |
| c | Filterstärke nach falschem Kriterium gestaffelt (Tempo statt Datenlage) | `_sim_stage.ps1`, `_sim_filters.ps1` | 01_ziele.md D6 |

## S6 „Lenkwinkel ist x Grad off" — VWAS (gekoppelt an das Heading)
| # | Ursache | Prüfung | Stand |
|---|---|---|---|
| a | **WAS-Nullpunkt** (Keya-Encoder startet bei jedem Start bei 0, ohne Bezug zu den Rädern) | Lenkwinkel-p50 auf geraden, sauberen Epochen (Soll 0); Nullpunkt über die Fahrt stabil? | B3.7, WORKFLOW.md §8.1 |
| b | Skala (`L_implied` über die Winkelbins nicht konstant) | `steering_angle_from_circle.ps1`, `circles.ps1` | B1.16 |
| c | Modell-Bias statt echter Nullfehler | `bias_runaway.ps1`, `sc_b` (Soll 0) | I3 |
| d | **Scheinbar falscher Lenkwinkel — Ursache ist das HEADING** (09-27: implizierte Radstände 1,0–4,8 m bei 2,14 m) | erst `_hsources.ps1`/`_straight.ps1` prüfen, dann den Sensor | B1.16 |
| e | VWAS rechnet aus dem Heading (**Zirkelbezug**) | Code-Audit `WheelAngleFusion`: kein `heading`/`az_out` als Eingang; Nullpunkt nur aus Rohfix | I11 |
| f | Roll beeinflusst den Lenkwinkel (k = Grad LW je Grad Roll) | `_kfit.ps1` | `_kfit.ps1` |
| g | Qualität zu niedrig ⇒ Modell muss schweigen (`yaw_zeroed`, `yr = 0`) | `wq` gegen `wheelQualityLowPct` | I10 |
| h | Virtueller Lenkwinkel aus dem Keya-Encoder ungenau (Getriebe/Übersetzung) | `_vwas.ps1` (Event-Log-Zeilen), `steering_angle_from_circle.ps1` | offen |

## S7 „Roll springt"
| a | Roll-Gate öffnet ohne Attitude (`roll_susp`) | `roll_check.ps1` + `roll_src`/`roll_susp` | |
| b | IMU-Roll Achsen/Einheiten | `imu_vs_gnss.ps1`, `_recon2.ps1` | B3.5 |

## S8 „Heading steht falsch / dreht sich weg im Stand"
| a | Richtungsentscheidung kippt (rev 1 trotz Vorwärts) | `revfix.ps1`, `reverse_consistency.ps1` | |
| b | Re-Seed/Sprung nach Ausfall | `_reentry.ps1`, `_steps.ps1` | B2.7 |
| c | **IMU-Ratenpfad aktiv** (BNO-Sägezahn ersetzt die Kinematik) | `imu_presence.ps1` (imu_flags Bit 2, `imurate`), `episode2.ps1` | B2.11 |

## Chronik: Probleme und Lösungen (September bis 11.10.)

Die eigentliche Datenbank. Jede Zeile: was war, was war die belegte Ursache, was haben wir getan,
was kam dabei heraus. **Vor jeder neuen Ursachensuche hier nachsehen** - die meisten Probleme
hatten wir schon (oft in einem anderen Gewand).

| Datum | Symptom/Bericht | Ursache (belegt) | Maßnahme | Ergebnis | Beleg |
|---|---|---|---|---|---|
| 09-22…09-27 | „Lenkwinkel falsch / Radstand stimmt nicht", implizierte Radstände 1,0–4,8 m bei 2,14 m | **Schlechte Heading-Frames**, nicht der Sensor; in Kurven zusätzlich Lever-Arm `atan(ω·a/v)` (~15°) | Qualitätsgate auf `headingQuality`; Truth-Checks nur auf Geraden; NICHT auf sats/σ gaten | gelöst (Werkzeug + Regel) | `steering_angle_from_circle.ps1`, `curve_proportionality.ps1` |
| 09-27 | „Satellitenzahl als Qualität nehmen" (falsche Annahme) | sats bleiben 14–20 unter Bäumen, während die Lösung degradiert; `sigma` ist unsere eigene, aufgeblähte Zahl | Gate über `headingQuality` (RTK-fixed der Moving-Baseline) | gelöst, Regel B1.17 | README tools/heading_bridge |
| 09-30 | „Heading springt/verwackelt" | **21 von 22 Peaks sind Einzel-Frame-Spikes**; der Langsam-Jitter kam aus **unserem eigenen `cog`-Fallback** | **Ratengrenze statt Mittelwert** (Mittelwert 3–12× schlechter bei TWITCH/FOLLOW/SWALLOW) | gelöst als Bauart, aber später wieder in Frage gestellt (siehe 10-11) | `spike_shape.ps1`, `filter_compare.ps1`, `az_peaks_by_src.ps1` |
| 10-02 | **„Heading läuft weg — ohne Bäume, ohne GNSS-Fehler"** (95° in 3,4 s) | `imu_flags` Bit 2: **BNO085-RVC-Sägezahn** (Parser summiert 20 Pakete, Reset) ersetzt die Kinematik komplett; `yawbias` klebt am ±30°/s-Clamp. Radwinkel-Bias war es NICHT (wq 12) | IMU-Ratenpfad zähmen/abschaltbar machen (`useImuRate`, `imuRateInvert`, Bias-Clamp) | Ursache belegt, Fix-Stand prüfen (Pfad existiert noch) | report 10-02 §0 |
| 10-02 | Rückwärts/Schleichfahrt dreht falsch | Heading-Prädiktion nutzt **ungerichtete** Geschwindigkeit, Projektion `dirSign` | Richtung in die Prädiktion (Befund B) | offen | report 10-02 §3/§4 |
| 10-03 | „Baseline/Attitude-Gate sperrt falsch" | Längenmessung gut, **gelernte Referenz** war der Defekt (0,852 m, +18,3 %) | Referenzlernen begrenzen; Länge UND hdgsd behalten | gelöst | baseline §1/§2, `basfix.ps1` |
| 10-04 | „fährt schräg" | `psi` folgt dem Modell, Attitude zieht nicht; `rl_clip` als Rückkopplung | R-Deckel 1,5°, `reseedBeyondDeg` 30, Ringpuffer als Kursquelle | teilweise (W1.x/B2.2) | skewmech, recovery |
| 10-04 | „Modell dreht in Kurven zu weit" | Modell 1,1–1,7× (k bis 2,6) | `effWheelbaseFac` 1,30 | **wieder verworfen 10-06** (23 % zu wenig Gierrate) → Widerspruch, offen | skewmech §5, HeadingBridge.h |
| 10-05 | „ohne RTK genauso gut" | relatives Positionsrauschen ist RTK-unabhängig (p50 0,5 cm) | `posRelativeNoiseM` 0,02 statt absoluter σ im Ring-R | gelöst, pfadverankerte Quellen trugen (err_psi −0,18°) | recovery 10-05 |
| 10-05 | „IMU-Roll unbrauchbar" | **falsch** — Einheiten sind Grad (Faktor 1), Achsen getauscht | Roll-Kanal nur mit frischen Daten, Roll+Position zeitlich ausrichten | gelöst (Korrektur des Befunds) | imu_roll §0/§1 |
| 10-09/10-10 | „90°-Kurve wird nicht gefahren / schräg" | `rawAz` fiel auf `psi` zurück (176° Lage vs. −95° publiziert, `out_lim`=0); plus `rl_clip` 24–51 frisst 141–151° | Fix A+B2 (rawAz immer Dual-Lage bei qfOk); Manöverterm vorgeschlagen | A+B2 drin, Manöverterm offen | 10-10 §1/§2.12 |
| 10-11 | „unter der Brücke total off", „1,5 km/h schräg" (17,1°!) | Ausgabe hängt zu 92 % an `att`, schlechte Epochen 10–24° daneben; WAS-Nullpunkt 5,6° (Betreiber) | LIVE-Gate/Kurs-Träger härten, WAS-Nullpunkt aus Rohfix | offen, zu entscheiden | 00_ueberblick.md §5, 80_plan.md §6 |
| 10-11 | „Positionssprung" in 7-01 (3,17 m) | Empfängerausfall: `uni_hdgsd` 93,69, `att_q` 0 — **nicht** unsere Projektion | Projektionsumschaltung ausschließen (I4) | belegt | `_jumps.ps1`, `_steps.ps1` |
| 10-11 | Werkzeug meldete „43,8 % Sprünge" | Sprungschwelle `max(0,5·v·dt+0,15; 0,45)` war für >6 km/h falsch skaliert | Schwelle auf `max(1,5·v·dt+0,15; 0,40)` | gelöst, alte Zahlen unbrauchbar | `_l1010.ps1` §3 |
| 10-11 | S4/S8-Kandidat „IMU-Ratenpfad (BNO-Sägezahn)" — **ausgeschlossen** | `imu_flags = 0`, `imurate = 0,00` in ALLEN Frames der drei Logs | keine Maßnahme nötig | **negativ belegt** (spart die ganze Suche) | `imu_presence.ps1 -Filter '2026-10-11*'` |
| 09-07/09-08 | **Beginn der Arbeit** (ältester Beleg). VWAS vorhanden, aber bei Schritttempo unbrauchbar: `gpsAngle` sättigt bei ±40°, `enc` zeigt −7,65° bei 0,86 m/s; im Stand 44 s lang alles 0 | GPS-Winkel aus nahezu stehender Position ist Rauschen; Encoder noch ohne Nullpunkt | Nullpunkt-Lernzustand in `WheelAngleFusion` bauen | 09-21 in Betrieb; am 09-08 noch **0** `VWAS zero:`-Zeilen | `_vwas.ps1`, `Log virtual WAS 2026-09-08.txt` |
| 09-21 | **Lenkwinkel-Kalibrierfahrten** (`0-re-li-0`, `20-re-li-0`, `AnschlagKali`, Halle, Holperstrecke) | **WAS-Nullpunkt 20–23° falsch** (`enc −23,28` gegen `gpsAngle +0,81` bei 1,34 m/s, RTK) | Lernzustand (`trim`/`used`/`gain`) getunt; Anschlag-Kalibrierung | Lernschritt zu klein: `res = −21,02`, aber nur `used = 1,02` bei `gain = 0,053`; später `trim = −8,67, gain = −1,274` | Logs 09-21 16-13/16-41/19-50 |
| 10-11 | Werkzeug `_vwas.ps1` las **1 statt 320** Zeilen | Event-Logs haben nach dem Trenner `<\r><\n>` **noch ein echtes CRLF** → `StartsWith('[')` scheiterte | `TrimStart([char]13,[char]10)` je Teil | gefixt, ab jetzt vollständig | README „Fallen" |
| 09-21 | **Keya-Nullpunkt nach Neustart** | Der Motor-Zähler beginnt mit dem Motor neu (gemessen: Position ~65232 vor, ~495 nach einer Motor-Unterbrechung, **Räder unverrückt**); gespeicherter Mittelwert + frischer Zähler ergab **−54,6°** bei ~15° Radeinschlag | Position beim Einschalten = **Geradeausstellung** (bewusste Policy) + Korrektur per GND-Taste / Web / AOG-Nullklick | gelöst, im Code begründet (B7.7) | `WheelAngleFusion.h:385-400`, Log 15-28 |
| 09-23 | Log-Flut / serielle Bandbreite | 10-Hz-VWAS-Zeilen sind 296 Zeichen lang | VWAS-Zeilen auf **0,5 Hz** | gelöst (Betreiber-Entscheidung) | `WheelAngleFusion.cpp:247/275` |
| 09-24 | Nullpunkt-Trimmer zu langsam | Die Roll-Streuungs-Skalierung drosselte den Trimmer: **0,71 statt 5,0 °/s** → 13,7° Korrektur brauchten 38 s statt ~3 — und auf **glattem** Boden wurde es schlimmer | Faktor **entfernt** statt neu getunt (nichts mehr zu schützen, Gain kommt aus der Config) | gelöst (B7.11) | `WheelAngleFusion.cpp:1410-1428` |
| 09-27 | Betreiber-Frage: **„Zirkelpfad?"** | Der Bridge sagt `psi += v·tan(δ)/L` mit **unserem** Lenkwinkel voraus, sein `steerBiasDeg` absorbiert genau den Bias des Beobachters; `bA` ist eine Differenz | Gate `headingIsMeasured`: getrimmt wird nur eine **Messung**; **LIVE allein genügt nicht**, das Modell-Gate hebt bis ~600 ms die Unsicherheit | abgeriegelt und gemessen belegt (`bias_deg` ±0,5°, folgte dem Trim −0,85° NICHT) | `WheelAngleFusion.cpp:1487-1518`, B2.15 |
| 09-28 | Heading dreht, **obwohl beide Empfänger RTK-fixed** | Ein bei Einschalten auf die Radposition gesetzter Keya-Nullpunkt gab **−31° „Lenkwinkel" bei Geradeausfahrt = 50 °/s erfundene Drehung** | `getAngleQuality()` 0…100 als Modell-Eingabe, bewusst ohne Zeitverfall | gelöst (B2.14, B7.14) | `WheelAngleFusion.cpp:1762-1773` |
| 09-29 | Rückwärts / Kurven-Vorzeichen | Die Fahrtrichtung wird für das Krümmungs-**Vorzeichen** gebraucht; Sign-Korrektur auch im Knicklenker-Pfad | Richtung aus der Position (positionszeugen-geschützt) | gelöst | `WheelAngleFusion.cpp:531/678/742` |

