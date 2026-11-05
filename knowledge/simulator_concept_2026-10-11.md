# Simulator-Konzept (2026-10-11) — virtuelle Fahrt aus echten Logs

## 1. Grundsatz (Betreiber-Entscheidung, oberste Regel)
**Der Simulator verwendet den ECHTEN Code** (`HeadingBridge.cpp`, `PositionHistory.cpp`,
spaeter `WheelAngleFusion.cpp`) — **nichts wird nachgebaut.** Eine Implementierung der Logik;
der Simulator stellt nur deren **Aussenwelt** bereit. Damit kann die Simulation nicht von der
Firmware abweichen (die bisherigen Skript-Nachbauten konnten das — und genau daraus entstanden
die Fehldeutungen dieser Sitzungen).

## 2. Grenze des Systems — Stufen (Schnittstellen von Anfang an vorsehen)
Zwischen `HeadingBridge` und `WheelAngleFusion` gibt es eine **echte Rueckkopplung**: die VWAS
liefert den Lenkwinkel an die Bridge, **die Bridge liefert Heading + Roll an die VWAS**.

| Stufe | geschlossen | Eingang | Zweck |
|---|---|---|---|
**S1** | `HeadingBridge` + `PositionHistory` | **Lenkwinkel + `wq` aus dem Log** (VWAS offen) | Heading-Logik: Quelle/Gate/Trust, Projektion, Spruenge, Ueberbrueckung |
**S2** | + `NAVProcessor` (PAOGI) | wie S1 | "was sieht AgOpenGPS" |
**S3** | + `WheelAngleFusion` (geschlossen) | **Roh-Lenkwinkelquellen** — stehen **nicht** im Log | die Befunde *in* der VWAS (LW 5,6 Grad off, Modell ~30 % zu niedrig) |

Die Bridge wird **nie direkt an Log-Spalten** gekoppelt, sondern an eine Eingangsschicht
(`sim_inputs`), die spaeter von der echten VWAS-Ausgabe gespeist werden kann ✓

## 3. Stimulus — zwei Betriebsarten
| Modus | Quelle | Wahrheit | Zweck |
|---|---|---|---|
**Replay** | echtes Feld-Log | keine (Vergleich mit Original) | Treue-Eichung + "was haette der neue Code getan" |
**Szenario** | synthetisch | **bekannt und exakt** | **echte Verifikation** + Regressionstests |

Szenarien: (1) gerade 2,5 m/s mit Rauschen, (2) **Bruecke/Baum**: Attitude bricht N s weg
(verrauscht/eingefroren), Positionen bleiben gut, (3) **90-Grad-Kurve** mit bekannter Drehung,
(4) **Schleichfahrt 0,2 km/h** unter Baeumen, (5) **60 km/h gerade** (Grenzfall der Schranken)

## 4. Ausgabe (schreibt die virtuelle Fahrt eine Datei? — ja)
1. **`<name>.virt.<variante>.txt`** in **demselben 101-Spalten-HR-Format** ⇒ **alle vorhandenen
   Werkzeuge lesen es ohne Aenderung** ✓ (Kern des Nutzens)
2. **`<name>.virt.<variante>.json`** — Rausch-sigma, Schritte >1/>3 Grad, max Schritt,
   Kurvenverluste, Spruenge, **Quellen-Mix** ⇒ Delta-Tabelle Variante A/B/Feld
3. **`<szenario>.truth.txt`** (nur Szenario) — Wahrheit je Zeitstempel ⇒ exakte Fehler
4. **`fidelity.txt`** (Eichlauf) — Spalte-fuer-Spalte-Vergleich des **unveraenderten** Codes
   gegen das Original **+ Liste der nicht rekonstruierbaren Spalten** ⇒ was nachgeloggt werden muss

## 5. Testablauf
```
1 Stimulus:   echtes Log ODER Szenario
2 Config:     Parameter als Argumente (Default = aktueller Stand)
3 Bauen:      Host-Target (g++/platformio native), Code-Stand = Arbeitsstand
4 FAHRT:      Stimulus -> [PositionHistory | HeadingBridge] -> virt-HR + JSON (+ truth)
5 Auswertung: vorhandene Skripte auf das virtuelle Log
6 VERGLEICH:  Variante A/B/Feld -> JSON-Delta
7 erst danach: Firmware bauen; Feldtest nur als Bestaetigung
```
Ein Befehl: `tools/sim/sim.ps1 -Stimulus <log|szenario> -Variante <name> [-Config k=v]`

## 6. Vertrauen (ohne das ist alles wertlos)
- **Eichlauf:** unveraenderter Code + echtes Log ⇒ **"Treue je Spalte"**
- Erst wenn `psi`, `az_out`, `state`, `publim`, Position stimmen, sind Variantenvergleiche belastbar
- Der Eichlauf liefert die **Liste fehlender Logs** ⇒ nachloggen ⇒ Treue steigt
- Grenzen (im Bericht ausgewiesen): offene Schleifen ausserhalb der Bridge (Lenkung/Treiber),
  fehlende Roh-Eingaenge (S3)

## 7. Dateistruktur
```
tools/sim/  main.cpp (Tick-Schleife)  sim_inputs.* (Schnittstelle!)  hr_reader.*  hr_writer.*
            metrics.* (JSON)  scenario.* (Szenarien+Wahrheit)  stubs/ (millis, Serial, LOG_*,
            FLASHMEM)  Makefile  sim.ps1
```
Geplant als zusaetzliches PlatformIO-Environment `[env:sim]` (`platform = native`).

## 8. Stub-Liste S1 (gemessen: Abhaengigkeits-Inventar)
`millis()` 15x · `FLASHMEM` 37x (Makro->leer) · `Serial` 4x · `LOG_*` 44x ·
**`gnssProcessor` 45x** (Hauptarbeit: `getData()`-Struct aus dem Log fuellen) ·
`imuProcessor` 11x (inaktiv) · `WheelAngleFusion` 12x (LW aus dem Log) ·
`configManager` 17x (Defaults/Argumente) · `PositionHistoryService` 2x.
**`micros()` und `PROGMEM` kommen nicht vor** ✓

## 9. Erste Schritte S1
1. Abhaengigkeits-Inventar ✓ (oben) → Stub-Liste
2. `sim_inputs` + `stubs/` + `main.cpp` + Host-Build ⇒ **kompiliert**
3. `hr_reader`/`hr_writer` (Format exakt) + Tick-Schleife mit echten Zeitstempeln
4. **Eichlauf** gegen 10-09/10-11-Logs ⇒ Treuebericht ⇒ nachbessern
5. `sim.ps1` + JSON ⇒ erste Variantenvergleiche; danach S2 (Szenarien) und S3 (VWAS)

## 10. Was der Simulator sofort beantwortet
- "Kurve rausgekommen?" mit bekannter Wahrheit ⇒ ja/nein je Variante
- "Wie lange traegt eine Bruecken-Blankung, wie weit driftet die Position?" — exakt
- **"Welche Quelle hat die Traeger-Kette pro Frame gewaehlt?"** — die Frage, an der wir heute raten
- "Hilft alpha 0,1 / Median-Tor / neue Schranke?" ⇒ Sekunden statt Sitzungen
- jede vergangene Fahrt wird ein **Regressionstest**

## 12. ARBEITSBEREICHE (Betreiber-Angabe 2026-10-11) — oberste Randbedingung
| Bereich | km/h | m/s | Bedeutung |
|---|---|---|---|
**Autosteer-Bereich** | **0,1 – 20** | 0,028 – 5,56 | alles, was gefahren wird |
**KERNBEREICH** ⭐ | **6 – 12** | **1,67 – 3,33** | hier muss die Qualitaet am besten sein |
Maximum (Maschine) | 60 | 16,7 | nur Obergrenze, **kein Arbeitsszenario** |

### Folgerungen (geprueft 10-11)
1. **A+B4 (`limRate = 30 + 25*v`) passt** ✓: 0,1 km/h -> 30 Grad/s (Faltung wird sicher abgewiesen),
   Kern 6-12 km/h -> 72-113 Grad/s (echte Kurve braucht 30-60), 20 km/h -> 169 (reale Kurve ~64,
   Faltung >1000 wird abgewiesen). Keine Aenderung noetig.
2. **S-B-Stufen liegen IM Kern** (`stage1SpeedMps` 2,0 = 7,2 km/h, `stage2SpeedMps` 3,0 = 10,8 km/h)
   ⇒ beim Durchfahren des Kerns springt alpha 0,2 -> 0,3 -> 0,5 ⇒ **Verhaltenssprung genau im
   wichtigsten Bereich** ⚠ ⇒ im Simulator mit einer glatten **6->12-km/h-Rampe** pruefen und
   ggf. **alpha stetig interpolieren** statt stufen.
3. Szenarien sind auf **0,1 / 2 / 6 / 8 / 12 / 20 km/h** plus die Rampen 6->12 und 0,1->5 auszulegen
   (60 km/h nur als Randpruefung).
4. Fidelity-Eichung: die 10-09/10-10/10-11-Logs liegen ueberwiegend im Kern ⇒ ideale Unterlagen.

## 13. EICHLAUF-BEFUND (2026-10-11, Replay des 10-10 12-31)

> **WARNUNG (nachgetragen):** die Tabelle unten ist **verfaelscht** - damals lief der
> Szenario-Overlay auch im Replay (siehe Punkt 2). **Gueltig bleibt nur der Rollkanal**
> (±0,37 Grad - roll setzt niemand, also kam er wirklich aus dem Log). Alles andere,
> auch sigma/az, ist erst nach der Bereinigung aussagekraeftig: siehe Punkt 2.

Der Replay laeuft (**1402 Saetze gelesen**). Vergleich **Sim (heutiger Code)** gegen das **Log
(Stand vom 10-10)** — Zuordnung: Sim-Sekunde S <-> Log-Record 10*S (10 Ticks pro Satz):

```
Sek |  az_sim   az_log   dAz   | roll_sim roll_log dRoll | sig_sim sig_log
  1 |  271,80   176,80  95,00  |    0,00    0,00   0,00  |   2,93   30,00
  2 |  271,80   176,60  95,20  |    3,14    3,10   0,04  |   1,16    3,73
  4 |  276,78   162,50 114,28  |    3,76    3,50   0,26  |   1,27    4,49
  8 |  280,45   186,90  93,55  |    3,47    3,10   0,37  |   1,27    4,50
 12 |  275,97   190,80  85,17  |    3,09    3,00   0,09  |   1,84    4,49
```

### Lesart (entscheidend)
1. **Rollkanal ist treu: |dRoll| <= 0,37 Grad** ⇒ Roll-Halt, LPF und Quellenwahl werden 1:1
   nachgebildet ⇒ **der Harness ist im Kern bewiesen** ✓✓
2. **dAz ~ 85...114 Grad war ein SIM-BUG - kein Frame-/Offset-Problem** ✗✗
   (gefunden 2026-10-11): der SZENARIO-OVERLAY in `tools/sim/main.cpp` lief auch im
   Replay und ueberschrieb JEDEN Satz (`dd.dualHeading = s_course - 88.2` -> 271.80 =
   0-88.2+360; `vScen = 0.055`; `feedStraight()`-Position). Nach der Sperre
   (`if (!s_replay) { ... }`) folgen die LOG-EINGAENGE EXAKT:
     rec 100: v/cog/roll = 2.855 / 277.59 / 2.09 (src = virt)  · az 189.4 vs 195.5
     rec 700: v/cog/roll = 7.658 / 111.58 / 1.62 (src = virt)  · az  23.6 vs  23.2
     rec 1401: v/cog/roll = 0.002 / 209.01 / 2.94 (src = virt) · az   6.0 vs   6.0
   ⇒ Die **Eingaenge sind identisch**, die Restdifferenzen (sigma, az, Zaehler,
   Position) sind der **CODE-Unterschied alt↔heute** - genau der Zweck des Werkzeugs.
   Der Reader ist korrekt (`hr_reader.h`: lat/lon = col1/2, Record 0 = 0/0 wie im Log).
   Lehre: Szenario- und Replay-Pfad strikt trennen.
3. **sig_sim (1,16...2,93) < sig_log (3,73...4,50)** ist die **Verbesserung** des heutigen Codes
   (verschaerfter psi-Anker A) gegenueber dem aelteren Stand im Log ⇒ **der Simulator zeigt genau
   die Aenderung** — das ist sein Zweck, kein Treuefehler ✓✓

### Konsequenzen
* **Eingaenge sind jetzt satzgenau** (v/cog/roll identisch zur Quelle) ⇒ jede weitere
  Differenz ist ein **Code-Unterschied** ⇒ Sim-Aussagen ueber alt↔heute sind belastbar.
* **`hr_writer` IST FERTIG** (2026-10-11): Format 1:1 aus `HeadingBridge.cpp:3926-3932`
  (101 Spalten), Argumente aus `Status`/`GNSSData`; `program.exe <log> <out>` schreibt
  `>HR,...<` je Satz (**1402 Saetze, 102 Felder gemessen**) ⇒ **DROP-IN-virtuelles Log
  fuer ALLE Werkzeuge** ✓✓ · Replay bricht am Log-Ende ab (kein Szenario-Anhang).
* Ab jetzt ist der **A/B-Vergleich zweier Code-Varianten auf derselben Fahrt** moeglich
  (Sim-Variante A vs. B vs. Log) — in Minuten statt Feldtagen ✓✓

### Bekannte Harness-Details (fuer den Nachbau)
* Log-Format: **eine einzige Zeile**, Delimiter `>` davor / `<` danach, **0 x `\n`**
  (sonst liest der Reader nur 1 Satz!) — steht so im Kommentar von `tools/sim/hr_reader.h`.
* Rhythmus: **100 Hz Ticks**, alle 10 Ticks ein neuer GNSS-Satz (10 Hz), dazwischen `ageBy(10)`.
* `tools/sim/src/` sind **Kopien** der echten Quellen (sonst gewinnt `#include "..."` den
  Datei-Nachbarn in `lib/aio_navigation` und zieht die echten Teensy-Header nach).

## 14. S2b FERTIG: AgOpenGPS live ueber UDP (2026-10-11)

`tools/sim/aog_udp.h` - Ports und Verhalten **1:1 wie die Firmware**:
* **EMPFANGEN auf 8888** (AgIO/AOG -> Modul; `QNEthernetUDPHandler.cpp:111`)
* **SENDEN an `<AOG>:9999`** (`QNEthernetUDPHandler.cpp:299`, `ConfigManager.cpp:510`)
* Ziel-IP wird aus dem letzten Datagramm **gelernt** (AOG sendet PGNs an 8888);
  Fallback-IP optional per Argument (z. B. PC-Alias `192.168.5.126`, Teensy aus)
* `$PAOGI`-Feldreihenfolge **1:1** aus `NAVProcessor.cpp:338-371`;
  **heading = `status.azimuthDeg`, roll = `status.rollDeg`** (wie `exposeToAog`)
  ⇒ AOG sieht genau das, was die Firmware senden wuerde
* 10 Hz (alle 10 Ticks) + `Sleep(10)` = Echtzeit (100-Hz-Ticks)

Aufruf:
* Szenario: `program.exe 192.168.5.126`
* Replay:   `program.exe <log> <virt-out> 192.168.5.126`

Verifiziert (10-11, UdpClient auf 9999, "AgIO-Gruss" an 8888):
```
$PAOGI,0.0,4855.20001,N,00916.20000,E,4,25,0.6,0.000,0.0,4.860,271.8,0.00,0,0.00*47
$PAOGI,0.1,...*43   $PAOGI,0.2,...*4E      (10 Hz, Checksummen stimmen, Peer gelernt)
```
Hinweis: bei umgeleitetem stdout erscheinen die AOG-UDP-Statuszeilen ggf. erst beim
Prozessende (Blockpuffer) - der Empfang selbst ist der Beweis.


## 11. S2b — AgOpenGPS als Zuschauer ueber UDP (Betreiber-Wunsch, vereinfacht)

**Betreiber-Entscheidung:** es genuegt, **nur mit aufgezeichneten Logs** zu arbeiten. Die
Echtdaten (Position, Lenkwinkel, Attitude, fixQ, sigma …) kommen aus dem alten Log, der
Simulator rechnet mit dem **neuen Code** und zeigt **live in AgOpenGPS, was AOG sehen wuerde**
— zusaetzlich entsteht eine **neue Logdatei derselben Fahrt**.

### Ports (Betreiber-Hinweis: nicht aenderbar! Verifiziert aus unseren Docs)
| Richtung | Port | Beleg |
|---|---|---|
**AgOpenGPS → Modul** (Steer-PGNs) | **8888** | `COMMUNICATION_AND_NETWORKING.md:26`, `QUICK_REFERENCE.md:5` |
**Modul → Netz** (PGN/NMEA) | **9999** | `ESP32_BRIDGE_INTERFACE.md:49/165/181` |
⇒ **Der Simulator verhaelt sich wie unser Modul: lauscht auf 8888, sendet auf 9999** ✓✓
PAOGI wird zusaetzlich auch an 8888 gesendet (schadet nicht, AOG hoert nur auf einem ✓).
Ziel-IP im Test: **192.168.5.x** (Betreiber: AOG nutzt dieses Subnetz, LAN-Port manuell so
konfiguriert) — **nicht** 127.0.0.1. Empfohlener Aufbau (Betreiber-Idee, damit AOG **nichts**
umkonfigurieren muss):

| Rolle | Adresse | Wer |
|---|---|---|
AOG sendet Steuer-PGNs an das "Modul" | **192.168.5.126 : 8888** (Modul-IP aus den Logs) | **Simulator lauscht dort** |
Simulator sendet PAOGI/PGN | **192.168.5.<PC> : 9999** + **192.168.5.255 : 9999** (Broadcast) + 8888 | AOG empfaengt |
Teensy | **abgeschaltet** waehrend der Sim fahrt | sonst zweites "Modul" im Netz |

**Der Trick:** dem LAN-Port des PCs **zusaetzlich (Alias) die Modul-IP 192.168.5.126** geben
(Windows: Adaptereigenschaften -> IPv4 -> Erweitert -> weitere IP-Adressen) ⇒ der AOG-Verkehr
an das Modul kommt direkt beim Simulator an ✓✓ und **keine AOG-Einstellung aendert sich**
(Ports bleiben 8888/9999 ✓). Noetig: die PC-Adresse per `ipconfig` ermitteln und als
`-UdpTo` mitgeben.
**Wichtig:** die virtuelle Fahrt und das virtuelle Log funktionieren **ohne UDP** — S1 wird
also nicht durch das Netz-Setup blockiert ✓; AOG-live wird danach zugeschaltet ✓

### Datenfluss (VS Code → UDP → AgOpenGPS)
```
VS Code Terminal:
  sim.ps1 -Log "<2026-10-11 ...txt>" -Variante neu [-Udp -Echtzeit]
    sim.exe  (Host-Build, enthaelt den ECHTEN HeadingBridge.cpp)
      1 hr_reader: Log -> Stimulus-Records (t_ms, lat, lon, att, wheel, fixQ, hdop, sd, speed)
      2 Takt: Echtzeit = bis t_ms warten (AOG braucht 10 Hz) | Offline = ohne Warten
      3 gnss_stub.set(record) · vwas_stub.set(wheel, wq) -> bridge.update(dt)
      4 Bridge -> Heading / Roll / Position / State            (ECHTE Logik)
      5 A: $PAOGI -> UDP 9999 (+8888)      B: HR-Zeile -> <log>.virt.neu.txt
        C: optional PGN 250/253 -> UDP 8888                     D: metrics.json
```
**Kein Sonderweg in VS Code**: VS Code ist Editor/Terminal; UDP ist ein normaler Socket,
keine Admin-Rechte, keine Firewall-Regel (localhost) ✓ Zwei Prozesse nebeneinander:
`sim.exe` und `AgOpenGPS.exe` ✓ Optional `tasks.json` (Bauen + Log starten) ✓

### Was AOG in S2b zeigt — und was nicht
| sichtbar / messbar | nicht |
|---|---|
Traktor-Icon: Position, **Heading**, **Roll**, Speed · Spur aus unseren Positionen ⇒ **Icon vs. Spur = das „steht schraeg zur Spur"** ✓✓ | AOG faehrt **nicht selbst** (kein virtueller Traktor) — die geschlossene Lenkschleife bleibt optional (S4+) |
**AOGs eigenes fix-to-fix-Heading** aus unseren Positionen ⇒ Vergleich mit unserem Heading ✓ | keine Pruefung der PWM/Motorstrecke |
Roll-Korrektur (`sin(roll)*h`) ⇒ Schraeglauf durch falsche Roll wird **sichtbar** ✓ | |

### Deliverables dieser Stufe
```
1  <log>.virt.neu.txt        virtuelle alte Fahrt im IDENTISCHEN 101-Spalten-Format
2  diff_neu.txt              Spalte-fuer-Spalte-Vergleich alt vs. neu (psi, az_out, src_out,
                             publim, Position ...)  = "was AOG jetzt sehen wuerde"
3  metrics.json              Rauschen, Spruenge, Kurvenverluste, Quellen-Mix, Cross-Track-Naeherung
4  AOG live                  Icon/Spur/Roll in Echtzeit
```
Weil die **Eingaenge identisch** sind, ist der Diff aussagekraeftig wie ein Laborversuch ✓✓

### Zusaetzliche Dateien
`tools/sim/udp_pump.*` (Host-Sockets: send 9999, recv 8888) · `tools/sim/diffcols.ps1`
(Spalten-Diff alt/neu) · `.vscode/tasks.json` (Bauen + Fahrt starten)

### Kosten
**~0,5 Sitzung on top auf S1/S2** — weil der PAOGI-Satz und der PGN-253-Parser **im echten Code
schon existieren** (`NAVProcessor::formatPAOGIMessage`, `AutosteerProcessor::handleSteerConfig`)
⇒ nur Transport + Takt + optionaler Traktor ✗ (der entfaellt in S2b ganz ✓)

## 15 AOG-Anbindung: gemessene Fakten (2026-10-11)

Messung mit laufendem AgIO v6.8.6 + AgOpenGPS v6.8.6 (`Get-NetUDPEndpoint`):

```
0.0.0.0:8888    <- unser Sim (nur mit SIM_LISTEN8888=1)
0.0.0.0:9999    <- AgIO   = NMEA-Eingang -> unser Sendeziel (127.0.0.1:9999)
127.0.0.1:17777 <- AgIO   = interner Loopback zu AOG
AgOpenGPS       = gar kein UDP-Listener -> GPS kommt AUSSCHLIESSLICH ueber AgIO
```

Daraus drei Regeln fuer den Sim:

1. **8888 NICHT belegen.** In AOG v6 laeuft der Link AgIO <-> AOG selbst ueber UDP 8888;
   ein Bind des Sim nimmt AOG den GPS-Eingang. Standard ist deshalb aus; nur fuer den
   spaeteren PGN-Rueckkanal (S3) per `SIM_LISTEN8888=1` einschalten.
2. **Senden an 127.0.0.1:9999** (AgIO lauscht auf `0.0.0.0`) - nicht an 192.168.5.13: dieses
   Interface hat neben der festen IP eine 169.254-Adresse (APIPA, kein Link) und laeuft ins Leere.
   Zusaetzlich wird immer der gelernte Peer beschickt (sendet AgIO jemals an 8888, gewinnt der).
3. **PAOGI Feld 11 = `speedKnots` MUSS gesetzt werden** (`d2.speedKnots = vMps * 1.943844f`).
   Fehlt das, zeigt AgOpenGPS Position/Heading/Roll korrekt, aber **Speed = 0** - genau der
   Fehler vom 10-11 im Replay-Pfad (die HR-Spalte 3 `v_m/s` landete nur in `vScen`, nicht im
   GNSSData).

Kontrollzeile des Sim (`TX:`): `$PAOGI,0.0,4855.49131,N,00916.19993,E,4,30,0.5,0.000,0.0,1.431,342.5,0.00,0,-0.06*6E`
- Feld 11 `1.431` kn = 0.736 m/s = 2.65 km/h, exakt HR-Spalte 3 dieses Satzes.



## 16 Echtzeit-Replay (10 Hz PAOGI wie ein echter Empfaenger)

Gemessen (10-11, Log `2026-10-11 7-01 bruecke 1-5kmh heading medium LW 5-6 off`):

```
TX-RATE: 100 PAOGI-Saetze in 9.9 s Wanduhr -> 10.13 Hz (Soll 10.00)
TX-RATE: 200 PAOGI-Saetze in 19.9 s Wanduhr -> 10.06 Hz
Statuszeilen: 25 Stueck ueber 24 s Logzeit  -> 1 Zeile je Sekunde = Logzeit 1:1
```

Umsetzung (`tools/sim/main.cpp`):

* **Zeitachse = die des Logs.** Bei jedem Satz wird `g_simNowMs = rr.tMs` gesetzt; Statuszeile
  und PAOGI-Zeitfeld stempeln damit die ECHTE Empfaengerzeit - inklusive der Fix-Luecken des
  Original-Logs.
* **Driftfreie Taktung.** Statt `Sleep(10)` je Tick (Windows-Timer-Granularitaet ~15 ms -> real
  nur ~64 Hz und PAOGI ~6 Hz) wird je Tick die Sollzeit `(g_simNowMs - t_ms[0])` gegen die
  Wanduhr gestellt und nur die Restzeit geschlafen. `timeBeginPeriod(1)` (winmm, `-lwinmm`)
  macht Sleep auf 1 ms genau.
* **PAOGI weiter auf `tick % 10`** = 100-Hz-Tick / 10 = 10 Hz, exakt wie die Firmware.
* **Selbstkontrolle**: `ECHTZEIT: an (AOG-Feed), 100-Hz-Takt an der Log-Zeitachse`,
  `TX-RATE: ... Hz`, und am Ende `ECHTZEIT-PRUEFUNG: Log X s in Y s Wanduhr -> Faktor 1.000`.
* **Batch bleibt schnell**: ohne AOG und ohne `rt`/`realtime`-Argument bzw. `SIM_RT=1` laeuft
  der Replay weiterhin so schnell wie moeglich (fuer Diff/Auswertung).

## 17 Die Sim stoppen (Stop-Wege, gemessen 10-11)

Ein Echtzeit-Replay laeuft so lange wie das Log (hier 155 s). Zum Anhalten gibt es fuenf Wege;
alle beenden **sauber** (Dateien geschlossen, Grund im Klartext, `FERTIG`):

| Weg | Ausloeser | Bemerkung |
|---|---|---|
| Taste | `ESC` oder `q` | nur wenn ein Konsolenfenster angehaengt ist |
| Strg+C | `SetConsoleCtrlHandler` | auch Fenster-X; sauberes Ende statt hartem Abbruch |
| Stop-Datei | `sim_stop.flag` im Arbeitsverzeichnis **anlegen** | funktioniert auch bei versteckt gestartetem Prozess (Script/CI) |
| AgIO-Watchdog | Abnehmer-Prozess verschwindet | Karenz `SIM_AGIO_GRACE` (Standard 5 s); scharf, sobald er einmal lief; `noagio` schaltet ihn ab |
| Dialogbox | Argument `dialog`/`box` bzw. `SIM_DIALOG=1` | "OK = weiterlaufen, **Abbrechen = stoppen**"; eigener Win32-Thread, die 100-Hz-Schleife laeuft weiter |

Startzeile nennt die aktiven Wege:

```
STOP: ESC/q | Strg+C | Datei 'sim_stop.flag' anlegen | AgIO-Watchdog   [AgIO.exe laeuft]
```

Umgebungsvariablen: `SIM_DIALOG=1` (Dialogbox), `SIM_AGIO_GRACE=<s>`, `SIM_AGIO_NAME=<exe>`
(Selbsttest mit einem Ersatz-Prozess, z. B. `PING.EXE`), `SIM_RT=1` (Echtzeit erzwingen).

Abschluss bei Stop:

```
STOPP: Stop-Datei (nach 6.0 s Logzeit)
ECHTZEIT-PRUEFUNG: Log 154.9 s in 6.0 s Wanduhr -> Faktor 0.039 (1.000 = echtzeit)
VIRT-LOG geschrieben: _virt.txt
FERTIG
```

Nachweise (10-11):
* Stop-Datei: `STOPP: Stop-Datei (nach 6.0 s Logzeit)` - Prozess binnen <1 s beendet.
* Watchdog (Testobjekt `PING.EXE`, Karenz 3 s): nach dessen Ende
  `SIM: 'PING.EXE' nicht mehr aktiv - Stop in 3.0 s ...` und `STOPP: Abnehmer (AgIO) beendet (nach 9.5 s)`.
* Prozesserkennung geprueft mit `tools/sim/tmp/probe_proc.cpp` (Toolhelp32, ANSI-Vergleich).

