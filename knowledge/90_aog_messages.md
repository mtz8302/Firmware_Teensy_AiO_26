# 90 AOG-Meldungen (PGN 221: Hardware-Messages)

**Version 1.0.0 · 2026-11-06** · gilt fuer den Stand `feat/hold-att-rate`
Thema: wie die Firmware dem Bediener **im Fuehrerhaus** etwas mitteilt (AgOpenGPS blendet
einen Text ein), welche Meldungen es gibt, wie sie ausgeloest und gespeichert werden - und
was dabei schiefgehen kann.

Angrenzende Dokumente: `10_heading.md` (die Bedingungen selbst, B10.80),
`00_ueberblick.md` (Themen-Karte), `HOW_TO_ADD_TRANSLATIONS.md` (Sprachmuster der Web-UI).

---

## 1. Der Kanal: PGN 221 ("DD")

Quelle: `SourceCode_AgOpenGPS/GPS/Forms/UDPComm.Designer.cs`, `case 221`, und
`OpenGL.Designer.cs` (`hardwareLineCounter`, `DrawHardwareMessageText`).

```
{ 0x80, 0x81, src, 221, len, sekunden, farbe, text..., CRC }
```

| Byte | Bedeutung |
|---|---|
| 0-3 | Kopf wie alle AOG-PGNs (`0x80, 0x81, src, 221`) |
| **4 (`len`)** | **2 + Textlaenge** (Sekunden + Farbe + Text) |
| **5** | Anzeigedauer in **Sekunden** (`hardwareLineCounter = data[5]*10`, je OpenGL-Frame bei 10 Hz eins herunter) |
| **6** | **Farbe: 0 = Salmon (Warnung)**, alles andere = Bisque (Info) |
| **7...** | Text, **UTF-8**, Laenge `len - 2` |
| letztes | CRC = Summe der Bytes ab Index 2 bis vor die CRC (Regel wie PGN 253) |

**Voraussetzung in AOG**: `Config -> Display -> "Hardware Messages"` muss **EIN** sein.
`GUI.Designer.cs` setzt `isHardwareMessages = false` als Default; in `UDPComm.Designer.cs`
wird das Label dann sogar explizit auf unsichtbar gesetzt - ohne das Haekchen passiert
also **nichts**, obwohl die Pakete korrekt ankommen. Das ist die haeufigste Fehlerursache.

**Label**: 888 x 43 px, Tahoma 20,25 **fett**, nicht AutoSize -> ca. **50 Zeichen** passen.

**Transport**: an AgIO (9999) **und** direkt an AOG (15555) - AgIO reicht Binaer-PGNs nicht
weiter. Im Sim macht das `AogUdp::sendToPeer` + `sendToAogDirect`.

---

## 2. Der Katalog (`lib/aio_system/aog_hwmsg.h`)

Vier Meldungen, je mit Anzeigedauer, Farbe, Prioritaet und Mindestabstand:

| Id | EN | DE (Stand der Maschine) | s | Farbe | Prio | Abstand |
|---|---|---|---|---|---|---|
| `WeakFix` | Weak GNSS fix | Empfang schwach | 5 | 0 Salmon | 2 | 15 s |
| `Bridge` | poor GNSS Heading quality | Empfang Fahrtrichtung ungenau | 5 | 1 Bisque | 1 | 20 s |
| `NoCourse` | No course - hold | Keine Fahrtrichtung bestimmbar: Kurs halten | 5 | 1 Bisque | 1 | 20 s |
| `PosJump` | Position jump | Positionssprung | 5 | 0 Salmon | 3 | 10 s |

**Bedingungen** (`AutosteerProcessor::updateHardwareMessages`):

| Meldung | Bedingung | Hardware-Voraussetzung |
|---|---|---|
| `WeakFix` | `!status.attitudeTrusted`, **2 s** anhaltend | **Koerper-Lage** (`hasDualHeading \|\| hasINS`) |
| `Bridge` | `status.state == BRIDGE \|\| COAST` | **Koerper-Lage** |
| `NoCourse` | `!pathChordValid && !travelWitnessValid && speed > 1 m/s`, **2 s** anhaltend | keine (Positionen) |
| `PosJump` | `status.pathChordSpike` (Ereignis) | keine (Positionen) |

**Warum die Hardware-Voraussetzung noetig ist** (B10.80): ohne Dual-Antenne/INS wird
`attitudeTrusted` nie wahr und der Zustandsautomat steht fuer immer in BRIDGE/COAST - die
zwei Meldungen wuerden **stehend** leuchten statt zu warnen.

---

## 3. Flankentriggerung: eine Meldung je AUFTRETEN

Betreiber-Regel (2026-11-06): *"jede Meldung nur 1x nach dem Auftreten, erst das naechste
Mal, wenn sich wieder was aendert"*.

* `AutosteerProcessor::hwEdge(id, cond, now)` fuehrt je Meldung ein Bit und den Zeitpunkt
  "zuletzt FALSCH". Gesendet wird nur bei der Flanke **0 -> 1**.
* **Flattern-Schutz**: war die Bedingung weniger als `hwEdgeHoldOffMs` = **2 s** weg, gilt
  sie nicht als neu aufgetreten (der Merker bleibt dabei FALSCH, die Meldung kommt also
  nach Ablauf der Ruhezeit von selbst).
* **Sicherheitsnetz je Meldung**: `AogHwMsg::send()` prueft `lastPerMsgMs_[id]` gegen
  `minGapMs`. Vorher verglich `sendText()` nur gegen die **zuletzt gesendete** Meldung -
  schob sich eine andere dazwischen (z. B. `WeakFix`), war der Abstand wirkungslos.
  Gemessen vor dem Fix: "HINWEIS Bridge" 2x in 12 s, danach 1x.
* `minGapMs` ist damit ein **Sicherheitsnetz gegen Flattern**, **nicht** der
  Wiederholungstakt. Wer die Wiederholung aendern will, aendert `hwEdge` - nicht den Abstand.
* **Ausnahme Uhr** (nur Sim): absichtlich periodisch, 10 s Takt / 2 s Anzeige.

Messung nach dem Umbau (Sim, 12 s Logzeit): `Bridge` 1x, `WeakFix` 1x, Uhr 2x.

---

## 4. Sprachen: EN/DE im Flash, LOCAL im EEPROM

Betreiber-Entscheidung: **eine** Spracheinstellung fuer Web-UI **und** Meldungen
(kein zweites EEPROM-Byte). Die Zahlenwerte sind deshalb gleich geordnet:
`WebLanguage {ENGLISH=0, GERMAN=1, LOCAL=2}` == `AogMsgLang {EN, DE, LOCAL}` - die
Zuordnung ist eine reine Umwandlung (`applyLanguageToModules()`).

| | Wo | Kosten |
|---|---|---|
| EN + DE | `const char*` im **Flash** | ~192 Byte `.rodata`, 0 EEPROM, 0 RAM |
| LOCAL | **EEPROM** (Block 2000), im Web-UI eingetippt | 200 Byte belegt, 500 reserviert |

**Rueckfallkette** (`AogHwMsg::textFor`): gewaehlte Sprache -> **EN**. Ein leeres
LOCAL-Feld ist damit **kein Fehler**, sondern "nimm Englisch". Die Web-Oberflaeche selbst
hat nur EN/DE; bei `LOCAL` greift dort ueberall der englische Text, weil alle Pruefungen
`== GERMAN ? ... : ...` lauten.

**Laenge**: `AOG_LOCAL_LEN = 48` (47 Zeichen), `AOG_MSG_MAX_BYTES = 48`. Umlaute sind
erlaubt (AOG dekodiert UTF-8) - sie zaehlen aber **zwei Byte**.
Der Deckel stand zuerst auf 24 Zeichen und hat den deutschen No-Course-Text (43 Zeichen)
beim Uebernehmen in die LOCAL-Felder abgeschnitten (Betreiber-Befund 2026-11-06).

---

## 5. Ablage im EEPROM

Karte siehe `lib/aio_config/EEPROMLayout.h` (dort als Kommentar, damit niemand raten muss):

```
1300 - 1499   frei (Reserve)
1500 - 1755   Fahrzeug-Setup (belegt bis 1608)
1756 - 1999   frei
2000 - 2499   AOG-Meldungstexte (AOG_MSG_SIZE 500 reserviert, belegt 200)
2500 - 2999   frei
3000 - 4283   Modul-Identifikation          <- Feld-Grundgeruest, unberuehrt
```

Prinzip (Betreiber-Vorgabe): **das Layout des Original-Forks bleibt als Grundgeruest** -
die Feld-Firmware belegt 1 ... 1299 und 3000 ... 4283; alles dazwischen ist in ihr
unbenutzt und traegt die Zusaetze, mit Reserve.

Aufbau `AogMsgBlob` (`lib/aio_system/aog_hwmsg_store.h`):

```
0  magic    uint16  0xA0C1
2  ver      uint8   3
3  crc      uint8   Summe ueber die Bytes ab Offset 4
4  enabled  uint8   Hauptschalter (0 = keine Meldungen)
5  len[4]   uint8   Laenge je Meldung (0 = leer -> EN)
9  text[4][48]      192 Byte, je nullterminiert
```

**Schreiblast**: Teensy-EEPROM ist **flash-emuliert** - jeder `write` kopiert einen ganzen
4-kB-Sektor um. `aogMsgStoreSave()` wird deshalb **nur** beim Speichern im Web-UI gerufen,
nie periodisch. `EEPROM_VERSION` bleibt unveraendert (ein Bump wuerde alle Config-Bloecke
loeschen); die Versionspruefung im Blob (`ver`) erledigt das Verwerfen alter Staende.

Versionen: **1** = 24-Zeichen-Texte, **2** = 48 Zeichen, **3** = + Hauptschalter.


---

## 6. Die Seite `/hwmsg`

Vorbild ist **`TouchFriendlyDeviceSettingsPage.h`** (Betreiber-Vorgabe): `.container`
(1024 px), `h1`, `.nav-buttons` (2 Spalten: "Back to Home" grau, "Apply Changes" blau),
`#status` als `.status`-Box, Inhalt in `.card`, Hilfetexte in `.help-text`.

**Kein `<h2>`**: `h2` ist in `touch.css` **weiss** und waere auf der weissen Karte
unsichtbar - Karten-Ueberschriften sind `.section-title` in dunkel.

**Statuszeile**: die Projekt-Konvention ist `.status.success` (gruener Hintergrund,
weisse Schrift) und `.status.error` (rot); neutral ist `.status` (hellgrau). Eigene
Schriftfarben sahen neben den anderen Seiten anders aus (Betreiber-Befund).

**Erster Punkt der Seite** ist der Hauptschalter (`AogMsgBlob.enabled`), im Stil der
Device-Settings-Seite (`.toggle-switch`/`.toggle-slider` aus `touch.css`, die drei
Container-Regeln woertlich von dort uebernommen). Er wirkt **sofort** (eigener POST mit nur
`{enabled}`) und schreibt dabei ins EEPROM.

**Anzeige der aktiven Texte**: die Felder enthalten die **LOCAL**-Texte; was mit der
gewaehlten Sprache wirklich rausgeht, steht als Zeile *"active now: ..."* unter jedem Feld
(API-Feld `active`). Ohne diese Zeile sah es nach dem Sprachwechsel so aus, als sei nichts
umgestellt worden. Zusatzknopf: *"Copy the active texts into the fields"*.

**Routen** (`SimpleWebManager`):

| Route | Zweck |
|---|---|
| `GET /hwmsg` | Seite (PROGMEM-Konstante, kein Template-Replace) |
| `GET /api/hwmsg/config` | lokale Texte, AKTIVE Texte, Sprache, `enabled` |
| `POST /api/hwmsg/config` | setzt Felder/`enabled`, dann **ein** `aogMsgStoreSave()` |
| `/lang/en\|de\|local` | `setLanguage()` -> EEPROM + `applyLanguageToModules()`; mit `?back=hwmsg` bleibt man auf der Seite (vorher feste Umleitung auf `/`) |

Nach dem Flashen **Strg+F5** auf `/hwmsg`: die Seite ist eine PROGMEM-Konstante ohne ETag,
der Browser haelt sonst das alte HTML.

---

## 7. Die Uhr gibt es nur im Sim

Im Replay zeigt `SIM_HWMSG=clock|sim|both` alle 10 s die Uhrzeit (2 s Anzeige) - sie war
der **Kanalbeweis**: man sieht sofort, ob PGN 221 durchkommt, ohne auf eine Stoerung zu
warten. In der **Firmware** gibt es sie nicht, und das ist Absicht: der Teensy 4.1 hat
**keine RTC mit Pufferbatterie**; ohne NTP oder ein Zeit-PGN von AgOpenGPS (beides nicht
vorhanden) bliebe nur die Laufzeit seit dem Start.

Schalter: `SIM_HWMSG_MS` (0 = aus), `SIM_HWMSG_SEC`, `SIM_HWMSG`, `SIM_LANG=de|en`.

---

## 8. Verifikation

Rohsatz aus einem echten Lauf (`HWM RAW` wird einmal je Lauf ausgegeben):

```
80 81 7E DD 0F 02 01 "09:28:01 140s" 3D
        |  |  |  |  |  +-- Farbe 1 = Bisque
        |  |  |  |  +----- 2 s Anzeige
        |  |  |  +-------- len 0x0F = 15 = 2 + 13 Zeichen
        |  |  +----------- PGN 221
        |  +-------------- Quelle
        +----------------- Kopf
```

Pruefpunkte: Text ab Index 7, Laenge `len-2`, CRC = Summe ab Index 2.

---

## 9. Offene Punkte

1. **Feldprobe fehlt**: alles oben ist im Sim/Replay gegen ein Log belegt. Zu pruefen:
   Nerven die Meldungen im Einsatz (Abstaende 15/20/20/10 s)? Ist die 5-s-Anzeige richtig?
2. **Abstaende nachmessen**: seit der Flankentriggerung ist `minGapMs` nur noch Netz. Eine
   echte neue Flanke (Zustand war > 2 s weg) wird erst nach 15/20 s gemeldet - wenn das zu
   traege wirkt, sind 5-10 s besser.
3. **`setLanguage()` schreibt bei jedem Klick** ins EEPROM (Sektor-Rewrite). Auf "nur bei
   Aenderung" optimieren.
4. **Sprachknopf fehlt auf der Startseite**: die Sprachwahl ist nur ueber `/hwmsg` und die
   Routen erreichbar; im HTML der Startseite gibt es dafuer keinen Knopf.
5. **Uhr in der Firmware**: nur sinnvoll mit einer Zeitquelle (NTP ueber QNEthernet oder
   ein Zeit-PGN). Nicht gebaut, bewusst.

