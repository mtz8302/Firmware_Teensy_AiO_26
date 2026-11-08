# 95 · Web-UI, ESP32-Bridge und die Seiten der Module

**Doku-Nachtrag 2026-11-07.** Diese Architektur ist **vor** der gemeinsamen Arbeit entstanden und
hatte nie eine schriftliche Beschreibung — hier wird sie nachgeholt. Grundlage sind der Betreiber-
Bericht und der Code; jede Aussage hat eine Fundstelle (`Datei:Zeile`). Was nur aus einem
Code-Kommentar stammt und nicht gegengeprüft ist, steht ausdrücklich dabei (Abschnitt 7).

## 1. Die Topologie

```
  PC mit AgOpenGPS            AiO-Platine (Teensy 4.1)                 Module (ESP32)
  192.168.5.x                 Web-UI  http://192.168.5.126             z. B. Section Control
      |                            |                                        |
      |  LAN                       |  UART (Serial1)                        |  WiFi
      +--------------------------->|  <------------>  ESP32 "AiO-Bridge"  <-+
           HTTP + DNS              |                   AP 192.168.137.1
                                   |                   Netz 192.168.137.0/24
```

Datenfluss in Worten des Betreibers:
**PC (AgOpenGPS) —LAN→ Teensy —UART→ ESP32 AiO —WiFi→ Modul (z. B. Section Control).**

Der Teensy steuert die AiO-Platine und hat das Web-UI für seine eigenen Einstellungen. Der ESP32
hängt per UART daran, spannt ein eigenes WLAN auf (`192.168.137.1`, gleiches /24 wie der
Windows-Hotspot, `AOG_AiO_ESP32XIAO_serial_to_WiFi_bridge.ino:50`) und **übersetzt HTTP in
UART-Frames und zurück**. Die Module buchen sich in dieses WLAN ein und bringen ihre eigene
Settings-Seite mit (z. B. `http://192.168.137.71`).

## 2. Die Anforderung

Die Settings-Seite der Section Control soll **über das LAN der AiO-Platine** erreichbar sein —
ohne dass der PC im WLAN des ESP32 hängt:

* mit der echten IP: `http://192.168.5.126/192.168.137.71/...`
* mit einem Namen: `http://sc.aog`

Damit muss die AiO zu **drei** Dingen in der Lage sein: (a) den Namen auflösen, (b) die Anfrage
über UART weiterreichen, (c) die Antwort so zurückgeben, dass Links und Formulare der Modul-Seite
weiter funktionieren.

## 3. Der Weg einer Anfrage

### 3.1 Alias-Weg: `http://sc.aog`

1. **DNS** — der PC fragt nach `sc.aog`. Der Teensy ist der DNS-Server (DHCPLite) und antwortet
   für **jeden** Namen auf `*.aog` und `*.local` mit **seiner eigenen IP**
   (`QNEthernetUDPHandler.cpp:420…437`). Der Browser verbindet sich also mit `192.168.5.126`.
2. **Virtual Host** — im HTTP-Request steht `Host: sc.aog`. `SimpleHTTPServer` erkennt die Endung,
   nimmt `sc` als Kurznamen und schreibt den Pfad auf `/sc/...` um
   (`SimpleHTTPServer.cpp:49…98`). Ausnahme: `aio` und die eigenen Namen
   (`gps`, `steer`, Benutzer-Aliase aus `/dns-alias`) bleiben auf dem Teensy.
3. **Modul-Proxy** — `/sc/…` passt auf keine Teensy-Route, also greift der Catch-All
   (`SimpleWebManager.cpp:648…656`) und ruft `handleModuleProxy()`. Das erste Pfadsegment sieht
   wie ein Modulname aus → Ziel ist immer der ESP32: `http://192.168.137.1/sc/…`
   (`SimpleWebManager.cpp:718…740`).
4. **UART** — `esp32Interface.proxyRequest()` schickt einen Rahmen
   `FE 01 [len_hi][len_lo][url]` (URL ≤ 400 Byte) an die Bridge (`…ino:363…384`).
5. **Bridge** — Host ist die Bridge selbst (`192.168.137.1`), also wird das Pfadsegment gegen die
   **Modulliste** aufgelöst: `DynModules[d].shortname` bzw. `longname` → Ziel
   `192.168.137.<ip_ending>` (`…ino:526…540`). Dann TCP-Verbindung auf Port 80 und
   `GET <path> HTTP/1.0` + `Host: <ziel-ip>`, Antwortkörper bis 48 000 Byte
   (`…ino:551…600`).
6. **Antwort** — zurück über UART als `FE 02 [len_hi][len_lo][body]` (`…ino:604…610`).
7. **Aufbereitung** — der Teensy schickt die Header sofort als `Transfer-Encoding: chunked`
   (damit der Browser nicht in den Timeout läuft, `SimpleWebManager.cpp:769…775`), schreibt bei
   HTML die Pfade um (`href`, `action`, `src`, `window.open` → jeweils `<modul>/…`,
   `SimpleWebManager.cpp:164…190`) und sendet in 1460-Byte-Stücken.
8. **Sticky-Session** — 30 s lang werden **präfixlose** Pfade dem zuletzt benutzten Modul
   zugeordnet (`SimpleWebManager.cpp:670`, `742…749`). Ohne das würde der nächste Klick aus der
   Modul-Seite auf der Teensy-Startseite landen.

### 3.2 IP-Weg: `http://192.168.5.126/192.168.137.71/settings`

Derselbe Weg, nur ohne DNS und ohne Sticky: das erste Segment ist eine IPv4-Adresse
(`SimpleWebManager.cpp:695…717`). Zeigt sie auf die Bridge selbst, liefert **sie** ihre eigene
Seite direkt aus dem Speicher (kein TCP-Selbstverbindungsversuch, `…ino:489…513`); sonst wird
per TCP an diese IP weitergereicht. Der Vorteil: dieser Weg funktioniert auch, wenn ein Modul
seinen Namen (noch) nicht gemeldet hat.

### 3.3 Direkt im WLAN

`http://192.168.137.71` (IP) oder `http://sc.local` (mDNS des Moduls,
`AOG_Section_Control_ESP32/zAOG_SC_Network.ino:131…136`). Für `sc.local` ist **kein** Server
nötig, das macht das Modul selbst — dieser Weg funktioniert aber nur für Geräte **im** WLAN des
ESP32, nicht für den PC am LAN.

## 4. Register B11 — die Bausteine, jeder mit Fundstelle

| # | Baustein | Was er tut und warum er nötig war |
|---|---|---|
| B11.1 | **DNS mit mehreren Namen (`DHCPLite::DNSreplyMulti`)** | Vorher antwortete DHCPLite auf **einen** Namen. Für die Alias-Seite (`/dns-alias`) und die Modulnamen muss die AiO mehrere Namen beantworten können. Neu in `lib/DHCPLite/DHCPLite.cpp` (+76 Zeilen), Deklaration in `DHCPLite.h:181`. **Commit `9d0c518`** — die Bibliothek ist sonst unveränderter Upstream-Code. |
| B11.2 | **Wildcard `*.aog` und `*.local`** | `QNEthernetUDPHandler.cpp:420…437`: endet die DNS-Frage auf `.local` oder `.aog`, antwortet der Teensy mit **seiner** IP — für **jeden** Namen. Das ist der Trick, der `sc.aog` ohne Pflege einer Alias-Liste möglich macht: der Name wird erst beim HTTP-Routing ausgewertet. Die feste Namensliste (`aio/wifi/gps/steer` in `.local` und `.aog`) bedient den Rest (`:442…444`). |
| B11.3 | **lwIP/mDNS auf dem Teensy wieder eingeschaltet** | `lib/QNEthernet/src/lwipopts.h` (Commit `9d0c518`): `LWIP_MDNS_RESPONDER 1`, `LWIP_IGMP 1`, `MEMP_NUM_UDP_PCB 10`, `MEMP_NUM_IGMP_GROUP 9`, `MEMP_NUM_SYS_TIMEOUT +6`, `LWIP_NUM_NETIF_CLIENT_DATA 1`. Vorher war mDNS bewusst abgeschaltet (Buffer gespart) — für den Bridge-Betrieb mit mehreren Sockets und Namen reicht das nicht. Der Commit nennt es „lwipopts für den Verkehr über die serielle Strecke zur ESP32 und zurück". |
| B11.4 | **HTTP-Virtual-Host** | `SimpleHTTPServer.cpp:49…98`: `Host: <name>.aog` → Kurzname → Pfad `/<name>/…`. Der Header wird beim Parsen gemerkt (`:159…167`, Feld `hostHeader` in `SimpleHTTPServer.h:71`). Namen, die dem Teensy gehören (`aio` fest, `gps`/`steer`/Benutzer-Aliase über `ownedHostnames`), werden **nicht** umgeschrieben. `wifi.*` ist absichtlich **nicht** „owned": es wird auf `/wifi/…` umgeschrieben, damit die Bridge-Seite über ihre eigene Route läuft (`:66…67`). |

| B11.5 | **Modul-Proxy im Teensy** | `SimpleWebManager::handleModuleProxy()` (`:669…807`): Ziel ist immer die Bridge (`192.168.137.1`), vier Fälle — IP-Pfad, Modulname im ersten Segment, Sticky-Session (30 s), kein Modul. Dazu die Sonderfälle `/wifi/…` (Bridge-Seite **ohne** Präfix, `:726…734`) und die Pfad-Umschreibung der Modul-Antworten (`rewriteModuleHtml`, `:164…190`). Header gehen **vor** dem UART-Aufruf raus (chunked), sonst Timeout im Browser (`:769…775`). Kein ESP32 erkannt → sofort **503** statt 15 s Warten (`:675…684`). |
| B11.6 | **UART-Protokoll Teensy ↔ Bridge** | Anfrage `FE 01 [len_hi][len_lo][url]`, URL ≤ 400 Byte, Rest wird nachgelesen (500-ms-Deadline) (`…ino:363…384`). Antwort `FE 02 [len_hi][len_lo][body]` (`…ino:604…610`). Der Puffer ist 512 Byte groß, weil darüber auch die PGN-Pakete laufen (`…ino:159…160`). |
| B11.7 | **Modulliste der Bridge (`DynModules`)** | Die Bridge kennt die Module aus dem **erweiterten PGN 203**: sie schickt den AgIO-IP-Scan (`0x80 0x81 0x7f 0xca 0x03 0xca 0xca 0x05 0xE5`, CRC 0xE5, `…ino:262`) ins WLAN; Module antworten klassisch (13 Byte, nur IP) oder **erweitert (149 Byte)** mit `shortname[12]`, `longname[20]`, `description[100]` und CRC (`zAOG_AiO_WebInterface.ino:179…215`). Daraus füllt sich `DynModules[]` (`:55`), und genau diese Namen löst der Proxy auf — `ip_ending` ist dabei **nur die letzte Oktette**, alle Module müssen also im /24 des AP liegen (`…ino:527…538`). |
| B11.8 | **`/api/modules` für die Startseite** | Der Teensy holt die Modulliste über denselben UART-Weg (`SimpleWebManager.cpp:293…296` → `handleApiModules`, `:982`); die Startseite baut daraus ihre Zeilen (JavaScript `loadModules()`, `TouchFriendlyHomePage.h`). Damit erscheinen neue Module ohne Firmware-Änderung im Menü. |
| B11.9 | **Eine Seite, zwei Server (Modul-Firmware)** | Die Section-Control-Firmware erzeugt ihre Seite **einmal**: `HTML_Buffer` = **ein** 80-kB-Heap-Puffer, auf den `HTML_Str_SC` und `HTML_Str_Settings` zeigen (`zAOG_SCfeature_EthWebServer.ino:19…36`). Gebaut wird mit `make_HTMLSC()`, `make_HTMLSettings()`, `make_HTMLExpert(grp)`, `make_OTA_HTML()`; geändert wird mit demselben `process_Setting()`. Beides benutzen **beide** Server: der WiFi-Server (`zAOG_SC_WebInterface.ino`) und der Ethernet-Server `EthernetServer Eth_Web_Server(80)` (`AOG_Section_Control_ESP32.ino:149`, Handler `zAOG_SCfeature_EthWebServer.ino:94…115`). Der Kommentar dort sagt es ausdrücklich: „Uses the shared `process_Setting()` and `make_HTML*()` functions" (`:8`). Die Seiten dienen nur dazu, dem Nutzer Einstellungen anzubieten — die Logik dahinter ist für beide Wege identisch. |
| B11.10 | **Geänderte Ethernet- und DNS-Bibliothek (ESP32-Seite)** | Die Section-Control hat eine **projekteigene Kopie** der Arduino-Ethernet-Bibliothek: `AOG_Section_Control_ESP32/libraries/Ethernet-Amas/` (`library.properties`: `name=Ethernet_Amas`, `version=2.0.2`), eingebunden als `#include "libraries/Ethernet-Amas/src/Ethernet_Amas.h"` (`AOG_Section_Control_ESP32.ino:135`). Angepasst für ESP32/ESP8266, damit **W5500-Ethernet und WiFi gleichzeitig** laufen: in `EthernetClient.cpp:50` steht `#if defined(ESP8266) || defined(ESP32)` (die ESP32-`IPAddress` braucht den `uint32_t`-Cast), und `utility/w5100.h:413…` hat den ESP8266-Schnellpfad für den SS-Pin (`GPO`/`GPOC`/`GPOS`). Die Kopie liegt **im Projekt**, damit die globale Installation unangetastet bleibt (der auskommentierte Include in `:136` nennt genau diesen Grund). |
| B11.11 | **mDNS auf dem Modul** | `MDNS.begin(Set.shortname)` + `MDNS.addService("http","tcp",80)` — auch im AP-Modus (`zAOG_SC_Network.ino:131…136`, `:200…205`). Der Kurzname ist derselbe, den das Modul im erweiterten PGN 203 meldet; damit stimmen `sc.local` (im WLAN) und `sc.aog` (über die AiO) überein. Die Bridge selbst meldet sich als `wifi.local` (`zAOG_AiO_Network.ino:239`). |


## 5. Grenzen und Fallstricke (aus dem Code, nicht geraten)

* **URL ≤ 400 Byte** — längere Anfragen verwirft die Bridge (`…ino:368`).
* **Antwortkörper ≤ 48 000 Byte** im Bridge-Puffer (`MAX_BODY_LOCAL`, `…ino:572`); größere Seiten
  werden abgeschnitten. Deshalb sind die Modul-Seiten kompakt.
* **Alle Module im gleichen /24** wie der AP — die Bridge speichert nur `ip_ending`
  (`…ino:538`). Ein Modul aus einem anderen Subnetz ist über den Namen **nicht** erreichbar,
  über den IP-Pfad schon (dann TCP direkt).
* **HTTP/1.0** im Bridge-Proxy (`…ino:561`) — bewusst, damit die Antwort **nicht** chunked kommt
  und der Header-Ende-Zustandsautomat sie zuverlässig findet.
* **Sticky 30 s** — öffnet man zwei Module in zwei Tabs, „gewinnt" das zuletzt benutzte. Der
  Alias-Weg (`sc.aog`) und der IP-Pfad sind davon **nicht** betroffen, weil sie das Modul im Pfad
  tragen.
* **Der PC braucht kein mDNS.** `sc.aog` funktioniert allein über die DNS-Antwort des Teensy
  (B11.2). `.local` über mDNS (B11.3/B11.11) ist die Zugabe für Geräte **im** ESP32-WLAN.

## 6. Datei-Karte

| Aufgabe | Datei |
|---|---|
| DNS-Antworten des Teensy | `lib/DHCPLite/DHCPLite.cpp/.h` (`DNSreplyMulti`), `lib/aio_system/QNEthernetUDPHandler.cpp` |
| lwIP-Konfiguration | `lib/QNEthernet/src/lwipopts.h` |
| Virtual Host / Host-Header | `lib/aio_system/SimpleHTTPServer.cpp/.h` |
| Modul-Proxy, `/api/modules`, Pfad-Umschreibung | `lib/aio_system/SimpleWebManager.cpp/.h` |
| UART-Protokoll (Teensy-Seite) | `lib/aio_communications/ESP32Interface.cpp/.h` |
| Startseite (Modulliste) | `lib/aio_system/web_pages/TouchFriendlyHomePage.h` |
| Bridge (ESP32) | `AOG_AiO_ESP32XIAO_serial_to_WiFi_bridge/`: `AOG_AiO_…bridge.ino` (Proxy, Modulliste), `zAOG_AiO_WebInterface.ino` (eigene Seiten, PGN 203), `zAOG_AiO_Network.ino` (AP, mDNS) |
| Modul-Firmware (Section Control) | `AOG_Section_Control_ESP32/`: `zAOG_SC_WebInterface.ino` (Bauer), `zAOG_SCfeature_EthWebServer.ino` (Ethernet-Server), `zAOG_SC_Network.ino` (WiFi/mDNS), `libraries/Ethernet-Amas/` |

## 7. Was hier NICHT verifiziert ist (bewusst offen)

1. **Welche Zeilen in `Ethernet_Amas` genau geändert wurden.** Die Kopie liegt im Projekt, ein
   Vergleich gegen das Upstream-Original (Arduino Ethernet 2.0.2) wurde **nicht** gemacht. Belegt
   sind nur die beiden ESP32/ESP8266-Stellen (B11.10) und der Umstand, dass die Bibliothek
   überhaupt kopiert und umbenannt wurde.
2. **Ob `Dns.h`/`Dhcp.h` in dieser Kopie geändert sind.** Aufgefallen ist nichts; ein Diff gegen
   Upstream fehlt.
3. **Wie die Modul-Seite im Detail aussieht** (`make_HTML*()`): hier steht nur, **dass** sie
   geteilt ist. Inhalt und Felder sind Sache der Modul-Projekte.
4. **Ob der Alias-Weg für jedes Modul getestet ist.** Belegt ist der Mechanismus; Sticky-Session
   und Pfad-Umschreibung sind die Stellen, an denen es bei neuen Modulen zuerst hakt.

Diese vier Punkte sind Kandidaten für einen kurzen Nachtrag, sobald der Betreiber sie bestätigt.

## 8. Kurzfassung für Entwickler (falls nur 30 Sekunden Zeit sind)

* Der Teensy ist **DNS-Server für `*.aog`** und antwortet mit seiner eigenen IP; danach entscheidet
  der **Host-Header** im HTTP-Request, welches Modul gemeint ist.
* Jede Modul-Anfrage geht als `FE 01 …` über UART an die Bridge; die Bridge löst den Namen gegen
  ihre **`DynModules`-Liste** (aus erweitertem PGN 203) auf, holt die Seite per TCP und schickt sie
  als `FE 02 …` zurück.
* Der Teensy schreibt die Pfade in der Antwort um (`/settings` → `/sc/settings`) und hält 30 s eine
  Sticky-Session, damit relative Links aus der Modul-Seite beim richtigen Modul bleiben.
* Die Modul-Firmware erzeugt **eine** Seite und verarbeitet Einstellungen **einmal**
  (`make_HTML*()`, `process_Setting()`) — angeboten über WiFi **und** W5500-Ethernet
  (`Ethernet_Amas`).

