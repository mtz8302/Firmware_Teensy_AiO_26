// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef EEPROM_LAYOUT_H
#define EEPROM_LAYOUT_H

// Fuer uint8_t/uint16_t/uint32_t in den Adress-Konstanten und in
// vehicleSetupCrc32() weiter unten. Bewusst stdint.h und nicht Arduino.h: dieser
// Header soll auch in einem reinen Host-Test baubar bleiben.
#include <stdint.h>

// EEPROM Version - increment this when EEPROM layout changes
//
// ACHTUNG (2026-11-02, im Code belegt): Ein Bump loescht mehr, als er
// "invalidiert". ConfigManager::init() ruft bei Versions-Mismatch
// saveAllConfigs() auf (ConfigManager.cpp:43-48) - also einen SCHREIBVORGANG
// auf alle Bloecke, bevor irgendein Modul gelesen hat. Fuer den
// Fahrzeug-Setup-Block heisst das: die gespeicherte Geometrie ist weg, bevor
// loadVWASConfig() sie sehen kann.
//
// Deshalb NICHT bumpen, wenn es nur um den Fahrzeug-Setup-Block geht: der traegt
// seinen eigenen Guard (magic + layoutVer + payloadLen + crc32, siehe
// VEHICLE_SETUP_* unten und VehicleSetupBlob.h) und invalidiert sich selbst.
// Ein Bump ist nur noetig, wenn sich ein ANDERER ConfigManager-Block aendert.
#define EEPROM_VERSION 113  // Bumped: forces a clean rewrite of the ConfigManager
                            // blocks after the VWAS config corruption.
                            // 2026-11-02: bewusst NICHT erhoeht - siehe oben.

// EEPROM Address Map
#define EE_VERSION_ADDR      1      // Version number (2 bytes)
#define NETWORK_CONFIG_ADDR  100    // Network configuration (100-199)
#define STEER_CONFIG_ADDR    200    // Steer configuration (200-299)
#define STEER_SETTINGS_ADDR  300    // Steer settings (300-399)
#define GPS_CONFIG_ADDR      400    // GPS configuration (400-499)
#define MACHINE_CONFIG_ADDR  500    // Machine configuration (500-599)
// Address 600-699 available (KWAS removed)
#define INS_CONFIG_ADDR      700    // INS configuration (700-799)
#define EVENT_CONFIG_ADDR    800    // EventLogger configuration (800-899)
#define WEB_CONFIG_ADDR      900    // Web interface configuration (900-999)
#define TURN_SENSOR_CONFIG_ADDR 1000 // Turn sensor configuration (1000-1099)
#define ANALOG_WORK_SWITCH_ADDR 1100 // Analog work switch configuration (1100-1199)
#define MISC_CONFIG_ADDR        1200 // Miscellaneous settings (1200-1299)
#define MODULE_IDENT_CONFIG_ADDR 3000 // Module identification: shortname(12)+longname(20)+description(100)
                                     // 2026-11-01: von 1300 ans ENDE gelegt (Betreiber: "das
                                     // koennte schnell wachsen"). Belegt sind 132 Byte, der
                                     // Bereich 3000-4283 (~1,2 kB) gibt Luft fuer laengere
                                     // Beschreibungen. Die alte Stelle 1300-1499 ist frei.
// 2026-11-02: Die alten Adressen sind nur noch Nachvollziehbarkeit, kein Code
// benutzt sie. Der Fahrzeug-Setup-Block liegt bei 1500 (vorher VWAS-Config) und
// deckt 1600 (vorher Heading-Bridge) mit ab; beide alten Bloecke sind abgeloest,
// siehe die Aufstellung darunter.
// 2026-11-02 (Betreiber): VWAS (1500) und Heading-Bridge (1600) sind zu EINEM Block
// "Fahrzeug-Setup" zusammengelegt - ein Guard, ein Writer, EINE Reserve:
//
//   1500  magic      uint16  0xA511
//   1502  layoutVer  uint16  1
//   1504  payloadLen uint16  96  (sizeof(VehicleSetupBlob), per static_assert geprueft)
//   1506  reserved   uint16  0
//   1508  crc32      uint32  ueber [1512 .. 1608)
//   1512  Nutzlast   VehicleSetupBlob = Config(60) + centerCounts(4) + bridge(32)
//   1608  Ende der belegten Flaeche -> 148 Byte Reserve bis 1755
//
// ACHTUNG: der Block reicht ueber 1600 hinaus. Die alte Bridge-Adresse ist damit
// belegt und wird von loadHeadingBridgeConfig() NICHT mehr gelesen (sonst laese
// die Firmware ihre eigenen Bytes als Marker zurueck). Waechst VehicleSetupBlob,
// waechst die belegte Flaeche weiter nach hinten - der Platz dafuer ist da, die
// 256 Byte sind reserviert.
#define VEHICLE_SETUP_ADDR      1500
#define VEHICLE_SETUP_SIZE      256

// --- 1300-1399: LOKALE TEXTE der AOG-Hardware-Meldungen (PGN 221) --- 2026-11-06
// EN und DE sind im Flash kompiliert und kosten KEINEN EEPROM. Hier liegt nur, was der
// Bediener im Web-UI selbst eintippt (Sprache "LOCAL"); ein leerer Eintrag faellt auf EN
// zurueck. Aufbau (siehe aog_hwmsg_store.h): magic(2) ver(1) crc(1) len[4] text[4][24].
// Aufwand: 104 Byte von 4284 - und geschrieben wird NUR beim Speichern im Web-UI
// (Teensy-EEPROM ist flash-emuliert: ein write kopiert einen ganzen 4-kB-Sektor um).
// ACHTUNG: EEPROM_VERSION dafuer NICHT erhoehen - ein Bump loescht alle Config-Bloecke.
#define AOG_MSG_ADDR            1300
#define AOG_MSG_SIZE            104    // sizeof(AogMsgBlob), per static_assert geprueft
#define AOG_MSG_MAGIC           0xA0C1
#define AOG_MSG_VER             1

// --- Gemeinsamer Fahrzeug-Setup-Block: Kopf ------------------------------------
// Aufbau (Little Endian, Teensy 4.1):
//   0   magic      uint16   VEHICLE_SETUP_MAGIC
//   2   layoutVer  uint16   VEHICLE_SETUP_LAYOUT_VER
//   4   payloadLen uint16   Laenge der Nutzlast ab Offset 12 (Variante A)
//   6   reserved   uint16   0 - haelt die crc32 auf 4-Byte-Grenze
//   8   crc32      uint32   ueber die Nutzlast [12 .. 12+payloadLen)
//   12  payload    ...      WheelAngleFusion::Config, centerCounts, Bridge
//
// Der Kopf ist absichtlich 12 Byte breit und nicht 10: sonst laege die crc32
// schief und die Nutzlast (mit float-Feldern) faenge bei 10 an - ein float auf
// einer nicht durch 4 teilbaren Adresse laesst den Cortex-M7 zwar rechnen, aber
// jeden unaligned-Zugriff teuer bezahlen.
//
// payloadLen steht im Kopf, damit ein spaeter gewachsener Config-Struct den
// Block nicht sprengt: beim Lesen wird gegen sizeof() geprueft, bei Abweichung
// gilt der Block als ungueltig und wird neu geschrieben (Reset statt Rate-spiel).
#define VEHICLE_SETUP_MAGIC       0xA511u  // 'A' + Kennung 0x11 - bewusst nicht 0xFFFF/0x0000
#define VEHICLE_SETUP_LAYOUT_VER  1u
#define VEHICLE_SETUP_HDR_MAGIC      0
#define VEHICLE_SETUP_HDR_LAYOUTVER  2
#define VEHICLE_SETUP_HDR_PAYLOADLEN 4
#define VEHICLE_SETUP_HDR_CRC32      8
#define VEHICLE_SETUP_HDR_SIZE      12
#define VEHICLE_SETUP_MAX_PAYLOAD  (VEHICLE_SETUP_SIZE - VEHICLE_SETUP_HDR_SIZE)  // 244 Byte

// Aufteilung der Nutzlast innerhalb des Blocks. Die Offsets sind NICHT fest
// verdrahtet - die Nutzlast wird als Ganzes geschrieben und gelesen, die
// Reihenfolge ergibt sich aus der Struktur. Diese Konstanten halten nur die
// geplante Groessenordnung fest, damit beim Erweitern sichtbar bleibt, wieviel
// Reserve noch da ist (Stand 2026-11-02: Config ~64, centerCounts 4, Bridge 23
// -> ~153 Byte Reserve).
#define VEHICLE_SETUP_RESERVE_MIN   128

// --- CRC32 fuer den Fahrzeug-Setup-Block ---------------------------------------
// Bewusst im Header und inline: die Pruefsumme wird beim Boot genau einmal
// gebraucht, eine Tabelle waere 1 KB Flash fuer einen einzigen Aufruf. Bitweise
// sind das 8 Schritte pro Byte, bei ~101 Byte also ~800 Durchlaeufe - nicht
// messbar, und das Ergebnis ist dasselbe.
//
// Polynom 0xEDB88320 (reflektiertes 0x04C11DB7, IEEE 802.3) - dasselbe, das
// zlib, zip und AOGs eigene Dateien benutzen. Damit laesst sich ein Dump des
// Blocks am PC mit jedem x-beliebigen crc32-Werkzeug nachrechnen, ohne dass man
// erst unsere Variante nachbauen muss.
//
// Startwert 0xFFFFFFFF und abschliessende Invertierung sind Teil der Norm: ohne
// sie unterscheidet sich das Ergebnis von jedem Standard-Werkzeug, und dann ist
// die Pruefsumme im Fehlerfall wertlos, weil niemand sie gegenrechnen kann.
inline uint32_t vehicleSetupCrc32(const void* data, unsigned len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (unsigned i = 0; i < len; ++i) {
        crc ^= p[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            // Wenn das niederwertigste Bit gesetzt ist: nach dem Schieben das
            // Polynom einrechnen, sonst nur schieben.
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

#define DNS_ALIAS_CONFIG_ADDR   600  // DNS alias configuration (600-699), was previously KWAS

#endif // EEPROM_LAYOUT_H
