// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef VEHICLE_SETUP_BLOB_H
#define VEHICLE_SETUP_BLOB_H

// Nutzlast des gemeinsamen Fahrzeug-Setup-Blocks (EEPROM 1500-1755).
//
// EINE Struktur, EIN Guard (magic + layoutVer + payloadLen + crc32, siehe
// EEPROMLayout.h), EINE Reserve. Vorher lagen VWAS-Config (1500, eigener
// Magic 0xF1) und Heading-Bridge (1600, Marker-PAAR 0xA5/0x5A) in zwei
// Bloecken mit zwei verschiedenen Guard-Mechanismen - zwei Gelegenheiten,
// nach einem fehlgeschlagenen Schreibvorgang halbierte Daten zu laden.
//
// Aufbau der Nutzlast:
//   vwas         WheelAngleFusion::Config   Fahrzeug-Geometrie + Regler-Parameter
//   centerCounts int32_t                    Encoder-Nullpunkt (Rohcounts)
//   bridge       uint8_t[]                  Bridge-Payload als ROHREGION
//
// Warum die Bridge als Rohregion und nicht als Feldliste:
// ihre Felder liegen heute einzeln in ConfigManager (bridgeSteerScale,
// bridgePhysicalBaselineM, bridgeDualHeadingOffsetDeg, bridgeMountOffsetDeg ...
// je mit eigenem ...Valid-Flag). Diese Liste hier zu wiederholen, wuerde eine
// zweite Wahrheit schaffen: sobald jemand in ConfigManager ein Feld ergaenzt,
// passt der Blob stillschweigend nicht mehr, ohne dass es auffaellt. Stattdessen
// serialisiert ConfigManager seine Bridge-Felder selbst in diese Region und
// meldet ihre tatsaechliche Laenge - der Blob muss die Feldliste nicht kennen.
//
// Die Region ist mit 32 Byte bewusst groesser als die heute belegten ~23: ein
// spaeter ergaenztes Bridge-Feld soll den Block nicht sofort sprengen. Reicht
// sie nicht mehr, schlaegt der static_assert hier beim Bauen an, nicht erst im
// Feld.
#define VEHICLE_SETUP_BRIDGE_BYTES 32

#include <stdint.h>
#include "EEPROMLayout.h"
#include "WheelAngleFusion.h"

struct VehicleSetupBlob {
    WheelAngleFusion::Config vwas;
    int32_t centerCounts;
    uint8_t bridge[VEHICLE_SETUP_BRIDGE_BYTES];
};

// --- Drahtformat der Bridge-Nutzlast -------------------------------------------
// Die Feldliste der Bridge steht in ConfigManager (vier Werte, je mit eigenem
// Gueltigkeits-Flag). Damit sie nicht zweimal gepflegt werden muss, schreibt
// ConfigManager sie ueber serializeBridgePayload() in genau dieses Format und
// liest sie mit deserializeBridgePayload() zurueck.
//
// Die vier Gueltigkeits-Flags liegen als BITMASKE in einem Byte statt als vier
// bool: vier bool belegen durch das Alignment der vier floats hindurch 16 Byte
// statt 4. Die Maske ist zudem selbstbeschreibend - ein unbekanntes Bit aus
// einer kuenftigen Version laesst sich ignorieren, ein zusaetzliches bool-Feld
// wuerde bei falscher Groesse die ganze Region unlesbar machen.
struct BridgePayload {
    float   steerScale;             // 1.0 = dem VWAS-Massstab vertrauen
    float   baselineM;              // Antennenabstand (m), 0 = lernen
    float   dualHeadingOffsetDeg;   // Dual-Heading-Offset (deg)
    float   antennaPivotM;          // Antenne vor dem Drehpunkt (m)
    uint8_t validBits;              // siehe BRIDGE_VALID_*
    uint8_t reserved[3];            // 0 - haelt die Struktur 4-Byte-aligned
};

// Bit-Belegung von BridgePayload::validBits. Die Reihenfolge ist Teil des
// Drahtformats: einmal vergeben, nie umsortieren (sonst zeigt ein gespeicherter
// Block auf das falsche Feld).
#define BRIDGE_VALID_STEER_SCALE      0x01
#define BRIDGE_VALID_BASELINE         0x02
#define BRIDGE_VALID_DUAL_HEADING     0x04
#define BRIDGE_VALID_ANTENNA_PIVOT    0x08

static_assert(sizeof(BridgePayload) <= VEHICLE_SETUP_BRIDGE_BYTES,
              "BridgePayload passt nicht in die Bridge-Region des Fahrzeug-Setup-Blocks - "
              "VEHICLE_SETUP_BRIDGE_BYTES in VehicleSetupBlob.h erhoehen.");


// Der Block hat 256 Byte, der Kopf 12 - also 244 Byte Nutzlast. Wenn diese
// Zusicherung anschlaegt, ist der Config-Struct gewachsen und der Block muss
// vergroessert werden (VEHICLE_SETUP_SIZE). Das faellt dann BEIM BAUEN auf und
// nicht als abgeschnittene Geometrie auf der Maschine.
static_assert(sizeof(VehicleSetupBlob) <= VEHICLE_SETUP_MAX_PAYLOAD,
              "VehicleSetupBlob passt nicht in den Fahrzeug-Setup-Block - "
              "VEHICLE_SETUP_SIZE in EEPROMLayout.h erhoehen und "
              "VEHICLE_SETUP_LAYOUT_VER mitziehen.");

// Mindest-Reserve, damit spaetere Zusatzfelder Platz haben, ohne dass sofort
// ein neues Layout faellig wird. Auch diese Zahl ist eine Bau-Zusicherung.
static_assert(VEHICLE_SETUP_MAX_PAYLOAD - sizeof(VehicleSetupBlob) >= 64,
              "Der Fahrzeug-Setup-Block hat zu wenig Reserve - Layout pruefen.");

// --- Schreib-Hook fuer den Block ----------------------------------------------
// Der Block enthaelt ZWEI Nutzlasten aus ZWEI Klassen: die VWAS-Config gehoert
// AutosteerProcessor, die Bridge-Felder gehoeren ConfigManager. Damit trotzdem
// nur EIN Writer existiert, meldet ConfigManager ueber diesen Zeiger, dass sich
// ein Bridge-Feld geaendert hat (saveHeadingBridgeConfig), und AutosteerProcessor
// schreibt den ganzen Block.
//
// Kein Include von AutosteerProcessor.h in ConfigManager.h: das waere eine
// Abhaengigkeit von aio_config nach aio_autosteer, also gegen die Schichtung -
// und der Bridge-Lerner ruft den Setter aus dem Navigations-Lib.
//
// Ist der Zeiger nicht gesetzt (waehrend ConfigManager::init(), oder in einem
// Build ohne VWAS), wird nichts geschrieben und nichts geht kaputt: die Werte
// stehen dann nur im RAM.
extern void (*g_vehicleSetupSaveHook)();

#endif // VEHICLE_SETUP_BLOB_H

