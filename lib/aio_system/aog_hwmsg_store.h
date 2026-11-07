// ---------------------------------------------------------------------------
// aog_hwmsg_store.h - LOCAL-Texte der AOG-Hardware-Meldungen im EEPROM (2026-11-06h)
// ---------------------------------------------------------------------------
// EN und DE stehen als const-Strings im Flash (aog_hwmsg.h) und kosten keinen EEPROM.
// Hier liegt NUR die dritte Sprache: was der Bediener im Web-UI eintippt. Ein leerer
// Eintrag faellt auf EN zurueck (Rueckfallkette in AogHwMsg::textFor).
//
// Block: EEPROMLayout.h -> AOG_MSG_ADDR = 2000 (dort mit der vollstaendigen Karte),
// AOG_MSG_SIZE = 500 reserviert. Belegt sind 200 Byte (Kopf 8 + 4 x 48); die restlichen
// 300 sind Luft, damit spaetere Meldungen nicht gegen den naechsten Block stossen.
// Aufbau:
//   0   magic uint16  0xA0C1
//   2   ver   uint8   2
//   3   crc   uint8   Summe ueber die Bytes ab Offset 4 bis zum Ende
//   4   len[] uint8   Laenge je Meldung (0 = leer -> EN)
//   8   text[4][48]   192 Byte, je nullterminiert
//
// WICHTIG: Teensy-EEPROM ist flash-emuliert - jeder write kopiert einen ganzen 4-kB-Sektor
// um. aogMsgStoreSave() deshalb NUR beim Speichern im Web-UI aufrufen, nie periodisch.
// Und EEPROM_VERSION dafuer NICHT erhoehen (ein Bump loescht alle Config-Bloecke).
#ifndef AOG_HWMSG_STORE_H
#define AOG_HWMSG_STORE_H

#include <EEPROM.h>
#include <cstring>
#include "EEPROMLayout.h"
#include "aog_hwmsg.h"

struct AogMsgBlob {
    uint16_t magic;
    uint8_t  ver;
    uint8_t  crc;
    uint8_t  enabled;                    // 2026-11-06: Hauptschalter (0 = keine Meldungen)
    uint8_t  len[AOG_MSG_COUNT];
    char     text[AOG_MSG_COUNT][AOG_LOCAL_LEN];
};
static_assert(sizeof(AogMsgBlob) <= AOG_MSG_SIZE, "AogMsgBlob passt nicht in AOG_MSG_SIZE");

inline uint8_t aogMsgBlobCrc(const AogMsgBlob& b) {
    const uint8_t* p = (const uint8_t*)&b;
    uint8_t ck = 0;
    for (size_t i = 4; i < sizeof(AogMsgBlob); ++i) { ck = (uint8_t)(ck + p[i]); }
    return ck;
}

// Laedt die lokalen Texte in den Sender. Unbekannter/alter Block: es bleibt bei EN.
inline bool aogMsgStoreLoad(AogHwMsg& m) {
    AogMsgBlob b;
    EEPROM.get(AOG_MSG_ADDR, b);
    if ((b.magic != AOG_MSG_MAGIC) || (b.ver != AOG_MSG_VER)) { return false; }
    if (aogMsgBlobCrc(b) != b.crc) { return false; }
    m.setEnabled(b.enabled != 0u);          // Hauptschalter mitladen
    for (uint8_t i = 0; i < AOG_MSG_COUNT; ++i) {
        if ((b.len[i] == 0u) || (b.len[i] >= AOG_LOCAL_LEN)) {
            m.setLocalText((AogMsgId)i, nullptr);      // leer -> EN
            continue;
        }
        char t[AOG_LOCAL_LEN];
        std::memcpy(t, b.text[i], AOG_LOCAL_LEN);
        t[AOG_LOCAL_LEN - 1] = '\0';
        m.setLocalText((AogMsgId)i, t);
    }
    return true;
}

// Schreibt die lokalen Texte. NUR auf Benutzeranforderung (Web-UI "Speichern")!
inline void aogMsgStoreSave(const AogHwMsg& m) {
    AogMsgBlob b{};
    b.magic = AOG_MSG_MAGIC;
    b.ver   = AOG_MSG_VER;
    b.enabled = m.isEnabled() ? 1u : 0u;    // Hauptschalter mitspeichern
    for (uint8_t i = 0; i < AOG_MSG_COUNT; ++i) {
        const char* t = m.localText((AogMsgId)i);
        const size_t n = (t != nullptr) ? std::strlen(t) : 0u;
        if ((n == 0u) || (n >= AOG_LOCAL_LEN)) { b.len[i] = 0u; continue; }
        b.len[i] = (uint8_t)n;
        std::memcpy(b.text[i], t, n);
    }
    b.crc = aogMsgBlobCrc(b);
    EEPROM.put(AOG_MSG_ADDR, b);
}

#endif  // AOG_HWMSG_STORE_H
