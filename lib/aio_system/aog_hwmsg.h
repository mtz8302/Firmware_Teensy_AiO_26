// ---------------------------------------------------------------------------
// aog_hwmsg.h - PGN 221 ("DD"), die Hardware-Message von AgOpenGPS - 2026-11-06g
// ---------------------------------------------------------------------------
// AOG blendet den Text als Label auf der Karte ein und nach `sekunden` wieder aus.
// Format (nachgelesen in SourceCode_AgOpenGPS/GPS/Forms/UDPComm.Designer.cs, case 221):
//   { 0x80, 0x81, src, 221, len, sekunden, farbe, text..., CRC }
//   text   = UTF-8 ab Byte 7, Laenge = len - 2
//   len    = 2 (Sekunden + Farbe) + Textlaenge
//   farbe  = 0 -> Salmon (Warnung), sonst Bisque (Info)
//   CRC    = Summe der Bytes ab Index 2 bis vor die CRC (wie PGN 253)
// IN AOG MUSS "Hardware Messages" EINGESCHALTET SEIN (Config -> Display), sonst
// setzt AOG das Label auf unsichtbar (GUI.Designer.cs: isHardwareMessages = false).
//
// Die Meldungen sind MEHRSPRACHIG angelegt: text[0] = Englisch, text[1] = Deutsch.
// Der Index passt zu WebLanguage der Firmware (ENGLISH = 0, GERMAN = 1), damit
// spaeter dieselbe Einstellung (Web-UI/EEPROM) die Sprache bestimmt.
//
// Reine C++-Header, keine Arduino-Abhaengigkeit: liegt im Sim unter tools/sim/ und
// kommt 1:1 in die Firmware (lib/aio_system/).
#ifndef AOG_HWMSG_H
#define AOG_HWMSG_H

#include <cstdint>
#include <cstdio>
#include <cstring>

// Muss zu WebLanguage passen (SimpleWebManager.h): ENGLISH = 0, GERMAN = 1.
// LOCAL = 2 liegt ABSICHTLICH dahinter: EN und DE sind im Flash fest kompiliert (kosten
// keinen EEPROM), LOCAL tippt der Bediener im Web-UI ein und liegt im EEPROM
// (lib/aio_system/aog_hwmsg_store.h, Block 1300). Ist ein LOCAL-Text leer, gilt EN.
enum class AogMsgLang : uint8_t { EN = 0, DE = 1, LOCAL = 2 };

// Meldungskatalog. Die Reihenfolge hier ist nur die Kennung - Prioritaet und
// Anzeigedauer stehen in der Tabelle.
enum class AogMsgId : uint8_t {
    WeakFix = 0,   // Lage nicht vertraut / Epoch degradiert
    Bridge,        // Heading-Bridge aktiv (Lage weg, Kurs traegt)
    NoCourse,      // weder Sehne noch Witness -> Ausgabe haelt/projiziert
    PosJump,       // Positionssprung erkannt
    Count
};

static constexpr uint8_t AOG_MSG_COUNT     = (uint8_t)AogMsgId::Count;
// 2026-11-06: 24 -> 48. Der Deckel war zu knapp: das AOG-Label ist 888 px breit bei
// Tahoma 20,25 fett und zeigt ~50 Zeichen, der deutsche No-Course-Text hat 43. Mit 24
// wurde er beim Uebernehmen in die LOCAL-Felder abgeschnitten.
static constexpr uint8_t AOG_LOCAL_LEN     = 48;   // inkl. Nullterminator (47 Zeichen)
static constexpr uint8_t AOG_MSG_MAX_BYTES = 48;   // harte Grenze je Meldung (UTF-8-Bytes)

struct AogMsgDef {
    const char* text[2];   // [0] = EN, [1] = DE (im Flash, kosten keinen EEPROM)
    uint8_t     seconds;   // Anzeigedauer in AOG
    uint8_t     colour;    // 0 = Salmon (Warnung), sonst Bisque (Info)
    uint8_t     prio;      // hoeher = darf eine LAUFENDE Meldung verdraengen
    uint32_t    minGapMs;  // SICHERHEITSNETZ gegen Flattern - NICHT der Wiederholungstakt!
                           // Die Meldungen sind FLANKENGETRIGGERT: gesendet
                           // wird einmal je Auftreten, erneut erst, wenn die Bedingung weg
                           // war (siehe AutosteerProcessor::hwEdge). Der Abstand greift nur,
                           // wenn eine Bedingung hektisch hin und her springt.
};

inline const AogMsgDef& aogMsgDef(AogMsgId id) {
    static const AogMsgDef k[(int)AogMsgId::Count] = {
        //  EN-Text                 DE-Text                  s  Farbe Prio  Abstand
        { { "Weak GNSS fix",     "GNSS Empfang schwach"      }, 4, 0, 2, 15000u },  // Warnung
        { { "poor GNSS Heading quality",   "GNSS Empfang Fahrtrichtung ungenau"        }, 3, 1, 1, 20000u },  // Hinweis
        { { "No course - hold",  "Keine Fahrtrichtung bestimmbar: Kurs halten"   }, 3, 1, 1, 20000u },  // Hinweis
        { { "Position jump",     "Positionssprung"      }, 4, 0, 3, 10000u },  // Warnung
    };
    return k[(int)id];
}

// Sender: baut das Paket, haelt Abstaende ein und verdraengt nur mit hoeherer Prioritaet.
class AogHwMsg {
public:
    // Transport: schreibt ein fertiges Paket. Sim: beide Ziele (AgIO + AOG direkt).
    using TxFn = void (*)(const uint8_t* data, int len);

    void begin(TxFn fn, AogMsgLang lang = AogMsgLang::DE) { tx_ = fn; lang_ = lang; }
    void setLang(AogMsgLang l) { lang_ = l; }
    AogMsgLang lang() const { return lang_; }
    // Hauptschalter (2026-11-06). Aus = es geht KEINE Meldung raus (auch die Uhr nicht).
    // Der Zustand liegt im EEPROM-Block (aog_hwmsg_store.h) und wird beim Start geladen;
    // ohne gespeicherten Block ist er AN, damit sich am bisherigen Verhalten nichts aendert.
    void setEnabled(bool on) { enabled_ = on; }
    bool isEnabled() const { return enabled_; }
    uint32_t lastSendMs() const { return lastMs_; }
    const char* lastName() const { return nameBuf_; }

    // --- LOCAL-Texte (vom Bediener im Web-UI eingetippt, liegen im EEPROM) -------------
    // Setzt/löscht den Text einer Meldung. nullptr oder "" löscht ihn; dann gilt EN.
    // Der Speicher (aog_hwmsg_store.h) ruft das beim Laden, das Web-UI beim Speichern.
    void setLocalText(AogMsgId id, const char* t) {
        const int i = (int)id;
        if ((i < 0) || (i >= (int)AOG_MSG_COUNT)) { return; }
        if ((t == nullptr) || (t[0] == '\0')) { local_[i][0] = '\0'; return; }
        std::snprintf(local_[i], AOG_LOCAL_LEN, "%s", t);
    }
    const char* localText(AogMsgId id) const {
        const int i = (int)id;
        if ((i < 0) || (i >= (int)AOG_MSG_COUNT)) { return ""; }
        return local_[i];
    }
    // Der Text, der WIRKLICH rausgeht: gewaehlte Sprache -> EN als Rueckfall.
    const char* textFor(AogMsgId id) const {
        const AogMsgDef& d = aogMsgDef(id);
        if (lang_ == AogMsgLang::LOCAL) {
            const char* t = local_[ (int)id ];
            if ((t != nullptr) && (t[0] != '\0')) { return t; }
        } else {
            const char* t = d.text[(int)lang_];
            if ((t != nullptr) && (t[0] != '\0')) { return t; }
        }
        return d.text[(int)AogMsgLang::EN];   // immer die vollstaendige Basis
    }

    // Freitext (z.B. Uhrzeit als Info). false = nicht gesendet (Abstand/laufende Meldung).
    bool sendText(const char* txt, uint8_t sec, uint8_t colour, uint8_t prio,
                  uint32_t minGapMs, uint32_t nowMs) {
        if (!enabled_) { return false; }          // Hauptschalter aus -> nichts senden
        if ((tx_ == nullptr) || (txt == nullptr) || (txt[0] == '\0')) { return false; }
        // Eine noch laufende Meldung nicht ueberschreiben - ausser die neue ist wichtiger.
        if ((nowMs < busyUntilMs_) && (prio <= lastPrio_)) { return false; }
        // Dieselbe Meldung nicht zu schnell wiederholen.
        if ((std::strcmp(txt, nameBuf_) == 0) && ((uint32_t)(nowMs - lastMs_) < minGapMs)) {
            return false;
        }
        const int n = (int)std::strlen(txt);
        if (n > 48) { return false; }
        uint8_t b[7 + 48 + 1];
        b[0] = 0x80; b[1] = 0x81; b[2] = 0x7E; b[3] = 221;
        b[4] = (uint8_t)(2 + n);          // Sekunden + Farbe + Text
        b[5] = sec;
        b[6] = colour;
        for (int i = 0; i < n; ++i) { b[7 + i] = (uint8_t)txt[i]; }
        uint8_t ck = 0;
        for (int k = 2; k < 7 + n; ++k) { ck += b[k]; }
        b[7 + n] = ck;
        tx_(b, 8 + n);
        lastMs_      = nowMs;
        busyUntilMs_ = nowMs + (uint32_t)sec * 1000u;
        lastPrio_    = prio;
        std::snprintf(nameBuf_, sizeof(nameBuf_), "%s", txt);
        return true;
    }

    // Katalogmeldung in der eingestellten Sprache.
    bool send(AogMsgId id, uint32_t nowMs) {
        const int i = (int)id;
        if ((i < 0) || (i >= (int)AOG_MSG_COUNT)) { return false; }
        const AogMsgDef& d = aogMsgDef(id);
        // Sicherheitsnetz JE MELDUNG (2026-11-06h): vorher wurde nur gegen die zuletzt
        // gesendete Meldung verglichen - schob sich eine andere dazwischen (z. B. WeakFix),
        // war der Abstand wirkungslos und dieselbe Meldung konnte kurz darauf erneut kommen.
        if ((lastPerMsgMs_[i] != 0u) && ((uint32_t)(nowMs - lastPerMsgMs_[i]) < d.minGapMs)) {
            return false;
        }
        const char* t = textFor(id);
        if ((t == nullptr) || (t[0] == '\0')) { return false; }
        if (!sendText(t, d.seconds, d.colour, d.prio, d.minGapMs, nowMs)) { return false; }
        lastPerMsgMs_[i] = (nowMs == 0u) ? 1u : nowMs;
        return true;
    }

private:
    TxFn        tx_ = nullptr;
    AogMsgLang  lang_ = AogMsgLang::DE;
    uint32_t    lastMs_ = 0;
    uint32_t    busyUntilMs_ = 0;
    uint8_t     lastPrio_ = 0;
    char        nameBuf_[49] = { 0 };
    bool        enabled_ = true;   // Hauptschalter, Default AN (wie bisher)
    // Letzter Sendezeitpunkt JE Katalogmeldung (Sicherheitsnetz, siehe send()).
    mutable uint32_t lastPerMsgMs_[AOG_MSG_COUNT] = {};
    // LOCAL-Texte (192 Byte RAM). Leer = EN benutzen.
    char        local_[AOG_MSG_COUNT][AOG_LOCAL_LEN] = {};
};

#endif  // AOG_HWMSG_H
