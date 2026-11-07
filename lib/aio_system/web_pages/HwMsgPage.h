// HwMsgPage.h - Sprache und Texte der AOG-Hardware-Meldungen (PGN 221) - 2026-11-06
// ---------------------------------------------------------------------------------
// AOG blendet diese Texte als Label auf der Karte ein (Config -> Display ->
// "Hardware Messages" muss dort EIN sein). EN und DE sind im Flash kompiliert,
// hier wird nur die dritte Sprache (LOCAL) eingetippt; sie liegt im EEPROM-Block 1300.
// Ein leeres Feld faellt auf Englisch zurueck - das ist Absicht, damit eine
// unvollstaendige eigene Uebersetzung nie ein Fehler ist.
//
// Aufbau = Vorlage TouchFriendlyDeviceSettingsPage.h (Betreiber-Vorgabe 2026-11-06):
//   .container (1024px) / h1 / .nav-buttons (2 Spalten) / #status / .card / .help-text
// Bewusst KEIN <h2>: h2 ist in touch.css WEISS und waere auf der weissen Karte unsichtbar.

#ifndef HW_MSG_PAGE_H
#define HW_MSG_PAGE_H

#include <Arduino.h>

const char HW_MSG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <meta name="apple-mobile-web-app-capable" content="yes">
    <title>Messages &amp; Language - AiO v26</title>
    <link rel="stylesheet" href="/touch.css">
    <style>
        .nav-buttons {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-bottom: 20px;
        }
        .help-text {
            font-size: 14px;
            color: #7f8c8d;
            margin-top: 5px;
            line-height: 1.3;
        }
        /* Schalter-Zeile: die vier Regeln sind WOERTLICH aus TouchFriendlyDeviceSettingsPage.h
           uebernommen (.toggle-switch/.toggle-slider selbst stehen in touch.css), damit der
           Schalter genau wie dort aussieht und sitzt. */
        .toggle-container {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 12px 0;
            border-bottom: 1px solid #ecf0f1;
        }
        .toggle-container:last-child {
            border-bottom: none;
        }
        .toggle-label {
            font-size: 18px;
            font-weight: 500;
            flex: 1;
            margin-right: 15px;
        }
        .toggle-info {
            flex: 1;
        }
        /* Ueberschrift INNERHALB einer Karte: dunkel, weil .card weiss ist */
        .section-title {
            font-size: 19px;
            font-weight: 700;
            color: #2c3e50;
            margin-bottom: 10px;
        }
        .lang-row {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }
        .lang-btn {
            display: block;
            padding: 16px 8px;
            border-radius: 10px;
            text-align: center;
            text-decoration: none;
            font-size: 17px;
            font-weight: 600;
            background: #ecf0f1;
            color: #2c3e50;
            border: 3px solid transparent;
        }
        .lang-btn.active {
            background: #3498db;
            color: #ffffff;
            border-color: #2c3e50;
        }
        .msg-row {
            padding: 12px 0;
            border-bottom: 1px solid #ecf0f1;
        }
        .msg-row:last-of-type {
            border-bottom: none;
        }
        .msg-head {
            font-size: 17px;
            font-weight: 600;
            color: #2c3e50;
            margin-bottom: 6px;
        }
        .msg-sub {
            font-size: 14px;
            font-weight: 400;
            color: #7f8c8d;
        }
        /* Zeigt, was JETZT nach AgOpenGPS geht - die Felder darueber enthalten nur die
           LOCAL-Texte. Ohne diese Zeile sah es nach dem Sprachwechsel so aus, als waere
           nichts umgestellt (Betreiber-Befund 2026-11-06). */
        .active-line {
            font-size: 14px;
            color: #2c3e50;
            margin-top: 5px;
        }
        .active-line span {
            font-weight: 600;
        }
        .msg-row input[type=text] {
            width: 100%;
            box-sizing: border-box;
            padding: 12px 14px;
            font-size: 17px;
            border: 2px solid #bdc3c7;
            border-radius: 8px;
            background: #ffffff;
            color: #111111;
        }
        /* Kein eigenes .ok/.err: die Projekt-Konvention ist .status.success (gruener
           Hintergrund, weisse Schrift) und .status.error (rot) - definiert in touch.css
           und von allen anderen Seiten benutzt. Eigene Schriftfarben sahen daneben
           anders aus (Betreiber-Befund 2026-11-06). */
        @media (max-width: 480px) {
            .nav-buttons, .lang-row { grid-template-columns: 1fr; }
        }
    </style>
</head>

<body>
    <div class="container">
        <h1>Messages &amp; Language</h1>

        <div class="nav-buttons">
            <button type="button" class="touch-button" style="background: #7f8c8d;"
                    onclick="window.location.href='/'">
                Back to Home
            </button>
            <button type="button" class="touch-button" onclick="save()">
                Apply Changes
            </button>
        </div>

        <div id="status" class="status">loading...</div>

        <div class="card">
            <div class="toggle-container">
                <div class="toggle-info">
                    <label class="toggle-label" for="enabled">Hardware messages on</label>
                    <div class="help-text">
                        Show warnings and the clock in AgOpenGPS. Off = nothing is sent at all.
                    </div>
                </div>
                <label class="toggle-switch">
                    <input type="checkbox" id="enabled" onchange="saveEnabled()">
                    <span class="toggle-slider"></span>
                </label>
            </div>
        </div>

        <div class="card">
            <div class="section-title">Language</div>
            <div class="lang-row">
                <a class="lang-btn" data-lang="0" href="/lang/en?back=hwmsg">English</a>
                <a class="lang-btn" data-lang="1" href="/lang/de?back=hwmsg">Deutsch</a>
                <a class="lang-btn" data-lang="2" href="/lang/local?back=hwmsg">Local</a>
            </div>
            <div class="help-text">
                "Local" uses the texts below. The web interface itself stays in English.
            </div>
        </div>

        <div class="card">
            <div class="section-title">Texts shown in AgOpenGPS</div>
            <div class="help-text" style="margin-bottom: 10px;">
                AgOpenGPS shows these as an on-screen message. Enable
                <b>Config &rarr; Display &rarr; Hardware Messages</b> in AOG, otherwise it
                stays hidden. An empty field falls back to English.
            </div>

            <div class="msg-row">
                <div class="msg-head">Weak GNSS fix <span class="msg-sub">&middot; warning &middot; 5 s</span></div>
                <input type="text" id="weakFix" maxlength="47" placeholder="Weak GNSS fix">
                <div class="active-line">active now: <span id="act_weakFix">-</span></div>
            </div>
            <div class="msg-row">
                <div class="msg-head">Poor GNSS heading quality <span class="msg-sub">&middot; notice &middot; 5 s</span></div>
                <input type="text" id="bridge" maxlength="47" placeholder="poor GNSS Heading quality">
                <div class="active-line">active now: <span id="act_bridge">-</span></div>
            </div>
            <div class="msg-row">
                <div class="msg-head">No course, heading holds <span class="msg-sub">&middot; notice &middot; 5 s</span></div>
                <input type="text" id="noCourse" maxlength="47" placeholder="No course - hold">
                <div class="active-line">active now: <span id="act_noCourse">-</span></div>
            </div>
            <div class="msg-row">
                <div class="msg-head">Position jump <span class="msg-sub">&middot; warning &middot; 5 s</span></div>
                <input type="text" id="posJump" maxlength="47" placeholder="Position jump">
                <div class="active-line">active now: <span id="act_posJump">-</span></div>
            </div>

            <div class="help-text" style="margin-top: 10px;">
                The fields hold the <b>local</b> texts only (they go out when the language is
                "Local"). "active now" shows what AgOpenGPS gets with the language above.
                Max. 47 characters each; written to the EEPROM only when you apply.
            </div>
            <div style="margin-top: 12px;">
                <button type="button" class="touch-button" style="background: #7f8c8d; padding: 14px 18px; font-size: 16px;"
                        onclick="copyActive()">
                    Copy the active texts into the fields
                </button>
            </div>
        </div>
    </div>

<script>
const KEYS = ['weakFix', 'bridge', 'noCourse', 'posJump'];
let CURRENT_ACTIVE = {};
const LANG_NAME = ['English', 'Deutsch', 'Local'];

// kind: 'success' | 'error' | null - genau die Klassen aus touch.css (gleiche Optik wie
// alle anderen Seiten: gruen/weiss bzw. rot/weiss; null = neutraler grauer Kasten).
function setStatus(text, kind) {
    const el = document.getElementById('status');
    el.innerText = text;
    el.className = 'status' + (kind === 'success' ? ' success' : (kind === 'error' ? ' error' : ''));
}

function setLangActive(lang) {
    document.querySelectorAll('[data-lang]').forEach(function (el) {
        el.classList.toggle('active', Number(el.dataset.lang) === Number(lang));
    });
}

// "active now": der Text, der mit der eingestellten Sprache WIRKLICH rausgeht.
function setActiveTexts(active, lang) {
    CURRENT_ACTIVE = active || {};
    KEYS.forEach(function (k) {
        const el = document.getElementById('act_' + k);
        if (el) { el.innerText = CURRENT_ACTIVE[k] || '-'; }
    });
    const name = LANG_NAME[Number(lang)];
    if (name) { setStatus('Language: ' + name + ' - the texts above are what AgOpenGPS shows.', null); }
}

// Die aktiven Texte in die Felder uebernehmen, damit man sie abwandeln kann
// (z. B. Deutsch -> eigener Dialekt). Gespeichert wird erst mit "Apply Changes".
function copyActive() {
    KEYS.forEach(function (k) {
        const el = document.getElementById(k);
        if (el && CURRENT_ACTIVE[k]) { el.value = CURRENT_ACTIVE[k].substring(0, 47); }
    });
    setStatus('Copied the active texts into the fields - press "Apply Changes" to store them.', null);
}

// Hauptschalter: wirkt sofort (nicht erst mit "Apply Changes"), weil er der
// Ein/Aus-Schalter der ganzen Funktion ist. Wird gleich mit ins EEPROM geschrieben.
function saveEnabled() {
    const on = document.getElementById('enabled').checked;
    setStatus(on ? 'Switching on...' : 'Switching off...', null);
    fetch('/api/hwmsg/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: on })
    }).then(function (r) { return r.json(); }).then(function (j) {
        if (j.status === 'ok') {
            setStatus(on ? 'Hardware messages are ON.' : 'Hardware messages are OFF.', 'success');
        } else { setStatus('Error: ' + (j.message || 'unknown'), 'error'); }
    }).catch(function (e) {
        setStatus('Save failed: ' + e, 'error');
    });
}

function load() {
    fetch('/api/hwmsg/config').then(function (r) { return r.json(); }).then(function (c) {
        const en = document.getElementById('enabled');
        if (en) { en.checked = (c.enabled !== false); }
        KEYS.forEach(function (k) {
            const el = document.getElementById(k);
            if (el) { el.value = c[k] || ''; }
        });
        setLangActive(c.lang);
        setActiveTexts(c.active, c.lang);
    }).catch(function (e) {
        // Sichtbar scheitern: ein stiller Fehler sieht aus wie "nichts gespeichert".
        setStatus('Could not load: ' + e, 'error');
    });
}

function save() {
    const body = {};
    KEYS.forEach(function (k) { body[k] = document.getElementById(k).value; });
    setStatus('Applying...', null);
    fetch('/api/hwmsg/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
    }).then(function (r) { return r.json(); }).then(function (j) {
        if (j.status === 'ok') {
            // Nach dem Speichern die aktiven Texte neu holen: bei Sprache "Local" aendert
            // sich damit auch die Anzeige "active now".
            setStatus('Saved. The new texts are active now.', 'success');
            load();
        } else { setStatus('Error: ' + (j.message || 'unknown'), 'error'); }
    }).catch(function (e) {
        setStatus('Save failed: ' + e, 'error');
    });
}

load();
</script>
</body>
</html>
)rawliteral";

#endif  // HW_MSG_PAGE_H
