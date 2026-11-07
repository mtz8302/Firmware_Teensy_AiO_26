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
        .status.ok { color: #1e7e34; }
        .status.err { color: #b02a37; }
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
            <div class="section-title">Language</div>
            <div class="lang-row">
                <a class="lang-btn" data-lang="0" href="/lang/en">English</a>
                <a class="lang-btn" data-lang="1" href="/lang/de">Deutsch</a>
                <a class="lang-btn" data-lang="2" href="/lang/local">Local</a>
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
                <input type="text" id="weakFix" maxlength="23" placeholder="Weak GNSS fix">
            </div>
            <div class="msg-row">
                <div class="msg-head">Poor GNSS heading quality <span class="msg-sub">&middot; notice &middot; 5 s</span></div>
                <input type="text" id="bridge" maxlength="23" placeholder="poor GNSS Heading quality">
            </div>
            <div class="msg-row">
                <div class="msg-head">No course, heading holds <span class="msg-sub">&middot; notice &middot; 5 s</span></div>
                <input type="text" id="noCourse" maxlength="23" placeholder="No course - hold">
            </div>
            <div class="msg-row">
                <div class="msg-head">Position jump <span class="msg-sub">&middot; warning &middot; 5 s</span></div>
                <input type="text" id="posJump" maxlength="23" placeholder="Position jump">
            </div>

            <div class="help-text" style="margin-top: 10px;">
                Max. 23 characters each. Stored in the EEPROM and written only when you apply.
            </div>
        </div>
    </div>

<script>
const KEYS = ['weakFix', 'bridge', 'noCourse', 'posJump'];

function setStatus(text, ok) {
    const el = document.getElementById('status');
    el.innerText = text;
    el.className = 'status' + (ok === true ? ' ok' : (ok === false ? ' err' : ''));
}

function setLangActive(lang) {
    document.querySelectorAll('[data-lang]').forEach(function (el) {
        el.classList.toggle('active', Number(el.dataset.lang) === Number(lang));
    });
}

function load() {
    fetch('/api/hwmsg/config').then(function (r) { return r.json(); }).then(function (c) {
        KEYS.forEach(function (k) {
            const el = document.getElementById(k);
            if (el) { el.value = c[k] || ''; }
        });
        setLangActive(c.lang);
        setStatus('Ready.', true);
    }).catch(function (e) {
        // Sichtbar scheitern: ein stiller Fehler sieht aus wie "nichts gespeichert".
        setStatus('Could not load: ' + e, false);
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
        if (j.status === 'ok') { setStatus('Saved. The new texts are active now.', true); }
        else { setStatus('Error: ' + (j.message || 'unknown'), false); }
    }).catch(function (e) {
        setStatus('Save failed: ' + e, false);
    });
}

load();
</script>
</body>
</html>
)rawliteral";

#endif  // HW_MSG_PAGE_H
