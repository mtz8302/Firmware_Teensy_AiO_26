// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// HwMsgPage.h - Sprache und Texte der AOG-Hardware-Meldungen (PGN 221) - 2026-11-06
// ---------------------------------------------------------------------------------
// AOG blendet diese Texte als Label auf der Karte ein (Config -> Display ->
// "Hardware Messages" muss dort EIN sein). EN und DE sind im Flash kompiliert,
// hier wird nur die dritte Sprache (LOCAL) eingetippt; sie liegt im EEPROM-Block 1300.
// Ein leeres Feld faellt auf Englisch zurueck - das ist Absicht, damit eine
// unvollstaendige eigene Uebersetzung nie ein Fehler ist.

#ifndef HW_MSG_PAGE_H
#define HW_MSG_PAGE_H

#include <Arduino.h>

const char HW_MSG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AOG Messages - AiO v26</title>
    <link rel="stylesheet" href="/touch.css">
    <style>
        .wrap { max-width: 720px; margin: 0 auto; padding: 16px; }
        h2 { margin: 8px 0 4px; }
        .hint { font-size: 13px; opacity: .75; margin: 0 0 16px; }
        .langs { display: flex; gap: 8px; margin-bottom: 18px; flex-wrap: wrap; }
        .langs a {
            padding: 10px 18px; border-radius: 8px; text-decoration: none;
            background: #e8eef5; color: #223; font-weight: 600; border: 2px solid transparent;
        }
        .langs a.active { background: #2d6cdf; color: #fff; border-color: #1b4ea8; }
        label { display: block; font-weight: 600; margin: 14px 0 4px; }
        label span { font-weight: 400; opacity: .65; font-size: 13px; }
        input[type=text] {
            width: 100%; box-sizing: border-box; padding: 10px 12px; font-size: 16px;
            border: 1px solid #b9c4d0; border-radius: 8px;
        }
        .row { display: flex; gap: 10px; align-items: center; margin-top: 6px; font-size: 13px; opacity: .8; }
        button {
            margin-top: 20px; padding: 12px 26px; font-size: 16px; font-weight: 700;
            border: 0; border-radius: 10px; background: #2d6cdf; color: #fff; cursor: pointer;
        }
        button:active { background: #1b4ea8; }
        #st { margin-left: 12px; font-weight: 600; }
        .back { display: inline-block; margin-top: 24px; font-size: 14px; }
    </style>
</head>
<body>
<div class="wrap">
    <h2>AOG hardware messages</h2>
    <p class="hint">
        These texts appear in AgOpenGPS as an on-screen message (PGN 221).
        In AOG enable <b>Config &rarr; Display &rarr; Hardware Messages</b>.
        An empty field falls back to English.
    </p>

    <div class="langs">
        <a href="/lang/en"    data-lang="0">English</a>
        <a href="/lang/de"    data-lang="1">Deutsch</a>
        <a href="/lang/local" data-lang="2">Local (below)</a>
    </div>

    <label>Trees / weak GNSS fix <span>- warning, 5 s</span>
        <input type="text" id="weakFix" maxlength="23" placeholder="Empfang schwach">
    </label>
    <label>Heading bridge active <span>- notice, 5 s</span>
        <input type="text" id="bridge" maxlength="23" placeholder="Kurs: Bruecke">
    </label>
    <label>No course, output holds <span>- notice, 5 s</span>
        <input type="text" id="noCourse" maxlength="23" placeholder="Kein Kurs - halten">
    </label>
    <label>Position jump <span>- warning, 5 s</span>
        <input type="text" id="posJump" maxlength="23" placeholder="Positionssprung">
    </label>

    <div class="row">max. 23 characters each &middot; stored in EEPROM, written only when you save</div>

    <button onclick="save()">Save</button><span id="st"></span>

    <a class="back" href="/">&larr; back</a>
</div>

<script>
const KEYS = ['weakFix', 'bridge', 'noCourse', 'posJump'];

function setLangActive(lang) {
    document.querySelectorAll('[data-lang]').forEach(function (el) {
        el.classList.toggle('active', Number(el.dataset.lang) === Number(lang));
    });
}

async function load() {
    try {
        const r = await fetch('/api/hwmsg/config');
        const c = await r.json();
        KEYS.forEach(function (k) {
            const el = document.getElementById(k);
            if (el) { el.value = c[k] || ''; }
        });
        setLangActive(c.lang);
    } catch (e) {
        document.getElementById('st').textContent = 'load failed';
    }
}

async function save() {
    const body = {};
    KEYS.forEach(function (k) { body[k] = document.getElementById(k).value; });
    document.getElementById('st').textContent = 'saving...';
    try {
        const r = await fetch('/api/hwmsg/config', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body)
        });
        const j = await r.json();
        document.getElementById('st').textContent =
            (j.status === 'ok') ? 'saved' : ('error: ' + (j.message || 'unknown'));
    } catch (e) {
        document.getElementById('st').textContent = 'save failed';
    }
}

load();
</script>
</body>
</html>
)rawliteral";

#endif  // HW_MSG_PAGE_H
