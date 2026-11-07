// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.

// VWASConfigPage.h
// Virtual WAS configuration: vehicle geometry (manual entry or AgOpenGPS
// vehicle.xml import), plus centre/endstop calibration buttons.

#ifndef VWAS_CONFIG_PAGE_H
#define VWAS_CONFIG_PAGE_H

#include <Arduino.h>

const char VWAS_CONFIG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Vehicle settings - AiO v26</title>
<link rel="stylesheet" href="/touch.css">
<style>
  .section { background:#fff; color:#1b2a3a; border-radius:10px; padding:16px; margin:12px 0; box-shadow:0 1px 3px rgba(0,0,0,0.15); }
  .section h3 { margin-top:0; color:#1b2a3a; }
  /* Two columns on a wide screen: the page is as wide as the other touch pages
     (1024px), and a single row of label+input stretched over that width reads
     badly and wastes half of it. Collapses to one column on a tablet/phone. */
  .cols { display:grid; grid-template-columns:1fr 1fr; gap:0 32px; }
  @media (max-width:800px) { .cols { grid-template-columns:1fr; } }
  .field { display:flex; align-items:center; justify-content:space-between; margin:8px 0; }
  .field label { flex:1; padding-right:10px; }
  .field input, .field select { width:130px; padding:6px; font-size:16px; }
  .hint { color:#4a5560; font-size:13px; margin:2px 0 10px 0; }
  .btn { padding:12px 18px; font-size:16px; border-radius:8px; border:none; margin:6px 6px 6px 0; cursor:pointer; }
  .btn-primary { background:#2e7d32; color:#fff; }
  .btn-danger { background:#c62828; color:#fff; }
  .btn-secondary { background:#546e7a; color:#fff; }
  .status-box { font-family:monospace; font-size:14px; background:#f5f5f5; color:#10202b; border-radius:8px; padding:10px; white-space:pre-wrap; }
  .warn { background:#fff3cd; color:#5c3d00; border:1px solid #a05200; border-radius:8px; padding:10px; margin:10px 0; }
  .nav-buttons { display:grid; grid-template-columns:1fr 1fr; gap:15px; margin:0 0 14px 0; }
  @media (max-width:600px) { .nav-buttons { grid-template-columns:1fr; } }
  .meas { font-family:monospace; font-size:13px; background:#eef4ff; border-radius:6px; padding:4px 8px; display:block; margin:-4px 0 10px 0; }
  .xmlval { display:none; font-family:monospace; font-size:13px; background:#eef4ff; border:1px dashed #7aa7d9; border-radius:6px; padding:4px 8px; margin:-4px 0 10px 0; cursor:pointer; }
  .xmlval.diff { background:#fff3cd; border-color:#a05200; }
</style>
</head>
<body>
<div style="max-width:1024px;margin:0 auto;padding:10px;">
  <h2>Vehicle settings</h2>
  <div class="nav-buttons">
    <button type="button" class="touch-button" style="background:#7f8c8d;"
            onclick="window.location.href='/'">Back</button>
    <button type="button" class="touch-button" style="background:#2e7d32;"
            onclick="saveConfig();saveGeometry();">Save</button>
  </div>

  <div class="section">
    <h3>Live status</h3>
    <div id="liveStatus" class="status-box">loading...</div>
  </div>

  <div class="section">
    <h3>Import from AgOpenGPS</h3>
    <div class="hint">Pick <code>vehicle.xml</code> from <code>Documents\AgOpenGPS\Vehicles\...</code>.
      Nothing is written into the fields - each value is offered below its field
      (<i>from XML: ...</i>, highlighted when it differs) and only reaches the board
      when you press <b>Save</b>.</div>
    <input type="file" id="vehicleXmlFile" accept=".xml">
    <button class="btn btn-secondary" onclick="applyAllXml()">Apply all XML values</button>
    <button class="btn btn-secondary" onclick="discardXml()">Discard</button>
    <div id="importResult" class="hint"></div>
  </div>

  <div class="section">
    <h3>Vehicle geometry</h3>
    <div class="cols">
      <div>
        <div class="field"><label>Wheelbase (cm)</label><input type="number" step="1" id="wheelbase"></div>
        <div class="xmlval" id="xml_wheelbase" onclick="applyXml('wheelbase')"></div>
      </div>
      <div>
        <div class="field"><label>Max. steering / joint angle (&deg;)</label><input type="number" step="0.1" id="maxSteeringAngle"></div>
        <div class="xmlval" id="xml_maxSteeringAngle" onclick="applyXml('maxSteeringAngle')"></div>
      </div>
      <div>
        <div class="field"><label>Vehicle type</label>
          <select id="vehicleType">
            <option value="0">Standard (Ackermann)</option>
            <option value="2">Articulated</option>
          </select>
        </div>
        <div class="xmlval" id="xml_vehicleType" onclick="applyXml('vehicleType')"></div>
      </div>
      <div>
        <div class="field"><label>Antenna ahead of pivot (cm)</label><input type="number" step="1" id="antennaPivotM"></div>
        <div class="xmlval" id="xml_antennaPivotM" onclick="applyXml('antennaPivotM')"></div>
      </div>
      <div>
        <div class="field"><label>Side hill gain (as AOG)</label><input type="number" step="0.005" id="sideHillGain"></div>
        <div class="xmlval" id="xml_sideHillGain" onclick="applyXml('sideHillGain')"></div>
        <div class="hint" id="gainInfo">-</div>
      </div>
    </div>
    <div class="hint"><b>Articulated:</b> enter the FULL joint angle - AgOpenGPS halves it
      itself. A joint beyond <code>1.2 &times;</code> this value is rejected and the output is
      clamped to &plusmn;this value, so too small here drops every tight turn
      (measured 01.11.: 37 entered, real 75).</div>
    <div class="hint"><b>Antenna ahead of pivot:</b> distance from the pivot to the antenna,
      positive forward. Pivot is the <b>rear axle</b> on a tractor, the <b>joint</b> on an
      articulated machine. The value that counts is the one AgOpenGPS's <b>main screen</b>
      draws correctly - its configuration drawing uses the rear axle as origin and is
      misleading for articulated. 0 = no lead correction.</div>
    <div class="hint"><b>Side hill gain:</b> magnitude only, identical to AOG's slider.
      The firmware derives the sign itself - the roll it reports is inverted relative to
      AOG. Needed so the zero-point trim does not read the roll effect on a slope as a
      zero error.</div>
    <button class="btn btn-primary" onclick="saveConfig()">Save vehicle geometry</button>
    <div id="saveResult" class="hint"></div>
  </div>

  <div class="section">
    <h3>Antenna geometry (baseline)</h3>
    <div class="hint">Both values must be identical here <b>and in AgOpenGPS</b> - the firmware
      and AOG use the same formula.</div>
    <div class="cols">
      <div>
        <div class="field"><label>Antenna spacing (cm)</label>
          <input type="number" step="0.5" id="baselineMeters">
        </div>
        <div class="meas" id="measBaseline">measured: -</div>
      </div>
      <div>
        <div class="field"><label>Dual heading offset (&deg;)</label>
          <input type="number" step="0.1" id="headingOffsetDeg">
        </div>
        <div class="meas" id="measMountOffset">measured: -</div>
        <div class="xmlval" id="xml_headingOffsetDeg" onclick="applyXml('headingOffsetDeg')"></div>
      </div>
    </div>
    <div class="hint"><b>Antenna spacing:</b> the mechanical distance between the two antenna
      phase centres. It is the <b>reference</b> for the plausibility test of the solved
      baseline: a length far off it means the heading solution is not trustworthy and is
      dropped. <b>Measured: 71.1 cm</b> (15 drives, 13 065 RTK frames). Without an entry
      this build value applies. <b>0 = test off</b>, the reference is learned again (the
      fragile old path - escape hatch only).</div>
    <div class="hint"><b>Dual heading offset:</b> the angle between the antenna baseline and
      the vehicle's longitudinal axis, with AOG's sign. <b>Measured: 88.3&deg;</b> (straight
      driving, 3 drives, 520 samples, sd 0.9&deg;). A generic 90 costs 1.7&deg; in <b>every</b>
      frame. Enter the same value in AgOpenGPS.</div>
    <button class="btn btn-primary" onclick="saveGeometry()">Save antenna geometry</button>
    <button class="btn btn-secondary" onclick="reloadAll()">Reload from device</button>
    <button class="btn btn-secondary" onclick="useMeasuredRef()">Use measured length as reference</button>

    <div id="geometryResult" class="hint">-</div>
  </div>

</div>

<script>
function loadConfig() {
  fetch('/api/vwas/config').then(r => r.json()).then(cfg => {
    // API in METERN, Feld in CM (siehe LEN_CM oben).
    document.getElementById('wheelbase').value = roundLen('wheelbase', toField('wheelbase', cfg.wheelbase));
    document.getElementById('maxSteeringAngle').value = cfg.maxSteeringAngle;
    document.getElementById('vehicleType').value = cfg.vehicleType;
    document.getElementById('antennaPivotM').value =
      roundLen('antennaPivotM', toField('antennaPivotM', (cfg.antennaPivotM === undefined ? 0 : cfg.antennaPivotM)));
    document.getElementById('sideHillGain').value = cfg.sideHillGain;
    gainInfo(cfg);
  }).catch(e => {
    // Visible on purpose. A silent failure here leaves every form field empty,
    // which is indistinguishable from "the EEPROM contains wrong values" - the
    // user sees a page of wrong-looking numbers and no indication that they were
    // never loaded at all.
    document.getElementById('saveResult').innerText =
      'Could not load configuration: ' + e;
  });
}

// Shows the three numbers side by side: what was configured (AgOpenGPS
// convention), what the firmware actually uses (with the derived sign), and what
// the diagnostic estimator reports. A reported value far from the effective one -
// or of the opposite sign - means the configuration is wrong, and that has to be
// visible without a rebuild.
function gainInfo(cfg) {
  const eff = Number(cfg.sideHillGainEffective);
  const rep = Number(cfg.sideHillGainReported);
  let t = 'effective internally ' + eff.toFixed(3) + ' deg/deg   (estimator reports ';
  if (!isFinite(rep) || rep === 0) {
    t += 'nothing yet - it needs a drive with >4 deg roll travel)';
  } else {
    t += rep.toFixed(3) + ' deg/deg';
    if (Math.abs(rep - eff) > 0.1) t += '  <- far off: check value and sign';
    t += ')';
  }
  document.getElementById('gainInfo').innerText = t;
}

function saveConfig() {
  const body = {
    // Felder in CM -> API in METERN.
    wheelbase: fromField('wheelbase', parseFloat(document.getElementById('wheelbase').value)),
    maxSteeringAngle: parseFloat(document.getElementById('maxSteeringAngle').value),
    vehicleType: parseInt(document.getElementById('vehicleType').value),
    antennaPivotM: fromField('antennaPivotM', parseFloat(document.getElementById('antennaPivotM').value) || 0),
    sideHillGain: parseFloat(document.getElementById('sideHillGain').value)
  };
  fetch('/api/vwas/config', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(body) })
    .then(r => r.json())
    .then(res => {
      if (res.status === 'saved') {
        let t = 'Saved.';
        if (typeof res.sideHillGainEffective === 'number') {
          t += '  effective internally ' + res.sideHillGainEffective.toFixed(3) + ' deg/deg';
        }
        document.getElementById('saveResult').innerText = t;
        loadConfig();   // refresh the derived/reported display in the same click
      } else {
        document.getElementById('saveResult').innerText = 'Error: ' + res.message;
      }
    })
    .catch(e => { document.getElementById('saveResult').innerText = 'Save failed: ' + e; });
}

// --- XML-Import: erst ZEIGEN, dann uebernehmen (B10.73, 2026-11-01) -------------
// Vorher wurden die XML-Werte sofort in die Felder geschrieben - der Anwender sah
// danach nicht mehr, was vorher dastand. Jetzt bleiben die Felder unberuehrt und
// unter jedem Feld steht "aus XML: <Wert>" (gelb, wenn er abweicht). Antippen
// uebernimmt EINEN Wert, "Apply all XML values" applies them all. Nothing is saved until "Save".
var xmlVals = {};
// EINHEITEN (2026-11-06, Betreiber): die Felder fuehren Laengen in **cm** - so zeigt
// AgOpenGPS sie in seiner Oberflaeche an (`mf.m2InchOrCm` in ConfigVehicle.Designer.cs).
// API und AOGs vehicle.xml fuehren sie dagegen in **m** (AOG speichert
// `setVehicle_wheelbase` in Metern und rechnet nur fuer die Anzeige um). Umgerechnet wird
// an genau diesen zwei Stellen - nirgends sonst, sonst rechnet man doppelt.
var LEN_CM = { wheelbase: 100, antennaPivotM: 100, baselineMeters: 100 };
function toField(key, meters) { return (key in LEN_CM) ? meters * LEN_CM[key] : meters; }
function fromField(key, value) { return (key in LEN_CM) ? value / LEN_CM[key] : value; }
function roundLen(key, v) { return (key in LEN_CM) ? Math.round(v * 10) / 10 : v; }
var xmlLabels = { wheelbase: 'Wheelbase', maxSteeringAngle: 'Steering angle',
                  vehicleType: 'Vehicle type', antennaPivotM: 'Antenna position',
                  headingOffsetDeg: 'Dual heading offset', sideHillGain: 'Side hill gain' };

function fieldValue(key) {
  var el = document.getElementById(key);
  return el.value === '' ? NaN : Number(el.value);
}

function renderXmlGhosts() {
  for (var key in xmlLabels) {
    var box = document.getElementById('xml_' + key);
    if (!box) continue;
    if (!(key in xmlVals)) { box.style.display = 'none'; box.innerText = ''; continue; }
    var cur = fieldValue(key);
    var nv = toField(key, xmlVals[key]);
    var isBool = (typeof nv === 'boolean');
    var diff = isBool ? (nv !== cur) : (!isFinite(cur) || Math.abs(nv - cur) > 1e-6);
    box.style.display = 'block';
    box.className = 'xmlval' + (diff ? ' diff' : '');
    var unitTxt = (key in LEN_CM) ? ' cm' : '';
    box.innerText = 'from XML: ' + (isBool ? (nv ? 'yes' : 'no') : roundLen(key, nv)) + unitTxt +
                    (diff ? '   -> tap to apply' : '   (identical)');
  }
}

function applyXml(key) {
  if (!(key in xmlVals)) return;
  document.getElementById(key).value = roundLen(key, toField(key, xmlVals[key]));
  renderXmlGhosts();
  document.getElementById('importResult').innerText =
    xmlLabels[key] + ' applied - NOT saved yet (press "Save").';
}

function applyAllXml() {
  var n = 0;
  for (var key in xmlVals) {
    document.getElementById(key).value = roundLen(key, toField(key, xmlVals[key]));
    n++;
  }
  renderXmlGhosts();
  document.getElementById('importResult').innerText = n
    ? (n + ' value(s) applied - NOT saved yet (press "Save").')
    : 'No XML values loaded.';
}

function discardXml() {
  xmlVals = {};
  renderGhostsSafe();
  document.getElementById('importResult').innerText = 'XML values discarded - fields unchanged.';
}

function renderGhostsSafe() { try { renderXmlGhosts(); } catch (e) {} }
// --- vehicle.xml import (parsed entirely in the browser - the board never
// sees the raw file, only the extracted numbers after you press Save) ---
document.getElementById('vehicleXmlFile').addEventListener('change', function(e) {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = function(evt) {
    try {
      const doc = new DOMParser().parseFromString(evt.target.result, 'text/xml');
      const settings = {};
      doc.querySelectorAll('setting').forEach(node => {
        const name = node.getAttribute('name');
        const valueNode = node.querySelector('value');
        if (name && valueNode) settings[name] = valueNode.textContent;
      });

      // B10.73 (2026-11-01, Betreiber): NICHT mehr still in die Felder schreiben -
      // sonst sieht der Anwender nicht mehr, was vorher dastand.
      xmlVals = {};
      if (settings['setVehicle_wheelbase'] !== undefined)
        xmlVals.wheelbase = Number(settings['setVehicle_wheelbase']);
      if (settings['setVehicle_maxSteerAngle'] !== undefined)
        xmlVals.maxSteeringAngle = Number(settings['setVehicle_maxSteerAngle']);
      if (settings['setVehicle_vehicleType'] !== undefined)
        xmlVals.vehicleType = Number(settings['setVehicle_vehicleType']);
      if (settings['setVehicle_antennaPivot'] !== undefined)
        xmlVals.antennaPivotM = Number(settings['setVehicle_antennaPivot']);
      // 2026-11-01: die beiden Werte standen schon immer in AOGs XML, wurden aber nie
      // gelesen - der Antennenwinkel (Basislinie -> Laengsachse) und der Side-Hill-Gain.
      // Beide erscheinen wie die anderen als antippbare Zeile, nichts wird ueberschrieben.
      if (settings['setGPS_dualHeadingOffset'] !== undefined)
        xmlVals.headingOffsetDeg = Number(settings['setGPS_dualHeadingOffset']);
      if (settings['setAS_sideHillComp'] !== undefined)
        xmlVals.sideHillGain = Number(settings['setAS_sideHillComp']);
      // NOTE: setGPS_headingFromWhichSource ("Dual") wird NICHT mehr importiert - die
      // Checkbox "Dual-Heading bevorzugen" ist entfallen (2026-11-01). Die Fusion nimmt die
      // Dual-Lage immer, wenn sie vorhanden und sauber ist; die Absicherung ist die
      // QF-Maske des Empfaengers.
      // NOTE: setAS_countsPerDegree is intentionally NOT imported - it calibrates
      // AgOpenGPS' analog-WAS ADC reading, not our encoder (AOG owns that value).
      renderXmlGhosts();
      var cnt = Object.keys(xmlVals).length;
      document.getElementById('importResult').innerText = cnt
        ? ('Read ' + cnt + ' value(s). Fields unchanged - "from XML: ..." is waiting under '
           + 'each field (tap it), or use "Apply all XML values".')
        : 'No known fields found in this file.';
    } catch (err) {
      document.getElementById('importResult').innerText = 'Could not read the XML file.';
    }
  };
  reader.readAsText(file);
});

function zStateText(st) {
  if (st === 1) return 'ACQUIRING (corrected)';
  if (st === 2) return 'HOLDING (heavily blocked)';
  if (st === 3) return 'LOCKED (autosteer/AOG)';
  return 'insufficient data';
}

// Live status: deliberately SHORT (a wall of numbers is not read). The antenna
// measurements sit next to the input fields of the antenna geometry section.
function pollStatus() {
  fetch('/api/vwas/status').then(r => r.json()).then(s => {
    var t = '';
    t += 'Steering / joint angle: ' + s.fusedAngle.toFixed(2) + '\u00b0' +
         '   (raw ' + s.encoderAngle.toFixed(2) + '\u00b0)\n';
    t += 'GPS angle (curvature): ' + (s.gpsAngleValid ? s.gpsAngle.toFixed(2) + '\u00b0' : 'not available') + '\n';
    t += 'Zero point: ' + zStateText(s.zeroState) + '   residual ' + s.zeroResidualDeg.toFixed(2) + '\u00b0' +
         '   fits ' + s.zeroFits + '   rate ' + s.zeroRate.toFixed(2) + ' \u00b0/s\n';
    t += 'Roll: ' + (s.rollValid ? s.roll.toFixed(2) + '\u00b0' : 'not available') + '\n';
    t += 'Heading: ' + s.bridgeState + ' / ' + s.bridgeSource + '   sigma ' + s.bridgeSigmaDeg.toFixed(2) + '\u00b0' +
         '   attitude age ' + (s.bridgeAttAgeMs / 1000).toFixed(1) + ' s\n';
    t += 'Direction: ' + (s.bridgeReverse ? 'reverse' : 'forward');
    document.getElementById('liveStatus').innerText = t;
    // Measurements next to the input fields (free while we are here)
    var mo = document.getElementById('measMountOffset');
    if (mo) mo.innerText = (s.bridgeMountSamples > 0)
      ? 'measured: ' + s.bridgeMountOffsetDeg.toFixed(2) + '\u00b0  (n=' + s.bridgeMountSamples + ')'
      : 'measured: no sample yet';
    var mb = document.getElementById('measBaseline');
    if (mb) mb.innerText = (s.bridgeBaselineM > 0.01)
      ? 'measured: ' + (s.bridgeBaselineM * 100).toFixed(1) + ' cm' +
        (s.bridgeBaselineDevPct !== 0 ? '  (deviation ' + s.bridgeBaselineDevPct.toFixed(1) + ' %)' : '')
      : 'measured: not provided by the receiver';
  }).catch(e => {
    document.getElementById('liveStatus').innerText = 'Status request failed: ' + e;
  });
}

// Antenna geometry: own endpoint (these are HeadingBridge settings, not VWAS ones), so it
// works even when the VWAS is not active.
function loadGeometry() {
  fetch('/api/bridge/geometry').then(r => r.json()).then(b => {
    // 0 = "learned": leave the field EMPTY, so nobody re-enters a 0 by accident.
    // Feld in CM, API in METERN (siehe LEN_CM oben).
    document.getElementById('baselineMeters').value =
      (b.meters > 0.001 ? roundLen('baselineMeters', toField('baselineMeters', b.meters)) : '');
    document.getElementById('headingOffsetDeg').value =
      (typeof b.offsetDeg === 'number' ? b.offsetDeg.toFixed(1) : '');
    let t = '--- Antenna spacing ---\n';
    if (b.set) {
      t += 'FIXED reference ' + (b.meters * 100).toFixed(1) + ' cm (stored in EEPROM)';
    } else if (b.meters > 0.001) {
      t += 'FIXED reference ' + (b.meters * 100).toFixed(1) + ' cm (build value, nothing in EEPROM)';
    } else {
      t += 'NO fixed reference - it is LEARNED from the receiver';
    }
    t += '\n';
    t += 'Reference in use: ' + (b.refM > 0.01 ? (b.refM * 100).toFixed(1) + ' cm' : 'none yet') + '\n';
    t += 'Solved by receiver: ' +
         (b.lenM > 0.01 ? (b.lenM * 100).toFixed(1) + ' cm' : 'not provided') +
         (b.refM > 0.01 && b.lenM > 0.01 ? '   deviation ' + b.devPct.toFixed(1) + ' %' : '') + '\n';
    if (typeof b.suggestM === 'number' && b.suggestCount > 0) {
      t += 'SELF-MEASURED (' + b.suggestCount + ' clean frames, no reference): ' +
           (b.suggestM * 100).toFixed(1) + ' cm' +
           (b.refSuspect ? '   -> DIFFERS from the fixed reference!' : '   -> matches') + '\n';
    } else {
      t += 'Self-measured: no clean frames yet (drive first)\n';
    }

    t += 'hdgstddev (receiver): ' +
         (typeof b.hdgSdDeg === 'number' ? b.hdgSdDeg.toFixed(2) : '?') + ' deg\n';
    if (b.testDisabled) {
      t += 'WARNING: test switched OFF by the safety net - the reference does not match ' +
           'this vehicle.';
    } else if (b.meters > 0.001 && b.refM > 0.01 && Math.abs(b.devPct) > 5) {
      t += 'WARNING: the solved length deviates more than 5 % from the reference.';
    } else {
      t += 'Reference and solved length match.';
    }
    t += '\n--- Dual heading offset ---\n';
    t += 'Firmware uses ' + Number(b.offsetDeg).toFixed(1) + ' deg' +
         (b.offsetStored ? ' (from EEPROM)'
                         : ' (build value ' + Number(b.offsetDefaultDeg).toFixed(1) + ' deg, nothing in EEPROM)') + '\n';
    t += 'Firmware measurement (mnt): ' + Number(b.mountMeasDeg).toFixed(1) + ' deg' +
         '   -> to be entered: ' + (-Number(b.mountMeasDeg)).toFixed(1) + ' deg\n';
    if (Math.abs(Number(b.mountMeasDeg) + Number(b.offsetDeg)) > 3) {
      t += 'DEVIATION > 3 deg: firmware offset and own measurement do not match - ' +
           'enter the same value in AgOpenGPS.';
    } else {
      t += 'matches the firmware measurement. Enter the same value in AgOpenGPS.';
    }
    document.getElementById('geometryResult').innerText = t;
  }).catch(e => {
    document.getElementById('geometryResult').innerText = 'Request failed: ' + e;
  });
}

function saveGeometry() {
  const body = {};
  const rawM = document.getElementById('baselineMeters').value.trim();
  if (rawM !== '') {
    const cm = parseFloat(rawM);
    if (!isFinite(cm)) {
      document.getElementById('geometryResult').innerText = 'Antenna spacing: not a number';
      return;
    }
    body.meters = fromField('baselineMeters', cm);   // Feld in CM -> API in METERN
  }
  const rawO = document.getElementById('headingOffsetDeg').value.trim();
  if (rawO !== '') {
    const o = parseFloat(rawO);
    if (!isFinite(o)) {
      document.getElementById('geometryResult').innerText = 'Offset: not a number';
      return;
    }
    body.offsetDeg = o;
  }
  if (Object.keys(body).length === 0) {
    document.getElementById('geometryResult').innerText =
      'Enter a value first. Antenna spacing 0 = test off (reference is learned).';
    return;
  }
  fetch('/api/bridge/geometry', { method: 'POST', headers: {'Content-Type':'application/json'},
                                  body: JSON.stringify(body) })
    .then(r => r.json())
    .then(res => {
      if (res.status === 'saved') {
        document.getElementById('geometryResult').innerText =
          'Saved: antenna spacing ' + (res.meters * 100).toFixed(1) + ' cm, offset ' +
          res.offsetDeg.toFixed(1) + ' deg.';
        loadGeometry();
      } else {
        document.getElementById('geometryResult').innerText = 'Error: ' + res.message;
      }
    });
}

// "Loesung als Referenz uebernehmen" (10-11): die vom Empfaenger GEMESSENE Laenge als feste
// Referenz speichern. Bewusst eine ausdrueckliche Betreiber-Aktion - die Firmware misst und
// MELDET nur, sie aendert die Referenz nie von selbst (ein falsch uebernommener Wert waere
// genau der Fehler, gegen den das Sicherheitsnetz existiert).
function useMeasuredRef() {
  fetch('/api/bridge/geometry', { method: 'POST', headers: {'Content-Type':'application/json'},
                                  body: JSON.stringify({ useMeasured: true }) })
    .then(r => r.json())
    .then(res => {
      if (res.status === 'saved') {
        document.getElementById('geometryResult').innerText =
          'Applied: ' + (res.meters * 100).toFixed(1) + ' cm as the FIXED reference (measured from ' +
          res.frames + ' clean frames). Now in the EEPROM.';
        loadGeometry();
      } else {
        document.getElementById('geometryResult').innerText = 'Not applied: ' + res.message;
      }
    })
    .catch(e => { document.getElementById('geometryResult').innerText = 'Error: ' + e; });
}


// 2026-11-06: "Reload from device" - ALLE Werte neu holen und sichtbar quittieren.
// Vorher hiess der Knopf "Refresh display" und rief nur loadGeometry(): standen die Werte
// schon im Formular, passierte sichtbar NICHTS - das sah aus wie "der Knopf geht nicht".
// Jetzt: sofortige Rueckmeldung, dann Fahrzeugdaten UND Antennen-Geometrie neu laden.
function reloadAll() {
  const g = document.getElementById('geometryResult');
  const s = document.getElementById('saveResult');
  if (g) { g.innerText = 'reloading from device...'; }
  if (s) { s.innerText = ''; }
  loadConfig();
  loadGeometry();
  if (typeof pollStatus === 'function') { pollStatus(); }
}

loadGeometry();
loadConfig();
pollStatus();
setInterval(pollStatus, 1000);
</script>
</body>
</html>
)rawliteral";

#endif // VWAS_CONFIG_PAGE_H
