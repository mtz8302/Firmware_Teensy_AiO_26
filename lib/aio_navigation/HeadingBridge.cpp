// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// HeadingBridge.cpp - see HeadingBridge.h for the measurements this is built on.

#include "HeadingBridge.h"
#include "GNSSProcessor.h"
#include "IMUProcessor.h"   // yaw rate support (Config::useImuRate)
#include "PositionHistory.h" // Step C: the central per-fix ring (updatePathFromRing)
#include "EventLogger.h"
#include "ConfigManager.h"
#include "WheelAngleFusion.h"   // B10.51: vehicleType/wheelbase (Fahrzeuggeometrie)
#include <math.h>

HeadingBridge headingBridge;

// B10.50 (2026-10-25): Werte dieser Sitzung als dateiweite Konstanten (NICHT in Config/Status -
// ein zusaetzliches Feld dort hat im Sim die Config-Defaults verloren, s. Sitzungsnotiz).
static const uint32_t kChordGateAcceptMs = 3000u;   // so lange muss eine abweichende Richtung STEHEN
static const uint16_t kHoldReturnRampMs = 800u;     // Rampe beim Verlassen des HALTENS (0 = aus)



#ifndef SIM_BUILD
extern ConfigManager configManager;   // in der Firmware in main.cpp definiert
#endif

// ---------------------------------------------------------------------------
// B10.51 (2026-10-25): FAHRZEUGGEOMETRIE UND KINEMATIK
// ---------------------------------------------------------------------------
// AOG will als Heading die MITTELACHSE des Fahrzeugs. Belegt in AgOpenGPS selbst
// (GPS/Classes/CTool.cs:151): "the hitch translation already starts from the averaged
// vehicle heading (fixHeading)" - AOG korrigiert das EMPFANGENE Heading NICHT, es muss
// also bereits die Mittelachse sein.
//   * Schlepper: Dual-Antenne starr am Koerper -> die Lage IST die gesuchte Richtung.
//   * Knicklenker: Antennen auf dem VORDERWAGEN -> psi_Mittel = psi_Vorderrahmen - gamma/2.
//     Gelenk-Kinematik: sin(gamma/2)/(L/2) statt tan(delta)/L; bei 40 Grad ist das 0,34
//     statt 0,84 * v/L (Faktor 2,4) - die Ackermann-Formel waere dort deutlich falsch.
// vehicleType und wheelbase kommen aus der VWAS-Config (dort landet der vehicle.xml-
// Import), antennaPivot aus dem EEPROM-Bereich der Bridge (wie Baseline und Offset).
static inline const WheelAngleFusion::Config* hbVwasCfg() {
    return wheelAngleFusionPtr ? &wheelAngleFusionPtr->getConfig() : nullptr;
}
static inline bool hbIsArticulated() {
    const WheelAngleFusion::Config* c = hbVwasCfg();
    return (c != nullptr) && (c->vehicleType == 2u);
}
static inline float hbWheelbaseM(float fallbackM) {
    const WheelAngleFusion::Config* c = hbVwasCfg();
    if ((c != nullptr) && (c->wheelbase > 0.5f)) return c->wheelbase;
    return (fallbackM > 0.5f) ? fallbackM : 2.50f;
}
// Modellrate des Rahmens, auf dem die Antenne sitzt (angleDeg = gamma beim Knicklenker).
static inline float hbModelRateDegS(float angleDeg, float vMps, float wheelbaseM) {
    if (wheelbaseM < 0.4f) return 0.0f;
    if (hbIsArticulated()) {
        return 57.29577951308232f * vMps *
               sinf(0.5f * angleDeg * 0.017453292519943295f) / (0.5f * wheelbaseM);
    }
    return 57.29577951308232f * vMps *
           tanf(angleDeg * 0.017453292519943295f) / wheelbaseM;
}
// Vorlauf des Antennenpunkts gegen die Koerperrichtung, in Grad (0 wenn nicht gesetzt).
static inline float hbAntennaLeadDeg(float yawRateDegS, float speedMps) {
    // Ein eingetragener Wert gewinnt; steht im EEPROM nichts, gilt der gemessene
    // Maschinenwert aus dem Build (dieselbe Regel wie physicalBaselineDefaultM
    // in init()). Frueher stand hier bei "nicht gesetzt" eine 0 - damit war die
    // Korrektur stillschweigend AUS, und niemand konnte im Log sehen, dass eine
    // Maschinenkonstante fehlte statt dass die Korrektur nicht gebraucht wurde.
    const float d = configManager.isBridgeAntennaPivotValid()
                        ? configManager.getBridgeAntennaPivotM()
                        : HeadingBridge::Config::antennaPivotDefaultM;
    // B10.70 (2026-11-01): Der Wert ist ab dem DREHPUNKT gemessen - Hinterachse beim
    // Schlepper (260V-Betrieb: "Abstand zur Hinterachse"), GELENK beim Knicklenker.
    // Feldbeleg 01.11. am AOG-Hauptbildschirm (Betreiber): mit 0,91 m steht die Antenne
    // 0,45 m vor der Vorderachse - und die Vorderachse liegt in dieser Ansicht bei
    // 0,46 m, also einer halben Radstandlaenge. Mit 0,45 m steht die Antenne AUF der
    // Vorderachse. AOGs Hauptbildschirm rechnet also ab dem Gelenk, und genau diesen
    // Hebel braucht auch die Geometrie hier (Antennenbahn -> Koerpermitte).
    // AOGs KONFIG-Zeichnung ist an dieser Stelle irrefuehrend: sie legt den Nullpunkt
    // auf y = 0 (Hinterachse) und zeichnet die Vorderachse bei y = Radstand - WERT VON
    // DORT NICHT UEBERNEHMEN, sondern den, den der Hauptbildschirm braucht.
    // B10.71 (2026-11-01) hat hier versucht, 0,5 * Radstand abzuziehen. VERWORFEN:
    // damit waere der Hebel 0,45 statt 0,91, also wieder auf der Vorderachse.
    if ((fabsf(d) < 0.01f) || (speedMps < 0.3f)) return 0.0f;
    return 57.29577951308232f *
           atanf(yawRateDegS * 0.017453292519943295f * d / speedMps);
}
// Knickwinkel-Korrektur auf die Mittelachse (0 beim Schlepper).
static inline float hbArticulationHalfDeg(float jointAngleDeg) {
    return hbIsArticulated() ? (0.5f * jointAngleDeg) : 0.0f;
}

// L1 (2026-11-06): Obergrenze fuer die gemessene Lagrate, die im HALTEN weitergefuehrt
// wird. 250 Grad/s liegt ueber allem, was eine Landmaschine physikalisch dreht (der
// Testmaeher schafft bei 1 m/s und R_min 0,6 m rund 95 Grad/s) und unter dem, was ein
// Falter der Dual-Lage erzeugt - Sprünge faengt ohnehin das kinematische Tor (attJumped).
static constexpr float HB_ATT_RATE_MAX_DEG_S = 250.0f;

// ---------------------------------------------------------------------------
// B10.53 (2026-10-25): DIE SEHNENLAENGE FOLGT DER GESCHWINDIGKEIT
// ---------------------------------------------------------------------------
// Die Sehne ist eine DISTANZ, ihre Qualitaet haengt aber an zwei Dingen, die beide von v
// abhaengen: das Richtungsrauschen ist 2 cm / B (wird mit B besser), der Nachlauf ist
// 0,5*omega*(B/v) (wird mit B schlechter; die Lag-Korrektur nimmt davon nur den
// systematischen Anteil bei konstanter Drehrate heraus). B = 3*v entspricht der Zeitbasis,
// die vorher die Altersgrenze erzwungen hat - jetzt ohne versteckte Tempogrenze.
//   * unten 0,5 m: 2 cm / 0,5 m = 2,3 Grad Rauschen; darunter ist keine Richtung mehr da
//   * oben config.pathChordMinM (2,0 m): mehr bringt bei diesen Geschwindigkeiten nichts
// Gemessen: B = 1 m gewinnt gegen die Modelldrift bis ~0,33 m/s (8x besser), B = 0,5 m
// bis ~0,17 m/s (4x besser) - die Sehne ist also bis unter 1 km/h die bessere Quelle.
static inline float hbChordMinM(float configuredMaxM, float vMps) {
    // B10.53-ERGEBNIS: B(v) und die 0,2-m/s-Schwelle wurden GEMESSEN und VERWORFEN.
    // Bei Schleichfahrt mit schlechtem Empfang liegen Sehne und Witness 11,6 Grad (p50)
    // auseinander - die POSITIONEN sind dort nicht besser als das Modell (B10.16 hatte recht).
    // Die Verfuegbarkeit stieg auf 74 %, die Qualitaet fiel (skew p90 4,3 -> 19,1, Schritt 70 -> 158).
    float b = configuredMaxM;
    if (b < 1.0f) b = 1.0f;
    if (b > configuredMaxM) b = configuredMaxM;
    return b;
}
// Frische des NEUESTEN Ringfixes - DAS ist die echte Veraltungspruefung (kein Empfang oder
// alles verworfen). 300 ms = zwei verpasste 10-Hz-Saetze.
static const uint32_t kNewestFixMaxAgeMs = 300u;
// Eigene Untergrenze fuer die SEHNE (der Witness behaelt seine 1,0-m/s-Schwelle, B10.16):
// sonst gibt es bei Schleichfahrt wieder gar keinen Kurs.
static const float kChordMinSpeedMps = 0.2f;

// B10.52 (2026-10-25): Diagnose fuer das Log - DATEI-Statics, bewusst NICHT in Status/Config
// (ein Feld dort hat im Sim die Config-Defaults zerstoert, s. B10.50).
//   s_pcRej: warum in diesem Frame KEINE Sehne veroeffentlicht wurde
//            0 ok, 1 aus, 2 keine Position/fixQ, 3 hdop, 4 v < travelWitnessSpeedMps,
//            5 rueckwaerts, 6 Fix-Sprung, 7 kein Ring-Sample, 8 Sprungtor
// Ohne diese Spalte war am 10-25 nicht entscheidbar, ob Geschwindigkeit, Ring oder Tor die
// Sehne gesperrt hat - der teuerste Blindflug der Sitzung.
static uint8_t s_pcRej     = 0;
static float   s_pcDevDeg  = 0.0f;   // Abweichung, die das Sprungtor gemessen hat
static float   s_pcDevLim  = 0.0f;   // ... und seine Grenze
static float   s_antLeadDeg = 0.0f;  // angewandter Antennen-Vorlauf (Stufe 1)
static float   s_artHalfDeg = 0.0f;  // angewandte Knick-Korrektur gamma/2 (Stufe 2)
// 2026-11-06: gefilterter Knickwinkel aus (Kurs - Lage-Offset), dateiweit - die Getter fuer die
// PGN-253-Ausgabe lesen ihn. AOG knickt die Maschine damit (CVehicle.cs: 0.5 * Winkel).
static float   s_artEstDeg   = 0.0f;
static bool    s_artEstValid = false;
float HeadingBridge::getArticulationEstimateDeg() const { return s_artEstDeg; }
bool  HeadingBridge::hasArticulationEstimate()    const { return s_artEstValid; }
// B10.54 (2026-10-25): ROHE Modellrate (vor dem P1b-Gate). Spalten-Audit: NICHT redundant -
// 'yr' ist die GEGATETE Rate (0, wenn unterdrueckt), 'opp_yr' deckt nur den Fall "gegen die
// Lage" ab, und die Formel braucht config.steerScale UND sideHillGainDegPerDeg (0,12 mit
// Vorzeichenumkehr) - beides steht nicht im Log. Genau diese Spalte fehlte bei der Frage
// "wie schnell dreht das Modell wirklich?" (Halten-Drift).
static float   s_rateRawDegS = 0.0f;
// B10.66 (2026-11-01, Betreiber): NOTBRUECKE KNICKLENKER OHNE GELENKSENSOR.
// Ein Knicklenker ohne WAS und ohne Encoder hat KEINEN Winkel: der VWAS ist
// encoder-gefuehrt (WheelAngleFusion.cpp `fusedAngle = encoderAngle - encoderOffset`) und
// meldet dann konstant -encoderOffset. Feldmessung 01.11. (Stiga Park, 264 Zeilen):
// enc_cnt = 0 in ALLEN Zeilen, wheel_deg = -0.36 Grad = -enc_off, waehrend der Maeher in
// den Achten real +-45 Grad knickte (aus der Bahnkruemmung gerechnet).
// Der VWAS RECHNET diesen Winkel aber bereits - aus der Kruemmung und ohne Sensor
// (`gamma = 2*asin(kappa*L/2)`, WheelAngleFusion.cpp), abgelegt in
// wheelObs.gpsAngleDeg (Logspalte gps_a). Genau dieser Wert fehlte der Bridge.
// Bedingung: artic + encRaw == 0. encRaw ist ein ACCUMULATOR (nur Impulse aendern ihn) ->
// auf einer Maschine MIT Encoder bewegt er sich beim ersten Lenken, und die Bruecke
// schaltet sich dort von selbst ab. Sie ist eine Notbruecke, kein Ersatz.
static bool    s_artAngleFromGps = false;        // Bruecke gerade aktiv?
static bool    s_artAngleFromGpsLogged = false;  // einmal je Boot melden
// B10.58 (2026-10-25): RESIDUUM der Sehnenausgleichung - wie gut passen die Fixes im Ring
// ueberhaupt noch zum Modell "konstante Kruemmung + Endpunktfehler" (B10.53)?
// Audit B10.57 (welche Spalten es schon gibt, bevor diese gebaut wurde):
//   * pos_sigma_m   - Selbstauskunft des Empfaengers, warnt in 6-57/6-52/7-01 NIE (<5 cm);
//                     in 66-100 % der schlechten Frames meldet er 2 cm.
//   * proj_ctk_m    - deckt bei v >= 1,5 m/s 75-100 % ab, im STAND strukturell 0 % (die
//                     Querspur braucht ein Projektionsfenster mit Bewegung).
//   * hdop / baseline_dev_pct - fangen nur einen Teil (7-01 65 %, 6-57/6-52 bei Schleichern 0 %).
//   * pc_dev        - KEIN Ersatz: Abweichung EINER Sehne gegen die letzte uebernommene,
//                     wird nur gesetzt, WENN das Sprungtor ausloest (s. 4128).
// Nutzen also genau dort, wo sonst nichts ist: Schleichfahrt/Mitteltempo (0,1...1,5 m/s).
// Grenzen: braucht 6 m Fahrt (bei 0,5 m/s = 12 s Reaktionszeit), im echten Stand gibt es
// keine Sehne -> Spalte bleibt stehen. Bewusst NUR Diagnose: kein Gate, keine Ausgabe.
static float   s_crResidDeg = 0.0f;
static uint16_t s_crLastPosCount16 = 0xFFFF;   // letzter Fix, fuer den gerechnet wurde

// B10.54: Zugriff fuer den Simulator (damit beide dieselben Spalten schreiben).
HeadingBridge::LogExtra HeadingBridge::logExtra() {
    HeadingBridge::LogExtra e;
    e.pcRej   = s_pcRej;
    e.pcDev   = s_pcDevDeg;
    e.antLead = s_antLeadDeg;
    e.artHalf = s_artHalfDeg;
    e.vehType = (uint8_t)(hbIsArticulated() ? 2u : 0u);
    e.yrRaw   = s_rateRawDegS;
    e.crResid = s_crResidDeg;
    return e;
}
// ---------------------------------------------------------------------------
// Update flow (called at 100 Hz from AutosteerProcessor::process)
// ---------------------------------------------------------------------------
//   1. New HPR sentence?  (arrival counted in GNSSProcessor)  -> gate it.
//        trusted  -> measurement into the filter, roll input, LIVE
//        rejected -> reason recorded, no measurement
//   2. Prediction            psi += v * tan(delta + bias) / L_eff * dt
//   3. Measurements that keep working while the attitude is gone:
//        COG (VTG, age checked)          -> rCogDeg
//        fix-to-fix (>= 0.5 m travel)    -> rFixToFixDeg
//   4. Roll channel          LPF when live, hold/fade when not
//   5. State machine         BRIDGE -> COAST -> TIMEOUT, with logging
//
// The prediction runs at the FULL 100 Hz rate on purpose: it is the only part
// that keeps the estimate moving between the 10 Hz measurements, and it is what
// makes the return to LIVE smooth (the filter is already on the new heading when
// the next valid frame arrives - no step, no learning transient).

// B10.61 (2026-10-26): harte Obergrenze der AUSGABE-Rate (Notbremse, siehe update()).
// Bewusst groesser als jede physikalische Drehrate des Fahrzeugs, damit echte Wenden
// nicht gebremst werden - sie faengt nur Artefakte wie die gemessenen 331 Grad/s.
static constexpr float kOutHardRateDegS = 200.0f;   // 2026-11-06: war 60. Notbremse gegen
// Artefakte (gemessen 331 Grad/s), NICHT die Formung - die macht die Physik-Grenze. 60 Grad/s
// konnte eine 75-Grad-Wende eines Knicklenkers (bis ~150 Grad/s) nicht durchlassen; 200 liegt
// ueber jeder realen Drehrate und faengt weiterhin Spruenge.

static inline float wrap180(float a) {
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static inline float wrap360(float a) {
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f)    a += 360.0f;
    return a;
}

// Metres between two WGS84 points, returned as north/east components. A local
// equirectangular projection is plenty over the few metres this class looks at.
static inline void localDeltaMeters(double lat1, double lon1, double lat2, double lon2,
                                    double& dN, double& dE) {
    const double mPerDegLat = 111320.0;
    const double mPerDegLon = 111320.0 * cos(lat1 * 0.017453292519943295);
    dN = (lat2 - lat1) * mPerDegLat;
    dE = (lon2 - lon1) * mPerDegLon;
}

// ---------------------------------------------------------------------------
// B10.58 (2026-10-25): Residuum der Sehnenausgleichung - EINMAL JE FIX (10 Hz).
// Modell (B10.53): theta_b = psi + 0,5*kappa*b + delta/b  ueber b = 1/2/3/4/6 m mit
// GEMEINSAMEM Endpunkt (dem neuesten Fix). Drei Unbekannte, fuenf Gleichungen; die
// Gewichte sind w = b^2 (Richtungsrauschen 2 cm / b), also ist die Pseudoinverse eine
// KONSTANTE. Erzeugt und geprueft von tools/analysis/chord_fit3.ps1 (-ShowM / -SelfTest,
// dort werden kappa = 0,02000 1/m und delta = 0,07000 m exakt zurueckgewonnen).
// Das Residuum sagt, wie gut die Fixes im Ring noch zu diesem Modell passen - es ist
// damit eine Positionsqualitaet, die der Empfaenger NICHT selbst meldet (B10.57).
// Reine Diagnose: kein Gate, keine Ausgabe, keine Statusfelder.
static void hbUpdateChordResidual(uint32_t now) {
    static const float kMB[5]   = { 1.0f, 2.0f, 3.0f, 4.0f, 6.0f };
    static const float kMPsi[5] = { -0.013970f, 0.001902f, 0.027414f, 0.042361f, -0.040253f };
    static const float kMKap[5] = { 0.003213f, -0.001337f, -0.007943f, -0.010899f, 0.016965f };
    static const float kMDel[5] = { 0.026990f, 0.009716f, -0.024830f, -0.049661f, 0.037785f };

    const PositionHistory::FixSample* end = positionHistory.newest();
    if (end == nullptr || end->posCount16 == s_crLastPosCount16) return;   // nur bei NEUEM Fix
    s_crLastPosCount16 = end->posCount16;
    if (positionHistory.count() < 20u) { s_crResidDeg = 0.0f; return; }

    // EIN Durchlauf durch den Ring, dabei die Bogenlaenge aufsummieren und die Position auf
    // GENAU b Meter interpolieren. Weder lookback() (Fenster [b, 1,7*b] = bis 70 % daneben)
    // noch das naechstgelegene Sample taugen: das Modell nimmt exakt b an, und der 1/b-Term
    // verstaerkt jeden Distanzfehler der kurzen Sehne (gemessen 10-25: p90 9,5 bzw. 6,4 Grad
    // gegen 0,38 Grad im geprueften Werkzeug).
    float th[5];
    bool  have[5] = { false, false, false, false, false };
    const int32_t cnt = (int32_t)positionHistory.count();
    float cum = 0.0f;
    for (int32_t back = 1; back < cnt; back++) {
        const PositionHistory::FixSample* a = positionHistory.at((uint16_t)back);       // aelter
        const PositionHistory::FixSample* b = positionHistory.at((uint16_t)(back - 1)); // neuer
        if (a == nullptr || b == nullptr) break;
        const uint16_t need = (PositionHistory::F_VALID | PositionHistory::F_POS_OK);
        if ((a->flags & need) != need || (b->flags & need) != need) break;
        if ((uint32_t)(now - a->tMs) > 20000u) break;   // 20 s: 6 m sind bei 0,5 m/s ~12 s
        const float seg = (float)PositionHistory::distanceM(*a, *b);
        const float cumPrev = cum;
        cum += seg;
        bool all = true;
        for (uint8_t k = 0; k < 5u; k++) {
            if (have[k]) continue;
            if (cum < kMB[k]) { all = false; continue; }
            float t = (seg > 1e-4f) ? ((kMB[k] - cumPrev) / seg) : 1.0f;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            const double lat = a->lat + (double)t * (b->lat - a->lat);
            const double lon = a->lon + (double)t * (b->lon - a->lon);
            double dN = 0.0, dE = 0.0;
            localDeltaMeters(lat, lon, end->lat, end->lon, dN, dE);
            th[k] = (float)(atan2(dE, dN) * 57.29577951308232);
            have[k] = true;
        }
        if (all) break;
    }
    for (uint8_t k = 0; k < 5u; k++) {
        if (!have[k]) { s_crResidDeg = 0.0f; return; }   // noch nicht genug Fahrt
    }
    // Auf die LAENGSTE Sehne beziehen: dann liegen alle fuenf innerhalb weniger Grad und
    // der Satz kann nicht ueber die 180-Grad-Naht fallen.
    const float ref = th[4];
    for (uint8_t k = 0; k < 5u; k++) th[k] = wrap180(th[k] - ref);

    float psi = 0.0f, kap = 0.0f, del = 0.0f;
    for (uint8_t k = 0; k < 5u; k++) {
        psi += kMPsi[k] * th[k];
        kap += kMKap[k] * th[k];
        del += kMDel[k] * th[k];
    }
    float ss = 0.0f;
    for (uint8_t k = 0; k < 5u; k++) {
        const float pred = (psi + 0.5f * kap * kMB[k] + del / kMB[k]) * 57.29577951308232f;
        const float r    = th[k] - pred;
        ss += r * r;
    }
    s_crResidDeg = sqrtf(ss * 0.2f);      // rms ueber die fuenf Sehnen, in Grad
}

void FLASHMEM HeadingBridge::init() {
    // The global IMU instance is declared in IMUProcessor.h (extern), no local
    // declaration needed here.
    lastHprCount = gnssProcessor.getHprCount();

    status = Status();
    haveTrusted = false;
    lastTrustedMs = 0;
    lastTrustedVehHeading = 0.0f;
    lastRoll = 0.0f;
    haveLastRoll = false;
    lastFrameMs = 0;
    consecutiveRejects = 0;

    psi = 0.0f;
    steerBiasDeg = 0.0f;
    // Initial covariance: the heading is unknown (360 deg^2 is far too
    // optimistic, but the first trusted frame or COG collapses it immediately),
    // the bias is assumed unknown within +-5 deg.
    p11 = 360.0f;
    p12 = 0.0f;
    p22 = 25.0f;

    // ---- Antenna spacing, FIRST, so the summary line below reports the real value -----
    // If it was ever entered, it is the reference of the baseline-length test from the FIRST
    // frame on and the learning path is skipped entirely (see the baseline block in
    // evaluateAttitudeFrame). This is the fix for the 2026-10-03 failure: the learned
    // reference had been frozen at 0.852 m (+18.3 % of the true 0.711 m) before the HR log
    // even started, and the gate then rejected 99.7 % of the frames the receiver itself
    // called clean - for the whole session, with no revision path.
    // Runs BEFORE the "HeadingBridge ENABLED: ... base=%.3fm" line on purpose: applying it
    // further down made that line report base=0.000m while the effective reference was
    // already 0.711 m, and that is the number a field test looks at first.
    {
        extern ConfigManager configManager;
        if (configManager.isBridgePhysicalBaselineValid()) {
            const float storedBase = configManager.getBridgePhysicalBaselineM();
            config.physicalBaselineM = storedBase;
            if (storedBase > 0.1f) {
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge physicalBaselineM from EEPROM: %.3f m - the baseline "
                           "test uses it as a FIXED reference (no learning)",
                           (double)storedBase);
            } else {
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge physicalBaselineM from EEPROM: 0 (switched OFF) - "
                           "the reference is LEARNED from the receiver");
            }
        } else {
            // Nothing entered: use the machine constant the build ships with, so the fix is
            // active without anyone having to configure it. The safety net in
            // evaluateAttitudeFrame() is what makes that safe on a board moved to another
            // vehicle with a different antenna spacing - it switches the test off instead of
            // silencing the heading.
            config.physicalBaselineM = Config::physicalBaselineDefaultM;
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge physicalBaselineM: not in the EEPROM, using the built-in "
                       "default %.3f m (the baseline test reference is fixed)",
                       (double)Config::physicalBaselineDefaultM);
        }

        // Dual-antenna heading offset: same rule - a stored value wins, otherwise the build
        // default. Also runs BEFORE the summary line, so "dualHdgOff=%.1fdeg" below reports
        // the value that is actually in use (a field test reads that number first).
        if (configManager.isBridgeDualHeadingOffsetValid()) {
            const float storedOff = configManager.getBridgeDualHeadingOffsetDeg();
            config.dualHeadingOffsetDeg = storedOff;
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge dualHeadingOffsetDeg from EEPROM: %+.2f deg - enter the "
                       "SAME value in AgOpenGPS", (double)storedOff);
        } else {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge dualHeadingOffsetDeg: not in the EEPROM, using the "
                       "built-in default %+.2f deg (measured) - enter the SAME value in "
                       "AgOpenGPS", (double)config.dualHeadingOffsetDeg);
        }
    }

    haveRef = false;
    refArcM = 0.0f;
    haveLastPos = false;
    lastWindowFixCount = 0;
    lastProjFixCount = 0;
    // Direction of travel: unknown at boot, the first comparison seeds the filter.
    dirDevEma = 0.0f;
    dirDevInit = false;
    status.reverseDirection = false;
    status.dirChanging = false;
    status.dirDevDeg = 0.0f;
    status.dirTravelDeg = 0.0f;
    status.dirRefDistM = 0.0f;
    status.dirSigmaM = 0.0f;
    status.mountOffsetSignOk = true;
    // Direction of travel state (distance based, see updateDirectionState()).
    dirRefValid = false;
    dirRefMs = 0;
    revVotes = 0;
    scaleStartRev = false;
    pathUsableLatch = false;
    pathUsableFalseMs = 0;
    prevTrustedLatch = false;
    reentryUntilMs = 0;
    settleHaveLast = false;
    settleGoodSinceMs = 0;
    status.reentrySettling = false;
    status.reentrySettlingMs = 0;
    status.posFallbackActive = false;
    status.posFallbackMs = 0;
    status.posFallbackM = 0.0f;
    status.posFallbackUpdates = 0;
    lastProjFixMs = 0;
    posFallbackStartMs = 0;
    posFallbackStartOdoM = 0.0f;
    posRtkGoodMs = 0;
    posRampStartMs = 0;
    status.posRampActive = false;
    status.outSmoothing = false;
    status.outSmoothDeg = 0.0f;
    outSmoothInit = false;
    outResidDeg = 0.0f;
    attMeasPeriodS = -1.0f;
    for (uint8_t k = 0; k < 4; k++) lastMeasMsByKind[k] = 0;
    outPendSign = 0;
    outPendMag = 0.0f;
    outPendCount = 0;
    outHoldSinceMs = 0;
    attMeasRateDegS = 0.0f;
    attMeasRateValid = false;
    psiJumpedThisTick = false;
    yawBiasDegS = 0.0f;
    psiPrevForBias = 0.0f;
    psiPrevValid = false;
    yawBiasWarned = false;
    status.imuRateDegS = 0.0f;
    status.imuYawBiasDegS = 0.0f;
    status.imuRateUsed = false;
    imuHdgOffsetDeg = 0.0f;
    imuHdgOffsetInit = false;
    imuHdgHaveFresh = false;
    status.imuHeadingDeg = 0.0f;
    status.imuHdgOffsetDeg = 0.0f;
    status.imuHeadingUsed = false;
    status.imuHeadingAgeMs = 0;
    loggedWheelbaseM = config.wheelbaseM;   // init() prints it; update() reports changes
    // Steering-scale collection window restarts on a re-init.
    scaleCollecting = false;
    scaleStartPsi = 0.0f;
    scaleStartRawDeg = 0.0f;
    scaleStartMs = 0;
    scaleAccX = 0.0f;
    scaleAccXX = 0.0;
    scaleY1 = 0.0;
    // Model-plausibility gate: its window restarts too, so a re-init does not compare
    // a heading from before the gap with a model sum from after it.
    gateAccX = 0.0f;
    gateAccQ = 0.0f;
    gateStartMs = 0;
    gateStartRawDeg = 0.0f;
    modelGateUntilMs = 0;

    mountOffsetSin = 0.0f;
    mountOffsetCos = 1.0f;
    lastMeasurementMs = millis();
    lastMeasurementOdometerM = 0.0f;
    frameIntervalEmaMs = 100.0f;      // nominal until the first two frames
    gapThresholdMs = 250;

    rollF = 0.0f;
    rollInit = false;
    rollLossMs = 0;

    inEvent = false;
    odometerM = 0.0f;
    timeoutLogged = false;
    lastLogMs = 0;

    LOG_NOTICE(EventSource::GNSS,
               "HeadingBridge %s: L=%.2fm scale=%.2f dualHdgOff=%+.1fdeg (AgOpenGPS value) "
               "bridge=%lums/%.0fm coast=%lums/%.0fm dead=%lums/%.0fm gradev=%.0f%% "
               "base=%.3fm",
               config.enabled ? "ENABLED" : "measuring only",
               (double)config.wheelbaseM, (double)config.steerScale,
               (double)config.dualHeadingOffsetDeg,
               (unsigned long)config.bridgeMaxMs, (double)config.bridgeMaxM,
               (unsigned long)config.coastMaxMs, (double)config.coastMaxM,
               (unsigned long)config.deadReckonMaxMs, (double)config.deadReckonMaxM,
               (double)config.baselineTolPct,
               (double)config.physicalBaselineM);

    // Second line: the heading-fusion configuration. Operator request 2026-09-29 - a log
    // file must document the SETUP by itself (single vs dual antenna cannot be seen later,
    // and the IMU switches decide which fast/slow signals exist). The IMU type itself is
    // printed by IMUProcessor::initialize() ("IMU Type: ...").
    LOG_NOTICE(EventSource::GNSS,
               "HeadingBridge config: useImuHeading=%d useImuRate=%d imuRateInvert=%d "
               "estimateSideSlip=%d imuAnchor=%lums/%.1fmps travelWitness=%.1fm/%.0fdeg "
               "reseedBeyond=%.0fdeg revReconfirm=%lums",
               config.useImuHeading ? 1 : 0, config.useImuRate ? 1 : 0,
               config.imuRateInvert ? 1 : 0, config.estimateSideSlip ? 1 : 0,
               (unsigned long)config.imuAnchorHoldMs, (double)config.imuAnchorMinSpeedMps,
               (double)config.travelWitnessMinM, (double)config.travelWitnessMaxTurnDeg,
               (double)config.reseedBeyondDeg, (unsigned long)config.revReconfirmMs);

    if (config.logRawFrames) {
#if defined(AIO_HR_LOG) && ((AIO_HR_LOG == 0) || (AIO_HR_LOG == 2))
        // B10.49 (2026-10-19, Betreiber): Log-Kopf NICHT ausgeben (Schalter in platformio.ini).
#else
        // CSV header for the 10 Hz raw frame log, so the dump is self-describing.
        Serial.print(F("HR_HDR,t_ms,lat,lon,v_mps,cog_deg,att_hdg_deg,roll_deg,qf,sats,fixQ,"
                       "wheel_deg,psi_deg,sigma_deg,bias_deg,src,state,f2f_deg,proj_ctk_m,"
                       "proj_rng_m,proj_along_m,proj_r_deg,mount_deg,sc,slip_deg,baseline_m,"
                       "baseline_dev_pct,rej,pf,gap_ms,vscale,"
                       "proj_upd,fix_cnt,pos_age_ms,sc_n,sc_spread,vsc_n,"
                       "rev,dirchg,dirdev,dir_ref_m,pos_sigma_m,pfb,pfb_m,imurate,yawbias,imuhdg,imuhdgoff,"
                       // The BRIDGED roll (status.rollDeg), appended 2026-09-24. roll_deg
                       // above is the RAW dual-antenna value; this one is what the VWAS
                       // actually consumes (WheelAngleFusion takes rollDeg from the
                       // bridge), so the log has to carry both to be able to tell a raw
                       // attitude fault from a roll that only the bridge could supply.
                       "roll_bridge_deg,"
                       // az_out: the heading that actually leaves the module (offset
                       // applied and output-smoothed) - added 2026-09-27 so the output
                       // smoothing can be checked against the raw attitude in the log.
                       "az_out,"
                       // Steering chain, added 2026-09-27: the angle AgOpenGPS asks for,
                       // the PWM that goes out and whether guidance is engaged. This is
                       // what shows whether a flickering heading really reaches the
                       // steering (operator report "wild steering").
                       // Appended 2026-09-29 (columns 60...69) after the operator asked for
                       // the fields that answer "which measurement moved the heading, and
                       // did a position jump reach AgOpenGPS": psi_att (pass-through
                       // check), innov/gain/meas (the filter update), travel (the
                       // direction the decision used), raw_step/pub_step (receiver vs
                       // published position step), publim (projection published),
                       // hdg_res/dir_veto (the 2026-09-29 guards, counters).
                       "aog_tgt_deg,pwm,guid,"
                       // MISSING NAMES FIXED 2026-09-29: sc_b (the wheel angle offset the
                       // model applies), mdev (model gate deviation), mgate (the gate holds
                       // R elevated, 0/1), yr (the yaw rate the MODEL used), wq (wheel
                       // angle reliability 0...100, -1 = not reported) and hdop were added
                       // on 2026-09-27/28 but never appeared in this header - the data
                       // line and any tool read them by position, the printed header
                       // drifted. 80 values + "HR" = 81 columns.
                       "sc_b,mdev,mgate,yr,wq,hdop,"
                       "psi_att,innov,gain,meas,travel,raw_step,pub_step,publim,hdg_res,dir_veto,"
                       // Appended 2026-09-29 with the single-antenna support (operator
                       // request): psi_imu (psi - offset-corrected IMU heading: the same
                       // pass-through check as psi_att, for the IMU), imu_flags (bit0
                       // ok/fresh, bit1 heading fed the filter, bit2 rate drives the model,
                       // bit3 rate NOT supported by this IMU type), imu_age_ms, imu_q,
                       // imu_hdg_raw (the raw IMU heading - this is what AOG receives in
                       // PANDA), imu_roll, vwas_hsrc (0 bridge / 1 dual / 2 COG - what the
                       // VWAS used), roll_src (0 none/1 dual/2 IMU/3 held), cog_ok (VTG
                       // course passed its gate), travel_wit (direction of travel from the
                       // POSITIONS: the direction-decision witness), nav (0 none/1 PANDA/
                       // 2 PAOGI) and gps_flags (bit0 dual, bit1 INS, bit2 GPS, bit3 fix).
                       //
                       // Appended 2026-09-30 (columns 82/83, operator request "P1"):
                       // roll_susp = 1 when THIS frame was accepted although |roll| was
                       //   outside the envelope, because the VTG course confirmed the
                       //   heading (the decoupled roll gate). The roll output then holds,
                       //   which is visible as roll_src = 2/3 in the same frame.
                       // yaw_zeroed = 1 when the kinematic yaw rate was NOT used because
                       //   the VWAS wheel-angle quality is below wheelQualityLowPct - the
                       //   signature of "the model invented rotation" (check together with
                       //   yr, which then reads 0, and wq).
                       //
                       // Appended 2026-09-30 (columns 84...87, P2):
                       // rl_allowed = the rate bound the measurement correction was clipped
                       //   to this tick (deg/s; kinematics + margin, or the path bound),
                       // rl_clip    = 1 when it HAD to clip (the limiter fired),
                       // rl_path    = the part of the bound that came from an accepted
                       //   fix-to-fix turn (0 = the wheel angle alone set the bound),
                       // meas_r     = the measurement noise R actually used by the applied
                       //   measurement in deg - shows the COG 1/v damping (cogRefSpeedMps)
                       //   and the adaptive estimate at work.
                       "psi_imu,imu_flags,imu_age_ms,imu_q,imu_hdg_raw,imu_roll,vwas_hsrc,"
                       "roll_src,cog_ok,travel_wit,nav,gps_flags,roll_susp,yaw_zeroed,"
                       "rl_allowed,rl_clip,rl_path,meas_r,uni_len,uni_hdg,uni_pitch,uni_hdgsd,uni_ptchsd,"
                       // Appended 2026-10-02 ("Stufe 2"): the per-epoch attitude quality the
                       // OUTPUT gate decided on and what the smoothing did with it.
                       //   att_q    = 1 clean epoch (snap allowed, no delay), 0 degraded
                       //              (snap suppressed, the residual is rate-limited),
                       //              -1 no per-epoch figure from this receiver (F9P, or
                       //              UNIHEADINGA not enabled) -> the gate is OPEN, i.e. the
                       //              output path behaves exactly as before this gate.
                       //   out_snap = 0 the residual had to be RATE-LIMITED (a jump was
                       //              suppressed), 1 it was SNAPPED (|resid| > outSnapDeg on
                       //              a clean epoch), 2 normal smoothing step. Together with
                       //              az_out this shows whether a receiver artifact reached
                       //              AgOpenGPS or was turned into a bounded ramp.
                       "att_q,out_snap,"
                       // Appended 2026-10-02 (R6): the OUTPUT stage's own state - at the
                       // EXISTING 10 Hz, no extra logging rate. The filter is deterministic,
                       // so what was missing is not more samples but the state it decided on:
                       //   out_sm  = the smoothed heading BEFORE the rate lead, so
                       //              lead = az_out - out_sm separates the two contributions
                       //              (that split was the open question of 2026-10-02),
                       //   out_res = the residual the smoothing is closing this frame,
                       //   out_lim = the rate bound the output used (deg/s),
                       //   netclip = 1 when the R1 net clamp on psi had to clip this tick.
                       "out_sm,out_res,out_lim,netclip,"
                       // opp_yr (R8): the model rate that was SUPPRESSED because it pointed
                       // against a trusted attitude (deg/s, 0 while the model runs normally).
                       // In a K-turn this is the number that was fighting the measurement.
                       "opp_yr,"
                       // Appended 2026-10-02 (W1/W2): the POSITION that actually goes to
                       // AgOpenGPS. Until now the raw log only ever carried the receiver's
                       // latitude/longitude, and the drawn track is neither of them when the
                       // bridge substitutes its projection - so a kink in AOG's track could
                       // not be reconstructed from any log.
                       //   pub_lat/pub_lon = the PUBLISHED position (projection while
                       //       publim=1, otherwise the receiver's own) - 0.0000 until the
                       //       first point was published,
                       //   nmea_dif = |latitudeNMEA - latitude| in m: what PAOGI transmits
                       //       versus what this log shows. -1 = nothing cached yet.
                       // --- columns 103...115 (2026-10-11): VWAS-Beobachter -----------------
                       // Alle Werte kommen aus WheelAngleFusion (siehe Status::WheelObs) und
                       // sind die Grundlage fuer (a) den Sim-Replay einer echten Fahrt
                       // (enc_cnt ist der VWAS-EINGANG!), (b) den Trim (zo_res) und (c) die
                       // attitude-freie Kreuzprobe (gps_a = eigene Kruemmung). Absichtlich
                       // ALLE auf einmal, damit ein Log kuenftig die ganze Kette traegt.
                       // Siehe Register B7.15...B7.22.
                       "pub_lat,pub_lon,nmea_dif,"
                       "enc_cnt,enc_deg,enc_off,zo_res,zo_st,zo_fit,zo_gain,zo_roll,zo_rate,"
                       "gps_a,aog_zero,aog_wasoff,cpd_aog,"
                       // vwas_trim: der Trim, den das MODELL anwendet (B7.21) - eigene
                       // Spalte, damit der Effekt messbar ist; sc_b bleibt der Fehlerindikator.
                       "vwas_trim,path_chord,pc_ok,pc_lag,az_src,pc_len,pc_rej,pc_dev,ant_lead,art_half,veh_type,yr_raw,cr_resid\r\n"));
#endif
    }

    // Steering scale: take the value learned on this machine from the EEPROM if it
    // was ever learned. 1.0 means "trust the VWAS/AOG scale" and is the start value
    // until the estimator has collected enough curved, well-received driving.
    {
        extern ConfigManager configManager;
        if (configManager.isBridgeSteerScaleValid()) {
            const float stored = configManager.getBridgeSteerScale();
            if (stored > config.steerScaleMin && stored < config.steerScaleLimit) {
                config.steerScale = stored;
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge steerScale from EEPROM: %.3f", (double)stored);
            }
        }
    }

    // Antenna spacing: if it was ever entered, it is the reference of the baseline-length
    // test from the FIRST frame on and the learning path is skipped entirely
    // (HeadingBridge.cpp:582). This is the fix for the 2026-10-03 failure: the learned
    // reference had been frozen at 0.852 m (+18.3 % of the true 0.711 m) before the HR log
    // even started, and the gate then rejected 99.7 % of the frames the receiver itself
    // called clean - for the whole session, with no revision path.
    // MOVED to the top of init() on 2026-10-03 (evening): the "HeadingBridge ENABLED: ...
    // base=%.3fm" line is printed near the start of init(), so applying the value down here
    // made that line report base=0.000m while the effective reference was already 0.711 m -
    // exactly the number a field test looks at first.
}

// Dual-antenna heading offset: apply at runtime AND persist. This is the SAME number
// AgOpenGPS carries as its "dual heading offset" - the web page tells the operator to enter
// it in both places, and the firmware uses it in both directions (body = azimuth + offset,
// published = psi - offset).
void FLASHMEM HeadingBridge::setDualHeadingOffsetDeg(float deg)
{
    if (!(deg >= -180.0f && deg <= 180.0f)) {
        if (config.logEvents) {
            LOG_WARNING(EventSource::GNSS,
                        "HeadingBridge dualHeadingOffsetDeg %.2f deg out of range "
                        "(-180 ... 180) - NOT stored, keeping %+.2f deg",
                        (double)deg, (double)config.dualHeadingOffsetDeg);
        }
        return;
    }
    config.dualHeadingOffsetDeg = deg;
    {
        extern ConfigManager configManager;
        configManager.setBridgeDualHeadingOffsetDeg(deg, true);
        configManager.saveHeadingBridgeConfig();
    }
    if (config.logEvents) {
        LOG_NOTICE(EventSource::GNSS,
                   "HeadingBridge dualHeadingOffsetDeg set + stored: %+.2f deg - enter the SAME "
                   "value in AgOpenGPS", (double)deg);
    }
}

// Antenna spacing: apply at runtime AND persist, so it survives a reboot. 0 (or anything
// outside 0.1 ... 5 m) clears the field, which returns to learning the reference from the
// receiver. Called by the config/web layer; mirrors applyLearnedSteerScale() below.
void FLASHMEM HeadingBridge::setPhysicalBaselineM(float m)
{
    // 0 is a VALID input: it means "switch the test off and learn the reference again", which
    // is the only way to override the built-in default of the build. Anything else must be a
    // plausible antenna spacing.
    if (!(m == 0.0f || (m >= 0.1f && m <= 5.0f))) {
        if (config.logEvents) {
            LOG_WARNING(EventSource::GNSS,
                        "HeadingBridge physicalBaselineM %.3f m out of range (0 or 0.1 ... 5.0) "
                        "- NOT stored, keeping %.3f m",
                        (double)m, (double)config.physicalBaselineM);
        }
        return;
    }
    config.physicalBaselineM = m;
    // The operator just corrected the reference: the safety net starts over from clean.
    status.baselineTestDisabled = false;
    status.baselineGuardCount = 0;
    status.baselineGuardRejects = 0;
    // ...and so does the ACTIVE reference check, otherwise the new value would never be
    // re-verified within this session (its window only runs once).
    status.baselineSuggestionM = 0.0f;
    status.baselineSuggestCount = 0;
    status.baselineRefSuspect = false;
    {
        extern ConfigManager configManager;
        configManager.setBridgePhysicalBaselineM(m, true);
        configManager.saveHeadingBridgeConfig();
    }
    if (config.logEvents) {
        if (m > 0.1f) {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge physicalBaselineM set + stored: %.3f m - the baseline "
                       "test reference is now FIXED (no learning)", (double)m);
        } else {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge physicalBaselineM set + stored: 0 - the test is OFF and "
                       "the reference is LEARNED from the receiver again");
        }
    }
}

// ---------------------------------------------------------------------------
// Attitude plausibility gate
// ---------------------------------------------------------------------------
// Called only when a NEW HPR sentence has arrived (counted in GNSSProcessor),
// so everything in here is per-frame, not per-loop-tick.
//
// Order matters: the cheap "the receiver said it is not measuring" tests come
// first, then the value tests that catch what QF does NOT catch. 1.8 ... 2.9 %
// of the QF 4/5 frames carry a garbage attitude (measured), so a QF test alone
// would let exactly those through - and they are the ones that reach AOG today.
void FLASHMEM HeadingBridge::evaluateAttitudeFrame(uint32_t now) {
    const auto& d = gnssProcessor.getData();
    status.framesSeen++;

    // --- measured sentence period -> dynamic gap threshold -------------------
    // UM982 can be configured for 20 Hz, the F9P pair typically runs at 10 Hz, and
    // a fixed 250 ms threshold would either flag every second frame at 20 Hz or
    // react 5 frames late at 5 Hz. 3 x the measured period, clamped.
    if (lastFrameMs > 0) {
        const float dt = (float)(now - lastFrameMs);
        if (dt > 1.0f && dt < 2000.0f) {
            frameIntervalEmaMs += 0.2f * (dt - frameIntervalEmaMs);
            float gap = 3.0f * frameIntervalEmaMs;
            if (gap < (float)config.hprGapMinMs) gap = (float)config.hprGapMinMs;
            if (gap > 600.0f) gap = 600.0f;
            gapThresholdMs = (uint32_t)gap;
            status.frameIntervalMs = frameIntervalEmaMs;
        }
    }

    const float roll = d.dualRoll;
    const float azimuth = wrap360(d.dualHeading);
    // Same formula as AgOpenGPS applies to the value we publish, so the body
    // heading here and in AOG are identical for the same offset:
    //     body = az_received + dualHeadingOffsetDeg
    const float vehicleHeading = wrap360(azimuth + config.dualHeadingOffsetDeg);

    RejectReason reason = RejectReason::NONE;

    // 1. The receiver's own statement about the baseline solution.
    //    QF 0 = invalid, 6 = dead reckoning, 7 = manual: not a measurement. The
    //    scale is not ordered by quality, hence the mask.
    const uint8_t qf = d.headingQuality;
    if (qf >= 16 || (config.headingQualityMask & (1u << qf)) == 0) {
        reason = RejectReason::QF;
    }

    // 2. A zeroed sentence. Observed verbatim under trees:
    //    "$GNHPR,...,000.0000,000.0000,000.0000,0,0,0.00,0000*6E"
    //    QF and sat go to 0 together with the angles, so "all four zero" is a
    //    reliable single test; the heading itself is allowed to be exactly 0.
    if (reason == RejectReason::NONE && qf == 0 && d.numSatellites == 0 &&
        d.dualHeading == 0.0f && roll == 0.0f) {
        reason = RejectReason::SENTENCE_ZEROED;
    }

    // 3. Roll envelope - DECOUPLED from the heading (operator request "P1a", 2026-09-30).
    //    Heading and roll come out of the SAME baseline solution, so a blow-up in the
    //    roll used to be treated as a free detector for a bad heading. The 2026-09-30
    //    tree log shows that this is too coarse: of 30 roll-envelope runs, 13 had a
    //    heading that matched the VTG course within 0.3...9 deg (one run: 76 frames,
    //    |roll| 66 deg, 26 satellites, heading deviation 0.4 deg, fixQ 1 - a good
    //    attitude thrown away for 7.6 s) and 17 were 12...135 deg off (those MUST stay
    //    out). So the roll no longer condemns the heading: the frame is accepted when
    //    an INDEPENDENT source confirms the heading - the same BODY vs PATH cross-check
    //    as step 6, with the same reverse handling - and rejected as before when no
    //    reference exists (slow, standing, no fix) or the check fails. The roll itself
    //    is never published from such a frame: updateRoll() has its own
    //    |dualRoll| <= maxRollAbsDeg test and holds/fades instead.
    bool rollOutlier = false;
    if (reason == RejectReason::NONE && fabsf(roll) > config.maxRollAbsDeg) {
        const bool refAvailable = cogValid && (lastSpeedMps > config.rollOutlierRefSpeedMps);
        float dev = 999.0f;
        if (refAvailable) {
            const float devFwd = fabsf(wrap180(vehicleHeading - cogMeas));
            const float devRev = fabsf(wrap180(vehicleHeading - (cogMeas + 180.0f)));
            dev = status.dirChanging ? ((devFwd < devRev) ? devFwd : devRev)
                                     : (status.reverseDirection ? devRev : devFwd);
        }
        if (refAvailable && (dev <= config.rollOutlierMaxDevDeg)) {
            rollOutlier = true;
            // Once per episode (status.rollSuspect still holds the previous frame),
            // so the drive log shows when a heading was rescued - and which roll was
            // NOT used for it.
            if (!status.rollSuspect && config.logEvents) {
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge roll envelope: heading KEPT at |roll| %.1f deg "
                           "(vehicle %.1f vs course %.1f, dev %.1f, %.2f m/s, qf %u, %u sats) "
                           "- the roll output holds",
                           (double)fabsf(roll), (double)vehicleHeading, (double)cogMeas,
                           (double)dev, (double)lastSpeedMps,
                           (unsigned)d.headingQuality, (unsigned)d.numSatellites);
            }
        } else {
            reason = RejectReason::ROLL_ENVELOPE;
        }
    }

    // 4. Roll step. The garbage frames move the roll by tens of degrees in one
    //    100 ms step (measured max 58.9 deg) because heading and roll come out of
    //    the SAME broken baseline solution, so a roll blow-up is a free second
    //    detector for a bad heading.
    //    Skipped directly after a rejection: the first frame of a RECOVERY also
    //    differs from the last value (which was zeroed or garbage), and blocking
    //    it would delay the return to LIVE by a frame. The QF/zeroed tests
    //    already cover that situation.
    if (reason == RejectReason::NONE && haveLastRoll && lastFrameMs > 0 &&
        consecutiveRejects == 0) {
        const float dRoll = fabsf(roll - lastRoll);
        const float dt100 = (float)(now - lastFrameMs) / 100.0f;
        // Speed proportional (see the config comment): the roll rate a bump
        // produces scales with speed, so a fixed 8 deg/100 ms would reject real
        // rows at 10 m/s and accept nonsense at 1 m/s.
        const float lim = (config.maxRollStepDegPer100ms +
                           config.rollStepPerSpeedDegPer100ms * lastSpeedMps) *
                          (dt100 > 0.2f ? dt100 : 0.2f);
        if (dRoll > lim) {
            reason = RejectReason::ROLL_STEP;
        }
    }

    // 4b. Settling after a reception gap (operator request 2026-09-24). The receiver
    //     needs a moment after a re-lock before its baseline solution is a solution:
    //     measured in the hall logs, the first 0.5...2 s carried roll values of
    //     10...19 deg on a machine driving straight, with the heading wandering along
    //     - and those frames passed every gate (roll envelope 25 deg, roll step only
    //     large jumps). They are OBSERVED (unused frames count as rejected here, so
    //     the log keeps showing them) but not USED until the sequence has been
    //     consistent for reentrySettleMs: |roll| small, and roll and azimuth changing
    //     slowly between frames. The timeout is the safety net for rough ground,
    //     where a perfectly still sequence may never appear.
    if (reason == RejectReason::NONE && !prevTrustedLatch) {
        if (!settleHaveLast) {
            // First frame after the gap: it decides nothing, it only seeds the
            // comparison.
            settleHaveLast = true;
            settleStartMs = now;
            settleGoodSinceMs = now;
            settleLastRoll = roll;
            settleLastAtt = azimuth;
            reason = RejectReason::SETTLING;
        } else {
            const bool stable =
                (fabsf(roll) <= config.reentrySettleRollDeg || rollOutlier) &&
                (fabsf(roll - settleLastRoll) <= config.reentrySettleRollStepDeg) &&
                (fabsf(wrap180(azimuth - settleLastAtt)) <= config.reentrySettleAttStepDeg);
            if (!stable) settleGoodSinceMs = now;
            settleLastRoll = roll;
            settleLastAtt = azimuth;
            const bool settled = (now - settleGoodSinceMs) >= config.reentrySettleMs;
            const bool timeout = (now - settleStartMs) >= config.reentrySettleTimeoutMs;
            if (!settled && !timeout) reason = RejectReason::SETTLING;
        }
        status.reentrySettling = (reason == RejectReason::SETTLING);
        status.reentrySettlingMs = now - settleStartMs;
        if (!status.reentrySettling && config.logEvents) {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge attitude settled after %lums (roll=%.1f, az=%.1f)",
                       (unsigned long)status.reentrySettlingMs, (double)roll,
                       (double)azimuth);
        }
    } else if (prevTrustedLatch) {
        // Normal operation: arm the settling test for the next gap.
        settleHaveLast = false;
        status.reentrySettling = false;
    }

    // 5. Heading step, limited by what the machine can physically do:
    //    omega = v * tan(delta_max) / L_eff plus a fixed allowance for a standing
    //    start. At 2.5 m/s the limit is ~5 deg + 3 deg per 100 ms, at 10 m/s
    //    ~17 deg per 100 ms - wide enough for real lock-to-lock turns, far too
    //    narrow for the observed 40 ... 60 deg artefacts.
    //    Reference is the last TRUSTED heading and the elapsed time is measured
    //    from that frame, so a recovery after a gap is judged against how long
    //    the machine was actually blind - otherwise the restored heading would
    //    look like a jump and the gate would never re-open.
    if (reason == RejectReason::NONE && haveTrusted && consecutiveRejects == 0) {
        const float blindS = (float)(now - lastTrustedMs) / 1000.0f;
        const float dtStep = (blindS > 0.05f) ? blindS : 0.1f;
        const float dPsi = fabsf(wrap180(vehicleHeading - lastTrustedVehHeading));
        // Physical part (speed proportional, see the config comment) plus a noise
        // part from the CURRENT measurement noise: at good reception the gate stays
        // tight and rejects more, under trees it opens up by 3 sigma instead of
        // throwing away usable frames.
        const float gatePhysical = config.maxHeadingStepDeg +
                                   config.headingStepPerSpeedDegS * lastSpeedMps * dtStep;
        // M1-Entkopplung (2026-10-06): das Step-Gate rechnet mit dem VOLLEN R, sonst
        // schaerft der psi-Anker das Gate mit und verwirft genau die Frames, mit denen
        // psi dem Empfaenger nachziehen soll (gemessen: Anker wirkte nur 11,6 -> 9,6 Grad
        // statt < 3 Grad).
        rAnchorSuppress = true;
        const float gateNoise = config.headingStepNoiseSigma * effectiveMeasurementR(0);
        rAnchorSuppress = false;
        const float limit = (gatePhysical > gateNoise) ? gatePhysical : gateNoise;
        if (dPsi > limit) {
            reason = RejectReason::HEADING_STEP;
        }
    }

    // 6. Cross-check against the two independent direction measurements. The
    //    attitude heading describes the BODY (vehicle FRONT axis); COG and
    //    fix-to-fix describe the PATH (direction of travel). Going FORWARD they
    //    agree (measured mean 1.3, p90 4.5 deg at 10 m/s); going BACKWARDS they are
    //    180 deg apart BY DEFINITION, because the front axis does not turn around
    //    when the driver reverses. A one-sided test would therefore throw away the
    //    attitude on every reverse manoeuvre (measured 2026-09-23 19:57: 1024 of
    //    1782 frames rejected as cogX, LIVE -> COAST for most of a purely FORWARD
    //    drive with a swapped antenna offset).
    //    So the difference is measured against the direction that is currently
    //    valid: while the direction of travel is known, only the matching side may
    //    be judged (a 180 deg wrong heading is still caught); during a direction
    //    change nothing can be concluded - AgOpenGPS pauses its own decision the
    //    same way ("isChangingDirection -> goto byPass", Position.designer.cs:493).
    if (reason == RejectReason::NONE && cogValid && lastSpeedMps > config.minSpeedForCogCheck) {
        const float devFwd = fabsf(wrap180(vehicleHeading - cogMeas));
        const float devRev = fabsf(wrap180(vehicleHeading - (cogMeas + 180.0f)));
        const float dev = status.dirChanging
                          ? ((devFwd < devRev) ? devFwd : devRev)      // undecided
                          : (status.reverseDirection ? devRev : devFwd);
        // F3 (2026-10-01): the reverse hypothesis must not be able to condemn a good
        // attitude. Both the COG and the flag can be wrong (measured: the flag was wrong
        // for 60 s, the COG and the attitude agreed within 3 deg the whole time, and
        // cogX rejected every frame because revRev was compared - 177 deg). The POSITION
        // witness (never mirrored) has the last word here: if it agrees with the
        // attitude, the frame is accepted.
        bool rescuedByWitness = false;
        if (status.travelWitnessValid) {
            const float devW = fabsf(wrap180(vehicleHeading - status.travelWitnessDeg));
            if (devW <= config.cogCrossCheckDeg) rescuedByWitness = true;
        }
        if (!rescuedByWitness && dev > config.cogCrossCheckDeg) {
            reason = RejectReason::COG_CROSSCHECK;
        }
    }

    // 7. Baseline length (u-blox F9P integration manual's own attitude quality
    //    test, and it transfers to any dual-antenna receiver that reports the
    //    solved length): the length the receiver solved for must match the
    //    mechanical distance between the antenna phase centres. Multipath, a
    //    disturbed antenna or a blocked sky view move the solved length away from
    //    the mechanical one, and that happens BEFORE the heading visibly degrades.
    //
    //    The UM982 does not need it (measured: its attitude failure is an
    //    immediate 0 / QF = 0 / sat = 0 jump, not a drift), and $GNHPR carries no
    //    length field - this test exists for the F9P moving-baseline setup, which
    //    the operator reports as drifting with implausible values at QF 4/5.
    //
    //    Reference: Config::physicalBaselineM if set, otherwise LEARNED here while
    //    the attitude is plausible. The learned value is an average over the first
    //    baselineLearnFrames frames (a healthy receiver solves the correct length,
    //    so the learned reference IS the mechanical one), which means the test
    //    needs no tape measure and no vehicle.xml field - AgOpenGPS does not store
    //    an antenna spacing.
    if (reason == RejectReason::NONE && d.baselineLengthM > 0.01f) {
        status.baselineLengthM = d.baselineLengthM;
        // B10.47 (2026-10-18): LERNEN und VORSCHLAG nur aus FRISCHEN frames. Unter der
        // Bruecke meldete der Empfaenger 2,09...3,09 m Loesungslaenge bei uni_hdgsd
        // 70...100 Grad - solche Frames sind zwar nicht "verworfen" (die Qualitaet ist ein
        // anderes Tor), wuerden aber eine GELERNTE Referenz dauerhaft verderben (Start
        // unter der Bruecke!) und mit festem EEPROM-Wert einen Fehlalarm ausloesen
        // ("Referenz falsch, uebernimm 3,09 m"). Gemessen 14-59: der Vorschlag stand auf
        // 3,094 m, waehrend die echte Basislinie 0,717 m ist.
        // uniHdgStdDevDeg == 0 heisst "kein Wert" (F9P) -> dann nicht filtern, sonst
        // koennte ein F9P nie lernen.
        const bool refFrameClean = (d.uniHdgStdDevDeg <= 0.0f) ||
                                   (d.uniHdgStdDevDeg <= config.uniHdgSdGoodDeg);
        // --- ACTIVE REFERENCE CHECK (operator request 2026-10-11) ------------------------
        // Runs BEFORE any reference is applied, and `reason == NONE` here means "clean
        // w.r.t. ALL other gates and the baseline test has not run yet" - so this average
        // is INDEPENDENT of the reference and works exactly in the case where the learn
        // path below is suppressed: a FIXED reference is in use.
        if (refFrameClean && status.baselineSuggestCount < config.baselineSuggestFrames) {
            const float n = (float)(status.baselineSuggestCount + 1);
            status.baselineSuggestionM +=
                (d.baselineLengthM - status.baselineSuggestionM) / n;
            status.baselineSuggestCount++;
            // Compare once, as soon as the window is full: early enough to act before the
            // safety net (300 frames) mutes the test.
            if (status.baselineSuggestCount >= config.baselineSuggestFrames &&
                config.physicalBaselineM > 0.1f) {
                const float suggestDevPct =
                    100.0f * (status.baselineSuggestionM - config.physicalBaselineM) /
                    config.physicalBaselineM;
                status.baselineRefSuspect =
                    (fabsf(suggestDevPct) > config.baselineTolPct);
                if (config.logEvents) {
                    if (status.baselineRefSuspect) {
                        LOG_WARNING(EventSource::GNSS,
                                    "HeadingBridge baseline: the FIXED reference %.3f m does "
                                    "NOT match what the receiver solves (%.3f m, %+.1f %%, "
                                    "%u clean frames). Every judged frame will be rejected "
                                    "and the safety net will switch the test off. Enter "
                                    "%.3f m, or 0 to learn, or use 'use measured reference' "
                                    "on the bridge geometry page.",
                                    (double)config.physicalBaselineM,
                                    (double)status.baselineSuggestionM,
                                    (double)suggestDevPct,
                                    (unsigned)status.baselineSuggestCount,
                                    (double)status.baselineSuggestionM);
                    } else {
                        LOG_NOTICE(EventSource::GNSS,
                                   "HeadingBridge baseline: the FIXED reference %.3f m "
                                   "matches the solved length (%.3f m, %+.1f %%, %u clean "
                                   "frames)",
                                   (double)config.physicalBaselineM,
                                   (double)status.baselineSuggestionM,
                                   (double)suggestDevPct,
                                   (unsigned)status.baselineSuggestCount);
                    }
                }
            }
        }
        if (config.physicalBaselineM > 0.1f) {
            status.baselineRefM = config.physicalBaselineM;
        } else if (refFrameClean && status.baselineLearnCount < config.baselineLearnFrames) {
            // Running mean over the learning window; frames that are already
            // implausible are excluded by the gates above (reason == NONE here).
            const float n = (float)(status.baselineLearnCount + 1);
            status.baselineRefM +=
                (d.baselineLengthM - status.baselineRefM) / n;
            status.baselineLearnCount++;
        }
        if (status.baselineRefM > 0.01f) {
            const float devPct = 100.0f * (d.baselineLengthM - status.baselineRefM) /
                                 status.baselineRefM;
            status.baselineDevPct = devPct;
            if (!status.baselineTestDisabled) {
                // Judge only after the reference is established.
                const bool judged = (config.physicalBaselineM > 0.1f) ||
                                    (status.baselineLearnCount >= config.baselineLearnFrames);
                if (judged && fabsf(devPct) > config.baselineTolPct) {
                    reason = RejectReason::BASELINE_LENGTH;
                }
                // Safety net: a reference that rejects nearly EVERYTHING is the wrong thing,
                // not the solution. Count the rejects per window and switch the test off
                // instead of removing the heading - a wrong reference, learned or entered,
                // must never be able to silence the bridge. (Not a re-learn: the wrong
                // reference is never adopted, see Config::baselineGuardWindow.)
                if (judged) {
                    status.baselineGuardCount++;
                    if (reason == RejectReason::BASELINE_LENGTH) {
                        status.baselineGuardRejects++;
                    }
                    if (status.baselineGuardCount >= config.baselineGuardWindow) {
                        const float rejPct =
                            100.0f * (float)status.baselineGuardRejects /
                            (float)status.baselineGuardCount;
                        if (rejPct > config.baselineGuardRejectPct) {
                            status.baselineTestDisabled = true;
                            if (config.logEvents) {
                                LOG_WARNING(EventSource::GNSS,
                                            "HeadingBridge baseline test DISABLED: %d %% of "
                                            "the last %d judged frames were rejected - the "
                                            "REFERENCE (%.3f m) is wrong, not the solution. "
                                            "Set the antenna spacing (measured 0.711 m) or "
                                            "clear it to learn again.",
                                            (int)rejPct,
                                            (int)status.baselineGuardCount,
                                            (double)status.baselineRefM);
                            }
                        }
                        status.baselineGuardCount = 0;
                        status.baselineGuardRejects = 0;
                    }
                }
            }
        }
    }

    // P1a (2026-09-30): report the roll situation of THIS frame, accepted or not, so
    // the drive log shows when a heading was kept while the roll was unusable.
    status.rollSuspect = rollOutlier;

    // --- F4 (operator decision 2026-10-01): break the stuck state --------------
    // Measured in the 13-25 log: 60 s of DEAD-RECKONING while a VALID dual attitude was
    // present (qf 4, 20 sats) - the wrongly latched REVERSE made cogX reject every frame,
    // so psi drifted 30 deg at 1.5 deg/s and AOG saw "the tractor driving sideways".
    // If no measurement has been accepted for Config::stuckRescueAfterMs and the frame is
    // only rejected by a PLAUSIBILITY gate (never by the receiver's own quality), the
    // STATE is the suspicious one: accept the frame with its R multiplied
    // (Config::stuckRescueRFactor) so the estimate is pulled in gently instead of never.
    stuckRescueActive = false;
    if (reason == RejectReason::COG_CROSSCHECK || reason == RejectReason::ROLL_ENVELOPE ||
        reason == RejectReason::SETTLING || reason == RejectReason::ROLL_STEP) {
        const bool qualityOk = (qf < 16) && ((config.headingQualityMask & (1u << qf)) != 0) &&
                               !(qf == 0 && d.numSatellites == 0 && d.dualHeading == 0.0f &&
                                 roll == 0.0f);
        const uint32_t blindMs = (lastMeasurementMs > 0) ? (now - lastMeasurementMs) : 0;
        if (qualityOk && blindMs >= config.stuckRescueAfterMs) {
            status.stuckRescues++;
            stuckRescueActive = true;
            forceLog = true;
            if (config.logEvents) {
                LOG_WARNING(EventSource::GNSS,
                            "HeadingBridge: frame RESCUED after %lums (%s, qf %u, %u sats, "
                            "roll %.1f) - stale state, R x %.1f",
                            (unsigned long)blindMs, reasonName(reason), (unsigned)qf,
                            (unsigned)d.numSatellites, (double)roll,
                            (double)config.stuckRescueRFactor);
            }
            reason = RejectReason::NONE;
        }
    }

    lastRoll = roll;
    haveLastRoll = true;
    lastFrameMs = now;
    status.lastReject = reason;

    if (reason != RejectReason::NONE) {
        status.framesRejected++;
        if (consecutiveRejects < 255) consecutiveRejects++;
        status.attitudeTrusted = false;
        return;
    }

    // --- trusted ------------------------------------------------------------
    consecutiveRejects = 0;
    // R4: the receiver's MEASURED yaw rate between two accepted attitude frames. Computed
    // before lastTrustedVehHeading/lastTrustedMs are overwritten, and over the gap since
    // the last ACCEPTED frame - so a run of rejected frames automatically widens the
    // interval instead of producing a meaningless spike rate. Window 0.02...1 s: below it
    // is noise, above it the average no longer describes the manoeuvre.
    if (haveTrusted && lastTrustedMs != 0) {
        const float dtAtt = (float)(now - lastTrustedMs) * 0.001f;
        if ((dtAtt > 0.02f) && (dtAtt < 1.0f)) {
            attMeasRateDegS = wrap180(vehicleHeading - lastTrustedVehHeading) / dtAtt;
            attMeasRateValid = true;
        } else {
            attMeasRateValid = false;
        }
    } else {
        attMeasRateValid = false;
    }
    lastTrustedVehHeading = vehicleHeading;
    lastTrustedMs = now;
    lastTrustedOdometerM = odometerM;      // the budget counter starts here
    lastMeasurementMs = now;               // dead-reckoning counter restarts too
    lastMeasurementOdometerM = odometerM;
    status.attitudeTrusted = true;
    status.source = Source::ATTITUDE;
    // Independent witness for the direction decision (operator correction 2026-09-29):
    // the body heading the dual antenna reports. It is the only heading source that is
    // NOT mirrored while reversing, so it can confirm or veto a reversal - and it is what
    // psi has to stay close to (psiAttDevDeg is the pass-through check in the raw log).
    attWitnessDeg = vehicleHeading;
    attWitnessValid = true;
    status.psiAttDevDeg = wrap180(psi - vehicleHeading);

    // Seed the filter when this is the first usable frame after boot.
    if (!haveTrusted) {
        haveTrusted = true;
        psi = vehicleHeading;
        psiJumpedThisTick = true;   // R1: a deliberate jump, not a rate-limited move
        p11 = 4.0f;                 // +-2 deg after the first frame
        LOG_NOTICE(EventSource::GNSS,
                   "HeadingBridge: first trusted attitude: az=%.2f veh=%.2f qf=%u sats=%u",
                   (double)azimuth, (double)vehicleHeading,
                   (unsigned)qf, (unsigned)d.numSatellites);
    }

    // Measurement into the filter. The noise estimate is taken per source, so the
    // attitude (the good one) and the path fall-back (the noisy one) are weighted
    // differently without any hand tuning.
    //
    // Re-entry ramp (operator request 2026-09-23): if the attitude was NOT trusted at
    // the end of the previous tick, this is the frame that would snap psi to the
    // receiver in a single step (measured: 244 deg in one 100 ms frame at a hall exit).
    // Such a frame is applied with an inflated R, which spreads the same correction
    // over the next ~reentryRampMs - AgOpenGPS never sees a heading step.
    // The very first frame after boot is exempt (haveTrusted == false there): locking
    // on quickly is what the operator wants from a cold start.
    if (haveTrusted && !prevTrustedLatch &&
        status.framesSeen > 2) {
        reentryUntilMs = now + config.reentryRampMs;
        if (config.logEvents) {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge re-entry: ramping the attitude correction for "
                       "%lums (psi=%.0f, raw az=%.0f)",
                       (unsigned long)config.reentryRampMs, (double)psi,
                       (double)azimuth);
        }
    }
    float rAtt = effectiveMeasurementR(0);
    if (reentryUntilMs > now) {
        const float remain = (float)(reentryUntilMs - now);
        const float f = 1.0f + (config.reentryRampGain - 1.0f) *
                        (remain / (float)config.reentryRampMs);
        rAtt *= f;
    }
    applyMeasurement(vehicleHeading, rAtt, 0);
    status.attitudeTrusted = true;
    prevTrustedLatch = true;
}

// ---------------------------------------------------------------------------
// Effective measurement noise. Nominal value from the config, optionally
// replaced by the innovation-based estimate: for a correctly tuned filter
// E[innovation^2] = P_pred + R, so R_est = EMA(innov^2) - P_pred, clamped to a
// sane window. Cheap (one multiply-add) and it is what makes the same build work
// on a good-reception day and under trees - the measured difference between the
// 2026-09-21 road data and the 2026-09-23 field data is a factor 3 in the
// attitude noise on the same machine.
float HeadingBridge::effectiveMeasurementR(uint8_t kind) {
    // M1 (2026-10-06): solange der Empfaenger sauber ist, ist sein Azimut die Wahrheit -
    // dann darf er psi mit kleinerem R (also staerker) nachfuehren, statt psi frei davon
    // driften zu lassen (gemessen 2026-10-06: 11,6 Grad Abstand bei 1,7 Grad Empfaenger-
    // genauigkeit). Wirkt NUR auf kind 0 (Attitude); alles andere bleibt unveraendert.
    float anchor = 1.0f;
    if ((kind == 0) && config.psiAnchorEnabled && status.attitudeTrusted &&
        !rAnchorSuppress) {
        const auto& dAnchor = gnssProcessor.getData();
        if (dAnchor.uniHdgStdDevDeg <= config.psiAnchorSdDeg) {
            anchor = config.psiAnchorRScale;
        }
    }
    const float nominal = ((kind == 0) ? config.rDualHeadingDeg : config.rFixToFixDeg) * anchor;
    if (!config.adaptiveMeasurementNoise) return nominal * receptionQualityFactor();
    const float ema = (kind == 0) ? rAttEmaSq : rPathEmaSq;
    float r = sqrtf(ema > 0.0f ? ema : 0.0f);
    // Subtract the prediction uncertainty in the variance domain (never below 0).
    const float var = (ema > p11) ? (ema - p11) : 0.0f;
    r = sqrtf(var);
    const float lo = (kind == 0) ? config.rAttMinDeg  : config.rPathMinDeg;
    const float hi = (kind == 0) ? config.rAttMaxDeg  : config.rPathMaxDeg;
    if (!(r > 0.0f)) r = nominal;          // also catches NaN
    if (r < lo) r = lo;
    if (r > hi) r = hi;
    // Quality factor LAST, so it is not clamped away by rAttMaxDeg: with the adaptive
    // estimate being far too optimistic in poor phases (measured 2026-09-27), this is
    // what actually protects the heading from the reception noise.
    float q = r * receptionQualityFactor();
    // F4 (2026-10-01): a frame rescued by the stuck detector is trusted GENTLY - its R is
    // multiplied so the stale state is pulled in over about a second instead of jumping.
    if (kind == 0 && stuckRescueActive) q *= config.stuckRescueRFactor;
    // ---- R-Deckel der Attitude (operator decision 2026-10-05) ----------------------
    // Der adaptive Wert (EMA der Innovation^2 minus p11) fuellt sich aus dem
    // ZUSTANDSfehler: ein bestehender Offset erhoeht R und senkt den Gewinn - der
    // Offset schuetzt sich selbst. Gemessen am 2026-10-04/05 (Logs 13-53, 16-04,
    // 17-28): R lief 3,95 -> 6,27 Grad, K1 0,023 -> 0,010, und ein 10-Grad-Offset
    // stand 35 s Geradeausfahrt still, obwohl die Attitude die belegte Wahrheit war
    // (Montageversatz att-cog stabil -88,2 Grad). Der Empfaenger liefert sein eigenes
    // 1-sigma je Epoche mit (uniHdgStdDevDeg = 0,40...0,60 Grad bei gutem Fix).
    // Untergrenze attRCapDeg, damit das Rauschen weiter gemittelt wird (die
    // Attitude streut 0,2 Grad/Frame); ein GROESSERES gemeldetes sigma hebt den
    // Deckel auf - schlechter Empfang zieht die Attitude dann wieder nicht hinein.
    if (kind == 0 && config.attRCapDeg > 0.0f) {
        float cap = config.attRCapDeg;
        const float sd = gnssProcessor.getData().uniHdgStdDevDeg;
        if (sd > cap) cap = sd;
        if (q > cap) q = cap;
    }
    return q;
}

// ---------------------------------------------------------------------------
// Yaw rate the machine can physically produce this tick (deg/s)
// ---------------------------------------------------------------------------
// omega = v * tan(wheel + slack) / L, plus a fixed allowance, raised by the turn rate
// of an ACCEPTED fix-to-fix window. Used by BOTH rate limiters:
//   * the MEASUREMENT limiter in applyMeasurement() (how far one measurement may pull
//     psi in the interval it belongs to) and
//   * the OUTPUT limiter in update() (how far the published heading may move per tick
//     while the receiver's own per-epoch quality says the attitude is degraded).
// One function, so the two can never describe different machines. The wheel angle is
// used as a MAGNITUDE only (a 10...15 % cpd error is harmless), and with an unreliable
// wheel angle the fallback keeps the bound open while the VWAS learns its zero.
//
// includeRaises = false gives the NARROW, purely wheel-based bound (2026-10-11, A10):
// the output FENCE must not be opened by the path course or by the measured attitude
// rate. Measured on the 10-11 logs: with the raises the fence allowed 2,0 deg steps on
// |wheel| <= 2 deg segments (the raises report a turn exactly when the wheel does not),
// i.e. it defeated its own purpose. The raises stay ON for the measurement limiter,
// where they exist to stop the limit from delaying the START of a real turn.
float FLASHMEM HeadingBridge::kinematicRateBoundDegS(bool includeRaises) const {
    const float vAbs = (lastSpeedMps > 0.0f) ? lastSpeedMps : 0.0f;
    const float wheelUse = wheelQualityOkState
                           ? (fabsf(lastWheelAngleDeg - status.vwasTrimAppliedDeg) + config.rateLimitWheelSlackDeg)
                           : config.rateLimitWheelFallbackDeg;
    const float L = ((config.wheelbaseM > 0.5f) ? config.wheelbaseM : 2.50f) *
                    config.effWheelbaseFac;    // 2026-10-05: effektive Laenge (siehe Config)
    // B10.51: gleiche Kinematik wie die Modellrate (Knicklenker: sin(gamma/2)/(L/2)).
    float allowed = hbModelRateDegS(wheelUse, vAbs, hbIsArticulated() ? hbWheelbaseM(L) : L) +
                    config.rateLimitMarginDegS;
    if (!includeRaises) return allowed;
    if (config.enablePathMeasurement && fabsf(pathRateDegS) > 0.5f) {
        const float pathAllowed = config.rateLimitPathFactor * fabsf(pathRateDegS) +
                                  config.rateLimitMarginDegS;
        if (pathAllowed > allowed) allowed = pathAllowed;
    }
    // R4 (2026-10-02): the receiver's OWN measured yaw rate. This is the term that makes
    // the bound correct in a tight manoeuvre - see Config::rateLimitAttFactor for the
    // measurement. It requires a TRUSTED attitude, so in the degraded phases of the
    // 2026-10-01 20-15 drive it contributes nothing and the bound stays purely kinematic.
    if ((config.rateLimitAttFactor > 0.0f) && status.attitudeTrusted && attMeasRateValid) {
        const float attAllowed = config.rateLimitAttFactor * fabsf(attMeasRateDegS);
        if ((attAllowed > config.rateLimitAttMinDegS) && (attAllowed > allowed)) {
            allowed = attAllowed;
        }
    }
    return allowed;
}

// ---------------------------------------------------------------------------
// Per-epoch attitude quality from the receiver's OWN UNIHEADINGA figures
// ---------------------------------------------------------------------------
// Why a separate signal: the adaptive measurement noise (effectiveMeasurementR) is an
// innovation EMA and needs ~10 s to react, while under the trees whole EPOCHS are bad.
// The Unicore receiver states its own 1-sigma heading/pitch standard deviations of the
// baseline solution per epoch (GNSSData::uniHdgStdDevDeg), and the solved baseline
// length is already cross-checked against the mechanical one (baselineDevPct).
//
// Return: 0 = clean epoch, 1 = degraded epoch, 2 = NO per-epoch figure available.
// 2 must leave the behaviour EXACTLY as before this gate (operator note: "the code
// works well with F9P + IMU" - an F9P sends no UNIHEADINGA).
uint8_t FLASHMEM HeadingBridge::attitudeQualityNow() const {
    if (!config.outQualityGateEnabled) return 2;
    const auto& d = gnssProcessor.getData();
    if (!d.hasDualHeading) return 2;     // attitude comes from COG/IMU: not this gate
    // Does this receiver state a per-epoch figure at all? uniCount > 0 means we have
    // seen an UNIHEADINGA sentence since boot; 0 = receiver does not send it.
    if (gnssProcessor.getUniCount() == 0) {
        return (d.headingQuality == 4) ? 2 : 1;
    }
    bool degraded = (d.headingQuality != 4);
    // uniHdgStdDevDeg == 0 with UNIHEADINGA present is the receiver's own
    // INSUFFICIENT_OBS: the sentence arrives, the solution does not exist.
    if (!(d.uniHdgStdDevDeg > 0.0f)) degraded = true;
    if (d.uniHdgStdDevDeg > config.uniHdgSdGoodDeg) degraded = true;
    if (d.uniHdgStdDevDeg >= config.uniHdgSdBadDeg) degraded = true;
    // NOTE: the SAME reference, the SAME deviation and (since 2026-10-03) the SAME tolerance
    // - Config::baselineTolPct - as the reject gate in evaluateAttitudeFrame(). Only the
    // consequence differs: here the epoch is marked DEGRADED (att_q = 0, the output is
    // rate-limited) and the attitude is still used; there the measurement is DISCARDED.
    // A wrong reference therefore hits BOTH paths, which is why the safety net switches both
    // off and why one correct EEPROM value fixes both.
    // baselineTolPct has three consistent users: this gate, the reject gate, and the learn
    // gate (which frames may feed the learned reference) - one number, one meaning.
    if (!status.baselineTestDisabled &&
        status.baselineRefM > 0.01f &&
        fabsf(status.baselineDevPct) > config.baselineTolPct) {
        degraded = true;
    }
    return degraded ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Reception quality factor for the measurement noise
// ---------------------------------------------------------------------------
// Operator report 2026-09-27: the heading flickers with bad reception and the
// autosteer then steers wildly. Measured: the per-frame step grows 5...6x (0.1 ->
// 0.7 deg p50, up to 9.5 deg p99) while the adaptive R estimate stays at 2.4 deg.
// This factor acts immediately (the adaptive one needs ~10 s) and is applied to every
// measurement source, so a bad phase also stops the self-learning (the gates use the
// same value).
float HeadingBridge::receptionQualityFactor() {
    const auto& d = gnssProcessor.getData();
    float q = 1.0f;
    if (!status.attitudeTrusted) {
        q = config.rQualityDegraded;
    } else if (d.headingQuality != 4) {
        q = config.rQualityMarginal;
    } else if (config.rQualitySats > 0 && d.numSatellites > 0 &&
               d.numSatellites < config.rQualitySats) {
        q = config.rQualityMarginal;
    }
    // Model-plausibility gate: a SUSTAINED disagreement between what the machine did
    // and what the kinematics say (see Config::gateFloorDeg). It composes with the
    // reception factor instead of replacing it and it damps, never discards - a wrong
    // scale/offset/wheelbase or slip must not be able to reject a real curve.
    if (config.modelGateEnabled && status.modelGateActive) q *= config.gateRFactor;
    return q;
}

// ---------------------------------------------------------------------------
// Ramp time for a position residual (operator correction 2026-09-29)
// ---------------------------------------------------------------------------
// A first-order blend covers 3*residual/T of the distance in its first second, so T is
// derived from the allowed catch-up SPEED: T = 3*residual/vmax. A 2.9 m bridge jump then
// takes ~5.8 s at vmax = 1.5 m/s instead of arriving as ~29 m/s in one 100 ms frame -
// which is exactly what the operator saw as "the tractor jumps on the map".
uint32_t FLASHMEM HeadingBridge::catchUpRampMs(float residualM) const {
    if (config.posMaxCatchUpMps <= 0.0f) return config.posRampMs;
    if (!(residualM > 0.0f)) return config.posRampMs;
    uint32_t ms = (uint32_t)(3000.0f * residualM / config.posMaxCatchUpMps);
    if (ms < config.posRampMs) ms = config.posRampMs;
    if (ms > config.posRampMaxMs) ms = config.posRampMaxMs;
    return ms;
}

// ---------------------------------------------------------------------------
// Scalar measurement update on the 2-state vector (heading, wheel-angle bias)
// ---------------------------------------------------------------------------
// Both states are observed only through the heading, so H = [1 0] and the
// covariance update is the standard 2x2 form. Keeping it hand-written instead of
// a matrix library is deliberate: three floats and no loops fit the 100 Hz loop
// and the RAM budget.
void FLASHMEM HeadingBridge::applyMeasurement(float measDeg, float rDeg, uint8_t kind,
                                              float dtRefS) {
    const float innov = wrap180(measDeg - psi);
    const float S = p11 + rDeg * rDeg;
    if (S <= 0.0f) return;

    // --- time base of the rate limiter (2026-10-02 "R2", extended 2026-10-05 "D2") ---------
    // The limiter bounds how far ONE measurement may pull psi, so that bound is
    // allowed_rate [deg/s] * dt, dt being the interval THIS measurement belongs to.
    // Every kind is applied at the rate of ITS OWN source, never on the 100 Hz tick:
    //   * the attitude arrives at ~10 Hz - using the tick length made the correction 10x
    //     tighter than designed, and the heading visibly lagged in curves (2026-10-01/02),
    //   * the path kinds (1 fix-to-fix, 2 COG, 3 IMU) are applied at the FIX rate (~0.1 s) and
    //     suffered the same 10x error until 2026-10-05. Measured on the 2026-10-04 logs: the
    //     projection is 0.70 deg accurate on the frames where psi is 9.40 deg off, yet psi never
    //     converged - a correct measurement whose allowed step is 10x too small is clipped away
    //     (rl_allowed ~13 deg/s against the ~1.7 deg/frame the filter asks for).
    // The interval is measured since the LAST APPLICATION OF THAT KIND, so an outage with
    // rejected frames widens the bound as well.
    float dtUse = (dtRefS > 0.0f) ? dtRefS : ((lastDtS > 0.0f) ? lastDtS : 0.01f);
    {
        const uint32_t nowKindMs = millis();
        uint32_t& lastKindMs = lastMeasMsByKind[kind & 3u];
        if (lastKindMs != 0) {
            const float d = (float)(nowKindMs - lastKindMs) * 0.001f;
            const float dMax = (kind == 0) ? 0.5f : 1.0f;   // attitude ~10 Hz; path kinds up to 1 Hz
            if (d > 0.02f && d < dMax) {
                dtUse = d;
                if (kind == 0) attMeasPeriodS = d;
            }
        }
        lastKindMs = nowKindMs;
    }

    // --- state sanity: re-seed instead of correcting the long way around -----
    // Operator correction 2026-09-29 ("AOG zeigt RÃ¼ckwÃ¤rts, obwohl ich vorwÃ¤rts fuhr").
    // A heading that is more than reseedBeyondDeg away from a TRUSTED attitude is not a
    // measurement problem - the STATE is wrong (measured: psi walked 124 deg away from a
    // steady qf=4 antenna heading within 1.2 s during a reverse phase, and integrating it
    // back took the long way around through the 180 deg ambiguity, which is exactly where
    // AgOpenGPS latches its own reverse detection). Re-seeding is a step, but a correct
    // one: AOG's comparison then agrees again in the same frame, while a gradient
    // correction keeps the wrong orientation alive for seconds.
    // Only the ATTITUDE (kind 0) and the position-confirmed IMU HEADING (kind 3) may do
    // this: both are body referenced and cannot be mirrored. Path/COG measurements may be
    // 180 deg off by definition.
    // The IMU heading needs the POSITION witness to agree (operator decision 2026-09-29):
    // its offset is learned against the GPS, so a wrong offset must never be able to pull
    // the state - the witness is the one direction that no heading source can fake.
    if ((kind == 0 || kind == 3) && fabsf(innov) > config.reseedBeyondDeg &&
        (kind == 0 || (status.travelWitnessValid &&
                       fabsf(wrap180(measDeg - status.travelWitnessDeg)) <=
                           config.reseedImuConfirmDeg))) {
        const float wasPsi = psi;
        // B (2026-10-07): nicht mehr hart setzen. Ein Re-Seed von +31...+94 Grad war ein
        // Sprung in der Ausgabe (Betreiber: "Traktor steht schraeg zur aufgezeichneten
        // Spur"). Begrenzter Schritt je Tick; die Ausgangsstufe darf ihn jetzt ebenfalls
        // begrenzen (psiJumpedThisTick bleibt bewusst aus).
        const float reseedDelta = wrap180(measDeg - psi);
        const float reseedMax = config.reseedRampDegPerTick;
        const float reseedUse = (reseedDelta > reseedMax) ? reseedMax
                              : ((reseedDelta < -reseedMax) ? -reseedMax : reseedDelta);
        psi = wrap360(psi + reseedUse);
        p11 = 4.0f;
        status.headingReseeds++;
        status.lastInnovDeg = innov;
        status.lastGainK = 1.0f;
        status.lastMeasKind = 1;
        forceLog = true;
        if (config.logEvents) {
            LOG_WARNING(EventSource::GNSS,
                        "HeadingBridge: heading RE-SEEDED by %.0f deg (was %.0f, attitude "
                        "%.0f deg, qf %u, %u sats, speed %.2f m/s) - the state was wrong, "
                        "not the measurement",
                        (double)innov, (double)wrap360(wasPsi), (double)wrap360(measDeg),
                        (unsigned)gnssProcessor.getData().headingQuality,
                        (unsigned)gnssProcessor.getData().numSatellites,
                        (double)lastSpeedMps);
        }
        return;
    }

    status.lastInnovDeg = innov;
    status.lastMeasKind = (uint8_t)((kind == 0) ? 1 : (kind == 1) ? 2 : (kind == 3) ? 4 : 3);
    // Any accepted measurement means the filter has a usable heading from now on - this is
    // what lets a machine without a dual antenna leave INIT (state machine, VWAS,
    // NAVProcessor; operator decision 2026-09-29).
    haveAnyHeadingLatch = true;

    // --- diagnostic steerScale estimate -------------------------------------
    // For a pure scale error the innovation is (steerScale - 1) * X, where X is
    // the geometric heading change the raw wheel angle would have produced since
    // the last measurement (accumulated in the prediction step). Recursive least
    // squares through the origin with forgetting.
    //
    // Only ATTITUDE measurements feed it, only in the LIVE state, and only when
    // the accumulated turn is large enough to stand out of the noise: measured on
    // the 2026-09-23 drive, feeding every innovation (including the COG, which
    // carries a few degrees of side slip) drove the estimate to its upper clamp in
    // 141 of 656 samples - a 2x too strong prediction, i.e. the opposite of a fix.
    // NOTE (2026-09-23): innov is the innovation of THIS measurement, but the scale
    // estimator compares the heading movement over the COLLECTION WINDOW with the
    // measurement that closes it - because a single 100 ms step accumulates only
    // ~1...2 deg of geometric turn (57.3 * v * tan(delta) / L * 0.1 s at 1.3 m/s),
    // i.e. far below the 5 deg signal threshold. Measured: sc_n stayed at exactly 0
    // over 244 s of driving, so the estimator never received a single sample. The
    // offline fit that produced the original 1.38 factor used 1 s windows - same
    // idea here, only in real time.
    // --- model-plausibility gate -------------------------------------------
    // Operator design 2026-09-27 (see Config::gateFloorDeg for the limits and the
    // measurements behind them): compare the heading change the RECEIVER reports over
    // a short window with the change the kinematics predict for that same window, and
    // DAMP (never discard) when they disagree. The window is short on purpose - the
    // deviation on good frames grows faster than the signal when the window grows.
    if (config.modelGateEnabled && wheelQualityOkState && kind == 0) {
        const uint32_t nowMsG = millis();
        if (gateStartMs == 0 || (nowMsG - gateStartMs) > 3u * config.gateWindowMs) {
            gateStartMs = nowMsG;               // (re-)open, nothing to judge yet
            gateStartRawDeg = measDeg;
            gateAccX = 0.0f;
            gateAccQ = 0.0f;
        } else if ((nowMsG - gateStartMs) >= config.gateWindowMs) {
            const float biasRad = steerBiasDeg * 0.017453292519943295f;
            const float modelChange = config.steerScale * gateAccX + gateAccQ * biasRad;
            const float measChange = wrap180(measDeg - gateStartRawDeg);
            const float dev = fabsf(measChange - modelChange);
            const float lim = fmaxf(config.gateFloorDeg,
                                    config.gateBandPct * fabsf(modelChange));
            status.modelDevDeg = dev;
            status.modelChangeDeg = modelChange;
            if (status.attitudeTrusted && dev > lim) {
                modelGateUntilMs = nowMsG + config.gateHoldMs;
                status.modelGateActive = true;   // takes effect on the next R call
            }
            gateStartMs = nowMsG;
            gateStartRawDeg = measDeg;
            gateAccX = 0.0f;
            gateAccQ = 0.0f;
        }
    }

    // --- diagnostic steerScale estimate (SCALE only since 2026-09-27, see [B8.103]) ---
    if (kind == 0 && status.attitudeTrusted) {
        const uint32_t nowMs = millis();
        if (!scaleCollecting) {
            scaleCollecting = true;
            scaleStartPsi = psi;            // heading before this measurement
            scaleStartRawDeg = measDeg;     // and the RAW value, see innovWin below
            scaleStartMs = nowMs;
            scaleStartRev = status.reverseDirection;
            scaleAccX = 0.0f;
        } else if ((nowMs - scaleStartMs) > config.steerScaleWindowMaxMs) {
            // Do not let the window span an unbounded stretch of driving.
            scaleStartPsi = psi;
            scaleStartRawDeg = measDeg;
            scaleStartMs = nowMs;
            scaleStartRev = status.reverseDirection;
            scaleAccX = 0.0f;
        }
        const float X = scaleAccX;
        if (fabsf(X) >= config.steerScaleMinSignalDeg) {
            // The window's heading change is taken from the RAW measurement, not from
            // the filter state: psi already contains the filter's own corrections during
            // the window, so measuring the change against it feeds the estimator partly
            // its own output (and, while the bias state is adapting, the wrong model).
            const float innovWin = wrap180(measDeg - scaleStartRawDeg);
            // Only CLEAN windows may shape a machine constant (2026-09-23): a hall
            // exit once snapped psi by 244 deg, and that single event would have
            // pulled the estimate to 1.54 and - had it been active - written it to
            // the EEPROM. So: enough satellites, no direction change (neither in
            // progress nor during the window) and a bounded innovation.
            const auto& gd = gnssProcessor.getData();
            const bool clean =
                (gd.fixQuality >= 4) &&
                ((gd.numSatellites == 0) ||
                 (gd.numSatellites >= config.minSatsForLearn)) &&
                (fabsf(status.baselineDevPct) <= config.baselineTolPct) &&
                (effectiveMeasurementR(0) <= config.maxAttRForLearn) &&
                !status.dirChanging &&
                (status.reverseDirection == scaleStartRev) &&
                (fabsf(innovWin) <= config.maxLearnInnovDeg);
            scaleCollecting = false;
            scaleAccX = 0.0f;
            // Do not learn from a window the model gate just flagged as implausible: the
            // gate protects the model, so it must not be trained by the data it rejected.
            // (2026-09-27: with the bias state saturated, this estimator walked to its
            // clamp on the 14-17 tree drive.)
            if (clean && !status.modelGateActive) {
                // SCALE only, solved through the origin (sum X*innov / sum X*X) - but in
                // double precision, because the float sums of a long drive lose the
                // difference that the solution consists of.
                // No offset term (2026-09-27, operator decision): the wheel angle ZERO
                // belongs to the WAS/VWAS ("wenn ein echter was verbaut ist, wird der von
                // der Hardware einmalig 0 gesetzt"), so the bridge must not estimate it.
                // The earlier two-parameter form (with the operator's derivative column Q)
                // was ill-conditioned anyway: X and Q are strongly correlated, so det/A11/A22
                // is a difference of two nearly equal large numbers and the estimate walked
                // to its clamps (measured: sc 2.000 / sc_b +6.00 on the 14-17 drive).
                const double lam = 0.99;
                scaleAccXX = lam * scaleAccXX + (double)X * (double)X;
                scaleY1 = lam * scaleY1 + (double)X * (double)innovWin;
                if (scaleAccXX > 1.0) {
                    float s = 1.0f + (float)(scaleY1 / scaleAccXX);
                    if (s < config.steerScaleMin) s = config.steerScaleMin;
                    if (s > config.steerScaleLimit) s = config.steerScaleLimit;
                    const float prev = status.steerScaleEstimate;
                    status.steerScaleEstimate = s;
                    if (status.steerScaleLearnSamples > 20) {
                        // Spread of the estimate: only a value that has stopped moving
                        // is worth persisting. Without this, a single curved stretch in
                        // bad reception would be stored as the machine constant.
                        status.steerScaleSpread +=
                            0.02f * (fabsf(s - prev) - status.steerScaleSpread);
                    }
                    status.steerScaleLearnSamples++;
                }
            }
        }
    } else {
        // Path measurements (and frames without a trusted attitude) do not feed the
        // scale estimate and end a running collection window.
        scaleCollecting = false;
        scaleAccX = 0.0f;
    }

    // --- adaptive measurement noise -----------------------------------------
    // R is what decides how much of the measurement noise reaches the output
    // (jitter ~= K * sigma_meas, K = P_pred / (P_pred + R)). The reception of the
    // day changes the real noise by a factor 3 on the same machine, so R is
    // estimated from the innovation variance: for a well tuned filter
    // E[innov^2] = P_pred + R, i.e. R = EMA(innov^2) - P_pred. Slow EMA, clamped,
    // and only used when enabled.
    if (config.adaptiveMeasurementNoise) {
        if (kind == 0) {
            rAttEmaSq += config.rAdaptAlpha * (innov * innov - rAttEmaSq);
        } else {
            rPathEmaSq += config.rAdaptAlpha * (innov * innov - rPathEmaSq);
        }
    }

    const float K1 = p11 / S;
    const float K2 = p12 / S;

    // --- P2: physical RATE LIMIT (operator decision 2026-09-30) --------------
    // A slow machine with a small wheel angle cannot change its heading fast, so a
    // large step cannot be real. Bound the rate at which this measurement may pull
    // psi by the kinematics and clip the correction to it. See
    // Config::rateLimitEnabled for the measurements behind this (a rate limit beats
    // every averaging variant, and the peaks are single-frame events).
    float applied = K1 * innov;
    status.lastMeasRDeg = rDeg;
    if (config.rateLimitEnabled) {
        // The bound now comes from kinematicRateBoundDegS() so the MEASUREMENT limiter
        // and the OUTPUT limiter (see the output section in update()) cannot drift
        // apart - both have to describe the same physical machine. The behaviour is
        // unchanged: kinematics + margin, raised by an ACCEPTED fix-to-fix turn (that
        // raise is what stops the limit from delaying the START of a turn; measured:
        // SWALLOW was halved by it, see Temp\filter_compare.ps1 variant limW2).
        const float allowed = kinematicRateBoundDegS();
        // For the log: the part of the bound that came from the accepted path turn.
        float pathAllowed = 0.0f;
        if (config.enablePathMeasurement && fabsf(pathRateDegS) > 0.5f) {
            pathAllowed = config.rateLimitPathFactor * fabsf(pathRateDegS) +
                          config.rateLimitMarginDegS;
        }
        status.rateLimitAllowedDegS = allowed;
        status.rateLimitPathDegS = pathAllowed;
        // PERSISTENCE: a deviation that keeps asking in the SAME direction for
        // rateLimitPersistMs is not noise - the state has to catch up (a long
        // dead-reckoning phase, a wrong seed). See Config::rateLimitPersistMs.
        float persistFactor = 1.0f;
        const float stepNeeded = fabsf(applied);
        if (stepNeeded > allowed * dtUse) {
            const float sign = (applied >= 0.0f) ? 1.0f : -1.0f;
            if ((rateLimitDevSign == sign) && (rateLimitDevSinceMs > 0) &&
                (millis() - rateLimitDevSinceMs) >= config.rateLimitPersistMs) {
                persistFactor = config.rateLimitPersistRelax;
            } else if (rateLimitDevSign != sign || rateLimitDevSinceMs == 0) {
                rateLimitDevSinceMs = millis();
                rateLimitDevSign = sign;
            }
        } else {
            rateLimitDevSinceMs = 0;
            rateLimitDevSign = 0.0f;
        }
        const float maxStep = allowed * persistFactor * dtUse;
        if (fabsf(applied) > maxStep) {
            applied = (applied > 0.0f) ? maxStep : -maxStep;
            status.rateLimitClipped = 1;
        } else {
            status.rateLimitClipped = 0;
        }
    } else {
        status.rateLimitAllowedDegS = 0.0f;
        status.rateLimitPathDegS = 0.0f;
        status.rateLimitClipped = 0;
        rateLimitDevSinceMs = 0;
        rateLimitDevSign = 0.0f;
    }

    // The EFFECTIVE fraction of the innovation that reached psi (the raw K1 is the
    // filter gain; with the rate limit the two differ, and the log has to show the
    // applied one - HR column "gain").
    status.lastGainK = (fabsf(innov) > 0.001f) ? (applied / innov) : K1;
    psi = wrap360(psi + applied);

    // The bias state is only corrected when it is enabled. Its sign convention
    // matches the prediction (delta_eff = wheelAngle + bias), so a positive
    // innovation means "we are turning more than the model thinks" and the bias
    // moves in the same direction.
    //
    // It is corrected ONLY in a clean, trusted window (2026-09-27): the state is part
    // of the model, so adapting it from a bad phase drifts the whole heading. Measured
    // on the 14-12 drive: during the first ~40 s (INIT, no dual heading, the VWAS still
    // learning its own zero) the state ran into its +15 deg clamp and the dead-reckoned
    // heading wandered by more than 100 deg - exactly the "the heading did not fit at
    // the beginning, the wheel angle zero was still wrong" the operator reported.
    if (config.estimateSteerBias || !wheelQualityOkState) {
        const auto& gdBias = gnssProcessor.getData();
        const bool biasWindowOk = status.attitudeTrusted &&
                                  (gdBias.fixQuality >= 4) &&
                                  !status.dirChanging &&
                                  !status.modelGateActive;
        if (biasWindowOk) {
            steerBiasDeg += K2 * innov;
            if (steerBiasDeg >  config.steerBiasLimitDeg) steerBiasDeg =  config.steerBiasLimitDeg;
            if (steerBiasDeg < -config.steerBiasLimitDeg) steerBiasDeg = -config.steerBiasLimitDeg;
        }
    }

    const float newP11 = (1.0f - K1) * p11;
    const float newP12 = (1.0f - K1) * p12;
    const float newP22 = p22 - K2 * p12;
    p11 = newP11;
    p12 = newP12;
    p22 = newP22;

    // Keep the covariance from collapsing numerically (this is also what lets
    // the filter follow a change instead of going deaf).
    if (p11 < 0.01f) p11 = 0.01f;
    if (p22 < 1e-4f) p22 = 1e-4f;
}

// ---------------------------------------------------------------------------
// Roll channel
// ---------------------------------------------------------------------------
// The roll is used by AOG for two things, and both react immediately:
//   * rollCorrectionDistance = sin(roll) * antennaHeight  (position shift, up to
//     1 m at 20 deg roll and 3 m antenna height),
//   * the sidehill draft term in the steering command.
// So a roll STEP is a position jump and a steering kick. A dropped-to-zero roll
// under trees is exactly that, which is why the roll gets its own channel here:
// low pass while live, hold when the attitude goes, then fade (the true roll of a
// machine that is being driven will change, so holding it forever is also wrong -
// fading to zero bounds the error instead of leaving a stale bias in the steering).
void HeadingBridge::updateRoll(float dtS, uint32_t now, bool attitudeTrusted) {
    const auto& d = gnssProcessor.getData();

    // IMU roll availability (operator decision 2026-09-29): the same freshness/quality
    // gates the IMU heading uses - a stale IMU must not steer the roll either.
    bool imuRollUsable = false;
    float imuRollMeas = 0.0f;
    if (imuProcessor.isActive()) {
        const IMUData& imu = imuProcessor.getCurrentData();
        const uint32_t imuAge = millis() - imu.timestamp;
        if (imu.isValid && imu.quality >= config.imuHeadingMinQuality &&
            imuAge <= config.imuHeadingMaxAgeMs) {
            imuRollMeas = imu.roll;
            status.imuRollDeg = imu.roll;
            imuRollUsable = true;
            // 2026-10-05: an envelope (fabsf(imu.roll) <= config.maxRollAbsDeg) was added here and
            // REVERTED the same day. It rested on the same wrong premise as the BNOAiOParser change
            // (see the "DO NOT FIX THIS AGAIN" note there): that the IMU roll arrives 10x too
            // large. It does not - the operator (author of the 2026-05-07 fix) confirms the roll
            // reaches AOG correctly, so an envelope would reject CORRECT values above
            // maxRollAbsDeg (25 deg) and change the behaviour the machines are tuned to.
            // The dual-roll envelope at the top of this function is untouched and stays.
        }
    }

    // ---- Roll-Qualitaet (2026-10-05) ---------------------------------------
    // Der Dual-Roll wurde bisher nur ueber "attitudeTrusted && |roll| <= Huelle"
    // angenommen - ohne Blick darauf, ob DIESE Epoche praezise ist. uniPtchStdDevDeg ist
    // das eigene 1-sigma des Baseline-Pitch = ROLL (die Baseline liegt quer), gemessen
    // 0,86...0,93 Grad bei gutem Empfang und 159 Grad bei schlechtem; baselineDevPct
    // beschreibt die Geometrie. Ein 159-Grad-sigma-Frame kann 30...60 Grad daneben liegen
    // und schiebt ueber die LPF einen Roll-Bias in AOG (sin(roll) * 2,2 m Antennenhoehe).
    // Hysterese 3/6 Grad. Auf einem Empfaenger ohne diese Felder (F9P: 0) bleibt das
    // Verhalten unveraendert, weil 0 <= rollQualitySdDeg gilt.
    static bool rollQualOk = false;
    const bool rollQualNow = (d.uniPtchStdDevDeg <= config.rollQualitySdDeg) &&
                             (fabsf(status.baselineDevPct) <= config.rollQualityBaselineTolPct);
    if (rollQualNow) { rollQualOk = true; }
    else if (d.uniPtchStdDevDeg > config.rollQualitySdOutDeg) { rollQualOk = false; }
    // --- Roll-BESTAETIGUNG (2026-10-18, Betreiber) ---------------------------------
    // Der HALTEMITTELWERT darf die letzten rollConfirmMs NICHT enthalten: vor einem
    // Ausfall sind genau diese Frames schon verdorben (gemessen 10-18: die letzte Sekunde
    // liegt 0,6...1,1 Grad hoeher, der LETZTE Wert springt, z.B. -2,1 -> -6,8 Grad;
    // |gehalten - raw| danach 3...5 Grad). Der Ring schiebt die Bestaetigung zeitlich
    // nach hinten - die AUSGABE bleibt live, nur der Haltewert wird gereinigt.
    // Auf der Holperstrecke (2026-09-23, guter Empfang) aendert sich der Roll max.
    // 0,89 Grad/Frame - die letzten Frames vor einem Ausfall liegen also ausserhalb.
    rollPendVal[rollPendIdx] = d.dualRoll;
    rollPendOk[rollPendIdx] = rollQualOk;
    rollPendMs[rollPendIdx] = now;
    rollPendIdx = (uint8_t)((rollPendIdx + 1) % RollPendN);
    float rollConfirmed = 0.0f;
    bool rollConfirmedOk = false;
    {
        uint8_t best = 255; uint32_t bestAge = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < RollPendN; k++) {
            if (rollPendMs[k] == 0u) { continue; }
            const uint32_t age = (uint32_t)(now - rollPendMs[k]);
            if (age < config.rollConfirmMs) { continue; }
            if (age < bestAge) { bestAge = age; best = k; }
        }
        if (best != 255) { rollConfirmed = rollPendVal[best]; rollConfirmedOk = rollPendOk[best]; }
    }
    // ---- Plausibilitaet der IMU-Roll (2026-10-06) --------------------------------
    // Nur wenn der Dual-Roll in DIESER Epoche sauber ist, ist er die Wahrheit - dann wird
    // die IMU daran gemessen. Bleibt die Abweichung klein, ist die IMU ein vollwertiger,
    // empfaengsunabhaengiger Roll und darf live gehen. Hysterese 1x / 1,6x der Grenze.
    if (attitudeTrusted && rollQualOk && imuRollUsable &&
        fabsf(d.dualRoll) <= config.maxRollAbsDeg) {
        const float dv = fabsf(imuRollMeas - d.dualRoll);
        const float aa = (config.imuRollPlausTauS > 0.001f)
                         ? (dtS / config.imuRollPlausTauS) : 1.0f;
        const float aaa = (aa > 1.0f) ? 1.0f : aa;
        if (!imuRollDevInit) { imuRollDevDeg = dv; imuRollDevInit = true; }
        else { imuRollDevDeg += (dv - imuRollDevDeg) * aaa; }
        if (imuRollDevDeg <= config.imuRollPlausDevDeg) { imuRollPlausible = true; }
        else if (imuRollDevDeg > (config.imuRollPlausDevDeg * 1.6f)) { imuRollPlausible = false; }
    }
    if (attitudeTrusted && rollQualOk && fabsf(d.dualRoll) <= config.maxRollAbsDeg) {
        const float measured = d.dualRoll;
        if (!rollInit) {
            // 2026-10-05 (Betreiber-Hinweis): der Re-Init hat den gesendeten Roll bisher
            // HART auf den Messwert gesetzt. AOG rechnet Roll in einen POSITIONS-Quer-
            // versatz um (sin(roll) * Antennenhoehe) - gemessen im Log 6-19 vom 2026-10-05
            // springt der GESENDETE Roll mit p99 13,2 Grad und max 25,0 Grad pro 100 ms,
            // obwohl |roll| selbst <= 4,2 Grad bleibt. Das sind genau diese Re-Seeds, und
            // sie ergeben 0,4...2,1 m AOG-Positionssprung (bei H = 1...5 m).
            // Nur der ALLERERSTE Init (rollF == 0, direkt nach dem Boot, da faehrt nichts)
            // setzt hart; danach zieht die vorhandene LPF (rollLpfTauS) den Wert rampenfrei
            // nach - kein Sprung mehr nach aussen.
            if (rollF == 0.0f) { rollF = measured; }
            rollInit = true;
        } else {
            // First order low pass; dt is the caller's loop time (100 Hz).
            const float alpha = (config.rollLpfTauS > 0.001f)
                                ? (dtS / config.rollLpfTauS) : 1.0f;
            const float a = (alpha > 1.0f) ? 1.0f : alpha;
            rollF += (measured - rollF) * a;
        }
        // 2026-10-06: dieselbe "gute Meter"-Pflege wie im IMU-Zweig. Ohne sie bleibt
        // rollGoodAvgDeg ungefuellt und das Halten hat im DUAL-Fall (Hauptpfad der
        // Maschine, Dual ohne IMU) nichts zum Halten - gemessen: Log 20-41 hatte kein
        // einziges roll_src = 4, obwohl 307 Epochen gefadet wurden.
        if (attitudeTrusted && (lastSpeedMps > 0.1f) && (dtS > 0.0f)) {
            const float dM = lastSpeedMps * dtS;
            const float w = (config.rollGoodAvgWinM > 0.5f)
                            ? (dM / config.rollGoodAvgWinM) : 1.0f;
            const float wa = rollConfirmedOk ? ((w > 1.0f) ? 1.0f : w) : 0.0f;
            if (!rollGoodAvgInit) { rollGoodAvgDeg = rollConfirmed; rollGoodAvgInit = true; }
            else { rollGoodAvgDeg += (rollConfirmed - rollGoodAvgDeg) * wa; }
        }
        rollLossMs = 0;
        rollLossM = 0.0f;
        status.rollSource = 1;
    } else if (imuRollUsable) {
        // Single-antenna fallback (operator decision 2026-09-29): without a dual antenna the
        // roll channel used to hold and then fade to ZERO, which cost the VWAS its
        // roll-binned zero learning (WheelAngleFusion bins the zero observations by roll)
        // and left AOG without roll. The IMU roll is a gravity reference (no magnetometer
        // involved), so it is the right substitute. The IMU mounting/axis configuration is
        // applied in IMUProcessor (BNO only - the TM171 is left untouched on purpose).
        const bool liveMode = config.imuRollLive && imuRollPlausible;
        const float measured = imuRollMeas;
        if (!rollInit) {
            // Siehe den Dual-Zweig oben: nur der allererste Init setzt hart, danach zieht
            // die LPF nach (sonst springt der Roll, den AOG als sin(roll)*H auf die
            // Position rechnet, bei jedem Wiederaufsetzen der Lage).
            if (rollF == 0.0f) { rollF = measured; }
            rollInit = true;
        } else {
            // Live-Modus: die IMU-Roll ist eine Gravitationsreferenz - nur eine sehr kurze
            // Daempfung (imuRollLpfTauS) gegen Einzelframe-Ausreisser, kein 0,8-s-Nachziehen.
            const float tauLpf = liveMode ? config.imuRollLpfTauS : config.rollLpfTauS;
            const float alpha = (tauLpf > 0.001f)
                                ? (dtS / tauLpf) : 1.0f;
            const float a = (alpha > 1.0f) ? 1.0f : alpha;
            rollF += (measured - rollF) * a;
        }
        // "gute Meter" fuer den Haltemittelwert: nur validierte Lage-Epochen zaehlen,
        // distanzgewichtet mit dem Fenster rollGoodAvgWinM. Das Fenster braucht ein paar
        // Meter, um nachzuziehen - genau der weiche Wiedereinstieg, den wir wollen.
        if (attitudeTrusted && (lastSpeedMps > 0.1f) && (dtS > 0.0f)) {
            const float dM = lastSpeedMps * dtS;
            const float w = (config.rollGoodAvgWinM > 0.5f)
                            ? (dM / config.rollGoodAvgWinM) : 1.0f;
            const float wa = rollConfirmedOk ? ((w > 1.0f) ? 1.0f : w) : 0.0f;
            if (!rollGoodAvgInit) { rollGoodAvgDeg = rollConfirmed; rollGoodAvgInit = true; }
            else { rollGoodAvgDeg += (rollConfirmed - rollGoodAvgDeg) * wa; }
        }
        rollLossMs = 0;
        rollLossM = 0.0f;
        status.rollSource = liveMode ? 5 : 2;   // 5 = IMU live/ungefiltert, 2 = IMU gefiltert
    } else {
        rollLossMs += (uint32_t)(dtS * 1000.0f);
        rollLossM += lastSpeedMps * dtS;
        // Halten heisst: den Mittelwert der letzten GUTEN Meter fahren - nicht den
        // stehengebliebenen letzten Wert, denn genau der ist bei schlechtem Empfang
        // zuerst verdorben, und nach dem Ausfall braucht der Roll Sekunden, bis er
        // wieder gut ist. Fade erst nach rollHoldM Metern ODER rollHoldMaxMs.
        const bool fadeTime = (config.rollHoldMaxMs > 0) &&
                              (rollLossMs > config.rollHoldMaxMs);
        const bool fadeDist = (config.rollHoldM > 0.0f) && (rollLossM > config.rollHoldM);
        if (!(fadeTime || fadeDist)) {
            // Halten: den Mittelwert der letzten guten Meter fahren - und das im Log
            // als EIGENE Quelle markieren (roll_src = 4). Ohne dieses Label sieht die
            // Haltephase wie "live" aus (2) und jede Auswertung ueber den Log luegt.
            // 4 = gehalten (letzte gute Meter), 3 = Fade nach 0, 2 = live (IMU/Zweig),
            // 0/1 = live (Dual-Zweig).
            if (rollGoodAvgInit) rollF = rollGoodAvgDeg;
            status.rollSource = 4;
        }
        if ((fadeTime || fadeDist) && config.rollFadeToZero) {
            const float alpha = (config.rollFadeTauS > 0.001f)
                                ? (dtS / config.rollFadeTauS) : 1.0f;
            const float a = (alpha > 1.0f) ? 1.0f : alpha;
            rollF += (0.0f - rollF) * a;
            status.rollSource = 3;
        }
        // else: hold the last filtered value unchanged
    }

    // Roll is a SIGNED angle around 0 - it must not be wrapped into 0..360.
    // --- Roll-RATENBEGRENZER (2026-10-18) -------------------------------------------
    // AOG rechnet den Fix SOFORT mit sin(roll)*Antennenhoehe um - ein Roll-Sprung ist
    // ein Positions-Quersprung (1 Grad bei 2,2 m = 3,8 cm). Gemessen: auf der Holper-
    // strecke bei gutem Empfang aendert sich der Roll max. 0,89 Grad/Frame; die in den
    // 10-18-Logs veroeffentlichten Schritte 1,4...1,6 (p99) bzw. 3,8...5,0 (max) sind
    // Artefakte. Grenze 10 Grad/s = 1,0 Grad/100 ms laesst also jede echte Bewegung durch.
    if (rollInit && (config.rollRateLimitDegPerS > 0.0f)) {
        const float maxStep = config.rollRateLimitDegPerS * ((dtS > 0.0f) ? dtS : 0.01f);
        float dStep = rollF - status.rollDeg;
        if (dStep > maxStep) { dStep = maxStep; }
        if (dStep < -maxStep) { dStep = -maxStep; }
        rollF = status.rollDeg + dStep;
    }
    status.rollDeg = rollF;
    (void)now;
}

// ---------------------------------------------------------------------------
// State machine + event logging
// ---------------------------------------------------------------------------
// The budget counts TIME and DISTANCE together, because the two failure cases
// differ: a standstill under a tree can last minutes without the heading drifting
// away (the kinematic model needs speed to be wrong), while 15 m through a bend at
// 10 m/s is only 1.5 s and turns the heading by tens of degrees.
// ---------------------------------------------------------------------------
// State machine + event logging
// ---------------------------------------------------------------------------
// The state is decided by WHICH source is still available, not by a timer alone.
// Operator requirement 2026-09-22: "construct values as long as possible, a
// complete drop-out is the bad case" - and that is correct, because a heading
// derived from the position stream is a MEASUREMENT of the path, not an estimate:
//
//   LIVE      attitude trusted                        -> raw attitude values
//   BRIDGE    attitude gone, path/COG valid, <= 2 s   -> path measurement
//   COAST     attitude gone, path/COG valid, longer   -> path measurement,
//                                                        "degraded", but NO time
//                                                        limit by default
//   DEADRECK  only wheel angle + speed left           -> budgeted (10 s / 50 m)
//   TIMEOUT   the dead-reckoning budget is spent      -> TimeoutAction towards AOG
//
// The distance term exists next to the time term because the two failure cases
// differ: standing under a tree can last minutes without the heading drifting
// away (a kinematic model needs speed to be wrong), while 15 m through a bend at
// 10 m/s is only 1.5 s and turns the heading by tens of degrees.
void HeadingBridge::updateState(uint32_t now) {
    const float ageMs = (float)(now - lastTrustedMs);
    const float distM = odometerM - lastTrustedOdometerM;
    // The fall-back sources dip in and out around their own limits (cogValid needs
    // speed and a fresh VTG, pathValid needs displacement), which made the state
    // flip between COAST and DEADRECKON every 100 ms under a roof (measured
    // 2026-09-23). Hold the last verdict for fallbackHoldMs so the state follows the
    // situation instead of the flicker.
    const bool pathUsableNow = (pathValid && config.enablePathMeasurement) ||
                               (cogValid && config.enableCogMeasurement);
    if (pathUsableNow) {
        pathUsableLatch = true;
        pathUsableFalseMs = now;
    } else if (pathUsableLatch && (now - pathUsableFalseMs) > config.fallbackHoldMs) {
        pathUsableLatch = false;
    }
    const bool pathUsable = pathUsableLatch;

    State next;
    // The INIT gate (operator decision 2026-09-29): INIT means "no usable heading from ANY
    // source yet" - not "no dual-antenna frame yet". A single-antenna machine (F9P/UM981 +
    // IMU, any GGA+VTG source) therefore reaches BRIDGE/COAST/DEADRECKON instead of being
    // stuck in INIT forever, where the VWAS fell back to the raw COG and NAVProcessor
    // published the receiver's empty heading. LIVE still requires a real BODY attitude
    // (dual antenna / INS) - see haveAnyHeadingLatch in applyMeasurement().
    if (!haveAnyHeadingLatch) {
        next = State::INIT;
    } else if (status.attitudeTrusted) {
        next = State::LIVE;
    } else if (pathUsable) {
        // Position-supported. coastMaxMs/coastMaxM == 0 means "no limit": as long
        // as the position stream keeps producing a trustworthy direction, the
        // heading stays a measurement and there is no reason to give up.
        const bool overCoast =
            (config.coastMaxMs > 0 && ageMs > (float)config.coastMaxMs) ||
            (config.coastMaxM > 0.0f && distM > config.coastMaxM);
        if (overCoast) {
            next = State::TIMEOUT;
        } else if (ageMs <= (float)config.bridgeMaxMs && distM <= config.bridgeMaxM) {
            next = State::BRIDGE;
        } else {
            next = State::COAST;
        }
    } else {
        // True dead reckoning - the only phase that has to be budgeted.
        const uint32_t drMs = now - lastMeasurementMs;
        const float drM = odometerM - lastMeasurementOdometerM;
        if (drMs > config.deadReckonMaxMs || drM > config.deadReckonMaxM) {
            next = State::TIMEOUT;
        } else {
            next = State::DEADRECKON;
        }
    }

    if (next != status.state) {
        const State prev = status.state;
        status.state = next;
        // Force the next diagnostic line out immediately: a 90 ms dropout between
        // two 0,5 Hz lines would otherwise be invisible in the session log.
        forceLog = true;

        if (next == State::BRIDGE && !inEvent) {
            inEvent = true;
            eventStartMs = now;
            eventStartOdometerM = odometerM;
            status.events++;
            timeoutLogged = false;
        }
        if (next == State::LIVE && inEvent) {
            inEvent = false;
            status.lastEventMs = (float)(now - eventStartMs);
            status.lastEventDistanceM = odometerM - eventStartOdometerM;
        }
        if (next == State::TIMEOUT && !timeoutLogged) {
            timeoutLogged = true;
            if (config.logEvents) {
                LOG_WARNING(EventSource::GNSS,
                            "HeadingBridge TIMEOUT: age=%.0fms dist=%.1fm action=%d "
                            "- heading no longer claimed as valid",
                            (double)ageMs, (double)distM, (int)config.timeoutAction);
            }
        }
        if (config.logEvents) {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge %s -> %s: age=%.0fms dist=%.1fm sigma=%.2fdeg "
                       "bias=%+.2fdeg att=%s lastReject=%s",
                       stateName(prev), stateName(next), (double)ageMs, (double)distM,
                       (double)sqrtf(p11), (double)steerBiasDeg,
                       status.attitudeTrusted ? "OK" : "BAD",
                       reasonName(status.lastReject));
        }
    }
}

// ---------------------------------------------------------------------------
// Geschwindigkeit aus WEG/ZEIT (2026-10-14, Betreiber-Auftrag "pruefe, ob wir bei
// schlechtem Empfang die Geschwindigkeit aus Weg/Zeit berechnen sollen")
// ---------------------------------------------------------------------------
// Referenz ist die Strecke zwischen dem NEUESTEN Fix und dem Fix, der mindestens
// speedFuseWindowS zurueckliegt (zentraler Ring, dieselbe Quelle wie Sehne/Tore).
// Danach eine EMA (speedFuseEmaS), weil Weg/Zeit bei Schleichfahrt rau ist.
// Zweifel am Empfaengerwert: |Empfaenger - vFused| > speedFuseDoubtMps fuer
// speedFuseDoubtMs; geloescht wird bei der HALBEN Schwelle (einfache Hysterese).
// Ohne Ring (Kaltstart, Dienst aus) bleibt der Empfaengerwert - die Funktion ist
// dann ein Durchreicher, damit sich am bisherigen Verhalten nichts aendert.
float FLASHMEM HeadingBridge::updateFusedSpeed(uint32_t now, float rawMps) {
    status.speedPosMps = 0.0f;
    status.speedDoubt = 0;
    if (!config.speedFuseEnabled) {
        status.speedFusedMps = rawMps;
        status.speedRatio = 1.0f;
        speedEmaInit = false;
        return rawMps;
    }

    float vPos = -1.0f;
    const PositionHistory::FixSample* newest = positionHistory.newest();
    if (newest != nullptr) {
        const uint32_t winMs = (uint32_t)(config.speedFuseWindowS * 1000.0f);
        const uint16_t cnt = positionHistory.count();
        const PositionHistory::FixSample* old = nullptr;
        for (uint16_t back = 1; back < cnt; back++) {
            const PositionHistory::FixSample* s = positionHistory.at(back);
            if (s == nullptr) break;
            old = s;                                  // neueste-zuerst: letzter Treffer zaehlt
            if ((newest->tMs - s->tMs) >= winMs) break;
        }
        if (old != nullptr) {
            const float dtW = (float)(newest->tMs - old->tMs) * 0.001f;
            double n = 0.0, e = 0.0;
            localDeltaMeters(old->lat, old->lon, newest->lat, newest->lon, n, e);
            const float dist = (float)sqrt(n * n + e * e);
            status.speedWinDtS = dtW;
            status.speedWinDistM = dist;
            if (dtW > 0.2f) {
                // Unter speedFuseMinDistM ist die Maschine gestanden: dann ist 0 richtig
                // (und nicht der Empfaengerwert, der im Stand gern 0.1...0.3 zeigt).
                vPos = (dist >= config.speedFuseMinDistM) ? (dist / dtW) : 0.0f;
            }
        }
    }
    if (vPos < 0.0f) {
        status.speedFusedMps = rawMps;                // Ring (noch) nicht nutzbar
        status.speedRatio = 1.0f;
        speedEmaInit = false;
        return rawMps;
    }
    status.speedPosMps = vPos;

    if (!speedEmaInit || (config.speedFuseEmaS <= 0.0f)) {
        speedEmaState = vPos;
        speedEmaInit = true;
        speedEmaMs = now;
    } else {
        const float dtE = (float)(now - speedEmaMs) * 0.001f;
        speedEmaMs = now;
        float al = dtE / config.speedFuseEmaS;
        if (al > 1.0f) al = 1.0f;
        if (al < 0.0f) al = 0.0f;
        speedEmaState += al * (vPos - speedEmaState);
    }
    status.speedFusedMps = speedEmaState;

    const float dev = fabsf(rawMps - speedEmaState);
    if (dev > config.speedFuseDoubtMps) {
        if (speedDoubtSinceMs == 0) speedDoubtSinceMs = now;
        if ((now - speedDoubtSinceMs) >= config.speedFuseDoubtMs) status.speedDoubt = 1;
    } else if (dev < (0.5f * config.speedFuseDoubtMps)) {
        speedDoubtSinceMs = 0;                       // Zweifel vorbei
    }
    status.speedRatio = (rawMps > 0.2f) ? (vPos / rawMps) : 1.0f;
    return speedEmaState;
}

// ---------------------------------------------------------------------------
// 100 Hz entry point
// ---------------------------------------------------------------------------
void HeadingBridge::update(float dtS, float wheelAngleDeg, float speedMps) {
    const uint32_t now = millis();
    if (dtS <= 0.0f) dtS = 0.01f;
    if (dtS > 0.25f) dtS = 0.01f;      // reject a stalled loop, do not integrate it
    // --- Speed FRUEH aufloesen (2026-10-12) ----------------------------------------
    // Die Trim- und Vertrauensgates weiter unten lesen speedMps. Der Aufrufer uebergibt
    // die Geschwindigkeit aber NICHT immer: AutosteerProcessor ruft
    // update(0.01f, steerAngleDeg) mit ZWEI Argumenten -> der Default -1.0f bleibt
    // stehen, und die Aufloesung aus der GNSS-Geschwindigkeit kam bisher erst NACH dem
    // Trim-Block. Folge im Feld: das Gate `speedMps >= vwasTrimMinSpeedMps` fiel IMMER
    // aus - der VWAS-Trim hat nie gearbeitet (Messung 10-12: vwas_trim = 0.00 in ALLEN
    // neun Logs, waehrend zo_res in 86...100 % der Frames != 0 war) UND
    // `wheelObsTrusted` war dauerhaft false, also war der Ratenvorhalt abgeschaltet
    // (B10.23 wirkte dadurch anders als gedacht). Die Aufloesung gehoert VOR die
    // Verbraucher - hier ist sie.
    if (speedMps < 0.0f) {
        speedMps = gnssProcessor.getData().speedKnots * 0.514444f;
    }
    // --- 2026-10-14: Empfaengerwert vs Weg/Zeit ------------------------------------
    // rawSpeedMps geht NACH AUSSEN (PAOGI Feld 11, roh - Betreiber-Regel "nach aussen
    // roh bei gutem Empfang") und in die unveraenderte HR-Spalte v_mps; speedMps
    // (innen) rechnet ab hier mit vFused (Weg/Zeit + EMA) - damit treffen Schritt-Tor,
    // Sehnen-Endpunkt/-Alter, Modellrate und Richtungstore die WAHRHEIT, auch wenn der
    // Doppler des Empfaengers falsch ist (gemessen 18-47: bis 2,94 m/s).
    const float rawSpeedMps = speedMps;
    if (config.speedFuseEnabled) speedMps = updateFusedSpeed(now, rawSpeedMps);
    lastSpeedMps = speedMps;
    // --- B10.66: Knicklenker OHNE Gelenksensor -> Winkel aus der Kruemmung ---------
    // Begruendung vollstaendig an den Datei-Statics oben und in knowledge/10_heading.md.
    // Die Gates sind bewusst konservativ: nur im Fahren (der Kandidat braucht Bewegung),
    // nur bei einem KLEINEN Knick NICHT (unter 0.5 Grad ist der Unterschied zur
    // VWAS-Meldung bedeutungslos), und nie, sobald ein Encoderimpuls gesehen wurde.
    const bool articAngleFromGps =
        hbIsArticulated() && (status.wheelObs.encRaw == 0) && (speedMps >= 0.5f) &&
        (fabsf(status.wheelObs.gpsAngleDeg) >= 0.5f);
    // Anzeige/Log (Spalte wheel_deg) bleibt der GEMELDETE Wert - der Betreiber soll
    // sehen, was das VWAS liefert. Physik und Knick-Korrektur rechnen mit dem wirksamen.
    const float wheelAngleUse = articAngleFromGps ? status.wheelObs.gpsAngleDeg : wheelAngleDeg;
    if (articAngleFromGps != s_artAngleFromGps) {
        s_artAngleFromGps = articAngleFromGps;
        if (articAngleFromGps && config.logEvents && !s_artAngleFromGpsLogged) {
            s_artAngleFromGpsLogged = true;
            LOG_NOTICE(EventSource::GNSS,
                       "Knicklenker ohne Gelenksensor: Knickwinkel aus der Kruemmung "
                       "(gps_a %.1f Grad) statt VWAS-Meldung (%.2f Grad)",
                       (double)status.wheelObs.gpsAngleDeg, (double)wheelAngleDeg);
        }
    }
    lastDtS = dtS;
    lastWheelAngleDeg = wheelAngleUse;

    // --- Wirksamer Lenkwinkel fuer das MODELL (2026-10-11, B7.21) --------------
    // Der VWAS setzt seinen Nullpunkt NUR ohne Guidance (ZO_BLOCKED, B7.15), kennt
    // aber den aktuellen Fehler (`residualDeg`). Die Bridge liest ihn und rechnet
    // ihr Modell mit dem WIRKSAMEN Winkel - sie schreibt nichts zurueck, speichert
    // nichts und gibt nichts an AOG/VWAS. Vorzeichen (WheelAngleFusion.cpp:1348
    // `res = est - offset`, Nullpunkt wirkt als `raw - offset`):
    //     delta_eff = delta - res
    // Nur wenn die Beobachter-Bedingungen erfuellt sind (er darf bei Schleichfahrt
    // nicht laufen: >= 0.8 m/s), und ratenbegrenzt nachgefuehrt (kein Sprung).
    {
        const Status::WheelObs& wo = status.wheelObs;
        // Der FAKT "dieser Lenkwinkel ist vertrauenswuerdig" wird einmal berechnet und von
        // ZWEI Verbrauchern gelesen: vom Trim (unten) und vom Ratenvorhalt (Ausgabe).
        // Dieselben Bedingungen wie bisher fuer den Trim - nur herausgezogen, damit sie
        // nicht an zwei Stellen auseinanderlaufen koennen.
        status.wheelObsTrusted = (wo.valid && wo.zoState != 0 && wo.fits > 0 &&
                                  speedMps >= config.vwasTrimMinSpeedMps &&
                                  fabsf(wo.residualDeg) <= config.vwasTrimMaxAbsDeg);
        float trimTarget = 0.0f;
        if (config.vwasTrimInModel && status.wheelObsTrusted) {
            trimTarget = wo.residualDeg;
        }
        const float dtUse = (dtS > 0.0f) ? dtS : 0.01f;
        const float maxStep = config.vwasTrimMaxRateDegPerS * dtUse;
        float d = trimTarget - status.vwasTrimAppliedDeg;
        if (d > maxStep)  { d =  maxStep; }
        if (d < -maxStep) { d = -maxStep; }
        status.vwasTrimAppliedDeg += d;
    }
    // PHYSIK (Vorhersage, Ratenbegrenzer, Ratenvorhalt) rechnet mit dem wirksamen
    // Winkel; ANZEIGE und Durchreiche (gemeldeter Winkel, Roll, Log-Spalte wheel_deg)
    // bleiben der gemeldete Wert - der Betreiber soll sehen, was das VWAS liefert.
    const float wheelEffDeg = wheelAngleUse - status.vwasTrimAppliedDeg;
    // R1 (2026-10-02): psi's movement in THIS tick - the measurement correction AND the
    // kinematic prediction together - is what has to stay inside the physical turn rate.
    // Captured before any measurement is applied; the clamp is at the end of the prediction.
    const float psiTickStart = psi;
    {
        float decay = dtS / 0.5f;
        if (decay > 1.0f) decay = 1.0f;
        pathRateDegS *= (1.0f - decay);
    }

    // Announce a wheelbase change once (the value is pushed in from the VWAS/AOG
    // config by the caller, so the init() line shows a default that is replaced a
    // few milliseconds later).
    if (fabsf(config.wheelbaseM - loggedWheelbaseM) > 0.005f) {
        if (loggedWheelbaseM > 0.0f && config.logEvents) {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge wheelbase %.2f -> %.2f m (VWAS/AOG config)",
                       (double)loggedWheelbaseM, (double)config.wheelbaseM);
        }
        loggedWheelbaseM = config.wheelbaseM;
    }

    // Speed: resolved at the TOP of update() since 2026-10-12 (see the note there) -
    // the trim/trust gates below need it, and the caller does not always supply it.
    const auto& d = gnssProcessor.getData();

    // --- 1. new attitude sentence? -----------------------------------------
    const uint32_t hprCount = gnssProcessor.getHprCount();
    if (hprCount != lastHprCount) {
        lastHprCount = hprCount;
        if (gnssProcessor.getHprAgeMs() <= gapThresholdMs) {
            evaluateAttitudeFrame(now);
        }
    }
    // A GAP (no sentence at all) fires no frame callback, so freshness is
    // re-derived here on every loop tick. Two consecutive rejected frames or a
    // gap longer than the measured frame period leave LIVE - see
    // consecutiveRejects and gapThresholdMs.
    const bool freshFrame = (haveTrusted &&
                             (now - lastTrustedMs) < gapThresholdMs &&
                             consecutiveRejects < 2);
    if (!freshFrame && haveTrusted && consecutiveRejects == 0 &&
        (now - lastTrustedMs) >= gapThresholdMs) {
        status.lastReject = RejectReason::GAP;   // silence, not a bad value
    }
    status.attitudeTrusted = freshFrame;
    status.everTrusted = haveTrusted;

    // --- wheel angle reliability (see Config::wheelQualityLowPct) -------------
    // The caller (AutosteerProcessor) forwards the VWAS's DOP-like number. Without a
    // figure (-1, e.g. no VWAS) the previous behaviour stays: gate on, bias off.
    if (pendingWheelQuality >= 0.0f) status.wheelAngleQuality = pendingWheelQuality;
    if (status.wheelAngleQuality < 0.0f) {
        wheelQualityOkState = true;
    } else if (wheelQualityOkState) {
        if (status.wheelAngleQuality < config.wheelQualityLowPct) wheelQualityOkState = false;
    } else {
        if (status.wheelAngleQuality > config.wheelQualityHighPct) wheelQualityOkState = true;
    }
    status.wheelQualityOk = wheelQualityOkState;
    if (!wheelQualityOkState) {
        // NO model gate while the wheel angle is unusable: the gate would damp exactly
        // the measurements the VWAS needs to learn its zero, the heading would follow
        // the wrong model and AOG would be fed a rotating heading although both
        // receivers are fine (measured 2026-09-28: -31 deg "steering" at straight
        // driving = 50 deg/s of invented rotation).
        modelGateUntilMs = 0;
        status.modelGateActive = false;
    } else if (!config.estimateSteerBias) {
        // The VWAS owns the zero again: let a bias left over from the learning phase
        // decay to zero instead of double-correcting a now correct angle.
        steerBiasDeg *= 0.995f;
        if (fabsf(steerBiasDeg) < 0.02f) steerBiasDeg = 0.0f;
    }
    status.gapThresholdMs = gapThresholdMs;
    if (!status.attitudeTrusted) prevTrustedLatch = false;   // re-entry ramp trigger

    // --- 2. COG (VTG) as an independent measurement -------------------------
    cogValid = false;
    if (gnssProcessor.getVtgAgeMs() <= config.maxCogAgeMs &&
        d.fixQuality > 0 &&
        speedMps > config.minSpeedForCogUse) {
        cogMeas = wrap360(d.headingTrue);
        cogValid = true;
        // Circular average over the last few samples. The COG is the noisiest
        // source (measured p90 6 ... 9 deg per 100 ms on the 2026-09-23 logs) and
        // it is only used while the attitude is gone, so 0.3 s of extra lag is
        // cheap - sqrt(3) less noise for the measurement that matters.
        const uint8_t nAvg = (config.cogAverageSamples > 0)
                             ? ((config.cogAverageSamples > 8) ? 8 : config.cogAverageSamples)
                             : 1;
        const float rad = cogMeas * 0.017453292519943295f;
        cogHistSin[cogHistIdx & 7] = sinf(rad);
        cogHistCos[cogHistIdx & 7] = cosf(rad);
        cogHistIdx++;
        if (cogHistCount < nAvg) cogHistCount++;
    }
    if (cogHistCount > 1) {
        float ss = 0.0f, cc = 0.0f;
        for (uint8_t i = 0; i < cogHistCount; i++) {
            const uint8_t k = (uint8_t)((cogHistIdx - 1 - i) & 7);
            ss += cogHistSin[k];
            cc += cogHistCos[k];
        }
        float m = atan2f(ss, cc) * 57.29577951308232f;
        if (m < 0.0f) m += 360.0f;
        cogMeas = m;
    }

    // --- 2b. direction of travel (forward / reverse) -------------------------
    // Has to run before any path measurement is chosen, because the path sources
    // measure the direction of TRAVEL and have to be mirrored while reversing.
    updateDirectionState(now);

    // --- 3. fix-to-fix from the position stream -----------------------------
    // Position quality is checked BEFORE anything is derived from it: a
    // fix-to-fix heading inherited a bad fix is worse than no heading at all,
    // because it looks like a measurement. See updatePositionWindow().
    updatePositionWindow(now, speedMps);

    // --- IMU heading, offset-corrected (see Config::useImuHeading) ------------
    // Learn the offset while the attitude is our reference - exactly AgOpenGPS'
    // imuGPS_Offset (Position.designer.cs:439). Nothing else about the sensor is
    // trusted: its heading is only used relative to that offset, and its noise grows
    // with the time since the last refresh.
    status.imuHeadingUsed = false;
    // Is a PATH/COG measurement a recent ABSOLUTE anchor? (operator decision 2026-09-29)
    // Without a dual antenna this is the only truth the IMU heading may be tied to, and it
    // is the same role AgOpenGPS gives its own GPS heading in the single-antenna path.
    const bool imuAnchorActive =
        (config.imuAnchorHoldMs > 0) && (imuAnchorMs != 0) &&
        ((millis() - imuAnchorMs) <= config.imuAnchorHoldMs) &&
        (lastSpeedMps >= config.imuAnchorMinSpeedMps);
    if (config.useImuHeading && imuProcessor.isActive()) {
        const IMUData& imu = imuProcessor.getCurrentData();
        const uint32_t imuAge = millis() - imu.timestamp;
        if (imu.isValid && imu.quality >= config.imuHeadingMinQuality &&
            imuAge <= config.imuHeadingMaxAgeMs) {
            const float raw = wrap360(imu.heading);
            if (!imuHdgOffsetInit) {
                // First sample: take the current heading as the offset reference.
                imuHdgOffsetDeg = wrap180(psi - raw);
                imuHdgOffsetInit = true;
                imuHdgFreshMs = millis();
                imuHdgHaveFresh = true;
            } else if (status.attitudeTrusted || imuAnchorActive) {
                // Operator decision 2026-09-29: the offset may ALSO be learned while a
                // PATH/COG measurement was accepted recently (imuAnchorMs) - that is the
                // single-antenna case, where no dual attitude exists. It is the same
                // construct AgOpenGPS uses: imuGPS_Offset learned against the GPS heading
                // (Position.designer.cs:439). Measured on the 09-29 v-r log, this is what
                // keeps the heading continuous when the magnetometer-assisted yaw cannot be
                // trusted absolutely - only its CHANGE between two GPS anchors matters.
                const float err = wrap180(wrap360(raw + imuHdgOffsetDeg) - psi);
                imuHdgOffsetDeg = wrap180(imuHdgOffsetDeg +
                                          config.imuHdgOffsetAlpha * err);
                imuHdgFreshMs = millis();
                imuHdgHaveFresh = true;
            }
            status.imuHdgOffsetDeg = imuHdgOffsetDeg;
            status.imuHeadingDeg = wrap360(raw + imuHdgOffsetDeg);
            status.imuHeadingAgeMs = imuHdgHaveFresh ? (millis() - imuHdgFreshMs) : 0xFFFFFFFFu;
            // Measurement for the fall-back path: noise starts at imuHeadingR0Deg and
            // grows with the age of the offset (drift), capped.
            float rImu = config.imuHeadingR0Deg +
                         config.imuHeadingDriftDegPerS *
                         ((float)status.imuHeadingAgeMs / 1000.0f);
            if (rImu > config.imuHeadingMaxRdeg) rImu = config.imuHeadingMaxRdeg;
            imuHeadingMeasDeg = status.imuHeadingDeg;
            imuHeadingMeasR = rImu;
        } else {
            imuHdgHaveFresh = false;
        }
    }

    // --- 4. measurement into the filter ------------------------------------
    // The forward projection runs on EVERY tick now (operator request 2026-09-24): it
    // integrates the POSITION at 100 Hz and is published whenever the receiver's fix
    // is unusable (bridges, trees, impossible steps - see the fix gate in
    // updateProjection()). Only while the attitude is not trusted does it also act as
    // a heading measurement.
    // Step A (2026-10-05): the projection is admitted as a measurement even while the attitude is
    // trusted (Config::projAlwaysMeasure). Evidence from the 2026-10-04 logs: it is the only
    // source anchored to the PATH (the attitude is anchored to the antenna baseline), its error
    // was 0.70 deg where psi was 9.40 deg off, and the old rule admitted it only during an
    // attitude LOSS - i.e. never (src=proj 0 % of 1294 frames). Its R stays adaptive and grows
    // with the along-track residual in curves (rUse *= 1 + |projLongM| / projPathM), so the
    // filter weighs it against the attitude instead of replacing it.
    const bool projAllow = config.projAlwaysMeasure || !status.attitudeTrusted;
    const bool projUsed = updateProjection(now, speedMps, dtS, projAllow);
    if (status.attitudeTrusted) {
        // evaluateAttitudeFrame() already applied the attitude measurement; with Step A the
        // projection may have added its own measurement - that is the point, the two are weighed
        // against each other by their R.
    } else if (!projUsed) {
        selectFallbackMeasurement(now);
    }

    // Step C (2026-10-05): the ring provides a path-anchored measurement at EVERY fix, over a
    // CONSTANT baseline (0.5 m RTK / 1.0 m float) instead of the re-based window. Only while the
    // attitude is trusted - the degraded phases belong to selectFallbackMeasurement() above.
    updatePathFromRing(now, speedMps);

    // --- 4b. side slip (path vs body) estimate ------------------------------
    // Only while the attitude is trusted: then psi is a body heading from the
    // baseline and the COG a path direction, and the difference is side slip plus
    // the residual mounting error. Fed into the fall-back measurements in
    // selectFallbackMeasurement(), so switching between attitude and COG no longer
    // produces a step.
    if (config.estimateSideSlip && status.attitudeTrusted &&
        speedMps > 1.0f && cogValid && p11 < 9.0f) {
        const float d = wrap180(psi - cogMeas);            // body - path
        sideSlipDeg += config.sideSlipAlpha * (d - sideSlipDeg);
        if (sideSlipDeg >  config.sideSlipLimitDeg) sideSlipDeg =  config.sideSlipLimitDeg;
        if (sideSlipDeg < -config.sideSlipLimitDeg) sideSlipDeg = -config.sideSlipLimitDeg;
        sideSlipSamples++;
    }

    // --- 5. prediction (100 Hz - the part that keeps the estimate moving) ---

    // A kinematic yaw rate needs speed. Below ~0.1 m/s it is meaningless, and
    // steering in place would fake a yaw rate, so the term is held instead.
    // The wheelbase is the AOG/vehicle.xml value (setWheelbaseM); steerScale
    // carries the VWAS-scale/slip factor, so that both are calibrated separately:
    // geometry from the vehicle data, scale from the measurement.
    float yawRateDegS = 0.0f;
    const float L = ((config.wheelbaseM > 0.5f) ? config.wheelbaseM : 2.50f) *
                    config.effWheelbaseFac;   // 2026-10-05: effektive Laenge, siehe Config
    const float deltaEff = config.steerScale * sideHillWheelDeg(wheelEffDeg) +
                           steerBiasDeg;
    // vScale (learned from the projection's along-track residual) applies to the
    // MODEL only: omega = v*tan(delta)/L, so a speed error is a direct heading-rate
    // error. The speed in PAOGI stays the receiver's own value - AgOpenGPS needs the
    // measurement, not our correction of it. PASSIVE for now: the learned scale is
    // reported (vScale, HR column 30) but the model uses the receiver's speed until
    // the estimates have been reviewed (Config::applyLearnedSpeedScale).
    const float vModel = speedMps * (config.applyLearnedSpeedScale ? vScale : 1.0f);
    // --- IMU yaw rate (operator request 2026-09-24) --------------------------
    // When enabled and a valid IMU is present, the rotation comes straight from the
    // gyro instead of the wheel-angle model. The bias is learned while the attitude
    // is trusted - the same idea as AgOpenGPS' imuGPS_Offset.
    // SAFETY (operator note 2026-09-29 "der TM171 ... funktioniert sonst nicht gut"):
    // the TM171 has NO yaw rate - IMUProcessor sets yawRate = 0 for it. With useImuRate
    // on by default that would inject a constant fake rate (0 - bias) into psi and drag
    // the heading; it only stops itself once |bias| exceeds the warning threshold. The
    // capability is therefore checked here (IMU type) instead of being assumed.
    bool imuRateActive = false;
    const bool imuRateSupported = (imuProcessor.getIMUType() != IMUType::TM171);
    if (!imuRateSupported && config.useImuRate) status.imuFlags |= 0x08;
    if (config.useImuRate && imuRateSupported && imuProcessor.isActive()) {
        const IMUData& imu = imuProcessor.getCurrentData();
        if (imu.isValid && imu.quality >= config.imuRateMinQuality) {
            const float r = imu.yawRate * (config.imuRateInvert ? -1.0f : 1.0f);
            if (fabsf(r) <= config.imuRateMaxDegS) {
                status.imuRateDegS = r;
                if ((status.attitudeTrusted || imuAnchorActive) && psiPrevValid && dtS > 0.001f) {
                    const float dPsi = wrap180(psi - psiPrevForBias) / dtS;
                    yawBiasDegS += config.yawBiasAlpha * (r - dPsi);
                    if (yawBiasDegS > 30.0f) yawBiasDegS = 30.0f;
                    if (yawBiasDegS < -30.0f) yawBiasDegS = -30.0f;
                    status.imuYawBiasDegS = yawBiasDegS;
                    // A bias of several deg/s on a stationary-grade gyro means the
                    // sign is inverted: then the learning fights itself.
                    if (!yawBiasWarned && fabsf(yawBiasDegS) > config.imuBiasWarnDegS) {
                        yawBiasWarned = true;
                        if (config.logEvents) {
                            LOG_WARNING(EventSource::GNSS,
                                        "HeadingBridge IMU yaw bias %.2f deg/s - check "
                                        "imuRateInvert (sign) and the IMU mounting; the "
                                        "gyro is NOT used until this is plausible",
                                        (double)yawBiasDegS);
                        }
                    }
                }
                // Safety: an implausible bias means the sign or the mounting is wrong.
                // Then the gyro must not drive the heading - the kinematic model does,
                // as before this feature existed.
                if (fabsf(yawBiasDegS) <= config.imuBiasWarnDegS) {
                    yawRateDegS = r - yawBiasDegS;
                    imuRateActive = true;
                }
            }
        }
    }
    status.imuRateUsed = imuRateActive;
    psiPrevForBias = psi;
    psiPrevValid = true;

    // --- P1b (operator decision 2026-09-30): the model must not invent rotation ---
    // While the wheel angle is UNTRUSTWORTHY (VWAS quality below
    // Config::wheelQualityLowPct - the zero is not learned yet, or the VWAS lost it)
    // the kinematic rate is NOT used. Measured in "Halle 2x" (2026-09-30 6-44): the
    // wheel angle read 34 deg while driving straight at 7.6 m/s, so the model
    // produced 86 deg/s and psi ran 52 deg away from the direction of travel, while
    // publishing sigma = 4.2 deg. The offline replay of that log gives
    // |psi - travel| p50 1.6 deg (p90 5.9) instead of 15.3 (53.1).
    // psi then HOLDS between measurements - honest, and the measurements correct it
    // faster because qUse below rises while the wheel is bad. The VWAS learns its
    // zero from the encoder + GPS angle, not from psi, so the zero learning that
    // Config::wheelQualityLowPct protects is not affected.
    const bool wheelRateUsable = wheelQualityOkState;
    // --- R8 (2026-10-02): the model must not work AGAINST a trusted attitude --------
    // See Config::modelAgainstAttGuard for the measurement. The rule needs no threshold on
    // the rate and no delay: if the model rate and the last ATTITUDE innovation want
    // OPPOSITE directions while that innovation is clearly above the measurement noise, the
    // model contributes nothing this tick. Zeroing yawRateDegS also removes it from the
    // output RATE LEAD (same variable), which is correct - a wrong rate must not be added
    // to the heading either.
    status.modelOpposedAtt = false;
    status.modelSuppressedDegS = 0.0f;
    // 10-15 (Schleichfahrt): die ROHE Modellrate mitschreiben, auch wenn P1b sie unten
    // verwirft. Grund: unter 0,8 m/s kann der VWAS den Lenkwinkel nicht beurteilen (wq = 0,
    // gemessen 18-59: yaw_zeroed 65 %), und dann hat die Bridge bei Schleichfahrt WEDER
    // Kurs (v < 1,0) NOCH Modell. Genau dort ist die Rate aber physikalisch klein
    // (57,3*1*tan(10 Grad)/2,1 = 4,8 Grad/s) - mit holdRateMaxDegS (3) ist sie begrenzbar.
    // B10.51: Knicklenker rechnet mit sin(gamma/2)/(L/2) statt tan(delta)/L.
    const float Lmodel = hbIsArticulated() ? hbWheelbaseM(L) : L;
    const float rateModelDegS = hbModelRateDegS(deltaEff, vModel, Lmodel);
    const float modelRateRawDegS = rateModelDegS;
    s_rateRawDegS = modelRateRawDegS;   // B10.54: Diagnose (Logspalte yr_raw)
    if (imuRateActive || ((speedMps > 0.1f) && wheelRateUsable)) {
        yawRateDegS = imuRateActive ? yawRateDegS : rateModelDegS;
        if (!imuRateActive && config.modelAgainstAttGuard && status.attitudeTrusted &&
            (status.lastMeasKind == 1) && (yawRateDegS != 0.0f) &&
            (fabsf(status.lastInnovDeg) >= config.modelAgainstAttMinInnovDeg) &&
            ((yawRateDegS > 0.0f) != (status.lastInnovDeg > 0.0f))) {
            status.modelOpposedAtt = true;
            status.modelSuppressedDegS = yawRateDegS;
            yawRateDegS = 0.0f;
        }
        psi = wrap360(psi + yawRateDegS * dtS);
    }
    status.yawRateDegS = yawRateDegS;
    status.yawRateZeroed = (!imuRateActive && (speedMps > 0.1f) && !wheelRateUsable);

    // B-Kette (2026-10-13, B10.34): der Ringpuffer-Kurs - Quelle fuer Zustand B und
    // Gegenprobe in A. Bewusst NACH der Rate: sie wird als Sehnen-Nachlauf gebraucht.
    updatePathChord(now, speedMps, yawRateDegS);
    hbUpdateChordResidual(now);   // B10.58: Diagnose-Spalte cr_resid, einmal je Fix

    // --- R1: NET rate limit on psi ---------------------------------------------
    // Why this is needed (measured 2026-10-01 20-15, critical section 20...50 m before the
    // western end): the measurement limiter bounds the MEASUREMENT's share of the move and
    // the prediction adds the MODEL's share, so in the same 100 ms psi could advance at
    // ~2x the physical rate - measured 2.76...3.52 deg per 100 ms against 11.9 deg/s
    // (1.19 deg) physical. The 3 deg output snap then turned each of those into a 4.30 deg
    // step for AgOpenGPS, which is what the operator felt as steering movement.
    // One NET bound removes both, and with R4 (Config::rateLimitAttFactor) it is as wide as
    // the truth where a measurement exists and as narrow as the model where none does.
    // A deliberate jump (cold start, re-seed) is exempt - it sets the state correctly and
    // undoing it would be a bug, not a safety.
    status.psiNetClip = false;
    if (!psiJumpedThisTick) {
        const float maxTick = kinematicRateBoundDegS() * ((dtS > 0.0f) ? dtS : 0.01f);
        const float dTick = wrap180(psi - psiTickStart);
        if ((maxTick > 0.0f) && (fabsf(dTick) > maxTick)) {
            psi = wrap360(psiTickStart + ((dTick > 0.0f) ? maxTick : -maxTick));
            status.psiNetClip = true;
            if (status.psiNetClips < 65535) status.psiNetClips++;
        }
    }
    psiJumpedThisTick = false;

    // Accumulators for the diagnostic steerScale estimate: the geometric heading
    // change that the RAW wheel angle would produce (scale 1, no bias) and its
    // DERIVATIVE column. It is compared with the innovation at the next measurement,
    // where innovation ~ (steerScale - 1) * X + offset_rad * Q for a scale/offset error
    // - the second column is what makes the wheel angle offset separable from the scale
    // (operator 2026-09-27: "die Ableitung der Heading-Ã„nderung zur Ã„nderung des
    // Lenkwinkels"), which matters because the VWAS re-trims its zero during a drive.
    // The gate gets the same increments in its own (short) window.
    // P1b (2026-09-30): not while the wheel angle is untrustworthy - then these
    // increments are the very garbage the model must not use, and the gate is off
    // anyway (see the wheelQualityOkState block in step 1).
    if (speedMps > 0.1f && wheelRateUsable) {
        // Der LERNER will den ROHEN Winkel: er soll Skala UND Nullpunkt gerade aus der
        // Differenz zur Wahrheit lernen (B8.64/B8.101) - mit dem getrimmten Winkel
        // wuerde er seinen eigenen Fehler wegdefinieren.
        const float dRadRaw = wheelAngleDeg * 0.017453292519943295f;
        const float incX = 57.29577951308232f * vModel * tanf(dRadRaw) / L * dtS;
        scaleAccX += incX;
        // Das GATE vergleicht die MESSUNG gegen das MODELL und muss deshalb mit
        // DEMSELBEN (wirksamen) Winkel rechnen wie die Vorhersage - sonst verwirft es
        // das nun richtige Modell. Gemessen 10-11 im Sim (Wanderung +6,25 Grad, Vibrations-
        // trim an): mdev p50 7,59 / mgate 684 von 1441, obwohl der Trim exakt anlag.
        const float dRadGate = wheelEffDeg * 0.017453292519943295f;
        const float incXg = 57.29577951308232f * vModel * tanf(dRadGate) / L * dtS;
        const float csW = cosf(dRadGate);
        const float sec2W = (csW * csW > 0.04f) ? (1.0f / (csW * csW)) : 25.0f;
        const float incQ = 57.29577951308232f * vModel * sec2W / L * dtS;
        gateAccX += incXg;
        gateAccQ += incQ;
        // Bound the accumulators: a long straight run accumulates nothing anyway, but a
        // long turn would otherwise dominate the estimate by sheer length. Q does not
        // vanish on a straight line, so it needs its own, much larger bound (it grows
        // with v/L per second of driving: ~53 per second at 2 m/s).
        if (scaleAccX > 60.0f) scaleAccX = 60.0f;
        if (scaleAccX < -60.0f) scaleAccX = -60.0f;
        if (gateAccX > 60.0f) gateAccX = 60.0f;
        if (gateAccX < -60.0f) gateAccX = -60.0f;
        if (gateAccQ > 400.0f) gateAccQ = 400.0f;
    }

    // Covariance prediction. The Jacobian of psi' w.r.t. the bias is
    // dt * v / (L * cos^2(delta)) (radians per radian) - the only coupling
    // between the two states. qHeadingDegS describes the model error while
    // DRIVING; at standstill the model is exact (a standing machine does not yaw),
    // so a much smaller q is used - otherwise the covariance would grow for no
    // reason and the return to LIVE would have to re-learn a heading it never lost.
    float qUse = imuRateActive
                 ? config.imuQHeadingDegS
                 : ((speedMps > config.minSpeedForMeasurement)
                    ? config.qHeadingDegS : 0.5f);
    // P1b (2026-09-30): with an unusable wheel angle the MODEL is the error source,
    // so its process noise must say "unknown". This makes sigma honest for the
    // consumers and raises the KF gain, i.e. psi follows the ACCEPTED measurements
    // instead of the (zeroed) model. Reversible: each accepted measurement shrinks
    // p11 again, so the normal tuning returns as soon as the VWAS quality is back.
    if (!imuRateActive && !wheelRateUsable) {
        qUse = config.qHeadingWheelBadDegS;
    }
    if (config.estimateSteerBias || !wheelQualityOkState) {
        const float delta = (sideHillWheelDeg(wheelEffDeg) + steerBiasDeg) * 0.017453292519943295f;
        const float c = cosf(delta);
        const float cc = (c * c) > 0.05f ? (c * c) : 0.05f;
        const float dfdb = (speedMps > 0.1f) ? (dtS * speedMps / (L * cc)) : 0.0f;
        const float n11 = p11 + 2.0f * dfdb * p12 + dfdb * dfdb * p22 +
                          qUse * qUse * dtS;
        const float n12 = p12 + dfdb * p22;
        const float n22 = p22 + config.qSteerBiasDegS * config.qSteerBiasDegS * dtS;
        p11 = n11; p12 = n12; p22 = n22;
        // Cap the heading variance at "unknown but recoverable" - an unbounded
        // random walk would make the return to LIVE slow.
        if (p11 > 900.0f) p11 = 900.0f;
        const float p22Max = config.steerBiasLimitDeg * config.steerBiasLimitDeg;
        if (p22 > p22Max) p22 = p22Max;
    } else {
        p11 += qUse * qUse * dtS;
        if (p11 > 900.0f) p11 = 900.0f;
    }

    // --- 6. outputs ---------------------------------------------------------
    odometerM += vModel * dtS;
    status.vehicleHeadingDeg = psi;
    // --- status mirrors for the consumers (VWAS, NAVProcessor, web UI, HR log) ----
    // Operator decision 2026-09-29: the state machine no longer hangs off "a dual-antenna
    // frame was trusted", so consumers need to see the difference between a real BODY
    // attitude and a heading that is anchored to the GPS path.
    status.haveAnyHeading = haveAnyHeadingLatch;
    status.imuFlags = (uint8_t)(status.imuFlags & 0x08);   // keep "rate not supported"
    if (imuProcessor.isActive()) {
        const IMUData& imu = imuProcessor.getCurrentData();
        const uint32_t imuAge = millis() - imu.timestamp;
        if (imu.isValid && imu.quality >= config.imuHeadingMinQuality &&
            imuAge <= config.imuHeadingMaxAgeMs) status.imuFlags |= 0x01;
    }
    if (status.imuHeadingUsed) status.imuFlags |= 0x02;
    if (status.imuRateUsed)    status.imuFlags |= 0x04;
    // --- what AgOpenGPS sees as the POSITION this tick ------------------------
    // Operator correction 2026-09-29 ("der Traktor springt auf der Karte"): the caller
    // substitutes the projection only while the fall-back/ramp runs, otherwise the
    // receiver's own position is published. Tracking exactly that point makes the step
    // between two published points measurable (HR: psi_att/publim/pub_step), so the next
    // log can prove that no jump reached AgOpenGPS.
    {
        bool projPublished = (status.posFallbackActive || status.posRampActive) && projValid;
        // A+B3 (2026-10-10): die Projektion darf den Empfaenger nie weiter als 0.5 m verlassen.
        // Beleg: Spruenge 30.0 m (10-09) und 44.1 / 46.3 m (10-10), ALLE bei publim 1->0 mit fixQ=4,
        // 20-26 Sats, pos_sigma 0.01 m - also bei einwandfreiem Empfaenger. Die Projektion war mit
        // falschem Heading weggelaufen (unter Bruecke/Baum faltet die Lage 150-180 Grad herum).
        // Wenn der Empfaenger eine gueltige Position hat, ist ER die Wahrheit.
        if (projPublished && d.hasPosition && d.fixQuality > 0) {
            double devN = 0.0, devE = 0.0;
            localDeltaMeters(projLat, projLon, d.latitude, d.longitude, devN, devE);
            if (sqrt(devN * devN + devE * devE) > (0.3 + 0.15 * speedMps)) projPublished = false;
        }
        const double newLat = projPublished ? projLat : d.latitude;
        const double newLon = projPublished ? projLon : d.longitude;
        const bool haveNew = projPublished || (d.hasPosition && d.fixQuality > 0);
        // Der Schritt der VEROEFFENTLICHTEN Position wird NICHT mehr hier gerechnet,
        // sondern in logRawFrame() - Begruendung dort (er kam aus dieser Stelle nie
        // im Log an). Hier bleibt nur, was der restliche Code braucht: der Punkt selbst.
        if (haveNew) {
            pubLat = newLat; pubLon = newLon; pubValid = true;
        }
        // Per tick: is the published position substituted (fall-back or catch-up ramp)?
        status.posLimited = projPublished;
        // Der Fix-zu-Fix-Schritt des Empfaengers wird ebenfalls in logRawFrame()
        // gebildet - aus d.latitude/longitude gegen den Stand der vorigen Log-Zeile.
        // Die frueheren Felder lastFixLat/lastFixLon/lastFixValid sind entfallen.
        // W2 (2026-10-02): what AgOpenGPS really receives is NOT gnssData.latitude - it is
        // gnssData.latitudeNMEA/longitudeNMEA (NAVProcessor.cpp:288), the receiver's own
        // ddmm.mmmm value, written by THREE parser paths. This log only ever carried
        // latitude/longitude, so a divergence between the two was invisible in every log up
        // to now while AOG would see the jump. Log the distance between them.
        //   -1 = no NMEA coordinate cached yet (first frames / receiver silent)
        {
            const double laN = (double)d.latitudeNMEA;
            const double loN = (double)d.longitudeNMEA;
            if ((laN > 1.0) && (loN > 1.0)) {
                const double laDeg = floor(laN / 100.0);
                double laDec = laDeg + (laN - laDeg * 100.0) / 60.0;
                if (d.latDir == 'S') laDec = -laDec;
                const double loDeg = floor(loN / 100.0);
                double loDec = loDeg + (loN - loDeg * 100.0) / 60.0;
                if (d.lonDir == 'W') loDec = -loDec;
                double nN = 0.0, nE = 0.0;
                localDeltaMeters(d.latitude, d.longitude, laDec, loDec, nN, nE);
                status.nmeaDiffM = (float)sqrt(nN * nN + nE * nE);
            } else {
                status.nmeaDiffM = -1.0f;
            }
        }
    }
    // Inverse of the same conversion AgOpenGPS performs on what it receives:
    // AOG does body = received + offset, so the firmware has to publish
    // received = body - offset. In normal operation (psi = az_received + offset)
    // this is exactly the receiver's own azimuth again - a pass-through, with the
    // bridge only replacing it while the attitude is gone.
    //
    // COLD START (operator correction 2026-09-29): before the FIRST trusted attitude frame
    // psi is dead reckoning from an unknown initial value, so publishing it produced a
    // 66...119 deg step in all three 2026-09-29 logs the moment the first attitude arrived
    // (measured t = 17.8 s) - and AgOpenGPS' own reverse detection compares exactly this
    // value with the direction of travel, so such a step can latch it. While a usable
    // receiver azimuth exists and no frame has been trusted yet, the receiver's own value
    // is passed through instead - that is what normal operation publishes anyway.
    const bool qfOk = (d.headingQuality < 16) &&
                      ((config.headingQualityMask & (1u << d.headingQuality)) != 0);
    // ---- LIVE-Gate (2026-10-05, Betreiber-Entscheidung) ---------------------
    // Bis hierher galt der Durchgriff des Empfaenger-Azimuts NUR vor dem ersten
    // vertrauenswuerdigen Frame (Kaltstart). Danach wurde immer der Filterzustand psi
    // gesendet - auch dann, wenn der Empfaenger die besseren Zahlen liefert. Genau das
    // ist der gemessene 9,6-Grad-Versatz auf den Geradeaus-Abschnitten.
    // Jetzt: solange der Empfaenger nachweislich sauber ist (eigenes 1-sigma von Heading
    // und Pitch/Roll, Baseline-Laenge, Roll-Huelle, RTK-fix), wird SEIN Azimut live
    // gesendet. Das ist dieselbe Groesse, die der Kaltstart schon verwendet, nur mit
    // Qualitaetsbedingung statt Zeitbedingung.
    // Hysterese: rein bei <= InSd, raus erst bei > OutSd (Zustand als Funktions-Statik -
    // das Modul ist ein Singleton; der Zustand ist im Log an "az_out == att_hdg_deg"
    // erkennbar, es wird deshalb KEINE Log-Spalte geaendert).
    static bool liveGateOn = false;
    static uint16_t liveGoodRun = 0, liveBadRun = 0, liveRampMsLeft = 0;
    // RTK-freie Qualitaet (Betreiber 2026-10-05: "gute Daten koennen auch float oder nur
    // GPS sein, denn gutes Heading und Roll kann auch ohne RTK klappen" = relative
    // Geometrie). fixQuality ist deshalb KEIN Kriterium mehr.
    const bool qualGood = d.hasDualHeading && qfOk && d.uniSolValid &&
                          (fabsf(d.dualRoll) <= config.maxRollAbsDeg) &&
                          (fabsf(status.baselineDevPct) <= config.liveGateBaselineTolPct) &&
                          (d.uniHdgStdDevDeg <= config.liveGateInSdDeg) &&
                          (d.uniPtchStdDevDeg <= config.liveGateInPtchSdDeg);
    // Einigkeit mit der Projektion: travelWitnessDeg ist die Richtung der Fahrt aus den
    // Positionen, also der Kurs, den die Projektion weitergefuehrt hat. ACHTUNG Bezugssystem:
    // das Empfaenger-Azimut (dualHeading) steht in der AOG-Achse, die Re-Seed-Pruefung
    // vergleicht dagegen Werte in der psi-Achse - welches der beiden hier gilt, wird auf
    // der ersten Fahrt mit den Log-Spalten travel_wit / az_out geprueft. Bis dahin wird
    // BEIDES akzeptiert (kein Falsch-Blockieren des Gates; ein ~88 Grad falscher Kurs
    // rutscht dadurch allerdings durch, siehe Protokoll).
    bool agreeOk = true;
    if (status.travelWitnessValid && (config.reentryAgreeDeg > 0.0f)) {
        const float dRaw = fabsf(wrap180(d.dualHeading - status.travelWitnessDeg));
        const float dPsi = fabsf(wrap180(wrap360(d.dualHeading + config.dualHeadingOffsetDeg) -
                                         status.travelWitnessDeg));
        agreeOk = (dRaw <= config.reentryAgreeDeg) || (dPsi <= config.reentryAgreeDeg);
    }
    if (config.liveGateEnabled) {
        if (qualGood) { if (liveGoodRun < 30000) liveGoodRun++; } else { liveGoodRun = 0; }
        if (!qualGood) { if (liveBadRun < 30000) liveBadRun++; } else { liveBadRun = 0; }
        if (!liveGateOn) {
            // Harter Start: anhaltend gut (reentryGoodFrames) UND einig mit der Projektion.
            // Gemessen: beim Wiedereintritt meldet der Empfaenger sd 0,4...0,8 Grad,
            // Baseline ~0 % und qf=4 - und liegt trotzdem 5...15 Grad (extrem 70...100
            // Grad) neben dem Kurs. Nur die Einigkeit faengt das.
            if ((liveGoodRun >= config.reentryGoodFrames) && agreeOk) {
                liveGateOn = true;
                liveGoodRun = 0; liveBadRun = 0;
                liveRampMsLeft = config.liveGateRampMs;
            }
        } else {
            // Gedaempftes Schliessen: erst nach gateBadFrames durchgehend schlechten
            // Epochen (gemessen: die Haelfte der "Wiedereintritte" sind 1-Frame-Aussetzer,
            // die ein Sofort-Schliessen zu einem Flackern machen) und dann nur, wenn es
            // hart schlecht ist oder die Messung ganz fehlt.
            const bool hardBad = (d.uniHdgStdDevDeg > config.liveGateOutSdDeg) ||
                                 (d.uniPtchStdDevDeg > config.liveGateOutPtchSdDeg) ||
                                 !d.hasDualHeading || !qfOk || !d.uniSolValid;
            if (hardBad && (liveBadRun >= config.gateBadFrames)) {
                // B (2026-10-07): auch beim SCHLIESSEN des Gates wird die Umbasierung
                // gerampt. Vorher ging der Wert hart von Empfaenger-Azimut auf psi
                // (gemessen 34 Grad in einem Frame: |az_out-att| p90 = 34,3 im Log 6-13).
                liveGateOn = false; liveGoodRun = 0; liveBadRun = 0;
                liveRampMsLeft = config.liveGateRampMs;
            }
        }
    } else {
        liveGateOn = false; liveGoodRun = 0; liveBadRun = 0; liveRampMsLeft = 0;
    }
    if (liveRampMsLeft > 0) {
        const uint16_t dec = (uint16_t)((dtS > 0.0f ? dtS : 0.01f) * 1000.0f);
        liveRampMsLeft = (liveRampMsLeft > dec) ? (uint16_t)(liveRampMsLeft - dec) : 0;
    }
    // A+B2 (2026-10-10): Die Ausgabe nimmt IMMER die Dual-Lage, wenn eine da ist und die
    // Fix-Qualitaet stimmt - auch bei geschlossenem Gate. Beleg aus 10-09/10-10:
    //   1) Gate schliesst -> psi wird publiziert -> psi FOLGT EINER ECHTEN 90-GRAD-KURVE NICHT
    //      (176 Grad wahre Drehung, nur 95 Grad publiziert, out_lim=0 = KEIN Begrenzer im Spiel)
    //   2) die Dual-Lage folgte jeder Kurve, auch als ihre Flags (uni_hdgsd) schlecht sagten.
    //   3) psi war 6-24 Grad von der Fahrtrichtung weg (Schraeglauf).
    // Restrisiko: eine eingefrorene Lage wird dann publiziert - dafuer kommt der Positionskurs
    // als Plausibilitaets-Tor und die Einfrier-Erkennung im naechsten Build.
    // A+B4 (2026-10-10): kinematisches Tor auf die LAGE. Unter Baeumen/Bruecke faltet die
    // Dual-Lage um 150-180 Grad in Sekunden (physikalisch unmoeglich, gemessen 10-10) und wurde
    // mit A+B2 direkt publiziert. Ein Sprung ueber der physikalisch moeglichen Rate => verwerfen.
    static float lastAttGateDeg = 0.0f;
    static bool  lastAttGateValid = false;

    // --- L1 (2026-11-06): GEMESSENE Lagrate fuer das HALTEN -----------------------------
    // Der Schleichfahrt-Fix unten fuehrt die Ausgabe im HALTEN mit der MODELLrate weiter.
    // Fuer eine Maschine ohne Lenkwinkel-Messung ist die Modellrate aber ~0 (der Knickwinkel
    // fehlt im Modell) - die Ausgabe fror ein, waehrend die Karosserie drehte. Messung
    // 2026-11-06 (Knicklenker-Testmaeher, RTK, Lage sauber): 50...180 Grad Fehler in der
    // Wende, 44 % der Kurven-Frames standen auf "halten".
    // Die Drehrate der Dual-Antenne IST eine Messung und schlaegt den Zustand - dieselbe
    // Hierarchie wie im ganzen Modul. Sie wird verworfen, wenn sie ein Sprung ist
    // (attJumped) oder unplausibel gross.
    static float lastDualHdgDeg = 0.0f;
    bool attJumped = false;
    if (d.hasDualHeading && lastAttGateValid) {
        const float jumpDeg = fabsf(wrap180(d.dualHeading - lastAttGateDeg));
        float dtUse = dtS; if (dtUse < 0.05f) dtUse = 0.05f;   // Zeitboden: Frames sind 20...100 ms
        const float limRate = 30.0f + 25.0f * speedMps;       // auch 60 km/h (16.7 m/s -> 448 Grad/s)
        if (jumpDeg > limRate * dtUse + 5.0f) attJumped = true;
        // Nur eine NICHT gesprungene Lage wird zur neuen Referenz - sonst wuerde der Falter
        // (150-180 Grad unter Baum/Bruecke, A+B4) im naechsten Frame als "normal" gelten.
        if (!attJumped) { lastAttGateDeg = d.dualHeading; }
        lastAttGateValid = true;
        // + Toleranz: Lage jittert 2-6 Grad/Frame
    }
    float attRateDegS = 0.0f;
    bool  attRateValid = false;
    // 2026-11-06: Rate ueber die ZEIT SEIT DER LETZTEN AENDERUNG, nicht ueber die
    // Aufrufperiode. Die Lagprobe kommt mit der Empfaengerrate (5/10/20 Hz), der Aufruf kann
    // schneller sein - dann waere dieselbe Probe mehrfach gezaehlt und die Rate zu gross.
    static float    s_attRateDegS = 0.0f;
    static uint32_t s_attRateMs   = 0u;
    static uint32_t lastDualHdgMs = 0u;
    if (config.holdRateFromAttitude && d.hasDualHeading) {
        const float delta = wrap180(d.dualHeading - lastDualHdgDeg);
        const uint32_t dtMs = now - lastDualHdgMs;
        if ((fabsf(delta) > 0.01f) && (dtMs >= 5u)) {
            const float r = delta * 1000.0f / (float)dtMs;
            if (fabsf(r) <= HB_ATT_RATE_MAX_DEG_S) { s_attRateDegS = r; s_attRateMs = now; }
            lastDualHdgDeg = d.dualHeading;
            lastDualHdgMs  = now;
        }
        // Der Wert gilt als Rate, solange die letzte Aenderung nicht zu alt ist.
        if ((s_attRateMs != 0u) && ((uint32_t)(now - s_attRateMs) <= 400u)) {
            attRateDegS = s_attRateDegS;
            attRateValid = true;
        }
    }

    // --- 2026-11-06: KNICKWINKEL AUS (Kurs - Lage) --------------------------------------
    // Fuer einen Punkt d vor der Vorderachse auf dem Vorderrahmen gilt rein geometrisch
    //     tan(att - cog) = d * tan(gamma/2) / (L/2)
    // umgestellt:  gamma = 2 * atan( (L/2) * tan(beta) / d )  mit beta = cog - (att + Offset)
    // Beide Groessen sind direkte Messungen (VTG-Kurs + Dual-Lage), es wird NICHT differenziert.
    // Damit bekommt die Kette ihren fehlenden Eingang, ohne dass ein Lenkwinkel-Sensor noetig
    // ist. Vorzeichen: gamma > 0 = nach RECHTS geknickt (so erwartet es AOG).
    float artEstDeg   = 0.0f;
    bool  artEstValid = false;
    if (config.artEstFromCourse && hbIsArticulated() && d.hasDualHeading &&
        (speedMps > 0.5f) && (d.fixQuality >= 2)) {
        const float beta  = wrap180(d.headingTrue - (d.dualHeading + config.dualHeadingOffsetDeg));
        const float halfL = 0.5f * hbWheelbaseM(2.5f);
        const float dM    = config.artEstAntennaLeadM;
        if ((dM > 0.05f) && (fabsf(beta) < 80.0f)) {
            const float g = 2.0f * atanf((halfL * tanf(beta * 0.0174532925f)) / dM) * 57.2957795f;
            if (fabsf(g) <= config.artEstMaxDeg) {
                // dt-korrekter Tiefpass + physikalische Ratenbegrenzung: roh schwankt gamma um
                // +-10 Grad (2 Grad beta-Rauschen wird bei gamma=0 mit Faktor ~2,2 verstaerkt).
                // Fuer die AOG-Darstellung zaehlt: kein Zappeln, keine Spruenge.
                if (!s_artEstValid) {
                    s_artEstDeg = g;
                } else {
                    const float a = (dtS > 0.0f) ? (dtS / (config.artEstTauS + dtS)) : 0.2f;
                    const float maxStep = config.artEstMaxRateDegS * ((dtS > 0.0f) ? dtS : 0.1f);
                    const float delta = (g - s_artEstDeg) * a;
                    const float lim   = (delta > maxStep) ? maxStep : ((delta < -maxStep) ? -maxStep : delta);
                    s_artEstDeg += lim;
                }
                s_artEstValid = true;
                artEstDeg   = s_artEstDeg;
                artEstValid = true;
            }
        }
    }
    if ((speedMps <= 0.5f) || !d.hasDualHeading) { s_artEstValid = false; }
    // 2026-11-06e: ANGEWANDTER Knickwinkel gamma/2 - ratenbegrenzt. Das ist der Wert, der ins
    // Heading (Mittelachse fuer AOG) und nach PGN 253 geht. Begruendung und Messung siehe
    // Config::artEstApplyRateDegS: der rohe Schaetzwert springt zwischen 0,53 und +-33 Grad
    // (Gueltigkeitswechsel + empfindlicher Hebel), und jeder Sprung landet 1:1 in der Ausgabe.
    {
        const float target = artEstValid ? (0.5f * artEstDeg)
                                         : hbArticulationHalfDeg(lastWheelAngleDeg);
        float dHalf = wrap180(target - s_artHalfDeg);
        const float maxHalf = config.artEstApplyRateDegS * ((dtS > 0.0f) ? dtS : 0.01f);
        if (dHalf >  maxHalf) { dHalf =  maxHalf; }
        if (dHalf < -maxHalf) { dHalf = -maxHalf; }
        s_artHalfDeg = wrap180(s_artHalfDeg + dHalf);
    }

    // 1a+1e (2026-10-10): Traeger-Kette statt psi. Lage (plausibel) -> FAHRTRICHTUNG aus den
    // POSITIONEN (status.travelWitnessDeg/travelWitnessValid = 1a, positionabgeleitet und damit
    // unabhaengig von der Dual-Lage) -> psi nur, wenn je eine Messung akzeptiert wurde
    // (haveAnyHeadingLatch = 1e) -> sonst den letzten Wert halten (status.azimuthDeg).
    // Beleg: Positionskurs trug echte Luecken mit 0.10-1.11 Grad gegen 13.9-29 Grad beim Modell.
    // 1.1 (10-11, B10.9): `psi` faellt als AUSGABE weg - der Zustand ist kein Messwert, er wird
    // nur noch zum Ankern der Messungen benutzt. Faellt die Lage weg und gibt es keinen Kurs
    // (Schleichfahrt unter der Witness-Schwelle), wird der letzte publizierte Wert GEHALTEN:
    // das ist strikt besser als ein Zustand, der 11...21 Grad daneben liegt (B10.3).
    const bool attOk = d.hasDualHeading && qfOk && !attJumped;
    // (1.1, 10-11) kein psiOk mehr: psi ist keine Ausgabe mehr (B10.9).
    const float outSmoothPrevDeg = status.outSmoothDeg; // A10: Zustand vor dieser Frame
    // B10.38 (10-13): die Ausgabe folgt der Lage nur, wenn AUCH DER FILTER sie vertraut. Vorher
    // entschied das per-Frame-Tor `attOk` allein - in BRIDGE/COAST (attitudeTrusted = false) ist
    // attOk aber meist true, also fuehrte genau die Lage, die der Filter eben verworfen hatte
    // (gemessen 12-38: 47 % der B-Frames > 2 Grad falsch, B-Kette wirkungslos, obwohl der
    // Ringpuffer-Kurs in 520 von 554 Fallback-Frames gueltig war). Ausnahme INIT: beim
    // Kaltstart gibt es noch kein Filterurteil - dort bleibt die Lage erlaubt (sonst
    // verzoegert sich der erste veroeffentlichte Wert).

    const bool lageFuehrt = attOk && (status.attitudeTrusted || status.state == State::INIT);
    // --- Schleichfahrt (2026-10-15, Betreiber: "Schleichfahrt fix") -------------------
    // Kein Kurs verfuegbar (Sehne UND Witness ungueltig - beide haben die 1,0-m/s-Schwelle):
    // dann laeuft die Ausgabe mit der MODELLRATE weiter statt einzufrieren. Bedingungen:
    // Modellrate vertrauenswuerdig (kein yawRateZeroed), langsam (|rate| <= holdRateMaxDegS,
    // sonst ist sie Rauschen), in Fahrt (v > 0.1) und NICHT rueckwaerts (das Modell rechnet
    // mit |v|, dort waere das Vorzeichen falsch). Sonst bleibt es beim HALTEN.
    // 2026-11-06b: `keinKurs` (kein Sehnen- und kein Witness-Kurs) entfaellt hier. Es war die
    // Bedingung fuer das Weiterfuehren mit der MODELLRATE; seit L2 fuehrt der Halte-Block
    // IMMER, und die Hierarchie steckt im Ratenwert selbst: gemessene Lagrate
    // (attRateValid), sonst Modellrate mit ihrem engen Deckel holdRateMaxDegS.
    // 2026-11-06: EIGENER Integrator fuer das HALTEN. Vorher wurde `holdAz` jeden Frame aus
    // `status.azimuthDeg` neu gesetzt - das ist die bereits BEGRENZTE Ausgabe. Weil der
    // Halte-Zweig nicht in jedem Frame laeuft (die Zustaende flattern LIVE>BRIDGE>LIVE... und
    // die Quellenbestaetigung haelt die Ausgabe auf 3), wurde der aufintegrierte Weg immer
    // wieder verworfen: gemessen 2,7 Grad/s Ausgabe gegen 42 Grad/s echte Drehung (94 % weg).
    static float  s_holdAzInt   = 0.0f;
    static bool   s_holdAzValid = false;
    float holdAz = status.azimuthDeg;
    {
        // 2026-11-06b: IMMER fuehren, nicht nur bei !lageFuehrt. Wird die Lage wieder vertraut,
        // entscheidet die 1-s-QUELLENBESTAETIGUNG (sourceConfirmMs) - solange wird `use = 3`
        // (halten) PUBLIZIERT, obwohl dieser Block nicht lief. holdAz war dann status.azimuthDeg,
        // also der eingefrorene Ausgang selbst: die Ausgabe stand bis zu 1 s still, waehrend die
        // Maschine weiterdrehte (gemessen im Sim, Ref-Log knick_125_241, t = 172,5...173,1 s:
        // az_out konstant 57,20 Grad bei 60 Grad/s echter Drehung).
        // Neu aufgezogen wird nur, wenn die VORIGE Ausgabe aus einer echten Quelle kam
        // (Lage/Ring/Witness) - dann startet der Halt sprungfrei von der Ausgabe.
        if (!s_holdAzValid || (status.azSrc != 3u)) { s_holdAzInt = status.azimuthDeg; s_holdAzValid = true; }
        // BEWUSST ohne !status.yawRateZeroed: bei Schleichfahrt ist die Qualitaet immer 0
        // (der VWAS urteilt erst ab 0,8 m/s), die Rate aber physikalisch klein. Der
        // BETRAG ist die Sicherung (holdRateMaxDegS), nicht das Qualitaetsurteil.
        const float rateModel = modelRateRawDegS;
        float rateHold = attRateValid ? attRateDegS : rateModel;
        // 2026-11-06b: VORZEICHEN-ALTERNANZ daempfen. Gemessen (knick_125_241, t = 208...217 s,
        // enge Wende bei 0,5...1,5 m/s): die Lagrate springt im 100-ms-Takt zwischen -164 und
        // +132 Grad/s, waehrend die Maschine monoton durch 100 Grad dreht - der Integrator hebt
        // sich damit selbst auf, die Ausgabe steht (34 von 142 Halten-Kurvenframes).
        if (config.holdRateTauS > 0.0f) {
            static float s_holdRateFilt = 0.0f;
            float aF = dtS / (config.holdRateTauS + dtS);
            if (aF > 1.0f) { aF = 1.0f; }
            s_holdRateFilt += (rateHold - s_holdRateFilt) * aF;
            rateHold = s_holdRateFilt;
        }
        // L1 (2026-11-06): Obergrenze PHYSIKALISCH - keine Maschine dreht schneller als
        // v / R_min. (Testmaeher: R_min 0,6 m -> bei 1 m/s sind das 95 Grad/s; der alte
        // Deckel holdRateMaxDegS = 3 Grad/s haette das Weiterfuehren sofort gesperrt und
        // genau damit die Ausgabe in der Wende eingefroren.) Die Modellrate behaelt den
        // alten, engen Deckel: sie ist ein Zustand, keine Messung.
        // L2 (2026-11-06b): KNICKLENKER - der Vorderrahmen dreht beim Einschlag ZUSAETZLICH
        // (gemessen bis 164 Grad/s bei 1,4 m/s, v/R_min ergaebe nur 138), daher der Zuschlag
        // holdRateSlackDegS. Und der Deckel wird GEKLEMMT statt die Integration zu VERWERFEN:
        // verworfen hiess "Ausgabe steht" (gemessen 20 der 34 stehenden Frames: Rate 100...164
        // gegen Deckel 60...70 bei v < 0,8).
        const float rateCap = attRateValid
                            ? (speedMps / config.holdMinTurnRadiusM * 57.29578f +
                               config.holdRateSlackDegS)
                            : config.holdRateMaxDegS;
        if (!status.reverseDirection && (speedMps > 0.1f)) {
            float rUse = rateHold;
            if (rUse >  rateCap) { rUse =  rateCap; }
            if (rUse < -rateCap) { rUse = -rateCap; }
            s_holdAzInt = wrap360(s_holdAzInt + rUse * dtS);
        }
        holdAz = s_holdAzInt;
    }
    // --- 1-s-BESTAETIGUNG der Quelle (2026-10-18, Betreiber) -------------------------
    // Kandidat wie bisher; uebernommen wird er erst, wenn er sourceConfirmMs lang
    // ununterbrochen derselbe ist ("die ersten Werte nach Rueckkehr vom Empfang sind
    // meist nicht gut"). Solange bleibt die BISHERIGE Quelle aktiv; ist die nicht mehr
    // verfuegbar, wird gehalten (az_src = 3, mit Modellrate aus dem Schleichfahrt-Fix).
    const uint8_t cand = lageFuehrt ? 0u
                       : (status.pathChordValid ? 1u : (status.travelWitnessValid ? 2u : 3u));
    if (cand == status.azSrc) {
        srcPendingMs = 0;
    } else if (srcPendingMs == 0u) {
        srcPendingMs = now;
    }
    uint8_t use = status.azSrc;
    if ((srcPendingMs != 0u) && ((uint32_t)(now - srcPendingMs) >= config.sourceConfirmMs)) {
        use = cand;
        srcPendingMs = 0;
    }
    float azUse = wrap360(holdAz);
    bool azHave = false;
    if ((use == 0u) && lageFuehrt) {
        // B10.47 (10-18): HIER stand kurz ein "Loesungs-Flip"-Tor (45 Grad gegen den
        // letzten Ausgang, 1,5 s Bestaetigung). Es ist wieder ENTFERNT, weil die
        // Feldmessung das Gegenteil zeigte: die 170-Grad-Aenderung bei t = 30,8 s war
        // eine ECHTE Wende am Brueckenende (40 -> 211 Grad in 4,4 s = 39 Grad/s, und die
        // Lage meldete danach dauerhaft 1,0 Grad 1-sigma) - das Tor hat eine richtige
        // Richtung 7 s lang gesperrt (Ausgabe fror bei 40,8 Grad ein, 160 Grad falsch).
        // Eine selbstkonsistente 180-Grad-Spiegelung ist von einer echten Wende mit den
        // vorhandenen Signalen NICHT zu unterscheiden - also nicht raten.
        // B10.51: Beim Knicklenker messen die Antennen den VORDERRAHMEN; AOG erwartet die
        // Mittelachse -> gamma/2 addieren. gamma kommt aus dem GELENKWINKEL (echter Sensor
        // direkt; beim virtuellen Keya-Weg der gelernte Encoder-Winkel des VWAS) oder aus der
        // Schaetzung. s_artHalfDeg wird OBEN einmal pro Frame ratenbegrenzt gefuehrt
        // (2026-11-06e) - hier NICHT mehr direkt zuweisen, sonst ist die Begrenzung wirkungslos.
        // Vorzeichen (2026-11-06): AOG zeichnet das VORDERteil bei "heading - gamma/2"
        // (CVehicle.cs), die Mittelachse liegt also bei att + gamma/2. Mit einem Minus stand
        // die Ausgabe um gamma (bis 75 Grad) daneben - das war der Schraeglauf.
        azUse = wrap360(d.dualHeading + s_artHalfDeg); azHave = true;
    } else if ((use == 1u) && status.pathChordValid) {
        azUse = wrap360(status.pathChordDeg - config.dualHeadingOffsetDeg); azHave = true;
    } else if ((use == 2u) && status.travelWitnessValid) {
        azUse = wrap360(status.travelWitnessDeg - config.dualHeadingOffsetDeg); azHave = true;
    }
    if (!azHave) { use = 3u; azUse = wrap360(holdAz); }
    // B10.50 (2026-10-25): Rueckkehr aus dem HALTEN wird gerampt. Das Halten ist die Quelle,
    // die DRIFTET (gemessen 7-50: 13,6 Grad in 4 s, max 41,6 Grad); die erste gute Quelle
    // danach war 20...33 Grad entfernt und ging als Sprung (bis 140 Grad/s) heraus.
    // Die Rampe ist die vorhandene LIVE-Rampe (liveRampDegPerS, 20 Grad/s).
    if ((status.azSrc == 3u) && (use != 3u)) {
        liveRampMsLeft = kHoldReturnRampMs;
    }
    status.azSrc = use;
    // --- Gleitender Durchschnitt auf der Quelle (2026-10-18, Vergleichsreihe) -------
    float azSrcVal = azUse;
    if (config.outMaMs > 0u) {
        outMaBuf[outMaIdx] = azUse;
        outMaIdx = (uint8_t)((outMaIdx + 1u) % OutMaN);
        if (outMaCount < OutMaN) { outMaCount++; }
        // Daempfung nach Einschlag (Betreiber 10-18): Fenster linear ausblenden.
        const float awMa = fabsf(lastWheelAngleDeg - status.vwasTrimAppliedDeg);
        float maMsEff = (float)config.outMaMs;
        if (awMa >= config.outMaWheelHiDeg) {
            maMsEff = 0.0f;
        } else if (awMa > config.outMaWheelLoDeg) {
            const float tMa = (awMa - config.outMaWheelLoDeg) /
                              ((config.outMaWheelHiDeg > config.outMaWheelLoDeg + 0.1f)
                               ? (config.outMaWheelHiDeg - config.outMaWheelLoDeg) : 0.1f);
            maMsEff *= (1.0f - tMa);
        }
        // 2026-11-06: dieselbe Daempfung auf der GEMESSENEN DREHRATE (Begruendung und Messreihe
        // siehe Config::outMaRateLoDegS). Ohne Lenkwinkelsensor ist awMa immer ~0, das Fenster
        // steht also in der Kurve voll - genau dort kostet es die Kurvenantwort.
        {
            const float rateMa = fabsf(attRateDegS);
            if (rateMa >= config.outMaRateHiDegS) {
                maMsEff = 0.0f;
            } else if (rateMa > config.outMaRateLoDegS) {
                const float tMa2 = (rateMa - config.outMaRateLoDegS) /
                                   ((config.outMaRateHiDegS > config.outMaRateLoDegS + 0.1f)
                                    ? (config.outMaRateHiDegS - config.outMaRateLoDegS) : 0.1f);
                maMsEff *= (1.0f - tMa2);
            }
        }
        uint16_t want = (uint16_t)((maMsEff + 5.0f) / 10.0f);   // Ticks zu 10 ms
        if (want > OutMaN) { want = OutMaN; }
        if (want > outMaCount) { want = (uint16_t)outMaCount; }
        if (want >= 2u) {
            float sSum = 0.0f, cSum = 0.0f;
            for (uint16_t k = 0; k < want; k++) {
                const uint8_t idx = (uint8_t)((outMaIdx + OutMaN - 1u - k) % OutMaN);
                const float ang = outMaBuf[idx] * 0.017453292519943295f;
                sSum += sinf(ang); cSum += cosf(ang);
            }
            float meanAz = atan2f(sSum, cSum) * 57.29577951308232f;
            if (config.outMaLead) {
                meanAz = wrap360(meanAz + yawRateDegS * (maMsEff * 0.0005f));
            }
            azSrcVal = wrap360(meanAz);
        }
    }
    const float rawAz = azSrcVal;
    status.azimuthDeg = rawAz;    // --- output smoothing with rate lead (operator request 2026-09-27) --------
    // "The heading flickers with bad reception and the autosteer steers wildly" and
    // "smooth it at higher speed too, the trace in AOG then is calmer". The filter is
    // therefore ALWAYS active now, but with the yaw rate added back:
    //   a pure low-pass of time constant tau lags a steady rotation by tau*omega
    //   (30 deg/s and 200 ms = 6 deg - unusable at creeping speed), while the lead
    //   term makes a real, smooth rotation pass through undelayed and lets the filter
    //   damp only the fast jitter.
    // psi itself is NOT touched - this is the value that leaves the module for AOG.
    if (config.outSmoothMs > 0) {
        const float tau = (float)config.outSmoothMs * 0.001f;
        if (!outSmoothInit) { status.outSmoothDeg = status.azimuthDeg; outSmoothInit = true; }
        float a = dtS / tau;
        if (a > 1.0f) a = 1.0f;
        // --- S-B (2026-10-10): gestaffeltes alpha statt festem tau --------------------
        // Betreiber-Vorgabe + Simulation an den 10-09-Logs: 80/20 (alpha 0,2) bei
        // |Lenkwinkel| <= 4 Grad UND v <= 2 m/s, 70/30 bei <= 6 Grad UND <= 3 m/s, sonst
        // 50/50. Begruendung aus der Messung: das Rauschen stoert nur bei ruhiger Fahrt
        // (dort Faktor 3,0-5,6 Gewinn, 0 ms Phase), waehrend in der Kurve der echte
        // Drehsinn das Rauschen verdeckt und eine traege Stufe nur Kurvenantwort kostet.
        // Der Modellraten-Vorhalt weiter unten bleibt UNVERAENDERT - ein Vorhalt aus der
        // Ableitung des geglaetteten Signals hat in der Simulation den Gewinn aufgehoben.
        if (config.stagedOutFilter) {
            const float awSt = fabsf(wheelAngleDeg);
            if ((awSt <= config.stage1WheelDeg) && (speedMps <= config.stage1SpeedMps)) {
                a = config.stage1Alpha;
            } else if ((awSt <= config.stage2WheelDeg) && (speedMps <= config.stage2SpeedMps)) {
                a = config.stage2Alpha;
            } else {
                a = config.stage3Alpha;
            }
            if (a > 1.0f) a = 1.0f;
            if (a < 0.0f) a = 0.0f;
        }
        // --- 10-14: die QUELLE entscheidet mit ---------------------------------------
        // Fuehrt der PFAD (azSrc != 0: Ring-Kurs / Witness / halten), ist das Rauschen
        // MESSRAUSCHEN der Positionen, nicht Fahrdynamik - dort wird stark geglaettet
        // (Stufe 1). A10 taugt dafuer nicht: seine Stufe schaltet bei Rauschen auf
        // WENIGER Glaettung, weil sie in A echte Dynamik vermutet. Messung 10-14:
        // EMA 0,2 s auf der Kursquelle ist die beste Kombination (18-53: 1,15/3,5 Grad
        // Band, Rate 2,4/8,5 statt 3,1/12,3).
        if (status.azSrc != 0u) {
            if (a > config.stage1Alpha) a = config.stage1Alpha;
        }
        // ---- A10 (2026-10-11): Stufe nach dem GEMESSENEN Rauschen --------------------
        // Der Eingang wird auf |d| gegen die VORIGE Frame geprueft (roher Empfaengerwert,
        // NICHT der eigene Ausgang - sonst misst man die eigene Glaettung). Median ueber
        // outNoiseSamples Werte, damit ein einzelner Ausreisser die Stufe nicht kippt.
        if (config.outNoiseAdaptive) {
            // Fenster in ZEIT (10-12): 0,5 s sind bei 5 Hz zwei Frames, bei 20 Hz zehn
            // (Ringdeckel 8). Ein frame-basiertes Fenster aenderte die Wirkung mit der Rate.
            const float dtMsN = (dtS > 0.0f) ? (dtS * 1000.0f) : 100.0f;
            uint8_t nSamples = (uint8_t)(((float)config.outNoiseWindowMs / dtMsN) + 0.5f);
            if (nSamples < 2) nSamples = 2;
            if (nSamples > 8) nSamples = 8;
            if (!outNoiseInit) {
                outNoisePrevAz = rawAz;
                outNoiseInit = true;
            } else if (attOk) {
                // NUR echte Empfaengerframes zaehlen: in den 10-11-Logs ist die Lage oft
                // unbrauchbar und rawAz wird dann GEHALTEN (aus dem Pfad oder als letzter
                // Wert) - Delta 0. Ein Zaehler, der diese Frames mitnimmt, steht immer auf
                // 0 und die Stufe klebt am Maximum (gemessen 10-11: nz p50 0,00).
                const float dIn = fabsf(wrap180(rawAz - outNoisePrevAz));
                outNoiseHist[outNoiseIdx] = dIn;
                outNoiseIdx = (uint8_t)((outNoiseIdx + 1) % nSamples);
                if (outNoiseCount < nSamples) outNoiseCount++;
                outNoisePrevAz = rawAz;
            }
            if (outNoiseCount > 0) {
                float tmp[8];
                for (uint8_t i = 0; i < outNoiseCount; i++) tmp[i] = outNoiseHist[i];
                // Einfuegesortierung - bei <= 8 Werten billiger als jede Bibliothek.
                for (uint8_t i = 1; i < outNoiseCount; i++) {
                    const float v = tmp[i];
                    int8_t j = (int8_t)(i - 1);
                    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
                    tmp[j + 1] = v;
                }
                status.outNoiseDeg = tmp[outNoiseCount / 2];
            }
            // Zielstufe: ruhig -> stage1Alpha, unruhig -> stage3Alpha, dazwischen linear.
            float aTgt;
            const float nQ = config.outNoiseQuietDeg;
            const float nB = config.outNoiseBusyDeg;
            if (status.outNoiseDeg <= nQ || nB <= nQ) {
                aTgt = config.stage1Alpha;
            } else if (status.outNoiseDeg >= nB) {
                aTgt = config.stage3Alpha;
            } else {
                aTgt = config.stage1Alpha +
                       (config.stage3Alpha - config.stage1Alpha) *
                       ((status.outNoiseDeg - nQ) / (nB - nQ));
            }
            // Traegheit: die Stufe wandert hoechstens outNoiseAlphaSlewPerS pro Sekunde.
            const float slew = config.outNoiseAlphaSlewPerS *
                               ((dtS > 0.0f) ? dtS : 0.01f);
            if (aTgt > outNoiseAlpha + slew) aTgt = outNoiseAlpha + slew;
            if (aTgt < outNoiseAlpha - slew) aTgt = outNoiseAlpha - slew;
            outNoiseAlpha = aTgt;
            // Kombination: die Physik-Regel bleibt die Obergrenze der Glaettung.
            if (outNoiseAlpha < a) a = outNoiseAlpha;
            if (a > 1.0f) a = 1.0f;
            if (a < 0.0f) a = 0.0f;
        }
        status.outAlphaUsed = a;
        // Residual the filter still has to close (difference between the incoming value
        // and what has been published so far).
        const float resid = wrap180(rawAz - status.outSmoothDeg);
        outResidDeg = resid;
        float step = a * resid;
        // ---- 2026-10-02 ("Stufe 2"): the hard snap is QUALITY-GATED --------------
        // Until now EVERY residual above 3 deg was copied into the output in ONE frame
        // ("the smoothed value must never lag a real turn"). Under the trees the raw
        // attitude carries single-frame artifacts of 2...20 deg while qf stays 4
        // (measured 2026-09-30 ... 10-02), so each of them larger than 3 deg reached
        // AgOpenGPS as a jump - the operator's "SprÃ¼nge in den AOG-Daten". The snap is
        // now allowed only while the RECEIVER itself reports a clean epoch
        // (Config::outQualityGateEnabled / uniHdgSdGoodDeg). On a degraded epoch the
        // same residual is instead rate-limited by kinematicRateBoundDegS() *
        // Config::outResidRateFactor, so an artifact becomes a bounded ramp piece while
        // a real turn - which is INSIDE the physical band - still passes at full speed
        // and without extra delay. attQ == 2 (no per-epoch figure, e.g. F9P) keeps the
        // previous behaviour exactly.
        const uint8_t attQ = attitudeQualityNow();
        status.attQualGood = (attQ == 2) ? (int8_t)-1 : (int8_t)((attQ == 0) ? 1 : 0);
        status.outSnapSrc = 2;
        status.outLimClip = 0;
        status.outResidLimDegS = 0.0f;
        // Snap-Schwellwert als RATE (10-12): bei 5 Hz ist ein Frame doppelt so lang, der
        // Residual also groesser - ohne dt-Umrechnung aendert sich die Wirkung mit der Rate.
        const float snapDeg = config.outSnapRateDegS * ((dtS > 0.0f) ? dtS : 0.01f);
        if (attQ != 1) {
            if (fabsf(resid) > snapDeg) {
                step = resid;
                status.outSnapSrc = 1;
            }
        } else {
            // Degraded epoch (see Config::outSnapConfirmFrames): a big residual is HELD
            // until a second frame confirms it. A single-frame artifact therefore never
            // reaches AgOpenGPS, while a real step is passed after ~100 ms - or at the
            // latest after outHoldMaxMs, so a chain of alternating artifacts cannot mask
            // a real state change.
            const float mag = fabsf(resid);
            const int8_t sgn = (resid >= 0.0f) ? 1 : -1;
            const bool big = (mag > snapDeg);
            bool confirmed = false;
            if (big) {
                if ((outPendSign == sgn) && (mag >= (0.5f * outPendMag))) {
                    if (outPendCount < 255) outPendCount++;
                } else {
                    outPendCount = 1;
                    outHoldSinceMs = now;
                }
                outPendSign = sgn;
                outPendMag = mag;
                // Zeitbasiert (10-12): Bestaetigungsdauer in ms, nicht in Frames.
                const float pendMs = (float)outPendCount *
                                     ((dtS > 0.0f) ? (dtS * 1000.0f) : 100.0f);
                confirmed = (pendMs >= (float)config.outSnapConfirmMs) ||
                            ((now - outHoldSinceMs) >= config.outHoldMaxMs);
            } else {
                outPendCount = 0;
                outPendSign = 0;
                outPendMag = 0.0f;
            }
            if (confirmed) {
                step = resid;                 // real change: pass it through
                status.outSnapSrc = 1;
            } else {
                // 2026-11-06f: NICHT einfrieren (hier stand `if (big) { step = 0.0f; }`).
                // Gemessen im Sim (Ref-Log): in einer gleichmaessigen Drehung verliert der Rest
                // im 100-ms-Takt seine Bestaetigung und wurde dauerhaft auf 0 gesetzt - die
                // Ausgabe STAND und holte danach in Spruengen auf (19 stehende Frames). Ein
                // grosser unbestoetigter Rest wird jetzt auf die physikalisch moegliche Rate
                // GEKLEMMT: die EMA-Stufe begrenzt ohnehin auf alpha*Rest, ein Sprung kann
                // also nicht nach draussen, das Stehenbleiben ist aber weg.
                // Secondary guard: never move the output faster than the machine can
                // physically yaw. This binds the SMALL residuals (a wide bound cannot
                // catch a spike - that is what the confirmation above is for).
                const float rateBound = (config.outResidRateFactor > 0.0f)
                                        ? (kinematicRateBoundDegS() * config.outResidRateFactor)
                                        : 0.0f;
                status.outResidLimDegS = rateBound;
                const float maxStep = rateBound * ((dtS > 0.0f) ? dtS : 0.01f);
                if ((maxStep > 0.0f) && (fabsf(step) > maxStep)) {
                    step = (step > 0.0f) ? maxStep : -maxStep;
                    status.outLimClip = 1;
                }
                status.outSnapSrc = big ? 0 : 2;
            }
        }
        // --- B10.61 (2026-10-26): HARTE NOTBREMSE der Ausgabe -------------------------
        // Gemessen 10-26 (16-00, t=32,4 s): die Lage war unbrauchbar (uni_hdgsd 660-700 Grad,
        // att_q = 0), der Hold-Zweig oben hat den 160-Grad-Residual nach der Bestaetigung
        // GANZ durchgelassen (`step = resid`, 3149) - das sind 331 Grad/s auf der Leitung.
        // GRUNDZIELE 1.0 fordert HART: "die Ausgabe springt nie". Also gilt fuer JEDEN Weg
        // (Snap-Zweig und bestaetigter Hold) eine feste Obergrenze.
        // 60 Grad/s ist bewusst GROSSZUEGIG (Betreiber 10-26): eine echte 180-Grad-Wende
        // geht in 3 s durch, ein 331-Grad/s-Artefakt wird auf 60 gekappt. Feine Glaettung
        // bleibt Sache der Physik-Grenze (kinematicRateBoundDegS) und der MA - diese Klemme
        // ist nur die Notbremse, kein Filter.
        const float hardMaxStep = kOutHardRateDegS * ((dtS > 0.0f) ? dtS : 0.01f);
        if ((hardMaxStep > 0.0f) && (fabsf(step) > hardMaxStep)) {
            step = (step > 0.0f) ? hardMaxStep : -hardMaxStep;
            status.outLimClip = 1;
            if (status.outResidLimDegS <= 0.0f) status.outResidLimDegS = kOutHardRateDegS;
        }
        // Beim Oeffnen des Gates darf der Empfaengerwert NICHT als Sprung herausgehen:
        // gemessen weicht das Empfaenger-Heading beim Wiedereintritt 5...15 Grad (extrem
        // 70...100 Grad) vom weitergefuehrten Projektionskurs ab, obwohl seine eigenen
        // Qualitaetszahlen sauber sind. In reentryRampMs wird der Schritt deshalb auf
        // liveRampDegPerS begrenzt - die Projektion fuehrt in der Zwischenzeit weiter,
        // az_out faehrt rampenfrei nach.
        if (liveRampMsLeft > 0) {
            // 2026-11-06f: Die Rampe darf nie LANGSAMER sein als die Maschine dreht - sonst
            // waechst der Rueckstand und wird spaeter als Sprung nachgeholt (gemessen im Sim,
            // Ref-Log t = 173,5...174,7 s: 2,0 Grad je Frame Ausgabe gegen 7 Grad Ziel,
            // 65 Grad Rueckstand, Aufloesung in 20-Grad-Spruengen). Sie oeffnet deshalb auf
            // die GEMESSENE Lagrate.
            float rampRate = config.liveRampDegPerS;
            if (attRateValid) {
                const float need = fabsf(attRateDegS) * config.liveRampFollowFac;
                if (need > rampRate) { rampRate = need; }
            }
            const float maxRampStep = rampRate * ((dtS > 0.0f) ? dtS : 0.01f);
            if (fabsf(step) > maxRampStep) {
                step = (step > 0.0f) ? maxRampStep : -maxRampStep;
                status.outLimClip = 1;
            }
        }
        status.outSmoothDeg = wrap360(status.outSmoothDeg + step);
        // Rate lead: a steady rotation passes through undelayed.
        // NOTE (2026-09-27): without an IMU this rate IS the kinematic model, so a wrong
        // wheel angle ZERO becomes a permanent heading offset (rate*tau) and - when the
        // wheel angle jitters - a wobble of about that size. Measured on the afternoon
        // drives: the VWAS zero (off) had moved by ~9 deg against the morning drives
        // (-38.33 -> -28.97), i.e. a false model yaw of ~4 deg/s per m/s = 0.8 deg of
        // lead at 2 m/s. That is the "the heading turned about 1 degree although the
        // steering was calm" the operator reported. So the lead is dropped whenever the
        // model gate says the model is not describing what the machine is doing.
        float lead = status.yawRateDegS * tau;
        // A10 (2026-10-11): der Vorhalt folgt dem WIRKSAMEN tau = dt/alpha, begrenzt auf
        // das konfigurierte tau. Eine starke Stufe (alpha < dt/tau) holt damit ihre
        // Kurvenantwort zurueck, und eine fast transparente Stufe (alpha ~ 1) bekommt
        // keinen unnoetigen Vorhalt mehr. Obergrenze bleibt outSmoothMs = heutiges
        // Verhalten, der Modell-Gate-Abbruch unten bleibt unveraendert.
        {
            const float dtLead = (dtS > 0.0f) ? dtS : 0.01f;
            float tauEff = (a > 1.0e-4f) ? (dtLead / a) : tau;
            if (tauEff > tau) tauEff = tau;
            lead = status.yawRateDegS * tauEff;
        }
        if (config.modelGateEnabled && status.modelGateActive) lead = 0.0f;
        // A10/10-11 (Betreiber-Auftrag): der Vorhalt ist eine Modell-Groesse und braucht
        // deshalb einen VERTRAUENSWUERDIGEN Lenkwinkel. Messung: die 2,0-Grad-Schritte auf
        // Pfad-Geraden kamen aus einem falschen Nullpunkt (roher Lenkwinkel 25...28 Grad
        // bei Pfadrate <= 1,5 Grad/s, B7.20) - der Vorhalt setzte dort ~7,8 Grad daneben.
        // Ohne Vorhalt folgt die Ausgabe dem Lage-Mittel; die Kurvenantwort bleibt, weil
        // der Zaun dort weit ist UND der Lenkwinkel vertrauenswuerdig ist.
        if (config.outLeadNeedsWheelTrust && !status.wheelObsTrusted) lead = 0.0f;
        status.azimuthDeg = wrap360(status.outSmoothDeg + lead);
        // ---- A10-Schicht 1 (2026-10-11): physikalischer Zaun auf der AUSGABE ---------
        // Begrenzt die Aenderung, die WIRKLICH nach AgOpenGPS geht, auf das, was die
        // Maschine physikalisch drehen kann (kinematicRateBoundDegS() = trim-korrigierter
        // Lenkwinkel + Slack + L_eff, geoeffnet von Pfadkurs und gemessener Gierrate).
        // Weil der Zaun als Faktor auf ein Produkt mit tan(delta_eff) wirkt, bleibt er auf
        // der Geraden praktisch geschlossen und oeffnet in der Kurve - keine
        // Bestaetigungsverzoegerung in Kurven.
        // Ventil: ein Rest, der den Zaun laenger als outFenceValveMs begrenzt, ist kein
        // Artefakt (Anker, Re-Seed, Gate-Wiedereintritt) - er wird mit liveRampDegPerS
        // nachgeholt, damit die Ausgabe nie haengen bleibt.
        status.outFenceClip = 0;
        status.outFenceAllowedDegS = 0.0f;
        if (config.outFenceK > 0.0f) {
            // Basis ist die MODELL-Gierrate DIESER Frame (status.yawRateDegS) - sie enthaelt
            // schon die Skala (steerScale), den trim-korrigierten Winkel und ggf. die IMU-
            // Rate, also genau die Groesse, die auch der Ratenvorhalt benutzt (eine Maschine,
            // eine Zahl). k ist die Reserve: 0,6 (gemessene Skala 0,7 auf dem Acker) bis 1,8
            // (gemessene 1,4 + Reserve). Der Boden outFenceFloorDegS ist die HARTE Forderung
            // aus GRUNDZIELE 1.0 (1 Grad/s) und gilt auch bei schlechtem Empfang.
            float kUse = config.outFenceK;
            if (kUse < 0.6f) kUse = 0.6f;
            if (kUse > 1.8f) kUse = 1.8f;
            // Ohne vertrauenswuerdigen Lenkwinkel ist die Modellrate KEINE Basis: dann gilt
            // nur der Boden (nicht springen, aber auch keinem unsicheren Modell folgen).
            float baseRate = status.wheelObsTrusted ? fabsf(status.yawRateDegS) : 0.0f;
            float boundRate = baseRate * kUse;
            if (boundRate < config.outFenceFloorDegS) boundRate = config.outFenceFloorDegS;
            if (status.outFenceHoldMs >= config.outFenceValveMs &&
                config.liveRampDegPerS > boundRate) {
                boundRate = config.liveRampDegPerS;
            }
            status.outFenceAllowedDegS = boundRate;
            const float budget = boundRate * ((dtS > 0.0f) ? dtS : 0.01f);
            // (1) Der ZUSTAND (EMA-Schritt UND Snap) darf sich nur um das Budget aendern.
            //     Der Rest bleibt stehen und wird mit demselben Budget nachgeholt - genau
            //     daraus entsteht die ruhige Rampe; nach outFenceValveMs uebernimmt das
            //     Ventil (liveRampDegPerS), damit die Ausgabe nie haengt bleibt.
            const float dState = wrap180(status.outSmoothDeg - outSmoothPrevDeg);
            if ((budget > 0.0f) && (fabsf(dState) > budget)) {
                status.outSmoothDeg = wrap360(outSmoothPrevDeg +
                                              ((dState > 0.0f) ? budget : -budget));
                outResidDeg = wrap180(rawAz - status.outSmoothDeg);
                status.outFenceClip = 1;
            }
            // (2) Der VORHALT ist eine KOMPENSATION (er hebt die Filterlatenz einer echten
            //     Drehung auf), kein Bewegungsanteil - er darf deshalb NICHT mit dem engen
            //     Zaun begrenzt werden. Gemessen 10-11: mit dem engen Budget verschlechterte
            //     sich die Treue p90 von 9,4 auf 23...27 Grad, weil der Vorhalt nicht mehr
            //     folgen konnte. Er wird daher am WEITEN Zaun gehalten (derselbe, den der
            //     Messungs-Begrenzer benutzt) - dort sind Pfadkurs und gemessene Gierrate
            //     mit drin, was fuer den Modellraten-Vorhalt genau richtig ist.
            const float leadBudget = kinematicRateBoundDegS() *
                                     ((dtS > 0.0f) ? dtS : 0.01f);
            const float dLead = wrap180(lead - outLeadPrevDeg);
            float leadUse = lead;
            if ((leadBudget > 0.0f) && (fabsf(dLead) > leadBudget)) {
                leadUse = outLeadPrevDeg + ((dLead > 0.0f) ? leadBudget : -leadBudget);
                status.outFenceClip = 1;
            }
            status.azimuthDeg = wrap360(status.outSmoothDeg + leadUse);
            outLeadPrevDeg = leadUse;
            if (status.outFenceClip) {
                const uint16_t addMs = (uint16_t)(((dtS > 0.0f) ? dtS : 0.01f) * 1000.0f);
                if (status.outFenceHoldMs < (65535 - addMs)) status.outFenceHoldMs += addMs;
            } else {
                status.outFenceHoldMs = 0;
            }
        } else {
            status.outFenceHoldMs = 0;
            outLeadPrevDeg = lead;
        }
    }
    {
        // Poor phases are still flagged, so the log/web can show when the estimate is
        // not trustworthy (the measurement noise factor uses the same criteria).
        // The satellite count is NOT one of them (operator correction 2026-09-27):
        // under trees it stays high while the solution quality degrades.
        const auto& gd = gnssProcessor.getData();
        status.outSmoothing = (config.outSmoothMs > 0) &&
                              ((!status.attitudeTrusted) ||
                               (gd.headingQuality != 4) ||
                               (config.rQualitySats > 0 && gd.numSatellites > 0 &&
                                gd.numSatellites < config.rQualitySats) ||
                               // The model gate detects the trouble DIRECTLY, i.e. also
                               // when the receiver keeps reporting quality 4: operator
                               // report 2026-09-27 "under the trees the heading turned
                               // about 1 degree although the steering was calm" - and in
                               // that tree drive 84 % of those steps came from the raw
                               // attitude (p50 2.1 deg per 100 ms) while qf stayed 4, so
                               // neither the quality factor nor the smoothing engaged.
                               status.modelGateActive);
    }
    status.sigmaDeg = sqrtf(p11 > 0.0f ? p11 : 0.0f);
    status.steerBiasDeg = steerBiasDeg;
    // Model-plausibility gate state for the log: true while the elevated R from the
    // last window evaluation is still held.
    status.modelGateActive = config.modelGateEnabled && modelGateUntilMs != 0 &&
                             ((int32_t)(modelGateUntilMs - now) > 0);
    // The wheel angle offset the model actually applies. Zero by design since
    // 2026-09-27 (operator: the zero point of the steering is set by the WAS/VWAS), so a
    // non-zero value here is a FAULT indicator, not a feature.
    status.steerOffsetEstimateDeg = steerBiasDeg;
    // A heading that is ONLY the model is not claimed as valid towards AgOpenGPS after
    // Config::drQf0AfterMs (operator decision 2026-09-28). NAVProcessor turns this into
    // fixQuality = 0, which is the only field in PAOGI that tells AOG the fix is not to
    // be trusted.
    status.headingModelOnly = (status.state == State::DEADRECKON) &&
                              (status.deadReckonMs > config.drQf0AfterMs);
    status.attitudeAgeMs = haveTrusted ? (now - lastTrustedMs) : 0;
    status.bridgeDistanceM = haveTrusted ? (odometerM - lastTrustedOdometerM) : 0.0f;
    status.deadReckonMs = now - lastMeasurementMs;
    status.deadReckonM = odometerM - lastMeasurementOdometerM;
    if (cogValid) status.cogLastDeg = cogMeas;
    if (pathValid) status.pathLastDeg = pathMeas;
    if (config.physicalBaselineM <= 0.1f) {
        // Report the solved length even while the reference is still being
        // learned, so the operator can read it off the web UI.
        status.baselineLengthM = gnssProcessor.getData().baselineLengthM;
    }

    updateMountOffset(now, speedMps);
    updateRoll(dtS, now, freshFrame);
    updateState(now);
    updateSteerScaleCalibration(now);

    if (config.logEvents) {
        logDebugLine(now, rawSpeedMps);
    }
    logRawFrame(now, rawSpeedMps, wheelAngleDeg);
}

// ---------------------------------------------------------------------------
// Steering-scale self calibration: learn, apply, store
// ---------------------------------------------------------------------------
// The scale (see Config::steerScale) is the ratio between the wheel angle the VWAS
// reports and the angle that actually produces the measured curvature. It is a
// MACHINE CONSTANT: tyres, steering geometry, load distribution. So it is learned
// here and stored in the EEPROM - the operator has nothing to measure or enter,
// which is also why the value is reported everywhere (HB1 `sc=`, HB2 `sc=`, raw
// frame `sc`, web UI) rather than hidden behind an inferred number.
//
// Three conditions before anything is written:
//   1. enough accepted estimator samples (curved driving with a trusted attitude),
//   2. the estimate has stopped moving (spread below the configured limit),
//   3. the difference to the active value is worth a write (>= tolerance).
// The EEPROM write happens ONCE per boot, guarded by steerScaleStored: EEPROM has
// a finite endurance and this is a constant, not a running value.
void FLASHMEM HeadingBridge::updateSteerScaleCalibration(uint32_t now) {
    if (!config.enabled) return;
    if (status.steerScaleLearnSamples < config.steerScaleLearnMinSamples) return;
    if (status.steerScaleSpread > config.steerScaleLearnMaxSpread) return;

    const bool firstTime = !status.steerScaleReady;
    status.steerScaleReady = true;
    if (!firstTime) return;   // only act on the transition

    const float learned = status.steerScaleEstimate;
    const float diff = fabsf(learned - config.steerScale);

    if (config.logEvents && config.applyLearnedSteerScale) {
        LOG_NOTICE(EventSource::GNSS,
                   "HeadingBridge steerScale LEARNED: %.3f (active %.3f, n=%lu, spread=%.4f)",
                   (double)learned, (double)config.steerScale,
                   (unsigned long)status.steerScaleLearnSamples,
                   (double)status.steerScaleSpread);
    }

    if (diff < config.steerScaleApplyTolerance) return;

    if (!config.applyLearnedSteerScale) {
        // PASSIVE mode (operator decision 2026-09-23): the estimate is computed and
        // reported, the active value and the EEPROM stay untouched. The sample
        // threshold above is still honoured, so the notice below says exactly what
        // WOULD be applied - that is the number to review after the test drive.
        const bool plausible = (learned >= config.steerScaleApplyMin &&
                                learned <= config.steerScaleApplyMax);
        if (config.logEvents) {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge steerScale LEARNED (%s, not applied): %.3f "
                       "(active %.3f, n=%lu, spread=%.4f)",
                       plausible ? "passive"
                                 : "IMPLAUSIBLE - check cpd/wheelbase, the value is "
                                   "outside the plausibility window",
                       (double)learned, (double)config.steerScale,
                       (unsigned long)status.steerScaleLearnSamples,
                       (double)status.steerScaleSpread);
        }
        (void)now;
        return;
    }

    // Never apply a value outside the plausibility window: such a number is not a
    // scale error but a broken measurement (a hall exit once produced 1.54).
    if (learned < config.steerScaleApplyMin || learned > config.steerScaleApplyMax) {
        if (config.logEvents) {
            LOG_WARNING(EventSource::GNSS,
                        "HeadingBridge steerScale %.3f outside the plausibility window "
                        "%.2f...%.2f - NOT applied",
                        (double)learned, (double)config.steerScaleApplyMin,
                        (double)config.steerScaleApplyMax);
        }
        (void)now;
        return;
    }

    // Apply at runtime and persist. The kinematic model uses config.steerScale, so
    // the very next prediction benefits - no reboot, no operator action.
    config.steerScale = learned;
    {
        extern ConfigManager configManager;
        configManager.setBridgeSteerScale(learned, true);
        configManager.saveHeadingBridgeConfig();
    }
    status.steerScaleStored = true;
    if (config.logEvents) {
        LOG_NOTICE(EventSource::GNSS,
                   "HeadingBridge steerScale APPLIED + stored: %.3f (was %.3f, delta %.3f)",
                   (double)learned, (double)(learned - diff), (double)diff);
    }
    (void)now;
}

// Current position sigma (1 sigma, metres) for the direction threshold.
// The receiver's own standard deviations are used when it reports them
// (INSPVA/INSPVAXA); otherwise the fix type says what to expect: RTK fixed is
// centimetre class, RTK float decimetre, a plain fix metre class scaled by HDOP.
// Why this matters: the direction of travel is derived from a position delta, and
// AgOpenGPS can hard code 0.25 m only because it assumes RTK. On a degraded fix the
// same 0.25 m step is pure noise and would fake a direction.
float FLASHMEM HeadingBridge::positionSigmaM() const {
    const auto& d = gnssProcessor.getData();
    if (d.posStdDevLat > 0.0f || d.posStdDevLon > 0.0f) {
        const float s = sqrtf(d.posStdDevLat * d.posStdDevLat +
                              d.posStdDevLon * d.posStdDevLon);
        if (s > 0.001f && s < 100.0f) return s;
    }
    float s;
    switch (d.fixQuality) {
        case 4:  s = 0.02f; break;   // RTK fixed
        case 5:  s = 0.25f; break;   // RTK float
        case 2:  s = 0.80f; break;   // DGPS
        default: s = 1.50f; break;   // single / unknown
    }
    if (d.hdop > 0.5f && d.fixQuality < 5) s *= d.hdop;
    return s;
}

// ---------------------------------------------------------------------------
// Direction of travel: forward or reverse (AgOpenGPS scheme, ported)
// ---------------------------------------------------------------------------
// OPERATOR RULE (2026-09-23): the heading that leaves the module is the vehicle
// FRONT axis, so driving forwards or backwards along a straight line does not
// change it. The attitude heading satisfies that by construction (the antennas are
// bolted to the front axis); the path derived sources do NOT, because they measure
// where the machine MOVED. They are mirrored by 180 deg while reversing - which is
// exactly what AgOpenGPS does in its single-antenna path
// ("newGPSHeading += Math.PI", Position.designer.cs:500) so that everything
// downstream works with the front axis.
//
// THE DECISION uses the POSITION DELTA, not the COG. AgOpenGPS does the same for
// dual antennas (Position.designer.cs:738-754: the machine must have travelled
// `dualReverseDetectionDistance` = 0.25 m, then the angle between that displacement
// and the heading decides, reverse above 2 rad = 114.6 deg). Measured 2026-09-23 in
// the hall logs, the COG version of this test inverted the direction for 6...12 s on
// every hall entry and exit: the VTG course is unreliable at walking pace under a
// roof, while the position (RTK) stayed good.
// Two guards on top of AOG's rule:
//   * required distance = max(0.25 m, dirSigmaFactor * sigma_pos), so a degraded fix
//     cannot fake a direction out of noise (AOG hard codes 0.25 m because it assumes
//     RTK),
//   * COG cross-check: if COG and position delta disagree, one of them is lying
//     (multipath, spike) and the previous decision is HELD - AOG's "goto byPass".
// The 0.2/0.8 filter and the "changing direction" pause are AOG's own protection
// against acting on the moment the machine turns around.
void FLASHMEM HeadingBridge::updateDirectionState(uint32_t now) {
    const auto& d = gnssProcessor.getData();

    // The position based travel witness first: it gives the direction decision a solid,
    // longer-baseline travel vector to check itself against (operator decision 2026-09-29).
    updateTravelWitness(now);

    // Standstill tracking (2026-10-03). It has to run BEFORE the speed gate below, because a
    // slow K-turn spends most of its time under minSpeedForDirection - that is precisely the
    // case this exists for. See the note at standstillSinceMs in the header.
    if (lastSpeedMps < config.standstillSpeedMps) {
        if (standstillSinceMs == 0) standstillSinceMs = now;
        if ((now - standstillSinceMs) >= config.standstillMinMs) dirReseedPending = true;
    } else {
        standstillSinceMs = 0;
    }

    const float sigma = positionSigmaM();
    status.dirSigmaM = sigma;
    // clamp(5 * sigma, 0.25 m, 0.5 m) - see the config note: the ceiling exists so a
    // confined turning manoeuvre is still detected. A degrading fix must not push the
    // decision further out; the COG cross-check and the direction-change pause take
    // over there.
    float needM = config.dirSigmaFactor * sigma;
    if (needM < config.dirMinDistanceM) needM = config.dirMinDistanceM;
    if (needM > config.dirMaxDistanceM) needM = config.dirMaxDistanceM;

    // A direction needs a trusted heading to compare with, a moving machine and a
    // fresh position. Standing still the position delta is noise, so the last
    // decision is kept (a machine parked in a hall must not start "reversing").
    // Measured 2026-09-24: while the attitude was NOT trusted (hall) psi is driven by
    // the path sources, so comparing it with the path is meaningless - and with a
    // jumping fix the delta points anywhere. Both cases must not decide anything.
    if (!status.attitudeTrusted || status.posFallbackActive ||
        lastSpeedMps < config.minSpeedForDirection ||
        !d.hasPosition || d.fixQuality <= 0 ||
        gnssProcessor.getPositionAgeMs() > (uint32_t)config.maxPositionAgeMs) {
        dirRefValid = false;
        return;
    }
    if (!dirRefValid) {
        dirRefLat = d.latitude; dirRefLon = d.longitude;
        dirRefValid = true;
        return;
    }

    double dN = 0.0, dE = 0.0;
    localDeltaMeters(dirRefLat, dirRefLon, d.latitude, d.longitude, dN, dE);
    const float distM = (float)sqrt(dN * dN + dE * dE);
    status.dirRefDistM = distM;
    const float dtRefS = (float)(now - dirRefMs) / 1000.0f;
    // The delta must also match the SPEED - a jumping fix (multipath in a hall) keeps
    // the distance plausible while the DIRECTION goes anywhere. Such an evaluation is
    // discarded, not used.
    if (dtRefS > 0.05f) {
        const float expect = lastSpeedMps * dtRefS;
        const float tol = config.dirSpeedTolFrac * expect + config.dirSpeedTolM;
        if (fabsf(distM - expect) > tol) {
            dirRefLat = d.latitude; dirRefLon = d.longitude; dirRefMs = now;
            return;
        }
    }
    if (distM < needM) return;               // wait for real displacement

    const float travel = wrap360(atan2f((float)dE, (float)dN) * 57.29577951308232f);
    // Re-base in every case, so the next decision uses a fresh interval.
    dirRefLat = d.latitude; dirRefLon = d.longitude; dirRefMs = now;

    if (cogValid && fabsf(wrap180(travel - cogMeas)) > config.dirCogCrossCheckDeg) {
        return;                              // sources disagree - hold the decision
    }

    status.dirTravelDeg = travel;
    const float raw = fabsf(wrap180(psi - travel));   // 0 ... 180 deg

    // --- independent witness (operator correction 2026-09-29) ----------------
    // "AOG zeigt RÃ¼ckwÃ¤rts, obwohl ich vorwÃ¤rts fuhr" and "die Richtung stimmt nur kurz
    // nicht, wenn der Empfang zurÃ¼ckkommt". Measured in the 06:31 log: ONE bogus
    // evaluation at 0.47 m/s (0.25 m baseline, position noise) set REVERSE and kept it
    // for 50 s, because everything the decision is made from is then mirrored - the
    // system confirmed itself.
    // The raw DUAL ANTENNA is the one heading that is never mirrored, so the travel
    // direction is compared with it as well (0 deg = forward, 180 deg = reverse):
    //   * a reversal is only ACCEPTED when the witness sees it too,
    //   * and it is CLEARED as soon as the witness says forward again - forward always
    //     wins, so a wrong reverse can never hold itself in place.
    // Without a dual heading (single antenna / no attitude) this reduces to the old
    // psi-based decision - the higher minSpeedForDirection stays.
    float attDev = -1.0f;
    if (attWitnessValid) attDev = fabsf(wrap180(travel - attWitnessDeg));
    const bool witnessForward = (attDev >= 0.0f && attDev <= config.reverseClearDeg);
    const bool witnessReverse = (attDev >= 0.0f && attDev >= config.reverseConfirmDeg);

    // --- F1 (operator decision 2026-10-01, "sofort"): emergency clear ------------
    // The psi-based number AND the travel vector can both be mirrored by the (wrong)
    // flag itself, and the "attitude witness" above is only valid while an attitude was
    // trusted. The one test that is independent of all of that is the POSITION witness
    // against the FRONT AXIS: if the machine moves along its front axis, it is going
    // FORWARD, no matter what any heading says.
    // Measured in the 2026-10-01 13-25 log: forward at +1.75 m/s along the body axis
    // (cos 0.99, positions only) while REVERSE stayed latched for 60 s, because the
    // wrong flag made the cogX gate reject every good attitude frame - the state then
    // drifted 30 deg at 1.5 deg/s and AOG saw "the tractor driving sideways".
    // Forward always wins immediately; a genuine reversal is never cut short because
    // there the witness points backwards (dev ~ 180 deg).
    if (status.reverseDirection && attWitnessValid && status.travelWitnessValid &&
        lastSpeedMps >= config.minSpeedForDirection) {
        const float devFront = fabsf(wrap180(status.travelWitnessDeg - attWitnessDeg));
        if (devFront <= config.reverseClearDeg) {
            status.reverseDirection = false;
            revConfirmedMs = now;
            revVotes = 0;
            status.reverseClears++;
            forceLog = true;
            if (config.logEvents) {
                LOG_WARNING(EventSource::GNSS,
                            "HeadingBridge: REVERSE cleared at once (front %.0f vs travel "
                            "%.0f = %.0f deg, %.2f m/s)",
                            (double)attWitnessDeg, (double)status.travelWitnessDeg,
                            (double)devFront, (double)lastSpeedMps);
            }
        }
    }
    // The position based travel witness (config-independent): it cannot say forward or
    // reverse, but it says whether the TRAVEL VECTOR of this evaluation deserves to be
    // believed - and the 09-29 log shows exactly that failure (a bogus 0.25 m baseline at
    // 0.47 m/s decided a 50 s reversal). A decision (and its re-confirmation) therefore
    // requires a solid travel measurement whenever one exists.
    const bool travelSolid = status.travelWitnessValid &&
                             (fabsf(wrap180(travel - status.travelWitnessDeg)) <=
                              config.travelWitnessMaxTurnDeg);

    // Seed (power-on) OR re-seed after a standstill (2026-10-03). Both are the same situation:
    // nothing reliable may be carried over, so the verdict is derived from THIS evaluation -
    // with the witness having the last word (2026-09-29).
    const bool reseed = dirReseedPending;
    if (!dirDevInit || reseed) {
        dirReseedPending = false;
        const bool wasReverse = status.reverseDirection;
        dirDevEma = raw;                               // seed, do not ramp from 0
        dirDevInit = true;
        status.dirDevDeg = raw;
        revVotes = 0;
        // Decide right away, otherwise the first frame of a reverse start would be
        // judged by the attitude gate with the (wrong) "forward" expectation.
        status.reverseDirection = witnessForward ? false
                                 : (witnessReverse ? true
                                                   : (raw > config.reverseDetectDeg));
        if (reseed && status.reverseDirection != wasReverse) {
            // A standstill that changes the verdict is exactly what a K-turn needs, and it is
            // the one line that shows whether the re-seed did its job.
            forceLog = true;
            if (config.logEvents) {
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge direction: %s after a standstill (heading-to-path "
                           "%.0f deg, attitude witness %.0f deg, %.2f m/s)",
                           status.reverseDirection ? "REVERSE (path sources mirrored by 180 deg)"
                                                   : "forward",
                           (double)raw, (double)attDev, (double)lastSpeedMps);
            }
        }
        return;
    }
    dirDevEma += config.directionFilterAlpha * (raw - dirDevEma);
    status.dirDevDeg = dirDevEma;

    // AgOpenGPS: a large |filtered - raw| means the direction is changing right
    // now, so nothing may be concluded. The old decision is held.
    status.dirChanging = (fabsf(dirDevEma - raw) > config.directionChangeDeg);
    if (status.dirChanging) return;

    bool reverse = (dirDevEma > config.reverseDetectDeg);
    // The witness may also OPEN a reversal on its own (psi can be dragged by the mirrored
    // path sources, so the psi-based number can be late or wrong) - but it never blocks a
    // clear.
    if (witnessReverse && !status.reverseDirection) reverse = true;
    if (witnessForward) reverse = false;

    if (reverse != status.reverseDirection) {
        // A single evaluation must not flip the direction (one jumping fix is enough
        // to produce one - measured 2026-09-24). The same verdict has to repeat.
        // A reversal additionally needs the witness (2026-09-29): without it the
        // evaluation is HELD and counted, instead of mirroring the whole chain.
        if (reverse && attDev >= 0.0f && !witnessReverse) {
            status.dirVetoes++;
            revVotes = 0;
            return;
        }
        // And it needs a SOLID travel vector (2026-09-29): the flip that held REVERSE for
        // 50 s was decided on a 0.25 m baseline at 0.47 m/s. Where a solid measurement
        // exists the decision must agree with it.
        if (reverse && status.travelWitnessValid && !travelSolid) {
            status.dirVetoes++;
            revVotes = 0;
            return;
        }
        revVotes++;
        if (revVotes < config.dirVotesNeeded) return;
        revVotes = 0;
        status.reverseDirection = reverse;
        revConfirmedMs = now;
        forceLog = true;    // a direction change must be visible in the session log
        if (config.logEvents) {
            LOG_NOTICE(EventSource::GNSS,
                       "HeadingBridge direction: %s (heading-to-path %.0f deg, attitude "
                       "witness %.0f deg, speed %.2f m/s)",
                       reverse ? "REVERSE (path sources mirrored by 180 deg)" : "forward",
                       (double)dirDevEma, (double)attDev, (double)lastSpeedMps);
        }
    } else {
        revVotes = 0;       // votes never carry over into an unrelated decision
        // Re-confirmation with a SOLID measurement (operator decision 2026-09-29): while the
        // travel vector deserves to be believed and it agrees with the current verdict, the
        // verdict is refreshed. If it cannot be refreshed for revReconfirmMs while solid
        // measurements exist, the verdict is DROPPED - that is the anti-lock exit for a
        // reversal that was set by a bogus vector. While the machine crawls (no solid
        // measurement) nothing is dropped, so a genuine slow reverse is never cut short.
        if (travelSolid) {
            revConfirmedMs = now;
        } else if (status.reverseDirection && config.revReconfirmMs > 0 &&
                   revConfirmedMs != 0 &&
                   (now - revConfirmedMs) > config.revReconfirmMs) {
            status.reverseDirection = false;
            revConfirmedMs = now;
            forceLog = true;
            if (config.logEvents) {
                LOG_WARNING(EventSource::GNSS,
                            "HeadingBridge direction: REVERSE dropped after %lums without a "
                            "confirming travel measurement (speed %.2f m/s, heading-to-path "
                            "%.0f deg)",
                            (unsigned long)config.revReconfirmMs,
                            (double)lastSpeedMps, (double)dirDevEma);
            }
        }
    }
    (void)now;
}

// ---------------------------------------------------------------------------
// Position quality gate + fix-to-fix window
// ---------------------------------------------------------------------------
// A fix-to-fix heading is only as good as the two fixes it is built from. The
// 2026-09-21 data set shows why this has to be checked separately from
// "hasPosition && fixQuality > 0":
//
//   * a single-sample POSITION SPIKE shifts the window direction for its whole
//     lifetime - in a 0.5 m window a 0.3 m spike is a ~30 deg heading error,
//   * the window direction is meaningless while the machine travels less than the
//     position noise (0.5 m is 1.6 s at 0.3 m/s, so a creep-and-stop profile keeps
//     manufacturing "directions" out of noise),
//   * and a window containing a spike has a rectified arc noticeably longer than
//     its net displacement - measurable for free.
//
// Three cheap scalar tests run on every new fix:
//
//   1. VALIDITY  fix quality > 0, position age, HDOP envelope. A failure
//                invalidates the reference, so the window restarts cleanly.
//   2. STEP      the displacement since the previous fix must be physically
//                possible: max(v * dt * margin) + a noise floor. A failure is a
//                spike -> the REFERENCE is dropped, not just the sample, because
//                the spike would otherwise stay in the window and bend the
//                direction until it ages out.
//   3. WINDOW    net displacement >= minDisplacementForFixToFixM AND
//                net / sum(distance to reference) >= minPathStraightness. That
//                ratio is 0.9997 for a 0.5 m window on a 6 m radius turn, so a
//                value below ~0.9 can only come from noise or a spike.
//
// The gates decide whether the measurement may be used at all; how much it then
// moves the estimate is rFixToFixDeg in the filter. Its consistency with the
// fused state and with the COG is checked in selectFallbackMeasurement().
void FLASHMEM HeadingBridge::updatePositionWindow(uint32_t now, float speedMps) {
    const auto& d = gnssProcessor.getData();
    pathValid = false;

    // --- 1. validity --------------------------------------------------------
    // The age comes from the dedicated position counter, not from
    // gpsData.lastUpdateTime: that one is moved by every sentence carrying a
    // position (GGA and GNS both), so it is not a fix age.
    const bool posFresh = d.hasPosition && d.fixQuality > 0 &&
                          gnssProcessor.getPositionAgeMs() <= (uint32_t)config.maxPositionAgeMs;
    const bool hdopOk = (d.hdop <= 0.0f) || (d.hdop <= config.maxHdopForPath);
    if (!posFresh || !hdopOk) {
        // Only the window is invalidated. The last SEEN fix is kept on purpose, so
        // that the step test below still has a reference when the stream returns -
        // otherwise the first fix after a gap would be the one step nobody checks,
        // and the hall data set shows exactly that glitch (164 deg direction error
        // in the first step after a fix-quality gap).
        haveRef = false;
        if (!hdopOk) status.posFaults++;
        return;
    }

    // Only react when the fix itself changed; the remaining 100 Hz ticks must not
    // re-run the geometry on the same point. The counter is read, not consumed -
    // updateProjection() needs to see the same fix (that was the 2026-09-23 bug).
    if (haveLastPos && gnssProcessor.getPositionCount() == lastWindowFixCount) {
        return;
    }

    // --- 2. step test against the previous fix ------------------------------
    // The step is needed twice: here for the spike test and below for the arc
    // accumulator of the straightness ratio.
    float stepM = 0.0f;
    if (haveLastPos) {
        double sN = 0.0, sE = 0.0;
        localDeltaMeters(lastPosLat, lastPosLon, d.latitude, d.longitude, sN, sE);
        stepM = (float)sqrt(sN * sN + sE * sE);
        const float dtS = (float)(now - lastPosMs) / 1000.0f;
        // Allowance: what the machine could have travelled (with a factor for a
        // wrong VTG) plus a floor for the position noise itself (F9P RTK: 1...3 cm
        // per fix, so 0.15 m is generous). The rule lives in maxPosStepAllowedM (B10.40) because the B-chain chord uses the SAME thresholds (operator decision 10-13).
        const float allowed = maxPosStepAllowedM(speedMps, dtS);
        if (stepM > allowed) {
            // Spike: drop the reference so the corrupted point cannot influence the
            // next window, and publish no direction this cycle.
            status.posFaults++;
            status.posStepM = stepM;
            // P4.1 (operator request 2026-09-30): tell the POSITION limiter that the
            // RECEIVER's own step is discontinuous. Its own test compares the new fix
            // with what WE published (the projection), which legitimately differs while
            // the catch-up ramp runs - that is the hole through which 1.4...3.7 m steps
            // reached AgOpenGPS in the "Halle 2x" log. This flag is independent of what
            // we publish, so it closes the hole.
            status.posSpikeFix = true;
            refArcM = 0.0f;
            haveRef = false;
            lastPosLat = d.latitude; lastPosLon = d.longitude;
            lastPosMs = now; lastWindowFixCount = gnssProcessor.getPositionCount();
            return;
        }
        status.posSpikeFix = false;      // the fix stream is continuous again
    }

    // --- 3. window ---------------------------------------------------------
    if (!haveRef) {
        // New reference. The arc accumulator starts at zero and is fed by the
        // steps that FOLLOW the reference, so at every later fix the arc covers
        // exactly the path from the reference to the current point - which is
        // what the ratio below has to compare with the net displacement.
        refLat = d.latitude; refLon = d.longitude; refMs = now; refArcM = 0.0f;
        haveRef = true;
    } else {
        refArcM += stepM;                    // arc travelled since the reference
        double dN = 0.0, dE = 0.0;
        localDeltaMeters(refLat, refLon, d.latitude, d.longitude, dN, dE);
        const float range = (float)sqrt(dN * dN + dE * dE);
        status.pathRangeM = range;
        status.pathStraightness = (refArcM > 0.01f) ? (range / refArcM) : 1.0f;

        if (range >= config.minDisplacementForFixToFixM &&
            status.pathStraightness >= config.minPathStraightness) {
            // atan2f, not atan2: the double version drags a large amount of
            // support code into RAM1/ITCM for a few degrees of accuracy that are
            // not needed here (measured: RAM1 free-for-locals dropped by 3 KB with
            // the double call, and RAM1 is the scarce region).
            pathMeas = wrap360(atan2f((float)dE, (float)dN) * 57.29577951308232f);
            pathValid = true;
            // P2 (2026-09-30): the turn rate of this ACCEPTED window, for the rate
            // limit. It is what tells the limiter "the machine really is turning",
            // without which the bound would delay the start of a turn.
            if (lastPathMeasMs > 0) {
                const float dtP = (float)(now - lastPathMeasMs) / 1000.0f;
                if (dtP > 0.05f && dtP < 2.0f) {
                    pathRateDegS = wrap180(pathMeas - lastPathMeasDeg) / dtP;
                }
            }
            lastPathMeasDeg = pathMeas;
            lastPathMeasMs = now;
            // Re-base on a real displacement so the window keeps growing until it
            // is worth something, and reset the arc accumulator with it.
            refLat = d.latitude; refLon = d.longitude; refMs = now;
            refArcM = 0.0f;
        }
    }

    lastPosLat = d.latitude; lastPosLon = d.longitude;
    lastPosMs = now; lastWindowFixCount = gnssProcessor.getPositionCount();
    haveLastPos = true;
}

// ---------------------------------------------------------------------------
// Forward projection of the POSITION (operator proposal 2026-09-23)
// ---------------------------------------------------------------------------
// Runs at 100 Hz: the predicted position is integrated with the same kinematics
// that drives psi (psi += v*tan(scale*delta + bias)/L * dt), so the WHEEL ANGLE
// is part of the projection. At every new fix the residual between the predicted
// and the measured position is split into
//
//   along-track  dLong  = (p_meas - p_proj) . e_head     -> speed/scale error
//   cross-track  dCtk   = (p_meas - p_proj) . e_right    -> HEADING error
//
// and the cross-track part gives a heading measurement
//
//   psi_meas = psi + atan2(-dCtk, path_length),   sigma = atan(sigma_pos / path)
//
// Why this is better than the two-point chord (fix-to-fix): the chord measures the
// average direction of the interval, so it lags by omega*T/2 while turning and it
// throws the kinematic information away. The projection carries the turn in psi
// already, so the residual only has to correct the ERROR. Same position noise,
// but no chord lag.
//
// Cost: one sin/cos, two multiplies and two adds per 100 Hz tick (~0.1 us), and one
// atan2f per 10 Hz fix - nothing compared to the PAOGI/NMEA formatting. It does not
// delay the output either: the measurement is used like any other.
// ---------------------------------------------------------------------------
// Step C (2026-10-05): a fix-to-fix measurement at EVERY fix, from the ring
// ---------------------------------------------------------------------------
// updatePositionWindow() produces the same geometry but RE-BASES after every accepted window
// (refArcM = 0), so a measurement exists only near the end of each window and its baseline
// varies from 0 to ~0.6 m. The central ring holds every fix with its raw position, so the same
// geometry is available over a CONSTANT, exact baseline: the oldest sample at least minDistM
// away. Measured 2026-10-04: proj_upd 6.3/s against fix_cnt 10.0/s, and the 9 deg skew lives in
// the phases where the attitude is trusted - exactly where nothing path-anchored was admitted.
//
// Applied ONLY while the attitude is trusted: in the degraded phases
// selectFallbackMeasurement() already covers the path with its own window, and two measurements
// of the same geometry in one tick would over-weight it.
//
// Everything follows the existing conventions on purpose (see selectFallbackMeasurement): the
// bearing is atan2f(E, N) in degrees, 0 = north clockwise; the side-slip offset turns a PATH
// measurement into a BODY heading; a reverse phase mirrors it by 180 deg; the innovation limit
// is the physical allowance; and the R is the adaptive path R inflated by the heading
// uncertainty of THIS baseline (2 sigma of the position over the baseline).
bool FLASHMEM HeadingBridge::updatePathFromRing(uint32_t now, float speedMps) {
    if (!config.ringPathMeasurement || !status.attitudeTrusted) return false;
    const auto& d = gnssProcessor.getData();
    if (!d.hasPosition || d.fixQuality <= 0) return false;
    if (d.hdop > 0.0f && d.hdop > config.maxHdopForPath) return false;

    // One measurement per FIX - the counter is read, never consumed (other consumers need it).
    const uint32_t fixCount = gnssProcessor.getPositionCount();
    if (fixCount == lastRingFixCount) return false;
    lastRingFixCount = fixCount;

    // One baseline length for EVERY fix type on purpose: the RELATIVE geometry of consecutive
    // fixes is as good without RTK as with it (measured 2026-10-04: relative noise RMS 3.2 cm at
    // fixQuality 1 against 3.7 cm at fixQuality 4), so a degraded fix must not be punished here -
    // only its QUALITY (the R below) differs. The baseline AGE is bounded so a standstill cannot
    // pair two remote moments.
    const float minDistM = config.minDisplacementForFixToFixM;
    const PositionHistory::FixSample* ref = nullptr;
    float distM = 0.0f, arcM = 0.0f;
    if (!positionHistory.lookback(d.latitude, d.longitude, now, minDistM, 0.0f, 3000u,
                                  ref, distM, arcM) || ref == nullptr) {
        return false;
    }
    const float straightness = (arcM > 0.05f) ? (distM / arcM) : 1.0f;
    if (straightness < config.minPathStraightness) return false;

    const float ageS = (float)(now - ref->tMs) * 0.001f;
    if (ageS > 3.0f) return false;
    // Speed witness (the concept's rule): the chord must match what the machine could have
    // travelled in that time - a mismatch means the fix or the reference is not trustworthy.
    if (ageS > 0.05f) {
        const float expectM = speedMps * ageS;
        if (fabsf(distM - expectM) > (0.5f * expectM + 0.4f)) return false;
    }

    double dN = 0.0, dE = 0.0;
    localDeltaMeters(ref->lat, ref->lon, d.latitude, d.longitude, dN, dE);
    const float slip = (config.estimateSideSlip && sideSlipSamples > 200) ? sideSlipDeg : 0.0f;
    const float meas = wrap360(atan2f((float)dE, (float)dN) * 57.29577951308232f +
                               slip + (status.reverseDirection ? 180.0f : 0.0f));

    const float innov = fabsf(wrap180(meas - psi));
    const float blindS = (float)(now - lastMeasurementMs) / 1000.0f;
    const float innovLimit = config.pathInnovationBaseDeg + config.pathInnovationPerS * blindS;
    if (innov > innovLimit) return false;

    // The R of this baseline: the RELATIVE position noise over the chord (2 sigma), never the
    // absolute one - see Config::posRelativeNoiseM. 4.5 deg at 0.5 m, 2.2 deg at 1.0 m.
    const float rGeom = (distM > 0.05f)
                        ? (2.0f * config.posRelativeNoiseM / distM * 57.29577951308232f) : 90.0f;
    if (rGeom > 20.0f) return false;   // safety net: a baseline this short is not a direction
    const float rBase = effectiveMeasurementR(1);
    applyMeasurement(meas, sqrtf(rBase * rBase + rGeom * rGeom), 1);
    status.fixToFixUpdates++;
    status.source = Source::PATH_FIX2FIX;
    lastMeasurementMs = now;
    imuAnchorMs = now;          // an absolute anchor for the IMU offset/rate learning
    return true;
}

// ---------------------------------------------------------------------------
// B-Kette (2026-10-13, B10.34/B10.35/B10.40): Kurs aus dem RINGPUFFER ueber eine
// Bogenlaenge - ADAPTIV, VORHER GEPRUEFT, nachlaufkorrigiert.
// ---------------------------------------------------------------------------
// Warum (gemessen 12-38 / 12-30, Zustand B): die bisherige Quelle `travelWitness` ist
// eine Netto-Verschiebung ueber >= 1 m und gilt NUR bei geradem Fenster
// (travelWitnessMaxTurnDeg = 30 Grad). In der Kurve faellt sie aus, der Ausgang wird
// GEHALTEN und springt beim Wiedereinsetzen - gemessen Jitter p99 83 Grad/s und Rate
// auf Geraden p99 64 Grad/s. Der Kurs ueber 2 m aus dem Ring existiert auch in der
// Kurve: offline (gleiche Methode, 2-s-Fit-Wahrheit) Jitter p99 22 Grad/s, Rate auf
// Geraden p99 16 Grad/s, Band p50 0,79 statt 1,58 Grad; in Zustand D 175 -> 10 Grad.
//
// Die Sehne zeigt die Richtung am ANFANG des Bogens, also um 0,5*omega*T nachgelaufen
// (GRUNDZIELE 2.1). omega kommt aus dem Modell (Lenkwinkel trim-korrigiert, v, L) und
// wird hereingereicht - dieselbe Zahl wie status.yawRateDegS. Damit tragen die
// POSITIONEN das Absolute und GIERRATE + LENKWINKEL das Modell (Betreiber-Vorgabe 10-12).
//
// Kein Side-Slip-Abzug: der Kurs ist eine FAHRTRICHTUNG (wie der Witness) und wird erst
// mit dualHeadingOffsetDeg auf die Koerperachse gedreht (das macht die Quellenwahl).
//
// ADAPTIV (Betreiber-Entscheidung 10-13: "adaptiv"): die KURZE Sehne (pathChordMinM) ist
// der Standard, weil in der Kurve der NACHLAUF begrenzt und nicht das Rauschen (B10.34:
// 2 m schlug 5 m in B UND in A). Die lange Sehne (pathChordLongM) wird nur genommen,
// wenn der eigene Weg gerade ist UND sie die kurze Richtung bestaetigt.
//
// GEPRUEFT, BEVOR GEMESSEN WIRD (Betreiber-Frage 10-13: "wird die Position geprueft,
// bevor wir den Bogen messen?"). Vorher gab es NUR die grobe Distanzplausibilitaet: ein
// Ausreisser von +0,3 m und -0,3 m zurueck bestand jede Einzelpruefung und kippte eine
// 2-m-Sehne um ~17 Grad. Jetzt:
//   1. FIX-zu-FIX-Schritt des neuesten Fixes gegen dieselbe Schwelle wie das Fenster
//      (maxPosStepAllowedM - Betreiber: "gleiche Schwellen"). Bei Verstoss wird GEHALTEN
//      (kein Kurs, die Ausgabe behaelt ihren letzten Wert) und der Ringeintrag bekommt
//      F_SPIKE_REJECTED (PositionHistoryService).
//   2. ENDPUNKT-Probe: die Sehne darf nicht WEITER sein als die eigene Fahrt plus
//      Toleranz - ein fortgesprungener Fix haelt die Distanz plausibel, nicht die Zeit.
//   3. Lang nur mit Bestaetigung durch kurz (|delta| <= pathChordDisagreeDeg), weil in
//      der Kurve kurz und lang stark auseinanderlaufen - dort MUSS kurz bleiben.
//   4. Geradheit arc/chord aus dem RING (der eigene Weg, nicht die Wahrheit).

// Weglaenge entlang des Rings vom Eintrag `back` bis zur AKTUELLEN Position. Die letzte
// Etappe (neuester Ringeintrag -> aktueller Fix) liegt noch nicht im Ring und wird direkt
// gemessen. Nur fuer die Geradheit arc/chord - kein Speicher, keine Rekursion.
static float ringArcToNowM(uint16_t back, double lat, double lon) {
    const PositionHistory::FixSample* prev = positionHistory.at(0);
    if (prev == nullptr) return 0.0f;
    double n = 0.0, e = 0.0;
    localDeltaMeters(prev->lat, prev->lon, lat, lon, n, e);
    float arc = (float)sqrt(n * n + e * e);
    for (uint16_t i = 0; i < back; i++) {
        const PositionHistory::FixSample* a = positionHistory.at(i);
        const PositionHistory::FixSample* b = positionHistory.at(i + 1);
        if (a == nullptr || b == nullptr) break;
        arc += (float)PositionHistory::distanceM(*a, *b);
    }
    return arc;
}

// Die EINE Schrittregel: was die Maschine gefahren sein koennte (Faktor fuer ein falsches
// VTG) plus ein Boden fuer das Positionsrauschen. Fenster und B-Kette benutzen sie beide -
// Betreiber-Entscheidung 10-13 ("gleiche Schwellen").
float FLASHMEM HeadingBridge::maxPosStepAllowedM(float speedMps, float dtS) const {
    const float dt = (dtS > 0.01f && dtS < 5.0f) ? dtS : 0.1f;
    return config.maxPosStepMargin * speedMps * dt + config.maxPosStepNoiseM;
}

// Die erste (NEUESTE) Ringprobe mindestens minDistM entfernt: von NEU nach ALT laufen.
// lookback() geht bewusst OLDEST-first (laengste Basis - fuer die Richtungsmessung in psi
// richtig); fuer den Kurs als AUSGABE ist die KUERZESTE Basis die richtige, weil sie am
// naechsten an der aktuellen Tangente liegt (der Nachlauf 0,5*omega*T bleibt klein).
bool FLASHMEM HeadingBridge::findChordSample(uint32_t now, double lat, double lon,
                                             float minDistM, uint16_t& backOut,
                                             float& distM, float& arcM) const {
    backOut = 0;
    distM = 0.0f;
    arcM = 0.0f;
    const uint16_t ringCnt = positionHistory.count();
    for (uint16_t back = 1; back < ringCnt; back++) {
        const PositionHistory::FixSample* cand = positionHistory.at(back);
        if (cand == nullptr) continue;
        if ((cand->flags & (PositionHistory::F_VALID | PositionHistory::F_POS_OK)) !=
            (PositionHistory::F_VALID | PositionHistory::F_POS_OK)) continue;
        if ((uint32_t)(now - cand->tMs) > config.pathChordMaxAgeMs) break;
        double cN = 0.0, cE = 0.0;
        localDeltaMeters(cand->lat, cand->lon, lat, lon, cN, cE);
        const float dM = (float)sqrt(cN * cN + cE * cE);
        if (dM >= minDistM) {
            backOut = back;
            distM = dM;
            arcM = ringArcToNowM(back, lat, lon);
            return true;
        }
    }
    return false;
}

void FLASHMEM HeadingBridge::updatePathChord(uint32_t now, float speedMps, float yawRateDegS) {

    status.pathChordValid = false;
    status.pathChordLenM = 0.0f;
    status.pathChordStepM = 0.0f;
    status.pathChordSpike = false;
    status.pathChordJumpRejected = false;
    s_pcRej = 0; s_pcDevDeg = 0.0f; s_pcDevLim = 0.0f;
    if (!config.pathChordEnable) { s_pcRej = 1; return; }
    const auto& d = gnssProcessor.getData();
    if (!d.hasPosition || d.fixQuality <= 0) { s_pcRej = 2; return; }
    if (d.hdop > 0.0f && d.hdop > config.maxHdopForPath) { s_pcRej = 3; return; }
    // B10.53: FRISCHE DES NEUESTEN FIXES statt einer Altersgrenze auf dem fernen Endpunkt.
    {
        const PositionHistory::FixSample* newest = positionHistory.at(0);
        if ((newest == nullptr) ||
            ((uint32_t)(now - newest->tMs) > kNewestFixMaxAgeMs)) { s_pcRej = 7; return; }
    }
    // B10.53: eigene Untergrenze fuer die SEHNE (der Witness behaelt 1,0 m/s). Gemessen mit
    // einer UNABHAENGIGEN Referenz (zentrierte 8-s-Sehne, nachlauffrei): bei v < 0,6 m/s ist
    // die Ausgabe damit p90 6,7 statt 7,8 Grad und max 15,5 statt 22,9 Grad neben der Wahrheit.
    if (speedMps < kChordMinSpeedMps) { s_pcRej = 4; return; }   // creeping: no direction from positions
    // NICHT rueckwaerts: der Kurs ist dann eine RUECKWAERTS-Richtung und darf nicht als
    // Vorderachse publiziert werden (Betreiber-Regel: die Vorderachse kippt nicht). Ohne
    // diese Sperre entsteht ein 180-Grad-Kipp-Zirkelbezug (im Sim 1 Hz sichtbar: 71<->271).
    if (status.reverseDirection) { s_pcRej = 5; return; }

    // --- 1. Ist der NEUESTE Fix ueberhaupt plausibel? (2-Fix-Gegenprobe) ---------
    const PositionHistory::FixSample* prevFix = positionHistory.at(0);
    if (prevFix != nullptr) {
        const float dtFix = (float)(now - prevFix->tMs) * 0.001f;
        if (dtFix > 0.001f && dtFix < 5.0f) {
            double n = 0.0, e = 0.0;
            localDeltaMeters(prevFix->lat, prevFix->lon, d.latitude, d.longitude, n, e);
            const float stepM = (float)sqrt(n * n + e * e);
            status.pathChordStepM = stepM;
            if (stepM > maxPosStepAllowedM(speedMps, dtFix)) {
                status.pathChordSpike = true;   // -> F_SPIKE_REJECTED im Ring, und HALTEN
                s_pcRej = 6;                   // Fix-Sprung
                return;                        // kein Kurs aus einer gesprungenen Spitze
            }
        }
    }

    // --- 2. die KURZE Sehne (Standard, kurventauglich) --------------------------
    uint16_t backShort = 0;
    float distShort = 0.0f, arcShort = 0.0f;
    // B10.53: B folgt der Geschwindigkeit (0,5 ... config.pathChordMinM).
    const float chordMinM = hbChordMinM(config.pathChordMinM, speedMps);
    if (!findChordSample(now, d.latitude, d.longitude, chordMinM,
                         backShort, distShort, arcShort)) { s_pcRej = 7; return; }

    // --- 3. ADAPTIV: lang nur, wenn der eigene Weg GERADE ist -------------------
    uint16_t back = backShort;
    float distM = distShort;
    if (config.pathChordLongM > (config.pathChordMinM + 0.05f) &&
        (arcShort > 0.01f) && ((distShort / arcShort) >= config.pathChordStraightRatio)) {
        uint16_t backLong = 0;
        float distLong = 0.0f, arcLong = 0.0f;
        if (findChordSample(now, d.latitude, d.longitude, config.pathChordLongM,
                            backLong, distLong, arcLong) &&
            (arcLong > 0.01f) && ((distLong / arcLong) >= config.pathChordStraightRatio)) {
            const PositionHistory::FixSample* rs = positionHistory.at(backShort);
            const PositionHistory::FixSample* rl = positionHistory.at(backLong);
            if (rs != nullptr && rl != nullptr) {
                double sN = 0.0, sE = 0.0, lN = 0.0, lE = 0.0;
                localDeltaMeters(rs->lat, rs->lon, d.latitude, d.longitude, sN, sE);
                localDeltaMeters(rl->lat, rl->lon, d.latitude, d.longitude, lN, lE);
                const float pr = 57.29577951308232f;
                const float dirShort = atan2f((float)sE, (float)sN) * pr;
                const float dirLong = atan2f((float)lE, (float)lN) * pr;
                if (fabsf(wrap180(dirLong - dirShort)) <= config.pathChordDisagreeDeg) {
                    back = backLong;
                    distM = distLong;
                }
            }
        }
    }
    const PositionHistory::FixSample* ref = positionHistory.at(back);
    if (ref == nullptr) return;
    const float ageS = (float)(now - ref->tMs) * 0.001f;
    if (ageS < 0.05f) return;
    // 10-14: starr 3 s (der Versuch "Alter mit der Laenge wachsen zu lassen" brachte in
    // 18-59 KEINE gueltige Sehne mehr - bei 0,7 m/s ist eine 2-m-Sehne 2,9 s alt, bei
    // 4 m 5,7 s; ein so alter Kurs ist wertlos. Die Lektion: unter ~1 m/s gibt es keinen
    // frischen Positionskurs, dort ist HALTEN richtig).
    if (ageS > ((float)config.pathChordMaxAgeMs * 0.001f)) return;

    // --- 4. Endpunkt-Probe: die Sehne darf nicht WEITER sein als die Fahrt ------
    const float expectM = speedMps * ageS;
    if (distM > (expectM + config.pathChordSpeedTol * expectM + config.maxPosStepNoiseM)) return;

    double dN = 0.0, dE = 0.0;
    localDeltaMeters(ref->lat, ref->lon, d.latitude, d.longitude, dN, dE);
    const float raw = atan2f((float)dE, (float)dN) * 57.29577951308232f;
    float lag = 0.0f;
    // 10-14: Nachlauf-Korrektur UNVERAENDERT (Feldmessung: 1,5 Grad p50 in 18-53). Die
    // Versuche dieser Sitzung (Alter mitwachsen lassen, Betrag auf 12 Grad deckeln,
    // Abschalten bei yawRateZeroed) haben den Sim verschlechtert (B-Band 10,5 statt 1,3),
    // weil dort die MODELLRATE falsch ist - und fuer den Feld-Nachlauf gibt es keine
    // Messung, die das stuetzt. Deshalb bleibt es beim Stand B10.40; offen: Nachlauf nur
    // mit vertrauenswuerdiger Modellrate (mdev/mgate als Signal, nicht yawRateZeroed).
    if (config.pathChordLagCorr) lag = 0.5f * yawRateDegS * ageS;
    if (lag > 30.0f) lag = 30.0f;          // one frame must not invent a big turn
    if (lag < -30.0f) lag = -30.0f;
    status.pathChordLagDeg = lag;
    // NICHT spiegeln: als QUELLE gilt die VORDERACHSE (Betreiber-Regel 09-23) - genau
    // wie travelWitnessDeg wird der Kurs nicht gedreht. Nur die Messungen, die in psi
    // eingehen (updatePathFromRing), werden bei Rueckwaerts um 180 Grad gespiegelt.
    float chord = wrap360(raw + lag);
    // B10.51: Der Antennenpunkt sitzt antennaPivot vor der Vorderachse und eilt der
    // Koerperrichtung in der Kurve voraus (atan(omega*d/v)). Die Sehne liefert also die
    // BAHNRICHTUNG des Antennenpunkts - gemessen 7-52: skew p50 +5,9 / p90 18,8 Grad, was
    // zu atan(0,83/R) passt. AOG will die Koerper- (bzw. Mittel-) richtung -> abziehen.
    s_antLeadDeg = hbAntennaLeadDeg(yawRateDegS, speedMps);
    chord = wrap360(chord - s_antLeadDeg);
    // --- 3b. SPRUNGTOR der Sehne (B10.47, 2026-10-18) ---------------------------
    // Ein Fix-Sprung kann die Sehne in EINEM Frame um >90 Grad drehen (Feld 14-59:
    // 143 Grad bei t = 26,4 s unter der Bruecke, waehrend dieselbe Loesung 195 %
    // Basisabweichung meldete und die Positionen 0,43...0,61 m bei 1,2 m/s sprangen).
    // Physikalisch sind bei 81 Grad/s hoechstens ~8 Grad je 0,1 s moeglich - alles
    // ueber 25 Grad je Frame ist ein Sprung und keine Drehung.
    //   * Referenz = die zuletzt UEBERNOMMENE Richtung; sie bleibt beim Verwerfen
    //     stehen (sonst wuerde die Referenz dem Sprung nachlaufen).
    //   * Ausnahme: die neue Richtung besteht 3 s UND die Lage ist nicht als
    //     degradiert gemeldet (attitudeQualityNow() != 1) - dann war es eine ECHTE
    //     Drehung hinter einem Ausfall (z. B. 90-Grad-Wende bei Funkloch), und die
    //     Ausgabe-Ratenbegrenzer glaetten den Nachlauf.
    //   * Erste Sicherung bleibt R1b (PositionHistoryService: der Ring nimmt nur
    //     plausible Fixes) und der Schritt-Test in Abschnitt 1.
    const float chordBody = wrap360(chord - config.dualHeadingOffsetDeg);
    if (chordGateValid) {
        const float dtGate = (float)(now - chordGateMs) * 0.001f;
        float devDeg   = fabsf(wrap180(chordBody - chordGateDeg));
        float limitDeg = 25.0f;                        // 250 Grad/s je Frame
        if (dtGate > 0.15f) limitDeg = 120.0f;         // nach einer Pause
        // LAGE als UNABHAENGIGE GEGENPROBE (B10.47): im Brueckenfall meldete die Lage
        // 25...42 Grad, die gedriftete Sehne 183 Grad (143 Grad Unterschied) - eine
        // ECHTE Drehung verschiebt beide gemeinsam. Die Lage darf dafuer schlecht sein
        // (sie ist nur eine Plausibilitaetsprobe), sie muss nur vorhanden und frisch sein.
        // Bewusst NICHT gegen den eigenen Ausgang geprueft: das war ein Deadlock (jede
        // Ablehnung hielt die Ausgabe, die dann die naechste Sehne erneut ablehnte -
        // gemessen 10-18: Halten 122 -> 621 Frames).
        // B10.50 (2026-10-25): NUR EIN GESUNDER ZEUGE DARF ZEUGEN.
        // attitudeQualityNow() ist genau dieses Urteil (headingQuality == 4, uniHdgStdDev > 0
        // und <= uniHdgSdGoodDeg, |baselineDevPct| <= baselineTolPct): 0 = gut, 1 = degradiert.
        // Gemessen 7-50: bei 1 (sats 0, uni_hdgsd 1120 Grad) sperrte der Zeuge 8 s lang eine
        // Sehne, die aus sauberen Fixes kam. Faellt er weg, bleiben die Gegenprobe
        // "Sehne gegen letzte uebernommene Sehne" (25 Grad/Frame) und der Fix-Schritt-Test.
        if (d.hasDualHeading && (gnssProcessor.getHprAgeMs() <= 300u)) {
            // B10.51: die Sehne ist jetzt Koerperrichtung, die Lage beim Knicklenker
            // Vorderrahmen -> dieselbe Bezugsgroesse herstellen (sonst 22 Grad Fehlurteil).
            const float devAtt = fabsf(wrap180(chordBody - (d.dualHeading - hbArticulationHalfDeg(lastWheelAngleDeg))));
            if (devAtt > devDeg) devDeg = devAtt;
            if (limitDeg > 45.0f) limitDeg = 45.0f;
        }
        if (devDeg > limitDeg) {
            status.pathChordJumpDeg = devDeg;
            s_pcDevDeg = devDeg; s_pcDevLim = limitDeg;
            if (chordGateRejectMs == 0u) chordGateRejectMs = now;
            // B10.50: die Freigabe haengt nicht mehr an der Lage, sondern nur daran, dass die
            // neue Richtung chordGateAcceptMs lang STEHEN bleibt (dieselbe Begruendung wie
            // oben: eine echte Drehung bleibt, ein Sprung kommt und geht).
            const bool attLooksOk = (attitudeQualityNow() != 1u);
            if (!(attLooksOk && ((uint32_t)(now - chordGateRejectMs) >= 3000u))) {
                status.pathChordJumpRejected = true;
                s_pcRej = 8;                   // Sprungtor
                return;                        // kein Kurs aus einem Sprung
            }
        }
        chordGateRejectMs = 0u;
    }
    chordGateDeg = chordBody;
    chordGateMs = now;
    chordGateValid = true;

    if (pathChordPrevValid) {
        const float dtc = (float)(now - pathChordPrevMs) * 0.001f;
        if (dtc > 0.02f) status.pathChordRateDegS = wrap180(chord - pathChordPrevDeg) / dtc;
    }
    pathChordPrevDeg = chord;
    pathChordPrevMs = now;
    pathChordPrevValid = true;
    status.pathChordDeg = chord;
    status.pathChordLenM = distM;
    status.pathChordValid = true;
}

bool FLASHMEM HeadingBridge::updateProjection(uint32_t now, float speedMps, float dtS,
                                     bool allowMeasurement) {
    // The fix is identified by gnssProcessor.getPositionCount() and its age by
    // getPositionAgeMs() - the wall clock is not the fix marker any more (see the
    // note on the counter in GNSSProcessor.h).
    (void)now;
    const auto& d = gnssProcessor.getData();
    status.projCtkM = projCtkM;
    status.projAlongM = projLongM;
    status.projRmsM = projRmsCtkM;
    status.projUpdates = projSamples;
    bool used = false;

    // 1. integrate the predicted position (needs psi and a usable speed)
    if (!projValid) {
        if (d.hasPosition && d.fixQuality > 0 && !isnan(psi) &&
            (speedMps > 0.1f || pubValid)) {
            // Operator correction 2026-09-29 ("der Traktor springt auf der Karte"): this
            // used to SNAP the projection onto the fix without any gate - so after any
            // interruption (fix quality 0, position too old) the next fix was published
            // 1:1, however far it had jumped (measured on the bridge drives: 1.4...2.9 m).
            // When a position was already published, the projection now starts from THAT
            // point instead and ramps onto the fix, so AgOpenGPS sees a continuous track.
            if (pubValid) {
                double sN = 0.0, sE = 0.0;
                localDeltaMeters(pubLat, pubLon, d.latitude, d.longitude, sN, sE);
                const float stepM = (float)sqrt(sN * sN + sE * sE);
                status.posStepPubM = stepM;
                projLat = pubLat;
                projLon = pubLon;
                if (config.posMaxCatchUpMps > 0.0f &&
                    stepM > (config.posMaxCatchUpMps * (float)config.posRampMs * 0.001f)) {
                    // T = 3*residual/vmax - see catchUpRampMs().
                    posRampMsActive = catchUpRampMs(stepM);
                    status.posRampActive = true;
                    posRampStartMs = now;
                    status.posLimited = true;
                    forceLog = true;
                    if (config.logEvents) {
                        LOG_NOTICE(EventSource::GNSS,
                                   "HeadingBridge position re-seed limited: %.2fm residual "
                                   "ramped over %lums (was published as a step before)",
                                   (double)stepM, (unsigned long)posRampMsActive);
                    }
                }
            } else {
                projLat = d.latitude;
                projLon = d.longitude;
            }
            projRefLat = projLat; projRefLon = projLon;
            projPathM = 0.0f;
            projValid = true;
            lastProjFixCount = gnssProcessor.getPositionCount();
        }
        return false;
    }

    // A new fix is identified by the dedicated position counter, NOT by
    // gpsData.lastUpdateTime: that field is also set by GNS/KSXT/INSPVA sentences
    // carrying the same position, and it was the shared lastPosStamp that let
    // updatePositionWindow() eat the fix before we could see it (2026-09-23).
    const bool newFix = (gnssProcessor.getPositionCount() != lastProjFixCount);
    if (!newFix && speedMps > 0.1f && !isnan(psi)) {
        const float rad = psi * 0.017453292519943295f;
        // vScale: the projection must use the same speed the model does, otherwise
        // its along-track residual would measure its own error. While the learned
        // scale is passive (Config::applyLearnedSpeedScale = false) both sides use
        // the receiver's own speed.
        const float vUse = config.applyLearnedSpeedScale ? vScale : 1.0f;
        // Direction of travel (operator rule): psi is the FRONT axis, so while the
        // machine reverses it moves AGAINST psi. projPathM stays the travelled
        // length (positive, it is used as a length and for the noise estimate), the
        // displacement carries the sign.
        const float dirSign = status.reverseDirection ? -1.0f : 1.0f;
        const double stepM = (double)(speedMps * vUse) * (double)dtS;
        const double dN = dirSign * cosf(rad) * stepM;
        const double dE = dirSign * sinf(rad) * stepM;
        const double mPerDegLat = 111320.0;
        const double mPerDegLon = 111320.0 * cos(projLat * 0.017453292519943295);
        projLat += dN / mPerDegLat;
        projLon += dE / mPerDegLon;
        projPathM += (float)stepM;
    }

    // 2. compare with the new fix
    if (!newFix) return false;
    lastProjFixCount = gnssProcessor.getPositionCount();

    if (!d.hasPosition || d.fixQuality <= 0 ||
        gnssProcessor.getPositionAgeMs() > (uint32_t)config.maxPositionAgeMs) {
        projValid = false;              // restart when the stream returns
        return false;
    }

    // --- fix gate (operator request 2026-09-24) -----------------------------
    // A fix that is physically impossible, below RTK fixed or stale must not pull the
    // projection - or AgOpenGPS - around. Measured: 2.2 m of position step at 5.3 m/s
    // on a bridge crossing, and a 5.5 Mm outlier while standing (invalid coordinates).
    // While such a fix is rejected the projection keeps integrating and IS published
    // (getFallbackPosition()), bounded by posFallbackMaxMs/posFallbackMaxM.
    {
        double gN = 0.0, gE = 0.0;
        localDeltaMeters(projLat, projLon, d.latitude, d.longitude, gN, gE);
        const float stepM = (float)sqrt(gN * gN + gE * gE);
        const float dtFix = (lastProjFixMs > 0)
                            ? (float)(now - lastProjFixMs) / 1000.0f : dtS;

        // RTK recovery first: after a bridge the receiver needs seconds to get its
        // ambiguity fix back. Once fixQuality 4 has held for posRtkRecoverMs, that
        // position is the truth and the extrapolation is dropped - otherwise the
        // fall-back would defend its own estimate until the budget ran out.
        if (d.fixQuality >= 4) {
            posRtkGoodMs += (uint32_t)(dtFix * 1000.0f);
        } else {
            posRtkGoodMs = 0;
        }
        if (status.posFallbackActive && posRtkGoodMs >= config.posRtkRecoverMs) {
            status.posFallbackActive = false;
            // Hand over through a RAMP, not a step (operator request): the extrapolation
            // is blended onto the fix over posRampMs, so AOG's cross-track stays
            // continuous even though the fix jumped by up to a metre.
            // Since 2026-09-29 the ramp is sized from the RESIDUAL (stepM), so a bigger
            // jump is also handed over without a step.
            status.posRampActive = true;
            posRampMsActive = catchUpRampMs(stepM);
            posRampStartMs = now;
            posRtkGoodMs = 0;
            lastProjFixMs = now;
            forceLog = true;
            if (config.logEvents) {
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge position fall-back off: RTK recovered after "
                           "%lums / %.1fm - ramping onto the fix (residual %.2fm) over %lums",
                           (unsigned long)status.posFallbackMs,
                           (double)status.posFallbackM, (double)stepM,
                           (unsigned long)config.posRampMs);
            }
            return false;
        }

        const float allowed = config.posSpikeMargin * speedMps * dtFix +
                              config.posSpikeNoiseM;
        // The gate reacts to IMPOSSIBLE positions and to a missing fix - not to a fix
        // that is merely less accurate: an RTK float solution (fixQuality 5) with a
        // plausible step is still better than extrapolating for seconds, so it is
        // accepted and resynchronises. (fixQuality >= 4 is required for MEASUREMENTS -
        // direction, learning - not for following the position.)
        // While the hand-over ramp runs the fix is KNOWN to differ - it must not be
        // judged as unusable then, or the fall-back would restart in a loop.
        // P4.1 (operator request 2026-09-30): the RAMP must not blind the spike test.
        // The residual against the projection legitimately differs while the ramp runs
        // (that is what a ramp is for), but the RECEIVER's own fix-to-fix step must
        // ALWAYS be physically possible. Measured in "Halle 2x": 1.4...3.7 m steps at
        // 1.9...2.2 m/s reached AgOpenGPS with posfall=0 while the ramp was running -
        // the operator's "Sprung in der Position" after the bridge.
        const bool rawSpike = status.posSpikeFix;
        const bool fixUsable = !rawSpike &&
                               (status.posRampActive ||
                                ((d.fixQuality > 0) && (stepM <= allowed)));

        if (!fixUsable) {
            if (status.posRampActive) {
                // P4.1: the ramp was blending onto a fix that turns out to be IMPOSSIBLE
                // (the receiver's own step is discontinuous). Abort it instead of
                // following the bad point - the spike test is applied before the ramp
                // advances, so the ramp has moved at most posMaxCatchUpMps * one fix
                // interval since the release and the published track stays continuous.
                status.posRampActive = false;
                status.posFallbackActive = true;
                status.posFallbackUpdates = 0;
                posFallbackStartMs = now;
                posFallbackStartOdoM = odometerM;
                status.posRampAborts++;
                forceLog = true;
                if (config.logEvents) {
                    LOG_WARNING(EventSource::GNSS,
                                "HeadingBridge ramp ABORTED after %.2fm receiver step "
                                "(qf %u, %.2f m/s) - the projection is published again",
                                (double)status.posStepM, (unsigned)d.fixQuality,
                                (double)speedMps);
                }
            } else if (!status.posFallbackActive) {
                status.posFallbackActive = true;
                status.posRampActive = false;
                posFallbackStartMs = now;
                posFallbackStartOdoM = odometerM;
                status.posFallbackUpdates = 0;
                forceLog = true;
                if (config.logEvents) {
                    LOG_WARNING(EventSource::GNSS,
                                "HeadingBridge position fall-back ON: step=%.2fm allowed "
                                "%.2fm fixQ=%u (projection published for up to %lums/%.0fm)",
                                (double)stepM, (double)allowed, (unsigned)d.fixQuality,
                                (unsigned long)config.posFallbackMaxMs,
                                (double)config.posFallbackMaxM);
                }
            }
            status.posFallbackMs = now - posFallbackStartMs;
            status.posFallbackM = odometerM - posFallbackStartOdoM;
            // Distance limit, tightened by the heading uncertainty: the position error
            // of an extrapolation is travel * sin(sigma_psi). Standing still nothing
            // drifts, so the time limit is not applied there - otherwise a creeping
            // machine could never sit out a tree (0.2 km/h = 0.056 m/s).
            float maxM = config.posFallbackMaxM;
            const float sigmaRad = status.sigmaDeg * 0.017453292519943295f;
            if (sigmaRad > 0.002f) {
                const float driftM = config.posFallbackMaxDriftM / sinf(sigmaRad);
                if (driftM < maxM) maxM = driftM;
            }
            const bool moving = (speedMps > 0.1f);
            const bool overBudget = (status.posFallbackM > maxM) ||
                                    (moving && status.posFallbackMs > config.posFallbackMaxMs);
            if (overBudget) {
                // Hand the receiver's position back - but through the RAMP, so AgOpenGPS
                // does not see a step. AgOpenGPS' own lost-RTK logic takes over from here.
                status.posFallbackActive = false;
                status.posRampActive = true;
                posRampMsActive = catchUpRampMs(stepM);
                posRampStartMs = now;
                forceLog = true;
                if (config.logEvents) {
                    LOG_WARNING(EventSource::GNSS,
                                "HeadingBridge position fall-back OFF after %lums/%.1fm "
                                "- ramping onto the receiver",
                                (unsigned long)status.posFallbackMs,
                                (double)status.posFallbackM);
                }
                return false;
            } else {
                lastProjFixMs = now;
                return false;           // no resync: keep extrapolating
            }
        } else if (status.posFallbackActive && !status.posRampActive) {
            // The fix is usable again; blend onto it instead of snapping.
            status.posFallbackActive = false;
            status.posRampActive = true;
            posRampMsActive = catchUpRampMs(stepM);
            posRampStartMs = now;
            forceLog = true;
            if (config.logEvents) {
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge position fall-back off: fix usable again "
                           "(step %.2fm, fixQ %u, after %lums) - ramping over %lums",
                           (double)stepM, (unsigned)d.fixQuality,
                           (unsigned long)status.posFallbackMs,
                           (unsigned long)config.posRampMs);
            }
        }
        status.posFallbackUpdates++;
    }
    lastProjFixMs = now;

    // --- position ramp (operator request 2026-09-24) -------------------------
    // Blend the extrapolated position onto the newest fix instead of jumping, so
    // AgOpenGPS' cross-track stays continuous. While the ramp runs the residuals are
    // NOT used as measurements - they would measure the ramp, not the model.
    if (status.posRampActive) {
        const float alpha = 3.0f * dtS / ((float)posRampMsActive * 0.001f);
        projLat += alpha * (d.latitude - projLat);
        projLon += alpha * (d.longitude - projLon);
        double rN = 0.0, rE = 0.0;
        localDeltaMeters(projLat, projLon, d.latitude, d.longitude, rN, rE);
        const float restM = (float)sqrt(rN * rN + rE * rE);
        if (restM < 0.05f || (now - posRampStartMs) > 3 * posRampMsActive) {
            status.posRampActive = false;
            projLat = d.latitude; projLon = d.longitude;
            forceLog = true;
            if (config.logEvents) {
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge position ramp finished (residual %.2fm after %lums)",
                           (double)restM, (unsigned long)(now - posRampStartMs));
            }
        }
        // Keep the reference at the blended point: no measurement, no vScale learning.
        projRefLat = projLat; projRefLon = projLon;
        projPathM = 0.0f;
        return false;
    }

    double dN = 0.0, dE = 0.0;
    localDeltaMeters(projLat, projLon, d.latitude, d.longitude, dN, dE);
    status.projDistM = projPathM;
    if (projPathM < 0.05f) {
        // Nothing travelled since the last resync: nothing to learn, just follow.
        projLat = d.latitude; projLon = d.longitude;
        return false;
    }

    const float rad = psi * 0.017453292519943295f;
    const float headN = cosf(rad), headE = sinf(rad);
    // right-hand normal (true north/east frame): (cos, sin) rotated by -90 deg
    const float rightN = headE, rightE = -headN;

    projLongM = (float)(dN * headN + dE * headE);
    projCtkM  = (float)(dN * rightN + dE * rightE);
    status.projCtkM = projCtkM;
    status.projAlongM = projLongM;

    projRmsCtkM += 0.02f * (fabsf(projCtkM) - projRmsCtkM);

    // 2b. speed scale from the ALONG-track residual.
    //     projPathM accumulated v*dt along psi; projLongM is how far the receiver
    //     says we actually moved along psi. Their ratio is the speed scale error.
    //     Averaged slowly and clamped: this feeds the kinematic model (omega is
    //     proportional to v), so a noisy factor here would inject noise into the
    //     heading. Requires a real interval and real speed.
    if (config.learnSpeedScale && speedMps > 0.5f &&
        projPathM >= config.speedScaleMinPathM) {
        const float kv = projLongM / projPathM;
        if (kv > 0.5f && kv < 2.0f) {
            vScaleEma += config.speedScaleAlpha * (kv - vScaleEma);
            vScaleSamples++;
            if (vScaleSamples >= config.speedScaleMinSamples) {
                float vs = vScaleEma;
                const float lo = 1.0f - config.speedScaleLimit;
                const float hi = 1.0f + config.speedScaleLimit;
                if (vs < lo) vs = lo;
                if (vs > hi) vs = hi;
                vScale = vs;
            }
        }
    }

    // 3. heading measurement from the cross-track residual
    //    sigma_dir = position noise / travelled distance, floor from the same
    //    physical limit the fix-to-fix window uses.
    if (projPathM >= config.minDisplacementForFixToFixM) {
        const float rDeg = 57.29577951308232f *
                           atanf(config.positionSigmaM / projPathM);
        float rUse = rDeg;
        // A large along-track residual means the interval is not described by the
        // model (speed error, slip, position fault), so the cross-track part is less
        // trustworthy too - weight it accordingly instead of using a constant R.
        rUse *= (1.0f + fabsf(projLongM) / projPathM);
        if (rUse < config.rFixToFixDeg * 0.5f) rUse = config.rFixToFixDeg * 0.5f;
        if (rUse > config.rFixToFixDeg * 3.0f) rUse = config.rFixToFixDeg * 3.0f;
        const float meas = wrap360(psi - 57.29577951308232f *
                                   atan2f(projCtkM, projPathM));
        status.projHeadingDeg = meas;
        status.projRdeg = rUse;
        const float innov = fabsf(wrap180(meas - psi));
        const float blindS = (float)(now - lastMeasurementMs) / 1000.0f;
        const float innovLimit = config.pathInnovationBaseDeg +
                                 config.pathInnovationPerS * blindS;
        if (innov <= innovLimit && allowMeasurement) {
            applyMeasurement(meas, rUse, 2);
            status.pathProjUpdates++;
            used = true;
            imuAnchorMs = now;    // the projected path is an absolute anchor too (2026-09-29)
            if (!status.attitudeTrusted) {
                status.source = Source::PATH_PROJECTED;
                lastMeasurementMs = now;
            }
        }
    }
    projSamples++;

    // 4. resync the projection to the measured position
    //    Operator correction 2026-09-29 ("der Traktor springt auf der Karte"): a resync
    //    is what the PUBLISHED point follows, so it must never be a step. If the fix is
    //    farther from what was published than the catch-up limit allows, the projection
    //    keeps being published and the ramp walks onto the fix instead of jumping -
    //    bounded by posMaxCatchUpMps/posRampMaxMs (a 2.9 m bridge jump becomes ~1.9 s of
    //    smooth travel at 1.5 m/s instead of 100 ms of teleporting).
    {
        const double resN = dN, resE = dE;
        const float resM = (float)sqrt(resN * resN + resE * resE);
        status.posStepPubM = resM;
        const float catchUpPerRamp = (config.posMaxCatchUpMps > 0.0f)
            ? (config.posMaxCatchUpMps * (float)config.posRampMs * 0.001f)
            : 1.0e9f;
        if (config.posMaxCatchUpMps > 0.0f && resM > catchUpPerRamp) {
            // T = 3*residual/vmax keeps the blend's initial rate at exactly vmax - see
            // catchUpRampMs().
            posRampMsActive = catchUpRampMs(resM);
            status.posRampActive = true;
            status.posLimited = true;
            posRampStartMs = now;
            forceLog = true;
            if (config.logEvents) {
                LOG_NOTICE(EventSource::GNSS,
                           "HeadingBridge position limited: %.2fm residual between the fix "
                           "and the published point (fixQ %u, %u sats) - ramped over %lums "
                           "instead of published as a step",
                           (double)resM, (unsigned)d.fixQuality,
                           (unsigned)d.numSatellites, (unsigned long)posRampMsActive);
            }
            projRefLat = projLat; projRefLon = projLon;
            projPathM = 0.0f;
            return used;
        }
        projLat = d.latitude; projLon = d.longitude;
    }
    projRefLat = projLat; projRefLon = projLon;
    projPathM = 0.0f;
    return used;
}

// ---------------------------------------------------------------------------
// Direction-of-travel witness from the POSITIONS (operator decision 2026-09-29)
// ---------------------------------------------------------------------------
// Config-independent and impossible to mirror: it is the direction of the NET
// DISPLACEMENT between two fixes. Its job is not to say "forward or reverse" (both the
// heading and the travel vector can be mirrored by the firmware) but to say whether the
// travel VECTOR a reversal decision is based on deserves to be believed. The 06:31 log
// shows the failure it prevents: a bogus 0.25 m baseline at 0.47 m/s decided REVERSE, and
// everything afterwards confirmed itself for 50 s.
// Conditions: at least travelWitnessMinM of net displacement while moving, and a window
// that is straight enough that the chord IS the direction of travel.
void FLASHMEM HeadingBridge::updateTravelWitness(uint32_t now) {
    const auto& d = gnssProcessor.getData();
    status.travelWitnessValid = false;
    if (!d.hasPosition || d.fixQuality <= 0 ||
        gnssProcessor.getPositionAgeMs() > (uint32_t)config.maxPositionAgeMs) {
        twRefValid = false;
        return;
    }
    if (lastSpeedMps < config.travelWitnessSpeedMps) {
        return;                    // creeping: the witness says nothing, keep the last value
    }
    if (!twRefValid) {
        twRefLat = d.latitude; twRefLon = d.longitude;
        twRefMs = now; twRefHeadingDeg = psi;
        twRefValid = true;
        return;
    }
    double wN = 0.0, wE = 0.0;
    localDeltaMeters(twRefLat, twRefLon, d.latitude, d.longitude, wN, wE);
    const float net = (float)sqrt(wN * wN + wE * wE);
    if (net < config.travelWitnessMinM) {
        // Not enough displacement yet - keep the last VALID value (F2, 2026-10-01). The
        // window is NOT re-based on psi any more: doing that made the witness blind
        // exactly when it was needed (measured: travel_wit = -1 for 60 s while the
        // positions were unambiguous, because psi itself had drifted away).
        return;
    }
    const float dtW = (float)(now - twRefMs) / 1000.0f;
    // Same speed consistency check the direction state uses: a jumping fix keeps the
    // distance plausible while pointing anywhere.
    if (dtW > 0.05f) {
        const float expect = lastSpeedMps * dtW;
        const float tol = config.dirSpeedTolFrac * expect + config.dirSpeedTolM;
        if (fabsf(net - expect) > tol) {
            twRefLat = d.latitude; twRefLon = d.longitude;
            twRefMs = now; twRefHeadingDeg = psi;
            return;
        }
    }
    // B10.51: DERSELBE Antennen-Vorlauf wie in der Sehne. Der Witness ist ein ZWEITER
    // Positionskurs-Pfad ueber dieselbe Antenne - ohne diese Korrektur liefern Sehne und
    // Witness unterschiedliche Koerperrichtungen, und der Quellenwechsel springt genau um
    // diese Differenz (gemessen 7-52: skew p50 +5,9 / p90 18,8 Grad).
    const float travel = wrap360(atan2f((float)wE, (float)wN) * 57.29577951308232f -
                                 hbAntennaLeadDeg(status.yawRateDegS, lastSpeedMps));
    // F2 (2026-10-01): the straightness test uses the CHANGE OF THE TRAVEL DIRECTION
    // itself, not psi. Using psi made the witness circular - it was only valid while the
    // heading agreed with it, i.e. never while the heading was wrong (the exact case it
    // exists for). A window whose own direction is consistent IS a direction of travel.
    if (fabsf(wrap180(travel - twRefHeadingDeg)) <= config.travelWitnessMaxTurnDeg) {
        status.travelWitnessDeg = travel;
        status.travelWitnessValid = true;
    }
    // Re-base in every case: the next window starts here. twRefHeadingDeg now carries the
    // TRAVEL direction (it used to carry psi).
    twRefLat = d.latitude; twRefLon = d.longitude;
    twRefMs = now; twRefHeadingDeg = travel;
}

// ---------------------------------------------------------------------------
// Which fall-back measurement enters the filter this cycle
// ---------------------------------------------------------------------------
// Preference: position derived direction, then COG. Both are cross-checked:
//
//   * against each other - they describe the same path, so a large disagreement
//     means one of them is broken. The COG needs no displacement at all, so it is
//     the safer choice in that case,
//   * against the fused state. The allowance grows with the time since the last
//     measurement, because after a long outage the STATE is the wrong one and the
//     measurement must be allowed to pull it back. Same reasoning as the
//     heading-step gate in evaluateAttitudeFrame().
void FLASHMEM HeadingBridge::selectFallbackMeasurement(uint32_t now) {
    // Below ~0.3 m/s neither the COG (needs motion) nor the position window
    // (needs displacement) says anything about the heading, and a standing machine
    // does not yaw - so the estimate is HELD. Measured on the 2026-09-23 logs: the
    // COG moves by 3.1 deg per 100 ms at 0.3 ... 1.5 m/s and by up to 20 deg per
    // 0.5 s at standstill, i.e. injecting it there is pure noise, and it also makes
    // the state machine chatter.
    if (lastSpeedMps < config.minSpeedForMeasurement) {
        status.source = Source::DEADRECKON;
        return;
    }

    const float blindS = (float)(now - lastMeasurementMs) / 1000.0f;
    const float innovLimit = config.pathInnovationBaseDeg +
                             config.pathInnovationPerS * blindS;

    const bool cogPathAgree = (!pathValid || !cogValid) ||
                              (fabsf(wrap180(pathMeas - cogMeas)) <=
                               config.cogPathDisagreeDeg);

    // The path derived directions describe the PATH, the filter state the BODY.
    // Applying the measured side-slip offset turns them into body measurements, so
    // a source switch (attitude gone / attitude back) no longer shows up as a step.
    const float slip = (config.estimateSideSlip && sideSlipSamples > 200)
                       ? sideSlipDeg : 0.0f;
    // While the machine drives BACKWARDS the path points against the front axis, so
    // the path measurements are mirrored by the same 180 deg that AgOpenGPS adds in
    // its own single-antenna path (Position.designer.cs:500). The published heading
    // must stay the vehicle FRONT axis (operator rule 2026-09-23): driving forward
    // or backwards along a straight line does not change it.
    const float mirror = status.reverseDirection ? 180.0f : 0.0f;

    if (pathValid && config.enablePathMeasurement && cogPathAgree) {
        const float meas = wrap360(pathMeas + slip + mirror);
        const float innov = fabsf(wrap180(meas - psi));
        if (innov <= innovLimit) {
            applyMeasurement(meas, effectiveMeasurementR(1), 1);
            status.fixToFixUpdates++;
            status.source = Source::PATH_FIX2FIX;
            lastMeasurementMs = now;
            imuAnchorMs = now;      // absolute anchor for the IMU offset/rate bias (2026-09-29)
            return;
        }
        // Beyond the physical allowance: treat it as a position fault and fall
        // through to the COG.
        status.posFaults++;
    }

    // IMU heading (offset-corrected) - better than the COG at low speed, which is
    // exactly where the COG is worst. It comes after the geometrically exact
    // fix-to-fix window and before the course over ground.
    // kind 3 marks it as a BODY-referenced measurement, which is what allows the
    // re-seed guard in applyMeasurement() to use it (with the position witness as the
    // confirmation) - operator decision 2026-09-29 for the single-antenna case.
    if (imuHdgHaveFresh && config.useImuHeading &&
        fabsf(wrap180(imuHeadingMeasDeg - psi)) <= innovLimit + imuHeadingMeasR) {
        applyMeasurement(imuHeadingMeasDeg, imuHeadingMeasR, 3);
        status.imuHeadingUsed = true;
        status.source = Source::PATH_IMU_HEADING;
        lastMeasurementMs = now;
        return;
    }

    if (cogValid && config.enableCogMeasurement) {
        const float meas = wrap360(cogMeas + slip + mirror);
        const float innov = fabsf(wrap180(meas - psi));
        if (innov <= innovLimit) {
            // P4.3 (operator request 2026-09-30): damp the raw course below
            // Config::cogRefSpeedMps instead of dropping it - its direction error grows
            // as ~1/v (measurements in the config comment). The GAIN shrinks with the
            // square of this factor, so the COG-driven jitter at 1.1...1.3 m/s (8
            // filter-only steps up to 9.4 deg in the 2026-09-30 bridge log) cannot
            // happen any more, while the source stays "cog" and the slow side-slip
            // estimate keeps its input.
            float rCog = effectiveMeasurementR(1);
            if (lastSpeedMps < config.cogRefSpeedMps) {
                const float vRef = (lastSpeedMps > 0.2f) ? lastSpeedMps : 0.2f;
                float f = config.cogRefSpeedMps / vRef;
                if (f > config.cogMaxNoiseFactor) f = config.cogMaxNoiseFactor;
                if (f > 1.0f) rCog *= f;
            }
            applyMeasurement(meas, rCog, 1);
            status.cogUpdates++;
            status.source = Source::PATH_COG;
            lastMeasurementMs = now;
            imuAnchorMs = now;      // absolute anchor for the IMU offset/rate bias (2026-09-29)
            return;
        }
    }

    // Nothing usable: dead reckoning. This is the only phase in which the estimate
    // can drift, and it is what the dead-reckoning budget in updateState() counts.
    status.source = Source::DEADRECKON;
}

// ---------------------------------------------------------------------------
// Online measurement of the antenna mounting offset
// ---------------------------------------------------------------------------
// az_reported = body - mountOffset  =>  mountOffset = az_reported - body. The body
// heading is not known directly, but the PATH direction is a good stand-in while
// driving (side slip moves body and path apart by 1 ... 3 deg, which this estimate
// absorbs - acceptable, because the operator only needs the mounting constant to
// set AgOpenGPS, and AOG's own offset then applies to the same reference).
//
// Circular mean by accumulation of sin/cos, so no wrap-around artefacts. The
// result is what the operator types into AgOpenGPS as the dual heading offset,
// with the sign AOG expects (AOG does received + offset).
void FLASHMEM HeadingBridge::updateMountOffset(uint32_t now, float speedMps) {
    if (!haveTrusted || speedMps < config.mountOffsetMinSpeed) return;
    // The reference has to be trustworthy. Without a TRUSTED attitude the path sources
    // drive psi, and in a hall/under trees those are exactly what is wrong - measured
    // 2026-09-27: the measured offset wandered from -88 deg to -76/-78 deg over the
    // hall logs because the slow EMA kept eating those phases. The value is a
    // dual-antenna diagnostic, so requiring the dual attitude is the honest gate.
    if (!status.attitudeTrusted) return;
    // Reference is the FRONT axis of the machine, and only from a source that can be
    // trusted for it: the position-derived direction when the window is valid
    // (0.5 m + straightness gate), otherwise the COG - but only at a speed where the
    // VTG course is trustworthy. Measured in the hall logs (2026-09-23): the COG was
    // inverted by 180 deg at 0.5...1.5 m/s under a roof and pulled the measured
    // offset from -88.8 to -83 deg, i.e. it faked a 6 deg mounting error.
    float refDir = -1.0f;
    if (pathValid) {
        refDir = pathMeas;
    } else if (cogValid && speedMps > config.minSpeedForCogCheck) {
        refDir = cogMeas;
    }
    if (refDir < 0.0f) return;
    // Reference is the FRONT axis, not the direction of travel: while reversing the
    // path points the other way, and without this the measured offset would swing by
    // 180 deg whenever the driver backs up. The front axis is what the antennas are
    // bolted to, so it is the only reference that gives a constant.
    if (status.reverseDirection) refDir = wrap360(refDir + 180.0f);

    const float azRaw = wrap360(gnssProcessor.getData().dualHeading);  // raw azimuth
    const float diff = wrap180(azRaw - refDir) * 0.017453292519943295f;
    const float dtS = 0.01f;
    const float a = (config.mountOffsetTauS > 0.5f)
                    ? (dtS / config.mountOffsetTauS) : 0.01f;

    // First sample seeds directly, so the value is usable immediately.
    if (status.mountOffsetSamples == 0) {
        mountOffsetSin = sinf(diff);
        mountOffsetCos = cosf(diff);
    } else {
        mountOffsetSin += (sinf(diff) - mountOffsetSin) * a;
        mountOffsetCos += (cosf(diff) - mountOffsetCos) * a;
    }
    status.mountOffsetSamples++;
    status.mountOffsetMeasuredDeg =
        atan2f(mountOffsetSin, mountOffsetCos) * 57.29577951308232f;

    // Sign check against the configured offset. az = body + offset (AOG convention)
    // and mnt = az - front axis, so the correct offset is the NEGATIVE of the
    // measurement. Equal signs mean the antennas sit the other way round than the
    // configuration assumes - the published heading then points backwards, which the
    // operator sees as "AgOpenGPS shows reverse while driving forward" (AOG's own
    // reverse detection compares direction of travel against that heading). Reported
    // once per change, never corrected silently.
    if (status.mountOffsetSamples > 200) {
        const bool signOk = ((status.mountOffsetMeasuredDeg > 0.0f) !=
                             (config.dualHeadingOffsetDeg > 0.0f));
        if (signOk != status.mountOffsetSignOk) {
            status.mountOffsetSignOk = signOk;
            forceLog = true;
            if (config.logEvents) {
                if (signOk) {
                    LOG_NOTICE(EventSource::GNSS,
                               "HeadingBridge mounting offset sign ok again "
                               "(measured %+.1f deg -> offset %+.1f)",
                               (double)status.mountOffsetMeasuredDeg,
                               (double)(-status.mountOffsetMeasuredDeg));
                } else {
                    LOG_WARNING(EventSource::GNSS,
                                "HeadingBridge mounting offset SIGN MISMATCH: measured "
                                "%+.1f deg (az - front axis) -> dualHeadingOffsetDeg must "
                                "be %+.1f, configured %+.1f. Antennas swapped? The "
                                "heading sent to AOG points backwards.",
                                (double)status.mountOffsetMeasuredDeg,
                                (double)(-status.mountOffsetMeasuredDeg),
                                (double)config.dualHeadingOffsetDeg);
                }
            }
        }
    }
    (void)now;
}

// ---------------------------------------------------------------------------
// Throttled diagnostic line. NOTICE on purpose: the INFO token bucket is already
// over-subscribed by WheelAngleFusion/HeadingMonitor (see HeadingMonitor.cpp for
// the arithmetic), and the bridge diagnostics must not be starved by it.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Raw frame log: one CSV line per fix, straight to the USB serial port.
// ---------------------------------------------------------------------------
// Deliberately NOT through the EventLogger: the logger rate-limits per severity
// (10 messages/s for INFO) and prefixes every line with a timestamp, which is
// useless for bulk data and would starve the real events. HeadingMonitor does the
// same for its ring-buffer dumps.
//
// It also never blocks: usb_serial_write() waits up to 120 ms when the TX buffer
// is full (usb_serial.c:281), so the line is only produced while the driver
// reports room for it. With no host attached availableForWrite() stays at 0 and
// the log silently pauses instead of stalling the steering loop.
//
// Columns (all CSV, no header - the header is printed once by the caller/README):
//   t_ms, lat, lon, v_mps, cog_deg, att_hdg_deg, roll_deg, qf, sats, fixQ,
//   wheel_deg, psi_deg, sigma_deg, bias_deg, src, state, f2f_deg, proj_ctk_m,
//   proj_rng_m, proj_along_m, proj_r_deg, mount_deg, sc, slip_deg, baseline_m,
//   baseline_dev_pct, rej, pf, gap_ms, vscale,
//   proj_upd (accepted projection updates since boot), fix_cnt (deduplicated
//   position fixes since boot), pos_age_ms, sc_n (steer-scale samples),
//   sc_spread, vsc_n (speed-scale samples), rev (1 = reversing, the path sources
//   are mirrored by 180 deg), dirchg (1 = direction change in progress, AOG's
//   byPass state), dirdev (filtered angle heading <-> direction of travel),
//   dirref_m, dirsigma_m, posfall (1 = position fall-back published), posfall_m,
//   imu_rate (deg/s), imu_bias (deg/s), imu_hdg, imu_hdg_off, roll_bridge_deg,
//   az_out (what leaves the module for AOG: psi - dualHeadingOffsetDeg, i.e. the
//   value AOG turns into the body heading with the SAME offset), aog_tgt_deg, pwm, guid,
//   sc_b (the wheel angle offset the model APPLIES - 0 by design, the WAS owns the zero,
//   so a non-zero value is a fault), mdev (model gate deviation in
//   degrees), mgate (1 = the model gate holds R elevated). sc_b/mdev/mgate were
//   added 2026-09-27 with the geometry estimator and the gate.
// Columns 60...69 were added 2026-09-29 after the operator asked for the fields that
// answer "which measurement moved the heading, and did a position jump reach AOG":
//   psi_att (psi - the attitude-derived body heading: must stay CONSTANT - a swing is what
//   AgOpenGPS turns into "reverse while driving forward"), innov (the innovation applied
//   this tick, signed deg), gain (the Kalman K1 that was applied to it), meas (which
//   measurement fed the filter: 0 none, 1 attitude, 2 path, 3 COG, 4 IMU, 5 projected),
//   travel (the direction of travel the direction decision used), raw_step (fix-to-fix step
//   of the receiver's position, m), pub_step (step of the PUBLISHED position, m - the jump
//   AOG actually sees), publim (1 = the published position is the projection), hdg_res
//   (heading re-seeds: the state was >120 deg wrong against a trusted attitude),
//   dir_veto (reverse flips rejected by the attitude witness).
// proj_upd/fix_cnt were added 2026-09-23: before the fix-signal repair the
// projection resynchronised ~once per drive while the position stream ran at
// 10 Hz - proj_upd/s versus fix_cnt/s is the direct check that it does not
// silently freeze again.
// Kept in the FLASH on purpose (FLASHMEM): the diagnostic output is not timing
// critical, and RAM1 (ITCM) is the scarce region on this board - measured
// 2026-09-23, the module tipped over the ITCM limit by 21 kB of alignment padding
// with the additions for the hall-log fixes.
void FLASHMEM HeadingBridge::logRawFrame(uint32_t now, float speedMps, float wheelAngleDeg) {
#if defined(AIO_HR_LOG) && ((AIO_HR_LOG == 0) || (AIO_HR_LOG == 2))
    // B10.49 (2026-10-19, Betreiber): HR-Zeile AUS (Schalter in platformio.ini).
    //   AIO_HR_LOG 0 = alles aus (Produktiv), 1 = nur HR, 2 = nur PH, 3 = beides.
    (void)now; (void)speedMps; (void)wheelAngleDeg;
    return;
#endif
    if (!config.logRawFrames) return;
    if ((now - lastRawLogMs) < config.rawLogIntervalMs) return;
    if (Serial.availableForWrite() < 256) return;   // never wait for the host
    lastRawLogMs = now;

    const auto& d = gnssProcessor.getData();
    // B10.49: NICHT mit Serial.printf schreiben. Die Zeile ist ~700 Byte lang, der
    // Puffer-Check oben prueft aber nur 256 - printf blockiert dann MITTEN in der Zeile,
    // bis der Host liest. Ohne angeschlossenen Host schreibt Teensy sofort ins Leere,
    // mit Terminal haengt der aufrufende Task (Betreiber-Beobachtung 10-19).
    // B10.49 (2026-10-20): Die Zeile wird in VIER Haeppchen gebaut. Grund (gemessen):
    // 120 Argumente in EINEM snprintf bedeuten ~1 KB vararg-Spill auf dem Stack, plus
    // newlib - das sprengt den RAM1-Stack (dort sind nur ~3,2 KB frei) und das Board
    // startet neu, BEVOR die erste Zeile fertig ist. Die kurze PH-Zeile (27 Argumente)
    // lief im selben Build 24 s stabil, nur die HR-Zeile starb sofort.
    // Je Haeppchen ~30 Argumente (~250 B Spill), Schnittstelle ist immer ein Komma -
    // die Zeile selbst bleibt Zeichen fuer Zeichen dieselbe (121 Namen = 120 Werte).
    DMAMEM static char hrBuf[1024];
    int used = 0;
    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  "HR,%lu,%.7f,%.7f,%.3f,%.2f,%.2f,%.2f,%u,%u,%u,%.2f,%.2f,%.2f,%+.2f,%s,",
                  (unsigned long)now, d.latitude,
                  d.longitude, (double)speedMps,
                  (double)d.headingTrue, (double)d.dualHeading,
                  (double)d.dualRoll, (unsigned)d.headingQuality,
                  (unsigned)d.numSatellites, (unsigned)d.fixQuality,
                  (double)wheelAngleDeg, (double)status.vehicleHeadingDeg,
                  (double)status.sigmaDeg, (double)status.steerBiasDeg,
                  sourceName(status.source));
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  "%s,%.2f,%.3f,%.2f,%.3f,%.2f,%+.1f,%.2f,%+.2f,%.3f,%+.1f,%s,%lu,%lu,%.3f,",
                  stateName(status.state), (double)status.pathLastDeg,
                  (double)status.projCtkM, (double)status.projDistM,
                  (double)status.projAlongM, (double)status.projRdeg,
                  (double)status.mountOffsetMeasuredDeg, (double)status.steerScaleEstimate,
                  (double)sideSlipDeg, (double)status.baselineLengthM,
                  (double)status.baselineDevPct, reasonName(status.lastReject),
                  (unsigned long)status.posFaults, (unsigned long)status.gapThresholdMs,
                  (double)vScale);
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  "%lu,%lu,%lu,%lu,%.4f,%lu,%u,%u,%.1f,%.2f,%.2f,%u,%.1f,%+.1f,%+.2f,",
                  (unsigned long)status.projUpdates, (unsigned long)gnssProcessor.getPositionCount(),
                  (unsigned long)gnssProcessor.getPositionAgeMs(), (unsigned long)status.steerScaleLearnSamples,
                  (double)status.steerScaleSpread, (unsigned long)vScaleSamples,
                  (unsigned)status.reverseDirection, (unsigned)status.dirChanging,
                  (double)status.dirDevDeg, (double)status.dirRefDistM,
                  (double)status.dirSigmaM, (unsigned)status.posFallbackActive,
                  (double)status.posFallbackM, (double)status.imuRateDegS,
                  (double)status.imuYawBiasDegS);
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  "%+.1f,%+.1f,%.1f,%.1f,%+.1f,%d,%u,%+.2f,%.2f,%u,%.1f,%d,%.2f,%+.1f,%+.2f,",
                  (double)status.imuHeadingDeg, (double)status.imuHdgOffsetDeg,
                  (double)status.rollDeg, (double)status.azimuthDeg,
                  (double)status.aogTargetDeg, (int)status.pwmOut,
                  (unsigned)(status.guidanceActive ? 1 : 0), (double)status.steerOffsetEstimateDeg,
                  (double)status.modelDevDeg, (unsigned)(status.modelGateActive ? 1 : 0),
                  (double)status.yawRateDegS, (int)status.wheelAngleQuality,
                  (double)d.hdop, (double)status.psiAttDevDeg,
                  (double)status.lastInnovDeg);
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    // --- raw_step / pub_step: Positionsschritt zwischen zwei LOG-Zeilen --------
    // (2026-11-02) Die beiden Felder wurden bisher in update() berechnet - und
    // kamen im Log trotzdem in ALLEN 262 Zeilen des 01.11.-Logs als 0.00 an,
    // auch in den 219 Zeilen mit v > 0.3 m/s. Die Ursache ist ohne Hardware nicht
    // abschliessend zu klaeren; BELEGT ist dagegen, dass pub_lat/pub_lon
    // (Spalten 99/100) und lat/lon (2/3) in jeder Zeile echte, wandernde Werte
    // tragen. Deshalb wird der Schritt hier aus genau diesen Werten gegen den
    // Stand der VORIGEN Log-Zeile gebildet.
    //
    // Das ist inhaltlich sogar das, was die Spalte verspricht: der Sprung, den
    // ein Log-Leser sieht - nicht der zwischen zwei Empfaenger-Fixes, die im Log
    // gar nicht alle auftauchen.
    static double hrPrevPubLat = 0.0, hrPrevPubLon = 0.0;
    static double hrPrevFixLat = 0.0, hrPrevFixLon = 0.0;
    static bool   hrPrevValid  = false;
    if (hrPrevValid) {
        double n = 0.0, e = 0.0;
        localDeltaMeters(hrPrevPubLat, hrPrevPubLon, pubLat, pubLon, n, e);
        status.posStepPubM = (float)sqrt(n * n + e * e);
        localDeltaMeters(hrPrevFixLat, hrPrevFixLon, d.latitude, d.longitude, n, e);
        status.posStepRawM = (float)sqrt(n * n + e * e);
    }
    hrPrevPubLat = pubLat;     hrPrevPubLon = pubLon;
    hrPrevFixLat = d.latitude; hrPrevFixLon = d.longitude;
    hrPrevValid  = true;

    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  "%.3f,%u,%.1f,%.2f,%.2f,%u,%u,%u,%+.1f,%u,%lu,%u,%.1f,%+.1f,%u,",
                  (double)status.lastGainK, (unsigned)status.lastMeasKind,
                  (double)status.dirTravelDeg, (double)status.posStepRawM,
                  (double)status.posStepPubM, (unsigned)(status.posLimited ? 1 : 0),
                  (unsigned)status.headingReseeds, (unsigned)status.dirVetoes,
                  (double)wrap180(psi - status.imuHeadingDeg), (unsigned)status.imuFlags,
                  (unsigned long)status.imuHeadingAgeMs, (unsigned)imuProcessor.getCurrentData().quality,
                  (double)imuProcessor.getCurrentData().heading, (double)status.imuRollDeg,
                  (unsigned)status.vwasHeadingSource);
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  "%u,%u,%+.1f,%u,%u,%u,%u,%.2f,%u,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,",
                  (unsigned)status.rollSource, (unsigned)(cogValid ? 1 : 0),
                  (double)(status.travelWitnessValid ? status.travelWitnessDeg : -1.0f), (unsigned)((d.hasDualHeading || d.hasINS) ? 2 : (gnssProcessor.hasGPS() ? 1 : 0)),
                  (unsigned)((d.hasDualHeading ? 1 : 0) | (d.hasINS ? 2 : 0) |                              (gnssProcessor.hasGPS() ? 4 : 0) |                              (gnssProcessor.hasFix() ? 8 : 0)), (unsigned)(status.rollSuspect ? 1 : 0),
                  (unsigned)(status.yawRateZeroed ? 1 : 0), (double)status.rateLimitAllowedDegS,
                  (unsigned)status.rateLimitClipped, (double)status.rateLimitPathDegS,
                  (double)status.lastMeasRDeg, (double)d.uniLengthM,
                  (double)d.uniHeadingDeg, (double)d.uniPitchDeg,
                  (double)d.uniHdgStdDevDeg);
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  "%.2f,%d,%u,%.2f,%.2f,%.2f,%u,%.2f,%.7f,%.7f,%.3f,%ld,%.2f,%.2f,%.2f,",
                  (double)d.uniPtchStdDevDeg, (int)status.attQualGood,
                  (unsigned)status.outSnapSrc, (double)status.outSmoothDeg,
                  (double)outResidDeg, (double)status.outResidLimDegS,
                  (unsigned)(status.psiNetClip ? 1 : 0), (double)status.modelSuppressedDegS,
                  (double)pubLat, (double)pubLon,
                  (double)status.nmeaDiffM, (long)status.wheelObs.encRaw,
                  (double)status.wheelObs.encoderDeg, (double)status.wheelObs.offDeg,
                  (double)status.wheelObs.residualDeg);
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  "%u,%u,%.3f,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.1f,%u,%.1f,%u,%.2f",
                  (unsigned)status.wheelObs.zoState, (unsigned)status.wheelObs.fits,
                  (double)status.wheelObs.gainDegPerDeg, (double)status.wheelObs.rollMeanDeg,
                  (double)status.wheelObs.acquireRate, (double)status.wheelObs.gpsAngleDeg,
                  (unsigned)(status.wheelObs.aogZero ? 1 : 0), (double)status.wheelObs.aogWasOff,
                  (double)status.wheelObs.cpdAog, (double)status.vwasTrimAppliedDeg,
                  (double)status.pathChordDeg, (unsigned)(status.pathChordValid ? 1 : 0),
                  (double)status.pathChordLagDeg, (unsigned)status.azSrc,
                  (double)status.pathChordLenM);
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    // B10.52: die FUENF Zusatzwerte als EIGENES Haeeppchen (5 Argumente). Die 120 bestehenden
    // Argumente bleiben unangetastet - der Stack-Spill pro Aufruf wird dadurch KLEINER, nicht
    // groesser (Lehre aus B10.49: 120 Argumente in EINEM snprintf = Bootloop ueber den RAM1-Stack).
    used += snprintf(hrBuf + used, sizeof(hrBuf) - (size_t)used,
                  ",%u,%.2f,%.2f,%.2f,%u,%.2f,%.2f\r\n",
                  (unsigned)s_pcRej, (double)s_pcDevDeg,
                  (double)s_antLeadDeg, (double)s_artHalfDeg,
                  (unsigned)(hbIsArticulated() ? 2u : 0u), (double)s_rateRawDegS,
                  (double)s_crResidDeg);
    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;

    if (used <= 0 || used >= (int)sizeof(hrBuf)) return;
    if (Serial.availableForWrite() < used + 16) return;
    Serial.write((const uint8_t*)hrBuf, (size_t)used);
}

// ---------------------------------------------------------------------------
// Throttled diagnostic output. NOTICE on purpose: the INFO token bucket is
// already over-subscribed by WheelAngleFusion/HeadingMonitor (see
// HeadingMonitor.cpp for the arithmetic), and the bridge diagnostics must not be
// starved by it.
//
// TWO lines of <= ~120 characters each instead of one long line: the log viewer,
// the USB console and the optional syslog forwarder all handle short lines much
// better, and a line that wraps is unreadable exactly when it matters. Line 1 is
// the state (what the heading IS), line 2 the evidence (why it is trusted or
// not). Line 2 is only emitted while the bridge is active or when the attitude is
// degraded - in normal LIVE operation the state line is enough.
// ---------------------------------------------------------------------------
void FLASHMEM HeadingBridge::logDebugLine(uint32_t now, float speedMps) {
    if (quiet) return;
    // 0.5 Hz, PLUS immediately on a state change (short dropouts must not fall
    // between two lines) and on a NEW position fault (the counter is cumulative,
    // so comparing against the last logged value is what keeps this honest - the
    // 2026-09-23 logs printed HB2 on every single HB1 because of the stale value).
    const bool forced = forceLog;
    if (!forced && (now - lastLogMs) < 2000u) return;
    forceLog = false;
    lastLogMs = now;
    const auto& d = gnssProcessor.getData();

    // Line 1: state + the two calibration numbers (<= ~105 chars payload).
    // The mount offset and the scale recommendation belong here and not in the
    // degraded-only line 2, because both need a healthy attitude to be measured -
    // they would otherwise never appear in a log.
    LOG_NOTICE(EventSource::GNSS,
               "HB1 %s/%s psi=%.1f az=%.1f roll=%.1f sig=%.1f bias=%+.1f "
               "age=%lu dist=%.1f yaw=%.1f v=%.1f mnt=%+.1f sc=%.2f",
               stateName(status.state), sourceName(status.source),
               (double)status.vehicleHeadingDeg, (double)status.azimuthDeg,
               (double)status.rollDeg, (double)status.sigmaDeg,
               (double)status.steerBiasDeg, (unsigned long)status.attitudeAgeMs,
               (double)status.bridgeDistanceM, (double)status.yawRateDegS,
               (double)speedMps, (double)status.mountOffsetMeasuredDeg,
               (double)status.steerScaleEstimate);

    // Line 2: evidence (<= ~120 characters of payload). Skipped while everything is
    // fine - in that case the raw attitude IS the value that was used, and the raw
    // frame log carries the numbers anyway.
    const bool degraded = forced || !status.attitudeTrusted ||
                          status.lastReject != RejectReason::NONE ||
                          status.posFaults != lastLoggedPosFaults;
    if (!degraded) return;
    lastLoggedPosFaults = status.posFaults;

    LOG_NOTICE(EventSource::GNSS,
               "HB2 raw=%.0f/%u/%u cog=%.0f f2f=%.0f rng=%.1f str=%.2f "
               "rej=%s pf=%lu gap=%lu bsl=%+.1f sc=%.2f sl=%+.1f vs=%.3f r=%u/%.1f "
               "psi_att=%+.1f res=%u veto=%u",
               (double)d.dualHeading, (unsigned)d.headingQuality,
               (unsigned)d.numSatellites,
               (double)status.cogLastDeg, (double)status.pathLastDeg,
               (double)status.pathRangeM, (double)status.pathStraightness,
               reasonName(status.lastReject), (unsigned long)status.posFaults,
               (unsigned long)status.gapThresholdMs,
               (double)status.baselineDevPct,
               (double)status.steerScaleEstimate,
               (double)sideSlipDeg,
               (double)vScale,
               (unsigned)rAttEmaSq, (double)effectiveMeasurementR(0),
               // The two 2026-09-29 guards: a heading re-seed means the state was >120 deg
               // wrong against a trusted attitude, a veto means a reversal was refused
               // because the antenna did not confirm it.
               (double)status.psiAttDevDeg,
               (unsigned)status.headingReseeds, (unsigned)status.dirVetoes);
}

// ---------------------------------------------------------------------------
// Name tables - so that log lines and the web UI cannot drift apart from the
// enums.
// ---------------------------------------------------------------------------
const char* FLASHMEM HeadingBridge::stateName(State s) {
    switch (s) {
        case State::INIT:      return "INIT";
        case State::LIVE:      return "LIVE";
        case State::BRIDGE:    return "BRIDGE";
        case State::COAST:     return "COAST";
        case State::DEADRECKON: return "DEADRECKON";
        case State::TIMEOUT:   return "TIMEOUT";
    }
    return "?";
}

const char* FLASHMEM HeadingBridge::sourceName(Source s) {
    switch (s) {
        case Source::NONE:          return "none";
        case Source::ATTITUDE:      return "att";
        case Source::PATH_PROJECTED: return "proj";
        case Source::PATH_FIX2FIX:  return "fix2fix";
        case Source::PATH_COG:      return "cog";
        case Source::PATH_IMU_HEADING: return "imuhdg";

        case Source::DEADRECKON:    return "deadreck";
    }
    return "?";
}

const char* FLASHMEM HeadingBridge::reasonName(RejectReason r) {
    switch (r) {
        case RejectReason::NONE:            return "none";
        case RejectReason::GAP:             return "gap";
        case RejectReason::QF:              return "qf";
        case RejectReason::SENTENCE_ZEROED: return "zeroed";
        case RejectReason::ROLL_ENVELOPE:   return "rollEnv";
        case RejectReason::ROLL_STEP:       return "rollStep";
        case RejectReason::HEADING_STEP:    return "hdgStep";
        case RejectReason::COG_CROSSCHECK:  return "cogX";
        case RejectReason::BASELINE_LENGTH: return "baseLen";
        case RejectReason::SETTLING:        return "settle";
        case RejectReason::NOT_YET:         return "notYet";
    }
    return "?";
}
