# 01 Ziele und Messlatte (Position / Heading / Roll / VWAS)

> **Summary (EN).** What this firmware has to deliver, and in which units it is measured.
> The box feeds AgOpenGPS (AOG) with **position** and **travel direction**; AOG steers.
> There is no IMU as a heading source in the core path: the dual antenna (UM982) is the
> absolute direction, the position course is its independent witness, the wheel angle is
> the short-term model. Acceptance is the operator's field report translated into numbers
> (table below). Note that all limits are given **per second and in degrees**, never per
> frame, because the logs arrive at 5/10/20 Hz. The goals are split per path at the end
> (position, heading, roll, virtual wheel angle), each pointing at its topic document.
> Status 2026-11-03: content unchanged, split out of `archive/GRUNDZIELE_2026-10-11.md`.

Diese Datei sagt, **was** gebaut wird und **woran** gemessen wird. Sie ist die Messlatte für
`10_heading.md`, `20_position.md`, `50_roll.md` und `40_vwas.md`. Ergänzungen nur mit Datum
und Log-Bezug. Ablauf und Arbeitsregeln stehen in `WORKFLOW.md`, der Phasenplan und die
offenen Punkte in `80_plan.md`, der Datei- und Werkzeugindex in `00_ueberblick.md`.

## 1. Akzeptanzkriterien (Betreiber-Sprache → Messgröße → Zielwert)

| Betreiber sagt | Messgröße | Zielwert |
|---|---|---|
| "fährt schräg" | sichtbare Schräglage = `travel - (az_out+88)` = psi-Fehler | p90 < 2 Grad auf Geraden |
| "Heading verwackelt" (Gerade) | `p99 \|dHeading\| / dt` | **<= 1 Grad/s** – jede GPS-Rate, auch bei schlechtem Empfang |
| "Heading verwackelt" (Treue) | `p95 \|Heading − Positionskurs (2-s-Fit)\|` | **<= 0,5 Grad** (gute Datenlage) |
| "Kurve wird nicht gefahren" | im Heading verlorene Grad je Kurve | < 5 Grad bei 90-Grad-Kurve |
| "Positionssprung" | Schritt der publizierten Position gegen `v*dt` | kein Schritt > 0,15 m im Fahren |
| "Roll springt" | Sprung in `roll_bridge_deg` | < 1 Grad je Frame |
| "Heading verloren/falsch" | `\|az_out+88−travel\|` | < 5 Grad, Dauer < 1 s |

Arbeitsbereich: **0,2…20 km/h** (Arbeitsfall 5…10 km/h), Daten können bis **60 km/h** kommen,
enge 90-Grad-Kurven, Rückwärts/K-Turn, Stand und Schleichfahrt. Diese Bereiche sind die
Prüffälle, nicht die Ausnahme.

### 1.0 Ziel in physikalischen Einheiten (2026-10-12, GPS-Rate-unabhängig)

Alle Ziele gelten in **Grad pro Sekunde bzw. Grad**, NICHT pro Frame: die Logs kommen mit
**5, 10 oder 20 Hz**, ein "je Frame"-Kriterium verschiebt sich dadurch um Faktor 2 oder 4.
(Die alte Zeile "sd je 0,1 s (10 Hz)" war genau das – ersetzt 10-12.)

**Warum diese Zahlen physikalisch sind**
* **0,5 Grad** ist das Eigenrauschen der Dual-Antenne (0,71 m Basis, `uni_hdgsd` gemessen
  0,4…0,6 Grad) – das Ziel heißt: *nicht schlechter als die Hardware*.
* **1 Grad/s** ist die reale Drehrate eines GERADE fahrenden Traktors bei 2…4 m/s mit
  ~0,5 Grad Lenkwinkel (`57,3*v*tan(delta)/L` = 0,4…0,8 Grad/s) – "gerade" heißt
  physikalisch fast keine Drehung.

**Kurven-Regel** (Betreiber 10-12: "wie gehen wir in Kurven damit um"): die Grenze ist KEINE
feste 1 Grad/s, sondern
`zulässig = max(1 Grad/s, k * omega_phys)`, `omega_phys = 57,3*v*tan(|delta_eff|+slack)/L`
mit dem **trim-korrigierten** Lenkwinkel (B7.21/B10.24). Auf der Geraden greift der Boden
1 Grad/s, in der Kurve öffnet die Physik (bei 15 Grad Lenkwinkel und 4 m/s: ~25 Grad/s).
**Keine Bestätigungs-Verzögerung in Kurven** – dort ist die Änderung durch die Physik
gedeckt und darf sofort durch; eine Verzögerung macht aus der Kurve ein Nachlauf-Problem.

**Schlechter Empfang** (Betreiber 10-12: "das Ziel muss auch bei schlechtem Empfang gelten,
sonst schaukelt sich der Traktor auf"): das BAND (0,5 Grad) ist dort NICHT haltbar – die
Quelle selbst ist mehrere Grad falsch. Deshalb ist die **Rate die HARTE** und das **Band die
QUALITÄTS**-Forderung:
* **HART (immer, auch bei Ausfall)**: die Ausgabe springt nie – `p99 |dHeading|/dt` <= 1 Grad/s
  (Gerade) bzw. <= der Kurven-Grenze. Begründung aus der Messung 10-12: bei `att_q = 0`
  Sprünge bis 60 Grad, dazu AOG-Zielwinkel sd 6,2 Grad – das ist das Aufschaukeln.
* **QUALITÄT (gute Datenlage)**: `p95 |Heading − Positionskurs (2-s-Fit)|` <= 0,5 Grad.
* **In schlechten Phasen**: nicht dem Unsinn folgen, sondern **nahe am letzten vertrauten Kurs
  bleiben** (driftet höchstens mit der Rate), und beim Wiedereintritt **als Rampe** nachführen
  (Rate <= dieselbe Grenze, nie ein Sprung).

**Wahrheit für die Kennzahl**: der **Positionskurs als 2-s-Fit** über `lat/lon` (Prinzip
siehe `20_position.md`). Die Einzelframe-Ableitung `travel` taugt NICHT als Wahrheit – sie ist
selbst zu verrauscht (gemessen 10-12: p90 14…21 Grad/s).

**Umsetzungs-Konsequenz (GPS-Rate)**: alle Schwellen künftig in **Grad/s und ms** speichern
und mit `dt` umrechnen. Heute noch FRAME-basiert und damit rate-abhängig:
`Config::outSnapDeg` (3 Grad – bei 5 Hz sind das 15 Grad/s, bei 20 Hz 60 Grad/s!),
`Config::outSnapConfirmFrames` (1 Frame = 100 ms bei 10 Hz, 200 ms bei 5 Hz),
`Config::outNoiseSamples` (5 Frames = 0,5 s bei 10 Hz). Schon zeitbasiert: `outSmoothMs`,
`outHoldMaxMs`, `liveRampDegPerS`, `outResidRateFactor`.

### 1.1 Betreiber-Festlegungen (2026-10-11) – verbindliche Messlatte

**Vorrang bei Zielkonflikt** (wörtlich): *"treu, wenn die Datenlage sicher ist = guter Empfang,
Empfänger eingeschwungen, sonst stetige Werte, keine Sprünge bei Roll, Heading und Position
(wobei die Position bei schlechtem Empfang meist noch gut ist), möglichst verzögerungsfrei vom
GPS zu AOG. Bei Ausfall eine robuste Überbrückung, bis GPS wieder da ist."*

Also kurz: **gute Datenlage → treu UND schnell. Schlechte Datenlage → stetig, halten, nie
springen.** Treue ist ein Vorrecht guter Daten, nicht eine Filtereigenschaft.

**Weitere Festlegungen:**

* **Tempo**: Arbeitsfall **5…10 km/h**, Band 0,2…20 km/h, Daten bis 60 km/h.
* **IMU**: bei **Single** (F9P + IMU) fast immer vorhanden; bei **Dual** (2x F9P oder UM982)
  selten, aber möglich. Der Kernpfad muss **ohne** IMU funktionieren und **mit** IMU besser werden.
* **Lenkwinkel**: echter Sensor (analog), Traktor-CAN, oder **virtuell aus dem Keya-Lenkradmotor
  (eingebauter Encoder)**. Der Keya-Encoder startet bei **jedem Neustart bei 0**, egal wie die
  Räder stehen → **die Null muss gefahren/gefunden werden**.
* **Ausgabe-Ideal**: *"sauber lenken, aber nahe an den GPS-Daten bleiben, wenn diese gut sind.
  Sonst kommt es zu Schwingungen. Im Idealfall GPS live weitergeben."* Für das Heading ist ein
  **gleitender Durchschnitt** gut, weil die UM982 Heading-Rauschen mitbringt.
* Umbau: Patches **und** größere Schritte sind erlaubt; Plan gemeinsam besprechen.
* "Fertig" definiert sich über den Plan; danach fährt der Betreiber Probe und entscheidet.

## 2. Wahrheiten (die Messlatte)

1. **Positionskurs = absolute Wahrheit.** 0,5…5 m Sehne aus `lat/lon`
   (krümmungskorrigiert um den Chord-Nachlauf `0,5*w*T`), unabhängig von Attitude, IMU,
   Lenkwinkel und Modell. Gemessen 10-10: 0,10…1,11 Grad gegen Modell 13,9…29 Grad
   über 11…29 s Ausfallücken.
2. **Attitude (`att_hdg_deg`, Dual) = Karosserie-Zeuge.** Wird nie gespiegelt und nie von
   uns erfunden; ihr Zuwachs beweist eine Drehung. Der COG ist der zweite, unabhängige Zeuge.
3. **`psi` ist ZUSTAND, keine Messung.** Folgt psi dem Modell und zieht die Attitude nicht,
   ist psi falsch – direkt messbar als `psi_att` (muss konstant bleiben).
4. **Die eigene publizierte (Projektions-)Position ist NIE Kursquelle** (5,22 statt 0,10 Grad).
5. **Konvention**: `az_out` verlässt das Modul, AOG addiert den Montage-Offset (+88) selbst
   → die in AOG sichtbare Richtung ist `az_out+88`. `psi`, `travel`, `cog`, `att`
   sind AOG-Frame-Größen.

## 3. Ziele je Pfad

Die Tabelle in §1 nennt alle Zahlen; hier stehen sie nach Pfad sortiert, damit klar ist,
welche Datei welches Ziel verantwortet.

### 3.1 Position (P) → `20_position.md`

* **Kein Positionsschritt > 0,15 m im Fahren** (jeder Schritt wird gegen `v*dt` geprüft).
* Die publizierte Position ist nie Kursquelle (Wahrheit 4) – die Projektion ist Notnagel,
  auf <= 0,5 m begrenzt.
* Positionen sind die **unabhängige Wahrheit** für alle anderen Pfade (Ring, Kurs, WAS-Nullpunkt).

### 3.2 Heading (H) → `10_heading.md`

* Rate (hart): `p99 |dHeading|/dt <= 1 Grad/s` auf Geraden, `<= max(1, k*omega_phys)` in Kurven.
* Treue (Qualität): `p95 |Heading − Positionskurs (2-s-Fit)| <= 0,5 Grad`.
* Kurven: < 5 Grad verloren bei einer 90-Grad-Kurve.
* Ausfall: < 5 Grad Fehler, Dauer < 1 s; Wiedereintritt als Rampe, kein Sprung.

### 3.3 Roll (R) → `50_roll.md`

* **Kein Sprung > 1 Grad je Frame** in `roll_bridge_deg`; `roll_susp` bleibt als Zustand.
* Roll wird getrennt gegatet – er darf Heading und Position nicht mitreißen.

### 3.4 Virtueller Lenkwinkel (VWAS) → `40_vwas.md`

* Lenkwinkel **p50 < 1 Grad auf Geraden** (Soll 0) – daraus folgt der Heading-Drift, siehe
  offener Punkt "Lenkwinkel-Versatz 5,6 Grad" in `80_plan.md`.
* `L_implied` über die Winkelbins **konstant**.
* **Nullpunkt nur aus Rohpositionen** (Krümmung gegen Lenkwinkel), nie aus Heading oder
  `az_out`, und **nicht im Stand** gelernt (dort gibt es keine Information).

