# Baseline-Referenz: Checkliste für die Testfahrt (2026-10-03)

Kurzfassung von `baseline_gate_findings_2026-10-03.md` für die Fahrt. Firmware:
`AiO_v26.5.1.hex` (neu gebaut, 03.10. 15:34).

## Vor der Fahrt

Flashen, dann im Boot-Log **eine** dieser Zeilen suchen:

| erwartet | Bedeutung |
|---|---|
| `HeadingBridge physicalBaselineM: not in the EEPROM, using the built-in default 0.711 m` | **Normalfall jetzt**: EEPROM leer, Build-Default greift |
| `HeadingBridge physicalBaselineM from EEPROM: 0.711 m - ... FIXED reference` | Betreiber-Wert im EEPROM (später über Web) |
| `HeadingBridge physicalBaselineM from EEPROM: 0 (switched OFF)` | Test bewusst aus, es wird wieder gelernt |

Und in der Zeile `HeadingBridge ENABLED: L=... ` muss am Ende **`base=0.711m`** stehen.
Steht dort `base=0.000m`, greift der Default nicht → sofort melden.

## Web-Oberfläche: Antennenabstand setzen und prüfen

Seite **`http://<ip>/vwas`** → Abschnitt **"Antennenabstand (Basislinie)"**.

* Feld leer und Anzeige "FESTE Referenz 0,711 m (Build-Wert)" → **der Fix ist aktiv**,
  es ist nichts einzutragen.
* **Eintragen + "Antennenabstand speichern"** überschreibt den Build-Wert und legt ihn ins
  EEPROM (übersteht einen Neustart). Gemessener Wert dieses Fahrzeug: **0,711**.
* **0 eintragen** = Test aus, die Referenz wird wieder vom Empfänger gelernt (nur als
  Notausgang). Ein leeres Feld wird bewusst abgewiesen, damit niemand versehentlich
  abschaltet.

Unter dem Feld steht live:

* welche Referenz in Benutzung ist und **woher** sie kommt (EEPROM / Build-Wert / gelernt),
* die vom Empfänger **gelöste Länge** und die **Abweichung in Prozent**,
* `hdgstddev` des Empfängers,
* ob das **Sicherheitsnetz** den Test abgeschaltet hat.

Erwartung nach dem Speichern von 0,711: *"Referenz und gelöste Länge passen zusammen"*,
Abweichung im einstelligen Prozentbereich. Vorher (alter Stand) stand dort **−16 %**.

Für Skripte direkt:

```
GET  /api/bridge/baseline
     {"meters":0.711,"set":false,"defaultM":0.711,"refM":0.711,"lenM":0.716,
      "devPct":0.7,"learnCount":0,"testDisabled":false,"hdgSdDeg":0.42}
POST /api/bridge/baseline   {"meters":0.711}      (0 = Test aus)
```

Im Live-Status derselben Seite steht zusätzlich `hdgstddev (Empfänger)` und bei den
Bridge-Diagnosen hinter `Referenz-Baseline` ein Vermerk `[FEST ... m aus dem EEPROM]`,
`[FEST ... m Build-Wert]` oder `[KEINE feste Referenz - gelernt]`.

## Die 4 Prüfpunkte im HR-Log

| # | Spalte | erwartet |
|---|---|---|
| 1 | `baseline_m` (25) | um **0,711 m** herum (p05 0,68 / p95 0,73 in gesunden Abschnitten) |
| 2 | `baseline_dev_pct` (26) | **nahe 0**, praktisch immer innerhalb ±5 % — vorher war es konstant −16 % |
| 3 | `rej` (27) | **kein `baseLen` mehr** in normalen Abschnitten |
| 4 | `uni_hdgsd` (90) | unverändert; dient nur der Einordnung (Jitter), nicht der Referenz |

Das Wackeln war die Folge von Punkt 3: bei `rej=baseLen` wird die Attitude verworfen und
die Bridge fällt auf COG/Tot-Rechnung zurück. Fällt `baseLen` weg, sollte das Wackeln weg sein.

## Abnahmekriterien

* Boot-Log zeigt den Default 0,711 m (oder den EEPROM-Wert).
* `baseline_dev_pct` liegt im Mittel bei ~0 % statt bei −16 %.
* Kein `baseLen` in ruhigen Fahrten. Unter dichten Bäumen darf es weiter auftauchen — das
  sind echte Multipath-Frames, ~10 % der RTK-Frames sind physikalisch außerhalb ±5 %.
* Die Heading-Sprünge in AgOpenGPS sind weg oder deutlich kleiner.

## Wenn etwas nicht stimmt

* Erscheint `HeadingBridge baseline test DISABLED: ... the REFERENCE (x.xxx m) is wrong`,
  passt der Wert nicht zu dieser Maschine (z. B. anderer Antennenabstand). Dann Log schicken —
  der Wert muss angepasst werden. Das Sicherheitsnetz verhindert in diesem Fall, dass das
  Heading ganz wegfällt.
* Bei einem **falschen** eingetragenen Wert dauert es 30 s bis das Netz greift.

## Nach der Fahrt

Das Wichtigste zuerst, ein Einzeiler (nimmt automatisch das **neueste** Log):

```powershell
cd <Teensy_AiO>
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\analysis\_devchk.ps1
```

Dort muss `ref samples ... p50` jetzt **0,711** zeigen. Vorher (alter Stand) stand da 0,852 —
das ist die Kernaussage der ganzen Übung. Ein bestimmtes Log:

```powershell
... .\tools\analysis\_devchk.ps1 -Filter '2026-10-03 1*'
```

Weiter, wenn die Referenz stimmt:

```powershell
# Länge, Gate-Kontamination, Trennschärfe gegen Rauschen und gegen FALSCH
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\analysis\lenqc.ps1 -Filter '2026-10-03 1*'
```
