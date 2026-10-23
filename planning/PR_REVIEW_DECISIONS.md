# Gemeinsame Pull-Request-Entscheidungen (23.10.2026)

Dieses Protokoll hält **Nutzerentscheidungen** fest. Die GitHub-PR-Nummern
sind fortlaufende IDs, nicht die Anzahl gleichzeitiger offener Änderungen.

| Ursprünglicher PR | Entscheidung | Technische Umsetzung / Bemerkung |
|---|---|---|
| [#1](https://github.com/Coffee-Tobi/robocasa/pull/1) | **Verwerfen** | Eigene experimentelle MPC-/Kontaktregelung nicht mergen. FAIRINO-SDK für Herstellerbewegung, Kollisionserkennung und optional kalibrierten echten 6-Achs-F/T-Sensor verwenden |
| [#2](https://github.com/Coffee-Tobi/robocasa/pull/2) | **Verwerfen** | Bereits durch gemergten #3 ersetzt |
| [#3](https://github.com/Coffee-Tobi/robocasa/pull/3) | **Aufräumen/Kamera behalten**, parallele Gripper durch 90° ersetzen | 90° F/T + IR40 visuell über [#19](https://github.com/Coffee-Tobi/robocasa/pull/19) nach main übernommen |
| [#4](https://github.com/Coffee-Tobi/robocasa/pull/4) | **Behalten und an den 90°-Adapter anpassen** | 20s kinematische Werkzeugmodelle über [#20](https://github.com/Coffee-Tobi/robocasa/pull/20) übernommen; noch keine echte Kontakt-/Kraftsimulation |
| [#5](https://github.com/Coffee-Tobi/robocasa/pull/5) | **Behalten als Simulationstest**, echte Bewegung über FAIRINO-Herstellerbibliothek | MuJoCo-Computed-Torque/PD nur als offline Vergleich; NICHT als eigener realer FR5-Laufzeitregler |
| [#6](https://github.com/Coffee-Tobi/robocasa/pull/6) | **VERWERFEN / aus aktiver main-Codebasis entfernen** | Alter FAIRINO-ähnlicher vs. Ruckig-Offlinevergleich darf keine Grundlage für echte FAIRINO-Steuerung sein; archivierte GitHub-Historie erhalten |
| [#7](https://github.com/Coffee-Tobi/robocasa/pull/7) | **BEHALTEN** | FAIRINO NewSpline und blend MoveL, ausschließlich trockene SDK-Schnittstellen-Tests bis echte Hardware/Controller-Validierung |
| [#8](https://github.com/Coffee-Tobi/robocasa/pull/8) | **BEHALTEN** | 3-Kamera-AprilTag → FAIRINO-Spline-Workflow weiterentwickeln, reale Kalibrierung noch nicht vorhanden |
| [#9](https://github.com/Coffee-Tobi/robocasa/pull/9) | **BEHALTEN UND ANPASSEN** | Experimentellen MuJoCo-NewSpline-Wegpunkt-Replay später auf genehmigte 90°-IR40-Seitenaufnahme aus gemergtem PR #21 rebasen; kein FAIRINO-Firmware-Nachweis |
| [#10](https://github.com/Coffee-Tobi/robocasa/pull/10) | **VERWERFEN, SPÄTER NEU ENTWICKELN** | Synthetische F/T-Grafik + alter 11-Stufen-Film entfernen, aber neue unabhängig freigegebene Kaffee-Choreografie und Hersteller-FT-Schnittstellen nicht blind löschen; künftig realen Inlux-FT im 90°-Modell sensorbasiert neu bauen |
| [#11](https://github.com/Coffee-Tobi/robocasa/pull/11) | **VERWERFEN** (geschlossen, nicht gemergt) | Bessere Lösung aus [#21](https://github.com/Coffee-Tobi/robocasa/pull/21) nach main gemergt: echte MuJoCo-Seitenanfahrt, horizontal schließende IR40-Backen, 90° Doppeladapter; alten parallelen Aufbau und den alten PR nicht übernehmen |

**Kein hardwareseitiger Sicherheitsnachweis:** Alte visuelle Griffe sind
starr angehängte Modelle, nicht wirklich greifende Finger. Auch neuer
Side-Approach erfordert noch vollständige doppelte Werkzeug-Kollisionen,
Griffkraft, Hersteller-STEP, Payload, F/T-Eichung und realen Prototyp.
