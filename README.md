# RoboBarista — 3-Kamera-Aufnahme → FAIRINO FR5

## AKTUELL – EIN RoboBarista-Entwicklungsstand

> **Aktiver Draft-Pull-Request:** [#18 – RoboBarista Integration](https://github.com/Coffee-Tobi/robocasa/pull/18)
> auf Basis von `main`. **Nicht gemergt und nicht für reale Hardware freigegeben.**
> Alte #11, #13, #14, #16 und #17 sind als Entwürfe archiviert, deren Branches
> bleiben erhalten. Neben #18 sind nur die getrennten Forschungs-PRs
> [#9 – FAIRINO NewSpline](https://github.com/Coffee-Tobi/robocasa/pull/9)
> und [#1 – MPC-Kontaktsensorik](https://github.com/Coffee-Tobi/robocasa/pull/1) offen.

**Bitte zuerst lesen:** [Projektstatus, Aufräumprotokoll und offene Aufgaben](planning/PROJECT_STATUS_AND_PR_CLEANUP_2026_10_23.md).

### Aktuelle visuelle Kaffee-Vorführung

- **Keramiktasse am Henkel:** ein expliziter `handle_mug`-Modus mit schmalen
  IR40-B-Backen am Henkel, Tassenkörper NICHT unter/zwischen Backen.
  **Einwegbecher** besitzt separat `--cup-style takeaway` für Körpergrip.
- **Kleine AprilTags direkt befestigt**: `--marker-profile compact` mit
  24–42 mm großen Tags auf Tasse, Milchkanne, Siebträger und Stationen.
  Die alten großen Tags bleiben ausschließlich Benchmarkoption `legacy`.
- **Griff von schräg vorne sichtbar:** `--camera-view handle_close`
  ist der aktuelle Kino-Kamerawinkel; 3/4 RGB-D-Messkameras sind davon
  unabhängig.
- **Keine physische Greifbehauptung:** Die Tasse bleibt ein starr
  angehängtes MuJoCo-Videomodell, keine reale Finger-Kontaktsimulation.

```bash
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
python -m pytest -q tests/test_handle_mug_compact_video.py tests/test_compact_apriltags.py
python -m robobarista.coffee_workflow \
    --marker-profile compact --cup-style handle_mug \
    --camera-view handle_close --frames-leg 3 \
    --output runs/handled_mug_compact
# Video: runs/handled_mug_compact/coffee_workflow.mp4
```

### Reale Gießdemonstrationen und Robotik

- [40 veröffentlichte LeRobot-Realman-Demonstrationen importieren](robobarista/real_latte_dataset.py):
  Die echte erste Episode lieferte 400 gültige Messpunkte bei 20 Hz,
  14D Gelenk+Greifer; keine gegebene 3D-Kannenbahn.
- [Originales RM65-URDF-FK-Experiment](robobarista/realman_fk_bridge.py):
  RealMan-Robotergeometrie und Flanschversatz müssen zu den Originaldaten
  passen; bisher wurde nur eine **nicht kalibrierte RM65-B-Annahme** getestet.
- [FR5 3D-Kamera-Messung und kinematisches Nachfahren](robobarista/camera_joint_replay.py):
  13/13 gültige virtuelle Kameraposen im früheren Test mit großen Tags,
  max. **0,561 mm virtueller Kannenpositionsfehler**;
  keine reale Ausgießerspitzen-/Servo-Präzisionsgarantie.
- [DFKI-DMP](robobarista/latte_dmp.py), [MoveIt2-Planungsszene](robobarista/moveit_scene_bridge.py)
  und [FAIRINO SimMachine-Prüfung](robobarista/vendor_runtime_preflight.py)
  sind getrennte **Offline-Schnittstellen**, nicht eine erprobte Herstellersteuerung.
- [MIT-2D-Latte-Flüssigkeitssimulator](robobarista/latte_fluid_adapter.py)
  wird als bestehender Open-Source-Sidecar verwendet. [Einfacher Python
  Milch-Oberflächenproxy](robobarista/latte_fluid_proxy.py) ist nur eine
  CPU-Fallback-Animation. **FluidLab `LatteArt-v0` ist noch nicht eingebaut**;
  weder 2D-Variante simuliert physikalisch echten Milchschaum.
- [Google-Colab-Notebook mit optionalen Simulationen](COLAB_ROBOBARISTA.ipynb).
  [Realman-Demo→FR5](planning/REAL_LATTE_DEMOS_TO_FR5_CAMERAS.md)
  und [Video-/Milchversuch](planning/REAL_LATTE_VIDEO_AND_FLUID_INTEGRATION.md)
  dokumentieren genaue Unterschiede.

**Wichtig:** Espresso-Bezug, Siebträgerverriegelung, Greifkräfte, Dampfkontakt,
Milchschaum und Latte-Art-Muster werden derzeit nur symbolisch/idealisiert
visualisiert. Der nächste Schritt muss tatsächliche Werkzeugkontakte,
kalibrierte Spitzengenauigkeit und echte Motorreaktion prüfen.

## Open-Source-Zubehör und FAIRINO-Check (Oktober 2026)

**Neu: tatsächliche, getestete Wiederverwendung statt bloßer eigener Geräteattrappen.**

- [RoboCasa-Kaffeemaschinenlogik (MIT) für unsere Espresso-Station](robobarista/robocasa_coffee_adapter.py): Prüft die reale Position des Kunststoffbechers zum Auslauf, bevor symbolischer Espresso angezeigt wird. Echte Verriegelung, Kontaktsensor und Brühdruck sind weiterhin **nicht simuliert**. Die neue Geometrie enthält benannte Auslauf- und Startknopf-Positionen.
- [FAIRINO-SDK: auslesbare Sensorik und acht IK-Lösungen](robobarista/fairino_sdk_preflight.py): `GetActualTCPPose`, Gelenkposition/-tempo, F/T, Not-Aus, Warteschlange und `TCFToAllJoint` sind per sicheren Offline-Mocks vorbereitet, ohne einen Roboter zu steuern.
- [LeRobot-inspirierter Export synthetischer Latte-Art-Kameraaufnahmen](robobarista/latte_dataset_bridge.py): Kameradaten und Simulationswahrheit für spätere echte Demonstrationen getrennt speichern. Noch kein natives LeRobot-Dataset.
- [CAD-Dateien vor Import prüfen](robobarista/external_asset_validator.py): Die recherchierte MIT-Espressomaschine enthielt ein **leeres STEP** und **ein STL mit nur einem Dreieck**. Keine Änderung an unserer physischen Kollisionsszene durch solche Dateien.
- [Vollständiger Open-Source-, SDK- und Prototypen-Reifebericht](planning/OSS_BARISTA_FR5_READINESS_AUDIT.md).

**Status:** Unser Projekt eignet sich als Grundlage für einen kontrollierten
ersten **Trockenprototypen**, nicht für einen unbeaufsichtigten echten
Milch-/Espresso-Barista. Es fehlen kalibrierte Greifer, Lastmessung,
Düsenkontakt, Portafilter-Einrasten, echte Hersteller-Regelung, reale
Messkameras und Flüssigkeits-/Temperatursicherheit.


## NEU: Gesamter Kaffeeprozess mit beiden IR40-50 und F/T-Prototypen

Der [34,5-sekündige komplette RoboBarista-Videotest](https://github.com/Coffee-Tobi/robocasa/actions/workflows/full-barista-contacts.yml)
zeigt elf Schritte: Siebträger, Mühle mit Tamper, Brühgruppe, Kunststoffbecher,
Espresso, Milchkanne, Dosieren, Dampflanze, Becherplatzierung, Latte Art, Rinser.

- [Filmcode](robobarista/full_barista_sensor_video.py)
- [FAIRINO F/T / LoadTrajectoryLA Anbindung](robobarista/fairino_force_and_trajectory.py)
- [Sensor- und Sicherheitsstatus](planning/COMPLETE_COFFEE_PROCESS_FORCE_SENSORS.md)

**WICHTIG:** Alle Kontakt-Kurven sind synthetische Beispielwerte; keine echte
Siebträger-Verriegelung oder Berührung der Dampfdüse wird durchgeführt.
Das Video visualisiert bislang kollisionsgeprüfte Vorpositionen.
Der direkte Standard für aus drei Kameras erzeugte 6D-Bewegungen bleibt
**FAIRINO `NewSpline`**; `LoadTrajectoryLA` erfordert eine eigene,
vom Hersteller unterstützte *Gelenktrajektoriedatei*, keine rohe
AprilTag-Positionsliste.


**Ein einziger aktiver Bewegungsansatz:** drei fest kalibrierte RGB-D-Kameras
erkennen AprilTags auf der Milchkanne; daraus wird eine 6D-Bewegung rekonstruiert,
in FR5-Werkzeugkoordinaten umgerechnet und als
**offizielle FAIRINO `NewSpline`-Befehle** bereitgestellt.

- **[Google Colab: komplette neue Pipeline](COLAB_ROBOBARISTA.ipynb)**
- [Drei-Kamera-Erfassung (nur Bildmessung)](robobarista/camera_joint_replay.py), Kommando `--capture-only`
- [Aktiver Ablauf: Kamera → FAIRINO](robobarista/fairino_barista_pipeline.py)
- [Offizielle FAIRINO-NewSpline/MoveL-Befehlsanbindung](robobarista/fairino_native_spline.py)
- [Hersteller-Motion-API](https://manual.fairino.support/latest/SDKManual/PythonRobotMovement.html)
- [Letzter getesteter offline Kamera/Spline-Workflow](https://github.com/Coffee-Tobi/robocasa/actions/workflows/fairino-camera-pipeline.yml)

## Wie die Kette genau funktioniert

1. **Aufnehmen (Kameras):** Drei simulierte RGB-D-Kameras beobachten AprilTags auf der Kanne. Das System rekonstruiert Position **und Orientierung** je Zeitstempel. Falls Tags in weniger als zwei Kameras sichtbar sind, wird die Aufnahme nicht freigegeben.
2. **Kalibrierung:** Die Position der Kanne ist nicht automatisch die Werkzeugpose. Es gilt `T_base_tcp = T_base_world @ T_world_pitcher @ T_pitcher_tcp`. Beide extrinsischen Transformationen müssen vor realem Einsatz gemessen und überprüft werden.
3. **FAIRINO-Vorbereitung:** Positions-/Orientierungswegpunkte in mm/Grad umwandeln und mit `NewSplineStart / NewSplinePoint / NewSplineEnd` eine zusammenhängende Bewegung anfordern (wahlweise `MoveL` mit `blendR`).
4. **Ausführen:** Erst an einem kompatiblen FAIRINO-Controller mit realen Sicherheitsprüfungen, korrekten Greifern und Werkzeugdaten. **Wir haben noch keinen realen FAIRINO-Controller verbunden.**
5. **Spätere Live-Korrektur:** Dieselben Kameras beobachten Kanne und Becher während der Bewegung. Die nötige Closed-Loop-Korrektur über FAIRINO `ServoCart` ist geplant, **noch nicht implementiert**.

## Was schon getestet wurde

| Merkmal | Status |
|---|---|
| Virtuelle FR5-Kaffeestation und drei Kameras/AprilTags | Kamera-/Szenentests bestanden |
| Rekonstruktion einer demonstrierten 6D-Bewegung | 31 simulierte Messbilder, 15 FPS, Aufnahme dauert 2 s |
| 20-s-Referenz mit max. 18 mm Kannenmittelpunkt-Auslenkung | Durch bewusstes Skalieren und zeitliches Strecken der aufgenommenen Bahn |
| FAIRINO-NewSpline-Befehlsgenerierung | Offline-Mock: offizielle Methodennamen und Parameter getestet |
| Herstellerinterne Spline-Interpolationsbahn | **NICHT getestet** (keine echte Controller- oder SimMachine-Ausführung) |
| Echte FAIRINO-Nachfahrgenauigkeit | **NICHT gemessen** |
| Tatsächlicher Abstand der Ausgießerspitze zur Crema | Noch nicht kalibriert; 18-mm-Grenze betrifft die Kannenmitte; die Ausgießerspitze wird nur mit einem explizit angegebenen virtuellen Offset mitgeprüft |
| Simulierte Milchdynamik, Kräfte an Greifbacken und Latte-Art-Muster | Noch nicht physikalisch validiert |

## Neue Pipeline reproduzieren

```bash
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa

# Virtuelle Kameraaufnahme aus drei Bildern, ohne alten Roboter-Nachfahrversuch:
python -m robobarista.camera_joint_replay \
  --capture-only --frames 31 --fps 15 --width 960 --height 720 \
  --output runs/fairino_camera_capture

# Standardausgabe: OFFLINE vorbereitete offizielle FAIRINO-Spline-Befehle:
python -m robobarista.fairino_barista_pipeline \
  --input runs/fairino_camera_capture/camera_measurements.json \
  --output runs/fairino_standard \
  --target-mm 18 --duration-s 20 --stride 2 \
  --virtual-spout-offset-m -.073 0 .117
```

Erzeugt `runs/fairino_standard/fairino_new_spline_commands.json`,
`fairino_movel_blend_commands.json`, `summary.json` und die
zeitlich angepassten Kamera-Wegpunkte. **Diese Programme verbinden sich aus
Sicherheitsgründen nicht mit einem Roboter und rufen keinen Controller auf.**
Für reale Roboterbefehle fehlen gemessene Kamera-/Werkzeugextrinsiken,
Werkzeugkalibrierung, Kollisionsplanung und Hardwarefreigabe.

## Was mit dem alten Ansatz passiert

Die eigene Bewegungsglättung mit Ruckig, IK-Nachfahrtests und
`mj_step`-Drehmomentregler sind **nicht mehr Teil des Standardablaufs**
und auch **nicht mehr die empfohlene Roboteransteuerung**. Der frühere Code
liegt zur wissenschaftlichen Nachvollziehbarkeit und zum Vergleich weiterhin
im Git-Verlauf beziehungsweise in experimentellen Modulen; die bisherigen
Referenzzahlen gehören nicht zur echten FAIRINO-Herstellerregelung.

Wichtig: Der eigene Kameracode, AprilTag-Kalibrierung, Szenenmodell,
Kollisionsabschätzung sowie die Logik für Latte-Art- und Milchflusskorrekturen
bleiben nützlich; eine reine FAIRINO-SDK ersetzt diese Fähigkeiten nicht.
