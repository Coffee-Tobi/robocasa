# RoboBarista — 3-Kamera-Aufnahme → FAIRINO FR5

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
| Tatsächlicher Abstand der Ausgießerspitze zur Crema | Noch nicht kalibriert; 18-mm-Grenze betrifft bisher den Kannenmittelpunkt |
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
  --target-mm 18 --duration-s 20 --stride 2
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
