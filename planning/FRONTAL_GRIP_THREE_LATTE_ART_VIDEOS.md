# Frontaler IR40-Griff, seitliche Kamera, drei Latte-Art-Videos

## 1. Werkzeugrahmen & Videokamera

`dual_ir40_concept.py`: Beide konzeptionellen IR40-50-Einheiten sind in der
virtuellen Geometrie um Y gedreht: Greifbacken zeigen von der Roboterhand
nach vorne entlang der angenäherten Werkzeugachse statt nach oben. 
Kindkörper für Kanne, Siebträger und Kunststoffbecher erhalten eine Gegenrotation,
damit z. B. die Kanne am aufrecht orientierten TCP senkrecht bleibt. Das sind
**keine vendorverifizierten CAD-Daten und keine realen Greifkräfte**.

`follow_camera.py`: Kamera bewegt sich gefiltert mit einer deutlich größeren
Seitenversetzung. Sie blickt auf einen gewichteten Mittelpunkt aus TCP und
sichtbarer Kanne/Becher/Siebträger. Die drei ortsfesten kalibrierten
AprilTag-Kameras für die Messung bleiben unverändert.

`coffee_workflow.py`: Bei Transporten mit Kanne oder Becher erfolgt nach
Möglichkeit eine aufrechte Orientierungskorrektur über sechsachsige IK;
ein erfolgloser Versuch wird in den Kennzahlen berichtet, nicht als
erfolgreiche Flüssigkeitssicherheit ausgegeben. Bei bewusster Latte-Art-
Neigung wird diese Korrektur deaktiviert. Die geplanten sieben Vorpositionen
bleiben weiterhin kontaktlos und symbolisch.

`full_barista_sensor_video.py`: Wiedergabe auf ca. 69 Sekunden statt 34,5
verlangsamt. Das ist **Retiming des Films**, kein Nachweis langsamer Motoren,
geänderter Gelenkbeschleunigung oder echter Getränke.

## 2. Drei Videos, dieselbe virtuelle Bewegung

Das Eingangsdatum ist eine **virtuelle** ~20-Sekunden-Kannengeste, nicht schon
eine von einem echten Menschen aufgenommene Latte-Art-Demonstration.
Die Simulation verwendet die bestehende Drei-RGB-D-Kamera-
AprilTag-Rekonstruktion ohne Zugriff auf die Simulationswahrheit.

1. `original_20s.mp4`: bewegte Referenz-Milchkanne allein
2. `tracking_20s.mp4`: aus den drei gerenderten RGB-D-Kameras
   rekonstruierte Milchkannenposition, mit Auszug aus tatsächlich gerenderten
   Kamera-/Tagbildern; kein synthetisches Umbenennen einer Ground-Truth-Bahn
3. `robot_20s.mp4`: der echte **virtuelle** sechsgelenkige FR5 in MuJoCo
   folgt mit begrenzter 6D-IK den gemessenen Kameraposen; überprüfte
   collision/keepout-Segmente; Kanne im Modell starr am Griffadapter.
   **Dies ist kein Nachweis der internen FAIRINO-NewSpline-Firmware und
   auch kein `mj_step`-Drehmoment/Motorfolgertest.**

Alle drei Filme verwenden dieselbe seitlich mitbewegte, virtuelle Videokamera.
Das Tracking verwendet wiederum **drei andere, statische RGB-D-Kameras**.

## 3. Zwei identische Fehlertypen

Das Kannen-Spout-Tip ist ein gemeinsamer festgelegter virtueller
`SPOUT_TIP_LOCAL_M`-Punkt aus dem 3D-Modell. Für Tracking und Roboter
wird jeder Punkt mit der generierten Originaltrajektorie verglichen:

- **ΔX(t), ΔY(t), ΔZ(t)** in mm (Weltkoordinaten)
- **e(t) = sqrt(ΔX²+ΔY²+ΔZ²)** in mm – eine einzige skalare Zahl
- mittlerer Fehler, Maximalfehler, P95 und RMSE
- zusätzlicher Fehler FR5 gegenüber gemessener Kamera
- Winkelfehler in Grad (diagnostisch)

Alle Werte sind in **denselben Zeitkoordinaten** verglichen. Die
Trackingbeobachtungen stammen aus real gerenderten synthetischen RGB-D-Bildern,
auf Video-FPS interpoliert; dies ist keine 8-Hz-Kameramessrate.
Die virtualisierte "Originalbewegung" wird für die numerische Bewertung
zurückgehalten, nicht dem IK-Replay gefüttert.

## Was es noch nicht ist

- Die echte menschliche Kannenaufzeichnung mit einer physischen
  AprilTag-Kameraeinrichtung fehlt.
- Keine physisch verifizierte Greifkraft, Milch- oder Crema-Dynamik.
- Keine gemessene Nachfahrgenauigkeit der FAIRINO-Firmware, nur die
  getrennte idealisierte MuJoCo-IK-Ausführung. Die neue Schnittstelle
  `NewSpline` aus unserem Standard-Colab ist ein Offline-Trockenlauf.

## Reproduzierbarkeit

```bash
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
python -m robobarista.camera_joint_replay \
  --capture-only --latte-art-source --seed 14 --frames 61 --fps 3 \
  --width 960 --height 720 --output runs/latte_capture

python -m robobarista.latte_art_three_videos \
  --capture runs/latte_capture --output runs/latte_three --fps 8

python -m robobarista.full_barista_sensor_video \
  --output runs/slow_coffee --width 640 --height 480 \
  --capture-fps 6 --repeat 2 --playback-fps 6 --frames-leg 9
```

Der GitHub Actions-Workflow `.github/workflows/latte-three-videos.yml`
prüft das Vorhandensein der drei MP4s, deren Dauer und die Einhaltung
der Rechendefinition des euklidischen Fehlers.
