# Kleine realistische AprilTags + LeRobot für Latte Art

**Stand: 23.10.2026.** Experiment separat auf
`feature/compact-realistic-apriltag-fixtures` und PR #13.
**Bestehende großen Tags NICHT überschrieben**; Vergleich explizit mit
`--marker-profile legacy`.

## 1. Kaffeestation und Latte-Art-Bibliothek

- **RoboCasa**: Die frei lizenzierte Kaffeemaschinen-Fixure-Funktion
  (Becher unter Auslauf, symbolischer Maschinenzustand) wurde in unserem
  letzten Arbeitsschritt unter `robobarista/robocasa_coffee_adapter.py`
  auf `main` übernommen. RoboCasa-Inventar hat zahlreiche
  elektrische Filter-/Kapselmaschinen, aber keinen nachgewiesenen
  mechanischen Siebträger-Gruppenkopf mit Bayonett und Dampflanze für
  unser Setup; deshalb verwenden wir weiterhin die eigene Geometrie
  zum Planen und die fremde Fixtures-Logik für den Zustand.
- [ridxm/latte-art-robot](https://github.com/ridxm/latte-art-robot):
  reale 3-Kamera-LeRobot-Demonstrationen von einem DUAL Realman-Roboter
  mit etwa 40 Gießversuchen, ~20s, 20Hz. **Keine explizite Lizenzdatei**
  gefunden; fremden Projektcode weder kopieren noch als FAIRINO-Steuerung
  ausgeben.
- **Jetzt direkt eingebunden**:
  `robobarista/lerobot_native_latte.py` nutzt bei optionaler
  Installation die tatsächliche
  [Hugging Face LeRobotDataset v3 API](https://huggingface.co/docs/lerobot/main/api/datasets)
  (`LeRobotDataset.create`, `add_frame`, `save_episode`, `finalize`).
  Aufnahmen über `--save-camera-frames` speichern **unbearbeitete drei
  RGB-Bilder** pro Zeitpunkt. Es entsteht ein lokaler Datensatz,
  ohne Upload, nur mit Kamera-Pose (7D) und *künftiger gemessener Pose
  als synthetischer Ersatzhandlung*. Es ist **kein** echter FAIRINO-
  Gelenk-Datensatz und **kein** menschlicher Latte-Art-Versuch.
- Für reales Trainieren müssen wir später zu jedem Bild die echten
  sechs Gelenkwinkel, Greiferstatus, Werkzeug-TCP, physische Kraft,
  gefüllte Kanne, Lage der Tasse und Zeitstempel synchronisieren.

## 2. Kleine, tatsächlich befestigte Tags

Ausgedruckte Außenkanten **inklusive weißer Ruhezone**. Tatsächliche
schwarz-weiße Bildbreite im 512px-Marker beträgt **440/512 = 0,8594**
der angegebenen Außenkante.

| Objekt | kompakte Außenkante | aktive Kante im PnP | echte geometrische Halterung |
|---|---:|---:|---|
| freie Kanne, seitlich | 36mm | 30,94mm | Flacher Seitenhalter y=-67mm am 48mm-Radius-Körper; 20mm Clipsteg |
| freie Kanne, Griff oben | 32mm | 27,50mm | Starrer 48mm Griffsteg, x=+83mm |
| freie Kanne, zweite Orientierung | 30mm | 25,78mm | kleiner 35°-Winkelhalter an Griffaußenseite |
| freier Plastikbecher | 28mm | 24,06mm | 15mm externer Clip am Rand; nicht auf gekrümmte Wand gepresst |
| Siebträgergriff abgelegt | 24mm | 20,63mm | flache Platte auf dem äußeren Griff |
| **getragene** Kanne | 36mm | 30,94mm | eigener 14mm Clip am kinematischen Greifermodell |
| **getragener** Becher | 28mm | 24,06mm | eigener 14mm Clip am Becherrand im Greifermodell |
| **getragener** Siebträger | 24mm | 20,63mm | eigene Platte auf gegriffenem Siebträgergriff |
| Mühle, Maschine und Milchstation | 42mm | 36,09mm | auf statischen flachen Geräteflächen |
| Restliche stationäre Referenzen | 36mm | 30,94mm | am Tisch-/Standkörper, keine hohen Stäbe |

Die starren Greifobjekte im Full-Coffee-Film sind Modellkopien der realen
Objekte: eindeutige AprilTag-IDs 74/75/76 vermeiden Doppel-ID-Decoding.
Die freie Kanne ID 23/71/72 bleibt für Demonstrations-Tracking verfügbar.
Das entspricht NICHT bereits einer physisch übergebenen realen Kanne.

**Wichtige Grenzen:** Halterpositionen und Zuverlässigkeit an heißer Kanne
noch nicht mechanisch, thermisch, lebensmittelhygienisch oder
wasserdicht validiert. Vor Echthardware geometrische
Außenkanten vermessen, Halter steif verschrauben/clippen, glänzende
Oberflächen entspiegelt ausführen, Kalibrierbilder ohne Milch durchführen.

## 3. Kamera und Messqualität

Wir stellen die **drei fest montierten Messkameras** von den alten
großräumigen Winkeln auf eine nähere Stereo-Lage um:
`cell_left=(-.74,-1.03,1.61)`,
`cell_right=(.58,-1.08,1.55)`,
`cell_back=(-.02,.42,1.69)`,
Ziel `(-.10,-.30,1.04)`, vertikales FOV 50°.
Einzelner Benutzer-Überblick (Kamerafahrt) bleibt unabhängig.

**Erster geprüfter Benchmark (13 Frames, 5Hz, 1280×960, Seed 14):**

| Vergleich | alte große Tags | neue kompakte Tags |
|---|---:|---:|
| Gültige Zwei-Kamera-Poseframes | 13/13 | 13/13 |
| Mittlerer 3D-Kannenpositionsfehler | 0,19077mm | 0,40767mm |
| Maximaler 3D-Kannenpositionsfehler | 0,34736mm | 0,59270mm |
| erkannte stationäre Referenzen | 8/8 | 2/8 |

Auswertungen sind von **derselben virtuellen Bewegung, Seed und
Pixelauflösung**; Kamerapositionen und Blickfelder unterscheiden sich
absichtlich, weil auch die kleinere Tagmontage ein neues Gesamtrig benötigt.
Die kompakte Variante ist also **noch unzuverlässiger für die
Stationskalibrierung**; maximale Spitze überschreitet 0,5mm.
**Nachtest nach Verlegung auf Gehäuseoberseiten:** Die Sensorpose blieb
13/13 gültig; die festen Markierungen verbesserten sich auf **4 von 8**.
Erkannt wurden nun Siebträgergriff, Kannenparkplatz, Gießplatz und
Roboterbasis. Maschinenfront/Mühle/Milchdispenser und Rinser benötigen
noch bessere Sichtwinkel oder eine ergänzende Kamera bzw. ein separates
Kalibrierrig. Dieser Nachtest ist erfolgreich durchgelaufen; der
höchste virtuelle Kannenfehler blieb jedoch 0,593mm.
Die hier genannte Fehlerzahl ist ein POSITIONSFEHLER des modellierten
Kannenbezugs, nicht unabhängig kalibrierte echte Ausgießerspitze.

### Pixelbudget

36h11-Decoding braucht gewöhnlich eine gewisse schwarze Quadratgröße
in Pixeln (in Praxis oft ≥20-30px für zuverlässig mehrwinkelige Pose).
Deswegen prüfen wir Pixelauflösung / Tiefenschärfe und ruhige Kante statt
eine kleinere Druckgröße durch willkürlich größere Kameratranslation
zu kompensieren. Die PnP-Schätzung verwendet die **aktive Kante**
und nicht die Außenkante inklusive weißem Rand.

## 4. Lokal auf Codespaces/Colab wiederholen

```bash
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa

# Kleine Tags – tatsächliches Kamera-RGB und aus diesen Bildern Pupil PnP
python -m robobarista.camera_joint_replay \
  --marker-profile compact --capture-only \
  --frames 13 --fps 5 --seed 14 \
  --width 1280 --height 960 \
  --output runs/compact_tags

# Große frühere Tags, identische generierte Originalbewegung
python -m robobarista.camera_joint_replay \
  --marker-profile legacy --capture-only \
  --frames 13 --fps 5 --seed 14 \
  --width 1280 --height 960 \
  --output runs/large_tags

# Optional Roh-RGB für LeRobot (kann viele MB verbrauchen!)
python -m robobarista.camera_joint_replay \
  --marker-profile compact --capture-only --save-camera-frames \
  --frames 13 --fps 5 --seed 14 --width 1280 --height 960 \
  --output runs/compact_for_lerobot

# Nur wenn 'lerobot' zusätzlich installiert ist:
python -m robobarista.lerobot_native_latte \
  --capture runs/compact_for_lerobot \
  --output runs/lerobot_virtual_latte

# Gesamter Kaffeeablauf mit den kleinen realistisch platzierten Tags:
python -m robobarista.full_barista_sensor_video \
  --marker-profile compact --output runs/compact_whole_coffee \
  --width 800 --height 600 --capture-fps 6 --playback-fps 6 \
  --repeat 1 --frames-leg 5
```

**Nicht daraus schließen:** gemessene real-world Genauigkeit oder belegte
Roboter-Firmware-Motion. Die nächsten echten Schritte sind
physische Marker mit gemessener Kantenlänge, Perspektivkontrolle,
unabhängiges Kalibrierobjekt, echte Kameradaten und
Robotergelenk-/Milchfluss-Regelung.
