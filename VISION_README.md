# RoboBarista Vision – FAIRINO FR5 + drei RGB-D-Kameras

**Aktives Entwicklungsprojekt (20.10.2026).** Das frühere RoboCasa/SmolVLA-Projekt wurde auf separaten Archiv-Branches gesichert. Der Projektplan befindet sich unter [barista_tests/project_management](barista_tests/project_management/ROBOBARISTA_DREI_STUFEN_MVP.md).

## Direkt in Google Colab starten

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_START.ipynb)

**Kein Download und kein Upload erforderlich.** Das Colab-Notebook installiert Abhängigkeiten und klont dieses Repository selbst.

## Was Version 0.1 tatsächlich kann

1. Lädt das bereits vorhandene [FAIRINO FR5 MuJoCo-Modell](https://github.com/ma-weiss/Fairino-Mujoco), Commit `459570714fc68c92e1fb3605cb60b4d8ef84419b`, **MIT-Lizenz**. Das Modell/der Greifer werden nicht selbst erstellt.
2. Baut eine einfache MuJoCo-Arbeitsfläche mit FR5, vereinfachter Tasse, vereinfachtem Milchkännchen, Behälter und Hindernis. Objekte und Arm sind in Version 0.1 **nicht beweglich gesteuert**.
3. Fügt an später greifbaren Objekten und am Werkzeug **ArUco-Tags** an (teils auf virtuellen Haltern, für erste Sichtbarkeit).
4. Rendert drei virtuelle **RGB-D-Kameras**: links, rechts und schräg oben. Tiefenbilder stammen aus der MuJoCo-Engine als ideale metrische Tiefen, nicht aus echter Kamerahardware.
5. Erkennt Tags aus den **RGB-Bildern**, berechnet 3D-Positionen mithilfe der Tiefe und kalibrierter Kameraposen, schätzt die Orientierung mit **OpenCV solvePnP** und kombiniert mehrere Ansichten.
6. Liest separat MuJoCo-Ground-Truth und vergleicht die Positions-/Orientierungsfehler. Diese **wahren Objektposen fließen nicht in die Kameraschätzung ein**.
7. Speichert `rgbd_montage.jpg`, RGB-Overlays, Tiefen-NPYs, Kalibrierungs-JSON, `pose_errors.csv`, `measurements.json` und `summary.json`.

### Von uns geschrieben vs. wiederverwendet

- **Fremde Projekte:** MuJoCo, FAIRINO-Modell inklusive Meshes und Greifer, OpenCV ArUco/solvePnP, SciPy, NumPy, Pillow.
- **Eigener Code (`camera_benchmark.py`):** Szene, drei virtuelle Kameraposen, Tag-Montage, Bild-zu-Pose-Datenfluss, Vergleich gegen Ground-Truth und Genauigkeitsprüfung.

## Genauigkeit: ambitionierte Zielwerte, kein Erfolgsversprechen

Vorläufiger Benchmark: **max. 5 mm Positionsfehler, 5° Rotationsfehler**, jeder der fünf Tags in **mindestens zwei der drei Kameras**. Ein fehlender Tag oder Überschreiten der Schwelle führt zu `passed_strict_accuracy_gate=false`, Exit 2.

**Noch nicht nachgewiesen:** Der komplette MuJoCo-Renderdurchlauf war in der Entwicklungsumgebung nicht ausführbar, da dort MuJoCo fehlte und kein Download möglich war. Der Code braucht seinen ersten Colab-Integrationstest. Insbesondere müssen Textur-/Tag-Ausrichtung, Verdeckung und Kameraaufstellung anhand der tatsächlichen Bilder validiert werden. Die ideale Simulationskalibrierung und ideale Tiefe sagen noch nichts über reale Tiefensensor-Genauigkeit aus.

## Sind drei Kameras links/rechts/oben ideal?

Es ist eine **erste Hypothese**, kein Standard oder optimiertes Design. Die beste Aufstellung hängt vom gemeinsamen Sichtfeld, der Erreichbarkeit des Tag-Musters, Reflexionen, Basisabstand, Bildauflösung und den relevanten Posen ab. Der Benchmark soll es ermöglichen, andere Kamerapositionen quantitativ zu vergleichen. Falls ein Objekt von weniger als zwei Kameras gesehen wird, **ist die Vorgabe nicht erfüllt**.

## Was noch fehlt

- Realistische offene Milchkanne, realistisch geneigte Tasse, greifbare Objekte und beweglicher FR5
- Kalibrierungsunsicherheit/Tiefenrauschen echter RGB-D-Kameras; photometrische Effekte
- Individuelle 3D-Pose **jedes Robotersegments**; aktuell nur ein Tag am Werkzeug plus bekannte Robotergelenke
- Zeitliche Bewegung, Trajektorienlernen, Greifen, Kollisionsvermeidung
- Rerun-Live-Dashboard: derzeit Bildmontage + CSV/JSON, keine Live-Ansicht
- Echte Latte Art, Milchschaum, Aufschäumen und Siebträgerbedienung

## Quellen und Projektdokumentation

- [FAIRINO FR5 Modell (MIT)](https://github.com/ma-weiss/Fairino-Mujoco)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [OpenCV ArUco](https://docs.opencv.org/4.x/d9/d6a/group__aruco.html)
- [Projektplan](barista_tests/project_management/ROBOBARISTA_DREI_STUFEN_MVP.md)
- [Recherche zu kommerziellen Baristas](barista_tests/project_management/ROBOBARISTA_MARKTRECHERCHE_20261020.md)

## Archivierte Vorgänger

- [Vorherige RoboCasa-Codebasis](https://github.com/Coffee-Tobi/robocasa/tree/archive/robocasa-original-2026-10-20)
- [Alte SmolVLA-/Barista-Experimente](https://github.com/Coffee-Tobi/robocasa/tree/archive/previous-barista-experiments-2026-10-20)
