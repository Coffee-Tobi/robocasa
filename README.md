# RoboBarista Vision · v0.2 (FAIRINO FR5 + drei RGB-D-Kameras)

**Status:** Erster wissenschaftlich überprüfbarer Wahrnehmungsprototyp, **kein fertiger Latte-Art-Roboter**. Der vollständige MuJoCo-Renderdurchlauf muss noch in Colab geprüft werden. Unsere acht reinen Geometrie-/Markertests bestehen lokal; die Übereinstimmung der simulierten Kameras mit MuJoCo-Ground-Truth ist **nicht** bereits belegt.

## In Google Colab starten

[**Direkt in Colab öffnen**](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_START.ipynb)

1. Notebook öffnen. Für MuJoCo/EGL in Colab am besten **T4 GPU** auswählen (Laufzeit → Laufzeittyp ändern); der erste Versuch lässt sich sonst eventuell mit OSMesa ausführen.
2. **Laufzeit → Alle ausführen**. Keine ZIP und keine manuelle Dateiübertragung nötig. Der Notebook-Code klont dieses Repository, installiert MuJoCo und OpenCV, lädt die FAIRINO-3D-Assets aus dem **MIT-lizenzierten [ma-weiss/Fairino-Mujoco](https://github.com/ma-weiss/Fairino-Mujoco)**-Projekt (geprüfter Commit `459570714fc68c92e1fb3605cb60b4d8ef84419b`) und rendert die Szene.
3. Den Testbericht (`runs/first_benchmark/summary.json`) und die Kameramontage (`rgbd_montage.jpg`) ansehen. **FAIL ist ein valides Forschungsergebnis, kein Erfolg.**

## Neues Experiment: Direktes Kännchen-Motion-Capture (Opt-in)

Die Projektentscheidung ist jetzt **Option 2: 6D-Bewegung des Milchkännchens erfassen**, nicht Handhandschuhe oder den Roboterarm manuell führen. Siehe [Dreistufenplan](planning/ROBOBARISTA_DREI_STUFEN_MVP.md).

**Im Colab-Notebook gibt es Schritt 7:** Zwei echte AprilTag-36h11-Markierungen auf starren, seitlichen, entgegengesetzt ausgerichteten virtuellen Montageflächen am Milchkännchen; zwei nähere Schrägkameras und eine dritte Gegenkamera; gemeinsame 6D-Pose-Schätzung aus den Bildpunkten beider Tags. Die Originalvariante mit oberem ArUco-Marker bleibt als Baseline erhalten.

Der Side-Tag-Test prüft drei **statische** Neigungs-/Positionskonfigurationen, nicht kontinuierliches Motion Capture. Pro Test werden der Positionsfehler des Kännchens, der Winkelabweichung, der **Ausgießerspitzenfehler**, die erkannten Kameras und separat der **relative Spitzen-zu-Tasse-Fehler** protokolliert. Die Side-Tag-Abnahme fordert höchstens 0,5 mm **absolute** Spitzenabweichung und 1° Kännchenwinkel sowie mindestens zwei Kameras je Objekt. Der relative Fehler zur Tasse bleibt explizit separat, bis die Tassenreferenz verbessert ist.

**Keine Zusicherung einer erreichten Genauigkeit:** Der neue Side-Tag-Modus wurde in dieser Chatumgebung nicht mit vollständigem MuJoCo-Rendering ausgeführt. Er muss in Colab anhand der drei Bildmontagen, Marker-Sichtbarkeit und des Messberichts überprüft werden. Selbst ein PASS unter idealen Simulationsbedingungen garantiert keine 0,5-mm-Präzision im realen Aufbau. Besonders Markerträger-, Objekt-/Spitzen-Kalibrierung, Objektbewegung, Kamerazeitsynchronisierung und reale Kalibrierfehler sind noch zu testen.

Für einen direkten Vergleich im installierten Colab-Projekt:
```bash
MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python -m robobarista.demo --pitcher-side-tags --output runs/pitcher_side_tags --trials 3 --width 1280 --height 960 --max-spout-mm 0.5 --max-pitcher-rotation-deg 1
```

## Vergleichsstudie: 1/2 AprilTags × 3/4 Kameras × zwei Detektoren × zwei 3D-Methoden

[**Vergleich direkt in Google Colab öffnen**](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_COMPARE.ipynb)

Neues separates CPU-Notebook `COLAB_COMPARE.ipynb` startet **16** kontrollierte MuJoCo-Versuche: 1 oder 2 AprilTag-36h11-Marker auf dem Milchkännchen, 3 oder 4 Kameras, pro Aufbau jeweils OpenCV vs. **AprilTag 3** (`pupil-apriltags`, PyPI 1.0.4.post11; Python 3.13 manylinux verfügbar). Alle Varianten verwenden identische drei Kännchenposen, denselben Seed und dieselbe Auflösung (1280×960); **die Kamerastandorte sind auf die Variante abgestimmt**: bei einem seitlichen Tag alle 3/4 Kameras auf seiner sichtbaren Seite, bei zwei Tags 2+1 bzw. 2+2 Kameras auf den gegenüberliegenden Seiten. **Ein direkter Detektor-Vergleich bleibt exakt kontrolliert**, weil OpenCV und AprilTag 3 jeweils dasselbe Layout nutzen. Der 1-vs-2-Tag-Vergleich ist dagegen bewusst ein *Vergleich vollständiger Systemdesigns* und kein isolierter Test ausschließlich der Tag-Anzahl. Falls keine Konfiguration die Qualitätsprüfungen besteht, wird ausdrücklich **kein Gewinner** behauptet.

**Caliscope integriert (0.11.8, BSD-2-Clause):** Im selben Notebook vergleichen wir zusätzlich die echte `caliscope.api.ImagePoints.triangulate(CameraArray)`-Funktion. Sie trianguliert die mit OpenCV bzw. AprilTag 3 erkannten Marker-Ecken aus mehreren Kameras. Unsere kleine Kabsch-Starrausrichtung berechnet daraus die 6D-Kännchenpose. Das ist ein realer Rekonstruktionsvergleich, **noch keine Kamerakalibrierung mit Caliscope**: MuJoCo liefert beiden Methoden identisch bekannte ideale Kameramatrizen. Caliscopes separate ChArUco-/ArUco-Kalibrierung benötigt viele unterschiedliche Ansichten einer fest vermessenen Tafel und gehört in einen späteren Versuch mit Kamerakalibrierfehlern. Ein Vergleich der Solver ist hier außerdem nicht vollständig inputidentisch: Unser bisheriger Schätzer nutzt neben RGB-Ecken zusätzlich virtuelle Depth-Werte als Nebenbedingung, Caliscope lediglich 2D-RGB-Ecken. Falls Caliscope eine Tag-Ecke nicht aus mindestens zwei Perspektiven triangulieren kann, gilt das als Messausfall und nicht als Fehler von 0 mm.

Einzelberichte `runs/rig_comparison/<variante>/summary.json` und `pose_errors.csv`, Bilder, `runs/rig_comparison/comparison.csv` plus Logs. Das Notebook bietet am Ende einen ZIP-Export an. Strenge Zielwerte: 0,5 mm absolute **Ausgießer**abweichung und 1° für das Kännchen; die relative Ausgießer-zu-Tasse-Abweichung ist ein zusätzliches Diagnosemaß und wegen der bisherigen Tassenkalibrierung **noch nicht in der PASS-Bedingung**. Nur drei **statische** Posen; kein Nachweis für zeitkontinuierliches Motion Capture oder reale Kameras.

Die Studie untersucht insbesondere: Verbessert eine vierte Kamera den sichtbaren Marker? Beseitigt der zweite Markertyp bzw. die zweite starre Fläche Verdeckungsprobleme? Ist der offizielle AprilTag-3-Detektor genauer als OpenCVs AprilTag-Unterstützung? Die Daten entscheiden; wir stellen keine hypothetischen Vorteile als bereits gemessen dar.

**Kalibrierungsframeworks** Caliscope und Multical bleiben für reale Kamera-Linsen/Kameraposen vorgesehen: Im Simulator sind die Werte bereits exakt und eine zusätzliche Kalibrierung wäre für diesen Benchmark kein fairer Genauigkeitsgewinn. TagSLAM/ROS kommt erst infrage, wenn die leichtere direkte Posenoptimierung in der bildbasierten Spur nicht ausreicht.

## Was läuft

- Ein FR5-Roboter mit vorhandenen Meshes und PGI-140-80-W-Greifer (vom Drittprojekt übernommen), einfache Arbeitsfläche mit vorerst geometrisch vereinfachtem Becher, Kännchen, Behälter und Hindernis; **statische Pose** für reproduzierbaren ersten Wahrnehmungstest.
- Drei simulierte RGB-D-Kameras (`rgbd_front_left`, `rgbd_front_right`, `rgbd_overhead`) mit bekannter Kalibrierung; Farb- und metrische Tiefenbilder. **Noch keine echten Sensorfehler**, keine reale Kalibrierung.
- ArUco-ID an **allen testweise greifbaren Objekten** und einem Werkzeug-Referenzpunkt; Erkennung aus RGB, 6D-Pose per OpenCV `solvePnP`, drei Kameras gemeinsam optimiert durch SciPy **Huber-gewichtete Reprojektion**. Tiefenbilder liefern eigenständige metrische Zusatzmessungen.
- Aus bekanntem Marker-zu-Objekt-Transform wird Objektkörperpose berechnet. Das System vergleicht diese **nur nach der Schätzung** mit der unabhängig aus MuJoCo ausgelesenen Ground-Truth-Pose. Künftige Ausgabe: Ausgießerposition aus bekanntem Starrkörper-Offset, Kollisionsabstände.
- Messbericht pro Versuch und Objekt: Übersetzung in mm, Winkel in Grad, Zahl sichtbarer Kameras, Pixel-Reprojektion, Ausfälle; `PASS` nur falls der strenge Grenzwert eingehalten ist (anfänglich **5 mm und 5°** bei min. zwei Kameras; diese Werte sind vorläufige Testziele).

## Was nicht behauptet wird

- **Noch keine nachgewiesene Genauigkeit unter 5 mm/5°** im vollständigen FAIRINO-Rendering. Positionen können durch verkehrte Texture-UV-Orientierung oder unzureichende Kamerasicht verfälscht sein. Die Fehleranzeige ist genau dafür da.
- **Noch keine Bewegung, Greifhandlung, Milch, Latte Art, SmolVLA oder eigene Fluidphysik.** Diese Dinge beginnen erst nach erfolgreicher und geprüfter 3D-Wahrnehmung.
- **Noch kein Rerun Live-Dashboard:** Die erste Colab-Version zeigt ihre drei Kameraansichten mit Overlays, Tiefenbilder, einen 3D-Vergleich und Messdateien direkt im Notebook. Ein interaktives Dashboard ist eine nächste Ausbaustufe.
- Für den FR5-Körper kennen wir die echte Gelenkpose aus der Kinematik; visuell wird zunächst ein ArUco-Tag am Werkzeug statt jeder einzelnen Gelenkachse unabhängig erfasst.
- Tags auf spiegelnden Objekten sind hier ideal gedruckte Simulationsflächen mit messbarem Montageoffset. In echter Welt können Reflexionen, Kamerakalibrierfehler, Bewegungsunschärfe und fehlende Tiefe die Genauigkeit beeinträchtigen.

## Technische Architektur

```text
MuJoCo: FR5 + Szene + 3 virtuelle RGB-D-Kameras
                  |
          RGB/Depth pro Kamera --------------------------+
                  |                                      |
      OpenCV ArUco, solvePnP                     Ground Truth (privat)
                  |                                      |
       SciPy gemeinsames 6D-Pose-Fit                      |
                  |                                      |
     bekannte Tag-zu-Objekt-Geometrie                     |
                  +-------------------- Fehlervergleich --+
                                       |
                         JPEG/PNG/JSON/CSV + Colab
```

## Code

- `robobarista/scene.py`: Download der upstream-Roboter-Assets und eigene testbare MJCF-Szene / Marker / Kameras.
- `robobarista/geometry.py`: Kamera-Kalibrierungen, Tag-Erkennung, PnP und Mehrkamera-Optimierung.
- `robobarista/demo.py`: RGB-D-Aufnahmen, separat ermittelte Ground Truth, Messbericht und Bilder.
- `tests/`: 8 schnelle Tests ohne MuJoCo, sodass wir Fehler in Geometrie/ArUco schon vor der Visualisierung entdecken.
- `planning/`: übernommener Projektplan, Recherche und Drei-Stufen-MVP.
- `COLAB_START.ipynb`: das eigentliche Colab-Einstiegsnotebook.

Drittprojekte werden **beim Colab-Start geklont**, nicht in dieses Repository kopiert. Die eigene Software ist MIT-lizenziert; FAIRINO-Assets unter eigener MIT-Lizenz der jeweiligen Quelle.

## Technische Grenzen / nächster Schritt

Der Kompilierungs-/Render-Test wurde in dieser Entwicklungsumgebung **nicht** ausgeführt, weil MuJoCo hier nicht installiert ist und externe Paketdownloads nicht funktionieren. Sollte Colab einen Fehler beim Aufruf `MjModel.from_xml_path`, beim Rendern oder bei zu großen Winkelfehlern melden, ist das ein sichtbarer Integrationsbefund; `FAIL` darf nicht in `PASS` umgedeutet werden. Dann überprüfen wir zunächst Mesh-/XML-Kompatibilität, Kamera-UV-Orientierung und Sichtbarkeit der Markertafeln, bevor wir Bewegung trainieren.
