# RoboBarista Vision · v0.2 (FAIRINO FR5 + drei RGB-D-Kameras)

**Status:** Erster wissenschaftlich überprüfbarer Wahrnehmungsprototyp, **kein fertiger Latte-Art-Roboter**. Der vollständige MuJoCo-Renderdurchlauf muss noch in Colab geprüft werden. Unsere acht reinen Geometrie-/Markertests bestehen lokal; die Übereinstimmung der simulierten Kameras mit MuJoCo-Ground-Truth ist **nicht** bereits belegt.

## In Google Colab starten

[**Direkt in Colab öffnen**](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_START.ipynb)

1. Notebook öffnen. Für MuJoCo/EGL in Colab am besten **T4 GPU** auswählen (Laufzeit → Laufzeittyp ändern); der erste Versuch lässt sich sonst eventuell mit OSMesa ausführen.
2. **Laufzeit → Alle ausführen**. Keine ZIP und keine manuelle Dateiübertragung nötig. Der Notebook-Code klont dieses Repository, installiert MuJoCo und OpenCV, lädt die FAIRINO-3D-Assets aus dem **MIT-lizenzierten [ma-weiss/Fairino-Mujoco](https://github.com/ma-weiss/Fairino-Mujoco)**-Projekt (geprüfter Commit `459570714fc68c92e1fb3605cb60b4d8ef84419b`) und rendert die Szene.
3. Den Testbericht (`runs/first_benchmark/summary.json`) und die Kameramontage (`rgbd_montage.jpg`) ansehen. **FAIL ist ein valides Forschungsergebnis, kein Erfolg.**

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
