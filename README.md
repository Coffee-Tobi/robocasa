# RoboBarista – FAIRINO FR5 / Camera-Based Pitcher Motion Capture

**Status (21.10.2026):** reproduzierbarer **statischer** MuJoCo-Kamera-Prototyp mit Messdaten aus Google Colab. Noch **kein dynamisches Motion Capture**, kein funktionierendes Greifen/Gießen, keine reale Hardware.

## Gewählter MVP-Aufbau

**Drei virtuelle RGB-D-Kameras, ein seitlicher AprilTag 36h11 am Milchkännchen, offizieller AprilTag-3-Detektor (`pupil-apriltags`) und SciPy-Mehrkamera-Optimierung.**

**Erste Messergebnisse:** Drei statische MuJoCo-Posen mit CPU/OSMesa, maximal **0,137779 mm absolute Ausgießerspitzenabweichung** und **0,045833° Kännchenwinkelabweichung**. Sowohl `pitcher_status` als auch `scene_status` waren **PASS**. Die Kamerakalibrierung ist in der Simulation exakt; diese Werte sind **kein Nachweis realer Genauigkeit oder kontinuierlichen Trackings**.

Die bisherigen 16 Vergleichsvarianten, einschließlich Caliscope und vier Kameras, bleiben reproduzierbar:

- [Alle gemeldeten 16 Ergebnisse als CSV](results/2026-10-21_rig_comparison_user_report.csv)
- [Vergleichsergebnis, Fehlersuche und Entscheidung](results/2026-10-21_camera_rig_decision.md)
- [Aktueller Projektplan / nächste Schritte](planning/ROBOBARISTA_DREI_STUFEN_MVP.md)

## Neuer Entwicklungsstand (21.10.2026): Bewegtes Kännchen

**Nächster Meilenstein implementiert:** Das Script `python -m robobarista.record_motion` bewegt das virtuelle Kännchen in einer vorprogrammierten sanften Gießgeste (annähern, neigen, angedeutet gießen, aufrichten, zurückfahren). Drei echte MuJoCo RGB-D-Kameras erzeugen pro Zeitpunkt eigene Aufnahmen. AprilTag 3 + die festgelegte gemeinsame Optimierung schätzen **ohne Zugriff auf die MuJoCo-Körperpose** Kännchen und Ausgießerspitze sowie – wenn ausreichend sichtbar – die Position der Spitze relativ zur Tasse. Die Ground Truth wird **in eigenen Evaluationsdateien** gespeichert.

**Realer automatischer Testlauf in GitHub Actions bestätigt (Linux/OSMesa, 25 synthetische Frames):** 25/25 Kännchenposen gemessen, maximal **0,257 mm Spitzenfehler** und **0,119° Winkelfehler**, `status=PASS` für diese beiden Kännchengrenzen. Die **Spitze relativ zur kamerabasiert einmal eingemessenen Tasse** war in allen 25 Frames berechenbar, wies aber einen maximalen Fehler von **0,818 mm** auf. Das übersteigt den langfristigen 0,5-mm-Wunsch und ist **nicht Teil dieses PASS-Kriteriums**. Die Tasse selbst war nur in 7/25 Frames erneut sichtbar; ihr aus Frame 0 gemessener fixer Bezugsrahmen blieb erhalten. Keine echte Latte-Art-Bewegung oder realistische Kamerastörungen geprüft.

**[Hier direkt das neue Animations-/Motion-Capture-Notebook starten](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_MOTION.ipynb)**. Es zeigt `preview.gif`, `trajectory.png`, `validation.png` und den zeitgestempelten Verlauf als Tabelle; `trajectory.csv`, `evaluation.csv` und RGB-Einzelbilder werden separat gesichert. Die Animation ist eine **synthetische kinematische Probe** ohne Flüssigkeit, echte menschliche Demonstration, Roboterbewegung oder kollisionsgeprüfte Ausführung. Fehlende Kameraposen werden als fehlend berichtet, nicht mit der Simulator-Wahrheit ausgefüllt.

## Start in Google Colab

| Notebook | Zweck |
| --- | --- |
| **[COLAB_START.ipynb](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_START.ipynb)** | **Standard**: Drei Kameras, ein AprilTag 3, gemeinsame Poseoptimierung |
| **[COLAB_MOTION.ipynb](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_MOTION.ipynb)** | **Jetzt ausprobieren**: animierte Bewegungssequenz + drei Kameraperspektiven + zeitgestempelte geschätzte 6D-Bahn |
| **[COLAB_COMPARE.ipynb](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_COMPARE.ipynb)** | **Forschung/Archiv**: 16 Varianten mit 1/2 Tags, 3/4 Kameras, OpenCV/AprilTag 3, direkter Optimierung/Caliscope |

In Colab **Laufzeit → Laufzeittyp ändern → Hardwarebeschleuniger: Keine** und **Laufzeit → Alle ausführen** wählen. Das Notebook lädt den aktuellen Code, installiert die OpenGL-/OSMesa-Abhängigkeiten und zeigt anschließend Kamerabilder und Ergebnisse. Eine GPU ist nicht erforderlich.

Das dritte Parteimodell [ma-weiss/Fairino-Mujoco](https://github.com/ma-weiss/Fairino-Mujoco) wird während des Starts nachgeladen (Commit `459570714fc68c92e1fb3605cb60b4d8ef84419b`); keine Assets im Repository einchecken.

## Was das Programm macht

1. `robobarista/scene.py`: Virtuelle Szene mit FR5, Tasse, Kännchen, Marker und Kameras konfigurieren.
2. `robobarista/geometry.py`: AprilTag 3 aus Kamerabildern erkennen und daraus die 6D-Kännchenpose mit mehreren Blickwinkeln berechnen; vorerst ergänzend idealisierte Tiefenbilder.
3. `robobarista/demo.py`: Vergleich der berechneten Pose mit der **nur zum Prüfen verwendeten** MuJoCo-Ground-Truth. Ausgabe von Position/Winkel, Abweichung der Ausgießerspitze, Anzahl brauchbarer Kameras, Fehlmessungen, RGB-/Tiefenbildern und CSV/JSON.
4. `robobarista/benchmark_gates.py`: `pitcher_status` und `scene_status` unterscheiden; andere Objekte können `scene_status=FAIL` verursachen, obwohl das Kännchen korrekt vermessen wird.
5. `robobarista/compare_rigs.py`: Vergleich der 16 Möglichkeiten und getrennte Diagnose alter Ergebnisse; `caliscope_integration.py` bleibt als Forschungsalternative.
6. `tests/` und `.github/workflows/python-tests.yml`: automatisierte Tests unter Python 3.13.

`runs/` ist **temporärer Ausgabeordner und wird nicht automatisch mit GitHub synchronisiert**. Die im Chat übermittelten numerischen Zusammenfassungen sind unter `results/` gespeichert; die ursprünglichen detailreichen Colab-Daten und Renderbilder wurden nicht hochgeladen.

## Nächster Schritt

**Die neu implementierte bewegte Funktionsprobe validieren** und daraus anschließend eine echte, reproduzierbare Demonstration samt mehr unterschiedliche Neigungs-/Positionsfälle, Verdeckung durch Griff/Greifer/Roboterarm, Ausgießerposition **relativ zur Tasse**, danach sichere inverse Kinematik und Bewegungswiedergabe mit FAIRINO FR5 **ohne Milch**.

Ein zusätzlicher Tag oder eine vierte Kamera kommt nur dann dazu, wenn Tests über den ganzen Bewegungsbereich zeigen, dass wir sie wirklich benötigen. Der zweite Marker verbessert möglicherweise Sichtbarkeit, verlangt aber genaues Einmessen seines Starrkörperversatzes.

Langfristige Roadmap: [Roadmap](planning/ROBOBARISTA_ROADMAP.md) · [Backlog](planning/ROBOBARISTA_BACKLOG.csv) · [Latte-Art-/Artly-/One-Shot-Recherche](planning/ROBOBARISTA_MARKTRECHERCHE_20261020.md).

Projektcode: MIT. Drittmodelle und Bibliotheken unter ihren jeweiligen Lizenzen.

