# RoboBarista – FAIRINO FR5 / Camera-Based Pitcher Motion Capture

**Status (21.10.2026):** Virtuelles Kamera-Motion-Capture und erstes **kinematisches FR5-Sechs-Gelenk-Replay** erfolgreich in MuJoCo getestet. Das Kännchen ist dabei als **starre visuelle Attrappe** ans FR5-Handgelenk montiert. **Kein physikalisches Greifen, keine Motorregelung, keine echten Flüssigkeiten und keine reale Hardware.**

## Neue Standard-Szene: kompakte Artly-inspirierte Kaffeeinsel (Oktober 2026)

**Ab jetzt Grundlage unserer räumlichen Entwicklung:** [COLAB_WORKCELL.ipynb](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_WORKCELL.ipynb)

`robobarista/barista_cell.py` erzeugt unabhängig vom alten Kameratest die Szene `rb_artly_inspired_island.xml` aus dem vorhandenen FR5-Modell und geometrischen Grundformen. **Artly ist eine konzeptionelle Inspiration, kein behaupteter CAD-Nachbau.** Arbeitsfläche z=0,82 m, FR5 zentral auf 9 cm Sockel, kompakte Tischinsel, Wandbegrenzungen und drei erhöhte reale MuJoCo-Kameras. Stationen: Espresso, Mühle, automatischer Tamper, Siebträger-Ablage, Milchspender, Milchkanne, Übungsbereich mit **Kunststoffbecher** und Rinser. Sie sind über `STATIONS` als Koordinaten/Interaktionspunkte definiert.

**Neu zur bisherigen Szene:** bewegliche Milchkanne (`rb_pitcher_free`, Masse und Kollisionen) und ein leichter transparenter Kunststoff-Übungsbecher (`rb_plastic_cup_free`, Masse/Kollision am Boden); der Tamper besitzt einen realen Schubgelenkantrieb (`rb_tamper_motor`). *Die Demo* bewegt Kännchen und Tamper noch kinematisch (ohne echten FR5-Griff, dynamische Tamper-Kraft, Milch/Flüssigkeit oder Kaffeeextraktion). Die Becher sind vereinfachte Volumenmodelle ohne echte Wand-/Schaumphysik.

**AprilTags:** Drei stationäre Ansichten von vorne links, vorne rechts und hinten. Für jede Station eigene erhöhte, nach oben gerichtete AprilTag-Referenzmarker. Auf beweglichen Gegenständen zusätzliche Marker: Kännchen seitlich und oben, Kunststoffbecher auf kleinem Markerträger. `robobarista/cell_demo.py` erkennt die Marker aus **gerenderten RGB-Bildern** (pupil-apriltags) und schreibt `visibility.json`; eine jederzeitige Sichtbarkeit bei verdeckenden Gegenständen oder anderen Greifstellungen ist damit **nicht pauschal garantiert**.

**Erreichbarkeit:** Der Grundriss `layout_topdown.png` zeigt eine räumliche Planungskontur; der echte FR5-Check löst außerdem pro Station eine Reihe von Musterposen durch inverse Kinematik und protokolliert `reachability.json`. Die bloße Erreichbarkeit einer Testpose garantiert noch keinen kollisionsfreien Anfahrweg; das folgt vor der physischen Umsetzung.

**Outputs:** gerenderte Szenenvorschau `workcell_preview.png`, MP4/GIF `workcell_demo.*`, Grundriss, Sichtbarkeits-/Reichweitenbericht; automatische GitHub-Tests unter [Workcell Preview](https://github.com/Coffee-Tobi/robocasa/actions/workflows/workcell.yml).

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

## Neuer Meilenstein: Der FR5 führt die Bewegung selbst mit sechs Gelenken aus

**[▶ FR5-Gelenk-Replay in Colab öffnen](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_ROBOT.ipynb)**

- `robobarista/robot_replay.py` verwendet den unveränderten FAIRINO-FR5-Roboter aus den Third-Party-Assets. Für die neue Szene steht der FR5 neben der Arbeitsfläche auf einem Sockel.
- Das Kännchen ist **als starres, masseloses Objekt** am Roboterhandgelenk befestigt. Nur die sechs echten Gelenkwinkel verändern seine Pose; `mujoco.mj_forward()` ermittelt unabhängig die Ist-Pose.
- Inverse Kinematik (SciPy) prüft mehrere alternative Armhaltungen über die gesamte synthetische Gießbewegung. Es werden echte Gelenkgrenzen und MuJoCo-Kontakte der FR5-Kollisionskörper geprüft, einschließlich Greiferkontakten. Die rein visuelle Kännchengeometrie ist dabei **nicht kollisionsaktiv**.
- Film `fr5_replay.mp4` (bei verfügbarem FFmpeg), Animation `fr5_replay.gif`, unabhängige Soll/Ist-Markierungen im Kamerabild (**orange = Soll-Ausgießerspitze, grün = FR5-Ist-Ausgießerspitze**), `ik_error.png` und alle Gelenkwinkel/Fehler in `trajectory.json`. Der Bericht `summary.json` trennt erfolgreiche IK von Kollisions- und Grenzprüfungen.
- **Vollständiger automatischer GitHub-Actions-Test (25 Frames) bestanden:** alle 25 Bewegungsposen per FR5-Gelenken reproduziert, keine Gelenkgrenzen verletzt, keine registrierten Fremd- oder Selbstkollisionen der berücksichtigten Roboter-/Greiferkörper. Das sind **idealisierte kinematische** Ergebnisse ohne dynamische Motorführung oder physikalische Kännchenkollision.

**[GitHub Actions – FR5 Robot Joint Replay](https://github.com/Coffee-Tobi/robocasa/actions/workflows/fr5-replay.yml)** stellt den Film und Diagnosebericht als herunterladbares Artefakt bereit.

## Start in Google Colab

| Notebook | Zweck |
| --- | --- |
| **[COLAB_START.ipynb](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_START.ipynb)** | **Standard**: Drei Kameras, ein AprilTag 3, gemeinsame Poseoptimierung |
| **[COLAB_WORKCELL.ipynb](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_WORKCELL.ipynb)** | **NEUE STANDARDSZENE:** gesamte Artly-inspirierte Kaffeeinsel, bewegliche Milchkanne und Kunststoffbecher, Tamper, Kameras und AprilTag-Sichtcheck |
| **[COLAB_ROBOT.ipynb](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_ROBOT.ipynb)** | **JETZT:** echter FR5-Gelenk-Dry-Run, Video mit Soll-/Ist-Ausgießerposition, Kollisions- und IK-Prüfung |
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

**Als Nächstes:** Die kameragemessene Bahn aus `COLAB_MOTION.ipynb` anstelle der synthetisch vorgeschriebenen Wunschbahn mit dem FR5 nachfahren lassen. Dafür müssen wir Koordinatensysteme und Zeitstempel abgleichen, die gesamte Greifer-/Kännchengeometrie für physikalische Kollisionen modellieren und dynamische Bewegungs- und Geschwindigkeitsgrenzen prüfen. Das neue kinematische FR5-Dry-Run-Notebook ist bereits erfolgreich automatisiert getestet.

Ein zusätzlicher Tag oder eine vierte Kamera kommt nur dann dazu, wenn Tests über den ganzen Bewegungsbereich zeigen, dass wir sie wirklich benötigen. Der zweite Marker verbessert möglicherweise Sichtbarkeit, verlangt aber genaues Einmessen seines Starrkörperversatzes.

Langfristige Roadmap: [Roadmap](planning/ROBOBARISTA_ROADMAP.md) · [Backlog](planning/ROBOBARISTA_BACKLOG.csv) · [Latte-Art-/Artly-/One-Shot-Recherche](planning/ROBOBARISTA_MARKTRECHERCHE_20261020.md).

Projektcode: MIT. Drittmodelle und Bibliotheken unter ihren jeweiligen Lizenzen.

