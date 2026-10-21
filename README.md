# RoboBarista – FAIRINO FR5 / Camera-Based Pitcher Motion Capture

## ▶️ Neues Haupt-Notebook: die gesamte RoboBarista-Simulation in **einem Colab** (21.10.2026)

**[☕ COLAB_ROBOBARISTA.ipynb – alle vier Experimente ausführen](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_ROBOBARISTA.ipynb)**

Dieses Notebook ersetzt die bisherige Kette einzelner Einstiegspunkte für unsere neue Artly-inspirierte Entwicklungszelle (die alten Notebooks bleiben für Einzeltests erhalten): **1. Kaffeeinsel und drei Kameras samt integriertem Mühlen-Tamper, 2. sieben komplette kinematisch kontaktgeprüfte Vorpositionsfahrten, 3. echte pixelbasierte AprilTag-3-Schätzung der virtuellen Zufallsbewegung und deren Nachfahren mit sechs FR5-Gelenken, 4. zusätzlich dynamische Sechs-Drehmoment-Motorsteuerung mit `mujoco.mj_step`**, Grundriss, Soll-Ist-Kurven und einem gemeinsamen ZIP-Export. Für Google Colab genügt **Laufzeit → Alle ausführen**, GPU nicht erforderlich.

**Getrennt verifizierte Ergebnisse:** 7/7 sichere **freie Vorpositionen** (kein Geräte-Andocken); 13/13 Kamera-Posen mindestens aus zwei virtuellen Ansichten erkannt und vom FR5 rein kinematisch nachgestellt (maximal 0,924 mm optischer Fehler gegen Simulatorwahrheit); alle 8 festen Referenz-AprilTags aus Bildern gemessen. Ein ergänzender Motorversuch mit um Faktor 2,5 gestreckter Trajektorie ist erfolgreich mit **12,36 mm maximalem und 4,46 mm mittlerem dynamischem Folgefehler**, ohne gemeldete Arm- oder Werkzeugkollision. **Die End-to-End-Workflow-Prüfung** für genau diese vier Schritte befindet sich unter [RoboBarista Complete Colab E2E](https://github.com/Coffee-Tobi/robocasa/actions/workflows/complete-colab.yml); bei erfolgreichem Lauf werden alle vier MP4-Videos als ein Actions-Artefakt bereitgestellt.

**Grenzen:** Die Kamera-Simulation besitzt idealisierte und bekannte Extrinsik, die Demonstration ist virtuell/zufällig (kein Mensch), der FR5-Greifer hält Kanne/Siebträger noch nicht physikalisch, die endgültigen Geräte-Andockkontakte sind nicht freigegeben. Torque-Regler und Objekt-Hüllmodelle sind Forschungscode **keine reale Roboter-Sicherheitssteuerung** oder Latte-Art-Fähigkeit.

**Status (21.10.2026):** Virtuelles Kamera-Motion-Capture und erstes **kinematisches FR5-Sechs-Gelenk-Replay** erfolgreich in MuJoCo getestet. Das Kännchen ist dabei als **starre visuelle Attrappe** ans FR5-Handgelenk montiert. **Kein physikalisches Greifen, keine Motorregelung, keine echten Flüssigkeiten und keine reale Hardware.**

## NEU (21.10.2026): Sieben sichere Vorpositionen und optisches Kamera→FR5-Replay

**[🎬 Safe Staging Colab – sieben Geräte-Anfahrten](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_SAFE_STAGING.ipynb)** · **[fertiger Video-Test mit MP4](https://github.com/Coffee-Tobi/robocasa/actions/runs/39632253591/artifacts/12358904241)**

Alle **sieben** benannten Geräte-/Werkzeug-Schnittstellen haben eine **kollisionsgeprüfte freie Vorposition** mit Gelenk-Hin-/Rückroute aus einem sicheren Startzustand gefunden. Die eigentlichen mechanischen letzten Zentimeter bis Brühgruppe, Dampflanze, Mühle und Milchspender sind **nicht** freigegeben. Alle Wege sind kinematisch; Robotermotoren, Griffkräfte und kontinuierliche hardwaretaugliche Kollisionsüberwachung fehlen.

**[🎬 Kamera-Motion-Capture → echter FR5-Gelenkreplay in Colab](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_SAFE_REPLAY.ipynb)** · **[Vergleichs-MP4 und Berichte](https://github.com/Coffee-Tobi/robocasa/actions/runs/39632915461/artifacts/12359487863)**

Eine **mit Seed reproduzierbare zufällige, sichere Gelenkbewegung** wird zunächst von einer **separaten virtuellen Kanne** vorgeführt und von drei MuJoCo-Kameras gefilmt. Aus gerenderten RGB-/Tiefenbildern rekonstruiert AprilTag-3 plus Mehrkamera-Optimierung die tatsächliche 6D-Bewegung: **13/13** Zeitpunkte optisch gültig (mindestens zwei Kameras); der FR5 fährt anschließend **13/13** ausschließlich gemessene Zielposen durch seine sechs Gelenke nach. Kein Zugriff auf die versteckten Quellgelenke/Quellpositionen während der Kameraschätzung oder beim Replay. Getrennt gemessener maximaler Fehler gegenüber der nur zur Bewertung gespeicherten Simulatorwahrheit: **0,924 mm** / **0,339°**. Alle **acht** festmontierten Stations-Referenzmarker wurden aus Kamerabildern im ersten Frame erkannt; maximale Positionsdifferenz zur Simulatorwelt **2,784 mm**. Dies sind **ideal kalibrierte virtuelle Kameras**, keine reale Präzisionsaussage. Ein zusätzlich schräg montierter AprilTag #72 verbessert die zeitweilige Sichtverdeckung.

Das neue Video stellt **die von den Kameras beobachtete Demonstration links und die nachgeführte FR5-Gelenkbewegung rechts** nebeneinander, dazu ein separater XYZ-Trajektorienvergleich. Für den vollständigen Aufbau, Code und Einschränkungen siehe **[technische Erklärung](planning/SAFE_CAMERA_REPLAY_EXPLAINED.md)**. Die früheren unabhängigen Kamera- und FR5-Benchmarks bleiben weiterhin im Repository.

## Neue Standard-Szene V0.2: Artly-inspirierte Kaffeeinsel mit Andockrahmen (21.10.2026)

**[Neue Szene in Colab öffnen](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_WORKCELL.ipynb)** · **[Verbindlicher virtueller Geometrie- und Sicherheitsplan](planning/WORKCELL_GEOMETRY_AND_SAFETY.md)**

Die kompakte freie Arbeitsinsel steht im Zentrum und trägt den FAIRINO FR5. Es gibt **sieben Stationen**: **Mühle mit integriertem motorischem Tamper**, Siebträger-Ablage, Espressomaschine, Milchspender, Kännchen-Ablage, Kunststoffbecher-Gießplatz und Rinser. **Keine separate Tamperstation mehr** (AprilTag #62 ist stillgelegt). Drei RGB-D-Kameras und oben montierte AprilTag-Referenzen bleiben erhalten. `robobarista/barista_cell.py` baut die MJCF-Szene als eigenes Modul, ohne ältere Forschungsnotebooks zu verändern.

**Sieben eigene Andockrahmen** in `robobarista/workcell_constraints.py`: Siebträger an Mühle, Siebträger an Espressomaschine, Kunststoffbecher an Espresso-Tropfschale, Kännchen an Espressodampflanze, Kännchen an Milchspender, Kännchen-Parkposition, Becher-Gießplatz. `DOCKS` speichert Position **in Metern**, Orientierungswinkel und Anfahrachsen; `rb_dock_*`-Sites visualisieren diese Ziele direkt in MuJoCo. Beispiel: Milchkanne an Dampflanze **(653, 222, 871) mm** relativ zum Szenenursprung. Alle Werte sind **nominale Konstruktionsannahmen**, keine aus echten Geräten vermessenen Präzisionswerte.

**No-Contact-Planungsfilter:** 30 mm Abstand bei Transit, 5 mm nahe erlaubtem Dock, Pose nur mit richtigem Werkzeug und max. 3 mm/2° nomineller Abweichung freigegeben. Auch Kanne **mit Griff und Ausgießer**, Siebträger und Becher werden als räumliche Körper approximiert; nicht nur der Handgelenkpunkt. `require_safe_segment` prüft Zwischenpositionen (max. 5-mm-Schritte) und **verweigert** einen geraden Weg durch die Espressomaschine; `require_safe_robot_state` blockiert bekannte MuJoCo-Kontakte der Arm-/Greiferkörper. Nur die jeweils benannte Schnittstelle kann bei richtigem Werkzeug eine räumlich eng begrenzte Ausnahme haben.

**Automatisch getesteter Stand:** Sieben Docking-Sites vorhanden und alle sieben *nominalen* getragenen Werkzeugkörper erfüllen den programmierten geometrischen Docking-Check. **Alle sieben Stationspunkte sind per IK erreichbar, aber nur drei von sieben getesteten Arm-Zielstellungen waren ohne MuJoCo-Kontakt**; die anderen vier sind für eine tatsächliche Armfahrt **noch nicht freigegeben**. Es müssen kollisionsfreie Alternativhaltungen und vollständige Bewegungswege geplant werden. Zehn verbleibende relevante AprilTags wurden jeweils in **8/8** kurzen Testframes erkannt (seitlicher Kännchenmarker weiterhin nicht). Der integrierte 55-mm-Mühlen-Tamper hat einen getrennten MuJoCo-Motortest mit Aus- und Rückfahrt bestanden, **ohne tatsächlichen Kaffeedruck**. GIF/MP4, `dock_targets.json`, `workspace_constraints.json`, `reachability.json` und Marker-Sichtbarkeit werden im [Workcell-Workflow](https://github.com/Coffee-Tobi/robocasa/actions/workflows/workcell.yml) erzeugt.

**Wichtige Einschränkung:** Es sind vereinfachte AABB-Gerätehüllen und Kugelmodelle für Werkzeuge. Weder vollständig kontinuierliche **Gelenkraum-Kollisionstests** noch dynamische Motor-/Kraftgrenzen, echte Greifversuche, Flüssigkeiten, präzises Hersteller-CAD oder realweltliche Sicherheitsfunktionen sind implementiert. Deshalb sind diese Abstände **Forschungs-/Planungswerte und keine Freigabe zur echten Roboterbewegung**.

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

