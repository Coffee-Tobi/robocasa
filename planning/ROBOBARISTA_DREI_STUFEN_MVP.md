# RoboBarista – verbindliche Dreistufenstrategie (20. Oktober 2026)

**Festgelegt:** FAIRINO FR5 als Hardware-Ziel und virtueller Arm, **ein Arm**, zwei Werkzeuge/Greifaufsätze gegebenenfalls **nacheinander**. Keine Hardwarebeschaffung für die ersten zwei Stufen. **Drei simulierte Kameras**, die Objektpose nicht bloß anzeigen, sondern **aus den eigenen RGB-Bildern rekonstruieren**. Keine laufende Kamera-Korrektur während des eigentlichen Bewegungs-Replays. Später echter Heart-Latte-Art-Versuch. Vorläufig keine Simulation von Milch/Schaum/Crema, keine Espresso-/Siebträger-/Mühlen-/Dampflanzen-Projekte.

## Verbindliche Entscheidung (20.10.2026): Kännchen-Motion-Capture, nicht Hand-Motion-Capture

**Option 2 ist gewählt:** Demonstrationen werden über die unabhängig aus Kamerabildern gemessene **6D-Pose des Milchkännchens** aufgezeichnet (Position + Orientierung + Zeit); daraus folgen die Sollbewegung der Ausgießerspitze und später die Robotergelenkbewegung. **Keine Handschuhe, keine verpflichtende menschliche Handpose, keine manuelle Führung des Roboters als primäre Lernmethode.** Die FR5-Vorwärtskinematik dient bei der Ausführung zur Zustandsabschätzung; Kameras dienen zusätzlich zur Initialisierung und optionalen Qualitätskontrolle. Die echte MuJoCo-Pose darf weiterhin nur zur Validierung verwendet werden.

**Neue Markerstrategie:** Mindestens zwei seitliche, verschieden ausgerichtete ArUco-/AprilTag-Referenzflächen auf einer **starren, bekannten Halterung am Kännchen**. Kein Tag auf dem Griff, kein Tag im Greifbereich, keine Tags auf der Milchkontaktfläche; Sichtbarkeit während Neigung und Griffbelegung prüfen. Im Simulator zunächst dünne nicht kollidierende Markertafeln, in Hardware allenfalls lebensmitteltaugliche abnehmbare Halterung / lasergravierte Referenz – **den gedruckten Code selbst nicht anschweißen**. Der Marker-zu-Kännchen-zu-Ausgießer-Offset muss kalibriert und validiert werden. ArUco als erster reproduzierbarer Markerstandard; AprilTag als spätere Vergleichsoption, keine unbemerkte Umstellung.

**Kameras:** Drei feste, kalibrierte Blickrichtungen mit mindestens zwei brauchbaren Sichtlinien je relevanter Kännchenpose. Gegenüberliegende Seitenmarker sind nicht notwendigerweise gleichzeitig in einer Kamera zu sehen. Prüfen, ob Greifer, Handgelenk, Griff, Ausgießer und Kannenrotation Marker verdecken. Relative Kannenposition zur Tasse inklusive zeitlicher Synchronisation protokollieren.

**Präzisionsziele (noch nicht nachgewiesen):** vorläufig maximal **0,5 mm Translationsfehler der Ausgießerspitze** und **1° Winkelabweichung**, mit 0,2 mm als Stretch-Ziel. In Auswertungen auch Kännchenmitte, Spitzenfehler, Fehlerquantile, worst case, Erkennungsrate und Kamerasichtbarkeit getrennt ausweisen; kein PASS bei fehlender Messung. Ein Winkel von 1° kann an 10 cm Hebelarm bereits ~1,75 mm Spitzenversatz bedeuten; langfristig ist daher eine strengere Winkeltoleranz oder direkte TCP-Fehlergrenze entscheidend. Diese Ziele gelten **zunächst als Ideal-Simulationsziel** und nicht als Hardwareversprechen.

**Nachweisplan:** Baseline beibehalten und getrennten Side-Tag-Test entwickeln; mehrere Kannenpositionen/-winkel, mindestens zwei Kameraperspektiven, Arm-/Greifer-Verdeckungen, Auflösungs- und Distanzvergleich, reale Kamerastörungen/Kalibrierfehler und wiederholte dynamische Trajektorien im Zeitverlauf. Einen einmaligen 3-Trial-Test nicht als Beleg für submillimetergenaue dynamische Motion-Capture ausgeben.

## Caliscope als separates Vergleichsverfahren (21. Oktober 2026)

Caliscope 0.11.8 ist als echte Python-Triangulation über `caliscope.api.CameraArray`/`ImagePoints.triangulate` eingebaut, nicht bloß als Platzhalter. Die 16 Varianten kombinieren 1/2 Tags, 3/4 Kameras, OpenCV/native AprilTag-3-Erkennung und direkte robuste 2D-Bundle-Optimierung vs. Caliscope-3D-Triangulation + starre 6D-Anpassung. Bei der zweiten Variante wird nur RGB verwendet, bei der ersten zusätzlich die virtuelle Tiefenmessung; die Ergebnisse dokumentieren diesen Unterschied. Die neue Pipeline ist **nicht** dasselbe wie Caliscopes echte Kamera-Kalibrierung mit ChArUco-Aufnahmen. Solche Kalibrierungsversuche sollten erst mit simulierten Fehlern bzw. realen Kamera-Videos getrennt stattfinden.

Die API wird für Colab im Vergleichsnotebook zusätzlich installiert; vollständige MuJoCo-Ergebnisse sind noch nicht verfügbar. Die Messung entscheidet, nicht eine Behauptung submillimetergenauer Genauigkeit.

## Vergleich vor weiterer Implementierung (20. Oktober 2026)

**Entscheidung steht erst nach Messung fest.** Es werden 8 Varianten desselben idealen Simulators getestet: ein/doppelseitiger AprilTag-36h11 am Kännchen × 3/4 Kameras × OpenCV-/nativer AprilTag-3-Detektor. Das separate Notebook [COLAB_COMPARE.ipynb](../COLAB_COMPARE.ipynb) übernimmt die Ausführung; Messdateien werden aufbewahrt. Die Kamerakonfiguration ist ein **Kandidat**, nicht ein belegtes Optimum.

**Geometrie:** Je eine Blicklinie links und rechts von der Vorderseite, eine auf die Kannenrückseite und optional eine vierte, höhere Vorderansicht; möglichst große Markerabbildung im relevanten Bewegungsvolumen, gute perspektivische Trennung, Griff/Arm-Verdeckung und gleichzeitige Sichtbarkeit von mindestens zwei Kameras. Auch mit vier Kameras bleibt eine einzelne Tagseite beim vollständigen Verdecken ein Single Point of Failure. Zweite starre Referenzfläche erhöht Robustheit, verlangt jedoch zusätzliche genaue Transformkalibrierung.

**Wahrnehmung und Kalibrierung trennen:** Offizielles AprilTag 3 über `pupil-apriltags` vs OpenCV bei gleichen Renderbildern vergleichen. Caliscope/Multical für späteren realen Kamera-Rig-Abgleich, nicht als vermeintliche Verbesserung der bereits perfekten MuJoCo-Kameraextrinsik. 0,5 mm Ausgießer, 1° Kännchen und 0,2 mm Stretch-Ziel bleiben Forschungsgrenzen; zusätzlich relativen Fehler Spitze ↔ Tasse ausweisen.

## Stufe 1 – 3D-Wahrnehmung mit drei simulierten Kameras (jetzt)

### 1A. Virtuelle Anlage
- FAIRINO FR5 in vorhandener freier Physik-/Roboter-Simulation zum Laufen bringen (z. B. MuJoCo mit geprüftem URDF/MJCF oder existierender FR5-Integration).
- Einen einfachen, **starr befestigten** Milchkännchen-Greifer, eine geometrisch bekannte Ausgießerspitze und eine zunächst **fest geneigte** Espresso-Tasse in die Szene setzen.
- **Drei RGB-Kameras** mit festen, bekannten, einander überlappenden Blickfeldern simulieren. Mögliche Baseline: Draufsicht auf die Tasse, Seiten-/Schrägansicht für Kännchenhöhe und Handgelenk bzw. zweite schräge Außenkamera zur Redundanz. Genaue Positionierung durch Sichtfeldtest festlegen; nicht an drei beliebigen Orten verteilen.

### 1B. Wahrnehmung – ausdrücklich nicht Simulator-Ground-Truth
- Pro Kamera farbiges Bild + Bildzeitstempel + intrinsische und extrinsische Kalibrierung.
- Kännchen, Ausgießer, Tassenrand/Tassenmittelpunkt, FR5-Greifer und bei Bedarf Robotersegmente als nachvollziehbare Erkennungen mit sichtbaren Overlays.
- **Am Anfang Marker erlaubt:** ArUco/AprilTag oder bekannte asymmetrische Referenzpunkte an Kännchen/Tassenhalter/Greifer; OpenCV Kalibrierung + `solvePnP`, Triangulation, Multiview-Fusion. Keine Pflicht zur teuren Segmentierungs-KI.
- Aus mehreren Ansichten **3D-Position (x,y,z)** und **Orientierung (Quaternion oder Rotationsmatrix)** der notwendigen Objekte im selben Welt-/Tassenkoordinatensystem schätzen. Die Ausgießerspitze kann aus der erkannten Kännchenpose und einmal vermessener, fester Geometrie abgeleitet werden.
- Für den FR5 dürfen wir seine per Vorwärtskinematik berechnete Flanschpose separat anzeigen. Sie darf jedoch nicht unbemerkt die geforderte visuelle Pose-Schätzung der Kanne ersetzen.

### 1C. Drei getrennte Ebenen der Visualisierung
1. **RGB-Realität des Simulators:** die drei Kamerabilder, möglichst zeitlich synchron.
2. **Wahrnehmung:** erkanntes Objekt, 2D-Marker, 3D-Pose, Unsicherheit und Ausgießer-Tassen-Vektor.
3. **Simulator-Wahrheit (nur Referenz):** interne MuJoCo-Posen und Winkel der Objekte – zur **unabhängigen Fehlerprüfung**, nicht als Eingabe an den Kamerapose-Algorithmus.

### Abnahme Stufe 1
- Die Szene lässt sich nach einem Kernel-Neustart reproduzierbar starten.
- Bei mehreren Kännchen- und Tassenposen liegen quantitative **Translations-/Rotationsfehler**, Detektionsausfälle und Zeitstempel vor; keine Aussagen wie „erkennt genau“ ohne Messwerte.
- 3D-Trajektorie, Orientierung, Fehler und alle Kameras lassen sich synchron nebeneinander untersuchen.
- **Erst dann** Zielgenauigkeit für die spätere Ausgießerspitze festlegen. Drei Kameras allein garantieren keine gewünschte Genauigkeit; Auflösung, Baseline, Verdeckungen, Kalibrierung und spiegelnde Edelstahloberflächen begrenzen die Methode.

## Stufe 2 – Aufgezeichnete Bewegung nachahmen (noch keine Latte Art nötig)

### Woher kommt die Demo?
Die Kameras **erzeugen** keine Bewegung. Mindestens einmal muss eine **virtuelle Bewegung vorgeführt** werden, beispielsweise durch Teleoperation, interaktive Kontrolle des Kännchens, eine vorhandene Demo-Datei oder eine einfache extern geladene Referenztrajektorie. Das dient zunächst **der technischen Pipeline**, nicht als Nachweis gelungener Latte Art.

### Lern- und Ausführungskette

`3 synchronisierte Kamerafolgen` → `3D-Kännchenpose pro Frame` → `Tassenbezogene 6D-Trajektorie + Zeit` → `DMP / offline imitation` → `FR5-Endeffektor-TCP-Sollpose` → `bestehende IK / Servo-Funktion` → `Bewegung in MuJoCo`.

- Position **und** Orientierung über die gesamte Zeit berücksichtigen, außerdem Tempo, Beschleunigung und Kannenwinkel; nur Punkte bzw. Start/Ziel reichen nicht.
- Tassenwinkel ist zunächst konstant und kalibriert. Die Bewegung sollte relativ zum Tassenkoordinatensystem gespeichert werden.
- Einmalig den Starrkörpertransform **Robotergreifer ↔ Kännchen ↔ Ausgießerspitze** kalibrieren. Sonst kann ein korrekt berechneter Roboterflansch trotzdem den falschen Ort der Ausgießerspitze treffen.
- **Keine Online-Replanung** aufgrund neuer Kamerabilder während der Ausführung. Ein optionaler vorheriger Szenencheck ist davon unabhängig.
- Gelernte Bewegung mehrfach abspielen; Geometrie-, Orientierungs-, Zeit- und Kollisionsfehler messen.

### Abnahme Stufe 2
- Reproduzierbare virtuelle Demo mit drei Kameraframes und eigenständig rekonstruierter 6D-Pose.
- FR5 bewegt sich entlang der gewünschten Bahn; Vergleich zur Demo über **alle Frames** (inkl. Spitzen-TCP, Winkel, Zeit).
- Kein echtes Gießen, kein Milchschaum, kein Versuch, aus der DMP die richtige Latte-Art-Technik zu erfinden.
- Falls die Kamerapose zu ungenau/verdeckt ist, erst dann externe 6D-Tracker oder alternative optische Marker/Depth hinzufügen.

## Stufe 3 – Echte Latte Art (erst nach Stufe 1+2)

- Eine **fachlich korrekte Heart-Pour-Demonstration** organisieren: bevorzugt öffentlich nutzbare Daten aus der [One-Shot-Latte-Art-Arbeit](https://arxiv.org/abs/2610.16179), wenn die Autor*innen sie freigeben, ansonsten eigene reale Kännchen-/Tassen-Demonstration oder anderes nutzbares Material.
- Eine reale Demonstration unterscheidet sich von einer virtuellen Testtrajektorie: Nur die erste enthält echtes Know-how über Milchstrahl, Neigung, Höhe, Phasen und Tempo.
- Die Demonstration in gleiches 6D-Datenformat bringen und mit Open-Source-DMP/CDMP wie [movement_primitives](https://github.com/dfki-ric/movement_primitives) nachbilden.
- Mit FR5-Modell, Milchkännchen und Tasse ohne Flüssigkeit virtuell prüfen: Reichweite, Geschwindigkeit, Kollision, Orientierung, Ausgießerposition.
- Nach gesonderter Hardware- und Sicherheitsfreigabe kontrollierte echte Gießversuche mit standardisierter Milch, gleicher Ausgangsmenge, Qualität und Temperatur. Latte Art nach dem Gießen fotografieren/bewerten.
- **Wichtiger Vorbehalt:** Die erfolgreiche One-Shot-Publikation hat die Tasse während des Gießens aktiv geneigt. Eine feste Tassenneigung ist bei uns eine Vereinfachung, deren Eignung für ein erkennbares Herz erst getestet werden muss. Ein motorisierter Halter bleibt die erste optionale Erweiterung.
- Wenn wiederholbar gute Muster entstehen, kann man bei diesem System bleiben. Falls nicht, die tatsächliche Fehlerquelle bewerten (Pose/TCP, Tassenwinkel, Trajektorie, Milchqualität, Geometrie, Timing), erst dann eine zusätzliche Echtzeitkamera-Regelung planen.

## Bestehende Komponenten: Was wird wiederverwendet?

| Benötigt | Bevorzugtes freies Projekt bzw. vorhandene Funktionen | Eigene Arbeit |
|---|---|---|
| Roboterkinematik | FAIRINO FR5 URDF/SDK bzw. MoveIt 2 (Lizenz/Kompatibilität prüfen) | Verbindung der Zeittrajektorie mit Zielposen |
| Roboter- und Kamerasimulation | MuJoCo und geeignetes FR5-Robotermodell | Drei Kameras ausrichten; Kännchen/Tasse einsetzen |
| Kamerakalibrierung und Pose | OpenCV, ArUco, `solvePnP`, Triangulation | Virtuelle Kamera-Kalibrierung und definierte Objektmarker |
| Nachvollziehbare Visualisierung | Rerun | Datenströme vereinheitlichen und nebeneinander anzeigen |
| Bewegungslernen | DFKI `movement_primitives`, veröffentlichte One-Shot-Arbeit als Vorlage | Kamera-Pose in zeitlich korrekte Demonstration umwandeln |
| Ausführung & Sicherheit | MoveIt 2 / MuJoCo IK / FR5 SDK | Werkzeugtransformation, Ausführungsprotokoll, Abnahme |
| Latte Art | Reale Milch + vorhandene Forschung; **kein eigener Fluidsimulator** | Reproduzierbarer realer Versuch und Ergebnisbewertung |

## Regeln für Abhängigkeiten und Umfang
- Die Rolle des früheren SmolVLA-Tests ist höchstens eine spätere Referenz. Keine Verpflichtung, die 900-Schritte-RoboCasa-Evaluation weiterzuführen.
- Externe Motion-Capture-Tracker **nur optional**, sofern das kamerabasierte Verfahren nicht ausreicht oder keine kalibrierte echte Demonstration verfügbar ist.
- Zwei Greifwerkzeuge **nacheinander**, nicht als zwei unabhängig bewegte Greifer. Zunächst nur Kännchen.
- Espresso, Siebträger, Mühle, Dampflanze, Milchaufschäumen **außerhalb** der drei MVP-Stufen.
- Kein Zwang, ein allgemeines Robotik-VLA zu trainieren. Keine eigene numerische Milchphysik.

## Zuordnung zum Tracker

- **Stufe 1:** RB-23, RB-24, RB-25, RB-14, RB-47, RB-15, RB-07, RB-08, RB-16, RB-17, RB-48, RB-09, RB-13.
- **Stufe 2:** RB-06, RB-49, RB-18, RB-19, RB-20, RB-44, RB-50, RB-26, RB-27, RB-51, RB-31, RB-28, RB-36.
- **Stufe 3:** RB-42, RB-29, RB-37, RB-12, RB-46.
- **Optional / später:** RB-43 (Tracker), RB-45 (aktiver Becherhalter), RB-32/33/34 (Feedback), RB-38/39/40 (weitere Barista-Schritte).

**Quelle der Roadmap:** [ROBOBARISTA_ROADMAP.md](./ROBOBARISTA_ROADMAP.md) · [Aufgaben-CSV](./ROBOBARISTA_BACKLOG.csv) · [Linear-Import](./LINEAR_IMPORT.csv). Die CSV-Daten sind als Tracker-Vorlagen vorbereitet, **nicht** bereits als Live-Linear-Issues synchronisiert.
