# RoboBarista – abgeschlossener Kamera-/AprilTag-Vergleich

**Versuchsdatum:** 21. Oktober 2026 · **Umgebung:** Google Colab, CPU, MuJoCo `osmesa`  
**Status der Quelle:** Vom Nutzer in der Unterhaltung übermittelte Konsolenübersicht (16 Varianten) + anschließende `--analyze-only`-Diagnose. **Keine hochgeladenen originalen Colab-Dateien**. Alle 16 numerischen maximalen Ausgießer- und Winkelwerte sind vollständig und ohne Rundung in [der CSV](./2026-10-21_rig_comparison_user_report.csv) gespeichert.

## Projektentscheidung

**Gewählter Standard für nächste Entwicklungsschritte:**

- **3 Kameras**, die aus unterschiedlichen Blickwinkeln auf **eine** gut sichtbare seitliche Kännchenfläche schauen (im heutigen Simulations-Layout).
- **1 starr am Kännchen befestigter AprilTag 36h11** mit kalibriertem Transform zur Ausgießerspitze.
- **Offizieller AprilTag-3-Detektor** über `pupil-apriltags`, nicht OpenCVs eigenen AprilTag-Detektor.
- **Gemeinsame Mehrkamera-Poseoptimierung** aus `robobarista.geometry.bundle_adjust_object` (RGB-Tag-Ecken, ergänzt um virtuelle Tiefenmesswerte), nicht Caliscope als Standard-Solver.
- FR5 als einarmiger virtueller Roboter; nach gesichertem Greifen kann die Kännchenpose beim Ausgießen auch aus FR5-Kinematik und starrem Greifer-Kännchen-Transform folgen. Kameras dienen für Demonstrations-Motion-Capture und Validierung.

**Warum:** Aus drei statischen Testposen: maximal **0,13777915758536008 mm absoluter Ausgießerspitzenfehler**, maximal **0,045832767365699856° Kännchenwinkelfehler**, `pitcher_status=PASS`, `scene_status=PASS`, jeweils Kännchen in allen drei Versuchen von drei Kameras gesehen. Es ist die einfachste Konfiguration mit vollständig bestandener Szene. **Keine Garantie für reale Kamera-/Robotikpräzision.**

## Was die Fehlermeldungen wirklich bedeuten

`pitcher_status` bewertet den Erkennungs- und Messfehler des **Milchkännchens**; `scene_status` umfasst daneben auch Tasse, Behälter und Hindernis. In der Diagnose wurden folgende Unterschiede festgestellt:

- **OpenCV:** Die Kännchenspitzenmessung ist in allen acht OpenCV-Vergleichen über der gewählten 0,5-mm-Grenze (z. B. 1,468 mm mit einem Tag/drei Kameras). Das ist **ungenau**, nicht bloß unsichtbar; die Kännchen wurden jeweils in drei Versuchen erkannt.
- **Pupil AprilTag 3, zwei Tags:** Sämtliche Kännchenmessungen erreichten `pitcher_status=PASS`; die Gesamtszene scheiterte am **Hindernis**, das nur **eine statt zwei** brauchbare Kameras hatte. Beim direkten Optimierer waren die Hindernisfehler 5,017 mm (Durchgang 0), 7,190 mm (Durchgang 1) und 128,461 mm samt 142,011° (Durchgang 2). Fehlende zweite Sicht **und** ungenaue Posen – kein reiner Sichtbarkeitsfehler.
- **Caliscope + Pupil, ein Tag, drei Kameras:** Kännchen `PASS`, Szene `FAIL`, weil das Hindernis im dritten Versuch 5,3666 mm statt höchstens 5 mm Positionsfehler aufwies.
- **Caliscope + Pupil, zwei Tags und vier Kameras:** technisch kleinster Ausgießerspitzenfehler von **0,05495711955703933 mm** und Winkel **0,03520983704337764°** bei allen drei Kännchenerkennungen. **Kännchen PASS, Gesamtszene FAIL:** Hindernis jeweils nur von einer Kamera erfasst und **keine gültige 6D-Pose** trianguliert.
- **Ein einzelner Marker kann ausreichen** – aber nur, solange genug Kameras ihn über die gesamte Demonstrationsbewegung sehen. Eine zusätzliche Kamera auf derselben Seite verbessert Sicht-/Redundanz, aber garantiert bei Armverdeckung keinen Fortschritt.

## Benchmarkspezifikation und Grenzen

- 16 Varianten = 2 Detektoren (OpenCV, `pupil`) × 2 Rekonstruktionswege (robuste direkte 2D-Optimierung, Caliscope 3D-Eckentriangulation + Kabsch) × 1/2 Marker × 3/4 Kameras.
- **Drei statische** Kännchenpositionen/Neigungen (0°, 12°, 25°); noch **keine** sequenziellen Demonstrationsbewegungen; identische Seeds und Auflösung 1280 × 960.
- `PASS` für Kännchen: max. 0,5 mm **absolute Spitzenposition** und 1° Winkel, mindestens zwei brauchbare Kameras, gültige Schätzung in allen Versuchen. Für andere Objekte in der Gesamtprüfung: höchstens 5 mm und 5°. **0,2 mm** ist ein Stretch-Ziel, noch kein reales Soll für Hardware.
- **Absolute** Ausgießerspitze ist nicht der **relative Fehler Ausgießer ↔ Tasse**. Letzterer wurde separat erfasst, ist aber hier nicht in der Nutzer-Konsolenübersicht enthalten und bisher kein `PASS`-Kriterium.
- MuJoCo liefert **ideale, exakt bekannte** Kamerakalibrierung und vereinfachten Kännchenkörper/TCP. Die Ergebnisse sagen nichts Sicheres über zeitliche Synchronisation, Reflexionen, Bewegungsunschärfe, mechanische Montage, echte Tiefensensoren, Kameraextrinsikfehler, Sicht bei großem Kannenwinkel oder Roboterverdeckung.
- Unterschiede zwischen den beiden Schätzern sind **nicht ausschließlich algorithmisch**: Der direkte Schätzer nutzt zusätzliche virtuelle Tiefenwerte als Nebenbedingung, Caliscope lediglich optische 2D-Tag-Ecken.
- Die für einen Tag optimierten Kamerapositionen befinden sich auf derselben Seite; die Zwei-Tag-Anordnung verteilt die Kameras auf beide Seiten. Daher ist der Markerzahl-Vergleich ein **ganzer Rig-Vergleich**, kein isolierter Test der Anzahl der Marker.
- **Bei zukünftigen Tests fehlende Erkennung nie als 0-mm-Fehler zählen.** Erkennungsquote, Worst Case und zu wenige Kameras stets separat berichten.

## Nächste Arbeitssitzung

1. Drei-Kamera-Ein-Tag-Rig in `COLAB_START.ipynb` als primären Versuchsaufbau verwenden. Den 16-Wege-Vergleich in `COLAB_COMPARE.ipynb` als **Reproduktions-/Forschungswerkzeug** behalten.
2. Kamerabilder und Markerhaltung bei mehr Posen, insbesondere **Kännchenneigung, Drehung und Roboterarm-/Greiferverdeckung**, überprüfen. Bei Verdeckungen Kamera neu ausrichten und/oder zweiten Tag als Fallback testen, statt vorschnell zu behaupten, ein einzelner Tag sei immer sichtbar.
3. Mehr dynamische, zeitgestempelte 6D-Kännchendemonstrationen erfassen, Ausgießerspitzenbewegung **relativ zur Tasse** bewerten, keine echte Handpose als Ersatz.
4. Aus Kamera-Kännchenpose und festem TCP-Offset FR5-Inverse-Kinematik und kollisionsfreies virtuelles **Dry-Run-Replay ohne Milch** aufbauen.
5. Später reale Kalibrierfehler/Noise/Synchronisierung und Markerbefestigung prüfen, erst dann Hardwareentscheidungen.

## Speicherung und Reproduktion

- **Auf GitHub gespeichert:** alle 16 gemeldeten Maximalwerte mit Status, einfache Fehlerkategorien [als CSV](./2026-10-21_rig_comparison_user_report.csv) und dieses Protokoll mit den diagnostizierten Einzelproblemen.
- **Nur in der Colab-Laufzeit verfügbar, sofern noch nicht exportiert:** originale `summary.json`, `measurements.json`, `pose_errors.csv`, die RGB-/Tiefenbilder und alle stderr/stdout-Logs. Die tabellarischen Zusammenfassungen ersetzen diese Rohdaten **nicht**. Das Vergleichsnotebook bietet einen ZIP-Export an.
- **Rerun:** `COLAB_COMPARE.ipynb`; es lädt die aktuelle Codeversion und erzeugt getrennte `runs/rig_comparison/<variante>/`-Berichte. Das Hauptnotebook `COLAB_START.ipynb` konzentriert sich auf die gewählte Baseline.

