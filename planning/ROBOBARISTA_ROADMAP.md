# RoboBarista – Projekttracker und Roadmap

> **Stand:** 21.10.2026 · **Verbindliche aktuelle Strategie:** (1) Drei Kameras + **ein seitlicher AprilTag 3** am Kännchen + `pupil-apriltags` + eigene Mehrkamera-Optimierung; danach zeitgestempeltes 6D-Kännchen-Motion-Capture, (2) FR5-Dry-Run anhand der Demonstration ohne Live-Nachregelung, (3) echte Latte Art.  
> **Projektcode:** [Coffee-Tobi/robocasa, Branch main](https://github.com/Coffee-Tobi/robocasa/tree/main) · **Tracker:** Markdown und CSV, da GitHub Issues im Repository deaktiviert sind.  
> **Herstellerentscheidung:** FAIRINO FR5 **als Planungsbasis, noch kein Hardwarekauf**. Ein Arm verwendet mögliche Greifaufsätze **nacheinander, nicht gleichzeitig**.

**Aktuell maßgeblich:** [Konsolidierter Projektplan](./ROBOBARISTA_DREI_STUFEN_MVP.md) und [16 Kamera-/Tag-Benchmarkwerte](../results/2026-10-21_rig_comparison_user_report.csv). Die folgenden detaillierten RB-Aufgaben sind ein **historischer langfristiger Backlog**; einzelne unveränderte ältere Testannahmen sind keine aktualisierte Designentscheidung. Es wurden **keine** Linear-Issues angelegt.

## Unsere drei Umsetzungsstufen (maßgeblich)

**Detaillierte Spezifikation und technische Erfolgskriterien:** [ROBOBARISTA_DREI_STUFEN_MVP.md](./ROBOBARISTA_DREI_STUFEN_MVP.md).

| Stufe | Jetzt zu lösen | Nachweis / Go-No-Go |
|---|---|---|
| **1 – Wahrnehmung** | FR5, Milchkännchen, Ausgießer und fest geneigte Tasse in MuJoCo; drei überlappende synchronisierte Kameras. Kalibrieren; Objekte markieren/tracken; aus **Kamerabildern** 3D-Position und Orientierung ermitteln; Anzeige in Rerun und Vergleich mit MuJoCo-Ground-Truth. | Objekt-Pose nachvollziehbar und wiederholt präzise geschätzt, Fehler und Ausfälle sichtbar |
| **2 – Imitation** | Eine virtuelle Demo aufnehmen; die 6D-Kännchenpose mit den drei Kameras erfassen; zeitlich/geometrisch per DMP oder kompatibler Open-Source-Implementierung reproduzieren; Tool Center Point auf Ausgießerspitze kalibrieren; FR5 fährt die Trajektorie mit bestehender IK. | Roboter reproduziert Position **und Orientierung** zur richtigen Zeit ohne Echtzeitkorrektur und ohne Kollision |
| **3 – Latte Art** | Echte Heart-Demonstration aus veröffentlichten Daten oder eigener Aufnahme verwenden, virtuell trocken testen; dann kontrollierter realer Eingießversuch mit standardisiertem Milchschaum; optional geführter, motorischer Tassenwinkel. | Erkennbares und wiederholbares reales Herz unter standardisierten Bedingungen |

**Wichtige Trennung für wissenschaftlich saubere Tests:** Die Positionen, die MuJoCo intern exakt kennt, dürfen zur **Validierung** dienen, **nicht** als heimliche Eingabedaten an unsere Kamerawahrnehmung. Für die erste 3D-Pose dürfen wir sichtbare ArUco-/AprilTag-Marker und bekannte Objektgeometrien verwenden; markerfreie Erkennung kann später kommen. Ohne Kalibrierung und Sichtbarkeit aus mehreren Winkeln sind 3D-Winkel nicht zuverlässig rekonstruierbar.

**Demonstrationsquelle ist Pflicht:** Kameras messen nur Bewegungen, die im Simulator vorgemacht/abgespielt werden. Wir benötigen in Stufe 2 eine virtuelle Teleoperation oder vorhandene zeitabhängige Demo (nicht automatisch einen perfekt gießenden Barista). In Stufe 3 nutzen wir erst dann eine fachlich gute **Heart-Pour**-Demonstration. Die One-Shot-Publikation nimmt echte Kännchenpose mit externen Vive-Trackern auf; wir testen als bewusste Abweichung, ob **drei Kameras** vergleichbare Posedaten liefern.

**Umfangsgrenzen:** Keine eigene Flüssigkeitssimulation. Keine kontinuierliche Korrektur anhand entstehender Latte Art im ersten MVP. Keine Espresso-/Siebträger-/Mühlen-/Milchaufschäum-Automation. Kein Zwang zu einem zweiten Arm oder zwei unabhängig bewegten Greifern. Ein fester Tassenwinkel ist vorerst eine Testvereinfachung – **nicht** identisch zur aktiv geneigten Tasse der One-Shot-Forschung.

## Technischer Baukasten – zunächst frei verfügbar

- **Robotik und Physik:** FAIRINO FR5 mit lizenzierter/kompatibler URDF/MJCF-Roboterbeschreibung, MuJoCo und ggf. ROS 2/MoveIt 2 (Verfügbarkeit und Modellintegration zunächst überprüfen).
- **Bildverarbeitung / 3D:** drei RGB-Ansichten, OpenCV-Kamerakalibrierung, ArUco bzw. AprilTag, PnP/Stereo-Triangulation. SAM2 ist nur dann nötig, wenn markerbasierte Erkennung nicht genügt.
- **Visualisierung:** Rerun für synchronisierte Kamera-Overlays, Konfidenzen, rekonstruierte 3D-Posen, echte Simulationspose und Trajektorien.
- **Lernen aus Demonstrationen:** [movement_primitives](https://github.com/dfki-ric/movement_primitives) für zeitabhängige DMP/CDMP; [One-Shot Learning for Double-Shot Latte Art](https://arxiv.org/abs/2610.16179) als fachliche Vergleichsreferenz, Originalcode noch nicht als frei verfügbar bestätigt.
- **Sicherer späterer Echtbetrieb:** FR5-Herstellersteuerung, unabhängig von unserem eigenen Datenformat; ROS-Schnittstellen oder Hersteller-SDK; Robotersicherheitskonzept vor echten Versuchen.

## Präzisierung Motion Capture und Testziele (20.10.2026)

- **Festgelegt: Option 2.** Ein menschlicher Barista bewegt das Milchkännchen; optische Marker **starr am Kännchen**, nicht bloß daneben, dienen als 6D-Bewegungsreferenz. Kein Handschuh-Tracking als Hauptmethode und kein Handführen des FR5 erforderlich. Im Simulations-MVP zunächst reproduzierbare virtuelle Kannenbewegungen.
- **Versuchsaufbau:** zwei seitliche, entgegengesetzt ausgerichtete Markerplatten an einem kalibrierten Kannenhalter, nicht auf Griff, Ausgießer, Greifflächen oder Milchkontaktbereichen. Drei Kameras werden nach Sichtlinie, Baseline, Markergröße in Pixeln, Schrägblick und Verdeckung positioniert; rechnerisch bekannte Extrinsik im Simulator, in realer Anlage separat zu kalibrieren.
- **Entscheidungsgrößen:** 0,5 mm maximaler Ausgießer-TCP-Positionsfehler und 1° maximaler Winkel als ambitioniertes **Forschungsziel**, 0,2 mm Stretch; Messrate und dynamische Robustheit gesondert. Für ~10 cm Abstand führt 1° zu etwa 1,75 mm Querabweichung am Ausgießer, daher beide Grenzen nicht als voneinander unabhängig interpretieren.
- **A/B-Test:** bisherigen Overhead-Marker-Prototyp unverändert als Baseline behalten; separater Side-Tag-Renderbenchmark mit eigener Ausgabedatei. Simulationsergebnis darf keinesfalls mit belegter Genauigkeit echter RGB-D-Kameras verwechselt werden. Die Handgelenk-Pose wird für Ausführung per FR5-Kinematik bestimmt, nicht als Pflichtmarkertest.
- **Vor Hardware:** Unterschiede zwischen reinen ArUco-Tafeln und AprilTags testen; feste Halterung bevorzugen, QR-Muster selbst nicht schweißen; Reflexion, Vibration, Temperatur, Griffkontakt, Kamerasync und Montageoffset prüfen.

## Abnahmekriterien (ohne erfundene Genauigkeitsversprechen)

**Stufe 1:** Bei mehreren Testposen werden Positions- und Winkelfehler pro Objekt und Kamera, Detektionsausfälle sowie erfolgreiche Multiview-Fusion gemessen. **Zielwerte erst nach Baseline-Messung** festlegen; Latte Art braucht möglicherweise höhere Genauigkeit als die erste Szene.

**Stufe 2:** Bewegungsfehler über die Zeit messen, nicht nur Start/Ende: TCP-Spitze, Kännchen-Quaternion, Geschwindigkeit, Dauer, Kollisionsfreiheit, Ausführbarkeit und wiederholte Versuche. Immer mit gleich ausgerichtetem Tassenkoordinatensystem.

**Stufe 3:** Echte Herzform mit Video/Bildvergleich dokumentieren; Tassenneigung und standardisierte Ausgangsbedingungen berücksichtigen. Nicht aus fehlender Flüssigkeit in MuJoCo auf Latte-Art-Erfolg schließen.

## Nächste konkrete Reihenfolge

**RB-23 → RB-24 → RB-25 → RB-14 → RB-47 → RB-15 → RB-07 → RB-08 → RB-16 → RB-48 → RB-09.**  
Dann **RB-49 → RB-18 → RB-19 → RB-20 → RB-44 → RB-50 → RB-26 → RB-51 → RB-36**.  
Erst danach **RB-42 → RB-29 → RB-37**. RB-43 (externe 6D-Tracker), RB-45 (motorisierte Tasse), RB-33/34 (Online-Latte-Art-Feedback), RB-38/39/40 (Espresso-/Milchworkflow) bleiben optional oder später.

## Thematische Phasen

Die älteren Kennungen `M0–M7` sind **thematische Kategorien, keine verpflichtende zeitliche Reihenfolge**. Maßgeblich ist die Spalte **Stufe** in der Aufgabenliste.

| Code | Themengebiet |
|---|---|
| M0 | Grundlagen und Referenzprojekte |
| M1 | Drei Kameraansichten und Bild-Tracking |
| M2 | Kalibrierung und überprüfte 3D-Wahrnehmung |
| M3 | Bewegungsdemonstration und Imitationslernen |
| M4 | FR5-Visualisierung, Kinematik und Replay |
| M5 | Reale Eingießvorbereitung ohne eigene Fluidphysik |
| M6 | Geschwindigkeit und spätere adaptive Regelung |
| M7 | Validierung und späterer Gesamtworkflow |

## Aufgabenübersicht

| ID | Aufgabe | Stufe | Bereich | Priorität | Schwierigkeit | Status | Abhängigkeiten |
|---|---|---|---|---|---:|---|---|
| [RB-01](#rb-01) | MVP festlegen: FR5, 3 Kameras, Demonstration, Dry-Run, später Latte Art | Stufe 1 | M0 | P0 | 1/5 | Backlog | – |
| [RB-02](#rb-02) | Messbare Erfolgskriterien für Wahrnehmung, Latte Art und Robotik | Stufe 1 | M0 | P0 | 2/5 | Backlog | RB-01 |
| [RB-03](#rb-03) | Systemarchitektur und gemeinsame Datenformate planen | Stufe 1 | M0 | P0 | 2/5 | Backlog | RB-01 |
| [RB-04](#rb-04) | Verfügbarkeit vorhandener Latte-Art-Projekte und Lizenzen prüfen | Später/Optional | M0 | P1 | 1/5 | Backlog | RB-01 |
| [RB-05](#rb-05) | Latte-Art-Videos für Vergleich und Qualitätsbewertung sammeln | Stufe 3 | M1 | P1 | 1/5 | Backlog | RB-01 |
| [RB-06](#rb-06) | Virtuelle Demo und Kameravideos als Referenz mit Zeitstempeln definieren | Stufe 2 | M1 | P0 | 1/5 | Backlog | RB-47 |
| [RB-07](#rb-07) | Kännchen, Tasse und FR5 in allen drei Kameras erkennen und tracken | Stufe 1 | M1 | P0 | 2/5 | Backlog | RB-47 |
| [RB-08](#rb-08) | Ausgießerspitze, Tassenrand und Marker-Orientierung bildbasiert erfassen | Stufe 1 | M1 | P0 | 3/5 | Backlog | RB-07 |
| [RB-09](#rb-09) | Drei Kameras + wahrgenommene und wahre 3D-Pose gemeinsam visualisieren | Stufe 1 | M1 | P0 | 1/5 | Backlog | RB-16; RB-47 |
| [RB-10](#rb-10) | Kleinen annotierten Testdatensatz und Auswertung erstellen | Später/Optional | M1 | P2 | 2/5 | Backlog | RB-07 |
| [RB-11](#rb-11) | Milchstrahl und Auftreffpunkt auf der Kaffeeoberfläche erkennen | Später/Optional | M1 | P2 | 4/5 | Backlog | RB-08; RB-10 |
| [RB-12](#rb-12) | Schaumoberfläche, Füllstand und Latte-Art-Konturen analysieren | Stufe 3 | M1 | P1 | 4/5 | Backlog | RB-10 |
| [RB-13](#rb-13) | Qualitäts- und Unsicherheitstest der Kamerawahrnehmung | Stufe 1 | M1 | P0 | 3/5 | Backlog | RB-48 |
| [RB-14](#rb-14) | Drei virtuelle Kamerapositionen mit ausreichender Überlappung planen | Stufe 1 | M2 | P0 | 2/5 | Backlog | RB-25 |
| [RB-15](#rb-15) | Drei virtuelle Kameras kalibrieren und Weltkoordinaten verknüpfen | Stufe 1 | M2 | P0 | 3/5 | Backlog | RB-47 |
| [RB-16](#rb-16) | 6D-Position und Orientierung von Kännchen/Ausgießer und Tasse aus Kameras rekonstruieren | Stufe 1 | M2 | P0 | 4/5 | Backlog | RB-08; RB-15 |
| [RB-17](#rb-17) | Kännchen- und Becherwinkel, relative Höhe und TCP-Lage bestimmen | Stufe 1 | M2 | P0 | 3/5 | Backlog | RB-16 |
| [RB-18](#rb-18) | Zeitgestempelte 6D-Demonstration aus drei simulierten Kameras rekonstruieren | Stufe 2 | M3 | P0 | 3/5 | Backlog | RB-49; RB-16 |
| [RB-19](#rb-19) | Geschwindigkeit, Kännchenneigung und Bewegungsphasen aus Kameraposen extrahieren | Stufe 2 | M3 | P0 | 3/5 | Backlog | RB-17; RB-18 |
| [RB-20](#rb-20) | Kamerabasierte virtuelle Demonstration phasenweise als DMP lernen | Stufe 2 | M3 | P0 | 3/5 | Backlog | RB-18; RB-19 |
| [RB-21](#rb-21) | Gelernte Bewegung auf neue Startpositionen und Tempi übertragen | Stufe 2 | M3 | P1 | 4/5 | Backlog | RB-20 |
| [RB-22](#rb-22) | Alternativen zu DMPs vergleichbar testen | Später/Optional | M3 | P2 | 4/5 | Backlog | RB-20 |
| [RB-23](#rb-23) | FAIRINO FR5 als geplanten Roboter wählen und Simulationsmodell prüfen | Stufe 1 | M4 | P0 | 2/5 | Backlog | RB-01 |
| [RB-24](#rb-24) | Virtuelle Tasse mit bekanntem festen Neigungswinkel platzieren | Stufe 1 | M4 | P0 | 2/5 | Backlog | RB-23 |
| [RB-25](#rb-25) | Leichte MuJoCo-Szene mit FR5, Milchkännchen und geneigter Tasse starten | Stufe 1 | M4 | P0 | 3/5 | Backlog | RB-23; RB-24 |
| [RB-26](#rb-26) | DMP-Trajektorie mit FR5-IK und kalibriertem Ausgießer-TCP abfahren | Stufe 2 | M4 | P0 | 4/5 | Backlog | RB-20; RB-50; RB-25 |
| [RB-27](#rb-27) | Kollisionen, Reichweite, Tassen- und Griffgeometrie prüfen | Stufe 2 | M4 | P0 | 3/5 | Backlog | RB-26 |
| [RB-28](#rb-28) | Geometrischen Auftreffpunkt ohne Flüssigkeitssimulation prüfen | Stufe 2 | M5 | P1 | 2/5 | Backlog | RB-17; RB-25 |
| [RB-29](#rb-29) | Ersten Versuch mit echter Milch unter konstanten Bedingungen planen | Stufe 3 | M5 | P1 | 3/5 | Backlog | RB-51; RB-28 |
| [RB-30](#rb-30) | Optionalen Flusssensor nur bei Bedarf evaluieren – keine eigene Fluidphysik | Später/Optional | M5 | P2 | 2/5 | Backlog | RB-29 |
| [RB-31](#rb-31) | Geschwindigkeitsprofil für KI, Kameras, Rendering und Physik erstellen | Stufe 2 | M6 | P1 | 2/5 | Backlog | RB-47; RB-03 |
| [RB-32](#rb-32) | Adaptive geschlossene Regelung anhand visueller Positionsfehler | Später/Optional | M6 | P2 | 4/5 | Backlog | RB-26; RB-15 |
| [RB-33](#rb-33) | Schnelle wiederholte Replanung und Sensorkontrolle | Später/Optional | M6 | P2 | 4/5 | Backlog | RB-32 |
| [RB-34](#rb-34) | Visuelle Latte-Art-Rückmeldung zu Bewegungskorrekturen nutzen | Später/Optional | M6 | P2 | 4/5 | Backlog | RB-11; RB-12; RB-29; RB-32 |
| [RB-35](#rb-35) | Robustheit gegen Startpositions- und Kameraveränderungen testen | Später/Optional | M7 | P2 | 4/5 | Backlog | RB-48; RB-51 |
| [RB-36](#rb-36) | Vollständiger FR5-Dry-Run mit 3D-Kamera-Tracking und aufgezeichneter Bewegung | Stufe 2 | M7 | P0 | 4/5 | Backlog | RB-51; RB-27 |
| [RB-37](#rb-37) | Ersten kontrollierten realen Eingießversuch durchführen | Stufe 3 | M7 | P1 | 4/5 | Backlog | RB-36; RB-29 |
| [RB-38](#rb-38) | Milch aufschäumen und Schaumqualität bewerten (späterer Teil) | Später/Optional | M7 | P2 | 5/5 | Backlog | RB-36 |
| [RB-39](#rb-39) | Espressomaschine und Mühle bedienen (späterer Teil) | Später/Optional | M7 | P2 | 4/5 | Backlog | RB-36 |
| [RB-40](#rb-40) | Dampflanze reinigen, Fehler erkennen und Arbeitsabläufe koordinieren | Später/Optional | M7 | P2 | 3/5 | Backlog | RB-38; RB-39 |
| [RB-41](#rb-41) | Ergebnisse, Modelle, Videos und Tests reproduzierbar archivieren | Später/Optional | M7 | P1 | 2/5 | Backlog | RB-03; RB-10 |
| [RB-42](#rb-42) | One-Shot-Paper: 6D-Vive-Tracking, 100-Hz-Becherencoder und frei verfügbaren Code verifizieren | Stufe 3 | M0 | P1 | 2/5 | Backlog | RB-01 |
| [RB-43](#rb-43) | OPTIONAL: Externe 6D-Tracker bei unzureichender Kamera-Pose nutzen | Später/Optional | M0 | P2 | 3/5 | Backlog | RB-48 |
| [RB-44](#rb-44) | Gelernte Kamera-Bewegung ohne Online-Korrektur reproduzieren | Stufe 2 | M3 | P0 | 2/5 | Backlog | RB-20 |
| [RB-45](#rb-45) | Motorisierten kippbaren Becherhalter nur bei Bedarf ergänzen | Später/Optional | M4 | P2 | 3/5 | Backlog | RB-24; RB-17; RB-44 |
| [RB-46](#rb-46) | Qualitätsprüfung zwischen Versuchen automatisieren | Stufe 3 | M7 | P1 | 2/5 | Backlog | RB-12; RB-37 |
| [RB-47](#rb-47) | Drei kalibrierbare virtuelle Kameras mit synchronen RGB-Frames einrichten | Stufe 1 | M1 | P0 | 2/5 | Backlog | RB-14; RB-25 |
| [RB-48](#rb-48) | Kamera-3D-Posen gegen die wahre MuJoCo-Pose quantitativ validieren | Stufe 1 | M2 | P0 | 3/5 | Backlog | RB-16; RB-15 |
| [RB-49](#rb-49) | Demonstration einer virtuellen Kännchenbewegung aufnehmen | Stufe 2 | M3 | P0 | 3/5 | Backlog | RB-47; RB-25 |
| [RB-50](#rb-50) | Ausgießer-/Kännchen-Pose in FR5-TCP und erreichbare Robotik-Ziele umrechnen | Stufe 2 | M4 | P0 | 3/5 | Backlog | RB-17; RB-23; RB-18 |
| [RB-51](#rb-51) | Kamera-gelerntes Bewegungsreplay in der Simulation messen | Stufe 2 | M4 | P0 | 3/5 | Backlog | RB-26; RB-48 |

## Aufgabendetails

### M0 – Grundlagen und Referenzprojekte

<a id="rb-01"></a>
#### RB-01 – MVP festlegen: FR5, 3 Kameras, Demonstration, Dry-Run, später Latte Art

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** keine
- **Freie Tools / Projekte (Kandidaten):** Markdown / GitHub
- [ ] **Abnahme:** Expliziter Plan in drei Stufen: 1) drei simulierte RGB-Kameras und daraus sichtbare/prüfbare 3D-Pose von FR5, Kännchen/Ausgießer und Tasse; 2) gemessene virtuelle Demonstration zeitlich nachahmen, ohne Live-Regelung; 3) reales Latte-Art-Eingießen. Keine eigene Flüssigkeitsphysik, keine Espresso-/Siebträger-/Aufschäumaufgaben.
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-02"></a>
#### RB-02 – Messbare Erfolgskriterien für Wahrnehmung, Latte Art und Robotik

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Freie Tools / Projekte (Kandidaten):** OpenCV
- [ ] **Abnahme:** Kamera-Posefehler in cm/mm und Grad gegen unabhängige MuJoCo-Ground-Truth, Tracking-Ausfälle und Sichtbarkeit, zeitlicher Trajektorien- und TCP-Wiedergabefehler, später Qualität echter Heart-Pours definiert
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-03"></a>
#### RB-03 – Systemarchitektur und gemeinsame Datenformate planen

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Freie Tools / Projekte (Kandidaten):** ROS 2 / Python / Rerun
- [ ] **Abnahme:** Herstellerunabhängige Schnittstellen: je 3 kalibrierte Kameraframes + Zeitstempel → Marker/Objekterkennung → 6D-Pose im Tassenkoordinatensystem + Unsicherheit → aufgezeichnete Demonstration → DMP/Playback → IK/FR5-Aktionen. Simulation liefert Ground Truth ausschließlich zur Evaluation
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-04"></a>
#### RB-04 – Verfügbarkeit vorhandener Latte-Art-Projekte und Lizenzen prüfen

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Freie Tools / Projekte (Kandidaten):** One-Shot Latte Art arXiv:2610.16179 / CatalyzeX / ridxm latte-art-robot
- [ ] **Abnahme:** FR5-Simulationsmodelle, SDK/MoveIt2/MuJoCo-Kompatibilität, OpenCV-Pose/Bildgebung, freie DMP-Bibliothek und One-Shot/π0-Referenzen samt Versionen/Lizenzen überprüft
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-42"></a>
#### RB-42 – One-Shot-Paper: 6D-Vive-Tracking, 100-Hz-Becherencoder und frei verfügbaren Code verifizieren

- **Stufe:** Stufe 3 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Freie Tools / Projekte (Kandidaten):** One-Shot arXiv:2610.16179 / CatalyzeX / dfki-ric/movement_primitives
- [ ] **Abnahme:** One-Shot-Arbeit auf Daten, echte Pose/Becherneigung und zugänglichen Code prüfen; öffentlich verfügbare 3-Kamera-Demos können alternativ verwendet werden; keine Hardwarebeschaffung als Voraussetzung
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-43"></a>
#### RB-43 – OPTIONAL: Externe 6D-Tracker bei unzureichender Kamera-Pose nutzen

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-48](#rb-48)
- **Freie Tools / Projekte (Kandidaten):** HTC Vive/SteamVR / OpenVR / geeignete Tracker / Winkelencoder
- [ ] **Abnahme:** Nur wenn Drei-Kamera-Tracking quantitativ unzureichend: 6D-Tracker als Zusatz oder Ground-Truth für reale Demonstration prüfen
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren


### M1 – Drei Kameraansichten und Bild-Tracking

<a id="rb-05"></a>
#### RB-05 – Latte-Art-Videos für Vergleich und Qualitätsbewertung sammeln

- **Stufe:** Stufe 3 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Freie Tools / Projekte (Kandidaten):** One-Shot-Latte-Art / ridxm latte-pour-demos / ffmpeg
- [ ] **Abnahme:** Mindestens ein rechtmäßig verwendbares Video mit bekannter Kameraperspektive als visuelle Referenz; Bewegung muss für MVP nicht allein aus Video rekonstruiert werden
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-06"></a>
#### RB-06 – Virtuelle Demo und Kameravideos als Referenz mit Zeitstempeln definieren

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** [RB-47](#rb-47)
- **Freie Tools / Projekte (Kandidaten):** OpenCV / ffmpeg
- [ ] **Abnahme:** Gleichzeitig drei Kamera-Videos/Framefolgen, Kamera-Posen, Zeitstempel und virtuelle Demonstration im gleichen Testlauf verfügbar
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-07"></a>
#### RB-07 – Kännchen, Tasse und FR5 in allen drei Kameras erkennen und tracken

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-47](#rb-47)
- **Freie Tools / Projekte (Kandidaten):** OpenCV ArUco/AprilTag / bekannte Objektgeometrie / SAM2 optional
- [ ] **Abnahme:** Kännchen, Tasse und Greifer/Arm mit klarer ID in drei simulierten Ansichten markiert, kontinuierlich verfolgt und als maschinenlesbare Keypoints/Detektionen gespeichert
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-08"></a>
#### RB-08 – Ausgießerspitze, Tassenrand und Marker-Orientierung bildbasiert erfassen

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-07](#rb-07)
- **Freie Tools / Projekte (Kandidaten):** OpenCV ArUco / PnP / Keypoints / bekannte Kännchengeometrie
- [ ] **Abnahme:** Pixelpositionen relevanter Referenzpunkte und Orientierungshinweise pro Ansicht mit Detektionskonfidenz verfügbar; sichtbare Marker sind für Version 1 zulässig
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-09"></a>
#### RB-09 – Drei Kameras + wahrgenommene und wahre 3D-Pose gemeinsam visualisieren

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** [RB-16](#rb-16), [RB-47](#rb-47)
- **Freie Tools / Projekte (Kandidaten):** Rerun / OpenCV / MuJoCo Viewer
- [ ] **Abnahme:** Drei Echtzeit- bzw. aufgezeichnete Bildansichten, Erkennungs-Overlays, rekonstruierte 3D-Objekt-/TCP-Pose, Unsicherheit und unabhängige MuJoCo-Ground-Truth nebeneinander sichtbar
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-10"></a>
#### RB-10 – Kleinen annotierten Testdatensatz und Auswertung erstellen

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-07](#rb-07)
- **Freie Tools / Projekte (Kandidaten):** CVAT / Label Studio
- [ ] **Abnahme:** Kleiner gelabelter Datensatz nur falls OpenCV/Marker für die optische Pose unzureichend sind; zunächst keine selbsttrainierte Objekterkennung erforderlich
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-11"></a>
#### RB-11 – Milchstrahl und Auftreffpunkt auf der Kaffeeoberfläche erkennen

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-08](#rb-08), [RB-10](#rb-10)
- **Freie Tools / Projekte (Kandidaten):** OpenCV / SAM 2 / eigenes kleines Segmentierungsmodell
- [ ] **Abnahme:** Auftreffpunkt/Strahl aus Kamerabild nur bei Bedarf für spätere Musterkorrektur; kein Muss für erste Reproduktion
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-12"></a>
#### RB-12 – Schaumoberfläche, Füllstand und Latte-Art-Konturen analysieren

- **Stufe:** Stufe 3 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-10](#rb-10)
- **Freie Tools / Projekte (Kandidaten):** OpenCV / Bildsegmentierung
- [ ] **Abnahme:** Nach realen Tests Herzmuster fotografieren und Form, Zentrierung und Reproduzierbarkeit vergleichen, zunächst manuell oder mit OpenCV
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-13"></a>
#### RB-13 – Qualitäts- und Unsicherheitstest der Kamerawahrnehmung

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-48](#rb-48)
- **Freie Tools / Projekte (Kandidaten):** OpenCV / eigene Tests
- [ ] **Abnahme:** Posefehler und Trackingausfälle für Verdeckungen, ungünstige Winkel, Reflexionen und unterschiedliche Kamerapositionen erkannt und visualisiert; keine stillen Schätzungen ohne ausreichende Sicht
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-47"></a>
#### RB-47 – Drei kalibrierbare virtuelle Kameras mit synchronen RGB-Frames einrichten

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-14](#rb-14), [RB-25](#rb-25)
- **Freie Tools / Projekte (Kandidaten):** MuJoCo Cameras / OpenCV / NumPy
- [ ] **Abnahme:** Drei benannte Perspektiven liefern gleichzeitig synchronisierte RGB-Frames mit dokumentierter intrinsischer/extrinsischer Kalibrierung und gemeinsam sichtbarer Ausgießer-/Tassenregion
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren


### M2 – Kalibrierung und überprüfte 3D-Wahrnehmung

<a id="rb-14"></a>
#### RB-14 – Drei virtuelle Kamerapositionen mit ausreichender Überlappung planen

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-25](#rb-25)
- **Freie Tools / Projekte (Kandidaten):** MuJoCo Kameras / OpenCV Kalibrierung / Stereo-Geometrie
- [ ] **Abnahme:** Drei virtuelle RGB-Kameras so platziert, dass Tasse, Kännchen/Ausgießer und Roboterwerkzeug möglichst aus mehreren Ansichten sichtbar sind; Sichtfeld, Überlappung und Verdeckungen dokumentiert
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-15"></a>
#### RB-15 – Drei virtuelle Kameras kalibrieren und Weltkoordinaten verknüpfen

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-47](#rb-47)
- **Freie Tools / Projekte (Kandidaten):** OpenCV calibrateCamera / ArUco / solvePnP / Triangulation
- [ ] **Abnahme:** Intrinsische/extrinsische Kalibrierung, Kamera-zu-Welt/FR5/Tasse-Transformationen und Messfehler reproduzierbar dokumentiert
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-16"></a>
#### RB-16 – 6D-Position und Orientierung von Kännchen/Ausgießer und Tasse aus Kameras rekonstruieren

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-08](#rb-08), [RB-15](#rb-15)
- **Freie Tools / Projekte (Kandidaten):** OpenCV triangulation / PnP / bekannte 3D-Objektmodelle / Marker
- [ ] **Abnahme:** Posen x,y,z und Orientierung (z. B. Quaternion) aus Kameraansichten berechnet, nicht aus MuJoCo-Ground-Truth übernommen; relative Ausgießer-Tassen-Pose samt Unsicherheit gesondert berechnet
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-17"></a>
#### RB-17 – Kännchen- und Becherwinkel, relative Höhe und TCP-Lage bestimmen

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-16](#rb-16)
- **Freie Tools / Projekte (Kandidaten):** OpenCV / Geometrie
- [ ] **Abnahme:** Kännchenneigung, feste Tassenneigung, Ausgießerhöhe und relative Orientierung aus drei Kameras sowie kalibriertem Werkzeug-Offset zeitlich verfolgt; echte Tassenkinematik erst später
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-48"></a>
#### RB-48 – Kamera-3D-Posen gegen die wahre MuJoCo-Pose quantitativ validieren

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-16](#rb-16), [RB-15](#rb-15)
- **Freie Tools / Projekte (Kandidaten):** MuJoCo ground truth / OpenCV / Rerun / pytest
- [ ] **Abnahme:** Separat berechnete Kamera-Pose und MuJoCo-Wahrheit bei mehreren Positionen verglichen; Lage- und Winkelfehler plus Verdeckungsrate sichtbar und Zielgenauigkeit festgelegt
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren


### M3 – Bewegungsdemonstration und Imitationslernen

<a id="rb-18"></a>
#### RB-18 – Zeitgestempelte 6D-Demonstration aus drei simulierten Kameras rekonstruieren

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-49](#rb-49), [RB-16](#rb-16)
- **Freie Tools / Projekte (Kandidaten):** OpenCV PnP/Triangulation / NumPy / Rerun
- [ ] **Abnahme:** Eine geführte virtuelle Kännchenbewegung als x,y,z/Quaternion über die Zeit ausschließlich aus den drei Kameraaufnahmen abgeleitet und gegen Ground Truth verglichen
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-19"></a>
#### RB-19 – Geschwindigkeit, Kännchenneigung und Bewegungsphasen aus Kameraposen extrahieren

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-17](#rb-17), [RB-18](#rb-18)
- **Freie Tools / Projekte (Kandidaten):** SciPy / NumPy
- [ ] **Abnahme:** Aus dem 6D-Kameratracking mit Zeitstempeln Bewegungsbahn, Winkel, Geschwindigkeit, Beschleunigung und Phaseneinteilung abgeleitet und geglättet
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-20"></a>
#### RB-20 – Kamerabasierte virtuelle Demonstration phasenweise als DMP lernen

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-18](#rb-18), [RB-19](#rb-19)
- **Freie Tools / Projekte (Kandidaten):** One-Shot Latte Art arXiv:2610.16179 / dfki-ric/movement_primitives
- [ ] **Abnahme:** Mit frei verfügbarer CDMP/DMP-Bibliothek eine konkrete virtuelle Demonstration gelernt; daraus werden zeitlich konsistente Ausgießerposen vorhergesagt; keine manuelle Robotergelenk-Trajektorie
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-21"></a>
#### RB-21 – Gelernte Bewegung auf neue Startpositionen und Tempi übertragen

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-20](#rb-20)
- **Freie Tools / Projekte (Kandidaten):** Movement Primitives
- [ ] **Abnahme:** Nach erfolgreichem Replay die gelernte Trajektorie relativ zur Tasse verschieben; Veränderung vor Bewegung auswerten, kein Live-Replanning
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-22"></a>
#### RB-22 – Alternativen zu DMPs vergleichbar testen

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-20](#rb-20)
- **Freie Tools / Projekte (Kandidaten):** LeRobot / Diffusion Policy / ACT
- [ ] **Abnahme:** Externe 6D-Motion-Capture-Tracker, PI0, normales Video-Lernen erst prüfen, wenn das Drei-Kamera-System oder DMP-Replay nicht ausreicht
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-44"></a>
#### RB-44 – Gelernte Kamera-Bewegung ohne Online-Korrektur reproduzieren

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-20](#rb-20)
- **Freie Tools / Projekte (Kandidaten):** dfki-ric/movement_primitives / Rerun / NumPy
- [ ] **Abnahme:** Zeit- und orientierungstreue virtuelle Kännchen-Trajektorie aus Kamerademonstration reproduzierbar; noch ohne Roboterregelung aus neuen Bildern
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-49"></a>
#### RB-49 – Demonstration einer virtuellen Kännchenbewegung aufnehmen

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-47](#rb-47), [RB-25](#rb-25)
- **Freie Tools / Projekte (Kandidaten):** MuJoCo Viewer / Teleoperation / vorhandene Demo-Trajektorie / Rerun
- [ ] **Abnahme:** Mindestens eine zeitliche virtuelle Demo mit 3 Kamera-Videostreams, bekannter Referenzpose und festen Tassenwinkel synchron aufgezeichnet; Demo muss nicht bereits perfekte Latte Art sein
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren


### M4 – FR5-Visualisierung, Kinematik und Replay

<a id="rb-23"></a>
#### RB-23 – FAIRINO FR5 als geplanten Roboter wählen und Simulationsmodell prüfen

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Freie Tools / Projekte (Kandidaten):** FAIRINO FR5 URDF/ROS2 MoveIt2 / MuJoCo / Robotik-SDK
- [ ] **Abnahme:** FR5-Modellquelle, Lizenz, Toolflansch, 6D-Kinematik, Reichweite, Greifer-Montage, Simulator-Anbindung und doppelte (nacheinander genutzte) Werkzeuge geprüft; kein Kauf erforderlich
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-24"></a>
#### RB-24 – Virtuelle Tasse mit bekanntem festen Neigungswinkel platzieren

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-23](#rb-23)
- **Freie Tools / Projekte (Kandidaten):** MuJoCo / CAD optional
- [ ] **Abnahme:** Feste, variabel einstellbare Tassenpose im Simulator; Tasse muss nicht gleichzeitig mit dem Kännchen bewegt werden
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-25"></a>
#### RB-25 – Leichte MuJoCo-Szene mit FR5, Milchkännchen und geneigter Tasse starten

- **Stufe:** Stufe 1 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-23](#rb-23), [RB-24](#rb-24)
- **Freie Tools / Projekte (Kandidaten):** Offizielles FAIRINO Modell + MuJoCo / gegebenenfalls bestehender Robotersimulator
- [ ] **Abnahme:** Ohne Milch/Schaumsimulation rendert eine reproduzierbare Szene FR5, TCP, Kännchen und Tasse; bekannte Weltposen dienen ausschließlich als Evaluationsreferenz
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-26"></a>
#### RB-26 – DMP-Trajektorie mit FR5-IK und kalibriertem Ausgießer-TCP abfahren

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-20](#rb-20), [RB-50](#rb-50), [RB-25](#rb-25)
- **Freie Tools / Projekte (Kandidaten):** MoveIt2 / MuJoCo IK / FR5 SDK
- [ ] **Abnahme:** FR5 folgt zeitgestempelten Kamera-abgeleiteten Ausgießerpositionen und -orientierungen; Bahnfehler und Tooloffset überprüft, ohne permanente Bildkorrektur
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-27"></a>
#### RB-27 – Kollisionen, Reichweite, Tassen- und Griffgeometrie prüfen

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-26](#rb-26)
- **Freie Tools / Projekte (Kandidaten):** MoveIt 2 / MuJoCo
- [ ] **Abnahme:** Kollisionen, unerreichbare Zielposen, Gelenkgrenzen, Geschwindigkeitsgrenzen und Ausgießer-Tassen-Abstand beim FR5-Dry-Run erkannt; sichere Start-/Stop-Bedingungen
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-45"></a>
#### RB-45 – Motorisierten kippbaren Becherhalter nur bei Bedarf ergänzen

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-24](#rb-24), [RB-17](#rb-17), [RB-44](#rb-44)
- **Freie Tools / Projekte (Kandidaten):** NEMA 17 / AS5600 / ESP32 / bestehende Stepper-Libraries
- [ ] **Abnahme:** Nur nach echtem Pour-Versuch prüfen, ob feste Tassenneigung unzureichend ist; motorisierte Becherneigung als Erweiterung statt zweitem Arm
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-50"></a>
#### RB-50 – Ausgießer-/Kännchen-Pose in FR5-TCP und erreichbare Robotik-Ziele umrechnen

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-17](#rb-17), [RB-23](#rb-23), [RB-18](#rb-18)
- **Freie Tools / Projekte (Kandidaten):** MoveIt2 TF2 / MuJoCo IK / URDF
- [ ] **Abnahme:** Kalibrierter starrer Ausgießer-zu-Flansch-Transform, Basis-/Tassen-Koordinaten und Pose-Umrechnung verifiziert, damit die Kamera-beschriebene Bewegung mit dem FR5 ausführbar ist
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-51"></a>
#### RB-51 – Kamera-gelerntes Bewegungsreplay in der Simulation messen

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-26](#rb-26), [RB-48](#rb-48)
- **Freie Tools / Projekte (Kandidaten):** MuJoCo / Rerun / pytest
- [ ] **Abnahme:** Gleicher zeitabhängiger Ausgießerpfad bei wiederholten virtuellen FR5-Dry-Runs; Position, Winkel, Geschwindigkeit, Kollisionen und Timing gegen Demonstration geprüft
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren


### M5 – Reale Eingießvorbereitung ohne eigene Fluidphysik

<a id="rb-28"></a>
#### RB-28 – Geometrischen Auftreffpunkt ohne Flüssigkeitssimulation prüfen

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-17](#rb-17), [RB-25](#rb-25)
- **Freie Tools / Projekte (Kandidaten):** OpenCV / MuJoCo-Geometrie / Rerun
- [ ] **Abnahme:** Ausgießerposition, Becherwinkel und theoretischer Auftreffpunkt als Geometrie visualisiert; keine Flüssigkeitsphysik
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-29"></a>
#### RB-29 – Ersten Versuch mit echter Milch unter konstanten Bedingungen planen

- **Stufe:** Stufe 3 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-51](#rb-51), [RB-28](#rb-28)
- **Freie Tools / Projekte (Kandidaten):** Echte Espressotasse / Milchkännchen / reproduzierbare Milchschaumzubereitung / Video
- [ ] **Abnahme:** Erst nach sicherem FR5-Dry-Run mit zeitlich reproduzierter demonstrierter Bewegung realen Eingießversuch planen; Milchmenge/Temperatur/Schaum unter Standardbedingungen
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-30"></a>
#### RB-30 – Optionalen Flusssensor nur bei Bedarf evaluieren – keine eigene Fluidphysik

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-29](#rb-29)
- **Freie Tools / Projekte (Kandidaten):** Küchenwaage / Video / einfache Zeitdifferenz
- [ ] **Abnahme:** Nur bei ungleichen realen Ergebnissen entscheiden, ob Massenzunahme pro Zeit gemessen werden muss; keine Pflichtsensorik, kein selbstgebauter Simulator
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren


### M6 – Geschwindigkeit und spätere adaptive Regelung

<a id="rb-31"></a>
#### RB-31 – Geschwindigkeitsprofil für KI, Kameras, Rendering und Physik erstellen

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-47](#rb-47), [RB-03](#rb-03)
- **Freie Tools / Projekte (Kandidaten):** Python perf_counter / torch profiler / Rerun
- [ ] **Abnahme:** Kamera-Rendering, Poseerkennung pro Frame, 3D-Rekonstruktion und FR5-Traj.-Ausführung getrennt messen; für erste Wiedergabe sind keine Live-Bildkorrekturen nötig
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-32"></a>
#### RB-32 – Adaptive geschlossene Regelung anhand visueller Positionsfehler

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-26](#rb-26), [RB-15](#rb-15)
- **Freie Tools / Projekte (Kandidaten):** MoveIt Servo / ROS 2
- [ ] **Abnahme:** Optionale Neuverankerung der bereits gelernten Bahn bei verschobener Tasse *vor* dem Gießen; keine verpflichtende Echtzeit-Nachregelung
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-33"></a>
#### RB-33 – Schnelle wiederholte Replanung und Sensorkontrolle

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-32](#rb-32)
- **Freie Tools / Projekte (Kandidaten):** ROS 2 / asynchrones Policy-Serving
- [ ] **Abnahme:** Schnelle Live-Nachregelung nur wenn Offline-Anpassung, Video-Erfolgskontrolle und feste Demonstrationsbahn noch nicht genügen
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-34"></a>
#### RB-34 – Visuelle Latte-Art-Rückmeldung zu Bewegungskorrekturen nutzen

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-11](#rb-11), [RB-12](#rb-12), [RB-29](#rb-29), [RB-32](#rb-32)
- **Freie Tools / Projekte (Kandidaten):** OpenCV / Rerun / MoveIt Servo
- [ ] **Abnahme:** Visuelle Anpassung während des Eingießens erst beginnen, wenn offene Wiedergabe/Startpositionskorrektur/Qualitätskontrolle nicht ausreichen
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren


### M7 – Validierung und späterer Gesamtworkflow

<a id="rb-35"></a>
#### RB-35 – Robustheit gegen Startpositions- und Kameraveränderungen testen

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-48](#rb-48), [RB-51](#rb-51)
- **Freie Tools / Projekte (Kandidaten):** MuJoCo / pytest / Rerun
- [ ] **Abnahme:** Robustheit gegen Kameraverschiebung, Markerverdeckung und veränderte Startpositionen testen, zunächst ohne fortlaufende Anpassung
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-36"></a>
#### RB-36 – Vollständiger FR5-Dry-Run mit 3D-Kamera-Tracking und aufgezeichneter Bewegung

- **Stufe:** Stufe 2 · **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-51](#rb-51), [RB-27](#rb-27)
- **Freie Tools / Projekte (Kandidaten):** MuJoCo / Rerun / OpenCV
- [ ] **Abnahme:** Von drei Kameraaufnahmen über 6D-Pose/Bewegungslernen bis zur FR5-Ausführung reproduzierbar ohne Fluid-Simulation und ohne Online-Adaptation
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-37"></a>
#### RB-37 – Ersten kontrollierten realen Eingießversuch durchführen

- **Stufe:** Stufe 3 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-36](#rb-36), [RB-29](#rb-29)
- **Freie Tools / Projekte (Kandidaten):** Realer Roboter / Kamera / Sicherheitsprüfung
- [ ] **Abnahme:** Nach sicherem virtuellen FR5-Dry-Run realen Herz-Gießversuch dokumentiert; standardisierte Milch, Bildvergleich, Fehlersuche; Hardware erst nach Go/No-Go-Entscheidung
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-38"></a>
#### RB-38 – Milch aufschäumen und Schaumqualität bewerten (späterer Teil)

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 5/5
- **Abhängigkeiten:** [RB-36](#rb-36)
- **Freie Tools / Projekte (Kandidaten):** Computer Vision / Temperatur- und Drucksensorik
- [ ] **Abnahme:** Bewusst außerhalb des aktuellen MVP: Milchaufschäumen als späteres eigenes Subprojekt
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-39"></a>
#### RB-39 – Espressomaschine und Mühle bedienen (späterer Teil)

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-36](#rb-36)
- **Freie Tools / Projekte (Kandidaten):** LeRobot / robosuite / ROS 2
- [ ] **Abnahme:** Bewusst außerhalb des aktuellen MVP: Siebträger-/Mühlen-/Maschinenbedienung als spätere separate, nacheinander verwendete Greifwerkzeuge
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-40"></a>
#### RB-40 – Dampflanze reinigen, Fehler erkennen und Arbeitsabläufe koordinieren

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-38](#rb-38), [RB-39](#rb-39)
- **Freie Tools / Projekte (Kandidaten):** Zustandsautomat / ROS 2
- [ ] **Abnahme:** Bewusst außerhalb des aktuellen MVP: Dampflanzenreinigung und Gesamtworkflow erst nach Latte Art
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-41"></a>
#### RB-41 – Ergebnisse, Modelle, Videos und Tests reproduzierbar archivieren

- **Stufe:** Später/Optional · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-03](#rb-03), [RB-10](#rb-10)
- **Freie Tools / Projekte (Kandidaten):** GitHub / Hugging Face Datasets / DVC
- [ ] **Abnahme:** Dokumentierte Versionen, Posen/Bewegung, Video und reale Versuchsergebnisse; Milchzubereitung standardisiert, keine Pflichtsensoren
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

<a id="rb-46"></a>
#### RB-46 – Qualitätsprüfung zwischen Versuchen automatisieren

- **Stufe:** Stufe 3 · **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-12](#rb-12), [RB-37](#rb-37)
- **Freie Tools / Projekte (Kandidaten):** OpenCV / Rerun / Bildbewertung
- [ ] **Abnahme:** Latte-Art-Qualitätsvergleich nach dem Gießen möglich; kontinuierliche Mustersteuerung bewusst nicht im MVP
- [ ] Code, Video, Metriken, Probleme und Versionen dokumentieren

## Entscheidungen und offene Risiken

- **FAIRINO FR5 nur Planung, kein Kauf.** Modellverfügbarkeit, Simulation und Toolflansch vor Bindung verifizieren.
- **Drei Kameras sind fest eingeplant.** Gute 3D-Pose setzt passende Sichtfelder, Kalibrierung, referenzierte Objektgeometrie oder Marker und eine unabhängige Prüfung voraus.
- **Simulations-Wahrheit ist nicht visuelle Wahrnehmung.** Wir protokollieren beide getrennt.
- **Kamera-Aufnahmen erzeugen keine erfolgreiche Gießbewegung.** Es braucht eine gute Ausgangsdemonstration; die Latte-Art-Referenz kommt spätestens in Stufe 3.
- **Ein Roboterarm, nacheinander wechselnde/auswählbare Greifwerkzeuge.** Kein zweiter unabhängiger Arm im MVP.
- **Keine eigene CFD/Milchschaum-Physik und keine Live-Replanung im MVP.** Echtzeit erst relevant, sobald wir solche Funktionen ergänzen.
- **Tassenneigung:** für 3D und Imitation gemessene feste Pose, reale dynamische Neigung optional; One-Shot-Ergebnis basiert auf aktiv bewegter Tasse.
- **Originalcode:** One-Shot-CDMP-Code/Daten noch nicht öffentlich bestätigt; offene DMP-Alternativen einplanen.
- **Gesamtsystem:** Kaffee mahlen, Siebträger bedienen, Milch aufschäumen und Reinigen erst nach Latte-Art-Demonstration.

## Tracking

Die CSV-Datei [ROBOBARISTA_BACKLOG.csv](./ROBOBARISTA_BACKLOG.csv) enthält den langfristigen Aufgabenbestand. Die ungenutzte, aus derselben Liste abgeleitete frühere Linear-Importvorlage wurde beim Aufräumen gelöscht (in der GitHub-Historie weiterhin vorhanden). **Es wurden keine Linear-Issues angelegt.** GitHub Issues sind derzeit deaktiviert. Die verbindlichen nächsten Schritte stehen in [ROBOBARISTA_DREI_STUFEN_MVP.md](./ROBOBARISTA_DREI_STUFEN_MVP.md).