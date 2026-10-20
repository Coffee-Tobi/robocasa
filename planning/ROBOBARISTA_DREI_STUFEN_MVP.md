# RoboBarista – aktueller Projektplan

**Stand 21. Oktober 2026 · Entscheidung dokumentiert · nur virtuelle Simulation, noch keine Hardware.**

## Ziel

Ein virtueller **FAIRINO FR5** soll aus der **optisch erfassten 6D-Bewegung eines Milchkännchens** lernen und später eine präzise Latte-Art-Gießbewegung gegenüber einer Tasse nachfahren.

Einarmiger Ansatz. Zuerst einfache, visuell prüfbare Bewegungen **ohne simulierte Milchphysik**; später kontrollierter realer Gießversuch. Espresso-Mühle, Siebträgerbedienung, Aufschäumen und Reinigung bleiben spätere separate Ausbaustufen.

### Verbindliche Designentscheidungen

| Frage | Entscheidung für den nächsten MVP |
| --- | --- |
| Was wird aufgenommen? | **Bewegung des Milchkännchens direkt** (nicht Tracking der Hände und nicht manuelles Führen des Roboterarms) |
| Referenz am Kännchen | **1 starrer, seitlicher AprilTag 36h11**; Montage nicht am Griff / Ausgießer / an Milchkontaktflächen; bekannten Tag→Kännchen→Ausgießer-Transform kalibrieren |
| Kameras | **3 feststehende RGB-D-Kameras** aus unterschiedlichen Winkeln auf der sichtbaren Tagseite; ausreichend gemeinsames Sichtfeld; mindestens 2 verwertbare Kameras je Frame |
| Erkennung | **Offizieller AprilTag-3-Detektor** über `pupil-apriltags` |
| 6D-Pose | **Eigene SciPy-basierte gemeinsame Mehrkamera-Optimierung** der Tag-Bildecken, virtuelle Tiefendaten als weiche Nebenbedingung |
| Robotermodell | FAIRINO FR5 aus `ma-weiss/Fairino-Mujoco` in MuJoCo; später Greifer-/Werkzeug- und Gelenkkinematik |
| Genaugkeitsziel | vorläufig max. **0,5 mm absolute Ausgießerspitzenabweichung** und **1° Drehfehler**, **0,2 mm** als Stretch; später präzise **Spitze relativ zur Tasse** |
| Alternativen | Zweiter Tag, vierte Kamera und Caliscope **behalten wir als Forschungs-/Fallbackvarianten**, nicht als verpflichtenden MVP |
| Umgebung | GitHub: Code und Ergebnisse; Google Colab CPU/OSMesa: Simulation, Prüfung, Kamerabilder |

**Warum gerade dieser Aufbau?** Aus 16 in Colab ausgeführten Vergleichsvarianten war die **Ein-Tag-/Drei-Kamera-Variante mit Pupil und eigener Optimierung** die einfachste mit `pitcher_status=PASS` und `scene_status=PASS`. Drei statische Testposen ergaben schlechtestenfalls **0,13777915758536008 mm** absolute Spitzenabweichung und **0,045832767365699856°** Winkelfehler. Siehe [vollständiges Versuchsprotokoll](../results/2026-10-21_camera_rig_decision.md) und [16-Werte-CSV](../results/2026-10-21_rig_comparison_user_report.csv).

**Unbedingt beachten:** Dieses Ergebnis gilt nur für drei idealisierte **statische** Posen (0°, 12°, 25°). MuJoCo kennt die Kameraparameter exakt und verwendet vereinfachte Kännchen-/Ausgießergeometrie. Kein Nachweis einer 0,2–0,5-mm-Genauigkeit im realen Betrieb oder über vollständige Bewegungsbahnen.

## Stufe 1 – Kamera-Motion-Capture zuverlässig machen

**Status:** erster statischer Sicht-/Positions-Benchmark bestanden; dynamisches Tracking steht aus.

**Ausgangspunkt:** [COLAB_START.ipynb](../COLAB_START.ipynb). Es verwendet jetzt standardmäßig genau den gewählten 1-Tag-/3-Kamera-/Pupil-Aufbau. [COLAB_COMPARE.ipynb](../COLAB_COMPARE.ipynb) bewahrt den Vergleich von 16 Alternativen.

**Nächste Arbeitssitzung:**

1. **Position und Neigung systematisch variieren:** Viele weitere Positionen und Kannenwinkel (einschließlich realistischer Schwenks/Rotationen), mehrere unabhängige Seeds. Marker muss in jeder wichtigen Bewegungssituation von mindestens zwei Kameras erkannt werden.
2. **Reale Verdeckungssituationen simulieren:** FR5-Greifer, Handgelenk und Griff in den relevanten Blickrichtungen. Bei zu häufigen Ausfällen erst Kameraausrichtung optimieren, erst dann als Backup zweiten Tag/weitere Kamera in Erwägung ziehen.
3. **Dynamische Demonstration aufzeichnen:** Zeitgestempelte RGB-D-Frames, Tag-Erkennungen, berechnete Kännchenpose (Position + Quaternion) und Ausgießerspitze in ein maschinenlesbares Trajektorienformat schreiben. MuJoCo-Ground-Truth nur zum anschließenden unabhängigen Fehlervergleich verwenden.
4. **Relatives Bezugssystem entwickeln:** Tasse vor der Demonstration kalibrieren und Ausgießerspitze **relativ zur Tasse** messen. Separate Grenzwerte für absolute Spitze, relative Spitze, Winkelfehler, zeitliche Glätte und Erkennungsquote.
5. **Robustheit:** Kamerarauschen, Bildunschärfe, Tiefenrauschen, Offsetfehler/Markerbefestigung, Extrinsikfehler und Zeitsynchronisation getrennt testen; nicht nur die bisher idealen Werte.

**Abnahme vor Stufe 2:** reproduzierbare zusammenhängende Trajektorie, brauchbare Abdeckung relevanter Posen, alle Ausfälle offengelegt, kontrollierbare Spitzen-/Winkelfehler, realistische Bildstörungen und bekannte Messunsicherheit. Ein einzelner guter Frame reicht nicht.

## Stufe 2 – Demonstration mit FR5 nachfahren

**Status:** noch nicht implementiert.

- Aufgezeichnete Kännchenpose relativ zur Tasse und bekannte Ausgießer-Geometrie in ein **Werkzeug-TCP-Ziel** umrechnen.
- Mit vorhandenen MuJoCo-/Robotikbibliotheken **Inverse Kinematik** für FR5 lösen, Gelenkgrenzen und Kollisionen prüfen.
- Eine phasenweise Trajektorie (z. B. **DMPs** und die fünf Bewegungsphasen aus der One-Shot-Latte-Art-Forschungsarbeit) lernen und als Dry-Run **ohne Milch** simulieren.
- Nach dem Greifen kann der Roboter die Kännchenpose über Gelenkwinkel und festen Kännchen-Greifer-Transform berechnen. Kameras werden nicht zwingend für jede Steuerbewegung benötigt, bleiben aber für Demonstrations-Motion-Capture und Fehlerkontrolle wichtig.
- Objektiv messen: Spitzenabweichung relativ zur Tasse, erreichbare Bahn, zeitlicher Fehler, Verdeckungen, Gelenkgrenzen und Kollisionen.

**Abnahme vor Stufe 3:** FR5 reproduziert eine vollständige sichere Kännchenbahn virtuell; kein Durchdringen von Tasse, Arm oder Arbeitsfläche.

## Stufe 3 – Kontrollierte echte Latte Art (später)

**Status:** Planung, **keine Hardware gekauft**.

- Geeignetes Kännchen, lebensmitteltauglichen Halter/Marker, kalibrierte echte Kameras und Roboterwerkzeug auswählen.
- Standardisierte Tasse, Milchschäumung, Espresso und Gießgeschwindigkeit.
- Erst sichere reale Bewegungs-/Kalibrierungstests, danach einfacher Herz-Gießversuch; Qualität dokumentieren, keine automatische Erfolgsgarantie.
- Optionale dynamische Tassenneigung nach der One-Shot-Forschungsarbeit bewerten. Die dort dokumentierte Lösung nutzt auch aktive Tassenrotation.

## Fehlerinterpretation der bisherigen 16 Tests

Viele Zwei-Tag-Varianten hatten **`pitcher_status=PASS`, aber `scene_status=FAIL`**: Das **Hindernis** wurde nur von einer Kamera erfasst, obwohl mindestens zwei gefordert waren. Bei einzelnen Varianten war dessen Posenfehler zusätzlich hoch (bis 128,461 mm). Bei Caliscope + Pupil + einem Tag / drei Kameras überschritt das Hindernis die 5-mm-Grenze nur knapp (5,367 mm), obwohl die Kännchenmessung gut war. Im Vergleichsbericht bleiben diese Fehler ausdrücklich bestehen und werden nicht als Erfolg umgedeutet.

**Für das Motion-Capture-MVP ist die Kännchenmessung entscheidend**, aber zuverlässige Sicht auf Arbeitsbereich, Tasse und künftige Kollisionshindernisse ist für den tatsächlichen Roboterbetrieb trotzdem erforderlich. Ein einzelner Tag kann durch Handgelenk oder Greifer verdeckt werden; daher keinesfalls Sichtbarkeit im realen Ausgießvorgang voraussetzen.

## Code und Dateien – zentrale Anlaufstellen

- [COLAB_START.ipynb](../COLAB_START.ipynb) – **primärer 1-Tag-/3-Kamera-Start**.
- [COLAB_COMPARE.ipynb](../COLAB_COMPARE.ipynb) – **16-Wege-Archivvergleich**, nicht löschen.
- [robobarista/scene.py](../robobarista/scene.py) – MuJoCo-Szene, FR5-Assets, Tags und Kamerageometrie.
- [robobarista/geometry.py](../robobarista/geometry.py) – AprilTag-Erkennung und 6D-Mehrkamera-Pose.
- [robobarista/demo.py](../robobarista/demo.py) – Render-/Genauigkeitsbenchmark, auch alternative Versuchskonfigurationen.
- [robobarista/benchmark_gates.py](../robobarista/benchmark_gates.py) – getrennte Genauigkeits-/Sichtbarkeitskriterien fürs Kännchen vs. Gesamtszene.
- [robobarista/compare_rigs.py](../robobarista/compare_rigs.py) – 16-Vergleichsmessungen und `--analyze-only` ohne neue Renderläufe.
- [robobarista/caliscope_integration.py](../robobarista/caliscope_integration.py) – Forschungsalternative, behalten.
- [tests/](../tests) – Python-Tests (GitHub Actions führt sie automatisiert aus).
- [results/](../results) – gesicherte vom Nutzer gepostete Vergleichszusammenfassungen.
- [ROBOBARISTA_ROADMAP.md](./ROBOBARISTA_ROADMAP.md), [ROBOBARISTA_BACKLOG.csv](./ROBOBARISTA_BACKLOG.csv) – langfristige Roadmap und ältere Aufgabenliste; diese Planung ist die **aktuelle technische Referenz**.
- [ROBOBARISTA_MARKTRECHERCHE_20261020.md](./ROBOBARISTA_MARKTRECHERCHE_20261020.md) – weiterhin nützliche Markt-/One-Shot-Latte-Art-Recherche.

**Archivierung:** Auf GitHub liegen die 16 berichteten Maximalwerte und die Fehlerdiagnose als Text/CSV. Die ursprünglichen Colab-Kamerabilder, vollständigen `measurements.json` und Logs sind **nicht** auf GitHub gespeichert, solange sie nicht eigens exportiert werden. Alte Versionen lassen sich über die GitHub-Commit-Historie wiederherstellen.

