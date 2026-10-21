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

## Sicheres Pre-Dock und visuelles FR5-Lernen – neuer Meilenstein (21.10.2026)

**Neu abgeschlossen:** Sieben vollständige kollisionsvorgeprüfte FR5-Hin-/Rückwege zu **freien Vorpositionen** aller sieben Stationsschnittstellen, vollständig mit Gelenkwinkeln und Werkzeug-Kollisionshüllen geprüft. **Keine endgültigen mechanischen Dockkontakte freigegeben.** [COLAB_SAFE_STAGING](../COLAB_SAFE_STAGING.ipynb) · [Videobeleg](https://github.com/Coffee-Tobi/robocasa/actions/runs/39632253591/artifacts/12358904241).

**Neu abgeschlossen:** Getrennte gerenderte Demonstrationsbilder von drei RGB-D-Kameras, AprilTag-3-Schätzung der beweglichen Kanne aus den Bildpunkten (zusätzlicher Marker #72), invers-kinematisches Replay **nur anhand der gemessenen Bahn** mit allen 13/13 Frames und Gelenk-Sicherheitsvorprüfung. Maximaler Kamerafehler in dieser ideal kalibrierten virtuellen Sequenz: 0,924 mm / 0,339°. Alle acht festen Referenzmarker aus Bildern erkannt; maximaler Referenzfehler 2,784 mm. **Keine echte Menschendemonstration oder motorische Simulation.** [COLAB_SAFE_REPLAY](../COLAB_SAFE_REPLAY.ipynb) · [Vergleichsvideo](https://github.com/Coffee-Tobi/robocasa/actions/runs/39632915461/artifacts/12359487863) · [Codeerklärung](SAFE_CAMERA_REPLAY_EXPLAINED.md).

## Neue Standard-Szene V0.2: Artly-inspirierte Kaffeeinsel mit Andockrahmen (21.10.2026)

**[Neue Szene in Colab öffnen](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_WORKCELL.ipynb)** · **[Verbindlicher virtueller Geometrie- und Sicherheitsplan](WORKCELL_GEOMETRY_AND_SAFETY.md)**

Die kompakte freie Arbeitsinsel steht im Zentrum und trägt den FAIRINO FR5. Es gibt **sieben Stationen**: **Mühle mit integriertem motorischem Tamper**, Siebträger-Ablage, Espressomaschine, Milchspender, Kännchen-Ablage, Kunststoffbecher-Gießplatz und Rinser. **Keine separate Tamperstation mehr** (AprilTag #62 ist stillgelegt). Drei RGB-D-Kameras und oben montierte AprilTag-Referenzen bleiben erhalten. `robobarista/barista_cell.py` baut die MJCF-Szene als eigenes Modul, ohne ältere Forschungsnotebooks zu verändern.

**Sieben eigene Andockrahmen** in `robobarista/workcell_constraints.py`: Siebträger an Mühle, Siebträger an Espressomaschine, Kunststoffbecher an Espresso-Tropfschale, Kännchen an Espressodampflanze, Kännchen an Milchspender, Kännchen-Parkposition, Becher-Gießplatz. `DOCKS` speichert Position **in Metern**, Orientierungswinkel und Anfahrachsen; `rb_dock_*`-Sites visualisieren diese Ziele direkt in MuJoCo. Beispiel: Milchkanne an Dampflanze **(653, 222, 871) mm** relativ zum Szenenursprung. Alle Werte sind **nominale Konstruktionsannahmen**, keine aus echten Geräten vermessenen Präzisionswerte.

**No-Contact-Planungsfilter:** 30 mm Abstand bei Transit, 5 mm nahe erlaubtem Dock, Pose nur mit richtigem Werkzeug und max. 3 mm/2° nomineller Abweichung freigegeben. Auch Kanne **mit Griff und Ausgießer**, Siebträger und Becher werden als räumliche Körper approximiert; nicht nur der Handgelenkpunkt. `require_safe_segment` prüft Zwischenpositionen (max. 5-mm-Schritte) und **verweigert** einen geraden Weg durch die Espressomaschine; `require_safe_robot_state` blockiert bekannte MuJoCo-Kontakte der Arm-/Greiferkörper. Nur die jeweils benannte Schnittstelle kann bei richtigem Werkzeug eine räumlich eng begrenzte Ausnahme haben.

**Automatisch getesteter Stand:** Sieben Docking-Sites vorhanden und alle sieben *nominalen* getragenen Werkzeugkörper erfüllen den programmierten geometrischen Docking-Check. **Alle sieben Stationspunkte sind per IK erreichbar, aber nur drei von sieben getesteten Arm-Zielstellungen waren ohne MuJoCo-Kontakt**; die anderen vier sind für eine tatsächliche Armfahrt **noch nicht freigegeben**. Es müssen kollisionsfreie Alternativhaltungen und vollständige Bewegungswege geplant werden. Zehn verbleibende relevante AprilTags wurden jeweils in **8/8** kurzen Testframes erkannt (seitlicher Kännchenmarker weiterhin nicht). Der integrierte 55-mm-Mühlen-Tamper hat einen getrennten MuJoCo-Motortest mit Aus- und Rückfahrt bestanden, **ohne tatsächlichen Kaffeedruck**. GIF/MP4, `dock_targets.json`, `workspace_constraints.json`, `reachability.json` und Marker-Sichtbarkeit werden im [Workcell-Workflow](https://github.com/Coffee-Tobi/robocasa/actions/workflows/workcell.yml) erzeugt.

**Wichtige Einschränkung:** Es sind vereinfachte AABB-Gerätehüllen und Kugelmodelle für Werkzeuge. Weder vollständig kontinuierliche **Gelenkraum-Kollisionstests** noch dynamische Motor-/Kraftgrenzen, echte Greifversuche, Flüssigkeiten, präzises Hersteller-CAD oder realweltliche Sicherheitsfunktionen sind implementiert. Deshalb sind diese Abstände **Forschungs-/Planungswerte und keine Freigabe zur echten Roboterbewegung**.

## Nächster Schritt fertig: kinematisches FR5-Gelenk-Replay (21.10.2026)

**Direkt in [COLAB_ROBOT.ipynb](../COLAB_ROBOT.ipynb)** demonstriert das virtuelle FAIRINO-FR5-Modell nun die vorherige synthetische Gießbewegung mithilfe seiner **sechs echten Arm-Gelenke**. Der Fügepunkt `rb_pitcher` ist im MJCF **fest an `wrist3_link` montiert**; keine Teleportation eines frei bewegten Kännchens. Über `scipy.optimize.least_squares` wird die 6D-Wunschpose pro Frame per IK aus den richtigen MuJoCo-Gelenkgrenzen errechnet; anschließend liefert `mj_forward` die tatsächlich erreichte 6D-Pose, die mit dem Ziel verglichen wird. Mehrere IK-Lösungen werden über alle 25 Bilder auch auf Roboter- und Greiferkontakte überprüft, um eine ungeeignete Armhaltung auszusortieren.

**Automatischer Volltest in GitHub Actions:** 25/25 IK-Bewegungen reproduzierbar, keine Gelenkgrenzen verletzt, keine registrierten Eigen-/Umgebungskontakte im ausgewählten kinematischen Lauf (die bewusst nicht kollidierende **virtuelle Kännchengeometrie** ausgenommen). MP4/GIF mit orangefarbigem Ziel-Ausgießer und grünem tatsächlichen Ausgießer, IK-Kurven, CSV und JSON stehen über das Notebook und die [Workflow-Artefakte](https://github.com/Coffee-Tobi/robocasa/actions/workflows/fr5-replay.yml) zur Verfügung. **Keine Motorregelung, kein dynamischer Schutzabstand, keine Greifphysik, keine erlernte menschliche Latte-Art-Trajektorie und noch keine kameragespeisten IK-Sollwerte.**

**Nächste Teilaufgaben:** Sollbewegung aus `COLAB_MOTION` übernehmen (gemessene 6D-Pose relativ zur Tasse), dann Konfiguration realistisch montierter Greifer und Kännchengeometrie, vollständige Kollisionsprüfung einschließlich Kännchen sowie zeitabhängige Gelenkgeschwindigkeit/-beschleunigung und Trajektorienglättung.

## Schritt nach der ersten Kameraentscheidung – bewegter Kännchen-Test (21.10.2026)

**Aktuell implementiert:** [COLAB_MOTION.ipynb](../COLAB_MOTION.ipynb) und [robobarista/record_motion.py](../robobarista/record_motion.py) geben eine deterministische **synthetische Gießgeste** in MuJoCo vor. In jedem Frame erzeugen die drei Kameras RGB- und Tiefenmessungen; AprilTag-3-Erkennung + Mehrkamera-Optimierung rekonstruiert aus den Kamerabildern die 6D-Kännchenpose. Aus der geschätzten Pose wird die Ausgießerspitze berechnet. Soweit die Tasse ausreichend erfasst wird, wird ihre Relativposition ergänzt.

**Verifizierter automatischer Full-Render-Test (21.10.2026, 25 Frames, Linux/OSMesa):** Kännchen in **25/25** Frames von mindestens zwei Kameras rekonstruiert; maximal **0,256854 mm** absoluter Ausgießerspitzenfehler und **0,119134°** Winkelabweichung (`pitcher_status` dieses synthetischen Tests: PASS). Die Tasse wurde nur in **7/25** einzelnen Frames erkannt; die im ersten Frame kamerabasiert eingemessene feste Tassenreferenz erlaubte trotzdem eine relative Spitzenposition für alle 25 Frames. **Maximaler relativer Spitze-zu-Tasse-Fehler: 0,817647 mm**; liegt über dem längerfristigen 0,5-mm-Ziel und wird **nicht durch das PASS für die Kännchenpräzision abgedeckt**. Diese Messungen stammen vom GitHub-Actions-Renderlauf, nicht von der Colab-Sitzung des Nutzers.

**Messdaten:** `trajectory.csv/.json` (ausschließlich geschätzte Posen und Zeitstempel), `evaluation.csv/.json` (separate MuJoCo-Ground-Truth und Fehler, nicht als Eingang der Posenschätzung), `preview.gif` (Kamerafilm), `trajectory.png`, `validation.png`, pro Kamera/JPG die annotierten Bilder. Kein Fehlframe wird mit der tatsächlichen MuJoCo-Pose ersetzt.

**Keine falschen Versprechen:** Dies ist eine **virtuell vorprogrammierte Bewegung**, nicht die Erfassung eines echten Menschen oder ein FR5-Replay. Die Bilder werden in zeitlicher Reihenfolge gerendert, der Simulationszeitstempel ist `frame/fps`; noch keine Echtzeit, Kollisionsprüfung oder Milch. Ein erfolgreicher Durchlauf bestätigt nur einen ersten dynamischen Sensorik-Datenfluss. Als nächstes die gemessene Bahn relativ zur Tasse prüfen, dann echten FR5-IK-/Dry-Run planen.

**Qualitätssicherung:** Zusätzliche Unit-Tests für Zeitbasis, Rotation, Start-/Endpose und fehlende Messungen sowie ein automatischer GitHub-Test mit echtem MuJoCo-Rendering (`.github/workflows/motion-smoke.yml`).

## Stufe 1 – Kamera-Motion-Capture zuverlässig machen

**Status:** statischer Benchmark bestanden. Bewegte synthetische Kameraaufnahme als nächster Prototyp implementiert; vollständige Bewegungsqualität und echte menschliche Demonstration noch ausstehend.

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

- [COLAB_START.ipynb](../COLAB_START.ipynb) – bisherige statische 1-Tag-/3-Kamera-Baseline.
- [COLAB_MOTION.ipynb](../COLAB_MOTION.ipynb) – **jetzt: animierte Kamerademonstration des bewegten Kännchens**.
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

