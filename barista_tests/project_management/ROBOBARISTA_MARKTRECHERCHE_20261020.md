# RoboBarista – Abgleich mit realisierten Latte-Art-Robotern
**Recherchestand: 20. Oktober 2026** · **Status: Quellenrecherche, nicht als Benchmark/Code-Verifikation ausgegeben.**

## Kernaussage
**Gemessene reale Bewegung statt gewöhnliches Video als primäre Lernquelle** ist für einen schnellen ersten Herz-Prototyp das risikoärmste Verfahren. Dass ein humanes Eingießen phasenweise als DMP gelernt und ohne Milchfeedback von einem realen Roboter wiedergegeben werden kann, ist durch die neue One-Shot-Arbeit demonstriert. **Visuelle Prüfung vor/nach der Bewegung** ist zuerst sinnvoll; ein simultanes End-to-End-Video-Training oder unmittelbares KI-Livefeedback während der Schaumbildung ist kein erforderlicher MVP-Baustein.

**Kein Nachweis eines einheitlichen „Branchenstandards“:** Artly und COFE+ sind proprietär; die publizierten Informationen belegen nicht, dass ihre *inneren Latte-Art-Bewegungsregler* identisch funktionieren. Die Forschungsprojekte verwenden unterschiedliche Ansätze.

## Vergleichsmatrix
| System/Projekt | Realität / Evidenz | Demonstrations- bzw. Bewegungsdaten | Visuelle Rückmeldung | Für uns |
|---|---|---|---|---|
| **Artly Coffee** | Kommerziell betriebene Barista-Bots; Herstellerdokumentation + Fachbericht | Profi-Barista-Demonstrationen, Motion Capture/VR-Wrist/Elbow-Movement; modulare Skills. Interner Code proprietär | Artly berichtet QA-Checks *zwischen* Arbeitsschritten sowie visuelle Anpassung bei Lageabweichungen; nicht sicher belegt ist, dass ein laufender Latte-Art-Guss auf sichtbare Foam-Konturen in jedem Frame regelt | Bewegungserfassung vom Barista; Skill-Bibliothek; zuerst Qualitätsprüfung, danach adaptive Regelung |
| **COFE+** | Hersteller beschreibt kommerziellen Robot-Kiosk mit wirklichem Eingießen *und* separater essbarer 3D-Oberflächendruck-Funktion | Digitalisierte, validierte Barista-Bewegungsrezepte; laut Hersteller Bewegungsbahnen abgespeichert | Hersteller berichtet standardisierte Milch-/Dampfführung und Rezeptparameter. Detaillierter Live-Musterregler nicht öffentlich verifiziert | Reproduzierbare Trajektorien und standardisierte Eingangsbedingungen |
| **One-Shot / phased CDMP** | Begutachtung nicht abschließend bekannt; arXiv preprint mit expliziten **realen Milch-/Espresso-Versuchen** und Ergebnisfotos | 2 HTC-Vive-Tracker (einer an Kanne, einer am Bezugsrahmen); Kännchen 6D bei ca. 90 Hz; 1 AS5600 Encoder ~100 Hz am drehbaren Tassenhalter; Phasen via Fußschalter; Daten auf 50 Hz interpoliert; 5 CDMPs | **Open loop bzgl. Milch**. Roboterausführung via Fairino FR5 / ROS2 Humble / MoveIt2. Auf 280- und 400-ml-Becher wurde ein erkennbares Herz gegossen; nicht gleich gut wie Mensch. Bildbasierte Tassenposition ist ausdrücklich *Zukunftsarbeit* | **Primäre technische Vorlage**; lieber 6D-Tracker plus Becherwinkel als Videopose-Rekonstruktion |
| **π0 OpenDroid Latte Art Hackathon** | Dokumentierter Forschungs-/Hackathon-Prototyp, kein validierter Kiosk-Betrieb | 40 durch geführte Roboterarme erhobene Demonstrationen, drei Kameraansichten, 12D-Gelenkdaten bei 20 Hz; π0 fine-tuning; temporal action chunks | Bilder sind Policy-Eingaben, aber keine interpretierbare Latte-Art-Masken-Ausgabe dokumentiert. Reales Training mit H100-Aufwand angegeben | **Späterer Vergleich** für offene, flexible Zweiarm-Manipulation; für ersten 1-Arm-MVP unnötig komplex |
| **Cafe X (2018)** | Kommerziell eingesetzter Roboter für Kaffeekiosks | Automatisierte Maschinen und robotisches Becherhandling | Keine echte Hand-Pour-Latte-Art (historischer Bericht) | Ungeeignete Referenz für **gegossene** Latte Art |

## Technische Befunde aus dem kompletten One-Shot-Paper (arXiv 2610.16179v2)
- Die Autoren messen **nicht** die Kännchenpose aus einem normalen RGB-Video, sondern mit Motion-Trackern (3D-Position + Quaternion). Der Becher wird durch einen **aktiven drehbaren Halter** geneigt. Neigungsprofil wird per AS5600 gemessen, im Roboterversuch synchron über **NEMA17 + ESP32-S3/TMC2209** wiedergegeben. Ein Fußschalter markiert fünf Phasen.
- **Phasen:** `base_to_mix`, `mixing`, `mix_to_art`, `art`, `return_base`. In den eigentlichen Eingießphasen wird der Zeitablauf bewusst **nicht** beliebig verkürzt/verlängert, weil dann die eingegossene Milchmenge anders wird.
- Der Ausgießer ist über einen festen Werkzeug-Transform zum Roboterflansch kalibriert; Inverse Kinematik löst jedes Waypoint; gemeinsame zeitgesteuerte Robotertrajektorie und separat synchronisierter Becherhalter.
- **Keine eigene numerische Milchsimulation** zum Erfolg nötig: echte Milch und Espresso im realen Versuch.
- Ergebnis **E2 (phased CDMP)**: recognisable Heart bei 280 ml; Übertragung auf 400 ml durch räumlichen Offset und angepasste Rezeptmenge. **Abweichungen bleiben:** das Herz war breiter/„blobbier“, Pull-through teilweise außermittig, obwohl die Kännchenbewegung präzise reproduziert wurde. Verfasser nennen wechselnde Milchschaumtextur als mögliche Erklärung, jedoch ohne kontrollierte Untersuchung.
- Ergebnis **E1 (ein einziges CDMP)**: Aus Sicherheitsgründen **nicht** mit Milch gegossen, da Kännchen während der Herzphase außerhalb der Tasse lag. Das begründet phasenweise statt einziger, globaler DMP.
- **Mustererkennung / visuelle Rückkopplung im laufenden Guss ist gerade NICHT Teil der Methode.** Reale Tassenlage per Kamera statt voreingestellter bzw. iterativ ermittelter Versatz ist zukünftige Arbeit.
- Code nicht öffentlich verifiziert; CatalyzeX zeigte „Request Code“.

## Interessante Artly-nahe Patentbeschreibung (als Quelle, nicht als Übernahmeempfehlung)
Die US-Patentanmeldung **US20250289117A1** beschreibt die Synchronisierung von 3D-Tracker-Kännchen-/Tassenbewegungen und die Anpassung des anfänglichen Kännchenwinkels an einen von einer Kamera ermittelten Milchfüllstand. **Anmelder laut Google Patents: Blue Hill Tech Inc.** Diese Anmeldung ist **kein Beweis**, dass sämtliche Varianten im aktuell ausgelieferten Artly Barista Bot eingesetzt werden, und der beschriebene Gegenstand ist nicht automatisch frei nachbaubar. Frei verfügbare Forschungsbibliotheken und eigene unabhängige Implementierungsentscheidungen bevorzugen.

## Konkrete Empfehlung für RoboBarista
**Erste Zielversion:** Ein erkennbares Herz bei identischer Tasse und standardisiertem Milchschaum.
1. **Eine fachlich gute Latte-Art-Demonstration mit Kännchen-6D und Tassenwinkel** aufzeichnen oder, wenn Originaldaten freigegeben werden, wiederverwenden.
2. **DMP in fünf Phasen** mit freier Bibliothek implementieren; visualisieren und mit gemessener Pose vergleichen.
3. **Roboterarm im vorhandenen MuJoCo** den Ausgießer entlang der gelernten Bahn bewegen lassen; Sicherheitsgrenzen/Abweichungen dokumentieren. **Keine Fluid-Simulation**.
4. **Tassenhalter zunächst statisch** nur als A/B-Baseline. **Wenn Herz mit fixem Halter misslingt,** synchronisierte Motorneigung ergänzen, die in der erfolgreichen Publikation zentral ist (RB-45).
5. **Realen kontrollierten Milchversuch** erst nach sicherem Dry-Run. Zunächst eine Draufsichtkamera für fertiges Herz und falls nötig eine Kameramessung der Tassenlage *vor* dem Eingießen. Nicht live in den Strahl regeln.
6. **Wenn Qualität nicht stabil:** Fehlerquelle anhand Bild und Pose bestimmen; optional Rezept/Milchschaumkonsistenz, Startfüllstand, Tassenlage kontrollieren. Erst danach visuelle Live-Korrektur oder getrenntes Video-Lernen.

**Nicht als notwendig für MVP behandeln:** normales RGB-Video → vollständige 6D-Kännchenpose; VLA end-to-end; High-End-Milchfluss-Sensorik; CFD/Schaumsimulation; 3D-gedruckte Motivbilder auf Milchschaum.

## Offene Punkte (prüfen, nicht vermuten)
- Gibt es offiziell freigegebene One-Shot-Paper-Trainingsdaten oder Code? Autor*innen direkt anfragen.
- Können wir eine passende 6D-Demonstration ohne eigenen Roboter günstig aufnehmen (SteamVR-Tracker + Anker oder alternatives Tracking)?
- Ist die dynamische Tassenneigung für einen erkennbaren Heart-Pour **notwendig** oder reicht eine feste Neigung für eine vereinfachte Demonstration?
- Wie hoch muss die Kännchen- und Winkelgenauigkeit im späteren realen Aufbau sein?
- Reicht die reine Wiederholung für mehrere Versuche reproduzierbar aus? **A/B-Prüfung**, keine Annahme.
- Bei öffentlicher / kommerzieller Implementierung von Motion-Capture-Ideen IP- und Lizenzlage berücksichtigen.

## Quellen (Primärquellen soweit verfügbar)
1. [One-Shot Learning for Double-Shot Latte Art, v2, ganzer Text](https://arxiv.org/abs/2610.16179) – Methoden, 2× HTC Vive, AS5600/NEMA17, fünf Phasen, echte Herzversuche und Grenzen.
2. [Artly – Barista Bot](https://www.artly.ai/applications/barista-bot) – Herstellerangaben Motion Capture, VLA, Adaptation und Skillbibliothek.
3. [Artly – Growing List of Skills](https://www.artly.ai/research/growing-list-of-skills-learnt-from-human-experts) – Herstellerangaben zu wiederverwendbaren gelernten Latte-Art-Fähigkeiten.
4. [Artly – Quality Checks](https://www.linkedin.com/posts/artly-ai_artly-activity-7444436610219413504-zdP4) – Herstellerbeitrag zu Qualitätskontrolle zwischen Schritten.
5. [Artly Interview/Fachbericht, National Restaurant News](https://www.nrn.com/restaurant-technology/automated-beverage-solutions-emerge-as-top-labor-relief-trend-for-restaurants) – VP beschreibt VR-Aufnahme von Arm-/Handgelenksbewegungen.
6. [US20250289117A1, Blue Hill Tech Inc.](https://patents.google.com/patent/US20250289117A1/en) – Patentansprüche/Beschreibung, kein Beweis des aktuell verbauten Artly-Systems.
7. [COFE+ – Kiosk Latte Art](https://hi-dolphin.com/robot-coffee-kiosk-latte-art-turning-every-cup-into-a-social-share/) – herstellerseitige Beschreibung der digitalisierten Rezepte, QA und Trennung von Gießen vs. essbarem Druck.
8. [Coffee+ Schweiz – gegossen versus gedruckt](https://coffeeplus.ch/so-funktioniert-es) – beschreibt gegossene Herz-/Blattmotive vs. Kakaodruck.
9. [OpenDroid π0 Latte Art Hackathon](https://ridxm.github.io/latte-art-robot/) – Projektangaben zu Robotern, Demonstrationen und Training.
10. [Cafe X 2018, Eater SF](https://sf.eater.com/2018/2/6/16976446/cafe-x-sf-robot-coffee-market-street) – damals keine Latte-Art-Funktion.
