# Neue Gesamtsimulation – Kamera / Roboter / Espresso / Milch / Latte Art
**Stand: 21.10.2026 | Virtuelles FAIRINO FR5 | kein Hardware- oder Getränke-Nachweis**

[Komplettes Colab](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_ROBOBARISTA.ipynb)

## Ablauf (alle Module im Colab)

### Arbeitszelle und Sicherheitsfilter
`barista_cell.py` baut einen Tisch mit dem FR5 im Zentrum auf, mit Mühle und **integriertem Tamper**, Siebträgerstation, Espressomaschine mit Brühgruppe/Becherplatz/Dampflanze, Milchspender, Kannenablage, Kunststoffbecher und Rinser. Sieben benannte Andockrahmen werden in `workcell_constraints.py` verwaltet. Geräte und die ganze geometrisch approximierte Kanne (inklusive Griff und Ausgießer) besitzen No-go-Zonen, die während den geprüften Wegen überprüft werden. Eine reale Kontaktsicherheit, Greifer und tatsächliche Geräteanschlüsse sind noch nicht modelliert.

### Kamera und Raumrekonstruktion
`room_reconstruction.py` rendert **drei RGB-D-Kamerabilder** bei 960×720. Daraus werden die Tiefenpixel über die **aus dem simulierten Kamera-Mount bekannten** intrinsischen/extrinsischen Parameter in eine echte sichtbare 3D-Punktwolke projiziert und acht AprilTag36h11-Stationen **aus den Pixeln** erkannt. Erst nach der Rekonstruktion werden die Markerpositionen und die aus Sichtpunkten abgeschätzte Tischhöhe gegen die bekannten MuJoCo-Koordinaten geprüft. Das ist keine Full-Scene-TSDF/SLAM-Karte: verdeckte Oberflächen fehlen, und die Kamerakalibrierung ist ideal.

**Erfolgreich getestetes Beispiel:** 52.161 Tiefenpunkte, 8/8 feste Tags, mittlerer 3D-Tag-Lagefehler **0,731 mm**, maximal **2,784 mm**, geschätzte Tischhöhe **820,939 mm** statt **820,000 mm** (0,939 mm Fehler; der Tischhöhen-Algorithmus kennt eine grobe Suchzone 760–870 mm). Das sind **virtuelle** Fehler; nicht auf echte Hardware übertragbar.

### Espresso / Milch / Latte-Art-Storyboard
`coffee_workflow.py` verwendet **11** getrennte Stages mit tatsächlichen **Sechs-Gelenk-Bewegungen** des FR5:

1. Siebträger-Aufnahme (**visuelle Übergabe**)
2. Mühle mit integriertem Tamper (**virtuelle Mahl-/Tamp-Phase**)
3. Brühgruppe anfahren (**kein echtes Verriegeln**)
4. Kunststoffbecher zur Espresso-Station (**visuelle Übergabe**)
5. Espressobezug (**braune Darstellungsfläche, keine Flüssigkeit**)
6. Milchkanne aufnehmen (**virtuelle Übergabe**)
7. Milchdosierstation (**keine physikalische Milchströmung**)
8. Dampflanze anfahren / Aufschäumen (**Dampf-/Schaumattrappe**)
9. Espresso-Kunststoffbecher zum Gießplatz bewegen (**visuell**)
10. Angenommene Gießgeste in fünf Phasen: **Approach → Base-fill → Lower → Sway → Cut-through** (mit Kollisionsprüfungen, aber noch keine echte Latte Art)
11. Symbolische Reinigung / Rinser.

Alle **11** Armfahrten zu **freien Vorpositionen** werden erneut geprüft. Jeder Roboter- und Werkzeugzwischenzustand unterliegt dem selben No-touch-Filter; es werden **keine Kollisionsausnahmen für das gesamte Gerät** gewährt. Der Film enthält **97 gerenderte Bilder**. Die tatsächliche Kanne und der Siebträger werden noch nicht physikalisch gegriffen, und die letzten Zentimeter bis zu Geräteöffnungen sind ausdrücklich **nicht ausgeführt**.

### 3a: kontinuierliche 6D-Kamera-Bewegung
`camera_joint_replay.py` rendert in dem getesteten 2-s-Intervall **31 echte virtuelle Kameraframes mit 15 FPS**. In jedem Bild werden zwei oder mehr Kameraansichten von AprilTag-Markern verwendet, um die Position und Rotation der getrennt bewegten Kanne zu messen. `continuous_motion.py` fitttet daraus X/Y/Z durch drei glättende kubische Splines und die Rotation durch `RotationSpline` auf SO(3). Die Positions- und Rotationsableitungen beschreiben die geschätzten Geschwindigkeiten und Beschleunigungen.

`camera_continuous_6d.json` enthält daraus **201 Samples bei 100 Hz**. **Das sind interpolierte Kurvenwerte, keine zusätzlichen optischen Messungen.**

### 3b: lokale jerk-begrenzte Ruckig-Gelenkbahn
Die 6D-Kamera-Kurve dient per IK als Ziel an ausgewählten Zeitpunkten. Zwischen **11** kameragespeisten Gelenkstützpunkten verbindet Ruckig **10 lokale State-to-State-Abschnitte** mit denselben, auch nicht-null Geschwindigkeiten und Beschleunigungen an Zwischenpunkten. **Keine Ruckig-Community-Cloud-Mehrwegpunktfunktion.** Die virtuelle Sechs-Gelenk-Bewegung wird nochmals gegen Kollisionskörper und Werkzeug-Volumen geprüft und als `continuous_ruckig.mp4` dargestellt. Ein Beispiel brauchte **6 s statt 2 s Originalzeit**, um die *angenommenen* Geschwindigkeits-/Beschleunigungs-/Jerk-Grenzen einzuhalten; diese sind noch keine Herstellergrenzen des realen FR5.

### Getrennter physikalischer Motorversuch
`motor_follow.py` simuliert mit `mujoco.mj_step()`, sechs FR5-Drehmomentantrieben, proportional-derivativer Regelung, Schwerkraft-Kompensation und Motor-Drehmomentgrenzen die optisch nachgeführte Gelenkbahn. Die *neue Ruckig-Sechs-Gelenk-Trajektorie* ist dabei noch **nicht** als Motorvorgabe verdrahtet; Motor- und Ruckig-Auswertung sind **getrennte Versuche**. Da der 15-FPS-Versuch bei 2,5-facher Zeitstreckung über 13 mm maximale Folgeabweichung hatte, wurde für den vollständigen neuen Colab-Durchlauf eine **4-fache Zeitstreckung** als weiterer Test gewählt, ohne dadurch echte Milchphysik/Latte-Art-Gleichheit zu behaupten.

## Relevante fremde Projekte und was wir übernehmen
- [Artly](https://www.artly.ai/applications/barista-bot): Arbeiten mit einer Bibliothek erlernter Barista-Fertigkeiten und Bewegungsdemonstrationen – **Empfehlung:** unsere klare Skill-Phasen-/Werkzeugzustandsarchitektur beibehalten.
- [One-Shot Learning for Double-Shot Latte Art (Khan et al., Oktober 2026)](https://arxiv.org/abs/2610.16179): **fünf phasenspezifische CDMPs** und demonstrierte Kännchendrehung – **Empfehlung:** vor realer Latte Art echte menschliche Demonstrationen lernen, statt eine lange globale Bewegung zu kodieren.
- [MoveIt Task Constructor](https://moveit.github.io/moveit_task_constructor/): getrennte Generate IK / Pick / Attach / Place / Detach / Collision-Scene-Stages – **Empfehlung:** später für präzise Greifer- und Geräte-Interaktionen übernehmen. Die vorhandene Colab-Simulation benötigt noch kein ROS 2.
- [Ruckig (Dokumentation)](https://docs.ruckig.com/tutorial.html): jerk-begrenzte Offline-Zustandsübergänge – verwenden wir jetzt **lokal**, ohne Cloud-Wegpunktfunktion.

## Offene Anforderungen vor echter Kaffeezubereitung
Roboterwerkzeug-CAD; Greif- und Verriegelungsmechanik für Siebträger; wirkliche Geräte-Interaktionspunkte mit kollisionsfähigen Ausschnitten; Milch- und Dampffluss, tatsächlicher Schaum und Latte-Art-Hydrodynamik; echte Kamerakalibrierung; Herstellergelenkgrenzen; unabhängige sicherheitsgerichtete Abschaltung. Bisherige Filme sind **logische und kinematische Simulationsnachweise**, keine realen Kaffee-/Latte-Art-Produkttests.
