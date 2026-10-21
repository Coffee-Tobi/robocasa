# RoboBarista – Arbeitszelle, Andockrahmen und Kollisionspolitik (virtueller Entwurf)

**Stand 21.10.2026 · Planungsstand V0.2, NICHT für reale Roboterfreigabe.**

## Ein einziges Koordinatensystem

MuJoCo-Weltkoordinaten, **Meter**, X rechts, Y Richtung Rückwand, Z aufwärts; Tischoberseite Z = **0,820 m**. FR5-Basis bei **(0, 0, 0,910) m**. Werkzeug-Zielwerte bezeichnen vorläufig den **Ursprung des getragenen Objekts** – nicht den Flansch/TCP des Roboters. Der starre Transform `Roboterflansch → Greifer → Werkzeugursprung` ist vor echten Greifversuchen separat zu kalibrieren.

**Artly-inspiriertes räumliches Konzept**, kein öffentlich verifiziertes Artly-CAD. Gerätegehäuse sind grobe Grundformen; Einbaumaße sind nicht die Herstellerdaten von Bengle, Mühle, Tamper oder Milchspender.

## Exakte nominelle Andockziele

**Alle Werte in Millimetern relativ zur MuJoCo-Welt.** Das Feld `Euler XYZ` ist in dieser frühen Modellausführung **(0°,0°,0°)**, bis echte Werkzeugaufnahme-Orientierungen und die Brühgruppen-Bajonettgeometrie konstruiert sind. Die Achse der *Anfahrt* ist getrennt definiert und wird später für Pre-dock-Vorgehen verwendet.

| ID im Code | Werkzeug | X | Y | Z | Anfahrt |
| --- | --- | ---: | ---: | ---: | --- |
| `grinder_portafilter` | Siebträger | -560 | 122 | 954 | +Y |
| `espresso_portafilter` | Siebträger | 440 | 287 | 1090 | +Y |
| `espresso_cup` | Kunststoffbecher | 440 | 205 | 882 | +Y |
| `espresso_steam_pitcher` | Milchkanne | 653 | 222 | 871 | +Y |
| `milk_dispenser_pitcher` | Milchkanne | 575 | -500 | 841 | +Y |
| `pitcher_park` | Milchkanne | -150 | -550 | 834 | -Z |
| `pour_cup` | Kunststoffbecher | 350 | -370 | 823 | -Z |

Brühgruppe/Siebträger, Becherschale und Dampflanze der Espressomaschine haben **getrennte, namentlich gesperrte Schnittstellen**. Die Mühle hat **einen** Siebträgeranschluss für Mahlen und den **internen** automatischen Tamper; die separate Tamper-Station und ihr AprilTag #62 wurden entfernt. Physische Kollisionsschalen sind auch um die seitlichen Schutzleisten des Mühlen-Tampers modelliert.

## Aktuelle Sicherheitsgeometrie

- Normaler Sicherheitsabstand zu Maschinengehäusen: **30 mm**.
- Andock-/Kontakt-Näherung: nominell **5 mm** außerhalb des ausdrücklich freigegebenen Ports, **nur** bei richtigem Werkzeug, genanntem Port und Sollpose innerhalb von **3 mm und 2°**.
- Nur an Port und passendem Werkzeug darf die definierte Schnittstellengeometrie näher herankommen. Es gibt **keine pauschale Kollisionsfreigabe** an einer Maschine; alle anderen Gehäuse bleiben geschützt.
- Bei tragbarer Milchkanne werden die Volumina der Kanne, des **Griffes** und des **Ausgießers** einbezogen (mehrere vereinfachende Kugeln); ebenso Siebträger und Kunststoffbecher mit eigenen Geometrien.
- Geradlinige Werkzeugbewegungen werden mit einer räumlichen Schrittweite von **maximal 5 mm** und einer Winkeländerung von **maximal 3°** vorgeprüft. Die Prüffunktion wirft bei verbotener Bewegung eine Exception (`require_safe_segment`).
- MuJoCo überprüft zusätzlich die vorhandenen **Roboter-/Greiferkontaktkörper** mit den kollidierbaren Maschinenkörpern. Eine kinematisch erreichbare Zielstellung **kann trotzdem verboten sein**, wenn ein Robotersegment ein Gerät berührt. Für einzelne Konfigurationen gibt es `require_safe_robot_state`.

Die Gerätehüllen sind derzeit **achsenparallele Bounding-Boxes** mit groben gerundeten Werkzeug-Volumenhüllen. Die Schutzregeln sind **Planungsfilter, keine sicherheitsgerichtete Steuerung, keine Garantie kollisionsfreier Zwischenstellungen in Gelenkräumen und keine Kontaktkraftsimulation**. Die Endeffektor- und Gerätegeomtrie muss erst an echten CAD-Dateien validiert werden; auch die zulässigen mechanischen Anschläge an Bajonett und Düse brauchen Detailmodelle. Erst dann kann eine vollständige Routenplanung und eine Geschwindigkeits-/Kraftfreigabe folgen.

## Code – eine Quelle der Wahrheit

- `robobarista/workcell_constraints.py`: `DOCKS`, `KEEP_OUTS`, `PAYLOAD_SPHERES`, `payload_check`, `docking_pose_check`, `validate_segment`, `require_safe_segment`, `require_safe_robot_state`.
- `robobarista/barista_cell.py`: Gerätegeometrie, integrierter Motor-Tamper, MuJoCo-`rb_dock_*`-Sites und AprilTags.
- `robobarista/cell_demo.py`: aktuelle 3-Kamera-Visualisierung, physikalisch separater Tamper-Motortest, pro Station IK-/Roboterkontakt-Diagnose, `workspace_constraints.json` und `dock_targets.json`.
- `COLAB_WORKCELL.ipynb`: Film, Grundriss, mm-Koordinaten und QA-Bericht.
- `tests/test_barista_cell.py`: blockierter Maschinen-Durchflug einschließlich Kannenvolumen, Werkzeug-Port-Zuordnung, Zielabweichung, Referenzen und Einzelkonfigurations-Stop.

## Ergebnisse der neuen Routensuche

Die reine IK-Erreichbarkeit aller Stationen reichte bislang nicht: Vier von sieben vorher getesteten direkten *Stationsmitte*-Armlagen erzeugten MuJoCo-Kontakte. Der neue konservative Planer `safe_staging.py` sucht deshalb für **alle sieben benannten Geräteschnittstellen** alternative **Vorpositionen außerhalb der Gerätehüllen** und prüft **den gesamten Weg hin und zurück** in Gelenkschritten ≤0,018 rad. Das hat für alle sieben einen überprüften, kinematisch kollisionsfreien Weg zu einer Vorposition ergeben. **Ein mechanischer Dockvorgang bis in das Geräteinnere ist weiterhin nicht freigegeben**.

Das separate Kameratestprogramm `camera_joint_replay.py` verwendet aus einer zufälligen, vorgeprüften Bewegung gerenderte AprilTag-Bilddaten für eine rekonstruierte 6D-Bahn und führt diese nur über die Kamera-Messwerte dem FR5 zu; Resultat: **13/13** Kameraframes und **13/13** FR5-Posen, maximal **0,924 mm** kamerabasierte Positionsabweichung gegenüber separater virtueller Wahrheit. Die **acht festen Referenzmarker** wurden optisch erfasst, Fehler zum bekannten Szenenbezug maximal **2,784 mm**. Mehr dazu in [SAFE_CAMERA_REPLAY_EXPLAINED.md](SAFE_CAMERA_REPLAY_EXPLAINED.md).

## Bevor eine echte Bewegung überhaupt freigegeben wird

1. **TCP und Werkzeuge vermessen:** Flansch → Greifer → Siebträgergriff/Kännchenkörper/Ausgießer; wiederholbare Greiflage und Werkzeugwechsel.
2. **CAD- und Kollisionsvolumina präzisieren:** insbesondere warme Espressomaschinenflächen, Dampflanze, Milchspender-Düse, Tamperkanal, Siebträger-Bajonett, Verkleidung, Stationsträger und Kabel.
3. **Andocktrajektorien festlegen:** Vorposition, freie Zufahrt, verlangsamte Dockfahrt, erlaubter mechanischer Kontakt, Rückzug – Werkzeug/Port/Phase müssen explizit übereinstimmen; bei Ausrichtungsfehler nicht mit Gewalt einfahren.
4. **Vollständige Armbewegungen planen:** FK/IK-Joint-Limits, Selbst-/Umgebungskollisionen entlang aller Zwischenstellungen, ausgetragene Objekte, Gelenkgeschwindigkeit/-beschleunigung, Kontaktkraft, Griffstabilität, Abbruch bei Marker-Verlust.
5. **Vermessbare Referenzen:** AprilTag-Zellkalibrierung, Kalibrierkörper an jedem Dock, Sichtbarkeitsmatrix je Betriebsphase, separate relative Positionsmessung von Becher und Kännchenspitze.
6. **Physische Sicherheit und Hygiene:** Gehäuseabschrankung, Not-Halt, sicheren Betriebsmodus, Verbrühungsschutz, trockene Elektronikzonen, Wasser-/Milchreinigung, Lebensmittelsicherheit.
7. **Plastikbecher auswählen:** standfeste Maße und Masse sowie Temperaturfreigabe des echten Übungsbechers; für heiße Getränke keine ungeeigneten Einwegkunststoffe einsetzen.

### Aktuelle Maschinen-Zielwerte nicht überinterpretieren

Ein im JSON gespeicherter Sollort auf **1 mm** bedeutet lediglich eine **präzise definierte Nominalkoordinate**, nicht, dass unsere Kameras, der reale FR5 oder eine echte Espressomaschine die Schnittstelle auf 1 mm genau ausgerichtet haben. Das ist ein **noch zu kalibrierendes Konstruktionsmaß**. `workcell_constraints.json` und `dock_targets.json` sind die nach jedem Colab-Lauf automatisch exportierten, maschinenlesbaren Fassungen dieser Planungsparameter.
