# RoboBarista – sichere Anfahrten und echte Kamera→FR5-Wiedergabe

**Stand: 21. Oktober 2026.** Alle Testergebnisse sind **rein virtuell**. Die Stationsplanung und das Kamera→Gelenk-Replay sind kinematisch und verwenden ideal kalibrierte MuJoCo-Kameras; **ein zusätzliches Experiment verwendet nun die echten sechs virtuellen Drehmomentmotoren mit `mujoco.mj_step`**. Dies ist kein Nachweis der Sicherheit oder Positioniergenauigkeit des realen FAIRINO FR5.

## Zwei getestete Simulationen

### A. Vorpositionen vor den sieben Kaffee-Schnittstellen

Mit `robobarista/safe_staging.py` werden um alle sieben in `DOCKS` definierten Geräteschnittstellen mehrere freie Vorpositionen (typisch 14–32 cm vor dem Gerät und 25–40 cm höher) gesucht. Für die echte FR5-Kinematik wird eine IK-Lösung benötigt, deren **Gelenkstellung** keine MuJoCo-Kontakte des Roboters mit Maschinen oder sich selbst erzeugt. Zusätzlich dürfen die vereinfachten Hüllkörper der mitgetragenen Werkzeuge/der **ganzen Milchkanne** Maschinen nicht berühren.

Für jede Vorposition prüft der Algorithmus einen **vollständigen Hin- und Rückweg** aus dem bekannten freien Startzustand in Gelenk-Schritten von maximal 0,018 rad pro Gelenk. Scheitert ein Test, wird dieser konkrete Weg **gesperrt**. Der Film zeigt nur die genehmigten Routen, die zu einem pre-dock führen.

**Automatisch verifiziert:** **7/7** Schnittstellen haben mindestens einen kollisionsgeprüften **freien Voranfahrpunkt** und einen geprüften Rückweg:
- Mühle–Siebträger
- Brühgruppe–Siebträger
- Tropfschale–Kunststoffbecher
- Dampflanze–Milchkanne
- Milchspender–Milchkanne
- Kännchen-Parkplatz
- Kunststoffbecher-Gießplatz

**NICHT** freigegeben: die eigentlichen Andockkontakte und mechanischen letzten Zentimeter; auch nicht dynamische Geschwindigkeits-/Beschleunigungsgrenzen oder vollständige continuous-time swept collision checks. Mit einem anderen Werkzeug oder neuer Umgebung muss neu geplant werden.

**Filme und Code:**
- [Colab: Sieben sichere Anfahrten](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_SAFE_STAGING.ipynb)
- [Gespeicherter erfolgreicher GitHub-Test mit MP4](https://github.com/Coffee-Tobi/robocasa/actions/runs/39632253591/artifacts/12358904241)
- [Funktion `plan_staging`](../robobarista/safe_staging.py), [Film-Simulator](../robobarista/safe_station_staging_demo.py)

### B. Zufällige erlaubte Demonstration optisch aufnehmen und vom FR5 nachfahren

`robobarista/camera_joint_replay.py` arbeitet in zwei strikt getrennten Phasen:

1. Ein deterministischer Zufallsweg wird **im Gelenkraum** erzeugt und vorab gegen das Modell geprüft; es ist also noch **keine menschliche Bewegung**, sondern eine kleine Demonstration innerhalb des sicheren Arbeitsbereichs.
2. Eine separate virtuelle Milchkanne führt die Bewegung vor. Drei virtuelle Kameras liefern normale MuJoCo-RGB- und Tiefenbilder.
3. Aus **Pixeln** des AprilTag-3-Detektors auf der Kanne, den bekannten Starrkörper-Transformationen der Marker und den Kameraparametern wird mittels `bundle_adjust_object` **Position + Orientierung** rekonstruiert. Die vorher bekannte Kännchenposition ist für die Berechnung **nicht** zugänglich.
4. Die gesamte rekonstruierte 6D-Kamerabahn wird per `compute_ik` in sechs Gelenkstellungen übersetzt. Jede Verbindung zweier Gelenkstellungen wird auf Zwischenkontakte und Kannen-Hüllkörper geprüft. **Fehlende Bildmessung, schlechte Reprojektion oder ein verbotener Bewegungsweg sperrt das Nachfahren; es gibt keinen Ground-Truth-Fallback.**
5. In einem **separaten Bewertungsbericht** vergleichen wir die Kamera- und FR5-Ergebnisse mit der im Simulator bekannten Originalbewegung.

**Automatisch verifiziert (3 virtuelle Kameras, 13 Frames, Seed 14):**
- **13/13** optische Kännchenposen aus mindestens zwei Kameras gemessen
- **13/13** Positionen mit den sechs echten Gelenken nachgestellt, jeder zurückgelegte Gelenkabschnitt mit Zwischenprüfungen
- maximaler Kamerafehler gegenüber gehaltenem Simulator-Wahrheitswert: **0,924 mm** und **0,339°**; bedeutet **nicht** 0,9-mm-Genauigkeit mit physischer Hardware
- Das nachgefahrene FR5-Ziel stammt **nur aus den Kamerabildern**, nicht aus den Quellgelenkwinkeln oder versteckten Originalposen.

**Zusätzlicher Szenenabgleich:** Im ersten Video-Frame werden auch die **acht stationären Referenz-AprilTags** direkt aus den gerenderten RGB-Pixeln rekonstruiert und gegen die fest im Modell bekannten Positionen ausgewertet. **8/8 erkannt, maximale Differenz 2,784 mm**. Das ist noch **kein Kalibrieren unbekannter externer realer Kameras** – der virtuelle Kamera→Welt-Transform ist exakt bekannt.

**Filme / Details:**
- [Colab: Kamera→FR5-Film und Messdiagramme](https://colab.research.google.com/github/Coffee-Tobi/robocasa/blob/main/COLAB_SAFE_REPLAY.ipynb)
- [Kamerapose-Programm](../robobarista/camera_joint_replay.py)
- [Kamerageometrie / AprilTag-Bildrekonstruktion](../robobarista/geometry.py)

## Warum stehen in der virtuellen Szene X-Marker, wenn wir AprilTags verwenden?

Das sind zwei Dinge:

**X/Label im Simulator**: Ein Anzeigehinweis kennt eine Weltkoordinate bereits aus der MJCF-Szene. Er prüft **nicht**, ob ein physischer Kamerapixel diese Stelle identifizieren kann.

**AprilTag-Detektion**: Die Kameras sehen RGB-Pixel mit Markerquadraten. Deren sichtbare Ecken werden gemessen, identifiziert und mit Objekt-Tag-Geometrie trianguliert / in einer Reprojektion optimiert. Daraus entstehen **beobachtete Weltkoordinaten** und Richtungen mit einem Fehler, der gegen die getrennte Simulatorwahrheit prüfbar ist.

**Muss man die ganze Szene neu aus den Bildern konstruieren?** **Nein**. Man behält eine einmalig definierte und kalibrierte CAD/MuJoCo-Arbeitszelle als **Weltreferenz**, prüft die Lage der festen Referenzmarkierungen und erkennt nur die beweglichen oder möglicherweise verstellten Objekte pro Frame neu. Später müssen unbekannte reale Kameraparameter und Kamera→Roboter-Basistransformation aus einem Kalibrierverfahren bestimmt werden, statt die perfekt bekannten virtuellen MuJoCo-Kameraparameter zu übernehmen.

In unserer neuen Standardzelle gibt es mehrere **feste** Stations- und **bewegliche** Kännchen-/Becher-Tags. Für die randomisierte Demonstration verwenden wir zwei Kännchen-Tags **#71** und **#72** mit bekannten Starrkörperpositionen; der seitliche #23 ist als potenzielle zusätzliche Perspektive enthalten. Der zweite oben schräg montierte Marker verbessert die Sicht aus den hoch-/seitlich montierten Kameras.

Die feste Arbeitszelle wird **nicht** per Neuberechnung komplett ersetzt. Sie fungiert als **Kartenkoordinatensystem**. Kamera-Messungen bestimmen neu, *wo sich Kanne und Becher gerade befinden*. Das ist der wesentliche Unterschied zwischen **wissen** und **messen**.

## Code-Ablauf in 20 Zeilen

```python
# Quelle: seeded Zufallsbewegung, IK-/Kontakt- und Nutzlastkollision-geprüft
q0, _ = _find_start(model, data, qids, limits, seed)
source_q, rejected = _random_joint_motion(model, data, qids,
                                            limits, q0, seed, frames)

# Nur Bilddaten in die Schätzung:
rgb, depth = render_pair(renderer, data, camera_name)
detected = detect_tags(rgb, include_apriltags=True, detector='pupil')
tag_pose = solve_single(pixel_corners, depth, calibration, known_tag_size)
pose = bundle_adjust_object(observations, TAG_DEFINITIONS, SIDE_LENGTHS)

# Ab hier AUSSCHLIESSLICH die Kamera-Schätzung als Zielwert:
p = np.asarray(camera_record['position_m'])
R = Rotation.from_quat(camera_record['rotation_xyzw']).as_matrix()
sol = compute_ik(model, data, tcp, qids, limits, p, R, previous_q)
if not sol['solved']:
    raise UnsafeMotionError('Keine gueltige IK-Pose')
allowed, reason, substeps = _safe_q_segment(model, data, qids,
                                            limits, previous_q, sol['q'])
if not allowed:
    raise UnsafeMotionError(reason)
_write_q(model, data, qids, sol['q'])  # NUR kinematische Visualisierung
```

**Zeit/Flüssigkeit:** Die virtuellen Zeitstempel sind `frame/fps`. Zwischen zufälligen Gelenk-Wegpunkten verwenden wir eine glatte `smoothstep`-Interpolation. Die simulierten **Gelenkstellungen** werden jedoch kinematisch gesetzt; `mj_step` mit realistischen Gelenkmotoren, Drehmomenten, Geschwindigkeits- und Beschleunigungsgrenzen ist in diesem Schritt **nicht** implementiert. Deshalb ist auch die Zuordnung „zum richtigen Zeitpunkt am richtigen Ort“ noch kein motorisch bestätigtes Ergebnis.

## Nächste konkrete Arbeit

- Die verbleibenden letzten Zentimeter von den sicheren Vorpositionen zur Mühle, Brühgruppe, Dampflanze und Milchspender-Düse als **einzelne task-abhängige mechanische Kontaktphasen** mit echtem Geräte-CAD modellieren.
- Den FK-TCP-Punkt zur wirklichen Siebträger- und Milchkannenmontage vermessen; die Kanne bislang nur als konservatives Hüllkörper-Modell und visuelles Objekt.
- Die Bewegung in MuJoCo mit **`mj_step` und echten FR5-Motoraktuatoren** reproduzieren, dadurch Zeit, Geschwindigkeiten, Beschleunigungen, Kräfte und mögliche Abweichungen prüfen.
- Stationäre AprilTag-Bildregistrierung gegen die bekannte Zellreferenz prüfen und eine echte Kamera-zu-Roboter-Kalibrierung vorbereiten.
- Später: menschliche Demonstrationsaufnahme statt des seedbasierten künstlichen Zufallswegs.

## Neues integriertes Haupt-Colab – alle vier Schritte in einem Ablauf

**[COLAB_ROBOBARISTA.ipynb](../COLAB_ROBOBARISTA.ipynb)** ist jetzt der empfohlene Einstieg. Es richtet die gemeinsame MuJoCo-Kaffeeinsel ein, rendert die Markerprüfung und sieben Maschinenstationen, berechnet freie pre-dock-Anfahrten, führt danach den beschriebenen drei-Kamera-Versuch durch und simuliert schließlich eine **dynamische motorische Ausführung** derselben kameragemessenen Gelenkbahn. Die Daten bleiben getrennt, sämtliche MP4s und JSON-Dateien gehen in ein gemeinsames ZIP.

**Motorische Versuchsregelung in [`motor_follow.py`](../robobarista/motor_follow.py):** Die bereits aus AprilTag-Pixeln gewonnene Sollbewegung wird mittels fünfter-Ordnung-Interpolation `10u³−15u⁴+6u⁵` und vorsichtiger Zeitstreckung um Faktor 2,5 verlangsamt. Ein PD-Regler verfolgt die sechs Gelenkwinkel und Geschwindigkeiten; die statische Schwerkraftkompensation wird mit `mujoco.mj_inverse` berechnet. Jeder der sechs FR5-Motoraktuatoren wird innerhalb der im Modell eingetragenen Drehmomentgrenzen angesteuert; `mujoco.mj_step()` berechnet die tatsächliche Bewegung. Kontakte von FR5 und vereinfachte Kollisionsabstände der gesamten mitgetragenen Kanne werden laufend überprüft. Anders als im kinematischen Vergleich können tatsächliche und gewünschte Gelenkpositionen **auseinanderliegen**.

**Getesteter virtueller Motorlauf:** **6,7 s simulierte Zeit**, 41 Videobilder; maximale Abweichung der tatsächlichen Kännchenposition von der aus dem Soll-Gelenkwinkel berechneten Position **12,36 mm**, mittlere **4,46 mm**. Keine gemeldeten Kontakt-/Abstandsverletzungen oder Drehmomentsättigung im Test. **Diese Werte sind zu unpräzise für echte Latte Art und vorerst nur ein Nachweis funktionierender Motorphysik!** Die Modellbeladung entspricht noch nicht einer echten gefüllten Kanne, der virtuelle Greifer ist nicht physikalisch fest angekoppelt und der PD-Regler nicht sicherheitsqualifiziert. Die motorische zeitliche Abfolge ist dadurch untersuchbar, aber **nicht** für eine echte Maschine validiert.

**Qualitätskontrolle:** [Kompletter automatischer Test von allen vier Colab-Schritten](https://github.com/Coffee-Tobi/robocasa/actions/workflows/complete-colab.yml), getrenntes [Motor-Folgeexperiment](https://github.com/Coffee-Tobi/robocasa/actions/workflows/motor-follow.yml). Beide erfordern keine lokale Hardware.

