# RoboBarista – Projekttracker und Roadmap

> **Status:** Startfassung, 20.10.2026. **Quelle:** unsere Projektplanung. Noch keine Erledigungsstände überprüft.  
> **Ziel:** Ein visueller, lernbasierter, anpassungsfähiger Roboter, der Latte Art eingießt; spätere Espresso- und Milchaufschäum-Funktionen separat.  
> **Wichtig:** Diese Datei ist der *vorläufige* Aufgaben-Tracker, da **Issues im GitHub-Repository deaktiviert** sind. Eine CSV steht für den Import in Linear oder einen anderen Projekttracker bereit.  
> **Branch:** `barista-dev`. **Das Trainieren eines end-to-end black-box VLA ist ausdrücklich keine Pflicht.**

## Bedienung und Status

- `[ ]` = offen, `[x]` = erledigt. In den Aufgabendetails kann jedes Kriterium separat abgehakt werden.
- `P0` = zuerst/unverzichtbar für MVP · `P1` = danach · `P2` = spätere Ausbaustufe.
- **Schwierigkeit 1–5:** grobe Einschätzung, unabhängig von Reihenfolge und Priorität.
- **Abhängigkeiten** sind RB-IDs. Die Aufgaben sind *nicht* chronologisch vollständig durchzuführen; parallelisierbare Arbeit ist möglich.
- Bei Beginn einer Aufgabe als Besitzer, Ergebnislink, Datum und Status (`Backlog / In Arbeit / Blockiert / Fertig`) in der CSV nachführen.
- Bewusst **kein** 900-Schritte-RoboCasa-Rollout als Voraussetzung für die ersten Videotests.

## Meilensteine

| Reihenfolge | Meilenstein | Nachweis |
|---|---|---|
| M0 | Projektgrundlage | Einengung auf eingießende Latte Art, überprüfbare Ziele, technische Architektur |
| M1 | Bildanalyse | Kännchen/Tasse/Ausgießer in Video verfolgt, Erkennung sichtbar |
| M2 | 3D-Erfassung | Position und Winkel mit kalibrierten Kameras geschätzt |
| M3 | Bewegungslernen | Gelernte, veränderbare Eingießbewegung aus Demonstrationen |
| M4 | Robotermechanik | Einarm-Roboter reproduziert gelernte Bewegung in MuJoCo |
| M5 | Flüssigkeit & Muster | Erst Strahlmodell, danach empirisches/physikalisches Musterfeedback |
| M6 | Adaptive Steuerung | Schnell messbare Korrektur bei Positionsänderungen |
| M7 | Integration | Bewertbare virtuelle Latte Art; Hardware und Restworkflow später |

## Empfohlener Start

**Zuerst RB-01, RB-02 und RB-05.** Danach RB-06 → RB-07 → RB-09 (frühe visuelle Überprüfbarkeit).  
**Geschwindigkeit parallel:** RB-31 kann früh starten, sobald die Datenschnittstellen feststehen.  
**Für Latte Art zuerst:** die Schritte zum **Eingießen** priorisieren; **Milchaufschäumen und Espressomaschine später**.

## Gesamtüberblick

| ID | Aufgabe | Meilenstein | Priorität | Schwierigkeit | Status | Abhängigkeiten |
|---|---|---|---|---:|---|---|
| [RB-01](#rb-01) | MVP-Umfang: zunächst Latte Art eingießen, ein Arm, geneigte feste Tasse | M0 | P0 | 1/5 | Backlog | – |
| [RB-02](#rb-02) | Messbare Erfolgskriterien für Wahrnehmung, Latte Art und Robotik | M0 | P0 | 2/5 | Backlog | RB-01 |
| [RB-03](#rb-03) | Systemarchitektur und gemeinsame Datenformate planen | M0 | P0 | 2/5 | Backlog | RB-01 |
| [RB-04](#rb-04) | Open-Source-Auswahl, Datenlizenzen und Reproduzierbarkeit prüfen | M0 | P1 | 1/5 | Backlog | RB-01 |
| [RB-05](#rb-05) | Geeignete Latte-Art-Demonstrationsvideos zusammentragen | M1 | P0 | 1/5 | Backlog | RB-01 |
| [RB-06](#rb-06) | Erstes Video und Kamera-Perspektive als Referenz definieren | M1 | P0 | 1/5 | Backlog | RB-05 |
| [RB-07](#rb-07) | Tasse und Milchkännchen im Video segmentieren und verfolgen | M1 | P0 | 2/5 | Backlog | RB-06 |
| [RB-08](#rb-08) | Ausgießerspitze und Tassenrand im Bild verfolgen | M1 | P0 | 3/5 | Backlog | RB-07 |
| [RB-09](#rb-09) | Wahrnehmung als Overlay und Zeitleiste darstellen | M1 | P0 | 1/5 | Backlog | RB-07 |
| [RB-10](#rb-10) | Kleinen annotierten Testdatensatz und Auswertung erstellen | M1 | P1 | 2/5 | Backlog | RB-07 |
| [RB-11](#rb-11) | Milchstrahl und Auftreffpunkt auf der Kaffeeoberfläche erkennen | M1 | P1 | 4/5 | Backlog | RB-08, RB-10 |
| [RB-12](#rb-12) | Schaumoberfläche, Füllstand und Latte-Art-Konturen analysieren | M1 | P1 | 4/5 | Backlog | RB-10 |
| [RB-13](#rb-13) | Qualitäts- und Unsicherheitstest der Kamerawahrnehmung | M1 | P1 | 3/5 | Backlog | RB-08, RB-11 |
| [RB-14](#rb-14) | Kamerazahl und Positionierung für räumliche Messung festlegen | M2 | P0 | 2/5 | Backlog | RB-08 |
| [RB-15](#rb-15) | Kameras kalibrieren und Koordinatensysteme verknüpfen | M2 | P0 | 3/5 | Backlog | RB-14 |
| [RB-16](#rb-16) | 3D-Pose von Tasse und Ausgießer rekonstruieren | M2 | P0 | 4/5 | Backlog | RB-08, RB-15 |
| [RB-17](#rb-17) | Becherwinkel, Oberfläche und Ausgießerabstand erfassen | M2 | P1 | 3/5 | Backlog | RB-11, RB-16 |
| [RB-18](#rb-18) | Aus Videos Bewegungsbahnen mit Zeitbezug extrahieren | M3 | P0 | 3/5 | Backlog | RB-16 |
| [RB-19](#rb-19) | Neigung, Geschwindigkeit und Beschleunigung der Kanne lernen | M3 | P0 | 3/5 | Backlog | RB-17, RB-18 |
| [RB-20](#rb-20) | DMP-Basismodell einer Latte-Art-Eingießbewegung trainieren | M3 | P0 | 3/5 | Backlog | RB-18, RB-19 |
| [RB-21](#rb-21) | Gelernte Bewegung auf neue Startpositionen und Tempi übertragen | M3 | P1 | 4/5 | Backlog | RB-20 |
| [RB-22](#rb-22) | Alternativen zu DMPs vergleichbar testen | M3 | P2 | 4/5 | Backlog | RB-20 |
| [RB-23](#rb-23) | Geeigneten Einarm-Roboter mit Greifer und Arbeitsraum wählen | M4 | P0 | 2/5 | Backlog | RB-01, RB-19 |
| [RB-24](#rb-24) | Geneigten festen Becherhalter als MVP definieren | M4 | P0 | 2/5 | Backlog | RB-17, RB-23 |
| [RB-25](#rb-25) | Minimale Simulation: Roboter, Kännchen, Tasse ohne Flüssigkeit | M4 | P0 | 3/5 | Backlog | RB-23, RB-24 |
| [RB-26](#rb-26) | Gelernte Ausgießertrajektorien durch inverse Kinematik ausführen | M4 | P0 | 4/5 | Backlog | RB-20, RB-25 |
| [RB-27](#rb-27) | Kollisionen, Reichweite, Tassen- und Griffgeometrie prüfen | M4 | P1 | 3/5 | Backlog | RB-26 |
| [RB-28](#rb-28) | Vereinfachtes Modell des Milchstrahls und Auftreffpunkts bauen | M5 | P1 | 3/5 | Backlog | RB-17, RB-25 |
| [RB-29](#rb-29) | Dynamisches Modell für Musterentwicklung aus Videodaten untersuchen | M5 | P1 | 4/5 | Backlog | RB-11, RB-12, RB-28 |
| [RB-30](#rb-30) | Physikbasierten Milch-/Schaum-/Crema-Simulator nur bei Bedarf evaluieren | M5 | P2 | 5/5 | Backlog | RB-28, RB-29 |
| [RB-31](#rb-31) | Geschwindigkeitsprofil für KI, Kameras, Rendering und Physik erstellen | M6 | P0 | 2/5 | Backlog | RB-03 |
| [RB-32](#rb-32) | Adaptive geschlossene Regelung anhand visueller Positionsfehler | M6 | P0 | 4/5 | Backlog | RB-16, RB-26, RB-31 |
| [RB-33](#rb-33) | Schnelle wiederholte Replanung und Sensorkontrolle | M6 | P1 | 4/5 | Backlog | RB-32 |
| [RB-34](#rb-34) | Live-Latte-Art-Feedback in Bewegungskorrekturen übersetzen | M6 | P1 | 4/5 | Backlog | RB-11, RB-12, RB-29, RB-32 |
| [RB-35](#rb-35) | Robustheit gegen Startpositions- und Kameraveränderungen testen | M7 | P1 | 4/5 | Backlog | RB-32, RB-33 |
| [RB-36](#rb-36) | Gelerntes Gießen mit visueller Rückmeldung integrieren | M7 | P1 | 4/5 | Backlog | RB-26, RB-28, RB-34 |
| [RB-37](#rb-37) | Übertragung auf reale Kamera und realen Roboter vorbereiten | M7 | P2 | 4/5 | Backlog | RB-35, RB-36 |
| [RB-38](#rb-38) | Milch aufschäumen und Schaumqualität bewerten (späterer Teil) | M7 | P2 | 5/5 | Backlog | RB-36 |
| [RB-39](#rb-39) | Espressomaschine und Mühle bedienen (späterer Teil) | M7 | P2 | 4/5 | Backlog | RB-36 |
| [RB-40](#rb-40) | Dampflanze reinigen, Fehler erkennen und Arbeitsabläufe koordinieren | M7 | P2 | 3/5 | Backlog | RB-38, RB-39 |
| [RB-41](#rb-41) | Ergebnisse, Modelle, Videos und Tests reproduzierbar archivieren | M7 | P1 | 2/5 | Backlog | RB-03, RB-10 |

## Aufgaben und Abnahmekriterien

### M0 – Projektgrundlage

<a id="rb-01"></a>
#### RB-01 · MVP-Umfang: zunächst Latte Art eingießen, ein Arm, geneigte feste Tasse

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** keine
- **Open Source / Tools (Kandidaten):** Markdown / GitHub
- [ ] **Abnahme:** MVP und bewusst ausgeschlossene Funktionen schriftlich festgelegt
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-02"></a>
#### RB-02 · Messbare Erfolgskriterien für Wahrnehmung, Latte Art und Robotik

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Open Source / Tools (Kandidaten):** OpenCV
- [ ] **Abnahme:** Messgrößen für Treffpunkt, Musterqualität, Pose, Erfolgsquote und Latenz dokumentiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-03"></a>
#### RB-03 · Systemarchitektur und gemeinsame Datenformate planen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Open Source / Tools (Kandidaten):** ROS 2 / Python / Rerun
- [ ] **Abnahme:** Schnittstellen für Frame, Zeitstempel, Pose, Aktionen und Vorhersage dokumentiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-04"></a>
#### RB-04 · Open-Source-Auswahl, Datenlizenzen und Reproduzierbarkeit prüfen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Open Source / Tools (Kandidaten):** GitHub / Hugging Face
- [ ] **Abnahme:** Für jede ausgewählte Bibliothek Version, Lizenz, Grenzen und Quelle vermerkt
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


### M1 – Bildgebung, Tracking und Visualisierung

<a id="rb-05"></a>
#### RB-05 · Geeignete Latte-Art-Demonstrationsvideos zusammentragen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** [RB-01](#rb-01)
- **Open Source / Tools (Kandidaten):** ffmpeg / yt-dlp nur mit passenden Nutzungsrechten
- [ ] **Abnahme:** Mindestens drei rechtmäßig nutzbare Aufnahmen mit gut sichtbarem Kännchen und Becher ausgewählt
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-06"></a>
#### RB-06 · Erstes Video und Kamera-Perspektive als Referenz definieren

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** [RB-05](#rb-05)
- **Open Source / Tools (Kandidaten):** OpenCV / ffmpeg
- [ ] **Abnahme:** Ein Referenzvideo mit FPS, Auflösung, Perspektive und Frame-Beispielen dokumentiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-07"></a>
#### RB-07 · Tasse und Milchkännchen im Video segmentieren und verfolgen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-06](#rb-06)
- **Open Source / Tools (Kandidaten):** SAM 2 / OpenCV
- [ ] **Abnahme:** Beide Objekte über ein durchgehendes Testvideo mit visualisierter Maske verfolgt
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-08"></a>
#### RB-08 · Ausgießerspitze und Tassenrand im Bild verfolgen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-07](#rb-07)
- **Open Source / Tools (Kandidaten):** OpenCV / SAM 2 / Keypoint-Modell
- [ ] **Abnahme:** Pro Frame Position und Konfidenz der Ausgießerspitze und des Tassenrandes gespeichert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-09"></a>
#### RB-09 · Wahrnehmung als Overlay und Zeitleiste darstellen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 1/5
- **Abhängigkeiten:** [RB-07](#rb-07)
- **Open Source / Tools (Kandidaten):** Rerun
- [ ] **Abnahme:** Video, erkannte Objekte, Positionswerte und Zeitstempel synchron sichtbar
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-10"></a>
#### RB-10 · Kleinen annotierten Testdatensatz und Auswertung erstellen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-07](#rb-07)
- **Open Source / Tools (Kandidaten):** CVAT / Label Studio
- [ ] **Abnahme:** Trainings- und Testszenen getrennt; definierte Metriken und Beispielannotationsdatei vorhanden
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-11"></a>
#### RB-11 · Milchstrahl und Auftreffpunkt auf der Kaffeeoberfläche erkennen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-08](#rb-08), [RB-10](#rb-10)
- **Open Source / Tools (Kandidaten):** OpenCV / SAM 2 / eigenes kleines Segmentierungsmodell
- [ ] **Abnahme:** Treffpunkt bei geeigneten Videos mit überprüfter Unsicherheit frameweise sichtbar
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-12"></a>
#### RB-12 · Schaumoberfläche, Füllstand und Latte-Art-Konturen analysieren

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-10](#rb-10)
- **Open Source / Tools (Kandidaten):** OpenCV / Bildsegmentierung
- [ ] **Abnahme:** Weißes Muster, Tassenfüllstand und Entwicklung über Zeit werden nachvollziehbar angezeigt
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-13"></a>
#### RB-13 · Qualitäts- und Unsicherheitstest der Kamerawahrnehmung

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-08](#rb-08), [RB-11](#rb-11)
- **Open Source / Tools (Kandidaten):** OpenCV / eigene Tests
- [ ] **Abnahme:** Fehlende Sicht, Verdeckung, Reflexion und Tracking-Fehler werden erkannt statt still geschätzt
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


### M2 – Kamerageometrie und 3D-Verständnis

<a id="rb-14"></a>
#### RB-14 · Kamerazahl und Positionierung für räumliche Messung festlegen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-08](#rb-08)
- **Open Source / Tools (Kandidaten):** OpenCV / Kamera-Geometrie
- [ ] **Abnahme:** Konkretes Layout (Draufsicht plus Seitenansicht, optional Handkamera) mit Sichtfeldbegründung
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-15"></a>
#### RB-15 · Kameras kalibrieren und Koordinatensysteme verknüpfen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-14](#rb-14)
- **Open Source / Tools (Kandidaten):** OpenCV ArUco / Kalibr
- [ ] **Abnahme:** Transformationskette Kamera → Tasse → Roboter mit Kalibrierfehler dokumentiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-16"></a>
#### RB-16 · 3D-Pose von Tasse und Ausgießer rekonstruieren

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-08](#rb-08), [RB-15](#rb-15)
- **Open Source / Tools (Kandidaten):** OpenCV / Triangulation / PnP
- [ ] **Abnahme:** 3D-Position, Orientierung und Unsicherheit in mehreren bekannten Testszenen überprüft
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-17"></a>
#### RB-17 · Becherwinkel, Oberfläche und Ausgießerabstand erfassen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-11](#rb-11), [RB-16](#rb-16)
- **Open Source / Tools (Kandidaten):** OpenCV / Geometrie
- [ ] **Abnahme:** Becherneigung, Oberflächenhöhe und Ausgießerabstand werden pro Frame geschätzt
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


### M3 – Bewegungen aus Demonstrationen lernen

<a id="rb-18"></a>
#### RB-18 · Aus Videos Bewegungsbahnen mit Zeitbezug extrahieren

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-16](#rb-16)
- **Open Source / Tools (Kandidaten):** OpenCV / SciPy
- [ ] **Abnahme:** Reproduzierbare Sequenz aus Ausgießerpose und Zeitstempeln liegt vor
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-19"></a>
#### RB-19 · Neigung, Geschwindigkeit und Beschleunigung der Kanne lernen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-17](#rb-17), [RB-18](#rb-18)
- **Open Source / Tools (Kandidaten):** SciPy / NumPy
- [ ] **Abnahme:** Zeitliche Kinematik inklusive Kannenwinkel mit Plausibilitätsgrafiken rekonstruiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-20"></a>
#### RB-20 · DMP-Basismodell einer Latte-Art-Eingießbewegung trainieren

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-18](#rb-18), [RB-19](#rb-19)
- **Open Source / Tools (Kandidaten):** dfki-ric/movement_primitives
- [ ] **Abnahme:** Gelernte Bewegung im virtuellen Kännchen ohne manuell programmierte Trajektorie reproduziert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-21"></a>
#### RB-21 · Gelernte Bewegung auf neue Startpositionen und Tempi übertragen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-20](#rb-20)
- **Open Source / Tools (Kandidaten):** Movement Primitives
- [ ] **Abnahme:** Startpositionen, Zielpositionen und Tempo variieren ohne komplette Neudemonstration
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-22"></a>
#### RB-22 · Alternativen zu DMPs vergleichbar testen

- **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-20](#rb-20)
- **Open Source / Tools (Kandidaten):** LeRobot / Diffusion Policy / ACT
- [ ] **Abnahme:** Vergleich von Datendurst, Adaptivität, Interpretierbarkeit, Qualität und Latenz dokumentiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


### M4 – Robotik und mechanische Umsetzung

<a id="rb-23"></a>
#### RB-23 · Geeigneten Einarm-Roboter mit Greifer und Arbeitsraum wählen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-01](#rb-01), [RB-19](#rb-19)
- **Open Source / Tools (Kandidaten):** MuJoCo / MoveIt 2 / URDF
- [ ] **Abnahme:** Robotermodell erfüllt notwendige Reichweite, Kännchenlast und Orientierung
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-24"></a>
#### RB-24 · Geneigten festen Becherhalter als MVP definieren

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-17](#rb-17), [RB-23](#rb-23)
- **Open Source / Tools (Kandidaten):** MuJoCo / CAD optional
- [ ] **Abnahme:** Neigungswinkel und Position variierbar; ein Arm muss nur Kännchen bewegen
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-25"></a>
#### RB-25 · Minimale Simulation: Roboter, Kännchen, Tasse ohne Flüssigkeit

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-23](#rb-23), [RB-24](#rb-24)
- **Open Source / Tools (Kandidaten):** MuJoCo / robosuite
- [ ] **Abnahme:** Reproduzierbare Szene rendert Kameras und erlaubt physikalische Roboterbewegungen
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-26"></a>
#### RB-26 · Gelernte Ausgießertrajektorien durch inverse Kinematik ausführen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-20](#rb-20), [RB-25](#rb-25)
- **Open Source / Tools (Kandidaten):** MoveIt 2 / MuJoCo IK
- [ ] **Abnahme:** Robotergreifer folgt Position und Orientierung der gelernten Bahn mit gemessenem Fehler
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-27"></a>
#### RB-27 · Kollisionen, Reichweite, Tassen- und Griffgeometrie prüfen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-26](#rb-26)
- **Open Source / Tools (Kandidaten):** MoveIt 2 / MuJoCo
- [ ] **Abnahme:** Sicherer Abstand und erreichbare Orientierungen bei variablen Becherlagen getestet
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


### M5 – Milchstrahl, Milchschaum und Flüssigkeitsmodelle

<a id="rb-28"></a>
#### RB-28 · Vereinfachtes Modell des Milchstrahls und Auftreffpunkts bauen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-17](#rb-17), [RB-25](#rb-25)
- **Open Source / Tools (Kandidaten):** MuJoCo + Geometrie / SciPy
- [ ] **Abnahme:** Strahlrichtung, Ausgießerhöhe, Treffpunkt und einfache Flussrate nachvollziehbar berechenbar
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-29"></a>
#### RB-29 · Dynamisches Modell für Musterentwicklung aus Videodaten untersuchen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-11](#rb-11), [RB-12](#rb-12), [RB-28](#rb-28)
- **Open Source / Tools (Kandidaten):** PyTorch / OpenCV
- [ ] **Abnahme:** Vorhersage nächster Musterveränderungen auf zuvor ungesehenen Beispielen evaluiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-30"></a>
#### RB-30 · Physikbasierten Milch-/Schaum-/Crema-Simulator nur bei Bedarf evaluieren

- **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 5/5
- **Abhängigkeiten:** [RB-28](#rb-28), [RB-29](#rb-29)
- **Open Source / Tools (Kandidaten):** SPlisHSPlasH / ggf. externe CFD
- [ ] **Abnahme:** Machbarkeitsvergleich von Physiksimulation und datengetriebenem Ersatzmodell inklusive Rechenzeit
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


### M6 – Echtzeit und adaptive Regelung

<a id="rb-31"></a>
#### RB-31 · Geschwindigkeitsprofil für KI, Kameras, Rendering und Physik erstellen

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-03](#rb-03)
- **Open Source / Tools (Kandidaten):** Python perf_counter / torch profiler / Rerun
- [ ] **Abnahme:** Latenz pro Komponente separat und wiederholbar gemessen, nicht nur Gesamtzeit
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-32"></a>
#### RB-32 · Adaptive geschlossene Regelung anhand visueller Positionsfehler

- **Status:** Backlog · **Priorität:** P0 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-16](#rb-16), [RB-26](#rb-26), [RB-31](#rb-31)
- **Open Source / Tools (Kandidaten):** MoveIt Servo / ROS 2
- [ ] **Abnahme:** Verschobene Tasse führt bei gleichem gelernten Ziel zu korrekter Bahnkorrektur
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-33"></a>
#### RB-33 · Schnelle wiederholte Replanung und Sensorkontrolle

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-32](#rb-32)
- **Open Source / Tools (Kandidaten):** ROS 2 / asynchrones Policy-Serving
- [ ] **Abnahme:** Messbare Update-Frequenz und Ende-zu-Ende-Latenz mit Zielwert dokumentiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-34"></a>
#### RB-34 · Live-Latte-Art-Feedback in Bewegungskorrekturen übersetzen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-11](#rb-11), [RB-12](#rb-12), [RB-29](#rb-29), [RB-32](#rb-32)
- **Open Source / Tools (Kandidaten):** Rerun / PyTorch / MoveIt Servo
- [ ] **Abnahme:** Abweichender Auftreffpunkt oder Muster führt zu nachvollziehbarer Anpassung der Bewegung
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


### M7 – Integration, Validierung und spätere Erweiterung

<a id="rb-35"></a>
#### RB-35 · Robustheit gegen Startpositions- und Kameraveränderungen testen

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-32](#rb-32), [RB-33](#rb-33)
- **Open Source / Tools (Kandidaten):** MuJoCo / pytest / Rerun
- [ ] **Abnahme:** Messbare Erfolgsquote über viele Seeds, Positionen und Sichtbedingungen
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-36"></a>
#### RB-36 · Gelerntes Gießen mit visueller Rückmeldung integrieren

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-26](#rb-26), [RB-28](#rb-28), [RB-34](#rb-34)
- **Open Source / Tools (Kandidaten):** Python / MuJoCo / Rerun
- [ ] **Abnahme:** Kompletter virtueller Eingießdurchgang inkl. Wahrnehmung, Korrektur und Resultatbewertung
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-37"></a>
#### RB-37 · Übertragung auf reale Kamera und realen Roboter vorbereiten

- **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-35](#rb-35), [RB-36](#rb-36)
- **Open Source / Tools (Kandidaten):** ROS 2 / MoveIt 2 / OpenCV
- [ ] **Abnahme:** Sicherheitskonzept, reale Kalibrierung und Sim-to-Real-Risiken dokumentiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-38"></a>
#### RB-38 · Milch aufschäumen und Schaumqualität bewerten (späterer Teil)

- **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 5/5
- **Abhängigkeiten:** [RB-36](#rb-36)
- **Open Source / Tools (Kandidaten):** Computer Vision / Temperatur- und Drucksensorik
- [ ] **Abnahme:** Eigenes Subprojekt mit Dampfposition, Temperatur, Schaumbildung und Reinigung angelegt
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-39"></a>
#### RB-39 · Espressomaschine und Mühle bedienen (späterer Teil)

- **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 4/5
- **Abhängigkeiten:** [RB-36](#rb-36)
- **Open Source / Tools (Kandidaten):** LeRobot / robosuite / ROS 2
- [ ] **Abnahme:** Aufgabensequenz und Anforderungen für Mahlgut, Siebträger, Extraktion skizziert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-40"></a>
#### RB-40 · Dampflanze reinigen, Fehler erkennen und Arbeitsabläufe koordinieren

- **Status:** Backlog · **Priorität:** P2 · **Schwierigkeit:** 3/5
- **Abhängigkeiten:** [RB-38](#rb-38), [RB-39](#rb-39)
- **Open Source / Tools (Kandidaten):** Zustandsautomat / ROS 2
- [ ] **Abnahme:** Schrittfolge, Abschlusskriterien, Sicherheits- und Hygieneschritte definiert
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


<a id="rb-41"></a>
#### RB-41 · Ergebnisse, Modelle, Videos und Tests reproduzierbar archivieren

- **Status:** Backlog · **Priorität:** P1 · **Schwierigkeit:** 2/5
- **Abhängigkeiten:** [RB-03](#rb-03), [RB-10](#rb-10)
- **Open Source / Tools (Kandidaten):** GitHub / Hugging Face Datasets / DVC
- [ ] **Abnahme:** Jeder Test verlinkt Codeversion, Datenversion, Seed, Video und Metriken
- [ ] Ergebnis/Video/Code/Validierung verlinken und Status aktualisieren


## Entscheidungen und Risiken (laufend ergänzen)

| Entscheidung/Risiko | Vorläufiger Stand |
|---|---|
| Anfangsumfang | Nur Latte Art eingießen, nicht Aufschäumen oder Espresso |
| Armkonzept | Ein Arm am Kännchen; Tasse zunächst im festen, geneigten Halter |
| Perzeption | Explizit interpretierbare Messwerte und Overlay statt ausschließlich impliziter VLA-Merkmale |
| Lernansatz | DMP als kleine überprüfbare Basis, weitere Policies erst zum Vergleich |
| Flüssigkeiten | Keine Latte-Art-Funktion aus RoboCasa/MuJoCo ableiten; eigenes Modell erforderlich |
| Echtzeit | Gemessene Colab-T4-RoboCasa-Rollouts waren langsam; erst Physik vs. Rendering einzeln benchmarken |
| Quellcode | Frei verfügbare Tools priorisieren; Eignung, Versionen und Lizenzen einzeln prüfen |
| Offene Frage | Kann ein echtes Video mit genügend kalibrierten Perspektiven aufgenommen werden? |
| Offene Frage | Welche qualitative Latte-Art-Zieldefinition soll der erste Prototyp erfüllen? |

### Wann ein echter Issue-Tracker möglich ist

**Option A:** Auf GitHub im Repository `Settings → General → Features → Issues` aktivieren; dann können die RB-Aufgaben als eigene Issues angelegt werden.  
**Option B:** Linear mit ChatGPT verbinden; die unten stehende CSV eignet sich als Import- oder Migration-Basis.  
Die Markdown-Datei und die CSV sind **ab sofort nutzbar**; bisher wurden **keine GitHub Issues angelegt**.
