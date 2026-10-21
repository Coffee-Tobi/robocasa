# RoboBarista – praktische Ideen aus Artly, Phase-DMP, MoveIt und Ruckig
**Stand 21.10.2026 · Recherche-/Integrationsnotiz zu unserem FAIRINO-FR5-Prototyp**

## Was aus externen Projekten besonders hilft

### Artly AI: Demonstrationen als Skills, nicht nur ein langer Positionsmitschnitt
Artly beschreibt sein Barista-System als eine durch menschliche Bewegungsdaten trainierte mehrstufige Skill-Bibliothek. Der Roboter kombiniert Mahlen, Brühen, Milchschaum und Gießen und kann situativ mithilfe der optischen Wahrnehmung korrigieren. Dies ist eine **Herstellerangabe**, keine hier unabhängig vermessene technische Umsetzung.
Quelle: https://www.artly.ai/applications/barista-bot

**Übertragen:** Wir haben eine feste Arbeitszelle, Andockrahmen und **11 einzeln überprüfbare Funktionsphasen** statt eines einzigen gigantischen Armprogramms. Später sollte jeder Schritt „Siebträger greifen / Mühle / Brühgruppe / Becher / Kanne / Milch / Düse / Giessen / Abstellen / Reinigen“ sein eigenes Start-/Ziel-, Werkzeug- und Kontaktprofil besitzen.

### One-Shot Learning for Double-Shot Latte Art: fünf getrennte Phasen
Khan, Hernandez Moreno und Le, arXiv **2610.16179** (12.–15.10.2026). Die Arbeit modelliert eine einmal demonstrierte Latte-Art-Bewegung in **fünf phasenspezifische kartesische Dynamic Movement Primitives (CDMPs)**. Die Autoren berichten bessere Übergangs- und Giessform-Fehler als bei einem einzigen globalen CDMP, einschließlich einer Winkelabweichung von unter 1° RMS während Gießphasen. **Das sind publizierte Ergebnisse der Autoren, nicht Ergebnisse unseres Roboters.**
Quelle: https://arxiv.org/abs/2610.16179

**Übertragen:** Unsere aktuelle **angenommene** Gießgeste teilt sich in fünf ausdrücklich **synthetische** Aufgabenphasen: *Approach, Base-fill, Lower, Sway, Cut-through*. Jede Phase verwendet nur einen kollisionsgeprüften Gelenkweg. Wir haben noch keine menschliche Demonstration gelernt und noch **keine CDMP-Bibliothek implementiert**. Später können wir aus Kamerademonstrationen echte CDMPs fitten. Für niedrige Kosten und mehr Präzision ist das sinnvoller als direkt eine allgemeine VLA zu trainieren.

### MoveIt Task Constructor: Aufgaben als überprüfbare Phasen
Die ROS-2-Bibliothek *MoveIt Task Constructor* strukturiert Greif- und Transportabläufe in einzelne Stages (Zielpose, IK, Grasp, Attach, Lift, Move, Place, Detach) und kombiniert sie mit Szenen-/Kollisionsänderungen.
Quellen: https://moveit.github.io/moveit_task_constructor/ und https://moveit.github.io/moveit_task_constructor/tutorials/pick-and-place.html

**Übertragen:** Wir können die Software-Architektur übernehmen, ohne für unseren einfachen Colab-Prototyp sofort ROS 2 zu installieren. Unser `coffee_workflow.py` behandelt **Maschinenvorpositionen** einzeln, verweigert unsichere Bewegungen und zeigt unimplementierte mechanische Übergaben als symbolisch. Der echte nächste Schritt ist das **Attach/Detach von realistischen Werkzeuggeometrien** und eng begrenzte mechanische Andockkontakte. Vorher darf keine reale Zubereitung behauptet werden.

### Ruckig: glattere Bewegungen mit Geschwindigkeits-, Beschleunigungs- und Jerk-Grenzen
Ruckig bietet lokale Berechnungen zwischen Zielzuständen. Die **Community-Mehrwegpunktfunktion** verwendet eine Cloud-API; Ruckig Pro bietet lokale Wegpunktplanung. 
Quelle: https://docs.ruckig.com/tutorial.html

**Übertragen:** Für unsere Forschungs-/Datenschutzziele verwenden wir **nur** `Ruckig(6)` und `Trajectory(6)` mit lokal gerechneten einzelnen Zustandsübergängen; nicht den cloudbasierten Wegpunktaufruf. Gemeinsame, teils nicht-null Geschwindigkeits-/Beschleunigungszustände verhindern künstliche Stopps an jedem Kameraframe. Die nachgeprüfte Testbewegung wird zeitlich auf die erforderliche, sicher begrenzte Dauer angepasst – bei Latte Art müssen wir das spätere Durchfluss-/Zeitverhalten separat mitdenken.

## Was im bestehenden Repo bereits ausprobiert ist

1. **Artly-inspirierte kompakte Arbeitszelle:** Mühle mit integriertem Tamper, FR5, Espressomaschine/Brühgruppe/Dampflanze/Becher, Milchspender, Kannenstand und Rinser.
2. **Drei RGB-D-Kameras:** 15 FPS bei 960×720; die 6D-Kanne wird aus AprilTag-Ecken geschätzt, nicht aus versteckten MuJoCo-Koordinaten. Die neue 3a-Auswertung liefert 100 Hz mathematische 6D-Kurven inkl. Ableitungen (das sind keine realen 100 FPS!).
3. **Lokale Ruckig-Bewegung:** Zehn verkettete, jerk-begrenzte Gelenkabschnitte ohne Halt an allen inneren Kameramesspunkten; Kollisionen der modellierten Roboter- und Kannengeometrien werden über Zwischenstellungen getestet. Die ursprüngliche 2-s-Kamerabewegung wurde im Beispiel auf **6 s** verlängert. Das ist eine andere zeitliche Ausführung.
4. **Komplette 11-stufige Kaffee-Choreografie:** Der FR5 führt die **erlaubten Vorpositionsfahrten** aus, Geräteaktionen und Flüssigkeiten bleiben vorläufig sichtbare **virtuelle Platzhalter**. Eine **angenommene fünfphasige Latte-Geste** bewegt das Handgelenk, ohne tatsächlichen Milchschaum zu formen.
5. **Arbeitsraum aus Kameras statt Modell:** Drei virtuelle RGB-D-Bilder werden zur sichtbaren XYZ-Punktwolke transformiert. Die acht festen AprilTag-Halter werden bildbasiert vermessen und **erst danach** gegen MuJoCo-Wahrheit verglichen. Dies ist keine unbekannt-kalibrierte SLAM-Lösung und erfasst keine verdeckten Gehäuseflächen.

## Die zwei größten technischen Risiken

**(A) Greif-/Gerätekontakte:** Wir haben im Prozess noch keine vollständige geometrische Integration für Siebträger-Bajonett, Tamperaufnahme, Milchkanne unter Düse, Brühgruppenverschraubung, echtes Aufschäumen oder Reinigungsdüse. Eine reine Vorpositionsfahrt beweist das nicht. Kritischer nächster Schritt: tool-spezifisches CAD, montierte Greif-Frames und genaue erlaubte Kontakte in der Maschine.

**(B) Flüssigkeits- und zeitliche Gießqualität:** Bisher sind Milch, Espresso und Latte-Konturen einfache farbige Attrappen. Eine realistische Latte-Art-Wirkung würde Durchfluss, Kannenrotation, Abstand zur Milchoberfläche, Milchschaumtextur und nötige zeitliche Abfolge benötigen. Eine auf sechs Sekunden verlängerte Geste ist nicht automatisch physikalisch dieselbe Gießbewegung wie zwei Sekunden.

Die meisten dieser Einschränkungen wären durch eine immer komplexere Attrappe **nicht** gelöst. Wir sollten echte kontaktfähige Werkzeug-Assets sowie mindestens eine gemessene demonstrierte Gießbewegung priorisieren und diese in klar benannte Phasen aufteilen.
