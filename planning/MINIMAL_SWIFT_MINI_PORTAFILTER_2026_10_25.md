# Neustart / Minimalversuch Swift Mini + Siebträger mit RoboBarista-Griff

**Stand 25.10.2026.** Kein FAIRINO FR5, keine RGB- oder AprilTag-Kameras,
keine weiteren Geräte. Echte MuJoCo-Kontaktdynamik zwischen zwei
vereinfachten Körpern, fest definiertem Druckanschlag und dem Siebträger.

## Herkunft / Abgrenzung

Swift Mini Gehäuse stammt aus unserem isolierten visuellen
`robobarista/swift_mini_visual_asset.py` mit **echter
Außenhülle 166 × 322 × 449 mm**, aber geschätzten Details.

Unser Siebträgergriff übernimmt die Idee und Hauptabmessungen von
`robobarista/dual_ir40_concept.py` und
`robobarista/dual_90_tool_attachments.py`:
**28 × 160 × 30 mm** rechteckiger IR40-50-freundlicher Griff, zusätzlich
zwei Anschläge/Indexrillen (nur visuell, nicht als echter IR40-Kontakt).
Basketdurchmesser und Haltergelenk nicht vom Hersteller vermessen.
Die originale FR5-Greiferbefestigung ist bewusst NICHT im isolierten
Modell enthalten.

### Anleitung entscheidend

La Marzocco Original Swift Mini Manual v1.2 Seite 8 / Abschnitt 8.2:
Siebträger sanft von unten einsetzen, **nur leichte Kraft**, niemals
so stark drücken, dass die Mühle auf der Arbeitsfläche verschoben wird.
**Rechtsdrehung/Einrastwinkel nicht als zwingender Schritt belegt.**

https://home.lamarzoccousa.com/wp-content/uploads/2020/08/lzDFNLH4rBBGwEovm9kSl.pdf

Quick Start Guide, Seite 2: Sanft Siebträger einsetzen, Starttaste
drücken, dann entfernen und anschließend **in der Espressomaschine
verriegeln** – das ist von der Mühle zu unterscheiden.

https://home.lamarzoccousa.com/wp-content/uploads/2020/08/Quick-Start-Guide-Swift-Mini.pdf

## Simulation

`robobarista/swift_mini_portafilter_only.py` stellt den
Siebträger in einem bewusst isolierten Arbeitsraum mit nur **zwei**
freiheitsgraden dar:

1. Vertikale Hub-Schiene Z unter der virtuellen Aufnahme, X/Y und
   Roll/Pitch sind ausdrücklich blockiert.
2. Drehgelenk Yaw um vertikale Z-Achse; bei positivem Winkel dreht
   der zum Bediener zeigende Griff nach rechts. Der Korb bleibt waagerecht.

Beide Gelenke werden mit echten virtuellen Position-Servos **mit
Kraft-/Drehmomentbegrenzung** und MuJoCo-`mj_step` geregelt,
nicht nur direkt in qpos abgespielt. Die Kontaktkräfte werden aus
MuJoCo-`mj_contactForce` berechnet.

- Einstecken: Modell-Kraftobergrenze **18 N**, aus Sicherheitsvorsicht
  frei gewählter **virtueller** Experimentwert, **kein** La Marzocco-Wert.
- Rechtsdrehung: **12°**, Drehmomentgrenze **1,6 Nm**.
  Hypothetischer Versuch; **keine** Original-Getriebe-/Verriegelungsgeometrie.
- Vergleichsversuch: nur sanft vertikal einsetzen und ohne Drehung halten.
- Fügemerkmal: ein provisorischer flacher Kontaktanschlag plus
  Korblippe. **Kein echtes Bajonett**, kein validierter Verriegelungspunkt.
- Kein Kaffee, kein externer Roboter, keine Griffreibung/Griffkraft.
- Die zwei Bildausschnitte im gerenderten Video sind reine virtuelle
  Beobachteransichten, **keine** Kameras/Sensoren im MuJoCo-Modell.

Die drei Phasen ALIGN, GENTLE_LIFT und CONTACT_HOLD werden gefolgt
von OPTIONAL_RIGHT_TWIST und HOLD.

**Ausgeführt in GitHub Actions (25.10.2026), erfolgreich:**
- 5/5 Struktur-/Geometrietests bestanden.
- Zwei real gerenderte MuJoCo-MP4s (je 68 Frames).
- Rechtsversuch: virtuelle Kontaktkraft maximal **2,523 N**,
  maximaler tatsächlicher Z-Gelenkhub **69,085 mm**,
  maximale berechnete Aufwärts-Servokraft **7,894 N**;
  Rechtsdrehung über 5° nachgewiesen.
- Vergleich ohne Rechtsdrehung: ebenfalls max. **2,523 N**
  virtuelle Reaktionskraft; kein Links-/Rechtsdrehen.
- XY-Ungenauigkeit **0 mm**, Korbneigung maximal **0,000001°**
  (bedingt durch ausschließlich Z-Hub/Yaw, kein realer Roboter-Test).
- Workflow: https://github.com/Coffee-Tobi/robocasa/actions/runs/40175762108
- Videos und JSON-Berichte: https://github.com/Coffee-Tobi/robocasa/actions/runs/40175762108/artifacts/12592540304

Die Kontaktflächen sind rotationssymmetrische Scheiben; der Test kann
**nur Anheben, begrenzten Druck und Rechts-Yaw demonstrieren**, nicht
wirkliche Bajonett-Verriegelung oder Freigabedrehmoment. Zahlen sind
Simulationsmodelleffekte, keine OEM-Spezifikationen.

## Wichtigste Grenze

Unsere alte Swift-Mini-Geometrie hat **keine exakte Position und Form
der Portafilter-Bajonettaufnahme**. Deshalb ist die Aufnahmekoordinate
ausdrücklich `UNCALIBRATED`. Selbst bei erfolgreichem Contact-Test
würde das **nicht** beweisen, dass ein echter Siebträger einrastet.
Für Herstellung: Original-STEP der Swift Mini oder Messschieber/
herstellerspezifische Greiferschnittstelle und tatsächliche
zulässige Montagekräfte ermitteln. Danach erst FR5-6D-IK, Anfahrt
und echte Kraft-/Drehmomentregelung aufbauen.

**Abgetrennter Branch:** `feature/swift-mini-portafilter-insert-minimal-2026-10-25`.
Kein Merge in die bisherige FR5-Arbeitszelle.
