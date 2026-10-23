# Zielaufbau – 90°-Dual-Tool-Adapter + 6D F/T + zwei FAIRINO IR40-50

**23.10.2026. Recherche-/Designstand, KEIN freigegebenes mechanisches CAD.**
Dies ist die vom Nutzer gewünschte Architektur für einen möglichen FAIRINO FR5.
Diese Notiz ersetzt bewusst **nicht** die realen Adapter-/Sensor-STEP-Dateien,
und nimmt den Greifphysik-Dry-run nicht als Hardware-Beleg.

## Bestätigte Nutzerauswahl: Force Torque Sensor (Inlux)

Der Nutzer hat am 23.10.2026 **ausdrücklich klargestellt**, dass für
das 90°-Dual-Tool-Szenario **genau der Inlux Robotics "Force Torque Sensor"**
mit Artikelnummer **ITOSX1000FTS** gemeint ist, und **NICHT**
der bislang ebenfalls recherchierte TMC/GZCX-6F-75B-Sensor.

**Verbindlicher Referenzaufbau für künftige Planung**:
FAIRINO FR5 → Inlux ITOSX1000FTS 6-Achs Force Torque Sensor →
Inlux `ICACX1210XDUAL` Dual Cobot Tool Adapter (90°-Orientierung aus CAD
noch exakt zu prüfen) → **2 × Inlux IR40-50**. Die tatsächliche
mechanische Montage in dieser Dreierkombination wurde noch **nicht**
mit passenden STEP-Dateien oder einer Händlerfreigabe nachgewiesen.

Bestätigte Herstellerangaben ITOSX1000FTS:
Messbereich 0–200 N / 0–5 Nm; Genauigkeit ±0,1 N / ±0,01 Nm;
IP63; 5-Pin M12; für FR5 laut Inlux plug-and-play am Roboterflansch;
Datenblatt/2D-Zeichnung und 3D-STEP herunterladbar.
Listenpreis 885 EUR auf Herstellerseite, MwSt nach Rechnungsstatus.
https://inluxrobotics.eu/products/force-torque-sensor

**Elektrischer wichtiger Neufund**:
Der offizielle Inlux `ICACX1500GSCSTA` Standardsplitter
bestätigt **nur** die Paarungen (IR40-50 + IR40-50) und
(Force Torque Sensor + IR40-50). Für **alle drei zugleich** gibt
es in dieser Kompatibilitätsliste **keine Freigabe**. Eine dritte
RS485-/M12-Verbindung ist erst nach Abklärung von Busadressierung,
24-V-Strombudget und ggf. getrennter Sensorleitung zulässig.
https://inluxrobotics.eu/collections/grippers/products/gripper-splitter-cable

## 3D-CAD-Dateien – geprüfte Downloadverfügbarkeit (nicht importiert)

**Nicht behaupten, dass sie bereits eingebaut sind.** Der Nutzer wünscht
für alle Hauptteile das originale maßhaltige CAD; im bisherigen MuJoCo-Video
sind Sensor, 90°-Adapter und IR40-50 weiterhin selbst erstellte
Visualisierungsgeometrie. Die **aktuelle, in main freigegebene aktive MuJoCo-Vorschau** nutzt inzwischen zwei
orthogonale IR40-Achsen und die seitliche Anfahrt aus #21.
Lediglich das **Legacy-Hilfsmodell** `robobarista/dual_ir40_concept.py`
zeichnet nach wie vor zwei parallele Greifer; es liefert nur alte
Objekt-Primitiven, nicht den aktiven Endeffektor.

| Modell | Verifizierte URL | Status |
|---|---|---|
| FR5 | https://manual.fairino.support/latest/download.html – `FRCobots-V6.0 STEP Models` | Offizieller Download ZIP vorhanden, CAD noch nicht pro Gelenk zugeordnet/importiert |
| Inlux Force Torque Sensor ITOSX1000FTS | https://cdn.shopify.com/s/files/1/0864/4024/7632/files/Force_Torque_Sensor.step?v=1772283373 | Originale .step-Datei direkt verlinkt, noch nicht importiert |
| Inlux IR40-50 IGRIX1000IR4050 (2× identisches Modell) | https://cdn.shopify.com/s/files/1/0864/4024/7632/files/IR40_50.step?v=1772283384 | Originale .step-Datei direkt verlinkt, noch nicht importiert |
| Inlux Dual Cobot Tool Adapter ICACX1210XDUAL | https://inluxrobotics.eu/products/dual-tool-adapter | Auf Produktseite **kein STEP- oder bemaßter Download** nachgewiesen; direkt bei sales@inluxrobotics.nl (Artikelnummer nennen) anfordern |

Die STEP-URLs sind direkte Hersteller-/Shopify-Downloads; ob die
CAD-Dateien vollständig (Teile getrennt, richtige Version) sind,
muss nach tatsächlichem Herunterladen per CAD-Leser, Bounding-Box
und Volumen geprüft werden. Der Modell-Download war aus der
aktuellen Laufzeitumgebung aufgrund fehlenden externen Netzwerkzugangs
nicht möglich. Keine Binärdateien ohne rechtlich geklärte CAD-Nutzungsrechte
ins öffentliche GitHub committen.

**Verbindlicher CAD→MuJoCo-Prozess**:
(1) STEP-Dateien per Hersteller/CI beziehen und Prüfsumme/Version erfassen;
(2) in FreeCAD/OpenCascade pro Körper triangulieren, Gelenk- und
Montagekoordinaten bestimmen und mm→m verifizieren;
(3) Detailmeshes nur als `visual`, vereinfachte konvexe oder zerlegte
`collision`-Geometrien separat, passende `mass`/`inertia` mit
Herstellerdaten statt willkürlichem CAD-Material berechnen;
(4) beide IR40-50 einschließlich beweglicher Finger als modellierte
Gelenke/Mimic-Mechanik mit 0–40-mm-Hub und Kontakt/Reibung abbilden;
(5) genaue 90°-Werkzeugrahmen nach Adapter-Montage-CAD ausrichten,
getrennte TCPs für Siebträger/Kanne/Tasse und Werkzeugwechsel prüfen;
(6) gespeicherte Blind-Kollisionstests einschließlich **inaktiven**
Greifers an Brühgruppe, Maschine, Tisch und Dampflanze; Last-/CG,
Schwerkraft/FT-Nullung und Abgleich mit echten Sensorversuchen.
STEP allein ermöglicht keine automatisch physikalisch genaue Simulation.

## Exakter Herstellerproduktname (am 23.10.2026 neu verifiziert)

**Inlux Robotics – Dual Cobot Tool Adapter, Artikelcode `ICACX1210XDUAL`.**
https://inluxrobotics.eu/products/dual-tool-adapter

Verifizierte Fakten: gleicher von Inlux hergestellter Adapter beim deutschen
Distributor Welectron (`IX1-900013`), volle FAIRINO-FR5-Kompatibilität
laut Hersteller, zwei simultan verwendbare Werkzeugaufnahmen, CNC
Aluminium 6061, 335 EUR Herstellerlistenpreis vor länderspezifischer
Mehrwertsteuer.

**CAD-Status nach Suche mit dem EXAKTEN Produktnamen:** Inlux zeigt
eine Rubrik „Downloads“, darunter aber **keinen tatsächlichen anklickbaren
Link auf STEP/STP/IGES, technische Zeichnung oder Montage-CAD**. Auch
Welectron bietet auf der Produktseite kein solches nachgewiesenes CAD.
Bitte keinen anderen `Dual Tool Adapter` von TMC/Fairino als denselben
Inlux-Adapter behandeln: das wäre ein separates Herstellerprodukt.
Das Ergebnis ist daher weiterhin: **Original-STEP dieses exakten Adapters
fehlt.** Das ist eine reale Beschaffungssperre für eine maßhaltige
komplette Montage; weder Maße noch Verbindungswinkel aus Marketingfotos
als offizielle CAD-Daten ausgeben.

Anfrage an `sales@inluxrobotics.nl`: 
`Could you please share the STEP CAD and dimensioned drawing for
ICACX1210XDUAL (Dual Cobot Tool Adapter), including flange coordinate
frames, 90-degree tool face relationship, mass and mount interfaces?
We intend to use it between an ITOSX1000FTS force/torque sensor and two
IR40-50 grippers on a FAIRINO FR5. Please confirm mechanical and RS485/
power compatibility of all three devices.`

## Historisches Parallelgreifer-Hilfsmodell (nicht die aktuelle Baugruppe)

In `robobarista/dual_ir40_concept.py` wurde bisher
`MOUNTS={"handle_tool":(-.070,0.,.040),"cup_tool":(.070,0.,.040)}`
verwendet: beide Greifer haben **dieselbe lokale Ausrichtung**, nur parallel
zueinander um 140 mm verschoben. Selbst nach dem Tassenhenkel-Fix ist das
**NICHT** der gewünschte reale 90°-Dual-Tool-Adapter.

Diesen Demo-Aufbau **nicht** als baubares IR40-Greifersystem bezeichnen.
Vor einer Änderung der tatsächlichen Szenenkinematik unbedingt die
Hersteller-STEP-Dateien und die Adapter-Werkzeugrahmen prüfen.

## Recherchierte Bauteile / Quellen

1. **Mechanischer rechtwinkliger Dual-Tool-Adapter**:
   Inlux `ICACX1210XDUAL`, 335 EUR (Händler in DE Welectron
   `IX1-900013`: 398 EUR inkl. 19% USt); FR5 laut Hersteller unterstützt,
   Aluminium 6061, zwei Montageflächen in unterschiedlicher Ausrichtung.
   https://inluxrobotics.eu/products/dual-tool-adapter
   https://www.welectron.com/Inlux-Dual-Cobot-Tool-Adapter-for-Fairino

   Vergleich: TMC Robotics/Fairino Belgien `Dual Tool Adapter`, 315 EUR
   ohne MwSt. vom Händler/Hersteller TMC Robotics; Montage FR5 angegeben.
   https://www.fairino.be/product/dual-tool-adapter-for-fairino-cobots

   **WICHTIG**: TMC Robotics ist ein Fairino-Vertriebspartner, der
   mechanische Adapter **selbst** herstellt; nicht als 100 % ORIGINAL
   FAIRINO-Herstellerteil behaupten. Exact 90° angle, flange orientations,
   mass, length, step and screw clearance from commercial CAD to validate.

2. **6-Achs-Kraft-/Drehmomentsensor** direkt hinter FR5-Flansch,
   **VOR** Dual-Tool-Adapter; misst externen Wrench der beiden
   nachgelagerten Werkzeuge zusammen:
   Inlux `ITOSX1000FTS`, 885 EUR; 0–200N / 0–5Nm, vom Händler angegebene
   accuracy ±0.1N / ±0.01Nm, IP63, STEP und 2D-Zeichnung verfügbar.
   https://inluxrobotics.eu/products/force-torque-sensor

   **Nicht vom Nutzer gewählte Alternative (nicht einplanen):** Fairino/TMC `GZCX-6F-75B`, 735 EUR zzgl. MwSt,
   Flanschposition ausdrücklich als zwischen FR-Roboter und Werkzeug genannt.
   **ACHTUNG**: TMC Produktdetailseite und Katalog nennen
   widersprüchliche Fx/Fy/Fz, Mz, Gewicht und Accuracy-Spezifikationen.
   Für die Auswahl MUSS eine versionsgenaue Hersteller-Datenblatt-Nr.
   eingeholt werden, nicht blind 0.1 N behaupten.
   https://www.fairino.be/product/force-sensor-cobot

3. **ZWEI IR40-50 Greifer** (nicht EPG40-050 verwechseln),
   Inlux `IGRIX1000IR4050` je 635 EUR, 0–40 mm Öffnungsbereich,
   einstellbare Greifkraft 2–50 N (pro Finger, siehe Datenblatt),
   Eigengewicht 0.36 kg/Gerät, 24 V, Modbus RTU (RS485),
   max 0.85 A/Gerät. Offizielles STEP und 2D-Datenblatt verfügbar:
   https://inluxrobotics.eu/products/fairino-gripper

## Mechanische Stückfolge (aus Sicht Roboter)

```
FAIRINO FR5 Werkzeugflansch
      |
      v
6-DOF Kraft-/Drehmomentsensor + Herstellerkompatible Montageadapter
      |
      v
mechanischer 90° Dual-Tool-Adapter
     / \
    /   \      (Achswinkel und exakte Flanschrotation aus STEP verifizieren)
   v     v
IR40-50 IR40-50
A       B
Siebtraeger / Milchkanne        Tassenhenkel / Einwegbecher (wechselbare Finger)
```

**Greifkraftmessung**: Ein Sensor VOR dem Adapter misst die SUMME
aus Gewicht, Beschleunigung/Schwerkraftanteilen und äußerer Kontaktkraft
(6D Wrench, nach Kalibrierung), aber NICHT getrennt die Klemmkraft
jedes Fingers an beiden Greifern. Exakte Fingerkraft/Grip-Slip benötigt
Greifertelemetrie oder sensorisierte Finger, getrennte F/T an den beiden
Armen ist etwas anderes.

**Rotationsanbindung**: mindestens ZWEI kalibrierte Tool Coordinate
Frames, inkl. der Halterung, Finger und Spout bzw. Henkelgriff; die
Position jedes Werkzeug-TCP und jedes Tool-Payload-Schwerpunktes
muss dem Robotersystem bekannt sein. Der passive Greifer darf nicht
an Kannen-/Espressomaschine anstoßen.

**Last**: 2×0.36 kg Greifer ohne Finger, 90° Adapter, F/T Sensor,
beide Sonder-Finger, Tasse/Siebträger/Kanne + Milch, Verbindungsteile,
Kabel und Trägheitsmomente zählen zur FR5 Nutzlast-/Handgelenklast;
die Adaptermasse ist noch unbekannt. FR5 5kg Nennlast bedeutet
**nicht automatisch** Zulässigkeit jedes auskragenden Doppelgriffs.

## Elektrik / CAN/RS485 – offener Gate

- FAIRINO enthält laut Herstellerdocs eine einzelne RS485-Flanschbuchse;
  der 69 EUR TMC `Y-Splitter Cable` versorgt **nur zwei** Teilnehmer
  über Modbus-RTU-Werkzeugport. Als `2-in-1` mechanischen Adapter
  NIE mit einer elektrischen Leitung verwechseln.
  https://www.fairino.be/product/y-splitter-cable-for-dual-tool-adapter
- FAIRINO Manual Kap. 8.2.2.3 "Multiple Grippers":
  Geräteadressen 1–8, individuell am PC setzen;
  Kap. 8.3.2: kombinierte Protokolle Greifer+Sensor existieren.
  https://manual.fairino.support/latest/CobotsManual/robot_peripherals.html
- **2 Greifer + 1 Sensor = 3 intelligente Teilnehmer.**
  Ohne validierten gemeinsamen RS485-Bus, ausreichenden Strom am Port,
  geeignete Verzweigung/Separatstromversorgung und zum Sensorpassenden
  Protokoll **NICHT als Plug-and-Play erklären**.
  Evtl. Sensor per Ethernet/ separater RS485 an Controller + zwei Greifer
  am Flansch-Y-Splitter, wenn genaues Modell das zulässt.
  Vendor muss Firmware-, Adress- und Versorgungsfreigabe erteilen.
- Die FAIRINO `FT_Control`, `FT_GetForceTorqueRCS`, `FT_Guard`,
  `SetToolCoord`, `GetTCPOffset`, `SetLoadWeight`,
  `SetLoadCoord` werden bevorzugt. Eigenentwickelten MPC-PR #1
  hat der Nutzer verworfen. Hardware nie automatisch ansteuern.

## Vor Simulation und Einkauf zwingend verifizieren

- CAD/STEP des exakten Dualadapters, 90° Montagebilder,
  Masse, Flansch-Schnittstellen und Befestigungsteile beider IR40;
- Sensor zu FR5 Flansch / Dualadapter: passen Ober-/Unterseite
  normgerecht direkt? Gibt es mechanische Adapterplatte? Wo läuft M12?
- Full 3D der zwei Greifer (Inlux IR40 STEP vorhanden), Finger
  für Kannenhenkel/Siebträger und Tassenhenkel/EINWEG-Becher separat.
- Wechselseitige Kollision: unbenutzter Greifer, Tisch,
  Kaffeemaschinenkopf, Dampflanze und andere Arbeitsflächen;
- Lastzentrum, empfindliche Kraftregler, Sensor-Nullpunkt mit
  Doppeladapter und beiden Greifern, externe Beschleunigungen;
- elektrische drei Teilnehmer: Slave-IDs, Adressen/Konfiguration,
  Bus terminiert und genügend 24-V-Spitzenstrom; Sicherheitsgrenzen.

**Projektstand nach PR #21 (23.10.2026):** Die aktive 90°-Werkzeugvorschau
steht in `robobarista/dual_tool_90_preview.py`, die kinematisch starr
montierten PR4-Werkzeuge in `robobarista/dual_90_tool_attachments.py`,
und seitliche Anfahrpose mit tatsächlicher FR5-MuJoCo-IK in
`robobarista/side_grasp_preview.py`. **Nicht** als echte Greifkraft,
Inlux-STEP oder vollständige Kollisionsvalidierung missverstehen.
Der Integrations-PR #18 wird nur komponentenweise nach aktueller
main-Kompatibilitätsprüfung im nächsten Chat geprüft.
