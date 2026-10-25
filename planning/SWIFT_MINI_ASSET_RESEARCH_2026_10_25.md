# Swift Mini – Recherche und sicherer MuJoCo-Visual-Proxy (25.10.2026)

## Ergebnis

**Kein öffentlich eindeutig verifiziertes Original-3D-CAD-/STEP-/STL-Modell für die La Marzocco SWIFT MINI gefunden.** Die Hersteller-Supportseite listet STEP/IGS/DWG für die **große gewerbliche Swift** (und andere Maschinen), nicht ausdrücklich die **Swift Mini**. Diese Modelle dürfen nicht fälschlich übernommen und skaliert werden.

Originalquellen:
- Betriebsanleitung Swift Mini v1.2 (S. 5): https://lamarzoccousa.com/wp-content/uploads/2020/08/MAN.13.05.1_SWIFTMINI_EN-FR_V1.2-.pdf
- Parts Catalog Swift Mini v1.0 mit Explosionszeichnungen: https://lamarzocco.com/de/de/wp-content/uploads/2023/07/15-SwiftMini_V1.0.pdf
- Original-Herstellerressourcen für Designer (aber Swift, nicht Swift Mini): https://www.lamarzoccousa.com/support/
- Original Schnellstartanleitung mit Portafilter-/Bajonettring-Abbildungen: https://home.lamarzoccousa.com/wp-content/uploads/2020/08/Quick-Start-Guide-Swift-Mini.pdf

Hersteller-Außenmaße (v1.2): **Breite 166 mm, Tiefe 322 mm, Höhe 449 mm**. Der Parts Catalog v1.0 rundet Höhe auf 448 mm, Gewicht 13.5kg; das v1.2 Manual nennt 14.2kg. Die unterschiedlichen Versionen/Angaben nicht stillschweigend vereinheitlichen.

## Was im Branch vorhanden ist

`robobarista/swift_mini_visual_asset.py` generiert
- ein 1:1 skaliertes statisches Visualisierungssurrogat (Boxen, Zylinder, Frontkopf, Dosierring, Aufsatz, Trichter, Knopf und Frontschale)
- separat drei Preview-Kameras und PNG-Ansichten über tatsächlichen MuJoCo Renderer
- explizite Referenz-Hüllmaße als identifizierbare MuJoCo-Sites
- einen als **UNVERMESSEN** gekennzeichneten visuellen Siebträger-Aufnahmepunkt
- `dimensions_and_limitations.json` mit Herkunft und Grenzen.

**Keine automatische Integration in `main`**. Der existierende Arbeitstisch, die bereits akzeptierte FAIRINO-FR5-Szene, die Greifer und die stationäre 1-DoF-Tassenhalterung werden nicht überschrieben.

## Was noch NICHT vorliegt

- **Echter** Hersteller-STEP für die Mini, reale Oberflächen/Hopper/Skalen, exakte Bayonett-Mechanik, tolerierte Siebträgergriffachse, Einführ- und Verriegelungsbewegung.
- Physisch kalibrierte Swift Mini Position in unserer Arbeitszelle, exakte Roboter-Dock-Koordinate, Kraft-/Kontaktkriterien, vollständige Kollisionsformen.
- Der bisherige generische Mühlen-MuJoCo-Körper (`barista_cell.py`, Breite ~184mm, Tiefe nur ~180mm, Höhe Box ~280mm) ist **nicht** die Swift Mini. Die offizielle Swift-Mini-Tiefe **322mm** ist sicherheitsrelevant: Die Maschine muss möglicherweise anders gestellt werden, damit sie nicht FR5-Bewegungen oder andere Stationen überlappt.
- Automatisch mahlen/dosieren/tampen wird weiterhin nur abstrakt durch den Arbeitsablauf repräsentiert; keine Füllphysik, keine elektrischen Signale oder Herstellerkommandos.

## Empfohlener nächster Schritt

Für die echte Swift Mini STEP/CAD und Zeichnungen der Bajonett-Schnittstelle direkt beim Hersteller anfragen. Dann das statische Visualisierungsmodell bei bestätigter Lizenz gegen echte CAD-Meshes ersetzen. Danach echte Docking-Frames und `workcell_constraints.py` aktualisieren; **erst dann** mit einer neuen Arbeitstisch-Simulation, Sichtbarkeitscheck für 3/4 Kameras und kollisionsgeprüfter FR5-6D-IK experimentieren.

**Tassenhalterung bleibt unverändert:** Festes Pivot-Z, ausschließlich 1 Winkel-DOF; *keine* unabhängige Hub-/X-/Y-Achse. Diese Asset-PR enthält keine Änderungen an der Tassenhalterung.

## Projektstand

Für **eine visuelle End-to-End-Demo** sind wesentliche Komponenten als abstrakte Modelle vorhanden. Für einen **funktionsfähigen echten RoboBarista** fehlen unabhängig von der Swift Mini u.a. echte Espresso-/Dampflanzen-Geometrie und Steuerung, magnetisch/schraubend tragfähige Griffbefestigungen, physikalischer Griffkontakt und Tool-Kollisionen, Hersteller-FR5-Bewegungsnachweis, original Latte-Art Kannenarm-/Tassenreferenz, Sensorik und reale Milchflussregelung. Ein realer Latte-Art-Effekt kann nicht aus dem Asset-Vorhandensein abgeleitet werden.
