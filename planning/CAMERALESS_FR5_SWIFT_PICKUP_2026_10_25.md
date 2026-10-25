# Kamera-freier FR5: Siebträger aus Halter zur Swift Mini (25.10.2026)

## Grundidee
Der Roboter benötigt für einen wiederholbaren, fest installierten Arbeitsprozess
nicht zwingend AprilTag-Kameratracking. FESTE Geräte und ein reproduzierbar
geometrisch definierter Halter erlauben das Offline-Teach-in von 6D-TCP-Posen
und die Wiederholung über FR5 Gelenkencoder. Die Sensorkonzepte
(v.a. Kraft-/Drehmoment, Motorstrom, Referenzschalter) sollten für
Kontakt-/Fehlererkennung aber bestehen bleiben. **Blindes Drücken und Drehen
bei Positionsfehlern ist nicht akzeptabel.**

## Neuer isolierter Umfang
- nur tatsächliches sechsachsiges FR5-MuJoCo-Robotermodell mit dem bisherigen
  90°-Dual-IR40-Tool und axialem IR40;
- Swift Mini als **maßstäbliche, aber in Details geschätzte** visuelle
  3D-Referenz (166×322×449 mm), PROVISORISCHER Einsetzpunkt;
- zusätzlicher **stationärer Siebträgerhalter** mit seitlichen
  Führungen und Rückanschlag; feste, wiederholbare Basket-Pose;
- der bestehende rechteckige IR40-kompatible Griff, zuerst als
  geparkte Geometrie, dann am axialen Greifer;
- keinerlei RGB-/Tiefen-/AprilTag-Kameras als Roboter- oder Szenensensoren.
  Die zwei Grafikansichten im Video sind nur Offscreen-MuJoCo-Renders;
- programmiertes FR5-6D-IK-Raster: Halter von oben/vorne anfahren,
  ausrichten, visuelle Übergabe vom Halter an den axialen Greifer,
  anheben, zum SWIFT-Mini-Aufnahmepunkt bewegen,
  von unten ausrichten, anheben und optional um Z nach rechts drehen;
- Der Basket wird von der IK aufrechterhalten (Roll/Pitch nahe 0).
  Die FR5-Gelenke bewegen sich tatsächlich; KEIN reines Object-Teleport.

## Sicherheits- und Realitätsgrenzen
- Greifen ist noch ein **kinematischer Zustandswechsel** (nicht
  physikalischer Fingerkontakt und keine Haltekraftmessung);
- die Hersteller-Geometrie der Swift-Mini-Aufnahme ist bislang nicht
  kalibriert; Modellgehäuse bisher noncolliding. Reale schädliche
  Kollisionen mit der Maschine können damit NICHT ausgeschlossen werden;
- das frühere separate MuJoCo-Kontaktmodell zeigte bis ca. 2.52 N
  virtuelle Kontaktkraft beim sanften Hub. Dieser Wert ist NICHT
  die Kraft, welche der FR5 bei diesem neuen Kinematikablauf ausübt.
  Solch eine gekoppelte FR5-F/T-Regelung fehlt noch;
- eine zusätzliche Rechtsdrehung in der Swift Mini wird im Original-
  Swift-Mini-Handbuch nicht als erforderliche Verriegelung genannt.
  Sie bleibt nur eine explizite **experimentelle** Wunschbewegung;
- Sämtliche Prüfungen sind offline/kollisionsvorbewertet und niemals
  Hardwarebefehle. Beim unerreichbaren IK oder bei Roboterselbstkontakt
  stoppt der gesamte Lauf statt sich durch Geometrie hindurchzubewegen.

## Perspektive ohne Kameras
Für echte Gerätepositionierung sind einmalige Hand-/Teach-Kalibrierung,
mechanische Passstifte/Anschläge, ein reproduzierbarer Siebträgerhalter,
eine genaue Werkzeugaufnahme, Ablaufgrenzen, Kontakterkennung und
schrittweise Inbetriebnahme bei reduzierter Geschwindigkeit sinnvoll.
Optional können Kameras später zur Qualitätskontrolle dienen; für das
Einstecken an fest montierte Maschinen ist kamerabasierte Trajektorien-
steuerung nicht zwingend der erste Weg.

## Dateien
- `robobarista/swift_fr5_pick_stand.py`: Einarm-6D-IK und Video.
- `tests/test_swift_fr5_pick_stand.py`: keine Kameras, FR5 + Halter +
  Swift Mini und erhaltene TCP-Kannen-/Siebträgergeometrie.
- `.github/workflows/fr5-swift-stand-pickup.yml`: echter GitHub Actions
  MuJoCo-Offline-Test, fährt alle IK-Posen und prüft Self-/Roboterkontakt.
- Altmodelle für Force-Limit-Kontakte:
  `robobarista/swift_mini_portafilter_only.py` (NICHT als FR5-Kraftbeweis).

Die zugehörige Swift-Mini-Standortposition ist ein
**Simulationslayout**, keine Empfehlung für ein reales Produkt.
