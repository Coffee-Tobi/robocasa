# Vollständiger kamera-freier FR5 Swift Mini Roundtrip mit West-Follow-Video

Stand: 25.10.2026. Isolierter Entwurfsbranch, keine Änderungen in main.

## Programmberechnung

1. Siebträger liegt initial in stationärem, um **-30°** ausgerichtetem Halter.
2. FR5 ohne Bild-Feedback fährt zu definierter Halterpose.
3. **Während des Films**: explizite Pause "Gripper schließen"; nur symbolischer Besitzwechsel von Halterdarstellung zum Greiferteil. Kein IR40-Aktuator und KEIN Fingerkontakt (noch nicht implementiert).
4. FR5 hebt den Korb waagerecht heraus und fährt zum provisorischen Swift Mini-Einsetzpunkt.
5. Nach dem Aufwärtsfädeln folgt eine 12°-Rechtsdrehung in kinematisch kollisionsgescreenten 1°-Schritten.
6. 7-Frame-Haltephase an der Mühle.
7. **Gegendrehung 12° nach links**, anschließend die exakt umgekehrte, zuvor kinematisch geprüfte FR5-Gelenkbahn – mit Kollisions- und Korb-Neigungstests auf ALLEN Hin- und Rück-Samples.
8. Korb wird geometrisch im Halter ausgerichtet, der Greifer öffnet **nur symbolisch im Video**, Siebträger wird in der festen Haltergeometrie sichtbar, FR5 zieht sich zurück.
9. Acht Frames mit wieder eingesetztem Siebträger zur visuellen Abnahme.

**Kein Objekt wird am Griffübergang räumlich teleportiert:** beim Umschalten stimmen Greifer-Korbzentrum und feste Halterpose auf maximal 3 mm überein. Die Kraftübertragung des echten IR40 wird nicht getestet.

## West-follow View ohne neue Bildsteuerung

Die Kamera-Renderinstanz enthält eine zusätzliche mocap-Betrachtungskamera.
Die FR5-Planungsinstanz hat **NULL Kameras**, und das Roboter-Gelenkprogramm
wird *vor* dem Erstellen der Renderinstanz berechnet.

Anfang: Eye = Halter-Korbzentrum + (-0.50 m, -0.20 m, +0.21 m).
Kameraorientierung einmalig nach Westen/Korbzentrum gerichtet.
Je Frame: neue POSITION = 0.42 alter Eye + 0.42 (Korb-XYZ + **gleicher Eye-Offset**),
ohne Quaternion-Anpassung. Da sich der Blickpunkt seitlich zum
Korb verschiebt, bleiben Greifer und Korb besser zentriert;
Rotation, Sichtfeld und Neigung bleiben unverändert.

Vor/nach dem Greifvorgang folgt die Kamera dem festen Halterkorb,
während des Tragens folgt sie dem FR5-Korb. Alle Aufnahmebilder
sind echte MuJoCo-Renderer-Frames, keine generierten Einzelbilder.

## Grenzen

- Swift Mini Innenaufnahme, Bajonettwinkel und erforderliche Rechtsdrehung
  sind **nicht** durch Original-CAD/-Handbuch bestätigt. Deshalb nur Versuch.
- Visualisierungsgeometrie der Swift Mini ist noncolliding; reale
  Roboter-/Mühlenkollisionen können nicht ausgeschlossen werden.
- Der gekennzeichnete symbolische Greifvorgang ist kein Kontakt mit
  fingerkontaktgesteuerten motorischen IR40-Fingern.
- Keine echte FR5-Kontaktkraft, kein Drehmomentregelungsnachweis,
  keine angeschlossene physische Maschine.
- Der Hinweg und Rückweg sind echte FR5-FK/IK-Gelenkposen mit dem
  tatsächlichen FAIRINO-Modell in MuJoCo und gescreenten Eigenkollisionen.
- Die Erstellung weiterer echter Roboteraktionen erfordert
  sicherheitstechnische Validierung, Kraftgrenzen und Hersteller-CAD.
