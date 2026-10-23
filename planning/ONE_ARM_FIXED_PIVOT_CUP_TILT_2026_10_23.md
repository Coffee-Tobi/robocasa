# Einarm-Latte-Art: festes Tassen-Drehgelenk (23.10.2026)

## Technische Entscheidung

**Exakt ein Roboterarm:** FAIRINO FR5 mit freigegebenem 90°-Dual-IR40-
Tool, im Gießteil greift nur der axiale IR40 die Milchkanne.
**Tasse:** stationäre Halterung am Pour-Dock. In MuJoCo ein **echtes
1-DoF-Drehgelenk** `rb_fixed_pivot_cup_tilt_joint`, lokale Y-Achse;
der feste Sockel hat null motorische translatorische DoFs. Die Tasse darf
sich nur um den einen fest verschraubten Drehpunkt auf einer Kreisbahn
bewegen. Eine Tassenkante bewegt sich dabei in XYZ als *Folge von
Rotation*, nicht durch eine zusätzliche XY- oder Z-Schiene.

- Winkelanschläge der **virtuellen Konstruktion**: -18° bis +18°.
- Max. programmierte Sollwinkeländerung: 45° pro Sekunde.
- Muster 1: sanft von +12° auf 0° über die Gießsequenz.
- Muster 2: **nur Y-Drehanteil** der Relativrotation des angenommenen
  anderen RealMan-Quellarms wird auf die reale virtuelle Drehachse
  projiziert, ggf. auf Grenzwinkel/-geschwindigkeit begrenzt.
- Die beiden Muster werden in separaten MuJoCo-MP4s ausgegeben.

Weder 18° noch 45°/s sind eine reale Hardware-Freigabe. Für echte
Getriebe/Servo müssten Trägheit, Drehmomente, Geschwindigkeit, Haltekraft
und eventuelle Milchschwappbewegung vermessen werden.

## Daten und Provenienz

Der bisherige Originalepisode-0-Test importiert **400 Original-RealMan-
Samples / 20 Hz**, verarbeitet einen unbestätigten **8–12s** Abschnitt
und rekonstruiert ihn mit *nicht bestätigter* RM65-B-URDF und
identischer RM65-Flansch-zu-Kanne-Mock-Transformation.

**Noch nicht identifiziert**, ob der linke Quellarm tatsächlich die
Milchkanne und der rechte die Tasse hält. Der erfolgreiche *linke*
13/13-geometrische FR5-IK-Clip kann deshalb ein übertragenes Stück
**Original-Tassenarm** sein. Der rechte Quellarm erreichte nur 2/13
und scheiterte an IK. Die Projekt-Demo wurde noch nicht als
originaler tatsächlich gegossener Milchpfad bestätigt.

Darum sind beide gerenderten Clips ausdrücklich
**Einarm-Geometrieexperimente**, keine validierte Reproduktion
der originalen Latte Art. Bei zwei original bewegten Armen kann die
Tassenrotation allein die relative XY/Z-Verschiebung **nicht** ersetzen:
der einzige FR5-Arm muss die Milchkannen-Bahn so korrigieren, dass
die Spitze über der feststehenden Tasse bleibt. Die aktuellen Clips
simulieren **keinen** Milchfluss, keinen Kontakt und prüfen noch
nicht, ob der Ausgießer nahe genug an der Tasse ist.
Die JSON-Berichte messen explizit Abstand Spout-Mitte zu Tassenrand,
aber ohne Kalibrierung ist das noch kein gießfähiger Abstand.

## Dateien

- `robobarista/one_arm_fixed_cup_hinge.py` — MuJoCo-1D-Hinge,
  Drehprofil, 2 Kameraperspektiven und MP4-Export.
- `tests/test_one_arm_fixed_cup_hinge.py` — echte MuJoCo-Modelltests,
  festes Pivot, keine Prismatik, ein Gelenk, Winkelgrenzen.
- `.github/workflows/real-demo-90-ik-exploratory.yml` —
  nur eine HF-Episode, offizielle URDF-Testdatei und zwei lokale
  Videovarianten. **Keine** Originalvideos oder Joint-Rohdaten als
  GitHub-Artefakte veröffentlichen.

## Nächste physikalische Verbesserung

Originalvideos ansehen, Kannen- und Tassenarm sicher identifizieren,
`T_flange_kannenspitze` und `T_flange_tassenkoerper` messen.
Dann relativen Ausgießerpunkt in das feste Tassensystem transformieren,
eine **neue** Einarm-IK über der *fixierten* Tasse lösen und die
Schwenkung der Tassenhalterung nur über 1D-Winkel ansteuern.
Bei unerreichbarer Pose/ungenügender Sicht strikt abbrechen,
nicht Gesamtbewegung als Latte Art ausgeben.

Kein Merge des historischen Draft #18; aktueller Review-Branch #24.
