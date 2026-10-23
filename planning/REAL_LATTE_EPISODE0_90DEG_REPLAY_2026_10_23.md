# Episode 0: echte RealMan-Gelenkdaten -> aktueller FR5 / 90°-IR40 (23.10.2026)

**Quelle:** `ridxm/latte-pour-demos`, numerische Episode 0 (LeRobot
Parquet). GitHub Actions lädt **nur 1 Originalepisode** für den Test,
nicht alle 40 Episoden, nicht die Originalvideos, keine Gewichte.
Der reale Datensatz liefert **400 Zeilen, 20 Hz, 14 Werte pro State/Action**.
State-Layout: je sechs Armwinkel und ein Greiferwert **pro Arm**.
Die Projekt-Website schreibt teils „12D“; der Rohdatensatz hat 14D
einschließlich Greifern. Nur Gelenkdaten sind als Kinematik-Eingabe
verifiziert, **keine originale 6D-Kannenpose**.

## Erste echte numerische CI (Originalepisode 0)

Skript: `robobarista/real_demo_90_ik.py`;
Workflow: `.github/workflows/real-demo-90-ik-exploratory.yml`.

- Offizielle RealMan-Repository-URDF **RM65-B** als *unbestätigte
  Hardware-Variante*; angenommen, nicht gemessen: rad-Einheiten,
  `link_6`, `joint_1..6`, `T_RM65_flange_pitcher=I`.
- Beide Arme wurden **getrennt** analysiert. Keiner ist anhand von
  Originalvideos bisher als Kannenarm bestätigt.
- Quell-Zeiten **8–12 Sekunden sind ein explorativer Ausschnitt**, nicht
  optisch nachgewiesen als tatsächliche Gießphase.
- Datenpunktabstand 50ms im Original, FK auf **13 virtuelle FR5-Posen
  über 4 Sekunden** abgetastet.
- Startposition: *freigegebene* 60mm horizontale FR5-Seitenanfahrt
  mit 90°-Dual-IR40; nicht alte Überkopf-/Parallelgreifer-Anfahrt.
- Transformation: `FR5_TCP -> axial IR40 -> visual held pitcher -> visual
  spout` aus dem aktuellen MuJoCo-Modell, dann inverse TCP-Transform
  für die gespeicherte RealMan-Relativbewegung.
- Pro Frame FR5-6D-IK, MuJoCo-Kontaktcheck, verfügbare AABB-
  Maschinenfreiräume, konservative Hüllkugel des inaktiven zweiten
  Greifers und Gelenkweg-Unterteilung <=0,018rad.

| Originalquell-Arm | Quell-FK-Pfad-Auslenkung | IK-Posen erreicht | Resultat |
|---|---:|---:|---|
| Links | 12,364mm | 13/13 | **Virtueller geometrischer IK-Test PASS** |
| Rechts | 41,275mm | 2/13 | **STOP** beim dritten Ziel: IK-Position 3,515mm Restfehler |

Die Zahlen **sind NICHT** tatsächliche Ausgießerfehler. Ein über
seinen eigenen IK-Sollwert nahezu exakt gerenderter Spout beweist
**nicht** reale Präzision oder unabhängig gemessene Kameragenauigkeit.

**Wichtig:** Der linke Erfolg belegt **nicht**, dass Links die
Milchkanne gehalten hat. Es könnte sich ebenso um den Becherarm
handeln; genau das müssen die Originalvideos bestätigen.

## Originalvideo und 40-Demo-Frage

Die 40 Originalaufnahmen enthalten nach Autorenseite
synchronisierte RGB-Views von oben sowie linker und rechter
Handgelenkkamera. Im bisher geladenen Original-Parquet sind zwei
6DOF-Gelenkvektoren, zwei Greiferwerte und Action/Time verfügbar.
Es wurden **keine** AprilTag-Originalmessungen, keine gemessene
Ausgießerposition und kein Milchfluss eingebaut.
Originalkamera-Daten und Original-Wrist/Base-Extrinsics sind
nicht validiert; die RealMan-URDF-FK-Geometrie ist *abgeleitet*.

## Explizite Blocker vor echtem Pour-Replay

1. **Kannenarm und tatsächliches Pour-only-Zeitfenster visuell** anhand
   der öffentlich verfügbaren Originalvideos auswählen.
2. Urspüngliche RM65-Variante, Joint Units, beide
   `T_World_RM65_base` und reale Flansch-Kannenbefestigung messen bzw.
   aus der Quelle bestätigen; keine Identitätsannahme verschweigen.
3. Das **Kannenrelativ zur Tasse**-Signal ist für latte art nötig, wenn
   in Originaldemos **beide** Arme bewegt wurden. Die räumliche
   Original-Dualarm-Basiskalibrierung fehlt noch.
4. Für den rechten Pfad alternative sichere Wrist-Startpose und
   IK-Robustheit untersuchen, ohne Bewegungsamplitude heimlich
   herunterzuskalieren. Kollisionshülle mit CAD/aktiver Greiferstellung
   validieren.
5. Eigenes Video ist ausschließlich MuJoCo-Visualisierung
   der Bewegungs-Rekonstruktion; keine Originalvideos, keine Physik.
6. Unabhängige kleine AprilTag-Kamerarekonstruktion und echte
   Hersteller-Motor/Schrittmessungen als **spätere** separate Tests.

`main` und ursprüngliche PRs #17/#18 bleiben unangetastet.
Dieser Versuch ist im Draft PR #24.
