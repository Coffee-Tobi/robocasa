# Selektive Integration von PR #17 und PR #18 (23.10.2026)

**Basis: freigegebenes `main` nach #21/#22.** Dieser neue Branch kopiert
keine alten Greifer-, Kaffee- oder Kamera-Szenen aus dem divergierten PR #18.
`side_grasp_preview.py`, `dual_90_tool_attachments.py`,
`dual_tool_90_preview.py`, `coffee_workflow.py`, `compact_tags.py`
und FAIRINO NewSpline aus `main` bleiben unverändert.

## Übernommen (isolierte Offline-Bausteine)

| Herkunft | Modul | Einsatzzweck und Grenze |
| --- | --- | --- |
| #17 | `real_latte_dataset.py` | Originale 14D RealMan-Zeitreihen für genau eine Episode einlesen; keine echten Videos im Git |
| #17 | `realman_fk_bridge.py` | 6D-FK aus verifizierter RM65-URDF, expliziter Arm-/Einheiten- und Tool-Transform; Identität nur als unkalibrierter Mock |
| #17 | `external_pose_replay.py` | Zeitlich abtasten, **nur relative Posen numerisch umrechnen**; DIREKTE FR5-IK-Wiedergabe entfernt, bis aktueller 90°-TCP→Kannen-Transform und Kollisionsräume vorliegen |
| #17 | `original_camera_tags.py` | Reale RGB-Bilder optional auf Marker-Pixel untersuchen, keine metrische Ursprungskamera-3D-Messung |
| #18 | `latte_dmp.py` | Optionales DFKI positional-3D-DMP aus **rekonstruierten** Kameraposen, keine Orientierung, Geschwindigkeits-/Kraft-/Milchregelung |
| #18 | `vision_quality.py` | Virtuelle Kamera-Güte und Station-Tag-Sichtbarkeit getrennt bewerten, nie realen Spout-Fehler behaupten |
| #18 | `moveit_scene_bridge.py` | Konservative nominale MuJoCo-KEEP_OUTS als MoveIt-Boxen exportieren; NICHT voller FR5-90°-Doppelgreifer / reale CAD-Kollision |
| #18 | `vendor_runtime_preflight.py` | Hersteller-SDK/ROS2/SimMachine nur lesend auf vorhandene Software/API-Oberfläche prüfen, keinerlei Controller-Verbindung |

## Aktuelle 90°-Werkzeugframe-Brücke (neu entwickelt)

`current_90_tool_frames.py` liest die **tatsächlichen aktuellen** MuJoCo-
Body- und Geom-Transforms aus `coffee_workflow._story_scene`:
`rb_cell_tcp` → `rb_90_ir40_handle` → `rb_gripped_pitcher` und
`rb_gripped_pitcher_spout`. Ein neuer Test prüft die Hierarchie, die
Konstanz von `T_TCP_pitcher` bei Gelenkbewegung sowie explizite
Rücktransformation eines Quell-Kannenpfades auf die FR5-Wrist-Pose.
Die berechnete Spout-Position ist ausdrücklich nur die Mitte der
Visualisierungs-Geometrie, kein vermessener Ausgießerrand.

**Der Schritt ist ausschließlich geometrisch.** Er führt keine IK,
Kollisionsprüfung des unbenutzten zweiten Greifers, Servobewegung,
Milchphysik oder Kalibrierung am realen Hersteller-CAD aus. Die
neue MuJoCo-Regressions-CI muss vor Merge grün sein.

Die synthetischen Unit-Tests der Ursprungs-PRs sind übernommen; ein alter
`camera_joint_replay.marker_config`-Test wurde entfernt, weil `main` bereits
ein anderes, verbindliches Objekt-AprilTag-System hat. Dieses neue Branch
verändert dieses System NICHT.

## Was wirklich vorliegt

Laut vorherigem PR #17 CI-Bericht wurde Episode 0 in GitHub Actions
als **400 Samples, 14D, rund 20 Hz, 19,95 s** geprüft. Dieser Integrations-PR
lädt **keine neue Originalepisode** und enthält nur synthetische Tests.
Die öffentliche Projektseite beschreibt teilweise „12D“, während die
in #17 geprüften Rohdateien 14D inklusive Greifer ergaben. Datenlayout
bei erneuter Nutzung strikt validieren; veröffentlichte Lizenz für
Originaldaten/Code weiterhin ungeklärt.

**NICHT nachgewiesen:** Original-Arm/Halterung/Kannen-Transformation, echte
RM65-Flansch-zu-Kanne-Kalibrierung, echte Marker in Originalvideos, aktuelle
FR5-90°-Gripper→Kannen-TCP-Transformation, DMP mit Orientierung, Bewegungs-
Servo, F/T-Kontakt, Milch-/Crema-Physik, reale unter 0,5 mm Spout-Präzision.

## Nutzung (nur Auswertung, kein Hardwarebetrieb)

```bash
# Kostenloser Basistest mit vorhandenen Requirements, ohne GPU:
python -m pytest -q tests/test_real_latte_import.py \
  tests/test_latte_dmp_integration.py \
  tests/test_integration_quality_moveit.py \
  tests/test_vendor_runtime_preflight.py

# Optional: eine originale numerische Episode lokal importieren
python -m pip install huggingface_hub pyarrow
python -m robobarista.real_latte_dataset --parquet-only \
  --episode 0 --output runs/research_episode0

# Nur nominale Kollisionsboxen exportieren (kein ROS2-Publish)
python -m robobarista.moveit_scene_bridge \
  --output runs/review_collision_scene.json

# Offline SDK-/Runtime-Probe ohne Robot.RPC
python -m robobarista.vendor_runtime_preflight \
  --output runs/runtime_read_only.json
```

`latte_dmp.py` benötigt optional `movement_primitives>=0.9,<1`
(BSD-3-Clause). Es braucht rekonstruiertes
`camera_measurements.json`; `main` erzeugt diese Datei noch nicht in
der neuen 90°-Seitengreifer-Scene. Das ist **noch keine verkabelte
End-to-End-Kette**. Auch die Marker-Scanfunktion benötigt zuvor wirklich
heruntergeladene Originalkameraframes.

## Vor der nächsten FR5-Bewegungsintegration

1. Die hier eingebaute *virtuelle* TCP→IR40→Kannen-Umrechnung an echter
   STEP-Geometrie und kalibriertem Spout überprüfen; erst dann als
   physische Transformation verwenden.
2. Originaler RM65-Datensatz: Schenkarm, URDF-Variante, Originalgrößen,
   rad/deg und pour-only Segment *an Originalbildern* bestätigen.
3. Für jede Quelle eigenständige Tests auf laterale Anfahrt, beide
   Greifer-Kollisionshüllen, Gelenkgrenzen und Station-Tag-Abdeckung.
4. Erst danach neues **read-only** FR5-IK-Replay und 3-/4-Kamera-Messung;
   keine Ground-Truth-Einspeisung in Rekonstruktion.
5. Hersteller SimMachine/Servo + Nassversuche getrennt validieren.

**Merge-Gate:** zunächst GitHub CI, Review, dann nur gezielt nach main.
Original #17 bleibt geschlossen, #18 bleibt unangetasteter Draft.
