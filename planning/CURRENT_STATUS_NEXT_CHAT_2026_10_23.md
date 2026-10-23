# RoboBarista – verbindlicher Stand und Startpunkt für den nächsten Chat

**Stand:** 23. Oktober 2026. **Standardbranch: [main](https://github.com/Coffee-Tobi/robocasa/tree/main)**.
Diese Datei ist der **Einstiegspunkt**, wenn ein neuer Chat beginnt.
Die alten Plan-/MVP-Dateien sind teilweise historische Forschungsnotizen
aus dem 21.10. und überschreiben diese neueren Designentscheidungen **nicht**.

## Explizite Entscheidungen (gemeinsam mit Nutzer durchgegangen)

Vollständige Liste: [PR_REVIEW_DECISIONS.md](./PR_REVIEW_DECISIONS.md).

- #1, #2 verwerfen; #3 Kamera/Struktur behalten, alten Parallelgreifer ersetzen.
- #4 behalten, durch [#20](https://github.com/Coffee-Tobi/robocasa/pull/20)
  auf 90°-Dualadapter umgebaut; #5 behalten **nur als simulativen
  Soll-Ist-Motorvergleich**.
- #6 verwerfen, veraltete Vergleichs-Dateien in [#22](https://github.com/Coffee-Tobi/robocasa/pull/22)
  aus main entfernt; #7 & #8 behalten: **offizielle FAIRINO NewSpline-/MoveL
  API** plus 3-Kamera→FAIRINO-Dry-Run.
- #9 behalten, **vor einem Merge an den aktuellen 90°-FR5 anpassen**;
  noch [offener Draft #9](https://github.com/Coffee-Tobi/robocasa/pull/9).
- #10 verwerfen und echter Sensor-Teil später neu: alte synthetische
  F/T-Kurven und Film in #22 aus main entfernt, **aktueller** 11-Schritt-Kaffeeablauf
  bleibt erhalten.
- #11 verwerfen, nie mergen. Die akzeptierte
  [#21-Seitenanfahrbewegung](https://github.com/Coffee-Tobi/robocasa/pull/21)
  mit horizontal schließenden Greiferbacken wurde stattdessen nach main gemergt.
- #12 behalten, bereits in main: RoboCasa-inspirierte Espresso-Fixture-Logik,
  FAIRINO-SDK-Read-only-Vorbereitung, Kamera-Datensatzbrücke.
- #13 und #14 alte konkurrierende Tag-PRs **verwerfen**, sind bereits
  geschlossen und ungemergt. Kleines Objekt-AprilTag-System im freigegebenen
  **main** weiterverwenden; alten Markergrößen-Code nicht zurückkopieren.
- #15 behalten, bereits main: wirklicher MIT-lizenzierter
  externer TypeScript-**2D**-Fluid-Sidecar; nicht als Milch-/Crema-CFD
  oder echtes gegossenes Latte-Art-Herz bezeichnen.
- #16 alten Gesamt-PR verwerfen, nur sinnvolle DMP, MoveIt, Kamera-QA,
  FAIRINO-QA **einzeln aus #18 nach gesondertem Test prüfen**, nicht
  ungeprüft importieren.
- **#17 behalten als relevante Forschungsquelle und im NÄCHSTEN CHAT besprechen**;
  der ursprüngliche PR ist historisch geschlossen, Codeinhalt in #18 erhalten.
- **#18 im NÄCHSTEN CHAT besprechen**, ausdrücklich **nicht komplett
  mergen** – nur gute, aktuelle, kompatible Funktionen auf neuem
  Branch **von aktuellem main** selektiv importieren.
- #19 und #20 beide **behalten, bereits main**, #21 verbessert sie;
  keine erneuten alten parallel/von-unten-Griffe zulassen.

## Verbindlicher aktueller Werkzeugaufbau

```
FAIRINO FR5 (virtuelles echtes MuJoCo-Gelenkmodell)
  -> Inlux Force Torque Sensor ITOSX1000FTS, zzt. Visualisierungsproxy
  -> Inlux Dual Cobot Tool Adapter ICACX1210XDUAL, 90° Visualisierungsproxy
     |-> IR40-50 axial: seitlich an Milchkanne + Siebträger
     |-> IR40-50 orthogonal: seitlich an Tassenhenkel; alternativ Becher
```

Die **beiden Werkzeugachsen sind rechtwinklig**, die **Finger schließen
seitlich/horizontal**, die Kanne/Tasse bleibt bei der jeweiligen
6D-Wrist-Pose aufrecht. Eine MuJoCo-IK-Seitenanfahrt für **Tasse und
Milchkanne je 60 mm** wurde tatsächlich gerendert und in CI bestanden.
Es handelt sich *nicht* um echtes Greifen durch Kontakt oder um
belastbares Hersteller-CAD; die frei stehenden Objekte werden im Demo
kinematisch an die Finger angeheftet.

- Aktuelle Geometrie: [dual_tool_90_preview.py](../robobarista/dual_tool_90_preview.py)
- Kinematisch gehaltene Tools: [dual_90_tool_attachments.py](../robobarista/dual_90_tool_attachments.py)
- Richtige Anfahrt mit sechs FR5-Gelenken: [side_grasp_preview.py](../robobarista/side_grasp_preview.py)
- 20-Sekunden-Gießbewegung: [latte_art_20s.py](../robobarista/latte_art_20s.py)
- Kaffee-/Milchablauf (11 illustrativ kinematische Schritte):
  [coffee_workflow.py](../robobarista/coffee_workflow.py)
- Kleine Objekt-AprilTags: [compact_tags.py](../robobarista/compact_tags.py)
- Kamera→FAIRINO-Methoden:
  [fairino_barista_pipeline.py](../robobarista/fairino_barista_pipeline.py),
  [fairino_native_spline.py](../robobarista/fairino_native_spline.py)
- Exakte Sensor-/Greifer-/Adapteridentifikation, direkte STEP-Links und
  offen gebliebene RS485-/3-Teilnehmer-Frage:
  [FAIRINO_DUAL_90_TOOL_FT_IR40_ASSEMBLY.md](./FAIRINO_DUAL_90_TOOL_FT_IR40_ASSEMBLY.md).

## Inhalt des nächsten Chats – in dieser Reihenfolge

### 1. Review von [PR #17](https://github.com/Coffee-Tobi/robocasa/pull/17)

Der PR ist **geschlossen, nicht in main gemergt**, aber als
Quellcode in [draft #18](https://github.com/Coffee-Tobi/robocasa/pull/18)
erhalten. Echte Demonstrationen stammen aus
`ridxm/latte-pour-demos` mit Realman RM65, ursprünglich nicht
direkt FAIRINO-kompatibel. Vor echten Bewegungsbehauptungen:

1. Welche realen Demonstrationsdateien wurden wirklich heruntergeladen,
   mit welchen Zeitstempeln, Joint-Einheiten und Lizenzbedingungen?
   Keine erdachten originalen RGBD-Kamera- oder Pitcher-Kalibrierungsdaten.
2. Tatsächliche offizielle Realman-URDF/FK-Kette, Arm auswählen und
   unbekanntes Flansch→Kannen-Transform ausdrücklich kenntlich machen.
3. An **aktuellen** FR5 und 90°-IR40-Gripper mit seitlicher Griffpose
   statt veralteter kinematischer Holder/Greifer retargeten.
4. Virtuelle Kamera-Rekonstruktion und FR5-Kollisionsprüfung
   **unabhängig** und ohne versteckte Ground-Truth-Einspeisung testen.
5. Optional eine **echte Originalbewegung** im Vergleich zu
   kamera-rekonstruiertem und 6-Gelenk-FR5-Kinematikvideo zeigen,
   aber Motor-Firmware, Kontakt und echtes Flüssiggießen ehrlich trennen.

Relevante Dateien im **#18-Branch**, nicht im jetzigen main:
`robobarista/lerobot_native_latte.py`,
`robobarista/real_latte_dataset.py`,
`robobarista/realman_fk_bridge.py`,
`robobarista/external_pose_replay.py`,
`robobarista/latte_art_three_videos.py`,
`tests/test_real_latte_import.py`.

### 2. Review von [PR #18](https://github.com/Coffee-Tobi/robocasa/pull/18)

Aktuell **DRAFT, nicht mergebar**, Branch
`feature/latte-real-data-fluid-preview-2026-10-23`.
Bei der Bereinigung war dieser Branch gegenüber main
**divergiert** (106 Commits voraus, 10 zurück; Werte sind Momentaufnahme).
Sein historischer Inhalt überschreibt sonst den neuen seitlichen
PR#21-Griff und kann die in #22 entfernten F/T-Demokurven zurückbringen.

**Vorgehen:** Frischen Integrationsbranch vom aktuellen `main` erstellen,
jeweils einzelne neu nützliche Dateien/Tests aus #18 übernehmen;
keine unmodifizierten Dateien `coffee_workflow.py`,
`dual_ir40_concept.py`, `dual_tool_90_preview.py`, `README.md`
oder veraltete Workflows aus #18 kopieren. Mögliche wertvolle
Einzelmodule: `latte_dmp.py`, `moveit_scene_bridge.py`,
`vision_quality.py`, `vendor_runtime_preflight.py`,
`realman_fk_bridge.py`, `latte_art_three_videos.py`,
`coffee_real_slow.py`. Lizenz, Modellgrößen, Tag-Sichtbarkeit,
neueste Werkzeug-Frames, echtes Video und CI **pro Modul** prüfen.

### 3. Nebenbaustellen (nur nach #17/#18)

- #9 NewSpline-Wegpunkt-Retarget auf denselben 90°-Gripper;
- langsamer vollständiger Espresso-/Milchfilm, nun mit korrektem
  seitlichen Griff und ohne physikalisch unmögliche Tamperdrehung;
- echter dynamischer F/T- und Fingerkontakt, Kollisionsvolumen beider
  Greifer und CAD-Zuordnung; offene 3-Geräte-RS485/24V-Frage;
- latten-/schaumbeeinflusste Flüssigkeitsphysik und reale
  Ausgießerspitzen-Genauigkeit bisher **nicht** nachgewiesen.

## Laufzeit/Reproduzierbarkeit und Sicherheitsgrenzen

Verwendet werden GitHub Actions und MuJoCo/OSMesa ohne GPU.
`main` ist die einzige freigegebene Integrationsbasis; #17/#18
bleiben ältere Quellbranches, keine aktiven Sicherheitsfreigaben.
Die originalen FAIRINO-NewSpline-RPCs wurden nur mit Mock-SDK-Objekten
geprüft; der echte Controller wurde **nicht verbunden**.
Alle bisherigen mm-Zahlen für Kamera und Spout sind virtuelle
Simulationsmetriken, keine reale Werkzeuggenauigkeit.
Das FR5 bewegt sich in den Filmen durch Gelenkzustände und IK,
keine vollständige Roboter-Motor-/Kontaktsimulation bei allen Filmen.

## Einstiegssatz für den neuen Chat

> Wir machen mit Coffee-Tobi/robocasa weiter. Bitte lies zuerst
> `planning/CURRENT_STATUS_NEXT_CHAT_2026_10_23.md` auf main.
> Wir haben #12–#20 sortiert; heute zuerst **PR #17** und danach
> **PR #18** gemeinsam prüfen. Nie den alten Draft ganz nach main
> mergen, sondern echte Latte-Art-Demonstrationen und sinnvolle Module
> einzeln auf die neueste 90°-FR5-Seitengreifer-Konfiguration adaptieren.
