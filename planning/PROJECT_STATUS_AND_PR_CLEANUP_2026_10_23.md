# RoboBarista – Ein Projekt, klare Varianten, sauberer Entwicklungsstand

**Stand: 23. Oktober 2026. Einzige aktive Integrations-PR: #18**
https://github.com/Coffee-Tobi/robocasa/pull/18

## Warum gab es 18 Pull Requests?

GitHub vergibt fortlaufende Nummern; die Zahl 18 bedeutet NICHT
18 gleichzeitig offene Änderungen. Bei diesem Cleanup:
- 9 ältere PRs sind bereits in `main` **gemergt**.
- 6 sind **geschlossen, nicht gemergt**, darunter die überholten
  Entwurfs-PRs #11, #13, #14, #16 und #17. Sie sind weiterhin
  über GitHub verlinkt, und ihre Branches wurden NICHT gelöscht.
- **3 Drafts bleiben offen**, jeweils mit einer klaren Rolle.

| PR | Aufgabe | Vorgehen |
|---|---|---|
| [#18](https://github.com/Coffee-Tobi/robocasa/pull/18) | EIN Integrationsstand für kompakte Tags, Griffkonstruktion, Kaffeestoryboard, echten Demonstrationsimport, kinematische FR5-Wiedergabe und Milchproxy | CURRENT. Base=`main`; Tests prüfen, **nicht blind mergen** |
| [#9](https://github.com/Coffee-Tobi/robocasa/pull/9) | Eigenständiges experimentelles FAIRINO-NewSpline/MuJoCo-Motor-Replay | Parken, gezielt mit Herstellercontroller benchmarken |
| [#1](https://github.com/Coffee-Tobi/robocasa/pull/1) | Eigenständiger MPC/Kontaktsensor-/Greifphysik-Versuch | Parken, nicht mit visuellen Dummy-Kontakten verwechseln |

`main` bleibt der getestete Ausgangspunkt; aktive Entwicklung nur in
`feature/latte-real-data-fluid-preview-2026-10-23`, bis PR #18 fertig ist.
Historische Experimente dienen als Quelle/Backup, nicht als parallele
Produktlinien. Keine Branches oder alte Daten gelöscht.

## Zwei Tassenmodi, **nicht** vermischen

| Modus | Physisches Objekt | Welches Greifwerkzeug? | Aktueller Status |
|---|---|---|---|
| `handle_mug` (aktuelles Video) | Keramiktasse mit äußerem Henkel (+X am freistehenden Becher). Am gehaltenen Prop ist der Tassenkörper >100 mm VOR den Backen entlang Tool +Y versetzt; der Henkel liegt zwischen den schmalen IR40-B-Backen | **Henkel** | MuJoCo VISUAL/rigid attached; Geometrie geprüft, aber **kein echter Griffkontakt, keine Griffkraft, keine Offset-Kollisionsfreigabe** |
| `takeaway` | Griff- und Henkelloser Plastik-/Einwegbecher | Separate breite Backen klemmen Becherkörper seitlich, NICHT den Boden | Separate Variante, visuelles Konzept, noch keine Kraftprüfung |

Der alte interne Name `rb_plastic_cup` bleibt aus Kompatibilitätsgründen
in der ursprünglichen MuJoCo-Stationslogik erhalten, selbst wenn die Szene
den Henkel-Tassenmodus rendert. Das ist ein technischer Alias, kein
Beleg dafür, dass wir im neuen Film einen Einwegbecher greifen wollen.
Beim späteren Aufräumen sollte dies durch ein einheitliches
`vessel_id`/`cup_mode`-Interface ersetzt werden.

**Wichtig:** Auch im neuen Film bewegt das Modell nur eine starr am
Greifer aufgehängte Tasse und versetzt die freie Tasse zwischen Stationen
symbolisch. Der reale Schritt *an den Henkel fahren → Backen schließen →
Tasse anheben* wird damit NICHT dynamisch simuliert. Kein echter physischer
Körperkontakt wurde nachgewiesen.

## AprilTags: EINE aktive Darstellung, alternative Mess-Baseline

- Aktives Kaffeevideo: `--marker-profile compact --cup-style handle_mug`.
  Kleine Platten auf Pitcher, Tasse, Siebträger und festen Stationen
  (je nach Objekt **24–42 mm Außenmaß** inklusive Weißrand), eindeutige
  IDs für zusätzlich starr gehaltene Werkzeugkopien.
- Alter Benchmark: `--marker-profile legacy` verwendet **82–105 mm**
  große Markierungen und darf ausschließlich als Vergleichstest
  erscheinen; **nicht stillschweigend im Vorzeigevideo**.
- Für Realman-Latte-Art-Daten führten bisher bei großen Tags 13/13
  gerenderte Kameraposen zu höchstens **0,561 mm** virtuellem
  Kannenpositionsfehler. Die kleineren Marker erreichten in einem
  älteren, NICHT direkt identischen Benchmark max. **0,593 mm** bei
  nur 4/8 stationär erkannten Markern. Die vierte Kamera ist
  experimentell, wird nicht als qualitätsverbessernd behauptet,
  bevor sie denselben Datensatz bestanden hat.
- Spout-Ziel 0,2–0,5 mm; die Zahl der idealen Arm-IK aus MuJoCo
  beschreibt **nicht** die Abweichung der echten Werkzeugspitze.
- Keine extern kalibrierte echte Kamera, kein echter Greifer,
  keine realistische Marker-Temperatur-/Milchreinigung getestet.

## Wiederverwendung vorhandener OSS – Duplikate kennzeichnen

| Bedarf | Unsere primäre Lösung | Status |
|---|---|---|
| Kamera-Tag-Messung | Pupil AprilTags mit gerenderten RGB-D-Bildern | Läuft virtuell; kompakte Sicht noch verbessern |
| Kalibrierung | Kalibr/Caliscope/Pupil als Optionen | Echte Extrinsik noch unkalibriert |
| Robotermotion | FAIRINO-SDK-Signaturprüfung, MoveIt2-Planungs-Szene, später echte SimMachine | Virtuelle Pose-IK läuft, Herstellerhardware/-firmware noch nicht |
| Gießbewegung | LeRobot realer ridxm-14D-Datensatz + offizielles RealMan RM65 URDF + optional DFKI DMP | Demonstration importiert; Original-TCP/Kannenmount unkalibriert |
| **2D Latte-Flüssigkeitsdemo** | **MIT** `p-sun/typescript-fluid-simulator` via vorhandenem `latte_fluid_adapter.py` und `latte-fluid-sidecar.ts` | Bereits in `main`, echte externe 2D Solverintegration, **keine** echte Milchschaumphysik |
| Einfacher Offline-Fallback | `latte_fluid_proxy.py` | Extra CPU 2D Animation (nur Fallback), ersetzt MIT nicht |
| Realistische mehrphasige Flüssigkeit | FluidLab `LatteArt-v0` als separates Zukunftsexperiment | Nicht installiert/mit FR5/GPU getestet; alte Python-3.7/Taichi-Pipeline |

**Keine Flüssigkeits-Animation – auch nicht die MIT-2D-Variante – darf als
physikalische Vorhersage der Milchschaum-Rosette ausgegeben werden.**

## Reproduzierbare aktuelle Vorführung (Ubuntu 22.04 / Colab CPU)

```bash
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
python -m pytest -q tests/test_handle_mug_compact_video.py tests/test_compact_apriltags.py
python -m robobarista.coffee_workflow \
  --marker-profile compact --cup-style handle_mug \
  --camera-view follow --frames-leg 3 --width 760 --height 570 --fps 7 \
  --output runs/handled_mug_compact
# Video: runs/handled_mug_compact/coffee_workflow.mp4
```

Der neue Clip zeigt 11 symbolische Espresso-/Milch-/Latte-Stationen,
nicht die tatsächlich demonstrierte Gießkinematik aus dem Realman-Repo.
Das **andere** LatteArt-Kamerareplay benutzt echte Realman-RM65-Gelenkwerte,
hat aber bislang nur eine unkalibrierte virtuelle Kanne ohne echten
Tassen-Henkelkontakt. Beide Experimente bleiben sauber getrennt.

## Nicht bestandene Gates vor einem Roboterkauf / echter Latte Art

1. Abgleich von echtem Siebträger-/Milchkannen-/Tassenmaß mit beiden
   IR40-Werkzeugen, tatsächliche Griffkraft, Fingerkontakt und sichere
   Rotationshalterung; erst dann eigene physische Tassenaufnahme.
2. Fortlaufende Planung von Vorposition → kontrollierter Griff → Dock →
   sichere Rückfahrt; nicht bloß starr mitbewegte Proxies.
3. Externe Kamerakalibrierung, alle kompakten Tags sichtbar oder
   redundante Beweise, **spout**-statt-Kannenmittelpunkt-Fehlerbudget.
4. SimMachine/echte FAIRINO Motorfeedbackverläufe statt perfekter
   MuJoCo `qpos`-Kinematik.
5. Temperatur-/Feuchte-/Milchfluss-/Crema-Modell und Laborvalidierung;
   keine echten heißen Getränke bei ungeprüfter Bewegung.
6. Einheitliche Objektnamen, nachvollziehbare Modell- und Datensatzlizenzen,
   entdoppelte `latte_fluid_proxy`-/MIT-Solver-Einstiegspunkte.

Diese Datei ist eine Übersicht – separate PR-Branches bleiben für
historische Referenz erhalten, sind aber keine aktiven Feature-Pläne.
