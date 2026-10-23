# RoboBarista – Ein Projekt, klare Varianten, sauberer Entwicklungsstand

**Stand: 23. Oktober 2026. Einzige aktive Integrations-PR: #18; freigegebene 90°-Vorschau liegt bereits in main.**

### Entscheidung 3/18: PR #3 – KAMERA/REFACTOR BEHALTEN; PARALLELHALTERUNG ERSETZEN

Der Nutzer hat das maßlich vereinfachte MuJoCo-Bild mit FR5 → Inlux Force Torque Sensor (ITOSX1000FTS, derzeit visuelles Proxy) → 90° Dual Cobot Tool Adapter (ICACX1210XDUAL, derzeit visuelles Proxy) → zwei orthogonal montierte IR40-50-Fingergreifer **freigegeben** und explizit nach `main` übernehmen lassen.

Dazu wurde der **gezielte** [PR #19](https://github.com/Coffee-Tobi/robocasa/pull/19) mit bestandenem MuJoCo-Scene-/Screenshot-/POV-CI auf `main` zusammengeführt. Enthält `robobarista/dual_tool_90_preview.py`, `robobarista/compact_tags.py`, Tests und eigenständige Rendering-Workflow. Der ursprüngliche PR #3 war bereits auf main und seine Kamerafunktionen bleiben. **WICHTIG:** Der Haupt-Kaffeeablauf verwendet bis zu gesonderten 90°-Kollisionstests noch den älteren parallelen Greiferkonzept-Zweig; nicht als bereits umgestellt ausgeben. Das Bild bestätigt nur optische Werkzeugorientierung, nicht realen Griff, Lasten oder elektrischen Anschluss.

### Entscheidung 2/18: PR #2 – VERWORFEN / bereits ersetzt (Nutzerentscheidung)

[PR #2](https://github.com/Coffee-Tobi/robocasa/pull/2) bleibt geschlossen und ungemergt. Der PR hing ursprünglich am verworfenen MPC-Branch #1; die sinnvollen Aufräum- und Kamera-Funktionen wurden unabhängig davon bereits mit dem gemergten [PR #3](https://github.com/Coffee-Tobi/robocasa/pull/3) nach main übernommen. Keine zusätzliche Übernahme oder Löschung notwendig.

### Entscheidung 1/18: PR #1 – VERWORFEN (Nutzerentscheidung)

[PR #1](https://github.com/Coffee-Tobi/robocasa/pull/1) wurde **geschlossen, nicht gemergt**. Es war ein experimenteller eigener MPC-Optimizer und ein MuJoCo-Kontaktausleser. Für den praktischen FAIRINO FR5 verwenden wir vorrangig die offiziellen, bereits dokumentierten Steuerungs- und Schutzfunktionen:

- `SetAnticollision`, `SetCollisionDetectionMethod`, `CustomCollisionDetectionStart` für Controller-Kollisionserkennung und Reaktionsstrategien;
- `FT_GetForceTorqueRCS`, `FT_Guard`, `FT_Control` für gemessene Kraft und Kraftregelung **nur mit tatsächlich vorhandenem, konfiguriertem und kalibriertem kompatiblem F/T-Sensor**;
- bestehende `fairino_force_and_trajectory.py` / `fairino_sdk_preflight.py` für Read-only/Dry-run, ohne selbst entwickelte doppelte Laufzeitsteuerung.

Die FAIRINO-artige Offline-Kamerapfad-Interpolation erreichte gegenüber einer anderen glatteren Referenz 0,239 vs 0,471 mm Mittelabweichung zum rohen Kamerapfad. Der Vergleich wurde **ohne echte FAIRINO Firmware** durchgeführt und beweist daher noch keinen hardwareseitigen Vorteil. Herstellersteuerung bleibt bevorzugter Kandidat für späteren realen, gleichen Test. MPC und MuJoCo-Kontaktcode von PR #1 nicht in PR #18 übernehmen; Archivbranch erhalten. Sicherheits-/Kollisionsgeometrie-Simulation ist als unabhängiger Vortest weiterhin sinnvoll.

https://github.com/Coffee-Tobi/robocasa/pull/18

## Warum gab es 18 Pull Requests?

GitHub vergibt fortlaufende Nummern; die Zahl 18 bedeutet NICHT
18 gleichzeitig offene Änderungen. Bei diesem Cleanup:
- 10 PRs sind inzwischen in `main` **gemergt** (einschließlich gezieltem freigegebenen #19).
- 7 sind **geschlossen, nicht gemergt**, darunter die überholten
  Entwurfs-PRs #11, #13, #14, #16 und #17. Sie sind weiterhin
  über GitHub verlinkt, und ihre Branches wurden NICHT gelöscht.
- **2 Drafts bleiben offen** nach dem Entscheid, jeweils mit einer klaren Rolle.

| PR | Aufgabe | Vorgehen |
|---|---|---|
| [#18](https://github.com/Coffee-Tobi/robocasa/pull/18) | EIN Integrationsstand für kompakte Tags, Griffkonstruktion, Kaffeestoryboard, echten Demonstrationsimport, kinematische FR5-Wiedergabe und Milchproxy | CURRENT. Base=`main`; Tests prüfen, **nicht blind mergen** |
| [#9](https://github.com/Coffee-Tobi/robocasa/pull/9) | Eigenständiges experimentelles FAIRINO-NewSpline/MuJoCo-Motor-Replay | Parken, gezielt mit Herstellercontroller benchmarken |
| [#1](https://github.com/Coffee-Tobi/robocasa/pull/1) | Eigenständiger MPC/Kontaktsensor-/Greifphysik-Versuch | **VERWORFEN**, geschlossen, nicht gemergt; FAIRINO-Herstellerfunktionen bevorzugen |

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
