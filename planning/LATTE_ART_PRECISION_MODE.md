# RoboBarista – Latte-Art-Präzisionsmodus und Motorregler-Vergleich

**Stand: 22.10.2026 · FAIRINO FR5 in MuJoCo, vollständig virtuell**

## Warum die kleinen Bewegungen einen eigenen Regler brauchen

Bei der bisherigen kameragespeisten 15-FPS-Zufallsbewegung mit einem
einfachen Gelenk-PD-Regler (4-fache Zeitstreckung, aber größere Bewegungen)
lagen die **motorischen** Fehler bei **3,52 mm im Mittel und 9,69 mm
maximal am Roboter-TCP**. Die **kamerabasierte 6D-Schätzung** war in
demselben Lauf deutlich genauer: **max. 0,977 mm und 0,392° gegenüber
MuJoCo-Wahrheit**. Diese Werte messen unterschiedliche Fehlerquellen.
Die Bildmessung ist also für diese idealen virtuellen Bilder bereits
vergleichsweise gut, bei dynamischer Nachführung ist mehr zu gewinnen.

Für einen präziseren, **synthetisch vorgegebenen** Latte-Test haben wir
eine kleine zeitparametrisierte Geste von 4 mm lateraler Amplitude,
2,4 mm Querbewegung und 0,75° Kannenbewegung über 3 s **oberhalb einer
freien, kollisionsgeprüften Becher-Vorposition** erzeugt. Das ist keine
Demonstration direkt an einer echten Flüssigkeitsoberfläche und die
kleine Bewegung stammt **nicht** aus einem neuen optischen Kameramitschnitt.
Die tatsächliche Ausgießerspitze wird als starrer virtueller Offset zur
Robotertoolpose geführt; ihr Fehler wird separat zum TCP-Fehler gemessen.

Die identische Ruckig-geplante 6-Gelenk-Sollbahn wurde in MuJoCo mit
`mujoco.mj_step` und den sechs Drehmomentantrieben ausgeführt:

| Regler | Ø Ausgießerspitzenfehler ohne Störung | Maximaler Fehler ohne Störung | Maximaler Fehler **mit unbekannter 0,5-Nm-Handgelenkskraft** |
|---|---:|---:|---:|
| Legacy Positions-PD + Schwerkraftausgleich | 1,207 mm | 2,630 mm | 4,130 mm |
| Positions-PD + Geschwindigkeitssollwert | 0,732 mm | 1,531 mm | 3,630 mm |
| Computed Torque: Vorsteuerung mit `mj_inverse` | 0,0035 mm | 0,0238 mm | **5,609 mm** |
| **Robustes Computed Torque**: höheres PD und begrenzter Integralterm | **0,0017 mm** | **0,0105 mm** | **1,238 mm** |

Test: 4 mm, 3 s, 3181 Zeitpunkte mit 1 ms MuJoCo-Schritten. Alle drei
Regler liefen ohne im Modell erkannte Maschinen-/Armkollisionen oder
Motor-Drehmomentsättigung. Die besonders geringe Computed-Torque-
Abweichung wird dadurch begünstigt, dass **Soll-Dynamik und physikalisch
integriertes Modell identisch sind**. Eine wirkliche gefüllte Milchkanne,
Greifnachgiebigkeit, Sensoraussetzer, Wahrnehmungsrauschen, kalibrierter
Werkzeug-TCP, Flüssigkeitsoberfläche und Hardware sind nicht enthalten.
Daher darf **0,024 mm niemals als echte Roboterpräzision behauptet werden**.

### Womit wir experimentieren

- **`robobarista/precision_pour.py`**: synthetische mm-Geste in freier
  kollisionsgeprüfter Becher-Vorposition; 6D-IK; Ruckig-Gelenktrajektorie;
  separater Simulationslauf mit *exakt gleichem Referenzweg* für alle drei
  Drehmomentregler; Prüfung aller tatsächlichen 1ms-Konfigurationen;
  Fehler an Handgelenk und virtueller Ausgießerspitze.
- **`COLAB_ROBOBARISTA.ipynb`**: Film der bewegten Hand, Soll-Ist-
  Vergleichskurven und Tabellen.
- **`.github/workflows/precision-pour.yml`**: getestete 4-mm-Basis,
  ein **10-mm-Vergleich bei identischer 3-s-Zeit** sowie ein separater
  Versuchsaufbau mit **unbekanntem Zusatz-Drehmoment von 0,5 Nm**
  am Handgelenk. Dieses Drehmoment repräsentiert einen bewusst
  vereinfachten *Modellmismatch*, nicht die realistische Kraft einer
  gefüllten Milchkanne.

**Beide Zusatzexperimente sind in der MuJoCo-Simulation bereits als
einzelne Motorläufe vollständig ausgeführt worden.** Im 10-mm-Versuch
(identische drei Sekunden und 0,75° Winkel) lag der maximale
Ausgießerfehler bei **4,760 mm** (Positions-PD), **2,427 mm**
(Positions-/Geschwindigkeits-PD), **0,0244 mm** (Computed Torque)
und **0,0106 mm** (robustes Computed Torque).

Die **0,5-Nm-Laststörung** zeigt den Hauptbefund: der herkömmliche
Computed-Torque-Regler leidet unter Modellabweichungen, während die
zusätzliche Fehler-/Integralkorrektur den Maximalfehler in genau diesem
virtuellen Versuch von 5,609 auf 1,238 mm senken konnte.
**Keine garantierte Verbesserung für unbekannte reale Lasten,
Gerätekontakt oder echten Milchschaum.**

## Libraries, die das wirklich verbessern können

### 1. MuJoCo inverse dynamics: sofort sinnvoll, bereits integriert

Die von `mujoco.mj_inverse` berechneten Drehmomente berücksichtigen die
im Simulationsmodell angenommene Massenmatrix, Coriolis-Effekte und
Schwerkraft. Der bestehende alte Regler kommandierte lediglich
`G(q) + Kp*(q_ref - q) - Kd*qdot`. Der neue Computed-Torque-Regler
kommandiert mit vollständig model-basierter Vorsteuerung die
gewünschte Gelenk-Beschleunigung samt Fehlerrückführung. MuJoCo
dokumentiert diesen Anwendungsfall ausdrücklich.

Quelle: https://mujoco.readthedocs.io/en/stable/computation/

### 2. Ruckig: bereits im Projekt

Ruckig erzeugt lokal Geschwindigkeits-, Beschleunigungs- und
Jerk-begrenzte Gelenktrajektorien. Die 3a/3b-Experimente haben
bereits *nicht-null* Geschwindigkeit an inneren Messpunkten und
kein periodisches Anhalten. Wir verwenden die lokale
State-to-State-Berechnung, **nicht** den serverseitigen
Community-Multi-Waypoint-Aufruf.

Quelle: https://docs.ruckig.com/

### 3. Mink: wertvoll für spätere dynamische kartesische Korrektur

[Mink](https://github.com/kevinzakka/mink) ist eine direkt
MuJoCo-kompatible differentielle inverse Kinematik. Es kann
kartesische TCP-Aufgaben mit Gelenk- und Geschwindigkeitsgrenzen
sowie definierbaren geometrischen Kollisionsgrenzen lösen. Das
könnte unsere bisherige `scipy.optimize.least_squares`-IK ergänzen,
vor allem bei Becherbewegungen und den engen Maschinenanschlüssen.
**Mink ist noch nicht in unsere aktuelle FR5-Kontakt-/Regelschleife integriert.**
Quelle: https://kevinzakka.github.io/mink/

### 4. Crocoddyl / Pinocchio: Forschungsoption für optimale Bahnen

[Crocoddyl](https://github.com/loco-3d/crocoddyl) bietet DDP-basierte
optimal-control-Verfahren und benutzt Pinocchio für schnelle Ableitungen
des Roboterdynamikmodells. Es wäre interessant, wenn wir später
gleichzeitig Sollmuster, Kräfte, Gießgeschwindigkeit, Motorgrenzen und
dynamische Abweichungen optimieren wollen. Allerdings wäre das für
unser jetziges rein virtuelles Einarm-Projekt viel zusätzlicher
Aufwand. **Wir verwenden Crocoddyl/Pinocchio derzeit nicht.**

## Warum die Kamera trotz besserem Regler wichtig bleibt

Selbst bei **0,024 mm Fehler gegenüber der künstlichen Solltrajektorie**
kann eine Kamera-Sollpose mit ~1 mm Fehler gegenüber der tatsächlichen
Kanne die gesamte Latte-Art-Bewegung um etwa diese Größenordnung
verschieben. Das sind **nicht einfach addierbare Worst-Case-Messwerte**,
aber die Kamera-/Werkzeugkalibrierung bleibt eine eigenständige
Fehlerquelle.

**Noch nicht erledigt:** wirklich gefüllter und gegriffener Kannenarm,
Ausgießer-/Milchoberflächenkalibrierung, Timing der Flüssigkeitsabgabe,
Messung unter veränderter Last, Sensorrauschen, gesonderter dynamischer
Motorversuch für die aktuell optisch gemessene Ruckig-Bahn und reale FR5-
Herstellergrenzen. Bei Hardwareeinsatz sind Not-Halt und sicherheits-
zertifizierte Steuerung zwingend separat zu berücksichtigen.
