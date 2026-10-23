# Kleine, realistische AprilTags direkt an Objekten (optische Simulation)

Profil **legacy**: bisherige große 82–90mm Messplaketten, die in
Erfolgstests mit 960×720 aufgenommen wurden. Dieser Status bleibt
unverändert verfügbar.

Profil **compact**: bewegte Markierungen sind echte, fest mit dem
Objektkoordinatensystem gekoppelte MuJoCo-Messplaketten:

| Objekt | ID | Außenmaß | Montage |
| --- | ---: | ---: | --- |
| Milchkanne: Oberseite am Griff | 71 | **44mm** | flacher Adapter über dem Kannengriff; außerhalb der Milch |
| Milchkanne: Vorderseite | 23 | **54mm** | tangentiales, zweifach abgestütztes Plättchen am runden Kannenkörper |
| Milchkanne: Rückseite | 72 | **54mm** | entgegengesetzte tangentiale Platte |
| Plastikbecher | 70 | **36mm** | seitlicher Clip knapp unter dem Trinkrand |

Die Seitenmarkierungen dürfen nicht direkt auf die gewölbte Metallfläche
geklebt werden: die aus der Blechkrümmung resultierende Verformung
verletzt die ebene Markerannahme. Der flache Adapter ist in der
virtuellen Szene als Platte **mit Abstandshaltern** dargestellt, nicht
als schwebendes Quadrat. In der Realität wäre ein gedruckter,
reinigechter Befestigungsclip notwendig.

Für Pose-Schätzung wird die **bedruckte aktive Innenkante**
verwendet: `440/512 * Außenmaß`, nicht das Außenmaß einschließlich des
weißen Rands. Der zugeordnete metrische Körper->Marker-Transform
und die MuJoCo-Halterung teilen sich `tag_mounts.py`. Wichtig:
kleinere Marker verursachen bei identischen Kameras mehr
Winkelfehler und häufigere Sichtbarkeitsausfälle. Unser früherer
Tracking-Erfolg <0.6 mm **darf nicht auf das kompakte Layout
übertragen werden**, solange die neue Messung nicht bestätigt ist.

## Reproduzieren

```bash
# High resolution for smaller AprilTags; no fake GT fallback.
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa
export ROBOBARISTA_TAG_PROFILE=compact
python -m robobarista.camera_joint_replay --capture-only \
  --seed 14 --frames 13 --fps 5 --width 1280 --height 960 \
  --output runs/compact_tags_1280
```

Prüfen: `summary.json` mit `camera_valid_frames=13` und
`camera_two_view_frames=13`; sonst **FAIL**. Die getestete
Kamerabewegung ist eine virtuelle Demonstration, keine
Mensch- oder echte Roboteraufnahme.

Der 36mm-Bechertag gehört derzeit zur Gerätegeometrie, wird
aber durch `camera_joint_replay.py` **noch nicht für die 6D-
Kannenschätzung verwendet**. Die separate optische Cup-Pose-
Messung wäre ein eigener nächster Test.

Stationäre 105mm-Tags dienen weiterhin als Kalibrierrefenzen
auf eigenen befestigten Ständern, nicht als bewegte Objekte.
Diese zweite Art von Marker kann später an echte Geräteflächen
versetzt werden, sobald deren metrische CAD-Modelle vorliegen.

## Latte-Art-Libraries

`ridxm/latte-art-robot`: echtes Aufnahme-/LeRobot-Training,
aber andere Doppelarme und bei Prüfung kein klares
GitHub-LICENSE. Nicht übernommen; JSONL-Datenbrücke in
`latte_dataset_bridge.py` bereits in main.

`spaghettiandmeatball/little-lattes`: experimentelle 3D-
Latte-Art-App mit interaktiven Fluss-/Höhenparametern, aber
bei Prüfung keine Lizenzdatei. Keine Kopie eingebaut.

`p-sun/typescript-fluid-simulator` (MIT): tatsächlich
weiterverwendbare, vereinfachte zweidimensionale Simulation.
Dafür liegt separat der PR für einen echten fluidbasierten
Sidecar-Test vor; er ersetzt die 3D-Greif-/Spout-Physik nicht.
