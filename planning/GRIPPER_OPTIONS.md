# RoboBarista – zwei Werkzeugaufsätze für den FAIRINO FR5

Stand 2026-10-22. **Recherche und Modellzuordnung, keine mechanische Montierung/Greifsimulation.**

## Werkzeug A: Milchkanne und Siebträger an ihren Griffen

**Bevorzugter Bibliotheksbaustein:** Robotiq 2F-85 (85 mm Nenngreifweite) mit zwei schmalen, rutschhemmend beschichteten Greiffingern. Beispiele:
- [Google DeepMind MuJoCo Menagerie: Robotiq 2F-85 v4 (vollständiges MJCF)](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85_v4)
- [robosuite Robotiq85Gripper](https://robosuite.ai/docs/modules/robots.html)

Sinn: Milchkanne **am Griff**, nicht mit hohen Kräften am heißen dünnen Kannenkörper. Siebträger **am Griff**, nicht am heißen Metallsieb.

Vor der Montage prüfen: lichte Grifföffnung, Durchmesser und Querschnitt beider Griffe; Flansch/Adapter, Greifschwerpunkt, erwartetes Drehmoment einer gefüllten Kanne; Abstand zum Brühgruppenanschluss; Temperatur, Reinigbarkeit und Lebensmittelsicherheit der Fingerspitzen.

## Werkzeug B: Kunststoffbecher

**Bevorzugter Bibliotheksbaustein:** [robosuite Robotiq140Gripper](https://robosuite.ai/docs/modules/robots.html) (140-mm-Nennöffnung) mit **breiten, nachgiebigen, leicht konkaven Kontaktpads** als erst noch zu konstruierendem Fingeraufsatz.

Sinn: Der im MuJoCo-Test verwendete Übungsbecher hat eine Oberkante von ungefähr **90 mm Durchmesser**. Die Finger sollen den Becher rund um die stabile Wand fassen, ohne ihn punktuell einzudrücken. Eine 85-mm-Öffnung kann hier für den Becherrand knapp sein. **Robosuite liefert den Greifer, nicht das maßgeschneiderte Soft-Pad oder automatisch eine passende FR5-Halterung.** Bei einem dünnen offenen Becher ist eine Sauglösung keine sichere universelle Alternative. Becherfüllung, Verformung und Nasskontakt müssen geprüft werden.

## Werkzeugwechsel

Die beiden **getrennten Endeffektoren** können in einem späteren MuJoCo-Modell jeweils über einen Werkzeugwechsler mit eigener `TCP`- und `collision_geometry`-Definition montiert werden. **Ein automatischer Werkzeugwechsler ist aktuell weder in MuJoCo noch physisch vorhanden.** Alternative: ein Robotiq-Grundgreifer mit zwei wechselbaren Finger-**Spitzen**, was das Gesamtgewicht reduziert, aber einen schnellen sicheren Spitzenwechsel voraussetzt.

Bereits im Repo:
- [`robobarista/tool_heads.py`](../robobarista/tool_heads.py): maschinenlesbare Profile für beide Aufsätze sowie Zuordnung zu den 11 Kaffeephasen.
- [`COLAB_ROBOBARISTA.ipynb`](../COLAB_ROBOBARISTA.ipynb): zeigt beide Aufsätze, Hersteller-Bibliothek und Zuordnung an.

Noch ausstehend:
1. Reale Gripper-MJCF-Assets direkt in die FR5-MJCF-Kinematikkette einfügen, Anschlussflansch kalibrieren.
2. Position der gefassten Milchkanne und des Siebträgers relativ zum **Greifer-TCP** statt zum bisherigen einfachen Roboter-TCP messen.
3. Richtige Kollisionskörper aller beweglichen Greiferfinger und des Werkzeugwechslers in **jede** Trajektorienprüfung aufnehmen.
4. Dynamische Greifkontakte, zulässige Greifkraft und Reibung validieren; Becher darf nicht zusammengedrückt werden.
5. Nach Werkzeugmontage alle bisher getesteten 11 Armbewegungen und sieben Anfahrwege **erneut** kollisionsprüfen; alte Freigaben gelten dann nicht automatisch.

**Das Projekt kann daher noch keine reale Kanne, keinen Siebträger und keinen Becher greifen.**
