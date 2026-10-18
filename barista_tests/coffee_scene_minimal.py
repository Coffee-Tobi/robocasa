"""PrepareCoffee minimalist scene smoke test, using original RoboCasa task logic.

Requires an original Objaverse mug model in models/assets/objects/objaverse/mug/.
Run: MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/coffee_scene_minimal.py
"""
from pathlib import Path
import traceback
import robocasa
import robosuite
import cv2
import numpy as np
from robocasa.environments.kitchen.composite.brewing.prepare_coffee import PrepareCoffee

# Preserve the task's real mug placement and success checks; omit only the
# optional random cabinet distractor from _get_obj_cfgs, not the task mug.
original_cfgs = PrepareCoffee._get_obj_cfgs
def coffee_only(self):
    cfgs = original_cfgs(self)
    return [cfg for cfg in cfgs if cfg.get("name") == "obj"]
PrepareCoffee._get_obj_cfgs = coffee_only

root = Path(robocasa.__file__).resolve().parent
mugs = sorted((root / "models/assets/objects/objaverse/mug").glob("*/model.xml"))
print("Installierte Original-Objaverse-Tassen:", len(mugs), flush=True)
if not mugs:
    raise SystemExit("FEHLT: Original-Tasse. Bitte zuerst nur die mug-Modelle aus objaverse.zip extrahieren.")

env = None
try:
    env = robosuite.make(
        "PrepareCoffee", robots="PandaOmron",
        has_renderer=False, has_offscreen_renderer=True,
        use_camera_obs=False, render_camera="robot0_agentview_center",
        obj_registries=("objaverse",),
        use_distractors=False, seed=0,
    )
    env.reset()
    print("ERFOLG: Original-PrepareCoffee-Szene mit Tasse geladen.", flush=True)
    print("Erfolgskriterium der Aufgabe bleibt unverändert.", flush=True)
    cameras = list(env.sim.model.camera_names)
    print("Verfügbare Kameras:", cameras, flush=True)
    preferred = ("robot0_agentview_center", "robot0_agentview_right", "robot0_robotview")
    cam = next((name for name in preferred if name in cameras), cameras[0])
    rgb = env.sim.render(height=480, width=640, camera_name=cam)
    output = Path(__file__).resolve().parent / "robocasa_coffee.png"
    cv2.imwrite(str(output), cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR))
    print("Bild gespeichert:", output, flush=True)
except Exception:
    print("Szene noch unvollständig. Letzter Fehler:", flush=True)
    traceback.print_exc(limit=8)
    raise
finally:
    if env is not None:
        env.close()
