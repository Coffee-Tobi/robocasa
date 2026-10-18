"""PrepareCoffee minimalist scene smoke test, using original RoboCasa task logic.

Requires an original Objaverse mug model in models/assets/objects/objaverse/mug/.
Run: MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/coffee_scene_minimal.py
"""
from pathlib import Path
import traceback
import shutil
import subprocess
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
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg fehlt. Bitte installieren, bevor das Video erstellt wird.")
    final = output.parent / "robocasa_coffee.mp4"
    partial = output.parent / "robocasa_coffee_in_progress.mp4"
    command = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", "640x480", "-r", "12", "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "veryfast",
        "-crf", "24", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(partial),
    ]
    proc = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for step in range(120):
            env.step(np.zeros(env.action_dim))
            rgb = env.sim.render(height=480, width=640, camera_name=cam)
            frame = np.ascontiguousarray(np.flipud(rgb), dtype=np.uint8)
            proc.stdin.write(frame.tobytes())
    finally:
        if proc.stdin and not proc.stdin.closed:
            proc.stdin.close()
        stderr = proc.stderr.read().decode("utf-8", errors="replace")
        code = proc.wait()
    if code != 0:
        raise RuntimeError("Videocodierung fehlgeschlagen: " + stderr[-800:])
    partial.replace(final)
    print("FERTIG: H.264-Video gespeichert:", final, flush=True)

except Exception:
    print("Szene noch unvollständig. Letzter Fehler:", flush=True)
    traceback.print_exc(limit=8)
    raise
finally:
    if env is not None:
        env.close()
