"""Render three stages of a Panda robot's motion in robosuite.

Run with: MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/move_panda.py
"""
from pathlib import Path

import numpy as np
import robosuite as suite
from PIL import Image

OUTPUT = Path(__file__).resolve().parent / "panda_frames"
OUTPUT.mkdir(parents=True, exist_ok=True)

print("Erstelle Panda-Umgebung...", flush=True)
env = suite.make(
    env_name="Lift",
    robots="Panda",
    has_renderer=False,
    has_offscreen_renderer=True,
    use_camera_obs=False,
    camera_names="frontview",
    camera_widths=640,
    camera_heights=480,
)
try:
    env.reset()
    def save_frame(label):
        img = env.sim.render(height=480, width=640, camera_name="frontview")
        img = np.flipud(img).copy()
        target = OUTPUT / f"{label}.png"
        Image.fromarray(img).save(target)
        print(f"{target.name}: Helligkeit {img.mean():.1f}", flush=True)

    def move(action, count):
        for _ in range(count):
            env.step(action)

    zeros = np.zeros(env.action_dim)
    save_frame("01_start")

    # Panda's standard controller uses normalized end-effector deltas.
    # A small x-direction command moves the arm, without requiring IK code.
    right = zeros.copy()
    right[0] = 0.25
    print("Bewege Arm...", flush=True)
    move(right, 12)
    save_frame("02_armbewegung")

    # The last action dimension is the gripper command in Lift.
    grip = zeros.copy()
    grip[-1] = -1.0
    print("Bewege Greifer...", flush=True)
    move(grip, 12)
    save_frame("03_greifer")

    print(f"Fertig! Bilder unter: {OUTPUT}", flush=True)
except Exception as exc:
    print(f"Fehler: {type(exc).__name__}: {exc}", flush=True)
    raise
finally:
    env.close()
