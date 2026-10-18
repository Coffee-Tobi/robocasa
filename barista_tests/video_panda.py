"""Record a short Panda robot motion video in the robosuite Lift environment.

Run:
MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/video_panda.py
"""
from pathlib import Path
import cv2
import numpy as np
import robosuite as suite

OUT = Path(__file__).resolve().parent / "panda_motion.mp4"
WIDTH, HEIGHT, FPS = 640, 480, 15
CAMERA = "agentview"  # angled view; more informative than frontview

print("Initialisiere Panda und schräge Kamera...", flush=True)
env = suite.make(
    env_name="Lift",
    robots="Panda",
    has_renderer=False,
    has_offscreen_renderer=True,
    use_camera_obs=False,
    camera_names=CAMERA,
    camera_widths=WIDTH,
    camera_heights=HEIGHT,
)
writer = None
try:
    env.reset()
    writer = cv2.VideoWriter(
        str(OUT), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT)
    )
    if not writer.isOpened():
        raise RuntimeError("MP4-Encoder konnte nicht gestartet werden.")

    def frame():
        rgb = env.sim.render(height=HEIGHT, width=WIDTH, camera_name=CAMERA)
        rgb = np.flipud(rgb)
        writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    def segment(title, action, steps, sample_every=2):
        print(title, flush=True)
        for i in range(steps):
            env.step(action)
            if i % sample_every == 0:
                frame()

    neutral = np.zeros(env.action_dim)
    # Hold the start pose briefly so it is easy to recognize.
    for _ in range(12):
        frame()

    # The Panda Lift environment normally uses a delta end-effector controller.
    # We test a sustained movement in both directions.
    forward = neutral.copy()
    forward[0] = 0.8
    backward = neutral.copy()
    backward[0] = -0.8

    segment("Arm bewegt sich in +X-Richtung...", forward, 40)
    segment("Arm bewegt sich in -X-Richtung...", backward, 40)

    close = neutral.copy()
    close[-1] = 1.0
    open_grip = neutral.copy()
    open_grip[-1] = -1.0
    segment("Greifer: erste Richtung...", close, 20)
    segment("Greifer: Gegenrichtung...", open_grip, 20)

    for _ in range(10):
        frame()
    print(f"Video erstellt: {OUT}", flush=True)
    print(f"Dateigröße: {OUT.stat().st_size / 1024 / 1024:.2f} MB", flush=True)
finally:
    if writer is not None:
        writer.release()
    env.close()
