"""Closed-loop Panda end-effector position test, recorded as H.264 when FFmpeg is available.

Run:
MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/target_panda.py
"""
from pathlib import Path
import shutil
import subprocess

import cv2
import numpy as np
import robosuite as suite

HERE = Path(__file__).resolve().parent
RAW = HERE / "panda_target_raw.mp4"
FINAL = HERE / "panda_target_h264.mp4"
WIDTH, HEIGHT, FPS = 640, 480, 12

print("Starte Panda-Lift-Umgebung...", flush=True)
env = suite.make(
    env_name="Lift",
    robots="Panda",
    has_renderer=False,
    has_offscreen_renderer=True,
    use_camera_obs=False,
    camera_names="agentview",
    camera_widths=WIDTH,
    camera_heights=HEIGHT,
)
writer = None
try:
    env.reset()
    robot = env.robots[0]
    print("Controller:", type(robot.composite_controller).__name__, flush=True)
    print("Aktionsdimension:", env.action_dim, flush=True)
    # Obtain the actual measured 3D tool position from MuJoCo (in meters).
    def tool_pos():
        return np.asarray(env.sim.data.site_xpos[robot.eef_site_id], dtype=float).copy()

    start = tool_pos()
    target = start + np.array([0.07, 0.0, 0.0])
    print("Start (m):", np.round(start, 4), flush=True)
    print("Ziel  (m):", np.round(target, 4), flush=True)

    writer = cv2.VideoWriter(str(RAW), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        raise RuntimeError("VideoWriter konnte keine Datei anlegen.")

    def record():
        rgb = env.sim.render(height=HEIGHT, width=WIDTH, camera_name="agentview")
        writer.write(cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR))

    for _ in range(8):
        record()

    # The stock Panda Lift controller accepts normalized XYZ Cartesian deltas
    # in the first 3 action dimensions. Feedback prevents endlessly drifting.
    for step in range(100):
        actual = tool_pos()
        error = target - actual
        action = np.zeros(env.action_dim)
        action[:3] = np.clip(error * 8.0, -0.25, 0.25)
        env.step(action)
        if step % 3 == 0:
            record()
        if np.linalg.norm(error) < 0.012 and step > 6:
            print(f"Zielbereich nach {step + 1} Schritten erreicht.", flush=True)
            break
    else:
        print("Ziel noch nicht innerhalb von 1,2 cm erreicht.", flush=True)

    actual = tool_pos()
    print("Ende (m): ", np.round(actual, 4), flush=True)
    print("Abstand zum Ziel (cm):", round(np.linalg.norm(target - actual) * 100, 2), flush=True)
    for _ in range(8):
        record()
finally:
    if writer is not None:
        writer.release()
    env.close()

if shutil.which("ffmpeg"):
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-i", str(RAW),
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "24",
           "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(FINAL)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        print("H.264-Video:", FINAL, flush=True)
        RAW.unlink()
    else:
        print("FFmpeg konnte nicht umwandeln:", result.stderr[-800:], flush=True)
        print("Alternatives MP4:", RAW, flush=True)
else:
    print("FFmpeg ist noch nicht installiert; MP4v-Video:", RAW, flush=True)
    print("Optional: sudo apt-get install -y ffmpeg", flush=True)
