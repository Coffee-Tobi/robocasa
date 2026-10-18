"""Experimental Panda pick-and-place in robosuite's Lift environment.

Run:
MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/pick_place_panda.py

This is a first diagnostic, not a guaranteed successful grasp. It logs measured
grasp, lift and placement success separately instead of claiming success from video.
"""
from pathlib import Path
import shutil
import subprocess

import cv2
import numpy as np
import robosuite as suite

ROOT = Path(__file__).resolve().parent
RAW = ROOT / "pick_place_raw.mp4"
FINAL = ROOT / "pick_place_h264.mp4"
W, H, FPS = 640, 480, 12
CUBE_WIDTH = 0.044

print("Starte Panda Pick-and-Place...", flush=True)
env = suite.make(
    env_name="Lift", robots="Panda",
    has_renderer=False, has_offscreen_renderer=True,
    use_camera_obs=False, camera_names="agentview",
    camera_widths=W, camera_heights=H,
    horizon=1200, ignore_done=True,
    initialization_noise=None,
)
writer = None
try:
    env.reset()
    robot = env.robots[0]
    arm = robot.arms[0]
    site = robot.eef_site_id[arm]

    def hand():
        return np.asarray(env.sim.data.site_xpos[site]).copy()

    def cube():
        return np.asarray(env.sim.data.body_xpos[env.cube_body_id]).copy()

    def grasping():
        return bool(env._check_grasp(gripper=robot.gripper[arm], object_geoms=env.cube))

    writer = cv2.VideoWriter(str(RAW), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    if not writer.isOpened():
        raise RuntimeError("Konnte Video nicht erzeugen.")

    tick = 0
    def send(delta, grip):
        nonlocal_dummy = None  # no state modified here
        action = np.zeros(env.action_dim)
        action[:3] = np.clip(delta * 7.0, -0.3, 0.3)
        action[-1] = grip
        env.step(action)

    def record():
        rgb = env.sim.render(height=H, width=W, camera_name="agentview")
        writer.write(cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR))

    def hold(grip, steps=20):
        for i in range(steps):
            send(np.zeros(3), grip)
            if i % 3 == 0:
                record()

    def goto(label, target, grip=-1.0, steps=160, tolerance=0.006):
        print(f"{label}: Ziel {np.round(target, 3)}", flush=True)
        for i in range(steps):
            err = target - hand()
            send(err, grip)
            if i % 3 == 0:
                record()
            if np.linalg.norm(target - hand()) < tolerance and i > 6:
                break
        error = 1000 * np.linalg.norm(target - hand())
        print(f"  Greifer-Positionsfehler: {error:.1f} mm, Schritte: {i+1}", flush=True)
        return error

    initial_cube = cube()
    initial_hand = hand()
    print("Würfel Startposition (m):", np.round(initial_cube, 4), flush=True)
    print("Greifer Startposition (m):", np.round(initial_hand, 4), flush=True)
    for _ in range(10):
        record()

    # These are tentative offsets from the tracked cube center to the end-effector
    # site and may need adjustment based on measured grasp success.
    safe_z = max(initial_hand[2], initial_cube[2] + 0.16)
    approach = np.array([initial_cube[0], initial_cube[1], safe_z])
    grasp_pose = np.array([initial_cube[0], initial_cube[1], initial_cube[2] + 0.015])

    goto("Anfahrt oberhalb", approach)
    goto("Vorsichtig absenken", grasp_pose, steps=160, tolerance=0.008)
    hold(grip=1.0, steps=35)

    did_grasp = grasping()
    print("Griff erkannt:", did_grasp, flush=True)

    lift_pose = grasp_pose + np.array([0, 0, 0.13])
    goto("Anheben", lift_pose, grip=1.0)
    lift_height = cube()[2] - initial_cube[2]
    print(f"Würfel angehoben um {lift_height*100:.1f} cm", flush=True)

    # Use a modest lateral move to leave the workspace while keeping grasp closed.
    destination = initial_cube + np.array([0.0, 0.10, 0.0])
    move_pose = lift_pose + np.array([0.0, 0.10, 0.0])
    goto("Transport", move_pose, grip=1.0)
    target_release = grasp_pose + np.array([0.0, 0.10, 0.01])
    goto("Abstellen", target_release, grip=1.0, tolerance=0.01)
    hold(grip=-1.0, steps=30)
    goto("Greifer zurückziehen", move_pose, grip=-1.0)
    hold(grip=-1.0, steps=15)

    final_cube = cube()
    placement_error_cm = np.linalg.norm(final_cube[:2] - destination[:2]) * 100
    displacement_cm = np.linalg.norm(final_cube[:2] - initial_cube[:2]) * 100
    final_height_delta_cm = (final_cube[2] - initial_cube[2]) * 100
    placed = placement_error_cm < 2.0 and abs(final_height_delta_cm) < 2.0
    print("Würfel Endposition (m):", np.round(final_cube, 4), flush=True)
    print(f"Horizontale Verschiebung: {displacement_cm:.2f} cm", flush=True)
    print(f"Abstand zum Ablageziel: {placement_error_cm:.2f} cm", flush=True)
    print("Bewertung:", "PLACEMENT PASS" if placed and lift_height > 0.05 else "NO SUCCESS", flush=True)
    print("Hinweis: Griffstatus wurde unmittelbar nach dem Schließen geprüft.", flush=True)
finally:
    if writer is not None:
        writer.release()
    env.close()

if shutil.which("ffmpeg"):
    result = subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-i", str(RAW),
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "24",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(FINAL)],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        RAW.unlink()
        print("Video (H.264):", FINAL, flush=True)
    else:
        print("FFmpeg-Konvertierung nicht möglich:", result.stderr[-500:], flush=True)
        print("Video (MP4v):", RAW, flush=True)
else:
    print("Video (MP4v):", RAW, flush=True)
