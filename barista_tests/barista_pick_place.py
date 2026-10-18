"""Panda pick-and-place with video and measured outcome.

Run:
MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/barista_pick_place.py
"""
from pathlib import Path
import shutil
import subprocess
import cv2
import numpy as np
import robosuite as suite

OUT = Path(__file__).resolve().parent
RAW = OUT / "pick_place_raw.mp4"
VIDEO = OUT / "pick_place.mp4"
W, H, FPS = 640, 480, 12

env = suite.make(env_name="Lift", robots="Panda",
                 has_renderer=False, has_offscreen_renderer=True,
                 use_camera_obs=False, camera_names="agentview",
                 camera_widths=W, camera_heights=H,
                 initialization_noise=None, horizon=1500, ignore_done=True)
writer = None
try:
    env.reset()
    robot = env.robots[0]
    arm = robot.arms[0]
    site = robot.eef_site_id[arm]
    gripper = robot.gripper[arm]
    cube_id = env.cube_body_id
    pad_ids = [env.sim.model.geom_name2id(name)
               for side in ("left_fingerpad", "right_fingerpad")
               for name in gripper.important_geoms[side]]
    writer = cv2.VideoWriter(str(RAW), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    if not writer.isOpened():
        raise RuntimeError("VideoWriter konnte nicht gestartet werden.")

    step_count = 0
    def hand():
        return np.asarray(env.sim.data.site_xpos[site]).copy()

    def cube():
        return np.asarray(env.sim.data.body_xpos[cube_id]).copy()

    def pad_center():
        return np.mean([env.sim.data.geom_xpos[i] for i in pad_ids], axis=0)

    def grasp():
        return bool(env._check_grasp(gripper=gripper, object_geoms=env.cube))

    def frame():
        rgb = env.sim.render(height=H, width=W, camera_name="agentview")
        writer.write(cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR))

    def action(target, grip):
        nonlocal_placeholder = None
        cmd = np.zeros(env.action_dim)
        cmd[:3] = np.clip((target - hand()) * 6.0, -0.16, 0.16)
        cmd[-1] = grip
        env.step(cmd)

    def travel(target, grip, limit=130, tol=0.003):
        for n in range(limit):
            action(target, grip)
            if n % 4 == 0:
                frame()
            if np.linalg.norm(target - hand()) < tol:
                break
        return 1000 * np.linalg.norm(target - hand())

    def stay(grip, count=25):
        target = hand()
        for n in range(count):
            action(target, grip)
            if n % 4 == 0:
                frame()

    print("Starte Panda Pick-and-Place mit Video...", flush=True)
    start = cube()
    for _ in range(8):
        frame()

    approach = start + np.array([0, 0, 0.105])
    print("Anfahrt, Fehler (mm):", round(travel(approach, -1), 2), flush=True)
    stay(-1, 15)

    ready = False
    for level in range(30):
        cp = cube()
        pad = pad_center()
        horizontal = np.linalg.norm(pad[:2] - cp[:2])
        vertical = pad[2] - cp[2]
        if horizontal > 0.012:
            print("ABBRUCH: Greifer zu weit seitlich versetzt.", flush=True)
            break
        if vertical <= 0.004:
            ready = True
            break
        next_z = max(hand()[2] - 0.008, cp[2] - 0.003)
        travel(np.array([cp[0], cp[1], next_z]), -1, limit=45, tol=0.0025)

    grabbed, lifted, maintained, placed = False, False, False, False
    if ready:
        print("Greifhöhe erreicht. Schließe Greifer.", flush=True)
        stay(1, 40)
        grabbed = grasp()
        print("Greifkontakt:", grabbed, flush=True)
        if grabbed:
            before_lift = cube()[2]
            lift_to = hand() + [0, 0, 0.09]
            print("Anheben, Positionsfehler (mm):", round(travel(lift_to, 1), 2), flush=True)
            lift_mm = (cube()[2] - before_lift) * 1000
            lifted = lift_mm > 50
            print(f"Würfelhub: {lift_mm:.1f} mm", flush=True)
            print("Greifkontakt nach Anheben:", grasp(), flush=True)
            if lifted and grasp():
                # Move 10 cm horizontally, while maintaining current gripping height.
                direction = np.array([0, 0.10, 0])
                destination = start + direction
                transport = hand() + direction
                print("Transport, Fehler (mm):", round(travel(transport, 1, limit=180), 2), flush=True)
                maintained = grasp()
                print("Greifkontakt nach Transport:", maintained, flush=True)
                if maintained:
                    # Relative displacement keeps the successful grasp geometry.
                    place_hand = hand().copy()
                    place_hand[2] -= 0.085
                    print("Absenken, Fehler (mm):", round(travel(place_hand, 1, limit=150), 2), flush=True)
                    stay(-1, 40)
                    withdraw = hand() + np.array([0,0,0.09])
                    travel(withdraw, -1)
                    stay(-1, 15)
                    final = cube()
                    error_mm = np.linalg.norm(final[:2] - destination[:2]) * 1000
                    height_mm = abs(final[2] - start[2]) * 1000
                    placed = error_mm < 20 and height_mm < 20
                    print("Ablagefehler horizontal:", round(error_mm, 1), "mm", flush=True)
                    print("Höhendifferenz zur ursprünglichen Tischlage:", round(height_mm, 1), "mm", flush=True)
    else:
        print("Greifhöhe nicht erreicht.", flush=True)

    print("Ergebnis:", "ERFOLG" if (grabbed and lifted and maintained and placed) else "NICHT ERFOLGREICH", flush=True)
    for _ in range(12):
        frame()
finally:
    if writer:
        writer.release()
    env.close()

if shutil.which("ffmpeg"):
    result = subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-i", str(RAW),
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "24",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(VIDEO)],
        capture_output=True, text=True)
    if result.returncode == 0:
        RAW.unlink()
        print("Video:", VIDEO, flush=True)
    else:
        print("FFmpeg-Fehler:", result.stderr[-400:], flush=True)
        print("Rohvideo:", RAW, flush=True)
else:
    print("FFmpeg fehlt. Rohvideo:", RAW, flush=True)
    print("Installation: sudo apt-get install -y ffmpeg", flush=True)
