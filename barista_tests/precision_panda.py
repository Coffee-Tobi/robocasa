"""Measure Panda positioning precision across multiple targets.

Run:
MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/precision_panda.py
"""
import csv
from pathlib import Path
import numpy as np
import robosuite as suite

out = Path(__file__).resolve().parent / "precision_results.csv"
env = suite.make(env_name="Lift", robots="Panda", has_renderer=False,
                 has_offscreen_renderer=False, use_camera_obs=False)
try:
    env.reset()
    robot = env.robots[0]
    arm = robot.arms[0]
    site_id = robot.eef_site_id[arm]

    def position():
        return np.asarray(env.sim.data.site_xpos[site_id]).copy()

    origin = position()
    offsets = [("x_plus_7cm", [0.07, 0, 0]),
               ("x_return", [0, 0, 0]),
               ("y_plus_5cm", [0, 0.05, 0]),
               ("z_plus_4cm", [0, 0, 0.04]),
               ("return_home", [0, 0, 0])]
    rows = []
    print("Ausgangsposition (m):", np.round(origin, 5), flush=True)
    for name, delta in offsets:
        target = origin + np.asarray(delta, dtype=float)
        reached = False
        for step in range(250):
            error = target - position()
            action = np.zeros(env.action_dim)
            action[:3] = np.clip(error * 6.0, -0.18, 0.18)
            env.step(action)
            if np.linalg.norm(target - position()) < 0.001:
                reached = True
                break
        actual = position()
        distance_mm = 1000 * np.linalg.norm(target - actual)
        rows.append([name, step+1, f"{distance_mm:.3f}", reached])
        print(f"{name}: {distance_mm:.3f} mm Fehler nach {step+1} Schritten"
              f" (unter 1 mm: {reached})", flush=True)

    with out.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["target", "steps", "error_mm", "within_1mm"])
        writer.writerows(rows)
    print(f"Messwerte gespeichert: {out}", flush=True)
finally:
    env.close()
