# RoboBarista — FAIRINO FR5 in MuJoCo

**Research prototype, virtual only.** This repository is centered on one simulated FAIRINO FR5 servicing a coffee station with grinder/integrated tamper, espresso machine, pitcher, milk dispenser, cup and cleaning area. None of the symbolic coffee/liquid/grasp actions should be mistaken for real machine operations.

## Start here

- **[Open the one unified Google Colab](COLAB_ROBOBARISTA.ipynb)** — main workcell, three RGB-D + AprilTags, room reconstruction, collision-checked station staging, 11-stage coffee storyboard, camera-based replay, Ruckig and motor/precision experiments.
- **[Read the coffee workflow source](robobarista/coffee_workflow.py)** — the 11-stage storyboard and rendering.
- **[See the smooth following camera source](robobarista/follow_camera.py)** — follows just behind and above the FR5 wrist; a small static whole-cell view remains inset. For old video style use `--camera-view overview`. **The following video camera is cinematic ONLY; it never replaces the fixed calibrated RGB-D cameras for AprilTag tracking.**
- **[Last verified complete demonstration (MP4 download artifact)](https://github.com/Coffee-Tobi/robocasa/actions/runs/39641910809/artifacts/12365696486)** — completed successfully on 21 Oct 2026 on `main`, **before** following camera was implemented.
- **[New following-camera preview test](https://github.com/Coffee-Tobi/robocasa/actions/workflows/pov-preview.yml)** — renders and uploads `coffee_workflow.mp4` from this branch when CI passes. Do not claim a new video has been verified until the run succeeds.

## What works vs. what remains an experiment

| Component | Current honest status | Code |
|---|---|---|
| Scene + 3 fixed RGB-D cameras + AprilTags | Complete previous integrated virtual test PASS; virtual camera calibration remains idealized | `barista_cell.py`, `geometry.py`, `camera_joint_replay.py` |
| Room 3D reconstruction | Virtual point cloud generated and checked, not SLAM | `room_reconstruction.py` |
| FR5 stations + collision-aware safe predocks | Seven staging targets checked, **real contacts are not performed** | `safe_staging.py` |
| 11-stage espresso/milk/latte visual sequence | Kinematic video and symbolic liquids, **no real flow / gripping** | `coffee_workflow.py` |
| Arm POV movie + small fixed overview | Implemented in branch; dedicated CI validation pending | `follow_camera.py` |
| Two Inlux IR40-50 grippers | Separate visual concept and isolated stationary contact bench (bench test PASS); **not yet mass/torque-certified or coupled to moving FR5** | `dual_ir40_concept.py`, `dual_ir40_physics.py` |
| Robust computed torque | Separate 3-second mm-scale simulation baseline PASS; 0.0105 mm ideal max and 1.238 mm max with 0.5 Nm artificial torque; no real payload | `precision_pour.py` |
| MPC | Experimental joint-space MPC implemented; full MuJoCo benchmark not verified | `mpc_control.py` |
| Steam-wand contact sensor | Isolated contact-force prototype only, **actual wand/pitcher grasp geometry missing** | `contact_sensing.py` |
| Real 20 s latte art / milk dynamics | **Not yet implemented** | — |

## Reproduce a coffee video

From a setup with dependencies in `requirements.txt`, MuJoCo OSMesa, FFmpeg and the project vendor assets:

```bash
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa
python -m robobarista.coffee_workflow \
  --camera-view follow \
  --output runs/coffee_pov --frames-leg 6 --fps 12
```

Output: `runs/coffee_pov/coffee_workflow.mp4`, `summary.json`, `recipe.json`.
To switch back to the old overview: `--camera-view overview`.

**What the video does:** per recipe step the arm travels from home to a checked free-space pre-dock and back. A small explicitly artificial, safe tilted sway is shown as latte art. Three RGB-D tracking cameras and the additional follow camera are *different camera systems*.

## Main source map (do not remove these dependencies)

- `robobarista/barista_cell.py` and `workcell_constraints.py`: machines, stations, robot geometry, clearance rules.
- `robobarista/coffee_workflow.py` and `follow_camera.py`: recipe/video and movable virtual viewpoint.
- `robobarista/camera_joint_replay.py`, `geometry.py`, `room_reconstruction.py`: pixel-based AprilTag/point cloud tests.
- `robobarista/robot_replay.py`, `continuous_motion.py`, `motor_follow.py`, `precision_pour.py`, `mpc_control.py`: IK, jerk-limited path, motor control and experimental MPC.
- `robobarista/dual_ir40_concept.py`, `dual_ir40_physics.py`, `contact_sensing.py`: optional hardware mock-ups and contact experiments.
- `planning/`: historical test assumptions, measured virtual benchmarks, CAD/material decisions. Use with caution; not all experiments were performed on the current branch.
- `.github/workflows/`: independent reproducible regression and video artifact jobs; these modules import other parts of the project.

## Why fewer notebooks?

Three former standalone Colabs (safe replay, safe staging, workcell) were superseded by **`COLAB_ROBOBARISTA.ipynb`** and removed **on the cleanup branch only**; their contents are retrievable through Git history. We retain the underlying Python packages and tests because the unified notebook and the accepted GitHub Actions pipeline rely on them.

## Safety and precision

The core prototype does not yet have an installed live robot, actual tool-changer/grip forces, dynamic cup/pitcher fluid, calibrated vendor cameras, compliant steam contact or production-safe collision handling. Numerical ideal-model precision is **not** real mechanical or latte-art precision.
