# Same-camera comparison: FAIRINO-style MoveL vs existing AprilTag/IK/Ruckig

**22 October 2026**. Source: completed [RoboBarista full Colab run #39641910809](https://github.com/Coffee-Tobi/robocasa/actions/runs/39641910809/artifacts/12365696486). Its ZIP contains `camera/camera_measurements.json`, `camera/robot_replay.json`, `continuous/camera_continuous_6d.json`, `continuous/ruckig_joint_trajectory.json`, and `motor/motor_tracking.json`.

### Input
- Same **31 synthetic RGB-D/AprilTag camera measurements** at **15 FPS**, recording lasts **2 s**; deliberately held-out ground truth was not fed into planning.
- Retimed uniformly to **6 s** for numerical path plotting.
- This input is *not* the new separate 20-second maximum-20-mm latte gesture. Camera demonstration here is generated and has much larger motions.

### Compared **offline** motion planning
1. **FAIRINO-like basic MoveL waypoint interpolation**: linear Cartesian movement with smoothstep speed at each of **all 31 identical observed pose positions**, stopping at each point. This is only a transparent *surrogate* for a simplistic `MoveL(..., blendR=-1)` strategy. We did NOT invoke the FAIRINO RPC, the controller's trajectory planner, hardware or SimMachine. SDK allows `MoveL`, `MoveJ`, `ServoCart`, blends and splines; these would yield different trajectories.
2. **Our currently saved path input**: smoothed Cartesian reference from all those same camera observations; already coupled to IK and a 6-second Ruckig joint trajectory in the earlier CI run. This new numerical comparison visualizes the recorded smoothing input, NOT an independently executed motor trajectory.

### Results

| Distance from linearly interpolated raw CAMERA path | FAIRINO-like offline MoveL-stop | Our camera-smoothed target |
|---|---:|---:|
| Mean | **0.239 mm** | 0.471 mm |
| Maximum | **0.821 mm** | 1.176 mm |

**These figures are NOT robot tracking errors and do NOT demonstrate FAIRINO SDK superiority**. A stop-go line follows raw, noisy sampled points closely by construction. Our smoothing intentionally trades exact point matching for continuity. Neither variant independently executed the FR5 motors during this comparison.

### Separately recorded **actual simulated FR5** vs camera

- Previous *kinematic IK replay*: max tracking discrepancy relative to AprilTag pose targets **0.000000668 mm**, an ideal solver result rather than motor performance.
- Previous dynamic MuJoCo **joint-PD** motor test of a different time-scaled version of this camera trajectory: average TCP tracking lag **3.521 mm**, maximum **9.690 mm**; measured using `mj_step`. These motor errors must not be attributed to the vendor-like offline MoveL curve.
- New 20-second 18-mm-spout `compare_latte_20s.py` with robust computed-torque control is in [separate experimental PR #5](https://github.com/Coffee-Tobi/robocasa/pull/5). At this checkpoint, its earlier video encode failed due odd height, subsequent CI attempts were cancelled. No verified new motor data yet.

### Replay

```sh
# Download successfully completed ZIP from run above first
python -m robobarista.compare_camera_planners \
  --artifact /path/to/RoboBarista-Full-Colab-Results.zip \
  --output runs/offline_camera
```

Outputs: `summary.json`, `paths.csv`, `camera_planner_comparison.png`.

A separate companion video was rendered from the original old GitHub Actions recording plus these two independently calculated offline XYZ paths, at three-times slowed playback. The separately saved old motor video overlays actual simulated joint target vs actual joint positions and the measured TCP error.

### Engineering recommendation

Do **not** rewrite our camera pipeline or throw away MuJoCo. Keep AprilTag detection and 6D spatial estimation; use FAIRINO built-in commands for actual robot joint/Cartesian motion, tool-frame calibration and gripper control. Compare `MoveL` with blending or vendor splines, `ServoCart`, and our reference/Ruckig+feedback on the **same recorded 20s trajectory** after connecting the real robot or official supported controller simulator. Collect synchronized actual TCP and independent pitcher spout tracking at 100+ Hz if possible. Never infer real hardware performance from the stop-go surrogate.

Official vendor SDK:
- https://manual.fairino.support/latest/SDKManual/PythonRobotMovement.html
- https://github.com/FAIR-INNOVATION/fairino-python-sdk
