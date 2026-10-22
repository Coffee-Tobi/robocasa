# FAIRINO's real smoothing methods instead of naïve MoveL stop/go

22 Oct 2026. This implements **official named Python SDK interfaces** in a no-hardware rehearsal. Original FAIRINO source is https://github.com/FAIR-INNOVATION/fairino-python-sdk and precise Python API descriptions are https://manual.fairino.support/latest/SDKManual/PythonRobotMovement.html

## Discovered methods

1. **MoveL** accepts `blendR >= 0` mm for nonblocking tangent/corner path blends; the default `blendR=-1` blocks to finish at every point. The earlier offline comparison unintentionally modeled blocking MoveL at all 31 camera frames; this is **not** the best FAIRINO strategy.
2. **NewSplineStart(type=1, averageTime=...)**, **NewSplinePoint(desc_pos=...,tool=...,user=...,lastFlag=...,blendR=...)**, **NewSplineEnd()**. Type 1 defines camera-derived position waypoints as path points. Type 0 blends arc transitions and can deviate geometrically from measured points. This appears the best initial test for a preplanned flowing latte gesture. The documentation is inconsistent about `lastFlag` (prototype 0/1 vs example -1): this preview uses the prototype's 0/1, and needs actual SDK/firmware integration verification.
3. **ServoMoveStart() / ServoCart(...) / ServoMoveEnd()**: recommended later for streaming 6D correction as flow/pitcher height changes, but the command transmission interval and watchdog need hardware timing tests, not an arbitrary 15 Hz vision stream.

## Implemented

`robobarista/fairino_native_spline.py`

- reads the **31 independently captured simulated RGB-D AprilTag pitcher poses at 15fps** from passed GitHub Actions archive.
- extracts 11 keyframes (every third observation), recovers translation and *fixed-axis* XYZ orientation, converts 3D camera pose to formal FAIRINO `[x,y,z,rx,ry,rz]` **mm/degrees**.
- names and calls the actual documented methods in order on a **mock recording object** for both options; throws if a real robot object is supplied. This is a command-interface rehearsal **not a simulation of FAIRINO firmware trajectory**. No vendor SDK installation is necessary to run this mocked call ordering.
- Requests `averageTime=2000` ms over 11 points for **nominal 20 seconds**, but controller timing is **not guaranteed** and output is not a hardware timing measurement. Stops at the last point for blended MoveL.
- Explicit transform chain is mandatory before any real movement: `T_base_TCP = T_base_world * T_world_pitcher * T_pitcher_TCP`, requiring both calibrated camera-to-robot extrinsics and the rigid pitcher-grip adapter transform. The bundled identity matrices are for mock demonstrations **only**; they are not valid hardware values.

### Reproduce

Download artifact from [previous fully passed E2E run](https://github.com/Coffee-Tobi/robocasa/actions/runs/39641910809/artifacts/12365696486), then:

```bash
pip install numpy scipy
python -m robobarista.fairino_native_spline \
  --artifact /path/to/RoboBarista-Full-Colab-Results.zip \
  --output runs/fairino_spline_dryrun \
  --duration 20 --stride 3 --blend-radius-mm 0.5
```

JSON results have exact documented API method names and all parameters; they never connect to real hardware. A light GitHub CI workflow additionally uses a mock to check all command sequences and calibration transformations.

## What we still **cannot** claim

- That FAIRINO NewSpline generated a 20-second physical robot path: no connected robot controller or verified FAIRINO SimMachine runtime was available.
- That blend/curvature error is <= 0.5 mm in reality: those values are only *requested radius*, not measured output error.
- That raw camera pose already is the correct robot TCP target. Our older replay deliberately used an idealized pitcher/TCP correspondence.
- That NewSpline automatically produces the best latte art: endpoint constraints, continuous pitcher tilt, nozzle/cup surface alignment, force-balanced handling, final tool stiffness and actual milk flow remain unmeasured.
- That the **old 31-frame camera gesture** is the same as the separate **20s 18mm-tip kinematic/motor test**; these are distinct experiments.

Next: validate SDK APIs and camera-to-flange calibration in FAIRINO SimMachine (if it supports the target API) or a supervised FR5 test, then compare independent AprilTag measurements of the ACTUAL pitcher tip against our custom Ruckig trajectory under identical constraints.
