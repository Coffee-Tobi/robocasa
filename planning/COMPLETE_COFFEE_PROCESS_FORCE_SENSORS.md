# RoboBarista 11-stage espresso + steam + latte storyboard with two IR40-50s

22 October 2026.

## Working preview

A fully rendered **34.5-second, 12fps (414 frames) 11-stage** MP4 is produced
by `robobarista/full_barista_sensor_video.py`. It reuses the existing
MuJoCo full workcell, FAIRINO FR5, dual IR40-50 visualization fixtures and
collision-checked *predock* paths. It adds German explanations, a rear/side
arm viewpoint, a smaller overall camera view and explicit training-only
force/torque traces.

### Stages

1. Align at portafilter parking place — IR40-50 A holds handle adapter
2. Approach grinder with integrated tamper — no actual grind/tamp contact
3. Approach brew group with portafilter — animated *concept* dial and virtual
   resistance 0.08–0.74 Nm (not observed physical contact)
4. Position plastic practice cup — IR40-50 B wide jaws
5. Show espresso extraction as simulated colored-disc state
6. Retrieve pitcher — IR40-50 A handle fixture
7. Approach milk dispenser — symbolic dosing
8. Approach steam wand — synthetic 0–1.2 N **lateral** F/T trace, contact *illustration*
9. Position espresso cup for pour — IR40-50 B
10. Show five phase pour gesture — kinematic illustrative, not milk flow
11. Approach rinser/parking — symbolic cleaning

## Sensor decisions in the new code

Source `robobarista/fairino_force_and_trajectory.py`.

- FAIRINO supports `FT_GetForceTorqueRCS()` yielding six values
  `[Fx,Fy,Fz,Mx,My,Mz]`, and `FT_Guard(...)`.
- `portafilter_lock_monitor` checks **an illustrative** torsional threshold
  and stops at overload; a resistance increase alone is **NOT LOCK PROOF**.
  Mechanical lock needs an independently calibrated final rotation/position,
  group-head geometry, locking detents and an actual force sensor.
- `steam_wand_side_contact` monitors small lateral force against an
  illustrative `0.45 N` detection and `2.5 N` stop threshold. **These are
  NOT safe hardware force limits**. Actual sensor axis/tool moment compensation,
  steam wand deflection, thermal effects and nozzle clearance are unknown.
- `illustrative_contact_curve` is a separate deterministic mock model of
  side force and lock torque, **not MuJoCo `mj_contactForce` or robot FT data**.
- Two simulated IR40-50 grippers are rigidly attached visualization examples,
  **not** jaw contact, electronic I/O or a mechanical tool changer.

## Trajectory LA integration

Official Python docs: https://manual.fairino.support/latest/SDKManual/PythonRobotTrajectoryRecurrence.html

`LoadTrajectoryLA(name,mode,errorLim,type,precision,vamx,amax,jmax,flag)`
and `MoveTrajectoryLA()` are supported in the SDK. **Crucially**
`LoadTrajectoryLA` preprocesses an existing **vendor-format uploaded joint
trajectory**, not any arbitrary RGB-D camera waypoints. The documented upload
sequence uses `TrajectoryJUpLoad(...)`. We deliberately do not fabricate
the vendor joint log structure and do not call `MoveTrajectoryLA`.
`LookaheadConfig` and `lookahead_sequence(RecordingFairino,...)` exercise
the exact documented API call, with mock uploaded-file status only.

For the camera-derived latte motion, keep `NewSpline` as the immediate
official FAIRINO playback approach; optional `LoadTrajectoryLA` can be
evaluated after creating a **verified** joint log through official robot
recording / supported trajectory tools.

## Safety & next physical development

A complete physically credible simulation would require:
- Actual machine portafilter coupling CAD/constraint and a measured tightening
  torque profile, pose/angle measurements, controlled jaw grasp and friction;
- Contact collision meshes for pitcher, steam wand, and cup;
- Force/torque sensor plugin or MuJoCo contacts, wrench compensation for payload
  and steam-wand spring stiffness;
- A verified stop-state machine, independent camera/sensor timeout guards;
- Actual fluid/foam/crema dynamics and optional latte pattern recognition;
- Real robot velocity, temperature, nozzle and electrical hazard constraints.

**Never transfer the synthetic force thresholds from this demo directly to a
physical FR5.** No mechanical contact is currently authorized by existing
collision-preflight.

## Reproduce

```bash
MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
python -m robobarista.full_barista_sensor_video \
  --output runs/full_barista_sensors --frames-leg 9 \
  --capture-fps 6 --repeat 2 --width 640 --height 480

python -m pytest -q tests/test_fairino_force_and_trajectory.py
```

Output: `complete_barista_with_sensor_concepts.mp4`, `summary.json`,
`step_events.json`. Workflow:
[Complete Full Coffee F/T Storyboard](https://github.com/Coffee-Tobi/robocasa/actions/workflows/full-barista-contacts.yml).

Official F/T manual:
https://manual.fairino.support/latest/SDKManual/PythonRobotForceControl.html
