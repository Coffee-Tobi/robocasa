# RoboBarista — Open Source reuse, FAIRINO SDK audit and physical FR5 gates

Research checkpoint: **22 October 2026**. This is a factual implementation
audit and hardware-readiness plan, NOT a physical verification.

## 1. Sources checked and what we actually integrated

| Source | License / status | What is useful | Integration decision |
|---|---|---|---|
| [RoboCasa365](https://github.com/robocasa/robocasa) `robocasa/models/fixtures/coffee_machine.py` | MIT | Press-to-enable fixture state, mug-under-outlet geometric check, start button and coffee-pouring visual sites | **ADAPTED** in `robocasa_coffee_adapter.py`; named nozzle/button *sites* in `barista_cell.py`, symbolic espresso only when cup passes real scene geometry gate in `coffee_workflow.py`. Not a real grouphead. |
| RoboCasa machine objects `CoffeeMachine036/...` | Assets require external download ~10 GB, fixture meshes not in normal repo | Improved cosmetic scene | **NOT imported**: largely coffee dispenser/button concepts rather than validated portafilter/steam-wand geometry; risk of breaking our collision scenes. |
| [Espresso CAD project](https://github.com/AlexandruAchilleF/automated-espresso-machine-cad) | MIT | Potential 235×290×340mm 58mm grouphead geometry | **REJECTED**. Inspected actual `espressor_ansamblu_complet.step` and `.stl`; STEP has **EMPTY DATA** (comments only) and STL contains **1 triangle**, no volumetric machine. Missing claimed independent 58mm group STEP and portafilter STL. Repro tests in `test_external_espresso_assets.py`. |
| [ridxm Latte Art Robot](https://github.com/ridxm/latte-art-robot) and [latte-pour-demos](https://huggingface.co/datasets/ridxm/latte-pour-demos) | GitHub repo has **no LICENSE file** at inspection; dataset license must be checked separately | ~40 ≈20s demonstration episodes, 3 camera streams at 20Hz, joint and gripper feedback, training pipeline via LeRobot | **SCHEMA ONLY**, no code/weights copied and no 1:1 use of dual Realman trajectories. `latte_dataset_bridge.py` exports strictly separated camera vs held-out truth to an *intermediate* JSONL structure, NOT native LeRobot v3. |
| [ABB YuMi Latte-Art course project](https://github.com/TranThanhTuan2509/Automated-Latte-Art-with-Using-ABB-s-YuMi) | MIT | End-to-end staged dual-arm coffee concept and kinematics | Inspiration only, not 6DOF FAIRINO algorithm or manufacturer SimMachine model |
| [COFFAIL coffee skill dataset](https://zenodo.org/records/18212316) | Dataset: check license before weights/data reuse | Real coffee-task demos and failures (cup transfers, pouring etc.), not necessarily steam/espresso | External validation benchmark candidate only, no local import |
| [FAIRINO Python SDK](https://github.com/FAIR-INNOVATION/fairino-python-sdk) | Apache 2.0 | Vendor trajectories, FK/IK, tool, F/T, gripper, collision and telemetry APIs | Already `fairino_native_spline.py`, `fairino_force_and_trajectory.py`. **NEW:** telemetry read-only test and selection from 8 vendor IK branches in `fairino_sdk_preflight.py` |

Original RoboCasa source code and MIT license:
https://github.com/robocasa/robocasa/blob/main/LICENSE

We use **adapted logical features**, not original coffee-machine mesh files.
Keep source attribution if redistributed; do not claim real machine mechanics.

## 2. Specific FAIRINO SDK features still worth implementing

Official [movement docs](https://manual.fairino.support/latest/SDKManual/PythonRobotMovement.html),
[common settings](https://manual.fairino.support/latest/SDKManual/PythonRobotCommonSettings.html),
[force control](https://manual.fairino.support/latest/SDKManual/PythonRobotForceControl.html),
[status/IK](https://manual.fairino.support/latest/SDKManual/PythonRobotStatusInquiry.html),
[trajectory recording](https://manual.fairino.support/latest/SDKManual/PythonRobotTrajectoryRecurrence.html).

| Priority | Vendor API | Why it matters; prerequisite |
|---|---|---|
| P0 – tool calibration | `ComputeToolCoordWithPoints`, `SetToolCoord`, `SetWObjCoord`, `GetTCPOffset` | Robot should control the **spout tip**, not just bare wrist; camera/world & two-gripper offset must be measured. |
| P0 – actual feedback | `GetActualTCPPose`, `GetActualJointPosDegree`, `GetActualJointSpeedsDegree`, `GetMotionQueueLength`, `GetRobotMotionDone`, `GetRobotEmergencyStopState`, `GetSafetyStopState` | Synced *actual* hardware vs measured AprilTag spout trajectories. **Implemented as read-only mock.** |
| P0 – payload model | `SetLoadWeight`, `SetLoadCoord`, `GetTargetPayload`, `GetTargetPayloadCog` | Must account for full/empty pitcher + front mounted two grippers; requires measured mass/CG. No hardware settings sent. |
| P0 – real force sensor | `FT_SetConfig`, `FT_Activate`, `FT_SetZero`, `SetForceSensorPayload`, `SetForceSensorPayloadCog`, `FT_GetForceTorqueRCS`, `FT_Guard` | Bayonet insertion and steam wand side loads; must calibrate wrench frame and tool payload first. Existing force charts are SYNTHETIC. |
| P0 – branch selection | `TCFToAllJoint` (up to 8 IK branches), `GetForwardKin` | Avoid unexplained wrist flips & joint-limit surprises. **Implemented mock branch ranking**; collision/path check remains separate. |
| P1 – planned pouring | `NewSplineStart` / `NewSplinePoint` / `NewSplineEnd`, `MoveL` with `blendR` | Existing official-name dry-run adapter. Need supported actual controller/SimMachine to measure firmware interpolation. |
| P1 – streamed correction | `ServoMoveStart`, `ServoCart`, `ServoMoveEnd` | Real-time spout height and milk-flow feedback; vision 3 fps or 15 fps alone does not yield validated 100+Hz smooth control. |
| P1 – recording and playback | `SetTPDParam`, `SetTPDStart`, `SetWebTPDStop`, `LoadTPD`, `MoveTPD` | Direct physical demonstration of taught movements on an actual FAIRINO, as distinct from video-only AprilTag tracking; vendor logging period 2/4/8ms. |
| P1 – trajectory lookahead | `LoadTrajectoryLA`, `MoveTrajectoryLA`, `SetTrajectoryJSpeed` | Requires already recorded/uploaded proper FAIRINO **JOINT** trajectory file, not raw RGB-D poses. Our current mock does not create that file. |
| P1 – hardware motion safety | `SetAnticollision`, `SetCollisionDetectionMethod`, safety speed caps, `GetRobotEmergencyStopState` | Physical contact, crash response. We will not set arbitrary vendor safety thresholds from virtual data. |
| P2 – filtering / live correction | LIN/ARC FIR filtering, acceleration smoothing | Can improve smoothness but must verify deviations against target when pouring fine lines. |
| P2 – feedback gripper | FAIRINO gripper activation/position and current feedback | Two IR40-50 tool actuators may NOT be independently supported via one gripper interface; tool changer, independent I/O and address must be verified. |

SDK snapshot reader and 8-branch selector:
`robobarista/fairino_sdk_preflight.py`.
It **never opens a real robot connection or authorizes motion**.

## 3. Readiness verdict

Virtual tests from previously completed jobs (different setups and
therefore **not** independent hardware validation):

- Synthetic RGB-D AprilTag camera reconstruction (20s small pour):
  **mean 0.259 mm**, max **0.534 mm**, P95 **0.437 mm**.
  The maximum already exceeds the requested 0.5mm upper target.
- Framewise ideal kinematic MuJoCo IK tracking adds near-zero *extra* error
  because actual qpos is **assigned directly**. No motor load dynamics.
- Separate 20s perfectly model-matched simulated computed-torque motor test
  had max **0.0127 mm**, but it **lacked pitcher fluid mass, mechanical
  two-gripper backlash/friction, camera disturbances, contact and mismatch**.
  It must NOT be added naïvely as a real hardware error guarantee.
- The 11-stage espresso movie moves to **safe predocks only**. No real
  portafilter grasp/docking/locking, tamper pressure, water/steam, fluid flow.
  The FT indicators are not real measured forces.

**No-go for autonomous serving with hot pressurized hardware today.**
**Go for a supervised dry-cup/empty-pitcher FIRST physical prototype after
workcell/tool calibration and formal safety setup** (not an unsafe autonomous
release).

### Missing gates required before wet espresso / latte art

1. Actual camera intrinsics/extrinsics and rigid calibrated
   pitcher-to-spout-to-gripper transforms, including occlusions, latency,
   real measured tip error from independent cameras.
2. Actual two IR40 grippers/axial adapter/fixed cup clamp:
   grasp force, slipping, tool masses and centre of gravity, arm-reachable
   workcell with correct mechanical envelope.
3. Real motion control/firmware NewSpline and ServoCart measurement under
   loaded and unloaded configurations, synchronization and independent
   multi-axis spout tracking at representative speeds.
4. Actual portafilter gasket/bayonet insertion force-torque, positional
   witness of lock, calibration and human-supervised contact conditions.
5. Actual steam wand and milk pitcher nozzle contact, force/tip feedback,
   temperature, contamination/water safety and fail-safe stop logic.
6. Physically validated control of cup position, grinder/tamper, brewing
   pressure/time, dosing and washing.
7. Milk-flow/crema physics or measured feedback and real latte-art pattern
   quality; shape error/flow rate needs measured wet trials.
8. Reproducible end-to-end success rate, fault injection (lost tag, slip,
   late frame, collision/temperature triggers), fault recovery.

The programmatic gate `hardware_readiness_gate()` is intentionally **FALSE**
until explicit evidence for those physical tests is supplied.
