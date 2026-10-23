# RoboBarista integration checkpoint – 23 October 2026

This branch extends **feature/compact-realistic-apriltag-fixtures**. It is
intentionally not merged into main until CI and hardware assumptions are reviewed.
The 11-step espresso movie is still **predock-only**. Latte Art demos and
three comparison movies are simulated, not proven milk/coffee production.

## Implemented in this branch

- Fixed stale GitHub Actions dependency: FAIRINO spline mock now consumes the
  same workflow run's fresh 11-frame camera captures, with at least two views,
  rather than a hardcoded artifact and an inconsistent ==3 requirement.
- Per-run AprilTag marker config, no global mutation when alternating
  compact and legacy runs in one Python process.
- Explicit measured-*virtual-pitcher* quality report, with 0.5 mm maximum
  position target and 8/8 station-visibility goal. A capture can PASS while
  both quality goals fail. Neither represents real calibrated spout accuracy.
- Optional DFKI movement_primitives positional DMP over AprilTag-reconstructed
  **virtual spout xyz**, not simulated ground truth. Small endpoint retargets
  (<=20 mm). No learned orientation, milk-flow physics, robot joint actions
  or deployment authorization.
- Reused original-vs-tracked-vs-kinematic-FR5 Latte Art videos from the
  earlier independent three-video experimental branch. The replay
  interpolates and assigns simulated joint positions, not motor dynamics or
  controller firmware. Slow source gesture is also optional in camera capture.
- ROS2 MoveIt CollisionObject publisher for our **nominal**, uncalibrated
  workcell keep-out AABBs. Default action is JSON export only. ROS2 publish
  needs explicit CLI flag and live MoveIt subscriber; NEVER drives robot.
- FAIRINO/SimMachine/ROS2 runtime readiness inspection that only reads local
  environment/installations. Does NOT connect to controllers.

## Quick CPU-only checks on the development branch

```bash
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
python -m pytest -q tests/test_latte_dmp_integration.py tests/test_integration_quality_moveit.py
python -m robobarista.moveit_scene_bridge --output runs/moveit2/nominal_collision_scene.json
python -m robobarista.vendor_runtime_preflight --output runs/fairino_runtime_preflight.json

# Real Python DFKI package, OPTIONAL dependency:
python -m pip install 'movement_primitives>=0.9,<1'
python -m robobarista.camera_joint_replay --capture-only --marker-profile compact \
  --frames 13 --fps 5 --width 1280 --height 960 --output runs/compact_tags
python -m robobarista.vision_quality --capture runs/compact_tags \
  --output runs/compact_tags/vision_quality.json
python -m robobarista.latte_dmp \
  --camera runs/compact_tags/camera_measurements.json \
  --output runs/compact_tags/offline_virtual_dmp.json \
  --virtual-tip-offset-m -.069 0 .136 --goal-offset-mm 1 0 0
```

All DMP output is explicitly **OFFLINE_POSITIONAL_DMP_ONLY**. The
supplied pitcher/spout offset is a VIRTUAL CAD estimate. Rotating the pitcher
can move its real tip by millimetres; physical validation is necessary.

### Optional three matched Latte Art films

```bash
python -m robobarista.camera_joint_replay --capture-only \
  --latte-art-source --frames 61 --fps 3 --width 960 --height 720 \
  --output runs/latte_capture
python -m robobarista.latte_art_three_videos \
  --capture runs/latte_capture --output runs/latte_three --fps 8
```

This test requires multiple RGB-D renders and can take longer on Codespaces CPU.

## MoveIt2: actual open-source provider, not our own planner

FAIRINO documentation currently offers ROS2 Humble (Ubuntu 22.04), package
`fairino5_v6_moveit2_config` and `frcobot_ros2`:
- https://manual.fairino.support/latest/ROSGuide/moveIt2.html
- https://support.fairino.us/portal/en/kb/articles/getting-started-with-the-fairino-moveit2-plugin
- https://github.com/FAIR-INNOVATION/frcobot_ros2

On a suitable computer **separate from our tiny OSMesa Codespace**:

1. Install ROS2 Humble, MoveIt2 and colcon per upstream documentation.
2. Obtain the matching FAIRINO version and compile
   `fairino_msgs`, `fairino_description` and
   `fairino5_v6_moveit2_config`. Consult upstream about version-specific
   package naming (e.g. `fairino_hardware_v3_9_5`).
3. Start `ros2 launch fairino5_v6_moveit2_config demo.launch.py`
   with **fake/mock hardware** and VERIFY the selected ros2_control backend.
   Do not connect a real FR5 to this experimental cell.
4. Export `python -m robobarista.moveit_scene_bridge` as JSON. In the
   ROS2 environment and only after verifying a matching `world` TF, run
   `python -m robobarista.moveit_scene_bridge --publish-to-planning-scene`.
   Check all collision boxes in RViz before using MoveIt Plan. Real
   portafilter insertion needs a separately validated contact/force controller.

This first bridge only publishes fixture boxes. We have **not** proven an
FR5 MoveIt plan, IK branch continuity, loaded gripper collision, or execution.

## FAIRINO SimMachine: manufacturer controller unavailable in this session

FAIRINO offers official SimMachine as a VM or Docker image, including
vendor controller firmware rather than our SciPy spline approximation:
- https://www.fairino.be/downloads
- https://fairino.support/

The runtime image must be obtained/installed separately on a suitable host.
No SimMachine image is checked into this repository. No host or controller
has been contacted during this code change. After installation:

1. Confirm from the vendor console that the endpoint is the **virtual**
   controller, not a real robot IP.
2. Verify tool ID, user coordinate, software version and actual declared
   `NewSplineStart/NewSplinePoint/NewSplineEnd` signatures.
3. Read the controller state with the read-only FAIRINO preflight before
   any virtual trajectory; compare reported TCP/joint trace against
   recorded camera and DMP output. Avoid automatic physical-controller
   actuation or treating vendor emulation as a certified safety result.
4. Independently confirm robot payload/tool mass, gripper dimensions,
   cup/crema surface, temperature, contact force and emergency stops
   on supervised physical tests before using hot liquid.

`python -m robobarista.vendor_runtime_preflight` checks only tools and
imports installed on the local host. A configured SIMMACHINE_HOST variable
does *not* prove the simulator is online.

## Explicit blocked gates

- Real camera intrinsics/extrinsics and reliable visibility of all 8
  compact station tags (last measured: 4/8); current compact position
  error 0.593 mm max on an ideal rendered virtual pitcher.
- True **spout tip** and glass/crema surface height, <0.5mm full
  camera+tracking+actuator+gripper error budget and angular accuracy.
- Actual FR5 manufacturer controller runtime/smoothness, MoveIt planning,
  collision meshes, load and motor feedback.
- Real two-IR40 grasp friction/backlash, F/T contact, portafilter bayonet
  insertion, steam wand wet/thermal/flow behavior.
- Real DMP on human demonstrations and observable latte-art quality;
  DMP position-only replay alone cannot close milk-flow feedback loop.

**No autonomous hot espresso / milk serving approval.**
