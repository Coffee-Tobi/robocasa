# FAIRINO FR5 — which code should we keep? (22 Oct 2026)

The phrase "FR50 library" likely refers to **FAIRINO FR5 SDK / the existing Fairino-Mujoco model**. They are different products.

- **Already reused:** the FR5 articulated MuJoCo robot model and joint/actuator representation from `vendor/Fairino-Mujoco`. `MuJoCo` remains our off-hardware physics sandbox.
- **Official FAIRINO SDK** for connecting to the *real* robot: https://github.com/FAIR-INNOVATION/fairino-python-sdk; official manual https://manual.fairino.support/latest/SDKManual/python_intro.html. v2.3.0 release accompanies robot v4.0.0. Check the actual controller firmware before choosing an SDK version.
- **ROS 2/MoveIt 2** available from https://github.com/FAIR-INNOVATION/frcobot_ros2. More moving parts for a beginner; not a mandatory prerequisite for the coffee prototype.
- **SimMachine** FAIRINO's separate official simulator, per https://manual.fairino.support/latest/download.html. Not a drop-in replacement for MuJoCo's custom visual AprilTag and contact experiments.

## Concrete migration plan

| Currently our code | Best future implementation | Reason |
|---|---|---|
| Joint waypoint playback, basic movements | SDK `MoveJ`, `MoveL`, `ServoJ`, `ServoCart`, spline and trajectory-replay facilities | Reuse vendor kinematics, firmware and built-in interpolation |
| Hardware servo and torque control | SDK supported servo interfaces and robot's internal certified controller, where compatible | Never assume a 1 kHz simulation controller can run unchanged on physical robot |
| Gripper commands | FAIRINO SDK gripper/IO interfaces; check that **two IR40-50 + splitter** have independent addresses and position/current feedback | Vendor/tool compatibility and safety |
| Force-torque/contact when lowering on steam wand | Official FT calibration and force control interfaces; hardware validation essential | Don't rewrite all force feedback |
| Workpiece/tool offset calibration | SDK tool/workpiece coordinate frame functions, hardware calibrate | Avoid false spout errors due to TCP frame mismatch |
| RGB-D AprilTag camera calibration and image-based pose | Keep our `barista_cell.py`, `geometry.py`, `camera_joint_replay.py` and improve measured-data pipeline | FAIRINO SDK does not reconstruct AprilTag marker/object poses from several cameras |
| Latte motion synthesis + milk-height/flow feedback | Keep and improve trajectory generation + adaptive controller | There is no vendor latte-art skill |
| Independent tests, visualization, virtual contact and collision hypotheses | Keep MuJoCo simulation and CI; optionally compare with FAIRINO SimMachine | Hardware SDK alone isn't a digital twin of espresso station |

## Very important

- FAIRINO SDK typically expects millimetres and degrees for Cartesian poses whereas MuJoCo geometry uses metres and radians. Always convert at a tested bridge layer.
- Nominal arm repeatability is not equivalent to end-to-end spout tip accuracy while gripping a filled pitcher.
- Test the FAIRINO built-in spline/servo/trajectory playback **against** custom controllers using the same reference gesture, actual tool mass, force sensor, camera and measured spout errors before replacing anything.
- This comparison code remains virtual; **do not send these scripts directly to a real robot without safety setup, limits, tool calibration and supervision**.
- The 20-second dual-path test is strictly a rigid-fixture, no-liquid MuJoCo study. Test CI outcome before claiming numeric accuracy.

Official SDK reference: https://manual.fairino.support/latest/SDKManual/PythonRobotMovement.html
Official SDK source: https://github.com/FAIR-INNOVATION/fairino-python-sdk
Official Python releases: https://github.com/FAIR-INNOVATION/fairino-python-sdk/releases
