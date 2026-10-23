# First REAL Latte Art demo -> virtual FR5 AprilTag remeasurement

**Experiment, 23 October 2026.** This is a new branch derived from the
unmerged RoboBarista integration PR #16. It **does not** copy third-party
Python source, checkpoint weights or raw Hugging Face MP4/Parquet into
our repository.

## What ridxm/latte-art-robot really contains

- https://github.com/ridxm/latte-art-robot
- https://huggingface.co/datasets/ridxm/latte-pour-demos
- https://ridxm.github.io/latte-art-robot/

The authors report 40 teleoperated, real-life pouring demonstrations,
three RGB cameras (overhead, two wrist-mounted) and 12D dual Realman RM65
joint states/actions. *This is not an AprilTag measurement of the actual
pitcher position and not a physically modeled milk/crema solver.* The
repository's README reports 20Hz while
`configs/robot/r2d3.yaml` advertises 10Hz; one converter is written for
LeRobot v2.1 and a 7D single-arm schema. **Read the actual episode
metadata and timestamps**, do not trust nominal rates, joint units or
camera calibration without checking. We did not find a top-level LICENSE
in the upstream code repository. A dataset license has not independently
been established; use local research only and respect its license.

For a physical milk dynamics simulator see *separately*
https://github.com/zhouxian/FluidLab (LatteArt-v0 differentiable
particle-fluid examples). FluidLab is not present in the ridxm project.
It is computationally demanding and its physics require wet calibration.

## Three distinct paths (never conflate)

1. **Original physical experiment**: published RGB videos (no known
   AprilTag fixtures/camera extrinsics) + RM65 joint samples.
2. **Reconstructed source**: official RealMan RM65 URDF FK of the chosen
   arm + measured/calibrated `T_flange_pitcher`. Only if the original
   joint ORDER, UNIT, URDF hardware variant and held-tool geometry are
   correct. This is *model-derived FK*, not measured from the original
   cameras. If unknown, explicitly use identity fixture *mock* and label
   the reconstruction uncalibrated.
3. **Our new camera experiment**: align relative source motion to a safe
   virtual FR5 reference pose, solve per-frame MuJoCo joint IK with
   collision/keep-out checks; render **new** virtual 3/4 RGB-D cameras,
   detect our small AprilTags using Pupil AprilTags and compare recovered
   6D poses with withheld source FK. This does NOT prove original footage
   is geometrically decoded or that the real FR5 can execute the path.

An action recorded on two RM65 arms is NOT an FR5 action. We never send
these recordings to FAIRINO hardware.

## First import, with internet (Colab or personal computer)

```bash
python -m pip install 'lerobot>=0.6,<1'
# Download ONE episode only. Start with numerical data, not all 40 videos.
python -m robobarista.real_latte_dataset \
    --episode 0 --output runs/real_latte_episode0

# Optional: only if camera videos needed; no dataset is committed:
python -m robobarista.real_latte_dataset --episode 1 --with-videos \
    --video-step 10 --output runs/real_latte_episode1_visual
```

This writes `recorded_realman_joints.json` and
`source_and_validity.json`. The importer refuses non-12D streams,
missing frames, nonfinite data and unexpected timestamps. Video frames,
if downloaded, are original image pixels, **not** 6D visual tracking.

## Model-based reconstruction (requires your validated original URDF)

Official RealMan model portal:
https://develop.realman-robotics.com/en/robot4th/download/model/
Official model repo:
https://github.com/RealManRobot/rm_models (Apache-2.0)

The URDF must match the exact RM65 variant used to record the data.
Find the corresponding six moving joint names in URDF **chain order**.
Determine from original recording documentation whether the joint
vectors are radians or degrees; the conversion CLI deliberately
requires `--joint-unit`.

Only for *first software smoke tests*, `--assume-identity-tool` treats
the RM65 flange as the pitcher body. This is a **wrong physical tool
assumption**, not real pitcher calibration:

```bash
python -m robobarista.realman_fk_bridge \
    --input-joints runs/real_latte_episode0/recorded_realman_joints.json \
    --urdf /path/to/the_verified_official_rm65.urdf \
    --flange the_actual_flange_link_name \
    --joint-names joint1 joint2 joint3 joint4 joint5 joint6 \
    --joint-unit rad --arm right \
    --assume-identity-tool \
    --output runs/real_latte_fk
```

The example arm, joint names and units are placeholders; they are
**not confirmed for the published episode**. For meaningful physical
tool frames, provide JSON `{"T_flange_pitcher":[[...],[...],[...],[...]]}`
from calibration and use `--mount-json` instead.

## Re-measure with our independent virtual AprilTag cameras

```bash
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
python -m robobarista.camera_joint_replay \
  --capture-only --marker-profile compact --camera-count 3 \
  --external-pitcher-trajectory runs/real_latte_fk/source_pitcher_6d_URDF_FK.json \
  --frames 61 --fps 3 --width 1280 --height 960 \
  --output runs/real_latte_virtual_camera_remeasurement

python -m robobarista.vision_quality \
  --capture runs/real_latte_virtual_camera_remeasurement \
  --output runs/real_latte_virtual_camera_remeasurement/vision_quality.json
```

- 61 frames at 3Hz capture a **20-second resampled** source. Any
  different source duration is time-scaled; summary reports it.
- The source segment must have at least 8 poses and a maximum
  **120mm 3D pitcher excursion** relative to its first frame. If
  pickup/transport is included, choose or crop a dedicated pouring
  segment in the original dataset instead of secretly normalizing away
  its scale. A long motion exceeding the limit is correctly rejected.
- The initial Realman coordinate frame is aligned to a collision-free
  FR5 pose (the absolute RM65 base-to-FR5 position is NOT known).
- A virtual FR5 fails closed if IK/joint/keepout checks fail; no
  simulated spout tracking number is a real robot accuracy guarantee.
- We do not infer milk flow, pouring rate, heart shape or actual
  crema response from joints or from rendered markers.

## Next measurements we need

1. **Actual dataset schema**: episode validity, nominal Hz, whether each
   real state is degrees/radians, whether the right or left arm holds the
   pitcher; corroborate visually with the videos.
2. **Verified official URDF** of the actual original RM65 hardware,
   exact six-joint order and base/frame names. Test FK against original
   reported measured TCP when possible.
3. **Rigid original gripper-to-pitcher transform** and camera
   intrinsics/extrinsics. Without them, none of the public videos is a
   reliable metric 3D pitcher trajectory.
4. Pour-only episode trimming/segmentation and same source timestamps
   in all camera frames.
5. On our FR5: tool adapter mass, reachable workcell, actual servo
   response, cup surface plane, force/torque, and wet milk-flow sensing.

**Status:** import/FK/safe virtual-camera remeasurement integration code
exists, and synthetic unit tests exist. Actual original Hugging Face
episode downloaded in this session: **NO** (network unavailable in current
execution environment). Original reconstructed 6D source run: **NOT YET**.
Real FR5 controller run: **NEVER**.
