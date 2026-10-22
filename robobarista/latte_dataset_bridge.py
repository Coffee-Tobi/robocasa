"""Convert our AprilTag demonstration into a portable, synchronized dataset.

This is NOT native LeRobot v3.0. External inspiration:
https://github.com/ridxm/latte-art-robot
https://huggingface.co/datasets/ridxm/latte-pour-demos
(40 human/teleoperated robot-arm demonstrations, multi-camera, 20 Hz).
We do not copy the project's source or weights: repo licensing is unclear.

The virtual source truth is *evaluation-only* and is kept in a separate file;
the observations/actions input MUST NOT import truth or source joint states.
Future hardware demos should include measured, time-synchronized image
files + calibrated robot state, real grasp sensor, and actual cup/pitcher poses.
"""
from __future__ import annotations
import csv,json,math
from pathlib import Path


def export_camera_capture(capture_dir,output_dir):
    capture=Path(capture_dir)
    out=Path(output_dir)
    observations=json.loads((capture/"camera_measurements.json").read_text())
    reference=json.loads((capture/"ground_truth_evaluation_only.json").read_text())
    if len(observations)!=len(reference) or len(observations)<8:
        raise ValueError("Require matching camera measurements and separated truth")
    out.mkdir(exist_ok=True,parents=True)
    rows=[]
    for i,item in enumerate(observations):
        t=float(item["time_s"])
        if (not math.isfinite(t) or (i and t<=rows[-1]["t_s"])
                or not item.get("valid") or item.get("visible_views",0)<2):
            raise ValueError(f"Missing/calibration-invalid cameras frame {i}")
        pos=item["position_m"]
        quat=item["rotation_xyzw"]
        if len(pos)!=3 or len(quat)!=4 or not all(math.isfinite(float(v)) for v in [*pos,*quat]):
            raise ValueError("Invalid pose")
        rows.append({"frame":i,"t_s":t,"pitcher_position_m":pos,
            "pitcher_orientation_xyzw":quat,
            "valid_cameras":int(item["visible_views"]),
            "tag_ids":item.get("tags_seen",[]),
            "calibration_source":"synthetic RGB-D MuJoCo fixture",
            "measured_robot_joint_angles":None,
            "real_gripper_feedback":None,
            "force_torque_6d":None,
            "human_demonstration":False})
    with (out/"camera_observations.jsonl").open("w") as f:
        for x in rows:f.write(json.dumps(x)+"\n")
    (out/"evaluation_truth_DO_NOT_TRAIN_ON.json").write_text(json.dumps(reference,indent=2))
    m={"schema":"robobarista_apriltag_camera_v1",
        "lerobot_compatible_directly":False,
        "inspired_by":"ridxm/latte-art-robot and HuggingFace LeRobot 3",
        "source":"synthetic virtual pitcher in MuJoCo, NOT human demonstration",
        "frames":len(rows),"duration_s":rows[-1]["t_s"]-rows[0]["t_s"],
        "average_hz":(len(rows)-1)/(rows[-1]["t_s"]-rows[0]["t_s"]),
        "camera_count_installed":3,
        "measurement_frames_with_at_least_two_views":sum(x["valid_cameras"]>=2 for x in rows),
        "synchronized_motor_state":False,"physical_force_feedback":False,
        "ground_truth_isolation":True,
        "next_step":"collect real synced camera MP4 and robot control feedback at 20+ Hz",
        "license":"Our exported synthetic data; external dataset NOT copied"}
    (out/"metadata.json").write_text(json.dumps(m,indent=2))
    return m
