"""Optional REAL Hugging Face LeRobot v3 writer for simulated Latte Art cameras.

This integrates the actual `lerobot.datasets.LeRobotDataset` API (Apache-2.0),
NOT code from ridxm/latte-art-robot (repository has no LICENSE).

You MUST capture unmodified camera images via --save-camera-frames. Camera
measurements (not hidden simulator truth) form 7D observation and 7D
next-measured-pose *pseudoactions*; they are NOT FAIRINO joint actions, not
a human demonstration, and not a policy suitable for hardware deployment.
No network/push_to_hub or pretrained model is invoked.
"""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
import numpy as np
from PIL import Image

CAMS=("cell_left","cell_right","cell_back")
POSE_NAMES=("x_m","y_m","z_m","qx","qy","qz","qw")


def _pose(ob):
    return np.asarray(
        [*ob["position_m"],*ob["rotation_xyzw"]],dtype=np.float32)


def load_virtual_episode(capture_dir,*,require_raw=True):
    """Strict raw camera synchronisation; no simulator ground truth leaks."""
    directory=Path(capture_dir)
    data=json.loads((directory/"camera_measurements.json").read_text())
    if not 3<=len(data)<=10000:
        raise ValueError("No framewise camera records")
    times=np.asarray([float(d["time_s"]) for d in data])
    step=np.diff(times)
    if not all(d["valid"] and d["visible_views"]>=2 for d in data):
        raise ValueError("Invalid 3D camera estimates: do not fill with truth")
    if not np.all(np.isfinite(times)) or np.min(step)<=0:
        raise ValueError("Camera time must increase")
    fps=1./float(np.median(step))
    fps_int=int(round(fps))
    if abs(fps-fps_int)>1e-3 or np.max(abs(step-1/fps_int))>1e-5:
        raise ValueError("Native LeRobot needs a regular integer FPS")
    records=[]
    for i,row in enumerate(data):
        paths={name:directory/"raw_cameras"/name/f"frame_{i:04d}.png"
               for name in CAMS}
        if require_raw and any(not f.is_file() for f in paths.values()):
            raise FileNotFoundError(f"Frame {i}: missing unmodified RGB camera images")
        pose=_pose(row)
        if not np.all(np.isfinite(pose)) or abs(np.linalg.norm(pose[3:])-1)>.1:
            raise ValueError(f"Invalid pose or quaternion {i}")
        records.append({"state":pose,"image_paths":paths,"t_s":times[i]})
    return records,fps_int


def create_native_lerobot_dataset(capture_dir,output_dir,repo_id="local/robobarista-synthetic-latte"):
    """Writes actual local LeRobot Dataset v3 with 3 camera videos.

    Dependencies are OPTIONAL and substantial: pip install 'lerobot>=0.6'.
    Never import source GT, never claim FR5 robot states as captured actions.
    """
    rows,fps=load_virtual_episode(capture_dir)
    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError as e:
        raise RuntimeError("Install Apache-2.0 Hugging Face LeRobot separately; no torch/GPU needed for this project's core MuJoCo tests") from e

    with Image.open(rows[0]["image_paths"][CAMS[0]]) as im:
        width,height=im.size
    feature={"dtype":"float32","shape":(7,),"names":list(POSE_NAMES)}
    features={"observation.state":feature,"action":dict(feature)}
    for name in CAMS:
        features["observation.images."+name]={
            "dtype":"video","shape":(height,width,3),
            "names":["height","width","channels"]}
    out=Path(output_dir).resolve()
    if out.exists() and any(out.iterdir()):
        raise FileExistsError("Do not overwrite existing LeRobot dataset")
    ds=LeRobotDataset.create(
        repo_id=repo_id,root=out,fps=fps,features=features,
        robot_type="virtual_milk_pitcher_pose_not_fr5",
        use_videos=True,
        image_writer_processes=0,image_writer_threads=2)
    for i,rec in enumerate(rows):
        # Next measured pose is a pseudoaction from the actual cameras;
        # DO NOT use robot motor action label here, because that would be false.
        nxt=rows[min(i+1,len(rows)-1)]["state"]
        frame={"observation.state":rec["state"],
               "action":nxt,
               "task":"Reconstruct and imitate a SYNTHETIC latte art pitcher pose path"}
        for name in CAMS:
            with Image.open(rec["image_paths"][name]) as im:
                frame["observation.images."+name]=np.asarray(
                    im.convert("RGB"),dtype=np.uint8).copy()
        ds.add_frame(frame)
    ds.save_episode()
    ds.finalize()
    meta={"source":"MuJoCo 3-camera rendered virtual pitcher",
          "actual_human_demonstration":False,
          "hardware_joint_actions_present":False,
          "action_meaning":"one-step future reconstructed pose, not motor commands",
          "ground_truth_used_as_training_input":False,
          "camera_names":list(CAMS),"frames":len(rows),"fps":fps,
          "lerobot_native_dataset":True,"pushed_to_hub":False}
    (out/"robobarista_provenance.json").write_text(json.dumps(meta,indent=2))
    return meta


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--capture",required=True)
    p.add_argument("--output",required=True)
    p.add_argument("--repo-id",default="local/robobarista-synthetic-latte")
    a=p.parse_args()
    print(json.dumps(create_native_lerobot_dataset(
        a.capture,a.output,a.repo_id),indent=2))


if __name__=="__main__":main()
