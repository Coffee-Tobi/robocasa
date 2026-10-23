"""Read one real latte-art demonstration from published LeRobot data.

Source: https://huggingface.co/datasets/ridxm/latte-pour-demos
The upstream GitHub repo has NO LICENSE at the time of this integration.
Use dataset locally, never copy source or redistribute video into Git history.

Important: 14D Realman *joint states and grippers* are NOT a calibrated 6D pitcher pose.
No AprilTags/known camera extrinsics are asserted for the upstream videos.
The follow-on realman_fk_bridge needs a matching manufacturer URDF, the
recorded joint order/unit, and a measured flange-to-pitcher transform.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np

DEFAULT_REPO="ridxm/latte-pour-demos"
CAMERA_KEYS=("observation.images.top_camera",
             "observation.images.left_wrist",
             "observation.images.right_wrist")


def validate_realman_episode(rows, *, expected_dims=14):
    """Validate the recorded synchronized joint series, not robot accuracy."""
    if not isinstance(rows,list) or len(rows)<8:
        raise ValueError("Need at least 8 samples from one episode")
    times=np.asarray([r["time_s"] for r in rows],float)
    # Reject malformed ragged arrays cleanly before NumPy raises a confusing
    # "inhomogeneous shape" exception.
    state_vectors=[np.asarray(r["state"],float) for r in rows]
    action_vectors=[np.asarray(r["action"],float) for r in rows]
    if any(v.shape!=(expected_dims,) for v in state_vectors+action_vectors):
        raise ValueError(f"Expected {expected_dims}D state/action per frame")
    states=np.stack(state_vectors)
    actions=np.stack(action_vectors)
    if not (np.isfinite(times).all() and np.isfinite(states).all()
            and np.isfinite(actions).all() and np.all(np.diff(times)>0)):
        raise ValueError("Nonfinite or nonmonotonic Realman recording")
    if not .05 < times[-1]-times[0] <= 180:
        raise ValueError("Unexpected recording duration")
    step=np.diff(times)
    return {
        "frames":len(rows),"duration_s":float(times[-1]-times[0]),
        "median_hz":float(1.0/np.median(step)),
        "max_interframe_gap_s":float(np.max(step)),
        "left_arm_max_raw_joint_change":float(np.max(
            np.abs(states[:, :6]-states[0,:6]))),
        "right_arm_max_raw_joint_change":float(np.max(
            np.abs(states[:, 7:13]-states[0,7:13]))),
        "joint_units_confirmed":False,
        "state_layout_from_upstream_Hardware_Bridge_ROS2":"[left six joints, left gripper, right six joints, right gripper]",
        "left_gripper_raw_first":float(states[0,6]),
        "right_gripper_raw_first":float(states[0,13]),
        "flange_and_pitcher_pose_available":False,
        "milk_fluid_simulation":False
    }


def _array(obj):
    """Handle numpy or torch tensors returned from LeRobot."""
    if hasattr(obj,"detach"):
        obj=obj.detach().cpu().numpy()
    return np.asarray(obj)


def _image_uint8(value):
    from PIL import Image
    rgb=_array(value)
    if rgb.ndim!=3:
        raise ValueError("Expected RGB tensor")
    if rgb.shape[0]==3 and rgb.shape[-1]!=3:
        rgb=rgb.transpose(1,2,0)
    if rgb.shape[-1]!=3:
        raise ValueError("Expected RGB, not depth or RGBA")
    if np.issubdtype(rgb.dtype,np.floating):
        if not np.isfinite(rgb).all() or rgb.min()<-.01 or rgb.max()>1.01:
            raise ValueError("Unexpected normalized image tensor")
        rgb=np.clip(rgb*255,0,255).round().astype(np.uint8)
    else:
        rgb=np.asarray(rgb,np.uint8)
    return Image.fromarray(rgb,"RGB")


def extract_single_parquet_episode(*, output_dir, repo_id=DEFAULT_REPO,
                                   episode=0):
    """CPU-light REAL public data import; no torch, no video, no full-dataset sync.

    For the verified ridxm release each episode is a separate Parquet file.
    Fail closed if the layout changes. Pin the actual source revision SHA.
    Requires `pip install huggingface_hub pyarrow`.
    """
    if not isinstance(episode,int) or episode<0:
        raise ValueError("Episode index must be non-negative")
    try:
        from huggingface_hub import HfApi,hf_hub_download
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise RuntimeError(
            "Lightweight import: pip install huggingface_hub pyarrow") from exc
    path=Path(output_dir)
    if path.exists() and any(path.iterdir()):
        raise FileExistsError("Choose fresh output directory; no overwrite")
    details=HfApi().dataset_info(repo_id,files_metadata=True)
    revision=details.sha
    candidates=[
        f for f in details.siblings
        if f.rfilename.startswith("data/")
        and f.rfilename.endswith(f"episode_{episode:06d}.parquet")]
    if len(candidates)!=1:
        raise ValueError(
            f"Expected one exact Parquet for episode {episode}; "
            "upstream storage layout may have changed")
    file=candidates[0]
    if file.size is not None and file.size>60_000_000:
        raise ValueError("Episode data >60 MB lightweight import budget")
    local=hf_hub_download(
        repo_id=repo_id,repo_type="dataset",
        filename=file.rfilename,revision=revision)
    table=pq.read_table(local)
    expected={"timestamp","observation.state","action","episode_index"}
    if not expected.issubset(table.column_names):
        raise ValueError("Original Parquet lacks required synchronized columns")
    records=table.select(sorted(expected)).to_pylist()
    records=[r for r in records if int(r["episode_index"])==episode]
    if not records:
        raise ValueError("Parquet has no rows for selected episode")
    t0=float(records[0]["timestamp"])
    rows=[{"time_s":float(r["timestamp"])-t0,
           "state":r["observation.state"],"action":r["action"]}
          for r in records]
    summary=validate_realman_episode(rows)
    summary.update({
        "source":repo_id,
        "source_revision":revision,
        "source_data_path":file.rfilename,
        "episode":episode,
        "source_type":"ACTUAL original dual Realman physical demonstration",
        "recorded_source_joint_state_dims":14,
        "source_camera_6d_pitcher_pose":False,
        "source_real_milk_physics_model":False,
        "source_cameras_downloaded":False,
        "data_license_verified":False,
        "hardware_execution_authorized":False,
        "source_joint_units_verified":False})
    path.mkdir(parents=True,exist_ok=True)
    (path/"recorded_realman_joints.json").write_text(json.dumps(rows,indent=2))
    (path/"source_and_validity.json").write_text(json.dumps(summary,indent=2))
    return summary


def extract_published_episode(*, output_dir, repo_id=DEFAULT_REPO,
                              episode=0, with_video=False, video_step=10,
                              local_root=None):
    """Load through official LeRobot API, with a *single* episode selected.

    No automatic full-40-episode video download. with_video=True opts in
    to MP4 loading/decoding, and exports downsampled original RGB frames.
    """
    if not isinstance(episode,int) or episode<0:
        raise ValueError("Episode index must be >=0")
    if not 1<=video_step<=100:
        raise ValueError("video_step out of range")
    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError(
            "Install the official Hugging Face LeRobot library in Colab "
            "or a connected development computer") from exc
    out=Path(output_dir)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError("Use a new output directory; do not overwrite data")
    ds_kwargs={"repo_id":repo_id,"episodes":[episode],
               "download_videos":bool(with_video)}
    if local_root is not None:ds_kwargs["root"]=Path(local_root)
    ds=LeRobotDataset(**ds_kwargs)
    # Select only low-dimensional state/action to avoid decoding any videos
    # when the user requests just a motion-data inspection.
    columns=["timestamp","observation.state","action"]
    data=ds.select_columns(columns)
    rows=[]
    for i,record in enumerate(data):
        state=_array(record["observation.state"]).reshape(-1).astype(float)
        action=_array(record["action"]).reshape(-1).astype(float)
        rows.append({
            "time_s":float(record["timestamp"]),
            "state":state.tolist(),"action":action.tolist()})
    if rows:
        base=rows[0]["time_s"]
        for row in rows:row["time_s"]-=base
    report=validate_realman_episode(rows)
    info=getattr(ds.meta,"info",{})
    features=info.get("features",{}) if isinstance(info,dict) else {}
    available=sorted(k for k in features if k.startswith("observation.images."))
    report.update({
        "source":repo_id,"episode":episode,
        "source_type":"REAL robot kinesthetic demonstration",
        "data_license_verified":False,
        "state_kind":"14D left(6)+left_gripper+right(6)+right_gripper, NOT TCP poses",
        "camera_views_advertised":available,
        "camera_intrinsics_extrinsics_available":False,
        "target_robot":"OpenDroid R2D3 with two Realman RM65 arms",
        "left_arm_indices":[0,1,2,3,4,5],
        "left_gripper_index":6,
        "right_arm_indices":[7,8,9,10,11,12],
        "right_gripper_index":13,
        "upstream_image_processing":"None; optional original decoded RGB only",
        "hardware_execution_authorized":False
    })
    out.mkdir(parents=True)
    (out/"recorded_realman_joints.json").write_text(
        json.dumps(rows,indent=2))
    if with_video:
        for i in range(0,len(rows),video_step):
            image_record=ds[i]
            for name in CAMERA_KEYS:
                if name not in image_record:
                    continue
                target=out/"original_camera_frames"/name
                target.mkdir(parents=True,exist_ok=True)
                _image_uint8(image_record[name]).save(
                    target/f"frame_{i:04d}.png")
        report["saved_original_rgb_every_nth_frame"]=video_step
        report["saved_rgb_is_metric_apriltag_pose"]=False
    (out/"source_and_validity.json").write_text(
        json.dumps(report,indent=2))
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",required=True)
    p.add_argument("--repo-id",default=DEFAULT_REPO)
    p.add_argument("--episode",type=int,default=0)
    p.add_argument("--with-videos",action="store_true")
    p.add_argument("--parquet-only",action="store_true",
                   help="Fast actual episode using pyarrow + huggingface_hub; no LeRobot/torch needed")
    p.add_argument("--video-step",type=int,default=10)
    p.add_argument("--local-root",default=None)
    a=p.parse_args()
    if a.parquet_only:
        if a.with_videos:
            raise ValueError("--parquet-only cannot download videos")
        result=extract_single_parquet_episode(
            output_dir=a.output,repo_id=a.repo_id,episode=a.episode)
    else:
        result=extract_published_episode(
            output_dir=a.output,repo_id=a.repo_id,episode=a.episode,
            with_video=a.with_videos,video_step=a.video_step,
            local_root=a.local_root)
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    main()
