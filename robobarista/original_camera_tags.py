"""Look for actual AprilTags in the ORIGINAL published RGB recordings.

Only counts visible 2D tag IDs/pixels. Without original rigid-marker sizes,
3D mount placement, moving wrist-camera extrinsics, calibration and time
sync, an original video CANNOT give a reliable world-frame 6D milk-pitcher
pose. Never substitute a guessed Realman trajectory when no tags are found.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def scan_recorded_camera_frames(capture_dir):
    from .geometry import detect_tags
    base=Path(capture_dir)/"original_camera_frames"
    if not base.exists():raise FileNotFoundError(
        "No original RGB found. Use --with-videos on real_latte_dataset first.")
    report={"original_videos_scanned":True,
            "camera_6d_world_pose_from_original":False,
            "calibration_available":False,
            "note":"AprilTag IDs in individual raw original images only; this does not solve moving wrist extrinsics."}
    camera_results={}
    for folder in sorted(x for x in base.iterdir() if x.is_dir()):
        frames=sorted(folder.glob("frame_*.png"))
        per_frame=[]
        for file in frames:
            rgb=np.asarray(Image.open(file).convert("RGB"))
            hits=detect_tags(rgb,include_apriltags=True,detector="pupil")
            per_frame.append({
                "file":file.name,
                "tag_ids":sorted(set(int(tagid) for tagid,_ in hits)),
                "corners_px":{str(tagid):np.asarray(corners).tolist()
                              for tagid,corners in hits}})
        camera_results[folder.name]={
            "scanned":len(frames),
            "frames_with_tags":sum(bool(x["tag_ids"]) for x in per_frame),
            "seen_tag_ids":sorted(set(t for row in per_frame for t in row["tag_ids"])),
            "per_frame":per_frame}
    if not camera_results or all(not v["scanned"] for v in camera_results.values()):
        raise ValueError("No decoded RGB frames found")
    report["cameras"]=camera_results
    report["all_original_frames_have_multiview_tags"]=False
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--capture",required=True)
    p.add_argument("--output",required=True)
    a=p.parse_args()
    result=scan_recorded_camera_frames(a.capture)
    out=Path(a.output)
    out.parent.mkdir(exist_ok=True,parents=True)
    out.write_text(json.dumps(result,indent=2))
    print(json.dumps({
        k:{"scanned":v["scanned"],"frames_with_tags":v["frames_with_tags"],
           "seen_tag_ids":v["seen_tag_ids"]}
        for k,v in result["cameras"].items()},indent=2))


if __name__=="__main__":
    main()
