"""Camera-derived FAIRINO MoveJ command preview. NEVER executes robot motion.

Extract 31 joint targets from an earlier validated 3-RGB-D-camera / AprilTag
-> 6D IK replay artifact, convert MuJoCo radians to FAIRINO degrees.
These are joint waypoints, not guaranteed controller execution timestamps.
Use --artifact ZIP --output PATH; no FAIRINO Python SDK dependency.
"""
import argparse,csv,json,zipfile
from pathlib import Path
import numpy as np


def export(archive, output):
    with zipfile.ZipFile(archive) as z:
        records=json.loads(z.read("camera/robot_replay.json"))
    if len(records)!=31:
        raise ValueError("Expected 31 saved camera->IK waypoint frames")
    out=Path(output)
    out.parent.mkdir(exist_ok=True,parents=True)
    with out.open("w",newline="") as f:
        w=csv.writer(f)
        w.writerow(["camera_time_s","SDK_command","q1_deg","q2_deg",
                    "q3_deg","q4_deg","q5_deg","q6_deg","blendT_ms","status"])
        for r in records:
            deg=np.rad2deg(r["q_rad"])
            if not np.isfinite(deg).all():
                raise ValueError("Invalid joint angles")
            w.writerow([r["time_s"],"MoveJ",*deg,-1.,
                        "DRY_RUN_ONLY_NO_HARDWARE_EXECUTION"])
    print("Wrote",out,"with",len(records),"preview joint waypoints")
    return out


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--artifact",required=True,help="Full passed Colab ZIP (run 39641910809)")
    p.add_argument("--output",default="runs/fairino_camera_movej_dryrun.csv")
    args=p.parse_args()
    export(args.artifact,args.output)
