"""RoboBarista DEFAULT: 3 RGB-D cameras -> rigid AprilTag pose -> FAIRINO NewSpline.

The only active *motion-planning output* here is the manufacturer's documented
NewSpline/MoveL command sequence. No Ruckig, custom motor torque or old robot
replay is needed after the camera-only 6D estimate.

Default is SAFE DRY RUN. This code never connects to a FAIRINO controller and
does not simulate the proprietary firmware. For real use, first calibrate
T_base_world & T_pitcher_tcp, tool/user IDs, safe approach, and get robot support.

CLI:
 python -m robobarista.fairino_barista_pipeline \
   --input path/to/camera_measurements.json --output runs/fairino_active
 python -m robobarista.fairino_barista_pipeline \
   --input path/to/successful_full_colab_artifact.zip \
   --output runs/fairino_active
"""
from __future__ import annotations
import argparse,json,zipfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .fairino_native_spline import (Calibration,make_plan,RecordingRobot,
                                   replay_commands,demo_calibration)

def load_camera(path):
    path=Path(path)
    if path.suffix.lower()==".zip":
        with zipfile.ZipFile(path) as z:
            rows=json.loads(z.read("camera/camera_measurements.json"))
    else:
        rows=json.loads(path.read_text())
    if not isinstance(rows,list) or len(rows)<8:
        raise ValueError("Need >=8 camera-reconstructed 6D frames")
    times=np.array([x["time_s"] for x in rows],float)
    if not np.all(np.isfinite(times)) or not np.all(np.diff(times)>0):
        raise ValueError("Camera time stamps must be finite and strictly increasing")
    if any(not rec.get("valid") or rec.get("visible_views",0)<2 for rec in rows):
        raise ValueError("Missing AprilTags or fewer than two cameras; re-record, no GT fill")
    if any(len(rec.get("tags_seen",[]))==0 for rec in rows):
        raise ValueError("Missing AprilTag observations")
    return rows

def read_calibration(path):
    if path is None:
        return demo_calibration(),False
    d=json.loads(Path(path).read_text())
    if not all(k in d for k in ("T_base_world","T_pitcher_tcp","tool_id","user_id")):
        raise ValueError("Calibration JSON lacks robot/world or pitcher/TCP transform")
    cal=Calibration(np.asarray(d["T_base_world"],float),
                    np.asarray(d["T_pitcher_tcp"],float),
                    int(d["tool_id"]),int(d["user_id"]))
    # File presence itself is not proof that measurements are valid.
    checked=bool(d.get("measured_and_verified",False))
    return cal,checked

def target_motion(rows,max_displacement_mm=20.,target_displacement_mm=18.,
                  duration_s=20.):
    """Return the time-retimed measured pitcher trajectory.

    The input old camera demonstration is ~33 mm, so explicitly scale its
    translation to <=18 mm in the mock replay. This CHANGES the motion; do
    not present it as a literal replay of a human original demonstration.
    """
    if not 1.<=target_displacement_mm<max_displacement_mm<=30.:
        raise ValueError("Require positive target below 20-30 mm cap")
    orig=np.array([d["position_m"] for d in rows],float)
    if orig.shape[1]!=3 or not np.isfinite(orig).all():
        raise ValueError("Invalid pitcher coordinates")
    for r in rows:
        q=np.asarray(r["rotation_xyzw"],float)
        if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.002:
            raise ValueError("Invalid camera orientation")
    excursion=float(np.linalg.norm(orig-orig[0],axis=1).max())
    if excursion<=1e-8:raise ValueError("No demonstrator movement")
    scale=min(1.,target_displacement_mm/(excursion*1000))
    source_time=np.array([r["time_s"] for r in rows],float)
    new_t=(source_time-source_time[0])/(source_time[-1]-source_time[0])*duration_s
    shifted=[]
    for r,new_time,p in zip(rows,new_t,orig):
        y=dict(r)
        y["time_s"]=float(new_time)
        y["position_m"]=(orig[0]+scale*(p-orig[0])).tolist()
        shifted.append(y)
    actual_excursion=float(np.linalg.norm(
        (np.array([r["position_m"] for r in shifted])-orig[0])*1000,axis=1).max())
    if actual_excursion>max_displacement_mm+1e-6:raise ValueError("spout/camera motion too big")
    return shifted,scale,float(excursion*1000),actual_excursion

def run(input_path,output,calibration_path=None,stride=2,target_mm=18.,
        duration_s=20.,blend_radius_mm=.5,tip_offset_pitcher_m=None):
    rows=load_camera(input_path)
    transformed,scale,original_mm,capped_mm=target_motion(
        rows,target_displacement_mm=target_mm,duration_s=duration_s)
    cal,verified=read_calibration(calibration_path)
    tip_excursion=None
    if tip_offset_pitcher_m is not None:
        offset=np.asarray(tip_offset_pitcher_m,float)
        if offset.shape!=(3,) or not np.isfinite(offset).all():
            raise ValueError("Spout offset must be 3 finite local metres")
        tips=np.array([np.asarray(x["position_m"],float)
                  +Rotation.from_quat(x["rotation_xyzw"]).apply(offset)
                  for x in transformed])
        tip_excursion=float(np.linalg.norm((tips-tips[0])*1000,axis=1).max())
        if tip_excursion>20.+1e-6:
            raise ValueError(f"Virtual spout excursion {tip_excursion:.3f}mm exceeds 20mm")
    if stride<1:raise ValueError("stride >=1")
    out=Path(output)
    out.mkdir(parents=True,exist_ok=True)
    report={
        "status":"OFFLINE_DRY_RUN_ONLY",
        "active_pipeline":"3 RGB-D cameras -> AprilTag 6D pitcher pose -> calibrated TCP targets -> FAIRINO NewSpline commands",
        "no_legacy_ruckig_or_torque_controllers_used":True,
        "camera_source":"Recorded AprilTag pose estimates, not camera pixel data",
        "source_camera_frames":len(rows),"minimum_cameras_required":2,
        "source_duration_s":float(rows[-1]["time_s"]-rows[0]["time_s"]),
        "requested_duration_s":float(duration_s),
        "original_pitcher_center_excursion_mm":original_mm,
        "rescaled_pitcher_center_excursion_mm":capped_mm,
        "translation_scale_applied":scale,
        "spout_tip_excursion_mm":tip_excursion,
        "measured_tip_offset_for_real_tool":False,
        "NOTE":"20mm always applies to pitcher CENTER; optional spout check uses a supplied virtual tool offset, not a calibrated real spout.",
        "camera_world_to_robot_base_calibrated":verified,
        "gripper_tool_offset_calibrated":verified,
        "real_fairino_controller_invoked":False,
        "real_fairino_tracking_error_mm":None,
        "firmware_path_available_for_comparison":False}
    (out/"camera_6d_retimed_for_fairino.json").write_text(json.dumps(transformed,indent=2))
    for mode in ("new_spline","movel_blend"):
        plan=make_plan(transformed,cal,mode=mode,duration_s=duration_s,
                       waypoint_stride=stride,blend_radius_mm=blend_radius_mm)
        fake=RecordingRobot()
        replay_commands(fake,plan)
        plan["real_hardware_run_allowed"]=False
        plan["calibrated"]=verified
        plan["source_camera_frames"]=len(rows)
        plan["gripper_fixture_physics"]="NOT SIMULATED"
        plan["issuer"]="Offline commands only, NOT a FAIRINO firmware planner"
        (out/("fairino_"+mode+"_commands.json")).write_text(json.dumps(plan,indent=2))
        report[mode+"_waypoints"]=plan["waypoints"]
        report[mode+"_mock_api_calls"]=len(fake.calls)
    (out/"summary.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    return report

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--input",required=True,help="Previously reconstructed camera JSON or successful GitHub artifact ZIP")
    p.add_argument("--output",default="runs/fairino_camera_pipeline")
    p.add_argument("--calibration",help="Explicit 4x4 robot/world and pitcher/TCP calibration (optional DRY RUN)")
    p.add_argument("--stride",type=int,default=2)
    p.add_argument("--target-mm",type=float,default=18.)
    p.add_argument("--duration-s",type=float,default=20.)
    p.add_argument("--blend-radius-mm",type=float,default=.5)
    p.add_argument("--virtual-spout-offset-m",type=float,nargs=3,default=None,
                   help="Only if measured in the same camera-object frame; reports and checks spout excursion")
    a=p.parse_args()
    run(a.input,a.output,a.calibration,a.stride,a.target_mm,a.duration_s,a.blend_radius_mm,a.virtual_spout_offset_m)

if __name__=="__main__":main()
