"""Official FAIRINO Python-SDK motion API bridge: safe OFFLINE command rehearsal.

FAIRINO documents MoveL(blendR=...), NewSplineStart(type,averageTime),
NewSplinePoint(desc_pos,tool,user,lastFlag,...) and NewSplineEnd().
We call exactly those named API methods on a mock recorder, never connect to
a real robot in this script. Without an FR5 controller/SimMachine, this
validates the SDK command plan, NOT the manufacturer's actual path/dynamics.

Camera measurement gives T_world_pitcher, NOT the robot TCP pose!
T_base_tcp = T_base_world @ T_world_pitcher @ T_pitcher_tcp
Two calibrations are mandatory for real hardware; the CLI demo uses
explicitly labeled identity substitutions ONLY to exercise command building.
MuJoCo Cartesian units meters/radians -> SDK millimetres/degrees.
"""
from __future__ import annotations
import argparse
import json
import zipfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

DOCS = "https://manual.fairino.support/latest/SDKManual/PythonRobotMovement.html"


@dataclass(frozen=True)
class Calibration:
    T_base_world: np.ndarray
    T_pitcher_tcp: np.ndarray
    tool_id: int
    user_id: int

    def __post_init__(self):
        if not (0 <= self.tool_id <= 14 and 0 <= self.user_id <= 14):
            raise ValueError("FAIRINO tool/user ID must be 0..14")
        for name in ("T_base_world", "T_pitcher_tcp"):
            a=np.asarray(getattr(self,name),dtype=float)
            if a.shape != (4,4) or not np.isfinite(a).all():
                raise ValueError(name+" must be finite 4x4")
            if not np.allclose(a[3],(0,0,0,1),atol=1e-8):
                raise ValueError(name+" invalid homogeneous row")
            if (not np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3),atol=1e-5)
                    or abs(np.linalg.det(a[:3,:3])-1)>1e-5):
                raise ValueError(name+" must have proper rotation")


def pose_from_camera(sample,calibration):
    """Build FAIRINO [X,Y,Z,RX,RY,RZ] in millimetres and FIXED XYZ degrees."""
    p=np.asarray(sample["position_m"],float)
    q=np.asarray(sample["rotation_xyzw"],float)
    if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all():
        raise ValueError("Invalid AprilTag camera pose")
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-3:
        raise ValueError("Quaternion must be normalized")
    T=np.eye(4)
    T[:3,:3]=Rotation.from_quat(q).as_matrix()
    T[:3,3]=p
    target=calibration.T_base_world@T@calibration.T_pitcher_tcp
    # FAIRINO RPY angles are around fixed X/Y/Z; scipy lowercase 'xyz'
    # is EXTRINSIC XYZ, not intrinsic 'XYZ'.
    rpy=Rotation.from_matrix(target[:3,:3]).as_euler("xyz",degrees=True)
    return [*map(float,target[:3,3]*1000),*map(float,rpy)]


def select_camera_samples(samples,waypoint_stride=3):
    if waypoint_stride<1:raise ValueError("stride must >=1")
    if len(samples)<4 or any(not a.get("valid") for a in samples):
        raise ValueError("Need >=4 valid AprilTag samples; no truth substitution")
    stamps=np.array([float(a["time_s"]) for a in samples])
    if not np.all(np.isfinite(stamps)) or not np.all(np.diff(stamps)>0):
        raise ValueError("Camera timestamps must strictly increase")
    indices=list(range(0,len(samples),waypoint_stride))
    if indices[-1]!=len(samples)-1:indices.append(len(samples)-1)
    return [samples[i] for i in indices]


def make_plan(samples,calibration,mode="new_spline",duration_s=20.,
              waypoint_stride=3,blend_radius_mm=0.5):
    """Build actual named FAIRINO API commands (not a predicted controller path).

    All replay coordinates require measured extrinsic/calibrated tool offsets
    for a real robot; a 20s target is not a certified controller duration.
    """
    if mode not in {"new_spline","movel_blend"}:raise ValueError("mode")
    if not 0<=blend_radius_mm<=5:raise ValueError("blend radius exceeds preview bounds")
    if not 1<=duration_s<=120:raise ValueError("duration")
    picked=select_camera_samples(samples,waypoint_stride)
    poses=[pose_from_camera(x,calibration) for x in picked]
    if len(poses)<3:raise ValueError("At least three points required")
    average_time_ms=int(round(1000*duration_s/(len(poses)-1)))
    if average_time_ms < 10:raise ValueError("Time between spline points too short")
    instructions=[]
    if mode=="new_spline":
        # type=1: pass through programmed waypoint rather than arc-cutting type=0.
        instructions.append({"api":"NewSplineStart","args":[1,average_time_ms],"kwargs":{}})
        for i,pose in enumerate(poses):
            instructions.append({"api":"NewSplinePoint","args":[],
                 "kwargs":{"desc_pos":pose,"tool":calibration.tool_id,
                   "user":calibration.user_id,
                   "lastFlag":int(i==len(poses)-1),
                   "blendR":blend_radius_mm}})
        instructions.append({"api":"NewSplineEnd","args":[],"kwargs":{}})
    else:
        # Last point blocks until completed; others allow blending. Firmware
        # chooses exact transitions -- the preview does NOT simulate these.
        for i,pose in enumerate(poses):
            instructions.append({"api":"MoveL","args":[],
                "kwargs":{"desc_pos":pose,"tool":calibration.tool_id,
                     "user":calibration.user_id,"vel":10.0,
                     "blendR":blend_radius_mm if i<len(poses)-1 else -1.0,
                     "blendMode":0}})
    return {"mode":mode,"duration_request_s":duration_s,
            "average_time_ms":average_time_ms,
            "actual_runtime_verified":False,
            "camera_samples":len(samples),"waypoints":len(poses),
            "reference_waypoint_timestamps_s":[float(x["time_s"]) for x in picked],
            "commands":instructions,
            "camera_to_robot_extrinsics_calibrated":False,
            "tool_and_gripper_offsets_calibrated":False,
            "calibration_required_before_hardware":True}


class RecordingRobot:
    """Strict OFFLINE command-signature recorder; NEVER a firmware emulator.

    Validate our own generated API shape, not manufacturer interpolation.
    A real installed SDK/controller still needs independent integration tests.
    """
    def __init__(self):
        self.calls=[]

    @staticmethod
    def _pose(desc_pos, tool, user):
        values=np.asarray(desc_pos,dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError("FAIRINO desc_pos must have six finite mm/degree values")
        if not (isinstance(tool,int) and isinstance(user,int)
                and 0<=tool<=14 and 0<=user<=14):
            raise ValueError("Invalid configured tool/user ID")

    def NewSplineStart(self, type, averageTime):
        if type not in (0,1) or not isinstance(averageTime,int) or averageTime<10:
            raise ValueError("Invalid spline interpolation type or averageTime")
        self.calls.append(("NewSplineStart",(type,averageTime),{}))
        return 0

    def NewSplinePoint(self, *, desc_pos, tool, user, lastFlag, blendR=0.0):
        self._pose(desc_pos,tool,user)
        if lastFlag not in (0,1) or not 0<=float(blendR)<=5:
            raise ValueError("Invalid NewSplinePoint lastFlag or blendR")
        self.calls.append(("NewSplinePoint",(),dict(
            desc_pos=desc_pos,tool=tool,user=user,lastFlag=lastFlag,blendR=blendR)))
        return 0

    def NewSplineEnd(self):
        self.calls.append(("NewSplineEnd",(),{}))
        return 0

    def MoveL(self, *, desc_pos, tool, user, vel=10.0, blendR=-1.0, blendMode=0):
        self._pose(desc_pos,tool,user)
        if not 0<float(vel)<=100 or blendMode!=0 or not (-1<=float(blendR)<=5):
            raise ValueError("Invalid dry-run MoveL velocity/blending arguments")
        self.calls.append(("MoveL",(),dict(
            desc_pos=desc_pos,tool=tool,user=user,vel=vel,
            blendR=blendR,blendMode=blendMode)))
        return 0


def replay_commands(robot,plan):
    """Dry-run ONLY. Real Robot.RPC objects are rejected by design."""
    if not isinstance(robot,RecordingRobot):
        raise RuntimeError("Real robot commands are disabled: calibrate, validate, and certify separately")
    for item in plan["commands"]:
        call=getattr(robot,item["api"])
        code=call(*item["args"],**item["kwargs"])
        if code!=0:raise RuntimeError(item["api"]+" rejected: "+str(code))
    return robot.calls


def read_camera_artifact(path):
    """Consume existing verified CI AprilTag RGB-D reconstructed camera records."""
    with zipfile.ZipFile(path) as f:
        records=json.loads(f.read("camera/camera_measurements.json"))
    return records


def demo_calibration():
    # **MOCK ONLY**. Never claim a world frame or camera/jug TCP is identity in
    # real hardware. The two transforms must be measured on the actual cell.
    return Calibration(np.eye(4),np.eye(4),tool_id=0,user_id=0)


def run_offline(archive,output,duration=20.,stride=3,blend=0.5):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    samples=read_camera_artifact(archive)
    reports={}
    for mode in ("new_spline","movel_blend"):
        plan=make_plan(samples,demo_calibration(),mode,duration,stride,blend)
        fake=RecordingRobot()
        calls=replay_commands(fake,plan)
        plan["mock_sdk_calls_exercised"]=len(calls)
        plan["WARNING"]="DEMO ONLY: identity base/world and pitcher/TCP transforms; DO NOT execute on real robot"
        (out/(mode+"_FAIRINO_SDK_DRY_RUN.json")).write_text(json.dumps(plan,indent=2))
        reports[mode]={"waypoints":plan["waypoints"],
                       "sdk_calls_exercised":len(calls),
                       "avg_segment_time_requested_ms":plan["average_time_ms"],
                       "first_target_xyz_mm":plan["commands"][1 if mode=="new_spline" else 0]["kwargs"]["desc_pos"][:3]}
    (out/"summary.json").write_text(json.dumps({
        "camera_frame_count":len(samples),"camera_time_range_s":[samples[0]["time_s"],samples[-1]["time_s"]],
        "requested_duration_s":duration,
        "used_actual_fairino_method_names":True,
        "connection_to_real_robot":False,
        "firmware_generated_trajectory":False,
        "real_physical_tool_offsets_loaded":False,
        "modes":reports},indent=2))
    return reports


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--artifact",required=True,help="Validated GitHub Actions full-colab ZIP")
    p.add_argument("--output",default="runs/fairino_spline_dryrun")
    p.add_argument("--duration",type=float,default=20.)
    p.add_argument("--stride",type=int,default=3)
    p.add_argument("--blend-radius-mm",type=float,default=0.5)
    args=p.parse_args()
    print(json.dumps(run_offline(args.artifact,args.output,args.duration,args.stride,args.blend_radius_mm),indent=2))


if __name__=="__main__":
    main()
