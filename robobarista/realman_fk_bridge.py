"""Convert RECORDED Realman RM65 joints to possible pitcher 6D poses.

This performs FK on a separately provided official RM65 URDF. Never assume
the dual-arm Realman joint commands are FR5 robot commands. User must supply:
- correct manufacturer URDF and link/joint names for the original RM65 arm;
- whether the original dataset joint states are degrees or radians;
- *measured* flange->pitcher rigid transform (or explicit uncalibrated
  --assume-identity-tool for a geometric feasibility demonstration only).

Camera-based remeasurement is a distinct downstream step. This FK is NOT
a camera observation of the original experiment.
"""
from __future__ import annotations
import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def homogeneous(p, rpy):
    T=np.eye(4)
    T[:3,:3]=Rotation.from_euler("xyz",rpy).as_matrix()
    T[:3,3]=p
    return T


def read_xyz(attr, default="0 0 0"):
    v=np.fromstring(attr if attr is not None else default,sep=" ")
    if v.shape!=(3,) or not np.isfinite(v).all():
        raise ValueError("Malformed URDF xyz/rpy/axis")
    return v


@dataclass(frozen=True)
class Joint:
    name:str
    child:str
    parent:str
    motion_type:str
    origin:np.ndarray
    axis:np.ndarray


def parse_urdf_chain(path,flange_link,joint_names):
    """Trace from target flange to base; refuse missing/branched chains."""
    root=ET.parse(path).getroot()
    if root.tag!="robot":
        raise ValueError("Expected URDF <robot> root")
    by_child={}
    for node in root.findall("joint"):
        child=node.find("child")
        parent=node.find("parent")
        if child is None or parent is None:
            raise ValueError("URDF joint missing parent or child")
        origin=node.find("origin")
        axis=node.find("axis")
        j=Joint(
            node.attrib["name"],child.attrib["link"],
            parent.attrib["link"],node.attrib.get("type","fixed"),
            homogeneous(
                read_xyz(origin.get("xyz") if origin is not None else None),
                read_xyz(origin.get("rpy") if origin is not None else None)),
            read_xyz(axis.get("xyz") if axis is not None else "1 0 0"))
        if j.child in by_child:
            raise ValueError("URDF has multiple parents for a link")
        by_child[j.child]=j
    chain=[]
    cur=flange_link
    visited=set()
    while cur in by_child:
        if cur in visited:
            raise ValueError("Cycle in URDF")
        visited.add(cur)
        j=by_child[cur]
        chain.append(j)
        cur=j.parent
    if not chain:
        raise ValueError(f"No URDF chain found to flange {flange_link!r}")
    chain.reverse()
    movable=[j.name for j in chain if j.motion_type in ("revolute","continuous")]
    if any(j.motion_type not in ("fixed","revolute","continuous") for j in chain):
        raise ValueError("Prismatic, floating or planar joints are not supported")
    if movable!=list(joint_names):
        raise ValueError(
            f"Record joint order does not match URDF chain: "
            f"URDF movable={movable}, supplied={list(joint_names)}")
    if len(movable)!=6:
        raise ValueError("Expected six RM65 motor joints to flange")
    return chain


def forward_kinematics(chain, angles_rad):
    angles=np.asarray(angles_rad,dtype=float)
    if angles.shape!=(6,) or not np.isfinite(angles).all():
        raise ValueError("Expected six finite RM65 joint positions")
    T=np.eye(4)
    idx=0
    for joint in chain:
        T=T@joint.origin
        if joint.motion_type in ("revolute","continuous"):
            axis=joint.axis
            norm=np.linalg.norm(axis)
            if norm<.5:raise ValueError("Bad URDF joint axis")
            turn=np.eye(4)
            turn[:3,:3]=Rotation.from_rotvec(
                axis/norm*angles[idx]).as_matrix()
            T=T@turn
            idx+=1
    return T


def validate_rigid_transform(value,label):
    T=np.asarray(value,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all():
        raise ValueError(f"{label} must be 4x4")
    if not np.allclose(T[3],[0,0,0,1],atol=1e-7):
        raise ValueError(f"{label}: homogeneous bottom row")
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-5) or np.linalg.det(R)<.999:
        raise ValueError(f"{label} not a rigid SE3 matrix")
    return T


def reconstruct_pitches(rows,chain,*,arm,joint_unit,flange_to_pitcher,
                        tool_calibrated=False):
    """Preserve *full* 6D motion; no 12D->6D truncation of joint commands."""
    if arm not in ("left","right"):
        raise ValueError("Select the arm that actually holds the pitcher")
    if joint_unit not in ("rad","deg"):
        raise ValueError("Explicit joint unit required")
    M=validate_rigid_transform(flange_to_pitcher,"flange_to_pitcher")
    from .real_latte_dataset import validate_realman_episode
    source=validate_realman_episode(rows)
    offset=0 if arm=="left" else 7
    ret=[]
    for row in rows:
        q=np.asarray(row["state"],float)[offset:offset+6]
        if joint_unit=="deg":q=np.deg2rad(q)
        T=forward_kinematics(chain,q)@M
        ret.append({
            "time_s":float(row["time_s"]),
            "position_m":T[:3,3].tolist(),
            "rotation_xyzw":Rotation.from_matrix(T[:3,:3]).as_quat().tolist()})
    delta=np.linalg.norm(np.asarray([r["position_m"] for r in ret])
                         -ret[0]["position_m"],axis=1)
    source.update({
        "source_arm":arm,
        "joint_unit_assumed":joint_unit,
        "source_6d_pitcher_pose_method":"manufacturer RM65 URDF FK + tool mount transform",
        "source_camera_6d_pitcher_measurement":False,
        "flange_to_pitcher_physically_calibrated":bool(tool_calibrated),
        "absolute_RM65_to_FR5_alignment_known":False,
        "source_pitcher_translation_excursion_mm":float(1000*max(delta)),
        "source_pitcher_pose_frames":len(ret),
        "FR5_command_generated":False,
        "real_milk_flow_modeled":False,
        "calibrated_real_world_spout_accuracy":False})
    return ret,source


def convert(*,input_joints,urdf,flange,joints,arm,unit,mount_file,
            output_dir,assume_identity_tool=False,start_s=None,end_s=None):
    if len(joints)!=6:
        raise ValueError("Provide six URDF joint names in correct dataset order")
    if bool(mount_file) == bool(assume_identity_tool):
        raise ValueError("Supply either measured mount JSON or explicit identity-tool mock")
    M=(np.eye(4) if assume_identity_tool else
       validate_rigid_transform(
           json.loads(Path(mount_file).read_text())["T_flange_pitcher"],
           "flange_to_pitcher"))
    rows=json.loads(Path(input_joints).read_text())
    original_count=len(rows)
    if start_s is not None or end_s is not None:
        if start_s is None or end_s is None or not 0<=start_s<end_s:
            raise ValueError("For a pour-only crop, provide valid start_s < end_s")
        rows=[r for r in rows if start_s<=float(r["time_s"])<=end_s]
        if len(rows)<8:
            raise ValueError("Chosen pour-only interval has fewer than 8 samples")
        t0=float(rows[0]["time_s"])
        rows=[{**r,"time_s":float(r["time_s"])-t0} for r in rows]
    chain=parse_urdf_chain(urdf,flange,joints)
    poses,report=reconstruct_pitches(
        rows,chain,arm=arm,joint_unit=unit,flange_to_pitcher=M,
        tool_calibrated=not assume_identity_tool)
    report["crop_applied"]=start_s is not None
    report["chosen_source_time_window_s"]=[start_s,end_s] if start_s is not None else None
    report["uncropped_episode_samples"]=original_count
    output=Path(output_dir)
    output.mkdir(parents=True,exist_ok=True)
    (output/"source_pitcher_6d_URDF_FK.json").write_text(
        json.dumps(poses,indent=2))
    (output/"source_pose_provenance.json").write_text(
        json.dumps(report,indent=2))
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--input-joints",required=True)
    p.add_argument("--urdf",required=True)
    p.add_argument("--flange",required=True,help="Exact last link of the official RM65 URDF")
    p.add_argument("--joint-names",nargs=6,required=True)
    p.add_argument("--arm",choices=["left","right"],required=True)
    p.add_argument("--joint-unit",choices=["deg","rad"],required=True)
    p.add_argument("--mount-json",default=None)
    p.add_argument("--start-s",type=float,default=None,
                   help="Start of a visually confirmed pour-only segment")
    p.add_argument("--end-s",type=float,default=None,
                   help="End of a visually confirmed pour-only segment")
    p.add_argument("--assume-identity-tool",action="store_true",
                   help="For uncalibrated geometric testing only")
    p.add_argument("--output",required=True)
    a=p.parse_args()
    result=convert(input_joints=a.input_joints,urdf=a.urdf,
                   flange=a.flange,joints=a.joint_names,arm=a.arm,
                   unit=a.joint_unit,mount_file=a.mount_json,
                   assume_identity_tool=a.assume_identity_tool,
                   output_dir=a.output,start_s=a.start_s,end_s=a.end_s)
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    main()
