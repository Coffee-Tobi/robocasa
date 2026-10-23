"""Safe, geometry-only replay of REALMAN-FK-source poses in virtual FR5 cell.

This is a 2-step validation:
1. independently recover RM65 pitcher SE3 from recorded *real* joint states
   and a correctly selected Realman URDF + verified tool fixture;
2. align relative RM65 pitcher motion with a collision-free FR5 start,
   solve FR5 kinematic IK, render a NEW set of 3/4 RGB-D AprilTag cameras,
   and re-estimate it from rendered pixels only; this module exports NO IK entrypoint until the approved 90-degree gripper-to-pitcher transform is available.

The simulated virtual tags were NOT attached to the original real pitcher:
results cannot claim the original videos contain metric AprilTag tracking.
No real FAIRINO firmware, motor dynamics, contact or milk simulation.
No direct wrist IK replay: current 90-degree FR5 wrist and pitcher frames differ.
"""
from __future__ import annotations
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def validate_source_trajectory(poses,max_excursion_m=.12):
    if not isinstance(poses,list) or len(poses)<8:
        raise ValueError("Require >=8 imported 6D source-pitcher poses")
    t=np.asarray([r["time_s"] for r in poses],dtype=float)
    p=np.asarray([r["position_m"] for r in poses],dtype=float)
    q=np.asarray([r["rotation_xyzw"] for r in poses],dtype=float)
    if p.shape!=(len(t),3) or q.shape!=(len(t),4):
        raise ValueError("Expected time, xyz and xyzw for each source pose")
    if not (np.isfinite(t).all() and np.isfinite(p).all()
            and np.isfinite(q).all() and np.all(np.diff(t)>0)):
        raise ValueError("Bad time axis or source pose")
    if np.max(abs(np.linalg.norm(q,axis=1)-1))>.002:
        raise ValueError("Nonunit source RM65 quaternion")
    duration=float(t[-1]-t[0])
    if duration<.1 or duration>120:
        raise ValueError("Imported episode duration unsupported")
    excursion=float(np.max(np.linalg.norm(p-p[0],axis=1)))
    if excursion>max_excursion_m:
        raise ValueError(
            f"RM65 source pitcher moved {excursion*1000:.1f} mm; "
            "this exceeds the restricted latte POUR segment. "
            "Extract a pouring-only portion; never scale an entire pickup secretly.")
    return t,p,Rotation.from_quat(q),{
        "original_source_duration_s":duration,
        "source_max_3d_translation_excursion_mm":excursion*1000,
        "source_poses":len(t)
    }


def align_imported_poses_to_fr5_start(source_poses,start_position,start_rotation,
                                     n_frames,fps,*,max_excursion_m=.12):
    """Rigid alignment + temporal resampling; no fake sensor measurements."""
    t,p,r,stats=validate_source_trajectory(source_poses,max_excursion_m)
    start=np.asarray(start_position,float)
    rot=np.asarray(start_rotation,float)
    if start.shape!=(3,) or rot.shape!=(3,3):
        raise ValueError("Invalid FR5 reference TCP pose")
    if n_frames<7 or not 1<=fps<=120:
        raise ValueError("Unsupported virtual camera frame settings")
    ts=np.linspace(t[0],t[-1],n_frames)
    new_p=np.column_stack([np.interp(ts,t,p[:,i]) for i in range(3)])
    new_r=Slerp(t,r)(ts)
    # All source motions are expressed relative to FIRST REALMAN pitcher
    # pose. The initial frame is aligned to a collision-free virtual FR5 TCP.
    # This intentionally discards unknown absolute dual RM65 base placement.
    source_first=r[0].as_matrix()
    out_p=[start + rot @ source_first.T @ (v-p[0]) for v in new_p]
    out_r=[rot @ source_first.T @ v.as_matrix()
           for v in new_r]
    if np.linalg.norm(out_p[0]-start)>1e-6:
        raise ValueError("First imported pose failed rigid alignment")
    stats.update({
        "resampled_frames":n_frames,
        "new_virtual_duration_s":(n_frames-1)/fps,
        "absolute_source_machine_alignment_preserved":False,
        "relative_rotation_and_translation_preserved":True,
        "robot_dynamics_simulated":False
    })
    return [(np.asarray(v),np.asarray(R))
            for v,R in zip(out_p,out_r)],stats


