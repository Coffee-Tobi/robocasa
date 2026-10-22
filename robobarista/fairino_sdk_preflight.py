"""Read-only FAIRINO FR5 telemetry gates & eight-solution IK option scorer.

API references:
https://manual.fairino.support/latest/SDKManual/PythonRobotStatusInquiry.html
https://manual.fairino.support/latest/SDKManual/PythonRobotCommonSettings.html
https://manual.fairino.support/latest/SDKManual/PythonRobotForceControl.html

NOT a robot controller. The mock exercises documented official method names
and response data; no real RPC, ServoCart, MoveJ, MoveL or guard actuation.
Actual physical motion requires vendor-specific calibration, safety
configuration, payload/FT identification and independent approval.
"""
from __future__ import annotations
from dataclasses import dataclass
import math


def _sdk_get(robot,method,*args):
    value=getattr(robot,method)(*args)
    if not isinstance(value,(tuple,list)) or len(value)!=2:
        raise RuntimeError(f"{method}: wrong SDK response shape")
    code,payload=value
    if code!=0:
        raise RuntimeError(f"{method}: SDK error {code}")
    return payload


def read_only_snapshot(robot):
    """ALL getters, no control; validates chain before any replay approval."""
    joints=_sdk_get(robot,"GetActualJointPosDegree",0)
    tcp=_sdk_get(robot,"GetActualTCPPose",0)
    velocities=_sdk_get(robot,"GetActualJointSpeedsDegree",0)
    ft=_sdk_get(robot,"FT_GetForceTorqueRCS")
    done=_sdk_get(robot,"GetRobotMotionDone")
    queued=_sdk_get(robot,"GetMotionQueueLength")
    emergency=_sdk_get(robot,"GetRobotEmergencyStopState")
    safety=_sdk_get(robot,"GetSafetyStopState")
    def vec(name,a,n):
        if not isinstance(a,(list,tuple)) or len(a)!=n or not all(math.isfinite(float(x)) for x in a):
            raise ValueError(f"{name}: invalid {n}-D feedback")
        return [float(x) for x in a]
    result={"joint_deg":vec("joints",joints,6),
        "tcp_mm_deg":vec("TCP",tcp,6),
        "joint_speed_deg_s":vec("speed",velocities,6),
        "force_torque_N_Nm":vec("FT",ft,6),
        "motion_done":int(done),"queue_length":int(queued),
        "emergency_stop":int(emergency),
        "safety_stops":vec("safety",safety,2),
        "hardware_provenance_verified":False}
    result["motion_authorizable_from_this_snapshot_alone"]=False
    result["stop_or_hold"]=bool(result["emergency_stop"]!=0 or
         any(s!=0 for s in result["safety_stops"]) or result["queue_length"]<0)
    return result


def select_nearest_ik(candidates,seed_deg,joint_limits_deg):
    """Rank vendor TCFToAllJoint candidates; no collision proof.

    Keep UNWRAPPED joint angles: selecting by nearest modulo 360 would
    disguise a physical joint-limit crossing. Never treat a valid IK
    candidate as a collision-free physical path.
    """
    seed=[float(x) for x in seed_deg]
    limits=list(joint_limits_deg)
    if len(seed)!=6 or len(limits)!=6:
        raise ValueError("6 seed and 6 joint bounds required")
    best=None
    for i,q in enumerate(candidates):
        if not isinstance(q,(list,tuple)) or len(q)!=6:
            continue
        if not all(math.isfinite(float(x)) for x in q):
            continue
        if any(not float(bounds[0])<=float(v)<=float(bounds[1])
               for v,bounds in zip(q,limits)):
            continue
        dist=sum((float(v)-ref)**2 for v,ref in zip(q,seed))
        if best is None or dist<best["squared_joint_distance_deg2"]:
            best={"vendor_branch_index":i,
                  "joint_deg":[float(x) for x in q],
                  "squared_joint_distance_deg2":dist,
                  "collision_checked":False,
                  "real_controller_used":False}
    if best is None:
        raise ValueError("No in-limit FAIRINO IK branches available")
    return best


def query_vendor_ik_branches(robot,target_tcp_mm_deg,tool_id,user_id,
                             seed_deg,joint_limits_deg):
    """Supported official method: TCFToAllJoint(tcp_pose,tool,workpiece,exPos)."""
    if len(target_tcp_mm_deg)!=6 or not all(math.isfinite(float(x)) for x in target_tcp_mm_deg):
        raise ValueError("invalid target")
    if not 0<=tool_id<=15 or not 0<=user_id<=15:
        raise ValueError("invalid calibrated tool/workpiece ID")
    all_joints=_sdk_get(robot,"TCFToAllJoint",list(target_tcp_mm_deg),
                       tool_id,user_id,[0.,0.,0.,0.])
    if len(all_joints)!=8:
        raise ValueError("FAIRINO returned unexpected number of IK branches")
    return select_nearest_ik(all_joints,seed_deg,joint_limits_deg)


def hardware_readiness_gate(*,world_calibrated=False,tool_tip_calibrated=False,
                            payload_and_cog_verified=False,
                            gripper_load_grasp_tested=False,
                            ft_calibrated=False,safe_machine_contact_tested=False,
                            robot_controller_trajectory_tested=False,
                            milk_flow_observable=False,
                            wet_trials_passed=False):
    fields=locals()
    pending=[key for key,v in fields.items() if v is not True]
    return {"hardware_ready_for_autonomous_espresso_latte_art":not pending,
            "missing_evidence":pending,
            "virtual_precision_does_not_override_hardware_gates":True,
            "next_milestone":"Supervised dry manipulation without hot water/steam",
            "autonomous_hot_liquid_operation_approved":False}
