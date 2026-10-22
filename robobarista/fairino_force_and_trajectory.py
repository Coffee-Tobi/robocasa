"""FAIRINO SDK lookahead + F/T guard: OFFLINE unit-testable integration.

FAIRINO LoadTrajectoryLA takes the name of an ALREADY UPLOADED vendor-format
joint trajectory, not an arbitrary AprilTag / Cartesian point cloud!
It requires the correct TrajectoryJ log format, GetTrajectoryStartPose and
a separately validated approach motion. This module records that interface
without ever executing physical movement.

FT_GetForceTorqueRCS() requires a physically installed/configured FT sensor.
The *demonstration* contact thresholds are illustrative only and must not be
used as collision/safety limits on a physical robot.
"""
from __future__ import annotations
import math
from dataclasses import dataclass

DOC_TRAJ="https://manual.fairino.support/latest/SDKManual/PythonRobotTrajectoryRecurrence.html"
DOC_FT="https://manual.fairino.support/latest/SDKManual/PythonRobotForceControl.html"


@dataclass(frozen=True)
class LookaheadConfig:
    filename:str
    mode:int=1
    error_limit_mm:float=0.5
    smooth_type:int=0
    precision_mm:float=0.5
    max_speed_mm_s:float=35.
    max_accel_mm_s2:float=120.
    max_jerk_mm_s3:float=750.
    constant_velocity_flag:int=0

    def __post_init__(self):
        if not self.filename or "/" in self.filename or "\\" in self.filename:
            raise ValueError("Use pre-uploaded, checked trajectory filename only")
        if self.mode not in (0,1,2) or self.smooth_type!=0:
            raise ValueError("Unsupported FAIRINO trajectory lookahead mode/type")
        if self.constant_velocity_flag not in (0,1):
            raise ValueError("flag must be 0 or 1")
        for n in ("error_limit_mm","precision_mm","max_speed_mm_s",
                  "max_accel_mm_s2","max_jerk_mm_s3"):
            v=float(getattr(self,n))
            if not math.isfinite(v) or v<=0:
                raise ValueError(n+" must be finite positive")


def lookahead_sequence(robot,cfg:LookaheadConfig,uploaded_verified=False):
    """Only accept MOCK robot; real controller requires separate safety release."""
    if not isinstance(robot,RecordingFairino):
        raise RuntimeError("Physical robot actuation is deliberately disabled")
    if not uploaded_verified:
        raise ValueError("FAIRINO formatted joint file must be uploaded/validated first")
    args=(cfg.filename,cfg.mode,cfg.error_limit_mm,cfg.smooth_type,
          cfg.precision_mm,cfg.max_speed_mm_s,cfg.max_accel_mm_s2,
          cfg.max_jerk_mm_s3,cfg.constant_velocity_flag)
    ret=robot.LoadTrajectoryLA(*args)
    if ret!=0:raise RuntimeError(f"LoadTrajectoryLA error {ret}")
    # Motion stays DISABLED until start pose, tool calibration, safety and
    # true joint log are verified on an actual supported FAIRINO controller.
    return {"api":"LoadTrajectoryLA","args":list(args),
            "mock_upload_gate_exercised":True,
            "actual_vendor_trajectory_format_validated":False,
            "real_motion_executed":False,
            "move_trajectory_la_not_called":True,
            "NOTE":"0.5mm is the REQUESTED planning parameter, not measured path error"}


@dataclass(frozen=True)
class Wrench:
    fx:float=0.
    fy:float=0.
    fz:float=0.
    tx:float=0.
    ty:float=0.
    tz:float=0.

    @classmethod
    def from_sdk(cls,values):
        if len(values)!=6 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError("FT requires six finite values")
        return cls(*map(float,values))


class RecordingFairino:
    """Records exactly documented names and signature; NO SDK/robot contact."""
    def __init__(self,force_samples=()):
        self.calls=[]
        self.samples=iter(force_samples)
    def LoadTrajectoryLA(self,*args):
        self.calls.append(("LoadTrajectoryLA",list(args)));return 0
    def MoveTrajectoryLA(self):
        raise RuntimeError("Must not actuate outside qualified hardware adapter")
    def FT_GetForceTorqueRCS(self):
        self.calls.append(("FT_GetForceTorqueRCS",[]))
        return 0,next(self.samples,[0.,0.,0.,0.,0.,0.])
    def FT_Guard(self,*args):
        self.calls.append(("FT_Guard",list(args)));return 0


def get_wrench(robot):
    if not isinstance(robot,RecordingFairino):
        raise RuntimeError("Only mock force samples; physical FT requires separate integration")
    code,data=robot.FT_GetForceTorqueRCS()
    if code!=0:raise RuntimeError("No valid F/T sensor reading")
    return Wrench.from_sdk(data)


def portafilter_lock_monitor(samples,engage_torque_min_nm=0.4,
                             max_torque_nm=1.1,max_force_n=12.):
    """Offline training rules, not certified portafilter insertion control.

    The measured axis and sign must be calibrated for the actual bayonet
    handle orientation. Clamp and stop on unexpected force rather than
    increasing insertion torque blindly.
    """
    obs=list(samples)
    if not obs:raise ValueError("No F/T samples")
    torque=max(abs(w.tz) for w in obs)
    force=max(math.sqrt(w.fx*w.fx+w.fy*w.fy+w.fz*w.fz) for w in obs)
    if torque>max_torque_nm or force>max_force_n:
        return {"state":"STOP_OVERLOAD","peak_torque_nm":torque,"peak_force_n":force}
    if torque<engage_torque_min_nm:
        return {"state":"NOT_CONFIRMED","peak_torque_nm":torque,"peak_force_n":force}
    return {"state":"RESISTANCE_SEEN_NOT_LOCK_PROVEN",
            "peak_torque_nm":torque,"peak_force_n":force}


def steam_wand_side_contact(samples,contact_min_n=0.45,
                            max_side_force_n=2.5):
    """Normal force is assumed along a CALIBRATED sensor y-axis (offline)."""
    obs=list(samples)
    if not obs:raise ValueError("No F/T samples")
    lateral=max(abs(w.fy) for w in obs)
    if lateral>max_side_force_n:
        return {"state":"STOP_EXCESS_LATERAL_FORCE","peak_side_n":lateral}
    return {"state":"LIGHT_CONTACT_DETECTED" if lateral>=contact_min_n
              else "NO_CONTACT_DETECTED","peak_side_n":lateral}


def illustrative_contact_curve(action,progress):
    """Training-only *synthetic spring* model independent of MuJoCo contacts.

    This is NOT data from FR5 force sensor or a claim about actual docking.
    """
    x=max(0.,min(1.,float(progress)))
    if action=="insert_grouphead":
        torque=0.08+0.66*min(1.,max(0.,(x-.55)/.4))**1.8
        side=0.0
        return Wrench(fz=.5+1.3*x,tz=torque), "SYNTHETIC BAYONET TORQUE"
    if action=="steam":
        side=1.2*min(1.,max(0.,(x-.6)/.20))
        return Wrench(fy=side), "SYNTHETIC WAND SIDE CONTACT"
    return Wrench(),"NO SENSOR CONTACT IN THIS PHASE"
