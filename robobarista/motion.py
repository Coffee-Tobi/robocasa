"""Deterministic, smooth *kinematic* mock pouring gesture.

This is a synthetic motion scripted for a virtual pitcher, NOT an observed
human demonstration and NOT a collision-checked real robot trajectory.
The only values vision may use are separately rendered camera frames.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class MotionSample:
    phase: str
    position_m: np.ndarray
    rotation: np.ndarray


def smoothstep(t: float) -> float:
    v=float(np.clip(t, 0.0, 1.0))
    return v*v*(3.0-2.0*v)


def motion_sample(progress: float, start_position_m, start_rotation) -> MotionSample:
    """Script a short approach -> tilt -> pour gesture -> untilt -> retreat."""
    t=float(progress)
    if not np.isfinite(t) or t < 0 or t > 1:
        raise ValueError("progress must be a finite number in [0, 1]")
    start=np.asarray(start_position_m, dtype=float).reshape(3)
    baseline=np.asarray(start_rotation, dtype=float).reshape(3,3)

    approach=np.array([-0.040, 0.185, 0.068], dtype=float)
    sweep=np.array([-0.012, 0.012, 0.0], dtype=float)
    if t < .28:
        phase="approach"
        offset=approach*smoothstep(t/.28)
        tilt=0.
    elif t < .48:
        phase="tilt"
        offset=approach
        tilt=26.*smoothstep((t-.28)/.20)
    elif t < .68:
        phase="mock_pour"
        alpha=smoothstep((t-.48)/.20)
        offset=approach + alpha*sweep
        tilt=26.
    elif t < .83:
        phase="untilt"
        offset=approach+sweep
        tilt=26.*(1.-smoothstep((t-.68)/.15))
    else:
        phase="retreat"
        offset=(approach+sweep)*(1.-smoothstep((t-.83)/.17))
        tilt=0.

    rotation=Rotation.from_euler("y",tilt,degrees=True).as_matrix() @ baseline
    return MotionSample(phase=phase,position_m=start+offset,rotation=rotation)


def frame_times(frame_count: int, fps: float):
    """Return normalized progress and monotonic timestamps, including endpoints."""
    if not 2 <= frame_count <= 1000:
        raise ValueError("frame_count must be 2..1000")
    if not np.isfinite(fps) or fps<=0 or fps>120:
        raise ValueError("fps must be >0 and <=120")
    for i in range(frame_count):
        yield i, i/fps, i/(frame_count-1)


def world_spout(position_m, rotation, spout_local_m):
    return np.asarray(position_m,float).reshape(3) + (
        np.asarray(rotation,float).reshape(3,3) @ np.asarray(spout_local_m,float).reshape(3))


def valid_pose(estimate, camera_count: int, min_cameras: int=2):
    return bool(
        estimate is not None and
        estimate.get("converged",False) and camera_count >= min_cameras and
        np.all(np.isfinite(np.asarray(estimate["position"],float))) and
        np.all(np.isfinite(np.asarray(estimate["rotation"],float)))
    )
