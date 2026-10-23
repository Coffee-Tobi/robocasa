"""Offline DFKI movement_primitives adapter for AprilTag-derived latte pouring.

DMP learns ONLY a 3D virtual spout path from reconstructed pitcher poses.
It does not estimate or actuate FR5 joints, motor torques, force or milk flow.
Orientation is *not* learned by this positional DMP. Do not send the result
straight to a robot: collisions, tip/cup clearance and timed controller
execution require an independent robot motion-planning and safety gate.

External API: https://github.com/dfki-ric/movement_primitives (BSD-3-Clause).
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def measured_spout_series(camera_records, tip_offset_m):
    """Derive virtual tip xyz from camera OBSERVATIONS, never simulation truth."""
    if not isinstance(camera_records, list) or len(camera_records) < 8:
        raise ValueError("Need >=8 recorded camera poses")
    ts = np.asarray([row["time_s"] for row in camera_records], dtype=float)
    if not np.isfinite(ts).all() or not np.all(np.diff(ts) > 0):
        raise ValueError("Camera timestamps must be finite and increasing")
    if any(not r.get("valid", False) or r.get("visible_views", 0) < 2
           for r in camera_records):
        raise ValueError("Two visible calibrated camera views required per frame")
    xyz = np.asarray([r["position_m"] for r in camera_records], dtype=float)
    quat = np.asarray([r["rotation_xyzw"] for r in camera_records], dtype=float)
    offset = np.asarray(tip_offset_m, dtype=float)
    if xyz.shape != (len(ts), 3) or quat.shape != (len(ts), 4):
        raise ValueError("Invalid camera position or quaternion arrays")
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("Virtual pitcher-to-tip offset must be finite xyz")
    if not np.isfinite(xyz).all() or not np.isfinite(quat).all():
        raise ValueError("Nonfinite camera observations")
    if np.max(np.abs(np.linalg.norm(quat, axis=1) - 1.0)) > .002:
        raise ValueError("Non-unit camera quaternion")
    tips = xyz + Rotation.from_quat(quat).apply(offset)
    return ts - ts[0], tips


def imitate_virtual_spout(camera_records, *, virtual_tip_offset_m,
                          goal_offset_mm=(0., 0., 0.),
                          output_fps=20, n_weights=15, max_excursion_mm=70):
    """Learn DMP and produce an OFFLINE path, with explicit geometric limits.

    goal_offset_mm is a *small adaptation of the demonstration endpoint*.
    The virtual tip transform is NOT a measured real pitcher calibration.
    """
    try:
        from movement_primitives.dmp import DMP
    except ImportError as exc:
        raise RuntimeError(
            "Optional dependency absent; install 'movement_primitives>=0.9,<1'"
        ) from exc
    ts, positions = measured_spout_series(camera_records, virtual_tip_offset_m)
    duration = float(ts[-1])
    if not .1 <= duration <= 120:
        raise ValueError("Demonstration duration outside sensible range")
    if not isinstance(output_fps, int) or not 5 <= output_fps <= 120:
        raise ValueError("Output fps must be an integer in 5..120")
    if not isinstance(n_weights, int) or not 3 <= n_weights <= 50:
        raise ValueError("DMP basis count must be in 3..50")
    delta = np.asarray(goal_offset_mm, dtype=float)
    if delta.shape != (3,) or not np.isfinite(delta).all():
        raise ValueError("Goal offset must be finite xyz millimetres")
    if float(np.linalg.norm(delta)) > 20:
        raise ValueError("Offline retarget goal shift limited to 20 mm")
    if not 1 <= max_excursion_mm <= 150:
        raise ValueError("Invalid offline geometric excursion bound")

    dt = 1.0 / output_fps
    # Use upstream DFKI imitation rather than a self-written DMP/regulator.
    dmp = DMP(n_dims=3, execution_time=duration, dt=dt,
              n_weights_per_dim=n_weights, smooth_scaling=True)
    dmp.imitate(ts, positions)
    dmp.configure(start_y=positions[0], goal_y=positions[-1] + delta / 1000.0)
    t_out, p_out = dmp.open_loop(run_t=duration)
    p_out = np.asarray(p_out, float)
    t_out = np.asarray(t_out, float)
    if (p_out.ndim != 2 or p_out.shape[1] != 3 or
            t_out.shape != (len(p_out),) or len(t_out) < 2 or
            not np.isfinite(p_out).all() or not np.isfinite(t_out).all()):
        raise ValueError("DMP generated invalid spout trajectory")
    excursion = float(np.max(np.linalg.norm(p_out - p_out[0], axis=1))*1000)
    if excursion > max_excursion_mm:
        raise ValueError(f"Offline tip trajectory excursion {excursion:.2f} mm exceeds limit")
    return {
        "status": "OFFLINE_POSITIONAL_DMP_ONLY",
        "library": "dfki-ric/movement_primitives",
        "input": "AprilTag camera reconstructions (not synthetic ground truth)",
        "camera_frames": len(ts),
        "source_duration_s": duration,
        "source_tip_excursion_mm": float(np.max(
            np.linalg.norm(positions - positions[0], axis=1))*1000),
        "predicted_tip_excursion_mm": excursion,
        "goal_offset_mm": delta.tolist(),
        "output_fps_requested": output_fps,
        "time_s": t_out.tolist(),
        "virtual_spout_positions_m": p_out.tolist(),
        "orientation_learned": False,
        "tip_offset_physically_calibrated": False,
        "motor_control_or_collision_planning": False,
        "actual_milk_flow_or_pattern": False,
        "hardware_execution_allowed": False,
        "note": "DMP is a learning/retargeting experiment; requires physical spout calibration, independent MoveIt/FR5 planning and wet-flow feedback."
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--camera", required=True, help="camera_measurements.json")
    p.add_argument("--output", required=True)
    p.add_argument("--virtual-tip-offset-m", nargs=3, type=float, required=True)
    p.add_argument("--goal-offset-mm", nargs=3, type=float, default=(0., 0., 0.))
    p.add_argument("--fps", type=int, default=20)
    args = p.parse_args()
    rows = json.loads(Path(args.camera).read_text())
    result = imitate_virtual_spout(
        rows, virtual_tip_offset_m=args.virtual_tip_offset_m,
        goal_offset_mm=args.goal_offset_mm, output_fps=args.fps)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ("time_s", "virtual_spout_positions_m")}, indent=2))


if __name__ == "__main__":
    main()
