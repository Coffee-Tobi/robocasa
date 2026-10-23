"""Map SOURCE pitcher-relative poses to the CURRENT approved FR5 90-degree dual-IR40 tool.

Uses the real MuJoCo model's rigid transforms: FR5 TCP -> axial IR40 ->
kinematically held pitcher body -> visual spout geometry. Does not reuse the
old parallel gripper or old camera_joint_replay scene. No contact physics,
robot IK, real CAD, physical tip calibration or motion commands are implied.
"""
from __future__ import annotations

import numpy as np

from .external_pose_replay import align_imported_poses_to_fr5_start


def _transform(position, rotation):
    p = np.asarray(position, dtype=float)
    r = np.asarray(rotation, dtype=float)
    if p.shape != (3,) or r.shape != (3, 3) or not np.isfinite(p).all() or not np.isfinite(r).all():
        raise ValueError("Expected finite 3D rigid pose")
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or np.linalg.det(r) < .999:
        raise ValueError("Expected orthogonal right-handed rotation")
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = p
    return out


def model_90_degree_tool_frames(model, data):
    """Read symbolic 90-degree geometry from the current APPROVED MuJoCo model.

    Pitcher frame is rb_gripped_pitcher BODY, visual spout is the center of
    rb_gripped_pitcher_spout GEOM, not measured physical pour-tip geometry.
    Requires a scene with kinematically held props, e.g. _story_scene.
    """
    import mujoco
    mujoco.mj_forward(model, data)
    tcp_id = model.body("rb_cell_tcp").id
    grip_id = model.body("rb_90_ir40_handle").id
    pitcher_id = model.body("rb_gripped_pitcher").id
    spout_id = model.geom("rb_gripped_pitcher_spout").id
    parent = pitcher_id
    while parent > 0 and parent != grip_id:
        parent = int(model.body_parentid[parent])
    if parent != grip_id:
        raise ValueError("Pitcher is not mounted under approved axial 90-degree IR40")
    if int(model.geom_bodyid[spout_id]) != pitcher_id:
        raise ValueError("Visual spout does not belong to the held pitcher body")
    world_tcp = _transform(data.xpos[tcp_id], data.xmat[tcp_id].reshape(3, 3))
    world_pitcher = _transform(data.xpos[pitcher_id], data.xmat[pitcher_id].reshape(3, 3))
    world_spout = _transform(data.geom_xpos[spout_id], data.geom_xmat[spout_id].reshape(3, 3))
    tcp_pitcher = np.linalg.inv(world_tcp) @ world_pitcher
    tcp_spout = np.linalg.inv(world_tcp) @ world_spout
    return {
        "world_tcp": world_tcp,
        "tcp_to_pitcher": tcp_pitcher,
        "tcp_to_visual_spout": tcp_spout,
        "source": "approved 90deg IR40 MuJoCo kinematic geometry only",
        "pitcher_tool_calibrated_physically": False,
        "actual_spout_tip_calibrated_physically": False,
    }


def offline_virtual_pitcher_to_wrist(source_poses, *, world_tcp_start,
                                     tcp_to_pitcher, tcp_to_visual_spout,
                                     frames, fps):
    """Return UNEXECUTABLE targets for current FR5 WRIST, not old TCP=pitcher.

    First source RM65 pitcher pose is aligned with our current virtual held
    pitcher; relative 6D motion is preserved after time resampling. The source
    FK and its unknown flange->pitcher transform still need verification.
    NO IK, collision screening, real world calibration, or robot commands.
    """
    world_tcp_start = np.asarray(world_tcp_start, float)
    tcp_to_pitcher = np.asarray(tcp_to_pitcher, float)
    tcp_to_visual_spout = np.asarray(tcp_to_visual_spout, float)
    for name, mat in (("world_tcp_start", world_tcp_start),
                      ("tcp_to_pitcher", tcp_to_pitcher),
                      ("tcp_to_visual_spout", tcp_to_visual_spout)):
        if mat.shape != (4, 4) or not np.isfinite(mat).all() or not np.allclose(mat[3], [0,0,0,1]):
            raise ValueError(name + " must be homogeneous 4x4")
        _transform(mat[:3,3],mat[:3,:3])
    first_pitcher = world_tcp_start @ tcp_to_pitcher
    targets, report = align_imported_poses_to_fr5_start(
        source_poses, first_pitcher[:3,3], first_pitcher[:3,:3], frames, fps)
    wrist_targets = []
    spout_positions = []
    for position, rotation in targets:
        world_pitcher = _transform(position, rotation)
        world_tcp = world_pitcher @ np.linalg.inv(tcp_to_pitcher)
        world_spout = world_tcp @ tcp_to_visual_spout
        wrist_targets.append(world_tcp.tolist())
        spout_positions.append(world_spout[:3,3].tolist())
    if not np.allclose(wrist_targets[0], world_tcp_start, atol=1e-7):
        raise ValueError("Retargeted first wrist pose differs from approved starting wrist")
    report.update({
        "target_frame": "FR5 rb_cell_tcp via approved axial IR40 visual attachment",
        "assumes_realman_pitcher_frame_calibrated": False,
        "visual_spout_geometry_only": True,
        "independent_6d_fr5_ik_executed": False,
        "dual_tool_collision_envelopes_validated": False,
        "real_controller_motion_generated": False
    })
    return {"wrist_targets_se3_NOT_EXECUTABLE": wrist_targets,
            "virtual_spout_xyz_m": spout_positions,
            "provenance": report}
