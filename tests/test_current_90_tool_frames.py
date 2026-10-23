"""Verify 90-degree dual-IR40 visual pose transforms WITHOUT robot actuation."""
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robobarista.coffee_workflow import _story_scene
from robobarista.robot_replay import robot_qpos_addresses
from robobarista.current_90_tool_frames import (
    model_90_degree_tool_frames, offline_virtual_pitcher_to_wrist,
)


def test_approved_90degree_mujoco_pitcher_tcp_not_identical_and_rigid():
    model = mujoco.MjModel.from_xml_path(str(_story_scene(Path("vendor"))))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    qids, _ = robot_qpos_addresses(model)
    original_q = data.qpos[qids].copy()
    first = model_90_degree_tool_frames(model, data)
    assert not np.allclose(first["tcp_to_pitcher"], np.eye(4))
    assert not np.allclose(first["tcp_to_visual_spout"], first["tcp_to_pitcher"])
    # The in-model kinematic attachment must retain its offset as FR5 joints move.
    moved_q = original_q.copy()
    moved_q[0] += .02
    moved_q[3] -= .03
    data.qpos[qids] = moved_q
    after = model_90_degree_tool_frames(model, data)
    assert np.allclose(first["tcp_to_pitcher"], after["tcp_to_pitcher"], atol=1e-6)
    assert np.allclose(first["tcp_to_visual_spout"], after["tcp_to_visual_spout"], atol=1e-6)
    assert not after["actual_spout_tip_calibrated_physically"]


def test_pitcher_source_retarget_maps_via_correct_tcp_offset():
    tcp = np.eye(4)
    tcp[:3,3] = [0., -.3, 1.]
    tcp_pitcher = np.eye(4)
    tcp_pitcher[:3,3] = [.13, 0, -.07]
    tcp_tip = np.eye(4)
    tcp_tip[:3,3] = [.18, 0, -.02]
    source = [
        {"time_s":i*.2, "position_m":[.4+i*.001, .1, .95],
         "rotation_xyzw":[0,0,0,1]} for i in range(11)
    ]
    result = offline_virtual_pitcher_to_wrist(
        source, world_tcp_start=tcp, tcp_to_pitcher=tcp_pitcher,
        tcp_to_visual_spout=tcp_tip, frames=11, fps=5)
    wrist=np.asarray(result["wrist_targets_se3_NOT_EXECUTABLE"])
    spout=np.asarray(result["virtual_spout_xyz_m"])
    assert wrist.shape==(11,4,4)
    assert wrist[0] == pytest.approx(tcp)
    assert wrist[-1][:3,3] - wrist[0][:3,3] == pytest.approx([.01,0,0])
    assert spout[0] == pytest.approx([.18, -.3, .98])
    assert not result["provenance"]["independent_6d_fr5_ik_executed"]
    assert not result["provenance"]["dual_tool_collision_envelopes_validated"]
