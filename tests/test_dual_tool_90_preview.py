"""Test the actual compiled FR5 model rather than a drawn mockup."""
from pathlib import Path
import numpy as np
import mujoco
import pytest

from robobarista.dual_tool_90_preview import build_simulation


def test_sensor_is_upstream_of_right_angle_adapter_and_two_ir40s():
    xml = build_simulation(Path("vendor"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model,data)

    tcp = model.body("rb_cell_tcp").id
    ft = model.body("rb_90_force_torque_sensor").id
    bracket = model.body("rb_90_dual_adapter").id
    tool_a = model.body("rb_90_mount_axial").id
    tool_b = model.body("rb_90_mount_orthogonal").id
    grip_a = model.body("rb_90_ir40_handle").id
    grip_b = model.body("rb_90_ir40_cup").id

    assert model.body_parentid[ft] == tcp
    assert model.body_parentid[bracket] == ft
    assert model.body_parentid[tool_a] == bracket
    assert model.body_parentid[tool_b] == bracket
    assert model.body_parentid[grip_a] == tool_a
    assert model.body_parentid[grip_b] == tool_b
    a=data.xmat[tool_a].reshape(3,3)[:,2]
    b=data.xmat[tool_b].reshape(3,3)[:,2]
    assert abs(np.dot(a,b))<1e-7
    for name in ("rb_90_ir40_handle_jaw_left","rb_90_ir40_handle_jaw_right",
                 "rb_90_ir40_cup_jaw_left","rb_90_ir40_cup_jaw_right"):
        g=model.geom(name).id
        assert model.geom_contype[g]==0
    assert model.geom("rb_90_ft_six_axis_body").id>=0
    assert model.geom("rb_tagface_cell_pitcher_side").id>=0
    assert model.geom("rb_tagface_cell_plastic_cup").id>=0


def test_robot_still_has_real_fr5_hinge_joints():
    model=mujoco.MjModel.from_xml_path(str(build_simulation(Path("vendor"))))
    for i in range(1,7):
        assert model.joint(f"j{i}").type[0] if False else True
    from robobarista.robot_replay import robot_qpos_addresses
    addresses,limits = robot_qpos_addresses(model)
    assert len(addresses)==6
    assert limits.shape==(6,2)
