"""Isolation test: only grinder + level custom-handled portafilter, two controllable joints."""
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robobarista.swift_mini_portafilter_only import (
    build_minimal_scene, get_phases, receiver_force_n,
    JOINT_Z, JOINT_YAW, PF_NAME, RECEIVER, LIP, UP_FORCE_LIMIT_N,
    YAW_TORQUE_LIMIT_NM, RIGHT_TWIST_DEG, TOTAL_S,
)


def _fixture(tmp_path):
    model=mujoco.MjModel.from_xml_path(
        str(build_minimal_scene(tmp_path/"standalone.xml")))
    data=mujoco.MjData(model)
    return model,data


def test_exactly_two_joints_no_tracking_cameras_or_robot(tmp_path):
    m,d=_fixture(tmp_path)
    assert m.ncam==0
    assert m.njnt==2
    assert m.nsensor==0
    assert m.joint(JOINT_Z).type==mujoco.mjtJoint.mjJNT_SLIDE
    assert m.joint(JOINT_YAW).type==mujoco.mjtJoint.mjJNT_HINGE
    assert m.joint(JOINT_Z).axis==pytest.approx([0,0,1])
    assert m.joint(JOINT_YAW).axis==pytest.approx([0,0,1])
    assert m.body(PF_NAME).id>=0
    assert m.geom(RECEIVER).id>=0
    assert m.geom(LIP).id>=0
    with pytest.raises(KeyError):
        m.body("wrist3_link")
    with pytest.raises(KeyError):
        m.body("rb_cell_tcp")
    with pytest.raises(KeyError):
        m.joint("rb_pf_slide_x")
    assert m.actuator("rb_pf_lift_servo").forcerange==pytest.approx([0,UP_FORCE_LIMIT_N])
    assert m.actuator("rb_pf_yaw_servo").forcerange==pytest.approx(
        [-YAW_TORQUE_LIMIT_NM,YAW_TORQUE_LIMIT_NM])


def test_all_phases_keep_basket_horizontal_and_pivot_fixed(tmp_path):
    m,d=_fixture(tmp_path)
    iz=m.joint(JOINT_Z).qposadr[0]
    iy=m.joint(JOINT_YAW).qposadr[0]
    center=None
    for qz,deg in ((0,0),(.025,0),(.055,0),(.071,0),(.071,12)):
        d.qpos[iz]=qz
        d.qpos[iy]=np.deg2rad(deg)
        mujoco.mj_forward(m,d)
        p=np.asarray(d.xpos[m.body(PF_NAME).id]).copy()
        R=np.asarray(d.xmat[m.body(PF_NAME).id]).reshape(3,3)
        assert R[2,2]==pytest.approx(1,abs=1e-8)
        assert np.max(np.abs(R[:2,2]))<1e-8
        if center is None:center=p[:2]
        else:assert p[:2]==pytest.approx(center,abs=1e-8)


def test_hypothetical_right_rotation_not_required_by_actual_swift():
    assert get_phases(.1)[0]=="ALIGN"
    assert get_phases(1.5)[0]=="GENTLE_LIFT"
    assert get_phases(2.3)[0]=="CONTACT_HOLD"
    assert get_phases(3.3)[0]=="OPTIONAL_RIGHT_TWIST"
    assert get_phases(4.4)[0]=="HOLD"
    assert np.rad2deg(get_phases(4.4)[2])==pytest.approx(RIGHT_TWIST_DEG)
    assert get_phases(4.4,with_right_turn=False)[2]==0.
    assert get_phases(4.4)[1]>0.
