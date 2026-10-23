"""Check actual MuJoCo stationary cup pivot + one angular DOF; no fake 6D cup motion."""
from pathlib import Path

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robobarista.coffee_workflow import _story_scene
from robobarista.one_arm_fixed_cup_hinge import (
    add_fixed_pivot_one_axis_cup, tilt_profile, JOINT, BODY,
    MAX_TILT_DEG, MAX_TILT_RATE_DEG_S,
)


def _compiled(tmp_path):
    scene=_story_scene(Path("vendor"), cup_style="handle_mug")
    return mujoco.MjModel.from_xml_path(
        str(add_fixed_pivot_one_axis_cup(scene,tmp_path/"hinged_cup_scene.xml")))


def test_hinge_is_actual_single_axis_joint_with_fixed_base(tmp_path):
    m=_compiled(tmp_path)
    d=mujoco.MjData(m)
    hinge=m.joint(JOINT).id
    assert m.jnt_type[hinge]==mujoco.mjtJoint.mjJNT_HINGE
    assert m.jnt_limited[hinge]
    assert m.jnt_axis[hinge]==pytest.approx([0,1,0])
    assert m.jnt_range[hinge]==pytest.approx(np.deg2rad([-18,18]),abs=1e-6)
    holder=m.body(BODY).id
    base=m.body("rb_stationary_cup_tilt_base").id
    assert m.body_parentid[holder]==base
    assert m.body_jntnum[holder]==1
    qid=m.jnt_qposadr[hinge]
    pivots=[]
    rim_positions=[]
    for angle in (0.,8.,14.,0.):
        d.qpos[qid]=np.deg2rad(angle)
        mujoco.mj_forward(m,d)
        pivots.append(d.xpos[holder].copy())
        rim_positions.append(d.geom_xpos[m.geom("rb_tilt_cup_rim").id].copy())
        R=d.xmat[holder].reshape(3,3)
        assert R==pytest.approx(Rotation.from_euler("y",angle,degrees=True).as_matrix(),abs=1e-6)
    assert max(np.linalg.norm(x-pivots[0]) for x in pivots)<1e-9
    # Cup points ARE allowed to trace arcs around the static pivot.
    assert np.linalg.norm(rim_positions[0]-rim_positions[2])>.01


def test_scripted_tilt_has_single_bounded_degree_of_freedom():
    t=np.linspace(0,4,13)
    a,meaning=tilt_profile(t,mode="scripted")
    assert a.shape==(13,)
    assert a[0]==pytest.approx(12.)
    assert a[-1]==pytest.approx(0.)
    assert np.min(a)>=-MAX_TILT_DEG
    assert np.max(a)<=MAX_TILT_DEG
    assert max(abs(np.diff(a)/np.diff(t)))<=MAX_TILT_RATE_DEG_S+1e-9
    assert "NOT RealMan" in meaning


def test_source_projection_requires_actual_recording_and_urdf():
    t=np.linspace(0,4,13)
    with pytest.raises(ValueError,match="needs original"):
        tilt_profile(t,mode="projected-source")
    with pytest.raises(ValueError,match="initial angle"):
        tilt_profile(t,mode="scripted",angle_start_deg=24)
    with pytest.raises(ValueError,match="increasing"):
        tilt_profile([0,0,1],mode="scripted")
