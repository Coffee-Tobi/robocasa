"""Regression gates for PR#4 kinematic props on the approved 90° tool."""
from pathlib import Path
import numpy as np
import mujoco
import pytest
from robobarista.coffee_workflow import _story_scene


def model(style="handle_mug"):
    return mujoco.MjModel.from_xml_path(str(_story_scene(Path("vendor"),cup_style=style)))


def ancestor_chain(m,b):
    ids=[]
    while b:
        ids.append(int(b)); b=int(m.body_parentid[b])
    return ids


def test_existing_real_fr5_90deg_not_replaced_by_old_parallel_greippers():
    m=model()
    names=[m.body(i).name for i in range(m.nbody)]
    assert "rb_handle_tool_IR40_50" not in names
    assert "rb_cup_tool_IR40_50" not in names
    handle=m.body("rb_90_ir40_handle").id
    cup=m.body("rb_90_ir40_cup").id
    assert handle!=cup
    assert m.body("rb_gripped_pitcher").id in range(m.nbody)
    assert handle in ancestor_chain(m,m.body("rb_gripped_pitcher").id)
    assert handle in ancestor_chain(m,m.body("rb_gripped_portafilter").id)
    assert cup in ancestor_chain(m,m.body("rb_gripped_cup").id)
    for other in ("rb_90_force_torque_sensor","rb_90_dual_adapter"):
        assert m.body(other).id>=0
    d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    a=d.xmat[m.body("rb_90_mount_axial").id].reshape(3,3)[:,2]
    b=d.xmat[m.body("rb_90_mount_orthogonal").id].reshape(3,3)[:,2]
    assert np.dot(a,b)==pytest.approx(0,abs=1e-8)


def test_grip_center_is_between_fingers_and_cup_has_real_rendered_handle():
    m=model()
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    l=m.geom("rb_90_ir40_handle_jaw_left").id
    r=m.geom("rb_90_ir40_handle_jaw_right").id
    adapter=m.geom("rb_gripped_pitcher_adapter").id
    center=(d.geom_xpos[l]+d.geom_xpos[r])*0.5
    assert np.linalg.norm(center-d.geom_xpos[adapter])<.0011
    for g in ("rb_gripped_cup_handle_upper","rb_gripped_cup_handle_lower",
              "rb_gripped_cup_handle_outer",
              "rb_free_mug_handle_upper",
              "rb_free_mug_handle_lower","rb_free_mug_handle_outer"):
        assert m.geom(g).id>=0
    a=m.body("rb_gripped_cup").id
    # Cup is upright relative to a level TCP: Rside=Ry(+90),
    # jaw spin=Rz(+90), cup payload=inverse(Rside@spin).
    tcp=m.body("rb_cell_tcp").id
    Rt=d.xmat[tcp].reshape(3,3)
    Rc=d.xmat[a].reshape(3,3)
    assert np.allclose(Rt.T@Rc,np.eye(3),atol=1e-7)
    # Bowl must be beside, not underneath, the clamping jaw centre.
    bowl=m.geom("rb_gripped_cup_body").id
    jaw=m.geom("rb_90_ir40_cup_jaw_left").id
    jaw2=m.geom("rb_90_ir40_cup_jaw_right").id
    gr=(d.geom_xpos[jaw]+d.geom_xpos[jaw2])/2
    rel=Rt.T@(d.geom_xpos[bowl]-gr)
    assert rel[0]>.07 and abs(rel[2])<.01
    # The closing direction of jaws is sideways in TCP (not vertical).
    close=Rt.T@(d.geom_xpos[jaw]-d.geom_xpos[jaw2])
    assert abs(close[2])<1e-7


def test_takeaway_has_no_fake_handle_but_reuses_orthogonal_arm():
    m=model("takeaway")
    assert m.body("rb_gripped_cup").id>=0
    with pytest.raises(KeyError):m.geom("rb_gripped_cup_handle_upper")
    with pytest.raises(KeyError):m.geom("rb_free_mug_handle_upper")
    assert m.body("rb_90_force_torque_sensor").id>=0


def test_held_props_are_visual_and_no_force_claim():
    m=model()
    for name in ("rb_gripped_pitcher_body",
                 "rb_gripped_portafilter_basket","rb_gripped_cup_body",
                 "rb_90_ir40_cup_housing"):
        assert m.geom_contype[m.geom(name).id]==0
        assert m.geom_conaffinity[m.geom(name).id]==0
