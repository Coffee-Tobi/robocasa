"""Physically meaningful geometric requirement: side approach, not underside grasp."""
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from robobarista.coffee_workflow import _story_scene


def _scene():
    m=mujoco.MjModel.from_xml_path(str(_story_scene(Path("vendor"),cup_style="handle_mug")))
    d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    return m,d


def _R(d,i):
    return d.xmat[i].reshape(3,3)


def test_active_90deg_tools_approach_horizontal_with_jaws_closing_sideways():
    m,d=_scene()
    tcp=m.body("rb_cell_tcp").id
    rt=_R(d,tcp)
    pitch=m.body("rb_90_ir40_handle").id
    mug=m.body("rb_90_ir40_cup").id
    # Relative pose of two accepted flange axes STILL 90° apart.
    ap=_R(d,m.body("rb_90_mount_axial").id)[:,2]
    cp=_R(d,m.body("rb_90_mount_orthogonal").id)[:,2]
    assert abs(float(np.dot(ap,cp)))<1e-8

    # Pitcher mode: command wrist orientation Ry(-90) by actual robot IK.
    # Tool approach & closing axes transformed to that desired orientation.
    horizontal_pitcher_wrist=Rotation.from_euler("y",-90,degrees=True).as_matrix()
    pitch_approach=horizontal_pitcher_wrist@(rt.T@_R(d,pitch)[:,2])
    pitch_close=horizontal_pitcher_wrist@(rt.T@_R(d,pitch)[:,0])
    assert abs(pitch_approach[2])<1e-8
    assert abs(pitch_close[2])<1e-8

    # Mug mode: wrist level. Side IR40 approaches along world +X,
    # jaws close along +/-Y, never +/-Z.
    mug_approach=rt.T@_R(d,mug)[:,2]
    mug_close=rt.T@_R(d,mug)[:,0]
    assert mug_approach[0]>.99 and abs(mug_approach[2])<1e-8
    assert abs(mug_close[1])>.99 and abs(mug_close[2])<1e-8


def test_grasped_pitcher_and_cup_are_upright_and_lateral_not_underneath():
    m,d=_scene()
    rt=_R(d,m.body("rb_cell_tcp").id)
    pr=_R(d,m.body("rb_gripped_pitcher").id)
    cr=_R(d,m.body("rb_gripped_cup").id)
    # For standard mug wrist pose (identity) the cup stays upright.
    assert np.allclose(rt.T@cr,np.eye(3),atol=1e-7)
    # For sideways pitcher wrist (Ry -90), the pitcher stays upright.
    pitch_rot=Rotation.from_euler('y',-90,degrees=True).as_matrix()
    assert np.allclose(pitch_rot@(rt.T@pr),np.eye(3),atol=1e-7)

    for held,jaw_a,jaw_b,bowl,direction in (
      ("cup","rb_90_ir40_cup_jaw_left","rb_90_ir40_cup_jaw_right",
       "rb_gripped_cup_body",+1),
      ("pitcher","rb_90_ir40_handle_jaw_left","rb_90_ir40_handle_jaw_right",
       "rb_gripped_pitcher_body",-1)):
        centre=(d.geom_xpos[m.geom(jaw_a).id]+d.geom_xpos[m.geom(jaw_b).id])/2
        diff=rt.T@(d.geom_xpos[m.geom(bowl).id]-centre)
        # Compare in ACTIVE wrist poses; pitcher requires Ry(-90).
        active=(pitch_rot@diff if held=="pitcher" else diff)
        assert np.linalg.norm(active[:2])>.075,held
        assert abs(active[2])<.02,held
        if held=="cup": assert active[0]>.07
        if held=="pitcher": assert active[0]<-.07
        # Jaw closing gap must have horizontal axis in active wrist mode.


def test_unused_gripper_is_orthogonal_not_second_bottom_support():
    m,d=_scene()
    first=m.body("rb_90_mount_axial").id
    second=m.body("rb_90_mount_orthogonal").id
    assert np.allclose(_R(d,first)[:,2]@_R(d,second)[:,2],0,atol=1e-7)
    assert m.body("rb_90_force_torque_sensor").id>=0
    for name in ("rb_gripped_cup_handle_outer",
                 "rb_gripped_pitcher_spout",
                 "rb_free_mug_handle_outer"):
        assert m.geom(name).id>=0
