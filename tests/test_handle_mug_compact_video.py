"""Regressions: mug MUST be visually gripped by its handle, tags stay small.

These are MuJoCo geometry inspections (not proof of fingertip contact).
The takeaway version stays as a separate supported variation.
"""
from pathlib import Path
import numpy as np
import mujoco
import pytest
from robobarista.coffee_workflow import _story_scene


def _model(style):
    return mujoco.MjModel.from_xml_path(str(
        _story_scene(Path("vendor"),marker_profile="compact",cup_style=style)))


def test_ceramic_mug_has_its_own_handle_and_gripper_jaws_target_it():
    m=_model("handle_mug")
    assert m.geom("rb_free_mug_handle_upper").id >= 0
    assert m.geom("rb_free_mug_handle_lower").id >= 0
    assert m.geom("rb_free_mug_handle_outer").id >= 0
    assert m.geom("rb_gripped_cup_handle_outer").id >= 0
    assert m.geom("rb_gripped_cup_handle_upper").id >= 0
    assert m.geom("rb_gripped_cup_handle_lower").id >= 0
    d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    gripper=m.body("rb_cup_tool_IR40_50").id
    held=m.body("rb_gripped_cup").id
    tool_R=d.xmat[gripper].reshape(3,3)
    tool_p=d.xpos[gripper]
    body_geom=m.geom("rb_gripped_cup_body").id
    body_local=tool_R.T@(d.geom_xpos[body_geom]-tool_p)
    # Vessel offset >10 cm past the jaws; gripper DOES NOT grip base.
    assert body_local[1] > .10
    jaw1=m.geom("rb_cup_jaw_-1").id
    jaw2=m.geom("rb_cup_jaw_1").id
    jawpos=[tool_R.T@(d.geom_xpos[x]-tool_p) for x in (jaw1,jaw2)]
    assert abs(jawpos[0][0]) < .020
    assert abs(jawpos[1][0]) < .020
    assert max(abs(x[1]) for x in jawpos) < .030
    assert np.linalg.norm(d.geom_xpos[jaw1]-d.geom_xpos[body_geom])>.08
    assert np.linalg.norm(d.geom_xpos[jaw2]-d.geom_xpos[body_geom])>.08
    # Small attached AprilTag, not 82-105 mm original marker.
    tag=m.geom("rb_tagface_held_cup").id
    assert 2*m.geom_size[tag,0]==pytest.approx(.028)


def test_takeaway_variant_still_has_body_jaws_and_no_fake_mug_handle():
    m=_model("takeaway")
    with pytest.raises(KeyError):
        m.geom("rb_gripped_cup_handle_upper")
    with pytest.raises(KeyError):
        m.geom("rb_free_mug_handle_upper")
    d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    unit=m.body("rb_cup_tool_IR40_50").id
    held=m.geom("rb_gripped_cup_body").id
    rel=d.xmat[unit].reshape(3,3).T@(d.geom_xpos[held]-d.xpos[unit])
    assert abs(rel[1])<.003


def test_invalid_gripping_modes_rejected():
    from robobarista.dual_ir40_concept import add_visual_dual_ir40_50
    import xml.etree.ElementTree as ET
    with pytest.raises(ValueError):
        add_visual_dual_ir40_50(ET.Element("body"),cup_style="underside-grip")
    with pytest.raises(ValueError):
        _story_scene(Path("vendor"),cup_style="underside-grip")


def test_compact_mug_shares_tag_reference_with_scene_and_keeps_unique_id():
    m=_model("handle_mug")
    cup=m.geom("rb_tagface_cell_plastic_cup").id
    held=m.geom("rb_tagface_held_cup").id
    assert m.geom_size[cup,0]*2==pytest.approx(.028)
    assert m.geom_size[held,0]*2==pytest.approx(.028)
    assert m.geom("rb_tagface_held_pitcher").id>=0
    assert m.geom("rb_tagface_held_portafilter").id>=0
