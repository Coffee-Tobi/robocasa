"""Swift Mini manufacturer-enveloped illustrative model; no false contact CAD."""
import numpy as np
import mujoco
import pytest

from robobarista.swift_mini_visual_asset import (
    HEIGHT_M, WIDTH_M, DEPTH_M, make_preview_scene, dimension_report,
)


def test_swift_mini_is_not_commercial_swift_and_has_correct_extents(tmp_path):
    report=dimension_report()
    assert report["manufacturer_envelope_m"] == {
        "width":.166,"depth":.322,"height":.449}
    assert "NOT the commercial Swift" in report["model"]
    assert not report["measured_bayonet_center"]
    assert not report["surface_mesh_from_manufacturer"]
    assert not report["tamper_mechanism_kinematics_modeled"]
    assert not report["ready_to_replace_active_workcell"]

    model=mujoco.MjModel.from_xml_path(
        str(make_preview_scene(tmp_path/"preview.xml")))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    body=model.body("rb_swift_mini_visual_reference").id
    assert model.body_jntnum[body]==0
    corners=np.asarray([
        data.site_xpos[model.site(f"rb_sm_envelope_corner_{i}").id].copy()
        for i in range(8)])
    assert np.ptp(corners[:,0])==pytest.approx(WIDTH_M,abs=1e-6)
    assert np.ptp(corners[:,1])==pytest.approx(DEPTH_M,abs=1e-6)
    assert np.ptp(corners[:,2])==pytest.approx(HEIGHT_M,abs=1e-6)
    assert model.ncam==3
    assert model.site("rb_swift_mini_bayonet_UNCALIBRATED").id>=0


def test_swift_mini_visual_elements_do_not_fake_collision_or_kinematics(tmp_path):
    model=mujoco.MjModel.from_xml_path(
        str(make_preview_scene(tmp_path/"preview.xml")))
    visual_ids=[
        i for i in range(model.ngeom)
        if (name:=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i))
        and name.startswith("rb_sm_") and name not in
        ("rb_sm_floor","rb_sm_countertop")
    ]
    assert len(visual_ids)>=16
    assert all(model.geom_contype[i]==0 and model.geom_conaffinity[i]==0
               for i in visual_ids)
    with pytest.raises(KeyError):
        model.joint("rb_grinder_tamper_slide")
    assert model.body("rb_swift_mini_visual_reference").id>=0
