"""Realistic small AprilTag geometry plus strict estimator-frame checks."""
import math
from pathlib import Path
import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from robobarista.compact_tags import (
    build_compact_tag_cell, PITCHER_MARKERS,CUP_MARKER,
    STATION_PLACEMENT,ACTIVE_FRACTION)
from robobarista.camera_joint_replay import _transform


def test_compact_apriltags_have_small_physical_sizes_and_mounts():
    path=build_compact_tag_cell(Path('vendor'))
    model=mujoco.MjModel.from_xml_path(str(path))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    for tid,size in [(71,.032),(72,.030),(23,.036),(70,.028)]:
        name={71:'cell_pitcher_top',72:'cell_pitcher_top_secondary',
              23:'cell_pitcher_side',70:'cell_plastic_cup'}[tid]
        fid=model.geom('rb_tagface_'+name).id
        assert np.isclose(model.geom_size[fid,0]*2,size,atol=1e-7)
        assert model.geom_contype[fid]==0
    for label,(_,_,_,side) in STATION_PLACEMENT.items():
        tag=model.geom('rb_tagface_'+label).id
        assert model.geom_size[tag,0]*2==pytest.approx(side)
        assert model.geom('rb_tag_backing_'+label).id>=0
        # No huge disconnected 30–70cm post in compact fixture!
        with pytest.raises(KeyError):
            model.geom('rb_tag_stem_'+label)
    for k in ('71','72','23','cup'):
        assert model.geom('rb_clip_post_'+k).id>=0


def test_compact_marker_relative_transforms_match_rendered_mujoco_body():
    path=build_compact_tag_cell(Path('vendor'))
    m=mujoco.MjModel.from_xml_path(str(path))
    d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    p_body=m.body('rb_milk_pitcher').id
    origin=d.xpos[p_body].copy()
    R=d.xmat[p_body].reshape(3,3)
    for tid,(loc,rpy,side) in PITCHER_MARKERS.items():
        name={71:'cell_pitcher_top',72:'cell_pitcher_top_secondary',
              23:'cell_pitcher_side'}[tid]
        face=m.geom('rb_tagface_'+name).id
        actual=d.geom_xpos[face]
        desired=origin+R@np.asarray(loc)
        assert np.allclose(actual,desired,atol=1e-6),(tid,actual,desired)
        R_marker=d.geom_xmat[face].reshape(3,3)
        expected=R@Rotation.from_euler('xyz',rpy).as_matrix()
        assert np.allclose(R_marker,expected,atol=2e-5)
        assert 0.025<=side<=.036
        assert .020<side*ACTIVE_FRACTION<.032
    # Side marker sits against pitcher wall, not 50mm off the cup.
    pitcher_side=d.geom_xpos[m.geom('rb_tagface_cell_pitcher_side').id]
    assert abs(pitcher_side[1]-origin[1])<.075
    assert abs(pitcher_side[1]-origin[1])>.048


def test_cup_marker_is_small_tab_attached_to_rim():
    path=build_compact_tag_cell(Path('vendor'))
    m=mujoco.MjModel.from_xml_path(str(path))
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    cup=d.xpos[m.body('rb_plastic_cup').id]
    fid=m.geom('rb_tagface_cell_plastic_cup').id
    pos=d.geom_xpos[fid]
    assert abs(np.linalg.norm(pos[:2]-cup[:2])-.058)<.0001
    assert pos[2]-cup[2]==pytest.approx(.105)


# Held-prop/film tests belong to the separate pending coffee-workflow PR #18.
# The approved main branch imports ONLY a standalone 90deg preview scene.


def test_optional_fourth_camera_compiles_separately_from_three_cam_rig():
    from robobarista.compact_tags import build_compact_tag_cell
    three=build_compact_tag_cell(Path('vendor'),camera_count=3)
    four=build_compact_tag_cell(Path('vendor'),camera_count=4)
    m3=mujoco.MjModel.from_xml_path(str(three))
    m4=mujoco.MjModel.from_xml_path(str(four))
    assert m4.ncam == m3.ncam + 1
    assert m4.camera('cell_overhead_aux').id >= 0
    with pytest.raises(KeyError):
        m3.camera('cell_overhead_aux')
    with pytest.raises(ValueError):
        build_compact_tag_cell(Path('vendor'),camera_count=5)
