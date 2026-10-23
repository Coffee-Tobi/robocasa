"""Compact fiducials must be metrically and geometrically consistent."""
import pytest
from scipy.spatial.transform import Rotation
from robobarista.tag_mounts import (
    profile,COMPACT_JUG,LEGACY_JUG,COMPACT_CUP,
    PRINTED_ACTIVE_FRACTION,active_edge_m)

def test_compact_object_tags_are_really_small():
    assert len(COMPACT_JUG)==3
    assert all(.030<=m.printed_outer_m<=.060 for m in COMPACT_JUG.values())
    assert max(m.printed_outer_m for m in COMPACT_JUG.values())<min(
        m.printed_outer_m for m in LEGACY_JUG.values())
    assert COMPACT_CUP.printed_outer_m<=.04
    assert abs(COMPACT_CUP.centre_m[1])<.060
    assert COMPACT_CUP.centre_m[2]<.110

def test_surface_mounts_consistent_with_real_pitcher_dimensions():
    near=COMPACT_JUG[23];far=COMPACT_JUG[72]
    assert near.centre_m[1]==pytest.approx(-.052)
    assert far.centre_m[1]==pytest.approx(.052)
    assert .062<near.centre_m[2]<.108
    assert near.euler_xyz_deg[0]==90
    assert far.euler_xyz_deg[0]==-90
    assert COMPACT_JUG[71].centre_m[0]==pytest.approx(.079)
    assert active_edge_m(near)==pytest.approx(.054*PRINTED_ACTIVE_FRACTION)

def test_profile_switch_rejects_typo(monkeypatch):
    monkeypatch.setenv("ROBOBARISTA_TAG_PROFILE","compact")
    assert profile()=="compact"
    with pytest.raises(ValueError):
        profile("large")
    monkeypatch.setenv("ROBOBARISTA_TAG_PROFILE","legacy")
    assert profile()=="legacy"
