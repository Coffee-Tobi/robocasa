import numpy as np
from scipy.spatial.transform import Rotation
from robobarista.camera_joint_replay import (
    MARKERS,TAG_DEFINITIONS,SIDE_LENGTHS,_transform,
    _safe_q_segment,
)
from robobarista.workcell_constraints import payload_check


def test_two_fixture_tags_are_known_without_online_object_truth():
    assert 71 in MARKERS and 72 in MARKERS and 23 in MARKERS
    assert len(TAG_DEFINITIONS)==3
    for tid,(T,side) in MARKERS.items():
        assert T.shape==(4,4)
        assert 0.04<side<0.1
        assert TAG_DEFINITIONS[tid][0]=='pitcher'
        assert abs(float(np.linalg.det(T[:3,:3]))-1)<1e-6


def test_full_pitcher_payload_is_collision_blocked_by_machine():
    R=np.eye(3)
    inside=payload_check([.49,.48,1.05],R,tool='pitcher')
    assert inside
    assert any(x['machine']=='espresso' for x in inside)
    safe=payload_check([-.17,-.53,1.21],R,tool='pitcher')
    assert not safe


def test_known_tag_to_body_transform_is_fixed():
    t=TAG_DEFINITIONS[71][1]
    np.testing.assert_allclose(t[:3,3],[-.065,0,.135],atol=1e-9)
    np.testing.assert_allclose(t[:3,:3],np.eye(3),atol=1e-9)
