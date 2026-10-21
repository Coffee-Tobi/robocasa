import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robobarista.robot_replay import (
    JOINTS, WRIST_PITCHER_OFFSET_M, rotation_error,
    pose_error_mm_deg, _contacts,
)


def test_arm_has_six_distinct_joints_and_fixed_grasp_fixture():
    assert len(JOINTS)==len(set(JOINTS))==6
    assert len(WRIST_PITCHER_OFFSET_M)==3
    assert 0.06 < np.linalg.norm(WRIST_PITCHER_OFFSET_M) < 0.30


def test_pose_error_units_and_rotation():
    position=np.array([.6,-.1,.5])
    p_err,r_err=pose_error_mm_deg(position,np.eye(3),position,np.eye(3))
    assert p_err==pytest.approx(0)
    assert r_err==pytest.approx(0)
    p_err,r_err=pose_error_mm_deg(
        position+np.array([0,0,.0025]),
        Rotation.from_euler('y',3,degrees=True).as_matrix(),
        position,np.eye(3))
    assert p_err==pytest.approx(2.5)
    assert r_err==pytest.approx(3,abs=1e-8)


def test_rotation_error_is_three_axis_orientation_not_euler_difference():
    target=Rotation.from_euler('xyz',[20,-10,40],degrees=True).as_matrix()
    current=Rotation.from_euler('xyz',[20,-10,40],degrees=True).as_matrix()
    np.testing.assert_allclose(rotation_error(target,current),[0,0,0],atol=1e-12)
