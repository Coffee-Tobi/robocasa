import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from robobarista.motion import (
    smoothstep,motion_sample,frame_times,world_spout,valid_pose
)


def test_smooth_motion_starts_and_ends_at_original_pose():
    origin=np.array([.67,-.14,.452])
    rotation=Rotation.from_euler("x",17,degrees=True).as_matrix()
    start=motion_sample(0,origin,rotation)
    end=motion_sample(1,origin,rotation)
    np.testing.assert_allclose(start.position_m,origin,atol=1e-10)
    np.testing.assert_allclose(end.position_m,origin,atol=1e-10)
    np.testing.assert_allclose(start.rotation,rotation,atol=1e-10)
    np.testing.assert_allclose(end.rotation,rotation,atol=1e-10)


def test_motion_has_smooth_phases_and_nonzero_tilt():
    origin=np.array([.67,-.14,.452])
    I=np.eye(3)
    samples=[motion_sample(i/100,origin,I) for i in range(101)]
    assert {'approach','tilt','mock_pour','untilt','retreat'} == {s.phase for s in samples}
    assert np.linalg.norm(samples[55].position_m-origin)>.15
    assert 25.9 < Rotation.from_matrix(samples[60].rotation).magnitude()*180/np.pi < 26.1
    for s in samples:
        assert np.linalg.det(s.rotation)==pytest.approx(1,abs=1e-10)
    max_delta=max(np.linalg.norm(b.position_m-a.position_m)
                  for a,b in zip(samples,samples[1:]))
    assert max_delta < .020  # 100-sample smooth scripted cycle, not a robot speed limit
    assert smoothstep(0)==0 and smoothstep(1)==1


def test_frame_timebase_and_spout_transform():
    t=list(frame_times(25,8))
    assert t[0]==(0,0,0)
    assert t[-1]==(24,3,1)
    assert len(t)==25
    np.testing.assert_allclose(world_spout([1,2,3],np.eye(3),[-.1,0,.2]),
                               [.9,2,3.2])
    with pytest.raises(ValueError):
        list(frame_times(1,8))
    with pytest.raises(ValueError):
        motion_sample(1.5,[0,0,0],np.eye(3))


def test_bad_pose_not_silently_accepted():
    assert not valid_pose(None,3)
    estimate={'position':np.zeros(3),'rotation':np.eye(3),'converged':True}
    assert valid_pose(estimate,3)
    assert not valid_pose(estimate,1)
    assert not valid_pose({**estimate,'position':[float('nan'),0,0]},3)
    assert not valid_pose({**estimate,'converged':False},3)
