"""Offline FAIRINO SDK syntax guards are stricter than a catch-all mock."""
import pytest
from robobarista.fairino_native_spline import RecordingRobot


def test_recorded_spline_method_arguments():
    r=RecordingRobot()
    assert r.NewSplineStart(1, 200)==0
    assert r.NewSplinePoint(desc_pos=[100., 0., 300., 0., 0., 0.],
                            tool=0, user=0, lastFlag=1, blendR=.5)==0
    assert r.NewSplineEnd()==0
    assert [v[0] for v in r.calls]==[
        "NewSplineStart","NewSplinePoint","NewSplineEnd"]


def test_malformed_sdk_commands_fail_at_offline_adapter():
    r=RecordingRobot()
    with pytest.raises(TypeError):
        r.NewSplinePoint(foo="not vendor compatible")
    with pytest.raises(ValueError):
        r.NewSplinePoint(desc_pos=[0.]*5,tool=0,user=0,lastFlag=1)
    with pytest.raises(ValueError):
        r.NewSplinePoint(desc_pos=[0.]*6,tool=19,user=0,lastFlag=1)
    with pytest.raises(ValueError):
        r.MoveL(desc_pos=[0.]*6,tool=0,user=0,vel=300)
    with pytest.raises(ValueError):
        r.NewSplineStart(1,2)
