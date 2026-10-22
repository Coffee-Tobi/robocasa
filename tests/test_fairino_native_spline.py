"""No-hardware tests for official FAIRINO movement API compatibility."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from robobarista.fairino_native_spline import (
    Calibration, RecordingRobot, pose_from_camera, make_plan,
    replay_commands, select_camera_samples
)


def fixture_camera():
    return [{"valid":True,"time_s":i/15.,
             "position_m":[-.17+i*.0006,-.53+np.sin(i*.25)*.0005,1.21],
             "rotation_xyzw":[0,0,0,1]}
            for i in range(31)]


def test_position_units_and_kinematic_offset_are_not_ignored():
    base=np.eye(4);base[:3,3]=[.25,.1,.0]
    offset=np.eye(4);offset[:3,3]=[0,.08,-.04]
    cal=Calibration(base,offset,tool_id=1,user_id=0)
    pose=pose_from_camera(fixture_camera()[0],cal)
    assert pose[:3]==pytest.approx([80,-350,1170])
    assert pose[3:]==pytest.approx([0,0,0])


def test_rotation_respects_fixed_xyz_convention():
    q=Rotation.from_euler("xyz",[10,-20,35],degrees=True).as_quat().tolist()
    rec={"position_m":[0,0,1],"rotation_xyzw":q}
    got=pose_from_camera(rec,Calibration(np.eye(4),np.eye(4),0,0))
    assert got[3:]==pytest.approx([10,-20,35])


def test_new_spline_calls_are_official_and_final_flag_is_set():
    cal=Calibration(np.eye(4),np.eye(4),0,0)
    plan=make_plan(fixture_camera(),cal,"new_spline",duration_s=20,waypoint_stride=3)
    assert plan["waypoints"]==11
    assert plan["average_time_ms"]==2000
    recorded=replay_commands(RecordingRobot(),plan)
    assert recorded[0]==("NewSplineStart",(1,2000),{})
    assert recorded[-1]==("NewSplineEnd",(),{})
    assert [c[2]["lastFlag"] for c in recorded[1:-1]]==[0]*10+[1]
    assert len(recorded)==13


def test_movel_blended_sections_nonblocking_then_final_blocking():
    plan=make_plan(fixture_camera(),Calibration(np.eye(4),np.eye(4),0,0),
                   "movel_blend",20,3,.5)
    calls=replay_commands(RecordingRobot(),plan)
    assert len(calls)==11
    assert all(c[2]["blendR"]==.5 for c in calls[:-1])
    assert calls[-1][2]["blendR"]==-1
    assert all(c[2]["blendMode"]==0 for c in calls)


def test_refuses_unconnected_hardware_and_unvalidated_camera():
    cal=Calibration(np.eye(4),np.eye(4),0,0)
    plan=make_plan(fixture_camera(),cal)
    with pytest.raises(RuntimeError,match="Real robot"):
        replay_commands(object(),plan)
    invalid=fixture_camera()
    invalid[5]["valid"]=False
    with pytest.raises(ValueError,match="valid"):
        make_plan(invalid,cal)
    with pytest.raises(ValueError):
        Calibration(np.zeros((4,4)),np.eye(4),0,0)


def test_camera_points_keep_original_end_timestamp():
    rows=select_camera_samples(fixture_camera(),waypoint_stride=4)
    assert rows[0]["time_s"]==0
    assert rows[-1]["time_s"]==2.
    assert len(rows)==9
