"""Physical-actuation disabled: unit tests for force/torque + lookahead adapter."""
import pytest
from robobarista.fairino_force_and_trajectory import (
    RecordingFairino,Wrench,LookaheadConfig,lookahead_sequence,get_wrench,
    portafilter_lock_monitor,steam_wand_side_contact,illustrative_contact_curve
)

def test_lookahead_uses_official_nine_argument_signature_without_robot_motion():
    robot=RecordingFairino()
    response=lookahead_sequence(robot,LookaheadConfig("approved_joint_log.txt"),
        uploaded_verified=True)
    assert response["api"]=="LoadTrajectoryLA"
    assert robot.calls==[("LoadTrajectoryLA",[
        "approved_joint_log.txt",1,0.5,0,0.5,35.,120.,750.,0])]
    with pytest.raises(RuntimeError):
        robot.MoveTrajectoryLA()
    with pytest.raises(RuntimeError):
        lookahead_sequence(object(),LookaheadConfig("approved_joint_log.txt"),True)
    with pytest.raises(ValueError):
        lookahead_sequence(RecordingFairino(),LookaheadConfig("a.txt"),False)

def test_portafilter_lock_does_not_confuse_torque_with_verified_lock():
    traces=[illustrative_contact_curve("insert_grouphead",j/30)[0] for j in range(31)]
    assert portafilter_lock_monitor(traces)["state"]=="RESISTANCE_SEEN_NOT_LOCK_PROVEN"
    assert portafilter_lock_monitor([Wrench(tz=.03)])["state"]=="NOT_CONFIRMED"
    assert portafilter_lock_monitor([Wrench(tz=2.)])["state"]=="STOP_OVERLOAD"

def test_steam_wand_lateral_guard():
    trace=[illustrative_contact_curve("steam",j/30)[0] for j in range(31)]
    assert steam_wand_side_contact(trace)["state"]=="LIGHT_CONTACT_DETECTED"
    assert steam_wand_side_contact([Wrench(fy=4)])["state"]=="STOP_EXCESS_LATERAL_FORCE"
    assert steam_wand_side_contact([Wrench()])["state"]=="NO_CONTACT_DETECTED"

def test_force_reader_checks_values_and_uses_official_sdk_name():
    mock=RecordingFairino([[1,2,3,.1,.2,.3]])
    assert get_wrench(mock).tz==pytest.approx(.3)
    assert mock.calls[0][0]=="FT_GetForceTorqueRCS"
    with pytest.raises(ValueError):
        Wrench.from_sdk([1,2])
