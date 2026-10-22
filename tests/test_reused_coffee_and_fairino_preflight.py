"""Reused RoboCasa fixture controls, FAIRINO read-only SDK gates, dataset export."""
import json,math
import pytest
from robobarista.robocasa_coffee_adapter import CoffeeMachineState
from robobarista.fairino_sdk_preflight import (
    read_only_snapshot,select_nearest_ik,query_vendor_ik_branches,
    hardware_readiness_gate)
from robobarista.latte_dataset_bridge import export_camera_capture


def test_adapter_detects_real_scene_cup_under_outlet():
    coffee=CoffeeMachineState()
    ready,status=coffee.check_cup((.440,.205,.882))
    assert ready,status
    assert status=="CUP_UNDER_NOZZLE"
    assert coffee.check_cup((.2,.1,.882))[1]=="CUP_MISALIGNED"
    assert coffee.check_cup((.44,.205,1.2))[1]=="CUP_RIM_WRONG_HEIGHT"


def test_brew_requires_lock_press_load_and_calibration():
    coffee=CoffeeMachineState()
    pos=(.440,.205,.882)
    demo=coffee.demo_brew_step(pos)
    assert demo["virtual_coffee_disc_may_be_drawn"] is True
    assert demo["actual_brewing_allowed"] is False
    assert coffee.command(pos,button_contact_confirmed=True)["brew_on"] is False
    assert coffee.command(pos,button_contact_confirmed=True,
                          portafilter_lock_confirmed=True)["brew_on"] is False
    assert coffee.command(pos,button_contact_confirmed=True,
                          portafilter_lock_confirmed=True,pressure_ok=True)["brew_on"] is False
    ok=coffee.command(pos,button_contact_confirmed=True,
                      portafilter_lock_confirmed=True,pressure_ok=True,
                      calibration_valid=True)
    assert ok["state"]=="BREWING_WITH_VERIFIED_FLAGS"
    assert coffee.brew_on
    assert coffee.command(pos,button_contact_confirmed=True,
                          synthetic_button=True)["state"]=="INVALID_CONTACT_PROVENANCE"


class MockReadOnly:
    def __init__(self,stop=False):
        self.calls=[]
        self.stop=stop
    def response(self,name,value):
        self.calls.append(name)
        return 0,value
    def GetActualJointPosDegree(self,flag=0):
        return self.response("GetActualJointPosDegree",[10,20,30,40,50,60])
    def GetActualTCPPose(self,flag=0):
        return self.response("GetActualTCPPose",[50,20,600,90,0,0])
    def GetActualJointSpeedsDegree(self,flag=0):
        return self.response("GetActualJointSpeedsDegree",[0,0,0,0,0,0])
    def FT_GetForceTorqueRCS(self):
        return self.response("FT_GetForceTorqueRCS",[0,0,0,0,0,0])
    def GetRobotMotionDone(self):
        return self.response("GetRobotMotionDone",1)
    def GetMotionQueueLength(self):
        return self.response("GetMotionQueueLength",0)
    def GetRobotEmergencyStopState(self):
        return self.response("GetRobotEmergencyStopState",int(self.stop))
    def GetSafetyStopState(self):
        return self.response("GetSafetyStopState",[0,0])
    def TCFToAllJoint(self,tcp_pose,tool,workpiece,exPos):
        self.calls.append("TCFToAllJoint")
        return 0,[[i*10]*6 for i in range(8)]


def test_read_only_vendor_state_fails_safe():
    mock=MockReadOnly()
    result=read_only_snapshot(mock)
    assert result["motion_done"]==1
    assert len(result["joint_deg"])==6
    assert result["hardware_provenance_verified"] is False
    assert result["motion_authorizable_from_this_snapshot_alone"] is False
    assert not result["stop_or_hold"]
    assert all(name.startswith(("Get","FT_")) for name in mock.calls)
    assert read_only_snapshot(MockReadOnly(True))["stop_or_hold"]


def test_vendor_eight_branch_ik_selection_nearest():
    mock=MockReadOnly()
    branch=query_vendor_ik_branches(
        mock,[200,100,600,0,0,0],1,1,[21]*6,
        [(-170,170)]*6)
    assert branch["vendor_branch_index"]==2
    assert branch["joint_deg"]==[20.]*6
    assert branch["collision_checked"] is False
    with pytest.raises(ValueError):
        select_nearest_ik([[200.]*6],[0]*6,[(-160,160)]*6)


def test_autonomous_readiness_not_inferred_from_virtual_precision():
    gate=hardware_readiness_gate()
    assert not gate["hardware_ready_for_autonomous_espresso_latte_art"]
    assert "tool_tip_calibrated" in gate["missing_evidence"]
    assert not gate["autonomous_hot_liquid_operation_approved"]


def test_latte_dataset_exports_only_cameras_not_hidden_truth(tmp_path):
    capture=tmp_path/"capture"
    capture.mkdir()
    observations=[{"time_s":i/3,"valid":True,"visible_views":3,
                   "position_m":[.05,0,.9+i*.001],
                   "rotation_xyzw":[0,0,0,1],
                   "tags_seen":[71]} for i in range(10)]
    gt=[{"time_s":i/3,"secret_original_xyz":[55,55,55]} for i in range(10)]
    (capture/"camera_measurements.json").write_text(json.dumps(observations))
    (capture/"ground_truth_evaluation_only.json").write_text(json.dumps(gt))
    exported=export_camera_capture(capture,tmp_path/"exported")
    assert exported["frames"]==10
    assert exported["average_hz"]==pytest.approx(3)
    assert exported["ground_truth_isolation"]
    lines=(tmp_path/"exported"/"camera_observations.jsonl").read_text()
    assert "secret_original_xyz" not in lines
    assert len(lines.splitlines())==10
    assert "secret_original_xyz" in (tmp_path/"exported"/"evaluation_truth_DO_NOT_TRAIN_ON.json").read_text()
