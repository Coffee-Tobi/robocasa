"""Test actual data integration rules WITHOUT redistributing the upstream dataset.

The test rig below is intentionally SYNTHETIC and makes NO claim about the
original latte-pour-demos or calibrated Realman geometries.
"""
from __future__ import annotations
import json
import sys
import types
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robobarista.real_latte_dataset import (
    validate_realman_episode,extract_published_episode)
from robobarista.realman_fk_bridge import (
    parse_urdf_chain,forward_kinematics,reconstruct_pitches)
from robobarista.external_pose_replay import (
    validate_source_trajectory,align_imported_poses_to_fr5_start)


def fake_joint_rows(n=11):
    return [{"time_s":i*.2,
             "state":[.001*i,0,0,0,0,0,.5]+[0,0,0,0,0,0,.2],
             "action":[.001*i,0,0,0,0,0,.5]+[0,0,0,0,0,0,.2]}
            for i in range(n)]


def fake_urdf(path):
    parts=['<robot name="synthetic_six_joints"><link name="base"/>']
    for k in range(1,7):
        parts.append(f'<link name="link{k}"/>')
        parts.append(f'<joint name="joint{k}" type="revolute">'
                     f'<parent link="{ "base" if k==1 else "link"+str(k-1) }"/>'
                     f'<child link="link{k}"/>'
                     '<origin xyz="0 0 0.1" rpy="0 0 0"/>'
                     '<axis xyz="0 0 1"/></joint>')
    parts.append('</robot>')
    path.write_text("".join(parts))
    return path


def test_public_episode_joint_format_not_fake_tcp():
    rows=fake_joint_rows()
    report=validate_realman_episode(rows)
    assert report["frames"]==11
    assert report["flange_and_pitcher_pose_available"] is False
    assert report["state_layout_from_upstream_Hardware_Bridge_ROS2"].startswith("[left six joints")
    assert report["right_gripper_raw_first"] == pytest.approx(.2)
    rows[2]["state"]=[0]*12
    with pytest.raises(ValueError,match="14D"):
        validate_realman_episode(rows)
    rows=fake_joint_rows()
    rows[2]["time_s"]=rows[1]["time_s"]
    with pytest.raises(ValueError,match="Nonfinite or nonmonotonic"):
        validate_realman_episode(rows)


def test_urdf_fk_and_full_6d_orientation(tmp_path):
    urdf=fake_urdf(tmp_path/"arm.urdf")
    chain=parse_urdf_chain(urdf,"link6",[f"joint{i}" for i in range(1,7)])
    zero=forward_kinematics(chain,[0]*6)
    assert zero[:3,3]==pytest.approx([0,0,.6])
    q=[np.pi/2,0,0,0,0,0]
    T=forward_kinematics(chain,q)
    assert T[:3,3]==pytest.approx([0,0,.6])
    assert Rotation.from_matrix(T[:3,:3]).magnitude()==pytest.approx(np.pi/2)
    with pytest.raises(ValueError,match="order"):
        parse_urdf_chain(urdf,"link6",["joint6"]+[f"joint{i}" for i in range(1,6)])
    poses,report=reconstruct_pitches(
        fake_joint_rows(),chain,arm="left",joint_unit="rad",
        flange_to_pitcher=np.eye(4))
    assert len(poses)==11
    assert poses[-1]["rotation_xyzw"]!=poses[0]["rotation_xyzw"]
    # Right arm is explicitly indices [7:13], NOT [6:12] (index 6 = left gripper).
    right_poses,_=reconstruct_pitches(
        fake_joint_rows(),chain,arm="right",joint_unit="rad",
        flange_to_pitcher=np.eye(4))
    assert right_poses[0]["rotation_xyzw"]==pytest.approx([0,0,0,1])
    assert right_poses[-1]["rotation_xyzw"]==pytest.approx([0,0,0,1])
    assert not report["source_camera_6d_pitcher_measurement"]
    assert report["source_pitcher_pose_frames"]==11


def test_external_6d_rigid_retargeting_preserves_relative_motion():
    source=[{"time_s":i*.2,"position_m":[.5+i*.001,.2,.7],
             "rotation_xyzw":[0,0,0,1]} for i in range(11)]
    aligned,report=align_imported_poses_to_fr5_start(
        source,[0.,-.4,1.2],np.eye(3),11,5)
    assert len(aligned)==11
    assert aligned[0][0]==pytest.approx([0.,-.4,1.2])
    assert aligned[-1][0]-aligned[0][0]==pytest.approx([.01,0,0])
    assert report["relative_rotation_and_translation_preserved"]
    assert not report["absolute_source_machine_alignment_preserved"]
    assert not report["robot_dynamics_simulated"]


def test_oversized_source_and_nonunit_pose_fail_before_replay():
    source=[{"time_s":i*.2,"position_m":[i*.03,0,0],
             "rotation_xyzw":[0,0,0,1]} for i in range(11)]
    with pytest.raises(ValueError,match="POUR segment"):
        validate_source_trajectory(source)
    source[4]["rotation_xyzw"]=[0,0,0,0]
    with pytest.raises(ValueError,match="Nonunit"):
        validate_source_trajectory(source)


def test_upstream_lerobot_import_one_episode_no_video_is_strict(tmp_path,monkeypatch):
    class FakeDataset:
        def __init__(self,**kwargs):
            assert kwargs["episodes"]==[0]
            assert kwargs["download_videos"] is False
            self.meta=types.SimpleNamespace(info={"features":{
                "observation.images.top_camera":{"dtype":"video"}}})
        def select_columns(self,columns):
            assert set(columns)=={"timestamp","observation.state","action"}
            return [{"timestamp":r["time_s"],
                     "observation.state":r["state"],"action":r["action"]}
                    for r in fake_joint_rows()]
        def __getitem__(self,idx):
            raise AssertionError("Video decode should not run")
    lm=types.ModuleType("lerobot")
    datasets=types.ModuleType("lerobot.datasets")
    datasets.LeRobotDataset=FakeDataset
    monkeypatch.setitem(sys.modules,"lerobot",lm)
    monkeypatch.setitem(sys.modules,"lerobot.datasets",datasets)
    report=extract_published_episode(
        output_dir=tmp_path/"import",episode=0,with_video=False)
    assert report["frames"]==11
    assert "observation.images.top_camera" in report["camera_views_advertised"]
    assert not report["flange_and_pitcher_pose_available"]
    assert not report["data_license_verified"]
    assert not report["hardware_execution_authorized"]
    assert (tmp_path/"import"/"recorded_realman_joints.json").is_file()
