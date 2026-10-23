"""Test real LeRobot v3 adapter API semantics WITHOUT installing torch/weights."""
import json,sys,types
import numpy as np
from PIL import Image
import pytest
from robobarista.lerobot_native_latte import (
    load_virtual_episode,create_native_lerobot_dataset,CAMS)


def synthetic_capture(root,n=4,valid=True):
    root.mkdir()
    rows=[]
    for i in range(n):
        rows.append({"frame":i,"time_s":i/5,
             "valid":valid,"visible_views":3 if valid else 1,
             "position_m":[0.,-.03,.93+i*.0001],
             "rotation_xyzw":[0.,0.,0.,1.]})
        for camera in CAMS:
            d=root/"raw_cameras"/camera
            d.mkdir(parents=True,exist_ok=True)
            Image.fromarray(np.full((24,32,3),75+i,dtype=np.uint8)).save(
                d/f"frame_{i:04d}.png")
    (root/"camera_measurements.json").write_text(json.dumps(rows))
    # Hidden simulation truth should NEVER enter dataset generation.
    (root/"ground_truth_evaluation_only.json").write_text(
        json.dumps([{"SOURCE_SECRET":9999999}]*n))


def test_precise_measurements_and_all_three_synchronized_rgb(tmp_path):
    source=tmp_path/"c"
    synthetic_capture(source)
    rows,fps=load_virtual_episode(source)
    assert fps==5 and len(rows)==4
    assert rows[0]["state"].shape==(7,)
    for c in CAMS:
        assert rows[0]["image_paths"][c].is_file()


def test_missing_camera_or_one_view_fails_not_backfilled(tmp_path):
    source=tmp_path/"c"
    synthetic_capture(source,valid=False)
    with pytest.raises(ValueError):
        load_virtual_episode(source)
    synthetic_capture_again=tmp_path/"d"
    synthetic_capture(synthetic_capture_again)
    (synthetic_capture_again/"raw_cameras"/CAMS[0]/"frame_0001.png").unlink()
    with pytest.raises(FileNotFoundError):
        load_virtual_episode(synthetic_capture_again)


def test_native_writer_is_offline_no_hub_and_no_truth(tmp_path,monkeypatch):
    source=tmp_path/"c"
    synthetic_capture(source)
    called=[]
    class FakeDataset:
        @classmethod
        def create(cls,**kwargs):
            called.append(("create",kwargs))
            return cls()
        def add_frame(self,data):
            called.append(("frame",data))
        def save_episode(self):
            called.append(("save",None))
        def finalize(self):
            called.append(("finalize",None))
    mod=types.ModuleType("lerobot")
    datasets=types.ModuleType("lerobot.datasets")
    datasets.LeRobotDataset=FakeDataset
    monkeypatch.setitem(sys.modules,"lerobot",mod)
    monkeypatch.setitem(sys.modules,"lerobot.datasets",datasets)
    out=tmp_path/"output"
    result=create_native_lerobot_dataset(source,out)
    assert result["lerobot_native_dataset"]
    assert not result["hardware_joint_actions_present"]
    assert not result["pushed_to_hub"]
    assert called[0][1]["fps"]==5
    assert called[0][1]["robot_type"]=="virtual_milk_pitcher_pose_not_fr5"
    assert sum(e[0]=="frame" for e in called)==4
    assert called[-1][0]=="finalize"
    record=next(x[1] for x in called if x[0]=="frame")
    assert set(record)=={"observation.state","action","task",*[f"observation.images.{n}" for n in CAMS]}
    assert record["observation.images.cell_left"].shape==(24,32,3)
    assert "SOURCE_SECRET" not in str(record)
