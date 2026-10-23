"""Geometry-only trace mapping; no ESPRESSO/MILK physics claimed."""
import json,pytest
from robobarista.latte_fluid_adapter import from_apriltag_capture, _rotate_xyzw

def test_quaternion_tip_rotation():
    assert _rotate_xyzw([0,0,0,1],(-.069,0,.136))==pytest.approx((-.069,0,.136))
    # Quaternion 180 degrees about Z flips spout X but not Z.
    assert _rotate_xyzw([0,0,1,0],(-.069,0,.136))==pytest.approx((.069,0,.136))

def test_real_tracking_not_truth_used(tmp_path):
    records=[{"frame":i,"time_s":i/3,"position_m":[.1+i*.002,0,1.2],
              "rotation_xyzw":[0,0,0,1],"valid":True,
              "visible_views":2} for i in range(15)]
    src=tmp_path/"camera.json";src.write_text(json.dumps(records))
    result=from_apriltag_capture(src,tmp_path/"trace.json")
    assert len(result["events"])==15
    assert result["source"].startswith("camera-only")
    assert result["flow_rate_from_camera"] is False
    assert result["simulated_material"].startswith("stylized")
    assert result["events"][1]["norm_x"]>result["events"][0]["norm_x"]

def test_reject_bad_multi_camera_pose(tmp_path):
    src=tmp_path/"camera.json"
    src.write_text(json.dumps([{"valid":False,"visible_views":1}] *3))
    with pytest.raises(ValueError):
        from_apriltag_capture(src,tmp_path/"trace.json")
