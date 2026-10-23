"""Meaningful quality gates, nominal MoveIt planning-scene adapter tests."""
import pytest

from robobarista.vision_quality import evaluate_camera_quality
from robobarista.moveit_scene_bridge import collision_objects, export_scene
from robobarista.workcell_constraints import KEEP_OUTS


def vision_data(n=13, error=.4, visible=8):
    summary = {
        "status": "PASS", "frames": n, "camera_two_view_frames": n,
        "fixed_station_tags_recognized_in_initial_render": visible,
        "fixed_station_reference_count": 8}
    return summary, [{"camera_vs_truth_mm": error} for _ in range(n)]


def test_camera_functional_pass_is_not_precision_or_station_pass():
    summary, rows = vision_data(error=.593, visible=4)
    report = evaluate_camera_quality(summary, rows)
    assert report["capture_status"] == "PASS"
    assert not report["virtual_position_target_met"]
    assert not report["station_coverage_target_met"]
    assert not report["ready_for_real_hardware"]
    assert not report["physical_spout_tip_calibrated"]


def test_camera_quality_meets_only_virtual_position_target():
    summary, rows = vision_data(error=.1)
    report = evaluate_camera_quality(summary, rows)
    assert report["virtual_position_target_met"]
    assert report["station_coverage_target_met"]
    assert report["independently_measured_motor_error_mm"] is None
    with pytest.raises(ValueError):
        evaluate_camera_quality(summary, [{"camera_vs_truth_mm": 0.1}])


def test_moveit_box_lengths_frames_and_non_actuation():
    objects = collision_objects("world", extra_clearance_m=.01)
    assert len(objects) == len(KEEP_OUTS)
    ids = {obj["id"] for obj in objects}
    assert len(ids) == len(objects)
    for name, keepout in KEEP_OUTS.items():
        obj = next(o for o in objects if o["id"] == "robobarista_keepout_" + name)
        assert obj["frame_id"] == "world"
        assert obj["dimensions_m"] == pytest.approx(
            [2*(extent + .01) for extent in keepout.half_extents_m])
        assert obj["position_m"] == pytest.approx(keepout.center_m)
        assert obj["orientation_xyzw"] == [0,0,0,1]
    report = export_scene()
    assert not report["hardware_actuation_allowed"]
    assert not report["real_physical_fixtures_calibrated"]


def test_moveit_export_rejects_bad_clearance():
    with pytest.raises(ValueError):
        collision_objects("world", extra_clearance_m=-.02)
    with pytest.raises(ValueError):
        collision_objects("world", extra_clearance_m=.1)
    with pytest.raises(ValueError):
        collision_objects("", extra_clearance_m=.0)
