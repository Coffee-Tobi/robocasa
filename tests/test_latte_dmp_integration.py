"""Tests for camera profile isolation and the optional DFKI DMP adapter."""
import sys
import types
import numpy as np
import pytest

from robobarista.latte_dmp import measured_spout_series, imitate_virtual_spout


def sample_camera(n=11):
    return [{"time_s": i * .2,
             "valid": True, "visible_views": 2,
             "position_m": [.35 + .004 * np.sin(i/3), -.30, .95],
             "rotation_xyzw": [0., 0., 0., 1.]}
            for i in range(n)]


def test_tip_series_from_cameras_not_ground_truth():
    rows = sample_camera()
    rows[0]["hidden_ground_truth"] = [9000, 9000, 9000]
    t, tip = measured_spout_series(rows, [-.07, 0., .12])
    assert t.shape == (11,) and tip.shape == (11, 3)
    assert tip[0, 0] == pytest.approx(.28)
    assert np.max(np.abs(tip)) < 10


def test_bad_camera_visibility_and_quaternion_rejected():
    rows = sample_camera()
    rows[5]["visible_views"] = 1
    with pytest.raises(ValueError, match="Two visible"):
        measured_spout_series(rows, [0., 0., 0.])
    rows = sample_camera()
    rows[3]["rotation_xyzw"] = [0., 0., 0., 0.]
    with pytest.raises(ValueError, match="Non-unit"):
        measured_spout_series(rows, [0., 0., 0.])


def test_real_dmp_api_is_used_but_never_robot_commands(monkeypatch):
    calls = []
    class FakeDMP:
        def __init__(self, **kwargs):
            calls.append(("init", kwargs))
            self.duration = kwargs["execution_time"]
            self.dt = kwargs["dt"]

        def imitate(self, t, y):
            calls.append(("imitate", (t, y)))
            self.begin = y[0]

        def configure(self, start_y, goal_y):
            calls.append(("configure", (start_y, goal_y)))
            self.goal = goal_y

        def open_loop(self, run_t):
            calls.append(("open_loop", run_t))
            tt = np.linspace(0, run_t, 11)
            yy = self.begin + (self.goal - self.begin) * (tt/run_t)[:, None]
            return tt, yy

    library = types.ModuleType("movement_primitives")
    dmp_mod = types.ModuleType("movement_primitives.dmp")
    dmp_mod.DMP = FakeDMP
    monkeypatch.setitem(sys.modules, "movement_primitives", library)
    monkeypatch.setitem(sys.modules, "movement_primitives.dmp", dmp_mod)
    result = imitate_virtual_spout(sample_camera(),
                                  virtual_tip_offset_m=[-.07, 0, .12],
                                  goal_offset_mm=[1., 0., 0.])
    assert result["status"] == "OFFLINE_POSITIONAL_DMP_ONLY"
    assert not result["hardware_execution_allowed"]
    assert not result["motor_control_or_collision_planning"]
    assert not result["orientation_learned"]
    assert [name for name, _ in calls] == ["init", "imitate", "configure", "open_loop"]
    assert calls[2][1][1][0] == pytest.approx(
        sample_camera()[-1]["position_m"][0] - .07 + .001)


def test_dmp_rejects_large_goal_shift_even_when_package_present(monkeypatch):
    class ImpossibleDMP:
        def __init__(self, **kwargs):
            pass
    library = types.ModuleType("movement_primitives")
    dmp_mod = types.ModuleType("movement_primitives.dmp")
    dmp_mod.DMP = ImpossibleDMP
    monkeypatch.setitem(sys.modules, "movement_primitives", library)
    monkeypatch.setitem(sys.modules, "movement_primitives.dmp", dmp_mod)
    with pytest.raises(ValueError, match="20 mm"):
        imitate_virtual_spout(sample_camera(), virtual_tip_offset_m=[0,0,0],
                              goal_offset_mm=[21,0,0])
