"""Fail closed on original source cropping and preserve real timestamps."""
import pytest
from robobarista.real_demo_90_ik import crop_real_episode

def _episode():
    return [{"time_s":i*.05,"state":[0.]*14,"action":[0.]*14}
            for i in range(400)]

def test_real_time_windows_preserve_original_frames():
    full=_episode()
    clip=crop_real_episode(full,8.,12.)
    assert 79<=len(clip)<=81
    assert clip[0]["time_s"]==pytest.approx(0.)
    assert full[160]["time_s"]==pytest.approx(8.)

def test_refuse_wrong_interval_and_invalid_original_schema():
    full=_episode()
    with pytest.raises(ValueError,match="explicit"):
        crop_real_episode(full,12.,8.)
    with pytest.raises(ValueError,match="eight"):
        crop_real_episode(full,.1,.2)
    full[100]["state"]=[0]*12
    with pytest.raises(ValueError,match="14D"):
        crop_real_episode(full,8.,12.)
