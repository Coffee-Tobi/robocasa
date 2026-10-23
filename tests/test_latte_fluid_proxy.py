"""Test illustration numerical behavior and prohibit physical accuracy claims."""
import numpy as np
import pytest
from robobarista.latte_fluid_proxy import (
    CoffeeSurfaceIllustration,validate_pose_source)


def test_proxy_is_finite_and_stays_inside_cup():
    cup=CoffeeSurfaceIllustration(80)
    for _ in range(12):
        field=cup.update([5.,-2.],.7)
        assert field.shape==(80,80)
        assert np.isfinite(field).all()
        assert np.min(field)>=0
        assert np.max(field)<=1
        assert np.all(field[~cup.mask]==0)
    assert np.max(cup.scalar)>.0
    assert cup.rgb().shape==(80,80,3)


def test_injected_flow_is_not_measured_or_guessed_from_video():
    cup=CoffeeSurfaceIllustration(64)
    initial=cup.scalar.copy()
    cup.update([0,0],0)
    assert np.allclose(cup.scalar,initial)
    with pytest.raises(ValueError):
        cup.update([0,0],1.3)
    with pytest.raises(ValueError):
        cup.update([float("nan"),0],.3)


def test_requires_bounded_source_poses():
    valid=[{"time_s":i*.1,"position_m":[.4,.2,.8]}
           for i in range(11)]
    ts,xyz=validate_pose_source(valid)
    assert xyz.shape==(11,3)
    valid[5]["time_s"]=valid[4]["time_s"]
    with pytest.raises(ValueError):
        validate_pose_source(valid)
