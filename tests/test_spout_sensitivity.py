"""Tip angle/lag physics: mathematical values must never be misreported as FAIRINO specs."""
import pytest
from robobarista.spout_sensitivity import (
    additional_tip_error_due_to_angle_mm,max_additional_angle_for_error_deg,
    additive_worst_case_bounds)

def test_160mm_lever_one_degree_about_2p8mm_error():
    assert additional_tip_error_due_to_angle_mm(.160,1)==pytest.approx(2.79249,abs=.002)
    assert max_additional_angle_for_error_deg(.160,.5)==pytest.approx(.17905,abs=.002)

def test_additive_bound_cannot_be_claimed_as_hardware_measurement():
    bound=additive_worst_case_bounds(.5342,extra_tool_angle_deg=.2,
        tool_lever_arm_m=.16,extra_lag_ms=50,
        tip_velocity_max_mm_s=30,additional_tool_compliance_mm=.2)
    assert bound['conservative_upper_bound_mm']>2.7
    assert bound['measured_hardware_performance'] is False

def test_invalid_negative_and_missing_lever_not_accepted():
    with pytest.raises(ValueError):
        additional_tip_error_due_to_angle_mm(0.,1)
    with pytest.raises(ValueError):
        additive_worst_case_bounds(.5,extra_lag_ms=-1)
