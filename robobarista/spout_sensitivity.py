"""Evaluate angle/latency and grasp uncertainty WITHOUT pretending they are FAIRINO data.

Use this with the validated `latte_three/robot_errors.json` or
`latte_three/tracking_errors.json` from a finished GitHub Action.
The recorded errors are from MuJoCo RGB-D cameras; perturbations added here
are hypothetical *additional real-world* effects, not measured motor specs.
"""
from __future__ import annotations
import argparse,json,math
from pathlib import Path


def additional_tip_error_due_to_angle_mm(lever_arm_m,angle_deg):
    if not (math.isfinite(lever_arm_m) and .02<=lever_arm_m<=.6):
        raise ValueError("Specify a plausible positive calibrated lever arm")
    if not math.isfinite(angle_deg) or not 0<=angle_deg<=45:
        raise ValueError("Angle must be in 0..45 degrees")
    return 2000*lever_arm_m*math.sin(math.radians(angle_deg)/2)


def max_additional_angle_for_error_deg(lever_arm_m,allowed_extra_mm):
    if lever_arm_m<=0 or allowed_extra_mm<0 or allowed_extra_mm>2000*lever_arm_m:
        raise ValueError("invalid geometric budget")
    return math.degrees(2*math.asin(allowed_extra_mm/(2000*lever_arm_m)))


def additive_worst_case_bounds(measured_camera_tip_error_max_mm,
                              *,extra_tool_angle_deg=0,
                              tool_lever_arm_m=.160,
                              extra_lag_ms=0,
                              tip_velocity_max_mm_s=0,
                              additional_tool_compliance_mm=0):
    """Triangle inequality bound, not statistical estimate of actual errors.

    measured_camera_tip_error_max_mm ALREADY includes its own camera
    orientation effect. extra_tool_angle_deg describes only NEW motor/grip
    angular errors; NEVER add the same measured orientation twice.
    """
    for x in (measured_camera_tip_error_max_mm,extra_lag_ms,
              tip_velocity_max_mm_s,additional_tool_compliance_mm):
        if not math.isfinite(x) or x<0:raise ValueError("nonnegative finite")
    additional_angle=additional_tip_error_due_to_angle_mm(
        tool_lever_arm_m,extra_tool_angle_deg)
    latency=extra_lag_ms*tip_velocity_max_mm_s/1000.
    overall=(measured_camera_tip_error_max_mm+additional_angle+
             latency+additional_tool_compliance_mm)
    return {"camera_original_tip_error_max_mm":measured_camera_tip_error_max_mm,
        "extra_angle_tip_error_bound_mm":round(additional_angle,6),
        "extra_lag_tip_error_bound_mm":round(latency,6),
        "extra_compliance_tip_error_bound_mm":additional_tool_compliance_mm,
        "conservative_upper_bound_mm":round(overall,6),
        "measured_hardware_performance":False,
        "interpretation":"Additive mathematical upper bound only; no probability guarantee"}


def evaluate_records(path):
    rows=json.loads(Path(path).read_text())
    if len(rows)<2 or any("error_3d_mm" not in r for r in rows):
        raise ValueError("Need measured camera tip-error JSON with timestamps")
    return {"n":len(rows),
            "virtual_tracking_mean_mm":sum(float(x["error_3d_mm"]) for x in rows)/len(rows),
            "virtual_tracking_max_mm":max(float(x["error_3d_mm"]) for x in rows),
            "hardware_measured":False}


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--camera-errors-json",required=True)
    args=parser.parse_args()
    report=evaluate_records(args.camera_errors_json)
    print(json.dumps({
        "observed":report,
        "angle_1_deg_at_160mm_mm":additional_tip_error_due_to_angle_mm(.160,1.),
        "angular_requirement_at_160mm_for_0p5mm_deg":max_additional_angle_for_error_deg(.160,.5),
        "worst_case_50ms_0p2deg_0p2mmflex":additive_worst_case_bounds(
            report["virtual_tracking_max_mm"],extra_tool_angle_deg=.2,
            tool_lever_arm_m=.160,extra_lag_ms=50,
            tip_velocity_max_mm_s=30,additional_tool_compliance_mm=.2)
    },indent=2))
