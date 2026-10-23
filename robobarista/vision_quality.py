"""Separate technical camera-capture PASS from precise-spout readiness.

The camera error file reports virtual PITCHER pose errors, NOT measured
real-world milk-spout errors, real camera calibration, or FR5 servo accuracy.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path


def evaluate_camera_quality(summary, errors, *, max_position_mm=0.5,
                            required_station_refs=8):
    if not isinstance(errors, list) or len(errors) < 3:
        raise ValueError("At least three independent reference comparisons required")
    numbers = [float(row["camera_vs_truth_mm"]) for row in errors]
    if any(not math.isfinite(x) or x < 0 for x in numbers):
        raise ValueError("Invalid reference errors")
    frames = int(summary["frames"])
    seen = int(summary["camera_two_view_frames"])
    stations = int(summary["fixed_station_tags_recognized_in_initial_render"])
    total = int(summary["fixed_station_reference_count"])
    if frames != len(numbers) or seen > frames or stations > total:
        raise ValueError("Malformed/incomplete reference frame summary")
    x = sorted(numbers)
    p95 = x[math.ceil(.95 * len(x)) - 1]
    meets_precision = seen == frames and max(x) <= max_position_mm
    meets_stations = stations >= required_station_refs
    return {
        "capture_status": summary["status"],
        "valid_two_view_frames": seen,
        "frames": frames,
        "mean_virtual_pitcher_position_error_mm": sum(x) / len(x),
        "p95_virtual_pitcher_position_error_mm": p95,
        "max_virtual_pitcher_position_error_mm": max(x),
        "position_target_max_mm": max_position_mm,
        "virtual_position_target_met": meets_precision,
        "visible_reference_stations": stations,
        "total_reference_stations": total,
        "required_reference_stations": required_station_refs,
        "station_coverage_target_met": meets_stations,
        "ready_for_real_hardware": False,
        "physical_spout_tip_calibrated": False,
        "independently_measured_motor_error_mm": None,
        "precision_evidence": "MuJoCo RGB-D rendered-camera pitcher-body pose only",
        "note": "Functional camera PASS must never be reported as sub-mm real spout tracking."
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture", required=True, help="Directory with summary and held-out camera evaluation")
    ap.add_argument("--output", required=True)
    args = ap.parse_args()
    root = Path(args.capture)
    report = evaluate_camera_quality(
        json.loads((root / "summary.json").read_text()),
        json.loads((root / "camera_vs_truth_EVAL_ONLY.json").read_text()))
    dest = Path(args.output)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
