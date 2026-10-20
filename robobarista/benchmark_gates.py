"""Separate pitcher-precision acceptance from whole-scene health checks.

The pitcher is the motion-capture research target; the cup/canister/obstacle
are secondary health checks. NEVER turn a scene FAIL into PASS by silently
dropping failing objects. Report both gates with explicit reasons.
"""
from __future__ import annotations
import math


def _bad_number(row, key):
    value=row.get(key)
    try:
        return value is None or not math.isfinite(float(value))
    except (ValueError,TypeError):
        return True


def evaluate_side(rows, min_cameras=2, max_spout_mm=.5,
                  max_pitcher_deg=1., max_other_mm=5., max_other_deg=5.):
    if not rows:
        return dict(pitcher_status="FAIL", scene_status="FAIL",
                    pitcher_failures=[{"reason":"no measurements"}],
                    other_failures=[{"reason":"no measurements"}])
    trials={int(r['trial']) for r in rows}
    pitcher_rows=[r for r in rows if r.get('object')=='pitcher']
    failures={"pitcher":[],"other":[]}
    for row in rows:
        label=row['object']
        part="pitcher" if label=="pitcher" else "other"
        reasons=[]
        if row.get('failed',True):
            reasons.append("no valid 6D pose")
        if int(row.get('cameras_seen',0))<min_cameras:
            reasons.append(f"fewer than {min_cameras} usable cameras")
        if not row.get('converged',False):
            reasons.append("solver not converged")
        limits=(
            [('spout_tip_error_mm',max_spout_mm),('rotation_deg',max_pitcher_deg)]
            if part=="pitcher" else
            [('translation_mm',max_other_mm),('rotation_deg',max_other_deg)])
        for key,bound in limits:
            if _bad_number(row,key):
                reasons.append(f"{key} missing")
            elif float(row[key])>bound:
                reasons.append(f"{key}={float(row[key]):.4f} > {bound}")
        if reasons:
            failures[part].append({"trial":int(row['trial']),"object":label,
                                   "cameras":int(row.get('cameras_seen',0)),
                                   "reasons":reasons})
    if len(pitcher_rows)!=len(trials):
        failures["pitcher"].append({
            "reason":f"pitcher measured for {len(pitcher_rows)} of {len(trials)} trials"})
    pitcher_status="FAIL" if failures["pitcher"] else "PASS"
    scene_status="PASS" if pitcher_status=="PASS" and not failures["other"] else "FAIL"
    return dict(pitcher_status=pitcher_status,scene_status=scene_status,
                pitcher_failures=failures["pitcher"],other_failures=failures["other"])


def summarize_existing_run(variant_folder):
    import json
    from pathlib import Path
    folder=Path(variant_folder)
    summary=json.loads((folder/"summary.json").read_text())
    rows=json.loads((folder/"measurements.json").read_text())
    if summary.get("experiment")!="pitcher_side_tags":
        raise ValueError("Expected pitcher-side benchmark")
    gates=evaluate_side(
        rows,min_cameras=int(summary.get("required_cameras_per_object",2)),
        max_spout_mm=float(summary.get("allowed_spout_tip_error_mm",.5)),
        max_pitcher_deg=float(summary.get("allowed_pitcher_rotation_deg",1.)),
        max_other_mm=float(summary.get("allowed_translation_mm",5.)),
        max_other_deg=float(summary.get("allowed_rotation_deg",5.)))
    return summary,gates
