"""Exploratory REAL RM65 episode -> approved FR5 90deg IR40 MuJoCo IK.

The source is actual recorded joint positions, NOT measured source pitcher
geometry. FK assumes an explicitly chosen manufacturer URDF, arm and units.
Identity RM65 flange-to-pitcher is allowed only as a clearly marked MOCK.
Never sends a command to hardware, ROS2 or FAIRINO; *failed* feasibility is
a legitimate result. Approved side-grasp from main is always used.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import shutil

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .real_latte_dataset import validate_realman_episode
from .realman_fk_bridge import parse_urdf_chain, reconstruct_pitches
from .current_90_tool_frames import (
    model_90_degree_tool_frames, offline_virtual_pitcher_to_wrist,
)
from .coffee_workflow import _story_scene, _alpha
from .camera_joint_replay import _find_start
from .side_grasp_preview import find_side_approach
from .robot_replay import compute_ik, robot_qpos_addresses
from .safe_staging import _valid_state, _check_route
from .workcell_constraints import payload_check, KEEP_OUTS, distance_to_box

# These *extra* envelopes only approximate collision-disabled, inactive tool
# parts in the approved MuJoCo scene. They are NOT manufacturer collision CAD.
INACTIVE_TOOL_PARTS = (
    ("rb_90_ir40_cup_jaw_left", .014),
    ("rb_90_ir40_cup_jaw_right", .014),
    ("rb_90_ir40_cup", .036),
)


def crop_real_episode(rows, start_s, end_s):
    validate_realman_episode(rows)
    if not (0 <= start_s < end_s):
        raise ValueError("Provide an explicit source time window; original video has not confirmed pour-only interval")
    take = [r for r in rows if start_s <= float(r["time_s"]) <= end_s]
    if len(take) < 8:
        raise ValueError("Fewer than eight source records in requested interval")
    t0 = float(take[0]["time_s"])
    take = [{**r, "time_s": float(r["time_s"]) - t0} for r in take]
    validate_realman_episode(take)
    return take


def _additional_keepout(model, data):
    """Check actual held pitcher body plus inactive gripper proxy near boxes."""
    held = model.body("rb_gripped_pitcher").id
    p = np.asarray(data.xpos[held]).copy()
    R = data.xmat[held].reshape(3, 3)
    problems = payload_check(p, R, "pitcher")
    if problems:
        return False, "held_pitcher_keepout:" + problems[0]["machine"]
    for part, radius in INACTIVE_TOOL_PARTS:
        point = np.asarray(data.geom_xpos[model.geom(part).id]
                           if "jaw" in part else data.xpos[model.body(part).id])
        for key, box in KEEP_OUTS.items():
            if distance_to_box(point, box) < radius + .03:
                return False, "inactive_90_tool_keepout:" + key
    return True, "OK"


def _swept_segment(model, data, qids, limits, a, b):
    """Swept conservative approximate proxy at <=.018rad joint steps."""
    # Existing safety routine checks robot contact pairs and current nominal
    # tool proxy; here we add held-pitcher and inactive IR40 proxy checks.
    ok, reason, count = _check_route(model, data, qids, limits, a, b, "pitcher")
    if not ok:
        return False, reason, count
    for k in range(count + 1):
        q = a + (b - a) * (k / count)
        data.qpos[qids] = q
        mujoco.mj_forward(model, data)
        ok, reason = _additional_keepout(model, data)
        if not ok:
            return False, reason, count
    return True, "OK", count


def attempt_replay(*, input_joints, urdf, output, arm, unit,
                   start_s, end_s, frames=13, fps=3, seed=14,
                   render_virtual_preview=False):
    """Records all failures as clear JSON; no fake success or skipped poses."""
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    report = {
        "status": "NOT_RUN",
        "episode_source": "actual original RealMan numerical samples",
        "arm_selection": arm,
        "joint_units_ASSUMED": unit,
        "rm65_variant_confirmed_for_dataset": False,
        "actual_pitcher_mount_calibrated": False,
        "original_camera_pose_used": False,
        "pour_segment_visually_confirmed": False,
        "source_window_s": [start_s, end_s],
        "fr5_approved_90deg_axial_ir40": True,
        "physical_grasp_simulated": False,
        "source_flange_to_pitcher": "IDENTITY MOCK, NOT REAL TOOL",
        "actual_milk_flow_simulated": False,
        "collision_geometry_physically_complete": False,
        "real_hardware_motion_authorized": False,
        "note": "Offline hypothesis test only; world-frame source alignments lost.",
    }
    try:
        rows = crop_real_episode(json.loads(Path(input_joints).read_text()), start_s, end_s)
        chain = parse_urdf_chain(urdf, "link_6",
                                [f"joint_{i}" for i in range(1, 7)])
        poses, source_report = reconstruct_pitches(
            rows, chain, arm=arm, joint_unit=unit,
            flange_to_pitcher=np.eye(4), tool_calibrated=False)
        report["real_samples_in_window"] = len(rows)
        report["source_fk_excursion_mm"] = source_report["source_pitcher_translation_excursion_mm"]
        model = mujoco.MjModel.from_xml_path(str(_story_scene(Path("vendor"),
                                                              cup_style="handle_mug")))
        data = mujoco.MjData(model)
        qids, limits = robot_qpos_addresses(model)
        home, _ = _find_start(model, data, qids, limits, seed)
        _, start_q, _, _, approach, _ = find_side_approach(
            model, data, qids, limits, home, "pitcher")
        data.qpos[qids] = start_q
        mujoco.mj_forward(model, data)
        frames_info = model_90_degree_tool_frames(model, data)
        trans = offline_virtual_pitcher_to_wrist(
            poses, world_tcp_start=frames_info["world_tcp"],
            tcp_to_pitcher=frames_info["tcp_to_pitcher"],
            tcp_to_visual_spout=frames_info["tcp_to_visual_spout"],
            frames=frames, fps=fps)
        report["side_approach_mm"] = float(1000 * approach)
        report["source_duration_s"] = trans["provenance"]["original_source_duration_s"]
        report["replay_duration_s"] = trans["provenance"]["new_virtual_duration_s"]
        report["requested_frames"] = frames
        wrist = np.asarray(trans["wrist_targets_se3_NOT_EXECUTABLE"])
        target_spouts = np.asarray(trans["virtual_spout_xyz_m"])
        prev = np.asarray(start_q, float).copy()
        accepted = []
        actual_spouts = []
        errors_pos_mm = []
        errors_orientation_deg = []
        failures = []
        tcp_id = model.body("rb_cell_tcp").id
        spout_id = model.geom("rb_gripped_pitcher_spout").id
        for i, T in enumerate(wrist):
            # Initial source frame is anchored to approved side-grasp.
            # Independently solve *all* subsequent FR5 6-joint IK frames.
            if i == 0:
                q = prev.copy()
            else:
                result = compute_ik(
                    model, data, tcp_id, qids, limits,
                    T[:3, 3], T[:3, :3], prev, max_evals=130)
                if not result["solved"]:
                    failures.append({"frame": i, "type": "ik_failed",
                                     "position_error_mm": result.get("position_error_mm")})
                    break
                q = np.asarray(result["q"], float)
            valid, reason = _valid_state(model, data, qids, limits, q, "pitcher")
            if not valid:
                failures.append({"frame": i, "type": reason})
                break
            safe, reason, checks = _swept_segment(
                model, data, qids, limits, prev, q)
            if not safe:
                failures.append({"frame": i, "type": reason,
                                 "joint_substeps_checked": checks})
                break
            data.qpos[qids] = q
            mujoco.mj_forward(model, data)
            observed_p = np.asarray(data.xpos[tcp_id]).copy()
            observed_R = np.asarray(data.xmat[tcp_id]).reshape(3,3).copy()
            pos_error = 1000 * float(np.linalg.norm(observed_p-T[:3,3]))
            ang_error = float(np.rad2deg(Rotation.from_matrix(
                T[:3,:3] @ observed_R.T).magnitude()))
            spout = np.asarray(data.geom_xpos[spout_id]).copy()
            accepted.append(q.tolist())
            actual_spouts.append(spout.tolist())
            errors_pos_mm.append(pos_error)
            errors_orientation_deg.append(ang_error)
            prev = q.copy()
        report.update({
            "status": "GEOMETRIC_IK_PASS_NOT_HARDWARE" if not failures and len(accepted)==frames else "STOPPED_UNSAFE_OR_UNREACHABLE",
            "accepted_frames": len(accepted),
            "first_failure": failures[0] if failures else None,
            "max_tcp_position_error_mm": max(errors_pos_mm) if errors_pos_mm else None,
            "max_tcp_orientation_error_deg": max(errors_orientation_deg) if errors_orientation_deg else None,
            "max_visual_spout_error_mm": float(np.max(np.linalg.norm(
                np.asarray(actual_spouts)-target_spouts[:len(actual_spouts)],
                axis=1))*1000) if actual_spouts else None,
            "kinematic_only": True,
            "small_tag_camera_reconstruction_performed": False,
            "physical_spout_tip_error_mm": None,
            "inactive_tool_proxy_collision_checked": True,
            "swept_arm_links_contact_screened": True,
            "hardware_controller_tested": False,
        })
        if render_virtual_preview and report["status"].startswith("GEOMETRIC_IK_PASS"):
            # Render our OWN MuJoCo geometry only. Source videos and original
            # copyrighted source joints are NEVER included in the artifact.
            from PIL import Image, ImageDraw
            from .cell_demo import _set_free_pose
            if not shutil.which("ffmpeg"):
                raise RuntimeError("ffmpeg is required for virtual preview")
            _set_free_pose(model, data, "rb_pitcher_free", [-10,-10,-10], 0)
            _alpha(model, "pitcher")
            png_dir = out / "virtual_fr5_preview_frames"
            png_dir.mkdir(exist_ok=True)
            renderer = mujoco.Renderer(model, width=640, height=480)
            try:
                for frame, q in enumerate(accepted):
                    data.qpos[qids] = q
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera="cell_overview")
                    view=Image.fromarray(renderer.render().copy())
                    canvas=Image.new("RGB",(640,530),(25,30,36))
                    canvas.paste(view,(0,40))
                    draw=ImageDraw.Draw(canvas)
                    draw.text((12,10),"RECORDED RM65 "+arm.upper()+" / UNCALIBRATED TOOL MOCK",fill=(250,220,170))
                    draw.text((12,511),"FR5 90deg MuJoCo GEOMETRY ONLY - no milk or servo",fill=(255,235,188))
                    canvas.save(png_dir/f"frame_{frame:04d}.png")
            finally:
                renderer.close()
            file=out/"EXPLORATORY_virtual_FR5_not_real_pour.mp4"
            subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(fps),
               "-i",str(png_dir/"frame_%04d.png"),"-c:v","libx264",
               "-pix_fmt","yuv420p",str(file)],check=True)
            report["virtual_preview_video"] = file.name
            report["video_is_original_demo"] = False
            report["video_is_real_milk_pour"] = False
        (out/"source_pose_provenance_EXPLORATORY.json").write_text(
            json.dumps(source_report, indent=2))
        (out/"IK_joint_path_ONLY_IF_COMPLETE.json").write_text(
            json.dumps(accepted if report["status"].startswith("GEOMETRIC_IK_PASS") else [],
                       indent=2))
    except Exception as exc:
        report["status"] = "STOPPED_INVALID_SOURCE_OR_SCENE"
        report["error_type"] = type(exc).__name__
        report["error"] = str(exc)
    (out/"report.json").write_text(json.dumps(report, indent=2))
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input-joints", required=True)
    p.add_argument("--urdf", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--arm", choices=("left", "right"), required=True)
    p.add_argument("--joint-unit", choices=("rad", "deg"), required=True)
    p.add_argument("--start-s", type=float, required=True)
    p.add_argument("--end-s", type=float, required=True)
    p.add_argument("--frames", type=int, default=13)
    p.add_argument("--fps", type=int, default=3)
    p.add_argument("--render-virtual-preview", action="store_true")
    a = p.parse_args()
    report = attempt_replay(
        input_joints=a.input_joints, urdf=a.urdf, output=a.output,
        arm=a.arm, unit=a.joint_unit, start_s=a.start_s,
        end_s=a.end_s, frames=a.frames, fps=a.fps,
        render_virtual_preview=a.render_virtual_preview)
    print(json.dumps(report, indent=2))
    if not report["status"].startswith("GEOMETRIC_IK_PASS"):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
