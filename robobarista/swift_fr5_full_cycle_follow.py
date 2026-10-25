"""Full camera-free robot program: fixed stand -> Swift Mini -> fixed stand.

The *robot* is executed as a recorded collision-screened 6-axis FR5 MuJoCo
kinematic joint path (FK/IK). The video uses a separate OFFLINE mocap view:
eye follows portafilter in XYZ, while the camera QUATERNION stays IDENTICAL
for every frame (west orientation). No tracking cameras, image feedback or
dynamic replanning. Grasp/release = clearly labeled VISUAL ownership changes,
NOT IR40 finger contact or torque/force-control simulation.

Original Swift Mini portafilter receiver position, actual OEM force limits and
right-turn requirement remain uncalibrated. Never send these joints to hardware.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from .geometry import look_at_quat
from .robot_replay import robot_qpos_addresses
from .swift_fr5_pick_stand import (
    build_robot_holder_swift_scene, plan_one_arm_demo,
    _check_robot_kinematic_step, HOLDER_BASKET_XYZ, RECEIVER_XYZ,
    BASKET_SITE,
)

FOLLOW_OFFSET_WEST_M=np.array([-.50,-.20,.21])
FOLLOW_FOCUS_OFFSET_M=np.array([0.,-.065,-.055])
CYCLE_PHASES=(
    "APPROACH_HOLDER","ALIGN_HANDLE","GRASP_VISUAL_SWITCH",
    "CLOSE_IR40_VISUAL_ONLY","LIFT_FROM_STAND","TRANSFER_ABOVE_SWIFT",
    "ALIGN_BELOW_SWIFT","LIFT_INTO_RECEIVER","OPTIONAL_RIGHT_TWIST",
    "UNLOCK_LEFT","LOWER_FROM_RECEIVER","WITHDRAW_UNDER_SWIFT",
    "RETURN_TRANSFER","LOWER_INTO_HOLDER","RETURN_ALIGNED",
    "RELEASE_IR40_VISUAL_ONLY","RETREAT_FROM_HOLDER"
)
BACKWARD_NAMES={
    "OPTIONAL_RIGHT_TWIST":"UNLOCK_LEFT",
    "LIFT_INTO_RECEIVER":"LOWER_FROM_RECEIVER",
    "ALIGN_BELOW_SWIFT":"WITHDRAW_UNDER_SWIFT",
    "TRANSFER_ABOVE_SWIFT":"RETURN_TRANSFER",
    "LIFT_FROM_STAND":"LOWER_INTO_HOLDER",
    "GRASP_VISUAL_SWITCH":"RETURN_ALIGNED",
    "ALIGN_HANDLE":"RETREAT_FROM_HOLDER",
    "APPROACH_HOLDER":"RETREAT_FROM_HOLDER",
}


def _sample(old, *, phase=None, picked=None, transition=None):
    result={
        "q_rad":list(old["q_rad"]),
        "phase":phase or old["phase"],
        "picked":bool(old["picked"] if picked is None else picked),
        "basket_target_xyz_m":list(old["basket_target_xyz_m"]),
        "yaw_target_deg":float(old["yaw_target_deg"]),
    }
    if transition:result["grip_event"]=transition
    return result


def make_complete_cycle(forward,*,grip_pause_frames=8,release_pause_frames=8,
                        end_pause_frames=8):
    """Mirror the verified joint path; insert distinct grip/release events.

    Only a *visual* prop-state switch is allowed. Position is never moved
    by toggling geoms, and the single existing tool body remains unchanged.
    """
    if not forward:
        raise ValueError("Missing 6-axis forward data")
    labels=[r["phase"] for r in forward]
    if "GRASP_VISUAL_SWITCH" not in labels or "OPTIONAL_RIGHT_TWIST" not in labels:
        raise ValueError("Forward IK sequence not complete")
    if not forward[-1]["picked"]:
        raise ValueError("End forward must carry the portafilter")
    if grip_pause_frames<3 or release_pause_frames<3 or end_pause_frames<1:
        raise ValueError("Need enough samples to observe grasp and return")
    cycle=[]
    grip_added=False
    for r in forward:
        if r["phase"]=="GRASP_VISUAL_SWITCH":
            # Do not abruptly attach at t=0. Pause on location with the
            # portafilter *still in its original holder*.
            cycle.append(_sample(r,phase="GRASP_VISUAL_SWITCH",picked=False))
            if not grip_added:
                for i in range(grip_pause_frames):
                    cycle.append(_sample(r,phase="CLOSE_IR40_VISUAL_ONLY",
                        picked=i>=grip_pause_frames//2,
                        transition=("symbolic_grasp_at_fixed_holder" if
                                    i==grip_pause_frames//2 else None)))
                grip_added=True
            continue
        cycle.append(_sample(r))
    cycle.extend([_sample(forward[-1],phase="OPTIONAL_RIGHT_TWIST",picked=True)
                  for _ in range(7)])  # show inserted moment before reversing
    return_aligned=None
    released=False
    for r in reversed(forward):
        backward=BACKWARD_NAMES[r["phase"]]
        if backward=="RETURN_ALIGNED":
            cycle.append(_sample(r,phase="RETURN_ALIGNED",picked=True))
            if not released:
                return_aligned=r
                for i in range(release_pause_frames):
                    cycle.append(_sample(r,phase="RELEASE_IR40_VISUAL_ONLY",
                         picked=i<release_pause_frames//2,
                         transition=("symbolic_release_into_fixed_holder" if
                              i==release_pause_frames//2 else None)))
                released=True
        elif backward=="RETREAT_FROM_HOLDER":
            cycle.append(_sample(r,phase=backward,picked=False))
        else:
            cycle.append(_sample(r,phase=backward,picked=True))
    if not released or return_aligned is None:
        raise ValueError("Return path failed to reach holder")
    cycle.extend([_sample(cycle[-1],phase="RETREAT_FROM_HOLDER",picked=False)
                  for _ in range(end_pause_frames)])
    events=[x.get("grip_event") for x in cycle if x.get("grip_event")]
    if events!=["symbolic_grasp_at_fixed_holder",
               "symbolic_release_into_fixed_holder"]:
        raise AssertionError("Exactly one real-time ownership change each way")
    return cycle


def _geom_ids(model,prefix):
    return [
        i for i in range(model.ngeom)
        if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or "").startswith(prefix)
    ]


def _scene_with_video_only_follow_camera(scene_path, dest):
    """One mocap camera in visualization model, NOT in the planning model."""
    root=ET.parse(scene_path)
    world=root.getroot().find("worldbody")
    camera_rig=ET.SubElement(world,"body",name="rb_video_WEST_TRANSLATION_ONLY",
                             pos="0 0 0",mocap="true")
    ET.SubElement(camera_rig,"camera",name="rb_video_WEST_fixed_rotation",
                  pos="0 0 0",quat="1 0 0 0",fovy="58")
    dest=Path(dest)
    root.write(dest,encoding="unicode")
    return dest


def render_full_cycle(output,*,vendor_dir="vendor",heading_deg=-30.,
                      width=780,height=700,fps=15):
    out=Path(output)
    out.mkdir(parents=True,exist_ok=True)
    scene=build_robot_holder_swift_scene(Path(vendor_dir),
                                          grasp_heading_deg=heading_deg)
    planning_model=mujoco.MjModel.from_xml_path(str(scene))
    planning_data=mujoco.MjData(planning_model)
    plan=plan_one_arm_demo(planning_model,planning_data,
                           samples_per_segment=5,allow_right=True,
                           grasp_heading_deg=heading_deg)
    if plan["status"]!="KINEMATIC_FR5_COMPLETE_PREVIEW_ONLY":
        result={
            "status":"BLOCKED_NO_COMPLETE_CYCLE",
            "first_failure":plan["first_failure"],
            "source_status":plan["status"],"heading_deg":heading_deg,
        }
        (out/"summary.json").write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
        return result
    cycle=make_complete_cycle(plan["samples"])
    qids,limits=robot_qpos_addresses(planning_model)
    for r in cycle:
        _check_robot_kinematic_step(planning_model,planning_data,qids,
                                    np.asarray(r["q_rad"]),True)
    # Check ownership switching occurs only AT fixed holder, and that
    # the basket has returned precisely to original parked position.
    basket_geom=planning_model.geom(BASKET_SITE).id
    transitions=[]
    max_tilt_deg=0.
    for i,r in enumerate(cycle):
        planning_data.qpos[qids]=r["q_rad"]
        mujoco.mj_forward(planning_model,planning_data)
        basket_position=planning_data.geom_xpos[basket_geom].copy()
        basket_R=planning_data.geom_xmat[basket_geom].reshape(3,3)
        max_tilt_deg=max(max_tilt_deg,
                         float(np.degrees(np.arccos(np.clip(basket_R[2,2],-1.,1.)))))
        if r.get("grip_event"):
            dist=float(np.linalg.norm(basket_position-HOLDER_BASKET_XYZ))
            if dist>.003:
                raise AssertionError("Grip/release off-holder by %.3f mm"%(dist*1000))
            transitions.append({"event":r["grip_event"],"frame":i,
                                "basket_alignment_mm":dist*1000})
    if [t["event"] for t in transitions]!=[
        "symbolic_grasp_at_fixed_holder",
        "symbolic_release_into_fixed_holder"]:
        raise AssertionError("Ownership was not switched exactly once each")
    if max_tilt_deg>1.0:
        raise AssertionError("Basket lost upright orientation")

    render_scene=_scene_with_video_only_follow_camera(scene,
                                      Path(scene).with_name("rb_swift_full_cycle_WEST_RENDER_ONLY.xml"))
    movie_model=mujoco.MjModel.from_xml_path(str(render_scene))
    movie_data=mujoco.MjData(movie_model)
    rqids,_=robot_qpos_addresses(movie_model)
    if planning_model.ncam!=0 or movie_model.ncam!=1:
        raise AssertionError("Video camera contaminated the robot planner")
    mocap_id=int(movie_model.body_mocapid[movie_model.body(
                          "rb_video_WEST_TRANSLATION_ONLY").id])
    if mocap_id<0:raise RuntimeError("Missing render-only mocap camera")

    cam_first_eye=HOLDER_BASKET_XYZ+FOLLOW_OFFSET_WEST_M
    cam_fixed_quat=look_at_quat(cam_first_eye,
                    HOLDER_BASKET_XYZ+FOLLOW_FOCUS_OFFSET_M)
    # At each frame, the camera OFFSET is fixed and no aim/look-at
    # calculation is repeated. Its position simply follows the held
    # portafilter basket, or the original holder when empty.
    camera_positions=[]
    camera_rotations=[]
    parked_ids=_geom_ids(movie_model,"rb_pf_stand_rb_gripped")
    held_ids=_geom_ids(movie_model,"rb_gripped_portafilter")
    mbasket=movie_model.geom(BASKET_SITE).id
    renderer=mujoco.Renderer(movie_model,width=width,height=height)
    frames_dir=out/"frames"
    frames_dir.mkdir(exist_ok=True)
    eye=cam_first_eye.copy()
    try:
        for k,r in enumerate(cycle):
            movie_data.qpos[rqids]=r["q_rad"]
            mujoco.mj_forward(movie_model,movie_data)
            picked=bool(r["picked"])
            for g in parked_ids: movie_model.geom_rgba[g,3]=0.0 if picked else 1.0
            for g in held_ids:movie_model.geom_rgba[g,3]=1.0 if picked else 0.0
            basket=movie_data.geom_xpos[mbasket].copy()
            follow=basket if picked else HOLDER_BASKET_XYZ
            desired_eye=follow+FOLLOW_OFFSET_WEST_M
            # Translation only, with soft, controlled view movement to
            # keep the tool on screen without rotating or changing FOV.
            eye += .42*(desired_eye-eye)
            movie_data.mocap_pos[mocap_id]=eye
            movie_data.mocap_quat[mocap_id]=cam_fixed_quat
            mujoco.mj_forward(movie_model,movie_data)
            camera_positions.append(eye.copy())
            camera_rotations.append(movie_data.mocap_quat[mocap_id].copy())
            renderer.update_scene(movie_data,camera="rb_video_WEST_fixed_rotation")
            raw=Image.fromarray(renderer.render().copy())
            canvas=Image.new("RGB",(width,height+74),(18,28,39))
            canvas.paste(raw,(0,37))
            draw=ImageDraw.Draw(canvas)
            draw.text((12,9),"FR5 | WEST-FOLLOW: Position folgt, Winkel bleibt FEST",
                      fill=(229,241,248))
            label={
                "GRASP_VISUAL_SWITCH":"Siebtraeger noch im Halter",
                "CLOSE_IR40_VISUAL_ONLY":"Greifen (visuelles Modell)",
                "OPTIONAL_RIGHT_TWIST":"Einsetzen + 12 Grad rechts",
                "UNLOCK_LEFT":"Loesen: 12 Grad zurueck / links",
                "RELEASE_IR40_VISUAL_ONLY":"Ablegen + Oeffnen (visuelles Modell)",
                "RETREAT_FROM_HOLDER":"Siebtraeger bleibt im Halter"
            }.get(r["phase"],r["phase"])
            draw.text((12,height+45),f"{k+1:03d}/{len(cycle):03d}  {label}",
                      fill=(255,219,155))
            canvas.save(frames_dir/f"frame_{k:04d}.png")
    finally:
        renderer.close()
    cam_drift=float(np.max(np.linalg.norm(
        np.asarray(camera_rotations)-np.asarray(camera_rotations)[0],axis=1)))
    cam_travel=float(np.linalg.norm(camera_positions[-1]-camera_positions[0]))
    cam_extent=(np.max(camera_positions,axis=0)-np.min(camera_positions,axis=0)).tolist()
    if cam_drift>1e-10:
        raise AssertionError("Follow camera ROTATED unexpectedly")
    if max(cam_extent)<.08:
        raise AssertionError("Camera did not translate with the moving object")

    movie=out/"FR5_WEST_FOLLOW_FULL_HOLDER_TO_SWIFT_AND_BACK.mp4"
    if not shutil.which("ffmpeg"):
        raise RuntimeError("Cannot create MP4; ffmpeg not installed")
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(fps),
        "-i",str(frames_dir/"frame_%04d.png"),"-c:v","libx264",
        "-pix_fmt","yuv420p","-movflags","+faststart",str(movie)],check=True)
    if movie.stat().st_size<50000:raise RuntimeError("Video too small")

    report={
        "status":"VIRTUAL_FR5_FULL_CYCLE_COMPLETE",
        "movie":movie.name,"frame_count":len(cycle),
        "runtime_seconds":len(cycle)/fps,
        "robot_joints":6,
        "robot_tracking_cameras":planning_model.ncam,
        "render_only_camera_count":movie_model.ncam,
        "camera_follows_translation":True,
        "camera_quaternion_max_change":cam_drift,
        "camera_xyz_travel_m":cam_travel,
        "camera_xyz_extent_m":cam_extent,
        "camera_west_offset_from_basket_m":FOLLOW_OFFSET_WEST_M.tolist(),
        "camera_initial_eye_xyz_m":cam_first_eye.tolist(),
        "camera_fixed_quaternion_wxyz":np.asarray(cam_fixed_quat).tolist(),
        "holder_heading_deg":heading_deg,
        "max_basket_tilt_deg":max_tilt_deg,
        "right_turn_deg":12.,
        "unlock_left_turn_deg":-12.,
        "portafilter_in_stand_initially":True,
        "portafilter_in_stand_at_end":True,
        "ownership_transitions":transitions,
        "phase_sequence":list(dict.fromkeys(s["phase"] for s in cycle)),
        "no_actual_gripper_finger_dynamics":True,
        "no_actual_robot_force_control":True,
        "swift_mini_bayonet_not_measured":True,
        "milling_machine_contact_mesh_noncolliding":True,
        "left_turn_manufacturer_required":False,
        "movie_size_bytes":movie.stat().st_size,
    }
    (out/"summary.json").write_text(json.dumps(report,indent=2))
    (out/"robot_joints_offline_NOT_REAL_HARDWARE.json").write_text(json.dumps(cycle))
    print(json.dumps(report,indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="runs/fr5_complete_cycle_west")
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--grasp-heading",type=float,default=-30.)
    p.add_argument("--width",type=int,default=780)
    p.add_argument("--height",type=int,default=700)
    p.add_argument("--fps",type=int,default=15)
    a=p.parse_args()
    result=render_full_cycle(a.output,vendor_dir=a.vendor_dir,
         heading_deg=a.grasp_heading,width=a.width,height=a.height,fps=a.fps)
    if result["status"]!="VIRTUAL_FR5_FULL_CYCLE_COMPLETE":
        raise SystemExit(2)


if __name__=="__main__":
    main()
