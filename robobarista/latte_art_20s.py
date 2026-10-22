"""Twenty-second, 15-FPS conceptual latte-art demonstration with IR40 handle grasp.

The cup/pitcher and grasp adapters are kinematically attached under IR40
fingers. This is NOT a force-closure test, real milk flow, or motor benchmark.
The five phases imitate SMALL artistic pouring gestures in collision-screened
free-space predock, not close-to-surface verified liquid operations.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import subprocess
import shutil

import mujoco
import numpy as np
from PIL import Image,ImageDraw

from .coffee_workflow import _story_scene,_alpha,_valid_state
from .safe_staging import plan_staging
from .camera_joint_replay import _find_start
from .robot_replay import robot_qpos_addresses
from .cell_demo import _set_free_pose
from .workcell_constraints import DOCKS
from .demo import render_pair
from .follow_camera import ArmFollowCamera,CAMERA_NAME
from .geometry import look_at_quat


DURATION_S=20.0
FPS=15
STAGES=(("Approach",0.,3.),("Base fill",3.,7.),
        ("Fine wiggle",7.,14.),("Heart shape",14.,17.),
        ("Lift and cut",17.,20.))


def smooth(t,a,b):
    x=np.clip((t-a)/max(b-a,1e-6),0.,1.)
    return float(x*x*(3-2*x))


def gesture(t):
    """Joint-angle offsets; actual spout displacement measured from MuJoCo."""
    q=np.zeros(6)
    # Small demonstration! The arm does NOT teleport between phases.
    base=smooth(t,0,3)*(1-smooth(t,17,20))
    q[4]=-0.0065*base  # about 0.37 degree pitcher tilt
    q[0]=0.0016*(smooth(t,0,4)-smooth(t,17,20))
    if 3<=t<7:
        e=np.sin(np.pi*(t-3)/4)**2
        q[0]+=0.0020*np.sin(2*np.pi*.5*(t-3))*e
        q[5]+=0.0025*np.sin(2*np.pi*.5*(t-3))*e
    if 7<=t<14:
        e=np.sin(np.pi*(t-7)/7)**2
        q[0]+=0.0030*np.sin(2*np.pi*1.2*(t-7))*e
        q[5]+=0.0020*np.sin(2*np.pi*1.2*(t-7))*e
    if 14<=t<17:
        e=np.sin(np.pi*(t-14)/3)**2
        q[0]+=0.0025*np.sin(2*np.pi*.65*(t-14))*e
        q[5]-=0.0040*e
    if 17<=t:
        q[4]+=0.008*smooth(t,17,20)
        q[0]+=0.002*smooth(t,17,20)
    return q


def phase_name(t):
    for name,a,b in STAGES:
        if a<=t<b:return name
    return STAGES[-1][0]


def preflight(model,data,qids,limits,origin,frames):
    # All generated poses MUST be verified. Scale tiny wiggle if needed.
    for scale in (1.,.7,.4,.2,.1):
        paths=origin[None,:]+scale*frames
        errors=[]
        for k,q in enumerate(paths):
            okay,reason=_valid_state(model,data,qids,limits,q,"pitcher")
            if not okay:
                errors.append((k,reason))
                break
        if not errors:
            return paths,scale
    raise RuntimeError("No collision-screened small gesture possible: "+str(errors))


def run(args):
    if args.duration!=20:
        raise ValueError("This demonstrator is designed for exactly 20 seconds")
    if not (5<=args.fps<=30):
        raise ValueError("FPS must be between 5 and 30")
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    scene=_story_scene(Path(args.vendor_dir))
    # Independent mocap camera for a detailed view of jaws and handle adapter.
    # It never enters the calibrated 3-camera AprilTag measurement pipeline.
    tree=ET.parse(scene)
    rig=ET.SubElement(tree.getroot().find("worldbody"),"body",
                      name="rb_grip_detail_rig",mocap="true",
                      pos="0 -0.8 1.7")
    ET.SubElement(rig,"camera",name="rb_grip_detail",fovy="51",
                  pos="0 0 0",quat="1 0 0 0")
    scene_detail=Path(scene).with_name("rb_latte_20s_detail_scene.xml")
    tree.write(scene_detail,encoding="unicode")
    model=mujoco.MjModel.from_xml_path(str(scene_detail))
    data=mujoco.MjData(model)
    _set_free_pose(model,data,"rb_pitcher_free",[-10,-10,-10],0)
    _set_free_pose(model,data,"rb_plastic_cup_free",[-10,-10,-10],0)
    mujoco.mj_forward(model,data)
    qids,limits=robot_qpos_addresses(model)
    home,_=_find_start(model,data,qids,limits,args.seed)
    stations=plan_staging(model,data,qids,limits,home,args.seed)
    item=stations["pour_cup"]
    if item["stage"] is None:
        raise RuntimeError("No collision-safe pour station predock")
    origin=np.array(item["stage"]["q_rad"],float)
    timestamps=np.arange(args.duration*args.fps)/args.fps
    offsets=np.array([gesture(t) for t in timestamps])
    paths,scale=preflight(model,data,qids,limits,origin,offsets)

    _alpha(model,"pitcher")
    model.geom("rb_gripped_pitcher_foam").rgba[3]=.88
    _set_free_pose(model,data,"rb_plastic_cup_free",
                   DOCKS["pour_cup"].position_m,0)
    renderer=mujoco.Renderer(model,width=args.width,height=args.height)
    follow=ArmFollowCamera(model,smoothing=.34)
    detail_mocap=int(model.body_mocapid[model.body('rb_grip_detail_rig').id])
    frames_dir=out/"frames"
    frames_dir.mkdir(exist_ok=True)
    spouts=[]
    wrists=[]
    collisions=[]
    grip_body=model.body("rb_gripped_pitcher").id
    tip=model.geom("rb_gripped_pitcher_spout").id
    neck=model.geom("rb_gripped_pitcher_adapter").id
    left=model.geom("rb_handle_jaw_-1").id
    right=model.geom("rb_handle_jaw_1").id
    try:
        for i,(t,q) in enumerate(zip(timestamps,paths)):
            valid,reason=_valid_state(model,data,qids,limits,q,"pitcher")
            if not valid:
                collisions.append({"time_s":float(t),"reason":reason})
                break
            # Inspect the actual model positions, not a hand-painted overlay.
            spout=np.asarray(data.geom_xpos[tip]).copy()
            grip=np.asarray(data.geom_xpos[neck]).copy()
            center=(data.geom_xpos[left]+data.geom_xpos[right])/2
            if np.linalg.norm(center-grip)>.0011:
                raise RuntimeError("Gripping jaws are not centered on handle adapter")
            spouts.append(spout.tolist())
            wrists.append(data.xpos[grip_body].tolist())
            follow.update(data)
            pitcher_R=data.xmat[grip_body].reshape(3,3)
            handle_pos=np.asarray(data.xpos[grip_body],float)
            look=handle_pos+pitcher_R@np.array([0,.057,-.006])
            close_eye=handle_pos+pitcher_R@np.array([.33,-.16,.23])
            data.mocap_pos[detail_mocap]=close_eye
            data.mocap_quat[detail_mocap]=look_at_quat(close_eye,look)
            mujoco.mj_forward(model,data)
            def render_rgb(camera):
                renderer.disable_depth_rendering()
                renderer.update_scene(data,camera=camera)
                return renderer.render().copy()
            rgb=render_rgb(CAMERA_NAME)
            detail=render_rgb("rb_grip_detail")
            overview=render_rgb("cell_overview")
            pic=Image.fromarray(rgb).resize((960,720),Image.Resampling.BILINEAR)
            frame=Image.new("RGB",(960,790),(15,26,32))
            frame.paste(pic,(0,38))
            inset=Image.fromarray(overview).resize((262,197),Image.Resampling.BILINEAR)
            frame.paste(inset,(682,51))
            close=Image.fromarray(detail).resize((306,230),Image.Resampling.BILINEAR)
            frame.paste(close,(15,51))
            draw=ImageDraw.Draw(frame)
            draw.rectangle((12,48,324,284),outline=(239,244,247),width=3)
            draw.rectangle((15,51,321,74),fill=(17,27,35))
            draw.text((22,56),"GRIPPER + HANDLE CLOSE-UP",fill="white")
            draw=ImageDraw.Draw(frame)
            draw.rectangle((679,48,947,252),outline=(240,240,240),width=3)
            draw.rectangle((682,51,943,72),fill=(17,27,35))
            draw.text((689,54),"OVERVIEW",fill="white")
            draw.text((15,11),f"IR40-50 grip | 20 s fine pour | {t:05.2f}s",fill="white")
            draw.text((15,765),f"{phase_name(t)} | millimetre-scale joint motion | symbolic foam, no liquid",fill=(240,214,170))
            frame.save(frames_dir/f"pour_{i:04d}.png")
    finally:
        renderer.close()

    if collisions:raise RuntimeError("Gesture collision: "+str(collisions[0]))
    if len(spouts)!=len(timestamps):raise RuntimeError("Incomplete video samples")
    spouts=np.asarray(spouts,float)
    movement_mm=np.linalg.norm((spouts-spouts[0])*1000,axis=1)
    summary={
        "status":"PASS","duration_s":20., "fps":args.fps,
        "frames":len(timestamps),
        "simulator":"MuJoCo forward-kinematics with checked poses, not mj_step motor control",
        "view":"wrist-follow POV + gripper close-up + whole scene inset",
        "gripper":"two conceptual IR40-50; handle between first gripper fingers",
        "rigid_kinematic_handle_fixture":True,
        "physical_grasp_force_verified":False,
        "milk_fluid_dynamics":False,
        "milk_pouring_onto_cup_surface":False,
        "predock_not_actual_cup_surface":True,
        "gesture_scale":scale,
        "spout_max_displacement_from_start_mm":float(np.max(movement_mm)),
        "spout_rms_displacement_from_start_mm":float(np.sqrt(np.mean(movement_mm**2))),
        "phases":[x[0] for x in STAGES],
        "movement_error_is_not_tracking_error":True
    }
    (out/"summary.json").write_text(json.dumps(summary,indent=2))
    (out/"spout_positions.json").write_text(json.dumps(
        [{"time_s":round(float(t),5),"world_xyz_m":p}
            for t,p in zip(timestamps,spouts.tolist())],indent=2))
    if not shutil.which("ffmpeg"):
        raise RuntimeError("FFmpeg required for reproducible 20-second video")
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(args.fps),
        "-i",str(frames_dir/"pour_%04d.png"),"-c:v","libx264","-pix_fmt",
        "yuv420p","-movflags","+faststart",str(out/"latte_art_20s.mp4")],
        check=True)
    summary["video_created"]=(out/"latte_art_20s.mp4").stat().st_size>10000
    (out/"summary.json").write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--vendor-dir",default="vendor")
    parser.add_argument("--output",default="runs/latte_art_20s")
    parser.add_argument("--seed",type=int,default=14)
    parser.add_argument("--duration",type=int,default=20)
    parser.add_argument("--fps",type=int,default=15)
    parser.add_argument("--width",type=int,default=640)
    parser.add_argument("--height",type=int,default=480)
    args=parser.parse_args()
    report=run(args)
    import sys
    sys.stdout.flush()
    if os.getenv("MUJOCO_GL","").lower()=="osmesa":
        os._exit(0 if report["video_created"] else 2)
    if not report["video_created"]:
        raise SystemExit(2)


if __name__=="__main__":
    main()
