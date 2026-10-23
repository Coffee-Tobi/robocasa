"""Replay OFFICIAL FAIRINO NewSpline *waypoints* with an honest MuJoCo robot.

Three simulated RGB-D cameras measure AprilTag pitcher poses. The previously
generated manufacturer NewSpline API waypoints are interpolated here by our
OWN Scipy Cartesian/rotation spline (NOT the proprietary FAIRINO firmware).
Per frame, constrained 6D IK drives six FR5 MuJoCo joints; rendered tool tip
is read from the actual scene, NOT painted or copied from the desired pose.

This is kinematic robot replay, not SDK execution, not motor torque dynamics,
not rigid-grasp physics, not a validated latte-art process, and not real robot
accuracy. It IS a video of the articulated FR5 moving.

Example:
 python -m robobarista.fairino_mujoco_replay \
   --input validated_camera.zip --output runs/fairino_replay --fps 8
"""
from __future__ import annotations
import argparse,json,os,shutil,subprocess
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation,RotationSpline
import mujoco
from PIL import Image,ImageDraw
from .fairino_barista_pipeline import load_camera,target_motion
from .fairino_native_spline import make_plan,demo_calibration
from .coffee_workflow import _story_scene,_alpha
from .camera_joint_replay import _find_start,_safe_q_segment
from .robot_replay import robot_qpos_addresses,compute_ik
from .cell_demo import _set_free_pose
from .follow_camera import ArmFollowCamera,CAMERA_NAME


def spline_from_fairino_commands(plan,duration):
    """Smooth CARTESIAN approximation through native FAIRINO waypoints.

    Real NewSpline interpolation, speed and handling of blendR may differ!
    """
    commands=[x for x in plan["commands"] if x["api"]=="NewSplinePoint"]
    if len(commands)<4:
        raise ValueError("Need >=4 FAIRINO spline points")
    targets=np.asarray([x["kwargs"]["desc_pos"] for x in commands],float)
    knots=np.linspace(0.,duration,len(targets))
    p=CubicSpline(knots,targets[:,:3]/1000,axis=0,bc_type="natural")
    quat=Rotation.from_euler("xyz",targets[:,3:],degrees=True)
    r=RotationSpline(knots,quat)
    return p,r


def film_frame(front,overview,t,desired_tip,actual_tip,track,origin,error):
    # Real robot rendering on top, numerical traces at bottom.
    im=Image.new("RGB",(960,790),(17,27,36))
    d=ImageDraw.Draw(im)
    im.paste(Image.fromarray(front).resize((960,660),Image.Resampling.BILINEAR),(0,40))
    inset=Image.fromarray(overview).resize((270,203),Image.Resampling.BILINEAR)
    im.paste(inset,(672,48))
    d.rectangle((669,45,945,254),outline=(235,235,240),width=3)
    d.rectangle((671,47,944,69),fill=(17,27,36))
    d.text((680,52),"GESAMTANSICHT",fill="white")
    d.text((14,11),f"FR5 aus FAIRINO NewSpline-Wegpunkten | {t:05.2f} s",fill=(245,245,250))
    d.rectangle((0,701,960,790),fill=(17,27,36))
    d.text((16,709),f"Virtueller FR5 faehrt IK-Weg | TCP-Spitzenabweichung {error:.3f} mm",fill=(245,245,245))
    # 2D traces of the actual scene's gripper-pitcher spout and desired tip.
    # This is a relative path plot with clearly labelled scale, not extra robot pose.
    def pixel(p):
        z=(np.asarray(p)-origin)*1000.
        return (int(470+z[0]*6),int(756-z[1]*6))
    for index,color in ((0,(125,230,248)),(1,(255,190,112))):
        pts=[pixel(e[index]) for e in track]
        if len(pts)>1:d.line(pts,fill=color,width=3)
        if pts:
            x,y=pts[-1];d.ellipse((x-4,y-4,x+4,y+4),fill=color)
    d.text((16,742),"BLAU Soll | ORANGE FR5 Ist",fill=(190,205,224))
    d.text((16,765),"6 px/mm, nur Diagramm | KEINE FAIRINO-FIRMWARE",fill=(190,205,224))
    return im


def render(args):
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    if args.duration!=20:raise ValueError("duration fixed to 20s")
    rows=load_camera(args.input)
    rows,translation_scale,original_excursion,scaled_excursion=target_motion(
        rows,target_displacement_mm=args.target_mm,duration_s=args.duration)
    # EXACT documented method command set; this is no real SDK connection.
    plan=make_plan(rows,demo_calibration(),"new_spline",args.duration,args.stride,.5)
    (out/"fairino_new_spline_commands.json").write_text(json.dumps(plan,indent=2))
    curve,rot=spline_from_fairino_commands(plan,args.duration)
    scene=_story_scene(Path(args.vendor_dir))
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    _set_free_pose(model,data,"rb_pitcher_free",[-10,-10,-10],0)
    _set_free_pose(model,data,"rb_plastic_cup_free",[-10,-10,-10],0)
    mujoco.mj_forward(model,data)
    _alpha(model,"pitcher")
    model.geom("rb_gripped_pitcher_foam").rgba[3]=.87
    qids,limits=robot_qpos_addresses(model)
    q,_=_find_start(model,data,qids,limits,args.seed)
    tcp=model.body("rb_cell_tcp").id
    spout_id=model.geom("rb_gripped_pitcher_spout").id
    data.qpos[qids]=q
    mujoco.mj_forward(model,data)
    # CAD-known local coordinate of the simulated spout; this is NOT a real
    # calibrated tool transform, and source was synthesized from TCP FK.
    p0=data.xpos[tcp].copy()
    r0=data.xmat[tcp].reshape(3,3).copy()
    local_tip=r0.T@(data.geom_xpos[spout_id]-p0)
    camera=ArmFollowCamera(model,smoothing=.36)
    render_engine=mujoco.Renderer(model,width=args.width,height=args.height)
    times=np.arange(args.duration*args.fps)/args.fps
    observations=[]
    frames=[]
    first_tip=None
    preview=out/"frames";preview.mkdir(exist_ok=True)
    try:
        for i,t in enumerate(times):
            desired_p=np.asarray(curve(float(t)),float)
            desired_r=rot(float(t)).as_matrix()
            # Continuous IK is applied at each simulation frame. If a move
            # cannot be reached or violates safety, abort instead of faking it.
            sol=compute_ik(model,data,tcp,qids,limits,desired_p,desired_r,q,max_evals=95)
            if not sol["solved"]:
                raise RuntimeError(f"Frame {i} / {t:.3f}s: IK not solved; "
                    f"{sol['position_error_mm']:.3f}mm/{sol['rotation_error_deg']:.3f}deg")
            safe,reason,_=_safe_q_segment(model,data,qids,limits,q,sol["q"])
            if not safe:raise RuntimeError(f"Frame {i}, unsafe: {reason}")
            q=sol["q"].copy()
            data.qpos[qids]=q
            mujoco.mj_forward(model,data)
            actual_tip=data.geom_xpos[spout_id].copy()
            target_tip=desired_p+desired_r@local_tip
            error=1000*np.linalg.norm(actual_tip-target_tip)
            if first_tip is None:first_tip=target_tip.copy()
            observations.append(dict(time_s=float(t),
               desired_spout_xyz_m=target_tip.tolist(),
               simulated_fr5_spout_xyz_m=actual_tip.tolist(),
               spout_replay_error_mm=float(error),
               tcp_ik_error_mm=float(sol["position_error_mm"]),
               tcp_ik_angle_error_deg=float(sol["rotation_error_deg"]),
               joint_positions_rad=q.tolist()))
            camera.update(data)
            render_engine.disable_depth_rendering()
            render_engine.update_scene(data,camera=CAMERA_NAME)
            main=render_engine.render().copy()
            render_engine.update_scene(data,camera="cell_overview")
            wide=render_engine.render().copy()
            traces=[(o["desired_spout_xyz_m"],o["simulated_fr5_spout_xyz_m"])
                    for o in observations]
            im=film_frame(main,wide,float(t),target_tip,actual_tip,traces,first_tip,float(error))
            im.save(preview/f"fr5_{i:04d}.png")
            frames.append(1)
    finally:
        render_engine.close()
    if len(frames)!=len(times):raise RuntimeError("Incomplete IK+render replay")
    if not shutil.which("ffmpeg"):raise RuntimeError("ffmpeg required")
    movie=out/"fairino_waypoints_fr5_mujoco_20s.mp4"
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(args.fps),
       "-i",str(preview/"fr5_%04d.png"),"-c:v","libx264","-pix_fmt","yuv420p",
       "-movflags","+faststart",str(movie)],check=True)
    errors=np.array([x["spout_replay_error_mm"] for x in observations],float)
    traces=np.array([o["desired_spout_xyz_m"] for o in observations])
    spout_excursion=float(np.linalg.norm((traces-traces[0])*1000,axis=1).max())
    summary=dict(
        status="PASS",duration_s=20,fps=args.fps,frames=len(frames),
        source_camera_frames=len(rows),fairino_new_spline_waypoints=plan["waypoints"],
        target_motion_source="3 virtual RGB-D cameras + AprilTag 6D pose",
        interpolation="SCIPY C2 Cartesian translation + SO3 RotationSpline PROXY, not FAIRINO firmware",
        robot_motion="Actual articulated FR5 hinge joints assigned via per-frame numerical inverse kinematics and verified MuJoCo forward kinematics",
        no_actual_fairino_firmware_run=True,
        no_real_robot_connected=True,
        no_mujoco_motor_torque_dynamics=True,
        no_real_gripper_forces_or_fluid_physics=True,
        model_only_gripper_spout_tip=True,
        synthetic_source_camera_tip_to_tcp_mapping=True,
        original_camera_translation_excursion_mm=original_excursion,
        scaled_camera_translation_excursion_mm=scaled_excursion,
        realized_target_tip_excursion_mm=spout_excursion,
        mean_spout_geometric_replay_error_mm=float(errors.mean()),
        max_spout_geometric_replay_error_mm=float(errors.max()),
        p95_spout_geometric_replay_error_mm=float(np.percentile(errors,95)),
        error_is_firmware_tracking_error=False)
    (out/"summary.json").write_text(json.dumps(summary,indent=2))
    (out/"soll_ist_ausgiesser.json").write_text(json.dumps(observations,indent=2))
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--input",required=True)
    ap.add_argument("--output",default="runs/new_fairino_fr5_follow")
    ap.add_argument("--vendor-dir",default="vendor")
    ap.add_argument("--seed",type=int,default=14)
    ap.add_argument("--fps",type=int,default=8)
    ap.add_argument("--duration",type=int,default=20)
    ap.add_argument("--target-mm",type=float,default=18.)
    ap.add_argument("--stride",type=int,default=2)
    ap.add_argument("--width",type=int,default=640)
    ap.add_argument("--height",type=int,default=480)
    args=ap.parse_args()
    result=render(args)
    import sys
    sys.stdout.flush()
    if os.getenv("MUJOCO_GL","").lower()=="osmesa":
        os._exit(0 if result["status"]=="PASS" else 1)


if __name__=="__main__":main()
