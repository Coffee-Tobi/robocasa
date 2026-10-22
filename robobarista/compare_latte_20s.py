"""Side-by-side 20s: generated latte-art joint trajectory vs torque-actuated FR5.

Uses EXACT SAME preflighted 20s joint trajectory as latte_art_20s.py.
LEFT: ideal forward kinematics (q(t) assigned to a reference data object).
RIGHT: independent MuJoCo mj_step integration using 6 bounded torque motors
and robust computed-torque/PD control, not reassigned target joints.

Spout errors are measured on the *gripper-held jug spout geom* itself,
not at bare TCP. The gripper and jug remain rigid decorative attachments:
no finger contact, milk mass, sloshing, realistic wet latte art or real robot.
"""
from __future__ import annotations
import argparse,json,os,shutil,subprocess
from pathlib import Path
import numpy as np
import mujoco
from scipy.interpolate import CubicSpline
from PIL import Image,ImageDraw
from .latte_art_20s import _story_scene,gesture,phase_name,preflight
from .coffee_workflow import _alpha,_valid_state
from .camera_joint_replay import _find_start
from .safe_staging import plan_staging
from .cell_demo import _set_free_pose
from .robot_replay import robot_qpos_addresses,JOINTS,_contacts
from .motor_follow import _actuators
from .precision_pour import _torque,DYN_STEP,CTRL_STEP
from .follow_camera import ArmFollowCamera,CAMERA_NAME
from .workcell_constraints import DOCKS,payload_check,TRAVEL_CLEARANCE_M


def initial_pose(model,data):
    _set_free_pose(model,data,"rb_pitcher_free",[-10,-10,-10],0)
    _set_free_pose(model,data,"rb_plastic_cup_free",DOCKS["pour_cup"].position_m,0)
    mujoco.mj_forward(model,data)


def composite(left,right,time_s,error_mm,phase,traces,initial_point):
    width,height=left.shape[1],left.shape[0]
    out=Image.new("RGB",(2*width,height+135),(15,24,31))
    canvas=ImageDraw.Draw(out)
    out.paste(Image.fromarray(left),(0,44))
    out.paste(Image.fromarray(right),(width,44))
    canvas.rectangle((width-2,44,width+2,44+height),fill=(240,233,210))
    canvas.text((14,11),"SOLL | generierte 20s-Bewegung (ideal)",fill=(182,235,245))
    canvas.text((width+14,11),"IST | FR5 mit MuJoCo-Drehmomentregelung",fill=(255,216,160))
    bottom=48+height
    canvas.text((14,bottom),f"{time_s:05.2f}s / 20.00s   |   {phase}   |   Ausgiesserfehler: {error_mm:.3f} mm",fill=(250,250,250))
    canvas.text((14,bottom+22),"Beide Bilder: gleiche Kamerapose. Keine echte Milch, Griffigkeit oder fluessige Latte Art.",fill=(190,202,212))
    # Exaggerated *plot scale only*, not an altered robot animation.
    origin=(width+width//2,bottom+58)
    canvas.text((width+14,bottom+22),"2D-Spur von oben: 1 mm = 14 px (nur Diagramm)",fill=(182,235,245))
    canvas.ellipse((origin[0]-2,origin[1]-2,origin[0]+2,origin[1]+2),fill=(230,230,230))
    def projected(row):
        rel=(np.asarray(row)-initial_point)*1000
        return (int(origin[0]+14*rel[0]),int(origin[1]-14*rel[1]))
    for index,color in [(0,(120,225,245)),(1,(255,193,117))]:
        coords=[projected(t[index]) for t in traces]
        if len(coords)>1:canvas.line(coords,fill=color,width=2)
        if coords:
            x,y=coords[-1]
            canvas.ellipse((x-4,y-4,x+4,y+4),fill=color)
    return out


def run(args):
    if args.duration!=20 or args.fps not in (10,12,15):
        raise ValueError("Use 20s and 10, 12 or 15 FPS")
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    model=mujoco.MjModel.from_xml_path(str(_story_scene(Path(args.vendor_dir))))
    model.opt.timestep=DYN_STEP
    pre=mujoco.MjData(model)
    initial_pose(model,pre)
    qids,limits=robot_qpos_addresses(model)
    home,_=_find_start(model,pre,qids,limits,args.seed)
    stage=plan_staging(model,pre,qids,limits,home,args.seed)["pour_cup"]["stage"]
    if stage is None:raise RuntimeError("no preflighted pour predock")
    origin=np.asarray(stage["q_rad"],float)
    points=np.linspace(0,20,20*15+1)
    trajectory,scale=preflight(model,pre,qids,limits,origin,
                              np.array([gesture(t) for t in points]))
    # Estimate smooth joint velocity & acceleration from the same poses.
    fit=CubicSpline(points,trajectory,axis=0,bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
    reference=mujoco.MjData(model)
    actual=mujoco.MjData(model)
    inverse=mujoco.MjData(model)
    for d in (reference,actual,inverse):
        initial_pose(model,d)
        d.qpos[qids]=trajectory[0]
        d.qvel[:]=0
        mujoco.mj_forward(model,d)
    _alpha(model,"pitcher")
    model.geom("rb_gripped_pitcher_foam").rgba[3]=.88
    tip_id=model.geom("rb_gripped_pitcher_spout").id
    qdofs=np.array([int(model.jnt_dofadr[model.joint(n).id]) for n in JOINTS],int)
    aids,tranges=_actuators(model)
    follow=ArmFollowCamera(model,smoothing=.35)
    film=[]
    observations=[]
    max_tau=0
    torque_saturated=0
    error_integral=np.zeros(6,float)
    control_next=0
    frame_next=0
    video_count=int(args.fps*20)
    total_steps=int(round(20/DYN_STEP))
    renderer=mujoco.Renderer(model,width=args.width,height=args.height)
    initial_tip=None
    try:
        for step in range(total_steps+1):
            now=min(step*DYN_STEP,20.)
            q=np.asarray(fit(now),float)
            qvel=np.asarray(fit(now,1),float)
            qacc=np.asarray(fit(now,2),float)
            if step==0 or now+1e-9>=control_next:
                error_integral=np.clip(
                    error_integral+CTRL_STEP*(q-actual.qpos[qids]),-.06,.06)
                saturated,tau=_torque(model,actual,inverse,qids,qdofs,
                    aids,tranges,"computed_torque_robust",q,qvel,qacc,error_integral)
                max_tau=max(max_tau,tau)
                torque_saturated+=int(saturated)
                control_next+=CTRL_STEP
            # Never overwrite ACTUAL joint positions: dynamics only.
            if step<total_steps:mujoco.mj_step(model,actual)
            if now+1e-8>=frame_next and len(film)<video_count:
                reference.qpos[qids]=q
                reference.qvel[qdofs]=qvel
                mujoco.mj_forward(model,reference)
                desired_tip=reference.geom_xpos[tip_id].copy()
                observed_tip=actual.geom_xpos[tip_id].copy()
                error=float(np.linalg.norm(observed_tip-desired_tip)*1000.)
                if not np.isfinite(error):raise RuntimeError("nonfinite motor error")
                if initial_tip is None:initial_tip=desired_tip.copy()
                if np.any(actual.qpos[qids]<limits[:,0]-1e-4) or np.any(actual.qpos[qids]>limits[:,1]+1e-4):
                    raise RuntimeError("motor joint limit violation")
                ext,internal=_contacts(model,actual)
                if ext or internal:raise RuntimeError("dynamic robot contact: "+str((ext+internal)[0]))
                # Check payload clearance at physical motor pose, not only reference.
                tcp=model.body("rb_cell_tcp").id
                hits=payload_check(actual.xpos[tcp],actual.xmat[tcp].reshape(3,3),
                    "pitcher",clearance_m=TRAVEL_CLEARANCE_M)
                if hits:raise RuntimeError("actual jug clearance violation: "+str(hits[0]))
                observations.append({"t_s":round(now,4),"error_spout_mm":error,
                    "generated_tip_xyz_m":desired_tip.tolist(),
                    "motor_tip_xyz_m":observed_tip.tolist()})
                follow.update(reference)
                # Fix identical viewing coordinates for both branches; render color only
                cam_mid=follow.mocap_id
                actual.mocap_pos[cam_mid]=reference.mocap_pos[cam_mid].copy()
                actual.mocap_quat[cam_mid]=reference.mocap_quat[cam_mid].copy()
                mujoco.mj_forward(model,actual)
                renderer.disable_depth_rendering()
                renderer.update_scene(reference,camera=CAMERA_NAME)
                left=renderer.render().copy()
                renderer.update_scene(actual,camera=CAMERA_NAME)
                right=renderer.render().copy()
                fig=composite(left,right,now,error,phase_name(now),[
                    (o["generated_tip_xyz_m"],o["motor_tip_xyz_m"]) for o in observations
                ],initial_tip)
                film.append(fig)
                frame_next=len(film)/args.fps
    finally:
        renderer.close()
    if len(film)!=video_count:raise RuntimeError(f"missing frames: {len(film)}")
    if not shutil.which("ffmpeg"):raise RuntimeError("ffmpeg missing")
    frames=out/"frames"
    frames.mkdir(exist_ok=True)
    for i,f in enumerate(film):f.save(frames/f"compare_{i:04d}.png")
    movie=out/"generated_vs_fr5_motor_20s.mp4"
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(args.fps),
        "-i",str(frames/"compare_%04d.png"),"-c:v","libx264","-pix_fmt","yuv420p",
        "-movflags","+faststart",str(movie)],check=True)
    errs=np.array([x["error_spout_mm"] for x in observations])
    report={"status":"PASS","duration_s":20,"fps":args.fps,"frames":len(film),
        "reference":"same gesture as robobarista.latte_art_20s, checked 301 knots",
        "actual":"MuJoCo mj_step, six bounded torque-controlled FR5 joints, robust computed torque",
        "generated_is_kinematic":True,"actual_joint_states_from_motor_dynamics":True,
        "tracked_geom":"rb_gripped_pitcher_spout","fixture":"rigid jaw-attached jug (no simulated grip contacts)",
        "mean_spout_tracking_error_mm":float(np.mean(errs)),
        "max_spout_tracking_error_mm":float(np.max(errs)),
        "p95_spout_tracking_error_mm":float(np.percentile(errs,95)),
        "torque_saturation_events":int(torque_saturated),
        "maximum_raw_motor_torque_nm":float(max_tau),
        "gesture_scale":float(scale),"real_liquid":False,"robot_hardware_validated":False,
        "notes":"Errors measured between 2 independently simulated data objects with same model; does not include payload mass, flexible handle, camera noise, actual safety envelope or latte art fluid."}
    (out/"summary.json").write_text(json.dumps(report,indent=2))
    (out/"spout_tracking.json").write_text(json.dumps(observations,indent=1))
    print(json.dumps(report,indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--output",default="runs/latte_compare_20s")
    p.add_argument("--seed",type=int,default=14)
    p.add_argument("--duration",type=int,default=20)
    p.add_argument("--fps",type=int,default=12)
    p.add_argument("--width",type=int,default=520)
    p.add_argument("--height",type=int,default=400)
    args=p.parse_args()
    r=run(args)
    import sys
    sys.stdout.flush()
    if os.getenv("MUJOCO_GL","").lower()=="osmesa":
        os._exit(0 if r["status"]=="PASS" else 2)

if __name__=="__main__":main()
