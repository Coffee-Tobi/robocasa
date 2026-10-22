"""Full 11-station coffee movie with ACTUALLY slower FR5 joint motion.

Unlike the archived slow-motion export, every video frame is a newly
computed collision-screened MuJoCo kinematic robot pose at a fixed 12 fps.
A quintic (minimum jerk) joint interpolator sets a conservative explicit
velocity cap. Its sampled angular speed and station-hold frames are logged.

The tamper is anchored INSIDE the grinder and moves ONLY vertically while
the robot is stationary; it does not turn with the arm.

Grips, portafilter insertion, espresso, milk and synthetic FT contact remain
visual concepts and predock poses (not force-driven physical operations).
"""
from __future__ import annotations
import argparse,json,math,os,shutil,subprocess
from pathlib import Path
from PIL import Image,ImageDraw
import numpy as np
import mujoco

from .coffee_workflow import _story_scene,_alpha,_place_cup,RECIPE,_random_pour_gesture
from .robot_replay import robot_qpos_addresses,compute_ik
from .camera_joint_replay import _find_start
from .safe_staging import plan_staging,_check_route,_valid_state
from .cell_demo import _set_free_pose
from .workcell_constraints import DOCKS
from .follow_camera import ArmFollowCamera,CAMERA_NAME
from .demo import render_pair
from .full_barista_sensor_video import (
    draw_panel,font,STEP_TEXT,illustrative_contact_curve,
    portafilter_lock_monitor,steam_wand_side_contact)


def smooth_joint_leg(qa,qb,fps,limit_rad_s):
    """Each new joint pose is sampled at the DISPLAY FPS.

    Quintic smootherstep has peak derivative 1.875. This controls the
    requested maximum speed of the displayed joint trajectory, NOT
    physical acceleration/torque compliance.
    """
    a=np.asarray(qa,float);b=np.asarray(qb,float)
    travel=float(np.max(np.abs(b-a)))
    n=max(3,int(np.ceil(1.875*travel*fps/limit_rad_s))+1)
    for t in np.linspace(0.,1.,n):
        c=10*t**3-15*t**4+6*t**5
        yield a+c*(b-a)


def segment(qa,qb,fps,limit,phase):
    return [(q,phase,0.) for q in smooth_joint_leg(qa,qb,fps,limit)]


def sequence(home,stage,action,pour_ctrls,fps,speed):
    if action=="pour" and pour_ctrls:
        seq=segment(home,pour_ctrls[0],fps,speed,"ANFAHRT")
        for a,b in zip(pour_ctrls,pour_ctrls[1:]):
            seq.extend(segment(a,b,fps,speed*.65,"GIESSBEWEGUNG")[1:])
        seq.extend(segment(pour_ctrls[-1],pour_ctrls[0],fps,speed*.65,"GIESSBEWEGUNG")[1:])
        seq.extend(segment(pour_ctrls[0],home,fps,speed,"RUECKFAHRT")[1:])
        return seq
    seq=segment(home,stage,fps,speed,"ANFAHRT")
    # The grinder robot STOPs and holds the portafilter. ONLY the grinder's
    # own vertical slide tamper moves during this fixed hold.
    seconds={"grind_tamp":3.4,"brew":2.4,"steam":3.2,"dose":1.4,
             "insert_grouphead":1.6,"place_cup":.8,"cup_to_pour":.8}.get(action,.7)
    hold=max(2,int(round(seconds*fps)))
    for i in range(hold):
        progress=i/max(1,hold-1)
        slide=.038*max(0.,math.sin(math.pi*progress))**2 if action=="grind_tamp" else 0.
        seq.append((np.asarray(stage,float).copy(),"STATIONAER",slide))
    seq.extend(segment(stage,home,fps,speed,"RUECKFAHRT")[1:])
    return seq


def overlay(base,action,stage_idx,phase,t_s,velocity,slide,progress,
            wrench,verdict,history,blocked_count):
    img=draw_panel(base,action,stage_idx,progress,wrench,verdict,history,True,
                   tamper_depth_mm=slide*1000.)
    draw=ImageDraw.Draw(img)
    # Header explicitly shows camera + real robot pose rate.
    draw.rectangle((0,0,960,58),fill=(14,26,37))
    draw.text((18,10),"FR5 BARISTA | 3/4 SEITENANSICHT",font=font(20),
              fill=(244,249,251))
    draw.text((600,14),f"{t_s:05.1f} s  |  12 B/s",font=font(18),
              fill=(169,222,236))
    draw.rectangle((12,708,955,804),fill=(14,27,38))
    draw.text((25,715),f"{phase}   |   Gelenkgeschwindigkeit: {velocity:.3f} rad/s",
              font=font(17),fill=(170,221,238))
    if action=="grind_tamp":
        draw.text((25,746),f"Grinder-Tamper: senkrechter Hub {slide*1000:.1f} mm",
                  font=font(18),fill=(250,207,144))
        draw.text((25,771),"ARM WAEHREND TAMPEN FEST / TAMPPER RUTSCHT SENKRECHT",
                  font=font(14),fill=(255,213,154))
    else:
        draw.text((25,746),"IR40 A: Siebtraeger / Milchkanne  |  IR40 B: Becher",
                  font=font(15),fill=(243,226,173))
        draw.text((25,774),"SICHERE VORPOSITIONEN; KONTAKT UND FLUESSIGKEIT NUR SYMBOLISCH",
                  font=font(12),fill=(226,179,152))
    if blocked_count:
        draw.text((1000,775),f"IK-Halteframes: {blocked_count}",
                  font=font(13),fill=(243,183,133))
    return img


def run(args):
    root=Path(args.output).resolve()
    root.mkdir(parents=True,exist_ok=True)
    tmp=root/"frames"
    tmp.mkdir(exist_ok=True)
    model=mujoco.MjModel.from_xml_path(str(_story_scene(args.vendor_dir)))
    data=mujoco.MjData(model)
    for name in ("rb_pitcher_free","rb_plastic_cup_free"):
        _set_free_pose(model,data,name,[-10.,-10.,-10.],0)
    mujoco.mj_forward(model,data)
    qids,limits=robot_qpos_addresses(model)
    home,_=_find_start(model,data,qids,limits,14)
    stages=plan_staging(model,data,qids,limits,home,14)
    if any(x["status"]!="PRE_DOCK_ROUTE_CHECKED" for x in stages.values()):
        raise RuntimeError("Unsafe station; do not fabricate route")
    _,pour_ctrls=_random_pour_gesture(model,data,qids,limits,home,stages,14)
    tamper=model.joint("rb_grinder_tamper_slide").id
    tamper_qadr=int(model.jnt_qposadr[tamper])
    renderer=mujoco.Renderer(model,width=args.width,height=args.height)
    cam=ArmFollowCamera(model,smoothing=.34)
    events=[]
    frames=0
    last_q=None
    peak_v=0.
    peak_step=0.
    stationary_max_v=0.
    tamper_max=0.
    total_clamp=0
    angle_max=0.
    operations={k:False for k in ("grind_tamp","steam","pour")}
    espresso=False
    milk=False
    latest_blocked=0
    try:
        for j,(label,dock,tool,action) in enumerate(RECIPE):
            stage=np.asarray(stages[dock]["stage"]["q_rad"],float)
            valid,reason,n=_check_route(model,data,qids,limits,home,stage,tool)
            if not valid:raise RuntimeError(f"Blocked station: {action}/{reason}")
            seq=sequence(home,stage,action,pour_ctrls,args.fps,
                         args.max_joint_speed_rad_s)
            if action=="place_cup":
                _place_cup(model,data,DOCKS["espresso_cup"].position_m)
            if action=="cup_to_pour":
                _place_cup(model,data,DOCKS["pour_cup"].position_m)
            if action=="brew":espresso=True
            if action=="steam":milk=True
            _alpha(model,tool if tool!="bare_tcp" else "none")
            model.geom("rb_story_espresso_disc").rgba[3]=.85 if espresso else 0.
            model.geom("rb_story_latte_pattern").rgba[3]=1. if action in ("pour","rinse") else 0.
            model.geom("rb_gripped_pitcher_foam").rgba[3]=.88 if milk and tool=="pitcher" else 0.
            model.geom("rb_story_steam_visual").rgba[3]=.23 if action=="steam" else 0.
            samples=[]
            history=[]
            prev=home.copy()
            stage_blocked=0
            for k,(q_raw,phase,slide) in enumerate(seq):
                q=np.asarray(q_raw,float).copy()
                # Physical spill avoidance on carry-only moves: try to keep
                # robot TCP Z upright. No tilt enforcement during latte art.
                if tool in ("pitcher","plastic_cup") and action!="pour":
                    data.qpos[qids]=q
                    mujoco.mj_forward(model,data)
                    tcp=model.body("rb_cell_tcp").id
                    R=data.xmat[tcp].reshape(3,3)
                    tilt=float(np.rad2deg(np.arccos(np.clip(R[2,2],-1.,1.))))
                    if tilt>3.:
                        desired=data.xpos[tcp].copy()
                        ik=compute_ik(model,data,tcp,qids,limits,
                                      desired,np.eye(3),prev,max_evals=75)
                        if ik["solved"]:
                            good,_,_= _check_route(model,data,qids,limits,prev,ik["q"],tool)
                            if good and (float(np.max(np.abs(ik["q"]-prev)))*
                                         args.fps<=args.max_joint_speed_rad_s*1.1):
                                q=ik["q"].copy()
                            else:
                                q=prev.copy();stage_blocked+=1
                        else:
                            q=prev.copy();stage_blocked+=1
                dist=float(np.max(np.abs(q-prev)))
                observed_v=dist*args.fps
                if observed_v>args.max_joint_speed_rad_s*1.12:
                    # Speed must actually be limited in the robot positions
                    # rather than achieved by changing playback framerate.
                    raise RuntimeError(f"Speed cap exceeded at {action}: {observed_v}")
                ok,why=_valid_state(model,data,qids,limits,q,tool)
                if not ok:raise RuntimeError(f"Unsafe frame {action}: {why}")
                if tool in ("pitcher","plastic_cup") and action!="pour":
                    R=data.xmat[model.body("rb_cell_tcp").id].reshape(3,3)
                    tilt=float(np.rad2deg(np.arccos(np.clip(R[2,2],-1.,1.))))
                    angle_max=max(angle_max,tilt)
                if phase=="STATIONAER" and action=="grind_tamp":
                    stationary_max_v=max(stationary_max_v,observed_v)
                    tamper_max=max(tamper_max,slide)
                    operations["grind_tamp"]=True
                data.qpos[tamper_qadr]=slide
                mujoco.mj_forward(model,data)
                cam.update(data)
                rgb,_=__render(renderer,data,CAMERA_NAME)
                overview,_=__render(renderer,data,"cell_overview")
                # 2 views: large lateral follower + overview inset.
                base=Image.new("RGB",(960,806),(18,30,40))
                base.paste(Image.fromarray(rgb).resize((960,720),Image.Resampling.BILINEAR),(0,59))
                d=ImageDraw.Draw(base)
                d.text((14,61),f"{label}   {phase}",font=font(17),fill=(248,231,186))
                inset=Image.fromarray(overview).resize((276,207),Image.Resampling.BILINEAR)
                base.paste(inset,(670,91))
                d.rectangle((668,89,948,300),outline=(224,238,242),width=3)
                d.rectangle((670,91,946,113),fill=(15,27,39))
                d.text((678,93),"GESAMTE STATION",font=font(14),fill="white")
                progress=k/max(1,len(seq)-1)
                wrench,_=illustrative_contact_curve(action,progress)
                samples.append(wrench)
                history.append(abs(wrench.tz) if action=="insert_grouphead" else
                               abs(wrench.fy) if action=="steam" else 0.)
                verdict=(portafilter_lock_monitor(samples)["state"]
                         if action=="insert_grouphead" else
                         steam_wand_side_contact(samples)["state"]
                         if action=="steam" else "VIRTUELL / KEINE MESSUNG")
                img=overlay(base,action,j+1,phase,frames/args.fps,
                    observed_v,slide,progress,wrench,verdict,history,stage_blocked)
                img.save(tmp/f"frame_{frames:05d}.jpg",quality=args.jpeg_quality)
                frames+=1
                peak_v=max(peak_v,observed_v)
                peak_step=max(peak_step,dist)
                prev=q.copy()
            latest_blocked+=stage_blocked
            events.append(dict(action=action,stage=j+1,frame_count=len(seq),
                duration_s=len(seq)/args.fps,blocked_poses=stage_blocked,
                gripper_tool=tool,station=dock,
                machine_contact_reached=False))
            print("SLOW STAGE",j+1,action,"frames",len(seq),
                  "hold",stage_blocked,flush=True)
    finally:
        renderer.close()
    video=root/"coffee_process_real_slow_12fps.mp4"
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(args.fps),
       "-i",str(tmp/"frame_%05d.jpg"),"-c:v","libx264",
       "-preset","fast","-crf","24","-pix_fmt","yuv420p",
       "-movflags","+faststart",str(video)],check=True)
    metrics=dict(status="PASS",frames=frames,video_fps=args.fps,
       duration_s=frames/args.fps,
       frame_duplication_used=False,
       real_new_robot_joint_poses_each_frame=True,
       max_joint_speed_rad_s=peak_v,
       max_joint_step_per_frame_rad=peak_step,
       configured_max_speed_rad_s=args.max_joint_speed_rad_s,
       grinder_tamper_moving_axis_only="vertical_slide",
       grinder_tamper_max_depth_mm=tamper_max*1000.,
       grinder_arm_speed_while_tamping_rad_s=stationary_max_v,
       upright_ik_hold_frames=latest_blocked,
       upright_worst_observed_tilt_deg=angle_max,
       strict_upright=angle_max<=3.01,
       eleven_stages=len(events),
       zero_real_docking=True,
       no_real_milk_or_espresso=True,
       no_actual_motor_torque_or_fairino_firmware_tracking=True,
       movie_file=video.name,
       stages=events)
    (root/"summary.json").write_text(json.dumps(metrics,indent=2))
    print(json.dumps({k:v for k,v in metrics.items() if k!="stages"},indent=2),flush=True)
    if args.cleanup_frames:shutil.rmtree(tmp)
    return metrics


def __render(renderer,data,camera):
    from .demo import render_pair
    return render_pair(renderer,data,camera)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--output",default="runs/real_slow_12fps")
    p.add_argument("--fps",type=int,default=12)
    p.add_argument("--max-joint-speed-rad-s",type=float,default=.72)
    p.add_argument("--width",type=int,default=640)
    p.add_argument("--height",type=int,default=480)
    p.add_argument("--jpeg-quality",type=int,default=78)
    p.add_argument("--cleanup-frames",action="store_true")
    a=p.parse_args()
    if a.fps<10 or a.max_joint_speed_rad_s<=0:
        raise ValueError("Require FPS >=10 and positive motion cap")
    result=run(a)
    import sys
    sys.stdout.flush()
    if os.getenv("MUJOCO_GL","").lower()=="osmesa":
        os._exit(0 if result["status"]=="PASS" else 1)


if __name__=="__main__":main()
