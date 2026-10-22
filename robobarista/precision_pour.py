"""FR5 precision-pour mode: SAME small task, three motor controllers, fair comparison.

Generate a few-mm/degree time-parametrized 6D movement in collision-free
pre-dock free space (NOT physical pouring under the steam wand). A local
Ruckig trajectory bounds *virtual* speed/acceleration/jerk at 6 DOF.
Use actual MuJoCo torque actuators, gravity and dynamics via mj_step.

Compare on EXACTLY THE SAME joint trajectory:
 (1) old gravity + position-PD + zero target velocity
 (2) gravity + position/velocity PD
 (3) computed torque: mj_inverse(q,qd,qdd_des + PD tracking feedback)

Measure real 3D world position errors both at wrist TCP and at a fixed,
visually rendered pitcher SPOUT point including rotation-induced offset.
Stop & mark failure for any joint, arm contact, or full-pitcher keepout.

No human demo, liquids, true gripper mass, force-feedback or online vision.
Sources: MuJoCo mj_inverse doc, Ruckig local state-to-state solver.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

import mujoco
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw

from .camera_joint_replay import _add_held_pitcher_robot_variant, _validate_q
from .cell_demo import _set_free_pose
from .robot_replay import robot_qpos_addresses,compute_ik,JOINTS
from .safe_staging import plan_staging,_check_route
from .camera_joint_replay import _find_start
from .continuous_motion import _ruckig_sections,MAX_VEL,MAX_ACC,MAX_JERK
from .motor_follow import _actuators,POSITION_KP,VELOCITY_KD
from .demo import render_pair

# Idealized geometric nozzle on the visually attached pitcher; the massless
# visual tip in camera_joint_replay is at (-.053,0,.117) relative to TCP.
SPOUT_LOCAL_M=np.array([-.073,0,.117],dtype=float)
DYN_STEP=.001
CTRL_STEP=.004


def _tip(data,body):
    R=data.xmat[body].reshape(3,3)
    return data.xpos[body].copy(),data.xpos[body]+R@SPOUT_LOCAL_M


def _desired_tcp(model,ref,body,qids,desired_q):
    ref.qpos[qids]=desired_q
    mujoco.mj_forward(model,ref)
    return _tip(ref,body)


def _small_cartesian_demo(model,data,qids,limits,origin_q,
                          amplitude_mm=4,angular_deg=1,seconds=3,knots=19):
    """Create a closed smooth X-Y wiggle with slight pitcher rotation.

    All targets start/end in the same collision-checked free-space region.
    Desired start/stop velocity is zero. Each IK frame checked separately.
    """
    body=model.body("rb_cell_tcp").id
    data.qpos[qids]=origin_q
    mujoco.mj_forward(model,data)
    p0=data.xpos[body].copy()
    R0=data.xmat[body].reshape(3,3).copy()
    amp=amplitude_mm/1000
    angle=np.deg2rad(angular_deg)
    times=np.linspace(0,float(seconds),knots)
    q=[]
    guess=origin_q.copy()
    for t in times:
        u=t/seconds
        smooth=np.sin(np.pi*u)**2
        dp=np.array([
            amp*np.sin(2*np.pi*u)*smooth,
            amp*.6*np.sin(4*np.pi*u)*smooth,
            amp*.25*(1-np.cos(2*np.pi*u))*smooth])
        # A little tilt makes tip error different from wrist error.
        tilt=Rotation.from_euler("xy",
                   [angle*np.sin(2*np.pi*u)*smooth,
                    angle*.45*np.sin(4*np.pi*u)*smooth]).as_matrix()
        sol=compute_ik(model,data,body,qids,limits,p0+dp,R0@tilt,
                       guess,max_evals=125)
        if not sol["solved"]:
            raise RuntimeError(f"Precision-pour IK failed at t={t:.3f}s")
        valid,reason=_validate_q(model,data,qids,limits,sol["q"])
        if not valid:
            raise RuntimeError(f"Precision-pour pose collides at t={t:.3f}: {reason}")
        if q:
            safe,why,n=_check_route(model,data,qids,limits,
                                    q[-1],sol["q"],"pitcher")
            if not safe:raise RuntimeError(
                f"Precision-pour segment unsafe t={t:.3f}: {why}")
        q.append(sol["q"].copy())
        guess=sol["q"].copy()
    return times,np.asarray(q)


def _choose_reference(model,data,qids,limits,seed,amplitude,angle,seconds):
    home,_=_find_start(model,data,qids,limits,seed)
    staged=plan_staging(model,data,qids,limits,home,seed)
    stage=staged["pour_cup"]["stage"]
    if stage is None:
        raise RuntimeError("No safe pour cup pre-dock: cannot authorize precision test")
    origin=np.asarray(stage["q_rad"],float)
    # Never misrepresent this free-space 20-40cm-above-cup exercise as
    # an actual surface-proximal latte-art pour.
    times,knots=_small_cartesian_demo(model,data,qids,limits,origin,
                      amplitude,angle,seconds)
    # Use REAL locally planned Ruckig path, rather than the legacy
    # stop-to-zero-between-all-camera-frames spline.
    candidate=None
    for slowdown in (1.,1.5,2.,3.,5.,8.):
        try:
            rows,sections=_ruckig_sections(times,knots,
                                           velocity_scale=slowdown,dt=.0125)
            # Check all actual sampled joints+interpolated segments.
            for a,b in zip(rows,rows[1:]):
                safe,why,_=_check_route(model,data,qids,limits,
                         np.asarray(a["q_rad"]),np.asarray(b["q_rad"]),
                         "pitcher")
                if not safe:
                    raise RuntimeError("Ruckig swept path: "+why)
            candidate=(rows,sections,slowdown)
            break
        except RuntimeError as exc:
            print("Precision retime",slowdown,"blocked:",str(exc),flush=True)
    if candidate is None:
        raise RuntimeError("No safe local Ruckig path for precision-pour gesture")
    return candidate,staged["pour_cup"]["stage"]


def _interpolator(rows):
    times=np.array([row["time_s"] for row in rows],float)
    qs=np.array([row["q_rad"] for row in rows],float)
    vs=np.array([row["velocity_rad_s"] for row in rows],float)
    if np.any(np.diff(times)<=0):
        raise ValueError("Ruckig trajectory has duplicate/nonmonotonic times")
    cs=CubicHermiteSpline(times,qs,vs,axis=0)
    def reference(t):
        t=float(np.clip(t,times[0],times[-1]))
        return np.asarray(cs(t),float),np.asarray(cs(t,1),float),np.asarray(cs(t,2),float)
    return reference,float(times[-1])


def _torque(model,actual,inverse,qids,dofs,act_ids,ranges,method,
            target_pos,target_vel,target_acc,error_integral=None):
    """Physical torque reference on true FR5 joints; simulation-only gains."""
    actual_q=np.asarray(actual.qpos[qids],float)
    actual_v=np.asarray(actual.qvel[dofs],float)
    error=target_pos-actual_q
    gravity_vel=np.zeros(6) if method=="legacy_pd" else target_vel
    if method in ("computed_torque","computed_torque_robust"):
        # Full inverse dynamics at current state and specified acceleration.
        # The robust variant raises damping/stiffness and uses bounded
        # integral compensation for constant unknown external wrench.
        if method=="computed_torque_robust":
            integral=(np.asarray(error_integral,float)
                      if error_integral is not None else np.zeros(6))
            acmd=(target_acc+320.0*error
                  +36.0*(target_vel-actual_v)+180.0*integral)
        else:
            acmd=target_acc+90.0*error+18.0*(target_vel-actual_v)
        inverse.qpos[:]=actual.qpos
        inverse.qvel[:]=actual.qvel
        inverse.qacc[:]=0
        inverse.qacc[dofs]=acmd
    else:
        inverse.qpos[:]=actual.qpos
        inverse.qvel[:]=0
        inverse.qacc[:]=0
    mujoco.mj_inverse(model,inverse)
    nominal=np.asarray(inverse.qfrc_inverse[dofs],float).copy()
    if method in ("computed_torque","computed_torque_robust"):
        raw=nominal
    else:
        raw=nominal+POSITION_KP*error+VELOCITY_KD*(gravity_vel-actual_v)
    limited=np.clip(raw,ranges[:,0],ranges[:,1])
    actual.ctrl[act_ids]=limited
    return bool(np.any(abs(raw-limited)>1e-8)),float(np.max(abs(raw)))


def _render(model,data,t,mode,spout_mm,tcp_mm,ref_p,ref_tip):
    rgb,_=render_pair(mujoco.Renderer(model,width=640,height=480),data,
                       "cell_overview") # renderer replaced in run for perf
    return rgb


def _run_one(model,qids,limits,dofs,act_ids,ranges,start_q,
             reference,t_end,method,fps,disturbance_nm=0.,renderer=None):
    data=mujoco.MjData(model)
    ref=mujoco.MjData(model)
    inv=mujoco.MjData(model)
    _set_free_pose(model,data,"rb_pitcher_free",[-10,-10,-10],0)
    _set_free_pose(model,data,"rb_plastic_cup_free",[-10,-10,-10],0)
    data.qpos[qids]=start_q
    data.qvel[:]=0.
    # Deliberately UNKNOWN TO THE INVERSE-DYNAMICS MODEL:
    # external constant wrist torque is a simple model-mismatch stress test,
    # NOT a physically simulated nonrigid handle, liquid or filled pitcher.
    data.qfrc_applied[dofs[4]]=float(disturbance_nm)
    mujoco.mj_forward(model,data)
    body=model.body("rb_cell_tcp").id
    ref.qpos[:]=data.qpos
    mujoco.mj_forward(model,ref)
    elapsed=0.
    controller_last=-1
    frame_next=0.
    tracking=[]
    video=[]
    max_torque=0.
    error_integral=np.zeros(6,float)
    sat_n=0
    fail=None
    while elapsed<=t_end+.18:
        source_time=min(elapsed,t_end)
        q,qd,qdd=reference(source_time)
        if elapsed-controller_last>=CTRL_STEP-1e-8:
            if method=="computed_torque_robust":
                error_integral=np.clip(
                    error_integral+CTRL_STEP*(q-data.qpos[qids]),
                    -.06,.06)
            sat,val=_torque(model,data,inv,qids,dofs,act_ids,ranges,
                            method,q,qd,qdd,error_integral)
            sat_n+=int(sat)
            max_torque=max(max_torque,val)
            controller_last=elapsed
        mujoco.mj_step(model,data)
        elapsed+=model.opt.timestep
        actual_tcp,actual_tip=_tip(data,body)
        desired_tcp,desired_tip=_desired_tcp(model,ref,body,qids,q)
        tcp_err=1000*np.linalg.norm(actual_tcp-desired_tcp)
        spout_err=1000*np.linalg.norm(actual_tip-desired_tip)
        # Fail closed on dynamic link/attached tool collision and joint limits.
        from .robot_replay import _contacts
        from .workcell_constraints import payload_check,TRAVEL_CLEARANCE_M
        ext,internal=_contacts(model,data)
        if ext or internal:
            fail=f"robot contact at {elapsed:.3f}s: {(ext+internal)[0]}"
        R=data.xmat[body].reshape(3,3)
        hits=payload_check(actual_tcp,R,"pitcher",
                           clearance_m=TRAVEL_CLEARANCE_M)
        if hits:fail=f"carried pitcher collision at {elapsed:.3f}s"
        qactual=data.qpos[qids]
        if (np.any(qactual<limits[:,0]) or np.any(qactual>limits[:,1])):
            fail=f"FR5 joint-limit violation at {elapsed:.3f}s"
        if not np.isfinite(data.qpos).all():
            fail="numeric divergence"
        tracking.append(dict(time_s=round(elapsed,4),
            actual_tcp_error_mm=float(tcp_err),
            actual_spout_error_mm=float(spout_err),
            reference_spout_m=desired_tip.tolist(),
            executed_spout_m=actual_tip.tolist(),
            target_velocity_norm_rad_s=float(np.linalg.norm(qd)),
            actual_velocity_norm_rad_s=float(np.linalg.norm(data.qvel[dofs]))))
        if renderer is not None and elapsed+1e-8>=frame_next:
            rgb,_=render_pair(renderer,data,"cell_overview")
            im=Image.fromarray(rgb).resize((960,720),
                                      Image.Resampling.BILINEAR)
            canvas=Image.new("RGB",(960,770),(20,32,41))
            canvas.paste(im,(0,40))
            d=ImageDraw.Draw(canvas)
            d.text((12,12),f"{method}   t={elapsed:.2f}s",
                   fill="white")
            d.text((12,745),
                   f"Wrist={tcp_err:.3f}mm  SPOUT={spout_err:.3f}mm",
                   fill="white")
            video.append(canvas)
            frame_next+=1/fps
        if fail:break
    errors=np.array([r["actual_spout_error_mm"] for r in tracking])
    wrist=np.array([r["actual_tcp_error_mm"] for r in tracking])
    values={
        "status":"PASS" if fail is None else "BLOCKED",
        "controller":method,
        "dynamic_mj_step":True,
        "real_six_torque_actuators":True,
        "max_spout_error_mm":float(max(errors)) if len(errors) else None,
        "mean_spout_error_mm":float(np.mean(errors)) if len(errors) else None,
        "p95_spout_error_mm":float(np.percentile(errors,95)) if len(errors) else None,
        "rms_spout_error_mm":float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
        "max_tcp_error_mm":float(max(wrist)) if len(wrist) else None,
        "mean_tcp_error_mm":float(np.mean(wrist)) if len(wrist) else None,
        "saturated_control_intervals":sat_n,
        "max_unclipped_torque_nm":max_torque,
        "samples":len(tracking),
        "first_safety_violation":fail}
    return values,tracking,video


def run(args):
    if not 0<args.amplitude_mm<=12:
        raise ValueError("Amplitude must be between 0 and 12 mm")
    if args.duration_s<=0:
        raise ValueError("Duration must be positive")
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    model=mujoco.MjModel.from_xml_path(str(
        _add_held_pitcher_robot_variant(Path(args.vendor_dir).resolve())))
    model.opt.timestep=DYN_STEP
    data=mujoco.MjData(model)
    _set_free_pose(model,data,"rb_pitcher_free",[-10,-10,-10],0)
    _set_free_pose(model,data,"rb_plastic_cup_free",[-10,-10,-10],0)
    mujoco.mj_forward(model,data)
    qids,limits=robot_qpos_addresses(model)
    dofs=np.array([int(model.jnt_dofadr[model.joint(n).id])
                   for n in JOINTS],dtype=int)
    act_ids,ranges=_actuators(model)
    (rows,sections,slowdown),stage=_choose_reference(
        model,data,qids,limits,args.seed,args.amplitude_mm,
        args.angle_deg,args.duration_s)
    reference,duration=_interpolator(rows)
    start_q=reference(0)[0]
    comparison={}
    samples={}
    film_frames=[]
    renderer=mujoco.Renderer(model,width=640,height=480)
    try:
        for method in ("legacy_pd","velocity_pd",
                       "computed_torque","computed_torque_robust"):
            result,track,video=_run_one(model,qids,limits,dofs,act_ids,
                     ranges,start_q,reference,duration,method,
                     fps=args.video_fps,
                     disturbance_nm=args.disturbance_nm,
                     renderer=renderer if method=="computed_torque_robust" else None)
            comparison[method]=result
            samples[method]=track
            if method=="computed_torque_robust":film_frames=video
            print("PRECISION",method,json.dumps(result),flush=True)
    finally:
        renderer.close()
    (out/"precision_tracking.json").write_text(json.dumps(samples,indent=1))
    mp4=False
    if film_frames and shutil.which("ffmpeg"):
        folder=out/"frames"
        folder.mkdir(exist_ok=True)
        for j,im in enumerate(film_frames):
            im.save(folder/f"pour_{j:04d}.png")
        subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",
            str(args.video_fps),"-i",str(folder/"pour_%04d.png"),
            "-c:v","libx264","-pix_fmt","yuv420p",
            str(out/"precision_pour.mp4")],check=True)
        mp4=(out/"precision_pour.mp4").stat().st_size>10000
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,8),sharex=True)
    for name,track in samples.items():
        t=[p["time_s"] for p in track]
        axes[0].plot(t,[p["actual_spout_error_mm"] for p in track],label=name)
        axes[1].plot(t,[p["actual_tcp_error_mm"] for p in track],label=name)
    axes[0].set(ylabel="Spout error [mm]",title="Same small Ruckig movement — actual MuJoCo motors")
    axes[1].set(xlabel="Simulated time [s]",ylabel="Wrist TCP error [mm]")
    for a in axes:a.grid(alpha=.25)
    axes[0].legend();fig.tight_layout()
    fig.savefig(out/"precision_controller_comparison.png",dpi=145)
    plt.close(fig)
    # Local-coordinate close-up, so a 4mm pour is visible even though
    # the whole-cell overview video barely shows millimetre motion.
    fig,ax=plt.subplots(figsize=(8,7))
    any_ref=np.asarray(samples["computed_torque_robust"][0]["reference_spout_m"],
                       dtype=float)
    for name,track in samples.items():
        actual=np.array([x["executed_spout_m"] for x in track],float)
        target=np.array([x["reference_spout_m"] for x in track],float)
        if name=="computed_torque_robust":
            ax.plot((target[:,0]-any_ref[0])*1000,
                    (target[:,1]-any_ref[1])*1000,
                    "k--",lw=2,label="Soll-Ausgießerspitze")
        ax.plot((actual[:,0]-any_ref[0])*1000,
                (actual[:,1]-any_ref[1])*1000,lw=1.3,
                label=name)
    ax.set(xlabel="Ausgießer ΔX [mm]",ylabel="Ausgießer ΔY [mm]",
           title="Latte-Art-Präzisionsmodus: reale Simulationsbahn der Ausgießerspitze")
    ax.set_aspect("equal",adjustable="datalim")
    ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out/"precision_spout_path_zoom.png",dpi=145)
    plt.close(fig)
    safe=all(v["status"]=="PASS" for v in comparison.values())
    report={
        "status":"PASS" if safe and mp4 else "FAIL",
        "mode":"offline small amplitude collision checked pre-dock precision-pour rehearsal",
        "pitcher_physically_grasped":False,
        "actual_fluid_latte_art_simulated":False,
        "near_surface_latte_contact_approved":False,
        "time_s":duration,
        "initial_requested_duration_s":args.duration_s,
        "amplitude_x_mm":args.amplitude_mm,
        "amplitude_y_mm":args.amplitude_mm*.6,
        "angle_deg_max_input":args.angle_deg,
        "collision_check":"all reference Ruckig samples, interpolated segments and all actual dynamic steps",
        "ruckig_slowdown_factor":slowdown,
        "ruckig_segments":len(sections),
        "kinematic_pose_source":"small deterministic task trajectory in safe pour-cup pre-dock; not an optical demonstration",
        "spout_local_offset_m":SPOUT_LOCAL_M.tolist(),
        "motor_simulator":"MuJoCo 6 joint torque actuators with mj_step",
        "unmodelled_external_wrist_torque_nm":args.disturbance_nm,
        "methods":comparison,
        "video_created":mp4,
        "warnings":"Virtual joints/tuning/geometry only. Spout is geometry approximated as rigid offset, not calibrated real nozzle. No grip mass, foam or liquid."
    }
    (out/"summary.json").write_text(json.dumps(report,indent=2))
    print("PRECISION SUMMARY:",json.dumps(report,indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="runs/precision_pour")
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--amplitude-mm",type=float,default=4.)
    p.add_argument("--angle-deg",type=float,default=.75)
    p.add_argument("--duration-s",type=float,default=3.)
    p.add_argument("--seed",type=int,default=14)
    p.add_argument("--video-fps",type=int,default=8)
    p.add_argument("--disturbance-nm",type=float,default=0.0,
                   help="Model mismatch stress: apply unknown wrist torque to plant, not to inverse model")
    args=p.parse_args()
    r=run(args)
    import sys
    sys.stdout.flush();sys.stderr.flush()
    if os.environ.get("MUJOCO_GL","").lower()=="osmesa":
        os._exit(0 if r["status"]=="PASS" else 2)
    if r["status"]!="PASS":
        raise SystemExit(2)


if __name__=="__main__":
    main()
