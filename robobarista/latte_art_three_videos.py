"""ONE virtual Latte-Art gesture -> three comparable filmed experiments.

1) independent virtual demonstrator pitcher, a stand-in for FUTURE human demo
2) reconstructed 6D pose from three *rendered RGB-D AprilTag cameras*
3) articulated FAIRINO FR5 replay from TRACKED (not source/GT) poses

The same new lateral-follow camera is used everywhere. Tracking inputs come
from camera_joint_replay --latte-art-source --capture-only (three actual
synthetic camera renders and AprilTag detections per sample), NOT fake noise.
Robot replay uses constrained numerical IK in MuJoCo, NOT FAIRINO firmware or
dynamically torque-driven motors. Actual geometrical poses read from MuJoCo.
Metrics compare observed / replay against withheld generated demonstrator.

20 seconds / 8 fps; source tracker capture 61 frames @ 3fps.
"""
from __future__ import annotations
import argparse,json,os,subprocess,shutil
from pathlib import Path
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation,RotationSpline
from PIL import Image,ImageDraw,ImageFont
from .camera_joint_replay import _find_start,_safe_q_segment
from .robot_replay import robot_qpos_addresses,compute_ik
from .coffee_workflow import _story_scene,_alpha
from .cell_demo import _set_free_pose
from .follow_camera import ArmFollowCamera,CAMERA_NAME
from .scene import SPOUT_TIP_LOCAL_M


def font(n):
    for name in ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"]:
        try:return ImageFont.truetype(name,n)
        except OSError:pass
    return ImageFont.load_default()


def load_capture(folder):
    folder=Path(folder)
    gt=json.loads((folder/"ground_truth_evaluation_only.json").read_text())
    observed=json.loads((folder/"camera_measurements.json").read_text())
    if len(gt)!=len(observed) or len(gt)<20:
        raise ValueError("Capture must have >=20 matching truth/camera frames")
    if any(not item["valid"] or item["visible_views"]<2 for item in observed):
        raise RuntimeError("Insufficient AprilTag cameras; do not fill missing pose with GT")
    for a,b in zip(gt,observed):
        if abs(float(a["time_s"])-float(b["time_s"]))>1e-8:
            raise RuntimeError("Mismatched source/tracking time")
    return gt,observed


def interpolate(rows):
    t=np.array([p["time_s"] for p in rows],float)
    p=np.asarray([p["position_m"] for p in rows],float)
    rot=Rotation.from_quat([x["rotation_xyzw"] for x in rows])
    if not np.all(np.diff(t)>0) or not np.all(np.isfinite(p)):
        raise ValueError("Bad trajectory measurements")
    spl=CubicSpline(t,p,axis=0)
    rs=RotationSpline(t,rot)
    return spl,rs


def pose(data,model,name,p,R):
    jid=model.joint(name).id
    adr=int(model.jnt_qposadr[jid])
    data.qpos[adr:adr+3]=p
    q=Rotation.from_matrix(R).as_quat()
    data.qpos[adr+3:adr+7]=[q[3],q[0],q[1],q[2]]
    mujoco.mj_forward(model,data)


def tip(p,R):
    return np.asarray(p)+R@SPOUT_TIP_LOCAL_M


def metrics(samples,prefix):
    errors=np.asarray([v["error_3d_mm"] for v in samples],float)
    return {prefix+"_mean_mm":float(np.mean(errors)),
            prefix+"_max_mm":float(np.max(errors)),
            prefix+"_p95_mm":float(np.percentile(errors,95)),
            prefix+"_rmse_mm":float(np.sqrt(np.mean(errors**2)))}


def write_plot(rows,title,path):
    """Same axes: true 3D spatial paths, Cartesian errors, and one scalar mm."""
    t=np.array([o["time_s"] for o in rows])
    coords=np.array([o["delta_xyz_mm"] for o in rows],float)
    errors=np.array([o["error_3d_mm"] for o in rows],float)
    source=np.array([o["original_tip_xyz_m"] for o in rows],float)*1000.
    measured_key=("tracked_tip_xyz_m" if "tracked_tip_xyz_m" in rows[0]
                  else "simulated_arm_tip_xyz_m")
    measured=np.array([o[measured_key] for o in rows],float)*1000.
    baseline=source[0].copy()
    source-=baseline
    measured-=baseline
    fig=plt.figure(figsize=(15,5.5))
    spatial=fig.add_subplot(131,projection="3d")
    spatial.plot(source[:,0],source[:,1],source[:,2],lw=2,
                 label="ORIGINAL",color="tab:blue")
    spatial.plot(measured[:,0],measured[:,1],measured[:,2],ls="--",lw=1.5,
                 label="TRACKING / FR5",color="tab:orange")
    spatial.set(xlabel="X [mm]",ylabel="Y [mm]",zlabel="Z [mm]",
                title="Beide Ausgiesserpfade in 3D")
    spatial.legend(fontsize=8)
    cart=fig.add_subplot(132)
    for i,letter in enumerate("XYZ"):
        cart.plot(t,coords[:,i],label=f"Delta {letter} [mm]")
    cart.set(xlabel="Zeit [s]",ylabel="Differenz [mm]",
             title="Koordinatenfehler X/Y/Z")
    cart.legend(fontsize=8)
    scalar=fig.add_subplot(133)
    scalar.plot(t,errors,label="Euklidischer 3D-Abstand",color="tab:orange")
    scalar.set(xlabel="Zeit [s]",ylabel="Fehlerbetrag [mm]",
               title="Einfacher Abweichungswert")
    scalar.legend(fontsize=8)
    for ax in (cart,scalar):ax.grid(alpha=.23)
    fig.suptitle(title+" gegen dieselbe virtuelle Originalbewegung")
    fig.tight_layout();fig.savefig(path,dpi=150);plt.close(fig)


def panel(img,scene,phase,t,err,delta,extra=None,raw_cam=None):
    frame=Image.new("RGB",(960,815),(15,26,34))
    bg=Image.fromarray(img).resize((960,720),Image.Resampling.BILINEAR)
    frame.paste(bg,(0,37))
    d=ImageDraw.Draw(frame)
    d.rectangle((0,0,960,38),fill=(14,25,36))
    d.text((15,9),f"{phase}  | {t:05.2f} / 20.00 s",font=font(18),fill="white")
    d.rectangle((0,756,960,815),fill=(14,25,36))
    if err is None:
        d.text((15,776),"ORIGINAL: synthetische Referenzgeste, kein Mensch aufgenommen",
            font=font(16),fill=(230,224,211))
    else:
        d.text((15,766),f"Delta X {delta[0]:+.3f} | Y {delta[1]:+.3f} | Z {delta[2]:+.3f} mm",
            font=font(16),fill=(159,220,242))
        d.text((15,790),f"Gesamter 3D-Fehler: {err:.3f} mm",font=font(17),fill=(255,212,151))
    if raw_cam is not None:
        inset=raw_cam.resize((270,196),Image.Resampling.BILINEAR)
        frame.paste(inset,(681,48))
        d.rectangle((678,45,953,246),outline="white",width=3)
        d.rectangle((682,48,949,72),fill=(14,25,36))
        d.text((690,52),"3 RGB-D KAMERAS",font=font(15),fill="white")
    if extra:
        d.rectangle((14,47,360,76),fill=(15,26,34))
        d.text((20,52),extra,font=font(16),fill=(254,230,175))
    return frame


def render(args):
    root=Path(args.capture).resolve()
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    gt,seen=load_capture(root)
    original_pos,original_rot=interpolate(gt)
    tracked_pos,tracked_rot=interpolate(seen)
    scene=_story_scene(Path(args.vendor_dir))
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    qids,limits=robot_qpos_addresses(model)
    q0,_=_find_start(model,data,qids,limits,14)
    freeid=int(model.jnt_qposadr[model.joint("rb_pitcher_free").id])
    jug=model.body("rb_gripped_pitcher").id
    wrist=model.body("rb_cell_tcp").id
    model.geom("rb_gripped_pitcher_foam").rgba[3]=0.
    data.qpos[qids]=q0
    mujoco.mj_forward(model,data)
    # Calibrated in virtual CAD, not measured hardware: TCP -> gripped pitcher.
    tcpR=data.xmat[wrist].reshape(3,3).copy()
    jugR=data.xmat[jug].reshape(3,3).copy()
    relativeR=tcpR.T@jugR
    relativeP=tcpR.T@(data.xpos[jug]-data.xpos[wrist])
    renderer=mujoco.Renderer(model,width=args.width,height=args.height)
    cam=ArmFollowCamera(model,smoothing=.42)
    frames={m:[] for m in ("original","tracking","robot")}
    records={"tracking":[],"robot":[]}
    times=np.arange(20*args.fps)/args.fps
    current=q0.copy()
    frame_capture=root/"video_frames"
    try:
        for i,t in enumerate(times):
            p0=np.asarray(original_pos(float(t)))
            R0=original_rot(float(t)).as_matrix()
            po=np.asarray(tracked_pos(float(t)))
            Ro=tracked_rot(float(t)).as_matrix()
            truth_tip=tip(p0,R0)
            track_tip=tip(po,Ro)
            delta=(track_tip-truth_tip)*1000
            track_error=float(np.linalg.norm(delta))
            tracked_row={"time_s":float(t),"delta_xyz_mm":delta.tolist(),
                         "error_3d_mm":track_error,
                         "angular_error_deg":float(np.rad2deg(
                           (original_rot(float(t)).inv()*tracked_rot(float(t))).magnitude())),
                         "original_tip_xyz_m":truth_tip.tolist(),
                         "tracked_tip_xyz_m":track_tip.tolist()}
            records["tracking"].append(tracked_row)
            nearest=int(np.argmin(np.abs(np.array([s["time_s"] for s in seen])-t)))
            raw_path=frame_capture/f"frame_{nearest:04d}.png"
            raw_img=Image.open(raw_path).convert("RGB") if raw_path.exists() else None
            shared_eye=None
            shared_quat=None
            for phase in ("original","tracking","robot"):
                if phase=="robot":
                    _set_free_pose(model,data,"rb_pitcher_free",[-10,-10,-10],0)
                    _alpha(model,"pitcher")
                    targetR=Ro@relativeR.T
                    targetP=po-targetR@relativeP
                    sol=compute_ik(model,data,wrist,qids,limits,targetP,targetR,
                                   current,max_evals=145)
                    if not sol["solved"]:
                        raise RuntimeError(f"Robot IK failed {t:.3f}s: "
                                           f"{sol['position_error_mm']:.3f} mm, "
                                           f"{sol['rotation_error_deg']:.3f} deg")
                    valid,reason,_=_safe_q_segment(model,data,qids,limits,
                                                  current,sol["q"])
                    if not valid:raise RuntimeError(f"Robot blocked at {t:.2f}s: {reason}")
                    current=sol["q"].copy()
                    data.qpos[qids]=current
                    mujoco.mj_forward(model,data)
                    actualJ=data.xpos[jug].copy()
                    actualR=data.xmat[jug].reshape(3,3).copy()
                    measured=tip(actualJ,actualR)
                    difference=(measured-truth_tip)*1000
                    err=float(np.linalg.norm(difference))
                    records["robot"].append({
                        "time_s":float(t),"delta_xyz_mm":difference.tolist(),
                        "error_3d_mm":err,
                        "angular_error_deg":float(np.rad2deg(
                            Rotation.from_matrix(R0.T@actualR).magnitude())),
                        "camera_to_robot_extra_mm":float(np.linalg.norm(
                            (measured-track_tip)*1000)),
                        "original_tip_xyz_m":truth_tip.tolist(),
                        "simulated_arm_tip_xyz_m":measured.tolist(),
                        "joint_q_rad":current.tolist()})
                    # EXACT SAME cinema camera as demonstrator at this t.
                    data.mocap_pos[cam.mocap_id]=shared_eye.copy()
                    data.mocap_quat[cam.mocap_id]=shared_quat.copy()
                    mujoco.mj_forward(model,data)
                    extra="FR5: 6 echte Gelenke, IK auf Kameradaten"
                    raw=None
                else:
                    _alpha(model,"none")
                    data.qpos[qids]=q0
                    pp,RR=(p0,R0) if phase=="original" else (po,Ro)
                    pose(data,model,"rb_pitcher_free",pp,RR)
                    if phase=="original":
                        cam.update(data,target_xyz=pp)
                        shared_eye=data.mocap_pos[cam.mocap_id].copy()
                        shared_quat=data.mocap_quat[cam.mocap_id].copy()
                    else:
                        data.mocap_pos[cam.mocap_id]=shared_eye.copy()
                        data.mocap_quat[cam.mocap_id]=shared_quat.copy()
                        mujoco.mj_forward(model,data)
                    err=None if phase=="original" else track_error
                    difference=None if phase=="original" else delta
                    extra="VIRTUELLE MENSCHEN-DEMONSTRATION" if phase=="original" else "AprilTag-3D-Rekonstruktion"
                    raw=raw_img if phase=="tracking" else None
                renderer.disable_depth_rendering()
                renderer.update_scene(data,camera=CAMERA_NAME)
                rgb=renderer.render().copy()
                im=panel(rgb,scene,phase.upper(),float(t),err,
                         difference,extra=extra,raw_cam=raw)
                folder=out/(phase+"_frames")
                folder.mkdir(exist_ok=True)
                im.save(folder/f"f_{i:04d}.jpg",quality=84)
                frames[phase].append(1)
            if raw_img is not None:raw_img.close()
    finally:
        renderer.close()
    videos={}
    for mode in frames:
        f=out/(mode+"_frames")
        video=out/(mode+"_20s.mp4")
        subprocess.run(["ffmpeg","-y","-loglevel","error",
                        "-framerate",str(args.fps),"-i",str(f/"f_%04d.jpg"),
                        "-vf","pad=ceil(iw/2)*2:ceil(ih/2)*2",
                        "-c:v","libx264","-pix_fmt","yuv420p","-movflags",
                        "+faststart",str(video)],check=True)
        videos[mode]=video.name
    for mode in ("tracking","robot"):
        (out/(mode+"_errors.json")).write_text(json.dumps(records[mode],indent=2))
        write_plot(records[mode],mode.upper()+" vs. Original",
                   out/(mode+"_xyz_and_3d_error.png"))
    ts=records["tracking"]
    rs=records["robot"]
    summary={"status":"PASS","total_duration_seconds":20,"video_fps":args.fps,
        "frames_per_video":len(times),"camera_measurement_frames":len(seen),
        "three_rendered_rgbd_apriltag_cameras":True,
        "source_is_virtual_human_like_demo_not_actual_human":True,
        "tracking_errors_from_actual_camera_estimates":True,
        "tracking_and_robot_compared_to_same_generated_original":True,
        "robot_actuation":"checked pose-level IK/kinematic MuJoCo FR5 replay, NOT FAIRINO firmware",
        "mujoco_motor_torque_dynamics":False,
        "same_lateral_follow_camera_in_all_videos":True,
        "gripper_jaws_and_fixtures_conceptual_not_force_validated":True,
        "physical_milk_flow_simulated":False,
        "tip_measurement":"same virtual SPOUT_TIP_LOCAL_M rigid offset for all 3 stages",
        "tracking":metrics(ts,"tracking"),
        "robot":metrics(rs,"robot"),
        "mean_additional_camera_to_arm_error_mm":float(np.mean(
            [v["camera_to_robot_extra_mm"] for v in rs])),
        "videos":videos,
        "tip_target_max_excursion_mm":float(max(
           1000*np.linalg.norm(np.asarray(v["original_tip_xyz_m"])-
                    np.asarray(rs[0]["original_tip_xyz_m"])) for v in rs))}
    (out/"summary.json").write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def main():
    a=argparse.ArgumentParser()
    a.add_argument("--capture",required=True)
    a.add_argument("--output",default="runs/latte_three_videos")
    a.add_argument("--vendor-dir",default="vendor")
    a.add_argument("--fps",type=int,default=8)
    a.add_argument("--width",type=int,default=640)
    a.add_argument("--height",type=int,default=480)
    args=a.parse_args()
    if args.fps not in (8,10,12):raise ValueError("fps must be 8/10/12")
    report=render(args)
    import sys
    sys.stdout.flush()
    if os.getenv("MUJOCO_GL","").lower()=="osmesa":
        os._exit(0 if report["status"]=="PASS" else 2)

if __name__=="__main__":main()
