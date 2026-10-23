"""Actual six-joint FR5 MuJoCo front-side approach demonstration.

Two independent, measured robot wrist poses:
 - mug: orthogonal IR40 approaches along horizontal +X, jaws close +/-Y;
 - pitcher: axial IR40 approaches horizontal -X, jaws close +/-Y.

CRITICAL: Pickup is a VISUAL STATE SWITCH at a pre-grasp pose (free -> held).
There is no MuJoCo gripper actuation, contact force, sensing or controller.
Arm IK/limiting geometry is checked, but entire inactive-tool swept 3D
collision geometry is NONCOLLIDING and cannot be certified here.
"""
from __future__ import annotations
import argparse,json,os,subprocess
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
import mujoco

from .coffee_workflow import _story_scene,_alpha
from .camera_joint_replay import _find_start
from .robot_replay import compute_ik,robot_qpos_addresses
from .safe_staging import _valid_state,_check_route
from .cell_demo import _set_free_pose
from .geometry import look_at_quat


def pose_at(model,data,qids,q):
    data.qpos[qids]=q
    mujoco.mj_forward(model,data)
    i=model.body("rb_cell_tcp").id
    return data.xpos[i].copy(),data.xmat[i].reshape(3,3).copy()


def _solve(model,data,qids,limits,p,R,seeds,tool):
    for seed in seeds:
        ans=compute_ik(model,data,model.body("rb_cell_tcp").id,
                       qids,limits,p,R,seed,max_evals=100)
        if ans["solved"] and ans["position_error_mm"]<2:
            ok,why=_valid_state(model,data,qids,limits,ans["q"],tool)
            if ok:return ans
    return None


def find_side_approach(model,data,qids,limits,qhome,mode):
    """Fail closed if a genuine horizontal-side IK pose cannot be found."""
    if mode=="cup":
        r=np.eye(3)
        p0,_=pose_at(model,data,qids,qhome)
        candidates=[p0,p0+np.array([.05,.02,0]),p0+np.array([0,.04,0])]
    else:
        r=Rotation.from_euler("y",-90,degrees=True).as_matrix()
        p0,_=pose_at(model,data,qids,qhome)
        candidates=[p0+np.array([0,.06,0]),p0,p0+np.array([.10,.08,.03]),
                    [-.21,-.40,1.32],[.11,-.40,1.35],[-.19,-.51,1.30],
                    [.10,-.40,1.26],[-.10,-.42,1.23]]
    target=None
    # Avoid using global visual cup pose or simulator held ground truth
    # to prescribe robot joints; this is a deterministic IK demo.
    for p in candidates:
        ans=_solve(model,data,qids,limits,np.asarray(p,float),r,
                   [qhome,np.array([0,-.7,1.1,-1.1,.8,0]),
                    np.array([-1,-.7,1.2,-1.0,.8,0]),
                    np.array([1,-.7,1.2,-1.0,-.8,0])],
                   "pitcher" if mode=="pitcher" else "cup")
        if ans is None:continue
        qend=ans["q"]
        approach_axis=np.array([1.,0.,0.]) if mode=="cup" else np.array([-1.,0.,0.])
        for travel in (.060,.045,.025):
            # Pre-grasp lies upstream along the tool's horizontal axis.
            pre=np.asarray(p,float)-approach_axis*travel
            before=_solve(model,data,qids,limits,pre,r,[qend,qhome],
                          "pitcher" if mode=="pitcher" else "cup")
            if before is None:continue
            good,why,n=_check_route(model,data,qids,limits,
                                    before["q"],qend,
                                    "pitcher" if mode=="pitcher" else "cup")
            if not good:continue
            target=(before["q"],qend,pre,np.asarray(p,float),travel,n)
            break
        if target is not None:break
    if target is None:
        raise RuntimeError(mode+": no safe horizontal-side approach pose; no fake underside fallback")
    return target


def _axes(model,data,mode):
    grip=model.body("rb_90_ir40_cup" if mode=="cup" else "rb_90_ir40_handle").id
    R=data.xmat[grip].reshape(3,3)
    return R[:,2].copy(),R[:,0].copy()


def render(args):
    out=Path(args.output).resolve()
    out.mkdir(exist_ok=True,parents=True)
    model=mujoco.MjModel.from_xml_path(str(_story_scene(Path(args.vendor_dir),
                                                  cup_style="handle_mug")))
    data=mujoco.MjData(model)
    _set_free_pose(model,data,"rb_pitcher_free",[-10,-10,-10],0)
    _set_free_pose(model,data,"rb_plastic_cup_free",[-10,-10,-10],0)
    mujoco.mj_forward(model,data)
    qids,limits=robot_qpos_addresses(model)
    home,_=_find_start(model,data,qids,limits,args.seed)

    renderer=mujoco.Renderer(model,width=args.width,height=args.height)
    eye_body=model.body("rb_90_preview_camera_rig").id
    mocap=int(model.body_mocapid[eye_body])
    imgs=[]
    result=[]
    try:
        for mode in ("cup","pitcher"):
            a,b,pa,pb,distance,checks=find_side_approach(
                model,data,qids,limits,home,mode)
            pose_at(model,data,qids,b)
            tool,closing=_axes(model,data,mode)
            # Geometry MUST be truly horizontal in the chosen FR5 IK state.
            assert abs(tool[2])<.035,(mode,"vertical approach",tool.tolist())
            assert abs(closing[2])<.035,(mode,"vertical clamping",closing.tolist())
            if mode=="cup":
                vessel_id=model.geom("rb_gripped_cup_body").id
            else:
                vessel_id=model.geom("rb_gripped_pitcher_body").id
            bowl=data.geom_xpos[vessel_id].copy()
            # Non-physical free->held handoff, approximate free body baseline.
            if mode=="cup":
                free_joint="rb_plastic_cup_free"
                centre=bowl-np.array([0,0,.052])
            else:
                free_joint="rb_pitcher_free"
                centre=bowl-np.array([0,0,.060])

            # The free mug/pitcher sits at a STATIC observed target; robot
            # moves by 6-joint IK toward its handle. At the last 2 frames
            # this free object is swapped for the visual rigid holder.
            for i,t in enumerate(np.linspace(0,1,args.frames_per_tool)):
                # cubic position easing on real robot joint qpos
                e=t*t*(3-2*t)
                q=(1-e)*a+e*b
                ok,reason=_valid_state(model,data,qids,limits,q,
                                        "pitcher" if mode=="pitcher" else "cup")
                if not ok: raise RuntimeError(mode+": unsafe trajectory frame "+reason)
                pose_at(model,data,qids,q)
                if i < args.frames_per_tool-2:
                    _alpha(model,"none")
                    _set_free_pose(model,data,free_joint,centre,0)
                else:
                    _alpha(model,"plastic_cup" if mode=="cup" else "pitcher")
                    _set_free_pose(model,data,free_joint,[-10,-10,-10],0)
                mujoco.mj_forward(model,data)
                # Always orient cinema camera to see handles from SIDE,
                # rather than the old robot-above view.
                tcp=data.xpos[model.body("rb_cell_tcp").id].copy()
                focus=tcp+np.array([.07,0,.01]) if mode=="cup" else tcp+np.array([-.08,0,.015])
                eye=focus+np.array([.32,-.62,.20])
                data.mocap_pos[mocap]=eye
                data.mocap_quat[mocap]=look_at_quat(eye,focus)
                mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera="rb_90_preview_closeup")
                im=Image.fromarray(renderer.render().copy())
                canvas=Image.new("RGB",(args.width,args.height+66),(20,29,37))
                canvas.paste(im,(0,47))
                draw=ImageDraw.Draw(canvas)
                label="TASSE – SEITLICH AM HENKEL" if mode=="cup" else "MILCHKANNE – SEITLICH AM GRIFF"
                draw.text((18,12),f"{label} | MuJoCo FR5 IK | {i+1}/{args.frames_per_tool}",fill=(235,245,248))
                draw.text((18,args.height+51),"Symbolischer Objektwechsel – KEIN realer Griffkontakt / kein Motorservo",
                          fill=(255,209,142))
                imgs.append(canvas)
                if i==args.frames_per_tool-1:
                    canvas.save(out/f"{mode}_side_grasp_mujoco.png")
            result.append({"mode":mode,
                "horizontal_approach_height_component":float(abs(tool[2])),
                "horizontal_finger_closing_height_component":float(abs(closing[2])),
                "robot_ik_end_xyz_m":pb.tolist(),
                "robot_ik_start_xyz_m":pa.tolist(),
                "horizontal_approach_distance_mm":1000*distance,
                "joint_path_checked_steps":checks,
                "physical_grip":False,
                "unused_tool_collision_geometry_validated":False,
                "cup_or_pitcher_station_height_calibrated":False})
            print("SIDE",result[-1],flush=True)
    finally:
        renderer.close()
    from shutil import which
    if not which("ffmpeg"):raise RuntimeError("ffmpeg needed")
    raw=out/"frames";raw.mkdir(exist_ok=True)
    for k,im in enumerate(imgs):im.save(raw/f"frame_{k:04d}.png")
    movie=out/"front_side_grasp_mujoco.mp4"
    subprocess.run(["ffmpeg","-y","-loglevel","error",
        "-framerate",str(args.fps),"-i",str(raw/"frame_%04d.png"),
        "-c:v","libx264","-pix_fmt","yuv420p",str(movie)],check=True)
    summary={"status":"PASS","simulator":"real MuJoCo 6-DOF FR5 joint IK",
        "movie":"front_side_grasp_mujoco.mp4",
        "original_inlux_tool_step_files_imported":False,
        "robot_gripper_closure_simulated":False,
        "real_force_sensor_data":False,
        "swept_unused_gripper_envelope_safe":False,
        "side_approaches":result,
        "video_frames":len(imgs)}
    (out/"summary.json").write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--output",default="runs/side_grasp_preview")
    p.add_argument("--frames-per-tool",type=int,default=10)
    p.add_argument("--fps",type=int,default=5)
    p.add_argument("--width",type=int,default=760)
    p.add_argument("--height",type=int,default=570)
    p.add_argument("--seed",type=int,default=14)
    result=render(p.parse_args())
    if os.getenv("MUJOCO_GL","").lower()=="osmesa":
        import sys
        sys.stdout.flush()
        os._exit(0 if result["status"]=="PASS" else 2)

if __name__=="__main__":
    main()
