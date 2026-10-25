"""One FR5 -> fixed portafilter stand -> Swift Mini; NO camera tracking.

Uses the approved FAIRINO six-joint MuJoCo model and existing axial IR40
portafilter attachment geometry. Numerical FR5 FK/IK drives the robot joints.
This is NOT a physical finger-contact pickup: the free prop is swapped into
the model's kinematically attached gripper at an aligned pickup frame.
The grinder position, bayonet geometry and right twist are uncalibrated.
Upward force is NOT controlled with this IK-only robot replay.
"""
from __future__ import annotations

import argparse
import copy
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from .coffee_workflow import _story_scene
from .robot_replay import compute_ik, robot_qpos_addresses, _contacts
from .swift_mini_visual_asset import add_swift_mini_proxy, APPROX_PORTAFILTER_ENTRY_XYZ_M
from .swift_mini_portafilter_only import RIGHT_TWIST_DEG
from .workcell_constraints import TRAVEL_CLEARANCE_M
from .geometry import look_at_quat

GRINDER_XYZ = np.array([-.42, -.12, .82])
RECEIVER_XYZ = GRINDER_XYZ + np.asarray(APPROX_PORTAFILTER_ENTRY_XYZ_M)
HOLDER_BASKET_XYZ = np.array([-.20,-.33,.925])
BASKET_SITE = "rb_gripped_portafilter_basket"
GRASP_PROXY = "rb_gripped_portafilter"
PARKED_COPY = "rb_parked_demo_portafilter"
PARKED_GEOMS = (
    "rb_gripped_portafilter_adapter","rb_gripped_portafilter_handle",
    "rb_gripped_portafilter_basket",
)
PREGRASP_LIFT_M = .055
FREE_SPACE_LIFT_M = .13
INSERT_LIFT_M = .035
MAX_PROVISIONAL_RIGHT_DEG = RIGHT_TWIST_DEG


def vec(a):
    return " ".join(f"{float(x):.9g}" for x in a)


def _hide_world_except_scene(root):
    world=root.find("worldbody")
    for e in list(world):
        if e.tag=="light":
            continue
        if e.tag=="geom" and e.get("name") in ("rb_cell_floor","rb_island_top","rb_robot_pedestal"):
            continue
        world.remove(e)
    for cam in root.findall(".//camera"):
        parent=next((p for p in root.iter() if cam in list(p)),None)
        if parent is not None:parent.remove(cam)
    return world


def build_robot_holder_swift_scene(vendor_dir=Path("vendor"),grasp_heading_deg=0.):
    scene_path=_story_scene(Path(vendor_dir),cup_style="handle_mug")
    scene=ET.parse(scene_path)
    root=scene.getroot()
    folder=Path(scene_path).resolve().parent
    robot_file=folder / root.find("include").get("file")
    robot=ET.parse(robot_file)
    rroot=robot.getroot()
    for cam in list(rroot.findall(".//camera")):
        parent=next((p for p in rroot.iter() if cam in list(p)),None)
        if parent is not None:parent.remove(cam)
    axial=rroot.find(".//body[@name='rb_90_ir40_handle']")
    assert axial is not None, "Must use current axial 90deg IR40 tool"
    pf=axial.find("body[@name='"+GRASP_PROXY+"']")
    assert pf is not None, "Must re-use current gripped portafilter prototype"
    # The old parked display independently drew its handle 14cm behind the
    # basket with a 17mm GAP. The single held proxy had no obvious sleeve.
    # Use the same basket-relative geoms at the stand and in the gripper.
    # Geometry is a VISUAL concept, not scanned OEM portafilter CAD.
    ET.SubElement(pf,"geom",name="rb_gripped_portafilter_custom_IR40_grip_sleeve",
         type="box",pos="-.085 0 -.005",size=".064 .018 .018",
         rgba=".13 .52 .54 1",mass="0",contype="0",conaffinity="0")
    ET.SubElement(pf,"geom",name="rb_gripped_portafilter_grip_stop",
         type="box",pos="-.019 0 -.005",size=".007 .025 .024",
         rgba=".94 .69 .16 1",mass="0",contype="0",conaffinity="0")
    ET.SubElement(pf,"geom",name="rb_gripped_portafilter_neck_bridge",
         type="box",pos="-.143 0 -.005",size=".028 .021 .017",
         rgba=".65 .69 .72 1",mass="0",contype="0",conaffinity="0")
    # No other held props. This scene contains ONLY grinder, holder and
    # the one portafilter kinematically exchanged at a frame.
    for e in list(axial):
        if e.tag=="body" and e is not pf:axial.remove(e)
    lateral=rroot.find(".//body[@name='rb_90_ir40_cup']")
    if lateral is not None:
        for e in list(lateral):
            if e.tag=="body" and e.get("name")=="rb_gripped_cup":
                lateral.remove(e)
    tcp=rroot.find(".//body[@name='rb_cell_tcp']")
    for child in list(tcp):
        if child.tag=="body" and child.get("name","").startswith("rb_story_"):
            tcp.remove(child)
        if child.tag=="geom" and child.get("name","").startswith("rb_story_"):
            tcp.remove(child)
    pf_out=folder/"rb_minimal_swift_fr5_90_robot.xml"
    robot.write(pf_out,encoding="unicode")
    root.find("include").set("file",pf_out.name)
    world=_hide_world_except_scene(root)
    # The removed original grinder contained a powered tamper joint. Delete
    # its scene actuator as well; otherwise MuJoCo rejects the dangling target.
    for act in list(root.findall("./actuator")):
        for motor in list(act):
            if motor.get("joint")=="rb_grinder_tamper_slide":
                act.remove(motor)
        if not len(act):root.remove(act)
    grinder=add_swift_mini_proxy(world,xyz=GRINDER_XYZ)
    # Fixed portafilter stand with open cradle, two lateral supports. Real
    # physical clamps/contact remain unmodeled; exact repeatable fixture!
    base=ET.SubElement(world,"body",{
        "name":"rb_pf_stand_fixed_3point", "pos":vec(HOLDER_BASKET_XYZ)
    })
    def show(parent,name,kind,pos,size,rgba):
        return ET.SubElement(parent,"geom",name=name,type=kind,
          pos=vec(pos),size=vec(size),rgba=rgba,mass="0",
          contype="0",conaffinity="0")
    show(base,"rb_pf_holder_plate","box",(0,0,-.057),(.092,.069,.009),".22 .27 .31 1")
    for sign in (-1,1):
        show(base,f"rb_pf_holder_rail_{sign}","box",
             (sign*.049,0,-.026),(.009,.059,.026),".20 .55 .57 1")
    show(base,"rb_pf_holder_backstop","box",(0,.047,-.025),
         (.047,.007,.027),".45 .50 .54 1")
    ET.SubElement(base,"site",name="rb_pf_holder_repeatable_pickup",
                  pos="0 0 0",size=".005",rgba=".2 1 .7 1")
    # Static copy of the *same held portafilter geometry*. Its basket center
    # is aligned with holder_basket_xyz by a transform applied below.
    parked=ET.SubElement(world,"body",{
        "name":PARKED_COPY, "pos":vec(HOLDER_BASKET_XYZ)
    })
    parked.set("euler","0 0 "+str(float(np.deg2rad(grasp_heading_deg))))
    # Reuse handle's shape, but local basket center is the spatial origin.
    bowl=pf.find("geom[@name='"+BASKET_SITE+"']")
    assert bowl is not None
    bowl_loc=np.fromstring(bowl.get("pos"),sep=" ")
    for g in list(pf.findall("geom")):
        if not g.get("name","").startswith("rb_gripped_portafilter_"):continue
        part=copy.deepcopy(g)
        part.set("name","rb_pf_stand_"+g.get("name"))
        xyz=np.fromstring(part.get("pos"),sep=" ")
        # Preserve EXACT original basket-relative rigid geometry at the
        # parking fixture; cannot leave a detached 17-mm visual gap.
        part.set("pos",vec(xyz-bowl_loc))
        parked.append(part)
    # Put a clear model pivot on the IR40 sleeve instead of inventing another
    # portafilter. This is geometry-only; grasp contact remains unmodeled.
    # An intentionally approximate thin insertion reference plane with
    # NO collision, so it cannot be mistaken for OEM compliant bayonet.
    ring=ET.SubElement(world,"body",name="rb_swift_mini_insertion_plane_proxy",
                       pos=vec(RECEIVER_XYZ))
    show(ring,"rb_swift_mini_visual_port_entry","cylinder",
         (0,0,0),(.049,.0025),".03 .9 .8 .23")
    ET.SubElement(ring,"site",name="rb_swift_mini_PORT_UNMEASURED",
                  pos="0 0 0",size=".004",rgba="1 .6 .13 1")
    out=folder/"rb_minimal_fr5_pick_holder_swift.xml"
    scene.write(out,encoding="unicode")
    return out


def T(position,rotation):
    t=np.eye(4)
    t[:3,:3]=np.asarray(rotation).reshape(3,3)
    t[:3,3]=np.asarray(position).reshape(3)
    return t


def read_tcp_to_basket(model,data):
    """Actual transform from current 90deg IR40 TCP to portafilter basket."""
    mujoco.mj_forward(model,data)
    tcp=model.body("rb_cell_tcp").id
    bowl=model.geom(BASKET_SITE).id
    tcp_world=T(data.xpos[tcp],data.xmat[tcp])
    bowl_world=T(data.geom_xpos[bowl],data.geom_xmat[bowl])
    return np.linalg.inv(tcp_world)@bowl_world


def _tcp_target_from_basket(basket_xyz,basket_R,tcp_to_basket):
    return T(basket_xyz,basket_R)@np.linalg.inv(tcp_to_basket)


def _check_robot_kinematic_step(model,data,qids,q,require_no_contacts=True):
    data.qpos[qids]=q
    mujoco.mj_forward(model,data)
    if require_no_contacts:
        external,internal=_contacts(model,data)
        if external or internal:
            raise RuntimeError("Robot MuJoCo geometry contact: "+
                               str((external[:2],internal[:2])))
    R=data.geom_xmat[model.geom(BASKET_SITE).id].reshape(3,3)
    vertical=R[:,2]
    if np.rad2deg(np.arccos(np.clip(vertical[2],-1,1)))>1.0:
        raise RuntimeError("Portafilter basket tilted, fail closed")


def _points(start,end,n):
    start=np.asarray(start,float);end=np.asarray(end,float)
    for u in np.linspace(0,1,n):
        s=u*u*(3-2*u)
        yield start+(end-start)*s


def plan_one_arm_demo(model,data,*,samples_per_segment=5,allow_right=True,grasp_heading_deg=0.):
    if model.ncam!=0:
        raise AssertionError("No camera bodies allowed in camera-free demo")
    qids,limits=robot_qpos_addresses(model)
    tcp=model.body("rb_cell_tcp").id
    toe=model.geom(BASKET_SITE).id
    tcp_to_basket=read_tcp_to_basket(model,data)
    # Stand and source target both define basket Z=up, which enforces
    # portafilter level. This is a planned FIXTURE pose, NOT a video estimate.
    angle0=0.
    neutral=Rotation.from_euler("z",grasp_heading_deg,degrees=True).as_matrix()
    # If local basket basis with R=I requires an unreachable tool orientation
    # the attempt FAILS; no fake teleport.
    recipe=[
      ("APPROACH_HOLDER", HOLDER_BASKET_XYZ+np.array([0,0,PREGRASP_LIFT_M]),0.0),
      ("ALIGN_HANDLE", HOLDER_BASKET_XYZ.copy(),0.0),
      ("GRASP_VISUAL_SWITCH", HOLDER_BASKET_XYZ.copy(),0.0),
      ("LIFT_FROM_STAND", HOLDER_BASKET_XYZ+np.array([0,0,FREE_SPACE_LIFT_M]),0.0),
      ("TRANSFER_ABOVE_SWIFT", RECEIVER_XYZ+np.array([0,0,.052]),0.0),
      ("ALIGN_BELOW_SWIFT", RECEIVER_XYZ-np.array([0,0,.065]),0.0),
      ("LIFT_INTO_RECEIVER",RECEIVER_XYZ-np.array([0,0,.004]),0.0),
      ("OPTIONAL_RIGHT_TWIST", RECEIVER_XYZ-np.array([0,0,.004]),MAX_PROVISIONAL_RIGHT_DEG if allow_right else 0.0),
    ]
    # Transfer should stay ABOVE equipment to avoid straight diagonal
    # penetration; the through-machine path is NOT approved by this test.
    # We attempt it honestly and flag keepout uncertainty in report.
    seeds=[
      np.array([0,-.7,1.1,-1.1,.8,0.]),
      np.array([-1,-.7,1.2,-1.,.8,0.]),
      np.array([1,-.7,1.2,-1.,-.8,0.]),
      np.array([-2.,-.9,1.3,-1.1,1.0,0.]),
      np.array([2.,-.9,1.3,-1.1,-1.,0.]),
    ]
    q_prev=None;attempts=[];samples=[];failures=[]
    for i,(label,center,yaw_deg) in enumerate(recipe):
        if label=="OPTIONAL_RIGHT_TWIST" and yaw_deg>0 and q_prev is not None:
            # A straight interpolation between two IK branches passed through
            # FR5 base/finger collision. This movement MUST follow continuous
            # 1-degree CARTESIAN yaw setpoints with fresh six-joint IK.
            for delta_yaw in np.linspace(0,yaw_deg,int(np.ceil(yaw_deg))+1)[1:]:
                Rinc=Rotation.from_euler("z",delta_yaw,degrees=True).as_matrix()@neutral
                target_inc=_tcp_target_from_basket(center,Rinc,tcp_to_basket)
                candidates=[q_prev,*seeds]
                picked=None
                for cand in candidates:
                    ik=compute_ik(model,data,tcp,qids,limits,
                           target_inc[:3,3],target_inc[:3,:3],cand,max_evals=145)
                    attempts.append({"step":label,"yaw_deg":float(delta_yaw),
                         "solved":ik["solved"],"position_error_mm":ik["position_error_mm"],
                         "rotation_error_deg":ik["rotation_error_deg"]})
                    if not ik["solved"]:continue
                    q=np.asarray(ik["q"]).copy()
                    if np.max(np.abs(q-q_prev))>.18:
                        attempts[-1]["reject_reason"]="joint jump > 0.18rad per 1deg";
                        continue
                    try:
                        for stepq in _points(q_prev,q,4):
                            _check_robot_kinematic_step(model,data,qids,stepq,True)
                    except RuntimeError as err:
                        attempts[-1]["reject_reason"]=str(err)
                        continue
                    picked=q
                    break
                if picked is None:
                    failures.append({"phase":label,"yaw_deg":float(delta_yaw),
                                     "reason":"Safe incremental wrist yaw unavailable",
                                     "attempts":attempts[-len(candidates):]})
                    break
                for stepq in _points(q_prev,picked,4):
                    samples.append({"phase":label,"q_rad":stepq.tolist(),
                        "picked":True,"basket_target_xyz_m":center.tolist(),
                        "yaw_target_deg":float(delta_yaw)})
                q_prev=picked
            if failures:break
            continue
        R=Rotation.from_euler("z",yaw_deg,degrees=True).as_matrix()@neutral
        target=_tcp_target_from_basket(center,R,tcp_to_basket)
        selected=None
        for seed in ([q_prev] if q_prev is not None else [])+seeds:
            ik=compute_ik(model,data,tcp,qids,limits,target[:3,3],target[:3,:3],
                          seed,max_evals=130)
            attempts.append({"step":label,"position_error_mm":ik["position_error_mm"],
                "rotation_error_deg":ik["rotation_error_deg"],"solved":ik["solved"]})
            if not ik["solved"]:
                continue
            try:
                _check_robot_kinematic_step(model,data,qids,ik["q"],True)
            except RuntimeError as err:
                attempts[-1]["reject_reason"]=str(err)
                continue
            selected=np.asarray(ik["q"]).copy()
            break
        if selected is None:
            failures.append({"phase":label,"reason":"No collision-free 6D FR5 IK","attempts":attempts[-len(seeds)-1:]})
            break
        # Interpolate joint space with collision and basket-level check,
        # including all joints; never move body directly.
        if q_prev is not None:
            steps=max(samples_per_segment,int(np.ceil(np.max(np.abs(selected-q_prev))/.035)))
            for stepq in _points(q_prev,selected,steps):
                try:
                    _check_robot_kinematic_step(model,data,qids,stepq,True)
                except RuntimeError as e:
                    failures.append({"phase":label,"reason":"Joint-space interpolation unsafe","detail":str(e)})
                    break
                samples.append({"phase":label,"q_rad":stepq.tolist(),
                                "picked":i>=3,"basket_target_xyz_m":center.tolist(),
                                "yaw_target_deg":yaw_deg})
            if failures:break
        else:
            samples.append({"phase":label,"q_rad":selected.tolist(),
                            "picked":False,"basket_target_xyz_m":center.tolist(),
                            "yaw_target_deg":yaw_deg})
        q_prev=selected
    status="KINEMATIC_FR5_COMPLETE_PREVIEW_ONLY" if not failures else "BLOCKED"
    return {
      "status":status,
      "six_fr5_joints_used":6,
      "tracking_cameras":model.ncam,
      "portafilter_insertion_target_is_OEM_calibrated":False,
      "portafilter_true_grasp_contact":False,
      "portafilter_right_twist_is_OEM_proven":False,
      "right_twist_command_deg":MAX_PROVISIONAL_RIGHT_DEG if allow_right else 0.,
      "grasp_fixture_heading_deg":float(grasp_heading_deg),
      "robot_force_control_executed":False,
      "robot_ik_kinematic_only":True,
      "holder_basket_xyz_m":HOLDER_BASKET_XYZ.tolist(),
      "grinder_nominal_receiver_xyz_m":RECEIVER_XYZ.tolist(),
      "tool_tcp_to_basket_SE3":tcp_to_basket.tolist(),
      "samples":samples,"ik_attempts":attempts,"first_failure":failures[0] if failures else None,
      "phases_completed":list(dict.fromkeys(s["phase"] for s in samples))
    }


def render(output_dir,*,vendor_dir="vendor",allow_right=True,
           frames_per_segment=5,width=560,height=480,fps=12,
           grasp_heading_deg=0.,west_view=False,insertion_only=False):
    out=Path(output_dir)
    out.mkdir(parents=True,exist_ok=True)
    scene=build_robot_holder_swift_scene(Path(vendor_dir),grasp_heading_deg=grasp_heading_deg)
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    result=plan_one_arm_demo(model,data,samples_per_segment=frames_per_segment,
                             allow_right=allow_right,
                             grasp_heading_deg=grasp_heading_deg)
    result["existing_swift_mini_visual_proxy"]=True
    result["uses_existing_approved_IR40_90deg_gripper"]=True
    result["no_other_coffee_devices_or_tracking_cameras"]=True
    report={k:v for k,v in result.items() if k not in ("samples","ik_attempts")}
    (out/"summary.json").write_text(json.dumps(report,indent=2))
    # A true unreachable/unsafe pose MUST not produce a falsely complete video.
    if result["status"]!="KINEMATIC_FR5_COMPLETE_PREVIEW_ONLY":
        print(json.dumps(report,indent=2),flush=True)
        return report
    # Add purely a render-only fixed *virtual* camera to a second MJCF,
    # AFTER planning the no-camera FR5 path. This cannot feed robot control.
    if west_view:
        camera_scene=ET.parse(scene)
        world=camera_scene.getroot().find("worldbody")
        view_eye=RECEIVER_XYZ+np.array([-.50,-.20,.21])
        # From WEST (-X), 20cm south/front; face right/east (+X).
        # West=quarter-turn around the grinder compared with south view.
        view_focus=RECEIVER_XYZ+np.array([0.,-.065,-.055])
        ET.SubElement(world,"camera",name="rb_WEST_video_view_only",
            pos=vec(view_eye),quat=vec(look_at_quat(view_eye,view_focus)),
            fovy="58")
        rendering_scene=Path(scene).with_name("rb_fr5_swift_WEST_VIEW_RENDER_ONLY.xml")
        camera_scene.write(rendering_scene,encoding="unicode")
        movie_model=mujoco.MjModel.from_xml_path(str(rendering_scene))
        movie_data=mujoco.MjData(movie_model)
        render_qids,_=robot_qpos_addresses(movie_model)
        report["west_view_eye_xyz_m"]=view_eye.tolist()
        report["west_view_focus_xyz_m"]=view_focus.tolist()
        report["west_view_is_only_offline_visualization"]=True
        report["robot_camera_feedback_used"]=False
    else:
        movie_model=model
        movie_data=data
        render_qids=qids if False else robot_qpos_addresses(movie_model)[0]
    renderer=mujoco.Renderer(movie_model,width=width,height=height)
    frame_dir=out/"frames"
    frame_dir.mkdir(exist_ok=True)
    qids=render_qids
    parked_g=[i for i in range(movie_model.ngeom)
              if (mujoco.mj_id2name(movie_model,mujoco.mjtObj.mjOBJ_GEOM,i) or "").startswith("rb_pf_stand_rb_gripped")]
    held_g=[i for i in range(movie_model.ngeom)
            if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or "").startswith("rb_gripped_portafilter")]
    prefix="FR5 WITH PROTOTYPE IR40 HANDLE / NO CAMERA FEEDBACK"
    try:
        # Restrict the West view to the insertion and clockwise twist,
        # not an arbitrary crop or replay of the previous movie.
        selected_samples=([r for r in result["samples"] if r["phase"] in
             ("ALIGN_BELOW_SWIFT","LIFT_INTO_RECEIVER","OPTIONAL_RIGHT_TWIST")]
              if insertion_only else result["samples"])
        report["frames_selected_for_view"]=len(selected_samples)
        for k,record in enumerate(selected_samples):
            movie_data.qpos[qids]=record["q_rad"]
            mujoco.mj_forward(movie_model,movie_data)
            picked=record["picked"]
            for i in parked_g:movie_model.geom_rgba[i,3]=0 if picked else 1
            for i in held_g:movie_model.geom_rgba[i,3]=1 if picked else 0
            if west_view:
                renderer.update_scene(movie_data,camera="rb_WEST_video_view_only")
                frame=Image.fromarray(renderer.render().copy()).resize(
                    (960,960),Image.Resampling.LANCZOS)
                canvas=Image.new("RGB",(960,1038),(19,28,39))
                canvas.paste(frame,(0,42))
                d=ImageDraw.Draw(canvas)
                d.text((15,12),"ECHTER WESTBLICK  |  Arm + IR40 + Siebtraeger",
                    fill=(232,244,249))
                d.text((15,1011),
                    f"{record['phase']} | +X-Blick aus WEST | nur Renderkamera",
                    fill=(251,219,160))
            else:
                cam=mujoco.MjvCamera()
                cam.type=mujoco.mjtCamera.mjCAMERA_FREE
                cam.lookat[:]=[-.31,-.11,1.01]
                cam.azimuth=138;cam.elevation=-20;cam.distance=1.26
                renderer.update_scene(movie_data,camera=cam)
                main=Image.fromarray(renderer.render().copy())
                cam.lookat[:]=RECEIVER_XYZ
                cam.azimuth=145;cam.elevation=-15;cam.distance=.58
                renderer.update_scene(movie_data,camera=cam)
                close=Image.fromarray(renderer.render().copy())
                canvas=Image.new("RGB",(width*2,height+78),(19,28,39))
                canvas.paste(main,(0,40));canvas.paste(close,(width,40))
                d=ImageDraw.Draw(canvas)
                d.text((12,10),prefix,fill=(232,244,249))
                d.text((12,height+51),
                    f"{record['phase']} | GRASP IS VISUAL; IK NOT TORQUE CONTROL",
                    fill=(251,219,160))
            canvas.save(frame_dir/f"fr5_{k:04d}.png")
    finally:
        renderer.close()
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg required for movie")
    movie=out/("FR5_SWIFT_ECHTE_WESTANSICHT.mp4" if west_view else
               "fr5_stand_pick_and_swift_insert_UNCALIBRATED.mp4")
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(fps),
                   "-i",str(frame_dir/"fr5_%04d.png"),"-c:v","libx264",
                   "-pix_fmt","yuv420p",str(movie)],check=True)
    report["mp4"]=movie.name
    report["video_frames"]=len(selected_samples)
    report["connected_portafilter_handle_visual_geometry"]=True
    report["video_file_size"]=movie.stat().st_size
    (out/"summary.json").write_text(json.dumps(report,indent=2))
    (out/"kinematic_joints_NOT_FOR_HARDWARE.json").write_text(json.dumps(result["samples"],indent=2))
    print(json.dumps(report,indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="runs/fr5_swift_stand_programmed")
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--no-right-turn",action="store_true")
    p.add_argument("--grasp-heading",type=float,default=0.)
    p.add_argument("--frames-per-segment",type=int,default=5)
    p.add_argument("--west-view",action="store_true")
    p.add_argument("--render-width",type=int,default=560)
    p.add_argument("--render-height",type=int,default=480)
    p.add_argument("--insertion-only",action="store_true")
    a=p.parse_args()
    result=render(a.output,vendor_dir=a.vendor_dir,
                  allow_right=not a.no_right_turn,
                  frames_per_segment=a.frames_per_segment,
                  grasp_heading_deg=a.grasp_heading,
                  west_view=a.west_view,insertion_only=a.insertion_only,
                  width=a.render_width,height=a.render_height)
    if result["status"]!="KINEMATIC_FR5_COMPLETE_PREVIEW_ONLY":
        raise SystemExit(2)


if __name__=="__main__":
    main()
