"""ONE-ARM FR5 + FIXED-PIVOT single-axis rotary cup holder in MuJoCo.

The holder is an actual MJCF HINGE joint with one angular qpos, not a free
joint, mocap body, or interpolated x/y/z translation. A sampled original
RealMan opposite-arm wrist ORIENTATION may optionally be projected onto
this one axis; this is NOT calibrated original cup inclination and the source
arm assignment is UNKNOWN.

The FR5 uses the existing validated *offline* kinematic joint path, not
motor/force control. Both movie variants contain only symbolic coffee geometry.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from .coffee_workflow import _story_scene, _alpha
from .cell_demo import _set_free_pose
from .geometry import look_at_quat
from .robot_replay import robot_qpos_addresses
from .workcell_constraints import DOCKS
from .real_demo_90_ik import crop_real_episode
from .realman_fk_bridge import parse_urdf_chain, reconstruct_pitches


JOINT = "rb_fixed_pivot_cup_tilt_joint"
BODY = "rb_fixed_pivot_cup_holder"
MAX_TILT_DEG = 18.0
MAX_TILT_RATE_DEG_S = 45.0


def _geom(parent, name, kind, pos, size, rgba, **kw):
    d={"name":name, "type":kind, "pos":pos, "size":size, "rgba":rgba,
       "contype":"0","conaffinity":"0","mass":"0"}
    d.update(kw)
    if "fromto" in kw:
        d.pop("pos", None)  # MuJoCo derives the capsule frame from its endpoints
    return ET.SubElement(parent,"geom",d)


def add_fixed_pivot_one_axis_cup(scene_file, output_file):
    """Add a real hinge, stationary pivot support and modeled cup; never slide."""
    root=ET.parse(scene_file)
    tree=root.getroot()
    world=tree.find("worldbody")
    if world is None or world.find(".//joint[@name='"+JOINT+"']") is not None:
        raise ValueError("No worldbody, or tilt hinge already added")
    pivot=DOCKS["pour_cup"].position_m
    stand=ET.SubElement(world,"body",name="rb_stationary_cup_tilt_base",
                        pos=" ".join(map(str,pivot)))
    _geom(stand,"rb_tilt_stand_base","cylinder","0 0 -.013",".064 .010",".30 .36 .43 1")
    _geom(stand,"rb_tilt_pivot_pin","sphere","0 0 0",".012",".35 .85 .85 1")
    holder=ET.SubElement(stand,"body",name=BODY,pos="0 0 0")
    ET.SubElement(holder,"joint",name=JOINT,type="hinge",
                  axis="0 1 0",pos="0 0 0",
                  limited="true",range="-18 18",damping="0.1")
    # The hinge is a proper moving MuJoCo body: nonzero inertia despite visual-only geoms.
    ET.SubElement(holder,"inertial",pos="0 0 .06",mass=".25",
                  diaginertia=".00023 .00023 .00015")
    _geom(holder,"rb_tilt_cradle","cylinder","0 0 .014",".046 .008",".24 .44 .52 1")
    _geom(holder,"rb_tilt_cup_body","cylinder","0 0 .064",".041 .045",".86 .84 .79 1")
    _geom(holder,"rb_tilt_cup_rim","cylinder","0 0 .109",".044 .003",".94 .93 .88 1")
    _geom(holder,"rb_tilt_cup_espresso_proxy","cylinder","0 0 .108",".039 .001",
          ".35 .17 .07 1")
    _geom(holder,"rb_tilt_cup_latte_pattern_proxy","ellipsoid",
          "0 0 .110",".016 .011 .001","1 1 .92 1")
    for name,coords in [
        ("upper","-.039 0 .083 -.080 0 .083"),
        ("lower","-.039 0 .041 -.080 0 .041"),
        ("outer","-.080 0 .041 -.080 0 .083"),
    ]:
        _geom(holder,"rb_tilt_cup_handle_"+name,"capsule",
              "0 0 0",".006",".91 .88 .82 1",fromto=coords)
    camera_rig=ET.SubElement(world,"body",name="rb_fixed_tilt_camera_rig",
                            pos="0 -1 1.3",mocap="true")
    ET.SubElement(camera_rig,"camera",name="rb_fixed_tilt_closeup",
                  pos="0 0 0",quat="1 0 0 0",fovy="49")
    out=Path(output_file)
    out.parent.mkdir(parents=True,exist_ok=True)
    root.write(out,encoding="unicode")
    return out


def tilt_profile(timestamps, *, mode, real_source_rows=None,
                 urdf_path=None, cup_source_arm="right",
                 source_start_s=8.,source_end_s=12.,
                 angle_start_deg=12.):
    """Single-axis tilt from program or projected original cup-arm wrist.

    All 6D cup-arm translations and rotations other than local hinge Y are
    discarded. No claim about original cup orientation without calibrated
    flange-to-cup transform and verified original arm assignment.
    """
    t=np.asarray(timestamps,float)
    if t.ndim!=1 or len(t)<2 or not np.isfinite(t).all() or not np.all(np.diff(t)>0):
        raise ValueError("Need strictly increasing output times")
    if not 0<=angle_start_deg<=MAX_TILT_DEG:
        raise ValueError("Invalid initial angle")
    if mode=="scripted":
        u=np.clip((t-t[0])/(t[-1]-t[0]),0,1)
        eased=u*u*(3-2*u)
        raw=angle_start_deg*(1-eased)
        meaning="programmed smooth single-axis return to level, NOT RealMan cup"
    elif mode=="projected-source":
        if real_source_rows is None or not urdf_path:
            raise ValueError("Projection needs original joint states and RM65 URDF")
        if cup_source_arm not in ("left","right"):
            raise ValueError("Cup source arm required explicitly")
        rows=crop_real_episode(real_source_rows,source_start_s,source_end_s)
        chain=parse_urdf_chain(urdf_path,"link_6",
                               [f"joint_{j}" for j in range(1,7)])
        poses,_=reconstruct_pitches(rows,chain,arm=cup_source_arm,
                                   joint_unit="rad",
                                   flange_to_pitcher=np.eye(4),
                                   tool_calibrated=False)
        rotations=Rotation.from_quat([p["rotation_xyzw"] for p in poses]).as_matrix()
        origin=rotations[0]
        # Project *relative* source wrist rotation onto the single allowed
        # world hinge axis. This is an engineering approximation, NOT actual
        # source cup mounting geometry or relative source dual-base calibration.
        rel=np.einsum("ij,njk->nik",origin.T,rotations)
        raw_y=np.unwrap(np.arctan2(rel[:,0,2]-rel[:,2,0],
                                    rel[:,0,0]+rel[:,2,2]))
        source_t=np.array([p["time_s"] for p in poses],float)
        query=(t-t[0])*(source_t[-1]/(t[-1]-t[0]))
        raw=angle_start_deg+np.rad2deg(np.interp(query,source_t,raw_y-raw_y[0]))
        meaning=("Uncalibrated relative RM65 wrist Y-axis projection; "
                 "hypothetical opposite arm, NOT confirmed original cup")
    else:
        raise ValueError("Expected scripted or projected-source")
    bounded=np.clip(raw,-MAX_TILT_DEG,MAX_TILT_DEG)
    angles=[float(bounded[0])]
    for i in range(1,len(t)):
        bound=MAX_TILT_RATE_DEG_S*float(t[i]-t[i-1])
        angles.append(float(np.clip(bounded[i],angles[-1]-bound,angles[-1]+bound)))
    result=np.asarray(angles,float)
    if np.any(abs(result)>MAX_TILT_DEG+1e-9):
        raise ValueError("Tilt exceeds mechanical stop")
    return result,meaning


def render_one_arm_fixed_pivot(*, robot_joint_path, output_dir, mode="scripted",
                               original_joints=None, urdf=None,
                               source_cup_arm="right", duration_s=4.,
                               fps=3, width=640, height=480):
    """Make a movie of the approved FR5 joint path plus 1-DOF cup hinge."""
    out=Path(output_dir)
    out.mkdir(parents=True,exist_ok=True)
    if not 1<=fps<=30 or not 0<duration_s<=120:
        raise ValueError("Invalid frame timing")
    qs=np.asarray(json.loads(Path(robot_joint_path).read_text()),float)
    if qs.ndim!=2 or qs.shape[1]!=6 or len(qs)<3 or not np.isfinite(qs).all():
        raise ValueError("Requires COMPLETE prior safe FR5 6-joint path")
    t=np.linspace(0,duration_s,len(qs))
    src=json.loads(Path(original_joints).read_text()) if original_joints else None
    tilt_deg,meaning=tilt_profile(t,mode=mode,real_source_rows=src,
          urdf_path=urdf,cup_source_arm=source_cup_arm)
    base=_story_scene(Path("vendor"),cup_style="handle_mug")
    scene=add_fixed_pivot_one_axis_cup(
        base,Path(base).parent/"rb_fixed_pivot_tilt_onearm.xml")
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    # Original cup becomes non-participating; only hinged replacement is visible.
    _set_free_pose(model,data,"rb_plastic_cup_free",[-10,-10,-10],0)
    _set_free_pose(model,data,"rb_pitcher_free",[-10,-10,-10],0)
    _alpha(model,"pitcher")
    qids,limits=robot_qpos_addresses(model)
    if np.any(qs<limits[:,0]) or np.any(qs>limits[:,1]):
        raise ValueError("FR5 path violates declared joint limits")
    hinge=model.joint(JOINT)
    if hinge.type!=mujoco.mjtJoint.mjJNT_HINGE:
        raise AssertionError("Station holder must be a physical single hinge")
    # MuJoCo range is converted to radians, qpos is ALWAYS radians.
    joint_address=int(model.jnt_qposadr[hinge.id])
    if not model.jnt_limited[hinge.id] or not np.allclose(
            model.jnt_range[hinge.id],np.deg2rad([-18.,18.]),atol=1e-7):
        raise AssertionError("Cup hinge end-stops unexpected")
    # One extra angular DOF. No prismatic slider, no 6D cup mocap.
    holder_id=model.body(BODY).id
    base_id=model.body("rb_stationary_cup_tilt_base").id
    parent=int(model.body_parentid[holder_id])
    assert parent==base_id
    pivot=None
    images=out/"frames"
    images.mkdir(parents=True,exist_ok=True)
    renderer=mujoco.Renderer(model,width=width,height=height)
    log=[]
    try:
        for i,(q,ang) in enumerate(zip(qs,tilt_deg)):
            data.qpos[qids]=q
            data.qpos[joint_address]=np.deg2rad(ang)
            mujoco.mj_forward(model,data)
            now=np.asarray(data.xpos[base_id]).copy()
            hinge_xyz=np.asarray(data.xpos[holder_id]).copy()
            if pivot is None:pivot=now
            if not np.allclose(now,pivot,atol=1e-9):
                raise AssertionError("Fixed pivot translated")
            if not np.allclose(hinge_xyz,pivot,atol=1e-9):
                raise AssertionError("Hinge mount translated")
            cup_R=data.xmat[holder_id].reshape(3,3)
            expected=Rotation.from_euler("y",ang,degrees=True).as_matrix()
            if not np.allclose(cup_R,expected,atol=1e-6):
                raise AssertionError("Unexpected extra rotation of cup")
            spout=np.asarray(data.geom_xpos[model.geom("rb_gripped_pitcher_spout").id]).copy()
            rim=np.asarray(data.geom_xpos[model.geom("rb_tilt_cup_rim").id]).copy()
            camera_eye=(spout+rim)/2+np.array([.17,-.65,.23])
            camera_target=(spout+rim)/2
            cam_id=model.body("rb_fixed_tilt_camera_rig").id
            mc_id=int(model.body_mocapid[cam_id])
            data.mocap_pos[mc_id]=camera_eye
            data.mocap_quat[mc_id]=look_at_quat(camera_eye,camera_target)
            mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera="rb_fixed_tilt_closeup")
            close=Image.fromarray(renderer.render().copy())
            renderer.update_scene(data,camera="cell_overview")
            overview=Image.fromarray(renderer.render().copy())
            canvas=Image.new("RGB",(width*2,height+80),(19,29,39))
            canvas.paste(close,(0,43))
            canvas.paste(overview,(width,43))
            draw=ImageDraw.Draw(canvas)
            draw.text((15,11),"FR5 ONE ARM: PITCHER + FIXED PIVOT 1-DOF CUP",fill=(235,245,250))
            draw.text((15,height+56),f"{mode}: {ang:.1f} deg | NO cup X/Y/Z motor | {i+1}/{len(qs)}",fill=(250,220,167))
            log.append({"time_s":float(t[i]),"holder_hinge_angle_deg":float(ang),
                        "stationary_pivot_xyz_m":pivot.tolist(),
                        "cup_rim_xyz_m":rim.tolist(),
                        "virtual_spout_xyz_m":spout.tolist(),
                        "visual_spout_to_cup_rim_mm":float(1000*np.linalg.norm(spout-rim))})
            canvas.save(images/f"frame_{i:04d}.png")
    finally:
        renderer.close()
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg required to produce requested preview")
    movie=out/"one_arm_fixed_pivot_"+mode+".mp4"
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",
                    str(fps),"-i",str(images/"frame_%04d.png"),
                    "-c:v","libx264","-pix_fmt","yuv420p",str(movie)],check=True)
    report={
        "status":"ONE_ARM_HINGE_VIDEO_CREATED",
        "mode":mode,"frames":len(qs),"fps":fps,
        "source_pitcher_arm_confirmed":False,
        "source_cup_arm_confirmed":False,
        "source_cup_data_used":mode=="projected-source",
        "source_cup_pitcher_mount_calibrated":False,
        "angle_source":meaning,
        "cup_independent_translation_dofs":0,
        "cup_independent_rotation_dofs":1,
        "cup_allowed_joint_axis":"local/world Y at fixed pivot",
        "cup_pivot_xyz_m":pivot.tolist(),
        "max_tilt_deg":float(np.max(np.abs(tilt_deg))),
        "holder_tilt_limits_deg":[-MAX_TILT_DEG,MAX_TILT_DEG],
        "max_speed_deg_s":MAX_TILT_RATE_DEG_S,
        "cup_rim_arc_translation_from_pivot_is_normal_rotation_geometry":True,
        "robot_joint_path_kinematic_not_motor":True,
        "physical_contact_or_milk_flow_simulated":False,
        "actual_spout_over_liquid_surface_verified":False,
        "distance_spout_to_rim_first_mm":log[0]["visual_spout_to_cup_rim_mm"],
        "distance_spout_to_rim_last_mm":log[-1]["visual_spout_to_cup_rim_mm"],
        "video":movie.name
    }
    (out/"summary.json").write_text(json.dumps(report,indent=2))
    (out/"cup_rotation_only_series.json").write_text(json.dumps(log,indent=2))
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--robot-joint-path",required=True)
    p.add_argument("--output",required=True)
    p.add_argument("--mode",choices=["scripted","projected-source"],default="scripted")
    p.add_argument("--original-joints")
    p.add_argument("--urdf")
    p.add_argument("--source-cup-arm",choices=["left","right"],default="right")
    p.add_argument("--duration",type=float,default=4.)
    p.add_argument("--fps",type=int,default=3)
    a=p.parse_args()
    result=render_one_arm_fixed_pivot(
        robot_joint_path=a.robot_joint_path,output_dir=a.output,
        mode=a.mode,original_joints=a.original_joints,
        urdf=a.urdf,source_cup_arm=a.source_cup_arm,
        duration_s=a.duration,fps=a.fps)
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    main()
