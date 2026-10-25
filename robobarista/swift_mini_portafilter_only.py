"""Minimal physical MuJoCo: Swift MINI + our IR40-friendly portafilter handle.

ONLY these two objects and a simplified sacrificial seating-contact pad.
No FR5, tracking cameras, machine electricity, coffee, tamper, or other props.

Requested experiment: keep basket LEVEL; move along Z under the receiver;
push up gently with BOUNDED virtual actuator force; rotate handle RIGHT in
the horizontal plane with BOUNDED yaw torque. "Right" = positive world +Z
rotation: a handle starting towards -Y sweeps towards +X.

Manufacturer Swift Mini manual 8.2: only LIGHT insertion force; does NOT
verify a mandatory right-twist locking angle. This test's right rotation is
explicitly HYPOTHETICAL and does NOT establish a mechanically locked bayonet.
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

from .swift_mini_visual_asset import (
    add_swift_mini_proxy, APPROX_PORTAFILTER_ENTRY_XYZ_M,
    WIDTH_M, DEPTH_M, HEIGHT_M,
)

PORT_ORIGIN_XY = tuple(APPROX_PORTAFILTER_ENTRY_XYZ_M[:2])
BODY_START_Z_M = .079
SOCKET_STOP_Z_M = .1865
LIP_LOCAL_TOP_Z_M = .036
UP_FORCE_LIMIT_N = 18.0  # exploratory engineering guardrail, not OEM datum
YAW_TORQUE_LIMIT_NM = 1.6
RIGHT_TWIST_DEG = 12.0  # exploratory right-side turn, NOT manufacturer instruction
JOINT_Z = "rb_pf_slide_z"
JOINT_YAW = "rb_pf_twist_z"
PF_NAME = "rb_custom_handle_portafilter"
RECEIVER = "rb_swift_mini_socket_stop_CONTACT_PROXY"
LIP = "rb_pf_mouth_lip_CONTACT_PROXY"
TOTAL_S = 4.5
PHASES = ("ALIGN", "GENTLE_LIFT", "CONTACT_HOLD", "OPTIONAL_RIGHT_TWIST", "HOLD")


def vec(xs):
    return " ".join(f"{float(v):.9g}" for v in xs)


def geom(p, name, kind, xyz, size, rgba, *, collision=False, mass=None, **kw):
    attrs={"name":name, "type":kind, "size":vec(size),
           "rgba":rgba,
           "contype":"1" if collision else "0",
           "conaffinity":"1" if collision else "0"}
    if "fromto" not in kw:
        attrs["pos"]=vec(xyz)
    else:
        attrs["fromto"]=vec(kw.pop("fromto"))
    if mass is not None:
        attrs["mass"]=str(mass)
    attrs.update(kw)
    return ET.SubElement(p,"geom",attrs)


def build_minimal_scene(path):
    root=ET.Element("mujoco",model="Minimal Swift Mini / One portafilter, no camera sensors")
    ET.SubElement(root,"compiler",angle="radian")
    vis=ET.SubElement(root,"visual")
    ET.SubElement(vis,"global",offwidth="1600",offheight="1000")
    ET.SubElement(root,"option",timestep=".0025",gravity="0 0 -9.81",integrator="implicitfast")
    ET.SubElement(root,"size",njmax="1500",nconmax="250")
    default=ET.SubElement(root,"default")
    ET.SubElement(default,"geom",condim="3",friction="0.7 0.02 0.01",
                  solref="0.012 1",solimp="0.95 0.995 0.001")
    world=ET.SubElement(root,"worldbody")
    ET.SubElement(world,"light",name="main_light",pos="-.4 -.6 1.2",
                  dir=".1 .2 -1",directional="true",diffuse="1 1 1")
    geom(world,"rb_simple_table","box",(0,0,-.028),(.35,.37,.028),".51 .57 .61 1")
    grinder=add_swift_mini_proxy(world)
    assert grinder.get("name")=="rb_swift_mini_visual_reference"

    # Intentionally simplified force-contact fixture - NOT real Swift bayonet.
    fixture=ET.SubElement(world,"body",name="rb_swift_mini_simple_receiver",
                          pos=vec((PORT_ORIGIN_XY[0],PORT_ORIGIN_XY[1],0)))
    geom(fixture,"rb_receiver_blue_target_outline","cylinder",
         (0,0,.178),(.049,.003),".08 .87 .90 .32")
    geom(fixture,RECEIVER,"cylinder",
         (0,0,SOCKET_STOP_Z_M),(.039,.0025),".32 .58 .66 .65",
         collision=True)
    ET.SubElement(fixture,"site",name="rb_pf_target_bayonet_UNCALIBRATED",
                  pos=vec((0,0,.180)),type="sphere",size=".003",
                  rgba="0.2 0.95 0.95 1")

    # Mechanically constrained stage with exactly 2 DOF:
    # vertical Z slide, then vertical yaw. XY, roll and pitch NOT MOTORABLE.
    vertical=ET.SubElement(world,"body",name="rb_pf_vertical_carriage",
                           pos=vec((PORT_ORIGIN_XY[0],PORT_ORIGIN_XY[1],BODY_START_Z_M)))
    ET.SubElement(vertical,"joint",name=JOINT_Z,type="slide",
                  axis="0 0 1",range="0 .085",limited="true",
                  damping="8",armature=".02")
    ET.SubElement(vertical,"inertial",pos="0 0 0",mass=".12",
                  diaginertia=".00004 .00004 .00004")
    pf=ET.SubElement(vertical,"body",name=PF_NAME,pos="0 0 0")
    ET.SubElement(pf,"joint",name=JOINT_YAW,type="hinge",
                  axis="0 0 1",range="0 .27925268",limited="true",
                  damping=".12",armature=".001")
    ET.SubElement(pf,"inertial",pos="0 -.11 .005",mass=".60",
                  diaginertia=".003 .003 .002")
    # BASKET: local Z is upright; independent roll and pitch forbidden by joints.
    geom(pf,"rb_pf_filter_basket","cylinder",(0,0,.015),(.042,.018),
         ".73 .76 .79 1")
    geom(pf,"rb_pf_inner_basket","cylinder",(0,0,.034),(.035,.001),
         ".35 .39 .43 1")
    geom(pf,LIP,"cylinder",(0,0,.033),(.039,.003),
         ".82 .85 .87 1",collision=True)
    # Basket spout and shoulder (visual only).
    geom(pf,"rb_pf_neck_bridge","box",(0,-.049,.017),
         (.017,.050,.012),".59 .61 .64 1")
    geom(pf,"rb_pf_spout","cylinder",(0,.014,-.006),(.009,.013),
         ".45 .48 .50 1")
    # Our existing rb_gripped_portafilter handle concept:
    # 28-mm gripper-optimized rectangular sleeve, 160-mm length,
    # axial mechanical locating shoulders + stopping flange.
    geom(pf,"rb_pf_our_IR40_50_handle","box",(0,-.145,.013),
         (.014,.080,.015),".27 .31 .35 1")
    geom(pf,"rb_pf_our_grip_stop_front","box",(0,-.212,.013),
         (.019,.005,.021),".80 .66 .20 1")
    geom(pf,"rb_pf_our_grip_stop_rear","box",(0,-.080,.013),
         (.019,.005,.021),".80 .66 .20 1")
    geom(pf,"rb_pf_our_grip_finger_land","box",(0,-.147,.013),
         (.0155,.035,.016),".14 .48 .51 1")
    for i,y in enumerate((-.105,-.118,-.173,-.187)):
        geom(pf,f"rb_pf_grip_index_groove_{i}","box",(0,y,.028),
             (.0148,.002,.0006),".06 .22 .25 1")
    ET.SubElement(pf,"site",name="rb_pf_handle_grip_center",
                  pos=vec((0,-.147,.013)),size=".004",rgba=".98 .76 .22 1")
    ET.SubElement(pf,"site",name="rb_pf_basket_center",
                  pos=vec((0,0,.033)),size=".003",rgba=".17 .95 .68 1")

    act=ET.SubElement(root,"actuator")
    ET.SubElement(act,"position",name="rb_pf_lift_servo",joint=JOINT_Z,
                  kp="600",kv="24",forcerange=f"0 {UP_FORCE_LIMIT_N}",
                  forcelimited="true",ctrlrange="0 .085",ctrllimited="true")
    ET.SubElement(act,"position",name="rb_pf_yaw_servo",joint=JOINT_YAW,
                  kp="10",kv=".3",forcerange=f"-{YAW_TORQUE_LIMIT_NM} {YAW_TORQUE_LIMIT_NM}",
                  forcelimited="true",ctrlrange="0 .27925268",ctrllimited="true")

    p=Path(path)
    p.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(root).write(p,encoding="unicode")
    return p


def get_phases(t,*,with_right_turn=True):
    """Staged *commanded* motion. Right twist is optional and unvalidated."""
    t=float(t)
    if t<.4:return "ALIGN",0.,0.
    if t<2.:  # lift from z0 to commanded compliant touch
        u=np.clip((t-.4)/1.6,0,1)
        s=u*u*(3-2*u)
        return "GENTLE_LIFT",.082*s,0.
    if t<2.6:return "CONTACT_HOLD",.082,0.
    if t<3.8:
        u=np.clip((t-2.6)/1.2,0,1)
        s=u*u*(3-2*u)
        return "OPTIONAL_RIGHT_TWIST",.082,(
            np.deg2rad(RIGHT_TWIST_DEG)*s if with_right_turn else 0.)
    return "HOLD",.082,(
        np.deg2rad(RIGHT_TWIST_DEG) if with_right_turn else 0.)


def receiver_force_n(model,data):
    """Normal contact impulse/force on ONLY the two intentionally colliding proxies."""
    a=model.geom(RECEIVER).id
    b=model.geom(LIP).id
    force=0.
    for i in range(data.ncon):
        c=data.contact[i]
        if {int(c.geom1),int(c.geom2)}!={a,b}:
            continue
        cf=np.zeros(6)
        mujoco.mj_contactForce(model,data,i,cf)
        force+=max(0.,float(cf[0]))  # normal component; not real measured load
    return force


def _render_camera(model,data,renderer,*,lookat,azimuth,distance,elevation):
    cam=mujoco.MjvCamera()
    cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:]=lookat
    cam.azimuth=azimuth
    cam.elevation=elevation
    cam.distance=distance
    renderer.update_scene(data,camera=cam)
    return Image.fromarray(renderer.render().copy())


def simulate(output,*,fps=15,with_right_turn=True):
    out=Path(output)
    out.mkdir(parents=True,exist_ok=True)
    scene=build_minimal_scene(out/"minimal_swift_portafilter.xml")
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    jz=model.joint(JOINT_Z).id
    jy=model.joint(JOINT_YAW).id
    zidx=int(model.jnt_qposadr[jz])
    yawidx=int(model.jnt_qposadr[jy])
    zvel=int(model.jnt_dofadr[jz])
    yawvel=int(model.jnt_dofadr[jy])
    z_act=model.actuator("rb_pf_lift_servo").id
    yaw_act=model.actuator("rb_pf_yaw_servo").id
    mjsteps=int(round(TOTAL_S/model.opt.timestep))
    width,height=560,480
    period=1./fps
    log=[]
    frames=out/"frames"
    frames.mkdir(exist_ok=True)
    renderer=mujoco.Renderer(model,width=width,height=height)
    closest_to_horizontal=0.0
    max_xy_shift=0.0
    contact_max=0.0
    seen_right_rotation=False
    max_actual_lift_m=0.
    max_lift_servo_N=0.
    start_center=np.array([*PORT_ORIGIN_XY],float)
    frame_count=0
    last_phase=""
    try:
        for step in range(mjsteps+1):
            t=float(data.time)
            phase,zgoal,yawgoal=get_phases(t,with_right_turn=with_right_turn)
            data.ctrl[z_act]=zgoal
            data.ctrl[yaw_act]=yawgoal
            mujoco.mj_step(model,data)
            body=model.body(PF_NAME).id
            pos=np.array(data.xpos[body],float)
            orientation=np.array(data.xmat[body],float).reshape(3,3)
            # Basket +Z must remain EXACT world +Z in every sample.
            tilt=float(np.rad2deg(np.arccos(np.clip(orientation[2,2],-1,1))))
            shift=float(np.linalg.norm(pos[:2]-start_center))
            closest_to_horizontal=max(closest_to_horizontal,tilt)
            max_xy_shift=max(max_xy_shift,shift)
            if tilt>0.001 or shift>1e-5:
                raise AssertionError("Non-horizontal basket or unintended XY translation")
            force=float(receiver_force_n(model,data))
            max_actual_lift_m=max(max_actual_lift_m,float(data.qpos[zidx]))
            max_lift_servo_N=max(max_lift_servo_N,float(data.actuator_force[z_act]))
            contact_max=max(contact_max,force)
            yaw=float(np.rad2deg(data.qpos[yawidx]))
            if yaw>5:seen_right_rotation=True
            if data.time>=frame_count*period and t<=TOTAL_S+.01:
                log.append({
                    "time_s":float(data.time),
                    "stage":phase,
                    "center_xyz_m":pos.tolist(),
                    "basket_tilt_deg":tilt,
                    "basket_yaw_deg":yaw,
                    "commanded_upward_force_limit_N":UP_FORCE_LIMIT_N,
                    "actual_simulated_upward_servo_N":float(data.actuator_force[z_act]),
                    "actual_simulated_rotation_torque_Nm":float(data.actuator_force[yaw_act]),
                    "socket_contact_normal_N":force,
                    "z_goal_m":float(zgoal),
                    "yaw_goal_deg":float(np.rad2deg(yawgoal)),
                })
                view=_render_camera(model,data,renderer,
                      lookat=(0,-.090,.224),azimuth=135,distance=.68,elevation=-20)
                close=_render_camera(model,data,renderer,
                      lookat=(0,-.115,.18),azimuth=140,distance=.38,elevation=-22)
                canvas=Image.new("RGB",(width*2,height+84),(18,29,39))
                canvas.paste(view,(0,49))
                canvas.paste(close,(width,49))
                d=ImageDraw.Draw(canvas)
                d.text((16,11),"SWIFT MINI + OUR IR40 HANDLE | NO FR5, NO TRACKING CAMERAS",
                       fill=(236,246,251))
                d.text((16,32),"Prototype geometry: socket/bayonet position NOT measured",fill=(250,201,114))
                d.text((16,height+56),f"{phase} | Z {pos[2]:.3f} m | yaw RIGHT {yaw:.1f} deg | force {force:.1f} N",
                       fill=(223,238,248))
                canvas.save(frames/f"frame_{frame_count:04d}.png")
                frame_count+=1
            last_phase=phase
    finally:
        renderer.close()

    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg required for MP4 video")
    movie=out/("swift_mini_portafilter_"+("right_twist_UNCONFIRMED" if with_right_turn else "gentle_insert_only")+".mp4")
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(fps),
            "-i",str(frames/"frame_%04d.png"),
            "-c:v","libx264","-pix_fmt","yuv420p",str(movie)],check=True)
    if movie.stat().st_size<12000:
        raise RuntimeError("Empty MP4")
    result={
        "status":"SIMULATED_GEOMETRIC_AND_CONTACT_TEST" if contact_max>.1 else "NO_CONTACT_FORCE_MUST_INVESTIGATE",
        "machine":"La Marzocco Swift Mini",
        "n_robot_arms":0,
        "n_tracking_cameras":0,
        "rendering_views_not_sensors":2,
        "portafilter_with_our_handle":True,
        "portafilter_independent_motion_dofs":2,
        "allowed_dofs":["Z vertical translation under model fixture","yaw about vertical Z; rightward handle turn"],
        "unintended_x_y_motion_m":max_xy_shift,
        "max_basket_tilt_deg":closest_to_horizontal,
        "single_pivot_2dof_fixture_only":True,
        "receiver_target_xyz_m":[*PORT_ORIGIN_XY,SOCKET_STOP_Z_M],
        "receiver_target_physically_measured":False,
        "real_portafilter_lock_verified":False,
        "required_right_twist_manufacturer_confirmed":False,
        "requested_optional_right_turn":with_right_turn,
        "right_turn_gt_5deg_reached":seen_right_rotation,
        "max_normal_contact_force_N_SIMULATED_ONLY":contact_max,
        "max_actual_slide_z_m":max_actual_lift_m,
        "max_actual_upward_motor_N":max_lift_servo_N,
        "gap_before_upward_insert_m":(SOCKET_STOP_Z_M-.0025)-(BODY_START_Z_M+LIP_LOCAL_TOP_Z_M),

        "lift_actuator_force_guardrail_N":UP_FORCE_LIMIT_N,
        "right_torque_guardrail_Nm":YAW_TORQUE_LIMIT_NM,
        "no_real_machine_force_value_known":True,
        "frames":frame_count,
        "mp4":movie.name,
        "source_manual_portafilter_insertion_requires_light_force":True,
        "source_manual":"https://home.lamarzoccousa.com/wp-content/uploads/2020/08/lzDFNLH4rBBGwEovm9kSl.pdf",
        "limitations":"No OEM engagement/bayonet CAD, no gripper contact, no FR5 robot IK, no physical machine motor/force transfer; simulated contact is only two simplified rings. Right twist is hypothetical, not OEM lock instruction."
    }
    (out/"report.json").write_text(json.dumps(result,indent=2))
    (out/"samples.json").write_text(json.dumps(log,indent=2))
    print(json.dumps(result,indent=2))
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="runs/minimal_swift_mini_portafilter")
    p.add_argument("--fps",type=int,default=15)
    p.add_argument("--no-right-turn",action="store_true")
    a=p.parse_args()
    result=simulate(a.output,fps=a.fps,with_right_turn=not a.no_right_turn)
    if not result["status"].startswith("SIMULATED_"):
        raise SystemExit(2)


if __name__=="__main__":
    main()
