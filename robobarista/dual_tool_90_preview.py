"""Real MuJoCo 3D preview of the agreed FR5 -> F/T -> 90-degree dual gripper.

Geometry is EXPLICITLY APPROXIMATE, not the missing Inlux adapter CAD.
The resulting PNG is rendered directly with MuJoCo and robot kinematics.
Neither grasp forces nor vendor sensor measurements are simulated.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .compact_tags import build_compact_tag_cell
from .camera_joint_replay import _find_start
from .robot_replay import robot_qpos_addresses
from .geometry import look_at_quat

# 3D model constants, metres; provisional CAD equivalent ONLY.
GRIPPER_STYLE = "IR40-50-visual-proxy"
SENSOR_STYLE = "ITOSX1000FTS-visual-proxy"
ADAPTER_STYLE = "ICACX1210XDUAL-90deg-visual-proxy"
TOOL_ANGLE_DEG = 90.


def nums(xs):
    return " ".join(f"{float(v):.9g}" for v in xs)


def body(parent, name, position, euler=None):
    attrs = {"name": name, "pos": nums(position)}
    if euler is not None:
        attrs["euler"] = nums(euler)
    return ET.SubElement(parent, "body", attrs)


def geom(parent, name, shape, xyz, size, color, **attrs):
    d = {"name": name, "type": shape, "pos": nums(xyz), "size": nums(size),
         "rgba": color, "mass": "0", "contype": "0", "conaffinity": "0"}
    d.update(attrs)
    return ET.SubElement(parent, "geom", d)


def gripper(mount, name, color):
    """Each arm has one IR40-like body and two opposed jaw geometries."""
    unit = body(mount, name, (0, 0, .020))
    # Roll each IR40 about its own APPROACH AXIS by 90 degrees.
    # With the selected front-side wrist orientations, opposed jaws
    # now close LEFT/RIGHT around the handle, not above/below it.
    # This leaves the two mounting/approach axes 90 degrees apart.
    unit.set("euler","0 0 1.5707963267948966")
    geom(unit, name+"_base", "cylinder", (0, 0, .003),
         (.029, .010), ".68 .71 .74 1")
    geom(unit, name+"_housing", "box", (0, 0, .034),
         (.029, .035, .030), color)
    geom(unit, name+"_top_plate", "box", (0, 0, .066),
         (.028, .034, .004), ".77 .80 .83 1")
    for sign in (-1, 1):
        geom(unit, name+"_jaw_"+("left" if sign < 0 else "right"),
             "box", (sign*.017, 0, .101), (.006, .019, .031),
             ".21 .25 .27 1")
        geom(unit, name+"_pad_"+("left" if sign < 0 else "right"),
             "box", (sign*.011, 0, .111), (.003, .014, .019),
             ".08 .09 .09 1")
    return unit


def build_simulation(vendor_dir):
    """Use the EXISTING FAIRINO FR5 / coffee cell / compact tags, replace tool only."""
    base = build_compact_tag_cell(Path(vendor_dir).resolve())
    folder = Path(base).parent
    scene = ET.parse(base)
    include = scene.getroot().find("include")
    if include is None:
        raise RuntimeError("Coffee workcell robot MJCF include missing")
    original_robot = folder / include.get("file")
    robot = ET.parse(original_robot)
    tcp = robot.getroot().find(".//body[@name='rb_cell_tcp']")
    if tcp is None:
        raise RuntimeError("FR5 tool reference frame missing")

    # Directly on FR5 tool reference is the SINGLE six-axis F/T sensor.
    sensor = body(tcp, "rb_90_force_torque_sensor", (0, 0, .016))
    geom(sensor, "rb_90_ft_lower_flange", "cylinder",
         (0, 0, -.012), (.044, .006), ".70 .74 .77 1")
    geom(sensor, "rb_90_ft_six_axis_body", "cylinder",
         (0, 0, .006), (.038, .013), ".23 .28 .34 1")
    geom(sensor, "rb_90_ft_upper_flange", "cylinder",
         (0, 0, .025), (.044, .006), ".76 .80 .83 1")
    # Four visible fixture bolts on the sensor shell.
    for i, (x, y) in enumerate([(-.029,-.025),(-.029,.025),
                                (.029,-.025),(.029,.025)]):
        geom(sensor, f"rb_90_ft_bolt_{i}", "cylinder",
             (x, y, .033), (.003,.002), ".15 .17 .18 1")

    # A machined right-angle adapter, approximate until vendor STEP arrives.
    adapter = body(sensor, "rb_90_dual_adapter", (0, 0, .039))
    geom(adapter, "rb_90_adapter_core", "box",
         (.012, 0, .019), (.048, .043, .021), ".69 .72 .75 1")
    geom(adapter, "rb_90_adapter_top", "box",
         (0, 0, .043), (.039, .041, .006), ".83 .85 .87 1")
    geom(adapter, "rb_90_adapter_side", "box",
         (.060, 0, .019), (.006, .038, .035), ".83 .85 .87 1")
    # First tool faces local +Z; second tool faces local +X,
    # because a +90-degree rotation about Y maps local +Z -> +X.
    axial = body(adapter, "rb_90_mount_axial", (0, 0, .055))
    orthogonal = body(adapter, "rb_90_mount_orthogonal",
                      (.073, 0, .019), euler=(0, np.pi/2, 0))
    geom(axial, "rb_90_axial_flange", "cylinder",
         (0, 0, .001), (.034, .008), ".76 .79 .81 1")
    geom(orthogonal, "rb_90_side_flange", "cylinder",
         (0, 0, .001), (.034, .008), ".76 .79 .81 1")
    gripper(axial, "rb_90_ir40_handle", ".29 .37 .44 1")
    gripper(orthogonal, "rb_90_ir40_cup", ".32 .40 .47 1")

    # Two different end-effector tips: small contrasting markers only in
    # the rendering. Camera AprilTags on REAL objects remain unchanged.
    geom(axial, "rb_90_handle_tip", "sphere",
         (0, 0, .163), (.007,), ".23 .92 .79 1")
    geom(orthogonal, "rb_90_cup_tip", "sphere",
         (0, 0, .163), (.007,), ".99 .67 .27 1")

    robot_out = folder / "rb_fr5_actual_90_dual_preview_robot.xml"
    robot.write(robot_out, encoding="unicode")
    include.set("file", robot_out.name)

    world = scene.getroot().find("worldbody")
    if world is None:
        raise RuntimeError("Missing workcell worldbody")
    camera = body(world, "rb_90_preview_camera_rig", (.5, -.8, 1.6))
    camera.set("mocap", "true")
    ET.SubElement(camera, "camera", {"name": "rb_90_preview_closeup",
                                     "fovy": "52", "pos": "0 0 0",
                                     "quat": "1 0 0 0"})
    out = folder / "rb_fr5_90_sensor_two_ir40_compact_preview.xml"
    scene.write(out, encoding="unicode")
    return out


def _font(size):
    for name in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def _render(model, data, eye, focus, width, height, renderer):
    idx = int(model.body_mocapid[model.body("rb_90_preview_camera_rig").id])
    data.mocap_pos[idx] = np.asarray(eye)
    data.mocap_quat[idx] = look_at_quat(eye, focus)
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera="rb_90_preview_closeup")
    return Image.fromarray(renderer.render().copy())


def render_preview(args):
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    scene = build_simulation(Path(args.vendor_dir))
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    # Use an actual collision-screened FR5 joint pose, not a drawn robot.
    qids, limits = robot_qpos_addresses(model)
    q0, _ = _find_start(model, data, qids, limits, args.seed)
    data.qpos[qids] = q0
    mujoco.mj_forward(model, data)

    sensor = model.body("rb_90_force_torque_sensor").id
    adapter = model.body("rb_90_dual_adapter").id
    axial = model.body("rb_90_mount_axial").id
    side = model.body("rb_90_mount_orthogonal").id
    z1 = data.xmat[axial].reshape(3,3)[:,2]
    z2 = data.xmat[side].reshape(3,3)[:,2]
    dot = float(np.clip(np.dot(z1,z2), -1., 1.))
    angle = float(np.degrees(np.arccos(dot)))
    if abs(angle - 90.) > .001:
        raise RuntimeError(f"Tool axes are not 90 degrees: {angle} deg")
    tcp = data.xpos[model.body("rb_cell_tcp").id].copy()

    # Every panel below is a rendered MuJoCo scene of the SAME articulated
    # FAIRINO 6-joint state. No generative model, illustration, or Photoshop.
    w, h = args.width, args.height
    renderer = mujoco.Renderer(model, width=w, height=h)
    try:
        main = _render(model, data,
                       tcp + np.array([.65,-.67,.28]),
                       tcp + np.array([.045,0,.13]), w,h,renderer)
        side_img = _render(model,data,
                           tcp + np.array([.56,.48,.23]),
                           tcp + np.array([.045,0,.12]),w,h,renderer)
        renderer.update_scene(data,camera="cell_overview")
        overview = Image.fromarray(renderer.render().copy())
    finally:
        renderer.close()

    main.save(out/"mujoco_90_dual_front.png")
    side_img.save(out/"mujoco_90_dual_side.png")
    overview.save(out/"mujoco_90_dual_overview.png")

    # Side-by-side contact sheet, labels OUTSIDE the raw MuJoCo images.
    board = Image.new("RGB",(w*2+36,h*2+140),(19,26,33))
    d=ImageDraw.Draw(board)
    d.text((22,14),"FAIRINO FR5  |  MuJoCo – echter Code-Render",
           font=_font(29), fill=(239,245,251))
    board.paste(main,(12,64))
    board.paste(side_img,(w+24,64))
    d.text((18,h+69),"Vorne: Sensor -> Adapter -> 2 Greifer",
           font=_font(19), fill=(224,237,246))
    d.text((w+30,h+69),"Seitlich: exakt 90 Grad zwischen Werkzeugachsen",
           font=_font(19), fill=(224,237,246))
    board.paste(overview,(12,h+114))
    d.text((w+30,h+116),"Gesamter Kaffeearbeitsplatz + kompakte AprilTags",
           font=_font(18), fill=(224,237,246))
    d.text((w+30,h+151),"Originales FAIRINO-FR5-Robotermodell",font=_font(17),
           fill=(174,203,224))
    d.text((w+30,h+189),"Sensor/Adapter/IR40 = unkalibrierte Modell-Proxys",
           font=_font(17),fill=(252,201,148))
    d.text((w+30,h+230),"KEIN STEP-Import / keine Greifphysik / keine echte F/T",
           font=_font(16),fill=(252,201,148))
    d.text((w+30,h+272),f"Gelenkwinkel der Werkzeugachsen: {angle:.1f}°",
           font=_font(20), fill=(128,229,204))
    board.save(out/"RoboBarista_FR5_90deg_DualTool_MuJoCo.png")

    result={
        "status":"PASS",
        "render_engine":"mujoco.Renderer using model from scene XML",
        "same_fr5_qpos_in_all_views":q0.tolist(),
        "dual_tool_angle_deg":angle,
        "force_sensor_before_adapter":bool(sensor and adapter),
        "two_visual_ir40_proxies":True,
        "uses_existing_compact_on_object_apriltags":True,
        "actual_grasp_contact":False,
        "real_ft_sensor_simulation":False,
        "real_adapter_step_imported":False,
        "real_ir40_step_imported":False,
        "collision_checked_for_extra_tool_envelopes":False,
        "files":["mujoco_90_dual_front.png","mujoco_90_dual_side.png",
                 "mujoco_90_dual_overview.png","RoboBarista_FR5_90deg_DualTool_MuJoCo.png"]}
    (out/"summary.json").write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--output",default="runs/dual90_preview")
    p.add_argument("--width",type=int,default=960)
    p.add_argument("--height",type=int,default=630)
    p.add_argument("--seed",type=int,default=14)
    r=render_preview(p.parse_args())
    import os,sys
    sys.stdout.flush()
    if os.getenv("MUJOCO_GL","").lower()=="osmesa":
        os._exit(0 if r["status"]=="PASS" else 1)


if __name__=="__main__":
    main()
