"""Illustrative La Marzocco SWIFT MINI visual proxy, NOT manufacturer CAD.

External dimensions are from La Marzocco Swift Mini parts catalog v1.0
and operating manual v1.2: width 166 mm, depth 322 mm, height 448-449 mm.
Unlike the BIG commercial "Swift grinder", this is the smaller Swift MINI.

Only outer envelope is grounded to factory dimensions. Details including
the actual portafilter bayonet/retention ring center, clamping tolerances,
clearances, tooling and tamper mechanism are NOT manufacturer calibrated.
This visual model cannot authorize FR5 docking or contact.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

WIDTH_M = 0.166
DEPTH_M = 0.322
HEIGHT_M = 0.449
SOURCE = {
    "manual": "https://lamarzoccousa.com/wp-content/uploads/2020/08/MAN.13.05.1_SWIFTMINI_EN-FR_V1.2-.pdf",
    "parts_catalog": "https://lamarzocco.com/de/de/wp-content/uploads/2023/07/15-SwiftMini_V1.0.pdf",
}
# Visual proportions below are estimates from manufacturer diagrams/photographs.
# They are NOT mating dimensions or portafilter CAD.
APPROX_PORTAFILTER_ENTRY_XYZ_M = (0.0, -0.101, 0.167)


def _numbers(values):
    return " ".join(f"{v:.7f}" for v in values)


def _geom(parent, name, shape, xyz, size, rgba, **kwargs):
    attrib = {
        "name": "rb_sm_" + name, "type": shape,
        "pos": _numbers(xyz), "size": _numbers(size),
        "rgba": rgba, "contype": "0", "conaffinity": "0",
        "mass": "0",
    }
    attrib.update(kwargs)
    return ET.SubElement(parent, "geom", attrib)


def add_swift_mini_proxy(world, *, xyz=(0,0,0)):
    """Insert fixed, NONCOLLIDING 1:1 visual proxy into a MuJoCo worldbody."""
    body = ET.SubElement(world, "body", {
        "name": "rb_swift_mini_visual_reference", "pos": _numbers(xyz),
    })
    black = ".105 .125 .14 1"
    graphite = ".18 .195 .21 1"
    silver = ".68 .70 .72 1"
    stainless = ".83 .85 .87 1"
    clear = ".73 .88 .93 .22"
    # The official 166x322mm full footprint. The following approximate
    # geometry stays inside the product design's declared external envelope.
    _geom(body, "base", "box", (0,0,.017),(.081,.157,.017),black)
    _geom(body, "front_tray", "box",(0,-.114,.039),(.067,.045,.003),graphite)
    _geom(body, "rear_leg", "box",(0,.088,.120),(.064,.061,.099),silver)
    _geom(body, "rear_panel", "box",(0,.153,.159),(.075,.006,.142),graphite)
    # Body upper shelf, rear/right support and front grinding cylinder.
    _geom(body, "upper_shell", "box",(0,.024,.301),(.080,.129,.055),black)
    _geom(body, "upper_seam", "box",(0,.024,.247),(.080,.127,.002),graphite)
    _geom(body, "front_head", "cylinder",(0,-.075,.291),(.069,.070),graphite)
    _geom(body, "grind_adjust_band", "cylinder",(0,-.075,.258),(.070,.006),stainless)
    _geom(body, "front_button", "sphere",(0,-.142,.270),(.009,),silver)
    _geom(body, "button_center", "sphere",(0,-.150,.270),(.004,),".36 .66 .47 1")
    _geom(body, "lower_doser", "cylinder",(0,-.075,.202),(.052,.043),black)
    _geom(body, "dose_adjust_band", "cylinder",(0,-.075,.180),(.053,.007),silver)
    _geom(body, "bayonet_ring_proxy", "cylinder",(0,-.075,.170),(.043,.006),stainless)
    # Portafilter center indicated by a SITE, not physical docking CAD.
    ET.SubElement(body, "site", {
        "name": "rb_swift_mini_bayonet_UNCALIBRATED", "type": "sphere",
        "pos": _numbers(APPROX_PORTAFILTER_ENTRY_XYZ_M),
        "size": ".004", "rgba": ".15 .9 .95 1",
    })
    # Slim hopper bowl: transparent visual approximation, no coffee dynamics.
    _geom(body, "hopper_neck", "cylinder",(0,-.075,.373),(.041,.013),black)
    _geom(body, "hopper_chamber", "box",(0,-.075,.407),(.063,.065,.024),clear)
    _geom(body, "hopper_lid", "box",(0,-.075,.439),(.074,.074,.008),black)
    _geom(body, "lid_highlight", "box",(0,-.075,.448),(.064,.061,.001),graphite)
    # Thin clear edges for visibility, kept inside hopper/lid bound.
    for side_x in (-1,1):
        _geom(body, "hopper_edge_"+str(side_x), "capsule",
              (0,0,0),(.0025,),stainless,
              fromto=_numbers(((side_x*.057),-.132,.380,(side_x*.068),-.137,.433))
              )
    _geom(body, "swing_arm_proxy", "box",(.052,.013,.368),
          (.009,.061,.009),graphite)
    # Product-bounding markers: explicit design envelope, not a collision hull.
    for idx,(sx,sy,zz) in enumerate(
            ( (sx,sy,zz)
              for sx in (-WIDTH_M/2,WIDTH_M/2)
              for sy in (-DEPTH_M/2,DEPTH_M/2)
              for zz in (0.,HEIGHT_M))):
        ET.SubElement(body,"site",{
            "name":f"rb_sm_envelope_corner_{idx}",
            "type":"sphere","pos":_numbers((sx,sy,zz)),
            "size":".0003","rgba":"0 0 0 0",
        })
    return body


def make_preview_scene(path):
    root=ET.Element("mujoco", model="Swift MINI scaled illustrative asset")
    ET.SubElement(root,"compiler",angle="radian")
    ET.SubElement(root,"option",gravity="0 0 -9.81")
    world=ET.SubElement(root,"worldbody")
    ET.SubElement(world,"light",pos="-.3 -.6 1.8",dir="0 0 -1",
                  directional="true",diffuse="1 1 1")
    _geom(world,"floor","plane",(0,0,-.04),(1,1,.1),".82 .83 .83 1")
    _geom(world,"countertop","box",(0,0,-.023),(.35,.30,.023),".50 .56 .60 1")
    add_swift_mini_proxy(world)
    from .geometry import look_at_quat
    views={
        "rb_swift_mini_front":((.43,-.70,.55),(0,0,.26)),
        "rb_swift_mini_side":((.70,-.32,.42),(0,0,.22)),
        "rb_swift_mini_back":((-.51,.53,.48),(0,0,.26)),
    }
    for name,(eye,focus) in views.items():
        ET.SubElement(world,"camera",{
            "name":name,"pos":_numbers(eye),
            "quat":_numbers(look_at_quat(eye,focus)),
            "fovy":"44",
        })
    path=Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path,encoding="unicode")
    return path


def dimension_report():
    return {
        "model":"La Marzocco Swift Mini (NOT the commercial Swift)",
        "units":"m",
        "manufacturer_envelope_m": {
            "width":WIDTH_M, "depth":DEPTH_M,"height":HEIGHT_M},
        "measured_bayonet_center":False,
        "bayonet_center_is_visual_placeholder":True,
        "bayonet_placeholder_local_xyz_m":APPROX_PORTAFILTER_ENTRY_XYZ_M,
        "portafilter_retention_mechanism_modeled":False,
        "tamper_mechanism_kinematics_modeled":False,
        "surface_mesh_from_manufacturer":False,
        "grinder_real_machine_control_connected":False,
        "no_real_robot_contact_authorized":True,
        "ready_to_replace_active_workcell":False,
        "active_workcell_grinder_keepouts_preserved":True,
        "sources":SOURCE,
    }


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output", default="runs/swift_mini_reference")
    p.add_argument("--width",type=int,default=720)
    p.add_argument("--height",type=int,default=540)
    a=p.parse_args()
    import mujoco
    from PIL import Image
    root=Path(a.output)
    root.mkdir(parents=True,exist_ok=True)
    scene=make_preview_scene(root/"swift_mini_reference.xml")
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,width=a.width,height=a.height)
    try:
        for name in ("rb_swift_mini_front","rb_swift_mini_side",
                     "rb_swift_mini_back"):
            renderer.update_scene(data,camera=name)
            Image.fromarray(renderer.render().copy()).save(root/(name+".png"))
    finally:
        renderer.close()
    report=dimension_report()
    (root/"dimensions_and_limitations.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
