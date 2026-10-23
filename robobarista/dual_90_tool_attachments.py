"""Reuse PR#4 visual held-tool prototypes on the approved 90-degree tool.

Inserts kinematically attached prop MODELS on the existing IR40-50
gripper bodies. Never creates a second set of parallel-jaw grippers.
The transforms are approximate, not calibrated manufacturer-to-tool TCPs,
and provide no physical grasp, F/T feedback or collision validation.
"""
from __future__ import annotations
import copy
import xml.etree.ElementTree as ET

from .dual_ir40_concept import add_visual_dual_ir40_50


def attach_90_held_tools(robot: ET.ElementTree, cup_style="handle_mug"):
    if cup_style not in ("handle_mug","takeaway"):
        raise ValueError("unknown cup style")
    root=robot.getroot()
    handle=root.find(".//body[@name='rb_90_ir40_handle']")
    cup=root.find(".//body[@name='rb_90_ir40_cup']")
    if handle is None or cup is None:
        raise RuntimeError("Approved 90-degree dual IR40 bodies missing")

    # Exact old PR#4 prop primitives are reused, but NOT its old
    # tcp-mounted two PARALLEL gripper modules.
    temporary=ET.Element("body",{"name":"rb_temporary_legacy_not_exported"})
    add_visual_dual_ir40_50(temporary,include_props=True)
    a=temporary.find(".//body[@name='rb_handle_tool_IR40_50']")
    b=temporary.find(".//body[@name='rb_cup_tool_IR40_50']")
    if a is None or b is None:
        raise RuntimeError("Legacy prop shapes unavailable")

    for name in ("rb_gripped_pitcher","rb_gripped_portafilter"):
        old=a.find(f"body[@name='{name}']")
        if old is None:raise RuntimeError(name)
        prop=copy.deepcopy(old)
        # Actual 90° IR40 jaw centrelines are at z=.101, not the old .080.
        prop.set("pos","0 0 .101")
        # At the pitcher/portafilter pickup the entire wrist is oriented
        # Ry(-90 deg): the axial IR40 travels toward world -X.
        # Local IR40 jaws were spun Rz(+90), so apply their exact inverse
        # plus wrist compensation to keep vessel/basket upright.
        prop.set("quat","0.5 0.5 0.5 -0.5")
        # Pitcher/portafilter centre must sit FORWARD, along -X from
        # fingers, NOT underneath them. The old PR4 shape was +Y offset.
        if name=="rb_gripped_pitcher":
            offsets={
                "rb_gripped_pitcher_bridge":("-0.055 0 -0.008","0.061 0.010 0.010"),
                "rb_gripped_pitcher_body":("-0.113 0 -0.022",None),
                "rb_gripped_pitcher_rim":("-0.113 0 0.033",None),
                "rb_gripped_pitcher_spout":("-0.160 0 0.032",None),
                "rb_gripped_pitcher_foam":("-0.113 0 0.034",None),
            }
        else:
            offsets={
                "rb_gripped_portafilter_handle":("-0.085 0 -0.005","0.080 0.014 0.015"),
                "rb_gripped_portafilter_basket":("-0.175 0 -0.005",None)}
        for g in prop.findall("geom"):
            if g.get("name") in offsets:
                loc,size=offsets[g.get("name")]
                g.set("pos",loc)
                if size is not None:
                    g.set("size",size)
        handle.append(prop)

    old=b.find("body[@name='rb_gripped_cup']")
    if old is None:raise RuntimeError("rb_gripped_cup")
    prop=copy.deepcopy(old)
    prop.set("pos","0 0 .101")
    if cup_style=="handle_mug":
        # 90-degree side flange axis points +X; compensate cup's own
        # axial orientation so the vessel remains vertical at a level TCP.
        # Side adapter Ry(+90) + IR40 jaw roll Rz(+90) means
        # cup frame needs inverse quaternion when wrist is LEVEL:
        # this keeps mug's vertical axis world +Z and jaw closing
        # direction horizontal world Y, NOT vertically above/below.
        prop.set("quat","0.5 -0.5 -0.5 -0.5")
        shape=prop.find("geom[@name='rb_gripped_cup_body']")
        # 90 mm from jaw centre -> bowl; accessible negative-X mug handle
        # lies between fingers. Unlike old 116mm model, these match.
        shape.set("pos","0.090 0 0")
        shape.set("size",".041 .047")
        shape.set("rgba",".88 .84 .77 1")
        # The handle passes between the TWO IR40 side-tool jaws; no
        # below-cup lift is represented.
        for name,fr,to in [
            ("upper",(.009,0,.024),(.055,0,.024)),
            ("lower",(.009,0,-.027),(.055,0,-.027)),
            ("outer",(.009,0,-.027),(.009,0,.024)),
        ]:
            ET.SubElement(prop,"geom",{
                "name":"rb_gripped_cup_handle_"+name,"type":"capsule",
                "fromto":" ".join(map(str,(*fr,*to))),
                "size":".006", "rgba":".89 .85 .79 1",
                "mass":"0","contype":"0","conaffinity":"0"})
    cup.append(prop)
    return {"orientation_deg":90,"cup_style":cup_style,
            "portafilter_gripper":"rb_90_ir40_handle",
            "pitcher_gripper":"rb_90_ir40_handle",
            "cup_gripper":"rb_90_ir40_cup",
            "physical_force_grasp_simulated":False,
            "real_cad_geometry":False}


def insert_free_ceramic_cup_handle(world):
    free=world.find(".//body[@name='rb_plastic_cup']")
    if free is None:raise RuntimeError("Free cup missing")
    # Maintain original rb_plastic_cup object ID for coffee fixture logic.
    # This is a rendered ceramic mug in handle_mug mode.
    for g in free.findall("geom"):
        if g.get("name") in ("rb_cup_base","rb_plastic_wall_0",
                             "rb_plastic_wall_1","rb_plastic_wall_2",
                             "rb_plastic_rim"):
            g.set("rgba",".86 .83 .78 1")
    # Real cup's handle is at negative X. The orthogonal tool
    # approaches horizontally along +X into that handle from the left.
    for name,ends in (
        ("upper",(-.038,0,.082,-.082,0,.082)),
        ("lower",(-.038,0,.038,-.082,0,.038)),
        ("outer",(-.082,0,.038,-.082,0,.082))):
        ET.SubElement(free,"geom",{
            "name":"rb_free_mug_handle_"+name,"type":"capsule",
            "fromto":" ".join(map(str,ends)), "size":".006",
            "rgba":".87 .84 .78 1",
            "mass":"0","contype":"0","conaffinity":"0"})
