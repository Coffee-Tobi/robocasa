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
        # Jaws of the 90-degree gripper are at z=.111, not the old .080.
        prop.set("pos","0 0 .111")
        handle.append(prop)

    old=b.find("body[@name='rb_gripped_cup']")
    if old is None:raise RuntimeError("rb_gripped_cup")
    prop=copy.deepcopy(old)
    prop.set("pos","0 0 .111")
    if cup_style=="handle_mug":
        # 90-degree side flange axis points +X; compensate cup's own
        # axial orientation so the vessel remains vertical at a level TCP.
        prop.set("euler","0 -1.57079632679 0")
        shape=prop.find("geom[@name='rb_gripped_cup_body']")
        shape.set("pos","0 .116 0")
        shape.set("size",".041 .047")
        shape.set("rgba",".88 .84 .77 1")
        # The handle passes between the TWO IR40 side-tool jaws; no
        # below-cup lift is represented.
        for name,fr,to in [
            ("upper",(0,.012,.024),(0,.079,.024)),
            ("lower",(0,.012,-.027),(0,.079,-.027)),
            ("outer",(0,.012,-.027),(0,.012,.024)),
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
    for name,ends in (
        ("upper",(.038,0,.082,.082,0,.082)),
        ("lower",(.038,0,.038,.082,0,.038)),
        ("outer",(.082,0,.038,.082,0,.082))):
        ET.SubElement(free,"geom",{
            "name":"rb_free_mug_handle_"+name,"type":"capsule",
            "fromto":" ".join(map(str,ends)), "size":".006",
            "rgba":".87 .84 .78 1",
            "mass":"0","contype":"0","conaffinity":"0"})
