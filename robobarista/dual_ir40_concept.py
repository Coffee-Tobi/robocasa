"""Parametric *visual concept* of two IR40-50 grippers on a dual-tool bracket.

IMPORTANT: Approximate dimensions. NOT vendor CAD, mass/inertia calibration,
gripper kinematics, a printable STL or verified mounting-hole geometry.
"""
from __future__ import annotations
import xml.etree.ElementTree as ET

GRIPPER_COUNT=2
MODEL="IR40-50"
FORCE_RANGE_N=(2.,50.)
STROKE_MM=40.
# Local robot TCP frame; concept-only offsets (metres).
MOUNTS={"handle_tool":(-.070,0.,.040),"cup_tool":(.070,0.,.040)}

def add_visual_dual_ir40_50(tcp:ET.Element,cup_diameter_mm:float=80.) -> None:
    """Add clearly named non-colliding approximate concept geometries to TCP."""
    if not 55<=cup_diameter_mm<=100:raise ValueError('supported cup concept: 55-100 mm')
    def body(parent,name,pos):
        return ET.SubElement(parent,"body",name=name,pos=" ".join(map(str,pos)))
    def box(parent,name,pos,size,rgba):
        return ET.SubElement(parent,"geom",{
            "name":name,"type":"box","pos":" ".join(map(str,pos)),
            "size":" ".join(map(str,size)),"rgba":rgba,
            "mass":"0","contype":"0","conaffinity":"0"})
    for tool,pos in MOUNTS.items():
        unit=body(tcp,"rb_"+tool+"_IR40_50",pos)
        box(unit,"rb_"+tool+"_housing",(0,0,.025),(.020,.024,.025),".24 .28 .32 1")
        if tool=="handle_tool":
            # Straight, separately replaceable jaws: later shape with real grip STEP.
            for sign in (-1,1):
                box(unit,"rb_handle_jaw_"+str(sign),(sign*.018,0,.080),
                    (.006,.013,.034),".18 .73 .77 1")
        else:
            # Two opposing approximate 3D-printed curved-jaw envelopes.
            # Curvature must be designed from the cup's actual taper.
            span=cup_diameter_mm/2000+.006
            for sign in (-1,1):
                box(unit,"rb_cup_jaw_"+str(sign),(sign*span,0,.080),
                    (.006,.027,.022),".93 .60 .24 1")

def build_dual_gripper_scene(vendor_dir):
    """Separate visual variant; existing production benchmark is unchanged."""
    from pathlib import Path
    from .camera_joint_replay import _add_held_pitcher_robot_variant
    original=Path(_add_held_pitcher_robot_variant(Path(vendor_dir).resolve()))
    tree=ET.parse(original)
    # MJCF include has robot in separate XML; clone robot into a *new* file.
    robot=(original.parent / tree.getroot().find('include').get('file'))
    rt=ET.parse(robot)
    tcp=rt.getroot().find(".//body[@name='rb_cell_tcp']")
    if tcp is None:raise ValueError('FR5 TCP missing')
    add_visual_dual_ir40_50(tcp)
    robot_out=robot.with_name('rb_fr5_dual_ir40_robot.xml')
    rt.write(robot_out,encoding='unicode')
    tree.getroot().find('include').set('file',robot_out.name)
    out=original.with_name('rb_fr5_dual_ir40_cell.xml')
    tree.write(out,encoding='unicode')
    return out
