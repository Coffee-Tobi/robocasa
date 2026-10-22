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

def add_visual_dual_ir40_50(tcp:ET.Element,cup_diameter_mm:float=80.,include_props=False) -> None:
    """Add approximate non-colliding gripper geometry; optionally rigid held tools. No grasp physics."""
    if not 55<=cup_diameter_mm<=100:raise ValueError('supported cup concept: 55-100 mm')
    def body(parent,name,pos):
        return ET.SubElement(parent,"body",name=name,pos=" ".join(map(str,pos)))
    def box(parent,name,pos,size,rgba):
        return ET.SubElement(parent,"geom",{
            "name":name,"type":"box","pos":" ".join(map(str,pos)),
            "size":" ".join(map(str,size)),"rgba":rgba,
            "mass":"0","contype":"0","conaffinity":"0"})
    mounts={}
    for tool,pos in MOUNTS.items():
        unit=body(tcp,"rb_"+tool+"_IR40_50",pos)
        unit.set("euler","0 -90 0")  # jaws extend forward (robot TCP -X), not vertically
        mounts[tool]=unit
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

    if include_props:
        # Pre-calibrated kinematic attachments: prop is located inside the
        # fingers of the IR40-50, NOT sitting on the naked wrist.
        # Their rigid joint does not represent actual grasp forces or friction.
        handle=mounts["handle_tool"]
        jug=body(handle,"rb_gripped_pitcher",(0,0,.080))
        jug.set("euler","0 90 0")  # counter-rotate for upright object
        box(jug,"rb_gripped_pitcher_adapter",(0,0,0),(.012,.013,.014),".83 .71 .21 1")
        box(jug,"rb_gripped_pitcher_stop_lower",(0,0,-.023),(.019,.021,.004),".83 .71 .21 1")
        box(jug,"rb_gripped_pitcher_stop_upper",(0,0,.023),(.019,.021,.004),".83 .71 .21 1")
        box(jug,"rb_gripped_pitcher_bridge",(-.055,0,-.008),(.061,.010,.010),".34 .42 .44 1")
        ET.SubElement(jug,"geom",name="rb_gripped_pitcher_body",type="cylinder",
            pos="-.113 0 -.022",size=".044 .055",rgba=".50 .72 .81 1",
            mass="0",contype="0",conaffinity="0")
        ET.SubElement(jug,"geom",name="rb_gripped_pitcher_rim",type="cylinder",
            pos="-.113 0 .033",size=".047 .003",rgba=".70 .79 .83 1",
            mass="0",contype="0",conaffinity="0")
        box(jug,"rb_gripped_pitcher_spout",(-.160,0,.032),(.018,.016,.004),".15 .89 .92 1")
        ET.SubElement(jug,"geom",name="rb_gripped_pitcher_foam",type="cylinder",
            pos="-.113 0 .034",size=".040 .002",rgba=".98 .96 .85 0",
            mass="0",contype="0",conaffinity="0")
        pf=body(handle,"rb_gripped_portafilter",(0,0,.080))
        pf.set("euler","0 90 0")  # counter-rotate for upright object
        box(pf,"rb_gripped_portafilter_adapter",(0,0,0),(.012,.013,.014),".75 .65 .22 1")
        box(pf,"rb_gripped_portafilter_handle",(-.085,0,-.005),(.080,.014,.015),".29 .29 .31 1")
        ET.SubElement(pf,"geom",name="rb_gripped_portafilter_basket",type="cylinder",
            pos="-.175 0 -.005",size=".043 .020",rgba=".7 .72 .74 1",
            mass="0",contype="0",conaffinity="0")
        cup=body(mounts["cup_tool"],"rb_gripped_cup",(0,0,.080))
        cup.set("euler","0 90 0")  # counter-rotate for upright object
        ET.SubElement(cup,"geom",name="rb_gripped_cup_body",type="cylinder",
            pos="0 0 0",size=f"{cup_diameter_mm/2000} .047",
            rgba=".72 .87 .93 .82",mass="0",contype="0",conaffinity="0")

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
    add_visual_dual_ir40_50(tcp,include_props=True)
    robot_out=robot.with_name('rb_fr5_dual_ir40_robot.xml')
    rt.write(robot_out,encoding='unicode')
    tree.getroot().find('include').set('file',robot_out.name)
    out=original.with_name('rb_fr5_dual_ir40_cell.xml')
    tree.write(out,encoding='unicode')
    return out
