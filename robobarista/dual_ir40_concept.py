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

def add_visual_dual_ir40_50(tcp:ET.Element,cup_diameter_mm:float=80.,include_props=False,compact_tags=False,cup_style='takeaway') -> None:
    if cup_style not in ('takeaway','handle_mug'):
        raise ValueError('cup_style must be takeaway or handle_mug')
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
            if cup_style=='handle_mug':
                # Narrow opposed fingers grasp the *handle stem* near local y=0.
                # The ceramic vessel is offset to y=+.12 m, clear of the
                # jaws. Old takeaway-style clamping around the base is NOT used.
                for sign in (-1,1):
                    box(unit,"rb_cup_jaw_"+str(sign),(sign*.013,.011,.080),
                        (.006,.014,.024),".93 .60 .24 1")
            else:
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
        box(jug,"rb_gripped_pitcher_adapter",(0,0,0),(.012,.013,.014),".83 .71 .21 1")
        box(jug,"rb_gripped_pitcher_stop_lower",(0,0,-.023),(.019,.021,.004),".83 .71 .21 1")
        box(jug,"rb_gripped_pitcher_stop_upper",(0,0,.023),(.019,.021,.004),".83 .71 .21 1")
        box(jug,"rb_gripped_pitcher_bridge",(0,.055,-.008),(.010,.061,.010),".34 .42 .44 1")
        ET.SubElement(jug,"geom",name="rb_gripped_pitcher_body",type="cylinder",
            pos="0 .113 -.022",size=".044 .055",rgba=".50 .72 .81 1",
            mass="0",contype="0",conaffinity="0")
        ET.SubElement(jug,"geom",name="rb_gripped_pitcher_rim",type="cylinder",
            pos="0 .113 .033",size=".047 .003",rgba=".70 .79 .83 1",
            mass="0",contype="0",conaffinity="0")
        box(jug,"rb_gripped_pitcher_spout",(0,.160,.032),(.018,.016,.004),".15 .89 .92 1")
        ET.SubElement(jug,"geom",name="rb_gripped_pitcher_foam",type="cylinder",
            pos="0 .113 .034",size=".040 .002",rgba=".98 .96 .85 0",
            mass="0",contype="0",conaffinity="0")
        pf=body(handle,"rb_gripped_portafilter",(0,0,.080))
        box(pf,"rb_gripped_portafilter_adapter",(0,0,0),(.012,.013,.014),".75 .65 .22 1")
        box(pf,"rb_gripped_portafilter_handle",(0,.085,-.005),(.014,.080,.015),".29 .29 .31 1")
        ET.SubElement(pf,"geom",name="rb_gripped_portafilter_basket",type="cylinder",
            pos="0 .175 -.005",size=".043 .020",rgba=".7 .72 .74 1",
            mass="0",contype="0",conaffinity="0")
        cup=body(mounts["cup_tool"],"rb_gripped_cup",(0,0,.080))
        if cup_style=='handle_mug':
            # Cup centre displaced AWAY from gripper jaws: y=+120 mm.
            # Ring handle lies at the jaws and connects toward the cup.
            ET.SubElement(cup,"geom",name="rb_gripped_cup_body",
                type="cylinder",pos="0 .120 0",
                size=".041 .047",rgba=".88 .84 .77 1",
                mass="0",contype="0",conaffinity="0")
            for part,coords in (
                ('upper',(0,.016,.026,0,.085,.026)),
                ('lower',(0,.016,-.027,0,.085,-.027)),
                ('outer',(0,.016,-.027,0,.016,.026))):
                ET.SubElement(cup,"geom",
                    name="rb_gripped_cup_handle_"+part,
                    type="capsule",fromto=" ".join(map(str,coords)),
                    size=".006",rgba=".89 .85 .79 1",
                    mass="0",contype="0",conaffinity="0")
        else:
            ET.SubElement(cup,"geom",name="rb_gripped_cup_body",
                type="cylinder",pos="0 0 0",
                size=f"{cup_diameter_mm/2000} .047",
                rgba=".72 .87 .93 .82",mass="0",
                contype="0",conaffinity="0")
        if compact_tags:
            # The held props in the storyboard are RIGID CLONES of the free
            # props. Give them UNIQUE AprilTag IDs to avoid duplicate-ID
            # detections from a parked clone elsewhere in the workcell.
            # Sizes INCLUDE quiet zone; corner solvers must use 440/512.
            def attach_tag(parent,name,tid,position,rpy,outer,post_m=0.):
                mount=ET.SubElement(parent,"body",name="rb_small_tag_"+name,
                    pos=" ".join(map(str,position)),
                    euler=" ".join(map(str,rpy)))
                ET.SubElement(mount,"geom",name="rb_tagface_"+name,
                    type="box",pos="0 0 0",
                    size=f"{outer/2} {outer/2} .0001",
                    material=f"rb_mat_{tid}",mass="0",contype="0",
                    conaffinity="0")
                ET.SubElement(mount,"geom",name="rb_tag_backing_"+name,
                    type="box",pos="0 0 -.0028",
                    size=f"{outer/2+.0015} {outer/2+.0015} .0022",
                    rgba=".27 .30 .32 1",mass="0",contype="0",
                    conaffinity="0")
                if post_m:
                    ET.SubElement(mount,"geom",name="rb_small_clip_"+name,
                        type="cylinder",pos=f"0 0 {-post_m/2}",
                        size=f".003 {post_m/2}",rgba=".35 .36 .39 1",
                        mass="0",contype="0",conaffinity="0")
            from math import pi
            # Pitcher cylinder: local centre (0,.113,-.022), radius .044.
            attach_tag(jug,"held_pitcher",75,(0,.055,-.008),
                       (pi/2,0,0),.036,post_m=.014)
            # Portafilter handle: long axis local +Y; top flat clip.
            attach_tag(pf,"held_portafilter",74,(0,.085,.014),
                       (0,0,0),.024)
            # Cup: removable food-safe clip concept on outside rim.
            attach_tag(cup,"held_cup",76,
                       (0,.064,.042) if cup_style=='handle_mug'
                       else (0,-.054,.042),
                       (pi/2,0,0),.028,post_m=.014)

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
