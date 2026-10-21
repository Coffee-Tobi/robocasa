"""Artly-inspired compact RoboBarista island: physical FR5 + primitive props.

This is a NEW scene, not a change to older camera/IK benchmark scenes.
Artly's precise hardware measurements are not publicly specified; placements
are our own engineering concept, not a traced copy of its workcell.

Frames: table center = (0,0), all positions m, table top z = 0.82.
Tamper has a real prismatic joint + position actuator; pitcher and practice
plastic cup have free joints and inertial/collision proxies. No coffee/liquid
or motorized espresso extraction is implemented.
"""
from __future__ import annotations
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image
from .scene import generate_scene, _nums, _tag, SIDE_PITCHER_TAG_IDS
from .geometry import look_at_quat

TABLE_TOP=0.82
ROBOT_BASE=(0.0,0.0,0.91)
STATIONS={
    'grinder': (-0.56, 0.25,1.02),
    'tamper': (-0.38,0.49,1.03),
    'portafilter': (-0.58,-0.18,0.91),
    'espresso': (0.49,0.43,1.12),
    'milk_dispenser': (0.60,-0.36,1.06),
    'pitcher': (-0.15,-0.55,0.94),
    'pour': (0.35,-0.37,0.96),
    'rinser': (0.07,0.59,0.89),
}
# Static mounts: raised top-oriented marker on exposed station edge, away from
# the machine enclosure. Include redundant markers for two dynamic props.
TAG_ID={
    'grinder':61,'tamper':62,'portafilter':63,'espresso':64,
    'milk_dispenser':65,'pitcher_stand':66,'pour':67,'rinser':68,
    'robot_base':69,'plastic_cup':70,'pitcher_top':71,
}
CAMERAS={
    'cell_left':((-1.12,-1.0,2.30),(0,0,1.02)),
    'cell_right':((1.12,-1.0,2.30),(0,0,1.02)),
    'cell_back':((0,1.05,2.34),(0,-0.07,1.04)),
}


def _geom(parent,name,shape,pos,size,rgba,*,mass=0,collide=False,extra=None):
    attrib=dict(name=name,type=shape,pos=_nums(pos),size=_nums(size),
                rgba=rgba,contype='1' if collide else '0',
                conaffinity='1' if collide else '0',mass=str(float(mass)))
    attrib.update(extra or {})
    return ET.SubElement(parent,'geom',attrib)


def _box(parent,name,pos,half,rgba,*,collide=False,mass=0):
    return _geom(parent,name,'box',pos,half,rgba,collide=collide,mass=mass)


def _body(parent,name,xyz):
    return ET.SubElement(parent,'body',{'name':name,'pos':_nums(xyz)})


def _marker(root,label,xy,z,tag_id,face_size=0.075):
    # A raised, unobstructed fiducial on its *own* mounting bracket.
    # This marker is a fixed frame, not a moving-object measurement.
    x,y=xy
    mount=_body(root,'rb_tag_mount_'+label,[x,y,z])
    stem_height=z-TABLE_TOP
    if stem_height>.04:
        # Thin physical support so a fiducial is never a floating plate.
        _geom(mount,'rb_tag_stem_'+label,'cylinder',
              [0,0,-stem_height/2],[.007,stem_height/2],
              '.25 .29 .32 1')
    _box(mount,'rb_tag_plate_'+label,[0,0,-.007],
         [face_size/2+.008,face_size/2+.008,.006],'.94 .94 .92 1')
    _tag(mount,label,tag_id,[0,0,0],tag_outer_side=face_size)
    return mount


def _assets(fr):
    robot=ET.parse(fr/'rb_fr5_robot.xml')
    root=robot.getroot()
    assets=root.find('asset')
    for label,tid in TAG_ID.items():
        # All new markers use a single AprilTag36h11 family (not ArUco).
        name='rb_tags/tag_%d.png'%tid
        file=fr/name
        arr=np.full((512,512),255,dtype=np.uint8)
        code=cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            tid,440,borderBits=1)
        arr[36:-36,36:-36]=code
        Image.fromarray(arr).convert('RGB').save(file)
        ET.SubElement(assets,'texture',{'name':f'rb_tex_{tid}','type':'2d','file':name})
        ET.SubElement(assets,'material',{'name':f'rb_mat_{tid}',
            'texture':f'rb_tex_{tid}','texrepeat':'1 1',
            'texuniform':'false','specular':'0','reflectance':'0'})
    base=root.find(".//body[@name='base_link']")
    if base is None: raise RuntimeError('Cannot find FR5 base')
    base.set('pos',_nums(ROBOT_BASE))
    # Auxiliary massless reference body on robot wrist for future IK tasks.
    wrist=root.find(".//body[@name='wrist3_link']")
    if wrist is None: raise RuntimeError('Cannot find FR5 wrist')
    ref=_body(wrist,'rb_cell_tcp',[0,0,.14])
    ET.SubElement(ref,'site',{'name':'rb_cell_tcp_site','pos':'0 0 0','size':'.008',
                             'rgba':'.1 1 .1 1'})
    robot_file=fr/'rb_fr5_cell_robot.xml'
    robot.write(robot_file,encoding='unicode')
    return robot_file


def build_cell_scene(vendor_dir: Path) -> Path:
    vendor_dir=Path(vendor_dir).resolve()
    # Existing pinned upstream FR5/assets and marker textures are reused.
    baseline=generate_scene(vendor_dir,pitcher_side_tags=True,
                            camera_count=3,pitcher_tag_count=1)
    fr=baseline.parent
    robot_file=_assets(fr)
    root=ET.Element('mujoco',{'model':'RoboBarista central island'})
    ET.SubElement(root,'include',{'file':robot_file.name})
    visual=ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',{'offwidth':'2048','offheight':'1536'})
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',{'pos':'0 0 2.8','dir':'0 0 -1',
        'diffuse':'1 1 1','directional':'true'})
    ET.SubElement(world,'light',{'pos':'-1 -0.6 2','dir':'.3 .3 -1',
        'diffuse':'.6 .6 .6','directional':'true'})
    _geom(world,'rb_cell_floor','plane',[0,0,0],[2.0,1.8,.1],
          '.65 .65 .64 1',collide=True)
    # Low room perimeter; do not place walls inside camera rays.
    for name,pos,half,rgba in [
        ('back',[0,1.34,1.07],[1.65,.045,1.07],'.77 .74 .71 1'),
        ('left',[-1.63,0,1.07],[.045,1.34,1.07],'.75 .73 .71 1'),
        ('right',[1.63,0,1.07],[.045,1.34,1.07],'.76 .73 .69 1')
    ]:
        _box(world,'rb_room_'+name,pos,half,rgba)
    # Freestanding island. Table top surface = z .82.
    _box(world,'rb_island_top',[0,0,.79],[.95,.79,.03],
         '.33 .39 .42 1',collide=True)
    _box(world,'rb_island_body',[0,0,.405],[.89,.73,.38],
         '.38 .40 .40 1')
    _geom(world,'rb_robot_pedestal','cylinder',[0,0,.865],
          [.112,.045],'.16 .18 .21 1',collide=True)
    # Espresso machine: front-right, facing inner work area.
    espresso=_body(world,'rb_espresso_machine',[.49,.43,TABLE_TOP])
    _box(espresso,'rb_espresso_body',[0,.055,.235],
         [.22,.17,.225],'.56 .60 .63 1',collide=True)
    _box(espresso,'rb_espresso_front',[0,-.118,.23],
         [.19,.012,.18],'.19 .22 .25 1')
    _box(espresso,'rb_espresso_drip_tray',[0,-.155,.044],
         [.18,.095,.016],'.33 .35 .36 1')
    _geom(espresso,'rb_brew_head','cylinder',[-.05,-.18,.27],
          [.033,.028],'.18 .19 .2 1')
    _geom(espresso,'rb_steam_wand','capsule',[.15,-.172,.143],
          [.009,.075],'.8 .8 .82 1',extra={'euler':'0 1.2 0'})
    # Grinder and automatic tamper, separated so both are reachable.
    grinder=_body(world,'rb_grinder',[-.56,.25,TABLE_TOP])
    _box(grinder,'rb_grinder_housing',[0,0,.14],[.092,.09,.14],
         '.19 .21 .24 1',collide=True)
    _geom(grinder,'rb_grinder_hopper','cylinder',[0,0,.34],
          [.074,.085],'.35 .26 .19 .74')
    _geom(grinder,'rb_grinder_dispense','cylinder',[0,-.092,.108],
          [.03,.018],'.10 .1 .1 1')
    tamper=_body(world,'rb_tamper',[-.38,.49,TABLE_TOP])
    _box(tamper,'rb_tamper_frame',[0,.04,.162],[.090,.078,.160],
         '.24 .28 .31 1',collide=True)
    _geom(tamper,'rb_tamper_puck','cylinder',[0,-.035,.080],
          [.033,.012],'.43 .28 .15 1')
    shaft=_body(tamper,'rb_tamper_press',[0,-.035,.255])
    ET.SubElement(shaft,'joint',{'name':'rb_tamper_slide','type':'slide',
        'axis':'0 0 -1','range':'0 .055','limited':'true',
        'damping':'4','armature':'.001'})
    _geom(shaft,'rb_tamper_shaft','cylinder',[0,0,.008],
          [.016,.09],'.80 .83 .84 1',mass=.18,collide=False)
    _geom(shaft,'rb_tamper_foot','cylinder',[0,0,-.081],
          [.032,.009],'.62 .66 .68 1',mass=.14,collide=False)
    actuators=ET.SubElement(root,'actuator')
    ET.SubElement(actuators,'position',{'name':'rb_tamper_motor',
       'joint':'rb_tamper_slide','kp':'300','ctrlrange':'0 .055',
       'ctrllimited':'true'})
    # Portafilter resting in a simple parked cradle near grinder.
    pf=_body(world,'rb_portafilter_stand',[-.58,-.18,TABLE_TOP])
    _box(pf,'rb_portafilter_cradle',[0,0,.036],[.095,.083,.036],
         '.19 .2 .22 1')
    _geom(pf,'rb_portafilter_basket','cylinder',[0,0,.078],
          [.048,.019],'.70 .72 .72 1')
    _geom(pf,'rb_portafilter_handle','capsule',[.12,0,.084],
          [.015,.12],'.16 .17 .18 1',
          extra={'euler':'0 1.5708 0'})
    # Chilled milk dosing station. No simulated flow yet.
    milk=_body(world,'rb_milk_dispenser',[.6,-.36,TABLE_TOP])
    _box(milk,'rb_milk_body',[0,.02,.16],[.105,.11,.16],
         '.82 .83 .83 1')
    _geom(milk,'rb_milk_nozzle','cylinder',[-.03,-.116,.196],
          [.012,.06],'.42 .43 .45 1')
    # Pitcher on a parking pad: an actual movable MJCF free body.
    pitcher_stand=_body(world,'rb_pitcher_stand',[-.15,-.55,TABLE_TOP])
    _box(pitcher_stand,'rb_pitcher_pad',[0,0,.007],[.093,.086,.007],
         '.2 .24 .27 1')
    pitcher=_body(world,'rb_milk_pitcher',[-.15,-.55,TABLE_TOP+.014])
    ET.SubElement(pitcher,'freejoint',{'name':'rb_pitcher_free'})
    _geom(pitcher,'rb_pitcher_body','cylinder',[0,0,.064],
          [.048,.064],'.69 .77 .81 1',mass=.20,collide=True)
    _geom(pitcher,'rb_pitcher_spout','box',[-.053,0,.117],
          [.020,.018,.006],'.76 .82 .84 1')
    _geom(pitcher,'rb_pitcher_handle','capsule',[.079,0,.07],
          [.012,.043],'.28 .30 .31 1',extra={'euler':'0 1.5708 0'})
    # Pitcher top tab + one side tag, redundant view angles while moving.
    top=_body(pitcher,'rb_pitcher_top_marker',[-.065,0,.135])
    _tag(top,'cell_pitcher_top',TAG_ID['pitcher_top'],[0,0,0],tag_outer_side=.090)
    # Apriltag side with marker-local Z normal toward camera left/front.
    side=_body(pitcher,'rb_pitcher_side_marker',[0,-.058,.083])
    side.set('euler','1.570796 0 0')
    _tag(side,'cell_pitcher_side',SIDE_PITCHER_TAG_IDS['pitcher_near'],
         [0,0,0],tag_outer_side=.082)
    # Clear/lightweight practice cup, not a ceramic mug. Moving free body.
    pour=_body(world,'rb_plastic_cup',[.35,-.37,TABLE_TOP+.003])
    ET.SubElement(pour,'freejoint',{'name':'rb_plastic_cup_free'})
    _geom(pour,'rb_cup_base','cylinder',[0,0,.003],
          [.036,.003],'.89 .95 .99 .85',mass=.015,collide=True)
    for k,(radius,z,half) in enumerate([
        (.038,.022,.019),(.041,.058,.017),(.044,.09,.015)
    ]):
        _geom(pour,f'rb_plastic_wall_{k}','cylinder',[0,0,z],
              [radius,half],'.88 .94 .98 .30')
    _geom(pour,'rb_plastic_rim','cylinder',[0,0,.107],
          [.045,.003],'.9 .95 .98 .68')
    # Corner tab at cup rim, with explicit marker; cup is movable.
    cup_tag=_body(pour,'rb_cup_marker',[.083,-.013,.118])
    _tag(cup_tag,'cell_plastic_cup',TAG_ID['plastic_cup'],[0,0,0],
         tag_outer_side=.090)
    rinser=_body(world,'rb_rinser',[.07,.59,TABLE_TOP])
    _box(rinser,'rb_rinser_tray',[0,0,.025],
         [.135,.095,.02],'.35 .36 .37 1')
    _geom(rinser,'rb_rinser_bowl','cylinder',[0,0,.046],
          [.067,.011],'.12 .19 .21 1')
    _geom(rinser,'rb_rinser_jet','cylinder',[0,0,.059],
          [.017,.014],'.73 .73 .74 1')
    # One spare, unmarked cup in its station; no ceramic cup anywhere.
    spare=_body(world,'rb_spare_plastic_cup',[.48,-.53,TABLE_TOP])
    _geom(spare,'rb_spare_plastic_wall','cylinder',[0,0,.051],
          [.044,.052],'.86 .93 1 .30')
    _geom(spare,'rb_spare_plastic_rim','cylinder',[0,0,.103],
          [.045,.002],'.92 .96 1 .65')
    # Dedicated elevated fiducials in open corners, outside working stroke.
    marker_xy={
        'grinder':(-.73,.24),'tamper':(-.36,.65),
        'portafilter':(-.75,-.18),'espresso':(.72,.52),
        'milk_dispenser':(.76,-.36),
        'pitcher_stand':(-.27,-.64),'pour':(.30,-.58),
        'rinser':(.08,.74),'robot_base':(.18,.055),
    }
    for name,(x,y) in marker_xy.items():
        # Markers stand ABOVE machine/cup rims and have enough pixel area
        # for AprilTag 36h11 decoding in our current 920x660 RGB views.
        heights={'espresso':1.48,'grinder':1.43,'tamper':1.33,
                 'portafilter':1.23,'milk_dispenser':1.40,
                 'pitcher_stand':1.28,'pour':1.25,'rinser':1.23,
                 'robot_base':1.22}
        height=heights[name]
        _marker(world,name,(x,y),height,TAG_ID[name],face_size=.105)
    for name,(eye,target) in CAMERAS.items():
        q=look_at_quat(eye,target)
        ET.SubElement(world,'camera',{'name':name,'pos':_nums(eye),
           'quat':_nums(q),'fovy':'58'})
    # An elevated 3/4 view of the whole workcell for pretty visualization.
    eye=(1.55,-1.55,2.33)
    target=(0,0,.95)
    ET.SubElement(world,'camera',{'name':'cell_overview',
        'pos':_nums(eye),'quat':_nums(look_at_quat(eye,target)),'fovy':'58'})
    out=fr/'rb_artly_inspired_island.xml'
    ET.ElementTree(root).write(out,encoding='unicode')
    return out
