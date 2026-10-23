"""Physically plausible small AprilTag mount option for the 3-RGBD FR5 cell.

Default large-marker historical benchmark remains UNCHANGED. This module
constructs an ALTERNATIVE MJCF, and new camera replay must select
--marker-profile compact. It NEVER passes pose from the simulator to the
AprilTag detector: all real observations come from camera pixels.

Size includes white quiet zone, whereas OpenCV solvePnP needs ACTIVE black
tag width = outer_size * 440/512. The constant comes from barista_cell._assets.

Real hardware note: these are modelled flat rigid mounting plates, NOT a
verified food-safe mount or an adhesive that withstands hot steam/dishwashing.
"""
from __future__ import annotations
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from .barista_cell import build_cell_scene
from .scene import _nums
from .geometry import look_at_quat

# Physical dimensions of OUTER printed flat panel, including quiet zone.
PITCHER_SIDE_OUTER_M = .036
PITCHER_TOP_OUTER_M = .032
PITCHER_AUX_OUTER_M = .030
PLASTIC_CUP_OUTER_M = .028
PORTAFILTER_OUTER_M = .024
STATION_OUTER_M = .042
STATION_SMALL_OUTER_M = .036
ACTIVE_FRACTION = 440.0 / 512.0

# Transform of the AprilTag top-face frame in the body's rigid frame.
# Body X convention: cylinder's axis is Z, handle is +X, spout is -X.
PITCHER_MARKERS = {
    71: ((.083,.022,.120), (0.,0.,0.),PITCHER_TOP_OUTER_M),
    72: ((.085,-.028,.107),(math.radians(35),0.,0.),
         PITCHER_AUX_OUTER_M),
    23: ((0.,-.067,.080),(math.pi/2,0.,0.),
         PITCHER_SIDE_OUTER_M),
}
CUP_MARKER = ((0.,-.058,.105),(math.pi/2,0.,0.),PLASTIC_CUP_OUTER_M)

# Station mounts located on actual rigid prop/housing, not long stalks.
# Local position coordinates follow barista_cell's primitive assets.
STATION_PLACEMENT={
    # Flat on appliance TOPS or on the real handle. No flag above a 20cm pole.
    # Top-facing tags survive oblique overhead camera views much better than
    # the short front walls, where the espresso housings occlude corners.
    'grinder':('rb_grinder',(-.063,-.057,.284),(0.,0.,0.),.042),
    'portafilter':('rb_portafilter_stand',(0.,-.137,.102),
                    (0.,0.,0.),PORTAFILTER_OUTER_M),
    'espresso':('rb_espresso_machine',(-.142,-.010,.465),
                (0.,0.,0.),.042),
    'milk_dispenser':('rb_milk_dispenser',(.068,-.023,.323),
                     (0.,0.,0.),.042),
    'pitcher_stand':('rb_pitcher_stand',(.071,.044,.016),(0.,0.,0.),.036),
    'pour':('worldbody',(.310,-.575,.825),(0.,0.,0.),.036),
    'rinser':('rb_rinser',(.105,-.062,.065),(0.,0.,0.),.036),
    'robot_base':('worldbody',(.285,-.060,.822),(0.,0.,0.),.036),
}


def _must(root,path):
    obj=root.find(path)
    if obj is None:raise RuntimeError(f"Missing known CAD body/geom: {path}")
    return obj


def _panel(marker_body,tag_face,outer_m):
    """Add a THIN physical tab behind, flush with tag surface.

    Does not alter simulated collision/contact dynamics: finite plate is
    massless and noncolliding, just like old fiduciary meshes.
    """
    tag_face.set('size',_nums((outer_m/2,outer_m/2,.0001)))
    face_name=tag_face.attrib['name']
    ET.SubElement(marker_body,'geom',{
        'name':face_name.replace('rb_tagface_','rb_tag_backing_'),
        'type':'box','pos':'0 0 -.0028',
        'size':_nums((outer_m/2+.0015,outer_m/2+.0015,.0022)),
        'rgba':'.22 .27 .30 1','mass':'0','contype':'0','conaffinity':'0'})


def _mount_on_object(body,name,xyz,rpy,tag_face,outer_m,clip_length_m=None):
    body.set('pos',_nums(xyz))
    body.set('euler',_nums(rpy))
    # Local plate behind printed tag plus short small clip strut
    # extending toward the physical object; NOT a hovering 10cm panel.
    for geom in list(body.findall('geom')):
        if geom is not tag_face:body.remove(geom)
    _panel(body,tag_face,outer_m)
    if clip_length_m is not None:
        # The strut spans from the back of the marker to the actual
        # mug rim/handle or pitcher shell. No unsupported floating face.
        ET.SubElement(body,'geom',{
            'name':f'rb_clip_post_{name}',
            'type':'cylinder','pos':_nums((0,0,-clip_length_m/2)),
            'size':_nums((.003,clip_length_m/2)),
            'rgba':'.3 .34 .38 1','mass':'0',
            'contype':'0','conaffinity':'0'})


def build_compact_tag_cell(vendor_dir:Path, camera_count:int=3) -> Path:
    if camera_count not in (3,4):
        raise ValueError('Only 3 or 4 compact measurement cameras supported')
    original=Path(build_cell_scene(Path(vendor_dir)))
    tree=ET.parse(original)
    root=tree.getroot()
    world=_must(root,'worldbody')
    pitcher=_must(root,".//body[@name='rb_milk_pitcher']")
    pitcher_names={71:'rb_pitcher_top_marker',
                   72:'rb_pitcher_top_secondary_marker',
                   23:'rb_pitcher_side_marker'}
    for tid,(xyz,rpy,size) in PITCHER_MARKERS.items():
        marker=_must(pitcher,f".//body[@name='{pitcher_names[tid]}']")
        face=next((g for g in marker.iter('geom')
                   if g.attrib['name'].startswith('rb_tagface_')),None)
        if face is None:raise RuntimeError(f"Pitcher marker missing {tid}")
        _mount_on_object(marker,str(tid),xyz,rpy,face,size,
                         clip_length_m={71:.048,72:.034,23:.020}[tid])

    cup=_must(root,".//body[@name='rb_plastic_cup']")
    cm=_must(cup,".//body[@name='rb_cup_marker']")
    face=_must(cm,"geom[@name='rb_tagface_cell_plastic_cup']")
    _mount_on_object(cm,'cup',*CUP_MARKER[:2],face,CUP_MARKER[2],
                     clip_length_m=.015)
    for name,(target,xyz,rpy,size) in STATION_PLACEMENT.items():
        node=_must(world,f"body[@name='rb_tag_mount_{name}']")
        # Remove the post, tall pedestal and huge backing plate.
        world.remove(node)
        parent=world if target=='worldbody' else _must(
            world,f".//body[@name='{target}']")
        parent.append(node)
        face=_must(node,f"geom[@name='rb_tagface_{name}']")
        _mount_on_object(node,name,xyz,rpy,face,size)
    # For 30-36mm AprilTags, pixel footprint matters more than raw tag
    # dimension. A realistic closer three-camera rig at ~0.7-1.1m replaces
    # the widely spaced legacy camera coordinates ONLY in compact mode.
    compact_eyes={
        'cell_left':(-.74,-1.03,1.61),
        'cell_right':(.58,-1.08,1.55),
        'cell_back':(-.02,.42,1.69)}
    target=(-.10,-.30,1.04)
    for name,eye in compact_eyes.items():
        camera=_must(world,f"camera[@name='{name}']")
        camera.set('pos',_nums(eye))
        camera.set('quat',_nums(look_at_quat(eye,target)))
        camera.set('fovy','50')
    if camera_count==4:
        # Experimental fourth downward-looking viewpoint, NOT the cinematic
        # follow camera. Its station coverage must be measured rather than assumed.
        eye=(-.10,0.,1.93)
        target4=(-.08,0.,.98)
        ET.SubElement(world,'camera',{
            'name':'cell_overhead_aux',
            'pos':_nums(eye),
            'quat':_nums(look_at_quat(eye,target4)),
            'fovy':'70'})
    out=original.with_name('rb_artly_compact_tags_4cam_island.xml' if camera_count==4 else 'rb_artly_compact_tags_island.xml')
    tree.write(out,encoding='unicode')
    return out


def expected_compact_marker_transforms():
    """Return position+rigid-body orientation+ACTIVE width for camera BA.

    These values are CAD-mount constants. The BA must NOT read any moving
    simulator object pose to estimate a camera observation.
    """
    return {tid:(pos,rpy,outer*ACTIVE_FRACTION)
            for tid,(pos,rpy,outer) in PITCHER_MARKERS.items()}
