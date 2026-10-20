"""Build a lean FAIRINO FR5 RGB-D testbench from the existing MIT-licensed model."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import cv2
from .geometry import look_at_quat

FR5_URL = 'https://github.com/ma-weiss/Fairino-Mujoco.git'
FR5_COMMIT = '459570714fc68c92e1fb3605cb60b4d8ef84419b'
TAG_IDS = {'pitcher': 3, 'cup': 7, 'canister': 11, 'obstacle': 15, 'robot_tool': 19}
TAG_SIDE_M = 0.066   # visual geom / known square marker width
MARKER_TEXTURE_PX = 512
MARKER_QUIET_PX = 36
MARKER_ACTIVE_M = TAG_SIDE_M * (MARKER_TEXTURE_PX - 2 * MARKER_QUIET_PX) / MARKER_TEXTURE_PX
CAMERAS = {
    'rgbd_front_left': ([0.16, -0.86, 1.20], [0.56, 0.0, 0.67]),
    'rgbd_front_right': ([1.10, -0.52, 1.27], [0.54, 0.04, 0.66]),
    'rgbd_overhead': ([0.58, 0.36, 1.60], [0.55, 0.03, 0.63]),
}


def _nums(seq):
    return ' '.join(f'{float(n):.9g}' for n in seq)


def ensure_fr5(directory: Path) -> Path:
    """Only external code/assets are cloned; nothing added to original GitHub repo."""
    dst = directory / 'Fairino-Mujoco'
    if not (dst / 'fr5_robot.xml').exists():
        directory.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git', 'clone', FR5_URL, str(dst)], check=True)
    try:
        sha = subprocess.check_output(['git', '-C', str(dst), 'rev-parse', 'HEAD'], text=True).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError('FR5 assets must be a Git checkout') from exc
    if sha != FR5_COMMIT:
        subprocess.run(['git', '-C', str(dst), 'checkout', FR5_COMMIT], check=True)
    return dst


def write_markers(folder: Path):
    target = folder / 'rb_tags'
    target.mkdir(exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for tag_id in TAG_IDS.values():
        # A white quiet zone around the marker improves detection at a distance.
        side, margin = MARKER_TEXTURE_PX, MARKER_QUIET_PX
        pattern = cv2.aruco.generateImageMarker(dictionary, tag_id, side - 2*margin, borderBits=1)
        white = np.full((side, side), 255, dtype=np.uint8)
        white[margin:-margin, margin:-margin] = pattern
        Image.fromarray(white).convert('RGB').save(target / f'tag_{tag_id}.png')


def _tag(parent, name, tag_id, offset, tag_outer_side=TAG_SIDE_M):
    # Use a very thin box, not a plane: MuJoCo forbids plane geoms on moving
    # robot bodies (e.g. wrist3_link). Keep it non-colliding and massless.
    # MuJoCo box texcoords cover the upper face. The tag image also contains
    # a white quiet zone, leaving ~84% active marker width.
    geom = ET.SubElement(parent, 'geom', attrib={
        'name': f'rb_tagface_{name}', 'type': 'box', 'pos': _nums(offset),
        'size': _nums([tag_outer_side / 2, tag_outer_side / 2, 0.0001]),
        'material': f'rb_mat_{tag_id}', 'contype': '0', 'conaffinity': '0',
        'mass': '0', 'group': '0'})
    return geom


def _object_body(worldbody, name, xyz):
    return ET.SubElement(worldbody, 'body', {'name': f'rb_{name}', 'pos': _nums(xyz)})


def _static_box(worldbody, name, xyz, dims, rgba):
    b = _object_body(worldbody, name, xyz)
    ET.SubElement(b, 'geom', {'name': f'rb_{name}_geom', 'type': 'box',
        'size': _nums(np.array(dims)/2), 'rgba': rgba})
    return b


def generate_scene(root_dir: Path) -> Path:
    """Creates two new XML files inside a cloned FAIRINO asset folder.

    The originals stay untouched. ROS robot and meshes are sourced upstream.
    The graspable objects are kinematically fixed in v1 to enable repeatable
    pose tests; a subsequent learning phase can add a physical gripper.
    """
    fr = ensure_fr5(root_dir)
    write_markers(fr)
    robot = ET.parse(fr / 'fr5_robot.xml')
    robot_root = robot.getroot()
    asset = robot_root.find('asset')
    if asset is None:
        raise RuntimeError('Missing robot asset section')
    robot_model = asset.find("model[@name='gripper_model']")
    if robot_model is None:
        raise RuntimeError('Unsupported FR5 robot XML layout')
    robot_model.set('file', 'gripper_pgi.xml')
    # Attach the tracking marker on the free wrist side, not on pitcher's rim.
    wrist = robot_root.find(".//body[@name='wrist3_link']")
    if wrist is None:
        raise RuntimeError('Cannot locate FR5 wrist')
    tool_tag = ET.SubElement(wrist, 'body', {
        'name': 'rb_robot_tool_tag', 'pos': '0 0.085 0.07'})
    _tag(tool_tag, 'robot_tool', TAG_IDS['robot_tool'], [0, 0, 0])
    # Textures/materials are defined in the same included robot MJCF model, so
    # assets are visible both to the tool marker and the world objects.
    for label, tid in TAG_IDS.items():
        ET.SubElement(asset, 'texture', {'name': f'rb_tex_{tid}', 'type': '2d',
            'file': f'rb_tags/tag_{tid}.png'})
        ET.SubElement(asset, 'material', {'name': f'rb_mat_{tid}',
            'texture': f'rb_tex_{tid}', 'texrepeat':'1 1', 'texuniform':'false',
            'specular':'0', 'reflectance':'0'})
    # Upstream keyframes may not include optional gripper slide-joint qpos.
    # Avoid making the entire scene uncompilable if the nested model adds joints.
    keyframes = robot_root.find('keyframe')
    if keyframes is not None:
        robot_root.remove(keyframes)
    robot.write(fr / 'rb_fr5_robot.xml', encoding='unicode')

    root = ET.Element('mujoco', {'model':'RoboBarista RGBD three cameras'})
    ET.SubElement(root, 'include', {'file':'rb_fr5_robot.xml'})
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', {'offwidth':'1280','offheight':'960'})
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', {'pos':'0.50 0 1.75','dir':'0 0 -1',
                                 'directional':'true','diffuse':'1 1 1'})
    ET.SubElement(world, 'geom', {'name':'rb_floor', 'type':'plane', 'size':'2 2 0.1', 'rgba':'0.75 0.76 0.76 1'})
    ET.SubElement(world, 'geom', {'name':'rb_counter', 'type':'box',
        'size': '0.43 0.37 0.035', 'pos': '0.57 0 0.415', 'rgba':'0.30 0.33 0.36 1'})

    # Stable scene locations, all in meters. Objects deliberately immobile in v1.
    cup = _object_body(world, 'cup', [.53, .15, .452])
    ET.SubElement(cup, 'geom', {'name':'rb_cup_ceramic', 'type':'cylinder',
        'pos':'0 0 0.052', 'size':'0.047 0.052', 'rgba':'0.93 0.91 0.85 1'})
    ET.SubElement(cup, 'geom', {'name':'rb_cup_coffee', 'type':'cylinder',
        'pos':'0 0 0.101', 'size':'0.042 0.001', 'rgba':'0.29 0.13 0.08 1'})
    ET.SubElement(cup, 'geom', {'name':'rb_cup_handle', 'type':'capsule',
        'fromto':'0.041 0.0 0.07 0.088 0.0 0.07', 'size':'0.009', 'rgba':'0.92 0.91 0.87 1'})
    # Tag on a small holder next to rim. Its known local pose is explicit.
    _tag(cup, 'cup', TAG_IDS['cup'], [0.072, 0.0, 0.112])

    p = _object_body(world, 'pitcher', [.67, -.14, .452])
    ET.SubElement(p, 'geom', {'name':'rb_pitcher_metal', 'type':'cylinder',
        'pos':'0 0 0.073', 'size':'0.05 0.073', 'rgba':'0.68 0.76 0.80 1'})
    ET.SubElement(p, 'geom', {'name':'rb_pitcher_spout', 'type':'box',
        'pos':'-0.051 0 0.136', 'size':'0.018 0.016 0.005','rgba':'0.66 0.74 0.79 1'})
    ET.SubElement(p, 'geom', {'name':'rb_pitcher_handle', 'type':'capsule',
        'fromto':'0.049 0 0.112 0.11 0 0.112','size':'0.012','rgba':'0.65 0.73 0.78 1'})
    # Tag not inside the cup/pitcher. Face above the pitcher for optics.
    _tag(p, 'pitcher', TAG_IDS['pitcher'], [0, 0, 0.155])

    canister = _static_box(world,'canister',[.35,-.20,.502],[.10,.085,.10],'.48 .62 .74 1')
    _tag(canister, 'canister', TAG_IDS['canister'], [0,0,.053])
    obstacle = _static_box(world,'obstacle',[.38,.20,.517],[.08,.10,.13],'.65 .43 .29 1')
    _tag(obstacle, 'obstacle', TAG_IDS['obstacle'], [0,0,.068])

    for name,(eye,target) in CAMERAS.items():
        q = look_at_quat(eye, target)
        ET.SubElement(world, 'camera', {'name':name, 'pos': _nums(eye),
            'quat':_nums(q), 'fovy':'46'})
    xmlfile = fr / 'rb_scene.xml'
    ET.ElementTree(root).write(xmlfile, encoding='unicode')
    return xmlfile


def tag_definitions():
    """Return tag ID -> (body label, marker-to-object known rigid transform).

    Tag coordinate axes must correspond to printed texture axes; renderer UV
    orientation is checked against GT (especially angle errors) on first run.
    """
    positions = {
        'cup': [0.072, 0, .112],
        'pitcher': [0, 0, .155],
        'canister': [0, 0, .053],
        'obstacle': [0, 0, .068],
        'robot_tool': [0,0,0],
    }
    output = {}
    for name, tag_id in TAG_IDS.items():
        T = np.eye(4)
        T[:3,3] = positions[name]
        output[tag_id] = (name, T)
    return output
