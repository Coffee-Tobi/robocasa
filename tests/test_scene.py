import ast
import xml.etree.ElementTree as ET
from pathlib import Path
from robobarista import scene


def test_camera_count_and_unique_ids():
    assert len(scene.CAMERAS)==3
    assert len(scene.TAG_IDS)==len(set(scene.TAG_IDS.values()))


def test_local_scene_source_syntax():
    ast.parse(Path(scene.__file__).read_text())


def test_marker_tags_are_known_transforms():
    tags=scene.tag_definitions()
    assert len(tags)==5
    for label,T in tags.values():
        assert T.shape==(4,4)
        assert len(label)>0

def test_markers_can_be_attached_to_moving_robot_links():
    """MuJoCo rejects plane geoms inside nonstatic articulated bodies."""
    root = ET.Element('body', {'name': 'moving_robot_wrist'})
    tag = scene._tag(root, 'robot_tool', scene.TAG_IDS['robot_tool'], [0, 0, 0])
    assert tag.get('type') == 'box'
    assert tag.get('contype') == '0'
    assert tag.get('conaffinity') == '0'
    assert tag.get('mass') == '0'
    assert float(tag.get('size').split()[2]) > 0


def test_optional_pitcher_side_mounts_are_rigid_and_known():
    from scipy.spatial.transform import Rotation
    import numpy as np
    defs=scene.tag_definitions(pitcher_side_tags=True)
    ids=set(scene.SIDE_PITCHER_TAG_IDS.values())
    assert ids.issubset(defs)
    assert 3 not in defs  # The overhead pitcher marker is removed in side mode.
    assert len([1 for label, _ in defs.values() if label=='pitcher'])==2
    assert len(scene.CAMERAS_SIDE)==3
    for name,(offset,angle) in scene.SIDE_PITCHER_TAGS.items():
        tid=scene.SIDE_PITCHER_TAG_IDS[name]
        label,T=defs[tid]
        assert label=='pitcher'
        np.testing.assert_allclose(T[:3,3], offset)
        np.testing.assert_allclose(
            T[:3,:3], Rotation.from_euler('x',angle,degrees=True).as_matrix())
        assert np.linalg.det(T[:3,:3]) > 0.999
    assert scene.SIDE_PITCHER_TAGS['pitcher_near'][0][1] < 0
    assert scene.SIDE_PITCHER_TAGS['pitcher_far'][0][1] > 0
