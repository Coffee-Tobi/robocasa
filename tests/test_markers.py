import numpy as np
import cv2
from robobarista.geometry import detect_tags


def test_synthetic_aruco():
    d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker=cv2.aruco.generateImageMarker(d,7,220)
    canvas=np.full((480,640),255,np.uint8)
    canvas[125:345,205:425]=marker
    rgb=cv2.cvtColor(canvas,cv2.COLOR_GRAY2RGB)
    hits=detect_tags(rgb)
    assert len(hits)==1
    assert hits[0][0]==7
    assert np.linalg.norm(hits[0][1].mean(0)-np.array([314.5,234.5]))<2


def test_synthetic_apriltag_on_pitcher_side():
    d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker=cv2.aruco.generateImageMarker(d,23,220)
    canvas=np.full((480,640),255,np.uint8)
    canvas[125:345,205:425]=marker
    rgb=cv2.cvtColor(canvas,cv2.COLOR_GRAY2RGB)
    hits=detect_tags(rgb,include_apriltags=True)
    assert any(tag_id==23 for tag_id,_ in hits)
