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


def test_pupil_corner_order_without_pupil_dependency():
    from types import SimpleNamespace
    from robobarista.geometry import _pupil_corner_order
    # Known 2D homography: local tag corners [-1,1], ... to image.
    H=np.array([[45.,0.,300.],[0.,-45.,200.],[0.,0.,1.]])
    ideals=np.array([[-1,1],[1,1],[1,-1],[-1,-1]],float)
    expected=np.c_[ideals,np.ones(4)]@H.T
    expected=expected[:,:2]/expected[:,2:]
    # The pupil library is allowed to return corners in another order.
    detection=SimpleNamespace(homography=H,corners=expected[[2,0,3,1]])
    # Homography-aligned native order is distinct from OpenCV's 4 corners.
    np.testing.assert_allclose(
        _pupil_corner_order(detection),expected[[1,0,3,2]])


def test_opencv_apriltag_detector_preserves_marker_corner_order():
    d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker=cv2.aruco.generateImageMarker(d,23,220)
    canvas=np.full((480,640),255,np.uint8)
    canvas[125:345,205:425]=marker
    rgb=cv2.cvtColor(canvas,cv2.COLOR_GRAY2RGB)
    hits=detect_tags(rgb,include_apriltags=True,detector='opencv')
    positions=[pts for tid,pts in hits if tid==23]
    assert positions
    assert positions[0].shape==(4,2)


def test_native_apriltag3_can_read_rendered_tag_with_correct_corner_order():
    import pytest
    pytest.importorskip('pupil_apriltags')
    d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker=cv2.aruco.generateImageMarker(d,23,320)
    canvas=np.full((600,800),255,np.uint8)
    canvas[140:460,240:560]=marker
    rgb=cv2.cvtColor(canvas,cv2.COLOR_GRAY2RGB)
    cv_hits={id_:pts for id_,pts in detect_tags(rgb,True,detector='opencv')}
    native_hits={id_:pts for id_,pts in detect_tags(rgb,True,detector='pupil')}
    assert 23 in cv_hits
    assert 23 in native_hits, 'Native AprilTag 3 did not detect the same tag'
    assert np.max(np.linalg.norm(cv_hits[23]-native_hits[23],axis=1))<4.0

    # Repeat with the printed marker rotated. This catches an orientation
    # mistake that could otherwise corrupt 6D pose estimates.
    for rotated in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180,
                    cv2.ROTATE_90_COUNTERCLOCKWISE):
        rotated_canvas=cv2.rotate(canvas,rotated)
        rotated_rgb=cv2.cvtColor(rotated_canvas,cv2.COLOR_GRAY2RGB)
        cv_hits={i:p for i,p in detect_tags(rotated_rgb,True,detector='opencv')}
        native_hits={i:p for i,p in detect_tags(rotated_rgb,True,detector='pupil')}
        assert 23 in cv_hits and 23 in native_hits
        assert np.max(np.linalg.norm(cv_hits[23]-native_hits[23],axis=1))<4.0
