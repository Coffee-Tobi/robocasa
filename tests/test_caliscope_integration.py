"""Caliscope integration smoke test using ideal projected 2D observations.

A pass only validates the camera-coordinate conversion and independent
triangulation / rigid alignment, NOT real-world or rendered precision.
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip("caliscope")

from robobarista.caliscope_integration import estimate_object_with_caliscope
from robobarista.geometry import Calibration, marker_corners, pose_errors


def test_caliscope_known_camera_rig_reconstructs_rigid_apriltag():
    K=np.array([[1200.,0.,640.],[0.,1200.,480.],[0.,0.,1.]])
    cams=[]
    for eye in ([0.,0.,0.],[.25,0.,0.],[.10,-.20,0.]):
        T=np.eye(4)
        T[:3,3]=eye
        cams.append(Calibration(1280,960,K,T))
    tag_size=0.052
    rotation=Rotation.from_euler("xyz",[9.,-14.,16.],degrees=True).as_matrix()
    position=np.array([0.07,-0.03,1.20])
    local=marker_corners(tag_size)
    world=(rotation @ local.T).T + position
    observations=[]
    for camera in cams:
        pixels=np.array([camera.project(pt) for pt in world])
        observations.append((23,dict(calibration=camera,corners=pixels)))
    definition={23:("pitcher",np.eye(4))}
    fit=estimate_object_with_caliscope(observations,definition,{23:tag_size})
    assert fit is not None
    mm,deg=pose_errors(fit,{'position':position,'rotation':rotation})
    assert mm<0.01,mm
    assert deg<0.01,deg
    assert fit['triangulated_corners']==4
    assert fit['cameras_seen']==3
