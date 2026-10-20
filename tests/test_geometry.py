import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from robobarista.geometry import (Calibration, marker_corners, solve_single,
                                 bundle_adjust_tag, object_from_tag, pose_errors,
                                 look_at_quat)


def calibration(position):
    T=np.eye(4)
    T[:3,3]=position
    T[:3,:3]=Rotation.from_euler('xyz',[0,.02,.03]).as_matrix()
    K=np.array([[880,0,480],[0,880,360],[0,0,1]],float)
    return Calibration(960,720,K,T)


def test_known_depth_roundtrip():
    c=calibration([.1,.05,0])
    p=np.array([.04,.10,1.3])
    uv=c.project(p)
    predicted=c.camera_to_world(np.linalg.inv(c.K) @ np.r_[uv,1] * c.world_to_camera(p)[2])
    np.testing.assert_allclose(predicted,p,atol=1e-10)


def test_single_pnp_with_depth():
    c=calibration([0,0,0])
    R=Rotation.from_euler('xy',[.1,-.08]).as_matrix()
    pos=np.array([.1,-.03,.85]); pts=(R@marker_corners(.056).T).T+pos
    px=np.array([c.project(v) for v in pts])
    depth=np.full((720,960),.85,np.float32)
    sol=solve_single(px,depth,c,.056)
    assert sol is not None
    err,ang=pose_errors(sol,{'position':pos,'rotation':R})
    assert err < 1.5
    assert ang < .2


def test_multicam_bundle_adjustment():
    pos=np.array([.065,-.03,1.15])
    R=Rotation.from_euler('xyz',[.11,-.08,.15]).as_matrix()
    cameras=[calibration(p) for p in [[-.25,0,0],[.22,.08,0],[.06,-.19,0]]]
    xyz=(R@marker_corners(.066).T).T+pos
    obs=[]
    for c in cameras:
        px=np.array([c.project(p) for p in xyz]); depth=np.full((720,960),c.world_to_camera(pos)[2],np.float32)
        o=solve_single(px,depth,c,.066)
        assert o is not None
        obs.append(o)
    est=bundle_adjust_tag(obs,.066)
    assert est['cameras_seen']==3
    mm,deg=pose_errors(est,{'position':pos,'rotation':R})
    assert mm < .2,mm
    assert deg < .15,deg
    T=np.eye(4);T[:3,3]=[0,0,.16]
    center=object_from_tag(est,T)
    np.testing.assert_allclose(center['position']+center['rotation']@[0,0,.16],est['position'],atol=1e-8)


def test_lookat_vertical():
    q=look_at_quat([0,0,2],[0,0,0]);R=Rotation.from_quat(np.r_[q[1:],q[0]]).as_matrix()
    np.testing.assert_allclose(R@[0,0,-1],[0,0,-1],atol=1e-10)
