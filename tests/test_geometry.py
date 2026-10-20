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


def test_two_sided_apriltag_rig_reconstructs_known_spout_from_multiview():
    """Ideal pinhole projections: mathematical multi-tag fit, NOT a rendered accuracy test."""
    from robobarista.scene import (
        SIDE_PITCHER_TAG_IDS, SIDE_PITCHER_TAGS, SIDE_TAG_SIDE_M,
        MARKER_TEXTURE_PX, MARKER_QUIET_PX, SPOUT_TIP_LOCAL_M,
        tag_definitions)
    from robobarista.geometry import bundle_adjust_object
    defs=tag_definitions(pitcher_side_tags=True)
    side=SIDE_TAG_SIDE_M*(MARKER_TEXTURE_PX-2*MARKER_QUIET_PX)/MARKER_TEXTURE_PX
    sides={tid:side for tid in SIDE_PITCHER_TAG_IDS.values()}
    pos=np.array([.55,-.12,.63])
    rot=Rotation.from_euler('xyz',[6,18,-9],degrees=True).as_matrix()
    eyes=[np.array(v) for v in [[.20,-.65,1.13],[.98,-.61,1.12],[.57,.43,1.14]]]
    observations=[]
    for eye in eyes:
        q=look_at_quat(eye,pos)
        qxyzw=np.r_[q[1:],q[0]]
        T=np.eye(4)
        T[:3,:3]=Rotation.from_quat(qxyzw).as_matrix() @ np.diag([1,-1,-1])
        T[:3,3]=eye
        K=np.array([[1300,0,640],[0,1300,480],[0,0,1]],float)
        cam=Calibration(1280,960,K,T)
        for tid in SIDE_PITCHER_TAG_IDS.values():
            Tobjtag=defs[tid][1]
            tag_pos=pos+rot@Tobjtag[:3,3]
            tag_rot=rot@Tobjtag[:3,:3]
            # Ignore the inward-facing plate in each view.
            normal=tag_rot@np.array([0.,0.,1.])
            if np.dot(normal,eye-tag_pos)<0.10:
                continue
            corners=(tag_rot@marker_corners(side).T).T+tag_pos
            pixels=np.array([cam.project(pt) for pt in corners])
            assert np.isfinite(pixels).all()
            observations.append((tid,dict(
                position=tag_pos,rotation=tag_rot,corners=pixels,
                calibration=cam,depth=None)))
    assert len(observations)>=3
    est=bundle_adjust_object(observations,defs,sides)
    assert est is not None
    mm,deg=pose_errors(est,{'position':pos,'rotation':rot})
    assert mm<0.01,mm
    assert deg<0.01,deg
    true_tip=pos+rot@SPOUT_TIP_LOCAL_M
    estimated_tip=est['position']+est['rotation']@SPOUT_TIP_LOCAL_M
    assert np.linalg.norm(true_tip-estimated_tip)*1000<0.01
