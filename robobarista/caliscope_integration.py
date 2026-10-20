"""Caliscope triangulation baseline for the same rigid AprilTag corner detections.

This uses Caliscope 0.11.8's real ImagePoints.triangulate implementation.
The camera intrinsics/extrinsics are provided by the ideal MuJoCo simulation;
it is deliberately NOT an extrinsic/intrinsic calibration benchmark.
The estimated 6D rigid transform from 3D reconstructed corners uses Kabsch.
No object ground truth enters the pipeline.
"""
from __future__ import annotations

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from .geometry import marker_corners


def estimate_object_with_caliscope(observations, tag_definitions, side_by_id):
    from caliscope.api import CameraArray, CameraData, ImagePoints

    obs = [(int(tid), o) for tid, o in observations if o is not None]
    if not obs:
        return None

    # Convert OpenCV conventions (camera-to-world) to Caliscope (world-to-camera).
    cameras_by_identity = {}
    for _, o in obs:
        cal = o['calibration']
        identity = id(cal)
        if identity in cameras_by_identity:
            continue
        camera_index = len(cameras_by_identity)
        R_cw = cal.T_world_camera[:3, :3]
        R_wc = R_cw.T
        p_world = cal.T_world_camera[:3, 3]
        cameras_by_identity[identity] = (camera_index, CameraData(
            cam_id=camera_index,
            size=(int(cal.width), int(cal.height)),
            matrix=np.asarray(cal.K, dtype=np.float64),
            distortions=np.zeros(5, dtype=np.float64),
            rotation=R_wc,
            translation=-R_wc @ p_world,
        ), cal)

    cameras = CameraArray({
        idx: camera for idx, camera, _ in cameras_by_identity.values()
    })
    rows = []
    local_points = {}
    for tid, o in obs:
        if tid not in tag_definitions:
            continue
        transform = np.asarray(tag_definitions[tid][1], dtype=float)
        marker_local = marker_corners(side_by_id[tid])
        local = (transform[:3, :3] @ marker_local.T).T + transform[:3, 3]
        cam_index = cameras_by_identity[id(o['calibration'])][0]
        for corner_index, pixel in enumerate(o['corners']):
            local_points[(tid,corner_index)] = local[corner_index]
            rows.append(dict(sync_index=0, cam_id=cam_index, object_id=tid,
                             keypoint_id=corner_index,
                             img_loc_x=float(pixel[0]), img_loc_y=float(pixel[1])))

    if not rows:
        return None
    projected = ImagePoints(pd.DataFrame(rows)).triangulate(cameras).df
    if len(projected)<3:
        return None

    in_object, in_world = [], []
    for item in projected.itertuples(index=False):
        key = (int(item.object_id), int(item.keypoint_id))
        if key not in local_points:
            continue
        point=np.array([item.x_coord,item.y_coord,item.z_coord], dtype=float)
        if not np.isfinite(point).all():
            continue
        in_object.append(local_points[key])
        in_world.append(point)

    if len(in_world)<3:
        return None
    obj=np.asarray(in_object)
    world=np.asarray(in_world)
    origin_obj=obj.mean(axis=0)
    origin_world=world.mean(axis=0)
    X=obj-origin_obj
    Y=world-origin_world
    if np.linalg.matrix_rank(X,tol=1e-8)<2:
        return None
    U, _, Vt = np.linalg.svd(X.T @ Y)
    correction=np.eye(3)
    correction[2,2]=np.linalg.det(Vt.T @ U.T)
    R=Vt.T @ correction @ U.T
    t=origin_world-R@origin_obj
    errors=[]
    for tid,o in obs:
        transform=np.asarray(tag_definitions[tid][1], dtype=float)
        corners=(transform[:3,:3]@marker_corners(side_by_id[tid]).T).T + transform[:3,3]
        predicted=(R@corners.T).T+t
        for pixel,point in zip(o['corners'],predicted):
            uv=o['calibration'].project(point)
            if uv is not None:
                errors.extend(np.asarray(uv)-np.asarray(pixel))
    return {
        'position':t,
        'rotation':R,
        'reprojection_rmse_px':float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
        'cameras_seen':len(cameras_by_identity),
        'tag_detections':len(obs),
        'tags_seen':sorted(set(tid for tid,_ in obs)),
        'depth_views':0,  # Caliscope triangulates RGB points, no MuJoCo depth used.
        'triangulated_corners':len(in_world),
        'converged':True,
        'pipeline':'caliscope_triangulation_plus_rigid_3d_fit',
    }
