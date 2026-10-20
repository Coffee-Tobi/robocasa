"""RGB-D camera geometry and multi-view marker pose estimation.

All image inference uses RGB corners and optional rendered depth only. MuJoCo object
poses are handled exclusively by the benchmark as held-out ground truth.
"""
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    K: np.ndarray
    T_world_camera: np.ndarray  # camera-local OpenCV axes x right/y down/z forward

    @classmethod
    def from_mujoco(cls, model, data, name, width, height):
        camera_id = model.camera(name).id
        fovy = np.deg2rad(float(model.cam_fovy[camera_id]))
        f = height / (2 * np.tan(fovy / 2))
        K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], float)
        T = np.eye(4)
        T[:3, :3] = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
        T[:3, 3] = data.cam_xpos[camera_id]
        return cls(width, height, K, T)

    def world_to_camera(self, pt):
        return self.T_world_camera[:3, :3].T @ (np.asarray(pt) - self.T_world_camera[:3, 3])

    def camera_to_world(self, pt):
        return self.T_world_camera[:3, :3] @ np.asarray(pt) + self.T_world_camera[:3, 3]

    def project(self, point_world):
        p = self.world_to_camera(point_world)
        if p[2] <= 1e-4:
            return None
        return (self.K @ p)[:2] / p[2]

    def saveable(self):
        return dict(width=self.width, height=self.height, K=self.K.tolist(),
                    T_world_camera=self.T_world_camera.tolist())


def marker_corners(side):
    """TL,TR,BR,BL in a tag-local, right-handed frame (z points outward)."""
    s = side / 2
    return np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]], float)


def detect_tags(rgb):
    gray = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2GRAY)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    options = cv2.aruco.DetectorParameters()
    options.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.ArucoDetector(d, options).detectMarkers(gray)
    if ids is None:
        return []
    return [(int(i), np.asarray(c).reshape(4, 2).astype(float))
            for i,c in zip(ids.ravel(), corners)]


def median_depth_at_tag(depth, corners):
    if depth is None:
        return None
    c = np.asarray(corners, float).reshape(4, 2)
    inner = c.mean(0) + 0.42*(c-c.mean(0))
    mask = np.zeros(depth.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, np.round(inner).astype(np.int32), 1)
    vals = np.asarray(depth, float)[mask > 0]
    vals = vals[np.isfinite(vals) & (vals > 0.04) & (vals < 5)]
    return float(np.median(vals)) if vals.size >= 8 else None


def solve_single(corners, depth, calibration, side):
    pts = np.asarray(corners, float).reshape(4, 2)
    obj = marker_corners(side)
    valid, rvec, tvec = cv2.solvePnP(obj, pts, calibration.K, None,
                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not valid:
        return None
    R = cv2.Rodrigues(rvec)[0]
    t = tvec.ravel()
    z = median_depth_at_tag(depth, pts)
    # Keep subpixel-PnP translation; depth is a separate soft constraint.
    # Overwriting translation using the mean corner pixel makes oblique tags
    # systematically inaccurate because projected-corner centroid != center.
    return dict(position=calibration.camera_to_world(t),
                rotation=calibration.T_world_camera[:3, :3] @ R,
                depth=z, corners=pts, calibration=calibration)


def bundle_adjust_tag(observations, side):
    """Jointly optimize SE(3) from all cameras' 2D ArUco corners.

    Minimizes robust image reprojection residuals. The depth-image estimates act
    as additional soft constraints (centimeter scale), rather than replacing
    accurate subpixel RGB corners. This is NOT simple pose averaging.
    """
    observations = [o for o in observations if o is not None]
    if not observations:
        return None
    inits = np.array([o['position'] for o in observations], float)
    meanp = np.median(inits, axis=0)
    Rs = Rotation.from_matrix(np.array([o['rotation'] for o in observations]))
    meanR = Rs.mean().as_rotvec()
    base = np.r_[meanR, meanp]
    local = marker_corners(side)

    def residual(x):
        R = Rotation.from_rotvec(x[:3]).as_matrix()
        pts_world = (R @ local.T).T + x[3:]
        errors = []
        for o in observations:
            cal = o['calibration']
            cam = np.array([cal.world_to_camera(pt) for pt in pts_world])
            if np.any(cam[:, 2] < 0.02):
                return np.ones(8*len(observations) + len(observations))*1e4
            uv = (cal.K @ cam.T).T
            uv = uv[:, :2] / uv[:, 2:]
            errors.extend(((uv - o['corners']) / 1.0).reshape(-1))
            if o['depth'] is not None:
                marker_depth = cal.world_to_camera(x[3:])[2]
                errors.append((marker_depth - o['depth']) / 0.010)
            else:
                errors.append(0.0)
        return np.array(errors)

    sol = least_squares(residual, base, loss='huber', f_scale=2, max_nfev=180)
    R = Rotation.from_rotvec(sol.x[:3]).as_matrix()
    pixel_rmse = float(np.sqrt(np.mean(residual(sol.x).reshape(len(observations), 9)[:,:8]**2)))
    return dict(position=sol.x[3:], rotation=R, cameras_seen=len(observations),
                reprojection_rmse_px=pixel_rmse, converged=bool(sol.success),
                depth_views=sum(o['depth'] is not None for o in observations))


def object_from_tag(estimated_tag, T_object_tag):
    """Return body pose using known tag placement in the object frame.

    IMPORTANT: T_object_tag is CAD/mount calibration, not a live object truth.
    """
    T = np.eye(4)
    T[:3,:3] = estimated_tag['rotation']
    T[:3,3] = estimated_tag['position']
    T_obj = T @ np.linalg.inv(T_object_tag)
    return {'position': T_obj[:3,3], 'rotation': T_obj[:3,:3]}


def pose_errors(estimate, actual):
    dist = float(np.linalg.norm(estimate['position']-actual['position'])*1000)
    angle = float(np.rad2deg(Rotation.from_matrix(
        estimate['rotation'] @ actual['rotation'].T).magnitude()))
    return dist, angle


def look_at_quat(eye, target):
    """MuJoCo camera looks along local -Z. Handle near-vertical views."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = eye-target
    z /= np.linalg.norm(z)
    up = np.array([0.,0.,1.])
    if abs(np.dot(z, up)) > .96:
        up = np.array([0.,1.,0.])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    quat_xyzw = Rotation.from_matrix(np.column_stack([x,y,z])).as_quat()
    return np.r_[quat_xyzw[3], quat_xyzw[:3]]
