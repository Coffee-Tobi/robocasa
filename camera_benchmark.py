"""RoboBarista Vision: a measurable three-RGB-D-camera MuJoCo FR5 test bench.

Run in Colab via COLAB_START.ipynb. Sensor estimations must NOT read object truth.
"""
import argparse
import csv
import json
import os
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

import cv2
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

FR5_REPO = "https://github.com/ma-weiss/Fairino-Mujoco.git"
FR5_SHA = "459570714fc68c92e1fb3605cb60b4d8ef84419b"
OBJECTS = {
    "pitcher": (3, [0.67, -0.14, 0.452], [0.0, 0.0, 0.155]),
    "cup": (7, [0.53, 0.15, 0.452], [0.072, 0.0, 0.112]),
    "canister": (11, [0.35, -0.20, 0.502], [0.0, 0.0, 0.053]),
    "obstacle": (15, [0.38, 0.20, 0.517], [0.0, 0.0, 0.068]),
}
TOOL_ID = 19
CAMERAS = {
    "left": ([0.16, -0.86, 1.20], [0.56, 0.00, 0.67]),
    "right": ([1.10, -0.52, 1.27], [0.54, 0.04, 0.66]),
    "overhead": ([0.58, 0.36, 1.60], [0.55, 0.03, 0.63]),
}
SIDE_OUTER = 0.066
QUIET_PX = 42
TEX_PX = 512
SIDE_ACTIVE = SIDE_OUTER * (TEX_PX - 2 * QUIET_PX) / TEX_PX


def nums(x):
    return " ".join(str(float(a)) for a in x)


def look_at(eye, target):
    z = np.asarray(eye, float) - np.asarray(target, float)
    z /= np.linalg.norm(z)
    x = np.cross([0.0, 0.0, 1.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    q = Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()
    return nums([q[3], q[0], q[1], q[2]])


def acquire_fr5(root):
    directory = root / "Fairino-Mujoco"
    if not (directory / "fr5_robot.xml").is_file():
        root.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "clone", FR5_REPO, str(directory)], check=True)
    sha = subprocess.check_output(["git", "-C", str(directory), "rev-parse", "HEAD"], text=True).strip()
    if sha != FR5_SHA:
        subprocess.run(["git", "-C", str(directory), "checkout", FR5_SHA], check=True)
    return directory


def tag_face(parent, name, tid, offset):
    ET.SubElement(parent, "geom", {
        "name": "rb_tag_" + name, "type": "box", "pos": nums(offset),
        "size": nums([SIDE_OUTER / 2, SIDE_OUTER / 2, 0.0005]),
        "material": "rb_mat_" + str(tid), "contype": "0", "conaffinity": "0",
    })


def setup_scene(root):
    vendor = acquire_fr5(root)
    texdir = vendor / "rb_tags"
    texdir.mkdir(exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for tid in [v[0] for v in OBJECTS.values()] + [TOOL_ID]:
        pattern = cv2.aruco.generateImageMarker(dictionary, tid, TEX_PX - 2 * QUIET_PX)
        arr = np.full((TEX_PX, TEX_PX), 255, np.uint8)
        arr[QUIET_PX:-QUIET_PX, QUIET_PX:-QUIET_PX] = pattern
        Image.fromarray(arr).convert("RGB").save(texdir / ("tag_" + str(tid) + ".png"))

    robot = ET.parse(vendor / "fr5_robot.xml")
    rr = robot.getroot()
    asset = rr.find("asset")
    wrist = rr.find(".//body[@name='wrist3_link']")
    if asset is None or wrist is None:
        raise RuntimeError("FR5 source model changed; cannot attach marker")
    marker_body = ET.SubElement(wrist, "body", {"name": "rb_robot_tool", "pos": "0 0.085 0.07"})
    tag_face(marker_body, "robot_tool", TOOL_ID, [0, 0, 0])
    for tid in [v[0] for v in OBJECTS.values()] + [TOOL_ID]:
        ET.SubElement(asset, "texture", {
            "type": "2d", "name": "rb_tex_" + str(tid),
            "file": "rb_tags/tag_" + str(tid) + ".png",
        })
        ET.SubElement(asset, "material", {
            "name": "rb_mat_" + str(tid), "texture": "rb_tex_" + str(tid),
            "texrepeat": "1 1", "texuniform": "false", "specular": "0",
        })
    robot.write(vendor / "rb_fr5_robot.xml", encoding="unicode")

    rootxml = ET.Element("mujoco", {"model": "RoboBarista RGBD"})
    ET.SubElement(rootxml, "include", {"file": "rb_fr5_robot.xml"})
    visual = ET.SubElement(rootxml, "visual")
    ET.SubElement(visual, "global", {"offwidth": "1280", "offheight": "960"})
    world = ET.SubElement(rootxml, "worldbody")
    ET.SubElement(world, "light", {"pos": "0.5 0 1.75", "dir": "0 0 -1", "directional": "true"})
    ET.SubElement(world, "geom", {"name": "rb_floor", "type": "plane", "size": "2 2 0.1", "rgba": ".75 .76 .77 1"})
    ET.SubElement(world, "geom", {
        "name": "rb_counter", "type": "box", "pos": ".57 0 .415",
        "size": ".43 .37 .035", "rgba": ".30 .33 .36 1",
    })
    for name, (tid, pos, offset) in OBJECTS.items():
        b = ET.SubElement(world, "body", {"name": "rb_" + name, "pos": nums(pos)})
        if name == "pitcher":
            ET.SubElement(b, "geom", {"type": "cylinder", "pos": "0 0 .073", "size": ".05 .073", "rgba": ".65 .72 .77 1"})
            ET.SubElement(b, "geom", {"type": "box", "pos": "-.051 0 .136", "size": ".018 .016 .005", "rgba": ".7 .77 .8 1"})
            ET.SubElement(b, "geom", {"type": "capsule", "fromto": ".049 0 .112 .11 0 .112", "size": ".012", "rgba": ".65 .72 .77 1"})
        elif name == "cup":
            ET.SubElement(b, "geom", {"type": "cylinder", "pos": "0 0 .052", "size": ".047 .052", "rgba": ".93 .91 .85 1"})
            ET.SubElement(b, "geom", {"type": "cylinder", "pos": "0 0 .101", "size": ".042 .001", "rgba": ".25 .12 .07 1"})
        else:
            ET.SubElement(b, "geom", {"type": "box", "size": ".050 .043 .05" if name == "canister" else ".040 .050 .065", "rgba": ".5 .65 .75 1"})
        tag_face(b, name, tid, offset)
    for name, (eye, target) in CAMERAS.items():
        ET.SubElement(world, "camera", {"name": name, "pos": nums(eye), "quat": look_at(eye, target), "fovy": "46"})
    path = vendor / "rb_scene.xml"
    ET.ElementTree(rootxml).write(path, encoding="unicode")
    return path


def marker_points(side):
    h = side / 2
    return np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]], np.float32)


def detect(rgb):
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    pts, ids, _ = cv2.aruco.ArucoDetector(dic).detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
    if ids is None:
        return []
    return [(int(tid), corners[0].astype(float)) for tid, corners in zip(ids.ravel(), pts)]


def calibrate(model, data, name, width, height):
    cam = model.camera(name).id
    fy = height / (2 * np.tan(np.deg2rad(float(model.cam_fovy[cam])) / 2))
    K = np.array([[fy, 0, width/2], [0, fy, height/2], [0, 0, 1]], float)
    R = data.cam_xmat[cam].reshape(3,3) @ np.diag([1, -1, -1])
    p = data.cam_xpos[cam].copy()
    return K,R,p


def tag_depth(depth, corners):
    polygon = np.asarray(corners, float)
    center = polygon.mean(axis=0)
    inner = center + 0.45 * (polygon-center)
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.rint(inner).astype(np.int32), 1)
    vals = depth[mask>0]
    vals = vals[np.isfinite(vals) & (vals>0.03) & (vals<8.0)]
    return float(np.median(vals)) if len(vals) >= 8 else None


def estimate(corners, depth, calibration):
    K, R_w_c, p_w_c = calibration
    ok, rvec, tvec = cv2.solvePnP(marker_points(SIDE_ACTIVE), np.asarray(corners,float),
                                 K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R_c_m = cv2.Rodrigues(rvec)[0]
    pixel = np.asarray(corners).mean(axis=0)
    d = tag_depth(depth, corners)
    if d is None:
        p_c_m = tvec.ravel()
    else:
        p_c_m = np.linalg.inv(K) @ np.array([pixel[0],pixel[1],1.0]) * d
    return np.array(p_w_c + R_w_c @ p_c_m), np.array(R_w_c @ R_c_m), d


def matrix_angle_deg(A, B):
    return float(np.rad2deg(Rotation.from_matrix(A @ B.T).magnitude()))


def render_rgbd(renderer, data, camera):
    renderer.disable_depth_rendering()
    renderer.update_scene(data, camera=camera)
    rgb = renderer.render().copy()
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=camera)
    depth = renderer.render().copy()
    renderer.disable_depth_rendering()
    return rgb, depth


def mosaic(images, file):
    W, H = 520, 390
    result = Image.new("RGB", (3*W, 2*H+36), "white")
    for j,(name,rgb,depth) in enumerate(images):
        result.paste(Image.fromarray(rgb).resize((W,H)), (j*W, 36))
        stretch = np.clip((depth-.25)/1.35, 0, 1)
        col = cv2.cvtColor(cv2.applyColorMap((255*(1-stretch)).astype(np.uint8), cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
        result.paste(Image.fromarray(col).resize((W,H)), (j*W,H+36))
        ImageDraw.Draw(result).text((j*W+10, 10),name+" RGB / depth", fill="black")
    result.save(file, quality=92)


def self_test():
    K = np.array([[500.,0,320],[0,500,240],[0,0,1]])
    R = Rotation.from_euler("xyz",[10,8,30],degrees=True).as_matrix()
    t = np.array([.01,.02,.7])
    p = (R @ marker_points(SIDE_ACTIVE).T).T + t
    corners = (K @ p.T).T
    corners = corners[:,:2] / corners[:,2,None]
    ok,rv,tv = cv2.solvePnP(marker_points(SIDE_ACTIVE),corners,K,None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    assert ok and np.linalg.norm(tv.ravel()-t) < 1e-5
    assert matrix_angle_deg(cv2.Rodrigues(rv)[0],R) < .05
    print("Synthetic PnP projection test: PASS")
    return 0


def run(args):
    # MUJOCO_GL must be configured before mujoco import.
    import mujoco
    root=Path(args.vendor_dir).resolve()
    path=setup_scene(root)
    model=mujoco.MjModel.from_xml_path(str(path))
    data=mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model,data,0)
    mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,height=args.height,width=args.width)
    calib={n:calibrate(model,data,n,args.width,args.height) for n in CAMERAS}
    rng=np.random.default_rng(args.seed)
    original={n:model.body("rb_"+n).pos.copy() for n in OBJECTS}
    rows=[]
    first=[]
    out=Path(args.output)
    out.mkdir(parents=True,exist_ok=True)
    try:
        for trial in range(args.trials):
            for name,base in original.items():
                model.body("rb_"+name).pos[:]=base+(np.zeros(3) if trial==0 else np.array([rng.uniform(-.02,.02),rng.uniform(-.015,.015),0]))
            mujoco.mj_forward(model,data)
            obs={tid:[] for tid,*_ in OBJECTS.values()}
            obs[TOOL_ID]=[]
            for camera in CAMERAS:
                rgb,depth=render_rgbd(renderer,data,camera)
                marked=rgb.copy()
                for tid,corners in detect(rgb):
                    if tid not in obs: continue
                    cv2.polylines(marked,[np.rint(corners).astype(np.int32)],True,(0,255,0),3)
                    cv2.putText(marked,str(tid),tuple(np.rint(corners[0]).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,40,255),2)
                    pose=estimate(corners,depth,calib[camera])
                    if pose is not None: obs[tid].append(pose)
                if trial==0:
                    first.append((camera,marked,depth))
                    Image.fromarray(marked).save(out/(camera+"_annotated.png"))
                    np.save(out/(camera+"_depth_m.npy"),depth)
                    K,R,t=calib[camera]
                    (out/(camera+"_calibration.json")).write_text(json.dumps({"K":K.tolist(),"R_world_camera":R.tolist(),"camera_xyz_m":t.tolist()},indent=2))
            for name,(tid,_,offset) in list(OBJECTS.items())+[("robot_tool",(TOOL_ID,[0,0,0],[0,0,0]))]:
                bodyname="rb_robot_tool" if name=="robot_tool" else "rb_"+name
                bodyid=model.body(bodyname).id
                gt=data.xpos[bodyid].copy()
                gtR=data.xmat[bodyid].reshape(3,3).copy()
                views=obs[tid]
                row={"trial":trial,"object":name,"id":tid,"views":len(views),"detected":bool(views)}
                if views:
                    # RGB-D/marker inference only; ground truth is accessed AFTER inference.
                    rotations=np.array([o[1] for o in views])
                    Rhat=Rotation.from_matrix(rotations).mean().as_matrix()
                    xyz=np.median([o[0] for o in views],axis=0)-Rhat@np.array(offset)
                    row.update({
                        "camera_position_m":xyz.tolist(),
                        "sim_position_m":gt.tolist(),
                        "position_error_mm":round(1000*float(np.linalg.norm(xyz-gt)),3),
                        "angle_error_deg":round(matrix_angle_deg(Rhat,gtR),3),
                    })
                else:
                    row.update({"position_error_mm":None,"angle_error_deg":None})
                rows.append(row)
    finally:
        renderer.close()
    if first:
        mosaic(first,out/"rgbd_montage.jpg")
    with (out/"pose_errors.csv").open("w",newline="") as stream:
        fields=["trial","object","id","views","detected","position_error_mm","angle_error_deg"]
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    detections=[r for r in rows if r["detected"]]
    passed=(len(detections)==len(rows) and all(
        r["views"]>=args.min_cameras and r["position_error_mm"]<=args.max_mm
        and r["angle_error_deg"]<=args.max_deg for r in detections))
    summary={
        "trials":args.trials, "expected_tags":len(rows),
        "detected_tags":len(detections),
        "max_position_mm":max([r["position_error_mm"] for r in detections],default=None),
        "max_angle_deg":max([r["angle_error_deg"] for r in detections],default=None),
        "target_translation_mm":args.max_mm,"target_rotation_deg":args.max_deg,
        "min_views":args.min_cameras,"passed_strict_accuracy_gate":bool(passed),
        "fr5_model":FR5_REPO,"fr5_commit":FR5_SHA,
        "warning":"Virtual known camera calibration + ideal MuJoCo depth, not real sensor accuracy",
    }
    (out/"measurements.json").write_text(json.dumps(rows,indent=2))
    (out/"summary.json").write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))
    print("Outputs:",out)
    return 0 if passed else 2


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--output",default="runs/first_benchmark")
    p.add_argument("--width",type=int,default=960)
    p.add_argument("--height",type=int,default=720)
    p.add_argument("--trials",type=int,default=3)
    p.add_argument("--seed",type=int,default=42)
    p.add_argument("--min-cameras",type=int,default=2)
    p.add_argument("--max-mm",type=float,default=5.0)
    p.add_argument("--max-deg",type=float,default=5.0)
    p.add_argument("--self-test",action="store_true")
    args=p.parse_args()
    raise SystemExit(self_test() if args.self_test else run(args))


if __name__=="__main__":
    main()
