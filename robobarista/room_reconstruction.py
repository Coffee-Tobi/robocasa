"""Independent RGB-D reconstruction of the *observed* RoboBarista workcell.

No MuJoCo truth enters the estimator: only 3 rendered RGB-D views, the
calibrated camera matrices, known AprilTag print size and pixel corners.
Known fixed-tag world poses, nominal tabletop and bounding rectangle are
opened only AFTER inference for comparison. This is NOT full room SLAM:
cameras have perfect *a priori* simulated extrinsics; unseen/occluded room
surfaces cannot be reconstructed.
"""
from __future__ import annotations
import argparse,json,os
from pathlib import Path
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .barista_cell import build_cell_scene,CAMERAS,TAG_ID,TABLE_TOP
from .geometry import Calibration,detect_tags,solve_single,bundle_adjust_tag
from .demo import render_pair,to_json
from .cell_demo import _set_free_pose

ACTIVE_SIDE=.105*440/512
STATIC={k:v for k,v in TAG_ID.items() if k not in (
    'plastic_cup','pitcher_top','pitcher_top_secondary')}


def depth_to_world(depth,calibration,stride=8):
    """Backproject genuine MuJoCo depth image to a calibrated XYZ cloud."""
    vv,uu=np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride]
    z=np.asarray(depth[::stride,::stride],float)
    ok=np.isfinite(z)&(z>.15)&(z<4)
    u,v,z=uu[ok],vv[ok],z[ok]
    x=(u-calibration.K[0,2])*z/calibration.K[0,0]
    y=(v-calibration.K[1,2])*z/calibration.K[1,1]
    cam=np.column_stack((x,y,z))
    return ((calibration.T_world_camera[:3,:3]@cam.T).T
            +calibration.T_world_camera[:3,3])


def run(args):
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    model=mujoco.MjModel.from_xml_path(str(build_cell_scene(
        Path(args.vendor_dir).resolve())))
    data=mujoco.MjData(model)
    _set_free_pose(model,data,'rb_pitcher_free',[-10,-10,-10],0)
    _set_free_pose(model,data,'rb_plastic_cup_free',[-10,-10,-10],0)
    mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,height=args.height,width=args.width)
    observations={name:[] for name in STATIC}
    cloud=[]
    camera_counts={}
    try:
        for name in CAMERAS:
            rgb,depth=render_pair(renderer,data,name)
            cal=Calibration.from_mujoco(
                model,data,name,args.width,args.height)
            cloud.append(depth_to_world(depth,cal,args.stride))
            detections=detect_tags(rgb,include_apriltags=True,detector='pupil')
            camera_counts[name]=len(detections)
            for tag,uv in detections:
                for station,tid in STATIC.items():
                    if tid==tag:
                        pose=solve_single(uv,depth,cal,ACTIVE_SIDE)
                        if pose is not None:
                            observations[station].append(pose)
                        break
    finally:
        renderer.close()
    source_camera_indices=np.concatenate(
        [np.full(len(points),i,dtype=np.uint8)
         for i,points in enumerate(cloud)])
    cloud=np.concatenate(cloud,axis=0)
    # The observed cloud is independent of the true body geometry. Full
    # room cannot be inferred outside the sensor frusta / behind occlusion.
    finite=np.isfinite(cloud).all(axis=1)
    cloud=cloud[finite]
    source_camera_indices=source_camera_indices[finite]
    rows=[]
    for station,obs in observations.items():
        if obs:
            pred=bundle_adjust_tag(obs,ACTIVE_SIDE)
            estimated=np.asarray(pred['position'])
            row=dict(station=station,
                     april_tag_id=STATIC[station],
                     position_estimated_m=estimated.tolist(),
                     camera_observations=len(obs),
                     reprojection_rmse_px=pred['reprojection_rmse_px'],
                     convergence=pred['converged'])
        else:
            row=dict(station=station,april_tag_id=STATIC[station],
                     position_estimated_m=None,camera_observations=0,
                     reprojection_rmse_px=None,convergence=False)
        rows.append(row)
    # Estimate tabletop Z only from visible RGB-D surface points: narrow
    # ROI is a *prior on approximate island location* (not the true height).
    roi=cloud[(np.abs(cloud[:,0])<.82)&(np.abs(cloud[:,1])<.70)
              &(cloud[:,2]>.76)&(cloud[:,2]<.87)]
    if len(roi)>50:
        hist,bins=np.histogram(roi[:,2],bins=np.linspace(.76,.87,112))
        mode=(bins[np.argmax(hist)]+bins[np.argmax(hist)+1])*.5
        near=roi[np.abs(roi[:,2]-mode)<.006]
        table_z=float(np.median(near[:,2])) if len(near)>25 else None
    else:table_z=None

    # Explicit ground truth is inspected only after ALL optical estimates.
    for row in rows:
        truth=data.xpos[model.body('rb_tag_mount_'+row['station']).id].copy()
        row['ground_truth_m']=truth.tolist()
        row['error_mm']=(float(np.linalg.norm(
            np.asarray(row['position_estimated_m'])-truth)*1000)
                         if row['position_estimated_m'] is not None else None)
    errors=[r['error_mm'] for r in rows if r['error_mm'] is not None]
    fig,ax=plt.subplots(figsize=(11,8))
    keep=cloud[(cloud[:,2]>.70)&(cloud[:,2]<1.8)]
    idx=np.linspace(0,max(0,len(keep)-1),
                    min(4500,len(keep)),dtype=int)
    if len(idx):ax.scatter(keep[idx,0],keep[idx,1],
                          s=1,alpha=.15,label='RGB-D sichtbar (XYZ)')
    for r in rows:
        truth=r['ground_truth_m']
        ax.scatter([truth[0]],[truth[1]],marker='o',s=55)
        if r['position_estimated_m'] is not None:
            p=r['position_estimated_m']
            ax.scatter([p[0]],[p[1]],marker='x',s=70,color='black')
            ax.annotate(r['station']+f" ({r['error_mm']:.1f} mm)",
                        (p[0],p[1]),fontsize=8)
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle((-.95,-.79),1.9,1.58,
                 fill=False,linestyle='--',lw=2,
                 label='Echter Inselumriss (nur Bewertung)'))
    ax.set_aspect('equal')
    ax.set(xlabel='X (m)',ylabel='Y (m)',
           title='Aus drei RGB-D-Bildern rekonstruierte Zellpunkte (x) / Simulatorreferenz (o)')
    ax.legend(loc='upper left')
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(out/'rgbd_room_vs_truth.png',dpi=140)
    plt.close(fig)
    # A real XYZ cloud visualisation, not a hand-authored drawing of the
    # workcell. It contains only camera-visible RGB-D surfaces.
    fig=plt.figure(figsize=(10,8))
    ax3=fig.add_subplot(111,projection='3d')
    active=cloud[(np.abs(cloud[:,0])<1.7)&(np.abs(cloud[:,1])<1.4)
                 &(cloud[:,2]>0)&(cloud[:,2]<2.2)]
    if len(active):
        active=active[np.linspace(0,len(active)-1,
                 min(6500,len(active)),dtype=int)]
        ax3.scatter(active[:,0],active[:,1],active[:,2],
                    c=active[:,2],cmap='viridis',s=.4,alpha=.38)
    for row in rows:
        if row['position_estimated_m'] is not None:
            xyz=row['position_estimated_m']
            ax3.scatter([xyz[0]],[xyz[1]],[xyz[2]],
                        marker='x',s=65,color='red')
    ax3.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',
            title='Gesehenes 3D-Volumen aus 3 RGB-D-Kameras + rekonstruierte AprilTags')
    ax3.view_init(elev=26,azim=-63)
    fig.tight_layout()
    fig.savefig(out/'rgbd_room_3d_cloud.png',dpi=135)
    plt.close(fig)
    (out/'reconstructed_xyz_cloud.npz').parent.mkdir(exist_ok=True,parents=True)
    np.savez_compressed(
        out/'reconstructed_xyz_cloud.npz',
        xyz=cloud.astype('float32'),
        camera_index=source_camera_indices,
        camera_names=np.asarray(list(CAMERAS)))
    report={
        'status':'PASS' if len(errors)==len(STATIC) else 'PARTIAL',
        'camera_count':len(CAMERAS),
        'camera_extrinsics_from_simulation_not_estimated':True,
        'input_images_per_camera':1,
        'rgbd_points':int(len(cloud)),
        'tag_count_expected':len(STATIC),
        'tag_count_observed':len(errors),
        'mean_fixed_tag_position_error_mm':float(np.mean(errors)) if errors else None,
        'max_fixed_tag_position_error_mm':max(errors,default=None),
        'estimated_tabletop_z_m':table_z,
        'true_tabletop_z_m':TABLE_TOP,
        'tabletop_error_mm':abs(table_z-TABLE_TOP)*1000 if table_z is not None else None,
        'fixed_station_details':rows,
        'room_coverage':'Only visible surfaces from 3 calibrated RGB-D cameras; incomplete behind machines/walls, NO full SLAM or real-camera calibration',
        'truth_usage':'ground_truth_m and TABLE_TOP only AFTER camera estimate to calculate errors',
    }
    (out/'summary.json').write_text(json.dumps(to_json(report),indent=2))
    print('ROOM FROM RGBD:',json.dumps(
        {k:v for k,v in report.items() if k!='fixed_station_details'},
        indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--vendor-dir',default='vendor')
    p.add_argument('--output',default='runs/room_reconstruction')
    p.add_argument('--width',type=int,default=960)
    p.add_argument('--height',type=int,default=720)
    p.add_argument('--stride',type=int,default=6)
    r=run(p.parse_args())
    import sys
    sys.stdout.flush();sys.stderr.flush()
    if os.environ.get('MUJOCO_GL','').lower()=='osmesa':
        os._exit(0 if r['status']=='PASS' else 2)
    if r['status']!='PASS':
        raise SystemExit(2)
if __name__=='__main__':
    main()
