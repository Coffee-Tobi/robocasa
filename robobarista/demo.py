"""Three simulated RGB-D camera views and strict GT-vs-vision benchmark."""
from __future__ import annotations
import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path
import numpy as np
import cv2
from PIL import Image, ImageDraw
from .scene import (generate_scene, CAMERAS, CAMERAS_SIDE, TAG_IDS,
                    SIDE_PITCHER_TAG_IDS, SIDE_TAG_SIDE_M, MARKER_ACTIVE_M,
                    MARKER_TEXTURE_PX, MARKER_QUIET_PX,
                    SPOUT_TIP_LOCAL_M, tag_definitions)
from .geometry import (Calibration, detect_tags, solve_single, bundle_adjust_tag,
                       bundle_adjust_object, object_from_tag, pose_errors)


def to_json(x):
    if isinstance(x,dict): return {k:to_json(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)): return [to_json(v) for v in x]
    if isinstance(x,np.ndarray): return x.tolist()
    if isinstance(x,(np.integer,np.floating)): return x.item()
    return x


def render_pair(renderer, data, name):
    renderer.disable_depth_rendering()
    renderer.update_scene(data, camera=name)
    rgb=renderer.render().copy()
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=name)
    depth=renderer.render().copy()
    renderer.disable_depth_rendering()
    if depth.ndim != 2 or rgb.ndim!=3:
        raise RuntimeError(f'Unexpected MuJoCo RGB-D shapes: {rgb.shape}, {depth.shape}')
    return rgb,depth


def get_gt(model,data,object_name):
    name=('rb_robot_tool_tag' if object_name=='robot_tool' else 'rb_'+object_name)
    body_id=model.body(name).id
    return {'position':data.xpos[body_id].copy(),
            'rotation':data.xmat[body_id].reshape(3,3).copy()}


def save_montage(rgbs,depths,path):
    w,h=540,400
    canvas=Image.new('RGB',(w*3,h*2+40),'#f3f4f6')
    d=ImageDraw.Draw(canvas)
    for n,(name,img) in enumerate(rgbs.items()):
        rgb=Image.fromarray(img).resize((w,h),Image.Resampling.LANCZOS)
        depth=depths[name]
        normalized=(1-np.clip((depth-0.25)/1.65,0,1))
        cmap=cv2.applyColorMap((255*normalized).astype('uint8'),cv2.COLORMAP_TURBO)
        z=Image.fromarray(cv2.cvtColor(cmap,cv2.COLOR_BGR2RGB)).resize((w,h),Image.Resampling.LANCZOS)
        canvas.paste(rgb,(n*w,30))
        canvas.paste(z,(n*w,h+35))
        d.text((n*w+9,8),name+' | RGB + depth (meters)',fill='black')
    canvas.save(path,quality=88)


def save_plot(rows,cameras,folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(11,8))
    ax=fig.add_subplot(projection='3d')
    plotted=set()
    for r in rows:
        if r['trial']!=0 or r['failed']: continue
        name=r['object']
        if name in plotted: continue
        plotted.add(name)
        actual=np.array(r['truth_position_m'])
        pred=np.array(r['estimate_position_m'])
        ax.scatter(*actual,marker='o',s=75,label=f'{name}: true')
        ax.scatter(*pred,marker='x',s=65,label=f'{name}: camera')
        ax.plot([actual[0],pred[0]],[actual[1],pred[1]],
                [actual[2],pred[2]],'k--',alpha=.25)
    for name,c in cameras.items():
        p=c.T_world_camera[:3,3]
        ax.scatter(*p,marker='^',color='gray',s=60)
        ax.text(*p,name,fontsize=7)
    ax.set_xlabel('X m');ax.set_ylabel('Y m');ax.set_zlabel('Z m')
    ax.set_title('Vision (x) vs. MuJoCo independent ground truth (o)')
    ax.legend(fontsize=8,loc='best')
    fig.tight_layout()
    fig.savefig(folder/'pose_comparison.png',dpi=150)
    plt.close(fig)


def run(args):
    import mujoco
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    scene_path=generate_scene(Path(args.vendor_dir).resolve(), pitcher_side_tags=args.pitcher_side_tags)
    model=mujoco.MjModel.from_xml_path(str(scene_path))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,height=args.height,width=args.width)
    cameras = CAMERAS_SIDE if args.pitcher_side_tags else CAMERAS
    calibrations={name:Calibration.from_mujoco(model,data,name,args.width,args.height)
                  for name in cameras}
    defs=tag_definitions(pitcher_side_tags=args.pitcher_side_tags)
    side_active = SIDE_TAG_SIDE_M * (MARKER_TEXTURE_PX - 2 * MARKER_QUIET_PX) / MARKER_TEXTURE_PX
    side_tag_ids = set(SIDE_PITCHER_TAG_IDS.values())
    tag_sides = {tid: (side_active if tid in side_tag_ids and args.pitcher_side_tags
                       else MARKER_ACTIVE_M) for tid in defs}
    rng=np.random.default_rng(args.seed)
    positions={n:model.body('rb_'+n).pos.copy()
               for n in ('cup','pitcher','canister','obstacle')}
    reports=[];frames={};depth_frames={}
    initial_pitcher_quat = model.body('rb_pitcher').quat.copy()
    start=time.monotonic()
    for trial in range(args.trials):
        for name,initial in positions.items():
            movement=np.zeros(3) if trial==0 else [rng.uniform(-0.024,.024),rng.uniform(-0.018,.018),0]
            model.body('rb_'+name).pos[:]=initial+movement
        if args.pitcher_side_tags:
            # Stationary pose snapshots at different pitcher tilts (not a timed pour).
            from scipy.spatial.transform import Rotation
            angle_deg = [0., 12., 25.][trial % 3]
            tilt = Rotation.from_euler('y', angle_deg, degrees=True)
            qbase = Rotation.from_quat(np.r_[initial_pitcher_quat[1:], initial_pitcher_quat[0]])
            xyzw = (tilt * qbase).as_quat()
            model.body('rb_pitcher').quat[:] = np.r_[xyzw[3], xyzw[:3]]
        mujoco.mj_forward(model,data)
        groups={tid:[] for tid in defs}
        for name,calib in calibrations.items():
            rgb,depth=render_pair(renderer,data,name)
            annotated=rgb.copy()
            detections=detect_tags(rgb)
            for tag_id,pixels in detections:
                if tag_id not in defs: continue
                cv2.polylines(annotated,[np.round(pixels).astype('int32')],True,(0,255,0),2)
                u,v=pixels.mean(0).astype(int)
                cv2.putText(annotated,f'#{tag_id}',(u,v-8),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2)
                est=solve_single(pixels,depth,calib,tag_sides[tag_id])
                if est is not None: groups[tag_id].append(est)
            if trial==0:
                frames[name]=annotated.copy()
                depth_frames[name]=depth.copy()
                Image.fromarray(annotated).save(out/f'{name}_annotated.png')
                np.save(out/f'{name}_depth_m.npy',depth)
                (out/f'{name}_calibration.json').write_text(
                    json.dumps(calib.saveable(),indent=2))
        if args.pitcher_side_tags:
            # Multiple tag IDs belong to one pitcher. Fuse all their image corners
            # into a SINGLE pitcher pose, never count them as separate objects.
            for label in ('pitcher', 'cup', 'canister', 'obstacle'):
                all_observations=[(tid, obs) for tid,(tag_label, _) in defs.items()
                                  if tag_label == label for obs in groups[tid]]
                gt=get_gt(model,data,label)
                est=bundle_adjust_object(all_observations,defs,tag_sides) if all_observations else None
                views=len(set(id(obs['calibration']) for _,obs in all_observations))
                row={'trial':trial,'object':label,
                     'tag_ids':sorted(set(tid for tid,_ in all_observations)),
                     'cameras_seen':views, 'failed':est is None,
                     'truth_position_m':gt['position'], 'truth_rotation':gt['rotation']}
                if est is not None:
                    t_err,r_err=pose_errors(est,gt)
                    row.update(estimate_position_m=est['position'],
                               estimate_rotation=est['rotation'],
                               translation_mm=t_err,rotation_deg=r_err,
                               reprojection_rmse_px=est['reprojection_rmse_px'],
                               depth_views=est['depth_views'],converged=est['converged'],
                               tag_detections=est['tag_detections'])
                    if label == 'pitcher':
                        # Spout offset comes from our simplified CAD geometry, not GT feedback.
                        tip_true=gt['position'] + gt['rotation'] @ SPOUT_TIP_LOCAL_M
                        tip_est=est['position'] + est['rotation'] @ SPOUT_TIP_LOCAL_M
                        row['spout_tip_error_mm']=float(np.linalg.norm(tip_est-tip_true)*1000)
                reports.append(row)
        else:
            for tid,(label,body_tag_transform) in defs.items():
                views=groups[tid]
                gt=get_gt(model,data,label)
                est=bundle_adjust_tag(views,MARKER_ACTIVE_M) if views else None
                row={'trial':trial,'object':label,'tag_id':tid,
                     'cameras_seen':len(views),'failed':not bool(est),
                     'truth_position_m':gt['position'],'truth_rotation':gt['rotation']}
                if est:
                    body=object_from_tag(est,body_tag_transform)
                    t_err,r_err=pose_errors(body,gt)
                    row.update(estimate_position_m=body['position'],estimate_rotation=body['rotation'],
                               translation_mm=t_err,rotation_deg=r_err,
                               reprojection_rmse_px=est['reprojection_rmse_px'],
                               depth_views=est['depth_views'],converged=est['converged'])
                reports.append(row)
    renderer.close()
    save_montage(frames,depth_frames,out/'rgbd_montage.jpg')
    save_plot(reports,calibrations,out)
    serializable=to_json(reports)
    (out/'measurements.json').write_text(json.dumps(serializable,indent=2))
    with (out/'pose_errors.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['trial','object','translation_mm','rotation_deg','spout_tip_error_mm','cameras_seen','reprojection_px','failed'])
        for r in reports:
            writer.writerow([r['trial'],r['object'],r.get('translation_mm'),r.get('rotation_deg'),
                             r.get('spout_tip_error_mm'),r['cameras_seen'],
                             r.get('reprojection_rmse_px'),r['failed']])
    complete=[r for r in reports if not r['failed']]
    missing=[{'trial':r['trial'],'object':r['object'],'cameras':r['cameras_seen']}
             for r in reports if r['failed'] or r['cameras_seen']<args.min_cameras]
    T=[r['translation_mm'] for r in complete]
    R=[r['rotation_deg'] for r in complete]
    quality_ok=(not missing and len(complete)==len(reports)
                and all(r.get('converged',False) for r in complete))
    if args.pitcher_side_tags:
        # Original 5mm/5deg limits remain on OTHER static objects.
        # New stricter acceptance is based on the PITCHER SPOUT.
        pitcher_rows=[r for r in reports if r['object']=='pitcher']
        other_rows=[r for r in reports if r['object']!='pitcher']
        passed=(quality_ok
                and all(r.get('spout_tip_error_mm',float('inf'))<=args.max_spout_mm
                        and r.get('rotation_deg',float('inf'))<=args.max_pitcher_rotation_deg
                        for r in pitcher_rows)
                and all(r.get('translation_mm',float('inf'))<=args.max_translation_mm
                        and r.get('rotation_deg',float('inf'))<=args.max_rotation_deg
                        for r in other_rows))
    else:
        passed=(quality_ok
                and max(T,default=float('inf'))<=args.max_translation_mm
                and max(R,default=float('inf'))<=args.max_rotation_deg)
    tip_errors=[r['spout_tip_error_mm'] for r in reports if 'spout_tip_error_mm' in r]
    summary={'status':'PASS' if passed else 'FAIL',
        'experiment':'pitcher_side_tags' if args.pitcher_side_tags else 'original_top_tags',
        'spout_geometry':'simplified known CAD offset; real fixture/calibration not validated',
        'max_spout_tip_error_mm':max(tip_errors) if tip_errors else None,
        'allowed_spout_tip_error_mm':args.max_spout_mm if args.pitcher_side_tags else None,
        'allowed_pitcher_rotation_deg':args.max_pitcher_rotation_deg if args.pitcher_side_tags else None,
        'important':'Full 3D renderer benchmark, not equivalent to actual sensor performance',
        'vendor':'ma-weiss/Fairino-Mujoco@459570714fc68c92e1fb3605cb60b4d8ef84419b',
        'trials':args.trials,'total_object_trials':len(reports),'measured':len(complete),
        'missing_or_insufficient_views':missing,
        'max_translation_mm':max(T) if T else None,
        'max_rotation_deg':max(R) if R else None,
        'mean_translation_mm':float(np.mean(T)) if T else None,
        'mean_rotation_deg':float(np.mean(R)) if R else None,
        'allowed_translation_mm':args.max_translation_mm,
        'allowed_rotation_deg':args.max_rotation_deg,
        'required_cameras_per_object':args.min_cameras,
        'duration_s':time.monotonic()-start}
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    print('\nObject | mm | degrees | visible cameras')
    for r in reports:
        print(r['object'],r.get('translation_mm','not detected'),r.get('rotation_deg','-'),r['cameras_seen'], 'spout_mm:',r.get('spout_tip_error_mm','-'))
    print('\nResult:',summary['status'],'| data in',out)
    if not passed:
        print('Accuracy gate NOT passed. Check image visibility, marker orientation/UV and calibration.',file=sys.stderr)
    return 0 if passed else 2


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--pitcher-side-tags',action='store_true',
                   help='Opt-in two-sided pitcher marker and camera experiment')
    p.add_argument('--max-spout-mm',type=float,default=0.5)
    p.add_argument('--max-pitcher-rotation-deg',type=float,default=1.0)
    p.add_argument('--vendor-dir',default='vendor')
    p.add_argument('--output',default='runs/first_benchmark')
    p.add_argument('--width',type=int,default=960)
    p.add_argument('--height',type=int,default=720)
    p.add_argument('--trials',type=int,default=3)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--min-cameras',type=int,default=2)
    p.add_argument('--max-translation-mm',type=float,default=5)
    p.add_argument('--max-rotation-deg',type=float,default=5)
    args=p.parse_args()
    raise SystemExit(run(args))


if __name__=='__main__':
    main()
