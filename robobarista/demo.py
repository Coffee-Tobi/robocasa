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
from .scene import (generate_scene, CAMERAS, side_camera_layout, TAG_IDS,
                    SIDE_PITCHER_TAG_IDS, SIDE_TAG_SIDE_M, MARKER_ACTIVE_M,
                    MARKER_TEXTURE_PX, MARKER_QUIET_PX,
                    SPOUT_TIP_LOCAL_M, tag_definitions)
from .geometry import (Calibration, detect_tags, solve_single, bundle_adjust_tag,
                       bundle_adjust_object, object_from_tag, pose_errors)
from .benchmark_gates import evaluate_side


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
    # Dynamic layout so 4th camera is visible instead of cropping it.
    w,h=(450,340) if len(rgbs)>3 else (540,400)
    canvas=Image.new('RGB',(w*len(rgbs),h*2+40),'#f3f4f6')
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
    scene_path=generate_scene(Path(args.vendor_dir).resolve(),
                              pitcher_side_tags=args.pitcher_side_tags,
                              camera_count=args.cameras,
                              pitcher_tag_count=args.pitcher_tags)
    model=mujoco.MjModel.from_xml_path(str(scene_path))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,height=args.height,width=args.width)
    cameras = side_camera_layout(args.cameras,args.pitcher_tags) if args.pitcher_side_tags else CAMERAS
    if args.pose_method == 'caliscope':
        from .caliscope_integration import estimate_object_with_caliscope
    calibrations={name:Calibration.from_mujoco(model,data,name,args.width,args.height)
                  for name in cameras}
    defs=tag_definitions(pitcher_side_tags=args.pitcher_side_tags,
                         pitcher_tag_count=args.pitcher_tags)
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
            detections=detect_tags(rgb,include_apriltags=args.pitcher_side_tags,
                                   detector=args.detector)
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
                if not all_observations:
                    est=None
                elif args.pose_method == 'caliscope':
                    est=estimate_object_with_caliscope(all_observations,defs,tag_sides)
                else:
                    est=bundle_adjust_object(all_observations,defs,tag_sides)
                views=len(set(id(obs['calibration']) for _,obs in all_observations))
                row={'trial':trial,'object':label,
                     'tag_ids':sorted(set(tid for tid,_ in all_observations)),
                     'visible_cameras':sorted(name for name,cam in calibrations.items()
                                              if any(obs['calibration'] is cam
                                                     for _,obs in all_observations)),
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
    # Separate metric: the relative tip-to-cup vector matters for latte art.
    # Even a perfect pitcher pose will not rescue a mislocalized cup.
    if args.pitcher_side_tags:
        for trial in range(args.trials):
            per_trial={r['object']:r for r in reports if r['trial']==trial}
            pitcher=per_trial.get('pitcher')
            cup=per_trial.get('cup')
            if not pitcher or not cup or pitcher['failed'] or cup['failed']:
                continue
            tip_true=(np.array(pitcher['truth_position_m']) +
                      np.array(pitcher['truth_rotation']) @ SPOUT_TIP_LOCAL_M)
            tip_est=(np.array(pitcher['estimate_position_m']) +
                     np.array(pitcher['estimate_rotation']) @ SPOUT_TIP_LOCAL_M)
            rel_true=tip_true - np.array(cup['truth_position_m'])
            rel_est=tip_est - np.array(cup['estimate_position_m'])
            pitcher['spout_to_cup_error_mm']=float(np.linalg.norm(rel_est-rel_true)*1000)
    renderer.close()
    save_montage(frames,depth_frames,out/'rgbd_montage.jpg')
    save_plot(reports,calibrations,out)
    serializable=to_json(reports)
    (out/'measurements.json').write_text(json.dumps(serializable,indent=2))
    with (out/'pose_errors.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['trial','object','translation_mm','rotation_deg','spout_tip_error_mm','spout_to_cup_error_mm','cameras_seen','reprojection_px','failed'])
        for r in reports:
            writer.writerow([r['trial'],r['object'],r.get('translation_mm'),r.get('rotation_deg'),
                             r.get('spout_tip_error_mm'),r.get('spout_to_cup_error_mm'),r['cameras_seen'],
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
        # Separate diagnostic gates make a failing cup / obstacle explicit.
        # Keep legacy scene-status semantics, never relabel a scene FAIL as PASS.
        gates=evaluate_side(reports,min_cameras=args.min_cameras,
                           max_spout_mm=args.max_spout_mm,
                           max_pitcher_deg=args.max_pitcher_rotation_deg,
                           max_other_mm=args.max_translation_mm,
                           max_other_deg=args.max_rotation_deg)
        assert passed == (gates['scene_status']=='PASS'), 'Acceptance gate mismatch'
    else:
        passed=(quality_ok
                and max(T,default=float('inf'))<=args.max_translation_mm
                and max(R,default=float('inf'))<=args.max_rotation_deg)
        gates=None
    pitcher_measured=[r for r in reports if r['object']=='pitcher' and not r['failed']]
    tip_errors=[r['spout_tip_error_mm'] for r in reports if 'spout_tip_error_mm' in r]
    rel_errors=[r['spout_to_cup_error_mm'] for r in reports
                if 'spout_to_cup_error_mm' in r]
    summary={'status':'PASS' if passed else 'FAIL',
        'pitcher_status':gates['pitcher_status'] if gates is not None else None,
        'pitcher_failures':gates['pitcher_failures'] if gates is not None else [],
        'other_failures':gates['other_failures'] if gates is not None else [],
        'experiment':'pitcher_side_tags' if args.pitcher_side_tags else 'original_top_tags',
        'detector':args.detector if args.pitcher_side_tags else 'opencv',
        'pose_method':args.pose_method if args.pitcher_side_tags else 'bundle_adjust',
        'pitcher_tag_count':args.pitcher_tags if args.pitcher_side_tags else 1,
        'configured_camera_count':len(cameras),
        'max_pitcher_rotation_deg':max(
            (r['rotation_deg'] for r in pitcher_measured), default=None),
        'min_pitcher_cameras_seen':min(
            (r['cameras_seen'] for r in reports if r['object']=='pitcher'), default=0),
        'pitcher_valid_trials':len(pitcher_measured),
        'spout_geometry':'simplified known CAD offset; real fixture/calibration not validated',
        'max_spout_tip_error_mm':max(tip_errors) if tip_errors else None,
        'max_relative_tip_to_cup_error_mm':max(rel_errors) if rel_errors else None,
        'relative_tip_to_cup_goal_mm':0.5 if args.pitcher_side_tags else None,
        'relative_goal_in_acceptance_gate':False,  # next step needs better cup calibration
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
    if args.pitcher_side_tags:
        print('Max relative spout-to-cup error (mm):',
              summary['max_relative_tip_to_cup_error_mm'],
              '| future research goal:',summary['relative_tip_to_cup_goal_mm'])
    if gates is not None:
        print('Pitcher-only:',gates['pitcher_status'],
              '| Entire scene:',gates['scene_status'])
        for issue in gates['pitcher_failures']+gates['other_failures']:
            print('Gate issue:',issue)
    print('\nResult:',summary['status'],'| data in',out)
    if not passed:
        print('Accuracy gate NOT passed. Check image visibility, marker orientation/UV and calibration.',file=sys.stderr)
    return 0 if passed else 2


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--cameras',type=int,choices=(3,4),default=3,
                   help='Number of simulated cameras in side-tag experiment')
    p.add_argument('--pitcher-tags',type=int,choices=(1,2),default=2,
                   help='Number of rigid AprilTags on the pitcher (near/far)')
    p.add_argument('--pose-method',choices=('bundle_adjust','caliscope'),
                   default='bundle_adjust',
                   help='Existing 2D bundle adjustment vs Caliscope RGB triangulation + rigid fit')
    p.add_argument('--detector',choices=('opencv','pupil'),default='opencv',
                   help='OpenCV AprilTag vs native AprilTag-3 pupil binding')
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
