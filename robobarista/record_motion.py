"""Record a synthetic moving pitcher with three actual rendered RGB-D cameras.

Run:
  MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python -m robobarista.record_motion \
    --frames 25 --fps 8 --width 960 --height 720

Important: This generates a programmed *kinematic* virtual movement.
No human demonstration, collision checked grasp, liquid, or robot replay yet.
Ground truth only feeds separate evaluation outputs; no estimator input.
"""
from __future__ import annotations
import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from .demo import get_gt, render_pair, to_json
from .geometry import Calibration, bundle_adjust_object, detect_tags, pose_errors, solve_single
from .motion import frame_times, motion_sample, valid_pose, world_spout
from .scene import (
    generate_scene, side_camera_layout, tag_definitions, SIDE_TAG_SIDE_M,
    MARKER_TEXTURE_PX, MARKER_QUIET_PX, MARKER_ACTIVE_M,
    SIDE_PITCHER_TAG_IDS, SPOUT_TIP_LOCAL_M,
)


def _write_csv(path: Path, rows, fields):
    with path.open('w', newline='', encoding='utf-8') as file:
        writer=csv.DictWriter(file,fieldnames=fields,extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def _pose_fields(result, prefix):
    if result is None:
        return {f'{prefix}_{k}':None for k in
                ('x_m','y_m','z_m','qx','qy','qz','qw')}
    p=np.asarray(result['position'],dtype=float).reshape(3)
    q=Rotation.from_matrix(result['rotation']).as_quat()
    return {f'{prefix}_{key}':float(value) for key,value in zip(
        ('x_m','y_m','z_m','qx','qy','qz','qw'),[p[0],p[1],p[2],*q])}


def _spout_fields(pos, key='spout'):
    if pos is None:
        return {f'{key}_{axis}_m':None for axis in 'xyz'}
    return {f'{key}_{axis}_m':float(v) for axis,v in
            zip('xyz',np.asarray(pos,dtype=float).reshape(3))}


def _preview_frame(images: dict, camera_names, frame, time_s, phase,
                   pitcher_cameras, tip_error_mm):
    panel_w,panel_h=340,290
    image_h=250
    collage=Image.new('RGB',(panel_w*len(camera_names),panel_h),(240,242,245))
    draw=ImageDraw.Draw(collage)
    if tip_error_mm is None:
        stat='NO VALID POSE'
    else:
        stat=f'spout error {tip_error_mm:.3f} mm'
    for k,name in enumerate(camera_names):
        image=Image.fromarray(images[name]).resize((panel_w,image_h),Image.Resampling.BILINEAR)
        collage.paste(image,(panel_w*k,40))
        draw.text((panel_w*k+7,5),f'{name}',fill=(10,15,20))
        draw.text((panel_w*k+7,21),f'#{frame:02d} {time_s:.2f}s | {phase}',fill=(10,15,20))
    draw.rectangle((0,panel_h-25,panel_w*len(camera_names),panel_h),fill=(19,28,40))
    draw.text((12,panel_h-19),
              f'PITCHER {pitcher_cameras}/{len(camera_names)} cameras | {stat}',
              fill=(255,255,255))
    return collage


def _trajectory_plot(rows, folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=np.array([r['time_s'] for r in rows])
    fig,ax=plt.subplots(figsize=(9,4.5))
    for component,color in zip('xyz',['C0','C1','C2']):
        values=np.array([r.get('spout_'+component+'_m')
                         if r.get('spout_'+component+'_m') is not None
                         else np.nan for r in rows],float)
        ax.plot(t,values,label='spout '+component,color=color,marker='.',ms=3)
    ax.set(xlabel='Virtual timestamp (s)',ylabel='Estimated spout position (m)',
           title='Spout trajectory estimated ONLY from camera images')
    ax.grid(alpha=.2);ax.legend()
    fig.tight_layout()
    fig.savefig(folder/'trajectory.png',dpi=140)
    plt.close(fig)


def _error_plot(rows, folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=[r['time_s'] for r in rows]
    errors=[r['spout_tip_error_mm'] if r['spout_tip_error_mm'] is not None
            else np.nan for r in rows]
    fig,ax=plt.subplots(figsize=(9,4))
    ax.plot(t,errors,marker='o',ms=3)
    ax.axhline(.5,color='gray',linestyle='--',label='0.5 mm research goal')
    ax.set(xlabel='Virtual timestamp (s)',ylabel='Spout error vs. MuJoCo truth (mm)',
           title='Independent evaluation (NOT estimator input)')
    ax.grid(alpha=.2);ax.legend()
    fig.tight_layout()
    fig.savefig(folder/'validation.png',dpi=140)
    plt.close(fig)


def run(args):
    import mujoco

    if args.frames < 5:
        raise ValueError('Please provide at least 5 frames to show all phases')
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    cameras=side_camera_layout(3,1)
    defs=tag_definitions(pitcher_side_tags=True,pitcher_tag_count=1)
    tag_side_by_id={tid: (
        SIDE_TAG_SIDE_M*(MARKER_TEXTURE_PX-2*MARKER_QUIET_PX)/MARKER_TEXTURE_PX
        if tid in SIDE_PITCHER_TAG_IDS.values() else MARKER_ACTIVE_M
    ) for tid in defs}
    model_path=generate_scene(Path(args.vendor_dir).resolve(),
                               pitcher_side_tags=True,camera_count=3,
                               pitcher_tag_count=1)
    model=mujoco.MjModel.from_xml_path(str(model_path))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,height=args.height,width=args.width)
    calibrations={name:Calibration.from_mujoco(model,data,name,args.width,args.height)
                  for name in cameras}
    for name,calib in calibrations.items():
        (out/f'{name}_calibration.json').write_text(
            json.dumps(calib.saveable(),indent=2),encoding='utf-8')

    body=model.body('rb_pitcher')
    first=get_gt(model,data,'pitcher')
    start_position=first['position'].copy()
    start_rotation=first['rotation'].copy()
    samples=[];evaluations=[];previews=[]
    # The cup is fixed in this experimental scene. Calibrate it from camera
    # images once and retain that OBSERVED pose as the world reference, so
    # temporary visual occlusion during pouring doesn't invalidate the trajectory.
    # This is never initialized from MuJoCo ground truth.
    cup_reference=None
    cup_reference_frame=None
    t0=time.monotonic()
    frame_root=out/'frames'
    try:
        for frame,time_s,u in frame_times(args.frames,args.fps):
            desired=motion_sample(u,start_position,start_rotation)
            body.pos[:]=desired.position_m
            q=Rotation.from_matrix(desired.rotation).as_quat()
            body.quat[:]=[q[3],q[0],q[1],q[2]]
            # Kinematically driven scripted pose (not a MuJoCo control policy).
            mujoco.mj_forward(model,data)

            groups={tid:[] for tid in defs}
            annotated_frames={}
            for name,cal in calibrations.items():
                rgb,depth=render_pair(renderer,data,name)
                annotated=rgb.copy()
                detections=detect_tags(rgb,include_apriltags=True,detector='pupil')
                for tid,corners in detections:
                    if tid not in defs:
                        continue
                    est=solve_single(corners,depth,cal,tag_side_by_id[tid])
                    if est is None:
                        continue
                    groups[tid].append(est)
                    cv2.polylines(annotated,
                                  [np.rint(corners).astype(np.int32)],
                                  True,(10,240,15),2)
                    center=np.mean(corners,axis=0).astype(int)
                    cv2.putText(annotated,str(tid),tuple(center),
                                cv2.FONT_HERSHEY_SIMPLEX,.7,(10,240,15),2)
                folder=frame_root/name
                folder.mkdir(parents=True,exist_ok=True)
                Image.fromarray(annotated).save(
                    folder/f'frame_{frame:04d}.jpg',quality=82)
                annotated_frames[name]=annotated
                if frame==0:
                    np.save(out/f'{name}_first_depth_m.npy',depth)

            poses={}
            views={}
            for label in ('pitcher','cup'):
                obs=[(tid,o) for tid,(tag_label,_) in defs.items()
                     if tag_label==label for o in groups[tid]]
                views[label]=len(set(id(o['calibration']) for _,o in obs))
                estimate=bundle_adjust_object(obs,defs,tag_side_by_id) if obs else None
                poses[label]=estimate if valid_pose(estimate,views[label]) else None

            pitcher=poses['pitcher']
            cup_now=poses['cup']
            if cup_reference is None and cup_now is not None:
                cup_reference=cup_now
                cup_reference_frame=frame
            tip=world_spout(pitcher['position'],pitcher['rotation'],SPOUT_TIP_LOCAL_M) if pitcher else None
            relative=(tip-cup_reference['position']
                      if tip is not None and cup_reference is not None else None)
            gt_pitcher=get_gt(model,data,'pitcher')
            gt_cup=get_gt(model,data,'cup')
            true_spout=world_spout(gt_pitcher['position'],gt_pitcher['rotation'],
                                   SPOUT_TIP_LOCAL_M)
            pos_err,rot_err=pose_errors(pitcher,gt_pitcher) if pitcher else (None,None)
            tip_err=float(np.linalg.norm(tip-true_spout)*1000) if tip is not None else None
            relative_error=(float(np.linalg.norm(
                relative-(true_spout-gt_cup['position']))*1000)
                            if relative is not None else None)

            row=dict(frame=frame,time_s=float(time_s),phase=desired.phase,
                     pitcher_cameras=views['pitcher'],cup_cameras=views['cup'],
                     pitcher_valid=pitcher is not None,
                     cup_observed_this_frame=cup_now is not None,
                     cup_reference_available=cup_reference is not None,
                     cup_reference_frame=cup_reference_frame,
                     relative_valid=relative is not None,
                     **_pose_fields(pitcher,'pitcher'),
                     **_pose_fields(cup_reference,'cup_reference'),
                     **_spout_fields(tip,'spout'),
                     **_spout_fields(relative,'spout_relative_to_cup'))
            samples.append(row)
            evaluation=dict(frame=frame,time_s=float(time_s),
                            phase=desired.phase,
                            **_pose_fields(gt_pitcher,'true_pitcher'),
                            **_spout_fields(true_spout,'true_spout'),
                            pitcher_center_error_mm=pos_err,
                            pitcher_rotation_error_deg=rot_err,
                            spout_tip_error_mm=tip_err,
                            relative_spout_to_cup_error_mm=relative_error)
            evaluations.append(evaluation)
            previews.append(_preview_frame(
                annotated_frames,list(cameras),frame,time_s,desired.phase,
                views['pitcher'],tip_err))
            print(f'Frame {frame+1:02d}/{args.frames} '
                  f'{desired.phase:10s} pitcher {views["pitcher"]}/3 cams '
                  f'| tip {tip_err if tip_err is not None else "MISSING"} mm',
                  flush=True)
    finally:
        renderer.close()

    if previews:
        gif_frames=[x.convert('P',palette=Image.Palette.ADAPTIVE,colors=96)
                    for x in previews]
        gif_frames[0].save(out/'preview.gif',save_all=True,
                           append_images=gif_frames[1:],
                           duration=int(round(1000/args.fps)),
                           loop=0,optimize=False)

    # Independent vision-only trajectory: never silently insert simulator poses.
    fields=list(samples[0].keys())
    _write_csv(out/'trajectory.csv',samples,fields)
    (out/'trajectory.json').write_text(
        json.dumps(to_json(dict(
            source='detected rendered images, independent of MuJoCo ground truth',
            motion_source='synthetic kinematic test gesture, NOT recorded human motion',
            fps=args.fps,cameras=list(cameras),samples=samples
        )),indent=2),encoding='utf-8')
    _write_csv(out/'evaluation.csv',evaluations,list(evaluations[0]))
    (out/'evaluation.json').write_text(
        json.dumps(to_json(evaluations),indent=2),encoding='utf-8')
    _trajectory_plot(samples,out)
    _error_plot(evaluations,out)

    pitcher_good=sum(r['pitcher_valid'] for r in samples)
    rel_good=sum(r['relative_valid'] for r in samples)
    err=[r['spout_tip_error_mm'] for r in evaluations
         if r['spout_tip_error_mm'] is not None]
    rot=[r['pitcher_rotation_error_deg'] for r in evaluations
         if r['pitcher_rotation_error_deg'] is not None]
    max_gap=0;gap=0
    for r in samples:
        gap=0 if r['pitcher_valid'] else gap+1
        max_gap=max(max_gap,gap)
    quality=(pitcher_good==args.frames and
             bool(err) and max(err)<=args.max_spout_mm and
             bool(rot) and max(rot)<=args.max_rotation_deg)
    report={
        'status':'PASS' if quality else 'FAIL',
        'interpretation':'Kinematically scripted moving pitcher vision test; no human latte-art skill or FR5 replay yet',
        'camera_setup':'3 camera, one side AprilTag 36h11, pupil detector, bundle_adjust',
        'frames':args.frames,'fps':args.fps,'virtual_duration_s':(args.frames-1)/args.fps,
        'pitcher_valid_frames':pitcher_good,
        'cup_observed_frames':sum(r['cup_observed_this_frame'] for r in samples),
        'cup_reference_frame':cup_reference_frame,
        'cup_reference_type':'first valid multiview camera observation; held fixed because simulated cup is stationary',
        'relative_valid_frames':rel_good,
        'longest_missing_pitcher_run_frames':max_gap,
        'max_spout_tip_error_mm':max(err) if err else None,
        'max_pitcher_rotation_error_deg':max(rot) if rot else None,
        'allowed_spout_mm':args.max_spout_mm,
        'allowed_rotation_deg':args.max_rotation_deg,
        'runtime_s':round(time.monotonic()-t0,2),
        'note':'PASS refers to synthetic ideal-camera trajectory only, not physical precision',
        'files':['preview.gif','trajectory.csv','trajectory.json',
                 'evaluation.csv','evaluation.json','trajectory.png','validation.png']
    }
    (out/'summary.json').write_text(
        json.dumps(report,indent=2),encoding='utf-8')
    print('\n=== Dynamic motion capture ===')
    print(json.dumps(report,indent=2))
    print('Outputs:',out)
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--frames',type=int,default=25)
    parser.add_argument('--fps',type=float,default=8)
    parser.add_argument('--width',type=int,default=960)
    parser.add_argument('--height',type=int,default=720)
    parser.add_argument('--output',default='runs/motion_capture')
    parser.add_argument('--vendor-dir',default='vendor')
    parser.add_argument('--max-spout-mm',type=float,default=.5)
    parser.add_argument('--max-rotation-deg',type=float,default=1.)
    run(parser.parse_args())
    # OSMesa/PyOpenGL occasionally segfaults during Python interpreter
    # shutdown *after* all MuJoCo frames, GIFs and JSON files have been
    # successfully written. It is an external GL teardown bug, not a failed
    # camera pose estimate. Only use this clean-process exit after run()
    # returns successfully; any actual exception still propagates as an error.
    import os, sys
    sys.stdout.flush()
    sys.stderr.flush()
    if os.environ.get('MUJOCO_GL','').lower()=='osmesa':
        os._exit(0)


if __name__=='__main__':
    main()
