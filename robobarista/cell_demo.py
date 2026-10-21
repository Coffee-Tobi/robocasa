"""Render & assess new island setup. No liquids or realistic grasping yet.

Preview: scripted tamper piston stroke plus kinematically repositioned free
pitcher. A motor actuator exists but preview does NOT invoke mj_step or prove
contact force. Camera visibility is tested on rendered AprilTag pixels.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

import cv2
import numpy as np
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation

from .barista_cell import (build_cell_scene,STATIONS,CAMERAS,TAG_ID,
                            TABLE_TOP,ROBOT_BASE)
from .geometry import Calibration,detect_tags
from .demo import to_json,render_pair
from .robot_replay import robot_qpos_addresses,compute_ik,HOME_HINT


def _set_free_pose(model,data,joint,xyz,angle_y_deg):
    joint_id=model.joint(joint).id
    import mujoco
    if model.jnt_type[joint_id]!=mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError(f'{joint} must be a real freejoint')
    qadr=int(model.jnt_qposadr[joint_id])
    q=Rotation.from_euler('y',angle_y_deg,degrees=True).as_quat()
    data.qpos[qadr:qadr+7]=[*xyz,q[3],q[0],q[1],q[2]]
    data.qvel[int(model.jnt_dofadr[joint_id]):int(model.jnt_dofadr[joint_id])+6]=0.


def _image_with_detections(rgb,dets):
    img=rgb.copy()
    for tid,corners in dets:
        cv2.polylines(img,[np.round(corners).astype(np.int32)],
                      True,(55,255,105),2)
        center=np.mean(corners,axis=0).astype(int)
        cv2.putText(img,f'#{tid}',tuple(center),
                    cv2.FONT_HERSHEY_SIMPLEX,.58,(40,250,55),2)
    return img


def _overview(rgb,calib):
    im=Image.fromarray(rgb)
    d=ImageDraw.Draw(im)
    pretty={
      'grinder':'Muehle','tamper':'Tamper',
      'portafilter':'Siebtraeger','espresso':'Espresso',
      'milk_dispenser':'Milchspender','pitcher':'Milchkanne',
      'pour':'Kunststoffbecher','rinser':'Rinser'}
    for station,xyz in STATIONS.items():
        uv=calib.project(np.asarray(xyz,float))
        if uv is None:continue
        x,y=map(float,uv)
        if x<0 or x>=calib.width or y<0 or y>=calib.height:continue
        name=pretty[station]
        w=7.4*len(name)+14
        d.rounded_rectangle((x-5,y-21,x+w,y+1),radius=5,
                            fill=(22,31,42),outline=(160,209,215),width=1)
        d.text((x+2,y-17),name,fill='white')
    return im


def _side_panel(images,progress,depth_m):
    overview=images['cell_overview'].resize((1000,700),Image.Resampling.BILINEAR)
    result=Image.new('RGB',(1500,780),(243,246,248))
    result.paste(overview,(0,58))
    other=['cell_left','cell_right','cell_back']
    for i,name in enumerate(other):
        tile=images[name].resize((480,215),Image.Resampling.BILINEAR)
        result.paste(tile,(1010,65+i*233))
    d=ImageDraw.Draw(result)
    d.rectangle((0,0,1500,54),fill=(21,33,42))
    d.text((22,12),'RoboBarista | Artly-inspirierte kompakte Arbeitsinsel',
           fill=(244,247,250),stroke_width=0)
    d.text((22,35),f'Tamperhub: {depth_m*1000:.0f} mm  |  Animation: {progress:.0%}',
           fill=(167,222,225))
    for i,name in enumerate(other):
        d.rectangle((1010,65+i*233,1490,86+i*233),fill=(14,24,30))
        d.text((1021,68+i*233),f'Kamera {i+1}: {name}',fill='white')
    d.text((20,760),'Marker: gruene Umrandung = im gerenderten Bild erkannt',
           fill=(30,45,54))
    return result


def _top_down(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names={
      'grinder':'Muehle','tamper':'Tamper',
      'portafilter':'Siebtraeger','espresso':'Espressomaschine',
      'milk_dispenser':'Milchspender','pitcher':'Milchkanne',
      'pour':'Plastikbecher','rinser':'Rinser'}
    fig,ax=plt.subplots(figsize=(9,8))
    ax.add_patch(plt.Rectangle((-.95,-.79),1.9,1.58,
                               fill=False,lw=2,label='Insel'))
    ax.add_patch(plt.Circle((0,0),.8,fill=False,ls='--',lw=1.4,
                            label='0.8 m nur PLANUNGS-Kontur (nicht IK-Nachweis)'))
    ax.scatter([0],[0],s=180,marker='X',label='FR5 Sockel',zorder=3)
    for key,(x,y,z) in STATIONS.items():
        ax.scatter([x],[y],s=100,zorder=3)
        ax.annotate(names[key],(x,y),xytext=(6,6),
                    textcoords='offset points',fontsize=9)
    for key,(pos,target) in CAMERAS.items():
        ax.scatter([pos[0]],[pos[1]],s=90,marker='^')
    ax.set_xlim(-1.3,1.3);ax.set_ylim(-1.2,1.25)
    ax.set_aspect('equal')
    ax.set(xlabel='X (m)',ylabel='Y (m)',
           title='RoboBarista Arbeitsinsel – Draufsicht (Planungskoordinaten)')
    ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=8)
    fig.tight_layout()
    fig.savefig(out/'layout_topdown.png',dpi=150)
    plt.close(fig)


def _reachcheck(model,data):
    """Attempt real FR5 FK/IK to each station using multiple tool orientations.

    A success is meaningful only for the TESTED tool pose, without payload,
    full collision-free transfer, velocities or approach feasibility.
    """
    import mujoco
    tcp=model.body('rb_cell_tcp').id
    qids,limits=robot_qpos_addresses(model)
    seeds=[
        HOME_HINT,
        np.array([0.,-.8,1.2,-1.1,.5,0.]),
        np.array([1.,-.7,1.4,-1.0,1.,0.]),
        np.array([-1.,-.7,1.4,-1.0,-1.,0.])
    ]
    orientations=[
        Rotation.from_euler('xyz',[0,0,0],degrees=True).as_matrix(),
        Rotation.from_euler('xyz',[0,90,0],degrees=True).as_matrix(),
        Rotation.from_euler('xyz',[180,0,0],degrees=True).as_matrix(),
    ]
    result={}
    base=np.asarray(ROBOT_BASE,float)
    for name,point in STATIONS.items():
        p=np.asarray(point,float)
        attempts=[]
        for R in orientations:
            for q in seeds:
                try:
                    sol=compute_ik(model,data,tcp,qids,limits,p,R,q,max_evals=70)
                except (ValueError,RuntimeError) as exc:
                    attempts.append(dict(solved=False,error=str(exc)))
                    continue
                attempts.append(dict(solved=sol['solved'],
                    position_error_mm=sol['position_error_mm'],
                    rotation_error_deg=sol['rotation_error_deg']))
                if sol['solved']:break
            if any(x.get('solved') for x in attempts):break
        achieved=[x for x in attempts if x.get('solved')]
        result[name]={
           'sample_target_world_m':p.tolist(),
           'distance_robot_base_m':float(np.linalg.norm(p-base)),
           'tested_pose_ik':bool(achieved),
           'best_position_error_mm':min(
               [x['position_error_mm'] for x in attempts
                if 'position_error_mm' in x],default=None),
           'attempts':len(attempts),
           'note':'Sample TCP target only; not collision-free approach or actual tool interaction'
        }
        print('Reachability:',name,
              'sample IK' if achieved else 'NOT YET CONFIRMED',
              'attempts',len(attempts),flush=True)
    return result


def run(args):
    import mujoco
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    xml=build_cell_scene(Path(args.vendor_dir))
    model=mujoco.MjModel.from_xml_path(str(xml))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    names=list(CAMERAS)+['cell_overview']
    renderer=mujoco.Renderer(model,height=args.height,width=args.width)
    calibration={n:Calibration.from_mujoco(model,data,n,args.width,args.height)
                 for n in names}
    _top_down(out)
    tamper_id=model.joint('rb_tamper_slide').id
    tamper_adr=int(model.jnt_qposadr[tamper_id])
    pitcher_start=model.body('rb_milk_pitcher').pos.copy()
    frames=[]
    visibility=[]
    try:
        for i in range(args.frames):
            u=i/(args.frames-1)
            # Scripted tamper stroke, same slide as the actual motor actuator:
            # first third descend, middle stay, last third retract.
            if u<.34:
                depth=.055*min(1.,u/.34)
            elif u<.68:
                depth=.055
            else:
                depth=.055*max(0.,(1-u)/.32)
            data.qpos[tamper_adr]=depth
            # Pitcher is a real MuJoCo free body with collision and mass. This
            # *preview* repositions qpos kinematically (not robotic grasp).
            # Hold in parking station for first half, then raise/tilt for pour.
            if u<.50:
                pitcher_xyz=pitcher_start
                pitcher_angle=0.
            else:
                h=(u-.50)/.50
                smooth=h*h*(3-2*h)
                pitcher_xyz=pitcher_start + smooth*np.array([.31,.16,.15])
                pitcher_angle=24*smooth
            _set_free_pose(model,data,'rb_pitcher_free',pitcher_xyz,pitcher_angle)
            mujoco.mj_forward(model,data)
            all_views={}
            observed={}
            for name in names:
                rgb,depth_img=render_pair(renderer,data,name)
                if name=='cell_overview':
                    all_views[name]=_overview(rgb,calibration[name])
                    continue
                dets=detect_tags(rgb,include_apriltags=True,detector='pupil')
                dets=[(tid,px) for tid,px in dets
                      if tid in set(TAG_ID.values()) | {23}]
                for tid,px in dets:
                    observed.setdefault(tid,[]).append(name)
                all_views[name]=Image.fromarray(_image_with_detections(rgb,dets))
            pic=_side_panel(all_views,u,depth)
            frames.append(pic)
            visibility.append({'frame':i,'progress':u,
                'visible_tags':{str(tid):sorted(set(cams))
                                for tid,cams in observed.items()},
                'tamper_slide_m':depth,
                'pitcher_pose':pitcher_xyz.tolist(),
                'pitcher_angle_y_deg':pitcher_angle})
            print('Frame',i+1,'/',args.frames,
                  'detected tags',sorted(observed),
                  'tamper(mm)',round(depth*1000,1),flush=True)
    finally:
        renderer.close()
    if frames:
        frames[0].save(out/'workcell_preview.png')
        frames[-1].save(out/'workcell_last_frame.png')
        pal=[im.convert('P',palette=Image.Palette.ADAPTIVE,colors=96) for im in frames]
        pal[0].save(out/'workcell_demo.gif',save_all=True,append_images=pal[1:],
                    duration=round(1000/args.fps),loop=0,optimize=False)
    mp4=False
    if frames and shutil.which('ffmpeg'):
        folder=out/'video_frames';folder.mkdir(exist_ok=True)
        for i,img in enumerate(frames):
            img.save(folder/f'cell_{i:04d}.png')
        try:
            subprocess.run(['ffmpeg','-y','-loglevel','error',
                '-framerate',str(args.fps),'-i',str(folder/'cell_%04d.png'),
                '-c:v','libx264','-pix_fmt','yuv420p',
                str(out/'workcell_demo.mp4')],check=True)
            mp4=(out/'workcell_demo.mp4').stat().st_size>1000
        except (OSError,subprocess.CalledProcessError) as exc:
            print('MP4 optional conversion failed:',exc,flush=True)
    # Check IK after visual frames: does not change already captured images.
    reach=_reachcheck(model,data) if args.reach else {}
    (out/'reachability.json').write_text(
        json.dumps(to_json(reach),indent=2),encoding='utf-8')
    counts={str(tid):sum(str(tid) in f['visible_tags'] for f in visibility)
            for tid in set(TAG_ID.values())|{23}}
    # A tag can be temporarily hidden by arm/tools. Never claim guaranteed
    # continuous 3-camera visibility from a static layout picture.
    report={'scene':'Artly-inspired central island concept, not copied CAD',
        'status':'RENDERED',
        'stations':{k:list(v) for k,v in STATIONS.items()},
        'cameras':list(CAMERAS),
        'assets':['FR5','espresso','grinder','actuated_tamper',
                  'portafilter','milk_dispenser','freejoint_pitcher',
                  'freejoint_plastic_cup','rinser'],
        'plastic_cup_instead_of_ceramic':True,
        'tamper_actuator_exists':True,
        'tamper_motor_physics_tested':False,
        'scripted_preview_no_real_grasp':True,
        'tag_visibility_by_frame':counts,
        'pitcher_tag_visible_frames':sum(
            bool({'23','71'} & set(f['visible_tags'])) for f in visibility),
        'reachability':reach,
        'mp4_created':mp4,
        'limitations':'Markers and targets need dynamic occlusion tests. Vision not guaranteed 100%. Reachability checks represent sample tool poses, not complete collision-free paths. No fluids or automated coffee making.'}
    (out/'summary.json').write_text(
        json.dumps(to_json(report),indent=2),encoding='utf-8')
    (out/'visibility.json').write_text(
        json.dumps(to_json(visibility),indent=2),encoding='utf-8')
    print('SUMMARY:',json.dumps(to_json(report),indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--vendor-dir',default='vendor')
    p.add_argument('--output',default='runs/artly_island')
    p.add_argument('--frames',type=int,default=8)
    p.add_argument('--fps',type=float,default=5)
    p.add_argument('--width',type=int,default=920)
    p.add_argument('--height',type=int,default=660)
    p.add_argument('--reach',action='store_true',
                   help='Run whole-body FR5 IK sample-target diagnostics')
    report=run(p.parse_args())
    import sys
    sys.stdout.flush();sys.stderr.flush()
    # OSMesa/PyOpenGL occasionally segfaults in post-work shader cleanup.
    if os.environ.get('MUJOCO_GL','').lower()=='osmesa':
        os._exit(0)


if __name__=='__main__':
    main()
