"""Render real FR5 kinematic SAFE staging routes for named coffee-machine bays.

A staging pose is at least 14 cm OUTSIDE the exact contact/dock position.
It is deliberately not a port insertion. Everything in this storyboard
passes conservative MuJoCo robot contact and carried pitcher/cup/portafilter
volume checks along all interpolated joint segments; impossible routes
remain BLOCKED and are never rendered as approved arm motion.
"""
from __future__ import annotations
import argparse
import json
import os
import shutil
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image,ImageDraw

from .barista_cell import build_cell_scene,CAMERAS
from .camera_joint_replay import _find_start,_write_q
from .robot_replay import robot_qpos_addresses
from .safe_staging import plan_staging,_check_route
from .demo import render_pair
from .workcell_constraints import DOCKS
from .cell_demo import _set_free_pose


def _image_panel(rgb,name,progress,count,status):
    im=Image.fromarray(rgb).resize((960,720),Image.Resampling.BILINEAR)
    frame=Image.new('RGB',(960,770),(24,32,39))
    frame.paste(im,(0,48))
    d=ImageDraw.Draw(frame)
    d.text((16,11),f'{name}  | {progress:.0%}  | '
                      f'checked joint steps: {count}',fill='white')
    d.text((15,746),status,fill=(245,229,166))
    return frame


def _make_video(frames,out,fps):
    if not frames:return False
    pal=[im.convert('P',palette=Image.Palette.ADAPTIVE,colors=80)
         for im in frames]
    pal[0].save(out/'safe_staging.gif',save_all=True,
                append_images=pal[1:],duration=round(1000/fps),
                loop=0,optimize=False)
    if not shutil.which('ffmpeg'):
        return False
    folder=out/'frames';folder.mkdir(exist_ok=True)
    for i,f in enumerate(frames):f.save(folder/f'img_{i:04d}.png')
    subprocess.run(['ffmpeg','-y','-loglevel','error','-framerate',
      str(fps),'-i',str(folder/'img_%04d.png'),'-c:v','libx264',
      '-pix_fmt','yuv420p',str(out/'safe_staging.mp4')],check=True)
    return (out/'safe_staging.mp4').stat().st_size>1000


def run(args):
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    model=mujoco.MjModel.from_xml_path(str(build_cell_scene(
        Path(args.vendor_dir).resolve())))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    # Keep source pitcher out of physical tests while arm 'carries' the
    # *conservative tool spheres* virtually.
    _set_free_pose(model,data,'rb_pitcher_free',[-10,-10,-10],0)
    _set_free_pose(model,data,'rb_plastic_cup_free',[-10,-10,-10],0)
    mujoco.mj_forward(model,data)
    qids,limits=robot_qpos_addresses(model)
    home,home_xyz=_find_start(model,data,qids,limits,args.seed)
    stages=plan_staging(model,data,qids,limits,home,args.seed)
    allowed=[key for key,val in stages.items()
             if val['status']=='PRE_DOCK_ROUTE_CHECKED']
    blocked=[key for key,val in stages.items()
             if val['status']=='BLOCKED']
    renderer=mujoco.Renderer(model,width=args.width,height=args.height)
    frames=[]
    try:
        for key in allowed:
            # Re-verify the whole joint route before displaying/using it.
            qtarget=np.asarray(stages[key]['stage']['q_rad'],float)
            okay,reason,n=_check_route(
                model,data,qids,limits,home,qtarget,stages[key]['tool'])
            if not okay:
                raise RuntimeError(f'Staging animation was unsafe: {key}: {reason}')
            # Safe complete out/back route. Dense joint sampling is recomputed
            # by _check_route; these display frames do not substitute that.
            sequence=[]
            for t in np.linspace(0,1,args.frames_per_station//2+1):
                u=t*t*(3-2*t)
                sequence.append(home+(qtarget-home)*u)
            for t in np.linspace(0,1,args.frames_per_station//2+1)[1:]:
                u=t*t*(3-2*t)
                sequence.append(qtarget+(home-qtarget)*u)
            # Render only after all local arm states were screened.
            for i,q in enumerate(sequence):
                _write_q(model,data,qids,q)
                rgb,depth=render_pair(renderer,data,'cell_overview')
                frames.append(_image_panel(rgb,key,
                   i/(len(sequence)-1),n,
                   'KINEMATISCH SICHERES VOR-DOCK – KEIN KONTAKT MIT GERAETEN'))
        if not frames:
            _write_q(model,data,qids,home)
            rgb,_=render_pair(renderer,data,'cell_overview')
            frames.append(_image_panel(rgb,'Keine freigegebene Station',
                          0,0,'Alle Stationen blockiert: Kollisionen vermeiden!'))
    finally:
        renderer.close()
    mp4=_make_video(frames,out,args.fps)
    summary={
       'status':'PASS' if allowed else 'NO_APPROVED_STAGE',
       'source':'Artly-inspired workcell, real FR5 six joints',
       'pre_dock_allowed_count':len(allowed),
       'pre_dock_allowed':allowed,
       'blocked_count':len(blocked),
       'blocked':blocked,
       'stage_poses':stages,
       'robot_both_direction_joint_sweeps_checked':True,
       'payload_collision_volume_applied':True,
       'actual_contact_docking_approved':False,
       'real_motor_dynamics_tested':False,
       'mp4_created':mp4,
       'note':'Only approach/staging poses in free space; no machine contact, final docking, grasp or safety certification.'
    }
    (out/'safe_staging_report.json').write_text(
        json.dumps(summary,indent=2),encoding='utf-8')
    print('STAGING SUMMARY:',json.dumps({k:v for k,v in summary.items()
           if k!='stage_poses'},indent=2),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--vendor-dir',default='vendor')
    p.add_argument('--output',default='runs/safe_staging')
    p.add_argument('--seed',type=int,default=14)
    p.add_argument('--width',type=int,default=960)
    p.add_argument('--height',type=int,default=720)
    p.add_argument('--fps',type=float,default=6)
    p.add_argument('--frames-per-station',type=int,default=12)
    report=run(p.parse_args())
    import sys
    sys.stdout.flush();sys.stderr.flush()
    if os.environ.get('MUJOCO_GL','').lower()=='osmesa':
        os._exit(0 if report['mp4_created'] else 2)
    if not report['mp4_created']:
        raise SystemExit(2)


if __name__=='__main__':
    main()
