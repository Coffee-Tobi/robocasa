"""FAIRINO FR5 kinematic joint replay of the existing scripted pitcher gesture.

This module does NOT move a free-floating pitcher: the virtual pitcher is a
fixed child of the real model's wrist3_link; only six FR5 hinge qpos change.
An IK optimizer solves for each wrist/pitcher pose. No mj_step, grip force,
control torque, motor dynamics, collision avoidance or liquid is claimed.

The input is the synthetic motion_sample, NOT captured camera CSV yet.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .motion import motion_sample, frame_times, world_spout
from .scene import generate_scene, SPOUT_TIP_LOCAL_M
from .demo import render_pair, to_json

JOINTS=("shoulder_pan","shoulder_lift","elbow",
        "wrist_1","wrist_2","wrist_3")
# Simplified grasp fixture: virtual wrist-to-pitcher-center transform.
# Not mechanically designed; real wrist/gripper attachment needs calibration.
WRIST_PITCHER_OFFSET_M=(-.13, 0., -.07)
HOME_HINT=np.array([0., 0., 0., -1.57079, 0., -.7853])


def prepare_grasp_scene(vendor_dir: Path):
    """Generate an isolated MJCF variant, preserving the existing vision scene."""
    scene_path=generate_scene(vendor_dir,pitcher_side_tags=True,
                              camera_count=3,pitcher_tag_count=1)
    scene=ET.parse(scene_path)
    root=scene.getroot()
    world=root.find("worldbody")
    pitcher=world.find("body[@name='rb_pitcher']")
    if pitcher is None:
        raise RuntimeError("rb_pitcher missing in generated scene")
    world.remove(pitcher)
    # A workbench-height pedestal under the robot base (the six-joint arm
    # belongs beside the counter, not at floor height inside the counter).
    ET.SubElement(world,'geom',{'name':'rb_robot_pedestal',
        'type':'cylinder','pos':'1.15 -0.42 0.225',
        'size':'0.095 0.225','rgba':'0.3 0.35 0.4 1'})
    pitcher.set('pos',' '.join(str(x) for x in WRIST_PITCHER_OFFSET_M))
    pitcher.set('quat','1 0 0 0')
    # Make the pitcher a massless purely graphical rigid fixture for this
    # kinematic milestone. No physics of grip/payload/collision is asserted.
    for geom in pitcher.iter('geom'):
        geom.set('mass','0')
        geom.set('contype','0')
        geom.set('conaffinity','0')
    import numpy as np
    from .geometry import look_at_quat
    eye=(1.32,-1.16,1.35)
    focus=(.48,0.,.51)
    q=look_at_quat(eye,focus)
    ET.SubElement(world,'camera',{'name':'rb_robot_overview',
        'pos':' '.join(str(x) for x in eye),
        'quat':' '.join(str(x) for x in q),'fovy':'57'})
    filename=scene_path.with_name("rb_robot_replay_scene.xml")
    scene.write(filename,encoding="unicode")
    # Existing rb_fr5_robot.xml already contains the six-joint articulated arm;
    # fixed pitcher has moved under that wrist using a separately generated
    # model document. MuJoCo includes are compiled before scene worldbody:
    # easiest legal MJCF setup is append the pitcher to wrist inside robot XML.
    robot_path=scene_path.with_name("rb_fr5_robot.xml")
    robot=ET.parse(robot_path)
    wrist=robot.getroot().find(".//body[@name='wrist3_link']")
    if wrist is None:
        raise RuntimeError("FR5 wrist3_link missing")
    # The upstream FR5 origin is at 0,0, pointing away from our coffee bench
    # and reaching its shoulder-pan stop as it tries to pour. Place the robot
    # NEXT TO the counter, aimed at it, as real installations would do.
    base=robot.getroot().find(".//body[@name='base_link']")
    if base is None:
        raise RuntimeError("FR5 base_link missing")
    base.set('pos','1.15 -0.42 0.45')
    base.set('euler','0 0 0')
    wrist.append(pitcher)
    attached_robot_path=scene_path.with_name('rb_fr5_replay_robot.xml')
    robot.write(attached_robot_path,encoding='unicode')
    # Remove world copy, point include to variant.
    include=scene.getroot().find('include')
    include.set('file',attached_robot_path.name)
    scene.write(filename,encoding='unicode')
    return filename


def robot_qpos_addresses(model):
    """MuJoCo joint data are NOT guaranteed to be the first six qpos."""
    import mujoco
    qids=[]
    limits=[]
    for name in JOINTS:
        jid=model.joint(name).id
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError(f"Expected hinge joint: {name}")
        qids.append(int(model.jnt_qposadr[jid]))
        limits.append(np.asarray(model.jnt_range[jid],float))
    if len(set(qids))!=6:
        raise ValueError("Duplicate robot joint qpos addresses")
    limits=np.asarray(limits)
    if np.any(limits[:,0]>=limits[:,1]):
        raise ValueError("Invalid joint range")
    return np.asarray(qids,dtype=int),limits


def read_pose(data,body_id):
    return (data.xpos[body_id].copy(),
            data.xmat[body_id].reshape(3,3).copy())


def rotation_error(target_R,actual_R):
    return Rotation.from_matrix(target_R @ actual_R.T).as_rotvec()


def pose_error_mm_deg(actual_position,actual_rotation,
                      target_position,target_rotation):
    return (float(np.linalg.norm(actual_position-target_position)*1000),
            float(np.linalg.norm(rotation_error(target_rotation,actual_rotation))*
                  180/np.pi))


def compute_ik(model,data,body_id,qids,limits,position,rotation,seed,
               max_evals=85):
    """Bounded whole-pose IK using actual MuJoCo FK; no direct body teleport."""
    import mujoco
    pos=np.asarray(position,float).reshape(3)
    R=np.asarray(rotation,float).reshape(3,3)
    lo=limits[:,0]+1e-4
    hi=limits[:,1]-1e-4
    start=np.clip(np.asarray(seed,float).reshape(6),lo+1e-6,hi-1e-6)
    def residual(q):
        data.qpos[qids]=q
        mujoco.mj_forward(model,data)
        p,r=read_pose(data,body_id)
        return np.r_[p-pos,rotation_error(R,r)*.30]
    result=least_squares(residual,start,bounds=(lo,hi),
                         max_nfev=max_evals,xtol=1e-9,
                         ftol=1e-9,gtol=1e-9)
    data.qpos[qids]=result.x
    mujoco.mj_forward(model,data)
    p,r=read_pose(data,body_id)
    pos_mm,deg=pose_error_mm_deg(p,r,pos,R)
    return dict(q=result.x.copy(),position_error_mm=pos_mm,
                rotation_error_deg=deg,solved=bool(
                    result.success and pos_mm<=3.0 and deg<=2.0),
                iterations=int(result.nfev))


def first_configuration(model,data,body_id,qids,limits):
    """Find feasible start near scene's demonstration pose, with honest fallback."""
    import mujoco
    target=np.array([.67,-.14,.60],float)
    target_R=np.eye(3)
    guesses=[
        HOME_HINT,
        [-2.7,-.9,1.0,-1.3,1.3,0],
        [2.7,-.9,1.0,-1.3,-1.3,0],
        [-2.4,-1.5,1.7,-1.3,1.4,0],
        [2.4,-1.5,1.7,-1.3,-1.4,0],
        [-2.6,.4,-1.0,-1.2,1.0,0],
        [2.6,.4,-1.0,-1.2,-1.0,0],
        [-1.6,-.6,1.4,-.8,1.0,.4],
        [1.6,-.6,1.4,-.8,-1.0,-.4],
    ]
    results=[]
    for hint in guesses:
        x=compute_ik(model,data,body_id,qids,limits,
                     target,target_R,np.array(hint,float),max_evals=110)
        results.append(x)
    successes=[x for x in results if x["solved"]]
    if successes:
        # Multiple 6D IK solutions can reach the SAME pitcher pose, yet fold
        # the elbow/wrist into radically different configurations. Check a
        # coarse full-path rehearsal before choosing the start configuration.
        # We evaluate all 25 kinematic frames WITHOUT rendering or changing
        # the desired target to force a superficially successful result.
        half_width=(limits[:,1]-limits[:,0])/2
        mid=(limits[:,1]+limits[:,0])/2
        def clearance(x):
            normalized_margin=1-np.abs((x['q']-mid)/half_width)
            return float(np.min(normalized_margin))
        ranked=[]
        for idx,candidate in enumerate(successes):
            q=candidate['q'].copy()
            n_failed=0
            contact_frames=0
            joint_jump=0.
            worst_error=0.
            for j in range(25):
                desired=motion_sample(j/24,target,target_R)
                x=compute_ik(model,data,body_id,qids,limits,
                             desired.position_m,desired.rotation,q,
                             max_evals=90)
                joint_jump=max(joint_jump,float(np.linalg.norm(x['q']-q)))
                q=x['q']
                if not x['solved']:
                    n_failed+=1
                worst_error=max(worst_error,x['position_error_mm'])
                external,internal=_contacts(model,data)
                if external or internal:
                    contact_frames+=1
            metric=(n_failed,contact_frames,round(worst_error,3),
                    round(joint_jump,3),-clearance(candidate))
            ranked.append((metric,candidate))
            print(f'Initial IK candidate {idx+1}/{len(successes)} '
                  f'route_score={metric}',flush=True)
        metric,best=min(ranked,key=lambda x:x[0])
        print('Selected collision-screened FR5 start:',metric,flush=True)
    else:
        best=min(results,key=lambda r:r["position_error_mm"]+
                 8*r["rotation_error_deg"])
    data.qpos[qids]=best['q']
    mujoco.mj_forward(model,data)
    actual_pos,actual_rot=read_pose(data,body_id)
    return best,actual_pos,actual_rot


def _contacts(model,data):
    """Report physical collision contacts; do not pretend checking avoidance."""
    external=[]
    internal=[]
    for c in data.contact:
        g1=model.geom(c.geom1)
        g2=model.geom(c.geom2)
        n1=g1.name or ''
        n2=g2.name or ''
        # Only include contact pairs with at least one robot physical collision geom.
        # Main FR5 collision proxies AND PGI gripper/finger proxies are robot.
        def robot_part(name):
            return name.startswith(('collision/','gripper_','finger'))
        robot1=robot_part(n1)
        robot2=robot_part(n2)
        if not robot1 and not robot2:
            continue
        # Supporting base-on-pedestal contact is intentional. Report all
        # other actual arm/environment and arm/self contacts.
        support={n1,n2}=={'collision/base_link','rb_robot_pedestal'}
        if support:
            continue
        if robot1 and robot2:
            internal.append([n1,n2])
        else:
            external.append([n1,n2])
    return external,internal


def _montage(image_array, frame, time_s, phase, position_err, angle_err,
             collisions):
    image=Image.fromarray(image_array).resize((1000,750),Image.Resampling.BILINEAR)
    canvas=Image.new('RGB',(1000,805),(240,243,247))
    canvas.paste(image,(0,55))
    d=ImageDraw.Draw(canvas)
    d.text((14,8),f'FR5 actual joint FK  |  frame {frame+1}  |  t={time_s:.2f}s  |  {phase}',
           fill=(20,29,42))
    d.text((14,28),f'Pitcher pose error: {position_err:.1f} mm / {angle_err:.2f} deg   '
                    f'Robot external contacts: {collisions}',fill=(20,29,42))
    return canvas


def _plot(rows,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ts=[r['time_s'] for r in rows]
    fig,axs=plt.subplots(2,1,figsize=(9,7),sharex=True)
    axs[0].plot(ts,[r['pitcher_pose_error_mm'] for r in rows],label='Position error (mm)')
    axs[0].axhline(3,color='gray',ls='--',label='3 mm IK threshold')
    axs[0].set_ylabel('Pitcher center error (mm)')
    axs[0].legend()
    axs[1].plot(ts,[r['pitcher_rotation_error_deg'] for r in rows],label='Angle error (deg)')
    axs[1].axhline(2,color='gray',ls='--',label='2 degree IK threshold')
    axs[1].set_ylabel('Pitcher orientation error (deg)')
    axs[1].set_xlabel('Synthetic motion time (s)')
    axs[1].legend()
    for a in axs:a.grid(alpha=.2)
    fig.suptitle('Desired pose versus actual FR5 six-joint kinematics')
    fig.tight_layout()
    fig.savefig(out/'ik_error.png',dpi=140)
    plt.close(fig)


def run(args):
    import mujoco
    start_time=time.monotonic()
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    scene_path=prepare_grasp_scene(Path(args.vendor_dir).resolve())
    model=mujoco.MjModel.from_xml_path(str(scene_path))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    body_id=model.body('rb_pitcher').id
    wrist_id=model.body('wrist3_link').id
    # Critical: verify the pitcher is a descendant of the six-joint wrist.
    parent=int(model.body_parentid[body_id])
    if parent!=wrist_id:
        raise RuntimeError("Pitcher is not rigidly attached to FR5 wrist")
    qids,limits=robot_qpos_addresses(model)
    initial,anchor_pos,anchor_rot=first_configuration(
        model,data,body_id,qids,limits)
    print('IK starting pose:',initial, 'anchor xyz=',anchor_pos,flush=True)
    # If we cannot even position the pitcher near the scene, fail explicitly,
    # instead of showing an unrelated remote gesture as success.
    if np.linalg.norm(anchor_pos - np.array([.67,-.14,.60]))>.065:
        raise RuntimeError('Initial robot pitcher too far from testbench; '
                           f'actual={anchor_pos.tolist()}')
    width=args.width
    height=args.height
    renderer=mujoco.Renderer(model,width=width,height=height)
    previous_q=initial["q"]
    previews=[]
    rows=[]
    qlo,qhi=limits[:,0],limits[:,1]
    try:
        for i,t,progress in frame_times(args.frames,args.fps):
            desired=motion_sample(progress,anchor_pos,anchor_rot)
            ik=compute_ik(model,data,body_id,qids,limits,
                          desired.position_m,desired.rotation,previous_q)
            previous_q=ik["q"].copy()
            actual_pos,actual_rot=read_pose(data,body_id)
            wrist_pos,wrist_rot=read_pose(data,wrist_id)
            wanted_tip=world_spout(desired.position_m,desired.rotation,SPOUT_TIP_LOCAL_M)
            actual_tip=world_spout(actual_pos,actual_rot,SPOUT_TIP_LOCAL_M)
            spout_mm=float(np.linalg.norm(wanted_tip-actual_tip)*1000)
            outside,inside=_contacts(model,data)
            # Frames are rendered with *the robot joint configuration* that FK
            # actually produced. The desired pitcher pose is NOT set on the body.
            rgb,depth=render_pair(renderer,data,'rb_robot_overview')
            image=_montage(rgb,i,t,desired.phase,
                           ik['position_error_mm'],ik['rotation_error_deg'],
                           len(outside))
            previews.append(image)
            if i==0 or outside or inside:
                print("Frame contacts:",i,outside,inside,flush=True)
            r=dict(frame=i,time_s=t,phase=desired.phase,
                   desired_pitcher_xyz_m=desired.position_m,
                   measured_pitcher_xyz_m=actual_pos,
                   desired_pitcher_rotation_xyzw=Rotation.from_matrix(
                       desired.rotation).as_quat(),
                   actual_pitcher_rotation_xyzw=Rotation.from_matrix(
                       actual_rot).as_quat(),
                   desired_spout_xyz_m=wanted_tip,
                   actual_spout_xyz_m=actual_tip,
                   joint_positions_rad=ik['q'],
                   wrist_xyz_m=wrist_pos,
                   pitcher_pose_error_mm=ik['position_error_mm'],
                   pitcher_rotation_error_deg=ik['rotation_error_deg'],
                   spout_error_mm=spout_mm,ik_solved=ik['solved'],
                   joint_limits_ok=bool(np.all(
                       ik['q']>=qlo) and np.all(ik['q']<=qhi)),
                   environment_contacts=outside,
                   self_contacts=inside)
            rows.append(to_json(r))
            print(f'Frame {i+1:02}/{args.frames}: {desired.phase:<10} '
                  f'IK={ik["solved"]} pose={ik["position_error_mm"]:.2f}mm/'
                  f'{ik["rotation_error_deg"]:.2f}deg spout={spout_mm:.2f}mm '
                  f'collision contacts ext={len(outside)} self={len(inside)}',
                  flush=True)
    finally:
        renderer.close()

    if previews:
        frames=[im.convert('P',palette=Image.Palette.ADAPTIVE,colors=96)
                for im in previews]
        frames[0].save(out/'fr5_replay.gif',save_all=True,
                       append_images=frames[1:],
                       duration=round(1000/args.fps),loop=0,optimize=False)
        for frame in (0,len(previews)//2,len(previews)-1):
            previews[frame].save(out/f'frame_{frame:04d}.png')
    _plot(rows,out)
    (out/'trajectory.json').write_text(json.dumps(rows,indent=2))
    fields=('frame','time_s','phase','pitcher_pose_error_mm',
            'pitcher_rotation_error_deg','spout_error_mm',
            'ik_solved','joint_limits_ok')
    with (out/'ik_metrics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k:row[k] for k in fields})
    all_ik=all(r['ik_solved'] for r in rows)
    all_within=all(r['joint_limits_ok'] for r in rows)
    collisions=sum(len(r['environment_contacts']) for r in rows)
    self_contacts=sum(len(r['self_contacts']) for r in rows)
    report=dict(
       status='PASS' if all_ik and all_within and collisions==0 and self_contacts==0 else 'FAIL',
       interpretation='True FR5 six hinge qpos are replayed kinematically via bounded IK; rigid visual pitcher mount; no motors or grasp physics',
       frames=args.frames,fps=args.fps,
       initial_position_error_mm=initial['position_error_mm'],
       initial_orientation_error_deg=initial['rotation_error_deg'],
       successful_ik_frames=sum(r['ik_solved'] for r in rows),
       joint_limit_failures=sum(not r['joint_limits_ok'] for r in rows),
       max_pitcher_position_error_mm=max(r['pitcher_pose_error_mm'] for r in rows),
       max_pitcher_orientation_error_deg=max(r['pitcher_rotation_error_deg'] for r in rows),
       max_spout_error_mm=max(r['spout_error_mm'] for r in rows),
       robot_environment_contacts=collisions,
       robot_self_contacts=self_contacts,
       collisions_checked='MuJoCo contact diagnostics on physical FR5 collision proxy geoms. Virtual pitcher collision geoms disabled.',
       safety_warning='No motor control, trajectory dynamics, physical grip, force constraints, velocity/acceleration limits, or realistic real-world safety proof.',
       camera_vision_used=False,
       runtime_s=round(time.monotonic()-start_time,2))
    (out/'summary.json').write_text(json.dumps(report,indent=2))
    print('RESULT:',json.dumps(report,indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--vendor-dir',default='vendor')
    p.add_argument('--output',default='runs/fr5_replay')
    p.add_argument('--frames',type=int,default=25)
    p.add_argument('--fps',type=float,default=8.)
    p.add_argument('--width',type=int,default=960)
    p.add_argument('--height',type=int,default=720)
    report=run(p.parse_args())
    # OSMesa/PyOpenGL teardown can crash only AFTER all generated artifacts.
    import sys
    sys.stdout.flush()
    sys.stderr.flush()
    if os.environ.get('MUJOCO_GL','').lower()=='osmesa':
        os._exit(0 if report['status']=='PASS' else 2)
    if report['status']!='PASS':
        raise SystemExit(2)


if __name__=='__main__':
    main()
