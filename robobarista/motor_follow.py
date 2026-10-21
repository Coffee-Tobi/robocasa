"""MuJoCo torque-actuated FR5 tracking of previously camera-measured IK joints.

A SECOND, deliberately separate demonstration: reference joint q(t) is read
from the already completed camera->IK replay JSON. The six real FR5 motor
actuators apply finite bounded TORQUES and mujoco.mj_step advances dynamics.

Unmodelled gripper force/pitcher mass and real hardware are NOT simulated.
Each dynamic arm/contact and carried-pitcher clearance violation marks FAIL.
This is a controller concept, NOT real-robot-safe torque/velocity tuning.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess

import mujoco
import numpy as np
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation

from .camera_joint_replay import (
    _add_held_pitcher_robot_variant,_set_replay_paint,_wrist_pitcher_geoms
)
from .cell_demo import _set_free_pose
from .robot_replay import robot_qpos_addresses,_contacts
from .workcell_constraints import payload_check,TRAVEL_CLEARANCE_M
from .demo import render_pair


CONTROL_DT=0.01
T_AFTER_END=0.25
START_HOLD=0.45
# Nominal trial gains, for the virtual FR5 model ONLY.
POSITION_KP=np.array([180,180,150,65,48,42],float)
VELOCITY_KD=np.array([28,28,22,8,6,5],float)


def _target_at(t,timestamp,q):
    """C1-ish smootherstep between actual camera-sample joint targets."""
    time=np.asarray(timestamp,float)
    if t<=time[0]:return q[0]
    if t>=time[-1]:return q[-1]
    i=np.clip(np.searchsorted(time,t,side='right')-1,0,len(q)-2)
    dt=time[i+1]-time[i]
    u=float((t-time[i])/dt)
    u2=u*u
    ease=u2*u*(10+u*(-15+6*u))  # fifth-order minimum-jerk segment
    return (1-ease)*q[i]+ease*q[i+1]


def _actuators(model):
    from .robot_replay import JOINTS
    acts=np.array([int(model.actuator(name).id) for name in JOINTS])
    ranges=np.asarray(model.actuator_ctrlrange[acts],float)
    assert len(acts)==6
    return acts,ranges


def _setup_frame(model,data,initial,qids):
    data.qpos[qids]=initial
    data.qvel[:]=0.
    _set_free_pose(model,data,'rb_pitcher_free',[-10,-10,-10],0)
    _set_free_pose(model,data,'rb_plastic_cup_free',[-10,-10,-10],0)
    mujoco.mj_forward(model,data)


def _gravity_ff(model,current_data,inverse_data,dofs):
    """Inverse dynamics with qdot=qddot=0 → static gravity compensation."""
    inverse_data.qpos[:]=current_data.qpos
    inverse_data.qvel[:]=0
    inverse_data.qacc[:]=0
    mujoco.mj_inverse(model,inverse_data)
    return np.asarray(inverse_data.qfrc_inverse[dofs]).copy()


def _frame(rgb,frame_number,t,p_error,angle_error,torque_saturation):
    im=Image.fromarray(rgb).resize((1100,825),Image.Resampling.BILINEAR)
    canvas=Image.new('RGB',(1100,900),(27,35,43))
    canvas.paste(im,(0,55))
    d=ImageDraw.Draw(canvas)
    d.text((18,10),f'FR5 MuJoCo TORQUE MOTOR simulation | t = {t:.2f} s | frame {frame_number}',
            fill='white')
    d.text((18,31),'Controlled with six motor torques, mj_step gravity + PID',
           fill=(166,219,230))
    d.text((16,865),f'Position lag {p_error:.1f} mm  |  orientation lag {angle_error:.2f} deg',
           fill='white')
    d.text((16,884),f'Actuator saturated: {torque_saturation}',
           fill=(244,183,123))
    return canvas


def run(args):
    import time
    root=Path(args.input).resolve()
    output=Path(args.output).resolve()
    output.mkdir(parents=True,exist_ok=True)
    qfile=root/'robot_replay.json'
    if not qfile.exists():
        raise FileNotFoundError(f'First run camera replay to generate {qfile}')
    camera=json.loads(qfile.read_text())
    if len(camera)<3:
        raise ValueError('Need 3+ camera-derived IK frames')
    t_ref=np.array([row['time_s'] for row in camera],float)
    q_ref=np.array([row['q_rad'] for row in camera],float)
    if np.any(np.diff(t_ref)<=0):
        raise ValueError('Non-monotonic vision time stamps')

    scene=_add_held_pitcher_robot_variant(Path(args.vendor_dir).resolve())
    model=mujoco.MjModel.from_xml_path(str(scene))
    model.opt.timestep=.001
    data=mujoco.MjData(model)
    qids,limits=robot_qpos_addresses(model)
    dofs=np.array([int(model.jnt_dofadr[model.joint(name).id])
            for name in ('shoulder_pan','shoulder_lift','elbow',
                         'wrist_1','wrist_2','wrist_3')],dtype=int)
    act_ids,ctrlrange=_actuators(model)
    _setup_frame(model,data,q_ref[0],qids)
    inverse=mujoco.MjData(model)
    sat_count=0
    first_failure=None
    frames=[]
    logs=[]
    last_control=-1e9
    target_q=q_ref[0]
    errors=[]
    renderer=mujoco.Renderer(model,width=args.width,height=args.height)
    t_stop=START_HOLD+args.time_scale*t_ref[-1]+T_AFTER_END
    next_render=0.0
    elapsed=0.0
    try:
        while elapsed < t_stop-1e-10:
            # Camera measurement timestamps are unchanged. A short start hold
            # allows torque motors to establish gravity-compensated equilibrium.
            source_t=(elapsed-START_HOLD)/args.time_scale
            target_q=_target_at(source_t,t_ref,q_ref)
            if elapsed-last_control>=CONTROL_DT-1e-7:
                now=np.asarray(data.qpos[qids],float)
                speed=np.asarray(data.qvel[dofs],float)
                gravity=_gravity_ff(model,data,inverse,dofs)
                raw=gravity+POSITION_KP*(target_q-now)-VELOCITY_KD*speed
                bounded=np.clip(raw,ctrlrange[:,0],ctrlrange[:,1])
                sat_count+=int(np.any(np.abs(raw-bounded)>1e-7))
                data.ctrl[act_ids]=bounded
                last_control=elapsed
            mujoco.mj_step(model,data)
            elapsed+=model.opt.timestep
            robot_contacts=_contacts(model,data)
            if (robot_contacts[0] or robot_contacts[1]) and first_failure is None:
                first_failure={'time_s':round(elapsed,4),
                               'reason':'physical_robot_contact',
                               'contacts':robot_contacts}
            body_id=model.body('rb_cell_tcp').id
            current_pos=data.xpos[body_id].copy()
            current_R=data.xmat[body_id].reshape(3,3).copy()
            payload=payload_check(current_pos,current_R,tool='pitcher',
                                  clearance_m=TRAVEL_CLEARANCE_M)
            if payload and first_failure is None:
                first_failure={'time_s':round(elapsed,4),
                               'reason':'pitcher_volume_near_machine',
                               'violations':payload}
            qactual=np.asarray(data.qpos[qids],float)
            if (np.any(qactual<limits[:,0]-1e-5)
                or np.any(qactual>limits[:,1]+1e-5)) and first_failure is None:
                first_failure={'time_s':round(elapsed,4),
                               'reason':'joint_limit_violation'}
            if not np.all(np.isfinite(data.qpos)):
                first_failure={'time_s':round(elapsed,4),
                               'reason':'non_finite_physics'}
                break
            if elapsed+1e-6 >= next_render:
                # Desired pose FK evaluated in a SEPARATE data record, never
                # overwrite the simulated motor's actual joint trajectory.
                ref_data=mujoco.MjData(model)
                ref_data.qpos[:]=data.qpos
                ref_data.qpos[qids]=target_q
                mujoco.mj_forward(model,ref_data)
                wanted_pos=ref_data.xpos[body_id].copy()
                wanted_R=ref_data.xmat[body_id].reshape(3,3).copy()
                pos_err_mm=float(np.linalg.norm(wanted_pos-current_pos)*1000)
                angle_err_deg=float(np.rad2deg(Rotation.from_matrix(
                    wanted_R@current_R.T).magnitude()))
                errors.append(pos_err_mm)
                logs.append(dict(time_s=round(elapsed,4),
                    original_camera_time_s=round(max(0.,source_t),4),
                    joint_velocity_rad_s=data.qvel[dofs].tolist(),
                    desired_q_rad=target_q.tolist(),
                    actual_q_rad=qactual.tolist(),
                    lag_position_mm=pos_err_mm,
                    lag_rotation_deg=angle_err_deg,
                    motor_torque_nm=data.ctrl[act_ids].tolist(),
                    torque_saturated=bool(np.any(np.abs(data.ctrl[act_ids])>=
                                      np.max(np.abs(ctrlrange),axis=1)-1e-6))))
                if len(frames)<args.max_frames:
                    rgb,_=render_pair(renderer,data,'cell_overview')
                    frames.append(_frame(rgb,len(frames),elapsed,
                                        pos_err_mm,angle_err_deg,
                                        logs[-1]['torque_saturated']))
                next_render+=1/args.fps
            if first_failure:
                print('MOTOR RUN STOPPED:',first_failure,flush=True)
                break
    finally:
        renderer.close()

    (output/'motor_tracking.json').write_text(
         json.dumps(logs,indent=2),encoding='utf-8')
    mp4=False
    if frames and shutil.which('ffmpeg'):
        folder=output/'frames'
        folder.mkdir(exist_ok=True)
        for i,f in enumerate(frames):
            f.save(folder/f'img_{i:04d}.png')
        cmd=['ffmpeg','-y','-loglevel','error','-framerate',str(args.fps),
             '-i',str(folder/'img_%04d.png'),'-c:v','libx264',
             '-pix_fmt','yuv420p',str(output/'motor_tracking.mp4')]
        subprocess.run(cmd,check=True)
        mp4=(output/'motor_tracking.mp4').stat().st_size>1000
    max_err=max(errors,default=None)
    report={
        'status':'PASS' if first_failure is None and mp4 else 'FAIL',
        'motor_dynamics_called_mj_step':True,
        'six_fr5_real_torque_actuators_used':True,
        'controller':'gravity compensation via mj_inverse + joint PD + torque clipping',
        'camera_trajectory_loaded_from':str(qfile),
        'frames_rendered':len(frames),
        'source_camera_frames':len(camera),
        'time_scale':args.time_scale,
        'actual_simulated_duration_s':round(elapsed,4),
        'max_joint_velocity_rad_s':max(
            [max(abs(v) for v in rec['joint_velocity_rad_s'])
             for rec in logs],default=None),
        'max_motor_tracking_position_lag_mm':max_err,
        'mean_motor_tracking_position_lag_mm':float(np.mean(errors)) if errors else None,
        'torque_saturation_control_intervals':sat_count,
        'first_safety_violation':first_failure,
        'walltime_not_real_robot_time':'MuJoCo integrated simulated time only',
        'cautions':'Non-validated PD gains/motor model. No velocity/acceleration/jerk certification, real payload force, gripping or water/milk physics.'
    }
    (output/'summary.json').write_text(json.dumps(report,indent=2))
    print('MOTOR FOLLOW:',json.dumps(report,indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--input',default='runs/allowed_camera_replay')
    p.add_argument('--output',default='runs/motor_follow')
    p.add_argument('--vendor-dir',default='vendor')
    p.add_argument('--width',type=int,default=960)
    p.add_argument('--height',type=int,default=720)
    p.add_argument('--fps',type=float,default=7)
    p.add_argument('--max-frames',type=int,default=64)
    p.add_argument('--time-scale',type=float,default=2.5,
                   help='Stretch camera trajectory in time; 2.5 means slower controlled movement')
    args=p.parse_args()
    if args.time_scale<1:
        raise ValueError('time-scale must be >=1 for this conservative motor experiment')
    r=run(args)
    import sys
    sys.stdout.flush();sys.stderr.flush()
    if os.environ.get('MUJOCO_GL','').lower()=='osmesa':
        os._exit(0 if r['status']=='PASS' else 2)
    if r['status']!='PASS':
        raise SystemExit(2)


if __name__=='__main__':
    main()
