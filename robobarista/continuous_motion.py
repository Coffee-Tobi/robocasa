"""Offline continuous SE(3) AprilTag trajectory + local jerk-limited Ruckig OTG.

Only reconstructed RGB-D AprilTag data are fitting inputs. Ground-truth JSON
is NEVER opened by this module. No visual servo / online camera feedback.

Ruckig Community multi-waypoint calculation uses a remote/cloud service; we
DO NOT use that API. We instead solve a chain of fully local, six-DOF
state-to-state segments with NONZERO velocity/acceleration at interior knots.
All resulting joint interpolation states are screened with the same MuJoCo
robot link and full carried pitcher collision/no-go tests as other demos.

An individual segment can extend time to respect jerk/acceleration/velocity
limits. This is a time-retiming planner, not exact original demonstration
timing. We report the stretching explicitly.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path

import numpy as np
import mujoco
from scipy.interpolate import UnivariateSpline, CubicSpline
from scipy.spatial.transform import Rotation, RotationSpline

from .camera_joint_replay import _add_held_pitcher_robot_variant, _validate_q
from .cell_demo import _set_free_pose
from .robot_replay import compute_ik, robot_qpos_addresses
from .workcell_constraints import TRAVEL_CLEARANCE_M
from .demo import to_json

# Conservative *virtual demonstration* limits, NOT FR5 manufacturer's ratings.
MAX_VEL=np.array([.55,.55,.55,.75,.75,.75])        # rad/s
MAX_ACC=np.array([1.0,1.0,1.0,1.3,1.3,1.3])       # rad/s²
MAX_JERK=np.array([4.0,4.0,4.0,5.0,5.0,5.0])     # rad/s³
SAMPLE_DT=.02                                      # 50 Hz checks/output


def _camera_samples(records):
    if len(records)<8 or any(not rec.get('valid') for rec in records):
        raise ValueError('Need >=8 valid camera-only 6D measurements. No GT fill.')
    t=np.array([float(rec['time_s']) for rec in records])
    p=np.array([rec['position_m'] for rec in records],float)
    r=Rotation.from_quat([rec['rotation_xyzw'] for rec in records])
    if not (np.all(np.diff(t)>0) and np.isfinite(p).all()):
        raise ValueError('Invalid monotone camera times or positions')
    return t,p,r


def fit_continuous_6d(records,position_smoothing_mm=.35):
    """3 translation smoothing splines + SO(3) rotation spline.

    Position splines are C2 smooth. SciPy's RotationSpline is continuous
    in rotation/angular velocity/acceleration. RotationSpline interpolates
    measured orientation; it does not denoise image rotation noise.
    """
    t,p,r=_camera_samples(records)
    sigma=float(position_smoothing_mm)*.001
    if sigma<0:raise ValueError('Negative smoothness')
    pos=[UnivariateSpline(t,p[:,i],k=3,s=len(t)*sigma**2)
         for i in range(3)]
    orientation=RotationSpline(t,r)
    def evaluate(times):
        times=np.atleast_1d(np.asarray(times,float))
        if (np.any(times<t[0]-1e-10) or np.any(times>t[-1]+1e-10)):
            raise ValueError('No extrapolation outside observed camera times')
        times=np.clip(times,t[0],t[-1])
        xyz=np.column_stack([f(times) for f in pos])
        velocity=np.column_stack([f.derivative(1)(times) for f in pos])
        acceleration=np.column_stack([f.derivative(2)(times) for f in pos])
        ori=orientation(times)
        angular_v=orientation(times,1)  # radians/s, world or body convention per SciPy
        angular_a=orientation(times,2)
        return dict(time_s=times, position_m=xyz,
                    translation_velocity_m_s=velocity,
                    translation_acceleration_m_s2=acceleration,
                    rotation_xyzw=ori.as_quat(),
                    angular_velocity_rad_s=angular_v,
                    angular_acceleration_rad_s2=angular_a)
    return evaluate


def _robot_ik_at_knots(model,data,times,positions,quats,initial_q=None):
    qids,limits=robot_qpos_addresses(model)
    body=model.body('rb_cell_tcp').id
    seed=(np.asarray(initial_q,float).copy()
          if initial_q is not None
          else np.asarray(data.qpos[qids],float).copy())
    knots=[]
    for i,(t,p,quat) in enumerate(zip(times,positions,quats)):
        R=Rotation.from_quat(quat).as_matrix()
        solved=compute_ik(model,data,body,qids,limits,p,R,seed,
                          max_evals=120)
        if not solved['solved']:
            raise RuntimeError(f'Camera spline IK invalid at t={t:.3f}s: '
                               f'{solved["position_error_mm"]:.2f} mm, '
                               f'{solved["rotation_error_deg"]:.2f} deg')
        valid,reason=_validate_q(model,data,qids,limits,solved['q'])
        if not valid:
            raise RuntimeError(f'Camera spline IK unsafe at t={t:.3f}s: {reason}')
        knots.append(solved['q'].copy())
        seed=solved['q'].copy()
    return np.asarray(knots),qids,limits


def _derive_joint_boundary_states(t,q,velocity_scale):
    """Pass same nonzero state into both adjacent Ruckig sections."""
    # Interpolate joint coordinates globally to derive continuous knot tangents.
    spline=CubicSpline(t,q,axis=0,bc_type='natural')
    v=np.asarray(spline(t,1),float)/velocity_scale
    a=np.asarray(spline(t,2),float)/(velocity_scale**2)
    v[0]=v[-1]=0
    a[0]=a[-1]=0
    # Do not silently clip unsafe boundary demands.
    if np.any(np.abs(v)>MAX_VEL*.85) or np.any(np.abs(a)>MAX_ACC*.65):
        return None,None
    return v,a


def _ruckig_sections(times,q,velocity_scale=3.0,dt=SAMPLE_DT):
    """Plan offline 6-axis locally, preserving nonzero internal derivatives.

    Segment durations are allowed to exceed original times to respect
    velocity, acceleration and jerk, so exact original timing is NOT claimed.
    """
    from ruckig import Ruckig,InputParameter,Trajectory,Result
    velocities,accelerations=_derive_joint_boundary_states(
        times,q,velocity_scale)
    if velocities is None:
        raise RuntimeError('Ruckig boundary velocity/acceleration exceeds limits')
    otg=Ruckig(6)
    samples=[]
    info=[]
    t_start=0.
    for i in range(len(times)-1):
        inp=InputParameter(6)
        inp.current_position=q[i].tolist()
        inp.current_velocity=velocities[i].tolist()
        inp.current_acceleration=accelerations[i].tolist()
        inp.target_position=q[i+1].tolist()
        inp.target_velocity=velocities[i+1].tolist()
        inp.target_acceleration=accelerations[i+1].tolist()
        inp.max_velocity=MAX_VEL.tolist()
        inp.max_acceleration=MAX_ACC.tolist()
        inp.max_jerk=MAX_JERK.tolist()
        nominal=(times[i+1]-times[i])*velocity_scale
        inp.minimum_duration=max(.06,float(nominal))
        traj=Trajectory(6)
        res=otg.calculate(inp,traj)
        if res != Result.Working:
            raise RuntimeError(f'Ruckig segment {i} failed: {res}')
        duration=float(traj.duration)
        n=max(1,int(np.ceil(duration/dt)))
        for j in range(n+1):
            if i>0 and j==0:continue
            local=min(j*duration/n,duration)
            pos,vel,acc=traj.at_time(local)
            samples.append(dict(time_s=t_start+local,
                q_rad=list(pos),velocity_rad_s=list(vel),
                acceleration_rad_s2=list(acc),section=i))
        info.append(dict(section=i,
                         camera_start_s=float(times[i]),
                         camera_end_s=float(times[i+1]),
                         duration_s=duration,
                         nominal_duration_s=float(nominal),
                         timing_stretch_s=float(duration-nominal),
                         start_speed_norm=float(np.linalg.norm(velocities[i])),
                         end_speed_norm=float(np.linalg.norm(velocities[i+1]))))
        t_start+=duration
    return samples,info


def _evaluate_safety(model,data,qids,limits,rows):
    """Recheck actual Ruckig samples and fine joint-space interpolants."""
    count=0
    for a,b in zip(rows,rows[1:]):
        qa=np.asarray(a['q_rad'],float)
        qb=np.asarray(b['q_rad'],float)
        sub=max(1,int(np.ceil(np.max(np.abs(qb-qa))/.01)))
        for i in range(sub+1):
            x=qa+(qb-qa)*(i/sub)
            safe,why=_validate_q(model,data,qids,limits,x)
            count+=1
            if not safe:
                raise RuntimeError(f'Ruckig swept-path unsafe '
                                   f'{a["time_s"]:.3f}s: {why}')
    return count


def _metrics(records,model,data,rows,source_times,source_poses,source_quats,
             fitted):
    from .robot_replay import _read_body
    qids,_=robot_qpos_addresses(model)
    body=model.body('rb_cell_tcp').id
    errors=[]
    for i,row in enumerate(rows):
        if i%max(1,len(rows)//100)==0 or i==len(rows)-1:
            data.qpos[qids]=row['q_rad']
            mujoco.mj_forward(model,data)
            p,_=_read_body(data,body)
            errors.append(float(np.linalg.norm(p-source_poses[
                np.argmin(np.abs(source_times -
                    (row['time_s']/max(rows[-1]['time_s'],
                        1e-3)*source_times[-1])))])*1000))
    return dict(max_frame_relative_path_error_mm=max(errors,default=None))


def run(args):
    path=Path(args.input)/'camera_measurements.json'
    if not path.is_file():
        raise FileNotFoundError(f'Missing camera-only measurements: {path}')
    records=json.loads(path.read_text())
    t,p,r=_camera_samples(records)
    evaluation=fit_continuous_6d(records,args.smooth_mm)
    # The full 6D curve is sampled more densely than the 15fps camera stream.
    dense_times=np.linspace(t[0],t[-1],
                            1+int(np.ceil((t[-1]-t[0])*args.spline_fps)))
    curve=evaluation(dense_times)
    # For IK, select every 3rd measured point plus endpoints; original camera
    # sample times are retained and represented accurately along splines.
    stride=max(1,int(args.knot_stride))
    idx=list(range(0,len(t),stride))
    if idx[-1]!=len(t)-1:idx.append(len(t)-1)
    key_time=t[idx]
    key_curve=evaluation(key_time)

    model=mujoco.MjModel.from_xml_path(str(_add_held_pitcher_robot_variant(
        Path(args.vendor_dir).resolve())))
    data=mujoco.MjData(model)
    _set_free_pose(model,data,'rb_pitcher_free',[-10,-10,-10],0)
    _set_free_pose(model,data,'rb_plastic_cup_free',[-10,-10,-10],0)
    mujoco.mj_forward(model,data)
    # Start IK near the known camera-derived solution, not the original
    # (hidden) random-demonstrator joints or original simulation truth.
    prior_ik_file=Path(args.input)/'robot_replay.json'
    init_q=None
    if prior_ik_file.is_file():
        previous_replay=json.loads(prior_ik_file.read_text())
        if previous_replay:
            init_q=np.asarray(previous_replay[0]['q_rad'],float)
    q,qids,limits=_robot_ik_at_knots(
        model,data,key_time,key_curve['position_m'],
        key_curve['rotation_xyzw'],initial_q=init_q)
    last_problem=None
    planned=None
    for factor in (args.initial_slowdown,4,6,9,13):
        if factor<args.initial_slowdown:continue
        try:
            candidate,info=_ruckig_sections(key_time,q,factor,args.dt)
            n_checks=_evaluate_safety(model,data,qids,limits,candidate)
            planned=(candidate,info,n_checks,factor)
            break
        except RuntimeError as exc:
            last_problem=str(exc)
            print('Ruckig attempt slower:',factor,'issue:',exc,flush=True)
    if planned is None:
        raise RuntimeError('No collision-free continuous jerk-limited path '
                           'under tested retiming factors. '+str(last_problem))
    trajectory,sections,safety_n,factor=planned
    # independent source camera spline data only; no GT file read.
    out=Path(args.output)
    out.mkdir(parents=True,exist_ok=True)
    def serialize_curve(e):
        return [{key:(float(value[i]) if value.ndim==1 else
                     value[i].tolist()) for key,value in e.items()}
                for i in range(len(e['time_s']))]
    (out/'camera_continuous_6d.json').write_text(
        json.dumps(serialize_curve(curve),indent=1),encoding='utf-8')
    (out/'ruckig_joint_trajectory.json').write_text(
        json.dumps(to_json(trajectory),indent=1),encoding='utf-8')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,1,figsize=(10,8))
    xyz=np.asarray(curve['position_m'],float)*1000
    for axis in range(3):
        axs[0].plot(dense_times,xyz[:,axis],
                    label=['X','Y','Z'][axis]+' fitted [mm]')
        axs[0].scatter(t,p[:,axis]*1000,s=8,alpha=.4)
    axs[0].set(title='15 FPS AprilTag samples -> continuous 6D trajectory',
               ylabel='Position [mm]')
    axs[0].legend()
    vel=np.array([x['velocity_rad_s'] for x in trajectory])
    axs[1].plot([x['time_s'] for x in trajectory],vel)
    axs[1].set(title='Ruckig joint-speed profiles (nonzero waypoint speed)',
               xlabel='Retimed motion [s]',ylabel='Joint speed [rad/s]')
    for ax in axs:ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(out/'continuous_6d_ruckig.png',dpi=130)
    plt.close(fig)
    vmax=np.max(np.abs(vel),axis=0)
    acc=np.array([x['acceleration_rad_s2'] for x in trajectory])
    amax=np.max(np.abs(acc),axis=0)
    jerk=np.abs(np.diff(acc,axis=0)/np.diff(
        np.array([x['time_s'] for x in trajectory]))[:,None])
    jmax=np.max(jerk,axis=0)
    # sampled FD jerk may overshoot analytical bound close to discontinuous
    # sample section boundaries: report, never pretend sampled derivative is
    # the analytical certificate. The Ruckig solver guarantees bound inside
    # each section; section joints preserve velocity and acceleration.
    report=dict(status='PASS',input_camera_fps=1/np.median(np.diff(t)),
       camera_frames=len(t),spline_sample_fps=args.spline_fps,
       spline_samples=len(dense_times),spline_translation_smoothing_mm=args.smooth_mm,
       continuous_so3_rotation_spline=True,
       camera_keyframe_count=len(idx),ruckig_segments=len(sections),
       ruckig_local_per_segment_no_cloud=True,
       pose_based_ik_from_camera_spline=True,
       nonzero_interior_waypoint_velocity=any(
         x['end_speed_norm']>1e-6 for x in sections[:-1]),
       source_duration_s=float(t[-1]-t[0]),
       joint_trajectory_duration_s=float(trajectory[-1]['time_s']),
       retiming_slowdown_factor=factor,
       max_joint_velocity_rad_s=vmax.tolist(),
       max_joint_acceleration_rad_s2=amax.tolist(),
       max_fd_joint_jerk_rad_s3=jmax.tolist(),
       planned_velocity_limits_rad_s=MAX_VEL.tolist(),
       planned_acceleration_limits_rad_s2=MAX_ACC.tolist(),
       planned_jerk_limits_rad_s3=MAX_JERK.tolist(),
       checked_joint_and_full_pitcher_states=safety_n,
       segments=sections,
       no_live_visual_servo=True,
       manufacturer_limit_calibrated=False,
       warnings='Only offline camera spline -> 6DOF Ruckig sections; extended timing may change physical latte-art result. Not manufacturer limits, motor dynamics or real robot safety.')
    (out/'summary.json').write_text(json.dumps(to_json(report),indent=2))
    print('SPLINE/RUCKIG REPORT:',json.dumps(
        {k:v for k,v in report.items() if k!='segments'},indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--input',default='runs/allowed_camera_replay')
    p.add_argument('--output',default='runs/continuous_motion')
    p.add_argument('--vendor-dir',default='vendor')
    p.add_argument('--smooth-mm',type=float,default=.35)
    p.add_argument('--spline-fps',type=int,default=100)
    p.add_argument('--knot-stride',type=int,default=3)
    p.add_argument('--dt',type=float,default=.02)
    p.add_argument('--initial-slowdown',type=float,default=3.0)
    args=p.parse_args()
    res=run(args)
    import sys
    sys.stdout.flush();sys.stderr.flush()
    if os.environ.get('MUJOCO_GL','').lower()=='osmesa':
        os._exit(0)


if __name__=='__main__':
    main()
