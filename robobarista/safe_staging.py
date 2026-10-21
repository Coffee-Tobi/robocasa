"""Conservative staging-pose search for the existing seven coffee stations.

An approach staging pose is 15-35 cm back/up from a dock and NEVER authorizes
mechanical contact at a port. A station is 'approved for demonstration' ONLY
when 6D IK, tool keepouts, MuJoCo robot contacts AND the entire interpolated
six-joint route to and from the known safe pose pass the nominal tests.

No safe staging pose? Mark BLOCKED and do not issue a fallback gesture that
touches equipment. This model has no real robot force/torque control.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation
from .workcell_constraints import DOCKS
from .robot_replay import compute_ik,_contacts


def _valid_state(model,data,qids,limits,q,tool='pitcher'):
    from .workcell_constraints import payload_check,TRAVEL_CLEARANCE_M
    import mujoco
    q=np.asarray(q,float)
    if np.any(q<limits[:,0]) or np.any(q>limits[:,1]):
        return False,'joint_limit'
    data.qpos[qids]=q
    mujoco.mj_forward(model,data)
    contacts=_contacts(model,data)
    if any(contacts):return False,'mu_joco_link_contact'
    tcp=model.body('rb_cell_tcp').id
    xyz=data.xpos[tcp]
    rot=data.xmat[tcp].reshape(3,3)
    if payload_check(xyz,rot,tool,clearance_m=TRAVEL_CLEARANCE_M):
        return False,'tool_hits_machine_keepout'
    return True,'OK'


def _check_route(model,data,qids,limits,qa,qb,tool):
    # Kinematic discrete upper bound ~0.018 rad per joint; MUST still model
    # whole continuous swept links and velocity/time limits before hardware.
    n=max(1,int(np.ceil(np.max(np.abs(np.asarray(qb)-qa))/.018)))
    for index in range(n+1):
        fraction=index/n
        q=qa+(qb-qa)*fraction
        good,reason=_valid_state(model,data,qids,limits,q,tool)
        if not good:return False,reason,n
    return True,'checked',n


def stage_candidates(dock):
    p=np.asarray(dock.position_m,float)
    direction=np.asarray(dock.approach_xyz,float)
    # Workcells require a clear 'pre-dock' IN FREE SPACE not precise insertion.
    # Each candidate remains outside nominal machine bounding boxes.
    for back,height in ((.20,.25),(.24,.37),(.32,.40),(.14,.38)):
        candidate=p-direction*back+np.array([0,0,height])
        if -0.75<candidate[0]<.75 and -.73<candidate[1]<.70:
            yield candidate


def plan_staging(model,data,qids,limits,home_q,seed=14):
    from .workcell_constraints import KEEP_OUTS
    tcp=model.body('rb_cell_tcp').id
    rng=np.random.default_rng(seed)
    lo,hi=limits[:,0],limits[:,1]
    seeds=[np.asarray(home_q).copy(),
           np.array([0,-.7,1.1,-1.1,.8,0.]),
           np.array([-1,-.7,1.2,-1.,.8,0.]),
           np.array([1,-.7,1.2,-1.,-.8,0.])]
    seeds.extend([lo+(.2+.6*rng.random(6))*(hi-lo) for _ in range(5)])
    angles=[
        np.eye(3),
        Rotation.from_euler('xyz',[0,90,0],degrees=True).as_matrix(),
        Rotation.from_euler('xyz',[180,0,0],degrees=True).as_matrix()
    ]
    # Two different grinder/espresso port types can have distinct free-space
    # staging poses, even though docking is not yet authorized.
    stages={}
    for name,d in DOCKS.items():
        found=None
        counters={'ik_reached':0,'pose_contact_free':0,
                  'safe_full_joint_routes':0,'rejected_reasons':[]}
        for p in stage_candidates(d):
            if found:break
            for R in angles:
                if found:break
                for start in seeds:
                    ik=compute_ik(model,data,tcp,qids,limits,p,R,start,
                                  max_evals=75)
                    if not ik['solved']:continue
                    counters['ik_reached']+=1
                    q=ik['q']
                    valid,reason=_valid_state(model,data,qids,limits,q,d.tool)
                    if not valid:
                        counters['rejected_reasons'].append(reason)
                        continue
                    counters['pose_contact_free']+=1
                    forward,reason,n=_check_route(
                        model,data,qids,limits,home_q,q,d.tool)
                    if not forward:
                        counters['rejected_reasons'].append('route:'+reason)
                        continue
                    backward,reason2,n2=_check_route(
                        model,data,qids,limits,q,home_q,d.tool)
                    if not backward:
                        counters['rejected_reasons'].append('return:'+reason2)
                        continue
                    counters['safe_full_joint_routes']+=1
                    found=dict(q_rad=q.tolist(),staging_position_m=p.tolist(),
                               staging_rotation_xyzw=Rotation.from_matrix(R).as_quat().tolist(),
                               sample_joint_substeps=n,
                               note='Kinematically vetted free-space pre-dock; NOT docking/contact/velocity release')
                    break
        stages[name]={
            'status':'PRE_DOCK_ROUTE_CHECKED' if found else 'BLOCKED',
            'stage':found,
            'attempts':{k:v for k,v in counters.items() if k!='rejected_reasons'},
            'rejected_example_reasons':sorted(set(counters['rejected_reasons']))[:7],
            'goal_position_m':list(d.position_m),
            'tool':d.tool,
            'actual_docking_approved':False
        }
        print('Staging',name,'=',stages[name]['status'],flush=True)
    return stages
