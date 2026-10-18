"""First calibrated Panda grasp-and-lift diagnostic (no transport).

Run:
MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/calibrated_grasp.py
"""
import numpy as np
import robosuite as suite

env = suite.make(env_name="Lift", robots="Panda", has_renderer=False,
                 has_offscreen_renderer=False, use_camera_obs=False,
                 initialization_noise=None, horizon=1000, ignore_done=True)
try:
    env.reset()
    robot=env.robots[0]
    arm=robot.arms[0]
    site_id=robot.eef_site_id[arm]
    gripper=robot.gripper[arm]
    cube_id=env.cube_body_id
    pad_names=gripper.important_geoms
    pad_ids=[env.sim.model.geom_name2id(name)
             for side in ("left_fingerpad", "right_fingerpad")
             for name in pad_names[side]]
    def hand(): return np.asarray(env.sim.data.site_xpos[site_id]).copy()
    def cube(): return np.asarray(env.sim.data.body_xpos[cube_id]).copy()
    def pad_positions(): return np.array([env.sim.data.geom_xpos[i].copy() for i in pad_ids])
    def contact():
        return env._check_grasp(gripper=gripper, object_geoms=env.cube)
    def advance(target, grip, steps=90, tol=0.003):
        for i in range(steps):
            error=target-hand()
            action=np.zeros(env.action_dim)
            action[:3]=np.clip(error*6,-0.16,0.16)
            action[-1]=grip
            env.step(action)
            if np.linalg.norm(target-hand())<tol: break
        return 1000*np.linalg.norm(target-hand())
    start_cube=cube()
    print("Würfelzentrum:", np.round(start_cube,4),flush=True)
    # Open gripper above the cube, avoiding accidental approach with closed fingers.
    above=start_cube+np.array([0,0,0.105])
    print("Anfahrt:",round(advance(above,-1.0),2),"mm Fehler",flush=True)
    for _ in range(15):
        a=np.zeros(env.action_dim);a[-1]=-1.0;env.step(a)
    # Pad center is ~3-4 mm higher than tracked EEF; use real pad center heights.
    # Move in small increments while continuously checking side alignment.
    reached=False
    for level in range(1,31):
        pads=pad_positions()
        pad_mid=pads.mean(axis=0)
        cube_now=cube()
        xy_offset=np.linalg.norm(pad_mid[:2]-cube_now[:2])
        dz=pad_mid[2]-cube_now[2]
        print(f"Stufe {level}: Pad-Zentrum über Würfelmitte {dz*1000:.1f} mm; "
              f"XY-Abstand {xy_offset*1000:.1f} mm",flush=True)
        if xy_offset > 12e-3:
            print("STOP: horizontale Fehlstellung > 12 mm",flush=True)
            break
        if dz <= 4e-3:
            reached=True
            break
        # Descend at most 8 mm per step, never beyond cube center minus 3mm.
        next_z=max(hand()[2]-0.008, cube_now[2]-0.003)
        next_target=np.array([cube_now[0], cube_now[1], next_z])
        advance(next_target,-1.0,steps=45,tol=0.0025)
    if reached:
        print("Greifhöhe erreicht; schließe Finger...",flush=True)
        for _ in range(40):
            action=np.zeros(env.action_dim);action[-1]=1.0;env.step(action)
        print("Griffkontakt erkannt:", bool(contact()),flush=True)
        initial_z=cube()[2]
        lift_target=hand()+np.array([0,0,0.07])
        advance(lift_target,1.0,steps=110)
        print("Würfelhub:", round((cube()[2]-initial_z)*1000,2),"mm",flush=True)
        print("Griffkontakt nach Hub:",bool(contact()),flush=True)
    else:
        print("Sicherheitsabbruch vor dem Schließen: Greifhöhe nicht bestätigt.",flush=True)
finally:
    env.close()
