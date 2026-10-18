"""Diagnose height of Panda finger collision geometry versus cube in robosuite.
Run: MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/gripper_height_diagnostic.py
"""
import numpy as np
import robosuite as suite

env = suite.make(env_name="Lift", robots="Panda", has_renderer=False,
                 has_offscreen_renderer=False, use_camera_obs=False,
                 initialization_noise=None)
try:
    env.reset()
    robot = env.robots[0]
    arm = robot.arms[0]
    site = robot.eef_site_id[arm]
    cube_id = env.cube_body_id
    def hand():
        return np.array(env.sim.data.site_xpos[site])
    def cube():
        return np.array(env.sim.data.body_xpos[cube_id])
    def pads():
        names = robot.gripper[arm].important_geoms
        for side in ("left_fingerpad", "right_fingerpad"):
            ids = [env.sim.model.geom_name2id(g) for g in names[side]]
            for gid in ids:
                print(side, env.sim.model.geom_id2name(gid),
                      "Position (m):", np.round(env.sim.data.geom_xpos[gid], 4), flush=True)
    print("EEF-Position:", np.round(hand(),4), flush=True)
    print("Würfelmitte:", np.round(cube(),4), flush=True)
    print("Würfeloberkante ungefähr:", round(cube()[2]+0.022,4), flush=True)
    print("Greiferpad-Geometrien zu Beginn:", flush=True)
    pads()
    target = cube() + np.array([0, 0, 0.09])
    print("Anfahrt zu konservativer Höhe (9 cm über Würfelmitte):", np.round(target,4), flush=True)
    for i in range(140):
        error = target-hand()
        action=np.zeros(env.action_dim)
        action[:3]=np.clip(error*6,-0.2,0.2)
        action[-1]=-1.0
        env.step(action)
        if np.linalg.norm(target-hand()) < 0.004:
            break
    print("EEF nach Anfahrt:", np.round(hand(),4), flush=True)
    print("Greiferpad-Geometrien nach Anfahrt:", flush=True)
    pads()
    print("Noch KEIN Greif-/Absenkversuch. Werte hier teilen.", flush=True)
finally:
    env.close()
