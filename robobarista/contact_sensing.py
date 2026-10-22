"""Read REAL MuJoCo contact forces at pitcher/wand geometry.

NOTE: rb_replay_pitcher_* currently has contype=0, conaffinity=0, so
no touch event can occur until a contact-enabled pitcher/wand model
is introduced. Never claim collision detection on visual-only geoms.
"""
import numpy as np

def validate_collision_geoms(model,names):
    for name in names:
        g=model.geom(name)
        if g.contype==0 and g.conaffinity==0:
            raise ValueError(f'{name}: collision disabled, cannot detect contact')

def read_pitcher_wand_contacts(model,data,pitcher_geoms,wand_geoms):
    import mujoco
    p,w=set(map(int,pitcher_geoms)),set(map(int,wand_geoms))
    entries=[]
    for i in range(data.ncon):
        c=data.contact[i]
        if not ((c.geom1 in p and c.geom2 in w) or (c.geom2 in p and c.geom1 in w)):
            continue
        wrench=np.zeros(6)
        mujoco.mj_contactForce(model,data,i,wrench)
        force_world=np.asarray(c.frame).reshape(3,3).T@wrench[:3]
        entries.append({'normal_n':float(wrench[0]),'force_world_n':force_world.tolist(),'point_m':np.asarray(c.pos).tolist()})
    return {'touch':bool(entries),'normal_force_n':sum(max(0,e['normal_n']) for e in entries),'contacts':entries}
