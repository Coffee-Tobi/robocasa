"""Smooth virtual over-the-shoulder camera for the FR5 *video renderer*.

Never substitute this moving cinematic camera for calibrated RGB-D cameras:
the latter remain fixed for AprilTag triangulation and metric evaluation.
"""
import numpy as np
import mujoco
import xml.etree.ElementTree as ET
from .geometry import look_at_quat

RIG_BODY="rb_follow_camera_rig"
CAMERA_NAME="rb_arm_follow"

def insert_camera(world):
    rig=ET.SubElement(world,"body",{"name":RIG_BODY,"mocap":"true",
                                   "pos":"0 -1.2 1.6"})
    ET.SubElement(rig,"camera",{"name":CAMERA_NAME,"fovy":"62",
                               "pos":"0 0 0","quat":"1 0 0 0"})
    return rig

class ArmFollowCamera:
    """Positions cinematic camera slightly behind/above FR5 TCP.

    Smooths CAMERA pose only; does not modify robot qpos, motor commands,
    AprilTag observations, or calibrated world-camera extrinsics.
    """
    def __init__(self,model,smoothing=.40,view='behind'):
        if view not in ('behind','handle_close'):
            raise ValueError('Unknown cinematic camera view')
        self.view=view
        self.model=model
        self.mocap_id=int(model.body_mocapid[model.body(RIG_BODY).id])
        if self.mocap_id<0:raise ValueError("Following camera must be a mocap body")
        self.tcp_id=model.body("rb_cell_tcp").id
        self.base_id=model.body("base_link").id
        self.smoothing=float(smoothing)
        self.eye=None
        self.focus=None

    def update(self,data):
        tcp=np.asarray(data.xpos[self.tcp_id],dtype=float).copy()
        base=np.asarray(data.xpos[self.base_id],dtype=float).copy()
        radial=tcp[:2]-base[:2]
        length=float(np.linalg.norm(radial))
        radial=radial/length if length>0.08 else np.array([0.,-1.])
        side=np.array([-radial[1],radial[0]])
        if self.view=='handle_close':
            # Front-quarter opposite side: look BACK AT the wrist and
            # handle from outside the arm, instead of hiding the mug
            # behind white robot links. This is a *cinematic* camera only,
            # never a calibrated AprilTag camera or collision geometry.
            eye=np.array([tcp[0]+.64*radial[0]+.48*side[0],
                          tcp[1]+.64*radial[1]+.48*side[1],
                          max(1.35,tcp[2]+.35)])
            focus=tcp.copy()
            focus[:2]+=.12*radial
            focus[2]+=.055
        else:
            # The old over-the-shoulder fallback is useful for overview,
            # but may hide cup handles behind links.
            eye=np.array([tcp[0]-1.06*radial[0]+.44*side[0],
                          tcp[1]-1.06*radial[1]+.44*side[1],
                          max(1.60,tcp[2]+.51)])
            focus=tcp.copy()
            focus[:2]+=.26*radial
            focus[2]-=.11
        alpha=self.smoothing
        if self.eye is None:
            self.eye=eye
            self.focus=focus
        else:
            self.eye=(1-alpha)*self.eye+alpha*eye
            self.focus=(1-alpha)*self.focus+alpha*focus
        data.mocap_pos[self.mocap_id]=self.eye
        data.mocap_quat[self.mocap_id]=look_at_quat(self.eye,self.focus)
        mujoco.mj_forward(self.model,data)
        return self.eye.copy(),self.focus.copy()
