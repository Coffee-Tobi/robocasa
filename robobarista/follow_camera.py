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
    ET.SubElement(rig,"camera",{"name":CAMERA_NAME,"fovy":"72",
                               "pos":"0 0 0","quat":"1 0 0 0"})
    return rig

class ArmFollowCamera:
    """Positions cinematic camera slightly behind/above FR5 TCP.

    Smooths CAMERA pose only; does not modify robot qpos, motor commands,
    AprilTag observations, or calibrated world-camera extrinsics.
    """
    def __init__(self,model,smoothing=.40):
        self.model=model
        self.mocap_id=int(model.body_mocapid[model.body(RIG_BODY).id])
        if self.mocap_id<0:raise ValueError("Following camera must be a mocap body")
        self.tcp_id=model.body("rb_cell_tcp").id
        self.base_id=model.body("base_link").id
        self.smoothing=float(smoothing)
        self.eye=None
        self.focus=None

    def update(self,data,target_xyz=None):
        """Shoulder view from the side: expose TCP, gripper and held object.

        For demonstration/tracking target_xyz is the free pitcher reference.
        Only changes the cinematic mocap camera, never calibrated RGB-D rigs.
        """
        tcp=np.asarray(data.xpos[self.tcp_id],dtype=float).copy()
        base=np.asarray(data.xpos[self.base_id],dtype=float).copy()
        target=(np.asarray(target_xyz,float).copy()
                if target_xyz is not None else tcp.copy())
        if target_xyz is None:
            for name in ("rb_gripped_pitcher_body","rb_gripped_portafilter_basket",
                         "rb_gripped_cup_body"):
                try:
                    geom=self.model.geom(name).id
                    if self.model.geom_rgba[geom,3]>.15:
                        target=np.asarray(data.geom_xpos[geom]).copy()
                        break
                except KeyError:
                    pass
        radial=target[:2]-base[:2]
        length=float(np.linalg.norm(radial))
        radial=radial/length if length>0.08 else np.array([0.,-1.])
        side=np.array([-radial[1],radial[0]])
        # Side-dominant three-quarter view: robot does not hide the payload.
        framing=.57*target+.43*tcp
        eye=np.array([framing[0]-.92*radial[0]+1.14*side[0],
                      framing[1]-.92*radial[1]+1.14*side[1],
                      max(1.60,framing[2]+.61)])
        focus=framing.copy()
        focus[2]-=.10
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
