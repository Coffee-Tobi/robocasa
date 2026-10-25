"""No tracking camera, actual 6-joint FR5, IR40 holder, and Swift-only scene."""
from pathlib import Path
import mujoco
import numpy as np

from robobarista.swift_fr5_pick_stand import (
    build_robot_holder_swift_scene, read_tcp_to_basket, RECEIVER_XYZ,
    HOLDER_BASKET_XYZ, BASKET_SITE, PARKED_COPY,
)
from robobarista.robot_replay import robot_qpos_addresses


def test_robot_and_holder_scene_has_zero_tracking_cameras():
    m=mujoco.MjModel.from_xml_path(str(build_robot_holder_swift_scene(Path("vendor"))))
    d=mujoco.MjData(m)
    qids, limits=robot_qpos_addresses(m)
    assert len(qids)==6
    assert m.ncam==0
    assert m.body(PARKED_COPY).id>=0
    assert m.body("rb_pf_stand_fixed_3point").id>=0
    assert m.geom(BASKET_SITE).id>=0
    assert m.body("rb_90_ir40_handle").id>=0
    assert m.body("rb_swift_mini_visual_reference").id>=0
    assert m.site("rb_swift_mini_PORT_UNMEASURED").id>=0
    assert m.site("rb_pf_holder_repeatable_pickup").id>=0
    mujoco.mj_forward(m,d)
    T=read_tcp_to_basket(m,d)
    assert T.shape==(4,4)
    assert np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
    assert HOLDER_BASKET_XYZ[1]<RECEIVER_XYZ[1]
    for name in ("rb_pitcher_free","rb_plastic_cup_free"):
        try: m.joint(name)
        except KeyError: pass
        else: raise AssertionError("Unused object present: "+name)
