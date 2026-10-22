"""Physics-level test: moving ball touches fixed steam wand proxy.

This deliberately tests contact force reader only, NOT complete pitcher geometry.
"""
import unittest
import numpy as np
import mujoco
from robobarista.contact_sensing import read_pitcher_wand_contacts,validate_collision_geoms

XML="""<mujoco model="contact_probe">
<option gravity="0 0 0" timestep="0.001"/>
<worldbody>
 <geom name="wand" type="sphere" size=".02" pos="0 0 0" />
 <body name="pitcher" pos=".039 0 0">
  <freejoint/>
  <geom name="pitcher_wall" type="sphere" size=".02" mass=".3"/>
 </body>
</worldbody>
</mujoco>"""

class ContactProbe(unittest.TestCase):
    def test_contact_force(self):
        m=mujoco.MjModel.from_xml_string(XML)
        d=mujoco.MjData(m)
        mujoco.mj_forward(m,d)
        pitcher=m.geom("pitcher_wall").id
        wand=m.geom("wand").id
        validate_collision_geoms(m,["pitcher_wall","wand"])
        # MuJoCo's mj_contactForce needs constraint solver results from mj_step.
        mujoco.mj_step(m,d)
        result=read_pitcher_wand_contacts(m,d,[pitcher],[wand])
        self.assertTrue(result["touch"])
        self.assertGreater(result["normal_force_n"],0)
        self.assertEqual(len(result["contacts"]),1)

    def test_no_contact(self):
        m=mujoco.MjModel.from_xml_string(XML)
        d=mujoco.MjData(m)
        d.qpos[0]=.2
        mujoco.mj_forward(m,d)
        mujoco.mj_step(m,d)
        result=read_pitcher_wand_contacts(m,d,[m.geom("pitcher_wall").id],[m.geom("wand").id])
        self.assertFalse(result["touch"])

if __name__=="__main__":unittest.main()
