"""Independent physics prototype: two IR40-50-inspired jaw sets and tool handles.

The 40-mm stroke is represented as two 20-mm sliding fingers. These are
conceptual dimensions, NOT the Inlux STEP geometry or tested hardware.
"""
from pathlib import Path
import argparse,json
import mujoco
import numpy as np

def scene_xml():
    return """<mujoco model="two_ir40_contact_probe">
  <compiler angle="radian"/>
  <option timestep="0.001" gravity="0 0 0" integrator="implicitfast"/>
  <default><geom friction="1.2 0.005 0.0001" solref="0.015 1" condim="4"/></default>
  <worldbody>
    <geom name="floor" type="plane" size=".8 .8 .02" rgba=".65 .68 .70 1"/>
    <light pos="0 -.3 .8"/>
    <!-- Separate fixed demo mounts represent the two arms of a dual-tool bracket.
         They are separated here so both grasps can be tested independently. -->
    <body name="handle_gripper" pos="-.18 0 .15">
      <geom type="box" size=".025 .03 .016" rgba=".2 .3 .4 1" contype="0" conaffinity="0"/>
      <body name="handle_left" pos="-.035 0 0">
        <joint name="handle_left_slide" type="slide" axis="1 0 0" range="0 .020" damping="3"/>
        <geom name="handle_left_pad" type="box" size=".006 .025 .022" pos="0 0 0" rgba=".9 .55 .2 1"/>
      </body>
      <body name="handle_right" pos=".035 0 0">
        <joint name="handle_right_slide" type="slide" axis="-1 0 0" range="0 .020" damping="3"/>
        <geom name="handle_right_pad" type="box" size=".006 .025 .022" rgba=".9 .55 .2 1"/>
      </body>
    </body>
    <body name="cup_gripper" pos=".18 0 .15">
      <geom type="box" size=".025 .03 .016" rgba=".2 .3 .4 1" contype="0" conaffinity="0"/>
      <body name="cup_left" pos="-.065 0 0">
        <joint name="cup_left_slide" type="slide" axis="1 0 0" range="0 .020" damping="3"/>
        <geom name="cup_left_pad" type="box" size=".008 .027 .024" rgba=".3 .7 .8 1"/>
      </body>
      <body name="cup_right" pos=".065 0 0">
        <joint name="cup_right_slide" type="slide" axis="-1 0 0" range="0 .020" damping="3"/>
        <geom name="cup_right_pad" type="box" size=".008 .027 .024" rgba=".3 .7 .8 1"/>
      </body>
    </body>
    <!-- Detachable standardized fixture, attached to virtual pitcher/portafilter.
         Neck is 25mm across, and indexed shoulders constrain lateral movement.
         Contact-enabled and removable via replaceable MJCF geometry. -->
    <body name="standardized_tool_handle" pos="-.18 0 .15">
      <freejoint/>
      <geom name="tool_grip_neck" type="box" size=".0125 .018 .018" mass=".12" rgba=".6 .6 .65 1"/>
      <geom name="tool_left_stop" type="box" size=".018 .021 .003" pos="0 0 -.022" mass=".01" rgba=".6 .6 .65 1"/>
      <geom name="tool_right_stop" type="box" size=".018 .021 .003" pos="0 0 .022" mass=".01" rgba=".6 .6 .65 1"/>
    </body>
    <!-- Simplified rigid cup proxy. Real thin-wall cone and compliant jaw pads
         are intentionally NOT represented by this cylinder. -->
    <body name="cup" pos=".18 0 .12">
      <freejoint/>
      <geom name="cup_collision" type="cylinder" size=".040 .050" mass=".15" rgba=".88 .8 .7 .9"/>
    </body>
  </worldbody>
  <actuator>
    <position joint="handle_left_slide" kp="900" ctrlrange="0 .020" forcerange="-40 40"/>
    <position joint="handle_right_slide" kp="900" ctrlrange="0 .020" forcerange="-40 40"/>
    <position joint="cup_left_slide" kp="900" ctrlrange="0 .020" forcerange="-40 40"/>
    <position joint="cup_right_slide" kp="900" ctrlrange="0 .020" forcerange="-40 40"/>
  </actuator>
</mujoco>"""

def run(output):
    m=mujoco.MjModel.from_xml_string(scene_xml())
    d=mujoco.MjData(m)
    # Begin open and close over 0.4 seconds, followed by hold.
    records=[]
    for k in range(1200):
        t=k*m.opt.timestep
        fraction=np.clip((t-.1)/.4,0,1)
        d.ctrl[:]=.020*fraction
        mujoco.mj_step(m,d)
        if k%10==0:
            contacts=[]
            for j in range(d.ncon):
                c=d.contact[j]
                pair=(m.geom(c.geom1).name,m.geom(c.geom2).name)
                if any("pad" in x for x in pair):
                    force=np.zeros(6)
                    mujoco.mj_contactForce(m,d,j,force)
                    contacts.append({"geoms":pair,"normal_force_n":float(force[0])})
            records.append({"time_s":round(t,4),"contacts":contacts,
                            "jaws_m":d.qpos[:4].tolist()})
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    (out/"two_ir40_scene.xml").write_text(scene_xml())
    (out/"grip_contacts.json").write_text(json.dumps(records,indent=2))
    result={"frames":len(records),
       "handle_contact_samples":sum(any(any("handle_" in n for n in e["geoms"]) for e in s["contacts"]) for s in records),
       "cup_contact_samples":sum(any(any("cup_" in n for n in e["geoms"]) for e in s["contacts"]) for s in records),
       "note":"Gravity-free stationary contact fixture and idealized rigid bodies; no robot transport, cup deformation, or actual vendor gripper performance."}
    (out/"summary.json").write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return result

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",default="runs/dual_ir40")
    a=p.parse_args();run(a.output)
