"""First cautious PandaOmron mug approach/grasp attempt in original RoboCasa.
Run:
MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/coffee_grasp_attempt.py
"""
from pathlib import Path
import subprocess
import numpy as np
import cv2
import robosuite
from robocasa.environments.kitchen.composite.brewing.prepare_coffee import PrepareCoffee

# Same minimal scene variant that already resets successfully.
_get_cfgs = PrepareCoffee._get_obj_cfgs
def only_mug(self):
    return [x for x in _get_cfgs(self) if x.get("name") == "obj"]
PrepareCoffee._get_obj_cfgs = only_mug

OUT = Path(__file__).resolve().parent
VIDEO = OUT / "coffee_grasp_attempt.mp4"
W, H, FPS = 640, 480, 12
env = None
proc = None
try:
    env = robosuite.make(
        "PrepareCoffee", robots="PandaOmron", seed=0,
        has_renderer=False, has_offscreen_renderer=True,
        use_camera_obs=False, render_camera="robot0_agentview_center",
        obj_registries=("objaverse",), use_distractors=False,
        horizon=1600, ignore_done=True,
    )
    env.reset()
    robot = env.robots[0]
    print("Aktionsdimensionen:", env.action_dim, "Arme:", robot.arms, flush=True)
    print("Aktionsgrenzen:", env.action_spec, flush=True)
    print("Objektnamen:", list(env.objects), flush=True)
    # RoboCasa labels the task mug 'obj'; lookup its actual MuJoCo root body.
    mug = env.objects["obj"]
    mug_body = env.sim.model.body_name2id(mug.root_body)
    arm = "right" if "right" in robot.arms else robot.arms[0]
    hand_site = robot.eef_site_id[arm]
    cam = next((c for c in ("robot0_agentview_center", "robot0_agentview_right", "robot0_robotview")
                if c in env.sim.model.camera_names), env.sim.model.camera_names[0])
    def hand():
        return np.asarray(env.sim.data.site_xpos[hand_site]).copy()
    def cup():
        return np.asarray(env.sim.data.body_xpos[mug_body]).copy()

    print("Handstart:", np.round(hand(),3), "Tasse:", np.round(cup(),3), flush=True)
    print("Abstand Hand/Tasse:", round(np.linalg.norm(hand()-cup()),3), "m", flush=True)
    ff = ["ffmpeg","-y","-loglevel","error","-f","rawvideo","-pix_fmt","rgb24",
          "-s",f"{W}x{H}","-r",str(FPS),"-i","-","-an","-c:v","libx264",
          "-preset","veryfast","-crf","25","-pix_fmt","yuv420p",
          "-movflags","+faststart",str(VIDEO)]
    proc = subprocess.Popen(ff, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    def frame():
        rgb = env.sim.render(height=H,width=W,camera_name=cam)
        proc.stdin.write(np.ascontiguousarray(np.flipud(rgb),dtype=np.uint8).tobytes())
    def step(target, grip, n):
        cmd = np.zeros(env.action_dim)
        cmd[:3] = np.clip((target-hand())*5.0, -0.12, 0.12)
        cmd[6] = grip  # PandaOmron ordering: right arm (6), gripper (1), base, torso.
        env.step(cmd)
        if n%4 == 0: frame()

    frame()
    # Safe waypoint: move above mug first, without entering cabinet or forcing grip.
    initial_cup = cup()
    waypoint = initial_cup + np.array([0,0,0.18])
    print("Anfahrt oberhalb der Tasse:",np.round(waypoint,3),flush=True)
    for n in range(180):
        step(waypoint,-1,n)
        if np.linalg.norm(hand()-waypoint)<0.025: break
    dist = np.linalg.norm(hand()-waypoint)
    print("Abstand zur Anfahrposition:",round(dist,3),"m",flush=True)
    if dist > 0.06:
        print("ABBRUCH: Tasse mit stationärer Armbewegung nicht erreichbar; keine riskante Greifbewegung.",flush=True)
    else:
        # Attempt to descend to roughly mug center height, keeping gripper open.
        goal = initial_cup + np.array([0,0,0.035])
        for n in range(100):
            step(goal,-1,n)
            if np.linalg.norm(hand()-goal)<0.022: break
        print("Greifpositionsfehler:",round(np.linalg.norm(hand()-goal),3),"m",flush=True)
        if np.linalg.norm(hand()-goal)<0.045:
            for n in range(28): step(hand(),1,n)
            held = bool(env._check_grasp(gripper=robot.gripper[arm],object_geoms=mug))
            print("Greifkontakt:",held,flush=True)
            if held:
                prev = cup()[2]
                lift = hand()+np.array([0,0,0.10])
                for n in range(100): step(lift,1,n)
                print("Tassenhub:",round((cup()[2]-prev)*1000,1),"mm",flush=True)
        else:
            print("Kein Greifversuch: Ziel nicht ausreichend erreicht.",flush=True)
    for n in range(16): frame()
finally:
    if proc:
        if proc.stdin and not proc.stdin.closed: proc.stdin.close()
        err = proc.stderr.read().decode(errors="replace")
        rc = proc.wait()
        print("Video:",VIDEO if rc==0 else "FFmpeg-Fehler: "+err[-500:],flush=True)
    if env: env.close()
