"""Visual SAFE espresso + milk + assumed latte-art process on FR5 island.

This is a complete *stage-based kinematic storyboard*, not a working barista:
robot travels collision-screened HOME <-> seven free-space predock poses;
grinding, espresso extraction, steam and milk/foam are colored visual-state
proxies, with NO equipment insertion, grasp contacts, liquid flow, or verified
latte-art hydrodynamics. The assumed pour gesture is an explicitly synthetic
pitcher tilt/sway at a collision-screened free-space pose.
"""
from __future__ import annotations
import argparse,json,os,shutil,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image,ImageDraw

from .camera_joint_replay import _add_held_pitcher_robot_variant,_find_start
from .safe_staging import plan_staging,_check_route,_valid_state
from .robot_replay import robot_qpos_addresses
from .cell_demo import _set_free_pose
from .workcell_constraints import DOCKS, PAYLOAD_SPHERES
from .demo import render_pair
from .follow_camera import insert_camera,ArmFollowCamera,CAMERA_NAME
from .robocasa_coffee_adapter import CoffeeMachineState
from .dual_ir40_concept import add_visual_dual_ir40_50


POUR_PHASES=('Approach','Base-fill','Lower','Sway','Cut-through')
# Five illustrative skill stages inspired by phase-structured latte-art
# demonstrations; NOT learned DMPs and NOT actual milk hydrodynamics.

RECIPE=[
    ('01  Siebtraeger aufnehmen (virtuell)','pitcher_park','portafilter','portafilter_pickup'),
    ('02  Mahlen + integrierter Tamper','grinder_portafilter','portafilter','grind_tamp'),
    ('03  Siebtraeger zur Bruehgruppe','espresso_portafilter','portafilter','insert_grouphead'),
    ('04  Kunststoffbecher unter Espresso','espresso_cup','plastic_cup','place_cup'),
    ('05  Espresso-Bezug (visualisiert)','espresso_cup','bare_tcp','brew'),
    ('06  Milchkannne vom Parkplatz','pitcher_park','pitcher','pitcher_pickup'),
    ('07  Milch dosieren','milk_dispenser_pitcher','pitcher','dose'),
    ('08  Dampflanze + Milchaufschaeumen','espresso_steam_pitcher','pitcher','steam'),
    ('09  Becher zum Giessplatz','pour_cup','plastic_cup','cup_to_pour'),
    ('10  Angenommene Latte-Art-Giessgeste','pour_cup','pitcher','pour'),
    ('11  Kanne am Rinser reinigen (symbolisch)','pitcher_park','pitcher','rinse'),
]


def _story_scene(vendor_dir,marker_profile='compact',cup_style='handle_mug'):
    if cup_style not in ('takeaway','handle_mug'):
        raise ValueError('Cup style must be takeaway or handle_mug')
    built=_add_held_pitcher_robot_variant(Path(vendor_dir).resolve(),marker_profile)
    fr=Path(built).parent
    root=ET.parse(fr/'rb_fr5_safe_cell_robot.xml')
    tcp=root.getroot().find(".//body[@name='rb_cell_tcp']")
    if tcp is None:raise RuntimeError('Missing FR5 wrist TCP')
    # Conceptual non-contact dual IR40-50 equipment. Not a physical pickup.
    add_visual_dual_ir40_50(tcp,include_props=True,
                            compact_tags=(marker_profile=='compact'),
                            cup_style=cup_style)
    for tool in ('portafilter','plastic_cup'):
        b=ET.SubElement(tcp,'body',{'name':'rb_story_'+tool,'pos':'0 0 0'})
        if tool=='portafilter':
            specs=[('basket','cylinder',[0,0,.045],[.047,.020]),
                   ('handle','capsule',[0,-.11,.045],[.014,.12])]
        else:
            specs=[('cup','cylinder',[0,0,.05],[.044,.052])]
        for part,typ,pos,size in specs:
            ET.SubElement(b,'geom',{
                'name':'rb_story_'+tool+'_'+part,
                'type':typ,'pos':' '.join(map(str,pos)),
                'size':' '.join(map(str,size)),
                'rgba':'.88 .37 .12 1' if tool=='portafilter' else '.48 .86 .99 .6',
                'contype':'0','conaffinity':'0','mass':'0',
                **({'euler':'1.5708 0 0'}
                   if part=='handle' else {})})
    # Coloured geometry is VISUAL ONLY, not simulated actual fluids.
    ET.SubElement(tcp,'geom',{'name':'rb_story_foam_in_pitcher',
        'type':'cylinder','pos':'0 0 .112','size':'.045 .003',
        'rgba':'.99 .99 .94 0','mass':'0','contype':'0','conaffinity':'0'})
    root.write(fr/'rb_fr5_story_robot.xml',encoding='unicode')
    env=ET.parse(built)
    env.getroot().find('include').set('file','rb_fr5_story_robot.xml')
    world=env.getroot().find('worldbody')
    insert_camera(world)
    cup=world.find(".//body[@name='rb_plastic_cup']")
    if cup is None:raise RuntimeError('Cannot find plastic cup')
    if cup_style=='handle_mug':
        # The legacy internal body name rb_plastic_cup is retained for
        # existing stage/dataset compatibility. Here it is visualized as a
        # handled reusable CERAMIC MUG, never as a disposable cup.
        for geom in cup.findall("geom"):
            if geom.get("name") in (
                "rb_cup_base","rb_plastic_wall_0","rb_plastic_wall_1",
                "rb_plastic_wall_2","rb_plastic_rim"):
                geom.set("rgba",".89 .85 .78 1")
        # Three massless geometry pieces form an OUTWARD +X mug loop,
        # accessible by the handle; no physical grasp force is simulated.
        for name,ends in (
            ("upper",(.037,0,.083,.083,0,.083)),
            ("lower",(.037,0,.039,.083,0,.039)),
            ("outer",(.083,0,.039,.083,0,.083))):
            ET.SubElement(cup,"geom",{
                "name":"rb_free_mug_handle_"+name,
                "type":"capsule",
                "fromto":" ".join(map(str,ends)),
                "size":".006",
                "rgba":".89 .85 .79 1",
                "contype":"0","conaffinity":"0","mass":"0"})
    ET.SubElement(cup,'geom',{'name':'rb_story_espresso_disc',
       'type':'cylinder','pos':'0 0 .067','size':'.038 .003',
       'rgba':'.3 .13 .05 0','mass':'0','contype':'0','conaffinity':'0'})
    ET.SubElement(cup,'geom',{'name':'rb_story_latte_pattern',
       'type':'ellipsoid','pos':'0 0 .071','size':'.019 .010 .002',
       'rgba':'1 1 .92 0','mass':'0','contype':'0','conaffinity':'0'})
    ET.SubElement(world,'geom',{'name':'rb_story_steam_visual',
       'type':'sphere','pos':'.645 .23 1.12','size':'.042',
       'rgba':'.99 .99 .99 0','mass':'0','contype':'0','conaffinity':'0'})
    out=fr/'rb_fr5_coffee_process.xml'
    env.write(out,encoding='unicode')
    return out


def _alpha(model,tool):
    # Hide legacy TCP tool clones; the new props mount under gripper jaws.
    tools={
        'pitcher':('rb_replay_pitcher_body',
                   'rb_replay_pitcher_spout','rb_replay_pitcher_handle'),
        'portafilter':('rb_story_portafilter_basket',
                      'rb_story_portafilter_handle'),
        'plastic_cup':('rb_story_plastic_cup_cup',)}
    for names in tools.values():
        for name in names:
            model.geom(name).rgba[3]=0.
    attached={
        'pitcher':('rb_gripped_pitcher_adapter','rb_gripped_pitcher_stop_lower',
                   'rb_gripped_pitcher_stop_upper','rb_gripped_pitcher_bridge',
                   'rb_gripped_pitcher_body','rb_gripped_pitcher_rim',
                   'rb_gripped_pitcher_spout'),
        'portafilter':('rb_gripped_portafilter_adapter',
                      'rb_gripped_portafilter_handle',
                      'rb_gripped_portafilter_basket'),
        'plastic_cup':('rb_gripped_cup_body',
                       'rb_gripped_cup_handle_upper',
                       'rb_gripped_cup_handle_lower',
                       'rb_gripped_cup_handle_outer')}
    for key,names in attached.items():
        for name in names:
            try:
                geom=model.geom(name)
            except KeyError:
                # Takeaway mode does not contain ceramic handle pieces.
                if name.startswith('rb_gripped_cup_handle_'):
                    continue
                raise
            geom.rgba[3]=(0.82 if key=='plastic_cup' else 1.0) if tool==key else 0.0


def _make_cup_contents(model):
    # A visible coffee-coloured disc INSIDE the practice cup, its pose
    # comes from the cup's freejoint. NOT fluid dynamics.
    pass


def _place_cup(model,data,where):
    _set_free_pose(model,data,'rb_plastic_cup_free',where,0)


def _interp(qa,qb,n):
    for t in np.linspace(0,1,n):
        s=t*t*(3-2*t)
        yield (1-s)*qa+s*qb


def _frame(rgb,label,action,phase,idx,n,total_clear,notes,overview=None):
    im=Image.fromarray(rgb).resize((960,720),Image.Resampling.BILINEAR)
    pic=Image.new('RGB',(960,806),(20,31,39))
    pic.paste(im,(0,59))
    if overview is not None:
        inset=Image.fromarray(overview).resize((276,207),Image.Resampling.BILINEAR)
        pic.paste(inset,(670,70))
        d0=ImageDraw.Draw(pic)
        d0.rectangle((667,67,948,280),outline=(239,243,246),width=3)
        d0.rectangle((670,70,946,89),fill=(18,30,41))
        d0.text((676,74),'GESAMTANSICHT',fill=(250,250,250))
    d=ImageDraw.Draw(pic)
    d.text((12,8),label,fill=(235,247,250))
    d.text((12,30),f'Phase: {action} | {idx+1}/{n} | '
           f'preflight accepted: {total_clear}',fill=(150,226,207))
    d.text((12,786),notes,fill=(244,211,140))
    return pic


def _random_pour_gesture(model,data,qids,limits,home,stages,seed):
    """Find safe raised-pour region and generate small side-to-side wiggle.

    Searches a safe IK stage first, then small six-joint changes; never
    claims actual liquid or true 'jug tip above surface' geometry.
    """
    # Candidate safe positions near the cup from known planning fixtures.
    rng=np.random.default_rng(seed+29)
    candidates=[]
    for name in ('pour_cup','espresso_cup','pitcher_park'):
        item=stages[name]
        if item['stage'] is not None:
            candidates.append((name,np.asarray(item['stage']['q_rad'])))
    for name,q in candidates:
        okay,reason,_=_check_route(model,data,qids,limits,
                                  home,q,'pitcher')
        if not okay:continue
        # Prefer a REPRODUCIBLE five-phase demonstrator: first settle,
        # gradually pitch the wrist, then sway sideways and cut through.
        # These are joint-space gestures, explicitly NOT milk-pouring curves.
        offsets=[
            (0.,0.,0.),
            (0.,-.020,0.),
            (-.015,-.035,0.),
            (.020,-.038,.012),
            (0.,-.010,-.006),
        ]
        for scale in (1.,.5,.25):
            proposed=[]
            for delta0,delta4,delta5 in offsets:
                sample=q.copy()
                sample[0]+=delta0*scale
                sample[4]+=delta4*scale
                sample[5]+=delta5*scale
                proposed.append(sample)
            segments=zip(proposed,proposed[1:])
            if all(_check_route(model,data,qids,limits,a,b,'pitcher')[0]
                   for a,b in segments) and _check_route(
                        model,data,qids,limits,q,proposed[0],'pitcher')[0]:
                return name,proposed
        controls=[q.copy()]
        for i in range(4):
            found=None
            for j in range(120):
                trial=controls[-1].copy()
                # Deliberate sway + tilt: always small, never free collision
                # exemptions merely because the stage is called "pour".
                trial[0]+=rng.uniform(-.038,.038)
                trial[4]+=rng.uniform(-.065,.065)
                trial[5]+=rng.uniform(-.045,.045)
                passable,_,_=_check_route(model,data,qids,limits,
                                         controls[-1],trial,'pitcher')
                if passable:
                    found=trial;break
            if found is None:break
            controls.append(found)
        if len(controls)>=3:
            for qa,qb in zip(controls[::-1],controls[-2::-1]):
                pass # all straight segments already tested in opposite dir
            return name,controls
    return None,None


def run(args):
    out=Path(args.output).resolve()
    out.mkdir(exist_ok=True,parents=True)
    cup_style=getattr(args,'cup_style','handle_mug')
    scene=_story_scene(Path(args.vendor_dir),
                       getattr(args,'marker_profile','compact'),
                       cup_style)
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    _set_free_pose(model,data,'rb_pitcher_free',[-10,-10,-10],0)
    _set_free_pose(model,data,'rb_plastic_cup_free',[-10,-10,-10],0)
    mujoco.mj_forward(model,data)
    qids,limits=robot_qpos_addresses(model)
    home,_=_find_start(model,data,qids,limits,args.seed)
    stages=plan_staging(model,data,qids,limits,home,args.seed)
    if any(v['status']!='PRE_DOCK_ROUTE_CHECKED' for v in stages.values()):
        raise RuntimeError('One of seven necessary predock sites is BLOCKED; cannot claim full workflow')

    _,pour_ctrls=_random_pour_gesture(model,data,qids,limits,
                                      home,stages,args.seed)
    note_pour='synthetic small safe wrist tilt/sway; NOT a verified pour'
    # Stage path screening: check before any rendering.
    recipe=[]
    for label,dock,tool,action in RECIPE:
        q=np.asarray(stages[dock]['stage']['q_rad'],float)
        safe,reason,checked=_check_route(
            model,data,qids,limits,home,q,tool)
        if not safe:
            # Safe failure: this step never animated, no bypass!
            raise RuntimeError(f'Recipe step {action} unsafe: {reason}')
        recipe.append(dict(label=label,dock=dock,tool=tool,
                           action=action,checked_joint_substeps=checked,
                           q=q))
    renderer=mujoco.Renderer(model,width=args.width,height=args.height)
    camera_mode=getattr(args,'camera_view','follow')
    follow=ArmFollowCamera(model) if camera_mode=='follow' else None
    frames=[]
    events=[]
    cup_at_espresso=False
    espresso_visual=False
    espresso_fixture=CoffeeMachineState()
    espresso_demo_gate=None
    pitcher_foam=False
    try:
        for idx,step in enumerate(recipe):
            action=step['action'];tool=step['tool']
            _alpha(model,tool if tool!='bare_tcp' else 'none')
            if action=='place_cup':
                _place_cup(model,data,DOCKS['espresso_cup'].position_m)
                cup_at_espresso=True
            if action=='brew':
                # MIT RoboCasa-inspired fixture state gate: cup MUST really
                # be under the outlet. No false switch/portafilter lock:
                # demo animation stays explicitly symbolic.
                mujoco.mj_forward(model,data)
                cup_xyz=data.xpos[model.body("rb_plastic_cup").id].copy()
                espresso_demo_gate=espresso_fixture.demo_brew_step(cup_xyz)
                espresso_visual=bool(espresso_demo_gate["virtual_coffee_disc_may_be_drawn"])
            if action=='cup_to_pour':
                _place_cup(model,data,DOCKS['pour_cup'].position_m)
                cup_at_espresso=False
            if action=='steam':
                pitcher_foam=True
            # Render status proxies only; these never affect physics/contact.
            model.geom('rb_story_espresso_disc').rgba[3]=.9 if espresso_visual else 0.
            model.geom('rb_story_latte_pattern').rgba[3]=(
                1.0 if action in ('pour','rinse') else 0.0)
            model.geom('rb_story_foam_in_pitcher').rgba[3]=0.
            model.geom('rb_gripped_pitcher_foam').rgba[3]=(
                .92 if pitcher_foam and tool=='pitcher' else 0.0)
            model.geom('rb_story_steam_visual').rgba[3]=(
                .25 if action=='steam' else 0.0)
            # Travel home->safe predock->home. All prechecked.
            leg=list(_interp(home,step['q'],args.frames_leg))
            leg+=list(_interp(step['q'],home,args.frames_leg))[1:]
            if action=='pour':
                if pour_ctrls is not None:
                    safe,reason,_=_check_route(model,data,qids,limits,
                                               home,pour_ctrls[0],'pitcher')
                    if safe:
                        # Include a small assumed decorative pour wave.
                        leg=list(_interp(home,pour_ctrls[0],args.frames_leg))
                        for a,b in zip(pour_ctrls,pour_ctrls[1:]):
                            leg+=list(_interp(a,b,5))[1:]
                        leg+=list(_interp(pour_ctrls[-1],pour_ctrls[0],5))[1:]
                        leg+=list(_interp(pour_ctrls[0],home,args.frames_leg))[1:]
            for seq_index,q in enumerate(leg):
                # Re-run collision check for displayed q, with actual tool
                # (even during animated stage hold).
                okay,why=_valid_state(model,data,qids,limits,q,tool)
                if not okay:
                    raise RuntimeError(f'Recipe render unsafe {action}: {why}')
                if follow is not None:
                    follow.update(data)
                    rgb,_=render_pair(renderer,data,CAMERA_NAME)
                    overview,_=render_pair(renderer,data,'cell_overview')
                else:
                    rgb,_=render_pair(renderer,data,'cell_overview')
                    overview=None
                desc=('coffee-coloured flow symbolic' if espresso_visual
                      else 'no espresso extracted')
                if pitcher_foam:desc+=' | milk foam SYMBOLIC'
                if action=='pour':
                    which=min(4,int(5*seq_index/max(1,len(leg))))
                    desc+=' | POUR STAGE: '+POUR_PHASES[which]+' (no milk physics)'
                frames.append(_frame(rgb,step['label'],action,
                    'predock',seq_index,len(leg),
                    step['checked_joint_substeps'],
                    desc+' | real docking NOT executed',overview=overview))
            events.append(dict(label=step['label'],action=action,
                tool=tool,target_dock=step['dock'],
                requires_mechanical_contact=action in (
                    'grind_tamp','insert_grouphead','brew','dose','steam'),
                contact_executed=False,
                kinematic_predock_checked=True,
                visual_process_only=True,
                frame_count=len(leg)))
            print('STEP COMPLETE (PRE-DOCK ONLY):',action,len(leg),flush=True)
    finally:
        renderer.close()
    (out/'recipe.json').write_text(json.dumps(events,indent=2))
    mp4=False
    if frames:
        folder=out/'images'
        folder.mkdir(exist_ok=True)
        for k,im in enumerate(frames):
            im.save(folder/f'barista_{k:04d}.png')
        gif=[im.convert('P',palette=Image.Palette.ADAPTIVE,colors=80)
             for im in frames]
        gif[0].save(out/'coffee_workflow.gif',save_all=True,
            append_images=gif[1:],duration=round(1000/args.fps),
            loop=0,optimize=False)
        if shutil.which('ffmpeg'):
            subprocess.run(['ffmpeg','-y','-loglevel','error',
             '-framerate',str(args.fps),'-i',str(folder/'barista_%04d.png'),
             '-c:v','libx264','-pix_fmt','yuv420p',
             str(out/'coffee_workflow.mp4')],check=True)
            mp4=(out/'coffee_workflow.mp4').stat().st_size>1000
    report={
        'status':'PASS' if len(events)==len(RECIPE) and mp4 else 'FAIL',
        'cup_style':cup_style,
        'cup_grip_claim':'visually positioned near handle, NOT physical force-validated grasp' if cup_style=='handle_mug' else 'conceptual opposed jaws around disposable cup body',
        'compact_tags_on_carried_tools':getattr(args,'marker_profile','compact')=='compact',
        'virtual_espresso_and_milk_workflow_stages':len(events),
        'safe_predock_stages':sum(v['kinematic_predock_checked'] for v in events),
        'true_machine_docking_completed':False,
        'true_portafilter_grasp_completed':False,
        'true_milk_frothing_or_espresso_flow':False,
        'robocasa_inspired_coffee_fixture_state_gate':espresso_demo_gate,
        'brew_animation_requires_cup_under_nozzle':True,
        'real_button_and_portafilter_lock_proven':False,
        'assumed_synthetic_wrist_latte_pattern':
            pour_ctrls is not None,
        'latte_art_five_phase_structure':list(POUR_PHASES),
        'pour_motion_is_demonstrative_not_cdmp_learned':True,
        'video_frames':len(frames),
        'video_camera_view':camera_mode,
        'tag_profile':getattr(args,'marker_profile','legacy'),
        'unique_gripped_object_tags':bool(getattr(args,'marker_profile','legacy')=='compact'),
        'over_shoulder_follow_with_overview_inset':camera_mode=='follow',
        'calibrated_apriltag_rgbd_cameras_unchanged':True,
        'two_IR40_grippers_visible_only':True,
        'tool_objects_attached_to_gripper_frames':True,
        'old_tcp_attached_tool_clones_hidden':True,
        'actual_finger_contact_force_simulated':False,
        'mp4_created':mp4,
        'no_mechanical_contact_or_liquid_physics':True,
        'limitations':'Full logical kinematic espresso/milk/latte choreography with explicit no-contact machine pre-docks; simulated processes are labels and proxy object poses only. No portafilter insertion, safe-contact modelling, espresso fluid, froth texture or actual latte art.'
    }
    (out/'summary.json').write_text(json.dumps(report,indent=2))
    print('COFFEE PROCESS:',json.dumps(report,indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--vendor-dir',default='vendor')
    p.add_argument('--output',default='runs/coffee_process')
    p.add_argument('--seed',type=int,default=14)
    p.add_argument('--width',type=int,default=760)
    p.add_argument('--height',type=int,default=570)
    p.add_argument('--fps',type=float,default=8)
    p.add_argument('--camera-view',choices=('follow','overview'),default='follow')
    p.add_argument('--marker-profile',choices=('legacy','compact'),default='legacy')
    p.add_argument('--frames-leg',type=int,default=4)
    args=p.parse_args()
    report=run(args)
    import sys
    sys.stdout.flush();sys.stderr.flush()
    if os.environ.get('MUJOCO_GL','').lower()=='osmesa':
        os._exit(0 if report['status']=='PASS' else 2)
    if report['status']!='PASS':
        raise SystemExit(2)

if __name__=='__main__':
    main()
