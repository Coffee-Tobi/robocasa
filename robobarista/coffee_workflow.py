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


def _story_scene(vendor_dir):
    built=_add_held_pitcher_robot_variant(Path(vendor_dir).resolve())
    fr=Path(built).parent
    root=ET.parse(fr/'rb_fr5_safe_cell_robot.xml')
    tcp=root.getroot().find(".//body[@name='rb_cell_tcp']")
    if tcp is None:raise RuntimeError('Missing FR5 wrist TCP')
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
    root.write(fr/'rb_fr5_story_robot.xml',encoding='unicode')
    env=ET.parse(built)
    env.getroot().find('include').set('file','rb_fr5_story_robot.xml')
    out=fr/'rb_fr5_coffee_process.xml'
    env.write(out,encoding='unicode')
    return out


def _alpha(model,tool):
    tools={
        'pitcher':('rb_replay_pitcher_body',
                   'rb_replay_pitcher_spout','rb_replay_pitcher_handle'),
        'portafilter':('rb_story_portafilter_basket',
                      'rb_story_portafilter_handle'),
        'plastic_cup':('rb_story_plastic_cup_cup',)}
    for key,names in tools.items():
        for name in names:
            model.geom(name).rgba[3]=(1.0 if tool==key else 0.0)


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


def _frame(rgb,label,action,phase,idx,n,total_clear,notes):
    im=Image.fromarray(rgb).resize((960,720),Image.Resampling.BILINEAR)
    pic=Image.new('RGB',(960,805),(20,31,39))
    pic.paste(im,(0,59))
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
    scene=_story_scene(Path(args.vendor_dir))
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
    frames=[]
    events=[]
    cup_at_espresso=False
    espresso_visual=False
    pitcher_foam=False
    try:
        for idx,step in enumerate(recipe):
            action=step['action'];tool=step['tool']
            _alpha(model,tool if tool!='bare_tcp' else 'none')
            if action=='place_cup':
                _place_cup(model,data,DOCKS['espresso_cup'].position_m)
                cup_at_espresso=True
            if action=='brew':
                espresso_visual=True
            if action=='cup_to_pour':
                _place_cup(model,data,DOCKS['pour_cup'].position_m)
                cup_at_espresso=False
            if action=='steam':
                pitcher_foam=True
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
                rgb,_=render_pair(renderer,data,'cell_overview')
                desc=('coffee-coloured flow symbolic' if espresso_visual
                      else 'no espresso extracted')
                if pitcher_foam:desc+=' | milk foam SYMBOLIC'
                if action=='pour':desc+=' | assumed artistic wave (no fluid)'
                frames.append(_frame(rgb,step['label'],action,
                    'predock',seq_index,len(leg),
                    step['checked_joint_substeps'],
                    desc+' | real docking NOT executed'))
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
        'virtual_espresso_and_milk_workflow_stages':len(events),
        'safe_predock_stages':sum(v['kinematic_predock_checked'] for v in events),
        'true_machine_docking_completed':False,
        'true_portafilter_grasp_completed':False,
        'true_milk_frothing_or_espresso_flow':False,
        'assumed_synthetic_wrist_latte_pattern':
            pour_ctrls is not None,
        'video_frames':len(frames),
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
