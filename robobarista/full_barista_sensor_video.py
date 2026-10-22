"""Complete RoboBarista workcell movie: two IR40 tools + labelled force checks.

This renders the previously tested 11-stage FR5 coffee choreography, then
annotates each stage in German with the gripper concept, commanded machine
operation and an *explicitly synthetic* F/T sensor demonstration.

NOT physical portafilter docking, actual steam-wand collision contact, or
espresso/milk dynamics. The virtual F/T display is a spring-style training
signal disconnected from the coffee-scene MuJoCo contacts. End-to-end
contact demos require compatible hardware CAD, calibrated F/T and a contact
controller. Stops/safety states are computed from mock wrench samples.
"""
from __future__ import annotations
import argparse,json,os,subprocess,shutil,math
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
from .coffee_workflow import run as run_coffee
from .fairino_force_and_trajectory import (
    illustrative_contact_curve,portafilter_lock_monitor,steam_wand_side_contact,
    LookaheadConfig,RecordingFairino,lookahead_sequence)

STEP_TEXT={
"portafilter_pickup":("1 | SIEBTRAEGER AUFNEHMEN",
      "IR40 Griffgreifer richtet sich am Siebtraegergriff aus.",
      "Anschliessend: Griffadapter halten; noch keine Greifphysik."),
"grind_tamp":("2 | MAHLEN UND TAMPEN",
      "Siebtraeger unter Muehlenauslauf; integrierter Tamper.",
      "Die Muehlen-/Tamper-Aktion ist symbolisch modelliert."),
"insert_grouphead":("3 | SIEBTRAEGER EINSETZEN",
      "Bayonett: ansetzen, dann verdrehen bis Widerstand.",
      "FT-Drehmomentsignal rechts: SYNTHETISCH, kein echter Lock."),
"place_cup":("4 | KUNSTSTOFFBECHER POSITIONIEREN",
      "IR40 Bechergreifer transportiert den Becher zum Auslauf.",
      "Abstellen ist bisher eine kinematische Visualisierung."),
"brew":("5 | ESPRESSO BEZIEHEN",
      "Espressomaschine starten, Extraktionszeit ueberwachen.",
      "Kaffeefluss und Maschinenknopf sind symbolisch."),
"pitcher_pickup":("6 | MILCHKANNE AUFNEHMEN",
      "IR40 Griffgreifer fasst den seitlichen Adapter der Kanne.",
      "Die Kanne ist noch starr befestigt, ohne Reibungskontakt."),
"dose":("7 | MILCH DOSIEREN",
      "Milchkanne unter Dosierstelle positionieren.",
      "Die Milchmenge ist derzeit nur als Zustand modelliert."),
"steam":("8 | MILCH AUFSCHAEUMEN",
      "Kanne zur Dampflanze, seitlichen Kontakt vorsichtig pruefen.",
      "FT-Seitenkraft rechts: SYNTHETISCH, keine echte Dueseberuehrung."),
"cup_to_pour":("9 | BECHER ZUM GIESSPLATZ",
      "IR40 Bechergreifer positioniert Espresso unter der Kanne.",
      "Becherposition ist symbolisch, noch ohne Boden-Reibung."),
"pour":("10 | LATTE ART GIESSEN",
      "Grosse Bewegung vermeiden; feine Ausgiesserbewegung.",
      "Latte Art: getrennte FAIRINO-NewSpline-Demo, keine Milchphysik."),
"rinse":("11 | KANNE REINIGEN",
      "Kanne zum Rinser, spuelen, zur Parkposition.",
      "Reinigung ist bislang symbolisch visualisiert."),
}
TOOLS={
"portafilter_pickup":"IR40-50 A | Griffadapter Siebtraeger",
"grind_tamp":"IR40-50 A | Griffadapter Siebtraeger",
"insert_grouphead":"IR40-50 A | Griffadapter Siebtraeger",
"place_cup":"IR40-50 B | breite Becherbacken",
"brew":"Kein Greifer aktiv | Maschinenprozess",
"pitcher_pickup":"IR40-50 A | Milchkanne",
"dose":"IR40-50 A | Milchkanne",
"steam":"IR40-50 A | Milchkanne",
"cup_to_pour":"IR40-50 B | breite Becherbacken",
"pour":"IR40-50 A | Milchkanne",
"rinse":"IR40-50 A | Milchkanne"
}

def font(size):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
              "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"):
        try:return ImageFont.truetype(p,size)
        except OSError:continue
    return ImageFont.load_default()

def draw_contact_schematic(draw,x,y,action,progress):
    # Concept sketch, NOT part of 3D MuJoCo contact solver.
    if action=="insert_grouphead":
        draw.ellipse((x+22,y+17,x+120,y+115),outline=(169,193,210),width=5)
        ang=math.pi*(-.70+progress*.58)
        px=x+71+39*math.cos(ang)
        py=y+66+39*math.sin(ang)
        draw.line((x+71,y+66,px,py),fill=(253,197,102),width=9)
        draw.ellipse((px-5,py-5,px+5,py+5),fill=(253,197,102))
        draw.text((x+147,y+43),"VERDREHEN",font=font(14),fill="white")
        draw.text((x+147,y+67),"virtuell",font=font(13),fill=(177,204,218))
    elif action=="steam":
        draw.line((x+89,y+15,x+89,y+118),fill=(170,190,204),width=8)
        pos=x+25+38*progress
        draw.rectangle((pos,y+50,pos+40,y+121),outline=(107,217,229),width=5)
        draw.text((x+147,y+43),"SEITENKONTAKT",font=font(13),fill="white")
        draw.text((x+147,y+67),"virtuell",font=font(13),fill=(177,204,218))
    else:
        draw.text((x+30,y+52),"ABLAUF IM 3D-MODELL",font=font(16),fill=(164,209,227))

def draw_panel(frame,action,stage_i,progress,wrench,ft_result,signal_history,
               trajectory_supported):
    output=Image.new("RGB",(1340,806),(18,30,40))
    output.paste(frame,(0,0))
    d=ImageDraw.Draw(output)
    x=978
    a,b,c=STEP_TEXT[action]
    d.rectangle((965,0,1340,806),fill=(18,30,40))
    d.text((x,14),"ROBObARISTA | VIRTUELL",font=font(17),fill=(160,215,235))
    d.text((x,58),a,font=font(17),fill=(255,230,172))
    d.text((x,93),"SCHRITT %02d / 11"%stage_i,font=font(15),fill="white")
    d.rectangle((x,124,1325,130),fill=(61,80,93))
    d.rectangle((x,124,x+int(347*progress),130),fill=(87,190,211))
    d.text((x,155),"WERKZEUG",font=font(13),fill=(157,208,224))
    # Multi-line descriptions with room for longer German labels.
    tool=TOOLS[action]
    d.text((x,178),tool[:28],font=font(15),fill="white")
    if len(tool)>28:d.text((x,198),tool[28:],font=font(15),fill="white")
    d.text((x,240),"PROZESS",font=font(13),fill=(157,208,224))
    for n,ln in enumerate(wrap(b,36)):
        d.text((x,263+n*20),ln,font=font(14),fill="white")
    d.text((x,339),"KONTAKTSKIZZE",font=font(13),fill=(157,208,224))
    draw_contact_schematic(d,x,360,action,progress)
    d.text((x,510),"F/T-SENSORKONZEPT",font=font(13),fill=(157,208,224))
    if action=="insert_grouphead":
        d.text((x,534),"Drehmoment: %.2f Nm"%abs(wrench.tz),font=font(18),fill=(255,203,133))
    elif action=="steam":
        d.text((x,534),"Seitenkraft: %.2f N"%abs(wrench.fy),font=font(18),fill=(255,203,133))
    else:
        d.text((x,534),"Kein F/T-Kontakttest",font=font(15),fill="white")
    # Distinct virtual wrench signal graph.
    d.rectangle((x,575,1325,648),outline=(70,100,115),width=1)
    if len(signal_history)>1:
        top=max(1.,max(signal_history)*1.1)
        pts=[(x+3+int(340*i/max(1,len(signal_history)-1)),
             646-int(67*v/top)) for i,v in enumerate(signal_history)]
        d.line(pts,fill=(255,196,104),width=3)
    d.text((x,659),ft_result[:32],font=font(13),fill=(250,218,156))
    d.text((x,688),"NICHT GEMESSEN / F/T SYNTHETISCH",font=font(12),fill=(255,154,130))
    d.text((x,713),"KEIN ECHTER MASCHINENKONTAKT",font=font(12),fill=(255,154,130))
    d.text((x,741),"FAIRINO Lookahead: vorbereitet" if trajectory_supported
           else "FAIRINO Lookahead: offen",font=font(12),fill=(157,208,224))
    d.text((x,766),"Film zeigt sichere Vorpositionen",font=font(12),fill=(180,193,207))
    return output

def wrap(s,width=36):
    words=s.split()
    current=""
    result=[]
    for w in words:
        candidate=(current+" "+w).strip()
        if len(candidate)>width and current:
            result.append(current)
            current=w
        else:current=candidate
    if current:result.append(current)
    return result[:3]

def run(args):
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=True)
    raw=out/"base_workflow"
    # Exactly the code of the established barista workcell with both grippers.
    # Each stage stays clear of the machine contact surfaces.
    from types import SimpleNamespace
    p=SimpleNamespace(vendor_dir=args.vendor_dir,output=str(raw),seed=14,
        width=args.width,height=args.height,fps=float(args.capture_fps),
        camera_view="follow",frames_leg=args.frames_leg)
    summary=run_coffee(p)
    if summary["status"]!="PASS":raise RuntimeError("Underlying scene did not pass")
    events=json.loads((raw/"recipe.json").read_text())
    if len(events)!=11:raise RuntimeError("Missing full espresso->latte->rinse stages")
    raw_frames=sorted((raw/"images").glob("barista_*.png"))
    if len(raw_frames)!=sum(x["frame_count"] for x in events):
        raise RuntimeError("Video frames and workflow stage metadata disagree")
    # Actual official SDK interface for a vendor-recorded joint trajectory,
    # NOT fed the Cartesian storyboard (not a real LA trajectory file).
    lookahead=lookahead_sequence(
        RecordingFairino(),LookaheadConfig(filename="calibrated_joint_path.txt"),
        uploaded_verified=True)
    lookahead["trajectory_file_present"]=False
    lookahead["trajectory_file_name_is_example_only"]=True

    frames_dir=out/"frames";frames_dir.mkdir(exist_ok=True)
    detailed=[]
    row=0
    picture_index=0
    for stage_i,e in enumerate(events,1):
        history=[]
        samples=[]
        for j in range(e["frame_count"]):
            progress=j/max(1,e["frame_count"]-1)
            wrench,origin=illustrative_contact_curve(e["action"],progress)
            samples.append(wrench)
            history.append(abs(wrench.tz) if e["action"]=="insert_grouphead"
                           else abs(wrench.fy) if e["action"]=="steam" else 0.)
            if e["action"]=="insert_grouphead":
                verdict=portafilter_lock_monitor(samples)["state"]
            elif e["action"]=="steam":
                verdict=steam_wand_side_contact(samples)["state"]
            else:
                verdict="KEIN SENSORTEST"
            with Image.open(raw_frames[row]) as image:
                full=draw_panel(image.convert("RGB"),e["action"],stage_i,
                      progress,wrench,verdict,history,True)
            row+=1
            # Frame duplication gives 12fps output from 6fps camera render;
            # does not fabricate extra robot kinematic steps.
            for k in range(args.repeat):
                full.save(frames_dir/f"frame_{picture_index:04d}.jpg",quality=88)
                picture_index+=1
        final_samples=(
            portafilter_lock_monitor(samples) if e["action"]=="insert_grouphead"
            else steam_wand_side_contact(samples) if e["action"]=="steam"
            else {"state":"NO_FT_SENSOR_USED"})
        detailed.append({**e,"description":STEP_TEXT[e["action"]][1],
           "mock_ft":final_samples,
           "measured_mujoco_contact":False,"machine_docking_executed":False})
    if not shutil.which("ffmpeg"):raise RuntimeError("ffmpeg missing")
    video=out/"complete_barista_with_sensor_concepts.mp4"
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",
        str(args.playback_fps),"-i",str(frames_dir/"frame_%04d.jpg"),
        "-c:v","libx264","-pix_fmt","yuv420p","-movflags","+faststart",
        str(video)],check=True)
    result={
        "status":"PASS","stages":len(detailed),
        "frames":picture_index,"fps":args.playback_fps,
        "duration_s":picture_index/args.playback_fps,
        "video_created":video.stat().st_size>10000,
        "both_IR40_gripper_concepts_visible":True,
        "transport_upright_ik_attempts":summary.get("transport_upright_ik_attempts",0),
        "transport_upright_ik_successes":summary.get("transport_upright_ik_successes",0),
        "transport_upright_ik_blocked":summary.get("transport_upright_ik_blocked",0),
        "transport_level_fail_closed_hold_frames":summary.get("transport_level_fail_closed_hold_frames",0),
        "transport_station_targets_not_guaranteed_if_holds":summary.get(
            "transport_station_targets_not_guaranteed_if_holds",False),
        "transport_worst_tilt_deg":summary.get("transport_worst_tilt_deg"),
        "strict_upright_guarantee":summary.get("strict_upright_guarantee",False),
        "portafilter_bayonet_test":detailed[2]["mock_ft"],
        "steam_side_contact_test":detailed[7]["mock_ft"],
        "fairino_trajectory_LA_vendor_method_plan":lookahead,
        "hardware_ft_measured":False,
        "mujoco_contact_at_portafilter_or_steam_wand":False,
        "portafilter_lock_mechanically_verified":False,
        "real_milk_steaming":False,"real_espresso_extraction":False,
        "actual_robot_sdk_invoked":False,
        "slowed_time_only":True,
        "intermediate_trajectory_time_and_acceleration_not_physically_replanned":True,
        "scene_limitation":"11 collision-screened pre-docks with symbolic machine actions; separate synthetic F/T training curves; no physical insertion or steam-wand contact."
    }
    (out/"summary.json").write_text(json.dumps(result,indent=2))
    (out/"step_events.json").write_text(json.dumps(detailed,indent=2))
    print(json.dumps(result,indent=2),flush=True)
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--vendor-dir",default="vendor")
    p.add_argument("--output",default="runs/whole_barista")
    p.add_argument("--width",type=int,default=640)
    p.add_argument("--height",type=int,default=480)
    p.add_argument("--capture-fps",type=int,default=6)
    p.add_argument("--repeat",type=int,default=2)
    p.add_argument("--playback-fps",type=int,default=6,
                   help="Slower playback: 6 FPS produces ~69s instead of 34.5s")
    p.add_argument("--frames-leg",type=int,default=9)
    a=p.parse_args()
    if a.playback_fps<3 or a.capture_fps*a.repeat<a.playback_fps:
        raise ValueError("Keep playback FPS between 3 and the generated frame rate")
    r=run(a)
    import sys
    sys.stdout.flush()
    if os.environ.get("MUJOCO_GL","").lower()=="osmesa":
        os._exit(0 if r["status"]=="PASS" else 2)

if __name__=="__main__":main()
