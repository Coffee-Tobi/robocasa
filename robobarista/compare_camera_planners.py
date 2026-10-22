"""Offline comparison using an existing VERIFIED GitHub Actions camera artifact.

Usage: python -m robobarista.compare_camera_planners --artifact ZIP --output runs/offline_camera
Requires numpy, scipy, matplotlib. Does NOT contact FAIRINO firmware or render MuJoCo.
FAIRINO-like strategy is a stop-at-waypoint, linear Cartesian MoveL SURROGATE.
Our strategy is the saved continuous RGB-D smoothing input to IK/Ruckig.
Both use all 31 SAME 15-fps camera measurements, lengthened from 2s to 6s.
These are geometric reference tracks, not independently executed robot motions.
"""
import argparse,csv,json,zipfile
from pathlib import Path
import numpy as np
from scipy.interpolate import interp1d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def run(archive, output):
    output=Path(output);output.mkdir(exist_ok=True,parents=True)
    with zipfile.ZipFile(archive) as z:
        cam=json.loads(z.read("camera/camera_measurements.json"))
        smooth=json.loads(z.read("continuous/camera_continuous_6d.json"))
        motor=json.loads(z.read("motor/motor_tracking.json"))
        replay=json.loads(z.read("camera/robot_replay.json"))
        ruckig=json.loads(z.read("continuous/ruckig_joint_trajectory.json"))
    if len(cam)!=31 or any(not x["valid"] for x in cam):
        raise ValueError("Expected the 31 validated recorded 15fps camera frames")
    tm=np.array([x["time_s"] for x in cam])
    pcm=np.array([x["position_m"] for x in cam],float)
    ts=np.array([x["time_s"] for x in smooth])
    ps=np.array([x["position_m"] for x in smooth],float)
    t=np.arange(0.,6.000001,1/15)
    source_t=t/3
    # STOP-GO MoveL SURROGATE through EACH input camera point.
    # Does not emulate real controller firmware/blending/ServoCart.
    j=np.clip(np.searchsorted(tm,source_t,side="right")-1,0,len(tm)-2)
    u=np.clip((source_t-tm[j])/(tm[j+1]-tm[j]),0,1)
    fade=u*u*(3-2*u)
    vendor=(1-fade[:,None])*pcm[j]+fade[:,None]*pcm[j+1]
    ours=interp1d(ts,ps,axis=0,fill_value="extrapolate")(source_t)
    observed=interp1d(tm,pcm,axis=0)(source_t)
    ev=1000*np.linalg.norm(vendor-observed,axis=1)
    eo=1000*np.linalg.norm(ours-observed,axis=1)
    ideal_replay=np.array([x["executed_position_m"] for x in replay])
    emotor=np.array([x["lag_position_mm"] for x in motor])
    metrics={
      "input_camera_frames":len(cam),
      "recording_duration_s":float(tm[-1]),
      "retimed_comparison_duration_s":6.,
      "same_camera_input_for_both":True,
      "camera_source_is_synthetic":True,
      "real_fairino_sdk_executed":False,
      "vendor_like_implementation":"Offline stop-go MoveL surrogate, not FAIRINO official controller",
      "custom_implementation":"Saved C2 smooth 6D camera reference used before IK/Ruckig",
      "vendor_like_mean_deviation_from_camera_mm":float(ev.mean()),
      "vendor_like_max_deviation_from_camera_mm":float(ev.max()),
      "custom_mean_deviation_from_camera_mm":float(eo.mean()),
      "custom_max_deviation_from_camera_mm":float(eo.max()),
      "old_kinematic_fk_replay_max_error_mm":float((1000*np.linalg.norm(ideal_replay-pcm,axis=1)).max()),
      "old_mujoco_PD_motor_max_TCP_lag_mm":float(emotor.max()),
      "old_mujoco_PD_motor_mean_TCP_lag_mm":float(emotor.mean()),
      "ruckig_joint_trajectory_duration_s":float(ruckig[-1]["time_s"]),
      "warning":"Closeness to camera measurements is NOT real pose accuracy, and is not proof of superior planning. Old motor error is from another reference path."
    }
    (output/"summary.json").write_text(json.dumps(metrics,indent=2))
    with (output/"paths.csv").open("w",newline="") as f:
        w=csv.writer(f)
        w.writerow(["t_s","camera_x_m","camera_y_m","camera_z_m",
          "MoveL_synthetic_x_m","MoveL_synthetic_y_m","MoveL_synthetic_z_m",
          "Ruckig_input_x_m","Ruckig_input_y_m","Ruckig_input_z_m"])
        for sec,p,v,o in zip(t,observed,vendor,ours):
            w.writerow([sec,*p,*v,*o])
    zero=observed[0]
    fig,axes=plt.subplots(1,2,figsize=(12,4.6))
    axes[0].scatter((pcm[:,0]-zero[0])*1000,(pcm[:,1]-zero[1])*1000,
        label="Camera 15 FPS",s=10,color="black")
    for path,label in ((vendor,"MoveL-like stop-go"),(ours,"Our smoothed trajectory")):
        axes[0].plot((path[:,0]-zero[0])*1000,
          (path[:,1]-zero[1])*1000,label=label)
    axes[0].set(xlabel="X relative [mm]",ylabel="Y relative [mm]",
         title="Camera-derived Cartesian reference paths")
    axes[0].set_aspect("equal",adjustable="datalim")
    axes[1].plot(t,ev,label="MoveL-like stop-go")
    axes[1].plot(t,eo,label="Our smooth target")
    axes[1].set(xlabel="Time [s]",ylabel="Distance to camera interpolation [mm]",
        title="Deviation from CAMERA observation (not motor error)")
    for ax in axes:
        ax.grid(alpha=.22);ax.legend(fontsize=8)
    fig.suptitle("OFFLINE planner surrogate; no FAIRINO firmware was run")
    fig.tight_layout()
    fig.savefig(output/"camera_planner_comparison.png",dpi=160)
    plt.close(fig)
    print(json.dumps(metrics,indent=2))
    return metrics

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--artifact",required=True,help="ZIP from successful complete Colab workflow")
    p.add_argument("--output",default="runs/offline_camera")
    a=p.parse_args()
    run(a.artifact,a.output)
