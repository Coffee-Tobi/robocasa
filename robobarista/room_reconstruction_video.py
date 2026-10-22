"""Render a genuine 3-camera RGB-D workcell reconstruction as an MP4.

Source of displayed geometry: reconstructed_xyz_cloud.npz, created by
room_reconstruction.py from *rendered depth images*. Camera color labels
come from captured provenance, never from ground-truth scene meshes.

A still synchronized camera capture is revealed over animation time:
camera 1 -> camera 2 -> camera 3 -> visible-map comparison. Thus an
animated REVEAL, NOT a claim of moving video cameras, SLAM or continuous
room sensing. MuJoCo truth is only overlaid in the last evaluation phase.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def render_video(input_dir, output=None, fps=8, frames=36,
                 max_points_per_camera=2000):
    folder=Path(input_dir).resolve()
    target=Path(output).resolve() if output else folder
    target.mkdir(exist_ok=True,parents=True)
    points_file=folder/"reconstructed_xyz_cloud.npz"
    report_file=folder/"summary.json"
    if not points_file.exists() or not report_file.exists():
        raise FileNotFoundError(
            "First execute robobarista.room_reconstruction to produce "
            "reconstructed_xyz_cloud.npz and summary.json")
    saved=np.load(points_file,allow_pickle=False)
    points=np.asarray(saved["xyz"],float)
    if "camera_index" not in saved:
        raise ValueError("Cloud has no camera provenance; regenerate with updated room_reconstruction.py")
    source=np.asarray(saved["camera_index"],int)
    names=[str(x) for x in saved["camera_names"]]
    if points.shape[0]!=source.shape[0] or points.shape[1]!=3:
        raise ValueError("Cloud and source camera indices disagree")
    if len(names)!=3 or len(points)==0:
        raise ValueError("Expected three RGB-D cameras with nonempty point cloud")
    report=json.loads(report_file.read_text(encoding="utf8"))
    if report.get("tag_count_observed",0)<1:
        raise ValueError("No observed fixed AprilTag positions to compare")
    if fps<=0 or frames<12:
        raise ValueError("Provide positive fps and at least twelve frames")
    # Deterministic evenly-spaced subsample PER CAMERA of measured points.
    clouds=[]
    for i in range(3):
        subset=points[source==i]
        if not len(subset):
            raise ValueError(f"Camera {i}: no reconstructed points")
        idx=np.linspace(0,len(subset)-1,
                        min(len(subset),max_points_per_camera),dtype=int)
        clouds.append(subset[idx])
    colors=["#2C99C1","#EF9F46","#8C73D3"]
    frame_path=target/"rgbd_reveal_frames"
    frame_path.mkdir(parents=True,exist_ok=True)
    img_count=0
    for k in range(frames):
        # The first 2/3 progressively accumulate point clouds by camera.
        progress=k/max(1,frames-1)
        show_truth=progress>=.77
        fig=plt.figure(figsize=(12,6.75),dpi=105)
        ax=fig.add_axes([.025,.11,.66,.80],projection="3d")
        ax.set_xlim(-1.75,1.75);ax.set_ylim(-1.50,1.55);ax.set_zlim(0,2.3)
        ax.set_box_aspect((3.5,3.05,2.3),zoom=.95)
        ax.view_init(elev=21+6*progress,azim=-55-30*progress)
        total_shown=0
        for i,pts in enumerate(clouds):
            fraction=np.clip(progress*3-i,0,1)
            if progress>=.77:fraction=1.
            n=int(len(pts)*fraction)
            if n:
                visible=pts[:n]
                ax.scatter(visible[:,0],visible[:,1],visible[:,2],
                           s=1.2,alpha=.55,c=colors[i],rasterized=True,
                           label=f"{names[i]}: {n:,} Punkte")
                total_shown+=n
        ax.set_xlabel("X [m]");ax.set_ylabel("Y [m]");ax.set_zlabel("Z [m]")
        ax.set_title("Aus Tiefenpixeln gemessene sichtbare 3D-Oberflächen",
                     fontsize=11,pad=8)
        if show_truth:
            for marker in report["fixed_station_details"]:
                est=marker.get("position_estimated_m")
                true=marker.get("ground_truth_m")
                if est is None or true is None:continue
                ax.scatter([est[0]],[est[1]],[est[2]],
                           marker="x",s=45,c="crimson",depthshade=False)
                ax.scatter([true[0]],[true[1]],[true[2]],
                           marker="o",s=12,c="black",depthshade=False)
        ax.legend(loc="upper left",fontsize=7,framealpha=.85)

        side=fig.add_axes([.73,.12,.25,.78])
        side.axis("off")
        side.text(0,1.03,"RGB-D → Weltkoordinaten",
                  fontsize=13,weight="bold",va="top",transform=side.transAxes)
        side.text(0,.94,"3 zeitgleich gerenderte Kamerabilder",
                  fontsize=9,va="top",transform=side.transAxes)
        for i,name in enumerate(names):
            done=np.clip(progress*3-i,0,1)
            side.scatter([.04],[.83-i*.10],s=65,color=colors[i],
                         transform=side.transAxes)
            side.text(.13,.83-i*.10,
                      f"{name}\n{int(done*100)} % eingeblendet",
                      fontsize=8.5,va="center",transform=side.transAxes)
        if show_truth:
            side.text(0,.47,"AB HIER: Bewertung mit\nbekannter MuJoCo-Wahrheit",
                      fontsize=10,weight="bold",va="top",
                      transform=side.transAxes)
            side.text(0,.35,
                f"AprilTags: {report['tag_count_observed']}/"
                f"{report['tag_count_expected']}\n"
                f"Fehler Mittel: {report['mean_fixed_tag_position_error_mm']:.2f} mm\n"
                f"Fehler Maximum: {report['max_fixed_tag_position_error_mm']:.2f} mm\n"
                f"Tischhöhe: {report['tabletop_error_mm']:.2f} mm Fehler\n"
                "Rot x = Kamera | Schwarz o = Wahrheit",
                fontsize=9,va="top",linespacing=1.7,transform=side.transAxes)
        else:
            side.text(0,.47,
                "Punktwolke zunächst NUR aus\n"
                "RGB-D-Pixeln und bekannter\n"
                "virtueller Kamerakalibrierung.\n\n"
                "Keine Geometrie aus dem\n"
                "MuJoCo-Maschinenmodell eingefügt.",
                fontsize=9,va="top",linespacing=1.6,
                transform=side.transAxes)
        side.text(0,.05,
            f"Einzelbild-Rekonstruktion | sichtbare Punkte: {total_shown:,}\n"
            "Nicht: neuer Raum-Scan pro Video-Frame",
            fontsize=8,va="bottom",transform=side.transAxes)
        fig.savefig(frame_path/f"scan_{k:04d}.png",dpi=105)
        plt.close(fig)
        img_count+=1
    ffmpeg=shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg required to export MP4 (also installed by Colab notebook)")
    film=target/"rgbd_room_reconstruction.mp4"
    subprocess.run([ffmpeg,"-y","-loglevel","error",
          "-framerate",str(fps),"-i",str(frame_path/"scan_%04d.png"),
          "-c:v","libx264","-pix_fmt","yuv420p",
          "-movflags","+faststart",str(film)],check=True)
    if film.stat().st_size<15000:
        raise RuntimeError("Video file too small/empty")
    result={"status":"PASS","frames":img_count,"fps":fps,
            "observed_point_count":len(points),
            "camera_names":names,"source":"Rendered RGB-D pixels ONLY",
            "camera_transforms":"Assumed from simulated calibration",
            "simulation_truth_used_only_in_final_visual_comparison":True,
            "video_is_progressive_reveal_of_one_static_scan":True,
            "video_name":film.name,"video_bytes":film.stat().st_size}
    (target/"rgbd_reconstruction_video_summary.json").write_text(
        json.dumps(result,indent=2),encoding="utf8")
    print("RGBD VIDEO:",json.dumps(result,indent=2),flush=True)
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--input",default="runs/room_reconstruction")
    p.add_argument("--output")
    p.add_argument("--frames",type=int,default=36)
    p.add_argument("--fps",type=int,default=8)
    p.add_argument("--max-points-per-camera",type=int,default=2000)
    a=p.parse_args()
    render_video(a.input,a.output,a.fps,a.frames,a.max_points_per_camera)


if __name__=="__main__":
    main()
