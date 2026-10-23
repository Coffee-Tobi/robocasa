"""Lightweight 2D milk-on-coffee SURROGATE for preview only.

A CPU semi-Lagrangian advection-diffusion illustration driven by an imported
relative pitcher/FLANGE path. NOT FluidLab, SPH/MPM, measured microfoam
viscosity, real crema hydrodynamics or scientifically predicted latte-art.
Milk-flow rate, cup-to-robot calibration and original flange-to-pitcher
transform remain unknown; the intensity and drawing scale are arbitrary.
Never use this visualization to authorize autonomous hot-liquid pouring.

The high-fidelity candidate remains the independent project
https://github.com/zhouxian/FluidLab (LatteArt-v0; legacy GPU/CUDA stack).
"""
from __future__ import annotations
import argparse
import json
import subprocess
from pathlib import Path
import numpy as np
from scipy.ndimage import gaussian_filter, map_coordinates
from PIL import Image,ImageDraw,ImageFont


class CoffeeSurfaceIllustration:
    def __init__(self, resolution=192):
        if not isinstance(resolution,int) or resolution<32 or resolution>512:
            raise ValueError("Resolution must be an integer between 32 and 512")
        self.n=resolution
        self.scalar=np.zeros((resolution,resolution),float)
        yy,xx=np.mgrid[:resolution,:resolution]
        self.x=xx.astype(float)
        self.y=yy.astype(float)
        self.center=(resolution-1)/2
        self.radius=np.hypot(xx-self.center,yy-self.center)
        self.mask=self.radius<.455*resolution

    def update(self, relative_xy_mm, illustrative_flow_fraction):
        xy=np.asarray(relative_xy_mm,float)
        if xy.shape!=(2,) or not np.isfinite(xy).all():
            raise ValueError("Finite relative x and y positions required")
        flow=float(illustrative_flow_fraction)
        if not np.isfinite(flow) or flow<0 or flow>1:
            raise ValueError("Illustrative flow must be within 0..1")
        dx=self.x-self.center
        dy=self.y-self.center
        # Arbitrarily illustrative radial expansion and gentle azimuthal curl.
        vx=.0075*dx-.016*dy
        vy=.0075*dy+.016*dx
        self.scalar=map_coordinates(self.scalar,
            [self.y-vy,self.x-vx],order=1,mode="constant",cval=0.0)
        self.scalar=gaussian_filter(self.scalar,sigma=.44)
        # Surface position normalization is VISUAL, not real cup calibration.
        tip_x=self.center+np.clip(xy[0]/60,-.8,.8)*self.n*.25
        tip_y=self.center+np.clip(xy[1]/60,-.8,.8)*self.n*.25
        injection=np.exp(-((self.x-tip_x)**2+(self.y-tip_y)**2)/(2*4.1**2))
        self.scalar=np.clip((self.scalar+.28*flow*injection)*self.mask,0,1)
        return self.scalar

    def rgb(self):
        f=np.power(self.scalar,.62)
        edge=np.clip((self.center*.92-self.radius)/3.,0,1)
        c=np.stack([66+146*f,35+167*f,21+153*f],axis=2)
        return np.uint8(np.clip(
            (1-edge[:,:,None])*np.array([15,24,31])+edge[:,:,None]*c,
            0,255))


def validate_pose_source(rows):
    if not isinstance(rows,list) or len(rows)<8:
        raise ValueError(">=8 pitcher-FK source samples required")
    ts=np.asarray([r["time_s"] for r in rows],float)
    xyz=np.asarray([r["position_m"] for r in rows],float)
    if (xyz.shape!=(len(rows),3) or not np.isfinite(xyz).all()
        or not np.isfinite(ts).all() or not np.all(np.diff(ts)>0)):
        raise ValueError("Invalid finite monotonic source pose trajectory")
    if ts[-1]-ts[0]>.0 and ts[-1]-ts[0]>120:
        raise ValueError("Unreasonably long segment")
    return ts,xyz


def make_proxy_video(source,output,fps=18):
    rows=json.loads(Path(source).read_text())
    ts,xyz=validate_pose_source(rows)
    if not isinstance(fps,int) or not 3<=fps<=60:
        raise ValueError("fps out of supported range")
    duration=float(ts[-1]-ts[0])
    n_frames=max(8,int(np.ceil(duration*fps)))
    dest=Path(output)
    dest.parent.mkdir(parents=True,exist_ok=True)
    n=512
    ff=["ffmpeg","-y","-loglevel","error","-f","rawvideo",
        "-vcodec","rawvideo","-s",f"{n}x{n}","-pix_fmt","rgb24",
        "-r",str(fps),"-i","-","-an","-c:v","libx264",
        "-preset","ultrafast","-crf","24","-pix_fmt","yuv420p",str(dest)]
    field=CoffeeSurfaceIllustration(192)
    proc=subprocess.Popen(ff,stdin=subprocess.PIPE)
    try:
        for i in range(n_frames):
            progress=i/max(1,n_frames-1)
            time=float(ts[0]+progress*duration)
            pos=np.asarray([np.interp(time,ts,xyz[:,k]) for k in range(3)])
            relative=(pos-xyz[0])*1000.0
            # Not measured milk flux; intentionally demonstrate assumptions.
            flow=(.12<progress<.89)*(.35+.5*np.sin(np.pi*progress)**2)
            field.update(relative[:2],flow)
            frame=Image.fromarray(field.rgb(),"RGB").resize(
                (n,n),Image.Resampling.BICUBIC)
            d=ImageDraw.Draw(frame)
            d.text((12,12),"2D MILK FLOW PROXY / NOT FLUIDLAB",
                   fill=(245,245,245),stroke_width=2,stroke_fill=(10,20,25))
            d.text((12,480),"Illustrative only / NOT real latte art",
                   fill=(255,205,160),stroke_width=2,stroke_fill=(10,20,25))
            proc.stdin.write(frame.tobytes())
    finally:
        if proc.stdin:proc.stdin.close()
        status=proc.wait()
    if status:
        raise RuntimeError(f"FFmpeg preview failed: {status}")
    meta={
        "model":"cpu_2d_advection_diffusion_illustration",
        "source":"Relative poses derived from joint FK, NOT original camera measurement",
        "source_poses":len(rows),
        "source_duration_s":duration,
        "original_realman_flange_to_pitcher_known":False,
        "flux_measured":False,
        "cup_geometry_calibrated":False,
        "physically_validated_fluidlab_execution":False,
        "real_latte_art_pattern_predicted":False,
        "robot_command_generated":False,
        "video":str(dest)}
    Path(str(dest)+".json").write_text(json.dumps(meta,indent=2))
    return meta


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--source",required=True,
                   help="RM65 URDF-FK poses, not original video camera observations")
    p.add_argument("--output",required=True)
    p.add_argument("--fps",type=int,default=18)
    a=p.parse_args()
    print(json.dumps(make_proxy_video(a.source,a.output,a.fps),indent=2))


if __name__=="__main__":
    main()
