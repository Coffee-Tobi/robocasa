"""Adapter: our calibrated 3D spout data -> MIT p-sun 2D latte-fluid renderer.

Upstream https://github.com/p-sun/typescript-fluid-simulator
MIT (c) 2023 Paige Sun. We do not duplicate upstream code here;
the TS sidecar imports an external checkout of the unmodified upstream core.

This is a visually useful 2D Eulerian milk/crema *proxy*, NOT CFD validation,
fluid volume calibration, true pouring height/flow, or robot command feedback.
"""
from __future__ import annotations
import argparse,json,math
from pathlib import Path

SPOUT_TIP_LOCAL_M=(-.069,0.,.136)  # existing simplified spout CAD offset

def _rotate_xyzw(q,v):
    x,y,z,w=map(float,q)
    n=math.sqrt(x*x+y*y+z*z+w*w)
    if n<1e-9:raise ValueError("Invalid quaternion")
    x/=n;y/=n;z/=n;w/=n
    vx,vy,vz=v
    # v' = v + 2w(qxyz x v) + 2(qxyz x (qxyz x v))
    cx=y*vz-z*vy;cy=z*vx-x*vz;cz=x*vy-y*vx
    dx=y*cz-z*cy;dy=z*cx-x*cz;dz=x*cy-y*cx
    return (vx+2*(w*cx+dx),vy+2*(w*cy+dy),vz+2*(w*cz+dz))

def from_apriltag_capture(source,output,*,cup_radius_m=.045):
    rows=json.loads(Path(source).read_text())
    if not rows or len(rows)<3:raise ValueError("Need time-aligned camera poses")
    if not .020<=cup_radius_m<=.100:raise ValueError("Cup radius invalid")
    frame=[]
    for r in rows:
        if not r.get("valid") or r.get("visible_views",0)<2:
            raise ValueError("Camera frame invalid: no truth fill-in permitted")
        q=r["rotation_xyzw"]
        p=r["position_m"]
        tip=[float(p[i])+_rotate_xyzw(q,SPOUT_TIP_LOCAL_M)[i]
             for i in range(3)]
        frame.append({"t_s":float(r["time_s"]),"tip_xyz_m":tip,
                      "measured_camera_views":int(r["visible_views"])})
    if any(b["t_s"]<=a["t_s"] for a,b in zip(frame,frame[1:])):
        raise ValueError("Unsorted timestamps")
    # Only relative movements of the demonstrator can be projected until
    # physical cup->camera extrinsics are measured. No promise that the
    # virtual motion actually enters the cup at the right world xyz.
    zero=frame[0]["tip_xyz_m"]
    events=[]
    for f in frame:
        p=f["tip_xyz_m"]
        # Upstream latte radius is .42 in normalized scene coordinates.
        radius_scale=.42/cup_radius_m
        x=.5+(p[0]-zero[0])*radius_scale
        y=.5+(p[1]-zero[1])*radius_scale
        events.append({"t_s":f["t_s"],
            "norm_x":max(.08,min(.92,x)),
            "norm_y":max(.08,min(.92,y)),
            "relative_spout_height_m":p[2]-zero[2],
            "camera_views":f["measured_camera_views"],
            "tool":"Milk"})
    artifact={"schema":"robobarista-fluid2d-input-v1",
       "source":"camera-only pupil AprilTag RGB-D poses, not ground truth",
       "engine":"p-sun/typescript-fluid-simulator (MIT), loaded externally",
       "coupling":"normalized relative XY spout trajectory; z annotated only",
       "flow_rate_from_camera":False,
       "simulated_material":"stylized 2D milk surface, not actual milk CFD",
       "cup_radius_m_assumed":cup_radius_m,
       "events":events}
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(artifact,indent=2))
    return artifact

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--camera-measurements",required=True)
    p.add_argument("--out",default="runs/latte_fluid/trace.json")
    args=p.parse_args()
    r=from_apriltag_capture(args.camera_measurements,args.out)
    print(json.dumps({k:v for k,v in r.items() if k!="events"},indent=2),
          len(r["events"]),"camera poses")

if __name__=="__main__":main()
