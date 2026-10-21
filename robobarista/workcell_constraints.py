"""Safety-oriented geometric workcell *planning* constraints, in metres.

Nominal docking frames are design specifications, NOT measured coordinates of
real equipment. Machine keep-outs apply to the wrist and any held payload.
An exception is restricted to a matching tool + task + tiny explicitly mapped
port; it never makes the rest of a machine collision-safe. This module is a
pre-filter only. A real FR5 requires link-level continuous swept geometry,
robot/manufacturer safety systems, contact sensing, speed/force limits,
calibrated fixture CAD and real hardware validation before operation.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

TRAVEL_CLEARANCE_M=0.030      # 30 mm planning margin outside equipment
DOCK_CLEARANCE_M=0.005        # 5 mm around target, never a contact guarantee
MAX_CARTESIAN_STEP_M=0.005   # ≤5 mm samples for translation
MAX_ROTATION_STEP_DEG=3.0
TARGET_POSITION_TOLERANCE_M=0.003  # desired accuracy criterion, not achieved
TARGET_ANGLE_TOLERANCE_DEG=2.0

# All coordinate frames use original MuJoCo world frame: x to the right,
# y towards back wall, z up. Identity angle aligns instrument local axes to
# world; no claim this matches a manufacturer's real bayonet geometry.
@dataclass(frozen=True)
class Dock:
    name: str
    station: str
    tool: str
    position_m: tuple[float,float,float]  # tool object origin, NOT FR5 wrist
    euler_xyz_deg: tuple[float,float,float]
    approach_xyz: tuple[float,float,float] # normalized direction in WORLD
    opening_radius_m: float
    contact_face: str
    # Task-specific distance into the machine's *nominal* body bounding box.
    # Important: aperture is a conceptual empty opening; detailed CAD TBD.
    permitted_depth_m: float = 0.012

DOCKS={
    # Grinder has built-in automatic tamper: one precise portafilter cradle
    # for both grind and tamp, not two separate docking machines.
    'grinder_portafilter':Dock(
        'grinder_portafilter','grinder','portafilter',
        (-.56,.122,.954),(0,0,0),(0,1,0),.052,'front',permitted_depth_m=.020),
    'espresso_portafilter':Dock(
        'espresso_portafilter','espresso','portafilter',
        (.440,.287,1.090),(0,0,0),(0,1,0),.090,'front',
        permitted_depth_m=.030),
    'espresso_cup':Dock(
        'espresso_cup','espresso','plastic_cup',
        (.440,.205,.882),(0,0,0),(0,1,0),.055,'front'),
    'espresso_steam_pitcher':Dock(
        'espresso_steam_pitcher','espresso','pitcher',
        (.653,.222,.871),(0,0,0),(0,1,0),.058,'front'),
    'milk_dispenser_pitcher':Dock(
        'milk_dispenser_pitcher','milk_dispenser','pitcher',
        (.575,-.500,.841),(0,0,0),(0,1,0),.150,'front'),
    'pitcher_park':Dock(
        'pitcher_park','pitcher_stand','pitcher',
        (-.150,-.550,.834),(0,0,0),(0,0,-1),.060,'top'),
    'pour_cup':Dock(
        'pour_cup','pour','plastic_cup',
        (.350,-.370,.823),(0,0,0),(0,0,-1),.055,'top'),
}

# Solid housing AABBs (machine head/side guards). They exclude actual open
# docking areas, described above in DOCKS. Separate physical collision shapes
# are also in MuJoCo. CAD and true port geometry must later supersede these.
@dataclass(frozen=True)
class Keepout:
    name: str
    center_m: tuple[float,float,float]
    half_extents_m: tuple[float,float,float]
    kind: str='machine'

KEEP_OUTS={
    'grinder':Keepout('grinder',(-.560,.250,.960),(.092,.090,.140)),
    'espresso':Keepout('espresso',(.490,.485,1.055),(.220,.170,.225)),
    'milk_dispenser':Keepout('milk_dispenser',(.600,-.340,.980),(.105,.110,.160)),
    # Small machine surfaces also matter if carrying the kettle.
    'grinder_hopper':Keepout('grinder_hopper',(-.560,.250,1.160),(.075,.075,.085)),
    'rinser':Keepout('rinser',(.070,.590,.855),(.135,.095,.025)),
}

# Approximate sphere decomposition of full carried payload including handle
# and spout, not just its nominal tool center. Tool frame = object's base.
PAYLOAD_SPHERES={
    'pitcher':[
        ((0,0,.025),.050),((0,0,.070),.052),((0,0,.116),.052),
        ((-.062,0,.120),.024),((.085,0,.075),.020)
    ],
    'portafilter':[
        ((0,0,0),.052),((.065,0,0),.030),((.125,0,0),.021),
        ((.18,0,0),.016),
    ],
    'plastic_cup':[
        ((0,0,.025),.042),((0,0,.075),.047),((0,0,.104),.049),
    ],
    'bare_tcp':[((0,0,0),.025)],
}


def rotation_matrix(euler_xyz_deg):
    return Rotation.from_euler('xyz',euler_xyz_deg,degrees=True).as_matrix()


def distance_to_box(point, box: Keepout):
    """Signed point-AABB distance: negative INSIDE; +outside distance."""
    local=np.abs(np.asarray(point,float)-np.asarray(box.center_m,float))
    local-=np.asarray(box.half_extents_m,float)
    outside=float(np.linalg.norm(np.maximum(local,0)))
    if np.all(local<=0):
        return float(np.max(local))
    return outside


def _allowed_docking_contact(centre, radius, box_name, dock_name, tool):
    """Only matching payload *in the exact configured docking aperture*.

    A sphere centre may be close to a nominal machine box *on its front face*.
    The exception is limited to a narrow front-facing slot, a named tool,
    and a small depth. Any other machine or rear-side approach is forbidden.
    """
    if not dock_name or dock_name not in DOCKS:
        return False
    d=DOCKS[dock_name]
    if d.tool!=tool or d.station!=box_name or d.contact_face!='front':
        return False
    p=np.asarray(centre,float)
    q=np.asarray(d.position_m,float)
    b=KEEP_OUTS[box_name]
    front=b.center_m[1]-b.half_extents_m[1]
    if not (front-.065 <= p[1] <= front+d.permitted_depth_m):
        return False
    # Tolerance applies to sphere centre in port's XY/XZ cross-section,
    # not to the whole arm. For payload components outside, normal rules.
    return (abs(p[0]-q[0])<=d.opening_radius_m
            and abs(p[2]-q[2])<=d.opening_radius_m)


def payload_check(position_m,rotation,tool='pitcher',dock=None,
                  clearance_m=TRAVEL_CLEARANCE_M):
    """Return violations for full carried tool envelope near solid housings.

    No magic safe flags: an authorized dock opens a precise aperture only.
    A same-device exception NEVER applies to other machine housings.
    AABB+spheres conservative approximation, separate from MuJoCo collisions.
    """
    if tool not in PAYLOAD_SPHERES:
        raise ValueError(f'Unknown payload: {tool}')
    p=np.asarray(position_m,float)
    R=np.asarray(rotation,float)
    if p.shape!=(3,) or R.shape!=(3,3):
        raise ValueError('Expected 3D tool pose')
    # A named dock must NEVER be a magic global exception: the precise
    # target pose (both position and angle) must be reached first.
    authorized_dock=(dock if dock and docking_pose_check(dock,tool,p,R)['within_goal']
                     else None)
    problems=[]
    for key,box in KEEP_OUTS.items():
        for index,(offset,radius) in enumerate(PAYLOAD_SPHERES[tool]):
            centre=p+R@np.asarray(offset,float)
            signed=distance_to_box(centre,box)-radius
            if signed >= clearance_m:
                continue
            if (_allowed_docking_contact(centre,radius,key,authorized_dock,tool)
                    and signed >= -max(0.,DOCKS[authorized_dock].permitted_depth_m)):
                continue
            problems.append(dict(machine=key,part=index,
                remaining_m=round(signed,6),
                required_clearance_m=clearance_m))
    return problems


def docking_pose_check(name,tool,position_m,rotation):
    """Strict target pose check (accuracy GOAL, not a real sensor guarantee)."""
    if name not in DOCKS:
        raise ValueError('Unknown docking pose: '+str(name))
    target=DOCKS[name]
    p=np.asarray(position_m,float)
    R=np.asarray(rotation,float)
    delta=float(np.linalg.norm(p-np.asarray(target.position_m)))
    dr=rotation_matrix(target.euler_xyz_deg)
    angle=float(np.rad2deg(Rotation.from_matrix(dr@R.T).magnitude()))
    return {'station':target.station,'tool':tool,
            'tool_matches':tool==target.tool,
            'position_error_m':delta,'orientation_error_deg':angle,
            'within_goal':(tool==target.tool
                and delta<=TARGET_POSITION_TOLERANCE_M
                and angle<=TARGET_ANGLE_TOLERANCE_DEG)}


def sample_motion(start_position,end_position,start_rotation,end_rotation,
                  max_step_m=MAX_CARTESIAN_STEP_M):
    """Dense spatial *precheck* samples, with quaternion SLERP orientation.

    More than 5mm Cartesian movement or 3deg rotation is subdivided.
    Does NOT bound joint-space sweeping between IK frames.
    """
    a=np.asarray(start_position,float)
    b=np.asarray(end_position,float)
    Ra=Rotation.from_matrix(start_rotation)
    Rb=Rotation.from_matrix(end_rotation)
    metres=float(np.linalg.norm(b-a))
    degrees=float(np.rad2deg((Ra.inv()*Rb).magnitude()))
    n=max(1,int(np.ceil(metres/max_step_m)),
          int(np.ceil(degrees/MAX_ROTATION_STEP_DEG)))
    slerp=Slerp([0,1],Rotation.from_quat([Ra.as_quat(),Rb.as_quat()]))
    return [(a+(b-a)*t,slerp([t]).as_matrix()[0])
            for t in np.linspace(0,1,n+1)]


def validate_segment(start_position,end_position,start_rotation,end_rotation,
                     tool='pitcher',dock=None):
    """Fail closed if ANY checked intermediate payload pose violates keepouts."""
    samples=sample_motion(start_position,end_position,
                          start_rotation,end_rotation)
    problems=[]
    for i,(p,R) in enumerate(samples):
        # Only the final approach sample may invoke the allowed opening.
        # During travel the normal 30 mm equipment clearance still applies.
        terminal=(i==len(samples)-1)
        clearance=DOCK_CLEARANCE_M if terminal and dock else TRAVEL_CLEARANCE_M
        prob=payload_check(p,R,tool,dock=dock if terminal else None,
                           clearance_m=clearance)
        for item in prob:
            problems.append({'sample':i,'of':len(samples)-1,**item})
    return {'safe':not problems,'samples':len(samples),
            'violations':problems}


class UnsafeMotionError(ValueError):
    """Raised when a planner attempts to execute an unsafe payload segment."""


def require_safe_segment(*args,**kwargs):
    """Mandatory guard for future movement code, not mere diagnostic logging."""
    report=validate_segment(*args,**kwargs)
    if not report['safe']:
        first=report['violations'][0]
        raise UnsafeMotionError(
            f"Blocked tool trajectory: {first['machine']} at sample "
            f"{first['sample']}/{first['of']}; "
            f"clearance={first['remaining_m']*1000:.1f} mm")
    return report


def export_workspace():
    """JSON-compatible single-source-of-truth for Colab, CAD and robot planning."""
    from dataclasses import asdict
    return {
      'units':'m, degrees; nominal simulated design ONLY',
      'clearance_travel_m':TRAVEL_CLEARANCE_M,
      'clearance_docking_m':DOCK_CLEARANCE_M,
      'position_tolerance_goal_m':TARGET_POSITION_TOLERANCE_M,
      'orientation_tolerance_goal_deg':TARGET_ANGLE_TOLERANCE_DEG,
      'max_cartesian_sample_m':MAX_CARTESIAN_STEP_M,
      'keepouts':{k:asdict(v) for k,v in KEEP_OUTS.items()},
      'docks':{k:asdict(v) for k,v in DOCKS.items()},
      'payload_spheres':{k:[{'local_center_m':p,'radius_m':r}
                           for p,r in values]
                         for k,values in PAYLOAD_SPHERES.items()},
      'limitations':'Conservative rigid AABB+sphere keepouts, not complete link-level physical collision detection; not suitable as real-robot safety controller.'
    }
