"""Physically plausible AprilTag placements for the *virtual* barista cell.

Profiles: legacy = validated oversized optical diagnostic targets; compact =
~40-54 mm flat printed plaques firmly attached to jug/handle/cup.
Marker poses MUST match the MuJoCo body transform exactly. Neither profile
asserts 0.5mm real hardware accuracy or practical printer durability.
"""
from __future__ import annotations
import os
from dataclasses import dataclass

PRINTED_ACTIVE_FRACTION=440./512.
PROFILE_ENV="ROBOBARISTA_TAG_PROFILE"

@dataclass(frozen=True)
class Mount:
    centre_m: tuple[float,float,float]
    euler_xyz_deg: tuple[float,float,float]
    printed_outer_m: float
    support: str

LEGACY_JUG={
    71:Mount((-.065,0.,.135),(0.,0.,0.),.090,"old overhead tag"),
    72:Mount((.085,0.,.145),(25.,0.,0.),.090,"old oblique overhead tag"),
    23:Mount((0.,-.058,.083),(90.,0.,0.),.082,"old side tag"),
}
COMPACT_JUG={
    # Top marker mounts to the *handle*, not directly in the milk volume.
    71:Mount((.079,0.,.130),(0.,0.,0.),.044,"rigid flat handle top clip"),
    # Side plaques are tangent to the cylinder; standoffs bridge curvature.
    72:Mount((0.,+.052,.081),(-90.,0.,0.),.054,"rear tangent plaque"),
    23:Mount((0.,-.052,.081),(90.,0.,0.),.054,"front tangent plaque"),
}
LEGACY_CUP=Mount((0.,-.091,.118),(0.,0.,0.),.090,
                 "old floating cup marker")
COMPACT_CUP=Mount((0.,-.051,.080),(90.,0.,0.),.036,
                  "front side rim clip on disposable cup")

def profile(name=None):
    value=(name if name is not None else os.environ.get(PROFILE_ENV,"legacy")).lower()
    if value not in ("legacy","compact"):
        raise ValueError("ROBOBARISTA_TAG_PROFILE must be 'legacy' or 'compact'")
    return value

def moving_jug_markers(name=None):
    return COMPACT_JUG if profile(name)=="compact" else LEGACY_JUG

def cup_marker(name=None):
    return COMPACT_CUP if profile(name)=="compact" else LEGACY_CUP

def active_edge_m(mount):
    return mount.printed_outer_m*PRINTED_ACTIVE_FRACTION
