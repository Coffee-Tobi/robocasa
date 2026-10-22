"""Preflight third-party espresso CAD before EVER swapping MuJoCo collision meshes.

A GitHub filename ending .step/.stl does not imply it contains usable geometry.
The 2026 public espressor_ansamblu_complet.step contains HEADER + EMPTY DATA.
The corresponding .stl contains ONLY ONE triangle.
These must be rejected despite the repository MIT license.

Validate geometry, measured dimensions, collision simplification and license
before integrating. Source file bytes are never re-licensed or shipped here.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import hashlib,math,re


@dataclass(frozen=True)
class GeometricAcceptance:
    valid:bool
    reason:str
    triangles_or_faces:int
    spans_m:tuple[float,float,float]
    sha256:str


def validate_ascii_stl(data:bytes,kind="espresso_machine"):
    """Only accept plausible nondegenerate nontrivial ASCII CAD meshes.

    STL in millimetres is converted to metres. Binary STL must be converted
    explicitly by the importer; never assume raw XYZ represents metres.
    This intentionally checks only geometric plausibility, NOT safety,
    manufacturability, signed collision models or origin alignment.
    """
    if kind not in {"espresso_machine","portafilter","grinder"}:
        raise ValueError("Unknown asset kind")
    digest=hashlib.sha256(data).hexdigest()
    try:text=data.decode("ascii")
    except UnicodeDecodeError:
        return GeometricAcceptance(False,"BINARY_STL_CONVERSION_REQUIRED",0,(0,0,0),digest)
    tris=len(re.findall(r"\bfacet\s+normal\b",text))
    xyz=[tuple(map(float,match)) for match in re.findall(
        r"\bvertex\s+(-?[0-9.eE+]+)\s+(-?[0-9.eE+]+)\s+(-?[0-9.eE+]+)",
        text)]
    if not xyz or len(xyz)!=3*tris or not all(math.isfinite(v) for p in xyz for v in p):
        return GeometricAcceptance(False,"BROKEN_STL_TRIANGLES",tris,(0,0,0),digest)
    spans=tuple((max(p[i] for p in xyz)-min(p[i] for p in xyz))/1000 for i in range(3))
    minimum={"espresso_machine":(0.12,0.12,0.15),
             "portafilter":(0.04,0.04,0.02),
             "grinder":(0.08,0.08,0.12)}[kind]
    if tris<32:
        return GeometricAcceptance(False,"MESH_TOO_FEW_TRIANGLES",tris,spans,digest)
    if any(v<lo for v,lo in zip(spans,minimum)):
        return GeometricAcceptance(False,"FLAT_OR_WRONG_SCALE",tris,spans,digest)
    if any(v>1.6 for v in spans):
        return GeometricAcceptance(False,"IMPLAUSIBLE_DIMENSIONS",tris,spans,digest)
    return GeometricAcceptance(True,"GEOMETRY_PLAUSIBLE_NOT_COLLISION_CERTIFIED",tris,spans,digest)


def validate_step(data:bytes):
    digest=hashlib.sha256(data).hexdigest()
    text=data.decode("latin-1")
    face_count=len(re.findall(r"\bADVANCED_FACE\s*\(",text))
    brep=len(re.findall(r"\b(MANIFOLD_SOLID_BREP|CLOSED_SHELL|OPEN_SHELL)\s*\(",text))
    if "ISO-10303-21;" not in text:
        reason="NOT_STEP_FILE"
    elif face_count<12 or brep<1:
        reason="EMPTY_OR_NONMANIFOLD_STEP"
    else:
        reason="STEP_TOPOLOGY_PRESENT_REQUIRES_TESSELLATION"
    return GeometricAcceptance(reason.startswith("STEP_TOPOLOGY"),reason,face_count,
                               (0,0,0),digest)


def validate_path(path,kind="espresso_machine"):
    file=Path(path)
    if file.suffix.lower() in (".stp",".step"):
        return validate_step(file.read_bytes())
    if file.suffix.lower()==".stl":
        return validate_ascii_stl(file.read_bytes(),kind=kind)
    raise ValueError("Only STL or STEP CAD allowed after format validation")
