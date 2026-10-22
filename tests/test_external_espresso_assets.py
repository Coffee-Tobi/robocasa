import pytest
from robobarista.external_asset_validator import (
    validate_ascii_stl,validate_step,validate_path)


def test_found_public_mit_step_stub_is_not_a_machine():
    # Exact representative header of the supposedly "complete" 2026 CAD
    # export: data section contains only comments, no shell/face topology.
    stub=b"""ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('AutoCAD 3D STEP Export - Espressor Automat de Cafea'),'2;1');
ENDSEC;
DATA;
/* Complete 3D Assembly STEP Geometry exchange file */
ENDSEC;
END-ISO-10303-21;"""
    a=validate_step(stub)
    assert not a.valid
    assert a.reason=="EMPTY_OR_NONMANIFOLD_STEP"


def test_found_public_mit_stl_is_one_triangle_only():
    stub=b"""solid espressor_ansamblu_complet
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 235 0 0
      vertex 235 290 0
    endloop
  endfacet
endsolid espressor_ansamblu_complet"""
    a=validate_ascii_stl(stub)
    assert not a.valid
    assert a.triangles_or_faces==1
    assert a.reason=="MESH_TOO_FEW_TRIANGLES"
    assert a.spans_m[2]==0


def test_plausible_nonzero_3d_mesh_only_passes_geometry_gate():
    vertices=[
        (0,0,0),(235,0,0),(235,290,0),(0,290,0),
        (0,0,340),(235,0,340),(235,290,340),(0,290,340)]
    faces=[(0,1,2),(0,2,3),(4,5,6),(4,6,7),
           (0,1,5),(0,5,4),(1,2,6),(1,6,5),
           (2,3,7),(2,7,6),(3,0,4),(3,4,7)]
    chunks=["solid test"]
    for face in faces*4:
        chunks+=[" facet normal 0 0 1"," outer loop"]
        chunks += [f" vertex {vertices[i][0]} {vertices[i][1]} {vertices[i][2]}"
                   for i in face]
        chunks+=[" endloop"," endfacet"]
    chunks+=["endsolid test"]
    res=validate_ascii_stl("\n".join(chunks).encode())
    assert res.valid
    assert res.spans_m==pytest.approx((.235,.290,.340))
    assert res.reason=="GEOMETRY_PLAUSIBLE_NOT_COLLISION_CERTIFIED"
