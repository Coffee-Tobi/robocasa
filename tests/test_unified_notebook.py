"""Structural smoke tests for the single user-facing all-in-one Colab."""
from pathlib import Path
import json


def test_one_colab_includes_all_four_verified_experiments():
    nb=json.loads((Path(__file__).resolve().parents[1]/
                   'COLAB_ROBOBARISTA.ipynb').read_text())
    assert nb['nbformat']==4
    sources=[]
    for cell in nb['cells']:
        code=''.join(cell['source'])
        if cell['cell_type']=='code':
            compile(code,cell['id'],'exec')
        sources.append(code)
    all_code='\n'.join(sources)
    for job in ('robobarista.cell_demo',
                'robobarista.safe_station_staging_demo',
                'robobarista.camera_joint_replay',
                'robobarista.motor_follow',
                'robobarista.continuous_motion'):
        assert job in all_code
    assert 'camera_vs_fr5_side_by_side' in all_code
    assert "'--fps','15'" in all_code
    assert "'--frames','31'" in all_code
    assert 'continuous_ruckig' in all_code
    assert 'RoboBarista_Gesamtpaket.zip' in all_code
    assert 'Video(filename=' not in all_code
    assert 'MUJOCO_GL' in all_code
