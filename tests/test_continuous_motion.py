"""Synthetic pure-python tests; no hidden MuJoCo pose is used."""
import numpy as np
from scipy.spatial.transform import Rotation

from robobarista.continuous_motion import (
    fit_continuous_6d, _ruckig_sections, MAX_VEL, MAX_ACC, MAX_JERK
)


def test_continuous_se3_spline_smooth_and_measured_times():
    t=np.linspace(0,2,31)
    rows=[]
    for x in t:
        rows.append(dict(valid=True,time_s=float(x),
            position_m=[.2+.018*np.sin(x),-.4+.012*x,1.25+.008*np.cos(x)],
            rotation_xyzw=Rotation.from_euler('y',7*x,degrees=True).as_quat().tolist()))
    f=fit_continuous_6d(rows,.25)
    sampled=f(np.linspace(0,2,201))
    assert sampled['position_m'].shape==(201,3)
    assert sampled['rotation_xyzw'].shape==(201,4)
    assert np.isfinite(sampled['translation_velocity_m_s']).all()
    assert np.isfinite(sampled['angular_velocity_rad_s']).all()
    assert np.max(np.linalg.norm(sampled['position_m'],axis=1))<2


def test_local_ruckig_preserves_nonzero_velocity_at_internal_knots():
    t=np.linspace(0,2,11)
    # Smooth monotone-ish six-joint synthetic motion.
    q=np.tile(np.linspace(0,.07,11)[:,None],(1,6))
    q[:,1]*=.6
    q[:,2]*=-.4
    samples,segments=_ruckig_sections(t,q,velocity_scale=3,dt=.02)
    assert len(segments)==10
    assert len(samples)>50
    assert samples[0]['time_s']==0
    assert segments[0]['end_speed_norm']>0
    assert segments[1]['start_speed_norm']>0
    assert abs(segments[0]['end_speed_norm']-
               segments[1]['start_speed_norm'])<1e-9
    for i,key in enumerate(['velocity_rad_s','acceleration_rad_s2']):
        values=np.max(np.abs(np.asarray([s[key] for s in samples])),axis=0)
        limit=(MAX_VEL if i==0 else MAX_ACC)
        assert np.all(values<limit+1e-4)
    assert samples[-1]['time_s']>t[-1]
