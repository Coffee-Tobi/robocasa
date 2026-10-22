import numpy as np
from robobarista.precision_pour import SPOUT_LOCAL_M, _interpolator


def test_spout_offset_is_not_tcp():
    assert np.linalg.norm(SPOUT_LOCAL_M)>0.10
    assert np.isfinite(SPOUT_LOCAL_M).all()


def test_reference_interpolator_matches_state_velocity_and_accel():
    t=np.linspace(0,1,15)
    rows=[dict(time_s=float(x),
        q_rad=[.02*np.sin(np.pi*x)]*6,
        velocity_rad_s=[.02*np.pi*np.cos(np.pi*x)]*6)
        for x in t]
    f,end=_interpolator(rows)
    assert end==1.
    for i,tm in enumerate(t):
        q,v,a=f(tm)
        np.testing.assert_allclose(q,rows[i]['q_rad'],atol=1e-12)
        np.testing.assert_allclose(v,rows[i]['velocity_rad_s'],atol=1e-12)
        assert np.isfinite(a).all()
    # Feed-forward must include the derivative, not merely the positions.
    assert np.linalg.norm(f(.2)[1])>1e-4
