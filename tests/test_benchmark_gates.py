"""Gate regression tests: preserve full-scene FAIL while reporting pitcher PASS."""
from robobarista.benchmark_gates import evaluate_side


def make_row(trial, label, spout=0.1, translation=0.1, cameras=3, measured=True):
    return dict(trial=trial, object=label, failed=not measured,
                cameras_seen=cameras, converged=measured,
                spout_tip_error_mm=spout if label=='pitcher' and measured else None,
                translation_mm=translation if measured else None,
                rotation_deg=0.04 if measured else None)


def test_pitcher_can_pass_when_unrelated_object_fails():
    rows=[make_row(n,'pitcher',spout=.055) for n in range(3)]
    rows.extend(make_row(n,'cup',translation=7) for n in range(3))
    report=evaluate_side(rows)
    assert report['pitcher_status']=='PASS'
    assert report['scene_status']=='FAIL'
    assert not report['pitcher_failures']
    assert len(report['other_failures'])==3
    assert all('translation_mm' in f['reasons'][0] for f in report['other_failures'])


def test_missing_pitcher_detection_is_not_a_pass():
    rows=[make_row(n,'pitcher') for n in range(3)]
    rows[1]['failed']=True
    rows[1]['spout_tip_error_mm']=None
    report=evaluate_side(rows)
    assert report['pitcher_status']=='FAIL'
    assert any(f['trial']==1 for f in report['pitcher_failures'])


def test_pitcher_fail_on_not_enough_cameras():
    rows=[make_row(n,'pitcher',cameras=(1 if n==1 else 3)) for n in range(3)]
    result=evaluate_side(rows)
    assert result['pitcher_status']=='FAIL'
    assert result['scene_status']=='FAIL'


def test_all_passing_and_wrong_spout():
    rows=[make_row(n,'pitcher') for n in range(3)]
    rows.extend(make_row(n,'cup') for n in range(3))
    assert evaluate_side(rows)['scene_status']=='PASS'
    rows[2]['spout_tip_error_mm']=.55
    result=evaluate_side(rows)
    assert result['pitcher_status']=='FAIL'
    assert result['scene_status']=='FAIL'
