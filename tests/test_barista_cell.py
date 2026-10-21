import numpy as np
import pytest

from robobarista.barista_cell import (
    STATIONS, TAG_ID, CAMERAS, TABLE_TOP, ROBOT_BASE
)
from robobarista.workcell_constraints import (
    DOCKS, KEEP_OUTS,TRAVEL_CLEARANCE_M,DOCK_CLEARANCE_M,
    TARGET_POSITION_TOLERANCE_M,
    distance_to_box, rotation_matrix, payload_check, docking_pose_check,
    sample_motion, validate_segment, require_safe_segment, UnsafeMotionError,
    export_workspace,
)


def test_integrated_grinder_tamper_and_all_workstations():
    assert len(STATIONS)==7
    assert {'grinder','portafilter','espresso','milk_dispenser',
            'pitcher','pour','rinser'}==set(STATIONS)
    assert 'tamper' not in STATIONS
    assert 'tamper' not in TAG_ID
    assert 62 not in TAG_ID.values()   # retired, never silently reused
    assert len(CAMERAS)==3
    assert len(TAG_ID)==len(set(TAG_ID.values()))
    assert all(0<=int(tid)<587 for tid in TAG_ID.values())


def test_each_station_within_compact_island():
    for name,(x,y,z) in STATIONS.items():
        assert abs(x)<.90,name
        assert abs(y)<.76,name
        assert z>TABLE_TOP,name
        assert np.linalg.norm(np.array([x,y])-ROBOT_BASE[:2])<.83


def test_docks_use_separate_tools_and_explicit_named_targets():
    assert set(DOCKS)=={
        'grinder_portafilter','espresso_portafilter','espresso_cup',
        'espresso_steam_pitcher','milk_dispenser_pitcher',
        'pitcher_park','pour_cup'
    }
    assert DOCKS['grinder_portafilter'].station=='grinder'
    assert DOCKS['espresso_portafilter'].station=='espresso'
    assert DOCKS['espresso_cup'].position_m!=DOCKS['espresso_portafilter'].position_m
    assert DOCKS['espresso_steam_pitcher'].position_m!=DOCKS['espresso_cup'].position_m
    assert DOCKS['milk_dispenser_pitcher'].station=='milk_dispenser'
    assert all(x.station in (set(STATIONS)|{'pitcher_stand'}) for x in DOCKS.values())


def test_nominal_dock_pose_requires_matching_tool_within_3mm_and_2degrees():
    dock=DOCKS['espresso_steam_pitcher']
    R=rotation_matrix(dock.euler_xyz_deg)
    assert docking_pose_check(dock.name,'pitcher',dock.position_m,R)['within_goal']
    assert not docking_pose_check(dock.name,'plastic_cup',dock.position_m,R)['within_goal']
    off=np.array(dock.position_m)+[0.0035,0,0]
    assert not docking_pose_check(dock.name,'pitcher',off,R)['within_goal']
    off_angle=rotation_matrix([0,0,2.1])
    assert not docking_pose_check(dock.name,'pitcher',dock.position_m,off_angle)['within_goal']


def test_pass_through_machine_blocked_including_carried_pitcher():
    forbidden=validate_segment([.49,-.55,1.10],[.49,.62,1.10],
                               np.eye(3),np.eye(3),tool='pitcher')
    assert not forbidden['safe']
    assert len(forbidden['violations'])>1
    assert any(v['machine']=='espresso' for v in forbidden['violations'])
    with pytest.raises(UnsafeMotionError,match='Blocked tool trajectory'):
        require_safe_segment([.49,-.55,1.10],[.49,.62,1.10],
                             np.eye(3),np.eye(3),tool='pitcher')


def test_magic_dock_cannot_exempt_other_tool_or_different_body():
    d=DOCKS['grinder_portafilter']
    R=rotation_matrix(d.euler_xyz_deg)
    wrong_tool=payload_check(d.position_m,R,'pitcher',dock=d.name,clearance_m=0)
    assert any(v['machine']=='grinder' for v in wrong_tool)
    # An attacker/untrusted planner passing a dock name at a remote position
    # does not open that port anywhere in the workcell.
    remote=np.array(d.position_m)+[0,0,.10]
    no_exemption=payload_check(remote,R,'portafilter',
                               dock=d.name,clearance_m=TRAVEL_CLEARANCE_M)
    default=payload_check(remote,R,'portafilter',
                          dock=None,clearance_m=TRAVEL_CLEARANCE_M)
    assert no_exemption==default


def test_continuous_spatial_sampling_and_export():
    steps=sample_motion([0,0,1.5],[.2,0,1.5],np.eye(3),np.eye(3))
    assert len(steps)>=41
    for (a,_),(b,_) in zip(steps,steps[1:]):
        assert np.linalg.norm(a-b)<=.00501
    workspace=export_workspace()
    assert workspace['clearance_travel_m']==pytest.approx(.03)
    assert workspace['clearance_docking_m']==pytest.approx(.005)
    assert workspace['position_tolerance_goal_m']==pytest.approx(.003)
    assert 'espresso_portafilter' in workspace['docks']
    assert 'pitcher' in workspace['payload_spheres']
