import numpy as np
from robobarista.barista_cell import STATIONS, TAG_ID, CAMERAS, TABLE_TOP, ROBOT_BASE


def test_artly_island_has_all_essential_workstations():
    assert len(STATIONS)==8
    assert {'grinder','tamper','portafilter','espresso','milk_dispenser',
            'pitcher','pour','rinser'}==set(STATIONS)
    assert len(CAMERAS)==3
    assert len(TAG_ID)==len(set(TAG_ID.values()))
    assert all(0 <= int(tid)<587 for tid in TAG_ID.values())


def test_every_station_fits_compact_island_plan():
    for name,(x,y,z) in STATIONS.items():
        assert abs(x)<0.90,name
        assert abs(y)<0.76,name
        assert z>TABLE_TOP,name
        # Reach SIZING heuristic only, actual FK/IK tested separately.
        assert np.linalg.norm(np.array((x,y))-np.array(ROBOT_BASE[:2]))<.83


def test_spatially_separated_handling_areas():
    def flat_gap(a,b):
        return np.linalg.norm(np.subtract(STATIONS[a][:2],STATIONS[b][:2]))
    assert flat_gap('grinder','espresso')>.65
    assert flat_gap('pitcher','pour')>.20
    assert flat_gap('pitcher','tamper')>.85
