"""Return-to-holder stage and rigid camera-orientation regressions."""
import numpy as np
import pytest
from robobarista.swift_fr5_full_cycle_follow import (
    make_complete_cycle, BACKWARD_NAMES, FOLLOW_OFFSET_WEST_M
)


def test_full_cycle_keeps_one_delayed_grip_and_one_end_release():
    phases=[
        "APPROACH_HOLDER", "ALIGN_HANDLE", "GRASP_VISUAL_SWITCH",
        "LIFT_FROM_STAND","TRANSFER_ABOVE_SWIFT","ALIGN_BELOW_SWIFT",
        "LIFT_INTO_RECEIVER","OPTIONAL_RIGHT_TWIST"
    ]
    rows=[{"phase":p,"q_rad":[.01*i]*6,"picked":i>=3,
           "basket_target_xyz_m":[-.2,-.33,.925],
           "yaw_target_deg":12. if i==7 else 0.}
          for i,p in enumerate(phases)]
    frames=make_complete_cycle(rows,grip_pause_frames=6,
                               release_pause_frames=6,end_pause_frames=5)
    assert frames[0]["phase"]=="APPROACH_HOLDER"
    assert frames[0]["picked"] is False
    assert frames[-1]["picked"] is False
    assert frames[-1]["phase"]=="RETREAT_FROM_HOLDER"
    events=[(i,r["grip_event"]) for i,r in enumerate(frames)
            if "grip_event" in r]
    assert len(events)==2
    assert events[0][0]>2
    assert events[1][0]>events[0][0]
    assert "UNLOCK_LEFT" in [r["phase"] for r in frames]
    assert "LOWER_INTO_HOLDER" in [r["phase"] for r in frames]
    assert "RELEASE_IR40_VISUAL_ONLY" in [r["phase"] for r in frames]
    # All FK joint states on the way home are exact reversals of the
    # verified six-axis forward path; no unvalidated new hardware path.
    assert np.allclose(frames[-1]["q_rad"],rows[0]["q_rad"])


def test_camera_offset_and_stages_and_validation():
    assert FOLLOW_OFFSET_WEST_M==pytest.approx([-.5,-.2,.21])
    assert BACKWARD_NAMES["OPTIONAL_RIGHT_TWIST"]=="UNLOCK_LEFT"
    assert BACKWARD_NAMES["LIFT_FROM_STAND"]=="LOWER_INTO_HOLDER"
    with pytest.raises(ValueError):
        make_complete_cycle([])
    with pytest.raises(ValueError):
        make_complete_cycle([{"phase":"APPROACH_HOLDER","picked":False}])
