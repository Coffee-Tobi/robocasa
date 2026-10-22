"""RoboCasa-inspired espresso fixture state gates for our MuJoCo workcell.

Original concept: robocasa/models/fixtures/coffee_machine.py
https://github.com/robocasa/robocasa/blob/main/robocasa/models/fixtures/coffee_machine.py
Copyright (c) 2026 RoboCasa Team; MIT License. Adapted with new code and
stricter espresso-portafilter workflow conditions; see THIRD_PARTY.md.

What is REAL here: geometric pose and Boolean state logic. What isn't:
portafilter mechanical lock, pressure, 9bar extraction, contact from MuJoCo
or an actual gripper button press. Explicit contact and lock confirmations
must arrive from qualified detectors. Demo signals MUST be named *_synthetic.
"""
from __future__ import annotations
from dataclasses import dataclass,field
from math import hypot,isfinite
from typing import Optional

@dataclass(frozen=True)
class CoffeeMachineGeometry:
    # Metres, from barista_cell.build_cell_scene:
    # espresso body world=(.49,.43,.82), brewhead local=(-.05,-.18,.27).
    outlet_xyz_m: tuple[float,float,float]=(.440,.250,1.090)
    cup_radius_m: float=.040
    allowable_xy_offset_m: float=.055
    # The existing rb_plastic_cup free body is anchored at its BASE, with rim +0.107m.
    # An empty cup rim must be below the brew spout without an excessive gap.
    min_nozzle_to_rim_m: float=.035
    max_nozzle_to_rim_m: float=.230

@dataclass
class CoffeeMachineState:
    """Gated virtual extraction; no contact is inferred from a proximity alone."""
    geometry: CoffeeMachineGeometry=field(default_factory=CoffeeMachineGeometry)
    brew_on: bool=False
    extraction_started: bool=False
    extraction_complete: bool=False
    extraction_seconds: float=0.
    flags: list[str]=field(default_factory=list)

    def check_cup(self,cup_centre_xyz_m,cup_half_height_m=.107):
        if cup_centre_xyz_m is None:
            return False,"CUP_MISSING"
        pos=tuple(float(x) for x in cup_centre_xyz_m)
        if len(pos)!=3 or not all(isfinite(x) for x in pos):
            return False,"INVALID_CUP_POSE"
        outlet=self.geometry.outlet_xyz_m
        xy=hypot(pos[0]-outlet[0],pos[1]-outlet[1])
        drop=outlet[2]-(pos[2]+cup_half_height_m)
        if xy>self.geometry.allowable_xy_offset_m:
            return False,"CUP_MISALIGNED"
        if not self.geometry.min_nozzle_to_rim_m<=drop<=self.geometry.max_nozzle_to_rim_m:
            return False,"CUP_RIM_WRONG_HEIGHT"
        return True,"CUP_UNDER_NOZZLE"

    def command(self,cup_centre_xyz_m,*,button_contact_confirmed=False,
                portafilter_lock_confirmed=False,pressure_ok=False,
                calibration_valid=False,synthetic_button=False):
        """Returns explicit status and refuses brewing if essential checks fail.

        A SIMULATED user-triggered button is allowed for storyboards only:
        it can NEVER be confused with a measured button contact.
        """
        if synthetic_button and button_contact_confirmed:
            return {"state":"INVALID_CONTACT_PROVENANCE",
                    "brew_on":False}
        okay,reason=self.check_cup(cup_centre_xyz_m)
        if not okay:
            self.flags.append(reason)
            self.brew_on=False
            return {"state":reason,"brew_on":False}
        if not calibration_valid:
            self.flags.append("UNCALIBRATED_MACHINE")
            self.brew_on=False
            return {"state":"UNCALIBRATED_MACHINE","brew_on":False}
        if not portafilter_lock_confirmed:
            self.brew_on=False
            return {"state":"PORTAFILTER_NOT_LOCKED","brew_on":False}
        if not pressure_ok:
            self.brew_on=False
            return {"state":"PRESSURE_NOT_VERIFIED","brew_on":False}
        if not button_contact_confirmed:
            self.brew_on=False
            return {"state":"NO_CONFIRMED_BUTTON_PRESS","brew_on":False}
        self.brew_on=True
        self.extraction_started=True
        return {"state":"BREWING_WITH_VERIFIED_FLAGS",
                "brew_on":True}

    def demo_brew_step(self,cup_centre_xyz_m):
        """Storyboard only: position gate WITHOUT fake lock / pressure results."""
        valid,reason=self.check_cup(cup_centre_xyz_m)
        self.brew_on=False
        self.extraction_started=False
        return {"state":"CUP_PLACEMENT_OK_BUT_NO_REAL_BREW" if valid else reason,
                "virtual_coffee_disc_may_be_drawn":bool(valid),
                "actual_brewing_allowed":False,
                "portafilter_locked":False,
                "button_contact_measured":False,
                "cup_geometry_verified":bool(valid)}
