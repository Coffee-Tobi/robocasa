"""Research-based end-effector candidate profiles for virtual FR5 studies.

These profiles are not mounted hardware, nor do they claim the FAIRINO FR5
flange fits a vendor gripper without an adapter. The existing seven coffee
stations and SAFE trajectory benchmarks are unchanged.

Library models:
- handle: google-deepmind/mujoco_menagerie robotiq_2f85_v4/2f85.xml
          OR robosuite.models.grippers.Robotiq85Gripper
- cup: robosuite.models.grippers.Robotiq140Gripper
       wide soft contact fingertips must be designed / validated.
This is selection metadata: simulation model loading, tool-changer hardware,
collisions, real contact and gripping are NOT yet implemented.
"""
from dataclasses import dataclass,asdict

@dataclass(frozen=True)
class ToolHead:
    name: str
    grasped_objects: tuple[str,...]
    library: str
    library_identifier: str
    manufacturer_nominal_opening_mm: int
    finger_style: str
    finger_liners: str
    preferred_contact: str
    needs_fr5_mount_calibration: bool=True
    model_integrated_in_fr5: bool=False
    physically_grasped_in_simulation: bool=False

TOOL_HEADS={
    "handle_gripper":ToolHead(
        name="handle_gripper",
        grasped_objects=("milk_pitcher_handle","portafilter_handle"),
        library="MuJoCo Menagerie / robosuite",
        library_identifier="robotiq_2f85_v4/2f85.xml | Robotiq85Gripper",
        manufacturer_nominal_opening_mm=85,
        finger_style="slim opposed fingertips, clearance through both handles",
        finger_liners="replaceable low-slip compliant pads",
        preferred_contact="grasp inside/around robust handle; avoid hot bowl"
    ),
    "plastic_cup_gripper":ToolHead(
        name="plastic_cup_gripper",
        grasped_objects=("empty_or_filled_plastic_cup_body",),
        library="robosuite",
        library_identifier="Robotiq140Gripper",
        manufacturer_nominal_opening_mm=140,
        finger_style="wide shallow-concave symmetric cup contact, no sharp tips",
        finger_liners="large-area food-compatible soft pads (design only)",
        preferred_contact="cup external wall; low-force limit pending crush tests"
    )
}

WORKFLOW_TOOLS={
    "portafilter_pickup":"handle_gripper",
    "grind_tamp":"handle_gripper",
    "insert_grouphead":"handle_gripper",
    "place_cup":"plastic_cup_gripper",
    "brew":None,
    "pitcher_pickup":"handle_gripper",
    "dose":"handle_gripper",
    "steam":"handle_gripper",
    "cup_to_pour":"plastic_cup_gripper",
    "pour":"handle_gripper",
    "rinse":"handle_gripper",
}

def export_profiles():
    return {
        "heads":{key:asdict(value) for key,value in TOOL_HEADS.items()},
        "workflow_tool_selection":WORKFLOW_TOOLS.copy(),
        "two_head_design":True,
        "automatic_tool_changer_simulated":False,
        "any_grip_physics_simulated":False,
        "real_robot_flange_verified":False,
        "next_checks":[
            "measure exact handle opening/diameter of pitcher and portafilter",
            "measure plastic cup body diameter, stiffness, mass, wet friction",
            "CAD transform between FR5 wrist flange, adapter, jaw contact and TCP",
            "import true gripper MJCF including actuators and collision geometries",
            "test grasp release and contact forces in MuJoCo before object carries",
            "recheck all safe paths with mounted gripper/link swept geometry",
            "validate physical quick tool changer or use interchangeable fingertips"
        ]
    }
