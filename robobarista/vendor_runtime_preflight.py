"""Read-only detection of FAIRINO SimMachine, official SDK, ROS2/MoveIt2.

Never opens a network socket, connects to the virtual or real controller,
calls Robot.RPC, publishes motion, or changes collision/safety settings.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
import os
import shutil
import subprocess

REQUIRED_SDK_METHODS=(
    "NewSplineStart", "NewSplinePoint", "NewSplineEnd", "MoveL",
    "GetActualTCPPose", "GetActualJointPosDegree", "GetRobotEmergencyStopState",
    "TCFToAllJoint", "FT_GetForceTorqueRCS"
)


def check_vendor_sdk_surface(robot_namespace):
    """Use class/static attributes only. Never instantiate or connect."""
    import inspect
    missing = [
        name for name in REQUIRED_SDK_METHODS
        if inspect.getattr_static(robot_namespace, name, None) is None
    ]
    return {"required_method_count": len(REQUIRED_SDK_METHODS),
            "missing_methods": missing,
            "offline_sdk_surface_complete": not missing,
            "vendor_controller_contacted": False}


def inspect_runtime(env=None, which=None, import_finder=None):
    env = dict(os.environ if env is None else env)
    which = shutil.which if which is None else which
    import_finder = importlib.util.find_spec if import_finder is None else import_finder
    ros2 = bool(which("ros2"))
    ros_distro = env.get("ROS_DISTRO")
    docker = bool(which("docker"))
    sdk = bool(import_finder("fairino"))
    # A configured address is NOT evidence a simulator/controller is running.
    simmachine_endpoint_configured = bool(env.get("ROBOBARISTA_SIMMACHINE_HOST"))
    return {
        "status": "READ_ONLY_DEPENDENCY_DISCOVERY",
        "ros2_cli_available": ros2,
        "ros_distro": ros_distro,
        "fairino_ros2_humble_ready_to_attempt_moveit_demo": ros2 and ros_distro == "humble",
        "docker_cli_available": docker,
        "fairino_python_sdk_importable": sdk,
        "simmachine_endpoint_configured": simmachine_endpoint_configured,
        "simmachine_started_and_reachable": None,
        "actual_fairino_new_spline_tested": False,
        "moveit2_robot_plan_executed": False,
        "hardware_or_simulator_contacted": False,
        "hardware_actuation_authorized": False,
        "next": "Install official SimMachine separately, confirm controller is a VIRTUAL test target, then validate NewSpline and feedback. For MoveIt use ROS2 Humble, official FR5 moveit config and fake hardware."
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", default="runs/fairino_runtime_preflight.json")
    args = p.parse_args()
    from pathlib import Path
    report = inspect_runtime()
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
