"""MoveIt2 planning-scene bridge for the nominal MuJoCo coffee workcell.

This exports only axis-aligned machine keep-outs as CollisionObjects. It does
NOT execute a trajectory or connect to a FAIRINO controller. The source is
a conceptual virtual workcell, NOT an as-built calibrated physical cell.
Do not use the published primitive boxes as a safety-rated barrier.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from .workcell_constraints import KEEP_OUTS


def collision_objects(frame_id="world", extra_clearance_m=0.0):
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("Provide a named MoveIt frame")
    if not 0 <= extra_clearance_m <= .05:
        raise ValueError("Clearance must be 0..50 mm")
    result = []
    for name, item in KEEP_OUTS.items():
        lengths = [2.0 * (half + extra_clearance_m)
                   for half in item.half_extents_m]
        result.append({
            "id": "robobarista_keepout_" + name,
            "frame_id": frame_id,
            "shape": "BOX",
            "dimensions_m": lengths,
            "position_m": list(item.center_m),
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "source": "nominal MuJoCo workcell keep-out"
        })
    return result


def export_scene(frame_id="world", clearance_m=.01):
    return {
        "status": "PLANNING_SCENE_EXPORT_ONLY",
        "objects": collision_objects(frame_id, clearance_m),
        "frame_id": frame_id,
        "hardware_actuation_allowed": False,
        "real_physical_fixtures_calibrated": False,
        "robot_self_collision_validated": False,
        "source": "robobarista.workcell_constraints.KEEP_OUTS",
        "note": "Published collision boxes are a planning aid; validate TF, meshes, full FR5/IR40 collision geometry and real CAD before any actual execution."
    }


def publish_to_ros2(scene):
    """Explicit planning-scene publication; NO robot command or action client.

    Called only with --publish-to-planning-scene and installed ROS2 humble.
    Requires a matching 'world' TF and a running MoveIt PlanningSceneMonitor.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
    except ImportError as e:
        raise RuntimeError(
            "ROS2/MoveIt2 not found: use Ubuntu 22.04 + ROS2 Humble "
            "and official FAIRINO fake-hardware MoveIt2 configuration") from e
    rclpy.init()
    node = Node("robobarista_nominal_scene_publisher")
    publisher = node.create_publisher(CollisionObject, "/collision_object", 10)
    try:
        import time
        end = time.monotonic() + 5.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.2)
        if publisher.get_subscription_count() < 1:
            raise RuntimeError("MoveIt /collision_object subscriber absent; no publish")
        for row in scene["objects"]:
            msg = CollisionObject()
            msg.id = row["id"]
            msg.header.frame_id = row["frame_id"]
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = row["dimensions_m"]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = row["position_m"]
            pose.orientation.w = 1.0
            msg.primitives.append(box)
            msg.primitive_poses.append(pose)
            msg.operation = CollisionObject.ADD
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.05)
        node.get_logger().info(
            f"Published {len(scene['objects'])} nominal collision boxes; check MoveIt RViz")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--output", default="runs/moveit2/nominal_collision_scene.json")
    a.add_argument("--frame", default="world")
    a.add_argument("--clearance-mm", type=float, default=10.)
    a.add_argument("--publish-to-planning-scene", action="store_true")
    args = a.parse_args()
    scene = export_scene(args.frame, args.clearance_mm / 1000.)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(scene, indent=2))
    if args.publish_to_planning_scene:
        publish_to_ros2(scene)
    print(json.dumps({key: val for key, val in scene.items() if key != "objects"},
                     indent=2))


if __name__ == "__main__":
    main()
