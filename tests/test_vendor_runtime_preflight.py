"""No FAIRINO network calls are made by runtime discovery and SDK inspection."""
from robobarista.vendor_runtime_preflight import (
    inspect_runtime, check_vendor_sdk_surface, REQUIRED_SDK_METHODS)


def test_missing_software_does_not_claim_controller_is_online():
    report = inspect_runtime(env={}, which=lambda name: None,
                             import_finder=lambda name: None)
    assert not report["fairino_ros2_humble_ready_to_attempt_moveit_demo"]
    assert not report["simmachine_endpoint_configured"]
    assert report["simmachine_started_and_reachable"] is None
    assert report["hardware_or_simulator_contacted"] is False
    assert not report["hardware_actuation_authorized"]


def test_fake_software_presence_is_not_actual_controller_contact():
    report = inspect_runtime(
        env={"ROS_DISTRO": "humble", "ROBOBARISTA_SIMMACHINE_HOST": "virtual-placeholder"},
        which=lambda name: "/usr/bin/" + name,
        import_finder=lambda name: object())
    assert report["fairino_ros2_humble_ready_to_attempt_moveit_demo"]
    assert report["simmachine_endpoint_configured"]
    assert report["simmachine_started_and_reachable"] is None
    assert not report["actual_fairino_new_spline_tested"]


def test_vendor_method_surface_static_without_instantiating_robot():
    cls = type("FakeNamespace", (), {
        name: (lambda *args, **kwargs: 0) for name in REQUIRED_SDK_METHODS})
    result=check_vendor_sdk_surface(cls)
    assert result["offline_sdk_surface_complete"]
    assert not result["vendor_controller_contacted"]
    delattr(cls, "MoveL")
    result=check_vendor_sdk_surface(cls)
    assert "MoveL" in result["missing_methods"]
