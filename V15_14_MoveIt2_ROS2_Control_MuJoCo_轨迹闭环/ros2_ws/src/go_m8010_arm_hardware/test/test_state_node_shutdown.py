import importlib.util
from pathlib import Path
import signal
import sys
from types import ModuleType, SimpleNamespace
from unittest import mock

import pytest


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "go_m8010_arm_hardware"
    / "whole_arm_state_node.py"
)


def load_with_ros_stubs(events, ros_state):
    package_name = "state_node_shutdown_test_package"
    module_name = f"{package_name}.whole_arm_state_node"
    no_signal_handlers = object()

    package = ModuleType(package_name)
    package.__path__ = []
    state_model = ModuleType(f"{package_name}.state_model")
    state_model.GEAR_RATIO = 6.329999923706055
    state_model.JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
    state_model.MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
    state_model.MirrorSessionReferenceV1 = object
    state_model.PositionSpanVelocityObserver = object
    state_model.WorkerSupervisorStatusError = type(
        "WorkerSupervisorStatusError", (ValueError,), {}
    )
    state_model.parse_feedback_payload = lambda _payload, _receipt: ()
    state_model.unavailable_worker_control_status = lambda _reason: {}
    state_model.validate_worker_supervisor_status = lambda value, _now, _age: value

    rclpy = ModuleType("rclpy")

    def init(*, args=None, signal_handler_options=None):
        events.append(("rclpy.init", signal_handler_options))

    def ok():
        return ros_state["ok"]

    def spin_once(_node, timeout_sec=0.0):
        events.append(("rclpy.spin_once", timeout_sec))
        ros_state["on_spin"]()

    def shutdown():
        events.append("rclpy.shutdown")
        ros_state["ok"] = False

    rclpy.init = init
    rclpy.ok = ok
    rclpy.spin_once = spin_once
    rclpy.shutdown = shutdown

    node_module = ModuleType("rclpy.node")
    node_module.Node = object
    executors_module = ModuleType("rclpy.executors")
    executors_module.ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {}
    )
    signals_module = ModuleType("rclpy.signals")
    signals_module.SignalHandlerOptions = SimpleNamespace(NO=no_signal_handlers)

    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs_msg = ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = type("JointState", (), {})
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})

    stubs = {
        package_name: package,
        f"{package_name}.state_model": state_model,
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "rclpy.executors": executors_module,
        "rclpy.signals": signals_module,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    with mock.patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location(module_name, SOURCE)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    return module, no_signal_handlers


def configure_fake_signals(monkeypatch, module):
    installed_handlers = {}
    previous_handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    monkeypatch.setattr(
        module.signal, "getsignal", lambda signum: previous_handlers[signum]
    )
    monkeypatch.setattr(
        module.signal,
        "signal",
        lambda signum, handler: installed_handlers.__setitem__(signum, handler),
    )
    return installed_handlers, previous_handlers


def test_sigint_destroys_state_resources_before_rclpy_shutdown(monkeypatch):
    events = []
    ros_state = {"ok": True, "on_spin": lambda: None}
    module, no_signal_handlers = load_with_ros_stubs(events, ros_state)
    installed_handlers, previous_handlers = configure_fake_signals(monkeypatch, module)

    class FakeNode:
        @staticmethod
        def destroy_node():
            events.append("node.destroy")

    monkeypatch.setattr(module, "WholeArmStateNode", FakeNode)

    def send_sigint():
        installed_handlers[signal.SIGINT](signal.SIGINT, None)

    ros_state["on_spin"] = send_sigint
    module.main([])

    assert ("rclpy.init", no_signal_handlers) in events
    assert events.index("node.destroy") < events.index("rclpy.shutdown")
    assert installed_handlers[signal.SIGINT] is previous_handlers[signal.SIGINT]
    assert installed_handlers[signal.SIGTERM] is previous_handlers[signal.SIGTERM]


def test_destroy_exception_still_shuts_down_and_restores_handlers(monkeypatch):
    events = []
    ros_state = {"ok": True, "on_spin": lambda: None}
    module, _no_signal_handlers = load_with_ros_stubs(events, ros_state)
    installed_handlers, previous_handlers = configure_fake_signals(monkeypatch, module)

    class FailingNode:
        @staticmethod
        def destroy_node():
            events.append("node.destroy")
            raise RuntimeError("synthetic state resource close failure")

    monkeypatch.setattr(module, "WholeArmStateNode", FailingNode)

    def send_sigint():
        installed_handlers[signal.SIGINT](signal.SIGINT, None)

    ros_state["on_spin"] = send_sigint
    with pytest.raises(RuntimeError, match="synthetic state resource close failure"):
        module.main([])

    assert events.index("node.destroy") < events.index("rclpy.shutdown")
    assert installed_handlers[signal.SIGINT] is previous_handlers[signal.SIGINT]
    assert installed_handlers[signal.SIGTERM] is previous_handlers[signal.SIGTERM]
