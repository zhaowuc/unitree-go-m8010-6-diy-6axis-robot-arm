import importlib.util
from pathlib import Path
import signal
import sys
from types import ModuleType, SimpleNamespace
from unittest import mock

import numpy  # Keep the real dependency outside temporary sys.modules patches.
import pytest


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "go_m8010_arm_hardware"
    / "mujoco_mirror_node.py"
)


def load_with_ros_stubs(events, ros_state):
    package_name = "mujoco_mirror_shutdown_test_package"
    module_name = f"{package_name}.mujoco_mirror_node"
    no_signal_handlers = object()

    package = ModuleType(package_name)
    package.__path__ = []
    state_model = ModuleType(f"{package_name}.state_model")
    state_model.JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
    state_model.MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")

    rclpy = ModuleType("rclpy")

    def init(*, args=None, signal_handler_options=None):
        events.append(("rclpy.init", signal_handler_options))

    def ok():
        return ros_state["ok"]

    def spin_once(_node, timeout_sec=0.0):
        events.append(("rclpy.spin_once", timeout_sec))

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
    std_msgs_msg.Float64MultiArray = type("Float64MultiArray", (), {})
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


def test_sigint_closes_viewer_before_ros_teardown(monkeypatch):
    events = []
    ros_state = {"ok": True}
    module, no_signal_handlers = load_with_ros_stubs(events, ros_state)
    installed_handlers = {}
    previous_handlers = {
        signal.SIGINT: object(),
        signal.SIGTERM: object(),
    }

    def fake_getsignal(signum):
        return previous_handlers[signum]

    def fake_signal(signum, handler):
        installed_handlers[signum] = handler

    class FakeNode:
        model = object()
        data = object()

        @staticmethod
        def get_parameter(_name):
            return SimpleNamespace(value=True)

        @staticmethod
        def destroy_node():
            events.append("node.destroy")

    class FakeViewer:
        signal_sent = False
        close_polls_remaining = 2

        @staticmethod
        def is_running():
            return True

        def sync(self):
            events.append("viewer.sync")
            if not self.signal_sent:
                self.signal_sent = True
                installed_handlers[signal.SIGINT](signal.SIGINT, None)

        @staticmethod
        def close():
            events.append("viewer.close")

        def _sim(self):
            if self.close_polls_remaining > 0:
                self.close_polls_remaining -= 1
                events.append("viewer.wait")
                return object()
            events.append("viewer.released")
            return None

    viewer_module = ModuleType("mujoco.viewer")
    viewer_module.launch_passive = lambda _model, _data: FakeViewer()
    mujoco_module = ModuleType("mujoco")
    mujoco_module.viewer = viewer_module

    monkeypatch.setattr(module, "WholeArmMujocoMirror", FakeNode)
    monkeypatch.setattr(module.signal, "getsignal", fake_getsignal)
    monkeypatch.setattr(module.signal, "signal", fake_signal)
    with mock.patch.dict(
        sys.modules,
        {"mujoco": mujoco_module, "mujoco.viewer": viewer_module},
    ):
        module.main([])

    assert ("rclpy.init", no_signal_handlers) in events
    assert events.index("viewer.close") < events.index("node.destroy")
    assert events.index("viewer.released") < events.index("node.destroy")
    assert events.index("node.destroy") < events.index("rclpy.shutdown")
    assert installed_handlers[signal.SIGINT] is previous_handlers[signal.SIGINT]
    assert installed_handlers[signal.SIGTERM] is previous_handlers[signal.SIGTERM]


def test_cleanup_exception_still_destroys_ros_and_restores_handlers(monkeypatch):
    events = []
    ros_state = {"ok": True}
    module, _no_signal_handlers = load_with_ros_stubs(events, ros_state)
    installed_handlers = {}
    previous_handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}

    class FakeNode:
        model = object()
        data = object()

        @staticmethod
        def get_parameter(_name):
            return SimpleNamespace(value=True)

        @staticmethod
        def destroy_node():
            events.append("node.destroy")

    class FailingViewer:
        @staticmethod
        def is_running():
            return False

        @staticmethod
        def close():
            events.append("viewer.close")
            raise RuntimeError("synthetic viewer close failure")

    viewer_module = ModuleType("mujoco.viewer")
    viewer_module.launch_passive = lambda _model, _data: FailingViewer()
    mujoco_module = ModuleType("mujoco")
    mujoco_module.viewer = viewer_module

    monkeypatch.setattr(module, "WholeArmMujocoMirror", FakeNode)
    monkeypatch.setattr(
        module.signal, "getsignal", lambda signum: previous_handlers[signum]
    )
    monkeypatch.setattr(
        module.signal,
        "signal",
        lambda signum, handler: installed_handlers.__setitem__(signum, handler),
    )
    with mock.patch.dict(
        sys.modules,
        {"mujoco": mujoco_module, "mujoco.viewer": viewer_module},
    ):
        with pytest.raises(RuntimeError, match="synthetic viewer close failure"):
            module.main([])

    assert events.index("viewer.close") < events.index("node.destroy")
    assert events.index("node.destroy") < events.index("rclpy.shutdown")
    assert installed_handlers[signal.SIGINT] is previous_handlers[signal.SIGINT]
    assert installed_handlers[signal.SIGTERM] is previous_handlers[signal.SIGTERM]
