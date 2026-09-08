import importlib.util
import json
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
TEST_THERMAL_LIMITS = SimpleNamespace(thermal_stop_c=60.0)


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
    state_model.parse_controller_feedback_metadata = lambda _payload, _samples: {}
    state_model.unavailable_worker_control_status = lambda _reason: {}
    state_model.validate_j6_feedback_identity = (
        lambda _payload, _receipt, **kwargs: kwargs.get("previous")
    )
    state_model.validate_worker_supervisor_status = lambda value, _now, _age: value
    state_model.validate_preserved_session_reference = lambda _value: {}
    state_model.supported_near_vertical_recovery = lambda _value: False
    thermal_manager = ModuleType(f"{package_name}.thermal_manager")

    def thermal_state(
        temperature_c, *, fault_latched, cooldown_ready, limits,
    ):
        if fault_latched:
            state = "WAIT_OPERATOR_CONFIRM" if cooldown_ready else "THERMAL_STOP"
        elif float(temperature_c) >= 60.0:
            state = "THERMAL_STOP"
        elif float(temperature_c) >= 55.0:
            state = "DERATING"
        elif float(temperature_c) >= 45.0:
            state = "WARNING"
        else:
            state = "NORMAL"
        return SimpleNamespace(value=state)

    thermal_manager.thermal_state_from_controller_metadata = thermal_state
    thermal_manager.THERMAL_CONFIG_SHA256 = "a" * 64
    thermal_manager.ThermalLimits = object
    thermal_manager.load_thermal_limits = lambda _path: TEST_THERMAL_LIMITS

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
        f"{package_name}.thermal_manager": thermal_manager,
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


def _normalized_metadata(source_ns=10, receipt_ns=11):
    return {
        "source_monotonic_ns": source_ns,
        "receipt_monotonic_ns": receipt_ns,
        "controller_mode": "brake",
        "domain_fault": False,
        "lease_safe_hold": False,
        "thermal": {
            "metadata_status": "UNKNOWN",
            "fault_latched": None,
            "cooldown_ready": None,
            "release_observed": None,
            "rearm_pending_next_cycle": None,
            "cooldown_valid_brake_frames": None,
            "trip_activation_epoch": None,
            "minimum_rearm_epoch": None,
            "domain_reported_state": None,
            "reported_state_by_motor": None,
            "reported_state": None,
            "trip_reason": None,
            "thermal_config_sha256": None,
            "derating_factor": None,
            "raw_temperature_c": None,
            "window_median_c": None,
            "slope_c_per_min": None,
        },
        "no_progress": {
            "metadata_status": "UNKNOWN",
            "fault_latched": None,
            "release_observed": None,
            "rearm_pending_next_cycle": None,
            "qualifying_frames": None,
            "observation_valid": None,
            "position_error_rad": None,
            "trip_position_error_rad": None,
            "software_saturation_observed": None,
            "watchdog_authority": None,
            "continuous_rating_authoritative": False,
            "trip_activation_epoch": None,
            "minimum_rearm_epoch": None,
        },
        "gravity": {
            "metadata_status": "UNKNOWN",
            "authority_present": None,
            "scale": None,
            "scale_target": None,
            "feedforward_nm": None,
            "applied_rotor_nm": None,
        },
    }


def _bare_state_node(module, *, model):
    node = object.__new__(module.WholeArmStateNode)
    node.model = model
    node.j2_sync_fault = False
    node.controller_feedback = {"J6": {"old": True}}
    node.controller_modes = {"J6": "hold"}
    node.controller_faults = {"J6": True}
    node.controller_lease_safe_hold = {"J6": True}
    node.controller_thermal = {"J6": {"old": True}}
    node.j6_feedback_identity = None
    node.session_id = "test-session"
    node.state_instance_id = "test-state-instance"
    node.invalid_payload_count = 0
    node.record_invalid_payload = lambda _error: setattr(
        node, "invalid_payload_count", node.invalid_payload_count + 1
    )
    return node


def test_feedback_metadata_is_not_committed_when_model_rejects(monkeypatch):
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    sample = SimpleNamespace(motor="J1")
    metadata = _normalized_metadata()
    monkeypatch.setattr(
        module, "parse_feedback_payload", lambda _payload, _receipt: (sample,)
    )
    monkeypatch.setattr(
        module,
        "parse_controller_feedback_metadata",
        lambda _payload, _samples: {"J1": metadata},
    )

    class RejectingModel:
        @staticmethod
        def update_batch(_samples):
            raise ValueError("synthetic model rejection")

    node = _bare_state_node(module, model=RejectingModel())
    before = (
        node.controller_feedback,
        node.controller_modes,
        node.controller_faults,
        node.controller_lease_safe_hold,
        node.controller_thermal,
    )
    node.accept_payload(json.dumps({
        "schema": "go-m8010-motor-feedback/1.0",
        "domain_fault": False,
    }), 11)

    assert node.invalid_payload_count == 1
    assert before == (
        node.controller_feedback,
        node.controller_modes,
        node.controller_faults,
        node.controller_lease_safe_hold,
        node.controller_thermal,
    )
    assert "J1" not in node.controller_feedback


def test_feedback_metadata_swaps_with_accepted_model_sample(monkeypatch):
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    sample = SimpleNamespace(motor="J1")
    metadata = _normalized_metadata()
    monkeypatch.setattr(
        module, "parse_feedback_payload", lambda _payload, _receipt: (sample,)
    )
    monkeypatch.setattr(
        module,
        "parse_controller_feedback_metadata",
        lambda _payload, _samples: {"J1": metadata},
    )

    class AcceptingModel:
        accepted = False

        def update_batch(self, _samples):
            self.accepted = True

    model = AcceptingModel()
    node = _bare_state_node(module, model=model)
    node.accept_payload(json.dumps({
        "schema": "go-m8010-motor-feedback/1.0",
        "domain_fault": False,
    }), 11)

    assert model.accepted is True
    assert node.invalid_payload_count == 0
    assert node.controller_feedback["J1"] == metadata
    assert node.controller_modes["J1"] == "brake"
    assert node.controller_faults["J1"] is False
    assert node.controller_lease_safe_hold["J1"] is False


def test_udp_raw_observer_gets_original_packet_without_reingestion(monkeypatch):
    module, _ = load_with_ros_stubs([], {"ok": False, "on_spin": lambda: None})
    events, seen = [], set()
    monkeypatch.setattr(module, "parse_feedback_payload", lambda payload, _receipt: (
        SimpleNamespace(motor="J1", sequence=payload["sequence"]),))

    class Model:
        def update_batch(self, samples):
            sequence = samples[0].sequence
            if sequence in seen:
                raise ValueError("replayed or out-of-order motor feedback")
            seen.add(sequence)
            events.append("ingest")

    node = _bare_state_node(module, model=Model())
    node.pending_raw_echoes = module.deque(maxlen=512)
    published = []
    node.raw_feedback_publisher = SimpleNamespace(publish=lambda message: (
        events.append("publish"), published.append(message.data)))
    packet = '{"schema":"go-m8010-motor-feedback/1.0","sequence":1}'
    node.udp_socket = SimpleNamespace(recvfrom=mock.Mock(side_effect=[
        (packet.encode(), ("127.0.0.1", 1234)), BlockingIOError(),
    ]))
    node.poll_udp()
    assert events == ["ingest", "publish"]
    assert published == [packet]
    node.on_feedback(SimpleNamespace(data=packet))  # One self echo only.
    assert events == ["ingest", "publish"]
    assert node.invalid_payload_count == 0
    node.on_feedback(SimpleNamespace(data=packet))  # A further replay still fails.
    assert node.invalid_payload_count == 1
    external = packet.replace('"sequence":1', '"sequence":2')
    node.on_feedback(SimpleNamespace(data=external))
    assert seen == {1, 2} and published == [packet]
    node.udp_socket.recvfrom = mock.Mock(side_effect=[
        (b"invalid JSON", ("127.0.0.1", 1234)), BlockingIOError(),
    ])
    node.poll_udp()
    assert node.invalid_payload_count == 2 and published == [packet]


def test_accepted_guidance_transport_keeps_encoders_and_pairs_native_reference(monkeypatch):
    from go_m8010_arm_hardware.state_model import (
        parse_feedback_payload, parse_controller_feedback_metadata, validate_j6_feedback_identity,
    )

    module, _ = load_with_ros_stubs([], {"ok": False, "on_spin": lambda: None})
    monkeypatch.setattr(module, "parse_feedback_payload", parse_feedback_payload)
    monkeypatch.setattr(module, "parse_controller_feedback_metadata", parse_controller_feedback_metadata)
    monkeypatch.setattr(module, "validate_j6_feedback_identity", validate_j6_feedback_identity)
    accepted = []
    node = _bare_state_node(module, model=SimpleNamespace(update_batch=lambda batch: accepted.extend(batch)))
    node.state_instance_id = "8" * 32
    ns = 5_000_000_000

    def payload(names=("J2A", "J2B"), mode="hold"):
        return {
            "schema": "go-m8010-motor-feedback/1.0", "source_monotonic_ns": ns,
            "source_instance_id": "9" * 32, "sequence": 1,
            "session_id": node.session_id, "state_instance_id": node.state_instance_id,
            "controller_mode": mode, "controller_mode_by_motor": {name: mode for name in names},
            "domain_fault": False, "j2_sync_fault": False,
            "tau_j2_logical_total_nm": 0.0 if "J2A" in names else None,
            "samples": [{"motor": name, "position_rad": 0.1, "velocity_rad_s": 0.0,
                "temperature_c": 25.0, "communication_ok": True, "merror": 0,
                "last_valid_feedback_monotonic_ns": ns,
                "tau_cmd_rotor_nm": None if name == "J6" else 0.0,
                "tau_feedback_rotor_nm": None if name == "J6" else 0.0,
                "tau_joint_estimated_nm": None if name == "J6" else 0.0,
                "accepted_guidance_target_rad": 0.2, "accepted_guidance_activation_epoch": 123,
            } for name in names],
        }

    def send(value):
        assert node.accept_payload(json.dumps(value), ns + 1)
        assert node.invalid_payload_count == 0
        assert all(sample.position_rad == 0.1 for sample in accepted)
        return {name: module.controller_metadata_for_hardware_state(
            node.controller_feedback[name], {
                "fresh": True, "feedback_source_monotonic_ns": ns,
                "feedback_receipt_monotonic_ns": ns + 1, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
            }, TEST_THERMAL_LIMITS) for name in value["controller_mode_by_motor"]}

    for names in (("J2A", "J2B"), ("J1",), ("J6",)):
        value = payload(names, mode="teach")
        observed = send(value)
        for name in names:
            assert observed[name]["accepted_guidance_metadata_status"] == "OBSERVED"
            assert observed[name]["accepted_guidance_target_rad"] == 0.2
            assert observed[name]["accepted_guidance_activation_epoch"] == 123
            assert observed[name]["guidance_paused_reason"] is None
        ns += 10_000_000
    value = payload()
    for sample in value["samples"]:
        sample["guidance_paused_reason"] = "DEADMAN_TIMEOUT"
    observed = send(value)
    assert observed["J2A"]["guidance_paused_reason"] == "DEADMAN_TIMEOUT"
    for sample in value["samples"]:
        sample["guidance_paused_reason"] = "GUI_RELEASE"
    observed = send(value)
    assert all(row["guidance_paused_reason"] == "GUI_RELEASE"
               and row["accepted_guidance_metadata_status"] == "OBSERVED"
               and row["domain_fault"] is False for row in observed.values())
    stale = module.controller_metadata_for_hardware_state(node.controller_feedback["J2A"],
        {"fresh": False}, TEST_THERMAL_LIMITS)
    assert stale["accepted_guidance_metadata_status"] == "STALE"
    assert stale["accepted_guidance_target_rad"] == 0.2
    assert stale["accepted_guidance_activation_epoch"] == 123
    for field, invalid in (("accepted_guidance_target_rad", float("nan")),
                           ("accepted_guidance_target_rad", True),
                           ("accepted_guidance_target_rad", 0.21),
                           ("accepted_guidance_activation_epoch", False),
                           ("accepted_guidance_activation_epoch", 124),
                           ("guidance_paused_reason", "UNKNOWN")):
        value = payload()
        value["samples"][0][field] = invalid
        observed = send(value)
        assert all(row["accepted_guidance_metadata_status"] == "INVALID" for row in observed.values())
    value = payload()
    value["samples"][0].pop("accepted_guidance_activation_epoch")
    assert send(value)["J2A"]["accepted_guidance_metadata_status"] == "INVALID"
    assert send(payload(mode="brake"))["J2A"]["accepted_guidance_metadata_status"] == "INVALID"
    value = payload()
    for sample in value["samples"]:
        sample.pop("accepted_guidance_target_rad")
        sample.pop("accepted_guidance_activation_epoch")
    assert all("accepted_guidance_metadata_status" not in row for row in send(value).values())


def test_stale_or_unpaired_controller_metadata_is_explicitly_unknown():
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    metadata = _normalized_metadata(source_ns=10, receipt_ns=11)
    motor_state = {
        "fresh": True,
        "feedback_source_monotonic_ns": 12,
        "feedback_receipt_monotonic_ns": 13,
        "temperature_c": 25.0,
        "communication_ok": True,
        "merror": 0,
    }
    result = module.controller_metadata_for_hardware_state(
        metadata, motor_state, TEST_THERMAL_LIMITS
    )
    assert result["metadata_status"] == "UNKNOWN"
    assert result["domain_fault"] is None
    assert result["thermal"]["state"] == "UNKNOWN"
    assert result["no_progress"]["fault_latched"] is None
    assert result["gravity"]["authority_present"] is None


def test_current_controller_metadata_keeps_unknown_optional_groups_unknown():
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    metadata = _normalized_metadata(source_ns=10, receipt_ns=11)
    motor_state = {
        "fresh": True,
        "feedback_source_monotonic_ns": 10,
        "feedback_receipt_monotonic_ns": 11,
        "temperature_c": 25.0,
        "communication_ok": True,
        "merror": 0,
    }
    result = module.controller_metadata_for_hardware_state(
        metadata, motor_state, TEST_THERMAL_LIMITS
    )
    assert result["metadata_status"] == "OBSERVED"
    assert result["controller_mode"] == "brake"
    assert result["brake_observed"] is True
    assert result["thermal"]["state"] == "UNKNOWN"
    assert result["no_progress"]["metadata_status"] == "UNKNOWN"
    assert result["gravity"]["metadata_status"] == "UNKNOWN"


def _observed_thermal_metadata(reported_state, *, mode="hold"):
    metadata = _normalized_metadata(source_ns=10, receipt_ns=11)
    metadata["controller_mode"] = mode
    metadata["thermal"].update({
        "metadata_status": "OBSERVED",
        "fault_latched": False,
        "cooldown_ready": False,
        "release_observed": False,
        "rearm_pending_next_cycle": False,
        "cooldown_valid_brake_frames": 0,
        "trip_activation_epoch": 0,
        "minimum_rearm_epoch": 0,
        "domain_reported_state": "DERATING",
        "reported_state_by_motor": None,
        "reported_state": reported_state,
        "trip_reason": "",
        "thermal_config_sha256": (
            "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
        ),
        "derating_factor": 0.6,
        "raw_temperature_c": 57.0,
        "window_median_c": None,
        "slope_c_per_min": None,
    })
    return metadata


def _fresh_motor_state(temperature_c, *, communication_ok=True):
    return {
        "fresh": True,
        "feedback_source_monotonic_ns": 10,
        "feedback_receipt_monotonic_ns": 11,
        "temperature_c": temperature_c,
        "communication_ok": communication_ok,
        "merror": 0,
    }


def test_cross_temperature_domain_uses_each_motor_reported_state():
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    cases = (
        (44.0, "NORMAL"),
        (55.0, "DERATING"),
        (57.0, "DERATING"),
    )
    for temperature_c, reported_state in cases:
        result = module.controller_metadata_for_hardware_state(
            _observed_thermal_metadata(reported_state),
            _fresh_motor_state(temperature_c),
            TEST_THERMAL_LIMITS,
        )
        assert result["thermal"]["domain_reported_state"] == "DERATING"
        assert result["thermal"]["reported_state"] == reported_state
        assert result["thermal"]["reported_state_consistent"] is True
        assert result["thermal"]["state"] == reported_state


def test_teach_feedback_stays_observed_without_weakening_freshness_or_faults():
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    metadata = _observed_thermal_metadata("NORMAL", mode="teach")
    state = _fresh_motor_state(25.0)
    result = module.controller_metadata_for_hardware_state(
        metadata, state, TEST_THERMAL_LIMITS
    )
    assert result["metadata_status"] == "OBSERVED"
    assert result["controller_mode"] == "teach"
    assert result["brake_observed"] is False
    assert result["thermal"]["state"] == "NORMAL"
    metadata["domain_fault"] = True
    result = module.controller_metadata_for_hardware_state(
        metadata, state, TEST_THERMAL_LIMITS
    )
    assert result["domain_fault"] is True
    state["fresh"] = False
    result = module.controller_metadata_for_hardware_state(
        metadata, state, TEST_THERMAL_LIMITS
    )
    assert result["metadata_status"] == "UNKNOWN"
    assert result["controller_mode"] == "unknown"
    assert result["domain_fault"] is None


def test_native_exit_hold_aggregation_requires_fresh_paired_session_and_one_proof():
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    proof = {
        "schema": "go-m8010-teach-exit-hold/1.0", "joint_index": 1,
        "press_activation_epoch": 7, "started_monotonic_ns": 10,
        "deadline_monotonic_ns": 1_000_000_010, "reason": "TIME_LIMIT",
        "targets_rad": [0.0] * 6, "initial_velocity_rad_s": 0.05,
        "restricted": True,
    }
    metadata = _observed_thermal_metadata("NORMAL", mode="hold")
    metadata.update({
        "assisted_teach_exit_hold": proof,
        "assisted_teach_exit_hold_validated": True,
        "assisted_teach_exit_hold_source_monotonic_ns": 10,
    })
    metadata["gravity"].update({
        "session_id": "session", "state_instance_id": "state",
        "source_instance_id": "a" * 32,
    })
    paired = module.controller_metadata_for_hardware_state(
        metadata, _fresh_motor_state(25.0), TEST_THERMAL_LIMITS
    )
    assert paired["assisted_teach_exit_hold"] == proof
    aggregate = module.assisted_teach_exit_hold_for_hardware_state
    assert aggregate({}, "session", "state") == {}
    both = {"J2A": paired, "J2B": dict(paired)}
    result = aggregate(both, "session", "state")
    assert result["assisted_teach_exit_hold_validated"] is True
    assert result["assisted_teach_exit_hold"] == proof
    assert result["assisted_teach_exit_hold"]["restricted"] is True
    assert result["assisted_teach_exit_hold_source_monotonic_ns"] == 10
    for changed in (
        {"assisted_teach_exit_hold_validated": False, "assisted_teach_exit_hold": None},
        {"assisted_teach_exit_hold": {**proof, "press_activation_epoch": 8}},
        {"assisted_teach_exit_hold_source_monotonic_ns": 12},
        {"controller_mode": "teach"},
        {"domain_fault": True},
    ):
        result = aggregate({"J2A": paired, "J2B": {**paired, **changed}}, "session", "state")
        assert result["assisted_teach_exit_hold_validated"] is False
        assert result["assisted_teach_exit_hold"] is None
    assert aggregate(both, "other-session", "state")["assisted_teach_exit_hold_validated"] is False
    assert aggregate(both, "session", "other-state")["assisted_teach_exit_hold_validated"] is False
    stale = module.controller_metadata_for_hardware_state(
        metadata, {**_fresh_motor_state(25.0), "fresh": False}, TEST_THERMAL_LIMITS
    )
    assert aggregate({"J2A": paired, "J2B": stale}, "session", "state")["assisted_teach_exit_hold_validated"] is False
    assert aggregate({"J2A": stale, "J2B": stale}, "session", "state") == {}


def test_exact_55c_boundary_is_derating_not_unknown():
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    result = module.controller_metadata_for_hardware_state(
        _observed_thermal_metadata("DERATING"),
        _fresh_motor_state(55.0),
        TEST_THERMAL_LIMITS,
    )
    assert result["thermal"]["state"] == "DERATING"
    assert result["thermal"]["reported_state_consistent"] is True


def test_unverified_brake_never_becomes_brake_observed():
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    unknown_mode = _observed_thermal_metadata("NORMAL", mode="unknown")
    result = module.controller_metadata_for_hardware_state(
        unknown_mode, _fresh_motor_state(25.0), TEST_THERMAL_LIMITS
    )
    assert result["controller_mode"] == "unknown"
    assert result["brake_observed"] is False

    verified_mode_no_communication = _observed_thermal_metadata(
        "NORMAL", mode="brake"
    )
    result = module.controller_metadata_for_hardware_state(
        verified_mode_no_communication,
        _fresh_motor_state(25.0, communication_ok=False),
        TEST_THERMAL_LIMITS,
    )
    assert result["brake_observed"] is False

    result = module.controller_metadata_for_hardware_state(
        verified_mode_no_communication,
        _fresh_motor_state(25.0, communication_ok=True),
        TEST_THERMAL_LIMITS,
    )
    assert result["brake_observed"] is True


def test_state_instance_id_is_strictly_pinnable_with_random_default(monkeypatch):
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    monkeypatch.setattr(module.secrets, "token_hex", lambda size: "9" * (size * 2))
    assert module.validated_state_instance_id("") == "9" * 32
    assert module.validated_state_instance_id("a" * 32) == "a" * 32
    for invalid in ("a" * 31, "a" * 33, "A" * 32, "g" * 32, " " * 32):
        with pytest.raises(ValueError):
            module.validated_state_instance_id(invalid)


def test_complete_references_determine_session_before_first_feedback():
    module, _no_signal_handlers = load_with_ros_stubs([], {
        "ok": False, "on_spin": lambda: None,
    })
    expected = (
        "persistent:0123456789abcdef:"
        "j2session:1111111111111111:"
        "goauxsession:2222222222222222"
    )
    assert module.complete_persistent_session_id(
        "0123456789abcdef" + "0" * 48,
        "1" * 64,
        "2" * 64,
    ) == expected
    for values in (
        (None, "1" * 64, "2" * 64),
        ("0" * 64, None, "2" * 64),
        ("0" * 64, "1" * 64, None),
    ):
        assert module.complete_persistent_session_id(*values) is None

    source = SOURCE.read_text(encoding="utf-8")
    initialize = source.index("self.session_id = complete_persistent_session_id(")
    first_feedback_validation = source.index("def accept_payload(")
    first_publish = source.index("def publish_state(")
    assert initialize < first_feedback_validation < first_publish

    launch = (
        SOURCE.parents[2]
        / "go_m8010_arm_gui"
        / "launch"
        / "arm_gui.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("state_instance_id", default_value="")' in launch
    assert '"state_instance_id": LaunchConfiguration("state_instance_id")' in launch
    assert 'condition=IfCondition(LaunchConfiguration("start_gravity_node"))' in launch
