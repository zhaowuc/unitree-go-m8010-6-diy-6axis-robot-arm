"""无需 ROS2、Qt 或硬件即可验证 GUI 的反馈准入逻辑。"""

import ast
import hashlib
import json
import math
import struct
import tempfile
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import Optional

import numpy as np
import pytest


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "go_m8010_arm_gui"
    / "main_window.py"
)
PRODUCTION_MODEL = (
    SOURCE.parents[4]
    / "mujoco_v15_14"
    / "go_m8010_arm_v15_14_kinematic.xml"
)
SESSION_POSE_DEG = [0.0, 90.0, -14.40, 13.49, 47.94, 0.0]
ABSOLUTE_MODEL_LIMITS_DEG = [
    (-180.0, 180.0),
    (-170.0, 170.0),
    (-170.0, 170.0),
    (-116.0, 159.0),
    (-70.6, 151.2),
    (-180.0, 180.0),
]
SESSION_MODEL_LIMITS_DEG = [
    (-180.0, 180.0),
    (-260.0, 80.0),
    (-155.6, 184.4),
    (-129.49, 145.51),
    (-118.54, 103.26),
    (-180.0, 180.0),
]
MODEL_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
COLLISION_CONTRACT_SHA256 = (
    "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
)
KINEMATIC_GUARD_SHA256 = (
    "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
)


def collision_hardware_state_hash(value):
    motor_names = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
    document = {
        "schema": value["schema"],
        "session_id": value["session_id"],
        "state_instance_id": value["state_instance_id"],
        "sequence": value["sequence"],
        "source_monotonic_ns": value["source_monotonic_ns"],
        "position_rad": list(value["position_rad"]),
        "velocity_rad_s": list(value["velocity_rad_s"]),
        "controller_mode_by_motor": [
            [name, value["controller_mode_by_motor"][name]]
            for name in motor_names
        ],
    }
    encoded = json.dumps(
        document, ensure_ascii=True, allow_nan=False, sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_function(name, extra_namespace=None):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {
        "hashlib": hashlib,
        "math": math,
        "np": np,
        "Path": Path,
        "Optional": Optional,
        "struct": struct,
        "ET": ET,
        "time": time,
        "JOINT_NAMES": tuple(f"joint{index}" for index in range(1, 7)),
        "MOTOR_NAMES": ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6"),
        "MOTOR_GROUPS": (
            ("J1",), ("J2A", "J2B"), ("J3",),
            ("J4",), ("J5",), ("J6",),
        ),
        "CONTROLLER_MODES": frozenset({
            "brake", "drag", "hold", "position", "unknown",
        }),
        "RAD": math.pi / 180.0,
        "COLLISION_GUARD_TIMEOUT_S": 8.0,
        "COLLISION_GUARD_MAX_STEP_DEG": 0.25,
        "COLLISION_BOUNDARY_TOLERANCE_DEG": 0.01,
        "COLLISION_GROUND_POLICY": "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION",
        "COLLISION_MARGIN_POLICY": (
            "SINGLE_JOINT_EXTENDED_SWEEP_SUPPORT_CROSS_FINAL_LINF_V2"
        ),
        "COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG": 2.25,
        "COLLISION_HOLD_TRACKING_TOLERANCE_DEG": 0.25,
        "COLLISION_TUBE_PROBE_JOINT_NAMES": (
            "J1", "J2", "J3", "J4", "J5", "J6",
        ),
        "COLLISION_TUBE_GRID_OFFSETS": (-1, 0, 1),
        "COLLISION_TUBE_GRID_MAX_POSE_COUNT": 729,
        "COLLISION_TUBE_AXIS_PROBE_STEP_DEG": 0.25,
        "COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT": 825,
        "PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG": tuple(
            ABSOLUTE_MODEL_LIMITS_DEG
        ),
        "COLLISION_START_MATCH_TOLERANCE_RAD": math.radians(0.25),
        "canonical_collision_hardware_state_sha256": (
            collision_hardware_state_hash
        ),
        "COLLISION_HOLD_TRACKING_TOLERANCE_RAD": math.radians(0.25),
        "COLLISION_HOLD_VELOCITY_TOLERANCE_RAD_S": math.radians(0.25),
        "TARGET_SELECTION_DEADBAND_RAD": math.radians(0.01),
        "PRODUCTION_MODEL_SHA256": MODEL_SHA256,
        "PRODUCTION_COLLISION_CONTRACT_SHA256": COLLISION_CONTRACT_SHA256,
        "PRODUCTION_KINEMATIC_GUARD_SHA256": KINEMATIC_GUARD_SHA256,
        "ROUTER_REJECTION_WARNING_WINDOW_MS": 5000.0,
        "HARDWARE_STATE_SOURCE_MAX_AGE_NS": 250_000_000,
        "HARDWARE_STATE_SOURCE_TAKEOVER_TIMEOUT_NS": 500_000_000,
        "source_instance_id_valid": lambda value: bool(
            isinstance(value, str)
            and len(value) == 32
            and all(character in "0123456789abcdef" for character in value)
        ),
        "optional_sha256_valid": lambda value: bool(
            value is None
            or (
                isinstance(value, str)
                and len(value) == 64
                and all(character in "0123456789abcdef" for character in value)
            )
        ),
    }
    if extra_namespace:
        namespace.update(extra_namespace)
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace[name]


def load_main_window_method(name, extra_namespace=None):
    return load_class_method("MainWindow", name, extra_namespace)


def load_class_method(class_name, name, extra_namespace=None):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    selected_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    method = next(
        node
        for node in selected_class.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    module = ast.Module(body=[method], type_ignores=[])
    namespace = {
        "datetime": datetime,
        "timezone": timezone,
        "json": json,
        "math": math,
        "time": time,
        "Optional": Optional,
        "TARGET_SELECTION_DEADBAND_RAD": math.radians(0.01),
        "COLLISION_GUARD_TIMEOUT_S": 8.0,
        "COLLISION_START_MATCH_TOLERANCE_RAD": math.radians(0.25),
    }
    if extra_namespace:
        namespace.update(extra_namespace)
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace[name]


def test_initial_pose_read_only_parameter_is_a_strict_boolean():
    strict = load_function("strict_bool_parameter")
    assert strict(True, "initial_pose_read_only") is True
    assert strict(False, "initial_pose_read_only") is False
    for invalid in (0, 1, "true", "false", None):
        try:
            strict(invalid, "initial_pose_read_only")
        except TypeError:
            pass
        else:
            raise AssertionError(f"non-boolean value was accepted: {invalid!r}")


def test_read_only_initial_pose_disables_set_button():
    class FakeButton:
        def __init__(self, text):
            self.text = text
            self.enabled = True
            self.tooltip = ""

        def setEnabled(self, enabled):
            self.enabled = enabled

        def setToolTip(self, tooltip):
            self.tooltip = tooltip

        def setStyleSheet(self, _style):
            pass

    class FakeLayout:
        def __init__(self, _parent):
            pass

        def addWidget(self, *_args):
            pass

    class FakeLabel:
        def __init__(self, _text):
            pass

        def setAlignment(self, _alignment):
            pass

        def setWordWrap(self, _enabled):
            pass

    method = load_main_window_method(
        "_control_panel",
        {
            "QGroupBox": lambda _title: object(),
            "QGridLayout": FakeLayout,
            "QLabel": FakeLabel,
            "Qt": SimpleNamespace(AlignCenter=0),
        },
    )

    class FakeWindow:
        def __init__(self, read_only):
            self.node = SimpleNamespace(initial_pose_read_only=read_only)
            self.buttons = []

        def _button(self, text, _callback, object_name=""):
            del object_name
            button = FakeButton(text)
            self.buttons.append(button)
            return button

        def __getattr__(self, _name):
            return lambda *_args, **_kwargs: None

    protected = FakeWindow(True)
    method(protected)
    protected_button = next(
        button for button in protected.buttons if button.text == "设置初始化姿态"
    )
    assert not protected_button.enabled
    assert "只读保护" in protected_button.tooltip

    writable = FakeWindow(False)
    method(writable)
    writable_button = next(
        button for button in writable.buttons if button.text == "设置初始化姿态"
    )
    assert writable_button.enabled


def test_direct_save_call_cannot_change_read_only_initial_pose_bytes_or_hash():
    save = load_main_window_method("_save_initial_pose")
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "initial_pose.json"
        original = b'{"preserved":"vertical-initialization"}\n'
        path.write_bytes(original)
        original_sha256 = hashlib.sha256(original).hexdigest()

        class FakeWindow:
            node = SimpleNamespace(
                initial_pose_read_only=True,
                initial_pose_path=path,
            )
            session_id = "persistent:preserved"
            actual = [1.0] * 6

            def __init__(self):
                self.notices = []

            def _notify(self, text, level="warning"):
                self.notices.append((text, level))

            def _require_control_feedback(self, *_args, **_kwargs):
                raise AssertionError("read-only guard must run before feedback checks")

        window = FakeWindow()
        save(window)
        assert path.read_bytes() == original
        assert hashlib.sha256(path.read_bytes()).hexdigest() == original_sha256
        assert not path.with_suffix(path.suffix + ".tmp").exists()
        assert window.notices == [
            ("生产初始化姿态受只读保护，GUI拒绝覆盖。", "warning")
        ]


def test_launch_declares_and_passes_typed_initial_pose_read_only_parameter():
    launch_source = (
        SOURCE.parents[1] / "launch" / "arm_gui.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("initial_pose_read_only", default_value="false")' in launch_source
    assert '"initial_pose_read_only": ParameterValue(' in launch_source
    assert 'LaunchConfiguration("initial_pose_read_only"), value_type=bool' in launch_source
    assert 'DeclareLaunchArgument("pose_matched", default_value="false")' in launch_source
    assert 'LaunchConfiguration("pose_matched"), value_type=bool' in launch_source


def test_partial_joint_health_allows_isolated_motion_actions():
    ready = load_function("control_feedback_ready")
    assert ready(True, True, [False, False, False, True, False, False])


def test_motion_action_still_requires_fresh_streams_and_one_healthy_joint():
    ready = load_function("control_feedback_ready")
    assert not ready(False, True, [True, False, False, False, False, False])
    assert not ready(True, False, [True, False, False, False, False, False])
    assert not ready(True, True, [False] * 6)


def hardware_state(*, stale_motor=None, fault_motor=None, controller_fault_motor=None):
    motors = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
    per_motor = {
        name: {
            "q_joint_rad": 0.0,
            "dq_joint_rad_s": 0.0,
            "temperature_c": 25.0,
            "merror": 1 if name == fault_motor else 0,
            "communication_ok": name != fault_motor,
            "fresh": name != stale_motor,
            "reference_captured": True,
        }
        for name in motors
    }
    return {
        "schema": "go-m8010-hardware-state/1.1",
        "sequence": 1,
        "session_id": "persistent:0123456789abcdef",
        "state_instance_id": "0123456789abcdef0123456789abcdef",
        "source_monotonic_ns": time.monotonic_ns(),
        "reference": "PERSISTENT_SOFTWARE_ZERO_V1",
        "persistent_zero_sha256": "a" * 64,
        "initial_pose_sha256": "b" * 64,
        "joint_names": [f"joint{index}" for index in range(1, 7)],
        "position_rad": [0.0] * 6,
        "velocity_rad_s": [0.0] * 6,
        "j2_e_sync_rad": 0.0,
        "j2_sync_fault": False,
        "per_motor": per_motor,
        "controller_mode_by_motor": {name: "hold" for name in motors},
        "controller_fault_by_motor": {
            name: name == controller_fault_motor for name in motors
        },
        "lease_safe_hold_by_motor": {name: False for name in motors},
    }


def load_collision_motion_state_ready():
    contract_valid = load_function("hardware_state_contract_valid")
    source_fresh = load_function("hardware_state_source_is_fresh")
    classify = load_function("classify_logical_joint_observations")
    return load_function(
        "collision_motion_state_ready",
        {
            "hardware_state_contract_valid": contract_valid,
            "hardware_state_source_is_fresh": source_fresh,
            "classify_logical_joint_observations": classify,
        },
    )


def load_virtual_target_edit_state_ready():
    contract_valid = load_function("hardware_state_contract_valid")
    source_fresh = load_function("hardware_state_source_is_fresh")
    classify = load_function("classify_logical_joint_observations")
    return load_function(
        "virtual_target_edit_state_ready",
        {
            "hardware_state_contract_valid": contract_valid,
            "hardware_state_source_is_fresh": source_fresh,
            "classify_logical_joint_observations": classify,
        },
    )


def test_hardware_state_heartbeat_requires_the_complete_strict_contract():
    valid = load_function("hardware_state_contract_valid")
    document = hardware_state()
    assert valid(document)
    for invalid in ({}, {"schema": "go-m8010-hardware-state/1.1"}):
        assert not valid(invalid)
    document["per_motor"]["J1"]["fresh"] = 1
    assert not valid(document)
    document = hardware_state()
    document["j2_e_sync_rad"] = "0.0"
    assert not valid(document)
    document = hardware_state()
    document["state_instance_id"] = "not-a-valid-instance"
    assert not valid(document)
    document = hardware_state()
    document["source_monotonic_ns"] = True
    assert not valid(document)


def test_virtual_preview_editing_is_not_blocked_by_vendor_velocity_noise():
    edit_ready = load_virtual_target_edit_state_ready()
    execute_ready = load_collision_motion_state_ready()
    document = hardware_state()
    document["velocity_rad_s"] = [math.radians(7.0)] * 6
    active = [True] * 6

    assert edit_ready(document, active, "hold")
    assert not execute_ready(document, [0.0] * 6, active, "hold")

    document["controller_mode_by_motor"]["J6"] = "position"
    assert not edit_ready(document, active, "hold")


def test_virtual_preview_does_not_flash_at_execution_source_age_boundary():
    edit_ready = load_virtual_target_edit_state_ready()
    execute_ready = load_collision_motion_state_ready()
    document = hardware_state()
    active = [True] * 6
    checked_ns = time.monotonic_ns()
    document["source_monotonic_ns"] = checked_ns - 400_000_000

    # Selecting a virtual target grants no hardware authority, so it remains
    # editable for the accepted receipt lifetime instead of toggling at 250 ms.
    assert edit_ready(document, active, "hold", now_ns=checked_ns)
    # Any real motion/collision authorization keeps the strict 250 ms gate.
    assert not execute_ready(
        document, [0.0] * 6, active, "hold", now_ns=checked_ns
    )


def test_virtual_widget_gate_uses_hold_health_not_motion_authorization():
    source = SOURCE.read_text(encoding="utf-8")
    start = source.index("    def _refresh_virtual_editability(self) -> None:")
    end = source.index("    def _record_virtual_target", start)
    refresh = source[start:end]
    assert "virtual_target_edit_state_ready(" in refresh
    assert "collision_motion_state_ready(" not in refresh
    assert "and edit_ready" in refresh


def test_initial_pose_is_bound_to_exact_state_node_loaded_file_and_parent_zero():
    binding_valid = load_function("initial_pose_binding_valid")
    raw = (
        b'{"valid":true,"session":"persistent:aaaaaaaaaaaaaaaa"}'
    )
    document = {
        "有效": True,
        "会话标识": "persistent:aaaaaaaaaaaaaaaa",
    }
    hardware = {
        "reference": "PERSISTENT_SOFTWARE_ZERO_V1",
        "persistent_zero_sha256": "a" * 64,
        "initial_pose_sha256": hashlib.sha256(raw).hexdigest(),
    }
    assert binding_valid(document, raw, hardware)

    replaced = dict(document)
    replaced["关节位置_弧度"] = {"J1": 1.0}
    assert not binding_valid(replaced, raw + b" ", hardware)

    wrong_parent = dict(document)
    wrong_parent["会话标识"] = "persistent:cccccccccccccccc"
    assert not binding_valid(wrong_parent, raw, hardware)

    missing_hash = dict(hardware)
    missing_hash["initial_pose_sha256"] = None
    assert not binding_valid(document, raw, missing_hash)

    transient_reference = dict(hardware)
    transient_reference["reference"] = "SESSION_REFERENCE_V1"
    assert not binding_valid(document, raw, transient_reference)


def test_hardware_state_source_time_rejects_replayed_or_future_snapshot():
    fresh = load_function("hardware_state_source_is_fresh")
    now_ns = 10_000_000_000
    document = hardware_state()
    document["source_monotonic_ns"] = now_ns - 250_000_000
    assert fresh(document, now_ns)
    document["source_monotonic_ns"] = now_ns - 250_000_001
    assert not fresh(document, now_ns)
    document["source_monotonic_ns"] = now_ns + 1
    assert not fresh(document, now_ns)


def test_stale_motor_observation_is_unknown_not_a_confirmed_hard_fault():
    classify = load_function("classify_logical_joint_observations")
    connected, faulted, uncertain = classify(hardware_state(stale_motor="J1"))
    assert not connected[0]
    assert not faulted[0]
    assert uncertain[0]

    connected, faulted, uncertain = classify(hardware_state(fault_motor="J1"))
    assert not connected[0]
    assert faulted[0]
    assert not uncertain[0]

    connected, faulted, uncertain = classify(hardware_state(
        stale_motor="J1", controller_fault_motor="J1"
    ))
    assert not connected[0]
    assert not faulted[0]
    assert uncertain[0]


def test_raw_j2_sync_error_preserves_authority_until_worker_confirms_fault():
    classify = load_function("classify_logical_joint_observations")
    document = hardware_state()
    document["j2_sync_fault"] = False
    document["j2_e_sync_rad"] = math.radians(1.10)
    connected, faulted, uncertain = classify(document)
    assert connected[1]
    assert not faulted[1]
    assert not uncertain[1]

    document["j2_sync_fault"] = True
    connected, faulted, uncertain = classify(document)
    assert not connected[1]
    assert faulted[1]
    assert not uncertain[1]

    document = hardware_state(stale_motor="J2A")
    document["j2_e_sync_rad"] = math.radians(1.10)
    document["j2_sync_fault"] = True
    connected, faulted, uncertain = classify(document)
    assert not connected[1]
    assert not faulted[1]
    assert uncertain[1]


def test_model_limits_are_derived_from_the_frozen_3d_model_and_vertical_anchor():
    parse_pose = load_function("parse_pose_degrees")
    validate_limits = load_function("verified_edit_limits")
    derive_limits = load_function(
        "model_session_relative_limits",
        {
            "parse_pose_degrees": parse_pose,
            "verified_edit_limits": validate_limits,
        },
    )
    assert PRODUCTION_MODEL.is_file()
    pose_text = ",".join(str(value) for value in SESSION_POSE_DEG)
    observed = derive_limits(PRODUCTION_MODEL, pose_text)
    for actual_pair, expected_pair in zip(observed, SESSION_MODEL_LIMITS_DEG):
        assert actual_pair == pytest.approx(expected_pair, abs=1.0e-7)
    recovered_absolute = [
        (
            lower + SESSION_POSE_DEG[index],
            upper + SESSION_POSE_DEG[index],
        )
        for index, (lower, upper) in enumerate(observed)
    ]
    for actual_pair, expected_pair in zip(
        recovered_absolute, ABSOLUTE_MODEL_LIMITS_DEG
    ):
        assert actual_pair == pytest.approx(expected_pair, abs=1.0e-7)


def test_gui_prevalidates_each_moving_target_against_its_complete_model_range():
    valid = load_function("moving_targets_within_model_limits")
    for index, (lower, upper) in enumerate(SESSION_MODEL_LIMITS_DEG):
        mask = [joint == index for joint in range(6)]
        at_lower = [0.0] * 6
        at_upper = [0.0] * 6
        below = [0.0] * 6
        above = [0.0] * 6
        at_lower[index] = math.radians(lower)
        at_upper[index] = math.radians(upper)
        below[index] = math.radians(lower - 0.01)
        above[index] = math.radians(upper + 0.01)
        assert valid(at_lower, mask, SESSION_MODEL_LIMITS_DEG)
        assert valid(at_upper, mask, SESSION_MODEL_LIMITS_DEG)
        assert not valid(below, mask, SESSION_MODEL_LIMITS_DEG)
        assert not valid(above, mask, SESSION_MODEL_LIMITS_DEG)

    # A non-moving joint is not republished by this validation stage.
    assert valid([math.inf] + [0.0] * 5, [False] * 6, SESSION_MODEL_LIMITS_DEG)
    assert not valid([math.inf] + [0.0] * 5, [True] + [False] * 5, SESSION_MODEL_LIMITS_DEG)
    assert not valid([0.0] * 5, [True] * 6, SESSION_MODEL_LIMITS_DEG)


def test_visible_target_inputs_expose_the_complete_model_range_without_fixed_caps():
    validate = load_function("verified_edit_limits")
    assert validate(SESSION_MODEL_LIMITS_DEG) == SESSION_MODEL_LIMITS_DEG
    for invalid in (
        SESSION_MODEL_LIMITS_DEG[:5],
        SESSION_MODEL_LIMITS_DEG[:5] + [(1.0, 1.0)],
        SESSION_MODEL_LIMITS_DEG[:5] + [(math.nan, 1.0)],
    ):
        try:
            validate(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError(f"invalid model limits accepted: {invalid!r}")


def test_joint_sliders_are_long_expanding_and_use_the_full_model_bounds():
    source = SOURCE.read_text(encoding="utf-8")
    slider_start = source.index("    def _slider(")
    slider_end = source.index("    def _virtual_panel", slider_start)
    slider = source[slider_start:slider_end]
    assert "self.limits[index] if bounds is None else bounds" in slider
    assert "slider.setRange(round(lower * 100.0), round(upper * 100.0))" in slider
    assert "slider.setMinimumWidth(420)" in slider
    assert "QSizePolicy.Expanding" in slider

    virtual_start = slider_end
    real_start = source.index("    def _real_panel", virtual_start)
    virtual = source[virtual_start:real_start]
    real_end = source.index("    def _button", real_start)
    real = source[real_start:real_end]
    assert "layout.setColumnStretch(1, 1)" in virtual
    assert "layout.setColumnStretch(1, 1)" in real


def test_window_fits_available_screen_and_keeps_large_content_scrollable():
    source = SOURCE.read_text(encoding="utf-8")
    assert "QScrollArea" in source
    assert "scroll.setWidgetResizable(True)" in source
    assert "QAbstractScrollArea.AdjustIgnored" in source
    assert "Qt.ScrollBarAsNeeded" in source
    assert "scroll.setWidget(root)" in source
    assert "self.setCentralWidget(scroll)" in source
    assert "screen.availableGeometry()" in source
    assert "fitted_window_size(available.width(), available.height())" in source
    assert "self.resize(1920, 980)" not in source


def test_scroll_gutter_and_summary_refresh_are_layout_stable():
    source = SOURCE.read_text(encoding="utf-8")
    assert "scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)" in source
    assert "SUMMARY_REFRESH_PERIOD_S = 0.2" in source
    summary_start = source.index("    def _refresh_summary(self) -> None:")
    summary_end = source.index("    def _open_log(self) -> None:", summary_start)
    summary = source[summary_start:summary_end]
    assert summary.index("self._refresh_safety_notice(now)") < summary.index(
        "if now - self.last_summary_refresh_at < SUMMARY_REFRESH_PERIOD_S:"
    )
    assert "set_widget_text_if_changed(self.summary, summary_text)" in summary


def test_latest_state_subscriptions_and_bounded_callback_pump_avoid_backlog():
    source = SOURCE.read_text(encoding="utf-8")
    assert (
        'create_subscription(JointState, "/joint_states", self._joint_callback, 1)'
        in source
    )
    assert (
        'create_subscription(String, "/whole_arm/hardware_state", '
        'self._hardware_callback, 1)'
        in source
    )
    pump = load_function(
        "pump_ros_callbacks", {"ROS_CALLBACK_BUDGET_PER_TICK": 8}
    )
    calls = []

    def fake_spin_once(node, *, timeout_sec):
        calls.append((node, timeout_sec))

    marker = object()
    pump(marker, fake_spin_once)
    assert calls == [(marker, 0.0)] * 8


def test_change_only_widget_helpers_suppress_redundant_qt_writes():
    class FakeWidget:
        def __init__(self):
            self._text = "same"
            self._enabled = True
            self._tooltip = "tip"
            self._value = 5
            self.calls = []

        def text(self):
            return self._text

        def setText(self, value):
            self.calls.append(("text", value))
            self._text = value

        def isEnabled(self):
            return self._enabled

        def setEnabled(self, value):
            self.calls.append(("enabled", value))
            self._enabled = value

        def toolTip(self):
            return self._tooltip

        def setToolTip(self, value):
            self.calls.append(("tooltip", value))
            self._tooltip = value

        def value(self):
            return self._value

        def setValue(self, value):
            self.calls.append(("value", value))
            self._value = value

    widget = FakeWidget()
    text_changed = load_function("set_widget_text_if_changed")
    enabled_changed = load_function("set_widget_enabled_if_changed")
    tooltip_changed = load_function("set_widget_tooltip_if_changed")
    value_changed = load_function("set_widget_value_if_changed")

    assert not text_changed(widget, "same")
    assert not enabled_changed(widget, True)
    assert not tooltip_changed(widget, "tip")
    assert not value_changed(widget, 5)
    assert widget.calls == []

    assert text_changed(widget, "new")
    assert enabled_changed(widget, False)
    assert tooltip_changed(widget, "new tip")
    assert value_changed(widget, 6)
    assert widget.calls == [
        ("text", "new"),
        ("enabled", False),
        ("tooltip", "new tip"),
        ("value", 6),
    ]


@pytest.mark.parametrize(
    ("available", "expected"),
    (
        ((1370, 933), (1301, 886)),
        ((1024, 600), (972, 570)),
        ((3840, 2160), (1920, 980)),
        ((1, 1), (1, 1)),
    ),
)
def test_fitted_window_size_never_exceeds_available_geometry(
    available, expected,
):
    fit = load_function("fitted_window_size")
    result = fit(*available)
    assert result == expected
    assert 1 <= result[0] <= available[0]
    assert 1 <= result[1] <= available[1]


def test_collision_target_hash_is_canonical_and_rejects_invalid_vectors():
    target_hash = load_function("collision_target_sha256")
    values = [0.0, -0.25, 1.5, -2.0, math.pi, -math.pi]
    expected = hashlib.sha256(struct.pack(">6d", *values)).hexdigest()
    assert target_hash(values) == expected
    assert target_hash(list(values)) == expected
    assert target_hash([0.0] * 6) != target_hash([-0.0] + [0.0] * 5)
    for invalid in ([0.0] * 5, [0.0] * 5 + [math.inf]):
        with pytest.raises(ValueError):
            target_hash(invalid)


def collision_request_and_result(*, safe=True, target=None, start=None):
    target = list(
        [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
        if target is None else target
    )
    start = list([0.0] * 6 if start is None else start)
    source_ns = time.monotonic_ns() - 1_000_000
    session_pose_hash = hashlib.sha256(struct.pack(">6d", *([0.0] * 6))).hexdigest()
    request = {
        "schema": "go-m8010-collision-guard-request/1.0",
        "source_instance_id": "0123456789abcdef0123456789abcdef",
        "request_sequence": 17,
        "source_monotonic_ns": source_ns,
        "kind": "execute",
        "session_id": "persistent:0123456789abcdef",
        "state_instance_id": "fedcba9876543210fedcba9876543210",
        "session_pose_sha256": session_pose_hash,
        "moving_joint_mask": [True, False, False, False, False, False],
        "start_relative_rad": start,
        "target_relative_rad": target,
        "target_sha256": hashlib.sha256(struct.pack(">6d", *target)).hexdigest(),
        "collision_margin_deg": 2.0,
    }
    result = {
        "schema": "go-m8010-collision-guard-result/1.0",
        "source_instance_id": request["source_instance_id"],
        "request_sequence": request["request_sequence"],
        "source_monotonic_ns": request["source_monotonic_ns"],
        "kind": request["kind"],
        "session_id": request["session_id"],
        "state_instance_id": request["state_instance_id"],
        "session_pose_sha256": request["session_pose_sha256"],
        "moving_joint_mask": list(request["moving_joint_mask"]),
        "start_relative_rad": list(request["start_relative_rad"]),
        "target_relative_rad": list(request["target_relative_rad"]),
        "target_sha256": request["target_sha256"],
        "model_sha256": MODEL_SHA256,
        "collision_contract_sha256": COLLISION_CONTRACT_SHA256,
        "kinematic_guard_sha256": KINEMATIC_GUARD_SHA256,
        "collision_margin_deg": request["collision_margin_deg"],
        "ground_contact_policy": "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION",
        "max_step_deg": 0.25,
        "boundary_tolerance_deg": 0.01,
        "hardware_state_sequence": 91,
        "hardware_state_source_monotonic_ns": source_ns - 1_000,
        "hardware_position_rad": list(start),
        "hardware_velocity_rad_s": [0.0] * 6,
        "hardware_controller_mode_by_motor": {
            name: "hold"
            for name in ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
        },
        "margin_policy": "SINGLE_JOINT_EXTENDED_SWEEP_SUPPORT_CROSS_FINAL_LINF_V2",
        "joint_space_tube_radius_deg": 2.25,
        "hold_tracking_tolerance_deg": 0.25,
        "tube_probe_joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
        "tube_grid_offsets": [-1, 0, 1],
        "tube_grid_max_pose_count": 729,
        "tube_axis_probe_step_deg": 0.25,
        "tube_cross_section_max_pose_count": 825,
        "absolute_joint_limits_deg": [list(bounds) for bounds in ABSOLUTE_MODEL_LIMITS_DEG],
        "single_joint_path_required": True,
        "moving_joint": "J1",
        "moving_joint_count": 1,
        "step_count": 1,
        "path_sample_count": 1,
        "pose_check_count": 1,
        "tube_probe_count": 1,
        "tube_probe_cache_hit_count": 0,
        "checked_monotonic_ns": time.monotonic_ns(),
        "safe": safe,
        "reason": "clear" if safe else "self_collision_margin",
        "contact_pairs": [] if safe else [["link2", "link4"]],
        "recommended_relative_rad": list(target if safe else start),
    }
    result["hardware_state_sha256"] = collision_hardware_state_hash({
        "schema": "go-m8010-hardware-state/1.1",
        "session_id": request["session_id"],
        "state_instance_id": request["state_instance_id"],
        "sequence": result["hardware_state_sequence"],
        "source_monotonic_ns": result["hardware_state_source_monotonic_ns"],
        "position_rad": result["hardware_position_rad"],
        "velocity_rad_s": result["hardware_velocity_rad_s"],
        "controller_mode_by_motor": result["hardware_controller_mode_by_motor"],
    })
    return request, result


def test_collision_result_must_match_exact_request_model_and_fresh_proof():
    matches = load_function("collision_guard_result_matches")
    request, result = collision_request_and_result()
    assert matches(result, request)

    for field, replacement in (
        ("request_sequence", 18),
        ("source_monotonic_ns", result["source_monotonic_ns"] + 1),
        ("session_pose_sha256", "0" * 64),
        ("target_sha256", "0" * 64),
        ("model_sha256", "0" * 64),
        ("collision_contract_sha256", "0" * 64),
        ("kinematic_guard_sha256", "0" * 64),
        ("collision_margin_deg", 1.99),
        ("ground_contact_policy", "SAFE"),
        ("max_step_deg", 0.251),
        ("boundary_tolerance_deg", 0.011),
        ("safe", 1),
    ):
        mismatched = dict(result)
        mismatched[field] = replacement
        assert not matches(mismatched, request)

    stale = dict(result)
    stale["checked_monotonic_ns"] = time.monotonic_ns() - 8_000_000_001
    assert not matches(stale, request)
    altered_path = dict(result)
    altered_path["target_relative_rad"] = list(result["target_relative_rad"])
    altered_path["target_relative_rad"][2] += 1.0e-11
    assert not matches(altered_path, request)
    contradictory = dict(result)
    contradictory["reason"] = "self_collision"
    assert not matches(contradictory, request)


def test_multi_joint_pose_preview_is_valid_but_cannot_be_an_execute_proof():
    matches = load_function("collision_guard_result_matches")
    request, result = collision_request_and_result()
    target = [0.01, -0.02, 0.03, 0.0, 0.0, 0.0]
    moving_mask = [True, True, True, False, False, False]
    target_digest = hashlib.sha256(struct.pack(">6d", *target)).hexdigest()
    request.update(
        kind="pose_preview",
        moving_joint_mask=moving_mask,
        target_relative_rad=target,
        target_sha256=target_digest,
    )
    result.update(
        kind="pose_preview",
        moving_joint_mask=list(moving_mask),
        target_relative_rad=list(target),
        target_sha256=target_digest,
        recommended_relative_rad=list(target),
        moving_joint=None,
        moving_joint_count=3,
    )
    assert matches(result, request)

    # The same multi-axis document can never masquerade as the authorizing
    # one-axis execute proof consumed by the command router.
    execute_request = dict(request)
    execute_request["kind"] = "execute"
    execute_result = dict(result)
    execute_result["kind"] = "execute"
    assert not matches(execute_result, execute_request)


def test_collision_request_is_asynchronous_and_does_not_mutate_drive_authority():
    arm_mode = SimpleNamespace(SIM_TO_REAL=object())
    target_hash = load_function("collision_target_sha256")
    limits_valid = load_function("moving_targets_within_model_limits")
    source_id_valid = lambda value: bool(
        isinstance(value, str)
        and len(value) == 32
        and all(character in "0123456789abcdef" for character in value)
    )
    make_request = load_main_window_method(
        "_new_collision_request",
        {
            "ArmMode": arm_mode,
            "COLLISION_MARGIN_DEG": 2.0,
            "collision_target_sha256": target_hash,
            "moving_targets_within_model_limits": limits_valid,
            "collision_motion_state_ready": load_collision_motion_state_ready(),
            "source_instance_id_valid": source_id_valid,
            "time": time,
        },
    )

    class FakeNode:
        command_source_instance_id = "0123456789abcdef0123456789abcdef"

        def __init__(self):
            self.published = []
            self.latest_hardware = hardware_state()
            self.latest_hardware["state_instance_id"] = (
                "fedcba9876543210fedcba9876543210"
            )

        def control_streams_fresh(self):
            return True

        def publish_collision_request(self, payload):
            self.published.append(dict(payload))

    class FakeWindow:
        def __init__(self):
            self.direction = arm_mode.SIM_TO_REAL
            self.session_id = "persistent:0123456789abcdef"
            self.state_instance_id = "fedcba9876543210fedcba9876543210"
            self.targets = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.actual = [0.0] * 6
            self.edit_limits = list(SESSION_MODEL_LIMITS_DEG)
            self.collision_request_sequence = 0
            self.collision_requests = {}
            self.node = FakeNode()
            self.hardware_mode = "hold"
            self.command_targets = [0.0] * 6
            self.requested_active_joint_mask = [True] * 6
            self.pending_target_joint_mask = [True, False, False, False, False, False]
            self.moving_joint_mask = [False] * 6
            self.command_stream_suspended = False
            self.session_pose_sha256 = "c" * 64
            self.activation_epoch = 1234

    window = FakeWindow()
    authority_before = (
        window.hardware_mode,
        list(window.command_targets),
        list(window.requested_active_joint_mask),
        list(window.moving_joint_mask),
        window.activation_epoch,
    )
    request = make_request(window, "execute")
    assert request is not None
    assert request["schema"] == "go-m8010-collision-guard-request/1.0"
    assert request["request_sequence"] == 1
    assert request["kind"] == "execute"
    assert request["collision_margin_deg"] == 2.0
    assert request["start_relative_rad"] == window.node.latest_hardware["position_rad"]
    assert request["session_pose_sha256"] == window.session_pose_sha256
    assert request["target_relative_rad"] == window.targets
    assert request["target_sha256"] == target_hash(window.targets)
    assert window.node.published == [request]
    assert (
        window.hardware_mode,
        window.command_targets,
        window.requested_active_joint_mask,
        window.moving_joint_mask,
        window.activation_epoch,
    ) == authority_before


def test_multi_joint_virtual_edit_requests_only_non_authorizing_pose_preview():
    preview = load_main_window_method("_request_collision_preview")

    class FakeWindow:
        def __init__(self, moving_mask):
            self.pending_target_joint_mask = list(moving_mask)
            self.latest_collision_preview_sequence = None
            self.kinds = []

        def _new_collision_request(self, kind):
            self.kinds.append(kind)
            return {"request_sequence": len(self.kinds)}

    multi = FakeWindow([True, False, True, False, False, True])
    preview(multi)
    assert multi.kinds == ["pose_preview"]
    assert multi.latest_collision_preview_sequence == 1

    single = FakeWindow([False, True, False, False, False, False])
    preview(single)
    assert single.kinds == ["preview"]

    none = FakeWindow([False] * 6)
    preview(none)
    assert none.kinds == []


def test_collision_recommendation_never_turns_support_tracking_error_into_edit():
    recommend = load_main_window_method(
        "_collision_recommendation",
        {
            "moving_targets_within_model_limits": load_function(
                "moving_targets_within_model_limits"
            ),
        },
    )

    class FakeWindow:
        edit_limits = list(SESSION_MODEL_LIMITS_DEG)

    command_target = [0.0, 0.20, -0.30, 0.10, -0.15, 0.05]
    measured_start = [0.0, 0.203, -0.297, 0.102, -0.148, 0.052]
    request = {
        "moving_joint_mask": [True, False, False, False, False, False],
        "start_relative_rad": measured_start,
        "target_relative_rad": [0.40] + command_target[1:],
    }
    result = {"recommended_relative_rad": list(measured_start)}
    observed = recommend(FakeWindow(), result, request)
    assert observed[0] == measured_start[0]
    assert observed[1:] == command_target[1:]


def test_collision_rejection_rolls_back_only_virtual_edit_state_not_servo_authority():
    reject = load_main_window_method(
        "_apply_collision_rejection",
        {
            "COLLISION_PREVIEW_DEBOUNCE_MS": 60,
            "TARGET_SELECTION_DEADBAND_RAD": math.radians(0.01),
        },
    )

    class FakeTimer:
        def __init__(self):
            self.starts = []

        def start(self, milliseconds):
            self.starts.append(milliseconds)

    class FakeWindow:
        def __init__(self):
            self.targets = [0.4, -0.3, 0.2, -0.1, 0.05, -0.02]
            self.actual = [0.0] * 6
            self.connected = [True] * 6
            self.pending_target_joint_mask = [True] * 6
            self.command_targets = [0.0] * 6
            self.hardware_mode = "position"
            self.requested_active_joint_mask = [True, True, False, True, True, True]
            self.moving_joint_mask = [True, False, False, True, False, False]
            self.activation_epoch = 987654321
            self.collision_preview_timer = FakeTimer()
            self.events = []

        def _collision_recommendation(self, _result, request):
            return list(request["start_relative_rad"])

        def _show_targets_on_virtual(self):
            self.events.append("virtual-render")

        def _show_collision_popup(self, _result):
            self.events.append("nonmodal-popup")

        def _notify(self, text, level):
            self.events.append((text, level))

        def _publish_command(self):
            raise AssertionError("collision rejection must not publish BRAKE or any command")

    window = FakeWindow()
    request = {
        "target_relative_rad": list(window.targets),
        "start_relative_rad": list(window.actual),
        "moving_joint_mask": [True, False, False, False, False, False],
    }
    authoritative_before = (
        list(window.command_targets),
        window.hardware_mode,
        list(window.requested_active_joint_mask),
        list(window.moving_joint_mask),
        window.activation_epoch,
    )
    reject(window, {"reason": "self_collision_margin"}, request)
    assert window.targets == window.actual
    assert window.pending_target_joint_mask == [False] * 6
    assert (
        window.command_targets,
        window.hardware_mode,
        window.requested_active_joint_mask,
        window.moving_joint_mask,
        window.activation_epoch,
    ) == authoritative_before
    # Rejection is fail-closed and waits for a new operator edit; it must not
    # create an automatic 60 ms re-submit loop for the same unsafe pose.
    assert window.collision_preview_timer.starts == []
    assert window.events[0:2] == ["virtual-render", "nonmodal-popup"]
    assert "原位置伺服保持继续" in window.events[2][0]

    source = SOURCE.read_text(encoding="utf-8")
    start = source.index("    def _apply_collision_rejection(")
    end = source.index("    def _consume_collision_guard_result", start)
    rejection = source[start:end]
    for forbidden in (
        "self.command_targets =",
        "self.hardware_mode =",
        "self.requested_active_joint_mask =",
        "self.moving_joint_mask =",
        "self.activation_epoch =",
        "self.machine.stop(",
        "self._publish_command(",
    ):
        assert forbidden not in rejection


def test_only_a_matching_safe_execute_result_reaches_the_commit_step():
    matches = load_function("collision_guard_result_matches")
    consume = load_main_window_method(
        "_consume_collision_guard_result",
        {
            "COLLISION_START_MATCH_TOLERANCE_RAD": math.radians(0.25),
            "collision_guard_result_matches": matches,
        },
    )
    request, result = collision_request_and_result(safe=True)

    class FakeWindow:
        def __init__(self, collision_result, checked_request=request):
            self.node = SimpleNamespace(latest_collision_result=collision_result)
            self.collision_requests = {
                checked_request["request_sequence"]: checked_request
            }
            self.last_consumed_collision_sequence = 0
            self.latest_collision_preview_sequence = None
            self.pending_collision_execute_sequence = checked_request[
                "request_sequence"
            ]
            self.session_id = checked_request["session_id"]
            self.state_instance_id = checked_request["state_instance_id"]
            self.targets = list(checked_request["target_relative_rad"])
            self.actual = list(checked_request["start_relative_rad"])
            self.queued_pose_target = None
            self.commits = []
            self.rejections = []
            self.notices = []

        def _commit_checked_target(self, checked, approved_result):
            self.commits.append((checked, approved_result))
            return True

        def _apply_collision_rejection(self, collision_result, checked):
            self.rejections.append((collision_result, checked))

        def _notify(self, text, level):
            self.notices.append((text, level))

        def _refresh_virtual_editability(self):
            pass

    approved = FakeWindow(result)
    consume(approved)
    assert approved.commits == [(request, result)]
    assert approved.rejections == []
    assert approved.pending_collision_execute_sequence is None

    mismatched = dict(result)
    mismatched["target_sha256"] = "0" * 64
    rejected_identity = FakeWindow(mismatched)
    consume(rejected_identity)
    assert rejected_identity.commits == []
    assert rejected_identity.pending_collision_execute_sequence == request["request_sequence"]

    unsafe_request, unsafe = collision_request_and_result(safe=False)
    collision = FakeWindow(unsafe, unsafe_request)
    consume(collision)
    assert collision.commits == []
    assert collision.rejections == [(unsafe, unsafe_request)]


@pytest.mark.parametrize(
    "failure_case",
    (
        "unsafe_collision",
        "unsafe_guard_failure",
        "pose_changed",
        "session_changed",
        "commit_failed",
    ),
)
def test_execute_proof_failure_clears_queue_and_late_safe_cannot_commit(
    failure_case,
):
    matches = load_function("collision_guard_result_matches")
    consume = load_main_window_method(
        "_consume_collision_guard_result",
        {"collision_guard_result_matches": matches},
    )
    cancel = load_main_window_method("_cancel_queued_pose")
    abort = load_main_window_method("_abort_queued_pose_after_collision")
    request, result = collision_request_and_result(
        safe=failure_case not in {"unsafe_collision", "unsafe_guard_failure"}
    )
    if failure_case == "unsafe_guard_failure":
        result["reason"] = "guard_internal_failure"
        result["contact_pairs"] = []

    class FakeTimer:
        def stop(self):
            pass

    class FakeWindow:
        _cancel_queued_pose = cancel
        _abort_queued_pose_after_collision = abort

        def __init__(self):
            self.node = SimpleNamespace(latest_collision_result=result)
            self.collision_requests = {request["request_sequence"]: request}
            self.last_consumed_collision_sequence = 0
            self.latest_collision_preview_sequence = None
            self.pending_collision_execute_sequence = request["request_sequence"]
            self.session_id = request["session_id"]
            self.state_instance_id = request["state_instance_id"]
            self.actual = list(request["start_relative_rad"])
            self.command_targets = list(request["start_relative_rad"])
            self.targets = list(request["target_relative_rad"])
            self.pending_target_joint_mask = [True, False, False, False, False, False]
            self.queued_pose_target = list(request["target_relative_rad"])
            self.queued_joint_indices = [1, 2]
            self.active_sequence_joint = 0
            self.collision_preview_timer = FakeTimer()
            self.commits = []
            self.notices = []
            self.popups = 0

        def _commit_checked_target(self, checked, approved):
            self.commits.append((checked, approved))
            return failure_case != "commit_failed"

        def _show_targets_on_virtual(self):
            pass

        def _refresh_virtual_editability(self):
            pass

        def _show_collision_popup(self, _result):
            self.popups += 1

        def _notify(self, message, level="warning"):
            self.notices.append((message, level))

    window = FakeWindow()
    if failure_case == "pose_changed":
        window.actual[0] += math.radians(0.251)
    elif failure_case == "session_changed":
        window.session_id = "persistent:changed-session"

    consume(window)
    assert window.pending_collision_execute_sequence is None
    assert window.queued_pose_target is None
    assert window.queued_joint_indices == []
    assert window.active_sequence_joint is None
    assert window.targets == window.command_targets
    assert window.pending_target_joint_mask == [False] * 6
    initial_commit_count = len(window.commits)
    assert initial_commit_count == (1 if failure_case == "commit_failed" else 0)

    # Even a later valid-looking safe result for the cancelled sequence cannot
    # regain POSITION authority or restart the discarded virtual plan.
    late_safe = dict(result)
    late_safe.update({
        "safe": True,
        "reason": "clear",
        "contact_pairs": [],
        "recommended_relative_rad": list(request["target_relative_rad"]),
        "checked_monotonic_ns": time.monotonic_ns(),
    })
    window.node.latest_collision_result = late_safe
    consume(window)
    assert len(window.commits) == initial_commit_count
    assert window.queued_pose_target is None


def test_collision_timeout_clears_queue_and_late_safe_cannot_commit():
    matches = load_function("collision_guard_result_matches")
    expire = load_main_window_method("_expire_collision_request")
    consume = load_main_window_method(
        "_consume_collision_guard_result",
        {"collision_guard_result_matches": matches},
    )
    cancel = load_main_window_method("_cancel_queued_pose")
    request, result = collision_request_and_result()
    now = time.monotonic()
    request["source_monotonic_ns"] = int((now - 9.0) * 1.0e9)
    result["source_monotonic_ns"] = request["source_monotonic_ns"]
    result["checked_monotonic_ns"] = time.monotonic_ns()

    class FakeTimer:
        def stop(self):
            pass

    class FakeWindow:
        _cancel_queued_pose = cancel

        def __init__(self):
            self.node = SimpleNamespace(latest_collision_result=result)
            self.collision_requests = {request["request_sequence"]: request}
            self.last_consumed_collision_sequence = 0
            self.latest_collision_preview_sequence = None
            self.pending_collision_execute_sequence = request["request_sequence"]
            self.session_id = request["session_id"]
            self.state_instance_id = request["state_instance_id"]
            self.actual = list(request["start_relative_rad"])
            self.command_targets = list(request["start_relative_rad"])
            self.targets = list(request["target_relative_rad"])
            self.pending_target_joint_mask = [True] + [False] * 5
            self.queued_pose_target = list(request["target_relative_rad"])
            self.queued_joint_indices = [1]
            self.active_sequence_joint = 0
            self.collision_preview_timer = FakeTimer()
            self.notices = []

        def _show_targets_on_virtual(self):
            pass

        def _refresh_virtual_editability(self):
            pass

        def _notify(self, message, level="warning"):
            self.notices.append((message, level))

        def _commit_checked_target(self, *_args):
            raise AssertionError("a timed-out collision result must never commit")

    window = FakeWindow()
    expire(window, now)
    assert window.pending_collision_execute_sequence is None
    assert window.queued_pose_target is None
    assert window.queued_joint_indices == []
    assert window.active_sequence_joint is None
    assert window.targets == window.command_targets
    assert any("超时" in notice[0] for notice in window.notices)

    consume(window)
    assert window.queued_pose_target is None


def test_suspended_gui_still_publishes_status_and_virtual_preview_only():
    source = SOURCE.read_text(encoding="utf-8")
    start = source.index("    def _publish_command(self) -> bool:")
    end = source.index("    def _refresh_safety_notice", start)
    publish = source[start:end]
    suspended = publish[
        publish.index("if self.command_stream_suspended:"):
        publish.index("self.command_sequence += 1")
    ]
    assert "self.node.mode_publisher.publish" in publish
    assert "command_stream_suspended_reason" in publish
    assert "self.node.target_publisher.publish" in suspended
    assert "self.node.publish_command(" not in suspended


def test_position_stop_preserves_fixed_axes_and_captures_only_moving_axes():
    stop_targets = load_function("fixed_hold_targets_after_position_stop")
    locked = [0.0, -1.0, 2.0, 3.0, 4.0, 5.0]
    displaced = [0.2, -0.8, 2.3, 3.4, 4.5, 5.6]
    active = [True, True, True, False, True, True]
    moving = [True, False, False, False, True, False]
    assert stop_targets(locked, displaced, active, moving) == [
        0.2, -1.0, 2.0, 3.4, 4.5, 5.0,
    ]


def test_complete_pose_actions_require_all_six_healthy():
    ready = load_function("control_feedback_ready")
    assert not ready(True, True, [True, True, True, True, True, False], require_all=True)
    assert ready(True, True, [True] * 6, require_all=True)


def test_active_mask_includes_connected_j2_and_excludes_disconnected_joints():
    mask = load_function("effective_active_joint_mask")
    assert mask(
        [True] * 6,
        [True, True, False, True, False, True],
        "hold",
    ) == [True, True, False, True, False, True]


def test_brake_always_clears_active_mask():
    mask = load_function("effective_active_joint_mask")
    assert mask([True] * 6, [True] * 6, "brake") == [False] * 6


def test_connection_loss_clears_latched_bit_and_recovery_does_not_restore_it():
    clear = load_function("clear_mask_on_connection_loss")
    dropped = clear(
        [False, False, False, True, False, False],
        [True] * 6,
        [True, True, True, False, True, True],
    )
    assert dropped == [False] * 6
    recovered = clear(dropped, [True, True, True, False, True, True], [True] * 6)
    assert recovered == [False] * 6


def test_initially_disconnected_joint_cannot_be_prearmed_for_recovery():
    mask = load_function("effective_active_joint_mask")
    connected_at_action = [True, True, True, False, True, True]
    requested = list(connected_at_action)
    assert mask(requested, connected_at_action, "hold")[3] is False
    assert mask(requested, [True] * 6, "hold")[3] is False
    assert "self.requested_active_joint_mask = [True] * 6" not in SOURCE.read_text(
        encoding="utf-8"
    )


def test_target_edit_uses_immutable_command_not_external_feedback_as_baseline():
    record = load_main_window_method(
        "_record_virtual_target",
        {"COLLISION_PREVIEW_DEBOUNCE_MS": 60},
    )

    class FakeTimer:
        def __init__(self):
            self.starts = []

        def start(self, milliseconds):
            self.starts.append(milliseconds)

    class FakeWindow:
        def __init__(self):
            self.targets = [0.0] * 6
            self.command_targets = [0.25] + [0.0] * 5
            self.actual = [0.80] + [0.0] * 5
            self.connected = [True] * 6
            self.pending_target_joint_mask = [False] * 6
            self.activation_epoch = 12345
            self.collision_preview_timer = FakeTimer()
            self.renders = 0

        def _show_targets_on_virtual(self):
            self.renders += 1

    window = FakeWindow()
    record(window, 0, 0.25)
    assert window.pending_target_joint_mask == [False] * 6
    record(window, 0, 0.30)
    assert window.pending_target_joint_mask == [True, False, False, False, False, False]
    assert window.command_targets[0] == 0.25
    assert window.actual[0] == 0.80
    assert window.activation_epoch == 12345
    assert window.collision_preview_timer.starts == [60, 60]


def test_execute_consumes_selection_and_connection_loss_clears_pending_bit():
    source = SOURCE.read_text(encoding="utf-8")
    execute_start = source.index("    def _execute_target(self) -> None:")
    execute_end = source.index("    def _stop(self) -> None:", execute_start)
    execute_source = source[execute_start:execute_end]
    assert "self.pending_target_joint_mask = [False] * 6" in execute_source

    apply_start = source.index("    def _apply_connection_state(")
    apply_end = source.index("    def _authorize_active_joints(", apply_start)
    apply_source = source[apply_start:apply_end]
    assert "self.pending_target_joint_mask = clear_mask_on_connection_loss(" in apply_source
    assert "for index, hard_fault in enumerate(faulted):" in apply_source
    assert "self.requested_active_joint_mask[index] = False" in apply_source


def test_execute_without_pending_target_preserves_active_hold_or_position_authority():
    arm_mode = SimpleNamespace(SIM_TO_REAL=object())
    execute = load_main_window_method("_execute_target", {"ArmMode": arm_mode})

    class FakeWindow:
        def __init__(self, mode):
            self.hardware_mode = mode
            self.direction = arm_mode.SIM_TO_REAL
            self.command_targets = [-0.1, 0.2, -0.3, 0.4, -0.5, 0.6]
            self.targets = list(self.command_targets)
            self.actual = [0.8, -0.7, 0.6, -0.5, 0.4, -0.3]
            self.connected = [True] * 6
            self.requested_active_joint_mask = [True, True, False, True, True, True]
            self.pending_target_joint_mask = [False] * 6
            self.moving_joint_mask = [False] * 6
            self.command_stream_suspended = False
            self.queued_pose_target = None
            self.activation_epoch = 123456
            self.notices = []

        def _require_control_feedback(self, _message, **_kwargs):
            return True

        def _prepare_hold_at_actual(self):
            raise AssertionError("active authoritative target must not be recaptured")

        def _notify(self, message, level="warning"):
            self.notices.append((message, level))

    for mode in ("hold", "position"):
        window = FakeWindow(mode)
        authority_before = (
            list(window.targets),
            list(window.command_targets),
            list(window.requested_active_joint_mask),
            list(window.pending_target_joint_mask),
            list(window.moving_joint_mask),
            window.activation_epoch,
        )
        execute(window)
        authority_after = (
            window.targets,
            window.command_targets,
            window.requested_active_joint_mask,
            window.pending_target_joint_mask,
            window.moving_joint_mask,
            window.activation_epoch,
        )
        assert authority_after == authority_before
        assert window.hardware_mode == mode
        assert len(window.notices) == 1
        expected = (
            "已有POSITION轨迹"
            if mode == "position" else
            "保留现有权威目标与命令流"
        )
        assert expected in window.notices[0][0]


def test_execute_without_pending_target_can_capture_actual_from_brake_or_drag():
    arm_mode = SimpleNamespace(SIM_TO_REAL=object())
    execute = load_main_window_method("_execute_target", {"ArmMode": arm_mode})

    class FakeWindow:
        def __init__(self, mode):
            self.hardware_mode = mode
            self.direction = arm_mode.SIM_TO_REAL
            self.targets = [0.0] * 6
            self.command_targets = [0.0] * 6
            self.connected = [True] * 6
            self.pending_target_joint_mask = [False] * 6
            self.moving_joint_mask = [False] * 6
            self.command_stream_suspended = False
            self.queued_pose_target = None
            self.captures = 0
            self.notices = []

        def _require_control_feedback(self, _message, **_kwargs):
            return True

        def _prepare_hold_at_actual(self):
            self.captures += 1
            self.hardware_mode = "hold"

        def _notify(self, message, level="warning"):
            self.notices.append((message, level))

    for mode in ("brake", "drag"):
        window = FakeWindow(mode)
        execute(window)
        assert window.captures == 1
        assert window.hardware_mode == "hold"
        assert "已锁定当前实际姿态" in window.notices[0][0]


def test_multi_axis_virtual_pose_queues_only_one_physical_segment_at_a_time():
    arm_mode = SimpleNamespace(SIM_TO_REAL=object())
    limits_valid = load_function("moving_targets_within_model_limits")
    readiness = load_collision_motion_state_ready()
    begin = load_main_window_method(
        "_begin_next_queued_segment",
        {"COLLISION_EXECUTE_SEGMENT_MAX_DEG": 30.0},
    )
    execute = load_main_window_method(
        "_execute_target",
        {
            "ArmMode": arm_mode,
            "collision_motion_state_ready": readiness,
            "moving_targets_within_model_limits": limits_valid,
        },
    )

    class FakeWindow:
        _begin_next_queued_segment = begin

        def __init__(self):
            self.hardware_mode = "hold"
            self.direction = arm_mode.SIM_TO_REAL
            self.command_targets = [0.0] * 6
            self.targets = [0.10, 0.0, -0.20, 0.0, 0.0, 0.30]
            self.connected = [True] * 6
            self.requested_active_joint_mask = [True] * 6
            self.pending_target_joint_mask = [False] * 6
            self.moving_joint_mask = [False] * 6
            self.edit_limits = list(SESSION_MODEL_LIMITS_DEG)
            self.command_stream_suspended = False
            self.queued_pose_target = None
            self.queued_joint_indices = []
            self.active_sequence_joint = None
            self.pending_collision_execute_sequence = None
            self.requests = []
            self.notices = []
            self.node = SimpleNamespace(latest_hardware=hardware_state())

        def _require_control_feedback(self, _message, **_kwargs):
            return True

        def _request_collision_execute(self):
            assert self.hardware_mode == "hold"
            assert self.moving_joint_mask == [False] * 6
            assert sum(self.pending_target_joint_mask) == 1
            self.requests.append((
                self.active_sequence_joint,
                list(self.targets),
                list(self.pending_target_joint_mask),
            ))
            self.pending_collision_execute_sequence = 99
            return True

        def _show_targets_on_virtual(self):
            pass

        def _notify(self, message, level="warning"):
            self.notices.append((message, level))

    window = FakeWindow()
    complete_virtual_pose = list(window.targets)
    execute(window)

    assert window.queued_pose_target == complete_virtual_pose
    assert window.queued_joint_indices == [2, 5]
    assert window.active_sequence_joint == 0
    assert len(window.requests) == 1
    assert window.requests[0] == (
        0,
        [0.10, 0.0, 0.0, 0.0, 0.0, 0.0],
        [True, False, False, False, False, False],
    )
    # Collision proof is still pending: no POSITION/moving authority exists.
    assert window.hardware_mode == "hold"
    assert window.moving_joint_mask == [False] * 6


def test_long_joint_move_is_split_into_bounded_fresh_proof_segments():
    begin = load_main_window_method(
        "_begin_next_queued_segment",
        {"COLLISION_EXECUTE_SEGMENT_MAX_DEG": 30.0},
    )

    class FakeWindow:
        def __init__(self):
            self.command_targets = [0.0] * 6
            self.targets = [0.0] * 6
            self.pending_target_joint_mask = [False] * 6
            self.queued_pose_target = [math.radians(75.0)] + [0.0] * 5
            self.queued_joint_indices = [0]
            self.active_sequence_joint = None
            self.pending_collision_execute_sequence = None
            self.requested = []

        def _show_targets_on_virtual(self):
            pass

        def _request_collision_execute(self):
            self.requested.append(math.degrees(self.targets[0]))
            self.pending_collision_execute_sequence = len(self.requested)
            return True

        def _notify(self, *_args):
            pass

        def _cancel_queued_pose(self, **_kwargs):
            raise AssertionError("bounded sequence must remain active")

    window = FakeWindow()
    for expected in (30.0, 60.0, 75.0):
        begin(window)
        assert window.requested[-1] == pytest.approx(expected)
        assert window.pending_target_joint_mask == [True, False, False, False, False, False]
        window.command_targets[0] = window.targets[0]
        window.active_sequence_joint = None
        window.pending_collision_execute_sequence = None

    assert window.requested == pytest.approx([30.0, 60.0, 75.0])
    assert window.queued_joint_indices == []


def test_matching_safe_execute_result_commits_exactly_one_moving_axis():
    arm_mode = SimpleNamespace(SIM_TO_REAL=object())
    target_hash = load_function("collision_target_sha256")
    limits_valid = load_function("moving_targets_within_model_limits")
    matches = load_function("collision_guard_result_matches")
    commit = load_main_window_method(
        "_commit_checked_target",
        {
            "ArmMode": arm_mode,
            "collision_target_sha256": target_hash,
            "collision_guard_result_matches": matches,
            "collision_motion_state_ready": load_collision_motion_state_ready(),
            "moving_targets_within_model_limits": limits_valid,
            "json": json,
            "time": time,
        },
    )
    events = []

    class FakeMachine:
        def position(self):
            events.append(("position",))

    class FakeWindow:
        def __init__(self):
            self.hardware_mode = "hold"
            self.direction = arm_mode.SIM_TO_REAL
            self.targets = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.command_targets = [0.0] * 6
            self.connected = [True] * 6
            self.requested_active_joint_mask = [True] * 6
            self.edit_limits = list(SESSION_MODEL_LIMITS_DEG)
            self.pending_target_joint_mask = [True, False, False, False, False, False]
            self.moving_joint_mask = [False] * 6
            self.active_sequence_joint = 0
            self.command_stream_suspended = False
            self.node = SimpleNamespace(latest_hardware=hardware_state())
            self.machine = FakeMachine()
            self.arrival = SimpleNamespace(
                start=lambda started_at: events.append(("arrival", started_at))
            )
            self.authorized = []
            self.resumed = 0

        def _require_control_feedback(self, _message, **_kwargs):
            return True

        def _authorize_active_joints(self, connected):
            self.authorized.append(list(connected))

        def _resume_command_stream(self):
            self.resumed += 1

        def _refresh_virtual_editability(self):
            pass

        def _update_mode_label(self, message):
            events.append(("label", message))

        def _notify(self, *_args):
            raise AssertionError("matching safe target must not be rejected")

    window = FakeWindow()
    request, result = collision_request_and_result(target=window.targets)
    assert commit(window, request, result)

    expected = [True, False, False, False, False, False]
    assert events[0] == ("position",)
    assert window.moving_joint_mask == expected
    assert window.command_targets == window.targets
    assert window.pending_target_joint_mask == [False] * 6
    assert window.authorized == [[True] * 6]
    assert window.resumed == 1
    assert window.active_collision_proof == result


def test_queued_segment_arrival_forces_exact_target_hold_and_waits_for_all_hold():
    arm_mode = SimpleNamespace(SIM_TO_REAL=object(), REAL_TO_SIM=object())
    refresh = load_main_window_method(
        "_refresh_joint_widgets",
        {
            "ArmMode": arm_mode,
            "RAD": math.pi / 180.0,
            "DEG": 180.0 / math.pi,
            "effective_active_joint_mask": load_function(
                "effective_active_joint_mask"
            ),
            "set_widget_value_if_changed": (
                lambda widget, value: widget.setValue(value)
            ),
            "set_widget_text_if_changed": (
                lambda widget, value: widget.setText(value)
            ),
        },
    )
    continue_queue = load_main_window_method(
        "_continue_queued_sequence_if_ready",
        {"collision_motion_state_ready": load_collision_motion_state_ready()},
    )

    class Sink:
        def setValue(self, _value):
            pass

        def setText(self, _value):
            pass

    class FakeArrival:
        def __init__(self):
            self.starts = []

        def update(self, _now, _errors, _connected):
            return ["已到位"] * 6

        def start(self, now):
            self.starts.append(now)

    class FakeMachine:
        fixed_hold_after_arrival = False

        def __init__(self):
            self.holds = 0

        def hold(self):
            self.holds += 1

    class FakeNode:
        def __init__(self, state):
            self.latest_hardware = state

        def control_streams_fresh(self, _now=None):
            return True

    class FakeWindow:
        def __init__(self):
            self.hardware_mode = "position"
            self.command_targets = [0.40, 0.0, 0.0, 0.0, 0.0, 0.0]
            # Simulate an external-force offset after reaching the endpoint;
            # the HOLD transition must not capture this measured value.
            self.actual = [0.65, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.targets = list(self.command_targets)
            self.connected = [True] * 6
            self.faulted = [False] * 6
            self.observation_uncertain = [False] * 6
            self.requested_active_joint_mask = [True] * 6
            self.pending_target_joint_mask = [False] * 6
            self.moving_joint_mask = [True, False, False, False, False, False]
            self.active_sequence_joint = 0
            self.active_collision_proof = {"safe": True}
            self.queued_pose_target = [0.40, 0.0, -0.20, 0.0, 0.0, 0.0]
            self.queued_joint_indices = [2]
            self.pending_collision_execute_sequence = None
            self.command_stream_suspended = False
            state = hardware_state()
            state["position_rad"] = list(self.command_targets)
            state["controller_mode_by_motor"]["J1"] = "position"
            self.node = FakeNode(state)
            self.arrival = FakeArrival()
            self.machine = FakeMachine()
            self.direction = arm_mode.SIM_TO_REAL
            self.real_widgets = [
                SimpleNamespace(
                    slider=Sink(), actual=Sink(), target=Sink(),
                    error=Sink(), state=Sink(),
                )
                for _ in range(6)
            ]
            self.labels = []
            self.next_segments = 0

        def _confirmed_joint_modes(self):
            return ["position"] + ["hold"] * 5

        def _show_targets_on_virtual(self):
            pass

        def _update_mode_label(self, message):
            self.labels.append(message)

        def _begin_next_queued_segment(self):
            self.next_segments += 1

    window = FakeWindow()
    refresh(window)

    assert window.machine.holds == 1
    assert window.hardware_mode == "hold"
    assert window.command_targets[0] == 0.40
    assert window.targets[0] == 0.40
    assert window.actual[0] == 0.65
    assert window.moving_joint_mask == [False] * 6
    assert window.active_sequence_joint is None
    assert window.active_collision_proof is None
    assert "未重采样反馈" in window.labels[-1]

    # Local HOLD request is insufficient: all seven motors must report HOLD
    # and stationary at the exact target before a fresh next proof is started.
    continue_queue(window)
    assert window.next_segments == 0
    window.node.latest_hardware["controller_mode_by_motor"]["J1"] = "hold"
    window.node.latest_hardware["source_monotonic_ns"] = time.monotonic_ns()
    continue_queue(window)
    assert window.next_segments == 1


def test_invalid_or_stale_hardware_observation_cannot_publish_empty_mask_brake():
    source = SOURCE.read_text(encoding="utf-8")
    callback_start = source.index("    def _hardware_callback(")
    callback_end = source.index("    def _mujoco_callback(", callback_start)
    callback = source[callback_start:callback_end]
    assert callback.index("hardware_state_contract_valid(value)") < callback.index(
        "self.last_hardware_receipt = time.monotonic()"
    )

    update_start = source.index("    def _update_connected(")
    update_end = source.index("    def _apply_connection_state(", update_start)
    update = source[update_start:update_end]
    assert "uncertain_active = [" in update
    assert "preserved_requested = list(self.requested_active_joint_mask)" in update
    assert "self.requested_active_joint_mask[index] = preserved_requested[index]" in update
    assert "self._suspend_command_stream(" in update
    assert "不以BRAKE覆盖" in update


def test_command_protocol_requires_versioned_per_joint_mask():
    source = SOURCE.read_text(encoding="utf-8")
    assert '"schema": "go-m8010-gui-command/1.2"' in source
    assert '"active_joint_mask": [False] * 6 if is_brake else active_joint_mask' in source
    assert '"moving_joint_mask": [False] * 6 if is_brake else moving_joint_mask' in source
    assert '"activation_epoch": 0 if is_brake else activation_epoch' in source
    assert '"source_instance_id": self.command_source_instance_id' in source
    assert '"source_monotonic_ns": time.monotonic_ns()' in source


def test_gui_brake_payload_is_normalized_and_position_payload_is_policy_independent():
    class Message:
        def __init__(self, data):
            self.data = data

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    publish = load_class_method(
        "ArmGuiNode",
        "publish_command",
        {
            "time": time,
            "json": json,
            "String": Message,
            "Float64MultiArray": Message,
            "RAD": math.pi / 180.0,
        },
    )
    config = {
        "控制": {
            "最大速度_度每秒": 5.0,
            "最大加速度_度每二次方秒": 20.0,
        },
        "关节": {
            f"J{index}": {"Kp": float(index), "Kd": float(index) / 10.0}
            for index in range(1, 7)
        },
    }

    def publish_once(mode, position_proof=None):
        fake = SimpleNamespace(
            command_source_instance_id="0123456789abcdef0123456789abcdef",
            command_publisher=Publisher(),
            target_publisher=Publisher(),
        )
        proof = None
        if mode == "position":
            proof = position_proof
            if proof is None:
                _, proof = collision_request_and_result(target=[0.1] * 6)
        publish(
            fake,
            9,
            mode,
            [0.1] * 6,
            [0.2] * 6,
            [True] * 6,
            [True] * 6,
            77,
            config,
            proof,
        )
        return json.loads(fake.command_publisher.messages[-1].data)

    brake = publish_once("brake")
    assert brake["mode"] == "brake"
    assert brake["targets_rad"] == [0.0] * 6
    assert brake["active_joint_mask"] == [False] * 6
    assert brake["moving_joint_mask"] == [False] * 6
    assert brake["activation_epoch"] == 0
    assert brake["kp"] == [0.0] * 6
    assert brake["kd"] == [0.0] * 6

    # The post-arrival policy is intentionally not a wire-level motion gate.
    # Both policy states therefore publish this exact same POSITION document.
    _, shared_position_proof = collision_request_and_result(target=[0.1] * 6)
    position_policy_on = publish_once("position", shared_position_proof)
    position_policy_off = publish_once("position", shared_position_proof)
    position_policy_on.pop("source_monotonic_ns")
    position_policy_off.pop("source_monotonic_ns")
    assert position_policy_on == position_policy_off
    assert position_policy_on["mode"] == "position"
    assert position_policy_on["targets_rad"] == [0.1] * 6
    assert position_policy_on["active_joint_mask"] == [True] * 6
    assert position_policy_on["moving_joint_mask"] == [True] * 6
    assert position_policy_on["collision_guard_proof"]["safe"] is True


def test_hardware_callback_rejects_stale_duplicate_or_out_of_order_state():
    source = SOURCE.read_text(encoding="utf-8")
    callback_start = source.index("    def _hardware_callback(")
    callback_end = source.index("    def _mujoco_callback(", callback_start)
    callback = source[callback_start:callback_end]
    assert "hardware_state_source_is_fresh(value, now_ns)" in callback
    assert "source != self.active_hardware_state_instance_id" in callback
    assert "<= HARDWARE_STATE_SOURCE_TAKEOVER_TIMEOUT_NS" in callback
    assert "sequence <= previous_sequence" in callback
    assert callback.index("sequence <= previous_sequence") < callback.index(
        "self.latest_hardware = value"
    )


def test_gui_uses_atomic_hardware_snapshot_for_control_position():
    source = SOURCE.read_text(encoding="utf-8")
    tick_start = source.index("    def _tick(self) -> None:")
    tick_end = source.index("    def _ros_context_ok", tick_start)
    tick = source[tick_start:tick_end]
    assert 'hardware["position_rad"]' in tick
    assert "latest_joint_state" not in tick


def test_ros_operation_is_skipped_after_shutdown():
    run = load_function("run_ros_context_operation")
    calls = []
    assert not run(lambda: False, lambda: calls.append("called"))
    assert calls == []


def test_shutdown_race_converts_ros_exception_to_clean_exit():
    run = load_function("run_ros_context_operation")
    states = iter((True, False))

    def fail_during_shutdown():
        raise RuntimeError("ROS context is not valid")

    assert not run(lambda: next(states), fail_during_shutdown)


def test_normal_runtime_exception_is_not_swallowed():
    run = load_function("run_ros_context_operation")

    def fail_while_running():
        raise RuntimeError("unexpected GUI failure")

    try:
        run(lambda: True, fail_while_running)
    except RuntimeError as error:
        assert str(error) == "unexpected GUI failure"
    else:
        raise AssertionError("运行时异常被错误吞掉")


def test_shutdown_after_successful_ros_operation_requests_exit():
    run = load_function("run_ros_context_operation")
    states = iter((True, False))
    calls = []
    assert not run(lambda: next(states), lambda: calls.append("called"))
    assert calls == ["called"]


def test_router_status_reports_ack_rejections_age_and_mode_without_claiming_hardware_confirmation():
    render = load_function("command_router_status_text")
    text, level = render(
        {
            "received": True,
            "rejected_commands": 0,
            "last_rejection_age_ms": None,
            "last_command_age_ms": 23.5,
            "last_mode": "hold",
        },
        True,
        500.0,
        "hold",
    )
    assert level == "normal"
    assert "收到=是" in text
    assert "拒绝=0" in text
    assert "命令年龄=24毫秒" in text
    assert "最近模式=hold" in text
    assert "控制器" not in text


def test_expired_command_is_a_persistent_critical_state():
    render = load_function("command_router_status_text")
    text, level = render(
        {
            "received": True,
            "rejected_commands": 17,
            "last_rejection_age_ms": 100.0,
            "last_command_age_ms": 740.0,
            "last_mode": "hold",
        },
        True,
        500.0,
        "hold",
    )
    assert level == "critical"
    assert "拒绝=17" in text and "命令年龄=740毫秒" in text


def test_historical_router_rejection_does_not_create_permanent_warning():
    render = load_function("command_router_status_text")
    text, level = render(
        {
            "received": True,
            "rejected_commands": 17,
            "last_rejection_age_ms": 15000.0,
            "last_command_age_ms": 20.0,
            "last_mode": "hold",
        },
        True,
        500.0,
        "hold",
    )
    assert level == "normal"
    assert "拒绝=17" in text and "最近拒绝=15000毫秒前" in text


def test_recent_router_rejection_is_a_warning_while_command_remains_fresh():
    render = load_function("command_router_status_text")
    _text, level = render(
        {
            "received": True,
            "rejected_commands": 1,
            "last_rejection_age_ms": 100.0,
            "last_command_age_ms": 20.0,
            "last_mode": "hold",
        },
        True,
        500.0,
        "hold",
    )
    assert level == "warning"


def test_router_mode_mismatch_is_warning_even_when_command_is_fresh():
    render = load_function("command_router_status_text")
    _text, level = render(
        {
            "received": True,
            "rejected_commands": 0,
            "last_command_age_ms": 20.0,
            "last_mode": "brake",
        },
        True,
        500.0,
        "hold",
    )
    assert level == "warning"


def test_selected_j2_hold_or_position_forwarded_as_brake_is_critical():
    render = load_function("command_router_status_text")
    for requested_mode in ("hold", "position"):
        text, level = render(
            {
                "received": True,
                "rejected_commands": 0,
                "last_command_age_ms": 20.0,
                "last_mode": requested_mode,
                "last_active_joint_mask": [False, True, False, False, False, False],
                "j2_forwarded_mode": "brake",
            },
            True,
            500.0,
            requested_mode,
        )
        assert level == "critical"
        assert "J2转发矛盾" in text
        assert f"已请求J2 {requested_mode}主动控制" in text
        assert "路由向J2转发brake" in text
        assert "J2未收到该保持请求" in text


def test_unselected_j2_forwarded_as_brake_is_normal_domain_isolation():
    render = load_function("command_router_status_text")
    text, level = render(
        {
            "received": True,
            "rejected_commands": 0,
            "last_command_age_ms": 20.0,
            "last_mode": "position",
            "last_active_joint_mask": [True, False, False, False, False, False],
            "j2_forwarded_mode": "brake",
        },
        True,
        500.0,
        "position",
    )
    assert level == "normal"
    assert "J2转发矛盾" not in text


def test_fixed_hold_policy_buttons_are_distinct_from_the_only_ordinary_release():
    source = SOURCE.read_text(encoding="utf-8")
    panel_start = source.index("    def _control_panel(self) -> QGroupBox:")
    panel_end = source.index("    def _set_virtual_editable", panel_start)
    panel = source[panel_start:panel_end]
    assert 'self._fixed_hold_after_arrival_on' in panel
    assert 'self._fixed_hold_after_arrival_off' in panel
    assert panel.count("self._drag_mode") == 1
    assert "可拖动模式（普通脱力）" in panel
    assert "位置伺服始终有效" in panel
    assert "“关”不是物理脱力" in panel

    drag_start = source.index("    def _drag_mode(self) -> None:")
    drag_end = source.index("    def _emergency_brake", drag_start)
    drag = source[drag_start:drag_end]
    assert drag.index("dialog.clickedButton() is not confirm") < drag.index(
        "self._enter_drag_after_confirmation()"
    )


def test_only_drive_release_confirmations_are_modal_and_collision_alert_is_nonmodal():
    source = SOURCE.read_text(encoding="utf-8")
    # Two explicit drive-release confirmations plus one dedicated collision
    # warning.  The latter must never block the 100 Hz command stream.
    assert source.count("QMessageBox(self)") == 3
    assert "    def _message(" not in source
    popup_start = source.index("    def _show_collision_popup(")
    popup_end = source.index("    def _collision_popup_finished", popup_start)
    popup = source[popup_start:popup_end]
    assert '"不允许机械臂互撞"' in popup
    assert '"不允许模型地面碰撞"' in popup
    assert "dialog.setModal(False)" in popup
    assert "dialog.open()" in popup
    assert "dialog.exec()" not in popup
    assert "现有POSITION/HOLD及原锁定角度保持不变" in popup
    finished_start = popup_end
    finished_end = source.index("    def _apply_collision_rejection", finished_start)
    assert "self.last_collision_popup_signature = None" in source[
        finished_start:finished_end
    ]
    notify_start = source.index("    def _notify(")
    notify_end = source.index("    def _require_control_feedback", notify_start)
    notify = source[notify_start:notify_end]
    assert "QMessageBox" not in notify and ".exec()" not in notify
    assert "self.operator_notice_text = text" in notify
    assert "self._refresh_safety_notice()" in notify


def test_fixed_hold_policy_toggle_preserves_existing_hold_or_position_authority():
    method = load_main_window_method(
        "_set_fixed_hold_after_arrival",
        {"ArmMode": SimpleNamespace(SIM_TO_REAL="sim_to_real")},
    )

    class FakeMachine:
        def __init__(self):
            self.fixed_hold_after_arrival = True

        def set_fixed_hold_after_arrival(self, enabled):
            self.fixed_hold_after_arrival = enabled

    for mode in ("hold", "position"):
        class FakeWindow:
            def __init__(self):
                self.hardware_mode = mode
                self.machine = FakeMachine()
                self.direction = "preserved-direction"
                self.targets = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
                self.command_targets = [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6]
                self.requested_active_joint_mask = [True, False, True, True, False, True]
                self.pending_target_joint_mask = [False, True, False, False, True, False]
                self.moving_joint_mask = [True, False, True, False, False, True]
                self.activation_epoch = 12345
                self.editor_states = []

            def _require_control_feedback(self, *_args, **_kwargs):
                raise AssertionError("existing HOLD/POSITION policy toggle needs no recapture")

            def _prepare_hold_at_actual(self):
                raise AssertionError("existing target must not be recaptured")

            def _set_virtual_editable(self, enabled):
                self.editor_states.append(enabled)

            def _update_mode_label(self, _message):
                pass

            def _notify(self, _message, _level):
                pass

        window = FakeWindow()
        authority_before = (
            list(window.targets),
            list(window.command_targets),
            list(window.requested_active_joint_mask),
            list(window.pending_target_joint_mask),
            list(window.moving_joint_mask),
            window.activation_epoch,
        )
        method(window, False)
        authority_after = (
            window.targets,
            window.command_targets,
            window.requested_active_joint_mask,
            window.pending_target_joint_mask,
            window.moving_joint_mask,
            window.activation_epoch,
        )
        assert authority_after == authority_before
        assert window.hardware_mode == mode
        assert window.machine.fixed_hold_after_arrival is False
        assert window.direction == "sim_to_real"
        assert window.editor_states == [True]


@pytest.mark.parametrize("mode", ("hold", "position"))
def test_repeated_position_mode_never_resamples_external_force_offset(mode):
    arm_mode = SimpleNamespace(SIM_TO_REAL=object())
    enter_position = load_main_window_method(
        "_position_mode", {"ArmMode": arm_mode}
    )

    class FakeWindow:
        def __init__(self):
            self.hardware_mode = mode
            self.direction = object()
            self.command_targets = [0.25, -0.20, 0.10, 0.0, 0.0, 0.0]
            self.targets = list(self.command_targets)
            self.actual = [0.80, -0.70, 0.60, 0.0, 0.0, 0.0]
            self.requested_active_joint_mask = [True] * 6
            self.pending_target_joint_mask = [False] * 6
            self.moving_joint_mask = [False] * 6
            self.activation_epoch = 123456
            self.active_collision_proof = {"safe": True}
            self.command_stream_suspended = True
            self.resume_count = 0
            self.notices = []

        def _require_control_feedback(self, _message, **_kwargs):
            return True

        def _prepare_hold_at_actual(self):
            raise AssertionError("established HOLD/POSITION must not recapture feedback")

        def _set_virtual_editable(self, _enabled):
            pass

        def _resume_command_stream(self):
            self.command_stream_suspended = False
            self.resume_count += 1

        def _update_mode_label(self, _message):
            pass

        def _notify(self, message, level="warning"):
            self.notices.append((message, level))

    window = FakeWindow()
    authoritative_before = (
        list(window.command_targets),
        list(window.targets),
        list(window.requested_active_joint_mask),
        list(window.moving_joint_mask),
        window.activation_epoch,
    )
    enter_position(window)
    assert (
        window.command_targets,
        window.targets,
        window.requested_active_joint_mask,
        window.moving_joint_mask,
        window.activation_epoch,
    ) == authoritative_before
    assert window.actual != window.command_targets
    assert window.direction is arm_mode.SIM_TO_REAL
    if mode == "hold":
        assert window.command_stream_suspended is False
        assert window.resume_count == 1
    else:
        assert window.command_stream_suspended is True
        assert window.resume_count == 0


def test_partial_hold_retakeover_preserves_every_existing_active_target():
    method = load_main_window_method(
        "_prepare_retakeover_hold_preserving_active_targets"
    )

    class FakeMachine:
        def __init__(self, events):
            self.events = events

        def hold(self):
            self.events.append("hold")

    class FakeArrival:
        def __init__(self, events):
            self.events = events

        def start(self, _now):
            self.events.append("arrival")

    class FakeWindow:
        def __init__(self):
            self.command_targets = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
            self.actual = [1.1, 1.2, 1.3, 1.4, 1.5, 1.6]
            self.requested_active_joint_mask = [True, False, True, False, True, False]
            self.connected = [True] * 6
            self.pending_target_joint_mask = [True] * 6
            self.moving_joint_mask = [True] * 6
            self.active_collision_proof = {"safe": True}
            self.activation_epoch = 10
            self.events = []
            self.machine = FakeMachine(self.events)
            self.arrival = FakeArrival(self.events)
            self.command_stream_suspended = True

        def _cancel_queued_pose(self, *, restore_command_target):
            self.events.append(("cancel", restore_command_target))

        def _set_virtual_editable(self, enabled):
            self.events.append(("editor", enabled))

        def _show_targets_on_virtual(self):
            self.events.append("preview")

        def _authorize_active_joints(self, requested):
            self.requested_active_joint_mask = list(requested)
            self.activation_epoch += 1
            self.events.append(("authorize", tuple(requested)))

        def _resume_command_stream(self):
            self.command_stream_suspended = False
            self.events.append("resume")

    window = FakeWindow()
    method(window)
    assert window.command_targets == [0.1, 1.2, 0.3, 1.4, 0.5, 1.6]
    assert window.targets == window.command_targets
    assert window.requested_active_joint_mask == [True] * 6
    assert window.pending_target_joint_mask == [False] * 6
    assert window.moving_joint_mask == [False] * 6
    assert window.activation_epoch == 11
    assert window.hardware_mode == "hold"
    assert window.command_stream_suspended is False
    assert window.events.index("hold") < window.events.index("resume")


def test_fixed_hold_policy_from_brake_or_drag_requires_feedback_then_enters_hold():
    method = load_main_window_method(
        "_set_fixed_hold_after_arrival",
        {"ArmMode": SimpleNamespace(SIM_TO_REAL="sim_to_real")},
    )

    class FakeMachine:
        fixed_hold_after_arrival = True

        def __init__(self, events):
            self.events = events

        def set_fixed_hold_after_arrival(self, enabled):
            self.events.append(("policy", enabled))
            self.fixed_hold_after_arrival = enabled

    class FakeWindow:
        def __init__(self, mode, feedback_ready):
            self.hardware_mode = mode
            self.feedback_ready = feedback_ready
            self.events = []
            self.machine = FakeMachine(self.events)
            self.direction = "real_to_sim"

        def _require_control_feedback(self, *_args, **_kwargs):
            self.events.append(("feedback", self.feedback_ready))
            return self.feedback_ready

        def _prepare_hold_at_actual(self):
            self.events.append(("capture-current-hold", True))
            self.hardware_mode = "hold"

        def _set_virtual_editable(self, enabled):
            self.events.append(("editor", enabled))

        def _update_mode_label(self, _message):
            self.events.append(("label", True))

        def _notify(self, _message, _level):
            self.events.append(("notice", True))

    for mode in ("brake", "drag"):
        blocked = FakeWindow(mode, False)
        method(blocked, False)
        assert blocked.events == [("feedback", False)]
        assert blocked.hardware_mode == mode

        allowed = FakeWindow(mode, True)
        method(allowed, False)
        assert allowed.events[:3] == [
            ("feedback", True),
            ("policy", False),
            ("capture-current-hold", True),
        ]
        assert allowed.hardware_mode == "hold"
        assert allowed.direction == "sim_to_real"
        assert ("editor", True) in allowed.events


def test_emergency_brake_confirms_once_and_bypasses_feedback_gate():
    cancel_queue = load_main_window_method("_cancel_queued_pose")
    consume = load_main_window_method(
        "_consume_collision_guard_result",
        {"collision_guard_result_matches": load_function(
            "collision_guard_result_matches"
        )},
    )
    request, late_safe = collision_request_and_result()

    class FakeTimer:
        def stop(self):
            pass

    class FakeMessageBox:
        class ButtonRole:
            AcceptRole = "accept"
            RejectRole = "reject"

        def __init__(self, _parent):
            self.confirm = None

        def setWindowTitle(self, _title):
            pass

        def setText(self, _text):
            pass

        def addButton(self, _text, role):
            token = object()
            if role == self.ButtonRole.AcceptRole:
                self.confirm = token
            return token

        def exec(self):
            pass

        def clickedButton(self):
            return self.confirm

    method = load_main_window_method(
        "_emergency_brake",
        {
            "ArmMode": SimpleNamespace(REAL_TO_SIM="real_to_sim"),
            "QMessageBox": FakeMessageBox,
        },
    )

    class FakeMachine:
        def __init__(self):
            self.stop_count = 0

        def stop(self):
            self.stop_count += 1

    class FakeWindow:
        _cancel_queued_pose = cancel_queue

        def __init__(self, mode):
            self.hardware_mode = mode
            self.machine = FakeMachine()
            self.direction = "sim_to_real"
            self.requested_active_joint_mask = [True] * 6
            self.pending_target_joint_mask = [True] * 6
            self.moving_joint_mask = [True] * 6
            self.command_targets = [0.1] * 6
            self.targets = [0.2] * 6
            self.pending_collision_execute_sequence = request["request_sequence"]
            self.queued_pose_target = [0.2] * 6
            self.queued_joint_indices = [1, 2, 3]
            self.active_sequence_joint = 0
            self.collision_preview_timer = FakeTimer()
            self.active_collision_proof = {"safe": True}
            self.node = SimpleNamespace(latest_collision_result=late_safe)
            self.collision_requests = {request["request_sequence"]: request}
            self.last_consumed_collision_sequence = 0
            self.latest_collision_preview_sequence = None
            self.session_id = request["session_id"]
            self.state_instance_id = request["state_instance_id"]
            self.actual = list(request["start_relative_rad"])
            self.published = 0
            self.feedback_checked = False

        def _require_control_feedback(self, *_args, **_kwargs):
            self.feedback_checked = True
            raise AssertionError("emergency BRAKE must not depend on feedback")

        def _resume_command_stream(self):
            pass

        def _show_targets_on_virtual(self):
            pass

        def _refresh_virtual_editability(self):
            pass

        def _set_virtual_editable(self, enabled):
            assert enabled is False

        def _update_mode_label(self, _message):
            pass

        def _notify(self, _message, level):
            assert level == "critical"

        def _publish_command(self):
            self.published += 1

        def _commit_checked_target(self, *_args):
            raise AssertionError("late safe proof must not undo emergency BRAKE")

    for mode in ("brake", "drag", "hold", "position"):
        window = FakeWindow(mode)
        method(window)
        assert not window.feedback_checked
        assert window.machine.stop_count == 1
        assert window.hardware_mode == "brake"
        assert window.direction == "real_to_sim"
        assert window.requested_active_joint_mask == [False] * 6
        assert window.pending_target_joint_mask == [False] * 6
        assert window.moving_joint_mask == [False] * 6
        assert window.pending_collision_execute_sequence is None
        assert window.queued_pose_target is None
        assert window.queued_joint_indices == []
        assert window.active_sequence_joint is None
        assert window.active_collision_proof is None
        assert window.published == 1
        consume(window)
        assert window.hardware_mode == "brake"
        assert window.queued_pose_target is None


@pytest.mark.parametrize("action", ("stop", "drag", "stale"))
def test_operator_or_stale_cancellation_clears_queue_and_blocks_late_safe(action):
    cancel = load_main_window_method("_cancel_queued_pose")
    suspend = load_main_window_method("_suspend_command_stream")
    stop = load_main_window_method(
        "_stop",
        {
            "ArmMode": SimpleNamespace(REAL_TO_SIM="real_to_sim"),
            "control_feedback_ready": load_function("control_feedback_ready"),
        },
    )
    drag = load_main_window_method(
        "_enter_drag_after_confirmation",
        {"ArmMode": SimpleNamespace(REAL_TO_SIM="real_to_sim")},
    )
    stale = load_main_window_method("_suspend_for_stale_feedback")
    consume = load_main_window_method(
        "_consume_collision_guard_result",
        {"collision_guard_result_matches": load_function(
            "collision_guard_result_matches"
        )},
    )
    request, late_safe = collision_request_and_result()

    class FakeTimer:
        def stop(self):
            pass

    class FakeMachine:
        def drag(self):
            pass

    class FakeNode:
        def __init__(self):
            self.latest_collision_result = late_safe

        def control_streams_fresh(self, _now=None):
            return action == "drag"

    class FakeWindow:
        _cancel_queued_pose = cancel
        _suspend_command_stream = suspend

        def __init__(self):
            self.node = FakeNode()
            self.hardware_mode = "position"
            self.direction = "sim_to_real"
            self.command_targets = list(request["start_relative_rad"])
            self.targets = list(request["target_relative_rad"])
            self.actual = list(request["start_relative_rad"])
            self.requested_active_joint_mask = [True] * 6
            self.pending_target_joint_mask = [True] + [False] * 5
            self.moving_joint_mask = [True] + [False] * 5
            self.pending_collision_execute_sequence = request["request_sequence"]
            self.queued_pose_target = list(request["target_relative_rad"])
            self.queued_joint_indices = [1, 2]
            self.active_sequence_joint = 0
            self.collision_preview_timer = FakeTimer()
            self.active_collision_proof = {"safe": True}
            self.command_stream_suspended = False
            self.command_stream_suspended_reason = ""
            self.active_observation_uncertain = False
            self.have_first_state = True
            self.connected = [True] * 6
            self.machine = FakeMachine()
            self.collision_requests = {request["request_sequence"]: request}
            self.last_consumed_collision_sequence = 0
            self.latest_collision_preview_sequence = None
            self.session_id = request["session_id"]
            self.state_instance_id = request["state_instance_id"]

        def _update_connected(self, _now=None):
            self.active_observation_uncertain = action == "stop"

        def _show_targets_on_virtual(self):
            pass

        def _refresh_virtual_editability(self):
            pass

        def _update_mode_label(self, _message):
            pass

        def _notify(self, *_args):
            pass

        def _authorize_active_joints(self, _mask):
            pass

        def _resume_command_stream(self):
            self.command_stream_suspended = False

        def _set_virtual_editable(self, _enabled):
            pass

        def _commit_checked_target(self, *_args):
            raise AssertionError("cancelled queue must reject a late safe proof")

    window = FakeWindow()
    if action == "stop":
        stop(window)
    elif action == "drag":
        assert drag(window)
    else:
        stale(window)

    assert window.pending_collision_execute_sequence is None
    assert window.queued_pose_target is None
    assert window.queued_joint_indices == []
    assert window.active_sequence_joint is None
    assert window.targets == window.command_targets
    consume(window)
    assert window.queued_pose_target is None


def test_hardware_session_change_clears_queue_and_blocks_old_session_proof():
    cancel = load_main_window_method("_cancel_queued_pose")
    suspend = load_main_window_method("_suspend_command_stream")
    update = load_main_window_method(
        "_update_connected",
        {
            "hardware_state_contract_valid": load_function(
                "hardware_state_contract_valid"
            ),
            "classify_logical_joint_observations": load_function(
                "classify_logical_joint_observations"
            ),
        },
    )
    consume = load_main_window_method(
        "_consume_collision_guard_result",
        {"collision_guard_result_matches": load_function(
            "collision_guard_result_matches"
        )},
    )
    request, late_safe = collision_request_and_result()
    changed_hardware = hardware_state()
    changed_hardware["session_id"] = "persistent:new-session"
    changed_hardware["state_instance_id"] = request["state_instance_id"]

    class FakeTimer:
        def stop(self):
            pass

    class FakeNode:
        latest_hardware = changed_hardware
        latest_collision_result = late_safe

        def control_streams_fresh(self, _now=None):
            return True

    class FakeWindow:
        _cancel_queued_pose = cancel
        _suspend_command_stream = suspend

        def __init__(self):
            self.node = FakeNode()
            self.session_id = request["session_id"]
            self.state_instance_id = request["state_instance_id"]
            self.actual = [0.15] * 6
            self.targets = list(request["target_relative_rad"])
            self.command_targets = list(request["start_relative_rad"])
            self.requested_active_joint_mask = [True] * 6
            self.pending_target_joint_mask = [True] + [False] * 5
            self.moving_joint_mask = [True] + [False] * 5
            self.active_collision_proof = {"safe": True}
            self.pending_collision_execute_sequence = request["request_sequence"]
            self.queued_pose_target = list(request["target_relative_rad"])
            self.queued_joint_indices = [1]
            self.active_sequence_joint = 0
            self.collision_preview_timer = FakeTimer()
            self.command_stream_suspended = False
            self.command_stream_suspended_reason = ""
            self.hardware_mode = "hold"
            self.connected = [True] * 6
            self.faulted = [False] * 6
            self.observation_uncertain = [False] * 6
            self.active_observation_uncertain = False
            self.collision_requests = {request["request_sequence"]: request}
            self.last_consumed_collision_sequence = 0
            self.latest_collision_preview_sequence = None

        def _apply_connection_state(self, connected, faulted):
            self.connected = list(connected)
            self.faulted = list(faulted)

        def _show_targets_on_virtual(self):
            pass

        def _refresh_virtual_editability(self):
            pass

        def _update_mode_label(self, _message):
            pass

        def _notify(self, *_args):
            pass

        def _commit_checked_target(self, *_args):
            raise AssertionError("an old-session proof must never commit")

    window = FakeWindow()
    update(window, time.monotonic())
    assert window.session_id == "persistent:new-session"
    assert window.pending_collision_execute_sequence is None
    assert window.queued_pose_target is None
    assert window.queued_joint_indices == []
    assert window.active_sequence_joint is None
    assert window.active_collision_proof is None
    assert window.command_stream_suspended is True
    consume(window)
    assert window.queued_pose_target is None


def test_gui_subscribes_to_and_displays_router_acknowledgement_state():
    source = SOURCE.read_text(encoding="utf-8")
    assert 'String, "/whole_arm/control_status", self._control_status_callback, 10' in source
    assert "self.latest_control_status = value" in source
    assert "self.last_control_status_receipt = time.monotonic()" in source
    assert "命令路由：{router_text}" in source
    assert "控制请求：{requested}" in source
    assert "控制器确认：{confirmed}" in source
    assert "租约安全保持：{lease_hold_text}" in source
    assert "驱动状态：已制动" not in source


def test_stop_holds_fresh_feedback_and_does_not_brake_an_unconfirmed_active_hold():
    source = SOURCE.read_text(encoding="utf-8")
    assert 'self._button("停止轨迹并保持（承重HOLD）", self._stop)' in source
    stop_start = source.index("    def _stop(self) -> None:")
    stop_end = source.index("    def _save_initial_pose", stop_start)
    stop = source[stop_start:stop_end]
    assert "control_feedback_ready(" in stop
    assert stop.index("self.active_observation_uncertain") < stop.index(
        'if self.hardware_mode == "position":'
    ) < stop.rindex("self._suspend_command_stream(") < stop.index(
        "self.machine.stop()"
    )
    assert 'self.hardware_mode = "brake"' in stop
    assert "GUI已停止发送新命令" in stop
    assert stop.count("self._publish_command()") == 2
    hold_branch = stop[
        stop.index('elif self.hardware_mode == "hold":'):
        stop.index("            else:", stop.index('elif self.hardware_mode == "hold":'))
    ]
    assert "self.command_targets = list(self.actual)" not in hold_branch
    assert "self.targets = list(self.command_targets)" in hold_branch


def test_close_holds_when_fresh_and_sends_nothing_for_unconfirmed_active_hold():
    source = SOURCE.read_text(encoding="utf-8")
    close_start = source.index("    def closeEvent(self, event: QCloseEvent) -> None:")
    close_end = source.index("\n\n\ndef main", close_start)
    close = source[close_start:close_end]
    assert 'self.hardware_mode in {"hold", "position"}' in close
    assert "control_feedback_ready(" in close
    assert close.index("self._transition_position_to_fixed_hold()") < close.index(
        'self.hardware_mode = "brake"'
    )
    assert "if self.command_stream_suspended:" in close
    assert 'elif self.hardware_mode in {"hold", "position"}:' in close
    assert "publish_before_close = False" in close
    assert "for _ in range(5):" in close
    assert 'if self.hardware_mode == "position":' in close
    assert "Closing the window is not permission to recapture a HOLD" in close

def test_position_to_hold_transition_keeps_epoch_and_fixed_targets():
    source = SOURCE.read_text(encoding="utf-8")
    start = source.index("    def _transition_position_to_fixed_hold(")
    end = source.index("    def _execute_target(", start)
    transition = source[start:end]
    assert "fixed_hold_targets_after_position_stop(" in transition
    assert "self.moving_joint_mask = [False] * 6" in transition
    assert "self._authorize_active_joints(" not in transition
    assert "self.activation_epoch" not in transition


def test_both_post_arrival_policies_execute_the_same_position_command():
    source = SOURCE.read_text(encoding="utf-8")
    execute_start = source.index("    def _execute_target(self) -> None:")
    execute_end = source.index("    def _stop(self) -> None:", execute_start)
    execute = source[execute_start:execute_end]
    assert "fixed_hold_after_arrival" not in execute
    assert 'self.hardware_mode = "position"' in execute
    assert "self.command_targets = list(self.targets)" in execute
    assert "self.moving_joint_mask = candidate_moving_joint_mask" in execute

    refresh_start = source.index("    def _refresh_joint_widgets(self) -> None:")
    refresh_end = source.index("        elif self.hardware_mode == \"brake\":", refresh_start)
    arrival = source[refresh_start:refresh_end]
    assert "sequence_joint is not None" in arrival
    assert "or self.machine.fixed_hold_after_arrival" in arrival
    assert "A physical sequence always inserts an exact-target" in arrival
    assert "继续发送POSITION目标" in arrival


def test_hold_target_is_captured_once_and_never_follows_external_displacement():
    source = SOURCE.read_text(encoding="utf-8")
    policy_start = source.index("    def _set_fixed_hold_after_arrival(self, enabled: bool) -> None:")
    policy_end = source.index("    def _enter_drag_after_confirmation", policy_start)
    policy = source[policy_start:policy_end]
    assert 'entering_position_servo = self.hardware_mode in {"brake", "drag"}' in policy
    assert "if entering_position_servo:" in policy
    assert "self._prepare_hold_at_actual()" in policy

    refresh_start = source.index("    def _refresh_joint_widgets(self) -> None:")
    refresh_end = source.index("    def _publish_command", refresh_start)
    refresh = source[refresh_start:refresh_end]
    assert 'self.hardware_mode in {"brake", "drag"}' in refresh
    assert 'self.hardware_mode in {"hold", "position"}' not in refresh[
        refresh.rindex("if ("):
    ]
    assert "or self.machine.fixed_hold_after_arrival" in refresh
    assert "A completed trajectory becomes fixed-target HOLD" in refresh
    assert 'self.hardware_mode = "hold"' in refresh
    completed_hold = refresh[
        refresh.index("A completed trajectory becomes fixed-target HOLD"):
        refresh.index('        elif self.hardware_mode == "brake":')
    ]
    assert "self.command_targets = list(self.actual)" not in completed_hold


def test_new_or_restarted_gui_cannot_publish_default_brake_before_operator_action():
    source = SOURCE.read_text(encoding="utf-8")
    assert "self.command_stream_suspended = True" in source
    publish_start = source.index("    def _publish_command(self) -> bool:")
    publish_end = source.index("    def _refresh_safety_notice", publish_start)
    publish = source[publish_start:publish_end]
    assert publish.index("if self.command_stream_suspended:") < publish.index(
        "self.command_sequence += 1"
    )


def test_direction_and_position_mode_buttons_preserve_load_bearing_hold():
    source = SOURCE.read_text(encoding="utf-8")
    real_start = source.index("    def _real_to_sim(self) -> None:")
    position_start = source.index("    def _position_mode(self) -> None:", real_start)
    direction_block = source[real_start:position_start]
    assert 'self.hardware_mode = "brake"' not in direction_block
    assert "self.requested_active_joint_mask = [False] * 6" not in direction_block

    position_end = source.index(
        "    def _fixed_hold_after_arrival_on(self) -> None:", position_start
    )
    position_block = source[position_start:position_end]
    assert "self._prepare_hold_at_actual()" in position_block
    assert 'self.hardware_mode = "position"' not in position_block


def test_single_joint_target_keeps_every_healthy_load_bearing_joint_active():
    source = SOURCE.read_text(encoding="utf-8")
    execute_start = source.index("    def _execute_target(self) -> None:")
    execute_end = source.index("    def _stop(self) -> None:", execute_start)
    execute = source[execute_start:execute_end]
    assert "self.queued_pose_target = list(self.targets)" in execute
    assert "self.queued_joint_indices = list(pending_indices)" in execute
    assert "self._begin_next_queued_segment()" in execute
    assert "candidate_moving_joint_mask = [" in execute
    assert "self.moving_joint_mask = candidate_moving_joint_mask" in execute
    assert "if sum(candidate_moving_joint_mask) != 1:" in execute
    assert "self._authorize_active_joints([True] * 6)" in execute
    assert execute.index("moving_targets_within_model_limits(") < execute.index(
        "self.queued_pose_target = list(self.targets)"
    ) < execute.index(
        "self.machine.position()"
    )
    commit_start = execute.index("    def _commit_checked_target(")
    commit = execute[commit_start:]
    invalid_start = commit.index("if not moving_targets_within_model_limits(")
    invalid_end = commit.index("        self.machine.position()", invalid_start)
    invalid_block = commit[invalid_start:invalid_end]
    assert "self.moving_joint_mask =" not in invalid_block
    assert "现有位置轨迹与移动掩码保持不变" in invalid_block
    assert "list(self.pending_target_joint_mask)" not in commit


def test_stale_feedback_and_session_change_suspend_instead_of_publish_brake():
    source = SOURCE.read_text(encoding="utf-8")
    stale_start = source.index("    def _suspend_for_stale_feedback(self) -> None:")
    stale_end = source.index("    def _show_targets_on_virtual", stale_start)
    stale = source[stale_start:stale_end]
    assert "self._suspend_command_stream(" in stale
    assert 'self.hardware_mode = "brake"' not in stale
    assert "self._publish_command()" not in stale

    session_start = source.index(
        "        if reference_changed or state_publisher_changed:"
    )
    session_end = source.index("        if incoming_session:", session_start)
    session = source[session_start:session_end]
    assert "self._suspend_command_stream(" in session
    assert 'self.hardware_mode = "brake"' not in session
    assert "硬件状态进程已重启" in session


def test_nonfinite_joint_feedback_and_j2_sync_fault_are_rejected_from_authority():
    source = SOURCE.read_text(encoding="utf-8")
    assert "all(math.isfinite(float(value)) for value in message.position)" in source
    classifier_start = source.index("def classify_logical_joint_observations(")
    classifier_end = source.index("\ndef collision_motion_state_ready(", classifier_start)
    classifier = source[classifier_start:classifier_end]
    assert 'and hardware["j2_sync_fault"]' in classifier
    assert 'j2_sync_error > RAD' not in classifier
    assert '"lease_safe_hold_by_motor"' in source


def test_stale_state_stream_never_reuses_old_hold_or_sync_confirmation():
    source = SOURCE.read_text(encoding="utf-8")
    notice_start = source.index("    def _refresh_safety_notice(")
    notice_end = source.index("    def _refresh_summary(", notice_start)
    notice = source[notice_start:notice_end]
    assert "streams_fresh = self.node.control_streams_fresh(checked_at)" in notice
    assert "if streams_fresh else []" in notice
    assert 'if j2_observed and hardware.get("j2_sync_fault", False):' in notice
    assert "elif j2_observed and sync_error > math.radians(0.5):" in notice

    widgets_start = source.index("    def _refresh_joint_widgets(")
    widgets_end = source.index("    def _publish_command(", widgets_start)
    widgets = source[widgets_start:widgets_end]
    assert "self.node.control_streams_fresh(now)" in widgets
    assert "and not self.observation_uncertain[1]" in widgets

    summary_start = notice_end
    summary_end = source.index("    def _open_log(", summary_start)
    summary = source[summary_start:summary_end]
    assert '"等待确认" if not ros_ok else' in summary
