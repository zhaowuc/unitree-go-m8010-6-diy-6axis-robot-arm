"""Read-only whole-arm gravity evaluator for the frozen MuJoCo model.

This node intentionally has no motor-command publisher.  It converts fresh
encoder state into model coordinates only when a session-bound
``MODEL_SESSION_ANCHOR_V2`` is valid, then publishes diagnostics and the six
logical static gravity torques.  Missing/stale authority is reported and no
torque sample is published.
"""

from __future__ import annotations

import concurrent.futures
import json
import hashlib
import math
import secrets
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .gravity_model import (
    PRODUCTION_MODEL_SHA256,
    GRAVITY_CONFIG_SHA256,
    GravityAnchorError,
    GravityFeedforwardController,
    GravityModelAnchorV2,
    StaticGravityEvaluator,
    normalize_named_joint_positions,
    sha256_file,
)
from .empirical_validation_envelope import (
    AUTHORITY_CLASS as EMPIRICAL_AUTHORITY_CLASS,
    EmpiricalStageGate,
    EmpiricalValidationEnvelope,
    RATING_CLASSIFICATION as EMPIRICAL_RATING_CLASSIFICATION,
    SOFTWARE_GRAVITY_ROTOR_LIMIT_NM,
    select_runtime_torque_authority,
)
from .planned_path_feasibility import (
    THERMAL_CONFIG_SHA256,
    PlannedPathError,
    PlannedPathRequest,
    PathLoadEnvelope,
    build_planned_path_proof,
    evaluate_path_load_envelope,
    hardware_pose_feedback_blocker,
    parse_continuous_rotor_limits,
    parse_planned_path_request,
    temperature_observation,
)


JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
PLANNED_REQUEST_MAXIMUM_BYTES = 8 * 1024 * 1024
PLANNED_REQUEST_MAXIMUM_CHARACTERS = PLANNED_REQUEST_MAXIMUM_BYTES
MAXIMUM_PLANNED_REQUEST_SOURCES = 64


class WholeArmGravityNode(Node):
    def __init__(self) -> None:
        super().__init__("whole_arm_gravity_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("gravity_config_path", "")
        self.declare_parameter("thermal_config_path", "")
        self.declare_parameter("anchor_path", "")
        self.declare_parameter("empirical_envelope_path", "")
        self.declare_parameter("expected_empirical_envelope_sha256", "")
        self.declare_parameter("empirical_claim_directory", "")
        self.declare_parameter("calculation_rate_hz", 100.0)
        self.declare_parameter("joint_state_maximum_age_ms", 250.0)
        self.declare_parameter("enabled_for_hardware", False)
        self.declare_parameter("gravity_scale_target", 0.0)
        self.declare_parameter("gravity_scale_ramp_seconds", 2.0)
        self.declare_parameter("maximum_rotor_torque_slew_nm_per_s", 1.0)

        self.model_path = Path(str(self.get_parameter("model_path").value)).resolve()
        self.gravity_config_path = Path(
            str(self.get_parameter("gravity_config_path").value)
        ).resolve()
        self.thermal_config_path = Path(
            str(self.get_parameter("thermal_config_path").value)
        ).resolve()
        anchor_text = str(self.get_parameter("anchor_path").value).strip()
        self.anchor_path = Path(anchor_text).resolve() if anchor_text else None
        empirical_envelope_text = str(
            self.get_parameter("empirical_envelope_path").value
        ).strip()
        self.empirical_envelope_path = (
            Path(empirical_envelope_text).resolve()
            if empirical_envelope_text else None
        )
        self.expected_empirical_envelope_sha256 = str(
            self.get_parameter("expected_empirical_envelope_sha256").value
        ).strip()
        claim_directory_text = str(
            self.get_parameter("empirical_claim_directory").value
        ).strip()
        self.empirical_claim_directory = (
            Path(claim_directory_text).resolve()
            if claim_directory_text
            else Path.home() / ".local" / "state" /
            "go_m8010_arm_gui" / "empirical_claims"
        )
        self.rate_hz = float(self.get_parameter("calculation_rate_hz").value)
        self.maximum_age_s = (
            float(self.get_parameter("joint_state_maximum_age_ms").value) / 1000.0
        )
        self.hardware_enable_requested = False
        ramp_seconds = float(
            self.get_parameter("gravity_scale_ramp_seconds").value
        )
        self.gravity_scale_ramp_seconds = ramp_seconds
        maximum_slew = float(
            self.get_parameter("maximum_rotor_torque_slew_nm_per_s").value
        )
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("calculation_rate_hz must be finite and positive")
        if not math.isfinite(self.maximum_age_s) or self.maximum_age_s <= 0.0:
            raise ValueError("joint_state_maximum_age_ms must be positive")
        self.feedforward_controller = GravityFeedforwardController(
            ramp_seconds=ramp_seconds,
            maximum_slew_nm_per_s=maximum_slew,
        )

        self.gravity_publisher = self.create_publisher(
            Float64MultiArray, "/whole_arm/gravity_joint_torque", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/whole_arm/gravity_status", 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            String, "/whole_arm/hardware_state", self._on_hardware_state, 20
        )
        self.create_subscription(
            String,
            "/whole_arm/planned_path_feasibility_request",
            self._on_planned_path_request,
            10,
        )
        self.create_subscription(
            String,
            "/whole_arm/empirical_stage_confirmation",
            self._on_empirical_stage_confirmation,
            10,
        )

        self.q_actual: Optional[tuple[float, ...]] = None
        self.hardware_q_actual: Optional[tuple[float, ...]] = None
        self.hardware_state_sequence: Optional[int] = None
        self.hardware_source_monotonic_ns: Optional[int] = None
        self.joint_received_s: Optional[float] = None
        self.session_id: Optional[str] = None
        self.state_instance_id: Optional[str] = None
        self.hardware_received_s: Optional[float] = None
        self.latest_hardware_state: Optional[dict] = None
        self.last_calculation_s: Optional[float] = None
        self.last_gravity_joint_nm: Optional[tuple[float, ...]] = None
        self.last_rotor_nm: Optional[dict[str, float]] = None
        self.last_feedforward_nm: tuple[float, ...] = (0.0,) * 6
        self.current_gravity_scale = 0.0
        self.gravity_source_instance_id = secrets.token_hex(16)
        self.gravity_status_sequence = 0
        self.anchor: Optional[GravityModelAnchorV2] = None
        self.anchor_sha256 = "UNAVAILABLE"
        self.evaluator: Optional[StaticGravityEvaluator] = None
        self.initialization_blocker = ""
        self.model_sha256 = "UNREADABLE"
        self.continuous_rotor_limits_authoritative = False
        self.continuous_rotor_limits_nm: Optional[dict[str, float]] = None
        self.short_peak_rotor_limits_nm: Optional[dict[str, float]] = None
        self.temperature_limit_config_authoritative = False
        self.thermal_derating_start_c: Optional[float] = None
        self.thermal_config_sha256 = "UNREADABLE"
        self.j6_joint_to_rotor_scale: Optional[float] = None
        self.empirical_stage_gate: Optional[EmpiricalStageGate] = None
        self.empirical_initialization_blocker = (
            "EMPIRICAL_ENVELOPE_NOT_CONFIGURED"
        )
        self._planned_lock = threading.Lock()
        self._planned_generation = 0
        self._planned_request: Optional[PlannedPathRequest] = None
        self._planned_envelope: Optional[PathLoadEnvelope] = None
        self._planned_evaluation_blocker = ""
        self._planned_request_last_by_source: dict[str, tuple[int, int]] = {}
        self._planned_cancel_event = threading.Event()
        self._planned_future: Optional[concurrent.futures.Future] = None
        self._planned_shutdown = False
        self._path_evaluator: Optional[StaticGravityEvaluator] = None
        self._planned_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="whole-path-gravity-proof"
        )
        self._initialize_authority()
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def _initialize_authority(self) -> None:
        try:
            self.model_sha256 = sha256_file(self.model_path)
            if self.model_sha256 != PRODUCTION_MODEL_SHA256:
                raise ValueError("冻结生产MuJoCo模型哈希不匹配")
            if (
                not self.gravity_config_path.is_file()
                or sha256_file(self.gravity_config_path) != GRAVITY_CONFIG_SHA256
            ):
                raise ValueError("冻结重力配置哈希不匹配")
            gravity_config = yaml.safe_load(
                self.gravity_config_path.read_text(encoding="utf-8")
            )
            if (
                not isinstance(gravity_config, dict)
                or gravity_config.get("schema")
                != "go-m8010-gravity-control/1.0"
                or gravity_config.get("production_model_sha256")
                != PRODUCTION_MODEL_SHA256
                or gravity_config.get("gravity_scale_levels")
                != [0.25, 0.50, 0.75, 1.00]
                or type(gravity_config.get(
                    "continuous_rotor_limits_authoritative"
                )) is not bool
            ):
                raise ValueError("冻结重力配置语义无效")
            self.continuous_rotor_limits_authoritative = bool(
                gravity_config["continuous_rotor_limits_authoritative"]
            )
            j6_scale = gravity_config.get("j6_joint_to_rotor_torque_scale")
            if j6_scale is not None:
                self.j6_joint_to_rotor_scale = float(j6_scale)
                if (
                    not math.isfinite(self.j6_joint_to_rotor_scale)
                    or self.j6_joint_to_rotor_scale <= 0.0
                ):
                    raise ValueError("J6 joint-to-rotor mapping is invalid")
            self.thermal_config_sha256 = sha256_file(self.thermal_config_path)
            if self.thermal_config_sha256 != THERMAL_CONFIG_SHA256:
                raise ValueError("冻结热管理配置哈希不匹配")
            thermal_config = yaml.safe_load(
                self.thermal_config_path.read_text(encoding="utf-8")
            )
            if (
                not isinstance(thermal_config, dict)
                or thermal_config.get("schema")
                != "go-m8010-thermal-limits/1.0"
            ):
                raise ValueError("冻结热管理配置语义无效")
            self.thermal_derating_start_c = float(
                thermal_config["derating_start_c"]
            )
            thermal_stop_c = float(thermal_config["thermal_stop_c"])
            if (
                not math.isfinite(self.thermal_derating_start_c)
                or not math.isfinite(thermal_stop_c)
                or not 0.0 < self.thermal_derating_start_c < thermal_stop_c
            ):
                raise ValueError("冻结热阈值无效")
            threshold_authority = thermal_config.get("threshold_authority")
            self.temperature_limit_config_authoritative = bool(
                isinstance(threshold_authority, str)
                and threshold_authority.strip()
                and "UNVERIFIED" not in threshold_authority.upper()
            )
            self.continuous_rotor_limits_nm = parse_continuous_rotor_limits(
                thermal_config.get("continuous_rotor_torque_limit_nm")
            )
            self.short_peak_rotor_limits_nm = parse_continuous_rotor_limits(
                thermal_config.get("short_peak_rotor_torque_limit_nm")
            )
            if self.anchor_path is None or not self.anchor_path.is_file():
                raise GravityAnchorError("MODEL_SESSION_ANCHOR_V2不存在")
            self.anchor = GravityModelAnchorV2.from_path(self.anchor_path)
            self.anchor_sha256 = sha256_file(self.anchor_path)
            self.evaluator = StaticGravityEvaluator(self.model_path)
        except Exception as exc:  # Keep diagnostics online while authority is blocked.
            self.anchor = None
            self.evaluator = None
            self.initialization_blocker = str(exc)
            self.get_logger().error(
                "重力解算保持失效关闭（不发布电机命令）：" + self.initialization_blocker
            )
        if self.anchor is not None and self.evaluator is not None:
            self._initialize_empirical_envelope()

    def _initialize_empirical_envelope(self) -> None:
        if (
            self.empirical_envelope_path is None
            and not self.expected_empirical_envelope_sha256
        ):
            return
        if (
            self.empirical_envelope_path is None
            or not self.expected_empirical_envelope_sha256
        ):
            self.empirical_initialization_blocker = (
                "EMPIRICAL_ENVELOPE_PATH_AND_SHA256_REQUIRED_TOGETHER"
            )
            return
        try:
            envelope = EmpiricalValidationEnvelope.from_path(
                self.empirical_envelope_path,
                self.expected_empirical_envelope_sha256,
            )
            if abs(
                self.gravity_scale_ramp_seconds - envelope.ramp_seconds
            ) > 1.0e-12:
                raise ValueError(
                    "EMPIRICAL_RUNTIME_RAMP_SECONDS_MISMATCH"
                )
            # Normal/official authority retains the V15.31A full-scale slew
            # semantics.  Only an accepted empirical envelope selects the
            # contract's fixed 2 s duration for *each* 25% transition.
            self.feedforward_controller.configure_fixed_stage_transition_ramp(
                envelope.ramp_seconds
            )
            envelope.claim_single_use(self.empirical_claim_directory)
            self.empirical_stage_gate = EmpiricalStageGate(envelope)
            self.empirical_initialization_blocker = ""
        except Exception as exc:
            self.empirical_stage_gate = None
            self.empirical_initialization_blocker = str(exc)
            self.get_logger().error(
                "Empirical validation authority remains fail-closed: "
                + self.empirical_initialization_blocker
            )

    def _on_empirical_stage_confirmation(self, message: String) -> None:
        gate = self.empirical_stage_gate
        if gate is None:
            return
        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not gate.observe_confirmation(
            value, now_monotonic_ns=time.monotonic_ns()
        ):
            self.get_logger().warning(
                "Rejected invalid/replayed empirical stage confirmation"
            )

    def _on_joint_state(self, message: JointState) -> None:
        try:
            self.q_actual = normalize_named_joint_positions(
                message.name, message.position
            )
            self.joint_received_s = time.monotonic()
        except ValueError as exc:
            self.q_actual = None
            self.joint_received_s = None
            self.get_logger().warning(f"拒绝无效/joint_states：{exc}")

    def _on_hardware_state(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if value.get("schema") != "go-m8010-hardware-state/1.1":
                raise ValueError("硬件状态schema无效")
            session_id = value["session_id"]
            state_instance_id = value["state_instance_id"]
            if not isinstance(session_id, str) or not session_id:
                raise ValueError("session_id无效")
            if not isinstance(state_instance_id, str) or not state_instance_id:
                raise ValueError("state_instance_id无效")
            sequence = value.get("sequence")
            source_ns = value.get("source_monotonic_ns")
            position = value.get("position_rad")
            if (
                type(sequence) is not int
                or sequence <= 0
                or type(source_ns) is not int
                or source_ns <= 0
                or not isinstance(position, list)
                or len(position) != 6
                or not all(
                    type(item) in {int, float} and math.isfinite(float(item))
                    for item in position
                )
            ):
                raise ValueError("硬件状态姿态authority无效")
            received_ns = time.monotonic_ns()
            if (
                source_ns > received_ns
                or received_ns - source_ns > int(self.maximum_age_s * 1.0e9)
            ):
                raise ValueError("硬件状态内嵌时间戳已过期")
            if (
                state_instance_id == self.state_instance_id
                and self.hardware_state_sequence is not None
                and sequence <= self.hardware_state_sequence
            ):
                raise ValueError("硬件状态sequence重放或倒退")
            if (
                state_instance_id == self.state_instance_id
                and self.hardware_source_monotonic_ns is not None
                and source_ns <= self.hardware_source_monotonic_ns
            ):
                raise ValueError("硬件状态时间戳重放或倒退")
            self.session_id = session_id
            self.state_instance_id = state_instance_id
            self.hardware_q_actual = tuple(float(item) for item in position)
            self.hardware_state_sequence = sequence
            self.hardware_source_monotonic_ns = source_ns
            self.hardware_received_s = time.monotonic()
            self.latest_hardware_state = value
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.session_id = None
            self.state_instance_id = None
            self.hardware_q_actual = None
            self.hardware_state_sequence = None
            self.hardware_source_monotonic_ns = None
            self.hardware_received_s = None
            self.latest_hardware_state = None
            self.get_logger().warning(f"拒绝无效硬件状态：{exc}")

    def _on_planned_path_request(self, message: String) -> None:
        """Do an O(1) character bound, then hand the raw JSON to the worker."""

        serialized = message.data
        if (
            not isinstance(serialized, str)
            or len(serialized) > PLANNED_REQUEST_MAXIMUM_CHARACTERS
        ):
            self._invalidate_planned_path_request(
                "planned-path request exceeds character bound"
            )
            return
        with self._planned_lock:
            if self._planned_shutdown:
                shutdown = True
                old_future = None
                generation = self._planned_generation
                cancel_event = self._planned_cancel_event
            else:
                shutdown = False
                self._planned_cancel_event.set()
                old_future = self._planned_future
                self._planned_generation += 1
                generation = self._planned_generation
                cancel_event = threading.Event()
                self._planned_cancel_event = cancel_event
                self._planned_future = None
                self._planned_request = None
                self._planned_envelope = None
                self._planned_evaluation_blocker = (
                    "PLANNED_PATH_PARSE_AND_EVALUATION_PENDING"
                )
        if shutdown:
            self._invalidate_planned_path_request(
                "planned-path evaluator is shutting down"
            )
            return
        if old_future is not None:
            old_future.cancel()
        try:
            future = self._planned_executor.submit(
                self._decode_parse_and_evaluate_planned_path,
                serialized,
                cancel_event,
                generation,
            )
        except RuntimeError as exc:
            self._invalidate_planned_path_request(str(exc))
            return
        with self._planned_lock:
            if (
                generation == self._planned_generation
                and not self._planned_shutdown
            ):
                self._planned_future = future
            else:
                cancel_event.set()
                future.cancel()
        future.add_done_callback(
            lambda completed, generation=generation: (
                self._finish_planned_path_evaluation(generation, completed)
            )
        )

    def _invalidate_planned_path_request(self, reason: str) -> None:
        with self._planned_lock:
            self._planned_generation += 1
            self._planned_cancel_event.set()
            old_future = self._planned_future
            self._planned_cancel_event = threading.Event()
            self._planned_cancel_event.set()
            self._planned_future = None
            self._planned_request = None
            self._planned_envelope = None
            self._planned_evaluation_blocker = (
                "PLANNED_PATH_REQUEST_INVALID:" + reason
            )
        if old_future is not None:
            old_future.cancel()
        self.get_logger().warning(
            f"拒绝无效整轨负载证明请求：{reason}"
        )

    def _decode_parse_and_evaluate_planned_path(
        self,
        serialized: str,
        cancel_event: threading.Event,
        generation: int,
    ) -> tuple[Optional[PlannedPathRequest], Optional[PathLoadEnvelope], str]:
        try:
            if len(serialized.encode("utf-8")) > PLANNED_REQUEST_MAXIMUM_BYTES:
                raise PlannedPathError("planned-path request exceeds byte bound")
            value = json.loads(serialized)
            request = parse_planned_path_request(
                value,
                # A request that sat behind a cancelled worker must still be
                # fresh when this worker actually begins parsing it.
                now_monotonic_ns=time.monotonic_ns(),
                cancellation_requested=cancel_event.is_set,
            )
            with self._planned_lock:
                if (
                    cancel_event.is_set()
                    or generation != self._planned_generation
                    or self._planned_shutdown
                ):
                    raise PlannedPathError("planned-path evaluation cancelled")
                previous = self._planned_request_last_by_source.get(
                    request.source_instance_id
                )
                current = (request.sequence, request.source_monotonic_ns)
                if previous is not None and (
                    current[0] <= previous[0] or current[1] <= previous[1]
                ):
                    raise PlannedPathError("planned-path request replayed")
                self._planned_request_last_by_source.pop(
                    request.source_instance_id, None
                )
                self._planned_request_last_by_source[
                    request.source_instance_id
                ] = current
                while (
                    len(self._planned_request_last_by_source)
                    > MAXIMUM_PLANNED_REQUEST_SOURCES
                ):
                    oldest = next(iter(self._planned_request_last_by_source))
                    del self._planned_request_last_by_source[oldest]
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            return None, None, "PLANNED_PATH_REQUEST_INVALID:" + str(exc)
        try:
            envelope = self._evaluate_planned_path(request, cancel_event)
            return request, envelope, ""
        except Exception as exc:
            envelope = None
            blocker = "PLANNED_PATH_EVALUATION_FAILED:" + str(exc)
            return request, envelope, blocker

    def _evaluate_planned_path(
        self,
        request: PlannedPathRequest,
        cancel_event: threading.Event,
    ) -> PathLoadEnvelope:
        if cancel_event.is_set():
            raise PlannedPathError("planned-path evaluation cancelled")
        if self.anchor is None:
            raise PlannedPathError("MODEL_SESSION_ANCHOR_V2 unavailable")
        if not self.anchor.matches_runtime(
            request.session_id, request.state_instance_id
        ):
            raise PlannedPathError("planned path does not match model anchor")
        if self._path_evaluator is None:
            # This MuJoCo model/data pair is created, used and destroyed only
            # on the single proof worker thread.
            self._path_evaluator = StaticGravityEvaluator(self.model_path)
        return evaluate_path_load_envelope(
            request,
            anchor=self.anchor,
            evaluator=self._path_evaluator,
            j6_joint_to_rotor_scale=self.j6_joint_to_rotor_scale,
            cancellation_requested=cancel_event.is_set,
        )

    def _finish_planned_path_evaluation(
        self,
        generation: int,
        future: concurrent.futures.Future,
    ) -> None:
        try:
            request, envelope, blocker = future.result()
        except Exception as exc:
            request = None
            envelope = None
            blocker = "PLANNED_PATH_EVALUATION_FAILED:" + str(exc)
        with self._planned_lock:
            if generation != self._planned_generation:
                return
            self._planned_future = None
            self._planned_request = request
            self._planned_envelope = envelope
            self._planned_evaluation_blocker = blocker

    def _authority_blocker(self, now_s: float, now_ns: int) -> str:
        if self.initialization_blocker:
            return self.initialization_blocker
        if self.anchor is None or self.evaluator is None:
            return "重力模型authority未初始化"
        if self.hardware_received_s is None or now_s - self.hardware_received_s > self.maximum_age_s:
            return "硬件session状态已过期"
        if self.hardware_q_actual is None or self.hardware_state_sequence is None:
            return "硬件姿态authority不存在"
        feedback_blocker = hardware_pose_feedback_blocker(
            self.latest_hardware_state,
            now_monotonic_ns=now_ns,
            maximum_age_ns=int(self.maximum_age_s * 1.0e9),
            session_id=self.session_id or "",
            state_instance_id=self.state_instance_id or "",
        )
        if feedback_blocker:
            return feedback_blocker
        if not self.anchor.matches_runtime(
            self.session_id or "", self.state_instance_id or ""
        ):
            return "MODEL_SESSION_ANCHOR_V2与当前session/instance不匹配"
        return ""

    def _tick(self) -> None:
        now_s = time.monotonic()
        now_ns = time.monotonic_ns()
        blocker = self._authority_blocker(now_s, now_ns)
        self.hardware_enable_requested = bool(
            self.get_parameter("enabled_for_hardware").value
        )
        try:
            requested_gravity_scale_target = (
                self.feedforward_controller.validate_scale_target(
                    float(self.get_parameter("gravity_scale_target").value)
                )
            )
        except (TypeError, ValueError) as exc:
            requested_gravity_scale_target = 0.0
            blocker = blocker or str(exc)
        empirical_authority_ready = False
        empirical_status = {
            "authority_class": EMPIRICAL_AUTHORITY_CLASS,
            "rating_classification": EMPIRICAL_RATING_CLASSIFICATION,
            "envelope_id": None,
            "envelope_sha256": None,
            "anchor_sha256": self.anchor_sha256,
            "expires_at_utc": None,
            "stage_index": None,
            "stage_level": None,
            "stage_complete": False,
            "phase": "UNAVAILABLE",
            "position_validation_authorized": False,
            "assisted_teach_authorized": False,
            "hand_guidance_authorized": False,
            "return_only": False,
            "motion_warnings": [],
            "maximum_teach_excursion_deg": None,
            "maximum_teach_seconds": None,
            "maximum_teach_velocity_deg_s": None,
            "allowed_teach_joints": [],
            "maximum_position_segment_seconds": None,
            "maximum_cumulative_position_trajectory_seconds": None,
            "maximum_abs_position_segment_deg": None,
            "invalidated": False,
            "blocker": self.empirical_initialization_blocker or None,
            "continuous_operation_authorized": False,
            "official_continuous_rating_claimed": False,
        }
        if not blocker and self.empirical_stage_gate is not None:
            empirical_authority_ready = self.empirical_stage_gate.step(
                requested_scale=requested_gravity_scale_target,
                applied_scale=self.current_gravity_scale,
                hardware_state=self.latest_hardware_state,
                session_id=self.session_id or "",
                state_instance_id=self.state_instance_id or "",
                anchor_sha256=self.anchor_sha256,
                hardware_enable_requested=self.hardware_enable_requested,
                now_monotonic_ns=now_ns,
            )
            empirical_status = self.empirical_stage_gate.status()
        selected_torque_authority = select_runtime_torque_authority(
            official_continuous_authoritative=(
                self.continuous_rotor_limits_authoritative
            ),
            empirical_authoritative=empirical_authority_ready,
            empirical_envelope_configured=(
                self.empirical_stage_gate is not None
            ),
        )
        selected_authority_ready = selected_torque_authority in {
            "OFFICIAL_CONTINUOUS_RATING",
            EMPIRICAL_AUTHORITY_CLASS,
        }
        gravity_scale_target = (
            requested_gravity_scale_target
            if selected_authority_ready else 0.0
        )
        if not blocker:
            try:
                assert self.anchor is not None
                assert self.evaluator is not None
                assert self.hardware_q_actual is not None
                model_q = self.anchor.model_q_from_actual(self.hardware_q_actual)
                gravity = self.evaluator.evaluate(model_q)
                if not all(math.isfinite(value) for value in gravity):
                    raise ValueError("重力解算含非有限值")
                self.last_gravity_joint_nm = gravity
                (
                    self.current_gravity_scale,
                    self.last_feedforward_nm,
                    self.last_rotor_nm,
                ) = self.feedforward_controller.step(
                    gravity,
                    enabled=(
                        self.hardware_enable_requested
                        and selected_authority_ready
                    ),
                    target_scale=gravity_scale_target,
                    now_s=now_s,
                )
                self.last_calculation_s = now_s
                self.gravity_publisher.publish(Float64MultiArray(data=list(gravity)))
            except Exception as exc:
                blocker = f"重力解算失败：{exc}"
                self.last_gravity_joint_nm = None
                self.last_rotor_nm = None
                self.last_feedforward_nm = (0.0,) * 6
                self.current_gravity_scale = 0.0
                self.feedforward_controller.force_zero(now_s)
                self.last_calculation_s = None
        else:
            self.last_feedforward_nm = (0.0,) * 6
            self.current_gravity_scale = 0.0
            self.feedforward_controller.force_zero(now_s)

        age = (
            None
            if self.last_calculation_s is None
            else max(0.0, now_s - self.last_calculation_s)
        )
        hardware_authority_ready = bool(
            not blocker
            and self.hardware_enable_requested
            and selected_authority_ready
        )
        empirical_status["assisted_teach_authorized"] = bool(
            empirical_status.get("assisted_teach_authorized") is True
            and empirical_status.get("return_only") is not True
            and hardware_authority_ready
            and selected_torque_authority == EMPIRICAL_AUTHORITY_CLASS
            and abs(self.current_gravity_scale - 1.0) <= 1.0e-6
            and gravity_scale_target == 1.0
        )
        empirical_status["hand_guidance_authorized"] = bool(
            empirical_status.get("hand_guidance_authorized") is True
            and empirical_status["assisted_teach_authorized"]
        )
        self.gravity_status_sequence += 1
        q_actual_sha256 = (
            hashlib.sha256(json.dumps(
                list(self.hardware_q_actual),
                allow_nan=False,
                separators=(",", ":"),
            ).encode("ascii")).hexdigest()
            if self.hardware_q_actual is not None else None
        )
        joint_state_crosscheck = (
            None
            if (
                self.q_actual is None
                or self.joint_received_s is None
                or now_s - self.joint_received_s > self.maximum_age_s
                or self.hardware_q_actual is None
            )
            else all(
                abs(joint_state - hardware_state) <= 1.0e-6
                for joint_state, hardware_state in zip(
                    self.q_actual, self.hardware_q_actual
                )
            )
        )
        with self._planned_lock:
            planned_request = self._planned_request
            planned_envelope = self._planned_envelope
            planned_evaluation_blocker = self._planned_evaluation_blocker
        temperature_authoritative = False
        minimum_thermal_margin_c = None
        hardware_continuous_authoritative = False
        temperature_blocker = "PLANNED_PATH_REQUEST_MISSING"
        if planned_request is not None and self.thermal_derating_start_c is not None:
            (
                temperature_observed,
                minimum_thermal_margin_c,
                hardware_continuous_authoritative,
                temperature_blocker,
            ) = temperature_observation(
                self.latest_hardware_state,
                session_id=planned_request.session_id,
                state_instance_id=planned_request.state_instance_id,
                derating_start_c=self.thermal_derating_start_c,
            )
            temperature_authoritative = bool(
                self.temperature_limit_config_authoritative
                and temperature_observed
            )
            if blocker:
                hardware_continuous_authoritative = False
                temperature_authoritative = False
                temperature_blocker = "RUNTIME_GRAVITY_AUTHORITY_INVALID"
            elif (
                planned_request.session_id != self.session_id
                or planned_request.state_instance_id != self.state_instance_id
            ):
                hardware_continuous_authoritative = False
                temperature_authoritative = False
                temperature_blocker = "PLANNED_PATH_RUNTIME_SESSION_MISMATCH"
        elif planned_request is not None:
            temperature_blocker = "THERMAL_CONFIG_THRESHOLD_UNAVAILABLE"
        planned_proof = build_planned_path_proof(
            planned_envelope,
            request=planned_request,
            source_instance_id=self.gravity_source_instance_id,
            sequence=self.gravity_status_sequence,
            source_monotonic_ns=now_ns,
            model_sha256=self.model_sha256,
            gravity_config_sha256=GRAVITY_CONFIG_SHA256,
            thermal_config_sha256=self.thermal_config_sha256,
            continuous_config_authoritative=(
                self.continuous_rotor_limits_authoritative
            ),
            continuous_hardware_authoritative=(
                hardware_continuous_authoritative
            ),
            continuous_rotor_limits_nm=self.continuous_rotor_limits_nm,
            short_peak_rotor_limits_nm=self.short_peak_rotor_limits_nm,
            temperature_limits_authoritative=temperature_authoritative,
            minimum_thermal_margin_c=minimum_thermal_margin_c,
            temperature_blocker=temperature_blocker,
            evaluation_blocker=planned_evaluation_blocker,
            empirical_validation_authoritative=bool(
                empirical_authority_ready
                and empirical_status.get(
                    "position_validation_authorized"
                ) is True
            ),
            empirical_rotor_limits_nm=(
                SOFTWARE_GRAVITY_ROTOR_LIMIT_NM
            ),
            empirical_envelope_id=empirical_status.get("envelope_id"),
            empirical_envelope_sha256=empirical_status.get(
                "envelope_sha256"
            ),
        )
        status = {
            "schema": "go-m8010-gravity-status/1.1",
            "source": "whole_arm_gravity_node",
            "source_instance_id": self.gravity_source_instance_id,
            "sequence": self.gravity_status_sequence,
            "source_monotonic_ns": now_ns,
            "model_sha256": self.model_sha256,
            "production_model_hash_match": self.model_sha256 == PRODUCTION_MODEL_SHA256,
            "gravity_config_sha256": GRAVITY_CONFIG_SHA256,
            "anchor_valid": not bool(blocker),
            "session_id": self.session_id,
            "state_instance_id": self.state_instance_id,
            "hardware_state_sequence": self.hardware_state_sequence,
            "hardware_state_source_monotonic_ns": self.hardware_source_monotonic_ns,
            "q_actual_sha256": q_actual_sha256,
            # /joint_states is a UI mirror without session/sequence identity.
            # It is useful as a diagnostic cross-check but must not gate the
            # atomic hardware_state pose used by the gravity controller.
            "joint_state_crosscheck": joint_state_crosscheck,
            "calculation_rate_hz": self.rate_hz,
            "last_update_age_s": age,
            "gravity_joint_nm": self.last_gravity_joint_nm,
            "feedforward_nm": list(self.last_feedforward_nm),
            "gravity_scale": self.current_gravity_scale,
            "gravity_scale_target": gravity_scale_target,
            "gravity_scale_requested": requested_gravity_scale_target,
            "finite_bounded": bool(
                not blocker
                and self.last_gravity_joint_nm is not None
                and all(math.isfinite(value) for value in self.last_feedforward_nm)
            ),
            "sign_consistency": "PENDING_POWERED_HARDWARE_DIRECTION_VALIDATION",
            "maximum_predicted_rotor_torque_nm": (
                max(abs(value) for value in self.last_rotor_nm.values())
                if self.last_rotor_nm else None
            ),
            "pose_feasibility": (
                "PASS" if hardware_authority_ready
                else "BLOCKED" if blocker
                else "BLOCKED_EMPIRICAL_VALIDATION_AUTHORITY"
            ),
            "blocker": (
                blocker
                or (
                    None if hardware_authority_ready
                    else empirical_status.get("blocker")
                    or "EMPIRICAL_VALIDATION_AUTHORITY_NOT_READY"
                )
            ),
            "continuous_rotor_limits_authoritative": (
                self.continuous_rotor_limits_authoritative
            ),
            "empirical_validation_authoritative": (
                empirical_authority_ready
            ),
            "torque_authority_class": selected_torque_authority,
            "empirical_validation": empirical_status,
            "hardware_enable_requested": self.hardware_enable_requested,
            # The node remains read-only; this flag means that its bounded
            # authority is consumable by the Router/GO path, not that this
            # process owns a motor socket.
            "actuation_interface_present": True,
            "hardware_tff_enabled": bool(
                hardware_authority_ready and gravity_scale_target > 0.0
            ),
            "planned_trajectory_feasibility": planned_proof,
        }
        hardware = self.latest_hardware_state or {}
        exit_hold = hardware.get("assisted_teach_exit_hold")
        if ((isinstance(exit_hold, dict) and exit_hold.get("restricted") is True)
                or hardware.get("assisted_teach_exit_hold_validated") is False):
            # Non-authority diagnostic from the very snapshot used above.
            status["assisted_teach_exit_hold"] = exit_hold
            status["assisted_teach_exit_hold_source_monotonic_ns"] = hardware.get("assisted_teach_exit_hold_source_monotonic_ns")
            status["assisted_teach_exit_hold_validated"] = bool(
                hardware.get("assisted_teach_exit_hold_validated") is True
                and all(hardware.get(key) == status.get(key) for key in ("session_id", "state_instance_id"))
                and hardware.get("sequence") == status["hardware_state_sequence"]
                and hardware.get("source_monotonic_ns") == status["hardware_state_source_monotonic_ns"])
        self.status_publisher.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )

    def destroy_node(self):
        with self._planned_lock:
            self._planned_shutdown = True
            self._planned_generation += 1
            self._planned_cancel_event.set()
            if self._planned_future is not None:
                self._planned_future.cancel()
            self._planned_future = None
        self._planned_executor.shutdown(wait=True, cancel_futures=True)
        self._path_evaluator = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WholeArmGravityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
