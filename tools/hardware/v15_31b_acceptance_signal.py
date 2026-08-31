#!/usr/bin/env python3
"""Offline-first ROS signal client for the V15.31B acceptance runner.

Examples::

    # Print one bound confirmation without creating a ROS node.
    python3 v15_31b_acceptance_signal.py --envelope ENV \
      --anchor-validation ANCHOR --expected-envelope-sha256 SHA \
      confirmation --target-gravity-scale 0.25

    # Live 10-second confirmation heartbeat until interrupted.
    python3 v15_31b_acceptance_signal.py --mode live --enable-live TOKEN \
      --envelope ENV --anchor-validation ANCHOR \
      --expected-envelope-sha256 SHA confirmation \
      --target-gravity-scale 1 --heartbeat-interval-seconds 10 \
      --repeat-count 0

This process can only publish the empirical confirmation topic or the
acceptance-control topic.  It never publishes a GUI command, synthesizes a
POSITION trajectory, or opens CAN, serial, or a worker UDP socket.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import secrets
import sys
import time
from typing import Any, Callable, Mapping, Optional, Sequence


# Running this file directly puts tools/hardware on sys.path.  Explicitly add
# it as well so importlib-based offline tests use the exact same runner module.
_TOOLS_DIRECTORY = Path(__file__).resolve().parent
if str(_TOOLS_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIRECTORY))

from v15_31b_active_acceptance_runner import (  # noqa: E402
    AcceptanceError,
    CONFIRMATION_SCHEMA,
    CONTROL_SCHEMA,
    EvidenceBinding,
    GRAVITY_LADDER_LEVELS,
    LIVE_ENABLE_TOKEN,
    PHYSICAL_CONFIRMATION_ENV,
    PHYSICAL_CONFIRMATION_GATE,
    validate_physical_startup_gate,
)


CONFIRMATION_TOPIC = "/whole_arm/empirical_stage_confirmation"
CONTROL_TOPIC = "/whole_arm/v15_31b/acceptance_control"
TOPIC_BY_SIGNAL = {
    "confirmation": CONFIRMATION_TOPIC,
    "control": CONTROL_TOPIC,
}
DRY_RUN_SCHEMA = "go-m8010-v15-31b-acceptance-signal-dry-run/1.0"
MAXIMUM_SEQUENCE = (1 << 63) - 1
MAXIMUM_HEARTBEAT_INTERVAL_SECONDS = 10.0
MAXIMUM_FINITE_REPEAT_COUNT = 100
MAXIMUM_EXTRA_JSON_BYTES = 65_536
SUBSCRIPTION_DISCOVERY_TIMEOUT_SECONDS = 5.0
POST_PUBLICATION_ACK_TIMEOUT_SECONDS = 0.5
ROS_SPIN_SLICE_SECONDS = 0.05
CONTROL_ACTIONS = frozenset({
    "START_GRAVITY_LADDER",
    "START_POSITION",
    "START_MULTI_JOINT",
    "START_THERMAL_SETUP",
    "START_RESTORE_INITIAL",
    "START_J2_COMPARISON",
    "STOP_J2_COMPARISON",
    "START_THERMAL_STAGE",
    "APPROVE_THERMAL_STAGE",
    "FINALIZE_THERMAL",
    "RECORD_MULTI_JOINT",
    "RECORD_RESTORE_INITIAL",
    "RECORD_GUI_POWERED",
    "FINAL_BRAKE",
    "FINALIZE_RESULT",
    "STOP_AND_BRAKE",
})
COMMON_PAYLOAD_FIELDS = frozenset({
    "schema",
    "source_instance_id",
    "sequence",
    "source_monotonic_ns",
    "envelope_id",
    "envelope_sha256",
    "session_id",
    "state_instance_id",
})
CONTROL_RESERVED_FIELDS = COMMON_PAYLOAD_FIELDS | {"anchor_sha256", "action"}


class AcceptanceSignalError(RuntimeError):
    """Bounded local signal-client failure."""


def _require(condition: bool, reason: str) -> None:
    if not condition:
        raise AcceptanceSignalError(reason)


def _valid_source_instance_id(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and len(value) == 32
        and all(character in "0123456789abcdef" for character in value)
    )


def _strict_object_pairs(pairs: Sequence[tuple[str, object]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise AcceptanceSignalError(f"EXTRA_JSON_DUPLICATE_KEY:{key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> object:
    raise AcceptanceSignalError(f"EXTRA_JSON_NONFINITE:{value}")


def validate_extra_fields(value: object) -> None:
    _require(isinstance(value, Mapping), "CONTROL_EXTRA_FIELDS_INVALID")
    _require(
        all(isinstance(key, str) and key for key in value),
        "EXTRA_JSON_KEY_INVALID",
    )
    try:
        json.dumps(value, ensure_ascii=True, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise AcceptanceSignalError("EXTRA_JSON_VALUE_INVALID") from exc


def parse_extra_json(text: str) -> dict[str, Any]:
    """Parse one bounded JSON object with no duplicates or non-finite values."""

    _require(isinstance(text, str), "EXTRA_JSON_TEXT_REQUIRED")
    _require(
        len(text.encode("utf-8")) <= MAXIMUM_EXTRA_JSON_BYTES,
        "EXTRA_JSON_TOO_LARGE",
    )
    try:
        value = json.loads(
            text,
            object_pairs_hook=_strict_object_pairs,
            parse_constant=_reject_json_constant,
        )
    except AcceptanceSignalError:
        raise
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise AcceptanceSignalError("EXTRA_JSON_INVALID") from exc
    _require(isinstance(value, dict), "EXTRA_JSON_ROOT_NOT_OBJECT")
    validate_extra_fields(value)
    return value


@dataclass(frozen=True)
class SignalRequest:
    kind: str
    target_gravity_scale: Optional[float] = None
    action: Optional[str] = None
    extra_fields: Optional[Mapping[str, Any]] = None
    heartbeat_interval_seconds: float = 0.0
    repeat_count: int = 1

    def validate(self) -> None:
        _require(self.kind in TOPIC_BY_SIGNAL, "SIGNAL_KIND_INVALID")
        if self.kind == "confirmation":
            _require(
                type(self.target_gravity_scale) in {int, float}
                and math.isfinite(float(self.target_gravity_scale))
                and float(self.target_gravity_scale) in GRAVITY_LADDER_LEVELS,
                "CONFIRMATION_TARGET_NOT_APPROVED_LADDER_LEVEL",
            )
            _require(self.action is None, "CONFIRMATION_ACTION_FORBIDDEN")
            _require(not self.extra_fields, "CONFIRMATION_EXTRA_FIELDS_FORBIDDEN")
            validate_confirmation_schedule(
                self.heartbeat_interval_seconds, self.repeat_count
            )
        else:
            _require(self.target_gravity_scale is None, "CONTROL_TARGET_FORBIDDEN")
            _require(
                isinstance(self.action, str) and self.action in CONTROL_ACTIONS,
                "CONTROL_ACTION_INVALID",
            )
            _require(
                self.heartbeat_interval_seconds == 0.0
                and self.repeat_count == 1,
                "CONTROL_REPEAT_FORBIDDEN",
            )
            extras = {} if self.extra_fields is None else self.extra_fields
            validate_extra_fields(extras)
            _require(
                not (set(extras) & CONTROL_RESERVED_FIELDS),
                "CONTROL_EXTRA_FIELDS_OVERRIDE_RESERVED",
            )


def validate_confirmation_schedule(interval_seconds: float, repeat_count: int) -> None:
    _require(
        type(interval_seconds) in {int, float}
        and math.isfinite(float(interval_seconds))
        and float(interval_seconds) >= 0.0,
        "CONFIRMATION_HEARTBEAT_INTERVAL_INVALID",
    )
    _require(
        type(repeat_count) is int
        and 0 <= repeat_count <= MAXIMUM_FINITE_REPEAT_COUNT,
        "CONFIRMATION_REPEAT_COUNT_INVALID",
    )
    interval = float(interval_seconds)
    if repeat_count == 1:
        _require(interval == 0.0, "SINGLE_CONFIRMATION_INTERVAL_MUST_BE_ZERO")
        return
    _require(
        0.0 < interval <= MAXIMUM_HEARTBEAT_INTERVAL_SECONDS,
        "PERIODIC_CONFIRMATION_INTERVAL_OUT_OF_RANGE",
    )
    _require(
        repeat_count == 0 or repeat_count >= 2,
        "PERIODIC_CONFIRMATION_REPEAT_COUNT_INVALID",
    )


class SignalPayloadSequence:
    """One stable source with strictly increasing sequence and monotonic time."""

    def __init__(
        self,
        binding: EvidenceBinding,
        *,
        source_instance_id: Optional[str] = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.binding = binding
        self.source_instance_id = (
            secrets.token_hex(16)
            if source_instance_id is None
            else source_instance_id
        )
        _require(
            _valid_source_instance_id(self.source_instance_id),
            "SOURCE_INSTANCE_ID_INVALID",
        )
        self._monotonic_ns = monotonic_ns
        self._sequence = 0
        self._last_source_monotonic_ns = 0

    def _next_source(self) -> tuple[int, int]:
        _require(self._sequence < MAXIMUM_SEQUENCE, "SOURCE_SEQUENCE_EXHAUSTED")
        candidate = self._monotonic_ns()
        _require(
            type(candidate) is int and candidate > 0,
            "SOURCE_MONOTONIC_CLOCK_INVALID",
        )
        source_ns = max(candidate, self._last_source_monotonic_ns + 1)
        self._sequence += 1
        self._last_source_monotonic_ns = source_ns
        return self._sequence, source_ns

    def _common(self, schema: str) -> dict[str, Any]:
        sequence, source_ns = self._next_source()
        return {
            "schema": schema,
            "source_instance_id": self.source_instance_id,
            "sequence": sequence,
            "source_monotonic_ns": source_ns,
            "envelope_id": self.binding.envelope_id,
            "envelope_sha256": self.binding.envelope_sha256,
            "session_id": self.binding.session_id,
            "state_instance_id": self.binding.state_instance_id,
        }

    def confirmation(self, target_gravity_scale: float) -> dict[str, Any]:
        request = SignalRequest(
            kind="confirmation",
            target_gravity_scale=target_gravity_scale,
        )
        request.validate()
        return {
            **self._common(CONFIRMATION_SCHEMA),
            "target_gravity_scale": float(target_gravity_scale),
            "operator_stop_ready": True,
            "j2_j3_support_reliable": True,
            "clearance_confirmed": True,
            "no_person_contact": True,
        }

    def control(
        self, action: str, extra_fields: Optional[Mapping[str, Any]] = None,
    ) -> dict[str, Any]:
        normalized_action = action.strip().upper() if isinstance(action, str) else action
        extras = {} if extra_fields is None else dict(extra_fields)
        request = SignalRequest(
            kind="control",
            action=normalized_action,
            extra_fields=extras,
        )
        request.validate()
        return {
            **self._common(CONTROL_SCHEMA),
            "anchor_sha256": self.binding.anchor_sha256,
            "action": normalized_action,
            **extras,
        }


def load_evidence_binding(args: argparse.Namespace) -> EvidenceBinding:
    _require(args.envelope is not None, "--envelope is required")
    _require(
        args.anchor_validation is not None,
        "--anchor-validation is required",
    )
    _require(
        bool(args.expected_envelope_sha256),
        "--expected-envelope-sha256 is required",
    )
    return EvidenceBinding.from_paths(
        args.envelope,
        args.anchor_validation,
        args.expected_envelope_sha256,
    )


def request_from_args(args: argparse.Namespace) -> SignalRequest:
    if args.signal == "confirmation":
        repeat_count = (
            1 if args.heartbeat_interval_seconds == 0.0 else 0
        ) if args.repeat_count is None else args.repeat_count
        request = SignalRequest(
            kind="confirmation",
            target_gravity_scale=args.target_gravity_scale,
            heartbeat_interval_seconds=args.heartbeat_interval_seconds,
            repeat_count=repeat_count,
        )
    elif args.signal == "control":
        request = SignalRequest(
            kind="control",
            action=str(args.action).strip().upper(),
            extra_fields=parse_extra_json(args.extra_json),
        )
    else:
        raise AcceptanceSignalError("SIGNAL_KIND_REQUIRED")
    request.validate()
    return request


def _next_payload(
    request: SignalRequest, sequence: SignalPayloadSequence,
) -> dict[str, Any]:
    if request.kind == "confirmation":
        assert request.target_gravity_scale is not None
        return sequence.confirmation(request.target_gravity_scale)
    assert request.action is not None
    return sequence.control(request.action, request.extra_fields)


def dry_run_plan() -> dict[str, Any]:
    return {
        "schema": DRY_RUN_SCHEMA,
        "mode": "OFFLINE_DRY_RUN",
        "requires_evidence_for_payload": True,
        "allowed_topics": [CONFIRMATION_TOPIC, CONTROL_TOPIC],
        "publishes_ros": False,
        "opens_can": False,
        "opens_serial": False,
        "opens_worker_udp": False,
        "publishes_gui_command": False,
        "synthesizes_position": False,
        "live_environment_gate": PHYSICAL_CONFIRMATION_ENV,
        "live_enable_token": LIVE_ENABLE_TOKEN,
        "confirmation_levels": list(GRAVITY_LADDER_LEVELS),
        "maximum_heartbeat_interval_seconds": (
            MAXIMUM_HEARTBEAT_INTERVAL_SECONDS
        ),
    }


def dry_run_signal(
    request: SignalRequest, binding: EvidenceBinding,
) -> dict[str, Any]:
    request.validate()
    sequence = SignalPayloadSequence(binding)
    preview_count = (
        2 if request.kind == "confirmation" and request.repeat_count == 0
        else request.repeat_count
    )
    payloads = [_next_payload(request, sequence) for _ in range(preview_count)]
    return {
        **dry_run_plan(),
        "signal": request.kind,
        "topic": TOPIC_BY_SIGNAL[request.kind],
        "planned_publication_count": (
            "UNTIL_INTERRUPTED"
            if request.kind == "confirmation" and request.repeat_count == 0
            else request.repeat_count
        ),
        "heartbeat_interval_seconds": request.heartbeat_interval_seconds,
        "preview_payloads": payloads,
        "binding": {
            "envelope_id": binding.envelope_id,
            "envelope_sha256": binding.envelope_sha256,
            "session_id": binding.session_id,
            "state_instance_id": binding.state_instance_id,
            "anchor_sha256": binding.anchor_sha256,
        },
    }


def validate_live_preconditions(
    *,
    binding: EvidenceBinding,
    enable_live: str,
    environment: Mapping[str, str],
) -> None:
    _require(enable_live == LIVE_ENABLE_TOKEN, "LIVE_ENABLE_TOKEN_MISSING")
    validate_physical_startup_gate(environment)
    binding.ensure_not_expired()


def wait_for_subscription(
    *,
    node: object,
    publisher: object,
    spin_once: Callable[[object, float], None],
    timeout_seconds: float = SUBSCRIPTION_DISCOVERY_TIMEOUT_SECONDS,
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    """Wait for one matched subscriber without publishing a stale payload."""

    _require(
        type(timeout_seconds) in {int, float}
        and math.isfinite(float(timeout_seconds))
        and 0.0 <= float(timeout_seconds)
        <= SUBSCRIPTION_DISCOVERY_TIMEOUT_SECONDS,
        "ROS_SUBSCRIPTION_DISCOVERY_TIMEOUT_INVALID",
    )
    deadline = monotonic() + float(timeout_seconds)
    while True:
        count = publisher.get_subscription_count()
        _require(
            type(count) is int and count >= 0,
            "ROS_SUBSCRIPTION_COUNT_INVALID",
        )
        if count >= 1:
            return
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            raise AcceptanceSignalError("ROS_SUBSCRIBER_DISCOVERY_TIMEOUT")
        spin_once(node, min(ROS_SPIN_SLICE_SECONDS, remaining))


def publish_payload_once(
    *,
    node: object,
    publisher: object,
    message: object,
    spin_once: Callable[[object, float], None],
    acknowledgement_timeout: object,
) -> None:
    """Publish exactly once, then require reliable acknowledgement."""

    publisher.publish(message)
    spin_once(node, ROS_SPIN_SLICE_SECONDS)
    wait_for_all_acked = getattr(publisher, "wait_for_all_acked", None)
    _require(
        callable(wait_for_all_acked),
        "ROS_PUBLICATION_ACK_API_UNAVAILABLE",
    )
    _require(
        wait_for_all_acked(timeout=acknowledgement_timeout) is True,
        "ROS_PUBLICATION_ACK_TIMEOUT",
    )
    _require(
        publisher.get_subscription_count() >= 1,
        "ROS_SUBSCRIBER_LOST_DURING_PUBLICATION",
    )
    spin_once(node, ROS_SPIN_SLICE_SECONDS)


def spin_until(
    *,
    node: object,
    deadline: float,
    spin_once: Callable[[object, float], None],
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    """Keep DDS progressing until one monotonic periodic deadline."""

    while True:
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            return
        spin_once(node, min(ROS_SPIN_SLICE_SECONDS, remaining))


def run_live(
    request: SignalRequest,
    binding: EvidenceBinding,
    *,
    enable_live: str,
    environment: Mapping[str, str],
) -> int:
    """Publish only after the runner-identical live gates pass."""

    request.validate()
    validate_live_preconditions(
        binding=binding,
        enable_live=enable_live,
        environment=environment,
    )
    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise AcceptanceSignalError("ROS2_PYTHON_RUNTIME_UNAVAILABLE") from exc

    sequence = SignalPayloadSequence(binding)
    topic = TOPIC_BY_SIGNAL[request.kind]
    node = None
    rclpy.init()
    try:
        node = Node("v15_31b_acceptance_signal")
        publisher = node.create_publisher(String, topic, 10)
        remaining = request.repeat_count
        publication_count = 0
        while True:
            binding.ensure_not_expired()
            # The initial DDS match may take seconds.  Later loss is terminal:
            # waiting again would violate the requested heartbeat period and
            # a control retry would be rejected by the runner as a replay.
            wait_for_subscription(
                node=node,
                publisher=publisher,
                spin_once=lambda current, timeout: rclpy.spin_once(
                    current, timeout_sec=timeout
                ),
                timeout_seconds=(
                    SUBSCRIPTION_DISCOVERY_TIMEOUT_SECONDS
                    if publication_count == 0 else 0.0
                ),
            )
            payload = _next_payload(request, sequence)
            message = String(data=json.dumps(
                payload,
                ensure_ascii=True,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ))
            published_at = time.monotonic()
            publish_payload_once(
                node=node,
                publisher=publisher,
                message=message,
                spin_once=lambda current, timeout: rclpy.spin_once(
                    current, timeout_sec=timeout
                ),
                acknowledgement_timeout=Duration(
                    seconds=POST_PUBLICATION_ACK_TIMEOUT_SECONDS
                ),
            )
            publication_count += 1
            print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
            if remaining > 0:
                remaining -= 1
                if remaining == 0:
                    break
            spin_until(
                node=node,
                deadline=(
                    published_at + request.heartbeat_interval_seconds
                ),
                spin_once=lambda current, timeout: rclpy.spin_once(
                    current, timeout_sec=timeout
                ),
            )
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("dry-run", "live"), default="dry-run")
    parser.add_argument("--envelope", type=Path)
    parser.add_argument("--anchor-validation", type=Path)
    parser.add_argument("--expected-envelope-sha256", default="")
    parser.add_argument("--enable-live", default="")
    subparsers = parser.add_subparsers(dest="signal")

    confirmation = subparsers.add_parser("confirmation")
    confirmation.add_argument(
        "--target-gravity-scale", type=float, required=True,
    )
    confirmation.add_argument(
        "--heartbeat-interval-seconds", type=float, default=0.0,
    )
    confirmation.add_argument(
        "--repeat-count",
        type=int,
        default=None,
        help=(
            "default: single when interval=0, otherwise until interrupted; "
            "1=single, 0=until interrupted, 2..100=bounded heartbeat"
        ),
    )

    control = subparsers.add_parser("control")
    control.add_argument("--action", required=True)
    control.add_argument("--extra-json", default="{}")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.signal is None:
        _require(args.mode == "dry-run", "LIVE_SIGNAL_KIND_REQUIRED")
        print(json.dumps(dry_run_plan(), ensure_ascii=False, indent=2))
        return 0
    binding = load_evidence_binding(args)
    request = request_from_args(args)
    if args.mode == "dry-run":
        print(json.dumps(
            dry_run_signal(request, binding),
            ensure_ascii=False,
            allow_nan=False,
            indent=2,
        ))
        return 0
    return run_live(
        request,
        binding,
        enable_live=args.enable_live,
        environment=os.environ,
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AcceptanceError, AcceptanceSignalError) as exc:
        print(f"V15.31B ACCEPTANCE SIGNAL BLOCKED: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
