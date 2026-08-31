from __future__ import annotations

import ast
import argparse
from dataclasses import replace
from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).with_name("v15_31b_acceptance_signal.py")
SPEC = importlib.util.spec_from_file_location(
    "v15_31b_acceptance_signal", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
signal_client = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = signal_client
SPEC.loader.exec_module(signal_client)
runner = sys.modules["v15_31b_active_acceptance_runner"]


def binding():
    return signal_client.EvidenceBinding(
        envelope_id="v15-31b-empirical-0123456789abcdef",
        envelope_sha256="a" * 64,
        session_id=(
            "persistent:0123456789abcdef:"
            "j2session:1111111111111111:"
            "goauxsession:2222222222222222"
        ),
        state_instance_id="3" * 32,
        anchor_sha256="b" * 64,
        expires_at_utc=datetime.now(timezone.utc) + timedelta(hours=2),
        maximum_position_seconds=600.0,
        maximum_segment_seconds=15.0,
        maximum_segment_displacement_deg=5.0,
        maximum_stage_temperature_rise_c=2.0,
    )


def test_imports_the_exact_runner_evidence_binding_and_live_gates() -> None:
    assert signal_client.EvidenceBinding is runner.EvidenceBinding
    assert signal_client.LIVE_ENABLE_TOKEN == runner.LIVE_ENABLE_TOKEN
    assert signal_client.PHYSICAL_CONFIRMATION_ENV == runner.PHYSICAL_CONFIRMATION_ENV
    assert signal_client.PHYSICAL_CONFIRMATION_GATE == runner.PHYSICAL_CONFIRMATION_GATE


def test_binding_loader_delegates_exact_paths_to_runner_class(
    monkeypatch, tmp_path: Path,
) -> None:
    expected = binding()
    calls = []

    def fake_from_paths(cls, envelope, anchor_validation, envelope_sha256):
        calls.append((envelope, anchor_validation, envelope_sha256))
        return expected

    monkeypatch.setattr(
        signal_client.EvidenceBinding,
        "from_paths",
        classmethod(fake_from_paths),
    )
    arguments = argparse.Namespace(
        envelope=tmp_path / "empirical_envelope.json",
        anchor_validation=tmp_path / "model_session_anchor_validation.json",
        expected_envelope_sha256="a" * 64,
    )

    assert signal_client.load_evidence_binding(arguments) is expected
    assert calls == [(
        arguments.envelope,
        arguments.anchor_validation,
        arguments.expected_envelope_sha256,
    )]


def test_default_invocation_is_inert_offline_plan(capsys) -> None:
    assert signal_client.main([]) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan["mode"] == "OFFLINE_DRY_RUN"
    assert plan["publishes_ros"] is False
    assert plan["opens_can"] is False
    assert plan["opens_serial"] is False
    assert plan["opens_worker_udp"] is False
    assert plan["publishes_gui_command"] is False
    assert plan["synthesizes_position"] is False
    assert plan["allowed_topics"] == [
        signal_client.CONFIRMATION_TOPIC,
        signal_client.CONTROL_TOPIC,
    ]


@pytest.mark.parametrize("target", runner.GRAVITY_LADDER_LEVELS)
def test_confirmation_payload_is_exact_and_runner_accepted(target: float) -> None:
    bound = binding()
    sequence = signal_client.SignalPayloadSequence(
        bound,
        source_instance_id="1" * 32,
        monotonic_ns=lambda: 5_000_000_000,
    )
    payload = sequence.confirmation(target)
    acceptance = runner.ActiveAcceptanceRunner(bound)

    assert set(payload) == {
        "schema",
        "source_instance_id",
        "sequence",
        "source_monotonic_ns",
        "envelope_id",
        "envelope_sha256",
        "session_id",
        "state_instance_id",
        "target_gravity_scale",
        "operator_stop_ready",
        "j2_j3_support_reliable",
        "clearance_confirmed",
        "no_person_contact",
    }
    assert payload["target_gravity_scale"] == target
    assert acceptance.observe_confirmation(
        payload, now_ns=payload["source_monotonic_ns"]
    ) is True


def test_repeated_payloads_keep_source_and_strictly_increase_identity() -> None:
    bound = binding()
    sequence = signal_client.SignalPayloadSequence(
        bound,
        source_instance_id="2" * 32,
        monotonic_ns=lambda: 6_000_000_000,
    )
    first = sequence.confirmation(0.25)
    second = sequence.confirmation(0.25)
    acceptance = runner.ActiveAcceptanceRunner(bound)

    assert first["source_instance_id"] == second["source_instance_id"]
    assert (first["sequence"], second["sequence"]) == (1, 2)
    assert second["source_monotonic_ns"] == first["source_monotonic_ns"] + 1
    assert acceptance.observe_confirmation(
        first, now_ns=first["source_monotonic_ns"]
    ) is True
    assert acceptance.observe_confirmation(
        second, now_ns=second["source_monotonic_ns"]
    ) is True
    assert acceptance.observe_confirmation(
        first, now_ns=second["source_monotonic_ns"]
    ) is False


def test_control_payload_preserves_extra_json_and_runner_binding() -> None:
    bound = binding()
    sequence = signal_client.SignalPayloadSequence(
        bound,
        source_instance_id="4" * 32,
        monotonic_ns=lambda: 7_000_000_000,
    )
    extras = {"duration_min": 5, "target_j2_rad": 0.125}
    payload = sequence.control("start_thermal_stage", extras)
    acceptance = runner.ActiveAcceptanceRunner(bound)

    assert payload["schema"] == runner.CONTROL_SCHEMA
    assert payload["action"] == "START_THERMAL_STAGE"
    assert payload["anchor_sha256"] == bound.anchor_sha256
    assert payload["duration_min"] == 5
    assert payload["target_j2_rad"] == 0.125
    assert acceptance.validate_control(
        payload, now_ns=payload["source_monotonic_ns"]
    ) == payload


@pytest.mark.parametrize(
    "text, reason",
    (
        ('{"duration_min":5,"duration_min":15}', "DUPLICATE_KEY"),
        ('{"value":NaN}', "NONFINITE"),
        ("[1,2,3]", "ROOT_NOT_OBJECT"),
        ("not-json", "INVALID"),
    ),
)
def test_extra_json_parser_is_strict(text: str, reason: str) -> None:
    with pytest.raises(signal_client.AcceptanceSignalError, match=reason):
        signal_client.parse_extra_json(text)


def test_control_extra_fields_cannot_replace_provenance_or_be_nonfinite() -> None:
    sequence = signal_client.SignalPayloadSequence(binding())
    with pytest.raises(
        signal_client.AcceptanceSignalError,
        match="OVERRIDE_RESERVED",
    ):
        sequence.control("START_POSITION", {"session_id": "forged"})
    with pytest.raises(
        signal_client.AcceptanceSignalError,
        match="VALUE_INVALID",
    ):
        sequence.control("START_POSITION", {"target": float("nan")})


@pytest.mark.parametrize(
    "interval,count,accepted",
    (
        (0.0, 1, True),
        (10.0, 0, True),
        (0.1, 2, True),
        (10.0001, 0, False),
        (0.0, 0, False),
        (1.0, 1, False),
        (1.0, 101, False),
        (-1.0, 2, False),
    ),
)
def test_confirmation_schedule_is_single_or_at_most_ten_seconds(
    interval: float, count: int, accepted: bool,
) -> None:
    if accepted:
        signal_client.validate_confirmation_schedule(interval, count)
    else:
        with pytest.raises(signal_client.AcceptanceSignalError):
            signal_client.validate_confirmation_schedule(interval, count)


def test_infinite_heartbeat_dry_run_previews_two_strict_payloads() -> None:
    request = signal_client.SignalRequest(
        kind="confirmation",
        target_gravity_scale=1.0,
        heartbeat_interval_seconds=10.0,
        repeat_count=0,
    )
    document = signal_client.dry_run_signal(request, binding())
    payloads = document["preview_payloads"]

    assert document["planned_publication_count"] == "UNTIL_INTERRUPTED"
    assert len(payloads) == 2
    assert payloads[1]["source_instance_id"] == payloads[0]["source_instance_id"]
    assert payloads[1]["sequence"] > payloads[0]["sequence"]
    assert (
        payloads[1]["source_monotonic_ns"]
        > payloads[0]["source_monotonic_ns"]
    )


def test_periodic_cli_defaults_to_until_interrupted() -> None:
    args = signal_client.build_parser().parse_args([
        "confirmation",
        "--target-gravity-scale", "0.5",
        "--heartbeat-interval-seconds", "10",
    ])
    request = signal_client.request_from_args(args)
    assert request.heartbeat_interval_seconds == 10.0
    assert request.repeat_count == 0


def test_live_preconditions_are_runner_identical_and_fail_before_ros_import() -> None:
    bound = binding()
    environment = {
        runner.PHYSICAL_CONFIRMATION_ENV: runner.PHYSICAL_CONFIRMATION_GATE
    }
    signal_client.validate_live_preconditions(
        binding=bound,
        enable_live=runner.LIVE_ENABLE_TOKEN,
        environment=environment,
    )
    with pytest.raises(
        signal_client.AcceptanceSignalError, match="LIVE_ENABLE_TOKEN_MISSING"
    ):
        signal_client.validate_live_preconditions(
            binding=bound,
            enable_live="",
            environment=environment,
        )
    with pytest.raises(
        runner.AcceptanceError,
        match="PHYSICAL_STARTUP_CONFIRMATION_MISSING_OR_MISMATCHED",
    ):
        signal_client.validate_live_preconditions(
            binding=bound,
            enable_live=runner.LIVE_ENABLE_TOKEN,
            environment={},
        )
    with pytest.raises(runner.AcceptanceError, match="EMPIRICAL_ENVELOPE_EXPIRED"):
        signal_client.validate_live_preconditions(
            binding=replace(
                bound,
                expires_at_utc=datetime.now(timezone.utc) - timedelta(seconds=1),
            ),
            enable_live=runner.LIVE_ENABLE_TOKEN,
            environment=environment,
        )

    source = SCRIPT.read_text(encoding="utf-8")
    live = source[source.index("def run_live(") : source.index("def build_parser(")]
    assert live.index("validate_live_preconditions(") < live.index("import rclpy")


class FakeMonotonicClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


class FakePublisher:
    def __init__(
        self,
        subscription_counts,
        *,
        acknowledgement=True,
    ) -> None:
        self._subscription_counts = iter(subscription_counts)
        self._last_subscription_count = 0
        self.acknowledgement = acknowledgement
        self.published = []
        self.acknowledgement_timeouts = []

    def get_subscription_count(self) -> int:
        try:
            self._last_subscription_count = next(self._subscription_counts)
        except StopIteration:
            pass
        return self._last_subscription_count

    def publish(self, message) -> None:
        self.published.append(message)

    def wait_for_all_acked(self, *, timeout) -> bool:
        self.acknowledgement_timeouts.append(timeout)
        return self.acknowledgement


def test_wait_for_subscription_spins_until_discovery_match() -> None:
    clock = FakeMonotonicClock()
    publisher = FakePublisher([0, 0, 1])
    spin_timeouts = []

    def fake_spin(_node, timeout: float) -> None:
        spin_timeouts.append(timeout)
        clock.advance(timeout)

    signal_client.wait_for_subscription(
        node=object(),
        publisher=publisher,
        spin_once=fake_spin,
        timeout_seconds=1.0,
        monotonic=clock,
    )

    assert spin_timeouts == [signal_client.ROS_SPIN_SLICE_SECONDS] * 2
    assert publisher.published == []


def test_discovery_timeout_is_fail_closed_and_never_publishes() -> None:
    clock = FakeMonotonicClock()
    publisher = FakePublisher([0])
    spin_timeouts = []

    def fake_spin(_node, timeout: float) -> None:
        spin_timeouts.append(timeout)
        clock.advance(timeout)

    with pytest.raises(
        signal_client.AcceptanceSignalError,
        match="ROS_SUBSCRIBER_DISCOVERY_TIMEOUT",
    ):
        signal_client.wait_for_subscription(
            node=object(),
            publisher=publisher,
            spin_once=fake_spin,
            timeout_seconds=0.1,
            monotonic=clock,
        )

    assert sum(spin_timeouts) == pytest.approx(0.1)
    assert publisher.published == []


def test_publish_payload_once_acks_without_repeating_payload() -> None:
    publisher = FakePublisher([1], acknowledgement=True)
    message = object()
    acknowledgement_timeout = object()
    spins = []

    signal_client.publish_payload_once(
        node="node",
        publisher=publisher,
        message=message,
        spin_once=lambda node, timeout: spins.append((node, timeout)),
        acknowledgement_timeout=acknowledgement_timeout,
    )

    assert publisher.published == [message]
    assert publisher.acknowledgement_timeouts == [acknowledgement_timeout]
    assert spins == [
        ("node", signal_client.ROS_SPIN_SLICE_SECONDS),
        ("node", signal_client.ROS_SPIN_SLICE_SECONDS),
    ]


def test_publication_ack_timeout_fails_after_exactly_one_publish() -> None:
    publisher = FakePublisher([1], acknowledgement=False)
    message = object()

    with pytest.raises(
        signal_client.AcceptanceSignalError,
        match="ROS_PUBLICATION_ACK_TIMEOUT",
    ):
        signal_client.publish_payload_once(
            node=object(),
            publisher=publisher,
            message=message,
            spin_once=lambda _node, _timeout: None,
            acknowledgement_timeout=object(),
        )

    assert publisher.published == [message]
    assert len(publisher.acknowledgement_timeouts) == 1


def test_source_has_one_bounded_publisher_and_no_hardware_or_gui_path() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported_roots = {
        alias.name.split(".", 1)[0]
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert not imported_roots & {"socket", "serial", "can", "dmcan"}
    assert "/whole_arm/gui_command" not in source
    assert all(str(port) not in source for port in (15310, 15311, 15312, 15313))
    assert source.count("create_publisher(") == 1
    assert source.count("publisher.publish(") == 1
    assert set(signal_client.TOPIC_BY_SIGNAL.values()) == {
        "/whole_arm/empirical_stage_confirmation",
        "/whole_arm/v15_31b/acceptance_control",
    }


def test_dry_run_control_cli_never_requires_ros(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        signal_client,
        "load_evidence_binding",
        lambda _args: binding(),
    )
    result = signal_client.main([
        "--envelope", "unused-envelope.json",
        "--anchor-validation", "unused-anchor.json",
        "--expected-envelope-sha256", "a" * 64,
        "control",
        "--action", "START_THERMAL_STAGE",
        "--extra-json", '{"duration_min":5,"target_j2_rad":0.125}',
    ])
    document = json.loads(capsys.readouterr().out)

    assert result == 0
    assert document["mode"] == "OFFLINE_DRY_RUN"
    assert document["publishes_ros"] is False
    assert document["topic"] == signal_client.CONTROL_TOPIC
    assert document["preview_payloads"][0]["action"] == "START_THERMAL_STAGE"
    assert document["preview_payloads"][0]["duration_min"] == 5
