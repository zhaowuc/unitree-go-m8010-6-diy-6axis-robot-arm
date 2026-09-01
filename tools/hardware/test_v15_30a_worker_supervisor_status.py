from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import threading

import pytest


ROOT = Path(__file__).resolve().parents[2]
PRODUCER_PATH = Path(__file__).with_name(
    "v15_30a_worker_supervisor_status.py"
)
STATE_MODEL_PATH = (
    ROOT
    / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
    / "ros2_ws/src/go_m8010_arm_hardware/go_m8010_arm_hardware/state_model.py"
)


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


producer_module = load_module("v15_30a_worker_supervisor_status", PRODUCER_PATH)
state_model = load_module("v15_30a_worker_supervisor_state_model", STATE_MODEL_PATH)


SUPERVISOR_PID = 1100
WORKER_PIDS = {
    "J1": 2101,
    "J2": 2102,
    "J345": 2103,
    "J6": 2104,
}


def make_identity_reader():
    identities = {
        SUPERVISOR_PID: (SUPERVISOR_PID, 10),
        **{pid: (pid, 20 + index) for index, pid in enumerate(WORKER_PIDS.values())},
    }

    def read(pid: int):
        return identities.get(pid)

    return identities, read


def make_producer():
    identities, reader = make_identity_reader()
    instance = "a" * 32
    heartbeat = producer_module.HeartbeatProducer(
        supervisor_pid=SUPERVISOR_PID,
        worker_pids=WORKER_PIDS,
        supervisor_instance_id=instance,
        process_identity_reader=reader,
    )
    return heartbeat, identities


def exact_udp_owners():
    return {
        producer_module.COMMAND_PORT_BY_DOMAIN[domain]: {pid}
        for domain, pid in WORKER_PIDS.items()
    }


def test_ready_document_exactly_matches_state_model_schema() -> None:
    heartbeat, _identities = make_producer()
    source_ns = 5_000_000_000
    document = heartbeat.next_document(
        udp_owners_by_port=exact_udp_owners(),
        source_monotonic_ns=source_ns,
    )

    validated = state_model.validate_worker_supervisor_status(
        document, source_ns + 1, 1_000_000_000
    )

    assert document["schema"] == state_model.WORKER_SUPERVISOR_STATUS_SCHEMA
    assert set(document["domains"]) == set(state_model.WORKER_CONTROL_DOMAINS)
    assert all(validated["control_available_by_domain"].values())
    assert set(validated["control_reason_by_domain"].values()) == {"ready"}
    assert validated["worker_supervisor_pid"] == SUPERVISOR_PID


def test_pid_reuse_and_nonexclusive_udp_owner_fail_closed() -> None:
    heartbeat, identities = make_producer()
    heartbeat.next_document(
        udp_owners_by_port=exact_udp_owners(),
        source_monotonic_ns=5_000_000_000,
    )
    identities[WORKER_PIDS["J2"]] = (WORKER_PIDS["J2"], 999)
    owners = exact_udp_owners()
    owners[producer_module.COMMAND_PORT_BY_DOMAIN["J6"]].add(9999)

    document = heartbeat.next_document(
        udp_owners_by_port=owners,
        source_monotonic_ns=5_100_000_000,
    )

    assert document["domains"]["J2"] == {
        "worker_pid": WORKER_PIDS["J2"],
        "command_port": 15312,
        "process_alive": False,
        "udp_owner_confirmed": False,
        "control_available": False,
        "reason": "process_exited",
    }
    assert document["domains"]["J6"]["process_alive"] is True
    assert document["domains"]["J6"]["udp_owner_confirmed"] is False
    assert document["domains"]["J6"]["control_available"] is False
    assert document["domains"]["J6"]["reason"] == "udp_owner_lost"
    validated = state_model.validate_worker_supervisor_status(
        document, 5_100_000_001, 1_000_000_000
    )
    assert validated["control_available_by_domain"]["J2"] is False
    assert validated["control_available_by_domain"]["J6"] is False


def test_missing_initial_udp_owner_is_starting_not_ready() -> None:
    heartbeat, _identities = make_producer()

    document = heartbeat.next_document(
        udp_owners_by_port={}, source_monotonic_ns=5_000_000_000
    )

    assert all(
        item["reason"] == "starting" and item["control_available"] is False
        for item in document["domains"].values()
    )
    state_model.validate_worker_supervisor_status(
        document, 5_000_000_001, 1_000_000_000
    )


def test_stopping_writes_schema_valid_all_domain_interlock() -> None:
    heartbeat, _identities = make_producer()
    writes = []
    stopped = threading.Event()
    stopped.set()

    result = producer_module.run_heartbeat(
        output=Path("worker_supervisor_status.json"),
        producer=heartbeat,
        interval_s=0.2,
        stop_event=stopped,
        udp_snapshot=lambda: exact_udp_owners(),
        writer=lambda _path, document: writes.append(document),
    )

    assert result == 0
    assert len(writes) == 1
    document = writes[0]
    assert set(item["reason"] for item in document["domains"].values()) == {
        "supervisor_stopping"
    }
    assert not any(
        item["control_available"] for item in document["domains"].values()
    )
    state_model.validate_worker_supervisor_status(
        document, document["source_monotonic_ns"] + 1, 1_000_000_000
    )


def test_atomic_writer_replaces_whole_owner_only_regular_file(tmp_path: Path) -> None:
    heartbeat, _identities = make_producer()
    output = tmp_path / producer_module.STATUS_FILENAME
    output.write_text("partial", encoding="utf-8")
    os.chmod(output, 0o666)
    document = heartbeat.next_document(
        udp_owners_by_port=exact_udp_owners(),
        source_monotonic_ns=5_000_000_000,
    )

    producer_module.atomic_write_status(output, document)

    assert json.loads(output.read_text(encoding="utf-8")) == document
    assert output.is_file() and not output.is_symlink()
    mode = stat.S_IMODE(output.stat().st_mode)
    if os.name == "posix":
        assert not mode & (stat.S_IWGRP | stat.S_IWOTH)
        assert mode == 0o600
        assert output.stat().st_uid == os.geteuid()
    assert list(tmp_path.glob(f".{producer_module.STATUS_FILENAME}.*.tmp")) == []


def test_ss_parser_tracks_exact_local_ports_and_all_owners() -> None:
    sample = "\n".join(
        (
            'UNCONN 0 0 127.0.0.1:15310 0.0.0.0:* users:(("go",pid=2101,fd=4))',
            'UNCONN 0 0 [::1]:15311 [::]:* users:(("j6",pid=2104,fd=7),("peer",pid=9999,fd=8))',
            'UNCONN 0 0 127.0.0.1:9999 0.0.0.0:* users:(("other",pid=1,fd=3))',
        )
    )

    assert producer_module.parse_ss_udp_owners(sample) == {
        15310: {2101},
        15311: {2104, 9999},
    }


def test_proc_identity_pins_starttime_and_rejects_stopped_process() -> None:
    # After ``(comm)`` the first item is kernel field 3 and item 19 is the
    # process starttime field 22.
    tail = ["S"] + ["0"] * 18 + ["987654"] + ["0"] * 3
    text = "2101 (worker name with spaces) " + " ".join(tail)
    assert producer_module.parse_process_stat_identity(2101, text) == (
        2101,
        987654,
    )
    stopped = text.replace(") S ", ") T ", 1)
    assert producer_module.parse_process_stat_identity(2101, stopped) is None
    assert producer_module.parse_process_stat_identity(2101, "truncated") is None


@pytest.mark.parametrize(
    "pins",
    (
        [],
        ["J1=2101", "J2=2102", "J345=2103"],
        ["J1=2101", "J2=2102", "J345=2103", "J6=2103"],
        ["J1=2101", "J2=2102", "J345=2103", "BAD=2104"],
        ["J1=2101", "J1=2102", "J345=2103", "J6=2104"],
    ),
)
def test_worker_pin_parser_rejects_incomplete_or_ambiguous_sets(pins) -> None:
    with pytest.raises(producer_module.SupervisorStatusError):
        producer_module.parse_worker_arguments(pins)


def test_start_script_integrates_heartbeat_with_readiness_monitor_and_cleanup() -> None:
    script = (ROOT / "start_arm_gui.sh").read_text(encoding="utf-8")
    for token in (
        'WORKER_SUPERVISOR_STATUS="$RUN_DIR/worker_supervisor_status.json"',
        'exec python3 "$WORKER_SUPERVISOR_TOOL"',
        '--supervisor-pid "$$"',
        '--interval-ms 200',
        '--worker "J1=${WORKER_PID_BY_DOMAIN[J1]}"',
        '--worker "J2=${WORKER_PID_BY_DOMAIN[J2]}"',
        '--worker "J345=${WORKER_PID_BY_DOMAIN[J345]}"',
        '--worker "J6=${WORKER_PID_BY_DOMAIN[J6]}"',
        'stat -c \'%a\' "$WORKER_SUPERVISOR_STATUS"',
        'EXPECTED_WORKER_SUPERVISOR_PID="$$"',
        'set(control_available) == EXPECTED_CONTROL_DOMAINS',
        'all(control_reasons.get(domain) == "ready"',
        'J6_FEEDBACK_SESSION_ID="$reference_session_id"',
        'J6_FEEDBACK_STATE_INSTANCE_ID="$(\n'
        "      python3 -c 'import secrets; print(secrets.token_hex(16))'",
        'J6_FEEDBACK_SESSION_ID="$EXPECTED_GRAVITY_SESSION_ID"',
        'J6_FEEDBACK_STATE_INSTANCE_ID="$EXPECTED_GRAVITY_STATE_INSTANCE_ID"',
        'state_instance_id:="$J6_FEEDBACK_STATE_INSTANCE_ID"',
        '--feedback-session-id "$J6_FEEDBACK_SESSION_ID"',
        '--feedback-state-instance-id "$J6_FEEDBACK_STATE_INSTANCE_ID"',
        '--feedback-handoff-file "$J6_FEEDBACK_HANDOFF"',
        'stop_worker_supervisor_status',
        'worker supervisor 心跳生产器已退出，立即进入制动清理',
    ):
        assert token in script
    workers_ready = script.index(
        "wait_workers_bounded\nstart_worker_supervisor_status"
    )
    readiness_probe = script.index(
        "正在检查七路反馈与签名参考哈希", workers_ready
    )
    assert workers_ready < readiness_probe
    j2_sha_pin = script.index(
        'J2_SESSION_REFERENCE_SHA256="${J2_GATE_FIELDS[1]}"'
    )
    go_aux_sha_pin = script.index(
        'GO_AUX_SESSION_REFERENCE_SHA256="${GO_AUX_GATE_FIELDS[1]}"'
    )
    feedback_identity_freeze = script.index(
        'reference_session_id="persistent:${PERSISTENT_ZERO_SHA256:0:16}"'
    )
    ros_launch = script.index("exec ros2 launch")
    assert max(j2_sha_pin, go_aux_sha_pin) < feedback_identity_freeze < ros_launch
    cleanup_start = script.index("cleanup() {")
    cleanup_end = script.index(
        '\n}\n\nif [[ "$PREBUILD_ONLY" -eq 1', cleanup_start
    )
    cleanup = script[cleanup_start:cleanup_end]
    assert cleanup.index("stop_worker_supervisor_status") < cleanup.index(
        'kill -TERM "$pid"'
    )


def test_reused_core_does_not_fail_its_own_udp_listener_preflight() -> None:
    script = (ROOT / "start_arm_gui.sh").read_text(encoding="utf-8")
    assert (
        'if [[ "$REUSE_RUNNING_ARM_GUI_CORE" != "true" ]] '
        '&& udp_port_in_use 15300; then'
    ) in script
    assert 'MONITORED_CORE_PID="$EXTERNAL_ARM_GUI_CORE_PID"' in script
    assert 'while kill -0 "$MONITORED_CORE_PID"' in script
