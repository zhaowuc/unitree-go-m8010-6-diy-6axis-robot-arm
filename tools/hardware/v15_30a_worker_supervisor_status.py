#!/usr/bin/env python3
"""Publish the owner-only worker-supervisor heartbeat consumed by state_model.

This helper never opens a motor, CAN, serial, or worker UDP socket.  It is a
read-only local supervisor witness: the parent ``start_arm_gui.sh`` PID and all
four worker PIDs are pinned before the loop starts, UDP ownership is observed
through ``ss``, and one atomic JSON heartbeat is written every cycle.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import re
import secrets
import signal
import stat
import subprocess
import tempfile
import threading
import time
from typing import Callable, Mapping, Optional, Sequence


STATUS_SCHEMA = "go-m8010-worker-supervisor-status/1.0"
STATUS_FILENAME = "worker_supervisor_status.json"
DOMAINS = ("J1", "J2", "J345", "J6")
COMMAND_PORT_BY_DOMAIN = {
    "J1": 15310,
    "J2": 15312,
    "J345": 15313,
    "J6": 15311,
}
MAX_SEQUENCE = (1 << 63) - 1
MINIMUM_INTERVAL_MS = 50
MAXIMUM_INTERVAL_MS = 500
DEFAULT_INTERVAL_MS = 200
_PID_PATTERN = re.compile(r"(?:^|[,\s(])pid=(\d+)(?=[,\s)])")


class SupervisorStatusError(RuntimeError):
    """Fail-closed local producer configuration/runtime error."""


def parse_worker_arguments(values: Sequence[str]) -> dict[str, int]:
    """Parse exactly one ``DOMAIN=PID`` pin for every command domain."""

    workers: dict[str, int] = {}
    for raw in values:
        if not isinstance(raw, str) or raw.count("=") != 1:
            raise SupervisorStatusError("worker pin must be DOMAIN=PID")
        domain, pid_text = raw.split("=", 1)
        if domain not in COMMAND_PORT_BY_DOMAIN or domain in workers:
            raise SupervisorStatusError("worker domains must be exact and unique")
        try:
            pid = int(pid_text, 10)
        except ValueError as exc:
            raise SupervisorStatusError("worker PID must be an integer") from exc
        if pid <= 1:
            raise SupervisorStatusError("worker PID must be greater than one")
        workers[domain] = pid
    if set(workers) != set(DOMAINS):
        raise SupervisorStatusError("all four worker domains are required")
    if len(set(workers.values())) != len(DOMAINS):
        raise SupervisorStatusError("worker PIDs must be unique")
    return workers


def parse_process_stat_identity(pid: int, text: str) -> Optional[tuple[int, int]]:
    """Parse one Linux ``/proc/PID/stat`` identity without accepting stops."""

    if type(pid) is not int or pid <= 1:
        return None
    if not isinstance(text, str):
        return None
    close = text.rfind(")")
    if close < 0:
        return None
    fields = text[close + 1 :].strip().split()
    # fields[0] is kernel field 3 (state), fields[19] is field 22
    # (starttime).  Pinning starttime prevents a recycled PID from inheriting
    # command authority. A stopped/traced process cannot service its socket.
    if len(fields) <= 19 or fields[0] in {"Z", "X", "x", "T", "t"}:
        return None
    try:
        start_ticks = int(fields[19], 10)
    except ValueError:
        return None
    if start_ticks <= 0:
        return None
    return pid, start_ticks


def read_process_identity(pid: int) -> Optional[tuple[int, int]]:
    """Return ``(pid, /proc starttime ticks)`` for a live runnable process."""

    if type(pid) is not int or pid <= 1:
        return None
    try:
        text = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except (FileNotFoundError, PermissionError, OSError, UnicodeError):
        return None
    return parse_process_stat_identity(pid, text)


def parse_ss_udp_owners(text: str) -> dict[int, set[int]]:
    """Parse one ``ss -H -lunp`` snapshot into local-port owner PID sets."""

    owners: dict[int, set[int]] = {}
    expected_ports = set(COMMAND_PORT_BY_DOMAIN.values())
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        local_address = fields[3]
        port_text = local_address.rsplit(":", 1)[-1]
        try:
            port = int(port_text, 10)
        except ValueError:
            continue
        if port not in expected_ports:
            continue
        pids = {int(match) for match in _PID_PATTERN.findall(line)}
        if pids:
            owners.setdefault(port, set()).update(pids)
    return owners


def snapshot_udp_owners() -> dict[int, set[int]]:
    """Take one bounded, non-mutating UDP owner snapshot with ``ss``."""

    try:
        result = subprocess.run(
            ["ss", "-H", "-l", "-u", "-n", "-p"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=0.5,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        return {}
    if result.returncode != 0:
        return {}
    return parse_ss_udp_owners(result.stdout)


def build_status_document(
    *,
    supervisor_instance_id: str,
    supervisor_pid: int,
    sequence: int,
    source_monotonic_ns: int,
    worker_pids: Mapping[str, int],
    process_alive_by_domain: Mapping[str, bool],
    udp_owners_by_port: Mapping[int, set[int]],
    previously_ready: set[str],
    stopping: bool = False,
) -> dict:
    """Build one schema-exact heartbeat and update ``previously_ready``."""

    if (
        not isinstance(supervisor_instance_id, str)
        or re.fullmatch(r"[0-9a-f]{32}", supervisor_instance_id) is None
        or type(supervisor_pid) is not int
        or supervisor_pid <= 1
        or type(sequence) is not int
        or not 1 <= sequence <= MAX_SEQUENCE
        or type(source_monotonic_ns) is not int
        or source_monotonic_ns <= 0
        or set(worker_pids) != set(DOMAINS)
        or set(process_alive_by_domain) != set(DOMAINS)
    ):
        raise SupervisorStatusError("invalid heartbeat inputs")

    domains = {}
    for domain in DOMAINS:
        worker_pid = worker_pids[domain]
        port = COMMAND_PORT_BY_DOMAIN[domain]
        alive = process_alive_by_domain[domain] is True
        owner_pids = udp_owners_by_port.get(port, set())
        # Exact ownership rejects an unexpected SO_REUSEPORT peer instead of
        # treating mere presence of the intended PID as command availability.
        owns_port = alive and owner_pids == {worker_pid}
        if stopping:
            available = False
            reason = "supervisor_stopping"
        elif not alive:
            available = False
            owns_port = False
            reason = "process_exited"
        elif not owns_port:
            available = False
            reason = "udp_owner_lost" if domain in previously_ready else "starting"
        else:
            available = True
            reason = "ready"
            previously_ready.add(domain)
        domains[domain] = {
            "worker_pid": worker_pid,
            "command_port": port,
            "process_alive": alive,
            "udp_owner_confirmed": owns_port,
            "control_available": available,
            "reason": reason,
        }
    return {
        "schema": STATUS_SCHEMA,
        "supervisor_instance_id": supervisor_instance_id,
        "supervisor_pid": supervisor_pid,
        "sequence": sequence,
        "source_monotonic_ns": source_monotonic_ns,
        "domains": domains,
    }


def atomic_write_status(path: Path, document: Mapping[str, object]) -> None:
    """Atomically replace one owner-only regular heartbeat file."""

    output = Path(path)
    if output.name != STATUS_FILENAME:
        raise SupervisorStatusError("heartbeat output filename is invalid")
    parent = output.parent
    if not parent.is_dir() or parent.is_symlink():
        raise SupervisorStatusError("heartbeat output directory is invalid")
    payload = (
        json.dumps(document, ensure_ascii=True, sort_keys=True, separators=(",", ":"))
        + "\n"
    ).encode("utf-8")
    if not payload or len(payload) > 16_384:
        raise SupervisorStatusError("heartbeat payload size is invalid")

    descriptor, temporary_text = tempfile.mkstemp(
        prefix=f".{STATUS_FILENAME}.", suffix=".tmp", dir=str(parent)
    )
    temporary = Path(temporary_text)
    try:
        if hasattr(os, "fchmod"):
            os.fchmod(descriptor, stat.S_IRUSR | stat.S_IWUSR)
        else:  # pragma: no cover - production is Linux; keeps offline NT tests useful
            os.chmod(temporary, stat.S_IRUSR | stat.S_IWUSR)
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            descriptor = -1
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
        if os.name == "posix":
            directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
            directory_fd = os.open(parent, directory_flags)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    except Exception:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
        raise


class HeartbeatProducer:
    """Stateful schema producer with PID-starttime and readiness pinning."""

    def __init__(
        self,
        *,
        supervisor_pid: int,
        worker_pids: Mapping[str, int],
        supervisor_instance_id: Optional[str] = None,
        process_identity_reader: Callable[[int], Optional[tuple[int, int]]] = (
            read_process_identity
        ),
    ) -> None:
        if type(supervisor_pid) is not int or supervisor_pid <= 1:
            raise SupervisorStatusError("supervisor PID must be greater than one")
        if set(worker_pids) != set(DOMAINS):
            raise SupervisorStatusError("all four worker domains are required")
        if any(type(pid) is not int or pid <= 1 for pid in worker_pids.values()):
            raise SupervisorStatusError("worker PIDs must be integers greater than one")
        if len(set(worker_pids.values())) != len(DOMAINS):
            raise SupervisorStatusError("worker PIDs must be unique")
        if supervisor_pid in set(worker_pids.values()):
            raise SupervisorStatusError("supervisor and worker PIDs must differ")
        self.supervisor_pid = supervisor_pid
        self.worker_pids = dict(worker_pids)
        self.supervisor_instance_id = (
            secrets.token_hex(16)
            if supervisor_instance_id is None
            else supervisor_instance_id
        )
        if re.fullmatch(r"[0-9a-f]{32}", self.supervisor_instance_id) is None:
            raise SupervisorStatusError("supervisor instance ID is invalid")
        self._identity_reader = process_identity_reader
        self._supervisor_identity = process_identity_reader(supervisor_pid)
        if self._supervisor_identity is None:
            raise SupervisorStatusError("supervisor process identity is unavailable")
        self._worker_identities = {
            domain: process_identity_reader(pid)
            for domain, pid in self.worker_pids.items()
        }
        self.previously_ready: set[str] = set()
        self.sequence = 0

    def supervisor_alive(self) -> bool:
        return self._identity_reader(self.supervisor_pid) == self._supervisor_identity

    def next_document(
        self,
        *,
        udp_owners_by_port: Mapping[int, set[int]],
        source_monotonic_ns: Optional[int] = None,
        stopping: bool = False,
    ) -> dict:
        if self.sequence >= MAX_SEQUENCE:
            raise SupervisorStatusError("heartbeat sequence exhausted")
        self.sequence += 1
        now_ns = time.monotonic_ns() if source_monotonic_ns is None else source_monotonic_ns
        alive = {
            domain: (
                identity is not None
                and self._identity_reader(self.worker_pids[domain]) == identity
            )
            for domain, identity in self._worker_identities.items()
        }
        return build_status_document(
            supervisor_instance_id=self.supervisor_instance_id,
            supervisor_pid=self.supervisor_pid,
            sequence=self.sequence,
            source_monotonic_ns=now_ns,
            worker_pids=self.worker_pids,
            process_alive_by_domain=alive,
            udp_owners_by_port=udp_owners_by_port,
            previously_ready=self.previously_ready,
            stopping=stopping,
        )


def install_parent_death_signal(expected_parent_pid: int) -> None:
    """Request SIGTERM if the Bash supervisor exits, then close the race."""

    if os.name != "posix" or not Path("/proc/self/stat").is_file():
        raise SupervisorStatusError("worker heartbeat requires Linux /proc")
    if os.getppid() != expected_parent_pid:
        raise SupervisorStatusError("heartbeat parent PID mismatch")
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise SupervisorStatusError(f"PR_SET_PDEATHSIG failed: errno={error}")
    if os.getppid() != expected_parent_pid:
        raise SupervisorStatusError("heartbeat parent exited during startup")


def run_heartbeat(
    *,
    output: Path,
    producer: HeartbeatProducer,
    interval_s: float,
    stop_event: threading.Event,
    udp_snapshot: Callable[[], Mapping[int, set[int]]] = snapshot_udp_owners,
    writer: Callable[[Path, Mapping[str, object]], None] = atomic_write_status,
) -> int:
    """Run until signalled or parent identity loss; always try a final interlock."""

    if not 0.05 <= interval_s <= 0.5:
        raise SupervisorStatusError("heartbeat interval is outside the safe range")
    while not stop_event.is_set() and producer.supervisor_alive():
        writer(
            output,
            producer.next_document(udp_owners_by_port=udp_snapshot()),
        )
        stop_event.wait(interval_s)
    try:
        writer(
            output,
            producer.next_document(
                udp_owners_by_port={}, stopping=True
            ),
        )
    except Exception:
        # If the final write itself fails, the last heartbeat becomes stale in
        # at most the state node's one-second window and remains fail-closed.
        return 2
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--supervisor-pid", type=int, required=True)
    parser.add_argument("--worker", action="append", default=[], metavar="DOMAIN=PID")
    parser.add_argument("--interval-ms", type=int, default=DEFAULT_INTERVAL_MS)
    args = parser.parse_args(argv)
    if not MINIMUM_INTERVAL_MS <= args.interval_ms <= MAXIMUM_INTERVAL_MS:
        raise SupervisorStatusError("heartbeat interval is outside the safe range")
    workers = parse_worker_arguments(args.worker)
    install_parent_death_signal(args.supervisor_pid)
    producer = HeartbeatProducer(
        supervisor_pid=args.supervisor_pid,
        worker_pids=workers,
    )
    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGHUP, request_stop)
    return run_heartbeat(
        output=args.output,
        producer=producer,
        interval_s=args.interval_ms / 1000.0,
        stop_event=stop_event,
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SupervisorStatusError as exc:
        print(f"WORKER_SUPERVISOR_STATUS_ERROR={exc}", file=os.sys.stderr)
        raise SystemExit(2) from exc
