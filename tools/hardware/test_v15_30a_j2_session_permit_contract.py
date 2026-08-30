#!/usr/bin/env python3
"""Static fail-closed contract for the J2 one-use startup permit."""

from __future__ import annotations

import copy
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONTROLLER = ROOT / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
ISSUER = (
    ROOT
    / "tools"
    / "hardware"
    / "v15_30a_create_j2_vertical_session_phase_anchor.py"
)
SUPERVISOR = ROOT / "start_arm_gui.sh"


def launch_gate_evidence_validator():
    """Execute the exact evidence-validation fragment embedded in the supervisor."""
    source = SUPERVISOR.read_text(encoding="utf-8")
    start_marker = "    safety = capture_document.get(\"safety\")\n"
    end_marker = (
        "    if (\n"
        "        raw_worker_path != str(worker_path)\n"
        "        or normalized_worker_path != str(worker_path)\n"
    )
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    fragment = source[start:end]

    def normalized_sha256(value, label):
        if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
            raise ValueError(f"{label} SHA-256 is malformed")
        return value

    def exact_integer(value, label):
        if type(value) is not int:
            raise ValueError(f"{label} must be an integer")
        return value

    def canonical_count(mapping, field, *, positive=True):
        value = mapping.get(field)
        if not isinstance(value, str) or re.fullmatch(r"0|[1-9][0-9]*", value) is None:
            raise ValueError(f"worker terminal {field} must be canonical decimal")
        parsed = int(value, 10)
        if (positive and parsed <= 0) or (not positive and parsed < 0):
            raise ValueError(f"worker terminal {field} is outside its count domain")
        return parsed

    namespace = {
        "math": math,
        "normalized_sha256": normalized_sha256,
        "exact_integer": exact_integer,
        "canonical_count": canonical_count,
    }
    exec(
        "def validate(capture_document, raw_capture, packet_count):\n" + fragment,
        namespace,
    )
    return namespace["validate"]


def launch_gate_evidence_fixture() -> tuple[dict, dict, int]:
    packet_count = 500
    invalid_prefix_pairs = 3
    healthy_pairs = 5
    attempted_pairs = invalid_prefix_pairs + healthy_pairs
    prime_tx = 2 * attempted_pairs
    aggregate_tx = prime_tx + 2 * packet_count + 40
    worker_sha256 = "f" * 64
    startup_prime = {
        "state": "PASS",
        "maximum_invalid_prefix_pairs": 3,
        "invalid_prefix_pairs": invalid_prefix_pairs,
        "required_healthy_pairs": 5,
        "healthy_pairs": healthy_pairs,
        "attempted_pairs": attempted_pairs,
        "tx_attempt_count": prime_tx,
        "elapsed_ns": 180_000_000,
        "feedback_published_during_prime": False,
        "window_reopens_after_first_healthy_pair": False,
    }
    phase_tx_accounting = {
        "completed_cycles": packet_count,
        "startup_prime_tx_attempt_count": prime_tx,
        "control_loop_tx_attempt_count": 2 * packet_count,
        "final_brake_tx_attempt_count": 40,
        "aggregate_tx_attempt_count": aggregate_tx,
    }
    terminal_proof = {
        "TERMINAL_PATH": "NORMAL",
        "GUI_GO_CONTROLLER_BUS": "j2",
        "EXECUTION_POLICY": "BRAKE_ONLY",
        "COMMAND_RX_ENABLED": "NO",
        "J2_SESSION_REFERENCE_CONFIGURED": "NO",
        "FOC_TX_ATTEMPT_COUNT": "0",
        "OTHER_MODE_TX_ATTEMPT_COUNT": "0",
        "BRAKE_ONLY_GUARD_BLOCK_COUNT": "0",
        "FOC_SERIAL_SEND_CALL_COUNT": "0",
        "BRAKE_ONLY_AUDIT": "PASS",
        "FINAL_MODE": "BRAKE",
        "FINAL_BRAKE": "PASS",
        "MOTOR_INTERNAL_ZERO_WRITE": "NO",
        "J2_STARTUP_BRAKE_PRIME_STATE": "PASS",
        "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS": "3",
        "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS": "5",
        "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS": str(invalid_prefix_pairs),
        "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS": str(healthy_pairs),
        "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS": str(attempted_pairs),
        "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT": str(prime_tx),
        "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS": "180000000",
        "COMPLETED_CYCLES": str(packet_count),
        "FINAL_BRAKE_TX_ATTEMPT_COUNT": "40",
        "TX_ATTEMPT_TOTAL": str(aggregate_tx),
        "BRAKE_TX_ATTEMPT_COUNT": str(aggregate_tx),
        "SERIAL_SEND_CALL_COUNT": str(aggregate_tx),
    }
    safety = {
        "execution_policy": "BRAKE_ONLY",
        "all_controller_modes": ["brake"],
        "command_rx_enabled": False,
        "j2_session_reference_configured": False,
        "foc_tx_attempt_count": 0,
        "foc_serial_send_call_count": 0,
        "other_mode_tx_attempt_count": 0,
        "active_or_hold_commands_sent": 0,
        "active_commands_sent": 0,
        "hold_commands_sent": 0,
        "communication_failure_packets": 0,
        "merror_nonzero_packets": 0,
        "motor_internal_zero_modified": False,
        "motor_internal_zero_write_count": 0,
        "rid_written": False,
        "rid_write_count": 0,
        "flash_or_eeprom_written": False,
        "flash_or_eeprom_write_count": 0,
        "startup_brake_prime_passed": True,
        "startup_invalid_prefix_pairs": invalid_prefix_pairs,
    }
    raw_worker = {
        "path": "/opt/go-m8010/v15_30a_gui_go_controller",
        "sha256": worker_sha256,
        "returncode": 0,
        "stop_escalation": "none",
        "forced_kill": False,
        "terminal_proof": terminal_proof,
    }
    capture_document = {
        "safety": safety,
        "physical_confirmation": {
            "confirmation_gate": (
                "24V_ON=YES;SUPPORT_RELIABLE=YES;"
                "J2_VERTICAL_INITIALIZATION_POSE=YES;ARM_NOT_MOVED=YES;"
                "ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES"
            ),
            "support_reliable": True,
            "j2_vertical_initialization_pose": True,
            "arm_not_moved": True,
            "arm_stationary": True,
            "not_at_mechanical_limit": True,
        },
        "raw_safety_summary": {
            "branch_inputs_loaded": False,
            "persistent_zero_loaded": False,
            "recovery_hint_loaded": False,
            "integer_turn_branch_inferred": False,
            "udp_send_calls": 0,
            "startup_brake_prime": startup_prime,
            "phase_tx_accounting": phase_tx_accounting,
            "worker": raw_worker,
        },
    }
    raw_capture = {
        "startup_brake_prime": copy.deepcopy(startup_prime),
        "phase_tx_accounting": copy.deepcopy(phase_tx_accounting),
        "worker": {
            "path": raw_worker["path"],
            "sha256": worker_sha256,
            "terminal_proof": copy.deepcopy(terminal_proof),
        },
    }
    return capture_document, raw_capture, packet_count


class J2SessionPermitContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.controller = CONTROLLER.read_text(encoding="utf-8")
        cls.issuer = ISSUER.read_text(encoding="utf-8")
        cls.supervisor = SUPERVISOR.read_text(encoding="utf-8")

    def test_launch_gate_rejects_missing_tampered_or_extended_normalized_evidence(
        self,
    ) -> None:
        validate = launch_gate_evidence_validator()

        baseline_capture, baseline_normalized, packet_count = (
            launch_gate_evidence_fixture()
        )
        validate(baseline_capture, baseline_normalized, packet_count)

        def remove_structure(document, field):
            document.pop(field)

        def remove_worker_terminal(document, _field):
            document["worker"].pop("terminal_proof")

        def remove_nested_field(document, field):
            document[field].pop(next(iter(document[field])))

        def remove_terminal_field(document, _field):
            document["worker"]["terminal_proof"].pop("FINAL_BRAKE")

        def tamper_prime(document, field):
            document[field]["invalid_prefix_pairs"] = 2

        def tamper_phase(document, field):
            document[field]["aggregate_tx_attempt_count"] += 1

        def tamper_terminal(document, _field):
            document["worker"]["terminal_proof"]["FINAL_BRAKE"] = "FAIL"

        def extend_nested(document, field):
            document[field]["unexpected_normalized_field"] = 1

        def extend_terminal(document, _field):
            document["worker"]["terminal_proof"]["UNEXPECTED_NORMALIZED_FIELD"] = "1"

        cases = (
            ("startup prime structure missing", "startup_brake_prime", remove_structure),
            ("startup prime field missing", "startup_brake_prime", remove_nested_field),
            ("startup prime tampered", "startup_brake_prime", tamper_prime),
            ("startup prime extra field", "startup_brake_prime", extend_nested),
            (
                "phase accounting structure missing",
                "phase_tx_accounting",
                remove_structure,
            ),
            (
                "phase accounting field missing",
                "phase_tx_accounting",
                remove_nested_field,
            ),
            ("phase accounting tampered", "phase_tx_accounting", tamper_phase),
            ("phase accounting extra field", "phase_tx_accounting", extend_nested),
            ("terminal proof structure missing", "terminal_proof", remove_worker_terminal),
            ("terminal proof field missing", "terminal_proof", remove_terminal_field),
            ("terminal proof tampered", "terminal_proof", tamper_terminal),
            ("terminal proof extra field", "terminal_proof", extend_terminal),
        )
        for label, field, mutate in cases:
            with self.subTest(label=label):
                capture = copy.deepcopy(baseline_capture)
                normalized = copy.deepcopy(baseline_normalized)
                mutate(normalized, field)
                with self.assertRaises(ValueError):
                    validate(capture, normalized, packet_count)

    def test_launch_gate_rejects_python_equal_but_wrong_json_types(self) -> None:
        validate = launch_gate_evidence_validator()
        baseline_capture, baseline_normalized, packet_count = (
            launch_gate_evidence_fixture()
        )

        def prime_integer_as_float(capture, normalized):
            normalized["startup_brake_prime"]["maximum_invalid_prefix_pairs"] = 3.0

        def prime_boolean_as_integer(capture, normalized):
            normalized["startup_brake_prime"][
                "feedback_published_during_prime"
            ] = 0

        def phase_integer_as_float(capture, normalized):
            normalized["phase_tx_accounting"]["completed_cycles"] = 500.0

        def worker_returncode_as_boolean(capture, normalized):
            capture["raw_safety_summary"]["worker"]["returncode"] = False

        def udp_send_count_as_boolean(capture, normalized):
            capture["raw_safety_summary"]["udp_send_calls"] = False

        cases = (
            ("normalized prime integer as float", prime_integer_as_float),
            ("normalized prime boolean as integer", prime_boolean_as_integer),
            ("normalized phase integer as float", phase_integer_as_float),
            ("raw worker returncode as boolean", worker_returncode_as_boolean),
            ("raw UDP count as boolean", udp_send_count_as_boolean),
        )
        for label, mutate in cases:
            with self.subTest(label=label):
                capture = copy.deepcopy(baseline_capture)
                normalized = copy.deepcopy(baseline_normalized)
                mutate(capture, normalized)
                with self.assertRaises(ValueError):
                    validate(capture, normalized, packet_count)

    def test_normal_j2_requires_all_session_and_permit_cli_bindings(self) -> None:
        for option in (
            "--j2-session-reference-file",
            "--j2-session-launch-permit-file",
            "--expected-zero-sha256",
            "--expected-j2-session-reference-sha256",
            "--expected-j2-power-session-id",
            "--expected-worker-sha256",
        ):
            self.assertIn(option, self.controller)
        self.assertIn("J2_SESSION_REFERENCE_GATE_MISSING", self.controller)
        self.assertIn("options.bus == \"j2\" && !options.brake_only", self.controller)

    def test_permit_is_consumed_before_serial_and_spent_before_command_bind(self) -> None:
        run_start = self.controller.index("int run(const Options& options)")
        run = self.controller[run_start:]
        self.assertLess(
            run.index("consume_j2_launch_permit(j2_launch_permit)"),
            run.index("SerialPort serial"),
        )
        verification = run.index("J2_STARTUP_BRAKE_RECHECK_FAILED")
        spend = run.index("spend_j2_launch_permit", verification)
        bind = run.index("command_socket = open_command_socket", spend)
        verified = run.index("j2_startup_verified = true", bind)
        self.assertLess(spend, bind)
        self.assertLess(bind, verified)

    def test_recheck_stays_brake_only_and_continuity_loss_is_terminal(self) -> None:
        for evidence in (
            "J2_STARTUP_RECHECK_NON_BRAKE_MODE",
            "minimum_brake_frames",
            "max_raw_phase_delta_rad",
            "max_raw_span_rad",
            "J2_STARTUP_BRAKE_RECHECK_FAILED",
            "J2_POWER_CONTINUITY_LOST",
            "(!active_power_session || j2_startup_verified)",
        ):
            self.assertIn(evidence, self.controller)
        # One malformed reply is published as communication_ok=false but does
        # not unload the pair.  Power-session continuity becomes terminal at
        # the audited consecutive-invalid threshold.
        invalid_mark = self.controller.index(
            "if (active_power_session && sustained_invalid)"
        )
        terminal = self.controller.index('"J2_POWER_CONTINUITY_LOST"', invalid_mark)
        self.assertLess(invalid_mark, terminal)

    def test_issuer_and_consumer_share_lifecycle_and_time_bounds(self) -> None:
        for source in (self.controller, self.issuer):
            for field in (
                "go-m8010-j2-power-session-launch-permit/1.0",
                "issued_boottime_ns",
                "expires_boottime_ns",
                "source_capture",
                "pending_path",
                "inflight_path",
                "spent_path",
                "minimum_brake_frames",
                "max_raw_phase_delta_rad",
                "max_raw_span_rad",
            ):
                self.assertIn(field, source)
        self.assertIn("30000000000ULL", self.controller)
        self.assertIn("LAUNCH_PERMIT_TTL_SECONDS = 30", self.issuer)
        self.assertIn("MAX_CAPTURE_TO_PERMIT_SECONDS = 300", self.issuer)

    def test_offline_bundle_audit_cannot_enable_execution(self) -> None:
        self.assertIn("--audit-j2-session-bundle", self.controller)
        self.assertIn("J2_SESSION_BUNDLE_AUDIT=PASS", self.controller)
        self.assertIn("SERIAL_OPENED=NO", self.controller)
        self.assertIn(
            "if (options.execute || options.brake_only || options.bus != \"j2\"",
            self.controller,
        )

    def test_evidence_hashes_are_computed_from_same_secure_fd_bytes(self) -> None:
        secure_reader = self.controller[
            self.controller.index("SecureFileBytes read_secure_regular_fd") :
            self.controller.index("nlohmann::json parse_json_bytes")
        ]
        for evidence in (
            "::fstat(descriptor, &before)",
            "before.st_uid != ::geteuid()",
            "before.st_nlink != 1",
            "(before.st_mode & 07777) != 0600",
            "errno == EINTR",
            "before.st_dev != after.st_dev",
            "before.st_ino != after.st_ino",
            "before.st_mtim.tv_nsec != after.st_mtim.tv_nsec",
            "before.st_ctim.tv_nsec != after.st_ctim.tv_nsec",
            "sha256_hex(data)",
        ):
            self.assertIn(evidence, secure_reader)
        self.assertIn(
            "O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK",
            self.controller,
        )

        zero_loader = self.controller[
            self.controller.index("std::string load_persistent_zero") :
            self.controller.index("std::string load_recovery_hints")
        ]
        hint_loader = self.controller[
            self.controller.index("std::string load_recovery_hints") :
            self.controller.index("void apply_j2_session_reference_document")
        ]
        reference_loader = self.controller[
            self.controller.index("LoadedJ2SessionReference load_j2_session_reference") :
            self.controller.index("struct J2LaunchPermit")
        ]
        for loader in (zero_loader, hint_loader, reference_loader):
            self.assertIn("read_secure_owned_file", loader)
            self.assertIn("parse_json_bytes(source.data", loader)
            self.assertIn("source.sha256", loader)
        self.assertIn("actual_recovery_hint_sha256", reference_loader)

        worker_loader = self.controller[
            self.controller.index("SecureFileBytes read_running_executable") :
            self.controller.index("std::string expected_permit_directory")
        ]
        self.assertIn('::open("/proc/self/exe", O_RDONLY | O_CLOEXEC)', worker_loader)
        self.assertIn("read_secure_regular_fd", worker_loader)
        permit_parser = self.controller[
            self.controller.index("J2LaunchPermit apply_j2_launch_permit_document") :
            self.controller.index("void require_j2_launch_permit_fresh")
        ]
        for comparison in (
            "options.expected_zero_sha256 != actual_zero_sha256",
            "options.expected_j2_session_reference_sha256 != actual_reference_sha256",
            "options.expected_worker_sha256 != actual_worker_sha256",
        ):
            self.assertIn(comparison, permit_parser)
        for forbidden in ("popen(", "std::system(", "openssl"):
            self.assertNotIn(forbidden, self.controller)

    def test_internal_sha256_known_answer_tests_run_on_every_entry_path(self) -> None:
        for digest in (
            "e3b0c44298fc1c149afbf4c8996fb924",
            "ba7816bf8f01cfea414140de5dae2223",
            "b35439a4ac6f0948b6d6f9e3c6af0f5",
            "41edece42d63e8d9bf515a9ba6932e1c",
        ):
            self.assertIn(digest, self.controller)
        run = self.controller[self.controller.index("int run(const Options& options)") :]
        self.assertLess(run.index("sha256_self_test();"), run.index("if (options.audit_j2_session_bundle)"))

    def test_permit_lifecycle_pins_move_only_fds_and_inode(self) -> None:
        unique_fd = self.controller[
            self.controller.index("class UniqueFd") :
            self.controller.index("struct SecureFileBytes")
        ]
        for evidence in (
            "UniqueFd(const UniqueFd&) = delete",
            "UniqueFd& operator=(const UniqueFd&) = delete",
            "UniqueFd(UniqueFd&& other) noexcept",
            "UniqueFd& operator=(UniqueFd&& other) noexcept",
        ):
            self.assertIn(evidence, unique_fd)
        permit = self.controller[
            self.controller.index("struct J2LaunchPermit") :
            self.controller.index("std::string canonical_existing_path", self.controller.index("struct J2LaunchPermit"))
        ]
        for evidence in (
            "UniqueFd permit_fd",
            "UniqueFd pending_directory_fd",
            "UniqueFd inflight_directory_fd",
            "UniqueFd spent_directory_fd",
            "dev_t file_device",
            "ino_t file_inode",
            "std::string filename",
        ):
            self.assertIn(evidence, permit)
        for evidence in (
            "!std::is_copy_constructible<J2LaunchPermit>::value",
            "std::is_nothrow_move_constructible<J2LaunchPermit>::value",
            "std::is_nothrow_move_assignable<J2LaunchPermit>::value",
        ):
            self.assertIn(evidence, self.controller)
        self.assertIn('permit.filename != permit.permit_id + ".json"', self.controller)

    def test_transitions_use_dirfds_and_verify_same_inode_before_and_after(self) -> None:
        transition = self.controller[
            self.controller.index("void rename_permit_no_replace") :
            self.controller.index("void consume_j2_launch_permit")
        ]
        self.assertIn("require_permit_at_directory", transition)
        self.assertIn("SYS_renameat2", transition)
        self.assertIn("RENAME_NOREPLACE", transition)
        self.assertIn("AT_SYMLINK_NOFOLLOW", transition)
        self.assertIn("secure_permit_identity_matches(destination, permit)", transition)
        self.assertNotIn("AT_FDCWD", transition)
        self.assertLess(
            transition.index("require_permit_at_directory"),
            transition.index("SYS_renameat2"),
        )
        self.assertLess(
            transition.index("SYS_renameat2"),
            transition.index("secure_permit_identity_matches(destination, permit)"),
        )
        consume = self.controller[
            self.controller.index("void consume_j2_launch_permit") :
            self.controller.index("void spend_j2_launch_permit")
        ]
        spend = self.controller[
            self.controller.index("void spend_j2_launch_permit") :
            self.controller.index("double reference_for_recovery_branch")
        ]
        for body, state in ((consume, '"INFLIGHT"'), (spend, '"SPENT"')):
            self.assertLess(body.index("rename_permit_no_replace"), body.index(f"permit.state = {state}"))

    def test_freshness_is_rechecked_immediately_before_each_rename(self) -> None:
        freshness = self.controller[
            self.controller.index("void require_j2_launch_permit_fresh") :
            self.controller.index("bool secure_permit_identity_matches")
        ]
        for evidence in (
            "now_boottime_ns < permit.issued_boottime_ns",
            "now_boottime_ns >= permit.expires_boottime_ns",
            "permit.capture_boottime_ns > now_boottime_ns",
            "now_boottime_ns - permit.capture_boottime_ns >",
            "kJ2CaptureToPermitMaximumAgeNs",
        ):
            self.assertIn(evidence, freshness)
        transition = self.controller[
            self.controller.index("void rename_permit_no_replace") :
            self.controller.index("void consume_j2_launch_permit")
        ]
        freshness_call = transition.index(
            "require_j2_launch_permit_fresh(permit, current_boottime_ns())"
        )
        rename_call = transition.index("SYS_renameat2")
        post_freshness_call = transition.rindex(
            "require_j2_launch_permit_fresh(permit, current_boottime_ns())"
        )
        self.assertLess(freshness_call, rename_call)
        self.assertLess(rename_call, post_freshness_call)
        self.assertNotEqual(freshness_call, post_freshness_call)
        for function_name in (
            "void consume_j2_launch_permit",
            "void spend_j2_launch_permit",
        ):
            body_start = self.controller.index(function_name)
            body_end = self.controller.index("\n}\n", body_start)
            self.assertIn("rename_permit_no_replace", self.controller[body_start:body_end])


if __name__ == "__main__":
    unittest.main()
