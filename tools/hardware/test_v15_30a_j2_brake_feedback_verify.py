import importlib.util
import hashlib
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).with_name("v15_30a_j2_brake_feedback_verify.py")
SPEC = importlib.util.spec_from_file_location("j2_brake_verify", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def make_packet(index, *, q_a=0.001, q_b=-0.001):
    persistent_a = -4.596924286444928
    persistent_b = 12.42541044895233
    ref_a = persistent_a + 2.0 * math.pi
    ref_b = persistent_b - 2.0 * math.pi
    wobble = 1.0e-5 * math.sin(index / 20.0)
    values = {
        "J2A": (ref_a - MODULE.GEAR_RATIO * (q_a + wobble), ref_a),
        "J2B": (ref_b + MODULE.GEAR_RATIO * (q_b + wobble), ref_b),
    }
    samples = []
    for name in ("J2A", "J2B"):
        unwrapped, reference = values[name]
        samples.append({
            "motor": name,
            "position_rad": unwrapped,
            "velocity_rad_s": 0.0,
            "temperature_c": 32.0,
            "merror": 0,
            "communication_ok": True,
            "unwrapped_raw_position_rad": unwrapped,
            "software_zero_reference_raw_rad": reference,
            "recovery_hint_configured": True,
            "recovery_hint_logical_position_rad": 0.0,
        })
    return {
        "schema": "go-m8010-motor-feedback/1.0",
        "source_monotonic_ns": 1_000_000_000 + index * 10_000_000,
        "samples": samples,
        "controller_mode": "brake",
        "controller_mode_by_motor": {"J2A": "brake", "J2B": "brake"},
        "j2_sync_fault": False,
        "domain_fault": False,
        "lease_safe_hold": False,
    }


def configs():
    return (
        {"motors": {
            "J2A": {"raw_position_rad": -4.596924286444928},
            "J2B": {"raw_position_rad": 12.42541044895233},
        }},
        {"motors": {
            "J2A": {"logical_position_rad": 0.0},
            "J2B": {"logical_position_rad": 0.0},
        }},
    )


def capture_metadata(count):
    received = [1_001_000_000 + index * 10_000_000 for index in range(count)]
    addresses = [("127.0.0.1", 49152)] * count
    return received, addresses


def analyze(packets, zero, hints, *, received=None, addresses=None):
    if received is None or addresses is None:
        default_received, default_addresses = capture_metadata(len(packets))
        received = default_received if received is None else received
        addresses = default_addresses if addresses is None else addresses
    return MODULE.analyze_packets(packets, received, addresses, zero, hints)


def terminal_proof(**overrides):
    fields = {
        "TERMINAL_PATH": "NORMAL",
        "GUI_GO_CONTROLLER_BUS": "j2",
        "EXECUTION_POLICY": "BRAKE_ONLY",
        "COMMAND_RX_ENABLED": "NO",
        "COMPLETED_CYCLES": "500",
        "TX_ATTEMPT_TOTAL": "1056",
        "BRAKE_TX_ATTEMPT_COUNT": "1056",
        "FOC_TX_ATTEMPT_COUNT": "0",
        "OTHER_MODE_TX_ATTEMPT_COUNT": "0",
        "BRAKE_ONLY_GUARD_BLOCK_COUNT": "0",
        "SERIAL_SEND_CALL_COUNT": "1056",
        "FOC_SERIAL_SEND_CALL_COUNT": "0",
        "BRAKE_ONLY_AUDIT": "PASS",
        "FINAL_MODE": "BRAKE",
        "FINAL_BRAKE": "PASS",
        "MOTOR_INTERNAL_ZERO_WRITE": "NO",
        "J2_STARTUP_BRAKE_PRIME_STATE": "PASS",
        "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS": "3",
        "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS": "3",
        "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS": "5",
        "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS": "5",
        "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS": "8",
        "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT": "16",
        "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS": "180000000",
        "FINAL_BRAKE_TX_ATTEMPT_COUNT": "40",
    }
    fields.update(overrides)
    return "\n".join(f"{key}={value}" for key, value in fields.items()) + "\n"


class BrakeFeedbackAnalysisTest(unittest.TestCase):
    def test_accepts_stationary_all_brake_feedback_and_maps_new_branch(self):
        zero, hints = configs()
        result = analyze([make_packet(index) for index in range(460)], zero, hints)
        self.assertEqual(result["packet_count"], 460)
        self.assertLess(abs(result["j2_common_mean_deg"]), 0.001)
        self.assertLess(result["j2_sync_max_abs_deg"], 0.5)
        self.assertAlmostEqual(
            result["selected_reference_raw_rad"]["J2A"],
            -4.596924286444928 + 2.0 * math.pi,
            places=12,
        )

    def test_rejects_non_brake_or_faulted_feedback(self):
        zero, hints = configs()
        packets = [make_packet(index) for index in range(460)]
        packets[20]["controller_mode"] = "hold"
        with self.assertRaises(MODULE.VerificationError):
            analyze(packets, zero, hints)
        packets = [make_packet(index) for index in range(460)]
        packets[20]["samples"][0]["merror"] = 1
        with self.assertRaises(MODULE.VerificationError):
            analyze(packets, zero, hints)

    def test_rejects_duplicate_motor_sample(self):
        zero, hints = configs()
        packets = [make_packet(index) for index in range(460)]
        packets[20]["samples"][1] = dict(packets[20]["samples"][0])
        with self.assertRaises(MODULE.VerificationError):
            analyze(packets, zero, hints)

    def test_rejects_opposed_individual_motor_motion(self):
        zero, hints = configs()
        packets = []
        for index in range(460):
            opposed = 0.0025 * math.sin(index / 15.0)
            packets.append(make_packet(index, q_a=opposed, q_b=-opposed))
        with self.assertRaises(MODULE.VerificationError):
            analyze(packets, zero, hints)

    def test_rejects_branch_change_and_replayed_timestamp(self):
        zero, hints = configs()
        packets = [make_packet(index) for index in range(460)]
        packets[100]["samples"][0]["software_zero_reference_raw_rad"] += 2.0 * math.pi
        with self.assertRaises(MODULE.VerificationError):
            analyze(packets, zero, hints)
        packets = [make_packet(index) for index in range(460)]
        packets[100]["source_monotonic_ns"] = packets[99]["source_monotonic_ns"]
        with self.assertRaises(MODULE.VerificationError):
            analyze(packets, zero, hints)

    def test_rejects_stale_source_or_compressed_local_capture(self):
        zero, hints = configs()
        packets = [make_packet(index) for index in range(460)]
        received, addresses = capture_metadata(len(packets))
        received[100] = packets[100]["source_monotonic_ns"] + 300_000_000
        with self.assertRaises(MODULE.VerificationError):
            analyze(packets, zero, hints, received=received, addresses=addresses)
        compressed = [1_001_000_000 + index * 1_000_000 for index in range(460)]
        with self.assertRaises(MODULE.VerificationError):
            analyze(packets, zero, hints, received=compressed, addresses=addresses)

    def test_worker_command_always_requests_brake_only(self):
        args = SimpleNamespace(
            worker=Path("/tmp/worker"),
            feedback_port=15300,
            zero_file=Path("/tmp/zero.json"),
            recovery_hint_file=Path("/tmp/hints.json"),
        )
        command = MODULE.build_worker_command(args)
        self.assertIn("--brake-only", command)
        self.assertNotIn("hold", command)
        self.assertNotIn("position", command)

    def test_terminal_proof_requires_exact_unique_zero_foc_evidence(self):
        proof = MODULE.parse_worker_terminal_proof(terminal_proof())
        self.assertEqual(proof["BRAKE_ONLY_AUDIT"], "PASS")
        zero_prefix = MODULE.parse_worker_terminal_proof(
            terminal_proof(
                J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS="0",
                J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS="5",
                J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT="10",
                J2_STARTUP_BRAKE_PRIME_ELAPSED_NS="40000000",
            )
        )
        self.assertEqual(
            zero_prefix["J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS"], "0"
        )
        with self.assertRaises(MODULE.VerificationError):
            MODULE.parse_worker_terminal_proof(
                terminal_proof(FOC_SERIAL_SEND_CALL_COUNT="1")
            )
        with self.assertRaises(MODULE.VerificationError):
            MODULE.parse_worker_terminal_proof(
                terminal_proof() + "FINAL_BRAKE=PASS\n"
            )
        with self.assertRaises(MODULE.VerificationError):
            MODULE.parse_worker_terminal_proof(
                terminal_proof(J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS="4")
            )
        with self.assertRaises(MODULE.VerificationError):
            MODULE.parse_worker_terminal_proof(
                terminal_proof(J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS="9")
            )
        with self.assertRaises(MODULE.VerificationError):
            MODULE.parse_worker_terminal_proof(
                terminal_proof(J2_STARTUP_BRAKE_PRIME_ELAPSED_NS="500000001")
            )
        with self.assertRaises(MODULE.VerificationError):
            MODULE.parse_worker_terminal_proof(
                terminal_proof(J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS="08")
            )
        missing_prime = "\n".join(
            line
            for line in terminal_proof().splitlines()
            if not line.startswith("J2_STARTUP_BRAKE_PRIME_STATE=")
        )
        with self.assertRaises(MODULE.VerificationError):
            MODULE.parse_worker_terminal_proof(missing_prime)

    def test_worker_cycle_accounting_is_an_exact_three_phase_total(self):
        proof = MODULE.parse_worker_terminal_proof(terminal_proof())
        accounting = MODULE.validate_worker_cycle_accounting(proof, 500)
        self.assertEqual(accounting["startup_prime_tx_attempt_count"], 16)
        self.assertEqual(accounting["control_loop_tx_attempt_count"], 1000)
        self.assertEqual(accounting["final_brake_tx_attempt_count"], 40)
        self.assertEqual(accounting["aggregate_tx_attempt_count"], 1056)
        for total in (1016, 1040, 1055, 1057, 1_000_000):
            with self.assertRaises(MODULE.VerificationError):
                MODULE.validate_worker_cycle_accounting(
                    MODULE.parse_worker_terminal_proof(
                        terminal_proof(
                            TX_ATTEMPT_TOTAL=str(total),
                            BRAKE_TX_ATTEMPT_COUNT=str(total),
                            SERIAL_SEND_CALL_COUNT=str(total),
                        )
                    ),
                    500,
                )
        with self.assertRaises(MODULE.VerificationError):
            MODULE.validate_worker_cycle_accounting(proof, 499)

    def test_configuration_must_match_pinned_hashes(self):
        zero = {
            "schema": "go-m8010-persistent-software-zero/1.0",
            "reference_name": "PERSISTENT_SOFTWARE_ZERO_V1",
            "motors": {
                "J2A": {"raw_position_rad": -4.596924286444928},
                "J2B": {"raw_position_rad": 12.42541044895233},
            },
        }
        hints = {
            "schema": "go-m8010-recovery-branch-hints/1.0",
            "reference_name": "PERSISTENT_SOFTWARE_ZERO_V1",
            "motors": {
                "J2A": {"logical_position_rad": 0.0},
                "J2B": {"logical_position_rad": 0.0},
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            zero_path = root / "persistent_software_zero.json"
            hint_path = root / "recovery_branch_hints.json"
            zero_bytes = (json.dumps(zero) + "\n").encode()
            hint_bytes = (json.dumps(hints) + "\n").encode()
            zero_path.write_bytes(zero_bytes)
            hint_path.write_bytes(hint_bytes)
            zero_sha = hashlib.sha256(zero_bytes).hexdigest()
            hint_sha = hashlib.sha256(hint_bytes).hexdigest()
            Path(str(zero_path) + ".sha256").write_text(
                f"{zero_sha}  {zero_path.name}\n", encoding="utf-8"
            )
            _, _, evidence = MODULE.load_configuration(
                zero_path, hint_path, zero_sha, hint_sha
            )
            self.assertEqual(evidence["persistent_zero_sha256"], zero_sha)
            with self.assertRaises(MODULE.VerificationError):
                MODULE.load_configuration(zero_path, hint_path, "0" * 64, hint_sha)


if __name__ == "__main__":
    unittest.main()
