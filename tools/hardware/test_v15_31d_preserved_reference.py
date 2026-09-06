"""Offline issuer + state mapping + optional production C++ consumer check."""
import copy
import hashlib
import json
import math
import os
import subprocess
import tempfile
import unittest
import time
from pathlib import Path

import test_v15_30a_create_j2_vertical_session_phase_anchor as j2
import test_v15_30a_create_go_aux_vertical_session_phase_anchor as aux
from go_m8010_arm_hardware.state_model import MirrorSessionReferenceV1, MotorFeedback


class PreservedReferenceTest(unittest.TestCase):
    def exercise(self, auxiliary=False, *, recovery=False, offsets=None):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        args, _ = j2.make_fixture_tree(root)
        tool = aux.MODULE if auxiliary else j2.MODULE
        if auxiliary:
            args.physical_confirmation = tool.PHYSICAL_GATE
            args.operator_power_session_id = aux.POWER_SESSION_ID
            auxiliary_capture = aux.capture()
            for domain in auxiliary_capture["domains"].values():
                domain["local_receive_coverage_s"] = 4.99
            args.expected_capture_sha256 = hashlib.sha256(
                j2.write_json(args.capture_statistics_file, auxiliary_capture)
            ).hexdigest()
        runtime = dict(now_utc=j2.TEST_ISSUED_AT_UTC, host_boot_id=j2.TEST_HOST_BOOT_ID,
                       issued_boottime_ns=j2.TEST_ISSUED_BOOTTIME_NS)
        original = tool.run(args, **runtime)["anchor"]
        original_capture_path = args.capture_statistics_file
        original_capture_path.chmod(0o600)
        source_path = root / "original_anchor.json"
        source_data = j2.write_json(source_path, original)
        source_path.chmod(0o600)
        args.preserve_reference_file = source_path
        args.expected_preserve_reference_sha256 = hashlib.sha256(source_data).hexdigest()
        args.supported_near_vertical_recovery = recovery
        protected = {p: p.read_bytes() for p in (
            args.zero_file, args.zero_sha256_file, args.recovery_hint_file,
            args.initial_pose_file, source_path, original_capture_path)}
        expected = ({"J1": 0.256, "J3": -0.021, "J4": 0.020, "J5": -1.57}
                    if auxiliary else {"J2A": 0.007, "J2B": 0.139})
        expected.update(offsets or {})
        capture = json.loads(args.capture_statistics_file.read_text(encoding="utf-8"))
        args.capture_statistics_file = root / "fresh_capture.json"
        for name, delta in expected.items():
            offset = original["motors"][name]["sign"] * j2.MODULE.GEAR_RATIO * math.radians(delta)
            # Exercise a rotor wrap as well as physical drift. Both must retain
            # the original zero; J2A and J2B retain their different deviations.
            offset += 2 * math.pi
            for key in ("mean", "minimum", "maximum"):
                capture["motors"][name]["unwrapped_raw_position_rad"][key] += offset
        args.expected_capture_sha256 = hashlib.sha256(
            j2.write_json(args.capture_statistics_file, capture)
        ).hexdigest()
        anchor = tool.run(args, **runtime)["anchor"]
        derived = j2.MODULE.validate_preserved_session_reference(anchor)
        for name, value in expected.items():
            self.assertAlmostEqual(math.degrees(derived[name]), value, places=9)
            self.assertEqual(anchor["motors"][name]["session_reference_raw_rad"],
                             original["motors"][name]["session_reference_raw_rad"])
            self.assertEqual(anchor["motors"][name]["logical_position_rad"], 0.0)
        self.assertEqual(source_path.read_bytes(), source_data)
        if not auxiliary:
            self.assertAlmostEqual(math.degrees(derived["J2B"] - derived["J2A"]), expected["J2B"] - expected["J2A"], places=9)
        if recovery:
            zero = json.loads(args.zero_file.read_bytes())
            model_args = dict(persistent_references={n: v["raw_position_rad"] for n, v in zero["motors"].items()},
                              recovery_hints={"J2A": 0.0, "J2B": 0.0}, runtime_startup_hints=derived)
            prefix = "go_aux" if auxiliary else "j2"
            model_args[prefix + "_session_references"] = {n: v["session_reference_raw_rad"] for n, v in anchor["motors"].items()}
            model_args[prefix + "_session_hints"] = {n: v["logical_position_rad"] for n, v in anchor["motors"].items()}
            model = MirrorSessionReferenceV1(**model_args)
            stamp = time.monotonic_ns()
            for name, value in derived.items():
                raw = anchor["raw_capture"]["motors"][name]["unwrapped_raw_position_rad"]["mean"]
                sign = anchor["motors"][name]["sign"]
                model.update(MotorFeedback(name, raw, 0.0, 30.0, 0, True, stamp, stamp))
                self.assertAlmostEqual(sign * (raw - model.references[name]) / j2.MODULE.GEAR_RATIO, value, places=12)
                self.assertEqual(model.power_session_hints[name], 0.0)
                outside = MirrorSessionReferenceV1(**model_args)
                with self.assertRaises(ValueError):
                    outside.update(MotorFeedback(name, raw + sign * j2.MODULE.GEAR_RATIO * math.radians(2.01), 0.0, 30.0, 0, True, stamp, stamp))
        binary = os.environ.get("M8010_REFERENCE_AUDIT_BINARY")
        if binary:
            completed = subprocess.run([binary], input=json.dumps(anchor), text=True,
                                       capture_output=True, timeout=10)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(completed.stdout)
            self.assertFalse(result["serial_opened"])
            for name, value in derived.items():
                self.assertAlmostEqual(result["motors"][name]["startup_logical"], value, places=12)
                self.assertAlmostEqual(result["motors"][name]["runtime_logical"], value, places=12)
                if recovery:
                    self.assertTrue(result["motors"][name]["runtime_two_degree_drift_rejected"])
            tampered = copy.deepcopy(anchor)
            tampered["motors"][next(iter(expected))]["startup_logical_position_rad"] = 0.0
            rejected = subprocess.run([binary], input=json.dumps(tampered), text=True,
                                      capture_output=True, timeout=10)
            self.assertNotEqual(rejected.returncode, 0)
        args.apply, args.confirm, args.defer_launch_permit = True, tool.APPLY_GATE, True
        published = tool.run(args, **runtime)
        self.assertEqual(json.loads(Path(published["anchor_path"]).read_text(encoding="utf-8")), anchor)
        self.assertTrue(all(p.read_bytes() == data for p, data in protected.items()))
        return anchor, source_path

    def test_j2_offsets_and_sync_survive_new_capture(self):
        self.exercise()

    def test_auxiliary_pose_survives_new_capture(self):
        self.exercise(True)

    def test_explicit_supported_recovery_preserves_geometry_and_runtime_window(self):
        with self.assertRaises(ValueError):
            self.exercise(True, offsets={"J4": 2.067051})
        anchor, _ = self.exercise(True, recovery=True, offsets={"J4": 2.067051})
        j2_anchor, _ = self.exercise(recovery=True, offsets={"J2A": 1.969163, "J2B": 2.211498})
        candidates = []
        missing = copy.deepcopy(anchor)
        del missing["preserved_reference"]["supported_near_vertical_recovery"]
        candidates.append(missing)
        for field, value in (("software_maximum_offset_deg", 6.0), ("support_reliable", False)):
            changed = copy.deepcopy(anchor)
            changed["preserved_reference"]["supported_near_vertical_recovery"][field] = value
            candidates.append(changed)
        for original, name, delta in ((anchor, "J4", 3.0), (j2_anchor, "J2B", 0.4)):
            changed = copy.deepcopy(original)
            record = changed["motors"][name]
            raw = changed["raw_capture"]["motors"][name]["unwrapped_raw_position_rad"]
            for field in ("mean", "minimum", "maximum"):
                raw[field] += record["sign"] * j2.MODULE.GEAR_RATIO * math.radians(delta)
            record["startup_logical_position_rad"] += math.radians(delta)
            candidates.append(changed)
        for candidate in candidates:
            with self.assertRaises(ValueError):
                j2.MODULE.validate_preserved_session_reference(candidate)
            binary = os.environ.get("M8010_REFERENCE_AUDIT_BINARY")
            if binary:
                result = subprocess.run([binary], input=json.dumps(candidate), text=True, capture_output=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)

    def test_supported_recovery_still_rejects_five_degree_edges_and_sync(self):
        for offsets in ({"J4": 5.01}, {"J4": 4.999}):
            with self.assertRaises(ValueError):
                self.exercise(True, recovery=True, offsets=offsets)
        with self.assertRaises(ValueError):
            self.exercise(recovery=True, offsets={"J2A": 2.0, "J2B": 2.6})

    def test_supported_recovery_requires_original_preserve_reference(self):
        with tempfile.TemporaryDirectory() as temporary:
            args, _ = j2.make_fixture_tree(Path(temporary))
            args.supported_near_vertical_recovery = True
            with self.assertRaises(ValueError):
                j2.run_tool(args)

    def test_geometry_source_and_claim_tampering_are_rejected(self):
        anchor, path = self.exercise()
        for field in ("session_reference_raw_rad", "startup_logical_position_rad"):
            changed = copy.deepcopy(anchor)
            changed["motors"]["J2A"][field] += 0.01
            with self.assertRaises(ValueError):
                j2.MODULE.validate_preserved_session_reference(changed)
        path.write_bytes(path.read_bytes() + b" ")
        with self.assertRaises(ValueError):
            j2.MODULE.validate_preserved_session_reference(anchor)

    def test_nearby_branch_and_sync_bounds_are_not_relaxed(self):
        anchor, _ = self.exercise()
        for delta in (0.7, 2.1):
            changed = copy.deepcopy(anchor)
            motor = changed["motors"]["J2B"]
            raw = changed["raw_capture"]["motors"]["J2B"]["unwrapped_raw_position_rad"]
            for field in ("mean", "minimum", "maximum"):
                raw[field] += j2.MODULE.GEAR_RATIO * math.radians(delta)
            motor["startup_logical_position_rad"] += math.radians(delta)
            with self.assertRaises(ValueError):
                j2.MODULE.validate_preserved_session_reference(changed)

    def test_rehashed_bad_historical_evidence_is_rejected(self):
        anchor, source_path = self.exercise()
        source = json.loads(source_path.read_bytes())
        raw_path = Path(source["source_evidence"]["path"])
        original_raw = raw_path.read_bytes()
        for kind in ("raw_hash", "confirmation", "embedded_statistics", "raw_fail",
                     "raw_low_count", "raw_short_coverage"):
            with self.subTest(kind=kind):
                raw_path.write_bytes(original_raw)
                changed = copy.deepcopy(source)
                if kind == "raw_hash":
                    changed["source_evidence"]["sha256"] = "0" * 64
                elif kind == "confirmation":
                    changed["operator_confirmation"]["support_reliable"] = False
                elif kind == "embedded_statistics":
                    changed["raw_capture"]["motors"]["J2A"]["unwrapped_raw_position_rad"]["mean"] += 0.001
                else:
                    raw = json.loads(original_raw)
                    if kind == "raw_fail":
                        raw["status"] = "FAIL"
                    elif kind == "raw_short_coverage":
                        raw["source_coverage_s"] = changed["raw_capture"]["source_coverage_s"] = 3.0
                    else:
                        raw["packet_count"] = changed["raw_capture"]["packet_count"] = 100
                        for name in raw["motors"]:
                            raw["motors"][name]["sample_count"] = 100
                            changed["raw_capture"]["motors"][name]["sample_count"] = 100
                            changed["motors"][name]["sample_count"] = 100
                    raw_data = j2.write_json(raw_path, raw)
                    digest = hashlib.sha256(raw_data).hexdigest()
                    changed["source_evidence"]["sha256"] = digest
                    changed["raw_capture"]["source_file_sha256"] = digest
                new_source = j2.write_json(source_path, changed)
                candidate = copy.deepcopy(anchor)
                candidate["preserved_reference"]["source_sha256"] = hashlib.sha256(new_source).hexdigest()
                with self.assertRaises(ValueError):
                    j2.MODULE.validate_preserved_session_reference(candidate)
                binary = os.environ.get("M8010_REFERENCE_AUDIT_BINARY")
                if binary:
                    result = subprocess.run([binary], input=json.dumps(candidate), text=True,
                                            capture_output=True, timeout=10)
                    self.assertNotEqual(result.returncode, 0, result.stdout)


if __name__ == "__main__":
    unittest.main()
