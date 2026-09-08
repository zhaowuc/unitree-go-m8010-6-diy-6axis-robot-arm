import math
import json
import ast
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import pytest

from j6_protocol_torque import (QUERY_RIDS, qualify_readback, readback_sha256,
                               signed_motor_torque_estimate, validate_readonly_request)


def test_readonly_wire_allowlist_and_pinned_motor_torque_scale():
    readback = {"7": 0, "8": 1, "10": 2, "21": 12.5, "22": 45.0,
                "23": 10.0, "1": 0.0, "20": 1.0}
    sha = readback_sha256(readback)
    qualified = qualify_readback(readback, expected_sha256=sha)
    assert qualified["accuracy_not_physically_calibrated"]
    assert not qualified["external_torque_measured"]
    assert not qualified["independent_current_measured"]
    # The exact midpoint code is not zero in a symmetric 12-bit DM mapping.
    protocol_value = -10 + 2047 * 20 / 4095
    estimate = signed_motor_torque_estimate(protocol_value, readback, expected_readback_sha256=sha)
    assert estimate["motor_torque_estimated_nm"] == protocol_value
    assert estimate["joint_motor_torque_estimated_nm"] == -protocol_value
    for rid in QUERY_RIDS:
        validate_readonly_request(0x7FF, bytes((1, 0, 0x33, rid, 0, 0, 0, 0)))
    validate_readonly_request(0x7FF, bytes((1, 0, 0xCC, 0, 0, 0, 0, 0)))
    for can_id, payload in [(1, b'\xff'*7+b'\xfc'), (1, b'\xff'*7+b'\xfd'),
                            (0x7FF, bytes((1, 0, 0x55, 10, 2, 0, 0, 0)))]:
        with pytest.raises(ValueError): validate_readonly_request(can_id, payload)
    changed = {**readback, "23": 18.0}
    with pytest.raises(ValueError, match="SHA256"):
        qualify_readback(changed, expected_sha256=sha)
    with pytest.raises(ValueError, match="RID_23"):
        qualify_readback(changed, expected_sha256=readback_sha256(changed))
    with pytest.raises(ValueError, match="SIGN"):
        qualify_readback(readback, expected_sha256=sha, protocol_to_joint_sign=1)
    with pytest.raises(ValueError, match="TORQUE"):
        signed_motor_torque_estimate(math.nan, readback, expected_readback_sha256=sha)
    # Exercise the actual production serializer without a socket or hardware.
    from test_v15_30a_gui_profile import load_send_feedback
    sender = load_send_feedback()
    sender.__globals__["math"] = math
    emitted = []
    sock = SimpleNamespace(sendto=lambda payload, address: emitted.append(json.loads(payload)))
    decoded = SimpleNamespace(position=1.0, velocity=0.0, torque=protocol_value,
                              state=1, mos_temp=25, coil_temp=25)
    sender(sock, 15300, decoded, True, "hold", False, False,
           last_valid_feedback_monotonic_ns=1234)
    assert "protocol_torque" not in emitted[-1]["samples"][0]
    sender(sock, 15300, decoded, True, "hold", False, False,
           last_valid_feedback_monotonic_ns=1234, observe_protocol_torque=True)
    sample = emitted[-1]["samples"][0]
    assert sample["protocol_torque"] == protocol_value
    assert sample["protocol_torque_source_monotonic_ns"] == 1234
    assert sample["tau_feedback_rotor_nm"] is None
    assert sample["protocol_torque_is_external_measurement"] is False
    sender(sock, 15300, decoded, False, "hold", False, False,
           last_valid_feedback_monotonic_ns=1234, observe_protocol_torque=True)
    assert emitted[-1]["samples"][0]["protocol_torque"] is None
    # The qualified opt-in traverses the real raw parser and the exact
    # per_motor snapshot subsequently serialized by whole_arm_state_node.
    root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(root / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环" /
                          "ros2_ws/src/go_m8010_arm_hardware"))
    from go_m8010_arm_hardware.state_model import parse_feedback_payload, MirrorSessionReferenceV1, MOTOR_NAMES
    native_stamp = time.monotonic_ns()
    sender(sock, 15300, decoded, True, "hold", False, False,
           last_valid_feedback_monotonic_ns=native_stamp, observe_protocol_torque=True,
           protocol_torque_qualification=qualified)
    payload = emitted[-1]
    receipt = payload["source_monotonic_ns"] + 1000
    sample = parse_feedback_payload(payload, receipt)[0]
    model = MirrorSessionReferenceV1(persistent_references={name: 0.0 for name in MOTOR_NAMES})
    model.update(sample)
    motor = model.snapshot_available(receipt)["per_motor"]["J6"]
    assert motor["joint_motor_torque_estimated_nm"] == -protocol_value
    assert motor["motor_torque_qualification_sha256"] == sha
    assert motor["motor_torque_source_monotonic_ns"] == native_stamp
    assert motor["motor_torque_estimate_metadata_status"] == "OBSERVED"
    assert motor["tau_feedback_rotor_nm"] is None
    # Re-publishing the same CAN measurement must retain its true source
    # stamp. UDP publication time is deliberately newer and is not equality-
    # checked against the torque measurement timestamp.
    sender.__globals__["time"] = SimpleNamespace(monotonic_ns=lambda: native_stamp + 40_000_000)
    sender(sock, 15300, decoded, True, "hold", False, False,
           last_valid_feedback_monotonic_ns=native_stamp, observe_protocol_torque=True,
           protocol_torque_qualification=qualified)
    repeated_payload = emitted[-1]
    assert repeated_payload["source_monotonic_ns"] != native_stamp
    repeated_receipt = native_stamp + 40_001_000
    model.update(parse_feedback_payload(repeated_payload, repeated_receipt)[0])
    retained = model.snapshot_available(repeated_receipt)["per_motor"]["J6"]
    assert retained["motor_torque_estimate_metadata_status"] == "OBSERVED"
    assert retained["motor_torque_source_monotonic_ns"] == native_stamp
    assert retained["joint_motor_torque_estimated_nm"] == -protocol_value
    # The same measurement genuinely expires at 100 ms even though the
    # packet itself and the ordinary position feedback remain fresh.
    expired = model.snapshot_available(native_stamp + 100_000_001)["per_motor"]["J6"]
    assert expired["fresh"] and expired["communication_ok"]
    assert expired["motor_torque_estimate_metadata_status"] == "STALE"
    assert expired["joint_motor_torque_estimated_nm"] is None
    stale = model.snapshot_available(receipt + 100_000_001)["per_motor"]["J6"]
    assert stale["joint_motor_torque_estimated_nm"] is None
    assert stale["motor_torque_estimate_metadata_status"] == "STALE"
    payload["samples"][0]["motor_torque_qualification_sha256"] = "bad"
    malformed = parse_feedback_payload(payload, receipt)[0]
    assert malformed.position_rad == decoded.position and malformed.communication_ok
    assert malformed.j6_motor_torque_observation["motor_torque_estimate_metadata_status"] == "INVALID"


def test_active_optin_requests_feedback_each_cycle_and_keeps_identical_real_frames():
    # Execute the production branch and production strict drain, with the
    # logger's real queue/classifier but without constructing any CAN device.
    from j6_raw_can_diagnostic import RawCanLogger, RawEvent, refresh_request, strict_decode
    path = Path(__file__).with_name("v15_30a_gui_j6_controller.py")
    tree = ast.parse(path.read_text(encoding="utf-8"))
    branch = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
        and any(isinstance(child, ast.Constant) and child.value == "GUI_PROTOCOL_TORQUE_REFRESH"
                for statement in node.body for child in ast.walk(statement))
        and "observe_protocol_torque" in ast.unparse(node.test))
    drain = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "drain_feedback_batch")
    logger = RawCanLogger.__new__(RawCanLogger)
    logger.channel, logger.motor_id, logger.master_id = 0, 1, 0
    logger._events, logger._lock = [], threading.Lock()
    requests = []
    def send(can_id, payload, label):
        validate_readonly_request(can_id, payload)
        requests.append((can_id, payload, label))
        logger._events.append(RawEvent(
            event_monotonic_s=1.0 + (len(requests)-1)*.01, event_kind="RX_CALLBACK",
            sdk_timestamp=len(requests), direction=0, channel=0, can_id=0,
            dlc=8, data_length=8, extended=False, canfd=False, rtr=False, brs=False,
            ack=False, esi=False, payload=bytes.fromhex("1180007ff7ff2020")))
    logger.send = send
    namespace = dict(RawCanLogger=RawCanLogger, strict_decode=strict_decode,
        refresh_request=refresh_request, logger=logger, enabled=True,
        args=SimpleNamespace(observe_protocol_torque=True),
        consume_transport_interlock=lambda _: False, received=[])
    loop = ast.parse("for _ in range(100):\n    pass\n    received.extend(drain_feedback_batch(logger)[0])").body[0]
    loop.body[0] = branch
    executable = compile(ast.fix_missing_locations(ast.Module(body=[drain, loop], type_ignores=[])), str(path), "exec")
    exec(executable, namespace)
    assert len(requests) == len(namespace["received"]) == 100
    assert {request[:2] for request in requests} == {(0x7FF, bytes((1,0,0xCC,0,0,0,0,0)))}
    received = namespace["received"]
    assert len({sample for sample, _ in received}) == 1  # No fabricated changes.
    assert len({stamp for _, stamp in received}) == 100  # Identical frames are not deduplicated.
    assert received[-1][1] - received[0][1] == pytest.approx(.99)
    namespace["args"].observe_protocol_torque = False
    exec(executable, namespace)
    namespace["args"].observe_protocol_torque = True
    namespace["enabled"] = False
    exec(executable, namespace)
    assert len(requests) == 100  # Legacy/default and disabled paths are unchanged.
