import ast
import importlib.util
import json
import math
import sys
from pathlib import Path


MODULE_DIRECTORY = Path(__file__).resolve().parent
CONTROLLER = MODULE_DIRECTORY / "v15_30a_gui_j6_controller.py"
STOP_TOOL = MODULE_DIRECTORY.parent / "v15_30a_gui_stop.py"
sys.path.insert(0, str(MODULE_DIRECTORY))
SPEC = importlib.util.spec_from_file_location(
    "v15_30a_profile", MODULE_DIRECTORY / "v15_30a_profile.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def load_parse_command():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "parse_command"
    )
    namespace = {
        "json": json,
        "math": math,
        "time": __import__("time"),
        "TARGET_LIMIT": math.radians(10.0),
        "VMAX_LIMIT": math.radians(5.0),
        "AMAX_LIMIT": math.radians(20.0),
    }
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace["parse_command"]


def load_mode_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "command_requests_j6_active",
        "command_epoch_is_acceptable",
        "effective_command_mode",
        "minimum_epoch_after_interarrival_lease",
        "observe_valid_command_epoch",
    }
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {"time": __import__("time"), "LEASE_S": 0.5}
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace


def load_command_channel_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "parse_command",
        "command_requests_j6_active",
        "observe_valid_command_epoch",
        "command_epoch_is_acceptable",
        "minimum_epoch_after_interarrival_lease",
        "receive_latest",
    }
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "json": json,
        "math": math,
        "socket": __import__("socket"),
        "time": __import__("time"),
        "LEASE_S": 0.5,
        "TARGET_LIMIT": math.radians(10.0),
        "VMAX_LIMIT": math.radians(5.0),
        "AMAX_LIMIT": math.radians(20.0),
    }
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace


def command_payload(schema="go-m8010-gui-command/1.1", mask=None):
    return json.dumps({
        "schema": schema,
        "mode": "position",
        "targets_rad": [0.0] * 6,
        "active_joint_mask": [False] * 5 + [True] if mask is None else mask,
        "activation_epoch": 1,
        "maximum_velocity_rad_s": math.radians(5.0),
        "maximum_acceleration_rad_s2": math.radians(20.0),
    }).encode()


def test_profile_converges_without_velocity_or_acceleration_jump():
    q_command = 0.0
    dq_command = 0.0
    vmax = math.radians(5.0)
    amax = math.radians(20.0)
    target = math.radians(10.0)
    reached_exactly = False
    for _ in range(3000):
        previous_velocity = dq_command
        q_command, dq_command = MODULE.update_profile(
            q_command, dq_command, target, vmax, amax, 0.01
        )
        assert abs(dq_command) <= vmax + 1e-12
        assert abs(dq_command - previous_velocity) <= amax * 0.01 + 1e-12
        if q_command == target and dq_command == 0.0:
            reached_exactly = True
            break
    assert reached_exactly


def test_j6_parser_requires_v11_six_boolean_mask():
    parse_command = load_parse_command()
    assert parse_command(command_payload())["active_joint_mask"][5] is True
    for payload in (
        command_payload(schema="go-m8010-gui-command/1.0"),
        command_payload(mask=[False] * 5),
        command_payload(mask=[False] * 5 + [1]),
    ):
        try:
            parse_command(payload)
        except ValueError:
            pass
        else:
            raise AssertionError("J6 接受了不安全的命令协议或激活掩码")


def test_j6_parser_keeps_legacy_brake_compatibility():
    parse_command = load_parse_command()
    document = json.loads(command_payload(schema="go-m8010-gui-command/1.0"))
    document["mode"] = "brake"
    document.pop("active_joint_mask")
    parsed = parse_command(json.dumps(document).encode())
    assert parsed["active_joint_mask"] == [False] * 6


def test_j6_parser_rejects_zero_epoch_active_command():
    parse_command = load_parse_command()
    document = json.loads(command_payload())
    document["activation_epoch"] = 0
    try:
        parse_command(json.dumps(document).encode())
    except ValueError as error:
        assert "必须大于零" in str(error)
    else:
        raise AssertionError("J6 接受了零激活纪元主动命令")


def test_j6_control_loop_checks_its_own_active_bit_before_enable():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert 'and command["active_joint_mask"][5]' in source
    assert 'pre_enable_mode = effective_command_mode' in source
    assert source.count("= receive_latest(") >= 4


def test_j6_effective_mode_requires_fresh_new_epoch_authorization():
    functions = load_mode_functions()
    effective = functions["effective_command_mode"]
    command = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    assert effective(command, 0, 10.1) == "position"
    assert effective(command, 8, 10.1) == "brake"
    assert effective(command, 0, 10.6) == "brake"
    command["active_joint_mask"][5] = False
    assert effective(command, 0, 10.1) == "brake"


def test_j6_brake_packet_revokes_same_epoch_even_if_followed_by_active_packet():
    functions = load_mode_functions()
    observe = functions["observe_valid_command_epoch"]
    effective = functions["effective_command_mode"]
    active = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    brake = {
        "mode": "brake",
        "active_joint_mask": [False] * 6,
        "activation_epoch": 0,
        "received_at": 10.0,
    }
    minimum, last_seen = observe(active, 0, 0)
    minimum, last_seen = observe(brake, minimum, last_seen)
    assert (minimum, last_seen) == (8, 7)
    assert effective(active, minimum, 10.1) == "brake"
    active["activation_epoch"] = 8
    assert effective(active, minimum, 10.1) == "position"


def test_j6_rejects_older_active_epoch_but_always_accepts_brake():
    functions = load_mode_functions()
    acceptable = functions["command_epoch_is_acceptable"]
    command = {
        "mode": "position",
        "activation_epoch": 100,
    }
    assert not acceptable(command, 101)
    command["activation_epoch"] = 101
    assert acceptable(command, 101)
    command["mode"] = "brake"
    command["activation_epoch"] = 0
    assert acceptable(command, 101)


def test_j6_interarrival_gap_revokes_same_epoch_before_timestamp_overwrite():
    functions = load_mode_functions()
    observe_gap = functions["minimum_epoch_after_interarrival_lease"]
    active = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 21,
        "received_at": 10.0,
    }
    assert observe_gap(active, 0, 10.49) == 0
    assert observe_gap(active, 0, 10.501) == 22
    active["mode"] = "brake"
    assert observe_gap(active, 0, 11.0) == 0


def test_j6_drains_large_backlog_and_hard_stop_fences_unseen_active_packet():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    active = command_payload()
    brake_document = json.loads(command_payload())
    brake_document["mode"] = "brake"
    brake_document["active_joint_mask"] = [False] * 6
    brake_document["activation_epoch"] = (1 << 63) - 1
    brake = json.dumps(brake_document).encode()

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    fake_socket = FakeSocket([active] * 100 + [brake, active])
    current, minimum, last_seen = receive_latest(fake_socket, None, 0, 0)
    assert fake_socket.packets == []
    assert current["mode"] == "brake"
    assert minimum == 1 << 63
    assert last_seen == (1 << 63) - 1


def test_external_stop_uses_session_terminal_activation_epoch_fence():
    source = STOP_TOOL.read_text(encoding="utf-8")
    assert '"activation_epoch": (1 << 63) - 1' in source
