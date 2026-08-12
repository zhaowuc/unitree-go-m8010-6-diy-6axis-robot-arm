#!/usr/bin/env python3
from __future__ import annotations

"""MuJoCo authority behind topic_based_ros2_control.

This node deliberately does not provide FollowJointTrajectory.  The standard
joint_trajectory_controller owns that action and publishes interpolated
position commands through topic_based_ros2_control.  This bridge validates the
swept command, writes the accepted qpos/qvel into the accepted kinematic MJCF,
and publishes the resulting state on a private raw-state topic.
"""

import importlib.util
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from typing import Iterable
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
ACCEPTED_MODEL_SHA256 = (
    "ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"
)
ACCEPTED_COLLISION_CONTRACT_SHA256 = (
    "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
)
ACCEPTED_GUARD_SHA256 = (
    "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
)
ACCEPTED_RUNTIME_MESH_MANIFEST_SHA256 = (
    "1174abcfef3b87ee77d4ce50af5afc1aa62d45759f1560ace361997e8c60b19d"
)
ACCEPTED_RUNTIME_ASSET_SET_SHA256 = (
    "fe517900413488414f9a606d7fd8f05ea8a7f2be08043672e75c57ae669f0425"
)
ACCEPTED_SOURCE_MESH_MANIFEST_SHA256 = (
    "22cfb9a194495b442731405a023cec8f83fb03d17b2e814cf13febc924d1ce3a"
)
MOTION_PROXY_MESH_SHA256 = {
    "meshes/collision_motion_proxy_mm/link2/UpperArm_Motion_Collision_Proxy__component_001.stl":
        "1f3b3c3878b93c450c9c90590542174e18259902b22eb81dc8a427563ee3f555",
    "meshes/collision_motion_proxy_mm/link2/UpperArm_Motion_Collision_Proxy__component_002.stl":
        "11fb6c54f4d972e7c13b86167ea7cb1987354fa59defce43a0d9d4ff9f0ccf2a",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_runtime_mesh_assets(model_path: Path) -> tuple[dict, dict[Path, tuple[int, int]]]:
    """Fail closed unless every MJCF mesh is an accepted immutable asset.

    The accepted V15.14 MJCF deliberately reuses the audited V15.13 visual
    chunks and convex collision pieces, plus the two new UpperArm Motion
    components.  Hashing only the XML would not protect those external files.
    """
    root = ET.parse(model_path).getroot()
    mesh_nodes = root.findall("./asset/mesh")
    mesh_files = [str(mesh.get("file")) for mesh in mesh_nodes if mesh.get("file")]
    mesh_names = [str(mesh.get("name")) for mesh in mesh_nodes if mesh.get("name")]
    external_file_elements = [element for element in root.iter() if element.get("file")]
    if len(mesh_nodes) != 1008 or len(external_file_elements) != 1008:
        raise RuntimeError("fail-closed: expected exactly 1008 mesh file assets")
    if any(element.tag != "mesh" for element in external_file_elements):
        raise RuntimeError("fail-closed: unaccepted non-mesh external MJCF asset")
    if len(mesh_names) != 1008 or len(set(mesh_names)) != 1008:
        raise RuntimeError("fail-closed: MJCF mesh names must be unique and complete")
    resolved_meshes = [(model_path.parent / value).resolve() for value in mesh_files]
    if len(resolved_meshes) != len(set(resolved_meshes)):
        raise RuntimeError("fail-closed: duplicate MJCF mesh file references")
    mesh_use_count: dict[str, int] = {name: 0 for name in mesh_names}
    for geom in root.findall(".//geom"):
        mesh_name = geom.get("mesh")
        if mesh_name:
            if mesh_name not in mesh_use_count:
                raise RuntimeError(
                    f"fail-closed: geom references undefined mesh: {mesh_name}"
                )
            mesh_use_count[mesh_name] += 1
    bad_use_count = {
        name: count for name, count in mesh_use_count.items() if count != 1
    }
    if bad_use_count:
        raise RuntimeError(
            "fail-closed: every accepted mesh must be used exactly once: "
            + json.dumps(bad_use_count, ensure_ascii=False)
        )
    geom_names = {
        str(geom.get("name"))
        for geom in root.findall(".//geom")
        if geom.get("name")
    }
    motion_geoms = sorted(
        name
        for name in geom_names
        if "UpperArm_Motion_Collision_Proxy" in name
    )
    j1_geoms = sorted(
        name
        for name in geom_names
        if "J1_Fixed_Collision_Proxy" in name
        or "J1_Moving_Collision_Proxy" in name
    )
    expected_explicit_pairs = {
        tuple(sorted((motion, other)))
        for motion in motion_geoms
        for other in j1_geoms
    }
    actual_explicit_pairs = {
        tuple(sorted((str(pair.get("geom1")), str(pair.get("geom2")))))
        for pair in root.findall("./contact/pair")
    }
    if (
        len(motion_geoms) != 2
        or len(j1_geoms) != 41
        or len(expected_explicit_pairs) != 82
        or actual_explicit_pairs != expected_explicit_pairs
    ):
        raise RuntimeError(
            "fail-closed: V15.14 component-level explicit pair set mismatch"
        )

    original_roots = {
        parent
        for path in resolved_meshes
        for parent in path.parents
        if parent.name == "mujoco_kinematic_v1"
    }
    if len(original_roots) != 1:
        raise RuntimeError(
            "fail-closed: unable to identify unique mujoco_kinematic_v1 asset root: "
            f"{sorted(map(str, original_roots))}"
        )
    original_root = original_roots.pop()
    manifest_path = original_root / "runtime_mesh_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(manifest_path)
    manifest_sha256 = sha256(manifest_path)
    if manifest_sha256 != ACCEPTED_RUNTIME_MESH_MANIFEST_SHA256:
        raise RuntimeError(
            "unaccepted V15.13 runtime mesh manifest hash: "
            f"{manifest_sha256} != {ACCEPTED_RUNTIME_MESH_MANIFEST_SHA256}"
        )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if (
        manifest.get("schema")
        != "go-m8010-arm-v15.13-mujoco-runtime-meshes/1.0"
        or manifest.get("revision") != "V15.13-MuJoCo-empty-load-kinematic-v1"
        or manifest.get("source_manifest") != "mesh_export_manifest.json"
        or str(manifest.get("source_manifest_sha256", "")).lower()
        != ACCEPTED_SOURCE_MESH_MANIFEST_SHA256
    ):
        raise RuntimeError("fail-closed: runtime mesh manifest contract mismatch")
    source_manifest_path = original_root / str(manifest["source_manifest"])
    if sha256(source_manifest_path) != ACCEPTED_SOURCE_MESH_MANIFEST_SHA256:
        raise RuntimeError("fail-closed: source mesh manifest hash mismatch")
    expected: dict[Path, str] = {}
    visual_count = 0
    collision_count = 0
    for row in manifest.get("visual", {}).values():
        for chunk in row.get("runtime_chunks", []):
            expected[(original_root / chunk["path"]).resolve()] = str(
                chunk["sha256"]
            ).lower()
            visual_count += 1
    for row in manifest.get("collision", {}).values():
        for member in row.get("members", []):
            for piece in member.get("pieces", []):
                expected[(original_root / piece["path"]).resolve()] = str(
                    piece["sha256"]
                ).lower()
                collision_count += 1
    for relative_path, digest in MOTION_PROXY_MESH_SHA256.items():
        expected[(model_path.parent / relative_path).resolve()] = digest

    actual_set = set(resolved_meshes)
    expected_set = set(expected)
    missing_from_xml = sorted(map(str, expected_set - actual_set))
    unaccepted_in_xml = sorted(map(str, actual_set - expected_set))
    if missing_from_xml or unaccepted_in_xml:
        raise RuntimeError(
            "fail-closed: MJCF mesh reference set differs from accepted manifests: "
            + json.dumps(
                {
                    "missing_from_xml": missing_from_xml,
                    "unaccepted_in_xml": unaccepted_in_xml,
                },
                ensure_ascii=False,
            )
        )

    total_bytes = 0
    stat_snapshot: dict[Path, tuple[int, int]] = {}
    asset_records = []
    mesh_node_by_path = {
        (model_path.parent / str(node.get("file"))).resolve(): node
        for node in mesh_nodes
    }
    for path in sorted(expected, key=str):
        if not path.is_file():
            raise FileNotFoundError(path)
        stat = path.stat()
        total_bytes += stat.st_size
        stat_snapshot[path] = (stat.st_size, stat.st_mtime_ns)
        actual_digest = sha256(path)
        if actual_digest != expected[path]:
            raise RuntimeError(
                f"fail-closed: runtime mesh hash mismatch: {path}: "
                f"{actual_digest} != {expected[path]}"
            )
        node = mesh_node_by_path[path]
        literal_file = str(node.get("file")).replace("\\", "/")
        if "visual_chunks_mm" in literal_file:
            role = "v15_13_visual"
            required_scale = "0.001 0.001 0.001"
        elif "collision_convex_m" in literal_file:
            role = "v15_13_collision"
            required_scale = ""
        elif "collision_motion_proxy_mm" in literal_file:
            role = "v15_14_motion"
            required_scale = "0.001 0.001 0.001"
        else:
            raise RuntimeError(f"fail-closed: unknown runtime mesh role: {literal_file}")
        scale = str(node.get("scale", ""))
        if scale != required_scale:
            raise RuntimeError(
                f"fail-closed: wrong mesh scale for {literal_file}: "
                f"{scale!r} != {required_scale!r}"
            )
        asset_records.append(
            (
                str(node.get("name")),
                literal_file,
                role,
                scale,
                stat.st_size,
                actual_digest,
            )
        )
    if total_bytes != 383_827_772:
        raise RuntimeError(
            f"fail-closed: runtime mesh byte total mismatch: {total_bytes}"
        )
    asset_set_digest = hashlib.sha256()
    asset_set_digest.update(b"go-m8010-arm-v15.14-runtime-asset-set/1.0\0")
    for name, literal_file, role, scale, size_bytes, digest in sorted(
        asset_records, key=lambda row: row[0]
    ):
        asset_set_digest.update(
            (
                name
                + "\0"
                + literal_file
                + "\0"
                + role
                + "\0"
                + scale
                + "\0"
                + str(size_bytes)
                + "\0"
                + digest
                + "\n"
            ).encode("utf-8")
        )
    asset_set_sha256 = asset_set_digest.hexdigest()
    if asset_set_sha256 != ACCEPTED_RUNTIME_ASSET_SET_SHA256:
        raise RuntimeError(
            "fail-closed: runtime asset-set digest mismatch: "
            f"{asset_set_sha256} != {ACCEPTED_RUNTIME_ASSET_SET_SHA256}"
        )
    report = {
        "pass": True,
        "runtime_manifest_path": str(manifest_path),
        "runtime_manifest_sha256": manifest_sha256,
        "mesh_count": len(expected),
        "visual_chunk_count": visual_count,
        "collision_piece_count": collision_count,
        "motion_proxy_piece_count": len(MOTION_PROXY_MESH_SHA256),
        "total_bytes": total_bytes,
        "asset_set_sha256": asset_set_sha256,
        "source_manifest_path": str(source_manifest_path),
        "source_manifest_sha256": ACCEPTED_SOURCE_MESH_MANIFEST_SHA256,
        "explicit_geom_pair_count": len(actual_explicit_pairs),
        "explicit_geom_pair_set_pass": True,
    }
    return report, stat_snapshot


def rotation_matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    """Return a normalized quaternion [x,y,z,w] from a proper 3x3 matrix."""
    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array(
            [
                (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s,
                0.25 * s,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(m)))
        if axis == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            q = np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
        elif axis == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            q = np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            q = np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s])
    q /= np.linalg.norm(q)
    if q[3] < 0.0:
        q *= -1.0
    return q


def load_guard(model_path: Path, model: mujoco.MjModel, data: mujoco.MjData):
    module_path = model_path.parent / "kinematic_guard.py"
    if not module_path.is_file():
        raise FileNotFoundError(f"required fail-closed guard not found: {module_path}")
    spec = importlib.util.spec_from_file_location("v15_13_kinematic_guard", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load guard module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.KinematicGuard(model=model, data=data)


class MujocoKinematicBridge(Node):
    def __init__(self) -> None:
        super().__init__("go_m8010_arm_mujoco_bridge")
        self.declare_parameter("model_path", "")
        self.declare_parameter("command_topic", "/mujoco/joint_commands")
        self.declare_parameter("state_topic", "/mujoco/joint_states_raw")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("guard_max_step_deg", 0.25)
        self.declare_parameter("max_velocity_rad_s", 0.5)
        self.declare_parameter("max_acceleration_rad_s2", 1.0)
        self.declare_parameter("execution_limit_abs_tolerance", 0.02)
        self.declare_parameter("velocity_consistency_tolerance_rad_s", 0.04)
        self.declare_parameter("command_timeout_s", 0.1)
        self.declare_parameter("prime_position_tolerance_rad", 1.0e-6)
        self.declare_parameter("prime_velocity_tolerance_rad_s", 1.0e-6)
        self.declare_parameter("latch_fault", True)
        self.declare_parameter("use_viewer", False)

        model_path_text = str(self.get_parameter("model_path").value)
        if not model_path_text:
            raise RuntimeError("model_path is required")
        self.model_path = Path(model_path_text).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(self.model_path)
        self.model_sha256 = sha256(self.model_path)
        if self.model_sha256 != ACCEPTED_MODEL_SHA256:
            raise RuntimeError(
                "unaccepted V15.14 MJCF hash: "
                f"{self.model_sha256} != {ACCEPTED_MODEL_SHA256}"
            )
        self.collision_contract_path = (
            self.model_path.parent / "collision_pair_contract_v15_14.json"
        )
        if not self.collision_contract_path.is_file():
            raise FileNotFoundError(
                "required V15.14 collision contract not found: "
                f"{self.collision_contract_path}"
            )
        self.collision_contract_sha256 = sha256(self.collision_contract_path)
        if (
            self.collision_contract_sha256
            != ACCEPTED_COLLISION_CONTRACT_SHA256
        ):
            raise RuntimeError(
                "unaccepted V15.14 collision contract hash: "
                f"{self.collision_contract_sha256} != "
                f"{ACCEPTED_COLLISION_CONTRACT_SHA256}"
            )
        self.collision_contract = json.loads(
            self.collision_contract_path.read_text(encoding="utf-8")
        )
        expected_contract = {
            "schema": "go-m8010-arm-v15.14-self-collision-pair-contract/2.0",
            "proxy_count": 25,
            "all_unordered_pair_count": 300,
            "runtime_full_pair_count": 231,
            "runtime_excluded_pair_count": 69,
        }
        contract_mismatch = {
            key: {
                "expected": expected,
                "actual": self.collision_contract.get(key),
            }
            for key, expected in expected_contract.items()
            if self.collision_contract.get(key) != expected
        }
        if contract_mismatch:
            raise RuntimeError(
                "V15.14 collision contract mismatch: "
                + json.dumps(contract_mismatch, ensure_ascii=False)
            )
        self.guard_path = self.model_path.parent / "kinematic_guard.py"
        if not self.guard_path.is_file():
            raise FileNotFoundError(self.guard_path)
        self.guard_sha256 = sha256(self.guard_path)
        if self.guard_sha256 != ACCEPTED_GUARD_SHA256:
            raise RuntimeError(
                f"unaccepted V15.14 kinematic guard hash: {self.guard_sha256}"
            )

        (
            self.runtime_mesh_integrity,
            runtime_mesh_stat_snapshot,
        ) = verify_runtime_mesh_assets(self.model_path)

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        changed_during_load = [
            str(path)
            for path, snapshot in runtime_mesh_stat_snapshot.items()
            if not path.is_file()
            or (path.stat().st_size, path.stat().st_mtime_ns) != snapshot
        ]
        if changed_during_load:
            raise RuntimeError(
                "fail-closed: runtime mesh changed while MuJoCo loaded it: "
                + json.dumps(changed_during_load, ensure_ascii=False)
            )
        # The accepted collision contract checks two directly connected-link
        # pairs. Disable MuJoCo's broad parent filter, then let KinematicGuard's
        # exact enabled/excluded pair matrix decide which contacts are valid.
        self.model.opt.disableflags |= int(
            mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
        )
        self.data = mujoco.MjData(self.model)
        self.guard_data = mujoco.MjData(self.model)
        self.guard = load_guard(self.model_path, self.model, self.guard_data)
        self.guard_max_step_deg = float(self.get_parameter("guard_max_step_deg").value)
        self.max_velocity_rad_s = float(
            self.get_parameter("max_velocity_rad_s").value
        )
        self.max_acceleration_rad_s2 = float(
            self.get_parameter("max_acceleration_rad_s2").value
        )
        self.execution_limit_abs_tolerance = float(
            self.get_parameter("execution_limit_abs_tolerance").value
        )
        self.velocity_consistency_tolerance_rad_s = float(
            self.get_parameter("velocity_consistency_tolerance_rad_s").value
        )
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.prime_position_tolerance_rad = float(
            self.get_parameter("prime_position_tolerance_rad").value
        )
        self.prime_velocity_tolerance_rad_s = float(
            self.get_parameter("prime_velocity_tolerance_rad_s").value
        )
        if (
            not math.isfinite(self.max_velocity_rad_s)
            or self.max_velocity_rad_s <= 0.0
            or not math.isfinite(self.max_acceleration_rad_s2)
            or self.max_acceleration_rad_s2 <= 0.0
            or not math.isfinite(self.execution_limit_abs_tolerance)
            or self.execution_limit_abs_tolerance < 0.0
            or not math.isfinite(self.velocity_consistency_tolerance_rad_s)
            or abs(self.velocity_consistency_tolerance_rad_s - 0.04) > 1.0e-12
            or not math.isfinite(self.command_timeout_s)
            or self.command_timeout_s <= 0.0
            or not math.isfinite(self.prime_position_tolerance_rad)
            or self.prime_position_tolerance_rad < 0.0
            or not math.isfinite(self.prime_velocity_tolerance_rad_s)
            or self.prime_velocity_tolerance_rad_s < 0.0
            or abs(self.guard_max_step_deg - 0.25) > 1.0e-12
        ):
            raise ValueError("invalid simulation execution envelope")
        self.latch_fault = bool(self.get_parameter("latch_fault").value)
        if not self.latch_fault:
            raise ValueError("V15.14 requires fail-closed latch_fault=true")
        self.lock = threading.RLock()

        self.joint_ids: list[int] = []
        self.qpos_addresses: list[int] = []
        self.qvel_addresses: list[int] = []
        for name in JOINTS:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"MJCF joint missing: {name}")
            self.joint_ids.append(joint_id)
            self.qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
            self.qvel_addresses.append(int(self.model.jnt_dofadr[joint_id]))

        self.tcp_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp_nominal"
        )
        if self.tcp_site_id < 0:
            raise RuntimeError("MJCF site tcp_nominal is required")

        mujoco.mj_forward(self.model, self.data)
        self.current_position = np.array(
            [self.data.qpos[address] for address in self.qpos_addresses], dtype=float
        )
        self.current_velocity = np.zeros(6, dtype=float)
        self.last_command_time_ns: int | None = None
        self.last_command_receipt_monotonic_ns: int | None = None
        self.command_stream_primed = False
        self.max_observed_command_velocity_rad_s = 0.0
        self.max_observed_command_acceleration_rad_s2 = 0.0
        self.stationary_command_gap_count = 0
        self.command_motion_active = False
        self.stationary_confirmation_count = 0
        self.moving_watchdog_timeout_count = 0
        self.accepted_command_count = 0
        self.accepted_moving_command_count = 0
        self.rejected_command_count = 0
        self.max_source_receipt_dt_difference_s = 0.0
        self.max_velocity_consistency_error_rad_s = 0.0
        self.max_command_source_age_s = 0.0
        self.dropped_stale_stationary_command_count = 0
        self.stationary_guard_reuse_count = 0
        # J1 is the only continuous joint in the frozen V15.13 URDF. MoveIt
        # and JTC may represent one physical angle on a neighbouring 2*pi
        # branch. Normalize each J1 command to the equivalent angle nearest the
        # authoritative MuJoCo state before derivative, swept-collision, and
        # qpos calculations. This changes representation only, never geometry.
        self.j1_continuous_branch_normalization_enabled = True
        self.j1_branch_adjustment_count = 0
        self.max_abs_j1_branch_adjustment_rad = 0.0
        self.fault_latched = False
        self.last_guard_result: dict = {"safe": True, "reason": "startup"}

        command_topic = str(self.get_parameter("command_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self.state_publisher = self.create_publisher(JointState, state_topic, 10)
        self.status_publisher = self.create_publisher(String, "/mujoco_bridge/status", 10)
        self.tcp_publisher = self.create_publisher(
            PoseStamped, "/mujoco_bridge/tcp_nominal_pose", 10
        )
        self.command_subscription = self.create_subscription(
            # Only the newest controller sample is useful to a kinematic
            # executor.  KeepLast(1) prevents a transient ROS callback backlog
            # from replaying stale motion commands; the swept guard validates
            # the full current-to-latest interval before accepting the update.
            JointState, command_topic, self.command_callback, 1
        )
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if (
            not math.isfinite(self.publish_rate_hz)
            or abs(self.publish_rate_hz - 50.0) > 1.0e-12
        ):
            raise ValueError("V15.14 requires publish_rate_hz=50")
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_state)
        self.get_logger().info(
            "MuJoCo kinematic bridge ready; FollowJointTrajectory remains owned by "
            f"joint_trajectory_controller; model={self.model_path}"
        )

    def ordered_command(self, message: JointState) -> tuple[np.ndarray, np.ndarray]:
        if len(message.name) != len(set(message.name)):
            raise ValueError("duplicate joint names")
        if set(message.name) != set(JOINTS):
            raise ValueError(f"expected exactly {JOINTS}, got {tuple(message.name)}")
        if len(message.position) != 6:
            raise ValueError("position array must contain six values")
        if len(message.velocity) != 6:
            raise ValueError("velocity array must contain six values")
        position_source = dict(zip(message.name, message.position))
        velocity_source = dict(zip(message.name, message.velocity))
        positions = np.array(
            [position_source[name] for name in JOINTS], dtype=float
        )
        velocities = np.array(
            [velocity_source[name] for name in JOINTS], dtype=float
        )
        if not np.all(np.isfinite(positions)):
            raise ValueError("non-finite position command")
        if not np.all(np.isfinite(velocities)):
            raise ValueError("non-finite velocity command")
        return positions, velocities

    def reject(self, reason: str, details: object) -> None:
        self.rejected_command_count += 1
        self.current_velocity[:] = 0.0
        self.data.qvel[self.qvel_addresses] = 0.0
        self.fault_latched = self.latch_fault or self.fault_latched
        self.last_guard_result = {
            "safe": False,
            "reason": reason,
            "details": details,
            "fault_latched": self.fault_latched,
        }
        self.get_logger().error(json.dumps(self.last_guard_result, ensure_ascii=False))

    def command_callback(self, message: JointState) -> None:
        receipt_now_ns = time.monotonic_ns()
        with self.lock:
            if self.fault_latched:
                return
            try:
                target, commanded_velocity = self.ordered_command(message)
                raw_j1_target = float(target[0])
                raw_j1_delta = raw_j1_target - float(self.current_position[0])
                wrapped_j1_delta = math.atan2(
                    math.sin(raw_j1_delta), math.cos(raw_j1_delta)
                )
                # Mirror JointTrajectoryController's deterministic direction
                # choice at the +/-pi singularity.
                if abs(abs(wrapped_j1_delta) - math.pi) < 1.0e-9:
                    wrapped_j1_delta = (
                        abs(wrapped_j1_delta)
                        if raw_j1_target > float(self.current_position[0])
                        else -abs(wrapped_j1_delta)
                    )
                target[0] = float(self.current_position[0]) + wrapped_j1_delta
                j1_branch_adjustment = float(target[0] - raw_j1_target)
                command_position_delta = float(
                    np.max(np.abs(target - self.current_position))
                )
                message_time_ns = (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                )
                if message_time_ns <= 0:
                    raise ValueError("command timestamp must be positive")
                source_age_s = (
                    self.get_clock().now().nanoseconds - message_time_ns
                ) * 1.0e-9
                source_command_is_stationary = bool(
                    command_position_delta <= self.prime_position_tolerance_rad
                    and np.max(np.abs(commanded_velocity))
                    <= self.prime_velocity_tolerance_rad_s
                    and np.max(np.abs(self.current_velocity))
                    <= self.prime_velocity_tolerance_rad_s
                )
                if not math.isfinite(source_age_s) or source_age_s < -1.0e-3:
                    raise ValueError(
                        "stale or future command timestamp: "
                        f"source_age_s={source_age_s}"
                    )
                if source_age_s > self.command_timeout_s:
                    # A delayed heartbeat cannot move the model.  Dropping it
                    # avoids a false latched fault while retaining fail-closed
                    # rejection for every delayed motion command.
                    if source_command_is_stationary:
                        self.dropped_stale_stationary_command_count += 1
                        return
                    raise ValueError(
                        "stale moving command timestamp: "
                        f"source_age_s={source_age_s}"
                    )
                self.max_command_source_age_s = max(
                    self.max_command_source_age_s, max(0.0, source_age_s)
                )
                if not self.command_stream_primed:
                    prime_position_error = float(
                        np.max(np.abs(target - self.current_position))
                    )
                    prime_velocity = float(np.max(np.abs(commanded_velocity)))
                    if (
                        prime_position_error > self.prime_position_tolerance_rad
                        or prime_velocity > self.prime_velocity_tolerance_rad_s
                    ):
                        self.get_logger().warning(
                            "holding zero state while waiting for a valid command "
                            f"prime: position_error={prime_position_error}, "
                            f"velocity={prime_velocity}"
                        )
                        return
                    velocity = commanded_velocity
                    acceleration = np.zeros(6, dtype=float)
                    self.command_stream_primed = True
                else:
                    source_dt = (
                        message_time_ns - self.last_command_time_ns
                    ) * 1.0e-9
                    receipt_dt = (
                        receipt_now_ns - self.last_command_receipt_monotonic_ns
                    ) * 1.0e-9
                    if (
                        not math.isfinite(source_dt)
                        or not math.isfinite(receipt_dt)
                        or source_dt <= 1.0e-6
                        or receipt_dt <= 1.0e-6
                    ):
                        raise ValueError(
                            "invalid command interval: "
                            f"source_dt={source_dt}, receipt_dt={receipt_dt}"
                        )
                    self.max_source_receipt_dt_difference_s = max(
                        self.max_source_receipt_dt_difference_s,
                        abs(source_dt - receipt_dt),
                    )
                    if (
                        source_dt > self.command_timeout_s
                        or receipt_dt > self.command_timeout_s
                    ):
                        stationary_gap = bool(
                            np.max(np.abs(target - self.current_position))
                            <= self.prime_position_tolerance_rad
                            and np.max(np.abs(commanded_velocity))
                            <= self.prime_velocity_tolerance_rad_s
                            and np.max(np.abs(self.current_velocity))
                            <= self.prime_velocity_tolerance_rad_s
                        )
                        if not stationary_gap:
                            raise ValueError(
                                "moving command stream timeout: "
                                f"source_dt={source_dt}, receipt_dt={receipt_dt}"
                            )
                        self.stationary_command_gap_count += 1
                        velocity = commanded_velocity
                        acceleration = np.zeros(6, dtype=float)
                    else:
                        source_finite_difference_velocity = (
                            target - self.current_position
                        ) / source_dt
                        velocity = commanded_velocity
                        acceleration = (
                            velocity - self.current_velocity
                        ) / source_dt
                        trapezoidal_velocity = 0.5 * (
                            self.current_velocity + velocity
                        )
                        velocity_consistency_error = float(
                            np.max(
                                np.abs(
                                    source_finite_difference_velocity
                                    - trapezoidal_velocity
                                )
                            )
                        )
                        velocity_consistency_tolerance = (
                            self.velocity_consistency_tolerance_rad_s
                            + (
                                self.max_acceleration_rad_s2
                                + self.execution_limit_abs_tolerance
                            )
                            * source_dt
                        )
                        if (
                            velocity_consistency_error
                            > velocity_consistency_tolerance
                        ):
                            raise ValueError(
                                "position/velocity command inconsistency: "
                                f"{velocity_consistency_error} > "
                                f"{velocity_consistency_tolerance}"
                            )
                        self.max_velocity_consistency_error_rad_s = max(
                            self.max_velocity_consistency_error_rad_s,
                            velocity_consistency_error,
                        )
                        observed_velocity = max(
                            float(np.max(np.abs(velocity))),
                            float(
                                np.max(
                                    np.abs(source_finite_difference_velocity)
                                )
                            ),
                        )
                        observed_acceleration = float(np.max(np.abs(acceleration)))
                        if observed_velocity > (
                            self.max_velocity_rad_s
                            + self.execution_limit_abs_tolerance
                        ):
                            raise ValueError(
                                "simulation velocity envelope exceeded: "
                                f"{observed_velocity} > {self.max_velocity_rad_s}"
                            )
                        if observed_acceleration > (
                            self.max_acceleration_rad_s2
                            + self.execution_limit_abs_tolerance
                        ):
                            raise ValueError(
                                "simulation acceleration envelope exceeded: "
                                f"{observed_acceleration} > {self.max_acceleration_rad_s2}"
                            )
                        self.max_observed_command_velocity_rad_s = max(
                            self.max_observed_command_velocity_rad_s,
                            observed_velocity,
                        )
                        self.max_observed_command_acceleration_rad_s2 = max(
                            self.max_observed_command_acceleration_rad_s2,
                            observed_acceleration,
                        )
                if (
                    self.command_stream_primed
                    and self.accepted_command_count > 0
                    and source_command_is_stationary
                    and self.last_guard_result.get("safe") is True
                ):
                    result = {
                        "safe": True,
                        "reason": "accepted_stationary_heartbeat",
                        "steps": 0,
                    }
                    self.stationary_guard_reuse_count += 1
                else:
                    result = self.guard.check_swept_deg(
                        np.degrees(self.current_position),
                        np.degrees(target),
                        max_step_deg=self.guard_max_step_deg,
                    )
                if not result["safe"]:
                    self.reject("guard_rejected_command", result)
                    return

                self.current_position[:] = target
                self.current_velocity[:] = velocity
                command_is_moving = bool(
                    command_position_delta > self.prime_position_tolerance_rad
                    or np.max(np.abs(velocity))
                    > self.prime_velocity_tolerance_rad_s
                )
                if command_is_moving:
                    self.command_motion_active = True
                    self.stationary_confirmation_count = 0
                    self.accepted_moving_command_count += 1
                else:
                    self.stationary_confirmation_count += 1
                    if self.stationary_confirmation_count >= 3:
                        self.command_motion_active = False
                self.data.qpos[self.qpos_addresses] = target
                self.data.qvel[self.qvel_addresses] = velocity
                mujoco.mj_forward(self.model, self.data)
                self.last_command_time_ns = message_time_ns
                self.last_command_receipt_monotonic_ns = receipt_now_ns
                self.accepted_command_count += 1
                if abs(j1_branch_adjustment) > 1.0e-12:
                    self.j1_branch_adjustment_count += 1
                    self.max_abs_j1_branch_adjustment_rad = max(
                        self.max_abs_j1_branch_adjustment_rad,
                        abs(j1_branch_adjustment),
                    )
                self.last_guard_result = result
                # Publish the authoritative MuJoCo state immediately after an
                # accepted command; the timer remains as the stationary
                # heartbeat. This minimizes TopicBasedSystem/JSB pipeline lag.
                self.publish_state()
            except Exception as error:  # fail closed at the ROS boundary
                self.reject("invalid_command", repr(error))

    def publish_state(self) -> None:
        with self.lock:
            command_age_s = None
            if self.last_command_receipt_monotonic_ns is not None:
                command_age_s = (
                    time.monotonic_ns() - self.last_command_receipt_monotonic_ns
                ) * 1.0e-9
            if (
                self.command_stream_primed
                and not self.fault_latched
                and command_age_s is not None
                and command_age_s > self.command_timeout_s
                and self.command_motion_active
            ):
                self.moving_watchdog_timeout_count += 1
                self.reject(
                    "moving_command_watchdog_timeout",
                    {"command_age_s": command_age_s},
                )
            stamp = self.get_clock().now().to_msg()
            message = JointState()
            message.header.stamp = stamp
            message.name = list(JOINTS)
            message.position = self.current_position.tolist()
            message.velocity = self.current_velocity.tolist()
            self.state_publisher.publish(message)

            tcp = PoseStamped()
            tcp.header.stamp = stamp
            tcp.header.frame_id = "world"
            tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z = map(
                float, self.data.site_xpos[self.tcp_site_id]
            )
            quaternion = rotation_matrix_to_quaternion_xyzw(
                self.data.site_xmat[self.tcp_site_id]
            )
            tcp.pose.orientation.x, tcp.pose.orientation.y, tcp.pose.orientation.z, tcp.pose.orientation.w = map(float, quaternion)
            self.tcp_publisher.publish(tcp)

            status = {
                "schema": "go-m8010-arm-v15.14-mujoco-bridge-status/1.0",
                "execution_mode": "kinematic_position_tracking",
                "dynamics_valid": False,
                "parent_collision_filter_disabled": True,
                "model": {
                    "path": str(self.model_path),
                    "sha256": self.model_sha256,
                },
                "collision_contract": {
                    "path": str(self.collision_contract_path),
                    "sha256": self.collision_contract_sha256,
                    "schema": self.collision_contract.get("schema"),
                    "revision": self.collision_contract.get("revision"),
                    "proxy_count": self.collision_contract.get("proxy_count"),
                    "all_unordered_pair_count": self.collision_contract.get(
                        "all_unordered_pair_count"
                    ),
                    "runtime_full_pair_count": self.collision_contract.get(
                        "runtime_full_pair_count"
                    ),
                    "runtime_excluded_pair_count": self.collision_contract.get(
                        "runtime_excluded_pair_count"
                    ),
                },
                "kinematic_guard": {
                    "path": str(self.guard_path),
                    "sha256": self.guard_sha256,
                },
                "runtime_mesh_integrity": self.runtime_mesh_integrity,
                "simulation_execution_envelope": {
                    "max_velocity_rad_s": self.max_velocity_rad_s,
                    "max_acceleration_rad_s2": self.max_acceleration_rad_s2,
                    "publish_rate_hz": self.publish_rate_hz,
                    "absolute_calculation_tolerance": self.execution_limit_abs_tolerance,
                    "velocity_consistency_tolerance_rad_s": self.velocity_consistency_tolerance_rad_s,
                    "command_timeout_s": self.command_timeout_s,
                    "prime_position_tolerance_rad": self.prime_position_tolerance_rad,
                    "prime_velocity_tolerance_rad_s": self.prime_velocity_tolerance_rad_s,
                    "guard_max_step_deg": self.guard_max_step_deg,
                    "max_observed_command_velocity_rad_s": self.max_observed_command_velocity_rad_s,
                    "max_observed_command_acceleration_rad_s2": self.max_observed_command_acceleration_rad_s2,
                    "stationary_command_gap_count": self.stationary_command_gap_count,
                    "max_source_receipt_dt_difference_s": self.max_source_receipt_dt_difference_s,
                    "max_velocity_consistency_error_rad_s": self.max_velocity_consistency_error_rad_s,
                    "max_command_source_age_s": self.max_command_source_age_s,
                    "dropped_stale_stationary_command_count": self.dropped_stale_stationary_command_count,
                    "stationary_guard_reuse_count": self.stationary_guard_reuse_count,
                },
                "fault_latched": self.fault_latched,
                "latch_fault_enabled": self.latch_fault,
                "command_stream_primed": self.command_stream_primed,
                "command_subscription_history": "keep_last",
                "command_subscription_depth": 1,
                "j1_continuous_branch_normalization": {
                    "enabled": self.j1_continuous_branch_normalization_enabled,
                    "joint_name": "J1",
                    "method": "nearest_equivalent_to_current",
                    "pi_tie_break": "match_joint_trajectory_controller",
                    "accepted_adjustment_count": self.j1_branch_adjustment_count,
                    "max_abs_adjustment_rad": self.max_abs_j1_branch_adjustment_rad,
                },
                "last_command_receipt_age_s": command_age_s,
                "moving_command_watchdog_active": True,
                "command_motion_active": self.command_motion_active,
                "stationary_confirmation_count": self.stationary_confirmation_count,
                "moving_watchdog_timeout_count": self.moving_watchdog_timeout_count,
                "accepted_command_count": self.accepted_command_count,
                "accepted_moving_command_count": self.accepted_moving_command_count,
                "rejected_command_count": self.rejected_command_count,
                "last_guard_result": self.last_guard_result,
                "joint_position_rad": self.current_position.tolist(),
                "joint_velocity_rad_s": self.current_velocity.tolist(),
            }
            self.status_publisher.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node: MujocoKinematicBridge | None = None
    viewer = None
    try:
        node = MujocoKinematicBridge()
        if bool(node.get_parameter("use_viewer").value):
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(node.model, node.data)
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(node, timeout_sec=0.01)
                viewer.sync()
        else:
            rclpy.spin(node)
    finally:
        if viewer is not None:
            viewer.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
