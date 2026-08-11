from __future__ import annotations

"""Independent, read-only QA for the V15 measured joint-axis handoff."""

import hashlib
import importlib.util
import json
import math
from pathlib import Path

import FreeCAD as App


SOURCE = Path(
    r"D:/AI_JIXIEBI/备份/机械臂完整工程_V14_交接备份_20260807_164438/"
    r"V14_正式工程/机械臂完整装配_六轴_舵机柔性二指夹爪_v14.FCStd"
)
OUTPUT_ROOT = Path(r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15")
V15 = OUTPUT_ROOT / "机械臂完整装配_六轴_真实关节轴_v15.FCStd"
AXIS_JSON = OUTPUT_ROOT / "V15_真实关节轴定义.json"
REPORT = OUTPUT_ROOT / "QA_V15_独立复核.json"
GUARD = Path(r"D:/AI_JIXIEBI/go_m8010_arm_gui/j123456_gripper_physical_collision_guard_v14.py")
EXPECTED_SOURCE_HASH = "9156F6EC5C402ECA00FE0388E7795C4CB293481C6716997E866289C97FE5CE6A"
JOINTS = [f"J{index}_Revolute" for index in range(1, 7)]
CORE_PROPERTIES = (
    "Angle",
    "AngleMin",
    "AngleMax",
    "EnableAngleMin",
    "EnableAngleMax",
    "JointType",
    "Offset1",
    "Offset2",
    "Placement1",
    "Placement2",
    "Suppressed",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def number(value) -> float:
    return float(value.Value) if hasattr(value, "Value") else float(value)


def signature(value):
    if isinstance(value, App.Placement):
        q = value.Rotation.Q
        return [value.Base.x, value.Base.y, value.Base.z, q[0], q[1], q[2], q[3]]
    if isinstance(value, App.Vector):
        return [value.x, value.y, value.z]
    if hasattr(value, "Value"):
        return float(value.Value)
    if isinstance(value, (tuple, list)):
        return [signature(item) for item in value]
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    return str(value)


def equal(left, right, tolerance: float = 1.0e-12) -> bool:
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(equal(a, b, tolerance) for a, b in zip(left, right))
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return abs(float(left) - float(right)) <= tolerance
    return left == right


def brep_text(obj) -> str | None:
    if not hasattr(obj, "Shape") or obj.Shape.isNull():
        return None
    return obj.Shape.exportBrepToString()


def compare_brep_serialization(left: str, right: str) -> dict:
    left_lines = left.splitlines()
    right_lines = right.splitlines()
    if len(left_lines) != len(right_lines):
        return {"equivalent": False, "reason": "line_count", "max_abs": None, "max_scaled": None}
    max_abs = 0.0
    max_scaled = 0.0
    differing_lines = 0
    for line_index, (left_line, right_line) in enumerate(zip(left_lines, right_lines)):
        if left_line == right_line:
            continue
        differing_lines += 1
        left_tokens = left_line.split()
        right_tokens = right_line.split()
        if len(left_tokens) != len(right_tokens):
            return {
                "equivalent": False,
                "reason": f"token_count_at_line_{line_index + 1}",
                "max_abs": max_abs,
                "max_scaled": max_scaled,
            }
        for token_index, (left_token, right_token) in enumerate(zip(left_tokens, right_tokens)):
            if left_token == right_token:
                continue
            try:
                left_value = float(left_token)
                right_value = float(right_token)
            except ValueError:
                return {
                    "equivalent": False,
                    "reason": f"topology_token_at_line_{line_index + 1}_token_{token_index + 1}",
                    "max_abs": max_abs,
                    "max_scaled": max_scaled,
                }
            absolute = abs(left_value - right_value)
            scale = max(1.0, abs(left_value), abs(right_value))
            scaled = absolute / scale
            max_abs = max(max_abs, absolute)
            max_scaled = max(max_scaled, scaled)
            if absolute > 1.0e-12 and scaled > 1.0e-13:
                return {
                    "equivalent": False,
                    "reason": f"numeric_geometry_at_line_{line_index + 1}_token_{token_index + 1}",
                    "max_abs": max_abs,
                    "max_scaled": max_scaled,
                }
    return {
        "equivalent": True,
        "reason": "exact" if differing_lines == 0 else "floating_point_reserialization_only",
        "differing_lines": differing_lines,
        "max_abs": max_abs,
        "max_scaled": max_scaled,
    }


def dot(a, b) -> float:
    return sum(float(x) * float(y) for x, y in zip(a, b))


def sub(a, b) -> list[float]:
    return [float(x) - float(y) for x, y in zip(a, b)]


def norm(a) -> float:
    return math.sqrt(dot(a, a))


def matrix_error(left, right) -> float:
    return max(abs(float(left[r][c]) - float(right[r][c])) for r in range(3) for c in range(3))


def rpy_matrix(rpy) -> list[list[float]]:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def quaternion_matrix_xyzw(quaternion) -> list[list[float]]:
    x, y, z, w = (float(value) for value in quaternion)
    scale = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / scale, y / scale, z / scale, w / scale
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def frame_basis(frame: dict) -> list[list[float]]:
    return [
        frame["x_axis_world_at_zero"],
        frame["y_axis_world_at_zero"],
        frame["z_axis_world_at_zero"],
    ]


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location("v15_independent_guard", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> dict:
    source_hash_before = sha256_file(SOURCE)
    v15_hash = sha256_file(V15)
    data = json.loads(AXIS_JSON.read_text(encoding="utf-8"))

    source_doc = App.openDocument(str(SOURCE))
    v15_doc = App.openDocument(str(V15))
    try:
        source_objects = {obj.Name: obj for obj in source_doc.Objects}
        v15_objects = {obj.Name: obj for obj in v15_doc.Objects}
        missing_objects = sorted(set(source_objects) - set(v15_objects))
        type_mismatches = sorted(
            name for name in source_objects if name in v15_objects and source_objects[name].TypeId != v15_objects[name].TypeId
        )

        shape_hash_mismatches = []
        shape_serialization_geometry_mismatches = []
        shape_reserialization_max_abs = 0.0
        shape_reserialization_max_scaled = 0.0
        shape_count = 0
        for name, source_obj in source_objects.items():
            source_brep = brep_text(source_obj)
            if source_brep is None:
                continue
            shape_count += 1
            target_brep = brep_text(v15_objects[name]) if name in v15_objects else None
            source_shape_hash = hashlib.sha256(source_brep.encode("utf-8")).hexdigest().upper()
            target_shape_hash = (
                hashlib.sha256(target_brep.encode("utf-8")).hexdigest().upper()
                if target_brep is not None
                else None
            )
            if source_shape_hash != target_shape_hash:
                shape_hash_mismatches.append(name)
                comparison = compare_brep_serialization(source_brep, target_brep or "")
                if not comparison["equivalent"]:
                    shape_serialization_geometry_mismatches.append({"object": name, **comparison})
                else:
                    shape_reserialization_max_abs = max(
                        shape_reserialization_max_abs,
                        float(comparison["max_abs"] or 0.0),
                    )
                    shape_reserialization_max_scaled = max(
                        shape_reserialization_max_scaled,
                        float(comparison["max_scaled"] or 0.0),
                    )

        joint_property_mismatches = []
        for joint_name in JOINTS:
            source_joint = source_objects[joint_name]
            target_joint = v15_objects[joint_name]
            for property_name in CORE_PROPERTIES:
                left = signature(getattr(source_joint, property_name))
                right = signature(getattr(target_joint, property_name))
                if not equal(left, right):
                    joint_property_mismatches.append(f"{joint_name}.{property_name}")

        frames = data["frames_at_mechanical_zero"]
        transform_errors = {
            "translation_mm": 0.0,
            "rotation_matrix": 0.0,
            "rpy_rotation_matrix": 0.0,
            "quaternion_rotation_matrix": 0.0,
            "axis_world_reconstruction": 0.0,
            "child_origin_to_joint_center_mm": 0.0,
        }
        canonical = {"X": [1.0, 0.0, 0.0], "Y": [0.0, 1.0, 0.0], "Z": [0.0, 0.0, 1.0]}
        canonical_errors = []
        for row in data["joints"]:
            parent = frames[row["parent_link"]]
            child = frames[row["child_link"]]
            parent_axes = frame_basis(parent)
            child_axes = frame_basis(child)
            delta = sub(child["origin_world_mm"], parent["origin_world_mm"])
            expected_translation = [dot(delta, axis) for axis in parent_axes]
            expected_rotation = [[dot(parent_axes[r], child_axes[c]) for c in range(3)] for r in range(3)]
            transform = row["parent_to_child_at_mechanical_zero"]
            transform_errors["translation_mm"] = max(
                transform_errors["translation_mm"],
                norm(sub(expected_translation, transform["xyz_mm"])),
            )
            transform_errors["rotation_matrix"] = max(
                transform_errors["rotation_matrix"],
                matrix_error(expected_rotation, transform["rotation_matrix_row_major"]),
            )
            transform_errors["rpy_rotation_matrix"] = max(
                transform_errors["rpy_rotation_matrix"],
                matrix_error(expected_rotation, rpy_matrix(transform["rpy_rad_urdf_fixed_axis"])),
            )
            transform_errors["quaternion_rotation_matrix"] = max(
                transform_errors["quaternion_rotation_matrix"],
                matrix_error(expected_rotation, quaternion_matrix_xyzw(transform["quaternion_xyzw"])),
            )
            local_axis = row["axis_vector_in_joint_child_frame"]
            reconstructed_world_axis = [
                sum(local_axis[index] * child_axes[index][component] for index in range(3))
                for component in range(3)
            ]
            transform_errors["axis_world_reconstruction"] = max(
                transform_errors["axis_world_reconstruction"],
                norm(sub(reconstructed_world_axis, row["axis_world_at_zero"])),
            )
            transform_errors["child_origin_to_joint_center_mm"] = max(
                transform_errors["child_origin_to_joint_center_mm"],
                norm(sub(child["origin_world_mm"], row["joint_origin_world_mm"])),
            )
            if norm(sub(row["axis_vector_in_parent_link_frame"], canonical[row["canonical_axis"]])) > 1.0e-9:
                canonical_errors.append(f"{row['name']}:parent")
            if norm(sub(local_axis, canonical[row["canonical_axis"]])) > 1.0e-9:
                canonical_errors.append(f"{row['name']}:joint")

        chain = data["downstream_kinematic_chain"]
        chain_names = [row["joint_name"] for row in chain]
        j2_rows = [row for row in chain if row["joint_name"] == "J2"]
        downstream_topology_ok = (
            chain_names == ["J1", "J2", "J3", "J4", "J5", "J6"]
            and len(j2_rows) == 1
            and j2_rows[0]["physical_actuators"] == ["J2A", "J2B"]
        )
        chain_by_name = {row["joint_name"]: row for row in chain}
        limit_contract_errors = []
        for index in range(1, 7):
            joint_name = f"J{index}"
            source_joint = source_objects[f"{joint_name}_Revolute"]
            limited = bool(source_joint.EnableAngleMin) and bool(source_joint.EnableAngleMax)
            expected_type = "revolute" if limited else "continuous"
            expected_range_deg = (
                [number(source_joint.AngleMin), number(source_joint.AngleMax)]
                if limited
                else None
            )
            row = chain_by_name[joint_name]
            if row["ros2_urdf"]["joint_type"] != expected_type:
                limit_contract_errors.append(f"{joint_name}:ros2_type")
            actual_limit = row["joint_limit_definition"]["position_limit_deg"]
            actual_range_deg = (
                [actual_limit["lower"], actual_limit["upper"]]
                if actual_limit is not None
                else None
            )
            if not equal(actual_range_deg, expected_range_deg, 1.0e-12):
                limit_contract_errors.append(f"{joint_name}:position_range")
            mujoco = row["mujoco_mjcf"]
            if mujoco["joint_limited"] != limited:
                limit_contract_errors.append(f"{joint_name}:mujoco_limited")
            expected_range_rad = (
                [math.radians(value) for value in expected_range_deg]
                if expected_range_deg is not None
                else None
            )
            if not equal(mujoco["joint_range_rad"], expected_range_rad, 1.0e-12):
                limit_contract_errors.append(f"{joint_name}:mujoco_range")
            if row["joint_limit_definition"]["velocity_limit_rad_s"] is not None:
                limit_contract_errors.append(f"{joint_name}:guessed_velocity")
            if row["joint_limit_definition"]["effort_limit_nm"] is not None:
                limit_contract_errors.append(f"{joint_name}:guessed_effort")
        j6_limit = chain_by_name["J6"]["joint_limit_definition"]
        j6_revolute_pm180_ok = (
            chain_by_name["J6"]["ros2_urdf"]["joint_type"] == "revolute"
            and j6_limit["position_limit_deg"] == {"lower": -180.0, "upper": 180.0}
            and equal(chain_by_name["J6"]["mujoco_mjcf"]["joint_range_rad"], [-math.pi, math.pi], 1.0e-12)
        )

        end_effector = data["j6_end_effector_interface"]
        mount = end_effector["j6_output_mount_frame"]
        seat = end_effector["connector_seat"]
        hole_interface = end_effector["six_m4_interface"]
        stator_mount = end_effector["three_m4_stator_mount"]
        clevis = end_effector["clevis_tongue_interface"]
        fixed_contract = end_effector["fixed_joint_contract"]
        ee_record = v15_doc.getObject("V15_J6_End_Effector_Interface")
        end_effector_contract_ok = (
            ee_record is not None
            and mount["origin_to_j6_joint_center_error_mm"] <= 1.0e-9
            and mount["axis_to_j6_error_deg"] <= 1.0e-9
            and abs(seat["contact_plane_brep_witness"]["axial_station_mm"]) <= 1.0e-9
            and seat["contact_plane_brep_witness"]["axis_normal_alignment_abs_dot"] >= 1.0 - 1.0e-12
            and hole_interface["maximum_connector_to_dm_hole_axis_error_mm"] > 0.0
            and abs(
                number(ee_record.MaxSixM4AxisErrorMM)
                - hole_interface["maximum_connector_to_dm_hole_axis_error_mm"]
            ) <= 1.0e-12
            and abs(number(ee_record.MaxThreeM4StatorAxisErrorMM) - stator_mount["maximum_axis_error_mm"]) <= 1.0e-12
            and abs(number(ee_record.MaxClevisCrossAxisErrorMM) - clevis["maximum_cross_axis_error_mm"]) <= 1.0e-12
            and abs(number(ee_record.ClevisRadialClearanceRemainingMM) - clevis["radial_clearance_remaining_mm"]) <= 1.0e-12
            and clevis["nominal_width_clearance_status"].startswith("ZERO_NOMINAL_CLEARANCE")
            and end_effector["mujoco_collision_contract_status"].startswith("PENDING")
            and fixed_contract["parent_link"] == "link6"
            and fixed_contract["child_link"] == "gripper_mount"
            and fixed_contract["origin_xyz_m"] == [0.0, 0.0, 0.0]
            and fixed_contract["origin_rpy_rad"] == [0.0, 0.0, 0.0]
            and end_effector["tcp_status"].startswith("NOT_DEFINED")
        )

        guard = load_module(GUARD)
        guard.install()
        guard_install_pass = all(abs(number(v15_doc.getObject(name).Angle)) <= 1.0e-12 for name in JOINTS)

        source_hash_after = sha256_file(SOURCE)
        pass_conditions = {
            "source_v14_hash_certified_and_unchanged": source_hash_before == EXPECTED_SOURCE_HASH == source_hash_after,
            "v15_hash_matches_axis_json": v15_hash == data["output_v15_fcstd_sha256"],
            "all_v14_objects_preserved": not missing_objects and not type_mismatches,
            "all_original_brep_topology_and_numeric_geometry_preserved": not shape_serialization_geometry_mismatches,
            "all_original_joint_core_properties_exact": not joint_property_mismatches,
            "all_zero_transforms_recomputed": max(transform_errors.values()) <= 1.0e-9,
            "all_canonical_axes_exact": not canonical_errors,
            "downstream_chain_is_six_dof_with_dual_actuator_j2": downstream_topology_ok,
            "all_joint_types_and_position_limits_match_freecad": not limit_contract_errors,
            "j6_is_revolute_with_pm180_range": j6_revolute_pm180_ok,
            "j6_dm_g6220_gripper_mount_contract_and_known_mismatch_preserved": end_effector_contract_ok,
            "v14_guard_installs_on_v15_at_zero": guard_install_pass,
        }
        report = {
            "schema": "go-m8010-arm-v15-independent-qa/1.0",
            "status": "PASS" if all(pass_conditions.values()) else "FAIL",
            "pass_conditions": pass_conditions,
            "source_v14_sha256": source_hash_before,
            "v15_fcstd_sha256": v15_hash,
            "source_object_count": len(source_objects),
            "v15_object_count": len(v15_objects),
            "original_shape_object_count": shape_count,
            "missing_original_objects": missing_objects,
            "original_object_type_mismatches": type_mismatches,
            "original_brep_shape_mismatches": shape_hash_mismatches,
            "original_brep_hash_mismatch_interpretation": "floating point reserialization only when geometry mismatch list is empty",
            "original_brep_serialization_geometry_mismatches": shape_serialization_geometry_mismatches,
            "brep_reserialization_max_absolute_token_delta": shape_reserialization_max_abs,
            "brep_reserialization_max_scaled_token_delta": shape_reserialization_max_scaled,
            "original_joint_core_property_mismatches": joint_property_mismatches,
            "zero_transform_max_errors": transform_errors,
            "canonical_axis_errors": canonical_errors,
            "downstream_joint_names": chain_names,
            "j2_physical_actuators": j2_rows[0]["physical_actuators"] if j2_rows else None,
            "joint_limit_contract_errors": limit_contract_errors,
            "j6_joint_type": chain_by_name["J6"]["ros2_urdf"]["joint_type"],
            "j6_position_limit_deg": j6_limit["position_limit_deg"],
            "j6_mujoco_range_rad": chain_by_name["J6"]["mujoco_mjcf"]["joint_range_rad"],
            "j6_gripper_mount_origin_error_mm": mount["origin_to_j6_joint_center_error_mm"],
            "j6_gripper_mount_axis_error_deg": mount["axis_to_j6_error_deg"],
            "connector_contact_plane_station_mm": seat["contact_plane_brep_witness"]["axial_station_mm"],
            "connector_to_dm_max_six_m4_axis_error_mm": hole_interface["maximum_connector_to_dm_hole_axis_error_mm"],
            "j6_stator_mount_max_three_m4_axis_error_mm": stator_mount["maximum_axis_error_mm"],
            "gripper_clevis_max_cross_axis_error_mm": clevis["maximum_cross_axis_error_mm"],
            "gripper_clevis_radial_clearance_remaining_mm": clevis["radial_clearance_remaining_mm"],
            "tcp_status": end_effector["tcp_status"],
        }
    finally:
        App.closeDocument(v15_doc.Name)
        App.closeDocument(source_doc.Name)

    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = main()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["status"] == "PASS" else 1)
