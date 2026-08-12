from __future__ import annotations

"""Build the V15.15 link2-to-gripper COM audit ledger.

This tool is intentionally fail-closed.  It reads the accepted mass ledger as
the only mass authority and opens both accepted FCStd files read-only.  It does
not write inertial data to URDF, Xacro, MJCF, or any source CAD document.

The accepted V15.13 source contains high-detail visual meshes for the printed
parts.  A visual mesh is accepted for a volume centroid only when it is closed,
manifold, and has positive volume.  There is no collision-proxy fallback.
"""

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
from decimal import Decimal
from pathlib import Path
from typing import Any, Iterable

try:
    import FreeCAD as App
    import Mesh
    import Part
except Exception as exc:  # pragma: no cover - exercised outside FreeCAD
    App = None
    Mesh = None
    Part = None
    FREECAD_IMPORT_ERROR = exc
else:
    FREECAD_IMPORT_ERROR = None


SCHEMA = "go-m8010-arm-v15.15-com-ledger-v1/1.0"
REVISION = "V15.15-COM_LEDGER_V1"
SOURCE_BRANCH = "agent/v15-15-mass-ledger-v1"
SOURCE_COMMIT = "b3dfc6d6be8c81b177aa90a9b68a580d0f6af295"
SOURCE_TREE = "e3f5b14a26fd7582c2ed7b2a2fb12589670bda49"
TARGET_BRANCH = "agent/v15-15-com-v1"

MASS_LEDGER = "V15_15_实测质量账本_v1.json"
OUTPUT_JSON = "V15_15_COM账本_v1.json"
OUTPUT_MD = "V15_15_COM账本_v1.md"
SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
DERIVED_CAD = "机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd"
MANIFEST = "rigid_links_v15_13/rigid_link_manifest.json"
MEMBERSHIP = "rigid_links_v15_13/rigid_link_membership.csv"
RIGID_BUILDER = "完整工程_V15_13_交付/tools/build_robot_arm_rigid_links_v15.py"
UPPER_A_MASS_GEOMETRY = (
    "mass_properties_geometry_v15_15/"
    "UpperArm_A_SleeveSide_PrintPart_mass_properties.stl"
)
UPPER_A_MASS_REPORT = (
    "mass_properties_geometry_v15_15/"
    "UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json"
)
UPPER_A_MASS_GEOMETRY_SHA256 = "67aed62b304cfef3a338b0e59b035426b8e012619c333d032e8304c389352f45"
UPPER_A_MASS_REPORT_SHA256 = "a2b58c9892797b26c4511ff2581e7224d99cd27ca2261cc864252a6e38c21815"
UPPER_A_REPORT_SCHEMA = "go-m8010-arm-v15.15-upperarm-a-mass-properties/1.0"

PROTECTED_HASHES = {
    "V15_15_实测质量映射契约.json": "40f86fa1a529dc86ba31e7e6a724f2fdf79022cf8962b7526f1dbdf0b4deac5d",
    "V15_15_实测质量映射说明.md": "d4dde2f1c77eeb595818f3fb953ad967d605478da10ba1c30c9976085bd6eae1",
    "tools/build_mass_mapping_v15_15.py": "e365d81c1233fcd7dedcde190cdf7d74cf13a19c1d1a8c0e6fcfb99b6a9ca329",
    MASS_LEDGER: "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    "V15_15_实测质量账本_v1.md": "fa52587a3309ea5665197283b62ac2e165e67663070408931611fa3b7b4e946d",
    "tools/validate_mass_ledger_v15_15.py": "b12d8e40399b86a6f7c169d3e2506173f2fcd36eff53ad29319841a12e0b8804",
    SOURCE_CAD: "b4a02a97afaff88e246a46340dad4cacd3c645a7456fdd087a69552e14d3ddc0",
    DERIVED_CAD: "2ae05fab2a15c5c75bb05db8b456f149da20f79cedba2fc8d47f1624bcec34c7",
    MEMBERSHIP: "3d2a4d52d679eb97917410c6e60bec22c0c269f91ed31b35fd53ff94167e1b8c",
    MANIFEST: "8db76f9228f7a60bd017276239e3669de3cfd443c8cd36d0dd555e184606384f",
    RIGID_BUILDER: "e9b66986377eb2a30d464c6b9ca9c1371bc2e83f4e7d180f23e0da08cf8fb944",
    UPPER_A_MASS_GEOMETRY: UPPER_A_MASS_GEOMETRY_SHA256,
    UPPER_A_MASS_REPORT: UPPER_A_MASS_REPORT_SHA256,
}

ALLOWED_CHANGED_PATHS = {
    OUTPUT_JSON,
    OUTPUT_MD,
    "tools/build_com_ledger_v15_15.py",
    "tools/validate_com_ledger_v15_15.py",
    "tools/build_upperarm_a_mass_properties_v15_15.py",
    "tools/validate_upperarm_a_mass_properties_v15_15.py",
    UPPER_A_MASS_GEOMETRY,
    UPPER_A_MASS_REPORT,
}

LINK_ORDER = ["link2", "link3", "link4", "link5", "link6", "gripper"]
EXPECTED_LINK_MASSES = {
    "link2": Decimal("0.7615"),
    "link3": Decimal("1.0900"),
    "link4": Decimal("0.6750"),
    "link5": Decimal("0.5040"),
    "link6": Decimal("0.1250"),
    "gripper": Decimal("0.2960"),
}
EXPECTED_TOTAL_MASS = Decimal("3.4515")
COMPONENT_ORDER = [
    "J2A_OUTPUT_EQ",
    "J2B_OUTPUT_EQ",
    "UPPER_ARM_PRINT_MEASURED",
    "J3_OUTPUT_EQ",
    "J3_STATOR_EQ",
    "FOREARM_PRINT_MEASURED",
    "J4_STATOR_EQ",
    "RESIDUAL_C_LINK3_FOREARM_HARDWARE_WIRING",
    "J4_OUTPUT_EQ",
    "WRIST_PRELINK_PRINT_MEASURED",
    "J5_STATOR_EQ",
    "RESIDUAL_B_LINK4_J5_INTERFACE_HARDWARE_WIRING",
    "J5_OUTPUT_EQ",
    "J6_DM_STATOR_EQ",
    "RESIDUAL_A_LINK5_ADAPTER_HARDWARE_WIRING",
    "J6_DM_OUTPUT_EQ",
    "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP",
]

OUTPUT_SOLID_INDICES = {24, 25, 26, 27, 28, 29, 30, 32, 34}
NEUTRAL_SOLID_INDICES = {31}
DERIVED_GO_OBJECTS = {
    "derived:J2A_GO_M8010_output": ("J2_Left_Joint_Motor_GO_M8010_6", OUTPUT_SOLID_INDICES),
    "derived:J2A_GO_M8010_neutral": ("J2_Left_Joint_Motor_GO_M8010_6", NEUTRAL_SOLID_INDICES),
    "derived:J2B_GO_M8010_output": ("J2_Right_Joint_Motor_GO_M8010_6", OUTPUT_SOLID_INDICES),
    "derived:J2B_GO_M8010_neutral": ("J2_Right_Joint_Motor_GO_M8010_6", NEUTRAL_SOLID_INDICES),
}

MESH_MEMBERS = {
    "UpperArm_A_SleeveSide_PrintPart",
    "UpperArm_B_Distal_PrintPart",
    "Forearm_v3_HighDetail_Display",
    "Wrist_Prelink_v1_HighDetail_Display",
}

# Independent read-only extraction performed before implementation.  The
# builder still recomputes every value from CAD; these references are a guard
# against accidentally applying a nested Placement twice or treating local
# geometry as CAD-world geometry.
AUDITED_WORLD_COM_MM = {
    "J2A_OUTPUT_EQ": [-37.737486715, 7.770220161, 182.640063717],
    "J2B_OUTPUT_EQ": [37.055008454, -7.627241979, 182.640063717],
    "J3_OUTPUT_EQ": [28.417849911, 247.969111783, 182.657273243],
    "UPPER_ARM_PRINT_MEASURED": [3.9119055019, 117.4139177018, 182.8607651079],
    "J3_STATOR_EQ": [44.659931122, 244.750925608, 182.661617302],
    "FOREARM_PRINT_MEASURED": [85.928666323, 339.768722600, 208.657945481],
    "J4_STATOR_EQ": [85.471172718, 442.938602111, 234.623041267],
    "RESIDUAL_C_LINK3_FOREARM_HARDWARE_WIRING": [87.330531502, 339.131619797, 208.637275568],
    "J4_OUTPUT_EQ": [69.229091507, 446.156788286, 234.618697208],
    "WRIST_PRELINK_PRINT_MEASURED": [89.207389534, 506.948053549, 221.528945491],
    "J5_STATOR_EQ": [106.529718804, 538.367333770, 234.923489599],
    "RESIDUAL_B_LINK4_J5_INTERFACE_HARDWARE_WIRING": [75.575013632, 481.164417243, 226.496230012],
    "J5_OUTPUT_EQ": [106.381830437, 538.166630929, 251.479448967],
    "J6_DM_STATOR_EQ": [176.022020187, 579.404348279, 235.517240059],
    "RESIDUAL_A_LINK5_ADAPTER_HARDWARE_WIRING": [134.462259915, 554.844881975, 247.141209898],
    "J6_DM_OUTPUT_EQ": [191.806868091, 588.635261124, 235.827588601],
    "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP": [222.774737862, 605.796576035, 239.641366307],
}
AUDITED_WORLD_COM_TOL_MM = 1.0e-6

EPS_VOLUME_MM3 = 1.0e-9
MASS_TOL_KG = Decimal("1e-9")
ROUNDTRIP_TOL_M = 1.0e-8
FRAME_TOL = 1.0e-9
BBOX_TOL_MM = 1.0e-6


class AuditError(RuntimeError):
    pass


def repo_root(start: Path | None = None) -> Path:
    roots = [(start or Path(__file__)).resolve().parent, Path.cwd().resolve()]
    candidates: list[Path] = []
    for here in roots:
        for candidate in (here, *here.parents):
            if candidate not in candidates:
                candidates.append(candidate)
    for candidate in candidates:
        if (candidate / ".git").exists() and (candidate / MASS_LEDGER).is_file():
            return candidate
    raise AuditError("REPO_ROOT_NOT_FOUND")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().lower()


def json_load_decimal(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"), parse_float=Decimal)


def as_float(value: Any) -> float:
    return float(value)


def finite_vector(values: Iterable[float], size: int = 3) -> bool:
    sequence = list(values)
    return len(sequence) == size and all(math.isfinite(float(value)) for value in sequence)


def vec(vector: Any) -> list[float]:
    return [float(vector.x), float(vector.y), float(vector.z)]


def bbox_dict(minimum: list[float], maximum: list[float]) -> dict[str, list[float]]:
    return {"min_mm": [float(x) for x in minimum], "max_mm": [float(x) for x in maximum]}


def bbox_from_boundbox(bound_box: Any) -> dict[str, list[float]]:
    return bbox_dict(
        [bound_box.XMin, bound_box.YMin, bound_box.ZMin],
        [bound_box.XMax, bound_box.YMax, bound_box.ZMax],
    )


def merge_bboxes(boxes: Iterable[dict[str, list[float]]]) -> dict[str, list[float]]:
    boxes = list(boxes)
    if not boxes:
        raise AuditError("EMPTY_BOUNDING_BOX_SET")
    return bbox_dict(
        [min(box["min_mm"][axis] for box in boxes) for axis in range(3)],
        [max(box["max_mm"][axis] for box in boxes) for axis in range(3)],
    )


def bbox_corners(box: dict[str, list[float]]) -> list[list[float]]:
    low, high = box["min_mm"], box["max_mm"]
    return [
        [x, y, z]
        for x in (low[0], high[0])
        for y in (low[1], high[1])
        for z in (low[2], high[2])
    ]


def point_in_bbox(point: list[float], box: dict[str, list[float]], tolerance: float = BBOX_TOL_MM) -> bool:
    return all(
        box["min_mm"][axis] - tolerance <= point[axis] <= box["max_mm"][axis] + tolerance
        for axis in range(3)
    )


def weighted_center(records: list[dict[str, Any]], weight_key: str) -> list[float]:
    total = sum(float(record[weight_key]) for record in records)
    if not math.isfinite(total) or total <= 0.0:
        raise AuditError("NONPOSITIVE_GEOMETRY_WEIGHT")
    return [
        sum(float(record[weight_key]) * record["center_world_mm"][axis] for record in records) / total
        for axis in range(3)
    ]


def git_output(root: Path, arguments: list[str]) -> bytes:
    result = subprocess.run(
        ["git", "-c", "core.quotepath=false", *arguments],
        cwd=root,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=False,
    )
    return result.stdout


def nul_paths(payload: bytes) -> set[str]:
    return {item.decode("utf-8", errors="strict") for item in payload.split(b"\0") if item}


def changed_paths(root: Path) -> set[str]:
    tracked = nul_paths(git_output(root, ["diff", "--name-only", "-z", SOURCE_COMMIT, "--"]))
    untracked = nul_paths(git_output(root, ["ls-files", "--others", "--exclude-standard", "-z"]))
    return tracked | untracked


def git_guard(root: Path) -> dict[str, Any]:
    branch = git_output(root, ["branch", "--show-current"]).decode("utf-8").strip()
    source_tree = git_output(root, ["show", "-s", "--format=%T", SOURCE_COMMIT]).decode("ascii").strip()
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD"],
        cwd=root,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        shell=False,
    ).returncode == 0
    paths = changed_paths(root)
    unexpected = sorted(paths - ALLOWED_CHANGED_PATHS)
    if branch != TARGET_BRANCH:
        raise AuditError(f"WRONG_BRANCH:{branch}")
    if source_tree != SOURCE_TREE:
        raise AuditError(f"SOURCE_TREE_MISMATCH:{source_tree}")
    if not ancestor:
        raise AuditError("SOURCE_COMMIT_NOT_ANCESTOR")
    if unexpected:
        raise AuditError("UNEXPECTED_CHANGED_PATHS:" + ",".join(unexpected))
    return {
        "source_branch": SOURCE_BRANCH,
        "source_commit": SOURCE_COMMIT,
        "source_tree": SOURCE_TREE,
        "target_branch": TARGET_BRANCH,
        "current_branch": branch,
        "changed_paths": sorted(paths),
        "allowed_changed_paths": sorted(ALLOWED_CHANGED_PATHS),
        "unexpected_changed_paths": unexpected,
        "source_commit_is_ancestor": ancestor,
        "pass": True,
    }


def verify_protected(root: Path) -> list[dict[str, Any]]:
    records = []
    for relative, expected in PROTECTED_HASHES.items():
        path = root / relative
        if not path.is_file():
            raise AuditError(f"PROTECTED_INPUT_MISSING:{relative}")
        actual = sha256(path)
        if actual != expected:
            raise AuditError(f"PROTECTED_INPUT_HASH_MISMATCH:{relative}:{actual}")
        records.append({"path": relative, "sha256": actual, "hash_mode": "RAW_BYTES", "locked": True})
    return records


def load_authorities(root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    ledger = json_load_decimal(root / MASS_LEDGER)
    manifest = json.loads((root / MANIFEST).read_text(encoding="utf-8"))
    if ledger.get("final_status") != "V15.15 MASS_LEDGER_V1 = PASS":
        raise AuditError("MASS_LEDGER_NOT_ACCEPTED")
    if ledger.get("unresolved_items") != []:
        raise AuditError("MASS_LEDGER_HAS_UNRESOLVED_ITEMS")
    components = ledger["component_to_link_mapping"]["additive_components"]
    component_ids = [item["component_id"] for item in components]
    if set(component_ids) != set(COMPONENT_ORDER) or len(component_ids) != 17:
        raise AuditError("MASS_COMPONENT_SET_MISMATCH")
    embedded = {
        item["component_id"]: item
        for link in LINK_ORDER
        for item in ledger["link_mass_ledger"][link]["components"]
    }
    top = {item["component_id"]: item for item in components}
    if embedded != top:
        raise AuditError("MASS_COMPONENT_AUTHORITIES_DISAGREE")
    for link, expected in EXPECTED_LINK_MASSES.items():
        actual = Decimal(str(ledger["link_mass_ledger"][link]["nominal_mass_kg"]))
        if abs(actual - expected) > MASS_TOL_KG:
            raise AuditError(f"FROZEN_LINK_MASS_MISMATCH:{link}:{actual}")
    total = Decimal(str(ledger["total_link2_to_gripper_nominal_mass_kg"]))
    if abs(total - EXPECTED_TOTAL_MASS) > MASS_TOL_KG:
        raise AuditError(f"FROZEN_TOTAL_MASS_MISMATCH:{total}")
    return ledger, manifest


def validate_frame(frame: dict[str, Any], link: str) -> tuple[list[float], list[list[float]]]:
    origin = [float(x) for x in frame["origin_mm"]]
    rotation = [[float(x) for x in row] for row in frame["rotation_matrix_row_major"]]
    if not finite_vector(origin) or len(rotation) != 3 or any(not finite_vector(row) for row in rotation):
        raise AuditError(f"NONFINITE_FRAME:{link}")
    for row in range(3):
        for column in range(3):
            dot = sum(rotation[k][row] * rotation[k][column] for k in range(3))
            expected = 1.0 if row == column else 0.0
            if abs(dot - expected) > FRAME_TOL:
                raise AuditError(f"FRAME_NOT_ORTHONORMAL:{link}")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > FRAME_TOL:
        raise AuditError(f"FRAME_DETERMINANT_INVALID:{link}:{determinant}")
    return origin, rotation


def world_to_local_mm(point: list[float], origin: list[float], rotation: list[list[float]]) -> list[float]:
    delta = [point[axis] - origin[axis] for axis in range(3)]
    return [sum(rotation[row][column] * delta[row] for row in range(3)) for column in range(3)]


def local_to_world_mm(point: list[float], origin: list[float], rotation: list[list[float]]) -> list[float]:
    return [origin[row] + sum(rotation[row][column] * point[column] for column in range(3)) for row in range(3)]


def bbox_world_to_local(box: dict[str, list[float]], origin: list[float], rotation: list[list[float]]) -> dict[str, list[float]]:
    points = [world_to_local_mm(point, origin, rotation) for point in bbox_corners(box)]
    return bbox_dict(
        [min(point[axis] for point in points) for axis in range(3)],
        [max(point[axis] for point in points) for axis in range(3)],
    )


def brep_member(doc: Any, token: str) -> dict[str, Any]:
    if "Collision_Proxy" in token or token.endswith("_collision"):
        raise AuditError(f"COLLISION_PROXY_FORBIDDEN:{token}")
    selected_indices: set[int] | None = None
    object_name = token
    if token in DERIVED_GO_OBJECTS:
        object_name, selected_indices = DERIVED_GO_OBJECTS[token]
    obj = doc.getObject(object_name)
    if obj is None:
        raise AuditError(f"CAD_MEMBER_MISSING:{token}")
    if not hasattr(obj, "Shape") or obj.Shape.isNull():
        raise AuditError(f"CAD_BREP_MISSING:{token}:{obj.TypeId}")
    all_solids = list(obj.Shape.Solids)
    if selected_indices is not None:
        if len(all_solids) != 34:
            raise AuditError(f"GO_SOLID_COUNT_MISMATCH:{object_name}:{len(all_solids)}")
        solids = [solid for index, solid in enumerate(all_solids, 1) if index in selected_indices]
        if len(solids) != len(selected_indices):
            raise AuditError(f"GO_SOLID_SELECTION_INCOMPLETE:{token}")
    else:
        solids = all_solids
    atom_records = []
    for solid in solids:
        signed_volume = float(solid.Volume)
        absolute_volume = abs(signed_volume)
        if absolute_volume <= EPS_VOLUME_MM3:
            continue
        center = vec(solid.CenterOfGravity)
        box = bbox_from_boundbox(solid.BoundBox)
        if not finite_vector(center):
            raise AuditError(f"NONFINITE_BREP_CENTER:{token}")
        atom_records.append(
            {
                "signed_volume_mm3": signed_volume,
                "abs_volume_mm3": absolute_volume,
                "center_world_mm": center,
                "bbox_world_mm": box,
            }
        )
    if not atom_records:
        raise AuditError(f"NO_POSITIVE_VOLUME_BREP_SOLIDS:{token}")
    center = weighted_center(atom_records, "abs_volume_mm3")
    box = merge_bboxes(record["bbox_world_mm"] for record in atom_records)
    return {
        "member": token,
        "source_object": object_name,
        "source_type": obj.TypeId,
        "selected_solid_indices": sorted(selected_indices) if selected_indices else None,
        "solid_count": len(atom_records),
        "negative_oriented_solid_count": sum(record["signed_volume_mm3"] < 0.0 for record in atom_records),
        "signed_volume_sum_mm3": sum(record["signed_volume_mm3"] for record in atom_records),
        "abs_volume_mm3": sum(record["abs_volume_mm3"] for record in atom_records),
        "center_world_mm": center,
        "bbox_world_mm": box,
    }


def brep_geometry(doc: Any, members: list[str]) -> dict[str, Any]:
    records = [brep_member(doc, member) for member in members]
    center = weighted_center(records, "abs_volume_mm3")
    return {
        "resolved": True,
        "center_world_mm": center,
        "bbox_world_mm": merge_bboxes(record["bbox_world_mm"] for record in records),
        "volume_mm3": sum(record["abs_volume_mm3"] for record in records),
        "geometry_atoms": records,
        "negative_oriented_solid_count": sum(record["negative_oriented_solid_count"] for record in records),
    }


def mesh_diagnostic(obj: Any, member: str, repair_non_manifolds: bool = False) -> dict[str, Any]:
    if obj is None or not hasattr(obj, "Mesh"):
        raise AuditError(f"CAD_VISUAL_MESH_MISSING:{member}")
    mesh = Mesh.Mesh(obj.Mesh)
    before = {
        "facets": int(mesh.CountFacets),
        "components": int(mesh.countComponents()),
        "is_solid": bool(mesh.isSolid()),
        "has_non_manifolds": bool(mesh.hasNonManifolds()),
        "volume_mm3": float(mesh.Volume),
    }
    repair = None
    if repair_non_manifolds:
        mesh.removeNonManifolds()
        repair = "IN_MEMORY_REMOVE_NON_MANIFOLD_FACETS"
    after = {
        "facets": int(mesh.CountFacets),
        "components": int(mesh.countComponents()),
        "is_solid": bool(mesh.isSolid()),
        "has_non_manifolds": bool(mesh.hasNonManifolds()),
        "volume_mm3": float(mesh.Volume),
    }
    return {"member": member, "before": before, "after": after, "repair": repair, "mesh": mesh}


def closed_mesh_geometry(doc: Any, member: str, policy: str) -> dict[str, Any]:
    obj = doc.getObject(member)
    if member not in MESH_MEMBERS:
        raise AuditError(f"VISUAL_MESH_NOT_WHITELISTED:{member}")
    diagnostic = mesh_diagnostic(obj, member, repair_non_manifolds=(policy == "upper_b_minimal_repair"))
    mesh = diagnostic.pop("mesh")
    components = []
    excluded_zero_volume_components = []
    for index, component in enumerate(mesh.getSeparateComponents()):
        volume = abs(float(component.Volume))
        if volume <= EPS_VOLUME_MM3:
            excluded_zero_volume_components.append(
                {
                    "member": member,
                    "component_index": index,
                    "facets": int(component.CountFacets),
                    "is_solid_reported_by_freecad": bool(component.isSolid()),
                    "has_non_manifolds": bool(component.hasNonManifolds()),
                    "abs_volume_mm3": volume,
                    "exclusion_reason": "ZERO_VOLUME_COMPONENT_NOT_MASS_GEOMETRY",
                }
            )
            continue
        components.append(
            {
                "member": member,
                "component_index": index,
                "facets": int(component.CountFacets),
                "is_solid": bool(component.isSolid()),
                "has_non_manifolds": bool(component.hasNonManifolds()),
                "abs_volume_mm3": volume,
                "center_world_mm": vec(component.CenterOfGravity),
                "bbox_world_mm": bbox_from_boundbox(component.BoundBox),
            }
        )
    invalid = [record for record in components if not record["is_solid"] or record["has_non_manifolds"]]
    mesh_component_evidence = {
        "positive_volume_component_count": len(components),
        "all_positive_volume_components_closed_manifold": bool(components) and not invalid,
        "excluded_zero_volume_component_count": len(excluded_zero_volume_components),
        "excluded_zero_volume_facets": sum(record["facets"] for record in excluded_zero_volume_components),
        "excluded_zero_volume_components": excluded_zero_volume_components,
    }
    if invalid or not components:
        return {
            "resolved": False,
            "error_codes": ["SOURCE_VISUAL_MESH_NOT_CLOSED", "SOURCE_VISUAL_MESH_NON_MANIFOLD"],
            "diagnostic": diagnostic,
            "geometry_atoms": components,
            **mesh_component_evidence,
        }
    return {
        "resolved": True,
        "center_world_mm": weighted_center(components, "abs_volume_mm3"),
        "bbox_world_mm": merge_bboxes(record["bbox_world_mm"] for record in components),
        "volume_mm3": sum(record["abs_volume_mm3"] for record in components),
        "geometry_atoms": components,
        "diagnostic": diagnostic,
        **mesh_component_evidence,
    }


def upper_a_mass_properties_geometry(root: Path, doc: Any) -> dict[str, Any]:
    """Independently reimport and validate the mass-properties-only artifact."""

    artifact_path = root / UPPER_A_MASS_GEOMETRY
    report_path = root / UPPER_A_MASS_REPORT
    if sha256(artifact_path) != UPPER_A_MASS_GEOMETRY_SHA256:
        raise AuditError("UPPER_A_MASS_GEOMETRY_HASH_MISMATCH")
    if sha256(report_path) != UPPER_A_MASS_REPORT_SHA256:
        raise AuditError("UPPER_A_MASS_REPORT_HASH_MISMATCH")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("schema") != UPPER_A_REPORT_SCHEMA or report.get("status") != "PASS":
        raise AuditError("UPPER_A_MASS_REPORT_STATUS_OR_SCHEMA_MISMATCH")
    if report["artifact"]["sha256"] != UPPER_A_MASS_GEOMETRY_SHA256:
        raise AuditError("UPPER_A_MASS_REPORT_ARTIFACT_HASH_MISMATCH")
    if report["source"]["sha256"] != PROTECTED_HASHES[SOURCE_CAD]:
        raise AuditError("UPPER_A_MASS_REPORT_SOURCE_HASH_MISMATCH")
    if not report["validation"]["pass"] or report["repair_method"]["collision_proxy_used"]:
        raise AuditError("UPPER_A_MASS_REPORT_VALIDATION_FAILURE")
    if any(bool(value) for value in report["forbidden_actions"].values()):
        raise AuditError("UPPER_A_MASS_REPORT_FORBIDDEN_ACTION")

    mesh = Mesh.Mesh(str(artifact_path))
    if (
        int(mesh.CountPoints) != 899
        or int(mesh.CountFacets) != 1834
        or not bool(mesh.isSolid())
        or bool(mesh.hasNonManifolds())
        or bool(mesh.hasSelfIntersections())
    ):
        raise AuditError("UPPER_A_MASS_GEOMETRY_TOPOLOGY_FAILURE")
    shape = Part.Shape()
    shape.makeShapeFromMesh(mesh.Topology, 0.001)
    if len(shape.Shells) != 1 or not shape.isClosed() or not shape.isValid():
        raise AuditError("UPPER_A_MASS_GEOMETRY_SHELL_INVALID")
    solid = Part.makeSolid(shape.Shells[0])
    if not solid.isClosed() or not solid.isValid() or len(solid.Solids) != 1:
        raise AuditError("UPPER_A_MASS_GEOMETRY_SOLID_INVALID")
    volume = float(solid.Volume)
    source_center = vec(solid.CenterOfGravity)
    expected_volume = float(report["volume_mm3"])
    if abs(volume - expected_volume) > 1.0e-6:
        raise AuditError(f"UPPER_A_MASS_GEOMETRY_VOLUME_MISMATCH:{volume}")
    if max(abs(source_center[axis] - report["centroid_mm_source_local"][axis]) for axis in range(3)) > 1.0e-8:
        raise AuditError("UPPER_A_MASS_GEOMETRY_SOURCE_CENTER_MISMATCH")

    source_obj = doc.getObject("UpperArm_A_SleeveSide_PrintPart")
    if source_obj is None or source_obj.TypeId != "Mesh::Feature":
        raise AuditError("UPPER_A_SOURCE_OBJECT_MISSING")
    placement = source_obj.Mesh.Placement
    world_vector = placement.multVec(App.Vector(*source_center))
    world_center = vec(world_vector)
    if max(abs(world_center[axis] - report["centroid_mm_world"][axis]) for axis in range(3)) > 1.0e-8:
        raise AuditError("UPPER_A_MASS_GEOMETRY_WORLD_CENTER_MISMATCH")
    world_points = [vec(placement.multVec(point.Vector)) for point in mesh.Points]
    world_box = bbox_dict(
        [min(point[axis] for point in world_points) for axis in range(3)],
        [max(point[axis] for point in world_points) for axis in range(3)],
    )
    return {
        "member": "UpperArm_A_SleeveSide_PrintPart",
        "source_object": "UpperArm_A_SleeveSide_PrintPart",
        "source_type": "MASS_PROPERTIES_ONLY_CLOSED_MESH_REIMPORTED_AS_VALID_BREP_SOLID",
        "solid_count": 1,
        "negative_oriented_solid_count": 0,
        "signed_volume_sum_mm3": volume,
        "abs_volume_mm3": volume,
        "center_source_local_mm": source_center,
        "center_world_mm": world_center,
        "bbox_world_mm": world_box,
        "artifact_path": UPPER_A_MASS_GEOMETRY,
        "artifact_sha256": UPPER_A_MASS_GEOMETRY_SHA256,
        "report_path": UPPER_A_MASS_REPORT,
        "report_sha256": UPPER_A_MASS_REPORT_SHA256,
        "coordinate_frame": report["artifact"]["coordinate_frame"],
        "independently_reimported": True,
        "closed": True,
        "manifold": True,
        "valid": True,
        "collision_proxy_used": False,
    }


def upper_arm_geometry(root: Path, doc: Any) -> dict[str, Any]:
    upper_a = upper_a_mass_properties_geometry(root, doc)
    upper_b = closed_mesh_geometry(doc, "UpperArm_B_Distal_PrintPart", "upper_b_minimal_repair")
    if not upper_b.get("resolved"):
        raise AuditError("UPPER_ARM_B_GEOMETRY_UNRESOLVED")
    volume_a = float(upper_a["abs_volume_mm3"])
    volume_b = float(upper_b["volume_mm3"])
    total_volume = volume_a + volume_b
    fraction_a = volume_a / total_volume
    fraction_b = volume_b / total_volume
    mass_total = 0.5515
    allocated_mass_a = mass_total * fraction_a
    allocated_mass_b = mass_total * fraction_b
    center = [
        (volume_a * upper_a["center_world_mm"][axis] + volume_b * upper_b["center_world_mm"][axis])
        / total_volume
        for axis in range(3)
    ]
    upper_b_atom = {
        "member": "UpperArm_B_Distal_PrintPart",
        "source_type": "CAD_HIGH_DETAIL_VISUAL_MESH_MINIMAL_NONMANIFOLD_FACET_REPAIR",
        "solid_count": 1,
        "negative_oriented_solid_count": 0,
        "signed_volume_sum_mm3": volume_b,
        "abs_volume_mm3": volume_b,
        "center_world_mm": upper_b["center_world_mm"],
        "bbox_world_mm": upper_b["bbox_world_mm"],
        "closed": True,
        "manifold": True,
        "valid": True,
        "collision_proxy_used": False,
    }
    return {
        "resolved": True,
        "center_world_mm": center,
        "bbox_world_mm": merge_bboxes([upper_a["bbox_world_mm"], upper_b["bbox_world_mm"]]),
        "volume_mm3": total_volume,
        "geometry_atoms": [upper_a, upper_b_atom],
        "negative_oriented_solid_count": 0,
        "positive_volume_component_count": 2,
        "all_positive_volume_components_closed_manifold": True,
        "mass_properties_artifact": {
            "path": UPPER_A_MASS_GEOMETRY,
            "sha256": UPPER_A_MASS_GEOMETRY_SHA256,
            "report_path": UPPER_A_MASS_REPORT,
            "report_sha256": UPPER_A_MASS_REPORT_SHA256,
            "independently_reimported": True,
            "report_crosscheck_pass": True,
        },
        "equal_density_volume_allocation": {
            "assumption": "SAME_PRINT_MATERIAL_UNIFORM_EFFECTIVE_DENSITY_V1",
            "total_measured_mass_kg": mass_total,
            "total_volume_mm3": total_volume,
            "parts": [
                {
                    "member": "UpperArm_A_SleeveSide_PrintPart",
                    "volume_mm3": volume_a,
                    "volume_fraction": fraction_a,
                    "allocated_mass_kg": allocated_mass_a,
                    "center_world_mm": upper_a["center_world_mm"],
                },
                {
                    "member": "UpperArm_B_Distal_PrintPart",
                    "volume_mm3": volume_b,
                    "volume_fraction": fraction_b,
                    "allocated_mass_kg": allocated_mass_b,
                    "center_world_mm": upper_b["center_world_mm"],
                },
            ],
            "volume_fraction_sum": fraction_a + fraction_b,
            "allocated_mass_sum_kg": allocated_mass_a + allocated_mass_b,
            "fifty_fifty_used": False,
            "allocation_is_explanatory_not_additive": True,
            "pass": abs(fraction_a + fraction_b - 1.0) < 1.0e-12
            and abs(allocated_mass_a + allocated_mass_b - mass_total) < 1.0e-12,
        },
        "notes": (
            "UpperArm A uses an independently reimported, deterministic mass-properties-only closed geometry; "
            "UpperArm B uses the accepted minimal source-mesh repair. The measured 0.5515 kg total is split "
            "for COM explanation only by equal-density volume fractions, never 50/50."
        ),
    }


def geometry_for_component(root: Path, doc: Any, component: dict[str, Any]) -> dict[str, Any]:
    component_id = component["component_id"]
    members = list(component["cad_members"])
    if component_id == "UPPER_ARM_PRINT_MEASURED":
        return upper_arm_geometry(root, doc)
    if component_id == "FOREARM_PRINT_MEASURED":
        return closed_mesh_geometry(doc, members[0], "positive_closed_components_only")
    if component_id == "WRIST_PRELINK_PRINT_MEASURED":
        return closed_mesh_geometry(doc, members[0], "closed_mesh")
    return brep_geometry(doc, members)


def component_status_and_method(component_id: str, geometry: dict[str, Any]) -> tuple[str, str]:
    if not geometry.get("resolved"):
        return "UNRESOLVED", "FAIL_CLOSED_NO_COLLISION_PROXY_FALLBACK"
    if component_id.startswith("RESIDUAL_"):
        return "ENGINEERING_ESTIMATE_GEOMETRIC_CENTROID", "CAD_MEMBER_ABS_VOLUME_WEIGHTED_GEOMETRIC_CENTROID"
    if component_id == "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP":
        return "MEASURED_TOTAL_MASS_WITH_GEOMETRIC_COM_ESTIMATE", "37_MEMBER_CAD_ABS_VOLUME_WEIGHTED_GEOMETRIC_CENTROID"
    if component_id == "UPPER_ARM_PRINT_MEASURED":
        return (
            "MEASURED_TOTAL_MASS_WITH_EQUAL_DENSITY_VOLUME_WEIGHTED_GEOMETRIC_COM",
            "UPPERARM_A_MASS_PROPERTIES_ARTIFACT_PLUS_UPPERARM_B_SOURCE_MESH_VOLUME_WEIGHTED_CENTER",
        )
    if component_id == "FOREARM_PRINT_MEASURED":
        return (
            "CAD_SOURCE_MESH_ZERO_VOLUME_ARTIFACT_EXCLUDED_VOLUME_CENTER",
            "POSITIVE_VOLUME_CLOSED_MANIFOLD_SOURCE_MESH_COMPONENT_VOLUME_CENTER",
        )
    if component_id == "WRIST_PRELINK_PRINT_MEASURED":
        return "CAD_HIGH_DETAIL_VISUAL_MESH_VOLUME_CENTROID", "CLOSED_SOURCE_VISUAL_MESH_VOLUME_CENTROID"
    if component_id in {"J2A_OUTPUT_EQ", "J2B_OUTPUT_EQ", "J3_OUTPUT_EQ", "J4_OUTPUT_EQ", "J5_OUTPUT_EQ"}:
        return "CAD_COMPOUND_VOLUME_WEIGHTED_CENTER", "GO_OUTPUT_PLUS_NEUTRAL_BREP_SOLID_ABS_VOLUME_WEIGHTED_CENTER"
    atoms = geometry.get("geometry_atoms", [])
    solid_count = sum(int(atom.get("solid_count", 1)) for atom in atoms)
    if len(atoms) == 1 and solid_count == 1:
        return "CAD_BREP_CENTER_OF_MASS", "FREECAD_BREP_SHAPE_CENTER_OF_GRAVITY"
    return "CAD_COMPOUND_VOLUME_WEIGHTED_CENTER", "FREECAD_BREP_SOLID_ABS_VOLUME_WEIGHTED_CENTER"


def build_component(
    root: Path,
    doc: Any,
    mass_component: dict[str, Any],
    origin: list[float],
    rotation: list[list[float]],
) -> dict[str, Any]:
    geometry = geometry_for_component(root, doc, mass_component)
    component_id = mass_component["component_id"]
    status, method = component_status_and_method(component_id, geometry)
    output = {
        "component_id": component_id,
        "name": mass_component["name"],
        "owner_link": mass_component["ledger_link"],
        "mass_kg": as_float(mass_component["nominal_mass_kg"]),
        "mass_status": mass_component["mass_status"],
        "source_mass_component_id": component_id,
        "geometry_source": SOURCE_CAD,
        "geometry_members": list(mass_component["cad_members"]),
        "geometry_method": method,
        "cad_com_world_mm": None,
        "cad_com_link_mm": None,
        "com_status": status,
        "cad_volume_mm3": geometry.get("volume_mm3"),
        "bbox_world_mm": geometry.get("bbox_world_mm"),
        "bbox_link_mm": None,
        "reconstructed_world_com_m": None,
        "round_trip_error_m": None,
        "component_com_inside_bbox": None,
        "collision_proxy_used": False,
        "geometry_atoms": geometry.get("geometry_atoms", []),
        "positive_volume_component_count": geometry.get("positive_volume_component_count"),
        "all_positive_volume_components_closed_manifold": geometry.get(
            "all_positive_volume_components_closed_manifold"
        ),
        "excluded_zero_volume_component_count": geometry.get("excluded_zero_volume_component_count"),
        "excluded_zero_volume_facets": geometry.get("excluded_zero_volume_facets"),
        "excluded_zero_volume_components": geometry.get("excluded_zero_volume_components", []),
        "cable_geometry_available": False if component_id.startswith("RESIDUAL_") else None,
        "notes": mass_component.get("notes", ""),
    }
    if component_id == "UPPER_ARM_PRINT_MEASURED" and geometry.get("resolved"):
        output["geometry_source"] = f"{UPPER_A_MASS_GEOMETRY} + {SOURCE_CAD}:UpperArm_B_Distal_PrintPart"
        output["mass_properties_artifact"] = geometry["mass_properties_artifact"]
        output["equal_density_volume_allocation"] = geometry["equal_density_volume_allocation"]
    if component_id == "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP":
        output["closure_angle_deg"] = 0.0
        output["notes"] += " Internal member masses were not guessed; complete measured 0.296 kg is assigned to the 37-member geometric centroid."
    if not geometry.get("resolved"):
        output["blocking"] = True
        output["error_codes"] = geometry["error_codes"]
        output["blocking_member"] = geometry.get("blocking_member")
        output["diagnostic_subgeometry"] = geometry.get("diagnostic_subgeometry", {})
        output["notes"] = geometry.get("notes", output["notes"])
        return output
    world = [float(x) for x in geometry["center_world_mm"]]
    reference = AUDITED_WORLD_COM_MM[component_id]
    reference_error_mm = math.sqrt(sum((world[axis] - reference[axis]) ** 2 for axis in range(3)))
    if reference_error_mm > AUDITED_WORLD_COM_TOL_MM:
        raise AuditError(f"CAD_WORLD_PLACEMENT_CROSSCHECK_FAILURE:{component_id}:{reference_error_mm}")
    local = world_to_local_mm(world, origin, rotation)
    reconstructed = local_to_world_mm(local, origin, rotation)
    error_m = math.sqrt(sum((reconstructed[axis] - world[axis]) ** 2 for axis in range(3))) * 0.001
    local_bbox = bbox_world_to_local(geometry["bbox_world_mm"], origin, rotation)
    if error_m >= ROUNDTRIP_TOL_M:
        raise AuditError(f"COMPONENT_ROUNDTRIP_FAILURE:{component_id}:{error_m}")
    if not point_in_bbox(world, geometry["bbox_world_mm"]):
        raise AuditError(f"COMPONENT_COM_OUTSIDE_BBOX:{component_id}")
    output.update(
        {
            "cad_com_world_mm": world,
            "cad_com_link_mm": local,
            "bbox_link_mm": local_bbox,
            "reconstructed_world_com_m": [value * 0.001 for value in reconstructed],
            "round_trip_error_m": error_m,
            "component_com_inside_bbox": True,
            "independent_audit_reference_world_com_mm": reference,
            "independent_audit_reference_error_mm": reference_error_mm,
            "independent_audit_reference_pass": True,
            "blocking": False,
        }
    )
    return output


def verify_gripper_zero(doc: Any) -> dict[str, Any]:
    controller = doc.getObject("Gripper_Servo_Controller")
    if controller is None or not hasattr(controller, "ClosureAngle"):
        raise AuditError("GRIPPER_CLOSURE_AUTHORITY_MISSING")
    angle = float(controller.ClosureAngle)
    if abs(angle) > 1.0e-12:
        raise AuditError(f"GRIPPER_NOT_AT_CLOSURE_ZERO:{angle}")
    return {"controller": "Gripper_Servo_Controller", "closure_angle_deg": angle, "pass": True}


def link_record(
    link: str,
    mass_kg: Decimal,
    components: list[dict[str, Any]],
    frame: dict[str, Any],
) -> dict[str, Any]:
    component_sum = sum(Decimal(str(component["mass_kg"])) for component in components)
    mass_error = abs(component_sum - mass_kg)
    if mass_error > MASS_TOL_KG:
        raise AuditError(f"LINK_COMPONENT_MASS_SUM_FAILURE:{link}:{component_sum}")
    resolved = all(not component["blocking"] for component in components)
    origin, rotation = validate_frame(frame, link)
    record = {
        "mass_kg": float(mass_kg),
        "component_mass_sum_kg": float(component_sum),
        "frame_world_at_zero": frame,
        "com_xyz_m_in_link_frame": None,
        "com_xyz_mm_in_link_frame": None,
        "com_xyz_m_world_at_zero": None,
        "combined_mass_component_bbox_link_mm": None,
        "components": components,
        "validation": {
            "component_mass_sum_error_kg": float(mass_error),
            "component_mass_sum_pass": True,
            "all_components_resolved": resolved,
            "com_finite": None,
            "com_inside_combined_component_bbox": None,
            "link_world_vs_local_round_trip_error_m": None,
            "pass": False,
        },
    }
    if not resolved:
        record["validation"]["failure_code"] = "LINK_COM_BLOCKED_BY_UNRESOLVED_COMPONENT"
        return record
    mass = float(component_sum)
    world_mm = [
        sum(component["mass_kg"] * component["cad_com_world_mm"][axis] for component in components) / mass
        for axis in range(3)
    ]
    local_mm = world_to_local_mm(world_mm, origin, rotation)
    reconstructed = local_to_world_mm(local_mm, origin, rotation)
    roundtrip = math.sqrt(sum((reconstructed[axis] - world_mm[axis]) ** 2 for axis in range(3))) * 0.001
    combined_bbox = merge_bboxes(component["bbox_link_mm"] for component in components)
    finite = finite_vector(world_mm) and finite_vector(local_mm)
    inside = point_in_bbox(local_mm, combined_bbox)
    passed = finite and inside and roundtrip < ROUNDTRIP_TOL_M
    if not passed:
        raise AuditError(f"LINK_COM_NUMERIC_FAILURE:{link}")
    record.update(
        {
            "com_xyz_m_in_link_frame": [value * 0.001 for value in local_mm],
            "com_xyz_mm_in_link_frame": local_mm,
            "com_xyz_m_world_at_zero": [value * 0.001 for value in world_mm],
            "combined_mass_component_bbox_link_mm": combined_bbox,
        }
    )
    record["validation"].update(
        {
            "com_finite": finite,
            "com_inside_combined_component_bbox": inside,
            "link_world_vs_local_round_trip_error_m": roundtrip,
            "pass": passed,
        }
    )
    return record


def unresolved_items_from_links(links: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    """Derive every unresolved record from blocking component evidence."""

    unresolved: list[dict[str, Any]] = []
    for link_name in LINK_ORDER:
        for component in links[link_name]["components"]:
            if not component.get("blocking"):
                continue
            error_codes = list(dict.fromkeys(component.get("error_codes") or ["COMPONENT_COM_UNRESOLVED"]))
            for code in error_codes:
                unresolved.append(
                    {
                        "code": code,
                        "owner_link": link_name,
                        "component_id": component["component_id"],
                        "member": component.get("blocking_member"),
                        "blocking": True,
                        "reason": component.get("notes", "Authoritative component COM geometry is unresolved."),
                        "required_resolution": (
                            "Provide or repair authoritative closed, manifold source BRep/STEP/visual geometry, "
                            "then re-run the read-only COM audit."
                        ),
                    }
                )
    return unresolved


def build_document(root: Path | None = None) -> dict[str, Any]:
    if App is None or Mesh is None:
        raise AuditError(f"FREECAD_RUNTIME_REQUIRED:{FREECAD_IMPORT_ERROR}")
    root = repo_root(root)
    guard = git_guard(root)
    protected_before = verify_protected(root)
    mass_ledger, manifest = load_authorities(root)
    mass_components = {
        item["component_id"]: item
        for item in mass_ledger["component_to_link_mapping"]["additive_components"]
    }
    source_doc = None
    derived_doc = None
    try:
        source_doc = App.openDocument(str(root / SOURCE_CAD))
        derived_doc = App.openDocument(str(root / DERIVED_CAD))
        if derived_doc is None:
            raise AuditError("DERIVED_CAD_OPEN_FAILED")
        gripper_zero = verify_gripper_zero(source_doc)
        components_by_link: dict[str, list[dict[str, Any]]] = {link: [] for link in LINK_ORDER}
        for component_id in COMPONENT_ORDER:
            authority = mass_components[component_id]
            link = authority["ledger_link"]
            frame = manifest["links"][link]["frame_world_at_zero"]
            origin, rotation = validate_frame(frame, link)
            component = build_component(root, source_doc, authority, origin, rotation)
            components_by_link[link].append(component)
    finally:
        if derived_doc is not None:
            App.closeDocument(derived_doc.Name)
        if source_doc is not None:
            App.closeDocument(source_doc.Name)
    protected_after = verify_protected(root)
    if protected_before != protected_after:
        raise AuditError("PROTECTED_INPUT_CHANGED_DURING_CAD_READ")

    links = {}
    for link in LINK_ORDER:
        frame = manifest["links"][link]["frame_world_at_zero"]
        links[link] = link_record(link, EXPECTED_LINK_MASSES[link], components_by_link[link], frame)
    component_roundtrips = [
        component["round_trip_error_m"]
        for link in links.values()
        for component in link["components"]
        if component["round_trip_error_m"] is not None
    ]
    total = sum(Decimal(str(links[link]["mass_kg"])) for link in LINK_ORDER)
    total_mass_pass = abs(total - EXPECTED_TOTAL_MASS) <= MASS_TOL_KG
    unresolved = unresolved_items_from_links(links)
    all_links_resolved = all(link["validation"]["all_components_resolved"] for link in links.values())
    all_link_numeric_pass = all(link["validation"]["pass"] for link in links.values())
    final_pass = total_mass_pass and all_links_resolved and all_link_numeric_pass and not unresolved
    final_status = "V15.15 COM_LEDGER_V1 = PASS" if final_pass else "V15.15 COM_LEDGER_V1 = FAIL"
    return {
        "schema": SCHEMA,
        "revision": REVISION,
        "source_branch": SOURCE_BRANCH,
        "source_commit": SOURCE_COMMIT,
        "source_tree": SOURCE_TREE,
        "target_branch": TARGET_BRANCH,
        "scope": "LINK2_TO_GRIPPER_COM_ONLY_NO_INERTIA_NO_DYNAMICS",
        "mass_ledger_path": MASS_LEDGER,
        "mass_ledger_sha256": PROTECTED_HASHES[MASS_LEDGER],
        "mass_ledger_hash_mode": "RAW_BYTES",
        "cad_source": SOURCE_CAD,
        "cad_source_sha256": PROTECTED_HASHES[SOURCE_CAD],
        "cad_sources": [
            {"path": SOURCE_CAD, "sha256": PROTECTED_HASHES[SOURCE_CAD], "opened_read_only": True},
            {"path": DERIVED_CAD, "sha256": PROTECTED_HASHES[DERIVED_CAD], "opened_read_only": True},
        ],
        "git_scope_guard": guard,
        "protected_input_guard": {
            "before": protected_before,
            "after": protected_after,
            "all_locked": True,
            "source_cad_saved_or_modified": False,
        },
        "units": {"mass": "kg", "cad_length": "mm", "authoritative_com_length": "m"},
        "mechanical_zero_condition": {"joint_configuration": "accepted V15.13 mechanical zero", **gripper_zero},
        "frame_authority": {
            "path": MANIFEST,
            "sha256": PROTECTED_HASHES[MANIFEST],
            "final_com_frame": "owner ROS2/MuJoCo link frame",
            "world_coordinates_role": "audit_cross_check_only",
            "transform_equations": {
                "world_mm_to_link_mm": "p_L = R_WL^T * (p_W - o_WL)",
                "link_mm_to_world_mm": "p_W = o_WL + R_WL * p_L",
            },
        },
        "geometry_policy": {
            "whole_link_stl_center_used": False,
            "collision_proxy_fallback_allowed": False,
            "collision_proxy_used": False,
            "upperarm_a_mass_properties_artifact": {
                "path": UPPER_A_MASS_GEOMETRY,
                "sha256": UPPER_A_MASS_GEOMETRY_SHA256,
                "report_path": UPPER_A_MASS_REPORT,
                "report_sha256": UPPER_A_MASS_REPORT_SHA256,
                "usage": "volume_and_geometric_centroid_only",
                "independently_reimported": True,
            },
            "brep_volume_policy": "per-solid abs(Volume) weighting prevents negative orientation cancellation",
            "printed_part_policy": (
                "each retained positive-volume source high-detail visual-mesh component must be closed and manifold; "
                "zero-volume components may be excluded only with explicit component/facet evidence"
            ),
            "go_output_policy": "0.070 kg output equivalent uses output solids plus neutral B6808 geometry; neutral has no separate mass",
            "residual_policy": "ledger CAD candidate members define geometric centroid; unavailable cable geometry is not fabricated",
            "gripper_policy": "complete 0.296 kg measured mass at ClosureAngle=0 assigned to exact 37-member geometric centroid; no internal mass guesses",
        },
        "com_status_definitions": {
            "CAD_BREP_CENTER_OF_MASS": "FreeCAD BRep single-solid geometric center of volume.",
            "CAD_COMPOUND_VOLUME_WEIGHTED_CENTER": "FreeCAD BRep multi-solid/member center weighted by absolute solid volume.",
            "CAD_HIGH_DETAIL_VISUAL_MESH_VOLUME_CENTROID": (
                "Volume centroid of retained positive-volume source high-detail visual-mesh components; "
                "each retained component is closed and manifold."
            ),
            "CAD_SOURCE_MESH_ZERO_VOLUME_ARTIFACT_EXCLUDED_VOLUME_CENTER": (
                "Source mesh volume center after explicitly excluding recorded zero-volume artifact components; "
                "every retained positive-volume component is closed and manifold."
            ),
            "MEASURED_TOTAL_MASS_WITH_EQUAL_DENSITY_VOLUME_WEIGHTED_GEOMETRIC_COM": (
                "Measured A+B print total placed at their equal-density volume-weighted geometric center; "
                "A uses the independently validated mass-properties-only artifact and B the accepted source mesh."
            ),
            "ENGINEERING_ESTIMATE_GEOMETRIC_CENTROID": "Residual mass placed at the traceable CAD member-set geometric centroid.",
            "MEASURED_TOTAL_MASS_WITH_GEOMETRIC_COM_ESTIMATE": "Measured total mass placed at complete assembly geometric centroid without internal mass guesses.",
            "UNRESOLVED": "No authoritative COM can be emitted; the build remains fail-closed.",
        },
        "links": links,
        "total_link2_to_gripper_mass_kg": float(total),
        "double_count_checks": {
            "additive_component_count": 17,
            "component_ids_unique": len({component_id for component_id in COMPONENT_ORDER}) == 17,
            "neutral_b6808_separate_mass_count": 0,
            "parent_compound_totals_added": False,
            "gripper_geometry_member_count": len(mass_components["GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP"]["cad_members"]),
            "pass": True,
        },
        "validation": {
            "mass_authority_locked": True,
            "cad_authority_locked": True,
            "all_component_mass_sums_pass": all(link["validation"]["component_mass_sum_pass"] for link in links.values()),
            "total_mass_expected_kg": float(EXPECTED_TOTAL_MASS),
            "total_mass_error_kg": float(abs(total - EXPECTED_TOTAL_MASS)),
            "total_mass_pass": total_mass_pass,
            "coordinate_round_trip_max_error_m": max(component_roundtrips, default=None),
            "coordinate_round_trip_tolerance_m": ROUNDTRIP_TOL_M,
            "all_resolved_component_round_trips_pass": all(error < ROUNDTRIP_TOL_M for error in component_roundtrips),
            "resolved_link_com_count": sum(link["validation"]["pass"] for link in links.values()),
            "all_links_resolved": all_links_resolved,
            "collision_proxy_used": False,
            "all_link_numeric_pass": all_link_numeric_pass,
            "all_assertions_pass": final_pass,
        },
        "unresolved_items": unresolved,
        "known_model_limitations": [
            "Residual CAD centroids omit cable geometry because no authoritative cable CAD is available; cable_geometry_available=false.",
            "The gripper COM is a static ClosureAngle=0 geometric estimate for a measured complete total, not an internal inertial allocation.",
            "UpperArm A/B mass allocation is a V1 equal-effective-density assumption because only their combined 0.5515 kg was measured.",
        ],
        "prohibited_outputs": {
            "mass_modified": False,
            "kinematics_tf_control_modified": False,
            "urdf_inertial_written": False,
            "mujoco_inertial_written": False,
            "inertia_tensor_written": False,
            "gravity_written": False,
            "damping_friction_armature_written": False,
        },
        "final_status": final_status,
    }


def json_text(document: dict[str, Any]) -> str:
    return json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n"


def format_vector(vector: list[float] | None) -> str:
    if vector is None:
        return "UNRESOLVED"
    return "[" + ", ".join(f"{value:.12g}" for value in vector) + "]"


def markdown_text(document: dict[str, Any]) -> str:
    upper_arm = next(
        component
        for component in document["links"]["link2"]["components"]
        if component["component_id"] == "UPPER_ARM_PRINT_MEASURED"
    )
    allocation = upper_arm["equal_density_volume_allocation"]
    lines = [
        "# V15.15 COM 账本 V1",
        "",
        f"**{document['final_status']}**",
        "",
        "> 本文档仅审计 link2 → gripper 的 COM。未计算惯量张量，未写入 URDF/MJCF inertial，未引入重力、阻尼、摩擦或电机动力学。",
        "",
        "## 权威锁定",
        "",
        f"- source commit: `{document['source_commit']}`",
        f"- mass ledger: `{document['mass_ledger_path']}` / `{document['mass_ledger_sha256']}`",
        f"- source CAD: `{document['cad_source']}` / `{document['cad_source_sha256']}`",
        "- source CAD 只读，未保存、未修改",
        "- collision proxy used: **NO**",
        "",
        "## Link 结果",
        "",
        "| Link | Mass (kg) | COM xyz in owner link frame (m) | Component mass sum | COM status |",
        "|---|---:|---|---|---|",
    ]
    for link_name in LINK_ORDER:
        link = document["links"][link_name]
        status = "PASS" if link["validation"]["pass"] else "FAIL / UNRESOLVED"
        lines.append(
            f"| {link_name} | {link['mass_kg']:.4f} | `{format_vector(link['com_xyz_m_in_link_frame'])}` | PASS | {status} |"
        )
    lines.extend(
        [
            "",
            "## UpperArm A 质量属性修复与 A/B 体积分配",
            "",
            f"- mass-properties-only artifact: `{upper_arm['mass_properties_artifact']['path']}` / "
            f"`{upper_arm['mass_properties_artifact']['sha256']}`",
            f"- repair report: `{upper_arm['mass_properties_artifact']['report_path']}` / "
            f"`{upper_arm['mass_properties_artifact']['report_sha256']}`",
            "- artifact 已独立重导入为 closed/manifold/valid solid；collision proxy used: **NO**",
            "- 上臂 A+B 总实测质量保持 `0.5515 kg`；按同材料统一等效密度进行真实闭合体积加权，未使用 50/50。",
            "",
            "| Part | Volume (mm^3) | Volume fraction | Explanatory allocated mass (kg) | COM world at zero (mm) |",
            "|---|---:|---:|---:|---|",
        ]
    )
    for part in allocation["parts"]:
        lines.append(
            f"| {part['member']} | {part['volume_mm3']:.12g} | {part['volume_fraction']:.12g} | "
            f"{part['allocated_mass_kg']:.12g} | `{format_vector(part['center_world_mm'])}` |"
        )
    lines.extend(
        [
            "",
            f"- upper-arm print COM world at zero (mm): `{format_vector(upper_arm['cad_com_world_mm'])}`",
            f"- upper-arm print COM in link2 (mm): `{format_vector(upper_arm['cad_com_link_mm'])}`",
            f"- link2 final COM in link frame (m): `{format_vector(document['links']['link2']['com_xyz_m_in_link_frame'])}`",
            f"- link2 final COM world at zero (m): `{format_vector(document['links']['link2']['com_xyz_m_world_at_zero'])}`",
        ]
    )
    lines.extend(
        [
            "",
            "## 组件级 COM 证据",
            "",
            "| Component | Owner | Mass (kg) | COM world at zero (mm) | COM owner-link (mm) | Method |",
            "|---|---|---:|---|---|---|",
        ]
    )
    for link_name in LINK_ORDER:
        for component in document["links"][link_name]["components"]:
            lines.append(
                f"| {component['component_id']} | {link_name} | {component['mass_kg']:.4f} | "
                f"`{format_vector(component['cad_com_world_mm'])}` | `{format_vector(component['cad_com_link_mm'])}` | "
                f"{component['com_status']} |"
            )
    lines.extend(
        [
            "",
            "## 阻塞项",
            "",
        ]
    )
    if document["unresolved_items"]:
        for unresolved in document["unresolved_items"]:
            lines.extend(
                [
                    f"- `{unresolved['code']}` / `{unresolved['component_id']}` / `{unresolved['member']}`",
                    f"  - reason: {unresolved['reason']}",
                    f"  - required resolution: {unresolved['required_resolution']}",
                ]
            )
        lines.append("- 影响：upper-arm 0.5515 kg 组件 COM、link2 COM 和全链 COM 无法权威冻结。")
    else:
        lines.append("- none")
    lines.extend(
        [
            "",
            "## 数值验证",
            "",
            f"- 六个 Link 组件质量和：{'PASS' if document['validation']['all_component_mass_sums_pass'] else 'FAIL'}",
            f"- 总质量 `{document['total_link2_to_gripper_mass_kg']:.4f} kg`：{'PASS' if document['validation']['total_mass_pass'] else 'FAIL'}",
            f"- 已解析组件坐标 round-trip 最大误差：`{document['validation']['coordinate_round_trip_max_error_m']:.12g} m`",
            f"- 全部 Link COM 解析：{'PASS' if document['validation']['all_links_resolved'] else 'FAIL'}",
            "- residual_A/B/C: `ENGINEERING_ESTIMATE_GEOMETRIC_CENTROID`, `cable_geometry_available=false`",
            "- gripper: `MEASURED_TOTAL_MASS_WITH_GEOMETRIC_COM_ESTIMATE`, `ClosureAngle=0`",
            "",
            "## 结论",
            "",
            (
                "六个 Link COM 已全部解析。UpperArm A 仅使用独立质量属性几何，"
                "未使用 collision proxy，未写入惯量/重力/动力学。"
                if document["validation"]["all_links_resolved"]
                else "COM 仍有阻塞，禁止使用 collision proxy 绕过。"
            ),
            "",
        ]
    )
    return "\n".join(lines)


def atomic_write(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)


def run(write: bool) -> int:
    root = repo_root()
    document = build_document(root)
    expected_json = json_text(document)
    expected_md = markdown_text(document)
    if write:
        atomic_write(root / OUTPUT_JSON, expected_json)
        atomic_write(root / OUTPUT_MD, expected_md)
        final_paths = changed_paths(root)
        if final_paths != ALLOWED_CHANGED_PATHS:
            raise AuditError("FINAL_CHANGED_PATH_SET_MISMATCH:" + ",".join(sorted(final_paths)))
        print(f"WROTE {OUTPUT_JSON}")
        print(f"WROTE {OUTPUT_MD}")
    else:
        if not (root / OUTPUT_JSON).is_file() or not (root / OUTPUT_MD).is_file():
            raise AuditError("COM_ARTIFACTS_MISSING")
        if (root / OUTPUT_JSON).read_text(encoding="utf-8") != expected_json:
            raise AuditError("COM_JSON_NOT_REPRODUCIBLE")
        if (root / OUTPUT_MD).read_text(encoding="utf-8") != expected_md:
            raise AuditError("COM_MARKDOWN_NOT_REPRODUCIBLE")
        print("COM_ARTIFACTS_REPRODUCIBLE = PASS")
    print(document["final_status"])
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--write", action="store_true", help="atomically write JSON and Markdown audit artifacts")
    mode.add_argument("--check", action="store_true", help="recompute and compare existing artifacts (default)")
    arguments = parser.parse_args()
    try:
        return run(write=arguments.write)
    except (AuditError, OSError, KeyError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"COM_LEDGER_BUILD_ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
