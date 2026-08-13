#!/usr/bin/env python3
"""Independent validator for the deterministic V15.16A occupied-volume FAIL audit.

The validator deliberately does not import the builder.  It reopens the
protected CAD and repaired print artifacts, rebuilds every one of the 266
mass-geometry atoms, independently classifies the 91 original positive-volume
pairs, and proves that the three reported stator slivers are exactly the
unresolved boolean blockers.  A zero exit status means that the FAIL evidence
is complete and honest; it never means that a mass-geometry authority was
established.  Only after those gates pass does it invoke the builder ``--check``
inside protected/output hash snapshots.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import struct
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    import FreeCAD as App
    import Mesh
    import MeshPart
    import Part
    import numpy as np
except ImportError as exc:  # pragma: no cover - runtime gate
    raise SystemExit("Run with FreeCAD Python (expected D:/freecad/bin/python.exe): " + str(exc))


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "bbd45c79eaefffa9813887937fd51739db1293d9"
SOURCE_TREE = "a6a806b016b96e00cfc4048b1d3a1de889eae779"
TARGET_BRANCH = "agent/v15-16-mass-geometry-authority-v1"

MASS_LEDGER = "V15_15_实测质量账本_v1.json"
COM_LEDGER = "V15_15_COM账本_v2.json"
SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
MEMBERSHIP = "rigid_links_v15_13/rigid_link_membership.csv"
MANIFEST = "rigid_links_v15_13/rigid_link_manifest.json"
BUILDER = "tools/build_mass_geometry_authority_v15_16.py"
VALIDATOR = "tools/validate_mass_geometry_authority_v15_16.py"
REPORT_JSON = "V15_16_质量几何去重审计_v1.json"
REPORT_MD = "V15_16_质量几何去重审计_v1.md"

ALLOWED_CHANGED_PATHS = {REPORT_JSON, REPORT_MD, BUILDER, VALIDATOR}

PRINT_PATHS = {
    "UpperArm_A_SleeveSide_PrintPart":
        "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties.stl",
    "UpperArm_B_Distal_PrintPart":
        "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties.stl",
    "Forearm_v3_HighDetail_Display":
        "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties.stl",
    "Wrist_Prelink_v1_HighDetail_Display":
        "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties.stl",
}

PROTECTED_HASHES = {
    MASS_LEDGER: "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    "V15_15_实测质量账本_v1.md": "fa52587a3309ea5665197283b62ac2e165e67663070408931611fa3b7b4e946d",
    COM_LEDGER: "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
    "V15_15_COM账本_v2.md": "64117a409bc37b70e436eef5fee18aa94b21239c3ccdf127adafc5f550c0e59f",
    "V15_15_COM账本_v1.json": "13e3470821541d7ae5c3f10223c51339d7d46df0d118a8e4dbeac2480c462d32",
    "V15_15_COM账本_v1.md": "8875ff34ab6c091b2d5de6fff418553b0834a141adf32f4c88aac7d1d916d11a",
    SOURCE_CAD: "b4a02a97afaff88e246a46340dad4cacd3c645a7456fdd087a69552e14d3ddc0",
    MEMBERSHIP: "3d2a4d52d679eb97917410c6e60bec22c0c269f91ed31b35fd53ff94167e1b8c",
    MANIFEST: "8db76f9228f7a60bd017276239e3669de3cfd443c8cd36d0dd555e184606384f",
    PRINT_PATHS["UpperArm_A_SleeveSide_PrintPart"]:
        "67aed62b304cfef3a338b0e59b035426b8e012619c333d032e8304c389352f45",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json":
        "a2b58c9892797b26c4511ff2581e7224d99cd27ca2261cc864252a6e38c21815",
    PRINT_PATHS["UpperArm_B_Distal_PrintPart"]:
        "bafb8dc8b06242a844196bc63971f95a0a7dd52834b2a05ab72f40b28460a081",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties_report.json":
        "dbe4c0751ab7bf4c1e564ce8b078f6de96336aaa203f060b402cc68fa6483e0f",
    PRINT_PATHS["Forearm_v3_HighDetail_Display"]:
        "631217db4ad44f53313aa4691bde3eaaec9e043c9ff83cc6549ea42bd2832760",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties_report.json":
        "29961b01d574e37ba28c4d7d12cba46b8836911044d29184d1457d7b0160d94f",
    PRINT_PATHS["Wrist_Prelink_v1_HighDetail_Display"]:
        "585d32a6ac54aca69a082947f31604e5f7b44e5be14eacab2c2db4f45a10f944",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties_report.json":
        "561d497524b3ed21349caa69f0659f157a2486c50861abe407fad15f8e753614",
    "V15_16_刚体惯量_FAIL审计_v2.json": "f8e0d62f027c932a49b5b38930ee770fca52c60ca2c3aa64b28308934cca55b8",
    "V15_16_刚体惯量_FAIL审计_v2.md": "94810a472a6e1a651f4a0b614e237d6241bc43bca3d7f66f908065ee26cdcfc7",
    "V15_16_打印件惯量几何适用性报告.json": "7308455fcdab006f68d90401c3206ce7209575feadfd700350122f96dc61cd14",
    "tools/build_inertia_ledger_v15_16_v2.py": "914d774f9abc4b5aa8638eec9b2533790e5ee446712358fbd33558d69c09e704",
    "tools/validate_inertia_ledger_v15_16_v2.py": "22aadb9d72a3b7e1952bf53c981c889fc251f585174424be8204556595776590",
    "tools/test_inertia_math_v15_16_v2.py": "141fcce0fb2b7af622f4c447bac5f5fa1ce72cf49745fb34aac6adac02c5e7fa",
}

COMPONENT_ORDER = (
    "J2A_OUTPUT_EQ", "J2B_OUTPUT_EQ", "UPPER_ARM_PRINT_MEASURED", "J3_OUTPUT_EQ",
    "J3_STATOR_EQ", "FOREARM_PRINT_MEASURED", "J4_STATOR_EQ",
    "RESIDUAL_C_LINK3_FOREARM_HARDWARE_WIRING", "J4_OUTPUT_EQ",
    "WRIST_PRELINK_PRINT_MEASURED", "J5_STATOR_EQ",
    "RESIDUAL_B_LINK4_J5_INTERFACE_HARDWARE_WIRING", "J5_OUTPUT_EQ",
    "J6_DM_STATOR_EQ", "RESIDUAL_A_LINK5_ADAPTER_HARDWARE_WIRING",
    "J6_DM_OUTPUT_EQ", "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP",
)
LINK_ORDER = ("link2", "link3", "link4", "link5", "link6", "gripper")
EXPECTED_LINK_MASSES = {
    "link2": 0.7615, "link3": 1.0900, "link4": 0.6750,
    "link5": 0.5040, "link6": 0.1250, "gripper": 0.2960,
}
EXPECTED_TOTAL_MASS = 3.4515
GO_OUTPUTS = {
    "J2A_OUTPUT_EQ": (30, 32), "J2B_OUTPUT_EQ": (30, 32),
    "J3_OUTPUT_EQ": (7, 8), "J4_OUTPUT_EQ": (7, 8), "J5_OUTPUT_EQ": (7, 8),
}
EXPECTED_CLASS_COUNTS = {
    "EXACT_DUPLICATE": 0, "FULL_CONTAINMENT": 5,
    "PARTIAL_INTERPENETRATION": 83, "NUMERICAL_SLIVER": 3,
}
EXPECTED_UNRESOLVED_PAIRS = {
    (
        "J3_STATOR_EQ",
        "J3_STATOR_EQ:J3_Motor_Stator_STEP_Display:solid_22",
        "J3_STATOR_EQ:J3_Motor_Stator_STEP_Display:solid_24",
    ),
    (
        "J4_STATOR_EQ",
        "J4_STATOR_EQ:J4_Motor_Stator_STEP_Display:solid_22",
        "J4_STATOR_EQ:J4_Motor_Stator_STEP_Display:solid_24",
    ),
    (
        "J5_STATOR_EQ",
        "J5_STATOR_EQ:J5_Motor_Stator_STEP_Display:solid_22",
        "J5_STATOR_EQ:J5_Motor_Stator_STEP_Display:solid_24",
    ),
}
FAIL_STATUS = "V15.16 MASS_GEOMETRY_AUTHORITY_V1 = FAIL"
FAIL_SCHEMA = "go-m8010-arm-v15.16-mass-geometry-authority-v1-fail-audit/1.0"

ORIGINAL_OVERLAP_TOL_MM3 = 1.0e-7
CANONICAL_ABSOLUTE_TOL_MM3 = 1.0e-9
CANONICAL_RELATIVE_TOL = 1.0e-12
CONTAINMENT_ABSOLUTE_TOL_MM3 = 1.0e-7
CONTAINMENT_RELATIVE_TOL = 1.0e-9
EXACT_VOLUME_REL_TOL = 1.0e-9
EXACT_BBOX_TOL_MM = 1.0e-7
EXACT_CENTROID_TOL_MM = 1.0e-7
EXACT_SURFACE_REL_TOL = 1.0e-9
SLIVER_FRACTION_MAX = 1.0e-6
VOLUME_REL_LIMIT = 1.0e-6
COM_LIMIT_M = 1.0e-6
MM_TO_M = 1.0e-3
PATH_B_DEFLECTION_LADDER_MM = (1.0e-4, 3.0e-5, 1.0e-5, 3.0e-6)
PATH_B_ANGULAR_DEFLECTION_RAD = 0.1


class ValidationError(RuntimeError):
    pass


def require(condition: bool, code: str) -> None:
    if not condition:
        raise ValidationError(code)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().lower()


def run_git(*args: str, check: bool = True) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        ["git", "-C", str(ROOT), "-c", "core.quotepath=false", *args],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=check,
    )


def changed_paths() -> set[str]:
    tracked = {
        item.decode("utf-8").replace("\\", "/")
        for item in run_git("diff", "--name-only", "-z", SOURCE_COMMIT, "--").stdout.split(b"\0") if item
    }
    untracked = {
        item.decode("utf-8").replace("\\", "/")
        for item in run_git("ls-files", "--others", "--exclude-standard", "-z").stdout.split(b"\0") if item
    }
    return tracked | untracked


def verify_git_scope() -> None:
    branch = run_git("branch", "--show-current").stdout.decode().strip()
    require(branch == TARGET_BRANCH, f"WRONG_BRANCH:{branch}")
    tree = run_git("rev-parse", f"{SOURCE_COMMIT}^{{tree}}").stdout.decode().strip()
    require(tree == SOURCE_TREE, f"SOURCE_TREE_MISMATCH:{tree}")
    require(run_git("merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD", check=False).returncode == 0,
            "SOURCE_COMMIT_NOT_ANCESTOR")
    actual = changed_paths()
    require(actual == ALLOWED_CHANGED_PATHS, f"CHANGED_PATHS_NOT_EXACT:{sorted(actual)}")
    prohibited_suffixes = (".urdf", ".xacro", ".srdf", ".mjcf")
    require(not any(path.lower().endswith(prohibited_suffixes) for path in actual), "DEPLOYMENT_CHANGED")


def protected_snapshot() -> dict[str, str]:
    result = {}
    for relative, expected in PROTECTED_HASHES.items():
        path = ROOT / relative
        require(path.is_file(), f"PROTECTED_MISSING:{relative}")
        actual = sha256_file(path)
        require(actual == expected, f"PROTECTED_HASH:{relative}:{actual}")
        result[relative] = actual
    return result


def output_snapshot() -> dict[str, str]:
    result = {}
    for relative in sorted(ALLOWED_CHANGED_PATHS):
        path = ROOT / relative
        require(path.is_file(), f"OUTPUT_MISSING:{relative}")
        result[relative] = sha256_file(path)
    return result


def validate_report_provenance(report: dict[str, Any]) -> None:
    provenance = report["provenance"]
    require(provenance["source_commit"] == SOURCE_COMMIT, "REPORT_SOURCE_COMMIT")
    require(provenance["source_tree"] == SOURCE_TREE, "REPORT_SOURCE_TREE")
    require(provenance["target_branch"] == TARGET_BRANCH, "REPORT_TARGET_BRANCH")
    require(provenance["source_commit_strictly_locked"] is True, "REPORT_SOURCE_NOT_LOCKED")

    guard = report["git_scope_guard"]
    require(guard["target_branch"] == TARGET_BRANCH, "REPORT_GIT_BRANCH")
    require(guard["source_commit"] == SOURCE_COMMIT and guard["source_tree"] == SOURCE_TREE,
            "REPORT_GIT_BASELINE")
    require(guard["source_commit_is_ancestor"] is True and guard["pass"] is True,
            "REPORT_GIT_GUARD_STATUS")
    require(set(guard["allowed_changed_paths"]) == ALLOWED_CHANGED_PATHS,
            "REPORT_GIT_ALLOWLIST")
    require(guard["unexpected_changed_paths"] == [], "REPORT_GIT_UNEXPECTED_PATHS")

    authorities = report["authorities"]
    for key, relative in (("mass", MASS_LEDGER), ("com_v2", COM_LEDGER), ("geometry", SOURCE_CAD)):
        authority = authorities[key]
        require(authority["path"] == relative, f"REPORT_AUTHORITY_PATH:{key}")
        require(authority["sha256"].lower() == PROTECTED_HASHES[relative],
                f"REPORT_AUTHORITY_HASH:{key}")
    require(authorities["geometry"]["opened_without_save"] is True,
            "REPORT_GEOMETRY_NOT_READ_ONLY")
    require(authorities["geometry"]["collision_proxy_used"] is False,
            "REPORT_GEOMETRY_COLLISION_PROXY")

    expected = dict(PROTECTED_HASHES)
    for snapshot_name in ("protected_inputs_before", "protected_inputs_after"):
        records = authorities[snapshot_name]
        require(isinstance(records, list) and len(records) == len(expected),
                f"REPORT_PROTECTED_RECORD_COUNT:{snapshot_name}")
        by_path = {str(record["path"]): record for record in records}
        require(len(by_path) == len(records) and set(by_path) == set(expected),
                f"REPORT_PROTECTED_PATH_SET:{snapshot_name}")
        for relative, digest in expected.items():
            record = by_path[relative]
            require(str(record["sha256"]).lower() == digest and record["locked"] is True,
                    f"REPORT_PROTECTED_RECORD:{snapshot_name}:{relative}")
    require(authorities["protected_inputs_before"] == authorities["protected_inputs_after"],
            "REPORT_PROTECTED_SNAPSHOTS_DIFFER")
    require(authorities["all_protected_inputs_unchanged"] is True,
            "REPORT_PROTECTED_UNCHANGED_FLAG")



def v3(value: Any) -> np.ndarray:
    if hasattr(value, "x"):
        return np.array([float(value.x), float(value.y), float(value.z)], dtype=float)
    return np.asarray(value, dtype=float).reshape(3)


def bbox_intersects(left: Any, right: Any, tol: float = 1.0e-9) -> bool:
    return not (
        left.XMax < right.XMin - tol or right.XMax < left.XMin - tol
        or left.YMax < right.YMin - tol or right.YMax < left.YMin - tol
        or left.ZMax < right.ZMin - tol or right.ZMax < left.ZMin - tol
    )


def bbox_values(shape: Any) -> np.ndarray:
    box = shape.BoundBox
    return np.array([box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax], dtype=float)


def positive_solid(shape_input: Any) -> Any:
    """Normalize a protected source atom using only shallow source gates.

    The deterministic FAIL audit intentionally investigates source BReps that
    are positive, closed, and shallow-valid but fail OCCT's deep BOP checker.
    Deep validity is therefore evidence gathered by the bounded blocker gates,
    not a prerequisite for reconstructing the original 266-atom population.
    """
    shape = shape_input.copy()
    if float(shape.Volume) < 0.0:
        shape.reverse()
    require(float(shape.Volume) > 0.0, "NONPOSITIVE_SOLID")
    require(shape.isValid() and shape.isClosed(), "INVALID_OR_OPEN_SOLID")
    return shape


def placement_from_rows(rows: Sequence[Sequence[float]]) -> Any:
    matrix = App.Matrix()
    value = np.asarray(rows, dtype=float).reshape(4, 4)
    for row in range(4):
        for column in range(4):
            setattr(matrix, f"A{row + 1}{column + 1}", float(value[row, column]))
    return App.Placement(matrix)


def parse_binary_stl(path: Path) -> tuple[list[np.ndarray], list[tuple[int, int, int]]]:
    data = path.read_bytes()
    require(len(data) >= 84, f"STL_TOO_SHORT:{path.name}")
    count = struct.unpack_from("<I", data, 80)[0]
    require(len(data) == 84 + 50 * count, f"STL_NOT_BINARY:{path.name}")
    vertices: list[np.ndarray] = []
    faces: list[tuple[int, int, int]] = []
    welded: dict[tuple[float, float, float], int] = {}
    for face_index in range(count):
        record = struct.unpack_from("<12fH", data, 84 + 50 * face_index)
        face = []
        for offset in (3, 6, 9):
            point = tuple(float(record[offset + axis]) for axis in range(3))
            require(all(math.isfinite(value) for value in point), f"STL_NONFINITE:{path.name}")
            if point not in welded:
                welded[point] = len(vertices)
                vertices.append(np.array(point, dtype=float))
            face.append(welded[point])
        faces.append(tuple(face))
    return vertices, faces


def topology_gate(vertices: Sequence[np.ndarray], faces: Sequence[Sequence[int]]) -> None:
    edges: dict[tuple[int, int], list[int]] = defaultdict(list)
    for face in faces:
        a, b, c = (int(value) for value in face)
        require(len({a, b, c}) == 3, "DEGENERATE_FACE_INDEX")
        require(float(np.linalg.norm(np.cross(vertices[b] - vertices[a], vertices[c] - vertices[a]))) > 0.0,
                "DEGENERATE_FACE_AREA")
        for start, end in ((a, b), (b, c), (c, a)):
            key = (min(start, end), max(start, end))
            edges[key].append(1 if (start, end) == key else -1)
    require(all(len(signs) == 2 and sum(signs) == 0 for signs in edges.values()), "MESH_NOT_WATERTIGHT_OR_ORIENTED")


def anchored_volume_com(vertices: Sequence[np.ndarray], faces: Sequence[Sequence[int]]) -> tuple[float, np.ndarray]:
    topology_gate(vertices, faces)
    points = np.vstack(vertices)
    anchor = 0.5 * (np.min(points, axis=0) + np.max(points, axis=0))
    signed_volumes: list[float] = []
    first = [[], [], []]
    for face in faces:
        a, b, c = (vertices[int(index)] - anchor for index in face)
        volume = float(a @ np.cross(b, c)) / 6.0
        signed_volumes.append(volume)
        coordinate_sum = a + b + c
        for axis in range(3):
            first[axis].append(volume * float(coordinate_sum[axis]) / 4.0)
    signed_total = math.fsum(signed_volumes)
    require(math.isfinite(signed_total) and signed_total != 0.0, "MESH_ZERO_VOLUME")
    sign = 1.0 if signed_total > 0.0 else -1.0
    volume = abs(signed_total)
    offset = np.array([sign * math.fsum(values) for values in first]) / volume
    return volume, anchor + offset


def part_from_mesh(path: Path) -> Any:
    mesh = Mesh.Mesh(str(path))
    require(mesh.isSolid() and not mesh.hasNonManifolds() and not mesh.hasSelfIntersections(),
            f"PRINT_MESH_INVALID:{path.name}")
    shape = Part.Shape()
    shape.makeShapeFromMesh(mesh.Topology, 0.001)
    require(len(shape.Shells) == 1 and shape.isClosed() and shape.isValid(), f"PRINT_SHELL_INVALID:{path.name}")
    return positive_solid(Part.makeSolid(shape.Shells[0]))


def load_authorities() -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    mass = json.loads((ROOT / MASS_LEDGER).read_text(encoding="utf-8"))
    com = json.loads((ROOT / COM_LEDGER).read_text(encoding="utf-8"))
    require(com["final_status"] == "V15.15 COM_LEDGER_V2 = PASS", "COM_V2_NOT_PASS")
    require(tuple(com["component_order"]) == COMPONENT_ORDER, "COMPONENT_ORDER")
    components = list(com["components"])
    require(len(components) == 17, "COMPONENT_COUNT")
    mass_entries = mass["component_to_link_mapping"]["additive_components"]
    mass_map = {entry["component_id"]: entry for entry in mass_entries}
    require(set(mass_map) == set(COMPONENT_ORDER), "MASS_COMPONENT_SET")
    for component in components:
        component_id = component["component_id"]
        authority = mass_map[component_id]
        require(authority["ledger_link"] == component["owner_link"], f"OWNER:{component_id}")
        require(abs(float(authority["nominal_mass_kg"]) - float(component["frozen_mass_kg"])) < 1.0e-15,
                f"MASS:{component_id}")
        require(authority["cad_members"] == component["geometry_member_order_inherited_exactly_from_v1_mass_ledger"],
                f"MEMBERSHIP:{component_id}")
    return mass, com, components, mass_map


def build_print_cache(components: Sequence[dict[str, Any]]) -> dict[str, Any]:
    by_path = {
        member.get("artifact_path"): member
        for component in components for member in component.get("members", []) if member.get("artifact_path")
    }
    cache = {}
    for relative in PRINT_PATHS.values():
        require(relative in by_path, f"PRINT_NOT_REFERENCED:{relative}")
        vertices, faces = parse_binary_stl(ROOT / relative)
        volume, center = anchored_volume_com(vertices, faces)
        solid = part_from_mesh(ROOT / relative)
        occt_volume = float(solid.Volume)
        occt_center = v3(solid.CenterOfMass)
        require(abs(occt_volume - volume) / volume < 1.0e-8, f"PRINT_VOLUME_PATHS:{relative}")
        require(float(np.linalg.norm(occt_center - center)) * MM_TO_M < 1.0e-7, f"PRINT_COM_PATHS:{relative}")
        cache[relative] = {"member": by_path[relative], "solid": solid, "volume": occt_volume, "center": occt_center}
    return cache


def build_atoms(doc: Any, components: Sequence[dict[str, Any]], print_cache: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {}
    for component in components:
        component_id = component["component_id"]
        atoms = []
        for member in component["members"]:
            token = str(member["member"])
            require("collision" not in token.lower() and "proxy" not in token.lower(), f"COLLISION_PROXY:{token}")
            if member.get("artifact_path"):
                cached = print_cache[member["artifact_path"]]
                shape = cached["solid"].copy()
                shape.Placement = placement_from_rows(member["source_placement_matrix_row_major"])
                shape = positive_solid(shape)
                atoms.append({
                    "atom_id": f"{component_id}:{token}", "member": token,
                    "source_object": token, "solid_index": 1, "shape": shape,
                    "volume": float(shape.Volume), "center": v3(shape.CenterOfMass),
                })
                continue
            object_name = str(member.get("source_object") or token)
            obj = doc.getObject(object_name)
            require(obj is not None and obj.TypeId == "Part::Feature", f"CAD_OBJECT:{object_name}")
            solids = list(obj.Shape.Solids)
            selected = member.get("selected_solid_indices")
            indices = [int(value) for value in selected] if selected else list(range(1, len(solids) + 1))
            require(indices and all(1 <= value <= len(solids) for value in indices), f"SOLID_SELECTION:{token}")
            for index in indices:
                shape = positive_solid(solids[index - 1])
                atoms.append({
                    "atom_id": f"{component_id}:{token}:solid_{index}", "member": token,
                    "source_object": object_name, "solid_index": index, "shape": shape,
                    "volume": float(shape.Volume), "center": v3(shape.CenterOfMass),
                })
        require(atoms, f"NO_ATOMS:{component_id}")
        result[component_id] = atoms
    require(sum(len(atoms) for atoms in result.values()) == 266, "ATOM_COUNT")
    return result


def topology_signature(shape: Any) -> tuple[Any, ...]:
    return (
        str(shape.ShapeType), len(shape.Solids), len(shape.Shells), len(shape.Faces),
        len(shape.Wires), len(shape.Edges), len(shape.Vertexes),
    )


def atom_metrics(atom: dict[str, Any]) -> dict[str, Any]:
    shape = atom["shape"]
    box = shape.BoundBox
    return {
        "volume_mm3": float(atom["volume"]),
        "centroid_world_mm": v3(atom["center"]),
        "bbox_world_mm": {
            "min_mm": np.array([box.XMin, box.YMin, box.ZMin], dtype=float),
            "max_mm": np.array([box.XMax, box.YMax, box.ZMax], dtype=float),
            "size_mm": np.array([box.XLength, box.YLength, box.ZLength], dtype=float),
        },
        "surface_area_mm2": float(shape.Area),
        "topology_signature": {
            "shape_type": str(shape.ShapeType),
            "solids": len(shape.Solids), "shells": len(shape.Shells), "faces": len(shape.Faces),
            "wires": len(shape.Wires), "edges": len(shape.Edges), "vertices": len(shape.Vertexes),
        },
    }


def relative_difference(left: float, right: float) -> float:
    return abs(left - right) / max(abs(left), abs(right), 1.0e-300)


def classify_pair(
    left: dict[str, Any], right: dict[str, Any], common_volume: float,
) -> tuple[str, dict[str, float], dict[str, Any]]:
    left_volume, right_volume = float(left["volume"]), float(right["volume"])
    smaller = min(left_volume, right_volume)
    fractions = {
        "left": common_volume / left_volume,
        "right": common_volume / right_volume,
        "smaller": common_volume / smaller,
    }
    containment_tolerance = max(CONTAINMENT_ABSOLUTE_TOL_MM3, CONTAINMENT_RELATIVE_TOL * smaller)
    contained = smaller - common_volume <= containment_tolerance
    volume_relative_difference = relative_difference(left_volume, right_volume)
    volume_equal = volume_relative_difference <= EXACT_VOLUME_REL_TOL
    symmetric_difference = max(0.0, left_volume + right_volume - 2.0 * common_volume)
    symmetric_tolerance = max(
        CONTAINMENT_ABSOLUTE_TOL_MM3,
        CONTAINMENT_RELATIVE_TOL * max(left_volume, right_volume),
    )
    bbox_delta = float(np.max(np.abs(bbox_values(left["shape"]) - bbox_values(right["shape"]))))
    center_delta = float(np.linalg.norm(left["center"] - right["center"]))
    surface_relative_difference = relative_difference(float(left["shape"].Area), float(right["shape"].Area))
    topology_equal = topology_signature(left["shape"]) == topology_signature(right["shape"])
    exact = (
        volume_equal
        and left_volume - common_volume <= symmetric_tolerance
        and right_volume - common_volume <= symmetric_tolerance
        and symmetric_difference <= symmetric_tolerance
        and bbox_delta <= EXACT_BBOX_TOL_MM
        and center_delta <= EXACT_CENTROID_TOL_MM
        and surface_relative_difference <= EXACT_SURFACE_REL_TOL
        and topology_equal
    )
    if exact:
        classification = "EXACT_DUPLICATE"
    elif contained:
        classification = "FULL_CONTAINMENT"
    elif fractions["smaller"] <= SLIVER_FRACTION_MAX:
        classification = "NUMERICAL_SLIVER"
    else:
        classification = "PARTIAL_INTERPENETRATION"
    evidence = {
        "volume_relative_difference": volume_relative_difference,
        "volume_equal": volume_equal,
        "smaller_minus_common_mm3": smaller - common_volume,
        "containment_tolerance_mm3": containment_tolerance,
        "smaller_fully_contained": contained,
        "bbox_max_coordinate_delta_mm": bbox_delta,
        "centroid_distance_mm": center_delta,
        "surface_area_relative_difference": surface_relative_difference,
        "topology_signature_equal": topology_equal,
        "exact_duplicate_all_gates_pass": exact,
    }
    return classification, fractions, evidence


def classify_original_pairs(atoms_by_component: dict[str, list[dict[str, Any]]]) -> tuple[list[dict[str, Any]], int]:
    events = []
    pair_count = 0
    for component_id in COMPONENT_ORDER:
        for left_index, right_index in itertools.combinations(range(len(atoms_by_component[component_id])), 2):
            pair_count += 1
            left, right = atoms_by_component[component_id][left_index], atoms_by_component[component_id][right_index]
            if not bbox_intersects(left["shape"].BoundBox, right["shape"].BoundBox):
                continue
            try:
                common = left["shape"].common(right["shape"])
                common_volume = abs(float(common.Volume)) if not common.isNull() else 0.0
            except Exception as exc:
                raise ValidationError(f"RAW_COMMON_FAILED:{component_id}:{left_index}:{right_index}:{exc}") from exc
            if common_volume <= ORIGINAL_OVERLAP_TOL_MM3:
                continue
            classification, fractions, evidence = classify_pair(left, right, common_volume)
            events.append({
                "pair_order": len(events) + 1,
                "component_id": component_id, "left": left, "right": right,
                "left_index": left_index, "right_index": right_index,
                "common_volume": common_volume, "fractions": fractions,
                "classification": classification, "classification_evidence": evidence,
            })
    require(pair_count == 3899, f"RAW_PAIR_COUNT:{pair_count}")
    require(len(events) == 91, f"RAW_OVERLAP_COUNT:{len(events)}")
    counts = Counter(event["classification"] for event in events)
    require({key: counts.get(key, 0) for key in EXPECTED_CLASS_COUNTS} == EXPECTED_CLASS_COUNTS,
            f"CLASSIFICATION_COUNTS:{dict(counts)}")
    return events, pair_count


def connected_components(size: int, edges: Iterable[tuple[int, int]]) -> list[list[int]]:
    parent = list(range(size))

    def find(value: int) -> int:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    def union(left: int, right: int) -> None:
        a, b = find(left), find(right)
        if a != b:
            parent[max(a, b)] = min(a, b)

    for left, right in edges:
        union(left, right)
    groups: dict[int, list[int]] = defaultdict(list)
    for index in range(size):
        groups[find(index)].append(index)
    return sorted((sorted(values) for values in groups.values()), key=lambda values: values[0])


def subtract_occupied(candidate: Any, occupied: Sequence[Any], tolerance: float) -> list[Any]:
    """Return the exact portion of candidate not already occupied.

    This is an occupied-set partition, hence exactly equivalent to a union,
    while avoiding a shallow-valid/self-intersecting monolithic fuse result.
    The resulting pieces are required to be valid closed positive solids.
    """
    remainders = [candidate.copy()]
    for prior in occupied:
        next_remainders = []
        for remainder in remainders:
            before_volume = abs(float(remainder.Volume))
            if not bbox_intersects(remainder.BoundBox, prior.BoundBox):
                next_remainders.append(remainder)
                continue
            common = remainder.common(prior)
            common_volume = 0.0 if common.isNull() else abs(float(common.Volume))
            if common_volume <= tolerance:
                next_remainders.append(remainder)
                continue
            cut = remainder.cut(prior)
            if cut.isNull() or abs(float(cut.Volume)) <= tolerance:
                require(common_volume + tolerance >= before_volume,
                        f"CUT_DISCARDED_NONCOVERED_REMAINDER:{before_volume}:{common_volume}")
                continue
            cut_solids = [positive_solid(solid) for solid in cut.Solids]
            after_volume = math.fsum(float(solid.Volume) for solid in cut_solids)
            require(after_volume <= before_volume + tolerance,
                    f"CUT_VOLUME_INCREASE:{before_volume}:{after_volume}:{tolerance}")
            require(abs((after_volume + common_volume) - before_volume) <= max(tolerance, 1.0e-10 * before_volume),
                    f"CUT_VOLUME_NOT_CONSERVED:{before_volume}:{after_volume}:{common_volume}")
            next_remainders.extend(cut_solids)
        remainders = next_remainders
        if not remainders:
            break
    return remainders


def occupied_partition(atoms: Sequence[dict[str, Any]]) -> list[Any]:
    canonical: list[Any] = []
    tolerance = max(1.0e-9, 1.0e-12 * math.fsum(float(atom["volume"]) for atom in atoms))
    for atom in sorted(atoms, key=lambda item: (-float(item["volume"]), item["atom_id"])):
        canonical.extend(subtract_occupied(atom["shape"], canonical, tolerance))
    return canonical


def union_component(
    atoms: Sequence[dict[str, Any]], component_events: Sequence[dict[str, Any]], recipe_method: str
) -> list[Any]:
    by_id = {atom["atom_id"]: atom for atom in atoms}
    dropped: set[str] = set()
    for event in component_events:
        if event["classification"] not in {"EXACT_DUPLICATE", "FULL_CONTAINMENT"}:
            continue
        left_id, right_id = event["left"]["atom_id"], event["right"]["atom_id"]
        if left_id in dropped or right_id in dropped:
            continue
        if event["classification"] == "EXACT_DUPLICATE":
            _, drop = sorted((left_id, right_id))
        else:
            left_volume, right_volume = by_id[left_id]["volume"], by_id[right_id]["volume"]
            drop = right_id if left_volume > right_volume else left_id
        dropped.add(drop)
    survivors = [atom for atom in atoms if atom["atom_id"] not in dropped]
    require(survivors, "CONTAINMENT_COLLAPSE_REMOVED_ALL")

    union_trigger = max(1.0e-9, 1.0e-12 * math.fsum(atom["volume"] for atom in atoms))
    index_by_id = {atom["atom_id"]: index for index, atom in enumerate(survivors)}
    edge_pairs = [
        (index_by_id[event["left"]["atom_id"]], index_by_id[event["right"]["atom_id"]])
        for event in component_events
        if event["common_volume"] > union_trigger
        and event["left"]["atom_id"] in index_by_id and event["right"]["atom_id"] in index_by_id
    ]
    if recipe_method in {
        "OCCT_SEQUENTIAL_CUT_EXACT_OCCUPIED_SET_PARTITION_LAST_RESORT",
        "OCCT_GENERAL_FUSE_EXACT_OCCUPIED_SET_CELL_PARTITION_FALLBACK",
    }:
        # Use an independently implemented sequential exact partition for both
        # builder fallback labels.  Agreement is checked by occupied volume,
        # COM, pairwise disjointness, source coverage, and canonical metrics;
        # the validator does not call or import the builder's route.
        canonical = occupied_partition(survivors)
    else:
        require(recipe_method == "CLASSIFICATION_COLLAPSE_PLUS_OCCT_MULTIFUSE_REMOVE_SPLITTER",
                f"UNRECOGNIZED_CANONICAL_RECIPE:{recipe_method}")
        canonical = []
        for indices in connected_components(len(survivors), edge_pairs):
            cluster = sorted((survivors[index] for index in indices), key=lambda atom: atom["atom_id"])
            if len(cluster) == 1:
                solids = [positive_solid(cluster[0]["shape"])]
            else:
                try:
                    shapes = [atom["shape"].copy() for atom in cluster]
                    result = shapes[0].multiFuse(shapes[1:]).removeSplitter()
                    solids = [positive_solid(solid) for solid in result.Solids if float(solid.Volume) > 1.0e-9]
                except Exception as exc:
                    raise ValidationError(f"UNION_FAILED:{atoms[0]['atom_id']}:{indices}:{exc}") from exc
            require(solids, f"UNION_NO_SOLIDS:{atoms[0]['atom_id']}:{indices}")
            canonical.extend(solids)
    canonical.sort(key=lambda shape: tuple(round(value, 9) for value in (*v3(shape.CenterOfMass), float(shape.Volume))))
    return canonical


def aggregate_volume_com(shapes: Sequence[Any]) -> tuple[float, np.ndarray]:
    volumes = [float(shape.Volume) for shape in shapes]
    total = math.fsum(volumes)
    require(total > 0.0, "UNION_ZERO_VOLUME")
    center = np.array([
        math.fsum(volume * float(shape.CenterOfMass[axis]) for volume, shape in zip(volumes, shapes)) / total
        for axis in range(3)
    ])
    return total, center


def canonical_overlap_gate(component_id: str, shapes: Sequence[Any], union_volume: float) -> None:
    tolerance = max(1.0e-9, 1.0e-12 * union_volume)
    for left, right in itertools.combinations(shapes, 2):
        if not bbox_intersects(left.BoundBox, right.BoundBox):
            continue
        common = left.common(right)
        volume = abs(float(common.Volume)) if not common.isNull() else 0.0
        require(volume <= tolerance, f"CANONICAL_POSITIVE_OVERLAP:{component_id}:{volume}:{tolerance}")


def occupied_set_coverage_gate(
    component_id: str, atoms: Sequence[dict[str, Any]], canonical: Sequence[Any], union_volume: float
) -> list[dict[str, Any]]:
    tolerance = max(1.0e-9, 1.0e-12 * union_volume)
    records = []
    for atom in atoms:
        terms = []
        for occupied in canonical:
            if not bbox_intersects(atom["shape"].BoundBox, occupied.BoundBox):
                continue
            common = atom["shape"].common(occupied)
            common_volume = 0.0 if common.isNull() else abs(float(common.Volume))
            if common_volume > 0.0:
                terms.append(common_volume)
        covered = math.fsum(terms)
        atom_volume = float(atom["volume"])
        uncovered = max(0.0, atom_volume - covered)
        overshoot = max(0.0, covered - atom_volume)
        local_tolerance = max(tolerance, 1.0e-12 * atom_volume)
        require(uncovered <= local_tolerance and overshoot <= local_tolerance,
                f"CANONICAL_COVERAGE:{component_id}:{atom['atom_id']}:{uncovered}:{overshoot}:{local_tolerance}")
        records.append({
            "atom_id": atom["atom_id"], "raw_atom_volume_mm3": atom_volume,
            "covered_by_canonical_volume_mm3": covered, "uncovered_volume_mm3": uncovered,
            "coverage_overshoot_mm3": overshoot, "tolerance_mm3": local_tolerance, "pass": True,
        })
    return records


def canonical_solid_metric_record(shape: Any, index: int) -> dict[str, Any]:
    metrics = atom_metrics({"shape": shape, "volume": float(shape.Volume), "center": v3(shape.CenterOfMass)})
    deep_errors = shape.check(True)
    errors = [] if deep_errors is None else sorted(str(item) for item in deep_errors)
    return {
        "canonical_solid_index_one_based": index,
        "stable_identity": f"derived_component_occupied_solid_{index:04d}",
        **metrics,
        "is_valid": bool(shape.isValid()), "is_closed": bool(shape.isClosed()),
        "deep_bop_check_errors": errors,
        "deep_valid": bool(shape.isValid() and shape.isClosed() and float(shape.Volume) > 0.0 and not errors),
    }


def tessellated_union_volume_com(
    shapes: Sequence[Any], deflection_mm: float,
) -> tuple[float, np.ndarray, int, int]:
    volumes = []
    centers = []
    point_count = 0
    triangle_count = 0
    for shape in shapes:
        mesh = MeshPart.meshFromShape(
            Shape=shape.cleaned(), LinearDeflection=float(deflection_mm),
            AngularDeflection=0.1, Relative=False,
        )
        working = Mesh.Mesh(mesh)
        working.harmonizeNormals()
        working.fixDegenerations()
        require(working.isSolid() and not working.hasNonManifolds(), "CANONICAL_MESH_INVALID")
        points, facets = working.Topology
        vertices = [v3(point) for point in points]
        faces = [tuple(int(index) for index in face) for face in facets]
        volume, center = anchored_volume_com(vertices, faces)
        volumes.append(volume)
        centers.append(center)
        point_count += int(working.CountPoints)
        triangle_count += int(working.CountFacets)
    total = math.fsum(volumes)
    center = np.array([
        math.fsum(volume * float(value[axis]) for volume, value in zip(volumes, centers)) / total
        for axis in range(3)
    ])
    return total, center, point_count, triangle_count


def crosscheck_path_b(component_id: str, shapes: Sequence[Any], volume_a: float, center_a: np.ndarray,
                      reported_component: dict[str, Any]) -> tuple[float, float]:
    selected = reported_component["path_b_anchored_tetra"]["selected_linear_deflection_mm"]
    deflection = float(selected)
    require(deflection in PATH_B_DEFLECTION_LADDER_MM and deflection <= 1.0e-4,
            f"PATH_B_DEFLECTION:{component_id}:{deflection}")
    volume_b, center_b, point_count, triangle_count = tessellated_union_volume_com(shapes, deflection)
    relative_volume = abs(volume_b - volume_a) / volume_a
    center_error_m = float(np.linalg.norm(center_b - center_a)) * MM_TO_M
    require(relative_volume < VOLUME_REL_LIMIT, f"PATH_B_VOLUME:{component_id}:{relative_volume}")
    require(center_error_m < COM_LIMIT_M, f"PATH_B_COM:{component_id}:{center_error_m}")
    reported_path_b = reported_component["path_b_anchored_tetra"]
    require(reported_path_b["method"] ==
            "FINE_WATERTIGHT_TESSELLATION_BBOX_ANCHORED_SIGNED_TETRAHEDRA",
            f"REPORT_PATH_B_METHOD:{component_id}")
    require(reported_path_b["mesh_volume_or_center_of_gravity_api_used"] is False,
            f"REPORT_PATH_B_FORBIDDEN_MESH_API:{component_id}")
    attempts = reported_path_b["attempts"]
    require(isinstance(attempts, list) and attempts,
            f"REPORT_PATH_B_ATTEMPTS:{component_id}")
    require([float(attempt["linear_deflection_mm"]) for attempt in attempts]
            == list(PATH_B_DEFLECTION_LADDER_MM[:len(attempts)]),
            f"REPORT_PATH_B_LADDER:{component_id}")
    require(float(attempts[-1]["linear_deflection_mm"]) == deflection
            and attempts[-1]["pass"] is True
            and all(attempt["pass"] is False for attempt in attempts[:-1]),
            f"REPORT_PATH_B_SELECTION:{component_id}")
    compare_scalar(attempts[-1]["angular_deflection_rad"], PATH_B_ANGULAR_DEFLECTION_RAD, 0.0,
                   f"REPORT_PATH_B_ANGULAR:{component_id}")
    compare_scalar(reported_path_b["selected_volume_mm3"], volume_b, 1.0e-8,
                   f"REPORT_PATH_B_SELECTED_VOLUME:{component_id}")
    require(float(np.linalg.norm(v3(reported_path_b["selected_com_world_mm"]) - center_b)) <= 1.0e-8,
            f"REPORT_PATH_B_SELECTED_COM:{component_id}")
    require(int(reported_path_b["selected_point_count"]) == point_count,
            f"REPORT_PATH_B_POINT_COUNT:{component_id}")
    require(int(reported_path_b["selected_triangle_count"]) == triangle_count,
            f"REPORT_PATH_B_TRIANGLE_COUNT:{component_id}")
    require(int(attempts[-1]["point_count"]) == point_count
            and int(attempts[-1]["triangle_count"]) == triangle_count,
            f"REPORT_PATH_B_ATTEMPT_COUNTS:{component_id}")
    compare_scalar(attempts[-1]["volume_mm3"], volume_b, 1.0e-8,
                   f"REPORT_PATH_B_ATTEMPT_VOLUME:{component_id}")
    compare_vector(attempts[-1]["com_world_mm"], center_b, 1.0e-8,
                   f"REPORT_PATH_B_ATTEMPT_COM:{component_id}")
    agreement = reported_component["path_a_b_agreement"]
    require(abs(float(agreement["volume_relative_error"]) - relative_volume) < 1.0e-10,
            f"REPORT_PATH_B_VOLUME:{component_id}")
    require(abs(float(agreement["com_difference_m"]) - center_error_m) < 1.0e-10,
            f"REPORT_PATH_B_COM:{component_id}")
    compare_scalar(agreement["volume_limit_strict_less_than"], VOLUME_REL_LIMIT, 0.0,
                   f"REPORT_PATH_B_VOLUME_LIMIT:{component_id}")
    compare_scalar(agreement["com_limit_strict_less_than_m"], COM_LIMIT_M, 0.0,
                   f"REPORT_PATH_B_COM_LIMIT:{component_id}")
    require(agreement["pass"] is True, f"REPORT_PATH_B_AGREEMENT_PASS:{component_id}")
    return relative_volume, center_error_m


def compare_scalar(actual: Any, expected: float, tolerance: float, code: str) -> None:
    require(abs(float(actual) - float(expected)) <= tolerance, f"{code}:{actual}:{expected}")


def reported_pair_key(event: dict[str, Any]) -> tuple[str, str, str]:
    return str(event["component_id"]), str(event["left"]["atom_id"]), str(event["right"]["atom_id"])


def compare_vector(actual: Any, expected: Any, tolerance: float, code: str) -> None:
    difference = float(np.linalg.norm(np.asarray(actual, dtype=float) - np.asarray(expected, dtype=float)))
    require(difference <= tolerance, f"{code}:{difference}")


def compare_metric_record(actual: dict[str, Any], expected: dict[str, Any], code: str) -> None:
    compare_scalar(actual["volume_mm3"], expected["volume_mm3"], 1.0e-8, f"{code}:VOLUME")
    compare_vector(actual["centroid_world_mm"], expected["centroid_world_mm"], 1.0e-8, f"{code}:CENTROID")
    for key in ("min_mm", "max_mm", "size_mm"):
        compare_vector(actual["bbox_world_mm"][key], expected["bbox_world_mm"][key], 1.0e-8,
                       f"{code}:BBOX:{key}")
    compare_scalar(actual["surface_area_mm2"], expected["surface_area_mm2"], 1.0e-8, f"{code}:AREA")
    require(actual["topology_signature"] == expected["topology_signature"], f"{code}:TOPOLOGY")


def validate_thresholds(report: dict[str, Any]) -> None:
    thresholds = report["classification_thresholds"]
    compare_scalar(thresholds["original_positive_volume_strictly_greater_than_mm3"],
                   ORIGINAL_OVERLAP_TOL_MM3, 0.0, "CLASS_THRESHOLD_ORIGINAL")
    compare_scalar(thresholds["numerical_sliver_smaller_fraction_max_inclusive"],
                   SLIVER_FRACTION_MAX, 0.0, "CLASS_THRESHOLD_SLIVER")
    compare_scalar(thresholds["exact_volume_relative_tolerance"],
                   EXACT_VOLUME_REL_TOL, 0.0, "CLASS_THRESHOLD_EXACT_VOLUME")
    compare_scalar(thresholds["exact_bbox_max_coordinate_delta_mm"],
                   EXACT_BBOX_TOL_MM, 0.0, "CLASS_THRESHOLD_EXACT_BBOX")
    compare_scalar(thresholds["exact_centroid_distance_mm"],
                   EXACT_CENTROID_TOL_MM, 0.0, "CLASS_THRESHOLD_EXACT_CENTROID")
    compare_scalar(thresholds["exact_surface_area_relative_tolerance"],
                   EXACT_SURFACE_REL_TOL, 0.0, "CLASS_THRESHOLD_EXACT_SURFACE")
    require(thresholds["exact_topology_signature_must_equal"] is True,
            "EXACT_DUPLICATE_TOPOLOGY_GATE")
    require(thresholds["full_containment_requires_common_approximately_smaller_and_not_exact_duplicate"] is True,
            "CONTAINMENT_MISDEFINED")
    require(thresholds["zero_volume_contact_is_not_mass_duplication"] is True,
            "ZERO_VOLUME_CONTACT_MISDEFINED")


def validate_raw_overlap_report(report: dict[str, Any], raw_events: Sequence[dict[str, Any]]) -> None:
    overlap = report["overlap_audit"]
    require(overlap["original_atom_count"] == 266, "REPORT_ATOMS")
    require(overlap["internal_pair_count_checked"] == 3899, "REPORT_PAIRS")
    require(overlap["original_positive_overlap_pair_count"] == 91, "REPORT_OVERLAPS")
    require(overlap["boolean_failures"] == [], "REPORT_BOOLEAN_FAILURES")
    compare_scalar(overlap["original_positive_overlap_threshold_mm3_strictly_greater_than"],
                   ORIGINAL_OVERLAP_TOL_MM3, 0.0, "REPORT_ORIGINAL_THRESHOLD")
    require(overlap["zero_volume_contacts_in_original_positive_population"] == [],
            "REPORT_ZERO_CONTACT_IN_POSITIVE_POPULATION")
    counts = {key: int(overlap["classification_counts"].get(key, 0)) for key in EXPECTED_CLASS_COUNTS}
    require(counts == EXPECTED_CLASS_COUNTS, f"REPORT_CLASS_COUNTS:{counts}")
    require(all(int(value) == 0 for key, value in overlap["classification_counts"].items()
                if key not in EXPECTED_CLASS_COUNTS), "REPORT_UNEXPECTED_CLASS_COUNTS")

    reported_events = {reported_pair_key(event): event for event in overlap["events"]}
    require(len(reported_events) == 91, "REPORT_EVENT_COUNT_OR_DUPLICATES")
    independent_events = {
        (event["component_id"], event["left"]["atom_id"], event["right"]["atom_id"]): event
        for event in raw_events
    }
    require(set(reported_events) == set(independent_events), "REPORT_EVENT_KEYS")
    for key, independent in independent_events.items():
        reported = reported_events[key]
        require(int(reported["pair_order"]) == int(independent["pair_order"]), f"REPORT_PAIR_ORDER:{key}")
        require(reported["classification"] == independent["classification"], f"REPORT_CLASS:{key}")
        compare_scalar(reported["common_volume_mm3"], independent["common_volume"], 1.0e-8,
                       f"REPORT_COMMON:{key}")
        for side in ("left", "right"):
            expected_atom = independent[side]
            side_report = reported[side]
            require(side_report["atom_id"] == expected_atom["atom_id"], f"REPORT_ATOM_ID:{key}:{side}")
            require(side_report["member"] == expected_atom["member"], f"REPORT_MEMBER:{key}:{side}")
            require(side_report["source_object"] == expected_atom["source_object"],
                    f"REPORT_SOURCE:{key}:{side}")
            require(int(side_report["source_solid_index_one_based"]) == int(expected_atom["solid_index"]),
                    f"REPORT_SOLID_INDEX:{key}:{side}")
            compare_metric_record(side_report["metrics"], atom_metrics(expected_atom),
                                  f"REPORT_METRICS:{key}:{side}")
        for name, reported_name in (("left", "overlap_fraction_left"),
                                    ("right", "overlap_fraction_right"),
                                    ("smaller", "smaller_fraction")):
            compare_scalar(reported[reported_name], independent["fractions"][name], 1.0e-12,
                           f"REPORT_FRACTION:{key}:{name}")
        symmetric_difference = max(
            0.0, float(independent["left"]["volume"]) + float(independent["right"]["volume"])
            - 2.0 * float(independent["common_volume"]),
        )
        compare_scalar(reported["symmetric_difference_volume_mm3"], symmetric_difference, 1.0e-8,
                       f"REPORT_SYMMETRIC_DIFFERENCE:{key}")
        actual_evidence = reported["classification_evidence"]
        expected_evidence = independent["classification_evidence"]
        require(set(actual_evidence) == set(expected_evidence), f"REPORT_CLASSIFICATION_EVIDENCE_KEYS:{key}")
        for evidence_name, expected_value in expected_evidence.items():
            actual_value = actual_evidence[evidence_name]
            if isinstance(expected_value, bool):
                require(actual_value is expected_value,
                        f"REPORT_CLASSIFICATION_EVIDENCE_BOOL:{key}:{evidence_name}")
            else:
                tolerance = 1.0e-8 if evidence_name in {
                    "smaller_minus_common_mm3", "containment_tolerance_mm3",
                } else 1.0e-12
                compare_scalar(actual_value, expected_value, tolerance,
                               f"REPORT_CLASSIFICATION_EVIDENCE:{key}:{evidence_name}")
        require(reported["occupied_volume_action"] ==
                "REQUIRES_COMPONENT_LOCAL_SPATIAL_UNION_NOT_YET_ESTABLISHED",
                f"REPORT_OCCUPIED_ACTION:{key}")

    sliver_keys = {
        key for key, event in independent_events.items() if event["classification"] == "NUMERICAL_SLIVER"
    }
    require(sliver_keys == EXPECTED_UNRESOLVED_PAIRS, f"INDEPENDENT_SLIVER_KEYS:{sorted(sliver_keys)}")
    for key in EXPECTED_UNRESOLVED_PAIRS:
        independent = independent_events[key]
        require(int(independent["left"]["solid_index"]) == 22
                and int(independent["right"]["solid_index"]) == 24,
                f"INDEPENDENT_SLIVER_INDICES:{key}")
        require(0.01620 < float(independent["common_volume"]) < 0.01621,
                f"INDEPENDENT_SLIVER_COMMON:{key}")
        require(9.521e-7 < float(independent["fractions"]["smaller"]) < 9.522e-7,
                f"INDEPENDENT_SLIVER_FRACTION:{key}")

    prior_identity = overlap["prior_v2_identity_crosscheck"]
    require(prior_identity["prior_path"] == "V15_16_刚体惯量_FAIL审计_v2.json",
            "REPORT_PRIOR_IDENTITY_PATH")
    require(prior_identity["prior_sha256"].lower() ==
            PROTECTED_HASHES["V15_16_刚体惯量_FAIL审计_v2.json"], "REPORT_PRIOR_IDENTITY_HASH")
    require(prior_identity["ordered_event_identities_identical"] is True
            and prior_identity["pass"] is True, "REPORT_PRIOR_IDENTITY_STATUS")
    require("NOT_NUMERIC_UNION_AUTHORITY" in prior_identity["role"],
            "REPORT_PRIOR_USED_AS_NUMERIC_AUTHORITY")


def unresolved_pair_key(item: dict[str, Any]) -> tuple[str, str, str]:
    component_id = str(item["component_id"])
    if isinstance(item.get("left"), dict):
        left_id = str(item["left"]["atom_id"])
        right_id = str(item["right"]["atom_id"])
    else:
        left_id = str(item["left_atom_id"])
        right_id = str(item["right_atom_id"])
    return component_id, left_id, right_id


def serialize_bounded_boolean(shape: Any, expected_volume: float) -> dict[str, Any]:
    if shape is None or shape.isNull():
        return {
            "is_null": True, "is_valid": False, "is_closed": False,
            "volume_mm3": None, "volume_drift_from_expected_union_mm3": None,
            "deep_bop_check_errors": ["NULL_RESULT"], "accepted": False,
        }
    volume = abs(float(shape.Volume))
    try:
        deep = shape.check(True)
        errors = [] if deep is None else sorted(str(item) for item in deep)
    except Exception as exc:
        errors = ["CHECK_EXCEPTION:" + str(exc)]
    valid = bool(shape.isValid())
    closed = bool(shape.isClosed())
    drift = volume - expected_volume
    tolerance = max(CANONICAL_ABSOLUTE_TOL_MM3, CANONICAL_RELATIVE_TOL * expected_volume)
    return {
        "is_null": False, "is_valid": valid,
        "is_closed": closed, "solid_count": len(shape.Solids),
        "volume_mm3": volume,
        "volume_drift_from_expected_union_mm3": drift,
        "acceptance_tolerance_mm3": tolerance,
        "deep_bop_check_errors": errors,
        "accepted": valid and closed and not errors and abs(drift) <= tolerance,
    }


def independently_reproduce_blocker_gates(left: Any, right: Any) -> dict[str, Any]:
    require(left.isValid() and left.isClosed() and right.isValid() and right.isClosed(),
            "BLOCKER_SOURCE_NOT_SHALLOW_VALID_CLOSED")
    left_volume, right_volume = float(left.Volume), float(right.Volume)
    common = left.common(right)
    common_volume = 0.0 if common.isNull() else abs(float(common.Volume))
    expected_union = left_volume + right_volume - common_volume
    results: dict[str, Any] = {
        "left_volume_mm3": left_volume, "right_volume_mm3": right_volume,
        "common_volume_mm3": common_volume,
        "set_theoretic_union_volume_mm3": expected_union,
    }
    for label, operation in (
        ("fuse_left_right", lambda: left.copy().fuse(right.copy())),
        ("fuse_right_left", lambda: right.copy().fuse(left.copy())),
        ("general_fuse", lambda: left.copy().generalFuse([right.copy()], 0.0)[0]),
    ):
        try:
            results[label] = serialize_bounded_boolean(operation(), expected_union)
        except Exception as exc:
            results[label] = {"exception": str(exc), "accepted": False}
    for label, minuend, subtrahend, expected_difference in (
        ("cut_left_minus_right", left, right, left_volume - common_volume),
        ("cut_right_minus_left", right, left, right_volume - common_volume),
    ):
        try:
            result = minuend.copy().cut(subtrahend.copy())
            actual = 0.0 if result.isNull() else abs(float(result.Volume))
            results[label] = {
                **serialize_bounded_boolean(result, expected_difference),
                "expected_difference_volume_mm3": expected_difference,
                "volume_drift_from_expected_difference_mm3": actual - expected_difference,
                "volume_monotonic": actual <= float(minuend.Volume) + 1.0e-7,
            }
            results[label]["accepted"] = bool(
                results[label]["accepted"] and results[label]["volume_monotonic"])
        except Exception as exc:
            results[label] = {"exception": str(exc), "accepted": False}
    route_labels = (
        "fuse_left_right", "fuse_right_left", "general_fuse",
        "cut_left_minus_right", "cut_right_minus_left",
    )
    results["all_candidate_routes_rejected"] = all(
        not bool(results[label].get("accepted", False)) for label in route_labels)
    require(results["all_candidate_routes_rejected"], "BLOCKER_ROUTE_UNEXPECTEDLY_ACCEPTED")
    return results


def compare_optional_number(actual: Any, expected: Any, tolerance: float, code: str) -> None:
    if expected is None:
        require(actual is None, code)
    else:
        compare_scalar(actual, expected, tolerance, code)


def compare_blocker_gate_record(actual: dict[str, Any], expected: dict[str, Any], code: str) -> None:
    require(set(actual) == set(expected), f"{code}:KEYS")
    for key, expected_value in expected.items():
        actual_value = actual[key]
        if key == "exception":
            require(isinstance(actual_value, str) and actual_value,
                    f"{code}:EXCEPTION")
        elif key == "deep_bop_check_errors":
            require(list(actual_value) == list(expected_value), f"{code}:DEEP_ERRORS")
        elif isinstance(expected_value, bool):
            require(actual_value is expected_value, f"{code}:{key}")
        elif expected_value is None or isinstance(expected_value, (int, float)):
            compare_optional_number(actual_value, expected_value, 1.0e-8, f"{code}:{key}")
        else:
            require(actual_value == expected_value, f"{code}:{key}")


def validate_failure_attempts(
    item: dict[str, Any], independent: dict[str, Any], independently_reproduced: dict[str, Any],
) -> None:
    require(item["code"] == "OCCT_SOURCE_BREP_BOOLEAN_PATHOLOGY", "UNRESOLVED_CODE")
    require(int(item["left_solid_index_one_based"]) == 22
            and int(item["right_solid_index_one_based"]) == 24,
            "UNRESOLVED_SOLID_INDICES")
    compare_scalar(item["left_volume_mm3"], independent["left"]["volume"], 1.0e-8,
                   "UNRESOLVED_LEFT_VOLUME")
    compare_scalar(item["right_volume_mm3"], independent["right"]["volume"], 1.0e-8,
                   "UNRESOLVED_RIGHT_VOLUME")
    compare_scalar(item["common_volume_mm3"], independent["common_volume"], 1.0e-8,
                   "UNRESOLVED_COMMON")
    compare_scalar(item["smaller_fraction"], independent["fractions"]["smaller"], 1.0e-12,
                   "UNRESOLVED_SMALLER_FRACTION")
    require(item["classification"] == "NUMERICAL_SLIVER", "UNRESOLVED_CLASSIFICATION")
    require(item.get("resolved") is False, "UNRESOLVED_RESOLVED_FLAG")
    require(item["shallow_source_is_valid_and_closed"] is True,
            "UNRESOLVED_SHALLOW_SOURCE_STATUS")
    require(str(item["deep_bop_source_check"]).startswith("FAIL_"),
            "UNRESOLVED_DEEP_SOURCE_STATUS")
    tolerances = item["source_max_edge_vertex_tolerances_mm"]
    require(0.0 < float(tolerances["solid_22"]) < 0.01
            and 0.0 < float(tolerances["solid_24"]) < 0.01,
            "UNRESOLVED_SOURCE_TOLERANCES")
    compare_scalar(item["set_theoretic_pair_union_volume_mm3"],
                   float(independent["left"]["volume"]) + float(independent["right"]["volume"])
                   - float(independent["common_volume"]), 1.0e-8,
                   "UNRESOLVED_SET_UNION_VOLUME")
    require(item["accepted_candidate"] is None, "UNRESOLVED_ACCEPTED_CANDIDATE")
    require(item["required_resolution"] ==
            "SUPPLY_REPAIRED_WATERTIGHT_STATOR_MASS_GEOMETRY_AUTHORITY_OR_DIRECT_OCCUPIED_VOLUME_AUTHORITY",
            "UNRESOLVED_REQUIRED_RESOLUTION")

    reported_gates = item["reproduced_blocker_gates"]
    for numeric_key in (
        "left_volume_mm3", "right_volume_mm3", "common_volume_mm3",
        "set_theoretic_union_volume_mm3",
    ):
        compare_scalar(reported_gates[numeric_key], independently_reproduced[numeric_key], 1.0e-8,
                       f"UNRESOLVED_REPRODUCED:{numeric_key}")
    for route in (
        "fuse_left_right", "fuse_right_left", "general_fuse",
        "cut_left_minus_right", "cut_right_minus_left",
    ):
        compare_blocker_gate_record(reported_gates[route], independently_reproduced[route],
                                    f"UNRESOLVED_REPRODUCED:{route}")
        require(reported_gates[route]["accepted"] is False,
                f"UNRESOLVED_ROUTE_FALSE_ACCEPT:{route}")
        if "exception" not in reported_gates[route]:
            deep_errors = reported_gates[route].get("deep_bop_check_errors", [])
            drift_key = ("volume_drift_from_expected_difference_mm3"
                         if route.startswith("cut_") else "volume_drift_from_expected_union_mm3")
            drift = reported_gates[route].get(drift_key)
            require(
                not reported_gates[route].get("is_valid", False)
                or not reported_gates[route].get("is_closed", False)
                or bool(deep_errors)
                or drift is None or abs(float(drift)) > CANONICAL_ABSOLUTE_TOL_MM3
                or (route.startswith("cut_") and reported_gates[route].get("volume_monotonic") is False),
                f"UNRESOLVED_ROUTE_UNEXPLAINED_REJECTION:{route}",
            )
    require(reported_gates["all_candidate_routes_rejected"] is True,
            "UNRESOLVED_ALL_ROUTES_REJECTED")

    supplemental = item["supplemental_read_only_probe_evidence"]
    require(supplemental["role"] == "BOUNDED_ADVERSARIAL_PROBE_EVIDENCE_NOT_REEXECUTED_BY_BUILDER",
            "UNRESOLVED_SUPPLEMENTAL_ROLE")
    methods = supplemental["method_attempts"]
    require(isinstance(methods, list) and len(methods) >= 8,
            "UNRESOLVED_SUPPLEMENTAL_METHOD_COUNT")
    require(len({str(method["method"]) for method in methods}) == len(methods),
            "UNRESOLVED_SUPPLEMENTAL_METHOD_DUPLICATES")
    require(any("FUZZY" in str(method["method"]).upper() for method in methods),
            "UNRESOLVED_SUPPLEMENTAL_NO_FUZZY")
    require(any("MESH" in str(method["method"]).upper()
                or "TESSELL" in str(method["method"]).upper() for method in methods),
            "UNRESOLVED_SUPPLEMENTAL_NO_FACETED")
    require(all(str(method.get("result", "")).startswith("REJECTED") for method in methods),
            "UNRESOLVED_SUPPLEMENTAL_NONREJECTED_METHOD")


def validate_frozen_masses(
    report: dict[str, Any], components: Sequence[dict[str, Any]], mass_map: dict[str, Any],
    atoms_by_component: dict[str, list[dict[str, Any]]], raw_events: Sequence[dict[str, Any]],
) -> None:
    records = report.get("components", [])
    by_id = {str(record["component_id"]): record for record in records}
    require(len(records) == 17 and set(by_id) == set(COMPONENT_ORDER), "REPORT_COMPONENT_SET")
    total = 0.0
    for component in components:
        component_id = component["component_id"]
        expected_mass = float(mass_map[component_id]["nominal_mass_kg"])
        total += expected_mass
        record = by_id[component_id]
        require(record["owner_link"] == component["owner_link"], f"REPORT_OWNER_LINK:{component_id}")
        compare_scalar(record["frozen_mass_kg"], expected_mass, 1.0e-15,
                       f"REPORT_MASS:{component_id}")
        require(record["mass_unchanged"] is True, f"REPORT_MASS_CHANGED:{component_id}")
        atoms = atoms_by_component[component_id]
        events = [event for event in raw_events if event["component_id"] == component_id]
        require(int(record["atom_count"]) == len(atoms), f"REPORT_COMPONENT_ATOMS:{component_id}")
        compare_scalar(record["raw_abs_volume_mm3"], math.fsum(float(atom["volume"]) for atom in atoms),
                       1.0e-8, f"REPORT_COMPONENT_RAW_VOLUME:{component_id}")
        require(int(record["original_positive_overlap_pair_count"]) == len(events),
                f"REPORT_COMPONENT_OVERLAPS:{component_id}")
        expected_counts = dict(sorted(Counter(event["classification"] for event in events).items()))
        require(record["classification_counts"] == expected_counts,
                f"REPORT_COMPONENT_CLASS_COUNTS:{component_id}")
        expected_status = (
            "BLOCKED_BY_SOURCE_BREP_BOOLEAN_PATHOLOGY"
            if component_id in {key[0] for key in EXPECTED_UNRESOLVED_PAIRS}
            else "NOT_ESTABLISHED_AFTER_GLOBAL_FAIL_GATE_CLOSED"
        )
        require(record["occupied_volume_policy_status"] == expected_status,
                f"REPORT_COMPONENT_POLICY_STATUS:{component_id}")
        for field in (
            "canonical_union_volume_mm3", "canonical_union_com_world_mm",
            "removed_overlap_effective_volume_mm3", "duplicate_fraction",
            "rho_union_kg_per_mm3", "canonical_positive_overlap_pair_count",
            "path_a_b_crosscheck", "union_minus_com_v2_delta",
        ):
            require(record[field] is None, f"REPORT_COMPONENT_FALSE_AUTHORITY_VALUE:{component_id}:{field}")
        require(record["authority_eligible"] is False,
                f"REPORT_COMPONENT_FALSE_AUTHORITY_ELIGIBLE:{component_id}")
        require(record.get("collision_proxy_used") is False,
                f"REPORT_COMPONENT_COLLISION_PROXY:{component_id}")
        if "raw_com_v2_world_mm" in record:
            compare_vector(record["raw_com_v2_world_mm"], component["com_world_mm"], 1.0e-12,
                           f"REPORT_COMPONENT_COM_V2:{component_id}")
    require(abs(total - EXPECTED_TOTAL_MASS) < 1.0e-12, "TOTAL_MASS")

    validation = report["validation"]
    require(validation["all_component_masses_unchanged"] is True, "REPORT_ALL_MASSES")
    compare_scalar(validation["total_mass_kg"], EXPECTED_TOTAL_MASS, 1.0e-12,
                   "REPORT_TOTAL_MASS")
    if "expected_total_mass_kg" in validation:
        compare_scalar(validation["expected_total_mass_kg"], EXPECTED_TOTAL_MASS, 1.0e-12,
                       "REPORT_EXPECTED_TOTAL_MASS")


def validate_fail_report(
    report: dict[str, Any], components: Sequence[dict[str, Any]], mass_map: dict[str, Any],
    raw_events: Sequence[dict[str, Any]], atoms_by_component: dict[str, list[dict[str, Any]]],
) -> None:
    require(report["schema"] == FAIL_SCHEMA, "REPORT_SCHEMA")
    require(report["final_status"] == FAIL_STATUS, "FINAL_STATUS_NOT_EXACT_FAIL")
    require(report["authoritative"] is False, "REPORT_FALSE_AUTHORITY_MISSING")
    require(report["occupied_volume_authority_established"] is False,
            "REPORT_OCCUPIED_AUTHORITY_ESTABLISHED")
    require(report["policy_not_established"] is True, "REPORT_POLICY_FALSELY_ESTABLISHED")
    require(report["freeze_permitted"] is False, "REPORT_FALSE_FREEZE_PERMISSION")
    if "authoritative_ledger_written" in report:
        require(report["authoritative_ledger_written"] is False,
                "REPORT_AUTHORITATIVE_LEDGER_WRITTEN")
    require(tuple(report["component_order"]) == COMPONENT_ORDER, "REPORT_COMPONENT_ORDER")
    validate_report_provenance(report)
    validate_thresholds(report)
    validate_raw_overlap_report(report, raw_events)
    validate_frozen_masses(report, components, mass_map, atoms_by_component, raw_events)

    unresolved = report["unresolved_overlap_items"]
    require(isinstance(unresolved, list) and len(unresolved) == 3,
            "UNRESOLVED_ITEM_COUNT")
    unresolved_by_key = {unresolved_pair_key(item): item for item in unresolved}
    require(len(unresolved_by_key) == 3 and set(unresolved_by_key) == EXPECTED_UNRESOLVED_PAIRS,
            f"UNRESOLVED_PAIR_SET:{sorted(unresolved_by_key)}")
    independent_events = {
        (event["component_id"], event["left"]["atom_id"], event["right"]["atom_id"]): event
        for event in raw_events
    }
    for key, item in unresolved_by_key.items():
        _, left_id, right_id = key
        atom_by_id = {atom["atom_id"]: atom for atom in atoms_by_component[key[0]]}
        independently_reproduced = independently_reproduce_blocker_gates(
            atom_by_id[left_id]["shape"], atom_by_id[right_id]["shape"],
        )
        validate_failure_attempts(item, independent_events[key], independently_reproduced)
    duplicate_unresolved = report["unresolved_items"]
    require(isinstance(duplicate_unresolved, list)
            and {unresolved_pair_key(item) for item in duplicate_unresolved} == EXPECTED_UNRESOLVED_PAIRS,
            "UNRESOLVED_ITEMS_SET")

    links = report["links"]
    require(isinstance(links, list) and len(links) == 6, "REPORT_LINK_COUNT")
    links_by_name = {str(link["link"]): link for link in links}
    require(set(links_by_name) == set(LINK_ORDER), "REPORT_LINK_SET")
    for link_name, link in links_by_name.items():
        require(link["union_candidate_com_world_mm"] is None
                and link["union_candidate_minus_com_v2_delta_norm_mm"] is None,
                f"REPORT_LINK_FALSE_CANDIDATE:{link_name}")
        require(link["status"] == "N/A_OCCUPIED_VOLUME_AUTHORITY_NOT_ESTABLISHED",
                f"REPORT_LINK_STATUS:{link_name}")

    prohibited = report["prohibited_actions"]
    for key in (
        "collision_proxy_used", "modified_com_v2", "modified_original_cad",
        "modified_v15_15_authorities", "final_inertia_tensor_computed", "final_inertia_frozen",
        "residual_sensitivity_recalculated", "deployment_files_modified",
    ):
        require(prohibited.get(key) is False, f"PROHIBITED_ACTION:{key}")

    validation = report["validation"]
    require(validation["fail_audit_evidence_complete"] is True,
            "REPORT_FAIL_EVIDENCE_INCOMPLETE")
    require(validation["all_91_original_overlaps_classified"] is True,
            "REPORT_91_RECOMPUTE_FLAG")
    require(validation["unresolved_overlap_item_count"] == 3, "REPORT_UNRESOLVED_COUNT")
    require(validation["all_17_component_policies_established"] is False,
            "REPORT_FALSE_ALL17_CLAIM")
    require(validation["canonical_positive_overlap_pair_count"] is None,
            "REPORT_FALSE_CANONICAL_COUNT")
    require(validation["canonical_positive_overlap_zero"] is False,
            "REPORT_FALSE_CANONICAL_ZERO")
    require(validation["all_path_a_b_crosschecks_pass"] is False,
            "REPORT_FALSE_PATH_AB_PASS")
    require(validation["max_path_a_b_volume_relative_error"] is None
            and validation["max_path_a_b_com_error_m"] is None,
            "REPORT_FALSE_PATH_AB_MAX")
    require(validation["authority_pass"] is False, "REPORT_FALSE_AUTHORITY_PASS")
    require(validation["authority_established"] is False, "REPORT_FALSE_AUTHORITY_ESTABLISHED")
    require(validation["pass"] is False, "REPORT_FALSE_VALIDATION_PASS")

    serialized = json.dumps(report, ensure_ascii=False, sort_keys=True)
    require("V15.16 MASS_GEOMETRY_AUTHORITY_V1 = PASS" not in serialized,
            "REPORT_CONTAINS_PASS_AUTHORITY_CLAIM")
    require("inertia_tensor_kg_m2" not in serialized, "FINAL_INERTIA_PRESENT")


def validate_pass_report(
    report: dict[str, Any], com: dict[str, Any], components: Sequence[dict[str, Any]],
    mass_map: dict[str, Any], atoms_by_component: dict[str, list[dict[str, Any]]],
    raw_events: Sequence[dict[str, Any]], canonical_by_component: dict[str, list[Any]],
) -> None:
    require(report["schema"] == "go-m8010-arm-v15.16-mass-geometry-authority-v1/1.0", "REPORT_SCHEMA")
    require(report["final_status"] == "V15.16 MASS_GEOMETRY_AUTHORITY_V1 = PASS", "FINAL_STATUS")
    require(tuple(report["component_order"]) == COMPONENT_ORDER, "REPORT_COMPONENT_ORDER")
    require(report["unresolved_overlap_items"] == [] and report["unresolved_items"] == [], "UNRESOLVED")
    validate_report_provenance(report)
    serialized = json.dumps(report, ensure_ascii=False).lower()
    require("collision_proxy_used\": false" in serialized or report["prohibited_actions"]["collision_proxy_used"] is False,
            "COLLISION_PROXY_FLAG")
    for key in ("modified_com_v2", "modified_original_cad", "final_inertia_frozen", "residual_sensitivity_recalculated"):
        require(report["prohibited_actions"].get(key) is False, f"PROHIBITED_ACTION:{key}")
    require("inertia_tensor_kg_m2" not in serialized, "FINAL_INERTIA_PRESENT")

    thresholds = report["classification_thresholds"]
    compare_scalar(thresholds["original_positive_volume_strictly_greater_than_mm3"],
                   ORIGINAL_OVERLAP_TOL_MM3, 0.0, "CLASS_THRESHOLD_ORIGINAL")
    compare_scalar(thresholds["numerical_sliver_smaller_fraction_max_inclusive"],
                   SLIVER_FRACTION_MAX, 0.0, "CLASS_THRESHOLD_SLIVER")
    compare_scalar(thresholds["exact_volume_relative_tolerance"],
                   EXACT_VOLUME_REL_TOL, 0.0, "CLASS_THRESHOLD_EXACT_VOLUME")
    compare_scalar(thresholds["exact_common_and_symmetric_difference_absolute_floor_mm3"],
                   CONTAINMENT_ABSOLUTE_TOL_MM3, 0.0, "CLASS_THRESHOLD_EXACT_ABSOLUTE")
    compare_scalar(thresholds["exact_common_and_symmetric_difference_relative_tolerance"],
                   CONTAINMENT_RELATIVE_TOL, 0.0, "CLASS_THRESHOLD_EXACT_RELATIVE")
    compare_scalar(thresholds["exact_bbox_max_coordinate_delta_mm"],
                   EXACT_BBOX_TOL_MM, 0.0, "CLASS_THRESHOLD_EXACT_BBOX")
    compare_scalar(thresholds["exact_centroid_distance_mm"],
                   EXACT_CENTROID_TOL_MM, 0.0, "CLASS_THRESHOLD_EXACT_CENTROID")
    compare_scalar(thresholds["exact_surface_area_relative_tolerance"],
                   EXACT_SURFACE_REL_TOL, 0.0, "CLASS_THRESHOLD_EXACT_SURFACE")
    require(thresholds["canonical_pair_tolerance"] ==
            "max(1e-9 mm^3,1e-12*component_union_volume)", "CLASS_THRESHOLD_CANONICAL")
    require(thresholds["exact_topology_signature_must_equal"] is True,
            "EXACT_DUPLICATE_TOPOLOGY_GATE")
    require(thresholds["full_containment_requires_common_approximately_smaller_and_not_exact_duplicate"] is True,
            "CONTAINMENT_MISDEFINED")
    require(thresholds["zero_volume_contact_is_not_mass_duplication"] is True,
            "ZERO_VOLUME_CONTACT_MISDEFINED")

    overlap = report["overlap_audit"]
    require(overlap["original_atom_count"] == 266, "REPORT_ATOMS")
    require(overlap["internal_pair_count_checked"] == 3899, "REPORT_PAIRS")
    require(overlap["original_positive_overlap_pair_count"] == 91, "REPORT_OVERLAPS")
    require(overlap["boolean_failures"] == [], "REPORT_BOOLEAN_FAILURES")
    compare_scalar(overlap["original_positive_overlap_threshold_mm3_strictly_greater_than"],
                   ORIGINAL_OVERLAP_TOL_MM3, 0.0, "REPORT_ORIGINAL_THRESHOLD")
    require(overlap["zero_volume_contacts_in_original_positive_population"] == [],
            "REPORT_ZERO_CONTACT_IN_POSITIVE_POPULATION")
    counts = {key: int(overlap["classification_counts"].get(key, 0)) for key in EXPECTED_CLASS_COUNTS}
    require(counts == EXPECTED_CLASS_COUNTS, f"REPORT_CLASS_COUNTS:{counts}")
    reported_events = {reported_pair_key(event): event for event in overlap["events"]}
    require(len(reported_events) == 91, "REPORT_EVENT_COUNT_OR_DUPLICATES")
    independent_events = {
        (event["component_id"], event["left"]["atom_id"], event["right"]["atom_id"]): event
        for event in raw_events
    }
    require(set(reported_events) == set(independent_events), "REPORT_EVENT_KEYS")
    for key, independent in independent_events.items():
        reported = reported_events[key]
        require(int(reported["pair_order"]) == int(independent["pair_order"]), f"REPORT_PAIR_ORDER:{key}")
        require(reported["classification"] == independent["classification"], f"REPORT_CLASS:{key}")
        compare_scalar(reported["common_volume_mm3"], independent["common_volume"], 1.0e-8, f"REPORT_COMMON:{key}")
        for side in ("left", "right"):
            expected_atom = independent[side]
            side_report = reported[side]
            require(side_report["atom_id"] == expected_atom["atom_id"], f"REPORT_ATOM_ID:{key}:{side}")
            require(side_report["member"] == expected_atom["member"], f"REPORT_MEMBER:{key}:{side}")
            require(side_report["source_object"] == expected_atom["source_object"], f"REPORT_SOURCE:{key}:{side}")
            require(int(side_report["source_solid_index_one_based"]) == int(expected_atom["solid_index"]),
                    f"REPORT_SOLID_INDEX:{key}:{side}")
            compare_metric_record(side_report["metrics"], atom_metrics(expected_atom), f"REPORT_METRICS:{key}:{side}")
        for name, reported_name in (("left", "overlap_fraction_left"), ("right", "overlap_fraction_right"),
                                    ("smaller", "smaller_fraction")):
            compare_scalar(reported[reported_name], independent["fractions"][name], 1.0e-12,
                           f"REPORT_FRACTION:{key}:{name}")
        symmetric_difference = max(
            0.0, float(independent["left"]["volume"]) + float(independent["right"]["volume"])
            - 2.0 * float(independent["common_volume"])
        )
        compare_scalar(reported["symmetric_difference_volume_mm3"], symmetric_difference, 1.0e-8,
                       f"REPORT_SYMMETRIC_DIFFERENCE:{key}")
        actual_evidence = reported["classification_evidence"]
        expected_evidence = independent["classification_evidence"]
        require(set(actual_evidence) == set(expected_evidence), f"REPORT_CLASSIFICATION_EVIDENCE_KEYS:{key}")
        for evidence_name, expected_value in expected_evidence.items():
            actual_value = actual_evidence[evidence_name]
            if isinstance(expected_value, bool):
                require(actual_value is expected_value,
                        f"REPORT_CLASSIFICATION_EVIDENCE_BOOL:{key}:{evidence_name}")
            else:
                tolerance = 1.0e-8 if evidence_name in {
                    "smaller_minus_common_mm3", "containment_tolerance_mm3"
                } else 1.0e-12
                compare_scalar(actual_value, expected_value, tolerance,
                               f"REPORT_CLASSIFICATION_EVIDENCE:{key}:{evidence_name}")
        require(reported["occupied_volume_action"] ==
                "INCLUDED_ONCE_BY_COMPONENT_LOCAL_SPATIAL_UNION", f"REPORT_OCCUPIED_ACTION:{key}")

    prior_identity = overlap["prior_v2_identity_crosscheck"]
    require(prior_identity["prior_path"] == "V15_16_刚体惯量_FAIL审计_v2.json",
            "REPORT_PRIOR_IDENTITY_PATH")
    require(prior_identity["prior_sha256"].lower() ==
            PROTECTED_HASHES["V15_16_刚体惯量_FAIL审计_v2.json"], "REPORT_PRIOR_IDENTITY_HASH")
    require(prior_identity["ordered_event_identities_identical"] is True
            and prior_identity["pass"] is True, "REPORT_PRIOR_IDENTITY_STATUS")
    require("NOT_NUMERIC_UNION_AUTHORITY" in prior_identity["role"],
            "REPORT_PRIOR_USED_AS_NUMERIC_AUTHORITY")

    report_components = {entry["component_id"]: entry for entry in report["components"]}
    require(set(report_components) == set(COMPONENT_ORDER) and len(report["components"]) == 17,
            "REPORT_COMPONENT_SET")
    masses = []
    union_centers: dict[str, np.ndarray] = {}
    for component in components:
        component_id = component["component_id"]
        entry = report_components[component_id]
        mass = float(mass_map[component_id]["nominal_mass_kg"])
        masses.append(mass)
        shapes = canonical_by_component[component_id]
        volume, center = aggregate_volume_com(shapes)
        union_centers[component_id] = center
        raw_volume = math.fsum(float(atom["volume"]) for atom in atoms_by_component[component_id])
        require(entry["owner_link"] == component["owner_link"], f"REPORT_OWNER_LINK:{component_id}")
        require(int(entry["atom_count"]) == len(atoms_by_component[component_id]),
                f"REPORT_ATOM_COUNT:{component_id}")
        require(int(entry["canonical_solid_count"]) == len(shapes),
                f"REPORT_CANONICAL_SOLID_COUNT:{component_id}")
        compare_scalar(entry["frozen_mass_kg"], mass, 1.0e-15, f"REPORT_MASS:{component_id}")
        require(entry["mass_unchanged"] is True, f"MASS_CHANGED:{component_id}")
        require(entry["collision_proxy_used"] is False, f"REPORT_COMPONENT_COLLISION_PROXY:{component_id}")
        compare_scalar(entry["raw_abs_volume_mm3"], raw_volume, 1.0e-8, f"REPORT_RAW_VOLUME:{component_id}")
        compare_scalar(entry["union_volume_mm3"], volume, 1.0e-8, f"REPORT_UNION_VOLUME:{component_id}")
        compare_scalar(entry["removed_overlap_effective_volume_mm3"], raw_volume - volume, 1.0e-8,
                       f"REPORT_REMOVED_VOLUME:{component_id}")
        compare_scalar(entry["duplicate_fraction"], (raw_volume - volume) / raw_volume, 1.0e-12,
                       f"REPORT_DUPLICATE_FRACTION:{component_id}")
        compare_scalar(entry["rho_raw_kg_per_mm3"], mass / raw_volume, 1.0e-15, f"REPORT_RHO_RAW:{component_id}")
        compare_scalar(entry["rho_union_kg_per_mm3"], mass / volume, 1.0e-15, f"REPORT_RHO_UNION:{component_id}")
        pair_audit = entry["canonical_internal_pair_audit"]
        require(pair_audit["positive_overlap_pair_count"] == 0, f"REPORT_CANONICAL_OVERLAP:{component_id}")
        require(pair_audit["boolean_failures"] == [], f"REPORT_CANONICAL_BOOLEAN:{component_id}")
        require(pair_audit["all_solids_valid_closed_deep_bop_clean"] is True,
                f"REPORT_CANONICAL_DEEP_VALIDITY:{component_id}")
        expected_solid_summaries = [
            canonical_solid_metric_record(shape, index)
            for index, shape in enumerate(shapes, 1)
        ]
        require(len(entry["canonical_solid_summaries"]) == len(expected_solid_summaries),
                f"REPORT_CANONICAL_SOLID_SUMMARY_COUNT:{component_id}")
        for actual_summary, expected_summary in zip(entry["canonical_solid_summaries"], expected_solid_summaries):
            require(actual_summary["canonical_solid_index_one_based"] == expected_summary["canonical_solid_index_one_based"],
                    f"REPORT_CANONICAL_SOLID_INDEX:{component_id}")
            compare_metric_record(actual_summary, expected_summary, f"REPORT_CANONICAL_SOLID:{component_id}")
            require(actual_summary["is_valid"] is True and actual_summary["is_closed"] is True
                    and actual_summary["deep_valid"] is True and actual_summary["deep_bop_check_errors"] == [],
                    f"REPORT_CANONICAL_SOLID_VALIDITY:{component_id}")
        coverage_records = occupied_set_coverage_gate(component_id, atoms_by_component[component_id], shapes, volume)
        coverage = entry["raw_atom_occupied_set_coverage_audit"]
        require(coverage["pass"] is True and coverage["boolean_failures"] == [],
                f"REPORT_COVERAGE_STATUS:{component_id}")
        actual_coverage = {record["atom_id"]: record for record in coverage["records"]}
        expected_coverage = {record["atom_id"]: record for record in coverage_records}
        require(set(actual_coverage) == set(expected_coverage), f"REPORT_COVERAGE_ATOM_SET:{component_id}")
        for atom_id, expected_record in expected_coverage.items():
            actual_record = actual_coverage[atom_id]
            for field in ("raw_atom_volume_mm3", "covered_by_canonical_volume_mm3", "uncovered_volume_mm3",
                          "coverage_overshoot_mm3", "tolerance_mm3"):
                compare_scalar(actual_record[field], expected_record[field], 1.0e-8,
                               f"REPORT_COVERAGE:{component_id}:{atom_id}:{field}")
            require(actual_record["pass"] is True, f"REPORT_COVERAGE_PASS:{component_id}:{atom_id}")
        recipe = entry["canonical_recipe"]
        require(recipe["derived_copies_only"] is True and recipe["component_local_only"] is True,
                f"REPORT_RECIPE_SCOPE:{component_id}")
        require(recipe["cross_component_union"] is False, f"REPORT_CROSS_COMPONENT_UNION:{component_id}")
        decisions = recipe["containment_and_exact_duplicate_collapse"]
        require(isinstance(decisions, list), f"REPORT_CONTAINMENT_COLLAPSE_TYPE:{component_id}")
        require(all(decision.get("classification") in {"EXACT_DUPLICATE", "FULL_CONTAINMENT"}
                    and decision.get("collapsed_atom_physical_component_membership_retained") is True
                    for decision in decisions), f"REPORT_CONTAINMENT_COLLAPSE_RECORD:{component_id}")
        require(recipe["collapse_preserves_physical_component_membership"] is True,
                f"REPORT_MEMBERSHIP_LOST:{component_id}")
        compare_scalar(entry["path_a_occt"]["volume_mm3"], volume, 1.0e-8, f"REPORT_PATH_A_VOLUME:{component_id}")
        require(float(np.linalg.norm(v3(entry["path_a_occt"]["com_world_mm"]) - center)) <= 1.0e-8,
                f"REPORT_PATH_A_COM:{component_id}")
        frozen = v3(component["com_world_mm"])
        compare_vector(entry["raw_com_v2_world_mm"], frozen, 1.0e-12,
                       f"REPORT_COMPONENT_RAW_COM_V2:{component_id}")
        compare_vector(entry["union_com_world_mm"], center, 1.0e-8,
                       f"REPORT_COMPONENT_UNION_COM:{component_id}")
        delta = center - frozen
        require(float(np.linalg.norm(v3(entry["union_minus_com_v2_delta_xyz_mm"]) - delta)) <= 1.0e-8,
                f"REPORT_COMPONENT_DELTA_XYZ:{component_id}")
        compare_scalar(entry["union_minus_com_v2_delta_norm_mm"], float(np.linalg.norm(delta)), 1.0e-8,
                       f"REPORT_COMPONENT_DELTA_NORM:{component_id}")

    require(abs(math.fsum(masses) - EXPECTED_TOTAL_MASS) < 1.0e-12, "TOTAL_MASS")
    validation = report["validation"]
    require(validation["all_component_masses_unchanged"] is True, "REPORT_ALL_MASSES")
    compare_scalar(validation["total_mass_kg"], EXPECTED_TOTAL_MASS, 1.0e-12, "REPORT_TOTAL_MASS")
    require(validation["canonical_positive_overlap_pair_count"] == 0, "REPORT_TOTAL_CANONICAL_OVERLAP")
    require(validation["component_count"] == 17, "REPORT_COMPONENT_COUNT")

    report_links = {entry["link"]: entry for entry in report["links"]}
    require(set(report_links) == set(LINK_ORDER) and len(report["links"]) == 6, "REPORT_LINK_SET")
    for link_name in LINK_ORDER:
        authority = com["links"][link_name]
        ids = list(authority["component_ids"])
        link_mass = math.fsum(float(mass_map[component_id]["nominal_mass_kg"]) for component_id in ids)
        compare_scalar(link_mass, EXPECTED_LINK_MASSES[link_name], 1.0e-12, f"LINK_MASS:{link_name}")
        candidate = np.array([
            math.fsum(float(mass_map[component_id]["nominal_mass_kg"]) * float(union_centers[component_id][axis])
                      for component_id in ids) / link_mass
            for axis in range(3)
        ])
        frozen = v3(authority["com_world_mm"])
        delta = candidate - frozen
        entry = report_links[link_name]
        compare_scalar(entry["frozen_mass_kg"], link_mass, 1.0e-12,
                       f"REPORT_LINK_FROZEN_MASS:{link_name}")
        require(list(entry["component_ids"]) == ids, f"REPORT_LINK_COMPONENT_IDS:{link_name}")
        compare_vector(entry["com_v2_world_mm"], frozen, 1.0e-12,
                       f"REPORT_LINK_COM_V2_WORLD:{link_name}")
        require(float(np.linalg.norm(v3(entry["union_candidate_com_world_mm"]) - candidate)) <= 1.0e-8,
                f"REPORT_LINK_CANDIDATE:{link_name}")
        require(float(np.linalg.norm(v3(entry["union_candidate_minus_com_v2_delta_xyz_world_mm"]) - delta)) <= 1.0e-8,
                f"REPORT_LINK_DELTA_XYZ:{link_name}")
        compare_scalar(entry["union_candidate_minus_com_v2_delta_norm_mm"], float(np.linalg.norm(delta)), 1.0e-8,
                       f"REPORT_LINK_DELTA_NORM:{link_name}")
        origin = v3(authority["frame_world_at_mechanical_zero"]["origin_mm"])
        rotation = np.asarray(
            authority["frame_world_at_mechanical_zero"]["rotation_matrix_row_major"], dtype=float,
        )
        candidate_owner = rotation.T @ (candidate - origin)
        frozen_owner = v3(authority["com_link_mm"])
        compare_vector(entry["union_candidate_com_owner_link_mm"], candidate_owner, 1.0e-8,
                       f"REPORT_LINK_CANDIDATE_OWNER:{link_name}")
        compare_vector(entry["com_v2_owner_link_mm"], frozen_owner, 1.0e-12,
                       f"REPORT_LINK_COM_V2_OWNER:{link_name}")
        compare_vector(entry["union_candidate_minus_com_v2_delta_xyz_owner_link_mm"],
                       candidate_owner - frozen_owner, 1.0e-8,
                       f"REPORT_LINK_DELTA_OWNER:{link_name}")
        require(entry["com_v2_modified"] is False, f"REPORT_LINK_COM_V2_MODIFIED:{link_name}")

    containments = {entry["component_id"]: entry for entry in report["go_output_full_containment"]}
    require(set(containments) == set(GO_OUTPUTS) and len(report["go_output_full_containment"]) == 5,
            "GO_CONTAINMENT_SET")
    for component_id, pair in GO_OUTPUTS.items():
        entry = containments[component_id].get("event", containments[component_id])
        require(entry["classification"] == "FULL_CONTAINMENT", f"GO_CONTAINMENT_CLASS:{component_id}")
        indices = {
            int(entry["left"]["source_solid_index_one_based"]),
            int(entry["right"]["source_solid_index_one_based"]),
        }
        require(indices == set(pair), f"GO_CONTAINMENT_INDICES:{component_id}:{indices}")
        recipe = report_components[component_id]["canonical_recipe"]
        decisions = recipe["containment_and_exact_duplicate_collapse"]
        require(any(decision["classification"] == "FULL_CONTAINMENT"
                    and decision["collapsed_atom_physical_component_membership_retained"] is True
                    for decision in decisions), f"GO_NEUTRAL_NOT_RETAINED:{component_id}")


def static_builder_audit() -> None:
    source = (ROOT / BUILDER).read_text(encoding="utf-8")
    require("collision_proxy_used" in source, "BUILDER_NO_COLLISION_FLAG")
    require("residual_sensitivity_recalculated" in source, "BUILDER_NO_RESIDUAL_FLAG")
    require("final_inertia_frozen" in source, "BUILDER_NO_INERTIA_FLAG")
    require("removeSplitter" in source and ("multiFuse" in source or ".fuse(" in source), "BUILDER_NO_OCCT_UNION")
    require(FAIL_STATUS in source, "BUILDER_NO_EXACT_FAIL_STATUS")
    require('"authoritative": False' in source, "BUILDER_NO_FALSE_AUTHORITY_FLAG")
    require("reproduced_blocker_gates" in source, "BUILDER_NO_REPRODUCED_BLOCKER_GATES")
    require("reproduced_blocker_gates" in source and "supplemental_read_only_probe_evidence" in source,
            "BUILDER_NO_EXHAUSTIVE_FAILURE_EVIDENCE")
    require("V15.16 MASS_GEOMETRY_AUTHORITY_V1 = PASS" not in source,
            "BUILDER_CONTAINS_PASS_AUTHORITY_STATUS")
    require("random" not in source.lower(), "BUILDER_RANDOMNESS")
    for token in (".urdf", ".xacro", ".srdf", ".mjcf"):
        require(token not in source.lower(), f"BUILDER_DEPLOYMENT_TOKEN:{token}")


def run_builder_check_with_snapshots(protected_before: dict[str, str]) -> None:
    output_before = output_snapshot()
    completed = subprocess.run(
        [sys.executable, str(ROOT / BUILDER), "--check"], cwd=ROOT,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    require(completed.returncode == 0, "BUILDER_CHECK_FAILED:\n" + completed.stdout)
    output_after = output_snapshot()
    protected_after = protected_snapshot()
    require(output_before == output_after, "BUILDER_CHECK_OUTPUT_TOCTOU")
    require(protected_before == protected_after, "BUILDER_CHECK_PROTECTED_TOCTOU")


def main() -> int:
    doc = None
    try:
        verify_git_scope()
        protected_before = protected_snapshot()
        static_builder_audit()
        mass, com, components, mass_map = load_authorities()
        print_cache = build_print_cache(components)
        doc = App.openDocument(str(ROOT / SOURCE_CAD))
        atoms_by_component = build_atoms(doc, components, print_cache)
        raw_events, pair_count = classify_original_pairs(atoms_by_component)
        print(f"Independent raw audit: atoms=266 pairs={pair_count} positive=91 classes={EXPECTED_CLASS_COUNTS}")
        report = json.loads((ROOT / REPORT_JSON).read_text(encoding="utf-8"))
        validate_fail_report(report, components, mass_map, raw_events, atoms_by_component)
        run_builder_check_with_snapshots(protected_before)
        print("Independent blocker audit: unresolved=3 stator solid_22<->solid_24; authority established=false")
        print("TOTAL PASS: V15.16A FAIL audit independently validated")
        return 0
    except (ValidationError, KeyError, ValueError, TypeError, OSError) as exc:
        print(f"V15.16 MASS_GEOMETRY_AUTHORITY_V1 VALIDATION = FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        if doc is not None:
            App.closeDocument(doc.Name)


if __name__ == "__main__":
    raise SystemExit(main())
