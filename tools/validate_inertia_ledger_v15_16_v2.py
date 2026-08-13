#!/usr/bin/env python3
"""Independently validate the V15.16 V2 rigid-inertia *failure audit*.

This program intentionally does not import the builder.  It reconstructs the
four STL volume integrals from raw binary triangles, obtains a second path via
OpenCASCADE, opens the source CAD read-only, positively normalizes every
selected physical solid, and independently reproduces the 17-component mass,
COM, candidate link tensors, residual sensitivities, and overlap blocker.

The expected result is a correctly evidenced FAIL audit.  A PASS inertia
ledger, a deployment file, or mutation of an authority is itself a validator
failure.  The builder ``--check`` is invoked only after all independent gates;
hash snapshots around it make that final reproducibility check fail closed.
"""

from __future__ import annotations

import ast
import hashlib
import itertools
import json
import math
import struct
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    import FreeCAD as App
    import Mesh
    import MeshPart
    import Part
    import numpy as np
except ImportError as exc:  # pragma: no cover - runtime gate
    raise SystemExit(
        "Run with FreeCAD Python (expected D:/freecad/bin/python.exe): " + str(exc)
    )


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "379b7174d276282d08295c7eec2592558a367ad9"
SOURCE_TREE = "8b098c540edf7a970b8f90cba9ededd9d1957cd2"
TARGET_BRANCH = "agent/v15-16-inertia-v2"

MASS_LEDGER = "V15_15_实测质量账本_v1.json"
COM_LEDGER = "V15_15_COM账本_v2.json"
COM_V1_JSON = "V15_15_COM账本_v1.json"
COM_V1_MD = "V15_15_COM账本_v1.md"
SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
MEMBERSHIP = "rigid_links_v15_13/rigid_link_membership.csv"
MANIFEST = "rigid_links_v15_13/rigid_link_manifest.json"
MATH_TEST = "tools/test_inertia_math_v15_16_v2.py"
BUILDER = "tools/build_inertia_ledger_v15_16_v2.py"
VALIDATOR = "tools/validate_inertia_ledger_v15_16_v2.py"
FAIL_JSON = "V15_16_刚体惯量_FAIL审计_v2.json"
FAIL_MD = "V15_16_刚体惯量_FAIL审计_v2.md"
SUITABILITY_JSON = "V15_16_打印件惯量几何适用性报告.json"
PASS_JSON = "V15_16_刚体惯量账本_v2.json"
PASS_MD = "V15_16_刚体惯量账本_v2.md"

ALLOWED_CHANGED_PATHS = {
    FAIL_JSON,
    FAIL_MD,
    SUITABILITY_JSON,
    BUILDER,
    VALIDATOR,
    MATH_TEST,
}

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
    COM_V1_JSON: "13e3470821541d7ae5c3f10223c51339d7d46df0d118a8e4dbeac2480c462d32",
    COM_V1_MD: "8875ff34ab6c091b2d5de6fff418553b0834a141adf32f4c88aac7d1d916d11a",
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
}

LINK_ORDER = ("link2", "link3", "link4", "link5", "link6", "gripper")
EXPECTED_LINK_MASSES = {
    "link2": 0.7615,
    "link3": 1.0900,
    "link4": 0.6750,
    "link5": 0.5040,
    "link6": 0.1250,
    "gripper": 0.2960,
}
EXPECTED_TOTAL_MASS = 3.4515
EXPECTED_COMPONENT_COUNT = 17
OUTPUT_COMPONENTS = (
    "J2A_OUTPUT_EQ",
    "J2B_OUTPUT_EQ",
    "J3_OUTPUT_EQ",
    "J4_OUTPUT_EQ",
    "J5_OUTPUT_EQ",
)
RESIDUALS = {
    "RESIDUAL_A_LINK5_ADAPTER_HARDWARE_WIRING": "A",
    "RESIDUAL_B_LINK4_J5_INTERFACE_HARDWARE_WIRING": "B",
    "RESIDUAL_C_LINK3_FOREARM_HARDWARE_WIRING": "C",
}

MM_TO_M = 1.0e-3
MM2_TO_M2 = 1.0e-6
IDENTITY3 = np.eye(3, dtype=float)
COM_TOL_M = 1.0e-8


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


def run_git(*arguments: str, check: bool = True) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        ["git", "-C", str(ROOT), "-c", "core.quotepath=false", *arguments],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def changed_paths() -> set[str]:
    tracked = {
        value.decode("utf-8").replace("\\", "/")
        for value in run_git("diff", "--name-only", "-z", SOURCE_COMMIT, "--").stdout.split(b"\0")
        if value
    }
    untracked = {
        value.decode("utf-8").replace("\\", "/")
        for value in run_git("ls-files", "--others", "--exclude-standard", "-z").stdout.split(b"\0")
        if value
    }
    return tracked | untracked


def verify_git_scope() -> None:
    require(run_git("branch", "--show-current").stdout.decode().strip() == TARGET_BRANCH, "WRONG_BRANCH")
    source_tree = run_git("rev-parse", f"{SOURCE_COMMIT}^{{tree}}").stdout.decode().strip()
    require(source_tree == SOURCE_TREE, f"SOURCE_TREE_MISMATCH:{source_tree}")
    require(
        run_git("merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD", check=False).returncode == 0,
        "SOURCE_COMMIT_NOT_ANCESTOR",
    )
    actual = changed_paths()
    require(actual == ALLOWED_CHANGED_PATHS, f"CHANGED_PATHS_NOT_EXACT:{sorted(actual)}")
    require(not (ROOT / PASS_JSON).exists() and not (ROOT / PASS_MD).exists(), "PASS_LEDGER_EXISTS")
    prohibited_suffixes = (".urdf", ".xacro", ".srdf", ".mjcf")
    require(not any(path.lower().endswith(prohibited_suffixes) for path in actual), "DEPLOYMENT_FILE_CHANGED")


def protected_snapshot() -> dict[str, str]:
    result: dict[str, str] = {}
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
        require(path.is_file(), f"ALLOWED_OUTPUT_MISSING:{relative}")
        result[relative] = sha256_file(path)
    return result


def run_math_test() -> None:
    completed = subprocess.run(
        [sys.executable, str(ROOT / MATH_TEST)],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    require(completed.returncode == 0, "MATH_TEST_FAILED:\n" + completed.stdout)
    for marker in (
        "PASS A:", "PASS B:", "PASS C:", "PASS D:", "PASS E:",
        "PASS UNIT_AUDIT", "TOTAL PASS",
    ):
        require(marker in completed.stdout, f"MATH_TEST_MARKER:{marker}")


def v3(value: Any) -> np.ndarray:
    if hasattr(value, "x"):
        return np.array([float(value.x), float(value.y), float(value.z)], dtype=float)
    return np.asarray(value, dtype=float).reshape(3)


def symmetrize(matrix: Any) -> np.ndarray:
    value = np.asarray(matrix, dtype=float).reshape(3, 3)
    return 0.5 * (value + value.T)


def matrix_of_inertia(shape: Any) -> np.ndarray:
    return np.asarray(shape.MatrixOfInertia.A, dtype=float).reshape(4, 4)[:3, :3].copy()


def frobenius(matrix: Any) -> float:
    return float(np.linalg.norm(np.asarray(matrix, dtype=float), ord="fro"))


def relative_frobenius(left: Any, right: Any) -> float:
    denominator = frobenius(right)
    require(denominator > 0.0, "ZERO_TENSOR_DENOMINATOR")
    return frobenius(np.asarray(left) - np.asarray(right)) / denominator


def parallel_axis(inertia_com: Any, mass_kg: float, displacement_m: Any) -> np.ndarray:
    displacement = v3(displacement_m)
    return symmetrize(inertia_com) + mass_kg * (
        float(displacement @ displacement) * IDENTITY3 - np.outer(displacement, displacement)
    )


def rotation_translation(rows: Sequence[Sequence[float]]) -> tuple[np.ndarray, np.ndarray]:
    matrix = np.asarray(rows, dtype=float).reshape(4, 4)
    return matrix[:3, :3], matrix[:3, 3]


def parse_binary_stl(path: Path) -> tuple[list[np.ndarray], list[tuple[int, int, int]]]:
    data = path.read_bytes()
    require(len(data) >= 84, f"STL_TOO_SHORT:{path.name}")
    count = struct.unpack_from("<I", data, 80)[0]
    require(len(data) == 84 + 50 * count, f"STL_NOT_CANONICAL_BINARY:{path.name}")
    vertices: list[np.ndarray] = []
    faces: list[tuple[int, int, int]] = []
    welded: dict[tuple[float, float, float], int] = {}
    for index in range(count):
        record = struct.unpack_from("<12fH", data, 84 + 50 * index)
        face: list[int] = []
        for offset in (3, 6, 9):
            point = tuple(float(record[offset + axis]) for axis in range(3))
            require(all(math.isfinite(value) for value in point), f"STL_NONFINITE:{path.name}")
            if point not in welded:
                welded[point] = len(vertices)
                vertices.append(np.array(point, dtype=float))
            face.append(welded[point])
        faces.append(tuple(face))
    return vertices, faces


def topology_audit(vertices: Sequence[np.ndarray], faces: Sequence[Sequence[int]]) -> dict[str, Any]:
    edges: dict[tuple[int, int], list[int]] = defaultdict(list)
    degenerate = 0
    for face in faces:
        i0, i1, i2 = (int(index) for index in face)
        if len({i0, i1, i2}) != 3:
            degenerate += 1
            continue
        area2 = float(np.linalg.norm(np.cross(vertices[i1] - vertices[i0], vertices[i2] - vertices[i0])))
        if not math.isfinite(area2) or area2 <= 0.0:
            degenerate += 1
            continue
        for start, end in ((i0, i1), (i1, i2), (i2, i0)):
            key = (min(start, end), max(start, end))
            edges[key].append(1 if (start, end) == key else -1)
    bad_incidence = sum(len(value) != 2 for value in edges.values())
    conflicts = sum(len(value) == 2 and sum(value) != 0 for value in edges.values())
    return {
        "points": len(vertices),
        "facets": len(faces),
        "bad_incidence": bad_incidence,
        "orientation_conflicts": conflicts,
        "degenerate": degenerate,
        "pass": bad_incidence == 0 and conflicts == 0 and degenerate == 0,
    }


def anchored_properties(
    vertices: Sequence[np.ndarray], faces: Sequence[Sequence[int]], mass_kg: float
) -> dict[str, Any]:
    topology = topology_audit(vertices, faces)
    require(topology["pass"], f"STL_TOPOLOGY:{topology}")
    points = np.vstack(vertices)
    anchor = 0.5 * (np.min(points, axis=0) + np.max(points, axis=0))
    volumes: list[float] = []
    first: list[list[float]] = [[], [], []]
    second: list[list[list[float]]] = [[[], [], []] for _ in range(3)]
    for face in faces:
        a, b, c = (vertices[int(index)] - anchor for index in face)
        signed = float(a @ np.cross(b, c)) / 6.0
        volumes.append(signed)
        coordinate_sum = a + b + c
        for row in range(3):
            first[row].append(signed * float(coordinate_sum[row]) / 4.0)
            for column in range(3):
                vertex_dot = float(a[row] * a[column] + b[row] * b[column] + c[row] * c[column])
                second[row][column].append(
                    signed * (float(coordinate_sum[row] * coordinate_sum[column]) + vertex_dot) / 20.0
                )
    signed_volume = math.fsum(volumes)
    require(math.isfinite(signed_volume) and signed_volume != 0.0, "STL_ZERO_VOLUME")
    sign = 1.0 if signed_volume > 0.0 else -1.0
    volume = abs(signed_volume)
    first_anchor = np.array([sign * math.fsum(values) for values in first])
    second_anchor = np.array([
        [sign * math.fsum(second[row][column]) for column in range(3)]
        for row in range(3)
    ])
    offset = first_anchor / volume
    com = anchor + offset
    central_second = second_anchor - volume * np.outer(offset, offset)
    raw_inertia = float(np.trace(central_second)) * IDENTITY3 - central_second
    inertia = symmetrize(raw_inertia * (mass_kg / volume) * MM2_TO_M2)
    require(np.all(np.isfinite(inertia)) and np.all(np.isfinite(com)), "STL_PROPERTY_NONFINITE")
    return {
        "topology": topology,
        "anchor_mm": anchor,
        "signed_volume_mm3": signed_volume,
        "volume_mm3": volume,
        "com_mm": com,
        "raw_inertia_mm5": raw_inertia,
        "inertia_kg_m2": inertia,
    }


def positive_solid(shape_input: Any) -> tuple[Any, bool]:
    shape = shape_input.copy()
    reversed_to_positive = False
    if float(shape.Volume) < 0.0:
        shape.reverse()
        reversed_to_positive = True
    require(float(shape.Volume) > 0.0, "SOLID_NONPOSITIVE")
    require(shape.isValid() and shape.isClosed(), "SOLID_INVALID_OR_OPEN")
    return shape, reversed_to_positive


def occt_properties(shape_input: Any, mass_kg: float) -> dict[str, Any]:
    shape, reversed_to_positive = positive_solid(shape_input)
    volume = float(shape.Volume)
    raw = matrix_of_inertia(shape)
    inertia = symmetrize(raw * (mass_kg / volume) * MM2_TO_M2)
    com = v3(shape.CenterOfMass)
    require(np.all(np.isfinite(inertia)) and np.all(np.isfinite(com)), "OCCT_PROPERTY_NONFINITE")
    return {
        "shape": shape,
        "volume_mm3": volume,
        "com_mm": com,
        "raw_inertia_mm5": raw,
        "inertia_kg_m2": inertia,
        "reversed": reversed_to_positive,
    }


def placement_from_rows(rows: np.ndarray) -> Any:
    matrix = App.Matrix()
    for row in range(4):
        for column in range(4):
            setattr(matrix, f"A{row + 1}{column + 1}", float(rows[row, column]))
    return App.Placement(matrix)


def independent_occt_semantics_gate() -> None:
    dimensions_mm = np.array([820.0, 470.0, 310.0])
    mass_kg = 7.3
    shape = Part.makeBox(
        *dimensions_mm.tolist(), App.Vector(*(-0.5 * dimensions_mm).tolist())
    )
    actual = occt_properties(shape, mass_kg)
    dimensions_m = dimensions_mm * MM_TO_M
    expected = np.diag([
        mass_kg * (dimensions_m[1] ** 2 + dimensions_m[2] ** 2) / 12.0,
        mass_kg * (dimensions_m[0] ** 2 + dimensions_m[2] ** 2) / 12.0,
        mass_kg * (dimensions_m[0] ** 2 + dimensions_m[1] ** 2) / 12.0,
    ])
    require(relative_frobenius(actual["inertia_kg_m2"], expected) < 1.0e-10,
            "OCCT_AXIS_BOX_SEMANTICS")
    angles = [math.radians(value) for value in (13.0, -21.0, 37.0)]
    rx, ry, rz = angles
    rotation_x = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    rotation_y = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    rotation_z = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    rotation = rotation_z @ rotation_y @ rotation_x
    rows = np.eye(4)
    rows[:3, :3] = rotation
    rows[:3, 3] = [110.0, -70.0, 50.0]
    transformed = shape.copy()
    transformed.Placement = placement_from_rows(rows)
    rotated = occt_properties(transformed, mass_kg)
    expected_rotated = rotation @ expected @ rotation.T
    require(relative_frobenius(rotated["inertia_kg_m2"], expected_rotated) < 1.0e-10,
            "OCCT_ROTATED_BOX_SEMANTICS")
    require(float(np.linalg.norm(rotated["com_mm"] - rows[:3, 3])) < 1.0e-10,
            "OCCT_MATRIX_REFERENCE_NOT_COM")
    require(max(abs(expected_rotated[0, 1]), abs(expected_rotated[0, 2]), abs(expected_rotated[1, 2])) > 0.0,
            "OCCT_ROTATED_BOX_OFFDIAGONAL_ZERO")


def tessellated_properties(shape_input: Any, mass_kg: float, deflection_mm: float) -> dict[str, Any]:
    mesh = MeshPart.meshFromShape(
        Shape=shape_input.cleaned(),
        LinearDeflection=float(deflection_mm),
        AngularDeflection=0.5,
        Relative=False,
    )
    # A detached working copy is allowed for triangulation diagnostics only.
    # These calls remove exact degeneracies and orient triangle winding; the
    # authoritative BRep is neither saved nor modified.
    working = Mesh.Mesh(mesh)
    working.harmonizeNormals()
    working.fixDegenerations(0.0)
    require(working.isSolid() and not working.hasNonManifolds(), "BREP_TESSELLATION_TOPOLOGY")
    points, facets = working.Topology
    vertices = [v3(point) for point in points]
    faces = [tuple(int(index) for index in face) for face in facets]
    poly = anchored_properties(vertices, faces, mass_kg)
    return {
        "volume_mm3": poly["volume_mm3"],
        "com_world_mm": poly["com_mm"],
        "inertia_com_world_kg_m2": poly["inertia_kg_m2"],
        "point_count": int(working.CountPoints),
        "triangle_count": int(working.CountFacets),
        "deflection_mm": deflection_mm,
    }


def part_from_mesh(path: Path) -> tuple[Any, Any]:
    mesh = Mesh.Mesh(str(path))
    require(mesh.isSolid(), f"PRINT_NOT_CLOSED:{path.name}")
    require(not mesh.hasNonManifolds(), f"PRINT_NONMANIFOLD:{path.name}")
    require(not mesh.hasSelfIntersections(), f"PRINT_SELF_INTERSECTIONS:{path.name}")
    shape = Part.Shape()
    shape.makeShapeFromMesh(mesh.Topology, 0.001)
    require(len(shape.Shells) == 1 and shape.isClosed() and shape.isValid(), f"PRINT_PART_SHELL:{path.name}")
    solid = Part.makeSolid(shape.Shells[0])
    solid, _ = positive_solid(solid)
    return mesh, solid


def audit_prints(com_components: Sequence[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    cache: dict[str, dict[str, Any]] = {}
    members_by_path = {
        member.get("artifact_path"): (component, member)
        for component in com_components
        for member in component.get("members", [])
        if member.get("artifact_path")
    }
    for member_name, relative in PRINT_PATHS.items():
        require(relative in members_by_path, f"PRINT_NOT_IN_COM_V2:{relative}")
        component, member = members_by_path[relative]
        path = ROOT / relative
        vertices, faces = parse_binary_stl(path)
        poly = anchored_properties(vertices, faces, 1.0)
        require(poly["signed_volume_mm3"] > 0.0, f"PRINT_WINDING:{relative}")
        mesh, solid = part_from_mesh(path)
        occt = occt_properties(solid, 1.0)
        volume_error = abs(occt["volume_mm3"] - poly["volume_mm3"]) / poly["volume_mm3"]
        com_error_m = float(np.linalg.norm(occt["com_mm"] - poly["com_mm"]) * MM_TO_M)
        inertia_error = relative_frobenius(occt["inertia_kg_m2"], poly["inertia_kg_m2"])
        require(volume_error < 1.0e-8, f"PRINT_VOLUME_PATHS:{relative}:{volume_error}")
        require(com_error_m < 1.0e-7, f"PRINT_COM_PATHS:{relative}:{com_error_m}")
        require(inertia_error < 1.0e-5, f"PRINT_INERTIA_PATHS:{relative}:{inertia_error}")
        rotation, translation = rotation_translation(member["source_placement_matrix_row_major"])
        world_com = rotation @ poly["com_mm"] + translation
        # Three V2-generated member records intentionally do not duplicate a
        # top-level world centroid.  Their placement is independently applied
        # here; the authoritative comparison is the component/link COM gate
        # below, using all four print atoms and their frozen masses.
        if "center_world_mm" in member:
            require(
                float(np.linalg.norm(world_com - v3(member["center_world_mm"])) * MM_TO_M) < COM_TOL_M,
                f"PRINT_COM_V2:{relative}",
            )
        cache[relative] = {
            "member_name": member_name,
            "component_id": component["component_id"],
            "member": member,
            "poly": poly,
            "occt": occt,
            "solid": solid,
            "rotation": rotation,
            "translation": translation,
            "mesh_points": int(mesh.CountPoints),
            "mesh_facets": int(mesh.CountFacets),
            "volume_error": volume_error,
            "com_error_m": com_error_m,
            "inertia_error": inertia_error,
        }
    return cache


def load_authorities() -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    mass = json.loads((ROOT / MASS_LEDGER).read_text(encoding="utf-8"))
    com = json.loads((ROOT / COM_LEDGER).read_text(encoding="utf-8"))
    require(com["final_status"] == "V15.15 COM_LEDGER_V2 = PASS", "COM_V2_AUTHORITY_STATUS")
    require(len(com["components"]) == EXPECTED_COMPONENT_COUNT, "COM_COMPONENT_COUNT")
    require(com["component_order"] == [item["component_id"] for item in com["components"]], "COM_COMPONENT_ORDER")
    mass_entries = mass["component_to_link_mapping"]["additive_components"]
    mass_map = {entry["component_id"]: entry for entry in mass_entries}
    require(set(mass_map) == set(com["component_order"]), "MASS_COM_COMPONENT_SET")
    for component in com["components"]:
        authority = mass_map[component["component_id"]]
        require(authority["ledger_link"] == component["owner_link"], f"OWNER_MISMATCH:{component['component_id']}")
        require(abs(float(authority["nominal_mass_kg"]) - float(component["frozen_mass_kg"])) < 1.0e-15,
                f"COMPONENT_MASS_MISMATCH:{component['component_id']}")
        require(authority["cad_members"] == component["geometry_member_order_inherited_exactly_from_v1_mass_ledger"],
                f"COMPONENT_MEMBERSHIP_MISMATCH:{component['component_id']}")
    return mass, com, list(com["components"]), mass_map


def component_print_atoms(
    component: dict[str, Any], mass_kg: float, print_cache: dict[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    allocations: dict[str, float] = {}
    if component["component_id"] == "UPPER_ARM_PRINT_MEASURED":
        split = component["equal_density_volume_allocation"]
        allocations = {part["member"]: float(part["allocated_mass_kg"]) for part in split["parts"]}
        require(abs(math.fsum(allocations.values()) - mass_kg) < 1.0e-12, "UPPER_AB_MASS_CLOSURE")
    else:
        require(len(component["members"]) == 1, f"PRINT_MEMBER_COUNT:{component['component_id']}")
        allocations[component["members"][0]["member"]] = mass_kg
    atoms = []
    for member in component["members"]:
        cache = print_cache[member["artifact_path"]]
        atom_mass = allocations[member["member"]]
        rotation, translation = cache["rotation"], cache["translation"]
        occt = cache["occt"]
        poly = cache["poly"]
        world_com = rotation @ occt["com_mm"] + translation
        world_inertia = symmetrize(rotation @ (occt["raw_inertia_mm5"] * (atom_mass / occt["volume_mm3"]) * MM2_TO_M2) @ rotation.T)
        poly_world_com = rotation @ poly["com_mm"] + translation
        poly_world_inertia = symmetrize(
            rotation
            @ (poly["raw_inertia_mm5"] * (atom_mass / poly["volume_mm3"]) * MM2_TO_M2)
            @ rotation.T
        )
        world_shape = cache["solid"].copy()
        matrix = App.Matrix()
        rows = member["source_placement_matrix_row_major"]
        for row in range(4):
            for column in range(4):
                setattr(matrix, f"A{row + 1}{column + 1}", float(rows[row][column]))
        world_shape.Placement = App.Placement(matrix)
        atoms.append({
            "component_id": component["component_id"],
            "member": member["member"],
            "source_object": member["member"],
            "solid_index": 1,
            "mass_kg": atom_mass,
            "volume_mm3": occt["volume_mm3"],
            "com_world_mm": world_com,
            "inertia_com_world_kg_m2": world_inertia,
            "shape": world_shape,
            "print": True,
            "print_path_b": {
                "volume_mm3": poly["volume_mm3"],
                "com_world_mm": poly_world_com,
                "inertia_com_world_kg_m2": poly_world_inertia,
            },
        })
    return atoms


def component_brep_atoms(doc: Any, component: dict[str, Any], mass_kg: float) -> list[dict[str, Any]]:
    unresolved = []
    for member in component["members"]:
        token = str(member["member"])
        require("collision" not in token.lower() and "proxy" not in token.lower(), f"COLLISION_PROXY:{token}")
        object_name = str(member.get("source_object") or token)
        obj = doc.getObject(object_name)
        require(obj is not None and obj.TypeId == "Part::Feature", f"BREP_OBJECT:{object_name}")
        solids = list(obj.Shape.Solids)
        selected = member.get("selected_solid_indices")
        indices = [int(value) for value in selected] if selected else list(range(1, len(solids) + 1))
        require(indices and all(1 <= value <= len(solids) for value in indices), f"BREP_SELECTION:{token}")
        for index in indices:
            properties = occt_properties(solids[index - 1], 1.0)
            unresolved.append({
                "component_id": component["component_id"],
                "member": token,
                "source_object": object_name,
                "solid_index": index,
                "volume_mm3": properties["volume_mm3"],
                "com_world_mm": properties["com_mm"],
                "raw_inertia_mm5": properties["raw_inertia_mm5"],
                "shape": properties["shape"],
                "reversed": properties["reversed"],
                "print": False,
            })
    total_volume = math.fsum(atom["volume_mm3"] for atom in unresolved)
    require(total_volume > 0.0, f"COMPONENT_VOLUME:{component['component_id']}")
    for atom in unresolved:
        atom_mass = mass_kg * atom["volume_mm3"] / total_volume
        atom["mass_kg"] = atom_mass
        atom["inertia_com_world_kg_m2"] = symmetrize(
            atom.pop("raw_inertia_mm5") * (atom_mass / atom["volume_mm3"]) * MM2_TO_M2
        )
    require(abs(math.fsum(atom["mass_kg"] for atom in unresolved) - mass_kg) < 1.0e-12,
            f"ATOM_MASS_CLOSURE:{component['component_id']}")
    return unresolved


def weighted_com(atoms: Sequence[dict[str, Any]]) -> np.ndarray:
    mass = math.fsum(float(atom["mass_kg"]) for atom in atoms)
    return np.array([
        math.fsum(float(atom["mass_kg"]) * float(atom["com_world_mm"][axis]) for atom in atoms) / mass
        for axis in range(3)
    ])


def aggregate_tensor(atoms: Sequence[dict[str, Any]], reference_world_mm: Any) -> np.ndarray:
    reference = v3(reference_world_mm)
    result = np.zeros((3, 3), dtype=float)
    for atom in atoms:
        displacement = (v3(atom["com_world_mm"]) - reference) * MM_TO_M
        result += parallel_axis(atom["inertia_com_world_kg_m2"], float(atom["mass_kg"]), displacement)
    return symmetrize(result)


def aggregate_atom_path(
    atoms: Sequence[dict[str, Any]], path_key: str, reference_world_mm: Any
) -> tuple[np.ndarray, np.ndarray]:
    reference = v3(reference_world_mm)
    total_mass = math.fsum(float(atom["mass_kg"]) for atom in atoms)
    center = np.array([
        math.fsum(float(atom["mass_kg"]) * float(atom[path_key]["com_world_mm"][axis]) for atom in atoms)
        / total_mass
        for axis in range(3)
    ])
    tensor = np.zeros((3, 3))
    for atom in atoms:
        displacement = (v3(atom[path_key]["com_world_mm"]) - reference) * MM_TO_M
        tensor += parallel_axis(atom[path_key]["inertia_com_world_kg_m2"], float(atom["mass_kg"]), displacement)
    return center, symmetrize(tensor)


def independent_brep_path_b(
    component_records: dict[str, dict[str, Any]], atoms_by_component: dict[str, list[dict[str, Any]]]
) -> dict[str, dict[str, Any]]:
    results: dict[str, dict[str, Any]] = {}
    for component_id, atoms in atoms_by_component.items():
        if all(atom["print"] for atom in atoms):
            for atom in atoms:
                # For a frozen print artifact Path B is the independently
                # parsed raw STL tetra integral, never the OCCT reimport.
                atom["path_b_normal"] = dict(atom["print_path_b"])
                atom["path_b_fine"] = dict(atom["print_path_b"])
        else:
            for atom in atoms:
                atom["path_b_normal"] = tessellated_properties(atom["shape"], atom["mass_kg"], 0.10)
                atom["path_b_fine"] = tessellated_properties(atom["shape"], atom["mass_kg"], 0.03)
        frozen = component_records[component_id]["frozen_com_world_mm"]
        normal_com, normal_tensor = aggregate_atom_path(atoms, "path_b_normal", frozen)
        fine_com, fine_tensor = aggregate_atom_path(atoms, "path_b_fine", frozen)
        convergence = relative_frobenius(normal_tensor, fine_tensor)
        selected_key = "path_b_fine"
        selected_com, selected_tensor = fine_com, fine_tensor
        ultra_error = None
        if convergence > 0.005 and not all(atom["print"] for atom in atoms):
            for atom in atoms:
                atom["path_b_ultra"] = tessellated_properties(atom["shape"], atom["mass_kg"], 0.01)
            selected_key = "path_b_ultra"
            selected_com, selected_tensor = aggregate_atom_path(atoms, selected_key, frozen)
            ultra_error = relative_frobenius(fine_tensor, selected_tensor)
            require(ultra_error < 0.005, f"COMPONENT_FINE_ULTRA_CONVERGENCE:{component_id}:{ultra_error}")
        path_a_tensor = component_records[component_id]["tensor_about_component_com_world"]
        path_error = relative_frobenius(path_a_tensor, selected_tensor)
        com_error_m = float(np.linalg.norm(selected_com - frozen) * MM_TO_M)
        require(path_error < 0.01, f"COMPONENT_PATH_A_B:{component_id}:{path_error}")
        # A tessellation is an approximation path for tensor convergence; its
        # centroid may differ by micrometres at 0.03 mm deflection.  The strict
        # 1e-8 m authority gate is independently enforced above on exact OCCT
        # geometry.  Do not silently reinterpret it as a tessellation limit.
        results[component_id] = {
            "normal_tensor": normal_tensor,
            "fine_tensor": fine_tensor,
            "selected_tensor": selected_tensor,
            "selected_key": selected_key,
            "normal_fine_error": convergence,
            "fine_ultra_error": ultra_error,
            "path_a_b_error": path_error,
            "com_error_m": com_error_m,
        }
    return results


def build_independent_geometry(
    doc: Any,
    components: Sequence[dict[str, Any]],
    mass_map: dict[str, Any],
    print_cache: dict[str, dict[str, Any]],
) -> tuple[dict[str, dict[str, Any]], dict[str, list[dict[str, Any]]]]:
    records: dict[str, dict[str, Any]] = {}
    atoms_by_component: dict[str, list[dict[str, Any]]] = {}
    for component in components:
        component_id = component["component_id"]
        mass_kg = float(mass_map[component_id]["nominal_mass_kg"])
        is_print = any(member.get("artifact_path") for member in component["members"])
        atoms = (
            component_print_atoms(component, mass_kg, print_cache)
            if is_print else component_brep_atoms(doc, component, mass_kg)
        )
        reproduced = weighted_com(atoms)
        frozen = v3(component["com_world_mm"])
        error_m = float(np.linalg.norm(reproduced - frozen) * MM_TO_M)
        require(error_m < COM_TOL_M, f"COMPONENT_COM_REPRO:{component_id}:{error_m}")
        records[component_id] = {
            "owner_link": component["owner_link"],
            "mass_kg": mass_kg,
            "frozen_com_world_mm": frozen,
            "reproduced_com_world_mm": reproduced,
            "com_error_m": error_m,
            "tensor_about_component_com_world": aggregate_tensor(atoms, frozen),
            "atom_count": len(atoms),
        }
        atoms_by_component[component_id] = atoms
    return records, atoms_by_component


def tensor_physics(matrix: Any, mass_kg: float, atoms: Sequence[dict[str, Any]], reference_mm: Any) -> dict[str, Any]:
    value = symmetrize(matrix)
    symmetry = float(np.max(np.abs(value - value.T)))
    eigenvalues = np.linalg.eigvalsh(value)
    margins = [
        float(eigenvalues[0] + eigenvalues[1] - eigenvalues[2]),
        float(eigenvalues[0] + eigenvalues[2] - eigenvalues[1]),
        float(eigenvalues[1] + eigenvalues[2] - eigenvalues[0]),
    ]
    reference = v3(reference_mm)
    farthest_mm = 0.0
    for atom in atoms:
        box = atom["shape"].BoundBox
        for point in itertools.product((box.XMin, box.XMax), (box.YMin, box.YMax), (box.ZMin, box.ZMax)):
            farthest_mm = max(farthest_mm, float(np.linalg.norm(v3(point) - reference)))
    r_max_m = farthest_mm * MM_TO_M
    radii = np.sqrt(np.maximum(eigenvalues, 0.0) / mass_kg)
    return {
        "symmetry": symmetry,
        "eigenvalues": eigenvalues,
        "margins": margins,
        "determinant": float(np.linalg.det(value)),
        "radii": radii,
        "r_max_m": r_max_m,
        "pass": (
            symmetry < 1.0e-12
            and bool(np.all(eigenvalues > 1.0e-12))
            and min(margins) >= -1.0e-12
            and float(np.linalg.det(value)) > 0.0
            and bool(np.all(radii > 0.0) and np.all(radii <= r_max_m + 1.0e-12))
        ),
    }


def build_link_diagnostics(
    com: dict[str, Any],
    component_records: dict[str, dict[str, Any]],
    atoms_by_component: dict[str, list[dict[str, Any]]],
) -> tuple[dict[str, dict[str, Any]], dict[str, float]]:
    links: dict[str, dict[str, Any]] = {}
    sensitivities: dict[str, float] = {}
    for link_name in LINK_ORDER:
        authority = com["links"][link_name]
        ids = list(authority["component_ids"])
        atoms = [atom for component_id in ids for atom in atoms_by_component[component_id]]
        mass = math.fsum(component_records[component_id]["mass_kg"] for component_id in ids)
        require(abs(mass - EXPECTED_LINK_MASSES[link_name]) < 1.0e-12, f"LINK_MASS:{link_name}")
        reproduced = weighted_com(atoms)
        frozen_world = v3(authority["com_world_mm"])
        com_error_m = float(np.linalg.norm(reproduced - frozen_world) * MM_TO_M)
        require(com_error_m < COM_TOL_M, f"LINK_COM_REPRO:{link_name}:{com_error_m}")
        tensor_world = aggregate_tensor(atoms, frozen_world)
        _, tensor_world_b = aggregate_atom_path(atoms, "path_b_selected", frozen_world)
        rotation = np.asarray(authority["frame_world_at_mechanical_zero"]["rotation_matrix_row_major"], dtype=float)
        tensor_link_via_world = symmetrize(rotation.T @ tensor_world @ rotation)
        tensor_link_direct = np.zeros((3, 3), dtype=float)
        for atom in atoms:
            intrinsic_link = rotation.T @ atom["inertia_com_world_kg_m2"] @ rotation
            displacement_link = rotation.T @ ((v3(atom["com_world_mm"]) - frozen_world) * MM_TO_M)
            tensor_link_direct += parallel_axis(intrinsic_link, float(atom["mass_kg"]), displacement_link)
        tensor_link_direct = symmetrize(tensor_link_direct)
        tensor_link_b = symmetrize(rotation.T @ tensor_world_b @ rotation)
        path_a_b_error = relative_frobenius(tensor_link_direct, tensor_link_b)
        require(path_a_b_error < 0.01, f"LINK_PATH_A_B:{link_name}:{path_a_b_error}")
        frame_error = frobenius(tensor_link_direct - tensor_link_via_world)
        require(frame_error < 1.0e-10, f"FRAME_TENSOR:{link_name}:{frame_error}")
        require(abs(float(np.trace(tensor_world)) - float(np.trace(tensor_link_direct))) < 1.0e-10,
                f"TRACE_INVARIANT:{link_name}")
        require(np.max(np.abs(np.linalg.eigvalsh(tensor_world) - np.linalg.eigvalsh(tensor_link_direct))) < 1.0e-10,
                f"PRINCIPAL_INVARIANT:{link_name}")
        physics = tensor_physics(tensor_link_direct, mass, atoms, frozen_world)
        require(physics["pass"], f"LINK_PHYSICS:{link_name}:{physics}")
        links[link_name] = {
            "mass_kg": mass,
            "frozen_com_world_mm": frozen_world,
            "frozen_com_link_m": v3(authority["com_link_m"]),
            "com_error_m": com_error_m,
            "tensor_world": tensor_world,
            "tensor_link": tensor_link_direct,
            "tensor_link_b": tensor_link_b,
            "path_a_b_error": path_a_b_error,
            "frame_error": frame_error,
            "physics": physics,
            "component_ids": ids,
        }
        residual_ids = [component_id for component_id in ids if component_id in RESIDUALS]
        for residual_id in residual_ids:
            residual_atoms = atoms_by_component[residual_id]
            nominal_residual = aggregate_tensor(residual_atoms, frozen_world)
            residual_mass = component_records[residual_id]["mass_kg"]
            residual_com = component_records[residual_id]["frozen_com_world_mm"]
            point = parallel_axis(np.zeros((3, 3)), residual_mass, (residual_com - frozen_world) * MM_TO_M)
            point_link_tensor = tensor_world - nominal_residual + point
            # The contract's denominator is the nominal uniform-proxy Link
            # tensor, not the point-model alternative.
            sensitivity = frobenius(tensor_world - point_link_tensor) / frobenius(tensor_world)
            sensitivities[RESIDUALS[residual_id]] = sensitivity
    require(set(sensitivities) == {"A", "B", "C"}, "RESIDUAL_SET")
    require(sensitivities["C"] < 0.05, f"RESIDUAL_C_NOT_BELOW_5_PERCENT:{sensitivities['C']}")
    for name in ("A", "B"):
        require(0.05 <= sensitivities[name] <= 0.10,
                f"RESIDUAL_{name}_NOT_5_TO_10_PERCENT:{sensitivities[name]}")
    return links, sensitivities


def bbox_contains(outer: Any, inner: Any, tolerance: float = 1.0e-7) -> bool:
    return (
        inner.XMin >= outer.XMin - tolerance and inner.XMax <= outer.XMax + tolerance
        and inner.YMin >= outer.YMin - tolerance and inner.YMax <= outer.YMax + tolerance
        and inner.ZMin >= outer.ZMin - tolerance and inner.ZMax <= outer.ZMax + tolerance
    )


def duplicate_overlap_audit(atoms_by_component: dict[str, list[dict[str, Any]]]) -> list[dict[str, Any]]:
    findings = []
    for component_id in OUTPUT_COMPONENTS:
        atoms = atoms_by_component[component_id]
        component_findings = []
        for left_index, right_index in itertools.combinations(range(len(atoms)), 2):
            left, right = atoms[left_index], atoms[right_index]
            candidates = []
            if bbox_contains(right["shape"].BoundBox, left["shape"].BoundBox):
                candidates.append((left, right))
            if bbox_contains(left["shape"].BoundBox, right["shape"].BoundBox):
                candidates.append((right, left))
            for inner, outer in candidates:
                common = inner["shape"].common(outer["shape"])
                common_volume = abs(float(common.Volume))
                fraction = common_volume / float(inner["volume_mm3"])
                if fraction >= 0.999:
                    component_findings.append({
                        "component_id": component_id,
                        "inner_source_object": inner["source_object"],
                        "inner_solid_index_one_based": inner["solid_index"],
                        "outer_source_object": outer["source_object"],
                        "outer_solid_index_one_based": outer["solid_index"],
                        "inner_volume_mm3": float(inner["volume_mm3"]),
                        "common_volume_mm3": common_volume,
                        "containment_fraction": fraction,
                    })
        require(component_findings, f"GO_OUTPUT_NO_FULL_CONTAINMENT:{component_id}")
        findings.extend(component_findings)
    require(len({item["component_id"] for item in findings}) == 5, "GO_OUTPUT_CONTAINMENT_INSTANCE_COUNT")
    return findings


def recursively_find_records(value: Any, key: str) -> list[dict[str, Any]]:
    found = []
    if isinstance(value, dict):
        if key in value:
            found.append(value)
        for nested in value.values():
            found.extend(recursively_find_records(nested, key))
    elif isinstance(value, list):
        for nested in value:
            found.extend(recursively_find_records(nested, key))
    return found


def compare_matrix(actual: Any, expected: Any, tolerance: float, code: str) -> None:
    difference = frobenius(np.asarray(actual, dtype=float) - np.asarray(expected, dtype=float))
    require(difference <= tolerance, f"{code}:{difference}")


def validate_reports(
    print_cache: dict[str, dict[str, Any]],
    component_records: dict[str, dict[str, Any]],
    component_path_b: dict[str, dict[str, Any]],
    links: dict[str, dict[str, Any]],
    sensitivities: dict[str, float],
    overlaps: list[dict[str, Any]],
) -> None:
    suitability = json.loads((ROOT / SUITABILITY_JSON).read_text(encoding="utf-8"))
    require(suitability["schema"] == "go-m8010-arm-v15.16-print-inertia-suitability-v2/1.0",
            "SUITABILITY_SCHEMA")
    require(len(suitability["artifacts"]) == 4, "SUITABILITY_ARTIFACT_COUNT")
    reported_prints = {entry["artifact_path"]: entry for entry in suitability["artifacts"]}
    require(set(reported_prints) == set(PRINT_PATHS.values()), "SUITABILITY_ARTIFACT_SET")
    for relative, cache in print_cache.items():
        entry = reported_prints[relative]
        require(entry["sha256"] == PROTECTED_HASHES[relative], f"SUITABILITY_HASH:{relative}")
        require(entry["status"] == "PASS", f"SUITABILITY_STATUS:{relative}")
        agreement = entry["path_a_b_agreement"]
        require(abs(float(agreement["volume_relative_error"]) - cache["volume_error"]) < 1.0e-12,
                f"SUITABILITY_VOLUME_NUMERIC:{relative}")
        require(abs(float(agreement["centroid_distance_m"]) - cache["com_error_m"]) < 1.0e-12,
                f"SUITABILITY_COM_NUMERIC:{relative}")
        require(abs(float(agreement["inertia_relative_frobenius_error"]) - cache["inertia_error"]) < 1.0e-10,
                f"SUITABILITY_INERTIA_NUMERIC:{relative}")

    audit = json.loads((ROOT / FAIL_JSON).read_text(encoding="utf-8"))
    require(audit["schema"] == "go-m8010-arm-v15.16-rigid-inertia-fail-audit-v2/1.0", "FAIL_SCHEMA")
    require("FAIL" in audit["final_status"] and "PASS" not in audit["final_status"], "FAIL_FINAL_STATUS")
    require(audit.get("authoritative_ledger_written") is False, "AUTHORITATIVE_LEDGER_WRITTEN")
    require(audit.get("freeze_permitted") is False, "FREEZE_PERMITTED")
    superseded = audit["authorities"]["superseded_com_audit_artifacts"]
    require(superseded["bytes_parsed"] is False and superseded["numeric_values_used"] is False,
            "SUPERSEDED_COM_V1_NUMERIC_USE")
    serialized = json.dumps(audit, ensure_ascii=False)
    require("MASS_GEOMETRY_DUPLICATION" in serialized, "DUPLICATION_BLOCKER_NOT_REPORTED")
    require("FAIL_WITH_MODEL_LIMITATION" in serialized, "MODEL_LIMITATION_NOT_REPORTED")
    require(PASS_JSON not in serialized and PASS_MD not in serialized, "PASS_LEDGER_REFERENCED_AS_OUTPUT")

    reported_components = {entry["component_id"]: entry for entry in audit["components"]}
    require(set(reported_components) == set(component_records), "AUDIT_COMPONENT_SET")
    for component_id, independent in component_records.items():
        entry = reported_components[component_id]
        require(abs(float(entry["mass_kg"]) - independent["mass_kg"]) < 1.0e-12,
                f"AUDIT_COMPONENT_MASS:{component_id}")
        require(abs(float(entry["com_v2_reproduction_error_m"]) - independent["com_error_m"]) < 1.0e-8,
                f"AUDIT_COMPONENT_COM_ERROR:{component_id}")
        matrix = entry["path_a_occt"]["inertia_about_component_frozen_com_world_kg_m2"]
        compare_matrix(matrix, independent["tensor_about_component_com_world"], 1.0e-10,
                       f"AUDIT_COMPONENT_TENSOR:{component_id}")
        require(abs(float(entry["path_a_b_relative_frobenius_error"])
                    - component_path_b[component_id]["path_a_b_error"]) < 1.0e-8,
                f"AUDIT_COMPONENT_PATH_AB:{component_id}")
        require(abs(float(entry["path_b_triangle"]["normal_to_fine_relative_frobenius_error"])
                    - component_path_b[component_id]["normal_fine_error"]) < 1.0e-8,
                f"AUDIT_COMPONENT_CONVERGENCE:{component_id}")
        if component_id in {
            "UPPER_ARM_PRINT_MEASURED",
            "FOREARM_PRINT_MEASURED",
            "WRIST_PRELINK_PRINT_MEASURED",
        }:
            require(entry["source_status"] == "ENGINEERING_ESTIMATE_REPAIRED_WATERTIGHT_INERTIA",
                    f"PRINT_SOURCE_STATUS:{component_id}")
            require(entry["mass_basis_status"] == "MEASURED_MASS_SCALED_GEOMETRIC_INERTIA_ESTIMATE",
                    f"PRINT_MASS_BASIS_STATUS:{component_id}")
            require(entry["limitations"]["canonical_geometry_is_working_copy_repair"] is True,
                    f"PRINT_REPAIR_LIMITATION:{component_id}")

    reported_links = {entry["link"]: entry for entry in audit["links"]}
    require(set(reported_links) == set(links), "AUDIT_LINK_SET")
    for link_name, independent in links.items():
        entry = reported_links[link_name]
        require(abs(float(entry["mass_kg"]) - independent["mass_kg"]) < 1.0e-12,
                f"AUDIT_LINK_MASS:{link_name}")
        require(abs(float(entry["com_v2_reproduction"]["max_error_m"]) - independent["com_error_m"]) < 1.0e-8,
                f"AUDIT_LINK_COM:{link_name}")
        candidate = entry["candidate_uniform_proxy_path_a_occt"]["candidate_inertia"]
        compare_matrix(candidate["matrix_kg_m2"], independent["tensor_link"], 1.0e-10,
                       f"AUDIT_LINK_TENSOR:{link_name}")
        require(abs(float(entry["path_a_b_relative_frobenius_error"])
                    - independent["path_a_b_error"]) < 1.0e-8,
                f"AUDIT_LINK_PATH_AB:{link_name}")

    residual_text = json.dumps(audit["residual_sensitivity"], ensure_ascii=False)
    for name, sensitivity in sensitivities.items():
        require(f"RESIDUAL_{name}" in residual_text, f"AUDIT_RESIDUAL_MISSING:{name}")
        records = [record for record in recursively_find_records(audit["residual_sensitivity"], "relative_sensitivity")
                   if f"RESIDUAL_{name}" in json.dumps(record, ensure_ascii=False)]
        require(records, f"AUDIT_RESIDUAL_RECORD:{name}")
        require(any(abs(float(record["relative_sensitivity"]) - sensitivity) < 1.0e-10 for record in records),
                f"AUDIT_RESIDUAL_VALUE:{name}:{sensitivity}")

    reported_overlap = audit["duplicate_overlap_audit"]
    require(reported_overlap.get("pass") is False, "AUDIT_OVERLAP_NOT_FAIL")
    require(len(overlaps) >= 5, "INDEPENDENT_OVERLAP_COUNT")
    selection_checks = reported_overlap["go_output_neutral_selection_checks"]
    expected_go_components = {
        "J2A_OUTPUT_EQ", "J2B_OUTPUT_EQ", "J3_OUTPUT_EQ", "J4_OUTPUT_EQ", "J5_OUTPUT_EQ"
    }
    require(len(selection_checks) == 5, f"GO_SELECTION_CHECK_COUNT:{len(selection_checks)}")
    require({entry["component_id"] for entry in selection_checks} == expected_go_components,
            "GO_SELECTION_CHECK_COMPONENT_SET")
    for entry in selection_checks:
        require(entry["output_and_neutral_selection_disjoint"] is True,
                f"GO_SELECTION_NOT_DISJOINT:{entry['component_id']}")
        require(entry["all_selection_identity_pairs_disjoint"] is True,
                f"GO_IDENTITY_PAIR_AGGREGATE:{entry['component_id']}")
        pairs = entry["selection_identity_pairs"]
        require(len(pairs) == 1, f"GO_IDENTITY_PAIR_COUNT:{entry['component_id']}:{len(pairs)}")
        pair = pairs[0]
        require(pair["disjoint"] is True, f"GO_IDENTITY_PAIR_NOT_DISJOINT:{entry['component_id']}")
        if pair["identity_basis"] == "SAME_SOURCE_OBJECT_DISJOINT_ONE_BASED_SOLID_INDEX_SETS":
            require(pair["left_source_object"] == pair["right_source_object"],
                    f"GO_SAME_SOURCE_IDENTITY:{entry['component_id']}")
            left = pair["left_selected_solid_indices_one_based"]
            right = pair["right_selected_solid_indices_one_based"]
            require(isinstance(left, list) and isinstance(right, list),
                    f"GO_SAME_SOURCE_INDEX_LISTS:{entry['component_id']}")
            require(set(int(value) for value in left).isdisjoint(int(value) for value in right),
                    f"GO_SAME_SOURCE_INDEX_OVERLAP:{entry['component_id']}")
            require(pair["intersection_one_based"] == [],
                    f"GO_SAME_SOURCE_REPORTED_INTERSECTION:{entry['component_id']}")
        else:
            require(pair["identity_basis"] == "DISTINCT_SOURCE_OBJECT_IDENTITY_SOLID_INDICES_NOT_COMPARABLE",
                    f"GO_IDENTITY_BASIS:{entry['component_id']}:{pair['identity_basis']}")
            require(pair["left_source_object"] != pair["right_source_object"],
                    f"GO_DISTINCT_SOURCE_EVIDENCE:{entry['component_id']}")
            require(pair["left_selected_solid_indices_one_based"] is None
                    and pair["right_selected_solid_indices_one_based"] is None
                    and pair["intersection_one_based"] is None,
                    f"GO_DISTINCT_SOURCE_INDEX_FIELDS:{entry['component_id']}")
    require(reported_overlap["all_go_output_neutral_selection_sets_disjoint"] is True,
            "GO_SELECTION_AGGREGATE_DISJOINT")
    require(reported_overlap["all_go_output_neutral_selection_identity_pairs_disjoint"] is True,
            "GO_SELECTION_IDENTITY_AGGREGATE_DISJOINT")
    reported_events = reported_overlap["positive_overlap_events"]
    for finding in overlaps:
        matching = [
            event for event in reported_events
            if event["component_id"] == finding["component_id"]
            and {
                int(event["left_source_solid_index_one_based"]),
                int(event["right_source_solid_index_one_based"]),
            } == {
                int(finding["inner_solid_index_one_based"]),
                int(finding["outer_solid_index_one_based"]),
            }
        ]
        require(matching, f"AUDIT_OVERLAP_EVENT_MISSING:{finding['component_id']}")
        require(any(float(event["fraction_of_smaller_solid"]) >= 0.999 for event in matching),
                f"AUDIT_OVERLAP_NOT_CONTAINMENT:{finding['component_id']}")

    prohibited = audit["prohibited_outputs"]
    for key, value in prohibited.items():
        if isinstance(value, bool):
            require(value is False, f"PROHIBITED_OUTPUT_TRUE:{key}")
    unresolved = audit["unresolved_items"]
    require(unresolved, "FAIL_AUDIT_UNRESOLVED_EMPTY")


def static_builder_audit() -> None:
    source = (ROOT / BUILDER).read_text(encoding="utf-8")
    validator_tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
    imported_modules = set()
    for node in ast.walk(validator_tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add(node.module)
    require(not any("build_inertia_ledger_v15_16_v2" in name for name in imported_modules),
            "VALIDATOR_IMPORTS_BUILDER")
    tree = ast.parse(source)
    require(source.count(COM_V1_JSON) == 2 and source.count(COM_V1_MD) == 2,
            "BUILDER_COM_V1_OCCURRENCES_NOT_ONLY_PROTECTED_AND_SUPERSEDED")
    # V1 may be named in the protected-hash table or as a superseded audit
    # artifact.  It may not be opened as numerical input.  Follow direct
    # aliases into open/read calls; the generic protected-hash loop remains
    # intentionally permitted because it only reads raw bytes for mutation
    # detection.
    v1_aliases: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            value = node.value
            if isinstance(value, ast.Constant) and value.value in (COM_V1_JSON, COM_V1_MD):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                v1_aliases.update(target.id for target in targets if isinstance(target, ast.Name))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        method = node.func.attr if isinstance(node.func, ast.Attribute) else (
            node.func.id if isinstance(node.func, ast.Name) else ""
        )
        if method not in {"open", "read_text", "read_bytes"}:
            continue
        constants = {child.value for child in ast.walk(node) if isinstance(child, ast.Constant)}
        names = {child.id for child in ast.walk(node) if isinstance(child, ast.Name)}
        require(
            not ({COM_V1_JSON, COM_V1_MD} & constants) and not (v1_aliases & names),
            f"BUILDER_OPENS_SUPERSEDED_COM_V1:{getattr(node, 'lineno', 0)}",
        )
    require("Mesh.CenterOfGravity" not in source and ".CenterOfGravity" not in source,
            "BUILDER_MESH_CENTER_OF_GRAVITY_REFERENCE")
    for forbidden in ("<inertial", "gravity=", "armature=", "reflected rotor"):
        require(forbidden.lower() not in source.lower(), f"BUILDER_PROHIBITED_TEXT:{forbidden}")


def run_builder_check_with_snapshots(protected_before: dict[str, str]) -> None:
    outputs_before = output_snapshot()
    completed = subprocess.run(
        [sys.executable, str(ROOT / BUILDER), "--check"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    require(completed.returncode == 0, "BUILDER_CHECK_FAILED:\n" + completed.stdout)
    require(output_snapshot() == outputs_before, "BUILDER_CHECK_MUTATED_ALLOWED_OUTPUTS")
    require(protected_snapshot() == protected_before, "PROTECTED_CHANGED_DURING_VALIDATION")
    require(changed_paths() == ALLOWED_CHANGED_PATHS, "SCOPE_CHANGED_DURING_BUILDER_CHECK")


def main() -> int:
    document = None
    try:
        verify_git_scope()
        protected_before = protected_snapshot()
        static_builder_audit()
        run_math_test()
        independent_occt_semantics_gate()
        mass, com, components, mass_map = load_authorities()
        print_cache = audit_prints(components)
        document = App.openDocument(str(ROOT / SOURCE_CAD))
        require(document is not None, "SOURCE_CAD_OPEN")
        component_records, atoms_by_component = build_independent_geometry(
            document, components, mass_map, print_cache
        )
        component_path_b = independent_brep_path_b(component_records, atoms_by_component)
        for component_id, atoms in atoms_by_component.items():
            selected_key = component_path_b[component_id]["selected_key"]
            for atom in atoms:
                atom["path_b_selected"] = atom[selected_key]
        require(len(component_records) == EXPECTED_COMPONENT_COUNT, "INDEPENDENT_COMPONENT_COUNT")
        total_mass = math.fsum(record["mass_kg"] for record in component_records.values())
        require(abs(total_mass - EXPECTED_TOTAL_MASS) < 1.0e-12, f"TOTAL_MASS:{total_mass}")
        links, sensitivities = build_link_diagnostics(com, component_records, atoms_by_component)
        overlaps = duplicate_overlap_audit(atoms_by_component)
        validate_reports(print_cache, component_records, component_path_b, links, sensitivities, overlaps)
        require(protected_snapshot() == protected_before, "PROTECTED_CHANGED_AFTER_CAD_READ")
        App.closeDocument(document.Name)
        document = None
        run_builder_check_with_snapshots(protected_before)
        print("source commit / exact six-path scope = PASS")
        print("protected authorities before/after = PASS")
        print("analytic inertia math + SI unit audit = PASS")
        print("four independent raw-STL/OCCT suitability paths = PASS")
        print("17-component mass/membership/COM reproduction = PASS")
        print("six candidate OCCT link tensors/frame/physics diagnostics = PASS")
        print("residual sensitivity A/B model limitation and C low sensitivity = CONFIRMED")
        print(f"independent full-containment findings = {len(overlaps)} across five GO outputs")
        print("MASS_GEOMETRY_DUPLICATION blocker = CONFIRMED")
        print("authoritative PASS ledger absent = PASS")
        print("builder --check output/protected TOCTOU snapshots = PASS")
        print("TOTAL PASS: V15.16 V2 FAIL audit independently validated")
        return 0
    except (ValidationError, KeyError, ValueError, OSError, subprocess.SubprocessError) as error:
        print(f"VALIDATION FAIL: {error}", file=sys.stderr)
        return 1
    finally:
        if document is not None:
            try:
                App.closeDocument(document.Name)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
