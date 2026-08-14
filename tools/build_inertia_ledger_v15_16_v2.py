#!/usr/bin/env python3
"""Build the deterministic V15.16 V2 rigid-inertia *failure audit*.

This builder deliberately never writes an authoritative PASS inertia ledger.
It independently recomputes candidate rigid-body tensors from the frozen mass
and COM-V2 authorities, then fails closed when residual-model sensitivity or
mass-geometry overlap prevents a defensible freeze.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    import FreeCAD as App
    import Mesh
    import MeshPart
    import Part
    import numpy as np
except ImportError as exc:  # pragma: no cover - exercised by the runtime gate
    raise SystemExit(
        "Run with a FreeCAD Python interpreter that can import FreeCAD: " + str(exc)
    )


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "379b7174d276282d08295c7eec2592558a367ad9"
SOURCE_TREE = "8b098c540edf7a970b8f90cba9ededd9d1957cd2"
TARGET_BRANCH = "agent/v15-16-inertia-v2"

MASS_LEDGER = "V15_15_实测质量账本_v1.json"
COM_LEDGER = "V15_15_COM账本_v2.json"
SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
MEMBERSHIP = "rigid_links_v15_13/rigid_link_membership.csv"
MANIFEST = "rigid_links_v15_13/rigid_link_manifest.json"
MATH_TEST = "tools/test_inertia_math_v15_16_v2.py"

FAIL_JSON = "V15_16_刚体惯量_FAIL审计_v2.json"
FAIL_MD = "V15_16_刚体惯量_FAIL审计_v2.md"
SUITABILITY_JSON = "V15_16_打印件惯量几何适用性报告.json"

ALLOWED_CHANGED_PATHS = {
    FAIL_JSON,
    FAIL_MD,
    SUITABILITY_JSON,
    "tools/build_inertia_ledger_v15_16_v2.py",
    "tools/validate_inertia_ledger_v15_16_v2.py",
    MATH_TEST,
}

PROTECTED_HASHES = {
    MASS_LEDGER: "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    "V15_15_实测质量账本_v1.md": "fa52587a3309ea5665197283b62ac2e165e67663070408931611fa3b7b4e946d",
    COM_LEDGER: "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
    "V15_15_COM账本_v2.md": "64117a409bc37b70e436eef5fee18aa94b21239c3ccdf127adafc5f550c0e59f",
    # These two deprecated files are hashed as immutable audit evidence only.
    # Their bytes are never parsed and none of their numeric content is used.
    "V15_15_COM账本_v1.json": "13e3470821541d7ae5c3f10223c51339d7d46df0d118a8e4dbeac2480c462d32",
    "V15_15_COM账本_v1.md": "8875ff34ab6c091b2d5de6fff418553b0834a141adf32f4c88aac7d1d916d11a",
    SOURCE_CAD: "b4a02a97afaff88e246a46340dad4cacd3c645a7456fdd087a69552e14d3ddc0",
    MEMBERSHIP: "3d2a4d52d679eb97917410c6e60bec22c0c269f91ed31b35fd53ff94167e1b8c",
    MANIFEST: "8db76f9228f7a60bd017276239e3669de3cfd443c8cd36d0dd555e184606384f",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties.stl": "67aed62b304cfef3a338b0e59b035426b8e012619c333d032e8304c389352f45",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json": "a2b58c9892797b26c4511ff2581e7224d99cd27ca2261cc864252a6e38c21815",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties.stl": "bafb8dc8b06242a844196bc63971f95a0a7dd52834b2a05ab72f40b28460a081",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties_report.json": "dbe4c0751ab7bf4c1e564ce8b078f6de96336aaa203f060b402cc68fa6483e0f",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties.stl": "631217db4ad44f53313aa4691bde3eaaec9e043c9ff83cc6549ea42bd2832760",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties_report.json": "29961b01d574e37ba28c4d7d12cba46b8836911044d29184d1457d7b0160d94f",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties.stl": "585d32a6ac54aca69a082947f31604e5f7b44e5be14eacab2c2db4f45a10f944",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties_report.json": "561d497524b3ed21349caa69f0659f157a2486c50861abe407fad15f8e753614",
}

PRINT_SPECS = (
    {
        "member": "UpperArm_A_SleeveSide_PrintPart",
        "path": "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties.stl",
    },
    {
        "member": "UpperArm_B_Distal_PrintPart",
        "path": "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties.stl",
    },
    {
        "member": "Forearm_v3_HighDetail_Display",
        "path": "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties.stl",
    },
    {
        "member": "Wrist_Prelink_v1_HighDetail_Display",
        "path": "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties.stl",
    },
)

LINK_ORDER = ("link2", "link3", "link4", "link5", "link6", "gripper")
EXPECTED_LINK_MASSES = {
    "link2": 0.7615,
    "link3": 1.09,
    "link4": 0.675,
    "link5": 0.504,
    "link6": 0.125,
    "gripper": 0.296,
}
EXPECTED_TOTAL_MASS_KG = 3.4515
EXPECTED_COMPONENT_COUNT = 17

NORMAL_DEFLECTION_MM = 0.10
FINE_DEFLECTION_MM = 0.03
ULTRA_DEFLECTION_MM = 0.01
ANGULAR_DEFLECTION_RAD = 0.5
MESH_CONVERGENCE_LIMIT = 0.005
BREP_PATH_LIMIT = 0.01
PRINT_PATH_LIMIT = 1.0e-5
PRINT_VOLUME_LIMIT = 1.0e-8
PRINT_COM_LIMIT_M = 1.0e-7
COM_REPRO_LIMIT_M = 1.0e-8
FRAME_ERROR_LIMIT = 1.0e-10
OVERLAP_VOLUME_TOL_MM3 = 1.0e-7

MM_TO_M = 1.0e-3
MM2_TO_M2 = 1.0e-6
IDENTITY3 = np.eye(3, dtype=float)


class AuditFailure(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AuditFailure(message)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def canonical_json_bytes(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        + "\n"
    ).encode("utf-8")


def run_git(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(ROOT), "-c", "core.quotepath=false", *args],
        text=True,
        encoding="utf-8",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def changed_paths() -> set[str]:
    paths: set[str] = set()
    status = run_git("status", "--porcelain=v1", "--untracked-files=all").stdout
    for line in status.splitlines():
        if not line:
            continue
        token = line[3:]
        if " -> " in token:
            token = token.split(" -> ", 1)[1]
        paths.add(token.replace("\\", "/"))
    diff = run_git("diff", "--name-only", SOURCE_COMMIT).stdout
    paths.update(line.replace("\\", "/") for line in diff.splitlines() if line)
    return paths


def git_scope_gate() -> dict[str, Any]:
    branch = run_git("branch", "--show-current").stdout.strip()
    require(branch == TARGET_BRANCH, f"WRONG_BRANCH:{branch}")
    tree = run_git("rev-parse", f"{SOURCE_COMMIT}^{{tree}}").stdout.strip()
    require(tree == SOURCE_TREE, f"SOURCE_TREE_MISMATCH:{tree}")
    ancestor = run_git(
        "merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD", check=False
    ).returncode == 0
    require(ancestor, "SOURCE_COMMIT_NOT_ANCESTOR")
    unexpected = sorted(changed_paths() - ALLOWED_CHANGED_PATHS)
    require(not unexpected, "UNEXPECTED_CHANGED_PATHS:" + repr(unexpected))
    return {
        "target_branch": TARGET_BRANCH,
        "source_commit": SOURCE_COMMIT,
        "source_tree": SOURCE_TREE,
        "source_commit_is_ancestor": True,
        "policy": "CHANGED_PATHS_MUST_BE_SUBSET_OF_FAIL_AUDIT_SIX_PATH_ALLOWLIST",
        "allowed_changed_paths": sorted(ALLOWED_CHANGED_PATHS),
        "unexpected_changed_paths": [],
        "pass": True,
    }


def protected_hash_snapshot() -> list[dict[str, Any]]:
    result = []
    for relative, expected in PROTECTED_HASHES.items():
        path = ROOT / relative
        require(path.is_file(), f"PROTECTED_INPUT_MISSING:{relative}")
        actual = sha256_file(path)
        require(actual == expected, f"PROTECTED_HASH_MISMATCH:{relative}:{actual}")
        result.append(
            {
                "path": relative,
                "sha256": actual,
                "expected_sha256": expected,
                "locked": True,
            }
        )
    return result


def run_math_test() -> dict[str, Any]:
    path = ROOT / MATH_TEST
    require(path.is_file(), "MATH_TEST_MISSING")
    completed = subprocess.run(
        [sys.executable, str(path)],
        cwd=str(ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    require(completed.returncode == 0, "MATH_TEST_FAILED:\n" + completed.stdout)
    require("TOTAL PASS" in completed.stdout, "MATH_TEST_NO_TOTAL_PASS")
    return {
        "path": MATH_TEST,
        "sha256": sha256_file(path),
        "executed_before_robot_cad_read": True,
        "exit_code": completed.returncode,
        "required_cases": [
            "AXIS_ALIGNED_BOX",
            "ROTATED_BOX_RX13_RY_MINUS21_RZ37",
            "PARALLEL_AXIS",
            "ROTATION_AND_TRANSLATION",
            "NONUNIFORM_TRIANGULATION",
            "MM_TO_SI",
        ],
        "status": "PASS",
        "pass": True,
    }


def v3(value: Any) -> np.ndarray:
    if hasattr(value, "x"):
        return np.array([float(value.x), float(value.y), float(value.z)], dtype=float)
    return np.asarray(value, dtype=float).reshape(3)


def rows_to_matrix(rows: Sequence[Sequence[float]]) -> App.Matrix:
    matrix = App.Matrix()
    for row in range(4):
        for column in range(4):
            setattr(matrix, f"A{row + 1}{column + 1}", float(rows[row][column]))
    return matrix


def placement_parts(rows: Sequence[Sequence[float]]) -> tuple[np.ndarray, np.ndarray]:
    matrix = np.asarray(rows, dtype=float)
    return matrix[:3, :3], matrix[:3, 3]


def matrix_of_inertia(shape: Any) -> np.ndarray:
    values = np.asarray(shape.MatrixOfInertia.A, dtype=float).reshape(4, 4)
    return values[:3, :3].copy()


def symmetrize(matrix: np.ndarray) -> np.ndarray:
    return 0.5 * (matrix + matrix.T)


def frobenius(matrix: np.ndarray) -> float:
    return float(np.linalg.norm(matrix, ord="fro"))


def relative_frobenius(left: np.ndarray, right: np.ndarray) -> float:
    denominator = frobenius(right)
    require(denominator > 0.0, "ZERO_FROBENIUS_DENOMINATOR")
    return frobenius(left - right) / denominator


def parallel_axis(inertia_com: np.ndarray, mass_kg: float, displacement_m: np.ndarray) -> np.ndarray:
    displacement_m = v3(displacement_m)
    return inertia_com + mass_kg * (
        float(displacement_m @ displacement_m) * IDENTITY3
        - np.outer(displacement_m, displacement_m)
    )


def bbox_dict(box: Any) -> dict[str, list[float]]:
    return {
        "min_mm": [float(box.XMin), float(box.YMin), float(box.ZMin)],
        "max_mm": [float(box.XMax), float(box.YMax), float(box.ZMax)],
        "size_mm": [float(box.XLength), float(box.YLength), float(box.ZLength)],
    }


def bbox_corners(box: Any) -> list[np.ndarray]:
    return [
        np.array(point, dtype=float)
        for point in itertools.product(
            (float(box.XMin), float(box.XMax)),
            (float(box.YMin), float(box.YMax)),
            (float(box.ZMin), float(box.ZMax)),
        )
    ]


def bbox_intersects(left: Any, right: Any, tolerance: float = 1.0e-9) -> bool:
    return not (
        float(left.XMax) < float(right.XMin) - tolerance
        or float(right.XMax) < float(left.XMin) - tolerance
        or float(left.YMax) < float(right.YMin) - tolerance
        or float(right.YMax) < float(left.YMin) - tolerance
        or float(left.ZMax) < float(right.ZMin) - tolerance
        or float(right.ZMax) < float(left.ZMin) - tolerance
    )


def finite_array(value: np.ndarray) -> bool:
    return bool(np.all(np.isfinite(value)))


def canonical_principal_axes(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    values, vectors = np.linalg.eigh(symmetrize(matrix))
    for column in range(3):
        vector = vectors[:, column]
        pivot = int(np.argmax(np.abs(vector)))
        if vector[pivot] < 0.0:
            vectors[:, column] *= -1.0
    if np.linalg.det(vectors) < 0.0:
        vectors[:, -1] *= -1.0
    return values, vectors


def tensor_metrics(matrix: np.ndarray, mass_kg: float, r_max_m: float) -> dict[str, Any]:
    matrix = np.asarray(matrix, dtype=float)
    symmetric_error = float(np.max(np.abs(matrix - matrix.T)))
    principal, axes = canonical_principal_axes(matrix)
    determinant = float(np.linalg.det(matrix))
    triangle_margins = [
        float(principal[0] + principal[1] - principal[2]),
        float(principal[0] + principal[2] - principal[1]),
        float(principal[1] + principal[2] - principal[0]),
    ]
    radius = np.sqrt(np.maximum(principal, 0.0) / mass_kg)
    finite = finite_array(matrix) and finite_array(principal)
    symmetry_pass = symmetric_error < 1.0e-12
    positive_pass = bool(np.all(principal > 1.0e-12) and determinant > 0.0)
    triangle_pass = min(triangle_margins) >= -1.0e-12
    radius_pass = bool(np.all(radius > 0.0) and np.all(radius <= r_max_m + 1.0e-12))
    return {
        "matrix_kg_m2": matrix.tolist(),
        "ixx": float(matrix[0, 0]),
        "iyy": float(matrix[1, 1]),
        "izz": float(matrix[2, 2]),
        "ixy": float(matrix[0, 1]),
        "ixz": float(matrix[0, 2]),
        "iyz": float(matrix[1, 2]),
        "trace_kg_m2": float(np.trace(matrix)),
        "determinant_kg3_m6": determinant,
        "principal_moments_kg_m2": principal.tolist(),
        "principal_axes_in_expression_frame_columns": axes.tolist(),
        "principal_triangle_margins_kg_m2": triangle_margins,
        "radius_of_gyration_m": radius.tolist(),
        "r_max_m": r_max_m,
        "finite_pass": finite,
        "symmetry_max_abs_error_kg_m2": symmetric_error,
        "symmetry_pass": symmetry_pass,
        "positive_definite_pass": positive_pass,
        "principal_triangle_inequality_pass": triangle_pass,
        "radius_of_gyration_sanity_pass": radius_pass,
        "pass": finite and symmetry_pass and positive_pass and triangle_pass and radius_pass,
    }


def edge_topology_audit(
    vertices: Sequence[np.ndarray], faces: Sequence[Sequence[int]]
) -> dict[str, Any]:
    edge_directions: dict[tuple[int, int], list[int]] = {}
    degenerate_count = 0
    for face_index, face in enumerate(faces):
        require(len(face) == 3, f"NON_TRIANGLE_FACE:{face_index}")
        indices = tuple(int(index) for index in face)
        if len(set(indices)) != 3:
            degenerate_count += 1
            continue
        p0, p1, p2 = (vertices[index] for index in indices)
        double_area = float(np.linalg.norm(np.cross(p1 - p0, p2 - p0)))
        if not math.isfinite(double_area) or double_area <= 0.0:
            degenerate_count += 1
            continue
        for start, end in (
            (indices[0], indices[1]),
            (indices[1], indices[2]),
            (indices[2], indices[0]),
        ):
            key = (min(start, end), max(start, end))
            direction = 1 if (start, end) == key else -1
            edge_directions.setdefault(key, []).append(direction)
    incidence_histogram: dict[str, int] = {}
    open_or_nonmanifold = 0
    orientation_conflicts = 0
    for directions in edge_directions.values():
        key = str(len(directions))
        incidence_histogram[key] = incidence_histogram.get(key, 0) + 1
        if len(directions) != 2:
            open_or_nonmanifold += 1
        elif sum(directions) != 0:
            orientation_conflicts += 1
    return {
        "vertex_count": len(vertices),
        "facet_count": len(faces),
        "edge_count": len(edge_directions),
        "edge_incidence_histogram": incidence_histogram,
        "degenerate_facet_count": degenerate_count,
        "open_or_nonmanifold_edge_count": open_or_nonmanifold,
        "orientation_conflict_edge_count": orientation_conflicts,
        "closed_two_manifold": open_or_nonmanifold == 0 and degenerate_count == 0,
        "consistent_orientation": orientation_conflicts == 0 and degenerate_count == 0,
        "pass": (
            open_or_nonmanifold == 0
            and orientation_conflicts == 0
            and degenerate_count == 0
        ),
    }


def anchored_polyhedron_properties(
    vertices_input: Sequence[Any],
    faces_input: Sequence[Sequence[int]],
    mass_kg: float,
    *,
    require_topology: bool,
) -> dict[str, Any]:
    """Uniform-volume properties via anchored signed tetrahedra.

    No FreeCAD Mesh volume or vertex-mean API participates.  Coordinates are
    in mm; the raw second moments and raw inertia are in mm^5.  Scaling by
    ``mass / volume * 1e-6`` yields kg*m^2.
    """
    vertices = [v3(point) for point in vertices_input]
    faces = [tuple(int(index) for index in face) for face in faces_input]
    require(vertices and faces, "EMPTY_POLYHEDRON")
    require(mass_kg > 0.0 and math.isfinite(mass_kg), "INVALID_POLY_MASS")
    require(all(finite_array(point) for point in vertices), "NONFINITE_VERTEX")
    topology = edge_topology_audit(vertices, faces)
    if require_topology:
        require(topology["pass"], "POLYHEDRON_TOPOLOGY_FAIL:" + repr(topology))

    stacked = np.vstack(vertices)
    anchor = 0.5 * (np.min(stacked, axis=0) + np.max(stacked, axis=0))
    volume_terms: list[float] = []
    first_terms: list[list[float]] = [[], [], []]
    second_terms: list[list[list[float]]] = [
        [[], [], []],
        [[], [], []],
        [[], [], []],
    ]
    for face_index, (i0, i1, i2) in enumerate(faces):
        require(
            0 <= i0 < len(vertices)
            and 0 <= i1 < len(vertices)
            and 0 <= i2 < len(vertices),
            f"FACE_INDEX_OUT_OF_RANGE:{face_index}",
        )
        a = vertices[i0] - anchor
        b = vertices[i1] - anchor
        c = vertices[i2] - anchor
        signed_volume = float(a @ np.cross(b, c)) / 6.0
        volume_terms.append(signed_volume)
        coordinate_sum = a + b + c
        for row in range(3):
            first_terms[row].append(signed_volume * float(coordinate_sum[row]) / 4.0)
            for column in range(3):
                vertex_dot = (
                    float(a[row] * a[column])
                    + float(b[row] * b[column])
                    + float(c[row] * c[column])
                )
                second_terms[row][column].append(
                    signed_volume
                    * (
                        float(coordinate_sum[row] * coordinate_sum[column])
                        + vertex_dot
                    )
                    / 20.0
                )

    signed_volume = math.fsum(volume_terms)
    require(math.isfinite(signed_volume) and signed_volume != 0.0, "ZERO_SIGNED_VOLUME")
    orientation = 1.0 if signed_volume > 0.0 else -1.0
    volume = abs(signed_volume)
    first_anchor = np.array(
        [orientation * math.fsum(terms) for terms in first_terms], dtype=float
    )
    second_anchor = np.array(
        [
            [orientation * math.fsum(second_terms[row][column]) for column in range(3)]
            for row in range(3)
        ],
        dtype=float,
    )
    com_offset = first_anchor / volume
    com = anchor + com_offset
    central_second = second_anchor - volume * np.outer(com_offset, com_offset)
    raw_inertia_com = float(np.trace(central_second)) * IDENTITY3 - central_second
    density = mass_kg / volume
    inertia_com = symmetrize(raw_inertia_com * density * MM2_TO_M2)

    first_global = first_anchor + volume * anchor
    second_global = (
        second_anchor
        + np.outer(anchor, first_anchor)
        + np.outer(first_anchor, anchor)
        + volume * np.outer(anchor, anchor)
    )
    require(
        finite_array(com)
        and finite_array(second_global)
        and finite_array(raw_inertia_com)
        and finite_array(inertia_com),
        "NONFINITE_POLY_PROPERTIES",
    )
    return {
        "anchor_mm": anchor,
        "signed_volume_mm3": signed_volume,
        "volume_mm3": volume,
        "first_moment_mm4": first_global,
        "second_moment_mm5": second_global,
        "central_second_moment_mm5": central_second,
        "raw_inertia_com_mm5": raw_inertia_com,
        "density_kg_per_mm3": density,
        "com_mm": com,
        "inertia_com_kg_m2": inertia_com,
        "topology": topology,
    }


def mesh_topology(mesh: Any) -> tuple[list[np.ndarray], list[tuple[int, int, int]]]:
    points, facets = mesh.Topology
    vertices = [v3(point) for point in points]
    faces = [tuple(int(index) for index in face) for face in facets]
    return vertices, faces


def part_solid_from_triangles(
    vertices: Sequence[np.ndarray], faces: Sequence[Sequence[int]]
) -> Any:
    part_faces = []
    for face_index, face in enumerate(faces):
        points = [App.Vector(*vertices[int(index)].tolist()) for index in face]
        try:
            wire = Part.makePolygon([points[0], points[1], points[2], points[0]])
            part_faces.append(Part.Face(wire))
        except Exception as exc:
            raise AuditFailure(f"PART_TRIANGLE_FACE_FAIL:{face_index}:{exc}") from exc
    shell = Part.makeShell(part_faces)
    require(shell.isClosed(), "STL_PART_SHELL_OPEN")
    require(shell.isValid(), "STL_PART_SHELL_INVALID")
    solid = Part.makeSolid(shell)
    require(not solid.isNull(), "STL_PART_SOLID_NULL")
    if float(solid.Volume) < 0.0:
        solid.reverse()
    require(float(solid.Volume) > 0.0, "STL_PART_SOLID_NONPOSITIVE")
    require(solid.isClosed(), "STL_PART_SOLID_OPEN")
    require(solid.isValid(), "STL_PART_SOLID_INVALID")
    return solid


def occt_properties(shape_input: Any, mass_kg: float) -> dict[str, Any]:
    shape = shape_input.copy()
    signed_volume_before = float(shape.Volume)
    reversed_to_positive = False
    if signed_volume_before < 0.0:
        shape.reverse()
        reversed_to_positive = True
    volume = float(shape.Volume)
    require(volume > 0.0 and math.isfinite(volume), "OCCT_NONPOSITIVE_VOLUME")
    require(shape.isClosed() and shape.isValid(), "OCCT_INVALID_OR_OPEN_SOLID")
    com = v3(shape.CenterOfMass)
    raw = matrix_of_inertia(shape)
    inertia = symmetrize(raw * (mass_kg / volume) * MM2_TO_M2)
    require(finite_array(com) and finite_array(raw) and finite_array(inertia), "OCCT_NONFINITE")
    return {
        "shape": shape,
        "signed_volume_before_normalization_mm3": signed_volume_before,
        "orientation_reversed_to_positive": reversed_to_positive,
        "volume_mm3": volume,
        "com_mm": com,
        "raw_inertia_com_mm5": raw,
        "inertia_com_kg_m2": inertia,
    }


def occt_semantics_gate() -> dict[str, Any]:
    dimensions_mm = np.array([820.0, 470.0, 310.0], dtype=float)
    mass_kg = 7.3
    shape = Part.makeBox(
        float(dimensions_mm[0]),
        float(dimensions_mm[1]),
        float(dimensions_mm[2]),
        App.Vector(*(-0.5 * dimensions_mm).tolist()),
    )
    properties = occt_properties(shape, mass_kg)
    dims_m = dimensions_mm * MM_TO_M
    expected = np.diag(
        [
            mass_kg * (dims_m[1] ** 2 + dims_m[2] ** 2) / 12.0,
            mass_kg * (dims_m[0] ** 2 + dims_m[2] ** 2) / 12.0,
            mass_kg * (dims_m[0] ** 2 + dims_m[1] ** 2) / 12.0,
        ]
    )
    base_error = relative_frobenius(properties["inertia_com_kg_m2"], expected)

    rx, ry, rz = (math.radians(value) for value in (13.0, -21.0, 37.0))
    rotation_x = np.array(
        [[1.0, 0.0, 0.0], [0.0, math.cos(rx), -math.sin(rx)], [0.0, math.sin(rx), math.cos(rx)]]
    )
    rotation_y = np.array(
        [[math.cos(ry), 0.0, math.sin(ry)], [0.0, 1.0, 0.0], [-math.sin(ry), 0.0, math.cos(ry)]]
    )
    rotation_z = np.array(
        [[math.cos(rz), -math.sin(rz), 0.0], [math.sin(rz), math.cos(rz), 0.0], [0.0, 0.0, 1.0]]
    )
    rotation = rotation_z @ rotation_y @ rotation_x
    rows = np.eye(4)
    rows[:3, :3] = rotation
    rows[:3, 3] = [110.0, -70.0, 50.0]
    transformed = shape.copy()
    transformed.Placement = App.Placement(rows_to_matrix(rows.tolist()))
    transformed_properties = occt_properties(transformed, mass_kg)
    expected_rotated = rotation @ expected @ rotation.T
    rotated_error = relative_frobenius(
        transformed_properties["inertia_com_kg_m2"], expected_rotated
    )
    expected_com = rows[:3, 3]
    com_error_mm = float(np.linalg.norm(transformed_properties["com_mm"] - expected_com))
    require(base_error < 1.0e-10, f"OCCT_BOX_BASE_SEMANTICS:{base_error}")
    require(rotated_error < 1.0e-10, f"OCCT_BOX_ROTATED_SEMANTICS:{rotated_error}")
    require(com_error_mm < 1.0e-10, f"OCCT_BOX_TRANSLATION_SEMANTICS:{com_error_mm}")
    return {
        "api": "Part.Shape.MatrixOfInertia",
        "reference_point": "SHAPE_CENTER_OF_MASS",
        "raw_units": "mm^5_at_unit_density",
        "matrix_off_diagonal_convention": "NEGATIVE_PRODUCT_STANDARD_INERTIA_TENSOR",
        "negative_oriented_solid_policy": "COPY_AND_REVERSE_BEFORE_PHYSICAL_PROPERTIES",
        "si_scale_formula": "MatrixOfInertia_mm5*(mass_kg/volume_mm3)*1e-6",
        "axis_aligned_box_relative_frobenius_error": base_error,
        "rotated_translated_box_relative_frobenius_error": rotated_error,
        "translation_com_error_mm": com_error_mm,
        "nonzero_rotated_off_diagonal_observed": bool(
            max(abs(expected_rotated[0, 1]), abs(expected_rotated[0, 2]), abs(expected_rotated[1, 2]))
            > 0.0
        ),
        "pass": True,
    }


def find_artifact_member(
    components: Sequence[dict[str, Any]], artifact_path: str
) -> tuple[dict[str, Any], dict[str, Any]]:
    for component in components:
        for member in component.get("members", []):
            if member.get("artifact_path") == artifact_path:
                return component, member
    raise AuditFailure(f"ARTIFACT_NOT_IN_COM_V2:{artifact_path}")


def audit_print_artifacts(
    components: Sequence[dict[str, Any]],
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    entries: list[dict[str, Any]] = []
    cache: dict[str, dict[str, Any]] = {}
    for spec in PRINT_SPECS:
        relative = spec["path"]
        component, member = find_artifact_member(components, relative)
        path = ROOT / relative
        actual_hash = sha256_file(path)
        require(actual_hash == PROTECTED_HASHES[relative], f"PRINT_HASH:{relative}")
        mesh = Mesh.Mesh(str(path))
        vertices, faces = mesh_topology(mesh)
        topology = edge_topology_audit(vertices, faces)
        self_intersections = tuple(mesh.getSelfIntersections())
        mesh_qa = {
            "point_count": int(mesh.CountPoints),
            "facet_count": int(mesh.CountFacets),
            "closed": bool(mesh.isSolid()),
            "manifold": not bool(mesh.hasNonManifolds()),
            "self_intersection_count": len(self_intersections),
            "self_intersection_free": len(self_intersections) == 0,
            "consistent_orientation": topology["consistent_orientation"],
            "topology": topology,
        }
        require(mesh_qa["closed"], f"PRINT_OPEN:{relative}")
        require(mesh_qa["manifold"], f"PRINT_NONMANIFOLD:{relative}")
        require(mesh_qa["self_intersection_free"], f"PRINT_SELF_INTERSECTION:{relative}")
        require(mesh_qa["consistent_orientation"], f"PRINT_ORIENTATION:{relative}")

        poly = anchored_polyhedron_properties(
            vertices, faces, 1.0, require_topology=True
        )
        require(poly["signed_volume_mm3"] > 0.0, f"PRINT_NEGATIVE_WINDING:{relative}")
        solid = part_solid_from_triangles(vertices, faces)
        part_properties = occt_properties(solid, 1.0)
        volume_relative_error = abs(
            part_properties["volume_mm3"] - poly["volume_mm3"]
        ) / poly["volume_mm3"]
        com_error_m = float(
            np.linalg.norm(part_properties["com_mm"] - poly["com_mm"]) * MM_TO_M
        )
        inertia_relative_error = relative_frobenius(
            part_properties["inertia_com_kg_m2"], poly["inertia_com_kg_m2"]
        )
        poly_eigenvalues = np.linalg.eigvalsh(poly["inertia_com_kg_m2"])
        poly_inertia_valid = bool(
            finite_array(poly["inertia_com_kg_m2"])
            and np.max(np.abs(poly["inertia_com_kg_m2"] - poly["inertia_com_kg_m2"].T))
            < 1.0e-12
            and np.all(poly_eigenvalues > 0.0)
        )
        path_pass = (
            volume_relative_error < PRINT_VOLUME_LIMIT
            and com_error_m < PRINT_COM_LIMIT_M
            and inertia_relative_error < PRINT_PATH_LIMIT
        )
        require(poly_inertia_valid, f"PRINT_POLY_INERTIA_INVALID:{relative}")
        require(path_pass, f"PRINT_DUAL_PATH_FAIL:{relative}")

        rows = member["source_placement_matrix_row_major"]
        rotation, translation = placement_parts(rows)
        com_world = rotation @ poly["com_mm"] + translation
        if "center_world_mm" in member:
            expected_world = v3(member["center_world_mm"])
        elif component["component_id"] == "UPPER_ARM_PRINT_MEASURED":
            allocation_part = next(
                part
                for part in component["equal_density_volume_allocation"]["parts"]
                if part["member"] == member["member"]
            )
            expected_world = v3(allocation_part["center_world_mm"])
        else:
            require(len(component["members"]) == 1, f"PRINT_EXPECTED_COM_AMBIGUOUS:{relative}")
            expected_world = v3(component["com_world_mm"])
        authority_com_error_m = float(np.linalg.norm(com_world - expected_world) * MM_TO_M)
        require(authority_com_error_m < COM_REPRO_LIMIT_M, f"PRINT_PLACEMENT_COM:{relative}")

        world_shape = solid.copy()
        world_shape.Placement = App.Placement(rows_to_matrix(rows))
        entry = {
            "member": spec["member"],
            "component_id": component["component_id"],
            "artifact_path": relative,
            "sha256": actual_hash,
            "coordinate_frame": "SOURCE_OBJECT_LOCAL",
            "source_placement_matrix_row_major": rows,
            "mesh_qa": mesh_qa,
            "positive_volume": poly["volume_mm3"] > 0.0,
            "finite_volume_centroid": finite_array(poly["com_mm"]),
            "path_a_occt_reimport": {
                "closed": bool(solid.isClosed()),
                "valid": bool(solid.isValid()),
                "volume_mm3": part_properties["volume_mm3"],
                "centroid_source_local_mm": part_properties["com_mm"].tolist(),
                "raw_matrix_of_inertia_mm5": part_properties["raw_inertia_com_mm5"].tolist(),
                "unit_mass_inertia_com_kg_m2": part_properties["inertia_com_kg_m2"].tolist(),
            },
            "path_b_anchored_signed_tetrahedra": {
                "anchor_source_local_mm": poly["anchor_mm"].tolist(),
                "signed_volume_mm3": poly["signed_volume_mm3"],
                "physical_volume_mm3": poly["volume_mm3"],
                "first_moment_mm4": poly["first_moment_mm4"].tolist(),
                "second_moment_mm5": poly["second_moment_mm5"].tolist(),
                "raw_inertia_com_mm5": poly["raw_inertia_com_mm5"].tolist(),
                "centroid_source_local_mm": poly["com_mm"].tolist(),
                "unit_mass_inertia_com_kg_m2": poly["inertia_com_kg_m2"].tolist(),
                "polyhedral_inertia_valid": poly_inertia_valid,
                "mesh_center_of_gravity_used": False,
                "mesh_volume_used": False,
            },
            "path_a_b_agreement": {
                "volume_relative_error": volume_relative_error,
                "volume_limit_strict_less_than": PRINT_VOLUME_LIMIT,
                "centroid_distance_m": com_error_m,
                "centroid_limit_m_strict_less_than": PRINT_COM_LIMIT_M,
                "inertia_relative_frobenius_error": inertia_relative_error,
                "inertia_limit_strict_less_than": PRINT_PATH_LIMIT,
                "pass": path_pass,
            },
            "world_centroid_from_placement_mm": com_world.tolist(),
            "com_v2_member_centroid_world_mm": expected_world.tolist(),
            "com_v2_reproduction_error_m": authority_com_error_m,
            "original_v15_15_report_modified": False,
            "upgraded_usage": "V15_16_INERTIA_SUITABLE_MASS_GEOMETRY",
            "status": "PASS",
            "pass": True,
        }
        entries.append(entry)
        cache[relative] = {
            "entry": entry,
            "member": member,
            "component": component,
            "solid_local": solid,
            "solid_world": world_shape,
            "poly": poly,
            "part": part_properties,
            "rotation": rotation,
            "translation": translation,
        }

    report = {
        "schema": "go-m8010-arm-v15.16-print-inertia-suitability-v2/1.0",
        "revision": "V15.16-PRINT_INERTIA_GEOMETRY_SUITABILITY_V2",
        "scope": "FOUR_FROZEN_CANONICAL_PRINT_STL_ARTIFACTS_NO_REPAIR_NO_DEPLOYMENT",
        "algorithm": {
            "path_a": "STL_TO_CLOSED_VALID_OCCT_SOLID_MATRIX_OF_INERTIA",
            "path_b": "BBOX_CENTER_ANCHORED_SIGNED_TETRAHEDRA_FULL_SECOND_MOMENTS",
            "mesh_center_of_gravity_role": "PROHIBITED_NOT_READ",
            "mesh_volume_role": "PROHIBITED_NOT_READ",
            "original_reports_modified": False,
        },
        "artifacts": entries,
        "all_four_pass": all(entry["pass"] for entry in entries),
        "final_status": "V15.16 PRINT_INERTIA_GEOMETRY_SUITABILITY = PASS",
    }
    return report, cache


def load_authorities() -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    mass = json.loads((ROOT / MASS_LEDGER).read_text(encoding="utf-8"))
    com = json.loads((ROOT / COM_LEDGER).read_text(encoding="utf-8"))
    require(mass.get("final_status") == "V15.15 MASS_LEDGER_V1 = PASS", "MASS_AUTHORITY_NOT_PASS")
    require(com.get("final_status") == "V15.15 COM_LEDGER_V2 = PASS", "COM_V2_AUTHORITY_NOT_PASS")
    require(mass.get("unresolved_items") == [], "MASS_AUTHORITY_UNRESOLVED")
    require(com.get("unresolved_items") == [], "COM_V2_AUTHORITY_UNRESOLVED")
    components = list(com["components"])
    component_order = list(com["component_order"])
    require(len(components) == EXPECTED_COMPONENT_COUNT, "COM_V2_COMPONENT_COUNT")
    require([component["component_id"] for component in components] == component_order, "COM_V2_COMPONENT_ORDER")

    additive = mass["component_to_link_mapping"]["additive_components"]
    mass_map = {entry["component_id"]: entry for entry in additive}
    require(len(mass_map) == EXPECTED_COMPONENT_COUNT, "MASS_COMPONENT_COUNT")
    require(set(mass_map) == set(component_order), "MASS_COM_COMPONENT_SET_MISMATCH")
    component_mass_sum = 0.0
    for component in components:
        component_id = component["component_id"]
        mass_kg = float(mass_map[component_id]["nominal_mass_kg"])
        require(
            abs(mass_kg - float(component["frozen_mass_kg"])) < 1.0e-15,
            f"COMPONENT_MASS_CROSSCHECK:{component_id}",
        )
        require(
            mass_map[component_id]["ledger_link"] == component["owner_link"],
            f"COMPONENT_OWNER_CROSSCHECK:{component_id}",
        )
        component["_authority_mass_kg"] = mass_kg
        component_mass_sum += mass_kg

    for link_name in LINK_ORDER:
        mass_value = float(mass["link_mass_ledger"][link_name]["nominal_mass_kg"])
        require(abs(mass_value - EXPECTED_LINK_MASSES[link_name]) < 1.0e-15, f"LINK_MASS:{link_name}")
        require(abs(float(com["links"][link_name]["mass_kg"]) - mass_value) < 1.0e-15, f"COM_LINK_MASS:{link_name}")
        ids = [
            component["component_id"]
            for component in components
            if component["owner_link"] == link_name
        ]
        require(ids == list(com["links"][link_name]["component_ids"]), f"LINK_COMPONENT_MEMBERSHIP:{link_name}")

    total = float(mass["total_link2_to_gripper_nominal_mass_kg"])
    require(abs(total - EXPECTED_TOTAL_MASS_KG) < 1.0e-15, "MASS_TOTAL_AUTHORITY")
    require(abs(component_mass_sum - EXPECTED_TOTAL_MASS_KG) < 1.0e-12, "COMPONENT_MASS_TOTAL")
    return mass, com, components


def normalized_solid(shape_input: Any) -> tuple[Any, float, bool]:
    shape = shape_input.copy()
    signed = float(shape.Volume)
    reversed_to_positive = False
    if signed < 0.0:
        shape.reverse()
        reversed_to_positive = True
    require(float(shape.Volume) > 0.0, "SOLID_NONPOSITIVE_AFTER_NORMALIZATION")
    require(shape.isValid() and shape.isClosed(), "SOLID_INVALID_OR_OPEN")
    return shape, signed, reversed_to_positive


def tessellated_shape_properties(
    shape_input: Any, mass_kg: float, linear_deflection_mm: float
) -> dict[str, Any]:
    shape = shape_input.cleaned()
    mesh = MeshPart.meshFromShape(
        Shape=shape,
        LinearDeflection=float(linear_deflection_mm),
        AngularDeflection=float(ANGULAR_DEFLECTION_RAD),
        Relative=False,
    )
    tessellation_before_cleanup = {
        "point_count": int(mesh.CountPoints),
        "triangle_count": int(mesh.CountFacets),
    }
    # OCCT may emit exactly zero-area seam triangles even though the source
    # solid is valid and the Mesh kernel classifies the result as solid.  This
    # deterministic cleanup is confined to the derived PATH-B tessellation;
    # it neither repairs nor writes any source geometry.
    mesh.harmonizeNormals()
    mesh.fixDegenerations()
    require(mesh.CountFacets > 0, "BREP_TESSELLATION_EMPTY")
    require(mesh.isSolid(), "BREP_TESSELLATION_OPEN")
    require(not mesh.hasNonManifolds(), "BREP_TESSELLATION_NONMANIFOLD")
    vertices, faces = mesh_topology(mesh)
    poly = anchored_polyhedron_properties(vertices, faces, mass_kg, require_topology=True)
    return {
        "linear_deflection_mm": float(linear_deflection_mm),
        "angular_deflection_rad": ANGULAR_DEFLECTION_RAD,
        "relative": False,
        "derived_tessellation_cleanup": {
            "method": "HARMONIZE_NORMALS_THEN_FIX_DEGENERATIONS",
            "source_geometry_modified": False,
            "before": tessellation_before_cleanup,
            "after": {
                "point_count": int(mesh.CountPoints),
                "triangle_count": int(mesh.CountFacets),
            },
        },
        "point_count": int(mesh.CountPoints),
        "triangle_count": int(mesh.CountFacets),
        "volume_mm3": poly["volume_mm3"],
        "signed_volume_mm3": poly["signed_volume_mm3"],
        "com_world_mm": poly["com_mm"],
        "raw_inertia_com_mm5": poly["raw_inertia_com_mm5"],
        "inertia_com_world_kg_m2": poly["inertia_com_kg_m2"],
        "topology": poly["topology"],
    }


def aggregate_atoms_about_reference(
    atoms: Sequence[dict[str, Any]], path_key: str, reference_world_mm: np.ndarray
) -> dict[str, Any]:
    total_mass = math.fsum(float(atom["mass_kg"]) for atom in atoms)
    total_volume = math.fsum(float(atom[path_key]["volume_mm3"]) for atom in atoms)
    require(total_mass > 0.0 and total_volume > 0.0, "AGGREGATE_NONPOSITIVE")
    com = np.array(
        [
            math.fsum(
                float(atom["mass_kg"]) * float(atom[path_key]["com_world_mm"][axis])
                for atom in atoms
            )
            / total_mass
            for axis in range(3)
        ],
        dtype=float,
    )
    tensor = np.zeros((3, 3), dtype=float)
    for atom in atoms:
        displacement_m = (
            v3(atom[path_key]["com_world_mm"]) - reference_world_mm
        ) * MM_TO_M
        tensor += parallel_axis(
            np.asarray(atom[path_key]["inertia_com_world_kg_m2"], dtype=float),
            float(atom["mass_kg"]),
            displacement_m,
        )
    return {
        "mass_kg": total_mass,
        "volume_mm3": total_volume,
        "com_world_mm": com,
        "inertia_about_reference_world_kg_m2": symmetrize(tensor),
    }


def component_source_status(component_id: str) -> tuple[str, dict[str, Any]]:
    if component_id.startswith("RESIDUAL_"):
        return (
            "ENGINEERING_ESTIMATE_RESIDUAL_PROXY_INERTIA",
            {
                "uniform_proxy_geometry_not_measured_mass_distribution": True,
                "wiring_geometry_incomplete": True,
            },
        )
    if component_id == "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP":
        return (
            "MEASURED_TOTAL_MASS_WITH_UNIFORM_GEOMETRIC_INERTIA_ESTIMATE",
            {
                "internal_component_mass_distribution_not_measured": True,
                "dynamic_gripper_inertia_not_modeled": True,
                "closure_angle_deg": 0,
            },
        )
    if component_id in {
        "J2A_OUTPUT_EQ",
        "J2B_OUTPUT_EQ",
        "J3_OUTPUT_EQ",
        "J3_STATOR_EQ",
        "J4_STATOR_EQ",
        "J4_OUTPUT_EQ",
        "J5_STATOR_EQ",
        "J5_OUTPUT_EQ",
        "J6_DM_STATOR_EQ",
        "J6_DM_OUTPUT_EQ",
    }:
        return (
            "ENGINEERING_SPLIT_MASS_SCALED_CAD_RIGID_INERTIA_ESTIMATE",
            {
                "not_vendor_measured_subassembly_inertia": True,
                "motor_internal_rotor_dynamic_inertia_not_added": True,
                "gear_ratio_reflected_inertia_not_added": True,
            },
        )
    if component_id in {
        "UPPER_ARM_PRINT_MEASURED",
        "FOREARM_PRINT_MEASURED",
        "WRIST_PRELINK_PRINT_MEASURED",
    }:
        return (
            "ENGINEERING_ESTIMATE_REPAIRED_WATERTIGHT_INERTIA",
            {
                "canonical_geometry_is_working_copy_repair": True,
                "material_density_distribution_assumed_uniform": True,
            },
        )
    raise AuditFailure(f"UNCLASSIFIED_COMPONENT_STATUS:{component_id}")


def print_component_atoms(
    component: dict[str, Any], artifact_cache: dict[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    component_id = component["component_id"]
    component_mass = float(component["_authority_mass_kg"])
    allocation: dict[str, float] = {}
    if component_id == "UPPER_ARM_PRINT_MEASURED":
        allocation_block = component["equal_density_volume_allocation"]
        require(
            abs(float(allocation_block["frozen_total_measured_mass_kg"]) - component_mass)
            < 1.0e-15,
            "UPPER_ARM_TOTAL_MASS",
        )
        for part in allocation_block["parts"]:
            allocation[part["member"]] = float(part["allocated_mass_kg"])
        require(abs(math.fsum(allocation.values()) - component_mass) < 1.0e-12, "UPPER_ARM_SPLIT_CLOSURE")
    else:
        require(len(component["members"]) == 1, f"PRINT_MEMBER_COUNT:{component_id}")
        allocation[component["members"][0]["member"]] = component_mass

    atoms: list[dict[str, Any]] = []
    for member in component["members"]:
        relative = member["artifact_path"]
        cached = artifact_cache[relative]
        mass_kg = allocation[member["member"]]
        rotation, translation = placement_parts(member["source_placement_matrix_row_major"])
        part_properties = cached["part"]
        poly = cached["poly"]
        path_a_inertia_local = (
            part_properties["raw_inertia_com_mm5"]
            * (mass_kg / part_properties["volume_mm3"])
            * MM2_TO_M2
        )
        path_b_inertia_local = poly["raw_inertia_com_mm5"] * (
            mass_kg / poly["volume_mm3"]
        ) * MM2_TO_M2
        path_a_com_world = rotation @ part_properties["com_mm"] + translation
        path_b_com_world = rotation @ poly["com_mm"] + translation
        atom = {
            "atom_id": f"{component_id}:{member['member']}",
            "component_id": component_id,
            "member": member["member"],
            "source_object": member["member"],
            "source_solid_index": 1,
            "mass_kg": mass_kg,
            "orientation_reversed_to_positive": part_properties["orientation_reversed_to_positive"],
            "path_a": {
                "volume_mm3": part_properties["volume_mm3"],
                "com_world_mm": path_a_com_world,
                "inertia_com_world_kg_m2": symmetrize(
                    rotation @ path_a_inertia_local @ rotation.T
                ),
            },
            "path_b_normal": {
                "volume_mm3": poly["volume_mm3"],
                "com_world_mm": path_b_com_world,
                "inertia_com_world_kg_m2": symmetrize(
                    rotation @ path_b_inertia_local @ rotation.T
                ),
                "triangle_count": cached["entry"]["mesh_qa"]["facet_count"],
                "point_count": cached["entry"]["mesh_qa"]["point_count"],
                "linear_deflection_mm": None,
                "angular_deflection_rad": None,
                "source": "FROZEN_CANONICAL_STL",
            },
            "path_b_fine": {
                "volume_mm3": poly["volume_mm3"],
                "com_world_mm": path_b_com_world,
                "inertia_com_world_kg_m2": symmetrize(
                    rotation @ path_b_inertia_local @ rotation.T
                ),
                "triangle_count": cached["entry"]["mesh_qa"]["facet_count"],
                "point_count": cached["entry"]["mesh_qa"]["point_count"],
                "linear_deflection_mm": None,
                "angular_deflection_rad": None,
                "source": "FROZEN_CANONICAL_STL",
            },
            "_shape": cached["solid_world"].copy(),
        }
        atoms.append(atom)
    return atoms


def brep_component_atoms(doc: Any, component: dict[str, Any]) -> list[dict[str, Any]]:
    component_id = component["component_id"]
    component_mass = float(component["_authority_mass_kg"])
    atoms: list[dict[str, Any]] = []
    for member in component["members"]:
        token = str(member["member"])
        require("collision" not in token.lower() and "proxy" not in token.lower(), f"COLLISION_PROXY_TOKEN:{token}")
        object_name = str(member.get("source_object") or token)
        obj = doc.getObject(object_name)
        require(obj is not None, f"CAD_OBJECT_MISSING:{object_name}")
        require(obj.TypeId == "Part::Feature", f"CAD_OBJECT_NOT_PART:{object_name}:{obj.TypeId}")
        solids = list(obj.Shape.Solids)
        selected = member.get("selected_solid_indices")
        selected_indices = list(selected) if selected else list(range(1, len(solids) + 1))
        require(selected_indices, f"NO_SELECTED_SOLIDS:{token}")
        require(all(1 <= int(index) <= len(solids) for index in selected_indices), f"SOLID_SELECTION_RANGE:{token}")
        for index in selected_indices:
            shape, signed_before, reversed_to_positive = normalized_solid(solids[int(index) - 1])
            volume = float(shape.Volume)
            raw = matrix_of_inertia(shape)
            atoms.append(
                {
                    "atom_id": f"{component_id}:{token}:solid_{int(index)}",
                    "component_id": component_id,
                    "member": token,
                    "source_object": object_name,
                    "source_solid_index": int(index),
                    "signed_volume_before_normalization_mm3": signed_before,
                    "orientation_reversed_to_positive": reversed_to_positive,
                    "path_a": {
                        "volume_mm3": volume,
                        "com_world_mm": v3(shape.CenterOfMass),
                        "raw_inertia_com_mm5": raw,
                    },
                    "_shape": shape,
                }
            )
    require(atoms, f"COMPONENT_HAS_NO_ATOMS:{component_id}")
    total_volume = math.fsum(float(atom["path_a"]["volume_mm3"]) for atom in atoms)
    require(total_volume > 0.0, f"COMPONENT_VOLUME_NONPOSITIVE:{component_id}")
    for atom in atoms:
        mass_kg = component_mass * float(atom["path_a"]["volume_mm3"]) / total_volume
        atom["mass_kg"] = mass_kg
        atom["path_a"]["inertia_com_world_kg_m2"] = symmetrize(
            atom["path_a"].pop("raw_inertia_com_mm5")
            * (mass_kg / float(atom["path_a"]["volume_mm3"]))
            * MM2_TO_M2
        )
    require(abs(math.fsum(float(atom["mass_kg"]) for atom in atoms) - component_mass) < 1.0e-12, f"ATOM_MASS_CLOSURE:{component_id}")
    return atoms


def tessellate_component_atoms(atoms: Sequence[dict[str, Any]]) -> None:
    for atom in atoms:
        for label, deflection in (
            ("path_b_normal", NORMAL_DEFLECTION_MM),
            ("path_b_fine", FINE_DEFLECTION_MM),
        ):
            result = tessellated_shape_properties(
                atom["_shape"], float(atom["mass_kg"]), deflection
            )
            atom[label] = {
                "volume_mm3": result["volume_mm3"],
                "com_world_mm": result["com_world_mm"],
                "inertia_com_world_kg_m2": result["inertia_com_world_kg_m2"],
                "triangle_count": result["triangle_count"],
                "point_count": result["point_count"],
                "linear_deflection_mm": result["linear_deflection_mm"],
                "angular_deflection_rad": result["angular_deflection_rad"],
                "relative": False,
            }


def serialize_aggregate(value: dict[str, Any]) -> dict[str, Any]:
    return {
        "mass_kg": value["mass_kg"],
        "volume_mm3": value["volume_mm3"],
        "com_world_mm": value["com_world_mm"].tolist(),
        "inertia_about_component_frozen_com_world_kg_m2": value[
            "inertia_about_reference_world_kg_m2"
        ].tolist(),
    }


def build_component_geometry(
    doc: Any,
    components: Sequence[dict[str, Any]],
    artifact_cache: dict[str, dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, list[dict[str, Any]]]]:
    records: list[dict[str, Any]] = []
    atoms_by_component: dict[str, list[dict[str, Any]]] = {}
    for component in components:
        component_id = component["component_id"]
        is_print = any(member.get("artifact_path") for member in component.get("members", []))
        if is_print:
            atoms = print_component_atoms(component, artifact_cache)
        else:
            atoms = brep_component_atoms(doc, component)
            tessellate_component_atoms(atoms)
        atoms_by_component[component_id] = atoms

        frozen_component_com = v3(component["com_world_mm"])
        path_a = aggregate_atoms_about_reference(atoms, "path_a", frozen_component_com)
        normal = aggregate_atoms_about_reference(atoms, "path_b_normal", frozen_component_com)
        fine = aggregate_atoms_about_reference(atoms, "path_b_fine", frozen_component_com)
        normal_fine_error = relative_frobenius(
            normal["inertia_about_reference_world_kg_m2"],
            fine["inertia_about_reference_world_kg_m2"],
        )
        selected_key = "path_b_fine"
        selected = fine
        ultra_summary: dict[str, Any] | None = None
        if normal_fine_error > MESH_CONVERGENCE_LIMIT and not is_print:
            for atom in atoms:
                result = tessellated_shape_properties(
                    atom["_shape"], float(atom["mass_kg"]), ULTRA_DEFLECTION_MM
                )
                atom["path_b_ultra"] = {
                    "volume_mm3": result["volume_mm3"],
                    "com_world_mm": result["com_world_mm"],
                    "inertia_com_world_kg_m2": result["inertia_com_world_kg_m2"],
                    "triangle_count": result["triangle_count"],
                    "point_count": result["point_count"],
                    "linear_deflection_mm": result["linear_deflection_mm"],
                    "angular_deflection_rad": result["angular_deflection_rad"],
                    "relative": False,
                }
            selected_key = "path_b_ultra"
            selected = aggregate_atoms_about_reference(atoms, selected_key, frozen_component_com)
            fine_ultra_error = relative_frobenius(
                fine["inertia_about_reference_world_kg_m2"],
                selected["inertia_about_reference_world_kg_m2"],
            )
            ultra_summary = {
                **serialize_aggregate(selected),
                "linear_deflection_mm": ULTRA_DEFLECTION_MM,
                "angular_deflection_rad": ANGULAR_DEFLECTION_RAD,
                "point_count": sum(int(atom[selected_key]["point_count"]) for atom in atoms),
                "triangle_count": sum(int(atom[selected_key]["triangle_count"]) for atom in atoms),
                "fine_to_ultra_relative_frobenius_error": fine_ultra_error,
            }
            require(fine_ultra_error < MESH_CONVERGENCE_LIMIT, f"MESH_ULTRA_CONVERGENCE:{component_id}:{fine_ultra_error}")

        path_a_b_error = relative_frobenius(
            path_a["inertia_about_reference_world_kg_m2"],
            selected["inertia_about_reference_world_kg_m2"],
        )
        component_com_error_m = float(
            np.linalg.norm(path_a["com_world_mm"] - frozen_component_com) * MM_TO_M
        )
        require(component_com_error_m < COM_REPRO_LIMIT_M, f"COMPONENT_COM_V2_REPRO:{component_id}:{component_com_error_m}")
        require(path_a_b_error < BREP_PATH_LIMIT, f"COMPONENT_PATH_A_B:{component_id}:{path_a_b_error}")
        for atom in atoms:
            atom["_selected_path_b_key"] = selected_key
        status, limitations = component_source_status(component_id)

        normal_summary = {
            **serialize_aggregate(normal),
            "linear_deflection_mm": None if is_print else NORMAL_DEFLECTION_MM,
            "angular_deflection_rad": None if is_print else ANGULAR_DEFLECTION_RAD,
            "point_count": sum(int(atom["path_b_normal"]["point_count"]) for atom in atoms),
            "triangle_count": sum(int(atom["path_b_normal"]["triangle_count"]) for atom in atoms),
        }
        fine_summary = {
            **serialize_aggregate(fine),
            "linear_deflection_mm": None if is_print else FINE_DEFLECTION_MM,
            "angular_deflection_rad": None if is_print else ANGULAR_DEFLECTION_RAD,
            "point_count": sum(int(atom["path_b_fine"]["point_count"]) for atom in atoms),
            "triangle_count": sum(int(atom["path_b_fine"]["triangle_count"]) for atom in atoms),
        }
        record = {
            "component_id": component_id,
            "owner_link": component["owner_link"],
            "mass_kg": float(component["_authority_mass_kg"]),
            "mass_authority": MASS_LEDGER,
            "geometry_membership_and_frozen_com_authority": COM_LEDGER,
            "frozen_com_world_mm": frozen_component_com.tolist(),
            "geometry_class": component["geometry_class"],
            "member_tokens": list(component["geometry_member_order_inherited_exactly_from_v1_mass_ledger"]),
            "atom_count": len(atoms),
            "atom_mass_geometry_summary": [
                {
                    "atom_id": atom["atom_id"],
                    "member": atom["member"],
                    "source_object": atom["source_object"],
                    "source_solid_index_one_based": atom["source_solid_index"],
                    "mass_kg": atom["mass_kg"],
                    "path_a_physical_volume_mm3": atom["path_a"]["volume_mm3"],
                    "path_a_com_world_mm": v3(atom["path_a"]["com_world_mm"]).tolist(),
                    "orientation_reversed_to_positive": atom[
                        "orientation_reversed_to_positive"
                    ],
                }
                for atom in atoms
            ],
            "negative_oriented_atom_count_normalized": sum(
                bool(atom["orientation_reversed_to_positive"]) for atom in atoms
            ),
            "source_status": status,
            **(
                {
                    "mass_basis_status": "MEASURED_MASS_SCALED_GEOMETRIC_INERTIA_ESTIMATE",
                    "canonical_mass_geometry_artifacts": [
                        {
                            "artifact_path": member["artifact_path"],
                            "artifact_sha256": member["artifact_sha256"],
                            "report_path": member["report_path"],
                            "report_sha256": member["report_sha256"],
                            "canonical_geometry_is_working_copy_repair": True,
                        }
                        for member in component["members"]
                    ],
                }
                if is_print
                else {}
            ),
            "limitations": limitations,
            "path_a_occt": serialize_aggregate(path_a),
            "path_b_triangle": {
                "normal": normal_summary,
                "fine": fine_summary,
                "ultra_if_required": ultra_summary,
                "normal_to_fine_relative_frobenius_error": normal_fine_error,
                "convergence_limit_strict_less_than_or_equal": MESH_CONVERGENCE_LIMIT,
                "selected_resolution": selected_key,
                "convergence_pass": (
                    normal_fine_error <= MESH_CONVERGENCE_LIMIT
                    or ultra_summary is not None
                ),
            },
            "path_a_b_relative_frobenius_error": path_a_b_error,
            "path_a_b_limit_strict_less_than": BREP_PATH_LIMIT,
            "com_v2_reproduction_error_m": component_com_error_m,
            "com_v2_limit_strict_less_than_m": COM_REPRO_LIMIT_M,
            "candidate_only_not_frozen": True,
            "pass_for_candidate_calculation": True,
            "_selected_path_b_key": selected_key,
        }
        records.append(record)
    return records, atoms_by_component


def assemble_link_path(
    atoms: Sequence[dict[str, Any]],
    path_selector: str,
    frame_origin_world_mm: np.ndarray,
    rotation_world_from_link: np.ndarray,
    frozen_com_link_m: np.ndarray,
    frozen_com_world_mm: np.ndarray,
) -> dict[str, Any]:
    total_mass = math.fsum(float(atom["mass_kg"]) for atom in atoms)
    require(total_mass > 0.0, "LINK_MASS_NONPOSITIVE")

    def selected(atom: dict[str, Any]) -> dict[str, Any]:
        key = path_selector
        if path_selector == "path_b_selected":
            key = str(atom["_selected_path_b_key"])
        return atom[key]

    recomputed_world_com = np.array(
        [
            math.fsum(
                float(atom["mass_kg"]) * float(selected(atom)["com_world_mm"][axis])
                for atom in atoms
            )
            / total_mass
            for axis in range(3)
        ]
    )
    recomputed_link_com = (
        rotation_world_from_link.T
        @ (recomputed_world_com - frame_origin_world_mm)
        * MM_TO_M
    )

    tensor_link_direct = np.zeros((3, 3), dtype=float)
    tensor_world = np.zeros((3, 3), dtype=float)
    for atom in atoms:
        path = selected(atom)
        mass_kg = float(atom["mass_kg"])
        atom_com_world = v3(path["com_world_mm"])
        atom_inertia_world = np.asarray(path["inertia_com_world_kg_m2"], dtype=float)
        atom_com_link_m = (
            rotation_world_from_link.T @ (atom_com_world - frame_origin_world_mm)
        ) * MM_TO_M
        atom_inertia_link = (
            rotation_world_from_link.T @ atom_inertia_world @ rotation_world_from_link
        )
        tensor_link_direct += parallel_axis(
            atom_inertia_link,
            mass_kg,
            atom_com_link_m - frozen_com_link_m,
        )
        tensor_world += parallel_axis(
            atom_inertia_world,
            mass_kg,
            (atom_com_world - frozen_com_world_mm) * MM_TO_M,
        )
    tensor_link_direct = symmetrize(tensor_link_direct)
    tensor_link_from_world = symmetrize(
        rotation_world_from_link.T @ tensor_world @ rotation_world_from_link
    )
    frame_error = frobenius(tensor_link_direct - tensor_link_from_world)
    trace_error = abs(float(np.trace(tensor_link_direct) - np.trace(tensor_world)))
    principal_direct = np.linalg.eigvalsh(tensor_link_direct)
    principal_world = np.linalg.eigvalsh(tensor_world)
    principal_error = float(np.max(np.abs(principal_direct - principal_world)))
    return {
        "mass_kg": total_mass,
        "recomputed_com_world_mm": recomputed_world_com,
        "recomputed_com_link_m": recomputed_link_com,
        "inertia_link_direct_kg_m2": tensor_link_direct,
        "inertia_world_kg_m2": symmetrize(tensor_world),
        "inertia_link_from_world_kg_m2": tensor_link_from_world,
        "world_link_frobenius_absolute_error_kg_m2": frame_error,
        "trace_invariant_absolute_error_kg_m2": trace_error,
        "principal_moment_invariant_max_abs_error_kg_m2": principal_error,
    }


def serialize_link_path(path: dict[str, Any], metrics: dict[str, Any]) -> dict[str, Any]:
    return {
        "mass_kg": path["mass_kg"],
        "recomputed_com_world_mm": path["recomputed_com_world_mm"].tolist(),
        "recomputed_com_link_m": path["recomputed_com_link_m"].tolist(),
        "candidate_inertia": metrics,
        "world_frame_inertia_about_same_frozen_com_kg_m2": path[
            "inertia_world_kg_m2"
        ].tolist(),
        "owner_link_inertia_from_world_kg_m2": path[
            "inertia_link_from_world_kg_m2"
        ].tolist(),
        "world_link_frobenius_absolute_error_kg_m2": path[
            "world_link_frobenius_absolute_error_kg_m2"
        ],
        "trace_invariant_absolute_error_kg_m2": path[
            "trace_invariant_absolute_error_kg_m2"
        ],
        "principal_moment_invariant_max_abs_error_kg_m2": path[
            "principal_moment_invariant_max_abs_error_kg_m2"
        ],
        "frame_limit_strict_less_than_kg_m2": FRAME_ERROR_LIMIT,
        "frame_pass": path["world_link_frobenius_absolute_error_kg_m2"] < FRAME_ERROR_LIMIT,
    }


def build_link_candidates(
    com: dict[str, Any],
    component_records: Sequence[dict[str, Any]],
    atoms_by_component: dict[str, list[dict[str, Any]]],
) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    component_record_map = {record["component_id"]: record for record in component_records}
    link_records: list[dict[str, Any]] = []
    private: dict[str, dict[str, Any]] = {}
    for link_name in LINK_ORDER:
        authority = com["links"][link_name]
        component_ids = list(authority["component_ids"])
        atoms = [
            atom
            for component_id in component_ids
            for atom in atoms_by_component[component_id]
        ]
        frozen_com_link_m = v3(authority["com_link_m"])
        frozen_com_world_mm = v3(authority["com_world_mm"])
        frame = authority["frame_world_at_mechanical_zero"]
        origin = v3(frame["origin_mm"])
        rotation = np.asarray(frame["rotation_matrix_row_major"], dtype=float)
        require(
            np.max(np.abs(rotation.T @ rotation - IDENTITY3)) < 1.0e-12,
            f"LINK_FRAME_NOT_ORTHONORMAL:{link_name}",
        )
        path_a = assemble_link_path(
            atoms,
            "path_a",
            origin,
            rotation,
            frozen_com_link_m,
            frozen_com_world_mm,
        )
        path_b = assemble_link_path(
            atoms,
            "path_b_selected",
            origin,
            rotation,
            frozen_com_link_m,
            frozen_com_world_mm,
        )
        mass_kg = float(authority["mass_kg"])
        require(abs(path_a["mass_kg"] - mass_kg) < 1.0e-12, f"LINK_ATOM_MASS:{link_name}")
        com_world_error_m = float(
            np.linalg.norm(path_a["recomputed_com_world_mm"] - frozen_com_world_mm)
            * MM_TO_M
        )
        com_link_error_m = float(
            np.linalg.norm(path_a["recomputed_com_link_m"] - frozen_com_link_m)
        )
        com_error_m = max(com_world_error_m, com_link_error_m)
        require(com_error_m < COM_REPRO_LIMIT_M, f"LINK_COM_V2_REPRO:{link_name}:{com_error_m}")

        r_max_m = 0.0
        for atom in atoms:
            for corner in bbox_corners(atom["_shape"].BoundBox):
                r_max_m = max(
                    r_max_m,
                    float(np.linalg.norm(corner - frozen_com_world_mm) * MM_TO_M),
                )
        metrics_a = tensor_metrics(path_a["inertia_link_direct_kg_m2"], mass_kg, r_max_m)
        metrics_b = tensor_metrics(path_b["inertia_link_direct_kg_m2"], mass_kg, r_max_m)
        path_error = relative_frobenius(
            path_a["inertia_link_direct_kg_m2"], path_b["inertia_link_direct_kg_m2"]
        )
        require(path_error < BREP_PATH_LIMIT, f"LINK_PATH_A_B:{link_name}:{path_error}")
        require(metrics_a["pass"], f"LINK_PHYSICS:{link_name}")
        require(
            path_a["world_link_frobenius_absolute_error_kg_m2"] < FRAME_ERROR_LIMIT,
            f"LINK_FRAME_A:{link_name}",
        )
        require(
            path_b["world_link_frobenius_absolute_error_kg_m2"] < FRAME_ERROR_LIMIT,
            f"LINK_FRAME_B:{link_name}",
        )
        record = {
            "link": link_name,
            "mass_kg": mass_kg,
            "frozen_com_xyz_m_in_link_frame": frozen_com_link_m.tolist(),
            "frozen_com_world_at_mechanical_zero_m": (frozen_com_world_mm * MM_TO_M).tolist(),
            "inertia_reference_point": "FROZEN_COM_V2",
            "inertia_expressed_in": "OWNER_LINK_FRAME",
            "component_ids": component_ids,
            "component_mass_sum_kg": math.fsum(
                float(component_record_map[component_id]["mass_kg"])
                for component_id in component_ids
            ),
            "com_v2_reproduction": {
                "world_error_m": com_world_error_m,
                "owner_link_error_m": com_link_error_m,
                "max_error_m": com_error_m,
                "limit_strict_less_than_m": COM_REPRO_LIMIT_M,
                "pass": True,
            },
            "candidate_uniform_proxy_path_a_occt": serialize_link_path(path_a, metrics_a),
            "candidate_uniform_proxy_path_b_triangle": serialize_link_path(path_b, metrics_b),
            "path_a_b_relative_frobenius_error": path_error,
            "path_a_b_limit_strict_less_than": BREP_PATH_LIMIT,
            "candidate_only_not_frozen": True,
            "authoritative_inertia_tensor": None,
            "freeze_status": "PROHIBITED_BY_FAIL_CLOSED_GATES",
        }
        link_records.append(record)
        private[link_name] = {
            "record": record,
            "atoms": atoms,
            "path_a": path_a,
            "path_b": path_b,
            "origin": origin,
            "rotation": rotation,
            "frozen_com_link_m": frozen_com_link_m,
            "frozen_com_world_mm": frozen_com_world_mm,
        }
    return link_records, private


def residual_sensitivity_audit(
    components: Sequence[dict[str, Any]],
    atoms_by_component: dict[str, list[dict[str, Any]]],
    link_private: dict[str, dict[str, Any]],
) -> list[dict[str, Any]]:
    component_map = {component["component_id"]: component for component in components}
    residual_ids = [
        "RESIDUAL_A_LINK5_ADAPTER_HARDWARE_WIRING",
        "RESIDUAL_B_LINK4_J5_INTERFACE_HARDWARE_WIRING",
        "RESIDUAL_C_LINK3_FOREARM_HARDWARE_WIRING",
    ]
    results: list[dict[str, Any]] = []
    for component_id in residual_ids:
        component = component_map[component_id]
        link_name = component["owner_link"]
        private = link_private[link_name]
        uniform_link = private["path_a"]["inertia_link_direct_kg_m2"]
        rotation = private["rotation"]
        origin = private["origin"]
        link_com = private["frozen_com_link_m"]
        component_com_world = v3(component["com_world_mm"])
        component_com_link_m = (
            rotation.T @ (component_com_world - origin)
        ) * MM_TO_M
        component_mass = float(component["_authority_mass_kg"])

        uniform_component_contribution = np.zeros((3, 3), dtype=float)
        for atom in atoms_by_component[component_id]:
            atom_path = atom["path_a"]
            atom_com_world = v3(atom_path["com_world_mm"])
            atom_com_link_m = rotation.T @ (atom_com_world - origin) * MM_TO_M
            atom_inertia_link = (
                rotation.T
                @ np.asarray(atom_path["inertia_com_world_kg_m2"], dtype=float)
                @ rotation
            )
            uniform_component_contribution += parallel_axis(
                atom_inertia_link,
                float(atom["mass_kg"]),
                atom_com_link_m - link_com,
            )
        point_component_contribution = parallel_axis(
            np.zeros((3, 3), dtype=float),
            component_mass,
            component_com_link_m - link_com,
        )
        point_model_link = (
            uniform_link - uniform_component_contribution + point_component_contribution
        )
        difference = frobenius(uniform_link - point_model_link)
        denominator = frobenius(uniform_link)
        sensitivity = difference / denominator
        percent = sensitivity * 100.0
        if percent < 5.0:
            status = "PASS"
        elif percent <= 10.0:
            status = "FAIL_WITH_MODEL_LIMITATION"
        else:
            status = "FAIL_HIGH_SENSITIVITY"
        results.append(
            {
                "component_id": component_id,
                "owner_link": link_name,
                "mass_kg": component_mass,
                "uniform_proxy_geometry_model": "V2_TRACEABLE_BREP_MEMBER_SET_UNIFORM_EFFECTIVE_DENSITY",
                "sensitivity_model": "POINT_MASS_AT_FROZEN_COMPONENT_COM_INTRINSIC_INERTIA_ZERO",
                "uniform_proxy_component_contribution_about_link_frozen_com_kg_m2": symmetrize(
                    uniform_component_contribution
                ).tolist(),
                "point_mass_component_contribution_about_link_frozen_com_kg_m2": symmetrize(
                    point_component_contribution
                ).tolist(),
                "candidate_final_link_uniform_proxy_tensor_kg_m2": uniform_link.tolist(),
                "candidate_final_link_point_model_tensor_kg_m2": symmetrize(point_model_link).tolist(),
                "difference_frobenius_kg_m2": difference,
                "denominator_candidate_final_uniform_link_frobenius_kg_m2": denominator,
                "relative_sensitivity": sensitivity,
                "sensitivity_percent": percent,
                "thresholds_percent": {
                    "pass_strict_less_than": 5.0,
                    "fail_with_model_limitation_inclusive_range": [5.0, 10.0],
                    "fail_high_sensitivity_strict_greater_than": 10.0,
                },
                "status": status,
                "pass": status == "PASS",
            }
        )
    return results


def geometry_signature(atom: dict[str, Any]) -> tuple[Any, ...]:
    shape = atom["_shape"]
    box = shape.BoundBox
    center = v3(shape.CenterOfMass)
    return tuple(
        round(value, 9)
        for value in (
            float(shape.Volume),
            *center.tolist(),
            float(box.XMin),
            float(box.YMin),
            float(box.ZMin),
            float(box.XMax),
            float(box.YMax),
            float(box.ZMax),
        )
    )


def duplicate_overlap_audit(
    components: Sequence[dict[str, Any]],
    atoms_by_component: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    all_atoms = [
        atom
        for component in components
        for atom in atoms_by_component[component["component_id"]]
    ]
    source_identity: dict[tuple[str, int], list[str]] = {}
    signature_identity: dict[tuple[Any, ...], list[str]] = {}
    for atom in all_atoms:
        key = (str(atom["source_object"]), int(atom["source_solid_index"]))
        source_identity.setdefault(key, []).append(atom["atom_id"])
        signature_identity.setdefault(geometry_signature(atom), []).append(atom["atom_id"])
    source_duplicates = [values for values in source_identity.values() if len(values) > 1]
    exact_geometry_duplicates = [values for values in signature_identity.values() if len(values) > 1]

    events: list[dict[str, Any]] = []
    boolean_failures: list[dict[str, Any]] = []
    pair_count = 0
    bbox_candidate_count = 0
    component_summaries: list[dict[str, Any]] = []
    for component in components:
        component_id = component["component_id"]
        atoms = atoms_by_component[component_id]
        component_pair_count = 0
        component_bbox_candidates = 0
        component_event_count = 0
        for left, right in itertools.combinations(atoms, 2):
            pair_count += 1
            component_pair_count += 1
            if not bbox_intersects(left["_shape"].BoundBox, right["_shape"].BoundBox):
                continue
            bbox_candidate_count += 1
            component_bbox_candidates += 1
            try:
                common = left["_shape"].common(right["_shape"])
                common_volume = abs(float(common.Volume)) if not common.isNull() else 0.0
            except Exception as exc:
                boolean_failures.append(
                    {
                        "component_id": component_id,
                        "left_atom_id": left["atom_id"],
                        "right_atom_id": right["atom_id"],
                        "error": str(exc),
                    }
                )
                continue
            if common_volume <= OVERLAP_VOLUME_TOL_MM3:
                continue
            component_event_count += 1
            left_volume = float(left["path_a"]["volume_mm3"])
            right_volume = float(right["path_a"]["volume_mm3"])
            smaller = min(left_volume, right_volume)
            events.append(
                {
                    "component_id": component_id,
                    "left_atom_id": left["atom_id"],
                    "right_atom_id": right["atom_id"],
                    "left_source_object": left["source_object"],
                    "right_source_object": right["source_object"],
                    "left_source_solid_index_one_based": left["source_solid_index"],
                    "right_source_solid_index_one_based": right["source_solid_index"],
                    "left_source_solid_index_zero_based": int(left["source_solid_index"]) - 1,
                    "right_source_solid_index_zero_based": int(right["source_solid_index"]) - 1,
                    "common_volume_mm3": common_volume,
                    "fraction_of_smaller_solid": common_volume / smaller,
                    "full_containment_within_numeric_tolerance": common_volume / smaller >= 0.99999,
                }
            )
        component_summaries.append(
            {
                "component_id": component_id,
                "atom_count": len(atoms),
                "all_internal_atom_pairs_checked": component_pair_count,
                "bbox_candidates_boolean_checked": component_bbox_candidates,
                "positive_overlap_pair_count": component_event_count,
            }
        )

    go_component_ids = {
        "J2A_OUTPUT_EQ",
        "J2B_OUTPUT_EQ",
        "J3_OUTPUT_EQ",
        "J4_OUTPUT_EQ",
        "J5_OUTPUT_EQ",
    }
    go_disjoint_checks: list[dict[str, Any]] = []
    for component in components:
        if component["component_id"] not in go_component_ids:
            continue
        members = list(component["members"])
        require(len(members) == 2, f"GO_OUTPUT_NEUTRAL_MEMBER_COUNT:{component['component_id']}")
        selection_identity_pairs: list[dict[str, Any]] = []
        for left, right in itertools.combinations(members, 2):
            left_object = str(left["source_object"])
            right_object = str(right["source_object"])
            if left_object == right_object:
                require(
                    left.get("selected_solid_indices") is not None
                    and right.get("selected_solid_indices") is not None,
                    f"GO_SAME_SOURCE_SELECTION_MISSING:{component['component_id']}",
                )
                left_selection = set(int(value) for value in left["selected_solid_indices"])
                right_selection = set(int(value) for value in right["selected_solid_indices"])
                intersection = left_selection & right_selection
                identity_basis = "SAME_SOURCE_OBJECT_DISJOINT_ONE_BASED_SOLID_INDEX_SETS"
                disjoint = not intersection
                intersection_output: list[int] | None = sorted(intersection)
            else:
                left_selection = set()
                right_selection = set()
                identity_basis = "DISTINCT_SOURCE_OBJECT_IDENTITY_SOLID_INDICES_NOT_COMPARABLE"
                disjoint = True
                intersection_output = None
            selection_identity_pairs.append(
                {
                    "left_member": left["member"],
                    "right_member": right["member"],
                    "left_source_object": left_object,
                    "right_source_object": right_object,
                    "identity_basis": identity_basis,
                    "left_selected_solid_indices_one_based": (
                        sorted(left_selection) if left_object == right_object else None
                    ),
                    "right_selected_solid_indices_one_based": (
                        sorted(right_selection) if left_object == right_object else None
                    ),
                    "intersection_one_based": intersection_output,
                    "disjoint": disjoint,
                }
            )
        require(
            selection_identity_pairs
            and all(pair["disjoint"] for pair in selection_identity_pairs),
            f"GO_OUTPUT_NEUTRAL_SELECTION_IDENTITY_OVERLAP:{component['component_id']}",
        )
        go_disjoint_checks.append(
            {
                "component_id": component["component_id"],
                "output_and_neutral_member_count": len(members),
                "selection_identity_pairs": selection_identity_pairs,
                "all_selection_identity_pairs_disjoint": True,
                "output_and_neutral_selection_disjoint": True,
            }
        )
    require(
        len(go_disjoint_checks) == len(go_component_ids)
        and {check["component_id"] for check in go_disjoint_checks} == go_component_ids,
        "GO_OUTPUT_NEUTRAL_DISJOINT_COVERAGE_NOT_EXACTLY_FIVE",
    )

    overlap_detected = bool(events or source_duplicates or exact_geometry_duplicates)
    return {
        "scope": "EXACT_SOURCE_IDENTITY_ACROSS_ALL_ATOMS_PLUS_OCCT_COMMON_VOLUME_FOR_EVERY_INTERNAL_ATOM_PAIR_OF_ALL_17_COMPONENTS",
        "bbox_prefilter": True,
        "occt_common_volume_tolerance_mm3": OVERLAP_VOLUME_TOL_MM3,
        "geometry_atom_count": len(all_atoms),
        "internal_pair_count_checked": pair_count,
        "bbox_candidate_count_boolean_checked": bbox_candidate_count,
        "component_summaries": component_summaries,
        "source_identity_duplicates": source_duplicates,
        "exact_geometry_signature_duplicates": exact_geometry_duplicates,
        "source_identity_duplicate_count": len(source_duplicates),
        "exact_geometry_signature_duplicate_count": len(exact_geometry_duplicates),
        "positive_overlap_pair_count": len(events),
        "positive_overlap_events": events,
        "boolean_failures": boolean_failures,
        "go_output_neutral_selection_checks": go_disjoint_checks,
        "go_output_neutral_expected_component_count": 5,
        "go_output_neutral_audited_component_count": len(go_disjoint_checks),
        "all_go_output_neutral_selection_identity_pairs_disjoint": all(
            check["all_selection_identity_pairs_disjoint"]
            for check in go_disjoint_checks
        ),
        "all_go_output_neutral_selection_sets_disjoint": all(
            check["output_and_neutral_selection_disjoint"] for check in go_disjoint_checks
        ),
        "mass_geometry_union_or_subtraction_applied": False,
        "authority_silently_repaired": False,
        "mass_geometry_duplication_detected": overlap_detected,
        "status": "MASS_GEOMETRY_DUPLICATION" if overlap_detected else "PASS",
        "pass": not overlap_detected and not boolean_failures,
    }


def fmt(value: float) -> str:
    return format(float(value), ".12g")


def markdown_report(ledger: dict[str, Any]) -> str:
    lines = [
        "# V15.16 刚体惯量 V2 失败审计",
        "",
        "> 结论：**FAIL-CLOSED**。本文件不是惯量权威账本，六个 Link 的张量仅是诊断用 candidate，未冻结、未部署。",
        "",
        "## 权威与范围",
        "",
        f"- source commit: `{ledger['provenance']['source_commit']}`",
        f"- mass ledger SHA256: `{ledger['authorities']['mass']['sha256']}`",
        f"- COM V2 ledger SHA256: `{ledger['authorities']['com_v2']['sha256']}`",
        "- COM 数值权威：仅 V2；已废弃文件仅作不可变审计证据，未读取其数值。",
        "- collision proxy / rotor reflected inertia / armature / gravity / deployment: 全部 NO。",
        "",
        "## 打印件惯量几何适用性",
        "",
        "| artifact | volume rel. | COM diff (m) | inertia rel. | status |",
        "|---|---:|---:|---:|---|",
    ]
    for artifact in ledger["print_inertia_suitability"]["artifacts"]:
        agreement = artifact["path_a_b_agreement"]
        lines.append(
            "| {} | {} | {} | {} | {} |".format(
                artifact["member"],
                fmt(agreement["volume_relative_error"]),
                fmt(agreement["centroid_distance_m"]),
                fmt(agreement["inertia_relative_frobenius_error"]),
                artifact["status"],
            )
        )

    lines.extend(
        [
            "",
            "## 六个 Link 诊断用 candidate 张量",
            "",
            "参考点均为该 Link 的 frozen COM V2，坐标轴均为 owner Link frame，单位 `kg·m²`。以下数据不得当作已冻结惯量。",
            "",
        ]
    )
    for link in ledger["links"]:
        tensor = link["candidate_uniform_proxy_path_a_occt"]["candidate_inertia"]["matrix_kg_m2"]
        lines.extend(
            [
                f"### {link['link']}",
                "",
                f"mass = `{fmt(link['mass_kg'])} kg`; COM = `{json.dumps(link['frozen_com_xyz_m_in_link_frame'])} m`",
                "",
                "```text",
                "[",
                *[
                    "  [" + ", ".join(format(float(value), ".17g") for value in row) + "]"
                    for row in tensor
                ],
                "]",
                "```",
                "",
                f"Path A/B relF = `{fmt(link['path_a_b_relative_frobenius_error'])}`; candidate only = `true`.",
                "",
            ]
        )

    lines.extend(
        [
            "## residual 模型敏感度",
            "",
            "| residual | Link | sensitivity | status |",
            "|---|---|---:|---|",
        ]
    )
    for residual in ledger["residual_sensitivity"]:
        lines.append(
            f"| {residual['component_id']} | {residual['owner_link']} | {fmt(residual['sensitivity_percent'])}% | {residual['status']} |"
        )

    overlap = ledger["duplicate_overlap_audit"]
    lines.extend(
        [
            "",
            "## duplicate / overlap 阻断",
            "",
            f"- status: `{overlap['status']}`",
            f"- geometry atoms: `{overlap['geometry_atom_count']}`",
            f"- internal pairs checked: `{overlap['internal_pair_count_checked']}`",
            f"- positive-volume overlap pairs: `{overlap['positive_overlap_pair_count']}`",
            "- 没有做 union/subtraction，没有静默改写几何 authority。",
            "",
            "前 20 个确认事件（原始 solid index 同时明示一基与零基语义）：",
            "",
            "| component | left(1-based) | right(1-based) | common mm³ | smaller fraction |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for event in overlap["positive_overlap_events"][:20]:
        lines.append(
            "| {} | {} | {} | {} | {} |".format(
                event["component_id"],
                event["left_source_solid_index_one_based"],
                event["right_source_solid_index_one_based"],
                fmt(event["common_volume_mm3"]),
                fmt(event["fraction_of_smaller_solid"]),
            )
        )

    lines.extend(
        [
            "",
            "## 阻断项",
            "",
        ]
    )
    lines.extend(f"- `{item['code']}`: {item['detail']}" for item in ledger["unresolved_items"])
    lines.extend(
        [
            "",
            "## 最终状态",
            "",
            f"`{ledger['final_status']}`",
            "",
            "未生成 `V15_16_刚体惯量账本_v2.json/.md`，未写入 URDF/MJCF inertial，未开启 gravity。",
            "",
        ]
    )
    return "\n".join(lines)


def build_outputs() -> tuple[bytes, bytes, bytes, dict[str, Any]]:
    git_guard = git_scope_gate()
    protected_before = protected_hash_snapshot()
    math_test = run_math_test()
    mass, com, components = load_authorities()
    occt_gate = occt_semantics_gate()

    document = App.openDocument(str(ROOT / SOURCE_CAD))
    try:
        suitability, artifact_cache = audit_print_artifacts(components)
        component_records, atoms_by_component = build_component_geometry(
            document, components, artifact_cache
        )
        link_records, link_private = build_link_candidates(
            com, component_records, atoms_by_component
        )
        residual = residual_sensitivity_audit(
            components, atoms_by_component, link_private
        )
        overlap = duplicate_overlap_audit(components, atoms_by_component)
    finally:
        App.closeDocument(document.Name)

    protected_after = protected_hash_snapshot()
    require(protected_before == protected_after, "PROTECTED_INPUT_CHANGED_DURING_BUILD")
    require(suitability["all_four_pass"], "PRINT_SUITABILITY_NOT_ALL_PASS")
    require(len(component_records) == EXPECTED_COMPONENT_COUNT, "OUTPUT_COMPONENT_COUNT")
    require(len(link_records) == len(LINK_ORDER), "OUTPUT_LINK_COUNT")

    max_component_com_error = max(
        float(record["com_v2_reproduction_error_m"]) for record in component_records
    )
    max_link_com_error = max(
        float(record["com_v2_reproduction"]["max_error_m"]) for record in link_records
    )
    max_com_error = max(max_component_com_error, max_link_com_error)
    max_component_path_error = max(
        float(record["path_a_b_relative_frobenius_error"])
        for record in component_records
    )
    max_link_path_error = max(
        float(record["path_a_b_relative_frobenius_error"])
        for record in link_records
    )
    max_path_error = max(max_component_path_error, max_link_path_error)
    max_convergence_error = max(
        float(record["path_b_triangle"]["normal_to_fine_relative_frobenius_error"])
        for record in component_records
    )
    max_frame_error = max(
        max(
            float(record[path]["world_link_frobenius_absolute_error_kg_m2"])
            for path in (
                "candidate_uniform_proxy_path_a_occt",
                "candidate_uniform_proxy_path_b_triangle",
            )
        )
        for record in link_records
    )
    all_physics = all(
        record["candidate_uniform_proxy_path_a_occt"]["candidate_inertia"]["pass"]
        for record in link_records
    )
    all_component_mass_closure = all(
        abs(
            math.fsum(float(atom["mass_kg"]) for atom in atoms_by_component[record["component_id"]])
            - float(record["mass_kg"])
        )
        < 1.0e-12
        for record in component_records
    )
    total_link_mass = math.fsum(float(record["mass_kg"]) for record in link_records)
    total_mass_pass = abs(total_link_mass - EXPECTED_TOTAL_MASS_KG) < 1.0e-12
    residual_failures = [item for item in residual if not item["pass"]]
    require(residual_failures, "EXPECTED_RESIDUAL_MODEL_LIMITATION_NOT_REPRODUCED")
    require(not overlap["pass"], "EXPECTED_MASS_GEOMETRY_DUPLICATION_NOT_REPRODUCED")

    unresolved_items = [
        {
            "code": "RESIDUAL_PROXY_MODEL_SENSITIVITY",
            "detail": "; ".join(
                f"{item['component_id']}={item['sensitivity_percent']:.12g}%:{item['status']}"
                for item in residual_failures
            ),
            "blocking": True,
            "required_resolution": "obtain defensible residual mass distribution/geometry or direct inertia measurement",
        },
        {
            "code": "MASS_GEOMETRY_DUPLICATION",
            "detail": (
                f"{overlap['positive_overlap_pair_count']} positive-volume internal atom overlaps "
                "were recomputed with OCCT common-volume booleans"
            ),
            "blocking": True,
            "required_resolution": "establish an explicitly de-overlapped, reviewable mass-geometry authority without changing V15.15 authorities",
        },
    ]

    suitability_bytes = canonical_json_bytes(suitability)
    public_component_records = []
    for source in component_records:
        record = dict(source)
        record.pop("_selected_path_b_key", None)
        public_component_records.append(record)

    ledger: dict[str, Any] = {
        "schema": "go-m8010-arm-v15.16-rigid-inertia-fail-audit-v2/1.0",
        "revision": "V15.16-RIGID_BODY_INERTIA_V2_FAIL_AUDIT",
        "scope": "LINK2_TO_GRIPPER_CANDIDATE_RIGID_INERTIA_DIAGNOSTIC_ONLY_NO_FREEZE_NO_DEPLOYMENT",
        "final_status": "V15.16 INERTIA_LEDGER_V2 = FAIL",
        "authoritative_ledger_written": False,
        "freeze_permitted": False,
        "candidate_tensor_policy": "DIAGNOSTIC_ONLY_NON_AUTHORITATIVE_NOT_FROZEN",
        "provenance": {
            "source_authority_branch": "agent/v15-15-com-volume-v2",
            "source_commit": SOURCE_COMMIT,
            "source_tree": SOURCE_TREE,
            "target_branch": TARGET_BRANCH,
            "source_commit_strictly_locked": True,
            "runtime": {
                "python": sys.version.split()[0],
                "freecad": ".".join(str(value) for value in App.Version()[:3]),
                "numpy": np.__version__,
            },
        },
        "git_scope_guard": git_guard,
        "authorities": {
            "mass": {
                "path": MASS_LEDGER,
                "sha256": PROTECTED_HASHES[MASS_LEDGER],
                "role": "ONLY_MASS_AUTHORITY",
                "cad_volume_used_to_derive_mass": False,
            },
            "com_v2": {
                "path": COM_LEDGER,
                "sha256": PROTECTED_HASHES[COM_LEDGER],
                "role": "ONLY_COMPONENT_MEMBERSHIP_PLACEMENT_AND_FROZEN_COM_AUTHORITY",
            },
            "superseded_com_audit_artifacts": {
                "paths": ["V15_15_COM账本_v1.json", "V15_15_COM账本_v1.md"],
                "role": "SUPERSEDED_AUDIT_ARTIFACT_PROTECTED_HASH_ONLY",
                "bytes_parsed": False,
                "numeric_values_used": False,
            },
            "geometry": {
                "path": SOURCE_CAD,
                "sha256": PROTECTED_HASHES[SOURCE_CAD],
                "opened_without_save": True,
                "collision_proxy_used": False,
            },
            "protected_inputs_before": protected_before,
            "protected_inputs_after": protected_after,
            "all_protected_inputs_unchanged": True,
        },
        "math_test": math_test,
        "occt_matrix_semantics_gate": occt_gate,
        "unit_audit": {
            "cad_length_unit": "mm",
            "mass_unit": "kg",
            "raw_volume_second_integral_unit": "mm^5",
            "effective_density_unit": "kg/mm^3",
            "final_inertia_unit": "kg*m^2",
            "conversion": "raw_mm5*(mass_kg/volume_mm3)*1e-6",
            "polyhedral_math_test_pass": math_test["pass"],
            "occt_analytic_semantics_pass": occt_gate["pass"],
            "pass": True,
        },
        "print_inertia_suitability": {
            "report_path": SUITABILITY_JSON,
            "report_sha256": sha256_bytes(suitability_bytes),
            "all_four_pass": suitability["all_four_pass"],
            "artifacts": suitability["artifacts"],
        },
        "component_order": [component["component_id"] for component in components],
        "components": public_component_records,
        "links": link_records,
        "residual_sensitivity": residual,
        "duplicate_overlap_audit": overlap,
        "validation": {
            "component_count": len(component_records),
            "component_mass_closure_pass": all_component_mass_closure,
            "link_count": len(link_records),
            "total_link2_to_gripper_mass_kg": total_link_mass,
            "expected_total_mass_kg": EXPECTED_TOTAL_MASS_KG,
            "total_mass_pass": total_mass_pass,
            "max_component_com_v2_reproduction_error_m": max_component_com_error,
            "max_link_com_v2_reproduction_error_m": max_link_com_error,
            "max_com_v2_reproduction_error_m": max_com_error,
            "com_v2_limit_strict_less_than_m": COM_REPRO_LIMIT_M,
            "com_v2_reproduction_pass": max_com_error < COM_REPRO_LIMIT_M,
            "max_component_path_a_b_relative_frobenius_error": max_component_path_error,
            "max_link_path_a_b_relative_frobenius_error": max_link_path_error,
            "max_path_a_b_relative_frobenius_error": max_path_error,
            "path_a_b_limit_strict_less_than": BREP_PATH_LIMIT,
            "path_a_b_pass": max_path_error < BREP_PATH_LIMIT,
            "max_normal_fine_mesh_convergence_relative_frobenius_error": max_convergence_error,
            "mesh_convergence_limit": MESH_CONVERGENCE_LIMIT,
            "mesh_convergence_pass": all(
                record["path_b_triangle"]["convergence_pass"]
                for record in component_records
            ),
            "max_world_link_tensor_frame_error_kg_m2": max_frame_error,
            "world_link_frame_limit_strict_less_than_kg_m2": FRAME_ERROR_LIMIT,
            "world_link_frame_pass": max_frame_error < FRAME_ERROR_LIMIT,
            "candidate_tensor_physics_checks_pass": all_physics,
            "print_suitability_pass": suitability["all_four_pass"],
            "residual_sensitivity_pass": not residual_failures,
            "duplicate_overlap_pass": overlap["pass"],
            "all_freeze_gates_pass": False,
        },
        "failure_reasons": [
            "RESIDUAL_PROXY_MODEL_SENSITIVITY",
            "MASS_GEOMETRY_DUPLICATION",
        ],
        "prohibited_outputs": {
            "collision_proxy_used": False,
            "v15_15_authority_modified": False,
            "cad_kinematics_tf_control_modified": False,
            "urdf_or_mjcf_inertial_written": False,
            "gravity_enabled": False,
            "armature_added": False,
            "gear_ratio_reflected_rotor_inertia_added": False,
            "motor_internal_dynamic_inertia_added": False,
            "damping_friction_or_actuator_dynamics_added": False,
        },
        "unresolved_items": unresolved_items,
    }
    require(not ledger["validation"]["all_freeze_gates_pass"], "FAIL_AUDIT_CANNOT_PASS")
    require(ledger["unresolved_items"], "FAIL_AUDIT_MUST_HAVE_UNRESOLVED")
    ledger_bytes = canonical_json_bytes(ledger)
    markdown_bytes = markdown_report(ledger).encode("utf-8")
    return ledger_bytes, markdown_bytes, suitability_bytes, ledger


def write_or_check(mode: str) -> None:
    ledger_bytes, markdown_bytes, suitability_bytes, ledger = build_outputs()
    outputs = {
        FAIL_JSON: ledger_bytes,
        FAIL_MD: markdown_bytes,
        SUITABILITY_JSON: suitability_bytes,
    }
    if mode == "write":
        for relative, data in outputs.items():
            (ROOT / relative).write_bytes(data)
        git_scope_gate()
        protected_hash_snapshot()
    else:
        for relative, expected in outputs.items():
            path = ROOT / relative
            require(path.is_file(), f"CHECK_OUTPUT_MISSING:{relative}")
            actual = path.read_bytes()
            require(actual == expected, f"CHECK_OUTPUT_NONDETERMINISTIC:{relative}")
    print("V15.16 V2 FAIL audit builder: {} PASS".format(mode.upper()))
    print("  final status                 = {}".format(ledger["final_status"]))
    print("  authoritative ledger written = {}".format(ledger["authoritative_ledger_written"]))
    print("  residual statuses            = {}".format(
        ", ".join(item["status"] for item in ledger["residual_sensitivity"])
    ))
    print("  overlap status               = {}".format(ledger["duplicate_overlap_audit"]["status"]))
    print("  max COM V2 error m           = {:.17g}".format(
        ledger["validation"]["max_com_v2_reproduction_error_m"]
    ))
    print("  max Path A/B relF            = {:.17g}".format(
        ledger["validation"]["max_path_a_b_relative_frobenius_error"]
    ))
    for relative in outputs:
        print("  {} SHA256 = {}".format(relative, sha256_file(ROOT / relative) if mode == "write" else sha256_bytes(outputs[relative])))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build/check the deterministic V15.16 V2 inertia failure audit."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--write", action="store_true", help="write the three failure-audit outputs")
    group.add_argument("--check", action="store_true", help="recompute and byte-compare the outputs")
    arguments = parser.parse_args()
    try:
        write_or_check("write" if arguments.write else "check")
    except AuditFailure as exc:
        print("V15.16 V2 FAIL audit builder: INTERNAL AUDIT ERROR", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
