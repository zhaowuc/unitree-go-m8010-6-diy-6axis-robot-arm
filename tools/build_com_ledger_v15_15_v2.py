from __future__ import annotations

"""Build the corrected V15.15 COM ledger volume V2.

This fail-closed tool recomputes all 17 frozen mass components.  It creates
three source-local, mass-properties-only mesh artifacts and derives their
volume centroids with an independent anchored signed-tetrahedron algorithm.
FreeCAD ``Mesh.Volume`` and ``Mesh.CenterOfGravity`` are diagnostic-only and
never participate in an accepted mass or COM formula.

No inertia tensor, gravity, damping, friction, armature, URDF inertial, MJCF
inertial, visual mesh, collision mesh, kinematics, TF, or control file is
created or modified.
"""

import argparse
import hashlib
import json
import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict, deque
from decimal import Decimal
from pathlib import Path
from typing import Any, Iterable, Sequence


def _import_freecad() -> tuple[Any, Any, Any, Exception | None]:
    try:
        import FreeCAD as app
        import Mesh as mesh_module
        import Part as part_module
        return app, mesh_module, part_module, None
    except Exception as first_error:
        # Standalone FreeCAD Python distributions sometimes omit ../lib from
        # sys.path.  This is deterministic runtime bootstrap, not a project
        # dependency or a geometry fallback.
        candidate = Path(sys.executable).resolve().parent.parent / "lib"
        if candidate.is_dir() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))
        try:
            import FreeCAD as app
            import Mesh as mesh_module
            import Part as part_module
            return app, mesh_module, part_module, None
        except Exception:
            return None, None, None, first_error


App, Mesh, Part, FREECAD_IMPORT_ERROR = _import_freecad()

SCHEMA = "go-m8010-arm-v15.15-com-ledger-volume-v2/1.0"
REVISION = "V15.15-COM_LEDGER_VOLUME_V2"
SCOPE = "LINK2_TO_GRIPPER_COM_ONLY_NO_INERTIA_NO_DYNAMICS"
SOURCE_BRANCH = "agent/v15-15-com-v1"
SOURCE_COMMIT = "71d77e902cd91bcdbce14500826f5a5fd24f67d3"
SOURCE_TREE = "7993e9b85d15b5934dcf0c3372b2ac12c6a26b67"
TARGET_BRANCH = "agent/v15-15-com-volume-v2"

SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
DERIVED_CAD = "机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd"
MASS_LEDGER = "V15_15_实测质量账本_v1.json"
V1_LEDGER = "V15_15_COM账本_v1.json"
MANIFEST = "rigid_links_v15_13/rigid_link_manifest.json"
MEMBERSHIP = "rigid_links_v15_13/rigid_link_membership.csv"
RIGID_BUILDER = "完整工程_V15_13_交付/tools/build_robot_arm_rigid_links_v15.py"
SYNTHETIC_TEST = "tools/test_mesh_volume_properties_v15_15_v2.py"
OUTPUT_JSON = "V15_15_COM账本_v2.json"
OUTPUT_MD = "V15_15_COM账本_v2.md"
ARTIFACT_DIR = "mass_properties_geometry_v15_15_v2"

UPPER_A_STL = (
    "mass_properties_geometry_v15_15/"
    "UpperArm_A_SleeveSide_PrintPart_mass_properties.stl"
)
UPPER_A_REPORT = (
    "mass_properties_geometry_v15_15/"
    "UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json"
)

MESH_SPECS: dict[str, dict[str, Any]] = {
    "UpperArm_B_Distal_PrintPart": {
        "short_name": "UpperArm_B",
        "repair": "REMOVE_NON_MANIFOLDS_THEN_EDGE_BFS",
        "before_points": 339,
        "before_facets": 696,
        "after_points": 337,
        "after_facets": 694,
    },
    "Forearm_v3_HighDetail_Display": {
        "short_name": "Forearm",
        "repair": "KEEP_2978_FACET_COMPONENT_HARMONIZE_NORMALS_FIX_DEGENERATIONS_EDGE_BFS",
        "before_points": 1460,
        "before_facets": 2980,
        "after_points": 1459,
        "after_facets": 2978,
    },
    "Wrist_Prelink_v1_HighDetail_Display": {
        "short_name": "Wrist",
        "repair": "HARMONIZE_NORMALS_FIX_DEGENERATIONS_EDGE_BFS",
        "before_points": 1131,
        "before_facets": 2314,
        "after_points": 1131,
        "after_facets": 2314,
    },
}

for _object_name, _spec in MESH_SPECS.items():
    _stem = _object_name + "_mass_properties"
    _spec["artifact_path"] = f"{ARTIFACT_DIR}/{_stem}.stl"
    _spec["report_path"] = f"{ARTIFACT_DIR}/{_stem}_report.json"

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
    UPPER_A_STL: "67aed62b304cfef3a338b0e59b035426b8e012619c333d032e8304c389352f45",
    UPPER_A_REPORT: "a2b58c9892797b26c4511ff2581e7224d99cd27ca2261cc864252a6e38c21815",
    V1_LEDGER: "13e3470821541d7ae5c3f10223c51339d7d46df0d118a8e4dbeac2480c462d32",
    "V15_15_COM账本_v1.md": "8875ff34ab6c091b2d5de6fff418553b0834a141adf32f4c88aac7d1d916d11a",
    "tools/build_com_ledger_v15_15.py": "3e4d9557d0f7c17232d96ba9c783fe31d9394342e61fde3f72fc7b049f8ae619",
    "tools/validate_com_ledger_v15_15.py": "e76a0b8dc83c6f4d2eb8b2b796a834e755fc598e016e5dc5b1432c265f5c1729",
    "tools/build_upperarm_a_mass_properties_v15_15.py": "1709e23ddd2ce2b9ca9dfbcc592970677992d5aa5ea9fadc84da3d606d50b216",
    "tools/validate_upperarm_a_mass_properties_v15_15.py": "bb521e0b8953fd413f76e5621b99bfc8ce9e87bb9c4be567ca2c6276ea058d73",
}

ALLOWED_CHANGED_PATHS = {
    OUTPUT_JSON,
    OUTPUT_MD,
    "tools/build_com_ledger_v15_15_v2.py",
    "tools/validate_com_ledger_v15_15_v2.py",
    SYNTHETIC_TEST,
    *(spec["artifact_path"] for spec in MESH_SPECS.values()),
    *(spec["report_path"] for spec in MESH_SPECS.values()),
}

LINK_ORDER = ["link2", "link3", "link4", "link5", "link6", "gripper"]
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
EXPECTED_LINK_MASSES = {
    "link2": Decimal("0.7615"),
    "link3": Decimal("1.0900"),
    "link4": Decimal("0.6750"),
    "link5": Decimal("0.5040"),
    "link6": Decimal("0.1250"),
    "gripper": Decimal("0.2960"),
}
EXPECTED_TOTAL_MASS = Decimal("3.4515")
OUTPUT_SOLID_INDICES = {24, 25, 26, 27, 28, 29, 30, 32, 34}
NEUTRAL_SOLID_INDICES = {31}
DERIVED_GO_OBJECTS = {
    "derived:J2A_GO_M8010_output": ("J2_Left_Joint_Motor_GO_M8010_6", OUTPUT_SOLID_INDICES),
    "derived:J2A_GO_M8010_neutral": ("J2_Left_Joint_Motor_GO_M8010_6", NEUTRAL_SOLID_INDICES),
    "derived:J2B_GO_M8010_output": ("J2_Right_Joint_Motor_GO_M8010_6", OUTPUT_SOLID_INDICES),
    "derived:J2B_GO_M8010_neutral": ("J2_Right_Joint_Motor_GO_M8010_6", NEUTRAL_SOLID_INDICES),
}

EPS_VOLUME_MM3 = 1.0e-9
BREP_COM_TOL_MM = 1.0e-6
BREP_VOLUME_TOL_MM3 = 1.0e-6
PATH_VOLUME_REL_TOL = 1.0e-8
PATH_COM_TOL_MM = 1.0e-6
FRAME_TOL = 1.0e-9
ROUNDTRIP_TOL_M = 1.0e-8
MASS_TOL_KG = Decimal("1e-9")


class AuditError(RuntimeError):
    pass


def require(condition: bool, code: str) -> None:
    if not condition:
        raise AuditError(code)


def repo_root(start: Path | None = None) -> Path:
    starts = [(start or Path(__file__)).resolve().parent, Path.cwd().resolve()]
    seen: set[Path] = set()
    for initial in starts:
        for candidate in (initial, *initial.parents):
            if candidate in seen:
                continue
            seen.add(candidate)
            if (candidate / ".git").exists() and (candidate / MASS_LEDGER).is_file():
                return candidate
    raise AuditError("REPO_ROOT_NOT_FOUND")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest().lower()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().lower()


def json_bytes(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")


def git_bytes(root: Path, arguments: list[str]) -> bytes:
    return subprocess.run(
        ["git", "-c", "core.quotepath=false", *arguments],
        cwd=root,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=False,
    ).stdout


def changed_paths(root: Path) -> set[str]:
    tracked = {
        item.decode("utf-8")
        for item in git_bytes(root, ["diff", "--name-only", "-z", SOURCE_COMMIT, "--"]).split(b"\0")
        if item
    }
    untracked = {
        item.decode("utf-8")
        for item in git_bytes(root, ["ls-files", "--others", "--exclude-standard", "-z"]).split(b"\0")
        if item
    }
    return tracked | untracked


def git_guard(root: Path) -> dict[str, Any]:
    branch = git_bytes(root, ["branch", "--show-current"]).decode("utf-8").strip()
    source_tree = git_bytes(root, ["show", "-s", "--format=%T", SOURCE_COMMIT]).decode("ascii").strip()
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD"],
        cwd=root,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        shell=False,
    ).returncode == 0
    paths = changed_paths(root)
    unexpected = sorted(paths - ALLOWED_CHANGED_PATHS)
    require(branch == TARGET_BRANCH, f"WRONG_BRANCH:{branch}")
    require(source_tree == SOURCE_TREE, f"SOURCE_TREE_MISMATCH:{source_tree}")
    require(ancestor, "SOURCE_COMMIT_NOT_ANCESTOR")
    require(not unexpected, "UNEXPECTED_CHANGED_PATHS:" + ",".join(unexpected))
    return {
        "source_branch": SOURCE_BRANCH,
        "source_commit": SOURCE_COMMIT,
        "source_tree": SOURCE_TREE,
        "target_branch": TARGET_BRANCH,
        "current_branch": branch,
        "source_commit_is_ancestor": True,
        "policy": "CURRENT_CHANGED_PATHS_MUST_BE_A_SUBSET_OF_EXACT_11_PATH_ALLOWLIST",
        "exact_allowed_changed_paths": sorted(ALLOWED_CHANGED_PATHS),
        "current_changed_paths_are_subset_of_allowlist": True,
        "unexpected_changed_paths": [],
        "pass": True,
    }


def verify_protected(root: Path) -> list[dict[str, Any]]:
    records = []
    for relative, expected in PROTECTED_HASHES.items():
        path = root / relative
        require(path.is_file(), f"PROTECTED_INPUT_MISSING:{relative}")
        actual = sha256_file(path)
        require(actual == expected, f"PROTECTED_INPUT_HASH_MISMATCH:{relative}:{actual}")
        records.append(
            {"path": relative, "sha256": actual, "hash_mode": "RAW_BYTES", "locked": True}
        )
    return records


def run_synthetic_test(root: Path) -> dict[str, Any]:
    test_path = root / SYNTHETIC_TEST
    require(test_path.is_file(), "SYNTHETIC_TEST_MISSING")
    command = Path(sys.executable).resolve()
    if command.name.lower().startswith("python"):
        candidate = command.with_name("FreeCADCmd.exe")
        if candidate.is_file():
            command = candidate
    if not command.name.lower().startswith("freecad"):
        located = shutil.which("FreeCADCmd.exe") or shutil.which("FreeCADCmd")
        require(located is not None, "FREECADCMD_NOT_FOUND_FOR_SYNTHETIC_TEST")
        command = Path(located)
    console_code = (
        "exec(compile(open('tools/test_mesh_volume_properties_v15_15_v2.py', "
        "encoding='utf-8').read(), 'synthetic_mesh_test', 'exec'))\n"
    )
    result = subprocess.run(
        [str(command), "-c"],
        cwd=root,
        input=console_code,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        shell=False,
        timeout=120,
    )
    output = result.stdout or ""
    require(result.returncode == 0, f"SYNTHETIC_TEST_EXIT:{result.returncode}")
    require("TOTAL PASS" in output, "SYNTHETIC_TEST_PASS_MARKER_MISSING")
    return {
        "path": SYNTHETIC_TEST,
        "sha256": sha256_file(test_path),
        "execution": "FRESH_FREECAD_CONSOLE_SUBPROCESS_BEFORE_CAD_OPEN",
        "executable": command.name,
        "returncode": result.returncode,
        "stdout_sha256": sha256_bytes(output.encode("utf-8")),
        "required_marker": "TOTAL PASS",
        "pass": True,
    }


def xyz(value: Any) -> tuple[float, float, float]:
    value = getattr(value, "Vector", value)
    if isinstance(value, (tuple, list)):
        require(len(value) == 3, "XYZ_SEQUENCE_LENGTH")
        return float(value[0]), float(value[1]), float(value[2])
    return float(value.x), float(value.y), float(value.z)


def finite3(values: Sequence[float]) -> bool:
    return len(values) == 3 and all(math.isfinite(float(value)) for value in values)


def vec_add(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def vec_sub(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def vec_dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(math.fsum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def bbox_from_vertices(vertices: Sequence[Sequence[float]]) -> dict[str, list[float]]:
    require(bool(vertices), "EMPTY_VERTEX_SET")
    low = [min(point[axis] for point in vertices) for axis in range(3)]
    high = [max(point[axis] for point in vertices) for axis in range(3)]
    return {
        "min_mm": low,
        "max_mm": high,
        "size_mm": [high[axis] - low[axis] for axis in range(3)],
    }


def merge_bboxes(boxes: Iterable[dict[str, list[float]]]) -> dict[str, list[float]]:
    items = list(boxes)
    require(bool(items), "EMPTY_BBOX_SET")
    low = [min(item["min_mm"][axis] for item in items) for axis in range(3)]
    high = [max(item["max_mm"][axis] for item in items) for axis in range(3)]
    return {"min_mm": low, "max_mm": high, "size_mm": [high[i] - low[i] for i in range(3)]}


def mesh_triangles(mesh: Any) -> list[tuple[tuple[float, float, float], ...]]:
    return [tuple(xyz(point) for point in facet.Points) for facet in mesh.Facets]


def weld_triangles(
    triangles: Sequence[Sequence[Sequence[float]]],
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    vertices: list[tuple[float, float, float]] = []
    lookup: dict[tuple[float, float, float], int] = {}
    faces: list[tuple[int, int, int]] = []
    for triangle in triangles:
        require(len(triangle) == 3, "NON_TRIANGULAR_FACET")
        face: list[int] = []
        for point in triangle:
            key = tuple(float(value) for value in point)
            require(finite3(key), "NONFINITE_MESH_VERTEX")
            if key not in lookup:
                lookup[key] = len(vertices)
                vertices.append(key)
            face.append(lookup[key])
        require(len(set(face)) == 3, "REPEATED_VERTEX_IN_TRIANGLE")
        faces.append(tuple(face))
    return vertices, faces


def edge_occurrences(
    faces: Sequence[tuple[int, int, int]],
) -> dict[tuple[int, int], list[tuple[int, int, int]]]:
    edges: dict[tuple[int, int], list[tuple[int, int, int]]] = defaultdict(list)
    for face_index, (a, b, c) in enumerate(faces):
        for start, end in ((a, b), (b, c), (c, a)):
            edges[tuple(sorted((start, end)))].append((face_index, start, end))
    return edges


def loose_topology_metrics(triangles: Sequence[Sequence[Sequence[float]]]) -> dict[str, Any]:
    vertices, faces = weld_triangles(triangles)
    edges = edge_occurrences(faces)
    histogram = Counter(len(items) for items in edges.values())
    conflicts = sum(
        len(items) == 2 and (items[0][1], items[0][2]) == (items[1][1], items[1][2])
        for items in edges.values()
    )
    return {
        "vertex_count": len(vertices),
        "facet_count": len(faces),
        "edge_count": len(edges),
        "edge_incidence_histogram": {str(key): histogram[key] for key in sorted(histogram)},
        "orientation_conflict_count": int(conflicts),
        "bbox_source_local": bbox_from_vertices(vertices),
    }


def anchored_signed_properties(
    vertices: Sequence[Sequence[float]],
    faces: Sequence[tuple[int, int, int]],
    require_positive: bool = True,
) -> tuple[float, list[float], list[float]]:
    box = bbox_from_vertices(vertices)
    anchor = [0.5 * (box["min_mm"][axis] + box["max_mm"][axis]) for axis in range(3)]
    volumes: list[float] = []
    moments = [[], [], []]
    for face in faces:
        points = [vec_sub(vertices[index], anchor) for index in face]
        determinant = vec_dot(points[0], vec_cross(points[1], points[2]))
        volume = determinant / 6.0
        volumes.append(volume)
        for axis in range(3):
            moments[axis].append(
                volume * (points[0][axis] + points[1][axis] + points[2][axis]) / 4.0
            )
    total = math.fsum(volumes)
    if require_positive:
        require(math.isfinite(total) and total > EPS_VOLUME_MM3, "NONPOSITIVE_SIGNED_VOLUME")
    else:
        require(math.isfinite(total), "NONFINITE_SIGNED_VOLUME")
        if abs(total) <= EPS_VOLUME_MM3:
            return total, [float("nan")] * 3, anchor
    center = [anchor[axis] + math.fsum(moments[axis]) / total for axis in range(3)]
    require(finite3(center), "NONFINITE_POLYHEDRAL_CENTROID")
    return total, center, anchor


def orient_edge_bfs(
    vertices: Sequence[Sequence[float]], faces: Sequence[tuple[int, int, int]]
) -> tuple[list[tuple[int, int, int]], dict[str, Any]]:
    edges = edge_occurrences(faces)
    bad = [edge for edge, items in edges.items() if len(items) != 2]
    require(not bad, f"EDGE_INCIDENCE_NOT_TWO:{len(bad)}")
    adjacency: dict[int, list[tuple[int, bool]]] = defaultdict(list)
    for items in edges.values():
        left, right = items
        same = (left[1], left[2]) == (right[1], right[2])
        adjacency[left[0]].append((right[0], same))
        adjacency[right[0]].append((left[0], same))
    flips: dict[int, bool] = {}
    component_count = 0
    for start in range(len(faces)):
        if start in flips:
            continue
        component_count += 1
        require(component_count == 1, "MASS_PROPERTIES_SURFACE_DISCONNECTED")
        flips[start] = False
        queue = deque([start])
        while queue:
            current = queue.popleft()
            for neighbour, same in adjacency[current]:
                expected = flips[current] ^ same
                if neighbour in flips:
                    require(flips[neighbour] == expected, "SURFACE_NOT_ORIENTABLE")
                else:
                    flips[neighbour] = expected
                    queue.append(neighbour)
    oriented = [
        (face[0], face[2], face[1]) if flips[index] else face
        for index, face in enumerate(faces)
    ]
    signed_volume, _, _ = anchored_signed_properties(vertices, oriented, require_positive=False)
    global_flip = signed_volume < 0.0
    if global_flip:
        oriented = [(face[0], face[2], face[1]) for face in oriented]
    volume, center, anchor = anchored_signed_properties(vertices, oriented)
    conflicts_after = sum(
        len(items) == 2 and (items[0][1], items[0][2]) == (items[1][1], items[1][2])
        for items in edge_occurrences(oriented).values()
    )
    require(conflicts_after == 0, "ORIENTATION_CONFLICT_REMAINS")
    return oriented, {
        "algorithm": "DETERMINISTIC_UNDIRECTED_EDGE_ADJACENCY_BFS",
        "connected_component_count": component_count,
        "per_face_flip_count": sum(flips.values()),
        "global_flip_to_positive_volume": global_flip,
        "orientation_conflict_count_after": 0,
        "anchor_mm": anchor,
        "signed_volume_mm3": volume,
        "centroid_mm": center,
    }


def canonical_triangles(
    vertices: Sequence[Sequence[float]], faces: Sequence[tuple[int, int, int]]
) -> list[tuple[tuple[float, float, float], ...]]:
    output = []
    for face in faces:
        points = tuple(tuple(float(value) for value in vertices[index]) for index in face)
        rotations = (points, (points[1], points[2], points[0]), (points[2], points[0], points[1]))
        output.append(min(rotations))
    output.sort()
    return output


def triangle_surface_area(triangles: Sequence[Sequence[Sequence[float]]]) -> float:
    areas = []
    for a, b, c in triangles:
        cross = vec_cross(vec_sub(b, a), vec_sub(c, a))
        areas.append(0.5 * math.sqrt(vec_dot(cross, cross)))
    return math.fsum(areas)


def unoriented_triangle_counter(
    triangles: Sequence[Sequence[Sequence[float]]],
) -> Counter[Any]:
    """Count geometric triangles while ignoring winding but retaining duplicates."""

    return Counter(
        tuple(sorted(tuple(float(value) for value in point) for point in triangle))
        for triangle in triangles
    )


def topology_sha256(triangles: Sequence[Sequence[Sequence[float]]]) -> str:
    digest = hashlib.sha256()
    digest.update(struct.pack("<Q", len(triangles)))
    for triangle in triangles:
        digest.update(struct.pack("<9d", *(value for point in triangle for value in point)))
    return digest.hexdigest().lower()


def canonical_binary_stl(
    object_name: str, triangles: Sequence[Sequence[Sequence[float]]]
) -> bytes:
    header_text = f"V15.15 V2 COM-only source-local canonical {object_name}"
    header = header_text.encode("ascii")[:80].ljust(80, b"\0")
    payload = bytearray(header + struct.pack("<I", len(triangles)))
    for triangle in triangles:
        a, b, c = triangle
        normal = vec_cross(vec_sub(b, a), vec_sub(c, a))
        length = math.sqrt(vec_dot(normal, normal))
        require(math.isfinite(length) and length > 0.0, "DEGENERATE_CANONICAL_TRIANGLE")
        unit = [value / length for value in normal]
        payload.extend(
            struct.pack(
                "<12fH",
                *(unit + list(a) + list(b) + list(c)),
                0,
            )
        )
    return bytes(payload)


def placement_matrix(placement: Any) -> list[list[float]]:
    matrix = placement.toMatrix()
    return [
        [float(getattr(matrix, f"A{row}{column}")) for column in range(1, 5)]
        for row in range(1, 5)
    ]


def transform_point(placement: Any, point: Sequence[float]) -> list[float]:
    return list(xyz(placement.multVec(App.Vector(*point))))


def transform_bbox(
    placement: Any, vertices: Sequence[Sequence[float]]
) -> dict[str, list[float]]:
    return bbox_from_vertices([transform_point(placement, point) for point in vertices])


def mesh_diagnostic_only(mesh: Any) -> dict[str, Any]:
    points = [xyz(point) for point in mesh.Points]
    require(bool(points), "MESH_DIAGNOSTIC_EMPTY")
    vertex_mean = [
        math.fsum(point[axis] for point in points) / len(points) for axis in range(3)
    ]
    # These two FreeCAD values are intentionally quarantined here.  No caller
    # receives them as accepted volume properties.
    center = list(xyz(mesh.CenterOfGravity))
    reported_volume = float(mesh.Volume)
    return {
        "freecad_mesh_center_of_gravity_mm": center,
        "vertex_arithmetic_mean_mm": vertex_mean,
        "distance_between_mesh_cog_and_vertex_mean_mm": distance(center, vertex_mean),
        "freecad_mesh_volume_mm3": reported_volume,
        "used_in_mass_formula": False,
        "role": "DIAGNOSTIC_ONLY_KNOWN_VERTEX_DENSITY_DEPENDENT_API",
    }


def mesh_to_part_validity_diagnostic(mesh: Any) -> dict[str, Any]:
    """Probe direct Mesh -> Part validity without accepting any mass property."""

    shape = Part.Shape()
    conversion_succeeded = False
    shell_count = 0
    shape_closed = False
    shape_valid = False
    solid_created = False
    solid_count = 0
    solid_closed = False
    solid_valid = False
    try:
        shape.makeShapeFromMesh(mesh.Topology, 0.001)
        conversion_succeeded = True
        shell_count = len(shape.Shells)
        shape_closed = bool(shape.isClosed())
        shape_valid = bool(shape.isValid())
        if shell_count == 1:
            solid = Part.makeSolid(shape.Shells[0])
            solid_created = True
            solid_count = len(solid.Solids)
            solid_closed = bool(solid.isClosed())
            solid_valid = bool(solid.isValid())
    except Exception:
        # Failure is evidence that a direct valid-solid path is unavailable;
        # exception text is deliberately omitted to keep reports deterministic.
        pass
    direct_valid_closed_solid = (
        conversion_succeeded
        and shell_count == 1
        and shape_closed
        and shape_valid
        and solid_created
        and solid_count == 1
        and solid_closed
        and solid_valid
    )
    return {
        "mesh_to_shape_tolerance_mm": 0.001,
        "conversion_succeeded": conversion_succeeded,
        "shape_shell_count": shell_count,
        "shape_closed": shape_closed,
        "shape_valid": shape_valid,
        "solid_created": solid_created,
        "solid_count": solid_count,
        "solid_closed": solid_closed,
        "solid_valid": solid_valid,
        "direct_valid_closed_solid": direct_valid_closed_solid,
        "repair_required": not direct_valid_closed_solid,
        "role": "REPAIR_DECISION_DIAGNOSTIC_ONLY_NOT_A_MASS_PROPERTY_PATH",
    }


def reimport_and_crosscheck(
    stl_bytes: bytes,
    expected_topology_sha256: str | None,
    expected_points: int,
    expected_facets: int,
) -> dict[str, Any]:
    require(App is not None and Mesh is not None and Part is not None, "FREECAD_RUNTIME_REQUIRED")
    with tempfile.TemporaryDirectory(prefix="v15_15_v2_mesh_reimport_") as temporary:
        path = Path(temporary) / "artifact.stl"
        path.write_bytes(stl_bytes)
        mesh = Mesh.Mesh(str(path))
        mesh_qa = {
            "point_count": int(mesh.CountPoints),
            "facet_count": int(mesh.CountFacets),
            "component_count": int(mesh.countComponents()),
            "is_solid": bool(mesh.isSolid()),
            "has_non_manifolds": bool(mesh.hasNonManifolds()),
            "has_self_intersections": bool(mesh.hasSelfIntersections()),
        }
        require(mesh_qa["point_count"] == expected_points, "ARTIFACT_POINT_COUNT_MISMATCH")
        require(mesh_qa["facet_count"] == expected_facets, "ARTIFACT_FACET_COUNT_MISMATCH")
        require(mesh_qa["component_count"] == 1, "ARTIFACT_COMPONENT_COUNT_MISMATCH")
        require(mesh_qa["is_solid"], "ARTIFACT_MESH_NOT_SOLID")
        require(not mesh_qa["has_non_manifolds"], "ARTIFACT_MESH_NON_MANIFOLD")
        require(not mesh_qa["has_self_intersections"], "ARTIFACT_MESH_SELF_INTERSECTION")

        vertices, faces = weld_triangles(mesh_triangles(mesh))
        oriented, bfs = orient_edge_bfs(vertices, faces)
        canonical = canonical_triangles(vertices, oriented)
        actual_topology_sha = topology_sha256(canonical)
        if expected_topology_sha256 is not None:
            require(
                actual_topology_sha == expected_topology_sha256,
                f"ARTIFACT_TOPOLOGY_HASH_MISMATCH:{actual_topology_sha}",
            )
        path_a_volume, path_a_com, anchor = anchored_signed_properties(vertices, oriented)

        shape = Part.Shape()
        shape.makeShapeFromMesh(mesh.Topology, 0.001)
        require(len(shape.Shells) == 1, "PART_REIMPORT_SHELL_COUNT_MISMATCH")
        require(shape.isClosed(), "PART_REIMPORT_SHAPE_NOT_CLOSED")
        require(shape.isValid(), "PART_REIMPORT_SHAPE_INVALID")
        solid = Part.makeSolid(shape.Shells[0])
        require(len(solid.Solids) == 1, "PART_REIMPORT_SOLID_COUNT_MISMATCH")
        require(solid.isClosed(), "PART_REIMPORT_SOLID_NOT_CLOSED")
        require(solid.isValid(), "PART_REIMPORT_SOLID_INVALID")
        path_b_volume = float(solid.Volume)
        path_b_com = list(xyz(solid.CenterOfGravity))
        relative_volume_error = abs(path_b_volume - path_a_volume) / path_a_volume
        com_error = distance(path_b_com, path_a_com)
        require(relative_volume_error < PATH_VOLUME_REL_TOL, "PATH_A_B_VOLUME_MISMATCH")
        require(com_error < PATH_COM_TOL_MM, "PATH_A_B_COM_MISMATCH")
        diagnostic = mesh_diagnostic_only(mesh)

    return {
        "mesh_qa": mesh_qa,
        "canonical_topology_sha256": actual_topology_sha,
        "path_a_anchored_signed_tetrahedra": {
            "coordinate_frame": "SOURCE_OBJECT_LOCAL",
            "anchor_mm": anchor,
            "signed_volume_mm3": path_a_volume,
            "volume_mm3": path_a_volume,
            "centroid_mm": path_a_com,
            "orientation": bfs,
            "authoritative_for_com": True,
        },
        "path_b_part_reimport": {
            "coordinate_frame": "SOURCE_OBJECT_LOCAL",
            "mesh_to_shape_tolerance_mm": 0.001,
            "shape_shell_count": 1,
            "shape_closed": True,
            "shape_valid": True,
            "solid_count": 1,
            "solid_closed": True,
            "solid_valid": True,
            "volume_mm3": path_b_volume,
            "centroid_mm": path_b_com,
            "crosscheck_only": True,
        },
        "path_a_b_agreement": {
            "relative_volume_error": relative_volume_error,
            "relative_volume_tolerance_strict_less_than": PATH_VOLUME_REL_TOL,
            "centroid_distance_mm": com_error,
            "centroid_tolerance_mm_strict_less_than": PATH_COM_TOL_MM,
            "pass": True,
        },
        "mesh_api_diagnostic": diagnostic,
        "vertices_source_local": vertices,
        "bbox_source_local": bbox_from_vertices(vertices),
    }


def component_signed_volume_without_centroid(mesh: Any) -> float:
    vertices, faces = weld_triangles(mesh_triangles(mesh))
    volume, _, _ = anchored_signed_properties(vertices, faces, require_positive=False)
    return volume


def prepare_repaired_mesh_artifact(root: Path, doc: Any, object_name: str) -> dict[str, Any]:
    spec = MESH_SPECS[object_name]
    obj = doc.getObject(object_name)
    require(obj is not None, f"SOURCE_MESH_OBJECT_MISSING:{object_name}")
    require(obj.TypeId == "Mesh::Feature", f"SOURCE_MESH_TYPE_MISMATCH:{object_name}:{obj.TypeId}")
    working = Mesh.Mesh(obj.Mesh)
    source_placement = working.Placement
    source_placement_matrix = placement_matrix(source_placement)
    before_counts = {
        "point_count": int(working.CountPoints),
        "facet_count": int(working.CountFacets),
        "component_count": int(working.countComponents()),
        "has_non_manifolds": bool(working.hasNonManifolds()),
    }
    require(before_counts["point_count"] == spec["before_points"], f"SOURCE_POINT_COUNT:{object_name}")
    require(before_counts["facet_count"] == spec["before_facets"], f"SOURCE_FACET_COUNT:{object_name}")

    # Mesh point access includes Placement.  Clearing Placement on the copy is
    # the explicit source-local extraction; the original CAD object is untouched.
    working.Placement = App.Placement()
    source_local_before = mesh_triangles(working)
    before_topology = loose_topology_metrics(source_local_before)
    direct_source_part_diagnostic = mesh_to_part_validity_diagnostic(working)
    if object_name == "Wrist_Prelink_v1_HighDetail_Display":
        require(before_counts["component_count"] == 1, "WRIST_DIRECT_COMPONENT_COUNT")
        require(not before_counts["has_non_manifolds"], "WRIST_DIRECT_NON_MANIFOLD")
        require(direct_source_part_diagnostic["shape_closed"], "WRIST_DIRECT_PART_SHAPE_OPEN")
        require(direct_source_part_diagnostic["solid_closed"], "WRIST_DIRECT_PART_SOLID_OPEN")
        require(not direct_source_part_diagnostic["shape_valid"], "WRIST_DIRECT_PART_SHAPE_VALID_UNEXPECTED")
        require(not direct_source_part_diagnostic["solid_valid"], "WRIST_DIRECT_PART_SOLID_VALID_UNEXPECTED")
        require(direct_source_part_diagnostic["repair_required"], "WRIST_REPAIR_NOT_REQUIRED")
    repair_actions: list[str] = []
    excluded_components: list[dict[str, Any]] = []

    if object_name == "UpperArm_B_Distal_PrintPart":
        working.removeNonManifolds()
        repair_actions.append("FREECAD_MESH_REMOVE_NON_MANIFOLDS_ON_WORKING_COPY")
    elif object_name == "Forearm_v3_HighDetail_Display":
        components = list(working.getSeparateComponents())
        facet_counts = sorted(int(component.CountFacets) for component in components)
        require(facet_counts == [2, 2978], f"FOREARM_COMPONENT_FACETS:{facet_counts}")
        retained = [component for component in components if int(component.CountFacets) == 2978]
        excluded = [component for component in components if int(component.CountFacets) == 2]
        require(len(retained) == 1 and len(excluded) == 1, "FOREARM_COMPONENT_SELECTION_FAILURE")
        excluded_mesh = excluded[0]
        excluded_signed_volume = component_signed_volume_without_centroid(excluded_mesh)
        require(abs(excluded_signed_volume) <= EPS_VOLUME_MM3, "FOREARM_EXCLUDED_COMPONENT_NONZERO_VOLUME")
        excluded_topology = loose_topology_metrics(mesh_triangles(excluded_mesh))
        excluded_components.append(
            {
                "facet_count": 2,
                "vertex_count": excluded_topology["vertex_count"],
                "signed_volume_mm3_independent": excluded_signed_volume,
                "reason": "EXACT_REVERSE_DUPLICATE_ZERO_VOLUME_TWO_FACET_COMPONENT",
            }
        )
        working = Mesh.Mesh(retained[0])
        working.Placement = App.Placement()
        repair_actions.append("RETAIN_EXACT_2978_FACET_CONNECTED_COMPONENT")
        repair_actions.append("EXCLUDE_EXACT_2_FACET_ZERO_SIGNED_VOLUME_COMPONENT")

    pre_harmonize_triangles = mesh_triangles(working)
    pre_harmonize_topology = loose_topology_metrics(pre_harmonize_triangles)
    vertices_before_actions = sorted(set(point for triangle in pre_harmonize_triangles for point in triangle))

    if object_name in {
        "Forearm_v3_HighDetail_Display",
        "Wrist_Prelink_v1_HighDetail_Display",
    }:
        working.harmonizeNormals()
        repair_actions.append("FREECAD_MESH_HARMONIZE_NORMALS_ON_WORKING_COPY")
        post_harmonize_topology = loose_topology_metrics(mesh_triangles(working))
        require(
            post_harmonize_topology["orientation_conflict_count"] == 0,
            f"HARMONIZE_NORMALS_CONFLICT_REMAINS:{object_name}",
        )
        working.fixDegenerations()
        repair_actions.append("FREECAD_MESH_FIX_DEGENERATIONS_ON_WORKING_COPY")
        first_fix_triangles = mesh_triangles(working)
        working.fixDegenerations()
        second_fix_triangles = mesh_triangles(working)
        require(first_fix_triangles == second_fix_triangles, f"FIX_DEGENERATIONS_NOT_IDEMPOTENT:{object_name}")
        repair_actions.append("SECOND_FIX_DEGENERATIONS_IDEMPOTENCE_CHECK")
    else:
        post_harmonize_topology = pre_harmonize_topology

    repaired_triangles = mesh_triangles(working)
    after_counts = {
        "point_count": int(working.CountPoints),
        "facet_count": int(working.CountFacets),
        "component_count": int(working.countComponents()),
        "is_solid": bool(working.isSolid()),
        "has_non_manifolds": bool(working.hasNonManifolds()),
        "has_self_intersections": bool(working.hasSelfIntersections()),
    }
    require(after_counts["point_count"] == spec["after_points"], f"REPAIRED_POINT_COUNT:{object_name}")
    require(after_counts["facet_count"] == spec["after_facets"], f"REPAIRED_FACET_COUNT:{object_name}")
    require(after_counts["component_count"] == 1, f"REPAIRED_COMPONENT_COUNT:{object_name}")
    require(after_counts["is_solid"], f"REPAIRED_MESH_NOT_SOLID:{object_name}")
    require(not after_counts["has_non_manifolds"], f"REPAIRED_MESH_NON_MANIFOLD:{object_name}")
    require(not after_counts["has_self_intersections"], f"REPAIRED_MESH_SELF_INTERSECTION:{object_name}")

    vertices_after_actions = sorted(set(point for triangle in repaired_triangles for point in triangle))
    if object_name != "UpperArm_B_Distal_PrintPart":
        require(vertices_before_actions == vertices_after_actions, f"REPAIR_MOVED_OR_REMOVED_VERTEX:{object_name}")

    before_bbox = before_topology["bbox_source_local"]
    after_loose = loose_topology_metrics(repaired_triangles)
    after_bbox = after_loose["bbox_source_local"]
    bbox_delta = [after_bbox["size_mm"][axis] - before_bbox["size_mm"][axis] for axis in range(3)]
    retained_vertex_max_displacement = 0.0
    source_triangle_counter = unoriented_triangle_counter(source_local_before)
    selected_triangle_counter = unoriented_triangle_counter(pre_harmonize_triangles)
    repaired_triangle_counter = unoriented_triangle_counter(repaired_triangles)
    source_to_artifact_removed = sum(
        (source_triangle_counter - repaired_triangle_counter).values()
    )
    source_to_artifact_added = sum(
        (repaired_triangle_counter - source_triangle_counter).values()
    )
    retained_retriangulation_removed = sum(
        (selected_triangle_counter - repaired_triangle_counter).values()
    )
    retained_retriangulation_added = sum(
        (repaired_triangle_counter - selected_triangle_counter).values()
    )
    excluded_component_facets = sum(
        int(item["facet_count"]) for item in excluded_components
    )
    require(
        before_counts["facet_count"] - source_to_artifact_removed
        + source_to_artifact_added
        == after_counts["facet_count"],
        f"SOURCE_ARTIFACT_FACET_ACCOUNTING:{object_name}",
    )
    require(
        len(pre_harmonize_triangles) - retained_retriangulation_removed
        + retained_retriangulation_added
        == after_counts["facet_count"],
        f"RETAINED_RETRIANGULATION_FACET_ACCOUNTING:{object_name}",
    )
    retained_facets_exact_source_subset = not bool(
        repaired_triangle_counter - source_triangle_counter
    )
    area_before_selected = triangle_surface_area(pre_harmonize_triangles)
    area_after = triangle_surface_area(repaired_triangles)
    area_delta = area_after - area_before_selected
    require(abs(area_delta) < 1.0e-8, f"REPAIR_SURFACE_AREA_CHANGED:{object_name}:{area_delta}")
    if object_name == "UpperArm_B_Distal_PrintPart":
        require(retained_facets_exact_source_subset, f"UPPER_B_RETAINED_FACET_NOT_SOURCE_SUBSET")
        fidelity_method = "EVERY_RETAINED_CANONICAL_FACET_IS_AN_EXACT_SOURCE_TRIANGLE;NO_VERTEX_MOVEMENT"
    else:
        fidelity_method = (
            "ZERO_VERTEX_MOVEMENT_AND_ZERO_BBOX_OR_AREA_CHANGE;FIX_DEGENERATIONS_IS_"
            "COPLANAR_LOCAL_RETRIANGULATION"
        )

    vertices, faces = weld_triangles(repaired_triangles)
    oriented_faces, bfs = orient_edge_bfs(vertices, faces)
    canonical = canonical_triangles(vertices, oriented_faces)
    canonical_sha = topology_sha256(canonical)
    stl_bytes = canonical_binary_stl(object_name, canonical)
    artifact_sha = sha256_bytes(stl_bytes)
    reimported = reimport_and_crosscheck(
        stl_bytes,
        canonical_sha,
        spec["after_points"],
        spec["after_facets"],
    )
    source_path_a_volume, source_path_a_com, source_anchor = anchored_signed_properties(
        vertices, oriented_faces
    )
    require(
        abs(source_path_a_volume - reimported["path_a_anchored_signed_tetrahedra"]["volume_mm3"])
        / source_path_a_volume
        < PATH_VOLUME_REL_TOL,
        f"SOURCE_TO_ARTIFACT_VOLUME_MISMATCH:{object_name}",
    )
    require(
        distance(source_path_a_com, reimported["path_a_anchored_signed_tetrahedra"]["centroid_mm"])
        < PATH_COM_TOL_MM,
        f"SOURCE_TO_ARTIFACT_COM_MISMATCH:{object_name}",
    )

    local_center = reimported["path_a_anchored_signed_tetrahedra"]["centroid_mm"]
    world_center = transform_point(source_placement, local_center)
    world_bbox = transform_bbox(source_placement, reimported["vertices_source_local"])
    report = {
        "schema": "go-m8010-arm-v15.15-mass-properties-geometry-v2/1.0",
        "revision": "V15.15-COM_LEDGER_VOLUME_V2",
        "status": "PASS",
        "scope": "COM_ONLY_NO_INERTIA_NO_DYNAMICS",
        "source_authority": {
            "source_branch": SOURCE_BRANCH,
            "source_commit": SOURCE_COMMIT,
            "source_tree": SOURCE_TREE,
            "path": SOURCE_CAD,
            "sha256": PROTECTED_HASHES[SOURCE_CAD],
            "object": object_name,
            "object_type": obj.TypeId,
            "opened_read_only": True,
            "modified_or_saved": False,
            "source_placement_matrix_row_major": source_placement_matrix,
        },
        "repair": {
            "working_copy_only": True,
            "decision_was_explicit_not_silent": True,
            "direct_source_part_crosscheck_before_any_working_copy_repair": (
                direct_source_part_diagnostic
            ),
            "method": spec["repair"],
            "actions_in_order": repair_actions + ["INDEPENDENT_DETERMINISTIC_EDGE_BFS_ORIENTATION"],
            "before": before_counts,
            "before_topology_source_local": before_topology,
            "selected_component_topology_before_harmonize": pre_harmonize_topology,
            "topology_after_harmonize_before_fix": post_harmonize_topology,
            "after": after_counts,
            "facet_accounting_semantics": (
                "UNORIENTED_TRIANGLE_MULTISET_COUNTS;REMOVED_AND_ADDED_ARE_"
                "SOURCE_WORKING_COPY_TO_REPAIRED_ARTIFACT_GEOMETRY"
            ),
            "removed_facet_count": source_to_artifact_removed,
            "added_facet_count": source_to_artifact_added,
            "excluded_zero_volume_component_facet_count": excluded_component_facets,
            "retained_component_retriangulation_removed_facet_count": (
                retained_retriangulation_removed
            ),
            "retained_component_retriangulation_added_facet_count": (
                retained_retriangulation_added
            ),
            "bbox_before_source_local_mm": before_bbox,
            "bbox_after_source_local_mm": after_bbox,
            "bbox_size_delta_mm": bbox_delta,
            "excluded_components": excluded_components,
            "retained_vertices_moved": False,
            "surface_fidelity": {
                "method": fidelity_method,
                "retained_canonical_facets_are_exact_subset_of_source_triangles": retained_facets_exact_source_subset,
                "max_retained_vertex_displacement_mm": retained_vertex_max_displacement,
                "selected_surface_area_before_mm2": area_before_selected,
                "surface_area_after_mm2": area_after,
                "surface_area_delta_mm2": area_delta,
                "pass": True,
            },
            "collision_proxy_used": False,
            "edge_bfs": bfs,
            "canonical_topology_sha256": canonical_sha,
        },
        "artifact": {
            "path": spec["artifact_path"],
            "format": "CANONICAL_BINARY_STL",
            "coordinate_frame": "SOURCE_OBJECT_LOCAL",
            "sha256": artifact_sha,
            "size_bytes": len(stl_bytes),
            "fixed_header": True,
            "canonical_face_order": "LEXICOGRAPHIC_AFTER_CYCLIC_MINIMUM",
        },
        "source_working_copy_path_a": {
            "anchor_mm": source_anchor,
            "volume_mm3": source_path_a_volume,
            "centroid_mm_source_local": source_path_a_com,
        },
        "artifact_reimport": {
            key: value for key, value in reimported.items() if key != "vertices_source_local"
        },
        "accepted_volume_mm3": reimported["path_a_anchored_signed_tetrahedra"]["volume_mm3"],
        "accepted_centroid_mm_source_local": local_center,
        "accepted_centroid_mm_world_at_mechanical_zero": world_center,
        "bbox_mm_source_local": reimported["bbox_source_local"],
        "bbox_mm_world_at_mechanical_zero": world_bbox,
        "algorithm_contract": {
            "accepted_formula": "ANCHORED_SIGNED_TETRAHEDRA_WITH_MATH_FSUM",
            "mesh_center_of_gravity_used_in_mass_formula": False,
            "mesh_volume_used_in_mass_formula": False,
            "part_center_of_gravity_role": "INDEPENDENT_CROSSCHECK_ONLY",
        },
        "validation": {
            "source_hash_locked": True,
            "source_to_artifact_path_a_pass": True,
            "artifact_mesh_closed_manifold_self_intersection_free": True,
            "part_closed_and_valid": True,
            "path_a_b_pass": True,
            "finite_positive_volume_and_centroid": True,
            "pass": True,
        },
        "prohibited_outputs": {
            "inertia_computed_or_written": False,
            "gravity_or_dynamics_computed_or_written": False,
            "urdf_or_mjcf_inertial_written": False,
            "visual_or_collision_geometry_modified": False,
            "kinematics_tf_or_control_modified": False,
        },
    }
    report_bytes = json_bytes(report)
    return {
        "object_name": object_name,
        "artifact_path": spec["artifact_path"],
        "artifact_bytes": stl_bytes,
        "artifact_sha256": artifact_sha,
        "report_path": spec["report_path"],
        "report_bytes": report_bytes,
        "report_sha256": sha256_bytes(report_bytes),
        "report": report,
        "volume_mm3": report["accepted_volume_mm3"],
        "center_source_local_mm": local_center,
        "center_world_mm": world_center,
        "bbox_world_mm": world_bbox,
        "placement_matrix_row_major": source_placement_matrix,
        "source_type_id": obj.TypeId,
    }


def validate_upper_a_dual_path(root: Path, doc: Any) -> dict[str, Any]:
    report = json.loads((root / UPPER_A_REPORT).read_text(encoding="utf-8"))
    require(report.get("status") == "PASS", "UPPER_A_REPORT_NOT_PASS")
    stl_bytes = (root / UPPER_A_STL).read_bytes()
    require(sha256_bytes(stl_bytes) == PROTECTED_HASHES[UPPER_A_STL], "UPPER_A_STL_HASH")
    checked = reimport_and_crosscheck(stl_bytes, None, 899, 1834)
    path_a = checked["path_a_anchored_signed_tetrahedra"]
    require(abs(path_a["volume_mm3"] - float(report["volume_mm3"])) < 1.0e-6, "UPPER_A_VOLUME")
    require(distance(path_a["centroid_mm"], report["centroid_mm_source_local"]) < 1.0e-6, "UPPER_A_COM")
    obj = doc.getObject("UpperArm_A_SleeveSide_PrintPart")
    require(obj is not None and obj.TypeId == "Mesh::Feature", "UPPER_A_SOURCE_OBJECT")
    placement = obj.Mesh.Placement
    world_center = transform_point(placement, path_a["centroid_mm"])
    require(distance(world_center, report["centroid_mm_world"]) < 1.0e-6, "UPPER_A_WORLD_COM")
    vertices = checked.pop("vertices_source_local")
    return {
        "object_name": "UpperArm_A_SleeveSide_PrintPart",
        "source_type_id": obj.TypeId,
        "artifact_path": UPPER_A_STL,
        "artifact_sha256": PROTECTED_HASHES[UPPER_A_STL],
        "report_path": UPPER_A_REPORT,
        "report_sha256": PROTECTED_HASHES[UPPER_A_REPORT],
        "coordinate_frame": "SOURCE_OBJECT_LOCAL",
        "source_placement_matrix_row_major": placement_matrix(placement),
        "volume_mm3": path_a["volume_mm3"],
        "center_source_local_mm": path_a["centroid_mm"],
        "center_world_mm": world_center,
        "bbox_world_mm": transform_bbox(placement, vertices),
        "artifact_reimport_dual_path": checked,
        "protected_existing_artifact_reused_without_repair": True,
    }


def load_authorities(root: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    mass = json.loads((root / MASS_LEDGER).read_text(encoding="utf-8"), parse_float=Decimal)
    v1 = json.loads((root / V1_LEDGER).read_text(encoding="utf-8"))
    manifest = json.loads((root / MANIFEST).read_text(encoding="utf-8"))
    require(mass.get("final_status") == "V15.15 MASS_LEDGER_V1 = PASS", "MASS_LEDGER_NOT_PASS")
    require(v1.get("final_status") == "V15.15 COM_LEDGER_V1 = PASS", "V1_COM_LEDGER_NOT_PASS")
    require(mass.get("unresolved_items") == [], "MASS_LEDGER_UNRESOLVED")
    require(v1.get("unresolved_items") == [], "V1_COM_LEDGER_UNRESOLVED")
    additive = mass["component_to_link_mapping"]["additive_components"]
    ids = [item["component_id"] for item in additive]
    require(len(ids) == 17 and set(ids) == set(COMPONENT_ORDER), "MASS_COMPONENT_SET_MISMATCH")
    top = {item["component_id"]: item for item in additive}
    embedded = {
        item["component_id"]: item
        for link in LINK_ORDER
        for item in mass["link_mass_ledger"][link]["components"]
    }
    require(top == embedded, "MASS_LEDGER_EMBEDDED_COMPONENT_MISMATCH")
    for link, expected in EXPECTED_LINK_MASSES.items():
        actual = Decimal(str(mass["link_mass_ledger"][link]["nominal_mass_kg"]))
        require(abs(actual - expected) <= MASS_TOL_KG, f"LINK_MASS_AUTHORITY:{link}:{actual}")
    total = Decimal(str(mass["total_link2_to_gripper_nominal_mass_kg"]))
    require(abs(total - EXPECTED_TOTAL_MASS) <= MASS_TOL_KG, f"TOTAL_MASS_AUTHORITY:{total}")
    return mass, v1, manifest


def v1_components(v1: dict[str, Any]) -> dict[str, dict[str, Any]]:
    output = {
        component["component_id"]: component
        for link in LINK_ORDER
        for component in v1["links"][link]["components"]
    }
    require(set(output) == set(COMPONENT_ORDER), "V1_COMPONENT_SET_MISMATCH")
    return output


def bbox_from_boundbox(box: Any) -> dict[str, list[float]]:
    return {
        "min_mm": [float(box.XMin), float(box.YMin), float(box.ZMin)],
        "max_mm": [float(box.XMax), float(box.YMax), float(box.ZMax)],
        "size_mm": [float(box.XLength), float(box.YLength), float(box.ZLength)],
    }


def weighted_center(records: Sequence[dict[str, Any]], volume_key: str) -> list[float]:
    total = math.fsum(float(record[volume_key]) for record in records)
    require(math.isfinite(total) and total > EPS_VOLUME_MM3, "NONPOSITIVE_GEOMETRY_WEIGHT")
    return [
        math.fsum(float(record[volume_key]) * record["center_world_mm"][axis] for record in records)
        / total
        for axis in range(3)
    ]


def brep_member(doc: Any, token: str) -> dict[str, Any]:
    require("Collision_Proxy" not in token and not token.endswith("_collision"), f"COLLISION_PROXY:{token}")
    selected_indices: set[int] | None = None
    object_name = token
    if token in DERIVED_GO_OBJECTS:
        object_name, selected_indices = DERIVED_GO_OBJECTS[token]
    obj = doc.getObject(object_name)
    require(obj is not None, f"BREP_OBJECT_MISSING:{token}")
    require(obj.TypeId == "Part::Feature", f"BREP_TYPE_ID:{token}:{obj.TypeId}")
    require(hasattr(obj, "Shape") and not obj.Shape.isNull(), f"BREP_SHAPE_MISSING:{token}")
    all_solids = list(obj.Shape.Solids)
    if selected_indices is not None:
        require(len(all_solids) == 34, f"GO_SOLID_COUNT:{object_name}:{len(all_solids)}")
        solids = [solid for index, solid in enumerate(all_solids, 1) if index in selected_indices]
        require(len(solids) == len(selected_indices), f"GO_SELECTION_INCOMPLETE:{token}")
    else:
        solids = all_solids
    require(bool(solids), f"BREP_NO_SOLIDS:{token}")
    atoms = []
    for source_index, solid in enumerate(solids, 1):
        raw_volume = float(solid.Volume)
        absolute_volume = abs(raw_volume)
        require(absolute_volume > EPS_VOLUME_MM3, f"BREP_ZERO_SOLID:{token}:{source_index}")
        require(solid.isValid(), f"BREP_SOLID_INVALID:{token}:{source_index}")
        require(solid.isClosed(), f"BREP_SOLID_OPEN:{token}:{source_index}")
        center = list(xyz(solid.CenterOfGravity))
        require(finite3(center), f"BREP_COM_NONFINITE:{token}:{source_index}")
        atoms.append(
            {
                "selected_sequence_index": source_index,
                "signed_volume_mm3": raw_volume,
                "abs_volume_mm3": absolute_volume,
                "center_world_mm": center,
                "bbox_world_mm": bbox_from_boundbox(solid.BoundBox),
                "valid": True,
                "closed": True,
            }
        )
    center = weighted_center(atoms, "abs_volume_mm3")
    return {
        "member": token,
        "source_object": object_name,
        "source_type_id": obj.TypeId,
        "source_shape_type": obj.Shape.ShapeType,
        "source_placement_matrix_row_major": placement_matrix(obj.Placement),
        "selected_solid_indices": sorted(selected_indices) if selected_indices else None,
        "solid_count": len(atoms),
        "valid_closed_solid_count": len(atoms),
        "negative_oriented_solid_count": sum(atom["signed_volume_mm3"] < 0.0 for atom in atoms),
        "signed_volume_sum_mm3": math.fsum(atom["signed_volume_mm3"] for atom in atoms),
        "abs_volume_mm3": math.fsum(atom["abs_volume_mm3"] for atom in atoms),
        "center_world_mm": center,
        "bbox_world_mm": merge_bboxes(atom["bbox_world_mm"] for atom in atoms),
        "solid_atoms": atoms,
    }


def brep_component_geometry(
    doc: Any,
    component_id: str,
    members: Sequence[str],
    baseline: dict[str, Any],
) -> dict[str, Any]:
    member_records = [brep_member(doc, member) for member in members]
    volume = math.fsum(record["abs_volume_mm3"] for record in member_records)
    center = weighted_center(member_records, "abs_volume_mm3")
    baseline_center = [float(value) for value in baseline["cad_com_world_mm"]]
    baseline_volume = float(baseline["cad_volume_mm3"])
    com_error = distance(center, baseline_center)
    max_axis_error = max(abs(center[i] - baseline_center[i]) for i in range(3))
    volume_error = abs(volume - baseline_volume)
    require(com_error < BREP_COM_TOL_MM, f"BREP_V1_COM_REGRESSION:{component_id}:{com_error}")
    require(volume_error < BREP_VOLUME_TOL_MM3, f"BREP_V1_VOLUME_REGRESSION:{component_id}:{volume_error}")
    return {
        "geometry_class": "BREP",
        "geometry_method": "PER_CLOSED_VALID_SOLID_ABS_VOLUME_WEIGHTED_BREP_CENTER",
        "volume_mm3": volume,
        "center_world_mm": center,
        "bbox_world_mm": merge_bboxes(record["bbox_world_mm"] for record in member_records),
        "members": member_records,
        "solid_count": sum(record["solid_count"] for record in member_records),
        "negative_oriented_solid_count": sum(
            record["negative_oriented_solid_count"] for record in member_records
        ),
        "v1_zero_delta_crosscheck": {
            "baseline_volume_mm3": baseline_volume,
            "recomputed_volume_mm3": volume,
            "absolute_volume_error_mm3": volume_error,
            "volume_tolerance_mm3_strict_less_than": BREP_VOLUME_TOL_MM3,
            "baseline_center_world_mm": baseline_center,
            "recomputed_center_world_mm": center,
            "centroid_distance_mm": com_error,
            "max_abs_axis_error_mm": max_axis_error,
            "centroid_tolerance_mm_strict_less_than": BREP_COM_TOL_MM,
            "pass": True,
        },
    }


def gripper_typeid_audit(
    doc: Any, manifest: dict[str, Any], gripper_members: Sequence[str]
) -> dict[str, Any]:
    require(len(gripper_members) == 37 and len(set(gripper_members)) == 37, "GRIPPER_MEMBER_COUNT")
    records = []
    compound_count = 0
    solid_shape_count = 0
    total_solids = 0
    negative_count = 0
    for name in gripper_members:
        obj = doc.getObject(name)
        require(obj is not None, f"GRIPPER_MEMBER_MISSING:{name}")
        require(obj.TypeId == "Part::Feature", f"GRIPPER_MEMBER_TYPE:{name}:{obj.TypeId}")
        shape_type = obj.Shape.ShapeType
        require(shape_type in {"Compound", "Solid"}, f"GRIPPER_SHAPE_TYPE:{name}:{shape_type}")
        compound_count += shape_type == "Compound"
        solid_shape_count += shape_type == "Solid"
        solids = list(obj.Shape.Solids)
        require(bool(solids), f"GRIPPER_MEMBER_NO_SOLIDS:{name}")
        for index, solid in enumerate(solids, 1):
            require(solid.isValid(), f"GRIPPER_SOLID_INVALID:{name}:{index}")
            require(solid.isClosed(), f"GRIPPER_SOLID_OPEN:{name}:{index}")
            require(abs(float(solid.Volume)) > EPS_VOLUME_MM3, f"GRIPPER_SOLID_ZERO:{name}:{index}")
            negative_count += float(solid.Volume) < 0.0
        total_solids += len(solids)
        records.append(
            {
                "member": name,
                "type_id": obj.TypeId,
                "shape_type": shape_type,
                "solid_count": len(solids),
                "all_underlying_solids_valid_closed": True,
            }
        )
    require(compound_count == 12 and solid_shape_count == 25, "GRIPPER_SHAPE_CLASS_COUNTS")
    require(total_solids == 56 and negative_count == 0, "GRIPPER_SOLID_AUDIT_COUNTS")

    expected_roles = {
        "left_outer": ["Gripper_Left_Outer_Link"],
        "right_outer": ["Gripper_Right_Outer_Link"],
        "left_drive": ["Gripper_Left_Drive_Link"],
        "right_drive": ["Gripper_Right_Drive_Link"],
        "left_finger": [
            "Gripper_Left_Finger", "Gripper_Hardware_02", "Gripper_Hardware_05",
            "Gripper_Hardware_11", "Gripper_Hardware_13", "Gripper_Hardware_14",
        ],
        "right_finger": [
            "Gripper_Right_Finger", "Gripper_Hardware_01", "Gripper_Hardware_06",
            "Gripper_Hardware_12", "Gripper_Hardware_17", "Gripper_Hardware_18",
            "Gripper_Hardware_19",
        ],
    }
    manifest_roles = manifest["gripper_internal_rigid_roles"]
    require(set(manifest_roles) == set(expected_roles), "GRIPPER_INTERNAL_ROLE_SET")
    for role, expected_members in expected_roles.items():
        require(manifest_roles[role]["members"] == expected_members, f"GRIPPER_INTERNAL_ROLE:{role}")
    policy = manifest["gripper_internal_policy"]
    require(policy["gripper_main_link_contains_only_static_members"], "GRIPPER_STATIC_POLICY")
    require(policy["moving_roles_are_not_merged_into_gripper"], "GRIPPER_OWNER_POLICY")
    controller = doc.getObject("Gripper_Servo_Controller")
    require(controller is not None and hasattr(controller, "ClosureAngle"), "GRIPPER_CONTROLLER")
    closure = float(controller.ClosureAngle)
    require(abs(closure) <= 1.0e-12, f"GRIPPER_CLOSURE_NONZERO:{closure}")
    return {
        "exact_member_order": list(gripper_members),
        "member_count": 37,
        "part_feature_count": 37,
        "mesh_feature_count": 0,
        "compound_count": compound_count,
        "top_level_solid_count": solid_shape_count,
        "underlying_solid_count": total_solids,
        "valid_closed_underlying_solid_count": total_solids,
        "negative_oriented_solid_count": negative_count,
        "closure_angle_deg": closure,
        "member_records": records,
        "internal_rigid_roles": {
            role: {"members": manifest_roles[role]["members"], "retained_true_owner": True}
            for role in expected_roles
        },
        "main_chain_rollup_does_not_reassign_internal_owners": True,
        "pass": True,
    }


def validate_frame(frame: dict[str, Any], link: str) -> tuple[list[float], list[list[float]]]:
    origin = [float(value) for value in frame["origin_mm"]]
    rotation = [[float(value) for value in row] for row in frame["rotation_matrix_row_major"]]
    require(finite3(origin), f"FRAME_ORIGIN:{link}")
    require(len(rotation) == 3 and all(finite3(row) for row in rotation), f"FRAME_ROTATION:{link}")
    for left in range(3):
        for right in range(3):
            actual = math.fsum(rotation[row][left] * rotation[row][right] for row in range(3))
            expected = 1.0 if left == right else 0.0
            require(abs(actual - expected) < FRAME_TOL, f"FRAME_ORTHONORMAL:{link}")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    require(abs(determinant - 1.0) < FRAME_TOL, f"FRAME_DETERMINANT:{link}:{determinant}")
    return origin, rotation


def world_to_local(point: Sequence[float], origin: Sequence[float], rotation: Sequence[Sequence[float]]) -> list[float]:
    delta = [point[axis] - origin[axis] for axis in range(3)]
    return [math.fsum(rotation[row][column] * delta[row] for row in range(3)) for column in range(3)]


def local_to_world(point: Sequence[float], origin: Sequence[float], rotation: Sequence[Sequence[float]]) -> list[float]:
    return [
        origin[row] + math.fsum(rotation[row][column] * point[column] for column in range(3))
        for row in range(3)
    ]


def bbox_world_to_local(
    box: dict[str, list[float]], origin: Sequence[float], rotation: Sequence[Sequence[float]]
) -> dict[str, list[float]]:
    low, high = box["min_mm"], box["max_mm"]
    corners = [
        [x, y, z]
        for x in (low[0], high[0])
        for y in (low[1], high[1])
        for z in (low[2], high[2])
    ]
    return bbox_from_vertices([world_to_local(point, origin, rotation) for point in corners])


def point_in_bbox(point: Sequence[float], box: dict[str, list[float]], tolerance: float = 1.0e-6) -> bool:
    return all(
        box["min_mm"][axis] - tolerance <= point[axis] <= box["max_mm"][axis] + tolerance
        for axis in range(3)
    )


def mesh_geometry_record(prepared: dict[str, Any]) -> dict[str, Any]:
    report = prepared["report"]
    path_a = report["artifact_reimport"]["path_a_anchored_signed_tetrahedra"]
    path_b = report["artifact_reimport"]["path_b_part_reimport"]
    return {
        "geometry_class": "MESH_MASS_PROPERTIES_ONLY_REPAIRED_ARTIFACT",
        "geometry_method": "SOURCE_LOCAL_CANONICAL_STL_ANCHORED_SIGNED_TETRAHEDRA",
        "volume_mm3": prepared["volume_mm3"],
        "center_world_mm": prepared["center_world_mm"],
        "bbox_world_mm": prepared["bbox_world_mm"],
        "members": [
            {
                "member": prepared["object_name"],
                "source_type_id": prepared["source_type_id"],
                "source_placement_matrix_row_major": prepared["placement_matrix_row_major"],
                "artifact_path": prepared["artifact_path"],
                "artifact_sha256": prepared["artifact_sha256"],
                "report_path": prepared["report_path"],
                "report_sha256": prepared["report_sha256"],
                "coordinate_frame": "SOURCE_OBJECT_LOCAL",
                "repair": report["repair"],
                "path_a_anchored_signed_tetrahedra": path_a,
                "path_b_part_reimport_crosscheck": path_b,
                "path_a_b_agreement": report["artifact_reimport"]["path_a_b_agreement"],
                "mesh_cog_diagnostic": report["artifact_reimport"]["mesh_api_diagnostic"],
            }
        ],
        "generated_artifact": {
            "path": prepared["artifact_path"],
            "sha256": prepared["artifact_sha256"],
            "report_path": prepared["report_path"],
            "report_sha256": prepared["report_sha256"],
            "canonical_topology_sha256": report["repair"]["canonical_topology_sha256"],
        },
    }


def upper_arm_geometry(upper_a: dict[str, Any], upper_b: dict[str, Any]) -> dict[str, Any]:
    volume_a = float(upper_a["volume_mm3"])
    volume_b = float(upper_b["volume_mm3"])
    total_volume = volume_a + volume_b
    center = [
        (
            volume_a * upper_a["center_world_mm"][axis]
            + volume_b * upper_b["center_world_mm"][axis]
        )
        / total_volume
        for axis in range(3)
    ]
    total_mass = 0.5515
    mass_a = total_mass * volume_a / total_volume
    mass_b = total_mass * volume_b / total_volume
    v1_mass_a = 0.3715953060813982
    v1_mass_b = 0.1799046939186018
    delta_mass_a = mass_a - v1_mass_a
    delta_mass_b = mass_b - v1_mass_b
    comparison_tolerance_kg = 1.0e-12
    comparison_status = (
        "SAME"
        if max(abs(delta_mass_a), abs(delta_mass_b)) <= comparison_tolerance_kg
        else "CHANGED_WITH_FROZEN_TOTAL"
    )
    require(abs(v1_mass_a + v1_mass_b - total_mass) < 1.0e-12, "UPPER_V1_SPLIT_SUM")
    upper_b_member = mesh_geometry_record(upper_b)["members"][0]
    return {
        "geometry_class": "MIXED_PROTECTED_AND_GENERATED_MASS_PROPERTIES_ARTIFACTS",
        "geometry_method": "EQUAL_EFFECTIVE_DENSITY_VOLUME_WEIGHTED_A_PLUS_B",
        "volume_mm3": total_volume,
        "center_world_mm": center,
        "bbox_world_mm": merge_bboxes([upper_a["bbox_world_mm"], upper_b["bbox_world_mm"]]),
        "members": [
            {
                "member": upper_a["object_name"],
                "source_type_id": upper_a["source_type_id"],
                "artifact_path": upper_a["artifact_path"],
                "artifact_sha256": upper_a["artifact_sha256"],
                "report_path": upper_a["report_path"],
                "report_sha256": upper_a["report_sha256"],
                "coordinate_frame": upper_a["coordinate_frame"],
                "source_placement_matrix_row_major": upper_a["source_placement_matrix_row_major"],
                "volume_mm3": volume_a,
                "center_source_local_mm": upper_a["center_source_local_mm"],
                "center_world_mm": upper_a["center_world_mm"],
                "artifact_reimport_dual_path": upper_a["artifact_reimport_dual_path"],
                "protected_existing_artifact_reused_without_repair": True,
            },
            upper_b_member,
        ],
        "equal_density_volume_allocation": {
            "assumption": "SAME_PRINT_MATERIAL_UNIFORM_EFFECTIVE_DENSITY_INHERITED_FROM_V1",
            "frozen_total_measured_mass_kg": total_mass,
            "total_volume_mm3": total_volume,
            "parts": [
                {
                    "member": upper_a["object_name"],
                    "volume_mm3": volume_a,
                    "volume_fraction": volume_a / total_volume,
                    "allocated_mass_kg": mass_a,
                    "v1_allocated_mass_kg": v1_mass_a,
                    "delta_from_v1_allocated_mass_kg": delta_mass_a,
                    "center_world_mm": upper_a["center_world_mm"],
                },
                {
                    "member": upper_b["object_name"],
                    "volume_mm3": volume_b,
                    "volume_fraction": volume_b / total_volume,
                    "allocated_mass_kg": mass_b,
                    "v1_allocated_mass_kg": v1_mass_b,
                    "delta_from_v1_allocated_mass_kg": delta_mass_b,
                    "center_world_mm": upper_b["center_world_mm"],
                },
            ],
            "allocated_mass_sum_kg": mass_a + mass_b,
            "v1_allocated_mass_sum_kg": v1_mass_a + v1_mass_b,
            "comparison_to_v1": {
                "status": comparison_status,
                "tolerance_kg": comparison_tolerance_kg,
                "reason": (
                    "V2_RECOMPUTED_A_B_VOLUME_FRACTIONS_WITH_FROZEN_0_5515_KG_TOTAL"
                ),
                "max_abs_allocated_mass_delta_kg": max(
                    abs(delta_mass_a), abs(delta_mass_b)
                ),
            },
            "fifty_fifty_used": False,
            "pass": abs(mass_a + mass_b - total_mass) < 1.0e-12,
        },
    }


def build_component_record(
    mass_component: dict[str, Any],
    geometry: dict[str, Any],
    baseline: dict[str, Any],
    frame: dict[str, Any],
) -> dict[str, Any]:
    component_id = mass_component["component_id"]
    owner = mass_component["ledger_link"]
    origin, rotation = validate_frame(frame, owner)
    world = [float(value) for value in geometry["center_world_mm"]]
    local = world_to_local(world, origin, rotation)
    reconstructed = local_to_world(local, origin, rotation)
    roundtrip_m = distance(world, reconstructed) * 0.001
    require(roundtrip_m < ROUNDTRIP_TOL_M, f"COMPONENT_FRAME_ROUNDTRIP:{component_id}")
    require(point_in_bbox(world, geometry["bbox_world_mm"]), f"COMPONENT_COM_OUTSIDE_BBOX:{component_id}")
    baseline_world = [float(value) for value in baseline["cad_com_world_mm"]]
    baseline_local = [float(value) for value in baseline["cad_com_link_mm"]]
    delta_world = [world[axis] - baseline_world[axis] for axis in range(3)]
    delta_local = [local[axis] - baseline_local[axis] for axis in range(3)]
    baseline_volume = float(baseline["cad_volume_mm3"])
    is_brep = geometry["geometry_class"] == "BREP"
    if is_brep:
        require(max(abs(value) for value in delta_world) < BREP_COM_TOL_MM, f"BREP_DELTA:{component_id}")
        gate_class = "ZERO_BREP_REGRESSION"
        defect = "NONE_V1_BREP_VOLUME_CENTER_REMAINS_VALID"
    else:
        gate_class = "EXPECTED_MESH_VOLUME_CENTROID_CORRECTION"
        defect = (
            "V1_USED_FREECAD_MESH_CENTEROFGRAVITY_OR_MESH_VOLUME;"
            "V2_REPLACES_WITH_ANCHORED_POLYHEDRAL_VOLUME_PROPERTIES"
        )
    record = {
        "component_id": component_id,
        "name": mass_component["name"],
        "owner_link": owner,
        "frozen_mass_kg": float(mass_component["nominal_mass_kg"]),
        "mass_status": mass_component["mass_status"],
        "mass_authority": MASS_LEDGER,
        "geometry_class": geometry["geometry_class"],
        "geometry_method": geometry["geometry_method"],
        "geometry_source": SOURCE_CAD,
        "geometry_member_order_inherited_exactly_from_v1_mass_ledger": list(mass_component["cad_members"]),
        "members": geometry["members"],
        "volume_mm3": float(geometry["volume_mm3"]),
        "com_world_mm": world,
        "com_owner_link_mm": local,
        "bbox_world_mm": geometry["bbox_world_mm"],
        "bbox_owner_link_mm": bbox_world_to_local(geometry["bbox_world_mm"], origin, rotation),
        "coordinate_round_trip": {
            "reconstructed_world_mm": reconstructed,
            "error_m": roundtrip_m,
            "tolerance_m_strict_less_than": ROUNDTRIP_TOL_M,
            "pass": True,
        },
        "v1_baseline": {
            "path": V1_LEDGER,
            "volume_mm3": baseline_volume,
            "com_world_mm": baseline_world,
            "com_owner_link_mm": baseline_local,
            "known_defect": defect,
        },
        "v2_delta_from_v1": {
            "volume_mm3": float(geometry["volume_mm3"]) - baseline_volume,
            "com_world_mm": delta_world,
            "com_world_norm_mm": distance(world, baseline_world),
            "com_owner_link_mm": delta_local,
            "com_owner_link_norm_mm": distance(local, baseline_local),
        },
        "regression_gate": {
            "class": gate_class,
            "pass": True,
        },
        "collision_proxy_used": False,
        "blocking": False,
        "notes": mass_component.get("notes", ""),
    }
    if "generated_artifact" in geometry:
        record["generated_artifact"] = geometry["generated_artifact"]
    if "equal_density_volume_allocation" in geometry:
        record["equal_density_volume_allocation"] = geometry["equal_density_volume_allocation"]
    if "v1_zero_delta_crosscheck" in geometry:
        record["v1_zero_delta_crosscheck"] = geometry["v1_zero_delta_crosscheck"]
    return record


def link_record(
    link: str,
    components: Sequence[dict[str, Any]],
    frame: dict[str, Any],
    baseline: dict[str, Any],
) -> dict[str, Any]:
    expected_mass = EXPECTED_LINK_MASSES[link]
    component_mass = sum(Decimal(str(item["frozen_mass_kg"])) for item in components)
    require(abs(component_mass - expected_mass) <= MASS_TOL_KG, f"LINK_COMPONENT_MASS:{link}")
    mass = float(expected_mass)
    world = [
        math.fsum(item["frozen_mass_kg"] * item["com_world_mm"][axis] for item in components) / mass
        for axis in range(3)
    ]
    origin, rotation = validate_frame(frame, link)
    local = world_to_local(world, origin, rotation)
    reconstructed = local_to_world(local, origin, rotation)
    roundtrip_m = distance(world, reconstructed) * 0.001
    require(roundtrip_m < ROUNDTRIP_TOL_M, f"LINK_FRAME_ROUNDTRIP:{link}")
    baseline_world_m = [float(value) for value in baseline["com_xyz_m_world_at_zero"]]
    baseline_local_m = [float(value) for value in baseline["com_xyz_m_in_link_frame"]]
    baseline_world = [value * 1000.0 for value in baseline_world_m]
    baseline_local = [value * 1000.0 for value in baseline_local_m]
    delta_world = [world[i] - baseline_world[i] for i in range(3)]
    delta_local = [local[i] - baseline_local[i] for i in range(3)]
    expected_delta = [
        math.fsum(item["frozen_mass_kg"] * item["v2_delta_from_v1"]["com_world_mm"][axis]
                  for item in components) / mass
        for axis in range(3)
    ]
    propagation_error = distance(delta_world, expected_delta)
    require(propagation_error < 1.0e-9, f"LINK_DELTA_PROPAGATION:{link}:{propagation_error}")
    if link in {"link5", "link6", "gripper"}:
        require(distance(world, baseline_world) < BREP_COM_TOL_MM, f"UNCHANGED_LINK_DELTA:{link}")
    combined_world_bbox = merge_bboxes(item["bbox_world_mm"] for item in components)
    combined_link_bbox = bbox_world_to_local(combined_world_bbox, origin, rotation)
    com_inside_world_bbox = point_in_bbox(world, combined_world_bbox)
    com_inside_link_bbox = point_in_bbox(local, combined_link_bbox)
    require(com_inside_world_bbox and com_inside_link_bbox, f"LINK_COM_OUTSIDE_MASS_COMPONENT_BBOX:{link}")
    if link == "link2":
        delta_reason = "UPPER_ARM_PRINT_MEASURED_VOLUME_CENTROID_CORRECTED_FROM_V1_MESH_VERTEX_MEAN"
    elif link == "link3":
        delta_reason = "FOREARM_PRINT_MEASURED_VOLUME_CENTROID_CORRECTED_FROM_V1_MESH_VERTEX_MEAN"
    elif link == "link4":
        delta_reason = "WRIST_PRELINK_PRINT_MEASURED_VOLUME_CENTROID_CORRECTED_FROM_V1_MESH_VERTEX_MEAN"
    else:
        delta_reason = "UNCHANGED_ALL_MEMBERS_BREP_AND_NO_AFFECTED_SOURCE_MESH_COMPONENT"
    return {
        "mass_kg": mass,
        "component_ids": [item["component_id"] for item in components],
        "component_mass_sum_kg": float(component_mass),
        "frame_world_at_mechanical_zero": frame,
        "com_world_mm": world,
        "com_world_m": [value * 0.001 for value in world],
        "com_link_mm": local,
        "com_link_m": [value * 0.001 for value in local],
        "combined_mass_component_bbox_world_mm": combined_world_bbox,
        "combined_mass_component_bbox_link_mm": combined_link_bbox,
        "v1_baseline": {
            "com_world_m": baseline_world_m,
            "com_link_m": baseline_local_m,
        },
        "v2_delta_from_v1": {
            "com_world_mm": delta_world,
            "com_world_m": [value * 0.001 for value in delta_world],
            "com_world_norm_mm": distance(world, baseline_world),
            "com_link_mm": delta_local,
            "com_link_norm_mm": distance(local, baseline_local),
            "reason": delta_reason,
        },
        "mass_weighted_component_delta_propagation": {
            "expected_world_delta_mm": expected_delta,
            "actual_world_delta_mm": delta_world,
            "error_mm": propagation_error,
            "pass": True,
        },
        "coordinate_round_trip": {
            "reconstructed_world_mm": reconstructed,
            "error_m": roundtrip_m,
            "pass": True,
        },
        "validation": {
            "component_mass_sum_pass": True,
            "all_components_resolved": True,
            "com_finite": finite3(world) and finite3(local),
            "com_inside_combined_mass_component_bbox_world": com_inside_world_bbox,
            "com_inside_combined_mass_component_bbox_link": com_inside_link_bbox,
            "pass": True,
        },
    }


def generated_geometry_hashes(prepared: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    records = []
    for item in prepared:
        records.extend(
            [
                {
                    "path": item["artifact_path"],
                    "sha256": item["artifact_sha256"],
                    "size_bytes": len(item["artifact_bytes"]),
                    "hash_mode": "RAW_BYTES",
                    "role": "SOURCE_LOCAL_CANONICAL_BINARY_STL_COM_ONLY",
                },
                {
                    "path": item["report_path"],
                    "sha256": item["report_sha256"],
                    "size_bytes": len(item["report_bytes"]),
                    "hash_mode": "RAW_BYTES",
                    "role": "MASS_PROPERTIES_REPAIR_AND_DUAL_PATH_REPORT_COM_ONLY",
                },
            ]
        )
    return records


def build_bundle(root: Path | None = None) -> tuple[dict[str, Any], dict[str, bytes]]:
    require(App is not None and Mesh is not None and Part is not None,
            f"FREECAD_RUNTIME_REQUIRED:{FREECAD_IMPORT_ERROR}")
    root = repo_root(root)
    guard = git_guard(root)
    protected_before = verify_protected(root)
    synthetic = run_synthetic_test(root)
    mass_ledger, v1, manifest = load_authorities(root)
    mass_components = {
        item["component_id"]: item
        for item in mass_ledger["component_to_link_mapping"]["additive_components"]
    }
    baseline_components = v1_components(v1)

    source_doc = App.openDocument(str(root / SOURCE_CAD))
    try:
        upper_a = validate_upper_a_dual_path(root, source_doc)
        repaired = {
            object_name: prepare_repaired_mesh_artifact(root, source_doc, object_name)
            for object_name in MESH_SPECS
        }
        geometries: dict[str, dict[str, Any]] = {
            "UPPER_ARM_PRINT_MEASURED": upper_arm_geometry(
                upper_a, repaired["UpperArm_B_Distal_PrintPart"]
            ),
            "FOREARM_PRINT_MEASURED": mesh_geometry_record(
                repaired["Forearm_v3_HighDetail_Display"]
            ),
            "WRIST_PRELINK_PRINT_MEASURED": mesh_geometry_record(
                repaired["Wrist_Prelink_v1_HighDetail_Display"]
            ),
        }
        for component_id in COMPONENT_ORDER:
            if component_id in geometries:
                continue
            authority = mass_components[component_id]
            geometries[component_id] = brep_component_geometry(
                source_doc,
                component_id,
                list(authority["cad_members"]),
                baseline_components[component_id],
            )
        gripper_members = list(
            mass_components["GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP"]["cad_members"]
        )
        gripper_audit = gripper_typeid_audit(source_doc, manifest, gripper_members)
    finally:
        App.closeDocument(source_doc.Name)

    protected_after = verify_protected(root)
    require(protected_before == protected_after, "PROTECTED_INPUT_CHANGED_DURING_BUILD")
    prepared_ordered = [repaired[name] for name in MESH_SPECS]
    generated_hashes = generated_geometry_hashes(prepared_ordered)

    components: list[dict[str, Any]] = []
    for component_id in COMPONENT_ORDER:
        authority = mass_components[component_id]
        owner = authority["ledger_link"]
        record = build_component_record(
            authority,
            geometries[component_id],
            baseline_components[component_id],
            manifest["links"][owner]["frame_world_at_zero"],
        )
        if component_id == "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP":
            record["gripper_typeid_and_internal_owner_audit"] = gripper_audit
        components.append(record)
    component_map = {item["component_id"]: item for item in components}
    links: dict[str, Any] = {}
    for link in LINK_ORDER:
        link_components = [item for item in components if item["owner_link"] == link]
        links[link] = link_record(
            link,
            link_components,
            manifest["links"][link]["frame_world_at_zero"],
            v1["links"][link],
        )

    total_mass = sum(Decimal(str(links[link]["mass_kg"])) for link in LINK_ORDER)
    require(abs(total_mass - EXPECTED_TOTAL_MASS) <= MASS_TOL_KG, "CHAIN_TOTAL_MASS")
    chain_mass = float(total_mass)
    chain_world = [
        math.fsum(links[link]["mass_kg"] * links[link]["com_world_mm"][axis] for link in LINK_ORDER)
        / chain_mass
        for axis in range(3)
    ]
    chain_world_from_leaf_components = [
        math.fsum(
            component["frozen_mass_kg"] * component["com_world_mm"][axis]
            for component in components
        )
        / chain_mass
        for axis in range(3)
    ]
    leaf_vs_six_link_error_m = distance(chain_world_from_leaf_components, chain_world) * 0.001
    require(leaf_vs_six_link_error_m < 1.0e-10, "WHOLE_CHAIN_LEAF_VS_SIX_LINK_CLOSURE")
    v1_chain_world = [
        math.fsum(links[link]["mass_kg"] * links[link]["v1_baseline"]["com_world_m"][axis]
                  for link in LINK_ORDER)
        / chain_mass
        * 1000.0
        for axis in range(3)
    ]
    chain_delta = [chain_world[axis] - v1_chain_world[axis] for axis in range(3)]
    expected_chain_delta = [
        math.fsum(
            component["frozen_mass_kg"] * component["v2_delta_from_v1"]["com_world_mm"][axis]
            for component in components
        )
        / chain_mass
        for axis in range(3)
    ]
    chain_propagation_error = distance(chain_delta, expected_chain_delta)
    require(chain_propagation_error < 1.0e-9, "CHAIN_DELTA_PROPAGATION")

    brep_components = [item for item in components if item["geometry_class"] == "BREP"]
    require(len(brep_components) == 14, "BREP_COMPONENT_COUNT")
    max_brep_com_error = max(
        item["v1_zero_delta_crosscheck"]["centroid_distance_mm"] for item in brep_components
    )
    max_brep_axis_error = max(
        item["v1_zero_delta_crosscheck"]["max_abs_axis_error_mm"] for item in brep_components
    )
    max_brep_volume_error = max(
        item["v1_zero_delta_crosscheck"]["absolute_volume_error_mm3"] for item in brep_components
    )
    component_roundtrip_max = max(
        item["coordinate_round_trip"]["error_m"] for item in components
    )
    link_roundtrip_max = max(links[link]["coordinate_round_trip"]["error_m"] for link in LINK_ORDER)

    document = {
        "schema": SCHEMA,
        "revision": REVISION,
        "scope": SCOPE,
        "final_status": "V15.15 COM_LEDGER_V2 = PASS",
        "provenance": {
            "source_branch": SOURCE_BRANCH,
            "source_commit": SOURCE_COMMIT,
            "source_tree": SOURCE_TREE,
            "target_branch": TARGET_BRANCH,
            "runtime": {
                "python": sys.version.split()[0],
                "freecad": ".".join(App.Version()[:3]),
            },
        },
        "git_scope_guard": guard,
        "authorities": {
            "mass": {
                "path": MASS_LEDGER,
                "sha256": PROTECTED_HASHES[MASS_LEDGER],
                "role": "ONLY_MASS_AND_17_COMPONENT_MEMBERSHIP_AUTHORITY",
            },
            "geometry": {
                "path": SOURCE_CAD,
                "sha256": PROTECTED_HASHES[SOURCE_CAD],
                "role": "OBJECT_GEOMETRY_AND_PLACEMENT_AUTHORITY_OPENED_READ_ONLY",
            },
            "frame": {
                "path": MANIFEST,
                "sha256": PROTECTED_HASHES[MANIFEST],
                "role": "OWNER_LINK_FRAME_AT_ACCEPTED_MECHANICAL_ZERO",
            },
            "v1_regression_baseline": {
                "path": V1_LEDGER,
                "sha256": PROTECTED_HASHES[V1_LEDGER],
                "role": "SUPERSEDED_AUDIT_ARTIFACT_DEFECT_AND_DELTA_BASELINE_ONLY",
                "authority_status": "SUPERSEDED_AUDIT_ARTIFACT",
                "superseded_reason": "FREECAD_MESH_CENTER_OF_GRAVITY_IS_VERTEX_MEAN_NOT_VOLUME_CENTROID",
                "paired_markdown": {
                    "path": "V15_15_COM账本_v1.md",
                    "sha256": PROTECTED_HASHES["V15_15_COM账本_v1.md"],
                    "authority_status": "SUPERSEDED_AUDIT_ARTIFACT",
                    "superseded_reason": "FREECAD_MESH_CENTER_OF_GRAVITY_IS_VERTEX_MEAN_NOT_VOLUME_CENTROID",
                },
            },
            "protected_inputs": protected_before,
            "protected_inputs_after_cad_read": protected_after,
            "generated_geometry_artifacts": generated_hashes,
        },
        "synthetic_mesh_semantics_gate": synthetic,
        "units": {"mass": "kg", "cad_length": "mm", "published_com": "m"},
        "mechanical_zero_condition": {
            "joint_configuration": "ACCEPTED_V15_13_MECHANICAL_ZERO_Q_0",
            "gripper_closure_angle_deg": gripper_audit["closure_angle_deg"],
            "operational_gui_home_not_used_as_mechanical_zero": True,
        },
        "algorithm_contract": {
            "mesh_volume_and_first_moment": "ANCHORED_SIGNED_TETRAHEDRA_FIXED_ORDER_MATH_FSUM",
            "anchor_policy": "SOURCE_LOCAL_BBOX_CENTER",
            "orientation_policy": "FREECAD_REQUIRED_REPAIRS_THEN_INDEPENDENT_EDGE_INCIDENCE_BFS",
            "artifact_policy": "SOURCE_LOCAL_CANONICAL_BINARY_STL_FIXED_HEADER_AND_FACE_ORDER",
            "mesh_center_of_gravity_role": "DIAGNOSTIC_ONLY_USED_IN_NO_MASS_FORMULA",
            "mesh_volume_role": "DIAGNOSTIC_ONLY_USED_IN_NO_MASS_FORMULA",
            "path_a_role": "AUTHORITATIVE_ANCHORED_SIGNED_TETRAHEDRA_VOLUME_AND_FIRST_MOMENT",
            "path_b_role": "CLOSED_VALID_PART_SOLID_INDEPENDENT_CROSSCHECK",
            "path_a_b_tolerances": {
                "relative_volume_strict_less_than": PATH_VOLUME_REL_TOL,
                "centroid_distance_mm_strict_less_than": PATH_COM_TOL_MM,
            },
        },
        "component_order": COMPONENT_ORDER,
        "components": components,
        "links": links,
        "chain_summary": {
            "mass_kg": chain_mass,
            "six_link_path": {
                "formula": "SUM_LINK_MASS_TIMES_LINK_COM_DIVIDED_BY_CHAIN_MASS",
                "com_world_mm": chain_world,
                "com_world_m": [value * 0.001 for value in chain_world],
            },
            "leaf_component_path": {
                "formula": "SUM_17_COMPONENT_MASS_TIMES_COMPONENT_COM_DIVIDED_BY_CHAIN_MASS",
                "com_world_mm": chain_world_from_leaf_components,
                "com_world_m": [value * 0.001 for value in chain_world_from_leaf_components],
            },
            "leaf_vs_six_link_closure": {
                "distance_m": leaf_vs_six_link_error_m,
                "tolerance_m_strict_less_than": 1.0e-10,
                "pass": True,
            },
            "com_world_mm": chain_world,
            "com_world_m": [value * 0.001 for value in chain_world],
            "v1_com_world_mm": v1_chain_world,
            "v1_com_world_m": [value * 0.001 for value in v1_chain_world],
            "delta_from_v1_world_mm": chain_delta,
            "delta_from_v1_world_m": [value * 0.001 for value in chain_delta],
            "component_delta_propagation": {
                "expected_delta_world_mm": expected_chain_delta,
                "actual_delta_world_mm": chain_delta,
                "error_mm": chain_propagation_error,
                "pass": True,
            },
        },
        "v1_defect_correction_summary": {
            "defect": "FREECAD_MESH_GET_GRAVITY_POINT_RETURNS_VERTEX_MEAN",
            "superseded_reason": "FREECAD_MESH_CENTER_OF_GRAVITY_IS_VERTEX_MEAN_NOT_VOLUME_CENTROID",
            "v1_json_and_markdown_authority_status": "SUPERSEDED_AUDIT_ARTIFACT",
            "affected_source_mesh_members": [
                "UpperArm_B_Distal_PrintPart",
                "Forearm_v3_HighDetail_Display",
                "Wrist_Prelink_v1_HighDetail_Display",
            ],
            "affected_component_v1_v2_delta": [
                {
                    "component_id": component_id,
                    "source_mesh_member": {
                        "UPPER_ARM_PRINT_MEASURED": "UpperArm_B_Distal_PrintPart",
                        "FOREARM_PRINT_MEASURED": "Forearm_v3_HighDetail_Display",
                        "WRIST_PRELINK_PRINT_MEASURED": "Wrist_Prelink_v1_HighDetail_Display",
                    }[component_id],
                    "v1_com_world_mm": component_map[component_id]["v1_baseline"]["com_world_mm"],
                    "v2_com_world_mm": component_map[component_id]["com_world_mm"],
                    "delta_xyz_mm": component_map[component_id]["v2_delta_from_v1"]["com_world_mm"],
                    "delta_norm_mm": component_map[component_id]["v2_delta_from_v1"]["com_world_norm_mm"],
                }
                for component_id in (
                    "UPPER_ARM_PRINT_MEASURED",
                    "FOREARM_PRINT_MEASURED",
                    "WRIST_PRELINK_PRINT_MEASURED",
                )
            ],
            "additional_affected_mesh_members_found": [],
            "corrected_component_ids": [
                "UPPER_ARM_PRINT_MEASURED",
                "FOREARM_PRINT_MEASURED",
                "WRIST_PRELINK_PRINT_MEASURED",
            ],
            "unchanged_brep_component_count": 14,
            "all_component_and_link_deltas_explicit": True,
        },
        "validation": {
            "required_gates": {
                "synthetic_subprocess_before_cad": synthetic["pass"],
                "protected_hashes_before_after_equal": protected_before == protected_after,
                "exact_17_component_order_mass_membership_inherited": len(components) == 17,
                "generated_artifact_and_report_hashes_recorded": len(generated_hashes) == 6,
                "three_mesh_artifacts_reimport_mesh_qa_pass": True,
                "three_mesh_artifacts_part_closed_valid_pass": True,
                "upper_a_existing_artifact_dual_path_pass": True,
                "all_four_mesh_artifacts_path_a_b_pass": True,
                "brep_component_count_14": len(brep_components) == 14,
                "brep_v1_zero_delta_pass": max_brep_com_error < BREP_COM_TOL_MM,
                "gripper_typeid_and_internal_owner_audit_pass": gripper_audit["pass"],
                "six_link_mass_closure_pass": True,
                "total_mass_3_4515_kg_pass": True,
                "component_world_local_roundtrip_pass": component_roundtrip_max < ROUNDTRIP_TOL_M,
                "link_world_local_roundtrip_pass": link_roundtrip_max < ROUNDTRIP_TOL_M,
                "link_and_chain_delta_propagation_pass": chain_propagation_error < 1.0e-9,
                "whole_chain_leaf_vs_six_link_closure_pass": leaf_vs_six_link_error_m < 1.0e-10,
                "every_link_com_inside_combined_mass_component_bbox": all(
                    links[link]["validation"]["com_inside_combined_mass_component_bbox_world"]
                    and links[link]["validation"]["com_inside_combined_mass_component_bbox_link"]
                    for link in LINK_ORDER
                ),
                "collision_proxy_used_false": True,
                "no_inertia_or_dynamics_outputs": True,
            },
            "max_brep_com_norm_error_from_v1_mm": max_brep_com_error,
            "max_brep_com_axis_error_from_v1_mm": max_brep_axis_error,
            "max_brep_abs_volume_error_from_v1_mm3": max_brep_volume_error,
            "max_component_coordinate_roundtrip_error_m": component_roundtrip_max,
            "max_link_coordinate_roundtrip_error_m": link_roundtrip_max,
            "all_assertions_pass": True,
        },
        "double_count_checks": {
            "additive_component_count": 17,
            "component_ids_unique": len(component_map) == 17,
            "neutral_b6808_separate_mass_count": 0,
            "gripper_rollup_member_count": len(gripper_audit["exact_member_order"]),
            "gripper_internal_moving_owners_preserved": True,
            "pass": True,
        },
        "known_limitations": [
            "Residual CAD centroids omit unavailable cable geometry; no cable geometry is fabricated.",
            "Gripper COM is the measured complete 0.296 kg rollup at ClosureAngle=0, not an internal dynamic allocation.",
            "UpperArm A/B retain the V1 equal-effective-density assumption for their combined measured mass.",
        ],
        "unresolved_items": [],
        "prohibited_outputs": {
            "source_cad_saved_or_modified": False,
            "visual_or_collision_geometry_modified": False,
            "kinematics_tf_or_control_modified": False,
            "urdf_or_mjcf_inertial_written": False,
            "inertia_tensor_computed_or_written": False,
            "gravity_damping_friction_armature_computed_or_written": False,
        },
    }

    files = {
        item["artifact_path"]: item["artifact_bytes"] for item in prepared_ordered
    }
    files.update({item["report_path"]: item["report_bytes"] for item in prepared_ordered})
    files[OUTPUT_JSON] = json_bytes(document)
    files[OUTPUT_MD] = markdown_text(document).encode("utf-8")
    return document, files


def format_vector(values: Sequence[float]) -> str:
    return "[" + ", ".join(f"{float(value):.12g}" for value in values) + "]"


def markdown_text(document: dict[str, Any]) -> str:
    lines = [
        "# V15.15 COM 账本 Volume V2",
        "",
        f"**{document['final_status']}**",
        "",
        "> 本账本仅修正并冻结 link2 → gripper 的质量与 COM。未计算或写入惯量、重力、阻尼、摩擦、armature、URDF/MJCF inertial；未修改 CAD、visual、collision、运动学、TF 或控制链。",
        "",
        "## 权威与范围",
        "",
        f"- source branch: `{document['provenance']['source_branch']}`",
        f"- source commit: `{document['provenance']['source_commit']}`",
        f"- source tree: `{document['provenance']['source_tree']}`",
        f"- target branch: `{document['provenance']['target_branch']}`",
        f"- mass authority: `{document['authorities']['mass']['path']}` / `{document['authorities']['mass']['sha256']}`",
        f"- geometry authority: `{document['authorities']['geometry']['path']}` / `{document['authorities']['geometry']['sha256']}`",
        f"- frame authority: `{document['authorities']['frame']['path']}` / `{document['authorities']['frame']['sha256']}`",
        "- `V15_15_COM账本_v1.json/.md`: **SUPERSEDED_AUDIT_ARTIFACT**",
        "- superseded reason: `FREECAD_MESH_CENTER_OF_GRAVITY_IS_VERTEX_MEAN_NOT_VOLUME_CENTROID`",
        "- V1 defect: `FREECAD_MESH_GET_GRAVITY_POINT_RETURNS_VERTEX_MEAN`",
        "- synthetic non-uniform mesh test: **PASS in a fresh FreeCAD subprocess before CAD open**",
        "- collision proxy used: **NO**",
        "",
        "## Mesh 质量属性修正",
        "",
        "FreeCAD `Mesh.CenterOfGravity` 与 `Mesh.Volume` 仅保留为 diagnostic，未进入任何质量或 COM 公式。权威路径为 anchored signed tetrahedra；Part closed/valid solid 是独立交叉检查。",
        "",
        "| Component | V1 COM world (mm) | V2 COM world (mm) | Delta norm (mm) | V1 defect gate |",
        "|---|---|---|---:|---|",
    ]
    for component_id in (
        "UPPER_ARM_PRINT_MEASURED",
        "FOREARM_PRINT_MEASURED",
        "WRIST_PRELINK_PRINT_MEASURED",
    ):
        component = next(item for item in document["components"] if item["component_id"] == component_id)
        lines.append(
            f"| {component_id} | `{format_vector(component['v1_baseline']['com_world_mm'])}` | "
            f"`{format_vector(component['com_world_mm'])}` | "
            f"{component['v2_delta_from_v1']['com_world_norm_mm']:.12g} | "
            f"{component['regression_gate']['class']} |"
        )
    upper_arm_component = next(
        item
        for item in document["components"]
        if item["component_id"] == "UPPER_ARM_PRINT_MEASURED"
    )
    upper_allocation = upper_arm_component["equal_density_volume_allocation"]
    upper_a_allocation, upper_b_allocation = upper_allocation["parts"]
    lines.extend(
        [
            "",
            "## Upper-arm A/B 等密度解释性质量分配",
            "",
            f"- comparison to V1: **{upper_allocation['comparison_to_v1']['status']}**",
            f"- A: V1 `{upper_a_allocation['v1_allocated_mass_kg']:.16g} kg`; "
            f"V2 `{upper_a_allocation['allocated_mass_kg']:.16g} kg`; "
            f"delta `{upper_a_allocation['delta_from_v1_allocated_mass_kg']:.16g} kg`",
            f"- B: V1 `{upper_b_allocation['v1_allocated_mass_kg']:.16g} kg`; "
            f"V2 `{upper_b_allocation['allocated_mass_kg']:.16g} kg`; "
            f"delta `{upper_b_allocation['delta_from_v1_allocated_mass_kg']:.16g} kg`",
            f"- frozen A+B total: `{upper_allocation['allocated_mass_sum_kg']:.16g} kg`",
            "",
            "## 新生成 source-local canonical 工件",
            "",
            "| Path | SHA256 | Role |",
            "|---|---|---|",
        ]
    )
    for artifact in document["authorities"]["generated_geometry_artifacts"]:
        lines.append(f"| `{artifact['path']}` | `{artifact['sha256']}` | {artifact['role']} |")
    lines.extend(
        [
            "",
            "## Link COM",
            "",
            "| Link | Mass (kg) | COM owner-link (m) | COM world at mechanical zero (m) | Delta from V1 world (mm) |",
            "|---|---:|---|---|---|",
        ]
    )
    for link in LINK_ORDER:
        record = document["links"][link]
        lines.append(
            f"| {link} | {record['mass_kg']:.4f} | `{format_vector(record['com_link_m'])}` | "
            f"`{format_vector(record['com_world_m'])}` | "
            f"`{format_vector(record['v2_delta_from_v1']['com_world_mm'])}` |"
        )
    chain = document["chain_summary"]
    lines.extend(
        [
            "",
            "## 全链闭包",
            "",
            f"- link2 → gripper total mass: `{chain['mass_kg']:.4f} kg`",
            f"- V2 chain COM world: `{format_vector(chain['com_world_m'])} m`",
            f"- V1 chain COM world: `{format_vector(chain['v1_com_world_m'])} m`",
            f"- delta: `{format_vector(chain['delta_from_v1_world_m'])} m`",
            f"- delta propagation closure error: `{chain['component_delta_propagation']['error_mm']:.12g} mm`",
            f"- leaf-component path COM: `{format_vector(chain['leaf_component_path']['com_world_m'])} m`",
            f"- six-link path COM: `{format_vector(chain['six_link_path']['com_world_m'])} m`",
            f"- leaf-vs-six-link closure: `{chain['leaf_vs_six_link_closure']['distance_m']:.12g} m` (< `1e-10 m`)",
            "",
            "## 17-component 质量与 COM",
            "",
            "| # | Component | Owner | Frozen mass (kg) | Geometry | COM world (mm) | Delta norm (mm) |",
            "|---:|---|---|---:|---|---|---:|",
        ]
    )
    for index, component in enumerate(document["components"], 1):
        lines.append(
            f"| {index} | {component['component_id']} | {component['owner_link']} | "
            f"{component['frozen_mass_kg']:.4f} | {component['geometry_class']} | "
            f"`{format_vector(component['com_world_mm'])}` | "
            f"{component['v2_delta_from_v1']['com_world_norm_mm']:.12g} |"
        )
    validation = document["validation"]
    lines.extend(
        [
            "",
            "## 验收",
            "",
            f"- 14 BRep component 对 V1 最大 COM 轴差：`{validation['max_brep_com_axis_error_from_v1_mm']:.12g} mm`",
            f"- 14 BRep component 对 V1 最大 volume 差：`{validation['max_brep_abs_volume_error_from_v1_mm3']:.12g} mm^3`",
            f"- component frame round-trip 最大误差：`{validation['max_component_coordinate_roundtrip_error_m']:.12g} m`",
            f"- link frame round-trip 最大误差：`{validation['max_link_coordinate_roundtrip_error_m']:.12g} m`",
            "- gripper: 37/37 Part::Feature，56/56 underlying solids valid+closed，ClosureAngle=0，六个内部移动 owner 未重挂",
            "- unresolved items: none",
            "- final: **PASS**",
            "",
        ]
    )
    return "\n".join(lines)


def atomic_write_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)


def run(write: bool) -> int:
    root = repo_root()
    document, files = build_bundle(root)
    if write:
        for relative in sorted(files):
            atomic_write_bytes(root / relative, files[relative])
            require(sha256_file(root / relative) == sha256_bytes(files[relative]),
                    f"POST_WRITE_HASH_MISMATCH:{relative}")
            print(f"WROTE {relative}")
        unexpected = changed_paths(root) - ALLOWED_CHANGED_PATHS
        require(not unexpected, "POST_WRITE_UNEXPECTED_PATHS:" + ",".join(sorted(unexpected)))
        print("COM_VOLUME_V2_ARTIFACTS_WRITTEN = PASS")
    else:
        for relative, expected in sorted(files.items()):
            path = root / relative
            require(path.is_file(), f"GENERATED_FILE_MISSING:{relative}")
            actual = path.read_bytes()
            require(actual == expected, f"GENERATED_FILE_NOT_REPRODUCIBLE:{relative}")
        print("COM_VOLUME_V2_ARTIFACTS_REPRODUCIBLE = PASS")
    print(document["final_status"])
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--write", action="store_true", help="atomically write all V2 artifacts")
    mode.add_argument("--check", action="store_true", help="recompute and byte-compare all V2 artifacts")
    arguments = parser.parse_args()
    try:
        return run(write=arguments.write)
    except (
        AuditError,
        OSError,
        KeyError,
        ValueError,
        TypeError,
        subprocess.CalledProcessError,
        subprocess.TimeoutExpired,
    ) as exc:
        print(f"COM_LEDGER_V2_BUILD_ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
