from __future__ import annotations

"""Independently validate the corrected V15.15 COM volume-V2 ledger.

This validator deliberately does not import the V2 builder.  It reads the
frozen authorities, parses every mass-properties STL itself, repeats the
source-mesh working-copy repairs, recomputes all BRep and mesh centroids, and
reconstructs every component, link, and whole-chain COM.  The builder is run
with ``--check`` only after all independent gates have passed, solely as a
byte-reproducibility check.
"""

import hashlib
import json
import math
import struct
import subprocess
import sys
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
SOURCE_BRANCH = "agent/v15-15-com-v1"
SOURCE_COMMIT = "71d77e902cd91bcdbce14500826f5a5fd24f67d3"
SOURCE_TREE = "7993e9b85d15b5934dcf0c3372b2ac12c6a26b67"
TARGET_BRANCH = "agent/v15-15-com-volume-v2"

SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
DERIVED_CAD = "机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd"
MASS_LEDGER = "V15_15_实测质量账本_v1.json"
V1_LEDGER = "V15_15_COM账本_v1.json"
V2_LEDGER = "V15_15_COM账本_v2.json"
V2_MARKDOWN = "V15_15_COM账本_v2.md"
MANIFEST = "rigid_links_v15_13/rigid_link_manifest.json"
MEMBERSHIP = "rigid_links_v15_13/rigid_link_membership.csv"
RIGID_BUILDER = "完整工程_V15_13_交付/tools/build_robot_arm_rigid_links_v15.py"
SYNTHETIC_TEST = "tools/test_mesh_volume_properties_v15_15_v2.py"
V2_BUILDER = "tools/build_com_ledger_v15_15_v2.py"
V2_VALIDATOR = "tools/validate_com_ledger_v15_15_v2.py"

UPPER_A_STL = (
    "mass_properties_geometry_v15_15/"
    "UpperArm_A_SleeveSide_PrintPart_mass_properties.stl"
)
UPPER_A_REPORT = (
    "mass_properties_geometry_v15_15/"
    "UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json"
)
ARTIFACT_DIR = "mass_properties_geometry_v15_15_v2"

MESH_SPECS: dict[str, dict[str, Any]] = {
    "UpperArm_B_Distal_PrintPart": {
        "before_points": 339,
        "before_facets": 696,
        "after_points": 337,
        "after_facets": 694,
        "artifact": f"{ARTIFACT_DIR}/UpperArm_B_Distal_PrintPart_mass_properties.stl",
        "report": f"{ARTIFACT_DIR}/UpperArm_B_Distal_PrintPart_mass_properties_report.json",
    },
    "Forearm_v3_HighDetail_Display": {
        "before_points": 1460,
        "before_facets": 2980,
        "after_points": 1459,
        "after_facets": 2978,
        "artifact": f"{ARTIFACT_DIR}/Forearm_v3_HighDetail_Display_mass_properties.stl",
        "report": f"{ARTIFACT_DIR}/Forearm_v3_HighDetail_Display_mass_properties_report.json",
    },
    "Wrist_Prelink_v1_HighDetail_Display": {
        "before_points": 1131,
        "before_facets": 2314,
        "after_points": 1131,
        "after_facets": 2314,
        "artifact": f"{ARTIFACT_DIR}/Wrist_Prelink_v1_HighDetail_Display_mass_properties.stl",
        "report": f"{ARTIFACT_DIR}/Wrist_Prelink_v1_HighDetail_Display_mass_properties_report.json",
    },
}

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
    V2_LEDGER,
    V2_MARKDOWN,
    V2_BUILDER,
    V2_VALIDATOR,
    SYNTHETIC_TEST,
    *(spec["artifact"] for spec in MESH_SPECS.values()),
    *(spec["report"] for spec in MESH_SPECS.values()),
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
MASS_TOL_KG = Decimal("1e-9")
COM_TOL_MM = 1.0e-6
LEDGER_NUMERIC_TOL_MM = 1.0e-9
VOLUME_REL_TOL = 1.0e-8
PART_COM_TOL_MM = 1.0e-6
ROUNDTRIP_TOL_M = 1.0e-8
CHAIN_TOL_M = 1.0e-10


class ValidationError(RuntimeError):
    pass


def require(condition: bool, code: str) -> None:
    if not condition:
        raise ValidationError(code)


def repo_root() -> Path:
    for initial in (Path(__file__).resolve().parent, Path.cwd().resolve()):
        for candidate in (initial, *initial.parents):
            if (candidate / ".git").exists() and (candidate / MASS_LEDGER).is_file():
                return candidate
    raise ValidationError("REPO_ROOT_NOT_FOUND")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().lower()


def git_output(root: Path, *arguments: str) -> bytes:
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
        value.decode("utf-8")
        for value in git_output(root, "diff", "--name-only", "-z", SOURCE_COMMIT, "--").split(b"\0")
        if value
    }
    untracked = {
        value.decode("utf-8")
        for value in git_output(root, "ls-files", "--others", "--exclude-standard", "-z").split(b"\0")
        if value
    }
    return tracked | untracked


def verify_git_and_protected(root: Path) -> None:
    branch = git_output(root, "branch", "--show-current").decode().strip()
    tree = git_output(root, "show", "-s", "--format=%T", SOURCE_COMMIT).decode().strip()
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD"],
        cwd=root,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        shell=False,
    ).returncode == 0
    require(branch == TARGET_BRANCH, f"WRONG_BRANCH:{branch}")
    require(tree == SOURCE_TREE, f"SOURCE_TREE_MISMATCH:{tree}")
    require(ancestor, "SOURCE_COMMIT_NOT_ANCESTOR")
    actual_paths = changed_paths(root)
    require(actual_paths == ALLOWED_CHANGED_PATHS, "CHANGED_PATH_ALLOWLIST_MISMATCH:" + ",".join(sorted(actual_paths)))
    for relative, expected in PROTECTED_HASHES.items():
        path = root / relative
        require(path.is_file(), f"PROTECTED_FILE_MISSING:{relative}")
        actual = sha256_file(path)
        require(actual == expected, f"PROTECTED_HASH_MISMATCH:{relative}:{actual}")


def run_synthetic_test(root: Path) -> None:
    result = subprocess.run(
        [str(Path(sys.executable).resolve()), str(root / SYNTHETIC_TEST)],
        cwd=root,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=120,
        shell=False,
    )
    require(result.returncode == 0, f"SYNTHETIC_TEST_EXIT:{result.returncode}:{result.stdout[-500:]}")
    require("TOTAL PASS" in result.stdout, "SYNTHETIC_TEST_PASS_MARKER_MISSING")
    require("vertex_mean" in result.stdout, "SYNTHETIC_TEST_VERTEX_MEAN_MISSING")
    require("analytic_volume_centroid" in result.stdout, "SYNTHETIC_TEST_ANALYTIC_CENTROID_MISSING")
    require("polyhedral_volume_centroid" in result.stdout, "SYNTHETIC_TEST_POLY_CENTROID_MISSING")


def xyz(value: Any) -> tuple[float, float, float]:
    value = getattr(value, "Vector", value)
    if isinstance(value, (tuple, list)):
        require(len(value) == 3, "XYZ_SEQUENCE_LENGTH")
        return float(value[0]), float(value[1]), float(value[2])
    return float(value.x), float(value.y), float(value.z)


def finite3(value: Sequence[float]) -> bool:
    return len(value) == 3 and all(math.isfinite(float(item)) for item in value)


def sub(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(math.fsum((float(a[index]) - float(b[index])) ** 2 for index in range(3)))


def bbox(points: Sequence[Sequence[float]]) -> dict[str, list[float]]:
    require(bool(points), "EMPTY_POINT_SET")
    low = [min(point[axis] for point in points) for axis in range(3)]
    high = [max(point[axis] for point in points) for axis in range(3)]
    return {"min_mm": low, "max_mm": high, "size_mm": [high[i] - low[i] for i in range(3)]}


def merge_bboxes(boxes: Iterable[dict[str, list[float]]]) -> dict[str, list[float]]:
    items = list(boxes)
    require(bool(items), "EMPTY_BBOX_SET")
    low = [min(item["min_mm"][axis] for item in items) for axis in range(3)]
    high = [max(item["max_mm"][axis] for item in items) for axis in range(3)]
    return {"min_mm": low, "max_mm": high, "size_mm": [high[i] - low[i] for i in range(3)]}


def bbox_from_boundbox(value: Any) -> dict[str, list[float]]:
    return {
        "min_mm": [float(value.XMin), float(value.YMin), float(value.ZMin)],
        "max_mm": [float(value.XMax), float(value.YMax), float(value.ZMax)],
        "size_mm": [float(value.XLength), float(value.YLength), float(value.ZLength)],
    }


def close_vector(actual: Sequence[float], expected: Sequence[float], tolerance: float, code: str) -> None:
    require(finite3(actual) and finite3(expected), f"{code}:NONFINITE")
    error = max(abs(float(actual[i]) - float(expected[i])) for i in range(3))
    require(error <= tolerance, f"{code}:{error}")


def close_scalar(actual: float, expected: float, tolerance: float, code: str) -> None:
    require(math.isfinite(actual) and math.isfinite(expected), f"{code}:NONFINITE")
    require(abs(actual - expected) <= tolerance, f"{code}:{actual}:{expected}")


def parse_binary_stl(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    """Parse binary STL without FreeCAD or builder assistance."""

    payload = path.read_bytes()
    require(len(payload) >= 84, f"STL_TOO_SHORT:{path.name}")
    count = struct.unpack_from("<I", payload, 80)[0]
    require(len(payload) == 84 + 50 * count, f"STL_BINARY_LENGTH:{path.name}")
    triangles: list[tuple[tuple[float, float, float], ...]] = []
    for index in range(count):
        values = struct.unpack_from("<12fH", payload, 84 + 50 * index)
        normal = values[0:3]
        triangle = (values[3:6], values[6:9], values[9:12])
        require(values[12] == 0, f"STL_ATTRIBUTE_NONZERO:{path.name}:{index}")
        require(all(math.isfinite(value) for value in values[:12]), f"STL_NONFINITE:{path.name}:{index}")
        geometric = cross(sub(triangle[1], triangle[0]), sub(triangle[2], triangle[0]))
        geometric_length = math.sqrt(dot(geometric, geometric))
        normal_length = math.sqrt(dot(normal, normal))
        require(geometric_length > 0.0, f"STL_DEGENERATE_TRIANGLE:{path.name}:{index}")
        require(abs(normal_length - 1.0) < 2.0e-6, f"STL_NORMAL_NOT_UNIT:{path.name}:{index}")
        require(dot(normal, geometric) > 0.0, f"STL_NORMAL_DIRECTION:{path.name}:{index}")
        triangles.append(tuple(tuple(float(value) for value in point) for point in triangle))
    return triangles


def mesh_triangles(mesh: Any) -> list[tuple[tuple[float, float, float], ...]]:
    return [tuple(xyz(point) for point in facet.Points) for facet in mesh.Facets]


def weld(
    triangles: Sequence[Sequence[Sequence[float]]],
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    vertices: list[tuple[float, float, float]] = []
    lookup: dict[tuple[float, float, float], int] = {}
    faces: list[tuple[int, int, int]] = []
    for triangle in triangles:
        require(len(triangle) == 3, "NON_TRIANGLE")
        face = []
        for point in triangle:
            key = tuple(float(value) for value in point)
            require(finite3(key), "NONFINITE_VERTEX")
            if key not in lookup:
                lookup[key] = len(vertices)
                vertices.append(key)
            face.append(lookup[key])
        require(len(set(face)) == 3, "TRIANGLE_REPEATED_VERTEX")
        faces.append(tuple(face))
    return vertices, faces


def edge_map(faces: Sequence[tuple[int, int, int]]) -> dict[tuple[int, int], list[tuple[int, int, int]]]:
    output: dict[tuple[int, int], list[tuple[int, int, int]]] = defaultdict(list)
    for face_index, (a, b, c) in enumerate(faces):
        for start, end in ((a, b), (b, c), (c, a)):
            output[tuple(sorted((start, end)))].append((face_index, start, end))
    return output


def topology_metrics(
    vertices: Sequence[Sequence[float]], faces: Sequence[tuple[int, int, int]]
) -> dict[str, Any]:
    edges = edge_map(faces)
    histogram = Counter(len(value) for value in edges.values())
    conflicts = sum(
        len(value) == 2 and (value[0][1], value[0][2]) == (value[1][1], value[1][2])
        for value in edges.values()
    )
    return {
        "point_count": len(vertices),
        "facet_count": len(faces),
        "edge_count": len(edges),
        "edge_incidence_histogram": dict(histogram),
        "orientation_conflicts": int(conflicts),
    }


def anchored_properties(
    vertices: Sequence[Sequence[float]],
    faces: Sequence[tuple[int, int, int]],
    positive: bool = True,
) -> tuple[float, list[float], list[float]]:
    """Independent signed-tetrahedron volume and first-moment integration."""

    bounds = bbox(vertices)
    anchor = [0.5 * (bounds["min_mm"][axis] + bounds["max_mm"][axis]) for axis in range(3)]
    signed_volumes: list[float] = []
    first_moments: list[list[float]] = [[], [], []]
    for face in faces:
        relative = [sub(vertices[index], anchor) for index in face]
        signed_volume = dot(relative[0], cross(relative[1], relative[2])) / 6.0
        signed_volumes.append(signed_volume)
        for axis in range(3):
            first_moments[axis].append(
                signed_volume
                * (relative[0][axis] + relative[1][axis] + relative[2][axis])
                / 4.0
            )
    total = math.fsum(signed_volumes)
    require(math.isfinite(total), "POLY_VOLUME_NONFINITE")
    if not positive and abs(total) <= EPS_VOLUME_MM3:
        return total, [float("nan")] * 3, anchor
    require(not positive or total > EPS_VOLUME_MM3, f"POLY_VOLUME_NONPOSITIVE:{total}")
    require(abs(total) > EPS_VOLUME_MM3, "POLY_VOLUME_ZERO")
    center = [anchor[axis] + math.fsum(first_moments[axis]) / total for axis in range(3)]
    require(finite3(center), "POLY_CENTROID_NONFINITE")
    return total, center, anchor


def orient_faces(
    vertices: Sequence[Sequence[float]], faces: Sequence[tuple[int, int, int]]
) -> list[tuple[int, int, int]]:
    """Orient one closed connected surface from edge incidence alone."""

    edges = edge_map(faces)
    require(all(len(value) == 2 for value in edges.values()), "SOURCE_EDGE_INCIDENCE_NOT_TWO")
    adjacency: dict[int, list[tuple[int, bool]]] = defaultdict(list)
    for value in edges.values():
        left, right = value
        same_direction = (left[1], left[2]) == (right[1], right[2])
        adjacency[left[0]].append((right[0], same_direction))
        adjacency[right[0]].append((left[0], same_direction))
    flips: dict[int, bool] = {}
    components = 0
    for start in range(len(faces)):
        if start in flips:
            continue
        components += 1
        flips[start] = False
        queue = deque([start])
        while queue:
            current = queue.popleft()
            for neighbour, same_direction in adjacency[current]:
                wanted = flips[current] ^ same_direction
                if neighbour in flips:
                    require(flips[neighbour] == wanted, "SOURCE_SURFACE_NOT_ORIENTABLE")
                else:
                    flips[neighbour] = wanted
                    queue.append(neighbour)
    require(components == 1, f"SOURCE_SURFACE_COMPONENT_COUNT:{components}")
    oriented = [
        (face[0], face[2], face[1]) if flips[index] else face
        for index, face in enumerate(faces)
    ]
    signed_volume, _, _ = anchored_properties(vertices, oriented, positive=False)
    if signed_volume < 0.0:
        oriented = [(face[0], face[2], face[1]) for face in oriented]
    require(topology_metrics(vertices, oriented)["orientation_conflicts"] == 0, "SOURCE_WINDING_CONFLICT")
    anchored_properties(vertices, oriented)
    return oriented


def canonical_coordinate_triangles(
    vertices: Sequence[Sequence[float]],
    faces: Sequence[tuple[int, int, int]],
    float32: bool = False,
) -> list[tuple[tuple[float, float, float], ...]]:
    output = []
    for face in faces:
        points = []
        for vertex_index in face:
            point = tuple(float(value) for value in vertices[vertex_index])
            if float32:
                point = tuple(struct.unpack("<f", struct.pack("<f", value))[0] for value in point)
            points.append(point)
        rotations = (
            tuple(points),
            (points[1], points[2], points[0]),
            (points[2], points[0], points[1]),
        )
        output.append(min(rotations))
    output.sort()
    return output


def unoriented_triangle_counter(
    triangles: Sequence[Sequence[Sequence[float]]],
) -> Counter[Any]:
    """Count geometric triangles while ignoring winding but retaining duplicates."""

    return Counter(
        tuple(sorted(tuple(float(value) for value in point) for point in triangle))
        for triangle in triangles
    )


def triangle_area_sum(triangles: Sequence[Sequence[Sequence[float]]]) -> float:
    return math.fsum(
        0.5 * math.sqrt(dot(normal, normal))
        for triangle in triangles
        for normal in [cross(sub(triangle[1], triangle[0]), sub(triangle[2], triangle[0]))]
    )


def part_crosscheck(path: Path, poly_volume: float, poly_center: Sequence[float]) -> dict[str, Any]:
    imported_mesh = Mesh.Mesh(str(path))
    require(imported_mesh.countComponents() == 1, f"ARTIFACT_MESH_COMPONENTS:{path.name}")
    require(imported_mesh.isSolid(), f"ARTIFACT_MESH_NOT_SOLID:{path.name}")
    require(not imported_mesh.hasNonManifolds(), f"ARTIFACT_MESH_NONMANIFOLD:{path.name}")
    require(not imported_mesh.hasSelfIntersections(), f"ARTIFACT_MESH_SELF_INTERSECTION:{path.name}")
    shape = Part.Shape()
    shape.makeShapeFromMesh(imported_mesh.Topology, 0.001)
    require(len(shape.Shells) == 1, f"ARTIFACT_PART_SHELL_COUNT:{path.name}")
    require(shape.isClosed() and shape.isValid(), f"ARTIFACT_PART_SHAPE_INVALID:{path.name}")
    solid = Part.makeSolid(shape.Shells[0])
    require(len(solid.Solids) == 1, f"ARTIFACT_PART_SOLID_COUNT:{path.name}")
    require(solid.isClosed() and solid.isValid(), f"ARTIFACT_PART_SOLID_INVALID:{path.name}")
    part_volume = float(solid.Volume)
    part_center = list(xyz(solid.CenterOfGravity))
    relative_error = abs(part_volume - poly_volume) / poly_volume
    center_error = distance(part_center, poly_center)
    require(relative_error < VOLUME_REL_TOL, f"PATH_A_B_VOLUME:{path.name}:{relative_error}")
    require(center_error < PART_COM_TOL_MM, f"PATH_A_B_COM:{path.name}:{center_error}")
    return {
        "part_volume_mm3": part_volume,
        "part_center_mm": part_center,
        "relative_volume_error": relative_error,
        "centroid_error_mm": center_error,
    }


def mesh_to_part_validity_diagnostic(mesh: Any) -> dict[str, Any]:
    """Probe direct Mesh -> Part validity without accepting a mass property."""

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


def placement_matrix(placement: Any) -> list[list[float]]:
    matrix = placement.toMatrix()
    return [[float(getattr(matrix, f"A{row}{column}")) for column in range(1, 5)] for row in range(1, 5)]


def transform_point(placement: Any, point: Sequence[float]) -> list[float]:
    return list(xyz(placement.multVec(App.Vector(*point))))


def source_mesh_repair(doc: Any, object_name: str) -> dict[str, Any]:
    """Repeat the documented minimum repair on a detached in-memory copy."""

    spec = MESH_SPECS[object_name]
    source = doc.getObject(object_name)
    require(source is not None and source.TypeId == "Mesh::Feature", f"SOURCE_MESH_TYPE:{object_name}")
    working = Mesh.Mesh(source.Mesh)
    source_placement = working.Placement
    require(int(working.CountPoints) == spec["before_points"], f"SOURCE_POINT_COUNT:{object_name}")
    require(int(working.CountFacets) == spec["before_facets"], f"SOURCE_FACET_COUNT:{object_name}")
    working.Placement = App.Placement()
    source_triangles = mesh_triangles(working)
    direct_source_part_diagnostic = mesh_to_part_validity_diagnostic(working)
    if object_name == "Wrist_Prelink_v1_HighDetail_Display":
        require(working.countComponents() == 1, "WRIST_DIRECT_COMPONENT_COUNT")
        require(not working.hasNonManifolds(), "WRIST_DIRECT_NON_MANIFOLD")
        require(direct_source_part_diagnostic["shape_closed"], "WRIST_DIRECT_PART_SHAPE_OPEN")
        require(direct_source_part_diagnostic["solid_closed"], "WRIST_DIRECT_PART_SOLID_OPEN")
        require(not direct_source_part_diagnostic["shape_valid"], "WRIST_DIRECT_PART_SHAPE_VALID_UNEXPECTED")
        require(not direct_source_part_diagnostic["solid_valid"], "WRIST_DIRECT_PART_SOLID_VALID_UNEXPECTED")
        require(direct_source_part_diagnostic["repair_required"], "WRIST_REPAIR_NOT_REQUIRED")
    excluded: list[dict[str, Any]] = []
    if object_name == "UpperArm_B_Distal_PrintPart":
        working.removeNonManifolds()
    elif object_name == "Forearm_v3_HighDetail_Display":
        components = list(working.getSeparateComponents())
        require(sorted(int(item.CountFacets) for item in components) == [2, 2978], "FOREARM_COMPONENT_SET")
        retained = [item for item in components if int(item.CountFacets) == 2978]
        removed = [item for item in components if int(item.CountFacets) == 2]
        require(len(retained) == len(removed) == 1, "FOREARM_COMPONENT_SELECTION")
        removed_vertices, removed_faces = weld(mesh_triangles(removed[0]))
        removed_volume, _, _ = anchored_properties(removed_vertices, removed_faces, positive=False)
        require(abs(removed_volume) <= EPS_VOLUME_MM3, f"FOREARM_REMOVED_NONZERO:{removed_volume}")
        excluded.append({"facets": 2, "signed_volume_mm3": removed_volume})
        working = Mesh.Mesh(retained[0])
        working.Placement = App.Placement()
    selected_triangles = mesh_triangles(working)
    vertices_before_harmonize = sorted(
        set(point for triangle in selected_triangles for point in triangle)
    )
    if object_name in {"Forearm_v3_HighDetail_Display", "Wrist_Prelink_v1_HighDetail_Display"}:
        working.harmonizeNormals()
        working.fixDegenerations()
        once = mesh_triangles(working)
        working.fixDegenerations()
        require(once == mesh_triangles(working), f"SOURCE_REPAIR_NOT_IDEMPOTENT:{object_name}")
    repaired_triangles = mesh_triangles(working)
    require(int(working.CountPoints) == spec["after_points"], f"REPAIRED_POINT_COUNT:{object_name}")
    require(int(working.CountFacets) == spec["after_facets"], f"REPAIRED_FACET_COUNT:{object_name}")
    require(working.countComponents() == 1 and working.isSolid(), f"REPAIRED_NOT_ONE_SOLID:{object_name}")
    require(not working.hasNonManifolds(), f"REPAIRED_NONMANIFOLD:{object_name}")
    require(not working.hasSelfIntersections(), f"REPAIRED_SELF_INTERSECTION:{object_name}")
    vertices_after = sorted(set(point for triangle in repaired_triangles for point in triangle))
    if object_name != "UpperArm_B_Distal_PrintPart":
        require(vertices_after == vertices_before_harmonize, f"REPAIR_VERTEX_SET_CHANGED:{object_name}")
    vertices, faces = weld(repaired_triangles)
    oriented = orient_faces(vertices, faces)
    source_volume, source_center, source_anchor = anchored_properties(vertices, oriented)
    source_counter = unoriented_triangle_counter(source_triangles)
    selected_counter = unoriented_triangle_counter(selected_triangles)
    repaired_counter = unoriented_triangle_counter(repaired_triangles)
    source_removed = sum((source_counter - repaired_counter).values())
    source_added = sum((repaired_counter - source_counter).values())
    retriangulation_removed = sum((selected_counter - repaired_counter).values())
    retriangulation_added = sum((repaired_counter - selected_counter).values())
    excluded_facets = sum(int(item["facets"]) for item in excluded)
    require(
        len(source_triangles) - source_removed + source_added == len(repaired_triangles),
        f"SOURCE_ARTIFACT_FACET_ACCOUNTING:{object_name}",
    )
    require(
        len(selected_triangles) - retriangulation_removed + retriangulation_added
        == len(repaired_triangles),
        f"RETAINED_RETRIANGULATION_FACET_ACCOUNTING:{object_name}",
    )
    return {
        "source": source,
        "placement": source_placement,
        "placement_matrix": placement_matrix(source_placement),
        "triangles": repaired_triangles,
        "vertices": vertices,
        "faces": oriented,
        "volume_mm3": source_volume,
        "center_local_mm": source_center,
        "anchor_mm": source_anchor,
        "bbox_local_mm": bbox(vertices),
        "surface_area_mm2": triangle_area_sum(repaired_triangles),
        "excluded": excluded,
        "direct_source_part_diagnostic": direct_source_part_diagnostic,
        "facet_accounting": {
            "source_to_artifact_removed": source_removed,
            "source_to_artifact_added": source_added,
            "excluded_zero_volume_component_facets": excluded_facets,
            "retained_retriangulation_removed": retriangulation_removed,
            "retained_retriangulation_added": retriangulation_added,
            "repaired_is_source_triangle_multiset_subset": not bool(
                repaired_counter - source_counter
            ),
        },
    }


def validate_generated_mesh(root: Path, doc: Any, object_name: str) -> dict[str, Any]:
    spec = MESH_SPECS[object_name]
    artifact_path = root / spec["artifact"]
    report_path = root / spec["report"]
    require(artifact_path.is_file(), f"ARTIFACT_MISSING:{object_name}")
    require(report_path.is_file(), f"ARTIFACT_REPORT_MISSING:{object_name}")
    source = source_mesh_repair(doc, object_name)

    triangles = parse_binary_stl(artifact_path)
    vertices, faces = weld(triangles)
    metrics = topology_metrics(vertices, faces)
    require(metrics["point_count"] == spec["after_points"], f"ARTIFACT_POINT_COUNT:{object_name}")
    require(metrics["facet_count"] == spec["after_facets"], f"ARTIFACT_FACET_COUNT:{object_name}")
    require(metrics["edge_incidence_histogram"] == {2: metrics["edge_count"]}, f"ARTIFACT_EDGE_INCIDENCE:{object_name}")
    require(metrics["orientation_conflicts"] == 0, f"ARTIFACT_WINDING:{object_name}")
    poly_volume, poly_center, anchor = anchored_properties(vertices, faces)
    part_result = part_crosscheck(artifact_path, poly_volume, poly_center)

    expected_canonical = canonical_coordinate_triangles(source["vertices"], source["faces"], float32=True)
    actual_canonical = canonical_coordinate_triangles(vertices, faces)
    require(actual_canonical == expected_canonical, f"SOURCE_ARTIFACT_TRIANGLE_SET:{object_name}")
    source_to_artifact_volume_rel = abs(source["volume_mm3"] - poly_volume) / source["volume_mm3"]
    source_to_artifact_com = distance(source["center_local_mm"], poly_center)
    require(source_to_artifact_volume_rel < VOLUME_REL_TOL, f"SOURCE_ARTIFACT_VOLUME:{object_name}")
    require(source_to_artifact_com < PART_COM_TOL_MM, f"SOURCE_ARTIFACT_COM:{object_name}")
    artifact_area = triangle_area_sum(triangles)
    surface_relative_error = abs(artifact_area - source["surface_area_mm2"]) / source["surface_area_mm2"]
    require(surface_relative_error < 1.0e-6, f"SOURCE_ARTIFACT_SURFACE:{object_name}:{surface_relative_error}")

    report = json.loads(report_path.read_text(encoding="utf-8"))
    reject_forbidden_numeric_fields(report)
    require(report["status"] == "PASS", f"REPORT_STATUS:{object_name}")
    require(report["scope"] == "COM_ONLY_NO_INERTIA_NO_DYNAMICS", f"REPORT_SCOPE:{object_name}")
    authority = report["source_authority"]
    require(authority["source_commit"] == SOURCE_COMMIT, f"REPORT_SOURCE_COMMIT:{object_name}")
    require(authority["source_tree"] == SOURCE_TREE, f"REPORT_SOURCE_TREE:{object_name}")
    require(authority["sha256"] == PROTECTED_HASHES[SOURCE_CAD], f"REPORT_SOURCE_HASH:{object_name}")
    require(authority["object"] == object_name and authority["object_type"] == "Mesh::Feature", f"REPORT_OBJECT:{object_name}")
    require(authority["opened_read_only"] is True and authority["modified_or_saved"] is False, f"REPORT_SOURCE_MUTATION:{object_name}")
    require(report["artifact"]["path"] == spec["artifact"], f"REPORT_ARTIFACT_PATH:{object_name}")
    require(report["artifact"]["sha256"] == sha256_file(artifact_path), f"REPORT_ARTIFACT_HASH:{object_name}")
    close_scalar(float(report["accepted_volume_mm3"]), poly_volume, 1.0e-6, f"REPORT_VOLUME:{object_name}")
    close_vector(report["accepted_centroid_mm_source_local"], poly_center, 1.0e-6, f"REPORT_LOCAL_COM:{object_name}")
    reported_placement = authority["source_placement_matrix_row_major"]
    require(len(reported_placement) == 4 and all(len(row) == 4 for row in reported_placement), f"REPORT_PLACEMENT_SHAPE:{object_name}")
    placement_error = max(
        abs(float(reported_placement[row][column]) - float(source["placement_matrix"][row][column]))
        for row in range(4)
        for column in range(4)
    )
    require(placement_error <= 1.0e-12, f"REPORT_PLACEMENT_4X4:{object_name}:{placement_error}")
    algorithm = report["algorithm_contract"]
    require(algorithm["mesh_center_of_gravity_used_in_mass_formula"] is False, f"REPORT_MESH_COG_USED:{object_name}")
    require(algorithm["mesh_volume_used_in_mass_formula"] is False, f"REPORT_MESH_VOLUME_USED:{object_name}")
    require(report["repair"]["working_copy_only"] is True, f"REPORT_REPAIR_NOT_COPY:{object_name}")
    require(report["repair"]["collision_proxy_used"] is False, f"REPORT_COLLISION_PROXY:{object_name}")
    repair = report["repair"]
    require(repair["decision_was_explicit_not_silent"] is True, f"REPORT_REPAIR_DECISION:{object_name}")
    require(
        repair["direct_source_part_crosscheck_before_any_working_copy_repair"]
        == source["direct_source_part_diagnostic"],
        f"REPORT_DIRECT_SOURCE_PART_DIAGNOSTIC:{object_name}",
    )
    accounting = source["facet_accounting"]
    require(
        repair["facet_accounting_semantics"]
        == (
            "UNORIENTED_TRIANGLE_MULTISET_COUNTS;REMOVED_AND_ADDED_ARE_"
            "SOURCE_WORKING_COPY_TO_REPAIRED_ARTIFACT_GEOMETRY"
        ),
        f"REPORT_FACET_ACCOUNTING_SEMANTICS:{object_name}",
    )
    require(
        repair["removed_facet_count"] == accounting["source_to_artifact_removed"],
        f"REPORT_REMOVED_FACETS:{object_name}",
    )
    require(
        repair["added_facet_count"] == accounting["source_to_artifact_added"],
        f"REPORT_ADDED_FACETS:{object_name}",
    )
    require(
        repair["excluded_zero_volume_component_facet_count"]
        == accounting["excluded_zero_volume_component_facets"],
        f"REPORT_EXCLUDED_FACETS:{object_name}",
    )
    require(
        repair["retained_component_retriangulation_removed_facet_count"]
        == accounting["retained_retriangulation_removed"],
        f"REPORT_RETRIANGULATION_REMOVED:{object_name}",
    )
    require(
        repair["retained_component_retriangulation_added_facet_count"]
        == accounting["retained_retriangulation_added"],
        f"REPORT_RETRIANGULATION_ADDED:{object_name}",
    )
    close_vector(
        repair["bbox_after_source_local_mm"]["min_mm"],
        bbox(vertices)["min_mm"],
        1.0e-6,
        f"REPORT_BBOX_MIN:{object_name}",
    )
    close_vector(
        repair["bbox_after_source_local_mm"]["max_mm"],
        bbox(vertices)["max_mm"],
        1.0e-6,
        f"REPORT_BBOX_MAX:{object_name}",
    )
    fidelity = repair["surface_fidelity"]
    require(fidelity["pass"] is True, f"REPORT_SURFACE_FIDELITY_FLAG:{object_name}")
    expected_exact_subset = accounting["repaired_is_source_triangle_multiset_subset"]
    require(
        fidelity["retained_canonical_facets_are_exact_subset_of_source_triangles"]
        is expected_exact_subset,
        f"REPORT_SURFACE_SUBSET:{object_name}",
    )
    if expected_exact_subset:
        require("EXACT_SOURCE_TRIANGLE" in fidelity["method"], f"REPORT_SURFACE_METHOD:{object_name}")
    else:
        require("COPLANAR_LOCAL_RETRIANGULATION" in fidelity["method"], f"REPORT_SURFACE_METHOD:{object_name}")
    close_scalar(float(fidelity["max_retained_vertex_displacement_mm"]), 0.0, 0.0, f"REPORT_VERTEX_DISPLACEMENT:{object_name}")
    close_scalar(float(fidelity["surface_area_after_mm2"]), artifact_area, 1.0e-5, f"REPORT_SURFACE_AREA:{object_name}")
    require(report["prohibited_outputs"]["inertia_computed_or_written"] is False, f"REPORT_INERTIA:{object_name}")
    require(report["prohibited_outputs"]["gravity_or_dynamics_computed_or_written"] is False, f"REPORT_DYNAMICS:{object_name}")

    world_center = transform_point(source["placement"], poly_center)
    world_vertices = [transform_point(source["placement"], point) for point in vertices]
    close_vector(report["accepted_centroid_mm_world_at_mechanical_zero"], world_center, 1.0e-6, f"REPORT_WORLD_COM:{object_name}")
    return {
        "geometry_class": "MESH_MASS_PROPERTIES_ONLY_REPAIRED_ARTIFACT",
        "volume_mm3": poly_volume,
        "center_world_mm": world_center,
        "center_local_mm": poly_center,
        "bbox_world_mm": bbox(world_vertices),
        "artifact_sha256": sha256_file(artifact_path),
        "report_sha256": sha256_file(report_path),
        "poly_anchor_mm": anchor,
        "part": part_result,
        "source_to_artifact_volume_relative_error": source_to_artifact_volume_rel,
        "source_to_artifact_com_error_mm": source_to_artifact_com,
        "surface_relative_error": surface_relative_error,
    }


def validate_upper_a(root: Path, doc: Any) -> dict[str, Any]:
    path = root / UPPER_A_STL
    require(sha256_file(path) == PROTECTED_HASHES[UPPER_A_STL], "UPPER_A_STL_HASH")
    require(sha256_file(root / UPPER_A_REPORT) == PROTECTED_HASHES[UPPER_A_REPORT], "UPPER_A_REPORT_HASH")
    triangles = parse_binary_stl(path)
    vertices, faces = weld(triangles)
    metrics = topology_metrics(vertices, faces)
    require(metrics["point_count"] == 899 and metrics["facet_count"] == 1834, "UPPER_A_COUNTS")
    require(metrics["edge_incidence_histogram"] == {2: metrics["edge_count"]}, "UPPER_A_EDGE_INCIDENCE")
    require(metrics["orientation_conflicts"] == 0, "UPPER_A_WINDING")
    volume, center_local, anchor = anchored_properties(vertices, faces)
    part_result = part_crosscheck(path, volume, center_local)
    report = json.loads((root / UPPER_A_REPORT).read_text(encoding="utf-8"))
    close_scalar(float(report["volume_mm3"]), volume, 1.0e-6, "UPPER_A_REPORT_VOLUME")
    close_vector(report["centroid_mm_source_local"], center_local, 1.0e-6, "UPPER_A_REPORT_LOCAL_COM")
    source = doc.getObject("UpperArm_A_SleeveSide_PrintPart")
    require(source is not None and source.TypeId == "Mesh::Feature", "UPPER_A_SOURCE_TYPE")
    placement = source.Mesh.Placement
    center_world = transform_point(placement, center_local)
    close_vector(report["centroid_mm_world"], center_world, 1.0e-6, "UPPER_A_REPORT_WORLD_COM")
    world_vertices = [transform_point(placement, point) for point in vertices]
    return {
        "geometry_class": "PROTECTED_MESH_MASS_PROPERTIES_ARTIFACT",
        "volume_mm3": volume,
        "center_world_mm": center_world,
        "center_local_mm": center_local,
        "bbox_world_mm": bbox(world_vertices),
        "artifact_sha256": PROTECTED_HASHES[UPPER_A_STL],
        "report_sha256": PROTECTED_HASHES[UPPER_A_REPORT],
        "poly_anchor_mm": anchor,
        "part": part_result,
    }


def weighted_center(records: Sequence[dict[str, Any]], volume_key: str = "volume_mm3") -> list[float]:
    total = math.fsum(float(record[volume_key]) for record in records)
    require(total > EPS_VOLUME_MM3 and math.isfinite(total), "WEIGHTED_CENTER_NONPOSITIVE_VOLUME")
    return [
        math.fsum(float(record[volume_key]) * float(record["center_world_mm"][axis]) for record in records) / total
        for axis in range(3)
    ]


def brep_member(doc: Any, token: str) -> dict[str, Any]:
    require("Collision_Proxy" not in token and not token.endswith("_collision"), f"COLLISION_PROXY_MEMBER:{token}")
    object_name = token
    selected: set[int] | None = None
    if token in DERIVED_GO_OBJECTS:
        object_name, selected = DERIVED_GO_OBJECTS[token]
    obj = doc.getObject(object_name)
    require(obj is not None, f"BREP_MEMBER_MISSING:{token}")
    require(obj.TypeId == "Part::Feature", f"BREP_MEMBER_TYPE:{token}:{obj.TypeId}")
    require(hasattr(obj, "Shape") and not obj.Shape.isNull(), f"BREP_SHAPE_MISSING:{token}")
    all_solids = list(obj.Shape.Solids)
    if selected is not None:
        require(len(all_solids) == 34, f"GO_SOLID_COUNT:{object_name}:{len(all_solids)}")
        solids = [solid for index, solid in enumerate(all_solids, 1) if index in selected]
        require(len(solids) == len(selected), f"GO_SOLID_SELECTION:{token}")
    else:
        solids = all_solids
    require(bool(solids), f"BREP_NO_SOLID:{token}")
    atoms = []
    for index, solid in enumerate(solids, 1):
        volume = abs(float(solid.Volume))
        require(volume > EPS_VOLUME_MM3, f"BREP_ZERO_VOLUME:{token}:{index}")
        require(solid.isValid() and solid.isClosed(), f"BREP_INVALID_SOLID:{token}:{index}")
        center = list(xyz(solid.CenterOfGravity))
        require(finite3(center), f"BREP_NONFINITE_COM:{token}:{index}")
        atoms.append(
            {
                "volume_mm3": volume,
                "center_world_mm": center,
                "bbox_world_mm": bbox_from_boundbox(solid.BoundBox),
                "signed_volume_mm3": float(solid.Volume),
            }
        )
    return {
        "member": token,
        "object_name": object_name,
        "type_id": obj.TypeId,
        "shape_type": obj.Shape.ShapeType,
        "solid_count": len(atoms),
        "volume_mm3": math.fsum(item["volume_mm3"] for item in atoms),
        "center_world_mm": weighted_center(atoms),
        "bbox_world_mm": merge_bboxes(item["bbox_world_mm"] for item in atoms),
        "negative_solid_count": sum(item["signed_volume_mm3"] < 0.0 for item in atoms),
    }


def brep_component(doc: Any, members: Sequence[str]) -> dict[str, Any]:
    records = [brep_member(doc, member) for member in members]
    return {
        "geometry_class": "BREP",
        "members": records,
        "volume_mm3": math.fsum(item["volume_mm3"] for item in records),
        "center_world_mm": weighted_center(records),
        "bbox_world_mm": merge_bboxes(item["bbox_world_mm"] for item in records),
    }


def validate_gripper(
    doc: Any, manifest: dict[str, Any], gripper_members: Sequence[str]
) -> dict[str, Any]:
    require(len(gripper_members) == 37 and len(set(gripper_members)) == 37, "GRIPPER_MEMBER_COUNT")
    part_features = 0
    mesh_features = 0
    compounds = 0
    top_solids = 0
    underlying_solids = 0
    negative = 0
    records = []
    for member in gripper_members:
        obj = doc.getObject(member)
        require(obj is not None, f"GRIPPER_MEMBER_MISSING:{member}")
        part_features += obj.TypeId == "Part::Feature"
        mesh_features += obj.TypeId == "Mesh::Feature"
        require(obj.TypeId == "Part::Feature", f"GRIPPER_MESH_OR_OTHER_MEMBER:{member}:{obj.TypeId}")
        require(obj.Shape.ShapeType in {"Compound", "Solid"}, f"GRIPPER_SHAPE_TYPE:{member}")
        compounds += obj.Shape.ShapeType == "Compound"
        top_solids += obj.Shape.ShapeType == "Solid"
        solids = list(obj.Shape.Solids)
        require(bool(solids), f"GRIPPER_NO_SOLID:{member}")
        for index, solid in enumerate(solids, 1):
            require(solid.isValid() and solid.isClosed(), f"GRIPPER_SOLID_INVALID:{member}:{index}")
            require(abs(float(solid.Volume)) > EPS_VOLUME_MM3, f"GRIPPER_SOLID_ZERO:{member}:{index}")
            negative += float(solid.Volume) < 0.0
        underlying_solids += len(solids)
        records.append({"member": member, "type_id": obj.TypeId, "solid_count": len(solids)})
    require(part_features == 37 and mesh_features == 0, "GRIPPER_TYPE_COUNTS")
    require(compounds == 12 and top_solids == 25, "GRIPPER_TOP_SHAPE_COUNTS")
    require(underlying_solids == 56 and negative == 0, "GRIPPER_SOLID_COUNTS")

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
    roles = manifest["gripper_internal_rigid_roles"]
    require(set(roles) == set(expected_roles), "GRIPPER_INTERNAL_ROLE_SET")
    for role, members in expected_roles.items():
        require(roles[role]["members"] == members, f"GRIPPER_INTERNAL_ROLE:{role}")
    policy = manifest["gripper_internal_policy"]
    require(policy["gripper_main_link_contains_only_static_members"], "GRIPPER_STATIC_POLICY")
    require(policy["moving_roles_are_not_merged_into_gripper"], "GRIPPER_OWNER_POLICY")
    controller = doc.getObject("Gripper_Servo_Controller")
    require(controller is not None and hasattr(controller, "ClosureAngle"), "GRIPPER_CONTROLLER")
    closure = float(controller.ClosureAngle)
    require(abs(closure) <= 1.0e-12, f"GRIPPER_CLOSURE:{closure}")
    return {
        "member_count": len(records),
        "part_feature_count": part_features,
        "mesh_feature_count": mesh_features,
        "underlying_solid_count": underlying_solids,
        "closure_angle_deg": closure,
    }


def validate_frame(frame: dict[str, Any], link: str) -> tuple[list[float], list[list[float]]]:
    origin = [float(value) for value in frame["origin_mm"]]
    rotation = [[float(value) for value in row] for row in frame["rotation_matrix_row_major"]]
    require(finite3(origin) and len(rotation) == 3 and all(finite3(row) for row in rotation), f"FRAME_NONFINITE:{link}")
    for first in range(3):
        for second in range(3):
            product = math.fsum(rotation[row][first] * rotation[row][second] for row in range(3))
            require(abs(product - (1.0 if first == second else 0.0)) <= 1.0e-9, f"FRAME_ORTHONORMAL:{link}")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    require(abs(determinant - 1.0) <= 1.0e-9, f"FRAME_DETERMINANT:{link}")
    return origin, rotation


def world_to_local(point: Sequence[float], origin: Sequence[float], rotation: Sequence[Sequence[float]]) -> list[float]:
    delta = [float(point[axis]) - float(origin[axis]) for axis in range(3)]
    return [math.fsum(rotation[row][column] * delta[row] for row in range(3)) for column in range(3)]


def local_to_world(point: Sequence[float], origin: Sequence[float], rotation: Sequence[Sequence[float]]) -> list[float]:
    return [float(origin[row]) + math.fsum(rotation[row][column] * point[column] for column in range(3)) for row in range(3)]


def point_in_bbox(point: Sequence[float], bounds: dict[str, list[float]], tolerance: float = 1.0e-6) -> bool:
    return all(
        bounds["min_mm"][axis] - tolerance <= float(point[axis]) <= bounds["max_mm"][axis] + tolerance
        for axis in range(3)
    )


def load_authorities(root: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    mass = json.loads((root / MASS_LEDGER).read_text(encoding="utf-8"), parse_float=Decimal)
    v1 = json.loads((root / V1_LEDGER).read_text(encoding="utf-8"))
    v2 = json.loads((root / V2_LEDGER).read_text(encoding="utf-8"))
    manifest = json.loads((root / MANIFEST).read_text(encoding="utf-8"))
    require(mass["final_status"] == "V15.15 MASS_LEDGER_V1 = PASS", "MASS_LEDGER_STATUS")
    require(v1["final_status"] == "V15.15 COM_LEDGER_V1 = PASS", "V1_LEDGER_STATUS")
    require(mass["unresolved_items"] == [], "MASS_LEDGER_UNRESOLVED")
    require(v2["schema"] == SCHEMA, "V2_SCHEMA")
    require(v2["final_status"] == "V15.15 COM_LEDGER_V2 = PASS", "V2_STATUS")
    provenance = v2["provenance"]
    require(provenance["source_branch"] == SOURCE_BRANCH, "V2_SOURCE_BRANCH")
    require(provenance["source_commit"] == SOURCE_COMMIT, "V2_SOURCE_COMMIT")
    require(provenance["source_tree"] == SOURCE_TREE, "V2_SOURCE_TREE")
    require(provenance["target_branch"] == TARGET_BRANCH, "V2_TARGET_BRANCH")
    correction = v2["v1_defect_correction_summary"]
    require(correction["defect"] == "FREECAD_MESH_GET_GRAVITY_POINT_RETURNS_VERTEX_MEAN", "V1_DEFECT_TEXT")
    require(
        correction["superseded_reason"]
        == "FREECAD_MESH_CENTER_OF_GRAVITY_IS_VERTEX_MEAN_NOT_VOLUME_CENTROID",
        "V1_SUPERSEDED_REASON",
    )
    require(correction["v1_json_and_markdown_authority_status"] == "SUPERSEDED_AUDIT_ARTIFACT", "V1_STATUS")
    require(v2["unresolved_items"] == [], "V2_UNRESOLVED")
    require(v2["algorithm_contract"]["mesh_center_of_gravity_role"] == "DIAGNOSTIC_ONLY_USED_IN_NO_MASS_FORMULA", "MESH_COG_ROLE")
    require(v2["algorithm_contract"]["mesh_volume_role"] == "DIAGNOSTIC_ONLY_USED_IN_NO_MASS_FORMULA", "MESH_VOLUME_ROLE")
    return mass, v1, v2, manifest


def validate_mass_authority(mass: dict[str, Any]) -> dict[str, dict[str, Any]]:
    top = mass["component_to_link_mapping"]["additive_components"]
    require(set(item["component_id"] for item in top) == set(COMPONENT_ORDER), "MASS_COMPONENT_SET")
    require(len({item["component_id"] for item in top}) == 17, "MASS_COMPONENT_UNIQUE")
    embedded = {
        item["component_id"]: item
        for link in LINK_ORDER
        for item in mass["link_mass_ledger"][link]["components"]
    }
    authority = {item["component_id"]: item for item in top}
    require(embedded == authority, "MASS_AUTHORITY_DUPLICATE_DISAGREEMENT")
    for link, expected in EXPECTED_LINK_MASSES.items():
        actual = Decimal(str(mass["link_mass_ledger"][link]["nominal_mass_kg"]))
        require(abs(actual - expected) <= MASS_TOL_KG, f"FROZEN_LINK_MASS:{link}:{actual}")
    total = Decimal(str(mass["total_link2_to_gripper_nominal_mass_kg"]))
    require(abs(total - EXPECTED_TOTAL_MASS) <= MASS_TOL_KG, f"FROZEN_TOTAL_MASS:{total}")
    return authority


def v1_component_map(v1: dict[str, Any]) -> dict[str, dict[str, Any]]:
    records = {
        item["component_id"]: item
        for link in LINK_ORDER
        for item in v1["links"][link]["components"]
    }
    require(set(records) == set(COMPONENT_ORDER), "V1_COMPONENT_SET")
    return records


def validate_document_and_recompute(
    root: Path,
    mass: dict[str, Any],
    v1: dict[str, Any],
    v2: dict[str, Any],
    manifest: dict[str, Any],
) -> dict[str, float]:
    mass_components = validate_mass_authority(mass)
    v1_components = v1_component_map(v1)
    require(v2["component_order"] == COMPONENT_ORDER, "V2_COMPONENT_ORDER")
    require([item["component_id"] for item in v2["components"]] == COMPONENT_ORDER, "V2_COMPONENT_RECORD_ORDER")
    v2_components = {item["component_id"]: item for item in v2["components"]}
    affected_sources = {
        "UPPER_ARM_PRINT_MEASURED": "UpperArm_B_Distal_PrintPart",
        "FOREARM_PRINT_MEASURED": "Forearm_v3_HighDetail_Display",
        "WRIST_PRELINK_PRINT_MEASURED": "Wrist_Prelink_v1_HighDetail_Display",
    }
    correction = v2["v1_defect_correction_summary"]
    require(
        correction["affected_source_mesh_members"] == list(affected_sources.values()),
        "AFFECTED_SOURCE_MESH_MEMBERS",
    )
    require(
        correction["corrected_component_ids"] == list(affected_sources),
        "AFFECTED_COMPONENT_IDS",
    )
    require(
        correction["additional_affected_mesh_members_found"] == [],
        "ADDITIONAL_AFFECTED_MESH_MEMBERS",
    )
    affected_delta_records = {
        item["component_id"]: item
        for item in correction["affected_component_v1_v2_delta"]
    }
    require(set(affected_delta_records) == set(affected_sources), "AFFECTED_DELTA_RECORD_SET")

    protected_before = {relative: sha256_file(root / relative) for relative in PROTECTED_HASHES}
    doc = App.openDocument(str(root / SOURCE_CAD))
    try:
        upper_a = validate_upper_a(root, doc)
        generated = {
            object_name: validate_generated_mesh(root, doc, object_name)
            for object_name in MESH_SPECS
        }
        upper_b = generated["UpperArm_B_Distal_PrintPart"]
        upper_total_volume = upper_a["volume_mm3"] + upper_b["volume_mm3"]
        upper_geometry = {
            "geometry_class": "MIXED_PROTECTED_AND_GENERATED_MASS_PROPERTIES_ARTIFACTS",
            "volume_mm3": upper_total_volume,
            "center_world_mm": [
                (
                    upper_a["volume_mm3"] * upper_a["center_world_mm"][axis]
                    + upper_b["volume_mm3"] * upper_b["center_world_mm"][axis]
                )
                / upper_total_volume
                for axis in range(3)
            ],
            "bbox_world_mm": merge_bboxes([upper_a["bbox_world_mm"], upper_b["bbox_world_mm"]]),
        }
        geometries: dict[str, dict[str, Any]] = {
            "UPPER_ARM_PRINT_MEASURED": upper_geometry,
            "FOREARM_PRINT_MEASURED": generated["Forearm_v3_HighDetail_Display"],
            "WRIST_PRELINK_PRINT_MEASURED": generated["Wrist_Prelink_v1_HighDetail_Display"],
        }
        for component_id in COMPONENT_ORDER:
            if component_id not in geometries:
                geometries[component_id] = brep_component(
                    doc, list(mass_components[component_id]["cad_members"])
                )
        gripper_members = list(
            mass_components["GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP"]["cad_members"]
        )
        gripper_audit = validate_gripper(doc, manifest, gripper_members)
    finally:
        App.closeDocument(doc.Name)
    protected_after = {relative: sha256_file(root / relative) for relative in PROTECTED_HASHES}
    require(protected_before == protected_after, "PROTECTED_INPUT_CHANGED_DURING_VALIDATION")

    require(gripper_audit["part_feature_count"] == 37, "GRIPPER_NOT_ALL_PART")
    require(gripper_audit["mesh_feature_count"] == 0, "GRIPPER_MESH_FOUND")
    require(v2["mechanical_zero_condition"]["gripper_closure_angle_deg"] == 0.0, "V2_CLOSURE")
    require(v2["double_count_checks"]["gripper_rollup_member_count"] == 37, "V2_GRIPPER_COUNT")
    require(v2["double_count_checks"]["neutral_b6808_separate_mass_count"] == 0, "NEUTRAL_DOUBLE_COUNT")

    upper_record = v2_components["UPPER_ARM_PRINT_MEASURED"]
    allocation = upper_record["equal_density_volume_allocation"]
    require(len(allocation["parts"]) == 2 and allocation["fifty_fifty_used"] is False, "UPPER_ALLOCATION_POLICY")
    expected_fractions = [upper_a["volume_mm3"] / upper_total_volume, upper_b["volume_mm3"] / upper_total_volume]
    v1_allocated_masses = [0.3715953060813982, 0.1799046939186018]
    expected_allocated_masses: list[float] = []
    for index, expected_fraction in enumerate(expected_fractions):
        close_scalar(float(allocation["parts"][index]["volume_fraction"]), expected_fraction, 1.0e-12, f"UPPER_FRACTION:{index}")
        expected_mass = 0.5515 * expected_fraction
        expected_allocated_masses.append(expected_mass)
        close_scalar(float(allocation["parts"][index]["allocated_mass_kg"]), expected_mass, 1.0e-12, f"UPPER_ALLOCATED_MASS:{index}")
        close_scalar(
            float(allocation["parts"][index]["v1_allocated_mass_kg"]),
            v1_allocated_masses[index],
            0.0,
            f"UPPER_V1_ALLOCATED_MASS:{index}",
        )
        close_scalar(
            float(allocation["parts"][index]["delta_from_v1_allocated_mass_kg"]),
            expected_mass - v1_allocated_masses[index],
            1.0e-15,
            f"UPPER_ALLOCATED_MASS_DELTA:{index}",
        )
    close_scalar(float(allocation["allocated_mass_sum_kg"]), 0.5515, 1.0e-12, "UPPER_ALLOCATED_MASS_SUM")
    close_scalar(float(allocation["v1_allocated_mass_sum_kg"]), 0.5515, 1.0e-12, "UPPER_V1_ALLOCATED_MASS_SUM")
    comparison = allocation["comparison_to_v1"]
    comparison_tolerance = float(comparison["tolerance_kg"])
    max_split_delta = max(
        abs(expected_allocated_masses[index] - v1_allocated_masses[index])
        for index in range(2)
    )
    expected_comparison_status = (
        "SAME" if max_split_delta <= comparison_tolerance else "CHANGED_WITH_FROZEN_TOTAL"
    )
    require(comparison["status"] == expected_comparison_status, "UPPER_V1_COMPARISON_STATUS")
    close_scalar(
        float(comparison["max_abs_allocated_mass_delta_kg"]),
        max_split_delta,
        1.0e-15,
        "UPPER_V1_COMPARISON_MAX_DELTA",
    )

    computed_components: dict[str, dict[str, Any]] = {}
    component_roundtrips: list[float] = []
    brep_count = 0
    max_brep_axis_error = 0.0
    max_brep_norm_error = 0.0
    for component_id in COMPONENT_ORDER:
        authority = mass_components[component_id]
        geometry = geometries[component_id]
        ledger = v2_components[component_id]
        baseline = v1_components[component_id]
        owner = authority["ledger_link"]
        require(ledger["owner_link"] == owner, f"OWNER_LINK:{component_id}")
        require(ledger["geometry_member_order_inherited_exactly_from_v1_mass_ledger"] == list(authority["cad_members"]), f"MEMBERSHIP_CHANGED:{component_id}")
        mass_kg = Decimal(str(authority["nominal_mass_kg"]))
        require(abs(Decimal(str(ledger["frozen_mass_kg"])) - mass_kg) <= MASS_TOL_KG, f"COMPONENT_MASS_CHANGED:{component_id}")
        require(ledger["collision_proxy_used"] is False, f"COMPONENT_COLLISION_PROXY:{component_id}")
        require(all("Collision_Proxy" not in member for member in authority["cad_members"]), f"AUTHORITY_COLLISION_MEMBER:{component_id}")
        world = list(geometry["center_world_mm"])
        require(finite3(world), f"COMPONENT_WORLD_NONFINITE:{component_id}")
        require(point_in_bbox(world, geometry["bbox_world_mm"]), f"COMPONENT_COM_OUTSIDE_BBOX:{component_id}")
        close_scalar(float(ledger["volume_mm3"]), float(geometry["volume_mm3"]), 1.0e-6, f"V2_COMPONENT_VOLUME:{component_id}")
        close_vector(ledger["com_world_mm"], world, LEDGER_NUMERIC_TOL_MM, f"V2_COMPONENT_WORLD:{component_id}")
        frame = manifest["links"][owner]["frame_world_at_zero"]
        origin, rotation = validate_frame(frame, owner)
        local = world_to_local(world, origin, rotation)
        close_vector(ledger["com_owner_link_mm"], local, LEDGER_NUMERIC_TOL_MM, f"V2_COMPONENT_LOCAL:{component_id}")
        reconstructed = local_to_world(local, origin, rotation)
        roundtrip = distance(reconstructed, world) * 0.001
        require(roundtrip < ROUNDTRIP_TOL_M, f"COMPONENT_ROUNDTRIP:{component_id}:{roundtrip}")
        close_scalar(float(ledger["coordinate_round_trip"]["error_m"]), roundtrip, 1.0e-15, f"V2_COMPONENT_ROUNDTRIP:{component_id}")
        baseline_world = [float(value) for value in baseline["cad_com_world_mm"]]
        expected_delta = [world[axis] - baseline_world[axis] for axis in range(3)]
        close_vector(ledger["v2_delta_from_v1"]["com_world_mm"], expected_delta, LEDGER_NUMERIC_TOL_MM, f"COMPONENT_V1_DELTA:{component_id}")
        close_scalar(float(ledger["v2_delta_from_v1"]["com_world_norm_mm"]), distance(world, baseline_world), LEDGER_NUMERIC_TOL_MM, f"COMPONENT_V1_DELTA_NORM:{component_id}")
        if geometry["geometry_class"] == "BREP":
            brep_count += 1
            axis_error = max(abs(world[axis] - baseline_world[axis]) for axis in range(3))
            norm_error = distance(world, baseline_world)
            max_brep_axis_error = max(max_brep_axis_error, axis_error)
            max_brep_norm_error = max(max_brep_norm_error, norm_error)
            require(norm_error < COM_TOL_MM, f"BREP_V1_COM_CHANGED:{component_id}:{norm_error}")
            require(abs(float(geometry["volume_mm3"]) - float(baseline["cad_volume_mm3"])) < 1.0e-6, f"BREP_V1_VOLUME_CHANGED:{component_id}")
            require(ledger["v1_zero_delta_crosscheck"]["pass"] is True, f"BREP_LEDGER_CROSSCHECK:{component_id}")
        computed_components[component_id] = {
            "owner_link": owner,
            "mass_kg": float(mass_kg),
            "world_mm": world,
            "local_mm": local,
            "bbox_world_mm": geometry["bbox_world_mm"],
        }
        component_roundtrips.append(roundtrip)
    require(brep_count == 14, f"BREP_COMPONENT_COUNT:{brep_count}")
    require(max_brep_norm_error < COM_TOL_MM, f"MAX_BREP_COM_DELTA:{max_brep_norm_error}")
    close_scalar(
        float(v2["validation"]["max_brep_com_norm_error_from_v1_mm"]),
        max_brep_norm_error,
        1.0e-15,
        "V2_MAX_BREP_COM_NORM_ERROR",
    )
    for component_id, source_member in affected_sources.items():
        summary = affected_delta_records[component_id]
        baseline_world = [
            float(value) for value in v1_components[component_id]["cad_com_world_mm"]
        ]
        corrected_world = computed_components[component_id]["world_mm"]
        delta = [corrected_world[axis] - baseline_world[axis] for axis in range(3)]
        require(summary["source_mesh_member"] == source_member, f"AFFECTED_SOURCE:{component_id}")
        close_vector(summary["v1_com_world_mm"], baseline_world, 0.0, f"AFFECTED_V1_COM:{component_id}")
        close_vector(summary["v2_com_world_mm"], corrected_world, LEDGER_NUMERIC_TOL_MM, f"AFFECTED_V2_COM:{component_id}")
        close_vector(summary["delta_xyz_mm"], delta, LEDGER_NUMERIC_TOL_MM, f"AFFECTED_DELTA:{component_id}")
        close_scalar(
            float(summary["delta_norm_mm"]),
            distance(corrected_world, baseline_world),
            LEDGER_NUMERIC_TOL_MM,
            f"AFFECTED_DELTA_NORM:{component_id}",
        )

    computed_links: dict[str, dict[str, Any]] = {}
    link_roundtrips: list[float] = []
    for link in LINK_ORDER:
        components = [computed_components[item] for item in COMPONENT_ORDER if computed_components[item]["owner_link"] == link]
        component_mass = sum(Decimal(str(item["mass_kg"])) for item in components)
        require(abs(component_mass - EXPECTED_LINK_MASSES[link]) <= MASS_TOL_KG, f"LINK_COMPONENT_MASS:{link}")
        mass_float = float(EXPECTED_LINK_MASSES[link])
        world = [
            math.fsum(item["mass_kg"] * item["world_mm"][axis] for item in components) / mass_float
            for axis in range(3)
        ]
        combined_bbox = merge_bboxes(item["bbox_world_mm"] for item in components)
        require(point_in_bbox(world, combined_bbox), f"LINK_COM_OUTSIDE_COMBINED_BBOX:{link}")
        ledger = v2["links"][link]
        manifest_frame = manifest["links"][link]["frame_world_at_zero"]
        require(
            ledger["frame_world_at_mechanical_zero"] == manifest_frame,
            f"V2_LINK_FRAME_AUTHORITY_MISMATCH:{link}",
        )
        require(ledger["component_ids"] == [item for item in COMPONENT_ORDER if computed_components[item]["owner_link"] == link], f"LINK_COMPONENT_ORDER:{link}")
        close_scalar(float(ledger["mass_kg"]), mass_float, 1.0e-12, f"V2_LINK_MASS:{link}")
        close_vector(ledger["com_world_mm"], world, LEDGER_NUMERIC_TOL_MM, f"V2_LINK_WORLD:{link}")
        close_vector(
            ledger["combined_mass_component_bbox_world_mm"]["min_mm"],
            combined_bbox["min_mm"],
            LEDGER_NUMERIC_TOL_MM,
            f"V2_LINK_BBOX_MIN:{link}",
        )
        close_vector(
            ledger["combined_mass_component_bbox_world_mm"]["max_mm"],
            combined_bbox["max_mm"],
            LEDGER_NUMERIC_TOL_MM,
            f"V2_LINK_BBOX_MAX:{link}",
        )
        origin, rotation = validate_frame(manifest_frame, link)
        local = world_to_local(world, origin, rotation)
        close_vector(ledger["com_link_mm"], local, LEDGER_NUMERIC_TOL_MM, f"V2_LINK_LOCAL:{link}")
        reconstructed = local_to_world(local, origin, rotation)
        roundtrip = distance(reconstructed, world) * 0.001
        require(roundtrip < ROUNDTRIP_TOL_M, f"LINK_ROUNDTRIP:{link}:{roundtrip}")
        close_scalar(float(ledger["coordinate_round_trip"]["error_m"]), roundtrip, 1.0e-15, f"V2_LINK_ROUNDTRIP:{link}")
        v1_world_m = [float(value) for value in v1["links"][link]["com_xyz_m_world_at_zero"]]
        v1_local_m = [float(value) for value in v1["links"][link]["com_xyz_m_in_link_frame"]]
        v1_world = [1000.0 * value for value in v1_world_m]
        v1_local = [1000.0 * value for value in v1_local_m]
        world_delta = [world[axis] - v1_world[axis] for axis in range(3)]
        local_delta = [local[axis] - v1_local[axis] for axis in range(3)]
        close_vector(ledger["v1_baseline"]["com_world_m"], v1_world_m, 0.0, f"LINK_V1_WORLD:{link}")
        close_vector(ledger["v1_baseline"]["com_link_m"], v1_local_m, 0.0, f"LINK_V1_LOCAL:{link}")
        close_vector(ledger["v2_delta_from_v1"]["com_world_mm"], world_delta, LEDGER_NUMERIC_TOL_MM, f"LINK_V1_WORLD_DELTA:{link}")
        close_vector(ledger["v2_delta_from_v1"]["com_link_mm"], local_delta, LEDGER_NUMERIC_TOL_MM, f"LINK_V1_LOCAL_DELTA:{link}")
        close_scalar(float(ledger["v2_delta_from_v1"]["com_world_norm_mm"]), distance(world, v1_world), LEDGER_NUMERIC_TOL_MM, f"LINK_V1_DELTA_NORM:{link}")
        close_scalar(float(ledger["v2_delta_from_v1"]["com_link_norm_mm"]), distance(local, v1_local), LEDGER_NUMERIC_TOL_MM, f"LINK_V1_LOCAL_DELTA_NORM:{link}")
        expected_reason = {
            "link2": "UPPER_ARM_PRINT_MEASURED_VOLUME_CENTROID_CORRECTED_FROM_V1_MESH_VERTEX_MEAN",
            "link3": "FOREARM_PRINT_MEASURED_VOLUME_CENTROID_CORRECTED_FROM_V1_MESH_VERTEX_MEAN",
            "link4": "WRIST_PRELINK_PRINT_MEASURED_VOLUME_CENTROID_CORRECTED_FROM_V1_MESH_VERTEX_MEAN",
        }.get(link, "UNCHANGED_ALL_MEMBERS_BREP_AND_NO_AFFECTED_SOURCE_MESH_COMPONENT")
        require(ledger["v2_delta_from_v1"]["reason"] == expected_reason, f"LINK_DELTA_REASON:{link}")
        if link in {"link5", "link6", "gripper"}:
            require(distance(world, v1_world) < COM_TOL_MM, f"UNCHANGED_LINK_MOVED:{link}")
        computed_links[link] = {"mass_kg": mass_float, "world_mm": world, "local_mm": local}
        link_roundtrips.append(roundtrip)

    total_mass = sum(Decimal(str(computed_links[link]["mass_kg"])) for link in LINK_ORDER)
    require(abs(total_mass - EXPECTED_TOTAL_MASS) <= MASS_TOL_KG, f"TOTAL_MASS:{total_mass}")
    chain_mass = float(total_mass)
    leaf_world = [
        math.fsum(computed_components[item]["mass_kg"] * computed_components[item]["world_mm"][axis] for item in COMPONENT_ORDER) / chain_mass
        for axis in range(3)
    ]
    link_world = [
        math.fsum(computed_links[link]["mass_kg"] * computed_links[link]["world_mm"][axis] for link in LINK_ORDER) / chain_mass
        for axis in range(3)
    ]
    chain_error_m = distance(leaf_world, link_world) * 0.001
    require(chain_error_m < CHAIN_TOL_M, f"CHAIN_LEAF_LINK_CLOSURE:{chain_error_m}")
    close_vector(v2["chain_summary"]["leaf_component_path"]["com_world_mm"], leaf_world, LEDGER_NUMERIC_TOL_MM, "V2_CHAIN_LEAF")
    close_vector(v2["chain_summary"]["six_link_path"]["com_world_mm"], link_world, LEDGER_NUMERIC_TOL_MM, "V2_CHAIN_LINK")
    close_scalar(float(v2["chain_summary"]["leaf_vs_six_link_closure"]["distance_m"]), chain_error_m, 1.0e-15, "V2_CHAIN_ERROR")

    max_component_roundtrip = max(component_roundtrips)
    max_link_roundtrip = max(link_roundtrips)
    close_scalar(float(v2["validation"]["max_component_coordinate_roundtrip_error_m"]), max_component_roundtrip, 1.0e-15, "V2_MAX_COMPONENT_ROUNDTRIP")
    close_scalar(float(v2["validation"]["max_link_coordinate_roundtrip_error_m"]), max_link_roundtrip, 1.0e-15, "V2_MAX_LINK_ROUNDTRIP")
    require(v2["validation"]["all_assertions_pass"] is True, "V2_ASSERTIONS_FLAG")
    require(all(value is False for value in v2["prohibited_outputs"].values()), "V2_PROHIBITED_OUTPUT_FLAG")
    return {
        "chain_error_m": chain_error_m,
        "max_component_roundtrip_m": max_component_roundtrip,
        "max_link_roundtrip_m": max_link_roundtrip,
        "max_brep_axis_error_mm": max_brep_axis_error,
    }


FORBIDDEN_NUMERIC_KEYS = {
    "inertia",
    "inertia_tensor",
    "ixx",
    "iyy",
    "izz",
    "ixy",
    "ixz",
    "iyz",
    "gravity",
    "damping",
    "friction",
    "armature",
}


def reject_forbidden_numeric_fields(value: Any, path: str = "$") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key.lower() in FORBIDDEN_NUMERIC_KEYS:
                raise ValidationError(f"FORBIDDEN_FIELD:{path}.{key}")
            reject_forbidden_numeric_fields(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            reject_forbidden_numeric_fields(child, f"{path}[{index}]")


def allowed_path_snapshot(root: Path) -> dict[str, tuple[int, str]]:
    snapshot: dict[str, tuple[int, str]] = {}
    for relative in sorted(ALLOWED_CHANGED_PATHS):
        path = root / relative
        require(path.is_file(), f"ALLOWED_OUTPUT_MISSING:{relative}")
        snapshot[relative] = (path.stat().st_size, sha256_file(path))
    return snapshot


def validate_markdown_contract(root: Path) -> None:
    markdown = (root / V2_MARKDOWN).read_text(encoding="utf-8")
    required_fragments = [
        "V15.15 COM_LEDGER_V2 = PASS",
        "SUPERSEDED_AUDIT_ARTIFACT",
        "FREECAD_MESH_CENTER_OF_GRAVITY_IS_VERTEX_MEAN_NOT_VOLUME_CENTROID",
        "FREECAD_MESH_GET_GRAVITY_POINT_RETURNS_VERTEX_MEAN",
        "UpperArm_B_Distal_PrintPart",
        "Forearm_v3_HighDetail_Display",
        "Wrist_Prelink_v1_HighDetail_Display",
        "CHANGED_WITH_FROZEN_TOTAL",
        "collision proxy used: **NO**",
        "leaf-vs-six-link closure",
    ]
    for fragment in required_fragments:
        require(fragment in markdown, f"MARKDOWN_CONTRACT_FRAGMENT_MISSING:{fragment}")


def run_builder_reproducibility_check(root: Path) -> None:
    before = allowed_path_snapshot(root)
    result = subprocess.run(
        [str(Path(sys.executable).resolve()), str(root / V2_BUILDER), "--check"],
        cwd=root,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=600,
        shell=False,
    )
    require(result.returncode == 0, f"BUILDER_CHECK_EXIT:{result.returncode}:{result.stdout[-1000:]}")
    require("COM_VOLUME_V2_ARTIFACTS_REPRODUCIBLE = PASS" in result.stdout, "BUILDER_REPRODUCIBLE_MARKER")
    after = allowed_path_snapshot(root)
    require(after == before, "BUILDER_CHECK_MUTATED_ALLOWED_OUTPUT")


def main() -> int:
    try:
        require(App is not None and Mesh is not None and Part is not None, f"FREECAD_RUNTIME_REQUIRED:{FREECAD_IMPORT_ERROR}")
        root = repo_root()
        verify_git_and_protected(root)
        run_synthetic_test(root)
        mass, v1, v2, manifest = load_authorities(root)
        reject_forbidden_numeric_fields(v2)
        validate_markdown_contract(root)
        metrics = validate_document_and_recompute(root, mass, v1, v2, manifest)
        # All semantic and numeric evidence has passed independently before
        # this subprocess.  Its sole role is deterministic byte reproduction.
        run_builder_reproducibility_check(root)
        verify_git_and_protected(root)
        print("V15.15 COM V2 independent validation = PASS")
        print("mesh semantic synthetic test = PASS")
        print("independent source repair and STL byte parsing = PASS")
        print("mesh polyhedral/Part dual paths = PASS")
        print("14 BRep components and gripper type audit = PASS")
        print("six-link mass/COM and whole-chain closure = PASS")
        print(f"leaf-vs-link chain COM error m = {metrics['chain_error_m']:.17g}")
        print(f"max component round-trip error m = {metrics['max_component_roundtrip_m']:.17g}")
        print(f"max link round-trip error m = {metrics['max_link_roundtrip_m']:.17g}")
        print(f"max BRep axis delta vs V1 mm = {metrics['max_brep_axis_error_mm']:.17g}")
        print("builder --check reproducibility = PASS")
        print("V15.15 COM_LEDGER_V2 = PASS")
        return 0
    except (
        ValidationError,
        OSError,
        KeyError,
        ValueError,
        TypeError,
        struct.error,
        subprocess.CalledProcessError,
        subprocess.TimeoutExpired,
    ) as error:
        print(f"COM_LEDGER_V2_VALIDATION_ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
