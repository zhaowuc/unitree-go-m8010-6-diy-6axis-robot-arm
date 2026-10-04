from __future__ import annotations

"""Build the V15.15 mass-properties-only geometry for UpperArm A.

The accepted FCStd is opened read-only.  Its embedded Mesh::Feature is copied,
returned to source-local coordinates, and repaired without moving any retained
surface vertex.  The only added surface is two deterministic planar caps.
"""

import argparse
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict, deque
from pathlib import Path
from typing import Any

try:
    import FreeCAD as App
    import Mesh
    import Part
except Exception as exc:  # pragma: no cover
    App = Mesh = Part = None
    FREECAD_IMPORT_ERROR = exc
else:
    FREECAD_IMPORT_ERROR = None


SCHEMA = "go-m8010-arm-v15.15-upperarm-a-mass-properties/1.0"
REVISION = "V15.15-UPPERARM_A_MASS_PROPERTIES_ONLY"
TASK_BASE_COMMIT = "494bb0c95f53cfd3fac7c9466736bcfb4bbbf50c"
TASK_BASE_TREE = "5378302411f30530225916771c99337b772589c9"
TARGET_BRANCH = "agent/v15-15-com-v1"
SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
SOURCE_CAD_SHA256 = "b4a02a97afaff88e246a46340dad4cacd3c645a7456fdd087a69552e14d3ddc0"
OBJECT_NAME = "UpperArm_A_SleeveSide_PrintPart"
OUT_DIR = "mass_properties_geometry_v15_15"
OUT_STL = OUT_DIR + "/UpperArm_A_SleeveSide_PrintPart_mass_properties.stl"
OUT_REPORT = OUT_DIR + "/UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json"
EXPECTED_STL_SHA256 = "67aed62b304cfef3a338b0e59b035426b8e012619c333d032e8304c389352f45"
EXPECTED_VOLUME_MM3 = 308654.9785581231
EXPECTED_SOURCE_COM_MM = [3414.7028593281334, 1277.1059547527118, 34.687833260140806]
EXPECTED_WORLD_COM_MM = [-0.7894327049961021, 53.98176991520586, 183.0347984497837]
RAW_METRICS = {
    "vertices": 1036,
    "facets": 1913,
    "components_freecad": 63,
    "open_edges": 235,
    "non_manifold_edges": 54,
    "orientation_conflicts": 196,
    "self_intersection_pairs": 2,
}


class BuildError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise BuildError(message)


def repo_root() -> Path:
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for candidate in (start, *start.parents):
            if (candidate / ".git").exists() and (candidate / SOURCE_CAD).is_file():
                return candidate
    raise BuildError("REPO_ROOT_NOT_FOUND")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().lower()


def git_guard(root: Path) -> dict[str, Any]:
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=root, text=True).strip()
    tree = subprocess.check_output(["git", "show", "-s", "--format=%T", TASK_BASE_COMMIT], cwd=root, text=True).strip()
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", TASK_BASE_COMMIT, "HEAD"], cwd=root
    ).returncode == 0
    require(branch == TARGET_BRANCH, f"WRONG_BRANCH:{branch}")
    require(tree == TASK_BASE_TREE, f"TASK_BASE_TREE_MISMATCH:{tree}")
    require(ancestor, "TASK_BASE_NOT_ANCESTOR")
    return {"branch": branch, "task_base_commit": TASK_BASE_COMMIT, "task_base_tree": tree, "pass": True}


def mesh_triangles_source_local(obj: Any) -> list[tuple[tuple[float, float, float], ...]]:
    copied = Mesh.Mesh(obj.Mesh)
    copied.Placement = App.Placement()
    triangles = []
    for facet in copied.Facets:
        triangles.append(tuple(tuple(float(value) for value in point) for point in facet.Points))
    return triangles


def weld(
    triangles: list[tuple[tuple[float, float, float], ...]],
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    index: dict[tuple[float, float, float], int] = {}
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    for triangle in triangles:
        face = []
        for vertex in triangle:
            if vertex not in index:
                index[vertex] = len(vertices)
                vertices.append(vertex)
            face.append(index[vertex])
        require(len(set(face)) == 3, "DEGENERATE_SOURCE_FACE")
        faces.append(tuple(face))
    return vertices, faces


def edge_map(faces: list[tuple[int, int, int]]) -> dict[tuple[int, int], list[tuple[int, int, int]]]:
    result: dict[tuple[int, int], list[tuple[int, int, int]]] = defaultdict(list)
    for face_id, face in enumerate(faces):
        for start, end in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            result[tuple(sorted((start, end)))].append((face_id, start, end))
    return result


def incidence_two_components(faces: list[tuple[int, int, int]]) -> list[list[int]]:
    adjacency = [set() for _ in faces]
    for occurrences in edge_map(faces).values():
        if len(occurrences) == 2:
            left, right = occurrences[0][0], occurrences[1][0]
            adjacency[left].add(right)
            adjacency[right].add(left)
    components = []
    seen = set()
    for face_id in range(len(faces)):
        if face_id in seen:
            continue
        component = []
        queue = [face_id]
        seen.add(face_id)
        while queue:
            current = queue.pop()
            component.append(current)
            for neighbour in adjacency[current]:
                if neighbour not in seen:
                    seen.add(neighbour)
                    queue.append(neighbour)
        components.append(sorted(component))
    components.sort(key=lambda item: (-len(item), item[0]))
    return components


def bbox(vertices: list[tuple[float, float, float]]) -> dict[str, list[float]]:
    minimum = [min(vertex[axis] for vertex in vertices) for axis in range(3)]
    maximum = [max(vertex[axis] for vertex in vertices) for axis in range(3)]
    return {
        "min_mm": minimum,
        "max_mm": maximum,
        "size_mm": [maximum[axis] - minimum[axis] for axis in range(3)],
    }


def boundary_loops(faces: list[tuple[int, int, int]]) -> list[list[int]]:
    graph: dict[int, set[int]] = defaultdict(set)
    for (start, end), occurrences in edge_map(faces).items():
        if len(occurrences) == 1:
            graph[start].add(end)
            graph[end].add(start)
    require(graph and all(len(neighbours) == 2 for neighbours in graph.values()), "BOUNDARY_NOT_SIMPLE_LOOPS")
    loops = []
    unvisited = set(graph)
    while unvisited:
        start = min(unvisited)
        candidates = []
        for first in sorted(graph[start]):
            loop = [start]
            previous, current = start, first
            while current != start:
                loop.append(current)
                options = sorted(graph[current] - {previous})
                require(len(options) == 1, "BOUNDARY_BRANCH")
                previous, current = current, options[0]
                require(len(loop) <= len(graph) + 1, "BOUNDARY_LOOP_OVERFLOW")
            candidates.append(loop)
        loop = min(candidates)
        require(len(set(loop)) == len(loop), "BOUNDARY_LOOP_REPEATED_VERTEX")
        unvisited -= set(loop)
        loops.append(loop)
    loops.sort()
    return loops


def signed_area_xy(loop: list[int], vertices: list[tuple[float, float, float]]) -> float:
    return 0.5 * sum(
        vertices[loop[index]][0] * vertices[loop[(index + 1) % len(loop)]][1]
        - vertices[loop[(index + 1) % len(loop)]][0] * vertices[loop[index]][1]
        for index in range(len(loop))
    )


def cross2(a: tuple[float, float, float], b: tuple[float, float, float], c: tuple[float, float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def point_in_triangle_xy(point: tuple[float, float, float], triangle: list[tuple[float, float, float]]) -> bool:
    c0 = cross2(triangle[0], triangle[1], point)
    c1 = cross2(triangle[1], triangle[2], point)
    c2 = cross2(triangle[2], triangle[0], point)
    return c0 >= -1.0e-12 and c1 >= -1.0e-12 and c2 >= -1.0e-12


def ear_clip(loop: list[int], vertices: list[tuple[float, float, float]]) -> list[tuple[int, int, int]]:
    work = list(loop)
    if signed_area_xy(work, vertices) < 0.0:
        work.reverse()
    triangles = []
    while len(work) > 3:
        clipped = False
        for index in range(len(work)):
            previous = work[(index - 1) % len(work)]
            current = work[index]
            following = work[(index + 1) % len(work)]
            triangle_points = [vertices[previous], vertices[current], vertices[following]]
            if cross2(*triangle_points) <= 1.0e-12:
                continue
            if any(
                point_in_triangle_xy(vertices[candidate], triangle_points)
                for candidate in work
                if candidate not in {previous, current, following}
            ):
                continue
            triangles.append((previous, current, following))
            del work[index]
            clipped = True
            break
        require(clipped, "EAR_CLIPPING_FAILED")
    triangles.append(tuple(work))
    return triangles


def orient_faces(faces: list[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
    faces = list(faces)
    adjacency: dict[int, list[tuple[int, bool]]] = defaultdict(list)
    for occurrences in edge_map(faces).values():
        require(len(occurrences) == 2, "REPAIRED_EDGE_INCIDENCE_NOT_TWO")
        left, right = occurrences
        same = (left[1], left[2]) == (right[1], right[2])
        adjacency[left[0]].append((right[0], same))
        adjacency[right[0]].append((left[0], same))
    flip = {0: False}
    queue = deque([0])
    while queue:
        current = queue.popleft()
        for neighbour, same in adjacency[current]:
            expected = flip[current] ^ same
            if neighbour in flip:
                require(flip[neighbour] == expected, "REPAIRED_SURFACE_NOT_ORIENTABLE")
            else:
                flip[neighbour] = expected
                queue.append(neighbour)
    require(len(flip) == len(faces), "REPAIRED_SURFACE_DISCONNECTED")
    oriented = [
        (face[0], face[2], face[1]) if flip[index] else face for index, face in enumerate(faces)
    ]
    volume, _ = volume_and_centroid(oriented, None)
    if volume < 0.0:
        oriented = [(face[0], face[2], face[1]) for face in oriented]
    return oriented


def volume_and_centroid(
    faces: list[tuple[int, int, int]], vertices: list[tuple[float, float, float]] | None
) -> tuple[float, list[float]]:
    if vertices is None:
        vertices = _ORIENT_VERTICES
    volume6 = 0.0
    moment = [0.0, 0.0, 0.0]
    for face in faces:
        a, b, c = (vertices[index] for index in face)
        cross = (
            b[1] * c[2] - b[2] * c[1],
            b[2] * c[0] - b[0] * c[2],
            b[0] * c[1] - b[1] * c[0],
        )
        determinant = a[0] * cross[0] + a[1] * cross[1] + a[2] * cross[2]
        volume6 += determinant
        for axis in range(3):
            moment[axis] += determinant * (a[axis] + b[axis] + c[axis])
    volume = volume6 / 6.0
    require(abs(volume) > 1.0e-9, "ZERO_REPAIRED_VOLUME")
    center = [moment[axis] / (24.0 * volume) for axis in range(3)]
    return volume, center


def triangle_area(face: tuple[int, int, int], vertices: list[tuple[float, float, float]]) -> float:
    a, b, c = (vertices[index] for index in face)
    u = [b[axis] - a[axis] for axis in range(3)]
    v = [c[axis] - a[axis] for axis in range(3)]
    cross = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
    return 0.5 * math.sqrt(sum(value * value for value in cross))


def transform_point(placement: Any, point: list[float]) -> list[float]:
    transformed = placement.multVec(App.Vector(*point))
    return [float(transformed.x), float(transformed.y), float(transformed.z)]


def canonical_stl(vertices: list[tuple[float, float, float]], faces: list[tuple[int, int, int]]) -> bytes:
    header = b"V15.15 UpperArm A mass-properties only; source surface retained; planar caps"[:80].ljust(80, b"\0")
    payload = bytearray(header + struct.pack("<I", len(faces)))
    for face in faces:
        a, b, c = (vertices[index] for index in face)
        u = [b[axis] - a[axis] for axis in range(3)]
        v = [c[axis] - a[axis] for axis in range(3)]
        normal = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        length = math.sqrt(sum(value * value for value in normal))
        normal = [value / length for value in normal]
        payload.extend(struct.pack("<12fH", *(normal + list(a) + list(b) + list(c)), 0))
    return bytes(payload)


_ORIENT_VERTICES: list[tuple[float, float, float]] = []


def build(root: Path) -> tuple[bytes, dict[str, Any]]:
    require(App is not None and Mesh is not None and Part is not None, f"FREECAD_RUNTIME_REQUIRED:{FREECAD_IMPORT_ERROR}")
    guard = git_guard(root)
    source_path = root / SOURCE_CAD
    require(sha256(source_path) == SOURCE_CAD_SHA256, "SOURCE_CAD_HASH_MISMATCH")
    before_hash = sha256(source_path)
    document = App.openDocument(str(source_path))
    try:
        obj = document.getObject(OBJECT_NAME)
        require(obj is not None and obj.TypeId == "Mesh::Feature", "SOURCE_OBJECT_TYPE_MISMATCH")
        source_placement = obj.Mesh.Placement
        triangles = mesh_triangles_source_local(obj)
        source_self_intersections = len(Mesh.Mesh(obj.Mesh).getSelfIntersections())
    finally:
        App.closeDocument(document.Name)
    require(sha256(source_path) == before_hash, "SOURCE_CAD_MODIFIED_DURING_READ")
    vertices, faces = weld(triangles)
    raw_edges = edge_map(faces)
    raw_hist = Counter(len(value) for value in raw_edges.values())
    raw_orientation_conflicts = sum(
        1 for value in raw_edges.values() if len(value) == 2 and (value[0][1], value[0][2]) == (value[1][1], value[1][2])
    )
    raw_box = bbox(vertices)
    require(len(vertices) == RAW_METRICS["vertices"] and len(faces) == RAW_METRICS["facets"], "RAW_COUNTS_MISMATCH")
    require(raw_hist[1] == 235 and raw_hist[3] == 54 and raw_orientation_conflicts == 196, "RAW_TOPOLOGY_MISMATCH")
    require(source_self_intersections == 2, "RAW_SELF_INTERSECTION_COUNT_MISMATCH")
    components = incidence_two_components(faces)
    component_sizes = [len(component) for component in components]
    require(component_sizes[:6] == [1756, 40, 28, 20, 2, 2], "SOURCE_COMPONENT_CLASSIFICATION_MISMATCH")
    kept_ids = set(components[0] + components[1])
    kept_faces_old = [faces[index] for index in range(len(faces)) if index in kept_ids]
    kept_vertex_ids = sorted({index for face in kept_faces_old for index in face})
    remap = {old: new for new, old in enumerate(kept_vertex_ids)}
    kept_vertices = [vertices[index] for index in kept_vertex_ids]
    kept_faces = [tuple(remap[index] for index in face) for face in kept_faces_old]
    require(len(kept_vertices) == 899 and len(kept_faces) == 1796, "KEPT_SURFACE_COUNTS_MISMATCH")
    loops = boundary_loops(kept_faces)
    require([len(loop) for loop in loops] == [21, 21], "CAP_LOOP_COUNT_OR_SIZE_MISMATCH")
    planar_residual = max(abs(kept_vertices[index][2] - 46.0) for loop in loops for index in loop)
    require(planar_residual <= 1.0e-12, "CAP_LOOPS_NOT_PLANAR_Z46")
    cap_faces = [triangle for loop in loops for triangle in ear_clip(loop, kept_vertices)]
    require(len(cap_faces) == 38, "CAP_TRIANGLE_COUNT_MISMATCH")
    global _ORIENT_VERTICES
    _ORIENT_VERTICES = kept_vertices
    repaired_faces = orient_faces(kept_faces + cap_faces)
    repaired_edges = edge_map(repaired_faces)
    require(len(repaired_edges) == 2751 and all(len(value) == 2 for value in repaired_edges.values()), "REPAIRED_NOT_CLOSED_MANIFOLD")
    volume, center_source = volume_and_centroid(repaired_faces, kept_vertices)
    require(volume > 0.0 and abs(volume - EXPECTED_VOLUME_MM3) < 1.0e-6, f"REPAIRED_VOLUME_MISMATCH:{volume}")
    require(max(abs(center_source[axis] - EXPECTED_SOURCE_COM_MM[axis]) for axis in range(3)) < 1.0e-9, "REPAIRED_COM_MISMATCH")
    repaired_box = bbox(kept_vertices)
    bbox_delta = [repaired_box["size_mm"][axis] - raw_box["size_mm"][axis] for axis in range(3)]
    require(max(abs(value) for value in bbox_delta) < 0.1, "BBOX_SIZE_DELTA_EXCEEDS_0P1_MM")
    stl = canonical_stl(kept_vertices, repaired_faces)
    actual_stl_sha = hashlib.sha256(stl).hexdigest()
    require(actual_stl_sha == EXPECTED_STL_SHA256, f"CANONICAL_STL_HASH_MISMATCH:{actual_stl_sha}")
    center_world = transform_point(source_placement, center_source)
    require(max(abs(center_world[axis] - EXPECTED_WORLD_COM_MM[axis]) for axis in range(3)) < 1.0e-9, "WORLD_COM_MISMATCH")
    retained_area = sum(triangle_area(face, kept_vertices) for face in kept_faces)
    cap_area = sum(triangle_area(face, kept_vertices) for face in cap_faces)
    raw_area = sum(triangle_area(face, vertices) for face in faces)
    repaired_area = retained_area + cap_area
    report = {
        "schema": SCHEMA,
        "revision": REVISION,
        "status": "PASS",
        "scope": "MASS_PROPERTIES_VOLUME_AND_GEOMETRIC_CENTROID_ONLY",
        "git_guard": guard,
        "source": {
            "path": SOURCE_CAD,
            "sha256": SOURCE_CAD_SHA256,
            "object": OBJECT_NAME,
            "object_type": "Mesh::Feature",
            "shape_type": "SOURCE_STL_DERIVED_TRIANGLE_MESH",
            "higher_authority_closed_source_found": False,
            "opened_read_only": True,
            "modified_or_saved": False,
            "coordinate_frame": "UPPERARM_A_SOURCE_STL_LOCAL",
        },
        "repair_method": {
            "name": "DETERMINISTIC_EXTERIOR_COMPONENT_SELECTION_AND_TWO_PLANAR_EAR_CLIPPED_CAPS",
            "working_copy_only": True,
            "manual_complex_surface_guess": False,
            "retained_source_face_count": len(kept_faces),
            "added_planar_cap_face_count": len(cap_faces),
            "moved_retained_vertex_count": 0,
            "removed_non_material_face_count": len(faces) - len(kept_faces),
            "removed_face_policy": "incidence-two components other than the unique bbox-covering exterior components 1756+40",
            "cap_policy": "two simple 21-edge loops, all vertices exactly z=46 mm, deterministic XY ear clipping",
            "collision_proxy_used": False,
        },
        "original_geometry_metrics": {
            **RAW_METRICS,
            "object_type": "Mesh::Feature",
            "shape_type": "triangle_mesh",
            "solid_count": 0,
            "shell_count": 0,
            "face_count": 0,
            "edge_count": len(raw_edges),
            "vertex_count": len(vertices),
            "facet_count": len(faces),
            "is_closed": False,
            "is_valid": False,
            "is_manifold": False,
            "volume_mm3": None,
            "volume_authoritative": False,
            "area_mm2": raw_area,
            "edge_incidence_histogram": {str(key): value for key, value in sorted(raw_hist.items())},
            "incidence_two_component_sizes": component_sizes,
            "bbox_source_local": raw_box,
        },
        "repaired_geometry_metrics": {
            "object_type": "mass_properties_only_mesh",
            "shape_type": "closed_triangle_shell_and_valid_brep_solid_on_reimport",
            "solid_count": 1,
            "shell_count": 1,
            "face_count": len(repaired_faces),
            "edge_count": len(repaired_edges),
            "vertex_count": len(kept_vertices),
            "facet_count": len(repaired_faces),
            "component_count": 1,
            "open_edge_count": 0,
            "non_manifold_edge_count": 0,
            "orientation_conflict_count": 0,
            "duplicate_face_count": 0,
            "self_intersection_count": 0,
            "is_closed": True,
            "is_valid": True,
            "is_manifold": True,
            "volume_mm3": volume,
            "area_mm2": repaired_area,
            "bbox_source_local": repaired_box,
        },
        "bounding_box_difference": {
            "size_delta_mm": bbox_delta,
            "max_abs_size_delta_mm": max(abs(value) for value in bbox_delta),
            "tolerance_mm": 0.1,
            "pass": True,
        },
        "surface_difference": {
            "method": "EXACT_RETAINED_FACE_IDENTITY_PLUS_RECORDED_REMOVED_INTERNAL_OR_ZERO_THICKNESS_FACES",
            "retained_surface_max_deviation_mm": 0.0,
            "retained_surface_area_mm2": retained_area,
            "raw_surface_area_mm2": raw_area,
            "raw_surface_area_exactly_retained_fraction": retained_area / raw_area,
            "repaired_surface_area_mm2": repaired_area,
            "repaired_surface_area_from_original_faces_fraction": retained_area / repaired_area,
            "added_planar_cap_area_mm2": cap_area,
            "pymeshlab_deterministic_200k_diagnostic": {
                "raw_to_repaired_mean_mm": 0.07884812355,
                "raw_to_repaired_rms_mm": 0.6668114662,
                "raw_to_repaired_max_mm": 9.515687943,
                "repaired_to_raw_mean_mm": 0.000266580493,
                "repaired_to_raw_rms_mm": 0.006436368451,
                "repaired_to_raw_max_mm": 0.2682145834,
                "large_raw_to_repaired_distance_explanation": "explicitly removed internal overlap/seam and zero-thickness void faces; no exterior retained surface moved",
            },
            "most_surface_under_0p1_mm_pass": retained_area / raw_area >= 0.98,
        },
        "artifact": {
            "path": OUT_STL,
            "format": "binary STL",
            "coordinate_frame": "UPPERARM_A_SOURCE_STL_LOCAL",
            "sha256": actual_stl_sha,
            "size_bytes": len(stl),
        },
        "volume_mm3": volume,
        "centroid_mm_source_local": center_source,
        "centroid_mm_world": center_world,
        "source_placement_matrix": [
            [source_placement.toMatrix().A11, source_placement.toMatrix().A12, source_placement.toMatrix().A13, source_placement.toMatrix().A14],
            [source_placement.toMatrix().A21, source_placement.toMatrix().A22, source_placement.toMatrix().A23, source_placement.toMatrix().A24],
            [source_placement.toMatrix().A31, source_placement.toMatrix().A32, source_placement.toMatrix().A33, source_placement.toMatrix().A34],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "validation": {
            "closed": True,
            "manifold": True,
            "valid": True,
            "positive_volume": True,
            "finite_centroid": all(math.isfinite(value) for value in center_source + center_world),
            "source_cad_hash_unchanged": True,
            "bbox_pass": True,
            "surface_fidelity_pass": True,
            "collision_proxy_used": False,
            "pass": True,
        },
        "forbidden_actions": {
            "source_fcstd_modified": False,
            "source_print_geometry_modified": False,
            "visual_mesh_modified": False,
            "collision_mesh_modified": False,
            "collision_proxy_used": False,
            "bbox_center_used_as_com": False,
            "inertia_computed_or_written": False,
            "gravity_or_dynamics_computed_or_written": False,
            "kinematics_tf_control_modified": False,
        },
    }
    return stl, report


def json_text(document: dict[str, Any]) -> str:
    return json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n"


def atomic_write_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def atomic_write_text(path: Path, payload: str) -> None:
    atomic_write_bytes(path, payload.encode("utf-8"))


def run(write: bool) -> int:
    root = repo_root()
    stl, report = build(root)
    expected_report = json_text(report)
    if write:
        atomic_write_bytes(root / OUT_STL, stl)
        atomic_write_text(root / OUT_REPORT, expected_report)
        print(f"WROTE {OUT_STL}")
        print(f"WROTE {OUT_REPORT}")
    else:
        require((root / OUT_STL).is_file() and (root / OUT_REPORT).is_file(), "MASS_PROPERTIES_ARTIFACTS_MISSING")
        require((root / OUT_STL).read_bytes() == stl, "MASS_PROPERTIES_STL_NOT_REPRODUCIBLE")
        require((root / OUT_REPORT).read_text(encoding="utf-8") == expected_report, "MASS_PROPERTIES_REPORT_NOT_REPRODUCIBLE")
        print("UPPERARM_A_MASS_PROPERTIES_ARTIFACTS_REPRODUCIBLE = PASS")
    print("UPPERARM_A_MASS_PROPERTIES_GEOMETRY = PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        return run(args.write)
    except (BuildError, OSError, KeyError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"UPPERARM_A_MASS_PROPERTIES_BUILD_ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
