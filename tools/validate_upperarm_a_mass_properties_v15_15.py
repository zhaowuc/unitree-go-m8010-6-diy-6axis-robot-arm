from __future__ import annotations

"""Independently validate the V15.15 UpperArm-A mass-properties artifact.

This validator does not import the builder.  It parses the canonical binary
STL itself, reimports it through FreeCAD Mesh/Part, and opens the accepted
source FCStd read-only to verify the source placement and bounding box.
"""

import hashlib
import json
import math
import struct
import sys
from collections import Counter, defaultdict, deque
from pathlib import Path
from typing import Any

try:
    import FreeCAD as App
    import Mesh
    import Part
except Exception as exc:  # pragma: no cover - exercised outside FreeCAD
    App = Mesh = Part = None
    FREECAD_IMPORT_ERROR = exc
else:
    FREECAD_IMPORT_ERROR = None


SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
SOURCE_CAD_SHA256 = "b4a02a97afaff88e246a46340dad4cacd3c645a7456fdd087a69552e14d3ddc0"
OBJECT_NAME = "UpperArm_A_SleeveSide_PrintPart"
ARTIFACT = "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties.stl"
REPORT = "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json"
EXPECTED_STL_SHA256 = "67aed62b304cfef3a338b0e59b035426b8e012619c333d032e8304c389352f45"
EXPECTED_VERTEX_COUNT = 899
EXPECTED_FACE_COUNT = 1834
EXPECTED_EDGE_COUNT = 2751
EXPECTED_VOLUME_MM3 = 308654.9785581231
EXPECTED_SOURCE_COM_MM = [3414.7028593281334, 1277.1059547527118, 34.687833260140806]
EXPECTED_WORLD_COM_MM = [-0.7894327049961021, 53.98176991520586, 183.0347984497837]
SCHEMA = "go-m8010-arm-v15.15-upperarm-a-mass-properties/1.0"


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def root_from_script() -> Path:
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for candidate in (start, *start.parents):
            if (candidate / ".git").exists() and (candidate / SOURCE_CAD).is_file():
                return candidate
    raise ValidationError("REPO_ROOT_NOT_FOUND")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().lower()


def close(left: float, right: float, tolerance: float, label: str) -> None:
    require(math.isfinite(left) and math.isfinite(right), f"NONFINITE:{label}")
    require(abs(left - right) <= tolerance, f"MISMATCH:{label}:{left}:{right}")


def close_vector(left: list[float], right: list[float], tolerance: float, label: str) -> None:
    require(len(left) == len(right), f"VECTOR_LENGTH_MISMATCH:{label}")
    for index, (actual, expected) in enumerate(zip(left, right)):
        close(float(actual), float(expected), tolerance, f"{label}[{index}]")


def parse_binary_stl(path: Path) -> tuple[bytes, list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    payload = path.read_bytes()
    require(len(payload) >= 84, "STL_TOO_SHORT")
    face_count = struct.unpack_from("<I", payload, 80)[0]
    require(len(payload) == 84 + 50 * face_count, "STL_BINARY_SIZE_MISMATCH")
    vertices: list[tuple[float, float, float]] = []
    vertex_index: dict[tuple[float, float, float], int] = {}
    faces: list[tuple[int, int, int]] = []
    for face_id in range(face_count):
        record = struct.unpack_from("<12fH", payload, 84 + 50 * face_id)
        require(all(math.isfinite(value) for value in record[:12]), f"STL_NONFINITE_FACE:{face_id}")
        face = []
        for offset in (3, 6, 9):
            point = (float(record[offset]), float(record[offset + 1]), float(record[offset + 2]))
            if point not in vertex_index:
                vertex_index[point] = len(vertices)
                vertices.append(point)
            face.append(vertex_index[point])
        require(len(set(face)) == 3, f"STL_DEGENERATE_FACE:{face_id}")
        faces.append(tuple(face))
    return payload, vertices, faces


def edge_occurrences(
    faces: list[tuple[int, int, int]],
) -> dict[tuple[int, int], list[tuple[int, int, int]]]:
    result: dict[tuple[int, int], list[tuple[int, int, int]]] = defaultdict(list)
    for face_id, (a, b, c) in enumerate(faces):
        for start, end in ((a, b), (b, c), (c, a)):
            result[tuple(sorted((start, end)))].append((face_id, start, end))
    return result


def component_count(faces: list[tuple[int, int, int]], edges: dict[Any, Any]) -> int:
    adjacency = [set() for _ in faces]
    for occurrences in edges.values():
        for left_index in range(len(occurrences)):
            for right_index in range(left_index + 1, len(occurrences)):
                left = occurrences[left_index][0]
                right = occurrences[right_index][0]
                adjacency[left].add(right)
                adjacency[right].add(left)
    seen: set[int] = set()
    count = 0
    for seed in range(len(faces)):
        if seed in seen:
            continue
        count += 1
        queue = deque([seed])
        seen.add(seed)
        while queue:
            current = queue.popleft()
            for neighbour in adjacency[current]:
                if neighbour not in seen:
                    seen.add(neighbour)
                    queue.append(neighbour)
    return count


def volume_centroid_area(
    vertices: list[tuple[float, float, float]], faces: list[tuple[int, int, int]]
) -> tuple[float, list[float], float]:
    volume6 = 0.0
    moment = [0.0, 0.0, 0.0]
    area = 0.0
    for face in faces:
        a, b, c = (vertices[index] for index in face)
        ab = [b[index] - a[index] for index in range(3)]
        ac = [c[index] - a[index] for index in range(3)]
        cross_area = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ]
        area += 0.5 * math.sqrt(sum(value * value for value in cross_area))
        cross_bc = [
            b[1] * c[2] - b[2] * c[1],
            b[2] * c[0] - b[0] * c[2],
            b[0] * c[1] - b[1] * c[0],
        ]
        determinant = sum(a[index] * cross_bc[index] for index in range(3))
        volume6 += determinant
        for axis in range(3):
            moment[axis] += determinant * (a[axis] + b[axis] + c[axis])
    volume = volume6 / 6.0
    require(volume > 0.0, f"STL_NONPOSITIVE_OR_REVERSED_VOLUME:{volume}")
    centroid = [value / (24.0 * volume) for value in moment]
    return volume, centroid, area


def bounds(vertices: list[tuple[float, float, float]]) -> dict[str, list[float]]:
    minimum = [min(point[axis] for point in vertices) for axis in range(3)]
    maximum = [max(point[axis] for point in vertices) for axis in range(3)]
    return {
        "min_mm": minimum,
        "max_mm": maximum,
        "size_mm": [maximum[axis] - minimum[axis] for axis in range(3)],
    }


def canonical_triangle(
    triangle: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(sorted(triangle))


def triangle_area(
    triangle: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
) -> float:
    a, b, c = triangle
    ab = [b[index] - a[index] for index in range(3)]
    ac = [c[index] - a[index] for index in range(3)]
    cross = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ]
    return 0.5 * math.sqrt(sum(value * value for value in cross))


def weld_triangles(
    triangles: list[
        tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    ],
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    vertex_index: dict[tuple[float, float, float], int] = {}
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    for triangle in triangles:
        face = []
        for point in triangle:
            if point not in vertex_index:
                vertex_index[point] = len(vertices)
                vertices.append(point)
            face.append(vertex_index[point])
        require(len(set(face)) == 3, "SOURCE_DEGENERATE_FACE")
        faces.append(tuple(face))
    return vertices, faces


def placement_matrix(placement: Any) -> list[list[float]]:
    matrix = placement.toMatrix()
    return [
        [matrix.A11, matrix.A12, matrix.A13, matrix.A14],
        [matrix.A21, matrix.A22, matrix.A23, matrix.A24],
        [matrix.A31, matrix.A32, matrix.A33, matrix.A34],
        [0.0, 0.0, 0.0, 1.0],
    ]


def transform(placement: Any, point: list[float]) -> list[float]:
    result = placement.multVec(App.Vector(*point))
    return [float(result.x), float(result.y), float(result.z)]


def validate() -> dict[str, Any]:
    require(App is not None and Mesh is not None and Part is not None, f"FREECAD_RUNTIME_REQUIRED:{FREECAD_IMPORT_ERROR}")
    root = root_from_script()
    stl_path = root / ARTIFACT
    report_path = root / REPORT
    source_path = root / SOURCE_CAD
    require(stl_path.is_file() and report_path.is_file(), "ARTIFACT_OR_REPORT_MISSING")
    actual_source_hash = sha256(source_path)
    actual_stl_hash = sha256(stl_path)
    require(actual_source_hash == SOURCE_CAD_SHA256, "SOURCE_CAD_HASH_MISMATCH")
    require(actual_stl_hash == EXPECTED_STL_SHA256, "STL_HASH_MISMATCH")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    require(report["schema"] == SCHEMA and report["status"] == "PASS", "REPORT_SCHEMA_OR_STATUS_MISMATCH")
    require(report["source"]["sha256"] == actual_source_hash, "REPORT_SOURCE_HASH_MISMATCH")
    require(report["artifact"]["sha256"] == actual_stl_hash, "REPORT_ARTIFACT_HASH_MISMATCH")
    require(report["artifact"]["path"] == ARTIFACT, "REPORT_ARTIFACT_PATH_MISMATCH")
    require(report["artifact"]["coordinate_frame"] == "UPPERARM_A_SOURCE_STL_LOCAL", "ARTIFACT_FRAME_MISMATCH")

    payload, vertices, faces = parse_binary_stl(stl_path)
    require(len(payload) == report["artifact"]["size_bytes"], "REPORT_ARTIFACT_SIZE_MISMATCH")
    require(len(vertices) == EXPECTED_VERTEX_COUNT, "STL_VERTEX_COUNT_MISMATCH")
    require(len(faces) == EXPECTED_FACE_COUNT, "STL_FACE_COUNT_MISMATCH")
    require(len({tuple(sorted(face)) for face in faces}) == len(faces), "STL_DUPLICATE_FACE")
    edges = edge_occurrences(faces)
    incidence = Counter(len(value) for value in edges.values())
    require(len(edges) == EXPECTED_EDGE_COUNT, "STL_EDGE_COUNT_MISMATCH")
    require(incidence == Counter({2: EXPECTED_EDGE_COUNT}), f"STL_NOT_CLOSED_MANIFOLD:{dict(incidence)}")
    orientation_conflicts = sum(
        1
        for occurrences in edges.values()
        if (occurrences[0][1], occurrences[0][2]) == (occurrences[1][1], occurrences[1][2])
    )
    require(orientation_conflicts == 0, "STL_WINDING_CONFLICT")
    require(component_count(faces, edges) == 1, "STL_DISCONNECTED")
    volume, centroid_source, area = volume_centroid_area(vertices, faces)
    stl_bbox = bounds(vertices)
    close(volume, EXPECTED_VOLUME_MM3, 1.0e-6, "STL_VOLUME_MM3")
    close_vector(centroid_source, EXPECTED_SOURCE_COM_MM, 1.0e-9, "STL_SOURCE_COM_MM")
    close(float(report["volume_mm3"]), volume, 1.0e-6, "REPORT_VOLUME_MM3")
    close_vector(report["centroid_mm_source_local"], centroid_source, 1.0e-9, "REPORT_SOURCE_COM_MM")
    close(area, float(report["repaired_geometry_metrics"]["area_mm2"]), 1.0e-6, "REPORT_AREA_MM2")
    for key in ("min_mm", "max_mm", "size_mm"):
        close_vector(stl_bbox[key], report["repaired_geometry_metrics"]["bbox_source_local"][key], 1.0e-9, f"REPORT_BBOX_{key}")

    imported_mesh = Mesh.Mesh(str(stl_path))
    require(imported_mesh.CountPoints == EXPECTED_VERTEX_COUNT, "FREECAD_MESH_POINT_COUNT_MISMATCH")
    require(imported_mesh.CountFacets == EXPECTED_FACE_COUNT, "FREECAD_MESH_FACET_COUNT_MISMATCH")
    require(imported_mesh.countComponents() == 1, "FREECAD_MESH_COMPONENT_COUNT_MISMATCH")
    require(imported_mesh.isSolid(), "FREECAD_MESH_NOT_SOLID")
    require(not imported_mesh.hasNonManifolds(), "FREECAD_MESH_NON_MANIFOLD")
    self_intersections = imported_mesh.getSelfIntersections()
    require(len(self_intersections) == 0, f"FREECAD_MESH_SELF_INTERSECTIONS:{len(self_intersections)}")
    shape = Part.Shape()
    shape.makeShapeFromMesh(imported_mesh.Topology, 1.0e-6)
    require(shape.isClosed() and shape.isValid(), "FREECAD_REIMPORTED_SHELL_INVALID")
    require(len(shape.Shells) == 1, "FREECAD_REIMPORTED_SHELL_COUNT_MISMATCH")
    solid = Part.makeSolid(shape.Shells[0])
    require(solid.isClosed() and solid.isValid(), "FREECAD_REIMPORTED_SOLID_INVALID")
    require(len(solid.Solids) == 1, "FREECAD_REIMPORTED_SOLID_COUNT_MISMATCH")
    close(float(solid.Volume), volume, 1.0e-6, "FREECAD_SOLID_VOLUME_MM3")
    close_vector(
        [float(solid.CenterOfMass.x), float(solid.CenterOfMass.y), float(solid.CenterOfMass.z)],
        centroid_source,
        1.0e-8,
        "FREECAD_SOLID_COM_MM",
    )

    before_hash = sha256(source_path)
    document = App.openDocument(str(source_path))
    try:
        source_object = document.getObject(OBJECT_NAME)
        require(source_object is not None and source_object.TypeId == "Mesh::Feature", "SOURCE_OBJECT_TYPE_MISMATCH")
        source_placement = source_object.Mesh.Placement
        source_copy = Mesh.Mesh(source_object.Mesh)
        source_copy.Placement = App.Placement()
        source_triangles = [
            tuple(tuple(float(value) for value in point) for point in facet.Points)
            for facet in source_copy.Facets
        ]
        source_bbox = {
            "min_mm": [source_copy.BoundBox.XMin, source_copy.BoundBox.YMin, source_copy.BoundBox.ZMin],
            "max_mm": [source_copy.BoundBox.XMax, source_copy.BoundBox.YMax, source_copy.BoundBox.ZMax],
            "size_mm": [source_copy.BoundBox.XLength, source_copy.BoundBox.YLength, source_copy.BoundBox.ZLength],
        }
        require(source_copy.CountPoints == 1036 and source_copy.CountFacets == 1913, "SOURCE_RAW_COUNTS_MISMATCH")
        require(not source_copy.isSolid() and source_copy.hasNonManifolds(), "SOURCE_RAW_TOPOLOGY_UNEXPECTED")
        require(len(source_copy.getSelfIntersections()) == 2, "SOURCE_SELF_INTERSECTION_COUNT_MISMATCH")
        matrix = placement_matrix(source_placement)
        centroid_world = transform(source_placement, centroid_source)
    finally:
        App.closeDocument(document.Name)
    require(sha256(source_path) == before_hash == SOURCE_CAD_SHA256, "SOURCE_CAD_CHANGED_DURING_VALIDATION")
    for key in ("min_mm", "max_mm", "size_mm"):
        close_vector(source_bbox[key], stl_bbox[key], 1.0e-9, f"SOURCE_VS_STL_BBOX_{key}")
        close_vector(source_bbox[key], report["original_geometry_metrics"]["bbox_source_local"][key], 1.0e-9, f"SOURCE_VS_REPORT_BBOX_{key}")
    for row in range(4):
        close_vector(matrix[row], report["source_placement_matrix"][row], 1.0e-12, f"SOURCE_PLACEMENT_ROW_{row}")
    close_vector(centroid_world, EXPECTED_WORLD_COM_MM, 1.0e-9, "WORLD_COM_MM")
    close_vector(report["centroid_mm_world"], centroid_world, 1.0e-9, "REPORT_WORLD_COM_MM")

    source_vertices, source_faces = weld_triangles(source_triangles)
    require(len(source_vertices) == 1036 and len(source_faces) == 1913, "SOURCE_WELDED_COUNTS_MISMATCH")
    source_edges = edge_occurrences(source_faces)
    source_incidence = Counter(len(value) for value in source_edges.values())
    require(source_incidence == Counter({1: 235, 2: 2671, 3: 54}), "SOURCE_EDGE_INCIDENCE_MISMATCH")
    source_orientation_conflicts = sum(
        1
        for occurrences in source_edges.values()
        if len(occurrences) == 2
        and (occurrences[0][1], occurrences[0][2]) == (occurrences[1][1], occurrences[1][2])
    )
    require(source_orientation_conflicts == 196, "SOURCE_ORIENTATION_CONFLICT_COUNT_MISMATCH")
    original = report["original_geometry_metrics"]
    require(original["open_edges"] == 235, "REPORT_SOURCE_OPEN_EDGE_COUNT_MISMATCH")
    require(original["non_manifold_edges"] == 54, "REPORT_SOURCE_NONMANIFOLD_EDGE_COUNT_MISMATCH")
    require(original["orientation_conflicts"] == 196, "REPORT_SOURCE_ORIENTATION_COUNT_MISMATCH")
    require(original["self_intersection_pairs"] == 2, "REPORT_SOURCE_SELF_INTERSECTION_COUNT_MISMATCH")

    source_triangle_set = {canonical_triangle(triangle) for triangle in source_triangles}
    repaired_triangles = [tuple(vertices[index] for index in face) for face in faces]
    repaired_triangle_set = {canonical_triangle(triangle) for triangle in repaired_triangles}
    require(len(source_triangle_set) == 1913, "SOURCE_DUPLICATE_FACE")
    require(len(repaired_triangle_set) == 1834, "REPAIRED_DUPLICATE_FACE")
    retained_triangles = source_triangle_set & repaired_triangle_set
    removed_triangles = source_triangle_set - repaired_triangle_set
    added_triangles = repaired_triangle_set - source_triangle_set
    require(len(retained_triangles) == 1796, "RETAINED_SOURCE_FACE_COUNT_MISMATCH")
    require(len(removed_triangles) == 117, "REMOVED_SOURCE_FACE_COUNT_MISMATCH")
    require(len(added_triangles) == 38, "ADDED_PLANAR_CAP_FACE_COUNT_MISMATCH")
    require(all(max(point[2] for point in triangle) - min(point[2] for point in triangle) <= 1.0e-12 for triangle in added_triangles), "ADDED_CAP_NOT_PLANAR")
    require(all(abs(point[2] - 46.0) <= 1.0e-12 for triangle in added_triangles for point in triangle), "ADDED_CAP_NOT_AT_Z46")
    raw_area = sum(triangle_area(triangle) for triangle in source_triangle_set)
    retained_area = sum(triangle_area(triangle) for triangle in retained_triangles)
    added_area = sum(triangle_area(triangle) for triangle in added_triangles)
    repaired_area = retained_area + added_area
    surface = report["surface_difference"]
    close(raw_area, float(surface["raw_surface_area_mm2"]), 1.0e-6, "SOURCE_SURFACE_AREA_MM2")
    close(retained_area, float(surface["retained_surface_area_mm2"]), 1.0e-6, "RETAINED_SURFACE_AREA_MM2")
    close(added_area, float(surface["added_planar_cap_area_mm2"]), 1.0e-6, "ADDED_CAP_AREA_MM2")
    close(repaired_area, float(surface["repaired_surface_area_mm2"]), 1.0e-6, "REPAIRED_SURFACE_AREA_MM2")
    close(retained_area / raw_area, float(surface["raw_surface_area_exactly_retained_fraction"]), 1.0e-12, "RAW_RETAINED_FRACTION")
    close(retained_area / repaired_area, float(surface["repaired_surface_area_from_original_faces_fraction"]), 1.0e-12, "REPAIRED_ORIGINAL_FACE_FRACTION")
    require(retained_area / raw_area >= 0.98, "SOURCE_SURFACE_FIDELITY_BELOW_98_PERCENT")
    require(retained_area / repaired_area >= 0.999, "REPAIRED_SURFACE_ORIGINAL_FRACTION_BELOW_99P9_PERCENT")

    repaired = report["repaired_geometry_metrics"]
    require(
        repaired["vertex_count"] == EXPECTED_VERTEX_COUNT
        and repaired["face_count"] == EXPECTED_FACE_COUNT
        and repaired["edge_count"] == EXPECTED_EDGE_COUNT,
        "REPORT_REPAIRED_COUNTS_MISMATCH",
    )
    for key in ("is_closed", "is_valid", "is_manifold"):
        require(repaired[key] is True, f"REPORT_REPAIRED_FLAG_FALSE:{key}")
    require(repaired["self_intersection_count"] == 0, "REPORT_SELF_INTERSECTION_COUNT_NONZERO")
    validation = report["validation"]
    require(validation["pass"] is True, "REPORT_VALIDATION_FAIL")
    for key in ("closed", "manifold", "valid", "positive_volume", "finite_centroid", "source_cad_hash_unchanged", "bbox_pass", "surface_fidelity_pass"):
        require(validation[key] is True, f"REPORT_VALIDATION_FLAG_FALSE:{key}")
    require(validation["collision_proxy_used"] is False, "REPORT_COLLISION_PROXY_USED")
    require(all(value is False for value in report["forbidden_actions"].values()), "REPORT_FORBIDDEN_ACTION_TRUE")
    require(report["repair_method"]["collision_proxy_used"] is False, "REPAIR_COLLISION_PROXY_USED")
    require(report["repair_method"]["moved_retained_vertex_count"] == 0, "RETAINED_VERTEX_MOVED")
    require(report["bounding_box_difference"]["pass"] is True, "REPORT_BBOX_DIFFERENCE_FAIL")
    require(report["surface_difference"]["most_surface_under_0p1_mm_pass"] is True, "REPORT_SURFACE_FIDELITY_FAIL")

    return {
        "status": "UPPERARM_A_MASS_PROPERTIES_VALIDATION = PASS",
        "stl_sha256": actual_stl_hash,
        "source_cad_sha256": actual_source_hash,
        "vertices": len(vertices),
        "faces": len(faces),
        "edges": len(edges),
        "closed": True,
        "manifold": True,
        "self_intersections": 0,
        "part_valid_solid": True,
        "volume_mm3": volume,
        "centroid_mm_source_local": centroid_source,
        "centroid_mm_world": centroid_world,
        "source_fcstd_modified": False,
        "collision_proxy_used": False,
    }


def main() -> int:
    try:
        print(json.dumps(validate(), ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (ValidationError, OSError, KeyError, ValueError, struct.error) as exc:
        print(f"UPPERARM_A_MASS_PROPERTIES_VALIDATION_ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
