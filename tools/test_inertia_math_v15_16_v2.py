#!/usr/bin/env python3
"""Self-contained analytic tests for the V15.16 V2 inertia mathematics.

No robot-arm CAD, ledger, mesh artifact, or project data file is read here.
The tests integrate oriented boundary triangles as signed tetrahedra about the
mesh bounding-box center.  They independently exercise volume, first and
second moments, COM inertia, the matrix off-diagonal sign convention,
rotation, the parallel-axis theorem, millimetre-to-SI conversion, and
invariance to non-uniform boundary vertex density.
"""

from __future__ import print_function

import math
import sys


RELATIVE_TOLERANCE = 1.0e-10
MM_TO_M = 1.0e-3
MM2_TO_M2 = 1.0e-6

# Deliberately scalene, with no dimension symmetry that could hide an axis or
# rotation error.  Geometry is expressed in CAD millimetres; mass is kg.
BOX_DIMS_MM = (820.0, 470.0, 310.0)
BOX_MASS_KG = 7.3


def vec_add(a, b):
    return tuple(a[index] + b[index] for index in range(3))


def vec_sub(a, b):
    return tuple(a[index] - b[index] for index in range(3))


def vec_scale(scale, vector):
    return tuple(scale * vector[index] for index in range(3))


def dot(a, b):
    return math.fsum(a[index] * b[index] for index in range(3))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vec_norm(vector):
    return math.sqrt(dot(vector, vector))


def identity_matrix():
    return (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )


def diagonal_matrix(values):
    return (
        (values[0], 0.0, 0.0),
        (0.0, values[1], 0.0),
        (0.0, 0.0, values[2]),
    )


def matrix_add(a, b):
    return tuple(
        tuple(a[row][column] + b[row][column] for column in range(3))
        for row in range(3)
    )


def matrix_sub(a, b):
    return tuple(
        tuple(a[row][column] - b[row][column] for column in range(3))
        for row in range(3)
    )


def matrix_scale(scale, matrix):
    return tuple(
        tuple(scale * matrix[row][column] for column in range(3))
        for row in range(3)
    )


def matrix_transpose(matrix):
    return tuple(
        tuple(matrix[column][row] for column in range(3))
        for row in range(3)
    )


def matrix_multiply(a, b):
    return tuple(
        tuple(
            math.fsum(a[row][index] * b[index][column]
                      for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def matrix_vector_multiply(matrix, vector):
    return tuple(
        math.fsum(matrix[row][column] * vector[column]
                  for column in range(3))
        for row in range(3)
    )


def outer(a, b):
    return tuple(
        tuple(a[row] * b[column] for column in range(3))
        for row in range(3)
    )


def matrix_trace(matrix):
    return math.fsum(matrix[index][index] for index in range(3))


def frobenius_norm(matrix):
    return math.sqrt(math.fsum(
        matrix[row][column] * matrix[row][column]
        for row in range(3)
        for column in range(3)
    ))


def rotate_tensor(rotation, tensor):
    return matrix_multiply(
        matrix_multiply(rotation, tensor),
        matrix_transpose(rotation),
    )


def parallel_axis_shift(inertia_com, mass_kg, displacement_m):
    """Shift COM inertia to a reference point.

    ``displacement_m`` is COM minus reference, expressed in the same axes.
    Matrix off-diagonals therefore acquire ``-mass*d_i*d_j`` exactly as
    required by the project's negative-product inertia convention.
    """
    distance_squared = dot(displacement_m, displacement_m)
    shift = matrix_sub(
        matrix_scale(distance_squared, identity_matrix()),
        outer(displacement_m, displacement_m),
    )
    return matrix_add(inertia_com, matrix_scale(mass_kg, shift))


def rotation_x(angle_deg):
    angle = math.radians(angle_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (1.0, 0.0, 0.0),
        (0.0, cosine, -sine),
        (0.0, sine, cosine),
    )


def rotation_y(angle_deg):
    angle = math.radians(angle_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (cosine, 0.0, sine),
        (0.0, 1.0, 0.0),
        (-sine, 0.0, cosine),
    )


def rotation_z(angle_deg):
    angle = math.radians(angle_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (cosine, -sine, 0.0),
        (sine, cosine, 0.0),
        (0.0, 0.0, 1.0),
    )


def specified_rotation():
    """Apply Rx(13 deg), then Ry(-21 deg), then Rz(37 deg)."""
    return matrix_multiply(
        rotation_z(37.0),
        matrix_multiply(rotation_y(-21.0), rotation_x(13.0)),
    )


def analytic_box_inertia_com(mass_kg, dimensions_mm):
    a, b, c = tuple(value * MM_TO_M for value in dimensions_mm)
    return diagonal_matrix((
        mass_kg * (b * b + c * c) / 12.0,
        mass_kg * (a * a + c * c) / 12.0,
        mass_kg * (a * a + b * b) / 12.0,
    ))


def default_knots(dimensions_mm):
    return tuple(
        (-0.5 * length, 0.5 * length)
        for length in dimensions_mm
    )


def validate_knots(knots, dimensions_mm):
    if len(knots) != 3:
        raise AssertionError("three axis knot sequences are required")
    for axis, (axis_knots, length) in enumerate(zip(knots, dimensions_mm)):
        if len(axis_knots) < 2:
            raise AssertionError("axis {} has fewer than two knots".format(axis))
        expected_ends = (-0.5 * length, 0.5 * length)
        if (axis_knots[0], axis_knots[-1]) != expected_ends:
            raise AssertionError(
                "axis {} knots do not span exact box bounds".format(axis)
            )
        if any(not left < right
               for left, right in zip(axis_knots, axis_knots[1:])):
            raise AssertionError(
                "axis {} knots are not strictly increasing".format(axis)
            )


def transform_point(local_point, rotation, center_mm):
    return vec_add(matrix_vector_multiply(rotation, local_point), center_mm)


def append_grid_face(triangles, constant_axis, constant_value,
                     u_axis, u_knots, v_axis, v_knots, reverse,
                     rotation, center_mm):
    """Append one conforming gridded face with explicitly outward winding."""
    for u0, u1 in zip(u_knots, u_knots[1:]):
        for v0, v1 in zip(v_knots, v_knots[1:]):
            local_points = []
            for u_value, v_value in (
                    (u0, v0), (u1, v0), (u1, v1), (u0, v1)):
                point = [0.0, 0.0, 0.0]
                point[constant_axis] = constant_value
                point[u_axis] = u_value
                point[v_axis] = v_value
                local_points.append(tuple(point))
            p00, p10, p11, p01 = tuple(
                transform_point(point, rotation, center_mm)
                for point in local_points
            )
            if reverse:
                triangles.extend(((p00, p11, p10), (p00, p01, p11)))
            else:
                triangles.extend(((p00, p10, p11), (p00, p11, p01)))


def build_box_mesh(dimensions_mm, center_mm=(0.0, 0.0, 0.0),
                   rotation=None, knots=None):
    """Return an outward, closed triangle boundary for an exact box.

    The three global local-coordinate knot sequences are shared by all faces,
    so even strongly non-uniform subdivisions remain topologically conforming
    along every box edge (no T-junctions).
    """
    if rotation is None:
        rotation = identity_matrix()
    if knots is None:
        knots = default_knots(dimensions_mm)
    knots = tuple(tuple(axis_knots) for axis_knots in knots)
    validate_knots(knots, dimensions_mm)
    half = tuple(0.5 * value for value in dimensions_mm)

    triangles = []
    # With increasing physical coordinates:
    #   cross(+Y,+Z)=+X, cross(+X,+Z)=-Y, cross(+X,+Y)=+Z.
    append_grid_face(triangles, 0, +half[0], 1, knots[1], 2, knots[2],
                     False, rotation, center_mm)
    append_grid_face(triangles, 0, -half[0], 1, knots[1], 2, knots[2],
                     True, rotation, center_mm)
    append_grid_face(triangles, 1, +half[1], 0, knots[0], 2, knots[2],
                     True, rotation, center_mm)
    append_grid_face(triangles, 1, -half[1], 0, knots[0], 2, knots[2],
                     False, rotation, center_mm)
    append_grid_face(triangles, 2, +half[2], 0, knots[0], 1, knots[1],
                     False, rotation, center_mm)
    append_grid_face(triangles, 2, -half[2], 0, knots[0], 1, knots[1],
                     True, rotation, center_mm)
    return triangles


def validate_closed_oriented_surface(triangles):
    """Require a finite, non-degenerate, oriented closed 2-manifold."""
    edge_incidence = {}
    for triangle_index, triangle in enumerate(triangles):
        if len(triangle) != 3 or len(set(triangle)) != 3:
            raise AssertionError(
                "triangle {} repeats a vertex".format(triangle_index)
            )
        if not all(math.isfinite(value) for point in triangle for value in point):
            raise AssertionError(
                "triangle {} has non-finite coordinates".format(triangle_index)
            )
        double_area = vec_norm(cross(
            vec_sub(triangle[1], triangle[0]),
            vec_sub(triangle[2], triangle[0]),
        ))
        if not math.isfinite(double_area) or double_area <= 0.0:
            raise AssertionError(
                "triangle {} is degenerate".format(triangle_index)
            )
        directed_edges = (
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        )
        for start, end in directed_edges:
            key = tuple(sorted((start, end)))
            direction = 1 if (start, end) == key else -1
            edge_incidence.setdefault(key, []).append(direction)

    bad_edges = [
        (edge, directions)
        for edge, directions in edge_incidence.items()
        if len(directions) != 2 or sum(directions) != 0
    ]
    if bad_edges:
        raise AssertionError(
            "surface is not a consistently oriented closed 2-manifold: "
            "{} bad edges".format(len(bad_edges))
        )


def bounding_box_center(triangles):
    vertices = [point for triangle in triangles for point in triangle]
    return tuple(
        0.5 * (
            min(point[axis] for point in vertices)
            + max(point[axis] for point in vertices)
        )
        for axis in range(3)
    )


def anchored_signed_tetrahedron_properties(triangles, mass_kg,
                                           reference_mm=(0.0, 0.0, 0.0)):
    """Integrate full uniform-density mass properties from boundary triangles.

    Each oriented boundary triangle and the stable bounding-box-center anchor
    form a signed tetrahedron.  First moments are accumulated in mm^4 and raw
    second moments ``integral(x_i*x_j)dV`` in mm^5.  The physical orientation
    sign is normalized only after summing, and inertia is formed as
    ``trace(Q)E - Q``; therefore matrix off-diagonals are the required
    ``-integral(x_i*x_j)dm`` rather than unsigned products of inertia.
    """
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise AssertionError("mass must be finite and positive")
    validate_closed_oriented_surface(triangles)
    anchor = bounding_box_center(triangles)

    signed_volume_terms = []
    signed_first_terms = [[] for _axis in range(3)]
    signed_second_terms = [
        [[] for _column in range(3)]
        for _row in range(3)
    ]

    for p0, p1, p2 in triangles:
        a = vec_sub(p0, anchor)
        b = vec_sub(p1, anchor)
        c = vec_sub(p2, anchor)
        signed_volume = dot(a, cross(b, c)) / 6.0
        signed_volume_terms.append(signed_volume)
        coordinate_sum = tuple(a[axis] + b[axis] + c[axis]
                               for axis in range(3))
        for row in range(3):
            signed_first_terms[row].append(
                signed_volume * coordinate_sum[row] / 4.0
            )
            for column in range(3):
                vertex_dot = (
                    a[row] * a[column]
                    + b[row] * b[column]
                    + c[row] * c[column]
                )
                # Uniform tetrahedron barycentric moments:
                # E[lambda_i^2]=1/10 and E[lambda_i*lambda_j]=1/20.
                signed_second_terms[row][column].append(
                    signed_volume
                    * (coordinate_sum[row] * coordinate_sum[column]
                       + vertex_dot)
                    / 20.0
                )

    signed_volume_mm3 = math.fsum(signed_volume_terms)
    if not math.isfinite(signed_volume_mm3) or signed_volume_mm3 == 0.0:
        raise AssertionError("signed polyhedron volume is non-finite or zero")
    orientation_sign = 1.0 if signed_volume_mm3 > 0.0 else -1.0
    volume_mm3 = abs(signed_volume_mm3)
    first_anchor_mm4 = tuple(
        orientation_sign * math.fsum(signed_first_terms[axis])
        for axis in range(3)
    )
    second_anchor_mm5 = tuple(
        tuple(
            orientation_sign * math.fsum(signed_second_terms[row][column])
            for column in range(3)
        )
        for row in range(3)
    )

    com_offset_mm = tuple(value / volume_mm3 for value in first_anchor_mm4)
    com_mm = vec_add(anchor, com_offset_mm)
    central_second_mm5 = matrix_sub(
        second_anchor_mm5,
        matrix_scale(volume_mm3, outer(com_offset_mm, com_offset_mm)),
    )
    raw_inertia_com_mm5 = matrix_sub(
        matrix_scale(matrix_trace(central_second_mm5), identity_matrix()),
        central_second_mm5,
    )
    density_kg_per_mm3 = mass_kg / volume_mm3
    inertia_com_kg_m2 = matrix_scale(
        density_kg_per_mm3 * MM2_TO_M2,
        raw_inertia_com_mm5,
    )

    displacement_m = vec_scale(MM_TO_M, vec_sub(com_mm, reference_mm))
    inertia_reference_kg_m2 = parallel_axis_shift(
        inertia_com_kg_m2, mass_kg, displacement_m
    )

    # Also expose global geometric moments for callers that audit each stage.
    first_global_mm4 = vec_add(
        first_anchor_mm4,
        vec_scale(volume_mm3, anchor),
    )
    second_global_mm5 = matrix_add(
        matrix_add(
            second_anchor_mm5,
            outer(anchor, first_anchor_mm4),
        ),
        matrix_add(
            outer(first_anchor_mm4, anchor),
            matrix_scale(volume_mm3, outer(anchor, anchor)),
        ),
    )

    finite_values = (
        [signed_volume_mm3, volume_mm3, density_kg_per_mm3]
        + list(anchor)
        + list(com_mm)
        + list(first_global_mm4)
        + [second_global_mm5[row][column]
           for row in range(3) for column in range(3)]
        + [inertia_com_kg_m2[row][column]
           for row in range(3) for column in range(3)]
        + [inertia_reference_kg_m2[row][column]
           for row in range(3) for column in range(3)]
    )
    if not all(math.isfinite(value) for value in finite_values):
        raise AssertionError("polyhedral mass properties contain non-finite data")

    return {
        "anchor_mm": anchor,
        "signed_volume_mm3": signed_volume_mm3,
        "volume_mm3": volume_mm3,
        "first_moment_mm4": first_global_mm4,
        "second_moment_mm5": second_global_mm5,
        "central_second_moment_mm5": central_second_mm5,
        "raw_inertia_com_mm5": raw_inertia_com_mm5,
        "density_kg_per_mm3": density_kg_per_mm3,
        "com_mm": com_mm,
        "inertia_com_kg_m2": inertia_com_kg_m2,
        "reference_mm": reference_mm,
        "inertia_reference_kg_m2": inertia_reference_kg_m2,
    }


def unique_vertices(triangles):
    return sorted(set(point for triangle in triangles for point in triangle))


def vertex_mean(triangles):
    vertices = unique_vertices(triangles)
    count = float(len(vertices))
    return tuple(
        math.fsum(point[axis] for point in vertices) / count
        for axis in range(3)
    )


def relative_scalar_error(actual, expected):
    if expected == 0.0:
        return abs(actual)
    return abs(actual - expected) / abs(expected)


def relative_vector_error(actual, expected, characteristic):
    return vec_norm(vec_sub(actual, expected)) / characteristic


def relative_matrix_error(actual, expected):
    denominator = frobenius_norm(expected)
    if denominator == 0.0:
        return frobenius_norm(actual)
    return frobenius_norm(matrix_sub(actual, expected)) / denominator


def require_less(label, actual, limit=RELATIVE_TOLERANCE):
    if not math.isfinite(actual) or not actual < limit:
        raise AssertionError(
            "{}: expected {:.17g} < {:.17g}".format(label, actual, limit)
        )


def require_greater(label, actual, limit):
    if not math.isfinite(actual) or not actual > limit:
        raise AssertionError(
            "{}: expected {:.17g} > {:.17g}".format(label, actual, limit)
        )


def format_vector(vector):
    return "[{}]".format(", ".join("{:.17g}".format(value)
                                   for value in vector))


def format_matrix(matrix):
    return "[{}]".format(
        ", ".join(format_vector(row) for row in matrix)
    )


def test_axis_aligned_box():
    center_mm = (3250.125, -2110.75, 987.5)
    triangles = build_box_mesh(BOX_DIMS_MM, center_mm=center_mm)
    properties = anchored_signed_tetrahedron_properties(
        triangles, BOX_MASS_KG, reference_mm=center_mm
    )
    expected_volume_mm3 = math.prod(BOX_DIMS_MM)
    expected_inertia = analytic_box_inertia_com(BOX_MASS_KG, BOX_DIMS_MM)
    characteristic_mm = vec_norm(BOX_DIMS_MM)

    volume_error = relative_scalar_error(
        properties["volume_mm3"], expected_volume_mm3
    )
    com_error = relative_vector_error(
        properties["com_mm"], center_mm, characteristic_mm
    )
    inertia_error = relative_matrix_error(
        properties["inertia_com_kg_m2"], expected_inertia
    )
    diagonal_scale = max(expected_inertia[index][index]
                         for index in range(3))
    off_diagonal_ratio = max(
        abs(properties["inertia_com_kg_m2"][row][column])
        for row, column in ((0, 1), (0, 2), (1, 2))
    ) / diagonal_scale

    require_less("axis-aligned box volume relative error", volume_error)
    require_less("axis-aligned box COM scaled error", com_error)
    require_less("axis-aligned box inertia relative error", inertia_error)
    require_less("axis-aligned box off-diagonal ratio", off_diagonal_ratio)
    if properties["signed_volume_mm3"] <= 0.0:
        raise AssertionError("axis-aligned box winding is not positive")

    print("PASS A: axis-aligned box")
    print("  bbox-center anchor mm       = {}".format(
        format_vector(properties["anchor_mm"])
    ))
    print("  volume relative error       = {:.17g}".format(volume_error))
    print("  COM scaled error            = {:.17g}".format(com_error))
    print("  inertia relative error      = {:.17g}".format(inertia_error))
    print("  off-diagonal/diag scale     = {:.17g}".format(
        off_diagonal_ratio
    ))

    # Independent dimensional audit of the exact required conversion:
    # mm^5 * (kg/mm^3) = kg*mm^2, then *1e-6 = kg*m^2.
    inertia_via_explicit_units = matrix_scale(
        properties["density_kg_per_mm3"] * MM2_TO_M2,
        properties["raw_inertia_com_mm5"],
    )
    conversion_identity = abs(
        (1.0e-15 * 1.0e9) - MM2_TO_M2
    )
    unit_matrix_error = relative_matrix_error(
        inertia_via_explicit_units, expected_inertia
    )
    require_less("mm^5 density SI identity", conversion_identity)
    require_less("mm to SI inertia audit", unit_matrix_error)
    print("PASS UNIT_AUDIT: raw_mm5 * rho_kg_per_mm3 * 1e-6 = kg*m^2")
    print("  unit inertia relative error = {:.17g}".format(unit_matrix_error))
    return expected_inertia


def test_rotated_box(base_inertia):
    rotation = specified_rotation()
    triangles = build_box_mesh(
        BOX_DIMS_MM, rotation=rotation, center_mm=(0.0, 0.0, 0.0)
    )
    properties = anchored_signed_tetrahedron_properties(
        triangles, BOX_MASS_KG
    )
    expected = rotate_tensor(rotation, base_inertia)
    inertia_error = relative_matrix_error(
        properties["inertia_com_kg_m2"], expected
    )
    com_error = vec_norm(properties["com_mm"]) / vec_norm(BOX_DIMS_MM)
    require_less("rotated box COM scaled error", com_error)
    require_less("rotated box inertia relative error", inertia_error)

    # A scalene box under this non-special rotation must have all three
    # nonzero off-diagonals.  Matching R*I*R^T verifies their negative-product
    # matrix signs, not merely the principal moments.
    off_diagonal_scale = max(abs(expected[row][column])
                             for row, column in ((0, 1), (0, 2), (1, 2)))
    require_greater("rotated analytic off-diagonal coverage",
                    off_diagonal_scale, 1.0e-3)

    print("PASS B: rotated box Rx(13deg), Ry(-21deg), Rz(37deg)")
    print("  rotation convention         = Rz * Ry * Rx")
    print("  expected R*I*R^T            = {}".format(format_matrix(expected)))
    print("  tetrahedron inertia         = {}".format(
        format_matrix(properties["inertia_com_kg_m2"])
    ))
    print("  inertia relative error      = {:.17g}".format(inertia_error))
    print("PASS SIGN_CONVENTION: matrix off-diagonal = -integral(x_i*x_j) dm")
    return rotation, expected


def test_parallel_axis(base_inertia):
    displacement_m = (0.11, -0.07, 0.05)
    reference_mm = vec_scale(-1.0 / MM_TO_M, displacement_m)
    triangles = build_box_mesh(BOX_DIMS_MM)
    properties = anchored_signed_tetrahedron_properties(
        triangles, BOX_MASS_KG, reference_mm=reference_mm
    )
    expected = parallel_axis_shift(
        base_inertia, BOX_MASS_KG, displacement_m
    )
    inertia_error = relative_matrix_error(
        properties["inertia_reference_kg_m2"], expected
    )
    require_less("parallel-axis relative error", inertia_error)

    print("PASS C: parallel-axis theorem")
    print("  d = COM - reference (m)     = {}".format(
        format_vector(displacement_m)
    ))
    print("  shifted inertia             = {}".format(
        format_matrix(properties["inertia_reference_kg_m2"])
    ))
    print("  inertia relative error      = {:.17g}".format(inertia_error))


def test_rotation_and_translation(rotation, rotated_inertia):
    translation_m = (1.35, -0.82, 0.46)
    translation_mm = vec_scale(1.0 / MM_TO_M, translation_m)
    triangles = build_box_mesh(
        BOX_DIMS_MM, rotation=rotation, center_mm=translation_mm
    )
    properties = anchored_signed_tetrahedron_properties(
        triangles, BOX_MASS_KG, reference_mm=(0.0, 0.0, 0.0)
    )
    expected = parallel_axis_shift(
        rotated_inertia, BOX_MASS_KG, translation_m
    )
    inertia_error = relative_matrix_error(
        properties["inertia_reference_kg_m2"], expected
    )
    com_error = relative_vector_error(
        properties["com_mm"], translation_mm, vec_norm(BOX_DIMS_MM)
    )
    require_less("rotation plus translation COM scaled error", com_error)
    require_less("rotation plus translation inertia relative error",
                 inertia_error)

    print("PASS D: rotation + translation about world origin")
    print("  translation m               = {}".format(
        format_vector(translation_m)
    ))
    print("  world-origin inertia        = {}".format(
        format_matrix(properties["inertia_reference_kg_m2"])
    ))
    print("  inertia relative error      = {:.17g}".format(inertia_error))


def test_nonuniform_triangulation(base_inertia):
    center_mm = (250.0, -125.0, 80.0)
    coarse_triangles = build_box_mesh(BOX_DIMS_MM, center_mm=center_mm)
    dense_knots = (
        (-410.0, 120.0, 260.0, 330.0, 370.0, 392.0, 404.0, 410.0),
        (-235.0, -100.0, 60.0, 170.0, 235.0),
        (-155.0, -50.0, 35.0, 100.0, 135.0, 149.0, 155.0),
    )
    dense_triangles = build_box_mesh(
        BOX_DIMS_MM, center_mm=center_mm, knots=dense_knots
    )
    coarse = anchored_signed_tetrahedron_properties(
        coarse_triangles, BOX_MASS_KG, reference_mm=center_mm
    )
    dense = anchored_signed_tetrahedron_properties(
        dense_triangles, BOX_MASS_KG, reference_mm=center_mm
    )

    characteristic_mm = vec_norm(BOX_DIMS_MM)
    volume_relative_difference = relative_scalar_error(
        dense["volume_mm3"], coarse["volume_mm3"]
    )
    com_scaled_difference = relative_vector_error(
        dense["com_mm"], coarse["com_mm"], characteristic_mm
    )
    inertia_relative_difference = relative_matrix_error(
        dense["inertia_com_kg_m2"], coarse["inertia_com_kg_m2"]
    )
    dense_analytic_error = relative_matrix_error(
        dense["inertia_com_kg_m2"], base_inertia
    )
    coarse_vertex_mean = vertex_mean(coarse_triangles)
    dense_vertex_mean = vertex_mean(dense_triangles)
    vertex_mean_shift_mm = vec_norm(vec_sub(
        dense_vertex_mean, coarse_vertex_mean
    ))

    require_less("nonuniform mesh volume relative difference",
                 volume_relative_difference)
    require_less("nonuniform mesh COM scaled difference",
                 com_scaled_difference)
    require_less("nonuniform mesh inertia relative difference",
                 inertia_relative_difference)
    require_less("nonuniform mesh analytic inertia relative error",
                 dense_analytic_error)
    require_greater("vertex-density mean displacement",
                    vertex_mean_shift_mm, 1.0)
    if len(unique_vertices(dense_triangles)) <= len(unique_vertices(coarse_triangles)):
        raise AssertionError("dense mesh must contain more unique vertices")
    if len(dense_triangles) <= len(coarse_triangles):
        raise AssertionError("dense mesh must contain more triangles")

    print("PASS E: identical box with non-uniform conforming triangulation")
    print("  coarse triangles / vertices = {} / {}".format(
        len(coarse_triangles), len(unique_vertices(coarse_triangles))
    ))
    print("  dense triangles / vertices  = {} / {}".format(
        len(dense_triangles), len(unique_vertices(dense_triangles))
    ))
    print("  coarse vertex mean mm       = {}".format(
        format_vector(coarse_vertex_mean)
    ))
    print("  dense vertex mean mm        = {}".format(
        format_vector(dense_vertex_mean)
    ))
    print("  vertex mean shift mm        = {:.17g}".format(vertex_mean_shift_mm))
    print("  volume relative difference  = {:.17g}".format(
        volume_relative_difference
    ))
    print("  COM scaled difference       = {:.17g}".format(
        com_scaled_difference
    ))
    print("  inertia relative difference = {:.17g}".format(
        inertia_relative_difference
    ))
    print("PASS VERTEX_DENSITY_INVARIANCE: volume, COM, and inertia unchanged")


def main():
    print("V15.16 V2 INERTIA MATH GOLDEN TEST")
    print("  project/CAD inputs read      = NO")
    print("  integration anchor           = mesh bbox center")
    print("  relative error threshold     = < {:.17g}".format(
        RELATIVE_TOLERANCE
    ))
    print("  geometry/mass units          = mm / kg")
    print("  final inertia unit           = kg*m^2")

    base_inertia = test_axis_aligned_box()
    rotation, rotated_inertia = test_rotated_box(base_inertia)
    test_parallel_axis(base_inertia)
    test_rotation_and_translation(rotation, rotated_inertia)
    test_nonuniform_triangulation(base_inertia)

    print("TOTAL PASS")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("FAIL: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
