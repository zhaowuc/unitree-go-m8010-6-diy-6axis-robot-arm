#!/usr/bin/env python3
"""Fail-closed regression test for FreeCAD Mesh mass-property semantics.

The two meshes below describe exactly the same closed, offset, scalene cuboid.
The second mesh only adds non-uniform boundary vertices concentrated near the
+X side.  No project CAD, ledger, or geometry file is read by this test.
"""

from __future__ import print_function

import math
import sys

try:
    import FreeCAD  # noqa: F401 - initializes FreeCAD's bundled module paths
    import Mesh
except Exception as exc:  # pragma: no cover - exercised by the launcher
    print("FAIL: FreeCAD Mesh module is required: {}".format(exc), file=sys.stderr)
    raise SystemExit(1)


BOUNDS = (-3.25, 8.75, 1.125, 5.875, -4.5, 2.25)
ANCHOR = (-17.125, 23.5, -11.75)


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(scale, vector):
    return (scale * vector[0], scale * vector[1], scale * vector[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(vector):
    return math.sqrt(dot(vector, vector))


def as_xyz(value):
    value = getattr(value, "Vector", value)
    return (float(value.x), float(value.y), float(value.z))


def append_quad(triangles, a, b, c, d, reverse=False):
    """Append two outward triangles; reverse flips the supplied normal."""
    if reverse:
        triangles.extend(((a, c, b), (a, d, c)))
    else:
        triangles.extend(((a, b, c), (a, c, d)))


def build_cuboid(x_knots):
    x0, x1, y0, y1, z0, z1 = BOUNDS
    if len(x_knots) < 2 or x_knots[0] != x0 or x_knots[-1] != x1:
        raise AssertionError("x knots must span the exact cuboid bounds")
    if any(not (left < right) for left, right in zip(x_knots, x_knots[1:])):
        raise AssertionError("x knots must be strictly increasing")

    triangles = []
    for xa, xb in zip(x_knots, x_knots[1:]):
        # y = y0, outward -Y; y = y1, outward +Y.
        append_quad(
            triangles,
            (xa, y0, z0), (xb, y0, z0),
            (xb, y0, z1), (xa, y0, z1),
        )
        append_quad(
            triangles,
            (xa, y1, z0), (xb, y1, z0),
            (xb, y1, z1), (xa, y1, z1),
            reverse=True,
        )

        # z = z0, outward -Z; z = z1, outward +Z.
        append_quad(
            triangles,
            (xa, y0, z0), (xb, y0, z0),
            (xb, y1, z0), (xa, y1, z0),
            reverse=True,
        )
        append_quad(
            triangles,
            (xa, y0, z1), (xb, y0, z1),
            (xb, y1, z1), (xa, y1, z1),
        )

    # End caps.  Their supplied order has a +X normal.
    append_quad(
        triangles,
        (x0, y0, z0), (x0, y1, z0),
        (x0, y1, z1), (x0, y0, z1),
        reverse=True,
    )
    append_quad(
        triangles,
        (x1, y0, z0), (x1, y1, z0),
        (x1, y1, z1), (x1, y0, z1),
    )
    return triangles


def validate_closed_oriented_surface(triangles):
    """Require a non-degenerate, consistently oriented closed 2-manifold."""
    incidences = {}
    for triangle_index, triangle in enumerate(triangles):
        if len(set(triangle)) != 3:
            raise AssertionError("triangle {} repeats a vertex".format(triangle_index))
        area2 = norm(cross(sub(triangle[1], triangle[0]),
                           sub(triangle[2], triangle[0])))
        if not math.isfinite(area2) or area2 <= 0.0:
            raise AssertionError("triangle {} is degenerate".format(triangle_index))
        for start, end in zip(triangle, (triangle[1], triangle[2], triangle[0])):
            key = tuple(sorted((start, end)))
            direction = 1 if (start, end) == key else -1
            incidences.setdefault(key, []).append(direction)

    bad = [
        (edge, directions)
        for edge, directions in incidences.items()
        if len(directions) != 2 or sum(directions) != 0
    ]
    if bad:
        raise AssertionError(
            "surface is not a closed consistently oriented 2-manifold: {} bad edges"
            .format(len(bad))
        )


def validate_exact_cuboid_boundary(triangles):
    x0, x1, y0, y1, z0, z1 = BOUNDS
    for triangle_index, triangle in enumerate(triangles):
        xs = {point[0] for point in triangle}
        ys = {point[1] for point in triangle}
        zs = {point[2] for point in triangle}
        on_boundary_plane = (
            xs == {x0} or xs == {x1}
            or ys == {y0} or ys == {y1}
            or zs == {z0} or zs == {z1}
        )
        if not on_boundary_plane:
            raise AssertionError(
                "triangle {} is not on the analytic cuboid boundary".format(
                    triangle_index
                )
            )
        for x, y, z in triangle:
            if not (x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1):
                raise AssertionError("triangle vertex lies outside the cuboid")


def anchored_signed_tetrahedron_properties(triangles, anchor):
    """Independent volume and first moment from oriented boundary triangles."""
    signed_volumes = []
    moment_x = []
    moment_y = []
    moment_z = []

    for p0, p1, p2 in triangles:
        a = sub(p0, anchor)
        b = sub(p1, anchor)
        c = sub(p2, anchor)
        volume = dot(a, cross(b, c)) / 6.0
        tetra_centroid_offset = mul(0.25, add(add(a, b), c))
        signed_volumes.append(volume)
        moment_x.append(volume * tetra_centroid_offset[0])
        moment_y.append(volume * tetra_centroid_offset[1])
        moment_z.append(volume * tetra_centroid_offset[2])

    total_volume = math.fsum(signed_volumes)
    if not math.isfinite(total_volume) or total_volume <= 0.0:
        raise AssertionError("signed tetrahedron volume must be finite and positive")

    centroid = add(
        anchor,
        (
            math.fsum(moment_x) / total_volume,
            math.fsum(moment_y) / total_volume,
            math.fsum(moment_z) / total_volume,
        ),
    )
    if not all(math.isfinite(value) for value in centroid):
        raise AssertionError("signed tetrahedron centroid must be finite")
    return total_volume, centroid


def vertex_mean(mesh):
    points = [as_xyz(point) for point in mesh.Points]
    if not points:
        raise AssertionError("mesh has no points")
    count = float(len(points))
    return tuple(math.fsum(point[axis] for point in points) / count
                 for axis in range(3))


def require_less(label, actual, limit):
    if not math.isfinite(actual) or not actual < limit:
        raise AssertionError(
            "{}: expected {:.17g} < {:.17g}".format(label, actual, limit)
        )


def require_greater(label, actual, limit):
    if not math.isfinite(actual) or not actual > limit:
        raise AssertionError(
            "{}: expected {:.17g} > {:.17g}".format(label, actual, limit)
        )


def analyse_case(name, x_knots, analytic_centroid, analytic_volume,
                 characteristic_length):
    triangles = build_cuboid(x_knots)
    validate_closed_oriented_surface(triangles)
    validate_exact_cuboid_boundary(triangles)

    poly_volume, poly_centroid = anchored_signed_tetrahedron_properties(
        triangles, ANCHOR
    )
    mesh = Mesh.Mesh(triangles)
    mesh_cog = as_xyz(mesh.CenterOfGravity)
    points_mean = vertex_mean(mesh)
    mesh_volume = float(mesh.Volume)

    centroid_error = norm(sub(poly_centroid, analytic_centroid))
    centroid_limit = 1.0e-10 * characteristic_length
    require_less(name + " polyhedral centroid error", centroid_error,
                 centroid_limit)
    require_less(
        name + " polyhedral volume relative error",
        abs(poly_volume - analytic_volume) / analytic_volume,
        1.0e-12,
    )
    require_less(
        name + " Mesh.CenterOfGravity vs vertex mean",
        norm(sub(mesh_cog, points_mean)),
        1.0e-6 * characteristic_length,
    )

    expected_facets = len(triangles)
    if int(mesh.CountFacets) != expected_facets:
        raise AssertionError(
            "{}: FreeCAD facet count {} != {}".format(
                name, mesh.CountFacets, expected_facets
            )
        )

    print("CASE {}".format(name))
    print("  facets / points             = {} / {}".format(
        mesh.CountFacets, mesh.CountPoints
    ))
    print("  FreeCAD Mesh.CenterOfGravity= {}".format(mesh_cog))
    print("  vertex_mean                 = {}".format(points_mean))
    print("  analytic_volume_centroid    = {}".format(analytic_centroid))
    print("  polyhedral_volume_centroid  = {}".format(poly_centroid))
    print("  FreeCAD Mesh.Volume         = {:.17g}".format(mesh_volume))
    print("  anchored tetra volume       = {:.17g}".format(poly_volume))
    print("  analytic volume             = {:.17g}".format(analytic_volume))
    print("  polyhedral centroid error   = {:.17g}".format(centroid_error))

    return {
        "mesh_cog": mesh_cog,
        "vertex_mean": points_mean,
        "poly_centroid": poly_centroid,
        "poly_volume": poly_volume,
        "point_count": int(mesh.CountPoints),
    }


def main():
    x0, x1, y0, y1, z0, z1 = BOUNDS
    lengths = (x1 - x0, y1 - y0, z1 - z0)
    characteristic_length = norm(lengths)
    analytic_centroid = (
        0.5 * (x0 + x1),
        0.5 * (y0 + y1),
        0.5 * (z0 + z1),
    )
    analytic_volume = lengths[0] * lengths[1] * lengths[2]

    coarse_knots = (x0, x1)
    biased_fractions = (0.0, 0.68, 0.84, 0.92, 0.965, 0.985, 0.995, 1.0)
    biased_knots = tuple(x0 + fraction * lengths[0]
                         for fraction in biased_fractions)

    print("anchor                        = {}".format(ANCHOR))
    print("characteristic length         = {:.17g}".format(
        characteristic_length
    ))
    print("centroid tolerance            = {:.17g}".format(
        1.0e-10 * characteristic_length
    ))

    coarse = analyse_case(
        "coarse", coarse_knots, analytic_centroid, analytic_volume,
        characteristic_length,
    )
    biased = analyse_case(
        "biased_+X", biased_knots, analytic_centroid, analytic_volume,
        characteristic_length,
    )

    # Both independent integrations recover one analytic solid, while only
    # vertex density changes.  FreeCAD's COG must therefore not be invariant.
    poly_between = norm(sub(coarse["poly_centroid"], biased["poly_centroid"]))
    mesh_cog_shift = norm(sub(coarse["mesh_cog"], biased["mesh_cog"]))
    vertex_mean_shift = norm(sub(coarse["vertex_mean"],
                                 biased["vertex_mean"]))
    require_less(
        "polyhedral centroids of identical geometry",
        poly_between,
        1.0e-10 * characteristic_length,
    )
    require_greater(
        "FreeCAD Mesh.CenterOfGravity vertex-density shift",
        mesh_cog_shift,
        1.0e-3 * characteristic_length,
    )
    require_greater(
        "vertex mean density shift",
        vertex_mean_shift,
        1.0e-3 * characteristic_length,
    )
    if biased["point_count"] <= coarse["point_count"]:
        raise AssertionError("biased mesh must contain more unique points")

    print("COMPARISON")
    print("  Mesh.CenterOfGravity shift  = {:.17g}".format(mesh_cog_shift))
    print("  vertex_mean shift           = {:.17g}".format(vertex_mean_shift))
    print("  polyhedral centroid shift   = {:.17g}".format(poly_between))
    print("PASS: Mesh.CenterOfGravity changes with boundary vertex density")
    print("PASS: anchored polyhedral centroid matches analytic centroid")
    print("TOTAL PASS")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("FAIL: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
