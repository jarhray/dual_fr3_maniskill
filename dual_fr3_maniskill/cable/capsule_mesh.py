"""Inscribed convex capsules for PhysX's convex/triangle contact path.

Cook at unit radius and apply the physical radius as mesh scale. Cooking
millimetre vertices directly discards them under PhysX's plane tolerance.
The 194 vertices keep the maximum surface deficit below 0.028 * radius.
"""
from pathlib import Path

import numpy as np


def capsule_vertices(half_length):
    """Unit-radius capsule with +X axis; half_length is in radius units."""
    points = []
    for sign in (-1., 1.):
        for theta in np.linspace(0., np.pi/2, 5)[:-1]:
            for phi in np.linspace(0., 2*np.pi, 24, endpoint=False):
                points.append([sign*(half_length+np.sin(theta)),
                               np.cos(theta)*np.cos(phi), np.cos(theta)*np.sin(phi)])
        points.append([sign*(half_length+1.), 0., 0.])
    return np.asarray(points)


def write_capsule(path, length, radius):
    import trimesh

    mesh = trimesh.convex.convex_hull(capsule_vertices(length/(2*radius)))
    mesh.export(Path(path))


def validate_cooked_capsule(shape, length, radius):
    """Require containment between the ideal capsule and its 2.8% erosion.

Cooking may remove redundant vertices. Check its actual support planes rather
than a vertex count: the smaller capsule must satisfy every convex halfspace.
"""
    geometry = shape.geometry
    from scipy.spatial import ConvexHull

    vertices = np.asarray(geometry.vertices, dtype=float)*np.asarray(geometry.scale)
    expected = np.array([length/2+radius, radius, radius])
    # Use support planes of the cooked point set. SAPIEN's exported triangle
    # tessellation can also contain triangles inside a merged convex polygon.
    planes = ConvexHull(vertices).equations
    normals, distances = planes[:, :3], -planes[:, 3]
    deficit = length/2*abs(normals[:, 0])+radius-distances
    radial = vertices.copy()
    radial[:, 0] -= np.clip(radial[:, 0], -length/2, length/2)
    if (np.max(deficit) > .028*radius or
            np.max(np.linalg.norm(radial, axis=1)) > radius*(1.+1.e-4) or
            not np.allclose(vertices.max(0), expected, rtol=0., atol=radius*1.e-4) or
            not np.allclose(vertices.min(0), -expected, rtol=0., atol=radius*1.e-4)):
        raise RuntimeError("PhysX simplified the convex cable collision geometry; reset is required")
