"""Read-only containment checks for the research fingers' actual guide hole.

The bore is star-shaped about the TCP in its transverse plane. Intersect the
collision meshes with that plane, then check line of sight through the aperture
from the TCP to the cable. Expanding the cut edges by the cable radius accounts
for its thickness and closes jaw seams narrower than the cable. This observer
never applies a force or changes a pose; native contacts supply all guidance.
"""
import numpy as np
from transforms3d.quaternions import quat2mat


def ray_clearance(segments, directions, radius):
    """First nonnegative ray hit on 2-D edges expanded to radius-thick capsules."""
    a, b = segments[:, 0], segments[:, 1]
    directions = np.atleast_2d(directions)
    result = np.full(len(directions), np.inf)
    # Circular ends of each expanded edge.
    for centers in (a, b):
        along = directions @ centers.T
        discriminant = along**2 - np.sum(centers**2, axis=1) + radius**2
        t = along - np.sqrt(np.maximum(discriminant, 0.))
        valid = (discriminant >= 0.) & (t >= 0.)
        result = np.minimum(result, np.min(np.where(valid, t, np.inf), axis=1))
    # Straight sides, restricted to the finite edge span.
    edge = b-a
    length = np.linalg.norm(edge, axis=1)
    valid = length > 1.e-12
    a, length, edge = a[valid], length[valid], edge[valid]/length[valid, None]
    if len(edge):
        normal = np.column_stack((-edge[:, 1], edge[:, 0]))
        denominator = directions @ normal.T
        parallel = np.abs(denominator) < 1.e-12
        safe = np.where(parallel, 1., denominator)
        for sign in (-1., 1.):
            t = (np.sum(a*normal, axis=1) + sign*radius)/safe
            along = t*(directions @ edge.T) - np.sum(a*edge, axis=1)
            valid = ~parallel & (t >= 0.) & (along >= -1.e-10) & (along <= length+1.e-10)
            result = np.minimum(result, np.min(np.where(valid, t, np.inf), axis=1))
    return result


class MeshAperture:
    """Observe the bounded research-finger aperture using current jaw poses.

    ``sources`` contains (physical link, collision shape) pairs. A shape may
    belong to a kinematic proxy; its transform is evaluated on the real link.
    """
    def __init__(self, tcp, sources, radius, tolerance):
        import trimesh
        from dual_fr3_maniskill.engine.sapien_compat import sapien

        self.tcp, self.radius, self.tolerance = tcp, radius, tolerance
        self.sources = []
        for actor, shape in sources:
            geometry = shape.geometry
            if isinstance(geometry, sapien.BoxGeometry):
                mesh = trimesh.creation.box(extents=2*np.asarray(geometry.half_lengths))
            elif isinstance(geometry, (sapien.ConvexMeshGeometry, sapien.NonconvexMeshGeometry)):
                vertices = np.asarray(geometry.vertices)*np.asarray(geometry.scale)
                if hasattr(geometry, "rotation"):
                    vertices = vertices @ quat2mat(geometry.rotation).T
                mesh = trimesh.Trimesh(vertices=vertices,
                    faces=np.asarray(geometry.indices).reshape(-1, 3), process=False)
            else:
                raise ValueError("Rope-Actor guide requires mesh or box aperture geometry")
            self.sources.append((actor, shape.get_local_pose(), mesh))
        if not self.sources:
            raise ValueError("Rope-Actor guide has no physical aperture collision geometry")
        self._poses = None
        self.segments = None
        self.wall_distance = None
        self.inside = False

    def _section(self):
        import trimesh

        poses = [self.tcp.pose.inv()*actor.pose*local for actor, local, _ in self.sources]
        values = np.array([np.r_[p.p, p.q] for p in poses])
        if self._poses is not None and np.allclose(values, self._poses, atol=1.e-7, rtol=0.):
            return
        cuts = []
        for pose, (_, _, mesh) in zip(poses, self.sources):
            inverse = pose.inv()
            section = trimesh.intersections.mesh_plane(mesh,
                plane_normal=quat2mat(inverse.q)[:, 0], plane_origin=inverse.p)
            if len(section):
                cuts.append((section @ quat2mat(pose.q).T + pose.p)[:, :, 1:])
        if not cuts:
            raise RuntimeError("Right finger collision geometry does not enclose the TCP guide plane")
        self.segments = np.concatenate(cuts)
        self._poses = values

    def check(self, world_point):
        self._section()
        point = (self.tcp.pose.inv().to_transformation_matrix() @ np.r_[world_point, 1.])[1:3]
        a, b = self.segments[:, 0], self.segments[:, 1]
        edge = b-a
        length2 = np.sum(edge**2, axis=1)

        def distance(p):
            t = np.sum((p-a)*edge, axis=1)/np.maximum(length2, 1.e-24)
            return float(np.linalg.norm(p-a-np.clip(t, 0., 1.)[:, None]*edge, axis=1).min())

        radius = max(0., self.radius-self.tolerance)
        if distance(np.zeros(2)) < radius:
            raise RuntimeError("Right finger aperture does not provide cable clearance at the TCP")
        radial = np.linalg.norm(point)
        direction = point/radial if radial > 1.e-12 else np.array([1., 0.])
        boundary = ray_clearance(self.segments, direction, radius)[0]
        self.inside = bool(np.isfinite(boundary) and radial <= boundary+1.e-9)
        self.wall_distance = distance(point)
        if not self.inside:
            raise RuntimeError("Rope-Actor cable left the right finger aperture or intersects its CAD boundary")
