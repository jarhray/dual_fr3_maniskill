"""Finite-radius cable contact, independent of the MPM grid resolution.

Section spheres conservatively cover the intervening cable capsules. Closest
surface queries include mesh interiors; conservative advancement catches thin
walls even when both ends of a time step lie outside. This solver must run
after position constraints, and again after the SAPIEN bodies move.
"""
import numpy as np
import trimesh
import warp as wp
from .mesh_contacts import closed_oriented_shells


@wp.func
def closed_mesh_sign(mesh: wp.uint64, p: wp.vec3, limit: float):
    inside = int(0)
    for ray in range(3):
        direction = wp.normalize(wp.vec3(1.0, 0.37139, 0.69427))
        if ray == 1:
            direction = wp.normalize(wp.vec3(-0.43817, 1.0, 0.21783))
        elif ray == 2:
            direction = wp.normalize(wp.vec3(0.19373, -0.53719, 1.0))
        t = float(0.0)
        u = float(0.0)
        v = float(0.0)
        sign = float(0.0)
        normal = wp.vec3(0.0)
        face = int(0)
        if wp.mesh_query_ray(mesh, p, direction, limit, t, u, v, sign, normal, face):
            # For a closed, consistently oriented solid, the first hit from
            # inside is an exit (back face). Three oblique rays avoid relying
            # on a single grazing/edge hit in the STL triangulation.
            if wp.dot(normal, direction) > 0.0:
                inside = inside + 1
    result = float(1.0)
    if inside >= 2:
        result = -1.0
    return result


@wp.func
def cable_surface(p: wp.vec3, kind: int, mesh: wp.uint64, scale: wp.vec3,
            lower: wp.vec3, upper: wp.vec3):
    n = wp.vec3(0.0, 0.0, 1.0)
    d = float(1.e6)
    ray_queries = int(0)
    if kind == 1:
        closest = wp.vec3(wp.clamp(p[0], -scale[0], scale[0]),
                          wp.clamp(p[1], -scale[1], scale[1]),
                          wp.clamp(p[2], -scale[2], scale[2]))
        delta = p - closest
        if wp.length(delta) > 1.e-10:
            d = wp.length(delta)
            n = delta / d
        else:
            axis = int(0)
            d = wp.abs(p[0]) - scale[0]
            for j in range(1, 3):
                if wp.abs(p[j]) - scale[j] > d:
                    axis = j
                    d = wp.abs(p[j]) - scale[j]
            sign = float(1.0)
            if p[axis] < 0.0:
                sign = -1.0
            n = wp.vec3(sign, 0.0, 0.0)
            if axis == 1:
                n = wp.vec3(0.0, sign, 0.0)
            elif axis == 2:
                n = wp.vec3(0.0, 0.0, sign)
    elif kind == 2:
        delta = p - wp.vec3(wp.clamp(p[0], -scale[1], scale[1]), 0.0, 0.0)
        d = wp.length(delta) - scale[0]
        if wp.length(delta) > 1.e-10:
            n = wp.normalize(delta)
    elif kind == 3:
        face = int(0)
        u = float(0.0)
        v = float(0.0)
        sign = float(0.0)
        # A narrow max_dist misses points deep INSIDE a solid. Broad phase
        # runs separately; this bound always includes the nearest triangle.
        limit = wp.length(p - (lower + upper)*0.5) + wp.length(upper-lower) + 0.01
        if wp.mesh_query_point(mesh, p / scale[0], limit / scale[0], sign, face, u, v):
            closest = wp.mesh_eval_position(mesh, face, u, v) * scale[0]
            delta = p - closest
            # Warp 0.3.1's nearest-face sign marks a point inside when ANY
            # equally close triangle faces away. Around a thin wall rim this
            # can label open cavity space as solid and cause centimetre jumps.
            sign = closed_mesh_sign(mesh, p / scale[0], limit / scale[0])
            ray_queries = 3
            d = wp.length(delta) * sign
            if wp.length(delta) > 1.e-9:
                n = wp.normalize(delta) * sign
            else:
                a = wp.mesh_eval_position(mesh, face, 1.0, 0.0)
                b = wp.mesh_eval_position(mesh, face, 0.0, 1.0)
                c = wp.mesh_eval_position(mesh, face, 0.0, 0.0)
                n = wp.normalize(wp.cross(b-a, c-a))
    return wp.vec4(n[0], n[1], n[2], d), ray_queries


@wp.func
def overlaps(a: wp.vec3, b: wp.vec3, radius: float, lower: wp.vec3, upper: wp.vec3):
    result = int(1)
    for j in range(3):
        if wp.min(a[j], b[j]) > upper[j] + radius or wp.max(a[j], b[j]) < lower[j] - radius:
            result = 0
    return result


@wp.func
def cached_surface(p: wp.vec3, kind: int, mesh: wp.uint64, scale: wp.vec3,
    lower: wp.vec3, upper: wp.vec3, section: int, shape: int,
    cache_point: wp.array2d(dtype=wp.vec3), cache_value: wp.array2d(dtype=wp.vec4),
    cache_valid: wp.array2d(dtype=int)):
    previous = cache_point[section, shape]
    if kind == 3 and cache_valid[section, shape] != 0:
        if p[0] == previous[0] and p[1] == previous[1] and p[2] == previous[2]:
            return cache_value[section, shape], int(0), int(0), int(1)
    value, rays = cable_surface(p, kind, mesh, scale, lower, upper)
    point_queries = int(0)
    if kind == 3:
        point_queries = 1
        cache_point[section, shape] = p
        cache_value[section, shape] = value
        cache_valid[section, shape] = 1
    return value, rays, point_queries, int(0)


@wp.func
def sweep(start: wp.vec3, target: wp.vec3, radius: float, margin: float,
    kind: int, mesh: wp.uint64, scale: wp.vec3, lower: wp.vec3, upper: wp.vec3,
    section: int, shape: int, cache_point: wp.array2d(dtype=wp.vec3),
    cache_value: wp.array2d(dtype=wp.vec4), cache_valid: wp.array2d(dtype=int)):
    x = start
    goal = target
    normal = wp.vec3(0.0)
    # Distance is 1-Lipschitz. Never advance farther than the clearance.
    # Exhausting the budget retains the last safe position, not the goal.
    iteration = int(0)
    active = int(1)
    ray_queries = int(0)
    point_queries = int(0)
    cache_hits = int(0)
    while iteration < 48 and active != 0:
        iteration = iteration + 1
        nd, rays, points, cached = cached_surface(x, kind, mesh, scale, lower, upper,
            section, shape, cache_point, cache_value, cache_valid)
        ray_queries = ray_queries + rays
        point_queries = point_queries + points
        cache_hits = cache_hits + cached
        n = wp.vec3(nd[0], nd[1], nd[2])
        gap = nd[3] - radius
        if gap < margin * 0.25:
            x = x + n * (margin - gap)
            goal = goal - n * wp.min(wp.dot(goal-x, n), 0.0)
            normal = n
            gap = margin
        delta = goal - x
        length = wp.length(delta)
        if length <= gap:
            x = goal
            active = 0
        else:
            x = x + delta * (0.9 * gap / wp.max(length, 1.e-12))
    return x, normal, iteration, ray_queries, active, point_queries, cache_hits


@wp.kernel
def section_centers(q: wp.array(dtype=wp.vec3), centers: wp.array(dtype=wp.vec3)):
    i = wp.tid()
    c = wp.vec3(0.0)
    for j in range(7):
        c = c + q[i*7+j] / 7.0
    centers[i] = c


@wp.kernel
def project_sections(q: wp.array(dtype=wp.vec3), v: wp.array(dtype=wp.vec3),
    affine: wp.array(dtype=wp.mat33),
    mass: wp.array(dtype=float), centers: wp.array(dtype=wp.vec3),
    accepted: wp.array(dtype=wp.vec3), count: int, pin_sections: int,
    radius: float, margin: float, friction: wp.array(dtype=float),
    old_body: wp.array(dtype=wp.transform), body: wp.array(dtype=wp.transform),
    body_v: wp.array(dtype=wp.spatial_vector), body_com: wp.array(dtype=wp.vec3),
    shape_pose: wp.array(dtype=wp.transform), shape_body: wp.array(dtype=int),
    kind: wp.array(dtype=int), mesh: wp.array(dtype=wp.uint64), scale: wp.array(dtype=wp.vec3),
    lower: wp.array(dtype=wp.vec3), upper: wp.array(dtype=wp.vec3), shape_count: int,
    impulse: wp.array(dtype=wp.spatial_vector), hits: wp.array(dtype=int), passes: int,
    profile: int, counters: wp.array3d(dtype=int), cache_point: wp.array2d(dtype=wp.vec3),
    cache_value: wp.array2d(dtype=wp.vec4), cache_valid: wp.array2d(dtype=int)):
    i = wp.tid()
    c = centers[i]
    origin = c
    previous = accepted[i]
    # Each sphere covers both adjacent half-capsules, including their outer
    # radius. Using the actual gap also covers moderately stretched sections.
    gap = float(0.0)
    if i > 0:
        gap = wp.length(c - centers[i-1])
    if i+1 < count:
        gap = wp.max(gap, wp.length(c - centers[i+1]))
    guard = wp.sqrt(radius*radius + 0.25*gap*gap)
    if i >= pin_sections:
        for repeat in range(passes):
            for s in range(shape_count):
                if profile != 0:
                    counters[i, s, 0] = counters[i, s, 0] + 1
                b = shape_body[s]
                transform = wp.transform_multiply(body[b], shape_pose[s])
                inverse = wp.transform_inverse(transform)
                old_transform = transform
                if repeat == 0:
                    old_transform = wp.transform_multiply(old_body[b], shape_pose[s])
                a = wp.transform_point(wp.transform_inverse(old_transform), previous)
                end = wp.transform_point(inverse, c)
                if overlaps(a, end, guard+margin, lower[s], upper[s]) != 0:
                    corrected, normal, iterations, rays, exhausted, point_queries, cache_hits = sweep(
                        a, end, guard, margin, kind[s], mesh[s], scale[s], lower[s], upper[s],
                        i, s, cache_point, cache_value, cache_valid)
                    if profile != 0:
                        # Each section owns its row: no profiling atomics or
                        # device-to-host copies inside the solve loop.
                        counters[i, s, 1] = counters[i, s, 1] + 1
                        counters[i, s, 2] = counters[i, s, 2] + iterations
                        counters[i, s, 3] = counters[i, s, 3] + point_queries
                        counters[i, s, 4] = counters[i, s, 4] + rays
                        counters[i, s, 5] = counters[i, s, 5] + exhausted
                        counters[i, s, 6] = wp.max(counters[i, s, 6], iterations)
                        counters[i, s, 8] = counters[i, s, 8] + cache_hits
                    if wp.length(corrected-end) > 0.0:
                        c = wp.transform_point(transform, corrected)
                    if wp.length(normal) > 0.0:
                        if profile != 0:
                            counters[i, s, 7] = counters[i, s, 7] + 1
                        wp.atomic_add(hits, s, 1)
                        normal = wp.transform_vector(transform, normal)
                        com = wp.transform_point(body[b], body_com[b])
                        w = wp.spatial_top(body_v[b])
                        spin = wp.mat33(0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0)
                        identity = wp.mat33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
                        projection = identity - wp.outer(normal, normal)
                        for j in range(7):
                            p = i*7+j
                            point = q[p] + c-origin
                            bv = wp.spatial_bottom(body_v[b]) + wp.cross(w, point-com)
                            relative = v[p] - bv
                            vn = wp.dot(relative, normal)
                            tangent = relative - normal*vn
                            speed = wp.length(tangent)
                            tangent = tangent * wp.max(1.0 + friction[s]*wp.min(vn, 0.0)/wp.max(speed, 1.e-12), 0.0)
                            new_v = bv + tangent
                            reaction = (v[p]-new_v) * mass[p]
                            wp.atomic_add(impulse, b, wp.spatial_vector(wp.cross(point-com, reaction), reaction))
                            v[p] = new_v
                            # APIC stores a local velocity field as well as v.
                            # Project that field too; otherwise the next P2G
                            # immediately restores momentum into the wall.
                            affine[p] = projection * (affine[p]-spin) + spin
            previous = c
        for j in range(7):
            q[i*7+j] = q[i*7+j] + c-origin
    accepted[i] = c


@wp.kernel
def measure_clearance(centers: wp.array(dtype=wp.vec3), count: int, pin_sections: int,
    radius: float, body: wp.array(dtype=wp.transform), shape_pose: wp.array(dtype=wp.transform),
    shape_body: wp.array(dtype=int), kind: wp.array(dtype=int), mesh: wp.array(dtype=wp.uint64),
    scale: wp.array(dtype=wp.vec3), lower: wp.array(dtype=wp.vec3), upper: wp.array(dtype=wp.vec3),
    pinned_exclusions: wp.array(dtype=int),
    depths: wp.array2d(dtype=float)):
    sample, s = wp.tid()
    # Audit the physical tube at four locations per edge, independently of
    # the inflated section spheres used by the constraint solver.
    edge = sample / 4
    point = centers[edge]
    if edge+1 < count:
        point = point + (centers[edge+1]-point) * (float(sample % 4) / 4.0)
    local = wp.transform_point(wp.transform_inverse(wp.transform_multiply(
        body[shape_body[s]], shape_pose[s])), point)
    depth = float(0.0)
    excluded = int(0)
    if edge < pin_sections and pinned_exclusions[s] != 0:
        excluded = 1
    if excluded == 0 and overlaps(local, local, radius, lower[s], upper[s]) != 0:
        nd, rays = cable_surface(local, kind[s], mesh[s], scale[s], lower[s], upper[s])
        depth = wp.max(radius - nd[3], 0.0)
    depths[sample, s] = depth


class CableContacts:
    def __init__(self, model, sections, pin_sections, config, device, pinned_exclusions=None,
                 frictionless_shapes=()):
        self.model, self.sections, self.pin_sections = model, sections, pin_sections
        self.radius = config['diameter'] / 2
        self.margin = config['contact_margin']
        friction = np.full(model.shape_count, config['friction'], dtype=np.float32)
        friction[list(frictionless_shapes)] = 0.
        self.friction = wp.array(friction, dtype=float, device=device)
        self.passes = config['contact_iterations']
        self.device = device
        lower, upper = [], []
        for kind, scale, source in zip(model.shape_geo_type.numpy(), model.shape_geo_scale.numpy(), model.shape_geo_src):
            if kind == 1:
                lo, hi = -scale, scale
            elif kind == 2:
                hi = np.array([scale[0]+scale[1], scale[0], scale[0]])
                lo = -hi
            elif kind == 3:
                if not np.allclose(scale, scale[0]):
                    raise ValueError('Bake nonuniform mesh scale into vertices before cable contact')
                points = np.asarray(source.vertices) * scale[0]
                mesh = trimesh.Trimesh(vertices=points, faces=np.asarray(source.indices).reshape(-1, 3), process=True)
                if not closed_oriented_shells(mesh):
                    raise ValueError('Cable collision meshes must be closed with outward consistent winding')
                lo, hi = points.min(axis=0), points.max(axis=0)
            else:
                raise ValueError(f'Unsupported cable collider type {kind}')
            lower.append(lo)
            upper.append(hi)
        self.lower = wp.array(np.asarray(lower), dtype=wp.vec3, device=device)
        self.upper = wp.array(np.asarray(upper), dtype=wp.vec3, device=device)
        self.centers = wp.zeros(sections, dtype=wp.vec3, device=device)
        self.accepted = wp.zeros_like(self.centers)
        self.impulse = wp.zeros(model.body_count, dtype=wp.spatial_vector, device=device)
        self.hits = wp.zeros(model.shape_count, dtype=int, device=device)
        mask = np.zeros(model.shape_count, dtype=np.int32) if pinned_exclusions is None else pinned_exclusions
        self.pinned_exclusions = wp.array(np.asarray(mask), dtype=int, device=device)
        self.depths = wp.zeros((sections*4, model.shape_count), dtype=float, device=device)
        self.max_depth = 0.0
        self.profile_enabled = False
        self.profile_counts = wp.zeros((1, 1, 1), dtype=int, device=device)
        self.profile_calls = 0
        self.cache_point = wp.zeros((sections, model.shape_count), dtype=wp.vec3, device=device)
        self.cache_value = wp.zeros((sections, model.shape_count), dtype=wp.vec4, device=device)
        self.cache_valid = wp.zeros((sections, model.shape_count), dtype=int, device=device)

    def invalidate_query_cache(self):
        """Invalidate exact local-space mesh results after body/geometry updates."""
        self.cache_valid.zero_()

    def enable_profiling(self):
        """Optional counters for a separate diagnostic run, disabled normally."""
        self.profile_counts = wp.zeros((self.sections, self.model.shape_count, 9),
                                       dtype=int, device=self.device)
        self.profile_enabled = True
        self.profile_calls = 0

    def profiling_report(self):
        if not self.profile_enabled:
            raise RuntimeError('Contact profiling was not enabled')
        values = self.profile_counts.numpy()
        totals = values.sum(axis=0, dtype=np.int64)
        totals[:, 6] = values[:, :, 6].max(axis=0)
        names = ('pair_tests', 'candidate_sweeps', 'surface_queries', 'mesh_point_queries',
                 'mesh_ray_queries', 'exhausted_sweeps', 'max_sweep_iterations', 'velocity_projections',
                 'exact_query_cache_hits')
        bodies, kinds = self.model.shape_body.numpy(), self.model.shape_geo_type.numpy()
        return dict(calls=self.profile_calls, sections=self.sections,
            free_sections=self.sections-self.pin_sections, shape_count=self.model.shape_count,
            per_shape=[dict(shape=i, body=int(bodies[i]), kind=int(kinds[i]),
                **{name: int(totals[i, j]) for j, name in enumerate(names)})
                for i in range(self.model.shape_count)])

    def capture(self, state):
        wp.launch(section_centers, dim=self.sections, inputs=[state.particle_q, self.accepted], device=self.device)

    def solve(self, state, old_body=None, *, reuse_query_cache=False):
        m = self.model
        # Default callers may change geometry between solves. The MPM loop
        # explicitly reuses results only between its body-state uploads.
        if not reuse_query_cache:
            self.invalidate_query_cache()
        if self.profile_enabled:
            self.profile_calls += 1
        wp.launch(section_centers, dim=self.sections, inputs=[state.struct.particle_q, self.centers], device=self.device)
        wp.launch(project_sections, dim=self.sections, inputs=[state.struct.particle_q,
            state.struct.particle_qd, state.struct.particle_C, m.struct.particle_mass, self.centers, self.accepted,
            self.sections, self.pin_sections, self.radius, self.margin, self.friction,
            state.body_q if old_body is None else old_body, state.body_q, state.body_qd, m.body_com,
            m.shape_transform, m.shape_body, m.shape_geo_type, m.shape_geo_id, m.shape_geo_scale,
            self.lower, self.upper, m.shape_count, self.impulse, self.hits, self.passes,
            int(self.profile_enabled), self.profile_counts,
            self.cache_point, self.cache_value, self.cache_valid], device=self.device)

    def audit(self, state, positions=None):
        m = self.model
        positions = state.struct.particle_q if positions is None else positions
        wp.launch(section_centers, dim=self.sections, inputs=[positions, self.centers], device=self.device)
        wp.launch(measure_clearance, dim=self.depths.shape, inputs=[self.centers, self.sections,
            self.pin_sections, self.radius, state.body_q, m.shape_transform, m.shape_body,
            m.shape_geo_type, m.shape_geo_id, m.shape_geo_scale, self.lower, self.upper,
            self.pinned_exclusions, self.depths], device=self.device)
        by_shape = self.depths.numpy().max(axis=0)
        self.max_depth = float(by_shape.max(initial=0))
        return by_shape
