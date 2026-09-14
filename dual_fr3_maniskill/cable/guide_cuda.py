"""Device-only sliding guide; compatible with ManiSkill2's bundled Warp 0.3.1.

Geometry uses the same crossing and arc-length taper as threading.guide_projection.
Warp 0.3.1 arithmetic is float32. Chunked scans bound accumulation error without
requiring a newer Warp, a second CUDA context, or a host particle readback.
"""
import warp as wp


CHUNK = 32


@wp.func
def finite(x: float):
    return x == x and wp.abs(x) <= 3.402823e38


@wp.kernel
def clear_reaction(summary: wp.array(dtype=float)):
    # Error is sticky until reset, including across successive solve calls.
    summary[wp.tid()] = 0.0


@wp.kernel
def guide_centers(q: wp.array(dtype=wp.vec3), centers: wp.array(dtype=wp.vec3)):
    section = wp.tid()
    total = wp.vec3(0.0)
    for j in range(7):
        total = total + q[section * 7 + j]
    centers[section] = total / 7.0


@wp.kernel
def scan_sections(centers: wp.array(dtype=wp.vec3), pose: wp.array(dtype=wp.vec3),
                  summary: wp.array(dtype=float), arc: wp.array(dtype=float),
                  chunks: wp.array(dtype=wp.vec4), sections: int):
    block = wp.tid()
    total = float(0.0)
    largest = float(0.0)
    selected = float(-1.0)
    best = float(33.0)
    error = float(0.0)
    for i in range(block * 32, wp.min((block + 1) * 32, sections)):
        center = centers[i]
        if not finite(center[0]) or not finite(center[1]) or not finite(center[2]):
            error = 1.0
        if i > 0:
            spacing = wp.length(center - centers[i - 1])
            total = total + spacing
            largest = wp.max(largest, spacing)
            a = wp.dot(centers[i - 1] - pose[0], pose[2])
            b = wp.dot(center - pose[0], pose[2])
            denominator = b - a
            if a * b <= 0.0 and wp.abs(denominator) > 1.e-10:
                candidate = float(i - 1) - a / denominator
                distance = wp.abs(candidate - summary[6])
                if distance <= 32.0 and distance < best:
                    selected = candidate
                    best = distance
        arc[i] = total
    chunks[block] = wp.vec4(total, largest, selected, error)


@wp.kernel
def select_crossing(chunks: wp.array(dtype=wp.vec4), arc: wp.array(dtype=float),
                    offsets: wp.array(dtype=float), summary: wp.array(dtype=float),
                    control: wp.array(dtype=float), count: int, sections: int, pinned: int):
    if summary[7] != 0.0:
        return
    total = float(0.0)
    largest = float(0.0)
    best = float(33.0)
    selected = float(-1.0)
    error = float(0.0)
    for block in range(count):
        chunk = chunks[block]
        offsets[block] = total
        total = total + chunk[0]
        largest = wp.max(largest, chunk[1])
        error = wp.max(error, chunk[3])
        distance = wp.abs(chunk[2] - summary[6])
        if chunk[2] >= 0.0 and distance < best:
            best = distance
            selected = chunk[2]
    if error != 0.0 or not finite(summary[6]) or not finite(total):
        summary[7] = 1.0
    elif selected < 0.0:
        summary[7] = 2.0
    elif selected <= float(pinned + 2) or selected >= float(sections - 3):
        summary[7] = 3.0
    else:
        left = int(selected)
        a = arc[left] + offsets[left / 32]
        b = arc[left + 1] + offsets[(left + 1) / 32]
        control[0] = a + (selected - float(left)) * (b - a)
        control[1] = wp.max(0.02, 4.0 * largest)
        summary[6] = selected


@wp.kernel
def project_guide(q: wp.array(dtype=wp.vec3), qd: wp.array(dtype=wp.vec3),
                  mass: wp.array(dtype=float), centers: wp.array(dtype=wp.vec3),
                  pose: wp.array(dtype=wp.vec3), arc: wp.array(dtype=float),
                  offsets: wp.array(dtype=float), control: wp.array(dtype=float),
                  summary: wp.array(dtype=float), reactions: wp.array(dtype=wp.spatial_vector),
                  half_length: float, pinned: int, dt: float):
    section = wp.tid()
    reactions[section] = wp.spatial_vector(wp.vec3(0.0), wp.vec3(0.0))
    if summary[7] != 0.0:
        return
    distance = wp.abs(arc[section] + offsets[section / 32] - control[0])
    u = wp.clamp((distance - half_length) / control[1], 0.0, 1.0)
    weight = 0.5 * (1.0 + wp.cos(3.141592653589793 * u))
    if section < pinned:
        weight = 0.0
    relative = centers[section] - pose[0]
    shift = (relative - wp.dot(relative, pose[2]) * pose[2]) * (-weight)
    mean_velocity = wp.vec3(0.0)
    for j in range(7):
        mean_velocity = mean_velocity + (qd[section * 7 + j] + shift / dt)
    hole_velocity = pose[4] + wp.cross(pose[5], centers[section] - pose[3])
    relative_velocity = mean_velocity / 7.0 - hole_velocity
    correction = weight * (relative_velocity - wp.dot(relative_velocity, pose[1]) * pose[1])
    force = wp.vec3(0.0)
    torque = wp.vec3(0.0)
    for j in range(7):
        particle = section * 7 + j
        before = qd[particle]
        after = (before + shift / dt) - correction
        position = q[particle] + shift
        impulse = (after - before) * (-mass[particle])
        force = force + impulse
        torque = torque + wp.cross(position - pose[3], impulse)
        q[particle] = position
        qd[particle] = after
    reactions[section] = wp.spatial_vector(torque, force)


@wp.kernel
def sum_reactions(reactions: wp.array(dtype=wp.spatial_vector),
                  summary: wp.array(dtype=float), sections: int):
    # One chunk per thread: bounded work and few atomic additions, with no
    # ordering dependence in the particle constraint itself.
    block = wp.tid()
    total = wp.spatial_vector(wp.vec3(0.0), wp.vec3(0.0))
    for i in range(block * 32, wp.min((block + 1) * 32, sections)):
        total = total + reactions[i]
    torque = wp.spatial_top(total)
    force = wp.spatial_bottom(total)
    for j in range(3):
        wp.atomic_add(summary, j, torque[j])
        wp.atomic_add(summary, j + 3, force[j])
