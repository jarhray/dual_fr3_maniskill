"""Geometry and ideal sliding-eyelet constraint, independent of CUDA and ROS."""
import numpy as np


LEFT_TCP = "left_fr3_hand_tcp"
RIGHT_TCP = "right_fr3_hand_tcp"
TOUCH_LINKS = (LEFT_TCP, "left_fr3_hand", "left_fr3_leftfinger", "left_fr3_rightfinger")


def task_usb_mount(config, *, orientation_direction=None):
    """Place the USB grip below the guide TCP, with its tip along the path.

    A reverse-facing leader holds USB +Y along TCP -X. TCP +Z points away
    from the palm (down for the task's roll=pi), toward the physical grip.
    """
    if orientation_direction is None:
        orientation_direction = config["usb"].get("orientation_direction", "reverse")
    if orientation_direction not in ("forward", "reverse"):
        raise ValueError("USB orientation_direction must be forward or reverse")
    rotation = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    if orientation_direction == "reverse":
        rotation[:2] *= -1.
    grip_offset = np.asarray(config["usb"].get("tcp_grip_offset", [0., 0., .012]))
    return grip_offset - rotation @ np.asarray(config["usb"]["grip_center"]), rotation


def _bezier(a, b, c, d, count=1000):
    t = np.linspace(0., 1., count)[:, None]
    return (1-t)**3*a + 3*(1-t)**2*t*b + 3*(1-t)*t*t*c + t**3*d


def left_guide_geometry(config, rotation, translation):
    """Recover the preparation TCP from the fixed creation transform, not live TF."""
    mount_p, mount_r = task_usb_mount(config)
    tcp_r = np.asarray(rotation) @ mount_r.T
    tcp_p = np.asarray(translation) - tcp_r @ mount_p
    hole = tcp_p + tcp_r @ np.asarray(config["guide"].get("center_offset", [0., 0., 0.]))
    return hole, tcp_r[:, 0], tcp_r[:, 2]


def initial_straight_half_length(config):
    """Cover the 21 mm CAD bore plus margin and two discrete cable segments."""
    cable, guide = config["cable"], config["guide"]
    spacing = float(cable.get("particle_spacing", .001))
    rope = config.get("rope_actor", {})
    if "links" in rope:
        spacing = max(spacing, cable["pin_length"],
                      (cable["length"]-cable["pin_length"])/(rope["links"]-1))
    return guide["half_length"] + max(guide.get("initial_margin", .025), 2*spacing, cable["diameter"])


def _threaded_path(config, rotation, translation, hole, axis):
    """Construct the selected USB-to-right or two-bore creation route.

    The USB exits beyond the left finger's +X face (reverse orientation).
    Returning outside the fingertips is necessary: a line drawn directly
    from that exit to the right hand never passes the left bore. The raised
    grip keeps the USB body clear of the cable passing through the bore.
    Only creation/reset uses this geometry; it does not constrain later motion.
    """
    hole = np.asarray(hole, dtype=float)
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    attachment = rotation @ np.asarray(config["usb"]["attachment"]) + translation
    direction = -rotation[:, 1]
    left_hole, _, outward = left_guide_geometry(config, rotation, translation)
    half = initial_straight_half_length(config)
    collar = config["cable"]["pin_length"] + .008
    a = attachment + collar * direction
    if config["guide"].get("routing", "both_guides") == "usb_to_right":
        # Leave the real USB exit in its axial direction, then smoothly join
        # the straight right bore. No return loop, left-bore constraint/support.
        if np.dot(axis, hole - a) < 0:
            axis = -axis
        b, d = hole-half*axis, hole+half*axis
        distance = np.linalg.norm(b-a)
        if distance < .025:
            raise ValueError("Right guide is too close to the USB cable exit")
        handle = min(.06, distance*.3)
        lead = _bezier(a, a+handle*direction, b-handle*axis, b)
        straight = np.linspace(b, d, 201)
        path = np.vstack((attachment, lead, straight[1:],
                          d+config["cable"]["length"]*axis))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        guide_arc = collar+np.linalg.norm(np.diff(lead, axis=0), axis=1).sum()+half
        if config["cable"]["length"] <= guide_arc+half+.02:
            raise ValueError("Cable is too short to reach the right bore and leave a free tail")
        return path, arc, None, guide_arc
    left_entry, left_exit = left_hole-half*direction, left_hole+half*direction
    loop_height = config["guide"].get("left_loop_clearance", .040)
    # Positive TCP Z points outside the fingertips; this avoids crossing the
    # USB, palm, and the X faces of the original research-finger CAD.
    apex = left_hole + loop_height*outward
    handle = max(.035, half)
    turn_out = _bezier(a, a+handle*direction, apex+handle*direction, apex)
    turn_back = _bezier(apex, apex-handle*direction,
                        left_entry-handle*direction, left_entry)
    left_straight = np.linspace(left_entry, left_exit, 201)
    # A hole has no preferred direction of travel: choose the side facing USB.
    if np.dot(axis, hole - left_exit) < 0:
        axis = -axis
    b = hole - half * axis
    d = hole + half * axis
    distance = np.linalg.norm(b - left_exit)
    if distance < .025:
        raise ValueError("The two TCPs are too close to insert the USB cable")
    handle = min(.12, distance * .3)
    lead = _bezier(left_exit, left_exit + handle*direction, b - handle*axis, b)
    straight = np.linspace(b, d, 201)
    tail = d + config["cable"]["length"]*axis
    path = np.vstack((attachment, turn_out, turn_back[1:], left_straight[1:],
                      lead[1:], straight[1:], tail))
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    left_arc = collar + sum(np.linalg.norm(np.diff(part, axis=0), axis=1).sum()
                            for part in (turn_out, turn_back)) + half
    guide_arc = left_arc + half + np.linalg.norm(np.diff(lead, axis=0), axis=1).sum() + half
    length = config["cable"]["length"]
    if length <= guide_arc + half + .02:
        raise ValueError("Cable is too short for the exterior USB loop, both guide bores and a free tail")
    return path, arc, left_arc, guide_arc


def threaded_centerline(config, s, rotation, translation, hole, axis):
    """Sample the configured route by material arc; return right guide arc [m]."""
    path, arc, _, guide_arc = _threaded_path(config, rotation, translation, hole, axis)
    center = np.column_stack([np.interp(s, arc, path[:, i]) for i in range(3)])
    return center, guide_arc


def bore_alignment(centers, hole, axis, half_length, *, radial_tolerance=.0001,
                   axis_tolerance_deg=1.):
    """Read-only polyline/bore check; inspect segments, including sparse ropes.

    Multiple plane crossings (e.g. the exterior loop) are resolved by radial
    distance. Coverage is the contiguous straight neighborhood of that crossing.
    No samples are projected and no constraints are created.
    """
    points, hole, axis = np.asarray(centers), np.asarray(hole), np.asarray(axis, dtype=float)
    axis = axis/np.linalg.norm(axis)
    delta, edges = points-hole, np.diff(points, axis=0)
    along = delta @ axis
    radial = np.linalg.norm(delta-along[:, None]*axis, axis=1)
    lengths = np.linalg.norm(edges, axis=1)
    angles = np.degrees(np.arccos(np.clip(np.abs(edges @ axis)/np.maximum(lengths, 1.e-12), 0., 1.)))
    crossing = np.flatnonzero((along[:-1]*along[1:] <= 0.) & (np.abs(np.diff(along)) > 1.e-12))
    base = dict(available=True, frame="world", radial_tolerance_m=float(radial_tolerance),
                axis_tolerance_deg=float(axis_tolerance_deg), required_range_m=[-half_length, half_length])
    if not len(crossing):
        return dict(base, passed=False, reason="no_bore_plane_crossing", radial_error_m=None,
                    axis_error_deg=None, straight_coverage_range_m=None)
    fractions = -along[crossing]/np.diff(along)[crossing]
    cross_points = delta[crossing]+fractions[:, None]*edges[crossing]
    selected = int(crossing[np.argmin(np.linalg.norm(cross_points, axis=1))])
    straight = (np.maximum(radial[:-1], radial[1:]) <= radial_tolerance) & (angles <= axis_tolerance_deg)
    lo = hi = selected
    while lo > 0 and straight[lo-1]:
        lo -= 1
    while hi+1 < len(straight) and straight[hi+1]:
        hi += 1
    region = np.arange(lo, hi+2)
    coverage = [float(along[region].min()), float(along[region].max())]
    core = np.arange(lo, hi+1)
    core = core[(np.minimum(along[core], along[core+1]) < half_length) &
                (np.maximum(along[core], along[core+1]) > -half_length)]
    radial_error = float(radial[np.unique(np.r_[core, core+1])].max()) if len(core) else float(radial[selected:selected+2].max())
    axis_error = float(angles[core].max()) if len(core) else float(angles[selected])
    passed = bool(straight[selected] and coverage[0] <= -half_length and coverage[1] >= half_length)
    return dict(base, passed=passed, reason="initial_straight_bore_verified" if passed else "bore_alignment_or_coverage_failed",
                radial_error_m=radial_error, axis_error_deg=axis_error,
                straight_coverage_range_m=coverage)


def initial_layout_report(config, centers, rotation, translation, hole, axis):
    """Serializable geometry evidence and short material intervals for support."""
    guide = config["guide"]
    left_hole, left_axis, _ = left_guide_geometry(config, rotation, translation)
    _, _, left_arc, right_arc = _threaded_path(config, rotation, translation, hole, axis)
    kwargs = dict(half_length=guide["half_length"],
                  radial_tolerance=guide.get("radial_tolerance", .0001),
                  axis_tolerance_deg=guide.get("axis_tolerance_deg", 1.))
    left = (bore_alignment(centers, left_hole, left_axis, **kwargs) if left_arc is not None else
            dict(available=False, required=False, passed=None,
                 reason="disabled_usb_to_right_routing", radial_error_m=None,
                 axis_error_deg=None, straight_coverage_range_m=None))
    right = bore_alignment(centers, hole, axis, **kwargs)
    # Material arc estimates are for selecting a *local* temporary support.
    # They are not fixed particles and must be removed with USB world support.
    support_half = guide["half_length"] + (initial_straight_half_length(config)-guide["half_length"])/2
    return dict(passed=(left_arc is None or left["passed"]) and right["passed"],
                routing=guide.get("routing", "both_guides"), units="m, deg", left=left, right=right,
                left_hole_world_m=left_hole.tolist(), right_hole_world_m=np.asarray(hole).tolist(),
                left_axis_world=left_axis.tolist(), right_axis_world=np.asarray(axis).tolist(),
                usb_attachment_world_m=(rotation @ np.asarray(config["usb"]["attachment"])+translation).tolist(),
                left_material_arc_m=None if left_arc is None else float(left_arc), right_material_arc_m=float(right_arc),
                support_material_intervals_m=[[float(v-support_half), float(v+support_half)]
                                              for v in (left_arc, right_arc) if v is not None])


def threaded_positions(config, local, rotation, translation, hole, axis):
    """Seven-point MPM cross sections on the common threaded curve."""
    sections = len(local) // 7
    length = config["cable"]["length"]
    s = np.linspace(0., length, sections)
    center, guide_arc = threaded_centerline(config, s, rotation, translation, hole, axis)
    tangent = np.gradient(center, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    x_axes = np.empty_like(tangent)
    previous = rotation[:, 0]
    for i, axis_i in enumerate(tangent):
        projected = previous - axis_i*np.dot(previous, axis_i)
        if np.linalg.norm(projected) < 1e-8:
            projected = np.cross(axis_i, np.eye(3)[np.argmin(np.abs(axis_i))])
        previous = projected / np.linalg.norm(projected)
        x_axes[i] = previous
    z_axes = np.cross(x_axes, -tangent)
    points = local.reshape(sections, 7, 3)
    offsets = points - points.mean(axis=1)[:, None, :]
    world = center[:, None, :] + offsets[:, :, 0, None]*x_axes[:, None, :] + offsets[:, :, 2, None]*z_axes[:, None, :]
    world = world.reshape(-1, 3)
    pinned = np.repeat(s <= config["cable"]["pin_length"], 7)
    world[pinned] = local[pinned] @ rotation.T + translation
    return world, guide_arc / length * (sections - 1)


def guide_projection(centers, hole, axis, material_coordinate, half_length, pin_sections):
    """Project a continuous material neighborhood onto a finite frictionless bore.

    Re-find the plane-crossing segment each iteration; no material point is
    pinned to the right gripper. Loss of threading is reported, never hidden by
    attaching an unrelated part of the cable.
    """
    centers = np.asarray(centers, dtype=float)
    axis = np.array(axis, dtype=float, copy=True)
    if (centers.ndim != 2 or centers.shape[1] != 3 or len(centers) < 4
            or not np.isfinite(centers).all() or not np.isfinite(hole).all()
            or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-10
            or not np.isfinite(material_coordinate) or half_length <= 0):
        raise RuntimeError("Invalid right TCP guide state")
    axis /= np.linalg.norm(axis)
    relative = centers - hole
    along = relative @ axis
    crossing = np.flatnonzero(along[:-1]*along[1:] <= 0.)
    denominator = along[crossing + 1] - along[crossing]
    valid = np.abs(denominator) > 1e-10
    crossing, denominator = crossing[valid], denominator[valid]
    coordinate = crossing - along[crossing] / denominator
    nearby = np.abs(coordinate - material_coordinate) <= 32
    coordinate = coordinate[nearby]
    if not len(coordinate):
        raise RuntimeError("Cable no longer crosses the right TCP guide; stop and reset the scene")
    selected = coordinate[np.argmin(np.abs(coordinate - material_coordinate))]
    if selected <= pin_sections + 2 or selected >= len(centers) - 3:
        raise RuntimeError("USB fixed end or cable free end reached the right TCP guide")
    spacing = np.linalg.norm(np.diff(centers, axis=0), axis=1)
    arc = np.r_[0., np.cumsum(spacing)]
    contact_arc = np.interp(selected, np.arange(len(centers)), arc)
    # Taper by material arc length. An axial-distance cutoff becomes abrupt
    # when the free cable bends near the mouth, separating neighboring sections.
    distance = np.abs(arc - contact_arc)
    transition = max(.02, 4*np.max(spacing))
    u = np.clip((distance-half_length)/transition, 0., 1.)
    weight = .5*(1. + np.cos(np.pi*u))
    weight[:pin_sections] = 0.
    radial = relative - along[:, None]*axis
    return -radial*weight[:, None], weight, float(selected)
