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
    grip_offset = np.asarray(config["usb"].get("tcp_grip_offset", [0., 0., .0075]))
    return grip_offset - rotation @ np.asarray(config["usb"]["grip_center"]), rotation


def _bezier(a, b, c, d, count=1000):
    t = np.linspace(0., 1., count)[:, None]
    return (1-t)**3*a + 3*(1-t)**2*t*b + 3*(1-t)*t*t*c + t**3*d


def threaded_centerline(config, s, rotation, translation, hole, axis):
    """Sample the USB-to-eyelet initial curve; return guide arc length in metres."""
    hole = np.asarray(hole, dtype=float)
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    attachment = rotation @ np.asarray(config["usb"]["attachment"]) + translation
    direction = -rotation[:, 1]
    # A hole has no preferred direction of travel: choose the side facing USB.
    if np.dot(axis, hole - attachment) < 0:
        axis = -axis
    collar = config["cable"]["pin_length"] + .008
    half = config["guide"]["half_length"] + .025
    a = attachment + collar * direction
    b = hole - half * axis
    d = hole + half * axis
    distance = np.linalg.norm(b - a)
    if distance < .025:
        raise ValueError("The two TCPs are too close to insert the USB cable")
    handle = min(.12, distance * .3)
    lead = _bezier(a, a + handle*direction, b - handle*axis, b)
    straight = np.linspace(b, d, 201)
    tail = d + config["cable"]["length"]*axis
    path = np.vstack((attachment, lead, straight[1:], tail))
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    guide_arc = collar + np.linalg.norm(np.diff(lead, axis=0), axis=1).sum() + half
    length = config["cable"]["length"]
    if length <= guide_arc + half + .02:
        raise ValueError("Cable is too short to pass through the right TCP with a free tail")
    center = np.column_stack([np.interp(s, arc, path[:, i]) for i in range(3)])
    return center, guide_arc


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
