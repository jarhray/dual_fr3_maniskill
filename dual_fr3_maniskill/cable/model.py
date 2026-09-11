"""Geometry shared by MoveIt, SAPIEN and the cable boundary condition."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

USB_LINK = "usb_cable_demo_plug"
TCP_LINK = "left_fr3_hand_tcp"


def load_config(path):
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    for section in ("usb", "cable", "mpm"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"Missing {section} configuration")
    c, m = config["cable"], config["mpm"]
    c.setdefault("contact_margin", 0.00002)
    c.setdefault("contact_iterations", 2)
    c.setdefault("penetration_tolerance", 0.0001)
    c.setdefault("initial_layout", "table_spiral")
    if c.get("initial_layout", "straight") not in ("straight", "table_spiral"):
        raise ValueError("initial_layout must be straight or table_spiral")
    for key in ("attachment", "grip_center"):
        vector = np.asarray(config["usb"][key])
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ValueError(f"usb.{key} must have three finite coordinates")
    for key in ("mass", "mesh_scale", "finger_position"):
        if not np.isfinite(config["usb"][key]) or config["usb"][key] <= 0:
            raise ValueError(f"usb.{key} must be positive and finite")
    if config["usb"]["finger_position"] > .04:
        raise ValueError("USB grip exceeds the FR3 finger travel")
    if not np.isfinite(c["friction"]) or c["friction"] < 0:
        raise ValueError("cable.friction must be nonnegative and finite")
    for key in ("length", "diameter", "particle_spacing", "pin_length", "density",
                "young_modulus", "axial_young_modulus", "yield_stress", "contact_margin", "penetration_tolerance"):
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f"cable.{key} must be positive and finite")
    if not 0 < c["poisson_ratio"] < 0.49 or c["pin_length"] >= c["length"]:
        raise ValueError("Invalid Poisson ratio or pin length")
    if not isinstance(c["axial_iterations"], int) or c["axial_iterations"] < 1:
        raise ValueError("axial_iterations must be a positive integer")
    if not isinstance(c["contact_iterations"], int) or c["contact_iterations"] < 1:
        raise ValueError("contact_iterations must be a positive integer")
    if c["contact_margin"] >= c["diameter"] / 2 or c["penetration_tolerance"] >= c["diameter"] / 2:
        raise ValueError("Contact margin and penetration tolerance must be smaller than the cable radius")
    for key in ("grid_spacing", "max_grid_spacing", "frequency", "max_grid_cells", "grid_padding", "marker_stride"):
        if not np.isfinite(m[key]) or m[key] <= 0:
            raise ValueError(f"mpm.{key} must be positive and finite")
    if m["max_grid_spacing"] < m["grid_spacing"]:
        raise ValueError("max_grid_spacing must be at least grid_spacing")
    for key in ("frequency", "max_grid_cells", "grid_padding", "marker_stride"):
        if not isinstance(m[key], int):
            raise ValueError(f"mpm.{key} must be an integer")
    if m["grid_padding"] < 6:
        raise ValueError("grid_padding must be at least 6 for the MPM stencil")
    if c["particle_spacing"] > c["diameter"] / 2:
        raise ValueError("particle_spacing must be no greater than half the diameter")
    guide = config.setdefault("guide", {})
    guide.setdefault("half_length", .012)
    for key in ("half_length",):
        if not np.isfinite(guide[key]) or guide[key] <= 0:
            raise ValueError(f"guide.{key} must be positive and finite")
    return config


def usb_mount(config):
    # With the left wrist at 3*pi/4 this leaves the USB's local +Y along world
    # +Y initially. Its X width lies between the fingers; nothing rotates the
    # STL itself or reinterprets the user-specified attachment coordinates.
    rotation = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
    return -rotation @ np.asarray(config["usb"]["grip_center"]), rotation


def add_usb_description(description, semantic, mesh_path, config, *, mesh_uri=None):
    robot, srdf = ET.fromstring(description), ET.fromstring(semantic)
    if robot.find(f"link[@name='{USB_LINK}']") is not None:
        raise ValueError("USB is already present in the description")
    if robot.find(f"link[@name='{TCP_LINK}']") is None:
        raise ValueError(f"USB cable scene requires the Franka hand TCP link {TCP_LINK!r}")
    mesh_path = Path(mesh_path).resolve(strict=True)
    link = ET.SubElement(robot, "link", name=USB_LINK)
    for tag in ("visual", "collision"):
        element = ET.SubElement(link, tag)
        geometry = ET.SubElement(element, "geometry")
        scale = str(config["usb"]["mesh_scale"])
        ET.SubElement(geometry, "mesh", filename=mesh_uri or str(mesh_path), scale=" ".join([scale] * 3))
        if tag == "visual":
            material = ET.SubElement(element, "material", name="usb_demo_blue")
            ET.SubElement(material, "color", rgba="0.08 0.25 0.65 1")
    inertial = ET.SubElement(link, "inertial")
    # Bounding-box inertia approximation for USB1.stl in metres. The grip
    # origin is at y=0; the box spans y=[-0.020, 0.0179].
    ET.SubElement(inertial, "origin", xyz="0 -0.00105 0", rpy="0 0 0")
    mass = float(config["usb"]["mass"])
    ET.SubElement(inertial, "mass", value=str(mass))
    size = np.array([0.0076, 0.0379, 0.0144])
    inertia = mass / 12 * (np.sum(size ** 2) - size ** 2)
    ET.SubElement(inertial, "inertia", ixx=str(inertia[0]), iyy=str(inertia[1]),
                  izz=str(inertia[2]), ixy="0", ixz="0", iyz="0")
    joint = ET.SubElement(robot, "joint", name="usb_cable_demo_mount", type="fixed")
    ET.SubElement(joint, "parent", link=TCP_LINK)
    ET.SubElement(joint, "child", link=USB_LINK)
    translation, rotation = usb_mount(config)
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, translation)),
                  rpy=f"{np.pi} 0 {-np.pi / 2}")
    for name in (TCP_LINK, "left_fr3_hand", "left_fr3_leftfinger", "left_fr3_rightfinger"):
        ET.SubElement(srdf, "disable_collisions", link1=USB_LINK, link2=name, reason="Attached")
    for state in srdf.findall("group_state"):
        if state.get("name") in ("ready", "both_ready"):
            for j in state.findall("joint"):
                if j.get("name") == "left_fr3_joint7":
                    j.set("value", str(3 * np.pi / 4))
    return ET.tostring(robot, encoding="unicode"), ET.tostring(srdf, encoding="unicode")


def cable_particles(config):
    """Seven volume samples per circular section, total rest volume exactly pi*r²*L."""
    c = config["cable"]
    radius = c["diameter"] / 2
    sections = int(np.ceil(c["length"] / c["particle_spacing"])) + 1
    s = np.linspace(0, c["length"], sections)
    theta = np.arange(6) * np.pi / 3
    cross = np.zeros((7, 3))
    cross[1:, 0] = np.cos(theta) * radius * 2 / 3
    cross[1:, 2] = np.sin(theta) * radius * 2 / 3
    center = np.tile(np.asarray(config["usb"]["attachment"]), (sections, 1))
    center[:, 1] -= s
    positions = (center[:, None, :] + cross[None, :, :]).reshape(-1, 3)
    ds = c["length"] / (sections - 1)
    weights = np.full(sections, ds)
    weights[[0, -1]] *= 0.5
    volumes = np.repeat(weights * np.pi * radius ** 2 / 7, 7)
    pinned = np.repeat(s <= c["pin_length"], 7)
    return positions, volumes, pinned, sections


def grid_layout(points, config, spacing=None):
    """Bound memory before allocation; never let upstream silently clamp a long cable."""
    m = config["mpm"]
    spacing = max(float(spacing or m["grid_spacing"]), m["grid_spacing"])
    span = np.ptp(np.asarray(points), axis=0)
    if span.shape != (3,) or not np.isfinite(span).all():
        raise ValueError("MPM bounds must contain finite 3D points")
    while True:
        dims = np.ceil(span / spacing).astype(int) + 2 * int(m["grid_padding"])
        dims = (dims + 15) // 16 * 16
        if int(np.prod(dims.astype(np.int64))) <= m["max_grid_cells"]:
            return spacing, tuple(map(int, dims))
        if not m["adaptive_resolution"] or spacing * 1.2 > m["max_grid_spacing"]:
            raise RuntimeError("MPM grid exceeds its memory budget; reset the demo or increase grid_spacing")
        spacing *= 1.2


def initial_particle_positions(config, local, rotation, translation):
    """Rest-shaped cable laid on the current table, with a straight clamp collar.

    Resample by arc length to preserve the configured material length.
    The spiral is an initial condition, never a constraint during simulation.
    """
    if config["cable"].get("initial_layout", "straight") == "straight":
        return local @ rotation.T + translation
    c = config["cable"]
    attachment = rotation @ np.asarray(config["usb"]["attachment"]) + translation
    direction = rotation @ np.array([0., -1., 0.])
    collar = c["pin_length"] + .005
    p0 = attachment + direction * collar
    p1 = p0 + direction * .14
    # Keep the initial coil to the +X side of the bases at (0.175, 0.35)
    # and (0.175, 0.95). The old ellipse crossed the right base before the
    # first simulation step, which no collision solver can undo faithfully.
    p3 = np.array([.51, .31, .08])
    p2 = p3 - np.array([.15, 0., 0.])
    t = np.linspace(0, 1, 1500)[:, None]
    bridge = (1-t)**3*p0 + 3*(1-t)**2*t*p1 + 3*(1-t)*t**2*p2 + t**3*p3
    theta = np.linspace(-np.pi/2, 3.5*np.pi, 8000)
    shrink = np.linspace(1., .5, len(theta))
    spiral = np.column_stack((.51 + .18*shrink*np.cos(theta),
                              .65 + .34*shrink*np.sin(theta), np.full(len(theta), .08)))
    # An additional inner turn preserves support for the original 3 m cable
    # after narrowing the ellipse to avoid the robot bases.
    theta_extra = np.linspace(3.5*np.pi, 5.5*np.pi, 4000)
    shrink_extra = np.linspace(.5, .35, len(theta_extra))
    inner = np.column_stack((.51 + .18*shrink_extra*np.cos(theta_extra),
                            .65 + .34*shrink_extra*np.sin(theta_extra), np.full(len(theta_extra), .08)))
    spiral = np.vstack([spiral, inner[1:]])
    path = np.vstack([attachment, bridge, spiral[1:]])
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    if arc[-1] < c["length"]:
        raise ValueError("table_spiral supports this scene's cable lengths up to %.3f m" % arc[-1])
    sections = len(local) // 7
    s = np.linspace(0., c["length"], sections)
    center = np.column_stack([np.interp(s, arc, path[:, axis]) for axis in range(3)])
    tangent = np.gradient(center, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    # Construct a continuous frame from the initial USB X axis.
    x_axis = np.empty_like(tangent)
    previous = rotation[:, 0]
    for i, axis in enumerate(tangent):
        projected = previous - axis * np.dot(previous, axis)
        previous = projected / np.linalg.norm(projected)
        x_axis[i] = previous
    z_axis = np.cross(x_axis, -tangent)
    offsets = local.reshape(sections, 7, 3) - local.reshape(sections, 7, 3).mean(axis=1)[:, None, :]
    world = center[:, None, :] + offsets[:, :, 0, None]*x_axis[:, None, :] + offsets[:, :, 2, None]*z_axis[:, None, :]
    # Enforce the exact user-specified rigid collar, independent of curve sampling.
    pinned = np.repeat(s <= c["pin_length"], 7)
    world = world.reshape(-1, 3)
    world[pinned] = local[pinned] @ rotation.T + translation
    return world
