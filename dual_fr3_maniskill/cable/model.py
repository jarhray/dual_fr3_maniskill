"""Geometry shared by MoveIt, SAPIEN and the cable boundary condition."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

USB_LINK = "usb_cable_demo_plug"
TCP_LINK = "left_fr3_hand_tcp"


def load_config(path, *, solver="mpm"):
    """Validate common geometry and only the selected solver's settings.

    ``solver=None`` is for MoveIt geometry consumers, which do not run physics.
    The launch/ROS parameter is the single source of solver selection.
    """
    from dual_fr3_maniskill.cable.backends import validate_solver
    if solver is not None:
        validate_solver(solver)
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Cable configuration must be a mapping")
    scene = config.setdefault("scene", {})
    if not isinstance(scene, dict):
        raise ValueError("scene configuration must be a mapping")
    for key in ("trunking_mesh", "trunking_visual_mesh"):
        if key in scene and scene[key] not in ("original", "simplified"):
            raise ValueError(f"scene.{key} must be original or simplified")
    for section in ("usb", "cable"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"Missing {section} configuration")
    insertion = config.get("insertion", {})
    if insertion:
        from dual_fr3_maniskill.usb.insertion import InsertionLimits, read_local_collision_check
        from dual_fr3_maniskill.usb.alignment import AlignmentLimits
        AlignmentLimits.read(insertion.get('alignment', {}), InsertionLimits.read(insertion))
        read_local_collision_check(insertion)
        for name in ("enabled", "retain_after_success", "release_after_retention", "return_after_release"):
            if name in insertion and not isinstance(insertion[name], bool):
                raise ValueError("insertion."+name+" must be boolean")
        for name, size in (("xy_m", 2), ("hole_center_m", 3), ("quaternion_wxyz", 4), ("clearance_yz_m", 2)):
            if name in insertion:
                value = np.asarray(insertion[name], dtype=float)
                if value.shape != (size,) or not np.isfinite(value).all():
                    raise ValueError("Invalid insertion."+name)
        if "clearance_yz_m" in insertion and not all(0 < x <= .001 for x in insertion["clearance_yz_m"]):
            raise ValueError("Insertion per-side clearance must be in (0, 1 mm]")
        if insertion.get("rest_offset_m", 0.) != 0.:
            raise ValueError("Rigid insertion requires zero rest offset")
        if not 0 < insertion.get("contact_offset_m", .0001) <= .0002:
            raise ValueError("Socket contact offset must be in (0, 0.2 mm]")
        for key, default in (("planning_timeout_s", 15.), ("retreat_m", .05)):
            if not np.isfinite(insertion.get(key, default)) or insertion.get(key, default) <= 0:
                raise ValueError("insertion."+key+" must be positive finite")
        attempts = insertion.get("planning_attempts", 10)
        ratio = insertion.get("max_path_length_ratio", 1.5)
        if isinstance(ratio, bool) or not isinstance(ratio, (int, float)) or not np.isfinite(ratio) or ratio < 1.:
            raise ValueError("insertion.max_path_length_ratio must be finite and >= 1.0")
        if not isinstance(attempts, int) or isinstance(attempts, bool) or attempts < 1:
            raise ValueError("insertion.planning_attempts must be a positive integer")
        for side, width, minimum in (("left", .08, .035), ("right", .03, .014)):
            width = insertion.get("release_width_m", {}).get(side, width)
            minimum = insertion.get("release_min_half_width_m", {}).get(side, minimum)
            if not 0 < width <= .08 or not (.005 if side == "left" else .003) <= minimum <= width/2:
                raise ValueError("Insertion release opening must clear USB/split bore and fit gripper travel")
        if config["usb"].get("mesh_scale", 1.) != 1.:
            raise ValueError("Insertion CAD and USB must use metres without scaling")
        if np.linalg.norm(insertion.get("quaternion_wxyz", [1.,0,0,0])) < 1e-8:
            raise ValueError("Socket quaternion must be nonzero")
        if abs(insertion.get("hole_center_m", [0.,.0175,.0686])[0]) > 1e-8:
            raise ValueError("Socket mouth must remain on CAD X=0")
    c = config["cable"]
    c.setdefault("contact_margin", 0.00002)
    c.setdefault("contact_iterations", 2)
    c.setdefault("penetration_tolerance", 0.0001)
    c.setdefault("initial_layout", "table_spiral")
    if c["initial_layout"] not in ("straight", "table_spiral"):
        raise ValueError("initial_layout must be straight or table_spiral")

    def positive(section, keys):
        for key in keys:
            value = config[section].get(key)
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not np.isfinite(value) or value <= 0):
                raise ValueError(f"{section}.{key} must be positive and finite")

    def integer(section, keys):
        positive(section, keys)
        for key in keys:
            if not isinstance(config[section][key], int):
                raise ValueError(f"{section}.{key} must be an integer")

    config["usb"].setdefault("tcp_grip_offset", [0., 0., .012])
    config["usb"].setdefault("contact_offset", .0002)
    config["usb"].setdefault("finger_contact_offset", .0001)
    positive("usb", ("contact_offset", "finger_contact_offset"))
    for key in ("attachment", "grip_center", "tcp_grip_offset"):
        vector = np.asarray(config["usb"].get(key), dtype=float)
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ValueError(f"usb.{key} must have three finite coordinates")
    positive("usb", ("mass", "mesh_scale", "finger_position"))
    if config["usb"]["finger_position"] > .04:
        raise ValueError("USB grip exceeds the FR3 finger travel")
    friction = c.get("friction")
    if not isinstance(friction, (int, float)) or not np.isfinite(friction) or friction < 0:
        raise ValueError("cable.friction must be nonnegative and finite")
    positive("cable", ("length", "diameter", "pin_length", "contact_margin", "penetration_tolerance"))
    if "linear_density" in c:
        positive("cable", ("linear_density",))
        # kg/m is the mass input when available; density remains the resolved
        # kg/m^3 value consumed by both solvers and saved in trace metadata.
        c["density"] = c["linear_density"]/(np.pi*(c["diameter"]/2)**2)
    positive("cable", ("density",))
    if c["pin_length"] >= c["length"]:
        raise ValueError("Invalid pin length")
    integer("cable", ("contact_iterations",))
    if c["contact_margin"] >= c["diameter"] / 2 or c["penetration_tolerance"] >= c["diameter"] / 2:
        raise ValueError("Contact margin and penetration tolerance must be smaller than the cable radius")
    guide = config.setdefault("guide", {})
    if not isinstance(guide, dict):
        raise ValueError("guide configuration must be a mapping")
    guide.setdefault("routing", "both_guides")
    if guide["routing"] not in ("both_guides", "usb_to_right"):
        raise ValueError("guide.routing must be both_guides or usb_to_right")
    guide.setdefault("half_length", .012)
    guide.setdefault("center_offset", [0., 0., 0.])
    guide.setdefault("initial_margin", .025)
    guide.setdefault("left_loop_clearance", .040)
    guide.setdefault("radial_tolerance", .0001)
    guide.setdefault("axis_tolerance_deg", 1.)
    offset = np.asarray(guide["center_offset"], dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("guide.center_offset must have three finite coordinates")
    positive("guide", ("half_length", "initial_margin", "left_loop_clearance",
                       "radial_tolerance", "axis_tolerance_deg"))
    if guide["axis_tolerance_deg"] >= 90:
        raise ValueError("guide.axis_tolerance_deg must be smaller than 90 degrees")
    if guide["radial_tolerance"] >= c["diameter"]/2:
        raise ValueError("guide.radial_tolerance must be smaller than the cable radius")
    display = config.setdefault("display", {})
    if not isinstance(display, dict):
        raise ValueError("display configuration must be a mapping")
    legacy_mpm = config.get("mpm", {})
    display.setdefault("marker_stride", legacy_mpm.get("marker_stride", 1)
                       if isinstance(legacy_mpm, dict) and solver == "mpm" else 1)
    integer("display", ("marker_stride",))

    if solver == "mpm":
        if not isinstance(config.get("mpm"), dict):
            raise ValueError("Missing mpm configuration")
        m = config["mpm"]
        for key in ("cuda_graph", "gpu_grid_check"):
            m.setdefault(key, True)
            if not isinstance(m[key], bool):
                raise ValueError(f"mpm.{key} must be boolean")
        positive("cable", ("particle_spacing", "young_modulus", "axial_young_modulus", "yield_stress"))
        if not 0 < c.get("poisson_ratio", 0) < .49:
            raise ValueError("Invalid Poisson ratio")
        integer("cable", ("axial_iterations",))
        positive("mpm", ("grid_spacing", "max_grid_spacing"))
        integer("mpm", ("frequency", "max_grid_cells", "grid_padding", "marker_stride"))
        if m["max_grid_spacing"] < m["grid_spacing"]:
            raise ValueError("max_grid_spacing must be at least grid_spacing")
        if m["grid_padding"] < 6:
            raise ValueError("grid_padding must be at least 6 for the MPM stencil")
        if c["particle_spacing"] > c["diameter"] / 2:
            raise ValueError("particle_spacing must be no greater than half the diameter")
    elif solver == "rope_actor":
        r = config.setdefault("rope_actor", {})
        if not isinstance(r, dict):
            raise ValueError("rope_actor configuration must be a mapping")
        defaults = dict(links=60, frequency=1000, adaptive_timestep=True, contact_offset=.00002, inertia_floor=1.1e-8,
                        solver_type="tgs", root_joint="fixed", collision_geometry="capsule",
                        max_contact_travel=.00005,
                        engine_tolerance_length=.1, engine_tolerance_speed=.2,
                        twist_limit_deg=85., bend_limit_deg=85.,
                        joint_stiffness=0., joint_damping=.001,
                        linear_damping=1., angular_damping=1.,
                        tail_weight_mass=0., tail_weight_radius=.004,
                        solver_iterations=40, solver_velocity_iterations=10,
                        constraint_tolerance=0.001, max_speed=10.0)
        for key, value in defaults.items():
            r.setdefault(key, value)
        positive("rope_actor", ("tail_weight_radius",))
        weight = r["tail_weight_mass"]
        if (not isinstance(weight, (int, float)) or isinstance(weight, bool)
                or not np.isfinite(weight) or weight < 0):
            raise ValueError("rope_actor.tail_weight_mass must be nonnegative and finite")
        if weight > 0 and r["tail_weight_radius"] < c["diameter"]/2:
            raise ValueError("rope_actor.tail_weight_radius must cover the cable radius")
        if not isinstance(r["adaptive_timestep"], bool):
            raise ValueError("rope_actor.adaptive_timestep must be boolean")
        if r["solver_type"] not in ("pgs", "tgs"):
            raise ValueError("rope_actor.solver_type must be pgs or tgs")
        if r["root_joint"] not in ("fixed", "spherical"):
            raise ValueError("rope_actor.root_joint must be fixed or spherical")
        if r["collision_geometry"] not in ("capsule", "convex_capsule"):
            raise ValueError("rope_actor.collision_geometry must be capsule or convex_capsule")
        integer("rope_actor", ("links", "frequency", "solver_iterations", "solver_velocity_iterations"))
        # PhysX rigid-body iteration counts are limited to 8 bits. Reject
        # larger values before SAPIEN can pass an unsupported count through.
        for key in ("solver_iterations", "solver_velocity_iterations"):
            if r[key] > 255:
                raise ValueError(f"rope_actor.{key} must be between 1 and 255")
        if not 6 <= r["links"] <= 256:
            raise ValueError("rope_actor.links must be between 6 and 256")
        positive("rope_actor", ("twist_limit_deg", "bend_limit_deg", "constraint_tolerance", "max_speed", "contact_offset", "inertia_floor", "engine_tolerance_length", "engine_tolerance_speed"))
        positive("rope_actor", ("max_contact_travel",))
        # Optional whole-cable guard: small errors at many joints can add up
        # to visible stretching while every individual joint still passes.
        if r.get("max_stretch_ratio") is not None:
            positive("rope_actor", ("max_stretch_ratio",))
            if r["max_stretch_ratio"] > 1:
                raise ValueError("rope_actor.max_stretch_ratio must not exceed 1")
        if r["max_contact_travel"] > c["diameter"]/4:
            raise ValueError("rope_actor.max_contact_travel must not exceed half the cable radius")
        if r["inertia_floor"] <= 1.e-8:
            raise ValueError("rope_actor.inertia_floor must exceed SAPIEN 2 minimum 1e-8")
        if max(r["twist_limit_deg"], r["bend_limit_deg"]) >= 180:
            raise ValueError("Rope joint limits must be smaller than 180 degrees")
        for key in ("joint_stiffness", "joint_damping", "linear_damping", "angular_damping"):
            value = r[key]
            if not isinstance(value, (int, float)) or not np.isfinite(value) or value < 0:
                raise ValueError(f"rope_actor.{key} must be nonnegative and finite")
        if min(c["pin_length"], (c["length"]-c["pin_length"])/(r["links"]-1)) <= c["diameter"]:
            raise ValueError("Rope segments and pin_length must exceed cable.diameter; reduce rope_actor.links")
    return config


def load_geometry_config(path):
    return load_config(path, solver=None)


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


def initial_centerline(config, s, rotation, translation):
    """Sample the shared initial curve at material distances in metres."""
    if config["cable"].get("initial_layout", "straight") == "straight":
        attachment = rotation @ np.asarray(config["usb"]["attachment"]) + translation
        return attachment[None, :] - np.asarray(s)[:, None]*rotation[:, 1]
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
    center = np.column_stack([np.interp(s, arc, path[:, axis]) for axis in range(3)])
    return center


def initial_particle_positions(config, local, rotation, translation):
    """Seven-point MPM cross sections on the common initial curve."""
    if config["cable"].get("initial_layout", "straight") == "straight":
        return local @ rotation.T + translation
    c = config["cable"]
    sections = len(local) // 7
    s = np.linspace(0., c["length"], sections)
    center = initial_centerline(config, s, rotation, translation)
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
