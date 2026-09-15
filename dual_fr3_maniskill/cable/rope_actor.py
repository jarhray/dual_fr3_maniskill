"""Rope-Actor capsule chain adapted to SAPIEN 2 using native spherical D6 joints.

The reference has three revolute joints and two helper links per capsule.
A D6 joint provides those three rotational freedoms without helper bodies or
SAPIEN 2's 64-link articulation limit. Physics and contacts run in PhysX.
"""
import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat

from ..sapien_compat import sapien
from .model import USB_LINK, initial_centerline


def segment_frames(points):
    """Continuous orthonormal frames with capsule +X along the centerline."""
    tangents = np.diff(points, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    frames = []
    previous = np.eye(3)[np.argmin(np.abs(tangents[0]))]
    for x in tangents:
        y = previous - x*np.dot(previous, x)
        if np.linalg.norm(y) < 1.e-8:
            y = np.cross(np.eye(3)[np.argmin(np.abs(x))], x)
        y /= np.linalg.norm(y)
        frames.append(np.column_stack((x, y, np.cross(x, y))))
        previous = y
    return np.asarray(frames)


def rigid_resample(curve, lengths):
    """Walk a dense curve using exact chord lengths, extending its final tangent.

    Arc-length samples alone shorten a bent rigid chain and preload its joints.
    Intersect each following polyline edge with a sphere around the last node.
    """
    curve = np.asarray(curve, dtype=float)
    if curve.ndim != 2 or curve.shape[1] != 3 or not np.isfinite(curve).all():
        raise ValueError("Invalid initial rope curve")
    direction = curve[-1]-curve[-2]
    direction /= np.linalg.norm(direction)
    curve = np.vstack((curve, curve[-1]+direction*np.sum(lengths)))
    nodes, cursor = [curve[0]], 0
    for length in lengths:
        center = nodes[-1]
        while cursor+1 < len(curve) and np.linalg.norm(curve[cursor+1]-center) < length:
            cursor += 1
        if cursor+1 == len(curve):
            raise ValueError("Initial rope curve is too short")
        a, edge = curve[cursor]-center, curve[cursor+1]-curve[cursor]
        aa, ab = np.dot(edge, edge), np.dot(a, edge)
        t = (-ab+np.sqrt(max(0., ab*ab-aa*(np.dot(a, a)-length*length))))/aa
        nodes.append(curve[cursor]+np.clip(t, 0., 1.)*edge)
    return np.asarray(nodes)


class RopeActorCable:
    solver = "rope_actor"

    def __init__(self, env, config, *, plug=None, layout=None, guide=None):
        self.env, self.config, self.guide = env, config, guide
        self.scene = env._scene
        self.plug = plug if plug is not None else env.agent.links[USB_LINK]
        self.layout = layout or initial_centerline
        c, r = config["cable"], config["rope_actor"]
        self.radius = c["diameter"]/2
        self.lengths = np.r_[c["pin_length"], np.full(r["links"]-1,
                            (c["length"]-c["pin_length"])/(r["links"]-1))]
        self.material_s = np.r_[0., np.cumsum(self.lengths)]
        self.sections = len(self.material_s)
        self.max_section_gap = float(self.lengths.max()+r["constraint_tolerance"])
        self.links, self.joints, self.proxies, self.changed_shapes = [], [], [], []
        self.anchor = self.aperture = None
        self._collision_files = None
        self.guide_index = None
        self.steps = 0
        self.max_depth = 0.
        self.max_penetration_contact = None
        self._contact_meshes = {}
        self._contact_candidates = {}
        self.contacted_bodies = set()
        self._pending_reactions = []
        self._proxy_targets = []
        self._last_contact_impulse = 0.
        self._last_contact_pair = None
        self._guide_normal_impulse = np.zeros(3)
        self._guide_normal_load_impulse = 0.
        self._energy_before_step = 0.
        self._dt = float(env.sim_timestep)
        self._contact_speed_bound = self._step_limit = None
        self.material = self.scene.create_physical_material(c["friction"], c["friction"], 0.)
        try:
            points = self._initial_points()
            self._configure_contacts()
            if self.guide is not None:
                from .aperture import MeshAperture
                self._guide_sources = [(link, shape) for proxy, link in self.proxies
                    if link.name.startswith("right_") for shape in proxy.get_collision_shapes()]
                if not self._guide_sources:
                    self._guide_sources = [(self.guide.link, shape)
                        for shape in self.guide.link.get_collision_shapes()]
                self.aperture = MeshAperture(self.guide, self._guide_sources,
                    self.radius, c["penetration_tolerance"])
            self._prepare_obstacle_bounds()
            self._audit_layout(points)
            self._build(points)
            self._update_guide()
        except Exception:
            self.close()
            raise

    def _initial_points(self):
        # Pure geometric samples; these are not simulated MPM particles.
        spacing = min(self.radius/2, .0005)
        s = np.linspace(0., self.config["cable"]["length"],
                        int(np.ceil(self.config["cable"]["length"]/spacing))+1)
        curve = self.layout(self.config, s, quat2mat(self.plug.pose.q), self.plug.pose.p)
        points = rigid_resample(curve, self.lengths)
        frames = segment_frames(points)
        bends = np.arccos(np.clip(np.sum(frames[:-1, :, 0]*frames[1:, :, 0], axis=1), -1., 1.))
        if np.any(bends > np.deg2rad(self.config["rope_actor"]["bend_limit_deg"])):
            raise ValueError("Initial rope bend exceeds joint limit; increase rope_actor.links or bend_limit_deg")
        if self.guide is not None:
            self._guide_crossing(points, initial=True)
        return points

    def _configure_contacts(self):
        """Keep original robot filters; add a cable-only contact affinity bit.

        Kinematic triangle proxies preserve the actual finger holes. Their
        contact impulses are relayed to the corresponding dynamic robot links.
        The proxies collide only with the cable and have no visual geometry.
        """
        import xml.etree.ElementTree as ET
        from ..assets import origin_matrix, pose_values

        fingers = {n for n in self.env.agent.links if n.endswith(("leftfinger", "rightfinger"))}
        from .threading import TOUCH_LINKS
        mount_links = {USB_LINK, *TOUCH_LINKS}
        use_proxies = self.guide is not None
        actors = list(self.env.agent.links.values()) + list(self.env.fixtures.values())
        if self.plug not in actors:
            actors.append(self.plug)
        self.obstacles = actors.copy()
        for actor in actors:
            for shape in actor.get_collision_shapes():
                groups = shape.get_collision_groups()
                self.changed_shapes.append((shape, groups, shape.contact_offset))
                # Distinct affinities for following segments (2) and root (4).
                # Articulations overwrite g3, so ignore-group bits cannot
                # reliably exclude the root from a different articulation.
                shape.set_collision_groups(groups[0] if use_proxies and actor.name in fingers
                                           else groups[0] | (2 if actor.name in mount_links else 6), *groups[1:])
        if not use_proxies or not fingers:
            return
        from .kinematic import KinematicContactProxy, load_kinematic_target
        self._set_kinematic_target = load_kinematic_target()
        root = ET.parse(self.env.assets.urdf_path).getroot()
        for name in sorted(fingers):
            link = self.env.agent.links[name]
            articulation_builder = self.scene.create_articulation_builder()
            builder = articulation_builder.create_link_builder()
            builder.set_name("rope_contact_"+name)
            # Triangle meshes have no dynamic mass integral. This link is
            # kinematic; its placeholder inertia never drives its motion.
            builder.set_mass_and_inertia(1., sapien.Pose(), [1., 1., 1.])
            for collision in root.findall(f"link[@name='{name}']/collision"):
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    raise ValueError(f"Rope-Actor requires research finger meshes: {name}")
                builder.add_nonconvex_collision_from_file(mesh.get("filename"),
                    pose=sapien.Pose(*pose_values(origin_matrix(collision.find("origin")))),
                    scale=np.fromstring(mesh.get("scale", "1 1 1"), sep=" "), material=self.material)
            builder.set_collision_groups(2 if name.startswith("left_") else 6, 0, 0, 0)
            proxy = KinematicContactProxy(self.scene, articulation_builder)
            self.proxies.append((proxy, link))
            proxy.set_pose(link.pose)
            for shape in proxy.get_collision_shapes():
                # Native contact generation uses the same shell on capsules
                # and finger meshes. The MPM particle margin is independent.
                shape.contact_offset = self.config["rope_actor"]["contact_offset"]
            if not proxy.get_collision_shapes():
                raise RuntimeError(f"Failed to create finger collision proxy: {name}")
            self.obstacles.remove(link)
            self.obstacles.append(proxy)
        self.proxy_links = {p.id: link for p, link in self.proxies}

    def _build(self, points):
        r, c = self.config["rope_actor"], self.config["cable"]
        convex = r.get("collision_geometry", "capsule") == "convex_capsule"
        meshes = {}
        if convex:
            import tempfile
            from pathlib import Path
            from .capsule_mesh import write_capsule, validate_cooked_capsule
            self._collision_files = tempfile.TemporaryDirectory(prefix="rope_convex_")
        for i, length in enumerate(self.lengths):
            builder = self.scene.create_actor_builder()
            if convex:
                if length not in meshes:
                    path = Path(self._collision_files.name)/f"capsule_{len(meshes)}.obj"
                    write_capsule(path, length, self.radius)
                    meshes[length] = path
                builder.add_collision_from_file(str(meshes[length]), scale=[self.radius]*3, material=self.material)
            else:
                builder.add_capsule_collision(radius=self.radius, half_length=length/2, material=self.material)
            builder.add_capsule_visual(radius=self.radius, half_length=length/2, color=[1., .32, .03])
            mass = c["density"]*np.pi*self.radius**2*length
            inertia = np.array([mass*self.radius**2/2, mass*(3*self.radius**2+length**2)/12,
                                mass*(3*self.radius**2+length**2)/12])
            builder.set_mass_and_inertia(mass, sapien.Pose(), np.maximum(inertia, r["inertia_floor"]))
            builder.set_collision_groups(0, 4 if i == 0 else 2, 0, 0)
            actor = builder.build(f"rope_actor_{i}")
            self.links.append(actor)
            actor.set_damping(r["linear_damping"], r["angular_damping"])
            actor.set_solver_iterations(r["solver_iterations"], r["solver_velocity_iterations"])
            actor.set_ccd(False)
            for shape in actor.get_collision_shapes():
                shape.contact_offset = r["contact_offset"]
                if convex:
                    validate_cooked_capsule(shape, length, self.radius)
        if self._collision_files is not None:
            self._collision_files.cleanup()
            self._collision_files = None
        self._set_configuration(points)
        for i in range(1, len(self.links)):
            joint = self.scene.create_drive(self.links[i-1], sapien.Pose([self.lengths[i-1]/2, 0, 0]),
                                           self.links[i], sapien.Pose([-self.lengths[i]/2, 0, 0]))
            self.joints.append(joint)
            joint.lock_motion(True, True, True, False, False, False)
            twist, bend = np.deg2rad([r["twist_limit_deg"], r["bend_limit_deg"]])
            joint.set_x_twist_limit(-twist, twist)
            joint.set_yz_cone_limit(bend, bend)
            joint.set_slerp_properties(r["joint_stiffness"], r["joint_damping"], is_acceleration=False)
        self.link_ids = {link.id for link in self.links}
        self.link_indices = {link.id: i for i, link in enumerate(self.links)}
        self.total_mass = sum(link.mass for link in self.links)
        self._masses = np.array([link.mass for link in self.links])
        self._inertias = np.array([link.inertia for link in self.links])
        if not np.isclose(self.total_mass, c["density"]*np.pi*self.radius**2*c["length"], rtol=1.e-5):
            raise RuntimeError("PhysX did not preserve the configured rope mass")
        self._create_anchor()
        self._set_velocities()

    def _create_anchor(self):
        """Attach either the rigid first segment or its flexible start endpoint."""
        r = self.config["rope_actor"]
        self.root_joint = r.get("root_joint", "fixed")
        flexible = self.root_joint == "spherical"
        self.root_local_pose = sapien.Pose([-self.lengths[0]/2, 0., 0.]) if flexible else sapien.Pose()
        self.anchor_pose = self.plug.pose.inv()*self.links[0].pose*self.root_local_pose
        self.anchor = self.scene.create_drive(self.plug, self.anchor_pose, self.links[0], self.root_local_pose)
        self.anchor.lock_motion(True, True, True, not flexible, not flexible, not flexible)
        if flexible:
            twist, bend = np.deg2rad([r["twist_limit_deg"], r["bend_limit_deg"]])
            self.anchor.set_x_twist_limit(-twist, twist)
            self.anchor.set_yz_cone_limit(bend, bend)
            self.anchor.set_slerp_properties(r["joint_stiffness"], r["joint_damping"], is_acceleration=False)

    def attachment_error(self):
        return float(np.linalg.norm((self.links[0].pose*self.root_local_pose).p-
                                    (self.plug.pose*self.anchor_pose).p))

    def _set_configuration(self, points):
        frames = segment_frames(points)
        for i, actor in enumerate(self.links):
            actor.set_pose(sapien.Pose((points[i]+points[i+1])/2, mat2quat(frames[i])))

    def _set_velocities(self):
        for actor in self.links:
            actor.set_velocity(self.plug.velocity + np.cross(self.plug.angular_velocity, actor.pose.p-self.plug.pose.p))
            actor.set_angular_velocity(self.plug.angular_velocity)

    def _endpoints(self):
        poses = [link.pose for link in self.links]
        centers = np.array([pose.p for pose in poses])
        w, x, y, z = np.array([pose.q for pose in poses], dtype=float).T
        axes = np.column_stack((1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)))
        self._centers, self._axes = centers, axes
        delta = axes*self.lengths[:, None]/2
        return centers-delta, centers+delta

    @property
    def centerline(self):
        starts, ends = self._endpoints()
        return np.vstack((starts[0], (ends[:-1]+starts[1:])/2, ends[-1]))

    def _guide_crossing(self, points, *, initial=False):
        pose = self.guide.pose
        axis = quat2mat(pose.q)[:, 0]
        along = (points-pose.p) @ axis
        indices = np.flatnonzero(along[:-1]*along[1:] <= 0.)
        denominator = along[indices+1]-along[indices]
        valid = np.abs(denominator) > 1.e-10
        indices, denominator = indices[valid], denominator[valid]
        coordinates = indices-along[indices]/denominator
        if not len(indices):
            raise RuntimeError("Cable no longer crosses the right TCP guide; stop and reset")
        if initial:
            distances = np.linalg.norm(points[indices]+(coordinates-indices)[:, None]*
                                       (points[indices+1]-points[indices])-pose.p, axis=1)
            chosen = np.argmin(distances)
        else:
            chosen = np.argmin(np.abs(coordinates-self.guide.material_coordinate))
            if abs(coordinates[chosen]-self.guide.material_coordinate) > 2.:
                raise RuntimeError("Cable lost its continuous right TCP threading")
        index, coordinate = int(indices[chosen]), float(coordinates[chosen])
        if coordinate <= 1 or coordinate >= len(points)-2:
            raise RuntimeError("USB fixed end or cable free end reached the right TCP guide")
        crossing = points[index]+(coordinate-index)*(points[index+1]-points[index])
        return index, coordinate, float(np.linalg.norm(crossing-pose.p)), axis

    def _update_guide(self):
        """Track material passing the hole without applying a guide constraint."""
        if self.guide is None:
            return
        index, coordinate, error, axis = self._guide_crossing(self.centerline, initial=self.guide_index is None)
        self.guide.material_coordinate, self.guide.radial_error = coordinate, error
        self.guide_index = index

    def guide_diagnostics(self):
        """Measure material flow and native hole loads without constraining it."""
        if self.guide is None:
            return {}
        coordinate = self.guide.material_coordinate
        index = self.guide_index
        arc = float(np.interp(coordinate, np.arange(self.sections), self.material_s))
        points = self.centerline
        crossing = points[index]+(coordinate-index)*(points[index+1]-points[index])
        link = self.links[index]
        tcp = self.guide.link
        axis = quat2mat(tcp.pose.q)[:, 0]
        cable_velocity = link.velocity + np.cross(link.angular_velocity, crossing-link.pose.p)
        tcp_com = (tcp.pose*tcp.cmass_local_pose).p
        tcp_velocity = tcp.velocity + np.cross(tcp.angular_velocity, crossing-tcp_com)
        span = float(np.linalg.norm(crossing-points[0]))
        return dict(guide_arc_length_m=arc, guide_free_tail_length_m=float(self.material_s[-1]-arc),
                    guide_span_distance_m=span, guide_span_excess_length_m=arc-span,
                    guide_axial_relative_speed_m_s=float(np.dot(cable_velocity-tcp_velocity, axis)),
                    guide_segment_angle_deg=float(np.rad2deg(np.arccos(np.clip(
                        abs(np.dot(quat2mat(link.pose.q)[:, 0], axis)), 0., 1.)))),
                    guide_normal_load_N=float(self._guide_normal_load_impulse/self._dt),
                    guide_normal_axial_force_N=float(np.dot(self._guide_normal_impulse, axis)/self._dt),
                    # PhysX 4 contact reports reconstruct impulse = normal *
                    # scalar impulse. Tangential solver impulses are absent.
                    guide_friction_axial_force_N=None,
                    guide_force_report_scope="normal_impulses_only")

    def step(self, rigid_dt):
        from .kinematic import advance_proxy

        self._dt = rigid_dt
        self._energy_before_step = self._kinetic_energy()
        self.max_depth = 0.
        self.max_penetration_contact = None
        self.contacted_bodies.clear()
        # These impulses belong to the previous contact solve. Divide by the
        # step that will actually integrate the force, not the previous dt.
        for link, impulse, point in self._pending_reactions:
            link.add_force_at_point(impulse/rigid_dt, point)
        self._pending_reactions.clear()
        self._proxy_targets = [(proxy, advance_proxy(proxy, link, rigid_dt, self._set_kinematic_target))
                               for proxy, link in self.proxies]
        self._update_guide()

    def _kinetic_energy(self):
        if not self.links:
            return 0.
        # Capsule inertia is axisymmetric, including the configured floor.
        self._endpoints()
        v = np.array([link.velocity for link in self.links])
        w = np.array([link.angular_velocity for link in self.links])
        axial = np.sum(w*self._axes, axis=1)
        return float(.5*np.sum(self._masses*np.sum(v*v, axis=1)
            + self._inertias[:, 1]*np.sum(w*w, axis=1)
            + (self._inertias[:, 0]-self._inertias[:, 1])*axial*axial))

    def _endpoint_speeds(self, linear, angular):
        rotation = np.cross(angular, self._axes)*self.lengths[:, None]/2
        return np.maximum(np.linalg.norm(linear+rotation, axis=1),
                          np.linalg.norm(linear-rotation, axis=1))

    def _speed_failure(self, index, speed):
        return RuntimeError(
            f"Rope-Actor segment {index} endpoint speed {speed:.3f} m/s exceeded max_speed; "
            f"dt={self._dt:.6g} s, kinetic energy "
            f"{self._energy_before_step:.6g}->{self._kinetic_energy():.6g} J, "
            f"largest contact impulse={self._last_contact_impulse:.6g} N s "
            f"({self._last_contact_pair}). Physics state is invalid; reset is required."
        )

    def suggested_timestep(self, maximum):
        starts, ends = self._endpoints()
        linear = np.array([link.velocity for link in self.links])
        angular = np.array([link.angular_velocity for link in self.links])
        speeds = self._endpoint_speeds(linear, angular)
        # Circular capsules are invariant to axial spin. Include translation
        # of the robot's contact surfaces in the relative-motion bound.
        rigid_speed = max((np.linalg.norm(link.velocity)+.15*np.linalg.norm(link.angular_velocity)
                           for link in self.env.agent.links.values()), default=0.)
        if not np.isfinite(rigid_speed):
            raise RuntimeError("Rope-Actor robot contact speed is non-finite; reset is required")
        if not np.isfinite(speeds).all() or speeds.max() > self.config["rope_actor"]["max_speed"]:
            i = int(np.argmax(speeds))
            raise self._speed_failure(i, speeds[i])
        # Resolve free-chain motion to 1/10 segment length, then smoothly
        # tighten the budget as a segment approaches any collider's AABB.
        # A binary near/far switch caused a 1 ms -> ~25 us jump in the drop
        # regression. At gaps <= 2*travel the original contact budget applies.
        bounds = self._world_obstacle_bounds()
        if len(bounds):
            low, high = np.minimum(starts, ends)-self.radius, np.maximum(starts, ends)+self.radius
            separation = np.maximum(np.maximum(bounds[None, :, 0, :]-high[:, None, :],
                low[:, None, :]-bounds[None, :, 1, :]), 0.)
            gap = np.linalg.norm(separation, axis=2).min(axis=1)
        else:
            gap = np.full(len(self.links), np.inf)
        travel = self.config["rope_actor"]["max_contact_travel"]
        budgets = np.minimum(.1*self.lengths, travel+.25*np.maximum(gap-2*travel, 0.))
        relative_speeds = speeds+rigid_speed+9.81*maximum+.001
        near = gap <= 2*travel
        self._contact_speed_bound = float(relative_speeds[near].max()) if np.any(near) else None
        self._motion_budget_rate = float(np.max(relative_speeds/budgets))
        self._step_limit = min(maximum, 1/self._motion_budget_rate)
        self._step_limit_for_step = self.steps+1
        return self._step_limit

    def _prepare_obstacle_bounds(self):
        self._obstacle_boxes = []
        for actor in self.obstacles:
            vertices = []
            for shape in actor.get_collision_shapes():
                g = shape.geometry
                if isinstance(g, sapien.BoxGeometry):
                    extent = np.asarray(g.half_lengths)
                elif isinstance(g, sapien.CapsuleGeometry):
                    extent = np.array([g.half_length+g.radius, g.radius, g.radius])
                else:
                    extent = None
                if extent is not None:
                    corners = np.array(np.meshgrid(*[[-v, v] for v in extent])).reshape(3, -1).T
                else:
                    corners = np.asarray(g.vertices)*np.asarray(g.scale)
                    if hasattr(g, "rotation"):
                        corners = corners @ quat2mat(g.rotation).T
                pose = shape.get_local_pose()
                vertices.append(corners @ quat2mat(pose.q).T+pose.p)
            if vertices:
                vertices = np.vstack(vertices)
                self._obstacle_boxes.append((actor, (vertices.max(0)+vertices.min(0))/2,
                                              (vertices.max(0)-vertices.min(0))/2))

    def _world_obstacle_bounds(self):
        bounds = []
        for actor, local, half in self._obstacle_boxes:
            pose = actor.pose
            rotation = quat2mat(pose.q)
            center, extent = pose.p+rotation @ local, np.abs(rotation) @ half
            bounds.append([center-extent, center+extent])
        return np.asarray(bounds).reshape(-1, 2, 3)

    def follow_plug(self):
        self.steps += 1
        self._collect_contacts()
        self._check_proxy_targets()
        self._update_guide()

    def _check_proxy_targets(self):
        for proxy, target in self._proxy_targets:
            pose = proxy.pose
            p, q = np.asarray(pose.p), np.asarray(pose.q)
            norm = np.linalg.norm(q)
            if (not np.isfinite(np.r_[p, q]).all() or abs(norm-1.) > 1.e-4
                    or np.linalg.norm(p-target.p) > 1.e-5
                    or abs(np.dot(q, target.q)) < .99999):
                raise RuntimeError(f"Rope-Actor kinematic proxy {proxy.name} did not reach its motion target; "
                                   "invalid contact boundary, reset is required")

    def _collect_contacts(self):
        proxy_links = getattr(self, "proxy_links", {})
        candidates = {}
        self._last_contact_impulse = 0.
        self._last_contact_pair = None
        self._guide_normal_impulse = np.zeros(3)
        self._guide_normal_load_impulse = 0.
        for contact in self.scene.get_contacts():
            a, b = contact.actor0, contact.actor1
            if a.id not in self.link_ids and b.id not in self.link_ids:
                continue
            other = b if a.id in self.link_ids else a
            self.contacted_bodies.add(proxy_links.get(other.id, other).name)
            cable_link = a if a.id in self.link_ids else b
            shape = contact.collision_shape1 if a.id in self.link_ids else contact.collision_shape0
            candidates.setdefault((other, shape), set()).add(cable_link)
            for point in contact.points:
                magnitude = float(np.linalg.norm(point.impulse))
                if magnitude > self._last_contact_impulse:
                    self._last_contact_impulse = magnitude
                    self._last_contact_pair = [a.name, b.name]
                if other.id in proxy_links:
                    if proxy_links[other.id].name.startswith("right_"):
                        on_cable = np.asarray(point.impulse)*(1. if a.id in self.link_ids else -1.)
                        normal = np.asarray(point.normal)
                        normal_impulse = np.dot(on_cable, normal)*normal
                        self._guide_normal_impulse += normal_impulse
                        self._guide_normal_load_impulse += abs(float(np.dot(on_cable, normal)))
                    # PhysX reports the normal impulse applied to actor0. Relay
                    # that available reaction to the robot on the next step;
                    # tangential friction impulses are not exposed by this API.
                    impulse = point.impulse if other.id == a.id else -point.impulse
                    self._pending_reactions.append((proxy_links[other.id],
                                                    np.array(impulse), np.array(point.position)))
        self._contact_candidates = candidates

    def _audit_contacts(self):
        # PhysX contact.separation describes the contact generated BEFORE its
        # constraint solve. Audit actual capsule poses AFTER the step instead.
        # This read-only geometric audit runs at the control boundary; native
        # finite capsule contacts run at every adaptive PhysX substep.
        self.max_depth = 0.
        self.max_penetration_contact = None
        proxy_links = getattr(self, "proxy_links", {})
        deepest = None
        candidates = {}
        for (other, shape), actors in self._contact_candidates.items():
            candidates.setdefault((proxy_links.get(other.id, other), shape), set()).update(actors)
        # Audit the whole nearby cable against the actual fingers even if the
        # last native substep reported no contact. Otherwise tunnelling or a
        # missed contact could silently defeat the passive aperture observer.
        if self.aperture is not None:
            starts, ends = self._endpoints()
            for (actor, shape), (_, local_pose, mesh) in zip(self._guide_sources, self.aperture.sources):
                pose = actor.pose*local_pose
                local_start = (starts-pose.p) @ quat2mat(pose.q)
                local_end = (ends-pose.p) @ quat2mat(pose.q)
                near = np.all((np.minimum(local_start, local_end)-self.radius <= mesh.bounds[1]) &
                              (np.maximum(local_start, local_end)+self.radius >= mesh.bounds[0]), axis=1)
                if np.any(near):
                    candidates.setdefault((actor, shape), set()).update(
                        self.links[i] for i in np.flatnonzero(near))
        for (other, shape), actors in candidates.items():
            points, indices = [], []
            for actor in sorted(actors, key=lambda actor: self.link_indices[actor.id]):
                index = self.link_indices[actor.id]
                length = self.lengths[index]
                along = np.linspace(-length/2, length/2, int(np.ceil(length/(self.radius/2)))+1)
                points.extend(actor.pose.p+along[:, None]*quat2mat(actor.pose.q)[:, 0])
                indices.extend([index]*len(along))
            pose = proxy_links.get(other.id, other).pose*shape.get_local_pose()
            local = (np.asarray(points)-pose.p) @ quat2mat(pose.q)
            depth = self._shape_signed_distance(shape, local)+self.radius
            sample = int(np.argmax(depth))
            if depth[sample] > self.max_depth:
                self.max_depth = float(depth[sample])
                deepest = (other, shape, indices[sample], points[sample], self.max_depth)
        if deepest is not None:
            self.max_penetration_contact = self._penetration_detail(*deepest)

    def _penetration_detail(self, other, shape, index, point, depth):
        """Describe a post-solve geometric overlap, not a native impulse point."""
        body = getattr(self, "proxy_links", {}).get(other.id, other)
        pose = body.pose*shape.get_local_pose()
        local = (np.asarray(point)-pose.p) @ quat2mat(pose.q)
        geometry = shape.geometry
        triangle = None
        if isinstance(geometry, sapien.BoxGeometry):
            half = np.asarray(geometry.half_lengths)
            closest = np.clip(local, -half, half)
            if np.all(np.abs(local) <= half):
                axis = int(np.argmin(half-np.abs(local)))
                closest[axis] = half[axis] if local[axis] >= 0 else -half[axis]
        elif isinstance(geometry, sapien.CapsuleGeometry):
            axis_point = np.array([np.clip(local[0], -geometry.half_length, geometry.half_length), 0., 0.])
            delta = local-axis_point
            distance = np.linalg.norm(delta)
            normal = delta/distance if distance > 1.e-12 else np.array([0., 1., 0.])
            closest = axis_point+geometry.radius*normal
        else:
            import trimesh
            # Initial-layout failures can reach here before this mesh is cached.
            if shape not in self._contact_meshes:
                self._shape_signed_distance(shape, local[None, :])
            closest, _, faces = trimesh.proximity.closest_point(self._contact_meshes[shape], local[None, :])
            closest, triangle = closest[0], int(faces[0])
        surface = pose.p+quat2mat(pose.q) @ closest
        return dict(object=body.name, segment_index=int(index), segment_name=f"rope_actor_{index}",
                    segment_index_base=0, segment_length_m=float(self.lengths[index]),
                    frame_id="world", depth_m=float(depth),
                    centerline_world_m=np.asarray(point, dtype=float).tolist(),
                    obstacle_surface_world_m=np.asarray(surface, dtype=float).tolist(),
                    collision_triangle_index=triangle)

    @staticmethod
    def _penetration_context(detail):
        center = ", ".join(f"{v:.6f}" for v in detail["centerline_world_m"])
        surface = ", ".join(f"{v:.6f}" for v in detail["obstacle_surface_world_m"])
        return (f"object={detail['object']}, segment_index={detail['segment_index']} (0-based), "
                f"segment={detail['segment_name']}, segment_length={detail['segment_length_m']*1000:.3f} mm, "
                f"frame=world, centerline=({center}) m, obstacle_surface=({surface}) m")

    def _shape_signed_distance(self, shape, local):
        """Positive inside; points beyond the radius-expanded mesh AABB are clear."""
        import trimesh
        geometry = shape.geometry
        if isinstance(geometry, sapien.BoxGeometry):
            d = np.abs(local)-geometry.half_lengths
            return -(np.linalg.norm(np.maximum(d, 0.), axis=1)+np.minimum(np.max(d, axis=1), 0.))
        if isinstance(geometry, sapien.CapsuleGeometry):
            delta = local.copy()
            delta[:, 0] -= np.clip(delta[:, 0], -geometry.half_length, geometry.half_length)
            return geometry.radius-np.linalg.norm(delta, axis=1)
        if shape not in self._contact_meshes:
            if not isinstance(geometry, (sapien.ConvexMeshGeometry, sapien.NonconvexMeshGeometry)):
                raise ValueError(f"Unsupported rope contact geometry: {type(geometry).__name__}")
            vertices = np.asarray(geometry.vertices)*np.asarray(geometry.scale)
            if hasattr(geometry, "rotation"):
                vertices = vertices @ quat2mat(geometry.rotation).T
            self._contact_meshes[shape] = trimesh.Trimesh(vertices=vertices,
                faces=np.asarray(geometry.indices).reshape(-1, 3), process=False)
        mesh = self._contact_meshes[shape]
        # Whole capsules may extend well past a finger. Those points cannot
        # penetrate it, and need no expensive mesh containment ray casts.
        near = np.all((local >= mesh.bounds[0]-self.radius) &
                      (local <= mesh.bounds[1]+self.radius), axis=1)
        distance = np.full(len(local), -np.inf)
        if np.any(near):
            distance[near] = trimesh.proximity.signed_distance(mesh, local[near])
        return distance

    def check_contacts(self):
        self._audit_contacts()
        if self.max_depth > self.config["cable"]["penetration_tolerance"]:
            raise RuntimeError(f"Rope-Actor rigid contact penetration {self.max_depth*1000:.3f} mm exceeds tolerance; "
                               +self._penetration_context(self.max_penetration_contact))
        self._check_constraints()

    def _check_constraints(self):
        starts, ends = self._endpoints()
        if not np.isfinite(starts).all() or not np.isfinite(ends).all():
            raise RuntimeError("Non-finite Rope-Actor state")
        tolerance = self.config["rope_actor"]["constraint_tolerance"]
        gaps = np.linalg.norm(ends[:-1]-starts[1:], axis=1)
        if gaps.max() > tolerance or self.attachment_error() > tolerance:
            raise RuntimeError("Rope-Actor joint/attachment constraint exceeded tolerance")
        if self.aperture is not None:
            self._check_aperture(self.centerline)
        linear = np.array([link.velocity for link in self.links])
        angular = np.array([link.angular_velocity for link in self.links])
        speeds = self._endpoint_speeds(linear, angular)
        if not np.isfinite(speeds).all() or speeds.max() > self.config["rope_actor"]["max_speed"]:
            i = int(np.argmax(speeds))
            raise self._speed_failure(i, speeds[i])

    def _check_aperture(self, points):
        index, coordinate, _, _ = self._guide_crossing(points, initial=self.guide_index is None)
        crossing = points[index]+(coordinate-index)*(points[index+1]-points[index])
        self.aperture.check(crossing)

    def _audit_layout(self, points):
        """Reject overlapping spawn/reset geometry before modifying live state.

        Dense centerline spheres conservatively cover each whole capsule. Native
        capsule contacts handle segment interiors during adaptive rigid steps.
        """
        import trimesh
        if self.aperture is not None:
            self._check_aperture(points)
        samples, ids = [], []
        spacing = self.radius/2
        for i, (a, b, length) in enumerate(zip(points[:-1], points[1:], self.lengths)):
            row = np.linspace(a, b, int(np.ceil(length/spacing))+1)
            samples.extend(row)
            ids.extend([i]*len(row))
        samples, ids = np.asarray(samples), np.asarray(ids)
        radius = np.sqrt(self.radius**2+(spacing/2)**2)
        from .threading import TOUCH_LINKS
        excluded = {USB_LINK, *TOUCH_LINKS, "rope_contact_left_fr3_leftfinger", "rope_contact_left_fr3_rightfinger"}
        for actor in self.obstacles:
            for shape in actor.get_collision_shapes():
                pose = actor.pose*shape.get_local_pose()
                local = (samples-pose.p) @ quat2mat(pose.q)
                geometry = shape.geometry
                if isinstance(geometry, sapien.BoxGeometry):
                    mesh = trimesh.creation.box(extents=2*np.asarray(geometry.half_lengths))
                elif isinstance(geometry, sapien.CapsuleGeometry):
                    # Analytic signed distance, positive inside.
                    delta = local.copy()
                    delta[:, 0] -= np.clip(delta[:, 0], -geometry.half_length, geometry.half_length)
                    depth = geometry.radius-np.linalg.norm(delta, axis=1)+radius
                    if actor.name in excluded:
                        depth[ids == 0] = -np.inf
                    if np.max(depth) > self.config["cable"]["penetration_tolerance"]:
                        j = int(np.argmax(depth))
                        detail = self._penetration_detail(actor, shape, ids[j], samples[j], depth[j])
                        raise ValueError(f"Initial rope intersects {detail['object']} by {depth[j]*1000:.3f} mm; "
                                         +self._penetration_context(detail))
                    continue
                elif isinstance(geometry, (sapien.ConvexMeshGeometry, sapien.NonconvexMeshGeometry)):
                    vertices = np.asarray(geometry.vertices)*np.asarray(geometry.scale)
                    if hasattr(geometry, "rotation"):
                        vertices = vertices @ quat2mat(geometry.rotation).T
                    mesh = trimesh.Trimesh(vertices=vertices,
                                          faces=np.asarray(geometry.indices).reshape(-1, 3), process=False)
                else:
                    raise ValueError(f"Unsupported rope contact geometry: {type(geometry).__name__}")
                nearby = np.all((local >= mesh.bounds[0]-radius) & (local <= mesh.bounds[1]+radius), axis=1)
                if actor.name in excluded:
                    nearby &= ids != 0
                if np.any(nearby):
                    depth = trimesh.proximity.signed_distance(mesh, local[nearby])+radius
                    if depth.max() > self.config["cable"]["penetration_tolerance"]:
                        j = int(np.argmax(depth))
                        sample = np.flatnonzero(nearby)[j]
                        detail = self._penetration_detail(actor, shape, ids[sample], samples[sample], depth[j])
                        raise ValueError(f"Initial rope intersects {detail['object']} by {depth[j]*1000:.3f} mm; "
                                         +self._penetration_context(detail))

    def reset(self):
        points = self._initial_points()
        for proxy, link in self.proxies:
            proxy.set_pose(link.pose)
        self._audit_layout(points)
        self.guide_index = None
        self._set_configuration(points)
        # The root's transported twist can differ after moving the USB.
        self.scene.remove_drive(self.anchor)
        self._create_anchor()
        self._set_velocities()
        self._update_guide()
        self.max_depth = 0.
        self.max_penetration_contact = None
        self.contacted_bodies.clear()
        self._contact_candidates.clear()
        self._pending_reactions.clear()
        self._energy_before_step = self._kinetic_energy()
        self._proxy_targets.clear()
        self._last_contact_impulse = 0.
        self._last_contact_pair = None
        self._contact_speed_bound = self._step_limit = None
        self._guide_normal_impulse = np.zeros(3)
        self._guide_normal_load_impulse = 0.
        self._step_limit_for_step = None
        self._motion_budget_rate = None

    def diagnostics(self):
        center = self.centerline
        starts, ends = self._endpoints()
        coordinate = self.guide.material_coordinate if self.guide else None
        return dict(solver=self.solver, physx_solver_type=self.config["rope_actor"].get("solver_type", "tgs"),
                    root_joint=self.root_joint,
                    collision_geometry=self.config["rope_actor"].get("collision_geometry", "capsule"),
                    collision_surface_deficit_bound_m=(.028*self.radius if
                        self.config["rope_actor"].get("collision_geometry") == "convex_capsule" else 0.),
                    segments=len(self.links), rest_length_m=float(self.material_s[-1]),
                    min_segment_length_m=float(self.lengths.min()), max_segment_length_m=float(self.lengths.max()),
                    mass_kg=float(self.total_mass),
                    inertia_floor_kg_m2=self.config["rope_actor"]["inertia_floor"],
                    max_contact_travel_m=self.config["rope_actor"]["max_contact_travel"],
                    penetration_tolerance_m=self.config["cable"]["penetration_tolerance"],
                    native_contact_offset_m=self.config["rope_actor"]["contact_offset"],
                    last_rigid_timestep_s=self._dt,
                    max_linear_speed_m_s=float(max(np.linalg.norm(link.velocity) for link in self.links)),
                    max_angular_speed_rad_s=float(max(np.linalg.norm(link.angular_velocity) for link in self.links)),
                    max_endpoint_speed_m_s=float(self._endpoint_speeds(
                        np.array([link.velocity for link in self.links]),
                        np.array([link.angular_velocity for link in self.links])).max()),
                    kinetic_energy_J=self._kinetic_energy(),
                    kinetic_energy_before_step_J=self._energy_before_step,
                    last_contact_impulse_Ns=self._last_contact_impulse,
                    last_contact_pair=self._last_contact_pair,
                    diameter_m=self.radius*2, current_centerline_length_m=float(np.linalg.norm(np.diff(center, axis=0), axis=1).sum()),
                    max_section_gap_m=float(np.linalg.norm(np.diff(center, axis=0), axis=1).max()),
                    max_joint_gap_m=float(np.linalg.norm(ends[:-1]-starts[1:], axis=1).max()),
                    max_joint_gap_index=int(np.argmax(np.linalg.norm(ends[:-1]-starts[1:], axis=1))),
                    attachment_error_m=self.attachment_error(),
                    guide_material_coordinate=coordinate, guide_material_coordinate_unit="section_index",
                    guide_arc_length_m=float(np.interp(coordinate, np.arange(self.sections), self.material_s)) if self.guide else None,
                    guide_radial_error_m=self.guide.radial_error if self.guide else None,
                    guide_model="mesh_contact_only" if self.guide else None,
                    guide_inside_aperture=self.aperture.inside if self.aperture else None,
                    guide_plane_wall_distance_m=self.aperture.wall_distance if self.aperture else None,
                    max_rigid_penetration_m=self.max_depth, max_penetration_contact=self.max_penetration_contact,
                    contact_margin_m=self.config["cable"]["contact_margin"],
                    contact_model=("physx_convex_capsules_adaptive_steps_triangle_finger_proxies" if
                        self.config["rope_actor"].get("collision_geometry") == "convex_capsule" else
                        "physx_capsules_adaptive_steps_triangle_finger_proxies"),
                    contacted_bodies=sorted(self.contacted_bodies),
                    rigid_steps=self.steps, self_collision=False,
                    guide_measurements=self.guide_diagnostics())

    def update_render(self):
        pass  # The measured PhysX actors are also the render geometry.

    def close(self):
        if self._collision_files is not None:
            self._collision_files.cleanup()
            self._collision_files = None
        for drive in [self.anchor, *self.joints]:
            if drive is not None:
                self.scene.remove_drive(drive)
        self.anchor = self.aperture = None
        self.joints.clear()
        for actor in self.links:
            self.scene.remove_actor(actor)
        for actor, _ in self.proxies:
            actor.close()
        self.links.clear()
        self.proxies.clear()
        for shape, groups, offset in self.changed_shapes:
            shape.set_collision_groups(*groups)
            shape.contact_offset = offset
        self.changed_shapes.clear()
        self._contact_candidates.clear()
        self._contact_meshes.clear()
        self._pending_reactions.clear()
        self._proxy_targets.clear()
        self.max_penetration_contact = None
