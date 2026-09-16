"""A cable using ManiSkill2's original CUDA MPM integrator and SAPIEN contacts.

Adds attachment boundaries, axial fibers, finite-radius contact and bounded
grid allocation. This is a volumetric MPM material, not a chain of rigid links.
"""
import ctypes
import math
from pathlib import Path
import shlex
import sys

from ..sapien_compat import sapien

import numpy as np
from transforms3d.quaternions import quat2mat

# Sets up the matching, bundled Warp module path.
from mani_skill2.envs.mpm.base_env import MPMModelBuilder
from warp.sim.model import Mesh
import warp as wp

from .model import USB_LINK, cable_particles, grid_layout, initial_particle_positions
from .solver import CableMPMSimulator
from .fibers import AxialFibers
from .execution import DeviceBounds, ConstraintGraphs
from .contacts import CableContacts
from mpm.mpm_model import MPMModelStruct, MPMStateStruct
from .mesh_contacts import finger_collision_meshes


@wp.kernel
def anchor_grid_kernel(model: MPMModelStruct, state: MPMStateStruct,
    pose: wp.transform, attachment: wp.vec3, length: float, radius: float,
    com: wp.vec3, linear: wp.vec3, angular: wp.vec3,
    impulse: wp.array(dtype=wp.spatial_vector)):
    ix, iy, iz = wp.tid()
    mass = state.grid_m[ix, iy, iz]
    if mass > 0.0:
        point = wp.vec3(float(ix + state.grid_lower[0]), float(iy + state.grid_lower[1]),
                        float(iz + state.grid_lower[2])) * model.dx
        local = wp.transform_point(wp.transform_inverse(pose), point) - attachment
        # Dirichlet velocity condition on the small USB strain-relief volume.
        # Include the interpolation support across the thin cross section.
        if local[1] <= model.dx * 0.5 and local[1] >= -length:
            if local[0]*local[0] + local[2]*local[2] <= radius*radius:
                velocity = linear + wp.cross(angular, point - com)
                delta = (state.grid_v[ix, iy, iz] - velocity) * mass
                wp.atomic_add(impulse, 0, wp.spatial_vector(wp.cross(point-com, delta), delta))
                state.grid_v[ix, iy, iz] = velocity


@wp.kernel
def render_centers(positions: wp.array(dtype=wp.vec3), centers: wp.array(dtype=wp.vec3)):
    i = wp.tid()
    center = wp.vec3(0.0)
    for j in range(7):
        center = center + positions[i * 7 + j] / 7.0
    centers[i] = center


@wp.kernel
def pin_particles(
    ids: wp.array(dtype=int), targets: wp.array(dtype=wp.vec3),
    velocities: wp.array(dtype=wp.vec3), positions: wp.array(dtype=wp.vec3),
    speeds: wp.array(dtype=wp.vec3), deformation: wp.array(dtype=wp.mat33),
    affine: wp.array(dtype=wp.mat33), mass: wp.array(dtype=float),
    rotation: wp.mat33, spin: wp.mat33, com: wp.vec3,
    impulse: wp.array(dtype=wp.spatial_vector), reaction: int,
):
    i = wp.tid()
    p = ids[i]
    if reaction != 0:
        f = (speeds[p] - velocities[i]) * mass[p]
        torque = wp.cross(targets[i] - com, f)
        wp.atomic_add(impulse, 0, wp.spatial_vector(torque, f))
    positions[p] = targets[i]
    speeds[p] = velocities[i]
    deformation[p] = rotation
    affine[p] = spin


@wp.kernel
def temporary_world_support(ids: wp.array(dtype=int), targets: wp.array(dtype=wp.vec3),
                            positions: wp.array(dtype=wp.vec3), velocities: wp.array(dtype=wp.vec3)):
    i = wp.tid()
    positions[ids[i]] = targets[i]
    velocities[ids[i]] = wp.vec3(0.0)


def pose_transform(pose):
    return wp.transform(tuple(pose.p), tuple(pose.q[[1, 2, 3, 0]]))


def initialize_warp():
    repair = f"{shlex.quote(sys.executable)} -m dual_fr3_maniskill.warp_setup --force"
    library = Path(wp.__file__).resolve().parent / "bin/warp.so"
    if not library.is_file():
        raise RuntimeError(f"ManiSkill2 Warp library is missing: {library}. Build it with: {repair}")
    try:
        wp.init()
    except OSError as exc:
        raise RuntimeError(f"Cannot load ManiSkill2 Warp: {exc}. Rebuild with: {repair}") from exc
    if not wp.is_cuda_available():
        raise RuntimeError("ManiSkill2 Warp could not initialize CUDA. Check NVIDIA driver access "
                           f"and CUDA_VISIBLE_DEVICES; if the library was built elsewhere, run: {repair}")


class MPMCable:
    solver = "mpm"

    @property
    def max_section_gap(self):
        return self.config["cable"]["particle_spacing"]*2

    def close(self):
        self.remove_temporary_supports()
        self.constraint_graphs.clear()
        if hasattr(self, "pcd"):
            self.env._scene.remove_particle_entity(self.pcd)
            del self.pcd

    def add_temporary_supports(self, intervals):
        """World positioning on short bore neighborhoods only, until release."""
        # Retire graphs BEFORE replacing arrays whose raw addresses were
        # captured. Reconfiguring equal-sized intervals must be safe as well.
        self.constraint_graphs.clear()
        s = np.linspace(0., self.config["cable"]["length"], self.sections)
        selected = np.zeros(self.sections, dtype=bool)
        for lower, upper in intervals:
            selected |= (s >= lower) & (s <= upper)
        ids = np.flatnonzero(np.repeat(selected, 7)).astype(np.int32)
        self.support_ids = wp.array(ids, dtype=int, device=self.device)
        self.support_targets = wp.array(self.positions[ids].astype(np.float32), dtype=wp.vec3, device=self.device)
        self.support_count = len(ids)

    def remove_temporary_supports(self):
        # Do not alter state, pose or velocity at release. Invalidate graphs
        # containing support kernels before dropping their backing buffers.
        self.constraint_graphs.clear()
        self.support_ids = self.support_targets = None
        self.support_count = 0

    def _apply_temporary_supports(self):
        if self.support_count:
            state = self.states[0].struct
            wp.launch(temporary_world_support, dim=self.support_count,
                inputs=[self.support_ids, self.support_targets, state.particle_q, state.particle_qd],
                device=self.device)

    def __init__(self, env, config, *, plug=None, layout=None, guide=None):
        self.env, self.config = env, config
        self.plug = plug if plug is not None else env.agent.links[USB_LINK]
        self.layout = layout or initial_particle_positions
        self.guide = guide
        self.support_count = 0
        self.support_ids = self.support_targets = None
        initialize_warp()
        self.device = "cuda"
        self.local, volumes, pinned, self.sections = cable_particles(config)
        self.initial_rotation = quat2mat(self.plug.pose.q)
        world = self.layout(config, self.local, self.initial_rotation, self.plug.pose.p)
        self.spacing, self.grid_dims = grid_layout(world, config)
        self.grid_changes = 0
        self.builder = MPMModelBuilder()
        self.builder.set_mpm_domain((np.array(self.grid_dims) + .1) * self.spacing, self.spacing)
        c = config["cable"]
        mu = c["young_modulus"] / (2 * (1 + c["poisson_ratio"]))
        lam = c["young_modulus"] * c["poisson_ratio"] / ((1 + c["poisson_ratio"]) * (1 - 2 * c["poisson_ratio"]))
        wave_speed = math.sqrt((lam + 2 * mu) / c["density"])
        if wave_speed / (config["mpm"]["frequency"] * self.spacing) > 0.65:
            raise ValueError("MPM time step is too large for the material stiffness: increase mpm.frequency")
        for p, v in zip(world, volumes):
            self.builder.add_mpm_particle(tuple(p), (0., 0., 0.), mass=v * c["density"],
                volume=v, type=0, material=(mu, lam, c["yield_stress"]),
                material2=(0., 0., 0.), color=(1., .32, .03))
        self.actors, self.meshes = [], []
        self._build_contacts()
        self.model = self.builder.finalize(self.device)
        self.model.gravity = np.array((0., 0., -9.81), dtype=np.float32)
        self.model.adaptive_grid = True
        self.model.grid_contact = False
        # A zero normal leaves velocity unchanged in the upstream ground
        # projection; the scene's actual table/plate/slot provide the contacts.
        self.model.struct.ground_normal = wp.vec3(0., 0., 0.)
        self.model.struct.ground_sticky = 0
        self.model.struct.body_sticky = 0
        self.model.struct.body_mu = c["friction"]
        # Finite-radius section contact replaces the upstream grid contact;
        # penalty stiffness is unsuitable for milligram-scale cable samples.
        self.model.struct.body_ke = 0.
        self.model.struct.body_kd = 0.
        self.model.struct.particle_radius = c["diameter"] / 6
        self.model.mpm_contact_margin = self.spacing * 2
        self.model.mpm_contact_max = max(65536, len(world) * 8)
        self.states = [self.model.state(), self.model.state()]
        self.integrator = CableMPMSimulator(device=self.device, padding=config["mpm"]["grid_padding"],
                                           anchor_grid=self._anchor_grid)
        self.pin_ids_np = np.flatnonzero(pinned).astype(np.int32)
        mount_links = {USB_LINK, 'left_fr3_hand', 'left_fr3_leftfinger', 'left_fr3_rightfinger'}
        pinned_exclusions = [int(self.actors[b].name in mount_links) for b in self.model.shape_body.numpy()]
        guide_shapes = [i for i, b in enumerate(self.model.shape_body.numpy())
            if self.guide is not None and self.actors[b].name in
            ("right_fr3_leftfinger", "right_fr3_rightfinger")]
        self.contacts = CableContacts(self.model, self.sections, len(self.pin_ids_np)//7, c, self.device,
                                      pinned_exclusions=pinned_exclusions, frictionless_shapes=guide_shapes)
        self.previous_body = wp.zeros_like(self.states[0].body_q)
        self.fibers = AxialFibers(world, c, len(self.pin_ids_np), self.device)
        self.pin_ids = wp.array(self.pin_ids_np, dtype=int, device=self.device)
        self.targets = wp.zeros(len(self.pin_ids_np), dtype=wp.vec3, device=self.device)
        self.target_velocities = wp.zeros_like(self.targets)
        self.pin_impulse = wp.zeros(1, dtype=wp.spatial_vector, device=self.device)
        self.pin_com_device = wp.zeros(1, dtype=wp.vec3, device=self.device)
        self.device_bounds = DeviceBounds(len(world), self.device)
        self.constraint_graphs = ConstraintGraphs(config['mpm'].get('cuda_graph', True))
        self.last_attachment_force = np.zeros(3)
        self.mpm_steps = 0
        if self.guide is not None:
            self.guide.bind(self)
        self._prepare_anchor()
        self._pin(False)
        self._update_bodies()
        self.contacts.capture(self.states[0].struct)
        self.check_contacts()
        try:
            self._setup_render()
        except Exception:
            if hasattr(self, "pcd"):
                self.env._scene.remove_particle_entity(self.pcd)
            raise

    def _build_contacts(self):
        # PhysX convexifies articulation meshes, filling the research fingers'
        # bore. MPM must query the source collision triangles used by MoveIt.
        raw_fingers = finger_collision_meshes(self.env.assets.urdf_path) if self.guide is not None else {}
        actors = list(self.env.agent.links.values()) + list(self.env.fixtures.values())
        if self.plug not in actors:
            actors.append(self.plug)
        for actor in actors:
            shapes = actor.get_collision_shapes()
            if not shapes:
                continue
            body = self.builder.add_body(origin=wp.transform_identity())
            self.actors.append(actor)
            for vertices, indices in raw_fingers.get(actor.name, []):
                mesh = Mesh(vertices, indices, compute_inertia=False)
                self.meshes.append(mesh)
                self.builder.add_shape_mesh(body=body, mesh=mesh, scale=(1., 1., 1.))
            for shape in ([] if actor.name in raw_fingers else shapes):
                g, pose = shape.geometry, shape.get_local_pose()
                args = dict(body=body, pos=tuple(pose.p), rot=tuple(pose.q[[1, 2, 3, 0]]))
                if isinstance(g, sapien.BoxGeometry):
                    self.builder.add_shape_box(**args, hx=g.half_lengths[0],
                        hy=g.half_lengths[1], hz=g.half_lengths[2])
                elif isinstance(g, sapien.CapsuleGeometry):
                    self.builder.add_shape_capsule(**args, radius=g.radius, half_width=g.half_length)
                elif isinstance(g, (sapien.ConvexMeshGeometry, sapien.NonconvexMeshGeometry)):
                    vertices = np.asarray(g.vertices) * np.asarray(g.scale)
                    if hasattr(g, "rotation"):
                        vertices = vertices @ quat2mat(g.rotation).T
                    mesh = Mesh(vertices, np.asarray(g.indices).reshape(-1), compute_inertia=False)
                    self.meshes.append(mesh)
                    # Direct triangle queries retain the open trunking cavity;
                    # the legacy actor2meshes helper rejects nonconvex meshes.
                    self.builder.add_shape_mesh(**args, mesh=mesh, scale=(1., 1., 1.))
                else:
                    raise ValueError(f"Unsupported MPM contact geometry: {type(g).__name__}")
            if actor.type == "static":
                self.builder.set_body_mass(body, 0., np.zeros((3, 3)), np.zeros(3))
            else:
                cmass = actor.cmass_local_pose
                rotation = quat2mat(cmass.q)
                self.builder.set_body_mass(body, actor.mass,
                    rotation @ np.diag(actor.inertia) @ rotation.T, cmass.p)

    @property
    def positions(self):
        return self.states[0].struct.particle_q.numpy()

    @property
    def centerline(self):
        return self.positions.reshape(self.sections, 7, 3).mean(axis=1)

    def _prepare_anchor(self):
        pose = self.plug.pose
        r = quat2mat(pose.q)
        p = self.local[self.pin_ids_np] @ r.T + pose.p
        com = r @ self.plug.cmass_local_pose.p + pose.p
        w = np.asarray(self.plug.angular_velocity)
        v = np.asarray(self.plug.velocity) + np.cross(w, p - com)
        self.targets.assign(p.astype(np.float32))
        self.target_velocities.assign(v.astype(np.float32))
        self.pin_rotation = wp.mat33(*(r @ self.initial_rotation.T).reshape(-1))
        self.pin_spin = wp.mat33(0., -w[2], w[1], w[2], 0., -w[0], -w[1], w[0], 0.)
        self.pin_com = wp.vec3(*com)
        self.pin_com_device.assign(np.asarray(com, dtype=np.float32).reshape(1, 3))
        self.pin_pose = pose_transform(pose)
        self.pin_linear = wp.vec3(*self.plug.velocity)
        self.pin_angular = wp.vec3(*w)

    def _anchor_grid(self, state):
        c = self.config["cable"]
        wp.launch(anchor_grid_kernel, dim=self.grid_dims, inputs=[
            self.model.struct, state.struct, self.pin_pose,
            wp.vec3(*self.config["usb"]["attachment"]), c["pin_length"],
            c["diameter"] / 2 + self.spacing, self.pin_com, self.pin_linear,
            self.pin_angular, self.pin_impulse], device=self.device)

    def _pin(self, reaction):
        state = self.states[0].struct
        wp.launch(pin_particles, dim=len(self.pin_ids_np), inputs=[
            self.pin_ids, self.targets, self.target_velocities, state.particle_q,
            state.particle_qd, state.particle_F, state.particle_C,
            self.model.struct.particle_mass, self.pin_rotation, self.pin_spin,
            self.pin_com, self.pin_impulse, int(reaction)], device=self.device)

    def _ensure_grid(self, positions=None):
        if positions is None:
            if self.config['mpm'].get('gpu_grid_check', True):
                positions = self.device_bounds.check(self.states[0].struct.particle_q,
                    self.spacing, self.config['mpm']['grid_padding'], self.grid_dims)
                if positions is None:
                    return
            else:
                positions = self.positions
        if not np.isfinite(positions).all():
            raise RuntimeError("MPM particle state became non-finite")
        spacing, dims = grid_layout(positions, self.config, self.spacing)
        if spacing == self.spacing and all(a <= b for a, b in zip(dims, self.grid_dims)):
            return
        # Keep growing axes between resolution changes, within the same budget.
        if spacing == self.spacing:
            grown = tuple(max(a, b) for a, b in zip(dims, self.grid_dims))
            if np.prod(grown) <= self.config["mpm"]["max_grid_cells"]:
                dims = grown
        wp.synchronize()
        self.constraint_graphs.clear()
        self.model.struct.dx, self.model.struct.inv_dx = spacing, 1 / spacing
        self.model.struct.grid_dim_x, self.model.struct.grid_dim_y, self.model.struct.grid_dim_z = dims
        for state in self.states:
            state.struct.grid_v = wp.zeros(dims, dtype=wp.vec3, device=self.device)
            state.struct.grid_mv = wp.zeros(dims, dtype=wp.vec3, device=self.device)
            state.struct.grid_m = wp.zeros(dims, dtype=float, device=self.device)
        if spacing > self.spacing:
            self.grid_changes += 1
            print(f"MPM grid spacing increased to {spacing:.6f} m to stay within GPU memory budget", flush=True)
        self.spacing, self.grid_dims = spacing, dims

    def _update_bodies(self):
        poses, velocities = [], []
        for actor in self.actors:
            pose = actor.pose
            poses.append(np.r_[pose.p, pose.q[[1, 2, 3, 0]]])
            velocities.append(np.zeros(6) if actor.type == "static" else
                              np.r_[actor.angular_velocity, actor.velocity])
        for state in self.states:
            state.body_q.assign(np.asarray(poses, dtype=np.float32))
            state.body_qd.assign(np.asarray(velocities, dtype=np.float32))
        # Contact cache entries are distances/normals in shape-local space.
        # Rigid motion changes the query coordinates, not the cached geometry;
        # the exact-coordinate/clearance checks account for that displacement.
        if self.guide is not None:
            self.guide.update_pose()

    def _apply_contact_impulses(self, dt):
        impulses = self.contacts.impulse.numpy()
        if not np.isfinite(impulses).all():
            raise RuntimeError("Non-finite cable contact impulse")
        for actor, impulse in zip(self.actors, impulses):
            collector = getattr(self.env, "force_collector", None)
            if collector is not None:
                origin = actor.pose.p if actor.type == "static" else (actor.pose*actor.cmass_local_pose).p
                collector.mpm_reaction(actor, impulse, origin)
            if actor.type not in ("static", "kinematic"):
                actor.add_force_torque(impulse[3:] / dt, impulse[:3] / dt)
        self.contacts.impulse.zero_()

    def step(self, rigid_dt):
        count = round(rigid_dt * self.config["mpm"]["frequency"])
        if count < 1 or abs(count / self.config["mpm"]["frequency"] - rigid_dt) > 1e-9:
            raise ValueError("mpm.frequency must be divisible by the rigid simulation frequency")
        dt = rigid_dt / count
        self.rigid_dt = rigid_dt
        self._update_bodies()
        self._prepare_anchor()
        self.pin_impulse.zero_()
        if self.guide is not None:
            self.guide.begin_step()
        for _ in range(count):
            self._pin(False)
            self.contacts.capture(self.states[0].struct)
            self._ensure_grid()
            self.integrator.simulate(self.model, self.states[0], self.states[1], dt)
            if self.states[1].struct.error.numpy()[0]:
                raise RuntimeError(f"ManiSkill2 MPM reported a numerical failure at substep {self.mpm_steps}")
            if self.states[0].mpm_contact_count.numpy()[0] > self.model.mpm_contact_max:
                raise RuntimeError("MPM contact buffer overflow")
            self.states.reverse()
            self._pin(True)
            self._solve_fibers(dt)
            self.mpm_steps += 1
        reaction = self.pin_impulse.numpy()[0] / rigid_dt
        if not np.isfinite(reaction).all():
            raise RuntimeError("Non-finite cable reaction force")
        self._apply_contact_impulses(rigid_dt)
        if self.guide is not None:
            self.guide.apply_reaction(rigid_dt)
        self.plug.add_force_torque(reaction[3:], reaction[:3])
        collector = getattr(self.env, "force_collector", None)
        if collector is not None:
            collector.mpm_reaction(self.plug, reaction*rigid_dt,
                (self.plug.pose*self.plug.cmass_local_pose).p, source="mpm_attachment_reaction")
        self.last_attachment_force = reaction[3:].copy()
        wp.copy(self.previous_body, self.states[0].body_q)

    def _solve_fibers(self, dt):
        def record():
            self.fibers.solve(self.states[0].struct, self.model.struct.particle_mass,
                              dt, self.pin_com_device, self.pin_impulse,
                              lambda: self._solve_constraints(dt))
        if self.contacts.profile_enabled:
            # Counters have a separate allocation and host call accounting.
            # Keep diagnostics on the ordinary path, never replay stale flags.
            record()
        else:
            c = self.config['cable']
            signature = (dt, c['axial_iterations'], c['axial_young_modulus'], c['diameter'],
                         self.spacing, self.grid_dims, self.contacts.passes,
                         self.contacts.radius, self.contacts.margin, self.contacts.profile_counts.ptr,
                         self.guide.config['half_length'] if self.guide else None, self.support_count)
            self.constraint_graphs.run(signature, self.states[0].struct.particle_q.ptr, record)
        if self.guide is not None:
            self.guide._dirty = True

    def follow_plug(self):
        self._prepare_anchor()
        self._pin(False)
        self._update_bodies()
        if self.guide is not None:
            self.guide.solve(self, self.rigid_dt)
            self.guide.apply_reaction(self.rigid_dt)
        self.contacts.solve(self.states[0], old_body=self.previous_body, reuse_query_cache=True)
        # These reactions are consumed by the next rigid step.
        self._apply_contact_impulses(self.rigid_dt)

    def _solve_constraints(self, dt):
        if self.guide is not None:
            self.guide.solve(self, dt)
        # Contact is last: the guide's exterior transition must not push the
        # free cable through a nearby trunking wall after collision correction.
        self.contacts.solve(self.states[0], reuse_query_cache=True)
        self._apply_temporary_supports()

    def check_contacts(self, positions=None):
        depths = self.contacts.audit(self.states[0], positions=positions)
        if self.contacts.max_depth > self.config['cable']['penetration_tolerance']:
            shape = int(np.argmax(depths))
            body = int(self.model.shape_body.numpy()[shape])
            raise RuntimeError(f"Cable penetrates {self.actors[body].name} by "
                               f"{self.contacts.max_depth*1000:.3f} mm; reset or reduce motion speed")

    def reset(self):
        if self.support_count:
            raise RuntimeError("Cannot reset MPM cable while temporary world supports are active; "
                               "use the scene reset to remove USB, cable and supports together")
        r = quat2mat(self.plug.pose.q)
        previous_coordinate = self.guide.material_coordinate if self.guide is not None else None
        # Validate the proposed layout before replacing the running state.
        self._update_bodies()
        previous_depth = self.contacts.max_depth
        try:
            world = self.layout(self.config, self.local, r, self.plug.pose.p)
            self.check_contacts(wp.array(world, dtype=wp.vec3, device=self.device))
            self._ensure_grid(world)
        except (RuntimeError, ValueError):
            self.contacts.max_depth = previous_depth
            if self.guide is not None:
                self.guide.material_coordinate = previous_coordinate
            raise
        if self.guide is not None:
            self.guide.begin_step(reset=True)
            self.guide.radial_error = 0.
        self.contacts.invalidate_query_cache()
        self.constraint_graphs.clear()
        self.initial_rotation = r.copy()
        self.fibers.rest.assign(np.linalg.norm(world[7:] - world[:-7], axis=1).astype(np.float32))
        for state in self.states:
            state.struct.particle_q.assign(world.astype(np.float32))
            state.struct.particle_qd.zero_()
            state.struct.particle_C.zero_()
            state.struct.particle_F.assign(np.tile(np.eye(3, dtype=np.float32), (len(world), 1, 1)))
            state.struct.particle_vol.assign(self.model.struct.particle_vol.numpy())
            state.struct.particle_volume_correction.zero_()
            state.struct.error.zero_()
            state.struct.grid_v.zero_()
            state.struct.grid_mv.zero_()
            state.struct.grid_m.zero_()
        self.last_attachment_force[:] = 0
        self._prepare_anchor()
        self._pin(False)
        self._update_bodies()
        self.contacts.capture(self.states[0].struct)
        self.contacts.impulse.zero_()
        self.contacts.hits.zero_()
        self.check_contacts()

    def _setup_render(self):
        n = self.sections
        self.render_positions = wp.zeros(n, dtype=wp.vec3, device=self.device)
        self.pcd = self.env._scene.add_particle_entity(np.zeros((n, 3), dtype=np.float32))
        self.pcd.visual_body.set_attribute("color", np.tile([1., .32, .03, 1.], (n, 1)).astype(np.float32))
        self.pcd.visual_body.set_attribute("scale", np.full(n, self.config["cable"]["diameter"] / 2, dtype=np.float32))
        self.pcd.visual_body.set_rendered_point_count(n)
        self.vertices_ptr = sapien.pysapien.dlpack.dl_ptr(self.pcd.visual_body.dl_vertices)
        self.vertices_shape = sapien.pysapien.dlpack.dl_shape(self.pcd.visual_body.dl_vertices)

    def update_render(self):
        wp.launch(render_centers, dim=self.sections, inputs=[self.states[0].struct.particle_q,
                  self.render_positions], device=self.device)
        wp.context.runtime.core.memcpy2d_d2d(ctypes.c_void_p(self.vertices_ptr),
            ctypes.c_size_t(self.vertices_shape[1] * 4),
            ctypes.c_void_p(self.render_positions.ptr), ctypes.c_size_t(12),
            ctypes.c_size_t(12), ctypes.c_size_t(self.sections))
        wp.synchronize()

    def diagnostics(self):
        x = self.positions
        center = x.reshape(self.sections, 7, 3).mean(axis=1)
        if self.guide is not None:
            self.guide.measure(center)
        target = self.local[self.pin_ids_np] @ quat2mat(self.plug.pose.q).T + self.plug.pose.p
        contacted_shapes = np.flatnonzero(self.contacts.hits.numpy())
        shape_bodies = self.model.shape_body.numpy()
        contacted_bodies = sorted({self.actors[shape_bodies[s]].name for s in contacted_shapes})
        return {"solver": self.solver, "guide_material_coordinate": self.guide.material_coordinate if self.guide else None,
            "guide_radial_error_m": self.guide.radial_error if self.guide else None,
            "particles": len(x), "rest_length_m": self.config["cable"]["length"],
            "diameter_m": self.config["cable"]["diameter"],
            "current_centerline_length_m": float(np.linalg.norm(np.diff(center, axis=0), axis=1).sum()),
            "max_section_gap_m": float(np.linalg.norm(np.diff(center, axis=0), axis=1).max()),
            "collar_next_section_gap_m": float(np.linalg.norm(center[len(self.pin_ids_np)//7] - center[len(self.pin_ids_np)//7-1])),
            "attachment_error_m": float(np.linalg.norm(x[self.pin_ids_np] - target, axis=1).max()),
            "attachment_force_N": self.last_attachment_force.tolist(),
            "max_rigid_penetration_m": self.contacts.max_depth,
            "contact_margin_m": self.contacts.margin,
            "contact_model": "swept_section_spheres_after_fibers",
            "contacted_bodies": contacted_bodies,
            "grid_spacing_m": self.spacing, "grid_dimensions": list(self.grid_dims),
            "grid_memory_MiB": float(np.prod(self.grid_dims) * 56 / 1024 ** 2),
            "grid_resolution_changes": self.grid_changes, "mpm_steps": self.mpm_steps,
            "cuda_graph": {"enabled": self.constraint_graphs.enabled,
                "captures": self.constraint_graphs.captures, "replays": self.constraint_graphs.replays,
                "fallback_reason": self.constraint_graphs.fallback_reason},
            "particle_bounds": [x.min(axis=0).tolist(), x.max(axis=0).tolist()]}
