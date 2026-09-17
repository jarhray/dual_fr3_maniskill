"""USB cable scene using the shared rigid-body and MPM simulation loop."""
from functools import partial
from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory

# Initialize SAPIEN compatibility before ManiSkill imports its renderer.
from ..simulation import DualFR3Env, Simulation

import numpy as np
from mani_skill2.sensors.camera import CameraConfig
from mani_skill2.utils import sapien_utils
from transforms3d.euler import quat2euler
from transforms3d.quaternions import mat2quat, quat2mat

from ..assets import convert_stl_to_glb
from ..cable.model import USB_LINK
from ..cable.threading import LEFT_TCP, task_usb_mount
from ..usb_grasp import UsbGraspMonitor, read_usb_finger_normal_loads

from ..cable.backends import create_cable, prepare_backend, shader_directory, validate_solver
from ..sapien_compat import sapien


class UsbCableEnv(DualFR3Env):
    def __init__(self, assets, *, cable_config, cable_solver="mpm", load_cable=True, **kwargs):
        self.load_cable = bool(load_cable)
        self.cable = self.plug = self.support_drive = None
        self.insertion = None
        # Compatibility attribute: never create a USB-to-TCP physical mount.
        self.mount_drive = None
        self.cable_config = cable_config
        self.cable_solver = validate_solver(cable_solver)
        self.grasp_monitor = UsbGraspMonitor(cable_config.get("grasp", {}))
        self._grasp_time = 0.
        self.initial_layout_diagnostics = None
        self.preparation_tcp_poses = {}
        self.temporary_supports = []
        self._usb_shape_offsets = []
        self._usb_collision_cache = None
        self._prepare_cable_engine()
        super().__init__(assets,
            shader_dir=shader_directory(cable_solver) if self.load_cable else "ibl", **kwargs)
        self._configure_cable_timestep()
        if LEFT_TCP not in self.agent.links:
            raise ValueError("USB contact grasp requires left_fr3_hand_tcp")

    def _create_cable_for_plug(self, plug):
        return create_cable(self, self.cable_config, solver=self.cable_solver, plug=plug)

    def spawn_cable(self, preparation_poses=None):
        """Create once at the preparation pose, then hold a dynamic USB in WORLD.

        Called only between physics steps. The historic name is retained for ROS.
        USB pose is sampled once; subsequent arm movement cannot move this fixture.
        """
        if self.plug is not None:
            return False
        self.grasp_monitor.reset()
        try:
            self.preparation_tcp_poses = dict(preparation_poses or {})
            if self.preparation_tcp_poses and "left" not in self.preparation_tcp_poses:
                raise ValueError("Preparation requires a left TCP target")
            if self.preparation_tcp_poses and self.load_cable and self.cable_solver != "rope_actor":
                raise ValueError("Pre-positioning before approach currently requires cable_solver:=rope_actor")
            if self.cable_config.get("insertion", {}).get("enabled", False) and self.insertion is None:
                from ..insertion_scene import SocketInsertion
                self.insertion = SocketInsertion(self)
            self._create_positioned_usb()
            self.grasp_monitor.created(self._grasp_time, self.plug.pose)
        except Exception as exc:
            self.clear_objects()
            self.grasp_monitor.fail("creation_failed: " + str(exc), self._grasp_time)
            raise
        return True

    def _create_positioned_usb(self):
        config = self.cable_config
        offset, rotation = task_usb_mount(config)
        tcp_pose = self.preparation_tcp_poses.get("left", self.agent.links[LEFT_TCP].pose)
        world_pose = tcp_pose * sapien.Pose(offset, mat2quat(rotation))
        builder = self._scene.create_actor_builder()
        mesh = Path(get_package_share_directory("dual_fr3_maniskill"))/"meshes/USB1.stl"
        scale = [config["usb"]["mesh_scale"]]*3
        friction = float(config["usb"].get("friction", .5))
        if not np.isfinite(friction) or not 0 <= friction <= 1.5:
            raise ValueError("usb.friction must be finite and between 0 and 1.5")
        material = self._scene.create_physical_material(friction, friction, 0.)
        # SAPIEN writes a cooked .convex.stl next to the input mesh. Keep that
        # runtime artifact in a private cache, never in the source/install tree.
        self._usb_collision_cache = tempfile.TemporaryDirectory(prefix="usb_contact_mesh_")
        collision_mesh = Path(self._usb_collision_cache.name)/mesh.name
        shutil.copyfile(mesh, collision_mesh)
        if config.get("insertion", {}).get("enabled", False):
            from ..insertion_geometry import usb_parts
            for index, part in enumerate(usb_parts(mesh)):
                part_path = Path(self._usb_collision_cache.name)/f"usb_part_{index}.stl"
                part.export(part_path)
                builder.add_collision_from_file(str(part_path), scale=scale, material=material)
        else:
            builder.add_collision_from_file(str(collision_mesh), scale=scale, material=material)
        if self._renderer is not None:
            visual_material = self._renderer.create_material()
            visual_material.base_color = [.08, .25, .65, 1.]
            visual = convert_stl_to_glb(mesh, self.assets.urdf_path.parent)
            builder.add_visual_from_file(str(visual), scale=scale, material=visual_material)
        size = np.array([.0076, .0379, .0144])*scale
        inertia = config["usb"]["mass"]/12 * (np.sum(size**2)-size**2)
        builder.set_mass_and_inertia(config["usb"]["mass"],
            sapien.Pose([0., -.00105*scale[0], 0.]), inertia)
        plug = builder.build(USB_LINK)
        self.plug = plug
        plug.set_pose(world_pose)
        # Resolve the 7.6 mm connector without the default 1 mm shell.
        # Detect approaching cable segments before penetration during descent;
        # actual rest offsets and penetration guards stay zero/unchanged.
        for actor in [plug, self.agent.links["left_fr3_leftfinger"],
                      self.agent.links["left_fr3_rightfinger"]]:
            for shape in actor.get_collision_shapes():
                if actor is not plug:
                    self._usb_shape_offsets.append((shape, shape.contact_offset))
                shape.contact_offset = config["usb"][
                    "contact_offset" if actor is plug else "finger_contact_offset"]
        self.support_drive = self._scene.create_drive(None, world_pose, plug, sapien.Pose())
        self.support_drive.lock_motion(True, True, True, True, True, True)
        if self.load_cable:
            self.cable = self._create_cable_for_plug(plug)

    def release_support(self, *, manual=False):
        """Remove only external positioning; never rewrite pose or velocities."""
        if self.plug is None:
            raise RuntimeError("USB not created")
        if self.support_drive is None:
            return False
        if not manual and not self.grasp_monitor.ready_to_release:
            raise RuntimeError("Sustained contact on both USB fingers is required")
        if not manual and self.load_cable and self.preparation_tcp_poses:
            from ..cable.threading import RIGHT_TCP, bore_alignment
            pose = self.agent.links[RIGHT_TCP].pose
            r = quat2mat(pose.q)
            guide = self.cable_config["guide"]
            report = bore_alignment(self.cable.centerline,
                pose.p+r@np.asarray(guide["center_offset"]), r[:, 0], guide["half_length"],
                radial_tolerance=guide.get("radial_tolerance", .0001),
                axis_tolerance_deg=guide.get("axis_tolerance_deg", 1.))
            if not report["passed"]:
                raise RuntimeError("Right guide has not reached the positioned cable: "+str(report))
        try:
            if self.cable is not None and hasattr(self.cable, "remove_temporary_supports"):
                self.cable.remove_temporary_supports()
            for support in list(self.temporary_supports):
                self._scene.remove_drive(support)
                self.temporary_supports.remove(support)
            self._scene.remove_drive(self.support_drive)
            self.support_drive = None
            self.grasp_monitor.released(self._grasp_time, self.agent.links[LEFT_TCP].pose,
                                        self.plug.pose, manual=manual)
            if hasattr(self.plug, "wake_up"):
                self.plug.wake_up()
        except Exception as exc:
            self.grasp_monitor.fail("support_release_failed: " + str(exc), self._grasp_time)
            raise
        return True

    def clear_objects(self):
        """Explicit reset: remove cable/proxies, local fixtures and the USB."""
        for support in self.temporary_supports:
            self._scene.remove_drive(support)
        self.temporary_supports.clear()
        if self.cable is not None:
            self.cable.close()
            self.cable = None
        # Rope restores source collision groups while the socket still exists.
        if self.insertion is not None:
            self.insertion.close()
            self.insertion = None
        for shape, offset in self._usb_shape_offsets:
            shape.contact_offset = offset
        self._usb_shape_offsets.clear()
        if self.support_drive is not None:
            self._scene.remove_drive(self.support_drive)
            self.support_drive = None
        if self.plug is not None:
            self._scene.remove_actor(self.plug)
            self.plug = None
        if self._usb_collision_cache is not None:
            self._usb_collision_cache.cleanup()
            self._usb_collision_cache = None
        self.initial_layout_diagnostics = None
        self.preparation_tcp_poses = {}
        self.grasp_monitor.reset()

    def _observe_grasp(self, dt):
        self._grasp_time += dt
        if self.insertion is not None:
            self.insertion.sample(dt)
        if self.plug is not None:
            loads = read_usb_finger_normal_loads(self, self.plug, dt)
            self.grasp_monitor.observe(self._grasp_time, self.agent.links[LEFT_TCP].pose,
                                       self.plug.pose, loads)

    def _prepare_cable_engine(self):
        self.rope_trace = None
        # SAPIEN 2 shares PhysX internals between Engine objects: the FIRST
        # engine sets its tolerances. Keep it alive before ManiSkill constructs
        # its own wrapper; robot/cable coordinates and masses remain SI units.
        self._cable_engine = None
        if self.load_cable and self.cable_solver == "rope_actor":
            r = self.cable_config["rope_actor"]
            self._cable_engine = sapien.Engine(tolerance_length=r["engine_tolerance_length"],
                                              tolerance_speed=r["engine_tolerance_speed"])

    def _get_default_scene_config(self):
        config = super()._get_default_scene_config()
        if self.load_cable and self.cable_solver == "rope_actor":
            config.enable_tgs = self.cable_config["rope_actor"]["solver_type"] == "tgs"
            # Persistent manifolds avoid the unstable redundant contact sets
            # produced by legacy capsule/triangle-mesh contacts at this scale.
            config.enable_pcm = True
            config.solver_iterations = self.cable_config["rope_actor"]["solver_iterations"]
            config.solver_velocity_iterations = self.cable_config["rope_actor"]["solver_velocity_iterations"]
        if self.cable_config.get("insertion", {}).get("enabled", False):
            config.enable_pcm = True
            config.solver_iterations = max(config.solver_iterations, 200)
            config.solver_velocity_iterations = max(config.solver_velocity_iterations, 10)
        return config

    def _configure_cable_timestep(self):
        self.rigid_substeps = (self.cable_config["rope_actor"]["frequency"] // self.sim_freq
                               if self.load_cable and self.cable_solver == "rope_actor" else 1)
        if self.cable_config.get("insertion", {}).get("enabled", False):
            frequency = int(self.cable_config["insertion"].get("rigid_frequency_hz", 1000))
            if frequency < self.sim_freq or frequency % self.sim_freq:
                raise ValueError("Insertion rigid_frequency_hz must be a multiple of sim_freq")
            self.rigid_substeps = max(self.rigid_substeps, frequency//self.sim_freq)
        self._scene.set_timestep(self.sim_timestep/self.rigid_substeps)

    def step_action(self, action):
        self.agent.set_action(action)
        if self.rope_trace is not None:
            self.rope_trace.capture("control", self, action=action)
        if self.cable is None:
            # USB-only never imports/schedules/steps a cable solver.
            dt = self.sim_timestep/self.rigid_substeps
            self._scene.set_timestep(dt)
            for _ in range(self._sim_steps_per_control*self.rigid_substeps):
                self.agent.before_simulation_step()
                self._scene.step()
                self._observe_grasp(dt)
                DualFR3Env._sample_forces(self, dt)
                if self.rope_trace is not None:
                    self.rope_trace.capture("rigid_step", dt=dt)
            return
        if self.load_cable and self.cable_solver == "rope_actor":
            from ..cable.timestep import RopeStepSchedule

            period = self._sim_steps_per_control*self.sim_timestep
            maximum = self.sim_timestep/self.rigid_substeps
            adaptive = self.cable.config["rope_actor"].get("adaptive_timestep", True)
            schedule = RopeStepSchedule(period, maximum,
                self.cable._dt if adaptive and self.cable.steps else None)
            while schedule.remaining_steps:
                # Fixed stepping is an explicit preview-quality setting. Keep
                # native contacts, grasp observations and control-boundary
                # geometry/constraint checks, but omit the motion-bound scan.
                limit = self.cable.suggested_timestep(maximum) if adaptive else maximum
                # Adaptive mode rechecks before every native step. Aligned
                # subdivision also tiles fixed-mode control periods exactly.
                dt = schedule.next_step(limit)
                self._scene.set_timestep(dt)
                self.agent.before_simulation_step()
                self.cable.step(dt)
                self._scene.step()
                try:
                    self.cable.follow_plug()
                    self._observe_grasp(dt)
                    DualFR3Env._sample_forces(self, dt)
                finally:
                    if self.rope_trace is not None:
                        self.rope_trace.capture("substep", self, dt=dt)
            return
        # Keep the existing MPM schedule. Dynamic release/drop coupling for
        # that backend is deferred; contact-grasp acceptance uses rope_actor.
        dt = self.sim_timestep/self.rigid_substeps
        self._scene.set_timestep(dt)
        for _ in range(self._sim_steps_per_control*self.rigid_substeps):
            self.agent.before_simulation_step()
            self.cable.step(dt)
            self._scene.step()
            self.cable.follow_plug()
            self._observe_grasp(dt)
            DualFR3Env._sample_forces(self, dt)

    def update_render(self):
        if self.cable is not None:
            self.cable.update_render()
        super().update_render()

    def close(self):
        if self.rope_trace is not None:
            self.rope_trace.capture("close")
            self.rope_trace = None
        self.clear_objects()
        super().close()
        self._cable_engine = None

    def _register_render_cameras(self):
        pose = self._view_pose()
        return CameraConfig("render_camera", pose.p, pose.q, 1280, 960, 1., .01, 20.)

    def _setup_viewer(self):
        super()._setup_viewer()
        pose = self._view_pose()
        r, p, y = quat2euler(pose.q)
        self._viewer.set_camera_xyz(*pose.p)
        self._viewer.set_camera_rpy(r, -p, -y)
        self._viewer.set_fovy(.8 if self.cable_config["cable"]["initial_layout"] == "table_spiral" else 1.)

    def _view_pose(self):
        if self.cable_config["cable"]["initial_layout"] == "table_spiral":
            return sapien_utils.look_at([1.7, -1.2, 1.5], [.4, .6, .25])
        return sapien_utils.look_at([3., -3.8, 2.5], [.45, -.65, -.55])


class UsbCableSimulation(Simulation):
    def __init__(self, assets, *, cable_config, cable_solver="mpm", load_cable=True, **kwargs):
        for i in (1, 2):
            assets.initial_positions[f"left_fr3_finger_joint{i}"] = cable_config["usb"].get("open_finger_position", .02)
        if load_cable:
            prepare_backend(cable_solver, cable_config, kwargs.get("sim_freq", 500))
        super().__init__(assets, env_factory=partial(UsbCableEnv, cable_config=cable_config,
            cable_solver=cable_solver, load_cable=load_cable), **kwargs)
        self.env.spawn_cable()

    def step(self):
        insertion = getattr(self.env, "insertion", None)
        if insertion is not None:
            insertion.begin_window()
        super().step()
        if insertion is not None:
            insertion.finish_window()
            collector = getattr(self.env, "force_collector", None)
            if collector is not None and collector.snapshot is not None:
                collector.snapshot["usb_insertion"] = insertion.policy.snapshot()
        if self.cable is None:
            return
        self.cable.check_contacts()
        center = self.cable.centerline
        if self.cable.guide is not None:
            from ..cable.threading import bore_alignment, RIGHT_TCP
            guide = self.env.cable_config["guide"]
            alignment = {}
            for side, name in (("left", LEFT_TCP), ("right", RIGHT_TCP)):
                if side == "right" and getattr(self.cable, "guide_release_started", False):
                    alignment[side] = dict(available=False, required=False, passed=None,
                        reason="socket_supported_guide_opening_or_released")
                    continue
                if side == "left" and guide.get("routing", "both_guides") == "usb_to_right":
                    alignment[side] = dict(available=False, required=False, passed=None,
                        reason="disabled_usb_to_right_routing", radial_error_m=None,
                        axis_error_deg=None, straight_coverage_range_m=None)
                    continue
                pose = self.env.agent.links[name].pose
                rotation = quat2mat(pose.q)
                hole = pose.p + rotation @ np.asarray(guide["center_offset"])
                alignment[side] = bore_alignment(center, hole, rotation[:, 0], guide["half_length"],
                    radial_tolerance=guide.get("radial_tolerance", .0001),
                    axis_tolerance_deg=guide.get("axis_tolerance_deg", 1.))
            collector = getattr(self.env, "force_collector", None)
            if collector is not None and collector.snapshot is not None:
                collector.snapshot["current_guide_alignment"] = alignment
        gap = np.linalg.norm(np.diff(center, axis=0), axis=1).max()
        if not np.isfinite(gap) or gap > self.cable.max_section_gap:
            raise RuntimeError(f"Cable lost axial continuity (largest section gap {gap:.6f} m)")

    @property
    def cable(self):
        return self.env.cable
