"""MTC scene: load only the robot, insert the held cable after preparation."""
from functools import partial
from pathlib import Path

from ..sapien_compat import sapien

from ament_index_python.packages import get_package_share_directory
import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat

from ..simulation import DualFR3Env, Simulation
from ..assets import convert_stl_to_glb
from ..cable.model import USB_LINK
from ..cable.threading import LEFT_TCP, RIGHT_TCP, TOUCH_LINKS, task_usb_mount, threaded_positions, threaded_centerline
from ..cable.guide import SlidingGuide
from ..cable.backends import create_cable, prepare_backend, shader_directory, validate_solver
from .usb_cable import UsbCableEnv, UsbCableSimulation


class TrunkingCableEnv(UsbCableEnv):
    def __init__(self, assets, *, cable_config, cable_solver="mpm", **kwargs):
        # Fail before publishing arm actions; insertion happens after preparation.
        self.cable_solver = validate_solver(cable_solver)
        self.cable = None
        self.plug = self.mount_drive = None
        self.cable_config = cable_config
        self._prepare_cable_engine()
        DualFR3Env.__init__(self, assets, shader_dir=shader_directory(cable_solver), **kwargs)
        self._configure_cable_timestep()
        for name in (LEFT_TCP, RIGHT_TCP):
            if name not in self.agent.links:
                raise ValueError(f"MTC cable requires the Franka hand TCP {name}")

    def step_action(self, action):
        if self.cable is None:
            self.agent.set_action(action)
            if self.rope_trace is not None:
                self.rope_trace.capture("control", self, action=action)
            for _ in range(self._sim_steps_per_control*self.rigid_substeps):
                self.agent.before_simulation_step()
                self._scene.step()
                if self.rope_trace is not None:
                    self.rope_trace.capture("rigid_step", dt=self.sim_timestep/self.rigid_substeps)
            return
        return super().step_action(action)

    def spawn_cable(self):
        if self.cable is not None:
            return

        config = self.cable_config
        left, right = self.agent.links[LEFT_TCP], self.agent.links[RIGHT_TCP]
        offset, rotation = task_usb_mount(config)
        mount = sapien.Pose(offset, mat2quat(rotation))
        builder = self._scene.create_actor_builder()
        mesh = str(Path(get_package_share_directory("dual_fr3_maniskill")) / "meshes/USB1.stl")
        scale = [config["usb"]["mesh_scale"]]*3
        builder.add_collision_from_file(mesh, scale=scale)
        if self._renderer is not None:
            material = self._renderer.create_material()
            material.base_color = [.08, .25, .65, 1.]
            visual = convert_stl_to_glb(Path(mesh), self.assets.urdf_path.parent)
            builder.add_visual_from_file(str(visual), scale=scale, material=material)
        size = np.array([.0076, .0379, .0144])*scale
        inertia = config["usb"]["mass"]/12 * (np.sum(size**2) - size**2)
        builder.set_mass_and_inertia(config["usb"]["mass"],
            sapien.Pose([0., -.00105*scale[0], 0.]), inertia)
        plug = builder.build(USB_LINK)
        plug.set_pose(left.pose * mount)
        plug.set_velocity(left.velocity + np.cross(left.angular_velocity,
            plug.pose.p - left.pose.p))
        plug.set_angular_velocity(left.angular_velocity)
        drive = None
        changed_shapes = []
        guide = SlidingGuide(right, config["guide"], 0.)

        def layout(cfg, local, r, p):
            if self.cable_solver == "rope_actor":
                points, _ = threaded_centerline(cfg, local, r, p,
                    guide.pose.p, quat2mat(guide.pose.q)[:, 0])
                return points
            points, coordinate = threaded_positions(cfg, local, r, p,
                guide.pose.p, quat2mat(guide.pose.q)[:, 0])
            guide.material_coordinate = coordinate
            return points

        try:
            # Bit 31 is reserved here; assets.collision_cliques uses bits 0..30.
            # Disable only the USB versus its fixed mounting links.
            for link in [plug] + [self.agent.links[n] for n in TOUCH_LINKS]:
                for shape in link.get_collision_shapes():
                    groups = shape.get_collision_groups()
                    changed_shapes.append((shape, groups))
                    shape.set_collision_groups(groups[0], groups[1], groups[2] | (1 << 31), groups[3])
            drive = self._scene.create_drive(left, mount, plug, sapien.Pose())
            drive.lock_motion(True, True, True, True, True, True)
            cable = create_cable(self, config, solver=self.cable_solver, plug=plug, layout=layout, guide=guide)
        except Exception:
            if drive is not None:
                self._scene.remove_drive(drive)
            for shape, groups in changed_shapes:
                shape.set_collision_groups(*groups)
            self._scene.remove_actor(plug)
            raise
        self.plug, self.mount_drive, self.cable = plug, drive, cable


class TrunkingCableSimulation(UsbCableSimulation):
    def __init__(self, assets, *, cable_config, cable_solver="mpm", **kwargs):
        prepare_backend(cable_solver, cable_config, kwargs.get("sim_freq", 500))
        Simulation.__init__(self, assets,
            env_factory=partial(TrunkingCableEnv, cable_config=cable_config, cable_solver=cable_solver), **kwargs)

    def step(self):
        if self.cable is None:
            return Simulation.step(self)
        return super().step()
