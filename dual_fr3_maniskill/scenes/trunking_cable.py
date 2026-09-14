"""MTC scene: load only the robot, insert the held cable after preparation."""
from functools import partial
from pathlib import Path

from ..sapien_compat import sapien

from ament_index_python.packages import get_package_share_directory
import mani_skill2
import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat

from ..simulation import DualFR3Env, Simulation
from ..assets import convert_stl_to_glb
from ..cable.model import USB_LINK
from ..cable.threading import LEFT_TCP, RIGHT_TCP, TOUCH_LINKS, task_usb_mount, threaded_positions
from ..cable.guide import SlidingGuide
from ..cable.mpm_cable import initialize_warp
from .usb_cable import UsbCableEnv, UsbCableSimulation


class TrunkingCableEnv(UsbCableEnv):
    def __init__(self, assets, *, cable_config, **kwargs):
        # Fail before publishing arm actions; insertion happens after preparation.
        initialize_warp()
        self.cable = None
        self.plug = self.mount_drive = None
        self.cable_config = cable_config
        shader = Path(mani_skill2.__file__).parent / "envs/mpm/shader/point"
        DualFR3Env.__init__(self, assets, shader_dir=str(shader), **kwargs)
        for name in (LEFT_TCP, RIGHT_TCP):
            if name not in self.agent.links:
                raise ValueError(f"MTC cable requires the Franka hand TCP {name}")

    def step_action(self, action):
        if self.cable is None:
            return DualFR3Env.step_action(self, action)
        return super().step_action(action)

    def spawn_cable(self):
        if self.cable is not None:
            return
        from ..cable.mpm_cable import MPMCable

        config = self.cable_config
        left, right = self.agent.links[LEFT_TCP], self.agent.links[RIGHT_TCP]
        offset, rotation = task_usb_mount(config)
        mount = sapien.Pose(offset, mat2quat(rotation))
        builder = self._scene.create_actor_builder()
        mesh = str(Path(get_package_share_directory("dual_fr3_maniskill")) / "meshes/USB1.stl")
        scale = [config["usb"]["mesh_scale"]]*3
        builder.add_collision_from_file(mesh, scale=scale)
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
            points, coordinate = threaded_positions(cfg, local, r, p,
                right.pose.p, quat2mat(right.pose.q)[:, 0])
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
            cable = MPMCable(self, config, plug=plug, layout=layout, guide=guide)
        except Exception:
            if drive is not None:
                self._scene.remove_drive(drive)
            for shape, groups in changed_shapes:
                shape.set_collision_groups(*groups)
            self._scene.remove_actor(plug)
            raise
        self.plug, self.mount_drive, self.cable = plug, drive, cable


class TrunkingCableSimulation(UsbCableSimulation):
    def __init__(self, assets, *, cable_config, **kwargs):
        if cable_config["mpm"]["frequency"] % kwargs.get("sim_freq", 500):
            raise ValueError("mpm.frequency must be divisible by sim_freq")
        Simulation.__init__(self, assets,
            env_factory=partial(TrunkingCableEnv, cable_config=cable_config), **kwargs)

    def step(self):
        if self.cable is None:
            return Simulation.step(self)
        return super().step()
