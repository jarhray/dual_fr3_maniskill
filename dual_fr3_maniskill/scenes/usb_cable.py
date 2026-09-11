"""USB cable scene using the shared rigid-body and MPM simulation loop."""
from functools import partial
from pathlib import Path

import mani_skill2
import numpy as np
from mani_skill2.sensors.camera import CameraConfig
from mani_skill2.utils import sapien_utils
from transforms3d.euler import quat2euler

from dual_fr3_maniskill.simulation import DualFR3Env, Simulation
from ..cable.mpm_cable import MPMCable


class UsbCableEnv(DualFR3Env):
    def __init__(self, assets, *, cable_config, **kwargs):
        self.cable = None
        self.cable_config = cable_config
        shader = Path(mani_skill2.__file__).parent / "envs/mpm/shader/point"
        super().__init__(assets, shader_dir=str(shader), **kwargs)
        self.cable = MPMCable(self, cable_config)

    def step_action(self, action):
        self.agent.set_action(action)
        for _ in range(self._sim_steps_per_control):
            self.agent.before_simulation_step()
            self.cable.step(self.sim_timestep)
            self._scene.step()
            self.cable.follow_plug()

    def update_render(self):
        if self.cable is not None:
            self.cable.update_render()
        super().update_render()

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
    def __init__(self, assets, *, cable_config, **kwargs):
        for i in (1, 2):
            assets.initial_positions[f"left_fr3_finger_joint{i}"] = cable_config["usb"]["finger_position"]
        if cable_config["mpm"]["frequency"] % kwargs.get("sim_freq", 500):
            raise ValueError("mpm.frequency must be divisible by sim_freq")
        super().__init__(assets, env_factory=partial(UsbCableEnv, cable_config=cable_config), **kwargs)

    def step(self):
        super().step()
        self.cable.check_contacts()
        center = self.cable.centerline
        gap = np.linalg.norm(np.diff(center, axis=0), axis=1).max()
        if not np.isfinite(gap) or gap > self.cable.config["cable"]["particle_spacing"] * 2:
            raise RuntimeError(f"Cable lost axial continuity (largest section gap {gap:.6f} m)")

    @property
    def cable(self):
        return self.env.cable
