"""USB cable scene using the shared rigid-body and MPM simulation loop."""
from functools import partial

# Initialize SAPIEN compatibility before ManiSkill imports its renderer.
from ..simulation import DualFR3Env, Simulation

import numpy as np
from mani_skill2.sensors.camera import CameraConfig
from mani_skill2.utils import sapien_utils
from transforms3d.euler import quat2euler

from ..cable.backends import create_cable, prepare_backend, shader_directory, validate_solver
from ..sapien_compat import sapien


class UsbCableEnv(DualFR3Env):
    def __init__(self, assets, *, cable_config, cable_solver="mpm", **kwargs):
        self.cable = None
        self.cable_config = cable_config
        self.cable_solver = validate_solver(cable_solver)
        self._prepare_cable_engine()
        super().__init__(assets, shader_dir=shader_directory(cable_solver), **kwargs)
        self._configure_cable_timestep()
        self.cable = create_cable(self, cable_config, solver=cable_solver)

    def _prepare_cable_engine(self):
        self.rope_trace = None
        # SAPIEN 2 shares PhysX internals between Engine objects: the FIRST
        # engine sets its tolerances. Keep it alive before ManiSkill constructs
        # its own wrapper; robot/cable coordinates and masses remain SI units.
        self._cable_engine = None
        if self.cable_solver == "rope_actor":
            r = self.cable_config["rope_actor"]
            self._cable_engine = sapien.Engine(tolerance_length=r["engine_tolerance_length"],
                                              tolerance_speed=r["engine_tolerance_speed"])

    def _get_default_scene_config(self):
        config = super()._get_default_scene_config()
        if self.cable_solver == "rope_actor":
            config.enable_tgs = self.cable_config["rope_actor"]["solver_type"] == "tgs"
            # Persistent manifolds avoid the unstable redundant contact sets
            # produced by legacy capsule/triangle-mesh contacts at this scale.
            config.enable_pcm = True
            config.solver_iterations = self.cable_config["rope_actor"]["solver_iterations"]
            config.solver_velocity_iterations = self.cable_config["rope_actor"]["solver_velocity_iterations"]
        return config

    def _configure_cable_timestep(self):
        self.rigid_substeps = (self.cable_config["rope_actor"]["frequency"] // self.sim_freq
                               if self.cable_solver == "rope_actor" else 1)
        self._scene.set_timestep(self.sim_timestep/self.rigid_substeps)

    def step_action(self, action):
        self.agent.set_action(action)
        if self.rope_trace is not None:
            self.rope_trace.capture("control", self, action=action)
        if self.cable_solver == "rope_actor":
            from ..cable.timestep import RopeStepSchedule

            period = self._sim_steps_per_control*self.sim_timestep
            maximum = self.sim_timestep/self.rigid_substeps
            schedule = RopeStepSchedule(period, maximum,
                self.cable._dt if self.cable.steps else None)
            while schedule.remaining_steps:
                limit = self.cable.suggested_timestep(maximum)
                # Recheck the motion bound before EVERY native step. Aligned
                # halving removes residual steps without freezing the bound
                # for a whole control period when contact speeds increase.
                dt = schedule.next_step(limit)
                self._scene.set_timestep(dt)
                self.agent.before_simulation_step()
                self.cable.step(dt)
                self._scene.step()
                try:
                    self.cable.follow_plug()
                finally:
                    if self.rope_trace is not None:
                        self.rope_trace.capture("substep", self, dt=dt)
            return
        for _ in range(self._sim_steps_per_control*self.rigid_substeps):
            self.agent.before_simulation_step()
            self.cable.step(self.sim_timestep/self.rigid_substeps)
            self._scene.step()
            self.cable.follow_plug()

    def update_render(self):
        if self.cable is not None:
            self.cable.update_render()
        super().update_render()

    def close(self):
        if self.rope_trace is not None:
            self.rope_trace.capture("close")
            self.rope_trace = None
        if self.cable is not None:
            self.cable.close()
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
    def __init__(self, assets, *, cable_config, cable_solver="mpm", **kwargs):
        for i in (1, 2):
            assets.initial_positions[f"left_fr3_finger_joint{i}"] = cable_config["usb"]["finger_position"]
        prepare_backend(cable_solver, cable_config, kwargs.get("sim_freq", 500))
        super().__init__(assets, env_factory=partial(UsbCableEnv, cable_config=cable_config, cable_solver=cable_solver), **kwargs)

    def step(self):
        super().step()
        self.cable.check_contacts()
        center = self.cable.centerline
        gap = np.linalg.norm(np.diff(center, axis=0), axis=1).max()
        if not np.isfinite(gap) or gap > self.cable.max_section_gap:
            raise RuntimeError(f"Cable lost axial continuity (largest section gap {gap:.6f} m)")

    @property
    def cable(self):
        return self.env.cable
