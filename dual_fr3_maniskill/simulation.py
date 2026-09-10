"""ManiSkill2 / SAPIEN 2 environment and measured state for the ROS bridge."""
from __future__ import annotations

import numpy as np
import sapien.core as sapien
from mani_skill2.agents.base_agent import AgentConfig, BaseAgent
from mani_skill2.agents.controllers import PDJointPosControllerConfig
from mani_skill2.envs.sapien_env import BaseEnv
from mani_skill2.sensors.camera import CameraConfig
from mani_skill2.utils import sapien_utils
from transforms3d.euler import quat2euler

from .assets import SceneAssets, origin_matrix, pose_values
from .collision import collision_cliques


class DualFR3Agent(BaseAgent):
    def __init__(self, scene, control_freq, control_mode, *, assets: SceneAssets):
        self.assets = assets
        super().__init__(scene, control_freq, control_mode, config=AgentConfig(
            urdf_path=str(assets.urdf_path), urdf_config={},
            controllers={"pd_joint_pos": None}, cameras={}))

    def _load_articulation(self):
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        loader.load_multiple_collisions_from_file = True
        self.robot = loader.load(self.urdf_path)
        if self.robot is None:
            raise RuntimeError(f"Failed to load dual FR3: {self.urdf_path}")
        self.robot.set_name("dual_fr3")
        self.robot_link_ids = [link.get_id() for link in self.robot.get_links()]
        self.links = {link.name: link for link in self.robot.get_links()}
        self.joints = {joint.name: joint for joint in self.robot.get_active_joints()}
        collidable = {name: link for name, link in self.links.items() if link.get_collision_shapes()}
        self.collision_exclusion_groups = collision_cliques(self.assets.disabled_collisions, collidable)
        for bit, names in enumerate(self.collision_exclusion_groups):
            for name in names:
                for shape in collidable[name].get_collision_shapes():
                    groups = list(shape.get_collision_groups())
                    groups[2] |= 1 << bit
                    shape.set_collision_groups(*groups)

    def _setup_controllers(self):
        names = list(self.joints)
        fingers = ["finger_joint" in name for name in names]
        self.controller_configs = {"pd_joint_pos": PDJointPosControllerConfig(
            joint_names=names, lower=None, upper=None,
            stiffness=[1000.0 if finger else 4000.0 for finger in fingers],
            damping=[50.0 if finger else 200.0 for finger in fingers],
            force_limit=[min(40.0, self.assets.limits[name][2]) if finger
                         else self.assets.limits[name][2] for name, finger in zip(names, fingers)],
            normalize_action=False, use_delta=False,
        )}
        self.supported_control_modes = list(self.controller_configs)
        super()._setup_controllers()

    def before_simulation_step(self):
        super().before_simulation_step()
        self.robot.set_qf(self.robot.compute_passive_force(
            gravity=True, coriolis_and_centrifugal=True, external=False))


class DualFR3Env(BaseEnv):
    SUPPORTED_REWARD_MODES = ("none",)

    def __init__(self, assets: SceneAssets, *, control_freq=100, sim_freq=500,
                 viewer=False):
        if control_freq <= 0 or sim_freq <= 0 or sim_freq % control_freq:
            raise ValueError("Positive sim_freq must be divisible by control_freq")
        self.assets = assets
        super().__init__(
            obs_mode="none", reward_mode="none", control_mode="pd_joint_pos",
            control_freq=control_freq, sim_freq=sim_freq,
            render_mode="human" if viewer else "rgb_array",
            renderer_kwargs={"offscreen_only": not viewer},
        )

    def _configure_agent(self):
        self._agent_cfg = None

    def _load_agent(self):
        self.agent = DualFR3Agent(self._scene, self._control_freq, self._control_mode,
                                  assets=self.assets)

    def _get_default_scene_config(self):
        config = super()._get_default_scene_config()
        config.contact_offset = 0.001
        config.solver_iterations = 20
        return config

    def _load_actors(self):
        self.fixtures = {}
        for link, transform in self.assets.fixtures:
            builder = self._scene.create_actor_builder()
            for tag in ("collision", "visual"):
                for element in link.findall(tag):
                    pose = sapien.Pose(*pose_values(origin_matrix(element.find("origin"))))
                    geometry = element.find("geometry")
                    box, mesh = geometry.find("box"), geometry.find("mesh")
                    material = None
                    if tag == "visual":
                        material = self._renderer.create_material()
                        spec = element.find("material")
                        if spec is not None:
                            color = spec.find("color")
                            material.base_color = (np.fromstring(color.get("rgba"), sep=" ").tolist()
                                if color is not None else
                                self.assets.materials.get(spec.get("name"), [0.7, 0.7, 0.7, 1]))
                    if box is not None:
                        size = np.fromstring(box.get("size"), sep=" ") / 2
                        if tag == "collision":
                            builder.add_box_collision(pose=pose, half_size=size)
                        else:
                            builder.add_box_visual(pose=pose, half_size=size, material=material)
                    elif mesh is not None:
                        params = dict(filename=mesh.get("filename"), pose=pose,
                                      scale=np.fromstring(mesh.get("scale", "1 1 1"), sep=" "))
                        if tag == "collision":
                            builder.add_nonconvex_collision_from_file(**params)
                        else:
                            builder.add_visual_from_file(**params, material=material)
                    else:
                        raise ValueError(f"Unsupported fixture geometry in {link.get('name')}")
            actor = builder.build_static(name=link.get("name"))
            actor.set_pose(sapien.Pose(*pose_values(transform)))
            if link.findall("collision") and not actor.get_collision_shapes():
                raise RuntimeError(f"Failed to load collision geometry for {link.get('name')}")
            self.fixtures[link.get("name")] = actor

    def _register_render_cameras(self):
        pose = sapien_utils.look_at([2.0, 2.3, 1.8], [0.45, 0.7, 0.25])
        return CameraConfig("render_camera", pose.p, pose.q, 1280, 960, 1.0, 0.01, 10)

    def _setup_viewer(self):
        super()._setup_viewer()
        pose = sapien_utils.look_at([2.0, 2.3, 1.8], [0.45, 0.7, 0.25])
        r, p, y = quat2euler(pose.q)
        self._viewer.set_camera_xyz(*pose.p)
        self._viewer.set_camera_rpy(r, -p, -y)

    def _initialize_agent(self):
        self.agent.reset(np.array([self.assets.initial_positions[name] for name in self.agent.joints],
                                  dtype=np.float32))

    def evaluate(self, **kwargs):
        return {}

    def get_reward(self, **kwargs):
        return 0.0

    def get_done(self, **kwargs):
        return False

    def _get_obs_extra(self):
        return {}


class Simulation:
    def __init__(self, assets, *, control_freq=100, sim_freq=500, viewer=False):
        self.env = DualFR3Env(assets, control_freq=control_freq, sim_freq=sim_freq, viewer=viewer)
        self.env.reset(seed=0)
        self.names = list(self.env.agent.joints)
        self.indices = {name: i for i, name in enumerate(self.names)}
        self.link_names = set(self.env.agent.links)
        self.viewer = viewer
        self.control_freq = control_freq
        self.steps = 0
        self.target = self.positions.copy()
        for name in self.names:
            if "finger_joint2" in name:
                first = name.replace("finger_joint2", "finger_joint1")
                if first not in self.indices:
                    raise ValueError(f"Missing mimic source {first}")

    @property
    def time(self):
        return self.steps / self.control_freq

    @property
    def positions(self):
        return self.env.agent.robot.get_qpos().astype(float)

    @property
    def velocities(self):
        return self.env.agent.robot.get_qvel().astype(float)

    def link_pose(self, name):
        """Return measured world position followed by quaternion (w, x, y, z)."""
        pose = self.env.agent.links[name].get_pose()
        return np.concatenate((pose.p, pose.q)).astype(float)

    def step(self):
        for name, index in self.indices.items():
            if "finger_joint2" in name:
                self.target[index] = self.target[self.indices[name.replace("finger_joint2", "finger_joint1")]]
        self.env.step(self.target.astype(np.float32))
        self.steps += 1
        if not np.isfinite(self.positions).all() or not np.isfinite(self.velocities).all():
            raise RuntimeError("Non-finite state from ManiSkill2 physics")
        if self.viewer and self.steps % max(1, self.control_freq // 30) == 0:
            self.env.render()

    def set_gripper_force(self, side, effort):
        for index in (1, 2):
            self.env.agent.joints[f"{side}_fr3_finger_joint{index}"].set_drive_property(
                1000.0, 50.0, force_limit=effort)

    def render_image(self, *, fixture_closeup=False):
        if fixture_closeup:
            self.env._render_cameras["render_camera"].camera.set_local_pose(
                sapien_utils.look_at([1.2, 0.65, 1.0], [0.54, 0.65, 0.02]))
        return self.env.render_rgb_array()

    def close(self):
        self.env.close()
