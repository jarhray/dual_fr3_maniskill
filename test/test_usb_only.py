"""USB-only branching and native actor lifecycle, without renderer/robot loading.

The ManiSkill robot loader is replaced with an isolated native PhysX scene.
These tests prove backend exclusion and world-support semantics, not robot grasp.
"""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from dual_fr3_maniskill.cable.model import load_config


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def isolated_env(monkeypatch):
    from dual_fr3_maniskill.scenes import usb_cable, trunking_cable
    from dual_fr3_maniskill.sapien_compat import sapien
    engine = sapien.Engine(tolerance_length=.1, tolerance_speed=.2)
    settings = sapien.SceneConfig()
    settings.gravity = [0, 0, -9.81]
    scene = engine.create_scene(settings)

    def forbidden(*args, **kwargs):
        raise AssertionError("USB-only attempted to initialize or create a cable backend")

    for module in (usb_cable, trunking_cable):
        for name in ("prepare_backend", "create_cable", "shader_directory", "SlidingGuide"):
            if hasattr(module, name):
                monkeypatch.setattr(module, name, forbidden)
    monkeypatch.setattr(usb_cable.sapien, "Engine", forbidden)
    monkeypatch.setattr(usb_cable, "get_package_share_directory", lambda name: str(ROOT))

    def init_robot(env, assets, **kwargs):
        assert kwargs["shader_dir"] == "ibl"
        env.assets, env._scene, env._renderer = assets, scene, None
        env._sim_freq, env._control_freq, env._sim_steps_per_control = 500, 50, 10
        tcp = SimpleNamespace(name="left_fr3_hand_tcp", pose=sapien.Pose([0, 0, 1]))
        fingers = []
        for side, position in (("left", -.1), ("right", .1)):
            builder = scene.create_actor_builder()
            builder.add_box_collision(half_size=[.005]*3)
            finger = builder.build_kinematic("left_fr3_"+side+"finger")
            finger.set_pose(sapien.Pose([position, 0, 1]))
            fingers.append(finger)
        env.agent = SimpleNamespace(links={actor.name: actor for actor in [tcp, *fingers]},
                                    set_action=lambda action: None, before_simulation_step=lambda: None)
        env.fixtures = {}
    monkeypatch.setattr(usb_cable.DualFR3Env, "__init__", init_robot)
    env = trunking_cable.TrunkingCableEnv(SimpleNamespace(),
        cable_config=load_config(ROOT/"config/trunking_cable.yaml", solver=None),
        cable_solver="mpm", load_cable=False)
    yield env, sapien, usb_cable, trunking_cable
    env.clear_objects()


def test_usb_only_spawns_one_actor_and_never_runs_solver_or_requires_right_tcp(isolated_env):
    env, _, _, _ = isolated_env
    before = len(env._scene.get_all_actors())
    assert env._cable_engine is None
    assert env.spawn_cable()
    assert not env.spawn_cable()
    assert len(env._scene.get_all_actors()) == before+1
    assert env.cable is None
    assert env.temporary_supports == []
    assert env.mount_drive is None
    for _ in range(3):
        env.step_action([])
    assert env._grasp_time == pytest.approx(.06)
    assert env.rigid_substeps == 1
    assert env.grasp_monitor.external_support


def test_temporary_support_is_world_fixed_and_release_preserves_state_then_falls(isolated_env):
    env, sapien, _, _ = isolated_env
    env.spawn_cable()
    plug = env.plug
    initial = plug.pose.p.copy()
    env.agent.links["left_fr3_hand_tcp"].pose = sapien.Pose([.25, 0, 1])
    for _ in range(5):
        env.step_action([])
    np.testing.assert_allclose(plug.pose.p, initial, atol=1.e-6)
    # Nonzero state makes an accidental release-time velocity reset visible.
    plug.set_velocity([.1, -.2, .3])
    plug.set_angular_velocity([.2, .1, -.1])
    pose, velocity, angular = plug.pose, plug.velocity.copy(), plug.angular_velocity.copy()
    assert env.release_support(manual=True)
    assert not env.release_support(manual=True)
    assert env.support_drive is None and env.mount_drive is None
    np.testing.assert_array_equal(plug.pose.p, pose.p)
    np.testing.assert_array_equal(plug.pose.q, pose.q)
    np.testing.assert_array_equal(plug.velocity, velocity)
    np.testing.assert_array_equal(plug.angular_velocity, angular)
    for _ in range(20):
        env.step_action([])
    assert plug.pose.p[2] < initial[2]-.2
    assert env.grasp_monitor.state == "dropped"
    assert not env.grasp_monitor.external_support


def test_usb_only_reset_removes_actor_and_constraint_then_allows_new_creation(isolated_env):
    env, _, _, _ = isolated_env
    before = len(env._scene.get_all_actors())
    env.spawn_cable()
    old = env.plug.id
    env.clear_objects()
    env._scene.step()  # Native actor deletion completes at the step boundary.
    assert len(env._scene.get_all_actors()) == before
    assert env.plug is None and env.support_drive is None
    assert env.grasp_monitor.state == "not_created"
    env.clear_objects()
    assert env.spawn_cable()
    assert env.plug.id != old
    assert len(env._scene.get_all_actors()) == before+1


def test_failed_world_support_creation_cleans_the_dynamic_usb(isolated_env, monkeypatch):
    env, _, _, _ = isolated_env
    scene = env._scene
    before = len(scene.get_all_actors())

    class BrokenSupportScene:
        def __getattr__(self, name):
            return getattr(scene, name)

        def create_drive(self, *args):
            raise RuntimeError("injected_world_drive_creation_failure")

    with monkeypatch.context() as patch:
        patch.setattr(env, "_scene", BrokenSupportScene())
        with pytest.raises(RuntimeError, match="injected_world_drive"):
            env.spawn_cable()
    scene.step()
    assert len(scene.get_all_actors()) == before
    assert env.plug is None and env.support_drive is None
    assert env.grasp_monitor.state == "failed"
    assert "creation_failed" in env.grasp_monitor.reason
    assert env.spawn_cable()
    assert env.grasp_monitor.state == "supported"
    assert env.grasp_monitor.external_support


def test_failed_release_reports_failure_and_keeps_support(isolated_env, monkeypatch):
    env, _, _, _ = isolated_env
    env.spawn_cable()
    scene, support = env._scene, env.support_drive

    class BrokenReleaseScene:
        def __getattr__(self, name):
            return getattr(scene, name)

        def remove_drive(self, drive):
            raise RuntimeError("injected_world_drive_removal_failure")

    with monkeypatch.context() as patch:
        patch.setattr(env, "_scene", BrokenReleaseScene())
        with pytest.raises(RuntimeError, match="injected_world_drive"):
            env.release_support(manual=True)
    assert env.support_drive is support
    result = env.grasp_monitor.snapshot()
    assert result["state"] == "failed" and result["external_support"]
    assert "support_release_failed" in result["reason"]


def test_mesh_preparation_failure_cleans_cache_before_retry(isolated_env, monkeypatch):
    env, _, usb, _ = isolated_env
    before = len(env._scene.get_all_actors())

    def broken_copy(*args):
        raise OSError("injected_mesh_copy_failure")

    with monkeypatch.context() as patch:
        patch.setattr(usb.shutil, "copyfile", broken_copy)
        with pytest.raises(OSError, match="injected_mesh"):
            env.spawn_cable()
    assert len(env._scene.get_all_actors()) == before
    assert env._usb_collision_cache is None
    assert env.grasp_monitor.state == "failed"
    assert env.spawn_cable()
    assert env.grasp_monitor.external_support


def test_creation_uses_future_preparation_pose_not_current_tcp(isolated_env):
    env, sapien, _, _ = isolated_env
    target = sapien.Pose([.5, .2, 1.2])
    env.spawn_cable({"left": target})
    np.testing.assert_allclose(env.plug.pose.p, [.5, .2, 1.212], atol=1.e-6)
    baseline = env.grasp_monitor.snapshot()["creation_world_pose"]
    for _ in range(3):
        env.step_action([])
    assert env.grasp_monitor.snapshot()["world_translation_m"] < 1.e-6
    assert env.grasp_monitor.snapshot()["creation_world_pose"] == baseline
    assert not env.spawn_cable({"left": sapien.Pose([2, 2, 2])})
    np.testing.assert_allclose(env.plug.pose.p, [.5, .2, 1.212], atol=1.e-6)


@pytest.mark.parametrize("scene_name", ["usb", "trunking"])
def test_simulation_constructor_skips_backend_preparation(isolated_env, monkeypatch, scene_name):
    env, _, usb, trunking = isolated_env

    def init_sim(sim, assets, *, env_factory, **kwargs):
        assert env_factory.keywords["load_cable"] is False
        sim.env = env

    monkeypatch.setattr(usb.Simulation, "__init__", init_sim)
    constructor = usb.UsbCableSimulation if scene_name == "usb" else trunking.TrunkingCableSimulation
    sim = constructor(SimpleNamespace(initial_positions={}), cable_config=env.cable_config,
                      cable_solver="mpm", load_cable=False)
    assert sim.env.cable is None
    assert sim.env._cable_engine is None
