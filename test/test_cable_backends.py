"""Backend selection and real PhysX rope checks; no renderer/CUDA required."""
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys

import numpy as np
import pytest
import yaml

from dual_fr3_maniskill.cable.backends import create_cable, prepare_backend, validate_solver
from dual_fr3_maniskill.cable.model import load_config, load_geometry_config
from dual_fr3_maniskill.cable.guide import SlidingGuide

ROOT = Path(__file__).resolve().parents[1]


def config():
    c = load_config(ROOT/"config/trunking_cable.yaml", solver="rope_actor")
    c["cable"]["initial_layout"] = "straight"
    return c


def test_rope_config_and_geometry_do_not_require_mpm(tmp_path):
    c = config()
    del c["mpm"]
    for key in ("particle_spacing", "young_modulus", "axial_young_modulus", "axial_iterations", "yield_stress", "poisson_ratio"):
        del c["cable"][key]
    path = tmp_path/"rope.yaml"
    path.write_text(yaml.safe_dump(c))
    assert load_config(path, solver="rope_actor")["rope_actor"]["links"] == 151
    assert load_geometry_config(path)["cable"]["diameter"] == c["cable"]["diameter"]
    with pytest.raises(ValueError, match="Missing mpm"):
        load_config(path)
    prepare_backend("rope_actor", c, 500)
    with pytest.raises(ValueError, match="cable_solver"):
        validate_solver("rod_typo")


@pytest.mark.parametrize("key,value", [("links", 0), ("links", 257), ("links", 2.5),
    ("joint_stiffness", -1.), ("joint_damping", float("nan")), ("bend_limit_deg", 180.),
    ("solver_iterations", True), ("solver_iterations", 256),
    ("solver_velocity_iterations", 256), ("adaptive_timestep", "false"), ("adaptive_timestep", 0),
    ("solver_type", "unknown"), ("root_joint", "unknown"),
    ("collision_geometry", "unknown"), ("constraint_tolerance", 0.),
    ("max_contact_travel", 0.), ("max_contact_travel", -.00001),
    ("max_contact_travel", True), ("max_contact_travel", float("nan")),
    ("max_contact_travel", float("inf")), ("max_contact_travel", .00076),
    ("max_stretch_ratio", 0.), ("max_stretch_ratio", -0.01),
    ("max_stretch_ratio", True), ("max_stretch_ratio", float("nan")),
    ("max_stretch_ratio", float("inf")), ("max_stretch_ratio", 1.1)])
def test_invalid_rope_settings(tmp_path, key, value):
    c = config()
    c["rope_actor"][key] = value
    path = tmp_path/"invalid.yaml"
    path.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError):
        load_config(path, solver="rope_actor")


def test_rope_factory_does_not_import_mpm_or_warp():
    subprocess.run([sys.executable, "-c", '''
import sys
class RejectMPM:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('warp', 'mpm', 'mani_skill2') or fullname.endswith('.mpm_cable'):
            raise AssertionError('Rope backend imported MPM: ' + fullname)
sys.meta_path.insert(0, RejectMPM())
from dual_fr3_maniskill.cable.rope_actor import RopeActorCable
from dual_fr3_maniskill.cable.backends import shader_directory
assert shader_directory('rope_actor') == 'ibl'
'''], check=True, capture_output=True, text=True)


def test_recorded_3mm_rim_contact_uses_private_fixture_shell(native_scene, tmp_path):
    """The shared 1 mm fixture shell missed this overlap even in a fresh scene."""
    import hashlib
    import json
    from ament_index_python.packages import get_package_share_directory
    from dual_fr3_maniskill.assets import convert_stl_to_glb

    sapien, scene, _, env = native_scene
    recorded = json.loads((ROOT/"test/data/rope_trunking_3mm_20260915.json").read_text())
    mesh = Path(get_package_share_directory("dual_fr3_moveit_config"))/"meshes/Trunking.STL"
    assert hashlib.sha256(mesh.read_bytes()).hexdigest() == recorded["mesh_sha256"]
    builder = scene.create_actor_builder()
    builder.add_nonconvex_collision_from_file(str(convert_stl_to_glb(mesh, tmp_path)))
    fixture = builder.build_static("trunking")
    fixture.set_pose(sapien.Pose(recorded["fixture_pose"]["position_m"],
                                recorded["fixture_pose"]["quaternion_wxyz"]))
    env.fixtures[fixture.name] = fixture
    original = fixture.get_collision_shapes()[0]
    original.contact_offset = .001
    groups = original.get_collision_groups()
    actors_before = len(scene.get_all_actors())
    c = config()
    c["cable"]["diameter"] = recorded["diameter_m"]
    cable = create_cable(env, c, solver="rope_actor")
    try:
        assert cable.lengths[2] == pytest.approx(recorded["segment_length_m"])
        proxy, source = cable.fixture_proxies[0]
        shape = proxy.get_collision_shapes()[0]
        assert source is fixture
        assert original.contact_offset == pytest.approx(.001)
        assert original.get_collision_groups() == groups
        assert shape.contact_offset == pytest.approx(c["rope_actor"]["contact_offset"])
        assert shape.rest_offset == original.rest_offset
        assert not proxy.get_visual_bodies()
        np.testing.assert_array_equal(shape.geometry.vertices, original.geometry.vertices)
        np.testing.assert_array_equal(shape.geometry.indices, original.geometry.indices)
        np.testing.assert_array_equal(shape.geometry.scale, original.geometry.scale)
        # The private copy contacts the rope, while the robot retains the original.
        private = shape.get_collision_groups()
        rope = cable.links[2].get_collision_shapes()[0].get_collision_groups()
        assert not (groups[0] & rope[1] or groups[1] & rope[0])
        assert private[0] & rope[1]
        assert not (private[0] & 1 or private[1] & 1)
        for drive in [cable.anchor, *cable.joints]:
            scene.remove_drive(drive)
        cable.anchor = None
        cable.joints.clear()
        for actor in cable.links:
            actor.set_pose(sapien.Pose([5., 0., 0.]))
            actor.set_velocity([0., 0., 0.])
            actor.set_angular_velocity([0., 0., 0.])
        actor = cable.links[2]
        state = recorded["capsule"]
        actor.set_pose(sapien.Pose(state["position_m"], state["quaternion_wxyz"]))
        scene.set_timestep(.00025)
        scene.step()
        contacts = [c for c in scene.get_contacts() if actor.id in {c.actor0.id, c.actor1.id}]
        assert contacts
        assert all(proxy.id in {c.actor0.id, c.actor1.id} for c in contacts)
        depth = max(-p.separation for contact in contacts for p in contact.points)
        assert abs(depth-recorded["expected_depth_m"]) < .00002
        cable._collect_contacts()
        cable._audit_contacts()
        assert "trunking" in cable.contacted_bodies
        assert cable.max_penetration_contact["object"] == "trunking"
        # Static test/scene edits must also reach the cable collider.
        bounds_before = cable._world_obstacle_bounds()
        old_position = fixture.pose.p.copy()
        fixture.set_pose(sapien.Pose([0., 0., -1.]))
        box_index = next(i for i, (actor, _, _) in enumerate(cable._obstacle_boxes) if actor is proxy)
        np.testing.assert_allclose(cable._world_obstacle_bounds()[box_index],
            bounds_before[box_index]+fixture.pose.p-old_position, atol=1.e-7)
        cable.step(.00025)
        np.testing.assert_array_equal(proxy.pose.p, fixture.pose.p)
    finally:
        cable.close()
    scene.step()  # SAPIEN finalizes pending actor removals at the next step.
    assert len(scene.get_all_actors()) == actors_before
    assert original.contact_offset == pytest.approx(.001)
    assert original.get_collision_groups() == groups


@pytest.fixture
def native_scene(request):
    pytest.importorskip("sapien.core")
    from dual_fr3_maniskill.sapien_compat import sapien
    r = config()["rope_actor"]
    engine = sapien.Engine(tolerance_length=r["engine_tolerance_length"], tolerance_speed=r["engine_tolerance_speed"])
    settings = sapien.SceneConfig()
    settings.enable_ccd = False
    settings.enable_tgs = r["solver_type"] == "tgs"
    settings.enable_pcm = True
    settings.solver_iterations = r["solver_iterations"]
    settings.solver_velocity_iterations = r["solver_velocity_iterations"]
    settings.gravity = [0., 0., -9.81 if getattr(request, "param", False) else 0.]
    scene = engine.create_scene(settings)
    scene.set_timestep(.002)
    plug = scene.create_actor_builder().build_kinematic("usb_cable_demo_plug")
    plug.set_pose(sapien.Pose([0., 0., .2]))
    env = SimpleNamespace(_scene=scene, sim_timestep=.002,
                          agent=SimpleNamespace(links={plug.name: plug}), fixtures={})
    yield sapien, scene, plug, env


def advance(cable, scene, steps):
    for _ in range(steps):
        cable.step(.002)
        scene.step()
        cable.follow_plug()


def test_cumulative_stretch_catches_many_small_joint_errors_without_repairing_state(native_scene):
    sapien, scene, plug, env = native_scene
    c = config()
    c["rope_actor"].update(links=151, constraint_tolerance=.002, max_stretch_ratio=.001)

    def straight(cfg, s, rotation, translation):
        return translation+np.asarray(s)[:, None]*np.array([1., 0., 0.])

    cable = create_cable(env, c, solver="rope_actor", layout=straight)
    try:
        cable._check_constraints()
        for i, actor in enumerate(cable.links):
            actor.set_pose(sapien.Pose(actor.pose.p+[i*.00005, 0., 0.], actor.pose.q))
        before = np.array([actor.pose.p for actor in cable.links])
        diagnostics = cable.diagnostics()
        assert diagnostics["max_joint_gap_m"] < .000051
        assert diagnostics["attachment_error_m"] < .000001
        assert diagnostics["cumulative_stretch_m"] == pytest.approx(.0075, abs=.000001)
        with pytest.raises(RuntimeError, match="cumulative stretch"):
            cable._check_constraints()
        np.testing.assert_array_equal(before, [actor.pose.p for actor in cable.links])
        # Omitted/disabled setting preserves legacy per-joint guard behavior.
        del c["rope_actor"]["max_stretch_ratio"]
        cable._check_constraints()
    finally:
        cable.close()


@pytest.mark.parametrize("native_scene", [True], indirect=True)
@pytest.mark.parametrize("root_joint", ["fixed", "spherical"])
def test_root_bends_at_usb_exit_without_detaching_and_reset_preserves_joint(native_scene, root_joint):
    from transforms3d.quaternions import quat2mat
    from dual_fr3_maniskill.cable.rope_diagnostics import RopeSubstepTrace
    sapien, scene, plug, env = native_scene
    c = config()
    c["cable"]["length"] = .04
    c["rope_actor"].update(links=6, root_joint=root_joint, joint_damping=0.)
    cable = create_cable(env, c, solver="rope_actor")
    try:
        initial_axis = quat2mat(cable.links[0].pose.q)[:, 0]
        advance(cable, scene, 100)
        axis = quat2mat(cable.links[0].pose.q)[:, 0]
        angle = np.arccos(np.clip(np.dot(initial_axis, axis), -1., 1.))
        assert angle > .1 if root_joint == "spherical" else angle < .001
        assert cable.attachment_error() < .00005
        if root_joint == "spherical":
            np.testing.assert_allclose(cable.centerline[0],
                (plug.pose*sapien.Pose(c["usb"]["attachment"])).p, atol=.00005)
        trace = RopeSubstepTrace()
        trace.record(cable, .2)
        assert trace.tail[-1]["attachment_error_m"] == cable.attachment_error()
        assert cable.diagnostics()["root_joint"] == root_joint
        plug.set_pose(sapien.Pose([.01, 0., .3], [.92387953, 0., 0., .38268343]))
        cable.reset()
        assert cable.root_joint == root_joint
        assert cable.attachment_error() < 1.e-6
        advance(cable, scene, 50)
        assert cable.attachment_error() < .00005
        # Moving the whole chain preserves intersegment gaps but breaks its
        # USB attachment. A rotating centre must not mask a detached endpoint.
        for actor in cable.links:
            actor.set_pose(sapien.Pose(actor.pose.p+[.005, 0., 0.], actor.pose.q))
        with pytest.raises(RuntimeError, match="joint/attachment"):
            cable._check_constraints()
    finally:
        cable.close()


def aperture_actor(sapien, scene, *, material=None):
    # A real, asymmetric bore: Y in [-4, 4] mm, Z in [-2, 8] mm.
    # The TCP is deliberately not at the aperture's geometric centre.
    builder = scene.create_actor_builder()
    for center, half in [([0, -.006, .003], [.012, .002, .009]),
                         ([0, .006, .003], [.012, .002, .009]),
                         ([0, 0, -.004], [.012, .004, .002]),
                         ([0, 0, .010], [.012, .004, .002])]:
        builder.add_box_collision(pose=sapien.Pose(center), half_size=half, material=material)
    return builder.build_kinematic("right_fr3_hand_tcp")


def test_proxy_motion_has_linear_and_angular_velocity(native_scene):
    native = pytest.importorskip("dual_fr3_maniskill._rope_physx")
    from dual_fr3_maniskill.cable.kinematic import advance_proxy
    sapien, scene, proxy, _ = native_scene
    dt = .002
    link = SimpleNamespace(pose=sapien.Pose([.1, .2, .3]),
                           velocity=np.array([.2, -.1, .3]),
                           angular_velocity=np.array([0., 0., 2.]),
                           cmass_local_pose=sapien.Pose([.01, 0., 0.]))
    advance_proxy(proxy, link, dt, native.set_kinematic_target)
    scene.set_timestep(dt)
    scene.step()
    # v_COM = v_origin + omega x (COM - origin).
    origin_velocity = np.array([.2, -.12, .3])
    np.testing.assert_allclose(proxy.pose.p, link.pose.p+origin_velocity*dt, atol=1.e-7)
    np.testing.assert_allclose(proxy.velocity, origin_velocity, atol=2.e-5)
    np.testing.assert_allclose(proxy.angular_velocity, link.angular_velocity, atol=2.e-5)


def test_depenetration_probe_changes_bias_limit_without_clipping_motion(native_scene):
    native = pytest.importorskip("dual_fr3_maniskill._rope_physx")
    _, scene, _, _ = native_scene
    builder = scene.create_actor_builder()
    builder.add_sphere_collision(radius=.01)
    actor = builder.build("bias_limit_probe")
    actor.set_damping(0., 0.)
    native.set_max_depenetration_velocity(actor._ptr, .02)
    assert native.get_max_depenetration_velocity(actor._ptr) == pytest.approx(.02)
    actor.set_velocity([.3, 0., 0.])
    scene.set_timestep(.001)
    scene.step()
    assert actor.velocity[0] == pytest.approx(.3)
    for value in (0., -1., float("nan"), float("inf")):
        with pytest.raises(ValueError, match="positive and finite"):
            native.set_max_depenetration_velocity(actor._ptr, value)
    static = scene.create_actor_builder().build_static("static_probe")
    with pytest.raises(ValueError, match="rigid body"):
        native.set_max_depenetration_velocity(static._ptr, .02)
    scene.remove_actor(static)
    scene.remove_actor(actor)


@pytest.mark.parametrize("dt", [.0002, .001])
def test_rotating_proxy_keeps_target_over_repeated_native_steps(native_scene, dt):
    from dual_fr3_maniskill.cable.kinematic import KinematicContactProxy, advance_proxy, load_kinematic_target
    from dual_fr3_maniskill.cable.rope_actor import RopeActorCable
    sapien, scene, _, _ = native_scene
    initial_articulations = len(scene.get_all_articulations())
    builder = scene.create_articulation_builder()
    root = builder.create_link_builder()
    root.set_name("rotating_proxy")
    root.add_box_collision(half_size=[.002]*3)
    proxy = KinematicContactProxy(scene, builder)
    source = SimpleNamespace(pose=sapien.Pose([.8, .5, .2], [0, 1, 0, 0]),
        velocity=np.array([.01, 0., -.02]), angular_velocity=np.array([.01, .03, .02]),
        cmass_local_pose=sapien.Pose())
    set_target = load_kinematic_target()
    scene.set_timestep(dt)
    cable = RopeActorCable.__new__(RopeActorCable)
    try:
        for i in range(100):
            source.pose = sapien.Pose(np.array([.8, .5, .2])+i*dt*source.velocity, [0, 1, 0, 0])
            expected = advance_proxy(proxy, source, dt, set_target)
            scene.step()
            np.testing.assert_allclose(proxy.pose.p, expected.p, atol=1.e-7)
            np.testing.assert_allclose(proxy.pose.q, expected.q, atol=1.e-7)
            np.testing.assert_allclose(proxy.angular_velocity, source.angular_velocity, atol=2.e-5)
            np.testing.assert_allclose(proxy.velocity, source.velocity, atol=5.e-4)
            cable._proxy_targets = [(proxy, expected)]
            cable._check_proxy_targets()
        proxy.set_pose(sapien.Pose([0., 0., 0.]))
        with pytest.raises(RuntimeError, match="invalid contact boundary"):
            cable._check_proxy_targets()
    finally:
        proxy.close()
        scene.step()
    assert len(scene.get_all_articulations()) == initial_articulations


def test_proxy_reaction_preserves_impulse_when_timestep_changes(native_scene):
    from dual_fr3_maniskill.cable.rope_actor import RopeActorCable
    sapien, scene, _, _ = native_scene
    builder = scene.create_actor_builder()
    builder.set_mass_and_inertia(1., sapien.Pose(), [.1, .1, .1])
    receiver = builder.build("reaction_receiver")
    cable = RopeActorCable.__new__(RopeActorCable)
    cable.links, cable.proxies, cable.guide = [], [], None
    cable.contacted_bodies = set()
    # A contact solved at 1 ms is applied during a subsequent 4 ms substep.
    cable._dt = .001
    cable._pending_reactions = [(receiver, np.array([.002, 0., 0.]), receiver.pose.p)]
    cable.step(.004)
    scene.set_timestep(.004)
    scene.step()
    np.testing.assert_allclose(receiver.velocity, [.002, 0., 0.], atol=1.e-8)
    assert not cable._pending_reactions


def test_endpoint_speed_measures_velocity_at_the_endpoint():
    from dual_fr3_maniskill.cable.rope_actor import RopeActorCable
    cable = RopeActorCable.__new__(RopeActorCable)
    cable._axes = np.array([[1., 0., 0.]])
    cable.lengths = np.array([2.])
    speed = cable._endpoint_speeds(np.array([[1., 0., 0.]]), np.array([[0., 0., 1.]]))
    np.testing.assert_allclose(speed, [np.sqrt(2.)])


@pytest.mark.parametrize("filename", ["trunking_cable.yaml", "usb_cable.yaml"])
def test_travel_default_is_independent_of_acceptance_tolerance(tmp_path, filename):
    c = load_config(ROOT/"config"/filename, solver="rope_actor")
    # Both shipped configurations retain their previous 0.05 mm travel cap.
    assert c["rope_actor"]["max_contact_travel"] == .5*c["cable"]["penetration_tolerance"]
    del c["rope_actor"]["max_contact_travel"]
    c["cable"]["penetration_tolerance"] = .0003
    path = tmp_path/"legacy.yaml"
    path.write_text(yaml.safe_dump(c))
    assert load_config(path, solver="rope_actor")["rope_actor"]["max_contact_travel"] == .00005


def test_tolerance_changes_acceptance_but_not_timestep_or_motion(native_scene):
    sapien, scene, plug, env = native_scene
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=[.05, 2., .01])
    floor = builder.build_static("floor")
    c = config()
    # 20 um surface gap: within both tested contact budgets at either diameter.
    floor.set_pose(sapien.Pose([0., -.75, .2-c["cable"]["diameter"]/2-.00002-.01]))
    env.fixtures[floor.name] = floor
    c["rope_actor"]["links"] = 13
    c["cable"]["length"] = .08
    cable = create_cable(env, c, solver="rope_actor")
    try:
        cap = c["rope_actor"]["max_contact_travel"]
        for _ in range(5):
            for actor in cable.links:
                actor.set_velocity([.2, 0., 0.])
            c["cable"]["penetration_tolerance"] = .0001
            dt = cable.suggested_timestep(.001)
            c["cable"]["penetration_tolerance"] = .0003
            assert cable.suggested_timestep(.001) == dt
            assert dt < .001  # Exercise the active contact travel bound.
            c["rope_actor"]["max_contact_travel"] = cap/2
            assert cable.suggested_timestep(.001) == pytest.approx(dt/2)
            c["rope_actor"]["max_contact_travel"] = cap
            scene.set_timestep(dt)
            cable.step(dt)
            scene.step()
            cable.follow_plug()
        # Independent acceptance test at an analytically known 0.15 mm depth.
        # Do not step physics or mutate the cable between the two checks.
        i = 5
        actor = cable.links[i]
        bottom = min((actor.pose.p+sign*cable.lengths[i]/2*cable._axes[i])[2] for sign in (-1, 1))
        floor.set_pose(sapien.Pose([0., -.75, bottom-cable.radius+.00015-.01]))
        cable._contact_candidates = {(floor, floor.get_collision_shapes()[0]): {actor}}
        c["cable"]["penetration_tolerance"] = .0001
        with pytest.raises(RuntimeError, match="penetration"):
            cable.check_contacts()
        positions = cable.centerline.copy()
        velocities = np.array([a.velocity for a in cable.links])
        c["cable"]["penetration_tolerance"] = .0003
        cable.check_contacts()
        np.testing.assert_array_equal(cable.centerline, positions)
        np.testing.assert_array_equal([a.velocity for a in cable.links], velocities)
    finally:
        cable.close()


def test_motion_bound_resolves_free_chain_and_tightens_before_contact(native_scene):
    sapien, scene, _, env = native_scene
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=[.1, 2., .01])
    floor = builder.build_static("floor")
    floor.set_pose(sapien.Pose([0., -.75, .169]))  # 30 mm below center, 20 mm surface gap
    env.fixtures["floor"] = floor
    cable = create_cable(env, config(), solver="rope_actor")
    try:
        for actor in cable.links:
            actor.set_velocity([0., 0., -2.])
        assert cable.suggested_timestep(.001) < .001  # Joint motion also matters away from contact.
        limits = []
        for gap in (.0021, .0019, .00002):
            floor.set_pose(sapien.Pose([0., -.75, .2-cable.radius-gap-.01]))
            limits.append(cable.suggested_timestep(.001))
        assert limits[0] > limits[1] > limits[2]
        assert limits[0]/limits[1] < 1.2  # No 1 ms -> ~25 us switch at the old proximity threshold.
    finally:
        cable.close()


def test_substep_trace_refreshes_geometry_and_keeps_bounded_failure_state(native_scene):
    from dual_fr3_maniskill.cable.rope_diagnostics import RopeSubstepTrace
    import io
    import json
    sapien, scene, _, env = native_scene
    c = config()
    c["rope_actor"]["links"] = 13
    c["cable"]["length"] = .08
    cable = create_cable(env, c, solver="rope_actor")
    stream = io.StringIO()
    trace = RopeSubstepTrace(capacity=2, stream=stream)
    try:
        for step in range(3):
            advance(cable, scene, 1)
            positions = cable.centerline.copy()
            velocities = np.array([a.velocity for a in cable.links])
            cable.max_depth = 1.  # Stale diagnostic value must not survive capture.
            row = trace.record(cable, (step+1)*.002)
            assert row["max_penetration_m"] == 0.
            np.testing.assert_array_equal(cable.centerline, positions)
            np.testing.assert_array_equal([a.velocity for a in cable.links], velocities)
        rows = [json.loads(line) for line in stream.getvalue().splitlines()]
        assert len(rows) == 3 and rows[-1]["step"] == 3
        assert "actors" not in rows[-1]
        assert len(trace.tail) == 2 and len(trace.tail[-1]["actors"]) == 13
        assert trace.summary()["substeps"] == 3
        cable.links[-1].set_velocity([11., 0., 0.])
        row = trace.record(cable, .006, audit=False)
        assert row["geometry_audited"] and row["max_speed_segment_index"] == 12
        assert row["max_endpoint_speed_m_s"] > c["rope_actor"]["max_speed"]
        with pytest.raises(RuntimeError, match="endpoint speed"):
            cable._check_constraints()
    finally:
        cable.close()


def trace_env(env, cable):
    from dual_fr3_maniskill.usb_grasp import UsbGraspMonitor
    env.cable = cable
    env.load_cable = True
    env.rope_trace = None
    env.cable_solver = "rope_actor"
    env._sim_steps_per_control = 2
    env.rigid_substeps = 1
    env.agent.set_action = lambda action: None
    env.agent.before_simulation_step = lambda: None
    env.agent.joints = {}
    env.agent.robot = SimpleNamespace(get_qpos=lambda: np.array([]), get_qvel=lambda: np.array([]))
    env.grasp_monitor = UsbGraspMonitor()
    env._grasp_time = 0.
    # This cable-only fixture has a kinematic USB anchor and no robot fingers.
    # Advance observer time without fabricating physical grasp observations.
    def observe_grasp(dt):
        env._grasp_time += dt
    env._observe_grasp = observe_grasp
    return env


@pytest.mark.parametrize("adaptive", [False, True])
def test_accuracy_mode_controls_substeps_but_preserves_observations(native_scene, monkeypatch, adaptive):
    from dual_fr3_maniskill.scenes.usb_cable import UsbCableEnv
    _, _, _, env = native_scene
    c = config()
    c["rope_actor"].update(links=13, adaptive_timestep=adaptive)
    c["cable"]["length"] = .08
    cable = create_cable(env, c, solver="rope_actor")
    trace_env(env, cable)
    timesteps, observations = [], []
    native_step, native_follow = cable.step, cable.follow_plug

    def step(dt):
        timesteps.append(dt)
        native_step(dt)

    def follow():
        native_follow()
        observations.append(cable.steps)

    def limit(maximum):
        if not adaptive:
            pytest.fail("Fixed mode must not perform the adaptive proximity scan")
        return maximum/4

    monkeypatch.setattr(cable, "step", step)
    monkeypatch.setattr(cable, "follow_plug", follow)
    monkeypatch.setattr(cable, "suggested_timestep", limit)
    # A leftover fine timestep must not throttle an explicitly fixed mode.
    cable._dt = .000125
    cable.steps = 1
    try:
        UsbCableEnv.step_action(env, np.array([]))
        assert sum(timesteps) == pytest.approx(.004)
        assert timesteps == pytest.approx([.00025]*16 if adaptive else [.002]*2)
        assert len(observations) == len(timesteps)
        assert env._grasp_time == pytest.approx(.004)
        cable.check_contacts()
        assert cable.diagnostics()["adaptive_timestep"] is adaptive
        # Preview mode retains the control-boundary geometry/constraint guard.
        cable.links[-1].set_pose(type(cable.links[-1].pose)([1., 1., 1.]))
        with pytest.raises(RuntimeError, match="constraint exceeded tolerance"):
            cable.check_contacts()
    finally:
        cable.close()


def test_trace_survives_support_release_before_first_guide_observation(native_scene, tmp_path):
    from dual_fr3_maniskill.cable.rope_diagnostics import RopeRunRecorder
    sapien, scene, _, env = native_scene
    c = config()
    c["rope_actor"].update(links=13, adaptive_timestep=False)
    c["cable"]["length"] = .08
    cable = create_cable(env, c, solver="rope_actor")
    trace_env(env, cable)
    # Preparation keeps the crossing observer inactive while the real hand
    # approaches. Removal of the world support precedes the next native step.
    tcp = scene.create_actor_builder().build_kinematic("right_fr3_hand_tcp")
    points = cable.centerline
    tcp.set_pose(sapien.Pose((points[6]+points[7])/2, [np.sqrt(.5), 0., 0., np.sqrt(.5)]))
    cable.guide = SlidingGuide(tcp, c, 6.5)
    env.preparation_tcp_poses = {"right": tcp.pose}
    env.support_drive = object()
    errors = []
    recorder = RopeRunRecorder(tmp_path, metadata={}, on_error=errors.append)
    try:
        assert not cable.guide_diagnostics()["available"]
        recorder.capture("control", env, action=np.array([]))
        env.support_drive = None
        before = cable.centerline.copy()
        recorder.capture("control", env, action=np.array([]))
        assert not errors
        assert recorder.last_control["state"]["cable"]["guide"]["reason"] == "waiting_for_first_guide_observation"
        np.testing.assert_array_equal(cable.centerline, before)
        cable._update_guide()
        recorder.capture("control", env, action=np.array([]))
        assert not errors
        assert recorder.last_control["state"]["cable"]["guide"]["guide_arc_length_m"] > 0
    finally:
        recorder.capture("close")
        cable.guide = None
        cable.close()


def test_force_observer_keeps_rope_state_and_adaptive_schedule_identical(native_scene):
    from dual_fr3_maniskill.forces import ForceCollector
    from dual_fr3_maniskill.scenes.usb_cable import UsbCableEnv
    sapien, scene, plug, env = native_scene
    tcp = scene.create_actor_builder().build_kinematic("left_fr3_hand_tcp")
    tcp.set_pose(plug.pose)
    env.agent.links[tcp.name] = tcp
    c = config()
    c["rope_actor"]["links"] = 13
    c["cable"]["length"] = .08
    results = []
    for enabled in (False, True):
        cable = create_cable(env, c, solver="rope_actor")
        trace_env(env, cable)
        observer = ForceCollector(env) if enabled else None
        env.force_collector = observer
        timesteps = []
        native_step = cable.step

        def step(dt):
            timesteps.append(dt)
            native_step(dt)

        cable.step = step
        try:
            for i in range(3):
                cable.links[-1].add_force_at_point([0., 0., -1.e-5], cable.links[-1].pose.p)
                if observer:
                    observer.begin(i*.004)
                UsbCableEnv.step_action(env, np.array([]))
                if observer:
                    row = observer.finish((i+1)*.004)
                    assert row["sensors"]["left/cable"]["available"]
                    assert row["physics_substeps"] > 0
            results.append((cable.centerline.copy(), np.array([a.velocity for a in cable.links]), timesteps))
        finally:
            env.force_collector = None
            cable.close()
    for first, second in zip(*results):
        np.testing.assert_array_equal(first, second)


def test_run_recorder_keeps_native_stepping_identical(native_scene, tmp_path):
    import json
    from dual_fr3_maniskill.cable.rope_diagnostics import RopeRunRecorder
    from dual_fr3_maniskill.scenes.usb_cable import UsbCableEnv
    _, _, _, env = native_scene
    c = config()
    c["rope_actor"]["links"] = 13
    c["cable"]["length"] = .08
    results = []
    for enabled in (False, True):
        cable = create_cable(env, c, solver="rope_actor")
        trace_env(env, cable)
        timesteps = []
        native_step = cable.step

        def step(dt):
            timesteps.append(dt)
            native_step(dt)

        cable.step = step
        errors = []
        recorder = RopeRunRecorder(tmp_path, metadata={}, on_error=errors.append, capacity=2) if enabled else None
        env.rope_trace = recorder
        try:
            for _ in range(3):
                cable.links[-1].add_force_at_point([0., 0., -1.e-5], cable.links[-1].pose.p)
                UsbCableEnv.step_action(env, np.array([]))
                cable.check_contacts()
            results.append((cable.centerline.copy(), np.array([a.velocity for a in cable.links]), timesteps))
            if recorder:
                assert not errors
                assert recorder.time == pytest.approx(.012)
                assert recorder.trace.summary()["geometry_audits"] == 0
                assert recorder.trace.summary()["max_penetration_m"] is None
                assert len(recorder.trace.tail) == 2
                assert recorder.last_control["state"]["time_s"] == pytest.approx(.008)
                assert recorder.last_control["state"]["cable"]["step"] < recorder.trace.tail[0]["step"]
                rows = [json.loads(s) for s in (recorder.directory / "substeps.jsonl").read_text().splitlines()]
                assert sum(row["dt_s"] for row in rows) == pytest.approx(.012)
                assert len(rows) == cable.steps
                assert all(0 < row["dt_s"] <= row["suggested_timestep_s"] for row in rows)
                assert all(row["predicted_contact_travel_m"] <= c["rope_actor"]["max_contact_travel"]
                           for row in rows if row["predicted_contact_travel_m"] is not None)
                assert all(row["predicted_motion_budget_fraction"] <= 1.+1.e-12 for row in rows)
                assert (recorder.directory / "initial_001.json").exists()
        finally:
            if recorder:
                recorder.capture("close")
            cable.close()
    np.testing.assert_array_equal(results[0][0], results[1][0])
    np.testing.assert_array_equal(results[0][1], results[1][1])
    assert results[0][2] == results[1][2]
    assert np.linalg.norm(results[0][1]) > 0.


def test_recorder_tracks_preparation_before_spawn(native_scene, tmp_path):
    import json
    from dual_fr3_maniskill.cable.rope_diagnostics import RopeRunRecorder
    from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableEnv
    from dual_fr3_maniskill.scenes.usb_cable import UsbCableEnv
    _, _, _, env = native_scene
    trace_env(env, None)
    errors = []
    recorder = RopeRunRecorder(tmp_path, metadata={}, on_error=errors.append)
    env.rope_trace = recorder
    c = config()
    c["rope_actor"]["links"] = 13
    c["cable"]["length"] = .08
    try:
        TrunkingCableEnv.step_action(env, np.array([]))
        env.cable = create_cable(env, c, solver="rope_actor")
        UsbCableEnv.step_action(env, np.array([]))
        rows = [json.loads(s) for s in (recorder.directory / "controls.jsonl").read_text().splitlines()]
        assert not rows[0]["cable_present"] and rows[0]["epoch"] == 0
        assert rows[1]["cable_present"] and rows[1]["epoch"] == 1
        assert rows[1]["time_s"] == pytest.approx(.004)
        assert recorder.time == pytest.approx(.008) and not errors
        initial = json.loads((recorder.directory / "initial_001.json").read_text())
        assert initial["time_s"] == pytest.approx(.004)
    finally:
        recorder.capture("close")
        if env.cable is not None:
            env.cable.close()


@pytest.mark.parametrize("failure_point", ["follow_plug", "control_guard", "suggested_timestep"])
def test_bridge_saves_substep_and_control_failures(native_scene, tmp_path, monkeypatch, failure_point):
    import json
    from dual_fr3_maniskill.cable.rope_diagnostics import RopeRunRecorder
    from dual_fr3_maniskill.cable.ros_bridge import UsbCableBridge, ManiSkillBridge
    from dual_fr3_maniskill.scenes.usb_cable import UsbCableEnv
    _, _, _, env = native_scene
    c = config()
    c["rope_actor"]["links"] = 13
    c["cable"]["length"] = .08
    cable = create_cable(env, c, solver="rope_actor")
    trace_env(env, cable)
    errors, messages = [], []
    recorder = RopeRunRecorder(tmp_path, metadata={}, on_error=errors.append)
    env.rope_trace = recorder
    env._viewer = None
    logger = SimpleNamespace(error=messages.append, info=messages.append)
    grasp_messages = []
    bridge = UsbCableBridge.__new__(UsbCableBridge)
    bridge.__dict__.update(sim=SimpleNamespace(env=env), failure=None, arms={}, grippers={},
        cable_solver="rope_actor", load_cable=True, _grasp_waiters=[], get_logger=lambda: logger,
        grasp_pub=SimpleNamespace(publish=lambda value: grasp_messages.append(json.loads(value.data))),
        _last_grasp_state=None,
        diag_pub=SimpleNamespace(publish=lambda value: None))
    reason = "injected " + failure_point

    def fail(*args):
        raise RuntimeError(reason)

    if failure_point != "control_guard":
        original = getattr(cable, failure_point)

        def failing_method(*args):
            original(*args)
            fail()

        monkeypatch.setattr(cable, failure_point, failing_method)

    def tick(self):
        UsbCableEnv.step_action(env, np.array([]))
        fail()

    monkeypatch.setattr(ManiSkillBridge, "tick", tick)
    try:
        UsbCableBridge.tick(bridge)
        assert bridge.failure == reason
        assert grasp_messages[-1]["state"] == "failed"
        report = json.loads((recorder.directory / "failure_001.json").read_text())
        assert report["reason"] == reason and not errors
        assert report["failure_state"]["cable"]["geometry_audited"]
        assert len(report["failure_state"]["cable"]["actors"]) == 13
        assert report["last_control"]["state"]["time_s"] == 0.
        assert report["last_control"]["state"]["cable"]["step"] == 0
        assert report["last_control"]["control"]["target"] == []
        if failure_point == "suggested_timestep":
            assert recorder.time == 0. and not report["tail"]
        else:
            assert report["tail"][-1]["step"] == cable.steps
            assert recorder.time > 0.
        assert any("Cable failure trace saved:" in message for message in messages)
        recorder.capture("reset", env)
        recorder.capture("control", env, action=[])
        assert (recorder.directory / "initial_002.json").exists()
        other = RopeRunRecorder(tmp_path, metadata={}, on_error=errors.append)
        assert other.directory != recorder.directory
        other.capture("close")
    finally:
        recorder.capture("close")
        cable.close()


def test_recorder_io_failure_does_not_stop_native_steps(native_scene, tmp_path):
    from dual_fr3_maniskill.cable.rope_diagnostics import RopeRunRecorder
    from dual_fr3_maniskill.scenes.usb_cable import UsbCableEnv
    _, _, _, env = native_scene
    c = config()
    c["rope_actor"]["links"] = 13
    c["cable"]["length"] = .08
    cable = create_cable(env, c, solver="rope_actor")
    trace_env(env, cable)
    errors = []
    recorder = RopeRunRecorder(tmp_path, metadata={}, on_error=errors.append)
    env.rope_trace = recorder
    recorder.substeps.close()  # Simulate a failed output stream.
    try:
        UsbCableEnv.step_action(env, np.array([]))
        assert cable.steps > 0 and recorder.time == pytest.approx(.004)
        assert len(errors) == 1 and "Cable trace substep failed" in errors[0]
        assert recorder.capture("failure", env, reason="original physics error").exists()
    finally:
        recorder.capture("close")
        cable.close()


@pytest.mark.parametrize("segments", [60, 151])
def test_real_chain_mass_motion_and_cleanup(native_scene, segments):
    sapien, scene, plug, env = native_scene
    c = config()
    c["rope_actor"]["links"] = segments
    cable = create_cable(env, c, solver="rope_actor")
    assert len(cable.links) == segments
    assert sum(link.mass for link in cable.links) == pytest.approx(.015*1.5)
    assert cable.lengths[0] == pytest.approx(.007)
    if segments == 151:
        assert cable.lengths.max() <= .010
    try:
        initial = cable.centerline.copy()
        for i in range(50):
            plug.set_pose(sapien.Pose([.00001*(i+1), 0., .2]))
            advance(cable, scene, 1)
        assert cable.centerline[0, 0] > initial[0, 0]+.0002
        assert cable.diagnostics()["max_joint_gap_m"] < .001
        cable.reset()
        assert np.linalg.norm(cable.centerline[0]-[.0005, -.02, .2]) < 1.e-6
    finally:
        cable.close()
        cable.close()
    scene.step()  # SAPIEN defers actor removal until the next fetch.
    assert len(scene.get_all_actors()) == 1
    assert not scene.get_all_articulations()


def test_convex_capsule_cooking_preserves_size_and_surface_accuracy(native_scene):
    _, _, _, env = native_scene
    c = config()
    c["rope_actor"].update(links=6, collision_geometry="convex_capsule")
    c["cable"]["length"] = .05
    cable = create_cable(env, c, solver="rope_actor")
    try:
        rng = np.random.default_rng(42)
        directions = rng.normal(size=(4096, 3))
        directions /= np.linalg.norm(directions, axis=1)[:, None]
        for i in (0, 1):
            shape, = cable.links[i].get_collision_shapes()
            geometry = shape.geometry
            vertices = np.asarray(geometry.vertices)*np.asarray(geometry.scale)
            assert len(vertices) == 194
            exact = cable.lengths[i]/2*np.abs(directions[:, 0])+cable.radius
            support = np.max(vertices @ directions.T, axis=0)
            assert np.min(exact-support) >= -1.e-7
            assert np.max(exact-support) < .028*cable.radius
            np.testing.assert_allclose(vertices.max(0),
                [cable.lengths[i]/2+cable.radius, cable.radius, cable.radius], atol=1.e-7)
        assert cable.diagnostics()["mass_kg"] == pytest.approx(.015*.05)
    finally:
        cable.close()


@pytest.mark.parametrize("collision_geometry", ["capsule", "convex_capsule"])
def test_guide_slides_across_segments_and_rejected_reset_keeps_state(native_scene, collision_geometry):
    sapien, scene, plug, env = native_scene
    right = aperture_actor(sapien, scene)
    q = [np.sqrt(.5), 0., 0., -np.sqrt(.5)]
    right.set_pose(sapien.Pose([0., -.6, .2], q))
    env.agent.links[right.name] = right
    guide = SlidingGuide(right, {}, 0.)
    c = config()
    c["rope_actor"]["collision_geometry"] = collision_geometry
    cable = create_cable(env, c, solver="rope_actor", guide=guide)
    try:
        initial = cable.centerline.copy()
        coordinate = guide.material_coordinate
        initial_arc = cable.guide_diagnostics()["guide_arc_length_m"]
        for i in range(200):
            right.set_pose(sapien.Pose([0., -.6-(i+1)*.0004, .2], q))
            advance(cable, scene, 1)
        assert guide.material_coordinate > coordinate+2
        measured = cable.guide_diagnostics()
        assert measured["guide_arc_length_m"]-initial_arc == pytest.approx(.08, abs=.0001)
        assert measured["guide_normal_load_N"] == 0.
        assert measured["guide_friction_axial_force_N"] is None
        assert measured["guide_force_report_scope"] == "normal_impulses_only"
        np.testing.assert_allclose(cable.centerline, initial, atol=1.e-4)
        # Move the bore sideways with clearance remaining. There must be no
        # artificial centring force or 1 mm radial-error rejection.
        right.set_pose(sapien.Pose([0., -.68, .197], q))
        advance(cable, scene, 5)
        cable.check_contacts()
        assert guide.radial_error > .002
        np.testing.assert_allclose(cable.centerline, initial, atol=1.e-4)
        before = cable.centerline.copy()
        right.set_pose(sapien.Pose([0., -2., .2], q))
        with pytest.raises(RuntimeError, match="no longer crosses"):
            cable.reset()
        np.testing.assert_array_equal(cable.centerline, before)
    finally:
        cable.close()


def test_aperture_allows_clearance_but_rejects_wall_and_escape(native_scene):
    from dual_fr3_maniskill.cable.aperture import MeshAperture
    sapien, scene, _, _ = native_scene
    right = aperture_actor(sapien, scene)
    aperture = MeshAperture(right, [(right, s) for s in right.get_collision_shapes()], .001, .0001)
    aperture.check([0., 0., .005])  # Five millimetres from TCP, still inside.
    assert aperture.inside
    assert aperture.wall_distance == pytest.approx(.003, abs=1.e-7)
    for point in ([0., .0035, .003], [0., 0., -.0015], [0., .03, .003]):
        with pytest.raises(RuntimeError, match="CAD boundary"):
            aperture.check(point)


def test_guide_audits_capsule_interiors_without_reported_contacts(native_scene):
    sapien, scene, _, env = native_scene
    right = aperture_actor(sapien, scene)
    right.set_pose(sapien.Pose([0., -.6, .2], [np.sqrt(.5), 0., 0., -np.sqrt(.5)]))
    env.agent.links[right.name] = right
    guide = SlidingGuide(right, {}, 0.)
    cable = create_cable(env, config(), solver="rope_actor", guide=guide)
    try:
        # The cable still crosses the middle of the aperture, but tilting the
        # finite bore puts its rim through a capsule away from the TCP plane.
        right.set_pose(sapien.Pose([0., -.6, .2], [np.cos(-np.pi/6), 0., 0., np.sin(-np.pi/6)]))
        cable._contact_candidates.clear()  # No native step/contact report yet.
        cable._check_aperture(cable.centerline)
        with pytest.raises(RuntimeError, match="contact penetration"):
            cable.check_contacts()
    finally:
        cable.close()


@pytest.mark.parametrize("native_scene", [True], indirect=True)
def test_passive_bore_support_and_friction(native_scene):
    sapien, scene, _, _ = native_scene
    final_speeds = []
    for friction in (0., .5):
        material = scene.create_physical_material(friction, friction, 0.)
        right = aperture_actor(sapien, scene, material=material)
        builder = scene.create_actor_builder()
        builder.add_capsule_collision(radius=.001, half_length=.0025, material=material)
        builder.set_mass_and_inertia(.00002, sapien.Pose(), [1.1e-8]*3)
        capsule = builder.build("free_cable_section")
        for actor in (right, capsule):
            for shape in actor.get_collision_shapes():
                shape.contact_offset = .00002
        scene.set_timestep(.0005)
        for _ in range(400):
            scene.step()
        # Gravity seats the cable on the lower wall, without any guide joint.
        assert capsule.pose.p[2] == pytest.approx(-.001, abs=.00005)
        assert abs(capsule.velocity[2]) < .001
        capsule.set_velocity([.05, 0., 0.])
        for _ in range(80):
            scene.step()
        final_speeds.append(abs(capsule.velocity[0]))
        scene.remove_actor(capsule)
        scene.remove_actor(right)
        scene.step()
    assert final_speeds[0] > .045
    assert final_speeds[1] < .005


@pytest.mark.parametrize("native_scene", [True], indirect=True)
@pytest.mark.parametrize("collision_geometry", ["capsule", "convex_capsule"])
@pytest.mark.parametrize("radius,offset", [(.001, 0.), (.0015, .0024)])
def test_research_finger_mesh_contacts_remain_stable(native_scene, tmp_path, collision_geometry, radius, offset):
    """Actual CAD, a free capsule, and gravity: no cable or guide joints.

    Legacy non-PCM contacts launch this capsule out of the bore. Check support
    and friction against the same geometry and physical scale as the MTC task.
    """
    import trimesh
    from transforms3d.quaternions import quat2mat
    from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
    from dual_fr3_maniskill.assets import prepare_assets
    from dual_fr3_maniskill.cable.mesh_contacts import finger_collision_meshes
    from dual_fr3_maniskill.cable.aperture import MeshAperture

    sapien, scene, _, _ = native_scene
    description, semantic = build_maniskill_description(scene="trunking_cable")
    assets = prepare_assets(description, semantic, tmp_path/"assets")
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(str(assets.urdf_path))
    positions = [0. if "finger_joint" in j.name else assets.initial_positions[j.name]
                 for j in robot.get_active_joints()]
    robot.set_qpos(positions)
    links = {link.name: link for link in robot.get_links()}
    tcp = links["right_fr3_hand_tcp"]
    meshes = []
    for name, rows in finger_collision_meshes(assets.urdf_path).items():
        if name.startswith("right_"):
            pose = tcp.pose.inv()*links[name].pose
            for vertices, faces in rows:
                meshes.append(trimesh.Trimesh(vertices=vertices @ quat2mat(pose.q).T+pose.p,
                    faces=faces.reshape(-1, 3), process=False))
    path = tmp_path/"closed_fingers.ply"
    trimesh.util.concatenate(meshes).export(path)
    scene.remove_articulation(robot)
    scene.step()

    speeds = []
    for friction in (0., .5):
        material = scene.create_physical_material(friction, friction, 0.)
        builder = scene.create_actor_builder()
        builder.add_nonconvex_collision_from_file(str(path), material=material)
        fingers = builder.build_kinematic("research_fingers")
        pose = sapien.Pose(q=[0., 1., 0., 0.])  # Gravity points along TCP +Z.
        fingers.set_pose(pose)
        builder = scene.create_actor_builder()
        length = (1.5-.007)/59
        mass = 1200*np.pi*radius**2*length
        if collision_geometry == "convex_capsule":
            from dual_fr3_maniskill.cable.capsule_mesh import write_capsule
            capsule_path = tmp_path/"supported_capsule.obj"
            write_capsule(capsule_path, length, radius)
            builder.add_collision_from_file(str(capsule_path), scale=[radius]*3, material=material)
        else:
            builder.add_capsule_collision(radius=radius, half_length=length/2, material=material)
        builder.set_mass_and_inertia(mass, sapien.Pose(), [1.1e-8]*3)
        capsule = builder.build("free_cable_section")
        capsule.set_pose(pose*sapien.Pose([0., 0., offset]))
        for actor in (fingers, capsule):
            for shape in actor.get_collision_shapes():
                shape.contact_offset = .00002
        guide = SlidingGuide(fingers, {"center_offset": [0., 0., offset]}, 0.)
        observer = MeshAperture(guide, [(fingers, s) for s in fingers.get_collision_shapes()],
                                radius, .0001)
        scene.set_timestep(.0001)
        peak_speed = 0.
        touched = False
        for _ in range(5000):
            scene.step()
            peak_speed = max(peak_speed, np.linalg.norm(capsule.velocity))
            touched |= bool(scene.get_contacts())
            assert peak_speed < .4
        crossing = (pose.inv()*capsule.pose).p
        crossing[0] = 0.
        observer.check((pose*sapien.Pose(crossing)).p)
        assert touched
        assert crossing[2] == pytest.approx(.00609-radius, abs=.0001)
        capsule.set_velocity([.05, 0., 0.])
        for _ in range(400):
            scene.step()
        speeds.append(abs(capsule.velocity[0]))
        scene.remove_actor(capsule)
        scene.remove_actor(fingers)
        scene.step()
    assert speeds[0] > .04
    assert speeds[1] < .005


def test_spawn_audits_capsule_interior_and_rolls_back(native_scene):
    sapien, scene, plug, env = native_scene
    c = config()
    # Small obstacle in the middle of a long capsule, away from both nodes.
    ds = (1.5-.007)/(c["rope_actor"]["links"]-1)
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=[.005, .001, .005])
    obstacle = builder.build_static("thin_wall")
    obstacle.set_pose(sapien.Pose([0., -.02-.007-10.5*ds, .2]))
    env.fixtures[obstacle.name] = obstacle
    shape = obstacle.get_collision_shapes()[0]
    groups = shape.get_collision_groups()
    with pytest.raises(ValueError, match="intersects thin_wall") as failure:
        create_cable(env, c, solver="rope_actor")
    assert "object=thin_wall" in str(failure.value)
    assert "segment_index=11 (0-based)" in str(failure.value)
    assert "frame=world" in str(failure.value)
    assert "obstacle_surface=" in str(failure.value)
    assert shape.get_collision_groups() == groups
    assert len(scene.get_all_actors()) == 2


@pytest.mark.parametrize("proxy", [False, True])
def test_penetration_diagnostics_identify_body_segment_and_world_points(native_scene, proxy):
    import json
    sapien, scene, _, env = native_scene
    builder = scene.create_actor_builder()
    builder.add_box_collision(pose=sapien.Pose([.001, 0., 0.]), half_size=[.004, .001, .004])
    obstacle = builder.build_static("rope_contact_right_fr3_leftfinger" if proxy else "trunking")
    obstacle.set_pose(sapien.Pose([.1, 0., .2]))
    env.fixtures[obstacle.name] = obstacle
    cable = create_cable(env, config(), solver="rope_actor")
    body = scene.create_actor_builder().build_kinematic("right_fr3_leftfinger") if proxy else obstacle
    try:
        index = 12
        center = cable.links[index].pose.p.copy()
        # Surface is 0.3 mm from the centreline. Include a local shape offset
        # and world translation; expected overlap follows the cable radius.
        if proxy:
            body.set_pose(sapien.Pose(center+[.0043, 0., .001], [np.sqrt(.5), 0., np.sqrt(.5), 0.]))
        else:
            body.set_pose(sapien.Pose(center+[.0033, 0., 0.]))
        if proxy:
            cable.proxy_links = {obstacle.id: body}
        shape = obstacle.get_collision_shapes()[0]
        cable._contact_candidates = {(obstacle, shape): {cable.links[index-1], cable.links[index]}}
        with pytest.raises(RuntimeError, match="contact penetration") as failure:
            cable.check_contacts()
        detail = cable.diagnostics()["max_penetration_contact"]
        assert detail["object"] == body.name
        assert detail["segment_index"] == index
        assert detail["segment_name"] == f"rope_actor_{index}"
        assert detail["segment_index_base"] == 0
        assert detail["frame_id"] == "world"
        assert detail["depth_m"] == pytest.approx(cable.radius-.0003, abs=1.e-7)
        np.testing.assert_allclose(detail["centerline_world_m"][::2], center[::2], atol=1.e-7)
        assert abs(detail["centerline_world_m"][1]-center[1]) <= .001
        np.testing.assert_allclose(np.array(detail["obstacle_surface_world_m"])-detail["centerline_world_m"],
                                   [.0003, 0., 0.], atol=1.e-7)
        text = str(failure.value)
        assert f"object={body.name}" in text
        assert "segment_index=12 (0-based)" in text
        assert "centerline=" in text and "obstacle_surface=" in text and "frame=world" in text
        json.dumps(detail, allow_nan=False)
        body.set_pose(sapien.Pose([.1, 0., .2]))
        cable.check_contacts()
        assert cable.diagnostics()["max_penetration_contact"] is None
        cable.reset()
        assert cable.diagnostics()["max_penetration_contact"] is None
    finally:
        cable.close()


@pytest.mark.parametrize("native_scene", [True], indirect=True)
def test_rope_falls_onto_floor_without_penetrating(native_scene):
    from dual_fr3_maniskill.cable.timestep import RopeStepSchedule

    sapien, scene, plug, env = native_scene
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=[.5, 2., .02])
    floor = builder.build_static("floor")
    floor.set_pose(sapien.Pose([0., -.75, -.02]))
    env.fixtures[floor.name] = floor
    cable = create_cable(env, config(), solver="rope_actor")
    touched = False
    initial_potential = sum(a.mass*9.81*a.pose.p[2] for a in cable.links)
    peak_energy = 0.
    try:
        for _ in range(50):
            schedule = RopeStepSchedule(.02, .001, cable._dt if cable.steps else None)
            while schedule.remaining_steps:
                limit = cable.suggested_timestep(.001)
                dt = schedule.next_step(limit)
                scene.set_timestep(dt)
                cable.step(dt)
                scene.step()
                cable.follow_plug()
                peak_energy = max(peak_energy, cable._energy_before_step)
                touched |= "floor" in cable.contacted_bodies
            cable.check_contacts()
        assert touched
        # The mount and floor are stationary; gravity is the only energy
        # source. Catch an energy burst even if its speed stays below 10 m/s.
        assert max(peak_energy, cable._kinetic_energy()) <= 1.05*initial_potential
        assert cable.centerline[:, 2].min() >= cable.radius-config()["cable"]["penetration_tolerance"]
    finally:
        cable.close()
