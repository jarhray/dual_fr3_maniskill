"""Force signs, moment origins, unequal substeps, missing data and native loads."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from dual_fr3_maniskill.forces import ForceCollector, LoadWindow


def pose(p=(0., 0., 0.), q=(1., 0., 0., 0.)):
    return SimpleNamespace(p=np.array(p, dtype=float), q=np.array(q, dtype=float))


def actor(name, index, p=(0., 0., 0.)):
    return SimpleNamespace(name=name, id=index, pose=pose(p), _ptr=index)


def environment():
    tcp = actor("left_fr3_hand_tcp", 1)
    finger1 = actor("left_fr3_leftfinger", 2)
    finger2 = actor("left_fr3_rightfinger", 3)
    right = actor("right_fr3_hand_tcp", 4)
    fixture = actor("socket", 5)
    contacts = []
    env = SimpleNamespace(agent=SimpleNamespace(links={a.name: a for a in (tcp, right, finger1, finger2)}),
        fixtures={fixture.name: fixture}, _scene=SimpleNamespace(get_contacts=lambda: contacts), cable=None)
    return env, contacts, tcp, finger1, finger2, fixture


def contact(a, b, impulse, position=(0., 0., 0.)):
    return SimpleNamespace(actor0=a, actor1=b, points=[SimpleNamespace(
        impulse=np.array(impulse, dtype=float), normal=np.array(impulse)/np.linalg.norm(impulse),
        position=np.array(position, dtype=float))])


def test_unequal_steps_use_total_impulse_and_preserve_peak_and_frame():
    window = LoadWindow("tcp")
    rotated = pose(q=(np.sqrt(.5), 0, 0, np.sqrt(.5)))
    # World +Y is sensor +X; a force at world +X creates sensor +Z moment.
    window.add(rotated, [0, .02, 0], [1, 0, 0], [0, 0, 0], "test", "socket", .02)
    window.end_step(.002)
    window.add(rotated, [0, .03, 0], [1, 0, 0], [0, 0, 0], "test", "socket", .03)
    window.end_step(.001)
    result = window.result(.003)
    np.testing.assert_allclose(result["force_N"], [50/3, 0, 0], atol=1.e-12)
    np.testing.assert_allclose(result["torque_Nm"], [0, 0, 50/3], atol=1.e-12)
    assert result["peak_substep_force_N"] == pytest.approx(30)
    assert result["normal_load_N"] == pytest.approx(50/3)


def test_opposing_fingers_do_not_hide_local_load_and_contact_order():
    env, contacts, tcp, f1, f2, base = environment()
    contacts.extend([contact(f1, base, [0, .01, 0]), contact(base, f2, [0, .01, 0])])
    collector = ForceCollector(env, ["socket"])
    collector.begin(1.)
    collector.sample(.002)
    result = collector.finish(1.002)["sensors"]
    np.testing.assert_allclose(result["left/fixtures"]["force_N"], [0, 0, 0])
    assert result["left/fixtures"]["normal_load_N"] == pytest.approx(10.)
    assert result["fingers/left_fr3_leftfinger/fixtures"]["force_N"][1] == pytest.approx(5.)
    assert result["fingers/left_fr3_rightfinger/fixtures"]["force_N"][1] == pytest.approx(-5.)
    assert result["left/usb_base"]["objects"] == ["socket"]
    assert result["left/fixtures"]["missing_components"] == ["tangential_contact_impulses"]


def test_unavailable_differs_from_no_contact_and_missing_window():
    env, *_ = environment()
    collector = ForceCollector(env)
    collector.begin(0.)
    collector.sample(.01)
    snapshot = collector.finish(.01)
    result = snapshot["sensors"]
    assert result["left/cable"]["force_N"] is None
    assert result["left/usb_base"]["force_N"] is None
    assert result["left/fixtures"]["force_N"] == [0., 0., 0.]
    json.dumps(snapshot, allow_nan=False)
    collector.begin(.01)
    collector.sample(.002)
    assert not collector.finish(.02)["sensors"]["left/fixtures"]["available"]


def test_proxy_contacts_and_usb_anchor_are_separate_load_paths():
    env, contacts, tcp, f1, _, _ = environment()
    plug, rope, proxy = actor("plug", 10), actor("rope_0", 11), actor("finger_proxy", 12)
    env.cable = SimpleNamespace(solver="rope_actor", plug=plug, links=[rope], link_ids={11},
                               proxy_links={12: f1}, root_joint="fixed")
    contacts.append(contact(rope, proxy, [0, -.004, 0]))
    reader = lambda *_: dict(force=[2., 0, 0], torque=[0, 0, 0], origin=[0, 0, 0], awake=True)
    collector = ForceCollector(env, constraint_reader=reader)
    collector.begin(0.)
    collector.sample(.002)
    result = collector.finish(.002)["sensors"]
    np.testing.assert_allclose(result["left/cable"]["force_N"], [0., 2., 0])
    np.testing.assert_allclose(result["usb/cable"]["force_N"], [2., 0., 0])
    np.testing.assert_allclose(result["fingers/left_fr3_leftfinger/cable"]["force_N"], [0., 2., 0])
    assert result["fingers/left_fr3_leftfinger/usb"]["force_N"] == [0., 0., 0.]


def test_usb_only_contacts_available_without_cable_and_keep_force_on_each_body():
    from dual_fr3_maniskill.usb_grasp import UsbGraspMonitor
    env, contacts, tcp, f1, f2, _ = environment()
    env.plug = actor("plug", 10)
    env.load_cable = False
    env.support_drive = object()
    env.grasp_monitor = UsbGraspMonitor()
    env.grasp_monitor.created(0.)
    contacts.extend([contact(f1, env.plug, [0, .01, 0]),
                     contact(env.plug, f2, [0, .01, 0])])
    collector = ForceCollector(env)
    collector.begin(0.)
    collector.sample(.002)
    snapshot = collector.finish(.002)
    result = snapshot["sensors"]
    assert snapshot["solver"] == "disabled"
    assert snapshot["usb_grasp"]["external_support"]
    assert snapshot["temporary_support"]["active"]
    assert snapshot["temporary_support"]["world_drive_count"] == 1
    assert snapshot["temporary_support"]["cable_particle_count"] == 0
    assert snapshot["initial_layout"] is None
    assert result["left/cable"]["reasons"] == ["cable_disabled"]
    assert result["usb/cable"]["force_N"] is None
    assert result["usb/usb_base"]["force_N"] is None
    assert result["fingers/left_fr3_leftfinger/usb"]["force_N"][1] == pytest.approx(5.)
    assert result["fingers/left_fr3_rightfinger/usb"]["force_N"][1] == pytest.approx(-5.)
    assert result["usb/fingers/left_fr3_leftfinger"]["force_N"][1] == pytest.approx(-5.)
    assert result["usb/fingers/left_fr3_rightfinger"]["force_N"][1] == pytest.approx(5.)
    np.testing.assert_allclose(result["left/usb"]["force_N"], [0., 0., 0.])
    json.dumps(snapshot, allow_nan=False)


def test_mpm_usb_anchor_is_not_assumed_to_be_transmitted_to_tcp():
    env, _, tcp, f1, _, _ = environment()
    env.plug = actor("plug", 10)
    env.cable = SimpleNamespace(solver="mpm", plug=env.plug, guide=None)
    collector = ForceCollector(env)
    collector.begin(0.)
    collector.mpm_reaction(env.plug, [0, 0, .003, 0, .004, 0], [1, 0, 0])
    collector.sample(.002)
    result = collector.finish(.002)["sensors"]
    np.testing.assert_allclose(result["usb/cable"]["force_N"], [0, 2., 0])
    np.testing.assert_allclose(result["left/cable"]["force_N"], [0., 0., 0.])


def test_mpm_reaction_preserves_spatial_order_and_torque_origin():
    env, _, tcp, f1, _, _ = environment()
    env.cable = SimpleNamespace(solver="mpm", plug=None, guide=None)
    collector = ForceCollector(env)
    collector.begin(0.)
    collector.mpm_reaction(f1, [0, 0, .003, 0, .004, 0], [1, 0, 0])
    collector.sample(.002)
    result = collector.finish(.002)["sensors"]
    np.testing.assert_allclose(result["left/cable"]["force_N"], [0, 2., 0])
    np.testing.assert_allclose(result["left/cable"]["torque_Nm"], [0, 0, 3.5])
    assert result["fingers/left_fr3_leftfinger/cable"]["normal_load_N"] is None


def test_nonfinite_load_is_unavailable_and_json_safe():
    window = LoadWindow("tcp")
    window.add(pose(), [np.nan, 0, 0], [0, 0, 0], [0, 0, 0], "bad", "cable", None)
    window.end_step(.001)
    value = window.result(.001)
    assert not value["available"] and value["force_N"] is None
    json.dumps(value, allow_nan=False)


def test_nonfinite_sensor_pose_does_not_break_failure_json():
    env, _, tcp, _, _, _ = environment()
    tcp.pose.p[0] = np.nan
    collector = ForceCollector(env)
    collector.begin(0.)
    collector.sample(.002)
    snapshot = collector.finish(.002)
    assert snapshot["frame_poses_world"][tcp.name] is None
    assert tcp.name in snapshot["unavailable_frame_poses"]
    assert not snapshot["sensors"]["left/fixtures"]["available"]
    assert "nonfinite_sensor_pose" in snapshot["sensors"]["left/fixtures"]["reasons"]
    json.dumps(snapshot, allow_nan=False)


@pytest.fixture
def native_scene():
    from dual_fr3_maniskill.sapien_compat import sapien
    # Match the rope suite: SAPIEN's first Engine owns process-wide tolerances.
    engine = sapien.Engine(tolerance_length=.1, tolerance_speed=.2)
    settings = sapien.SceneConfig()
    settings.gravity = [0, 0, 0]
    scene = engine.create_scene(settings)
    scene.set_timestep(.001)
    yield sapien, scene


@pytest.mark.parametrize("reverse", [False, True])
def test_native_constraint_known_force_torque_and_pair_order(native_scene, reverse):
    from dual_fr3_maniskill._rope_physx import read_pair_constraint
    sapien, scene = native_scene
    receiver = scene.create_actor_builder().build_kinematic("receiver")
    builder = scene.create_actor_builder()
    builder.set_mass_and_inertia(1., sapien.Pose(), [.1]*3)
    load = builder.build("load")
    load.set_pose(sapien.Pose([1, 0, 0]))
    if reverse:
        drive = scene.create_drive(load, sapien.Pose(), receiver, sapien.Pose([1, 0, 0]))
    else:
        drive = scene.create_drive(receiver, sapien.Pose([1, 0, 0]), load, sapien.Pose())
    drive.lock_motion(True, True, True, True, True, True)
    for _ in range(100):
        load.add_force_torque([0, 2, 0], [0, 0, .3])
        scene.step()
    result = read_pair_constraint(receiver._ptr, load._ptr)
    np.testing.assert_allclose(result["force"], [0, 2, 0], atol=1.e-4)
    np.testing.assert_allclose(result["torque"], [0, 0, .3], atol=1.e-4)
    window = LoadWindow("tcp")
    window.add(pose(), np.array(result["force"])*.001, result["origin"],
               np.array(result["torque"])*.001, "constraint", "load", None)
    window.end_step(.001)
    assert window.result(.001)["torque_Nm"][2] == pytest.approx(2.3, abs=1.e-4)
    scene.remove_drive(drive)
    with pytest.raises(ValueError, match="No constraint"):
        read_pair_constraint(receiver._ptr, load._ptr)


def test_native_finger_fixture_contact_supports_known_weight(native_scene):
    sapien, scene = native_scene
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=[1, 1, .05])
    base = builder.build_static("socket")
    base.set_pose(sapien.Pose([0, 0, -.05]))
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=[.02]*3)
    builder.set_mass_and_inertia(1., sapien.Pose(), [.1]*3)
    finger = builder.build("left_fr3_leftfinger")
    finger.set_pose(sapien.Pose([0, 0, .02]))
    tcp = scene.create_actor_builder().build_kinematic("left_fr3_hand_tcp")
    env = SimpleNamespace(_scene=scene, agent=SimpleNamespace(links={tcp.name: tcp, finger.name: finger}),
                          fixtures={base.name: base}, cable=None)
    collector = ForceCollector(env, ["socket"])
    for _ in range(300):
        finger.add_force_torque([0, 0, -9.81], [0, 0, 0])
        scene.step()
    collector.begin(0.)
    for _ in range(20):
        finger.add_force_torque([0, 0, -9.81], [0, 0, 0])
        scene.step()
        collector.sample(.001)
    result = collector.finish(.02)["sensors"]
    np.testing.assert_allclose(result["left/fixtures"]["force_N"], [0, 0, 9.81], atol=.02)
    assert result["fingers/left_fr3_leftfinger/fixtures"]["normal_load_N"] == pytest.approx(9.81, abs=.02)
    assert result["left/usb_base"]["objects"] == ["socket"]
