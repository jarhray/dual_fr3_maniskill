"""Policy/geometry checks. These do not represent actual robot insertion."""
from pathlib import Path
import numpy as np
import pytest
from dual_fr3_maniskill.insertion import InsertionPolicy, InsertionLimits
from dual_fr3_maniskill.insertion_geometry import *


def observation(**changes):
    values = dict(depth_m=-.008, lateral_error_m=0., orientation_error_rad=0.,
        feedback_available=True, grasp_valid=True, temporary_support=False,
        resistance_N=0., lateral_N=0., torque_Nm=0., relative_speed_m_s=0., relative_angular_rad_s=0.)
    values.update(changes)
    return values


def test_actual_grasp_transform_and_direction():
    from transforms3d.euler import euler2mat
    base, tcp, usb = np.eye(4), np.eye(4), np.eye(4)
    base[:3, 3] = [.105, .72, .005]
    tcp[:3, :3] = euler2mat(.3, .5, -.2)
    tcp[:3, 3] = [.5, .3, .2]
    usb[:3, :3] = euler2mat(.4, .6, -.1)
    usb[:3, 3] = tcp[:3, 3]+[.01, -.02, .008]
    relative = np.linalg.inv(tcp)@usb
    goal = tcp_goal(base, tcp, usb, .010)
    measured = measure(base, goal@relative)
    assert measured['depth_m'] == pytest.approx(.010)
    assert measured['lateral_error_m'] < 1e-15
    assert measured['orientation_error_rad'] < 3e-8
    outside = tcp_goal(base, tcp, usb, -.008)
    np.testing.assert_allclose(goal[:3,3]-outside[:3,3], [-.018,0,0], atol=1e-15)


def test_shared_wall_adjustment_preserves_other_geometry_and_uses_original_sides():
    # Shift beyond an original wall so classifying against the new centre would
    # invert a side. Both consumers must still use the CAD centre for selection.
    vertices = np.array([[0., .015, .06246], [-.011, .0199, .07466],
                         [0., .01, .05], [-.0748, .03, .0888]])
    original = vertices.copy()
    hole = np.array([0., .020, .070])
    clearance = [.0002, .0007]
    adjusted = adjust_hole_vertices(vertices, clearance, hole)
    np.testing.assert_array_equal(vertices, original)
    np.testing.assert_array_equal(adjusted[:, 0], original[:, 0])
    np.testing.assert_array_equal(adjusted[2:], original[2:])
    np.testing.assert_allclose(adjusted[:2, 1:] - hole[1:],
                               [[-.002425, -.0067], [.002425, .0067]], atol=1e-17)


def test_prisms_do_not_seal_hole_and_preserve_back():
    from scipy.spatial import ConvexHull
    root = Path(__file__).parents[1]/'meshes'
    parts = socket_parts(root/'usb_base_collision.stl')
    def inside(point):
        return any(np.all(ConvexHull(m.vertices).equations[:,:3]@point+ConvexHull(m.vertices).equations[:,3] < -1e-9) for m in parts)
    for x in np.linspace(-.0109, -.0001, 20):
        for y in [-.002225, .002225]:
            for z in [-.006, .006]:
                assert not inside(np.array([x,y,z])+HOLE)
    assert inside(np.array([-.012,0,0])+HOLE)
    assert inside(np.array([-.005,.003,0])+HOLE)
    report = geometry_report(root)
    assert report['socket_convex_shape_count'] == 37
    assert report['target_depth_m'] == pytest.approx(.010)
    assert all(p.is_watertight for p in usb_parts(root/'USB1.stl'))


def test_feedback_continuously_changes_velocity():
    p = InsertionPolicy({})
    p.begin(0., observation())
    for i in range(1,26):
        p.update(i*.02, observation(depth_m=-.008+i*.00002))
    no_contact = p.speed
    for i in range(26,51):
        p.update(i*.02, observation(depth_m=-.008+i*.00002,resistance_N=2.))
    assert 0 < p.speed < no_contact*.4


@pytest.mark.parametrize('change,state', [({'resistance_N':6.},'overload'),
    ({'lateral_N':3.},'overload'), ({'torque_Nm':.1},'overload'),
    ({'grasp_valid':False},'slip_or_drop'), ({'temporary_support':True},'slip_or_drop'),
    ({'feedback_available':False},'feedback_unavailable'),
    ({'depth_m':.0102},'blocked'), ({'orientation_error_rad':.03},'blocked')])
def test_failure_never_success(change,state):
    p=InsertionPolicy({}); p.begin(0.,observation())
    assert p.update(.02,observation(**change)) == 0
    assert p.state == state and not p.insertion_success and not p.retention_active


def test_success_requires_measured_depth_dwell_and_low_velocity():
    p=InsertionPolicy({}); p.begin(0.,observation())
    p.update(.1,observation(depth_m=.01,relative_speed_m_s=.002))
    assert not p.insertion_success
    p.update(.2,observation(depth_m=.01))
    p.update(.49,observation(depth_m=.01))
    assert not p.insertion_success
    p.update(.51,observation(depth_m=.01))
    assert p.insertion_success and not p.retention_active
    assert not p.begin(.52,observation())
    p.reset(); assert p.state == 'not_started' and not p.insertion_success


def test_stall_and_sim_timeout():
    p=InsertionPolicy({}); p.begin(0.,observation())
    p.update(1.6,observation()); assert p.state == 'blocked'
    p=InsertionPolicy({}); p.begin(0.,observation())
    p.update(46.,observation()); assert p.state == 'timeout'


def test_retention_preconditions():
    from dual_fr3_maniskill.insertion_scene import SocketInsertion
    obj=SocketInsertion.__new__(SocketInsertion)
    obj.policy=InsertionPolicy({}); obj.retention=None; obj.env=None
    with pytest.raises(RuntimeError,match='recorded success'): obj.retain()


def test_retained_monitor_preserves_record_and_no_false_drop():
    from dual_fr3_maniskill.usb_grasp import UsbGraspMonitor
    from types import SimpleNamespace
    pose=SimpleNamespace(p=np.zeros(3),q=np.array([1.,0,0,0]))
    m=UsbGraspMonitor(); m.created(0.,pose)
    m.observe(.1,pose,pose,dict(a=1.,b=1.)); m.observe(.3,pose,pose,dict(a=1.,b=1.))
    m.released(.3,pose,pose); m.observe(.7,pose,pose,dict(a=1.,b=1.))
    m.socket_supported(.7)
    for t in [1.,2.]: m.observe(t,pose,pose,dict(a=0.,b=0.))
    assert m.state == 'gripper_released' and m.pre_retention_snapshot is not None


def test_retention_failure_cannot_authorize_gripper_release():
    from types import SimpleNamespace
    from dual_fr3_maniskill.insertion_scene import SocketInsertion
    from dual_fr3_maniskill.insertion_bridge import InsertionBridge
    from dual_fr3_maniskill.sapien_compat import sapien
    obj = SocketInsertion.__new__(SocketInsertion)
    obj.policy = InsertionPolicy({})
    obj.policy.insertion_success = True
    obj.policy.state = 'inserted_unretained'
    obj.retention = None
    obj.config = {}
    obj.base = SimpleNamespace(pose=sapien.Pose())
    def fail(*args):
        raise RuntimeError('injected_constraint_creation_failure')
    obj.env = SimpleNamespace(plug=SimpleNamespace(pose=sapien.Pose(),
        velocity=np.zeros(3), angular_velocity=np.zeros(3)), _scene=SimpleNamespace(create_drive=fail))
    obj.observe = lambda: observation(depth_m=.01)
    assert not obj.retain()
    assert obj.policy.state == 'retention_failed'
    assert obj.policy.insertion_success and not obj.policy.retention_active
    adapter = InsertionBridge.__new__(InsertionBridge)
    adapter.bridge = SimpleNamespace(sim=SimpleNamespace(env=SimpleNamespace(insertion=obj)))
    response = adapter.command('released', SimpleNamespace())
    assert not response.success and not obj.policy.grippers_released


def test_cancel_restores_drive_ownership_without_modifying_measured_state():
    from types import SimpleNamespace
    from dual_fr3_maniskill.insertion_bridge import InsertionBridge
    obj = InsertionBridge.__new__(InsertionBridge)
    obj.owner = True
    measured = np.arange(7)/10
    obj.bridge = SimpleNamespace(sim=SimpleNamespace(target=np.ones(7), positions=measured),
        desired={}, reserved={('left','insertion')}, arm_indices={'left':list(range(7))})
    obj.relinquish()
    np.testing.assert_array_equal(obj.bridge.sim.target, measured)
    assert not obj.owner and not obj.bridge.reserved
    obj.relinquish()  # repeat cancellation is harmless


def test_wall_watchdog_stops_even_when_simulation_clock_has_not_advanced():
    import time
    from types import SimpleNamespace
    from dual_fr3_maniskill.insertion_bridge import InsertionBridge
    p = InsertionPolicy({})
    p.begin(10., observation())
    scene = SimpleNamespace(policy=p, observe=lambda: observation())
    obj = InsertionBridge.__new__(InsertionBridge)
    obj.owner = True
    obj.last_heartbeat = time.monotonic()-11.
    obj.bridge = SimpleNamespace(sim=SimpleNamespace(env=SimpleNamespace(insertion=scene),
        time=10., target=np.ones(7), positions=np.zeros(7)), failure=None,
        arm_indices={'left':list(range(7))}, desired={}, reserved={('left','insertion')})
    obj.before_tick()
    assert p.state == 'timeout' and p.reason == 'client_wall_clock_heartbeat_lost'
    assert not obj.owner


def test_failed_controller_initialization_does_not_leave_active_policy():
    from types import SimpleNamespace
    from dual_fr3_maniskill.insertion_bridge import InsertionBridge
    from dual_fr3_maniskill.sapien_compat import sapien
    def fail_model():
        raise RuntimeError('pinocchio model unavailable')
    policy = InsertionPolicy({})
    scene = SimpleNamespace(policy=policy, observe=lambda: observation())
    agent = SimpleNamespace(links={'left_fr3_hand_tcp': SimpleNamespace(pose=sapien.Pose())},
                            robot=SimpleNamespace(create_pinocchio_model=fail_model))
    obj = InsertionBridge.__new__(InsertionBridge)
    obj.owner = False
    obj.bridge = SimpleNamespace(sim=SimpleNamespace(time=0., env=SimpleNamespace(
        insertion=scene, agent=agent)), reserved=set(), arms={}, pending_arms={}, grippers={}, failure=None)
    policy.right_return_complete = True
    obj.require_open = lambda side: None
    obj.require_ready = lambda side: None
    response = obj.command('start', SimpleNamespace())
    assert not response.success
    assert policy.state == 'blocked' and 'initialization_failed' in policy.reason
    assert not obj.owner and not obj.bridge.reserved and not policy.retention_active


def test_native_installation_filters_only_fixed_pairs_and_restores_on_close():
    from types import SimpleNamespace
    from dual_fr3_maniskill.sapien_compat import sapien
    from dual_fr3_maniskill.insertion_scene import SocketInsertion
    engine = sapien.Engine()
    scene = engine.create_scene()
    scene.set_timestep(.001)
    fixtures = {}
    originals = []
    for name in ('plate', 'trunking'):
        builder = scene.create_actor_builder()
        builder.add_box_collision(half_size=[.2, .2, .2])
        fixtures[name] = builder.build_static(name)
        originals.extend((shape, list(shape.get_collision_groups())) for shape in fixtures[name].get_collision_shapes())
    env = SimpleNamespace(_scene=scene, _renderer=None, fixtures=fixtures,
        agent=SimpleNamespace(links={'left_fr3_link0':fixtures['trunking']}),
        cable_config={'insertion':{'xy_m':[0.,0.]}})
    socket = SocketInsertion(env)
    try:
        assert socket.base.pose.p[2] == pytest.approx(.010)
        for _ in range(5):
            scene.step()
        assert not scene.get_contacts()
        # A third object penetrating the socket is still a real contact.
        builder = scene.create_actor_builder()
        builder.add_box_collision(half_size=[.005]*3)
        probe = builder.build_static('usb_contact_probe')
        probe.set_pose(sapien.Pose([-.04, .015, .05]))
        for _ in range(5):
            scene.step()
        pairs = {frozenset((c.actor0.name,c.actor1.name)) for c in scene.get_contacts()}
        assert frozenset(('usb_socket','usb_contact_probe')) in pairs
        assert not any('usb_socket' in pair and ('plate' in pair or 'trunking' in pair) for pair in pairs)
    finally:
        socket.close()
    for shape, groups in originals:
        assert list(shape.get_collision_groups()) == groups
    socket.close()


def test_right_release_and_return_require_measured_open_ready_and_valid_left_grasp():
    from types import SimpleNamespace as NS
    from dual_fr3_maniskill.insertion_bridge import InsertionBridge
    from dual_fr3_maniskill.sapien_compat import sapien
    policy = InsertionPolicy({})
    scene = NS(policy=policy, config={}, observe=lambda: observation(), base=NS(pose=sapien.Pose()))
    obj = InsertionBridge.__new__(InsertionBridge); obj.owner = False
    positions = np.zeros(9)
    names = ['r'+str(i) for i in range(7)]
    obj.bridge = NS(sim=NS(env=NS(insertion=scene), positions=positions, velocities=np.zeros(9),
        indices={'right_fr3_finger_joint1':7,'right_fr3_finger_joint2':8}), failure=None,
        reserved=set(), arms={}, pending_arms={}, grippers={}, arm_indices={'right':np.arange(7)},
        arm_names={'right':names}, assets=NS(initial_positions=dict.fromkeys(names,0.)))
    call = lambda op: obj.command(op, NS())
    assert not call('start').success
    assert not call('target').success
    assert call('right_release').success
    positions[7] = .015
    assert not call('right_released').success  # second finger still closed
    positions[8] = .015
    assert call('right_released').success
    positions[0] = .1
    assert not call('right_returned').success
    positions[0] = 0.
    scene.observe = lambda: observation(grasp_valid=False)
    assert not call('right_returned').success
    scene.observe = lambda: observation()
    assert call('right_returned').success
    assert policy.right_return_complete and not policy.insertion_success and not policy.grippers_released
    policy.state = 'complete'
    assert call('right_returned').success and policy.state == 'complete'
    policy.reset()
    assert not policy.right_return_complete and not policy.right_gripper_released
