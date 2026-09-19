"""Entry-only control and optional per-step validation with live USB grasp."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
import time

import numpy as np
import pytest
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import Mesh
from geometry_msgs.msg import Pose

from dual_fr3_maniskill.usb.alignment import AlignmentLimits
from dual_fr3_maniskill.usb.collision_guard import LocalCollisionGuard


def guard():
    requests, futures = [], []
    def call(request):
        requests.append(request)
        future = Future()
        futures.append(future)
        return future
    obj = LocalCollisionGuard.__new__(LocalCollisionGuard)
    obj.bridge = NS(sim=NS(names=['left1', 'right1', 'finger1']))
    obj.limits = AlignmentLimits()
    obj.client = NS(service_is_ready=lambda: True, call_async=call)
    obj.scene_client = obj.client
    obj.attachment = AttachedCollisionObject(link_name='left_fr3_hand_tcp', object=CollisionObject(
        id='usb_cable_demo_plug', meshes=[Mesh()], mesh_poses=[Pose()]))
    obj.futures, obj.scene_future, obj.started = [], None, None
    return obj, requests, futures


def test_start_midpoint_end_and_measured_attachment_all_checked():
    obj, requests, futures = guard()
    mount = np.eye(4); mount[:3, 3] = [.001, .002, .013]
    obj.submit(np.zeros(3), np.array([.002, 0., 0.]), mount)
    assert len(requests) == 3 and not obj.poll()
    for index, request in enumerate(requests):
        state = request.robot_state
        assert request.group_name == '' and state.is_diff
        assert list(state.joint_state.position) == [index*.001, 0., 0.]
        pose = state.attached_collision_objects[0].object.mesh_poses[0]
        assert [pose.position.x, pose.position.y, pose.position.z] == list(mount[:3, 3])
        futures[index].set_result(NS(valid=True))
    assert obj.poll()


def test_stationary_entry_sends_one_state_query():
    obj, requests, futures = guard()
    obj.submit(np.zeros(3), np.zeros(3), np.eye(4))
    assert len(requests) == 1 and not obj.poll()
    futures[0].set_result(NS(valid=True))
    assert obj.poll()


@pytest.mark.parametrize('allow,pairs,accepted', [
    (False, [('usb_cable_demo_plug', 'usb_socket')], False),
    (True, [('usb_cable_demo_plug', 'usb_socket')], True),
    (True, [('usb_cable_demo_plug', 'usb_socket'), ('left_fr3_leftfinger', 'usb_socket')], False),
    (True, [], False), (False, [('left1', 'right1')], False)])
def test_only_insertion_allows_usb_socket_pair(allow, pairs, accepted):
    obj, _, futures = guard()
    obj.submit(np.zeros(3), np.ones(3)*.0001, np.eye(4), allow_socket=allow)
    for future in futures:
        future.set_result(NS(valid=False, constraint_result=[], contacts=[
            NS(contact_body_1=a, contact_body_2=b) for a, b in pairs]))
    if accepted:
        assert obj.poll()
    else:
        with pytest.raises(RuntimeError, match='local_path_collision'):
            obj.poll()


def test_interior_collision_cannot_be_hidden_by_valid_endpoints():
    obj, _, futures = guard()
    obj.submit(np.zeros(3), np.array([.002, 0., 0.]), np.eye(4))
    futures[0].set_result(NS(valid=True))
    futures[1].set_result(NS(valid=False, contacts=[NS(contact_body_1='left1', contact_body_2='plate')]))
    futures[2].set_result(NS(valid=True))
    with pytest.raises(RuntimeError, match='plate'):
        obj.poll()


def test_timeout_service_error_and_cancellation_fail_closed():
    obj, _, futures = guard()
    obj.submit(np.zeros(3), np.zeros(3), np.eye(4))
    obj.started = time.monotonic()-4.
    with pytest.raises(RuntimeError, match='timeout'):
        obj.poll()
    obj.cancel()
    assert all(future.cancelled() for future in futures)
    assert not obj.futures and obj.started is None
    obj.client.service_is_ready = lambda: False
    with pytest.raises(RuntimeError, match='unavailable'):
        obj.submit(np.zeros(3), np.zeros(3), np.eye(4))


def test_guard_requires_socket_and_unique_attached_usb_scene():
    obj, _, futures = guard()
    obj.prepare()
    assert not obj.ready()
    futures[-1].set_result(NS(scene=NS(world=NS(collision_objects=[]))))
    with pytest.raises(RuntimeError, match='missing_socket'):
        obj.ready()


@pytest.mark.parametrize('drift', ['joints', 'mount', 'rotation', None])
def test_late_result_for_changed_start_or_grasp_never_moves_arm(drift):
    from transforms3d.axangles import axangle2mat
    from dual_fr3_maniskill.usb.bridge import InsertionBridge
    from dual_fr3_maniskill.usb.insertion import InsertionPolicy
    policy = InsertionPolicy({})
    control = InsertionBridge.__new__(InsertionBridge)
    control.collision_guard = NS(poll=lambda: True)
    pose = NS(p=np.zeros(3), q=np.array([1., 0., 0., 0.]))
    goal = NS(p=np.array([.00002, 0., 0.]), q=pose.q)
    control.goal = pose
    start = np.zeros(3)
    control.pending_step = (start+.0001, goal, start.copy(), np.eye(4))
    mount = np.eye(4)
    if drift == 'joints': start[0] = .002
    if drift == 'mount': mount[1, 3] = .0001
    if drift == 'rotation': mount[:3, :3] = axangle2mat([1, 0, 0], .01)
    control.measured_mount = lambda: mount
    control.bridge = NS(sim=NS(positions=start, env=NS(insertion=NS(policy=policy))))
    applied = []
    control.apply_step = lambda *args: applied.append(args)
    control.finish_checked_step(alignment=True)
    assert bool(applied) is (drift is None)
    assert control.pending_step is None


def local_controller(mode='entry'):
    """Synthetic IK for command routing; actual physics has a separate check."""
    from transforms3d.quaternions import mat2quat
    from dual_fr3_maniskill.engine.sapien_compat import sapien
    from dual_fr3_maniskill.usb.bridge import InsertionBridge
    from dual_fr3_maniskill.usb.geometry import HOLE, TIP, USB_IN_SOCKET
    from dual_fr3_maniskill.usb.insertion import InsertionPolicy
    policy = InsertionPolicy(dict(local_collision_check=mode, speed_m_s=.002))
    obs = dict(depth_m=-.008, lateral_error_m=.0008, orientation_error_rad=0.,
        grasp_valid=True, temporary_support=False, feedback_available=True,
        resistance_N=0., lateral_N=0., torque_Nm=0.,
        relative_speed_m_s=0., relative_angular_rad_s=0.)
    policy.right_return_complete = True
    policy.begin_alignment(0., obs)
    tcp = NS(pose=sapien.Pose([.1, .2, .3]))
    scene = NS(policy=policy, config={}, observe=lambda: dict(obs),
        base=NS(pose=sapien.Pose()), hole=HOLE)
    plug = NS(pose=sapien.Pose(HOLE+[.008, .0008, 0.]-USB_IN_SOCKET@TIP, mat2quat(USB_IN_SOCKET)))
    names = [f'{side}{i}' for side in ('left', 'right') for i in range(7)]
    sim = NS(time=0., positions=np.zeros(14), target=np.zeros(14), names=names)
    def ik(*args, **kwargs):
        q = sim.target.copy()
        q[:7] += .00001
        return q, True, 0.
    model = NS(compute_inverse_kinematics=ik)
    robot = NS(pose=sapien.Pose(), get_links=lambda: [tcp], create_pinocchio_model=lambda: model)
    sim.env = NS(insertion=scene, plug=plug, agent=NS(robot=robot, links={'left_fr3_hand_tcp': tcp}))
    bridge = NS(sim=sim, dt=.02, failure=None, desired={}, reserved={('left', 'insertion')},
        arms={}, pending_arms={}, grippers={},
        arm_indices={'left': np.arange(7), 'right': np.arange(7, 14)},
        arm_names={'left': names[:7], 'right': names[7:]},
        assets=NS(limits={name: (-3., 3.) for name in names}))
    obj = InsertionBridge.__new__(InsertionBridge)
    obj.bridge, obj.owner, obj.goal, obj.model = bridge, True, tcp.pose, model
    obj.last_heartbeat = time.monotonic()
    obj.alignment_checked = obj.entry_waiting = False
    obj.pending_step = None
    obj.collision_guard, requests, futures = guard()
    obj.collision_guard.bridge = bridge
    obj.require_open = obj.require_ready = lambda side: None
    return obj, obs, requests, futures


@pytest.mark.parametrize('mode', ['entry', 'per_step'])
def test_entry_is_required_and_only_per_step_mode_waits_again(mode):
    obj, obs, requests, futures = local_controller(mode)
    obj.before_tick()
    assert len(requests) == 1
    for i in range(1, 5):
        obj.bridge.sim.time = i*.02
        obj.before_tick()
        assert not np.any(obj.bridge.sim.target)
    futures[0].set_result(NS(valid=True))
    obj.bridge.sim.time += .02
    obj.before_tick()
    assert obj.alignment_checked
    if mode == 'per_step':
        assert len(requests) >= 4
        assert not np.any(obj.bridge.sim.target)
        return
    for _ in range(5):
        obj.bridge.sim.time += .02
        obj.before_tick()
    assert len(requests) == 1 and obj.pending_step is None
    assert np.all(obj.bridge.sim.target[:7] > 0.)
    assert not np.any(obj.bridge.sim.target[7:])
    assert obj.scene.policy.alignment_travel > 0.
    # Finish measured dwell, then hand off to axial insertion without queries.
    obs['lateral_error_m'] = 0.
    for _ in range(20):
        obj.bridge.sim.time += .02
        obj.before_tick()
    assert obj.scene.policy.state == 'aligned' and not obj.owner
    obj.handle_start()
    for _ in range(10):
        obj.bridge.sim.time += .02
        obj.before_tick()
    assert obj.owner and obj.scene.policy.travel > 0.
    assert len(requests) == 1 and obj.pending_step is None


def test_invalid_entry_never_issues_motion():
    obj, _, _, futures = local_controller()
    obj.before_tick()
    futures[0].set_result(NS(valid=False, contacts=[NS(contact_body_1='left1', contact_body_2='plate')]))
    obj.bridge.sim.time = .02
    obj.before_tick()
    assert not obj.owner and obj.scene.policy.state == 'blocked'
    assert 'local_path_collision' in obj.scene.policy.reason
    assert not np.any(obj.bridge.sim.target)


@pytest.mark.parametrize('phase', ['alignment', 'insertion'])
@pytest.mark.parametrize('failure,state', [
    ({'resistance_N': 6.}, 'overload'), ({'grasp_valid': False}, 'slip_or_drop'),
    ({'feedback_available': False}, 'feedback_unavailable')])
def test_fast_control_still_stops_on_invalid_feedback(phase, failure, state):
    obj, obs, _, futures = local_controller()
    obj.before_tick()
    futures[0].set_result(NS(valid=True))
    obj.bridge.sim.time = .02
    obj.before_tick()
    if phase == 'insertion':
        obj.relinquish()
        obj.scene.policy.transition('aligned')
        obs['lateral_error_m'] = 0.
        obj.handle_start()
    obs.update(failure)
    obj.bridge.sim.time += .02
    obj.before_tick()
    assert obj.scene.policy.state == state and not obj.owner
    assert not obj.bridge.reserved and not obj.scene.policy.insertion_success


@pytest.mark.parametrize('wait_s', [2., 21.])
def test_direct_fast_start_checks_entry_without_consuming_progress_time(wait_s):
    obj, obs, requests, futures = local_controller()
    obj.relinquish()
    policy = obj.scene.policy
    policy.reset()
    policy.right_return_complete = True
    obs['lateral_error_m'] = 0.
    obj.handle_start()
    # The first response is the scene snapshot requested during preparation.
    futures[0].set_result(NS(scene=NS(world=NS(collision_objects=[NS(
        id='usb_socket', meshes=[object()], primitives=[])]), robot_state=NS(
        attached_collision_objects=[obj.collision_guard.attachment]))))
    obj.before_tick()
    assert len(requests) == 2  # one scene read + one state query
    obj.bridge.sim.time = wait_s  # Longer than the 1.5 s stall window.
    obj.before_tick()
    if wait_s > policy.alignment_limits.timeout_s:
        assert not obj.owner and policy.reason == 'entry_simulation_time_limit'
        assert futures[1].cancelled() and not np.any(obj.bridge.sim.target)
        return
    assert obj.owner and not np.any(obj.bridge.sim.target)
    futures[1].set_result(NS(valid=True))
    obj.before_tick()
    assert not obj.entry_waiting and policy.started == wait_s
    obj.bridge.sim.time += .02
    obj.before_tick()
    assert policy.state == 'feedback_advance' and policy.travel > 0.
    assert len(requests) == 2
