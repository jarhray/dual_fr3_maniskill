"""Trigger contracts and ownership transitions without a ROS executor or renderer."""
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from dual_fr3_maniskill.insertion import InsertionPolicy, SERVICE_OPERATIONS, SERVICE_PREFIX
from dual_fr3_maniskill.insertion_bridge import InsertionBridge


@pytest.fixture
def controller():
    observation = dict(
        depth_m=-.008, lateral_error_m=0., orientation_error_rad=0.,
        feedback_available=True, grasp_valid=True, temporary_support=False,
        resistance_N=0., lateral_N=0., torque_Nm=0.,
        relative_speed_m_s=0., relative_angular_rad_s=0.)
    pose = NS(p=np.zeros(3), q=np.array([1., 0., 0., 0.]))
    policy = InsertionPolicy({})
    scene = NS(policy=policy, config={}, observe=lambda: dict(observation), base=NS(pose=pose))

    def target(depth=None):
        policy.transition('approach')
        return dict(frame='world', position_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.])

    scene.target = target
    names = {side: [f'{side}{i}' for i in range(7)] for side in ('left', 'right')}
    bridge = NS(
        sim=NS(time=0., target=np.ones(18), positions=np.r_[np.zeros(14), [.04, .04, .015, .015]],
               velocities=np.zeros(18), env=NS(insertion=scene, agent=NS(
                   links={'left_fr3_hand_tcp': NS(pose=pose)},
                   robot=NS(create_pinocchio_model=lambda: object()))),
               indices={f'{side}_fr3_finger_joint{i + 1}': start + i
                        for side, start in [('left', 14), ('right', 16)] for i in range(2)}),
        arm_names=names, arm_indices={'left': np.arange(7), 'right': np.arange(7, 14)},
        assets=NS(initial_positions={n: 0. for group in names.values() for n in group}),
        reserved=set(), arms={}, pending_arms={}, grippers={}, desired={}, failure=None)
    adapter = InsertionBridge.__new__(InsertionBridge)
    adapter.bridge = bridge
    adapter.owner = False
    adapter.last_heartbeat = -1.
    adapter.goal = adapter.model = None
    return adapter, observation


def call(adapter, operation):
    response = adapter.command(operation, NS())
    assert response.success, response.message
    return json.loads(response.message)


def prepare(adapter):
    for operation in ('right_release', 'right_released', 'right_returned', 'target'):
        call(adapter, operation)


def test_full_service_sequence_retention_order_and_repeated_calls(controller):
    adapter, observation = controller
    prepare(adapter)
    call(adapter, 'start')
    model = adapter.model
    call(adapter, 'start')
    assert adapter.owner and adapter.model is model
    assert adapter.bridge.reserved == {('left', 'insertion')}

    policy = adapter.scene.policy
    observation['depth_m'] = .01
    policy.update(.1, observation)
    assert not policy.insertion_success

    def retain():
        # before_tick must record unretained success and give up drive ownership
        # BEFORE the physical scene is allowed to install the holding constraint.
        assert policy.insertion_success and policy.state == 'inserted_unretained'
        assert not adapter.owner and not adapter.bridge.reserved
        policy.retention_active = True
        policy.transition('retained')

    adapter.scene.retain = retain
    adapter.bridge.sim.time = .41
    adapter.before_tick()
    np.testing.assert_array_equal(adapter.bridge.sim.target[:7], adapter.bridge.sim.positions[:7])
    for operation in ('retained', 'released', 'returning', 'returned'):
        first = call(adapter, operation)
        assert call(adapter, operation) == first
    complete = call(adapter, 'status')
    for operation in ('start', 'right_release', 'right_released', 'right_returned',
                      'retained', 'released', 'returning', 'returned', 'cancel'):
        assert call(adapter, operation) == complete
    assert all(complete[key] for key in (
        'insertion_success', 'retention_active', 'grippers_released', 'return_complete'))


@pytest.mark.parametrize('busy', ['reserved', 'arms', 'pending_arms', 'grippers'])
@pytest.mark.parametrize('operation', ['right_release', 'start'])
def test_busy_actions_cannot_handoff(controller, busy, operation):
    adapter, _ = controller
    if operation == 'start':
        prepare(adapter)
    setattr(adapter.bridge, busy, {('left', 'test')})
    response = adapter.command(operation, NS())
    assert not response.success
    assert response.message == ('Wait for original MTC actions to finish' if operation == 'right_release'
                                else 'MTC/gripper action still owns a drive')
    assert not adapter.owner


def test_cancel_failure_reset_and_missing_scene(controller):
    adapter, _ = controller
    prepare(adapter)
    call(adapter, 'start')
    stopped = call(adapter, 'cancel')
    assert stopped['state'] == 'cancelled' and not adapter.owner
    assert call(adapter, 'cancel') == stopped
    assert not adapter.command('start', NS()).success
    adapter.scene.policy.reset()
    assert call(adapter, 'status')['state'] == 'not_started'
    prepare(adapter)
    call(adapter, 'start')
    call(adapter, 'cancel')
    adapter.bridge.sim.env.insertion = None
    for operation in SERVICE_OPERATIONS:
        response = adapter.command(operation, NS())
        assert not response.success
        assert response.message == 'Insertion disabled or reset; spawn first'


@pytest.mark.parametrize('operation,state,reason', [
    ('fail_right_release', 'right_release_failed', 'right_arm_preinsertion_failed; see_MTC_log'),
    ('fail_right_return', 'right_return_failed', 'right_arm_preinsertion_failed; see_MTC_log'),
    ('fail_release', 'release_failed', 'gripper_open_or_planning_detach_failed; see_MTC_log'),
    ('fail_return', 'return_failed', 'release_detach_or_return_failed; see_MTC_log'),
])
def test_failure_report_strings_and_idempotency(controller, operation, state, reason):
    adapter, _ = controller
    result = call(adapter, operation)
    assert result['state'] == state and result['reason'] == reason
    assert not result['insertion_success'] and not result['retention_active']
    assert call(adapter, operation) == result


def test_status_does_not_renew_lease_and_heartbeat_does(controller, monkeypatch):
    adapter, _ = controller
    monkeypatch.setattr('dual_fr3_maniskill.insertion_bridge.time.monotonic', lambda: 42.)
    before = call(adapter, 'status')
    assert adapter.last_heartbeat == -1.
    assert call(adapter, 'heartbeat') == before
    assert adapter.last_heartbeat == 42.
    assert SERVICE_PREFIX == '/maniskill/usb/insertion/'


def test_service_registration_keeps_operation_bound_to_each_callback(controller):
    adapter, _ = controller
    callbacks = {}
    bridge = adapter.bridge
    bridge.cable_config = {}
    bridge.create_publisher = lambda *args: None
    bridge.create_service = lambda kind, name, callback: callbacks.setdefault(name, callback)
    registered = InsertionBridge(bridge)
    operations = []
    registered.command = lambda operation, response: operations.append(operation)
    for name, callback in callbacks.items():
        callback(None, NS())
        assert name == SERVICE_PREFIX + operations[-1]
    assert tuple(operations) == SERVICE_OPERATIONS
