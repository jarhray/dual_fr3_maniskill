"""Measured-tip servo convergence, failure bounds and service ownership."""
from types import SimpleNamespace as NS

import numpy as np
import pytest
from transforms3d.axangles import axangle2mat

from dual_fr3_maniskill.usb.alignment import AlignmentLimits, correction_goal, rotation_distance
from dual_fr3_maniskill.usb.geometry import HOLE, TIP, USB_IN_SOCKET, measure
from dual_fr3_maniskill.usb.insertion import InsertionLimits, InsertionPolicy


def observation(**changes):
    return dict(dict(depth_m=-.008, lateral_error_m=0., orientation_error_rad=0.,
        grasp_valid=True, temporary_support=False, feedback_available=True,
        resistance_N=0., lateral_N=0., torque_Nm=0.,
        relative_speed_m_s=0., relative_angular_rad_s=0.), **changes)


def test_alignment_requires_stable_measured_dwell_and_preserves_start_gate():
    policy = InsertionPolicy({})
    initial = observation(lateral_error_m=.001, orientation_error_rad=.03)
    with pytest.raises(RuntimeError, match='lateral_error_mm=1.0000'):
        policy.begin(0., initial)
    assert policy.begin_alignment(0., initial)
    assert policy.update_alignment(.1, initial)
    assert not policy.update_alignment(.2, observation())
    assert policy.state == 'alignment_verification'
    policy.update_alignment(.45, observation(relative_speed_m_s=.002))
    assert policy.state == 'aligning'
    policy.update_alignment(.5, observation())
    policy.update_alignment(.81, observation())
    assert policy.state == 'aligned' and not policy.insertion_success
    assert policy.begin(.82, observation())
    assert policy.state == 'feedback_advance'


@pytest.mark.parametrize('change', [dict(depth_m=-.0101), dict(lateral_error_m=.0021),
    dict(depth_m=-.0095, lateral_error_m=.0015), dict(orientation_error_rad=.06),
    dict(depth_m=0.), dict(grasp_valid=False), dict(temporary_support=True),
    dict(feedback_available=False), dict(lateral_error_m=float('nan')), dict(resistance_N=6.)])
def test_alignment_refuses_unrecoverable_or_invalid_initial_state(change):
    policy = InsertionPolicy({})
    with pytest.raises(RuntimeError):
        policy.begin_alignment(0., observation(**change))
    assert policy.state == 'not_started' and not policy.insertion_success


@pytest.mark.parametrize('change,state', [
    (dict(grasp_valid=False), 'slip_or_drop'), (dict(resistance_N=6.), 'overload'),
    (dict(feedback_available=False), 'feedback_unavailable'),
    (dict(orientation_error_rad=.06), 'blocked'), (dict(depth_m=-.004), 'blocked')])
def test_alignment_stops_on_actual_feedback_failure(change, state):
    policy = InsertionPolicy({})
    policy.begin_alignment(0., observation(lateral_error_m=.001))
    assert not policy.update_alignment(.1, observation(**change))
    assert policy.state == state and not policy.insertion_success


def test_alignment_budget_stall_timeout_and_reset():
    policy = InsertionPolicy({})
    obs = observation(lateral_error_m=.001)
    policy.begin_alignment(0., obs)
    policy.update_alignment(.1, obs)
    policy.update_alignment(3.2, obs)
    assert policy.reason == 'alignment_no_measured_progress'
    policy.reset()
    policy.begin_alignment(0., obs)
    policy.update_alignment(21., obs)
    assert policy.state == 'timeout'
    policy.reset()
    policy.begin_alignment(0., obs)
    policy.record_alignment_step(.005, .05)
    with pytest.raises(RuntimeError, match='cumulative'):
        policy.record_alignment_step(.002, .01)
    assert policy.alignment_travel == .005
    with pytest.raises(RuntimeError, match='cumulative'):
        policy.record_alignment_step(0., .1)
    policy.reset()
    assert policy.alignment_travel == policy.alignment_rotation == 0.


@pytest.mark.parametrize('settings', [dict(position_tolerance_m=.001), dict(capture_translation_m=.008),
    dict(angle_tolerance_rad=.1), dict(speed_m_s=0.), dict(hold_s=float('nan'))])
def test_alignment_cannot_relax_original_limits(settings):
    with pytest.raises(ValueError):
        AlignmentLimits.read(settings, InsertionLimits())


@pytest.mark.parametrize('mode', [None, False, 'off', 'per-step', 1])
def test_unknown_collision_mode_is_rejected(mode):
    with pytest.raises(ValueError, match='local_collision_check'):
        InsertionPolicy({'local_collision_check': mode})


def poses():
    base = np.eye(4)
    base[:3, :3] = axangle2mat([.1, .4, .7], .5)
    base[:3, 3] = [.4, .3, .1]
    usb = np.eye(4)
    usb[:3, :3] = axangle2mat([0., 0., 1.], .03) @ USB_IN_SOCKET
    usb[:3, 3] = HOLE + [.0085, .0007, -.0003] - usb[:3, :3] @ TIP
    usb = base @ usb
    mount = np.eye(4)
    mount[:3, 3] = [.001, -.002, .012]
    mount[:3, :3] = axangle2mat([1., 0., 0.], .07)
    return base, usb @ np.linalg.inv(mount), usb, mount


def test_bounded_tip_servo_converges_with_rotated_base_and_changing_grasp():
    base, tcp, usb, mount = poses()
    a = AlignmentLimits()
    initial = measure(base, usb)
    for i in range(350):
        old_tip = usb[:3, 3] + usb[:3, :3] @ TIP
        target = correction_goal(base, tcp, usb, HOLE, .008, a, .02)
        result = target @ mount
        new_tip = result[:3, 3] + result[:3, :3] @ TIP
        assert np.linalg.norm(new_tip-old_tip) <= a.speed_m_s*.02 + 1e-12
        assert rotation_distance(usb[:3, :3], result[:3, :3]) <= a.angular_speed_rad_s*.02 + 1e-9
        tcp, usb = target, result
        if i == 100:
            # A physical grasp shift changes the next measured target; keeping
            # the original mount would leave an uncorrected tip error.
            mount[:3, 3] += [.0002, 0., -.0001]
            usb = tcp @ mount
    final = measure(base, usb)
    assert initial['lateral_error_m'] > .00025
    assert final['lateral_error_m'] < .00001
    assert abs(final['depth_m']+.008) < .00001
    assert final['orientation_error_rad'] < .001


def test_rotation_compensates_tcp_to_tip_lever_arm():
    base, tcp, usb, mount = poses()
    # Already position-aligned, only the plug's angle needs correction.
    desired_tip = base[:3, 3] + base[:3, :3] @ (HOLE + [.008, 0., 0.])
    usb[:3, 3] = desired_tip - usb[:3, :3] @ TIP
    tcp = usb @ np.linalg.inv(mount)
    target = correction_goal(base, tcp, usb, HOLE, .008, AlignmentLimits(), .02)
    moved_usb = target @ mount
    np.testing.assert_allclose(moved_usb[:3, 3] + moved_usb[:3, :3] @ TIP, desired_tip, atol=1e-12)
    assert np.linalg.norm(target[:3, 3]-tcp[:3, 3]) > 0.


def test_async_wait_does_not_consume_commanded_insertion_distance():
    policy = InsertionPolicy({})
    policy.begin(0., observation())
    for i in range(1, 20):
        policy.update(i*.02, observation(depth_m=-.008+i*.00001), command_dt=0.)
    assert policy.speed > 0. and policy.travel == 0.


def test_start_during_alignment_is_rejected_without_cancelling_it():
    from dual_fr3_maniskill.usb.bridge import InsertionBridge
    policy = InsertionPolicy({})
    policy.begin_alignment(0., observation(lateral_error_m=.001))
    control = InsertionBridge.__new__(InsertionBridge)
    control.owner = True
    control.bridge = NS(sim=NS(env=NS(insertion=NS(policy=policy))))
    result = control.command('start', NS())
    assert not result.success and 'alignment' in result.message
    assert control.owner and policy.state == 'aligning'


@pytest.mark.parametrize('cause', ['cancel', 'wall_timeout', 'grasp_lost', 'sim_timeout'])
def test_pending_alignment_step_is_cancelled_before_any_motion(cause):
    import time
    from dual_fr3_maniskill.usb.bridge import InsertionBridge
    policy = InsertionPolicy({})
    obs = observation(lateral_error_m=.001)
    policy.begin_alignment(0., obs)
    cancelled = []
    control = InsertionBridge.__new__(InsertionBridge)
    control.owner = True
    control.pending_step = object()
    control.collision_guard = NS(cancel=lambda: cancelled.append(True))
    control.last_heartbeat = time.monotonic() - (11. if cause == 'wall_timeout' else 0.)
    if cause == 'grasp_lost': obs['grasp_valid'] = False
    scene = NS(policy=policy, observe=lambda: obs)
    control.bridge = NS(failure=None, sim=NS(env=NS(insertion=scene),
        time=21. if cause == 'sim_timeout' else .1,
        positions=np.zeros(7), target=np.ones(7)), desired={},
        arm_indices={'left':list(range(7))}, reserved={('left', 'insertion')})
    if cause == 'cancel':
        control.handle_cancel()
    else:
        control.before_tick()
    assert cancelled and not control.owner and not control.bridge.reserved
    assert control.pending_step is None and not policy.insertion_success
    assert policy.state == {'cancel':'cancelled', 'wall_timeout':'timeout',
                            'grasp_lost':'slip_or_drop', 'sim_timeout':'timeout'}[cause]
    np.testing.assert_array_equal(control.bridge.sim.target, np.zeros(7))
