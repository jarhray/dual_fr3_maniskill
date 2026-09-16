"""Grasp state logic and ground-truth transform semantics, without a renderer."""
import json
from types import SimpleNamespace

import numpy as np
import pytest
from transforms3d.axangles import axangle2mat
from transforms3d.quaternions import mat2quat

from dual_fr3_maniskill.usb_grasp import (
    GraspThresholds, UsbGraspMonitor, read_usb_finger_normal_loads, relative_pose)


def pose(p=(0., 0., 0.), rotation=None):
    return SimpleNamespace(p=np.asarray(p), q=mat2quat(np.eye(3) if rotation is None else rotation))


LOADS = dict(left_fr3_leftfinger=2., left_fr3_rightfinger=2.)


def ready_monitor():
    monitor = UsbGraspMonitor()
    monitor.created(0.)
    monitor.begin_closing(0.)
    monitor.observe(0., pose(), pose(), LOADS)
    monitor.observe(.1, pose(), pose(), LOADS)
    assert monitor.ready_to_release
    return monitor


def released_monitor():
    monitor = ready_monitor()
    monitor.released(.1, pose(), pose())
    return monitor


def test_bilateral_contact_requires_continuous_duration_and_support_is_not_success():
    monitor = UsbGraspMonitor()
    assert not monitor.ready_to_release
    assert monitor.created(0.)
    assert not monitor.created(.1)
    monitor.begin_closing(0.)
    for time in (0., .05):
        monitor.observe(time, pose(), pose(), LOADS)
    monitor.observe(.08, pose(), pose(), dict(left_fr3_leftfinger=2., left_fr3_rightfinger=0.))
    monitor.observe(.12, pose(), pose(), LOADS)
    monitor.observe(.18, pose(), pose(), LOADS)
    assert not monitor.ready_to_release
    with pytest.raises(ValueError, match="sustained bilateral"):
        monitor.released(.18, pose(), pose())
    monitor.observe(.23, pose(), pose(), LOADS)
    result = monitor.snapshot()
    assert result["ready_to_release"] and result["external_support"]
    assert not result["stable"]


def test_common_translation_and_rotation_are_not_slip():
    monitor = released_monitor()
    for time in np.arange(.11, 1.01, .01):
        rotation = axangle2mat([0., 0., 1.], time)
        shared = pose([time, 2.*time, -.5*time], rotation)
        monitor.observe(time, shared, shared, LOADS)
    result = monitor.snapshot()
    assert result["stable"] and not result["external_support"]
    assert result["relative_translation_m"] == pytest.approx(0.)
    assert result["relative_rotation_rad"] == pytest.approx(0., abs=3.e-8)
    assert result["world_translation_m"] > 1.
    assert result["world_rotation_rad"] == pytest.approx(1.)
    np.testing.assert_allclose(result["relative_displacement_m"], [0, 0, 0], atol=1.e-10)
    np.testing.assert_allclose(result["relative_linear_velocity_m_s"], [0, 0, 0], atol=1.e-10)


def test_relative_transform_uses_real_offsets_and_tcp_rotation():
    rotation = axangle2mat([0, 0, 1], np.pi/2)
    local_rotation = axangle2mat([0, 1, 0], .3)
    tcp = pose([1., 2., 3.], rotation)
    usb = pose(tcp.p + rotation @ np.array([.02, .03, -.01]), rotation @ local_rotation)
    p, r = relative_pose(tcp, usb)
    np.testing.assert_allclose(p, [.02, .03, -.01])
    np.testing.assert_allclose(r, local_rotation, atol=1.e-14)


def test_relative_drift_slips_and_opening_drops_after_debounce():
    monitor = released_monitor()
    for time in np.arange(.11, .51, .01):
        monitor.observe(time, pose(), pose(), LOADS)
    assert monitor.snapshot()["stable"]
    monitor.observe(.51, pose(), pose([.004, 0, 0]), LOADS)
    assert monitor.state == "stable"  # A single noisy sample is not a failure.
    monitor.observe(.62, pose(), pose([.004, 0, 0]), LOADS)
    assert monitor.state == "slipping"
    monitor.observe(.63, pose(), pose([.004, 0, 0]), dict.fromkeys(LOADS, 0.))
    monitor.observe(.70, pose(), pose([.004, 0, 0]), dict.fromkeys(LOADS, 0.))
    assert monitor.state != "dropped"
    monitor.observe(.90, pose(), pose([.004, 0, 0]), dict.fromkeys(LOADS, 0.))
    assert monitor.state == "dropped"
    assert monitor.reason == "both_finger_contacts_lost"
    monitor.observe(1., pose(), pose([.08, 0, 0]), dict.fromkeys(LOADS, 0.))
    assert monitor.state == "dropped"
    assert monitor.snapshot()["relative_translation_m"] == pytest.approx(.08)
    np.testing.assert_allclose(monitor.snapshot()["relative_displacement_m"], [.08, 0, 0])
    json.dumps(monitor.snapshot(), allow_nan=False)


def test_contact_jitter_recovers_and_is_not_immediate_dropout():
    monitor = released_monitor()
    monitor.observe(.11, pose(), pose(), LOADS)
    monitor.observe(.48, pose(), pose(), LOADS)
    monitor.observe(.50, pose(), pose(), dict.fromkeys(LOADS, 0.))
    monitor.observe(.55, pose(), pose(), LOADS)
    monitor.observe(.9, pose(), pose(), LOADS)
    assert monitor.snapshot()["stable"]


def test_rotation_detected_and_release_baseline_does_not_follow_usb():
    monitor = released_monitor()
    usb = pose(rotation=axangle2mat([1, 0, 0], .2))
    monitor.observe(.11, pose(), usb, LOADS)
    monitor.observe(.22, pose(), usb, LOADS)
    assert monitor.state == "slipping"
    assert monitor.snapshot()["relative_rotation_rad"] == pytest.approx(.2)
    assert not monitor.released(.23, pose(), usb, manual=True)
    assert monitor.snapshot()["release_baseline"]["quaternion_wxyz"] == [1., 0., 0., 0.]


def test_close_timeout_manual_release_failure_and_reset_are_explicit():
    monitor = UsbGraspMonitor()
    monitor.created(0.)
    monitor.begin_closing(0.)
    monitor.observe(5.1, pose(), pose(), dict.fromkeys(LOADS, 0.))
    assert monitor.state == "failed" and monitor.external_support
    assert monitor.reason == "closing_timeout_insufficient_bilateral_contact"
    monitor.reset()
    monitor.created(0.)
    monitor.released(0., pose(), pose(), manual=True)
    assert not monitor.external_support and monitor.state == "verifying"
    monitor.fail("world_support_removal_failed")
    assert monitor.state == "failed"
    monitor.reset()
    assert monitor.snapshot()["relative_pose"] is None
    assert monitor.state == "not_created"
    assert monitor.snapshot()["creation_world_pose"] is None


def test_verification_deadline_fails_when_jitter_never_satisfies_stability():
    monitor = released_monitor()
    for i, time in enumerate(np.arange(.12, 3.2, .02)):
        loads = LOADS if i % 2 else dict.fromkeys(LOADS, 0.)
        monitor.observe(time, pose(), pose(), loads)
    assert monitor.state == "failed"
    assert monitor.reason == "released_grasp_verification_timeout"
    assert not monitor.external_support


def test_nonfinite_observation_clears_current_metrics_and_keeps_json_serializable():
    monitor = released_monitor()
    monitor.observe(.2, pose(), pose([np.nan, 0, 0]), LOADS)
    snapshot = monitor.snapshot()
    assert snapshot["state"] == "failed"
    assert snapshot["relative_pose"] is None and snapshot["relative_translation_m"] is None
    json.dumps(snapshot, allow_nan=False)


def test_nonfinite_release_pose_fails_without_claiming_support_still_exists():
    monitor = ready_monitor()
    with pytest.raises(RuntimeError, match="nonfinite pose"):
        monitor.released(.2, pose(), pose([np.nan, 0, 0]))
    snapshot = monitor.snapshot()
    assert snapshot["state"] == "failed" and not snapshot["external_support"]
    assert snapshot["release_baseline"] is None
    json.dumps(snapshot, allow_nan=False)


@pytest.mark.parametrize("key,value", [("slip_hold_s", 0), ("contact_min_force_N", float("nan")),
    ("drop_translation_m", .001), ("verification_timeout_s", .01)])
def test_invalid_thresholds_fail_before_physics(key, value):
    with pytest.raises(ValueError):
        GraspThresholds.from_config({key: value})


def test_real_finger_contacts_count_but_rope_proxies_do_not():
    plug = SimpleNamespace(id=1)
    fingers = [SimpleNamespace(id=2, name="left_fr3_leftfinger"),
               SimpleNamespace(id=3, name="left_fr3_rightfinger")]
    proxy = SimpleNamespace(id=4, name="rope_contact_left_fr3_leftfinger")
    contacts = [SimpleNamespace(actor0=plug, actor1=body, points=[SimpleNamespace(
        impulse=np.array([0., .002, 0.]), normal=np.array([0., 1., 0.]))])
        for body in [*fingers, proxy]]
    env = SimpleNamespace(agent=SimpleNamespace(links={body.name: body for body in fingers}),
                          _scene=SimpleNamespace(get_contacts=lambda: contacts))
    assert read_usb_finger_normal_loads(env, plug, .001) == dict.fromkeys(LOADS, 2.)


def test_creation_world_reference_precedes_release_and_is_not_slip_baseline():
    monitor = UsbGraspMonitor()
    monitor.created(0., pose([1., 2., 3.]))
    monitor.observe(.02, pose(), pose([1.1, 2., 3.]), LOADS)
    snapshot = monitor.snapshot()
    assert snapshot["world_displacement_m"] == pytest.approx([.1, 0., 0.])
    assert snapshot["world_translation_m"] == pytest.approx(.1)
    assert snapshot["relative_displacement_m"] is None
    assert snapshot["external_support"] and not snapshot["stable"]
    monitor.reset()
    assert monitor.snapshot()["world_pose"] is None
