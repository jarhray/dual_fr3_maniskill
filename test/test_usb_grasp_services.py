"""ROS action admission and accepted-goal races; no native ROS context needed."""
from types import SimpleNamespace as NS
import json

import numpy as np
import pytest
from rclpy.action import GoalResponse

from dual_fr3_maniskill.cable.ros_bridge import UsbCableBridge
from dual_fr3_maniskill.ros_bridge import ManiSkillBridge
from dual_fr3_maniskill.usb_grasp import UsbGraspMonitor


@pytest.fixture
def bridge(monkeypatch):
    bridge = object.__new__(UsbCableBridge)
    bridge.failure = None
    bridge._grasp_waiters = []
    bridge.reserved = set()
    monitor = UsbGraspMonitor()
    monitor.created(0.)
    bridge.sim = NS(env=NS(support_drive=object(), grasp_monitor=monitor, _grasp_time=0.),
                    indices={"left_fr3_finger_joint1": 0}, positions=np.array([.01]))
    bridge.get_logger = lambda: NS(warning=lambda _: None)
    def accept(self, side, goal):
        self.reserved.add((side, "gripper"))
        return GoalResponse.ACCEPT
    monkeypatch.setattr(ManiSkillBridge, "accept_gripper", accept)
    return bridge


def goal(position):
    return NS(command=NS(position=position, max_effort=10.))


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("position", [0., .02])
def test_pending_release_or_verify_prevents_new_gripper_motion(bridge, side, position):
    bridge._grasp_waiters.append(object())
    assert bridge.accept_gripper(side, goal(position)) == GoalResponse.REJECT
    assert not bridge.reserved


@pytest.mark.parametrize("state", ["failed", "not_created"])
def test_failed_or_missing_monitor_cannot_start_supported_close_but_can_open(bridge, state):
    monitor = bridge.sim.env.grasp_monitor
    monitor.fail("closing_timeout") if state == "failed" else monitor.reset()
    assert bridge.accept_gripper("left", goal(0.)) == GoalResponse.REJECT
    assert not bridge.reserved
    assert bridge.accept_gripper("left", goal(.02)) == GoalResponse.ACCEPT


def test_empty_scene_still_allows_ordinary_gripper_motion(bridge):
    bridge.sim.env.support_drive = None
    bridge.sim.env.grasp_monitor.reset()
    assert bridge.accept_gripper("left", goal(0.)) == GoalResponse.ACCEPT


def test_monitor_failure_after_acceptance_aborts_and_releases_reservation(bridge, monkeypatch):
    request = goal(0.)
    assert bridge.accept_gripper("left", request) == GoalResponse.ACCEPT
    bridge.sim.env.grasp_monitor.fail("closing_timeout")
    events = []
    monkeypatch.setattr(ManiSkillBridge, "start_gripper", lambda *args: events.append("start"))
    def finish(side):
        events.append("abort")
        bridge.reserved.discard((side, "gripper"))
    bridge.finish_gripper = finish
    bridge.start_gripper("left", NS(request=request))
    assert events == ["start", "abort"]
    assert not bridge.reserved


def test_status_exposes_actual_world_actor_pose_even_when_supported(bridge):
    bridge.sim.env.plug = NS(pose=NS(p=np.array([.4, .5, .6]), q=np.array([1., 0., 0., 0.])))
    response = bridge.usb_status(None, NS())
    result = json.loads(response.message)
    assert not response.success
    assert result["world_pose"] == dict(frame="world", position_m=[.4, .5, .6], quaternion_wxyz=[1., 0., 0., 0.])
    bridge.sim.env.plug = None
    assert json.loads(bridge.usb_status(None, NS()).message)["world_pose"] is None


def test_preparation_target_is_transformed_from_keypoint_frame_once(bridge):
    from dual_fr3_maniskill.sapien_compat import sapien
    bridge.load_cable = False
    bridge.sim.env.agent = NS(links={"left_fr3_link0": NS(pose=sapien.Pose([.175, .35, 0.]))})
    bridge.sim.env.fixtures = {}
    targets = bridge._preparation_targets({"left": dict(frame="left_fr3_link0",
        position_m=[.604, .133, .1], quaternion_wxyz=[1., 0., 0., 0.])})
    np.testing.assert_allclose(targets["left"].p, [.779, .483, .1], atol=1.e-7)
    bridge.sim.env.agent.links["left_fr3_link0"].pose = sapien.Pose([1., 2., 3.])
    np.testing.assert_allclose(targets["left"].p, [.779, .483, .1], atol=1.e-7)
    bridge.load_cable = True
    with pytest.raises(ValueError, match="Missing preparation"):
        bridge._preparation_targets({"left": {}})


def test_grasp_stream_does_not_require_force_output_and_logs_only_changes(bridge):
    messages, logs = [], []
    bridge.grasp_pub = NS(publish=lambda message: messages.append(json.loads(message.data)))
    bridge.get_logger = lambda: NS(info=logs.append)
    bridge._last_grasp_state = None
    bridge.publish_grasp_state()
    bridge.publish_grasp_state()
    assert len(messages) == 2 and len(logs) == 1
    assert messages[0]["external_support"] is True
    assert "creation_world_pose" in messages[0]
    bridge.sim.env.grasp_monitor.fail("injected_failure")
    bridge.publish_grasp_state()
    assert messages[-1]["state"] == "failed" and len(logs) == 2
