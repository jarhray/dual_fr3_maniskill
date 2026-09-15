"""Serialized deferred activation; both completed gripper actions are required."""
import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.action import GoalResponse
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from ..ros_bridge import ManiSkillBridge, main as bridge_main, stamp
from ..scenes import resolve_cable_config
from .model import USB_LINK, load_config
from .ros_bridge import UsbCableBridge
from .threading import task_usb_mount


class TrunkingCableBridge(UsbCableBridge):
    def __init__(self):
        self.closed_sides = set()
        super().__init__()
        self.usb_tf = TransformBroadcaster(self)
        self.spawn_service = self.create_service(Trigger, "/maniskill/cable/spawn", self.spawn_cable)
        self.get_logger().info("MTC cable deferred: waiting for both grippers to close, then /maniskill/cable/spawn")

    def create_simulation(self, assets):
        from ..scenes.trunking_cable import TrunkingCableSimulation
        path = self.declare_parameter("cable_config", resolve_cable_config(scene="trunking_cable")).value
        self.cable_solver = self.declare_parameter("cable_solver", "mpm").value
        self.cable_config = load_config(path, solver=self.cable_solver)
        self.cable_config["usb"]["orientation_direction"] = self.declare_parameter(
            "leader_orientation_direction", "reverse").value
        task_usb_mount(self.cable_config)  # Validate before starting the simulator.
        return TrunkingCableSimulation(assets, cable_config=self.cable_config, cable_solver=self.cable_solver,
            control_freq=self.config["control_freq"], sim_freq=self.config["sim_freq"], viewer=self.config["viewer"])

    def finish_gripper(self, side, *, reached=False, stalled=False, cancel=False):
        target = self.grippers[side].position
        expected = self.cable_config["usb"]["finger_position"] if side == "left" else 0.
        if reached and not cancel and abs(target - expected) <= .0001:
            self.closed_sides.add(side)
        else:
            self.closed_sides.discard(side)
        super().finish_gripper(side, reached=reached, stalled=stalled, cancel=cancel)

    def accept_gripper(self, side, goal):
        if self.failure:
            return GoalResponse.REJECT
        if self.sim.cable is not None:
            expected = self.cable_config["usb"]["finger_position"] if side == "left" else 0.
            if abs(goal.command.position - expected) > .0001:
                self.get_logger().warning("MTC cable uses a fixed left clamp and a closed right guide; opening requires a new scene")
                return GoalResponse.REJECT
        result = ManiSkillBridge.accept_gripper(self, side, goal)
        if result == GoalResponse.ACCEPT:
            self.closed_sides.discard(side)
        return result

    def spawn_cable(self, request, response):
        if self.failure:
            response.success, response.message = False, self.failure
            return response
        if self.sim.cable is not None:
            response.success, response.message = True, "Cable already active; unchanged"
            return response
        if self.reserved or self.closed_sides != {"left", "right"}:
            response.success, response.message = False, "Complete both gripper closures and all active motions before inserting cable"
            return response
        expected = {"left": self.cable_config["usb"]["finger_position"], "right": 0.}
        for side, target in expected.items():
            indices = [self.sim.indices[f"{side}_fr3_finger_joint{i}"] for i in (1, 2)]
            if not np.all(np.abs(self.sim.positions[indices] - target) <= self.config["gripper_goal_tolerance"]):
                response.success, response.message = False, f"{side} gripper is not at its preparation width"
                return response
        try:
            self.sim.env.spawn_cable()
        except Exception as exc:  # GPU/geometry failures must stop MTC before descent.
            response.success, response.message = False, f"Cable insertion failed: {exc}"
            self.get_logger().error(response.message)
            return response
        response.success, response.message = True, "USB fixed to left TCP; cable threaded through sliding right TCP guide"
        self.get_logger().info(response.message)
        return response

    def publish_state(self, q, v):
        super().publish_state(q, v)
        if self.sim.cable is None:
            return
        pose = self.sim.cable.plug.pose
        transform = TransformStamped(child_frame_id=USB_LINK)
        transform.header.frame_id, transform.header.stamp = "world", stamp(self.sim.time)
        xyz, quat = transform.transform.translation, transform.transform.rotation
        xyz.x, xyz.y, xyz.z = map(float, pose.p)
        quat.w, quat.x, quat.y, quat.z = map(float, pose.q)
        self.usb_tf.sendTransform(transform)


def main():
    bridge_main(TrunkingCableBridge)
