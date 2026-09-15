"""Measured cable markers and reset service; arm actions use the existing bridge."""
import json
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from rclpy.action import GoalResponse
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from dual_fr3_maniskill.ros_bridge import ManiSkillBridge, main as bridge_main, stamp
from dual_fr3_maniskill.scenes import resolve_cable_config
from .model import load_config


class UsbCableBridge(ManiSkillBridge):
    def __init__(self):
        super().__init__()
        trace_dir = self.declare_parameter("cable_trace_dir", "").value
        if trace_dir:
            if self.cable_solver != "rope_actor":
                raise ValueError("cable_trace_dir requires cable_solver:=rope_actor")
            from .rope_diagnostics import RopeRunRecorder
            self.sim.env.rope_trace = RopeRunRecorder(trace_dir,
                metadata=dict(scene=type(self.sim.env).__name__, cable_config=self.cable_config,
                              bridge_config=self.config), on_error=self.get_logger().error)
            self.get_logger().info(f"Cable trace enabled: {self.sim.env.rope_trace.directory}")
        self.marker_pub = self.create_publisher(MarkerArray, "/usb_cable_demo/markers", 1)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/usb_cable_demo/diagnostics", 1)
        self.reset_service = self.create_service(Trigger, "/usb_cable_demo/reset", self.reset_cable)
        self.last_diagnostic = 0.
        self.failure = None
        self.wall_start = time.monotonic()
        if self.sim.cable is not None:
            self.get_logger().info("USB fixed to left gripper; drag left_fr3_arm in RViz, then Plan & Execute. "
                                   f"Orange cable shows measured {self.cable_solver} state.")

    def create_simulation(self, assets):
        from ..scenes.usb_cable import UsbCableSimulation
        config_path = self.declare_parameter("cable_config", resolve_cable_config()).value
        self.cable_solver = self.declare_parameter("cable_solver", "mpm").value
        self.cable_config = load_config(config_path, solver=self.cable_solver)
        return UsbCableSimulation(assets, cable_config=self.cable_config, cable_solver=self.cable_solver,
            control_freq=self.config["control_freq"], sim_freq=self.config["sim_freq"], viewer=self.config["viewer"])

    def accept_gripper(self, side, goal):
        if self.failure:
            return GoalResponse.REJECT
        if side == "left" and self.sim.cable is not None:
            self.get_logger().warning("The left gripper is fixed around the USB in this demo")
            return GoalResponse.REJECT
        return super().accept_gripper(side, goal)

    def accept_arm(self, side, goal):
        if self.failure:
            self.get_logger().warning("Reset the cable after the numerical error before executing another plan")
            return GoalResponse.REJECT
        return super().accept_arm(side, goal)

    def tick(self):
        if self.sim.env._viewer is not None and self.sim.env._viewer.closed:
            self.get_logger().info("ManiSkill viewer closed")
            raise KeyboardInterrupt
        if self.failure:
            # Keep the existing viewer responsive without uploading failed state.
            if self.sim.env._viewer is not None:
                self.sim.env._viewer.render()
            return
        try:
            super().tick()
        except RuntimeError as exc:
            self.failure = str(exc)
            if self.sim.env.rope_trace is not None:
                report = self.sim.env.rope_trace.capture("failure", self.sim.env, reason=self.failure)
                if report is not None:
                    self.get_logger().error(f"Cable failure trace saved: {report}")
            for side in list(self.arms):
                self.finish_arm(side, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                self.cable_solver + " stopped: " + self.failure)
            for side in list(self.grippers):
                self.finish_gripper(side)
            self.get_logger().error(self.cable_solver + " paused: " + self.failure + "; call /usb_cable_demo/reset to restart")
            self.diag_pub.publish(DiagnosticArray(status=[DiagnosticStatus(
                name="usb_cable_demo/" + ("MPM" if self.cable_solver == "mpm" else self.cable_solver), level=DiagnosticStatus.ERROR, message=self.failure)]))

    def reset_cable(self, request, response):
        if self.sim.cable is None:
            response.success, response.message = False, "Cable has not been created by MTC preparation yet"
            return response
        if self.reserved:
            response.success, response.message = False, "Wait for or cancel the active arm/gripper motion before resetting"
            return response
        try:
            self.sim.cable.reset()
        except (RuntimeError, ValueError) as exc:
            response.success, response.message = False, str(exc)
            return response
        self.failure = None
        if self.sim.env.rope_trace is not None:
            self.sim.env.rope_trace.capture("reset", self.sim.env)
        response.success, response.message = True, "Cable reset at the current USB pose; arm pose is unchanged"
        return response

    def publish_state(self, q, v):
        super().publish_state(q, v)
        if self.sim.cable is None:
            return
        center = self.sim.cable.centerline
        stride = self.cable_config["display"]["marker_stride"]
        indices = list(range(0, len(center) - 1, stride)) + [len(center) - 1]
        line = Marker()
        line.header.frame_id, line.header.stamp = "world", stamp(self.sim.time)
        line.ns, line.id, line.type, line.action = self.cable_solver + "_cable", 0, Marker.LINE_STRIP, Marker.ADD
        line.pose.orientation.w = 1.
        line.scale.x = self.cable_config["cable"]["diameter"]
        line.color.r, line.color.g, line.color.b, line.color.a = 1., .32, .03, 1.
        line.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in center[indices]]
        self.marker_pub.publish(MarkerArray(markers=[line]))
        wall = time.monotonic()
        if wall - self.last_diagnostic >= 1.:
            values = self.sim.cable.diagnostics()
            values["achieved_realtime_factor"] = self.sim.time / max(wall - self.wall_start, .001)
            status = DiagnosticStatus(name="usb_cable_demo/" + ("MPM" if self.cable_solver == "mpm" else self.cable_solver), hardware_id="ManiSkill2/" + self.cable_solver,
                level=DiagnosticStatus.OK, message=("MPM with axial XPBD fibers" if self.cable_solver == "mpm" else "PhysX capsule chain with D6 joints"),
                values=[KeyValue(key=k, value=json.dumps(v)) for k, v in values.items()])
            message = DiagnosticArray(status=[status])
            message.header = line.header
            self.diag_pub.publish(message)
            self.last_diagnostic = wall


def main():
    bridge_main(UsbCableBridge)
