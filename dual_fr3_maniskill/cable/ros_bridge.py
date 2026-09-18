"""Measured cable markers and reset service; arm actions use the existing bridge."""
import json
import time
import numpy as np
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.action import GoalResponse
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from dual_fr3_maniskill.ros.bridge import ManiSkillBridge, main as bridge_main, stamp
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_maniskill.cable.model import load_config, USB_LINK


class UsbCableBridge(ManiSkillBridge):
    def __init__(self):
        super().__init__()
        self._grasp_waiters = []
        from dual_fr3_maniskill.usb.bridge import InsertionBridge
        self.insertion_control = InsertionBridge(self)
        self._planning_reset = None
        self._planning_reset_client = None
        if self.cable_config.get("insertion", {}).get("enabled", False):
            from moveit_msgs.srv import ApplyPlanningScene
            self._planning_reset_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.declare_parameter("usb_preparation_poses", "")
        self.grasp_pub = self.create_publisher(String, "/maniskill/usb/grasp_state", 10)
        self._last_grasp_state = None
        self.usb_tf = TransformBroadcaster(self)
        group = ReentrantCallbackGroup()
        self.usb_services = [
            self.create_service(Trigger, "/maniskill/cable/spawn", self.spawn_cable, callback_group=group),
            self.create_service(Trigger, "/maniskill/usb/release", self.release_usb, callback_group=group),
            self.create_service(Trigger, "/maniskill/usb/release_manual", self.release_manual, callback_group=group),
            self.create_service(Trigger, "/maniskill/usb/verify", self.verify_usb, callback_group=group),
            self.create_service(Trigger, "/maniskill/usb/status", self.usb_status, callback_group=group),
        ]
        trace_dir = self.declare_parameter("cable_trace_dir", "").value
        if trace_dir and self.load_cable:
            if self.cable_solver != "rope_actor":
                raise ValueError("cable_trace_dir requires cable_solver:=rope_actor")
            from dual_fr3_maniskill.cable.rope_diagnostics import RopeRunRecorder
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
        self.get_logger().info("USB contact grasp: spawn with open fingers, close, release, verify. "
                               f"Cable backend: {self.cable_solver if self.load_cable else 'disabled'}")

    def create_simulation(self, assets):
        from dual_fr3_maniskill.scenes.usb_cable import UsbCableSimulation
        config_path = self.declare_parameter("cable_config", resolve_cable_config()).value
        self.cable_solver = self.declare_parameter("cable_solver", "mpm").value
        self.load_cable = self.declare_parameter("load_cable", True).value
        self.cable_config = load_config(config_path, solver=self.cable_solver if self.load_cable else None)
        return UsbCableSimulation(assets, cable_config=self.cable_config, cable_solver=self.cable_solver, load_cable=self.load_cable,
            control_freq=self.config["control_freq"], sim_freq=self.config["sim_freq"], viewer=self.config["viewer"])

    def accept_gripper(self, side, goal):
        if self.failure or self._grasp_waiters or getattr(getattr(self, "insertion_control", None), "owner", False):
            return GoalResponse.REJECT
        if side == "left" and self.sim.env.support_drive is not None:
            index = self.sim.indices["left_fr3_finger_joint1"]
            monitor = self.sim.env.grasp_monitor
            if (goal.command.position < self.sim.positions[index] and
                    (monitor.state in ("failed", "not_created") or not monitor.external_support)):
                self.get_logger().warning("USB grasp preparation failed; reset before closing again (opening remains allowed)")
                return GoalResponse.REJECT
        return super().accept_gripper(side, goal)

    def start_gripper(self, side, handle):
        insertion = getattr(self.sim.env, "insertion", None)
        if side == "right" and insertion is not None and (insertion.policy.retention_active or insertion.policy.state == 'right_releasing'):
            minimum = insertion.config.get("release_min_half_width_m", {}).get("right", .014)
            if handle.request.command.position >= minimum and self.sim.env.cable is not None:
                # The two physical half-bores now separate. Keep contacts and
                # USB/cable joint; only retire the closed-hole threading assertion.
                self.sim.env.cable.guide_release_started = True
        if side == "left" and self.sim.env.support_drive is not None:
            index = self.sim.indices["left_fr3_finger_joint1"]
            if handle.request.command.position < self.sim.positions[index]:
                try:
                    self.sim.env.grasp_monitor.begin_closing(self.sim.env._grasp_time)
                except ValueError as exc:
                    # A physics observation may have failed after goal acceptance.
                    # Complete the accepted action explicitly and release its
                    # reservation rather than leaking it out of the executor.
                    super().start_gripper(side, handle)
                    self.get_logger().warning("USB closing aborted: " + str(exc))
                    self.finish_gripper(side)
                    return
        return super().start_gripper(side, handle)

    def spawn_cable(self, request, response):
        pending = getattr(self, "_planning_reset", None)
        if pending is not None:
            if not pending.done() or pending.exception() or not pending.result().success:
                response.success, response.message = False, "Planning scene reset pending or failed; retry reset"
                return response
            self._planning_reset = None
        if self.failure:
            response.success, response.message = False, self.failure
        elif self.sim.env.plug is not None:
            response.success, response.message = True, "USB already created; unchanged"
        elif self.reserved or self._grasp_waiters:
            response.success, response.message = False, "Wait for active operations before creating USB"
        else:
            indices = [self.sim.indices[f"left_fr3_finger_joint{i}"] for i in (1, 2)]
            minimum = self.cable_config["usb"].get("open_finger_position", .02)-.001
            if min(self.sim.positions[indices]) < minimum:
                response.success, response.message = False, "Open left fingers before creating USB"
                return response
            try:
                payload = self.get_parameter("usb_preparation_poses").value
                targets = self._preparation_targets(json.loads(payload)) if payload else None
                self.sim.env.spawn_cable(targets)
                response.success = True
                response.message = "Dynamic USB positioned by removable WORLD support; " + (
                    "cable created" if self.load_cable else "cable disabled")
            except Exception as exc:
                self.sim.env.grasp_monitor.fail("creation_failed: " + str(exc), self.sim.env._grasp_time)
                response.success, response.message = False, "USB creation failed: " + str(exc)
        return response

    def _preparation_targets(self, payload):
        """Resolve keypoint frames once, at the serialized spawn boundary."""
        from dual_fr3_maniskill.engine.sapien_compat import sapien
        required = {"left", "right"} if self.load_cable else {"left"}
        if not isinstance(payload, dict) or not required.issubset(payload):
            raise ValueError("Missing preparation TCP poses: "+str(sorted(required)))
        targets = {}
        env = self.sim.env
        for side in required:
            raw = payload[side]
            p, q = np.asarray(raw["position_m"], dtype=float), np.asarray(raw["quaternion_wxyz"], dtype=float)
            if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all() or np.linalg.norm(q) < 1.e-12:
                raise ValueError("Invalid preparation pose for "+side)
            pose = sapien.Pose(p, q/np.linalg.norm(q))
            frame = raw["frame"]
            if frame != "world":
                actor = env.agent.links.get(frame, env.fixtures.get(frame))
                if actor is None:
                    raise ValueError("Unknown preparation frame: "+str(frame))
                pose = actor.pose * pose
            targets[side] = pose
        return targets

    def publish_grasp_state(self):
        snapshot = self.sim.env.grasp_monitor.snapshot()
        self.grasp_pub.publish(String(data=json.dumps(snapshot, allow_nan=False)))
        current = (snapshot["state"], snapshot["reason"])
        if current != self._last_grasp_state:
            self.get_logger().info("USB grasp: %s; reason=%s; external_support=%s; slip_m=%s" % (
                *current, snapshot["external_support"], snapshot["relative_translation_m"]))
            self._last_grasp_state = current

    async def release_usb(self, request, response):
        return await self._wait_grasp("release", response)

    async def verify_usb(self, request, response):
        return await self._wait_grasp("verify", response)

    async def _wait_grasp(self, operation, response):
        env = self.sim.env
        if self.failure or env.plug is None or self.reserved or self._grasp_waiters:
            response.success, response.message = False, self.failure or "USB missing or another operation is active"
            return response
        if operation == "verify" and env.support_drive is not None:
            response.success, response.message = False, "USB is still externally supported; release first"
            return response
        if operation == "release" and env.support_drive is None:
            response.success, response.message = True, "Positioning already released; unchanged (verify separately)"
            return response
        future = Future()
        self._grasp_waiters.append((operation, response, future, self.sim.time))
        return await future

    def _complete_grasp_waiters(self):
        env = self.sim.env
        for operation, response, future, start in list(self._grasp_waiters):
            snapshot = env.grasp_monitor.snapshot()
            state = snapshot["state"]
            complete, success = False, False
            reason = snapshot["reason"]
            if self.failure or state in ("failed", "dropped", "slipping"):
                complete, reason = True, self.failure or reason
            elif operation == "release" and env.grasp_monitor.ready_to_release:
                try:
                    env.release_support()
                    complete, success, reason = True, True, "WORLD support removed; verification required"
                except Exception as exc:
                    complete, reason = True, str(exc)
            elif operation == "verify" and state == "stable":
                complete, success, reason = True, True, "USB stable after release under bilateral finger contact"
            elif self.sim.time-start > float(self.cable_config.get("grasp", {}).get(
                    "close_timeout_s" if operation == "release" else "verification_timeout_s", 5. if operation == "release" else 3.)):
                complete, reason = True, operation + "_timeout: " + reason
                env.grasp_monitor.fail(reason, env._grasp_time)
            if complete:
                response.success, response.message = success, reason
                future.set_result(response)
                self._grasp_waiters.remove((operation, response, future, start))

    def release_manual(self, request, response):
        if self.reserved or self._grasp_waiters:
            response.success, response.message = False, "Wait for active operations before manual release"
            return response
        try:
            changed = self.sim.env.release_support(manual=True)
            response.success, response.message = True, "Manual WORLD support release; contact success not assumed" if changed else "Already released; unchanged"
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def usb_status(self, request, response):
        snapshot = self.sim.env.grasp_monitor.snapshot()
        plug = self.sim.env.plug
        snapshot["world_pose"] = (None if plug is None else dict(
            frame="world", position_m=list(map(float, plug.pose.p)),
            quaternion_wxyz=list(map(float, plug.pose.q))))
        response.success = snapshot["stable"]
        response.message = json.dumps(snapshot, allow_nan=False)
        return response

    def accept_arm(self, side, goal):
        if self.failure or self._grasp_waiters or getattr(getattr(self, "insertion_control", None), "owner", False):
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
            self.publish_grasp_state()
            if hasattr(self, "insertion_control"):
                self.insertion_control.publish()
            return
        try:
            if hasattr(self, "insertion_control"):
                self.insertion_control.before_tick()
            super().tick()
            if hasattr(self, "insertion_control"):
                self.insertion_control.publish()
            self._complete_grasp_waiters()
            self.publish_grasp_state()
        except RuntimeError as exc:
            self.failure = str(exc)
            if hasattr(self, "insertion_control"):
                self.insertion_control.relinquish()
            if getattr(self.sim.env, "insertion", None) is not None:
                self.sim.env.insertion.policy.stop("feedback_unavailable", self.failure)
                self.insertion_control.publish()
            self.sim.env.grasp_monitor.fail(self.failure, self.sim.env._grasp_time)
            self.publish_grasp_state()
            self._complete_grasp_waiters()
            if getattr(self, "force_output", None) is not None:
                self.force_output.fail(self.failure)
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

    def close(self):
        if hasattr(self, "insertion_control"):
            if self.insertion_control.owner and self.sim.env.insertion is not None:
                self.sim.env.insertion.policy.stop("cancelled", "simulation_node_shutdown")
                self.insertion_control.publish()
            self.insertion_control.relinquish()
        super().close()

    def reset_cable(self, request, response):
        if self.reserved or self._grasp_waiters:
            response.success, response.message = False, "Wait for or cancel active operations before resetting"
            return response
        try:
            self.sim.env.clear_objects()
        except (RuntimeError, ValueError) as exc:
            response.success, response.message = False, str(exc)
            return response
        collector = getattr(self.sim.env, "force_collector", None)
        if collector is not None:
            collector.active = False
            collector.snapshot = None
        client = getattr(self, "_planning_reset_client", None)
        if client is not None:
            from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
            from moveit_msgs.srv import ApplyPlanningScene
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                object=CollisionObject(id=USB_LINK, operation=CollisionObject.REMOVE))]
            scene.world.collision_objects = [CollisionObject(id=name, operation=CollisionObject.REMOVE)
                                             for name in (USB_LINK, "usb_socket")]
            if not client.service_is_ready():
                response.success, response.message = False, "Physics cleared; MoveIt unavailable, retry reset for planning cleanup"
                self.failure = "planning_scene_reset_required"
                return response
            self._planning_reset = client.call_async(ApplyPlanningScene.Request(scene=scene))
        self.failure = None
        self.publish_grasp_state()
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        response.success, response.message = True, "USB/cable/supports removed; open fingers then spawn again"
        return response

    def publish_state(self, q, v):
        super().publish_state(q, v)
        insertion = getattr(self.sim.env, "insertion", None)
        if insertion is not None:
            from dual_fr3_maniskill.engine.sapien_compat import sapien
            for name, pose in (("usb_socket", insertion.base.pose),
                               ("usb_socket_hole", insertion.base.pose*sapien.Pose(insertion.hole))):
                transform = TransformStamped(child_frame_id=name)
                transform.header.frame_id, transform.header.stamp = "world", stamp(self.sim.time)
                xyz, quat = transform.transform.translation, transform.transform.rotation
                xyz.x, xyz.y, xyz.z = map(float, pose.p)
                quat.w, quat.x, quat.y, quat.z = map(float, pose.q)
                self.usb_tf.sendTransform(transform)
        if self.sim.env.plug is not None:
            pose = self.sim.env.plug.pose
            transform = TransformStamped(child_frame_id=USB_LINK)
            transform.header.frame_id, transform.header.stamp = "world", stamp(self.sim.time)
            xyz, quat = transform.transform.translation, transform.transform.rotation
            xyz.x, xyz.y, xyz.z = map(float, pose.p)
            quat.w, quat.x, quat.y, quat.z = map(float, pose.q)
            self.usb_tf.sendTransform(transform)
            # Runtime USB has no fixed URDF visual; display its measured pose
            # even when no cable/line marker exists.
            marker = Marker()
            marker.header = transform.header
            marker.ns, marker.id = "usb_contact_grasp", 0
            marker.type, marker.action = Marker.MESH_RESOURCE, Marker.ADD
            marker.mesh_resource = "package://dual_fr3_maniskill/meshes/USB1.stl"
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, pose.p)
            marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z = map(float, pose.q)
            marker.scale.x = marker.scale.y = marker.scale.z = float(self.cable_config["usb"]["mesh_scale"])
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .08, .25, .65, 1.
            self.marker_pub.publish(MarkerArray(markers=[marker]))
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
