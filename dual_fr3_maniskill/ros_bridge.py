"""ROS actions and feedback driven exclusively by measured ManiSkill state."""
from __future__ import annotations

from dataclasses import dataclass
from functools import partial
import math
from pathlib import Path
import tempfile
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .assets import prepare_assets
from .trajectory import Trajectory, seconds, tolerances


SIDES = ("left", "right")


def stamp(value, cls=Time):
    ns = max(0, round(value * 1_000_000_000))
    return cls(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def point(position, velocity, acceleration, elapsed=0.0):
    return JointTrajectoryPoint(
        positions=position.tolist(), velocities=velocity.tolist(),
        accelerations=acceleration.tolist(), time_from_start=stamp(elapsed, Duration))


@dataclass
class ArmJob:
    handle: object
    done: Future
    trajectory: Trajectory
    start: float
    path_tolerance: np.ndarray
    goal_tolerance: np.ndarray
    grace: float


@dataclass
class GripperJob:
    handle: object
    done: Future
    position: float
    start: float
    last_motion: float
    effort: float


class ManiSkillBridge(Node):
    def __init__(self):
        super().__init__("dual_fr3_maniskill")
        defaults = {
            "robot_description": "", "robot_description_semantic": "",
            "viewer": False, "control_freq": 100, "sim_freq": 500,
            "publish_freq": 50, "realtime_factor": 1.0,
            "path_position_tolerance": 0.2, "goal_position_tolerance": 0.005,
            "goal_velocity_tolerance": 0.02, "goal_time_tolerance": 3.0,
            "gripper_speed": 0.04, "gripper_force": 40.0,
            "gripper_goal_tolerance": 0.001, "gripper_stall_timeout": 0.5,
            "gripper_timeout": 5.0,
        }
        self.config = {name: self.declare_parameter(name, default).value
                       for name, default in defaults.items()}
        c = self.config
        for name, value in c.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{name} must be finite and positive")
        if c["sim_freq"] % c["control_freq"] or c["control_freq"] % c["publish_freq"]:
            raise ValueError("sim_freq must divide by control_freq, and control_freq by publish_freq")
        if not c["robot_description"] or not c["robot_description_semantic"]:
            raise ValueError("robot_description and robot_description_semantic are required")

        self.cache = tempfile.TemporaryDirectory(prefix="dual_fr3_maniskill_")
        self.assets = prepare_assets(c["robot_description"], c["robot_description_semantic"], Path(self.cache.name))
        self.sim = self.create_simulation(self.assets)
        self.dt = 1.0 / c["control_freq"]
        self.arm_names = {side: [f"{side}_fr3_joint{i}" for i in range(1, 8)] for side in SIDES}
        self.arm_indices = {side: [self.sim.indices[name] for name in names]
                            for side, names in self.arm_names.items()}
        self.arms = {}
        self.pending_arms = {}
        self.grippers = {}
        self.reserved = set()
        self.desired = {side: (self.sim.positions[indices], np.zeros(7), np.zeros(7))
                        for side, indices in self.arm_indices.items()}
        self.last_velocity = self.sim.velocities.copy()
        self.acceleration = np.zeros(len(self.sim.names))
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.state_pubs = {side: self.create_publisher(
            JointTrajectoryControllerState, f"/{side}_fr3_arm_controller/controller_state", 10)
            for side in SIDES}
        self.tcp_pubs = {side: self.create_publisher(
            PoseStamped, f"/maniskill/{side}_tcp_pose", 10) for side in SIDES
            if f"{side}_fr3_hand_tcp" in self.sim.link_names}
        self.actions = []
        group = ReentrantCallbackGroup()
        for side in SIDES:
            self.actions.append(ActionServer(
                self, FollowJointTrajectory, f"/{side}_fr3_arm_controller/follow_joint_trajectory",
                partial(self.execute_arm, side), callback_group=group,
                goal_callback=partial(self.accept_arm, side),
                handle_accepted_callback=partial(self.start_arm, side),
                cancel_callback=lambda _: CancelResponse.ACCEPT))
            if f"{side}_fr3_finger_joint1" in self.sim.indices:
                self.actions.append(ActionServer(
                    self, GripperCommand, f"/{side}_franka_gripper/gripper_cmd",
                    partial(self.execute_gripper, side), callback_group=group,
                    goal_callback=partial(self.accept_gripper, side),
                    handle_accepted_callback=partial(self.start_gripper, side),
                    cancel_callback=lambda _: CancelResponse.ACCEPT))
        self.get_logger().info(
            f"ManiSkill2 ready: {len(self.sim.names)} joints, CPU physics {c['sim_freq']} Hz, "
            f"control {c['control_freq']} Hz; measured state → /joint_states")

    def create_simulation(self, assets):
        """Allow isolated experiments to reuse measured-state ROS execution."""
        try:
            from .simulation import Simulation
        except ImportError as exc:
            raise RuntimeError("Use maniskill_python pointing to the ManiSkill2 / SAPIEN 2 venv; source ROS Humble first") from exc
        return Simulation(assets, control_freq=self.config["control_freq"],
                          sim_freq=self.config["sim_freq"], viewer=self.config["viewer"])

    def validate_arm(self, side, goal):
        if (goal.multi_dof_trajectory.points or goal.component_path_tolerance
                or goal.component_goal_tolerance):
            raise ValueError("Multi-DOF/component trajectories are not supported")
        names, indices = self.arm_names[side], self.arm_indices[side]
        trajectory = Trajectory.from_message(goal.trajectory, names, self.sim.positions[indices],
                                             self.sim.velocities[indices], self.assets.limits)
        start = seconds(goal.trajectory.header.stamp)
        if start != 0 and start < self.sim.time:
            raise ValueError("Trajectory header is in the past")
        path = tolerances(goal.path_tolerance, names,
                          [self.config["path_position_tolerance"], math.inf, math.inf])
        target = tolerances(goal.goal_tolerance, names,
                            [self.config["goal_position_tolerance"],
                             self.config["goal_velocity_tolerance"], math.inf])
        grace = seconds(goal.goal_time_tolerance) or self.config["goal_time_tolerance"]
        return trajectory, start, path, target, grace

    def accept_arm(self, side, goal):
        key = (side, "arm")
        try:
            if key in self.reserved:
                raise ValueError(f"{side} arm is busy; cancel the current goal first")
            self.pending_arms[side] = self.validate_arm(side, goal)
        except ValueError as exc:
            self.get_logger().warning(f"Rejected {side} trajectory: {exc}")
            return GoalResponse.REJECT
        self.reserved.add(key)
        return GoalResponse.ACCEPT

    def start_arm(self, side, handle):
        trajectory, start, path, target, grace = self.pending_arms.pop(side)
        self.arms[side] = ArmJob(handle, Future(), trajectory, start or self.sim.time,
                                 path, target, grace)
        handle.maniskill_done = self.arms[side].done
        handle.execute()

    async def execute_arm(self, side, handle):
        return await handle.maniskill_done

    def finish_arm(self, side, code, message, *, cancel=False):
        job = self.arms.pop(side)
        if cancel or code != FollowJointTrajectory.Result.SUCCESSFUL:
            indices = self.arm_indices[side]
            self.sim.target[indices] = self.sim.positions[indices]
            self.desired[side] = (self.sim.target[indices].copy(), np.zeros(7), np.zeros(7))
        if cancel:
            job.handle.canceled()
        elif code == FollowJointTrajectory.Result.SUCCESSFUL:
            job.handle.succeed()
        else:
            job.handle.abort()
            self.get_logger().error(f"{side}: {message}")
        job.done.set_result(FollowJointTrajectory.Result(error_code=code, error_string=message))
        self.reserved.discard((side, "arm"))

    def accept_gripper(self, side, goal):
        key = (side, "gripper")
        lower, upper = self.assets.limits[f"{side}_fr3_finger_joint1"][:2]
        if (key in self.reserved or not math.isfinite(goal.command.position)
                or not lower <= goal.command.position <= upper
                or not math.isfinite(goal.command.max_effort) or goal.command.max_effort < 0):
            return GoalResponse.REJECT
        self.reserved.add(key)
        return GoalResponse.ACCEPT

    def start_gripper(self, side, handle):
        effort = min(handle.request.command.max_effort or self.config["gripper_force"],
                     self.config["gripper_force"], self.assets.limits[f"{side}_fr3_finger_joint1"][2])
        self.sim.set_gripper_force(side, effort)
        self.grippers[side] = GripperJob(handle, Future(), handle.request.command.position,
                                       self.sim.time, self.sim.time, effort)
        handle.maniskill_done = self.grippers[side].done
        handle.execute()

    async def execute_gripper(self, side, handle):
        return await handle.maniskill_done

    def finish_gripper(self, side, *, reached=False, stalled=False, cancel=False):
        job = self.grippers.pop(side)
        index = self.sim.indices[f"{side}_fr3_finger_joint1"]
        if cancel or not (reached or stalled):
            self.sim.target[index] = self.sim.positions[index]
        if cancel:
            job.handle.canceled()
        elif reached or stalled:
            job.handle.succeed()
        else:
            job.handle.abort()
        # Effort is left at zero: a drive limit is not a measured contact force.
        job.done.set_result(GripperCommand.Result(
            position=float(self.sim.positions[index]), effort=0.0,
            reached_goal=bool(reached), stalled=bool(stalled)))
        self.reserved.discard((side, "gripper"))

    def tick(self):
        t = self.sim.time + self.dt
        for side, job in list(self.arms.items()):
            if job.handle.is_cancel_requested:
                self.finish_arm(side, 0, "Cancelled; holding measured position", cancel=True)
                continue
            if t < job.start:
                continue
            desired = job.trajectory.sample(t - job.start)
            if any(not self.assets.limits[name][0] - 1e-6 <= value <= self.assets.limits[name][1] + 1e-6
                   for name, value in zip(self.arm_names[side], desired[0])):
                self.finish_arm(side, FollowJointTrajectory.Result.INVALID_GOAL,
                                "Interpolated trajectory exceeds joint limits")
                continue
            self.sim.target[self.arm_indices[side]] = desired[0]
            self.desired[side] = desired
        for side, job in list(self.grippers.items()):
            if job.handle.is_cancel_requested:
                self.finish_gripper(side, cancel=True)
                continue
            index = self.sim.indices[f"{side}_fr3_finger_joint1"]
            step = self.config["gripper_speed"] * self.dt
            self.sim.target[index] += np.clip(job.position - self.sim.target[index], -step, step)
        self.sim.step()
        q, v = self.sim.positions, self.sim.velocities
        self.acceleration = (v - self.last_velocity) / self.dt
        self.last_velocity = v.copy()
        self.clock_pub.publish(Clock(clock=stamp(self.sim.time)))
        for side, job in list(self.arms.items()):
            elapsed = self.sim.time - job.start
            if elapsed < 0:
                continue
            indices = self.arm_indices[side]
            desired = self.desired[side]
            errors = np.column_stack((desired[0] - q[indices], desired[1] - v[indices],
                                      desired[2] - self.acceleration[indices]))
            if elapsed < job.trajectory.duration:
                if np.any(np.abs(errors) > job.path_tolerance):
                    self.finish_arm(side, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                    f"Path tolerance violated; maximum position error {max(abs(errors[:, 0])):.5f}")
            elif np.all(np.abs(errors) <= job.goal_tolerance):
                self.finish_arm(side, FollowJointTrajectory.Result.SUCCESSFUL, "Measured goal reached")
            elif elapsed > job.trajectory.duration + job.grace:
                self.finish_arm(side, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                f"Goal tolerance violated; maximum position error {max(abs(errors[:, 0])):.5f}")
        for side, job in list(self.grippers.items()):
            index = self.sim.indices[f"{side}_fr3_finger_joint1"]
            if abs(v[index]) > 0.001:
                job.last_motion = self.sim.time
            reached = abs(q[index] - job.position) <= self.config["gripper_goal_tolerance"] and abs(v[index]) < 0.005
            stalled = (not reached and self.sim.time - job.last_motion >= self.config["gripper_stall_timeout"])
            if reached or stalled:
                self.finish_gripper(side, reached=reached, stalled=stalled)
            elif self.sim.time - job.start > self.config["gripper_timeout"]:
                self.finish_gripper(side)
        if self.sim.steps % (self.config["control_freq"] // self.config["publish_freq"]) == 0:
            self.publish_state(q, v)

    def publish_state(self, q, v):
        now = stamp(self.sim.time)
        state = JointState(name=self.sim.names, position=q.tolist(), velocity=v.tolist())
        state.header.stamp = now
        self.joint_pub.publish(state)
        for side, publisher in self.tcp_pubs.items():
            raw = self.sim.link_pose(f"{side}_fr3_hand_tcp")
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = "world", now
            message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, raw[:3])
            message.pose.orientation.w, message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z = map(float, raw[3:])
            publisher.publish(message)
        for side, indices in self.arm_indices.items():
            job = self.arms.get(side)
            elapsed = max(0, self.sim.time - job.start) if job else 0.0
            desired = point(*self.desired[side], elapsed)
            actual = point(q[indices], v[indices], self.acceleration[indices], elapsed)
            error = point(self.desired[side][0] - q[indices], self.desired[side][1] - v[indices],
                          self.desired[side][2] - self.acceleration[indices], elapsed)
            message = JointTrajectoryControllerState(joint_names=self.arm_names[side],
                                                     desired=desired, actual=actual, error=error)
            message.header.stamp = now
            # New control_msgs also exposes reference/feedback; fill both layouts.
            if hasattr(message, "reference"):
                message.reference, message.feedback = desired, actual
            self.state_pubs[side].publish(message)
            if job:
                feedback = FollowJointTrajectory.Feedback(joint_names=self.arm_names[side],
                                                          desired=desired, actual=actual, error=error)
                feedback.header.stamp = now
                job.handle.publish_feedback(feedback)
        for side, job in self.grippers.items():
            index = self.sim.indices[f"{side}_fr3_finger_joint1"]
            job.handle.publish_feedback(GripperCommand.Feedback(position=float(q[index]),
                                                                 effort=0.0, reached_goal=False, stalled=False))

    def close(self):
        try:
            if rclpy.ok():
                for side in list(self.arms):
                    self.finish_arm(side, FollowJointTrajectory.Result.INVALID_GOAL, "Simulator shutting down")
                for side in list(self.grippers):
                    self.finish_gripper(side)
            for action in self.actions:
                action.destroy()
        finally:
            self.sim.close()
            self.cache.cleanup()


def main(node_factory=ManiSkillBridge):
    rclpy.init()
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = node_factory()
        executor.add_node(node)
        period = node.dt / node.config["realtime_factor"]
        next_tick = time.monotonic()
        while rclpy.ok():
            executor.spin_once(timeout_sec=max(0.0, min(period, next_tick - time.monotonic())))
            if time.monotonic() >= next_tick:
                node.tick()
                next_tick = max(next_tick + period, time.monotonic())
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
