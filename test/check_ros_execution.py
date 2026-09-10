"""Integration check against a running, isolated ManiSkill launch.

Run with the ROS Humble Python interpreter after sourcing the workspace. This
intentionally moves the simulation and requires the ManiSkill node to be present.
"""
import json
import os
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    if os.environ.get("ROS_DOMAIN_ID", "0") == "0":
        raise RuntimeError("Run integration checks in an isolated nonzero ROS_DOMAIN_ID")
    rclpy.init()
    node = rclpy.create_node("maniskill_execution_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    state, controllers, tcp, feedback_count = {}, {}, {}, {side: 0 for side in ("left", "right")}
    subscriptions = [node.create_subscription(JointState, "/joint_states", lambda msg: state.update(message=msg), 10)]
    for side in ("left", "right"):
        subscriptions.append(node.create_subscription(JointTrajectoryControllerState,
            f"/{side}_fr3_arm_controller/controller_state", lambda msg, side=side: controllers.update({side: msg}), 10))
        subscriptions.append(node.create_subscription(PoseStamped, f"/maniskill/{side}_tcp_pose",
            lambda msg, side=side: tcp.update({side: msg}), 10))
    tf = Buffer()
    listener = TransformListener(tf, node)

    def wait(predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert predicate(), "Integration check timed out"

    def result(future, timeout=30):
        wait(future.done, timeout)
        return future.result()

    def current(side):
        msg = state["message"]
        return [msg.position[msg.name.index(f"{side}_fr3_joint{i}")] for i in range(1, 8)]

    def goal(side, delta, duration=2):
        start = current(side)
        end = start.copy()
        end[0] += delta
        message = FollowJointTrajectory.Goal()
        message.trajectory.joint_names = [f"{side}_fr3_joint{i}" for i in range(1, 8)]
        message.trajectory.points = [
            JointTrajectoryPoint(positions=start, velocities=[0.0]*7, time_from_start=Duration()),
            JointTrajectoryPoint(positions=end, velocities=[0.0]*7, time_from_start=Duration(sec=duration)),
        ]
        return message

    try:
        wait(lambda: "dual_fr3_maniskill" in node.get_node_names())
        if os.environ.get("MANISKILL_CHECK_RVIZ") == "1":
            wait(lambda: "rviz2" in node.get_node_names())
        wait(lambda: "message" in state and len(controllers) == 2)
        assert node.count_publishers("/joint_states") == 1, "Duplicate joint-state sources"
        arms = {side: ActionClient(node, FollowJointTrajectory, f"/{side}_fr3_arm_controller/follow_joint_trajectory")
                for side in ("left", "right")}
        hands = {side: ActionClient(node, GripperCommand, f"/{side}_franka_gripper/gripper_cmd")
                 for side in ("left", "right")}
        for client in list(arms.values()) + list(hands.values()):
            assert client.wait_for_server(timeout_sec=10)
        initial = {side: current(side) for side in arms}
        goals = {side: goal(side, 0.08 if side == "left" else -0.08) for side in arms}
        common_start = node.get_clock().now().nanoseconds + 1_000_000_000
        for message in goals.values():
            message.trajectory.header.stamp = Time(nanoseconds=common_start).to_msg()

        def feedback(side, _):
            feedback_count[side] += 1

        pending = {side: client.send_goal_async(goals[side], feedback_callback=lambda msg, side=side: feedback(side, msg))
                   for side, client in arms.items()}
        handles = {side: result(future) for side, future in pending.items()}
        assert all(handle.accepted for handle in handles.values())
        endings = {side: handle.get_result_async() for side, handle in handles.items()}
        for side, future in endings.items():
            wrapped = result(future)
            assert wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.error_code == 0, wrapped
        wait(lambda: all(abs(current(side)[0] - goals[side].trajectory.points[-1].positions[0]) < 0.005 for side in arms))
        assert all(value > 0 for value in feedback_count.values())
        print("PASS: simultaneous arm trajectories and measured action feedback", flush=True)

        for opening in (0.015, 0.04):
            futures = {}
            for side, client in hands.items():
                message = GripperCommand.Goal()
                message.command.position, message.command.max_effort = opening, 15.0
                futures[side] = client.send_goal_async(message)
            hand_handles = {side: result(future) for side, future in futures.items()}
            for handle in hand_handles.values():
                assert handle.accepted
                wrapped = result(handle.get_result_async())
                assert wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.reached_goal, wrapped
            wait(lambda: all(abs(state["message"].position[state["message"].name.index(f"{side}_fr3_finger_joint1")] - opening) < 0.001 for side in hands))
        print("PASS: both grippers close/open with measured position results", flush=True)

        handle = result(arms["left"].send_goal_async(goal("left", 0.08, 4)))
        assert handle.accepted
        until = node.get_clock().now().nanoseconds + 300_000_000
        wait(lambda: node.get_clock().now().nanoseconds >= until)
        assert result(handle.cancel_goal_async()).goals_canceling
        assert result(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED
        print("PASS: trajectory cancellation", flush=True)

        invalid = goal("left", 0.01)
        invalid.trajectory.joint_names[0] = "right_fr3_joint1"
        assert not result(arms["left"].send_goal_async(invalid)).accepted
        failing = goal("left", 0.08)
        failing.path_tolerance = [JointTolerance(name="left_fr3_joint1", position=1e-10)]
        handle = result(arms["left"].send_goal_async(failing))
        assert handle.accepted
        wrapped = result(handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED and wrapped.result.error_code == -4, wrapped
        print("PASS: invalid-joint rejection and physical tolerance failure", flush=True)

        # Return both arms to their initial positions after checks.
        futures = {}
        for side, client in arms.items():
            message = goal(side, 0.0)
            message.trajectory.points[-1].positions = initial[side]
            futures[side] = client.send_goal_async(message)
        for future in futures.values():
            handle = result(future)
            assert handle.accepted and result(handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
        wait(lambda: len(tcp) == 2)
        errors = {}
        for side in arms:
            measured = tcp[side]
            at = Time.from_msg(measured.header.stamp)
            wait(lambda side=side: tf.can_transform("world", f"{side}_fr3_hand_tcp", at))
            transform = tf.lookup_transform("world", f"{side}_fr3_hand_tcp", at)
            p, actual = transform.transform.translation, measured.pose.position
            position_error = float(np.linalg.norm([p.x-actual.x, p.y-actual.y, p.z-actual.z]))
            a, b = transform.transform.rotation, measured.pose.orientation
            q1, q2 = np.array([a.x, a.y, a.z, a.w]), np.array([b.x, b.y, b.z, b.w])
            cosine = abs(np.dot(q1, q2) / (np.linalg.norm(q1) * np.linalg.norm(q2)))
            rotation_error = float(2 * np.arccos(np.clip(cosine, 0.0, 1.0)))
            errors[side] = {"position_m": position_error, "rotation_rad": rotation_error}
            assert position_error < 1e-4 and rotation_error < 1e-4, errors
        print("PASS: RViz TF matches ManiSkill TCP " + json.dumps(errors), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
