#!/usr/bin/env python3
"""Validate a running, completed USB-only MTC grasp through real ROS interfaces.

Uses the current ROS_DOMAIN_ID. Opens the simulated left gripper and resets
objects after checking status, planning attachment and existing force protocols.
Start the documented USB-only MTC command first; output must not already exist.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from std_msgs.msg import String
from std_srvs.srv import Trigger


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def wait_result(node, future, operation, timeout=10.):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f"Timed out after {timeout:g} s waiting for {operation}")
    if future.cancelled():
        raise RuntimeError(f"{operation} was cancelled")
    try:
        result = future.result()
    except Exception as exc:
        raise RuntimeError(f"{operation} failed: {exc}") from exc
    require(result is not None, f"{operation} returned no response")
    return result


def wait_observation(node, predicate, description, timeout=3.):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Timed out after {timeout:g} s waiting for {description}")
        rclpy.spin_once(node, timeout_sec=.1)


def call_trigger(node, name):
    client = node.create_client(Trigger, name)
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise TimeoutError(f"Service {name} was unavailable after 5 s")
        return wait_result(node, client.call_async(Trigger.Request()), name)
    finally:
        node.destroy_client(client)


def attached_objects(node):
    name = "/get_planning_scene"
    client = node.create_client(GetPlanningScene, name)
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise TimeoutError(f"Service {name} was unavailable after 5 s")
        request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
        response = wait_result(node, client.call_async(request), name)
        return [obj.object.id for obj in response.scene.robot_state.attached_collision_objects]
    finally:
        node.destroy_client(client)


def open_left_gripper(node):
    name = "/left_franka_gripper/gripper_cmd"
    client = ActionClient(node, GripperCommand, name)
    try:
        if not client.wait_for_server(timeout_sec=5.):
            raise TimeoutError(f"Action server {name} was unavailable after 5 s")
        goal = GripperCommand.Goal()
        goal.command.position = .02
        goal.command.max_effort = 10.
        handle = wait_result(node, client.send_goal_async(goal), name+" goal acceptance")
        require(handle.accepted, "Left gripper rejected the opening goal")
        wait_result(node, handle.get_result_async(), name+" opening result")
    finally:
        client.destroy()


def validate_grasp(node, latest):
    status = call_trigger(node, "/maniskill/usb/status")
    require(status.success, "Completed stable USB-only MTC grasp required: "+status.message)
    report = {"initial_status": json.loads(status.message)}

    attached = attached_objects(node)
    require("usb_cable_demo_plug" in attached, f"Stable USB is missing from planning attachments: {attached}")
    report["attached_after_stable"] = attached
    for name in ("/maniskill/cable/spawn", "/maniskill/usb/release"):
        response = call_trigger(node, name)
        require(response.success, f"Repeated {name} failed: {response.message}")
        report[name] = response.message

    wait_observation(node, lambda: bool(latest), "/maniskill/forces JSON")
    snapshot = latest[-1]
    require(snapshot["cable"]["enabled"] is False and snapshot["cable"]["created"] is False,
            "This check requires USB-only mode with no cable backend")
    sensors = snapshot["sensors"]
    require(not sensors["left/usb_base"]["available"], "Missing USB base must remain unavailable")
    require(sensors["left/cable"]["force_N"] is None, "Disabled cable force must be null")
    for finger in ("left_fr3_leftfinger", "left_fr3_rightfinger"):
        load = sensors[f"fingers/{finger}/usb"]["normal_load_N"]
        require(load is not None and load > .1, f"Insufficient USB contact on {finger}: {load} N")
    report["force_snapshot"] = snapshot

    open_left_gripper(node)
    wait_observation(node, lambda: latest[-1].get("usb_grasp", {}).get("state") == "dropped",
                     "USB dropped state after opening")
    status = call_trigger(node, "/maniskill/usb/status")
    grasp = json.loads(status.message)
    require(not status.success and grasp["state"] == "dropped", f"USB drop was not confirmed: {grasp}")
    report["after_open"] = grasp

    response = call_trigger(node, "/usb_cable_demo/reset")
    require(response.success, "Object reset failed: "+response.message)
    status = call_trigger(node, "/maniskill/usb/status")
    grasp = json.loads(status.message)
    require(grasp["state"] == "not_created", f"Reset retained USB state: {grasp}")
    report["reset_status"] = grasp
    report["passed"] = True
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    rclpy.init()
    node = None
    try:
        node = rclpy.create_node("usb_grasp_acceptance")
        latest = []
        grasp_states = []
        node.create_subscription(String, "/maniskill/forces",
                                 lambda message: latest.append(json.loads(message.data)), 10)
        node.create_subscription(String, "/maniskill/usb/grasp_state",
                                 lambda message: grasp_states.append(json.loads(message.data)), 10)
        report = validate_grasp(node, latest)
        wait_observation(node, lambda: any(s["state"] == "not_created" for s in grasp_states),
                         "reset on independent grasp-state topic")
        for state in ("stable", "dropped", "not_created"):
            require(any(s["state"] == state for s in grasp_states), "Missing streamed grasp state: "+state)
        stable = next(s for s in grasp_states if s["state"] == "stable")
        require(stable["creation_world_pose"] is not None and stable["relative_displacement_m"] is not None,
                "Grasp stream is missing creation/world or relative displacement observations")
        report["grasp_state_topic"] = dict(topic="/maniskill/usb/grasp_state", stable=stable,
                                         states=sorted({s["state"] for s in grasp_states}))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(report, indent=2)+"\n")
        print(json.dumps(dict(passed=True, attached=report["attached_after_stable"],
            open_state=report["after_open"]["state"], reset_state=report["reset_status"]["state"])))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
