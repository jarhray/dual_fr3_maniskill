"""Construct exactly one physics bridge for the selected scene."""
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from .cable.backends import validate_solver
from .scenes import resolve_cable_config, scene_spec


def create_bridge_node(*, scene, robot_description, robot_description_semantic,
                       python, viewer, simulation_config="", cable_config="", cable_solver="mpm",
                       cable_trace_dir="", leader_orientation_direction="reverse", load_cable=True):
    validate_solver(cable_solver)
    if cable_trace_dir and (scene not in ("usb_cable", "trunking_cable") or cable_solver != "rope_actor"):
        raise ValueError("cable_trace_dir requires a cable scene and cable_solver:=rope_actor")
    spec = scene_spec(scene)
    options = {"viewer": ParameterValue(viewer, value_type=bool)}
    if scene in ("usb_cable", "trunking_cable"):
        options["load_cable"] = ParameterValue(load_cable, value_type=bool)
        options["cable_solver"] = cable_solver
        options["cable_config"] = resolve_cable_config(cable_config, scene=scene)
        options["cable_trace_dir"] = ParameterValue(cable_trace_dir, value_type=str)
    if scene == "trunking_cable":
        options["leader_orientation_direction"] = ParameterValue(
            leader_orientation_direction, value_type=str)
    return Node(
        package="dual_fr3_maniskill", executable=spec.bridge_executable,
        prefix=[python], output="screen",
        parameters=[simulation_config or spec.simulation_config,
                    robot_description, robot_description_semantic, options],
    )
