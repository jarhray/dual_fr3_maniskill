"""Construct exactly one physics bridge for the selected scene."""
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from .scenes import resolve_cable_config, scene_spec


def create_bridge_node(*, scene, robot_description, robot_description_semantic,
                       python, viewer, simulation_config="", cable_config=""):
    spec = scene_spec(scene)
    options = {"viewer": ParameterValue(viewer, value_type=bool)}
    if scene in ("usb_cable", "trunking_cable"):
        options["cable_config"] = resolve_cable_config(cable_config, scene=scene)
    return Node(
        package="dual_fr3_maniskill", executable=spec.bridge_executable,
        prefix=[python], output="screen",
        parameters=[simulation_config or spec.simulation_config,
                    robot_description, robot_description_semantic, options],
    )
