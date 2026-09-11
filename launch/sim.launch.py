"""Physics-only launch; the caller supplies the same model used by MoveIt."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from dual_fr3_maniskill.launch_support import create_bridge_node
from dual_fr3_maniskill.scenes import SCENES


def launch_setup(context):
    return [create_bridge_node(
        scene=LaunchConfiguration("maniskill_scene").perform(context),
        robot_description={"robot_description": ParameterValue(LaunchConfiguration("robot_description"), value_type=str)},
        robot_description_semantic={"robot_description_semantic": ParameterValue(
            LaunchConfiguration("robot_description_semantic"), value_type=str)},
        python=LaunchConfiguration("maniskill_python"), viewer=LaunchConfiguration("maniskill_viewer"),
        simulation_config=LaunchConfiguration("maniskill_config").perform(context),
        cable_config=LaunchConfiguration("cable_config").perform(context),
    )]


def generate_launch_description():
    python = os.environ.get("MANISKILL_PYTHON", str(Path.cwd() / ".venv/bin/python"))
    return LaunchDescription([
        DeclareLaunchArgument("robot_description"),
        DeclareLaunchArgument("robot_description_semantic"),
        DeclareLaunchArgument("maniskill_scene", default_value="robot", choices=SCENES),
        DeclareLaunchArgument("cable_config", default_value=""),
        DeclareLaunchArgument("maniskill_config", default_value=""),
        DeclareLaunchArgument("maniskill_python", default_value=python),
        DeclareLaunchArgument("maniskill_viewer", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
