"""Physics-only launch; the caller supplies the same model used by MoveIt."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("dual_fr3_maniskill")
    python = os.environ.get("MANISKILL_PYTHON", str(Path.cwd() / ".venv/bin/python"))
    return LaunchDescription([
        DeclareLaunchArgument("robot_description"),
        DeclareLaunchArgument("robot_description_semantic"),
        DeclareLaunchArgument("maniskill_python", default_value=python),
        DeclareLaunchArgument("maniskill_viewer", default_value="false"),
        Node(package="dual_fr3_maniskill", executable="maniskill_bridge.py",
             prefix=[LaunchConfiguration("maniskill_python")], output="screen",
             parameters=[os.path.join(share, "config/simulation.yaml"), {
                 "robot_description": ParameterValue(LaunchConfiguration("robot_description"), value_type=str),
                 "robot_description_semantic": ParameterValue(LaunchConfiguration("robot_description_semantic"), value_type=str),
                 "viewer": ParameterValue(LaunchConfiguration("maniskill_viewer"), value_type=bool),
             }]),
    ])
