"""Construct exactly one physics bridge for the selected scene."""
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dual_fr3_maniskill.cable.backends import validate_solver
from dual_fr3_maniskill.scenes import resolve_cable_config, scene_spec


def create_bridge_node(*, scene, robot_description, robot_description_semantic,
                       python, viewer, simulation_config="", cable_config="", cable_solver="mpm",
                       cable_trace_dir="", leader_orientation_direction="reverse", load_cable=True, insertion_enabled=False,
                       perception_enabled=False, camera_config=""):
    validate_solver(cable_solver)
    if cable_trace_dir and (scene not in ("usb_cable", "trunking_cable") or cable_solver != "rope_actor"):
        raise ValueError("cable_trace_dir requires a cable scene and cable_solver:=rope_actor")
    spec = scene_spec(scene)
    options = {"viewer": ParameterValue(viewer, value_type=bool),
               "perception_enabled": ParameterValue(perception_enabled, value_type=bool)}
    if scene in ("usb_cable", "trunking_cable"):
        options["load_cable"] = ParameterValue(load_cable, value_type=bool)
        options["cable_solver"] = cable_solver
        options["cable_config"] = resolve_cable_config(cable_config, scene=scene)
        options["cable_trace_dir"] = ParameterValue(cable_trace_dir, value_type=str)
    if scene == "trunking_cable":
        options["insertion_enabled"] = ParameterValue(insertion_enabled, value_type=bool)
        options["leader_orientation_direction"] = ParameterValue(
            leader_orientation_direction, value_type=str)
    return Node(
        package="dual_fr3_maniskill", executable=spec.bridge_executable,
        prefix=[python], output="screen",
        parameters=[simulation_config or spec.simulation_config,
                    *([camera_config] if camera_config else []),
                    robot_description, robot_description_semantic, options],
    )


def perception_arguments():
    import os
    from pathlib import Path
    from launch.actions import DeclareLaunchArgument
    from ament_index_python.packages import get_package_share_directory
    return [DeclareLaunchArgument(k, default_value=v) for k, v in dict(
        perception_enabled="false", perception_python=os.environ.get("PERCEPTION_PYTHON", str(Path.cwd()/".venv-perception/bin/python")),
        perception_config="", perception_checkpoint=os.environ.get("SAM2_CHECKPOINT", ""),
        perception_camera_config="", perception_preview="", perception_device="",
        perception_save_dir="").items()]


def validate_perception(context, backend, scene=None):
    from launch.substitutions import LaunchConfiguration
    enabled = LaunchConfiguration("perception_enabled", default="false").perform(context).lower() == "true"
    if enabled and backend != "maniskill":
        raise ValueError("perception_enabled currently supports simulation_backend:=maniskill only; D455 requires an external aligned RGB-D/TF source and standalone perception.launch.py")
    if enabled and scene is not None and scene not in ("usb_cable", "trunking_cable"):
        raise ValueError("perception_enabled requires maniskill_scene:=usb_cable or trunking_cable")
    if enabled and LaunchConfiguration("load_cable", default="true").perform(context).lower() != "true":
        raise ValueError("perception_enabled requires load_cable:=true (deferred MTC spawn is supported)")
    return enabled


def perception_nodes(context):
    from launch.substitutions import LaunchConfiguration as LC
    from ament_index_python.packages import get_package_share_directory
    if not validate_perception(context, "maniskill"):
        return []
    share = get_package_share_directory("dual_fr3_cable_perception")
    options = {"use_sim_time": True}
    for argument, parameter, kind in (("perception_checkpoint", "checkpoint", str),
            ("perception_preview", "preview", bool), ("perception_device", "device", str),
            ("perception_save_dir", "save_dir", str)):
        if LC(argument).perform(context):
            options[parameter] = ParameterValue(LC(argument), value_type=kind)
    return [Node(package="dual_fr3_cable_perception", executable="cable_perception_node.py",
        prefix=[LC("perception_python")], output="screen",
        parameters=[LC("perception_config").perform(context) or share+"/config/perception.yaml", options])]



def perception_camera_config(context, scene):
    from launch.substitutions import LaunchConfiguration
    from ament_index_python.packages import get_package_share_directory
    override = LaunchConfiguration("perception_camera_config").perform(context)
    filename = "perception_camera_trunking.yaml" if scene == "trunking_cable" else "perception_camera.yaml"
    return override or get_package_share_directory("dual_fr3_maniskill") + "/config/" + filename
