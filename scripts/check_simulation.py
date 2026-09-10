#!/usr/bin/env python3
"""Run physical arm/gripper motion and compare FK against the MoveIt URDF."""
import argparse
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import xacro
from ament_index_python.packages import get_package_share_directory

from dual_fr3_maniskill.assets import origin_matrix, prepare_assets
from dual_fr3_maniskill.simulation import Simulation


def forward_kinematics(description, positions):
    root = ET.fromstring(description)
    transforms = {"world": np.eye(4)}
    pending = list(root.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in transforms:
                continue
            motion = np.eye(4)
            if joint.get("type") != "fixed":
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                value = positions[joint.get("name")]
                mimic = joint.find("mimic")
                if mimic is not None:
                    value = positions[mimic.get("joint")] * float(mimic.get("multiplier", 1)) + float(mimic.get("offset", 0))
                if joint.get("type") == "prismatic":
                    motion[:3, 3] = axis * value
                else:
                    motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
            transforms[child] = transforms[parent] @ origin_matrix(joint.find("origin")) @ motion
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError("Disconnected URDF")
    return transforms


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--render-output", help="Save one rendered scene image (requires Vulkan)")
    parser.add_argument("--fixture-closeup", action="store_true",
                        help="Render a close view of the plate and trunking for mesh inspection")
    args = parser.parse_args()
    share = Path(get_package_share_directory("dual_fr3_moveit_config"))
    description = xacro.process_file(str(share / "config/dual_fr3.urdf.xacro"), mappings={
        "load_left_ros2_control": "false", "load_right_ros2_control": "false",
    }).toxml()
    semantic = xacro.process_file(str(share / "config/dual_fr3.srdf.xacro")).toxml()
    with tempfile.TemporaryDirectory(prefix="dual_fr3_check_") as temp:
        sim = Simulation(prepare_assets(description, semantic, Path(temp)), viewer=args.viewer)
        try:
            initial = sim.positions.copy()
            for _ in range(100):
                sim.step()
            hold_error = float(max(abs(sim.positions - initial)))
            for side, delta in (("left", 0.1), ("right", -0.1)):
                sim.target[sim.indices[f"{side}_fr3_joint1"]] += delta
                sim.target[sim.indices[f"{side}_fr3_finger_joint1"]] = 0.015
            for _ in range(200):
                sim.step()
            tracking_error = float(max(abs(sim.positions - sim.target)))
            fk = forward_kinematics(description, dict(zip(sim.names, sim.positions)))
            position_errors, rotation_errors = {}, {}
            for side in ("left", "right"):
                name = f"{side}_fr3_hand_tcp"
                pose = sim.link_pose(name)
                position_errors[side] = float(np.linalg.norm(fk[name][:3, 3] - pose[:3]))
                rotation = Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix()
                rotation_errors[side] = float(Rotation.from_matrix(fk[name][:3, :3].T @ rotation).magnitude())
            report = {"joints": len(sim.names), "hold_error": hold_error,
                      "tracking_error": tracking_error, "tcp_position_errors_m": position_errors,
                      "tcp_rotation_errors_rad": rotation_errors,
                      "collision_groups": len(sim.env.agent.collision_exclusion_groups)}
            print(json.dumps(report, indent=2), flush=True)
            assert hold_error < 0.005 and tracking_error < 0.005, report
            assert max(position_errors.values()) < 1e-4 and max(rotation_errors.values()) < 1e-4, report
            if args.render_output:
                from PIL import Image
                image = sim.render_image(fixture_closeup=args.fixture_closeup)
                Image.fromarray(image).save(args.render_output)
        finally:
            sim.close()


if __name__ == "__main__":
    main()
