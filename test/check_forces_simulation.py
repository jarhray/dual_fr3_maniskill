#!/usr/bin/env python3
"""Short force-capture check in the real deferred-spawn scene (simulation only).

Optionally verify ROS serialization/subscription and JSONL recording as well.
Requires the same CUDA/Vulkan runtime as the normal cable scene.
"""
import argparse
import json
from pathlib import Path
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np

from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_maniskill.forces import ForceCollector
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--solver", choices=("rope_actor", "mpm"), default="rope_actor")
    parser.add_argument("--steps", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ros", action="store_true")
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("steps must be positive")
    path = resolve_cable_config(scene="trunking_cable")
    config = load_config(path, solver=args.solver)
    description, semantic = build_maniskill_description(scene="trunking_cable", cable_config=path)
    from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableSimulation
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="force_capture_check_") as directory:
        assets = prepare_assets(description, semantic, Path(directory))
        share = Path(get_package_share_directory("dual_fr3_maniskill"))
        assets.initial_positions.update(json.loads((share/"config/profiling_trunking_pose.json").read_text()))
        for side, width in (("left", config["usb"]["finger_position"]), ("right", 0.)):
            assets.initial_positions.update({f"{side}_fr3_finger_joint{i}": width for i in (1, 2)})
        sim = TrunkingCableSimulation(assets, cable_config=config, cable_solver=args.solver,
                                     control_freq=50, sim_freq=500)
        output = node = listener = None
        record = None
        received, wrenches = [], []
        try:
            if args.ros:
                import rclpy
                from geometry_msgs.msg import WrenchStamped
                from std_msgs.msg import String
                from dual_fr3_maniskill.force_output import ForceOutput
                from dual_fr3_maniskill.ros_bridge import stamp
                rclpy.init()
                node = rclpy.create_node("force_capture_validation")
                node.config = dict(force_record_path=str(args.output), force_usb_base_names="")
                node.sim, node.arms, node.grippers = sim, {}, {}
                output = ForceOutput(node, stamp)
                listener = rclpy.create_node("force_capture_validation_listener")
                subscriptions = [
                    listener.create_subscription(String, "/maniskill/forces",
                        lambda msg: received.append(json.loads(msg.data)), 100),
                    listener.create_subscription(WrenchStamped, "/maniskill/forces/left/cable/wrench",
                        wrenches.append, 100)]
                until = time.monotonic()+1.
                while time.monotonic() < until:
                    rclpy.spin_once(listener, timeout_sec=.02)
                collector = output.collector
            else:
                collector = ForceCollector(sim.env)
                sim.env.force_collector = collector
                record = args.output.open("x", encoding="utf-8")

            sim.step()
            assert not collector.snapshot["sensors"]["left/cable"]["available"]
            if output:
                output.publish()
            else:
                record.write(json.dumps(collector.snapshot, allow_nan=False)+"\n")
            sim.env.spawn_cable()
            force_norms = []
            for _ in range(args.steps):
                sim.step()
                snapshot = collector.snapshot
                value = snapshot["sensors"]["left/cable"]
                assert value["available"], value
                assert abs(snapshot["duration_s"]-.02) < 1.e-9
                assert not snapshot["sensors"]["left/usb_base"]["available"]
                assert not snapshot["sensors"]["fingers/left_fr3_leftfinger/usb"]["available"]
                force_norms.append(float(np.linalg.norm(value["force_N"])))
                if output:
                    output.publish()
                    rclpy.spin_once(listener, timeout_sec=.05)
                else:
                    record.write(json.dumps(snapshot, allow_nan=False)+"\n")
            if output:
                until = time.monotonic()+5.
                while (len(received) < args.steps+1 or len(wrenches) < args.steps) and time.monotonic() < until:
                    rclpy.spin_once(listener, timeout_sec=.05)
                assert len(received) == args.steps+1, len(received)
                assert len(wrenches) == args.steps, len(wrenches)
                msg, value = wrenches[-1], received[-1]["sensors"]["left/cable"]
                np.testing.assert_allclose([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], value["force_N"])
                assert msg.header.frame_id == "left_fr3_hand_tcp"
                output.close()
            else:
                record.close()
            rows = [json.loads(line) for line in args.output.read_text().splitlines()]
            assert len(rows) == args.steps+1
            print(json.dumps(dict(passed=True, solver=args.solver, ros=args.ros, records=len(rows),
                left_cable_force_norm_N=force_norms, last_physics_substeps=rows[-1]["physics_substeps"],
                output=str(args.output)), indent=2))
        finally:
            if record is not None:
                record.close()
            if output is not None:
                output.close()
            if listener is not None:
                listener.destroy_node()
            if node is not None:
                node.destroy_node()
                rclpy.shutdown()
            sim.close()


if __name__ == "__main__":
    main()
