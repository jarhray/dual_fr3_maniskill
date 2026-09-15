#!/usr/bin/env python3
"""Exercise either cable backend at the recorded MTC pose (simulation only)."""
import argparse
import json
from pathlib import Path
import tempfile
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory

from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.cable.backends import CABLE_SOLVERS
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cable-solver", choices=CABLE_SOLVERS, default="mpm")
    parser.add_argument("--scene", choices=("trunking_cable", "usb_cable"), default="trunking_cable")
    parser.add_argument("--cable-config", default="")
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render-output", type=Path)
    parser.add_argument("--spawn-height-offset", type=float, default=0., help="Offset both recorded TCP heights [m]")
    parser.add_argument("--descent", type=float, default=0., help="Exercise a synchronized TCP descent [m]")
    args = parser.parse_args()
    if args.steps < 10:
        parser.error("Use at least 10 motion steps")
    if not np.isfinite([args.spawn_height_offset, args.descent]).all() or args.descent < 0:
        parser.error("Heights must be finite and descent must be nonnegative")
    if args.scene != "trunking_cable" and (args.spawn_height_offset or args.descent):
        parser.error("Height overrides require the deferred-spawn trunking_cable scene")
    path = resolve_cable_config(args.cable_config, scene=args.scene)
    config = load_config(path, solver=args.cable_solver)
    description, semantic = build_maniskill_description(scene=args.scene, cable_config=path)
    from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableSimulation
    from dual_fr3_maniskill.scenes.usb_cable import UsbCableSimulation
    factory = TrunkingCableSimulation if args.scene == "trunking_cable" else UsbCableSimulation
    report = dict(solver=args.cable_solver, scene=args.scene, config=config,
                  control_frequency=50, rigid_frequency=500, diagnostics=[], passed=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="check_cable_") as directory:
        assets = prepare_assets(description, semantic, Path(directory))
        if args.scene == "trunking_cable":
            share = Path(get_package_share_directory("dual_fr3_maniskill"))
            assets.initial_positions.update(json.loads((share/"config/profiling_trunking_pose.json").read_text()))
            for side, width in (("left", config["usb"]["finger_position"]), ("right", 0.)):
                assets.initial_positions.update({f"{side}_fr3_finger_joint{i}": width for i in (1, 2)})
        sim = factory(assets, cable_config=config, cable_solver=args.cable_solver, control_freq=50, sim_freq=500)
        times = []
        try:
            from dual_fr3_maniskill.sapien_compat import sapien
            pinocchio = sim.env.agent.robot.create_pinocchio_model()
            robot_links = sim.env.agent.robot.get_links()

            def targets_at_height(poses, offset):
                q = sim.target.copy()
                for side in ("left", "right"):
                    mask = np.zeros(len(q), dtype=int)
                    mask[[sim.indices[f"{side}_fr3_joint{i}"] for i in range(1, 8)]] = 1
                    pose = poses[side]
                    target = sapien.Pose(pose.p+[0., 0., offset], pose.q)
                    link = sim.env.agent.links[f"{side}_fr3_hand_tcp"]
                    q, success, error = pinocchio.compute_inverse_kinematics(
                        robot_links.index(link), target, initial_qpos=q, active_qmask=mask,
                        eps=1.e-6, max_iterations=200)
                    if not success:
                        raise RuntimeError(f"{side} descent IK failed: {error}")
                return q

            if args.spawn_height_offset:
                poses = {s: sim.env.agent.links[f"{s}_fr3_hand_tcp"].pose for s in ("left", "right")}
                sim.target[:] = targets_at_height(poses, args.spawn_height_offset)
                sim.env.agent.robot.set_qpos(sim.target)
            if args.scene == "trunking_cable":
                assert sim.cable is None
                sim.env.spawn_cable()
                cable = sim.cable
                sim.env.spawn_cable()
                assert sim.cable is cable, "Repeated insertion must be idempotent"
            report["initial"] = sim.cable.diagnostics()
            initial = sim.target.copy()
            start_pose = sim.link_pose("right_fr3_hand_tcp")[:3]
            poses = {s: sim.env.agent.links[f"{s}_fr3_hand_tcp"].pose for s in ("left", "right")}
            times, motions = [], []
            for step in range(args.steps):
                phase = 2*np.pi*(step+1)/args.steps
                for name, amplitude in (("right_fr3_joint1", .002), ("right_fr3_joint7", .01), ("left_fr3_joint7", .003)):
                    index = sim.indices[name]
                    sim.target[index] = initial[index]+amplitude*np.sin(phase)
                if args.descent:
                    u = (step+1)/args.steps
                    sim.target[:] = targets_at_height(poses, -args.descent*(3*u*u-2*u*u*u))
                start = time.perf_counter()
                sim.step()
                times.append(time.perf_counter()-start)
                motions.append(float(np.linalg.norm(sim.link_pose("right_fr3_hand_tcp")[:3]-start_pose)))
                report["diagnostics"].append(sim.cable.diagnostics())
                if (step+1) % 20 == 0:
                    print(f"{args.cable_solver}: {step+1}/{args.steps} control steps", flush=True)
            report["mean_control_step_ms"] = float(np.mean(times)*1000)
            report["p95_control_step_ms"] = float(np.percentile(times, 95)*1000)
            report["realtime_factor"] = .02/np.mean(times)
            report["maximum_right_tcp_motion_m"] = max(motions)
            assert max(motions) > .0002, "Regression must move the right TCP"
            joints = sim.positions.copy()
            centerline = sim.cable.centerline.copy()
            try:
                sim.cable.reset()
            except ValueError as exc:
                if not args.descent or "Initial rope intersects" not in str(exc):
                    raise
                # A cable draped over a wall can be valid while a new straight
                # insertion at the lowered TCPs intersects that wall.
                np.testing.assert_array_equal(sim.cable.centerline, centerline)
                report["reset_rejected_without_mutation"] = str(exc)
            np.testing.assert_array_equal(sim.positions, joints)
            for _ in range(5):
                sim.step()
            report["after_reset"] = sim.cable.diagnostics()
            if args.render_output:
                from PIL import Image
                Image.fromarray(sim.render_image()).save(args.render_output)
            report["passed"] = True
        except Exception as exc:
            report["failure"] = str(exc)
            if sim.cable is not None:
                report["failure_diagnostics"] = sim.cable.diagnostics()
            raise
        finally:
            if times:
                report["mean_control_step_ms"] = float(np.mean(times)*1000)
                report["p95_control_step_ms"] = float(np.percentile(times, 95)*1000)
                report["realtime_factor"] = .02/np.mean(times)
            args.output.write_text(json.dumps(report, indent=2)+"\n")
            sim.close()
    print(json.dumps({k: v for k, v in report.items() if k in
                     ("solver", "passed", "mean_control_step_ms", "p95_control_step_ms", "realtime_factor")}, indent=2))


if __name__ == "__main__":
    main()
