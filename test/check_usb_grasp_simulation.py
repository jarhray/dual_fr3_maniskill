#!/usr/bin/env python3
"""Real SAPIEN/ManiSkill USB contact grasp, world release and drop validation.

Runs without ROS motion services; uses the real robot CAD, joint drives and
native contacts. Writes every force/grasp snapshot as JSONL for review.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile

import numpy as np
from ament_index_python.packages import get_package_share_directory

from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_maniskill.forces import ForceCollector
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--solver", choices=("mpm", "rope_actor"), default="mpm")
    parser.add_argument("--load-cable", action="store_true")
    parser.add_argument("--cable-config", type=Path, help="Cable YAML override for accuracy/performance comparisons")
    parser.add_argument("--drop-regression-only", action="store_true", help="Short stationary grasp/release/drop check; omits arm-motion checks")
    parser.add_argument("--preposition-before-approach", action="store_true", help="Create at future preparation TCPs while arms are still away, then approach")
    parser.add_argument("--descend-after-grasp", type=float, default=0., help="Both TCPs descend this distance [m] after verification")
    parser.add_argument("--grip-force", type=float, default=10., help="Fixed per-finger drive force limit [N], not measured contact force")
    parser.add_argument("--grip-stiffness", type=float, default=1000., help="Fixed finger PD stiffness [N/m] for this experiment only")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--spawn-height-offset", type=float, default=0., help="Raise both preparation TCPs before creation [m]")
    args = parser.parse_args()
    if not np.isfinite(args.descend_after_grasp) or args.descend_after_grasp < 0:
        parser.error("descend-after-grasp must be a finite nonnegative distance")
    if (not np.isfinite(args.grip_force) or not 0 < args.grip_force <= 40. or
            not np.isfinite(args.grip_stiffness) or args.grip_stiffness <= 0):
        parser.error("grip-force must be in (0, 40] N and grip-stiffness positive and finite")
    path = resolve_cable_config(args.cable_config, scene="trunking_cable")
    config = load_config(path, solver=args.solver if args.load_cable else None)
    description, semantic = build_maniskill_description(scene="trunking_cable", cable_config=path)
    from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableSimulation
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="usb_grasp_validation_") as directory:
        assets = prepare_assets(description, semantic, Path(directory))
        share = Path(get_package_share_directory("dual_fr3_maniskill"))
        assets.initial_positions.update(json.loads((share/"config/profiling_trunking_pose.json").read_text()))
        for side, width in (("left", .02), ("right", .02)):
            assets.initial_positions.update({f"{side}_fr3_finger_joint{i}": width for i in (1, 2)})
        sim = TrunkingCableSimulation(assets, cable_config=config, cable_solver=args.solver,
            load_cable=args.load_cable, control_freq=50, sim_freq=500)
        if args.spawn_height_offset:
            from dual_fr3_maniskill.sapien_compat import sapien
            robot = sim.env.agent.robot
            model = robot.create_pinocchio_model()
            q = sim.target.copy()
            for side in ("left", "right"):
                link = sim.env.agent.links[f"{side}_fr3_hand_tcp"]
                target = sapien.Pose(link.pose.p+[0., 0., args.spawn_height_offset], link.pose.q)
                mask = np.zeros(len(q), dtype=int)
                mask[[sim.indices[f"{side}_fr3_joint{i}"] for i in range(1, 8)]] = 1
                q, success, error = model.compute_inverse_kinematics(robot.get_links().index(link),
                    target, initial_qpos=q, active_qmask=mask, eps=1.e-6, max_iterations=200)
                if not success:
                    raise RuntimeError(f"Preparation height IK failed: {error}")
            robot.set_qpos(q)
            sim.target[:] = q
        preparation_poses = None
        if args.preposition_before_approach:
            preparation_poses = {side: sim.env.agent.links[f"{side}_fr3_hand_tcp"].pose
                                 for side in (("left", "right") if args.load_cable else ("left",))}
            prepared_q = sim.target.copy()
            approach_q = prepared_q.copy()
            for side in preparation_poses:
                approach_q[sim.indices[f"{side}_fr3_joint1"]] += .03
            sim.env.agent.robot.set_qpos(approach_q)
            sim.target[:] = approach_q
        collector = ForceCollector(sim.env)
        sim.env.force_collector = collector
        summary = dict(preposition_before_approach=args.preposition_before_approach,
                       drop_regression_only=args.drop_regression_only, load_cable=args.load_cable, solver=args.solver, config=config,
                       gripper_target_speed_m_s=.04, spawn_height_offset_m=args.spawn_height_offset,
                       grip_force_limit_N=args.grip_force, grip_stiffness_N_m=args.grip_stiffness,
                       descend_after_grasp_m=args.descend_after_grasp, checks={})
        with args.output.open("x", encoding="utf-8") as output:
            def advance(count, phase):
                for _ in range(count):
                    sim.step()
                    row = dict(collector.snapshot, validation_phase=phase)
                    output.write(json.dumps(row, allow_nan=False)+"\n")
                    output.flush()
            try:
                advance(5, "before_spawn")
                sim.env.spawn_cable(preparation_poses)
                plug_id = sim.env.plug.id
                sim.env.spawn_cable()
                assert sim.env.plug.id == plug_id
                summary["checks"]["idempotent_creation"] = True
                if args.preposition_before_approach:
                    initial_pose = sim.env.plug.pose.p.copy()
                    advance(10, "positioned_before_approach")
                    for step in range(30):
                        sim.target[:] = approach_q+(prepared_q-approach_q)*(step+1)/30
                        advance(1, "approach_prepositioned_usb")
                    advance(15, "approach_settle")
                    assert np.linalg.norm(sim.env.plug.pose.p-initial_pose) < .0001
                    summary["checks"]["prepositioned_usb_stays_in_world_during_approach"] = True
                if not args.load_cable:
                    assert sim.cable is None and sim.env._cable_engine is None and sim.env.rigid_substeps == 1
                    assert not any(name in sys.modules for name in (
                        "dual_fr3_maniskill.cable.mpm_cable", "dual_fr3_maniskill.cable.rope_actor",
                        "dual_fr3_maniskill.cable.guide_cuda")), [name for name in sys.modules if name.startswith("dual_fr3_maniskill.cable.")]
                    summary["checks"]["no_cable_backend_initialization"] = True
                initial = sim.env.plug.pose.p.copy()
                joint = sim.indices["left_fr3_joint1"]
                original = sim.target[joint]
                if not args.drop_regression_only:
                    sim.target[joint] += .02
                    advance(25, "supported_arm_motion")
                drift = float(np.linalg.norm(sim.env.plug.pose.p-initial))
                summary["world_support_drift_m"] = drift
                assert drift < .0001, drift
                sim.target[joint] = original
                if not args.drop_regression_only:
                    advance(30, "return_to_grasp")
                sim.set_gripper_force("left", args.grip_force)
                for index in (1, 2):
                    sim.env.agent.joints[f"left_fr3_finger_joint{index}"].set_drive_property(
                        args.grip_stiffness, 50., force_limit=args.grip_force)
                sim.env.grasp_monitor.begin_closing(sim.env._grasp_time)
                # Match ROS gripper_speed=.04 m/s rather than teleporting the
                # drive target across its full travel in one control tick.
                for step in range(25):
                    target = max(0., .02-.04*(step+1)/sim.control_freq)
                    sim.target[sim.indices["left_fr3_finger_joint1"]] = target
                    if args.load_cable:
                        sim.target[sim.indices["right_fr3_finger_joint1"]] = target
                    advance(1, "contact_closing")
                advance(10 if args.drop_regression_only else 50, "contact_closing")
                snapshot = sim.env.grasp_monitor.snapshot()
                summary["initial_layout"] = sim.env.initial_layout_diagnostics
                summary["contact_ready"] = snapshot
                assert sim.env.grasp_monitor.ready_to_release, snapshot
                p, v, w = sim.env.plug.pose.p.copy(), sim.env.plug.velocity.copy(), sim.env.plug.angular_velocity.copy()
                sim.env.release_support()
                np.testing.assert_array_equal(sim.env.plug.pose.p, p)
                np.testing.assert_array_equal(sim.env.plug.velocity, v)
                np.testing.assert_array_equal(sim.env.plug.angular_velocity, w)
                assert sim.env.support_drive is None and sim.env.mount_drive is None and not sim.env.temporary_supports
                assert getattr(sim.cable, "support_count", 0) == 0
                summary["checks"]["release_preserves_state_no_tcp_mount"] = True
                advance(30, "released_verification")
                snapshot = sim.env.grasp_monitor.snapshot()
                summary["released"] = snapshot
                assert snapshot["stable"], snapshot
                summary["checks"]["real_contact_stable_grasp"] = True
                if args.descend_after_grasp:
                    from dual_fr3_maniskill.sapien_compat import sapien
                    robot = sim.env.agent.robot
                    model = robot.create_pinocchio_model()
                    starts = {side: sim.env.agent.links[f"{side}_fr3_hand_tcp"].pose for side in ("left", "right")}
                    q = sim.target.copy()
                    for step in range(50):
                        for side, start in starts.items():
                            link = sim.env.agent.links[f"{side}_fr3_hand_tcp"]
                            goal = sapien.Pose(start.p-[0., 0., args.descend_after_grasp*(step+1)/50], start.q)
                            mask = np.zeros(len(q), dtype=int)
                            mask[[sim.indices[f"{side}_fr3_joint{i}"] for i in range(1, 8)]] = 1
                            q, success, error = model.compute_inverse_kinematics(robot.get_links().index(link),
                                goal, initial_qpos=q, active_qmask=mask, eps=1.e-6, max_iterations=200)
                            if not success:
                                raise RuntimeError(f"Descent IK failed: {error}")
                        sim.target[:] = q
                        advance(1, "dual_descent")
                    advance(10, "descent_settle")
                    assert sim.env.grasp_monitor.snapshot()["stable"], sim.env.grasp_monitor.snapshot()
                    summary["checks"]["dual_descent_stable"] = True
                    original = sim.target[joint]
                if not args.drop_regression_only:
                    for i in range(25):
                        sim.target[joint] = original+.01*(i+1)/25
                        advance(1, "common_arm_motion")
                    advance(10, "common_motion_settle")
                    assert sim.env.grasp_monitor.snapshot()["stable"], sim.env.grasp_monitor.snapshot()
                    summary["checks"]["common_motion_not_slip"] = True
                for step in range(25):
                    sim.target[sim.indices["left_fr3_finger_joint1"]] = min(.02, .04*(step+1)/sim.control_freq)
                    advance(1, "open_and_drop")
                advance(10, "open_and_drop")
                snapshot = sim.env.grasp_monitor.snapshot()
                summary["opened"] = snapshot
                assert snapshot["state"] == "dropped", snapshot
                assert any(row["state"] == "slipping" for row in snapshot["transitions"]), snapshot
                summary["checks"]["opening_slip_and_drop"] = True
                summary["initial_layout"] = sim.env.initial_layout_diagnostics
                sim.env.clear_objects()
                sim.env.clear_objects()
                assert sim.env.plug is None and sim.cable is None
                summary["checks"]["reset_cleanup"] = True
                summary["passed"] = True
            except Exception as exc:
                summary["passed"], summary["error"] = False, str(exc)
                raise
            finally:
                args.output.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
                print(json.dumps(summary, indent=2, allow_nan=False))
                sim.close()


if __name__ == "__main__":
    main()
