#!/usr/bin/env python3
"""Isolate a short rope and moving research-finger proxies in CPU PhysX.

No ROS node, robot controller, CUDA or Vulkan renderer is started. This is a
reduced experiment, not an exact replay of the complete MTC failure state.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time

import numpy as np
from transforms3d.quaternions import quat2mat

from dual_fr3_maniskill.engine.sapien_compat import sapien
from dual_fr3_maniskill.robot.assets import prepare_assets
from dual_fr3_maniskill.cable.kinematic import load_kinematic_target
from dual_fr3_maniskill.cable.model import USB_LINK, load_config
from dual_fr3_maniskill.cable.rope_actor import RopeActorCable
from dual_fr3_maniskill.cable.rope_diagnostics import RopeSubstepTrace, json_safe


def create_scene(config, directory, *, coupling):
    """Copy the closed right fingers' frames/inertia; retain their original CAD.

    Each dynamic source finger is fixed to a prescribed TCP carrier. Cable
    impulses reach it through the unchanged production proxy code one step
    later. The carrier replaces the rest of the robot's dynamics. The
    prescribed control case makes the source fingers kinematic instead.
    """
    from ament_index_python.packages import get_package_share_directory
    from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description

    r = config["rope_actor"]
    engine = sapien.Engine(tolerance_length=r["engine_tolerance_length"],
                           tolerance_speed=r["engine_tolerance_speed"])
    settings = sapien.SceneConfig()
    settings.enable_tgs = config["rope_actor"]["solver_type"] == "tgs"
    settings.enable_pcm = True
    settings.enable_ccd = False
    settings.contact_offset = .001
    settings.solver_iterations = r["solver_iterations"]
    settings.solver_velocity_iterations = r["solver_velocity_iterations"]
    scene = engine.create_scene(settings)
    scene.set_timestep(1/r["frequency"])
    description, semantic = build_maniskill_description(scene="trunking_cable")
    assets = prepare_assets(description, semantic, directory)
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(str(assets.urdf_path))
    if robot is None:
        raise RuntimeError("Could not load reference robot for finger frames")
    pose_path = Path(get_package_share_directory("dual_fr3_maniskill"))/"config/profiling_trunking_pose.json"
    positions = json.loads(pose_path.read_text())
    robot.set_qpos([0. if "finger_joint" in joint.name else
                   positions.get(joint.name, assets.initial_positions[joint.name])
                   for joint in robot.get_active_joints()])
    original = {link.name: link for link in robot.get_links()}
    tcp_pose = original["right_fr3_hand_tcp"].pose
    specifications = []
    for name in ("right_fr3_leftfinger", "right_fr3_rightfinger"):
        link = original[name]
        specifications.append((name, tcp_pose.inv()*link.pose, link.mass,
                               link.cmass_local_pose, link.inertia))
    scene.remove_articulation(robot)
    scene.step()

    carrier = scene.create_actor_builder().build_kinematic("right_fr3_hand_tcp")
    # Keep the recorded orientation and gravity direction, but move the
    # isolated experiment near the origin to avoid unrelated world geometry.
    carrier.set_pose(sapien.Pose([0., 0., .2], tcp_pose.q))
    origin = carrier.pose
    actors = {carrier.name: carrier}
    source_offsets, drives = [], []
    for name, local, mass, com, inertia in specifications:
        builder = scene.create_actor_builder()
        builder.set_mass_and_inertia(mass, com, inertia)
        source = (builder.build(name) if coupling == "delayed" else builder.build_kinematic(name))
        source.set_pose(origin*local)
        source.set_solver_iterations(r["solver_iterations"], r["solver_velocity_iterations"])
        if coupling == "delayed":
            drive = scene.create_drive(carrier, local, source, sapien.Pose())
            drive.lock_motion(True, True, True, True, True, True)
            drives.append(drive)
        source_offsets.append((source, local))
        actors[name] = source
    from dual_fr3_maniskill.cable.guide import SlidingGuide
    guide = SlidingGuide(carrier, config["guide"], 0.)
    axis = quat2mat(guide.pose.q)[:, 0]
    start = guide.pose.p-axis*config["cable"]["length"]/2
    plug = scene.create_actor_builder().build_kinematic(USB_LINK)
    plug.set_pose(sapien.Pose(start, origin.q))
    actors[plug.name] = plug
    env = SimpleNamespace(_scene=scene, sim_timestep=1/r["frequency"],
                          agent=SimpleNamespace(links=actors), fixtures={}, assets=assets)

    def layout(cfg, s, rotation, translation):
        return start+np.asarray(s)[:, None]*axis

    cable = RopeActorCable(env, config, plug=plug, guide=guide, layout=layout)
    # Identify the actual cooked collision triangles, not a temporary pathname.
    meshes = []
    for proxy, source in cable.proxies:
        for shape in proxy.get_collision_shapes():
            g = shape.geometry
            payload = np.asarray(g.vertices).tobytes()+np.asarray(g.indices).tobytes()
            meshes.append(dict(finger=source.name, triangles=len(g.indices)//3,
                               sha256=hashlib.sha256(payload).hexdigest()))
    metadata = dict(collision_meshes=meshes, recorded_tcp_quaternion_wxyz=tcp_pose.q.tolist(),
                    finger_masses_kg={s.name: float(s.mass) for s, _ in source_offsets},
                    initial_cable=cable.diagnostics())
    return engine, scene, cable, carrier, source_offsets, drives, metadata


def displacement(t, settle, duration, distance):
    u = np.clip((t-settle)/duration, 0., 1.)
    return distance*(3*u*u-2*u*u*u)


def run(args, config, report, stream):
    with tempfile.TemporaryDirectory(prefix="rope_finger_") as directory:
        engine, scene, cable, carrier, sources, drives, metadata = create_scene(
            config, Path(directory), coupling=args.coupling)
        report.update(metadata)
        target = load_kinematic_target()
        trace = RopeSubstepTrace(args.tail_steps, stream)
        origin = carrier.pose
        direction = quat2mat(origin.q) @ np.asarray(args.motion_axis, dtype=float)
        direction /= np.linalg.norm(direction)
        elapsed, count = 0., 0
        schedule = None
        total = args.settle+args.duration+args.hold
        try:
            cable.check_contacts()
            while elapsed < total-1.e-12:
                maximum = args.dt
                if args.timestep == "adaptive":
                    from dual_fr3_maniskill.cable.timestep import RopeStepSchedule

                    limit = cable.suggested_timestep(maximum)
                    if schedule is None or not schedule.remaining_steps:
                        schedule = RopeStepSchedule(min(.02, total-elapsed), maximum,
                            cable._dt if count else None)
                    dt = schedule.next_step(limit)
                else:
                    dt = maximum if args.timestep == "fixed" or count % 2 == 0 else args.alternate_dt
                    dt = min(dt, total-elapsed)
                next_pose = sapien.Pose(origin.p+direction*displacement(
                    elapsed+dt, args.settle, args.duration, args.distance), origin.q)
                target(carrier._ptr, next_pose.p.tolist(), next_pose.q.tolist())
                if args.coupling == "prescribed":
                    # Explicit one-way control case; forces do not act on a
                    # kinematic source. Never alter this in the delayed case.
                    cable._pending_reactions.clear()
                    for source, local in sources:
                        pose = next_pose*local
                        target(source._ptr, pose.p.tolist(), pose.q.tolist())
                scene.set_timestep(dt)
                cable.step(dt)
                # Optional boundary traction represents a removed cable tail;
                # this prescribed force is an experiment input, not a fitted
                # reconstruction of the full MTC tension.
                if any(args.tail_force):
                    _, ends = cable._endpoints()
                    cable.links[-1].add_force_at_point(args.tail_force, ends[-1])
                scene.step()
                elapsed += dt
                count += 1
                follow_failed = True
                try:
                    cable.follow_plug()
                    follow_failed = False
                finally:
                    # Sample geometry periodically and always on a speed or
                    # threading failure. Every step still records native
                    # contacts, speed, energy, joint errors and proxy errors.
                    row = trace.record(cable, elapsed, audit=(follow_failed or count == 1 or
                        count % args.audit_every == 0 or elapsed >= total-1.e-12))
                if row["geometry_audit_error"]:
                    raise RuntimeError("Post-step geometry audit failed: "+row["geometry_audit_error"])
                if (not np.isfinite(row["max_endpoint_speed_m_s"]) or
                        row["max_endpoint_speed_m_s"] > config["rope_actor"]["max_speed"]):
                    raise cable._speed_failure(row["max_speed_segment_index"], row["max_endpoint_speed_m_s"])
                if row["geometry_audited"] and row["max_penetration_m"] > config["cable"]["penetration_tolerance"]:
                    raise RuntimeError("Rope-Actor rigid contact penetration exceeds tolerance; "+
                                       cable._penetration_context(cable.max_penetration_contact))
                if (row["geometry_audited"] or row["max_joint_gap_m"] > config["rope_actor"]["constraint_tolerance"]
                        or row["attachment_error_m"] > config["rope_actor"]["constraint_tolerance"]):
                    cable._check_constraints()
            if trace.contact_steps == 0:
                raise RuntimeError("Experiment never contacted the fingers; this is not a contact validation")
            report["passed"] = True
        except Exception as exc:
            report["failure"] = str(exc)
        finally:
            report["simulated_time_s"] = elapsed
            report["summary"] = trace.summary()
            report["tail"] = list(trace.tail)
            report["final_cable"] = cable.diagnostics()
            cable.close()
            for drive in drives:
                scene.remove_drive(drive)
        # Keep the engine alive throughout cleanup (SAPIEN shares internals).
        del engine


def main():
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cable-config", type=Path, default=
                        Path(get_package_share_directory("dual_fr3_maniskill"))/"config/trunking_cable_simplified_2mm.yaml")
    parser.add_argument("--links", type=int, default=13, help="Short chain size; ordinary segment length is retained")
    parser.add_argument("--reference-links", type=int, default=256, help="Full cable segmentation defining segment length")
    parser.add_argument("--coupling", choices=("delayed", "prescribed"), default="delayed")
    parser.add_argument("--timestep", choices=("fixed", "alternating", "adaptive"), default="fixed")
    parser.add_argument("--dt", type=float, default=.0001)
    parser.add_argument("--alternate-dt", type=float, default=.000357143)
    parser.add_argument("--settle", type=float, default=.02)
    parser.add_argument("--duration", type=float, default=.1)
    parser.add_argument("--hold", type=float, default=.02)
    parser.add_argument("--distance", type=float, default=.001, help="Finger translation [m]; zero gives static control")
    parser.add_argument("--motion-axis", type=float, nargs=3, default=[0., 0., 1.], help="Direction in initial TCP frame")
    parser.add_argument("--tail-force", type=float, nargs=3, default=[0., 0., 0.],
                        help="Prescribed force at the free cable endpoint in world coordinates [N]")
    parser.add_argument("--penetration-tolerance", type=float)
    parser.add_argument("--max-contact-travel", type=float)
    parser.add_argument("--tail-steps", type=int, default=64)
    parser.add_argument("--audit-every", type=int, default=20,
                        help="Post-solve geometry sampling interval; 1 audits every substep; speed failures always audited")
    parser.add_argument("--output", type=Path, required=True, help="New JSON report; sibling .jsonl stores every substep")
    args = parser.parse_args()
    if (not 6 <= args.links <= 256 or not 6 <= args.reference_links <= 256
            or min(args.tail_steps, args.audit_every) < 1):
        parser.error("links/reference-links must be 6..256 and tail-steps/audit-every positive")
    if not np.isfinite([args.dt, args.alternate_dt, args.settle, args.duration, args.hold,
                        args.distance, *args.motion_axis, *args.tail_force]).all():
        parser.error("Motion and timestep parameters must be finite")
    if min(args.dt, args.alternate_dt, args.duration) <= 0 or min(args.settle, args.hold) < 0:
        parser.error("Timesteps/duration must be positive; settle/hold must be nonnegative")
    if np.linalg.norm(args.motion_axis) == 0:
        parser.error("motion-axis must be nonzero")
    config = deepcopy(load_config(args.cable_config, solver="rope_actor"))
    c, r = config["cable"], config["rope_actor"]
    segment_length = (c["length"]-c["pin_length"])/(args.reference_links-1)
    r["links"] = args.links
    c["length"] = c["pin_length"]+(args.links-1)*segment_length
    c["initial_layout"] = "straight"
    for section, key, value in ((c, "penetration_tolerance", args.penetration_tolerance),
                                (r, "max_contact_travel", args.max_contact_travel)):
        if value is not None:
            section[key] = value
    # Use the production validator for overrides and the derived short length.
    import yaml
    with tempfile.TemporaryDirectory(prefix="rope_config_") as directory:
        path = Path(directory)/"config.yaml"
        path.write_text(yaml.safe_dump(config))
        config = load_config(path, solver="rope_actor")
    trace_path = args.output.with_suffix(".jsonl")
    if args.output == trace_path or args.output.exists() or trace_path.exists():
        parser.error("Choose a new .json output path; reports are never overwritten")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(passed=False, experiment="short_rope_moving_right_finger", frame_id="world",
                  scope="Reduced finger sources fixed to a prescribed carrier; optional prescribed tail traction; no full-arm inertia, trunking or MTC replay",
                  config=config, options={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                  trace_path=str(trace_path), source_hashes={})
    import inspect
    from dual_fr3_maniskill.cable import model, kinematic, rope_actor, rope_diagnostics
    for module in (model, kinematic, rope_actor, rope_diagnostics):
        path = Path(inspect.getfile(module)).resolve()
        report["source_hashes"][path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    report["source_hashes"][Path(__file__).name] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    started = time.perf_counter()
    try:
        with trace_path.open("x") as stream:
            run(args, config, report, stream)
    except Exception as exc:
        report["failure"] = str(exc)
    finally:
        report["wall_time_s"] = time.perf_counter()-started
        args.output.write_text(json.dumps(json_safe(report), indent=2, allow_nan=False)+"\n")
    print(json.dumps(json_safe({k: v for k, v in report.items() if k in
                     ("passed", "failure", "summary", "simulated_time_s", "wall_time_s")}), indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
