#!/usr/bin/env python3
"""Profile unchanged MTC cable physics at a repeatable preparation pose.

No ROS node, action client or hardware connection is created. Run after sourcing
ROS and the workspace, with the same virtualenv as the ManiSkill bridge.
"""
import argparse
from collections import defaultdict
from contextlib import contextmanager, ExitStack
from functools import wraps
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import tempfile
import time
from unittest.mock import patch

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

# Import the project's SAPIEN compatibility layer before ManiSkill/Warp.
from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableSimulation
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
import warp as wp


class Timings:
    """Nested wall timing with exclusive component totals (no double counting).

    Kernel/copy timings include Python/driver submission and completion waits;
    they are not CUDA-event-only device times. Explicit barriers prevent prior
    kernels from being charged to the following device-to-host copy.
    """
    def __init__(self):
        self.frames = []
        self.components = defaultdict(float)
        self.operations = defaultdict(lambda: dict(calls=0, inclusive_seconds=0., exclusive_seconds=0.))
        self.transfers = defaultdict(lambda: dict(calls=0, bytes=0))
        self.sync = wp.synchronize
        self.capture_active = lambda: False

    @contextmanager
    def measure(self, name, *, operation=False, barrier=False):
        component = self.frames[-1]['component'] if operation and self.frames else name
        frame = dict(component=component, child=0.)
        self.frames.append(frame)
        start = time.perf_counter()
        try:
            if barrier and not self.capture_active():
                self.sync()
            yield
        finally:
            try:
                if barrier and not self.capture_active():
                    self.sync()
            finally:
                elapsed = time.perf_counter() - start
                self.frames.pop()
                own = elapsed - frame['child']
                self.components[component] += own
                if self.frames:
                    self.frames[-1]['child'] += elapsed
                row = self.operations[name]
                row['calls'] += 1
                row['inclusive_seconds'] += elapsed
                row['exclusive_seconds'] += own

    def wrapper(self, original, label, *, operation=False, barrier=False):
        @wraps(original)
        def wrapped(*args, **kwargs):
            if self.capture_active():
                return original(*args, **kwargs)
            with self.measure(label, operation=operation, barrier=barrier):
                return original(*args, **kwargs)
        return wrapped

    @contextmanager
    def instrument(self, sim):
        cable = sim.cable
        self.capture_active = lambda: getattr(getattr(cable, 'constraint_graphs', None), 'capturing', False)
        bindings = [
            (cable, 'step', 'cable orchestration'),
            (cable, 'follow_plug', 'rigid/cable coupling'),
            (cable.integrator, 'simulate', 'MPM'),
            (cable.fibers, 'solve', 'axial fibers'),
            (cable.guide, 'solve', 'right guide'),
            (cable.guide, 'update_pose', 'right guide pose upload'),
            (cable.guide, 'apply_reaction', 'right guide reaction'),
            (cable.guide, 'begin_step', 'right guide accumulation'),
            (cable, '_ensure_grid', 'grid size checks'),
            (cable, '_prepare_anchor', 'anchor and rigid body state'),
            (cable, '_update_bodies', 'anchor and rigid body state'),
            (cable, '_pin', 'particle pinning'),
            (cable.contacts, 'capture', 'contact history'),
            (cable.contacts, 'solve', 'contact projection'),
            (cable.contacts, 'audit', 'contact audit'),
            (type(sim.env._scene), 'step', 'PhysX rigid step'),
        ]
        if hasattr(cable, '_solve_fibers'):
            bindings.append((cable, '_solve_fibers', 'fiber/guide/contact sequence'))
        original_launch, original_copy = wp.launch, wp.context.copy

        def launch(kernel, *args, **kwargs):
            if self.capture_active():
                return original_launch(kernel, *args, **kwargs)
            with self.measure('kernel/' + kernel.key, operation=True, barrier=True):
                return original_launch(kernel, *args, **kwargs)

        def copy(dest, src, dest_offset=0, src_offset=0, count=0):
            if self.capture_active():
                return original_copy(dest, src, dest_offset, src_offset, count)
            direction = str(src.device) + '->' + str(dest.device)
            key = self.frames[-1]['component'] + '/' + direction if self.frames else direction
            size = count if count > 0 else src.size
            self.transfers[key]['calls'] += 1
            self.transfers[key]['bytes'] += size * wp.types.type_size_in_bytes(src.dtype)
            with self.measure('copy/' + direction, operation=True, barrier=True):
                return original_copy(dest, src, dest_offset, src_offset, count)

        with ExitStack() as stack:
            for owner, attribute, label in bindings:
                stack.enter_context(patch.object(owner, attribute,
                    self.wrapper(getattr(owner, attribute), label)))
            stack.enter_context(patch.object(wp, 'launch', launch))
            stack.enter_context(patch.object(wp, 'capture_launch', self.wrapper(
                wp.capture_launch, 'graph/constraints', operation=True, barrier=True)))
            stack.enter_context(patch.object(wp, 'copy', copy))
            stack.enter_context(patch.object(wp.context, 'copy', copy))
            for attribute in ('numpy', 'assign', 'zero_'):
                stack.enter_context(patch.object(wp.array, attribute, self.wrapper(
                    getattr(wp.array, attribute), 'array/' + attribute, operation=True,
                    barrier=attribute == 'zero_')))
            yield

    def report(self):
        return dict(component_seconds=dict(self.components), operations=dict(self.operations),
                    transfers=dict(self.transfers))


def snapshot(sim):
    state = sim.cable.states[0].struct
    return dict(particles=state.particle_q.numpy(), velocities=state.particle_qd.numpy(),
                deformation=state.particle_F.numpy(), joints=sim.positions.copy())


def one_run(args, config, simulation_config, joints, description, semantic, profiled, contact_details=False):
    name = 'contact_details' if contact_details else ('instrumented' if profiled else 'baseline')
    with tempfile.TemporaryDirectory(prefix='trunking_profile_') as directory:
        assets = prepare_assets(description, semantic, Path(directory))
        assets.initial_positions.update(joints)
        sim = TrunkingCableSimulation(assets, cable_config=config,
            control_freq=simulation_config['control_freq'], sim_freq=simulation_config['sim_freq'])
        try:
            sim.env.spawn_cable()
            for _ in range(args.warmup_steps):
                sim.step()
            wp.synchronize()
            before = sim.cable.diagnostics()
            if contact_details:
                sim.cable.contacts.enable_profiling()
            times = []
            profiler = Timings() if profiled else None
            with ExitStack() as stack:
                if profiler:
                    stack.enter_context(profiler.instrument(sim))
                for index in range(args.steps):
                    start = time.perf_counter()
                    if profiler:
                        with profiler.measure('other stepping and validation'):
                            sim.step()
                            wp.synchronize()
                    else:
                        sim.step()
                        wp.synchronize()
                    elapsed = time.perf_counter() - start
                    times.append(elapsed)
                    print(f'{name} step {index+1}/{args.steps}: {elapsed:.4f} s', flush=True)
            after = sim.cable.diagnostics()
            state = snapshot(sim)
            if not all(np.isfinite(value).all() for value in state.values()):
                raise RuntimeError('Non-finite benchmark state')
            if after['attachment_error_m'] >= 1e-5 or after['guide_radial_error_m'] >= 1e-4:
                raise RuntimeError(f'Attachment/guide check failed: {after}')
            sim_seconds = args.steps / sim.control_freq
            report = dict(step_wall_seconds=times, wall_seconds=sum(times),
                simulation_seconds=sim_seconds, realtime_factor=sim_seconds/sum(times),
                diagnostics_before=before, diagnostics_after=after)
            if contact_details:
                report['contact_details'] = sim.cable.contacts.profiling_report()
                for row in report['contact_details']['per_shape']:
                    row['actor'] = sim.cable.actors[row['body']].name
            if profiler:
                report['timings'] = profiler.report()
            elif not contact_details:
                # Offscreen image rendering includes pixel readback. This is not
                # the GUI viewer or RViz cost and does not overlap physics here.
                sim.render_image()
                wp.synchronize()
                render_times = []
                for _ in range(args.render_frames):
                    start = time.perf_counter()
                    sim.render_image()
                    wp.synchronize()
                    render_times.append(time.perf_counter() - start)
                report['offscreen_render_wall_seconds'] = render_times
            np.savez_compressed(args.output / (name + '_state.npz'), **state)
            return report, state
        finally:
            sim.close()


def write_summary(report, path):
    base, measured = report['baseline'], report['instrumented']
    components = measured['timings']['component_seconds']
    total = sum(components.values())
    lines = ['# MTC cable performance baseline', '',
        'Fixed preparation pose, stationary arm targets, unchanged cable parameters. '
        'Headless physics; no ROS publication, MTC planning, GUI viewer or RViz.', '',
        f"Baseline: {base['simulation_seconds']:.3f} simulated s / {base['wall_seconds']:.3f} wall s; "
        f"RTF {base['realtime_factor']:.6f}.",
        f"Instrumented/baseline wall ratio: {report['instrumentation_wall_ratio']:.3f}.", '',
        'Component times below are exclusive of nested components, but include their '
        'own CUDA work and transfers. Kernel/copy detail is already included, not additive.', '',
        '| Component | Seconds | Share |', '| --- | ---: | ---: |']
    for name, elapsed in sorted(components.items(), key=lambda item: -item[1]):
        lines.append(f'| {name} | {elapsed:.4f} | {elapsed/total:.1%} |')
    lines += ['', '| Operation (inclusive) | Calls | Seconds |', '| --- | ---: | ---: |']
    operations = measured['timings']['operations']
    for name, values in sorted(operations.items(), key=lambda item: -item[1]['inclusive_seconds']):
        if name.startswith(('kernel/', 'copy/', 'graph/')):
            lines.append(f"| {name} | {values['calls']} | {values['inclusive_seconds']:.4f} |")
    if 'contact_details' in report:
        details = report['contact_details']['contact_details']
        lines += ['', 'Per-shape counts from a separate run (its wall time is not the speed baseline).', '',
            '| Actor / shape | Projection pair tests | Skipped pass pairs | Candidate sweeps | Mesh point queries | Mesh rays | Exact cache hits | Clearance skips | Max iterations | Exhausted |',
            '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
        for row in sorted(details['per_shape'], key=lambda r: -r['surface_queries']):
            if row['candidate_sweeps'] or row.get('screen_clearance_skips', 0):
                lines.append(f"| {row['actor']} / {row['shape']} | {row['pair_tests']} | "
                    f"{row.get('skipped_pass_pair_tests', 0)} | {row['candidate_sweeps']} | {row['mesh_point_queries']} | {row['mesh_ray_queries']} | "
                    f"{row.get('exact_query_cache_hits', 0)} | {row.get('clearance_skips', 0) + row.get('screen_clearance_skips', 0)} | "
                    f"{row['max_sweep_iterations']} | {row['exhausted_sweeps']} |")
        lines.append(f"\nAll shapes: {sum(r['pair_tests'] for r in details['per_shape'])} projection pair tests; "
                     f"{sum(r.get('screen_pair_tests', 0) for r in details['per_shape'])} screening pair tests; "
                     f"{sum(r.get('skipped_pass_pair_tests', 0) for r in details['per_shape'])} skipped pass pair tests; "
                     f"{sum(r['candidate_sweeps'] for r in details['per_shape'])} candidate sweeps.")
    render = base['offscreen_render_wall_seconds']
    lines += ['', f'Offscreen RGB rendering including readback: mean {np.mean(render)*1000:.2f} ms/frame.', '',
        'Instrumentation synchronizes each Warp launch/copy; times include host submission and waits, '
        'not pure device kernel time. Per-frame baseline sync waits for work already required by physics checks. '
        'Warm-up and setup are excluded. See JSON for versions, input hashes, transfer counts and numerical differences.', '',
        'The benchmark starts from a recorded preparation IK pose (rounded to four decimals). '
        'It does not replay the stopped run or prove full-motion performance or stability.', '']
    path.write_text('\n'.join(lines))


def main():
    share = Path(get_package_share_directory('dual_fr3_maniskill'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cable-config', type=Path, default=share / 'config/trunking_cable.yaml')
    parser.add_argument('--simulation-config', type=Path, default=share / 'config/simulation_usb_cable.yaml')
    parser.add_argument('--initial-positions', type=Path, default=share / 'config/profiling_trunking_pose.json')
    parser.add_argument('--steps', type=int, default=3)
    parser.add_argument('--warmup-steps', type=int, default=1)
    parser.add_argument('--render-frames', type=int, default=3)
    parser.add_argument('--contact-details', action='store_true',
                        help='Add a separate run counting per-shape queries and sweep iterations')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if min(args.steps, args.warmup_steps, args.render_frames) < 1:
        parser.error('steps, warmup-steps and render-frames must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    config = load_config(args.cable_config)
    simulation_config = yaml.safe_load(args.simulation_config.read_text())['dual_fr3_maniskill']['ros__parameters']
    joints = json.loads(args.initial_positions.read_text())
    expected = {f'{side}_fr3_joint{i}' for side in ('left', 'right') for i in range(1, 8)}
    if set(joints) != expected or not all(np.isfinite(float(v)) for v in joints.values()):
        raise ValueError('initial-positions must contain all 14 finite arm joint positions')
    for side, width in [('left', config['usb']['finger_position']), ('right', 0.)]:
        joints.update({f'{side}_fr3_finger_joint{i}': width for i in (1, 2)})
    description, semantic = build_maniskill_description(scene='trunking_cable', cable_config=args.cable_config)
    gpu = subprocess.run(['nvidia-smi', '--query-gpu=name,driver_version,utilization.gpu,memory.used',
                          '--format=csv'], capture_output=True, text=True)
    report = dict(versions={name: importlib.metadata.version(name) for name in
                           ('mani-skill2', 'sapien', 'numpy')}, python=platform.python_version(),
        gpu_before=gpu.stdout.strip(), cpu=platform.processor(), initial_positions=joints,
        cable_config=config, simulation_config=simulation_config,
        inputs={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                (args.cable_config, args.simulation_config, args.initial_positions)},
        robot_description_sha256=hashlib.sha256(description.encode()).hexdigest(),
        environment={name: os.environ.get(name) for name in ('CUDA_VISIBLE_DEVICES',
            'CUDA_LAUNCH_BLOCKING', 'OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')},
        steps=args.steps, warmup_steps=args.warmup_steps)
    report['physics_source_sha256'] = {
        name: hashlib.sha256(Path(importlib.import_module(
            'dual_fr3_maniskill.cable.' + name).__file__).read_bytes()).hexdigest()
        for name in ('contacts', 'mpm_cable', 'guide', 'guide_cuda', 'fibers', 'solver', 'execution')}
    report['baseline'], baseline_state = one_run(args, config, simulation_config, joints, description, semantic, False)
    (args.output / 'baseline.json').write_text(json.dumps(report, indent=2))
    report['instrumented'], measured_state = one_run(args, config, simulation_config, joints, description, semantic, True)
    report['instrumentation_wall_ratio'] = report['instrumented']['wall_seconds'] / report['baseline']['wall_seconds']
    report['state_max_absolute_difference'] = {
        name: float(np.max(np.abs(baseline_state[name] - measured_state[name]))) for name in baseline_state}
    if args.contact_details:
        report['contact_details'], detail_state = one_run(
            args, config, simulation_config, joints, description, semantic, False, contact_details=True)
        report['contact_details_state_max_absolute_difference'] = {
            name: float(np.max(np.abs(baseline_state[name] - detail_state[name]))) for name in baseline_state}
    (args.output / 'report.json').write_text(json.dumps(report, indent=2))
    write_summary(report, args.output / 'report.md')
    print(f"Report: {args.output / 'report.md'}", flush=True)


if __name__ == '__main__':
    main()
