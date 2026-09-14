#!/usr/bin/env python3
"""Bounded moving-arm/reset comparison of GPU and NumPy guides in the MTC scene.

Uses the recorded preparation pose and unchanged physics configuration. No ROS
node, viewer or robot connection. This is not a full MTC trajectory replay.
"""
import argparse
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_maniskill.cable.guide import SlidingGuide
from dual_fr3_maniskill.scenes import trunking_cable
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
from reference.guide_numpy import SlidingGuide as NumpyGuide


class ReferenceGuide(NumpyGuide):
    """Adapt the frozen solver to the explicit GPU lifecycle, without changing its math."""
    def bind(self, cable):
        pass

    def update_pose(self):
        pass

    def begin_step(self, *, reset=False):
        super().begin_step()


def run(guide_type, config, frequencies, joints, description, semantic, steps, output, label=None):
    label = label or ('numpy' if guide_type is ReferenceGuide else 'gpu')
    with tempfile.TemporaryDirectory(prefix='trunking_guide_regression_') as directory:
        assets = prepare_assets(description, semantic, Path(directory))
        assets.initial_positions.update(joints)
        with patch.object(trunking_cable, 'SlidingGuide', guide_type):
            sim = trunking_cable.TrunkingCableSimulation(assets, cable_config=config,
                control_freq=frequencies['control_freq'], sim_freq=frequencies['sim_freq'])
            try:
                sim.env.spawn_cable()
                initial = sim.target.copy()
                start_pose = sim.link_pose('right_fr3_hand_tcp')
                rows = []
                snapshot = {}
                maximum_motion = 0.
                for step in range(steps + 5):
                    if step < steps:
                        phase = 2*np.pi*(step+1)/steps
                        for name, amplitude in [('right_fr3_joint1', .002),
                                                 ('right_fr3_joint7', .01),
                                                 ('left_fr3_joint7', .003)]:
                            index = sim.indices[name]
                            sim.target[index] = initial[index] + amplitude*np.sin(phase)
                    elif step == steps:
                        sim.target[:] = initial
                        sim.cable.reset()
                    sim.step()
                    maximum_motion = max(maximum_motion, float(np.linalg.norm(
                        sim.link_pose('right_fr3_hand_tcp')[:3]-start_pose[:3])))
                    diagnostic = sim.cable.diagnostics()
                    assert diagnostic['attachment_error_m'] < 1.e-5, diagnostic
                    assert diagnostic['guide_radial_error_m'] < 1.e-4, diagnostic
                    assert diagnostic['max_rigid_penetration_m'] <= config['cable']['penetration_tolerance'], diagnostic
                    rows.append(diagnostic)
                    if step == steps - 1:
                        state = sim.cable.states[0].struct
                        snapshot.update(motion_particles=state.particle_q.numpy(),
                                        motion_velocities=state.particle_qd.numpy(),
                                        motion_joints=sim.positions.copy())
                    if (step+1) % 5 == 0:
                        print(f'{label}: {step+1}/{steps+5} steps, guide error '
                              f'{diagnostic["guide_radial_error_m"]:.3g} m', flush=True)
                assert maximum_motion > .0002, 'The regression must actually move the right TCP'
                state = sim.cable.states[0].struct
                snapshot.update(particles=state.particle_q.numpy(), velocities=state.particle_qd.numpy(),
                                joints=sim.positions.copy())
                assert all(np.isfinite(value).all() for value in snapshot.values())
                np.savez_compressed(output / (label + '_state.npz'), **snapshot)
                return dict(diagnostics=rows, maximum_tcp_motion_m=maximum_motion), snapshot
            finally:
                sim.close()


def validate_comparison(report, reference, repeated, actual):
    """Check both physical limits and the measured repeatability of unchanged MPM.

MPM's float32 grid atomics make even two NumPy-guide runs diverge slightly.
Do not hide this by attributing every final-state difference to the guide, or
by using an unbounded tolerance derived from a noisy reference run.
"""
    limits = {'particles': (1.e-4, 3.e-5, 1.e-5, 2.e-6),
              'velocities': (.1, .003, .01, .001)}
    comparisons = {}
    for key in actual:
        delta = actual[key].astype(float)-reference[key]
        repeat_delta = repeated[key].astype(float)-reference[key]
        row = dict(max_abs=float(np.max(np.abs(delta))), rms=float(np.sqrt(np.mean(delta**2))),
                   reference_repeat_max_abs=float(np.max(np.abs(repeat_delta))),
                   reference_repeat_rms=float(np.sqrt(np.mean(repeat_delta**2))))
        if key.endswith('joints'):
            row.update(max_limit=2.e-6, rms_limit=2.e-6)
        else:
            absolute_max, absolute_rms, floor_max, floor_rms = limits[key.removeprefix('motion_')]
            row['max_limit'] = min(absolute_max, 2*max(floor_max, row['reference_repeat_max_abs']))
            row['rms_limit'] = min(absolute_rms, 2*max(floor_rms, row['reference_repeat_rms']))
        row['passed'] = row['max_abs'] <= row['max_limit'] and row['rms'] <= row['rms_limit']
        comparisons[key] = row
    report['comparison'] = comparisons
    report['passed'] = all(row['passed'] for row in comparisons.values())
    return report['passed']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=20)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.steps < 10:
        parser.error('steps must be at least 10')
    args.output.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('dual_fr3_maniskill'))
    config_path = share / 'config/trunking_cable.yaml'
    config = load_config(config_path)
    # The frozen NumPy guide deliberately performs CPU readbacks each solve.
    config['mpm']['cuda_graph'] = False
    frequencies = yaml.safe_load((share / 'config/simulation_usb_cable.yaml').read_text())['dual_fr3_maniskill']['ros__parameters']
    joints = json.loads((share / 'config/profiling_trunking_pose.json').read_text())
    for side, width in [('left', config['usb']['finger_position']), ('right', 0.)]:
        joints.update({f'{side}_fr3_finger_joint{i}': width for i in (1, 2)})
    description, semantic = build_maniskill_description(scene='trunking_cable', cable_config=config_path)
    report = dict(steps=args.steps, post_reset_steps=5, cable_config=config, simulation_config=frequencies)
    report['numpy'], reference = run(ReferenceGuide, config, frequencies, joints, description, semantic, args.steps, args.output)
    report['numpy_repeat'], repeated = run(ReferenceGuide, config, frequencies, joints, description,
                                          semantic, args.steps, args.output, label='numpy_repeat')
    report['gpu'], actual = run(SlidingGuide, config, frequencies, joints, description, semantic, args.steps, args.output)
    report['state_max_absolute_difference'] = {key: float(np.max(np.abs(actual[key]-reference[key]))) for key in actual}
    validate_comparison(report, reference, repeated, actual)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
    assert report['passed'], report['comparison']
    print('PASS: moving arms, guide coupling and reset; differences:', report['state_max_absolute_difference'])


if __name__ == '__main__':
    main()
