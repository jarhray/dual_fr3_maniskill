#!/usr/bin/env python3
"""Compare graph/bounds acceleration and 6/4 iterations through motion and reset.

An unchanged six-iteration repeat measures float32 MPM variability. Report the
entire sampled trajectory, not just a fortunate endpoint. The 0.16 mm position
budget is explicit; penetration and attachment checks retain their own limits.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
from unittest.mock import patch

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_maniskill.cable.mpm_cable import MPMCable
from dual_fr3_maniskill.cable.guide import SlidingGuide
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
from check_trunking_guide_cuda import run


def compare(reference, actual, position_limit):
    result = {}
    for key in actual:
        a, b = actual[key].astype(float), reference[key].astype(float)
        delta = a-b
        row = dict(max_abs=float(np.max(np.abs(delta))), rms=float(np.sqrt(np.mean(delta**2))))
        if key.endswith('particles'):
            row.update(max_limit=position_limit, rms_limit=3.e-5,
                       max_point_distance=float(np.max(np.linalg.norm(delta, axis=-1))))
        elif key.endswith('velocities'):
            row.update(max_limit=.1, rms_limit=.003)
        else:
            row.update(max_limit=2.e-6, rms_limit=2.e-6)
        row['passed'] = row['max_abs'] <= row['max_limit'] and row['rms'] <= row['rms_limit']
        result[key] = row
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=20)
    parser.add_argument('--position-limit-mm', type=float, default=.16)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.steps < 10 or not np.isfinite(args.position_limit_mm) or args.position_limit_mm <= 0:
        parser.error('Use at least 10 steps and a positive finite position budget')
    args.output.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('dual_fr3_maniskill'))
    path = share/'config/trunking_cable.yaml'
    config = load_config(path)
    frequencies = yaml.safe_load((share/'config/simulation_usb_cable.yaml').read_text())['dual_fr3_maniskill']['ros__parameters']
    joints = json.loads((share/'config/profiling_trunking_pose.json').read_text())
    for side, width in [('left', config['usb']['finger_position']), ('right', 0.)]:
        joints.update({f'{side}_fr3_finger_joint{i}': width for i in (1, 2)})
    description, semantic = build_maniskill_description(scene='trunking_cable', cable_config=path)
    report = dict(steps=args.steps, post_reset_steps=5, position_limit_mm=args.position_limit_mm,
                  cable_config=config, simulation_config=frequencies,
                  source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in Path(__file__).resolve().parents[1].joinpath('dual_fr3_maniskill/cable').glob('*.py')})
    states = {}
    for label, graph, bounds, iterations in [('plain6', False, False, 6),
            ('repeat6', False, False, 6), ('accelerated6', True, True, 6),
            ('accelerated4', True, True, 4)]:
        current = deepcopy(config)
        current['mpm'].update(cuda_graph=graph, gpu_grid_check=bounds)
        current['cable']['axial_iterations'] = iterations
        samples = []
        original = MPMCable.diagnostics

        def diagnostics(cable):
            value = original(cable)
            samples.append(cable.positions.copy())
            return value

        start = time.perf_counter()
        with patch.object(MPMCable, 'diagnostics', diagnostics):
            result, state = run(SlidingGuide, current, frequencies, joints, description,
                                semantic, args.steps, args.output, label=label)
        result['setup_and_checks_wall_seconds'] = time.perf_counter()-start
        state['trajectory_particles'] = np.asarray(samples)
        np.savez_compressed(args.output/(label+'_state.npz'), **state)
        states[label], report[label] = state, result
        (args.output/'partial_report.json').write_text(json.dumps(report, indent=2)+'\n')
    report['comparison'] = {label: compare(states['plain6'], states[label], args.position_limit_mm/1000)
                            for label in ('repeat6', 'accelerated6', 'accelerated4')}
    report['acceleration_accepted'] = all(r['passed'] for r in report['comparison']['accelerated6'].values())
    report['four_iterations_accepted'] = all(r['passed'] for r in report['comparison']['accelerated4'].values())
    report['selected_iterations'] = 4 if report['four_iterations_accepted'] else 6
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key: value for key, value in report.items() if key in (
        'comparison', 'acceleration_accepted', 'four_iterations_accepted', 'selected_iterations')}, indent=2))
    assert report['acceleration_accepted'], 'Graph/bounds motion validation failed; inspect report.json'


if __name__ == '__main__':
    main()
