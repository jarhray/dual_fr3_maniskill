#!/usr/bin/env python3
"""Compare every live collision projection on identical GPU input buffers.

The accelerated trajectory drives both solvers. This avoids attributing MPM
run-to-run differences to collision code. Shadow outputs never drive physics.
"""
import argparse
from reference import CONTACTS_REFERENCE
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from dual_fr3_maniskill.cable import mpm_cable
from dual_fr3_maniskill.cable.contacts import CableContacts
from dual_fr3_maniskill.cable.guide import SlidingGuide
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
from check_trunking_contacts_cuda import load_reference
from check_trunking_guide_cuda import run
import warp as wp


@wp.kernel
def compare_particle_state(q: wp.array(dtype=wp.vec3), v: wp.array(dtype=wp.vec3),
    affine: wp.array(dtype=wp.mat33), reference_q: wp.array(dtype=wp.vec3),
    reference_v: wp.array(dtype=wp.vec3), reference_affine: wp.array(dtype=wp.mat33),
    maximum: wp.array2d(dtype=float)):
    i = wp.tid()
    dq = q[i] - reference_q[i]
    dv = v[i] - reference_v[i]
    da = affine[i] - reference_affine[i]
    position_error = float(0.0)
    velocity_error = float(0.0)
    affine_error = float(0.0)
    for j in range(3):
        position_error = wp.max(position_error, wp.abs(dq[j]))
        velocity_error = wp.max(velocity_error, wp.abs(dv[j]))
        if dq[j] != dq[j]:
            position_error = 1.e6
        if dv[j] != dv[j]:
            velocity_error = 1.e6
        for k in range(3):
            affine_error = wp.max(affine_error, wp.abs(da[j, k]))
            if da[j, k] != da[j, k]:
                affine_error = 1.e6
    if position_error > 0.0:
        maximum[i, 0] = wp.max(maximum[i, 0], position_error)
    if velocity_error > 0.0:
        maximum[i, 1] = wp.max(maximum[i, 1], velocity_error)
    if affine_error > 0.0:
        maximum[i, 2] = wp.max(maximum[i, 2], affine_error)


@wp.kernel
def compare_accepted(actual: wp.array(dtype=wp.vec3), reference: wp.array(dtype=wp.vec3),
                     maximum: wp.array2d(dtype=float)):
    i = wp.tid()
    delta = actual[i] - reference[i]
    for j in range(3):
        if delta[j] != delta[j]:
            maximum[i, 3] = wp.max(maximum[i, 3], 1.e6)
        elif delta[j] != 0.0:
            maximum[i, 3] = wp.max(maximum[i, 3], wp.abs(delta[j]))


@wp.kernel
def compare_reactions(actual: wp.array(dtype=wp.spatial_vector), reference: wp.array(dtype=wp.spatial_vector),
    actual_hits: wp.array(dtype=int), reference_hits: wp.array(dtype=int),
    bodies: int, shapes: int, maximum: wp.array2d(dtype=float)):
    i = wp.tid()
    if i < bodies:
        delta = actual[i] - reference[i]
        torque = wp.spatial_top(delta)
        force = wp.spatial_bottom(delta)
        for j in range(3):
            maximum[i, 4] = wp.max(maximum[i, 4], wp.abs(torque[j]))
            maximum[i, 4] = wp.max(maximum[i, 4], wp.abs(force[j]))
            if torque[j] != torque[j] or force[j] != force[j]:
                maximum[i, 4] = 1.e6
    if i < shapes:
        maximum[i, 5] = wp.max(maximum[i, 5], float(wp.abs(actual_hits[i] - reference_hits[i])))


def verify_comparator():
    """Inject known errors before relying on an all-zero live comparison."""
    wp.init()
    maximum = wp.zeros((2, 6), dtype=float, device='cuda')
    vector = wp.zeros(2, dtype=wp.vec3, device='cuda')
    matrix = wp.zeros(2, dtype=wp.mat33, device='cuda')
    q = wp.array(np.full((2, 3), .001, dtype=np.float32), dtype=wp.vec3, device='cuda')
    v = wp.array(np.full((2, 3), -.002, dtype=np.float32), dtype=wp.vec3, device='cuda')
    affine = wp.array(np.full((2, 3, 3), .003, dtype=np.float32), dtype=wp.mat33, device='cuda')
    accepted = wp.array(np.full((2, 3), .004, dtype=np.float32), dtype=wp.vec3, device='cuda')
    impulse = wp.zeros(2, dtype=wp.spatial_vector, device='cuda')
    reference = wp.array(np.full((2, 6), .005, dtype=np.float32), dtype=wp.spatial_vector, device='cuda')
    hits = wp.zeros(2, dtype=int, device='cuda')
    reference_hits = wp.array(np.array([1, 0], dtype=np.int32), device='cuda')
    wp.launch(compare_particle_state, dim=2, inputs=[vector, vector, matrix, q, v, affine, maximum], device='cuda')
    wp.launch(compare_accepted, dim=2, inputs=[vector, accepted, maximum], device='cuda')
    wp.launch(compare_reactions, dim=2,
              inputs=[impulse, reference, hits, reference_hits, 2, 2, maximum], device='cuda')
    np.testing.assert_allclose(maximum.numpy().max(axis=0), [.001, .002, .003, .004, .005, 1.], rtol=1.e-6)
    reference.assign(np.full((2, 6), np.nan, dtype=np.float32))
    wp.launch(compare_reactions, dim=2,
              inputs=[impulse, reference, hits, reference_hits, 2, 2, maximum], device='cuda')
    assert maximum.numpy()[:, 4].max() == 1.e6


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=CONTACTS_REFERENCE,
                        help='Collision reference source (default: bundled frozen baseline)')
    parser.add_argument('--steps', type=int, default=20)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.steps < 10:
        parser.error('steps must be at least 10')
    args.output.mkdir(parents=True, exist_ok=False)
    verify_comparator()
    reference = load_reference('_shadow_contact_reference', args.reference)
    instances = []

    class ShadowContacts(CableContacts):
        def __init__(self, *positional, **keyword):
            super().__init__(*positional, **keyword)
            self.reference = reference.CableContacts(*positional, **keyword)
            self.maximum = wp.zeros((max(self.sections*7, self.model.body_count, self.model.shape_count), 6),
                                    dtype=float, device=self.device)
            self.calls = 0
            self.shadow_state = None
            instances.append(self)

        def solve(self, state, old_body=None, *, reuse_query_cache=False):
            if self.shadow_state is None:
                self.shadow_state = NS(struct=NS(**{name: wp.zeros_like(getattr(state.struct, name))
                    for name in ('particle_q', 'particle_qd', 'particle_C')}))
            shadow = self.shadow_state
            shadow.body_q, shadow.body_qd = state.body_q, state.body_qd
            for name in ('particle_q', 'particle_qd', 'particle_C'):
                wp.copy(getattr(shadow.struct, name), getattr(state.struct, name))
            for name in ('accepted', 'impulse', 'hits'):
                wp.copy(getattr(self.reference, name), getattr(self, name))
            # Fresh reference queries are deliberately used; no accelerated
            # cache or projected state is supplied to the reference solver.
            self.reference.solve(shadow, old_body=old_body)
            super().solve(state, old_body=old_body, reuse_query_cache=reuse_query_cache)
            wp.launch(compare_particle_state, dim=self.sections*7, inputs=[state.struct.particle_q,
                state.struct.particle_qd, state.struct.particle_C, shadow.struct.particle_q,
                shadow.struct.particle_qd, shadow.struct.particle_C, self.maximum], device=self.device)
            wp.launch(compare_accepted, dim=self.sections,
                      inputs=[self.accepted, self.reference.accepted, self.maximum], device=self.device)
            wp.launch(compare_reactions, dim=max(self.model.body_count, self.model.shape_count),
                inputs=[self.impulse, self.reference.impulse, self.hits, self.reference.hits,
                        self.model.body_count, self.model.shape_count, self.maximum], device=self.device)
            self.calls += 1

    share = Path(get_package_share_directory('dual_fr3_maniskill'))
    config_path = share / 'config/trunking_cable.yaml'
    config = load_config(config_path)
    # This diagnostic observes every Python solve and lazily allocates shadow
    # buffers. Graph correctness is covered by check_execution_cuda.py.
    config['mpm']['cuda_graph'] = False
    frequencies = yaml.safe_load((share / 'config/simulation_usb_cable.yaml').read_text())['dual_fr3_maniskill']['ros__parameters']
    joints = json.loads((share / 'config/profiling_trunking_pose.json').read_text())
    for side, width in [('left', config['usb']['finger_position']), ('right', 0.)]:
        joints.update({f'{side}_fr3_finger_joint{i}': width for i in (1, 2)})
    description, semantic = build_maniskill_description(scene='trunking_cable', cable_config=config_path)
    with patch.object(mpm_cable, 'CableContacts', ShadowContacts):
        trajectory, snapshot = run(SlidingGuide, config, frequencies, joints, description,
                                    semantic, args.steps, args.output, label='shadow')
    fields = ('positions', 'velocities', 'affine', 'accepted', 'impulse', 'hits')
    maximum = np.max([solver.maximum.numpy().max(axis=0) for solver in instances], axis=0)
    limits = np.array([2.e-6, 1.e-4, 1.e-4, 2.e-6, 2.e-8, 0.])
    report = dict(calls=sum(solver.calls for solver in instances), trajectory=trajectory,
                  max_absolute_difference=dict(zip(fields, maximum.tolist())),
                  absolute_tolerances=dict(zip(fields, limits.tolist())),
                  passed=bool(np.all(np.isfinite(maximum)) and np.all(maximum <= limits)))
    (args.output / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'trajectory'}, indent=2))
    assert report['passed'], report['max_absolute_difference']
    print('PASS: every live collision solve agrees with fresh reference queries')


if __name__ == '__main__':
    main()
