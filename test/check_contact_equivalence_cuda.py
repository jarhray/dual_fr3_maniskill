#!/usr/bin/env python3
"""Compare collision acceleration with a saved pre-optimization implementation."""
import argparse
from reference import CONTACTS_REFERENCE
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from dual_fr3_maniskill.sapien_compat import sapien  # preserve Vulkan environment
from check_contacts_cuda import setup
import warp as wp


def outputs(state, contact):
    return dict(positions=state.struct.particle_q.numpy(), velocities=state.struct.particle_qd.numpy(),
                affine=state.struct.particle_C.numpy(), impulse=contact.impulse.numpy(),
                accepted=contact.accepted.numpy(), hits=contact.hits.numpy())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=CONTACTS_REFERENCE,
                        help='Collision reference source (default: bundled frozen baseline)')
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    name = 'dual_fr3_maniskill.cable._contact_reference'
    spec = importlib.util.spec_from_file_location(name, args.reference)
    reference = importlib.util.module_from_spec(spec)
    sys.modules[name] = reference
    spec.loader.exec_module(reference)
    wp.init()
    assert wp.is_cuda_available()
    random = np.random.default_rng(20260912)
    wall = trimesh.creation.box(extents=[.001, .1, .1])
    shape_rotation = Rotation.from_rotvec([.13, -.21, .29])
    shape_position = np.array([.003, -.002, .004])
    state, contact, template = setup(wall.vertices, wall.faces, [-.01, 0, 0],
        shape_pos=tuple(shape_position), shape_rot=tuple(shape_rotation.as_quat()))
    old = reference.CableContacts(contact.model, contact.sections, contact.pin_sections,
        dict(diameter=.0035, contact_margin=.00002, friction=.5, contact_iterations=2), 'cuda')
    old_body = wp.zeros_like(state.body_q)
    maximum = {}
    # Use quantities in their own units: position bounds are micrometres;
    # 0.1 mm/s matches the existing float32 mesh-contact velocity regression.
    tolerances = dict(positions=2e-6, accepted=2e-6, velocities=1e-4,
                      affine=1e-4, impulse=2e-8, hits=0.)
    failures = []
    count = 0
    for case in range(24):
        body_rotation = Rotation.from_rotvec(random.uniform(-1., 1., 3))
        body_position = random.uniform(-.2, .2, 3)
        local = template.copy()
        if case % 3 == 1:
            local[:, 0] = .008
        initial = body_rotation.apply(shape_rotation.apply(local) + shape_position) + body_position
        old_body.assign(np.r_[body_position, body_rotation.as_quat()][None].astype(np.float32))
        rotation = body_rotation
        position = body_position.copy()
        if case % 3 == 0:
            target = initial + body_rotation.apply(shape_rotation.apply([.03, .0002, 0.]))
        elif case % 3 == 1:
            position += body_rotation.apply(shape_rotation.apply([.02, 0., 0.]))
            target = initial.copy()
        else:
            rotation = body_rotation * Rotation.from_rotvec([0., .25, -.35])
            target = initial + body_rotation.apply(shape_rotation.apply([.02, 0., 0.]))
        state.body_q.assign(np.r_[position, rotation.as_quat()][None].astype(np.float32))
        state.body_qd.assign(random.uniform(-1., 1., (1, 6)).astype(np.float32))
        speed = random.uniform(-3., 3., (len(template), 3)).astype(np.float32)
        affine = random.uniform(-.1, .1, (len(template), 3, 3)).astype(np.float32)
        for solver in (old, contact):
            state.struct.particle_q.assign(initial.astype(np.float32))
            solver.capture(state.struct)
            solver.impulse.zero_()
            solver.hits.zero_()
        if case == 0:
            contact.enable_profiling()
        for solver in (old, contact):
            state.struct.particle_q.assign(target.astype(np.float32))
            state.struct.particle_qd.assign(speed)
            state.struct.particle_C.assign(affine)
            if solver is old:
                solver.solve(state, old_body=old_body)
                expected = outputs(state, solver)
            else:
                if case % 2:
                    solver.invalidate_query_cache()
                    solver.solve(state, old_body=old_body, reuse_query_cache=True)
                else:
                    solver.solve(state, old_body=old_body)
                actual = outputs(state, solver)
                for key, value in actual.items():
                    delta = float(np.max(np.abs(value - expected[key])))
                    maximum[key] = max(maximum.get(key, 0.), delta)
                    if not np.allclose(value, expected[key], rtol=0., atol=tolerances[key]):
                        failures.append(dict(case=case, field=key, max_difference=delta))
                count += 1
    details = contact.profiling_report()
    assert sum(row['pair_tests'] for row in details['per_shape']) <= 24*3*2
    if 'screen_pair_tests' in details['per_shape'][0]:
        assert sum(row['screen_pair_tests'] for row in details['per_shape']) == 24*3
    assert any(row['mesh_point_queries'] > 0 for row in details['per_shape'])
    assert all(row['mesh_ray_queries'] <= 3*row['mesh_point_queries'] for row in details['per_shape'])
    # Exercise actual cache hits in an open nonconvex cavity. The randomized
    # impacts above often move every query point, so misses alone are insufficient.
    mesh_path = Path(__file__).resolve().parents[2] / 'dual_fr3_moveit_config/meshes/Trunking.STL'
    trunk = trimesh.load(mesh_path, force='mesh', process=True)
    cavity = np.array([.673084, .7192516, .0583994]) - [.809, 1.123, 0]
    state, contact, points = setup(trunk.vertices, trunk.faces, cavity, spacing=.0001)
    contact.enable_profiling()
    for _ in range(3):
        contact.solve(state, reuse_query_cache=True)
    np.testing.assert_allclose(state.struct.particle_q.numpy(), points, rtol=0., atol=1e-7)
    cached = contact.profiling_report()['per_shape'][0]
    assert (cached['exact_query_cache_hits'] + cached.get('clearance_skips', 0)
            + cached.get('screen_clearance_skips', 0)) > 0, cached
    assert cached['mesh_point_queries'] + cached['exact_query_cache_hits'] == cached['surface_queries']
    contact.invalidate_query_cache()
    assert not contact.cache_valid.numpy().any()
    before_refresh = cached['mesh_point_queries']
    contact.solve(state, reuse_query_cache=True)
    refreshed = contact.profiling_report()['per_shape'][0]
    assert refreshed['mesh_point_queries'] > before_refresh
    report = dict(cases=count, max_absolute_difference=maximum, absolute_tolerances=tolerances,
                  failures=failures, contact_details=details, cache_hit_case=cached,
                  cache_invalidation_checked=True,
                  reference=str(args.reference.resolve()))
    args.report.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    assert not failures, failures
    print('PASS: translated/rotated shapes, particle sweeps, moving/rotating bodies, query caching', flush=True)


if __name__ == '__main__':
    main()
