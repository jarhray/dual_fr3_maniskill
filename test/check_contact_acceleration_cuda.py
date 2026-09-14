#!/usr/bin/env python3
"""CUDA regressions for conservative clearance reuse and candidate filtering.

Compare against a frozen contacts.py; no viewer, ROS node or robot connection.
"""
import argparse
from reference import CONTACTS_REFERENCE
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from dual_fr3_maniskill.sapien_compat import sapien
from mani_skill2.envs.mpm.base_env import MPMModelBuilder  # matching Warp
import warp as wp
from warp.sim.model import Mesh, ModelBuilder
from dual_fr3_maniskill.cable.contacts import CableContacts
from check_contact_equivalence_cuda import outputs


def box(size, center):
    mesh = trimesh.creation.box(extents=size)
    mesh.apply_translation(center)
    return mesh


def setup(meshes, reference, center=(0., 0., 0.), sections=3):
    builder = ModelBuilder()
    for mesh in meshes:
        body = builder.add_body(origin=wp.transform_identity())
        builder.add_shape_mesh(body=body, mesh=Mesh(mesh.vertices, mesh.faces.reshape(-1), compute_inertia=False))
    model = builder.finalize('cuda')
    centers = np.tile(center, (sections, 1))
    centers[:, 1] += np.arange(sections)*.00175
    points = np.repeat(centers, 7, axis=0).astype(np.float32)
    model.struct = NS(particle_mass=wp.array(np.full(len(points), 1.e-5, dtype=np.float32), device='cuda'))
    state = NS(body_q=model.body_q, body_qd=wp.zeros(len(meshes), dtype=wp.spatial_vector, device='cuda'),
               struct=NS(particle_q=wp.array(points, dtype=wp.vec3, device='cuda'),
                         particle_qd=wp.zeros(len(points), dtype=wp.vec3, device='cuda'),
                         particle_C=wp.zeros(len(points), dtype=wp.mat33, device='cuda')))
    config = dict(diameter=.0035, contact_margin=.00002, friction=.5, contact_iterations=2)
    old = reference.CableContacts(model, sections, 0, config, 'cuda')
    new = CableContacts(model, sections, 0, config, 'cuda')
    for solver in (old, new):
        solver.capture(state.struct)
    new.enable_profiling()
    return state, old, new, points


def compare(state, old, new, target, maxima, old_body=None):
    count = len(target)
    velocity = np.tile([.12, -.07, .03], (count, 1)).astype(np.float32)
    affine = np.tile(np.diag([.1, -.2, .3]), (count, 1, 1)).astype(np.float32)
    for solver in (old, new):
        state.struct.particle_q.assign(np.asarray(target, dtype=np.float32))
        state.struct.particle_qd.assign(velocity)
        state.struct.particle_C.assign(affine)
        solver.impulse.zero_()
        solver.hits.zero_()
        with (patch.object(wp.array, 'numpy', side_effect=AssertionError('solve readback')),
              patch.object(wp.array, 'assign', side_effect=AssertionError('solve upload'))):
            solver.solve(state, old_body=old_body, reuse_query_cache=True)
        if solver is old:
            expected = outputs(state, solver)
        else:
            actual = outputs(state, solver)
            for key, value in actual.items():
                error = float(np.max(np.abs(value-expected[key])))
                maxima[key] = max(maxima.get(key, 0.), error)
                # Only reaction reduction order may change when candidates
                # are compacted; all per-section computation stays ordered.
                tolerance = 2.e-8 if key == 'impulse' else 0.
                np.testing.assert_allclose(value, expected[key], rtol=0., atol=tolerance, err_msg=key)
    return actual


def totals(contact):
    rows = contact.profiling_report()['per_shape']
    result = {key: sum(row.get(key, 0) for row in rows) for key in
              ('mesh_point_queries', 'clearance_skips', 'exact_query_cache_hits', 'velocity_projections',
               'skipped_pass_pair_tests')}
    result['clearance_skips'] += sum(row.get('screen_clearance_skips', 0) for row in rows)
    return result


def run(reference):
    wp.init()
    assert wp.is_cuda_available()
    maxima = {}
    # A closed four-wall tunnel: free interior lies inside the mesh AABB.
    tunnel = trimesh.util.concatenate([
        box([.002, .024, .1], [x, 0., 0.]) for x in (-.011, .011)] + [
        box([.02, .002, .1], [0., y, 0.]) for y in (-.011, .011)])
    state, old, new, points = setup([tunnel], reference)
    compare(state, old, new, points, maxima)
    initial = totals(new)
    anchor = new.cache_point.numpy().copy()
    for step in range(16):
        moved = points + [.00002*(step+1), .00001*np.sin(step), 0.]
        compare(state, old, new, moved, maxima)
    warm = totals(new)
    assert warm['clearance_skips'] > initial['clearance_skips'], warm
    assert warm['mesh_point_queries'] == initial['mesh_point_queries'], (initial, warm)
    if 'skipped_pass_pair_tests' in new.profiling_report()['per_shape'][0]:
        assert warm['skipped_pass_pair_tests'] > initial['skipped_pass_pair_tests']
    np.testing.assert_array_equal(new.cache_point.numpy(), anchor)
    print('PASS: moving particles reuse measured clearance without copying or replacing the anchor', flush=True)
    if hasattr(new, 'active_count'):
        assert new.active_count.numpy()[0] == 0, 'All freely moving sections should be screened out'

    # A long sweep has both endpoints clear but crosses the tunnel wall.
    # Certification must consider travel, not just endpoint clearance.
    result = compare(state, old, new, points + [.03, 0., 0.], maxima)
    assert totals(new)['mesh_point_queries'] > warm['mesh_point_queries']
    assert result['positions'][:, 0].max() < .01-new.radius
    assert result['hits'].sum() > 0
    print('PASS: crossing a thin wall invalidates clearance and performs exact swept contact', flush=True)

    # A first pass can accept a target that the second pass moves out to the
    # contact margin. Whole-section screening must certify the endpoint too.
    state, old, new, points = setup([tunnel], reference)
    compare(state, old, new, points, maxima)
    guard = np.sqrt(new.radius**2 + .25*.00175**2)
    near_wall = points + [.01-guard-.000003, 0., 0.]
    result = compare(state, old, new, near_wall, maxima)
    assert result['hits'].sum() > 0
    assert result['positions'][0, 0] < near_wall[0, 0] - .00001
    print('PASS: second-pass endpoint correction is retained', flush=True)

    # Deep overlap must never become a positive clearance certificate.
    state, old, new, points = setup([tunnel], reference, center=(.0105, 0., 0.))
    result = compare(state, old, new, points, maxima)
    assert result['hits'].sum() > 0
    print('PASS: initial mesh-interior overlap follows the exact solver', flush=True)

    # Changing section spacing increases the covering radius even though the
    # center of the first section is stationary.
    state, old, new, points = setup([tunnel], reference)
    compare(state, old, new, points, maxima)
    count_before = totals(new)['mesh_point_queries']
    stretched = points.copy().reshape(3, 7, 3)
    stretched[1, :, 2] += .025
    result = compare(state, old, new, stretched.reshape(-1, 3), maxima)
    assert totals(new)['mesh_point_queries'] > count_before
    assert result['hits'].sum() > 0
    print('PASS: changed covering radius cannot reuse an insufficient clearance', flush=True)

    # The rigid body can move into stationary particles. Retain the old pose
    # so the original moving-body sweep is exercised with a warm cache.
    state, old, new, points = setup([tunnel], reference)
    compare(state, old, new, points, maxima)
    previous_body = wp.array(state.body_q.numpy(), dtype=wp.transform, device='cuda')
    state.body_q.assign(np.array([[.016, 0., 0., 0., 0., 0., 1.]], dtype=np.float32))
    state.body_qd.assign(np.array([[0., 0., 0., 8., 0., 0.]], dtype=np.float32))
    result = compare(state, old, new, points, maxima, old_body=previous_body)
    assert result['hits'].sum() > 0
    new.invalidate_query_cache()
    assert not new.cache_valid.numpy().any()
    print('PASS: moving-body sweep and explicit invalidation', flush=True)

    # Shape-local distances remain valid across translations and rotations.
    # The displacement bound uses freshly transformed endpoints on every solve.
    state, old, new, points = setup([tunnel], reference)
    compare(state, old, new, points, maxima)
    count_before = totals(new)['mesh_point_queries']
    for angle in np.linspace(-.12, .12, 13):
        previous_body = wp.array(state.body_q.numpy(), dtype=wp.transform, device='cuda')
        quaternion = Rotation.from_rotvec([angle, -.5*angle, .3*angle]).as_quat()
        position = np.array([.0001*np.sin(angle), -.0001*np.cos(angle), 0.])
        state.body_q.assign(np.r_[position, quaternion][None].astype(np.float32))
        compare(state, old, new, points, maxima, old_body=previous_body)
    assert totals(new)['mesh_point_queries'] == count_before
    print('PASS: warm local-space clearance survives rigid translations and rotations', flush=True)

    # Projection against the first shape can create a collision with a shape
    # that did not overlap the initial swept AABB. Keep the complete original
    # shape order for every retained section.
    shapes = [box([.02, .08, .08], [0., 0., 0.]),
              box([.02, .08, .08], [.023, 0., 0.])]
    state, old, new, points = setup(shapes, reference, center=(.002, 0., 0.))
    result = compare(state, old, new, points, maxima)
    assert np.all(result['hits'] > 0), result['hits']
    print('PASS: one contact activates another shape during ordered projection', flush=True)

    # No shape-count bitmask limit; a collider beyond slot 32 must be retained.
    shapes = [box([.001, .01, .01], [2.+i, 0., 0.]) for i in range(39)] + [tunnel]
    state, old, new, points = setup(shapes, reference)
    compare(state, old, new, points, maxima)
    result = compare(state, old, new, points + [.03, 0., 0.], maxima)
    assert result['hits'][-1] > 0 and not result['hits'][:-1].any()
    print('PASS: 40 collision shapes, including contact beyond index 32', flush=True)

    for passes in (1, 3, 4):
        state, old, new, points = setup([tunnel], reference)
        old.passes = new.passes = passes
        compare(state, old, new, points, maxima)
        compare(state, old, new, points + [.0001, 0., 0.], maxima)
        result = compare(state, old, new, points + [.03, 0., 0.], maxima)
        assert result['hits'].sum() > 0
    print('PASS: 1/3/4 projection passes preserve ordered contact behavior', flush=True)
    return dict(passed=True, max_absolute_difference=maxima,
                clearance_initial=initial, clearance_after_motion=warm)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=CONTACTS_REFERENCE,
                        help='Collision reference source (default: bundled frozen baseline)')
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    name = 'dual_fr3_maniskill.cable._acceleration_reference'
    spec = importlib.util.spec_from_file_location(name, args.reference)
    reference = importlib.util.module_from_spec(spec)
    sys.modules[name] = reference
    spec.loader.exec_module(reference)
    report = run(reference)
    report['reference'] = str(args.reference.resolve())
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
