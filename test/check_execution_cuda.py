#!/usr/bin/env python3
"""Exercise GPU bounds, graph replay, live poses, resize and reset on real CUDA.

The shadow checks replay identical constraint inputs, so MPM atomic reduction
noise cannot conceal a stale graph argument or double execution on capture.
"""
import argparse
import ctypes
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.cable.model import grid_layout, load_config
from dual_fr3_maniskill.cable.mpm_cable import initialize_warp
from dual_fr3_maniskill.cable.execution import DeviceBounds, ConstraintGraphs
from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableSimulation
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
import warp as wp


@wp.kernel
def increment(value: wp.array(dtype=int)):
    value[0] = value[0] + 1


def check_capture_failure():
    value = wp.zeros(1, dtype=int, device='cuda')
    cache = ConstraintGraphs(True)
    original = wp.capture_end

    def rejected():
        graph = original()
        wp.context.runtime.core.cuda_graph_destroy(ctypes.c_void_p(graph))
        raise RuntimeError('injected graph instantiation rejection')

    def record():
        wp.launch(increment, dim=1, inputs=[value], device='cuda')

    with patch.object(wp, 'capture_end', rejected):
        cache.run((), 0, record)
    cache.run((), 0, record)
    assert value.numpy()[0] == 2, 'Capture fallback lost or duplicated execution'
    assert not cache.enabled and not cache.capturing and not cache.graphs
    return True


def check_bounds(config):
    rng = np.random.default_rng(517)
    cases = 0
    for count in (1, 7, 128, 129, 10507):
        checker = DeviceBounds(count, 'cuda')
        q = wp.zeros(count, dtype=wp.vec3, device='cuda')
        for spacing in (.006, .0072, .015):
            for span in (.001, .048, .04800001, .23999998, 1.5):
                points = rng.uniform(-span, span, (count, 3)).astype(np.float32)
                points += np.array([-.732, 1.215, -.044], dtype=np.float32)
                q.assign(points)
                exact_spacing, exact = grid_layout(points, config, spacing)
                for dims in (exact, tuple(max(16, x-16) for x in exact),
                             tuple(x+16 for x in exact)):
                    summary = checker.check(q, spacing, config['mpm']['grid_padding'], dims)
                    must_grow = exact_spacing != spacing or any(a > b for a, b in zip(exact, dims))
                    assert summary is not None or not must_grow, (count, spacing, points, dims)
                    if summary is not None:
                        np.testing.assert_array_equal(summary, [points.min(axis=0), points.max(axis=0)])
                        assert grid_layout(summary, config, spacing) == (exact_spacing, exact)
                    cases += 1
        for invalid in (np.nan, np.inf, -np.inf, 1.e20):
            points = np.zeros((count, 3), dtype=np.float32)
            points[-1, 2] = invalid
            q.assign(points)
            try:
                checker.check(q, .006, 12, (32, 32, 32))
            except RuntimeError:
                cases += 1
            else:
                raise AssertionError(f'Invalid bound escaped: {invalid}')
    return cases


def mutable_arrays(cable):
    s, c, g = cable.states[0].struct, cable.contacts, cable.guide
    arrays = {name: getattr(s, name) for name in ('particle_q', 'particle_qd', 'particle_C')}
    arrays.update(pin_impulse=cable.pin_impulse, multipliers=cable.fibers.multipliers)
    for name in ('accepted', 'centers', 'impulse', 'hits', 'cache_point', 'cache_value', 'cache_valid'):
        arrays['contact_' + name] = getattr(c, name)
    for name in ('_centers', '_arc', '_chunk_data', '_offsets', '_control', '_reactions', '_summary'):
        arrays['guide' + name] = getattr(g, name)
    return arrays


def check_live(config, frequencies, joints, description, semantic):
    maxima, checks, seen_states = {}, 0, set()
    with tempfile.TemporaryDirectory(prefix='cable_graph_check_') as directory:
        assets = prepare_assets(description, semantic, Path(directory))
        assets.initial_positions.update(joints)
        sim = TrunkingCableSimulation(assets, cable_config=config,
            control_freq=frequencies['control_freq'], sim_freq=frequencies['sim_freq'])
        try:
            sim.env.spawn_cable()
            cable = sim.cable
            original = cable._solve_fibers
            calls = 0

            def shadow(dt):
                nonlocal calls, checks
                calls += 1
                # Both ping-pong states, creation and replay; repeat while the
                # arms move, after reallocation, after reset and at contact.
                if calls % 100 not in (1, 2):
                    return original(dt)
                arrays = mutable_arrays(cable)
                before = {name: wp.zeros_like(value) for name, value in arrays.items()}
                expected = {name: wp.zeros_like(value) for name, value in arrays.items()}
                for name, value in arrays.items():
                    wp.copy(before[name], value)
                cable.fibers.solve(cable.states[0].struct, cable.model.struct.particle_mass,
                    dt, cable.pin_com_device, cable.pin_impulse, lambda: cable._solve_constraints(dt))
                for name, value in arrays.items():
                    wp.copy(expected[name], value)
                    wp.copy(value, before[name])
                original(dt)
                seen_states.add(cable.states[0].struct.particle_q.ptr)
                for name, value in arrays.items():
                    actual, reference = value.numpy(), expected[name].numpy()
                    assert np.isfinite(actual).all() and np.isfinite(reference).all(), name
                    error = float(np.max(np.abs(actual.astype(float)-reference.astype(float))))
                    maxima[name] = max(maxima.get(name, 0.), error)
                    # Atomic force summation may change rounding; position,
                    # velocity, cache state and guide coordinates must agree.
                    tolerance = 2.e-8 if name in ('pin_impulse', 'contact_impulse',
                                                  'guide_summary') else 1.e-7
                    assert error <= tolerance, (name, error, tolerance)
                checks += 1

            initial = sim.target.copy()
            origin = sim.link_pose('right_fr3_hand_tcp')[:3].copy()
            motion = 0.
            resized = reset = False
            with patch.object(cable, '_solve_fibers', shadow):
                for step in range(20):
                    if step == 5:
                        old_dims = cable.grid_dims
                        # Allocate a larger grid without changing actual cable
                        # particles; this exercises graph retirement safely.
                        bounds = cable.positions[[0, -1]].copy()
                        bounds[1, 0] = bounds[0, 0] + old_dims[0] * cable.spacing
                        cable._ensure_grid(bounds)
                        assert cable.grid_dims != old_dims
                        assert not cable.constraint_graphs.graphs
                        resized = True
                    if step == 14:
                        sim.target[:] = initial
                        cable.reset()
                        assert not cable.constraint_graphs.graphs
                        reset = True
                    else:
                        phase = 2*np.pi*(step+1)/20
                        for name, amplitude in [('right_fr3_joint1', .002),
                                                ('right_fr3_joint7', .01), ('left_fr3_joint7', .003)]:
                            index = sim.indices[name]
                            sim.target[index] = initial[index] + amplitude*np.sin(phase)
                    sim.step()
                    d = cable.diagnostics()
                    assert d['attachment_error_m'] < 1.e-5, d
                    assert d['guide_radial_error_m'] < 1.e-4, d
                    motion = max(motion, float(np.linalg.norm(sim.link_pose('right_fr3_hand_tcp')[:3]-origin)))
                    if (step+1) % 5 == 0:
                        print(f'graph shadow: {step+1}/20 control steps', flush=True)
            assert checks == 40 and len(seen_states) == 2 and resized and reset
            assert motion > .0002
            assert cable.constraint_graphs.captures >= 6
            # A late change to iteration count must invalidate scalar args.
            previous = cable.constraint_graphs.captures
            cable.config['cable']['axial_iterations'] = 4
            sim.step()
            assert cable.constraint_graphs.captures == previous + 2
            # A resolution change updates model dx/grid buffers and retires
            # graphs too; a rejected allocation leaves the live state intact.
            before = cable.positions.copy()
            previous_spacing = cable.spacing
            cable.config['mpm']['max_grid_cells'] = 1000000
            cable._ensure_grid(np.array([[0., 0., 0.], [.85, .85, .85]], dtype=np.float32))
            assert cable.spacing > previous_spacing and not cable.constraint_graphs.graphs
            np.testing.assert_array_equal(cable.positions, before)
            sim.step()
            assert cable.constraint_graphs.graphs
            previous_graphs = dict(cable.constraint_graphs.graphs)
            before = cable.positions.copy()
            try:
                cable._ensure_grid(np.array([[0., 0., 0.], [100., 100., 100.]], dtype=np.float32))
            except RuntimeError as exc:
                assert 'memory budget' in str(exc)
            else:
                raise AssertionError('Grid exceeded its allocation budget')
            np.testing.assert_array_equal(cable.positions, before)
            assert cable.constraint_graphs.graphs == previous_graphs
            cable.contacts.enable_profiling()
            previous = cable.constraint_graphs.replays
            sim.step()
            assert cable.constraint_graphs.replays == previous
            assert cable.contacts.profile_calls == 410
            # Exercise the compatibility path independently of the main graph.
            fallback = ConstraintGraphs(True)
            recorded = []
            with patch.object(wp.config, 'verify_cuda', True):
                fallback.run((), 1, lambda: recorded.append(True))
            assert recorded == [True] and not fallback.enabled and fallback.fallback_reason
            return dict(shadow_checks=checks, max_absolute_difference=maxima,
                maximum_tcp_motion_m=motion, resized=resized, reset=reset,
                graph=cable.diagnostics()['cuda_graph'], fallback_checked=True,
                resolution_change_checked=True, rejected_allocation_checked=True,
                contact_profile_calls=cable.contacts.profile_calls)
        finally:
            sim.close()
            if 'cable' in locals():
                assert not cable.constraint_graphs.graphs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    initialize_warp()
    share = Path(get_package_share_directory('dual_fr3_maniskill'))
    path = share/'config/trunking_cable.yaml'
    config = load_config(path)
    config['cable']['axial_iterations'] = 6
    config['mpm'].update(cuda_graph=True, gpu_grid_check=True)
    frequencies = yaml.safe_load((share/'config/simulation_usb_cable.yaml').read_text())['dual_fr3_maniskill']['ros__parameters']
    joints = json.loads((share/'config/profiling_trunking_pose.json').read_text())
    for side, width in [('left', config['usb']['finger_position']), ('right', 0.)]:
        joints.update({f'{side}_fr3_finger_joint{i}': width for i in (1, 2)})
    description, semantic = build_maniskill_description(scene='trunking_cable', cable_config=path)
    report = dict(bounds_cases=check_bounds(config), capture_failure_checked=check_capture_failure())
    report['live'] = check_live(config, frequencies, joints, description, semantic)
    report['passed'] = True
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
