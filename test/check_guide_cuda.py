#!/usr/bin/env python3
"""GPU guide regression against the frozen NumPy implementation; no ROS/viewer.

Run with the demo venv after sourcing install/setup.bash. Checks particle state,
reaction, sliding, moving poses, ping-pong buffers, and sticky error reporting.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
from transforms3d.euler import euler2quat
from transforms3d.quaternions import quat2mat
from mani_skill2.envs.mpm.base_env import MPMModelBuilder  # selects bundled Warp
import warp as wp

from dual_fr3_maniskill.cable.guide import SlidingGuide
from reference.guide_numpy import SlidingGuide as NumpyGuide


def setup(points, velocities, quaternion, hole, coordinate, cls=SlidingGuide, moving=False):
    link = NS(pose=NS(p=np.asarray(hole), q=quaternion), velocity=np.zeros(3),
              angular_velocity=np.zeros(3), cmass_local_pose=NS(p=np.array([.01, -.02, .015])))
    if moving:
        link.velocity = np.array([.02, -.03, .01])
        link.angular_velocity = np.array([.3, -.2, .1])
    link.applied = []
    link.add_force_torque = lambda force, torque: link.applied.append(np.r_[torque, force])
    state = NS(particle_q=wp.array(points, dtype=wp.vec3, device='cuda'),
               particle_qd=wp.array(velocities, dtype=wp.vec3, device='cuda'))
    cable = NS(states=[NS(struct=state)], device='cuda', sections=len(points)//7,
               pin_ids_np=np.arange(28), model=NS(struct=NS(particle_mass=wp.array(
                   np.linspace(.8e-5, 1.2e-5, len(points)).astype(np.float32), device='cuda'))))
    guide = cls(link, {'half_length': .008}, coordinate)
    if cls is SlidingGuide:
        guide.bind(cable)
        guide.update_pose()
    return guide, cable


def run():
    wp.init()
    assert wp.is_cuda_available(), 'CUDA is required for this regression'
    rng = np.random.default_rng(2917)
    results = []
    for sections in (201, 1501):
        for rotated in (False, True):
            for moving in (False, True):
                for dt in (.0002, .002):
                    quaternion = euler2quat(.7, -.4, 1.1) if rotated else np.array([1., 0., 0., 0.])
                    rotation = quat2mat(quaternion)
                    hole = np.array([.779, 1.083, .1]) if rotated else np.zeros(3)
                    t = (np.arange(sections)-(sections-1)/2)*.001
                    centers = np.column_stack((t, .0007*np.cos(9*t), .0004*np.sin(17*t)))
                    offsets = rng.normal(0., .0002, (sections, 7, 3))
                    offsets -= offsets.mean(axis=1)[:, None, :]
                    points = ((centers[:, None, :] + offsets).reshape(-1, 3) @ rotation.T + hole).astype(np.float32)
                    velocities = (rng.normal(0., .02, points.shape) + [.3, .2, .1]).astype(np.float32)
                    coordinate = (sections-1)/2
                    gpu, cable = setup(points, velocities, quaternion, hole, coordinate, moving=moving)
                    ref, reference = setup(points, velocities, quaternion, hole, coordinate, NumpyGuide, moving)
                    for iteration in range(6):
                        # Instrument all array methods, not just the input arrays:
                        # a hidden scratch-buffer copy in the solve must also fail.
                        with (patch.object(wp.array, 'numpy', side_effect=AssertionError('solve readback')),
                              patch.object(wp.array, 'assign', side_effect=AssertionError('solve upload')),
                              patch.object(wp, 'copy', side_effect=AssertionError('solve copy')),
                              patch.object(wp.context, 'copy', side_effect=AssertionError('solve copy'))):
                            gpu.solve(cable, dt)
                        ref.solve(reference, dt)
                    actual = cable.states[0].struct
                    expected = reference.states[0].struct
                    q, q_ref = actual.particle_q.numpy(), expected.particle_q.numpy()
                    v, v_ref = actual.particle_qd.numpy(), expected.particle_qd.numpy()
                    impulse, impulse_ref = gpu.impulse, ref.impulse.copy()
                    row = dict(sections=sections, rotated=rotated, moving=moving, dt=dt,
                               position_max_abs_m=float(np.max(np.abs(q-q_ref))),
                               velocity_max_abs_m_s=float(np.max(np.abs(v-v_ref))),
                               impulse_max_abs=float(np.max(np.abs(impulse-impulse_ref))),
                               coordinate_abs=abs(gpu.material_coordinate-ref.material_coordinate))
                    print(row, flush=True)
                    # Float32 geometry vs NumPy float64, at a 0.2 ms substep:
                    # sub-micrometre position and sub-mm/s velocity tolerances.
                    np.testing.assert_allclose(q, q_ref, rtol=0., atol=5.e-7)
                    np.testing.assert_allclose(v, v_ref, rtol=0., atol=4.e-4)
                    np.testing.assert_allclose(impulse, impulse_ref, rtol=2.e-4, atol=2.e-7)
                    assert row['coordinate_abs'] < .001  # < 1 micrometre at 1 mm spacing
                    np.testing.assert_array_equal(q[:28], points[:28])
                    np.testing.assert_array_equal(v[:28], velocities[:28])
                    gpu.apply_reaction(.002)
                    ref.apply_reaction(.002)
                    np.testing.assert_allclose(gpu.link.applied, ref.link.applied, rtol=2.e-4, atol=1.e-4)
                    np.testing.assert_array_equal(gpu.impulse, np.zeros(6))
                    results.append(row)

    centers = np.column_stack((np.linspace(-.1, .1, 201), np.zeros((201, 2))))
    points = np.repeat(centers, 7, axis=0).astype(np.float32)
    velocities = np.tile([.3, .2, .1], (len(points), 1)).astype(np.float32)
    gpu, cable = setup(points, velocities, [1., 0., 0., 0.], np.zeros(3), 100.)
    gpu.solve(cable, .002)
    velocity = cable.states[0].struct.particle_qd.numpy().reshape(201, 7, 3)
    np.testing.assert_allclose(velocity[:, :, 0], .3)
    np.testing.assert_allclose(velocity[96:105, :, 1:], 0., atol=1.e-7)
    assert abs(gpu.impulse[3]) < 1.e-10 and gpu.impulse[4] > 0
    # Swap the active MPM buffer, then move the gripper. No stale pointer or
    # pose may survive the explicit rigid-boundary update.
    old = cable.states[0]
    shifted = points + [.01, 0., 0.]
    cable.states.insert(0, NS(struct=NS(particle_q=wp.array(shifted, dtype=wp.vec3, device='cuda'),
                           particle_qd=wp.array(velocities, dtype=wp.vec3, device='cuda'))))
    gpu.link.pose.p = np.array([.002, .0003, 0.])
    gpu.update_pose()
    gpu.solve(cable, .002)
    assert abs(gpu.material_coordinate-92.) < .0001
    np.testing.assert_allclose(old.struct.particle_q.numpy(), points, rtol=0., atol=1.e-8)
    print('PASS: axial sliding, transverse velocity, moving pose and MPM buffer swap', flush=True)

    # Exactly representable coordinates exercise prefix-scan boundaries and
    # the original first-crossing tie rule without floating-point ambiguity.
    for coordinate in (31.5, 32., 63.5, 64., 95.5):
        center = np.column_stack(((np.arange(129)-coordinate)/1024., np.full(129, .001), np.zeros(129)))
        q = np.repeat(center, 7, axis=0).astype(np.float32)
        v = np.zeros_like(q)
        gpu, cable = setup(q, v, [1., 0., 0., 0.], np.zeros(3), coordinate)
        ref, reference = setup(q, v, [1., 0., 0., 0.], np.zeros(3), coordinate, NumpyGuide)
        gpu.solve(cable, .002)
        ref.solve(reference, .002)
        assert gpu.material_coordinate == ref.material_coordinate == coordinate
        np.testing.assert_allclose(cable.states[0].struct.particle_q.numpy(),
                                   reference.states[0].struct.particle_q.numpy(), rtol=0., atol=2.e-8)
    center[:, 0] = 1./1024.
    center[40:61, 0] = -1./1024.
    q = np.repeat(center, 7, axis=0).astype(np.float32)
    gpu, cable = setup(q, v, [1., 0., 0., 0.], np.zeros(3), 50.)
    gpu.solve(cable, .002)
    assert gpu.material_coordinate == 39.5  # equally distant crossings: 39.5, 60.5
    gpu.material_coordinate = 51.
    gpu.solve(cable, .002)
    assert gpu.material_coordinate == 60.5
    print('PASS: five chunk boundaries and nearest/first crossing selection', flush=True)

    failures = [('lost', points + [1., 0., 0.], 100., 'no longer crosses'),
                ('fixed', points, 5., 'fixed end'),
                ('free', points, 198., 'free end'),
                ('nan', points.copy(), 100., 'Invalid'),
                ('coordinate', points, np.nan, 'Invalid')]
    for name, bad_points, coordinate, message in failures:
        hole = np.zeros(3)
        if name in ('fixed', 'free'):
            hole[0] = centers[int(coordinate), 0]
        if name == 'nan':
            bad_points[0, 1] = np.nan
        gpu, cable = setup(bad_points, velocities, [1., 0., 0., 0.], hole, coordinate)
        gpu.solve(cable, .002)
        np.testing.assert_array_equal(cable.states[0].struct.particle_q.numpy(), bad_points.astype(np.float32))
        np.testing.assert_array_equal(cable.states[0].struct.particle_qd.numpy(), velocities)
        # Fix geometry without reset: the prior failure must stay visible.
        cable.states[0].struct.particle_q.assign(points)
        gpu.link.pose.p = np.zeros(3)
        gpu.update_pose()
        gpu.material_coordinate = 100.
        gpu.begin_step()
        gpu.solve(cable, .002)
        try:
            gpu.apply_reaction(.002)
        except RuntimeError as exc:
            assert message in str(exc), str(exc)
        else:
            raise AssertionError(f'{name}: missing error at rigid boundary')
        assert not gpu.link.applied
        gpu.begin_step(reset=True)
        gpu.solve(cable, .002)
        gpu.apply_reaction(.002)
        assert len(gpu.link.applied) == 1
        print(f'PASS: {name}, sticky failure, reset recovery', flush=True)
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rows = run()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({'comparisons': rows, 'passed': True}, indent=2)+'\n')
    print(f'PASS: {len(rows)} NumPy/GPU comparisons and guide behavior/error regressions')
