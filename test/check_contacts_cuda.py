#!/usr/bin/env python3
"""Bounded CUDA regressions for tunnelling, deep overlap and moving bodies.

Run with the demo venv; these tests do not launch ROS or a viewer.
"""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import trimesh
from mani_skill2.envs.mpm.base_env import MPMModelBuilder  # selects matching Warp
import warp as wp
from warp.sim.model import Mesh, ModelBuilder

from dual_fr3_maniskill.cable.contacts import CableContacts


def setup(vertices, faces, center, spacing=.00175, frictionless=False):
    builder = ModelBuilder()
    body = builder.add_body(origin=wp.transform_identity())
    builder.add_shape_mesh(body=body, mesh=Mesh(vertices, faces.reshape(-1), compute_inertia=False))
    model = builder.finalize('cuda')
    points = np.tile(center, (3, 1)).astype(np.float32)
    points[:, 1] += np.arange(3)*spacing
    points = np.repeat(points, 7, axis=0)
    mass = wp.array(np.full(len(points), 1.e-5), dtype=float, device='cuda')
    model.struct = SimpleNamespace(particle_mass=mass)
    state = SimpleNamespace(body_q=model.body_q, body_qd=wp.zeros(1, dtype=wp.spatial_vector, device='cuda'),
                            struct=SimpleNamespace(particle_q=wp.array(points, dtype=wp.vec3, device='cuda'),
                                                   particle_C=wp.zeros(len(points), dtype=wp.mat33, device='cuda'),
                                                   particle_qd=wp.zeros(len(points), dtype=wp.vec3, device='cuda')))
    config = dict(diameter=.0035, contact_margin=.00002, friction=.5, contact_iterations=2)
    contacts = CableContacts(model, 3, 0, config, 'cuda',
                             frictionless_shapes=[0] if frictionless else [])
    contacts.capture(state.struct)
    return state, contacts, points


def run():
    wp.init()
    assert wp.is_cuda_available(), 'CUDA is required for this regression'
    box = trimesh.creation.box(extents=[.001, .1, .1])
    box.apply_translation([.003, 0, 0])  # wall lies between 6 mm MPM grid nodes
    state, contact, points = setup(box.vertices, box.faces, [-.01, 0, 0])
    moved = points + [.03, 0, 0]  # endpoint is completely beyond the thin wall
    state.struct.particle_q.assign(moved.astype(np.float32))
    state.struct.particle_qd.assign(np.tile([15., 0., 0.], (len(points), 1)).astype(np.float32))
    contact.solve(state)
    assert state.struct.particle_q.numpy()[:, 0].max() < .0025-.00175
    assert np.max(np.abs(state.struct.particle_qd.numpy())) < 1.e-5
    assert contact.audit(state).max() < 1.e-6
    assert contact.impulse.numpy()[0, 3] > 0, 'rigid body must receive the opposite impulse'
    print('PASS: 30 mm sweep through a 1 mm mesh wall, with momentum reaction', flush=True)

    # Sliding must remain possible; merely freezing colliding particles is not a fix.
    before = state.struct.particle_q.numpy()
    slide = before + [0, .0002, 0]
    state.struct.particle_q.assign(slide.astype(np.float32))
    contact.solve(state)
    np.testing.assert_allclose(state.struct.particle_q.numpy(), slide, atol=2.e-6)
    print('PASS: tangential sliding along the wall', flush=True)

    state, contact, points = setup(box.vertices, box.faces, [-.01, 0, 0], frictionless=True)
    state.struct.particle_q.assign((points + [.03, 0, 0]).astype(np.float32))
    state.struct.particle_qd.assign(np.tile([15., .3, 0.], (len(points), 1)).astype(np.float32))
    contact.solve(state)
    velocity = state.struct.particle_qd.numpy()
    np.testing.assert_allclose(velocity[:, 0], 0., atol=1.e-5)
    # Float32 closest-triangle normals introduce <0.1 mm/s tangential error
    # under this deliberately large 15 m/s normal impact.
    np.testing.assert_allclose(velocity[:, 1], .3, atol=1.e-4)
    print('PASS: frictionless guide contact removes normal speed and preserves axial speed', flush=True)

    cube = trimesh.creation.box(extents=[.02, .02, .02])
    state, contact, _ = setup(cube.vertices, cube.faces, [.008, 0, 0])
    assert contact.audit(state).max() > .003, 'deep initial overlap must be detected'
    contact.solve(state)
    assert state.struct.particle_q.numpy()[:, 0].min() > .01+.00175
    assert contact.audit(state).max() < 1.e-6
    print('PASS: deep mesh-interior overlap detected and resolved', flush=True)

    state, contact, _ = setup(box.vertices, box.faces, [.008, 0, 0])
    old = wp.array(state.body_q.numpy(), dtype=wp.transform, device='cuda')
    state.body_q.assign(np.array([[.012, 0, 0, 0, 0, 0, 1]], dtype=np.float32))
    state.body_qd.assign(np.array([[0, 0, 0, 6., 0, 0]], dtype=np.float32))
    contact.solve(state, old_body=old)
    assert state.struct.particle_q.numpy()[:, 0].min() > .0155+.00175
    assert contact.audit(state).max() < 1.e-6
    print('PASS: moving mesh sweeps into a stationary cable', flush=True)

    # A thin horizontal shelf intersects the middle of an edge while the
    # two physical endpoint spheres are clear. Inflated section spheres must
    # cover this otherwise untested gap in the rendered tube.
    shelf = trimesh.creation.box(extents=[.1, .0001, .1])
    state, contact, points = setup(shelf.vertices, shelf.faces, [.01, -.002, 0], spacing=.004)
    assert contact.audit(state).max() > .001
    contact.solve(state)
    # This already threaded initial state cannot preserve topology, but the
    # measurement must expose it instead of falsely declaring it collision-free.
    assert contact.audit(state).max() > .001
    print('PASS: audit detects a wall crossing between section centers', flush=True)

    # Real watertight trunking, with the scene's nonconvex cavity intact.
    path = Path(__file__).resolve().parents[2] / 'dual_fr3_moveit_config/meshes/Trunking.STL'
    trunk = trimesh.load(path, force='mesh', process=True)
    cavity = np.array([.673084, .7192516, .0583994]) - [.809, 1.123, 0]
    state, contact, points = setup(trunk.vertices, trunk.faces, cavity, spacing=.0001)
    assert contact.audit(state).max() == 0.0
    contact.solve(state)
    np.testing.assert_allclose(state.struct.particle_q.numpy(), points, atol=1.e-7)
    print('PASS: open cavity beside the wall rim is not misclassified as solid', flush=True)
    # Select a horizontal upward face well away from triangle edges; a
    # vertical sweep must stop on its top surface, even from above the grid.
    candidates = np.flatnonzero((trunk.face_normals[:, 2] > .999) & (trunk.area_faces > 1.e-5))
    face = candidates[np.argmax(trunk.area_faces[candidates])]
    p = trunk.triangles_center[face]
    state, contact, points = setup(trunk.vertices, trunk.faces, p + [0, 0, .02], spacing=.0001)
    state.struct.particle_q.assign((points - [0, 0, .04]).astype(np.float32))
    contact.solve(state)
    assert state.struct.particle_q.numpy()[:, 2].min() >= p[2] + .00175 - 1.e-6
    assert contact.audit(state).max() < 1.e-6
    print('PASS: 40 mm sweep stopped by actual nonconvex trunking STL', flush=True)


if __name__ == '__main__':
    run()
