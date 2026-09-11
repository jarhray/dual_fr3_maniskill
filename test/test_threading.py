from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dual_fr3_maniskill.cable.model import cable_particles, load_config, USB_LINK
from dual_fr3_maniskill.cable.threading import task_usb_mount, threaded_positions, guide_projection
from dual_fr3_maniskill.cable.guide import SlidingGuide


CONFIG = Path(__file__).resolve().parents[1] / "config/trunking_cable.yaml"


@pytest.mark.parametrize("left_yaw", [0., np.pi])
@pytest.mark.parametrize("right_yaw", [-np.pi/2, np.pi/2])
def test_layout_tracks_both_tcp_directions_and_preserves_collar_and_length(left_yaw, right_yaw):
    config = load_config(CONFIG)
    local, _, pinned, sections = cable_particles(config)
    mount_p, mount_r = task_usb_mount(config)
    left_r = Rotation.from_euler("xyz", [np.pi, 0., left_yaw]).as_matrix()
    right_r = Rotation.from_euler("xyz", [np.pi, 0., right_yaw]).as_matrix()
    r, p = left_r @ mount_r, np.array([.779, .483, .10]) + left_r @ mount_p
    hole, axis = np.array([.779, 1.083, .10]), right_r[:, 0]
    np.testing.assert_allclose(r[:, 1], left_r[:, 0])
    points, coordinate = threaded_positions(config, local, r, p, hole, axis)
    center = points.reshape(sections, 7, 3).mean(axis=1)
    np.testing.assert_allclose(points[pinned], local[pinned] @ r.T + p, atol=1e-10)
    length = np.linalg.norm(np.diff(center, axis=0), axis=1).sum()
    assert length == pytest.approx(config["cable"]["length"], abs=.0002)
    close = np.abs(np.arange(sections)-coordinate) < 8
    delta = center[close]-hole
    np.testing.assert_allclose(delta - (delta@axis)[:, None]*axis, 0., atol=1e-8)
    assert np.max(np.linalg.norm(np.diff(center, axis=0), axis=1)) < config["cable"]["particle_spacing"]*1.001


def test_too_short_cable_is_rejected_before_creating_particles():
    config = load_config(CONFIG)
    config["cable"]["length"] = .2
    local, *_ = cable_particles(config)
    with pytest.raises(ValueError, match="too short"):
        threaded_positions(config, local, np.eye(3), np.zeros(3), [0., -.6, 0.], [0., 1., 0.])


def test_material_crossing_changes_under_axial_sliding_without_axial_correction():
    center = np.column_stack((np.linspace(-.1, .1, 201), np.full(201, .001), np.zeros(201)))
    shift, weights, first = guide_projection(center, np.zeros(3), [1., 0., 0.], 100., .008, 4)
    np.testing.assert_allclose(shift[:, 0], 0.)
    np.testing.assert_allclose(shift[weights == 1., 1], -.001)
    center[:, 0] += .01
    _, _, second = guide_projection(center, np.zeros(3), [1., 0., 0.], first, .008, 4)
    assert second == pytest.approx(first-10.)
    assert np.all(shift[:4] == 0.)


class Array:
    def __init__(self, data):
        self.data = np.array(data)

    def numpy(self):
        return self.data.copy()

    def assign(self, data):
        self.data = data.copy()


def test_guide_removes_transverse_velocity_but_keeps_axial_sliding():
    centers = np.column_stack((np.linspace(-.1, .1, 201), np.zeros((201, 2))))
    state = NS(particle_q=Array(np.repeat(centers, 7, axis=0)),
               particle_qd=Array(np.tile([.3, .2, .1], (201*7, 1))))
    link = NS(pose=NS(p=np.zeros(3), q=[1., 0., 0., 0.]), velocity=np.zeros(3),
              angular_velocity=np.zeros(3), cmass_local_pose=NS(p=np.zeros(3)))
    cable = NS(states=[NS(struct=state)], sections=201, pin_ids_np=np.arange(28),
               model=NS(struct=NS(particle_mass=Array(np.full(201*7, 1e-5)))))
    guide = SlidingGuide(link, {"half_length": .008}, 100.)
    guide.solve(cable, .002)
    velocities = state.particle_qd.numpy().reshape(201, 7, 3)
    np.testing.assert_allclose(velocities[:, :, 0], .3)
    np.testing.assert_allclose(velocities[96:105, :, 1:], 0., atol=1e-7)
    assert guide.impulse[3] == pytest.approx(0.)
    assert guide.impulse[4] > 0.

def test_closed_shell_validation_keeps_cad_junctions_and_rejects_open_mesh():
    import trimesh
    from dual_fr3_maniskill.cable.mesh_contacts import closed_oriented_shells
    mesh = trimesh.creation.box()
    assert closed_oriented_shells(mesh)
    mesh.update_faces(np.arange(len(mesh.faces)-1))
    assert not closed_oriented_shells(mesh)

