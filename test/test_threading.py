from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dual_fr3_maniskill.cable.model import cable_particles, load_config, USB_LINK
from dual_fr3_maniskill.cable.threading import (
    task_usb_mount, threaded_positions, threaded_centerline, guide_projection,
    left_guide_geometry, initial_layout_report, bore_alignment,
)


CONFIG = Path(__file__).resolve().parents[1] / "config/trunking_cable.yaml"


def test_guide_offset_tracks_rotating_tcp_without_changing_its_pose():
    from types import SimpleNamespace
    from dual_fr3_maniskill.cable.guide import SlidingGuide
    from dual_fr3_maniskill.sapien_compat import sapien

    config = load_config(CONFIG)
    tcp = SimpleNamespace(pose=sapien.Pose([.2, .3, .1]))
    guide = SlidingGuide(tcp, config["guide"], 0.)
    for rpy in ([0., 0., 0.], [np.pi, 0., .7], [.4, -.3, 1.2]):
        rotation = Rotation.from_euler("xyz", rpy)
        tcp.pose = sapien.Pose([.2, .3, .1], np.roll(rotation.as_quat(), 1))
        np.testing.assert_allclose(guide.pose.p,
            tcp.pose.p + rotation.apply([0., 0., .0024]), atol=2.e-8)
        np.testing.assert_allclose(tcp.pose.p, [.2, .3, .1])
    legacy = SlidingGuide(tcp, {}, 0.)
    np.testing.assert_allclose(legacy.pose.p, tcp.pose.p)


@pytest.mark.parametrize("section,key,value", [
    ("guide", "routing", "invalid"),
    ("guide", "center_offset", [0., 0.]),
    ("guide", "center_offset", [0., 0., float("nan")]),
    ("guide", "initial_margin", 0.),
    ("guide", "left_loop_clearance", -1.),
    ("guide", "axis_tolerance_deg", 90.),
    ("guide", "radial_tolerance", .002),
    ("scene", "trunking_mesh", "typo"),
    ("scene", "trunking_visual_mesh", "typo"),
])
def test_invalid_experiment_geometry_is_rejected(tmp_path, section, key, value):
    import yaml
    config = load_config(CONFIG)
    config[section][key] = value
    path = tmp_path/"invalid.yaml"
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match=f"{section}.{key}"):
        load_config(path)


@pytest.mark.parametrize("left_yaw", [0., np.pi])
@pytest.mark.parametrize("right_yaw", [-np.pi/2, np.pi/2])
@pytest.mark.parametrize("direction,sign", [("forward", 1.), ("reverse", -1.)])
def test_layout_tracks_both_tcp_directions_and_preserves_collar_and_length(left_yaw, right_yaw, direction, sign):
    config = load_config(CONFIG)
    config["guide"]["routing"] = "both_guides"
    config["usb"]["orientation_direction"] = direction
    local, _, pinned, sections = cable_particles(config)
    mount_p, mount_r = task_usb_mount(config, orientation_direction=direction)
    left_r = Rotation.from_euler("xyz", [np.pi, 0., left_yaw]).as_matrix()
    right_r = Rotation.from_euler("xyz", [np.pi, 0., right_yaw]).as_matrix()
    r, p = left_r @ mount_r, np.array([.779, .483, .10]) + left_r @ mount_p
    hole, axis = np.array([.779, 1.083, .10]), right_r[:, 0]
    np.testing.assert_allclose(r[:, 1], sign*left_r[:, 0])
    np.testing.assert_allclose(r @ config["usb"]["grip_center"] + p,
                               [.779, .483, .10-.012], atol=1e-10)
    points, coordinate = threaded_positions(config, local, r, p, hole, axis)
    center = points.reshape(sections, 7, 3).mean(axis=1)
    np.testing.assert_allclose(points[pinned], local[pinned] @ r.T + p, atol=1e-10)
    length = np.linalg.norm(np.diff(center, axis=0), axis=1).sum()
    assert length == pytest.approx(config["cable"]["length"], abs=.0002)
    close = np.abs(np.arange(sections)-coordinate) < 8
    delta = center[close]-hole
    np.testing.assert_allclose(delta - (delta@axis)[:, None]*axis, 0., atol=1e-8)
    assert np.max(np.linalg.norm(np.diff(center, axis=0), axis=1)) < config["cable"]["particle_spacing"]*1.001
    report = initial_layout_report(config, center, r, p, hole, axis)
    assert report["passed"], report
    for side in ("left", "right"):
        assert report[side]["radial_error_m"] < 1.e-9
        assert report[side]["axis_error_deg"] < 1.e-4
        lo, hi = report[side]["straight_coverage_range_m"]
        assert lo < -.030 and hi > .030


@pytest.mark.parametrize("direction,sign", [("forward", 1.), ("reverse", -1.)])
@pytest.mark.parametrize("rpy", [[np.pi, 0., .7], [.4, -.3, 1.2]])
def test_grip_offset_rotates_with_tcp_and_preserves_usb_grip_coordinates(direction, sign, rpy):
    config = load_config(CONFIG)
    config["usb"]["grip_center"] = [.001, -.002, .003]
    config["guide"]["routing"] = "both_guides"
    config["usb"]["orientation_direction"] = direction
    p, r = task_usb_mount(config)
    tcp_r = Rotation.from_euler("xyz", rpy).as_matrix()
    np.testing.assert_allclose(tcp_r @ (r @ config["usb"]["grip_center"] + p),
                               .012*tcp_r[:, 2], atol=1e-12)
    np.testing.assert_allclose(r[:, 1], [sign, 0., 0.])
    np.testing.assert_allclose(r.T @ r, np.eye(3))
    assert np.linalg.det(r) == pytest.approx(1.)


def test_mount_defaults_to_reverse_and_accepts_custom_grip_offset():
    config = load_config(CONFIG)
    config["usb"]["tcp_grip_offset"] = [.001, -.002, .008]
    p, r = task_usb_mount(config)
    np.testing.assert_allclose(r[:, 1], [-1., 0., 0.])
    np.testing.assert_allclose(r @ config["usb"]["grip_center"] + p, [.001, -.002, .008])
    with pytest.raises(ValueError, match="orientation_direction"):
        task_usb_mount(config, orientation_direction="invalid")


def test_too_short_cable_is_rejected_before_creating_particles():
    config = load_config(CONFIG)
    config["cable"]["length"] = .2
    local, *_ = cable_particles(config)
    with pytest.raises(ValueError, match="too short"):
        threaded_positions(config, local, np.eye(3), np.zeros(3), [0., -.6, 0.], [0., 1., 0.])


def test_bore_measurement_rejects_shift_tilt_and_partial_coverage():
    centers = np.column_stack((np.linspace(-.04, .04, 9), np.zeros((9, 2))))
    assert bore_alignment(centers, np.zeros(3), [1, 0, 0], .012)["passed"]
    shifted = centers+[0, .0005, 0]
    result = bore_alignment(shifted, np.zeros(3), [1, 0, 0], .012)
    assert not result["passed"] and result["radial_error_m"] >= .0005
    tilted = centers @ Rotation.from_euler("z", 3, degrees=True).as_matrix().T
    assert not bore_alignment(tilted, np.zeros(3), [1, 0, 0], .012)["passed"]
    assert not bore_alignment(centers[3:6], np.zeros(3), [1, 0, 0], .012)["passed"]
    assert not bore_alignment(centers[5:], np.zeros(3), [1, 0, 0], .012)["passed"]


@pytest.mark.parametrize("direction", ["forward", "reverse"])
def test_sparse_rigid_chain_has_unstrained_straight_spans_in_both_bores(direction):
    from dual_fr3_maniskill.cable.rope_actor import rigid_resample
    config = load_config(CONFIG, solver="rope_actor")
    config["guide"]["routing"] = "both_guides"
    config["usb"]["orientation_direction"] = direction
    p, r = task_usb_mount(config)
    hole, axis = np.array([0, .6, .0024]), np.array([0, 1, 0])
    curve, _ = threaded_centerline(config, np.linspace(0, 1.5, 3001), r, p, hole, axis)
    lengths = np.r_[.007, np.full(150, (1.5-.007)/150)]
    nodes = rigid_resample(curve, lengths)
    np.testing.assert_allclose(np.linalg.norm(np.diff(nodes, axis=0), axis=1), lengths, atol=1.e-12)
    report = initial_layout_report(config, nodes, r, p, hole, axis)
    assert report["passed"], report
    assert len(report["support_material_intervals_m"]) == 2
    assert sum(b-a for a, b in report["support_material_intervals_m"]) < .10


def test_left_bore_uses_distinct_grip_exit_and_hole_centers_under_arbitrary_motion():
    config = load_config(CONFIG)
    config["usb"]["grip_center"] = [.001, -.002, .003]
    config["usb"]["tcp_grip_offset"] = [.004, -.003, .012]
    p, r = task_usb_mount(config)
    tcp_r = Rotation.from_euler("xyz", [.4, -.7, 1.2]).as_matrix()
    tcp_p = np.array([.2, -.4, .8])
    hole, axis, outward = left_guide_geometry(config, tcp_r@r, tcp_p+tcp_r@p)
    np.testing.assert_allclose(hole, tcp_p+tcp_r@config["guide"]["center_offset"])
    np.testing.assert_allclose(axis, tcp_r[:, 0])
    np.testing.assert_allclose(outward, tcp_r[:, 2])


def test_material_crossing_changes_under_axial_sliding_without_axial_correction():
    center = np.column_stack((np.linspace(-.1, .1, 201), np.full(201, .001), np.zeros(201)))
    shift, weights, first = guide_projection(center, np.zeros(3), [1., 0., 0.], 100., .008, 4)
    np.testing.assert_allclose(shift[:, 0], 0.)
    np.testing.assert_allclose(shift[weights == 1., 1], -.001)
    center[:, 0] += .01
    _, _, second = guide_projection(center, np.zeros(3), [1., 0., 0.], first, .008, 4)
    assert second == pytest.approx(first-10.)
    assert np.all(shift[:4] == 0.)


# The particle/velocity guide regression now runs on real CUDA arrays in
# check_guide_cuda.py, alongside comparison against the frozen NumPy solver.

def test_closed_shell_validation_keeps_cad_junctions_and_rejects_open_mesh():
    import trimesh
    from dual_fr3_maniskill.cable.mesh_contacts import closed_oriented_shells
    mesh = trimesh.creation.box()
    assert closed_oriented_shells(mesh)
    mesh.update_faces(np.arange(len(mesh.faces)-1))
    assert not closed_oriented_shells(mesh)


@pytest.mark.parametrize("direction", ["forward", "reverse"])
def test_direct_usb_to_right_has_no_left_loop_or_left_support(direction):
    from dual_fr3_maniskill.cable.rope_actor import rigid_resample
    config = load_config(CONFIG, solver="rope_actor")
    assert config["guide"]["routing"] == "usb_to_right"
    config["usb"]["orientation_direction"] = direction
    p, r = task_usb_mount(config)
    hole, axis = np.array([0., .6, .0024]), np.array([0., 1., 0.])
    s = np.linspace(0., 1.5, 3001)
    points, _ = threaded_centerline(config, s, r, p, hole, axis)
    lengths = np.r_[.007, np.full(150, (1.5-.007)/150)]
    nodes = rigid_resample(points, lengths)
    report = initial_layout_report(config, nodes, r, p, hole, axis)
    assert report["passed"] and report["right"]["passed"]
    assert report["left"]["available"] is False and report["left"]["passed"] is None
    assert report["left_material_arc_m"] is None
    assert len(report["support_material_intervals_m"]) == 1
    assert report["right_material_arc_m"] < .65
    # A direct lead stays between the USB exit and bore heights: no lower U loop.
    np.testing.assert_array_less(points[:, 2], max(p[2], hole[2])+.000001)
    assert points[:, 2].min() >= min(p[2], hole[2])-.000001
    np.testing.assert_allclose(np.linalg.norm(np.diff(nodes, axis=0), axis=1), lengths, atol=1.e-12)
    exit_point = r @ config["usb"]["attachment"]+p
    np.testing.assert_allclose(nodes[0], exit_point, atol=1.e-12)
    np.testing.assert_allclose(nodes[1], exit_point-.007*r[:, 1], atol=1.e-5)
