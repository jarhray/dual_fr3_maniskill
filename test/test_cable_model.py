from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from dual_fr3_maniskill.cable.model import (
    USB_LINK, add_usb_description, cable_particles, grid_layout, load_config, usb_mount,
    initial_particle_positions)

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('length', [1.5, 3.0])
def test_cable_dimensions_mass_and_attachment(length):
    config = load_config(ROOT / "config/usb_cable.yaml")
    config['cable']['length'] = length
    points, volume, pinned, sections = cable_particles(config)
    center = points.reshape(sections, 7, 3).mean(axis=1)
    np.testing.assert_allclose(center[0], [0., -.02, 0.], atol=1e-12)
    np.testing.assert_allclose(center[-1], [0., -.02-length, 0.], atol=1e-12)
    assert volume.sum() == pytest.approx(np.pi * (.0035 / 2) ** 2 * length)
    assert volume.sum() * config['cable']['density'] == pytest.approx(.015 * length)
    assert pinned.sum() >= 7
    assert np.ptp(points[pinned, 1]) <= .007
    assert np.linalg.norm(np.diff(center, axis=0), axis=1).sum() == pytest.approx(length)


@pytest.mark.parametrize("filename", ["usb_cable.yaml", "trunking_cable.yaml", "trunking_cable_simplified_2mm.yaml"])
def test_reference_total_mass_and_shortened_cable(tmp_path, filename):
    cfg = load_config(ROOT/"config"/filename, solver="rope_actor")
    # 45 g at 2 m, retaining the existing unweighed 15 g USB estimate.
    assert cfg["cable"]["linear_density"]*2+cfg["usb"]["mass"] == pytest.approx(.045)
    assert cfg["cable"]["linear_density"]*1.5+cfg["usb"]["mass"] == pytest.approx(.0375)
    # Geometry changes and serialized resolved configs must not change kg/m.
    cfg["cable"]["diameter"] *= 1.2
    path = tmp_path/"resized.yaml"
    path.write_text(yaml.safe_dump(cfg))
    c = load_config(path, solver="rope_actor")["cable"]
    assert c["density"]*np.pi*(c["diameter"]/2)**2 == pytest.approx(.015)


def test_legacy_volume_density_remains_supported(tmp_path):
    cfg = load_config(ROOT/"config/usb_cable.yaml")
    cfg["cable"].pop("linear_density")
    cfg["cable"]["density"] = 1200.
    path = tmp_path/"legacy.yaml"
    path.write_text(yaml.safe_dump(cfg))
    assert load_config(path)["cable"]["density"] == 1200.


def test_fixed_mount_uses_original_usb_coordinates():
    config = load_config(ROOT / "config/usb_cable.yaml")
    t, r = usb_mount(config)
    np.testing.assert_allclose(r @ config['usb']['grip_center'] + t, 0., atol=1e-12)
    # Actual left TCP ready rotation with joint7=3*pi/4, compared with URDF FK.
    np.testing.assert_allclose(np.array([[0,-1,0],[-1,0,0],[0,0,-1]]) @ r, np.eye(3))
    source = '<robot name="test"><link name="left_fr3_hand_tcp"/></robot>'
    srdf = '<robot name="test"><group_state name="ready" group="left_fr3_arm"><joint name="left_fr3_joint7" value="0.7854"/></group_state></robot>'
    robot, semantic = add_usb_description(source, srdf, ROOT / "meshes/USB1.stl", config)
    root = ET.fromstring(robot)
    assert root.find(f"link[@name='{USB_LINK}']/visual/geometry/mesh").get('scale') == '1.0 1.0 1.0'
    assert root.find('joint').get('type') == 'fixed'
    assert float(ET.fromstring(semantic).find('group_state/joint').get('value')) == pytest.approx(3*np.pi/4)
    assert USB_LINK not in source


def test_grid_budget_and_invalid_state():
    config = load_config(ROOT / "config/usb_cable.yaml")
    points = np.array([[0,0,0],[0,3,0]])
    dx, dims = grid_layout(points, config)
    assert dx == .006 and np.prod(dims) <= config['mpm']['max_grid_cells']
    large = np.array([[0,0,0],[3,3,3]])
    coarse, dims = grid_layout(large, config)
    assert coarse > dx and np.prod(dims) <= config['mpm']['max_grid_cells']
    with pytest.raises(RuntimeError, match='memory budget'):
        grid_layout(large * 100, config)
    with pytest.raises(ValueError, match='finite'):
        grid_layout([[0,0,0],[np.nan,1,1]], config)


@pytest.mark.parametrize('configured_length', [1.5, 3.0])
def test_spiral_preserves_material_length_and_rigid_collar(configured_length):
    config = load_config(ROOT / "config/usb_cable.yaml")
    config['cable']['length'] = configured_length
    local, _, pinned, sections = cable_particles(config)
    translation = np.array([.474690567, .338, .434082052])
    world = initial_particle_positions(config, local, np.eye(3), translation)
    center = world.reshape(sections, 7, 3).mean(axis=1)
    length = np.linalg.norm(np.diff(center, axis=0), axis=1).sum()
    assert length == pytest.approx(config['cable']['length'], abs=1e-4)
    np.testing.assert_allclose(world[pinned], local[pinned] + translation)
    assert np.isfinite(world).all()
    assert world[:, 2].min() > .07
    assert np.ptp(world[:, 1]) < 1.


@pytest.mark.parametrize('section,key,value', [
    ('cable','length',-1), ('cable','diameter',0), ('cable','friction',-1),
    ('cable','linear_density',0), ('cable','linear_density',True),
    ('cable','linear_density',float('nan')), ('cable','linear_density',float('inf')),
    ('usb','attachment',[0,0]), ('usb','finger_position',.1),
    ('usb','tcp_grip_offset',[0,0]), ('usb','tcp_grip_offset',[0,0,float('nan')]),
    ('mpm','frequency',12.5), ('mpm','grid_padding',1),
    ('mpm','cuda_graph','false'), ('mpm','gpu_grid_check',1),
    ('cable','contact_margin',0), ('cable','contact_iterations',0),
    ('cable','penetration_tolerance',.0035)])
def test_invalid_config(tmp_path, section, key, value):
    config = deepcopy(load_config(ROOT / 'config/usb_cable.yaml'))
    config[section][key] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        load_config(path)
