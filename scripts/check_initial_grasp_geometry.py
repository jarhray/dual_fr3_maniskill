#!/usr/bin/env python3
"""CPU-only CAD evidence for the two-bore creation layout; no physics claim."""
import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_maniskill.cable.threading import (
    initial_layout_report, task_usb_mount, threaded_centerline,
)

ROOT = Path(__file__).resolve().parents[1]
MOVEIT = ROOT.parent / "dual_fr3_moveit_config"
if not (ROOT / "config/trunking_cable_simplified_2mm.yaml").is_file():
    # Normal installs copy this executable into lib/<package>; assets remain
    # in package share directories. Source and symlink installs use ROOT above.
    from ament_index_python.packages import get_package_share_directory
    ROOT = Path(get_package_share_directory("dual_fr3_maniskill"))
    MOVEIT = Path(get_package_share_directory("dual_fr3_moveit_config"))


def cad_fingers(q=0.):
    """Original meshes in hand-TCP coordinates, using current Xacro origins."""
    tree = ET.parse(MOVEIT / "config/research_franka_hand.xacro")
    properties = {e.attrib['name']: e.attrib['value'] for e in tree.iter()
                  if e.tag.endswith('property')}
    macro = next(e for e in tree.iter() if e.tag.endswith('macro'))
    tcp_z = float(re.search(r"tcp_xyz:='0 0 ([0-9.]+)'", macro.attrib['params'])[1])
    finger_joint = next(e for e in tree.iter('joint') if e.attrib['name'].endswith('finger_joint1'))
    joint_origin = np.fromstring(finger_joint.find('origin').attrib['xyz'], sep=' ')
    result = []
    for side, angle in (("left", 0.), ("right", np.pi)):
        mesh = trimesh.load(MOVEIT / 'meshes/research_finger/finger1.STL', force='mesh')
        offset = np.fromstring(properties[f'research_{side}_finger_xyz'], sep=' ')
        transform = trimesh.transformations.compose_matrix(translate=offset, angles=[0, 0, -np.pi/2])
        assembly = trimesh.transformations.compose_matrix(translate=joint_origin-[0, 0, tcp_z], angles=[0, 0, angle])
        assembly[1, 3] += q if side == 'left' else -q
        mesh.apply_transform(assembly @ transform)
        result.append(mesh)
    return result


def usb_mesh(config):
    mesh = trimesh.load(ROOT / 'meshes/USB1.stl', force='mesh')
    p, r = task_usb_mount(config)
    mesh.vertices = mesh.vertices * config['usb']['mesh_scale'] @ r.T + p
    return mesh


def audit(config):
    from dual_fr3_maniskill.cable.rope_actor import rigid_resample
    p, r = task_usb_mount(config)
    hole, axis = np.array([0., .6, .0024]), np.array([0., 1., 0.])
    hole[2] = config['guide']['center_offset'][2]
    s = np.linspace(0., config['cable']['length'], int(np.ceil(config['cable']['length']/.0005))+1)
    points, _ = threaded_centerline(config, s, r, p, hole, axis)
    layout = initial_layout_report(config, points, r, p, hole, axis)
    c, rope = config['cable'], config['rope_actor']
    lengths = np.r_[c['pin_length'], np.full(rope['links']-1,
                                            (c['length']-c['pin_length'])/(rope['links']-1))]
    nodes = rigid_resample(points, lengths)
    material = np.r_[0., np.cumsum(lengths)]
    rigid_points = np.column_stack([np.interp(s, material, nodes[:, dim]) for dim in range(3)])
    rigid_layout = initial_layout_report(config, nodes, r, p, hole, axis)
    directions = np.diff(nodes, axis=0)/lengths[:, None]
    max_bend = float(np.degrees(np.arccos(np.clip(np.sum(directions[1:]*directions[:-1], axis=1), -1., 1.))).max())
    radius = config['cable']['diameter']/2
    fingers = cad_fingers(0.)
    # Analytic bore identity applies equally to both arms; also audit the full
    # path against both transformed pairs of CAD meshes for accidental detours.
    right_transform = trimesh.transformations.compose_matrix(translate=[0, .6, 0], angles=[0, 0, np.pi/2])
    obstacles = [('left_'+str(i), m) for i, m in enumerate(fingers)]
    for i, mesh in enumerate(fingers):
        mesh = mesh.copy()
        mesh.apply_transform(right_transform)
        obstacles.append(('right_'+str(i), mesh))
    clearances = {}
    def minimum_surface_clearance(mesh, samples):
        # Points outside this expanded AABB are >10 mm from the mesh; they
        # cannot be the minimum when the near set reports a smaller distance.
        near = np.all((samples >= mesh.bounds[0]-.01) & (samples <= mesh.bounds[1]+.01), axis=1)
        selected = samples[near] if np.any(near) else samples
        signed = trimesh.proximity.signed_distance(mesh, selected)
        return float(-signed.max()-radius)

    for name, mesh in obstacles:
        clearances[name] = min(minimum_surface_clearance(mesh, samples)
                               for samples in (points, rigid_points))
    plug = usb_mesh(config).convex_hull
    # The real attachment touches USB by definition; audit free cable beyond
    # the root connection independently of the expected root/USB contact.
    clearances['usb_free_cable'] = min(minimum_surface_clearance(plug, samples[s > c['pin_length']+.001])
                                      for samples in (points, rigid_points))
    axial = np.column_stack([np.linspace(-.012, .012, 97), np.zeros(97), np.full(97, .0024)])
    old = dict(config, usb=dict(config['usb'], tcp_grip_offset=[0., 0., .0075]))
    old_penetration = float(trimesh.proximity.signed_distance(usb_mesh(old).convex_hull, axial).max()+radius)
    hull_contacts = {}
    for q in (.0037, .004, .005):
        hull_contacts[str(q)] = float(max(trimesh.proximity.signed_distance(m.convex_hull, plug.vertices).max()
                                         for m in cad_fingers(q)))
    report = dict(passed=layout['passed'] and rigid_layout['passed'] and min(clearances.values()) >= .00025
                         and max_bend < rope['bend_limit_deg'],
                  scope='creation_geometry_only_not_dynamic_grasp_validation', layout=layout,
                  rigid_chain_layout=rigid_layout, rigid_chain_max_bend_deg=max_bend,
                  sampling_step_m=float(np.diff(s).max()),
                  clearance_sampling_error_bound_m=float(np.diff(s).max()/2),
                  cable_radius_m=radius, minimum_surface_clearance_m=clearances,
                  old_grip_left_bore_cable_overlap_m=old_penetration,
                  finger_hull_usb_vertex_overlap_by_finger_position_m=hull_contacts,
                  finger_cad_tcp_bounds_m=[m.bounds.tolist() for m in fingers],
                  usb_tcp_bounds_m=plug.bounds.tolist())
    return report, points, fingers, plug


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/trunking_cable_simplified_2mm.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report, _, _, _ = audit(load_config(args.config, solver='rope_actor'))
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
