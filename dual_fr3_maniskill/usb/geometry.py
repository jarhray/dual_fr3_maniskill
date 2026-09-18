"""SI geometry for the supplied socket CAD. No ROS, PhysX or cable imports."""
from pathlib import Path
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat

SOCKET_NAME = 'usb_socket'
SOCKET_INSTALLATION_FIXTURES = ('plate', 'trunking')
TIP = np.array([0., .0179, 0.])
# USB +Y goes into CAD -X; USB thickness X maps to socket Y.
USB_IN_SOCKET = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
HOLE = np.array([0., .0175, .0686])
DEPTH = .011
SHOULDER = .0059
# Metal stem thickness/width mapped into socket Y/Z, in metres.
STEM_CROSS_SECTION_M = np.array([.00445, .012])
DEFAULT_CLEARANCE_YZ_M = (.0004, .0004)


def hole_half_extents(clearance=DEFAULT_CLEARANCE_YZ_M):
    """Socket Y/Z half aperture: metal half size plus per-side clearance (m)."""
    return STEM_CROSS_SECTION_M / 2 + np.asarray(clearance)


def adjust_hole_vertices(vertices, clearance=DEFAULT_CLEARANCE_YZ_M, hole=HOLE):
    """Copy CAD vertices, moving only the upper hole walls in socket axes.

    Accept any (..., 3) array. The selection bounds include the original STL's
    rounding tolerance. Classify each wall against the ORIGINAL hole centre,
    even when the configured centre moves. Leave X and other apertures intact.
    Both the MoveIt triangle mesh and PhysX prisms use this calculation.
    """
    adjusted = np.array(vertices, dtype=float, copy=True)
    flat = adjusted.reshape(-1, 3)
    selected = ((.0624599 <= flat[:, 2]) & (flat[:, 2] <= .0746601) &
                (.0149999 <= flat[:, 1]) & (flat[:, 1] <= .0199001))
    half = hole_half_extents(clearance)
    for axis, extent in ((1, half[0]), (2, half[1])):
        flat[selected, axis] = hole[axis] + np.where(
            flat[selected, axis] < HOLE[axis], -extent, extent)
    return adjusted


def matrix(pose):
    """Homogeneous local-to-parent transform from a pose with p and wxyz q."""
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = quat2mat(pose.q), pose.p
    return t


def pose_dict(t, frame="world"):
    return dict(frame=frame, position_m=t[:3, 3].tolist(),
                quaternion_wxyz=mat2quat(t[:3, :3]).tolist())


def measure(base, usb, hole=HOLE):
    local = np.linalg.inv(base) @ usb
    tip = local[:3, 3] + local[:3, :3] @ TIP - hole
    rotation = USB_IN_SOCKET.T @ local[:3, :3]
    angle = float(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1)))
    return dict(tip_in_hole_m=tip.tolist(), depth_m=float(-tip[0]),
                lateral_error_m=float(np.linalg.norm(tip[1:])), orientation_error_rad=angle,
                usb_in_socket=local.tolist())


def tcp_goal(base, tcp, usb, depth, hole=HOLE):
    """World TCP goal using measured grasp; positive depth enters socket -X."""
    target = np.eye(4)
    target[:3, :3] = USB_IN_SOCKET
    target[:3, 3] = np.asarray(hole) + [-depth, 0, 0] - USB_IN_SOCKET @ TIP
    # Use observed grasp, including the rotation acquired after support release.
    return base @ target @ np.linalg.inv(np.linalg.inv(tcp) @ usb)


def socket_parts(mesh_path, clearance=DEFAULT_CLEARANCE_YZ_M, hole=HOLE):
    """Extrude CAD front triangulation into separate convex prisms; solid back.

    Only vertices bounding the upper USB hole move. All other apertures and
    the 11 mm back wall remain in place. No whole-object convex hull or scaling.
    """
    import trimesh
    mesh = trimesh.load(mesh_path, process=False)
    front = mesh.triangles[np.all(abs(mesh.triangles[:, :, 0]) < 1e-7, axis=1)]
    triangles = adjust_hole_vertices(front, clearance, hole)
    parts = []
    for face in triangles:
        back = face.copy()
        back[:, 0] = -DEPTH
        parts.append(trimesh.convex.convex_hull(np.vstack((face, back))))
    back = trimesh.creation.box(extents=[.0748-DEPTH, .03, .0888])
    back.apply_translation([-(.0748+DEPTH)/2, .015, .0444])
    return [back] + parts


def usb_parts(mesh_path):
    """Keep body convex hull; separate metal stem to remove fictitious taper."""
    import trimesh
    v = trimesh.load(mesh_path).vertices
    return [trimesh.convex.convex_hull(v[v[:, 1] <= SHOULDER+1e-7]),
            trimesh.convex.convex_hull(v[(v[:, 1] >= SHOULDER-1e-7) &
                (abs(v[:, 0]) <= .0022251) & (abs(v[:, 2]) <= .0060001)])]


def geometry_report(mesh_dir, clearance=DEFAULT_CLEARANCE_YZ_M, end_clearance=.001):
    import trimesh
    root = Path(mesh_dir)
    plug = trimesh.load(root/'USB1.stl')
    original = trimesh.load(root/'usb_base_collision.stl')
    visual = trimesh.load(root/'usb_base_visual.stl')
    parts = socket_parts(root/'usb_base_collision.stl', clearance)
    return dict(unit='m', visual_bounds=visual.bounds.tolist(), collision_bounds=original.bounds.tolist(),
        usb_bounds=plug.bounds.tolist(), socket_origin='CAD bottom plane z=0; front x=0',
        hole_center_given_m=HOLE.tolist(), cad_hole_center_m=[0., .01745, .06856],
        outward_normal=[1, 0, 0], insertion_axis=[-1, 0, 0],
        original_aperture_m=[.0049, .0122], stem_cross_section_m=STEM_CROSS_SECTION_M.tolist(),
        original_clearance_at_given_center_m=[[.000275, .000175], [.00014, .00006]],
        adjusted_aperture_m=(2*hole_half_extents(clearance)).tolist(),
        adjusted_per_side_clearance_m=list(clearance), cavity_depth_m=DEPTH,
        metal_length_m=float(TIP[1]-SHOULDER), end_clearance_m=end_clearance,
        target_depth_m=DEPTH-end_clearance, shoulder_to_mouth_at_target_m=float(TIP[1]-SHOULDER-DEPTH+end_clearance),
        socket_convex_shape_count=len(parts), usb_convex_shape_count=2,
        note='Geometry evidence only; PhysX import and motion require separate validation')
