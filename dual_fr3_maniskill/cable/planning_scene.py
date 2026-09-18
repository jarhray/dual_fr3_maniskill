"""Deferred USB geometry shared by MTC preview and the live planning scene.

No ManiSkill/CUDA imports: this also runs in the system ROS Python interpreter.
"""
from pathlib import Path
import struct
import numpy as np

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from shape_msgs.msg import Mesh, MeshTriangle

from dual_fr3_maniskill.cable.model import USB_LINK, load_geometry_config
from dual_fr3_maniskill.cable.threading import LEFT_TCP, TOUCH_LINKS, task_usb_mount


def usb_mesh_path():
    return Path(get_package_share_directory("dual_fr3_maniskill")) / "meshes/USB1.stl"


def _quaternion_xyzw(r):
    # Symmetric extraction is stable at the mount's 180-degree rotation.
    # Keep this ROS-side helper independent of the simulation venv's SciPy ABI.
    matrix = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ])
    _, vectors = np.linalg.eigh(matrix)
    return vectors[:, -1]


def usb_collision_object(config_path, *, orientation_direction=None, preparation_tcp_pose=None):
    config = load_geometry_config(config_path)
    data = usb_mesh_path().read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50*count:
        raise ValueError("USB1.stl must be a binary STL")
    mesh = Mesh()
    scale = config["usb"]["mesh_scale"]
    for offset in range(84, len(data), 50):
        values = struct.unpack_from("<12fH", data, offset)
        start = len(mesh.vertices)
        mesh.vertices.extend(Point(x=values[i]*scale, y=values[i+1]*scale,
                                   z=values[i+2]*scale) for i in (3, 6, 9))
        mesh.triangles.append(MeshTriangle(vertex_indices=[start, start+1, start+2]))
    position, rotation = task_usb_mount(config, orientation_direction=orientation_direction)
    frame = LEFT_TCP
    if preparation_tcp_pose is not None:
        frame = preparation_tcp_pose["frame"]
        p = np.asarray(preparation_tcp_pose["position_m"], dtype=float)
        q = np.asarray(preparation_tcp_pose["quaternion_wxyz"], dtype=float)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all() or np.linalg.norm(q) < 1.e-12:
            raise ValueError("Invalid preparation TCP pose")
        w, x, y, z = q/np.linalg.norm(q)
        r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        position, rotation = p+r@position, r@rotation
    pose = Pose()
    pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
    # Use the same mount transform as SAPIEN, including quaternion ordering.
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(
        float, _quaternion_xyzw(rotation))
    obj = CollisionObject(id=USB_LINK, operation=CollisionObject.ADD,
                          meshes=[mesh], mesh_poses=[pose])
    obj.header.frame_id = frame
    return obj


def attached_usb_scene(config_path, *, orientation_direction=None):
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
        link_name=LEFT_TCP, object=usb_collision_object(config_path,
            orientation_direction=orientation_direction), touch_links=list(TOUCH_LINKS))]
    return scene
