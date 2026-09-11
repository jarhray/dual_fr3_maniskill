"""Original research-finger triangles, preserving the closed-gripper bore."""
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

from ..assets import origin_matrix


def closed_oriented_shells(mesh):
    # The CAD finger consists of closed shells sharing ten edges. Such edges
    # have four incident faces, so trimesh.is_watertight is false. Require
    # balanced oriented edges (no open boundaries), consistent winding and
    # positive volume; retain the source triangles without filling any holes.
    signs = np.where(mesh.edges[:, 0] < mesh.edges[:, 1], 1., -1.)
    balance = np.bincount(mesh.edges_unique_inverse, weights=signs)
    return bool(np.all(balance == 0) and mesh.is_winding_consistent and mesh.volume > 0)


def finger_collision_meshes(urdf_path):
    robot = ET.parse(urdf_path).getroot()
    result = {}
    for side in ("left", "right"):
        for finger in ("left", "right"):
            name = f"{side}_fr3_{finger}finger"
            elements = robot.findall(f"link[@name='{name}']/collision")
            result[name] = []
            if not elements:
                raise ValueError(f"Missing collision geometry: {name}")
            for element in elements:
                mesh = element.find("geometry/mesh")
                if mesh is None:
                    raise ValueError("MTC cable requires research finger collision meshes")
                triangles = trimesh.load(mesh.get("filename"), force="mesh")
                scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                transform = origin_matrix(element.find("origin"))
                vertices = np.asarray(triangles.vertices)*scale
                vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
                result[name].append((vertices, np.asarray(triangles.faces).reshape(-1)))
    return result
