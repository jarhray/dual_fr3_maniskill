"""Derive simulation assets from the exact descriptions used by MoveIt."""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation


def origin_matrix(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        transform[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return transform


def pose_values(transform: np.ndarray) -> tuple[list, list]:
    q = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return transform[:3, 3].tolist(), q[[3, 0, 1, 2]].tolist()


@dataclass
class SceneAssets:
    urdf_path: Path
    fixtures: list[tuple[ET.Element, np.ndarray]]
    materials: dict[str, list[float]]
    initial_positions: dict[str, float]
    limits: dict[str, tuple[float, float, float]]
    disabled_collisions: set[frozenset[str]]


def convert_stl_to_glb(source: Path, directory: Path) -> Path:
    """Preserve STL triangle geometry and hard edges in SAPIEN's render mesh."""
    import trimesh

    # Version the conversion recipe as well as the source to avoid reusing a
    # smoothed GLB after upgrading an existing asset cache.
    key = sha256(
        f"flat-normals-v1:{source}:{source.stat().st_mtime_ns}".encode()
    ).hexdigest()[:20]
    converted = directory / f"{source.stem}_{key}.glb"
    if not converted.exists():
        # Default processing welds the STL's per-triangle vertices. Averaging
        # normals across the resulting sharp edges makes the flat plate and
        # channel walls look rounded, even though their positions are correct.
        mesh = trimesh.load_mesh(str(source), process=False)
        mesh.unmerge_vertices()
        mesh.vertex_normals = np.repeat(mesh.face_normals, 3, axis=0)
        mesh.export(str(converted), include_normals=True)
    return converted


def prepare_assets(description: str, semantic: str, directory: Path) -> SceneAssets:
    """Keep robot frames intact; move world-fixed geometry to static actors.

    SAPIEN reads the sibling SRDF for collision exclusions. STL conversion avoids
    loader-specific STL problems, preserves facet normals, and puts generated/
    cooked files in our cache.
    No changes are made to source meshes or the MoveIt description.
    """
    directory.mkdir(parents=True, exist_ok=True)
    robot = ET.fromstring(description)
    srdf = ET.fromstring(semantic)
    for tag in ("ros2_control", "gazebo", "transmission"):
        for element in robot.findall(tag):
            robot.remove(element)

    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://"):
            package, relative = filename[10:].split("/", 1)
            path = Path(get_package_share_directory(package)) / relative
        else:
            path = Path(filename)
        path = path.resolve(strict=True)
        if path.suffix.lower() == ".stl":
            path = convert_stl_to_glb(path, directory)
        mesh.set("filename", str(path))

    links = {link.get("name"): link for link in robot.findall("link")}
    child_links = {joint.find("child").get("link") for joint in robot.findall("joint")}
    roots = set(links) - child_links
    if roots != {"world"}:
        raise ValueError(f"Expected a world-rooted dual FR3 description, got {roots}")
    fixed_poses = {"world": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            if parent in fixed_poses and joint.get("type") == "fixed":
                fixed_poses[joint.find("child").get("link")] = (
                    fixed_poses[parent] @ origin_matrix(joint.find("origin"))
                )
                pending.remove(joint)
                progressed = True
        if not progressed:
            break

    fixtures = []
    # Mounts and robot bases must remain robot links, including their geometry.
    for name in ("worktable", "plate", "trunking"):
        if name not in fixed_poses:
            raise ValueError(f"Fixture {name} must have a fixed transform from world")
        fixtures.append((deepcopy(links[name]), fixed_poses[name]))
        for child in list(links[name]):
            if child.tag in ("visual", "collision"):
                links[name].remove(child)

    initial = {}
    for group in srdf.findall("group_state"):
        if group.get("name") in ("ready", "both_ready"):
            initial.update({j.get("name"): float(j.get("value")) for j in group.findall("joint")})
    limits = {}
    for joint in robot.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        name = joint.get("name")
        limit = joint.find("limit")
        limits[name] = (float(limit.get("lower")), float(limit.get("upper")),
                        float(limit.get("effort")))
        initial.setdefault(name, 0.04 if "finger_joint" in name else 0.0)

    materials = {}
    for material in robot.findall("material"):
        color = material.find("color")
        if color is not None:
            materials[material.get("name")] = np.fromstring(color.get("rgba"), sep=" ").tolist()
    path = directory / "dual_fr3.urdf"
    ET.ElementTree(robot).write(path, encoding="utf-8", xml_declaration=True)
    disabled = {frozenset((pair.get("link1"), pair.get("link2")))
                for pair in srdf.findall("disable_collisions")}
    # Apply the complete SRDF ourselves with an exact clique cover, independent
    # of the SAPIEN loader's interpretation of SRDF reason strings and bit limits.
    for pair in list(srdf.findall("disable_collisions")):
        srdf.remove(pair)
    for joint in robot.findall("joint"):
        disabled.add(frozenset((joint.find("parent").get("link"), joint.find("child").get("link"))))
    ET.ElementTree(srdf).write(path.with_suffix(".srdf"), encoding="utf-8", xml_declaration=True)
    return SceneAssets(path, fixtures, materials, initial, limits, disabled)
