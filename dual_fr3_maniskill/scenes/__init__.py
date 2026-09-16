"""Scene selection and model extensions; importing this module needs no GPU."""
from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


SCENES = ("robot", "usb_cable", "trunking_cable")


@dataclass(frozen=True)
class SceneSpec:
    bridge_executable: str
    simulation_config: str
    rviz_config: str | None = None


def scene_spec(scene: str) -> SceneSpec:
    share = Path(get_package_share_directory("dual_fr3_maniskill"))
    if scene == "robot":
        return SceneSpec("maniskill_bridge.py", str(share / "config/simulation.yaml"))
    if scene == "usb_cable":
        return SceneSpec("usb_cable_bridge.py", str(share / "config/simulation_usb_cable.yaml"),
                         str(share / "config/usb_cable.rviz"))
    if scene == "trunking_cable":
        return SceneSpec("trunking_cable_bridge.py", str(share / "config/simulation_usb_cable.yaml"))
    raise ValueError(f"Unknown ManiSkill scene {scene!r}; choose one of {SCENES}")


def resolve_cable_config(path: str | Path | None = None, *, scene="usb_cable") -> str:
    candidate = (Path(path).expanduser() if path else
                 Path(get_package_share_directory("dual_fr3_maniskill")) / "config" /
                 ("trunking_cable_simplified_2mm.yaml" if scene == "trunking_cable" else "usb_cable.yaml"))
    return str(candidate.resolve(strict=True))


def extend_scene_description(description: str, semantic: str, *, scene: str,
                             cable_config: str | Path | None = None) -> tuple[str, str]:
    """Extend caller-owned URDF/SRDF without depending on a MoveIt package."""
    scene_spec(scene)
    if scene == "robot":
        return description, semantic
    from ..cable.model import add_usb_description, load_geometry_config

    if scene == "trunking_cable":
        import xml.etree.ElementTree as ET
        config = load_geometry_config(resolve_cable_config(cable_config, scene=scene))
        variant = config["scene"].get("trunking_mesh")
        variants = {"visual": config["scene"].get("trunking_visual_mesh", variant),
                    "collision": variant}
        if not any(variants.values()):
            return description, semantic
        robot = ET.fromstring(description)
        trunking = robot.find("link[@name='trunking']")
        if trunking is None:
            raise ValueError("Trunking mesh selection requires a trunking link")
        meshes = {
            "original": ("Trunking.STL", "0 0 0"),
            "simplified": ("Trunking_simplify.stl", "0.00014546 -0.00068397 0"),
        }
        for kind, variant in variants.items():
            if variant is None:
                continue
            # Each CAD export has its own origin; visual overrides must not
            # change the collision mesh or inherit its alignment offset.
            filename, origin = meshes[variant]
            shapes = trunking.findall(kind)
            if len(shapes) != 1 or shapes[0].find("geometry/mesh") is None:
                raise ValueError(f"Expected one trunking {kind} mesh")
            shape = shapes[0]
            pose = shape.find("origin")
            if pose is None:
                pose = ET.SubElement(shape, "origin")
            pose.set("xyz", origin)
            pose.set("rpy", "0 0 0")
            shape.find("geometry/mesh").set("filename",
                "package://dual_fr3_moveit_config/meshes/" + filename)
        return ET.tostring(robot, encoding="unicode"), semantic

    # USB is a dynamic runtime actor. Never add a fixed URDF joint or a TF
    # derived from the TCP; the bridge publishes the measured actor pose.
    return description, semantic
