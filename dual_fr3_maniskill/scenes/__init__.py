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
                 ("trunking_cable.yaml" if scene == "trunking_cable" else "usb_cable.yaml"))
    return str(candidate.resolve(strict=True))


def extend_scene_description(description: str, semantic: str, *, scene: str,
                             cable_config: str | Path | None = None) -> tuple[str, str]:
    """Extend caller-owned URDF/SRDF without depending on a MoveIt package."""
    scene_spec(scene)
    if scene in ("robot", "trunking_cable"):
        return description, semantic
    from ..cable.model import add_usb_description, load_config

    share = Path(get_package_share_directory("dual_fr3_maniskill"))
    return add_usb_description(description, semantic, share / "meshes/USB1.stl",
                               load_config(resolve_cable_config(cable_config)),
                               mesh_uri="package://dual_fr3_maniskill/meshes/USB1.stl")
