"""Importing this package does not load ManiSkill or require a GPU."""

# Keep legacy modules importable without loading ROS, CUDA or PhysX.
from pathlib import Path as _Path

__path__.append(str(_Path(__file__).parent / "_compat"))
