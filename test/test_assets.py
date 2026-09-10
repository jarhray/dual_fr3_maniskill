"""STL conversion must retain sharp edges and the original collision surface."""
from pathlib import Path

import numpy as np
import pytest
import trimesh

from dual_fr3_maniskill.assets import convert_stl_to_glb


MESHES = Path(__file__).resolve().parents[2] / "dual_fr3_moveit_config" / "meshes"


@pytest.mark.parametrize("filename", ["base_link.STL", "Plate.STL", "Trunking.STL"])
def test_converted_fixture_keeps_triangles_and_flat_normals(tmp_path, filename):
    source = trimesh.load_mesh(MESHES / filename, process=False)
    converted_path = convert_stl_to_glb(MESHES / filename, tmp_path)
    # Trimesh 3 returns a Scene for GLB. Inspect its single stored mesh directly:
    # force="mesh" concatenation discards the exported normals and recomputes
    # them, which would test the loader rather than our actual render asset.
    scene = trimesh.load(converted_path, process=False)
    assert len(scene.geometry) == 1
    converted = next(iter(scene.geometry.values()))

    np.testing.assert_array_equal(converted.triangles, source.triangles)
    np.testing.assert_allclose(
        converted.vertex_normals[converted.faces],
        np.repeat(source.face_normals[:, None, :], 3, axis=1),
        atol=1e-6,
    )
    # A second load can reuse the generated asset without rewriting it.
    mtime = converted_path.stat().st_mtime_ns
    assert convert_stl_to_glb(MESHES / filename, tmp_path) == converted_path
    assert converted_path.stat().st_mtime_ns == mtime
