"""CLI reporting checks for the renderer-free, reduced finger experiment."""
import json
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize("case,collision_geometry", [
    ("rope_finger_rim_20260915", "capsule"),
    ("rope_finger_rim_20260915", "convex_capsule"),
    ("rope_finger_exit_20260915", "convex_capsule"),
    ("rope_trunking_rim_20260915", "convex_capsule"),
])
def test_recorded_rim_overlap_generates_contact_with_correct_depth(tmp_path, case, collision_geometry):
    """The old 20 µm shells missed the deepest rim contact on this pose.

    Use production finger proxies and capsules, then remove the chain joints.
    Compare native pre-solve separation with the starting mesh depth, before
    gravity or friction response can alter the geometry under test.
    """
    import importlib.util
    import numpy as np
    import trimesh
    from transforms3d.quaternions import quat2mat
    from dual_fr3_maniskill.cable.model import load_config
    from dual_fr3_maniskill.sapien_compat import sapien

    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location("finger_probe", root/"scripts/check_rope_finger_contact.py")
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    recorded = json.loads((root/"test/data"/(case+".json")).read_text())
    # Frozen poses/depths were captured with the successful 2 mm baseline.
    baseline = root/"config/trunking_cable_simplified_2mm.yaml"
    config = load_config(baseline, solver="rope_actor")
    config["rope_actor"]["collision_geometry"] = collision_geometry
    config["rope_actor"]["links"] = 13
    config["cable"]["length"] = config["cable"]["pin_length"]+12*recorded["segment_length_m"]
    engine, scene, cable, _, sources, _, _ = probe.create_scene(config, tmp_path, coupling="prescribed")
    try:
        for drive in [cable.anchor, *cable.joints]:
            scene.remove_drive(drive)
        cable.anchor = None
        cable.joints.clear()
        for actor in cable.links:
            actor.set_pose(sapien.Pose([5., 0., 0.]))
            actor.set_velocity([0., 0., 0.])
            actor.set_angular_velocity([0., 0., 0.])
        actor = cable.links[1]
        state = recorded["capsule"]
        actor.set_pose(sapien.Pose(state["position_m"], state["quaternion_wxyz"]))
        finger = recorded.get("finger")
        for proxy, source in cable.proxies:
            pose = (sapien.Pose(finger["position_m"], finger["quaternion_wxyz"])
                    if finger and source.name == finger["name"] else sapien.Pose([5., 0., 0.]))
            source.set_pose(pose)
            proxy.set_pose(pose)
            if finger and source.name == finger["name"]:
                target = proxy
        if "fixture" in recorded:
            from types import SimpleNamespace
            from dual_fr3_maniskill.assets import prepare_assets
            from dual_fr3_maniskill.simulation import DualFR3Env
            from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
            description, semantic = build_maniskill_description(scene="trunking_cable", cable_config=baseline)
            assets = prepare_assets(description, semantic, tmp_path/"fixtures")
            for element, _ in assets.fixtures:
                for visual in list(element.findall("visual")):
                    element.remove(visual)
            env = SimpleNamespace(_scene=scene, _renderer=None, assets=assets)
            DualFR3Env._load_actors(env)
            target = env.fixtures[recorded["fixture"]]
            np.testing.assert_allclose(target.pose.p, recorded["fixture_pose"]["position_m"], atol=1.e-7)
            for shape in target.get_collision_shapes():
                shape.contact_offset = config["rope_actor"]["contact_offset"]
                groups = shape.get_collision_groups()
                shape.set_collision_groups(groups[0] | 6, *groups[1:])
        samples = actor.pose.p+np.linspace(-cable.lengths[1]/2, cable.lengths[1]/2, 501)[:, None]*quat2mat(actor.pose.q)[:, 0]
        depths = []
        for shape in target.get_collision_shapes():
            g = shape.geometry
            mesh = trimesh.Trimesh(vertices=np.asarray(g.vertices)*g.scale,
                                  faces=np.asarray(g.indices).reshape(-1, 3), process=False)
            pose = target.pose*shape.get_local_pose()
            depths.append(float((trimesh.proximity.signed_distance(mesh, (samples-pose.p)@quat2mat(pose.q))+cable.radius).max()))
        geometric_depth = max(depths)
        assert abs(geometric_depth-recorded.get("expected_depth_m", .00011958)) < .000003
        for body in (actor, target):
            assert all(s.rest_offset == 0. for s in body.get_collision_shapes())
        scene.set_timestep(.0000625)
        scene.step()
        points = [p for c in scene.get_contacts()
                  if {c.actor0.id, c.actor1.id} == {actor.id, target.id} for p in c.points]
        assert points, "Native solver missed the recorded rim contact"
        native_depth = max(-p.separation for p in points)
        assert abs(native_depth-geometric_depth) < .00002
    finally:
        cable.close()


def test_no_contact_run_is_rejected_and_still_exports_substeps(tmp_path):
    pytest.importorskip("sapien.core")
    pytest.importorskip("dual_fr3_maniskill._rope_physx")
    root = Path(__file__).resolve().parents[1]
    output = tmp_path/"no_contact.json"
    command = [sys.executable, str(root/"scripts/check_rope_finger_contact.py"),
               "--cable-config", str(root/"config/trunking_cable.yaml"),
               "--duration", ".0002", "--settle", "0", "--hold", "0", "--distance", "0",
               "--audit-every", "1", "--output", str(output)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=60)
    assert result.returncode == 1, result.stdout+result.stderr
    report = json.loads(output.read_text())
    assert not report["passed"]
    assert "never contacted" in report["failure"]
    assert report["summary"]["substeps"] == 2
    assert report["summary"]["contact_substeps"] == 0
    assert report["summary"]["geometry_audits"] == 2
    assert len(report["collision_meshes"]) == 2
    assert len(report["tail"][-1]["fingers"]) == 2
    rows = [json.loads(line) for line in output.with_suffix(".jsonl").read_text().splitlines()]
    assert len(rows) == 2 and rows[-1]["geometry_audited"]
    before = output.read_bytes()
    repeated = subprocess.run(command, capture_output=True, text=True, timeout=60)
    assert repeated.returncode == 2 and "never overwritten" in repeated.stderr
    assert output.read_bytes() == before
