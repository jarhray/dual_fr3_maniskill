"""Legacy imports must share state, while pure USB helpers stay engine-free."""
import importlib
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('old,new', [
    ('assets', 'robot.assets'), ('collision', 'robot.collision'),
    ('simulation', 'robot.simulation'), ('trajectory', 'robot.trajectory'),
    ('ros_bridge', 'ros.bridge'), ('launch_support', 'ros.launch'),
    ('insertion', 'usb.insertion'), ('insertion_geometry', 'usb.geometry'),
    ('insertion_scene', 'usb.scene'), ('insertion_bridge', 'usb.bridge'),
    ('usb_grasp', 'usb.grasp'), ('forces', 'sensing.forces'),
    ('force_output', 'sensing.force_output'), ('perception_camera', 'sensing.camera'),
    ('sapien_compat', 'engine.sapien_compat'), ('warp_setup', 'engine.warp_setup'),
])
def test_legacy_module_is_canonical_module(old, new):
    legacy = importlib.import_module('dual_fr3_maniskill.' + old)
    canonical = importlib.import_module('dual_fr3_maniskill.' + new)
    assert legacy is canonical


def test_source_geometry_and_policies_do_not_import_runtime():
    source = str(Path(__file__).resolve().parents[1])
    code = '''
import importlib.abc
import sys
sys.path.insert(0, sys.argv[1])
class NoRuntime(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname.split('.')[0] in {'rclpy', 'sapien', 'mani_skill2', 'warp', 'torch'}:
            raise AssertionError('Pure helpers imported ' + fullname)
sys.meta_path.insert(0, NoRuntime())
from dual_fr3_maniskill.usb.geometry import hole_half_extents
from dual_fr3_maniskill.insertion_geometry import hole_half_extents as legacy
from dual_fr3_maniskill.usb.insertion import InsertionLimits
from dual_fr3_maniskill.usb.grasp import GraspThresholds
from dual_fr3_maniskill.cable.model import load_config
assert legacy is hole_half_extents
assert InsertionLimits().target_depth_m == .010
assert GraspThresholds().contact_min_force_N == .10
'''
    subprocess.run([sys.executable, '-c', code, source], check=True)


def test_legacy_warp_module_keeps_command_line_entry():
    result = subprocess.run(
        [sys.executable, '-m', 'dual_fr3_maniskill.warp_setup', '--help'],
        check=True, capture_output=True, text=True,
    )
    assert '--cuda-path' in result.stdout
    assert '--force' in result.stdout
