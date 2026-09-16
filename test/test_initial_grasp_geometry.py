"""Actual CAD clearance regressions; these do not assert a dynamic grasp."""
import importlib.util
from pathlib import Path
import pytest

from dual_fr3_maniskill.cable.model import load_config

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("filename", ["trunking_cable.yaml", "trunking_cable_simplified_2mm.yaml"])
def test_original_cad_clears_both_bores_and_usb_after_measured_offset_fix(filename):
    spec = importlib.util.spec_from_file_location('initial_geometry', ROOT/'scripts/check_initial_grasp_geometry.py')
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    report, _, _, _ = geometry.audit(load_config(ROOT/'config'/filename, solver='rope_actor'))
    assert report['passed'], report
    assert report['old_grip_left_bore_cable_overlap_m'] > .003
    for side in ('left_0', 'left_1', 'right_0', 'right_1'):
        assert report['minimum_surface_clearance_m'][side] > .0025
    assert report['minimum_surface_clearance_m']['usb_free_cable'] > .0013
    # The actual physics hull stops the bounded finger drive BEFORE its nominal
    # 3.7 mm target. This is why pose/command completion cannot certify grasp.
    overlaps = report['finger_hull_usb_vertex_overlap_by_finger_position_m']
    assert overlaps['0.004'] > 0.
    assert overlaps['0.005'] < 0.
