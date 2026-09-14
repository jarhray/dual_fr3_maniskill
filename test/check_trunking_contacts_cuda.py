#!/usr/bin/env python3
"""Moving-arm/reset collision comparison, including an unchanged-reference repeat."""
import argparse
from reference import CONTACTS_REFERENCE, MPM_REFERENCE
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from dual_fr3_maniskill.cable import mpm_cable
from dual_fr3_maniskill.cable.guide import SlidingGuide
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
from check_trunking_guide_cuda import run, validate_comparison


def load_reference(name, path):
    spec = importlib.util.spec_from_file_location('dual_fr3_maniskill.cable.' + name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=CONTACTS_REFERENCE,
                        help='Collision reference source (default: bundled frozen baseline)')
    parser.add_argument('--reference-mpm', type=Path, default=MPM_REFERENCE,
                        help='MPM lifecycle reference (default: bundled frozen baseline)')
    parser.add_argument('--steps', type=int, default=20)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.steps < 10:
        parser.error('steps must be at least 10')
    args.output.mkdir(parents=True, exist_ok=False)
    old_contacts = load_reference('_motion_contact_reference', args.reference)
    old_mpm = load_reference('_motion_mpm_reference', args.reference_mpm)
    old_mpm.CableContacts = old_contacts.CableContacts
    share = Path(get_package_share_directory('dual_fr3_maniskill'))
    config_path = share / 'config/trunking_cable.yaml'
    config = load_config(config_path)
    # Isolate collision math from graph scheduling in this reference test.
    config['mpm']['cuda_graph'] = False
    frequencies = yaml.safe_load((share / 'config/simulation_usb_cable.yaml').read_text())['dual_fr3_maniskill']['ros__parameters']
    joints = json.loads((share / 'config/profiling_trunking_pose.json').read_text())
    for side, width in [('left', config['usb']['finger_position']), ('right', 0.)]:
        joints.update({f'{side}_fr3_finger_joint{i}': width for i in (1, 2)})
    description, semantic = build_maniskill_description(scene='trunking_cable', cable_config=config_path)
    report = dict(steps=args.steps, post_reset_steps=5, cable_config=config,
                  simulation_config=frequencies, reference_sources={str(path.resolve()):
                  hashlib.sha256(path.read_bytes()).hexdigest() for path in (args.reference, args.reference_mpm)})
    with patch.object(mpm_cable, 'MPMCable', old_mpm.MPMCable):
        report['reference'], reference = run(SlidingGuide, config, frequencies, joints, description,
                                             semantic, args.steps, args.output, label='reference')
        report['reference_repeat'], repeated = run(SlidingGuide, config, frequencies, joints, description,
                                                   semantic, args.steps, args.output, label='reference_repeat')
    report['accelerated'], actual = run(SlidingGuide, config, frequencies, joints, description,
                                       semantic, args.steps, args.output, label='accelerated')
    validate_comparison(report, reference, repeated, actual)
    report['state_max_absolute_difference'] = {key: float(np.max(np.abs(actual[key]-reference[key]))) for key in actual}
    (args.output / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
    assert report['passed'], report['comparison']
    print('PASS: moving arms and cable reset with original versus accelerated contacts')
    print(json.dumps(report['comparison'], indent=2))


if __name__ == '__main__':
    main()
