#!/usr/bin/env python3
"""Reproducible CAD measurements; this is not a simulation success claim."""
import argparse
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from dual_fr3_maniskill.insertion_geometry import geometry_report

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = geometry_report(Path(get_package_share_directory('dual_fr3_maniskill'))/'meshes')
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
