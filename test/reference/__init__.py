"""Frozen test inputs; production physics must never import this package."""
from pathlib import Path

CONTACTS_REFERENCE = Path(__file__).with_name('contacts_before_clearance.py')
MPM_REFERENCE = Path(__file__).with_name('mpm_before_clearance.py')
