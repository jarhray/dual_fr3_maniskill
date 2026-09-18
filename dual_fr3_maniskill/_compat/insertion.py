"""Compatibility alias for :mod:`dual_fr3_maniskill.usb.insertion`."""
from importlib import import_module
import sys

sys.modules[__name__] = import_module("dual_fr3_maniskill.usb.insertion")
