"""Compatibility alias for :mod:`dual_fr3_maniskill.engine.warp_setup`."""
from importlib import import_module
import sys

_module = import_module("dual_fr3_maniskill.engine.warp_setup")
sys.modules[__name__] = _module

if __name__ == "__main__":
    _module.main()
