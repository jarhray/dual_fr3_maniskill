"""Check fresh-process physics imports against real SAPIEN import side effects."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("sapien") is None or
    importlib.util.find_spec("mani_skill2") is None,
    reason="requires the optional ManiSkill2/SAPIEN Python environment",
)


@pytest.mark.parametrize("module", [
    "dual_fr3_maniskill.simulation",
    "dual_fr3_maniskill.scenes.usb_cable",
    "dual_fr3_maniskill.scenes.trunking_cable",
])
@pytest.mark.parametrize("overrides", [
    {},
    {"VK_ICD_FILENAMES": ""},
    {"VK_ICD_FILENAMES": "/custom/first.json:/custom/second.json"},
    {"VK_DRIVER_FILES": "/custom/drivers.json"},
    {"VK_DRIVER_FILES": "/custom/new.json", "VK_ICD_FILENAMES": "/custom/old.json"},
    {"VK_ADD_DRIVER_FILES": "/custom/extra.json", "XDG_DATA_DIRS": "/custom/data"},
])
def test_physics_import_preserves_driver_configuration(module, overrides):
    env = dict(os.environ)
    for name in ("VK_ICD_FILENAMES", "VK_DRIVER_FILES", "VK_ADD_DRIVER_FILES"):
        env.pop(name, None)
    env.update(overrides)
    source = str(Path(__file__).resolve().parents[1])
    env["PYTHONPATH"] = os.pathsep.join(filter(None, (source, env.get("PYTHONPATH"))))
    script = """
import importlib
import json
import os
import sys
keys = ('VK_ICD_FILENAMES', 'VK_DRIVER_FILES', 'VK_ADD_DRIVER_FILES', 'XDG_DATA_DIRS')
before = {name: os.environ.get(name) for name in keys}
importlib.import_module(sys.argv[1])
after = {name: os.environ.get(name) for name in keys}
assert after == before, (before, after)
assert 'sapien.core' in sys.modules
print(json.dumps(after))
"""
    result = subprocess.run([sys.executable, "-c", script, module], env=env,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    after = json.loads(result.stdout.splitlines()[-1])
    assert after["VK_ICD_FILENAMES"] == overrides.get("VK_ICD_FILENAMES")
