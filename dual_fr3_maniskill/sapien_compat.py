"""Import SAPIEN without overriding the Vulkan loader's driver discovery.

SAPIEN 2.2 synthesizes VK_ICD_FILENAMES from /usr/share/vulkan/icd.d at
import time. That hides drivers in /etc, XDG directories, and additional
driver paths. Restore the caller's setting before any renderer is created,
so the system loader keeps its normal search and environment priorities.

Physics modules must import SAPIEN through this module before importing
ManiSkill, which also imports SAPIEN. Keep this out of model/launch imports.
"""
import os


_original_icd = os.environ.get("VK_ICD_FILENAMES")
try:
    import sapien.core as sapien
finally:
    if _original_icd is None:
        os.environ.pop("VK_ICD_FILENAMES", None)
    else:
        os.environ["VK_ICD_FILENAMES"] = _original_icd
