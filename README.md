# Dual FR3 ManiSkill backend

This package runs the project's FR3 models in ManiSkill2 0.5.3 / SAPIEN 2.2.2.
It supplies FollowJointTrajectory and GripperCommand actions, measured joint
states, controller states, and a simulation clock. MoveIt and MTC remain the
planning and task execution layers.

See `dual_fr3_moveit_config/docs/maniskill.md` for build, launch and verification.
Install with `bash src/dual_fr3_maniskill/scripts/setup_maniskill2.sh` from the
workspace root. The script preserves the previous `.venv`, creates an isolated
Python 3.10 environment, installs pinned dependencies, and builds ManiSkill2's
matching CUDA Warp fork. Set `MANISKILL_CUDA_PATH` if CUDA 11.8 is elsewhere.
Source ROS Humble and the workspace before using the environment.

The dual-FR3 scene uses CPU rigid-body physics; SAPIEN 2's environment renderer
requires Vulkan even without an interactive window. CUDA is additionally needed
for MPM. `scripts/check_mpm.py` exercises the official `Hang-v0` soft-body task
and checks particle motion, finite deformation and rigid-body coupling.
This MPM check is a separate reference scene; the dual-FR3 task does not yet
contain a calibrated MPM cable.
