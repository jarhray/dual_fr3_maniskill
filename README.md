# Dual FR3 ManiSkill backend

This package owns the dual-FR3 rigid-body simulation and its optional USB/MPM
cable scene. It supplies FollowJointTrajectory and GripperCommand actions,
measured joint/controller states and the simulation clock. MoveIt and MTC remain
in their own packages.

- `simulation.py`, `ros_bridge.py`: existing robot physics and ROS execution.
- `cable/`: cable generation, MPM, axial fibers, rigid contacts and cable ROS feedback.
- `scenes/usb_cable.py`: robot/USB/cable coupling in one simulation loop.
- `scenes/trunking_cable.py`: deferred cable insertion for MTC preparation.
- `scenes/__init__.py`: lightweight scene selection and URDF/SRDF extension.
- `launch/sim.launch.py`: physics-only entry; the caller supplies the final model.
- `config/simulation.yaml`: ordinary robot bridge parameters.
- `config/simulation_usb_cable.yaml`: cable bridge parameters.
- `config/usb_cable.yaml`, `meshes/`: cable material/layout and USB assets.

Full MoveIt/RViz launches live in `dual_fr3_moveit_config`:

```bash
ros2 launch dual_fr3_moveit_config maniskill.launch.py maniskill_scene:=robot
ros2 launch dual_fr3_moveit_config maniskill.launch.py maniskill_scene:=usb_cable
# Convenience entry for the same USB scene:
ros2 launch dual_fr3_moveit_config usb_cable.launch.py
```

Use `cable_config:=/absolute/path/cable.yaml` for USB/cable settings and
`maniskill_config:=/absolute/path/bridge.yaml` for bridge rates/tolerances.
Only one scene and one physics bridge run at a time. The standalone USB scene
retains its fixed USB mount and rejected left-gripper commands.

For MTC, use `ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py
simulation_backend:=maniskill execute:=true`. This selects `trunking_cable`:
the initial model contains only the robot. After both preparation gripper
closures succeed, `/maniskill/cable/spawn` fixes a USB actor to the left TCP and
threads the MPM cable through the right TCP. The right guide constrains lateral
motion and permits axial sliding; the free tail starts straight along the bore.
The left USB tip faces TCP +X, so it follows the task's forward/reverse heading.
Opening either gripper while these ideal constraints are active is rejected.

`config/trunking_cable.yaml` defaults to a 2 mm cable and 1 mm particle spacing.
The existing research-finger CAD bore has about 1.27 mm minimum radial clearance
at the closed TCP; the standalone 3.5 mm cable does not fit. Cable contacts query
the original finger triangles, preserving the hole that PhysX's convex hulls
would fill. No finger meshes or global collision exclusions are changed.
`maniskill_cable:=false` disables this MTC scene. Other backends do not insert it.
See [MTC integration](docs/mtc_cable.md) for the stage and interface details.

See [USB cable setup and validation](docs/usb_cable.md) and
`dual_fr3_moveit_config/docs/maniskill.md` for full build and usage instructions.
The former `dual_fr3_usb_cable_demo` package has been merged here; use the new
launch commands above. Existing `/usb_cable_demo/*` ROS interfaces are preserved.

Install with `bash src/dual_fr3_maniskill/scripts/setup_maniskill2.sh` from the
workspace root. This creates an isolated Python 3.10 environment with pinned
ManiSkill2 0.5.3 / SAPIEN 2.2.2 dependencies and the matching CUDA Warp fork.
Set `MANISKILL_CUDA_PATH` if CUDA 11.8 is elsewhere. Source ROS Humble and the
workspace before using the environment.

The robot scene uses CPU rigid-body physics and requires Vulkan for SAPIEN's
renderer even without an interactive window. CUDA is additionally needed for
the cable scene. Model construction and launch selection do not import the
physics libraries. `scripts/check_mpm.py` remains the separate official `Hang-v0`
reference test; it does not validate the project's cable material calibration.
