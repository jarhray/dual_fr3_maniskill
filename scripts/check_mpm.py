#!/usr/bin/env python3
"""Exercise ManiSkill2's actual CUDA MPM, rendering and rigid-body coupling."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import time

# Import before ManiSkill to preserve the system Vulkan driver discovery.
from dual_fr3_maniskill.engine.sapien_compat import sapien  # noqa: F401

import gymnasium as gym
import numpy as np

# Importing the MPM base adds ManiSkill2's matching Warp fork to sys.path.
from mani_skill2.envs.mpm.base_env import MPMBaseEnv
import mani_skill2.envs
import warp as wp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--render-output")
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")
    wp.init()
    if not wp.is_cuda_available():
        raise RuntimeError("ManiSkill2 MPM needs a visible NVIDIA GPU and its compiled CUDA Warp fork")
    env = gym.make("Hang-v0", obs_mode="none", control_mode="pd_joint_delta_pos",
                   render_mode="human" if args.viewer else "rgb_array",
                   renderer_kwargs={"offscreen_only": not args.viewer})
    try:
        env.reset(seed=0)
        physics = env.unwrapped
        if not isinstance(physics, MPMBaseEnv):
            raise RuntimeError("Expected ManiSkill2's MPM environment")
        before = physics.get_mpm_state()["x"].copy()
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        start = time.monotonic()
        for _ in range(args.steps):
            _, _, _, _, info = env.step(action)
            if physics.sim_crashed or info.get("crashed", False):
                raise RuntimeError("MPM solver reported a numerical failure")
            if args.viewer:
                env.render()
        state = physics.get_mpm_state()
        if not all(np.isfinite(value).all() for value in state.values()):
            raise RuntimeError("MPM returned a non-finite particle state")
        displacement = float(np.max(np.linalg.norm(state["x"] - before, axis=1)))
        if displacement <= 1e-7:
            raise RuntimeError("MPM particles did not advance")
        if not physics._coupled_actors:
            raise RuntimeError("No rigid bodies were coupled to the MPM solver")
        print(json.dumps({
            "mani_skill2": importlib.metadata.version("mani-skill2"),
            "sapien": importlib.metadata.version("sapien"),
            "warp_module": wp.__file__, "environment": "Hang-v0",
            "particles": len(before), "coupled_actors": len(physics._coupled_actors),
            "control_steps": args.steps, "wall_seconds": time.monotonic() - start,
            "max_particle_displacement_m": displacement,
            "max_deformation_from_identity": float(np.max(np.abs(state["F"] - np.eye(3)))),
        }, indent=2), flush=True)
        if args.render_output:
            from PIL import Image
            output = Path(args.render_output)
            output.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(physics.render_rgb_array()).save(output)
    finally:
        env.close()


if __name__ == "__main__":
    main()
