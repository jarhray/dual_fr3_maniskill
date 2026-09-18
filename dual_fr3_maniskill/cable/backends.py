"""Cable selection without importing a physics runtime on the launch path."""

CABLE_SOLVERS = ("mpm", "rope_actor")


def validate_solver(solver):
    if solver not in CABLE_SOLVERS:
        raise ValueError(f"Unknown cable_solver {solver!r}; choose one of {CABLE_SOLVERS}")
    return solver


def prepare_backend(solver, config, sim_freq):
    validate_solver(solver)
    if solver == "mpm":
        if config["mpm"]["frequency"] % sim_freq:
            raise ValueError("mpm.frequency must be divisible by sim_freq")
    elif config["rope_actor"]["frequency"] % sim_freq:
        raise ValueError("rope_actor.frequency must be divisible by sim_freq")


def shader_directory(solver):
    validate_solver(solver)
    if solver == "mpm":
        from pathlib import Path
        import mani_skill2
        from dual_fr3_maniskill.cable.mpm_cable import initialize_warp
        initialize_warp()
        return str(Path(mani_skill2.__file__).parent / "envs/mpm/shader/point")
    return "ibl"


def create_cable(env, config, *, solver="mpm", **kwargs):
    validate_solver(solver)
    if solver == "mpm":
        from dual_fr3_maniskill.cable.mpm_cable import MPMCable
        return MPMCable(env, config, **kwargs)
    from dual_fr3_maniskill.cable.rope_actor import RopeActorCable
    return RopeActorCable(env, config, **kwargs)
