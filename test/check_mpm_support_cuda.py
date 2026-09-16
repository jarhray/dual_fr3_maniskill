#!/usr/bin/env python3
"""Real CUDA test of temporary MPM support and graph retirement, no renderer.

Uses the production support kernel/cache and GPU arrays. This is a support
lifecycle experiment, not a full MPM cable or robot grasp simulation.
"""
import json
from types import SimpleNamespace

import numpy as np

from dual_fr3_maniskill.cable.mpm_cable import MPMCable, initialize_warp
from dual_fr3_maniskill.cable.execution import ConstraintGraphs
import warp as wp


@wp.kernel
def advance_free(positions: wp.array(dtype=wp.vec3), velocities: wp.array(dtype=wp.vec3), dt: float):
    i = wp.tid()
    positions[i] = positions[i]+velocities[i]*dt


def main():
    initialize_warp()
    cable = MPMCable.__new__(MPMCable)
    cable.config = {"cable": {"length": 1.}}
    cable.sections, cable.device = 101, "cuda"
    count = cable.sections*7
    initial = np.zeros((count, 3), dtype=np.float32)
    initial[:, 0] = np.repeat(np.linspace(0., 1., cable.sections), 7)
    initial[:, 2] = 1.
    cable.states = [SimpleNamespace(struct=SimpleNamespace(
        particle_q=wp.array(initial, dtype=wp.vec3, device="cuda"),
        particle_qd=wp.array(np.ones_like(initial)*.1, dtype=wp.vec3, device="cuda")))
        for _ in range(2)]
    cable.constraint_graphs = ConstraintGraphs(True)
    cable.support_count = 0
    cable.support_ids = cable.support_targets = None
    removed = []
    cable.env = SimpleNamespace(_scene=SimpleNamespace(remove_particle_entity=removed.append))
    cable.pcd = object()

    def execute():
        state = cable.states[0].struct

        def record():
            wp.launch(advance_free, dim=count, inputs=[state.particle_q, state.particle_qd, .01], device="cuda")
            cable._apply_temporary_supports()

        cable.constraint_graphs.run((cable.support_count,), state.particle_q.ptr, record)

    try:
        cable.add_temporary_supports([[.20, .24], [.60, .64]])
        assert 0 < cable.support_count < count*.15
        ids = cable.support_ids.numpy()
        supported = cable.support_count
        for _ in range(4):
            execute()
            state = cable.states[0].struct
            np.testing.assert_array_equal(state.particle_q.numpy()[ids], initial[ids])
            np.testing.assert_array_equal(state.particle_qd.numpy()[ids], np.zeros((len(ids), 3)))
            cable.states.reverse()
        assert len(cable.constraint_graphs.graphs) == 2
        assert cable.constraint_graphs.captures == 2 and cable.constraint_graphs.replays == 4
        try:
            cable.reset()
        except RuntimeError as exc:
            assert "scene reset" in str(exc)
        else:
            raise AssertionError("Reset with live world supports must not retain stale targets")
        before = [(s.struct.particle_q.numpy(), s.struct.particle_qd.numpy()) for s in cable.states]
        cable.remove_temporary_supports()
        assert cable.support_count == 0
        assert cable.support_ids is None and cable.support_targets is None
        assert not cable.constraint_graphs.graphs
        for state, (positions, velocities) in zip(cable.states, before):
            np.testing.assert_array_equal(state.struct.particle_q.numpy(), positions)
            np.testing.assert_array_equal(state.struct.particle_qd.numpy(), velocities)
        # With support retired, the old supported particles must now move and
        # preserve nonzero velocity on both rebuilt/replayed graphs.
        for _ in range(4):
            state = cable.states[0].struct
            moved = state.particle_q.numpy()+.005
            speed = np.ones_like(initial)*.1
            state.particle_q.assign(moved)
            state.particle_qd.assign(speed)
            execute()
            np.testing.assert_allclose(state.particle_q.numpy()[ids], (moved+.001)[ids], atol=1.e-7)
            np.testing.assert_array_equal(state.particle_qd.numpy()[ids], speed[ids])
            cable.states.reverse()
        assert cable.constraint_graphs.captures == 4
        # Equal-count replacement also retires old raw pointers before reuse.
        cable.add_temporary_supports([[.30, .34], [.70, .74]])
        assert not cable.constraint_graphs.graphs
        execute()
        cable.close()
        cable.close()
        assert len(removed) == 1 and not hasattr(cable, "pcd")
        assert not cable.constraint_graphs.graphs and cable.support_count == 0
        print(json.dumps(dict(passed=True, device="cuda", total_particles=count,
            temporary_supported_particles=supported, supported_graphs=2,
            graphs_recaptured_after_release=2, position_velocity_preserved_at_release=True,
            no_support_writes_after_release=True, particle_render_entity_removed_once=True), indent=2))
    finally:
        cable.close()


if __name__ == "__main__":
    main()
