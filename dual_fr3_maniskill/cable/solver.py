"""Drive the original ManiSkill2 MPM kernels with a padded moving grid.

The upstream bound uses only four cells of lower padding, allowing its
three-cell wall projection to touch a thin, freely falling cable. Keep that
numerical wall outside all particle interpolation stencils.
"""
import warp as wp
from mpm.mpm_integrator import (
    MPMModelStruct, MPMStateStruct, zero_everything, p2g, grid_op_with_contact, g2p)


@wp.kernel
def padded_bounds(model: MPMModelStruct, state: MPMStateStruct, padding: int):
    p = state.particle_q[wp.tid()]
    for axis in range(3):
        wp.atomic_min(state.grid_lower, axis, int(wp.floor(p[axis] * model.inv_dx)) - padding)
        wp.atomic_max(state.grid_upper, axis, int(wp.ceil(p[axis] * model.inv_dx)) + padding)


class CableMPMSimulator:
    def __init__(self, *, device, padding, anchor_grid):
        self.device, self.padding, self.anchor_grid = device, padding, anchor_grid

    def simulate(self, model, state_in, state_out, dt):
        m, s = model.struct, state_in.struct
        cells = m.grid_dim_x * m.grid_dim_y * m.grid_dim_z

        def launch(kernel, dim, inputs):
            wp.launch(kernel, dim=int(dim), inputs=inputs, device=self.device)

        launch(zero_everything, max(cells, m.n_particles, model.body_count), [
            s, state_in.ext_body_f, state_in.int_body_f, state_in.mpm_contact_count,
            m.grid_dim_x, m.grid_dim_y, m.grid_dim_z, m.n_particles, model.body_count])
        launch(padded_bounds, m.n_particles, [m, s, self.padding])
        launch(p2g, m.n_particles, [m, s, state_out.struct, model.gravity, dt])
        launch(grid_op_with_contact, cells, [
            m, s, dt, state_in.body_q, state_in.body_qd, model.body_com, model.shape_transform,
            model.shape_body, model.shape_geo_type, model.shape_geo_id, model.shape_geo_scale,
            # Rigid contact is solved on the finite-radius cable after G2P
            # and every fiber iteration. Avoid double-counting grid impulses.
            0, state_in.ext_body_f])
        self.anchor_grid(state_in)
        launch(g2p, m.n_particles, [m, s, state_out.struct, dt])
