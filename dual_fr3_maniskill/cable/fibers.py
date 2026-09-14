"""Axial XPBD fibers between MPM material points; no rigid links or joints.

Volumetric MPM alone loses tensile connectivity for a cable thinner than its
grid. Seven longitudinal fibers supply explicit axial stiffness while MPM
still handles volume, deformation gradient and momentum. Rigid contact is
interleaved with these position constraints by the supplied callback.
"""
import numpy as np
import warp as wp


@wp.kernel
def solve_fibers(position: wp.array(dtype=wp.vec3), velocity: wp.array(dtype=wp.vec3),
    mass: wp.array(dtype=float), rest: wp.array(dtype=float), multipliers: wp.array(dtype=float),
    pin_count: int, edge_count: int, color: int, area: float, young: float, dt: float,
    com: wp.array(dtype=wp.vec3), impulse: wp.array(dtype=wp.spatial_vector)):
    index = wp.tid()
    # Two-color Gauss-Seidel: neighboring edges never write the same point.
    edge = (index / 7 * 2 + color) * 7 + index % 7
    if edge < edge_count:
        a = edge
        b = a + 7
        wa = float(0.0)
        wb = float(0.0)
        if a >= pin_count:
            wa = 1.0 / mass[a]
        if b >= pin_count:
            wb = 1.0 / mass[b]
        delta = position[b] - position[a]
        distance = wp.length(delta)
        if wa + wb > 0.0 and distance > 1e-8:
            normal = delta / distance
            alpha = rest[edge] / (young * area * dt * dt)
            dl = (rest[edge] - distance - alpha*multipliers[edge]) / (wa + wb + alpha)
            multipliers[edge] = multipliers[edge] + dl
            da = normal * (-wa * dl)
            db = normal * (wb * dl)
            position[a] = position[a] + da
            position[b] = position[b] + db
            velocity[a] = velocity[a] + da / dt
            velocity[b] = velocity[b] + db / dt
            if a < pin_count:
                reaction = normal * (-dl / dt)
                wp.atomic_add(impulse, 0, wp.spatial_vector(wp.cross(position[a]-com[0], reaction), reaction))


class AxialFibers:
    def __init__(self, initial, config, pin_count, device):
        self.config, self.pin_count, self.device = config, pin_count, device
        self.edge_count = len(initial) - 7
        self.rest = wp.array(np.linalg.norm(initial[7:] - initial[:-7], axis=1).astype(np.float32), device=device)
        self.multipliers = wp.zeros(self.edge_count, dtype=float, device=device)
        self._legacy_com = wp.zeros(1, dtype=wp.vec3, device=device)

    def solve(self, state, mass, dt, com, impulse, contact):
        if not isinstance(com, wp.array):
            # Preserve standalone callers; the cable supplies a persistent
            # device COM updated once per rigid step, including graph replay.
            self._legacy_com.assign(np.asarray(list(com), dtype=np.float32).reshape(1, 3))
            com = self._legacy_com
        self.multipliers.zero_()
        area = np.pi * (self.config['diameter']/2)**2 / 7
        for _ in range(self.config['axial_iterations']):
            for color in (0, 1):
                wp.launch(solve_fibers, dim=((self.edge_count//7 + 1)//2)*7, inputs=[
                    state.particle_q, state.particle_qd, mass, self.rest, self.multipliers,
                    self.pin_count, self.edge_count, color, area, self.config['axial_young_modulus'], dt,
                    com, impulse], device=self.device)
            contact()
