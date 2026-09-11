"""Ideal frictionless right-TCP guide coupled to MPM particle state."""
import numpy as np
from transforms3d.quaternions import quat2mat

from .threading import guide_projection


class SlidingGuide:
    def __init__(self, link, config, material_coordinate):
        self.link, self.config = link, config
        self.material_coordinate = material_coordinate
        self.impulse = np.zeros(6)
        self.radial_error = 0.

    def begin_step(self):
        self.impulse[:] = 0.

    def solve(self, cable, dt):
        state = cable.states[0].struct
        positions = state.particle_q.numpy().reshape(cable.sections, 7, 3)
        velocities = state.particle_qd.numpy().reshape(cable.sections, 7, 3)
        before = velocities.copy()
        pose = self.link.pose
        rotation = quat2mat(pose.q)
        axis = rotation[:, 0]
        centers = positions.mean(axis=1)
        shift, weight, coordinate = guide_projection(centers, pose.p, axis,
            self.material_coordinate, self.config["half_length"], len(cable.pin_ids_np)//7)
        self.material_coordinate = coordinate
        positions += shift[:, None, :]
        velocities += shift[:, None, :] / dt
        com = rotation @ self.link.cmass_local_pose.p + pose.p
        hole_velocity = self.link.velocity + np.cross(self.link.angular_velocity, centers - com)
        relative = velocities.mean(axis=1) - hole_velocity
        normal = relative - (relative @ axis)[:, None]*axis
        velocities -= (weight[:, None]*normal)[:, None, :]
        mass = cable.model.struct.particle_mass.numpy().reshape(cable.sections, 7, 1)
        impulse = -mass * (velocities - before)
        self.impulse[3:] += impulse.sum(axis=(0, 1))
        self.impulse[:3] += np.cross(positions - com, impulse).sum(axis=(0, 1))
        state.particle_q.assign(positions.reshape(-1, 3).astype(np.float32))
        state.particle_qd.assign(velocities.reshape(-1, 3).astype(np.float32))

    def measure(self, centers):
        """Measure the final state, including any subsequent rigid contact correction."""
        pose = self.link.pose
        axis = quat2mat(pose.q)[:, 0]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(centers, axis=0), axis=1))]
        at_hole = np.interp(self.material_coordinate, np.arange(len(centers)), arc)
        core = np.abs(arc-at_hole) <= self.config["half_length"]
        radial = centers - pose.p
        radial -= (radial @ axis)[:, None]*axis
        self.radial_error = float(np.max(np.linalg.norm(radial[core], axis=1), initial=0.))

    def apply_reaction(self, dt):
        if not np.isfinite(self.impulse).all():
            raise RuntimeError("Non-finite right-guide reaction")
        self.link.add_force_torque(self.impulse[3:]/dt, self.impulse[:3]/dt)
        self.impulse[:] = 0.
