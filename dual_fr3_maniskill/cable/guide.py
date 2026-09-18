"""Ideal frictionless right-TCP guide coupled to MPM particle state."""
import numpy as np
from transforms3d.quaternions import quat2mat


class SlidingGuide:
    def __init__(self, link, config, material_coordinate):
        self.link, self.config = link, config
        self._coordinate = float(material_coordinate)
        self._cable = None
        self._dirty = False
        self.radial_error = 0.

    @property
    def pose(self):
        """Physical hole reference in the existing TCP's local frame."""
        from dual_fr3_maniskill.engine.sapien_compat import sapien
        return self.link.pose * sapien.Pose(self.config.get("center_offset", [0., 0., 0.]))

    def bind(self, cable):
        """Allocate once; every solve uses the current MPM ping-pong state."""
        # ManiSkill selects its matching Warp before the MPM cable is created.
        # Keep imports lazy so geometry/configuration tools stay CUDA-independent.
        import warp as wp
        from dual_fr3_maniskill.cable import guide_cuda as kernels
        if self._cable is not None:
            if self._cable is not cable:
                raise ValueError("A sliding guide cannot be shared between cables")
            return
        if (cable.sections < 4 or not np.isfinite(self.config['half_length'])
                or self.config['half_length'] <= 0):
            raise RuntimeError("Invalid right TCP guide state")
        self.wp, self.kernels = wp, kernels
        self.device = cable.device
        self._cable = cable
        self._chunks = (cable.sections + kernels.CHUNK - 1) // kernels.CHUNK
        self._centers = wp.zeros(cable.sections, dtype=wp.vec3, device=self.device)
        self._arc = wp.zeros(cable.sections, dtype=float, device=self.device)
        self._chunk_data = wp.zeros(self._chunks, dtype=wp.vec4, device=self.device)
        self._offsets = wp.zeros(self._chunks, dtype=float, device=self.device)
        self._control = wp.zeros(2, dtype=float, device=self.device)
        self._reactions = wp.zeros(cable.sections, dtype=wp.spatial_vector, device=self.device)
        self._pose = wp.zeros(6, dtype=wp.vec3, device=self.device)
        summary = np.zeros(8, dtype=np.float32)
        summary[6] = self._coordinate
        self._summary = wp.array(summary, dtype=float, device=self.device)

    @property
    def material_coordinate(self):
        if self._cable is not None and self._dirty:
            self._coordinate = float(self._summary.numpy()[6])
            self._dirty = False
        return self._coordinate

    @material_coordinate.setter
    def material_coordinate(self, value):
        self._coordinate = float(value)
        self._dirty = False
        if self._cable is not None:
            source = self.wp.array(np.array([value], dtype=np.float32), dtype=float, device='cpu')
            self.wp.copy(self._summary, source, dest_offset=6, count=1)

    @property
    def impulse(self):
        """Diagnostic readback only; the solve loop never accesses this property."""
        return self._summary.numpy()[:6].copy() if self._cable is not None else np.zeros(6)

    def update_pose(self):
        """Upload 72 bytes once at each pre/post-PhysX coupling boundary."""
        pose = self.pose
        rotation = quat2mat(pose.q)
        axis = rotation[:, 0]
        com = rotation @ self.link.cmass_local_pose.p + self.link.pose.p
        data = np.array([pose.p, axis, axis / np.linalg.norm(axis), com,
                         self.link.velocity, self.link.angular_velocity], dtype=np.float32)
        if not np.isfinite(data).all():
            raise RuntimeError("Invalid right TCP guide state")
        self._pose.assign(data)

    def begin_step(self, *, reset=False):
        if self._cable is None:
            return
        self.wp.launch(self.kernels.clear_reaction, dim=6,
                       inputs=[self._summary], device=self.device)
        if reset:
            zero = self.wp.array(np.zeros(1, dtype=np.float32), dtype=float, device='cpu')
            self.wp.copy(self._summary, zero, dest_offset=7, count=1)

    def solve(self, cable, dt):
        if self._cable is None:
            self.bind(cable)
            self.update_pose()
        elif self._cable is not cable:
            raise ValueError("A sliding guide cannot be shared between cables")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Guide time step must be positive and finite")
        state = cable.states[0].struct
        wp, k = self.wp, self.kernels
        pinned = len(cable.pin_ids_np) // 7
        wp.launch(k.guide_centers, dim=cable.sections,
                  inputs=[state.particle_q, self._centers], device=self.device)
        wp.launch(k.scan_sections, dim=self._chunks, inputs=[self._centers, self._pose,
                  self._summary, self._arc, self._chunk_data, cable.sections], device=self.device)
        wp.launch(k.select_crossing, dim=1, inputs=[self._chunk_data, self._arc, self._offsets,
                  self._summary, self._control, self._chunks, cable.sections, pinned], device=self.device)
        wp.launch(k.project_guide, dim=cable.sections, inputs=[state.particle_q, state.particle_qd,
                  cable.model.struct.particle_mass, self._centers, self._pose, self._arc,
                  self._offsets, self._control, self._summary, self._reactions,
                  self.config['half_length'], pinned, dt], device=self.device)
        wp.launch(k.sum_reactions, dim=self._chunks,
                  inputs=[self._reactions, self._summary, cable.sections], device=self.device)
        self._dirty = True

    def measure(self, centers):
        """Measure the final state, including any subsequent rigid contact correction."""
        pose = self.pose
        axis = quat2mat(pose.q)[:, 0]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(centers, axis=0), axis=1))]
        at_hole = np.interp(self.material_coordinate, np.arange(len(centers)), arc)
        core = np.abs(arc-at_hole) <= self.config["half_length"]
        radial = centers - pose.p
        radial -= (radial @ axis)[:, None]*axis
        self.radial_error = float(np.max(np.linalg.norm(radial[core], axis=1), initial=0.))

    def apply_reaction(self, dt):
        # One 32-byte readback, before advancing PhysX. Device failures are
        # sticky, so a later valid crossing cannot conceal loss of threading.
        summary = self._summary.numpy()
        self._coordinate = float(summary[6])
        self._dirty = False
        errors = {1: "Invalid right TCP guide state",
                  2: "Cable no longer crosses the right TCP guide; stop and reset the scene",
                  3: "USB fixed end or cable free end reached the right TCP guide"}
        if not np.isfinite(summary[7]):
            raise RuntimeError("Non-finite right-guide reaction")
        if summary[7] != 0:
            raise RuntimeError(errors.get(int(summary[7]), "Invalid right TCP guide state"))
        if not np.isfinite(summary).all():
            raise RuntimeError("Non-finite right-guide reaction")
        self.link.add_force_torque(summary[3:6].astype(float)/dt, summary[:3].astype(float)/dt)
        collector = getattr(getattr(self._cable, "env", None), "force_collector", None)
        if collector is not None:
            collector.mpm_reaction(self.link, summary[:6].astype(float),
                (self.link.pose*self.link.cmass_local_pose).p, source="mpm_ideal_guide_reaction")
        self.begin_step()
