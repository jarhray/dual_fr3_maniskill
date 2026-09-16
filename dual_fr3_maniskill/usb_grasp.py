"""Contact-gated USB grasp observation using simulation pose ground truth.

This module observes physics; it neither changes poses nor increases grip force.
Times are simulation seconds. Pose quaternions use SAPIEN's w, x, y, z order.
"""
from dataclasses import asdict, dataclass, fields

import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat


@dataclass(frozen=True)
class GraspThresholds:
    contact_min_force_N: float = .10
    contact_hold_s: float = .10
    close_timeout_s: float = 5.
    verification_min_s: float = .35
    stable_hold_s: float = .25
    verification_timeout_s: float = 3.
    slip_translation_m: float = .002
    slip_rotation_rad: float = .14
    slip_hold_s: float = .10
    drop_translation_m: float = .035
    drop_rotation_rad: float = 1.05
    drop_hold_s: float = .10
    contact_loss_hold_s: float = .25

    @classmethod
    def from_config(cls, config=None):
        config = config or {}
        names = {field.name for field in fields(cls)}
        result = cls(**{key: float(value) for key, value in config.items() if key in names})
        for name, value in asdict(result).items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"grasp.{name} must be positive and finite")
        if result.drop_translation_m <= result.slip_translation_m:
            raise ValueError("grasp.drop_translation_m must exceed slip_translation_m")
        if result.drop_rotation_rad <= result.slip_rotation_rad:
            raise ValueError("grasp.drop_rotation_rad must exceed slip_rotation_rad")
        if result.verification_timeout_s <= max(result.verification_min_s, result.stable_hold_s):
            raise ValueError("grasp.verification_timeout_s must exceed verification/stable durations")
        return result


def relative_pose(tcp_pose, usb_pose):
    """Return inverse(T_tcp) * T_usb, independent of common rigid movement."""
    rotation = quat2mat(np.asarray(tcp_pose.q)).T
    return (rotation @ (np.asarray(usb_pose.p)-np.asarray(tcp_pose.p)),
            rotation @ quat2mat(np.asarray(usb_pose.q)))


def _rotation_angle(rotation):
    return float(np.arccos(np.clip((np.trace(rotation)-1.)/2., -1., 1.)))


def read_usb_finger_normal_loads(env, plug, dt, side="left"):
    """Normal force ON each real finger from USB contacts, in N.

    Rope contact proxies are deliberately excluded. Only reported normal
    impulses are available; this does not estimate tangential friction.
    """
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("Contact observation requires a positive finite timestep")
    names = [side+"_fr3_"+suffix for suffix in ("leftfinger", "rightfinger")]
    loads = dict.fromkeys(names, 0.)
    if plug is None:
        return loads
    ids = {env.agent.links[name].id: name for name in names if name in env.agent.links}
    for contact in env._scene.get_contacts():
        a, b = contact.actor0, contact.actor1
        finger = b if a.id == plug.id else a if b.id == plug.id else None
        if finger is None or finger.id not in ids:
            continue
        loads[ids[finger.id]] += sum(abs(float(np.dot(point.impulse, point.normal)))
                                     for point in contact.points)/dt
    return loads


class UsbGraspMonitor:
    """Finite-state observer; call once after each completed physics substep.

    Scene operations own the actual world support. Call ``released`` only after
    removing every temporary support successfully. A failed operation must call
    ``fail`` and leaves ``external_support`` unchanged.
    """
    def __init__(self, config=None):
        self.thresholds = GraspThresholds.from_config(config)
        self.reset()

    def reset(self):
        self.state, self.reason = "not_created", "usb_not_created"
        self.external_support = False
        self.time_s = 0.
        self.created_at = self.closing_at = self.released_at = None
        self._contact_since = self._stable_since = self._slip_since = None
        self._drop_since = self._lost_since = None
        self._baseline = self._relative = self._previous = None
        self._creation_world = self._world = None
        self._previous_time = None
        self.loads = {}
        self.translation_m = self.rotation_rad = None
        self.relative_linear_velocity = self.relative_angular_speed = None
        self.ever_stable = False
        self.manual_release = False
        self.history = []

    def _transition(self, state, reason, time_s):
        if (state, reason) != (self.state, self.reason):
            self.history.append(dict(time_s=float(time_s), state=state, reason=reason))
            self.history = self.history[-32:]
        self.state, self.reason = state, reason

    def created(self, time_s, usb_pose=None):
        if self.state != "not_created":
            return False
        self.created_at = self.time_s = float(time_s)
        self.external_support = True
        if usb_pose is not None:
            self._observe_world(usb_pose)
        self._transition("supported", "temporary_world_support_active", time_s)
        return True

    def _observe_world(self, pose):
        position, rotation = np.asarray(pose.p).copy(), quat2mat(np.asarray(pose.q))
        self._world = (position, rotation) if np.isfinite(position).all() and np.isfinite(rotation).all() else None
        if self._creation_world is None and self._world is not None:
            self._creation_world = (position.copy(), rotation.copy())

    def begin_closing(self, time_s):
        if not self.external_support or self.state == "failed":
            raise ValueError("Closing requires a created USB with temporary support")
        if self.closing_at is None:
            self.closing_at = float(time_s)

    @property
    def ready_to_release(self):
        return self.external_support and self.state == "contact_ready"

    def released(self, time_s, tcp_pose, usb_pose, manual=False):
        if self.released_at is not None:
            return False
        if not self.external_support or (not manual and not self.ready_to_release):
            raise ValueError("Release requires sustained bilateral USB contact or explicit manual release")
        self.released_at = self.time_s = float(time_s)
        self.external_support = False
        self.manual_release = bool(manual)
        self._baseline = relative_pose(tcp_pose, usb_pose)
        if not all(np.isfinite(value).all() for value in self._baseline):
            self._baseline = self._relative = None
            self.translation_m = self.rotation_rad = None
            self.relative_linear_velocity = self.relative_angular_speed = None
            self.fail("nonfinite_release_pose", time_s)
            raise RuntimeError("Cannot establish USB release baseline from a nonfinite pose")
        self._relative = self._baseline
        self._previous = self._baseline
        self._previous_time = float(time_s)
        self._stable_since = self._slip_since = self._drop_since = self._lost_since = None
        self.translation_m = self.rotation_rad = 0.
        self.relative_linear_velocity = np.zeros(3)
        self.relative_angular_speed = 0.
        self._transition("verifying", "world_support_released_verifying_contact", time_s)
        return True

    def fail(self, reason, time_s=None):
        self._transition("failed", str(reason), self.time_s if time_s is None else time_s)

    @staticmethod
    def _since(previous, condition, time_s):
        return (time_s if previous is None else previous) if condition else None

    @staticmethod
    def _held(since, duration, time_s):
        return since is not None and time_s-since >= duration-1.e-10

    def observe(self, time_s, tcp_pose, usb_pose, finger_normal_loads_N):
        now = float(time_s)
        if not np.isfinite(now) or now < self.time_s-1.e-10:
            raise ValueError("Grasp observation time must be finite and monotonic")
        self.time_s = now
        if self.state == "not_created":
            return self.snapshot()
        self.loads = {str(name): float(load) for name, load in finger_normal_loads_N.items()}
        self._observe_world(usb_pose)
        self._relative = relative_pose(tcp_pose, usb_pose)
        if (not all(np.isfinite(load) and load >= 0 for load in self.loads.values()) or
                not all(np.isfinite(value).all() for value in self._relative)):
            self.loads = {name: load if np.isfinite(load) else None for name, load in self.loads.items()}
            self._relative = None
            self.translation_m = self.rotation_rad = None
            self.relative_linear_velocity = self.relative_angular_speed = None
            self._previous = self._previous_time = None
            self.fail("nonfinite_grasp_observation", now)
            return self.snapshot()
        if self._previous_time is not None and now > self._previous_time:
            dt = now-self._previous_time
            self.relative_linear_velocity = (self._relative[0]-self._previous[0])/dt
            self.relative_angular_speed = _rotation_angle(self._previous[1].T @ self._relative[1])/dt
        self._previous, self._previous_time = self._relative, now
        t = self.thresholds
        bilateral = len(self.loads) == 2 and all(load >= t.contact_min_force_N for load in self.loads.values())
        no_contacts = not any(load >= t.contact_min_force_N for load in self.loads.values())
        self._contact_since = self._since(self._contact_since, bilateral, now)
        # Terminal classification does not freeze observations: a dropped USB
        # keeps falling, and its current displacement must match relative_pose.
        if self._baseline is not None:
            self.translation_m = float(np.linalg.norm(self._relative[0]-self._baseline[0]))
            self.rotation_rad = _rotation_angle(self._baseline[1].T @ self._relative[1])
        if self.state in ("failed", "dropped"):
            return self.snapshot()
        if self.external_support:
            if self._held(self._contact_since, t.contact_hold_s, now):
                self._transition("contact_ready", "bilateral_contact_held_still_externally_supported", now)
            else:
                self._transition("supported", "waiting_for_sustained_bilateral_contact", now)
                if self.closing_at is not None and now-self.closing_at >= t.close_timeout_s:
                    self.fail("closing_timeout_insufficient_bilateral_contact", now)
            return self.snapshot()
        if self._baseline is None:
            self.fail("released_without_pose_baseline", now)
            return self.snapshot()
        moved = self.translation_m > t.slip_translation_m or self.rotation_rad > t.slip_rotation_rad
        escaped = self.translation_m > t.drop_translation_m or self.rotation_rad > t.drop_rotation_rad
        self._slip_since = self._since(self._slip_since, moved or not bilateral, now)
        self._drop_since = self._since(self._drop_since, escaped, now)
        self._lost_since = self._since(self._lost_since, no_contacts, now)
        self._stable_since = self._since(self._stable_since, bilateral and not moved, now)
        if self._held(self._drop_since, t.drop_hold_s, now):
            self._transition("dropped", "usb_left_grasp_pose_envelope", now)
        elif self._held(self._lost_since, t.contact_loss_hold_s, now):
            self._transition("dropped", "both_finger_contacts_lost", now)
        elif self._held(self._slip_since, t.slip_hold_s, now):
            self._transition("slipping", "relative_pose_drift" if moved else "bilateral_contact_lost", now)
        elif (now-self.released_at >= t.verification_min_s and
              self._held(self._stable_since, t.stable_hold_s, now)):
            self.ever_stable = True
            self._transition("stable", "released_grasp_stable", now)
        elif not self.ever_stable and now-self.released_at >= t.verification_timeout_s:
            self.fail("released_grasp_verification_timeout", now)
        return self.snapshot()

    def snapshot(self):
        def world_pose(value):
            return None if value is None else dict(frame="world", position_m=value[0].tolist(),
                quaternion_wxyz=mat2quat(value[1]).tolist())
        world_delta = (None if self._world is None or self._creation_world is None
                       else self._world[0]-self._creation_world[0])
        relative_delta = (None if self._relative is None or self._baseline is None
                          else self._relative[0]-self._baseline[0])
        relative = None if self._relative is None else dict(
            position_m=self._relative[0].tolist(), quaternion_wxyz=mat2quat(self._relative[1]).tolist())
        baseline = None if self._baseline is None else dict(
            position_m=self._baseline[0].tolist(), quaternion_wxyz=mat2quat(self._baseline[1]).tolist())
        return dict(state=self.state, reason=self.reason, external_support=self.external_support,
            stable=self.state == "stable" and not self.external_support,
            ready_to_release=self.ready_to_release, time_s=self.time_s,
            created_at_s=self.created_at, closing_at_s=self.closing_at, released_at_s=self.released_at,
            manual_release=self.manual_release, observation_source="simulation_ground_truth_and_normal_contacts",
            relative_frame="left_fr3_hand_tcp", relative_pose=relative, release_baseline=baseline,
            world_pose=world_pose(self._world), creation_world_pose=world_pose(self._creation_world),
            world_displacement_m=None if world_delta is None else world_delta.tolist(),
            world_translation_m=None if world_delta is None else float(np.linalg.norm(world_delta)),
            world_rotation_rad=None if world_delta is None else _rotation_angle(self._creation_world[1].T@self._world[1]),
            relative_displacement_m=None if relative_delta is None else relative_delta.tolist(),
            relative_translation_m=self.translation_m, relative_rotation_rad=self.rotation_rad,
            relative_linear_velocity_m_s=(None if self.relative_linear_velocity is None
                                         else self.relative_linear_velocity.tolist()),
            relative_angular_speed_rad_s=self.relative_angular_speed,
            finger_normal_loads_N=dict(self.loads), thresholds=asdict(self.thresholds),
            transitions=list(self.history))
