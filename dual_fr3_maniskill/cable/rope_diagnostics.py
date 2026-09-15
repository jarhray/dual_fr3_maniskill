"""Opt-in, post-substep diagnostics for native rope experiments (no renderer)."""
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np


def json_safe(value):
    """Preserve non-finite failure evidence as strings in standards-compliant JSON."""
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return str(value)
    return value


def actor_state(actor):
    return dict(name=actor.name, position_m=actor.pose.p.tolist(), quaternion_wxyz=actor.pose.q.tolist(),
                velocity_m_s=np.asarray(actor.velocity).tolist(),
                angular_velocity_rad_s=np.asarray(actor.angular_velocity).tolist())


class RopeSubstepTrace:
    """Keep failure context without changing poses, velocities, forces or guards.

    Call after ``follow_plug`` and before starting another physics substep.
    Geometry is freshly audited when requested and on a speed failure; contact point
    separation from PhysX is explicitly labelled as a pre-solve quantity.
    Full actor states are retained only in the bounded tail, not the JSONL stream.
    """

    def __init__(self, capacity=64, stream=None):
        self.tail = deque(maxlen=capacity)
        self.stream = stream
        self.steps = 0
        self.peak_speed = self.peak_energy = self.peak_penetration = 0.
        self.peak_proxy_error = self.peak_impulse = self.peak_joint_gap = 0.
        self.contact_steps = 0
        self.geometry_audits = 0
        self.nonfinite_state_detected = False
        self.invalid_proxy_pose_detected = False

    def record(self, cable, time_s, *, audit=True):
        starts, ends = cable._endpoints()
        linear = np.array([actor.velocity for actor in cable.links])
        angular = np.array([actor.angular_velocity for actor in cable.links])
        speeds = cable._endpoint_speeds(linear, angular)
        speed = float(speeds.max())
        audit = audit or not np.isfinite(speed) or speed > cable.config["rope_actor"]["max_speed"]
        audit_error = None
        if audit:
            try:
                cable._audit_contacts()
            except Exception as exc:
                # A non-finite state may defeat mesh distance queries. Keep
                # the velocities and contacts that led to it in the report.
                audit_error = str(exc)
        energy = cable._kinetic_energy()
        errors = []
        for proxy, source in cable.proxies:
            pose, source_pose = proxy.pose, source.pose
            a, b = pose.q, source_pose.q
            norm, source_norm = float(np.linalg.norm(a)), float(np.linalg.norm(b))
            valid = bool(np.isfinite(np.r_[pose.p, a]).all() and abs(norm-1.) <= 1.e-4)
            dot = abs(float(np.dot(a, b)/(norm*source_norm))) if norm*source_norm > 0. else float("nan")
            errors.append(dict(source=source.name,
                               position_m=float(np.linalg.norm(pose.p-source_pose.p)),
                               quaternion_norm=norm, pose_valid=valid,
                               angle_rad=float(2*np.arccos(np.clip(dot, 0., 1.)))))
        self.invalid_proxy_pose_detected |= any(not e["pose_valid"] for e in errors)
        contacts = []
        for contact in cable.scene.get_contacts():
            if contact.actor0.id not in cable.link_ids and contact.actor1.id not in cable.link_ids:
                continue
            contacts.append(dict(actor0=contact.actor0.name, actor1=contact.actor1.name,
                points=[dict(position_m=np.asarray(p.position).tolist(),
                             normal=np.asarray(p.normal).tolist(),
                             impulse_Ns=np.asarray(p.impulse).tolist(),
                             pre_solve_separation_m=float(p.separation)) for p in contact.points]))
        current_limit = getattr(cable, "_step_limit_for_step", None) == cable.steps
        row = dict(step=cable.steps, time_s=float(time_s), dt_s=cable._dt,
                   suggested_timestep_s=getattr(cable, "_step_limit", None) if current_limit else None,
                   predicted_contact_travel_m=(cable._dt*cable._contact_speed_bound
                       if current_limit and getattr(cable, "_contact_speed_bound", None) is not None else None),
                   predicted_motion_budget_fraction=(cable._dt*cable._motion_budget_rate
                       if current_limit and getattr(cable, "_motion_budget_rate", None) is not None else None),
                   max_endpoint_speed_m_s=speed,
                   max_speed_segment_index=int(np.argmax(speeds)),
                   kinetic_energy_before_J=cable._energy_before_step,
                   kinetic_energy_J=energy, kinetic_energy_change_J=energy-cable._energy_before_step,
                   largest_contact_impulse_Ns=cable._last_contact_impulse,
                   largest_contact_pair=cable._last_contact_pair,
                   max_joint_gap_m=float(np.linalg.norm(ends[:-1]-starts[1:], axis=1).max()),
                   attachment_error_m=cable.attachment_error(),
                   guide=cable.guide_diagnostics(),
                   proxy_errors=errors, contacts=contacts,
                   geometry_audited=bool(audit and audit_error is None), geometry_audit_error=audit_error,
                   max_penetration_m=cable.max_depth if audit and audit_error is None else None,
                   max_penetration_contact=cable.max_penetration_contact if audit and audit_error is None else None)
        self.steps += 1
        self.nonfinite_state_detected |= not bool(np.isfinite([speed, energy]).all())
        self.contact_steps += bool(contacts)
        self.peak_speed = max(self.peak_speed, row["max_endpoint_speed_m_s"])
        self.peak_energy = max(self.peak_energy, energy)
        if row["geometry_audited"]:
            self.geometry_audits += 1
            self.peak_penetration = max(self.peak_penetration, cable.max_depth)
        self.peak_joint_gap = max(self.peak_joint_gap, row["max_joint_gap_m"])
        self.peak_impulse = max(self.peak_impulse, cable._last_contact_impulse)
        self.peak_proxy_error = max(self.peak_proxy_error, max((e["position_m"] for e in errors), default=0.))
        if self.stream is not None:
            import json
            self.stream.write(json.dumps(json_safe(row), allow_nan=False)+"\n")
            self.stream.flush()
        self.tail.append(dict(row, actors=[actor_state(a) for a in cable.links],
                              fingers=[actor_state(source) for _, source in cable.proxies],
                              proxies=[actor_state(proxy) for proxy, _ in cable.proxies],
                              plug=actor_state(cable.plug)))
        return row

    def summary(self):
        return dict(substeps=self.steps, contact_substeps=self.contact_steps,
                    geometry_audits=self.geometry_audits,
                    nonfinite_state_detected=self.nonfinite_state_detected,
                    invalid_proxy_pose_detected=self.invalid_proxy_pose_detected,
                    max_endpoint_speed_m_s=self.peak_speed, max_kinetic_energy_J=self.peak_energy,
                    max_penetration_m=self.peak_penetration if self.geometry_audits else None,
                    max_joint_gap_m=self.peak_joint_gap,
                    max_proxy_position_error_m=self.peak_proxy_error,
                    max_contact_impulse_Ns=self.peak_impulse)


class RopeRunRecorder:
    """Optional full-run inputs plus a bounded substep tail; never advances physics.

    Normal substeps skip mesh distance audits. Failure snapshots audit the current
    geometry separately, preserving the original guard's evidence and exception.
    Actor states are diagnostic evidence, not a PhysX restart checkpoint.
    """

    def __init__(self, directory, *, metadata, on_error, capacity=64):
        root = Path(directory).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        prefix = datetime.now(timezone.utc).strftime("run_%Y%m%dT%H%M%SZ_")
        self.directory = Path(tempfile.mkdtemp(prefix=prefix, dir=root))
        self.on_error = on_error
        self.capacity = capacity
        self.time = 0.
        self.control_steps = self.epoch = self.failures = 0
        self.errors = []
        self.cable = None
        self.last_control = None
        self.trace = RopeSubstepTrace(capacity)
        source_root = Path(__file__).resolve().parents[1]
        hashes = {str(p.relative_to(source_root)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in sorted(source_root.rglob("*.py"))}
        self._write("metadata.json", dict(schema_version=2, **metadata, source_sha256=hashes,
            tail_capacity=capacity, geometry_audit_policy="on failure or speed limit",
            state_scope="Diagnostic actor states and control inputs; not a PhysX restart checkpoint"))
        self.controls = (self.directory / "controls.jsonl").open("x")
        self.substeps = (self.directory / "substeps.jsonl").open("x")

    def _write(self, filename, value):
        with (self.directory / filename).open("x") as stream:
            json.dump(json_safe(value), stream, allow_nan=False, indent=2)
            stream.write("\n")

    @staticmethod
    def _robot(env):
        robot = env.agent.robot
        return dict(joint_names=list(env.agent.joints), qpos=robot.get_qpos().tolist(),
                    qvel=robot.get_qvel().tolist(),
                    tcp_poses={name: dict(position_m=link.pose.p.tolist(), quaternion_wxyz=link.pose.q.tolist())
                               for name, link in env.agent.links.items() if name.endswith("hand_tcp")})

    def _state(self, env, *, audit=False):
        result = dict(time_s=self.time, robot=self._robot(env),
            fixtures={name: dict(position_m=actor.pose.p.tolist(), quaternion_wxyz=actor.pose.q.tolist())
                      for name, actor in env.fixtures.items()})
        cable = env.cable
        if cable is not None:
            snapshot = RopeSubstepTrace(capacity=1)
            snapshot.record(cable, self.time, audit=audit)
            result["cable"] = snapshot.tail[-1]
            result["pending_proxy_reactions"] = [dict(source=link.name,
                impulse_Ns=np.asarray(impulse).tolist(), position_m=np.asarray(point).tolist())
                for link, impulse, point in cable._pending_reactions]
        return result

    def capture(self, event, env=None, **values):
        """IO/diagnostic errors must not replace a physics error or stop stepping."""
        try:
            return getattr(self, "_" + event)(env, **values)
        except Exception as exc:
            message = f"Cable trace {event} failed: {type(exc).__name__}: {exc}"
            if message not in self.errors:
                self.errors.append(message)
                self.on_error(message)
            return None

    def _control(self, env, *, action):
        self.control_steps += 1
        new_cable = env.cable is not None and env.cable is not self.cable
        if new_cable:
            self.epoch += 1
            self.cable = env.cable
            self.trace = RopeSubstepTrace(self.capacity)
        # Include preparation and keyboard waiting, including ticks before spawn.
        row = dict(control_step=self.control_steps, time_s=self.time, epoch=self.epoch,
                   cable_present=env.cable is not None, target=np.asarray(action).tolist(),
                   **self._robot(env))
        self.controls.write(json.dumps(json_safe(row), allow_nan=False) + "\n")
        self.controls.flush()
        # A 64-substep tail may cover less than one 20 ms control period.
        # Retain its start independently so failures still have a usable
        # pre-failure cold-start state at the recorded robot/control boundary.
        self.last_control = dict(control=row, state=self._state(env)) if env.cable is not None else None
        if new_cable:
            self._write(f"initial_{self.epoch:03d}.json", self.last_control["state"])

    def _rigid_step(self, env, *, dt):
        self.time += dt

    def _substep(self, env, *, dt):
        self.time += dt
        row = self.trace.record(env.cable, self.time, audit=False)
        self.substeps.write(json.dumps(json_safe(dict(row, epoch=self.epoch,
            control_step=self.control_steps)), allow_nan=False) + "\n")
        self.substeps.flush()

    def _failure(self, env, *, reason):
        self.failures += 1
        # Save the original guard result BEFORE a fresh diagnostic mesh query.
        cable = env.cable
        guard = None if cable is None else dict(max_penetration_m=cable.max_depth,
            max_penetration_contact=cable.max_penetration_contact)
        report = dict(reason=reason, time_s=self.time, epoch=self.epoch,
                      control_step=self.control_steps, guard=guard,
                      summary=self.trace.summary(), tail=list(self.trace.tail),
                      last_control=self.last_control)
        try:
            report["failure_state"] = self._state(env, audit=True)
        except Exception as exc:
            report["snapshot_error"] = f"{type(exc).__name__}: {exc}"
        report["recording_errors"] = list(self.errors)
        filename = f"failure_{self.failures:03d}.json"
        self._write(filename, report)
        return self.directory / filename

    def _reset(self, env):
        # Retain the old files; next control tick starts a new cable epoch.
        self.cable = None
        self.last_control = None

    def _close(self, env):
        try:
            self._write("summary.json", dict(time_s=self.time, control_steps=self.control_steps,
                epochs=self.epoch, failures=self.failures, last_epoch=self.trace.summary(),
                tail=list(self.trace.tail), recording_errors=self.errors))
        finally:
            self.controls.close()
            self.substeps.close()
