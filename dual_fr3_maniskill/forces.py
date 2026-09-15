"""Read-only interaction loads, integrated over the actual physics substeps.

All signs describe force ON the named sensor body. Wrenches are expressed in
that body's axes, with moments about its frame origin (TCP for hand signals).
These are source-resolved interaction loads, not a simulated wrist transducer.
"""
from dataclasses import dataclass, field
import numpy as np
from transforms3d.quaternions import quat2mat

from .cable.model import USB_LINK


@dataclass
class LoadWindow:
    frame_id: str
    available: bool = True
    reasons: set = field(default_factory=set)
    sources: set = field(default_factory=set)
    objects: set = field(default_factory=set)
    impulse: np.ndarray = field(default_factory=lambda: np.zeros(6))
    step_impulse: np.ndarray = field(default_factory=lambda: np.zeros(6))
    normal_impulse: float = 0.
    step_normal: float = 0.
    normal_known: bool = True
    peak_force: float = 0.
    peak_normal: float = 0.

    def invalidate(self, reason):
        self.available = False
        self.reasons.add(reason)

    def add(self, pose, impulse, origin, angular_impulse, source, other, normal):
        linear = np.asarray(impulse, dtype=float)
        angular = np.asarray(angular_impulse, dtype=float) + np.cross(
            np.asarray(origin)-pose.p, linear)
        rotation = quat2mat(pose.q).T
        value = np.r_[rotation @ linear, rotation @ angular]
        if not np.isfinite(value).all() or (normal is not None and not np.isfinite(normal)):
            self.invalidate("nonfinite_force_sample")
            return
        self.step_impulse += value
        self.sources.add(source)
        if np.any(value) or normal:
            self.objects.add(other)
        if normal is None:
            self.normal_known = False
        else:
            self.step_normal += normal

    def end_step(self, dt):
        self.impulse += self.step_impulse
        self.normal_impulse += self.step_normal
        self.peak_force = max(self.peak_force, float(np.linalg.norm(self.step_impulse[:3]))/dt)
        self.peak_normal = max(self.peak_normal, self.step_normal/dt)
        self.step_impulse[:] = 0.
        self.step_normal = 0.

    def result(self, duration):
        valid = self.available and duration > 0
        native = "physx_normal_contact" in self.sources
        missing = []
        if native:
            missing.append("tangential_contact_impulses")
        if "rope_anchor_constraint" in self.sources:
            missing.append("angular_drive_torque_if_enabled")
        if "ideal_guide_load_not_resolved_per_finger" in self.reasons:
            missing.append("ideal_guide_load_not_resolved_per_finger")
        return dict(frame_id=self.frame_id, available=valid,
                    scope="partial_interaction_wrench" if missing else "reported_model_interaction_wrench",
                    reasons=sorted(self.reasons), sources=sorted(self.sources),
                    missing_components=missing, objects=sorted(self.objects),
                    force_N=(self.impulse[:3]/duration).tolist() if valid else None,
                    torque_Nm=(self.impulse[3:]/duration).tolist() if valid else None,
                    normal_load_N=self.normal_impulse/duration if valid and self.normal_known else None,
                    peak_substep_force_N=self.peak_force if valid else None,
                    peak_substep_normal_load_N=self.peak_normal if valid and self.normal_known else None)


class ForceCollector:
    """Collect native contacts and cable reactions without modifying physics."""

    def __init__(self, env, base_names=(), constraint_reader=None):
        self.env = env
        self.base_names = set(base_names)
        self.active = False
        self.snapshot = None
        self.constraint_reader = constraint_reader
        if constraint_reader is None:
            try:
                from ._rope_physx import read_pair_constraint
                self.constraint_reader = read_pair_constraint
            except ImportError:
                pass

    def begin(self, time):
        self.start_time = float(time)
        self.duration = 0.
        self.substeps = 0
        self.windows = {}
        self.frames = {}
        self.active = True
        env = self.env
        self.cable = getattr(env, "cable", None)
        self.plug = getattr(self.cable, "plug", None)
        self.solver = getattr(self.cable, "solver", "none")
        self.fixtures = {a.id: a for a in env.fixtures.values()}
        self.rope_ids = getattr(self.cable, "link_ids", set())
        self.proxies = getattr(self.cable, "proxy_links", {})
        self.plug_held = self.plug is not None and (
            getattr(env, "mount_drive", None) is not None or
            env.agent.links.get(USB_LINK) is self.plug)
        self.fingers = {n: a for n, a in env.agent.links.items()
                        if n.endswith(("leftfinger", "rightfinger"))}
        loaded_bases = self.base_names & set(env.fixtures)
        for side in ("left", "right"):
            frame = side+"_fr3_hand_tcp"
            if frame not in env.agent.links:
                continue
            self.frames[frame] = env.agent.links[frame]
            for category in ("cable", "fixtures", "usb_base"):
                self._new(side+"/"+category, frame, category, loaded_bases)
        for name, actor in self.fingers.items():
            self.frames[name] = actor
            for category in ("cable", "fixtures", "usb"):
                key = "fingers/"+name+"/"+category
                self._new(key, name, category, loaded_bases)
                if category == "usb" and name.startswith("left_") and self.plug_held:
                    self.windows[key].invalidate("fixed_mount_excludes_usb_finger_contacts")
        if self.plug is not None:
            self.frames[self.plug.name] = self.plug
            for category in ("fixtures", "usb_base"):
                self._new("usb/"+category, self.plug.name, category, loaded_bases)
        if self.cable is not None and self.solver == "rope_actor":
            window = self.windows.get("left/cable")
            if window is not None and self.plug_held:
                window.sources.add("rope_anchor_constraint")
                window.normal_known = False
                if self.constraint_reader is None:
                    window.invalidate("constraint_reader_unavailable_rebuild_dual_fr3_maniskill")
        # The MPM ideal guide acts on the TCP, not on individual fingers.
        if self.solver == "mpm" and getattr(self.cable, "guide", None) is not None:
            for key, window in self.windows.items():
                if key.startswith("fingers/right_") and key.endswith("/cable"):
                    window.reasons.add("ideal_guide_load_not_resolved_per_finger")

    def _new(self, key, frame, category, loaded_bases):
        window = self.windows[key] = LoadWindow(frame)
        if category in ("fixtures", "usb_base", "usb") or self.solver == "rope_actor":
            window.sources.add("physx_normal_contact")
        if category == "cable" and self.solver == "mpm":
            window.sources.add("mpm_applied_reaction")
            window.normal_known = False
        if category in ("cable", "usb") and self.cable is None:
            window.invalidate("cable_not_spawned")
        if category == "usb_base" and not loaded_bases:
            window.invalidate("usb_base_not_configured_or_not_loaded")
        if category == "fixtures" and not self.fixtures:
            window.invalidate("no_fixtures_loaded")

    def _route(self, receiver, category, other, impulse, origin, angular=(0., 0., 0.),
               normal=None, source="physx_normal_contact"):
        if not self.active:
            return
        name = receiver.name
        keys = []
        side = None
        if receiver is self.plug:
            if category == "fixtures":
                keys.append("usb/fixtures")
                if other in self.base_names:
                    keys.append("usb/usb_base")
            if self.plug_held:
                side = "left"
        else:
            for candidate in ("left", "right"):
                if name in (candidate+"_fr3_hand", candidate+"_fr3_hand_tcp",
                            candidate+"_fr3_leftfinger", candidate+"_fr3_rightfinger"):
                    side = candidate
        if side and category in ("cable", "fixtures"):
            keys.append(side+"/"+category)
            if category == "fixtures" and other in self.base_names:
                keys.append(side+"/usb_base")
        if name in self.fingers:
            keys.append("fingers/"+name+"/"+category)
        for key in keys:
            window = self.windows.get(key)
            if window is not None:
                window.add(self.frames[window.frame_id].pose, impulse, origin,
                           angular, source, other, normal)

    def mpm_reaction(self, receiver, spatial_impulse, origin, source="mpm_applied_reaction"):
        """Observe the exact impulse applied by MPM; spatial order is torque, force."""
        if self.active:
            self._route(receiver, "cable", "cable", spatial_impulse[3:], origin,
                        angular=spatial_impulse[:3], source=source)

    def sample(self, dt):
        if not self.active:
            return
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Force sampling requires a positive finite physics timestep")
        for contact in self.env._scene.get_contacts():
            a, b = contact.actor0, contact.actor1
            for raw, other, sign in ((a, b, 1.), (b, a, -1.)):
                receiver = self.proxies.get(raw.id, raw)
                if other.id in self.rope_ids:
                    category, other_name = "cable", "cable"
                elif other.id in self.fixtures:
                    category, other_name = "fixtures", other.name
                elif self.plug is not None and other.id == self.plug.id:
                    category, other_name = "usb", other.name
                else:
                    continue
                for point in contact.points:
                    # PhysX 4's report is the normal impulse ON actor0.
                    impulse = sign*np.asarray(point.impulse, dtype=float)
                    self._route(receiver, category, other_name, impulse, point.position,
                                normal=abs(float(np.dot(impulse, point.normal))))
        if self.solver == "rope_actor" and self.plug_held and self.constraint_reader is not None:
            try:
                load = self.constraint_reader(self.plug._ptr, self.cable.links[0]._ptr)
                if not load["awake"]:
                    raise ValueError("sleeping_constraint_report_not_updated")
                self._route(self.plug, "cable", "cable", np.asarray(load["force"])*dt,
                            load["origin"], angular=np.asarray(load["torque"])*dt,
                            source="rope_anchor_constraint")
            except (ValueError, RuntimeError) as exc:
                if "left/cable" in self.windows:
                    self.windows["left/cable"].invalidate(str(exc))
        for window in self.windows.values():
            window.end_step(dt)
        self.duration += dt
        self.substeps += 1

    def finish(self, time):
        self.active = False
        if not np.isclose(float(time)-self.start_time, self.duration, atol=1.e-9, rtol=1.e-7):
            for window in self.windows.values():
                window.invalidate("incomplete_physics_sampling_window")
        self.snapshot = dict(schema_version=1, time_s=float(time),
            window_start_s=self.start_time, duration_s=self.duration, physics_substeps=self.substeps,
            solver=self.solver, convention="force_on_sensor_body",
            aggregation="time_weighted_mean_in_instantaneous_sensor_axes",
            configured_usb_bases=sorted(self.base_names),
            loaded_usb_bases=sorted(self.base_names & set(self.env.fixtures)),
            sensors={key: window.result(self.duration) for key, window in self.windows.items()},
            frame_poses_world={name: dict(position_m=actor.pose.p.tolist(),
                                        quaternion_wxyz=actor.pose.q.tolist())
                               for name, actor in self.frames.items()})
        return self.snapshot
