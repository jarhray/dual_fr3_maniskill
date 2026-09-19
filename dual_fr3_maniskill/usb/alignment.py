"""Bounded USB-tip pose corrections; no ROS, physics mutation or global planner."""
from dataclasses import asdict, dataclass, fields

import numpy as np
from transforms3d.axangles import axangle2mat, mat2axangle

from dual_fr3_maniskill.usb.geometry import TIP, USB_IN_SOCKET


@dataclass(frozen=True)
class AlignmentLimits:
    capture_translation_m: float = .002
    capture_angle_rad: float = np.deg2rad(3.)
    position_tolerance_m: float = .0001
    angle_tolerance_rad: float = np.deg2rad(.5)
    speed_m_s: float = .001
    angular_speed_rad_s: float = .03
    gain_per_s: float = 2.
    max_travel_m: float = .006
    max_rotation_rad: float = np.deg2rad(6.)
    hold_s: float = .3
    timeout_s: float = 20.
    blocked_s: float = 3.
    collision_timeout_s: float = 3.  # wall time, unlike alignment dwell/timeout
    collision_joint_step_rad: float = .001

    @classmethod
    def read(cls, config, insertion):
        result = cls(**{f.name: config[f.name] for f in fields(cls) if f.name in config})
        if any(isinstance(v, bool) or not np.isfinite(v) or v <= 0 for v in asdict(result).values()):
            raise ValueError('Alignment limits must be positive finite SI values')
        if result.capture_translation_m >= insertion.preinsert_m:
            raise ValueError('Alignment capture range must remain outside the socket')
        if (result.position_tolerance_m > min(insertion.lateral_tolerance_m, insertion.tracking_limit_m)
                or result.angle_tolerance_rad > insertion.angle_tolerance_rad):
            raise ValueError('Alignment tolerances must not exceed insertion tolerances')
        return result


def error_norm(observation, preinsert):
    return float(np.hypot(observation['depth_m'] + preinsert, observation['lateral_error_m']))


def rotation_distance(a, b):
    return float(np.arccos(np.clip((np.trace(a.T @ b) - 1.) / 2., -1., 1.)))


def correction_goal(base, tcp, usb, hole, preinsert, limits, dt):
    """Move/rotate about the measured tip, then recover the TCP using live grasp.

    Both translation and rotation are bounded per simulation control tick. The
    TCP translation includes the lever-arm compensation for a rotating plug.
    """
    tip = usb[:3, 3] + usb[:3, :3] @ TIP
    target_tip = base[:3, 3] + base[:3, :3] @ (np.asarray(hole) + [preinsert, 0., 0.])
    delta = target_tip - tip
    distance = np.linalg.norm(delta)
    if distance:
        delta *= min(1., limits.gain_per_s * dt, limits.speed_m_s * dt / distance)
    rotation = base[:3, :3] @ USB_IN_SOCKET @ usb[:3, :3].T
    axis, angle = mat2axangle(rotation)
    angle = float(np.clip(angle * min(1., limits.gain_per_s * dt),
                          -limits.angular_speed_rad_s * dt, limits.angular_speed_rad_s * dt))
    corrected = np.eye(4)
    corrected[:3, :3] = axangle2mat(axis, angle) @ usb[:3, :3]
    corrected[:3, 3] = tip + delta - corrected[:3, :3] @ TIP
    return corrected @ np.linalg.inv(np.linalg.inv(tcp) @ usb)
