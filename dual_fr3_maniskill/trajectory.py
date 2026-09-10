"""Validated, joint-name-aware sampling of ROS trajectories (no ROS runtime)."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def seconds(stamp) -> float:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("Invalid ROS time/duration")
    return stamp.sec + stamp.nanosec * 1e-9


def tolerances(messages, names, defaults):
    """0 means default and -1 disables a component, as in ros2_control."""
    result = np.tile(np.asarray(defaults, dtype=float), (len(names), 1))
    seen = set()
    for message in messages:
        if message.name not in names or message.name in seen:
            raise ValueError(f"Invalid/duplicate tolerance joint: {message.name}")
        seen.add(message.name)
        index = names.index(message.name)
        for column, field in enumerate(("position", "velocity", "acceleration")):
            value = getattr(message, field)
            if not math.isfinite(value) or (value < 0 and value != -1):
                raise ValueError(f"Invalid {field} tolerance")
            if value != 0:
                result[index, column] = math.inf if value == -1 else value
    return result


@dataclass
class Trajectory:
    names: list[str]
    times: np.ndarray
    positions: list[np.ndarray]
    velocities: list[np.ndarray | None]
    accelerations: list[np.ndarray | None]

    @classmethod
    def from_message(cls, message, expected_names, initial, initial_velocity, limits):
        names = list(message.joint_names)
        if len(names) != len(set(names)) or set(names) != set(expected_names):
            raise ValueError("Trajectory must contain exactly the seven joints of this arm")
        if not message.points:
            raise ValueError("Empty trajectory")
        order = [names.index(name) for name in expected_names]
        times, positions, velocities, accelerations = [], [], [], []
        for point in message.points:
            t = seconds(point.time_from_start)
            if times and t <= times[-1]:
                raise ValueError("Trajectory times must increase strictly")
            values = []
            for label in ("positions", "velocities", "accelerations"):
                vector = getattr(point, label)
                if not vector and label != "positions":
                    values.append(None)
                    continue
                if len(vector) != len(names) or not np.isfinite(vector).all():
                    raise ValueError(f"Invalid trajectory {label}")
                values.append(np.asarray(vector, dtype=float)[order])
            if len(point.effort):
                raise ValueError("Effort trajectories are not supported by the position bridge")
            if values[2] is not None and values[1] is None:
                raise ValueError("Accelerations require velocities")
            for name, value in zip(expected_names, values[0]):
                lower, upper = limits[name][:2]
                if not lower - 1e-6 <= value <= upper + 1e-6:
                    raise ValueError(f"Position outside limits for {name}: {value}")
            times.append(t)
            positions.append(values[0])
            velocities.append(values[1])
            accelerations.append(values[2])
        if times[0] > 0:
            times.insert(0, 0.0)
            positions.insert(0, np.asarray(initial).copy())
            velocities.insert(0, np.asarray(initial_velocity).copy() if velocities[0] is not None else None)
            accelerations.insert(0, np.zeros(len(names)) if accelerations[0] is not None else None)
        return cls(list(expected_names), np.asarray(times), positions, velocities, accelerations)

    @property
    def duration(self):
        return float(self.times[-1])

    def sample(self, elapsed):
        zero = np.zeros(len(self.names))
        if elapsed >= self.duration:
            return self.positions[-1].copy(), zero, zero.copy()
        if elapsed < 0:
            return self.positions[0].copy(), zero, zero.copy()
        i = min(int(np.searchsorted(self.times, elapsed, side="right")) - 1, len(self.times) - 2)
        duration = self.times[i + 1] - self.times[i]
        s = (elapsed - self.times[i]) / duration
        p0, p1 = self.positions[i:i + 2]
        v0, v1 = self.velocities[i:i + 2]
        a0, a1 = self.accelerations[i:i + 2]
        if v0 is None or v1 is None:
            return p0 + s * (p1 - p0), (p1 - p0) / duration, zero
        c0, c1 = p0, v0 * duration
        if a0 is None or a1 is None:
            c2 = 3 * (p1 - p0) - (2 * v0 + v1) * duration
            c3 = 2 * (p0 - p1) + (v0 + v1) * duration
            c4, c5 = zero, zero
        else:
            c2 = a0 * duration**2 / 2
            delta_p = p1 - c0 - c1 - c2
            delta_v = v1 * duration - c1 - 2 * c2
            delta_a = a1 * duration**2 - 2 * c2
            c3 = 10 * delta_p - 4 * delta_v + delta_a / 2
            c4 = -15 * delta_p + 7 * delta_v - delta_a
            c5 = 6 * delta_p - 3 * delta_v + delta_a / 2
        position = c0 + s * (c1 + s * (c2 + s * (c3 + s * (c4 + s * c5))))
        velocity = (c1 + s * (2*c2 + s * (3*c3 + s * (4*c4 + s * 5*c5)))) / duration
        acceleration = (2*c2 + s * (6*c3 + s * (12*c4 + s * 20*c5))) / duration**2
        return position, velocity, acceleration
