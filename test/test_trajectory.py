from itertools import combinations
from types import SimpleNamespace as NS

import numpy as np
import pytest

from dual_fr3_maniskill.collision import collision_cliques
from dual_fr3_maniskill.trajectory import Trajectory, tolerances


NAMES = [f"left_fr3_joint{i}" for i in range(1, 8)]
LIMITS = {name: (-3.0, 3.0, 80.0) for name in NAMES}


def knot(t, p, v=(), a=()):
    return NS(time_from_start=NS(sec=int(t), nanosec=round((t-int(t))*1e9)),
              positions=list(p), velocities=list(v), accelerations=list(a), effort=[])


def trajectory(points, names=NAMES):
    return Trajectory.from_message(NS(joint_names=names, points=points), NAMES,
                                   np.zeros(7), np.zeros(7), LIMITS)


def test_shuffled_joints_and_implicit_start():
    result = trajectory([knot(2, np.arange(7)/3)], list(reversed(NAMES)))
    np.testing.assert_allclose(result.sample(1)[0], np.arange(7)[::-1]/6)
    np.testing.assert_allclose(result.sample(10)[0], np.arange(7)[::-1]/3)
    np.testing.assert_allclose(result.sample(10)[1], 0)


@pytest.mark.parametrize("derivatives", [False, True])
def test_smooth_interpolation_respects_boundary_conditions(derivatives):
    zero = np.zeros(7)
    acceleration = zero if derivatives else []
    result = trajectory([knot(0, zero, zero, acceleration), knot(2, np.ones(7), zero, acceleration)])
    np.testing.assert_allclose(result.sample(1)[0], 0.5)
    np.testing.assert_allclose(result.sample(0)[1], 0)
    np.testing.assert_allclose(result.sample(2 - 1e-7)[1], 0, atol=1e-6)
    if derivatives:
        np.testing.assert_allclose(result.sample(0)[2], 0)


@pytest.mark.parametrize("points", [[], [knot(0, [float('nan')]*7)],
    [knot(0, [4]*7)], [knot(1, [0]*7), knot(1, [1]*7)], [knot(1, [0]*6)]])
def test_invalid_trajectories_rejected(points):
    with pytest.raises(ValueError):
        trajectory(points)


def test_tolerance_defaults_and_disabled_components():
    limits = tolerances([NS(name=NAMES[0], position=-1, velocity=0.1, acceleration=0)],
                        NAMES, [0.2, 0.02, np.inf])
    assert np.isinf(limits[0, 0]) and limits[0, 1] == 0.1
    assert limits[1, 0] == 0.2


def test_collision_masks_never_disable_an_unspecified_pair():
    allowed = {frozenset(pair) for pair in [("a", "b"), ("b", "c"), ("a", "c"), ("c", "d")]}
    groups = collision_cliques(allowed, {"a", "b", "c", "d"})
    covered = {frozenset(pair) for group in groups for pair in combinations(group, 2)}
    assert covered == allowed
    assert len(groups) == 2
