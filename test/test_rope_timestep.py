"""Motion-bound enforcement and time accounting for changing contact speeds."""
import math

import pytest

from dual_fr3_maniskill.cable.timestep import RopeStepSchedule


def test_refines_during_control_period_without_residual_or_early_coarsening():
    schedule = RopeStepSchedule(.02, .001)
    timesteps = []
    while schedule.remaining_steps:
        # A contact accelerates, then slows during this same control period.
        limit = .001 if len(timesteps) < 3 else .00009 if len(timesteps) < 12 else .001
        dt = schedule.next_step(limit)
        assert 0 < dt <= limit
        assert dt <= (timesteps[-1] if timesteps else .001)
        timesteps.append(dt)
    assert math.fsum(timesteps) == pytest.approx(.02, abs=1.e-15)
    assert timesteps[-1] == .001/16
    with pytest.raises(RuntimeError, match="completed"):
        schedule.next_step(.001)


def test_next_control_period_grows_by_at_most_two_and_honors_noninteger_frequency():
    schedule = RopeStepSchedule(.02, .0007, previous_dt=.00008)
    timesteps = []
    while schedule.remaining_steps:
        timesteps.append(schedule.next_step(.001))
    assert max(timesteps) <= .00016
    assert math.fsum(timesteps) == pytest.approx(.02, abs=1.e-15)


@pytest.mark.parametrize("limit", [0., -1., float("nan"), float("inf")])
def test_invalid_bound_cannot_advance_time(limit):
    schedule = RopeStepSchedule(.02, .001)
    with pytest.raises(ValueError):
        schedule.next_step(limit)
    assert schedule.remaining_steps == 20
