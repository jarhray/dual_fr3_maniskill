"""Aligned adaptive substeps: refine within a control period, grow between it."""
import math


class RopeStepSchedule:
    """Tile a control period exactly with dyadic subdivisions of a base step.

    Each step obeys the caller's freshly calculated motion bound. Refinement
    halves the pending cells, so no tiny remainder is appended at the end.
    Coarsening is deferred to the next control period and limited to a factor
    of two relative to the last physical step.
    """

    def __init__(self, period, maximum, previous_dt=None):
        if any(not math.isfinite(x) or x <= 0 for x in (period, maximum)):
            raise ValueError("Step period and maximum must be positive and finite")
        self.remaining_steps = max(1, math.ceil(period/maximum))
        self.dt = period/self.remaining_steps
        if previous_dt is not None:
            self._refine(2*previous_dt)

    def _refine(self, limit):
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError("Step limit must be positive and finite")
        while self.dt > limit:
            self.dt /= 2
            self.remaining_steps *= 2

    def next_step(self, limit):
        if self.remaining_steps == 0:
            raise RuntimeError("Control period already completed")
        self._refine(limit)
        self.remaining_steps -= 1
        return self.dt
