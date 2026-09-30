"""Time-based scheduling helpers (Webots-independent).

Periods are defined in seconds, never as step counts, because the official
worlds use a 64 ms basicTimeStep and some test worlds 32 ms
(docs/research/10_FINAL_ARCHITECTURE.md §2.2).
"""

import math


def period_to_steps(period_s, timestep_ms):
    """Number of basic steps (>= 1) closest to ``period_s`` seconds."""
    if timestep_ms <= 0:
        raise ValueError("timestep_ms must be > 0")
    return max(1, int(round(period_s * 1000.0 / timestep_ms)))


def sensor_period_ms(period_s, timestep_ms):
    """Sensor enable() period [ms]: an integer multiple of the basic time step."""
    return period_to_steps(period_s, timestep_ms) * int(timestep_ms)


class Periodic:
    """``due(now)`` is True at most once per ``period`` seconds of simulation time.

    The next deadline is advanced from the previous deadline (no drift). If the
    caller fell behind by more than one period, missed runs are counted instead
    of being executed in a burst.
    """

    def __init__(self, period_s):
        if period_s <= 0:
            raise ValueError("period_s must be > 0")
        self.period = float(period_s)
        self.next_time = None
        self.missed = 0

    def due(self, now):
        # Small tolerance so a 0.128 s period on a 64 ms step does not slip a
        # step because of float rounding in robot.getTime().
        eps = 1e-6
        if self.next_time is None:
            self.next_time = now + self.period
            return True
        if now + eps < self.next_time:
            return False
        behind = int(math.floor((now + eps - self.next_time) / self.period))
        self.missed += behind
        self.next_time += (behind + 1) * self.period
        return True


class StepTimer:
    """Collects controller step durations [s] and reports median / p95 / max."""

    def __init__(self):
        self.samples = []

    def add(self, duration_s):
        self.samples.append(float(duration_s))

    def summary(self):
        """(median, p95, max, count) in seconds, or None if empty. Clears the samples."""
        if not self.samples:
            return None
        data = sorted(self.samples)
        n = len(data)
        self.samples = []
        return (data[n // 2], data[int(math.floor(0.95 * (n - 1)))], data[-1], n)
