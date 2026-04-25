"""Trapezoidal velocity profile for 1-D motion."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class TrapezoidalProfile:
    """Trapezoidal velocity profile descriptor."""

    distance: float      # total displacement  (≥ 0)
    v_max: float         # requested max velocity
    a_max: float         # acceleration / deceleration magnitude

    # Derived quantities (computed in __post_init__)
    v_peak: float = 0.0  # actual peak velocity (may be < v_max for short moves)
    t_ramp: float = 0.0  # duration of each accel / decel phase
    t_cruise: float = 0.0  # duration of constant-velocity phase
    t_total: float = 0.0   # total motion time

    def __post_init__(self):
        d, vm, am = self.distance, self.v_max, self.a_max
        assert d >= 0.0 and vm > 0.0 and am > 0.0

        # Distance covered during a full accel + decel (no cruise)
        d_full_ramp = vm * vm / am  # = 2 * (0.5 * a * t_ramp^2)

        if d < 1e-9:
            # Trivial zero-distance move
            object.__setattr__(self, "v_peak", 0.0)
            object.__setattr__(self, "t_ramp", 0.0)
            object.__setattr__(self, "t_cruise", 0.0)
            object.__setattr__(self, "t_total", 0.0)
            return

        if d >= d_full_ramp:
            # Full trapezoidal profile
            t_ramp = vm / am
            d_cruise = d - d_full_ramp
            t_cruise = d_cruise / vm
            v_peak = vm
        else:
            # Triangular profile – can't reach v_max
            v_peak = math.sqrt(d * am)
            t_ramp = v_peak / am
            t_cruise = 0.0

        object.__setattr__(self, "v_peak", v_peak)
        object.__setattr__(self, "t_ramp", t_ramp)
        object.__setattr__(self, "t_cruise", t_cruise)
        object.__setattr__(self, "t_total", 2 * t_ramp + t_cruise)

    # Query

    def evaluate(self, t: float) -> Tuple[float, float]:
        """Return (fraction, velocity_fraction) at time t."""
        if self.t_total <= 0.0:
            return 1.0, 0.0

        t = max(0.0, min(t, self.t_total))
        tr = self.t_ramp

        if t <= tr:
            # Acceleration phase
            vel_frac = t / tr
            pos = 0.5 * self.a_max * t * t
        elif t <= tr + self.t_cruise:
            # Cruise phase
            dt = t - tr
            vel_frac = 1.0
            pos = 0.5 * self.a_max * tr * tr + self.v_peak * dt
        else:
            # Deceleration phase
            dt = t - tr - self.t_cruise
            vel_frac = 1.0 - dt / tr
            pos = (
                0.5 * self.a_max * tr * tr
                + self.v_peak * self.t_cruise
                + self.v_peak * dt
                - 0.5 * self.a_max * dt * dt
            )

        fraction = pos / self.distance if self.distance > 1e-9 else 1.0
        return min(fraction, 1.0), vel_frac

    def velocity_at(self, t: float) -> float:
        """Absolute velocity (m/s or rad/s) at time *t*."""
        _, vf = self.evaluate(t)
        return vf * self.v_peak

    def position_at(self, t: float) -> float:
        """Absolute displacement at time *t*."""
        frac, _ = self.evaluate(t)
        return frac * self.distance
