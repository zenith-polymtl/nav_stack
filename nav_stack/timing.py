"""Traffic avoidance by timing alone, without ROS: stay on the path, choose when to be where.

The lateral planner (local_planner.py) cannot wait behind an aircraft: its convex form fixes the
side on which it passes each one. Next to a waypoint, with an aircraft holding near it, it then
misses the waypoint (v5 bench, 2026-11-11: 21 of 26 missed waypoints of min15, in conflict). Here the
drone stays on the path and only its speed changes. Each candidate holds a reduced speed (a fraction
of the path's speeds, or a stop) for a while, then flies the path's speeds for PASS_S: it is valid
only if the whole of it stays clear of every predicted aircraft, so that a wait is never chosen just
because the conflict lies beyond the horizon (which waits forever). Of the valid candidates, the one
furthest along the path at the common end time is flown. None: the lateral planner takes over.

The prediction must hold over the whole candidate (up to max(HOLDS_S) + PASS_S): exact for the
learned loops (traffic.py), approximate for the constant turn of a track whose loop is not known.

Frames: as local_planner (2D, m, s).
"""
from dataclasses import dataclass

import numpy as np

DT = 0.5
PASS_S = 10.0               # after the hold, the path's speeds for this long, clear of conflict (about 190 m: past a
                            # holding aircraft; conflicts further on are left to the next replans)
SCALES = (0.0, 0.25, 0.5, 0.75)                   # speeds held, as fractions of the path's speeds
HOLDS_S = (2.0, 4.0, 6.0, 8.0, 10.0, 13.0, 16.0)  # held this long
HORIZON_S = max(HOLDS_S) + 20.0   # candidates are compared by their progression at this time
STEPS = int(round(HORIZON_S / DT))
ACCEL = 4.0                 # m/s^2, speeding up and braking along the path
MARGIN_M = 12.0             # beyond the keep-away, at every step


@dataclass
class Profile:
    t0: float              # caller's clock, s
    speed: np.ndarray      # (n,) speed along the path at each step
    theta: np.ndarray      # (n,) progression at each step
    name: str

    def at(self, t):
        """(speed, acceleration along the path) at time t."""
        tau = max(t - self.t0, 0.0) / DT
        i = min(int(tau), len(self.speed) - 2)
        f = min(tau - i, 1.0)
        v = self.speed[i] * (1 - f) + self.speed[i + 1] * f
        return float(v), float((self.speed[i + 1] - self.speed[i]) / DT)

    def covers(self, t):
        return 0.0 <= t - self.t0 < (len(self.speed) - 2) * DT


def _fly(ref, theta0, speed0, scales, holds):
    """(progressions, speeds), arrays (candidates, STEPS + 1) every DT up to HORIZON_S: scale x the
    path's speeds for hold s, then the path's speeds, one candidate per (scale, hold)."""
    scales, holds = np.asarray(scales, float), np.asarray(holds, float)
    theta = np.empty((len(scales), STEPS + 1))
    v = np.empty_like(theta)
    theta[:, 0], v[:, 0] = theta0, speed0
    for i in range(STEPS):
        target = np.where(i * DT < holds, scales, 1.0) * ref.at(theta[:, i])[2]
        v[:, i + 1] = np.maximum(np.clip(target, v[:, i] - ACCEL * DT, v[:, i] + ACCEL * DT), 0.0)
        theta[:, i + 1] = theta[:, i] + 0.5 * (v[:, i] + v[:, i + 1]) * DT
    return theta, v


def choose(ref, theta0, speed0, aircraft, t_now):
    """The Profile to fly, or None. ref: local_planner.Reference; theta0: progression; speed0: speed
    along the path now; aircraft: [(centers (STEPS + 1, 2) every DT from now, keep-away m)]."""
    candidates = [(1.0, 0.0)] + [(s, h) for s in SCALES for h in HOLDS_S]
    scales, holds = zip(*candidates)
    theta, v = _fly(ref, theta0, speed0, scales, holds)
    xy = ref.at(theta.ravel())[0].reshape(len(candidates), STEPS + 1, 2)
    ok = np.ones(len(candidates), bool)
    for centers, keep_away in aircraft:
        clear = np.hypot(*(centers[None] - xy).transpose(2, 0, 1)) >= keep_away + MARGIN_M
        ends = np.round((np.asarray(holds) + PASS_S) / DT).astype(int) + 1     # the hold, then the pass
        ok &= np.array([clear[k, :ends[k]].all() for k in range(len(candidates))])
    if not ok.any():
        return None
    k = int(np.argmax(np.where(ok, theta[:, -1], -np.inf)))
    name = 'chemin' if holds[k] == 0 else f'{scales[k]:.0%} pendant {holds[k]:.0f} s'
    return Profile(t_now, v[k], theta[k], name)
