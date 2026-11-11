"""The traffic as the drone knows it, without ROS: the history and prediction of each aircraft.

The competition server sends each aircraft about once a second (topics.TRAFFIC, TrafficArray). For
each aircraft we keep its last HISTORY updates, estimate a velocity and a turn rate from them, and
predict where it will be over the next seconds at constant speed and turn rate: the centers of the
circles the local planner keeps away from.

The prediction is exact for an aircraft at constant speed and turn rate (the server's model: transits
and holding circles), from the time of the position, which comp_client estimates (reception minus the
measured link latency). laps_navigation_sitl_research.md §6.7, point 2: the previous version (chord
velocity, step integration, rounded age) was 4 to 10 m off after 10 s on a holding circle.

Frames: ENU, meters (x east, y north: the course sent by the server is read that way); the guidance
node uses the mavros local frame. Altitudes: meters above ground.
"""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

HISTORY = 3                # updates kept per aircraft
STALE_S = 3.0              # an aircraft without an update for this long is ignored, its history restarts
MIN_SPEED_FOR_TURN = 0.5   # m/s: below this, no turn rate estimate
MIN_TURN_RATE = 1e-6       # rad/s: below this, a straight line
MAX_ARC_CORRECTION = math.pi / 4   # rad: past this half-turn between updates, the chord is not lengthened


@dataclass
class Update:
    stamp: float                  # s, time of the measurement
    xy: np.ndarray                # position
    altitude: float               # m above ground
    speed: float                  # m/s, as sent by the server
    course_deg: float             # deg true, as sent by the server
    horizontal_keep_away: float   # m
    vertical_keep_away: float     # m


class Track:
    """One aircraft: its last HISTORY updates."""

    def __init__(self, name):
        self.name = name
        self.updates = deque(maxlen=HISTORY)

    def add(self, update):
        """Keeps the update; a repeat is ignored, a gap longer than STALE_S restarts the history."""
        if self.updates:
            last = self.updates[-1].stamp
            if update.stamp <= last:
                return
            if update.stamp - last > STALE_S:
                self.updates.clear()
        self.updates.append(update)

    @property
    def last(self):
        return self.updates[-1]

    def age(self, now):
        return now - self.last.stamp

    def velocity_and_turn(self):
        """(velocity at the last update, turn rate rad/s counterclockwise).

        One update: the velocity the server sent, no turn. Two or more: the finite difference of the
        last two is the chord velocity, along the velocity at mid-interval; three: the angle between
        the last two chords, over the time between their midpoints, is the turn rate. On an arc, the
        chord is shorter than the arc by sin(h) / h, h = omega x half the interval: the chord
        velocity divided by that (for |h| < MAX_ARC_CORRECTION) and turned by h is the velocity at
        the last update."""
        u = list(self.updates)
        if len(u) == 1:
            c = math.radians(u[0].course_deg)
            return u[0].speed * np.array([math.sin(c), math.cos(c)]), 0.0
        interval = u[-1].stamp - u[-2].stamp
        v1 = (u[-1].xy - u[-2].xy) / interval
        omega = 0.0
        if len(u) >= 3:
            v0 = (u[-2].xy - u[-3].xy) / (u[-2].stamp - u[-3].stamp)
            if np.linalg.norm(v0) > MIN_SPEED_FOR_TURN and np.linalg.norm(v1) > MIN_SPEED_FOR_TURN:
                angle = math.atan2(v0[0] * v1[1] - v0[1] * v1[0], float(v0 @ v1))
                omega = angle / ((u[-1].stamp - u[-3].stamp) / 2)
        if omega:
            h = omega * interval / 2
            c, s = math.cos(h), math.sin(h)
            arc = h / s if abs(h) < MAX_ARC_CORRECTION else 1.0   # noisy close updates: h near pi, h / s unbounded
            v1 = arc * np.array([c * v1[0] - s * v1[1], s * v1[0] + c * v1[1]])
        return v1, omega

    def predict(self, now, horizon_s, dt):
        """Predicted positions from now to now + horizon_s, every dt: array (n, 2)."""
        velocity, omega = self.velocity_and_turn()
        tau = max(self.age(now), 0.0) + np.arange(int(round(horizon_s / dt)) + 1) * dt
        return _constant_turn(self.last.xy, velocity, omega, tau)


def _constant_turn(position, velocity, omega, tau):
    """Positions tau seconds after (position, velocity), at constant speed and turn rate omega."""
    p, v = np.asarray(position, float), np.asarray(velocity, float)
    if abs(omega) < MIN_TURN_RATE:
        return p + tau[:, None] * v
    perp = np.array([-v[1], v[0]])               # v turned 90 degrees counterclockwise
    along, across = np.sin(omega * tau) / omega, (1.0 - np.cos(omega * tau)) / omega
    return p + along[:, None] * v + across[:, None] * perp
