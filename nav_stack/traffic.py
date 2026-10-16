"""The traffic as the drone knows it, without ROS: the history and prediction of each aircraft.

The competition server sends each aircraft about once a second (topics.TRAFFIC, TrafficArray). For
each aircraft we keep its last HISTORY updates, estimate from them a velocity and a turn rate (finite
differences), and predict where it will be over the next seconds at constant speed and turn rate:
the centers of the circles the local planner keeps away from.

The prediction is exact for an aircraft at constant speed and turn rate (the server's model: transits
and holding circles), from the time of the position, which comp_client estimates (reception minus the
measured link latency): the velocity is the tangent at the last update, the turn is integrated in
closed form, and the age is not rounded. The previous version (chord velocity, step integration, age
rounded to the step) was 4 to 10 m off after 10 s on a holding circle; laps_navigation_sitl_research.md
§6.7, point 2, and laps_bench/traffic_legacy.py.

Frames: ENU, meters (x east, y north: the course sent by the server is read that way); the lap
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


def predict(position, velocity, omega, age, horizon_s, dt):
    """Constant speed and turn rate from the last known position (age s old): the positions from
    now to now + horizon_s, every dt, as an array (n, 2). velocity is the one at that position."""
    p, v = np.asarray(position, float), np.asarray(velocity, float)
    perp = np.array([-v[1], v[0]])               # v turned 90 degrees counterclockwise
    tau = max(age, 0.0) + np.arange(int(round(horizon_s / dt)) + 1) * dt
    if abs(omega) < MIN_TURN_RATE:
        return p + tau[:, None] * v
    along, across = np.sin(omega * tau) / omega, (1.0 - np.cos(omega * tau)) / omega
    return p + along[:, None] * v + across[:, None] * perp


@dataclass
class Update:
    stamp: float            # s, time of the measurement
    xy: np.ndarray          # position
    altitude: float         # m above ground
    speed: float            # m/s, as sent by the server
    course_deg: float       # deg true, as sent by the server
    hka: float              # horizontal keep-away, m
    vka: float              # vertical keep-away, m


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
        """(velocity at the last update, turn rate rad/s counterclockwise) from the last updates; with a
        single one, the velocity the server sent and no turn. The finite difference of the last two
        updates is the chord velocity, the one at mid-interval: turned by omega x half the interval,
        it is the tangent at the last update."""
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
                omega = angle / ((u[-1].stamp - u[-3].stamp) / 2)   # between the two midpoints
        if omega:
            c, s = math.cos(omega * interval / 2), math.sin(omega * interval / 2)
            v1 = np.array([c * v1[0] - s * v1[1], s * v1[0] + c * v1[1]])
        return v1, omega

    def predict(self, now, horizon_s, dt):
        v, omega = self.velocity_and_turn()
        return predict(self.last.xy, v, omega, self.age(now), horizon_s, dt)
