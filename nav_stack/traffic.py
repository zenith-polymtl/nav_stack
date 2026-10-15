"""The traffic as the drone knows it, without ROS: the history and prediction of each aircraft.

The competition server sends each aircraft about once a second (topics.TRAFFIC, TrafficArray). For
each aircraft we keep its last HISTORY updates, estimate from them a velocity and a turn rate (finite
differences), and predict where it will be over the next seconds at constant speed and turn rate:
the centers of the circles the local planner keeps away from.

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


def predict(position, velocity, omega, age, horizon_s, dt):
    """Constant speed and turn rate from the last known position (age s old): the positions from
    now to now + horizon_s, every dt, as an array (n, 2). The age is rounded to a multiple of dt."""
    out, q, w = [], np.array(position, float), np.array(velocity, float)
    skip = int(round(max(age, 0.0) / dt))       # steps already flown since the update
    c, s = math.cos(omega * dt), math.sin(omega * dt)
    for i in range(skip + int(round(horizon_s / dt)) + 1):
        if i >= skip:
            out.append(q.copy())
        w = np.array([c * w[0] - s * w[1], s * w[0] + c * w[1]])
        q = q + w * dt
    return np.array(out)


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
        """(velocity, turn rate rad/s counterclockwise) from the last updates; with a single one,
        the velocity the server sent and no turn."""
        u = list(self.updates)
        if len(u) == 1:
            c = math.radians(u[0].course_deg)
            return u[0].speed * np.array([math.sin(c), math.cos(c)]), 0.0
        v1 = (u[-1].xy - u[-2].xy) / (u[-1].stamp - u[-2].stamp)
        omega = 0.0
        if len(u) >= 3:
            v0 = (u[-2].xy - u[-3].xy) / (u[-2].stamp - u[-3].stamp)
            if np.linalg.norm(v0) > MIN_SPEED_FOR_TURN and np.linalg.norm(v1) > MIN_SPEED_FOR_TURN:
                angle = math.atan2(v0[0] * v1[1] - v0[1] * v1[0], float(v0 @ v1))
                omega = angle / ((u[-1].stamp - u[-3].stamp) / 2)   # between the two midpoints
        return v1, omega

    def predict(self, now, horizon_s, dt):
        v, omega = self.velocity_and_turn()
        return predict(self.last.xy, v, omega, self.age(now), horizon_s, dt)
