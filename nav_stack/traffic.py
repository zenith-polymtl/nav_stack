"""The traffic as the drone knows it, without ROS: the history and prediction of each aircraft.

The competition server sends each aircraft about once a second (topics.TRAFFIC, TrafficArray). For
each aircraft we keep its last HISTORY updates, estimate a velocity and a turn rate from them, and
predict where it will be over the next seconds at constant speed and turn rate: the centers of the
circles the local planner keeps away from.

The prediction is exact for an aircraft at constant speed and turn rate (the server's model: transits
and holding circles), from the time of the position, which comp_client estimates (reception minus the
measured link latency). laps_navigation_sitl_research.md §6.7, point 2: the previous version (chord
velocity, step integration, rounded age) was 4 to 10 m off after 10 s on a holding circle.

Learned loops. The server's traffic is deterministic and loops forever (a transit runs its polyline
again and again, segments of whole seconds; a holding circle turns at constant rate), and comp_client
receives it from its connection, minutes before takeoff. Each Track keeps a journal of its updates;
once the last LOOP_CHECK updates repeat updates exactly one period earlier (an integer number of
seconds, LOOP_MATCH_M), the period is the aircraft's loop and the prediction replays the journal one
period back: exact at any horizon, corners of transits included (the corner between two updates is
rebuilt from the two courses). A replay that misses a new update by more than LOOP_MATCH_M is dropped,
and the constant turn prediction takes over until a loop is found again.

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
JOURNAL_S = 900.0          # s of updates kept to find loops (the longest loop that can be learned)
MIN_LOOP_S = 20.0          # s: shorter periods are not searched (a holding circle is exact without a loop)
LOOP_CHECK = 10            # updates that must repeat one period earlier
LOOP_MATCH_M = 2.0         # m: a repeat, and the tolerance of a replay against a new update
STAMP_MATCH_S = 0.3        # s: two stamps one period apart, within this
SAME_COURSE_DEG = 1.0      # below this course change between two updates, no corner between them
CIRCLE_SPREAD_DEG = 0.5    # course changes this steady (and not zero) over a loop: a holding circle, left to
                           # the constant turn (exact; its period is not a whole number of seconds)


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

    def __init__(self, name, learn_loops=True):
        self.name = name
        self.updates = deque(maxlen=HISTORY)
        self.learn_loops = learn_loops
        self.journal = deque()        # every update of the last JOURNAL_S
        self.loop = None              # Loop once learned

    def add(self, update):
        """Keeps the update; a repeat is ignored, a gap longer than STALE_S restarts the history."""
        if self.updates:
            last = self.updates[-1].stamp
            if update.stamp <= last:
                return
            if update.stamp - last > STALE_S:
                self.updates.clear()
        self.updates.append(update)
        if not self.learn_loops:
            return
        self.journal.append(update)
        while self.journal[0].stamp < update.stamp - JOURNAL_S:
            self.journal.popleft()
        if self.loop is not None and np.hypot(*(self.loop.at(np.array([update.stamp]))[0] - update.xy)) > LOOP_MATCH_M:
            self.loop = None          # the aircraft left its loop (or the site changed)
        if self.loop is None:
            self.loop = find_loop(list(self.journal))

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
        if self.loop is not None:
            return self.loop.at(now + np.arange(int(round(horizon_s / dt)) + 1) * dt)
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


def _direction(course_deg):
    c = math.radians(course_deg)
    return np.array([math.sin(c), math.cos(c)])


class Loop:
    """One period of an aircraft's journal, replayed: positions at any time, corners rebuilt."""

    def __init__(self, updates, period):
        self.period = period
        t, xy = [], []
        for a, b in zip(updates, updates[1:]):
            t.append(a.stamp)
            xy.append(a.xy)
            corner = _corner(a, b)
            if corner is not None:
                t.append(corner[0])
                xy.append(corner[1])
        t.append(updates[-1].stamp)
        xy.append(updates[-1].xy)
        self.t = np.array(t)
        self.xy = np.array(xy)

    def at(self, times):
        """Positions (n, 2) at the given times."""
        phase = self.t[0] + np.mod(np.asarray(times, float) - self.t[0], self.period)
        return np.column_stack([np.interp(phase, self.t, self.xy[:, 0]), np.interp(phase, self.t, self.xy[:, 1])])


def _corner(a, b):
    """(time, position) of the corner between updates a and b, or None: same course, or no sensible corner."""
    turn = (b.course_deg - a.course_deg + 180.0) % 360.0 - 180.0
    if abs(turn) < SAME_COURSE_DEG:
        return None
    da, db = _direction(a.course_deg), _direction(b.course_deg)
    cross = da[0] * db[1] - da[1] * db[0]
    if abs(cross) < 1e-6:          # reversal (180 degrees): the corner is beyond both, along a's course
        span = float(np.hypot(*(b.xy - a.xy)))
        ahead = float((b.xy - a.xy) @ da)
        ka = (span + ahead) / 2 if ahead > 0 else span / 2
        point = a.xy + da * ka
        kb = float(np.hypot(*(b.xy - point)))
    else:
        rel = b.xy - a.xy
        ka = (rel[0] * db[1] - rel[1] * db[0]) / cross      # a.xy + ka da = b.xy - kb db
        kb = -(rel[0] * da[1] - rel[1] * da[0]) / cross
        if ka < 0 or kb < 0:
            return None
        point = a.xy + ka * da
    if ka + kb <= 0:
        return None
    return a.stamp + (b.stamp - a.stamp) * ka / (ka + kb), point


def find_loop(journal):
    """The aircraft's Loop if the last LOOP_CHECK updates repeat one period earlier, else None."""
    if len(journal) < LOOP_CHECK + 2:
        return None
    last = journal[-1]
    stamps = np.array([u.stamp for u in journal])
    xy = np.array([u.xy for u in journal])
    candidates = np.flatnonzero((stamps <= last.stamp - MIN_LOOP_S) & (np.hypot(*(xy - last.xy).T) <= LOOP_MATCH_M))
    for i in candidates[::-1]:     # the most recent repeat first: the shortest period
        period = last.stamp - stamps[i]
        ok = True
        for u in list(journal)[-LOOP_CHECK:]:
            j = int(np.argmin(np.abs(stamps - (u.stamp - period))))
            if abs(stamps[j] - (u.stamp - period)) > STAMP_MATCH_S or np.hypot(*(xy[j] - u.xy)) > LOOP_MATCH_M:
                ok = False
                break
        if ok:
            first = int(np.argmin(np.abs(stamps - (last.stamp - period))))
            cycle = list(journal)[first:]
            turns = np.array([(b.course_deg - a.course_deg + 180.0) % 360.0 - 180.0 for a, b in zip(cycle, cycle[1:])])
            if np.std(turns) < CIRCLE_SPREAD_DEG and abs(np.mean(turns)) > SAME_COURSE_DEG:
                return None
            return Loop(cycle, period)
    return None
