"""The path guidance follows: one NavIntent as arrays, without ROS.

Tracking is by progression, not by time (guidance.md §6): each tick, the progression moves forward to
the projection of the drone on the path, but never further than a window of
ADVANCE_FACTOR x speed x dt + ADVANCE_MARGIN_M. It never goes back and never jumps over a hairpin
apex or to a later lap, even with the drone off the line, so a drone that slows down (traffic) is
late but skips nothing. The same window as the lap node that flew the 2026-10 SITL campaigns
(nav_stack dev branch, lap.py).

Frame: mavros local ENU (x east, y north, z up), meters, seconds. z here is already in that frame
(guidance adds the home height to the NavPoint z, a height above home).
"""
import math

import numpy as np

ADVANCE_FACTOR = 1.5
ADVANCE_MARGIN_M = 2.0
MIN_SPEED_FOR_WINDOW = 1.0   # m/s, so that the window opens at the start, at rest
GATE_GIVE_UP_M = 30.0        # progression this far past a gate apex: the gate is left behind


class Gate:
    def __init__(self, index, s_apex, x, y, tolerance):
        self.index = index            # waypoint number, for the logs
        self.s_apex = s_apex
        self.xy = np.array([x, y])
        self.tolerance = tolerance
        self.passed = False


class IntentPath:
    def __init__(self, intent_id, points, gates):
        """points: rows (s, x, y, z, vx, vy, vz, ax, ay), the NavPoint order, local frame.
        gates: list of Gate. Points whose s does not increase are dropped."""
        self.intent_id = intent_id
        points = np.asarray(points, float)
        keep = [0] + [i for i in range(1, len(points)) if points[i, 0] > points[i - 1, 0]]
        self.dropped = len(points) - len(keep)
        points = points[keep]
        if len(points) < 2:
            raise ValueError('il faut au moins deux points à abscisse s croissante')
        self.s = points[:, 0]
        self.xyz = points[:, 1:4]
        self.velocity = points[:, 4:7]
        self.accel_xy = points[:, 7:9]

        self.progress = float(self.s[0])   # abscissa of the drone on the path, never goes back
        self.gates = sorted(gates, key=lambda g: g.s_apex)
        self.gate_next = 0                 # index in self.gates of the next gate to pass
        self.gates_passed = 0              # gates passed within their tolerance

    @property
    def length(self):
        return float(self.s[-1])

    @property
    def remaining(self):
        return self.length - self.progress

    @property
    def start(self):
        """(x, y) of the first point."""
        return self.xyz[0, :2]

    @property
    def next_gate(self):
        return self.gates[self.gate_next] if self.gate_next < len(self.gates) else None

    def _segment(self, s):
        """Index i of the segment [s[i], s[i + 1]] that holds s (the first or last one outside)."""
        return min(max(int(np.searchsorted(self.s, s, side='right')) - 1, 0), len(self.s) - 2)

    def sample(self, s):
        """The path at abscissa s, linear interpolation: (pos xyz, vel xyz, acc xyz)."""
        i = self._segment(s)
        f = min(max((s - self.s[i]) / (self.s[i + 1] - self.s[i]), 0.0), 1.0)
        pos = (1.0 - f) * self.xyz[i] + f * self.xyz[i + 1]
        vel = (1.0 - f) * self.velocity[i] + f * self.velocity[i + 1]
        acc = (1.0 - f) * self.accel_xy[i] + f * self.accel_xy[i + 1]
        return pos, vel, np.array([acc[0], acc[1], 0.0])

    def cross_track(self, s, xy):
        """Signed distance from the path at s to xy, left of the direction of travel positive."""
        i = self._segment(s)
        seg = self.xyz[i + 1, :2] - self.xyz[i, :2]
        norm = float(np.hypot(*seg))
        if norm == 0.0:
            return 0.0
        offset = xy - self.sample(s)[0][:2]
        return float(seg[0] * offset[1] - seg[1] * offset[0]) / norm

    def advance(self, xy, speed, dt):
        """Moves the progression forward to the projection of xy on the path, inside the window."""
        i0 = self._segment(self.progress)
        ref_speed = float(np.hypot(self.velocity[i0, 0], self.velocity[i0, 1]))
        s_max = (self.progress + ADVANCE_FACTOR * max(ref_speed, speed, MIN_SPEED_FOR_WINDOW) * dt
                 + ADVANCE_MARGIN_M)
        i1 = min(int(np.searchsorted(self.s, s_max, side='right')), len(self.s) - 1)
        xy_ref = self.xyz[:, :2]
        k = i0 + int(np.argmin(np.hypot(*(xy_ref[i0:i1 + 1] - xy).T)))
        # The nearest point of the window, then the projection on the segments on each side of it
        best_d, best_s = math.inf, self.s[k]
        for a in (k - 1, k):
            if a < i0 or a + 1 > i1:
                continue
            seg = xy_ref[a + 1] - xy_ref[a]
            length2 = float(seg @ seg)
            u = 0.0 if length2 == 0.0 else min(max(float((xy - xy_ref[a]) @ seg) / length2, 0.0), 1.0)
            d = float(np.hypot(*(xy_ref[a] + u * seg - xy)))
            if d < best_d:
                best_d, best_s = d, self.s[a] + u * (self.s[a + 1] - self.s[a])
        self.progress = min(max(self.progress, best_s), s_max, self.length)

    def update_gates(self, xy):
        """Marks the gates passed (xy within their tolerance) and moves past the gates passed or left
        behind. Returns the gates left behind on this call, for the logs."""
        # Every gate whose apex is within GATE_GIVE_UP_M ahead is watched: a missed gate does not
        # hide the next one
        for gate in self.gates[self.gate_next:]:
            if self.progress < gate.s_apex - GATE_GIVE_UP_M:
                break
            if not gate.passed and np.hypot(*(xy - gate.xy)) <= gate.tolerance:
                gate.passed = True
                self.gates_passed += 1
        missed = []
        while self.gate_next < len(self.gates):
            gate = self.gates[self.gate_next]
            if not gate.passed:
                if self.progress <= gate.s_apex + GATE_GIVE_UP_M:
                    break
                missed.append(gate)
            self.gate_next += 1
        return missed
