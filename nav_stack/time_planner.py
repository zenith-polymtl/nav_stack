"""Time planner: keep the racing line, choose only the speed along it, over a long horizon. No ROS.

Why: with waypoints aimed at 5 m, leaving the line costs waypoints; and an aircraft in a holding
pattern flies a circle that traffic.py predicts exactly, so a free passage is known long in advance.
On 74 random encounters (laps_navigation_sitl_research.md §6.10), flying the line at the right
speed, with the local planner as a fallback, touches 14 % of them and leaves 2 waypoints beyond 5 m,
against 42 % and 17 for the local planner alone with a 4 m gate, for about 18 s more.

TimePlanner.plan(): a reachable set on a grid (abscissa every DS_M, speed every V_STEP, time every
DT_S), from the present state; at each step the speed changes by at most A_MAX x DT_S and stays below
the reference's; the point of the line must stay out of every predicted keep-away circle grown by the
margin. The goal is the most advanced state at the last step; a backward pass keeps the states from
which it stays reachable, and a forward pass takes the most advanced of them at each step: the
profile slows down only for what forces it to (the first step, the only one flown before the next
replan, would otherwise slow down for nothing). If the set empties before the end of the horizon
(an aircraft coming along the line: waiting is not enough), plan() returns None.

TimeGuide: what the lap node runs every REPLAN_S, as flown on the bench (strategy T9 of §6.10):
  - obstacles: every aircraft in vertical overlap; a holding aircraft (turn rate above LOITER_OMEGA)
    over the whole horizon, a transit only over TRANSIT_TRUST_S (it may turn or reverse; beyond, a
    straight line made up conflicts and delays that put the drone in the wrong place);
  - start: the state of the profile being flown if it sticks to the drone, else the drone's
    progression and its speed ALONG the line (a drone flying sideways does not progress);
  - margin ladder MARGINS_M: a profile planned exactly at the edge of the margin becomes impossible at
    the next traffic update; less margin is accepted before handing over, down to 2 m (at 0 m, the
    tracking error was enough to enter, on the bench and through MAVROS);
  - stall: a profile progressing less than STALL_M over the horizon (an aircraft circling on the line)
    is dropped, so that the local planner goes around;
  - after any hand-over, the local planner keeps control for HOLD_S (no chattering).
setpoint() turns the profile into the lap setpoint (position, velocity, acceleration, 2D).
"""
import time
from dataclasses import dataclass

import numpy as np

HORIZON_S = 60.0
DT_S = 0.5
DS_M = 0.25
V_STEP = 1.0
A_MAX = 3.0                  # m/s², along the line (the rest for the turns)
LOITER_OMEGA = 0.05          # rad/s: above, an aircraft circling (predictable over the whole horizon)
MIN_SPEED_CAP = 2.0          # m/s: allowed everywhere, so that the profile can leave a standstill
# TimeGuide
REPLAN_S = 1.0
MARGINS_M = (6.0, 4.0, 2.0)  # keep-away + this, tried in turn; not 0: the tracking error then enters (§6.10)
TRANSIT_TRUST_S = 15.0
STALL_M = 30.0
HOLD_S = 5.0
VERTICAL_MARGIN_M = 13.0     # aircraft further than vka + this in altitude are ignored (the drone may climb)
CONTINUITY_M = 8.0           # the profile flown is the start state while within this of the progression


@dataclass
class Profile:
    t0: float
    s: np.ndarray        # abscissa at each step
    v: np.ndarray        # speed at each step
    delayed: bool        # the profile goes below the reference speed somewhere

    def at(self, t):
        """(s, v, a) at time t (linear between steps; beyond the last step, the last one)."""
        x = (t - self.t0) / DT_S
        i = int(np.clip(np.floor(x), 0, len(self.s) - 2))
        f = float(np.clip(x - i, 0.0, 1.0))
        a = (self.v[i + 1] - self.v[i]) / DT_S
        return self.s[i] + f * (self.s[i + 1] - self.s[i]), self.v[i] + f * (self.v[i + 1] - self.v[i]), a


class TimePlanner:
    def __init__(self, s, xy, velocity, ds=DS_M, v_step=V_STEP):
        self.ds, self.v_step = ds, v_step        # grid; ds = v_step x DT_S / 2 keeps integer shifts
        self.s = np.asarray(s, float)
        self.xy = np.asarray(xy, float)
        self.v_ref = np.linalg.norm(velocity, axis=1)
        self.solve_times = []

    def plan(self, t0, s0, v0, obstacles, margin):
        """obstacles: [(centers (n, 2) every DT_S from t0, keep-away radius)]. A Profile, or None."""
        start = time.perf_counter()
        steps = int(round(HORIZON_S / DT_S))
        reach = min(self.s[-1], s0 + self.v_ref.max() * HORIZON_S + 5.0)
        DS, VS = self.ds, self.v_step
        ns = int(np.ceil((reach - s0) / DS)) + 1
        gs = s0 + np.arange(ns) * DS
        pts = np.stack([np.interp(gs, self.s, self.xy[:, 0]), np.interp(gs, self.s, self.xy[:, 1])], axis=1)
        vmax = np.interp(gs, self.s, self.v_ref)
        nv = int(np.floor(self.v_ref.max() / VS)) + 2
        vs = np.arange(nv) * VS
        allowed = vs[None, :] <= np.maximum(vmax, MIN_SPEED_CAP)[:, None] + 0.5 * VS
        dv = max(int(round(A_MAX * DT_S / VS)), 1)
        R = np.zeros((ns, nv), bool)
        R[0, int(np.clip(round(v0 / VS), 0, nv - 1))] = True
        history = [R]
        for k in range(1, steps + 1):
            free = np.ones(ns, bool)
            for centers, radius in obstacles:
                q = centers[min(k, len(centers) - 1)]
                free &= np.hypot(pts[:, 0] - q[0], pts[:, 1] - q[1]) > radius + margin
            new = np.zeros_like(R)
            for v in range(nv):
                col = R[:, v]
                if not col.any():
                    continue
                for v2 in range(max(v - dv, 0), min(v + dv, nv - 1) + 1):
                    shift = int(round((vs[v] + vs[v2]) / 2 * DT_S / DS))
                    if shift == 0:
                        new[:, v2] |= col
                    elif shift < ns:
                        new[shift:, v2] |= col[:-shift]
                        new[-1, v2] |= col[ns - shift:].any()   # end of the reference: stays there
            new &= allowed & free[:, None]
            if not new.any():
                self.solve_times.append(time.perf_counter() - start)
                return None
            history.append(new)
            R = new
        # goal, backward pass (states from which it stays reachable), forward pass (most advanced)
        cells = np.argwhere(R)
        goal = tuple(cells[np.lexsort((cells[:, 1], cells[:, 0]))[-1]])
        G = [None] * (steps + 1)
        G[steps] = np.zeros_like(R)
        G[steps][goal] = True
        moves = [(v, v2, int(round((vs[v] + vs[v2]) / 2 * DT_S / DS)))
                 for v in range(nv) for v2 in range(max(v - dv, 0), min(v + dv, nv - 1) + 1)]
        for k in range(steps - 1, -1, -1):
            nxt, pre = G[k + 1], np.zeros_like(R)
            for v, v2, shift in moves:
                col = nxt[:, v2]
                if not col.any():
                    continue
                if shift == 0:
                    pre[:, v] |= col
                else:
                    pre[:max(ns - shift, 0), v] |= col[shift:]
                    if col[-1]:                       # the end of the reference absorbs whatever passes it
                        pre[max(ns - 1 - shift, 0):, v] = True
            G[k] = pre & history[k]
        si, vi = int(np.argmax(G[0].any(axis=1))), int(np.argmax(G[0][int(np.argmax(G[0].any(axis=1)))]))
        path = [(si, vi)]
        for k in range(1, steps + 1):
            best = None
            for v2 in range(max(vi - dv, 0), min(vi + dv, nv - 1) + 1):
                s2 = min(si + int(round((vs[vi] + vs[v2]) / 2 * DT_S / DS)), ns - 1)
                if G[k][s2, v2] and (best is None or (s2, v2) > best):
                    best = (s2, v2)
            if best is None:                          # should not happen: stop there
                break
            si, vi = best
            path.append(best)
        s_prof = gs[[p[0] for p in path]]
        v_prof = vs[[p[1] for p in path]]
        delayed = bool(np.any(v_prof < np.interp(s_prof, self.s, self.v_ref) - 1.5 * VS))
        self.solve_times.append(time.perf_counter() - start)
        return Profile(t0, s_prof, v_prof, delayed)


class TimeGuide:
    """The time planner as the lap node runs it (see the module docstring). Frames: the lap reference
    (x east, y north); tracks: traffic.Track, in the same frame."""

    def __init__(self, s, xy, velocity, acceleration, margins=MARGINS_M, stale_s=3.0):
        self.s = np.asarray(s, float)
        self.xy = np.asarray(xy, float)
        self.velocity = np.asarray(velocity, float)
        self.acceleration = np.asarray(acceleration, float)
        self.margins = tuple(margins)
        self.stale_s = stale_s
        self.planner = TimePlanner(self.s, self.xy, self.velocity)
        self.profile = None
        self.last_plan = -np.inf
        self.hold_until = -np.inf
        self.stats = dict(profiles=0, delayed=0, no_profile=0, stalls=0)

    def obstacles(self, now, tracks, altitude):
        out = []
        for track in tracks:
            if not track.updates or track.age(now) > self.stale_s:
                continue
            if abs(altitude - track.last.altitude) > track.last.vka + VERTICAL_MARGIN_M:
                continue
            centers = track.predict(now, HORIZON_S, DT_S)
            if abs(track.velocity_and_turn()[1]) < LOITER_OMEGA:      # a transit: trusted TRANSIT_TRUST_S only
                centers = centers.copy()
                centers[int(TRANSIT_TRUST_S / DT_S) + 1:] = 1e9
            out.append((centers, track.last.hka))
        return out

    def update(self, now, s_prog, velocity, obstacles):
        """Replans every REPLAN_S (obstacles from obstacles()). Returns the profile to fly, or None
        (no aircraft, or no safe profile: the local planner has control)."""
        if now < self.hold_until:
            self.profile = None
            return None
        if now - self.last_plan < REPLAN_S:
            return self.profile
        self.last_plan = now
        if not obstacles:
            self.profile = None
            return None
        i = int(np.clip(np.searchsorted(self.s, s_prog) - 1, 0, len(self.s) - 1))
        tangent = self.velocity[i] / max(float(np.linalg.norm(self.velocity[i])), 0.5)
        s0, v0 = float(s_prog), max(float(np.asarray(velocity, float)[:2] @ tangent), 0.0)
        if self.profile is not None:
            sp, vp, _ = self.profile.at(now)
            if abs(sp - s0) < CONTINUITY_M:
                s0, v0 = sp, vp
        profile = None
        for margin in self.margins:
            profile = self.planner.plan(now, s0, v0, obstacles, margin)
            if profile is not None:
                break
        if profile is not None and profile.s[-1] - s0 < STALL_M and s0 < self.s[-1] - 2 * STALL_M:
            profile = None
            self.stats['stalls'] += 1
        if profile is None:
            self.stats['no_profile'] += 1
            self.hold_until = now + HOLD_S
        else:
            self.stats['profiles'] += 1
            self.stats['delayed'] += profile.delayed
        self.profile = profile
        return profile

    def setpoint(self, now):
        """(position, velocity, acceleration), 2D, from the profile at now; None without a profile."""
        if self.profile is None:
            return None
        sp, vp, ap = self.profile.at(now)
        i = int(np.clip(np.searchsorted(self.s, sp) - 1, 0, len(self.s) - 2))
        f = float(np.clip((sp - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-6), 0.0, 1.0))
        position = self.xy[i] * (1 - f) + self.xy[i + 1] * f
        v_ref = max(float(np.linalg.norm(self.velocity[i])), 0.5)
        tangent = self.velocity[i] / v_ref
        a_ref = self.acceleration[i]
        a_normal = a_ref - tangent * float(a_ref @ tangent)       # the turn, scaled by (v / v_ref)²
        return position, tangent * vp, a_normal * (vp / v_ref) ** 2 + tangent * ap
