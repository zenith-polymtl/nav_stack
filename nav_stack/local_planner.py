"""Local planner (convex MPC) that avoids the traffic while following the lap reference. No ROS.

The lap node calls it while an aircraft threatens. Each replan optimizes the next HORIZON_S seconds
(STEPS steps of DT) of a point mass:
  - double integrator: p+ = p + v dt + a dt^2 / 2, v+ = v + a dt, with |v| <= V_MAX, |a| <= A_MAX;
  - a progression theta along the reference, free to slow down (theta+ = theta + nu dt, nu >= 0):
    the cost rewards progress instead of chasing a timetable, so the drone can WAIT for an aircraft
    to move away;
  - stay close to the reference at theta, the lateral error weighted much more than the
    longitudinal one (the reference is linearized around the previous plan);
  - each aircraft, at each step: the half-plane tangent to its predicted circle, on the side of the
    previous plan (this keeps the problem convex). Two radii: keep-away + margin_hard_m, almost a
    hard constraint, and keep-away + margin_soft_m, a mere preference. A single wide hard margin
    can make a waypoint unreachable (an aircraft circling near it), and the drone then waits forever;
  - flight boundary: the polygon edges that face the drone, hard, FENCE_MARGIN_M inside;
  - next waypoint: within gate_tol_m (GATE_TOL_M by default) of it at the step where the progression
    reaches it (hard).
Quadratic cost, linear and second-order cone constraints: Clarabel (interior point) solves it in
about 20 iterations, a few ms. If it fails, plan() returns None and the caller keeps its previous
plan while that one still covers the present, else goes back to the reference.
Why each choice, and the bench and SITL results: laps_navigation_sitl_research.md §6.3 and §6.4.

Frames: any 2D frame, the same for the reference, the boundary, the waypoints and the aircraft (the
lap node uses ENU local). Units: m, s.
"""
import math
import time
from dataclasses import dataclass

import clarabel
import numpy as np
import scipy.sparse as sp

G = 9.80665

HORIZON_S = 10.0
DT = 0.5
STEPS = int(round(HORIZON_S / DT))
V_MAX = 21.0                    # m/s
A_MAX = G * math.tan(math.radians(30.0))   # 30 deg of lean
FENCE_MARGIN_M = 25.0
FENCE_REACH_M = 300.0           # boundary edges farther than this are out of reach within the horizon
GATE_TOL_M = 9.0                # default gate: the rules give 10 m (question for the Q&A), 1 m of margin;
                                # lap passes its own (wp_tolerance_m - 1)
GATE_WINDOW_M = 8.0             # progression within this of the waypoint apex at the gate step
NU_MAX_FACTOR = 1.15            # progression speed at most this x the reference speed
# Cost weights
W_LATERAL = 0.5
W_LONGITUDINAL = 0.05
W_PROGRESS = 6.0
W_ACC = 0.05
W_JERK = 0.2
W_HARD = 400.0                  # per m inside the hard margin
W_SOFT = 2.0                    # per m inside the soft margin
# Solver
TIME_LIMIT_S = 0.05
MAX_ITER = 60


# The solver's vector x: positions (STEPS + 1 x 2), velocities (STEPS + 1 x 2), accelerations
# (STEPS x 2), progressions (STEPS + 1), progression speeds (STEPS), then for each obstacle its
# depth into the hard margin at each step, then the same for the soft margin.
def _pos(i, k):
    return 2 * i + k


def _vel(i, k):
    return 2 * (STEPS + 1) + 2 * i + k


def _acc(i, k):
    return 4 * (STEPS + 1) + 2 * i + k


def _theta(i):
    return 4 * (STEPS + 1) + 2 * STEPS + i


def _nu(i):
    return 5 * (STEPS + 1) + 2 * STEPS + i


_SLACKS = 5 * (STEPS + 1) + 3 * STEPS


def _slack(soft, j, i, n_obstacles):
    return _SLACKS + (soft * n_obstacles + j) * (STEPS + 1) + i


@dataclass
class Obstacle:
    centers: np.ndarray   # (STEPS + 1, 2): predicted positions from now, every DT
    keep_away: float      # horizontal keep-away, m


@dataclass
class Plan:
    t0: float              # time of the plan (caller's clock), s
    position: np.ndarray   # (STEPS + 1, 2)
    velocity: np.ndarray   # (STEPS + 1, 2)
    accel: np.ndarray      # (STEPS + 1, 2)
    theta: np.ndarray      # (STEPS + 1,), progression along the reference

    def covers(self, t):
        """True while the plan still says where to go from t (the last step is kept as a margin)."""
        return 0.0 <= t - self.t0 < HORIZON_S - DT

    def at(self, t):
        """Position, velocity, acceleration at time t, linear between steps."""
        tau = max(t - self.t0, 0.0) / DT
        i = min(int(tau), len(self.theta) - 2)
        f = min(tau - i, 1.0)
        return (self.position[i] * (1 - f) + self.position[i + 1] * f,
                self.velocity[i] * (1 - f) + self.velocity[i + 1] * f,
                self.accel[i].copy())


class Reference:
    """The lap reference as the planner sees it: points, tangents and speeds along s."""

    def __init__(self, s, xy, velocity):
        self.s = np.asarray(s, float)
        self.xy = np.asarray(xy, float)
        self.speed = np.maximum(np.linalg.norm(velocity, axis=1), 0.5)
        self.tangent = np.asarray(velocity, float) / self.speed[:, None]

    def at(self, theta):
        """(points, tangents, speeds) at the progressions theta (array)."""
        i = np.clip(np.searchsorted(self.s, theta) - 1, 0, len(self.s) - 2)
        f = np.clip((theta - self.s[i]) / np.maximum(self.s[i + 1] - self.s[i], 1e-6), 0, 1)
        return self.xy[i] * (1 - f)[:, None] + self.xy[i + 1] * f[:, None], self.tangent[i], self.speed[i]


def boundary_halfplanes(boundary, position):
    """Edges of the flight boundary polygon that face the drone, as (n, c): n . p >= c inside.

    The boundary may be non-convex, so its edges are not all valid half-planes everywhere. Kept:
    the edges whose line the drone is inside of, whose segment it projects onto (+- 50 m), and
    that are within FENCE_REACH_M: locally, these describe the boundary around the drone."""
    pts = np.asarray(boundary, float)
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    out = []
    for a, b in zip(pts, np.roll(pts, -1, axis=0)):
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length < 1e-6:
            continue
        # inward normal: left of the edge for a counterclockwise polygon
        n = np.array([-edge[1], edge[0]]) / length * (1.0 if area > 0 else -1.0)
        u = float((position - a) @ edge) / length
        inside = float(n @ (position - a))
        if inside >= 0.0 and -50.0 <= u <= length + 50.0 and inside <= FENCE_REACH_M:
            out.append((n, float(n @ a)))
    return out


class _Cost:
    """1/2 x'Hx + q'x, built term by term."""

    def __init__(self, nx):
        self.nx, self.H, self.q = nx, {}, np.zeros(nx)

    def add(self, i, j, v):
        self.H[(i, j)] = self.H.get((i, j), 0.0) + v

    def square(self, w, coefs, b):                # w (sum c x - b)^2
        for i, ci in coefs:
            for j, cj in coefs:
                self.add(i, j, 2 * w * ci * cj)
            self.q[i] -= 2 * w * ci * b

    def matrix(self):
        H = sp.csc_matrix((list(self.H.values()), tuple(zip(*self.H.keys()))), shape=(self.nx, self.nx))
        return sp.triu(H, format='csc')


class _Constraints:
    """Equalities, inequalities and norm bounds for Clarabel, one line each."""

    def __init__(self):
        self.zero, self.nonneg, self.soc = [], [], []

    def eq(self, coefs, b):                       # sum c x = b
        self.zero.append((coefs, b))

    def le(self, coefs, b):                       # sum c x <= b
        self.nonneg.append((coefs, b))

    def ge(self, coefs, b):                       # sum c x >= b
        self.nonneg.append(([(i, -c) for i, c in coefs], -b))

    def norm_le(self, ix, iy, cx, cy, radius):    # ||(x_ix - cx, x_iy - cy)|| <= radius
        self.soc.append([([], radius), ([(ix, -1.0)], -cx), ([(iy, -1.0)], -cy)])

    def matrices(self, nx):
        """(A, b, cones) such that b - A x lies in the cones."""
        rows = self.zero + self.nonneg + [row for block in self.soc for row in block]
        cones = ([clarabel.ZeroConeT(len(self.zero)), clarabel.NonnegativeConeT(len(self.nonneg))]
                 + [clarabel.SecondOrderConeT(3)] * len(self.soc))
        triplets = [(r, i, c) for r, (coefs, _) in enumerate(rows) for i, c in coefs]
        r, i, c = zip(*triplets)
        A = sp.csc_matrix((c, (r, i)), shape=(len(rows), nx))
        return A, np.array([rhs for _, rhs in rows]), cones


class LocalPlanner:
    def __init__(self, reference, boundary, margin_hard_m, margin_soft_m, gate_tol_m=None):
        self.ref = reference
        self.boundary = np.asarray(boundary, float)
        self.margin_hard, self.margin_soft = margin_hard_m, margin_soft_m
        self.gate_tol = gate_tol_m       # None: GATE_TOL_M
        self.previous = None     # last Plan: the linearization point of the next one
        self.solve_times = []    # s, for the logs
        self.failures = 0

    def plan(self, theta0, position, velocity, obstacles, gate, t_now):
        """theta0: current progression on the reference (m); position, velocity: drone (2D);
        obstacles: list of Obstacle; gate: (progression of the waypoint apex, waypoint xy) or None;
        t_now: caller's clock (s). Returns a Plan, or None if the solver failed."""
        o, th0 = np.asarray(position, float), float(theta0)
        theta_bar, xy_bar = self._nominal(th0, o, t_now)
        r_bar, t_bar, speed_bar = self.ref.at(theta_bar)
        # Everything relative to the drone and its progression: well scaled for the solver
        r_bar, xy_bar, theta_bar = r_bar - o, xy_bar - o, theta_bar - th0
        nx = _SLACKS + 2 * len(obstacles) * (STEPS + 1)

        cost = self._cost(nx, len(obstacles), r_bar, t_bar, theta_bar)
        c = self._dynamics(velocity, speed_bar)
        for n, cn in boundary_halfplanes(self.boundary, o):
            for i in range(1, STEPS + 1):
                c.ge([(_pos(i, 0), n[0]), (_pos(i, 1), n[1])], cn - float(n @ o) + FENCE_MARGIN_M)
        self._avoid(c, obstacles, o, xy_bar, r_bar)
        if gate is not None:
            self._gate(c, gate[0] - th0, np.asarray(gate[1], float) - o, theta_bar,
                       GATE_TOL_M if self.gate_tol is None else self.gate_tol)

        A, b, cones = c.matrices(nx)
        settings = clarabel.DefaultSettings()
        settings.verbose = False
        settings.time_limit = TIME_LIMIT_S
        settings.max_iter = MAX_ITER
        start = time.perf_counter()
        sol = clarabel.DefaultSolver(cost.matrix(), cost.q, A, b, cones, settings).solve()
        self.solve_times.append(time.perf_counter() - start)
        x = np.array(sol.x)
        if str(sol.status) not in ('Solved', 'AlmostSolved') or not np.all(np.isfinite(x)):
            self.failures += 1
            return None
        steps = range(STEPS + 1)
        acc = np.array([[x[_acc(i, 0)], x[_acc(i, 1)]] for i in range(STEPS)])
        self.previous = Plan(t_now,
                             np.array([[x[_pos(i, 0)], x[_pos(i, 1)]] for i in steps]) + o,
                             np.array([[x[_vel(i, 0)], x[_vel(i, 1)]] for i in steps]),
                             np.vstack([acc, acc[-1:]]),
                             np.array([x[_theta(i)] for i in steps]) + th0)
        return self.previous

    def _nominal(self, theta0, position, t_now):
        """Where the previous plan says we go from now (progressions, positions), else the reference."""
        prev, N = self.previous, STEPS
        if prev is not None and prev.covers(t_now):
            tq = (t_now - prev.t0) + np.arange(N + 1) * DT
            tp = np.arange(N + 1) * DT
            theta = np.interp(tq, tp, prev.theta, right=np.nan)
            xy = np.stack([np.interp(tq, tp, prev.position[:, k], right=np.nan) for k in range(2)], axis=1)
            last = int(np.where(np.isfinite(theta))[0][-1])
            for i in range(last + 1, N + 1):          # past the end of the previous plan: extrapolate
                theta[i] = theta[i - 1] + (prev.theta[-1] - prev.theta[-2])
                xy[i] = xy[i - 1] + (prev.position[-1] - prev.position[-2])
            return theta - theta[0] + theta0, xy - xy[0] + position
        speed = self.ref.at(np.array([theta0]))[2][0]
        theta = theta0 + np.arange(N + 1) * DT * speed
        return theta, self.ref.at(theta)[0]

    @staticmethod
    def _cost(nx, n_obstacles, r_bar, t_bar, theta_bar):
        cost = _Cost(nx)
        n_bar = np.stack([-t_bar[:, 1], t_bar[:, 0]], axis=1)
        for i in range(1, STEPS + 1):
            n, t = n_bar[i], t_bar[i]
            # lateral error n . (p - r) ; longitudinal error t . (p - r) - (theta - theta_bar)
            cost.square(W_LATERAL, [(_pos(i, 0), n[0]), (_pos(i, 1), n[1])], float(n @ r_bar[i]))
            cost.square(W_LONGITUDINAL, [(_pos(i, 0), t[0]), (_pos(i, 1), t[1]), (_theta(i), -1.0)],
                        float(t @ r_bar[i]) - float(theta_bar[i]))
            cost.add(_theta(i), _theta(i), 1e-4)
        for i in range(STEPS):
            cost.q[_nu(i)] = -W_PROGRESS
            for k in range(2):
                cost.square(W_ACC, [(_acc(i, k), 1.0)], 0.0)
                if i > 0:
                    cost.square(W_JERK, [(_acc(i, k), 1.0), (_acc(i - 1, k), -1.0)], 0.0)
        for j in range(n_obstacles):
            for i in range(STEPS + 1):
                hard, soft = _slack(0, j, i, n_obstacles), _slack(1, j, i, n_obstacles)
                cost.q[hard], cost.q[soft] = W_HARD, W_SOFT
                cost.add(hard, hard, 1e-3)
                cost.add(soft, soft, 1e-3)
        return cost

    @staticmethod
    def _dynamics(velocity, speed_bar):
        """Start here at this velocity, double integrator, bounded speeds and acceleration."""
        c = _Constraints()
        for k in range(2):
            c.eq([(_pos(0, k), 1.0)], 0.0)
            c.eq([(_vel(0, k), 1.0)], float(velocity[k]))
        c.eq([(_theta(0), 1.0)], 0.0)
        for i in range(STEPS):
            for k in range(2):
                c.eq([(_pos(i + 1, k), 1.0), (_pos(i, k), -1.0), (_vel(i, k), -DT), (_acc(i, k), -0.5 * DT * DT)], 0.0)
                c.eq([(_vel(i + 1, k), 1.0), (_vel(i, k), -1.0), (_acc(i, k), -DT)], 0.0)
            c.eq([(_theta(i + 1), 1.0), (_theta(i), -1.0), (_nu(i), -DT)], 0.0)
            c.ge([(_nu(i), 1.0)], 0.0)
            c.le([(_nu(i), 1.0)], NU_MAX_FACTOR * float(speed_bar[i]))
            c.norm_le(_acc(i, 0), _acc(i, 1), 0.0, 0.0, A_MAX)
        for i in range(1, STEPS + 1):
            c.norm_le(_vel(i, 0), _vel(i, 1), 0.0, 0.0, V_MAX)
        return c

    def _avoid(self, c, obstacles, o, xy_bar, r_bar):
        """Each aircraft, each step: outside the half-plane tangent to its circle, facing the nominal
        path, by the two margins (each with its slack)."""
        J = len(obstacles)
        for j, ob in enumerate(obstacles):
            centers = ob.centers - o
            for i in range(STEPS + 1):
                hard, soft = _slack(0, j, i, J), _slack(1, j, i, J)
                c.ge([(hard, 1.0)], 0.0)
                c.ge([(soft, 1.0)], 0.0)
                if i == 0:
                    continue
                away = xy_bar[i] - centers[i]
                if np.linalg.norm(away) < 1e-3:       # nominal path on the center: use the reference
                    away = r_bar[i] - centers[i]
                if np.linalg.norm(away) < 1e-3:
                    away = np.array([1.0, 0.0])
                n = away / np.linalg.norm(away)
                base = float(n @ centers[i]) + ob.keep_away
                c.ge([(_pos(i, 0), n[0]), (_pos(i, 1), n[1]), (hard, 1.0)], base + self.margin_hard)
                c.ge([(_pos(i, 0), n[0]), (_pos(i, 1), n[1]), (soft, 1.0)], base + self.margin_soft)

    @staticmethod
    def _gate(c, s_apex, wp, theta_bar, tol):
        """The next waypoint, if the nominal progression reaches it within the horizon."""
        if theta_bar[0] <= s_apex <= theta_bar[-1]:
            i = max(int(np.argmin(np.abs(theta_bar - s_apex))), 1)
            c.norm_le(_pos(i, 0), _pos(i, 1), float(wp[0]), float(wp[1]), tol)
            c.ge([(_theta(i), 1.0)], s_apex - GATE_WINDOW_M)
            c.le([(_theta(i), 1.0)], s_apex + GATE_WINDOW_M)
