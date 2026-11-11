"""Local planner (convex MPC) that avoids the traffic while following a reference (a NavIntent path). No ROS.

guidance calls it while an aircraft threatens. Each replan optimizes the next HORIZON_S seconds
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
  - next waypoint: within its tolerance at the step where the progression reaches it (hard).
Quadratic cost, linear and second-order cone constraints: Clarabel (interior point) solves it in
about 20 iterations, a few ms. If it fails, plan() returns None and the caller keeps its previous
plan while that one still covers the present, else goes back to the reference.
Why each choice, and the bench and SITL results: laps_navigation_sitl_research.md §6.3 and §6.4.

Frames: any 2D frame, the same for the reference, the boundary, the waypoints and the aircraft (the
guidance node uses the mavros local frame, ENU). Units: m, s.
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
MIN_REFERENCE_SPEED = 0.5       # m/s: where the path asks for less (a stop), its direction at this speed
FENCE_MARGIN_M = 10.0            # hard: from the first step, so a drone closer than about this to an edge has no plan
FENCE_REACH_M = 300.0           # boundary edges farther than this are out of reach within the horizon
FENCE_SIDE_REACH_M = 50.0       # an edge counts if the drone projects onto it, give or take this
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
W_THETA_REG = 1e-4              # tiny quadratic terms on the progressions and slacks, for the solver
W_SLACK_REG = 1e-3
# Solver
TIME_LIMIT_S = 0.05
MAX_ITER = 60

# Layout of the solver's vector x, as index arrays. Positions and progressions are relative to the
# drone and its progression at the start of the plan (well scaled for the solver).
POS = np.arange(2 * (STEPS + 1)).reshape(STEPS + 1, 2)            # position at each step
VEL = POS + 2 * (STEPS + 1)                                       # velocity at each step
ACC = 4 * (STEPS + 1) + np.arange(2 * STEPS).reshape(STEPS, 2)    # acceleration over each step
THETA = 4 * (STEPS + 1) + 2 * STEPS + np.arange(STEPS + 1)        # progression at each step
NU = 5 * (STEPS + 1) + 2 * STEPS + np.arange(STEPS)               # progression speed over each step
N_FIXED = 5 * (STEPS + 1) + 3 * STEPS
# Then, per obstacle and per step, its depth into the hard margin, then the same for the soft margin
# (LocalPlanner.plan builds those indices: their number depends on the obstacles).


@dataclass
class Obstacle:
    centers: np.ndarray   # (STEPS + 1, 2): predicted positions from now, every DT
    keep_away: float      # horizontal keep-away, m


@dataclass
class Plan:
    t0: float              # time of the plan (caller's clock), s
    position: np.ndarray   # (STEPS + 1, 2)
    velocity: np.ndarray   # (STEPS + 1, 2)
    accel: np.ndarray      # (STEPS, 2), constant over each step
    theta: np.ndarray      # (STEPS + 1,), progression along the reference

    def covers(self, t):
        """True while the plan still says where to go from t (the last step is kept as a margin)."""
        return 0.0 <= t - self.t0 < HORIZON_S - DT

    def at(self, t):
        """Position, velocity, acceleration at time t, linear between steps."""
        tau = max(t - self.t0, 0.0) / DT
        i = min(int(tau), STEPS - 1)
        f = min(tau - i, 1.0)
        return (self.position[i] * (1 - f) + self.position[i + 1] * f,
                self.velocity[i] * (1 - f) + self.velocity[i + 1] * f,
                self.accel[i].copy())


class Reference:
    """The path as the planner sees it: points, tangents and speeds along s."""

    def __init__(self, s, xy, velocity):
        velocity = np.array(velocity, float)
        direction = np.gradient(xy, axis=0)
        norm = np.maximum(np.linalg.norm(direction, axis=1), 1e-9)
        slow = np.linalg.norm(velocity, axis=1) < MIN_REFERENCE_SPEED
        velocity[slow] = MIN_REFERENCE_SPEED * direction[slow] / norm[slow, None]
        self.s = np.asarray(s, float)
        self.xy = np.asarray(xy, float)
        self.speed = np.maximum(np.linalg.norm(velocity, axis=1), MIN_REFERENCE_SPEED)
        self.tangent = velocity / self.speed[:, None]

    def at(self, theta):
        """(points, tangents, speeds) at the progressions theta (array)."""
        i = np.clip(np.searchsorted(self.s, theta) - 1, 0, len(self.s) - 2)
        f = np.clip((theta - self.s[i]) / np.maximum(self.s[i + 1] - self.s[i], 1e-6), 0, 1)
        return self.xy[i] * (1 - f)[:, None] + self.xy[i + 1] * f[:, None], self.tangent[i], self.speed[i]


class LocalPlanner:
    def __init__(self, reference, boundary, margin_hard_m, margin_soft_m):
        self.ref = reference
        self.edges = _boundary_edges(boundary)
        self.margin_hard, self.margin_soft = margin_hard_m, margin_soft_m
        self.previous = None     # last Plan: the linearization point of the next one
        self.solve_times = []    # s, for the logs
        self.failures = 0

    def plan(self, theta0, position, velocity, obstacles, gate, t_now):
        """theta0: current progression on the reference (m); position, velocity: drone (2D);
        obstacles: list of Obstacle; gate: (progression of the waypoint apex, waypoint xy,
        tolerance m) or None; t_now: caller's clock (s). Returns a Plan, or None if the solver failed."""
        o, th0 = np.asarray(position, float), float(theta0)
        # Where the drone is expected to go (the previous plan, else the reference), relative to it
        theta_bar, xy_bar = self._nominal(th0, o, t_now)
        r_bar, t_bar, speed_bar = self.ref.at(theta_bar)
        r_bar, xy_bar, theta_bar = r_bar - o, xy_bar - o, theta_bar - th0
        n_slacks = len(obstacles) * (STEPS + 1)
        hard = N_FIXED + np.arange(n_slacks).reshape(len(obstacles), STEPS + 1)
        soft = hard + n_slacks
        nx = N_FIXED + 2 * n_slacks

        cost = self._cost(nx, r_bar, t_bar, theta_bar, hard, soft)
        c = _Constraints()
        self._add_dynamics(c, velocity, speed_bar)
        self._add_fence(c, o)
        self._add_aircraft(c, obstacles, o, xy_bar, r_bar, hard, soft)
        if gate is not None:
            s_apex, wp, tol = gate
            self._add_gate(c, s_apex - th0, np.asarray(wp, float) - o, theta_bar, tol)

        x = self._solve(cost, c, nx)
        if x is None:
            return None
        self.previous = Plan(t_now, x[POS] + o, x[VEL], x[ACC], x[THETA] + th0)
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
    def _cost(nx, r_bar, t_bar, theta_bar, hard, soft):
        cost = _Cost(nx)
        n_bar = np.stack([-t_bar[:, 1], t_bar[:, 0]], axis=1)
        for i in range(1, STEPS + 1):
            n, t = n_bar[i], t_bar[i]
            # lateral error n . (p - r) ; longitudinal error t . (p - r) - (theta - theta_bar)
            cost.square(W_LATERAL, [(POS[i, 0], n[0]), (POS[i, 1], n[1])], float(n @ r_bar[i]))
            cost.square(W_LONGITUDINAL, [(POS[i, 0], t[0]), (POS[i, 1], t[1]), (THETA[i], -1.0)],
                        float(t @ r_bar[i]) - float(theta_bar[i]))
            cost.add(THETA[i], THETA[i], W_THETA_REG)
        for i in range(STEPS):
            cost.q[NU[i]] = -W_PROGRESS
            for k in range(2):
                cost.square(W_ACC, [(ACC[i, k], 1.0)], 0.0)
                if i > 0:
                    cost.square(W_JERK, [(ACC[i, k], 1.0), (ACC[i - 1, k], -1.0)], 0.0)
        for h, s in zip(hard.flat, soft.flat):
            cost.q[h], cost.q[s] = W_HARD, W_SOFT
            cost.add(h, h, W_SLACK_REG)
            cost.add(s, s, W_SLACK_REG)
        return cost

    @staticmethod
    def _add_dynamics(c, velocity, speed_bar):
        """Start here at this velocity, double integrator, bounded speeds and acceleration."""
        for k in range(2):
            c.eq([(POS[0, k], 1.0)], 0.0)
            c.eq([(VEL[0, k], 1.0)], float(velocity[k]))
        c.eq([(THETA[0], 1.0)], 0.0)
        for i in range(STEPS):
            for k in range(2):
                c.eq([(POS[i + 1, k], 1.0), (POS[i, k], -1.0), (VEL[i, k], -DT), (ACC[i, k], -0.5 * DT * DT)], 0.0)
                c.eq([(VEL[i + 1, k], 1.0), (VEL[i, k], -1.0), (ACC[i, k], -DT)], 0.0)
            c.eq([(THETA[i + 1], 1.0), (THETA[i], -1.0), (NU[i], -DT)], 0.0)
            c.ge([(NU[i], 1.0)], 0.0)
            c.le([(NU[i], 1.0)], NU_MAX_FACTOR * float(speed_bar[i]))
            c.norm_le(ACC[i, 0], ACC[i, 1], 0.0, 0.0, A_MAX)
        for i in range(1, STEPS + 1):
            c.norm_le(VEL[i, 0], VEL[i, 1], 0.0, 0.0, V_MAX)

    def _add_fence(self, c, o):
        """The boundary edges that face the drone, FENCE_MARGIN_M inside, from the first step on.

        The boundary may be non-convex, so its edges are not all valid half-planes everywhere. Kept:
        the edges whose line the drone is inside of, whose segment it projects onto (give or take
        FENCE_SIDE_REACH_M), and that are within FENCE_REACH_M: locally, these describe the boundary."""
        for a, edge, length, n, cn in self.edges:
            u = float((o - a) @ edge) / length
            inside = float(n @ (o - a))
            if inside >= 0.0 and -FENCE_SIDE_REACH_M <= u <= length + FENCE_SIDE_REACH_M and inside <= FENCE_REACH_M:
                for i in range(1, STEPS + 1):
                    c.ge([(POS[i, 0], n[0]), (POS[i, 1], n[1])], cn - float(n @ o) + FENCE_MARGIN_M)

    def _add_aircraft(self, c, obstacles, o, xy_bar, r_bar, hard, soft):
        """Each aircraft, each step: outside the half-plane tangent to its circle, facing the nominal
        path, by the two margins (each with its slack)."""
        for j, ob in enumerate(obstacles):
            centers = ob.centers - o
            for i in range(STEPS + 1):
                c.ge([(hard[j, i], 1.0)], 0.0)
                c.ge([(soft[j, i], 1.0)], 0.0)
                if i == 0:
                    continue
                away = xy_bar[i] - centers[i]
                if np.linalg.norm(away) < 1e-3:       # nominal path on the center: use the reference
                    away = r_bar[i] - centers[i]
                if np.linalg.norm(away) < 1e-3:
                    away = np.array([1.0, 0.0])
                n = away / np.linalg.norm(away)
                base = float(n @ centers[i]) + ob.keep_away
                c.ge([(POS[i, 0], n[0]), (POS[i, 1], n[1]), (hard[j, i], 1.0)], base + self.margin_hard)
                c.ge([(POS[i, 0], n[0]), (POS[i, 1], n[1]), (soft[j, i], 1.0)], base + self.margin_soft)

    @staticmethod
    def _add_gate(c, s_apex, wp, theta_bar, tol):
        """The next waypoint, if the nominal progression reaches it within the horizon."""
        if theta_bar[0] <= s_apex <= theta_bar[-1]:
            i = max(int(np.argmin(np.abs(theta_bar - s_apex))), 1)
            c.norm_le(POS[i, 0], POS[i, 1], float(wp[0]), float(wp[1]), tol)
            c.ge([(THETA[i], 1.0)], s_apex - GATE_WINDOW_M)
            c.le([(THETA[i], 1.0)], s_apex + GATE_WINDOW_M)

    def _solve(self, cost, c, nx):
        """The solution vector x, or None if the solver failed."""
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
        return x


def _boundary_edges(boundary):
    """Edges of the boundary polygon as (start, edge vector, length, inward normal n, n . start):
    n . p >= n . start inside."""
    pts = np.asarray(boundary, float)
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    edges = []
    for a, b in zip(pts, np.roll(pts, -1, axis=0)):
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length < 1e-6:
            continue
        # inward normal: left of the edge for a counterclockwise polygon
        n = np.array([-edge[1], edge[0]]) / length * (1.0 if area > 0 else -1.0)
        edges.append((a, edge, length, n, float(n @ a)))
    return edges


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
