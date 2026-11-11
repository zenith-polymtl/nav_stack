"""Traffic avoidance along one path, without ROS: what threatens, when to replan, which plan to fly.

guidance keeps one Avoidance per path mode, built with its path. Every tick in that mode it calls
update(): every replan_s, the aircraft whose predicted path comes within threat_range_m of the drone
or of the path ahead, in vertical overlap with the drone, go to the local planner (local_planner.py),
which plans the next 10 s. update() returns the plan to fly now, or None to follow the path: no
aircraft threatens, or the last replans failed and the previous plan no longer covers the present.
Why each choice: the lap node of the nav_stack dev branch and laps_navigation_sitl_research.md
(aeac-2027, before d1cfce7).
"""
import math

import numpy as np

from nav_stack import local_planner, traffic

GATE_MARGIN_M = 1.0          # the planner's gate is the NavGate tolerance minus this (at least 1 m)
MAX_OBSTACLES = 4            # nearest threatening aircraft given to the planner
VERTICAL_MARGIN_M = 3.0      # an aircraft counts if |altitude difference| <= its vertical keep-away + this
BLOCKED_PROGRESS_M = 1.0     # a plan that progresses less than this over its horizon waits


class Avoidance:
    def __init__(self, path, boundary, replan_s, threat_range_m, margin_hard_m, margin_soft_m, logger):
        """path: the IntentPath followed; boundary: flight boundary vertices in the same frame."""
        reference = local_planner.Reference(path.s, path.xyz[:, :2], path.velocity[:, :2])
        self.planner = local_planner.LocalPlanner(reference, boundary, margin_hard_m, margin_soft_m)
        self.replan_s = replan_s
        self.threat_range = threat_range_m
        self.log = logger
        self.plan = None              # local_planner.Plan being flown, or None
        self.last_replan = -math.inf  # s
        self.active = False           # an aircraft threatens: plans are being made
        self.fail_streak = 0          # failed replans in a row (logs)
        self.threat_count = 0         # for the status
        self.min_margin = math.nan    # m, predicted margin to the nearest keep-away circle, for the status

    @property
    def waiting(self):
        """The plan barely progresses over its horizon: the drone waits for an aircraft to move away."""
        return self.plan.theta[-1] - self.plan.theta[0] < BLOCKED_PROGRESS_M

    def update(self, now, path, xy, velocity, altitude, tracks):
        """The plan to fly at now (s), or None: follow the path.

        xy: drone position (latency compensated); velocity: measured, xyz; altitude: above home;
        tracks: the traffic.Track of every aircraft known."""
        if now - self.last_replan >= self.replan_s:
            self.last_replan = now
            obstacles, self.threat_count, self.min_margin = self._threats(
                now, path.progress, xy, velocity, altitude, tracks)
            if not obstacles:
                self.stop(path.progress)
                return None
            if not self.active:
                self.active = True
                self.log.info(f'Évitement : {len(obstacles)} avion(s) menace(nt), plans locaux.')
            self._replan(now, path, xy, velocity[:2], obstacles)
        if self.plan is None or not self.plan.covers(now):
            return None
        return self.plan

    def stop(self, progress):
        """No aircraft threatens any more (or avoidance is switched off, or the path ends): back to
        the path. progress: where the drone is on the path, for the log."""
        self.plan = None
        self.last_replan = -math.inf
        self.threat_count = 0
        self.min_margin = math.nan
        self.fail_streak = 0
        if not self.active:
            return
        self.active = False
        times = np.array(self.planner.solve_times) * 1000
        if len(times):
            self.log.info(
                f"Fin d'évitement à {progress:.0f} m. Plans : {len(times)}, {np.median(times):.0f} ms en "
                f'médiane, {times.max():.0f} ms au pire, {self.planner.failures} ratés.')
            self.planner.solve_times.clear()   # the statistics of the next avoidance start over
            self.planner.failures = 0

    def _threats(self, now, progress, xy, velocity, altitude, tracks):
        """(obstacles, number of threats, min margin). The threats: aircraft whose predicted path
        comes within threat_range of the drone or of the path ahead, in vertical overlap with the
        drone's predicted altitude; the obstacles: the MAX_OBSTACLES nearest. The margin: predicted
        distance from the nearest keep-away circle to what the drone will fly (the plan, else the path)."""
        t = np.arange(local_planner.STEPS + 1) * local_planner.DT
        altitudes = np.clip(altitude + velocity[2] * t, 0.0, None)
        speed = max(float(np.hypot(*velocity[:2])), 1.0)
        ahead = self.planner.ref.at(progress + t * speed)[0]
        flown = ahead
        if self.plan is not None and self.plan.covers(now):
            flown = np.array([self.plan.at(now + dt)[0] for dt in t])
        found, margin = [], math.inf
        for track in tracks:
            if track.age(now) > traffic.STALE_S:
                continue
            last = track.last
            if not np.any(np.abs(altitudes - last.altitude) <= last.vertical_keep_away + VERTICAL_MARGIN_M):
                continue
            centers = track.predict(now, local_planner.HORIZON_S, local_planner.DT)
            margin = min(margin, float(np.min(np.hypot(*(centers - flown).T))) - last.horizontal_keep_away)
            near = min(float(np.min(np.hypot(*(centers - ahead).T))), float(np.hypot(*(centers[0] - xy))))
            if near < self.threat_range:
                found.append((near, local_planner.Obstacle(centers, last.horizontal_keep_away)))
        found.sort(key=lambda f: f[0])
        return [ob for _, ob in found[:MAX_OBSTACLES]], len(found), margin if math.isfinite(margin) else math.nan

    def _replan(self, now, path, xy, velocity_xy, obstacles):
        """A new plan through the next gate; when it fails, the previous one stays (while it covers now)."""
        gate = path.next_gate
        if gate is not None:
            gate = (gate.s_apex, gate.xy, max(gate.tolerance - GATE_MARGIN_M, 1.0))
        plan = self.planner.plan(path.progress, xy, velocity_xy, obstacles, gate, now)
        if plan is not None:
            if self.fail_streak > 1:
                self.log.info(f'Plan local rétabli après {self.fail_streak} échecs de suite.')
            self.fail_streak = 0
            self.plan = plan
            return
        # A streak of failures: a single WARN, at its start
        self.fail_streak += 1
        if self.fail_streak == 1:
            self.log.warn("Plan local raté : le précédent reste en vigueur s'il est encore valide.")
