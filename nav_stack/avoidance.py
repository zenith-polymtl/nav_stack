"""Traffic avoidance along one path, without ROS: what threatens, when to act, how.

guidance keeps one Avoidance per path mode, built with its path. Every tick in that mode it calls
update(); every replan_s:
  - the threats: aircraft whose predicted path comes within threat_range_m of the drone or of the path
    ahead, in vertical overlap with the drone;
  - avoidance engages only while the path ahead (or what the drone flies) comes within ENGAGE_MARGIN_M of
    a keep-away circle, and lets go once both are ENGAGE_HYSTERESIS_M farther and the drone is back within
    RETURN_M of the path. Elsewhere the drone follows the path, which passes its waypoints better than a
    plan cutting corners;
  - engaged, and near the path: first timing alone (timing.py), the drone stays on the path and slows or
    waits; when no timing clears the aircraft, or the drone is off the path, the lateral plan
    (local_planner.py, the next 10 s).
update() returns the lateral plan to fly now, or None: the path, at the timing profile's speed if there
is one (speed_on_path()).
Why each choice and the bench results: _guidance-eval, docs/journal.md (2026-11-10 to 12), v5 bench,
server keep-away world: points lost 954 (before) to about 155 (30 s of traffic before takeoff).
"""
import math

import numpy as np

from nav_stack import local_planner, timing, traffic

GATE_MARGIN_M = 1.0          # the planner's gate is the NavGate tolerance minus this (at least 1 m)
MAX_OBSTACLES = 4            # nearest threatening aircraft given to the planner
VERTICAL_MARGIN_M = 3.0      # an aircraft counts if |altitude difference| <= its vertical keep-away + this
BLOCKED_PROGRESS_M = 1.0     # a plan that progresses less than this over its horizon waits
ENGAGE_MARGIN_M = 15.0       # avoidance only while the path ahead or the flown plan comes this close to a
                             # keep-away circle (planning as soon as an aircraft threatens cut corners and
                             # missed waypoints)
ENGAGE_HYSTERESIS_M = 10.0   # avoidance stops once both are this much farther than ENGAGE_MARGIN_M...
RETURN_M = 10.0              # ...and the drone is back within this of the path at its progression (a plan
                             # that had backed the drone up 70 m let go, and the drone flew straight back
                             # to the path through a transit)
TIMING_PATH_M = 10.0         # timing only while the drone is this close to the path: it flies the path's position
STALL_S = 25.0               # engaged and the progression stuck this long (longer than any timing hold): avoidance
STALL_PROGRESS_M = 5.0       # lets go and the drone follows the path for FORCE_PATH_S, at the risk of an incursion
FORCE_PATH_S = 10.0          # (5 points) rather than a flight that never finishes (two aircraft holding around a
                             # waypoint kept the lateral plan circling for 13 min, v5 bench rc02_be06)


class Avoidance:
    def __init__(self, path, boundary, replan_s, threat_range_m, margin_hard_m, margin_soft_m, logger):
        """path: the IntentPath followed; boundary: flight boundary vertices in the same frame."""
        reference = local_planner.Reference(path.s, path.xyz[:, :2], path.velocity[:, :2])
        self.planner = local_planner.LocalPlanner(reference, boundary, margin_hard_m, margin_soft_m)
        self.replan_s = replan_s
        self.threat_range = threat_range_m
        self.log = logger
        self.plan = None              # local_planner.Plan being flown, or None
        self.profile = None           # timing.Profile being flown, or None
        self.last_replan = -math.inf  # s
        self.active = False           # avoidance engaged
        self.fail_streak = 0          # failed replans in a row (logs)
        self.threat_count = 0         # for the status
        self.min_margin = math.nan    # m, predicted margin to the nearest keep-away circle, for the status
        self.threat_tracks = []       # the traffic.Track of the threats found at the last replan
        self.progress_mark = -math.inf  # progression at the last real progress, and when (stall detector)
        self.progress_time = -math.inf
        self.forced_until = -math.inf   # until then, the path regardless of the traffic

    @property
    def waiting(self):
        """The plan barely progresses over its horizon: the drone waits for an aircraft to move away."""
        return self.plan.theta[-1] - self.plan.theta[0] < BLOCKED_PROGRESS_M

    def update(self, now, path, xy, velocity, altitude, tracks):
        """The lateral plan to fly at now (s), or None: follow the path (at speed_on_path() if not None).

        xy: drone position (latency compensated); velocity: measured, xyz; altitude: above home;
        tracks: the traffic.Track of every aircraft known."""
        if path.progress > self.progress_mark + STALL_PROGRESS_M or not self.active:
            self.progress_mark, self.progress_time = path.progress, now
        if self.active and now - self.progress_time > STALL_S:
            self.log.warn(f'Évitement bloqué {STALL_S:.0f} s à {path.progress:.0f} m : le chemin pendant '
                          f'{FORCE_PATH_S:.0f} s, quitte à frôler un avion.')
            self.stop(path.progress)
            self.forced_until = now + FORCE_PATH_S
            self.progress_mark, self.progress_time = path.progress, now
        if now < self.forced_until:
            return None
        if now - self.last_replan >= self.replan_s:
            self.last_replan = now
            near_path = float(np.hypot(*(self.planner.ref.at(np.array([path.progress]))[0][0] - xy))) <= RETURN_M
            obstacles, threat_count, min_margin, path_margin = self._threats(now, path.progress, xy, velocity,
                                                                             altitude, tracks)
            if obstacles:
                clear = ENGAGE_MARGIN_M + (ENGAGE_HYSTERESIS_M if self.active else 0.0)
                back = not self.active or near_path
                if back and min(path_margin, min_margin if math.isfinite(min_margin) else math.inf) >= clear:
                    obstacles = []
            if not obstacles:
                self.stop(path.progress)
                self.min_margin = min_margin   # still reported: the nearest aircraft, even far away
                return None
            self.threat_count, self.min_margin = threat_count, min_margin
            if not self.active:
                self.active = True
                self.log.info(f'Évitement : {len(obstacles)} avion(s) menace(nt).')
            self.profile = None
            if float(np.hypot(*(self.planner.ref.at(np.array([path.progress]))[0][0] - xy))) <= TIMING_PATH_M:
                aircraft = [(track.predict(now, timing.HORIZON_S, timing.DT), track.last.horizontal_keep_away)
                            for track in self.threat_tracks]
                self.profile = timing.choose(self.planner.ref, path.progress, float(np.hypot(*velocity[:2])),
                                             aircraft, now)
            if self.profile is not None:
                self.plan = None                    # on the path, at the profile's speed
                self.planner.previous = None
            else:
                self._replan(now, path, xy, velocity[:2], obstacles)
        if self.plan is None or not self.plan.covers(now):
            return None
        return self.plan

    def speed_on_path(self, now):
        """(speed, acceleration) along the path from the timing profile, or None: the path's own."""
        if self.profile is None or self.plan is not None or not self.profile.covers(now):
            return None
        return self.profile.at(now)

    def stop(self, progress):
        """No aircraft threatens any more (or avoidance is switched off, or the path ends): back to
        the path. progress: where the drone is on the path, for the log."""
        self.plan = None
        self.profile = None
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
        else:
            self.log.info(f"Fin d'évitement à {progress:.0f} m (temps seul).")

    def _threats(self, now, progress, xy, velocity, altitude, tracks):
        """(obstacles, number of threats, min margin, path margin). The threats: aircraft whose predicted path
        comes within threat_range of the drone or of the path ahead, in vertical overlap with the
        drone's predicted altitude; the obstacles: the MAX_OBSTACLES nearest. The margin: predicted
        distance from the nearest keep-away circle to what the drone will fly (the plan, else the path);
        the path margin: the same along the path at its own speeds."""
        t = np.arange(local_planner.STEPS + 1) * local_planner.DT
        altitudes = np.clip(altitude + velocity[2] * t, 0.0, None)
        speed = max(float(np.hypot(*velocity[:2])), 1.0)
        ahead = self.planner.ref.at(progress + t * speed)[0]
        flown = ahead
        if self.plan is not None and self.plan.covers(now):
            flown = np.array([self.plan.at(now + dt)[0] for dt in t])
        on_path = self._path_ahead(progress)
        found, margin, path_margin = [], math.inf, math.inf
        for track in tracks:
            if track.age(now) > traffic.STALE_S:
                continue
            last = track.last
            if not np.any(np.abs(altitudes - last.altitude) <= last.vertical_keep_away + VERTICAL_MARGIN_M):
                continue
            centers = track.predict(now, local_planner.HORIZON_S, local_planner.DT)
            margin = min(margin, float(np.min(np.hypot(*(centers - flown).T))) - last.horizontal_keep_away)
            path_margin = min(path_margin, float(np.min(np.hypot(*(centers - on_path).T))) - last.horizontal_keep_away)
            near = min(float(np.min(np.hypot(*(centers - ahead).T))), float(np.hypot(*(centers[0] - xy))))
            if near < self.threat_range:
                found.append((near, local_planner.Obstacle(centers, last.horizontal_keep_away), track))
        found.sort(key=lambda f: f[0])
        self.threat_tracks = [track for _, _, track in found]
        return ([ob for _, ob, _ in found[:MAX_OBSTACLES]], len(found), margin if math.isfinite(margin) else math.nan,
                path_margin)

    def _path_ahead(self, progress):
        """Where the path puts the drone over the horizon, at the path's own speeds, every DT."""
        theta = [float(progress)]
        for _ in range(local_planner.STEPS):
            theta.append(theta[-1] + float(self.planner.ref.at(np.array([theta[-1]]))[2][0]) * local_planner.DT)
        return self.planner.ref.at(np.array(theta))[0]

    def _replan(self, now, path, xy, velocity_xy, obstacles):
        """A new lateral plan through the next gate; when it fails, the previous one stays (while it covers
        now), then the path."""
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
