"""What guidance flies in a path mode, without ROS: one IntentPath, its avoidance and its tracking guard.

guidance keeps one PathFollower per path mode (LAPS, SEARCH, RETURN) and calls step() every tick of
that mode; the SITL bench calls the same step(), so what it measures is the flight code. ROS, the
modes, the NavIntent checks and the setpoint messages stay in guidance.py.

Frame: mavros local ENU (x east, y north, z up), meters, seconds.
"""
import numpy as np

from nav_stack.avoidance import Avoidance

MAX_EXTRAPOLATION_S = 0.3    # latency compensation never extrapolates further
END_DISTANCE_M = 1.0         # progression this close to the end: path done
TRACKING_RESTORE_M = 8.0     # tracking guard: feedforward back below this error...
RESTORE_PROGRESS_M = 1.0     # ...once the progression has moved this much since the loss

# What step() reports (guidance turns them into GuidanceStatus)
FOLLOWING, AVOIDING, BLOCKED, DONE = 'FOLLOWING', 'AVOIDING', 'BLOCKED', 'DONE'


class PathFollower:
    def __init__(self, path, boundary_xy, settings, logger):
        """path: IntentPath; boundary_xy: flight boundary vertices in the local frame, or None (no
        avoidance); settings: guidance parameters (latency_compensation, uplink_delay_s,
        max_tracking_error_m, replan_s, threat_range_m, margin_hard_m, margin_soft_m)."""
        self.path = path
        self.log = logger
        self.latency_compensation = settings['latency_compensation']
        self.uplink_delay = settings['uplink_delay_s']
        self.max_tracking_error = settings['max_tracking_error_m']
        self.avoidance = None
        if boundary_xy is not None:
            self.avoidance = Avoidance(path, boundary_xy, settings['replan_s'], settings['threat_range_m'],
                                       settings['margin_hard_m'], settings['margin_soft_m'], logger)
        self.tracking_lost = False
        self.progress_at_loss = 0.0
        self.done = False

    def step(self, now, dt, measured, velocity, pose_age, home_z, tracks, avoid=True):
        """(pos, vel, acc, state) to send now. now: s; dt: s since the last tick (bounded by the
        caller); measured, velocity: xyz; pose_age: s; home_z: z of home; tracks: traffic.Track of
        every aircraft known; avoid: avoidance switched on."""
        path = self.path
        # Where the drone will be when this setpoint reaches the autopilot
        predicted = measured
        if self.latency_compensation:
            predicted = measured + velocity * min(max(pose_age, 0.0) + self.uplink_delay, MAX_EXTRAPOLATION_S)
        path.advance(predicted[:2], float(np.hypot(*velocity[:2])), dt)
        for gate in path.update_gates(measured[:2]):
            self.log.error(f'Porte du WP n°{gate.index} laissée derrière sans y passer à moins de {gate.tolerance:.0f} m.')

        if path.remaining < END_DISTANCE_M:
            self.stop_avoidance()
            final = path.sample(path.length)[0]
            if not self.done:
                self.log.info(f'Chemin n°{path.intent_id} : fin, point final tenu '
                              f'({path.gates_passed}/{len(path.gates)} portes passées).')
            self.done = True
            return final, np.zeros(3), np.zeros(3), DONE

        pos, vel, acc = path.sample(path.progress)
        state = FOLLOWING
        if avoid and self.avoidance is not None:
            plan = self.avoidance.update(now, path, predicted[:2], velocity, measured[2] - home_z, tracks)
            if plan is not None:   # horizontal from the local plan, vertical from the path
                pos[:2], vel[:2], acc[:2] = plan.at(now)
                state = BLOCKED if self.avoidance.waiting and path.next_gate is not None else AVOIDING
        if not self.tracking_guard(pos[:2], measured[:2], path.progress):
            vel, acc = np.zeros(3), np.zeros(3)
        return pos, vel, acc, state

    def stop_avoidance(self):
        """Back to the path, if it was avoiding (logs the end of the avoidance)."""
        if self.avoidance is not None:
            self.avoidance.stop(self.path.progress)

    def reset_guard(self):
        self.tracking_lost = False

    def tracking_guard(self, setpoint_xy, measured_xy, progress):
        """False while the drone is lost from its setpoint: then the position alone is sent.

        Beyond max_tracking_error_m, a velocity feedforward that the position no longer justifies
        sends the drone away in a straight line (ArduPilot bounds the position correction, not the
        feedforward). It comes back once the drone is within TRACKING_RESTORE_M and the progression
        has moved RESTORE_PROGRESS_M since the loss."""
        error = float(np.linalg.norm(setpoint_xy - measured_xy))
        if not self.tracking_lost and error > self.max_tracking_error:
            self.tracking_lost = True
            self.progress_at_loss = progress
            self.log.error(f'Drone à {error:.0f} m de sa consigne : feedforward coupé, position seule.')
        elif self.tracking_lost and error < TRACKING_RESTORE_M and progress > self.progress_at_loss + RESTORE_PROGRESS_M:
            self.tracking_lost = False
            self.log.info('Drone revenu sur sa consigne : feedforward rétabli.')
        return not self.tracking_lost
