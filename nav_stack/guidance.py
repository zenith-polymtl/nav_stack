#!/usr/bin/env python3
"""Guidance node: the only node that commands the drone during the Task 1 navigation.

Contract and messages: context_research/architecure/Task-1/guidance.md (aeac-2027). In short, the mode
nodes say where to go (laps_nav, search_nav, return_nav: a NavIntent, a path published once;
pilot_nav: a VelocityIntent at 20 Hz), state_machine_t1 says which one is active (NavMode), and this
node streams the GUIDED setpoint to mavros and reports on topics.GUIDANCE_STATUS.

Path modes (LAPS, SEARCH, RETURN): the last NavIntent of the active mode is followed by progression,
not by time (intent_path.IntentPath, the window of the lap node that flew the 2026-10 SITL
campaigns): setpoint = the path at the drone's progression (position, velocity, acceleration), without
lookahead. A new intent_id restarts from the start of the new path, which must be within
max_start_distance_m of the drone; the same intent_id republished changes nothing. Each mode keeps its
own path and progression: after a PILOT interlude, LAPS resumes where it was, nothing skipped. At the
end of the path, the last point is held at zero velocity (DONE).

Pilot mode (PILOT): the VelocityIntent is flown as a velocity setpoint (FRAME_BODY_FLU turned by the
yaw). Exactly zero (x = y = z = 0), or no VelocityIntent for pilot_timeout_s: the drone brakes, then
holds the point where it stopped (yaw_rate still followed).

Traffic avoidance (path modes): from topics.TRAFFIC (comp_client). Every replan_s, if an aircraft
threatens (its predicted path within threat_range_m of the drone or of the path ahead, in vertical
overlap), local_planner (the convex MPC of the 2026-10 SITL campaigns, Clarabel) plans the next 10 s:
follow the path by progression, stay inside the flight boundary of the site file, pass the next gate
(its tolerance minus 1 m), stay out of the predicted keep-away circles by margin_hard_m (almost hard)
or margin_soft_m (preferred). The plan is flown horizontally (altitude from the path) until no
aircraft threatens; when a replan fails, the previous plan is kept while it covers the present, else
the path. AVOIDING while a plan is flown, BLOCKED when that plan waits before a gate. Without
boundary.* in the site file, avoidance is off. `ros2 param set /guidance avoidance false` switches it
off from the ground. Not yet in PILOT mode (threats 0, min_margin_m NaN there). Why each choice: the
lap node of the nav_stack dev branch and laps_navigation_sitl_research.md (aeac-2027, before d1cfce7).

Modes: the node never arms and never changes the flight mode. Disarmed or not in GUIDED: silent
(NOT_GUIDED). NavMode NONE: silent. Stale pose (pose_timeout_s): silent, ArduPilot holds after
GUID_TIMEOUT.

Tracking guard (from the lap node): beyond max_tracking_error_m from the setpoint, the position alone
is sent; the feedforward comes back once the drone is within TRACKING_RESTORE_M and the progression
has moved. Without it, a velocity feedforward that the position no longer justifies sends the drone
away in a straight line (ArduPilot bounds the position correction, not the feedforward).

Parameters (config/hexa.yaml, section guidance): control_rate_hz, latency_compensation,
uplink_delay_s, max_start_distance_m, max_tracking_error_m, pilot_timeout_s, pose_timeout_s, avoidance,
replan_s, threat_range_m, margin_hard_m, margin_soft_m. From the site file: boundary.lat/lon.
"""
import math

import numpy as np
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import HomePosition, PositionTarget, State

from custom_interfaces.msg import GuidanceStatus, NavIntent, NavMode, TrafficArray, VelocityIntent
from tools import topics

from nav_stack import local_planner, traffic
from nav_stack.intent_path import Gate, IntentPath

# mavros, package tiers : ses noms restent ici (ARCHITECTURE.md, règles de code)
MAVROS_POSE = '/mavros/local_position/pose'
MAVROS_VELOCITY = '/mavros/local_position/velocity_local'
MAVROS_STATE = '/mavros/state'
MAVROS_HOME = '/mavros/home_position/home'
MAVROS_SETPOINT = '/mavros/setpoint_raw/local'

GUIDED = 'GUIDED'
LOCAL_FRAME = 'map'   # mavros local frame
PATH_MODES = (NavMode.LAPS, NavMode.SEARCH, NavMode.RETURN)
MODE_NAMES = {NavMode.NONE: 'NONE', NavMode.LAPS: 'LAPS', NavMode.SEARCH: 'SEARCH',
              NavMode.RETURN: 'RETURN', NavMode.PILOT: 'PILOT'}

# Setpoint type masks (mavros converts ENU -> NED, yaw rate included)
MASK_PATH = PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE   # position, velocity, acceleration
MASK_HOLD = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
             | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
             | PositionTarget.IGNORE_YAW)                                 # position and yaw rate
MASK_VELOCITY = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                 | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                 | PositionTarget.IGNORE_YAW)                             # velocity and yaw rate

STATUS_RATE_HZ = 10.0        # topics.GUIDANCE_STATUS (guidance.md §4)
SUMMARY_RATE_HZ = 1.0        # topics.GUIDANCE_SUMMARY, to the ground
MAX_DT = 0.25                # s, a longer gap between ticks does not open the progression window wider
MAX_EXTRAPOLATION_S = 0.3    # latency compensation never extrapolates further
END_DISTANCE_M = 1.0         # progression this close to the end: path done
TRACKING_RESTORE_M = 8.0     # tracking guard: feedforward back below this error...
RESTORE_PROGRESS_M = 1.0     # ...once the progression has moved this much since the loss
HOLD_CAPTURE_SPEED = 0.3     # m/s: holding, the drone brakes (zero velocity) down to this, then holds its point
GATE_MARGIN_M = 1.0          # the planner's gate is the NavGate tolerance minus this
MAX_OBSTACLES = 4            # nearest threatening aircraft given to the planner
VERTICAL_MARGIN_M = 3.0      # an aircraft counts if |altitude difference| <= its vertical keep-away + this
BLOCKED_PROGRESS_M = 1.0     # a plan that progresses less than this over its horizon waits: BLOCKED
RUNTIME_PARAMETERS = ('avoidance',)

# ArduPilot LATLON_TO_M (AP_Math/definitions.h), per degree: the same flat-earth formula as convert
METERS_PER_DEGREE = 0.011131884502145034 * 1e7

# Latched (guidance.md §4). Depth 10: several mode nodes share NAV_INTENT, each one's last intent arrives.
LATCHED = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
STATE_QOS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def path_from_intent(msg, home_z):
    """IntentPath in the local frame: NavPoint z is a height above home."""
    points = [(p.s, p.x, p.y, p.z + home_z, p.vx, p.vy, p.vz, p.ax, p.ay) for p in msg.points]
    gates = [Gate(g.index, g.s_apex, g.x, g.y, g.tolerance_m) for g in msg.gates]
    return IntentPath(msg.intent_id, points, gates)


def to_local(lat, lon, home):
    """lat/lon -> mavros local frame (x east, y north), from the mavros home (as convert does)."""
    north = (lat - home.geo.latitude) * METERS_PER_DEGREE
    east = (lon - home.geo.longitude) * METERS_PER_DEGREE * math.cos(math.radians((lat + home.geo.latitude) / 2))
    return np.array([home.position.x + east, home.position.y + north])


def planner_reference(path):
    """local_planner.Reference of a path. Its tangent comes from the velocity; where the path asks
    for less than 0.5 m/s (a stop), the geometric direction is used instead, at 0.5 m/s."""
    xy = path.xyz[:, :2]
    velocity = path.velocity[:, :2].copy()
    direction = np.gradient(xy, axis=0)
    norm = np.maximum(np.linalg.norm(direction, axis=1), 1e-9)
    slow = np.linalg.norm(velocity, axis=1) < 0.5
    velocity[slow] = 0.5 * direction[slow] / norm[slow, None]
    return local_planner.Reference(path.s, xy, velocity)


def yaw_of(q):
    """Yaw (rad, ENU, counterclockwise from east) of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Guidance(Node):
    def __init__(self):
        super().__init__('guidance')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('latency_compensation', True)
        self.declare_parameter('uplink_delay_s', 0.02)
        self.declare_parameter('max_start_distance_m', 15.0)
        self.declare_parameter('max_tracking_error_m', 25.0)
        self.declare_parameter('pilot_timeout_s', 0.5)
        self.declare_parameter('pose_timeout_s', 0.5)
        self.period = 1.0 / self.get_parameter('control_rate_hz').value
        self.latency_compensation = self.get_parameter('latency_compensation').value
        self.uplink_delay = self.get_parameter('uplink_delay_s').value
        self.max_start_distance = self.get_parameter('max_start_distance_m').value
        self.max_tracking_error = self.get_parameter('max_tracking_error_m').value
        self.pilot_timeout = self.get_parameter('pilot_timeout_s').value
        self.pose_timeout = self.get_parameter('pose_timeout_s').value
        self.declare_parameter('avoidance', True)
        self.declare_parameter('replan_s', 0.25)
        self.declare_parameter('threat_range_m', 120.0)
        self.declare_parameter('margin_hard_m', 3.0)
        self.declare_parameter('margin_soft_m', 15.0)
        self.declare_parameter('boundary.lat', Parameter.Type.DOUBLE_ARRAY)   # from the site file
        self.declare_parameter('boundary.lon', Parameter.Type.DOUBLE_ARRAY)
        self.avoidance = self.get_parameter('avoidance').value
        self.replan_s = self.get_parameter('replan_s').value
        self.threat_range = self.get_parameter('threat_range_m').value
        self.margin_hard = self.get_parameter('margin_hard_m').value
        self.margin_soft = self.get_parameter('margin_soft_m').value

        # Autopilot
        self.fcu_mode = ''
        self.armed = False
        self.home = None
        self.pose = None              # last PoseStamped
        self.velocity = np.zeros(3)   # measured, ENU

        # Navigation
        self.nav_mode = NavMode.NONE
        self.intents = {}             # NavMode constant -> last NavIntent of that mode
        self.paths = {}               # NavMode constant -> IntentPath being followed in that mode
        self.rejected = set()         # (mode, intent_id) refused: not retried until a new intent_id
        self.pilot = None             # last VelocityIntent
        self.pilot_received = None    # Time of its reception
        self.pilot_lost = False

        # Output
        self.state = GuidanceStatus.NOT_GUIDED
        self.last_tick = None
        self.hold_point = None        # xyz held, None while not holding
        self.tracking_lost = False
        self.progress_at_loss = 0.0
        self.warned_no_pose = False

        # Traffic avoidance
        self.boundary_gps = self.read_boundary()   # (lat, lon) vertices, or None: avoidance off
        self.tracks = {}              # aircraft index -> traffic.Track
        self.planners = {}            # NavMode constant -> local_planner.LocalPlanner of its path
        self.plan = None              # local_planner.Plan being flown, or None
        self.last_replan = -math.inf  # s, node clock
        self.avoiding = False
        self.fail_streak = 0          # failed replans in a row (logs)
        self.threat_count = 0
        self.min_margin = math.nan    # m, predicted margin to the nearest keep-away circle
        self.warned_traffic_no_home = False
        self.add_on_set_parameters_callback(self.callback_parameters)

        self.setpoint_pub = self.create_publisher(PositionTarget, MAVROS_SETPOINT, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(GuidanceStatus, topics.GUIDANCE_STATUS, 10)
        self.summary_pub = self.create_publisher(GuidanceStatus, topics.GUIDANCE_SUMMARY, 10)

        self.create_subscription(State, MAVROS_STATE, self.callback_state, STATE_QOS)
        self.create_subscription(HomePosition, MAVROS_HOME, self.callback_home, STATE_QOS)
        self.create_subscription(PoseStamped, MAVROS_POSE, self.callback_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, MAVROS_VELOCITY, self.callback_velocity, qos_profile_sensor_data)
        self.create_subscription(NavMode, topics.NAV_MODE, self.callback_nav_mode, LATCHED)
        self.create_subscription(NavIntent, topics.NAV_INTENT, self.callback_intent, LATCHED)
        self.create_subscription(VelocityIntent, topics.NAV_PILOT_VELOCITY, self.callback_pilot, 10)
        self.create_subscription(TrafficArray, topics.TRAFFIC, self.callback_traffic, 10)

        self.create_timer(self.period, self.tick)
        self.create_timer(1.0 / STATUS_RATE_HZ, lambda: self.publish_status(self.status_pub))
        self.create_timer(1.0 / SUMMARY_RATE_HZ, lambda: self.publish_status(self.summary_pub))

        avoidance = 'avec' if self.avoidance and self.boundary_gps else 'sans'
        self.get_logger().info(f'Node guidance prête ({avoidance} évitement du trafic), en attente de NavMode.')

    def read_boundary(self):
        """Flight boundary (lat, lon) from the site file, or None: avoidance then off."""
        try:
            return list(zip(self.get_parameter('boundary.lat').value, self.get_parameter('boundary.lon').value))
        except ParameterUninitializedException:
            if self.avoidance:
                self.get_logger().warn('Pas de limite de vol dans le fichier du site (boundary.*) : évitement du trafic désactivé.')
            return None

    def callback_parameters(self, params):
        """At run time, only avoidance changes (from the ground)."""
        if any(p.name not in RUNTIME_PARAMETERS for p in params):
            return SetParametersResult(successful=False, reason='seul avoidance se change en vol')
        self.avoidance = bool(params[0].value)
        if not self.avoidance:
            self.stop_avoiding()
        self.get_logger().warn(f'Évitement du trafic {"activé" if self.avoidance else "coupé"} (paramètre avoidance).')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ inputs
    def callback_state(self, msg):
        self.fcu_mode = msg.mode
        self.armed = msg.armed

    def callback_home(self, msg):
        if any(math.isnan(v) for v in (msg.position.x, msg.position.y, msg.position.z)):
            return   # no EKF origin yet
        self.home = msg

    def callback_pose(self, msg):
        self.pose = msg

    def callback_velocity(self, msg):
        v = msg.twist.linear
        self.velocity = np.array([v.x, v.y, v.z])

    def callback_nav_mode(self, msg):
        if msg.mode not in MODE_NAMES:
            self.get_logger().error(f'NavMode inconnu ({msg.mode}) : ignoré.')
            return
        if msg.mode != self.nav_mode:
            self.get_logger().info(f'Mode de navigation : {MODE_NAMES[self.nav_mode]} -> {MODE_NAMES[msg.mode]}.')
        self.stop_avoiding()
        self.nav_mode = msg.mode
        self.hold_point = None
        self.last_tick = None
        self.tracking_lost = False

    def callback_intent(self, msg):
        if msg.mode not in PATH_MODES:
            self.get_logger().error(f'NavIntent de mode {msg.mode} : seuls LAPS, SEARCH et RETURN ont un chemin. Ignorée.')
            return
        previous = self.intents.get(msg.mode)
        if previous is None or previous.intent_id != msg.intent_id:
            self.get_logger().info(
                f'NavIntent reçue : {MODE_NAMES[msg.mode]} n°{msg.intent_id}, {len(msg.points)} points, '
                f'{len(msg.gates)} portes.')
        self.intents[msg.mode] = msg

    def callback_traffic(self, msg):
        """Keeps the last updates of each aircraft, in the local frame."""
        if self.home is None:
            if not self.warned_traffic_no_home:
                self.warned_traffic_no_home = True
                self.get_logger().warn('Trafic reçu avant le home de mavros : ignoré tant que le home manque.')
            return
        now = self.now_s()
        for a in msg.aircraft:
            stamp = Time.from_msg(a.stamp).nanoseconds * 1e-9 if (a.stamp.sec or a.stamp.nanosec) else now
            track = self.tracks.get(a.index)
            if track is None or track.name != a.name:
                track = self.tracks[a.index] = traffic.Track(a.name)
                self.get_logger().info(
                    f'Trafic : {a.name} (#{a.index}), keep-away {a.horizontal_keep_away:.0f} m, {a.altitude_agl:.0f} m AGL.')
            track.add(traffic.Update(stamp, to_local(a.latitude, a.longitude, self.home), a.altitude_agl,
                                     a.speed, a.course_deg, a.horizontal_keep_away, a.vertical_keep_away))

    def callback_pilot(self, msg):
        self.pilot = msg
        self.pilot_received = self.get_clock().now()

    # ------------------------------------------------------------------ the loop
    def tick(self):
        now = self.get_clock().now()
        if not (self.armed and self.fcu_mode == GUIDED):
            if self.state != GuidanceStatus.NOT_GUIDED:
                reason = 'désarmé' if not self.armed else f'mode {self.fcu_mode}'
                self.get_logger().warn(f'Pilote : {reason}. Guidance se tait.')
            self.set_state(GuidanceStatus.NOT_GUIDED)
            self.hold_point = None
            self.last_tick = None
            return
        if self.pose is None or (now - Time.from_msg(self.pose.header.stamp)).nanoseconds * 1e-9 > self.pose_timeout:
            if not self.warned_no_pose:
                self.warned_no_pose = True
                self.get_logger().warn('Pas de pose récente de mavros : guidance se tait.')
            self.last_tick = None
            return
        self.warned_no_pose = False

        dt = self.period if self.last_tick is None else min((now - self.last_tick).nanoseconds * 1e-9, MAX_DT)
        self.last_tick = now
        p = self.pose.pose.position
        measured = np.array([p.x, p.y, p.z])

        if self.nav_mode == NavMode.NONE:
            self.set_state(GuidanceStatus.NO_INTENT)   # nothing active: silent
        elif self.nav_mode == NavMode.PILOT:
            self.fly_pilot(now, measured)
        else:
            self.fly_path(now, measured, dt)

    def fly_path(self, now, measured, dt):
        path = self.select_path(measured)
        if path is None:
            self.hold(measured)
            self.set_state(GuidanceStatus.NO_INTENT)
            return
        self.hold_point = None
        position = measured
        if self.latency_compensation:
            age = (now - Time.from_msg(self.pose.header.stamp)).nanoseconds * 1e-9
            position = measured + self.velocity * min(max(age, 0.0) + self.uplink_delay, MAX_EXTRAPOLATION_S)

        path.advance(position[:2], float(np.hypot(*self.velocity[:2])), dt)
        for gate in path.update_gates(measured[:2]):
            self.get_logger().error(
                f'Porte du WP n°{gate.index} laissée derrière sans y passer à moins de {gate.tolerance:.0f} m.')

        if path.remaining < END_DISTANCE_M:
            final = path.sample(path.length)[0]
            self.publish_setpoint(final, np.zeros(3), np.zeros(3), MASK_PATH)
            if self.state != GuidanceStatus.DONE:
                self.get_logger().info(
                    f'{MODE_NAMES[self.nav_mode]} n°{path.intent_id} : fin du chemin, point final tenu '
                    f'({path.gates_passed}/{len(path.gates)} portes passées).')
            self.set_state(GuidanceStatus.DONE)
            return

        pos, vel, acc = path.sample(path.progress)
        state = GuidanceStatus.FOLLOWING
        local = self.avoid(path, position, measured[2] - self.home.position.z)
        if local is not None:     # the local plan: horizontal from it, vertical from the path
            pos[:2], vel[:2], acc[:2] = local
            waiting = self.plan.theta[-1] - self.plan.theta[0] < BLOCKED_PROGRESS_M
            state = GuidanceStatus.BLOCKED if waiting and path.next_gate is not None else GuidanceStatus.AVOIDING
        if not self.feedforward_allowed(pos[:2], measured[:2], path.progress):
            vel, acc = np.zeros(3), np.zeros(3)
        self.publish_setpoint(pos, vel, acc, MASK_PATH)
        self.set_state(state)

    def select_path(self, measured):
        """The path of the active mode, built from its last NavIntent when that one is new."""
        mode = self.nav_mode
        msg = self.intents.get(mode)
        if msg is None:
            return None
        current = self.paths.get(mode)
        if current is not None and current.intent_id == msg.intent_id:
            return current
        # A new intent_id replaces the path; until it can be followed, the drone holds its position
        if (mode, msg.intent_id) in self.rejected or self.home is None:   # NavPoint z is above home
            return None
        try:
            path = path_from_intent(msg, self.home.position.z)
        except ValueError as e:
            self.reject(mode, msg.intent_id, str(e))
            return None
        distance = float(np.hypot(*(path.start - measured[:2])))
        if distance > self.max_start_distance:
            self.reject(mode, msg.intent_id,
                        f'drone à {distance:.1f} m du premier point (max {self.max_start_distance:.0f} m)')
            return None
        if path.dropped:
            self.get_logger().warn(f'NavIntent n°{msg.intent_id} : {path.dropped} points à abscisse s non croissante retirés.')
        self.paths[mode] = path
        self.tracking_lost = False
        self.stop_avoiding()
        self.planners.pop(mode, None)
        if self.boundary_gps:
            boundary = [to_local(lat, lon, self.home) for lat, lon in self.boundary_gps]
            self.planners[mode] = local_planner.LocalPlanner(planner_reference(path), boundary,
                                                             self.margin_hard, self.margin_soft)
        self.get_logger().info(
            f'{MODE_NAMES[mode]} n°{msg.intent_id} suivie : {path.length:.0f} m, {len(path.gates)} portes, '
            f'drone à {distance:.1f} m du premier point.')
        return path

    def reject(self, mode, intent_id, reason):
        self.rejected.add((mode, intent_id))
        self.get_logger().error(f'{MODE_NAMES[mode]} n°{intent_id} refusée : {reason}. Le drone tient sa position.')

    def fly_pilot(self, now, measured):
        v = self.pilot
        fresh = v is not None and (now - self.pilot_received).nanoseconds * 1e-9 <= self.pilot_timeout
        if not fresh:
            if not self.pilot_lost:
                self.pilot_lost = True
                self.get_logger().warn('Mode PILOT sans VelocityIntent récente (pilot_nav) : le drone tient sa position.')
            self.hold(measured)
            self.set_state(GuidanceStatus.HOLDING)
            return
        if self.pilot_lost:
            self.pilot_lost = False
            self.get_logger().info('VelocityIntent de nouveau reçues : vitesse du pilote suivie.')

        if v.x == 0.0 and v.y == 0.0 and v.z == 0.0:
            self.hold(measured, v.yaw_rate)
            self.set_state(GuidanceStatus.HOLDING)
            return
        if v.frame == VelocityIntent.FRAME_BODY_FLU:
            yaw = yaw_of(self.pose.pose.orientation)
            c, s = math.cos(yaw), math.sin(yaw)
            velocity = np.array([c * v.x - s * v.y, s * v.x + c * v.y, v.z])
        elif v.frame == VelocityIntent.FRAME_LOCAL_ENU:
            velocity = np.array([v.x, v.y, v.z])
        else:
            self.get_logger().error(f'VelocityIntent de repère inconnu ({v.frame}) : tenue de position.',
                                    throttle_duration_sec=2.0)
            self.hold(measured)
            self.set_state(GuidanceStatus.HOLDING)
            return
        self.hold_point = None
        self.publish_setpoint(np.zeros(3), velocity, np.zeros(3), MASK_VELOCITY, v.yaw_rate)
        self.set_state(GuidanceStatus.FOLLOWING)

    # ------------------------------------------------------------------ traffic avoidance
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def avoid(self, path, position, altitude):
        """The local plan's (pos, vel, acc) xy at this instant when an aircraft threatens, else None."""
        planner = self.planners.get(self.nav_mode)
        if not self.avoidance or planner is None:
            return None
        now = self.now_s()
        if now - self.last_replan >= self.replan_s:
            self.last_replan = now
            obstacles = self.threats(now, path, planner, position[:2], altitude)
            if not obstacles:
                self.stop_avoiding()
                return None
            if not self.avoiding:
                self.avoiding = True
                self.get_logger().info(f'Évitement : {len(obstacles)} avion(s) menace(nt), plans locaux.')
            self.replan(now, path, planner, position[:2], obstacles)
        if self.plan is None or not self.plan.covers(now):
            return None
        return self.plan.at(now)

    def replan(self, now, path, planner, xy, obstacles):
        gate = path.next_gate
        if gate is not None:
            planner.gate_tol = max(gate.tolerance - GATE_MARGIN_M, 1.0)
        plan = planner.plan(path.progress, xy, self.velocity[:2], obstacles,
                            None if gate is None else (gate.s_apex, gate.xy), now)
        if plan is not None:
            if self.fail_streak > 1:
                self.get_logger().info(f'Plan local rétabli après {self.fail_streak} échecs de suite.')
            self.fail_streak = 0
            self.plan = plan
            return
        # une rafale d'échecs : un seul WARN au début (le plan précédent reste valide 9,5 s)
        self.fail_streak += 1
        if self.fail_streak == 1:
            self.get_logger().warn("Plan local raté : le précédent reste en vigueur s'il est encore valide.")

    def threats(self, now, path, planner, xy, altitude):
        """Aircraft whose predicted path comes near the drone or the path ahead, in vertical overlap
        with the drone's predicted altitude: the MAX_OBSTACLES nearest. Also updates threat_count and
        min_margin (predicted distance to the nearest keep-away circle, along the plan or the path)."""
        t = np.arange(local_planner.STEPS + 1) * local_planner.DT
        altitudes = np.clip(altitude + self.velocity[2] * t, 0.0, None)
        speed = max(float(np.hypot(*self.velocity[:2])), 1.0)
        ahead = planner.ref.at(path.progress + t * speed)[0]
        flown = ahead
        if self.plan is not None and self.plan.covers(now):
            flown = np.array([self.plan.at(now + dt)[0] for dt in t])
        found, margin = [], math.inf
        for track in self.tracks.values():
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
        self.threat_count = len(found)
        self.min_margin = margin if math.isfinite(margin) else math.nan
        return [ob for _, ob in found[:MAX_OBSTACLES]]

    def stop_avoiding(self):
        self.plan = None
        self.last_replan = -math.inf
        self.threat_count = 0
        self.min_margin = math.nan
        if not self.avoiding:
            return
        self.avoiding = False
        planner = self.planners.get(self.nav_mode)
        if planner is not None and planner.solve_times:
            times = np.array(planner.solve_times) * 1000
            self.get_logger().info(
                f'Fin d\'évitement à {self.paths[self.nav_mode].progress:.0f} m. Plans : {len(times)}, '
                f'{np.median(times):.0f} ms en médiane, {times.max():.0f} ms au pire, {planner.failures} ratés.')

    def hold(self, measured, yaw_rate=0.0):
        """Brakes (zero velocity) down to HOLD_CAPTURE_SPEED, then holds the point where it stopped."""
        if self.hold_point is None:
            if np.linalg.norm(self.velocity) > HOLD_CAPTURE_SPEED:
                self.publish_setpoint(np.zeros(3), np.zeros(3), np.zeros(3), MASK_VELOCITY, yaw_rate)
                return
            self.hold_point = measured.copy()
        self.publish_setpoint(self.hold_point, np.zeros(3), np.zeros(3), MASK_HOLD, yaw_rate)

    def feedforward_allowed(self, setpoint_xy, measured_xy, progress):
        """Tracking guard, with its hysteresis: False while the drone is lost from its setpoint."""
        error = float(np.linalg.norm(setpoint_xy - measured_xy))
        if not self.tracking_lost and error > self.max_tracking_error:
            self.tracking_lost = True
            self.progress_at_loss = progress
            self.get_logger().error(f'Drone à {error:.0f} m de sa consigne : feedforward coupé, position seule.')
        elif self.tracking_lost and error < TRACKING_RESTORE_M and progress > self.progress_at_loss + RESTORE_PROGRESS_M:
            self.tracking_lost = False
            self.get_logger().info('Drone revenu sur sa consigne : feedforward rétabli.')
        return not self.tracking_lost

    # ------------------------------------------------------------------ outputs
    def set_state(self, state):
        self.state = state

    def publish_setpoint(self, pos, vel, acc, mask, yaw_rate=0.0):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = LOCAL_FRAME
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED   # mavros converts ENU -> NED
        sp.type_mask = mask
        sp.position.x, sp.position.y, sp.position.z = (float(v) for v in pos)
        sp.velocity.x, sp.velocity.y, sp.velocity.z = (float(v) for v in vel)
        sp.acceleration_or_force.x, sp.acceleration_or_force.y, sp.acceleration_or_force.z = (float(v) for v in acc)
        sp.yaw_rate = float(yaw_rate)
        self.setpoint_pub.publish(sp)

    def publish_status(self, publisher):
        msg = GuidanceStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = LOCAL_FRAME
        msg.mode = self.nav_mode
        msg.state = self.state
        msg.threats = min(self.threat_count, 255)
        msg.min_margin_m = float(self.min_margin)
        path = self.paths.get(self.nav_mode)
        if path is not None:
            msg.intent_id = path.intent_id
            msg.progress_m = float(path.progress)
            msg.length_m = float(path.length)
            msg.gates_passed = path.gates_passed
            if self.pose is not None:
                p = self.pose.pose.position
                msg.cross_track_m = path.cross_track(path.progress, np.array([p.x, p.y]))
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Guidance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
