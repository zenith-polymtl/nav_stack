#!/usr/bin/env python3
"""Guidance node: the only node that commands the drone during the Task 1 navigation.

Contract and messages: context_research/architecure/Task-1/guidance.md (aeac-2027). The mode nodes
say where to go, state_machine_t1 says which mode is active (NavMode), and this node streams the
GUIDED setpoint to mavros and reports what it does on topics.GUIDANCE_STATUS.

- Path modes (LAPS, SEARCH, RETURN): the last NavIntent of the mode is followed by progression, not
  by time (intent_path.py). Each mode keeps its own path and progression, so LAPS resumes after a
  PILOT interlude. At the end of the path, the last point is held (DONE). While an aircraft
  threatens, the horizontal setpoint comes from a local plan instead (avoidance.py).
- PILOT: the VelocityIntent of pilot_nav is flown as a velocity setpoint. Exactly zero, or silent for
  pilot_timeout_s: the drone brakes, then holds the point where it stopped.
- The node never arms and never changes the flight mode. It is silent when disarmed or not in GUIDED,
  in NavMode NONE, or without a recent pose (ArduPilot then holds after GUID_TIMEOUT).

Parameters: config/hexa.yaml, section guidance; boundary.lat/lon from the site file (without it, no
avoidance). `ros2 param set /guidance avoidance false` switches avoidance off from the ground.
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

from nav_stack import traffic
from nav_stack.avoidance import Avoidance
from nav_stack.intent_path import Gate, IntentPath

# mavros is a third-party package: its topic names stay here, not in tools.topics (ARCHITECTURE.md)
MAVROS_POSE = '/mavros/local_position/pose'
MAVROS_VELOCITY = '/mavros/local_position/velocity_local'
MAVROS_STATE = '/mavros/state'
MAVROS_HOME = '/mavros/home_position/home'
MAVROS_SETPOINT = '/mavros/setpoint_raw/local'

PARAMETERS = {                     # name: default; config/hexa.yaml explains each one
    'control_rate_hz': 20.0,
    'latency_compensation': True,
    'uplink_delay_s': 0.02,
    'max_start_distance_m': 15.0,
    'max_tracking_error_m': 25.0,
    'pilot_timeout_s': 0.5,
    'pose_timeout_s': 0.5,
    'avoidance': True,             # the only one that changes in flight
    'replan_s': 0.25,
    'threat_range_m': 120.0,
    'margin_hard_m': 3.0,
    'margin_soft_m': 15.0,
}

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

# ArduPilot LATLON_TO_M (AP_Math/definitions.h), per degree: the same flat-earth formula as convert
METERS_PER_DEGREE = 0.011131884502145034 * 1e7

# Latched (guidance.md §4). Depth 10: several mode nodes share NAV_INTENT, each one's last intent arrives.
LATCHED = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class Guidance(Node):
    def __init__(self):
        super().__init__('guidance')
        p = {name: self.declare_parameter(name, default).value for name, default in PARAMETERS.items()}
        self.period = 1.0 / p['control_rate_hz']
        self.latency_compensation = p['latency_compensation']
        self.uplink_delay = p['uplink_delay_s']
        self.max_start_distance = p['max_start_distance_m']
        self.max_tracking_error = p['max_tracking_error_m']
        self.pilot_timeout = p['pilot_timeout_s']
        self.pose_timeout = p['pose_timeout_s']
        self.avoidance = p['avoidance']
        self.avoidance_settings = dict(replan_s=p['replan_s'], threat_range_m=p['threat_range_m'],
                                       margin_hard_m=p['margin_hard_m'], margin_soft_m=p['margin_soft_m'])
        self.declare_parameter('boundary.lat', Parameter.Type.DOUBLE_ARRAY)   # from the site file
        self.declare_parameter('boundary.lon', Parameter.Type.DOUBLE_ARRAY)
        self.boundary_gps = self.read_boundary()   # (lat, lon) vertices, or None: no avoidance

        # Autopilot, from mavros
        self.fcu_mode = ''
        self.armed = False
        self.home = None              # HomePosition, once the EKF has an origin
        self.pose = None              # last PoseStamped
        self.velocity = np.zeros(3)   # measured, ENU
        # What to do, from the mode nodes and comp_client
        self.nav_mode = NavMode.NONE
        self.intents = {}             # path mode -> its last NavIntent
        self.pilot = None             # last VelocityIntent
        self.pilot_received = None    # Time of its reception
        self.tracks = {}              # aircraft index -> traffic.Track, local frame
        # Path modes
        self.paths = {}               # path mode -> IntentPath being followed
        self.avoidances = {}          # path mode -> Avoidance along its path (none without a boundary)
        self.rejected = set()         # (mode, intent_id) refused: not retried until a new intent_id
        self.last_tick = None         # Time of the last path tick, for dt
        self.tracking_lost = False    # tracking guard (update_tracking_guard)
        self.progress_at_loss = 0.0
        # Output
        self.state = GuidanceStatus.NOT_GUIDED
        self.hold_point = None        # xyz held, None while not holding
        # Warnings given once per episode
        self.warned_no_pose = False
        self.warned_traffic_no_home = False
        self.warned_intent_no_home = False
        self.pilot_lost = False
        self.add_on_set_parameters_callback(self.callback_parameters)

        self.setpoint_pub = self.create_publisher(PositionTarget, MAVROS_SETPOINT, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(GuidanceStatus, topics.GUIDANCE_STATUS, 10)
        self.summary_pub = self.create_publisher(GuidanceStatus, topics.GUIDANCE_SUMMARY, 10)

        self.create_subscription(State, MAVROS_STATE, self.callback_state, LATCHED)
        self.create_subscription(HomePosition, MAVROS_HOME, self.callback_home, LATCHED)
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
        if msg.mode == self.nav_mode:
            return   # republished (latched topic, state_machine_t1 restarted): nothing changes
        self.get_logger().info(f'Mode de navigation : {MODE_NAMES[self.nav_mode]} -> {MODE_NAMES[msg.mode]}.')
        self.stop_avoidance(self.nav_mode)
        self.nav_mode = msg.mode
        self.hold_point = None
        self.last_tick = None
        self.tracking_lost = False
        self.pilot_lost = False

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

    def callback_pilot(self, msg):
        self.pilot = msg
        self.pilot_received = self.get_clock().now()

    def callback_traffic(self, msg):
        """Keeps the last updates of each aircraft, in the local frame."""
        if self.home is None:
            if not self.warned_traffic_no_home:
                self.warned_traffic_no_home = True
                self.get_logger().warn('Trafic reçu avant le home de mavros : ignoré tant que le home manque.')
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for a in msg.aircraft:
            stamp = Time.from_msg(a.stamp).nanoseconds * 1e-9 if (a.stamp.sec or a.stamp.nanosec) else now
            track = self.tracks.get(a.index)
            if track is None or track.name != a.name:
                track = self.tracks[a.index] = traffic.Track(a.name)
                self.get_logger().info(
                    f'Trafic : {a.name} (#{a.index}), keep-away {a.horizontal_keep_away:.0f} m, {a.altitude_agl:.0f} m AGL.')
            track.add(traffic.Update(stamp, to_local(a.latitude, a.longitude, self.home), a.altitude_agl,
                                     a.speed, a.course_deg, a.horizontal_keep_away, a.vertical_keep_away))

    def callback_parameters(self, params):
        """In flight, only avoidance changes (from the ground)."""
        if any(p.name != 'avoidance' for p in params):
            return SetParametersResult(successful=False, reason='seul avoidance se change en vol')
        self.avoidance = bool(params[0].value)
        if not self.avoidance:
            self.stop_avoidance(self.nav_mode)
        self.get_logger().warn(f'Évitement du trafic {"activé" if self.avoidance else "coupé"} (paramètre avoidance).')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ the loop
    def tick(self):
        now = self.get_clock().now()
        if not (self.armed and self.fcu_mode == GUIDED):
            if self.state != GuidanceStatus.NOT_GUIDED:
                reason = 'désarmé' if not self.armed else f'mode {self.fcu_mode}'
                self.get_logger().warn(f'Pilote : {reason}. Guidance se tait.')
            self.state = GuidanceStatus.NOT_GUIDED
            self.hold_point = None
            self.last_tick = None
            return
        pose_age = None if self.pose is None else (now - Time.from_msg(self.pose.header.stamp)).nanoseconds * 1e-9
        if pose_age is None or pose_age > self.pose_timeout:
            if not self.warned_no_pose:
                self.warned_no_pose = True
                self.get_logger().warn('Pas de pose récente de mavros : guidance se tait.')
            self.last_tick = None
            return
        self.warned_no_pose = False

        p = self.pose.pose.position
        measured = np.array([p.x, p.y, p.z])
        if self.nav_mode == NavMode.NONE:
            self.state = GuidanceStatus.NO_INTENT   # nothing active: silent
        elif self.nav_mode == NavMode.PILOT:
            self.fly_pilot(now, measured)
        else:
            self.fly_path(now, measured, pose_age)

    # ------------------------------------------------------------------ path modes
    def fly_path(self, now, measured, pose_age):
        dt = self.period if self.last_tick is None else min((now - self.last_tick).nanoseconds * 1e-9, MAX_DT)
        self.last_tick = now
        path = self.select_path(measured)
        if path is None:
            self.hold(measured)
            self.state = GuidanceStatus.NO_INTENT
            return
        self.hold_point = None

        # Where the drone will be when this setpoint reaches the autopilot
        predicted = measured
        if self.latency_compensation:
            predicted = measured + self.velocity * min(max(pose_age, 0.0) + self.uplink_delay, MAX_EXTRAPOLATION_S)
        path.advance(predicted[:2], float(np.hypot(*self.velocity[:2])), dt)
        for gate in path.update_gates(measured[:2]):
            self.get_logger().error(
                f'Porte du WP n°{gate.index} laissée derrière sans y passer à moins de {gate.tolerance:.0f} m.')

        if path.remaining < END_DISTANCE_M:
            self.stop_avoidance(self.nav_mode)
            final = path.sample(path.length)[0]
            self.publish_setpoint(final, np.zeros(3), np.zeros(3), MASK_PATH)
            if self.state != GuidanceStatus.DONE:
                self.get_logger().info(
                    f'{MODE_NAMES[self.nav_mode]} n°{path.intent_id} : fin du chemin, point final tenu '
                    f'({path.gates_passed}/{len(path.gates)} portes passées).')
            self.state = GuidanceStatus.DONE
            return

        pos, vel, acc = path.sample(path.progress)
        state = GuidanceStatus.FOLLOWING
        avoidance = self.avoidances.get(self.nav_mode) if self.avoidance else None
        if avoidance is not None:
            t = now.nanoseconds * 1e-9
            plan = avoidance.update(t, path, predicted[:2], self.velocity, measured[2] - self.home.position.z,
                                    self.tracks.values())
            if plan is not None:   # horizontal from the local plan, vertical from the path
                pos[:2], vel[:2], acc[:2] = plan.at(t)
                waiting = avoidance.waiting and path.next_gate is not None
                state = GuidanceStatus.BLOCKED if waiting else GuidanceStatus.AVOIDING
        if not self.update_tracking_guard(pos[:2], measured[:2], path.progress):
            vel, acc = np.zeros(3), np.zeros(3)
        self.publish_setpoint(pos, vel, acc, MASK_PATH)
        self.state = state

    def select_path(self, measured):
        """The path of the active mode. A new intent_id starts a new path (start_path); until one
        can be followed, None: the drone holds its position."""
        mode = self.nav_mode
        msg = self.intents.get(mode)
        if msg is None:
            return None
        current = self.paths.get(mode)
        if current is not None and current.intent_id == msg.intent_id:
            return current
        if (mode, msg.intent_id) in self.rejected:
            return None
        if self.home is None:   # NavPoint z is a height above home
            if not self.warned_intent_no_home:
                self.warned_intent_no_home = True
                self.get_logger().warn('NavIntent reçue avant le home de mavros : le drone tient sa position tant que le home manque.')
            return None
        return self.start_path(mode, msg, measured)

    def start_path(self, mode, msg, measured):
        """Builds the path of a new NavIntent and follows it from now on, or refuses it (None)."""
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
        self.stop_avoidance(mode)   # logs the end of an avoidance on the path being replaced
        self.paths[mode] = path
        self.tracking_lost = False
        self.avoidances.pop(mode, None)
        if self.boundary_gps:
            boundary = [to_local(lat, lon, self.home) for lat, lon in self.boundary_gps]
            self.avoidances[mode] = Avoidance(path, boundary, logger=self.get_logger(), **self.avoidance_settings)
        self.get_logger().info(
            f'{MODE_NAMES[mode]} n°{msg.intent_id} suivie : {path.length:.0f} m, {len(path.gates)} portes, '
            f'drone à {distance:.1f} m du premier point.')
        return path

    def reject(self, mode, intent_id, reason):
        """Refuses a new intent_id for good. The previous path of the mode is dropped: the mode
        node replaced it, so it will not be flown again, and the status no longer shows it."""
        self.rejected.add((mode, intent_id))
        self.get_logger().error(f'{MODE_NAMES[mode]} n°{intent_id} refusée : {reason}. Le drone tient sa position.')
        self.stop_avoidance(mode)
        self.paths.pop(mode, None)
        self.avoidances.pop(mode, None)

    def stop_avoidance(self, mode):
        """Back to the path in that mode, if it was avoiding."""
        avoidance = self.avoidances.get(mode)
        if avoidance is not None:
            avoidance.stop(self.paths[mode].progress)

    def update_tracking_guard(self, setpoint_xy, measured_xy, progress):
        """False while the drone is lost from its setpoint: then the position alone is sent.

        Beyond max_tracking_error_m, a velocity feedforward that the position no longer justifies
        sends the drone away in a straight line (ArduPilot bounds the position correction, not the
        feedforward). It comes back once the drone is within TRACKING_RESTORE_M and the progression
        has moved RESTORE_PROGRESS_M since the loss."""
        error = float(np.linalg.norm(setpoint_xy - measured_xy))
        if not self.tracking_lost and error > self.max_tracking_error:
            self.tracking_lost = True
            self.progress_at_loss = progress
            self.get_logger().error(f'Drone à {error:.0f} m de sa consigne : feedforward coupé, position seule.')
        elif self.tracking_lost and error < TRACKING_RESTORE_M and progress > self.progress_at_loss + RESTORE_PROGRESS_M:
            self.tracking_lost = False
            self.get_logger().info('Drone revenu sur sa consigne : feedforward rétabli.')
        return not self.tracking_lost

    # ------------------------------------------------------------------ pilot mode
    def fly_pilot(self, now, measured):
        v = self.pilot
        fresh = v is not None and (now - self.pilot_received).nanoseconds * 1e-9 <= self.pilot_timeout
        if not fresh:
            if not self.pilot_lost:
                self.pilot_lost = True
                self.get_logger().warn('Mode PILOT sans VelocityIntent récente (pilot_nav) : le drone tient sa position.')
            self.hold(measured)
            self.state = GuidanceStatus.HOLDING
            return
        if self.pilot_lost:
            self.pilot_lost = False
            self.get_logger().info('VelocityIntent de nouveau reçues : vitesse du pilote suivie.')

        if v.x == 0.0 and v.y == 0.0 and v.z == 0.0:
            self.hold(measured, v.yaw_rate)
            self.state = GuidanceStatus.HOLDING
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
            self.state = GuidanceStatus.HOLDING
            return
        self.hold_point = None
        self.publish_setpoint(np.zeros(3), velocity, np.zeros(3), MASK_VELOCITY, v.yaw_rate)
        self.state = GuidanceStatus.FOLLOWING

    # ------------------------------------------------------------------ outputs
    def hold(self, measured, yaw_rate=0.0):
        """Brakes (zero velocity) down to HOLD_CAPTURE_SPEED, then holds the point where it stopped."""
        if self.hold_point is None:
            if np.linalg.norm(self.velocity) > HOLD_CAPTURE_SPEED:
                self.publish_setpoint(np.zeros(3), np.zeros(3), np.zeros(3), MASK_VELOCITY, yaw_rate)
                return
            self.hold_point = measured.copy()
        self.publish_setpoint(self.hold_point, np.zeros(3), np.zeros(3), MASK_HOLD, yaw_rate)

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
        avoidance = self.avoidances.get(self.nav_mode)
        msg.threats = 0 if avoidance is None else min(avoidance.threat_count, 255)
        msg.min_margin_m = math.nan if avoidance is None else float(avoidance.min_margin)
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


def yaw_of(q):
    """Yaw (rad, ENU, counterclockwise from east) of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
