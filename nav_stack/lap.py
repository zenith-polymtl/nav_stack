#!/usr/bin/env python3
"""Lap node: flies the Task 1 laps by streaming GUIDED setpoints (position + velocity + acceleration),
and avoids the competition traffic with a local planner.

Reference: precomputed offline (context_research/research/laps_bench/export_reference.py), a CSV
file: comment lines `# key: value` (among them origin_lat/origin_lon, the home the plan was computed
for), then the header `s,north,east,vn,ve,an,ae,alt,vz`: curvilinear abscissa, position north/east
in meters from the planned home, velocity and acceleration north/east, height above home, vertical
speed (up). It starts at rest at home and ends at rest inside the search area. It is anchored in the
mavros local frame (ENU, from the EKF origin) like convert does: the home position in that frame,
plus the flat-earth offset from the actual home to the planned one. The setpoint goes out in
FRAME_LOCAL_NED: mavros converts it from ENU.

Tracking: the reference is indexed by progression, not by time. On each local pose (stream_rate_hz
of init), the progression moves to the projection of the drone on the reference, inside a window of
1.5 x v x dt + 2 m: it never goes back and never jumps over a hairpin apex, even with the drone off
the line. The setpoint is the reference at that progression, without lookahead, so the planned
speed is the flown speed. With latency_compensation, the projected position is the measured one
extrapolated by the measured velocity over the pose age plus uplink_delay_s: where the drone is
when the autopilot receives the setpoint.

Modes: the node never arms and never changes mode. It starts once topics.TAKEOFF_COMPLETED is True
(init) and the vehicle is armed in GUIDED. If the pilot leaves GUIDED or disarms, it stops sending;
back in GUIDED, it resumes at the current progression, which catches up with the drone through the
same window, so no waypoint is skipped.

Tracking guard: beyond max_tracking_error_m from the setpoint, the node sends the position alone.
Otherwise a velocity feedforward that the position no longer justifies (a frozen progression) sends
the drone away in a straight line: ArduPilot bounds the position correction, not the feedforward.
The feedforward comes back once the drone is within TRACKING_RESTORE_M of the setpoint and the
progression has moved since the loss; without this hysteresis the guard oscillates.
test_freeze_progress_m (bench only, 0 = off, settable at run time) freezes the progression at that
abscissa to trigger the guard in SITL (laps_bench/ros_test).

End: at the end of the reference, the node holds the final point at zero velocity for HOLD_TICKS
poses (a position target does not time out), publishes topics.LAPS_COMPLETED (latched), then stays
silent so that the next node can take over in GUIDED.

Traffic avoidance: from topics.TRAFFIC (comp_client; sim_mocks traffic_mock in simulation). Every
replan_s, if an aircraft threatens (its predicted path within threat_range_m of the drone or of the
line ahead, in vertical overlap with the drone's predicted altitude), local_planner plans the next
10 s: follow the line, stay inside the flight boundary, pass the next waypoint, stay out of the
predicted keep-away circles by margin_hard_m (almost hard) or margin_soft_m (preferred). The node
flies that plan horizontally (altitude still from the reference) until no aircraft threatens; when a
replan fails, it keeps the previous plan while it covers the present, else the reference. Without
boundary.* and task1.course.* in the site file, avoidance is off. `ros2 param set /lap avoidance
false` switches it off from the ground. Why each choice: laps_navigation_sitl_research.md §5, §6.

Parameters (config/<mission>.yaml): reference_file (required, absolute path), latency_compensation,
uplink_delay_s, max_start_distance_m, max_tracking_error_m, avoidance, replan_s, threat_range_m,
margin_hard_m, margin_soft_m, test_freeze_progress_m. From the site file (config/sites/<site>.yaml):
boundary.lat/lon, task1.course.lat/lon, task1.course.laps.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import HomePosition, PositionTarget, State
from std_msgs.msg import Bool

from custom_interfaces.msg import TrafficArray
from tools import topics

from nav_stack import local_planner, traffic

# mavros, package tiers : ses noms restent ici (ARCHITECTURE.md, règles de code)
MAVROS_POSE = '/mavros/local_position/pose'
MAVROS_VELOCITY = '/mavros/local_position/velocity_local'
MAVROS_STATE = '/mavros/state'
MAVROS_HOME = '/mavros/home_position/home'
MAVROS_SETPOINT = '/mavros/setpoint_raw/local'

GUIDED = 'GUIDED'
LOCAL_FRAME = 'map'   # mavros local frame
COLUMNS = ('s', 'north', 'east', 'vn', 've', 'an', 'ae', 'alt', 'vz')
# The reference in the local frame (ENU): its columns, from the CSV ones
LOCAL_COLUMNS = [COLUMNS.index(c) for c in ('east', 'north', 'alt', 've', 'vn', 'vz', 'ae', 'an')]

# Position, velocity and acceleration; yaw left to the autopilot (faces the direction of travel)
TYPE_MASK = PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE

# Progression window per tick: ADVANCE_FACTOR x v x dt + ADVANCE_MARGIN_M
ADVANCE_FACTOR = 1.5
ADVANCE_MARGIN_M = 2.0
MIN_SPEED_FOR_WINDOW = 1.0   # m/s, so that the window opens at the start, at rest
NOMINAL_DT = 0.05            # s, first tick and after a pause
MAX_DT = 0.25                # s, a pose gap longer than this does not open the window wider
MAX_EXTRAPOLATION_S = 0.3    # latency compensation never extrapolates further
END_DISTANCE_M = 1.0         # progression this close to the end: laps done
HOLD_TICKS = 10              # final setpoint repeated (0.5 s at 20 Hz) against a lost message
HOME_MISMATCH_WARN_M = 2.0
TRACKING_RESTORE_M = 8.0     # tracking guard: feedforward back below this error...
RESTORE_PROGRESS_M = 1.0     # ...once the progression has moved this much since the loss
GATE_REACHED_M = 10.0        # a waypoint counts as passed within this distance (the rules: 10 m)
GATE_GIVE_UP_M = 30.0        # progression this far past a waypoint apex: it is left behind (logged)
GATE_SEARCH_M = 60.0         # gates: the reference passes a waypoint within this...
GATE_PASS_LENGTH_M = 200.0   # ...and its nearest point within this much reference after the approach
MAX_OBSTACLES = 4            # nearest threatening aircraft given to the planner
VERTICAL_MARGIN_M = 3.0      # an aircraft counts if |altitude difference| <= its vka + this

# ArduPilot LATLON_TO_M (AP_Math/definitions.h), per degree: the same flat-earth formula as convert
METERS_PER_DEGREE = 0.011131884502145034 * 1e7

# Latched: a node started late (the mission) still gets the last value
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
STATE_QOS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

RUNTIME_PARAMETERS = ('avoidance', 'test_freeze_progress_m')


def to_local(lat, lon, home):
    """lat/lon -> mavros local frame (x east, y north), from the mavros home (as convert does)."""
    north = (lat - home.geo.latitude) * METERS_PER_DEGREE
    east = (lon - home.geo.longitude) * METERS_PER_DEGREE * math.cos(math.radians((lat + home.geo.latitude) / 2))
    return np.array([home.position.x + east, home.position.y + north])


def load_reference(path):
    """Reads the CSV: returns (metadata dict, array N x 9 in COLUMNS order)."""
    meta, rows, header = {}, [], None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                key, _, value = line[1:].partition(':')
                meta[key.strip()] = value.strip()
            elif header is None:
                header = [c.strip() for c in line.split(',')]
            else:
                rows.append([float(v) for v in line.split(',')])
    if header is None or tuple(header) != COLUMNS:
        raise ValueError(f'en-tête attendu {",".join(COLUMNS)}, trouvé {header}')
    data = np.array(rows, dtype=float)
    if len(data) < 2 or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError('il faut au moins deux points et une abscisse s strictement croissante')
    return meta, data


def waypoint_gates(s, xy, waypoints):
    """(progression of the apex, waypoint) for each waypoint in order: the nearest point of the
    reference on its first pass near the waypoint after the previous one (a lap later, the
    reference passes there again)."""
    gates, k0 = [], 0
    for wp in waypoints:
        d = np.hypot(*(xy[k0:] - wp).T)
        near = np.flatnonzero(d < GATE_SEARCH_M)
        stop = len(d)
        if len(near):
            stop = int(np.searchsorted(s[k0:], s[k0 + near[0]] + GATE_PASS_LENGTH_M))
        k = k0 + int(np.argmin(d[:max(stop, 1)]))
        gates.append((float(s[k]), wp))
        k0 = k + 1
    return gates


class Lap(Node):
    def __init__(self):
        super().__init__('lap')

        self.declare_parameter('reference_file', Parameter.Type.STRING)
        self.declare_parameter('latency_compensation', True)
        self.declare_parameter('uplink_delay_s', 0.02)
        self.declare_parameter('max_start_distance_m', 15.0)
        self.declare_parameter('max_tracking_error_m', 25.0)
        self.declare_parameter('avoidance', True)
        self.declare_parameter('replan_s', 0.25)
        self.declare_parameter('threat_range_m', 120.0)
        self.declare_parameter('margin_hard_m', 4.0)
        self.declare_parameter('margin_soft_m', 15.0)
        self.declare_parameter('test_freeze_progress_m', 0.0)
        for name in ('boundary.lat', 'boundary.lon', 'task1.course.lat', 'task1.course.lon'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)   # from the site file
        self.declare_parameter('task1.course.laps', Parameter.Type.INTEGER)
        reference_file = self.get_parameter('reference_file').value
        self.latency_compensation = self.get_parameter('latency_compensation').value
        self.uplink_delay = self.get_parameter('uplink_delay_s').value
        self.max_start_distance = self.get_parameter('max_start_distance_m').value
        self.max_tracking_error = self.get_parameter('max_tracking_error_m').value
        self.avoidance = self.get_parameter('avoidance').value
        self.replan_s = self.get_parameter('replan_s').value
        self.threat_range = self.get_parameter('threat_range_m').value
        self.margin_hard = self.get_parameter('margin_hard_m').value
        self.margin_soft = self.get_parameter('margin_soft_m').value
        self.freeze_at = 0.0
        self.set_freeze(self.get_parameter('test_freeze_progress_m').value)
        self.site = self.read_site()
        self.add_on_set_parameters_callback(self.callback_parameters)

        try:
            self.meta, self.ref = load_reference(reference_file)
        except (OSError, ValueError) as e:
            self.get_logger().error(f'Référence {reference_file} illisible : {e}')
            raise
        self.s = self.ref[:, 0]
        self.local = None   # the reference in the mavros local frame, columns LOCAL_COLUMNS, set at the start

        # Autopilot and mission state
        self.home = None
        self.takeoff_done = False
        self.mode = ''
        self.armed = False
        self.velocity = np.zeros(3)   # measured, ENU

        # Tracking state
        self.started = False
        self.flying = False
        self.finished = False
        self.s_prog = 0.0
        self.last_stamp = None
        self.hold_ticks_left = HOLD_TICKS
        self.tracking_lost = False
        self.s_at_loss = 0.0

        # Traffic avoidance (set up at the start, once the reference is anchored)
        self.tracks = {}                # aircraft index -> traffic.Track
        self.planner = None             # local_planner.LocalPlanner
        self.gates = []                 # (progression of the apex, waypoint xy), in course order
        self.gate_next = 0              # index of the next waypoint to pass
        self.plan = None                # local_planner.Plan being flown, or None
        self.last_replan = -math.inf    # s, node clock
        self.avoiding = False
        self.fail_streak = 0            # failed replans in a row (logs)
        self.warned_traffic_no_home = False

        self.setpoint_pub = self.create_publisher(PositionTarget, MAVROS_SETPOINT, qos_profile_sensor_data)
        self.completed_pub = self.create_publisher(Bool, topics.LAPS_COMPLETED, LATCHED)

        self.create_subscription(HomePosition, MAVROS_HOME, self.callback_home, LATCHED)
        self.create_subscription(State, MAVROS_STATE, self.callback_state, STATE_QOS)
        self.create_subscription(Bool, topics.TAKEOFF_COMPLETED, self.callback_takeoff, LATCHED)
        self.create_subscription(TwistStamped, MAVROS_VELOCITY, self.callback_velocity, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, MAVROS_POSE, self.callback_pose, qos_profile_sensor_data)
        self.create_subscription(TrafficArray, topics.TRAFFIC, self.callback_traffic, 10)

        self.get_logger().info(
            f"Node lap prête : référence {reference_file} ({self.meta.get('kind', '?')}, "
            f'{self.s[-1]:.0f} m, plan {self.meta.get("planned_time_s", "?")} s), '
            'en attente du décollage, de GUIDED et de l\'armement.')

    # ------------------------------------------------------------------ inputs
    def callback_home(self, msg):
        position = (msg.position.x, msg.position.y, msg.position.z)
        if any(math.isnan(v) for v in position):
            self.get_logger().warn("Home reçu sans position locale (pas encore d'origine EKF) : ignoré.")
            return
        if self.started:
            self.get_logger().warn('Home modifié en vol : la référence reste ancrée sur le home du départ.')
        self.home = msg

    def callback_state(self, msg):
        self.mode = msg.mode
        self.armed = msg.armed

    def callback_takeoff(self, msg):
        self.takeoff_done = msg.data
        if not msg.data:
            self.get_logger().warn('Décollage refusé (init) : les tours attendront un décollage réussi.')

    def callback_velocity(self, msg):
        v = msg.twist.linear
        self.velocity = np.array([v.x, v.y, v.z])

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
                    f'Trafic : {a.name} (#{a.index}), keep-away {a.horizontal_keep_away:.0f} m, '
                    f'{a.altitude_agl:.0f} m AGL.')
            track.add(traffic.Update(stamp, to_local(a.latitude, a.longitude, self.home), a.altitude_agl,
                                     a.speed, a.course_deg, a.horizontal_keep_away, a.vertical_keep_away))

    def callback_parameters(self, params):
        """At run time, only avoidance (from the ground) and test_freeze_progress_m (bench) change."""
        if any(p.name not in RUNTIME_PARAMETERS for p in params):
            return SetParametersResult(successful=False,
                                       reason='seuls avoidance et test_freeze_progress_m se changent en vol')
        for p in params:
            if p.name == 'avoidance':
                self.avoidance = bool(p.value)
                if not self.avoidance and self.planner is not None:
                    self.stop_avoiding()
                self.get_logger().warn(f'Évitement du trafic {"activé" if self.avoidance else "coupé"} (paramètre avoidance).')
            else:
                self.set_freeze(p.value)
        return SetParametersResult(successful=True)

    def set_freeze(self, value):
        was_frozen = self.freeze_at > 0.0
        self.freeze_at = float(value)
        if self.freeze_at > 0.0:
            self.get_logger().warn(
                f'TEST : progression figée à {self.freeze_at:.0f} m (test_freeze_progress_m). Jamais en vol.')
        elif was_frozen:
            self.get_logger().warn('TEST : progression libérée (test_freeze_progress_m = 0).')

    def read_site(self):
        """Flight boundary and course from the site file, or None (avoidance then off)."""
        try:
            get = lambda name: self.get_parameter(name).value   # noqa: E731
            return dict(boundary=list(zip(get('boundary.lat'), get('boundary.lon'))),
                        course=list(zip(get('task1.course.lat'), get('task1.course.lon'))),
                        laps=get('task1.course.laps'))
        except ParameterUninitializedException:
            if self.avoidance:
                self.get_logger().warn(
                    'Pas de limite de vol ni de parcours dans le fichier du site (boundary.*, task1.course.*) : '
                    'évitement du trafic désactivé.')
            return None

    # ------------------------------------------------------------------ start
    def ready_to_fly(self):
        return self.takeoff_done and self.home is not None and self.mode == GUIDED and self.armed

    def start(self, position):
        local = self.anchor_reference()
        distance = float(np.hypot(*(local[0, :2] - position[:2])))
        if distance > self.max_start_distance:
            self.get_logger().error(
                f'Drone à {distance:.1f} m du début de la référence (max {self.max_start_distance} m) : '
                'tours annulés. Vérifier le home et la référence.')
            self.finished = True
            return False
        self.local = local
        self.started = True
        self.setup_avoidance()
        self.get_logger().info(f'Départ des tours : drone à {distance:.1f} m du début de la référence.')
        return True

    def anchor_reference(self):
        """The reference in the mavros local frame: the planned home at its GPS position, north/east
        as y/x."""
        home = self.home
        origin = np.array([home.position.x, home.position.y])
        if 'origin_lat' in self.meta and 'origin_lon' in self.meta:
            planned = to_local(float(self.meta['origin_lat']), float(self.meta['origin_lon']), home)
            gap = float(np.hypot(*(planned - origin)))
            if gap > HOME_MISMATCH_WARN_M:
                self.get_logger().warn(
                    f'Le home est à {gap:.1f} m de celui du plan : la référence reste sur ses points GPS, '
                    'le début du tracé est décalé du drone d\'autant.')
            origin = planned
        local = self.ref[:, LOCAL_COLUMNS]
        local[:, 0] += origin[0]
        local[:, 1] += origin[1]
        local[:, 2] += home.position.z
        return local

    def setup_avoidance(self):
        """Planner, boundary and waypoints in the local frame, once the reference is anchored."""
        if self.site is None:
            return
        boundary = [to_local(lat, lon, self.home) for lat, lon in self.site['boundary']]
        reference = local_planner.Reference(self.s, self.local[:, 0:2], self.local[:, 3:5])
        self.planner = local_planner.LocalPlanner(reference, boundary, self.margin_hard, self.margin_soft)
        # the waypoints to pass, in order: each lap, then back to the first one before the zone
        course = [to_local(lat, lon, self.home) for lat, lon in self.site['course']]
        self.gates = waypoint_gates(self.s, self.local[:, 0:2], course * self.site['laps'] + course[:1])
        self.get_logger().info(
            f'Évitement du trafic prêt : {len(boundary)} sommets de limite, {len(self.gates)} passages de WP.')

    # ------------------------------------------------------------------ the loop, on each pose
    def callback_pose(self, msg):
        if self.finished:
            return
        if not self.ready_to_fly():
            if self.flying:
                self.flying = False
                reason = 'désarmé' if not self.armed else f'mode {self.mode}'
                self.get_logger().warn(
                    f'Pilote : {reason}. Flux suspendu à {self.s_prog:.0f} m sur {self.s[-1]:.0f} m.')
            return

        p = msg.pose.position
        measured = np.array([p.x, p.y, p.z])
        if not self.started and not self.start(measured):
            return
        if not self.flying:
            self.flying = True
            self.last_stamp = None
            if self.s_prog > 0.0:
                self.get_logger().info(f'GUIDED : reprise des tours à {self.s_prog:.0f} m.')

        stamp = Time.from_msg(msg.header.stamp)
        dt = NOMINAL_DT if self.last_stamp is None else \
            min(max((stamp - self.last_stamp).nanoseconds * 1e-9, 0.0), MAX_DT)
        self.last_stamp = stamp
        position = measured
        if self.latency_compensation:
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            position = measured + self.velocity * min(max(age, 0.0) + self.uplink_delay, MAX_EXTRAPOLATION_S)

        self.advance(position[:2], dt)
        if self.s[-1] - self.s_prog < END_DISTANCE_M:
            self.finish()
            return
        pos, vel, acc = self.sample(self.s_prog)
        self.update_gates(position[:2])
        local = self.avoid(position, p.z - self.home.position.z)
        if local is not None:     # the local plan: horizontal from it, vertical from the line
            pos[:2], vel[:2], acc[:2] = local
        if not self.feedforward_allowed(pos[:2], measured[:2]):
            vel, acc = np.zeros(3), np.zeros(3)
        self.publish_setpoint(pos, vel, acc)

    def advance(self, xy, dt):
        """Moves the progression forward to the projection of xy, inside the per-tick window."""
        i0 = int(np.searchsorted(self.s, self.s_prog, side='right')) - 1
        speed = max(float(np.hypot(*self.local[i0, 3:5])),
                    float(np.hypot(*self.velocity[:2])), MIN_SPEED_FOR_WINDOW)
        s_max = self.s_prog + ADVANCE_FACTOR * speed * dt + ADVANCE_MARGIN_M
        i1 = min(int(np.searchsorted(self.s, s_max, side='right')), len(self.s) - 1)
        k = i0 + int(np.argmin(np.hypot(*(self.local[i0:i1 + 1, :2] - xy).T)))
        # projection on the segments around the nearest point, for a continuous progression
        best_d, best_s = math.inf, self.s[k]
        for a in (k - 1, k):
            if a < i0 or a + 1 > i1:
                continue
            seg = self.local[a + 1, :2] - self.local[a, :2]
            length2 = float(seg @ seg)
            u = 0.0 if length2 == 0.0 else min(max(float((xy - self.local[a, :2]) @ seg) / length2, 0.0), 1.0)
            d = float(np.hypot(*(self.local[a, :2] + u * seg - xy)))
            if d < best_d:
                best_d, best_s = d, self.s[a] + u * (self.s[a + 1] - self.s[a])
        self.s_prog = min(max(self.s_prog, best_s), s_max, self.s[-1])
        if self.freeze_at > 0.0:
            self.s_prog = min(self.s_prog, self.freeze_at)   # bench only, see the module docstring

    def sample(self, s):
        """Reference at abscissa s, linear interpolation: (pos xyz, vel xyz, acc xyz), ENU."""
        i = min(int(np.searchsorted(self.s, s, side='right')) - 1, len(self.s) - 2)
        f = (s - self.s[i]) / (self.s[i + 1] - self.s[i])
        row = (1.0 - f) * self.local[i] + f * self.local[i + 1]
        return row[0:3], row[3:6], np.array([row[6], row[7], 0.0])

    def feedforward_allowed(self, setpoint_xy, measured_xy):
        """Tracking guard, with its hysteresis: False while the drone is lost from its setpoint."""
        error = float(np.linalg.norm(setpoint_xy - measured_xy))
        if not self.tracking_lost and error > self.max_tracking_error:
            self.tracking_lost = True
            self.s_at_loss = self.s_prog
            self.get_logger().error(
                f'Drone à {error:.0f} m de sa consigne : feedforward coupé, retour en position seule.')
        elif (self.tracking_lost and error < TRACKING_RESTORE_M
              and self.s_prog > self.s_at_loss + RESTORE_PROGRESS_M):
            self.tracking_lost = False
            self.get_logger().info('Drone revenu sur sa consigne : feedforward rétabli.')
        return not self.tracking_lost

    # ------------------------------------------------------------------ traffic avoidance
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def avoid(self, position, altitude):
        """The local plan's (pos, vel, acc) at this instant when an aircraft threatens, else None."""
        if not self.avoidance or self.planner is None or not self.tracks:
            return None
        now = self.now_s()
        if now - self.last_replan >= self.replan_s:
            self.last_replan = now
            obstacles = self.threats(now, position[:2], altitude)
            if not obstacles:
                self.stop_avoiding()
                return None
            if not self.avoiding:
                self.avoiding = True
                self.get_logger().info(f'Évitement : {len(obstacles)} avion(s) menace(nt), plans locaux.')
            self.replan(now, position[:2], obstacles)
        if self.plan is None or not self.plan.covers(now):
            return None
        return self.plan.at(now)

    def replan(self, now, xy, obstacles):
        gate = self.gates[self.gate_next] if self.gate_next < len(self.gates) else None
        plan = self.planner.plan(self.s_prog, xy, self.velocity[:2], obstacles, gate, now)
        if plan is not None:
            if self.fail_streak > 1:
                self.get_logger().info(f'Plan local rétabli après {self.fail_streak} échecs de suite.')
            self.fail_streak = 0
            self.plan = plan
            return
        # une rafale d'échecs : un seul WARN au début (le plan précédent reste valide 9,5 s)
        self.fail_streak += 1
        if self.fail_streak == 1:
            self.get_logger().warn('Plan local raté : le précédent reste en vigueur s\'il est encore valide.')

    def threats(self, now, xy, altitude):
        """Aircraft whose predicted path comes near the drone or the line ahead, in vertical overlap
        with the drone's predicted altitude (it may be climbing): the MAX_OBSTACLES nearest."""
        t = np.arange(local_planner.STEPS + 1) * local_planner.DT
        altitudes = np.clip(altitude + self.velocity[2] * t, 0.0, None)
        speed = max(float(np.hypot(*self.velocity[:2])), 1.0)
        ahead = self.planner.ref.at(self.s_prog + t * speed)[0]
        found = []
        for track in self.tracks.values():
            if track.age(now) > traffic.STALE_S:
                continue
            last = track.last
            if not np.any(np.abs(altitudes - last.altitude) <= last.vka + VERTICAL_MARGIN_M):
                continue
            centers = track.predict(now, local_planner.HORIZON_S, local_planner.DT)
            near = min(float(np.min(np.hypot(*(centers - ahead).T))), float(np.hypot(*(centers[0] - xy))))
            if near < self.threat_range:
                found.append((near, local_planner.Obstacle(centers, last.hka)))
        found.sort(key=lambda f: f[0])
        return [ob for _, ob in found[:MAX_OBSTACLES]]

    def update_gates(self, xy):
        """Moves to the next waypoint once this one is passed (within 10 m), or left behind."""
        while self.gate_next < len(self.gates):
            s_apex, wp = self.gates[self.gate_next]
            if np.hypot(*(xy - wp)) <= GATE_REACHED_M:
                self.gate_next += 1
            elif self.s_prog > s_apex + GATE_GIVE_UP_M:
                self.get_logger().error(
                    f'WP n°{self.gate_next + 1} laissé derrière sans y être passé à moins de {GATE_REACHED_M:.0f} m.')
                self.gate_next += 1
            else:
                break

    def stop_avoiding(self):
        self.plan = None
        if self.avoiding:
            self.avoiding = False
            times = np.array(self.planner.solve_times) * 1000
            self.get_logger().info(
                f'Fin d\'évitement, retour sur la ligne à {self.s_prog:.0f} m. Plans : {len(times)}, '
                f'{np.median(times):.0f} ms en médiane, {times.max():.0f} ms au pire, {self.planner.failures} ratés.')

    # ------------------------------------------------------------------ output
    def publish_setpoint(self, pos, vel, acc):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = LOCAL_FRAME
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED   # mavros converts ENU -> NED
        sp.type_mask = TYPE_MASK
        sp.position.x, sp.position.y, sp.position.z = (float(v) for v in pos)
        sp.velocity.x, sp.velocity.y, sp.velocity.z = (float(v) for v in vel)
        sp.acceleration_or_force.x, sp.acceleration_or_force.y, sp.acceleration_or_force.z = \
            (float(v) for v in acc)
        self.setpoint_pub.publish(sp)

    def finish(self):
        """Holds the final point (inside the search area, at rest) for HOLD_TICKS poses, then reports
        and goes silent."""
        self.publish_setpoint(self.local[-1, 0:3], np.zeros(3), np.zeros(3))
        self.hold_ticks_left -= 1
        if self.hold_ticks_left > 0:
            return
        self.finished = True
        self.completed_pub.publish(Bool(data=True))
        self.get_logger().info('Tours terminés : position finale tenue dans la zone de recherche, flux arrêté.')


def main(args=None):
    rclpy.init(args=args)
    node = Lap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
