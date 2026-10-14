#!/usr/bin/env python3
"""Lap node: flies the Task 1 laps by streaming GUIDED setpoints (position + velocity + acceleration).

The trajectory is precomputed offline (context_research/research/laps_bench/export_reference.py)
and read from a CSV file: comment lines `# key: value` (metadata, among them origin_lat/origin_lon,
the home the plan was computed for), then the header `s,north,east,vn,ve,an,ae,alt,vz`: curvilinear
abscissa, position north/east in meters from the planned home, velocity and acceleration north/east,
height above home, vertical speed (positive up). The reference starts at rest at home and ends at
rest inside the search area.

Frames: /mavros/local_position/pose and /mavros/setpoint_raw/local are ENU (x east, y north, z up)
from the EKF origin. The reference is anchored like convert does: the home position in that local
frame (/mavros/home_position/home), plus the flat-earth offset between the actual home and the
planned one (zero when the plan was computed for this home), plus north/east swapped to x/y. The
setpoint uses FRAME_LOCAL_NED: mavros converts position, velocity and acceleration ENU -> NED.

Tracking: the reference is indexed by progression, not by time. On each local pose (stream_rate_hz
of init) the node projects the drone on the reference, inside a window that starts at the current
progression and extends by at most 1.5 x v x dt + 2 m: the progression never goes back and never
jumps over a hairpin apex, even with the drone off the line. No lookahead: the setpoint is the
reference at the drone's own progression, so the planned speed is the flown speed (a 0.1 s
lookahead flew 10 % faster than the plan in SITL). With latency_compensation, the measured position
is extrapolated by the measured velocity over the pose age plus uplink_delay_s, so that the setpoint
matches where the drone is when the autopilot receives it.

Start and modes: the node never arms and never changes mode (the pilot does). It starts once
topics.TAKEOFF_COMPLETED is True (init) and /mavros/state reports GUIDED and armed. If the pilot
leaves GUIDED (or the vehicle disarms), the node stops sending and logs it. When the pilot puts
GUIDED back, it resumes at the current progression: the progression catches up with the drone
along the reference, within the same per-tick window, so it cannot skip a waypoint.

Tracking guard: if the drone is more than max_tracking_error_m from the setpoint, the node sends the
position alone (no velocity, no acceleration) until it is back. Found in SITL: when the progression
stops while a velocity feedforward keeps going out, ArduPilot follows the feedforward (it bounds the
position correction) and the drone flies away in a straight line at full speed.

Test only: the parameter test_freeze_progress_m (0 = off, the default; settable at run time with
`ros2 param set /lap test_freeze_progress_m <m>`) stops the progression at that abscissa, to trigger
the tracking guard in SITL (context_research/research/laps_bench/ros_test). Never set in flight.

End: when the progression reaches the end of the reference (inside the search area, at rest), the
node sends the final point with zero velocity for HOLD_TICKS poses (the autopilot keeps holding it:
a position target does not time out), publishes topics.LAPS_COMPLETED (True, latched), then stays
silent so that the next node can take over in GUIDED.

Deconfliction hook (the logic lives in other nodes), three inputs, neutral by default:
- topics.DECONFLICTION_CORRECTION (Vector3, ENU m/s, z ignored): the predictive repulsion of the
  deconfliction node (nav_stack/deconfliction.py, pf_pred of laps_bench/deconfliction.py), which
  publishes it on every pose, zero when nothing threatens. Added to the velocity feedforward, and the
  position is shifted by correction x correction_lead_s so that position and velocity agree. Its
  change is limited to correction_accel_mps2, and it decays to zero when no message arrived for
  correction_timeout_s (deconfliction node down): a frozen correction must never keep pushing the drone.
- topics.LAP_SPEED_FACTOR (Float64, 0 to 1) scales the speed along the line; time scaling gives
  velocity k v and acceleration k^2 a + k' v. k changes no faster than speed_factor_accel_mps2
  allows at the current reference speed (|k'| v <= that acceleration).
- topics.LAP_OFFSET (Vector3: x to the right of the direction of travel, z up, in meters, y
  ignored) shifts the setpoint, rate limited by offset_rate_mps. The last value is kept.

Parameters (config/<mission>.yaml): reference_file (required, absolute path), latency_compensation,
uplink_delay_s, max_start_distance_m, speed_factor_accel_mps2, offset_rate_mps, correction_lead_s,
correction_accel_mps2, correction_timeout_s, max_tracking_error_m; test_freeze_progress_m (test only).
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from mavros_msgs.msg import HomePosition, PositionTarget, State
from std_msgs.msg import Bool, Float64

from tools import topics

# mavros, package tiers : ses noms restent ici (ARCHITECTURE.md, règles de code)
MAVROS_POSE = '/mavros/local_position/pose'
MAVROS_VELOCITY = '/mavros/local_position/velocity_local'
MAVROS_STATE = '/mavros/state'
MAVROS_HOME = '/mavros/home_position/home'
MAVROS_SETPOINT = '/mavros/setpoint_raw/local'

GUIDED = 'GUIDED'
LOCAL_FRAME = 'map'   # mavros local frame
COLUMNS = ('s', 'north', 'east', 'vn', 've', 'an', 'ae', 'alt', 'vz')

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

# ArduPilot LATLON_TO_M (AP_Math/definitions.h), per degree, the same formula as convert
METERS_PER_DEGREE = 0.011131884502145034 * 1e7

# Latched: a node started late (the mission) still gets the last value
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
STATE_QOS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


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


class Lap(Node):
    def __init__(self):
        super().__init__('lap')

        self.declare_parameter('reference_file', Parameter.Type.STRING)
        self.declare_parameter('latency_compensation', True)
        self.declare_parameter('uplink_delay_s', 0.02)
        self.declare_parameter('max_start_distance_m', 15.0)
        self.declare_parameter('speed_factor_accel_mps2', 4.0)
        self.declare_parameter('offset_rate_mps', 2.0)
        self.declare_parameter('correction_lead_s', 1.0)
        self.declare_parameter('correction_accel_mps2', 6.0)
        self.declare_parameter('correction_timeout_s', 1.5)
        self.declare_parameter('max_tracking_error_m', 25.0)
        self.declare_parameter('test_freeze_progress_m', 0.0)
        reference_file = self.get_parameter('reference_file').value
        self.latency_compensation = self.get_parameter('latency_compensation').value
        self.uplink_delay = self.get_parameter('uplink_delay_s').value
        self.max_start_distance = self.get_parameter('max_start_distance_m').value
        self.speed_factor_accel = self.get_parameter('speed_factor_accel_mps2').value
        self.offset_rate = self.get_parameter('offset_rate_mps').value
        self.correction_lead = self.get_parameter('correction_lead_s').value
        self.correction_accel = self.get_parameter('correction_accel_mps2').value
        self.correction_timeout = self.get_parameter('correction_timeout_s').value
        self.max_tracking_error = self.get_parameter('max_tracking_error_m').value
        self.tracking_lost = False
        self.freeze_at = 0.0
        self.set_freeze(self.get_parameter('test_freeze_progress_m').value)
        self.add_on_set_parameters_callback(self.callback_parameters)

        try:
            self.meta, self.ref = load_reference(reference_file)
        except (OSError, ValueError) as e:
            self.get_logger().error(f'Référence {reference_file} illisible : {e}')
            raise
        self.s = self.ref[:, 0]
        self.local = None   # reference in the mavros local frame (ENU), set at the start

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

        # Deconfliction: commands (topics) and the rate-limited values applied
        self.speed_factor_cmd = 1.0
        self.speed_factor = 1.0
        self.offset_cmd = np.zeros(2)   # right of travel, up
        self.offset = np.zeros(2)
        self.speed_factor_rate_now = 0.0   # dk/dt, for the acceleration feedforward
        self.offset_rate_now = np.zeros(2)
        self.correction_cmd = np.zeros(2)   # ENU m/s, as received
        self.correction_stamp = None        # node clock time of the last correction message
        self.correction = np.zeros(2)       # applied, rate limited

        self.setpoint_pub = self.create_publisher(PositionTarget, MAVROS_SETPOINT, qos_profile_sensor_data)
        self.completed_pub = self.create_publisher(Bool, topics.LAPS_COMPLETED, LATCHED)

        self.create_subscription(HomePosition, MAVROS_HOME, self.callback_home, LATCHED)
        self.create_subscription(State, MAVROS_STATE, self.callback_state, STATE_QOS)
        self.create_subscription(Bool, topics.TAKEOFF_COMPLETED, self.callback_takeoff, LATCHED)
        self.create_subscription(TwistStamped, MAVROS_VELOCITY, self.callback_velocity, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, MAVROS_POSE, self.callback_pose, qos_profile_sensor_data)
        self.create_subscription(Float64, topics.LAP_SPEED_FACTOR, self.callback_speed_factor, 10)
        self.create_subscription(Vector3, topics.LAP_OFFSET, self.callback_offset, 10)
        self.create_subscription(Vector3, topics.DECONFLICTION_CORRECTION, self.callback_correction, 10)

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

    def callback_speed_factor(self, msg):
        k = min(max(msg.data, 0.0), 1.0)
        if k != msg.data:
            self.get_logger().warn(f'Facteur de vitesse {msg.data} hors de [0, 1] : ramené à {k}.')
        self.speed_factor_cmd = k

    def callback_offset(self, msg):
        self.offset_cmd = np.array([msg.x, msg.z])

    def callback_correction(self, msg):
        self.correction_cmd = np.array([msg.x, msg.y])
        self.correction_stamp = self.get_clock().now()

    def callback_parameters(self, params):
        """Only test_freeze_progress_m changes at run time (test of the tracking guard)."""
        for p in params:
            if p.name != 'test_freeze_progress_m':
                return SetParametersResult(successful=False, reason='seul test_freeze_progress_m se change en vol')
        for p in params:
            self.set_freeze(p.value)
        return SetParametersResult(successful=True)

    def set_freeze(self, value):
        was_frozen = self.freeze_at > 0.0
        self.freeze_at = float(value or 0.0)
        if self.freeze_at > 0.0:
            self.get_logger().warn(
                f'TEST : progression figée à {self.freeze_at:.0f} m (test_freeze_progress_m). Jamais en vol.')
        elif was_frozen:
            self.get_logger().warn('TEST : progression libérée (test_freeze_progress_m = 0).')

    # ------------------------------------------------------------------ start, pause, resume
    def anchor_reference(self):
        """Reference in the mavros local frame: home + (planned home - actual home) + north/east."""
        home = self.home
        shift_n = shift_e = 0.0
        if 'origin_lat' in self.meta and 'origin_lon' in self.meta:
            lat0, lon0 = float(self.meta['origin_lat']), float(self.meta['origin_lon'])
            lat, lon = home.geo.latitude, home.geo.longitude
            shift_n = (lat0 - lat) * METERS_PER_DEGREE
            shift_e = (lon0 - lon) * METERS_PER_DEGREE * math.cos(math.radians((lat0 + lat) / 2))
            gap = math.hypot(shift_n, shift_e)
            if gap > HOME_MISMATCH_WARN_M:
                self.get_logger().warn(
                    f'Le home est à {gap:.1f} m de celui du plan : la référence reste sur ses points GPS, '
                    'le début du tracé est décalé du drone d\'autant.')
        r = self.ref
        local = np.empty((len(r), 8))
        local[:, 0] = home.position.x + shift_e + r[:, 2]   # x east
        local[:, 1] = home.position.y + shift_n + r[:, 1]   # y north
        local[:, 2] = home.position.z + r[:, 7]             # z up, height above home
        local[:, 3] = r[:, 4]                                # vx = v east
        local[:, 4] = r[:, 3]                                # vy = v north
        local[:, 5] = r[:, 8]                                # vz up
        local[:, 6] = r[:, 6]                                # ax = a east
        local[:, 7] = r[:, 5]                                # ay = a north
        return local

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
        self.get_logger().info(f'Départ des tours : drone à {distance:.1f} m du début de la référence.')
        return True

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
        position = np.array([p.x, p.y, p.z])
        if not self.started and not self.start(position):
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

        if self.latency_compensation:
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            lead = min(max(age, 0.0) + self.uplink_delay, MAX_EXTRAPOLATION_S)
            position = position + self.velocity * lead

        self.advance(position[:2], dt)
        if self.s[-1] - self.s_prog < END_DISTANCE_M:
            self.finish()
            return
        sample = self.sample(self.s_prog)
        self.update_deconfliction(dt, float(np.hypot(sample[1][0], sample[1][1])))
        pos, vel, acc = self.deconflicted(sample)
        error = float(np.linalg.norm(pos[:2] - np.array([p.x, p.y])))
        if error > self.max_tracking_error:
            # feedforward coupé : le drone revient sur la consigne au lieu de suivre une vitesse
            # qui ne correspond plus à sa position (sinon il part en ligne droite)
            vel, acc = np.zeros(3), np.zeros(3)
            if not self.tracking_lost:
                self.get_logger().error(
                    f'Drone à {error:.0f} m de sa consigne : feedforward coupé, retour en position seule.')
            self.tracking_lost = True
        elif self.tracking_lost:
            self.tracking_lost = False
            self.get_logger().info('Drone revenu sur sa consigne : feedforward rétabli.')
        self.publish_setpoint(pos, vel, acc)

    def advance(self, xy, dt):
        """Moves the progression forward to the projection of xy, inside the per-tick window."""
        i0 = int(np.searchsorted(self.s, self.s_prog, side='right')) - 1
        speed = max(float(np.hypot(*self.local[i0, 3:5])) * self.speed_factor,
                    float(np.hypot(*self.velocity[:2])), MIN_SPEED_FOR_WINDOW)
        s_max = self.s_prog + ADVANCE_FACTOR * speed * dt + ADVANCE_MARGIN_M
        i1 = min(int(np.searchsorted(self.s, s_max, side='right')), len(self.s) - 1)
        pts = self.local[i0:i1 + 1, :2]
        k = i0 + int(np.argmin(np.hypot(*(pts - xy).T)))
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
            self.s_prog = min(self.s_prog, self.freeze_at)   # test only, see the module docstring

    def sample(self, s):
        """Reference at abscissa s, linear interpolation: (pos xyz, vel xyz, acc xyz), ENU."""
        i = min(int(np.searchsorted(self.s, s, side='right')) - 1, len(self.s) - 2)
        f = (s - self.s[i]) / (self.s[i + 1] - self.s[i])
        row = (1.0 - f) * self.local[i] + f * self.local[i + 1]
        pos = row[0:3]
        vel = row[3:6]
        acc = np.array([row[6], row[7], 0.0])
        return pos, vel, acc

    # ------------------------------------------------------------------ deconfliction hook
    def update_deconfliction(self, dt, v_ref):
        """Rate-limits the commanded speed factor, offset and velocity correction; keeps the rates
        of the first two for the feedforward."""
        step = self.speed_factor_accel / max(v_ref, 1.0) * dt
        k = self.speed_factor + min(max(self.speed_factor_cmd - self.speed_factor, -step), step)
        self.speed_factor_rate_now = (k - self.speed_factor) / dt if dt > 0 else 0.0
        self.speed_factor = k
        step = self.offset_rate * dt
        offset = self.offset + np.clip(self.offset_cmd - self.offset, -step, step)
        self.offset_rate_now = (offset - self.offset) / dt if dt > 0 else np.zeros(2)
        self.offset = offset
        target = self.correction_cmd
        if self.correction_stamp is None or \
                (self.get_clock().now() - self.correction_stamp).nanoseconds * 1e-9 > self.correction_timeout:
            target = np.zeros(2)            # silence of the deconfliction node: back to the line
        delta = target - self.correction
        step = self.correction_accel * dt
        norm = float(np.hypot(*delta))
        self.correction = self.correction + (delta if norm <= step else delta * (step / norm))

    def deconflicted(self, sample):
        """Applies the speed factor k (time scaling) and the lateral/vertical offset to a sample."""
        pos, vel, acc = sample
        k, k_dot = self.speed_factor, self.speed_factor_rate_now
        new_vel = k * vel
        new_acc = k * k * acc + k_dot * vel
        if np.any(self.offset != 0.0) or np.any(self.offset_rate_now != 0.0):
            speed = float(np.hypot(vel[0], vel[1]))
            if speed > 0.1:
                right = np.array([vel[1], -vel[0], 0.0]) / speed   # ENU: right of the direction of travel
                up = np.array([0.0, 0.0, 1.0])
                pos = pos + right * self.offset[0] + up * self.offset[1]
                new_vel = new_vel + right * self.offset_rate_now[0] + up * self.offset_rate_now[1]
        if np.any(self.correction != 0.0):
            c = np.array([self.correction[0], self.correction[1], 0.0])
            pos = pos + c * self.correction_lead
            new_vel = new_vel + c
        return pos, new_vel, new_acc

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
        """Holds the final point (zero velocity) for HOLD_TICKS poses, then reports and goes silent."""
        # no deconfliction offset here: the end point is the one planned inside the area
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
