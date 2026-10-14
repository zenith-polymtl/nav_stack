#!/usr/bin/env python3
"""Deconfliction node: predictive repulsion from the competition traffic, as a velocity correction.

Input: topics.TRAFFIC (custom_interfaces/TrafficArray, about 1 Hz): the `traffic` event of the
competition server, one entry per aircraft (aircraftIndex, name, lat/lon, altitude AGL, speed,
course, keep-away distances) with the measurement stamp. Published by the competition server client
in flight, by sim_mocks traffic_mock in simulation.

Output: topics.DECONFLICTION_CORRECTION (geometry_msgs/Vector3, ENU m/s, z = 0), on every local pose
(stream_rate_hz of init, 20 Hz) once the home is known, ZERO WHEN NOTHING THREATENS. The flying node
(lap now, the survey later) adds it to its velocity feedforward. Publishing zero rather than nothing:
the flying node then tells "no threat" (zero, applied at once) from "deconfliction node down" (no
message: its correction decays to zero after its own timeout, correction_timeout_s of lap).

Strategy: pf_pred of context_research/research/laps_bench/deconfliction.py, the only one of the SITL
bench with no incursion on the four simple encounters. For each aircraft in vertical overlap
(|drone AGL - aircraft AGL| <= vka + vertical_margin_m):
  1. predict the aircraft at constant speed and turn rate, both estimated from its last three
     updates, from its last position aged by (now - stamp), over horizon_s at step_s;
  2. predict the drone in a straight line at its MEASURED velocity. The bench used the velocity of
     the lap reference; the node does not know the reference, so it serves the survey as well;
  3. take the closest point of approach (CPA) of the two predictions: distance d, and the direction
     from the aircraft to the drone at that instant;
  4. if d < d0 = hka + influence_margin_m, push gain_mps x ((d0 - d) / (d0 - hka))^2 m/s, at most
     max_push_mps, along that direction.
The pushes of all aircraft add up (no limit on the sum: lap limits its rate of change).

Velocity and turn rate: finite differences of the last updates (the bench's TrafficFeed.known):
v = (p2 - p1) / (t2 - t1), turn rate = angle(v0, v1) / ((t2 - t0) / 2), only when both speeds
exceed 0.5 m/s. With a single update, the server speed and course, no turn.

Frames: aircraft lat/lon go to the mavros local frame (ENU) like convert does: the flat-earth formula
of ArduPilot from the mavros home, plus the home position in that frame. Altitude: the server
compares our DECLARED altitude AGL; here the height above home (/mavros/global_position/rel_alt),
flat ground assumed.

History and age: per aircraftIndex, the last three updates (stamp, position). A new name on an
index, or a gap longer than max_traffic_age_s, restarts the history. An aircraft whose last update is
older than max_traffic_age_s is ignored (WARN once, INFO when updates come back). A zero stamp is
replaced by the reception time.

Logs: INFO when an aircraft starts or stops pushing (CPA inside d0), WARN when the prediction
announces a conflict (CPA within hka and vertical overlap within vka), once per episode.

The node never arms, never changes mode and never sends a setpoint.

Parameters (config/<mission>.yaml): horizon_s, step_s, influence_margin_m, gain_mps, max_push_mps,
vertical_margin_m, max_traffic_age_s.
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from mavros_msgs.msg import HomePosition
from std_msgs.msg import Float64

from custom_interfaces.msg import TrafficArray
from tools import topics

# mavros, package tiers : ses noms restent ici (ARCHITECTURE.md, règles de code)
MAVROS_POSE = '/mavros/local_position/pose'
MAVROS_VELOCITY = '/mavros/local_position/velocity_local'
MAVROS_HOME = '/mavros/home_position/home'
MAVROS_REL_ALT = '/mavros/global_position/rel_alt'

HISTORY = 3                  # updates kept per aircraft: velocity and turn rate
MIN_SPEED_FOR_TURN = 0.5     # m/s, below this the turn rate is not estimated (bench)

# ArduPilot LATLON_TO_M (AP_Math/definitions.h), per degree, the same formula as convert
METERS_PER_DEGREE = 0.011131884502145034 * 1e7

# Latched: a node started late still gets the last value
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def predict_intruder(p, v, omega, age, horizon, dt):
    """Constant turn rate prediction from the last known position p (age s old), as in the bench:
    returns the positions from now to now + horizon, every dt."""
    out, q, w = [], np.array(p, float), np.array(v, float)
    n_age = int(round(max(age, 0.0) / dt))
    c, s = math.cos(omega * dt), math.sin(omega * dt)
    for i in range(n_age + int(round(horizon / dt)) + 1):
        if i >= n_age:
            out.append(q.copy())
        w = np.array([c * w[0] - s * w[1], s * w[0] + c * w[1]])
        q = q + w * dt
    return np.array(out)


class Track:
    """What the node knows of one aircraft: its last HISTORY updates, and its log state."""

    def __init__(self, name):
        self.name = name
        self.samples = deque(maxlen=HISTORY)   # (stamp s, Traffic msg)
        self.stale = False
        self.threat = False
        self.conflict = False


class Deconfliction(Node):
    def __init__(self):
        super().__init__('deconfliction')

        self.declare_parameter('horizon_s', 10.0)
        self.declare_parameter('step_s', 0.5)
        self.declare_parameter('influence_margin_m', 50.0)
        self.declare_parameter('gain_mps', 15.0)
        self.declare_parameter('max_push_mps', 15.0)
        self.declare_parameter('vertical_margin_m', 3.0)
        self.declare_parameter('max_traffic_age_s', 3.0)
        self.horizon = self.get_parameter('horizon_s').value
        self.step = self.get_parameter('step_s').value
        self.influence_margin = self.get_parameter('influence_margin_m').value
        self.gain = self.get_parameter('gain_mps').value
        self.max_push = self.get_parameter('max_push_mps').value
        self.vertical_margin = self.get_parameter('vertical_margin_m').value
        self.max_age = self.get_parameter('max_traffic_age_s').value

        self.home = None
        self.rel_alt = None
        self.velocity = np.zeros(2)   # measured, ENU
        self.tracks = {}              # aircraftIndex -> Track
        self.warned_no_home = False

        self.correction_pub = self.create_publisher(Vector3, topics.DECONFLICTION_CORRECTION, 10)

        self.create_subscription(HomePosition, MAVROS_HOME, self.callback_home, LATCHED)
        self.create_subscription(Float64, MAVROS_REL_ALT, self.callback_rel_alt, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, MAVROS_VELOCITY, self.callback_velocity, qos_profile_sensor_data)
        self.create_subscription(TrafficArray, topics.TRAFFIC, self.callback_traffic, 10)
        self.create_subscription(PoseStamped, MAVROS_POSE, self.callback_pose, qos_profile_sensor_data)

        self.get_logger().info(
            f'Node deconfliction prête : prédiction {self.horizon:.0f} s, portée keep-away + '
            f'{self.influence_margin:.0f} m, poussée max {self.max_push:.0f} m/s. En attente du home et du trafic.')

    # ------------------------------------------------------------------ inputs
    def callback_home(self, msg):
        position = (msg.position.x, msg.position.y, msg.position.z)
        if any(math.isnan(v) for v in position):
            self.get_logger().warn("Home reçu sans position locale (pas encore d'origine EKF) : ignoré.")
            return
        self.home = msg

    def callback_rel_alt(self, msg):
        self.rel_alt = msg.data

    def callback_velocity(self, msg):
        self.velocity = np.array([msg.twist.linear.x, msg.twist.linear.y])

    def callback_traffic(self, msg):
        now = self.now_s()
        for a in msg.aircraft:
            stamp = Time.from_msg(a.stamp).nanoseconds * 1e-9 if (a.stamp.sec or a.stamp.nanosec) else now
            track = self.tracks.get(a.index)
            if track is None or track.name != a.name:
                track = self.tracks[a.index] = Track(a.name)
                self.get_logger().info(
                    f'Trafic : avion {a.name} (#{a.index}), keep-away {a.horizontal_keep_away:.0f} m '
                    f'horizontal, {a.vertical_keep_away:.0f} m vertical, à {a.altitude_agl:.0f} m AGL.')
            elif track.samples:
                last = track.samples[-1][0]
                if stamp <= last:
                    continue                         # same update pushed again
                if stamp - last > self.max_age:
                    track.samples.clear()            # too old to estimate a velocity with
            track.samples.append((stamp, a))
            if track.stale:
                track.stale = False
                self.get_logger().info(f'Trafic {a.name} (#{a.index}) : mises à jour revenues.')
        if self.home is None and not self.warned_no_home:
            self.warned_no_home = True
            self.get_logger().warn('Trafic reçu avant le home de mavros : pas de correction tant que le home manque.')

    # ------------------------------------------------------------------ geometry
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def to_local(self, lat, lon):
        """lat/lon -> mavros local frame (ENU, x east, y north), as convert does."""
        h = self.home
        north = (lat - h.geo.latitude) * METERS_PER_DEGREE
        east = (lon - h.geo.longitude) * METERS_PER_DEGREE * math.cos(math.radians((lat + h.geo.latitude) / 2))
        return np.array([h.position.x + east, h.position.y + north])

    def estimate(self, track):
        """Last position (ENU), velocity, turn rate (rad/s, counterclockwise positive) of a track."""
        stamps = [s for s, _ in track.samples]
        pts = [self.to_local(a.latitude, a.longitude) for _, a in track.samples]
        last = track.samples[-1][1]
        if len(pts) == 1:
            course = math.radians(last.course_deg)
            return pts[-1], last.speed * np.array([math.sin(course), math.cos(course)]), 0.0
        v1 = (pts[-1] - pts[-2]) / (stamps[-1] - stamps[-2])
        omega = 0.0
        if len(pts) == 3:
            v0 = (pts[-2] - pts[-3]) / (stamps[-2] - stamps[-3])
            if np.linalg.norm(v0) > MIN_SPEED_FOR_TURN and np.linalg.norm(v1) > MIN_SPEED_FOR_TURN:
                angle = math.atan2(v0[0] * v1[1] - v0[1] * v1[0], float(v0 @ v1))
                omega = angle / ((stamps[-1] - stamps[-3]) / 2)
        return pts[-1], v1, omega

    # ------------------------------------------------------------------ the loop, on each pose
    def callback_pose(self, msg):
        if self.home is None or self.rel_alt is None:
            return
        now = self.now_s()
        position = np.array([msg.pose.position.x, msg.pose.position.y])
        push = np.zeros(2)
        for index, track in self.tracks.items():
            push += self.repulsion(index, track, now, position)
        self.correction_pub.publish(Vector3(x=float(push[0]), y=float(push[1]), z=0.0))

    def repulsion(self, index, track, now, position):
        """pf_pred for one aircraft: the push (ENU m/s) it causes, and the threat/conflict logs."""
        stamp, last = track.samples[-1]
        age = now - stamp
        if age > self.max_age:
            if not track.stale:
                track.stale = True
                self.get_logger().warn(
                    f'Trafic {track.name} (#{index}) sans mise à jour depuis {age:.1f} s : ignoré.')
            self.set_threat(index, track, False)
            return np.zeros(2)
        hka, vka = last.horizontal_keep_away, last.vertical_keep_away
        dz = abs(self.rel_alt - last.altitude_agl)
        if dz > vka + self.vertical_margin:
            self.set_threat(index, track, False)
            return np.zeros(2)

        p, v, omega = self.estimate(track)
        intruder = predict_intruder(p, v, omega, age, self.horizon, self.step)
        drone = position + np.outer(np.arange(len(intruder)) * self.step, self.velocity)
        i_cpa = int(np.argmin(np.linalg.norm(drone - intruder, axis=1)))
        d_vec = drone[i_cpa] - intruder[i_cpa]
        d = float(np.linalg.norm(d_vec))
        t_cpa = i_cpa * self.step

        conflict = d <= hka and dz <= vka
        if conflict and not track.conflict:
            self.get_logger().warn(
                f'Conflit prévu avec {track.name} (#{index}) : {d:.0f} m au plus près dans {t_cpa:.1f} s '
                f'(keep-away {hka:.0f} m, écart vertical {dz:.0f} m).')
        track.conflict = conflict

        d0 = hka + self.influence_margin
        if d >= d0:
            self.set_threat(index, track, False)
            return np.zeros(2)
        magnitude = min(self.gain * ((d0 - d) / (d0 - hka)) ** 2, self.max_push)
        self.set_threat(index, track, True, d, t_cpa, magnitude)
        return d_vec / max(d, 1e-3) * magnitude

    def set_threat(self, index, track, threat, d=0.0, t_cpa=0.0, magnitude=0.0):
        if threat and not track.threat:
            self.get_logger().info(
                f'Menace : {track.name} (#{index}), {d:.0f} m au plus près dans {t_cpa:.1f} s, '
                f'poussée {magnitude:.1f} m/s.')
        elif track.threat and not threat:
            self.get_logger().info(f'Fin de menace : {track.name} (#{index}).')
        if not threat:
            track.conflict = False
        track.threat = threat


def main(args=None):
    rclpy.init(args=args)
    node = Deconfliction()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
