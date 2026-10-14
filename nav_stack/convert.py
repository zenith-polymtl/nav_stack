#!/usr/bin/env python3
"""Convert node: the topics.CONVERT_GPS_TO_LOCAL service turns a GPS point into a local ENU point.

The result is meant for the mavros local frame (/mavros/setpoint_raw/local, compared with
/mavros/local_position/pose): x east, y north, z up, in meters from the EKF origin. The reference
is the autopilot home from mavros: its GPS position (geo) and its position in that local frame
(position), so the result is exact even if home is not the EKF origin. ArduCopter sets home at the
first EKF position, resets it at arming and sends it on every change; mavros keeps the last one.
gps_point.altitude is a height above home (the 2026 waypoints carried 10.0 for 10 m), not an
altitude above sea level.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from mavros_msgs.msg import HomePosition

from custom_interfaces.srv import ConvertGpsToLocal

from tools import topics

# mavros, package tiers : ses noms restent ici (ARCHITECTURE.md, règles de code)
MAVROS_HOME = '/mavros/home_position/home'

# mavros publishes home latched: a node started late still gets the last value
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
LOCAL_FRAME = 'map'   # mavros local frame

# ArduPilot LATLON_TO_M (AP_Math/definitions.h), per degree: spherical earth of radius 6378100 m
METERS_PER_DEGREE = 0.011131884502145034 * 1e7


class GpsToLocal(Node):
    def __init__(self):
        super().__init__('gps_to_local')

        self.home = None   # last mavros HomePosition with a usable position

        self.create_subscription(HomePosition, MAVROS_HOME, self.callback_home, LATCHED)
        self.create_service(ConvertGpsToLocal, topics.CONVERT_GPS_TO_LOCAL, self.convert_to_local)

        self.get_logger().info("Node gps_to_local prête, en attente du home de l'autopilote.")

    def callback_home(self, msg):
        # position is NaN when the autopilot has no EKF origin yet
        position = (msg.position.x, msg.position.y, msg.position.z)
        if any(math.isnan(v) for v in position):
            self.get_logger().warn("Home reçu sans position locale (pas encore d'origine EKF) : ignoré.")
            return
        self.home = msg
        self.get_logger().info(
            f'Home : lat={msg.geo.latitude:.7f}, lon={msg.geo.longitude:.7f}, '
            f'local x={msg.position.x:.2f}, y={msg.position.y:.2f}, z={msg.position.z:.2f}')

    def convert_to_local(self, request, response):
        if self.home is None:
            self.get_logger().warn('Conversion demandée avant de recevoir le home : refusée.')
            response.success = False
            return response

        lat, lon = request.gps_point.latitude, request.gps_point.longitude
        home_lat, home_lon = self.home.geo.latitude, self.home.geo.longitude

        # Same flat-earth formula as the EKF (Location::get_distance_NE), not a geodesic: the
        # result then lands where the autopilot itself puts this GPS point
        north = (lat - home_lat) * METERS_PER_DEGREE
        east = (lon - home_lon) * METERS_PER_DEGREE * math.cos(math.radians((lat + home_lat) / 2))

        # Home is not always the EKF origin: shift by its position in the local frame
        home_local = self.home.position
        response.local_point.header.stamp = self.get_clock().now().to_msg()
        response.local_point.header.frame_id = LOCAL_FRAME
        response.local_point.pose.position.x = home_local.x + east
        response.local_point.pose.position.y = home_local.y + north
        response.local_point.pose.position.z = home_local.z + float(request.gps_point.altitude)
        response.success = True
        self.get_logger().debug(
            f'({lat:.7f}, {lon:.7f}) -> x={response.local_point.pose.position.x:.2f}, '
            f'y={response.local_point.pose.position.y:.2f}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GpsToLocal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
