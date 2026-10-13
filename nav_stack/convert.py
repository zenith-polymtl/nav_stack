#!/usr/bin/env python3
"""Convert node: the topics.CONVERT_GPS_TO_LOCAL service turns a GPS point into a local ENU point.

The origin is the takeoff point published (latched) by init. The result is meant for the mavros
local frame (/mavros/setpoint_raw/local, compared with /mavros/local_position/pose): x east,
y north, in meters from the origin. gps_point.altitude is a height above the origin, passed
through unchanged as z (the 2026 waypoints carried 10.0 for 10 m), not an altitude above sea level.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from geopy.distance import distance
from sensor_msgs.msg import NavSatFix

from custom_interfaces.srv import ConvertGpsToLocal

from tools import topics

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
LOCAL_FRAME = 'map'   # mavros local frame


class GpsToLocal(Node):
    def __init__(self):
        super().__init__('gps_to_local')

        self.origin = None   # (lat, lon) of the takeoff point

        self.create_subscription(NavSatFix, topics.TAKEOFF_POINT, self.set_origin_callback, LATCHED)
        self.create_service(ConvertGpsToLocal, topics.CONVERT_GPS_TO_LOCAL, self.convert_to_local)

        self.get_logger().info('Node gps_to_local prête, en attente du point de décollage.')

    def set_origin_callback(self, msg):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            self.get_logger().warn('Origine GPS invalide (0.0, 0.0) : ignorée.')
            return
        self.origin = (msg.latitude, msg.longitude)
        self.get_logger().info(f'Origine fixée : lat={msg.latitude:.7f}, lon={msg.longitude:.7f}')

    def convert_to_local(self, request, response):
        if self.origin is None:
            self.get_logger().warn("Conversion demandée avant de recevoir l'origine : refusée.")
            response.success = False
            return response

        lat, lon = request.gps_point.latitude, request.gps_point.longitude
        ref_lat, ref_lon = self.origin

        # Distances along the meridian and the parallel of the origin, signed by direction
        north = distance(self.origin, (lat, ref_lon)).meters
        east = distance(self.origin, (ref_lat, lon)).meters
        if lat < ref_lat:
            north = -north
        if lon < ref_lon:
            east = -east

        response.local_point.header.stamp = self.get_clock().now().to_msg()
        response.local_point.header.frame_id = LOCAL_FRAME
        response.local_point.pose.position.x = east
        response.local_point.pose.position.y = north
        response.local_point.pose.position.z = float(request.gps_point.altitude)
        response.success = True
        self.get_logger().debug(f'({lat:.7f}, {lon:.7f}) -> x={east:.2f}, y={north:.2f}')
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
