#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geopy.distance import distance
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

from custom_interfaces.srv import ConvertGpsToLocal


class GpsPoint:
    def __init__(self, lat=0.0, lon=0.0, alt=0.0):
        self.lat = lat
        self.lon = lon
        self.alt = alt


class GpsToLocal(Node):
    def __init__(self):
        super().__init__("gps_to_local")

        self.home = None  # important
        self.home_set = False

        self.create_subscription(
            NavSatFix, "/mission/takeoff_point", self.set_origin_callback, 10
        )

        self.srv = self.create_service(
            ConvertGpsToLocal, "convert/gps_to_local", self.convert_to_local
        )

        self.get_logger().info("gps_to_local node started.")

    def set_origin_callback(self, msg: NavSatFix):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            self.get_logger().warn("Received invalid GPS origin (0.0, 0.0). Ignoring.")
            return
        self.home = GpsPoint(msg.latitude, msg.longitude, msg.altitude)
        self.home_set = True
        self.get_logger().info(
            f"Origin set: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}"
        )

    def convert_to_local(self, request, response):
        if not self.home_set:
            response.local_point = PoseStamped()
            response.success = False
            self.get_logger().warn("Service called but origin not set yet.")
            return response

        fix = request.gps_point
        lat = float(fix.latitude)
        lon = float(fix.longitude)
        alt = float(fix.altitude)

        self.get_logger().info(
            f"Converting GPS point: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}"
        )

        ref_lat, ref_lon, ref_alt = self.home.lat, self.home.lon, self.home.alt



        # Northing (m)
        dN = distance((ref_lat, ref_lon), (lat, ref_lon)).meters
        if lat < ref_lat:
            dN = -dN

        # Easting (m)
        dE = distance((ref_lat, ref_lon), (ref_lat, lon)).meters
        if lon < ref_lon:
            dE = -dE

        # Up (m) relative to home
        dU = alt

        local_pos = PoseStamped()
        local_pos.header.stamp = self.get_clock().now().to_msg()
        local_pos.header.frame_id = "map"  # or parameterize

        # ENU convention:
        local_pos.pose.position.x = float(dE)
        local_pos.pose.position.y = float(dN)
        local_pos.pose.position.z = float(dU)

        response.local_point = local_pos
        self.get_logger().info(
            f"Converted to local ENU: x={dE:.2f}, y={dN:.2f}, z={dU:.2f}"
        )
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GpsToLocal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
