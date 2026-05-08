#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget


class CmdVelToMavrosBasic(Node):
    def __init__(self):
        super().__init__("cmdvel_to_mavros_basic")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("setpoint_topic", "/mavros/setpoint_raw/local")
        self.declare_parameter("fixed_altitude", 20.0)
        self.declare_parameter("fixed_yaw", 0.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.setpoint_topic = self.get_parameter("setpoint_topic").value
        self.fixed_altitude = float(self.get_parameter("fixed_altitude").value)
        self.fixed_yaw = float(self.get_parameter("fixed_yaw").value)

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_cb, 10)
        self.pub = self.create_publisher(PositionTarget, self.setpoint_topic, 10)

    def cmd_cb(self, msg):
        speed = float(msg.linear.x)

        vx = speed * math.cos(self.fixed_yaw)
        vy = speed * math.sin(self.fixed_yaw)

        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        sp.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        sp.velocity.x = vx
        sp.velocity.y = vy
        sp.velocity.z = 0.0

        sp.position.z = self.fixed_altitude
        sp.yaw = self.fixed_yaw

        self.pub.publish(sp)


def main():
    rclpy.init()
    node = CmdVelToMavrosBasic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()