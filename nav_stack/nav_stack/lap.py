#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Vector3 
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import MessageInterval 
from sensor_msgs.msg import NavSatFix

class MissionInit(Node):
    def __init__(self):
        super().__init__('mission_init')

        
        self.ready = False
        self.internal_ok = False
        self.external_ok = False
        self.go = False
        self.finished_takeoff = False

        self.set_up_parameters()
        self.set_up_services()
        self.set_up_topics()

        self.odom_rate = 25.0  # Desired rate for ODOM and ODOM_COV messages
        # TODO : add parameter for odom rate
    
        #Create time to check and update state
        
        self.create_timer(1.0, self.check_state)

        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        # run once, 1s after startup
        self.setup_timer = self.create_timer(1.0, self.setup_message_intervals)

        self.get_logger().info("init node ready.")

    
    def setup_message_intervals(self):
        if True:
            """Set up message intervals after node initialization"""  
            if not self.msg_interval_client.wait_for_service(timeout_sec=5.0):  
                self.get_logger().warn('Message interval service not available, aborting request...')  
                self.destroy_timer(self.setup_timer) 
                return  
            
            request = MessageInterval.Request()  
            request.message_id = 32  
            request.message_rate = self.odom_rate

            request2 = MessageInterval.Request()  
            request2.message_id = 33  
            request2.message_rate = self.odom_rate

            future = self.msg_interval_client.call_async(request2)  
            future.add_done_callback(self.message_interval_callback) 

            future2 = self.msg_interval_client.call_async(request)  
            future2.add_done_callback(self.message_interval_callback)
            
        # Destroy the timer since we only need to run this once  
        self.destroy_timer(self.setup_timer) 

    def set_up_parameters(self):
        self.declare_parameter("takeoff_alt", 10.0)
        self.takeoff_alt = self.get_parameter("takeoff_alt").value

        default_waypoint = -74.0, 36.0, 10.0

        # parametres x, y, z coord du point avec des valeurs par defaut (5, 5, 10)
        self.declare_parameter("waypoint1_lat", default_waypoint[0])
        self.declare_parameter("waypoint1_lon", default_waypoint[1])
        self.declare_parameter("waypoint1_height", default_waypoint[2])

        # recupere les coord
        self.waypoint1_lat = self.get_parameter("waypoint1_lat").value
        self.waypoint1_lon = self.get_parameter("waypoint1_lon").value
        self.waypoint1_height = self.get_parameter("waypoint1_height").value

    def set_up_topics(self):
        #ici y a les publishers (etat, takeoff, rtl)
        self.status_pub = self.create_publisher(Bool, '/mission/increment_state', 10)   #noms des topics hardcoded mais je vais changer ca
        self.takeoff_pub = self.create_publisher(PoseStamped, '/drone/takeoff_cmd', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/mission/waypoint', 10)
        self.rtl_pub = self.create_publisher(Bool, '/drone/rtl', 10)

        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.callback_gps, 10)

        #ici y a les subsciptions (le go, abort, internal, external) 
        self.go_sub = self.create_subscription(Bool, '/mission/go', self.callback_go, 10)
        self.abort_sub = self.create_subscription(Bool, '/mission/abort', self.callback_abort, 10)

    def callback_gps(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

        self.internal_ok = True  # Placeholder for actual internal readiness check




def main(args=None):
    rclpy.init(args=args)
    node = MissionInit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()