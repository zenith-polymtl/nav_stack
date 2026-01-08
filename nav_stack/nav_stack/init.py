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
        self.home_set = False
        self.internal_ok = False
        self.external_ok = False
        self.go = False
        self.finished_takeoff = False
        self.takeoff_accepted = False
        self.successful_message_requests = 0

        self.set_up_parameters()
        self.set_up_services()
        self.set_up_topics()

        self.odom_rate = 25.0  # Desired rate for ODOM and controller messages
        # TODO : add parameter for odom rate
    
        #Create time to check and update state
        
        self.create_timer(1.0, self.check_state)

        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        # run once, 1s after startup

        self.setup_message_intervals()

        self.get_logger().info("init node ready.")

    def make_request(self, message_id, message_rate):
        request = MessageInterval.Request()  
        request.message_id = message_id  
        request.message_rate = message_rate  

        future = self.msg_interval_client.call_async(request)  
        future.add_done_callback(self.message_interval_callback) 

    
    def setup_message_intervals(self):
        if True:
            """Set up message intervals after node initialization"""  
            if not self.msg_interval_client.wait_for_service(timeout_sec=5.0):  
                self.get_logger().warn('Message interval service not available, aborting request...')  
                return 

            #global position messages
            self.make_request(33, self.odom_rate)

            #local position messages
            self.make_request(32, self.odom_rate)  

            #Controller messages
            self.make_request(65, self.odom_rate) 
            

    def message_interval_callback(self, future):  
        try:  
            response = future.result()  
            if response.success:  
                self.get_logger().info("Message interval set successfully")  
                self.successful_message_requests += 1
                if self.successful_message_requests >= 3:
                    self.get_logger().info("All message intervals set successfully")

                    if hasattr(self, 'setup_timer'):
                        self.destroy_timer(self.setup_timer)
            else:  
                self.get_logger().error("Failed to set message interval, retrying...")  
                self.setup_timer = self.create_timer(5.0, self.setup_message_intervals)
        except Exception as e:  
            self.get_logger().error(f"Service call failed: {e}") 

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

    def set_up_services(self):
        
        self.takeoff_client = self.create_client(  
            CommandTOL, '/mavros/cmd/takeoff'  
        )  

    def set_up_topics(self):
        #ici y a les publishers (etat, takeoff, rtl)
        self.status_pub = self.create_publisher(Bool, '/mission/increment_state', 10)   #noms des topics hardcoded mais je vais changer ca
        self.takeoff_pub = self.create_publisher(PoseStamped, '/drone/takeoff_cmd', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/mission/waypoint', 10)
        self.rtl_pub = self.create_publisher(Bool, '/drone/rtl', 10)
        self.home_pub = self.create_publisher(NavSatFix, '/mission/takeoff_point', 10)

        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.callback_gps, 10)
        #ici y a les subsciptions (le go, abort, internal, external) 
        self.go_sub = self.create_subscription(Bool, '/mission/go', self.callback_go, 10)
        self.abort_sub = self.create_subscription(Bool, '/mission/abort', self.callback_abort, 10)

    def callback_gps(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

        self.internal_ok = True  # Placeholder for actual internal readiness check
        #This message received means GPS stream is working, decent check
        if not self.home_set:
            home_msg = NavSatFix()
            home_msg.latitude = self.current_lat
            home_msg.longitude = self.current_lon
            home_msg.altitude = self.current_alt
            self.home_pub.publish(home_msg)
            self.home_set = True
            self.get_logger().info(f"Home position set to lat={self.current_lat}, lon={self.current_lon}, alt={self.current_alt}")
        
    def callback_go(self, msg):
        self.go = msg.data
        if self.go and self.ready and self.internal_ok and self.external_ok:
            self.start_mission()
            
            

    def callback_abort(self, msg):
        if msg.data:
            self.global_abort()

    def check_state(self):
        #Placeholders for actual checks
        self.ready = True
        self.external_ok = True
        if not self.go:
            if self.ready and self.internal_ok and self.external_ok:
                self.get_logger().info("System READY. Waiting for GO...")
            else:
                self.get_logger().info(f"System NOT READY : \ninternal_ok = {self.internal_ok} \nexternal_ok = {self.external_ok}")
            

    def send_takeoff_command(self):
        # Wait for service to be available  
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):  
            self.get_logger().info('takeoff service not available, waiting...')  
          
        # Create regular takeoff request for ArduCopter  
        req = CommandTOL.Request(  
            min_pitch=0.0,  
            yaw=0.0,  
            latitude=self.current_lat,  
            longitude=self.current_lon,  
            altitude=float(self.takeoff_alt)  
        )  
          
        # Call the service  
        future = self.takeoff_client.call_async(req)  
        future.add_done_callback(self.confirm_takeoff)  

        self.ground_takeoff_height = self.current_alt

    def check_takeoff_completion(self):
        if not self.finished_takeoff:
            self.finished_takeoff = (self.current_alt - self.ground_takeoff_height) >= (self.takeoff_alt - 2.0)
        else:
            self.status_pub.publish(Bool(data=True))  # Indicate takeoff finished
            self.destroy_timer(self.check_takeoff_timer)
            self.get_logger().info("Takeoff completed. Initializing finished")
            self.destroy_subscription(self.gps_sub)

    def confirm_takeoff(self, future):
        try:  
            response = future.result()  
            if response.success:  
                self.get_logger().info("Takeoff command succeeded")
                # Verify if there is no takeoff timer
                if not hasattr(self, 'check_takeoff_timer'):
                    self.check_takeoff_timer = self.create_timer(
                        0.5, self.check_takeoff_completion
                    )
                self.takeoff_accepted = True  
            else:  
                  
                if response.result == 4:
                    if self.takeoff_accepted:
                        self.get_logger().error("Takeoff rejected: Vehicle is already in air.")
                    else:
                        self.get_logger().error("Please check if vehicule is in GUIDED mode and armed.")
                else:
                    self.get_logger().error(f"Takeoff failed: {response.result}")

        except Exception as e:  
            self.get_logger().error(f"Takeoff service call failed: {e}")  

    def start_mission(self):

        self.get_logger().info("GO received : starting mission.")

        self.send_takeoff_command()


    def global_abort(self):
        self.status_pub.publish(String(data="ABORT"))
        self.rtl_pub.publish(Bool(data=True))
        
        self.get_logger().warn("GLOBAL ABORT RTL engaged.")




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