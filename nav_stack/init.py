#!/usr/bin/env python3
"""Init node: raises the MAVLink stream rates, publishes the takeoff point, then takes off on GO.

Arming and GUIDED mode stay with the pilot: on GO the node only sends the takeoff command, and
reports the outcome on topics.TAKEOFF_COMPLETED (True once at altitude, False if refused).

Parameters (config/<mission>.yaml): takeoff_alt (m above home, required), stream_rate_hz.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

from std_msgs.msg import Bool, Float64
from mavros_msgs.srv import CommandTOL, MessageInterval
from sensor_msgs.msg import NavSatFix

from tools import topics

# mavros, package tiers : ses noms restent ici (ARCHITECTURE.md, règles de code)
MAVROS_GLOBAL_POSITION = '/mavros/global_position/global'
MAVROS_REL_ALT = '/mavros/global_position/rel_alt'
MAVROS_TAKEOFF = '/mavros/cmd/takeoff'
MAVROS_SET_MESSAGE_INTERVAL = '/mavros/set_message_interval'

# MAVLink messages streamed at stream_rate_hz: local and global position, RC channels
MAVLINK_LOCAL_POSITION_NED = 32
MAVLINK_GLOBAL_POSITION_INT = 33   # also feeds rel_alt
MAVLINK_RC_CHANNELS = 65
STREAMED_MESSAGES = (MAVLINK_GLOBAL_POSITION_INT, MAVLINK_LOCAL_POSITION_NED, MAVLINK_RC_CHANNELS)

MAV_RESULT_FAILED = 4   # CommandTOL result: not armed in GUIDED, or already flying
TAKEOFF_ALT_TOLERANCE_M = 2.0

# Latched: a node started late (convert, the mission) still gets the last value
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class MissionInit(Node):
    def __init__(self):
        super().__init__('mission_init')

        self.declare_parameter('takeoff_alt', Parameter.Type.DOUBLE)
        self.declare_parameter('stream_rate_hz', 25.0)
        self.takeoff_alt = self.get_parameter('takeoff_alt').value
        self.stream_rate = self.get_parameter('stream_rate_hz').value

        self.fix = None   # last NavSatFix with a position
        self.takeoff_requested = False
        self.successful_stream_requests = 0

        self.takeoff_client = self.create_client(CommandTOL, MAVROS_TAKEOFF)
        self.msg_interval_client = self.create_client(MessageInterval, MAVROS_SET_MESSAGE_INTERVAL)

        self.takeoff_point_pub = self.create_publisher(NavSatFix, topics.TAKEOFF_POINT, LATCHED)
        self.takeoff_completed_pub = self.create_publisher(Bool, topics.TAKEOFF_COMPLETED, LATCHED)

        self.gps_sub = self.create_subscription(
            NavSatFix, MAVROS_GLOBAL_POSITION, self.callback_gps, qos_profile_sensor_data)
        self.go_sub = self.create_subscription(Bool, topics.MISSION_GO, self.callback_go, 10)
        self.rel_alt_sub = None   # created once the takeoff is accepted

        # Waits for mavros without blocking, then cancels itself
        self.stream_timer = self.create_timer(1.0, self.request_stream_rates)

        self.get_logger().info('Node init prête, en attente du GPS puis du GO.')

    def request_stream_rates(self):
        if not self.msg_interval_client.service_is_ready():
            return
        self.stream_timer.cancel()
        self.successful_stream_requests = 0
        for message_id in STREAMED_MESSAGES:
            request = MessageInterval.Request(message_id=message_id, message_rate=self.stream_rate)
            self.msg_interval_client.call_async(request).add_done_callback(self.stream_rate_done)

    def stream_rate_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Appel de {MAVROS_SET_MESSAGE_INTERVAL} échoué : {e}')
            return
        if response.success:
            self.successful_stream_requests += 1
            if self.successful_stream_requests == len(STREAMED_MESSAGES):
                self.get_logger().info(f'Messages MAVLink de position et RC réglés à {self.stream_rate} Hz')
        elif self.stream_timer.is_canceled():   # one retry per batch, even if several requests fail
            self.get_logger().warn('Réglage de fréquence MAVLink refusé, nouvel essai')
            self.stream_timer.reset()

    def callback_gps(self, msg):
        # global is the EKF output (GLOBAL_POSITION_INT): zero until the EKF has an origin. Its
        # status field comes from GPS_RAW_INT, often not streamed on the mavros link: not reliable.
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        first_fix = self.fix is None
        self.fix = msg
        # The first fix gives convert an origin even if the pilot takes off by hand, without GO
        if first_fix:
            self.publish_takeoff_point()

    def callback_go(self, msg):
        if not msg.data or self.takeoff_requested:
            return
        if self.fix is None:
            self.get_logger().warn('GO reçu sans position GPS : ignoré.')
            return
        self.get_logger().info('GO reçu : décollage.')
        self.publish_takeoff_point()   # the origin is where the drone stands at GO
        self.send_takeoff_command()

    def publish_takeoff_point(self):
        self.takeoff_point_pub.publish(self.fix)
        self.get_logger().info(
            f'Point de décollage : lat={self.fix.latitude:.7f}, lon={self.fix.longitude:.7f}')

    def send_takeoff_command(self):
        if not self.takeoff_client.service_is_ready():
            self.get_logger().error(f'{MAVROS_TAKEOFF} indisponible : décollage non envoyé, renvoyer le GO.')
            return
        self.takeoff_requested = True
        # ArduCopter in GUIDED reads only the altitude, relative to home
        request = CommandTOL.Request(altitude=float(self.takeoff_alt))
        self.takeoff_client.call_async(request).add_done_callback(self.takeoff_done)

    def takeoff_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Appel de {MAVROS_TAKEOFF} échoué : {e}')
            self.takeoff_requested = False
            return

        if response.success:
            self.get_logger().info(f'Décollage accepté, montée à {self.takeoff_alt} m')
            self.rel_alt_sub = self.create_subscription(
                Float64, MAVROS_REL_ALT, self.callback_rel_alt, qos_profile_sensor_data)
            return

        self.takeoff_requested = False
        if response.result == MAV_RESULT_FAILED:
            self.get_logger().error('Décollage refusé : vérifier que le pilote a armé en GUIDED.')
        else:
            self.get_logger().error(f'Décollage refusé, code {response.result}')
        self.takeoff_completed_pub.publish(Bool(data=False))

    def callback_rel_alt(self, msg):
        if msg.data < self.takeoff_alt - TAKEOFF_ALT_TOLERANCE_M:
            return
        self.destroy_subscription(self.rel_alt_sub)
        self.destroy_subscription(self.gps_sub)
        self.takeoff_completed_pub.publish(Bool(data=True))
        self.get_logger().info(f'Décollage terminé à {msg.data:.1f} m.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionInit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
