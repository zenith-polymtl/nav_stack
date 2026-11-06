#!/usr/bin/env python3
"""Init node: raises the MAVLink stream rates, then takes off on GO.

Arming and GUIDED mode stay with the pilot: on GO the node only sends the takeoff command, and
reports the outcome on topics.TAKEOFF_COMPLETED (True once at altitude, False if refused).
The takeoff point is the autopilot home, set at arming: convert reads it from mavros.

Parameters (config/<mission>.yaml): takeoff_alt (m above home, required), stream_rate_hz.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

from std_msgs.msg import Bool, Float64
from mavros_msgs.srv import CommandTOL, MessageInterval

from tools import topics

# mavros, package tiers : ses noms restent ici (ARCHITECTURE.md, règles de code)
MAVROS_REL_ALT = '/mavros/global_position/rel_alt'
MAVROS_TAKEOFF = '/mavros/cmd/takeoff'
MAVROS_SET_MESSAGE_INTERVAL = '/mavros/set_message_interval'

# MAVLink messages requested.
# Pour rajouter un message :
# 1. ajouter son ID MAVLink dans MAVLINK_XXX ci-dessous.
# 2. ajouter son taux de streaming dans STREAM_RATES_HZ ci-dessous.
# 3. ajouter une ligne dans la fonction stream_rate_done() pour loguer le taux demandé.
MAVLINK_SYS_STATUS = 1
MAVLINK_SYSTEM_TIME = 2
MAVLINK_GPS_RAW_INT = 24

MAVLINK_LOCAL_POSITION_NED = 32
MAVLINK_GLOBAL_POSITION_INT = 33   # also feeds rel_alt
MAVLINK_RC_CHANNELS = 65

STREAM_RATES_HZ = {
    MAVLINK_SYS_STATUS: 1.0,              # comp_client
    MAVLINK_SYSTEM_TIME: 1.0,             # comp_client
    MAVLINK_GPS_RAW_INT: 10.0,            # comp_client
    MAVLINK_LOCAL_POSITION_NED: 25.0,
    MAVLINK_GLOBAL_POSITION_INT: 25.0,
    MAVLINK_RC_CHANNELS: 25.0
    }

MAV_RESULT_FAILED = 4   # CommandTOL result: not armed in GUIDED, or already flying
TAKEOFF_ALT_TOLERANCE_M = 2.0

# Latched: a node started late (the mission) still gets the last value
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class MissionInit(Node):
    def __init__(self):
        super().__init__('mission_init')

        self.declare_parameter('takeoff_alt', Parameter.Type.DOUBLE)
        self.takeoff_alt = self.get_parameter('takeoff_alt').value

        self.takeoff_requested = False
        self.successful_stream_requests = 0

        self.takeoff_client = self.create_client(CommandTOL, MAVROS_TAKEOFF)
        self.msg_interval_client = self.create_client(MessageInterval, MAVROS_SET_MESSAGE_INTERVAL)

        self.takeoff_completed_pub = self.create_publisher(Bool, topics.TAKEOFF_COMPLETED, LATCHED)

        self.go_sub = self.create_subscription(Bool, topics.MISSION_GO, self.callback_go, 10)
        self.rel_alt_sub = None   # created once the takeoff is accepted

        # Waits for mavros without blocking, then cancels itself
        self.stream_timer = self.create_timer(1.0, self.request_stream_rates)

        self.get_logger().info('Node init prête, en attente du GO.')

    # ------------------------------------------------------------------ 
    # mavros message rate
    # ------------------------------------------------------------------
    def request_stream_rates(self):
        if not self.msg_interval_client.service_is_ready():
            return
        self.stream_timer.cancel()
        self.successful_stream_requests = 0
        for message_id, rate in STREAM_RATES_HZ.items():
            request = MessageInterval.Request(message_id=message_id, message_rate=rate)
            self.msg_interval_client.call_async(request).add_done_callback(self.stream_rate_done)

    def stream_rate_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Appel de {MAVROS_SET_MESSAGE_INTERVAL} échoué : {e}')
            return
        if response.success:
            self.successful_stream_requests += 1
            if self.successful_stream_requests == len(STREAM_RATES_HZ):
                self.get_logger().info(f'Messages MAVLink réglés à : '
                                       f'SYS_STATUS {STREAM_RATES_HZ[MAVLINK_SYS_STATUS]} Hz, '
                                       f'SYSTEM_TIME {STREAM_RATES_HZ[MAVLINK_SYSTEM_TIME]} Hz, '
                                       f'GPS_RAW_INT {STREAM_RATES_HZ[MAVLINK_GPS_RAW_INT]} Hz, '
                                       f'LOCAL_POSITION_NED {STREAM_RATES_HZ[MAVLINK_LOCAL_POSITION_NED]} Hz, '
                                       f'GLOBAL_POSITION_INT {STREAM_RATES_HZ[MAVLINK_GLOBAL_POSITION_INT]} Hz, '
                                       f'RC_CHANNELS {STREAM_RATES_HZ[MAVLINK_RC_CHANNELS]} Hz')
        elif self.stream_timer.is_canceled():   # one retry per batch, even if several requests fail
            self.get_logger().warn('Réglage de fréquence MAVLink refusé, nouvel essai')
            self.stream_timer.reset()

    # ------------------------------------------------------------------
    #
    # ------------------------------------------------------------------
    
    def callback_go(self, msg):
        if not msg.data or self.takeoff_requested:
            return
        self.get_logger().info('GO reçu : décollage.')
        self.send_takeoff_command()

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
