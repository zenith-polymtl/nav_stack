import math, time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from tools import topics
from custom_interfaces.msg import NavIntent, NavMode
from mavros_msgs.msg import HomePosition

from nav_stack.laps_path_sim import LapsPathSimulator, SimulationParameters

LATITUDE = 0
LONGITUDE = 1
ALTITUDE = 2

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
MAVROS_HOME = '/mavros/home_position/home'

class Laps(Node):
    def __init__(self):
      """
      IMPORTANT: puisque le home n'est pas connu lorsqu'on roule la node,
        on doit décider de où il décolle. Modifier ce champ selon le lieu de décollage
        ou créer le chemin optimal lorsque le drone décolle (et que le vrai home est connu).
      
      Note: la conversion de GPS vers ENU est faite par rapport au self.home
      """
      super().__init__('laps')

      self.home = None
      self.home_handled = False
      self.takeoff_handled = False
      self.intent = False

      self.declare_parameter('task1.course.laps', Parameter.Type.INTEGER)
      self.declare_parameter('task1.course.lat', Parameter.Type.DOUBLE_ARRAY)
      self.declare_parameter('task1.course.lon', Parameter.Type.DOUBLE_ARRAY)
      self.declare_parameter('task1.course.altitude_agl', Parameter.Type.DOUBLE)

      self.declare_parameter('task1.search_area.lat', Parameter.Type.DOUBLE_ARRAY)
      self.declare_parameter('task1.search_area.lon', Parameter.Type.DOUBLE_ARRAY)

      self.initialize_publishers()
      self.initialize_subscriptions()

    def initialize_publishers(self):
      self.nav_intent_publisher = self.create_publisher(
          NavIntent, topics.NAV_INTENT, LATCHED)

    def initialize_subscriptions(self):
      # Trouver le bon QoS
      self.nav_mode_subscriber = self.create_subscription(
          NavMode, topics.NAV_MODE,
          self.received_nav_mode, LATCHED)

      self.home_sub = self.create_subscription(
          HomePosition, MAVROS_HOME, self.callback_home, LATCHED)
    
    def get_simulation_intent(self, home):
      # Modifier avec hexa.yaml
      params = SimulationParameters(v_max=20, a_max=3, tolerence=5.0, nb_laps=self.nb_laps, dt=0.05, path_resolution=1)
      
      home_enu = self.gps_to_enu_path([home], home)[0]

      # Le dernier point atteint par les laps est en fait le premier waypoint
      search_waypoint = self.get_search_zone_objective(home, self.waypoints[0])

      sim = LapsPathSimulator(
        self.waypoints,
        start_position=home_enu,
        end_position=search_waypoint,
        params=params
      )

      t_start = time.time()
      intent = sim.generate_simulation_intent(self.get_clock().now().to_msg())
      elapsed_time = time.time() - t_start

      self.get_logger().info(f'Simulation completed within {(elapsed_time * 1000):.2f} ms')

      return intent

    def callback_home(self, msg: HomePosition):
      if self.home_handled:
          return
      self.home_handled = True
      self.home = (msg.geo.latitude, msg.geo.longitude, 10.0)
      self.get_logger().info(f'Home reçu: {self.home[LATITUDE]:.7f}, {self.home[LONGITUDE]:.7f}')
      self.waypoints, self.nb_laps = self.initialize(self.home)
      self.intent = self.get_simulation_intent(self.home)

    def received_nav_mode(self, msg):
      if msg.mode == NavMode.LAPS:
        self.get_logger().info(f'NavMode reçu: {msg}')

        if self.intent is None:
          self.get_logger().info(f'NavIntent non précalculé... simulation en cours.')
          self.intent = self.get_simulation_intent(self.home)

        self.publish_intent(self.intent)
        self.get_logger().info(f'NavIntent publié!')

        self.save_intent(self.intent)
      
    def publish_intent(self, intent):
      self.nav_intent_publisher.publish(intent)
    
    def save_intent(self, intent, path=None):
      """
      Dans l'optique de faire des tests avec un autre programme,
      enregistrer les points à parcourir.
      """
      path = path or Path(__file__).resolve().parent / 'anim_info.txt'
      with open(path, 'a+', encoding='utf-8') as f:
        f.truncate(0)
        f.write(str(intent))

    def initialize(self, home):
      """
      Initialiser les paramètres de nombre de laps et la liste des waypoints
      en utilisant le fichier sitl_test.yaml (le nom du fichier va changer)
      """
      nb_laps = self.get_parameter('task1.course.laps').value
      waypoints = self.initialize_waypoints(home)

      return waypoints, nb_laps
    
    def initialize_waypoints(self, home):
      """
      Transformer les points en format GPS du .yaml en format ENU
      pour mavros et la simulation de laps_path_sim.py
      """
      lat_list = self.get_parameter('task1.course.lat').value   # liste de latitudes
      lon_list = self.get_parameter('task1.course.lon').value   # liste de longitudes
      altitude_agl = self.get_parameter('task1.course.altitude_agl').value

      gps_points = [
        (lat_list[i], lon_list[i], altitude_agl)
        for i in range(len(lat_list))
      ]

      return self.gps_to_enu_path(gps_points, home=home)
    
    def get_search_zone_objective(self, home, last_waypoint):
      """
      Retourne temporairement le point (en ENU) de la zone de recherche le plus proche
      du dernier waypoint des laps.
      """
      lat_list = self.get_parameter('task1.search_area.lat').value
      lon_list = self.get_parameter('task1.search_area.lon').value

      # On veut conserver la même hauteur que les laps
      altitude_agl = self.get_parameter('task1.course.altitude_agl').value

      gps_points = [
          (lat_list[i], lon_list[i], altitude_agl)
          for i in range(len(lat_list))
      ]

      enu_points = self.gps_to_enu_path(gps_points, home=home)

      # Distance horizontale seulement (même altitude partout)
      return min(
          enu_points,
          key=lambda p: math.hypot(p[0] - last_waypoint[0], p[1] - last_waypoint[1])
      )

    def gps_to_enu_path(self, gps_points, home):
      """
      À REMPLACER - par les conversions built-in
      que les autres programmes utilisent.
      """
      enu_points = []

      h_lat = home[LATITUDE]
      h_lon = home[LONGITUDE]
      h_alt = home[ALTITUDE]

      for gps_pt in gps_points:
        diff_lat = gps_pt[LATITUDE] - h_lat
        diff_lon = gps_pt[LONGITUDE] - h_lon
        dy = diff_lat * 111320
        dx = diff_lon * 111320 * math.cos(math.radians(h_lat))
        d_alt = gps_pt[ALTITUDE]
        enu_points.append((
          dx, dy, d_alt
        ))
      
      return enu_points

def main():
    rclpy.init()
    laps_node = Laps()

    rclpy.spin(laps_node)
    rclpy.shutdown()


"""
Pour faire rouler, dans l'architecture où le node a été fait
C=testing docker compose -f compose/sim.yml up -d mavros-sim

make down
C=testing docker compose -f compose/dev.yml up -d dev

make build C=hexa
make shell
source /aeac/workspaces/hexa_ws/install/setup.bash
ros2 run nav_stack laps_nav --ros-args --params-file /aeac/config/sites/medicine_hat.yaml

ros2 topic pub -r 1 --qos-durability transient_local --qos-reliability reliable \
  /mavros/home_position/home mavros_msgs/msg/HomePosition \
  "{geo: {latitude: 50.0978083, longitude: -110.7336480, altitude: 30.0}}"
"""
