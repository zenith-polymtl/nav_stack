import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import math
from nav_stack.laps_path_sim import LapsPathSimulator, SimulationParameters
import time
from pathlib import Path

LATITUDE = 0
LONGITUDE = 1
ALTITUDE = 2

class Laps(Node):
    def __init__(self):
        super().__init__('laps')
        """
        IMPORTANT: puisque le home n'est pas connu lorsqu'on roule la node,
          on doit décider de où il décolle. Modifier ce champ selon le lieu de décollage
          ou créer le chemin optimal lorsque le drone décolle (et que le vrai home est connu).
        
        Note: la conversion de GPS vers ENU est faite par rapport au self.home
        """

        # Position de départ dans Mission Planner (depuis sitl.yaml)
        self.declare_parameter('home_sitl.lat', Parameter.Type.DOUBLE)
        self.declare_parameter('home_sitl.lon', Parameter.Type.DOUBLE)
        self.home = (self.get_parameter('home_sitl.lat').value,
                    self.get_parameter('home_sitl.lon').value, 10.0)  # Le path commence à partir d'un takeoff à 10 m

        # Initialiser les waypoints et le nombre de laps (provenant de config/sites/sitl.yaml)
        self.waypoints, self.nb_laps = self.initialize()
        
        for i, (x, y, z) in enumerate(self.waypoints):
          self.get_logger().info(f'WP{i}: x={x:.1f} y={y:.1f} z={z:.1f}')
        
        # Modifier avec hexa.yaml
        params = SimulationParameters(v_max=5, a_max=1, tolerence=5.0, nb_laps=self.nb_laps, dt=0.5, path_resolution=0.5)
        sim = LapsPathSimulator(
          self.waypoints,
          start_position=(0, 0, 10),
          end_position=(0, 0, 10),
          params=params
        )

        t_start = time.time()
        self.intent = sim.generate_simulation_intent(self.get_clock().now().to_msg())
        elapsed_time = time.time() - t_start

        self.get_logger().info(f'Simulation completed within {(elapsed_time * 1000):.2f} ms')

        self.save_intent()
    
    def save_intent(self, path=None):
      """
      Dans l'optique de faire des tests avec un autre programme,
      enregistrer les points à parcourir.
      """
      path = path or Path(__file__).resolve().parent / 'anim_info.txt'
      with open(path, 'a+', encoding='utf-8') as f:
        f.truncate(0)
        f.write(str(self.intent))

    def initialize(self):
      """
      Initialiser les paramètres de nombre de laps et la liste des waypoints
      en utilisant le fichier sitl_test.yaml (le nom du fichier va changer)
      """
      self.declare_parameter('task1.course.laps', Parameter.Type.INTEGER)
      nb_laps = self.get_parameter('task1.course.laps').value
      waypoints = self.initialize_waypoints()

      return waypoints, nb_laps
    
    def initialize_waypoints(self):
      """
      Transformer les points en format GPS du .yaml en format ENU
      pour mavros et la simulation de laps_path_sim.py
      """
      self.declare_parameter('task1.course.lat', Parameter.Type.DOUBLE_ARRAY)
      self.declare_parameter('task1.course.lon', Parameter.Type.DOUBLE_ARRAY)
      self.declare_parameter('task1.course.altitude_agl', Parameter.Type.DOUBLE)

      lat_list = self.get_parameter('task1.course.lat').value   # liste de latitudes
      lon_list = self.get_parameter('task1.course.lon').value   # liste de longitudes
      altitude_agl = self.get_parameter('task1.course.altitude_agl').value

      gps_points = [
        (lat_list[i], lon_list[i], altitude_agl)
        for i in range(len(lat_list))
      ]

      return self.gps_to_enu_path(gps_points, home=self.home)

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
        d_alt = gps_pt[ALTITUDE] - h_alt
        enu_points.append((
          dx, dy, d_alt
        ))
      
      return enu_points

def main():
    rclpy.init(); rclpy.spin(Laps()); rclpy.shutdown()


"""
Pour faire rouler, dans l'architecture où le node a été fait

make down
C=testing docker compose -f compose/sim.yml up -d mavros-sim
C=testing docker compose -f compose/dev.yml up -d dev

make build C=hexa
make shell
source /aeac/workspaces/hexa_ws/install/setup.bash
ros2 run nav_stack laps_nav --ros-args --params-file /aeac/config/sites/medicine_hat.yaml
"""
