from custom_interfaces.msg import NavGate, NavPoint, NavIntent, NavMode
import math

class LapsPathSimulator():
  """
  Simulation entièrement numérique et séparée de Mavros.

  On donne la position, la vitesse et l'accélération du drone et
  on simule son déplacement entre les waypoints en modifiant à
  chaque instant 'dt' :
   - Ajuster la vitesse (selon l'objectif courant et a_max)
   - Ajuste la position (en utilisant la vitesse sur dt)
  
  Des fonctions plus avancées sont également utilisées pour optimiser
  le temps et la vitesse de trajet :
   - get_next_objective_impact_radius()
    - Détermine une distance à laquelle le drone va tourner
      vers l'objectif suivant pour ne pas faire de 'overshoot',
      ce qui ralentirait le trajet.
  
  On stoque ensuite les positions et les waypoints dans un NavIntent.

  La simulation est démarrée dans le laps_nav.py
  """
  def __init__(self, waypoints, start_position=(0, 0, 0), end_position=None, params=None):
    self.waypoints = waypoints
    self.start_position = start_position
    self.end_position = end_position
    self.params = params or SimulationParameters(v_max=5, a_max=3)

    self.lap_index = 0
    self.waypoint_index = -1
  
  def get_next_objective(self, waypoint_index=None, lap_index=None):
    """
    Si on est au départ, on veut aller vers le premier waypoint.
    Si on est à un waypoint, on veut aller au suivant.
     - Si le suivant est la fin du parcours, on loop au premier waypoint
       s'il reste des tours à faire
     - Sinon on va à la position de fin
    """

    waypoint_index = waypoint_index or self.waypoint_index
    lap_index = lap_index or self.lap_index

    if waypoint_index == len(self.waypoints) - 1:
        if lap_index == self.params.nb_laps - 1:
            if self.end_position is None:
                return None, waypoint_index, lap_index
            return self.end_position, -1, lap_index
        lap_index += 1
        return self.waypoints[0], 0, lap_index

    if waypoint_index == -1 and lap_index == 0:
        return self.waypoints[0], 0, lap_index

    if waypoint_index != -1:
        waypoint_index += 1
        return self.waypoints[waypoint_index], waypoint_index, lap_index

    # On a tout complété
    return None, waypoint_index, lap_index

  def get_new_speed(self, current_speed, target_speed):
    """
    Appliquer au maximum une accélération de a_max sur le temps
    dt pour modifier la vitesse pour match celle qu'on voudrait avoir
    (par exemple pour tourner vers un nouvel objectif).
    """
    # self.params.a_max, self.params.dt
    delta = tuple(t - c for t, c in zip(target_speed, current_speed))
    dist = self.norm(delta)
    dt = self.params.dt

    max_dv = self.params.a_max * dt

    # Cible atteignable en un pas : on l'atteint exactement
    if dist <= max_dv:
        new_speed = tuple(target_speed)
    else:
        # Sinon, on avance de max_dv en ligne droite vers la cible
        direction = self.normalize(delta)
        new_speed = tuple(c + d * max_dv for c, d in zip(current_speed, direction))

    # Accélération réellement appliquée (vecteur)
    accel = tuple((n - c) / dt for n, c in zip(new_speed, current_speed))

    return new_speed, accel

  def get_objective_direction(self, position, objective):
    """
    Retourner simplement le vecteur unitaire de la direction
    de l'objectif à atteindre.
    """
    # Vecteur de la position vers l'objectif (x, y, z)
    delta = tuple(o - p for o, p in zip(objective, position))

    # Déjà sur l'objectif : pas de direction définie
    if self.norm(delta) == 0.0:
        return (0.0, 0.0, 0.0)

    # Vecteur unitaire pointant vers l'objectif
    return self.normalize(delta)
  
  def get_distance(self, a, b):
    """
    Distance entre deux points
    """
    return self.norm(tuple(x - y for x, y in zip(a, b)))

  def get_next_objective_impact_radius(self, speed, position, objective, next_objective,
                                     tolerance, confidence=0.3, max_steps=500):
    """
    Coeur de l'optimisation :
    On veut tourner un peu avant d'avoir atteint un waypoint pour aller
    plus vite vers le suivant.

    Cependant ! Il faut rester dans le rayon 'tolerance' autour de l'objectif
    courant, sinon le drone doit repartir en arrière pour le sécuriser.
     - On simule une seule fois le virage vers l'objectif suivant (même règle
       que get_new_direction : vitesse constante, accélération limitée).
     - On regarde jusqu'où la trajectoire reste dans la tolérance, et on en
       déduit la distance max à laquelle on peut commencer à tourner.

    On retourne ce rayon : quand le drone passe en dessous, on vise
    l'objectif suivant. C'est toujours >= tolerance (= pas de virage à
    anticiper).

    speed, position, objective, next_objective : tuples (x, y, z)
    confidence dans [0, 0.95] : 0 = coupe le coin au maximum (bord de la
    tolérance), plus haut = marge de sécurité.
    """
    
    v = self.norm(speed)
    dt = self.params.dt
    max_dv = self.params.a_max * dt

    # Pas de virage à anticiper : on est arrivé dès qu'on est dans la tolérance
    if next_objective is None or v < 1e-9:
        return tolerance

    d_in = tuple(o - p for o, p in zip(objective, position))
    d_out = tuple(n - o for n, o in zip(next_objective, objective))
    if self.norm(d_in) == 0.0 or self.norm(d_out) == 0.0:
        return tolerance
    u = self.normalize(d_in)   # direction d'arrivée
    w = self.normalize(d_out)  # direction vers l'objectif suivant

    tol = tolerance * (1.0 - max(0.0, min(0.95, confidence)))

    vel = tuple(v * c for c in u)
    target = tuple(v * c for c in w)
    pos = (0.0, 0.0, 0.0)
    best = tol  # virage instantané : on passe à ~0 de l'objectif

    # Faire une petite simulation du tournant.
    for _ in range(max_steps):
        # Même règle que get_new_direction
        delta = tuple(t - c for t, c in zip(target, vel))
        dist = self.norm(delta)
        if dist <= max_dv:
            vel = target
        else:
            vel = tuple(c + d / dist * max_dv for c, d in zip(vel, delta))
        pos = tuple(p + c * dt for p, c in zip(pos, vel))  # position avec la nouvelle vitesse

        x = sum(p * a for p, a in zip(pos, u))                  # le long de l'arrivée
        y = math.sqrt(max(0.0, sum(p * p for p in pos) - x * x))  # écart latéral
        if y > tol:
            break  # on est sorti de la tolérance, les points suivants ne comptent plus

        best = max(best, x + math.sqrt(tol * tol - y * y))

        if sum(c * a for c, a in zip(vel, u)) <= 0.0:
            break  # le drone ne progresse plus vers l'objectif (épingle)

    # +v*dt/2 : le test de distance n'a lieu qu'à chaque pas (retard moyen d'un demi-pas)
    return max(tolerance, best + v * dt / 2.0)
  
  def generate_simulation_intent(self, stamp):
    """
    Simulation

    On commence à la position, la vitesse et l'accélération donnée,
    et l'on simule le déplacement du drone.
    
    On veut passer dans le rayon 'tolérence' de chaque objectif et alors
    on commence par viser le premier objectif, puis le suivant,
    et ainsi de suite, en faisant le bon nombre de tours.

    Lorsqu'on s'approche d'un premier objectif, on teste un peu avant s'il est
    possible de s'orienter directement vers l'objectif suivant tout en
    passant dans le rayon de 'tolérence' du waypoint.
     - Grâce à get_next_objective_impact_radius(...)
    
    Lorsqu'on a fini les laps et qu'on est arrivé à la fin, on retourne l'instance
    de NevIntent qui est remplie des NavPoints à chaque instant de la simulation
    en stoquant :
     - Position (x y et z)
     - Vitesse (x y et z)
     - Accélération (x et y)
    
    On stoque aussi des NavGates qui sont les passages obligatoires.
     - Index de passage
     - Distance curviligne depuis le début
     - Position (x et y)
     - Tolérence
    
    Pour réduire la taille de la liste de NavPoints dans
    NavIntent, on enlève des points s'ils sont tous alignés.
     - Par contre, garder les points s'ils permettent de passer dans
       les rayons 'tolérence' autour des objectifs
    """

    intent = NavIntent()
    intent.header.frame_id = 'map'
    intent.header.stamp = stamp
    intent.mode = NavMode.LAPS
    intent.intent_id = 1          # nouveau chemin = nouvel id
    intent.vertical_freedom_m = 0.0

    drone = DroneStats(
      self.start_position,
      speed=(0, 0, 0),
      acceleration=(0, 0, 0)
    )

    # Point initial
    self.add_point_to_intent(
      intent,
      drone.position,
      drone.speed,
      drone.acceleration[0:2],
      s=0)

    objective, self.waypoint_index, self.lap_index = self.get_next_objective()
    curve_distance = 0.0
    gate_index = 0
    t = 0
    prev_is_gate = False
    while objective is not None:
      # Voir si on doit anticiper un virage
      next_objective, _, _ = self.get_next_objective()

      # Mesurer le rayon auquel on devrait commencer à tourner vers le point suivant
      next_objective_impact_radius = self.get_next_objective_impact_radius(
        drone.speed, drone.position,
        objective, next_objective,
        self.params.tolerence,
        confidence=0.3
      )

      if next_objective is not None and self.get_distance(drone.position, objective) < next_objective_impact_radius:
        # Viser l'objectif d'après, même avant d'avoir atteint l'objectif présent
        direction = self.get_objective_direction(drone.position, next_objective)
      else:
        direction = self.get_objective_direction(drone.position, objective)
      
      target_speed = tuple(d * self.params.v_max for d in direction)

      # Tourner vers la bonne direction en allant à la vitesse target (+ vite possible)
      drone.speed, drone.acceleration = self.get_new_speed(drone.speed, target_speed)

      drone_movement = tuple(drone.speed[i] * self.params.dt for i in range(len(drone.speed)))
      curve_distance += self.norm(drone_movement)

      drone.position = tuple(drone.position[i] + drone_movement[i] for i in range(len(drone.position)))

      # Ajouter le point avec sa position, vitesse, accélération dans le NavIntent
      self.add_point_to_intent(
        intent,
        drone.position,
        drone.speed,
        drone.acceleration[0:2],
        s=curve_distance)

      # Simplifier : le point d'avant est enlevé s'il est en ligne (sauf si c'était une gate)
      self.simplify_last_point(intent, protect_middle=prev_is_gate)
      prev_is_gate = False

      if self.get_distance(drone.position, objective) <= self.params.tolerence:
        # Passage validé : la gate prend la vraie distance curviligne
        gate = NavGate(
          index=gate_index,
          s_apex=float(curve_distance),
          x=float(objective[0]), y=float(objective[1]),
          tolerance_m=float(self.params.tolerence))
        intent.gates.append(gate)
        gate_index += 1
        prev_is_gate = True   # ce point-là ne sera jamais supprimé
        objective, self.waypoint_index, self.lap_index = self.get_next_objective()

        if objective is None:
          # On a atteint tous les objectifs !!!
          break

      if t > 1000:
        # IMPORTANT : modifier cette ligne (ou l'enlever) si les parcours générés sont plus longs
        # Si le code est bloqué, quitter même sans avoir fini
        break

      t += self.params.dt

    return intent
  
  def add_point_to_intent(self, intent, pos, speed, acc, s):
    """
    À partir d'un point avec sa position, vitesse et accélération,
    on crée une instance de NavPoint.
    Note:
     - s est la distance curviligne jusqu'au point
    """
    p = NavPoint()
    p.x, p.y, p.z = map(float, pos)       # (x, y, z) en ENU
    p.vx, p.vy, p.vz = map(float, speed)
    p.ax, p.ay = map(float, acc)
    p.s = float(s)
    intent.points.append(p)
  
  def turn_angle(self, a, b, c):
    """
    Angle (rad) de changement de direction en b, entre a->b et b->c.
    """
    ab = tuple(bi - ai for ai, bi in zip(a, b))
    bc = tuple(ci - bi for bi, ci in zip(b, c))
    n1, n2 = self.norm(ab), self.norm(bc)
    if n1 == 0.0 or n2 == 0.0:
        return 0.0  # point immobile : rien à garder
    cos = sum(x * y for x, y in zip(ab, bc)) / (n1 * n2)
    return math.acos(max(-1.0, min(1.0, cos)))

  def simplify_last_point(self, intent, protect_middle):
    """
    Enlève l'avant-dernier point s'il est (presque) aligné avec ses voisins.
    """
    if protect_middle or len(intent.points) < 3:
        return
    if self.params.path_resolution >= 1.0:
        return  # résolution max : on garde tout

    # Virage max possible en un pas (rad), proportionnel à dt
    max_turn_per_step = self.params.a_max / self.params.v_max * self.params.dt
    max_angle = (1.0 - self.params.path_resolution) * max_turn_per_step

    a, b, c = intent.points[-3:]
    if self.turn_angle((a.x, a.y, a.z), (b.x, b.y, b.z), (c.x, c.y, c.z)) <= max_angle:
        del intent.points[-2]
  
  def normalize(self, vec):
    """
    Normaliser un vecteur à N dimensions :)
    """
    norm = self.norm(vec)
    if norm == 0: return vec
    new_vec = tuple(c / norm for c in vec)
    return new_vec
  
  def norm(self, vec):
    """
    Jolie petite fonction pour calculer la norme...
    d'un vecteur à N dimensions !
    """
    return sum([c ** 2 for c in vec]) ** 0.5


class SimulationParameters():
  """
  Paramètres de simulation :
   - v_max : Vitesse maximale du drone
   - a_max : Accélération maximale du drone
   - tolerence : Rayon de tolérance
   - dt : pas de temps entre chaque itération de la simulation
   - nb_laps : nombre de tours à compléter
   - path_resolution : à quel point on conserve des points qui sont alignés (réduire la taille de NavIntent)
  """
  def __init__(self, v_max, a_max, tolerence=3, dt=0.1, nb_laps=1, path_resolution=1):
    self.v_max = v_max
    self.a_max = a_max
    self.tolerence = tolerence  # Meters from targets
    self.dt = dt
    self.nb_laps = nb_laps
    self.path_resolution = path_resolution  # dans [0, 1] : 0 = on réduit un max, 1 = on garde tous les points


class DroneStats():
  """
  Paramètres de vol du drone
   - position (x, y, z)
   - vitesse (x, y, z)
   - accélération (x, y, z)
  """
  def __init__(self, position, speed, acceleration):
    self.position = position
    self.speed = speed
    self.acceleration = acceleration
