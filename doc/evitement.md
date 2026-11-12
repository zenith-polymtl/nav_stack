# Évitement du trafic dans guidance (Task 1)

Comment `guidance` évite les avions simulés par le serveur de compétition pendant les tours, et pourquoi
chaque choix. Code : `nav_stack/avoidance.py`, `timing.py`, `local_planner.py`, `traffic.py`,
`path_follower.py`.

## Ce que fait guidance, dans l'ordre

1. **Prédire le trafic** (`traffic.py`). comp_client reçoit chaque avion du serveur à 1 Hz. Le trafic du
   serveur est déterministe et tourne en boucle (transits sur une polyligne, attentes en cercle) : chaque
   avion garde un journal de ses positions, et dès que ses dernières positions se répètent une période plus
   tôt, sa boucle est apprise et rejouée (prédiction exacte, virages compris). Avant cela, prédiction à
   vitesse et virage constants. En compétition, le décollage suit la connexion au serveur de 30 s au plus :
   les boucles s'apprennent surtout en vol.
2. **Suivre le chemin tant que rien ne menace** (`avoidance.py`). L'évitement ne s'engage que si le
   chemin devant (ou ce que le drone vole) passe à moins de `ENGAGE_MARGIN_M` (15 m) du cylindre interdit
   d'un avion. Il se relâche avec 10 m d'hystérésis, et seulement quand le drone est revenu à moins de
   `RETURN_M` (10 m) du chemin. Planifier dès qu'un avion approche faisait couper les virages et manquer
   les WP.
3. **D'abord le temps** (`timing.py`). Engagé et près du chemin, le drone reste sur le chemin et choisit
   seulement sa vitesse : 29 profils candidats (une fraction de la vitesse du chemin, ou un arrêt, tenu
   2 à 16 s, puis la vitesse du chemin), chacun valable s'il reste à 12 m au moins des cylindres pendant la
   tenue et les 10 s de passage qui suivent (sinon, attendre ne ferait que repousser le conflit). Le plus
   avancé gagne. Environ 8 ms.
4. **Sinon, contourner** (`local_planner.py`). MPC convexe sur 10 s (21 pas de 0,5 s) : point matériel,
   accélération bornée, suivi de la référence, un demi-plan tangent au cercle prédit de chaque avion à
   chaque pas (marge dure 3 m, molle 15 m, en coût), limite de vol, et la **porte du WP** imposée au pas où
   le chemin nominal passe au plus près du WP. Résolu par Clarabel en ~5 ms, 4 fois par seconde. Si le plan
   ne peut pas passer le WP, il échoue et le drone reprend le chemin : un WP manqué coûte plus qu'une
   incursion.
5. **Ne jamais rester bloqué**. Si la progression n'avance pas de 5 m en 25 s pendant un évitement, le
   drone suit le chemin pendant 10 s, quitte à frôler un avion (5 points) plutôt que de ne jamais finir.

## Résultats (banc SITL)

ArduCopter SITL 4.7.1 (hexa), trafic exact du serveur (même modèle que le faux serveur de sim_mocks),
lots de 96 à 120 rencontres aléatoires calées pour croiser le drone, 2 passes par variante, 30 s de trafic
avant le décollage. Score : 5 points par incursion (5 au plus), 22 par WP passé à plus de 5 m (hypothèse :
la CONOPS ne donne pas de tolérance de WP).

| Lot | avant (task1/guidance) | après | |
|---|---|---|---|
| keep-away du serveur (30 et 80 m), lot de mise au point | 954 | 162 | −83 % |
| keep-away du serveur, rencontres jamais vues | 1 147 | 238 | −79 % |
| keep-away vertical 15 m, rencontres jamais vues | 782 | 232 | −70 % |
| références à 20 m/s | 1 148 | 181 | −84 % |

Aucun vol non fini sur 1 008. Coût : +4 à +9 s de chrono en médiane (surtout la fin de la survitesse en
évitement, `NU_MAX_FACTOR` 1,0 au lieu de 1,15, et les attentes). Validé aussi dans la vraie node par
mavros (SITL temps réel, faux serveur, comp_client) sur 8 scénarios : mêmes résultats que le banc, à
quelques dixièmes de seconde près.

## Essayé, sans gain mesuré (retiré)

Porte du WP molle seule (pire : le plan ne rate plus mais coupe les virages), évitement vertical (sans gain
avec les keep-away verticaux du serveur, 30 et 80 m), plan de temps global sur 60 s, plan latéral qui
essaie les deux côtés de l'avion le plus proche, réglages de coût et de marges.

## À valider

- Sur l'hexa réel : dynamique, vitesse max (15 m/s aujourd'hui, 20 visés), latence du calcul.
- Règle de passage des WP (2D ou 3D, tolérance) : à demander à l'organisation.
- Heure de la télémétrie de comp_client : en SITL, quand le simulateur attend la connexion de mavros au
  démarrage (port par défaut), l'heure GPS de l'autopilote retarde de ~4 s et le serveur compte des
  incursions fantômes (6 sur un vol qui n'en a aucune). Lancer le SITL avec `--serial0 tcp:0` (sans
  attente) ramène le retard à ~0,5 s et le serveur à 0 incursion. Sur le Pixhawk réel (heure GNSS),
  vérifier au sol que unixTime égale l'heure réelle à 0,1 s près : 0,5 s de retard, c'est ~7 m d'erreur
  de jugement à 15 m/s.
