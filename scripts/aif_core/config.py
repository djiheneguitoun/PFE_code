"""Tous les réglages de la simulation AIF (classe SimConfig), avec leurs valeurs par défaut.

12_aif_isaac_sim.py remplit une SimConfig à partir des options de la ligne de commande, puis adapte la zone
à l'entrepôt chargé. Unités : m, s ou ms (suffixe _ms) ; « pas » = pas de décision AIF.
Repère local : origine au coin de la grille (origin_x, origin_y) ; repère monde : celui d'Isaac Sim.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class SimConfig:
    """Réglages de la simulation : zone, drones, lidar, carte, poids de G, résilience, réseau, pannes, sorties."""
    env_width: float = 30.0            # m : largeur de la zone (remplacée par celle de l'entrepôt + 1 m de marge)
    env_height: float = 20.0           # m : hauteur de la zone (idem)
    grid_resolution: float = 0.5       # m : côté d'une case de la carte
    fly_altitude: float = 2.0          # m : altitude de vol

    num_drones: int = 3
    drone_spacing: float = 5.0         # m : écart entre deux drones au départ
    step_size: float = 1.0             # m : longueur d'un déplacement (une action)

    num_rays: int = 360                # directions lidar gardées (une par degré)
    lidar_fov_h: float = 360.0         # degrés : champ de vue horizontal
    lidar_fov_v: float = 40.0          # degrés : champ de vue vertical
    lidar_vert_res: float = 5.0        # degrés : écart vertical entre deux nappes du lidar
    lidar_max_range: float = 8.0       # m : portée maximale
    lidar_min_range: float = 0.15      # m : portée minimale
    lidar_hz: float = 10.0             # tours par seconde
    floor_filter_z: float = 0.25       # m : un impact plus bas est le sol, il est ignoré

    prior_occupancy: float = 0.5       # probabilité d'occupation de départ (0,5 = inconnu)
    lo_free: float = -0.55             # log-odds ajouté à une case traversée par un rayon (plus libre)
    lo_occ: float = 0.85               # log-odds ajouté à la case où le rayon s'arrête (plus occupée)
    lo_max: float = 30.0               # borne des log-odds (évite les débordements numériques)
    occ_threshold: float = 0.65        # probabilité à partir de laquelle une case est un obstacle

    w_epistemic: float = 2.5           # poids de la curiosité (gain d'information attendu) dans G
    w_pragmatic: float = 0.8           # poids de l'attrait des zones incertaines proches (« frontières »)
    w_movement: float = 0.1            # petit coût pour bouger plutôt que rester
    w_collision: float = 5.0           # poids de la pénalité de proximité aux autres drones (à moins de 8 m)
    w_clearance: float = 25.0          # poids de la pénalité de proximité aux obstacles
    clearance_cells: int = 6           # cases : rayon de cette pénalité (6 × 0,5 m = 3 m)
    softmax_temp: float = 0.3          # température du tirage : plus bas = le meilleur G est choisi plus souvent
    fusion_mix: float = 0.3            # part de la carte fusionnée dans la carte de planification (0 = carte du drone seule)

    waypoint_tol: float = 0.3          # m : distance à la cible sous laquelle le drone est « arrivé »

    alpha: int = 30                    # pas : durée de la phase "recovery" après un stress
    beta: int = 60                     # pas : nombre de pas « rétablis » d'affilée pour lever le stress
    H_target: float = 0.44             # entropie moyenne (nats) sous laquelle la carte est « rétablie »
    innov_target: float = 0.16         # surprise moyenne sous laquelle la carte est « rétablie »
    k_sigma: float = 2.0               # pic de surprise = au-dessus de moyenne + 2 écarts-types
    ema_alpha: float = 0.05            # vitesse d'adaptation de la moyenne glissante de la surprise

    w_entropy_recover: float = 3.0     # phase recovery : poids de l'entropie
    w_innov_recover: float = 1.2       # phase recovery : poids ajouté au gain d'information
    w_deadline: float = 12.0
    w_entropy_durable: float = 1.2     # phase durable : poids de l'entropie
    w_innov_durable: float = 0.8
    w_churn_durable: float = 2.2
    w_maintain: float = 10.0           # phase durable : pénalité si l'entropie dépasse H_target

    planner: str = "aif"               # choix d'action : "aif" ou "heuristic"
    arch: str = "centralized"          # architecture : "centralized" ou "distributed"
    neighbor_radius_m: float = 5.0     # m : portée d'échange des cartes entre drones (distribué)
    ns3_mode: str = "wifi"             # latences ns-3 : "wifi", "5g" ou "none"
    cloud_round_trip_ms: float = 500.0 # ms : aller-retour drone → cloud → drone (centralisé)
    switch_latency_ms: float = 2000.0  # ms : durée de la bascule après une coupure (aucune nouvelle action)
    ns3_sim_time: int = 600            # s : durée de la simulation ns-3
    physics_dt_s: float = 1.0 / 60.0   # s : durée d'un world.step (une image à 60 Hz)

    kill_drone_at_step: int = -1       # pas de la perte d'un drone (-1 = jamais)
    kill_drone_id: int = 0             # numéro du drone perdu
    cut_cloud_at_step: int = -1        # pas de la coupure du cloud (-1 = jamais)
    cut_drone_link: str = ""           # liens drone-drone coupés : "i-j" ou "all"
    cut_drone_link_at_step: int = -1   # pas de cette coupure (-1 = jamais)
    drop_obstacle_at_step: int = -1    # pas d'apparition de l'obstacle (-1 = jamais)
    drop_obstacle_xy: str = ""         # "x,y" : position de l'obstacle, en m dans le repère monde

    headless: bool = False             # vrai = sans fenêtre graphique
    max_steps: int = 500               # nombre maximal de pas de décision
    sim_steps_per_aif: int = 60        # pas physiques (world.step) entre deux décisions
    target_coverage: float = 93.0      # % : couverture qui termine la mission
    output_dir: str = "/tmp"           # dossier des fichiers mis à jour en direct (aif_state.json, journal…)
    run_tag: str = "default"           # nom du run (dossier logs/runs/run_<date>_<tag>)
    runs_dir: str = ""                 # dossier des runs ("" = logs/runs du projet)

    world_origin_x: float = float("nan")   # m monde : X du coin (0, 0) de la grille (nan = zone centrée sur 0)
    world_origin_y: float = float("nan")   # m monde : Y du coin (0, 0) de la grille
    factory_bounds_world: Optional[Tuple[float, float, float, float]] = None   # m monde : boîte (x0, y0, x1, y1) de l'entrepôt
    interior_inset_m: float = 2.5      # m : marge retirée de chaque côté de cette boîte pour obtenir l'« intérieur »

    @property
    def origin_x(self) -> float:
        """Renvoie le X monde (m) du coin (0, 0) de la grille : world_origin_x, ou -env_width / 2 s'il n'est pas fixé."""
        return -self.env_width / 2 if math.isnan(self.world_origin_x) else self.world_origin_x

    @property
    def origin_y(self) -> float:
        """Renvoie le Y monde (m) du coin (0, 0) de la grille : world_origin_y, ou -env_height / 2 s'il n'est pas fixé."""
        return -self.env_height / 2 if math.isnan(self.world_origin_y) else self.world_origin_y

    @property
    def grid_width(self) -> int:
        """Renvoie le nombre de cases en largeur (env_width / grid_resolution)."""
        return int(self.env_width / self.grid_resolution)

    @property
    def grid_height(self) -> int:
        """Renvoie le nombre de cases en hauteur (env_height / grid_resolution)."""
        return int(self.env_height / self.grid_resolution)

    @property
    def max_range_cells(self) -> int:
        """Renvoie la portée du lidar en cases (8 m / 0,5 m = 16)."""
        return int(self.lidar_max_range / self.grid_resolution)

    @property
    def step_dt_ms(self) -> float:
        """Renvoie la durée simulée d'un pas de décision en ms (60 × 1/60 s = 1000 ms) ; sert à convertir les latences en pas."""
        return self.sim_steps_per_aif * self.physics_dt_s * 1000.0

    @property
    def ray_angles(self):
        """Renvoie les num_rays angles (rad) des rayons simulés par le planificateur."""
        import numpy as np
        return np.linspace(0, 2 * math.pi, self.num_rays, endpoint=False)

    def factory_bounds_grid(self) -> Optional[Tuple[int, int, int, int]]:
        """Renvoie la boîte de l'entrepôt en indices de cases (gx0, gy0, gx1, gy1), ou None si l'entrepôt est inconnu."""
        if self.factory_bounds_world is None:
            return None
        fx0w, fy0w, fx1w, fy1w = self.factory_bounds_world
        gx0 = max(0, int((fx0w - self.origin_x) / self.grid_resolution))
        gy0 = max(0, int((fy0w - self.origin_y) / self.grid_resolution))
        gx1 = min(self.grid_width,
                  int(math.ceil((fx1w - self.origin_x) / self.grid_resolution)))
        gy1 = min(self.grid_height,
                  int(math.ceil((fy1w - self.origin_y) / self.grid_resolution)))
        return gx0, gy0, gx1, gy1

    def interior_bounds_local(self) -> Optional[Tuple[float, float, float, float]]:
        """Renvoie l'intérieur de l'entrepôt (boîte réduite de interior_inset_m par côté) en m locaux (x0, y0, x1, y1),
        ou None si l'entrepôt est inconnu (par exemple sans Isaac Sim)."""
        if self.factory_bounds_world is None:
            return None
        fx0w, fy0w, fx1w, fy1w = self.factory_bounds_world
        inset = float(self.interior_inset_m)
        x0 = (fx0w - self.origin_x) + inset
        y0 = (fy0w - self.origin_y) + inset
        x1 = (fx1w - self.origin_x) - inset
        y1 = (fy1w - self.origin_y) - inset
        return x0, y0, x1, y1

    def interior_area_cells(self) -> int:
        """Renvoie le nombre de cases de l'intérieur de l'entrepôt (toute la grille s'il est inconnu) : dénominateur de la couverture."""
        if self.factory_bounds_world is None:
            return self.grid_width * self.grid_height
        fx0w, fy0w, fx1w, fy1w = self.factory_bounds_world
        inset = float(self.interior_inset_m)
        interior_w = max(0.0, (fx1w - fx0w) - 2 * inset)
        interior_h = max(0.0, (fy1w - fy0w) - 2 * inset)
        cells = (interior_w * interior_h) / (self.grid_resolution ** 2)
        return max(1, int(round(cells)))
