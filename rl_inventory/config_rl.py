"""Tous les réglages de l'essai RL (LiDAR, action, récompense, lecture QR, scène, entraînement), en Python pur.

Importé partout via l'instance CFG (bas du fichier) ; se lit sans Isaac Sim.
Les renvois « doc §… » visent le document de conception CONCEPTION_controleur_RL_inventaire.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class LidarConfig:
    """Réglages du LiDAR (capteur laser qui mesure les distances) : 360 directions × 5 bandes de hauteur, portée 8 m (doc §3.1)."""

    num_azimuth: int = 360          # colonnes (1 par degré, tour complet)
    num_channels: int = 5           # bandes de hauteur
    horizontal_fov_deg: tuple[float, float] = (0.0, 360.0)  # degrés : tour complet
    vertical_fov_deg: tuple[float, float] = (-15.0, 15.0)  # degrés : couvre les hauteurs d'étagères
    horizontal_res_deg: float = 1.0  # degrés : 360 / 1.0 = 360 colonnes
    max_distance_m: float = 8.0      # m : portée du LiDAR (même valeur que l'approche AIF)
    offset_z_m: float = 0.0          # m : capteur centré sur le corps du drone
    ray_alignment: str = "yaw"       # les rayons suivent le cap du drone et restent à plat
    # bruit du capteur (réglable), désactivé par défaut
    noise_std_m: float = 0.0         # m : écart-type du bruit gaussien ajouté aux distances
    dropout_prob: float = 0.0        # probabilité qu'un rayon ne renvoie rien (absorption, reflet)

    @property
    def num_rays(self) -> int:
        """Renvoie le nombre de rayons par LiDAR (360 × 5 = 1800)."""
        return self.num_azimuth * self.num_channels


@dataclass
class ActionConfig:
    """Réglages de l'action : vitesse visée (vx, vy, vz, lacet) en repère drone, bornée, et modèle d'actionneur (doc §3.2)."""

    # La vitesse réelle rejoint la consigne via actuator.py (retard + accélération bornée).
    # Sources : firmware Bitcraze du Crazyflie (platform_defaults_cf2.h), Preiss et al. (Crazyswarm,
    # ICRA 2017), Eschmann et al. (arXiv:2404.07837).
    max_lin_vel_mps: float = 1.0     # m/s : PID_POS_VEL_X/Y_MAX du firmware (1,5 avant : non tenable)
    max_vz_up_mps: float = 1.0       # m/s : PID_POS_VEL_Z_MAX (valeur unique du firmware Bitcraze)
    max_vz_down_mps: float = 1.0     # m/s, SYMÉTRIQUE de la montée. Bridée à 0,5, la descente rendait
                                     # la commande verticale asymétrique : un bruit de moyenne nulle
                                     # donnait +0,060 m/s, soit +9 m par épisode ; les drones restaient
                                     # au plafond, d'où 36 des 90 QR invisibles (mesuré)
    max_yaw_rate_rps: float = 1.5    # rad/s (86 °/s) ; entre MotionCommander (72) et cfclient (200), outils Crazyflie
    altitude_min_m: float = 0.3      # m : plancher (le drone monte et descend pour scanner les étagères)
    altitude_max_m: float = 3.0      # m : plafond

    # modèle d'actionneur : sans lui, 0 → 1,5 m/s en un pas = 4,59 g (Crazyflie : 0,36 g)
    accel_xy_mps2: float = 3.6       # m/s² : g·tan(20°), inclinaison max 20° (PID_VEL_ROLL_MAX = 20.0f)
    accel_z_mps2: float = 3.0        # m/s² (vertical)
    yaw_accel_rps2: float = 10.0     # rad/s² : choix d'ingénierie, aucune source publiée
    tau_xy_s: float = 0.30           # s : ≈ v_max / a_max ; passer de 1,0 à 0,6 m/s prend ~5 pas de contrôle
    tau_z_s: float = 0.30            # s
    tau_yaw_s: float = 0.15          # s

    @property
    def dim(self) -> int:
        """Renvoie la taille de l'action : 4 (vx, vy, vz, vitesse de lacet)."""
        return 4


@dataclass
class RewardConfig:
    """Poids de la récompense de chaque drone (doc §3.4) — valeurs à calibrer."""

    new_qr: float = 10.0             # points quand un QR encore non lu devient lu
    mission_complete: float = 100.0  # points quand tous les QR sont lus
    collision: float = -5.0          # points PAR PAS tant qu'en collision (LiDAR mini < seuil) ; pas de fin d'épisode
    time_step: float = -0.1          # points par pas (pousse à finir vite)
    jerk_lambda: float = 0.01        # poids λ de la pénalité −λ·‖Δaction‖² (vol fluide)
    approach_scale: float = 1.0      # points par mètre gagné vers le QR non lu le plus proche (arrêt à la distance de lecture)
    posture_scale: float = 0.5       # points par pas quand le drone est BIEN PLACÉ pour lire (proche + face + LENT) un QR non lu
    proximity_penalty: float = 0.0   # (option) rester à distance de sécurité
    safe_distance_m: float = 0.5     # m : seuil de proximité d'obstacle (sert au facteur de risque de eval.py)
    collision_distance_m: float = 0.25  # m : distance LiDAR minimale en dessous de laquelle on compte une collision


@dataclass
class QRProxyConfig:
    """Règle géométrique « QR lu », utilisée à l'entraînement à la place d'un vrai décodage d'image (doc §4)."""

    # Un QR compte comme lu s'il est dans le champ de la caméra, assez proche, vu sous un bon angle, avec
    # une ligne de vue dégagée. Le vrai décodeur pyzbar n'est utilisé qu'à l'évaluation.
    n_tags: int = 12                 # nombre de QR (taille de mission, doc §11)
    max_read_distance_m: float = 3.0  # m : distance max de lecture
    max_view_angle_deg: float = 35.0  # degrés : angle max par rapport à la normale du QR
    camera_fov_deg: float = 60.0      # degrés : DEMI-angle du cône de la caméra (cône total 120°)
    min_dwell_steps: int = 2          # durée de visée minimale, en pas de contrôle consécutifs (2 pas ≈ 67 ms)
    max_read_speed_mps: float = 0.6   # m/s : vitesse horizontale max pour lire (sinon flou) — à calibrer sur pyzbar
    max_read_yawrate_rps: float = 0.8  # rad/s : vitesse de lacet max pour lire
    check_line_of_sight: bool = True  # lancer de rayon drone → QR contre la géométrie (raffinement ultérieur)
    coverage_target: float = 1.0      # fraction de QR lus qui termine la mission (1.0 = tous)


@dataclass
class SceneConfig:
    """Réglages de la scène : fichier de l'entrepôt (USD, format de scène 3D d'Isaac Sim) et écart entre ses copies (doc §5)."""

    # Espace intérieur rectangulaire, rangées d'étagères formant des allées, QR sur les faces à plusieurs hauteurs.
    # Décision (le LiDAR d'Isaac demande un maillage statique) : étagères et murs FIXES (vus par le LiDAR),
    # inventaire (cartons, QR) tiré au hasard à chaque épisode (la règle « QR lu » s'appuie sur la position
    # des cartons, pas sur le LiDAR).

    # entrepôt Isaac 4.2 lu sur le serveur public de NVIDIA (accès Internet requis) ; remplaçable par AIF_FACTORY_USD
    warehouse_usd: str = (
        "http://omniverse-content-production.s3-us-west-2.amazonaws.com/"
        "Assets/Isaac/4.2/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    )
    env_size_xy_m: tuple[float, float] = (30.0, 20.0)  # m : taille de l'entrepôt (x, y)
    env_spacing_m: float = 40.0      # m : écart entre copies d'entrepôt (> taille de l'entrepôt : copies isolées)
    num_racks: int = 4               # nombre de rangées d'étagères
    randomize_inventory: bool = True  # positions des cartons/QR variables à chaque épisode
    randomize_racks: bool = False     # True => reconstruire le maillage à chaque reset (coûteux)


@dataclass
class TrainConfig:
    """Réglages de la boucle de simulation : entrepôts en parallèle, pas de temps, durée d'épisode, taille de l'essaim (doc §9)."""

    num_envs: int = 16               # entrepôts simulés en parallèle (à mesurer sur la carte graphique de 8 Go)
    decimation: int = 4              # pas physiques par pas de contrôle
    physics_dt: float = 1.0 / 120.0  # s : physique à 120 Hz → contrôle à 30 Hz
    episode_length_s: float = 45.0   # s : budget pour inspecter l'entrepôt (1350 pas de contrôle)
    num_drones: int = 3              # essaim : 3 drones par entrepôt
    seed: int = 42                   # graine aléatoire

    @property
    def control_dt(self) -> float:
        """Renvoie la durée d'un pas de contrôle en s : 4 × 1/120 = 1/30 s (≈ 33 ms)."""
        return self.physics_dt * self.decimation


@dataclass
class RLConfig:
    """Réglage racine : regroupe les six blocs ci-dessus (importé partout sous le nom CFG)."""

    lidar: LidarConfig = field(default_factory=LidarConfig)
    action: ActionConfig = field(default_factory=ActionConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    qr: QRProxyConfig = field(default_factory=QRProxyConfig)
    scene: SceneConfig = field(default_factory=SceneConfig)
    train: TrainConfig = field(default_factory=TrainConfig)


# Instance par défaut, importée partout.
CFG = RLConfig()
