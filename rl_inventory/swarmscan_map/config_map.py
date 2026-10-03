"""Tous les réglages de SwarmScan-Map : lecture des QR, curriculum, cartes, récompense, entraînement.

Pur Python (sans Isaac). Les autres fichiers importent l'instance unique `MAP_CFG` (en bas).
Conception : docs/conception_solution_finale.md.
Chaque commentaire garde la raison de la valeur ; l'historique complet des corrections est dans
A_LIRE_POUR_LE_PROMOTEUR/RESULTATS.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class GateConfig:
    """Règle de lecture d'un QR (le « gate ») : seuils nominaux mesurés et plages assouplies du curriculum."""

    # Seuils NOMINAUX mesurés par calibrate_gate.py le 2026-07-09 : décodeur OpenCV
    # (QRCodeDetectorAruco, image agrandie ×3, famille du décodeur de Pore et al.), caméra
    # 1280×960 de 60° de champ horizontal. Décodage jusqu'à 1,5 m et 55° : on garde une marge.
    # DEUX caméras LATÉRALES (à ±90° du cap, mêmes réglages) : le drone lit en longeant les racks.
    # Choix du 2026-07-24 : en caméra frontale, la visée fine ne marchait que grâce au bruit
    # d'exploration (lecture 0,91 avec bruit → 0,11 sans) ; en latéral, la géométrie permet
    # de tout lire sur les vraies configurations (1,00).
    read_distance_m: float = 1.25         # m : distance max caméra → QR (mesuré : 1,5 m)
    view_angle_deg: float = 50.0          # degrés : angle max avec la normale du QR (mesuré : 55°)
    camera_fov_deg: float = 30.0          # degrés : DEMI-angle du cône de chaque caméra (champ 60°)
    max_speed_mps: float = 0.6            # m/s : au-delà l'image est floue (seuil déclaré, loi de Cristiani 2020)
    max_yawrate_rps: float = 0.8          # rad/s : vitesse de rotation max pendant la lecture
    dwell_steps: int = 2                  # pas de 33 ms : le QR doit être vu 2 images de suite

    # Plages du curriculum ADR (randomisation automatique du domaine : la règle est tolérante
    # au cran 0 puis se resserre cran par cran) : (valeur au cran 0, valeur nominale).
    adr_read_distance_m: tuple[float, float] = (4.0, 1.25)   # m
    adr_view_angle_deg: tuple[float, float] = (60.0, 50.0)   # degrés
    # Au cran 0, la vitesse ne doit PAS gêner la lecture : on apprend une difficulté à la fois.
    # Avec (0,75 ; 0,60), seules 22 % des actions passaient déjà la règle de vitesse au cran 0
    # (11 % au nominal) et le curriculum n'a jamais avancé en 34 entraînements.
    # 1,45 m/s dépasse la vitesse 3D maximale du drone (√(1²+1²) = 1,414 m/s) : aucun blocage.
    adr_max_speed_mps: tuple[float, float] = (1.45, 0.60)    # m/s
    adr_max_yawrate_rps: tuple[float, float] = (1.6, 0.8)    # rad/s
    # Le temps de visée (2 images) est une propriété de la caméra, jamais un cran du curriculum :
    # 1 seul pas (33 ms) serait physiquement irréaliste.
    adr_dwell_steps: tuple[int, int] = (2, 2)                # pas
    # Vitesse comptée en 3D (montée comprise) : avec des caméras latérales, monter floute autant
    # qu'avancer. Sans cela, un drone qui montait à 1,5 m/s était vu « immobile » par la règle.
    speed_uses_vz: bool = True
    # Aucune lecture comptée pendant les 15 premiers pas (0,5 s) : au départ la vitesse est nulle,
    # et un drone placé exprès près d'un QR le lisait gratuitement.
    credit_blackout_steps: int = 15       # pas


@dataclass
class CurriculumConfig:
    """Réglages du curriculum : quand resserrer ou relâcher la règle de lecture, départs aidés et pannes."""

    notches: int = 10                     # crans entre la règle tolérante (0) et la nominale (10) ; crans
                                          # fins car l'axe vitesse va de 1,45 à 0,60 m/s
    # Barre de PROMOTION (« ce cran est acquis, passe au suivant »), à ne pas confondre avec le
    # critère de réussite final (0,85 au gate nominal, mesuré par eval_map.py).
    # Règle : moyenne glissante (EMA) du taux de lecture > success_hi + promo_margin = 0,88,
    # tenue 25 épisodes de suite.
    # Historique : une simulation Monte-Carlo sur les taux mesurés (moyenne 0,39) donnait 0,00 %
    # de chances de promotion à 0,88, et encore 0,00 % avec une moyenne de 0,75 : la barre a été
    # baissée. Elle a été remise à 0,88, identique à tous les crans, par choix délibéré le
    # 2026-07-27 : un modèle promu à 0,62 ne lisait plus que 0,551 remesuré au cran 0, et sa
    # courbe montait encore (0,28 → 0,60, sans plateau) : on ne le laissait pas finir d'apprendre.
    success_hi: float = 0.85              # barre de base (taux de lecture, entre 0 et 1)
    hi_decay_per_notch: float = 0.0       # baisse de la barre à chaque cran (0 : barre plate)
    hi_min: float = 0.85                  # plancher de la barre
    lo_gap: float = 0.20                  # écart sous la barre qui fait reculer d'un cran ; plancher de recul = 0,40
    lo_min: float = 0.35                  # plancher absolu du seuil de recul
    promo_margin: float = 0.03            # promotion si EMA > 0,88 TENUE 25 épisodes (anti-chance)
    confirm_episodes: int = 25            # épisodes de suite au-dessus de la barre
    ema_alpha: float = 0.05               # poids d'un nouvel épisode dans la moyenne glissante
    min_episodes_per_notch: int = 120     # patience minimale avant de monter
    min_episodes_down: int = 250          # délai minimal avant tout recul (sinon allers-retours)
    spawn_near_prob: tuple[float, float] = (0.7, 0.3)   # proba de naître près d'un QR non lu (cran 0 → nominal) ; plancher 0,3 : toujours des occasions de lire
    dropout_prob: tuple[float, float] = (0.0, 0.30)     # proba de panne d'un drone par épisode : monte seulement APRÈS le gate nominal
    dropout_after_nominal: bool = True
    episode_s_per_notch: float = 15.0     # s ajoutées par cran : 150 s au cran 0 → 295 s (plafond) au nominal
    episode_s_max: float = 295.0          # s : plafond ; plan optimal au nominal = 207 s avec l'inertie


@dataclass
class MapConfig:
    """Cartes construites en vol (mémoire commune de l'essaim) et vues centrées sur chaque drone."""

    bounds_x_m: tuple[float, float] = (-18.0, 18.0)   # m : étendue de la grille en x
    bounds_y_m: tuple[float, float] = (-13.0, 13.0)   # m : étendue de la grille en y
    cell_m: float = 0.25                  # m : côté d'une case de la grille globale
    crop_px: int = 32                     # cases de côté de chaque vue donnée au réseau
    crop_spans_m: tuple[float, ...] = (8.0, 32.0)   # m : largeur de chaque vue (proche, large)
    height_bands_m: tuple[float, ...] = (0.0, 1.5, 3.0, 4.7)  # m : limites des 3 tranches de hauteur des façades
    explored_radius_m: float = 3.0        # m : rayon marqué « exploré » autour de chaque drone
    traj_decay: float = 0.98              # effacement de la trace des drones à chaque pas (×0,98)
    scan_range_m: float = 2.5             # m : portée de l'empreinte de couverture, FIXE (indépendante de la règle de lecture)
    # Ces deux seuils ne doivent jamais bloquer la carte de couverture. À 1,0 m/s, ils la
    # coupaient 47,7 % du temps (la vitesse 3D max est √(1²+1²) = 1,414 m/s) : la mémoire
    # spatiale du drone restait vide une action sur deux.
    scan_speed_mps: float = 1.5           # m/s
    scan_yawrate_rps: float = 1.6         # rad/s

    @property
    def n_bands(self) -> int:
        """Renvoie le nombre de tranches de hauteur (3)."""
        return len(self.height_bands_m) - 1

    @property
    def grid_wh(self) -> tuple[int, int]:
        """Renvoie la taille de la grille globale en cases (largeur, hauteur) = (144, 104)."""
        w = int(round((self.bounds_x_m[1] - self.bounds_x_m[0]) / self.cell_m))
        h = int(round((self.bounds_y_m[1] - self.bounds_y_m[0]) / self.cell_m))
        return w, h

    @property
    def n_channels(self) -> int:
        """Renvoie le nombre de couches d'une vue : 6 + une par tranche de hauteur = 9."""
        # obstacles, exploré, couverture × tranches, frontière, QR lus, trace du drone, trace des coéquipiers
        return 6 + self.n_bands

    @property
    def obs_dim(self) -> int:
        """Renvoie la taille de la partie « cartes » de l'observation : 2 × 9 × 32 × 32 = 18 432."""
        return len(self.crop_spans_m) * self.n_channels * self.crop_px * self.crop_px


@dataclass
class RewardMapConfig:
    """Poids de la récompense v2 : couverture (signal fréquent), lectures, aide au guidage, pénalités ; aucune taxe de temps."""

    facade_gain: float = 1.0              # par case neuve couverte À CÔTÉ d'un obstacle vu (façade de rack)
    area_gain: float = 0.1                # par case neuve quelconque
    marginal_gain: float = 0.5            # par case neuve couverte par CE SEUL drone (évite qu'ils s'agglutinent)
    overlap_penalty: float = 0.02         # par case déjà couverte, re-balayée en même temps qu'un coéquipier ;
                                          # abaissée avec scan_norm : la taxe reste ~0,05 point/pas/drone
    overlap_off_coverage: float = 0.90    # cette pénalité s'arrête quand 90 % des QR lisibles sont lus
    new_qr: float = 25.0                  # par nouveau QR lu : revenu PRINCIPAL (~50 QR × 25 ≫ couverture), viser devient rentable
    new_qr_nominal: float = 10.0          # bonus 1×/QR/épisode si la posture NOMINALE est tenue devant lui : le détour doit valoir une lecture
    read_gain_rise: float = 2.0           # lectures × (1 + 2·t), t = avancement du curriculum (0 → 1) : au nominal une lecture coûte ~4 s de manœuvre, elle paye 30
    coverage_fade: float = 0.5            # couverture × (1 − 0,5·t) : avec les progrès, la priorité passe de balayer à lire
    milestones: tuple[float, ...] = (0.5, 0.75, 0.9)   # paliers de fraction de QR lus qui donnent un bonus d'équipe
    milestone_bonus: float = 10.0         # points par palier franchi (pour toute l'équipe)
    shaping_scale: float = 0.5            # poids du potentiel d'aide γΦ'−Φ (rapprochement d'un QR non lu), ENTRAÎNEMENT seulement
    shaping_clip_m: float = 12.0          # m : distance max prise en compte par le potentiel
    slow_potential: float = 0.3           # part du potentiel qui paie le fait d'être LENT près d'un QR non lu (guide vers la posture de lecture)
    collision_scale: float = 2.0          # pénalité croissante sous safe_distance, sans jamais finir l'épisode
    safe_distance_m: float = 0.5          # m
    contact_penalty: float = 5.0          # par pas sous collision_distance (contact)
    collision_distance_m: float = 0.25    # m
    # Pénalité des à-coups ‖a_t − a_{t−1}‖². Elle est calculée sur l'action TIRÉE AU HASARD, donc
    # elle punissait surtout le bruit d'exploration : à σ = 0,29, environ −454 points par épisode
    # venaient du seul bruit, pour −486 mesurés. Effet : l'entropie (le hasard des actions)
    # s'effondrait de 2,90 à 0,41 en 1600 itérations. Divisée par 10 : −45 points par épisode.
    action_diff: float = 0.005
    bound_penalty: float = 0.5            # pénalité (|a|−1)₊² hors de [−1, 1] : redonne le gradient que l'écrêtage seul supprime
    separation_m: float = 0.6             # m : pénalité GRADUÉE entre drones sous cette distance, pas une marche :
    separation_penalty: float = 0.25      # en marche à 1,0, elle coûtait −1098/épisode dans les BONS
                                          # épisodes contre −111 dans les mauvais (mesuré) et annulait
                                          # à elle seule le revenu des lectures (+1142)
    # Unité d'aire FIXE (en cases) pour la couverture. Normaliser par l'empreinte du moment
    # (105 cases pour deux cônes latéraux de 2,5 m, contre ~13 en frontal à 1,25 m) divisait par 8
    # le revenu de couverture : couvrir toute l'arène rapportait 124 points contre 111 QR × 25 = 2775
    # pour les lectures. Avec 13, couvrir une tranche de hauteur complète vaut ~1000 points.
    scan_norm: float = 13.0


@dataclass
class LayoutConfig:
    """Tirage des configurations d'inventaire (cartons portant un QR) et découpage figé train/val/test."""

    # Part des cartons qui portent un QR : presque tous (réaliste). Avec moins, les façades à QR
    # étaient indiscernables des autres sur la carte et la lecture plafonnait à ~0,4.
    active_frac: tuple[float, float] = (0.95, 1.0)
    # Part des QR « déjà lus » au départ : ZÉRO, pour que « façade non balayée » = « travail restant ».
    preread_frac: tuple[float, float] = (0.0, 0.0)
    train_configs: int = 500              # configurations d'entraînement (numéros 0-499)
    val_configs: int = 20                 # configurations de validation (500-519)
    test_configs: int = 20                # configurations de test, jamais vues à l'entraînement (520-539)
    base_seed: int = 1234                 # graine de base des tirages (reproductibles)
    max_tag_z_m: float = 4.55             # m : altitude max 4,0 + 1,25·sin(30°) ; au-dessus, QR inatteignable (exclu du total)


@dataclass
class TrainMapConfig:
    """Réglages de la boucle d'entraînement (durée d'épisode, objectif de mission, PPO)."""

    episode_length_s: float = 150.0       # s au cran 0 : plan optimal = 131 s avec visée de 2 pas et
                                          # vitesse limitée à 0,75 m/s ; 120 s était physiquement infaisable
    mission_target: float = 0.95          # fin d'épisode quand 95 % des QR LISIBLES (actifs et atteignables) sont lus
    gamma: float = 0.995                  # facteur d'actualisation de PPO (poids du futur)
    lam: float = 0.95                     # λ du GAE (estimation de l'avantage)
    lidar_sectors: int = 72               # secteurs de 5° : distance mini par secteur, tirée des 1800 rayons du lidar
    init_noise_std: float = 0.5           # écart-type initial σ du bruit d'exploration des actions
    # Bonus d'entropie (garde du hasard dans les actions) : 0,01 en phase de MONTÉE (défaut) ;
    # 0,001 en phase de CONVERGENCE (relancer avec --entropy 0.001, ou bascule automatique de
    # train.py au niveau --converge_level). Raison : le bruit permanent faisait la visée à la
    # place du réseau (testé : 0,05 en déterministe, sans bruit, contre 0,66 avec bruit).
    entropy_coef: float = 0.01


@dataclass
class SwarmScanMapConfig:
    """Regroupe toutes les sections de réglages ci-dessus."""

    gate: GateConfig = field(default_factory=GateConfig)
    curriculum: CurriculumConfig = field(default_factory=CurriculumConfig)
    map: MapConfig = field(default_factory=MapConfig)
    reward: RewardMapConfig = field(default_factory=RewardMapConfig)
    layout: LayoutConfig = field(default_factory=LayoutConfig)
    train: TrainMapConfig = field(default_factory=TrainMapConfig)


MAP_CFG = SwarmScanMapConfig()  # instance unique importée par tous les fichiers du dossier
