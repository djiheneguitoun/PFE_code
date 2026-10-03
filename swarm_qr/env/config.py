"""Constantes du simulateur : dimensions de l'entrepôt et des racks, mesurées dans son fichier USD (format de scène 3D
d'Isaac Sim), grille de la carte, réglages des drones, des caméras, du lidar et des QR. Longueurs en mètres.

Python pur, sans simulateur ; importé par tous les modules de swarm_qr.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Fichier USD de l'entrepôt « Simple_Warehouse » d'Isaac 4.2, chargé par son adresse web (gardé en cache local).
WAREHOUSE_USD = (
    "http://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/4.2/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
)
WAREHOUSE_PRIM = "/World/Warehouse"   # chemin de l'entrepôt dans la scène


@dataclass(frozen=True)
class Interior:
    """L'intérieur de l'entrepôt (m) : murs en x et en y, plafond, et altitude de vol maximale."""
    x_min: float = -10.13
    x_max: float = 9.21
    y_min: float = -12.00
    y_max: float = 17.80
    z_ceiling: float = 9.00   # m : plafond
    z_fly_max: float = 5.80   # m : altitude de vol maximale

    @property
    def width(self) -> float:
        """Renvoie la largeur intérieure en x (19,34 m)."""
        return self.x_max - self.x_min

    @property
    def length(self) -> float:
        """Renvoie la longueur intérieure en y (29,80 m)."""
        return self.y_max - self.y_min


@dataclass(frozen=True)
class Racks:
    """Les trois racks (grandes étagères) de l'entrepôt : noms dans le fichier USD, position d'origine et dimensions (m)."""
    prims: tuple[str, ...] = ("Shelf_0", "Shelf_1", "Shelf_2")   # noms des racks dans le fichier USD
    default_x: tuple[float, ...] = (-9.30, -0.30, 8.68)   # m : x du centre de chaque rack dans le fichier d'origine
    depth: float = 1.40   # m : épaisseur d'un rack (en x)
    length: float = 17.62   # m : longueur d'un rack (en y)
    height: float = 6.00   # m
    default_y_min: float = -4.489   # m : y du bout sud des racks dans le fichier d'origine
    shelf_levels: tuple[float, ...] = (1.325, 2.825, 4.125)   # m : hauteur des trois étages (planches) qui portent les cartons


@dataclass(frozen=True)
class MapGrid:
    """La grille 3D de la carte partagée : emprise (m) un peu plus grande que l'entrepôt, découpée en cubes de 25 cm."""
    x_min: float = -11.0
    x_max: float = 10.0
    y_min: float = -13.0
    y_max: float = 19.0
    z_min: float = 0.0
    z_max: float = 6.0
    cell: float = 0.25   # m : côté d'un cube

    @property
    def shape(self) -> tuple[int, int, int]:
        """Renvoie le nombre de cubes en x, y et z (84 × 128 × 24)."""
        return (
            round((self.x_max - self.x_min) / self.cell),
            round((self.y_max - self.y_min) / self.cell),
            round((self.z_max - self.z_min) / self.cell),
        )


@dataclass(frozen=True)
class Drones:
    """Les réglages des drones : nombre, taille, altitude de départ tirée, marge aux racks et vitesse maximale."""
    count: int = 3   # drones par défaut
    body_size: float = 0.18   # m : taille du corps
    spawn_z_range: tuple[float, float] = (1.2, 2.2)   # m : altitude tirée pour chaque départ
    clearance: float = 1.2   # m : distance minimale entre un point de départ et un rack
    max_speed: float = 1.0   # m/s


@dataclass(frozen=True)
class Cameras:
    """Les réglages des caméras des drones : deux latérales 1024 × 768 pour lire les QR, une frontale 160 × 120, champ de 60°."""
    side_width: int = 1024   # px
    side_height: int = 768   # px
    front_width: int = 160   # px
    front_height: int = 120   # px
    fov_deg: float = 60.0   # degrés : champ horizontal
    horizontal_aperture: float = 20.955   # largeur du capteur, même unité que la focale : leur rapport fixe le champ
    near: float = 0.05   # m : distance minimale vue
    far: float = 40.0   # m : distance maximale vue
    update_hz: float = 5.0   # images par seconde
    side_offset: float = 0.10      # les latérales sont à 10 cm du centre du corps...
    below: float = 0.11            # ...et 11 cm plus bas ; vérifié en vol (06_position_vraie)

    @property
    def focal_length(self) -> float:
        """Renvoie la focale qui donne le champ de 60° avec cette largeur de capteur (environ 18,15, même unité)."""
        return self.horizontal_aperture / (2.0 * math.tan(math.radians(self.fov_deg) / 2.0))


@dataclass(frozen=True)
class Lidar:
    """Les réglages du lidar (capteur laser qui mesure les distances) : un tour complet à chaque rendu, pour cartographier.
    Un anneau tous les 2° et non 4° : à 4°, une planche 24 cm sous le drone n'était touchée qu'à 1,4 m devant lui, et le drone
    est passé au-dessus d'une planche dans un rack vide (mesuré à l'étape 7) ; à 2°, la planche est vue dès 3,4 m."""

    horizontal_fov_deg: float = 360.0   # degrés : tour complet, sans rotation simulée
    vertical_fov_deg: float = 36.0   # degrés : 19 anneaux, bande de 1,3 m de haut à 2 m ; un passage par étage, qui se recouvrent
    horizontal_res_deg: float = 2.0   # degrés entre deux rayons d'un anneau (180 rayons par anneau)
    vertical_res_deg: float = 2.0   # degrés entre deux anneaux
    min_range: float = 0.4          # m, au-delà des hélices : sinon le drone se mesure lui-même
    max_range: float = 25.0   # m : au-delà, le rayon compte comme « rien touché »


@dataclass(frozen=True)
class QR:
    """Les réglages des QR : quels objets sont des cartons, texte encodé, taille du panneau, faces qui en portent un."""
    box_name_filter: str = "SM_CardBox"   # un carton est un objet dont le nom contient ce texte
    payload_fmt: str = "BOX_{:03d}"   # texte encodé : BOX_000, BOX_001… (7 caractères, voir qr_tags.py)
    panel_ratio: float = 0.4   # côté du panneau = 2 × 0,4 = 80 % du plus petit côté de la face du carton
    offset: float = 0.01   # décalage du panneau devant la face du carton (1 cm)
    faces: tuple[tuple[int, int], ...] = ((0, 1), (0, -1))   # faces qui portent un QR, en (axe local, sens) : les deux faces ±x


SIM_DT = 1.0 / 60.0   # s : pas de physique utilisé par render_interval


class RenderClock:
    """Décide à quels pas de physique le simulateur rend une image : rendre à chaque pas coûte sept fois le temps réel ;
    à 5 images par seconde, le coût s'effondre et la lecture des QR reste assez fréquente pour un drone lent."""

    def __init__(self, hz: float):
        """Calcule le nombre de pas entre deux rendus pour `hz` images par seconde."""
        self.every = render_interval(hz)
        self.i = -1

    def step(self, sim) -> bool:
        """Fait un pas de simulation, avec rendu un pas sur `every` ; renvoie vrai si ce pas a rendu une image."""
        self.i += 1
        rendered = self.i % self.every == 0
        sim.step(render=rendered)
        return rendered


def render_interval(hz: float) -> int:
    """Renvoie le nombre de pas de physique (de SIM_DT) entre deux rendus pour `hz` images par seconde, au moins 1.
    C'est ce réglage qui fixe le coût : lire les caméras moins souvent ne sert à rien si le simulateur rend quand même."""
    return max(1, round(1.0 / (hz * SIM_DT)))


# Les réglages ci-dessus, prêts à importer par les autres modules.
INTERIOR = Interior()
RACKS = Racks()
MAP = MapGrid()
DRONES = Drones()
CAMERAS = Cameras()
LIDAR = Lidar()
QR_CFG = QR()

# Obstacles bas fixes du couloir nord : (x_min, x_max, y_min, y_max, hauteur), en m.
OBSTACLES = (
    (-7.06, -5.19, 12.90, 14.36, 1.17),
    (3.52, 5.38, 12.90, 14.36, 1.17),
)

TRAIN_SEEDS = range(0, 500)   # graines des entrepôts d'entraînement et de réglage
SEALED_SEEDS = range(9000, 9040)   # graines « scellées » : entrepôts gardés pour l'évaluation (dont 9033 et 9019)
