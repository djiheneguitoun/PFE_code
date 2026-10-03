"""Construit la scène Isaac Sim d'un entrepôt : bâtiment (chargé par son adresse web, gardé en cache), racks placés
selon la graine, cartons gardés, QR collés, drones Iris de Pegasus avec trois caméras et un lidar chacun.

À importer seulement après le démarrage de SimulationApp ; pas d'Isaac Lab, incompatible avec le monde (World) de Pegasus.
Utilisé par mission.py, pore_mission.py et les expériences : scene = build(make_layout(graine), with_sitl=True).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.camera import Camera
from scipy.spatial.transform import Rotation

import omni.usd

from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (
    ArduPilotMavlinkBackend,
    ArduPilotMavlinkBackendConfig,
)
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, WORLD_SETTINGS

from . import qr_tags
from .config import CAMERAS, DRONES, LIDAR, QR_CFG, RACKS, WAREHOUSE_PRIM, WAREHOUSE_USD
from .layout import Layout, select_boxes

DRONE_PRIM = "/World/Drone_{:02d}"   # chemin du drone dans la scène : Drone_00, Drone_01…
GROUND_Z = 0.07   # m : altitude de pose des drones au départ

# Le rendu a jusqu'à 4 images de retard sur la position réelle : 10 rendus garantissent une
# image à jour. Chaque rendu est précédé d'un peu de physique, sans quoi le pilote ArduPilot,
# qui tourne dans un processus séparé, cesse d'être alimenté et perd le contrôle du drone.
RENDER_LAG = 10   # rendus
PHYSICS_BETWEEN_RENDERS = 40   # pas de physique (1/800 s chacun) avant chaque rendu

def _cam_orientation(yaw_deg: float) -> np.ndarray:
    """Renvoie le quaternion (w, x, y, z) d'une caméra tournée de `yaw_deg` degrés par rapport à l'avant du drone.
    Un simple lacet suffit : Camera lit l'orientation en convention monde (avant +X, haut +Z) et convertit elle-même pour USD."""
    h = math.radians(yaw_deg) / 2.0
    return np.array([math.cos(h), 0.0, 0.0, math.sin(h)])


@dataclass
class Scene:
    """La scène construite : monde, entrepôt tiré, drones, caméras et lidar de chaque drone, QR posés, cartons gardés."""
    world: World
    layout: Layout
    drones: list[Multirotor]
    cameras: list[dict[str, Camera]]
    lidars: list[str] = field(default_factory=list)
    tags: list = field(default_factory=list)
    kept_boxes: list = field(default_factory=list)
    _lidar_api: object = None

    def finalize(self) -> None:
        """Branche les caméras des drones au rendu et règle leur optique ; à appeler une fois, après world.reset()."""
        for per_drone in self.cameras:
            for cam in per_drone.values():
                cam.initialize()
                cam.set_focal_length(CAMERAS.focal_length)
                cam.set_horizontal_aperture(CAMERAS.horizontal_aperture)
                cam.set_clipping_range(CAMERAS.near, CAMERAS.far)

    def positions(self) -> np.ndarray:
        """Renvoie les positions vraies (m) de tous les drones, une ligne par drone."""
        return np.array([d.state.position for d in self.drones])

    def position(self, drone: int = 0) -> np.ndarray:
        """Renvoie la position vraie (m) du drone dans le monde."""
        return np.array(self.drones[drone].state.position, float)

    def yaw(self, drone: int = 0) -> float:
        """Renvoie le cap vrai (rad) du drone dans le monde."""
        return float(Rotation.from_quat(self.drones[drone].state.attitude).as_euler("ZYX")[0])

    def velocity(self, drone: int = 0) -> np.ndarray:
        """Renvoie la vitesse vraie (m/s) du drone dans le monde."""
        return np.array(self.drones[drone].state.linear_velocity, float)

    def rgb(self, name: str, drone: int = 0) -> np.ndarray:
        """Renvoie la dernière image de la caméra `name` du drone, sans attendre une image à jour."""
        return self.cameras[drone][name].get_rgb()

    def lidar(self, drone: int = 0):
        """Renvoie un tour de lidar dans le repère du monde : directions unitaires et distance par rayon (infini si rien n'est touché).
        Pièges traités : le capteur ne se met à jour qu'au rendu (erreur s'il est muet) et son angle vertical se compte vers le bas."""
        from omni.isaac.range_sensor import _range_sensor

        if self._lidar_api is None:
            self._lidar_api = _range_sensor.acquire_lidar_sensor_interface()
        chemin = self.lidars[drone]
        d = self._lidar_api.get_linear_depth_data(chemin)
        if d is None:
            raise RuntimeError("lidar sans donnees : le capteur n'est pas initialise")
        portees = np.asarray(d, dtype=float).reshape(-1)
        if not np.count_nonzero(portees > LIDAR.min_range):
            raise RuntimeError("lidar muet : aucun rendu depuis le dernier deplacement")
        az = np.asarray(self._lidar_api.get_azimuth_data(chemin), dtype=float)
        zen = np.asarray(self._lidar_api.get_zenith_data(chemin), dtype=float)
        A, Z = np.meshgrid(az, zen, indexing="ij")
        # « zénith » compté vers le bas : −15° pointe 15° vers le haut, d'où le signe moins sur z
        local = np.stack([np.cos(Z) * np.cos(A), np.cos(Z) * np.sin(A), -np.sin(Z)], axis=-1)
        # directions tournées avec l'attitude du drone, sinon la carte serait de travers dès qu'il tourne
        R = Rotation.from_quat(self.drones[drone].state.attitude).as_matrix()
        dirs = local.reshape(-1, 3) @ R.T
        portees[portees >= LIDAR.max_range - 1e-3] = np.inf
        return dirs, portees

    def capture(self, name: str, drone: int = 0, settle: int = RENDER_LAG) -> np.ndarray:
        """Renvoie une image à jour de la caméra `name` du drone (voir capture_camera)."""
        return capture_camera(self.world, self.cameras[drone][name], settle)


def capture_camera(world: World, cam: Camera, settle: int = RENDER_LAG,
                   physique_entre_rendus: int = PHYSICS_BETWEEN_RENDERS) -> np.ndarray:
    """Renvoie une image à jour de la caméra : par défaut 10 rendus, précédés chacun de 40 pas de physique ; erreur après 120 rendus vides.
    `physique_entre_rendus=0` est réservé aux scènes sans drone SITL : personne à alimenter, capture 4 fois plus rapide."""
    for _ in range(settle):
        for _ in range(physique_entre_rendus):
            world.step(render=False)
        world.step(render=True)
    for _ in range(120):
        img = cam.get_rgb()
        if img is not None and getattr(img, "ndim", 0) == 3 and img.size:
            return img
        world.step(render=True)
    raise RuntimeError("camera vide apres 120 rendus")


def _add_light(stage) -> None:
    """Ajoute une lumière ambiante en dôme, d'intensité 2500, qui éclaire tout l'entrepôt."""
    from pxr import UsdLux

    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(2500.0)


def _place_racks(stage, layout: Layout) -> None:
    """Déplace chaque rack du fichier USD jusqu'à la position tirée pour cette graine, en décalant sa translation."""
    from pxr import Gf, UsdGeom

    for placement in layout.racks:
        prim = stage.GetPrimAtPath(f"{WAREHOUSE_PRIM}/{placement.prim}")
        if not prim or not prim.IsValid():
            continue
        default_x = RACKS.default_x[RACKS.prims.index(placement.prim)]
        dx = placement.x - default_x
        dy = placement.y_min - RACKS.default_y_min
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                cur = op.Get() or Gf.Vec3d(0, 0, 0)
                op.Set(Gf.Vec3d(cur[0] + dx, cur[1] + dy, cur[2]))
                break


def _find_boxes(stage) -> list[str]:
    """Renvoie, triés, les chemins des cartons (objets dont le nom contient « SM_CardBox »), en ne gardant que l'objet
    le plus haut : ses enfants portent souvent le même nom, les compter doublerait cartons et QR."""
    from pxr import UsdGeom

    needle = QR_CFG.box_name_filter.lower()
    out: list[str] = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Xformable):
            continue
        if needle not in prim.GetName().lower():
            continue
        path = str(prim.GetPath())
        if any(path.startswith(kept + "/") for kept in out):
            continue
        out.append(path)
    return sorted(out)


def _hide(stage, paths) -> None:
    """Désactive les cartons non retenus : ni rendu, ni physique. Seulement invisibles, ils gardaient leur forme de collision
    et arrêtaient encore les rayons du lidar (étape 7 : neuf cartons cachés sur neuf)."""
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if prim and prim.IsValid():
            prim.SetActive(False)


def _orientation_monde(yaw_deg: float, plongee_deg: float) -> np.ndarray:
    """Renvoie le quaternion (w, x, y, z) d'une caméra fixe : lacet `yaw_deg` autour de z, puis plongée `plongee_deg`
    vers le bas (degrés), en convention monde de Camera (avant +X, haut +Z)."""
    a, b = math.radians(yaw_deg) / 2.0, math.radians(plongee_deg) / 2.0
    ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
    return np.array([ca * cb, -sa * sb, ca * sb, sa * cb])


# Cinq caméras fixes de vidéosurveillance, à 3,5 m sur les murs, qui regardent le long des
# couloirs : à cette hauteur on voit toute la longueur d'un couloir, ses deux faces de rack, et
# à quel étage vole chaque drone. La cinquième, en hauteur dans un coin, voit tout l'entrepôt.
# Positions pour l'entrepôt 9033 (murs à x = -10,5 et 9,25, y = -12,25 et 17,75).
CAMERAS_VIDEO = {
    "sud_ouest":    dict(position=(-9.40, -11.6, 3.5), yaw=90.0,  plongee=8.0,  fov=60.0),
    "sud_central":  dict(position=(-4.96, -11.6, 3.5), yaw=90.0,  plongee=8.0,  fov=60.0),
    "nord_central": dict(position=(-4.96, 17.3, 3.5),  yaw=-90.0, plongee=8.0,  fov=60.0),
    "sud_grande":   dict(position=(2.70, -11.6, 3.5),  yaw=90.0,  plongee=8.0,  fov=80.0),
    "ensemble":     dict(position=(-10.1, -11.6, 5.8), yaw=56.0,  plongee=22.0, fov=95.0),
}
# Le jeu retenu pour les vidéos : les deux caméras qui regardent dans les deux couloirs
# principaux, le couloir central et la grande zone. Les trois autres restent disponibles.
CAMERAS_VIDEO_2 = {n: CAMERAS_VIDEO[n] for n in ("sud_central", "sud_grande")}
CAMERAS_VIDEO_3 = {n: CAMERAS_VIDEO[n] for n in ("ensemble", "sud_central", "sud_grande")}
RESOLUTION_VIDEO = (960, 540)   # px


def cameras_fixes(specs: dict = CAMERAS_VIDEO, resolution=RESOLUTION_VIDEO) -> dict[str, Camera]:
    """Crée les caméras fixes de vidéosurveillance décrites dans `specs` (après world.reset) ; renvoie {nom: caméra}."""
    cams = {}
    for nom, c in specs.items():
        cam = Camera(prim_path=f"/World/Video/Cam_{nom}", position=np.array(c["position"], dtype=float),
                     orientation=_orientation_monde(c["yaw"], c["plongee"]), resolution=resolution)
        cam.initialize()                                # branche la caméra au rendu (après world.reset)
        # ouverture ET focale, dans la même unité que les caméras des drones : avec l'ouverture par
        # défaut, une focale de 18 donnait un téléobjectif dix fois trop serré
        cam.set_horizontal_aperture(CAMERAS.horizontal_aperture)
        cam.set_focal_length(CAMERAS.horizontal_aperture / (2.0 * math.tan(math.radians(c["fov"]) / 2.0)))
        cam.set_clipping_range(0.2, 80.0)
        cams[nom] = cam
    return cams


def ajoute_obstacle(nom: str, centre_xy, dims=(1.0, 1.0, 2.0)):
    """Pose au sol, en cours de mission, un bloc plein (1 × 1 × 2 m par défaut) avec sa forme de collision ; renvoie son emprise.
    Le lidar doit le découvrir et la carte doit faire recalculer les chemins."""
    from isaacsim.core.api.objects import FixedCuboid

    lx, ly, lz = dims
    FixedCuboid(prim_path=f"/World/Obstacles/{nom}", position=np.array([centre_xy[0], centre_xy[1], lz / 2.0]),
                scale=np.array([lx, ly, lz]), size=1.0, color=np.array([0.85, 0.35, 0.1]))
    return {"nom": nom, "x": [centre_xy[0] - lx / 2, centre_xy[0] + lx / 2],
            "y": [centre_xy[1] - ly / 2, centre_xy[1] + ly / 2], "z": [0.0, lz]}


def _drone_cameras(drone_prim: str) -> dict[str, Camera]:
    """Crée les trois caméras d'un drone : deux latérales 1024 × 768 pour lire, une frontale 160 × 120 pour voir.
    Montées 11 cm sous le corps, elles ne voient ni coque ni hélices (plus de 25° au-dessus de l'axe, champ vertical ±23,6°)."""
    out_dist = CAMERAS.side_offset
    specs = {
        "left": (90.0, CAMERAS.side_width, CAMERAS.side_height),
        "right": (-90.0, CAMERAS.side_width, CAMERAS.side_height),
        "front": (0.0, CAMERAS.front_width, CAMERAS.front_height),
    }
    cams = {}
    for name, (yaw, w, h) in specs.items():
        a = math.radians(yaw)
        cams[name] = Camera(
            prim_path=f"{drone_prim}/body/Cam_{name}",
            translation=np.array([out_dist * math.cos(a), out_dist * math.sin(a), -CAMERAS.below]),
            orientation=_cam_orientation(yaw),
            resolution=(w, h),
        )
    return cams


def _add_lidar(drone_prim: str) -> str:
    """Ajoute le lidar au centre du corps du drone et renvoie son chemin ; sa portée minimale (0,4 m) dépasse les hélices,
    sinon le drone se mesurerait lui-même."""
    import omni.kit.commands

    omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path="/body/Lidar",
        parent=drone_prim,
        min_range=LIDAR.min_range,
        max_range=LIDAR.max_range,
        draw_points=False,
        draw_lines=False,
        horizontal_fov=LIDAR.horizontal_fov_deg,
        vertical_fov=LIDAR.vertical_fov_deg,
        horizontal_resolution=LIDAR.horizontal_res_deg,
        vertical_resolution=LIDAR.vertical_res_deg,
        rotation_rate=0.0,
        high_lod=True,
        yaw_offset=0.0,
        enable_semantics=False,
    )
    return f"{drone_prim}/body/Lidar"


def _make_drone(index: int, spawn_xy: tuple, with_sitl: bool, pg: PegasusInterface) -> Multirotor:
    """Crée le drone Iris numéro `index`, posé au sol au point de départ ; avec `with_sitl`, Pegasus lance aussi son ArduPilot SITL."""
    config = MultirotorConfig()
    if with_sitl:
        config.backends = [
            ArduPilotMavlinkBackend(
                config=ArduPilotMavlinkBackendConfig(
                    {
                        "vehicle_id": index,
                        "ardupilot_autolaunch": True,
                        "ardupilot_dir": pg.ardupilot_path,
                        "ardupilot_vehicle_model": "gazebo-iris",
                    }
                )
            )
        ]
    else:
        config.backends = []

    return Multirotor(
        DRONE_PRIM.format(index),
        ROBOTS["Iris"],
        index,
        [spawn_xy[0], spawn_xy[1], GROUND_Z],
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=config,
    )


def build(layout: Layout, with_sitl: bool = False, n_drones: int | None = None) -> Scene:
    """Construit et renvoie la scène complète de l'entrepôt `layout`, avec `n_drones` drones (3 par défaut).
    `with_sitl=False` : drones posés et inertes, sans autopilote ni terminal, pour les tests qui ne volent pas."""
    n = DRONES.count if n_drones is None else n_drones

    pg = PegasusInterface()
    pg._world = World(**WORLD_SETTINGS["ardupilot"])
    world = pg.world

    add_reference_to_stage(usd_path=WAREHOUSE_USD, prim_path=WAREHOUSE_PRIM)
    world.scene.add_default_ground_plane()

    stage = omni.usd.get_context().get_stage()
    _add_light(stage)
    _place_racks(stage, layout)

    all_boxes = _find_boxes(stage)
    kept = select_boxes(all_boxes, layout)
    _hide(stage, [p for p in all_boxes if p not in set(kept)])

    images = qr_tags.generate_images(len(kept))
    tags = qr_tags.attach(stage, kept, images)

    drones, cameras, lidars = [], [], []
    for i in range(n):
        x, y, _ = layout.spawns[i]
        drones.append(_make_drone(i, (x, y), with_sitl, pg))
        cameras.append(_drone_cameras(DRONE_PRIM.format(i)))
        lidars.append(_add_lidar(DRONE_PRIM.format(i)))

    return Scene(
        world=world,
        layout=layout,
        drones=drones,
        cameras=cameras,
        lidars=lidars,
        tags=tags,
        kept_boxes=kept,
    )
