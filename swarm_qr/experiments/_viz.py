"""Vue de dessus de l'entrepôt pour les expériences : caméra de survol fixe et toit masqué.

À importer après le démarrage d'Isaac Sim (caméras natives du simulateur). Réexporte aussi les
fonctions d'image de `_img`. Utilisé par les tests 01, 02, 03 et par 05_sitl/debug_cams.py.
"""

from __future__ import annotations

import numpy as np
from isaacsim.sensors.camera import Camera

from ._img import board, label, save, side_by_side, to_bgr, video  # noqa: F401

OVERVIEW_PRIM = "/World/Overview"  # chemin de la caméra de survol dans la scène
ROOF_KEYWORDS = ("ceiling", "beam", "lamp", "pillar", "roof")  # objets cachés, repérés par leur nom


def hide_roof(stage) -> int:
    """Rend invisibles plafond, poutres, lampes, piliers et toit pour la vue de dessus (seulement à
    l'image : collisions et lidar les voient encore) ; renvoie le nombre d'objets cachés."""
    from pxr import UsdGeom

    n = 0
    for prim in stage.Traverse():
        if any(k in prim.GetName().lower() for k in ROOF_KEYWORDS):
            UsdGeom.Imageable(prim).MakeInvisible()
            n += 1
    return n


def overview_camera(height: float = 26.0, size: int = 720, focal: float = 18.0) -> Camera:
    """Crée et renvoie la caméra fixe qui regarde l'entrepôt d'en haut (à `height` m, image
    carrée de `size` px)."""
    # Orientation en convention monde (avant = +X) : un tangage de +90° fait regarder droit vers
    # le sol.
    return Camera(
        prim_path=OVERVIEW_PRIM,
        position=np.array([-0.5, 3.0, height]),
        orientation=np.array([0.70710678, 0.0, 0.70710678, 0.0]),
        resolution=(size, size),
    )


def overview_init(cam: Camera, focal: float = 18.0) -> None:
    """Branche la caméra au rendu (à appeler après world.reset()) et règle son objectif : focale
    `focal`, ouverture 20,955, images entre 0,5 et 80 m."""
    cam.initialize()
    cam.set_focal_length(focal)
    cam.set_horizontal_aperture(20.955)
    cam.set_clipping_range(0.5, 80.0)
