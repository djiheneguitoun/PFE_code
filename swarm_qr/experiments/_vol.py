"""Chemins de vol simples par les allées, pour les expériences faites avant la carte de l'étape 4.

Entre deux racks, l'allée est vide : deux points d'une même allée se rejoignent en ligne droite.
Pour changer d'allée, on passe par le couloir libre au bout des racks (la ligne droite
traverserait un rack). Utilisé par 03_images_qr, 07_enveloppe et 08_controle ; un seul module
partagé, pour éviter des copies qui divergent.
"""

from __future__ import annotations

import numpy as np

from swarm_qr.env.config import INTERIOR

ALT_TRANSIT_MIN = 1.6     # m : altitude minimale ; le couloir nord porte des obstacles bas (1,17 m)
MARGE_COULOIR = 1.0       # m : écart entre le bout des racks et la ligne de vol du couloir


def allee(layout, x: float) -> int:
    """Renvoie le numéro de l'allée qui contient x : le nombre de racks entièrement à sa gauche
    (côté des x plus petits)."""
    return sum(1 for r in layout.racks if r.x_bounds[1] < x)


def couloirs(layout) -> tuple[float, float]:
    """Renvoie le y des couloirs nord et sud : 1 m au-delà du bout des racks, mais jamais à moins
    de 0,8 m d'un mur."""
    y_nord = max(r.y_bounds[1] for r in layout.racks) + MARGE_COULOIR
    y_sud = min(r.y_bounds[0] for r in layout.racks) - MARGE_COULOIR
    lim = (INTERIOR.y_min + 0.8, INTERIOR.y_max - 0.8)
    return float(np.clip(y_nord, *lim)), float(np.clip(y_sud, *lim))


def chemin(layout, depart, cible) -> list[np.ndarray]:
    """Renvoie les points de passage de `depart` à `cible` : aucun dans la même allée, sinon deux
    par le couloir (nord ou sud) le plus court, à l'altitude de la cible mais à 1,6 m au moins."""
    p = np.asarray(depart, float)
    q = np.asarray(cible, float)
    if allee(layout, p[0]) == allee(layout, q[0]):
        return []
    y_nord, y_sud = couloirs(layout)
    y = min((y_nord, y_sud), key=lambda c: abs(p[1] - c) + abs(q[1] - c))
    z = max(q[2], ALT_TRANSIT_MIN)
    return [np.array([p[0], y, z]), np.array([q[0], y, z])]


def transit(pilot, layout, x_allee: float, alt: float, psi: float, get_pos, get_yaw) -> None:
    """Amène le drone (cap `psi`) par le couloir jusqu'à l'entrée de l'allée `x_allee`, sans rien
    faire s'il y est déjà ; l'appelant finit ensuite le trajet dans l'allée."""
    p = np.asarray(get_pos(), float)
    for wp in chemin(layout, p, np.array([x_allee, p[1], alt])) or []:
        pilot.goto(wp, psi, get_pos, tol=0.4, get_yaw=get_yaw, timeout_sim_s=120.0)
