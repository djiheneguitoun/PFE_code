"""Tire un entrepôt à partir d'une graine (un numéro) : position des racks, part des cartons gardés, départs des drones.

Python pur, sans simulateur : la reproductibilité et la variation se testent en quelques millisecondes
(experiments/01_reproductibilite et 02_variation). Utilisé par scene.py et par les missions : make_layout(9033).
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from .config import DRONES, INTERIOR, OBSTACLES, RACKS

MIN_AISLE = 3.0   # m : largeur minimale d'une allée entre deux racks
MARGIN = 0.6   # m : marge gardée le long des murs


@dataclass(frozen=True)
class RackPlacement:
    """La place d'un rack dans un entrepôt tiré : nom dans le fichier USD, x de son centre et y de son bout sud (m)."""
    prim: str
    x: float
    y_min: float

    @property
    def x_bounds(self) -> tuple[float, float]:
        """Renvoie les bornes en x du rack (m) : son centre ± 0,70 m."""
        half = RACKS.depth / 2.0
        return (self.x - half, self.x + half)

    @property
    def y_bounds(self) -> tuple[float, float]:
        """Renvoie les bornes en y du rack (m), du bout sud au bout nord, 17,62 m plus loin."""
        return (self.y_min, self.y_min + RACKS.length)


@dataclass(frozen=True)
class Layout:
    """Un entrepôt tiré d'une graine : racks, part des cartons gardés, graine du tirage des cartons, départs des drones (x, y, z)."""
    seed: int
    racks: tuple[RackPlacement, ...]
    fill_fraction: float
    box_seed: int
    spawns: tuple[tuple[float, float, float], ...] = field(default=())

    def signature(self) -> tuple:
        """Renvoie un résumé arrondi de l'entrepôt, pour vérifier que deux tirages de la même graine sont identiques."""
        return (
            tuple((r.prim, round(r.x, 4), round(r.y_min, 4)) for r in self.racks),
            round(self.fill_fraction, 6),
            self.box_seed,
            tuple(tuple(round(c, 4) for c in s) for s in self.spawns),
        )

    def rack_spread(self) -> float:
        """Renvoie l'écart en x (m) entre le rack le plus à l'ouest et le plus à l'est."""
        return max(r.x for r in self.racks) - min(r.x for r in self.racks)


def _sorted_positions(rng: random.Random, lo: float, hi: float, n: int, gap: float) -> list[float]:
    """Tire `n` positions triées entre `lo` et `hi`, espacées d'au moins `gap` ; lève ValueError si l'intervalle est trop court."""
    span = (hi - lo) - (n - 1) * gap
    if span < 0:
        raise ValueError("intervalle trop court pour placer les racks")
    picks = sorted(rng.uniform(0.0, span) for _ in range(n))
    return [lo + p + i * gap for i, p in enumerate(picks)]


def _rack_x_positions(rng: random.Random) -> list[float]:
    """Tire le x du centre des trois racks : au moins 3 m d'allée entre eux et 0,6 m de marge aux murs."""
    half = RACKS.depth / 2.0
    lo = INTERIOR.x_min + half + MARGIN
    hi = INTERIOR.x_max - half - MARGIN
    return _sorted_positions(rng, lo, hi, len(RACKS.prims), RACKS.depth + MIN_AISLE)


def _rack_y_min(rng: random.Random) -> float:
    """Tire le y du bout sud d'un rack, pour qu'il tienne en entier entre les murs avec 0,6 m de marge."""
    lo = INTERIOR.y_min + MARGIN
    hi = INTERIOR.y_max - MARGIN - RACKS.length
    return rng.uniform(lo, hi)


def _blocked(x: float, y: float, racks: tuple[RackPlacement, ...]) -> bool:
    """Renvoie vrai si le point (x, y) est à moins de 1,2 m d'un rack ou à moins de 1 m d'un obstacle fixe."""
    for r in racks:
        rx0, rx1 = r.x_bounds
        ry0, ry1 = r.y_bounds
        c = DRONES.clearance
        if rx0 - c <= x <= rx1 + c and ry0 - c <= y <= ry1 + c:
            return True
    for ox0, ox1, oy0, oy1, _ in OBSTACLES:
        if ox0 - 1.0 <= x <= ox1 + 1.0 and oy0 - 1.0 <= y <= oy1 + 1.0:
            return True
    return False


def _spawns(rng: random.Random, racks: tuple[RackPlacement, ...]) -> tuple:
    """Tire les départs des drones (x, y, z) hors des zones bloquées, à plus de 2 m l'un de l'autre ; erreur après 4 000 essais."""
    out: list[tuple[float, float, float]] = []
    for _ in range(4000):
        if len(out) == DRONES.count:
            break
        x = rng.uniform(INTERIOR.x_min + MARGIN, INTERIOR.x_max - MARGIN)
        y = rng.uniform(INTERIOR.y_min + MARGIN, INTERIOR.y_max - MARGIN)
        if _blocked(x, y, racks):
            continue
        if any((x - px) ** 2 + (y - py) ** 2 < 4.0 for px, py, _ in out):
            continue
        out.append((x, y, rng.uniform(*DRONES.spawn_z_range)))
    if len(out) < DRONES.count:
        raise RuntimeError("pas assez de place libre pour poser les drones")
    return tuple(out)


def make_layout(seed: int) -> Layout:
    """Renvoie l'entrepôt de la graine `seed` (même graine, même entrepôt) : racks, 50 à 100 % des cartons gardés, départs."""
    rng = random.Random(seed)
    xs = _rack_x_positions(rng)
    racks = tuple(
        RackPlacement(prim=p, x=x, y_min=_rack_y_min(rng))
        for p, x in zip(RACKS.prims, xs)
    )
    fill = rng.uniform(0.5, 1.0)
    box_seed = rng.randrange(1 << 30)
    return Layout(
        seed=seed,
        racks=racks,
        fill_fraction=fill,
        box_seed=box_seed,
        spawns=_spawns(rng, racks),
    )


def select_boxes(box_paths: list[str], layout: Layout) -> list[str]:
    """Renvoie, triés, les cartons gardés : la part `fill_fraction` des cartons, tirée avec la graine des cartons (au moins un)."""
    rng = random.Random(layout.box_seed)
    keep = sorted(box_paths)
    rng.shuffle(keep)
    n = max(1, round(len(keep) * layout.fill_fraction))
    return sorted(keep[:n])
