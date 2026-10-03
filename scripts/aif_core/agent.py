"""Un drone de l'essaim (DroneAgent) : sa carte de croyance, sa perception et l'exécution de ses actions.

À chaque pas, le drone lit son lidar (capteur laser qui mesure les distances), mesure sa « surprise »
(innovation : à quel point les obstacles vus n'étaient pas prévus par sa carte), met sa carte à jour,
puis envoie à son pilote automatique le point visé par l'action choisie.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .belief import BeliefGrid, fuse_beliefs_logodds
from .math_utils import clamp


# Anti-blocage : nombre de pas d'affilée avec moins de 0,5 m parcouru avant d'imposer une direction au hasard
_STAG_THRESHOLD: int = 5


class DroneAgent:
    """Un drone : position, carte de croyance, pilote et lidar simulés, dernière décision prise."""

    def __init__(self, drone_id: int, sx: float, sy: float, cfg):
        """Crée le drone n° drone_id en (sx, sy) m locaux, avec sa carte et son hasard propre (graine 42 + 1000 × id)."""
        self.id = drone_id
        self.cfg = cfg
        self.belief = BeliefGrid(cfg)
        self.rng = np.random.default_rng(42 + drone_id * 1000)

        self.trail: List[Tuple[float, float]] = [(sx, sy)]

        self.controller: Optional[Any] = None
        self.lidar: Optional[Any] = None

        self.active: bool = True
        self.total_dist: float = 0.0
        self._prev_xy: Tuple[float, float] = (sx, sy)
        self._stag_pos: Tuple[float, float] = (sx, sy)
        self._stag_steps: int = 0

        self.last_action: str = "stay"
        self.last_G: float = 0.0
        self.last_ig: float = 0.0
        self.last_innovation: float = 0.0
        self.candidates_diag: List[Dict] = []
        self.selected_idx: int = 0

        self.pending_action: Optional[Tuple[str, float, float]] = None
        self.last_action_fresh: bool = False
        self.last_decision_source: str = "none"
        self.last_plan_belief: Optional[BeliefGrid] = None

        self.lidar_diag: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]] = None

        self.last_received_belief: Dict[int, BeliefGrid] = {}
        self.local_fused: Optional[BeliefGrid] = None

    def setup_physical(self, controller, lidar) -> None:
        """Branche le pilote (SitlController) et le lidar (LidarReader) du drone simulé dans Isaac Sim."""
        self.controller = controller
        self.lidar = lidar

    @property
    def x(self) -> float:
        """Renvoie la position X locale (m) : lue sur le pilote s'il existe, sinon dernier point de la trace."""
        if self.controller:
            return self.controller.get_position_xy()[0] - self.cfg.origin_x
        return self.trail[-1][0]

    @property
    def y(self) -> float:
        """Renvoie la position Y locale (m) : lue sur le pilote s'il existe, sinon dernier point de la trace."""
        if self.controller:
            return self.controller.get_position_xy()[1] - self.cfg.origin_y
        return self.trail[-1][1]

    def accumulate_lidar(self) -> None:
        """Ajoute le scan lidar courant au tampon (appelé à chaque pas physique entre deux décisions)."""
        if not self.active or self.lidar is None or self.controller is None:
            return
        self.lidar.accumulate(self.controller.get_yaw())

    def perceive(self) -> None:
        """Lit les scans accumulés, calcule la surprise (moyenne de 1 − p prévue sur les cases touchées),
        puis met la carte à jour ; surprise nulle si le drone est hors service ou sans lidar."""
        if not self.active:
            self.lidar_diag = None
            self.last_innovation = 0.0
            return

        if self.lidar is not None:
            angles, ranges, hits = self.lidar.get_accumulated()
            self.lidar_diag = (angles.copy(), ranges.copy(), hits.copy())

            # Surprise calculée AVANT la mise à jour : 0 si la carte prévoyait l'obstacle, 1 si elle croyait la case libre
            innov_sum = 0.0
            n_compare = 0
            for i in range(len(angles)):
                if not hits[i]:
                    continue
                cos_a = math.cos(angles[i])
                sin_a = math.sin(angles[i])
                hx = self.x + float(ranges[i]) * cos_a
                hy = self.y + float(ranges[i]) * sin_a
                gx, gy = self.belief.world_to_grid(hx, hy)
                if self.belief.in_bounds(gx, gy):
                    predicted_occ = self.belief.probability[gy, gx]
                    innov_sum += abs(1.0 - predicted_occ)
                    n_compare += 1
            self.last_innovation = innov_sum / max(n_compare, 1)

            self.belief.update_from_lidar(
                self.x, self.y, angles, ranges, hits,
                self.cfg.lidar_max_range, self.cfg.lo_free, self.cfg.lo_occ,
            )
        else:
            self.lidar_diag = None
            self.last_innovation = 0.0

    def fuse_with_neighbors(self, neighbors: List["DroneAgent"],
                            prior_lo: float) -> BeliefGrid:
        """Fusionne la carte du drone avec la dernière carte reçue de chaque voisin donné ; renvoie la carte fusionnée."""
        beliefs = [self.belief]
        for n in neighbors:
            cached = self.last_received_belief.get(n.id)
            if cached is not None:
                beliefs.append(cached)
        self.local_fused = fuse_beliefs_logodds(beliefs, prior_lo)
        return self.local_fused

    def execute(self, action: Tuple[str, float, float]) -> None:
        """Envoie au pilote le point visé (position + 1 pas dans la direction de l'action, gardé dans la zone).
        Anti-blocage : après 5 pas presque immobile (< 0,5 m), une direction au hasard est imposée."""
        name, dx, dy = action
        cx, cy = self.x, self.y

        moved = math.hypot(cx - self._stag_pos[0], cy - self._stag_pos[1])
        if moved < 0.5:
            self._stag_steps += 1
        else:
            self._stag_steps = 0
            self._stag_pos = (cx, cy)

        if self._stag_steps >= _STAG_THRESHOLD:
            # Direction tirée au hasard parmi les 8 mouvements (jamais "stay")
            from .planner import ACTIONS
            idx = int(self.rng.integers(1, len(ACTIONS)))
            name, dx, dy = ACTIONS[idx]
            self._stag_steps = 0
            print(f"  [STAG] D{self.id} stuck → forced {name}")

        tx = clamp(cx + dx * self.cfg.step_size, 0.5, self.cfg.env_width - 0.5)
        ty = clamp(cy + dy * self.cfg.step_size, 0.5, self.cfg.env_height - 0.5)

        if self.controller is not None:
            self.controller.set_target(
                tx + self.cfg.origin_x,
                ty + self.cfg.origin_y,
                self.cfg.fly_altitude,
            )

        self.total_dist += math.hypot(cx - self._prev_xy[0], cy - self._prev_xy[1])
        self._prev_xy = (cx, cy)
        self.trail.append((round(cx, 2), round(cy, 2)))
        if len(self.trail) > 200:
            self.trail = self.trail[-200:]

        self.last_action = name
        if self.candidates_diag and self.selected_idx < len(self.candidates_diag):
            sel = self.candidates_diag[self.selected_idx]
            self.last_G = sel.get("G", 0.0)
            self.last_ig = sel.get("ig", 0.0)

    def land(self) -> None:
        """Met le drone hors service (perte simulée) : il ne décide plus et descend à 0,1 m sur place."""
        self.active = False
        self.last_action = "LANDED"
        if self.controller:
            wx = self.x + self.cfg.origin_x
            wy = self.y + self.cfg.origin_y
            self.controller.set_target(wx, wy, 0.1)
        print(f"  [RESILIENCE] 🛬 Drone {self.id} LANDED (out of service)")

    def get_state(self) -> Dict:
        """Renvoie l'état du drone pour le tableau de bord (position, cap, action, G, gain d'info, entropie, trace…)."""
        heading = 0.0
        if len(self.trail) >= 2:
            dx = self.trail[-1][0] - self.trail[-2][0]
            dy = self.trail[-1][1] - self.trail[-2][1]
            if dx != 0 or dy != 0:
                heading = math.atan2(dy, dx)
        return {
            "id": self.id,
            "x": round(self.x, 3), "y": round(self.y, 3),
            "heading": round(heading, 3),
            "action": self.last_action,
            "free_energy": round(self.last_G, 4),
            "info_gain": round(self.last_ig, 4),
            "total_distance": round(self.total_dist, 2),
            "local_entropy": round(self.belief.mean_entropy(), 4),
            "trail": self.trail[-60:],
            "active": self.active,
            "innovation": round(self.last_innovation, 4),
            "decision_source": self.last_decision_source,
        }
