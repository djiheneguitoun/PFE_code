"""Curriculum de la règle de lecture : on commence tolérant puis on resserre cran par cran.

Méthode ADR (randomisation automatique du domaine, OpenAI ; DORAEMON) pilotée par le taux de
lecture mesuré : monte d'un cran si les drones lisent bien, redescend s'ils s'effondrent.
Pur Python, testable sans Isaac (rl_inventory/tests/test_swarmscan_map_pure.py) ; utilisé par env_map.py.
"""

from __future__ import annotations

from .config_map import CurriculumConfig, GateConfig


def _interp(loose: float, nominal: float, t: float) -> float:
    """Renvoie la valeur entre `loose` (t = 0, cran 0) et `nominal` (t = 1, dernier cran)."""
    return loose + (nominal - loose) * t


class GateCurriculum:
    """Niveau courant du curriculum (0 = règle tolérante, `notches` = règle nominale) et seuils qui en découlent."""

    def __init__(self, gate: GateConfig, cfg: CurriculumConfig):
        """Démarre au niveau 0, avec une moyenne glissante nulle et aucun épisode compté."""
        self.gate = gate
        self.cfg = cfg
        self.level = 0
        self.ema = 0.0
        self.episodes_at_level = 0
        self.episodes_after_nominal = 0
        self.confirm = 0

    @property
    def progress(self) -> float:
        """Renvoie l'avancement du curriculum entre 0 (cran 0) et 1 (règle nominale)."""
        # Écrêté à 1 : au-delà, l'interpolation donnerait une règle PLUS stricte que la nominale.
        # Historique : verify_env.py imposait --level 7 alors qu'il n'y avait plus que 6 crans →
        # avancement 1,167 → distance de lecture 4,0 + (1,25 − 4,0)·1,167 = 0,79 m (37 % plus serré).
        # Toutes les validations ont été faites ainsi et leurs « zéro lecture » ont été pris,
        # à tort, pour la preuve d'un environnement sain.
        return min(self.level / self.cfg.notches, 1.0)

    @property
    def nominal(self) -> bool:
        """Renvoie vrai si le dernier cran (règle nominale) est atteint."""
        return self.level >= self.cfg.notches

    def thresholds(self) -> dict[str, float]:
        """Renvoie les seuils de lecture du niveau courant : distance, angle, vitesse, rotation, temps de visée."""
        t = self.progress
        g = self.gate
        return {
            "read_distance_m": _interp(*g.adr_read_distance_m, t),
            "view_angle_deg": _interp(*g.adr_view_angle_deg, t),
            "max_speed_mps": _interp(*g.adr_max_speed_mps, t),
            "max_yawrate_rps": _interp(*g.adr_max_yawrate_rps, t),
            # temps de visée : ne change qu'au dernier cran (aujourd'hui 2 pas partout)
            "dwell_steps": float(g.adr_dwell_steps[1] if self.nominal else g.adr_dwell_steps[0]),
        }

    def spawn_near_prob(self) -> float:
        """Renvoie la probabilité de faire naître un drone près d'un QR non lu (0,7 au cran 0 → 0,3 au nominal)."""
        return _interp(*self.cfg.spawn_near_prob, self.progress)

    def dropout_prob(self) -> float:
        """Renvoie la probabilité de panne d'un drone : 0 avant le nominal, puis jusqu'à 0,30 en 500 épisodes."""
        if self.cfg.dropout_after_nominal and not self.nominal:
            return self.cfg.dropout_prob[0]
        ramp = min(1.0, self.episodes_after_nominal / 500.0)
        return _interp(*self.cfg.dropout_prob, ramp)

    def thresholds_hi_lo(self) -> tuple[float, float]:
        """Renvoie (barre de promotion, seuil de recul) pour le niveau courant."""
        hi = max(self.cfg.success_hi - self.cfg.hi_decay_per_notch * self.level, self.cfg.hi_min)
        lo = max(hi - self.cfg.lo_gap, self.cfg.lo_min)
        return hi, lo

    def on_episodes_end(self, read_fracs: list[float]):
        """Ajoute les taux de lecture (règle du niveau courant) des épisodes finis, puis monte ou descend d'un cran."""
        hi, lo = self.thresholds_hi_lo()
        for f in read_fracs:
            self.ema = (1 - self.cfg.ema_alpha) * self.ema + self.cfg.ema_alpha * f
            self.episodes_at_level += 1
            if self.nominal:
                self.episodes_after_nominal += 1
            # nombre d'épisodes DE SUITE avec la moyenne au-dessus de la barre (0,88)
            self.confirm = self.confirm + 1 if self.ema > hi + self.cfg.promo_margin else 0
        # monte : 25 épisodes de suite au-dessus et au moins 120 épisodes à ce cran ;
        # descend : moyenne sous le seuil de recul, après au moins 250 épisodes à ce cran
        promotable = (self.confirm >= self.cfg.confirm_episodes
                      and self.episodes_at_level >= self.cfg.min_episodes_per_notch)
        if promotable and self.level < self.cfg.notches:
            self.level += 1
            self.episodes_at_level = 0
            self.confirm = 0
        elif self.ema < lo and self.level > 0 and self.episodes_at_level >= self.cfg.min_episodes_down:
            self.level -= 1
            self.episodes_at_level = 0
            self.confirm = 0

    def state(self) -> dict[str, float]:
        """Renvoie le niveau, la moyenne glissante et la probabilité de panne (pour les journaux)."""
        return {"level": float(self.level), "ema": self.ema, "dropout_p": self.dropout_prob()}
