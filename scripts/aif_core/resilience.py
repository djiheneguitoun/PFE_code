"""Suivi de la résilience : détection des « stress » et phase de comportement de l'essaim.

Un stress (panne programmée ou pic de surprise) fait passer en phase "recovery" pendant 30 pas (alpha),
puis en phase "durable". Il est levé après 60 pas d'affilée (beta) avec une carte « rétablie »
(entropie ≤ 0,44 et surprise ≤ 0,16). planner.py lit la phase pour ajuster les poids de G.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class ResilienceState:
    """Mémoire du stress en cours : actif ou non, cause, pas de début, pas de rétablissement, événements."""

    stress_active: bool = False
    stress_t0: int = -1
    recovered_at: int = -1
    durable_count: int = 0
    cause: str = ""
    events: List[Dict] = field(default_factory=list)

    def to_dict(self) -> Dict:
        """Renvoie l'état sous forme de dictionnaire pour le JSON (20 derniers événements seulement)."""
        return {
            "stress_active": self.stress_active,
            "cause": self.cause,
            "phase": "normal" if not self.stress_active else self.cause,
            "stress_t0": self.stress_t0,
            "recovered_at": self.recovered_at,
            "durable_count": self.durable_count,
            "events": self.events[-20:],
        }


class ResilienceManager:
    """Gestionnaire de résilience : déclenche les stress, repère les pics de surprise et donne la phase courante."""

    def __init__(self, cfg):
        """Démarre sans stress, avec une moyenne et une variance de la surprise à zéro."""
        self.cfg = cfg
        self.state = ResilienceState()
        self.innov_ema: float = 0.0
        self.innov_var: float = 0.0

    def trigger(self, step: int, cause: str) -> None:
        """Déclenche un stress au pas donné ; si un stress est déjà en cours, note seulement l'événement."""
        if self.state.stress_active:
            # Déjà en stress : on note seulement l'événement
            self.state.events.append({
                "step": step, "type": "stress_addon", "cause": cause,
            })
            return
        self.state.stress_active = True
        self.state.stress_t0 = step
        self.state.recovered_at = -1
        self.state.durable_count = 0
        self.state.cause = cause
        self.state.events.append({
            "step": step, "type": "stress_start", "cause": cause,
        })
        print(f"  [RESILIENCE] ⚠ STRESS ACTIVATED at step {step}: {cause}")

    def update_innovation_stats(self, innov_mean: float) -> bool:
        """Met à jour la moyenne et la variance glissantes de la surprise ; renvoie vrai si la surprise du pas
        dépasse moyenne + 2 écarts-types (pic)."""
        err = innov_mean - self.innov_ema
        self.innov_ema += self.cfg.ema_alpha * err
        self.innov_var += self.cfg.ema_alpha * ((err * err) - self.innov_var)
        sigma = math.sqrt(max(self.innov_var, 1e-8))
        return innov_mean > (self.innov_ema + self.cfg.k_sigma * sigma)

    def update_phase(self, step: int, h_mean: float, innov_mean: float) -> None:
        """Note le premier pas où la carte est « rétablie » (entropie ≤ H_target et surprise ≤ innov_target),
        puis lève le stress après beta (60) pas rétablis d'affilée."""
        if not self.state.stress_active:
            return
        cfg = self.cfg
        recovered_now = (h_mean <= cfg.H_target) and (innov_mean <= cfg.innov_target)
        if self.state.recovered_at < 0:
            if recovered_now:
                self.state.recovered_at = step
                self.state.durable_count = 0
                self.state.events.append({
                    "step": step, "type": "recovery_reached",
                    "entropy": round(h_mean, 4),
                    "innovation": round(innov_mean, 4),
                })
                print(f"  [RESILIENCE] ✓ Recovery reached at step {step} (H={h_mean:.4f})")
        else:
            if recovered_now:
                self.state.durable_count += 1
            else:
                self.state.durable_count = 0
            if self.state.durable_count >= cfg.beta:
                self.state.stress_active = False
                self.state.events.append({
                    "step": step, "type": "stress_resolved",
                    "duration": step - self.state.stress_t0,
                })
                print(f"  [RESILIENCE] ✓ STRESS RESOLVED at step {step} "
                      f"(duration={step - self.state.stress_t0} steps)")

    def current_phase(self, step: int) -> str:
        """Renvoie la phase : "normal" sans stress, "recovery" pendant les alpha (30) premiers pas du stress, "durable" ensuite."""
        if not self.state.stress_active:
            return "normal"
        elapsed = step - self.state.stress_t0
        if elapsed <= self.cfg.alpha:
            return "recovery"
        return "durable"
