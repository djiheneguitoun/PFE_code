"""Interface commune aux deux architectures (centralisée et distribuée).

À chaque pas, swarm.py appelle broadcast_beliefs puis plan_step sur l'architecture choisie.
"""
from __future__ import annotations

from typing import List, Protocol


class ArchitecturePlanner(Protocol):
    """Ce que doit savoir faire une architecture : diffuser les cartes, puis faire décider les drones."""

    def broadcast_beliefs(self, active_agents: List, current_step: int) -> None:
        """Envoie les cartes de croyance des drones à ceux qui doivent les recevoir (voisins ou cloud)."""
        ...

    def plan_step(self, active_agents: List, global_fused_belief,
                  phase: str, current_step: int) -> None:
        """Choisit l'action de chaque drone actif pour ce pas et la fait exécuter (ou la met en route)."""
        ...
