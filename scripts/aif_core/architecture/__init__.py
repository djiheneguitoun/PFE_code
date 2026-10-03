"""Les deux façons d'organiser la décision de l'essaim, choisies avec l'option --arch.

- CloudPlanner (centralized.py) : un serveur central (le « cloud ») décide pour tous les drones.
- DistributedPlanner (distributed.py) : chaque drone décide seul, avec les cartes de ses voisins.
"""
from .base import ArchitecturePlanner
from .centralized import CloudPlanner
from .distributed import DistributedPlanner

__all__ = ["ArchitecturePlanner", "CloudPlanner", "DistributedPlanner"]
