"""aif_core : cœur de la simulation par inférence active (AIF) d'un essaim de drones.

Chaque drone tient une carte de croyance (belief.py) et choisit son mouvement (planner.py) ;
swarm.py orchestre l'essaim en architecture centralisée ou distribuée (dossier architecture/).
Utilisé par scripts/12_aif_isaac_sim.py ; explications dans A_LIRE_POUR_LE_PROMOTEUR/README.md.
"""
