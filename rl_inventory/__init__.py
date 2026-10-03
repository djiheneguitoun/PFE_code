"""Paquet rl_inventory : essai d'apprentissage par renforcement (RL) pour l'inventaire QR par un essaim de drones.

Importer le paquet enregistre la tâche gym « Isaac-QR-Inventory-Swarm-Direct-v0 », utilisée par train_ppo.py et eval.py.
env.py n'est chargé qu'au gym.make, après le démarrage d'Isaac Sim : l'import reste léger (config_rl.py se lit sans simulateur).
Explications : A_LIRE_POUR_LE_PROMOTEUR/README.md ; conception : CONCEPTION_controleur_RL_inventaire.md.
"""

import gymnasium as gym

from . import agents

# Tâche essaim : classe d'environnement, classe de réglages et les deux fichiers de réglages skrl (IPPO, MAPPO).
gym.register(
    id="Isaac-QR-Inventory-Swarm-Direct-v0",
    entry_point="rl_inventory.env:SwarmQREnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "rl_inventory.env:SwarmQREnvCfg",
        "skrl_ippo_cfg_entry_point": f"{agents.__name__}:skrl_ippo_cfg.yaml",
        "skrl_mappo_cfg_entry_point": f"{agents.__name__}:skrl_mappo_cfg.yaml",
    },
)
