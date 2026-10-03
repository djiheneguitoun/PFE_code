"""Adapte l'environnement à 3 drones au format rsl_rl : 3 drones × B entrepôts = 3B « pseudo-environnements ».

PS-PPO (PPO à paramètres partagés) : un seul réseau commun voit ainsi 3 fois plus de données.
Découpe aussi l'observation en trois groupes (maps, vector, privileged) pour rsl_rl 3.0.
Utilisé par train.py, eval_map.py, verify_env.py et record_traj.py.
"""

from __future__ import annotations

import torch
from tensordict import TensorDict

from rsl_rl.env import VecEnv

from ..env import AGENTS
from .config_map import MAP_CFG
from .env_map import PRIV_DIM, VEC_DIM, SwarmScanMapEnv

MAP_DIM = MAP_CFG.map.obs_dim  # taille de la partie « cartes » de l'observation (18 432 valeurs)


class SwarmMapVecEnv(VecEnv):
    """Vue rsl_rl de l'essaim : le pseudo-env i est le drone i // B dans l'entrepôt i % B."""

    def __init__(self, env: SwarmScanMapEnv):
        """Garde l'environnement et calcule le nombre de pseudo-envs (3 × B) et d'actions (4)."""
        self.env = env
        self.B = env.num_envs
        self.num_envs = self.B * len(AGENTS)
        self.num_actions = env.cfg.action_spaces[AGENTS[0]]
        self.device = env.device
        self.cfg = {}
        self._obs = None

    @property
    def max_episode_length(self) -> int:
        """Renvoie la durée maximale d'un épisode en pas (elle grandit avec le niveau du curriculum)."""
        return int(self.env.max_episode_length)  # dynamique : le budget d'épisode croît avec le niveau

    @property
    def episode_length_buf(self) -> torch.Tensor:
        """Renvoie le compteur de pas de chaque pseudo-env (celui de son entrepôt, répété 3 fois)."""
        return self.env.episode_length_buf.repeat(len(AGENTS))

    @episode_length_buf.setter
    def episode_length_buf(self, value: torch.Tensor):
        """Écrit le compteur de pas des entrepôts à partir des B premières valeurs."""
        self.env.episode_length_buf = value[: self.B]

    def _pack(self, obs_dict: dict) -> TensorDict:
        """Empile les observations des 3 drones et les coupe en groupes maps / vector / privileged."""
        flat = torch.cat([obs_dict[a] for a in AGENTS], dim=0)
        return TensorDict(
            {
                "maps": flat[:, :MAP_DIM],
                "vector": flat[:, MAP_DIM : MAP_DIM + VEC_DIM],
                "privileged": flat[:, MAP_DIM + VEC_DIM : MAP_DIM + VEC_DIM + PRIV_DIM],
            },
            batch_size=[self.num_envs],
        )

    def get_observations(self) -> TensorDict:
        """Renvoie les dernières observations (réinitialise l'environnement au premier appel)."""
        if self._obs is None:
            obs_dict, _ = self.env.reset()
            self._obs = self._pack(obs_dict)
        return self._obs

    def step(self, actions: torch.Tensor) -> tuple[TensorDict, torch.Tensor, torch.Tensor, dict]:
        """Applique les actions des 3B pseudo-envs ; renvoie observations, récompenses, fins et infos de journal."""
        act = {a: actions[i * self.B : (i + 1) * self.B] for i, a in enumerate(AGENTS)}
        obs_dict, rew, terminated, truncated, extras = self.env.step(act)
        self._obs = self._pack(obs_dict)
        rewards = torch.cat([rew[a] for a in AGENTS], dim=0)
        time_outs = torch.cat([truncated[a] for a in AGENTS], dim=0)
        dones = torch.cat([(terminated[a] | truncated[a]) for a in AGENTS], dim=0)
        infos = {"time_outs": time_outs}
        if self.env.map_log:  # émis UNE fois quand frais (répété, il biaisait les moyennes TensorBoard)
            infos["log"] = dict(self.env.map_log)
            self.env.map_log = {}
        return self._obs, rewards, dones.float(), infos

    def reset(self):
        """Réinitialise tous les entrepôts et renvoie les observations groupées."""
        obs_dict, _ = self.env.reset()
        self._obs = self._pack(obs_dict)
        return self._obs

    def close(self):
        """Ferme l'environnement Isaac."""
        self.env.close()
