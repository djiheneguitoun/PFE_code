"""Réseau acteur-critique pour rsl_rl : un CNN lit les cartes, un MLP lit le vecteur d'état.

L'acteur (qui choisit l'action) ne voit JAMAIS le groupe « privileged » ; seul le critique (qui
estime la valeur, utile à l'entraînement) le voit (CTDE : entraînement centralisé, exécution
décentralisée). Utilisé par train.py, eval_map.py, verify_env.py et record_traj.py.
"""

from __future__ import annotations

import torch
import torch.nn as nn

from rsl_rl.modules import ActorCritic


class _MapNet(nn.Module):
    """Réseau qui coupe l'entrée en [cartes | reste] : CNN sur les cartes, puis MLP sur le tout."""

    def __init__(self, map_channels: int, map_px: int, extra_dim: int, out_dim: int, hidden: tuple[int, ...]):
        """Construit le CNN (3 convolutions, résumé de 256 valeurs) et le MLP (couches `hidden`) vers `out_dim`."""
        super().__init__()
        self.map_dim = map_channels * map_px * map_px
        self.shape = (map_channels, map_px, map_px)
        self.cnn = nn.Sequential(
            nn.Conv2d(map_channels, 32, 3, stride=2, padding=1), nn.ELU(),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ELU(),
            nn.Conv2d(64, 64, 3, stride=2, padding=1), nn.ELU(),
            nn.Flatten(),
            nn.Linear(64 * (map_px // 8) ** 2, 256), nn.ELU(),
        )
        layers, prev = [], 256 + extra_dim
        for h in hidden:
            layers += [nn.Linear(prev, h), nn.ELU()]
            prev = h
        layers.append(nn.Linear(prev, out_dim))
        self.head = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Renvoie la sortie du réseau pour un lot d'observations aplaties [cartes | reste]."""
        maps = x[..., : self.map_dim].reshape(-1, *self.shape)
        extra = x[..., self.map_dim:]
        return self.head(torch.cat([self.cnn(maps), extra], dim=-1))


class MapActorCritic(ActorCritic):
    """Acteur-critique compatible rsl_rl (même interface qu'ActorCritic) dont l'acteur et le critique sont des _MapNet."""

    is_recurrent = False  # pas de mémoire interne (la mémoire, c'est la carte)

    def __init__(
        self,
        obs,
        obs_groups,
        num_actions,
        map_channels: int = 18,          # 2 échelles × 9 couches de carte
        map_px: int = 32,                # côté des vues en cases
        actor_hidden_dims=(256, 128),
        critic_hidden_dims=(256, 128),
        init_noise_std: float = 0.5,
        **kwargs,
    ):
        """Crée l'acteur, le critique et l'écart-type σ des 4 actions (appris, initialisé à `init_noise_std`)."""
        # on saute le constructeur d'ActorCritic (il créerait ses propres MLP) : tout est recréé ici
        nn.Module.__init__(self)
        self.obs_groups = obs_groups
        map_dim = map_channels * map_px * map_px

        actor_dim = sum(obs[g].shape[-1] for g in obs_groups["policy"])
        critic_dim = sum(obs[g].shape[-1] for g in obs_groups["critic"])
        self.actor = _MapNet(map_channels, map_px, actor_dim - map_dim, num_actions, tuple(actor_hidden_dims))
        self.critic = _MapNet(map_channels, map_px, critic_dim - map_dim, 1, tuple(critic_hidden_dims))
        # dernière couche de l'acteur presque nulle : au départ, actions moyennes proches de 0
        nn.init.uniform_(self.actor.head[-1].weight, -1e-3, 1e-3)
        nn.init.zeros_(self.actor.head[-1].bias)

        # pas de normalisation automatique des observations (env_map.py les met déjà à l'échelle)
        self.actor_obs_normalization = False
        self.critic_obs_normalization = False
        self.actor_obs_normalizer = nn.Identity()
        self.critic_obs_normalizer = nn.Identity()

        self.noise_std_type = "scalar"
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution = None
        torch.distributions.Normal.set_default_validate_args(False)

    def _bounded_mean(self, obs) -> torch.Tensor:
        """Renvoie la moyenne des actions, bornée en douceur à ±3 par 3·tanh(x/3)."""
        # Sans borne, une mise à jour violente éjectait les moyennes loin de [−1, 1], où elles
        # restaient coincées (3 entraînements perdus). Presque l'identité sur [−1, 1] (écart max
        # 3,5 %), et le gradient n'est jamais nul.
        return 3.0 * torch.tanh(self.actor(obs) / 3.0)

    def update_distribution(self, obs):
        """Prépare la loi normale des actions : moyenne bornée, écart-type σ limité à [0,05 ; 1,2]."""
        mean = self._bounded_mean(obs)
        # σ plafonné : sans borne, une mauvaise mise à jour + le bonus d'entropie ont fait exploser
        # σ jusqu'à 9 (pathologie n°3 de l'audit IPPO, corrigée ici par un plafond)
        std = self.std.clamp(0.05, 1.2).expand_as(mean)
        self.distribution = torch.distributions.Normal(mean, std)

    def act_inference(self, obs):
        """Renvoie l'action déterministe (la moyenne bornée, sans bruit) pour l'évaluation."""
        obs = self.get_actor_obs(obs)
        obs = self.actor_obs_normalizer(obs)
        return self._bounded_mean(obs)
