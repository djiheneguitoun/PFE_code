"""Tire les configurations d'inventaire : quels cartons portent un QR, dans chaque entrepôt simulé.

Les numéros de configuration sont répartis en trois ensembles figés (entraînement, validation,
test ; Kirk et al. 2023) pour mesurer la généralisation sur des cas jamais vus. Utilisé par env_map.py.
"""

from __future__ import annotations

import torch

from .config_map import LayoutConfig

SPLITS = ("train", "val", "test")  # noms des trois ensembles : entraînement, validation, test


class LayoutGenerator:
    """Générateur reproductible des masques « QR actif » et « QR déjà lu », un tirage par numéro de configuration."""

    def __init__(self, cfg: LayoutConfig, n_tags: int, device: torch.device | str):
        """Réserve des plages de numéros disjointes : 0-499 entraînement, 500-519 validation, 520-539 test."""
        self.cfg = cfg
        self.n_tags = n_tags
        self.device = torch.device(device)
        sizes = {"train": cfg.train_configs, "val": cfg.val_configs, "test": cfg.test_configs}
        start = 0
        self._ranges = {}
        for split in SPLITS:
            self._ranges[split] = (start, start + sizes[split])
            start += sizes[split]

    def config_ids(self, split: str) -> range:
        """Renvoie la plage des numéros de configuration d'un ensemble ("train", "val" ou "test")."""
        lo, hi = self._ranges[split]
        return range(lo, hi)

    def sample_ids(self, n: int, split: str, rng: torch.Generator | None = None) -> torch.Tensor:
        """Tire au hasard `n` numéros de configuration dans l'ensemble demandé."""
        lo, hi = self._ranges[split]
        ids = torch.randint(lo, hi, (n,), generator=rng, device="cpu")
        return ids.to(self.device)

    def masks(self, config_ids: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Renvoie les masques (actif, déjà lu) de forme (nb de numéros, nb de QR) ; même numéro → même tirage."""
        n = config_ids.shape[0]
        active = torch.zeros(n, self.n_tags, dtype=torch.bool, device=self.device)
        preread = torch.zeros(n, self.n_tags, dtype=torch.bool, device=self.device)
        lo_a, hi_a = self.cfg.active_frac
        lo_p, hi_p = self.cfg.preread_frac
        for i, cid in enumerate(config_ids.tolist()):
            # générateur propre à chaque numéro : tirage identique d'une machine à l'autre
            g = torch.Generator(device="cpu").manual_seed(self.cfg.base_seed + int(cid))
            u = torch.rand(2, generator=g)
            frac_a = lo_a + (hi_a - lo_a) * float(u[0])
            frac_p = lo_p + (hi_p - lo_p) * float(u[1])
            perm = torch.randperm(self.n_tags, generator=g)
            n_active = max(1, int(round(frac_a * self.n_tags)))
            act_idx = perm[:n_active]
            active[i, act_idx.to(self.device)] = True
            n_pre = int(round(frac_p * n_active))
            if n_pre > 0:
                preread[i, act_idx[:n_pre].to(self.device)] = True
        return active, preread
