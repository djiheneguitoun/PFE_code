"""Petites fonctions mathématiques communes aux modules AIF.

Passage probabilité ↔ log-odds (autre écriture d'une probabilité : 0 = 50 %, positif = plutôt occupé,
négatif = plutôt libre), entropie d'une case (son incertitude) et tirage au hasard d'une action (softmax).
"""
from __future__ import annotations

import math

import numpy as np


def clamp(x: float, lo: float, hi: float) -> float:
    """Renvoie x ramené dans l'intervalle [lo, hi]."""
    return max(lo, min(hi, x))


def logit(p: float) -> float:
    """Convertit une probabilité en log-odds, en évitant 0 et 1 (qui donneraient l'infini)."""
    p = clamp(p, 1e-7, 1 - 1e-7)
    return math.log(p / (1 - p))


def inv_logit(l: float) -> float:
    """Convertit un log-odds en probabilité (calcul stable même pour de très grandes valeurs)."""
    if l >= 0:
        return 1.0 / (1.0 + math.exp(-l))
    return math.exp(l) / (1.0 + math.exp(l))


def inv_logit_v(arr: np.ndarray) -> np.ndarray:
    """Convertit un tableau de log-odds en probabilités (valeurs bornées à ±30 avant le calcul)."""
    return 1.0 / (1.0 + np.exp(-np.clip(arr, -30, 30)))


def bernoulli_entropy(p: float) -> float:
    """Renvoie l'entropie (incertitude, en nats) d'une case occupée avec la probabilité p : 0,69 à p = 0,5 ; 0 à p = 0 ou 1."""
    p = clamp(p, 1e-7, 1 - 1e-7)
    return -(p * math.log(p) + (1 - p) * math.log(1 - p))


def bernoulli_entropy_v(arr: np.ndarray) -> np.ndarray:
    """Renvoie l'entropie (en nats) de chaque case d'un tableau de probabilités."""
    p = np.clip(arr, 1e-7, 1 - 1e-7)
    return -(p * np.log(p) + (1 - p) * np.log(1 - p))


def softmax_sample(values: np.ndarray, temperature: float,
                   rng: np.random.Generator) -> int:
    """Tire au hasard l'indice d'une action : plus sa valeur (G) est basse, plus elle a de chances d'être tirée.
    Température basse = la meilleure action est presque toujours choisie."""
    temperature = max(temperature, 1e-6)
    shifted = -(values - np.min(values)) / temperature
    weights = np.exp(shifted - np.max(shifted))
    probs = weights / weights.sum()
    return int(rng.choice(len(values), p=probs))
