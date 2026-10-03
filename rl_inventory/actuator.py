"""Modèle d'actionneur du drone : l'action fixe la vitesse visée, la vitesse réelle la rejoint avec retard et accélération bornée.

Pur torch, sans Isaac (testable sans simulateur) ; utilisé par SwarmQREnv._drive dans env.py.
Pourquoi : sans lui, le drone passait de 0 à 1,5 m/s en un pas (45 m/s² = 4,59 g, alors qu'un Crazyflie plafonne à 3,57 m/s²)
et 87,9 % des lectures d'une politique aléatoire suivaient une survitesse (détails : A_LIRE_POUR_LE_PROMOTEUR/RESULTATS.md).
"""

from __future__ import annotations

import torch


def integrate(vel, cmd, dt, *, accel_xy, accel_z, yaw_accel, tau_xy, tau_z, tau_yaw):
    """Avance la vitesse vel (N, 4) = (vx, vy, vz, lacet), repère monde, d'un pas dt vers la consigne cmd ; renvoie la nouvelle vitesse.
    L'accélération horizontale est bornée en norme (elle ne dépend que de l'inclinaison du drone) ; tau = temps de réponse (s)."""
    out = vel.clone()
    acc_xy = (cmd[:, :2] - vel[:, :2]) / tau_xy
    # facteur ≤ 1 qui ramène la norme de l'accélération horizontale à accel_xy au plus
    scale = (accel_xy / acc_xy.norm(dim=1, keepdim=True).clamp_min(1e-9)).clamp(max=1.0)
    out[:, :2] = vel[:, :2] + acc_xy * scale * dt
    out[:, 2] = vel[:, 2] + ((cmd[:, 2] - vel[:, 2]) / tau_z).clamp(-accel_z, accel_z) * dt
    out[:, 3] = vel[:, 3] + ((cmd[:, 3] - vel[:, 3]) / tau_yaw).clamp(-yaw_accel, yaw_accel) * dt
    return out


def altitude_envelope(vz, z, alt_min, alt_max, accel_z, max_delta=None):
    """Limite vz pour qu'un freinage à accel_z arrête le drone avant alt_min / alt_max ; renvoie le vz corrigé (m/s).
    S'applique à l'état, pas à la consigne ; max_delta borne la correction (sans lui : 0,71 g mesuré pour un modèle limité à 0,37 g)."""
    # vitesse max qui permet encore de s'arrêter avant le plafond (up) ou le plancher (down) : v = √(2·a·distance)
    up = torch.sqrt(2.0 * accel_z * (alt_max - z).clamp_min(0.0))
    down = torch.sqrt(2.0 * accel_z * (z - alt_min).clamp_min(0.0))
    out = torch.maximum(torch.minimum(vz, up), -down)
    if max_delta is not None:
        out = vz + (out - vz).clamp(-max_delta, max_delta)
    return out
