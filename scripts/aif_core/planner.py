"""Choix du mouvement de chaque drone par inférence active (AIF), plus une méthode simple de comparaison.

Le drone envisage 9 actions (rester, ou avancer de 1 m dans une des 8 directions) et note chacune par G,
l'« énergie libre attendue » : G baisse si l'action promet d'apprendre beaucoup (curiosité) ou d'approcher
des zones encore incertaines, et monte si elle rapproche d'un autre drone ou d'un obstacle.
L'action est tirée au hasard en favorisant les G bas. Utilisé par architecture/*.py via get_planner.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .belief import BeliefGrid, mix_beliefs
from .math_utils import bernoulli_entropy, softmax_sample


def _build_actions() -> List[Tuple[str, float, float]]:
    """Renvoie les 9 actions (nom, dx, dy) : rester, ou aller vers N, NE, E, SE, S, SW, W, NW (direction de longueur 1)."""
    raw = [
        ("stay", 0.0, 0.0),
        ("N", 0.0, -1.0), ("NE", 1.0, -1.0), ("E", 1.0, 0.0),
        ("SE", 1.0, 1.0), ("S", 0.0, 1.0), ("SW", -1.0, 1.0),
        ("W", -1.0, 0.0), ("NW", -1.0, -1.0),
    ]
    out = []
    for name, dx, dy in raw:
        norm = math.hypot(dx, dy) or 1.0
        out.append((name, dx / norm, dy / norm))
    return out


ACTIONS: List[Tuple[str, float, float]] = _build_actions()   # un déplacement = cfg.step_size (1 m) dans la direction (dx, dy)


def expected_info_gain(wx: float, wy: float, belief: BeliefGrid, cfg) -> float:
    """Estime ce qu'apprendrait un scan lidar fait depuis (wx, wy) : somme, sur 360 rayons simulés, de l'incertitude
    des cases que chaque rayon a des chances d'atteindre (une case probablement occupée arrête le rayon)."""
    total = 0.0
    for angle in cfg.ray_angles:
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        # p_reach : probabilité que le rayon arrive jusqu'à la case sans avoir été arrêté avant
        p_reach = 1.0
        for step in range(1, cfg.max_range_cells + 1):
            cx = wx + step * cfg.grid_resolution * cos_a
            cy = wy + step * cfg.grid_resolution * sin_a
            gx, gy = belief.world_to_grid(cx, cy)
            if not belief.in_bounds(gx, gy):
                break
            p_occ = belief.probability[gy, gx]
            total += p_reach * bernoulli_entropy(p_occ)
            p_reach *= (1.0 - p_occ)
            if p_reach < 1e-4:
                break
    return total


def frontier_attraction(wx: float, wy: float, belief: BeliefGrid) -> float:
    """Renvoie l'attrait des zones incertaines autour de (wx, wy) : moyenne sur 11 × 11 cases de l'incertitude
    de chaque case (1 si p = 0,5 ; 0 si p = 0 ou 1), divisée par (distance + 1)."""
    gx, gy = belief.world_to_grid(wx, wy)
    window = 5
    total, count = 0.0, 0
    for dy in range(-window, window + 1):
        for dx in range(-window, window + 1):
            nx, ny = gx + dx, gy + dy
            if belief.in_bounds(nx, ny):
                p = belief.probability[ny, nx]
                total += (1.0 - abs(2.0 * p - 1.0)) / (math.hypot(dx, dy) + 1.0)
                count += 1
    return total / max(count, 1)


def obstacle_clearance(wx: float, wy: float, belief: BeliefGrid, cfg) -> float:
    """Renvoie une pénalité qui grandit près des cases occupées (rayon clearance_cells : 6 cases = 3 m) ;
    elle fait contourner l'obstacle avec une marge au lieu de le longer ou de l'éviter brutalement."""
    gx, gy = belief.world_to_grid(wx, wy)
    R = int(cfg.clearance_cells)
    pen = 0.0
    for dy in range(-R, R + 1):
        for dx in range(-R, R + 1):
            d = math.hypot(dx, dy)
            if d < 1e-9 or d > R:
                continue
            nx, ny = gx + dx, gy + dy
            if belief.in_bounds(nx, ny) and belief.probability[ny, nx] >= cfg.occ_threshold:
                pen += 1.0 / (d + 0.1)
    return pen


def _planner_bounds(cfg) -> Tuple[float, float, float, float]:
    """Renvoie la zone permise (x0, y0, x1, y1) en m locaux : l'intérieur de l'usine (marge de 2,5 m) s'il est connu,
    sinon toute la zone moins 0,5 m ; empêche les drones d'errer dans la marge des murs."""
    ib = cfg.interior_bounds_local()
    if ib is not None:
        return ib
    return (0.5, 0.5, cfg.env_width - 0.5, cfg.env_height - 0.5)


# Choix d'action AIF : minimiser l'énergie libre attendue G
def select_action(pos_x: float, pos_y: float,
                  others: List[Tuple[float, float]],
                  belief: BeliefGrid,
                  fused: Optional[BeliefGrid],
                  cfg,
                  rng: np.random.Generator,
                  resilience_phase: str = "normal"
                  ) -> Tuple[Tuple[str, float, float], List[Dict], int]:
    """Choisit l'action AIF : calcule G pour chaque mouvement permis, puis en tire un (G bas = plus probable).
    Renvoie (action, diagnostic des 9 candidats, indice choisi)."""
    # Carte de planification : 70 % carte du drone + 30 % carte fusionnée (fusion_mix), en log-odds
    plan_belief = mix_beliefs(belief, fused, cfg.fusion_mix) if fused else belief
    H = plan_belief.mean_entropy()
    bx0, by0, bx1, by1 = _planner_bounds(cfg)
    n = len(ACTIONS)
    # G = 1e6 : action interdite (hors zone ou case occupée), jamais tirée
    G = np.full(n, 1e6)
    valid = np.zeros(n, dtype=bool)
    cand_diag: List[Dict] = []

    for i, (name, dx, dy) in enumerate(ACTIONS):
        nx = pos_x + dx * cfg.step_size
        ny = pos_y + dy * cfg.step_size
        entry: Dict[str, Any] = {
            "idx": i, "name": name,
            "nx": round(nx, 3), "ny": round(ny, 3),
            "valid": False, "reason": "",
            "ig": 0.0, "frontier": 0.0,
            "move": 0.0, "coll": 0.0, "clearance": 0.0, "G": 1e6,
        }
        if not (bx0 <= nx <= bx1 and by0 <= ny <= by1):
            entry["reason"] = "out-of-bounds"
            cand_diag.append(entry)
            continue
        gx, gy = plan_belief.world_to_grid(nx, ny)
        if plan_belief.probability[gy, gx] >= cfg.occ_threshold:
            entry["reason"] = f"occupied p={plan_belief.probability[gy, gx]:.2f}"
            cand_diag.append(entry)
            continue

        valid[i] = True
        ig = expected_info_gain(nx, ny, plan_belief, cfg)
        fr = frontier_attraction(nx, ny, plan_belief)
        move = 0.0 if name == "stay" else 1.0
        coll = sum(
            1.0 / (math.hypot(nx - ox, ny - oy) + 0.1)
            for ox, oy in others if math.hypot(nx - ox, ny - oy) < 8.0
        )
        clr = obstacle_clearance(nx, ny, plan_belief, cfg)

        # G (plus bas = mieux) = - curiosité (ig) - attrait des frontières (fr) + coûts (bouger, drones, obstacles).
        # Les phases "recovery" et "durable" ajoutent des termes liés à l'entropie H (poids w_*_recover, w_*_durable).
        if resilience_phase == "recovery":
            G[i] = (
                - cfg.w_entropy_recover * H
                - cfg.w_innov_recover * ig
                + cfg.w_movement * move
                + cfg.w_collision * coll
                + cfg.w_clearance * clr
                - cfg.w_epistemic * ig
                - cfg.w_pragmatic * fr
            )
        elif resilience_phase == "durable":
            maintain_pen = 0.0
            if H > cfg.H_target:
                maintain_pen += (H - cfg.H_target)
            G[i] = (
                - cfg.w_entropy_durable * H
                - cfg.w_epistemic * ig
                - cfg.w_pragmatic * fr
                + cfg.w_movement * move
                + cfg.w_collision * coll
                + cfg.w_clearance * clr
                + cfg.w_maintain * maintain_pen
            )
        else:
            G[i] = (
                - cfg.w_epistemic * ig
                - cfg.w_pragmatic * fr
                + cfg.w_movement * move
                + cfg.w_collision * coll
                + cfg.w_clearance * clr
            )

        entry.update({
            "valid": True, "reason": "ok",
            "ig": round(ig, 4), "frontier": round(fr, 4),
            "move": round(move, 2), "coll": round(coll, 4),
            "clearance": round(clr, 4),
            "G": round(G[i], 4),
        })
        cand_diag.append(entry)

    # Aucune action permise : on force "rester sur place"
    if not valid.any():
        valid[0] = True
        G[0] = 0.0
        if cand_diag:
            cand_diag[0]["valid"] = True
            cand_diag[0]["reason"] = "forced-stay"
            cand_diag[0]["G"] = 0.0
    idx = softmax_sample(G, cfg.softmax_temp, rng)
    return ACTIONS[idx], cand_diag, idx


# Choix d'action heuristique (méthode simple « frontière », pour comparer avec l'AIF)
HEUR_FREE_THR = 0.4      # probabilité d'occupation sous laquelle une case est jugée libre
HEUR_COLL_RADIUS = 1.5   # m : distance minimale à un autre drone

_HEUR_STATE: Dict[int, Dict[str, Any]] = {}   # direction en cours de chaque drone (clé : id de son générateur aléatoire)


def _count_unknown_neighbors(gx: int, gy: int, belief: BeliefGrid) -> int:
    """Compte les cases inconnues (probabilité entre 0,4 et 0,6) dans le carré 3 × 3 centré sur (gx, gy)."""
    n = 0
    p = belief.probability
    for dy in range(-1, 2):
        for dx in range(-1, 2):
            nx, ny = gx + dx, gy + dy
            if belief.in_bounds(nx, ny):
                pc = p[ny, nx]
                if 0.4 <= pc <= 0.6:
                    n += 1
    return n


def select_action_heuristic(pos_x: float, pos_y: float,
                            others: List[Tuple[float, float]],
                            belief: BeliefGrid,
                            fused: Optional[BeliefGrid],
                            cfg,
                            rng: np.random.Generator,
                            resilience_phase: str = "normal"
                            ) -> Tuple[Tuple[str, float, float], List[Dict], int]:
    """Choisit l'action heuristique : garde sa direction tant qu'elle reste libre, sinon en tire une nouvelle
    (de préférence vers de l'inconnu) ; renvoie le même triplet que select_action."""
    del resilience_phase
    plan_belief = mix_beliefs(belief, fused, cfg.fusion_mix) if fused else belief
    bx0, by0, bx1, by1 = _planner_bounds(cfg)

    key = id(rng)
    state = _HEUR_STATE.get(key)
    if state is None:
        state = {"current_idx": None}
        _HEUR_STATE[key] = state

    cand_diag: List[Dict] = []
    valid_indices: List[int] = []

    for i, (name, dx, dy) in enumerate(ACTIONS):
        nx = pos_x + dx * cfg.step_size
        ny = pos_y + dy * cfg.step_size
        entry: Dict[str, Any] = {
            "idx": i, "name": name,
            "nx": round(nx, 3), "ny": round(ny, 3),
            "valid": False, "reason": "",
            "ig": 0.0, "frontier": 0.0,
            "move": 0.0, "coll": 0.0, "G": 0.0,
        }
        if name == "stay":
            cand_diag.append(entry)
            continue
        if not (bx0 <= nx <= bx1 and by0 <= ny <= by1):
            entry["reason"] = "out-of-bounds"
            cand_diag.append(entry)
            continue
        gx, gy = plan_belief.world_to_grid(nx, ny)
        p = float(plan_belief.probability[gy, gx])
        if p >= cfg.occ_threshold:
            entry["reason"] = f"occupied p={p:.2f}"
            cand_diag.append(entry)
            continue
        if p >= HEUR_FREE_THR:
            entry["reason"] = f"unknown p={p:.2f}"
            cand_diag.append(entry)
            continue
        min_d = math.inf
        for ox, oy in others:
            d = math.hypot(nx - ox, ny - oy)
            if d < min_d:
                min_d = d
        if min_d < HEUR_COLL_RADIUS:
            entry["reason"] = f"neighbor d={min_d:.2f}"
            entry["coll"] = round(1.0 / (min_d + 0.1), 3)
            cand_diag.append(entry)
            continue
        entry["valid"] = True
        entry["reason"] = "ok"
        entry["move"] = 1.0
        cand_diag.append(entry)
        valid_indices.append(i)

    if not valid_indices:
        cand_diag[0].update({"valid": True, "reason": "forced-stay (all blocked)"})
        state["current_idx"] = None
        return ACTIONS[0], cand_diag, 0

    if state["current_idx"] is not None and state["current_idx"] in valid_indices:
        idx = state["current_idx"]
        cand_diag[idx]["reason"] = "keep"
        return ACTIONS[idx], cand_diag, idx

    unknown_touching: List[int] = []
    for i in valid_indices:
        _, dx, dy = ACTIONS[i]
        nx = pos_x + dx * cfg.step_size
        ny = pos_y + dy * cfg.step_size
        gx, gy = plan_belief.world_to_grid(nx, ny)
        unk = _count_unknown_neighbors(gx, gy, plan_belief)
        cand_diag[i]["frontier"] = float(unk)
        if unk > 0:
            unknown_touching.append(i)

    pool = unknown_touching if unknown_touching else valid_indices
    idx = int(rng.choice(pool))
    state["current_idx"] = idx
    cand_diag[idx]["reason"] = "new-direction (unknown)" if unknown_touching else "new-direction (any)"
    return ACTIONS[idx], cand_diag, idx


def get_planner(name: str):
    """Renvoie la fonction de choix d'action : select_action_heuristic si name vaut "heuristic", sinon select_action (AIF)."""
    if name == "heuristic":
        return select_action_heuristic
    return select_action
