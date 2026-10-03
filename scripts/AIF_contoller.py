"""Premier prototype du contrôleur d'inférence active (AIF), sur une grille abstraite et sans simulateur.

Chaque agent mesure la distance aux obstacles dans 4 directions (N, E, S, W), met à jour sa carte d'occupation,
les cartes sont fusionnées, puis chaque agent choisit l'une des 9 actions en minimisant une « énergie libre attendue » G
(calculs parallélisés, un fil par agent). Ancêtre de aif_core/ (mêmes paramètres).
Usage : controller = ThreadedLiDARController(w, h, n_agents, positions) puis actions = controller.step(observations).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional, Sequence
from concurrent.futures import ThreadPoolExecutor

# ============================================================
# Basic math utils
# ============================================================

L_MAX = 30.0  # borne des log-odds : évite des probabilités exactement égales à 0 ou 1

def clamp(x: float, lo: float, hi: float) -> float:
    """Renvoie x ramené dans l'intervalle [lo, hi]."""
    return max(lo, min(hi, x))

def bernoulli_entropy(p: float) -> float:
    """Renvoie l'incertitude (entropie, en nats) d'une case occupée avec la probabilité p : 0 si sûre, ln 2 ≈ 0,69 si p = 0,5."""
    p = clamp(p, 1e-6, 1 - 1e-6)
    return -(p * math.log(p) + (1 - p) * math.log(1 - p))

def logit(p: float) -> float:
    """Convertit une probabilité en log-odds ln(p / (1 − p))."""
    p = clamp(p, 1e-6, 1 - 1e-6)
    return math.log(p / (1 - p))

def inv_logit(l: float) -> float:
    """Convertit des log-odds en probabilité (calcul stable même pour de grandes valeurs)."""
    if l >= 0:
        z = math.exp(-l)
        return 1.0 / (1.0 + z)
    else:
        z = math.exp(l)
        return z / (1.0 + z)

def softmax_pick(candidates, T: float, rng: random.Random):
    """Tire au hasard une action parmi `candidates` (nom, dx, dy, G) avec une probabilité ∝ exp(−G/T) :
    plus G est bas, plus l'action a de chances d'être choisie ; renvoie l'action tirée."""
    T = max(T, 1e-6)
    m = min(c[3] for c in candidates)  # stability shift
    weights = [math.exp(-(c[3] - m) / T) for c in candidates]
    s = sum(weights)
    r = rng.random() * s
    acc = 0.0
    for c, w in zip(candidates, weights):
        acc += w
        if acc >= r:
            return c
    return candidates[-1]

# ============================================================
# Directions and actions
# ============================================================

# 4 directions du LiDAR simplifié : décalage (dx, dy) en cases ; y augmente vers le sud
DIRS4 = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}
DIR_LIST = ["N", "E", "S", "W"]

# 9 actions : rester sur place ou aller dans l'une des 8 cases voisines
ACTIONS = [
    ("stay", 0, 0),
    ("N", 0, -1),
    ("NE", 1, -1),
    ("E", 1, 0),
    ("SE", 1, 1),
    ("S", 0, 1),
    ("SW", -1, 1),
    ("W", -1, 0),
    ("NW", -1, -1),
]

# ============================================================
# Belief map
# ============================================================

class BeliefMap:
    """Carte d'occupation d'un agent : probabilité que chaque case soit un obstacle, gardée aussi en log-odds."""
    def __init__(self, w: int, h: int, p0: float = 0.5):
        """Crée une carte de w × h cases valant toutes la probabilité a priori p0 (0,5 = inconnu)."""
        self.w = w
        self.h = h
        self.p0 = p0
        self.l0 = logit(p0)
        self.p = [[p0 for _ in range(w)] for _ in range(h)]
        self.l = [[self.l0 for _ in range(w)] for _ in range(h)]

    def prob(self, x: int, y: int) -> float:
        """Renvoie la probabilité d'obstacle de la case (x, y)."""
        return self.p[y][x]

    def update_logodds(self, x: int, y: int, delta_l: float):
        """Ajoute delta_l aux log-odds de la case (borné à ±L_MAX) et met à jour sa probabilité."""
        self.l[y][x] = clamp(self.l[y][x] + delta_l, -L_MAX, L_MAX)
        self.p[y][x] = inv_logit(self.l[y][x])

    def mean_entropy(self) -> float:
        """Renvoie l'entropie moyenne de la carte (incertitude moyenne par case)."""
        tot = 0.0
        n = self.w * self.h
        for y in range(self.h):
            for x in range(self.w):
                tot += bernoulli_entropy(self.p[y][x])
        return tot / n

# ============================================================
# Belief fusion / mixing
# ============================================================

def fuse_beliefs(beliefs: List[BeliefMap], method: str = "logodds", weights=None) -> BeliefMap:
    """Fusionne plusieurs cartes en une seule : moyenne pondérée des log-odds (« logodds ») ou des probabilités (« avg »)."""
    assert len(beliefs) > 0
    w, h = beliefs[0].w, beliefs[0].h
    fused = BeliefMap(w, h, p0=beliefs[0].p0)
    l0 = fused.l0

    if weights is None:
        weights = [1.0] * len(beliefs)
    s = sum(weights)
    weights = [wi / s for wi in weights]

    if method == "avg":
        for y in range(h):
            for x in range(w):
                p = sum(b.p[y][x] * wi for b, wi in zip(beliefs, weights))
                fused.p[y][x] = p
                fused.l[y][x] = clamp(logit(p), -L_MAX, L_MAX)
        return fused

    if method != "logodds":
        raise ValueError(f"Unknown fusion method: {method}")

    for y in range(h):
        for x in range(w):
            l_sum = l0
            for b, wi in zip(beliefs, weights):
                l_sum += wi * (b.l[y][x] - l0)
            l_sum = clamp(l_sum, -L_MAX, L_MAX)
            fused.l[y][x] = l_sum
            fused.p[y][x] = inv_logit(l_sum)

    return fused

def mix_beliefs(local_b: BeliefMap, fused_b: BeliefMap, lam: float) -> BeliefMap:
    """Renvoie le mélange (1 − lam) × carte locale + lam × carte fusionnée, calculé en log-odds."""
    out = BeliefMap(local_b.w, local_b.h, p0=local_b.p0)
    for y in range(out.h):
        for x in range(out.w):
            l = (1 - lam) * local_b.l[y][x] + lam * fused_b.l[y][x]
            l = clamp(l, -L_MAX, L_MAX)
            out.l[y][x] = l
            out.p[y][x] = inv_logit(l)
    return out

# ============================================================
# Epistemic proxy
# ============================================================

def local_ray_expected_ig(bmap: BeliefMap, ax: int, ay: int, max_range: int) -> float:
    """Estime l'information à gagner depuis (ax, ay) : somme des entropies des cases que les 4 rayons ont des chances d'atteindre."""
    tot = 0.0
    for d in DIR_LIST:
        dx, dy = DIRS4[d]
        x, y = ax, ay
        p_reach = 1.0
        for _ in range(max_range):
            x += dx
            y += dy
            if x < 0 or y < 0 or x >= bmap.w or y >= bmap.h:
                break
            Hc = bernoulli_entropy(bmap.prob(x, y))
            tot += p_reach * Hc
            p_free = 1.0 - bmap.prob(x, y)
            p_reach *= p_free
            if p_reach < 1e-4:
                break
    return tot

# ============================================================
# Feasibility proxy (belief-based)
# ============================================================

def can_step_from_belief(
    belief: BeliefMap,
    x: int,
    y: int,
    dx: int,
    dy: int,
    occ_thr: float = 0.65,
) -> bool:
    """Renvoie vrai si la case visée est dans la grille et probablement libre (p ≤ occ_thr), coins compris en diagonale."""
    nx, ny = x + dx, y + dy
    if nx < 0 or ny < 0 or nx >= belief.w or ny >= belief.h:
        return False
    if belief.prob(nx, ny) > occ_thr:
        return False
    if abs(dx) == 1 and abs(dy) == 1:
        if belief.prob(x + dx, y) > occ_thr or belief.prob(x, y + dy) > occ_thr:
            return False
    return True

# ============================================================
# Agent (controller-side)
# ============================================================

class Agent:
    """Agent côté contrôleur : identifiant fixe (= rang dans les observations et les actions), position en cases, hasard propre."""
    def __init__(self, agent_id: int, x: int, y: int, max_range: int, seed: int):
        """Crée l'agent n° agent_id en (x, y), avec une portée LiDAR de max_range cases et sa graine aléatoire."""
        self.agent_id = int(agent_id)  # identifiant stable de l'agent
        self.x = x
        self.y = y
        self.max_range = max_range
        self.rng = random.Random(seed)

    def lidar_predict_from_belief(self, belief: BeliefMap) -> List[float]:
        """Renvoie, pour N, E, S, W, la probabilité prévue par la carte que le rayon touche un obstacle (le bord compte comme obstacle)."""
        preds = []
        for d in DIR_LIST:
            dx, dy = DIRS4[d]
            x, y = self.x, self.y
            p_no_hit = 1.0
            for _ in range(self.max_range):
                x += dx
                y += dy
                if x < 0 or y < 0 or x >= belief.w or y >= belief.h:
                    p_no_hit *= 0.0
                    break
                p_no_hit *= (1.0 - belief.prob(x, y))
            preds.append(1.0 - p_no_hit)
        return preds

    def apply_action_assuming_env_executes(self, dx: int, dy: int, w: int, h: int):
        """Déplace l'agent de (dx, dy) sans sortir de la grille, en supposant que l'environnement exécute bien l'action."""
        nx = clamp(self.x + dx, 0, w - 1)
        ny = clamp(self.y + dy, 0, h - 1)
        self.x, self.y = int(nx), int(ny)

# ============================================================
# Mapping update from external LiDAR distances
# ============================================================

def inverse_sensor_update_distance(
    belief: BeliefMap,
    agent: Agent,
    dists: Sequence[int],
    lo_free: float,
    lo_occ: float,
):
    """Met à jour la carte avec les 4 distances mesurées : cases traversées plus libres (lo_free), case touchée plus occupée (lo_occ)."""
    ax, ay = agent.x, agent.y
    for i, dname in enumerate(DIR_LIST):
        dx, dy = DIRS4[dname]
        dist = int(dists[i])
        x, y = ax, ay

        free_steps = min(dist - 1, agent.max_range)
        for _ in range(free_steps):
            x += dx
            y += dy
            if 0 <= x < belief.w and 0 <= y < belief.h:
                belief.update_logodds(x, y, lo_free)

        if dist <= agent.max_range:
            x += dx
            y += dy
            if 0 <= x < belief.w and 0 <= y < belief.h:
                belief.update_logodds(x, y, lo_occ)

# ============================================================
# Planning / resilience params
# ============================================================

@dataclass
class Params:
    """Poids de l'énergie libre G par phase et cibles de récupération (entropie H_target = 0,44 ; innovation innov_target = 0,16)."""
    w_epistemic: float = 2.5
    w_entropy_recover: float = 3.0
    w_innov_recover: float = 1.2
    w_move: float = 0.0
    w_deadline: float = 12.0
    w_entropy_durable: float = 1.2
    w_innov_durable: float = 0.8
    w_churn_durable: float = 2.2
    w_maintain: float = 10.0
    H_target: float = 0.44
    innov_target: float = 0.16

@dataclass
class ResilienceState:
    """État de résilience : stress en cours ou non, step de début, step de récupération, nombre de steps stables depuis."""
    stress_active: bool = False
    stress_t0: int = -1
    recovered_at: int = -1
    durable_count: int = 0

def plan_action_belief_only(
    belief: BeliefMap,
    agent: Agent,
    res: ResilienceState,
    t: int,
    alpha: int,
    beta: int,
    params: Params,
    horizon: int = 2,
    softmax_T: float = 0.25,
    rng: Optional[random.Random] = None,
    occ_thr: float = 0.65,
) -> Tuple[str, int, int, float]:
    """Évalue les 9 actions sur `horizon` steps avec une énergie libre G propre à la phase (normale, récupération pendant
    alpha steps, durable pendant beta steps), puis en tire une par softmax ; renvoie (nom, dx, dy, G)."""
    if rng is None:
        rng = agent.rng

    step_rel0 = (t - res.stress_t0) if res.stress_active else 10**9
    candidates: List[Tuple[str, int, int, float]] = []

    for name, dx, dy in ACTIONS:
        if can_step_from_belief(belief, agent.x, agent.y, dx, dy, occ_thr=occ_thr):
            nx, ny = agent.x + dx, agent.y + dy
        else:
            nx, ny = agent.x, agent.y

        total_G = 0.0
        ax, ay = nx, ny

        for h in range(horizon):
            step_rel = step_rel0 + h

            ig_proxy = local_ray_expected_ig(belief, ax, ay, agent.max_range)
            epistemic_bonus = params.w_epistemic * ig_proxy

            H = belief.mean_entropy()

            tmp_agent = Agent(agent_id=-1, x=ax, y=ay, max_range=agent.max_range, seed=0)
            pred = tmp_agent.lidar_predict_from_belief(belief)
            innov_proxy = sum(p * (1 - p) for p in pred) / 4.0
            churn_proxy = innov_proxy

            move_cost = 0.0 if name == "stay" else 1.0

            if res.stress_active and step_rel <= alpha:
                G = (
                    params.w_entropy_recover * H
                    + params.w_innov_recover * innov_proxy
                    + params.w_move * move_cost
                    - epistemic_bonus
                )
                if step_rel == alpha and H > params.H_target:
                    G += params.w_deadline * (H - params.H_target)

            elif res.stress_active and (alpha < step_rel <= alpha + beta):
                maintain_pen = 0.0
                if H > params.H_target:
                    maintain_pen += (H - params.H_target)
                if innov_proxy > params.innov_target:
                    maintain_pen += (innov_proxy - params.innov_target)

                G = (
                    params.w_entropy_durable * H
                    + params.w_innov_durable * innov_proxy
                    + params.w_churn_durable * churn_proxy
                    + params.w_move * move_cost
                    + params.w_maintain * maintain_pen
                    - epistemic_bonus
                )
            else:
                G = (0.6 * H + 0.2 * innov_proxy + params.w_move * move_cost - epistemic_bonus)

            total_G += G

            if can_step_from_belief(belief, ax, ay, dx, dy, occ_thr=occ_thr):
                ax, ay = ax + dx, ay + dy

        total_G += rng.uniform(-1e-4, 1e-4)
        candidates.append((name, dx, dy, total_G))

    return softmax_pick(candidates, T=softmax_T, rng=rng)

# ============================================================
# Controller
# ============================================================

Obs4 = Tuple[int, int, int, int]  # observation : distances N, E, S, W (cases)
Pos2 = Tuple[int, int]            # position (x, y) en cases
Act2 = Tuple[int, int]            # action : déplacement (dx, dy)

class ThreadedLiDARController:
    """Contrôleur multi-agents : reçoit les 4 distances (N, E, S, W) de chaque agent et renvoie un déplacement (dx, dy) par agent, dans le même ordre."""

    def __init__(
        self,
        w: int,
        h: int,
        n_agents: int,
        initial_positions: Tuple[Pos2, ...],
        max_range: int = 6,
        fusion: str = "logodds",
        plan_mix: float = 0.3,
        softmax_T: float = 0.25,
        horizon: int = 2,
        alpha: int = 200,
        beta: int = 600,
        seed: int = 1,
        workers: int = 0,
        lo_free: float = -0.55,
        lo_occ: float = 0.85,
        occ_thr_plan: float = 0.65,
        params: Optional[Params] = None,
    ):
        """Crée les agents et leurs cartes, la carte fusionnée, les statistiques d'innovation et les fils d'exécution (un par agent par défaut)."""
        if len(initial_positions) != n_agents:
            raise ValueError("initial_positions must have length n_agents")

        self.w = w
        self.h = h
        self.n_agents = n_agents

        self.max_range = max_range
        self.fusion = fusion
        self.plan_mix = plan_mix
        self.softmax_T = softmax_T
        self.horizon = horizon
        self.alpha = alpha
        self.beta = beta

        self.lo_free = lo_free
        self.lo_occ = lo_occ
        self.occ_thr_plan = occ_thr_plan

        self.params = params if params is not None else Params()
        self.res = ResilienceState()

        self.t = 0

        self.agents: List[Agent] = []
        self.beliefs: List[BeliefMap] = []

        for i, (x, y) in enumerate(initial_positions):
            self.agents.append(Agent(agent_id=i, x=x, y=y, max_range=max_range, seed=seed * 1000 + i))
            self.beliefs.append(BeliefMap(w, h, p0=0.5))

        self.fused_belief = fuse_beliefs(
            self.beliefs,
            method=self.fusion,
            weights=[1.0 / self.n_agents] * self.n_agents
        )

        self.innov_ema = 0.0
        self.innov_var = 0.0
        self.ema_alpha = 0.05
        self.k_sigma = 2.0

        max_workers = workers if workers and workers > 0 else n_agents
        self._pool = ThreadPoolExecutor(max_workers=max_workers)

    @property
    def agent_ids(self) -> Tuple[int, ...]:
        """Renvoie les identifiants des agents, dans l'ordre des observations et des actions."""
        return tuple(a.agent_id for a in self.agents)

    def close(self):
        """Arrête les fils d'exécution (à appeler en fin d'utilisation)."""
        self._pool.shutdown(wait=True)

    def set_positions(self, positions: Tuple[Pos2, ...]) -> None:
        """Remplace les positions internes par les vraies positions de l'environnement ; à appeler avant step()
        si un déplacement peut être bloqué (sinon le contrôleur suppose que chaque action a été exécutée)."""
        if len(positions) != self.n_agents:
            raise ValueError("positions must have length n_agents")
        for i, (x, y) in enumerate(positions):
            self.agents[i].x = int(clamp(x, 0, self.w - 1))
            self.agents[i].y = int(clamp(y, 0, self.h - 1))

    def _update_one_agent(self, i: int, fused_snapshot: BeliefMap, obs_i: Sequence[int]) -> float:
        """Met à jour la carte de l'agent i avec ses 4 distances et renvoie son innovation (écart moyen entre contacts prévus et mesurés)."""
        agent = self.agents[i]
        b = self.beliefs[i]

        pred_probs = agent.lidar_predict_from_belief(fused_snapshot)
        innov_i = sum(
            abs((int(d) <= agent.max_range) - p)
            for d, p in zip(obs_i, pred_probs)
        ) / 4.0

        inverse_sensor_update_distance(b, agent, obs_i, self.lo_free, self.lo_occ)
        return innov_i

    def _plan_one_agent(self, i: int, fused_snapshot: BeliefMap) -> Act2:
        """Choisit l'action de l'agent i sur un mélange de sa carte (70 %) et de la carte fusionnée (30 %) ; renvoie (dx, dy)."""
        agent = self.agents[i]
        plan_belief = mix_beliefs(self.beliefs[i], fused_snapshot, lam=self.plan_mix)

        _name, dx, dy, _G = plan_action_belief_only(
            belief=plan_belief,
            agent=agent,
            res=self.res,
            t=self.t,
            alpha=self.alpha,
            beta=self.beta,
            params=self.params,
            horizon=self.horizon,
            softmax_T=self.softmax_T,
            rng=agent.rng,
            occ_thr=self.occ_thr_plan,
        )
        return (dx, dy)

    def step(self, observations: Tuple[Obs4, ...]) -> Tuple[Act2, ...]:
        """Fait un pas : mises à jour en parallèle, fusion, détection d'un pic d'innovation (stress), planification en parallèle ; renvoie les actions."""
        if len(observations) != self.n_agents:
            raise ValueError("observations must have length n_agents")
        for obs in observations:
            if len(obs) != 4:
                raise ValueError("each observation must be 4 distances for (N,E,S,W)")

        # A) parallel belief updates
        fused_snapshot = self.fused_belief
        futs = [
            self._pool.submit(self._update_one_agent, i, fused_snapshot, observations[i])
            for i in range(self.n_agents)
        ]
        innovations = [f.result() for f in futs]
        innov_mean = sum(innovations) / max(1, len(innovations))

        # Moyenne et variance glissantes de l'innovation : « pic » si elle dépasse la moyenne de k_sigma écarts-types
        err = innov_mean - self.innov_ema
        self.innov_ema += self.ema_alpha * err
        self.innov_var += self.ema_alpha * ((err * err) - self.innov_var)
        sigma = math.sqrt(max(self.innov_var, 1e-8))
        spike = innov_mean > (self.innov_ema + self.k_sigma * sigma)

        # fuse
        self.fused_belief = fuse_beliefs(
            self.beliefs,
            method=self.fusion,
            weights=[1.0 / self.n_agents] * self.n_agents
        )
        Hmean = self.fused_belief.mean_entropy()

        # B) optional stress logic
        if not self.res.stress_active and spike:
            self.res.stress_active = True
            self.res.stress_t0 = self.t
            self.res.recovered_at = -1
            self.res.durable_count = 0

        if self.res.stress_active:
            recovered_now = (Hmean <= self.params.H_target) and (innov_mean <= 0.30)
            if self.res.recovered_at < 0:
                if recovered_now:
                    self.res.recovered_at = self.t
                    self.res.durable_count = 0
            else:
                if recovered_now:
                    self.res.durable_count += 1
                else:
                    self.res.durable_count = 0
                if self.res.durable_count >= self.beta:
                    self.res.stress_active = False

        # C) parallel planning
        fused_snapshot2 = self.fused_belief
        futs = [self._pool.submit(self._plan_one_agent, i, fused_snapshot2) for i in range(self.n_agents)]
        actions_list = [f.result() for f in futs]
        actions: Tuple[Act2, ...] = tuple(actions_list)

        # D) update internal poses (assumes env executes actions)
        for i, (dx, dy) in enumerate(actions):
            self.agents[i].apply_action_assuming_env_executes(dx, dy, self.w, self.h)

        self.t += 1
        return actions


# ============================================================
# Example
# ============================================================

if __name__ == "__main__":
    controller = ThreadedLiDARController(
        w=10, h=10,
        n_agents=2,
        initial_positions=((2, 2), (7, 7)),
        max_range=6,
        seed=1,
    )

    print("agent ids:", controller.agent_ids)  # (0, 1)

    obs = ((3, 7, 2, 5), (6, 1, 6, 2))
    actions = controller.step(obs)
    print("actions:", actions)

    controller.close()