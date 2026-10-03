"""Environnement SwarmScan-Map (Isaac Lab, conception_solution_finale.md) : 3 drones par entrepôt apprennent à lire tous les QR.

Hérite de SwarmQREnv (../env.py : scène, drones, lidar, pilotage) et remplace l'observation (cartes
+ vecteur, jamais la position d'un QR non lu), la récompense, la règle de lecture à crans, les
départs aléatoires et les pannes ; corrige aussi des bugs d'audit (lidar égocentrique, visée par
drone, crédit tiré au hasard, à-coups mesurés avant écrêtage). Utilisé via flatten_wrapper.py.
"""

from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass
from isaaclab.utils.math import euler_xyz_from_quat
from isaaclab.utils.warp import raycast_mesh

from ..config_rl import CFG
from ..env import AGENTS, MAX_DIST, NUM_DRONES, SwarmQREnv, SwarmQREnvCfg
from ..qr_task import FACES
from .config_map import MAP_CFG
from .curriculum import GateCurriculum
from .layouts import LayoutGenerator
from .mapping import SwarmMapper

D = NUM_DRONES  # nombre de drones par entrepôt (3)
# Taille du vecteur d'état d'un drone (98) : 72 secteurs lidar + vitesse (3) + rotation (1)
# + cap sin/cos (2) + altitude (1) + temps écoulé (1) + 2 coéquipiers × (position relative 3
# + en vie 1) + panne (1) + 2 indicateurs « assez lent » + 4 seuils de lecture + identité (D).
# Identité du drone (one-hot) : les 3 drones partagent UN seul réseau ; sans identifiant, ils
# prenaient les mêmes décisions et se regroupaient au même endroit. Mesuré : pénalité de
# proximité 10× plus lourde dans les bons épisodes que dans les mauvais, et couverture
# d'équipe négative (travail fait en triple).
VEC_DIM = MAP_CFG.train.lidar_sectors + 3 + 1 + 2 + 1 + 1 + (D - 1) * 4 + 1 + 2 + 4 + D
PRIV_DIM = 7  # infos privilégiées, vues par le critique seul (voir priv dans _get_observations)
OBS_DIM = MAP_CFG.map.obs_dim + VEC_DIM + PRIV_DIM  # observation complète d'un drone : 18 432 + 98 + 7


@configclass
class SwarmScanMapEnvCfg(SwarmQREnvCfg):
    """Réglages Isaac de l'environnement : durée d'épisode, taille des observations, altitude max 4 m."""

    episode_length_s = MAP_CFG.train.episode_length_s  # s : 150 s au cran 0 (allongée ensuite à chaque cran)
    observation_spaces = {a: OBS_DIM for a in AGENTS}
    state_space = 0  # pas d'état global séparé : le critique reçoit le groupe « privileged »
    alt_max: float = 4.0  # m : lecture à 1,25 m max, il faut monter pour lire les QR hauts


class SwarmScanMapEnv(SwarmQREnv):
    """Environnement à 3 drones par entrepôt : carte partagée, règle de lecture à crans, pannes simulées."""

    cfg: SwarmScanMapEnvCfg

    def __init__(self, cfg: SwarmScanMapEnvCfg, render_mode: str | None = None, **kwargs):
        """Crée la carte partagée, le curriculum et tous les tampons (par entrepôt et par drone)."""
        super().__init__(cfg, render_mode, **kwargs)
        m = MAP_CFG
        self._mapper = SwarmMapper(m.map, self.num_envs, D, self.device)
        self._curr = GateCurriculum(m.gate, m.curriculum)
        self._split = "train"          # ensemble de configurations utilisé : "train", "val" ou "test"
        self._layouts: LayoutGenerator | None = None
        self._config_ids = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        B = self.num_envs
        self._prev_raw = {a: torch.zeros(B, CFG.action.dim, device=self.device) for a in AGENTS}
        self._jerk_pre = {a: torch.zeros(B, device=self.device) for a in AGENTS}
        self._bound_pen = {a: torch.zeros(B, device=self.device) for a in AGENTS}
        self._dead = torch.zeros(B, D, dtype=torch.bool, device=self.device)               # drone en panne
        self._kill_step = torch.full((B,), -1, dtype=torch.long, device=self.device)       # pas de la panne (−1 : aucune)
        self._victim = torch.zeros(B, dtype=torch.long, device=self.device)                # drone qui tombera en panne
        self._milestones = torch.zeros(B, len(m.reward.milestones), dtype=torch.bool, device=self.device)
        self._read_frac_readable = torch.zeros(B, device=self.device)
        self._phi = torch.zeros(B, D, device=self.device)
        self._phi_fresh = torch.ones(B, dtype=torch.bool, device=self.device)
        self._nearest = torch.full((B, D), m.reward.shaping_clip_m, device=self.device)
        self._bearing = torch.zeros(B, D, 2, device=self.device)
        self._scan_ok = torch.zeros(B, D, dtype=torch.bool, device=self.device)
        self._yaw_ok = torch.zeros(B, D, dtype=torch.bool, device=self.device)
        self._new_reads_nom = torch.zeros(D, B, device=self.device)
        self._lin_k = torch.zeros(B, D, device=self.device)
        self._yawrate_k = torch.zeros(B, D, device=self.device)
        self._alg_ref = None           # posé par train.py : bascule d'entropie en cours d'entraînement au niveau cible
        self._switch_level = 10        # niveau qui déclenche la bascule (train.py le remplace par --converge_level)
        self._switch_entropy = 0.001   # nouveau coefficient d'entropie (train.py : --entropy_final)
        self._switched = False
        self._scan_counts = {k: torch.zeros(B, D, device=self.device) for k in ("new", "facade", "marginal", "overlap")}
        self._pos_local = torch.zeros(B, D, 3, device=self.device)
        self._yaw = torch.zeros(B, D, device=self.device)
        self._raw_min = torch.full((B, D), MAX_DIST, device=self.device)
        # diagnostic : chaque poste de récompense et chaque grandeur de comportement, cumulés
        # par épisode. Sans ça, un épisode à 0,9 et un à 0,1 sont indiscernables dans les courbes.
        self._rw_acc = {k: torch.zeros(B, device=self.device)
                        for k in ("couverture", "lectures", "chocs", "potentiel", "commande", "separation")}
        self._beh_acc = {k: torch.zeros(B, device=self.device)
                         for k in ("altitude", "vitesse", "contacts", "distance", "pas")}
        self._prev_xy = None
        self._agent_id = torch.eye(D, device=self.device).unsqueeze(1)   # (D,1,D) one-hot par drone
        self.map_log: dict[str, float] = {}

    # ------------------------------------------------------------------ QR & layouts

    def _ensure_qr(self):
        """Au premier pas : lit les QR (classe mère), repère ceux qui sont atteignables et tire les configurations."""
        super()._ensure_qr()
        n = self._n_cartons
        z = self._qr_pos_local.view(n, len(FACES), 3)[:, 0, 2]
        self._reachable = z <= MAP_CFG.layout.max_tag_z_m
        self._tag_xy = self._qr_pos_local.view(n, len(FACES), 3).mean(dim=1)[:, :2]
        self._layouts = LayoutGenerator(MAP_CFG.layout, n, self.device)
        self._config_ids = self._layouts.sample_ids(self.num_envs, self._split)
        self._active, self._preread = self._layouts.masks(self._config_ids)
        self._read |= self._preread
        # QR lisible = porte un QR actif, assez bas pour être atteint, et pas déjà lu au départ
        self._readable = self._active & self._reachable.unsqueeze(0) & ~self._preread
        # compteurs de visée (pas consécutifs) PAR DRONE, pour la règle courante et la nominale
        self._dwell_k = torch.zeros(D, self.num_envs, n, dtype=torch.int16, device=self.device)
        self._dwell_nom = torch.zeros(D, self.num_envs, n, dtype=torch.int16, device=self.device)
        self._read_nominal = torch.zeros(self.num_envs, n, dtype=torch.bool, device=self.device)

    def _resample_layout(self, env_ids: torch.Tensor):
        """Tire de nouvelles configurations pour les entrepôts `env_ids` et remet leurs compteurs de lecture à zéro."""
        ids = self._layouts.sample_ids(len(env_ids), self._split)
        self._config_ids[env_ids] = ids
        active, preread = self._layouts.masks(ids)
        self._active[env_ids] = active
        self._preread[env_ids] = preread
        self._read[env_ids] = preread
        self._readable[env_ids] = active & self._reachable.unsqueeze(0) & ~preread
        self._dwell_k[:, env_ids] = 0
        self._dwell_nom[:, env_ids] = 0
        self._read_nominal[env_ids] = False

    # ------------------------------------------------------------------ gate (ADR + nominal)

    def _update_qr(self):
        """Applique la règle de lecture à chaque pas : crédite les nouveaux QR lus et calcule le potentiel d'aide."""
        if not self._qr_ready:
            self._ensure_qr()
        th = self._curr.thresholds()
        g = MAP_CFG.gate
        B, n = self.num_envs, self._n_cartons
        qr_pos = self._qr_pos_local.unsqueeze(0) + self.scene.env_origins.unsqueeze(1)
        cos_fov = math.cos(math.radians(g.camera_fov_deg))

        vis_cur = torch.zeros(D, B, n, dtype=torch.bool, device=self.device)
        vis_nom = torch.zeros(D, B, n, dtype=torch.bool, device=self.device)
        dist_tag = torch.full((D, B, n), 1e6, device=self.device)
        for k in range(D):
            d = self._drones[k].data
            vec = qr_pos - d.root_pos_w.unsqueeze(1)
            dist = torch.norm(vec, dim=-1).clamp_min(1e-6)
            direction = vec / dist.unsqueeze(-1)
            # DEUX caméras LATÉRALES (±90° du cap) : le drone lit en LONGEANT les racks, sans viser
            # finement avec le cap (en frontal, la visée ne marchait que grâce au bruit : 0,91 avec
            # bruit → 0,11 sans ; en latéral, tout est lisible géométriquement : 1,0).
            # Axe gauche = (−sin, cos, 0), axe droit = son opposé : un seul test en valeur absolue.
            side = torch.stack([-torch.sin(self._yaw[:, k]), torch.cos(self._yaw[:, k]),
                                torch.zeros_like(self._yaw[:, k])], dim=-1)
            in_fov = (direction * side.unsqueeze(1)).sum(-1).abs() >= cos_fov
            face_cos = (-direction * self._qr_normal.unsqueeze(0)).sum(-1)
            lin = torch.norm(
                d.root_lin_vel_w if g.speed_uses_vz else d.root_lin_vel_w[:, :2], dim=-1
            )
            yr = torch.abs(d.root_ang_vel_w[:, 2])
            alive = ~self._dead[:, k]
            self._lin_k[:, k] = lin
            self._yawrate_k[:, k] = yr
            self._scan_ok[:, k] = (lin <= th["max_speed_mps"]) & alive
            self._yaw_ok[:, k] = (yr <= th["max_yawrate_rps"]) & alive

            def gate(dist_max, ang_deg, v_max, yr_max):
                """Renvoie, par entrepôt et par QR, si ce drone le lit avec ces seuils (distance, angle, vitesse, rotation)."""
                ok = (dist <= dist_max) & in_fov & (face_cos >= math.cos(math.radians(ang_deg)))
                ok = ok.view(B, n, len(FACES)).any(-1)
                slow = (lin <= v_max) & (yr <= yr_max) & alive
                return ok & slow.unsqueeze(1)

            # vis_cur : règle du niveau courant ; vis_nom : règle nominale (suivie à part pour le bonus)
            vis_cur[k] = gate(th["read_distance_m"], th["view_angle_deg"],
                              th["max_speed_mps"], th["max_yawrate_rps"])
            vis_nom[k] = gate(g.read_distance_m, g.view_angle_deg, g.max_speed_mps, g.max_yawrate_rps)
            dist_tag[k] = dist.view(B, n, len(FACES)).min(-1).values

        # au départ la vitesse est nulle : sans ce délai (15 pas), un drone placé exprès près d'un
        # QR le lisait gratuitement dès le premier pas (géométrie et vitesse déjà bonnes)
        live = (self.episode_length_buf >= g.credit_blackout_steps).view(1, B, 1)
        vis_cur &= live
        vis_nom &= live

        self._dwell_k = torch.where(vis_cur, self._dwell_k + 1, torch.zeros_like(self._dwell_k))
        self._dwell_nom = torch.where(vis_nom, self._dwell_nom + 1, torch.zeros_like(self._dwell_nom))

        # seuls les QR LISIBLES comptent (_readable et non _active) : avant, un QR trop haut payait
        # 25 points sans jamais compter dans le total de la mission ni du curriculum
        ready = (self._dwell_k >= int(th["dwell_steps"])) & self._readable.unsqueeze(0) & (~self._read).unsqueeze(0)
        ready_nom = (self._dwell_nom >= g.dwell_steps) & self._readable.unsqueeze(0) & (~self._read_nominal).unsqueeze(0)

        self._new_reads = torch.zeros(D, B, device=self.device)
        self._new_reads_nom = torch.zeros(D, B, device=self.device)
        credited = torch.zeros(B, n, dtype=torch.bool, device=self.device)
        credited_nom = torch.zeros(B, n, dtype=torch.bool, device=self.device)
        # ordre des drones tiré au hasard : un QR vu par deux drones est crédité à un seul,
        # sans favoriser toujours le même
        for k in torch.randperm(D).tolist():
            cred = ready[k] & ~credited
            credited |= cred
            self._new_reads[k] = cred.sum(dim=1).float()
            # posture nominale = événement SÉPARÉ, payé 1 fois par QR et par épisode : la règle
            # tolérante lit avant que la posture nominale soit tenue 2 pas, donc un bonus lié à
            # la lecture serait inatteignable
            cred_n = ready_nom[k] & ~credited_nom
            credited_nom |= cred_n
            self._new_reads_nom[k] = cred_n.sum(dim=1).float()
        self._read_nominal |= credited_nom
        newly = credited
        self._read |= newly

        denom = self._readable.sum(dim=1).clamp_min(1)
        self._read_frac_readable = ((self._read & self._readable).sum(dim=1).float()) / denom.float()
        self._read_frac = self._read_frac_readable  # utilisé par la classe mère (fin d'épisode, journaux)

        env_idx, tag_idx = torch.nonzero(newly, as_tuple=True)
        if env_idx.numel() > 0:
            self._mapper.mark_read(env_idx, self._tag_xy[tag_idx])

        clip = MAP_CFG.reward.shaping_clip_m
        # cible du potentiel : les QR non lus avec la règle du niveau COURANT. Viser la posture
        # nominale à tous les niveaux a été testé : double travail infaisable en 120 s, le
        # curriculum se bloquait. Aux derniers crans, c'est la règle elle-même qui l'exige.
        unread = self._readable & ~self._read
        far = dist_tag.masked_fill(~unread.unsqueeze(0), 1e6)
        nearest, arg = far.min(dim=2)
        self._nearest = nearest.clamp(max=clip).transpose(0, 1)
        has_unread = unread.any(dim=1)
        # potentiel : se rapprocher d'un QR non lu rapporte, et être lent PRÈS de lui aussi (guide
        # vers la posture de lecture). Lenteur mesurée par rapport à la limite du niveau COURANT :
        # par rapport au nominal 0,6 m/s, le gradient était NUL entre 0,6 et 1,5 m/s, justement
        # la zone à apprendre.
        v_ref = CFG.action.max_lin_vel_mps
        slowness = ((v_ref - self._lin_k) / max(v_ref - th["max_speed_mps"], 1e-3)).clamp(0.0, 1.0)
        proximity = torch.exp(-(self._nearest - g.read_distance_m).clamp(min=0.0) / 1.5)
        phi_val = -self._nearest / clip + MAP_CFG.reward.slow_potential * slowness * proximity
        phi = torch.where(has_unread.unsqueeze(1), phi_val, torch.zeros_like(phi_val))
        # direction (sin, cos, par rapport au cap) du QR non lu le plus proche : info privilégiée
        for k in range(D):
            tag_pos = self._tag_xy[arg[k]]
            delta = tag_pos + self.scene.env_origins[:, :2] - self._drones[k].data.root_pos_w[:, :2]
            ang = torch.atan2(delta[:, 1], delta[:, 0]) - self._yaw[:, k]
            self._bearing[:, k, 0] = torch.sin(ang)
            self._bearing[:, k, 1] = torch.cos(ang)
        self._phi_now = phi

    # ------------------------------------------------------------------ perception

    def _raycast_full(self, k: int):
        """Lance les 1800 rayons du lidar du drone k ; renvoie distances (m), points d'impact (repère entrepôt) et masque d'impact."""
        if self._wh_mesh is None:
            from isaaclab.sensors import MultiMeshRayCaster
            self._wh_mesh = next(iter(MultiMeshRayCaster.meshes.values()))
        origins = self.scene.env_origins
        r = self._ray_dirs.shape[0]
        base = (self._drones[k].data.root_pos_w + self._ray_off).unsqueeze(1)
        # un seul maillage d'entrepôt (celui de env_0) : chaque drone y est ramené par translation
        starts = (base - origins.unsqueeze(1) + origins[0]).expand(self.num_envs, r, 3).contiguous()
        dirs = self._ray_dirs.unsqueeze(0).expand(self.num_envs, r, 3).contiguous()
        hits = raycast_mesh(starts, dirs, max_dist=MAX_DIST, mesh=self._wh_mesh)[0]
        dist = torch.norm(hits - starts, dim=-1)
        dist = torch.nan_to_num(dist, nan=MAX_DIST, posinf=MAX_DIST).clamp(0.0, MAX_DIST)
        valid = dist < MAX_DIST * 0.999
        hits_local = torch.where(valid.unsqueeze(-1), hits - origins[0], torch.zeros_like(hits))
        return dist, hits_local, valid

    def _ego_sectors(self, dist: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        """Réduit les 1800 rayons à 72 secteurs de 5° tournés avec le drone (distance mini ÷ 8 m, entre 0 et 1)."""
        B = dist.shape[0]
        per_az = dist.view(B, CFG.lidar.num_channels, 360).min(dim=1).values
        yaw_deg = torch.rad2deg(yaw).round().long() % 360
        idx = (torch.arange(360, device=self.device).unsqueeze(0) + yaw_deg.unsqueeze(1)) % 360
        ego = per_az.gather(1, idx)
        S = MAP_CFG.train.lidar_sectors
        return ego.view(B, S, 360 // S).min(dim=2).values / MAX_DIST

    def _refresh(self):
        """Calcule une seule fois par pas : lidar, cap, position, collisions, puis (en plein pas) lecture QR et carte."""
        if self._refreshed:
            return
        self._lidar_cache, self._state_cache = [], []
        hits_all, valid_all = [], []
        for k in range(D):
            d = self._drones[k].data
            _, _, yaw = euler_xyz_from_quat(d.root_quat_w)
            self._yaw[:, k] = yaw
            self._pos_local[:, k] = d.root_pos_w - self.scene.env_origins
            dist, hits_local, valid = self._raycast_full(k)
            self._raw_min[:, k] = dist.min(dim=1).values
            self._collision[k] = self._raw_min[:, k] < MAP_CFG.reward.collision_distance_m
            self._lidar_min[k] = self._raw_min[:, k]
            self._lidar_cache.append(self._ego_sectors(dist, yaw))
            hits_all.append(hits_local)
            valid_all.append(valid)
            self._state_cache.append(
                torch.cat([d.root_pos_w - self.scene.env_origins, d.root_quat_w,
                           d.root_lin_vel_b, d.root_ang_vel_b], dim=-1)
            )
        if self._stepping:
            self._update_qr()
            m = MAP_CFG.map
            # couverture INDÉPENDANTE de la règle de lecture (portée et vitesses fixes). Liée à elle,
            # le revenu de couverture fondait avec le niveau et dépasser la vitesse limite supprimait
            # la pénalité de recouvrement : ne pas lire devenait rentable (blocage aux niveaux 5-6)
            scan_gate = (self._lin_k <= m.scan_speed_mps) & (self._yawrate_k <= m.scan_yawrate_rps)
            self._scan_counts = self._mapper.update(
                pos=self._pos_local, yaw=self._yaw,
                hits=torch.stack(hits_all, dim=1), hit_valid=torch.stack(valid_all, dim=1),
                scan_ok=scan_gate, scan_range=m.scan_range_m,
                fov_deg=MAP_CFG.gate.camera_fov_deg, alive=~self._dead,
            )
        self._refreshed = True

    # ------------------------------------------------------------------ boucle RL

    def _pre_physics_step(self, actions: dict) -> None:
        """Déclenche les pannes prévues, mesure à-coups et dépassements de [−1, 1], puis borne les actions des 3 drones."""
        self._stepping = True
        self._refreshed = False
        # panne : le drone « victime » s'arrête au pas tiré au reset (entre 25 % et 75 % de l'épisode)
        kill = (self._kill_step >= 0) & (self.episode_length_buf >= self._kill_step)
        newly_dead = kill & ~self._dead[torch.arange(self.num_envs, device=self.device), self._victim]
        if newly_dead.any():
            envs = torch.nonzero(newly_dead).squeeze(-1)
            self._dead[envs, self._victim[envs]] = True
            self._phi_fresh[envs] = True
        for i, a in enumerate(AGENTS):
            raw = actions[a].clone().clamp(-3.0, 3.0)  # garde-fou : pénalités bornées, aucun emballement possible
            # à-coups et dépassement mesurés AVANT l'écrêtage à [−1, 1] (sinon ils seraient invisibles)
            self._jerk_pre[a] = ((raw - self._prev_raw[a]) ** 2).sum(-1)
            self._bound_pen[a] = ((raw.abs() - 1.0).clamp(min=0.0) ** 2).sum(-1)
            self._prev_raw[a] = raw
            act = raw.clamp(-1.0, 1.0)
            norm_xy = act[:, :2].norm(dim=1, keepdim=True).clamp(min=1.0)
            act[:, :2] = act[:, :2] / norm_xy  # vitesse HORIZONTALE ≤ 1,5 m/s (bornée axe par axe, la diagonale montait à 2,12)
            act[self._dead[:, i]] = 0.0
            self._vel_state[i, self._dead[:, i]] = 0.0   # drone en panne : vitesse remise à zéro, sinon il dérive ~1 s
            self._actions[a] = act

    def _get_observations(self) -> dict:
        """Construit l'observation de chaque drone : ses vues de la carte + son vecteur d'état + les infos privilégiées."""
        self._refresh()
        maps = self._mapper.ego_maps(self._pos_local, self._yaw)
        th = self._curr.thresholds()
        # seuils de lecture du niveau courant, mis à l'échelle (÷ 4 m, 60°, 2 m/s, 2 rad/s)
        th_vec = torch.tensor(
            [th["read_distance_m"] / 4.0, th["view_angle_deg"] / 60.0,
             th["max_speed_mps"] / 2.0, th["max_yawrate_rps"] / 2.0],
            device=self.device,
        ).expand(self.num_envs, 4)
        t_frac = (self.episode_length_buf.float() / self.max_episode_length).unsqueeze(-1)
        alive_frac = (~self._dead).float().mean(dim=1, keepdim=True)
        unread_frac = 1.0 - self._read_frac_readable.unsqueeze(-1)
        obs = {}
        for k, a in enumerate(AGENTS):
            d = self._drones[k].data
            mates = []
            for j in range(D):
                if j != k:
                    rel = (self._drones[j].data.root_pos_w - d.root_pos_w) / 10.0   # position relative en dizaines de m
                    mates.append(torch.cat([rel, (~self._dead[:, j]).float().unsqueeze(-1)], dim=-1))
            # vecteur d'état (VEC_DIM = 98 valeurs), vu par l'acteur et le critique
            vec = torch.cat(
                [
                    self._lidar_cache[k],
                    d.root_lin_vel_b / CFG.action.max_lin_vel_mps,
                    (d.root_ang_vel_b[:, 2] / CFG.action.max_yaw_rate_rps).unsqueeze(-1),
                    torch.sin(self._yaw[:, k]).unsqueeze(-1), torch.cos(self._yaw[:, k]).unsqueeze(-1),
                    (self._pos_local[:, k, 2] / self.cfg.alt_max).unsqueeze(-1),
                    t_frac,
                    *mates,
                    self._dead[:, k].float().unsqueeze(-1),
                    self._scan_ok[:, k].float().unsqueeze(-1),
                    self._yaw_ok[:, k].float().unsqueeze(-1),
                    th_vec,
                    self._agent_id[k].expand(self.num_envs, D),
                ],
                dim=-1,
            )
            # infos privilégiées (PRIV_DIM = 7), vues par le critique SEUL : fraction lue, fraction
            # restante, potentiel, direction et distance du QR non lu le plus proche, part de drones en vie
            priv = torch.cat(
                [
                    self._read_frac_readable.unsqueeze(-1),
                    unread_frac,
                    self._phi[:, k].unsqueeze(-1),
                    self._bearing[:, k],
                    (self._nearest[:, k] / MAP_CFG.reward.shaping_clip_m).unsqueeze(-1),
                    alive_frac,
                ],
                dim=-1,
            )
            obs[a] = torch.cat([maps[:, k], vec, priv], dim=-1)
        return obs

    def _get_rewards(self) -> dict:
        """Calcule la récompense de chaque drone (couverture, lectures, chocs, potentiel, commande, séparation) ; 0 s'il est en panne."""
        self._refresh()
        rw = MAP_CFG.reward
        gamma = MAP_CFG.train.gamma
        norm = rw.scan_norm
        cov_low = (self._read_frac_readable < rw.overlap_off_coverage).float()

        milestone_bonus = torch.zeros(self.num_envs, device=self.device)
        for i, ms in enumerate(rw.milestones):
            crossed = (self._read_frac_readable >= ms) & ~self._milestones[:, i]
            self._milestones[:, i] |= crossed
            milestone_bonus += crossed.float() * rw.milestone_bonus

        # aide « potentiel » γΦ' − Φ : paie le progrès vers un QR non lu entre deux pas ; mise à 0
        # juste après un reset ou une panne (l'ancien Φ n'a alors plus de sens)
        phi_now = getattr(self, "_phi_now", self._phi)
        shaping = gamma * phi_now - self._phi
        shaping[self._phi_fresh] = 0.0
        self._phi = phi_now
        self._phi_fresh[:] = False

        # avec les progrès du curriculum (t de 0 à 1) : la couverture paie moins, la lecture plus
        t = self._curr.progress
        cov_scale = 1.0 - rw.coverage_fade * t
        read_scale = 1.0 + rw.read_gain_rise * t
        pos_xy = self._pos_local[..., :2]
        out = {}
        for k, a in enumerate(AGENTS):
            alive = (~self._dead[:, k]).float()
            couverture = (
                cov_scale * rw.facade_gain * self._scan_counts["facade"][:, k] / norm
                + cov_scale * rw.area_gain
                * (self._scan_counts["new"][:, k] - self._scan_counts["facade"][:, k]) / norm
                + cov_scale * rw.marginal_gain * self._scan_counts["marginal"][:, k] / norm
                - rw.overlap_penalty * self._scan_counts["overlap"][:, k] / norm * cov_low
            )
            lectures = (read_scale * rw.new_qr * self._new_reads[k]
                        + read_scale * rw.new_qr_nominal * self._new_reads_nom[k]
                        + milestone_bonus / D)   # jalon d'ÉQUIPE : payé 1× au total, pas 1× par drone
            # chocs : pénalité qui croît sous 0,5 m d'un obstacle, plus une pénalité de contact sous 0,25 m
            gap = ((rw.safe_distance_m - self._raw_min[:, k]) / rw.safe_distance_m).clamp(0.0, 1.0)
            chocs = -(rw.collision_scale * gap * gap
                      + rw.contact_penalty * (self._raw_min[:, k] < rw.collision_distance_m).float())
            self._rw_acc["couverture"] += couverture * alive
            self._rw_acc["lectures"] += lectures * alive
            self._rw_acc["chocs"] += chocs * alive
            self._rw_acc["potentiel"] += rw.shaping_scale * shaping[:, k] * alive
            self._rw_acc["commande"] -= (rw.action_diff * self._jerk_pre[a]
                                         + rw.bound_penalty * self._bound_pen[a]) * alive
            r = couverture + lectures + chocs
            r += rw.shaping_scale * shaping[:, k]
            r -= rw.action_diff * self._jerk_pre[a]
            r -= rw.bound_penalty * self._bound_pen[a]
            # séparation : pénalité graduée si deux drones en vie sont à moins de 0,6 m (à plat)
            for j in range(D):
                if j != k:
                    sep = torch.norm(pos_xy[:, k] - pos_xy[:, j], dim=-1)
                    prox = ((rw.separation_m - sep) / rw.separation_m).clamp(0.0, 1.0)
                    prox = prox * (~self._dead[:, j] & ~self._dead[:, k]).float()
                    pen = rw.separation_penalty * prox * prox
                    r -= pen
                    self._rw_acc["separation"] -= pen * alive
            out[a] = r * alive
        # comportement : ce qui distingue un bon épisode d'un mauvais
        self._beh_acc["altitude"] += self._pos_local[..., 2].mean(dim=1)
        self._beh_acc["vitesse"] += self._lin_k.mean(dim=1)
        self._beh_acc["contacts"] += (self._raw_min < rw.collision_distance_m).float().sum(dim=1)
        if self._prev_xy is not None:
            self._beh_acc["distance"] += (pos_xy - self._prev_xy).norm(dim=-1).sum(dim=1)
        self._prev_xy = pos_xy.clone()
        self._beh_acc["pas"] += 1.0
        return out

    def _get_dones(self) -> tuple[dict, dict]:
        """Renvoie (terminé, tronqué) : mission réussie (95 % des QR lisibles lus) ou temps écoulé."""
        self._refresh()
        mission = self._read_frac_readable >= MAP_CFG.train.mission_target
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return ({a: mission for a in AGENTS}, {a: time_out for a in AGENTS})

    # ------------------------------------------------------------------ resets

    def _reset_idx(self, env_ids):
        """Réinitialise les entrepôts finis : bilan d'épisode, curriculum, carte, pannes, configuration et départs."""
        if env_ids is None:
            env_ids = self._drones[0]._ALL_INDICES
        if self._qr_ready and len(env_ids) > 0:
            # bilan des épisodes qui se terminent : curriculum, bascule d'entropie, journal (map_log)
            fracs = self._read_frac_readable[env_ids].tolist()
            self._curr.on_episodes_end(fracs)
            if (self._alg_ref is not None and not self._switched
                    and self._curr.level >= self._switch_level):
                self._alg_ref.entropy_coef = self._switch_entropy
                self._switched = True
                print(f"\n>>> BASCULE CONVERGENCE : niveau {self._curr.level} atteint → "
                      f"entropy_coef={self._switch_entropy} (le bruit se retire)\n")
            nom = self._read_nominal[env_ids] & self._readable[env_ids]
            denom = self._readable[env_ids].sum(dim=1).clamp_min(1).float()
            pas = self._beh_acc["pas"][env_ids].clamp_min(1.0)
            self.map_log = {
                "curriculum/level": float(self._curr.level),
                "curriculum/ema": self._curr.ema,
                "curriculum/dropout_p": self._curr.dropout_prob(),
                "episode/read_frac_gate": sum(fracs) / max(1, len(fracs)),
                "episode/read_frac_nominal": (nom.sum(dim=1).float() / denom).mean().item(),
                "comportement/altitude_m": (self._beh_acc["altitude"][env_ids] / pas).mean().item(),
                "comportement/vitesse_mps": (self._beh_acc["vitesse"][env_ids] / pas).mean().item(),
                "comportement/distance_m": self._beh_acc["distance"][env_ids].mean().item(),
                "comportement/contacts_pct": (self._beh_acc["contacts"][env_ids] / (pas * D)).mean().item() * 100,
                "comportement/couverture_cellules": self._mapper.scan[env_ids].amax(dim=1).flatten(1).sum(-1).mean().item(),
                **{f"recompense/{k}": v[env_ids].mean().item() for k, v in self._rw_acc.items()},
            }
        super()._reset_idx(env_ids)
        self._refreshed = False

        for a in AGENTS:
            self._prev_actions[a][env_ids] = 0.0
            self._prev_raw[a][env_ids] = 0.0
            self._jerk_pre[a][env_ids] = 0.0
            self._bound_pen[a][env_ids] = 0.0
        self._dead[env_ids] = False
        self._milestones[env_ids] = False
        self._phi[env_ids] = 0.0
        self._phi_fresh[env_ids] = True
        self._read_frac_readable[env_ids] = 0.0
        for acc in (self._rw_acc, self._beh_acc):
            for v in acc.values():
                v[env_ids] = 0.0
        self._prev_xy = None
        self._mapper.reset(env_ids)

        # panne éventuelle (entraînement seulement) : un drone tiré au hasard, à un pas tiré entre
        # 25 % et 75 % de l'épisode
        p_drop = self._curr.dropout_prob() if self._split == "train" else 0.0
        roll = torch.rand(len(env_ids), device=self.device)
        lo, hi = int(0.25 * self.max_episode_length), int(0.75 * self.max_episode_length)
        kill = torch.randint(lo, hi, (len(env_ids),), device=self.device)
        self._kill_step[env_ids] = torch.where(roll < p_drop, kill, torch.full_like(kill, -1))
        self._victim[env_ids] = torch.randint(0, D, (len(env_ids),), device=self.device)

        if self._qr_ready:
            self._resample_layout(env_ids)
            self._randomize_spawns(env_ids)

        # durée d'épisode croissante avec le niveau (règle plus stricte = mission plus longue) :
        # 150 s + 15 s par cran, plafond 295 s
        c = MAP_CFG.curriculum
        self.cfg.episode_length_s = min(
            MAP_CFG.train.episode_length_s + c.episode_s_per_notch * self._curr.level, c.episode_s_max
        )
        # Correction du bug des lectures jamais payées : sans cette ligne, l'observation calculée
        # après le reset relançait _update_qr (les récompenses étaient déjà calculées AVANT le
        # reset), donc les QR lus au départ étaient comptés sans jamais être payés.
        self._stepping = False

    def _randomize_spawns(self, env_ids: torch.Tensor):
        """Place chaque drone au hasard, ou près d'un QR non lu (flanc tourné vers lui), loin des obstacles."""
        if self._wh_mesh is None:
            return
        n = len(env_ids)
        m = MAP_CFG.map
        x0, x1 = m.bounds_x_m[0] + 1.0, m.bounds_x_m[1] - 1.0
        y0, y1 = m.bounds_y_m[0] + 1.0, m.bounds_y_m[1] - 1.0

        def uniform(count):
            """Tire `count` positions uniformes dans l'arène (marge 1 m, altitude 0,6 à 2,5 m) et des caps au hasard."""
            pos = torch.rand(count, 3, device=self.device)
            pos[:, 0] = x0 + pos[:, 0] * (x1 - x0)
            pos[:, 1] = y0 + pos[:, 1] * (y1 - y0)
            pos[:, 2] = 0.6 + pos[:, 2] * 1.9
            yaw = (torch.rand(count, device=self.device) * 2 - 1) * math.pi
            return pos, yaw

        for k in range(D):
            pos, yaw = uniform(n)
            p_near = self._curr.spawn_near_prob() if self._split == "train" else 0.0
            near = torch.rand(n, device=self.device) < p_near
            # départ aidé : à 0,8-2,0 m devant la face d'un QR non lu tiré au hasard, à sa hauteur
            if near.any():
                envs = env_ids[near]
                unread = self._readable[envs] & ~self._read[envs]
                weights = unread.float() + 1e-6
                tags = torch.multinomial(weights, 1).squeeze(-1)
                face = torch.randint(0, len(FACES), (len(envs),), device=self.device)
                fidx = tags * len(FACES) + face
                tag_p = self._qr_pos_local[fidx]
                tag_n = self._qr_normal[fidx]
                dist = 0.8 + torch.rand(len(envs), device=self.device) * 1.2
                cand = tag_p + tag_n * dist.unsqueeze(-1)
                cand[:, 2] = tag_p[:, 2].clamp(self.cfg.alt_min + 0.1, self.cfg.alt_max - 0.1)
                pos[near] = cand
                # départ aidé le FLANC tourné vers le QR (caméras latérales) : nez vers le QR, aucune
                # caméra ne le voit et le curriculum ne reçoit plus de lectures, sans alerte
                side_pick = torch.randint(0, 2, (len(envs),), device=self.device).float() * 2.0 - 1.0
                yaw[near] = (torch.atan2(-tag_n[:, 1], -tag_n[:, 0]) + side_pick * (math.pi / 2)
                             + (torch.rand(len(envs), device=self.device) - 0.5) * 0.6)

            # position trop près d'un obstacle : nouveau tirage, puis repli sur une rangée près de (0, 0, 1 m)
            free = self._clearance_ok(pos)
            if (~free).any():
                pos2, yaw2 = uniform(int((~free).sum()))
                pos[~free], yaw[~free] = pos2, yaw2
                free2 = self._clearance_ok(pos)
                bad = ~free2
                if bad.any():
                    pos[bad] = torch.tensor([0.0, 0.0, 1.0], device=self.device)
                    pos[bad, 0] += (k - (D - 1) / 2.0) * 0.7
                    yaw[bad] = 0.0

            # pose écrite dans Isaac : position + quaternion (cos(cap/2), 0, 0, sin(cap/2)), vitesse nulle
            root = self._drones[k].data.default_root_state[env_ids].clone()
            root[:, :3] = pos + self.scene.env_origins[env_ids]
            root[:, 3] = torch.cos(yaw / 2)
            root[:, 4:6] = 0.0
            root[:, 6] = torch.sin(yaw / 2)
            root[:, 7:] = 0.0
            self._drones[k].write_root_pose_to_sim(root[:, :7], env_ids)
            self._drones[k].write_root_velocity_to_sim(root[:, 7:], env_ids)

    def _clearance_ok(self, pos_local: torch.Tensor, min_clear: float = 0.45) -> torch.Tensor:
        """Renvoie vrai pour chaque position dégagée : aucun obstacle à moins de 0,45 m sur 10 rayons (8 à plat, haut, bas)."""
        n = pos_local.shape[0]
        ang = torch.arange(8, device=self.device) * (2 * math.pi / 8)
        dirs = torch.stack([torch.cos(ang), torch.sin(ang), torch.zeros_like(ang)], dim=-1)
        dirs = torch.cat([dirs, torch.tensor([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]], device=self.device)], dim=0)
        starts = (pos_local + self.scene.env_origins[0]).unsqueeze(1).expand(n, 10, 3).contiguous()
        hits = raycast_mesh(starts, dirs.unsqueeze(0).expand(n, 10, 3).contiguous(),
                            max_dist=MAX_DIST, mesh=self._wh_mesh)[0]
        dist = torch.norm(hits - starts, dim=-1)
        dist = torch.nan_to_num(dist, nan=MAX_DIST, posinf=MAX_DIST)
        return dist.min(dim=1).values > min_clear
