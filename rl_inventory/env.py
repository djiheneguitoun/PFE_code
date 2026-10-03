"""Arène d'apprentissage par renforcement pour l'inventaire QR (Isaac Sim 5.1 / Isaac Lab 2.3) : entrepôt, drones, LiDAR, lecture des QR.

QRInventoryEnv : 1 drone, sans récompense (sert aux tests T1.x) ; SwarmQREnv : 3 drones, la tâche entraînée par train_ppo.py.
Les drones (Crazyflie) sont pilotés en vitesse ; un QR est « lu » selon une règle géométrique (distance, angle, vitesse).
Chargé par gym.make après le démarrage d'Isaac Sim (voir __init__.py) ; même monde pour les 3 cerveaux (Pore / PPO / Dreamer).
"""

import math
import os

import omni.usd
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectMARLEnv, DirectMARLEnvCfg, DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import MultiMeshRayCaster, MultiMeshRayCasterCfg
from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import euler_xyz_from_quat
from isaaclab.utils.warp import raycast_mesh
from isaaclab_assets import CRAZYFLIE_CFG

from . import actuator
from .config_rl import CFG
from .qr_task import FACES, carton_qr_world_poses, find_cartons

WAREHOUSE_USD = os.getenv("AIF_FACTORY_USD", CFG.scene.warehouse_usd)  # fichier USD de l'entrepôt (variable d'environnement, sinon config_rl)
MAX_DIST = CFG.lidar.max_distance_m  # m : portée du LiDAR (8 m)


@configclass
class QRInventoryEnvCfg(DirectRLEnvCfg):
    """Tous les réglages de l'arène (durée, tailles d'obs/action, sim, scène, drone, LiDAR, bornes)."""

    decimation = CFG.train.decimation
    episode_length_s = CFG.train.episode_length_s
    action_space = CFG.action.dim
    observation_space = CFG.lidar.num_rays + 14  # LiDAR (1800) + état (13) + mémoire QR (1) = 1814
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=CFG.train.physics_dt, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=CFG.train.num_envs, env_spacing=CFG.scene.env_spacing_m, replicate_physics=True
    )
    robot: ArticulationCfg = CRAZYFLIE_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    warehouse: sim_utils.UsdFileCfg = sim_utils.UsdFileCfg(usd_path=WAREHOUSE_USD)
    lidar: MultiMeshRayCasterCfg = MultiMeshRayCasterCfg(
        prim_path="/World/envs/env_.*/Robot",
        mesh_prim_paths=[
            MultiMeshRayCasterCfg.RaycastTargetCfg(
                prim_expr="/World/envs/env_.*/Warehouse",
                merge_prim_meshes=True,
                track_mesh_transforms=False,
            )
        ],
        pattern_cfg=LidarPatternCfg(
            channels=CFG.lidar.num_channels,
            vertical_fov_range=CFG.lidar.vertical_fov_deg,
            horizontal_fov_range=CFG.lidar.horizontal_fov_deg,
            horizontal_res=CFG.lidar.horizontal_res_deg,
        ),
        max_distance=MAX_DIST,
        ray_alignment=CFG.lidar.ray_alignment,
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, CFG.lidar.offset_z_m)),
    )

    max_lin_vel: float = CFG.action.max_lin_vel_mps
    max_yaw_rate: float = CFG.action.max_yaw_rate_rps
    alt_min: float = CFG.action.altitude_min_m
    alt_max: float = CFG.action.altitude_max_m
    start_altitude: float = 1.0  # m : altitude de départ


class QRInventoryEnv(DirectRLEnv):
    """Environnement à 1 drone : construit le monde, applique l'action, renvoie observation / récompense (nulle) / fin."""

    cfg: QRInventoryEnvCfg

    def __init__(self, cfg: QRInventoryEnvCfg, render_mode: str | None = None, **kwargs):
        """Prépare les tampons internes (dernière action, suivi des QR lus) ; les QR sont repérés au 1er pas."""
        super().__init__(cfg, render_mode, **kwargs)
        self._actions = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)
        self._qr_ready = False
        self._new_reads = torch.zeros(self.num_envs, device=self.device)
        self._read_frac = torch.zeros(self.num_envs, device=self.device)

    def _setup_scene(self):
        """Construit le monde une fois : drone sans gravité (altitude imposée) + LiDAR, entrepôt, sol, lumière,
        puis copie la scène dans tous les environnements parallèles."""
        spawn = self.cfg.robot.spawn
        if spawn is not None:
            spawn.rigid_props = spawn.rigid_props or sim_utils.RigidBodyPropertiesCfg()
            spawn.rigid_props.disable_gravity = True

        self._robot = Articulation(self.cfg.robot)
        self._lidar = MultiMeshRayCaster(self.cfg.lidar)
        self.cfg.warehouse.func("/World/envs/env_0/Warehouse", self.cfg.warehouse, translation=(0.0, 0.0, 0.0))

        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/ground", ground)
        light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9))
        light.func("/World/Light", light)

        self.scene.clone_environments(copy_from_source=False)
        self.scene.articulations["robot"] = self._robot
        self.scene.sensors["lidar"] = self._lidar

    def _ensure_qr(self):
        """Repère au 1er pas (géométrie chargée) la position et la normale des QR de l'entrepôt n° 0,
        et crée le suivi : QR lus, et depuis combien de pas chacun est visé."""
        stage = omni.usd.get_context().get_stage()
        cartons = [p for p in find_cartons(stage) if "/env_0/" in p.GetPath().pathString]
        pos, norm = carton_qr_world_poses(stage, cartons)
        self._qr_pos_local = torch.tensor(pos, dtype=torch.float32, device=self.device) - self.scene.env_origins[0]
        self._qr_normal = torch.tensor(norm, dtype=torch.float32, device=self.device)
        self._n_cartons = len(cartons)
        self._read = torch.zeros(self.num_envs, self._n_cartons, dtype=torch.bool, device=self.device)
        self._dwell = torch.zeros(self.num_envs, self._n_cartons, dtype=torch.int32, device=self.device)
        self._qr_ready = True

    def _lidar_ranges(self) -> torch.Tensor:
        """Renvoie les 1800 distances du LiDAR divisées par la portée (entre 0 et 1), avec bruit optionnel."""
        hits = self._lidar.data.ray_hits_w
        pos = self._lidar.data.pos_w.unsqueeze(1)
        dist = torch.norm(hits - pos, dim=-1)
        dist = torch.nan_to_num(dist, nan=MAX_DIST, posinf=MAX_DIST).clamp(0.0, MAX_DIST)
        if CFG.lidar.noise_std_m > 0.0:
            dist = dist + torch.randn_like(dist) * CFG.lidar.noise_std_m
        if CFG.lidar.dropout_prob > 0.0:
            drop = torch.rand_like(dist) < CFG.lidar.dropout_prob
            dist = torch.where(drop, torch.full_like(dist, MAX_DIST), dist)
        return dist.clamp(0.0, MAX_DIST) / MAX_DIST

    def _update_qr(self):
        """Décide à chaque pas quels QR deviennent « lus » (≤ 3 m, dans le champ, de face, drone lent,
        2 pas de suite) et met à jour la fraction de QR lus."""
        if not self._qr_ready:
            self._ensure_qr()
        d = self._robot.data
        qr_pos = self._qr_pos_local.unsqueeze(0) + self.scene.env_origins.unsqueeze(1)
        vec = qr_pos - d.root_pos_w.unsqueeze(1)
        dist = torch.norm(vec, dim=-1).clamp_min(1e-6)
        direction = vec / dist.unsqueeze(-1)

        _, _, yaw = euler_xyz_from_quat(d.root_quat_w)
        fwd = torch.stack([torch.cos(yaw), torch.sin(yaw), torch.zeros_like(yaw)], dim=-1)
        in_fov = (direction * fwd.unsqueeze(1)).sum(-1) >= math.cos(math.radians(CFG.qr.camera_fov_deg))
        facing = (-direction * self._qr_normal.unsqueeze(0)).sum(-1) >= math.cos(math.radians(CFG.qr.max_view_angle_deg))
        close = dist <= CFG.qr.max_read_distance_m
        # un carton compte comme visible si l'une de ses 2 faces à QR l'est
        visible = (close & in_fov & facing).view(self.num_envs, self._n_cartons, len(FACES)).any(-1)

        lin = torch.norm(d.root_lin_vel_w[:, :2], dim=-1)
        yawrate = torch.abs(d.root_ang_vel_w[:, 2])
        slow = (lin <= CFG.qr.max_read_speed_mps) & (yawrate <= CFG.qr.max_read_yawrate_rps)
        readable = visible & slow.unsqueeze(1)

        self._dwell = torch.where(readable, self._dwell + 1, torch.zeros_like(self._dwell))
        newly = readable & (self._dwell >= CFG.qr.min_dwell_steps) & (~self._read)
        self._read |= newly
        self._new_reads = newly.sum(dim=1).float()
        self._read_frac = self._read.float().mean(dim=1)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        """Mémorise l'action de la politique (vx, vy, vz, vitesse de lacet), bornée entre -1 et 1."""
        self._actions = actions.clone().clamp(-1.0, 1.0)

    def _apply_action(self) -> None:
        """Convertit l'action en vitesse (m/s, repère monde) et l'impose directement au drone ;
        vz est mis à zéro aux butées d'altitude."""
        a = self._actions
        vx_b = a[:, 0] * self.cfg.max_lin_vel
        vy_b = a[:, 1] * self.cfg.max_lin_vel
        vz = a[:, 2] * self.cfg.max_lin_vel
        yaw_rate = a[:, 3] * self.cfg.max_yaw_rate

        _, _, yaw = euler_xyz_from_quat(self._robot.data.root_quat_w)
        cy, sy = torch.cos(yaw), torch.sin(yaw)
        vx_w = vx_b * cy - vy_b * sy
        vy_w = vx_b * sy + vy_b * cy

        z = self._robot.data.root_pos_w[:, 2] - self.scene.env_origins[:, 2]
        vz = torch.where((z >= self.cfg.alt_max) & (vz > 0), torch.zeros_like(vz), vz)
        vz = torch.where((z <= self.cfg.alt_min) & (vz < 0), torch.zeros_like(vz), vz)

        vel = torch.zeros(self.num_envs, 6, device=self.device)
        vel[:, 0], vel[:, 1], vel[:, 2] = vx_w, vy_w, vz
        vel[:, 5] = yaw_rate
        self._robot.write_root_velocity_to_sim(vel)

    def _get_observations(self) -> dict:
        """Renvoie l'observation : LiDAR (1800) + état du drone (position, orientation, vitesses : 13) + fraction de QR lus (1)."""
        d = self._robot.data
        state = torch.cat(
            [d.root_pos_w - self.scene.env_origins, d.root_quat_w, d.root_lin_vel_b, d.root_ang_vel_b], dim=-1
        )
        return {"policy": torch.cat([self._lidar_ranges(), state, self._read_frac.unsqueeze(-1)], dim=-1)}

    def _get_rewards(self) -> torch.Tensor:
        """Renvoie toujours 0 (récompense notée « à remplir en T1.6 » ; seule SwarmQREnv a une récompense)."""
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Met à jour la lecture des QR, puis renvoie (terminé, tronqué) : jamais terminé, tronqué à la fin du temps (45 s)."""
        self._update_qr()
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return terminated, time_out

    def _reset_idx(self, env_ids):
        """Replace le drone au départ (1 m d'altitude) dans les environnements finis et efface leurs QR lus."""
        if env_ids is None:
            env_ids = self._robot._ALL_INDICES
        super()._reset_idx(env_ids)
        root_state = self._robot.data.default_root_state[env_ids].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids]
        root_state[:, 2] = self.scene.env_origins[env_ids, 2] + self.cfg.start_altitude
        self._robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)
        if getattr(self, "_qr_ready", False):
            self._read[env_ids] = False
            self._dwell[env_ids] = 0
            self._read_frac[env_ids] = 0.0


# =====================================================================================
# Essaim multi-drone (3 drones / entrepôt) — politique partagée décentralisée (IPPO :
# chaque drone décide seul à partir de ce qu'il perçoit, avec le même réseau pour tous).
# =====================================================================================

NUM_DRONES = CFG.train.num_drones  # 3 drones par entrepôt
AGENTS = [f"drone_{k}" for k in range(NUM_DRONES)]  # noms des agents : drone_0, drone_1, drone_2
OBS_DIM = CFG.lidar.num_rays + 13 + (NUM_DRONES - 1) * 3 + 1  # 1800 + 13 + 6 (2 voisins) + 1 = 1820 par drone
STATE_DIM = NUM_DRONES * 13 + NUM_DRONES * CFG.lidar.num_rays + 1  # 39 + 5400 + 1 = 5440 (état global pour MAPPO)
_FAR = 1.0e6  # m : distance « infinie » donnée aux QR déjà lus (récompense d'approche)


@configclass
class SwarmQREnvCfg(DirectMARLEnvCfg):
    """Réglages de l'essaim : 3 drones par entrepôt, observation et action par drone, état global pour MAPPO, modèle d'actionneur."""

    decimation = CFG.train.decimation
    episode_length_s = CFG.train.episode_length_s
    possible_agents = AGENTS
    action_spaces = {a: CFG.action.dim for a in AGENTS}
    observation_spaces = {a: OBS_DIM for a in AGENTS}
    state_space = STATE_DIM

    sim: SimulationCfg = SimulationCfg(dt=CFG.train.physics_dt, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=CFG.train.num_envs, env_spacing=CFG.scene.env_spacing_m,
        replicate_physics=True, lazy_sensor_update=True,
    )
    robot: ArticulationCfg = CRAZYFLIE_CFG.replace(prim_path="/World/envs/env_.*/Drone_0")
    warehouse: sim_utils.UsdFileCfg = sim_utils.UsdFileCfg(usd_path=WAREHOUSE_USD)
    lidar: MultiMeshRayCasterCfg = MultiMeshRayCasterCfg(
        prim_path="/World/envs/env_.*/Drone_0",
        update_period=1.0e9,  # capteur seulement pour CONSTRUIRE le mesh ; raycast fait à la main (statique)
        mesh_prim_paths=[
            MultiMeshRayCasterCfg.RaycastTargetCfg(
                prim_expr="/World/envs/env_.*/Warehouse",
                merge_prim_meshes=True,
                track_mesh_transforms=False,
                is_shared=True,  # entrepôt identique partout : 1 mesh lu/partagé (démarrage rapide)
            )
        ],
        pattern_cfg=LidarPatternCfg(
            channels=CFG.lidar.num_channels,
            vertical_fov_range=CFG.lidar.vertical_fov_deg,
            horizontal_fov_range=CFG.lidar.horizontal_fov_deg,
            horizontal_res=CFG.lidar.horizontal_res_deg,
        ),
        max_distance=MAX_DIST,
        ray_alignment=CFG.lidar.ray_alignment,
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, CFG.lidar.offset_z_m)),
    )

    max_lin_vel: float = CFG.action.max_lin_vel_mps
    max_yaw_rate: float = CFG.action.max_yaw_rate_rps
    alt_min: float = CFG.action.altitude_min_m
    alt_max: float = CFG.action.altitude_max_m
    start_altitude: float = 1.0  # m : altitude de départ
    max_vz_up: float = CFG.action.max_vz_up_mps
    max_vz_down: float = CFG.action.max_vz_down_mps
    accel_xy: float = CFG.action.accel_xy_mps2
    accel_z: float = CFG.action.accel_z_mps2
    yaw_accel: float = CFG.action.yaw_accel_rps2
    tau_xy: float = CFG.action.tau_xy_s
    tau_z: float = CFG.action.tau_z_s
    tau_yaw: float = CFG.action.tau_yaw_s


class SwarmQREnv(DirectMARLEnv):
    """Essaim de 3 drones dans le même entrepôt : chacun voit son LiDAR, ses voisins et la fraction de QR lus par l'équipe."""

    cfg: SwarmQREnvCfg

    def __init__(self, cfg: SwarmQREnvCfg, render_mode: str | None = None, **kwargs):
        """Prépare les tampons par drone (actions, à-coups, collisions, lecture QR partagée, vitesse de l'actionneur)."""
        super().__init__(cfg, render_mode, **kwargs)
        self._actions = {a: torch.zeros(self.num_envs, CFG.action.dim, device=self.device) for a in AGENTS}
        self._prev_actions = {a: torch.zeros(self.num_envs, CFG.action.dim, device=self.device) for a in AGENTS}
        self._jerk = {a: torch.zeros(self.num_envs, device=self.device) for a in AGENTS}
        self._collision = torch.zeros(NUM_DRONES, self.num_envs, dtype=torch.bool, device=self.device)
        self._lidar_min = torch.full((NUM_DRONES, self.num_envs), MAX_DIST, device=self.device)  # pour l'éval (risque)
        self._new_reads = torch.zeros(NUM_DRONES, self.num_envs, device=self.device)
        self._posture = torch.zeros(NUM_DRONES, self.num_envs, device=self.device)
        self._read_frac = torch.zeros(self.num_envs, device=self.device)
        self._nearest_unread = torch.full((NUM_DRONES, self.num_envs), _FAR, device=self.device)
        self._approach = torch.zeros(NUM_DRONES, self.num_envs, device=self.device)
        self._qr_ready = False
        self._refreshed = False
        self._stepping = False
        # état d'actionneur, repère MONDE : (vx, vy, vz, taux de lacet) par drone.
        # C'est LUI la vitesse du drone ; l'action n'est plus qu'une consigne.
        self._vel_state = torch.zeros(NUM_DRONES, self.num_envs, 4, device=self.device)
        self._vel_buf = torch.zeros(NUM_DRONES, self.num_envs, 6, device=self.device)
        # LiDAR statique maison : motif de rayons (généré 1 fois) + mesh entrepôt partagé (récupéré au 1er pas)
        pcfg = self.cfg.lidar.pattern_cfg
        _, self._ray_dirs = pcfg.func(pcfg, self.device)
        self._ray_off = torch.tensor([0.0, 0.0, CFG.lidar.offset_z_m], device=self.device)
        self._wh_mesh = None

    def _setup_scene(self):
        """Crée les 3 drones (gravité coupée) et leurs 3 LiDAR, charge l'entrepôt, puis copie le tout dans chaque environnement."""
        self._drones, self._lidars = [], []
        for k in range(NUM_DRONES):
            d_cfg = self.cfg.robot.replace(prim_path=f"/World/envs/env_.*/Drone_{k}")
            spawn = d_cfg.spawn
            if spawn is not None:
                spawn.rigid_props = spawn.rigid_props or sim_utils.RigidBodyPropertiesCfg()
                spawn.rigid_props.disable_gravity = True
            drone = Articulation(d_cfg)
            self._drones.append(drone)
            self.scene.articulations[f"drone_{k}"] = drone
            lidar = MultiMeshRayCaster(self.cfg.lidar.replace(prim_path=f"/World/envs/env_.*/Drone_{k}"))
            self._lidars.append(lidar)
            self.scene.sensors[f"lidar_{k}"] = lidar

        self.cfg.warehouse.func("/World/envs/env_0/Warehouse", self.cfg.warehouse, translation=(0.0, 0.0, 0.0))
        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/ground", ground)
        light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9))
        light.func("/World/Light", light)
        self.scene.clone_environments(copy_from_source=False)

    def _ensure_qr(self):
        """Repère au 1er pas la position et la normale de chaque QR, et crée le suivi PARTAGÉ (une liste de QR lus par entrepôt)."""
        stage = omni.usd.get_context().get_stage()
        cartons = [p for p in find_cartons(stage) if "/env_0/" in p.GetPath().pathString]
        pos, norm = carton_qr_world_poses(stage, cartons)
        self._qr_pos_local = torch.tensor(pos, dtype=torch.float32, device=self.device) - self.scene.env_origins[0]
        self._qr_normal = torch.tensor(norm, dtype=torch.float32, device=self.device)
        self._n_cartons = len(cartons)
        self._read = torch.zeros(self.num_envs, self._n_cartons, dtype=torch.bool, device=self.device)
        self._dwell = torch.zeros(self.num_envs, self._n_cartons, dtype=torch.int32, device=self.device)
        self._qr_ready = True

    def _lidar_ranges(self, lidar):
        """Renvoie les distances (0 à 1) lues sur un capteur LiDAR Isaac et la distance minimale brute (m)."""
        hits = lidar.data.ray_hits_w
        pos = lidar.data.pos_w.unsqueeze(1)
        dist = torch.norm(hits - pos, dim=-1)
        dist = torch.nan_to_num(dist, nan=MAX_DIST, posinf=MAX_DIST).clamp(0.0, MAX_DIST)
        raw_min = dist.min(dim=1).values
        if CFG.lidar.noise_std_m > 0.0:
            dist = dist + torch.randn_like(dist) * CFG.lidar.noise_std_m
        if CFG.lidar.dropout_prob > 0.0:
            drop = torch.rand_like(dist) < CFG.lidar.dropout_prob
            dist = torch.where(drop, torch.full_like(dist, MAX_DIST), dist)
        return dist.clamp(0.0, MAX_DIST) / MAX_DIST, raw_min

    def _update_qr(self):
        """Met à jour la lecture PARTAGÉE : un QR est lu si l'un des 3 drones le voit bien et lentement ;
        calcule aussi, par drone, la posture, les QR crédités et les mètres gagnés vers le QR non lu le plus proche."""
        if not self._qr_ready:
            self._ensure_qr()
        qr_pos = self._qr_pos_local.unsqueeze(0) + self.scene.env_origins.unsqueeze(1)
        readable_any = torch.zeros(self.num_envs, self._n_cartons, dtype=torch.bool, device=self.device)
        readable_per, dist_cf_per = [], []
        for k in range(NUM_DRONES):
            d = self._drones[k].data
            vec = qr_pos - d.root_pos_w.unsqueeze(1)
            dist = torch.norm(vec, dim=-1).clamp_min(1e-6)
            direction = vec / dist.unsqueeze(-1)
            _, _, yaw = euler_xyz_from_quat(d.root_quat_w)
            fwd = torch.stack([torch.cos(yaw), torch.sin(yaw), torch.zeros_like(yaw)], dim=-1)
            in_fov = (direction * fwd.unsqueeze(1)).sum(-1) >= math.cos(math.radians(CFG.qr.camera_fov_deg))
            facing = (-direction * self._qr_normal.unsqueeze(0)).sum(-1) >= math.cos(math.radians(CFG.qr.max_view_angle_deg))
            close = dist <= CFG.qr.max_read_distance_m
            visible = (close & in_fov & facing).view(self.num_envs, self._n_cartons, len(FACES)).any(-1)
            lin = torch.norm(d.root_lin_vel_w[:, :2], dim=-1)
            yawrate = torch.abs(d.root_ang_vel_w[:, 2])
            slow = (lin <= CFG.qr.max_read_speed_mps) & (yawrate <= CFG.qr.max_read_yawrate_rps)
            rk = visible & slow.unsqueeze(1)
            readable_per.append(rk)
            dist_cf_per.append(dist.view(self.num_envs, self._n_cartons, len(FACES)).min(-1).values)
            readable_any |= rk
        self._dwell = torch.where(readable_any, self._dwell + 1, torch.zeros_like(self._dwell))
        newly = readable_any & (self._dwell >= CFG.qr.min_dwell_steps) & (~self._read)
        unread = ~self._read  # posture : drone bien placé (proche + face + LENT) sur un QR non lu
        self._posture = torch.zeros(NUM_DRONES, self.num_envs, device=self.device)
        for k in range(NUM_DRONES):
            self._posture[k] = (readable_per[k] & unread).any(dim=1).float()
        self._read |= newly
        self._read_frac = self._read.float().mean(dim=1)
        credited = torch.zeros_like(self._read)
        self._new_reads = torch.zeros(NUM_DRONES, self.num_envs, device=self.device)
        for k in range(NUM_DRONES):
            cred = readable_per[k] & newly & (~credited)
            credited |= cred
            self._new_reads[k] = cred.sum(dim=1).float()
        # récompense dense : mètres gagnés vers le QR NON LU le plus proche, par drone
        read_d = CFG.qr.max_read_distance_m  # ne plus récompenser l'approche au-delà de la distance de lecture
        for k in range(NUM_DRONES):
            nearest = dist_cf_per[k].masked_fill(self._read, _FAR).amin(dim=1).clamp_min(read_d)
            prev = self._nearest_unread[k]
            appr = torch.clamp(prev - nearest, min=0.0)
            self._approach[k] = torch.where(prev >= _FAR, torch.zeros_like(appr), appr)
            self._nearest_unread[k] = nearest

    def _raycast_static(self, k):
        """Lance les 1800 rayons du drone k sur le maillage fixe partagé de l'entrepôt (sans mise à jour du maillage) ;
        renvoie les distances (0 à 1) et la distance minimale brute (m)."""
        if self._wh_mesh is None:
            self._wh_mesh = next(iter(MultiMeshRayCaster.meshes.values()))
        origins = self.scene.env_origins
        r = self._ray_dirs.shape[0]
        base = (self._drones[k].data.root_pos_w + self._ray_off).unsqueeze(1)  # (N,1,3)
        # un seul maillage, celui de l'entrepôt n° 0 : chaque drone y est ramené par décalage d'origine
        starts = (base - origins.unsqueeze(1) + origins[0]).expand(self.num_envs, r, 3).contiguous()
        dirs = self._ray_dirs.unsqueeze(0).expand(self.num_envs, r, 3).contiguous()
        hits = raycast_mesh(starts, dirs, max_dist=MAX_DIST, mesh=self._wh_mesh)[0]
        dist = torch.norm(hits - starts, dim=-1)
        dist = torch.nan_to_num(dist, nan=MAX_DIST, posinf=MAX_DIST).clamp(0.0, MAX_DIST)
        raw_min = dist.min(dim=1).values
        if CFG.lidar.noise_std_m > 0.0:
            dist = dist + torch.randn_like(dist) * CFG.lidar.noise_std_m
        if CFG.lidar.dropout_prob > 0.0:
            drop = torch.rand_like(dist) < CFG.lidar.dropout_prob
            dist = torch.where(drop, torch.full_like(dist, MAX_DIST), dist)
        return dist.clamp(0.0, MAX_DIST) / MAX_DIST, raw_min

    def _refresh(self):
        """Calcule une seule fois par pas le LiDAR, l'état et les collisions de chaque drone, et la lecture des QR (pendant un pas)."""
        if self._refreshed:
            return
        self._lidar_cache, self._state_cache = [], []
        self._collision = torch.zeros(NUM_DRONES, self.num_envs, dtype=torch.bool, device=self.device)
        for k in range(NUM_DRONES):
            lk, raw_min = self._raycast_static(k)
            self._lidar_cache.append(lk)
            self._collision[k] = raw_min < CFG.reward.collision_distance_m
            self._lidar_min[k] = raw_min
            d = self._drones[k].data
            self._state_cache.append(
                torch.cat([d.root_pos_w - self.scene.env_origins, d.root_quat_w, d.root_lin_vel_b, d.root_ang_vel_b], dim=-1)
            )
        if self._stepping:
            self._update_qr()
        self._refreshed = True

    def _pre_physics_step(self, actions: dict) -> None:
        """Mémorise les actions des 3 drones (bornées à ±1), calcule leurs à-coups ‖Δaction‖² et marque le début d'un pas."""
        self._stepping = True
        self._refreshed = False
        for a in AGENTS:
            act = actions[a].clone().clamp(-1.0, 1.0)
            self._jerk[a] = ((act - self._prev_actions[a]) ** 2).sum(-1)
            self._prev_actions[a] = act
            self._actions[a] = act

    def _apply_action(self) -> None:
        """Applique un pas d'actionneur à chacun des 3 drones (appelé à chaque pas physique, 4 fois par pas de contrôle)."""
        for k in range(NUM_DRONES):
            self._drive(k)

    def _drive(self, k: int) -> None:
        """Fait rejoindre au drone k sa vitesse visée avec retard et accélération bornée (actuator.py), puis l'impose au simulateur.
        Filtre en repère monde (en repère drone, la vitesse tournerait avec le nez) ; la consigne est tournée du drone vers le monde."""
        robot, a, cfg = self._drones[k], self._actions[f"drone_{k}"], self.cfg
        _, _, yaw = euler_xyz_from_quat(robot.data.root_quat_w)
        cy, sy = torch.cos(yaw), torch.sin(yaw)
        vz_cap = torch.where(a[:, 2] >= 0, cfg.max_vz_up, -cfg.max_vz_down)
        cmd = torch.stack(
            [
                (a[:, 0] * cy - a[:, 1] * sy) * cfg.max_lin_vel,
                (a[:, 0] * sy + a[:, 1] * cy) * cfg.max_lin_vel,
                a[:, 2].abs() * vz_cap,
                a[:, 3] * cfg.max_yaw_rate,
            ],
            dim=-1,
        )
        v = actuator.integrate(
            self._vel_state[k], cmd, self.physics_dt,
            accel_xy=cfg.accel_xy, accel_z=cfg.accel_z, yaw_accel=cfg.yaw_accel,
            tau_xy=cfg.tau_xy, tau_z=cfg.tau_z, tau_yaw=cfg.tau_yaw,
        )
        # butée d'altitude appliquée à l'ÉTAT, après le filtre : couper la seule commande
        # laisserait le retard du filtre dépasser la butée.
        z = robot.data.root_pos_w[:, 2] - self.scene.env_origins[:, 2]
        v[:, 2] = actuator.altitude_envelope(v[:, 2], z, cfg.alt_min, cfg.alt_max, cfg.accel_z,
                                             max_delta=cfg.accel_z * self.physics_dt)
        self._vel_state[k] = v

        vel = self._vel_buf[k]
        vel.zero_()
        vel[:, :3] = v[:, :3]
        vel[:, 5] = v[:, 3]
        robot.write_root_velocity_to_sim(vel)

    def _get_observations(self) -> dict:
        """Renvoie l'observation locale de chaque drone : LiDAR (1800) + état (13) + positions relatives des 2 voisins (6) + fraction lue (1)."""
        self._refresh()
        shared = self._read_frac.unsqueeze(-1)
        obs = {}
        for k in range(NUM_DRONES):
            pos_k = self._drones[k].data.root_pos_w
            neigh = torch.cat(
                [self._drones[j].data.root_pos_w - pos_k for j in range(NUM_DRONES) if j != k], dim=-1
            )
            obs[f"drone_{k}"] = torch.cat([self._lidar_cache[k], self._state_cache[k], neigh, shared], dim=-1)
        return obs

    def _get_states(self) -> torch.Tensor:
        """Renvoie l'état global (3 états + 3 LiDAR + fraction lue = 5440 valeurs) pour le critique central de MAPPO."""
        self._refresh()
        return torch.cat(self._state_cache + self._lidar_cache + [self._read_frac.unsqueeze(-1)], dim=-1)

    def _get_rewards(self) -> dict:
        """Renvoie la récompense de chaque drone : ses QR lus (+10), posture, approche, mission (+100), temps, collision, à-coups."""
        self._refresh()
        mission = (self._read_frac >= CFG.qr.coverage_target).float() * CFG.reward.mission_complete
        out = {}
        for k in range(NUM_DRONES):
            r = self._new_reads[k] * CFG.reward.new_qr
            r = r + self._posture[k] * CFG.reward.posture_scale
            r = r + self._approach[k] * CFG.reward.approach_scale
            r = r + mission + CFG.reward.time_step
            r = r + self._collision[k].float() * CFG.reward.collision
            r = r - CFG.reward.jerk_lambda * self._jerk[f"drone_{k}"]
            out[f"drone_{k}"] = r
        return out

    def _get_dones(self) -> tuple[dict, dict]:
        """Renvoie (terminé, tronqué) par drone : terminé quand tous les QR sont lus, tronqué à la fin du temps (45 s).
        La collision n'arrête pas l'épisode : Isaac Lab ne réinitialise un entrepôt que si tous ses drones ont fini."""
        self._refresh()
        mission_done = self._read_frac >= CFG.qr.coverage_target
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = {f"drone_{k}": mission_done for k in range(NUM_DRONES)}
        truncated = {f"drone_{k}": time_out for k in range(NUM_DRONES)}
        return terminated, truncated

    def _reset_idx(self, env_ids):
        """Replace les 3 drones au départ (espacés de 0,7 m en x, à 1 m d'altitude), annule leur vitesse et efface les QR lus."""
        if env_ids is None:
            env_ids = self._drones[0]._ALL_INDICES
        super()._reset_idx(env_ids)
        for k in range(NUM_DRONES):
            robot = self._drones[k]
            rs = robot.data.default_root_state[env_ids].clone()
            rs[:, :3] += self.scene.env_origins[env_ids]
            rs[:, 0] += (k - (NUM_DRONES - 1) / 2.0) * 0.7
            rs[:, 2] = self.scene.env_origins[env_ids, 2] + self.cfg.start_altitude
            robot.write_root_pose_to_sim(rs[:, :7], env_ids)
            robot.write_root_velocity_to_sim(rs[:, 7:], env_ids)
        if hasattr(self, "_vel_state"):
            self._vel_state[:, env_ids] = 0.0     # sinon le drone repart avec l'élan de l'épisode précédent
        if getattr(self, "_qr_ready", False):
            self._read[env_ids] = False
            self._dwell[env_ids] = 0
            self._read_frac[env_ids] = 0.0
        if hasattr(self, "_nearest_unread"):
            self._nearest_unread[:, env_ids] = _FAR
            self._approach[:, env_ids] = 0.0
