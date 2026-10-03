"""Enregistre les trajectoires d'un modèle pour voir OÙ vont les drones (les courbes disent seulement combien ils lisent).

Sauvegarde positions, caps, vitesses et lectures dans un .npz (défaut /tmp/traj.npz), à analyser hors ligne.
  bash rl_inventory/launch.sh rl_inventory/swarmscan_map/record_traj.py --headless \
       --checkpoint swarmscan_runs/<run>/model_XXXX.pt --level 0 --num_envs 8
"""

# Questions visées : le drone balaie-t-il l'entrepôt méthodiquement, tourne-t-il en rond,
# reste-t-il dans une seule zone ?

import argparse
import os

# limite la fragmentation de la mémoire GPU (à poser avant que torch n'utilise le GPU)
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Enregistrement de trajectoires.")
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--level", type=int, default=0)
parser.add_argument("--sigma", type=float, default=-1.0, help="-1 : σ du checkpoint ; 0 : déterministe")
parser.add_argument("--every", type=int, default=5, help="enregistre 1 pas sur N")
parser.add_argument("--out", type=str, default="/tmp/traj.npz")
parser.add_argument("--seed", type=int, default=0)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
# Isaac Sim doit démarrer AVANT d'importer les modules isaaclab et le reste du projet
simulation_app = AppLauncher(args).app

import sys

import numpy as np
import torch

# racine du projet ajoutée au chemin : le script se lance depuis n'importe quel dossier
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from rl_inventory.env import AGENTS
from rl_inventory.swarmscan_map.config_map import MAP_CFG
from rl_inventory.swarmscan_map.env_map import SwarmScanMapEnv, SwarmScanMapEnvCfg
from rl_inventory.swarmscan_map.flatten_wrapper import SwarmMapVecEnv
from rl_inventory.swarmscan_map.models import MapActorCritic

D = len(AGENTS)  # nombre de drones par entrepôt (3)
# groupes d'observation : l'acteur (policy) et le critique, comme à l'entraînement
OBS_GROUPS = {"policy": ["maps", "vector"], "critic": ["maps", "vector", "privileged"]}


def main():
    """Charge le modèle, joue un épisode complet et enregistre 1 pas sur `--every` dans le fichier .npz."""
    torch.manual_seed(args.seed)
    cfg = SwarmScanMapEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = args.device
    env = SwarmScanMapEnv(cfg)
    env._curr.level = args.level
    env._curr.cfg.min_episodes_per_notch = 10**9    # niveau figé : ni montée ni recul
    env._curr.cfg.min_episodes_down = 10**9
    MAP_CFG.train.mission_target = 2.0              # pas de fin anticipée : épisodes complets
    vec = SwarmMapVecEnv(env)
    vec.get_observations()
    env._ensure_qr()      # AVANT le reset : sinon configurations et départs aidés ne sont pas tirés
    vec.reset()

    net = MapActorCritic(
        vec.get_observations(), OBS_GROUPS, vec.num_actions,
        map_channels=len(MAP_CFG.map.crop_spans_m) * MAP_CFG.map.n_channels,
        map_px=MAP_CFG.map.crop_px, init_noise_std=MAP_CFG.train.init_noise_std,
    ).to(env.device)
    net.load_state_dict(torch.load(args.checkpoint, map_location=env.device,
                                   weights_only=False)["model_state_dict"])
    net.eval()
    if args.sigma >= 0:
        net.std.data.fill_(max(args.sigma, 1e-6))
    det = args.sigma == 0.0

    horizon = int(env.max_episode_length)
    pos_log, yaw_log, vel_log, read_log = [], [], [], []
    for step in range(horizon - 1):
        obs = vec.get_observations()
        with torch.no_grad():
            net.update_distribution(net.actor_obs_normalizer(net.get_actor_obs(obs)))
            act = net.distribution.mean if det else net.distribution.sample()
        vec.step(act)
        if step % args.every == 0:
            pos_log.append(env._pos_local.cpu().numpy().copy())        # (B,D,3)
            yaw_log.append(env._yaw.cpu().numpy().copy())              # (B,D)
            vel_log.append(env._lin_k.cpu().numpy().copy())            # (B,D)
            read_log.append(env._new_reads.sum(0).cpu().numpy().copy())  # (B,)
        if (step + 1) % 1000 == 0:
            print(f"    pas {step + 1}/{horizon}", flush=True)

    # contenu du .npz : pos (m), yaw (rad), vel (m/s), reads (lectures par entrepôt), read_frac,
    # bornes de l'arène, position des QR (tag_xy) et masque des QR lisibles
    frac = env._read_frac_readable.cpu().numpy().copy()
    np.savez_compressed(
        args.out, pos=np.stack(pos_log), yaw=np.stack(yaw_log), vel=np.stack(vel_log),
        reads=np.stack(read_log), read_frac=frac, every=args.every,
        bounds_x=np.array(MAP_CFG.map.bounds_x_m), bounds_y=np.array(MAP_CFG.map.bounds_y_m),
        tag_xy=env._tag_xy.cpu().numpy(), readable=env._readable.cpu().numpy(),
    )
    print(f"\ntrajectoires enregistrées : {args.out}")
    print(f"fraction lue par entrepôt : {np.array2string(frac, precision=2)}")
    vec.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
