"""Compare, pour un LiDAR, le lancer de rayons dynamique de MultiMeshRayCaster et un lancer statique (raycast_mesh) sur le maillage partagé.

Preuve que le lancer statique est ~100× plus rapide : c'est la solution retenue dans env.py. Ne modifie pas l'environnement.
Lancement depuis la racine : bash rl_inventory/launch.sh rl_inventory/diag/test_static_raycast.py --headless --num_envs 32
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=32)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import os
import sys
import time

import torch

from isaaclab.sensors import MultiMeshRayCaster
from isaaclab.utils.warp import raycast_mesh

# rend le paquet rl_inventory importable (racine du projet = deux dossiers au-dessus)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from rl_inventory.env import AGENTS, NUM_DRONES, SwarmQREnv, SwarmQREnvCfg  # noqa: E402


def timeit(fn, n=20):
    """Renvoie la durée moyenne d'un appel à fn, en ms, mesurée sur n appels (GPU synchronisé avant et après)."""
    torch.cuda.synchronize()
    t = time.perf_counter()
    for _ in range(n):
        fn()
    torch.cuda.synchronize()
    return (time.perf_counter() - t) / n * 1000.0


def main():
    """Crée l'essaim, chronomètre les deux lancers de rayons pour le LiDAR du drone 0, puis affiche le gain et la part de rayons qui touchent."""
    cfg = SwarmQREnvCfg()
    cfg.scene.num_envs = args.num_envs
    env = SwarmQREnv(cfg)
    for _ in range(10):  # warmup (init capteurs + rayons)
        env.step({a: torch.empty((env.num_envs, 4), device=env.device).uniform_(-1.0, 1.0) for a in AGENTS})

    dt = env.cfg.sim.dt
    origins = env.scene.env_origins  # (N,3)

    # mesh fusionné déjà construit par MultiMeshRayCaster (1 seul, partagé)
    mesh = list(MultiMeshRayCaster.meshes.values())[0]
    lidar = env._lidars[0]

    # rayons du capteur (monde) -> ramenés dans le repère du mesh (celui de l'env 0)
    starts_w = lidar._ray_starts_w
    dirs_w = lidar._ray_directions_w
    starts_canon = starts_w - origins.unsqueeze(1) + origins[0]

    # 1) dynamique (ce qu'on subit aujourd'hui), pour 1 LiDAR
    def dyn():
        """Relance tous les rayons du capteur Isaac (chemin dynamique, maillage remis à jour)."""
        lidar.update(dt, force_recompute=True)

    t_dyn = timeit(dyn, 10)

    # 2) statique (le correctif proposé), pour 1 LiDAR
    def stat():
        """Lance les mêmes rayons sur le maillage fixe avec raycast_mesh (chemin statique, portée 8 m)."""
        raycast_mesh(starts_canon, dirs_w, max_dist=8.0, mesh=mesh)

    t_stat = timeit(stat, 20)

    # vérif correctness : combien de rayons touchent (doit être > 0 et raisonnable)
    hits = raycast_mesh(starts_canon, dirs_w, max_dist=8.0, mesh=mesh)[0]
    d = torch.norm(hits - starts_canon, dim=-1)
    finite = torch.isfinite(d) & (d < 8.0)
    frac_touch = finite.float().mean().item()

    print("\n===== PREUVE RAYCAST STATIQUE (1 LiDAR) =====")
    print(f"num_envs              : {env.num_envs}")
    print(f"dynamique (refit)     : {t_dyn:8.1f} ms")
    print(f"STATIQUE (raycast_mesh): {t_stat:8.1f} ms")
    print(f"accélération          : {t_dyn / max(t_stat, 1e-6):8.1f} ×")
    print(f"rayons qui touchent   : {frac_touch * 100:.1f} %  (cohérence du résultat)")
    print("=============================================\n")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
