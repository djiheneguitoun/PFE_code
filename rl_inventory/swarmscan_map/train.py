"""Entraîne la politique SwarmScan-Map : un seul réseau partagé par les 3 drones (PS-PPO, rsl_rl 3.0).

Depuis le dossier rl_inventory/ :
  bash ../launch.sh swarmscan_map/train.py --headless --num_envs 32 --max_iterations 2000
Sorties (modèle .pt toutes les 200 itérations + journaux TensorBoard) : swarmscan_runs/<date>_<run_name>/.
"""

import argparse
import os

# limite la fragmentation de la mémoire GPU (à poser avant que torch n'utilise le GPU)
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Entraînement SwarmScan-Map (rsl_rl PS-PPO).")
parser.add_argument("--num_envs", type=int, default=64)
# Le GAE (estimation de l'avantage) ne regarde qu'environ 1/(1−γλ) ≈ 17 pas (0,57 s) en avant,
# pour un épisode de 4500 pas : c'est très court. Passer à 64 pas avec --mini_batches 4 ne coûte
# pas plus de mémoire (3072 échantillons par rétropropagation au lieu de 6144) et double cette
# fenêtre. À tester SÉPARÉMENT du correctif de curriculum.
parser.add_argument("--num_steps", type=int, default=32, help="pas collectés par env et par itération")
parser.add_argument("--mini_batches", type=int, default=1, help="mini-lots par époque (monter avec --num_steps)")
parser.add_argument("--max_iterations", type=int, default=5000)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--run_name", type=str, default="v2_map")
parser.add_argument("--resume", type=str, default=None, help="checkpoint .pt à reprendre")
parser.add_argument("--start_level", type=int, default=0, help="niveau de curriculum au démarrage (resume : remettre celui du run précédent)")
parser.add_argument("--freeze_level", type=int, default=-1, help="fige le curriculum à ce niveau (fin de parcours : palier stable, sans promotions/reculs)")
parser.add_argument("--entropy", type=float, default=None, help="override du coefficient d'entropie initial")
parser.add_argument("--entropy_final", type=float, default=0.001, help="entropie de la phase convergence (bascule automatique)")
parser.add_argument("--converge_level", type=int, default=9, help="niveau qui déclenche la bascule d'entropie (curriculum à 10 crans)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
# Isaac Sim doit démarrer AVANT d'importer les modules isaaclab et le reste du projet
simulation_app = AppLauncher(args).app

import sys
from datetime import datetime

import torch

# racine du projet ajoutée au chemin : le script se lance depuis n'importe quel dossier
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import rsl_rl.runners.on_policy_runner as _opr
from rsl_rl.runners import OnPolicyRunner

from rl_inventory.swarmscan_map.config_map import MAP_CFG
from rl_inventory.swarmscan_map.env_map import SwarmScanMapEnv, SwarmScanMapEnvCfg
from rl_inventory.swarmscan_map.flatten_wrapper import SwarmMapVecEnv
from rl_inventory.swarmscan_map.models import MapActorCritic

_opr.MapActorCritic = MapActorCritic  # le runner rsl_rl retrouve la classe par son nom (eval) dans son module

N_SCALES = len(MAP_CFG.map.crop_spans_m)  # nombre d'échelles de carte (2)


def train_cfg(num_envs: int) -> dict:
    """Renvoie la configuration rsl_rl : PPO (5 époques, clip 0,2, taux d'apprentissage 3e-4),
    réseau MapActorCritic, sauvegarde du modèle toutes les 200 itérations."""
    steps = args.num_steps
    return {
        "seed": args.seed,
        "num_steps_per_env": steps,
        "max_iterations": args.max_iterations,
        "save_interval": 200,
        # l'acteur voit cartes + vecteur ; le critique voit en plus les infos privilégiées
        "obs_groups": {"policy": ["maps", "vector"], "critic": ["maps", "vector", "privileged"]},
        "logger": "tensorboard",
        "policy": {
            "class_name": "MapActorCritic",
            "map_channels": N_SCALES * MAP_CFG.map.n_channels,
            "map_px": MAP_CFG.map.crop_px,
            "actor_hidden_dims": [256, 128],
            "critic_hidden_dims": [256, 128],
            "init_noise_std": MAP_CFG.train.init_noise_std,
        },
        "algorithm": {
            "class_name": "PPO",
            "num_learning_epochs": 5,
            "num_mini_batches": args.mini_batches,
            "clip_param": 0.2,
            "gamma": MAP_CFG.train.gamma,
            "lam": MAP_CFG.train.lam,
            "value_loss_coef": 1.0,
            "entropy_coef": args.entropy if args.entropy is not None else MAP_CFG.train.entropy_coef,
            "learning_rate": 3.0e-4,
            "max_grad_norm": 1.0,
            "schedule": "fixed",
            "desired_kl": 0.01,
        },
    }


def main():
    """Crée l'environnement, règle le curriculum (niveau de départ ou gel), puis lance l'apprentissage PPO."""
    torch.manual_seed(args.seed)
    env_cfg = SwarmScanMapEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env = SwarmScanMapEnv(env_cfg)
    env._curr.level = args.start_level
    if args.freeze_level >= 0:
        # curriculum figé : délais de montée et de recul rendus infinis
        env._curr.level = args.freeze_level
        env._curr.cfg.min_episodes_per_notch = 10**9
        env._curr.cfg.min_episodes_down = 10**9
    vec = SwarmMapVecEnv(env)

    # sorties : <racine du projet>/swarmscan_runs/<aa-mm-jj_hh-mm-ss>_<run_name>/
    log_dir = os.path.join(
        os.path.dirname(__file__), "..", "..", "swarmscan_runs",
        f"{datetime.now().strftime('%y-%m-%d_%H-%M-%S')}_{args.run_name}",
    )
    runner = OnPolicyRunner(vec, train_cfg(args.num_envs), log_dir=os.path.abspath(log_dir), device=str(env.device))
    if args.resume:
        runner.load(args.resume)
    # bascule d'entropie EN COURS d'entraînement, sans modifier la boucle (celle des 19 entraînements
    # sains) : l'environnement écrit le nouveau coefficient dans l'algorithme au niveau cible
    env._alg_ref = runner.alg
    env._switch_level = args.converge_level
    env._switch_entropy = args.entropy_final

    print(f"\n>>> SwarmScan-Map | envs={args.num_envs}×3 drones | iters={args.max_iterations} | log={log_dir}\n")
    # init_at_random_ep_len : compteurs d'épisode tirés au hasard, pour que les entrepôts ne
    # finissent pas tous au même moment
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)
    vec.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
