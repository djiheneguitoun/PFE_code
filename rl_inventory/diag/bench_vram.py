"""Mesure la mémoire graphique (VRAM) et le débit (pas/s) de l'essaim SwarmQREnv pour un nombre d'entrepôts --num_envs donné.

Sert à trouver le plus grand --num_envs qui tient sur la carte de 8 Go ; affiche une ligne « VRAM_RESULT … OK » ou « … FAIL ».
Lancement depuis la racine : bash rl_inventory/launch.sh rl_inventory/diag/bench_vram.py --headless --num_envs 8
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--steps", type=int, default=60)
parser.add_argument("--warmup", type=int, default=25)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import os
import subprocess
import sys
import time

import torch

# rend le paquet rl_inventory importable (racine du projet = deux dossiers au-dessus)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from rl_inventory.env import AGENTS, SwarmQREnv, SwarmQREnvCfg  # noqa: E402


def gpu_used_mb():
    """Renvoie (mémoire utilisée, mémoire totale) du GPU en Mo, lues avec nvidia-smi."""
    out = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=memory.used,memory.total", "--format=csv,noheader,nounits"]
    ).decode().strip().splitlines()[0]
    used, total = (int(x) for x in out.split(","))
    return used, total


def main():
    """Crée l'essaim, fait 25 pas de chauffe puis 60 pas chronométrés (actions aléatoires) et affiche VRAM et débit, ou l'échec."""
    cfg = SwarmQREnvCfg()
    cfg.scene.num_envs = args.num_envs
    try:
        env = SwarmQREnv(cfg)
        env.reset()

        def act():
            """Renvoie des actions aléatoires uniformes dans [-1, 1] pour les 3 drones."""
            return {a: torch.empty((env.num_envs, 4), device=env.device).uniform_(-1.0, 1.0) for a in AGENTS}

        for _ in range(args.warmup):  # warmup non chronométré (build BVH warp + JIT kernels + _ensure_qr)
            env.step(act())
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        for _ in range(args.steps):
            env.step(act())
        torch.cuda.synchronize()
        dt = time.perf_counter() - t0
        sps = args.steps / dt
        samples = sps * env.num_envs * len(AGENTS)
        used, total = gpu_used_mb()
        print(f"VRAM_RESULT num_envs={args.num_envs} OK | GPU_used={used}MB / {total}MB "
              f"| {sps:.1f} steps/s | {samples:.0f} samples/s (régime établi)")
        env.close()
    except RuntimeError as e:
        msg = str(e).splitlines()[0]
        print(f"VRAM_RESULT num_envs={args.num_envs} FAIL | {msg}")


if __name__ == "__main__":
    main()
    simulation_app.close()
