"""Test rapide (« smoke test » : tout démarre-t-il ?) de l'arène à 3 drones SwarmQREnv, sous actions aléatoires.

Réussi si l'observation fait 1820 valeurs par drone, l'état global 5440, les récompenses sont finies et chaque drone a bougé de plus de 0,1 m.
Lancement depuis la racine : bash rl_inventory/launch.sh rl_inventory/tests/test_swarm.py --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=2)
parser.add_argument("--steps", type=int, default=12)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import os
import sys

import torch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from rl_inventory.env import AGENTS, NUM_DRONES, OBS_DIM, STATE_DIM, SwarmQREnv, SwarmQREnvCfg  # noqa: E402


def main():
    """Fait --steps pas d'actions aléatoires, puis vérifie tailles d'observation et d'état, récompenses et déplacement des 3 drones."""
    cfg = SwarmQREnvCfg()
    cfg.scene.num_envs = args.num_envs
    env = SwarmQREnv(cfg)
    obs, _ = env.reset()
    start = [env._drones[k].data.root_pos_w.clone() for k in range(NUM_DRONES)]

    def rand_actions():
        """Renvoie des actions aléatoires uniformes dans [-1, 1] pour les 3 drones."""
        return {a: torch.empty((env.num_envs, 4), device=env.device).uniform_(-1.0, 1.0) for a in AGENTS}

    rew = term = None
    for _ in range(args.steps):
        obs, rew, term, trunc, info = env.step(rand_actions())

    state = env._get_states()
    moved = [(env._drones[k].data.root_pos_w - start[k]).norm(dim=-1).mean().item() for k in range(NUM_DRONES)]

    print("\n=============== SMOKE TEST ESSAIM ===============")
    print(f"agents          : {env.possible_agents}")
    for a in AGENTS:
        print(f"  obs[{a}]    : {tuple(obs[a].shape)}  (attendu (N,{OBS_DIM}))")
    print(f"état global     : {tuple(state.shape)}  (attendu (N,{STATE_DIM}))")
    print(f"récompense      : {list(rew.keys())}  shape {tuple(rew[AGENTS[0]].shape)}")
    print(f"  valeur moy/drone : {[round(rew[a].mean().item(), 3) for a in AGENTS]}")
    print(f"  approche moy     : {env._approach.mean().item():.4f} (m gagnés vers QR / pas)")
    print(f"déplacement/drone : {[round(m, 2) for m in moved]} m")
    print(f"cartons QR      : {env._n_cartons}  | read_frac moy = {env._read_frac.mean().item():.3f}")
    ok = (
        all(tuple(obs[a].shape) == (env.num_envs, OBS_DIM) for a in AGENTS)
        and tuple(state.shape) == (env.num_envs, STATE_DIM)
        and all(m > 0.1 for m in moved)
        and all(torch.isfinite(rew[a]).all().item() for a in AGENTS)
    )
    print("RÉSULTAT :", "OK — 3 drones bougent, obs/état/récompense corrects ✅" if ok else "À VÉRIFIER")
    print("================================================\n")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
