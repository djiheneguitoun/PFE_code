"""Test 4 (étape 1) : combien de pas de simulation par seconde tient la scène complète ?

La physique ArduPilot tourne à 1/800 s : le temps réel correspond à 800 pas/s. Sur la scène
complète de la graine 7 (3 drones, 9 caméras), on chronomètre trois régimes : sans rendu (vol
et commandes), rendu à 5 images/s (mission), rendu à chaque pas (référence).
  $PY swarm_qr/experiments/04_debit/run.py           (les trois régimes ; --steps N, 1600 par défaut)
  $PY swarm_qr/experiments/04_debit/run.py --plot    (affiche le tableau, sans Isaac Sim)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent  # dossier du test
ROOT = HERE.parents[2]                  # racine du projet, ajoutée au chemin d'import (swarm_qr)
sys.path.insert(0, str(ROOT))
OUT = HERE / "mesures.json"             # une entrée par régime : pas/s et pas de physique

sys.stdout.reconfigure(line_buffering=True)

# --steps : nombre de pas chronométrés par régime (8 fois moins, 200 au moins, pour le rendu à
# chaque pas).
parser = argparse.ArgumentParser()
parser.add_argument("--steps", type=int, default=1600)
parser.add_argument("--plot", action="store_true")
args, _ = parser.parse_known_args()


def plot() -> int:
    """Affiche, pour chaque régime de mesures.json, les pas/s, la vitesse par rapport au temps
    réel et la durée de calcul d'une mission de 10 min."""
    rows = json.loads(OUT.read_text())
    print(f"{'régime':<28} {'pas/s':>8} {'x temps réel':>13} {'mission 10 min':>15}")
    for r in rows:
        rt = r["steps_per_s"] * r["phys_dt"]
        print(f"{r['regime']:<28} {r['steps_per_s']:>8.0f} {rt:>12.2f}x {600/max(rt,1e-9)/60:>12.1f} min")
    return 0


if args.plot:
    raise SystemExit(plot())

from isaacsim import SimulationApp  # noqa: E402

# Isaac Sim doit démarrer avant tout import de omni.* et de swarm_qr.env.
simulation_app = SimulationApp(
    {"headless": True, "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"]}
)

import traceback  # noqa: E402

import omni.timeline  # noqa: E402

from swarm_qr.env import scene as scene_mod  # noqa: E402
from swarm_qr.env.layout import make_layout  # noqa: E402
from swarm_qr.env.pilot import PHYS_DT  # noqa: E402

RENDER_EVERY_5HZ = max(1, round(1.0 / (5.0 * PHYS_DT)))  # pas entre deux rendus : 160 à 1/800 s


def bench(world, steps: int, render_every: int | None) -> float:
    """Fait `steps` pas (un rendu tous les `render_every` pas, aucun si None) et renvoie le
    nombre de pas par seconde (temps réel)."""
    t0 = time.perf_counter()
    for i in range(steps):
        render = render_every is not None and i % render_every == 0
        world.step(render=render)
    return steps / (time.perf_counter() - t0)


def main() -> None:
    """Construit la scène complète de la graine 7, chronomètre les trois régimes et écrit
    mesures.json ; affiche une ligne [RESULTAT] par régime."""
    scene = scene_mod.build(make_layout(7), with_sitl=False)
    scene.world.reset()
    scene.finalize()
    omni.timeline.get_timeline_interface().play()

    for _ in range(30):
        scene.world.step(render=True)

    regimes = [
        ("sans rendu", None),
        ("rendu 5 images/s", RENDER_EVERY_5HZ),
        ("rendu chaque pas", 1),
    ]
    rows = []
    for name, every in regimes:
        # Le rendu à chaque pas est très lent : 8 fois moins de pas, 200 au moins.
        n = args.steps if every != 1 else max(200, args.steps // 8)
        sps = bench(scene.world, n, every)
        rows.append({"regime": name, "steps_per_s": round(sps, 1), "phys_dt": PHYS_DT})
        print(f"[RESULTAT] {name:<20} : {sps:7.1f} pas/s ({sps * PHYS_DT:.2f}x le temps reel)")

    OUT.write_text(json.dumps(rows, indent=2))


try:
    main()
except Exception:
    traceback.print_exc()
finally:
    simulation_app.close()
