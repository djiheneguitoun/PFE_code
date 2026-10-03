#!/usr/bin/env python3
"""Phase 11 : premier passage à Isaac Sim + Pegasus — fait apparaître N drones Iris dans un entrepôt.

Les drones n'ont aucun pilote automatique (pas de « backend » de commande) : ils restent posés.
On vérifie seulement que la scène, l'entrepôt et les modèles de drones se chargent.
Lancé par 11_launch_isaac_sim_drones.sh, ou : python scripts/11_isaac_sim_drones.py --num-drones 3 [--headless]
"""

import argparse
import signal
import sys

# Affiche chaque ligne tout de suite (sortie lisible même redirigée vers un fichier).
sys.stdout.reconfigure(line_buffering=True)


def parse_args():
    """Lit les options : --num-drones (3), --headless (sans fenêtre), --spacing (écart entre drones, 3 m)."""
    parser = argparse.ArgumentParser(description="Isaac Sim multi-drone simulation")
    parser.add_argument("--num-drones", type=int, default=3)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--spacing", type=float, default=3.0)
    return parser.parse_args()


def create_sim_app(headless):
    """Démarre Isaac Sim (fenêtre 1280×720, ou sans fenêtre) sans contrôle de version du pilote NVIDIA ; renvoie l'application."""
    from isaacsim import SimulationApp

    config = {
        "headless": headless,
        "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"],
    }
    if not headless:
        config["width"] = 1280
        config["height"] = 720
    return SimulationApp(config)


def setup_scene():
    """Crée le monde Pegasus avec un sol, ajoute l'entrepôt « Simple_Warehouse » (premier chemin USD qui se charge) et renvoie le monde."""
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

    try:
        from isaacsim.core.utils.stage import add_reference_to_stage
    except ImportError:
        from omni.isaac.core.utils.stage import add_reference_to_stage

    pegasus_if = PegasusInterface()
    pegasus_if.initialize_world()
    world = pegasus_if.world
    world.scene.add_default_ground_plane()

    warehouse_assets = [
        "omniverse://localhost/NVIDIA/Assets/Isaac/4.2/Isaac/Environments/Simple_Warehouse/warehouse.usd",
        "omniverse://localhost/NVIDIA/Assets/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    ]
    for usd in warehouse_assets:
        try:
            add_reference_to_stage(usd_path=usd, prim_path="/World/Warehouse")
            break
        except Exception:
            continue

    return world


def create_drones(num_drones, spacing):
    """Place `num_drones` drones Iris en ligne sur l'axe x (écart `spacing` m, rangée centrée sur 0, z = 0,1 m) et renvoie la liste."""
    from pegasus.simulator.params import ROBOTS
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

    drones = []
    for i in range(num_drones):
        x = -((num_drones - 1) * spacing / 2.0) + i * spacing
        config = MultirotorConfig()
        config.backends = []

        drone = Multirotor(
            f"/World/Drone_{i:02d}",
            ROBOTS["Iris"],
            i,
            [x, 0.0, 0.1],
            [0.0, 0.0, 0.0, 1.0],
            config,
        )
        drones.append(drone)
        print(f"[INFO] Drone {i} @ ({x:.1f}, 0.0, 0.1)")
    return drones


def main():
    """Lance Isaac Sim, crée la scène et les drones, puis fait avancer la physique jusqu'à Ctrl+C."""
    args = parse_args()

    running = True
    def on_signal(sig, frame):
        """Demande l'arrêt propre de la boucle de simulation (Ctrl+C ou SIGTERM)."""
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print(f"[INFO] Chargement Isaac Sim (mode {'headless' if args.headless else 'GUI'})...")
    sim_app = create_sim_app(args.headless)

    world = setup_scene()
    create_drones(args.num_drones, args.spacing)

    world.reset()
    print(f"[INFO] Simulation active : {args.num_drones} drone(s). Ctrl+C pour arrêter.")

    while running and sim_app.is_running():
        world.step(render=not args.headless)

    print("[INFO] Fermeture...")
    sim_app.close()


if __name__ == "__main__":
    main()
