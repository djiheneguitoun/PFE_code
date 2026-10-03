"""Test 5, sonde manuelle avec fenêtre : un drone ArduPilot SITL dans l'entrepôt.

Ouvre Isaac Sim en fenêtre, charge l'entrepôt et un drone Iris ; un terminal MAVProxy (console
de commande d'ArduPilot) s'ouvre seul : y taper `mode guided`, `arm throttle`, `takeoff 3`. La
position du drone s'affiche ici toutes les 2 s. Arrêt : Ctrl+C ou fermer la fenêtre (étapes et
origine de la recette : A_LIRE_POUR_LE_PROMOTEUR/README.md). Depuis la racine du projet :
  DISPLAY=:1 ~/isaac5_env/bin/python swarm_qr/experiments/05_sitl/sonde_gui.py
"""

from __future__ import annotations

import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]  # racine du projet, ajoutée au chemin d'import (swarm_qr)
sys.path.insert(0, str(ROOT))

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {
        "headless": False,
        "width": 1280,
        "height": 720,
        "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"],
    }
)

# Tout ce qui suit exige que l'application soit démarrée.
import omni.timeline  # noqa: E402
from isaacsim.core.api.world import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (  # noqa: E402
    ArduPilotMavlinkBackend,
    ArduPilotMavlinkBackendConfig,
)
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS, WORLD_SETTINGS  # noqa: E402

from swarm_qr.env.config import WAREHOUSE_PRIM, WAREHOUSE_USD  # noqa: E402

SPAWN = (-5.0, 0.0, 0.10)  # m (x, y, z) : dans l'allée ouest, dégagée du sol au plafond


def build_world(pg: PegasusInterface) -> World:
    """Crée et renvoie le monde Isaac au pas de physique officiel d'ArduPilot (1/800 s), et non
    à celui de l'exemple Pegasus."""
    pg._world = World(**WORLD_SETTINGS["ardupilot"])
    return pg.world


def load_scene(world: World) -> None:
    """Charge notre entrepôt (adresse directe, déjà en cache local, jamais les décors en ligne de
    Pegasus), le sol et une lumière d'ambiance."""
    add_reference_to_stage(usd_path=WAREHOUSE_USD, prim_path=WAREHOUSE_PRIM)
    world.scene.add_default_ground_plane()

    from pxr import UsdLux

    import omni.usd

    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(2500.0)


def create_drone(pg: PegasusInterface) -> Multirotor:
    """Crée et renvoie le drone Iris relié à ArduPilot SITL, que Pegasus lance automatiquement."""
    backend = ArduPilotMavlinkBackend(
        config=ArduPilotMavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "ardupilot_autolaunch": True,
                "ardupilot_dir": pg.ardupilot_path,
                "ardupilot_vehicle_model": "gazebo-iris",
            }
        )
    )
    config = MultirotorConfig()
    config.backends = [backend]

    return Multirotor(
        "/World/Drone_00",
        ROBOTS["Iris"],
        0,
        list(SPAWN),
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=config,
    )


def main() -> None:
    """Construit la scène, lance la simulation et affiche la position du drone toutes les 2 s,
    jusqu'à Ctrl+C ou la fermeture de la fenêtre."""
    running = True

    def on_signal(sig, frame):
        """Demande l'arrêt propre de la boucle (Ctrl+C ou signal d'arrêt)."""
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    world = build_world(pg)

    print("[SONDE] chargement de l'entrepot...")
    load_scene(world)
    drone = create_drone(pg)
    world.reset()

    print("[SONDE] scene prete, demarrage de la simulation")
    print("[SONDE] un terminal MAVProxy va s'ouvrir : y taper")
    print("[SONDE]   mode guided   puis   arm throttle   puis   takeoff 3")
    timeline.play()

    last_print = 0.0
    while running and simulation_app.is_running():
        world.step(render=True)
        now = time.monotonic()
        if now - last_print >= 2.0:
            p = drone.state.position
            print(f"[SONDE] drone x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:+.2f} m")
            last_print = now

    print("[SONDE] fermeture...")
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
