"""Rend 3 images RGB 1280×720 de l'entrepôt (vue d'ensemble, gros plan sur des cartons, allée) pour distinguer cartons et bacs KLT.

Les images (view_*.png) vont dans le dossier OUT (sur la machine de simulation).
Lancement (avec fenêtre, devant l'écran de la machine) : bash rl_inventory/launch.sh rl_inventory/diag/render_view.py
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import os

import numpy as np
import omni.replicator.core as rep
import omni.usd

# entrepôt : variable AIF_FACTORY_USD, sinon fichier du serveur public de NVIDIA
USD = os.getenv(
    "AIF_FACTORY_USD",
    "http://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/4.2/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
)
# dossier de sortie des images (dossier personnel de la machine de simulation)
OUT = "/home/djihene_guitoun/simulation_mc02/rl_inventory"

omni.usd.get_context().open_stage(USD)
for _ in range(30):
    simulation_app.update()

# (position de la caméra, point visé, nom) — coordonnées en m dans le repère de l'entrepôt
views = [
    ((-10.0, -10.0, 7.0), (2.0, 6.0, 1.5), "overview"),
    ((3.0, 11.0, 2.2), (9.0, 11.0, 1.4), "cardbox_closeup"),
    ((-2.0, 2.0, 2.0), (-10.0, 8.0, 1.5), "aisle"),
]

for pos, tgt, name in views:
    cam = rep.create.camera(position=pos, look_at=tgt)
    rp = rep.create.render_product(cam, (1280, 720))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([rp])
    for _ in range(45):
        simulation_app.update()
    data = np.asarray(rgb.get_data())
    path = f"{OUT}/view_{name}.png"
    try:
        from PIL import Image

        Image.fromarray(data[..., :3]).save(path)
        print(f"SAVED {path}  shape={data.shape}")
    except Exception as e:
        np.save(path.replace(".png", ".npy"), data)
        print(f"PIL KO ({e}) -> {path.replace('.png', '.npy')}")
    rgb.detach([rp])
    rp.destroy()

simulation_app.close()
