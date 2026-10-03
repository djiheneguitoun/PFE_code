"""Test T1.5a sans rendu : colle un QR sur chaque carton de l'entrepôt et affiche le nombre de QR posés et d'images générées.

Les images vont dans assets/qr/ ; aucun rendu, aucune fenêtre.
Lancement depuis la racine : bash rl_inventory/launch.sh rl_inventory/diag/test_qr_attach.py --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import os
import sys

import omni.usd

# rend le paquet rl_inventory importable (racine du projet = deux dossiers au-dessus)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from rl_inventory.qr_task import QR_DIR, attach_qr_to_cartons  # noqa: E402

# entrepôt : variable AIF_FACTORY_USD, sinon fichier du serveur public de NVIDIA
USD = os.getenv(
    "AIF_FACTORY_USD",
    "http://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/4.2/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
)

omni.usd.get_context().open_stage(USD)
stage = omni.usd.get_context().get_stage()
info = attach_qr_to_cartons(stage)

print(f"\nQR posés sur {len(info)} cartons")
for box_id, path in list(info.items())[:3]:
    tag = stage.GetPrimAtPath(path + "/QRTag")
    print(f"  {box_id}: {path}/QRTag  valide={tag.IsValid()} type={tag.GetTypeName()}")
n_png = len([f for f in os.listdir(QR_DIR) if f.endswith(".png")]) if os.path.isdir(QR_DIR) else 0
print(f"PNG QR générés : {n_png} dans {QR_DIR}")

simulation_app.close()
