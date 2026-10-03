"""Mesure jusqu'où et sous quel angle le vrai décodeur OpenCV (celui de Pore et al.) lit un QR de l'entrepôt.

Rend des images caméra à distances et angles contrôlés, affiche les seuils conseillés et écrit docs/calibration_gate.csv.
  PYTHONUNBUFFERED=1 ~/isaac5_env/bin/python rl_inventory/swarmscan_map/calibrate_gate.py \
      --headless --enable_cameras --kit_args="--/rtx/verifyDriverVersion/enabled=false"
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 960))
parser.add_argument("--fov_deg", type=float, default=60.0, help="FOV horizontal caméra (= gate)")
parser.add_argument("--settle_frames", type=int, default=12, help="frames de rendu par pose")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
# Isaac Sim doit démarrer AVANT d'importer omni, pxr, isaaclab et le reste du projet
simulation_app = AppLauncher(args).app

import csv
import math
import os
import sys

import cv2
import numpy as np
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, UsdGeom

import isaaclab.sim as sim_utils
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from rl_inventory.config_rl import CFG  # noqa: E402
from rl_inventory.qr_task import FACES, attach_qr_to_cartons, carton_qr_world_poses, find_cartons  # noqa: E402

DOCS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "docs"))  # dossier docs/ à la racine (sortie du CSV)


def look_at_matrix(eye: np.ndarray, target: np.ndarray) -> Gf.Matrix4d:
    """Renvoie la matrice de pose d'une caméra USD placée en `eye` et visant `target` (elle regarde selon −Z)."""
    fwd = target - eye
    fwd = fwd / np.linalg.norm(fwd)
    z = -fwd
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, z)
    x = x / max(np.linalg.norm(x), 1e-9)
    y = np.cross(z, x)
    m = Gf.Matrix4d(
        x[0], x[1], x[2], 0.0,
        y[0], y[1], y[2], 0.0,
        z[0], z[1], z[2], 0.0,
        eye[0], eye[1], eye[2], 1.0,
    )
    return m


def main():
    """Construit la scène, choisit un QR dégagé, balaie la distance puis l'angle, écrit le CSV et affiche les seuils conseillés."""
    create_new_stage()
    stage = omni.usd.get_context().get_stage()
    # entrepôt chargé depuis l'URL de config_rl.py (serveur de contenu Omniverse : accès Internet requis)
    add_reference_to_stage(CFG.scene.warehouse_usd, "/World/Warehouse")

    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.95, 0.95, 0.95))
    light.func("/World/Light", light)

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 60.0))

    attach_qr_to_cartons(stage)
    cartons = find_cartons(stage)
    pos, norm = carton_qr_world_poses(stage, cartons)
    pos, norm = np.array(pos), np.array(norm)

    cam = UsdGeom.Camera.Define(stage, "/World/CalibCam")
    aperture = 20.955     # mm : largeur du capteur de la caméra USD ; la focale en découle pour un champ de --fov_deg
    focal = aperture / (2.0 * math.tan(math.radians(args.fov_deg) / 2.0))
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
    xform = UsdGeom.Xformable(cam.GetPrim())
    op = xform.AddTransformOp()

    rp = rep.create.render_product("/World/CalibCam", tuple(args.resolution))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    # décodeur OpenCV « Aruco » s'il existe dans la version installée, sinon le décodeur classique
    try:
        detector = cv2.QRCodeDetectorAruco()
    except AttributeError:
        detector = cv2.QRCodeDetector()
    sim.reset()

    def decode_all(img_bgr: np.ndarray) -> set[str]:
        """Renvoie les textes des QR décodés, image essayée telle quelle puis agrandie ×3 (OpenCV peine en basse résolution)."""
        found: set[str] = set()
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        for im in (gray, cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)):
            ok, texts, _, _ = detector.detectAndDecodeMulti(im)
            if ok:
                found |= {t for t in texts if t}
        return found

    def render_decode(eye: np.ndarray, target: np.ndarray, expected: str) -> bool:
        """Place la caméra, rend l'image (--settle_frames rendus, 12 par défaut) et renvoie vrai si le QR attendu est décodé."""
        op.Set(look_at_matrix(eye, target))
        for _ in range(args.settle_frames):
            sim.render()
        img = np.asarray(annot.get_data())[..., :3].astype(np.uint8)
        return expected in decode_all(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

    # 1. choisir une face de test dégagée : parmi les 40 QR les plus proches de 1,4 m de haut,
    #    le premier dont le QR est décodé de face à 1,5 m
    heights = pos[:, 2]
    order = np.argsort(np.abs(heights - 1.4))
    face_idx = None
    for i in order[:40]:
        expected = f"CARTON_{int(i) // len(FACES):04d}"
        eye = pos[i] + norm[i] * 1.5
        if 0.5 < eye[2] < 3.0 and render_decode(eye, pos[i], expected):
            face_idx = int(i)
            break
    if face_idx is None:
        print("CALIBRATION IMPOSSIBLE : aucune face décodée à 1.5 m — vérifier textures/lumière.")
        return
    p, n = pos[face_idx], norm[face_idx]
    target_id = f"CARTON_{face_idx // len(FACES):04d}"
    print(f"face de test : idx={face_idx} ({target_id}), pos={np.round(p, 2).tolist()}, z={p[2]:.2f} m")

    rows, trials = [], 2

    def success_rate(eye):
        """Renvoie le taux de décodage du QR de test vu depuis `eye` (sur 2 essais)."""
        return sum(render_decode(eye, p, target_id) for _ in range(trials)) / trials

    # 2. distance max, de face : de 0,75 à 5 m par pas de 0,25 m ; « décodé » = au moins 50 % ;
    #    arrêt dès qu'on dépasse de 0,75 m la dernière distance réussie
    d_ok = 0.0
    for d in np.arange(0.75, 5.01, 0.25):
        rate = success_rate(p + n * d)
        rows.append(("distance", round(float(d), 2), 0, rate))
        print(f"  d={d:4.2f} m  angle=0°   décodage={rate:.0%}")
        if rate >= 0.5:
            d_ok = float(d)
        elif d > d_ok + 0.75:
            break

    # 3. angle max, à 2/3 de la distance max : de 0 à 75° par pas de 5° (caméra tournée autour
    #    du QR, à plat) ; arrêt 15° après le dernier angle réussi
    d_test = max(0.75, round(d_ok * 2 / 3 / 0.25) * 0.25)
    a_ok = 0.0
    side = np.cross(np.array([0.0, 0.0, 1.0]), n)
    side /= max(np.linalg.norm(side), 1e-9)
    for a in np.arange(0.0, 75.1, 5.0):
        rad = math.radians(a)
        eye = p + (n * math.cos(rad) + side * math.sin(rad)) * d_test
        rate = success_rate(eye)
        rows.append(("angle", round(float(d_test), 2), int(a), rate))
        print(f"  d={d_test:4.2f} m  angle={a:3.0f}°  décodage={rate:.0%}")
        if rate >= 0.5:
            a_ok = float(a)
        elif a > a_ok + 15:
            break

    os.makedirs(DOCS, exist_ok=True)
    out = os.path.join(DOCS, "calibration_gate.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sweep", "distance_m", "angle_deg", "decode_rate"])
        w.writerows(rows)

    # seuils conseillés avec marge : 85 % de la distance max, angle max − 5°
    reco_d = round(d_ok * 0.85, 2)
    reco_a = max(0.0, a_ok - 5.0)
    print("\n=============== CALIBRATION GATE (cv2.QRCodeDetector, décodeur de Pore) ===============")
    print(f"distance max décodée      : {d_ok:.2f} m   → seuil recommandé read_distance_m = {reco_d}")
    print(f"angle max décodé (@{d_test:.2f} m) : {a_ok:.0f}°     → seuil recommandé view_angle_deg = {reco_a:.0f}")
    print("vitesse : non mesurable en rendu statique → loi de Cristiani 2020 (P=1/(v+1)^kr), seuil déclaré 0.6 m/s")
    print(f"CSV : {out}")
    print("→ reporter ces valeurs dans swarmscan_map/config_map.py:GateConfig puis relancer les tests.")
    print("=======================================================================================\n")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
