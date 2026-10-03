"""Test 3 (étape 1) : le drone piloté par ArduPilot SITL lit-il les QR en volant ?

Un seul vol. La planche : le drone se place à six distances (0,5 à 3 m) face à un panneau,
photographie, et on vérifie que c'est bien la cible visée qui est lue, pas un voisin. La vidéo :
il longe le rack en lisant en continu (compteur des codes différents lus). Lancé par run_all.sh,
ou seul depuis la racine du projet (DISPLAY : ArduPilot ouvre un terminal) :
  DISPLAY=:1 $PY swarm_qr/experiments/03_images_qr/run.py --seed 7
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent  # dossier du test : toutes les sorties y sont écrites
ROOT = HERE.parents[2]                  # racine du projet, ajoutée au chemin d'import (swarm_qr)
sys.path.insert(0, str(ROOT))

sys.stdout.reconfigure(line_buffering=True)

# --speed : vitesse le long du rack pendant la vidéo (m/s) ; --standoff : distance entre la
# caméra et le plan du panneau visé pendant la vidéo (m).
parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--speed", type=float, default=0.5)
parser.add_argument("--standoff", type=float, default=1.2)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": True, "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"]}
)

import traceback  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from swarm_qr import perception as P
from swarm_qr.env.config import CAMERAS  # noqa: E402
from swarm_qr.env import scene as scene_mod  # noqa: E402
from swarm_qr.env.layout import make_layout  # noqa: E402
from swarm_qr.env.pilot import Pilot  # noqa: E402
from swarm_qr.experiments import _img, _viz  # noqa: E402
from swarm_qr.experiments._vol import transit  # noqa: E402

DISTANCES = (0.5, 0.8, 1.1, 1.5, 2.0, 3.0)  # m : distances caméra-panneau de la planche
FLY_ALT = 1.6                      # m : altitude de décollage
CAM_LATERAL = CAMERAS.side_offset  # m : caméra gauche à 10 cm du centre du drone, sur le côté...
CAM_BAS = CAMERAS.below            # m : ... et 11 cm plus bas


def codes_lus(bgr) -> list[str]:
    """Renvoie les textes des QR lus dans l'image (décodeur zxing de swarm_qr.perception, réglé
    pour ne lire que les QR)."""
    gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return [t for t, _ in P.DECODEURS["zxing"](gris)]


def vue_de_dessus(scene, overview):
    """Renvoie la vue de dessus en BGR dès qu'elle n'est plus vide, en faisant avancer le rendu
    (30 essais au plus, sinon erreur)."""
    for _ in range(30):
        img = overview.get_rgb()
        if img is not None and getattr(img, "ndim", 0) == 3 and img.size:
            return _img.to_bgr(img)
        scene.world.step(render=True)
    raise RuntimeError("vue de dessus vide")


def main() -> None:
    """Fait décoller le drone, photographie le panneau visé à six distances (planche_qr.jpg), puis
    longe le rack en filmant (vol_le_long_du_rack.mp4) ; écrit resultat.json."""
    layout = make_layout(args.seed)
    scene = scene_mod.build(layout, with_sitl=True, n_drones=1)
    overview = _viz.overview_camera()
    _viz.hide_roof(omni.usd.get_context().get_stage())
    scene.world.reset()
    scene.finalize()
    _viz.overview_init(overview)
    omni.timeline.get_timeline_interface().play()

    def get_pos():
        """Renvoie la position vraie du drone 0 (lue dans le simulateur), en mètres."""
        return scene.drones[0].state.position

    def get_yaw():
        """Renvoie le cap (lacet) vrai du drone 0, en radians."""
        return float(Rotation.from_quat(scene.drones[0].state.attitude).as_euler("ZYX")[0])

    pilot = Pilot(scene.world, 0)
    if not pilot.ready(FLY_ALT, lambda: float(get_pos()[2])):
        print("ECHEC : le drone n'a pas decolle")
        return

    # Cible : un panneau tourné vers +x, entre 1 et 3 m de haut, le plus proche de 1,6 m.
    bons = [t for t in scene.tags if t.normal[0] > 0.9 and 1.0 < t.position[2] < 3.0]
    tag = sorted(bons or list(scene.tags), key=lambda t: abs(t.position[2] - FLY_ALT))[0]
    n = np.array(tag.normal, float)
    psi = math.atan2(n[1], n[0]) + math.pi / 2.0        # caméra gauche face au panneau
    lateral = np.array([-math.sin(psi), math.cos(psi), 0.0]) * CAM_LATERAL
    print(f"QR vise : {tag.tag_id} a {tuple(round(c, 2) for c in tag.position)} "
          f"taille {tag.size:.2f} m")

    transit(pilot, layout, tag.position[0] + 2.0, tag.position[2] + CAM_BAS,
            psi, get_pos, get_yaw)

    # --- planche : une photo par distance, identite de la cible verifiee ---
    tuiles, mesures = [], []
    for d in DISTANCES:
        cible_cam = np.array(tag.position, float) + n * d
        cible_cam[2] = max(0.8, tag.position[2])
        # Point visé pour le centre du drone : on compense le décalage de la caméra, sur le côté
        # ET en hauteur (l'ancienne visée ne compensait que la hauteur).
        cible = cible_cam - lateral + np.array([0.0, 0.0, CAM_BAS])
        # 90 s : le premier trajet fait 16 m depuis le point d'apparition, plus la rotation
        atteint = pilot.goto(cible, psi, get_pos, tol=0.15, get_yaw=get_yaw, timeout_sim_s=90.0)
        pilot.pump(1.0)
        img = _img.to_bgr(scene.capture("left", 0))
        vus = codes_lus(img)
        ok = tag.tag_id in vus
        voisins = [t for t in vus if t != tag.tag_id]
        err = float(np.linalg.norm(cible - np.array(get_pos())))
        mesures.append({"distance": d, "atteint": bool(atteint), "cible_lue": ok,
                        "voisins_lus": voisins, "err_pos_m": round(err, 3)})
        etat = f"CIBLE LUE {tag.tag_id}" if ok else "cible non lue"
        tuiles.append(_viz.label(img, f"{d:.1f} m - {etat}"))
        print(f"  {d:.1f} m : {etat}"
              f"{' (+ voisins ' + ','.join(voisins) + ')' if voisins else ''}"
              f"{'' if atteint else ' (point non atteint)'} | pos {err:.2f} m")

    _viz.board(tuiles, HERE / "planche_qr.jpg", cols=3, cell=460)

    # --- video : longer le rack en lisant en continu ---
    rack = min(layout.racks, key=lambda r: abs(r.x - (tag.position[0] - 0.7)))
    y0, y1 = rack.y_bounds
    depart = np.array([tag.position[0] + args.standoff, y0 - 1.0,
                       tag.position[2] + CAM_BAS])
    pilot.goto(depart - lateral, psi, get_pos, tol=0.2, get_yaw=get_yaw, timeout_sim_s=90.0)

    # Vitesse +y du monde (le long du rack) et cap, convertis dans le repère NED d'ArduPilot
    # (nord, est, bas).
    R = pilot._ned_in_world
    v_ned = R.T @ np.array([0.0, args.speed, 0.0])
    cw = np.array([math.cos(psi), math.sin(psi), 0.0])
    psi_ned = float(np.arctan2(np.dot(cw, R[:, 1]), np.dot(cw, R[:, 0])))

    images, lus = [], set()
    fin_y = y1 + 1.0
    # Garde-fou : au plus 3 fois le nombre d'images prévu (une toutes les 0,2 s), plus 100.
    garde = int((fin_y - y0 + 1.0) / args.speed / 0.2 * 3) + 100
    while float(get_pos()[1]) < fin_y and len(images) < garde:
        pilot.velocity(float(v_ned[0]), float(v_ned[1]), float(v_ned[2]), yaw_rad=psi_ned)
        pilot.pump(0.2)
        cam = _img.to_bgr(scene.capture("left", 0))
        vus = codes_lus(cam)
        lus.update(vus)
        p = get_pos()
        cam = _viz.label(cam, f"camera gauche - {len(lus)} QR lus")
        if vus:
            cam = _viz.label(cam, f"lu : {', '.join(sorted(vus))}", bottom=True)
        top = _viz.label(vue_de_dessus(scene, overview),
                         f"vue de dessus - drone x={p[0]:.1f} y={p[1]:.1f}")
        images.append(cv2.resize(_viz.side_by_side(cam, top), (1280, 480)))
    pilot.velocity(0.0, 0.0, 0.0)
    pilot.pump(0.5)

    chemin = _viz.video(images, HERE / "vol_le_long_du_rack.mp4", fps=8)
    print(f"video : {chemin.name}, {len(images)} images, {len(lus)} QR lus pendant le vol")

    (HERE / "resultat.json").write_text(json.dumps({
        "socle": "pegasus+ardupilot_sitl",
        "tag": tag.tag_id,
        "tag_size_m": round(tag.size, 3),
        "distances": mesures,
        "portee_max_m": max([m["distance"] for m in mesures if m["cible_lue"]], default=0.0),
        "qr_lus_en_vol": sorted(lus),
        "images_video": len(images),
    }, indent=2))


try:
    main()
except Exception:
    traceback.print_exc()
finally:
    simulation_app.close()
