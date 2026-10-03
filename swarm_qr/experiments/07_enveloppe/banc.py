"""Banc de l'étape 2 (Isaac Sim) : photographie un QR depuis des milliers de poses connues.

Modes : optique (caméra seule, poses au hasard), sans-qr (mêmes poses, QR cachés), vol (12 poses
tenues par le drone), traversee (le drone longe le rack à 4 vitesses). Chaque image est notée avec
la pose VRAIE de la caméra dans poses_<nom>.jsonl ; le banc s'arrête si un auto-contrôle échoue.
Lancé par campagne.sh ; seul : DISPLAY=:1 $PY banc.py --mode optique --poses 2000
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))

sys.stdout.reconfigure(line_buffering=True)

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=("optique", "sans-qr", "vol", "traversee"),
                    default="optique")
parser.add_argument("--seed", type=int, default=9033)
parser.add_argument("--poses", type=int, default=2000)
parser.add_argument("--graine-tirage", type=int, default=20260901)
parser.add_argument("--nom", default=None, help="nom de la campagne (fichiers de sortie)")
args, _ = parser.parse_known_args()

# Sorties, écrites dans ce dossier : images_<nom>/, poses_<nom>.jsonl et meta_<nom>.json.
NOM = args.nom or args.mode.replace("-", "_")
SORTIE = HERE / f"images_{NOM}"
MANIFESTE = HERE / f"poses_{NOM}.jsonl"
META = HERE / f"meta_{NOM}.json"

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": True, "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"]}
)

import traceback  # noqa: E402

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from swarm_qr import perception as P  # noqa: E402
from swarm_qr.env import scene as scene_mod  # noqa: E402
from swarm_qr.env.config import CAMERAS, INTERIOR, RACKS  # noqa: E402
from swarm_qr.env.layout import make_layout  # noqa: E402
from swarm_qr.env.pilot import Pilot  # noqa: E402
from swarm_qr.experiments import _img  # noqa: E402
from swarm_qr.experiments._vol import transit  # noqa: E402

# --- géométrie du tirage (modes optique et sans-qr) ---
D_MIN, D_MAX = 0.45, 8.0            # m : distance caméra-panneau, tirée selon une loi log-uniforme
ALPHA_MAX = 75.0                    # degrés : angle maximal entre la visée et la normale du panneau
VISEE_H, VISEE_V = 24.0, 17.0       # degrés : décentrage maximal du panneau dans l'image (visée imparfaite)
ROULIS_SIGMA, ROULIS_MAX = 3.0, 8.0  # degrés : roulis de la caméra (écart-type, plafond)
MARGE_MUR = 0.60                    # m : distance minimale d'une pose tirée aux murs et aux racks

# --- vol ---
FLY_ALT = 1.6                       # m : altitude de décollage
CAM_LATERAL = CAMERAS.side_offset   # m : la caméra gauche est à 10 cm du centre du drone...
CAM_BAS = CAMERAS.below             # m : ...et 11 cm plus bas
DEGAGEMENT_MIN = 0.50     # m : écart minimal au rack = hélices 0,26 m + oscillation mesurée 0,21 m + marge
# Les 12 poses tenues en vol : (angle à la normale en degrés, distance caméra-panneau en m).
POSES_VOL = ([(0.0, d) for d in (0.90, 1.20, 1.60, 2.10, 2.70, 3.50)]
             + [(30.0, 1.20), (30.0, 2.10), (30.0, 3.50),
                (-45.0, 1.20), (-45.0, 2.10), (-45.0, 3.50)])
IMAGES_PAR_POSE = 15                # images prises à chaque pose tenue

# --- traversée ---
VITESSES = (0.1, 0.3, 0.6, 1.0)     # m/s : vitesses des 4 traversées
# 160 étapes de physique entre deux images = 0,2 s simulée, la cadence de mission (5 images/s).
PHYS_PAR_IMAGE = 160
# m : la traversée couvre 4,5 m de part et d'autre du panneau (la zone lisible), pas les 20 m du rack.
DEMI_FENETRE = 4.5


# ---------------------------------------------------------------- outils communs

def quat_visee(position, cible, bh_deg=0.0, bv_deg=0.0, roulis_deg=0.0):
    """Renvoie l'orientation (quaternion w, x, y, z) d'une caméra placée en `position` qui vise `cible`.
    `bh_deg`/`bv_deg` décentrent la cible dans l'image et `roulis_deg` tourne l'image (degrés)."""
    avant = np.array(cible, float) - np.array(position, float)
    avant /= max(np.linalg.norm(avant), 1e-9)
    haut_ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(avant, haut_ref))) > 0.98:
        haut_ref = np.array([0.0, 1.0, 0.0])
    gauche = np.cross(haut_ref, avant)
    gauche /= max(np.linalg.norm(gauche), 1e-9)
    haut = np.cross(avant, gauche)
    R = np.stack([avant, gauche, haut], axis=1)
    R = R @ Rotation.from_euler("ZYX", [bh_deg, -bv_deg, roulis_deg], degrees=True).as_matrix()
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def choisit_cible(scene):
    """Renvoie le panneau tourné vers +X, entre 1,30 et 1,90 m de haut, le plus proche de 1,60 m."""
    bons = [t for t in scene.tags if t.normal[0] > 0.9 and 1.30 < t.position[2] < 1.90]
    if not bons:
        raise RuntimeError("aucun panneau utilisable")
    return sorted(bons, key=lambda t: abs(t.position[2] - 1.60))[0]


def pose_reelle(cam, tag):
    """Relit la pose vraie de la caméra ; renvoie position, quaternion, distance au panneau (m)
    et angle entre la visée et la normale du panneau (degrés)."""
    p, q = cam.get_world_pose()
    p = np.asarray(p, dtype=float)
    v = p - np.array(tag.position, float)
    D = float(np.linalg.norm(v))
    cos_a = float(np.dot(v / max(D, 1e-9), np.array(tag.normal, float)))
    alpha = float(np.degrees(np.arccos(np.clip(abs(cos_a), 0.0, 1.0))))
    return p, [round(float(x), 5) for x in np.asarray(q)], D, alpha


class Journal:
    """Journal d'une campagne : écrit chaque image et sa pose aussitôt, pour qu'un plantage ne perde rien."""

    def __init__(self):
        """Crée le dossier images_<nom>/ et ouvre poses_<nom>.jsonl en écriture."""
        SORTIE.mkdir(parents=True, exist_ok=True)
        self.fichier = MANIFESTE.open("w", buffering=1)
        self.n = 0

    def note(self, cam, tag, image_bgr, run="", indice=None, vitesse=0.0, atteint=True):
        """Enregistre l'image (si elle existe) et une ligne JSON : pose vraie, distance, angle, vitesse."""
        p, q, D, alpha = pose_reelle(cam, tag)
        nom = None
        if image_bgr is not None:
            nom = f"{self.n:05d}.jpg"
            _img.save(image_bgr, SORTIE / nom)
        self.fichier.write(json.dumps({
            "image": nom, "run": run, "indice": self.n if indice is None else indice,
            "D_m": round(D, 4), "alpha_deg": round(alpha, 3),
            "cam_pos": [round(float(x), 4) for x in p], "cam_quat": q,
            "tag": tag.tag_id, "tag_pos": [round(float(x), 4) for x in tag.position],
            "tag_normale": [round(float(x), 3) for x in tag.normal],
            "panneau_m": round(tag.size, 4), "vitesse_ms": round(float(vitesse), 3),
            "atteint": bool(atteint),
        }) + "\n")
        self.n += 1

    def clot(self, K, extra=None):
        """Ferme le journal et écrit meta_<nom>.json (mode, graines, calibration K, nombre de mesures)."""
        self.fichier.close()
        meta = {"mode": args.mode, "seed_entrepot": args.seed,
                "graine_tirage": args.graine_tirage,
                "K": [[round(float(v), 4) for v in r] for r in K], "images": self.n}
        meta.update(extra or {})
        META.write_text(json.dumps(meta, indent=2))
        print(f"\n{self.n} mesures dans {MANIFESTE.name}")


# ---------------------------------------------------------------- auto-verification

def verifie_calibration(cam) -> np.ndarray:
    """Vérifie que la focale relue (fx = fy) égale la valeur calculée à 1 pixel près, sinon toute
    distance serait fausse et le banc s'arrête ; renvoie la matrice de calibration K."""
    K = np.asarray(cam.get_intrinsics_matrix(), dtype=float)
    fx_attendu = CAMERAS.side_width / (2.0 * math.tan(math.radians(CAMERAS.fov_deg) / 2.0))
    if abs(K[0, 0] - fx_attendu) > 1.0 or abs(K[0, 0] - K[1, 1]) > 0.1:
        raise RuntimeError(f"calibration incoherente : fx={K[0,0]:.1f}, attendu {fx_attendu:.1f}")
    print(f"[VERIF] calibration : fx = fy = {K[0,0]:.1f} (attendu {fx_attendu:.1f})")
    return K


def mesure_retard(world, cam, tag, K) -> int:
    """Mesure le retard du rendu : vérifie la lecture à 1,5 m (à 8 cm près), saute à 3 m et renvoie
    le nombre de rendus avant que l'image montre 3 m ; arrêt s'il dépasse la marge RENDER_LAG."""
    cote = P.taille_code(tag.size)

    def distance_vue():
        """Lit le QR visé avec zxing ; renvoie la distance (m) déduite de sa taille en pixels, ou None."""
        import cv2

        img = cam.get_rgb()
        if img is None or getattr(img, "ndim", 0) != 3 or not img.size:
            return None
        gris = cv2.cvtColor(_img.to_bgr(img), cv2.COLOR_BGR2GRAY)
        for texte, coins in P.DECODEURS["zxing"](gris):
            if texte == tag.tag_id:
                c = P._ordonne(coins)
                px = float(np.mean([np.linalg.norm(c[(k + 1) % 4] - c[k]) for k in range(4)]))
                return K[0, 0] * cote / px
        return None

    n = np.array(tag.normal, float)
    for d, label in ((1.5, "depart"), (3.0, "saut")):
        pos = np.array(tag.position, float) + n * d
        cam.set_world_pose(position=pos, orientation=quat_visee(pos, tag.position),
                           camera_axes="world")
        if label == "depart":
            for _ in range(30):
                world.step(render=True)
            vu = distance_vue()
            if vu is None or abs(vu - 1.5) > 0.08:
                raise RuntimeError(f"controle d'aplomb : distance vue {vu} pour 1,50 m")
            print(f"[VERIF] aplomb : lu a 1,50 m, mesure {vu:.3f} m")
        else:
            for k in range(1, 31):
                world.step(render=True)
                vu = distance_vue()
                if vu is not None and abs(vu - 3.0) < 0.20:
                    print(f"[VERIF] retard du rendu : {k} rendus (marge utilisee : "
                          f"{scene_mod.RENDER_LAG})")
                    if k > scene_mod.RENDER_LAG:
                        raise RuntimeError(f"retard {k} > marge {scene_mod.RENDER_LAG}")
                    return k
            raise RuntimeError("l'image ne reflete jamais le saut de position")
    return 0


# ---------------------------------------------------------------- modes sans drone

def hors_zone(p, layout) -> bool:
    """Renvoie vrai si le point est interdit : à moins de 60 cm d'un mur ou d'un rack, ou plus bas
    que 0,4 m ou plus haut que le plafond de vol."""
    if not (INTERIOR.x_min + MARGE_MUR < p[0] < INTERIOR.x_max - MARGE_MUR
            and INTERIOR.y_min + MARGE_MUR < p[1] < INTERIOR.y_max - MARGE_MUR
            and 0.4 < p[2] < INTERIOR.z_fly_max):
        return True
    for r in layout.racks:
        y0, y1 = r.y_bounds
        if (abs(p[0] - r.x) < RACKS.depth / 2.0 + MARGE_MUR
                and y0 - MARGE_MUR < p[1] < y1 + MARGE_MUR and p[2] < RACKS.height):
            return True
    return False


def tire_pose(rng, tag, layout):
    """Tire au hasard une pose de caméra devant le panneau (distance, angle, décentrage, roulis) ;
    renvoie (position, bh, bv, roulis), angles en degrés, ou None après 200 essais ratés."""
    n = np.array(tag.normal, float)
    t = np.cross(np.array([0.0, 0.0, 1.0]), n)
    t /= max(np.linalg.norm(t), 1e-9)
    centre = np.array(tag.position, float)
    for _ in range(200):
        D0 = float(np.exp(rng.uniform(math.log(D_MIN), math.log(D_MAX))))
        a0 = math.radians(float(rng.uniform(0.0, ALPHA_MAX)))
        signe = 1.0 if rng.random() < 0.5 else -1.0
        pos = centre + D0 * (math.cos(a0) * n + signe * math.sin(a0) * t)
        pos[2] += float(rng.uniform(-0.30, 0.30)) * min(D0, 2.0)
        # Distance et angle sont recalculés sur la pose finale : le décalage vertical les change,
        # et enregistrer les valeurs tirées fausserait l'analyse.
        v = pos - centre
        D = float(np.linalg.norm(v))
        cos_a = float(np.dot(v / max(D, 1e-9), n))
        if cos_a <= 0.0:
            continue
        alpha = float(np.degrees(np.arccos(min(1.0, cos_a))))
        if alpha > ALPHA_MAX or D * cos_a < 0.40 or not D_MIN <= D <= D_MAX:
            continue
        if hors_zone(pos, layout):
            continue
        bh = float(rng.uniform(-VISEE_H, VISEE_H))
        bv = float(rng.uniform(-VISEE_V, VISEE_V))
        roulis = float(np.clip(rng.normal(0.0, ROULIS_SIGMA), -ROULIS_MAX, ROULIS_MAX))
        return pos, bh, bv, roulis
    return None


def masque_tous_les_panneaux(stage) -> int:
    """Rend invisibles tous les panneaux QR de la scène ; renvoie leur nombre (228 dans l'entrepôt 9033)."""
    from pxr import UsdGeom

    n = 0
    for prim in stage.Traverse():
        if "QRTag" in prim.GetName():
            UsdGeom.Imageable(prim).MakeInvisible()
            n += 1
    return n


def mode_optique(sans_qr: bool) -> None:
    """Lance les modes optique et sans-qr : une caméra seule, sans drone, photographie le panneau
    depuis --poses poses tirées au hasard (QR cachés si `sans_qr`)."""
    layout = make_layout(args.seed)
    scene = scene_mod.build(layout, with_sitl=False, n_drones=0)
    tag = choisit_cible(scene)
    if sans_qr:
        print(f"panneaux masques : {masque_tous_les_panneaux(omni.usd.get_context().get_stage())}")

    pos0 = np.array(tag.position) + np.array(tag.normal) * 2.0
    cam = Camera(prim_path="/World/BancCam", translation=pos0,
                 orientation=quat_visee(pos0, tag.position),
                 resolution=(CAMERAS.side_width, CAMERAS.side_height))
    scene.world.reset()
    cam.initialize()
    cam.set_focal_length(CAMERAS.focal_length)
    cam.set_horizontal_aperture(CAMERAS.horizontal_aperture)
    cam.set_clipping_range(CAMERAS.near, CAMERAS.far)
    omni.timeline.get_timeline_interface().play()

    K = verifie_calibration(cam)
    retard = 0
    if not sans_qr:
        retard = mesure_retard(scene.world, cam, tag, K)
    print(f"cible {tag.tag_id} en {np.round(tag.position, 3)}, panneau {tag.size:.3f} m\n")

    rng = np.random.default_rng(args.graine_tirage)
    journal = Journal()
    for i in range(args.poses):
        tirage = tire_pose(rng, tag, layout)
        if tirage is None:
            continue
        pos, bh, bv, roulis = tirage
        cam.set_world_pose(position=pos,
                           orientation=quat_visee(pos, tag.position, bh, bv, roulis),
                           camera_axes="world")
        img = scene_mod.capture_camera(scene.world, cam, physique_entre_rendus=0)
        journal.note(cam, tag, _img.to_bgr(img))
        if (i + 1) % 200 == 0:
            print(f"  {i + 1}/{args.poses} poses")
    journal.clot(K, {"retard_mesure": retard, "sans_qr": sans_qr})


# ---------------------------------------------------------------- modes en vol

def cap_perpendiculaire(tag) -> float:
    """Renvoie le cap (rad) qui aligne le drone le long du rack, caméra gauche face au panneau."""
    n = np.array(tag.normal, float)
    return math.atan2(n[1], n[0]) + math.pi / 2.0


def cap_vers_panneau(cible_cam, tag) -> float:
    """Renvoie le cap (rad) qui pointe la caméra gauche vers le panneau ; sert aux poses en biais, où
    un cap perpendiculaire sortirait le panneau du champ (on mesurerait le cadrage, pas l'angle)."""
    v = np.array(tag.position, float) - np.array(cible_cam, float)
    return math.atan2(v[1], v[0]) - math.pi / 2.0


def consigne_drone(cible_cam, psi):
    """Renvoie la position du drone qui amène sa caméra gauche (10 cm sur le côté, 11 cm plus bas)
    en `cible_cam`."""
    lateral = np.array([-math.sin(psi), math.cos(psi), 0.0]) * CAM_LATERAL
    return np.array(cible_cam, float) - lateral + np.array([0.0, 0.0, CAM_BAS])


def degagement(p, layout) -> float:
    """Renvoie la distance (m) entre le point et la face du rack le plus proche qu'il longe (99 si aucun)."""
    d = 99.0
    for r in layout.racks:
        y0, y1 = r.y_bounds
        if y0 < p[1] < y1:
            d = min(d, abs(p[0] - r.x) - RACKS.depth / 2.0)
    return d


def demarre_vol(scene):
    """Connecte, arme et fait décoller le drone 0 à 1,6 m ; renvoie le pilote et deux fonctions qui
    lisent la position et le cap vrais du drone."""
    def get_pos():
        """Renvoie la position vraie du drone 0 (m)."""
        return scene.drones[0].state.position

    def get_yaw():
        """Renvoie le cap vrai du drone 0 (rad)."""
        return float(Rotation.from_quat(scene.drones[0].state.attitude).as_euler("ZYX")[0])

    pilot = Pilot(scene.world, 0)
    if not pilot.ready(FLY_ALT, lambda: float(get_pos()[2])):
        raise RuntimeError("le drone n'a pas decolle")
    return pilot, get_pos, get_yaw


def verifie_en_vol(scene, pilot, get_pos, get_yaw, tag, K) -> None:
    """Vérifie en vol, à 2 m du panneau, que la distance lue dans l'image égale la distance vraie
    à 10 cm près ; sinon arrête le banc."""
    cible_cam = np.array(tag.position, float) + np.array(tag.normal, float) * 2.0
    psi = cap_vers_panneau(cible_cam, tag)
    pilot.goto(consigne_drone(cible_cam, psi), psi, get_pos, tol=0.25,
               get_yaw=get_yaw, yaw_tol_rad=0.06, timeout_sim_s=90.0)
    pilot.pump(2.0)
    img = _img.to_bgr(scene.capture("left", 0))
    L = P.lire(img, K, tag.size, cible=tag.tag_id, decodeur="zxing")
    _, _, D, _ = pose_reelle(scene.cameras[0]["left"], tag)
    if not L.position_fiable or abs(L.distance - D) > 0.10:
        raise RuntimeError(f"aplomb en vol : vu {L.distance}, pose vraie {D:.2f}")
    print(f"[VERIF] aplomb en vol : pose vraie {D:.2f} m, image {L.distance:.2f} m\n")


def mode_vol() -> None:
    """Lance le mode vol : le drone tient les 12 poses de POSES_VOL et prend 15 images à chacune."""
    layout = make_layout(args.seed)
    scene = scene_mod.build(layout, with_sitl=True, n_drones=1)
    scene.world.reset()
    scene.finalize()
    omni.timeline.get_timeline_interface().play()
    cam = scene.cameras[0]["left"]
    K = verifie_calibration(cam)
    pilot, get_pos, get_yaw = demarre_vol(scene)
    tag = choisit_cible(scene)
    psi0 = cap_perpendiculaire(tag)
    transit(pilot, layout, tag.position[0] + 2.0, tag.position[2] + CAM_BAS,
            psi0, get_pos, get_yaw)
    verifie_en_vol(scene, pilot, get_pos, get_yaw, tag, K)

    n = np.array(tag.normal, float)
    t = np.cross(np.array([0.0, 0.0, 1.0]), n)
    t /= max(np.linalg.norm(t), 1e-9)
    journal = Journal()
    for k, (alpha, D) in enumerate(POSES_VOL):
        a = math.radians(alpha)
        cible_cam = np.array(tag.position, float) + D * (math.cos(a) * n + math.sin(a) * t)
        psi = cap_vers_panneau(cible_cam, tag)
        cible = consigne_drone(cible_cam, psi)
        marge = degagement(cible, layout)
        if marge < DEGAGEMENT_MIN:
            print(f"  pose {D:.2f} m/{alpha:+.0f} deg ecartee : {marge:.2f} m du rack")
            continue
        atteint = pilot.goto(cible, psi, get_pos, tol=0.25, get_yaw=get_yaw,
                             yaw_tol_rad=0.06, timeout_sim_s=60.0)
        pilot.pump(3.0)
        lus = 0
        for i in range(IMAGES_PAR_POSE):
            img = _img.to_bgr(scene.capture("left", 0))
            journal.note(cam, tag, img, run=f"pose{k:02d}", atteint=atteint)
            lus += int(P.lire(img, K, tag.size, cible=tag.tag_id,
                              decodeur="zxing").etat is P.Etat.LU)
            pilot.pump(0.3)
        _, _, D_reel, a_reel = pose_reelle(cam, tag)
        print(f"  pose {D:.2f} m/{alpha:+.0f} deg -> reel {D_reel:.2f} m/{a_reel:.0f} deg  "
              f"{lus}/{IMAGES_PAR_POSE} lues{'' if atteint else '  (non atteint)'}")
    pilot.velocity(0.0, 0.0, 0.0)
    pilot.pump(0.5)
    journal.clot(K)


def mode_traversee() -> None:
    """Lance le mode traversée : le drone longe le panneau à 1,2 m de distance, à 0,1 puis 0,3, 0,6
    et 1 m/s, en prenant une image toutes les 0,2 s."""
    layout = make_layout(args.seed)
    scene = scene_mod.build(layout, with_sitl=True, n_drones=1)
    scene.world.reset()
    scene.finalize()
    omni.timeline.get_timeline_interface().play()
    cam = scene.cameras[0]["left"]
    K = verifie_calibration(cam)
    pilot, get_pos, get_yaw = demarre_vol(scene)
    tag = choisit_cible(scene)
    psi = cap_perpendiculaire(tag)
    transit(pilot, layout, tag.position[0] + 2.0, tag.position[2] + CAM_BAS,
            psi, get_pos, get_yaw)
    verifie_en_vol(scene, pilot, get_pos, get_yaw, tag, K)

    # Capture à la cadence de mission, sans attendre le rendu : chaque image est en retard d'un
    # nombre constant de rendus, la pose est notée à chaque rendu, et l'analyse retrouve ce
    # décalage puis associe chaque image à sa vraie pose.
    journal = Journal()
    for v in VITESSES:
        depart = np.array(tag.position, float) + np.array(tag.normal, float) * 1.20
        depart[1] = tag.position[1] - DEMI_FENETRE
        pilot.goto(consigne_drone(depart, psi), psi, get_pos, tol=0.25,
                   get_yaw=get_yaw, yaw_tol_rad=0.06, timeout_sim_s=120.0)
        pilot.pump(2.0)

        R = pilot._ned_in_world
        v_ned = R.T @ np.array([0.0, v, 0.0])
        cw = np.array([math.cos(psi), math.sin(psi), 0.0])
        psi_ned = float(np.arctan2(np.dot(cw, R[:, 1]), np.dot(cw, R[:, 0])))

        fin_y = tag.position[1] + DEMI_FENETRE
        garde = int(2.0 * DEMI_FENETRE / v / 0.2 * 2) + 100
        i = 0
        while float(get_pos()[1]) < fin_y and i < garde:
            pilot.velocity(float(v_ned[0]), float(v_ned[1]), float(v_ned[2]), yaw_rad=psi_ned)
            for _ in range(PHYS_PAR_IMAGE - 1):
                scene.world.step(render=False)
            scene.world.step(render=True)
            img = cam.get_rgb()
            pleine = img is not None and getattr(img, "ndim", 0) == 3 and img.size > 0
            journal.note(cam, tag, _img.to_bgr(img) if pleine else None,
                         run=f"v{v:.1f}", indice=i, vitesse=v)
            i += 1
        pilot.velocity(0.0, 0.0, 0.0)
        pilot.pump(1.0)
        print(f"  vitesse {v:.1f} m/s : {i} images")
    journal.clot(K)


# Valeur de --mode -> fonction lancée ; le simulateur est toujours fermé à la fin, même après une erreur.
MODES = {"optique": lambda: mode_optique(False), "sans-qr": lambda: mode_optique(True),
         "vol": mode_vol, "traversee": mode_traversee}

try:
    MODES[args.mode]()
    print(f"BANC {args.mode.upper()} FINI")
except Exception:
    traceback.print_exc()
finally:
    simulation_app.close()
