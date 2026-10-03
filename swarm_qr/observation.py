"""Verse dans la carte partagée ce que voient les capteurs d'un drone, à chaque cycle d'observation.

Lidar → cases libres ou occupées ; deux caméras latérales → QR décodés (zxing) et placés, couverture ;
détecteur appris de l'étape 7 (ou repérage classique) → pistes et cartons. La distance d'un code vient
du lidar : l'image ne donne que son identité et son orientation. Classe `Observateur`, utilisée par
mission.py et experiments/09_carte/banc.py.
"""

from __future__ import annotations

import re
import time
from collections import deque

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from . import mapping
from . import perception as P
from .env.config import QR_CFG
from .experiments import _img

INCIDENCE_MAX = 60.0         # degrés : un code lu plus de biais que cela n'est pas placé sur la carte
# Format des codes de l'inventaire (« BOX_ » + 3 chiffres) ; les autres QR du décor sont comptés à part.
FORMAT_CODE = re.compile("^" + re.escape(QR_CFG.payload_fmt).replace(re.escape("{:03d}"), r"\d{3}") + "$")
# m : côté supposé d'un panneau (les vrais font de 12 à 40 cm). Il sert à l'orientation et au contrôle
# de vraisemblance des repérages, jamais à la distance, qui vient du lidar.
PANNEAU_M = 0.40


def _isaac(v_opencv):
    """Convertit un vecteur du repère caméra d'OpenCV (z devant, x à droite, y en bas) vers celui
    d'Isaac (x devant, y à gauche, z en haut)."""
    return np.array([v_opencv[2], -v_opencv[0], -v_opencv[1]])


def rayon_du_pixel(pixel, K, cam_R) -> np.ndarray:
    """Renvoie la direction unitaire, en repère monde, du rayon de caméra qui passe par `pixel`."""
    rayon_cam = np.array([(pixel[0] - K[0, 2]) / K[0, 0], (pixel[1] - K[1, 2]) / K[1, 1], 1.0])
    return cam_R @ _isaac(rayon_cam / np.linalg.norm(rayon_cam))


def distance_par_le_lidar(origine, direction, nuage):
    """Renvoie la distance (m) le long d'un rayon de caméra, lue sur le point lidar le plus proche du
    rayon ; None si aucun point n'en est assez près."""
    if nuage is None or not len(nuage):
        return None
    v = nuage - origine
    t = v @ direction
    perp = np.linalg.norm(v - t[:, None] * direction[None, :], axis=1)
    perp[t < 0.3] = np.inf                       # points à moins de 30 cm devant (ou derrière) ignorés
    k = int(np.argmin(perp))
    # écart toléré au rayon : 6 cm + 4 cm par mètre (les rayons lidar s'écartent), 25 cm au plus
    if perp[k] > min(0.25, 0.06 + 0.04 * t[k]):
        return None
    return float(t[k])


def position_monde(coins, K, cam_pos, cam_R, nuage):
    """Renvoie (position, normale) en monde d'un code lu, ou None s'il est vu à plus de 60° de biais
    ou si la distance lidar sort de 0,3–4 m."""
    # l'image donne l'orientation ; la distance viendra du lidar (la vraie taille du panneau est inconnue)
    t3, R3 = P.pose_3d(coins, P.taille_code(PANNEAU_M), K)
    if t3 is None:
        return None
    inc = P.incidence_deg(t3, R3[:, 2])
    if min(inc, 180.0 - inc) > INCIDENCE_MAX:
        return None
    rayon = rayon_du_pixel(coins.mean(axis=0), K, cam_R)
    d = distance_par_le_lidar(cam_pos, rayon, nuage)
    if d is None or not (0.3 < d <= mapping.LIRE_MAX):
        return None
    monde = cam_pos + rayon * d
    n = cam_R @ _isaac(R3[:, 2])
    if np.dot(n, cam_pos - monde) < 0:           # normale tournée vers la caméra
        n = -n
    return monde, n


def point_vise(pixel, K, cam_pos, cam_R, nuage, carte, portee: float = mapping.PORTEE_CARTE):
    """Renvoie (point visé en monde, rayon) pour un pixel : distance lue au lidar, sinon premier
    obstacle connu de la carte ; None si rien entre 0,3 m et `portee`."""
    rayon = rayon_du_pixel(pixel, K, cam_R)
    d = distance_par_le_lidar(cam_pos, rayon, nuage)
    if d is None or d > mapping.LIRE_MAX:
        d = carte.premier_obstacle(cam_pos, rayon, portee)
    if d is None or not (0.3 < d <= portee):
        return None
    return cam_pos + rayon * d, rayon


def poses_cameras(scene, drone: int, origine, dirs, portees) -> dict:
    """Renvoie les poses (position, rotation) des deux caméras latérales et le nuage lidar en monde,
    pris au même instant."""
    out = {}
    for nom in ("left", "right"):
        p, q = scene.cameras[drone][nom].get_world_pose()
        # Isaac donne le quaternion en (w, x, y, z), scipy l'attend en (x, y, z, w)
        out[nom] = (np.asarray(p, float), Rotation.from_quat(np.asarray(q)[[1, 2, 3, 0]]).as_matrix())
    ok = np.isfinite(portees) & (portees > 0)    # rayons sans écho exclus
    out["nuage"] = origine + dirs[ok] * portees[ok, None]
    return out


class Observateur:
    """Les capteurs d'un drone versés dans la carte partagée, cycle après cycle."""

    def __init__(self, scene, carte: mapping.Carte, K, drone: int = 0, detecteur=None):
        """Prépare l'observateur du drone `drone` : `K` = matrice de calibration de la caméra (focale,
        centre de l'image), `detecteur` = œil appris (None : repérage classique)."""
        self.scene, self.carte, self.K, self.drone = scene, carte, np.asarray(K, float), drone
        self.detecteur = detecteur
        self.poses = deque(maxlen=2)                     # poses des deux derniers cycles (voir observe)
        self.trajectoire: list[list[float]] = []         # [t, x, y, z]
        self.inclinaisons: list[list[float]] = []        # [t, inclinaison en degrés]
        self.compte = {"images": 0, "lectures": 0, "reperages": 0, "lectures_placees": 0,
                       "cartons_reperes": 0}
        self.couts = {"lidar": [], "couverture": [], "decodage": [], "detecteur": []}   # ms par étape
        self.cycles = 0

    def observe(self, t: float, oeil: bool = True) -> None:
        """Fait un cycle d'observation à l'instant `t` : lidar, position annoncée, puis les deux caméras.
        `oeil=False` : l'œil appris saute ce drone ce cycle (les drones se le partagent à tour de rôle)."""
        scene, carte, i = self.scene, self.carte, self.drone
        pos = scene.position(i)
        self.trajectoire.append([round(t, 2), *[round(float(x), 3) for x in pos]])
        # inclinaison = angle (degrés) entre l'axe « haut » du drone et la verticale
        haut = Rotation.from_quat(scene.drones[i].state.attitude).apply([0.0, 0.0, 1.0])
        self.inclinaisons.append([round(t, 2), round(float(np.degrees(np.arccos(np.clip(haut[2], -1.0, 1.0)))), 1)])
        dirs, portees = scene.lidar(i)
        t0 = time.perf_counter()
        carte.integre_lidar(pos, dirs, portees, t=t)
        self.couts["lidar"].append((time.perf_counter() - t0) * 1000)
        carte.annonce(i, pos, t=t)
        self.poses.append(poses_cameras(scene, i, pos, dirs, portees))
        if len(self.poses) < 2:
            return
        # l'image lue maintenant a un cycle de retard : on l'associe aux poses et au nuage du cycle précédent
        anciennes = self.poses[0]
        nuage = anciennes["nuage"]
        self.cycles += 1
        cam_apprise = ("left", "right")[self.cycles % 2]     # l'œil appris : une caméra par cycle, en alternance
        for nom in ("left", "right"):
            img = scene.cameras[i][nom].get_rgb()
            if img is None or getattr(img, "ndim", 0) != 3 or not img.size:
                continue
            cam_pos, cam_R = anciennes[nom]
            bgr = _img.to_bgr(img)
            self._lit(bgr, cam_pos, cam_R, nuage, t)
            if self.detecteur is None:
                self._repere_classique(bgr, cam_pos, cam_R, nuage, t)
            elif oeil and nom == cam_apprise:
                self._repere_appris(bgr, cam_pos, cam_R, nuage, t)
            t0 = time.perf_counter()
            carte.integre_couverture(cam_pos, cam_R @ np.array([1.0, 0.0, 0.0]), t=t)
            self.couts["couverture"].append((time.perf_counter() - t0) * 1000)
            self.compte["images"] += 1

    def _lit(self, bgr, cam_pos, cam_R, nuage, t) -> None:
        """Décode les QR d'une image (zxing) ; chaque code au bon format est compté, puis placé sur la
        carte si sa position est fiable. Les autres codes sont comptés dans « codes_etrangers »."""
        gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        t0 = time.perf_counter()
        for code, coins in P.DECODEURS["zxing"](gris):
            if not FORMAT_CODE.match(code):
                self.compte["codes_etrangers"] = self.compte.get("codes_etrangers", 0) + 1
                continue
            self.compte["lectures"] += 1
            pn = position_monde(coins, self.K, cam_pos, cam_R, nuage)
            if pn is not None:
                self.carte.integre_lecture(code, pn[0], pn[1], t=t)
                self.compte["lectures_placees"] += 1
        self.couts["decodage"].append((time.perf_counter() - t0) * 1000)

    def _repere_classique(self, bgr, cam_pos, cam_R, nuage, t) -> None:
        """Repère les motifs qui ressemblent à un QR (sans réseau appris) et en fait des pistes sur la carte."""
        for quad in P.repere_motifs(bgr):
            self.compte["reperages"] += 1
            pn = position_monde(quad, self.K, cam_pos, cam_R, nuage)
            if pn is not None:
                self.carte.integre_reperage(pn[0], pn[1], t=t)

    def _repere_appris(self, bgr, cam_pos, cam_R, nuage, t) -> None:
        """Passe l'image à l'œil appris : un QR repéré (et plausible) devient une piste tournée vers la
        caméra ; un carton repéré est marqué sur la carte."""
        t0 = time.perf_counter()
        for d in self.detecteur.detecte(bgr):
            pv = point_vise(d.centre, self.K, cam_pos, cam_R, nuage, self.carte)
            if pv is None:
                continue
            pos, rayon = pv
            if d.classe == "qr":
                # distance qu'aurait un panneau de 40 cm qui paraîtrait de cette taille dans l'image
                d_cadre = self.K[0, 0] * P.taille_code(PANNEAU_M) / max(d.cote_px, 1.0)
                dist = float(np.linalg.norm(pos - cam_pos))
                # point trop loin ou trop près pour ce cadre (ex. rayon passé par un trou du rack) : rejeté
                if not (0.45 * d_cadre <= dist <= 1.8 * d_cadre):
                    self.compte["reperages_incoherents"] = self.compte.get("reperages_incoherents", 0) + 1
                    continue
                self.compte["reperages"] += 1
                # la piste regarde vers la caméra, à l'horizontale
                vers_camera = -np.array([rayon[0], rayon[1], 0.0])
                norme = np.linalg.norm(vers_camera)
                self.carte.integre_reperage(pos, vers_camera / norme if norme > 1e-6 else None, t=t)
            else:
                self.compte["cartons_reperes"] += self.carte.marque(pos)
        self.couts["detecteur"].append((time.perf_counter() - t0) * 1000)
