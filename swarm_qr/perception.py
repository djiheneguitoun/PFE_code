"""Perception : lire et situer un QR code dans une image (aucun lien avec le simulateur).

Décodeurs interchangeables (zxing, retenu à l'étape 2 ; zbar ; OpenCV ; PyBoof), pose 3D du code par
solvePnP (position et orientation déduites de ses 4 coins), et repérage de motifs qui ressemblent à
un QR sans le lire. Utilisé par observation.py et par les bancs de experiments/ (03, 07, 08, 10).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np

MODULES_CODE = 21            # modules (petits carrés) par côté d'un QR version 1, celle des étiquettes
MODULES_MARGE = 2            # modules de marge blanche de chaque côté (voir env/qr_tags.py)
MODULES_PANNEAU = MODULES_CODE + 2 * MODULES_MARGE      # 25 : côté du panneau entier, marges comprises


def taille_code(taille_panneau: float) -> float:
    """Renvoie le côté du code seul, sans la marge blanche (21/25 du panneau), dans la même unité."""
    return taille_panneau * MODULES_CODE / MODULES_PANNEAU


class Etat(str, Enum):
    """Résultat d'une image : QR LU (décodé), REPERE (motif vu mais pas lu) ou RIEN."""
    LU = "lu"
    REPERE = "repere"
    RIEN = "rien"


@dataclass
class Lecture:
    """Résultat de la lecture d'une image ; PIÈGE : `position` et `distance` ne valent qu'à l'état LU."""

    etat: Etat
    texte: str | None              # contenu du QR, None s'il n'est pas lu
    coins: np.ndarray | None       # 4 coins en pixels, dans l'ordre de _ordonne
    position: np.ndarray | None    # m : centre du code dans le repère OpenCV de la caméra
    distance: float | None         # m : de la caméra au centre du code
    incidence_deg: float | None    # degrés : 0 = vu de face

    @property
    def position_fiable(self) -> bool:
        """Renvoie vrai seulement si le QR est lu et sa position calculée."""
        return self.etat is Etat.LU and self.position is not None


def _ordonne(coins: np.ndarray) -> np.ndarray:
    """Renvoie les 4 coins dans un ordre unique : haut-gauche, haut-droit, bas-droit, bas-gauche."""
    p = np.asarray(coins, dtype=np.float64).reshape(-1, 2)
    centre = p.mean(axis=0)
    angles = np.arctan2(p[:, 1] - centre[1], p[:, 0] - centre[0])
    p = p[np.argsort(angles)]                    # tri par angle autour du centre : sens horaire à l'écran
    depart = int(np.argmin(p[:, 0] + p[:, 1]))   # départ : le coin haut-gauche (plus petit x + y)
    return np.roll(p, -depart, axis=0)


def _dec_opencv(gray, aruco: bool):
    """Décode les QR avec OpenCV (variante « Aruco » si `aruco`) ; renvoie [(texte, coins)], vide si échec."""
    det = cv2.QRCodeDetectorAruco() if aruco else cv2.QRCodeDetector()
    try:
        ok, textes, points, _ = det.detectAndDecodeMulti(gray)
    except cv2.error:
        return []
    if not ok or points is None:
        return []
    return [(t, np.asarray(p).reshape(-1, 2)) for t, p in zip(textes, points) if t]


def _dec_zxing(gray):
    """Décode les QR avec zxing-cpp, le décodeur retenu ; renvoie [(texte, 4 coins)]. QR seulement :
    l'entrepôt porte des codes-barres imprimés sur son décor."""
    import zxingcpp

    out = []
    for r in zxingcpp.read_barcodes(gray, formats=zxingcpp.BarcodeFormat.QRCode):
        p = r.position
        out.append((r.text, np.array([
            [p.top_left.x, p.top_left.y], [p.top_right.x, p.top_right.y],
            [p.bottom_right.x, p.bottom_right.y], [p.bottom_left.x, p.bottom_left.y]], float)))
    return out


def _dec_zbar(gray):
    """Décode les QR avec zbar (pyzbar), l'alternative à zxing ; ignore les résultats de moins de 4 coins."""
    from pyzbar import pyzbar

    out = []
    for r in pyzbar.decode(gray, symbols=[pyzbar.ZBarSymbol.QRCODE]):
        if len(r.polygon) < 4:
            continue
        out.append((r.data.decode(errors="replace"),
                    np.array([[pt.x, pt.y] for pt in r.polygon[:4]], float)))
    return out


def _dec_boof(gray):
    """Décode les QR avec PyBoof (bibliothèque BoofCV) ; renvoie [(texte, coins)]."""
    import pyboof as pb

    det = pb.FactoryFiducial(np.uint8).qrcode()
    det.detect(pb.ndarray_to_boof(np.ascontiguousarray(gray)))
    return [(d.message, np.array([[v.x, v.y] for v in d.bounds.vertexes], float))
            for d in det.detections]


# Nom → fonction (image en gris → [(texte, coins)]). « _x3 » : image agrandie 3 fois avant décodage.
DECODEURS = {
    "opencv": lambda g: _dec_opencv(g, aruco=False),
    "opencv_aruco": lambda g: _dec_opencv(g, aruco=True),
    "opencv_aruco_x3": lambda g: _agrandi(g, lambda x: _dec_opencv(x, aruco=True), 3),
    "zxing": _dec_zxing,
    "zbar": _dec_zbar,
    "pyboof": _dec_boof,
}


def _agrandi(gray, fn, facteur: int):
    """Décode sur une image agrandie et ramène les coins à l'échelle d'origine."""
    grand = cv2.resize(gray, None, fx=facteur, fy=facteur, interpolation=cv2.INTER_CUBIC)
    return [(t, c / facteur) for t, c in fn(grand)]


def decode_tous(bgr, decodeurs=("zxing",)) -> dict[str, list[tuple[str, np.ndarray]]]:
    """Lance chaque décodeur demandé sur la même image en niveaux de gris ; renvoie {nom: [(texte,
    coins)]}, avec une liste vide pour un décodeur qui échoue."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    out = {}
    for nom in decodeurs:
        try:
            out[nom] = DECODEURS[nom](gray)
        except Exception:
            out[nom] = []
    return out


def repere_motifs(bgr, cote_min: int = 12) -> list[np.ndarray]:
    """Renvoie les quadrilatères qui ressemblent à un QR (côtés ≥ `cote_min` px, 20 à 70 % de sombre),
    sans les lire ; dans un entrepôt sans QR, il en voit à tort une fois sur quatre (27 %, étape 2)."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    # seuil local (fenêtre de 31 px), peu sensible à l'éclairage : le sombre devient blanc
    seuil = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY_INV, 31, 5)
    contours, _ = cv2.findContours(seuil, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        aire = cv2.contourArea(c)
        if aire < cote_min * cote_min:
            continue
        # contour simplifié : on ne garde que les quadrilatères convexes
        approx = cv2.approxPolyDP(c, 0.04 * cv2.arcLength(c, True), True)
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue
        q = approx.reshape(-1, 2).astype(float)
        cotes = [np.linalg.norm(q[(i + 1) % 4] - q[i]) for i in range(4)]
        if min(cotes) < cote_min or max(cotes) / max(min(cotes), 1e-6) > 2.5:     # trop petit ou trop allongé
            continue
        masque = np.zeros(gray.shape, np.uint8)
        cv2.fillPoly(masque, [approx], 255)
        part_sombre = float((gray[masque > 0] < 128).mean())
        if 0.20 < part_sombre < 0.70:            # un QR mêle noir et blanc : ni presque vide, ni presque plein
            out.append(_ordonne(q))
    return out


def pose_3d(coins: np.ndarray, cote_m: float, K: np.ndarray,
            normale_attendue: np.ndarray | None = None):
    """Renvoie (t, R) : position (m) et rotation du code dans le repère OpenCV de la caméra, par
    solvePnP ; R[:, 2] est la normale du code. (None, None) si le calcul échoue."""
    demi = cote_m / 2.0
    # les 4 coins du code dans son propre plan (z = 0), dans l'ordre de _ordonne
    modele = np.array([[-demi, demi, 0.0], [demi, demi, 0.0],
                       [demi, -demi, 0.0], [-demi, -demi, 0.0]])
    img = _ordonne(coins).astype(np.float64)
    # méthode faite pour un carré plan ; elle peut rendre deux solutions (un carré vu de biais est ambigu)
    ok, rvecs, tvecs, err = cv2.solvePnPGeneric(
        modele, img, K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not len(tvecs):
        return None, None

    # on garde la solution dont la normale colle à `normale_attendue`, sinon la plus petite erreur
    meilleur = 0
    if normale_attendue is not None and len(rvecs) > 1:
        scores = []
        for r in rvecs:
            R, _ = cv2.Rodrigues(r)
            scores.append(float(np.dot(R[:, 2], normale_attendue)))
        meilleur = int(np.argmax(scores))
    elif len(err) > 1:
        meilleur = int(np.argmin(err))

    t = tvecs[meilleur].reshape(3)
    R, _ = cv2.Rodrigues(rvecs[meilleur])
    return t, R


def incidence_deg(centre_cam: np.ndarray, normale_cam: np.ndarray) -> float:
    """Renvoie l'angle (degrés, de 0 à 90) entre la normale du panneau et la direction panneau →
    caméra : 0 = vu de face."""
    v = -centre_cam / max(np.linalg.norm(centre_cam), 1e-9)
    c = float(np.clip(np.dot(v, normale_cam / max(np.linalg.norm(normale_cam), 1e-9)), -1.0, 1.0))
    return float(np.degrees(np.arccos(abs(c))))


def lire(bgr, K: np.ndarray, taille_panneau: float, cible: str | None = None,
         decodeur: str = "zxing", normale_attendue=None) -> Lecture:
    """Lit une image (sert aux bancs des étapes 2 et 3) : renvoie une Lecture LU (premier code lu, égal
    à `cible` si elle est donnée), sinon REPERE (motif vu), sinon RIEN."""
    cote = taille_code(taille_panneau)
    trouves = decode_tous(bgr, (decodeur,))[decodeur]
    for texte, coins in trouves:
        if cible is not None and texte != cible:
            continue
        t, R = pose_3d(coins, cote, K, normale_attendue)
        d = float(np.linalg.norm(t)) if t is not None else None
        a = incidence_deg(t, R[:, 2]) if t is not None else None
        return Lecture(Etat.LU, texte, _ordonne(coins), t, d, a)

    # rien de lu : un motif repéré ? Sa position suppose la taille du panneau, elle n'est pas fiable
    motifs = repere_motifs(bgr)
    if motifs:
        t, R = pose_3d(motifs[0], cote, K, normale_attendue)
        d = float(np.linalg.norm(t)) if t is not None else None
        return Lecture(Etat.REPERE, None, motifs[0], t, d, None)

    return Lecture(Etat.RIEN, None, None, None, None, None)
