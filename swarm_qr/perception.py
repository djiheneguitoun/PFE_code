"""Perception : voir, lire et situer un QR code dans une image."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np

MODULES_CODE = 21
MODULES_MARGE = 2
MODULES_PANNEAU = MODULES_CODE + 2 * MODULES_MARGE


def taille_code(taille_panneau: float) -> float:
    """Côté du code seul."""
    return taille_panneau * MODULES_CODE / MODULES_PANNEAU


class Etat(str, Enum):
    LU = "lu"
    REPERE = "repere"
    RIEN = "rien"


@dataclass
class Lecture:
    """PIÈGE : `position` et `distance` n'ont de sens qu'à l'état LU."""

    etat: Etat
    texte: str | None
    coins: np.ndarray | None
    position: np.ndarray | None
    distance: float | None
    incidence_deg: float | None

    @property
    def position_fiable(self) -> bool:
        return self.etat is Etat.LU and self.position is not None


def _ordonne(coins: np.ndarray) -> np.ndarray:
    """Coins dans un ordre unique : haut-gauche, haut-droit, bas-droit, bas-gauche."""
    p = np.asarray(coins, dtype=np.float64).reshape(-1, 2)
    centre = p.mean(axis=0)
    angles = np.arctan2(p[:, 1] - centre[1], p[:, 0] - centre[0])
    p = p[np.argsort(angles)]
    depart = int(np.argmin(p[:, 0] + p[:, 1]))
    return np.roll(p, -depart, axis=0)


def _dec_opencv(gray, aruco: bool):
    det = cv2.QRCodeDetectorAruco() if aruco else cv2.QRCodeDetector()
    try:
        ok, textes, points, _ = det.detectAndDecodeMulti(gray)
    except cv2.error:
        return []
    if not ok or points is None:
        return []
    return [(t, np.asarray(p).reshape(-1, 2)) for t, p in zip(textes, points) if t]


def _dec_zxing(gray):
    """QR seulement : l'entrepôt porte des codes-barres imprimés sur son décor."""
    import zxingcpp

    out = []
    for r in zxingcpp.read_barcodes(gray, formats=zxingcpp.BarcodeFormat.QRCode):
        p = r.position
        out.append((r.text, np.array([
            [p.top_left.x, p.top_left.y], [p.top_right.x, p.top_right.y],
            [p.bottom_right.x, p.bottom_right.y], [p.bottom_left.x, p.bottom_left.y]], float)))
    return out


def _dec_zbar(gray):
    from pyzbar import pyzbar

    out = []
    for r in pyzbar.decode(gray, symbols=[pyzbar.ZBarSymbol.QRCODE]):
        if len(r.polygon) < 4:
            continue
        out.append((r.data.decode(errors="replace"),
                    np.array([[pt.x, pt.y] for pt in r.polygon[:4]], float)))
    return out


def _dec_boof(gray):
    import pyboof as pb

    det = pb.FactoryFiducial(np.uint8).qrcode()
    det.detect(pb.ndarray_to_boof(np.ascontiguousarray(gray)))
    return [(d.message, np.array([[v.x, v.y] for v in d.bounds.vertexes], float))
            for d in det.detections]


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
    """Chaque décodeur voit exactement la même image en niveaux de gris."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    out = {}
    for nom in decodeurs:
        try:
            out[nom] = DECODEURS[nom](gray)
        except Exception:
            out[nom] = []
    return out


def repere_motifs(bgr, cote_min: int = 12) -> list[np.ndarray]:
    """Quadrilatères sombres/clairs qui ressemblent à un QR, sans chercher à le lire."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    seuil = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY_INV, 31, 5)
    contours, _ = cv2.findContours(seuil, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        aire = cv2.contourArea(c)
        if aire < cote_min * cote_min:
            continue
        approx = cv2.approxPolyDP(c, 0.04 * cv2.arcLength(c, True), True)
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue
        q = approx.reshape(-1, 2).astype(float)
        cotes = [np.linalg.norm(q[(i + 1) % 4] - q[i]) for i in range(4)]
        if min(cotes) < cote_min or max(cotes) / max(min(cotes), 1e-6) > 2.5:
            continue
        masque = np.zeros(gray.shape, np.uint8)
        cv2.fillPoly(masque, [approx], 255)
        part_sombre = float((gray[masque > 0] < 128).mean())
        if 0.20 < part_sombre < 0.70:
            out.append(_ordonne(q))
    return out


def pose_3d(coins: np.ndarray, cote_m: float, K: np.ndarray,
            normale_attendue: np.ndarray | None = None):
    """Position du centre du code dans le repère de la caméra."""
    demi = cote_m / 2.0
    modele = np.array([[-demi, demi, 0.0], [demi, demi, 0.0],
                       [demi, -demi, 0.0], [-demi, -demi, 0.0]])
    img = _ordonne(coins).astype(np.float64)
    ok, rvecs, tvecs, err = cv2.solvePnPGeneric(
        modele, img, K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not len(tvecs):
        return None, None

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
    """Angle entre l'axe de visée du panneau et la direction caméra → panneau."""
    v = -centre_cam / max(np.linalg.norm(centre_cam), 1e-9)
    c = float(np.clip(np.dot(v, normale_cam / max(np.linalg.norm(normale_cam), 1e-9)), -1.0, 1.0))
    return float(np.degrees(np.arccos(abs(c))))


def lire(bgr, K: np.ndarray, taille_panneau: float, cible: str | None = None,
         decodeur: str = "zxing", normale_attendue=None) -> Lecture:
    """La lecture complète d'une image, telle que le système l'utilisera."""
    cote = taille_code(taille_panneau)
    trouves = decode_tous(bgr, (decodeur,))[decodeur]
    for texte, coins in trouves:
        if cible is not None and texte != cible:
            continue
        t, R = pose_3d(coins, cote, K, normale_attendue)
        d = float(np.linalg.norm(t)) if t is not None else None
        a = incidence_deg(t, R[:, 2]) if t is not None else None
        return Lecture(Etat.LU, texte, _ordonne(coins), t, d, a)

    motifs = repere_motifs(bgr)
    if motifs:
        t, R = pose_3d(motifs[0], cote, K, normale_attendue)
        d = float(np.linalg.norm(t)) if t is not None else None
        return Lecture(Etat.REPERE, None, motifs[0], t, d, None)

    return Lecture(Etat.RIEN, None, None, None, None, None)
