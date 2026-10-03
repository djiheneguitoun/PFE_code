"""Détecteur appris (étape 7) : un petit réseau de neurones YOLO qui repère les QR et les cartons, sans les lire.

Il repère bien plus loin qu'on ne lit (98 % des QR repérés à 6–8 m, lecture fiable jusqu'à 4 m seulement) : un QR
repéré devient une piste sur la carte, un carton repéré marque le canal sémantique. Poids et réglages : assets/detecteur/.
Chargé par mission.py (option --detecteur auto), puis utilisé par observation.py sur les images des drones."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ASSETS = Path(__file__).resolve().parent / "assets" / "detecteur"   # installé par experiments/10_detecteur/banc.py
POIDS = ASSETS / "detecteur.pt"         # poids du réseau retenu : YOLO11 nano, images de 1024 px
REGLAGES = ASSETS / "detecteur.json"    # taille d'image (1024 px), seuil de confiance (0,5), classes (qr, carton)


@dataclass(frozen=True)
class Detection:
    """Un objet repéré dans une image : sa classe (« qr » ou « carton »), sa confiance et son cadre."""
    classe: str
    confiance: float        # entre 0 et 1
    bbox: np.ndarray        # cadre [x1, y1, x2, y2] en pixels

    @property
    def centre(self) -> np.ndarray:
        """Renvoie le centre du cadre (x, y), en pixels."""
        return np.array([(self.bbox[0] + self.bbox[2]) / 2.0, (self.bbox[1] + self.bbox[3]) / 2.0])

    @property
    def cote_px(self) -> float:
        """Renvoie le plus petit côté du cadre, en pixels : sert à estimer la distance d'un QR de taille connue."""
        return float(min(self.bbox[2] - self.bbox[0], self.bbox[3] - self.bbox[1]))


class Detecteur:
    """Le réseau YOLO chargé une fois sur la carte graphique, prêt à repérer QR et cartons."""

    def __init__(self, poids: str | Path = POIDS, imgsz: int | None = None,
                 conf: float | None = None, device: int | str = 0):
        """Charge les poids et les réglages (1024 px et seuil 0,5 par défaut) ; `device` 0 = première carte graphique."""
        os.environ.setdefault("YOLO_AUTOINSTALL", "false")    # interdit à Ultralytics d'installer des paquets seul
        from ultralytics import YOLO

        reglages = json.loads(REGLAGES.read_text()) if REGLAGES.exists() else {}
        self.imgsz = int(imgsz or reglages.get("imgsz", 1024))
        self.conf = float(conf if conf is not None else reglages.get("conf", 0.5))
        self.device = device
        self.modele = YOLO(str(poids))
        self.noms = {int(k): str(v) for k, v in self.modele.names.items()}

    def detecte(self, bgr: np.ndarray) -> list[Detection]:
        """Renvoie les QR et cartons repérés au-dessus du seuil dans une image couleur OpenCV (BGR).
        Calcul en demi-précision (nombres sur 16 bits) : environ 12 ms par image, mesuré à l'étape 7."""
        r = self.modele.predict(bgr, imgsz=self.imgsz, conf=self.conf, half=True,
                                device=self.device, verbose=False)[0]
        b = r.boxes
        return [Detection(self.noms[int(c)], float(s), np.asarray(xyxy, dtype=float))
                for c, s, xyxy in zip(b.cls.tolist(), b.conf.tolist(), b.xyxy.tolist())]
