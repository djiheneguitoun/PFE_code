"""Test du détecteur appris (`detecteur.py`, étape 7), sans réseau, sans GPU, sans simulateur.

Vérifie la géométrie d'une détection (centre et côté du cadre). ultralytics n'est pas importé :
`detecteur.py` ne le charge qu'à la création d'un `Detecteur`.
Lancement, depuis la racine du projet : python -m pytest swarm_qr/tests/test_detecteur.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from swarm_qr.detecteur import Detection  # noqa: E402


def test_une_detection_donne_son_centre_et_son_cote():
    """Vérifie qu'un cadre de (100, 200) à (160, 250) px donne le centre (130, 225) et un côté de
    50 px, le plus petit des deux côtés (60 × 50)."""
    d = Detection("qr", 0.9, np.array([100.0, 200.0, 160.0, 250.0]))
    assert np.allclose(d.centre, [130.0, 225.0])
    assert d.cote_px == 50.0
