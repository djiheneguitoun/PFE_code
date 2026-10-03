"""Tests du cerveau (`planning.py`, étape 5) : choisit-il la bonne cible, avant tout vol ?

Des cartes jouets écrites à la main (un couloir, une salle avec un mur), dont la bonne réponse
est connue : lire avant d'explorer, éviter les réservations, trois chances par piste, etc.
Lancement, depuis la racine du projet : python -m pytest swarm_qr/tests/test_planning.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from swarm_qr import mapping as M  # noqa: E402
from swarm_qr import planning as PL  # noqa: E402
from swarm_qr.mapping import Carte  # noqa: E402


def _bloc(c, lo, hi, valeur):
    """Écrit `valeur` dans toutes les cases de la boîte allant du coin `lo` au coin `hi` (m) :
    −5 = libre à coup sûr, +5 = occupé à coup sûr."""
    i0 = c.indice([lo])[0]
    i1 = c.indice([hi])[0]
    c.occupation[i0[0]:i1[0] + 1, i0[1]:i1[1] + 1, i0[2]:i1[2] + 1] = valeur


def couloir(murs: bool = True) -> Carte:
    """Renvoie une carte jouet : un couloir libre de 2 m de large le long de x (de −6 à 6 m),
    bordé de murs si `murs`, inconnu au-delà de ses deux bouts."""
    c = Carte()
    _bloc(c, (-6.0, -1.0, 0.5), (6.0, 1.0, 3.0), -5.0)
    if murs:
        _bloc(c, (-6.0, 1.0, 0.5), (6.0, 1.5, 3.0), 5.0)
        _bloc(c, (-6.0, -1.5, 0.5), (6.0, -1.0, 3.0), 5.0)
    return c


def couvre_tout(c: Carte) -> None:
    """Marque toutes les cases libres comme déjà regardées, depuis les quatre côtés."""
    c.couverture[c.occupation < M.SEUIL_LIBRE] = 255


def salle() -> Carte:
    """Renvoie une carte jouet : une salle libre de 12 × 8 m (x de −6 à 6, y de −4 à 4) avec un
    mur de x = 3 à 3,5 m sur toute sa largeur."""
    c = Carte()
    _bloc(c, (-6.0, -4.0, 0.5), (6.0, 4.0, 3.0), -5.0)
    _bloc(c, (3.0, -4.0, 0.5), (3.5, 4.0, 3.0), 5.0)
    return c


def test_le_cap_pointe_la_camera_gauche_vers_la_cible():
    """Vérifie que, pour regarder vers +x (l'est), le cap vaut −π/2 (drone tourné vers le sud) :
    la caméra gauche, à 90 degrés à gauche du cap, pointe alors vers l'est."""
    cap = PL.cap_pour_regarder(np.array([1.0, 0.0, 0.0]))     # regarder vers +x
    assert abs(cap - (-math.pi / 2)) < 1e-9                    # cap vers le sud, caméra gauche vers l'est


def test_une_piste_proche_bat_une_frontiere_lointaine():
    """Vérifie que, dans la salle, la piste repérée sur le mur (x = 3 m) passe avant les frontières
    lointaines : pose de lecture 2 m devant elle (x ≈ 1 m), caméra tournée vers le mur."""
    c = salle()
    c.integre_reperage(np.array([3.0, 0.0, 1.6]), np.array([-1.0, 0.0, 0.0]), t=0.0)
    cerveau = PL.Cerveau(c, drone=0)
    choix = cerveau.choisit(np.array([0.0, 0.0, 1.6]))
    assert choix is not None and choix.genre == "lire"
    assert abs(choix.position[0] - (3.0 - PL.D_LECTURE)) < 0.3 and abs(choix.position[1]) < 0.3
    assert abs(PL.cap_pour_regarder([1.0, 0.0, 0.0]) - choix.cap) < 1e-9


def test_une_zone_reservee_par_un_coequipier_est_evitee():
    """Vérifie qu'avec deux pistes (y = 2 et −2 m), le drone 0 laisse celle que le drone 1 a
    réservée (y = 2) et prend l'autre, tandis que le drone 1 garde la sienne."""
    c = salle()
    for y in (2.0, -2.0):
        c.integre_reperage(np.array([3.0, y, 1.6]), np.array([-1.0, 0.0, 0.0]), t=0.0)
    c.reserve(1, np.array([1.0, 2.0, 1.71]))
    choix = PL.Cerveau(c, drone=0).choisit(np.array([0.0, 0.0, 1.6]))
    assert choix is not None and choix.genre == "lire" and choix.origine[1] < 0
    # le drone 1, lui, garde sa propre réservation
    choix1 = PL.Cerveau(c, drone=1).choisit(np.array([0.0, 0.0, 1.6]))
    assert choix1 is not None and choix1.origine[1] > 0


def test_entre_deux_frontieres_equivalentes_la_plus_proche_gagne():
    """Vérifie que, dans un couloir déjà tout regardé, il ne reste que des frontières (aux deux
    bouts) et que le drone va vers la plus proche : x > 4 depuis x = 4, x < −4 depuis x = −4."""
    c = couloir()
    couvre_tout(c)
    cerveau = PL.Cerveau(c, drone=0)
    cands = cerveau.candidats()
    assert cands and all(k.genre == "explorer" for k in cands)
    choix = cerveau.choisit(np.array([4.0, 0.0, 1.6]), cands)
    assert choix is not None and choix.position[0] > 4.0
    choix = cerveau.choisit(np.array([-4.0, 0.0, 1.6]), cands)
    assert choix is not None and choix.position[0] < -4.0


def test_quand_tout_est_explore_il_ne_reste_rien():
    """Vérifie que sur une carte entièrement libre et regardée, il n'y a aucun candidat et que
    `choisit` renvoie None : plus rien à faire."""
    c = Carte()
    c.occupation[:] = -5.0
    couvre_tout(c)
    assert PL.Cerveau(c, drone=0).candidats() == []
    assert PL.Cerveau(c, drone=0).choisit(np.array([0.0, 0.0, 1.6])) is None


def test_une_surface_jamais_regardee_du_bon_cote_attire_puis_disparait():
    """Vérifie que la face ouest du mur, jamais regardée, crée une cible « couvrir » à 2 m devant
    (x ≈ 1 m), caméra vers le mur, qui disparaît une fois la face regardée depuis l'ouest."""
    c = salle()
    cands = PL.Cerveau(c, drone=0).candidats()
    faces = [k for k in cands if k.genre == "couvrir" and k.cote == 1]     # vues depuis l'ouest
    assert faces
    k = min(faces, key=lambda k: abs(k.origine[1]))
    assert abs(k.position[0] - (3.0 - PL.D_LECTURE)) < 0.4                 # à distance de lecture
    assert abs(PL.cap_pour_regarder([1.0, 0.0, 0.0]) - k.cap) < 1e-9        # caméra gauche vers le mur
    # une caméra regardant vers l'est a maintenant tout couvert : plus rien à faire de ce côté
    libre = c.occupation < M.SEUIL_LIBRE
    c.couverture[libre] |= np.uint8(1 << 0)
    cands = PL.Cerveau(c, drone=0).candidats()
    assert not [k for k in cands if k.genre == "couvrir" and k.cote == 1]


def test_un_carton_repere_vaut_plus_qu_un_mur_nu():
    """Vérifie qu'une portion de mur où l'œil appris a marqué des cartons (vers y = 2 m) vaut plus
    qu'une portion nue (vers y = −2 m) : une case carton compte 3 au lieu de 1."""
    c = salle()
    c.marque(np.array([[3.1, 2.0, 1.6], [3.1, 2.25, 1.6], [3.1, 1.75, 1.6], [3.1, 2.0, 1.85]]))
    cands = [k for k in PL.Cerveau(c, drone=0).candidats() if k.genre == "couvrir" and k.cote == 1]
    avec = max(k.utilite for k in cands if abs(k.origine[1] - 2.0) < 0.6)
    sans = max(k.utilite for k in cands if abs(k.origine[1] + 2.0) < 0.6)
    assert avec > sans


def test_trois_visites_sans_lecture_ecartent_la_piste():
    """Vérifie les trois chances d'une piste non lue : à 2 m, puis plus près (moins de 1,45 m),
    puis de l'autre côté ou plus loin ; après le 3e échec, la piste est écartée pour de bon."""
    c = salle()
    c.integre_reperage(np.array([3.0, 0.0, 1.6]), np.array([-1.0, 0.0, 0.0]), t=0.0)
    cerveau = PL.Cerveau(c, drone=0)
    choix = cerveau.choisit(np.array([0.0, 0.0, 1.6]))
    assert choix.genre == "lire"
    cerveau.constate(choix, lu=False)
    second = cerveau.choisit(np.array([0.0, 0.0, 1.6]))
    assert second.genre == "lire" and 3.0 - second.position[0] < 1.45   # une seconde chance, plus près
    cerveau.constate(choix, lu=False)
    troisieme = cerveau.choisit(np.array([0.0, 0.0, 1.6]))
    assert troisieme is None or troisieme.genre != "lire" or abs(3.0 - troisieme.position[0]) >= 1.45
    cerveau.constate(choix, lu=False)
    assert c.est_ecartee(np.array([3.0, 0.0, 1.6]))
    assert not [k for k in cerveau.candidats() if k.genre == "lire"]


def test_une_pose_dans_la_marge_d_un_obstacle_est_reculee():
    """Vérifie qu'une pose de lecture tombant dans la marge d'un second mur (x = 0,3 à 0,5 m) est
    rapprochée du panneau (x entre 1,1 et 1,5 m), et qu'une piste sans pose praticable est omise."""
    c = salle()
    # une piste tournée vers l'ouest sur un mur, mais un second mur derrière la pose à 2 m :
    # cette pose tombe dans sa marge, la pose se rapproche du panneau jusqu'à en sortir
    _bloc(c, (0.3, -4.0, 0.5), (0.5, 4.0, 3.0), 5.0)
    c.integre_reperage(np.array([3.0, 0.0, 1.6]), np.array([-1.0, 0.0, 0.0]), t=0.0)
    cands = [k for k in PL.Cerveau(c, drone=0).candidats() if k.genre == "lire"]
    assert cands and 0.5 + M.RAYON_DRONE < cands[0].position[0] < 3.0 - M.LIRE_MIN + 0.01
    # et si aucune pose de lecture n'est praticable, la piste n'est pas proposée
    c2 = salle()
    _bloc(c2, (0.8, -4.0, 0.5), (1.0, 4.0, 3.0), 5.0)
    c2.integre_reperage(np.array([3.0, 0.0, 1.6]), np.array([-1.0, 0.0, 0.0]), t=0.0)
    assert not [k for k in PL.Cerveau(c2, drone=0).candidats() if k.genre == "lire"]


def test_l_avis_du_guide_fait_pencher_la_balance():
    """Vérifie qu'entre deux pistes symétriques (y = 2 et −2 m), l'avis du guide (zone centrée sur
    la piste y = −2 m, côté ouest, poids λ = 1) fait choisir cette piste-là."""
    c = salle()
    for y in (2.0, -2.0):
        c.integre_reperage(np.array([3.0, y, 1.6]), np.array([-1.0, 0.0, 0.0]), t=0.0)
    p = np.array([0.0, 0.0, 1.6])
    sans = PL.Cerveau(c, drone=0, lam=0.0).choisit(p)
    avis = PL.Avis(centre=np.array([3.0, -2.0, 1.6]), rayon=1.0, cote=1)
    avec = PL.Cerveau(c, drone=0, lam=1.0).choisit(p, avis=avis)
    assert avec.origine[1] < 0
    assert sans.origine[1] * avec.origine[1] <= 0 or abs(sans.origine[1]) == abs(avec.origine[1])


def test_la_direction_d_apercu_se_moyenne_et_donne_le_cote():
    """Vérifie qu'une vue de biais puis trois de face donnent une direction moyenne tournée vers
    l'ouest (x < −0,9), des côtés de lecture ouest puis est, et que `oublie_pistes` l'efface."""
    c = salle()
    # vu d'abord de biais depuis le nord-ouest, puis de face depuis l'ouest : la moyenne penche vers l'ouest
    c.integre_reperage(np.array([3.0, 0.0, 1.6]), np.array([-0.5, 0.87, 0.0]), t=0.0)
    for _ in range(3):
        c.integre_reperage(np.array([3.0, 0.0, 1.6]), np.array([-1.0, 0.0, 0.0]), t=0.0)
    assert c.pistes[0].normale[0] < -0.9
    cotes = PL.Cerveau(c, drone=0)._cotes_possibles(c.pistes[0])
    assert np.allclose(cotes[0], [-1.0, 0.0, 0.0]) and np.allclose(cotes[1], [1.0, 0.0, 0.0])
    assert c.oublie_pistes(np.array([3.0, 0.2, 1.6]), 1.0) == 1 and not c.pistes


def test_le_cote_de_lecture_suit_le_grand_axe_du_rack_meme_au_bout():
    """Vérifie qu'une piste au bout nord de la face ouest d'un rack, aperçue à 70 degrés de biais,
    se lit depuis l'allée ouest : le côté suit le grand axe du rack, pas la direction d'aperçu."""
    c = Carte()
    _bloc(c, (-6.0, -6.0, 0.5), (6.0, 8.0, 3.0), -5.0)                # une salle libre
    _bloc(c, (2.0, -4.0, 0.5), (3.4, 4.0, 3.0), 5.0)                  # un rack le long de y
    # une piste sur la face ouest, tout au bout nord, aperçue depuis le nord-ouest (70 degrés)
    c.integre_reperage(np.array([2.1, 3.9, 1.6]), np.array([-0.3, 0.95, 0.0]), t=0.0)
    cotes = PL.Cerveau(c, drone=0)._cotes_possibles(c.pistes[0])
    assert np.allclose(cotes[0], [-1.0, 0.0, 0.0])                    # l'allée, pas le bout


def test_on_ne_survole_pas_une_structure():
    """Vérifie qu'un vol à 5,3 m est interdit au-dessus d'un rack de 4,6 m (il faut 2 m de vide
    dessous) mais reste permis au-dessus de l'allée (coût 1)."""
    c = Carte()
    _bloc(c, (-6.0, -4.0, 0.5), (6.0, 4.0, 6.0), -5.0)                 # une salle libre jusqu'au plafond
    _bloc(c, (2.0, -4.0, 0.5), (3.4, 4.0, 4.6), 5.0)                   # un rack de 4,6 m de haut
    cout = c.couts_de_vol(5.3)                                          # 0,7 m au-dessus du rack
    i, j, _ = c.indice([2.7, 0.0, 5.3])[0]
    assert np.isinf(cout[i, j])                                         # bloqué : le rack est à moins de 2 m dessous
    i, j, _ = c.indice([-3.0, 0.0, 5.3])[0]
    assert cout[i, j] == 1.0                                            # l'allée reste libre


def test_une_cible_reservee_par_un_autre_est_exclue():
    """Vérifie qu'une cible réservée par le drone 1 reçoit, pour le drone 0, une note sous −900
    (pénalité de 1000) : elle n'est prise que s'il ne reste rien d'autre."""
    c = salle()
    c.integre_reperage(np.array([3.0, 0.0, 1.6]), np.array([-1.0, 0.0, 0.0]), t=0.0)
    c.reserve(1, np.array([1.0, 0.0, 1.71]))
    cands = [k for k in PL.Cerveau(c, drone=0).candidats() if k.genre == "lire"]
    p = np.array([0.0, 0.0, 1.6])
    note = PL.Cerveau(c, drone=0).note(cands[0], p, 1.0)
    assert note < -900                                                  # exclue, sauf s'il ne reste rien
