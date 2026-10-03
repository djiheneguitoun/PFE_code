"""Tests de la méthode de Pore et al. (`pore.py`, la référence de 13_pore), sans simulateur.

La plupart des tests citent la phrase de l'article qu'ils contrôlent. Le premier vérifie que le
module n'emprunte rien à notre système : c'est la condition d'une comparaison honnête.
Lancement, depuis la racine du projet : python -m pytest swarm_qr/tests/test_pore.py -q
"""

from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from swarm_qr import pore as P  # noqa: E402
from swarm_qr.env.config import INTERIOR, RACKS  # noqa: E402
from swarm_qr.env.layout import make_layout  # noqa: E402

LAYOUT = make_layout(9033)          # l'entrepôt du vol nominal de l'évaluation finale


# ------------------------------------------------------------------ l'indépendance du module

def test_le_module_n_emprunte_rien_a_notre_systeme():
    """Vérifie, en lisant ses imports, que pore.py n'utilise ni la carte, ni le cerveau, ni
    l'observateur, ni la perception, ni le détecteur appris de notre système."""
    source = (Path(__file__).resolve().parents[1] / "pore.py").read_text()
    interdits = {"mapping", "planning", "observation", "detecteur", "perception", "baselines"}
    for noeud in ast.walk(ast.parse(source)):
        if isinstance(noeud, ast.Import):
            noms = {a.name.split(".")[-1] for a in noeud.names}
        elif isinstance(noeud, ast.ImportFrom):
            noms = {(noeud.module or "").split(".")[-1]}
        else:
            continue
        assert not (noms & interdits), f"pore.py importe {noms & interdits}"


# ------------------------------------------------------------------ le plan de couverture

def test_un_rack_entier_par_drone():
    """Vérifie le cas III, « a sector allocator assigns each vehicle a disjoint rack » : sur les
    entrepôts 9033 et 9019, chaque drone (3) reçoit un seul rack et toutes les faces sont prises."""
    for graine in (9033, 9019):
        lay = make_layout(graine)
        faces = P.faces_lisibles(lay)
        blocs = P.secteurs(faces, 3)
        assert sum(len(b) for b in blocs) == len(faces)
        assert all(bloc for bloc in blocs), f"un drone sans secteur sur l'entrepot {graine}"
        for bloc in blocs:
            assert len({f["rack"] for f in bloc}) == 1


def test_plus_de_drones_que_de_racks_coupe_le_rack_en_deux():
    """Vérifie le cas II, « two UAVs scanning the same rack from opposite ends » (deux drones sur un
    rack, depuis ses deux bouts) : avec 4 drones, le rack le plus chargé est coupé en deux."""
    faces = P.faces_lisibles(LAYOUT)
    blocs = P.secteurs(faces, 4)
    assert all(bloc for bloc in blocs)
    longueurs = [sum(f["y1"] - f["y0"] for f in b) for b in blocs]
    assert min(longueurs) > 0
    coupes = [b for b in blocs if any(f["y1"] - f["y0"] < 12.0 for f in b)]
    assert len(coupes) == 2, "le rack le plus charge est partage entre deux drones"


def test_moins_de_drones_que_de_racks_regroupe_les_voisins():
    """Vérifie qu'avec 2 drones pour 3 racks, des racks voisins sont regroupés : chaque drone a un
    secteur et toutes les faces sont attribuées."""
    blocs = P.secteurs(P.faces_lisibles(LAYOUT), 2)
    assert all(bloc for bloc in blocs)
    assert sum(len(b) for b in blocs) == len(P.faces_lisibles(LAYOUT))


def test_le_plan_monte_en_escalier_et_alterne_le_sens():
    """Vérifie l'escalier de l'article (« discrete shelf-level command z(t) » : altitude par paliers
    d'étage) : sur chaque face et étage, un seul sens, qui alterne ; les 3 étages sont visités."""
    for plan in P.plan_zigzag(LAYOUT, 3):
        par_etage: dict[tuple, list[float]] = {}
        for a in plan:
            par_etage.setdefault((a.face, a.etage), []).append(float(a.position[1]))
        for ys in par_etage.values():
            monte = all(u < v for u, v in zip(ys, ys[1:]))
            descend = all(u > v for u, v in zip(ys, ys[1:]))
            assert monte or descend
        sens = [ys[-1] > ys[0] for ys in par_etage.values()]
        assert all(u != v for u, v in zip(sens, sens[1:]))
        assert {a.etage for a in plan} == {0, 1, 2}


def test_les_drones_voisins_partent_de_bouts_opposes():
    """Vérifie les serpentins en miroir du §2.5 : deux drones voisins partent de bouts opposés de
    leur secteur (l'un vers les y croissants, l'autre vers les y décroissants)."""
    plans = P.plan_zigzag(LAYOUT, 3)
    montants = [p[0].position[1] < p[1].position[1] for p in plans if len(p) > 1]
    assert all(u != v for u, v in zip(montants, montants[1:]))


def test_chaque_arret_regarde_la_face_a_bonne_distance():
    """Vérifie chaque arrêt : à 0,9–1,65 m de la face visée, cap tourné vers elle, dans l'entrepôt,
    à la hauteur d'une étagère plus 22,5 cm."""
    for plan in P.plan_zigzag(LAYOUT, 3):
        for a in plan:
            d = float(np.linalg.norm(a.vise[:2] - a.position[:2]))
            assert P.RECUL_MIN <= d <= P.RECUL + 1e-6
            vers = a.vise[:2] - a.position[:2]
            assert abs(math.atan2(vers[1], vers[0]) - a.cap) < 1e-6
            assert INTERIOR.x_min < a.position[0] < INTERIOR.x_max
            assert a.position[2] in [z + P.HAUTEUR_PANNEAU for z in RACKS.shelf_levels]


def test_chaque_drone_recoit_le_secteur_le_plus_proche():
    """Vérifie que `attribue` donne à chaque drone le secteur qui commence le plus près de lui :
    départs posés au début des secteurs 2, 0 et 1, secteurs reçus 2, 0 et 1."""
    plans = P.plan_zigzag(LAYOUT, 3)
    departs = [plans[2][0].position, plans[0][0].position, plans[1][0].position]
    donnes = P.attribue(plans, departs)
    assert [id(p) for p in donnes] == [id(plans[2]), id(plans[0]), id(plans[1])]


# ------------------------------------------------------------------ le champ de risque

def test_le_risque_monte_pres_d_un_rack_et_tombe_au_milieu_de_l_allee():
    """Vérifie l'équation 9, R_f = σ(β (r_safe − r(p))) avec r_safe = 0,5 m : à 10 cm d'un rack, le
    risque dépasse le seuil de 0,4 (point non sûr) ; à 2,2 m, il est sous 0,05."""
    champ = P.ChampDeRisque(LAYOUT)
    rack = LAYOUT.racks[0]
    colle = np.array([rack.x_bounds[1] + 0.1, 0.0, 1.6])
    loin = np.array([rack.x_bounds[1] + 2.2, 0.0, 1.6])
    assert champ.risque(colle) > P.TAU and not champ.sur(colle)
    assert champ.risque(loin) < 0.05 and champ.sur(loin)
    assert abs(champ.distance(colle) - 0.1) < 0.02


def test_un_echo_lidar_cree_du_risque_puis_s_efface():
    """Vérifie qu'un écho lidar à 45 cm rend le point non sûr, puis qu'il est oublié 10 s plus tard
    (mémoire de 3 s) : le champ suit le lidar sans mémoire longue."""
    champ = P.ChampDeRisque(LAYOUT, memoire_s=3.0)
    p = np.array([0.0, 0.0, 1.6])
    assert champ.sur(p)
    champ.integre_lidar(p, np.array([[1.0, 0.0, 0.0]]), np.array([0.45]), t=10.0)
    assert not champ.sur(p)
    champ.integre_lidar(p, np.array([[0.0, 0.0, 1.0]]), np.array([30.0]), t=20.0)
    assert champ.sur(p)                                   # l'écho a expiré


def test_un_coequipier_est_un_obstacle_du_champ():
    """Vérifie qu'un coéquipier annoncé à 40 cm rend le point non sûr, et que le point redevient
    sûr quand plus aucun coéquipier n'est annoncé."""
    champ = P.ChampDeRisque(LAYOUT)
    p = np.array([0.0, 0.0, 1.6])
    champ.annonce_coequipiers([np.array([0.4, 0.0, 1.6])])
    assert not champ.sur(p)
    champ.annonce_coequipiers([])
    assert champ.sur(p)


def test_la_deflexion_ecarte_du_risque_sans_s_eloigner():
    """Vérifie la déflexion du §2.3, « minimally deflecting the nominal path while penalizing
    risk » (dévier peu) : près d'un écho lidar, point sûr, à 1,2 m au plus, à la même altitude."""
    champ = P.ChampDeRisque(LAYOUT)
    p = np.array([0.0, 0.0, 1.6])
    champ.integre_lidar(p + np.array([0.3, 0.0, 0.0]), np.array([[1.0, 0.0, 0.0]]), np.array([0.5]), t=0.0)
    q = P.deflexion(p, champ)
    assert q is not None and champ.sur(q)
    assert float(np.linalg.norm(q[:2] - p[:2])) <= P.DEFLEXION_MAX
    assert abs(q[2] - p[2]) < 1e-9                        # la déflexion reste dans le plan


def test_la_deflexion_renonce_quand_il_n_y_a_pas_d_issue():
    """Vérifie que la déflexion renonce (renvoie None) pour un point au milieu d'un rack, d'où
    aucune issue sûre n'existe à moins de 30 cm."""
    champ = P.ChampDeRisque(LAYOUT)
    rack = LAYOUT.racks[0]
    dedans = np.array([0.5 * (rack.x_bounds[0] + rack.x_bounds[1]), 0.0, 1.6])
    assert P.deflexion(dedans, champ, maxi=0.3) is None


# ------------------------------------------------------------------ le routage dans les allées

def test_le_chemin_reste_dans_les_allees_et_ne_traverse_aucun_rack():
    """Vérifie le §2.3, « linear keep-in constraints that restrict motion to the aisle polytope »
    (rester dans les allées) : d'une allée à l'autre, on passe par un couloir sans couper de rack."""
    r = P.Routeur(LAYOUT)
    plans = P.plan_zigzag(LAYOUT, 3)
    a = plans[0][0].position
    b = plans[2][0].position
    points = r.chemin(a, b)
    assert points, "deux allées différentes : il faut passer par un couloir"
    route = [a] + points + [b]
    for u, v in zip(route[:-1], route[1:]):
        assert not r.coupe_un_rack(u, v)


def test_deux_arrets_voisins_se_rejoignent_en_ligne_droite():
    """Vérifie que les deux premiers arrêts d'un plan, voisins sur la même face (1,5 m d'écart), se
    rejoignent en ligne droite, sans point de passage."""
    r = P.Routeur(LAYOUT)
    plan = P.plan_zigzag(LAYOUT, 3)[0]
    assert r.chemin(plan[0].position, plan[1].position) == []


def test_le_routeur_choisit_le_couloir_le_plus_court():
    """Vérifie que, de l'allée 0 à l'allée 2 à y = 0 (plus près du bout sud des racks), le routeur
    contourne par le couloir sud, le plus court, et non par le nord."""
    r = P.Routeur(LAYOUT)
    sud, nord = r.couloirs
    y = 0.0                                                # au milieu des racks, plus près du sud
    a = np.array([r.allees[0]["centre"], y, 1.6])
    b = np.array([r.allees[2]["centre"], y, 1.6])
    points = r.chemin(a, b)
    assert points, "un rack sépare les deux allées"
    assert all(abs(q[1] - sud) < 1e-6 for q in points)
    assert all(abs(q[1] - nord) > 1.0 for q in points)


# ------------------------------------------------------------------ la lecture au sol

def _image_avec_code(texte: str, taille: int = 420, flou: int = 0) -> np.ndarray:
    """Fabrique l'image BGR d'un vrai QR codant `texte` (`taille` px, marge blanche de 60 px),
    floutée par un filtre gaussien si `flou` > 0."""
    import cv2
    det = cv2.QRCodeEncoder.create()
    code = det.encode(texte)
    code = cv2.resize(code, (taille, taille), interpolation=cv2.INTER_NEAREST)
    fond = np.full((taille + 120, taille + 120), 255, np.uint8)
    fond[60:60 + taille, 60:60 + taille] = code
    bgr = cv2.cvtColor(fond, cv2.COLOR_GRAY2BGR)
    return cv2.GaussianBlur(bgr, (flou * 2 + 1, flou * 2 + 1), 0) if flou else bgr


def test_le_decodeur_opencv_lit_un_code_net():
    """Vérifie que la lecture au sol (OpenCV) lit exactement « BOX_042 » sur un QR net, avec une
    confiance positive."""
    lus = P.decode(_image_avec_code("BOX_042"))
    assert [c for c, _ in lus] == ["BOX_042"]
    assert lus[0][1] > 0.0


def test_la_confiance_baisse_quand_l_image_est_floue():
    """Vérifie que le même QR flouté (filtre gaussien 9 × 9) est encore lu, mais avec une confiance
    (netteté) plus basse que l'image nette."""
    net = P.decode(_image_avec_code("BOX_042"))
    flou = P.decode(_image_avec_code("BOX_042", flou=4))
    assert net and flou
    assert net[0][1] > flou[0][1]


def test_le_decodeur_ne_lit_rien_sur_une_image_vide():
    """Vérifie qu'une image grise unie de 300 × 300 px ne donne aucune lecture (aucun code inventé)."""
    assert P.decode(np.full((300, 300, 3), 200, np.uint8)) == []


# ------------------------------------------------------------------ l'inventaire au sol

def test_un_code_n_entre_qu_une_fois_a_l_inventaire():
    """Vérifie qu'un code relu 5 s plus tard par un autre drone n'entre pas deux fois : un seul code
    à l'inventaire, un doublon compté."""
    inv = P.Inventaire()
    assert inv.recoit("A1", 0, np.zeros(3), t=0.0, confiance=0.9)
    assert not inv.recoit("A1", 1, np.zeros(3), t=5.0, confiance=0.9)
    assert inv.codes == {"A1"} and inv.doublons == 1


def test_les_relectures_rapprochees_sont_ecartees():
    """Vérifie l'anti-doublon de l'article, « dedup dwell ∆t_dedup (e.g., 1.5 s) » : une relecture
    0,5 s après la première n'est pas enregistrée, elle compte comme doublon."""
    inv = P.Inventaire(dedup_s=1.5)
    inv.recoit("A1", 0, np.zeros(3), t=0.0, confiance=0.9)
    inv.recoit("A1", 0, np.zeros(3), t=0.5, confiance=0.9)
    assert len(inv.evenements) == 1 and inv.doublons == 1


def test_une_lecture_peu_sure_reprogramme_l_arret():
    """Vérifie le §2.4 (lectures peu sûres reprogrammées) : une lecture de confiance 0,2, sous le
    seuil de 0,5, met son arrêt dans la liste des arrêts à refaire."""
    inv = P.Inventaire(confiance_min=0.5)
    arret = P.plan_zigzag(LAYOUT, 3)[0][0]
    inv.recoit("A1", 0, np.zeros(3), t=0.0, confiance=0.2, arret=arret)
    assert arret.cle() in inv.a_relire


# ------------------------------------------------------------------ la coordination

def test_le_temps_avant_conflit_voit_venir_une_rencontre_frontale():
    """Vérifie que deux drones face à face à 10 m, à 1 m/s chacun, seront à moins de 1,2 m dans
    4,4 s (8,8 m refermés à 2 m/s)."""
    t = P.temps_avant_conflit([0, 0, 1.6], [1, 0, 0], [10, 0, 1.6], [-1, 0, 0])
    assert 4.0 < t < 4.5                                   # 8,8 m à refermer à 2 m/s


def test_deux_drones_qui_s_eloignent_n_ont_pas_de_conflit():
    """Vérifie que deux drones à 5 m qui s'éloignent l'un de l'autre n'ont jamais de conflit :
    temps avant conflit infini."""
    assert P.temps_avant_conflit([0, 0, 1.6], [-1, 0, 0], [5, 0, 1.6], [1, 0, 0]) == float("inf")


def test_le_suiveur_cede_le_passage_au_meneur():
    """Vérifie la règle du jeton (cas II) : pour un conflit dans 2,4 s (seuil 3 s), le drone 1
    s'arrête mais pas le drone 0 ; pour un conflit dans 9,4 s, personne ne s'arrête."""
    autres_pour_1 = [(0, [6, 0, 1.6], [-1, 0, 0])]         # conflit dans 2,4 s, sous les 3 s
    assert P.cede_le_passage(1, [0, 0, 1.6], [1, 0, 0], autres_pour_1)
    autres_pour_0 = [(1, [6, 0, 1.6], [-1, 0, 0])]
    assert not P.cede_le_passage(0, [0, 0, 1.6], [1, 0, 0], autres_pour_0)
    loin = [(0, [20, 0, 1.6], [-1, 0, 0])]                 # conflit dans 9,4 s : personne ne cède
    assert not P.cede_le_passage(1, [0, 0, 1.6], [1, 0, 0], loin)


def test_la_vitesse_baisse_quand_la_confiance_baisse():
    """Vérifie le ralentissement du cas III : pleine vitesse pour une confiance de 0,9 ou sans
    lecture, vitesse réduite (entre 0,3 et 1 m/s) pour 0,1, sous le seuil de 0,35."""
    assert P.vitesse_selon_confiance(1.0, 0.9) == 1.0
    assert P.vitesse_selon_confiance(1.0, None) == 1.0
    lent = P.vitesse_selon_confiance(1.0, 0.1, seuil=0.35)
    assert 0.3 <= lent < 1.0


# ------------------------------------------------------------------ les portes avant vol

def test_le_plan_met_chaque_code_dans_le_champ_et_dans_la_portee_d_opencv():
    """Vérifie, avec les vrais codes des vols 9033 et 9019, que chaque code est vu d'au moins un
    arrêt, dans le champ et à 1,0–2,7 m (portée OpenCV, étape 2) : le plan ne perd aucun code."""
    import json
    from swarm_qr.env.config import CAMERAS
    fov_h = math.radians(CAMERAS.fov_deg)
    fov_v = 2 * math.atan(math.tan(fov_h / 2) * CAMERAS.side_height / CAMERAS.side_width)
    base = Path(__file__).resolve().parents[1] / "experiments" / "11_mission" / "tests of system"
    for graine, dossier in ((9033, "eval_nominal"), (9019, "eval_9019")):
        fichier = base / dossier / "mission.json"
        if not fichier.exists():
            continue
        tags = json.loads(fichier.read_text())["verite"]
        vus = set()
        for plan in P.plan_zigzag(make_layout(graine), 3):
            for a in plan:
                axe = a.vise - a.position
                axe = axe / np.linalg.norm(axe)
                for t in tags:
                    v = np.asarray(t["position"], float) - a.position
                    d = float(np.linalg.norm(v))
                    avant = float(v @ axe)
                    if not (1.0 <= d <= 2.7) or avant <= 0:
                        continue
                    lat = v - avant * axe
                    ah = abs(math.atan2(float(lat[1]) if abs(axe[0]) > 0.5 else float(lat[0]), avant))
                    av = abs(math.atan2(float(lat[2]), avant))
                    if ah <= fov_h / 2 and av <= fov_v / 2:
                        vus.add(t["code"])
        n = len({t["code"] for t in tags})
        assert len(vus) == n, f"entrepot {graine} : {n - len(vus)} codes hors de portee du plan"


def test_le_recul_reste_dans_la_portee_utile_d_opencv():
    """Vérifie que le recul (1,65 m) est entre 1,5 et 1,8 m, où OpenCV lit 92 % des codes (sous
    80 % au-delà de 2,2 m), et qu'aucun arrêt n'est plus loin de sa face."""
    assert 1.5 <= P.RECUL <= 1.8
    for plan in P.plan_zigzag(LAYOUT, 3):
        for a in plan:
            assert float(np.linalg.norm(a.vise[:2] - a.position[:2])) <= P.RECUL + 1e-6


def test_la_mission_n_appelle_que_des_methodes_qui_existent():
    """Vérifie, en lisant les sources (pore_mission.py ne s'importe pas sans Isaac Sim), que chaque
    méthode ou attribut qu'il utilise existe : un `clock.tick()` inexistant a déjà coûté un vol."""
    import re
    racine = Path(__file__).resolve().parents[1]
    attendus = {
        "env/pilot.py": {"Clock": ["t"]},
        "control.py": {"Controleur": ["assigne", "bilan", "consigne", "i_point", "points",
                                      "tick", "v_transit"]},
        "pore.py": {"ChampDeRisque": ["annonce_coequipiers", "integre_lidar", "sur"],
                    "Routeur": ["chemin"], "Inventaire": ["bilan", "codes", "recoit"]},
        "env/scene.py": {"Scene": ["drones", "finalize", "lidar", "position", "tags",
                                   "velocity", "world", "yaw"]},
    }
    for fichier, classes in attendus.items():
        texte = (racine / fichier).read_text()
        for classe, noms in classes.items():
            suite = texte[texte.index(f"class {classe}"):]
            fin = suite.find("\nclass ", 1)
            corps = suite[:fin if fin > 0 else len(suite)]
            for n in noms:
                motif = (rf"(def {re.escape(n)}\b|self\.{re.escape(n)}\s*[:=]"
                         rf"|^\s*{re.escape(n)}\s*[:=])")
                assert re.search(motif, corps, re.M), f"{classe}.{n} n'existe pas"
    source = (racine / "pore_mission.py").read_text()
    for nom in ("Arret", "ChampDeRisque", "Inventaire", "Routeur", "attribue", "cede_le_passage",
                "decode", "deflexion", "plan_zigzag", "vitesse_selon_confiance"):
        if f"pore.{nom}" in source:
            assert hasattr(P, nom), f"pore.{nom} n'existe pas"
