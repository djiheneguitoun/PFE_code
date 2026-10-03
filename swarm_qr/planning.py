"""Le « cerveau » d'un drone : choisit sa prochaine cible d'après la carte partagée (étape 5).

Trois genres de cibles, jamais tirées du plan : LIRE une piste (QR repéré, pas encore lu), COUVRIR
une surface jamais regardée d'assez près, EXPLORER une frontière entre connu et inconnu. Note d'une
cible : utilité − 1 point par mètre − pénalités (réservation, voisin) + λ × bonus du guide ; les 6
meilleures sont vérifiées par un vrai chemin. Appelé par mission.py ; tests : tests/test_planning.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import mapping
from .env.config import CAMERAS

D_LECTURE = 2.0              # m, au milieu de l'enveloppe de lecture 1,5–4 m (étape 2)
UTILITE_LIRE = 30.0          # points : utilité d'une piste à lire
UTILITE_SURFACE = 1.0        # point par cube (25 cm) de surface jamais regardé du bon côté...
BONUS_CARTON = 3.0           # ...trois fois plus si l'œil appris y a vu un carton
PLAFOND_COUVRIR = 30.0       # points : utilité maximale d'un groupe de surfaces (autant qu'une piste)
UTILITE_FRONTIERE = 0.15     # point par case de frontière (25 cm) du groupe
PLAFOND_EXPLORER = 20.0      # points : utilité maximale d'un groupe de frontière
COUT_METRE = 1.0             # point retiré par mètre de trajet
PENALITE_RESERVE = 1000.0    # points : une cible réservée par un autre est exclue, sauf s'il ne reste rien
RAYON_RESERVE = 4.0          # m : rayon autour de la cible réservée où cette pénalité s'applique
PENALITE_VOISIN = 40.0       # points : une cible à moins de 4 m d'un coéquipier coûte plus qu'une piste
RAYON_VOISIN = 4.0           # m
BONUS_ZONE = 10.0            # points, multipliés par λ : cible dans la zone conseillée par le guide...
BONUS_COTE = 5.0             # ...et abordée par le côté conseillé
GROUPE_XY = 1.0              # m, taille des groupes de surfaces
GROUPE_Z = 0.5               # m, hauteur des groupes de surfaces
GROUPE_FRONTIERE = 2.0       # m, taille des groupes de cases de frontière
SURFACE_MIN = 3              # cubes ; en dessous, c'est du bruit
Z_MIN, Z_MAX = 0.9, 4.5      # m, bande de vol ; au-dessus, on survole les racks, et leurs panneaux de signalisation sont à 5 m
PISTE_Z_MAX = 5.0            # m ; une piste plus haute que le dernier étage est un fantôme
ALT_MIN = 1.4                # m, altitude minimale d'une pose
ALT_EXPLORATION = 1.8        # m, altitude des poses d'exploration
DEGAGEMENT_MAX = 1.25        # m, jusqu'où déplacer une pose d'exploration prise dans la marge d'un obstacle
MARGE_POSE = 0.9             # m ; une pose tenue oscille : plus loin des obstacles que le trajet (0,6)
CANDIDATS_VERIFIES = 6       # les meilleurs par la ligne droite reçoivent un vrai chemin

# Les 4 côtés, numérotés comme Carte.cardinal : 0 est (+x), 1 ouest (−x), 2 nord (+y), 3 sud (−y).
CARDINAUX = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]])
OPPOSE = {0: 1, 1: 0, 2: 3, 3: 2}            # numéro du côté opposé
NOMS_COTES = {0: "est", 1: "ouest", 2: "nord", 3: "sud"}


@dataclass
class Cible:
    """Une cible candidate : où placer le drone, ce qu'il y regardera, et sa note."""
    genre: str                  # lire / couvrir / explorer
    position: np.ndarray        # m : la pose où placer le drone
    cap: float | None           # rad ; None : garder le cap courant
    utilite: float              # points, avant le coût du trajet et les pénalités
    origine: np.ndarray         # m : ce qu'on va regarder (piste, centre de surface, frontière)
    cote: int                   # côté d'où l'on regarde (cardinal de origine → drone)
    distance: float = math.inf  # m : longueur du trajet (ligne droite, puis vrai chemin)
    note: float = -math.inf     # calculée par Cerveau.note
    points: list = field(default_factory=list)   # points de passage du chemin calculé sur la carte

    def cle(self) -> tuple:
        """Renvoie une clé qui identifie la cible : son genre et le point regardé, arrondi à 10 cm."""
        return (self.genre, *np.round(self.origine, 1).tolist())


@dataclass
class Avis:
    """Le conseil du guide : une zone (centre, rayon) et un côté d'abordage."""
    centre: np.ndarray          # m
    rayon: float                # m
    cote: int | None = None     # numéro du côté conseillé (voir CARDINAUX) ; None : pas d'avis
    phrase: str = ""            # résumé lisible de l'avis, pour le journal


def cap_pour_regarder(direction) -> float:
    """Renvoie le cap (rad) qui pointe la caméra gauche vers `direction` : cette caméra regarde
    à 90° à gauche du cap, donc le cap vaut la direction moins 90°."""
    return math.atan2(direction[1], direction[0]) - math.pi / 2.0


def _horizontal(v) -> np.ndarray | None:
    """Renvoie `v` ramené à l'horizontale et normalisé, ou None s'il est vertical (ou nul)."""
    h = np.array([v[0], v[1], 0.0], dtype=float)
    n = np.linalg.norm(h)
    return h / n if n > 1e-6 else None


class Cerveau:
    """La décision d'un drone ; sa seule mémoire : ses visites sans lecture de chaque piste (3 au plus)."""

    def __init__(self, carte: mapping.Carte, drone: int, lam: float = 0.0):
        """Prépare le cerveau du drone `drone` ; `lam` = poids λ de l'avis du guide (0 : sans guide)."""
        self.carte, self.drone, self.lam = carte, drone, lam
        self.tentatives: list[list] = []          # [position, nombre de visites sans lecture]
        self._couts: dict[int, np.ndarray] = {}   # cartes de coûts déjà calculées, par tranche de 25 cm

    # ---------------------------------------------------------------- candidats

    CACHE_S = 2.0          # s : durée de vie de la liste des candidats partagée sur la carte

    def candidats(self, t: float | None = None) -> list[Cible]:
        """Renvoie les cibles candidates, communes à tous les drones et recalculées au plus toutes les 2 s :
        un calcul par drone et par cycle bloquerait la physique, et le pilote ArduPilot perdrait le drone."""
        cache = getattr(self.carte, "_cache_cibles", None)
        if t is not None and cache is not None and 0.0 <= t - cache[0] < self.CACHE_S:
            return cache[1]
        self._couts = {}
        cibles = self._lire() + self._couvrir() + self._explorer()
        if t is not None:
            self.carte._cache_cibles = (t, cibles)
        return cibles

    def _couts_a(self, z: float) -> np.ndarray:
        """Renvoie la carte des coûts (vue de dessus) pour une pose à l'altitude `z`, avec la marge
        MARGE_POSE ; gardée en mémoire par tranche de 25 cm."""
        k = int(round(z / 0.25))
        if k not in self._couts:
            self._couts[k] = self.carte.couts(z - mapping.DESSOUS, z + mapping.EPAISSEUR, marge=MARGE_POSE)
        return self._couts[k]

    def _praticable(self, p) -> bool:
        """Vérifie que `p` est sur la carte et qu'une pose y tient, à MARGE_POSE de tout obstacle connu."""
        if not self.carte.sur_la_carte(p)[0]:
            return False
        i, j, _ = self.carte.indice(p)[0]
        return bool(np.isfinite(self._couts_a(float(p[2]))[i, j]))

    DISTANCES_LECTURE = (2.0, 1.75, 1.5, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0)   # m, dans l'ordre d'essai
    DISTANCES_PRES = (1.2, 1.0, 1.4, 0.9)       # m ; une étiquette de 12 cm se lit vers 1,2 m (étape 7)

    def _pose_de_lecture(self, origine, n, distances=DISTANCES_LECTURE) -> np.ndarray | None:
        """Renvoie la première pose valable devant `origine`, du côté `n`, aux `distances` essayées dans
        l'ordre (None sinon) : praticable, et sans obstacle connu entre la pose et la surface."""
        for d in distances:
            # drone 11 cm plus haut que le point visé : ses caméras latérales sont 11 cm sous son centre
            pose = np.asarray(origine, dtype=float) + n * d + np.array([0.0, 0.0, CAMERAS.below])
            pose[2] = float(np.clip(pose[2], ALT_MIN, Z_MAX))
            if not self._praticable(pose):
                continue
            # un obstacle connu à plus de 40 cm avant la surface cacherait la vue
            devant = self.carte.premier_obstacle(pose, -n, portee=d + 0.5)
            if devant is not None and devant < d - 0.4:
                continue
            return pose
        return None

    def point_degage(self, position, marge: float) -> np.ndarray | None:
        """Renvoie le point le plus proche de `position`, à la même altitude et à `marge` de tout obstacle
        connu ; None s'il n'y en a pas à moins de 3 m. Sert à changer d'altitude loin des racks (mission.py)."""
        p = np.asarray(position, dtype=float)
        cout = self.carte.couts(p[2] - mapping.DESSOUS, p[2] + mapping.EPAISSEUR, marge=marge)
        pas = self.carte.g.cell
        n = int(3.0 / pas)
        for dx, dy in sorted(((dx, dy) for dx in range(-n, n + 1) for dy in range(-n, n + 1)),
                             key=lambda o: o[0] * o[0] + o[1] * o[1]):
            q = p + np.array([dx * pas, dy * pas, 0.0])
            if not self.carte.sur_la_carte(q)[0]:
                continue
            i, j, _ = self.carte.indice(q)[0]
            if np.isfinite(cout[i, j]):
                return q
        return None

    def _pose_libre(self, centre) -> np.ndarray | None:
        """Renvoie la pose praticable la plus proche de `centre` (altitude bornée entre ALT_MIN et Z_MAX),
        à moins de DEGAGEMENT_MAX ; None sinon."""
        c = np.asarray(centre, dtype=float).copy()
        c[2] = float(np.clip(c[2], ALT_MIN, Z_MAX))
        pas = self.carte.g.cell
        n = int(DEGAGEMENT_MAX / pas)
        offsets = sorted(((dx, dy) for dx in range(-n, n + 1) for dy in range(-n, n + 1)),
                         key=lambda o: o[0] * o[0] + o[1] * o[1])
        for dx, dy in offsets:
            if dx * dx + dy * dy > n * n:
                continue
            q = c + np.array([dx * pas, dy * pas, 0.0])
            if self._praticable(q):
                return q
        return None

    def normale_par_la_carte(self, position, indice=None) -> np.ndarray | None:
        """Renvoie le côté (vecteur) d'où lire un panneau : perpendiculaire au grand axe du rack qui le
        porte, là où l'espace libre est le plus proche (à égalité : vers l'aperçu `indice`) ; ou None."""
        # La direction d'aperçu seule ne suffit pas : un QR aperçu à 70° de biais se lit de face.
        c = self.carte
        p = np.asarray(position, dtype=float)
        # cases occupées dans un carré de 6 m autour du point, sur ±0,75 m de hauteur
        k0, k1 = c._tranche(p[2] - 0.75, p[2] + 0.75)
        i0, j0, _ = c.indice(p - 3.0)[0]
        i1, j1, _ = c.indice(p + 3.0)[0]
        bloc = c.occupation[max(i0, 0):i1 + 1, max(j0, 0):j1 + 1, k0:k1] > mapping.SEUIL_OCCUPE
        ij = np.argwhere(bloc.any(axis=2)) + [max(i0, 0), max(j0, 0)]
        if len(ij) < 10:
            # trop peu de structure autour : on se contente de la direction d'aperçu
            return None if indice is None else CARDINAUX[c.cardinal(indice)].copy()
        xy = ij * c.g.cell
        xy = xy - xy.mean(axis=0)
        # grand axe = vecteur propre de la plus grande valeur propre (analyse en composantes principales)
        _, vecteurs = np.linalg.eigh(xy.T @ xy)
        axe = vecteurs[:, -1]                                      # le grand axe, horizontal
        # côtés à plus de 60° du grand axe
        perpendiculaires = [k for k, d in enumerate(CARDINAUX) if abs(np.dot(d[:2], axe)) < 0.5]
        if not perpendiculaires:
            return None if indice is None else CARDINAUX[c.cardinal(indice)].copy()
        pas = np.array([[0.25], [0.5], [0.75], [1.0], [1.25], [1.5]])     # m, testés le long de chaque côté

        def premier_libre(d):
            """Renvoie la distance (m) de la première case libre dans la direction `d`, ou l'infini."""
            etats = c.etat(p + d[None, :] * pas)
            libres = np.flatnonzero(etats == mapping.LIBRE)
            return float(pas[libres[0], 0]) if len(libres) else np.inf

        # le côté où l'espace libre est le plus proche ; à égalité, le plus tourné vers l'aperçu
        meilleur = min(perpendiculaires, key=lambda k: (premier_libre(CARDINAUX[k]),
                                                        0.0 if indice is None else -float(np.dot(CARDINAUX[k][:2], indice[:2]))))
        return CARDINAUX[meilleur].copy() if np.isfinite(premier_libre(CARDINAUX[meilleur])) else None

    def _cotes_possibles(self, piste) -> list[np.ndarray]:
        """Renvoie les côtés d'où lire une piste : celui qu'impose le rack porteur, puis son opposé
        (pour la dernière chance) ; liste vide si la carte ne permet pas de décider."""
        apercu = None if piste.normale is None else _horizontal(piste.normale)
        n = self.normale_par_la_carte(piste.position, apercu)
        if n is None:
            return []
        return [n, -n]

    def _lire(self) -> list[Cible]:
        """Renvoie une cible LIRE par piste encore valable, avec une pose qui dépend du nombre de visites."""
        out = []
        for piste in self.carte.pistes:
            if self.carte.est_ecartee(piste.position) or piste.position[2] > PISTE_Z_MAX:
                continue
            cotes = self._cotes_possibles(piste)
            if not cotes:
                continue
            # trois chances : à 2 m ; puis plus près, à 1,2 m, parce que les petites
            # étiquettes ne se lisent pas de loin (étape 7) ; puis de l'autre côté. Ensuite
            # la piste est écartée.
            visites = self._visites(piste.position)
            essais = ([(cotes[0], self.DISTANCES_LECTURE)] if visites == 0 else
                      [(cotes[0], self.DISTANCES_PRES), (cotes[0], self.DISTANCES_LECTURE)] if visites == 1 else
                      [(c, self.DISTANCES_LECTURE) for c in cotes[::-1]])
            pose, n = None, None
            for n, distances in essais:
                pose = self._pose_de_lecture(piste.position, n, distances)
                if pose is not None:
                    break
            if pose is None:
                continue
            out.append(Cible("lire", pose, cap_pour_regarder(-n), UTILITE_LIRE,
                             piste.position.copy(), self.carte.cardinal(n)))
        return out

    def _couvrir(self) -> list[Cible]:
        """Renvoie les cibles COUVRIR : surfaces occupées dont la case libre d'en face n'a jamais été
        regardée de ce côté, groupées par mètre (et par demi-mètre de hauteur)."""
        c = self.carte
        k0, k1 = c._tranche(Z_MIN, Z_MAX)
        occ = c.occupation[:, :, k0:k1]
        dur = occ > mapping.SEUIL_OCCUPE
        libre = occ < mapping.SEUIL_LIBRE
        couv = c.couverture[:, :, k0:k1]
        groupes: dict[tuple, list] = {}
        for cote, (dx, dy) in enumerate([(1, 0), (-1, 0), (0, 1), (0, -1)]):
            bit = np.uint8(1 << OPPOSE[cote])          # une caméra regardant vers la surface
            # surface en (i, j), case libre en face en (i+dx, j+dy)
            sx = slice(max(-dx, 0), dur.shape[0] - max(dx, 0))
            sy = slice(max(-dy, 0), dur.shape[1] - max(dy, 0))
            fx = slice(max(dx, 0), dur.shape[0] - max(-dx, 0))
            fy = slice(max(dy, 0), dur.shape[1] - max(-dy, 0))
            a_faire = dur[sx, sy, :] & libre[fx, fy, :] & ((couv[fx, fy, :] & bit) == 0)
            idx = np.argwhere(a_faire)
            if not len(idx):
                continue
            idx[:, 0] += sx.start
            idx[:, 1] += sy.start
            idx[:, 2] += k0
            centres = c.centre(idx)
            carton = c.semantique[tuple(idx.T)] == mapping.SEM_CARTON
            cles = np.column_stack([np.full(len(idx), cote), np.floor(centres[:, 0] / GROUPE_XY),
                                    np.floor(centres[:, 1] / GROUPE_XY), np.floor(centres[:, 2] / GROUPE_Z)])
            for cle, p, est_carton in zip(map(tuple, cles.astype(int)), centres, carton):
                groupes.setdefault(cle, []).append((p, est_carton))
        out = []
        for cle, elems in groupes.items():
            if len(elems) < SURFACE_MIN:
                continue
            cote = int(cle[0])
            d = CARDINAUX[cote]
            pts = np.array([p for p, _ in elems])
            centre = pts.mean(axis=0)
            # 1 point par cube, 3 si l'œil appris y a vu un carton, 30 au plus
            utilite = min(sum(BONUS_CARTON if k else UTILITE_SURFACE for _, k in elems), PLAFOND_COUVRIR)
            pose = self._pose_de_lecture(centre, d)
            if pose is None or self.carte.est_ecartee(pose):
                continue
            out.append(Cible("couvrir", pose, cap_pour_regarder(-d), utilite, centre, cote))
        return out

    def _explorer(self) -> list[Cible]:
        """Renvoie les cibles EXPLORER : cases de frontière groupées par carrés de 2 m, avec une pose
        à 1,8 m d'altitude près du centre de chaque groupe."""
        pts = self.carte.frontieres(Z_MIN, Z_MAX)
        if not len(pts):
            return []
        cles = np.floor(pts[:, :2] / GROUPE_FRONTIERE).astype(int)
        groupes: dict[tuple, list] = {}
        for cle, p in zip(map(tuple, cles), pts):
            groupes.setdefault(cle, []).append(p)
        out = []
        for elems in groupes.values():
            centre = np.mean(elems, axis=0)
            pose = self._pose_libre(np.array([centre[0], centre[1], ALT_EXPLORATION]))
            if pose is None or self.carte.est_ecartee(pose):
                continue
            out.append(Cible("explorer", pose, None, min(UTILITE_FRONTIERE * len(elems), PLAFOND_EXPLORER),
                             pose.copy(), 0))
        return out

    # ---------------------------------------------------------------- décision

    def note(self, cible: Cible, position, distance: float, avis: Avis | None = None) -> float:
        """Calcule la note d'une cible : utilité − 1 point par mètre − pénalités (réservée par un autre,
        coéquipier à moins de 4 m) + λ × bonus si elle est dans la zone (et du côté) conseillés."""
        n = cible.utilite - COUT_METRE * distance
        if self.carte.reserve_par_un_autre(cible.position, self.drone, RAYON_RESERVE):
            n -= PENALITE_RESERVE
        for d, c in self.carte.coequipiers.items():
            if d != self.drone and np.linalg.norm(c.position[:2] - cible.position[:2]) < RAYON_VOISIN:
                n -= PENALITE_VOISIN
        if avis is not None and self.lam > 0:
            if np.linalg.norm(cible.origine[:2] - avis.centre[:2]) <= avis.rayon:
                n += self.lam * BONUS_ZONE
                if avis.cote is not None and cible.cote == avis.cote and cible.genre != "explorer":
                    n += self.lam * BONUS_COTE
        return float(n)

    def choisit(self, position, cibles: list[Cible] | None = None,
                avis: Avis | None = None, t: float | None = None) -> Cible | None:
        """Renvoie la meilleure cible atteignable depuis `position`, avec son chemin, ou None : tri à vol
        d'oiseau, puis vrai chemin calculé pour les 6 premières (sans chemin, une cible est écartée)."""
        p = np.asarray(position, dtype=float)
        if cibles is None:
            cibles = self.candidats(t)
        self._couts = {}
        # une piste déjà visitée 3 fois sans lecture n'est plus proposée
        vivants = [c for c in cibles if self._visites(c.origine) < 3]
        for c in vivants:
            c.distance = float(np.linalg.norm(c.position - p))
            c.note = self.note(c, p, c.distance, avis)
        vivants.sort(key=lambda c: -c.note)
        meilleur = None
        for c in vivants[:CANDIDATS_VERIFIES]:
            points = self.carte.chemin(p, c.position, altitude=float(c.position[2]))
            if points is None:
                c.note = -math.inf
                continue
            trajet = [p] + points + [c.position]
            c.distance = float(sum(np.linalg.norm(b - a) for a, b in zip(trajet[:-1], trajet[1:])))
            c.note = self.note(c, p, c.distance, avis)
            c.points = points
            if meilleur is None or c.note > meilleur.note:
                meilleur = c
        return meilleur

    def _visites(self, origine) -> int:
        """Renvoie le nombre de visites sans lecture de la piste proche de `origine` (à moins de 60 cm), 0 sinon."""
        for pos, n in self.tentatives:
            if np.linalg.norm(pos - origine) < mapping.FUSION + 0.15:
                return n
        return 0

    def constate(self, cible: Cible, lu: bool) -> None:
        """Enregistre une visite : une piste visitée sans lecture gagne une tentative ; à la 3e, elle est
        écartée pour tous. Comptage par proximité : une piste bouge de quelques cm à chaque vue."""
        if cible.genre != "lire" or lu:
            return
        for entree in self.tentatives:
            if np.linalg.norm(entree[0] - cible.origine) < mapping.FUSION + 0.15:
                entree[1] += 1
                if entree[1] >= 3:
                    self.carte.ecarte(cible.origine, duree=1e9)      # pour de bon
                return
        self.tentatives.append([np.asarray(cible.origine, dtype=float).copy(), 1])
