"""Carte partagée par les drones : la mémoire commune de l'essaim (étape 4).

Grille 3D de cubes de 25 cm : occupation (lidar, le laser qui mesure les distances), couverture (côtés d'où une case
a été vue d'assez près pour y lire un QR), cartons repérés ; plus panneaux lus, pistes, réservations qui expirent et
chemins (A*). Utilisée par observation.py (remplissage), planning.py (décision) et mission.py."""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .env.config import CAMERAS, MAP

# Occupation : chaque case cumule des « preuves » (vide en négatif, occupé en positif) au lieu d'être écrasée ;
# une case vue libre dix fois puis touchée une fois reste libre.
L_OCCUPE = 0.85         # preuve ajoutée là où un rayon s'arrête : deux impacts rendent occupée une case inconnue
L_LIBRE = -0.40         # preuve de vide, par point de rayon (tous les 8 cm) qui traverse la case
L_PLAFOND = 5.0         # la preuve reste entre -5 et +5 : une case peut toujours changer d'état
SEUIL_OCCUPE = 1.0      # au-dessus : case occupée
SEUIL_LIBRE = -1.0      # en dessous : case libre ; entre les deux seuils : inconnue

PORTEE_CARTE = 8.0      # m, portée maximale des rayons lidar versés dans la carte

LIRE_MIN = 1.5          # m, distance apparente minimale de lecture fiable d'un QR (mesurée à l'étape 2)
LIRE_MAX = 4.0          # m, distance apparente maximale (apparente = distance réelle / cosinus de l'angle de vue)

RAYON_DRONE = 0.6       # m, marge de sécurité ajoutée autour des obstacles pour les chemins
EPAISSEUR = 0.85        # m au-dessus de l'altitude de vol où un obstacle bloque le passage (rayon + oscillation)
DESSOUS = 2.0           # m sous l'altitude de vol où un obstacle bloque aussi : un rack se contourne, ne se survole pas
FUSION = 0.45           # m : deux observations plus proches sont le même objet (deux cartons voisins sont à 50 cm)
COUT_INCONNU = 20.0     # coût d'une case inconnue, 20 fois celui d'une case libre : un chemin connu est préféré
RESERVATION_S = 45.0    # s, durée de vie d'une réservation de cible
SILENCE_S = 5.0         # s sans nouvelles d'un drone : ses réservations sont libérées (cas de la panne)
LISTE_NOIRE_S = 120.0   # s pendant lesquelles une cible abandonnée est écartée
SEM_CARTON = 1          # valeur du canal sémantique pour « carton repéré par le détecteur »

INCONNU, LIBRE, OCCUPE = 0, 1, 2    # les trois états d'une case (voir Carte.etat)


@dataclass
class Panneau:
    """Un QR lu : son code, sa position, sa normale (tournée vers la caméra), sa dernière lecture et leur nombre."""
    code: str               # par exemple « BOX_007 »
    position: np.ndarray    # m, moyenne des lectures
    normale: np.ndarray
    vu_le: float            # s, temps simulé de la dernière lecture
    lectures: int = 1


@dataclass
class Piste:
    """Un QR repéré de loin mais pas encore lu : il deviendra une cible « lire » pour les drones."""
    position: np.ndarray    # m, moyenne des repérages
    normale: np.ndarray | None     # direction d'où il a été aperçu ; None si inconnue
    vu_le: float            # s, temps simulé du dernier repérage
    vues: int = 1


@dataclass
class Reservation:
    """La cible qu'un drone a réservée, valable jusqu'à l'instant `jusqu_a` (s, temps simulé)."""
    drone: int
    cible: np.ndarray
    jusqu_a: float


@dataclass
class Coequipier:
    """La dernière position annoncée par un drone, et quand (s, temps simulé)."""
    position: np.ndarray
    vu_le: float


class Carte:
    """La carte partagée : grille 3D (occupation, couverture, sémantique), panneaux, pistes et réservations."""

    def __init__(self, grille=MAP):
        """Crée une carte vide sur `grille` (par défaut 84 × 128 × 24 cubes de 25 cm, soit 1,5 Mo)."""
        self.g = grille
        self.forme = tuple(grille.shape)
        self.origine = np.array([grille.x_min, grille.y_min, grille.z_min], dtype=float)
        self.occupation = np.zeros(self.forme, dtype=np.float32)    # preuve cumulée par cube
        self.couverture = np.zeros(self.forme, dtype=np.uint8)      # bit k : vu de près par une caméra tournée vers k
        self.semantique = np.zeros(self.forme, dtype=np.uint8)      # SEM_CARTON là où un carton a été repéré
        self.obstacles_mobiles: list = []       # (position, rayon) des coéquipiers, le temps d'un calcul de chemin
        self.panneaux: list[Panneau] = []
        self.pistes: list[Piste] = []
        self.reservations: dict[int, Reservation] = {}
        self.coequipiers: dict[int, Coequipier] = {}
        self.liste_noire: list[tuple[np.ndarray, float]] = []      # (cible écartée, fin de l'exclusion en s)
        self.t = 0.0                            # s, temps simulé de la dernière mise à jour

    def indice(self, points) -> np.ndarray:
        """Renvoie les indices (i, j, k) du cube qui contient chaque point ; hors grille ou NaN : -1 ou la taille."""
        p = np.atleast_2d(np.asarray(points, dtype=float))
        brut = np.floor((p - self.origine) / self.g.cell)
        brut = np.nan_to_num(brut, nan=-1.0, posinf=1e6, neginf=-1.0)
        return np.clip(brut, -1.0, np.array(self.forme, dtype=float)).astype(np.int32)

    def centre(self, idx) -> np.ndarray:
        """Renvoie les coordonnées (m) du centre des cubes d'indices `idx`."""
        i = np.atleast_2d(np.asarray(idx, dtype=float))
        return self.origine + (i + 0.5) * self.g.cell

    def dedans(self, idx) -> np.ndarray:
        """Renvoie vrai pour chaque indice qui tombe dans la grille."""
        i = np.atleast_2d(np.asarray(idx))
        return np.all((i >= 0) & (i < np.array(self.forme)), axis=1)

    def sur_la_carte(self, points) -> np.ndarray:
        """Renvoie vrai pour chaque point fini (ni NaN ni infini) situé dans la grille."""
        p = np.atleast_2d(np.asarray(points, dtype=float))
        fini = np.isfinite(p).all(axis=1)
        return fini & self.dedans(self.indice(np.where(fini[:, None], p, 0.0)))

    def etat(self, points) -> np.ndarray:
        """Renvoie l'état de la case de chaque point : INCONNU (0), LIBRE (1) ou OCCUPE (2) ; inconnu hors grille."""
        idx = self.indice(points)
        ok = self.dedans(idx)
        out = np.full(len(idx), INCONNU, dtype=np.int8)
        if ok.any():
            v = self.occupation[tuple(idx[ok].T)]
            e = np.full(len(v), INCONNU, dtype=np.int8)
            e[v > SEUIL_OCCUPE] = OCCUPE
            e[v < SEUIL_LIBRE] = LIBRE
            out[ok] = e
        return out

    def integre_lidar(self, origine, directions, portees, t: float | None = None) -> None:
        """Verse un tour de lidar dans la carte : les cases traversées par un rayon gagnent une preuve de vide,
        la case où il s'arrête une preuve d'occupation (directions unitaires en repère monde, portées en m)."""
        if t is not None:
            self.t = t
        o = np.asarray(origine, dtype=float)
        d = np.asarray(directions, dtype=float)
        d = d / np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
        r = np.asarray(portees, dtype=float)

        touche = np.isfinite(r) & (r > 0.0) & (r <= PORTEE_CARTE)
        vide = np.clip(np.where(np.isfinite(r), r, PORTEE_CARTE), 0.0, PORTEE_CARTE)
        # un point tous les 8 cm le long de chaque rayon, jusqu'à une case avant l'impact (ou jusqu'à 8 m)
        pas = self.g.cell / 3.0
        ts = np.arange(1, int(PORTEE_CARTE / pas) + 1, dtype=float) * pas
        libres = ts[None, :] < (vide[:, None] - self.g.cell)
        if libres.any():
            pts = o + d[:, None, :] * ts[None, :, None]
            self._ajoute(pts[libres], L_LIBRE)
        if touche.any():
            self._ajoute(o + d[touche] * r[touche, None], L_OCCUPE)

    def _ajoute(self, points, valeur: float) -> None:
        """Ajoute `valeur` à la preuve des cases contenant les points (hors grille ignorés), bornée à ±5."""
        idx = self.indice(points)
        idx = idx[self.dedans(idx)]
        if not len(idx):
            return
        np.add.at(self.occupation, tuple(idx.T), valeur)
        np.clip(self.occupation, -L_PLAFOND, L_PLAFOND, out=self.occupation)

    @staticmethod
    def cardinal(direction) -> int:
        """Renvoie le côté cardinal le plus proche d'une direction horizontale : 0 +x, 1 -x, 2 +y, 3 -y."""
        v = np.asarray(direction, dtype=float)
        if abs(v[0]) >= abs(v[1]):
            return 0 if v[0] > 0 else 1
        return 2 if v[1] > 0 else 3

    def couvert_pour(self, points, normale) -> np.ndarray:
        """Dit, pour chaque point, si un QR tourné selon `normale` aurait pu y être lu (case déjà vue de face et d'assez près)."""
        bit = 1 << self.cardinal(-np.asarray(normale, dtype=float))     # la caméra regarde à l'opposé de la normale
        idx = self.indice(points)
        ok = self.dedans(idx)
        out = np.zeros(len(idx), dtype=bool)
        if ok.any():
            out[ok] = (self.couverture[tuple(idx[ok].T)] & bit) > 0
        return out

    def integre_couverture(self, position, avant, t: float | None = None,
                           pas_deg: float = 3.0) -> int:
        """Marque, pour le côté regardé, les cases où un QR tourné vers la caméra serait lisible (distance apparente
        de 1,5 à 4 m, rien d'occupé devant) ; `avant` = axe de la caméra. Renvoie le nombre de cases nouvelles."""
        if t is not None:
            self.t = t
        o = np.asarray(position, dtype=float)
        axe = np.asarray(avant, dtype=float)
        axe = axe / max(np.linalg.norm(axe), 1e-9)
        bit = np.uint8(1 << self.cardinal(axe))
        # un rayon tous les 3 degrés dans le champ de la caméra, un point tous les 12,5 cm jusqu'à 4 m
        dirs = _cone(axe, CAMERAS.fov_deg, CAMERAS.fov_deg * CAMERAS.side_height / CAMERAS.side_width,
                     pas_deg)
        cosinus = np.clip(dirs @ axe, 1e-3, 1.0)
        pas = self.g.cell * 0.5
        ts = np.arange(pas, LIRE_MAX + pas, pas)
        apparente = ts[None, :] / cosinus[:, None]      # un QR face à la caméra, vu de biais, paraît plus loin
        pts = o + dirs[:, None, :] * ts[None, :, None]
        idx = self.indice(pts.reshape(-1, 3)).reshape(len(dirs), len(ts), 3)
        dedans = self.dedans(idx.reshape(-1, 3)).reshape(len(dirs), len(ts))
        plein = np.zeros(dedans.shape, dtype=bool)
        plein[dedans] = self.occupation[tuple(idx[dedans].T)] > SEUIL_OCCUPE
        cache = np.cumsum(plein, axis=1) > 0            # tout ce qui suit la première case occupée est caché
        bon = dedans & ~cache & (apparente >= LIRE_MIN) & (apparente <= LIRE_MAX)
        if not bon.any():
            return 0
        cases = tuple(idx[bon].T)
        nouvelles = int(np.count_nonzero((self.couverture[cases] & bit) == 0))
        self.couverture[cases] |= bit
        return nouvelles

    def premier_obstacle(self, origine, direction, portee: float = PORTEE_CARTE,
                         pas: float = 0.1) -> float | None:
        """Renvoie la distance (m) du premier cube occupé connu le long d'un rayon, cherché tous les 10 cm
        jusqu'à `portee` ; None s'il n'y en a pas."""
        o = np.asarray(origine, dtype=float)
        d = np.asarray(direction, dtype=float)
        d = d / max(np.linalg.norm(d), 1e-9)
        ts = np.arange(pas, portee + pas / 2, pas)
        pts = o + ts[:, None] * d
        idx = self.indice(pts)
        ok = self.dedans(idx)
        occ = np.zeros(len(ts), dtype=bool)
        occ[ok] = self.occupation[tuple(idx[ok].T)] > SEUIL_OCCUPE
        k = np.flatnonzero(occ)
        return float(ts[k[0]]) if len(k) else None

    def marque(self, points, valeur: int = SEM_CARTON) -> int:
        """Marque « carton » (ou `valeur`) dans le canal sémantique, aux cubes des points ;
        renvoie le nombre de points tombés dans la grille."""
        idx = self.indice(points)
        ok = self.dedans(idx)
        if ok.any():
            self.semantique[tuple(idx[ok].T)] = valeur
        return int(ok.sum())

    @property
    def codes(self) -> set[str]:
        """Renvoie l'ensemble des codes lus (cartons dont on connaît le contenu) : c'est la mesure de la mission."""
        return {p.code for p in self.panneaux}

    def faces(self, code: str) -> list[Panneau]:
        """Renvoie les panneaux lus portant ce code (au plus deux : un carton porte le même code sur deux faces)."""
        return [p for p in self.panneaux if p.code == code]

    def integre_lecture(self, code: str, position, normale, t: float | None = None):
        """Ajoute un QR décodé : fusionné avec la face connue de même code à moins de 45 cm, sinon nouvelle face
        (deux au plus) ; efface les pistes voisines. Renvoie le panneau, ou None si la lecture est refusée."""
        if t is not None:
            self.t = t
        p = np.asarray(position, dtype=float)
        if not self.sur_la_carte(p)[0]:
            return None
        n = np.asarray(normale, dtype=float)
        memes = self.faces(code)
        proche = min(memes, key=lambda q: np.linalg.norm(q.position - p), default=None)
        if proche is None or np.linalg.norm(proche.position - p) > FUSION:
            if len(memes) >= 2:
                return None         # une 3e face loin des deux connues : erreur de décodage, refusée
            proche = Panneau(code, p, n, self.t)
            self.panneaux.append(proche)
        else:
            # même face : la position devient la moyenne de toutes les lectures
            k = proche.lectures
            proche.position = (proche.position * k + p) / (k + 1)
            proche.normale = n
            proche.lectures = k + 1
            proche.vu_le = self.t
        self.pistes = [q for q in self.pistes if np.linalg.norm(q.position - p) > FUSION]
        return proche

    def integre_reperage(self, position, normale=None, t: float | None = None):
        """Ajoute un QR repéré mais pas lu : ignoré près d'un panneau déjà lu, fusionné avec la piste à moins de 45 cm,
        sinon nouvelle piste. Renvoie la piste, ou None si le repérage est ignoré."""
        if t is not None:
            self.t = t
        p = np.asarray(position, dtype=float)
        if not self.sur_la_carte(p)[0]:
            return None
        for panneau in self.panneaux:
            if np.linalg.norm(panneau.position - p) <= FUSION:
                return None
        for piste in self.pistes:
            if np.linalg.norm(piste.position - p) <= FUSION:
                k = piste.vues
                piste.position = (piste.position * k + p) / (k + 1)
                piste.vues = k + 1
                piste.vu_le = self.t
                if normale is not None:
                    n = np.asarray(normale, dtype=float)
                    if piste.normale is not None:
                        n = piste.normale * k + n
                    piste.normale = n / max(np.linalg.norm(n), 1e-9)
                return piste
        piste = Piste(p, None if normale is None else np.asarray(normale, dtype=float), self.t)
        self.pistes.append(piste)
        return piste

    def oublie_pistes(self, point, rayon: float) -> int:
        """Efface les pistes à moins de `rayon` (m) d'un point (après une lecture réussie) ; renvoie leur nombre."""
        p = np.asarray(point, dtype=float)
        avant = len(self.pistes)
        self.pistes = [q for q in self.pistes if np.linalg.norm(q.position - p) > rayon]
        return avant - len(self.pistes)

    def annonce(self, drone: int, position, t: float | None = None) -> None:
        """Note la position d'un drone et l'heure : c'est son signe de vie sur la carte partagée."""
        if t is not None:
            self.t = t
        self.coequipiers[drone] = Coequipier(np.asarray(position, dtype=float), self.t)

    def reserve(self, drone: int, cible, duree: float = RESERVATION_S) -> None:
        """Réserve une cible pour ce drone pendant `duree` (45 s par défaut), à la place de sa réservation précédente."""
        self.reservations[drone] = Reservation(drone, np.asarray(cible, dtype=float),
                                               self.t + duree)

    def libere(self, drone: int) -> None:
        """Supprime la réservation de ce drone, s'il en a une."""
        self.reservations.pop(drone, None)

    def reserve_par_un_autre(self, point, drone: int, rayon: float = 2.0) -> bool:
        """Renvoie vrai si un autre drone a réservé une cible à moins de `rayon` (m) de ce point."""
        p = np.asarray(point, dtype=float)
        return any(r.drone != drone and np.linalg.norm(r.cible - p) < rayon
                   for r in self.reservations.values())

    def ecarte(self, cible, duree: float = LISTE_NOIRE_S) -> None:
        """Met une cible en liste noire pendant `duree` (120 s par défaut) : aucun drone ne la choisit plus."""
        self.liste_noire.append((np.asarray(cible, dtype=float), self.t + duree))

    def est_ecartee(self, point, rayon: float = 0.6) -> bool:
        """Renvoie vrai si le point est à moins de `rayon` (60 cm par défaut) d'une cible en liste noire."""
        p = np.asarray(point, dtype=float)
        return any(np.linalg.norm(c - p) < rayon for c, _ in self.liste_noire)

    def vieillit(self, t: float) -> list[int]:
        """Avance l'horloge : retire réservations et exclusions expirées, et les réservations des drones muets
        depuis plus de 5 s (une panne se gère ainsi toute seule) ; renvoie la liste de ces drones muets."""
        self.t = t
        self.reservations = {d: r for d, r in self.reservations.items() if r.jusqu_a > t}
        self.liste_noire = [(c, e) for c, e in self.liste_noire if e > t]
        muets = [d for d, c in self.coequipiers.items() if t - c.vu_le > SILENCE_S]
        for d in muets:
            self.reservations.pop(d, None)
        return muets

    def frontieres(self, z_min: float, z_max: float) -> np.ndarray:
        """Renvoie les colonnes libres qui touchent l'inconnu dans une tranche d'altitude (m), en points (x, y, z au
        milieu de la tranche) : ce sont les cibles « explorer »."""
        k0, k1 = self._tranche(z_min, z_max)
        occ = self.occupation[:, :, k0:k1]
        libre = (occ < SEUIL_LIBRE).any(axis=2) & ~(occ > SEUIL_OCCUPE).any(axis=2)
        inconnu = ~((occ < SEUIL_LIBRE) | (occ > SEUIL_OCCUPE)).any(axis=2)

        def voisins(m):
            """Compte, pour chaque case, combien de ses 4 voisines (en ±x et ±y) sont vraies dans `m`."""
            v = np.zeros(m.shape, dtype=np.int8)
            v[1:, :] += m[:-1, :]
            v[:-1, :] += m[1:, :]
            v[:, 1:] += m[:, :-1]
            v[:, :-1] += m[:, 1:]
            return v

        region = inconnu & (voisins(inconnu.astype(np.int8)) >= 2)     # un trou inconnu isolé ne compte pas
        ij = np.argwhere(libre & (voisins(region.astype(np.int8)) > 0))
        if not len(ij):
            return np.zeros((0, 3))
        z = 0.5 * (z_min + z_max)
        return np.column_stack([self.centre(np.column_stack([ij, np.zeros(len(ij))]))[:, :2],
                                np.full(len(ij), z)])

    def _tranche(self, z_min: float, z_max: float) -> tuple[int, int]:
        """Convertit une tranche d'altitude (m) en indices de couches [k0, k1) de la grille (au moins une couche)."""
        k0 = max(0, int((z_min - self.g.z_min) / self.g.cell))
        k1 = min(self.forme[2], int((z_max - self.g.z_min) / self.g.cell) + 1)
        return k0, max(k1, k0 + 1)

    def couts(self, z_min: float, z_max: float, marge: float = RAYON_DRONE) -> np.ndarray:
        """Renvoie la carte de coût vue de dessus d'une tranche d'altitude : 1 libre, 20 inconnu, infini sur un obstacle
        élargi de `marge` (m) ou sur un coéquipier ; une colonne compte comme obstacle dès qu'un de ses cubes l'est."""
        k0, k1 = self._tranche(z_min, z_max)
        occ = self.occupation[:, :, k0:k1]
        dur = (occ > SEUIL_OCCUPE).any(axis=2)
        libre = (occ < SEUIL_LIBRE).any(axis=2)
        # obstacles élargis d'un disque de rayon `marge`
        r = marge / self.g.cell
        n = int(math.ceil(r))
        nx, ny = dur.shape
        gros = dur.copy()
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy > r * r or (dx == 0 and dy == 0):
                    continue
                sx, sy = slice(max(dx, 0), nx + min(dx, 0)), slice(max(dy, 0), ny + min(dy, 0))
                tx, ty = slice(max(-dx, 0), nx - max(dx, 0)), slice(max(-dy, 0), ny - max(dy, 0))
                gros[sx, sy] |= dur[tx, ty]
        cout = np.full(dur.shape, COUT_INCONNU, dtype=np.float32)
        cout[libre] = 1.0
        cout[gros] = np.inf
        # chaque coéquipier bloque un disque de son rayon (2 m en mission)
        for position, rayon in self.obstacles_mobiles:
            i, j, _ = self.indice(position)[0]
            n = int(math.ceil(rayon / self.g.cell))
            i0, i1 = max(i - n, 0), min(i + n + 1, nx)
            j0, j1 = max(j - n, 0), min(j + n + 1, ny)
            if i0 < i1 and j0 < j1:
                ii, jj = np.mgrid[i0:i1, j0:j1]
                cout[i0:i1, j0:j1][((ii - i) ** 2 + (jj - j) ** 2) * self.g.cell ** 2 <= rayon ** 2] = np.inf
        return cout

    def couts_de_vol(self, z: float) -> np.ndarray:
        """Renvoie la carte de coût pour un vol à l'altitude `z` (tranche de z - 2 m à z + 0,85 m)."""
        return self.couts(z - DESSOUS, z + EPAISSEUR)

    def chemin(self, depart, arrivee, altitude: float | None = None,
               epaisseur: float = EPAISSEUR) -> list[np.ndarray] | None:
        """Calcule les points de passage de `depart` à `arrivee` à altitude constante (A* sur la carte de coût) :
        [] si la ligne droite ne traverse que du libre connu, None s'il n'existe aucun chemin."""
        a = np.asarray(depart, dtype=float)
        b = np.asarray(arrivee, dtype=float)
        z = float(a[2] if altitude is None else altitude)
        cout = self.couts(z - DESSOUS, z + epaisseur)
        ia, ib = tuple(self.indice(a)[0][:2]), tuple(self.indice(b)[0][:2])
        if not self._praticable(ib, cout):
            return None
        if not self._praticable(ia, cout):
            # départ dans la marge de sécurité : permis d'en sortir, sauf s'il est dans l'obstacle lui-même
            k0, k1 = self._tranche(z - DESSOUS, z + epaisseur)
            if (self.occupation[ia[0], ia[1], k0:k1] > SEUIL_OCCUPE).any():
                return None
            cout = cout.copy()
            cout[ia] = COUT_INCONNU
        if self._droite_ok(ia, ib, cout, 1.0):
            return []
        brut = _astar(cout, ia, ib)
        if brut is None:
            return None
        lisse = self._elague(brut, cout)
        return [np.array([*self.centre([i, j, 0])[0][:2], z]) for i, j in lisse[1:-1]]

    def segment_libre(self, a, b, altitude: float | None = None, epaisseur: float = EPAISSEUR) -> bool:
        """Renvoie faux dès qu'un obstacle connu (élargi) ou un coéquipier coupe ce segment : sert à revérifier
        en vol un chemin déjà planifié (l'inconnu, lui, ne coupe pas)."""
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        z = float(a[2] if altitude is None else altitude)
        cout = self.couts(z - DESSOUS, z + epaisseur)
        ia, ib = tuple(self.indice(a)[0][:2]), tuple(self.indice(b)[0][:2])
        return np.isfinite(self._cout_droite(ia, ib, cout))

    def _praticable(self, ij, cout) -> bool:
        """Renvoie vrai si la colonne (i, j) est dans la grille et de coût fini (ni obstacle, ni marge)."""
        i, j = int(ij[0]), int(ij[1])
        return 0 <= i < cout.shape[0] and 0 <= j < cout.shape[1] and np.isfinite(cout[i, j])

    @staticmethod
    def _cout_droite(a, b, cout) -> float:
        """Renvoie le coût d'un segment droit : longueur × coût moyen des cases traversées (infini sur un obstacle)."""
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
        longueur = math.hypot(b[0] - a[0], b[1] - a[1])
        total = 0.0
        for k in range(n + 1):
            i = int(round(a[0] + (b[0] - a[0]) * k / n))
            j = int(round(a[1] + (b[1] - a[1]) * k / n))
            c = float(cout[i, j])
            if not np.isfinite(c):
                return math.inf
            total += c
        return total * longueur / (n + 1)

    def _droite_ok(self, a, b, cout, cout_max: float) -> bool:
        """Renvoie vrai si toutes les cases traversées par le segment droit de a à b coûtent au plus `cout_max`."""
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
        for k in range(n + 1):
            i = int(round(a[0] + (b[0] - a[0]) * k / n))
            j = int(round(a[1] + (b[1] - a[1]) * k / n))
            if cout[i, j] > cout_max:
                return False
        return True

    def _elague(self, cases, cout) -> list:
        """Simplifie un chemin A* en ne gardant que les coins : on saute en ligne droite tant qu'elle ne coûte pas
        plus de 2 % de plus que le chemin qu'elle remplace."""
        cumul = [0.0]
        for a, b in zip(cases[:-1], cases[1:]):
            cumul.append(cumul[-1] + math.hypot(b[0] - a[0], b[1] - a[1]) * float(cout[b]))
        out = [cases[0]]
        i = 0
        while i < len(cases) - 1:
            j = len(cases) - 1
            while j > i + 1 and self._cout_droite(cases[i], cases[j], cout) > (cumul[j] - cumul[i]) * 1.02 + 1e-6:
                j -= 1
            out.append(cases[j])
            i = j
        return out

    def sauve(self, souche) -> None:
        """Enregistre la carte : grilles dans `souche`.npz (compressé), panneaux et pistes dans `souche`.json."""
        souche = Path(souche)
        np.savez_compressed(souche.with_suffix(".npz"), occupation=self.occupation,
                            couverture=self.couverture, semantique=self.semantique)
        souche.with_suffix(".json").write_text(json.dumps({
            "t": self.t,
            "panneaux": [{"code": p.code, "position": p.position.tolist(),
                          "normale": p.normale.tolist(), "vu_le": p.vu_le,
                          "lectures": p.lectures} for p in self.panneaux],
            "pistes": [{"position": q.position.tolist(),
                        "normale": None if q.normale is None else q.normale.tolist(),
                        "vu_le": q.vu_le, "vues": q.vues} for q in self.pistes],
        }))

    @classmethod
    def charge(cls, souche) -> "Carte":
        """Recharge une carte écrite par `sauve` (grilles, panneaux, pistes), sans réservations ni coéquipiers."""
        souche = Path(souche)
        c = cls()
        z = np.load(souche.with_suffix(".npz"))
        c.occupation, c.couverture = z["occupation"], z["couverture"]
        if "semantique" in z:
            c.semantique = z["semantique"]
        d = json.loads(souche.with_suffix(".json").read_text())
        c.t = d["t"]
        c.panneaux = [Panneau(p["code"], np.array(p["position"]), np.array(p["normale"]),
                              p["vu_le"], p["lectures"]) for p in d["panneaux"]]
        c.pistes = [Piste(np.array(q["position"]),
                          None if q["normale"] is None else np.array(q["normale"]),
                          q["vu_le"], q["vues"]) for q in d["pistes"]]
        return c

    def resume(self) -> dict:
        """Renvoie les chiffres clés : cases occupées, libres, inconnues, couvertes, part connue, codes, pistes, mémoire."""
        occ = self.occupation > SEUIL_OCCUPE
        libre = self.occupation < SEUIL_LIBRE
        total = int(np.prod(self.forme))
        return {
            "cases": total,
            "occupees": int(occ.sum()),
            "libres": int(libre.sum()),
            "inconnues": total - int(occ.sum()) - int(libre.sum()),
            "couvertes": int(np.count_nonzero(self.couverture)),
            "part_connue": round(float((occ | libre).mean()), 4),
            "codes_lus": len(self.codes),
            "faces_lues": len(self.panneaux),
            "pistes": len(self.pistes),
            "reservations": len(self.reservations),
            "octets": int(self.occupation.nbytes + self.couverture.nbytes + self.semantique.nbytes),
        }


def _cone(axe, fov_h_deg: float, fov_v_deg: float, pas_deg: float) -> np.ndarray:
    """Renvoie des directions unitaires réparties tous les `pas_deg` degrés dans le champ d'une caméra d'axe `axe`."""
    a = np.asarray(axe, dtype=float)
    a = a / max(np.linalg.norm(a), 1e-9)
    haut = np.array([0.0, 0.0, 1.0])
    droite = np.cross(a, haut)
    droite /= max(np.linalg.norm(droite), 1e-9)
    haut = np.cross(droite, a)
    hs = np.radians(np.arange(-fov_h_deg / 2, fov_h_deg / 2 + 1e-6, pas_deg))
    vs = np.radians(np.arange(-fov_v_deg / 2, fov_v_deg / 2 + 1e-6, pas_deg))
    H, V = np.meshgrid(hs, vs, indexing="ij")
    d = a[None, None, :] + np.tan(H)[..., None] * droite + np.tan(V)[..., None] * haut
    d = d.reshape(-1, 3)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


# les 8 voisines d'une case et la longueur du pas : 1 tout droit, 1,414 (racine de 2) en diagonale
_VOISINS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]


def _astar(cout: np.ndarray, depart: tuple, arrivee: tuple) -> list | None:
    """Cherche le chemin le moins coûteux entre deux colonnes (algorithme A*, 8 voisines) ; renvoie ses cases ou None."""
    nx, ny = cout.shape
    h = lambda c: math.hypot(c[0] - arrivee[0], c[1] - arrivee[1])     # estimation : distance à vol d'oiseau
    ouvert = [(h(depart), 0.0, depart)]
    venu = {depart: None}
    meilleur = {depart: 0.0}
    while ouvert:
        _, g, c = heapq.heappop(ouvert)
        if c == arrivee:
            chemin = []
            while c is not None:
                chemin.append(c)
                c = venu[c]
            return chemin[::-1]
        if g > meilleur.get(c, math.inf):
            continue
        for dx, dy, pas in _VOISINS:
            v = (c[0] + dx, c[1] + dy)
            if not (0 <= v[0] < nx and 0 <= v[1] < ny) or not np.isfinite(cout[v]):
                continue
            g2 = g + pas * float(cout[v])
            if g2 < meilleur.get(v, math.inf):
                meilleur[v] = g2
                venu[v] = c
                heapq.heappush(ouvert, (g2 + h(v), g2, v))
    return None


# couleurs de la vue de dessus, dans l'ordre (bleu, vert, rouge) d'OpenCV
COULEURS = {
    "inconnu": (110, 110, 110),
    "libre": (185, 185, 185),
    "couvert": (245, 245, 245),
    "occupe": (60, 60, 60),
}
COULEURS_DRONES = [(255, 60, 0), (0, 160, 255), (200, 0, 200)]     # drone 0 bleu, 1 orange, 2 violet


def vue_de_dessus(carte: Carte, echelle: int = 6, z_min: float = 0.6, z_max: float = 5.5,
                  trajectoire=None) -> np.ndarray:
    """Dessine la carte vue de dessus (obstacles gris foncé, libre gris clair, couvert blanc, inconnu gris moyen),
    avec pistes (orange), panneaux lus (vert), drones et leur cible ; renvoie l'image (`echelle` pixels par case)."""
    import cv2

    k0, k1 = carte._tranche(z_min, z_max)
    tranche = carte.occupation[:, :, k0:k1]
    occ = (tranche > SEUIL_OCCUPE).any(axis=2)
    libre = (tranche < SEUIL_LIBRE).any(axis=2)
    couvert = (carte.couverture[:, :, k0:k1] > 0).any(axis=2)

    nx, ny = occ.shape
    img = np.full((ny, nx, 3), COULEURS["inconnu"], dtype=np.uint8)
    img[libre.T] = COULEURS["libre"]
    img[couvert.T] = COULEURS["couvert"]
    img[occ.T] = COULEURS["occupe"]
    img = cv2.resize(img, (nx * echelle, ny * echelle), interpolation=cv2.INTER_NEAREST)
    img = cv2.flip(img, 0)          # le nord (+y) en haut de l'image

    def px(p):
        """Convertit un point du monde en pixel (colonne, ligne) de l'image ; None hors de la carte."""
        if not carte.sur_la_carte(p)[0]:
            return None
        i, j = carte.indice(p)[0][:2]
        return int((i + 0.5) * echelle), int((ny - j - 0.5) * echelle)

    if trajectoire is not None and len(trajectoire) > 1:
        pts = [q for q in (px(p) for p in trajectoire) if q is not None]
        for a, b in zip(pts[:-1], pts[1:]):
            cv2.line(img, a, b, (255, 130, 40), 1, cv2.LINE_AA)
    for piste in carte.pistes:
        if (q := px(piste.position)) is not None:
            cv2.circle(img, q, max(2, echelle // 2), (0, 140, 255), -1)
    for panneau in carte.panneaux:
        if (q := px(panneau.position)) is not None:
            cv2.circle(img, q, max(2, echelle // 2), (0, 170, 0), -1)
    for drone, c in carte.coequipiers.items():
        p = px(c.position)
        if p is None:
            continue
        couleur = COULEURS_DRONES[drone % len(COULEURS_DRONES)]
        r = carte.reservations.get(drone)
        if r is not None and (q := px(r.cible)) is not None:
            cv2.line(img, p, q, couleur, 1, cv2.LINE_AA)
        cv2.circle(img, p, echelle, couleur, -1)
        cv2.putText(img, str(drone), (p[0] + echelle, p[1] - echelle),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, couleur, 1, cv2.LINE_AA)
    return img
