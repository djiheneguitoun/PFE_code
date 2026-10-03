"""Méthode de référence de Pore, Patle et Thorat (Symmetry 2026, 18(4):548), réécrite sur notre simulateur pour comparer.

Contient le plan en zigzag figé avant le vol, le champ de risque (équation 9 de l'article), le routage dans les allées,
la priorité aux croisements, le décodage « au sol » avec OpenCV et le serveur d'inventaire.
N'emprunte rien à notre système (vérifié par tests/test_pore.py) ; utilisé par pore_mission.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .env.config import INTERIOR, RACKS

R_SAFE = 0.5            # m : distance de sécurité r_safe de l'équation 9 (le risque y vaut 0,5)
BETA = 10.0             # 1/m : raideur β de la sigmoïde du risque
TAU = 0.4               # seuil de risque : un point est sûr si R_f ≤ 0,4, soit à plus de 0,54 m de tout obstacle
SEPARATION_MIN = 1.2    # m : deux drones plus proches que cela sont en conflit
CONFLIT_S = 3.0         # s : un conflit prévu dans moins de 3 s fait céder le passage au suiveur
DEDUP_S = 1.5           # s : « dedup dwell » : le même code relu plus vite que cela est un doublon
DEFLEXION_MAX = 1.2     # m : déplacement maximal d'un arrêt pour le sortir d'une zone à risque

RECUL = 1.65            # m : distance drone-face (OpenCV lit 92 % des codes entre 1,5 et 1,8 m)
RECUL_MIN = 0.9         # m : si le recul possible tombe sous cette valeur, l'allée est trop étroite et n'est pas longée
MARGE_OPPOSEE = 0.85    # m : écart minimal gardé entre le drone et la face d'en face
HAUTEUR_PANNEAU = 0.225  # m : le drone vole et vise à cette hauteur au-dessus de chaque planche, à hauteur des QR
PAS_ARRET = 1.5         # m : espacement des arrêts le long d'une face
BORD = 0.3              # m : pas d'arrêt à moins de 30 cm des bouts d'un rack
MARGE_COULOIR = 1.2     # m : les couloirs transversaux passent à 1,2 m au-delà des bouts des racks
CONFIANCE_MIN = 0.35    # confiance (0 à 1) sous laquelle une lecture est jugée peu sûre


@dataclass
class Arret:
    """Un arrêt « hover-and-scan » (vol stationnaire puis lecture) : position (m), cap (rad), face et étage visés, point visé."""
    position: np.ndarray
    cap: float
    face: str
    etage: int
    vise: np.ndarray

    def cle(self) -> tuple:
        """Renvoie l'identifiant de l'arrêt (face, étage, y arrondi au cm), utilisé pour la liste des arrêts à relire."""
        return (self.face, self.etage, round(float(self.position[1]), 2))


def allees(layout) -> list[dict]:
    """Renvoie les allées, bandes libres entre murs et racks d'ouest en est, lues sur le plan connu :
    bornes et centre en x, et faces de rack qui les bordent."""
    racks = sorted(layout.racks, key=lambda r: r.x)
    bornes = [INTERIOR.x_min] + [x for r in racks for x in r.x_bounds] + [INTERIOR.x_max]
    out = []
    for k in range(len(racks) + 1):
        gauche, droite = bornes[2 * k], bornes[2 * k + 1]
        faces = []
        if k > 0:
            faces.append((racks[k - 1], +1))
        if k < len(racks):
            faces.append((racks[k], -1))
        out.append({"k": k, "x0": gauche, "x1": droite, "centre": 0.5 * (gauche + droite), "faces": faces})
    return out


def faces_lisibles(layout) -> list[dict]:
    """Renvoie les faces de rack qu'un drone peut longer : il se tient à 1,65 m de la face (moins si l'allée est
    étroite), et l'allée est ignorée si ce recul tombe sous 0,9 m."""
    out = []
    for a in allees(layout):
        recul = min(RECUL, (a["x1"] - a["x0"]) - MARGE_OPPOSEE)
        if recul < RECUL_MIN:
            continue
        for rack, cote in a["faces"]:
            out.append({"nom": f"{rack.prim}:{'est' if cote > 0 else 'ouest'}", "allee": a["k"],
                        "x_drone": rack.x + cote * (RACKS.depth / 2.0 + recul),
                        "x_face": rack.x + cote * RACKS.depth / 2.0,
                        "y0": rack.y_bounds[0] + BORD, "y1": rack.y_bounds[1] - BORD,
                        "rack": rack.prim, "cote": cote})
    return out


def _ys(face: dict) -> np.ndarray:
    """Renvoie les y (m) des arrêts le long d'une face, tous les 1,5 m."""
    return np.arange(face["y0"], face["y1"] + 1e-6, PAS_ARRET)


def _coupe_en_deux(face: dict) -> list[dict]:
    """Coupe une face en deux moitiés (sud et nord), confiées à deux drones qui partent des bouts opposés."""
    milieu = 0.5 * (face["y0"] + face["y1"])
    return [{**face, "y1": milieu}, {**face, "y0": milieu}]


def secteurs(faces: list[dict], n_drones: int) -> list[list[dict]]:
    """Répartit les faces entre les drones avant le vol (cas III : « a sector allocator assigns each vehicle a disjoint rack »).
    Un rack par drone ; avec moins de drones on regroupe des racks voisins, avec plus on coupe le plus chargé en deux."""
    groupes: list[list[dict]] = []
    for f in faces:
        if groupes and groupes[-1][0]["rack"] == f["rack"]:
            groupes[-1].append(f)
        else:
            groupes.append([f])

    def charge(g):
        """Renvoie la charge d'un groupe de faces : son nombre d'arrêts sur un étage."""
        return sum(len(_ys(f)) for f in g)

    while len(groupes) > n_drones and len(groupes) > 1:
        k = min(range(len(groupes) - 1), key=lambda k: charge(groupes[k]) + charge(groupes[k + 1]))
        groupes[k:k + 2] = [groupes[k] + groupes[k + 1]]
    while len(groupes) < n_drones and groupes:
        k = max(range(len(groupes)), key=lambda k: charge(groupes[k]))
        lourd = groupes.pop(k)
        moities = [[], []]
        for f in lourd:
            for m, moitie in zip(moities, _coupe_en_deux(f)):
                m.append(moitie)
        groupes[k:k] = moities
    return groupes + [[] for _ in range(n_drones - len(groupes))]


def plan_zigzag(layout, n_drones: int, etages=(0, 1, 2)) -> list[list[Arret]]:
    """Construit avant le vol le plan figé de chaque drone : « a predefined zig-zag coverage path with hover-and-scan stops ».
    Un arrêt tous les 1,5 m par face et par étage, sens inversé à chaque passage et entre drones voisins ; renvoie un plan par drone."""
    plans: list[list[Arret]] = []
    for d, bloc in enumerate(secteurs(faces_lisibles(layout), n_drones)):
        arrets: list[Arret] = []
        sens = 1 if d % 2 == 0 else -1
        for f in bloc:
            for etage in etages:
                z = RACKS.shelf_levels[etage] + HAUTEUR_PANNEAU
                for y in _ys(f)[::sens]:
                    p = np.array([f["x_drone"], float(y), z])
                    vise = np.array([f["x_face"], float(y), z])
                    arrets.append(Arret(p, math.atan2(0.0, f["x_face"] - f["x_drone"]), f["nom"], etage, vise))
                sens = -sens
        plans.append(arrets)
    return plans


def attribue(plans: list[list[Arret]], departs) -> list[list[Arret]]:
    """Donne à chaque drone, dans l'ordre des numéros, le plan restant dont le premier arrêt est le plus proche de lui.
    Renvoie les plans rangés dans l'ordre des drones."""
    libres = list(range(len(plans)))
    out: list[list[Arret]] = [[] for _ in plans]
    for i, p in enumerate(departs):
        if not libres:
            break
        k = min(libres, key=lambda k: float(np.linalg.norm(plans[k][0].position - np.asarray(p)))
                if plans[k] else float("inf"))
        out[i] = plans[k]
        libres.remove(k)
    return out


def _distance_boite(p, x0: float, x1: float, y0: float, y1: float) -> float:
    """Renvoie la distance horizontale (m) du point p au rectangle [x0, x1] × [y0, y1], 0 s'il est dedans."""
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


class ChampDeRisque:
    """Le champ de risque de l'article (« a signed-distance-derived, sigmoid Risk Factor ») : un risque continu et sa pente,
    tirés de la distance libre aux murs, aux racks, aux échos lidar récents et aux coéquipiers."""

    MEMOIRE_S = 3.0   # s : durée de vie d'un écho lidar dans le champ

    def __init__(self, layout, memoire_s: float = MEMOIRE_S):
        """Prépare le champ : emprises des racks lues sur le plan, échos lidar gardés `memoire_s` secondes, aucun coéquipier."""
        self.racks = [(r.x_bounds[0], r.x_bounds[1], r.y_bounds[0], r.y_bounds[1]) for r in layout.racks]
        self.memoire_s = memoire_s
        self.echos: list[tuple[float, np.ndarray]] = []
        self.mobiles: list[np.ndarray] = []

    def integre_lidar(self, origine, directions, portees, t: float) -> None:
        """Ajoute les points touchés par un tour de lidar (portées entre 0,4 et 25 m) et oublie les échos de plus de `memoire_s` s."""
        o = np.asarray(origine, float)
        d = np.asarray(directions, float)
        r = np.asarray(portees, float)
        vus = np.isfinite(r) & (r > 0.4) & (r < 25.0)
        if vus.any():
            self.echos.append((t, o + d[vus] * r[vus, None]))
        self.echos = [(u, p) for u, p in self.echos if t - u <= self.memoire_s]

    def annonce_coequipiers(self, positions) -> None:
        """Remplace la liste des positions des coéquipiers, comptés comme obstacles."""
        self.mobiles = [np.asarray(p, float) for p in positions]

    def distance(self, p) -> float:
        """Renvoie la distance libre horizontale (m) au plus proche obstacle : murs, racks, échos lidar, coéquipiers."""
        p = np.asarray(p, float)
        d = min(p[0] - INTERIOR.x_min, INTERIOR.x_max - p[0],
                p[1] - INTERIOR.y_min, INTERIOR.y_max - p[1])
        for x0, x1, y0, y1 in self.racks:
            d = min(d, _distance_boite(p, x0, x1, y0, y1))
        for _, pts in self.echos:
            e = pts - p
            loin = np.abs(e[:, 2]) < 1.0   # seuls comptent les échos à moins de 1 m au-dessus ou au-dessous du point
            if loin.any():
                d = min(d, float(np.linalg.norm(e[loin, :2], axis=1).min()))
        for q in self.mobiles:
            d = min(d, float(np.linalg.norm(q[:2] - p[:2])))
        return float(d)

    def risque(self, p) -> float:
        """Renvoie le risque R_f(p) = σ(β (r_safe − r(p))) de l'équation 9 (σ : sigmoïde) : 0,5 à 0,5 m d'un obstacle, près de 1 au contact."""
        return 1.0 / (1.0 + math.exp(-BETA * (R_SAFE - self.distance(p))))

    def sur(self, p) -> bool:
        """Renvoie vrai si le point est sûr : risque ≤ τ = 0,4, soit à plus de 0,54 m de tout obstacle."""
        return self.risque(p) <= TAU

    def gradient(self, p, pas: float = 0.15) -> np.ndarray:
        """Renvoie la pente (gradient) du risque dans le plan horizontal, z nul, par différences finies sur ±15 cm."""
        p = np.asarray(p, float)
        g = np.zeros(3)
        for k in (0, 1):
            e = np.zeros(3)
            e[k] = pas
            g[k] = (self.risque(p + e) - self.risque(p - e)) / (2 * pas)
        return g


def deflexion(p, champ: ChampDeRisque, dans_allee=None, pas: float = 0.25,
              maxi: float = DEFLEXION_MAX) -> np.ndarray | None:
    """Déplace un arrêt dangereux (« when a predicted segment violates safety ») en descendant la pente du risque par pas de 0,25 m.
    Renvoie le premier point sûr (et dans l'allée si `dans_allee` est donné) à moins de 1,2 m, ou None s'il n'y en a pas."""
    p0 = np.asarray(p, float)
    q = p0.copy()
    for _ in range(int(maxi / pas) + 1):
        if champ.sur(q) and (dans_allee is None or dans_allee(q)):
            return q
        g = champ.gradient(q)
        n = float(np.linalg.norm(g[:2]))
        if n < 1e-9:
            return None
        q = q - pas * g / n
        if float(np.linalg.norm(q[:2] - p0[:2])) > maxi:
            return None
    return q if champ.sur(q) and (dans_allee is None or dans_allee(q)) else None


class Routeur:
    """Le routage dans les allées : « linear keep-in constraints that restrict motion to the aisle polytope » (§2.3).
    Le drone reste dans les allées ; pour en changer, il contourne les racks par un couloir transversal, au sud ou au nord."""

    def __init__(self, layout):
        """Prépare les allées, les emprises des racks et la position en y des deux couloirs transversaux (sud et nord)."""
        self.allees = allees(layout)
        self.racks = [(r.x_bounds[0], r.x_bounds[1], r.y_bounds[0], r.y_bounds[1]) for r in layout.racks]
        y0 = min(r[2] for r in self.racks) - MARGE_COULOIR
        y1 = max(r[3] for r in self.racks) + MARGE_COULOIR
        self.couloirs = [max(y0, INTERIOR.y_min + MARGE_COULOIR / 2), min(y1, INTERIOR.y_max - MARGE_COULOIR / 2)]

    def allee_de(self, p) -> dict:
        """Renvoie l'allée qui contient le point p (selon x), ou la plus proche s'il n'est dans aucune."""
        x = float(p[0])
        for a in self.allees:
            if a["x0"] <= x <= a["x1"]:
                return a
        return min(self.allees, key=lambda a: min(abs(x - a["x0"]), abs(x - a["x1"])))

    def coupe_un_rack(self, a, b) -> bool:
        """Renvoie vrai si le segment a-b passe dans l'emprise d'un rack (testé tous les 20 cm)."""
        a, b = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
        n = max(int(np.linalg.norm(b - a) / 0.2), 1)
        for k in range(n + 1):
            q = a + (b - a) * k / n
            for x0, x1, y0, y1 in self.racks:
                if x0 <= q[0] <= x1 and y0 <= q[1] <= y1:
                    return True
        return False

    def chemin(self, depart, arrivee) -> list[np.ndarray]:
        """Renvoie les points de passage entre deux arrêts, sans l'arrivée : aucun si la ligne droite est libre,
        sinon deux points par le couloir transversal (sud ou nord) le plus court."""
        a = np.asarray(depart, float)
        b = np.asarray(arrivee, float)
        if not self.coupe_un_rack(a, b):
            return []
        aa, ab = self.allee_de(a), self.allee_de(b)
        candidats = []
        for y in self.couloirs:
            p1 = np.array([aa["centre"], y, b[2]])
            p2 = np.array([ab["centre"], y, b[2]])
            route = [p1, p2]
            if any(self.coupe_un_rack(u, v) for u, v in zip([a] + route, route + [b])):
                continue
            longueur = sum(float(np.linalg.norm(v - u)) for u, v in zip([a] + route, route + [b]))
            candidats.append((longueur, route))
        if not candidats:
            return []
        return min(candidats, key=lambda c: c[0])[1]


def _gris_seuille(bgr: np.ndarray) -> np.ndarray:
    """Prétraite l'image comme l'article : « grayscale conversion and adaptive thresholding ».
    Renvoie une image noir et blanc, seuillée par zones de 31 px pour compenser l'éclairage inégal."""
    gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return cv2.adaptiveThreshold(gris, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5)


def _nettete(gris: np.ndarray, coins: np.ndarray) -> float:
    """Renvoie le score de confiance d'une lecture (0 à 1) : la netteté de la zone du QR, variance du laplacien / 500, plafonnée à 1."""
    x0, y0 = np.clip(coins.min(axis=0).astype(int) - 2, 0, None)
    x1, y1 = coins.max(axis=0).astype(int) + 2
    zone = gris[max(y0, 0):min(y1, gris.shape[0]), max(x0, 0):min(x1, gris.shape[1])]
    if zone.size < 16:
        return 0.0
    return float(min(cv2.Laplacian(zone, cv2.CV_64F).var() / 500.0, 1.0))


def decode(bgr: np.ndarray) -> list[tuple[str, float]]:
    """Décode les QR d'une image comme la station sol de l'article : image en gris, puis seuillée, lue par `cv2.QRCodeDetector`.
    Renvoie les paires (code, confiance) triées, avec la meilleure confiance de chaque code."""
    gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    out: dict[str, float] = {}
    for image in (gris, _gris_seuille(bgr)):
        det = cv2.QRCodeDetector()
        try:
            ok, textes, points, _ = det.detectAndDecodeMulti(image)
        except cv2.error:
            continue
        if not ok or points is None:
            continue
        for texte, coins in zip(textes, points):
            if not texte:
                continue
            c = _nettete(gris, np.asarray(coins).reshape(-1, 2))
            out[texte] = max(out.get(texte, 0.0), c)
    return sorted(out.items())


@dataclass
class Lecture:
    """Une lecture reçue au sol : code lu, numéro du drone, position du drone (m), instant (s) et confiance (0 à 1)."""
    code: str
    drone: int
    position: np.ndarray
    t: float
    confiance: float


class Inventaire:
    """Le serveur d'inventaire au sol de l'article : il reçoit les lectures, écarte les doublons et note les arrêts à relire."""

    def __init__(self, dedup_s: float = DEDUP_S, confiance_min: float = CONFIANCE_MIN):
        """Prépare un inventaire vide : délai anti-doublon `dedup_s` (1,5 s) et seuil de confiance `confiance_min` (0,35)."""
        self.dedup_s, self.confiance_min = dedup_s, confiance_min
        self.evenements: list[Lecture] = []
        self.uniques: dict[str, Lecture] = {}
        self.doublons = 0
        self.a_relire: set[tuple] = set()

    def recoit(self, code: str, drone: int, position, t: float, confiance: float, arret=None) -> bool:
        """Enregistre une lecture et renvoie vrai seulement si le code entre pour la première fois à l'inventaire.
        Relu en moins de 1,5 s : doublon ignoré ; confiance sous 0,35 : arrêt noté à relire ; on garde la lecture la plus sûre."""
        e = Lecture(code, drone, np.asarray(position, float).copy(), t, confiance)
        ancien = self.uniques.get(code)
        if ancien is not None and t - ancien.t < self.dedup_s:
            self.doublons += 1
            return False
        self.evenements.append(e)
        if confiance < self.confiance_min:
            if arret is not None:
                self.a_relire.add(arret.cle())
            if ancien is not None:
                self.doublons += 1
                return False
        if ancien is None:
            self.uniques[code] = e
            return True
        self.doublons += 1
        if confiance > ancien.confiance:
            self.uniques[code] = e
        return False

    @property
    def codes(self) -> set[str]:
        """Renvoie l'ensemble des codes déjà à l'inventaire."""
        return set(self.uniques)

    def bilan(self) -> dict:
        """Renvoie le résumé écrit dans mission.json : codes uniques, lectures, doublons, arrêts à relire, confiance moyenne et minimale."""
        conf = [e.confiance for e in self.evenements] or [0.0]
        return {"codes_uniques": len(self.uniques), "lectures": len(self.evenements),
                "doublons": self.doublons, "a_relire": len(self.a_relire),
                "confiance_moyenne": round(float(np.mean(conf)), 3),
                "confiance_min": round(float(np.min(conf)), 3)}


def temps_avant_conflit(pa, va, pb, vb, s_min: float = SEPARATION_MIN) -> float:
    """Calcule le « predicted time-to-conflict τ_c(∆x, ∆y, ∆v) » du cas II : dans combien de secondes (vitesses constantes,
    dans le plan) deux drones passeront à moins de 1,2 m l'un de l'autre ; renvoie l'infini si cela n'arrive pas."""
    dp = np.asarray(pb, float)[:2] - np.asarray(pa, float)[:2]
    dv = np.asarray(vb, float)[:2] - np.asarray(va, float)[:2]
    a = float(dv @ dv)
    if a < 1e-9:
        return 0.0 if float(dp @ dp) < s_min ** 2 else float("inf")
    b = 2.0 * float(dp @ dv)
    c = float(dp @ dp) - s_min ** 2
    disc = b * b - 4 * a * c
    if disc < 0:
        return float("inf")
    t = (-b - math.sqrt(disc)) / (2 * a)
    if t < 0:
        t = (-b + math.sqrt(disc)) / (2 * a)
    return t if t >= 0 else float("inf")


def cede_le_passage(moi: int, pa, va, autres) -> bool:
    """Renvoie vrai si ce drone doit s'arrêter : « a token-based rule at the GCS pauses the follower » (GCS : station sol).
    C'est le cas si un conflit est prévu dans moins de 3 s avec un drone de numéro plus petit, qui a la priorité."""
    for j, pb, vb in autres:
        if temps_avant_conflit(pa, va, pb, vb) < CONFLIT_S and j < moi:
            return True
    return False


def vitesse_selon_confiance(v_max: float, confiance: float | None,
                            seuil: float = CONFIANCE_MIN, plancher: float = 0.35) -> float:
    """Renvoie la vitesse de transit (m/s) : « a confidence-aware controller that throttles forward velocity ».
    v_max si la confiance est inconnue ou ≥ 0,35 ; sinon réduite en proportion, sans descendre sous 0,35 m/s."""
    if confiance is None or confiance >= seuil:
        return v_max
    return max(plancher, v_max * max(confiance, 0.0) / seuil)
