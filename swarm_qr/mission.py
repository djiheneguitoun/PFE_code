"""Chef d'orchestre d'une mission d'inventaire à plusieurs drones (étape 5), point d'entrée de swarm_qr.

Fait décoller les drones (pilotes ArduPilot simulés, SITL) puis répète des cycles de 0,2 s simulée : commande,
physique, observation, décision, arrêt ; tout est écrit dans --sortie. Depuis la racine ($PY = Python d'Isaac Sim) :
    $PY swarm_qr/mission.py --seed 9033 --drones 3 --budget 600 --detecteur auto --sortie <dossier>"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]      # racine du projet, ajoutée au chemin d'import pour « swarm_qr »
sys.path.insert(0, str(ROOT))
sys.stdout.reconfigure(line_buffering=True)     # chaque ligne affichée part tout de suite dans le journal

# Options lues avant de démarrer Isaac Sim ; parse_known_args ignore les options inconnues au lieu de s'arrêter.
parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=9033)
parser.add_argument("--drones", type=int, default=3)
parser.add_argument("--budget", type=float, default=600.0, help="secondes de temps simulé")
parser.add_argument("--detecteur", default="auto", help="'auto', un .pt, ou '' pour le repérage classique")
parser.add_argument("--guide", default="", help="'' = sans guide ; 'entraine' = le modèle 3B + adaptateur LoRA ; sinon un nom de modèle (étape 8)")
parser.add_argument("--modele-guide", default=str(Path.home() / "Documents" / "qwen2.5-vl-3b"), dest="modele_guide")
parser.add_argument("--adaptateur", default=str(Path(__file__).resolve().parent / "experiments" / "12_guide" / "adaptateur_lora"))
parser.add_argument("--lam", type=float, default=0.0, help="poids de l'avis du guide dans la note")
parser.add_argument("--panne", default="", help="drone:temps — ce drone cesse d'agir à cet instant simulé")
parser.add_argument("--instantanes", type=float, default=10.0, help="secondes simulées entre deux instantanés (0 = aucun)")
parser.add_argument("--codes-attendus", type=int, default=0, dest="codes_attendus",
                    help="taille connue de l'inventaire (0 = le nombre de cartons de la scène)")
parser.add_argument("--part-arret", type=float, default=0.95, dest="part_arret",
                    help="part des codes attendus à partir de laquelle on accorde la grâce puis on s'arrête (0 = jamais)")
parser.add_argument("--grace", type=float, default=60.0, help="secondes de vol accordées après la part atteinte")
parser.add_argument("--sans-progres", type=float, default=120.0, dest="sans_progres",
                    help="secondes sans code nouveau après lesquelles on s'arrête (0 = jamais)")
parser.add_argument("--video", action="store_true", help="enregistre les caméras fixes à chaque rendu (vidéo à vitesse réelle)")
parser.add_argument("--cameras", type=int, default=2, choices=[2, 3, 5],
                    help="2 = couloir central + grande zone (choix de l'utilisatrice) ; 3 = + vue d'ensemble ; 5 = tous les couloirs")
parser.add_argument("--obstacle", default="", help="x,y,t — un bloc de 1x1x2 m apparaît à cet endroit à cet instant simulé")
parser.add_argument("--sortie", required=True)
args, _ = parser.parse_known_args()

# Isaac Sim doit être démarré (sans fenêtre, sans vérifier la version du pilote de la carte graphique) AVANT
# d'importer les modules omni et ceux du projet qui en dépendent : d'où les imports qui suivent.
from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": True, "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"]}
)

import traceback

import cv2
import numpy as np
import omni.timeline

from swarm_qr import control, mapping, planning
from swarm_qr.env import scene as scene_mod
from swarm_qr.env.config import CAMERAS
from swarm_qr.env.layout import make_layout
from swarm_qr.env.pilot import PHYS_DT, Clock, Pilot
from swarm_qr.experiments import _img
from swarm_qr.observation import Observateur

PHYS_PAR_CYCLE = 160       # pas de physique par cycle : 160 × 1/800 s = 0,2 s simulée (un rendu par cycle)
FLY_ALT = 1.6              # m, altitude de décollage du drone 0 ; chaque drone suivant décolle 30 cm plus haut
SEPARATION = 2.5           # m : en dessous, le drone au plus grand numéro cède le passage
SEPARATION_Z = 1.2         # m : écart vertical sous lequel deux drones se gênent (règles de passage)
URGENCE = 1.5              # m : en dessous, tout drone s'arrête, prioritaire ou non
BLOCAGE_S = 15.0           # s : une cible depuis 15 s sans avoir bougé de 30 cm, le drone est coincé et abandonne
RECALCULS_MAX = 30         # recalculs de chemin au plus pour une même cible, ensuite abandon
DECISION_REPOS_S = 2.0     # s : un drone sans cible ne redécide qu'une fois toutes les 2 s
V_APPROCHE = 0.6           # m/s, vitesse maximale en approche finale
RAYON_COEQUIPIER = 2.0     # m, rayon de l'obstacle que forme chaque coéquipier pendant un calcul de chemin
MARGE_ALTITUDE = 1.5       # m : un changement d'altitude se fait à au moins 1,5 m de tout obstacle connu
TOL_ARRIVEE = 0.35         # m, écart de position accepté pour dire la pose atteinte
GAIN_MISSION = 0.5         # 1/s : vitesse = gain × distance restante ; 0,9 (réglé à un drone) oscillait à trois
V_TRANSIT_MISSION = 1.0    # m/s, vitesse de croisière entre les points de passage
LECTURE_S = 2.0            # s de tenue devant une cible « lire » ou « couvrir » (10 images par caméra)
FIN_S = 30.0               # s : quand plus aucun drone n'a de cible pendant 30 s, la mission s'arrête
AVIS_S = 5.0               # s, délai minimal entre deux demandes d'avis au guide, pour un même drone
SORTIE = Path(args.sortie)  # dossier de sortie : mission.json, carte, instantanés, vidéo, journaux ArduPilot


def _json_sur(o):
    """Convertit les nombres et tableaux numpy en types Python pour json.dumps ; lève TypeError pour le reste."""
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"non serialisable : {type(o)}")


class Agent:
    """Un drone dans la mission : son pilote, son contrôleur, ses capteurs, son cerveau."""

    def __init__(self, i, scene, clock, carte, K, detecteur, lam):
        """Fait décoller le drone `i` (à 1,6 m + 30 cm par numéro) puis crée son contrôleur, son observateur et son
        cerveau ; lève RuntimeError si le décollage échoue."""
        self.i = i
        self.pilot = Pilot(scene.world, i, clock)
        if not self.pilot.ready(FLY_ALT + 0.3 * i, lambda: float(scene.position(i)[2])):
            raise RuntimeError(f"le drone {i} n'a pas decolle")
        # le contrôleur lit la position, le cap et la vitesse VRAIS dans la simulation
        self.ctrl = control.Controleur(self.pilot, lambda: scene.position(i), lambda: scene.yaw(i),
                                       lambda: scene.velocity(i), v_approche=V_APPROCHE, tol=TOL_ARRIVEE,
                                       gain=GAIN_MISSION, v_transit=V_TRANSIT_MISSION)
        self.obs = Observateur(scene, carte, K, drone=i, detecteur=detecteur)    # capteurs -> carte partagée
        self.cerveau = planning.Cerveau(carte, i, lam=lam)                       # choix de la prochaine cible
        self.cible: planning.Cible | None = None
        self.t_arrivee: float | None = None     # s, arrivée devant la cible (phase « atteint »)
        self.t_sans_cible: float | None = None  # s, depuis quand le drone n'a plus de cible (None s'il en a une)
        self.vivant = True                      # faux après une panne ou une chute
        self.decisions: list[dict] = []
        self.attentes = 0                       # nombre de cycles passés à céder le passage
        self.avis: planning.Avis | None = None
        self.t_avis = -1e9
        self.codes_avant = 0                    # codes lus au moment de choisir la cible (pour savoir si elle a été lue)
        self.t_cible = 0.0                      # s, heure du choix de la cible
        self.t_decision = -1e9
        self.attente = None                     # (consigne, points) mis de côté pendant qu'il cède le passage


def zones_candidates(cibles, n_max: int = 6, taille: float = 3.0) -> list[dict]:
    """Regroupe les cibles par carrés de 3 m et renvoie les 6 zones les plus utiles, numérotées pour le guide
    (centre, rayon, nombre de cibles de chaque sorte, côté le plus fréquent)."""
    groupes: dict[tuple, list] = {}
    for c in cibles:
        cle = (math.floor(c.origine[0] / taille), math.floor(c.origine[1] / taille))
        groupes.setdefault(cle, []).append(c)
    zones = []
    for cle, cs in groupes.items():
        centre = np.mean([c.origine[:2] for c in cs], axis=0)
        cotes = [c.cote for c in cs if c.genre != "explorer"]
        zones.append({"centre": [round(float(v), 2) for v in centre], "rayon": taille * 0.75,
                      "utilite": round(float(sum(c.utilite for c in cs)), 1), "cibles": len(cs),
                      "genres": sorted({c.genre for c in cs}),
                      "n_lire": sum(1 for c in cs if c.genre == "lire"),
                      "n_couvrir": sum(1 for c in cs if c.genre == "couvrir"),
                      "n_couvrir_cartons": sum(1 for c in cs if c.genre == "couvrir" and c.utilite > planning.UTILITE_SURFACE * planning.SURFACE_MIN * 2),
                      "n_explorer": sum(1 for c in cs if c.genre == "explorer"),
                      "cote": planning.NOMS_COTES[max(set(cotes), key=cotes.count)] if cotes else None})
    zones.sort(key=lambda z: -z["utilite"])
    for k, z in enumerate(zones[:n_max]):
        z["numero"] = k + 1
    return zones[:n_max]


def vue_annotee(carte, trajectoires, zones) -> np.ndarray:
    """Renvoie la vue de dessus de la carte avec la trajectoire de chaque drone et les zones numérotées (cercles rouges)."""
    img = mapping.vue_de_dessus(carte, trajectoire=trajectoires[0] if trajectoires else None)
    ech = img.shape[1] / carte.forme[0]
    ny = carte.forme[1]

    def px(p):
        """Convertit un point du monde en pixel (colonne, ligne) de l'image."""
        i, j = carte.indice(p)[0][:2]
        return int((i + 0.5) * ech), int((ny - j - 0.5) * ech)

    for tr, couleur in zip(trajectoires[1:], mapping.COULEURS_DRONES[1:]):
        pts = [px(p) for p in tr if carte.sur_la_carte(p)[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            cv2.line(img, a, b, couleur, 1, cv2.LINE_AA)
    for z in zones:
        c = px([*z["centre"], 0.0])
        cv2.circle(img, c, int(z["rayon"] * ech / carte.g.cell), (0, 0, 230), 2)
        cv2.putText(img, str(z["numero"]), (c[0] - 8, c[1] + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 230), 2, cv2.LINE_AA)
    return img


def verite_des_zones(zones, tags, carte) -> dict:
    """Calcule avec la vérité de la simulation la réponse attendue du guide : la zone qui contient le plus de QR
    non lus, et le côté vers lequel ils sont tournés (sert à entraîner et juger le guide hors ligne)."""
    lus = carte.codes
    restants = [t for t in tags if t.tag_id not in lus]
    meilleur, n_meilleur, cote = None, -1, None
    for z in zones:
        c = np.array(z["centre"])
        dans = [t for t in restants if np.linalg.norm(np.array(t.position[:2]) - c[:2]) <= z["rayon"]]
        if len(dans) > n_meilleur:
            meilleur, n_meilleur = z["numero"], len(dans)
            if dans:
                normale = np.mean([t.normal for t in dans], axis=0)
                cote = planning.NOMS_COTES[carte.cardinal(normale)]
    return {"zone": meilleur, "panneaux_restants": n_meilleur, "cote": cote,
            "restants_total": len(restants)}


def main() -> None:
    """Construit l'entrepôt et l'essaim, fait voler la mission cycle par cycle jusqu'à un critère d'arrêt,
    puis écrit mission.json, la carte et les images dans le dossier de sortie."""
    SORTIE.mkdir(parents=True, exist_ok=True)
    (SORTIE / "instantanes").mkdir(exist_ok=True)
    # entrepôt tiré de la graine, drones et pilotes automatiques SITL, puis le temps simulé démarre
    layout = make_layout(args.seed)
    scene = scene_mod.build(layout, with_sitl=True, n_drones=args.drones)
    scene.world.reset()
    scene.finalize()
    omni.timeline.get_timeline_interface().play()
    # calibration de la caméra vérifiée avant de voler : focale attendue 1024 / (2 × tan 30°) ≈ 886,8 pixels
    K = np.asarray(scene.cameras[0]["left"].get_intrinsics_matrix(), float)
    fx_attendu = CAMERAS.side_width / (2.0 * math.tan(math.radians(CAMERAS.fov_deg) / 2.0))
    if abs(K[0, 0] - fx_attendu) > 1.0:
        raise RuntimeError(f"calibration incoherente : fx={K[0, 0]:.1f}, attendu {fx_attendu:.1f}")

    detecteur = None
    if args.detecteur:
        from swarm_qr.detecteur import Detecteur

        detecteur = Detecteur() if args.detecteur == "auto" else Detecteur(args.detecteur)
        print(f"oeil appris : {detecteur.imgsz} px, seuil {detecteur.conf}")
    guide = None
    if args.guide:
        from swarm_qr.guide import Guide, GuideEntraine

        guide = (GuideEntraine(args.modele_guide, args.adaptateur) if args.guide == "entraine"
                 else Guide(args.guide))
        print(f"guide : {args.guide}, lambda {args.lam}")

    clock = Clock(scene.world)
    carte = mapping.Carte()         # une seule carte, partagée par tous les drones
    # chaque Agent fait décoller son drone : les décollages se suivent (environ 95 s simulées pour trois drones)
    agents = [Agent(i, scene, clock, carte, K, detecteur, args.lam) for i in range(args.drones)]
    panne = None
    if args.panne:
        d, t = args.panne.split(":")
        panne = (int(d), float(t))
    # vérité de la scène (codes, positions) : nombre de codes attendus, journal, réponse attendue du guide
    tags = scene.tags
    jeux = {2: scene_mod.CAMERAS_VIDEO_2, 3: scene_mod.CAMERAS_VIDEO_3, 5: scene_mod.CAMERAS_VIDEO}
    cams_video = scene_mod.cameras_fixes(jeux[args.cameras]) if args.video else {}
    if args.video:
        for nom in list(cams_video) + ["lecteur"]:
            (SORTIE / "video" / nom).mkdir(parents=True, exist_ok=True)
        index_video: list[dict] = []
    obstacle_prevu = None
    if args.obstacle:
        ox, oy, ot = (float(v) for v in args.obstacle.split(","))
        obstacle_prevu = {"x": ox, "y": oy, "t": ot, "pose": False}
    # emprise des racks, transmise au guide entraîné
    racks_connus = [{"prim": r.prim, "x": list(r.x_bounds), "y": list(r.y_bounds)} for r in layout.racks]
    journal = {"seed": args.seed, "drones": args.drones, "budget_s": args.budget,
               "arret": {"codes_attendus": args.codes_attendus or len({t.tag_id for t in tags}), "part": args.part_arret,
                         "grace_s": args.grace, "sans_progres_s": args.sans_progres},
               "detecteur": args.detecteur, "guide": args.guide, "lam": args.lam,
               "panne": args.panne, "evenements": [], "codes_par_t": [], "instantanes": []}
    mur0 = time.monotonic()
    n_cycle = 0
    t_fin_candidats = None          # s, depuis quand plus aucun drone n'a de cible
    fin = None                      # raison de l'arrêt de la mission
    codes_attendus = args.codes_attendus or len({t.tag_id for t in tags})
    t_dernier_code, t_part_atteinte = 0.0, None
    prochain_instantane = 0.0
    avis_en_cours: dict[int, object] = {}       # demande au guide en cours pour chaque drone (calcul en arrière-plan)

    def evenement(genre, **kw):
        """Ajoute au journal un événement daté en temps simulé (panne, chute, abandon, avis…)."""
        journal["evenements"].append({"t": round(clock.t, 1), "genre": genre, **kw})

    def libre_de_passage(a: Agent) -> bool:
        """Renvoie faux si le drone doit s'arrêter : un coéquipier vivant à moins de 1,5 m, ou un drone de plus petit
        numéro à moins de 2,5 m (seulement s'ils sont à moins de 1,2 m l'un de l'autre en hauteur)."""
        p = scene.position(a.i)
        for b in agents:
            if b.i == a.i or not b.vivant:
                continue
            q = scene.position(b.i)
            proche = np.linalg.norm(p[:2] - q[:2])
            if proche < URGENCE and abs(p[2] - q[2]) < SEPARATION_Z:
                return False
            if b.i < a.i and proche < SEPARATION and abs(p[2] - q[2]) < SEPARATION_Z:
                return False
        return True

    def bloque(a: Agent) -> bool:
        """Renvoie vrai si le drone a une cible depuis 15 s sans avoir bougé de plus de 30 cm : il est coincé."""
        if a.cible is None or clock.t - a.t_cible < BLOCAGE_S:
            return False
        recents = [p for p in a.obs.trajectoire if p[0] >= max(a.t_cible, clock.t - BLOCAGE_S)]
        if len(recents) < 5:
            return False
        pts = np.array([p[1:] for p in recents])
        return bool(np.ptp(pts, axis=0).max() < 0.3)

    def tient_sur_place(a: Agent) -> None:
        """Donne au drone l'ordre de tenir sa position actuelle : une simple vitesse nulle le laisserait dériver."""
        ici = scene.position(a.i)
        a.ctrl.assigne(control.Consigne(ici.copy(), scene.yaw(a.i)))

    def avec_coequipiers(a: Agent):
        """Fait des autres drones vivants des obstacles de 2 m de rayon sur la carte, le temps que celui-ci planifie."""
        carte.obstacles_mobiles = [(scene.position(b.i).copy(), RAYON_COEQUIPIER)
                                   for b in agents if b.i != a.i and b.vivant]

    def decide(a: Agent) -> None:
        """Clôt la cible en cours (lue ou non), choisit la suivante, la réserve et l'envoie au contrôleur."""
        if a.cible is not None:
            # une cible « lire » est lue si le nombre de codes a augmenté depuis qu'on l'a choisie
            lu = a.cible.genre == "lire" and len(carte.codes) > a.codes_avant
            if lu:
                carte.oublie_pistes(a.cible.origine, 1.0)
            a.cerveau.constate(a.cible, lu)
            if a.ctrl.phase is control.Phase.ABANDON:
                carte.ecarte(a.cible.position)
            a.decisions[-1].update({"fin": round(clock.t, 1), "phase": a.ctrl.phase.value,
                                    "lu": bool(lu), "raison": a.ctrl.bilan.raison if a.ctrl.bilan else ""})
            carte.libere(a.i)
            a.cible = None
        avec_coequipiers(a)
        choix = a.cerveau.choisit(scene.position(a.i), avis=a.avis, t=clock.t)
        carte.obstacles_mobiles = []
        if choix is None:
            if a.t_sans_cible is None:
                a.t_sans_cible = clock.t
                evenement("sans_cible", drone=a.i)
                tient_sur_place(a)
            return
        a.t_sans_cible = None
        cap = choix.cap if choix.cap is not None else scene.yaw(a.i)
        ici = scene.position(a.i)
        points = list(choix.points)
        # plus de 50 cm de dénivelé : aller d'abord, à l'altitude actuelle, vers un point dégagé (à 1,5 m de tout
        # obstacle connu ; à défaut, sur place), y monter ou descendre, puis suivre le chemin
        if abs(choix.position[2] - ici[2]) > 0.5:
            degage = a.cerveau.point_degage(ici, MARGE_ALTITUDE)
            if degage is None or not carte.segment_libre(ici, np.array([degage[0], degage[1], ici[2]])):
                degage = ici
            prefixe = [] if np.linalg.norm(degage[:2] - ici[:2]) < 0.3 else [np.array([degage[0], degage[1], ici[2]])]
            points = prefixe + [np.array([degage[0], degage[1], choix.position[2]])] + points
        a.ctrl.assigne(control.Consigne(choix.position, cap), points)
        carte.reserve(a.i, choix.position)
        a.cible = choix
        a.attente = None
        a.t_cible = clock.t
        a.t_arrivee = None
        a.codes_avant = len(carte.codes)
        a.decisions.append({"t": round(clock.t, 1), "genre": choix.genre,
                            "position": [round(float(v), 2) for v in choix.position],
                            "origine": [round(float(v), 2) for v in choix.origine],
                            "cote": choix.cote, "note": round(choix.note, 1),
                            "distance": round(choix.distance, 1), "points": len(choix.points),
                            "avis": None if a.avis is None else a.avis.phrase})

    def replanifie(a: Agent) -> None:
        """Revérifie le reste du chemin sur la carte ; s'il est coupé, en calcule un nouveau, ou abandonne la cible
        s'il n'y en a pas ou après 30 recalculs."""
        c = a.cible
        ici = scene.position(a.i)
        reste = [ici] + list(a.ctrl.points[a.ctrl.i_point:]) + [c.position]
        avec_coequipiers(a)
        try:
            if all(carte.segment_libre(u, v, altitude=float(c.position[2])) for u, v in zip(reste[:-1], reste[1:])):
                return
            nouveau = carte.chemin(ici, c.position, altitude=float(c.position[2]))
        finally:
            carte.obstacles_mobiles = []
        if nouveau is None:
            abandonne(a, "chemin coupe")
            return
        cap = c.cap if c.cap is not None else scene.yaw(a.i)
        a.decisions[-1]["replanifications"] = a.decisions[-1].get("replanifications", 0) + 1
        if a.decisions[-1]["replanifications"] > RECALCULS_MAX:
            abandonne(a, "trop de recalculs")
            return
        a.ctrl.assigne(control.Consigne(c.position, cap), nouveau)

    def abandonne(a: Agent, raison: str) -> None:
        """Fait abandonner sa cible au drone (phase ABANDON, avec la raison) et l'inscrit au journal."""
        a.ctrl.phase = control.Phase.ABANDON
        a.ctrl.bilan = control.Bilan(control.Phase.ABANDON, raison, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        evenement("abandon", drone=a.i, raison=raison)

    def sauve_journaux_ardupilot() -> None:
        """Copie les journaux de vol ArduPilot (.BIN) de chaque drone dans <sortie>/ardupilot_logs : ils sont écrits
        dans un dossier temporaire, effacé à la fermeture du simulateur."""
        import shutil
        for i, d in enumerate(scene.drones):
            try:
                outil = d._backends[0].ardupilot_tool
                src = Path(outil.root_fs.name) / "logs"
                dst = SORTIE / "ardupilot_logs" / f"drone_{i}"
                dst.mkdir(parents=True, exist_ok=True)
                for f in src.glob("*.BIN"):
                    shutil.copy2(f, dst / f.name)
            except Exception as e:
                print(f"  journaux ArduPilot du drone {i} non copies ({type(e).__name__}: {e})", flush=True)

    def enregistre_video() -> None:
        """Enregistre une image par caméra fixe à chaque cycle, plus celle de la caméra du drone qui lit ;
        met à jour video/index.json toutes les 100 images."""
        n = len(index_video)
        # le « lecteur » : un drone arrivé devant un QR à lire, sinon le premier drone vivant
        lecteur = next((a.i for a in agents if a.vivant and a.cible is not None and a.cible.genre == "lire"
                        and a.ctrl.phase is control.Phase.ATTEINT), None)
        if lecteur is None:
            lecteur = next((a.i for a in agents if a.vivant), None)
        for nom, cam in cams_video.items():
            img = cam.get_rgb()
            if img is not None and getattr(img, "ndim", 0) == 3 and img.size:
                cv2.imwrite(str(SORTIE / "video" / nom / f"{n:05d}.jpg"), _img.to_bgr(img), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if lecteur is not None:
            img = scene.cameras[lecteur]["left"].get_rgb()
            if img is not None and getattr(img, "ndim", 0) == 3 and img.size:
                cv2.imwrite(str(SORTIE / "video" / "lecteur" / f"{n:05d}.jpg"),
                            cv2.resize(_img.to_bgr(img), (480, 360)), [cv2.IMWRITE_JPEG_QUALITY, 85])
        index_video.append({"n": n, "t": round(clock.t, 1), "codes": len(carte.codes), "lecteur": lecteur,
                            "drones": [{"i": a.i, "vivant": a.vivant,
                                        "position": [round(float(v), 2) for v in scene.position(a.i)],
                                        "genre": a.cible.genre if a.cible else None} for a in agents]})
        if n % 100 == 0:
            (SORTIE / "video" / "index.json").write_text(json.dumps(
                {"pas_s": 0.2, "codes_attendus": codes_attendus, "images": index_video}))

    def instantane() -> None:
        """Enregistre un instantané (toutes les 10 s simulées par défaut) : vue annotée, images des caméras, état des
        drones, zones et contenu de la carte ; sert au rejeu et aux bancs du guide. Rien s'il n'y a aucune zone."""
        cibles = agents[0].cerveau.candidats(clock.t)
        zones = zones_candidates(cibles)
        if not zones:
            return
        k = len(journal["instantanes"])
        trajs = [[p[1:] for p in a.obs.trajectoire] for a in agents]
        vue = vue_annotee(carte, trajs, zones)
        cv2.imwrite(str(SORTIE / "instantanes" / f"{k:03d}_vue.png"), vue)
        for a in agents:
            if a.vivant:
                for nom, suffixe in (("left", ""), ("right", "d")):
                    img = scene.cameras[a.i][nom].get_rgb()
                    if img is not None and getattr(img, "ndim", 0) == 3 and img.size:
                        cv2.imwrite(str(SORTIE / "instantanes" / f"{k:03d}_cam{a.i}{suffixe}.jpg"),
                                    cv2.resize(_img.to_bgr(img), (512, 384)), [cv2.IMWRITE_JPEG_QUALITY, 85])
        journal["instantanes"].append({
            "k": k, "t": round(clock.t, 1), "zones": zones,
            "drones": [{"i": a.i, "position": [round(float(v), 2) for v in scene.position(a.i)],
                        "cap": round(scene.yaw(a.i), 3), "vivant": a.vivant,
                        "phase": a.ctrl.phase.name if a.ctrl.phase is not None else None,
                        "cible": None if a.cible is None else {
                            "genre": a.cible.genre, "position": _liste(a.cible.position),
                            "origine": _liste(a.cible.origine), "cote": a.cible.cote,
                            "utilite": round(float(a.cible.utilite), 1), "note": round(float(a.cible.note), 1),
                            "depuis": round(clock.t - a.t_cible, 1)}} for a in agents],
            "verite": verite_des_zones(zones, tags, carte),
            **dossier_de_la_carte(k, cibles)})

    def _liste(v):
        """Renvoie un vecteur sous forme de liste de flottants arrondis au centimètre, pour le JSON."""
        return [round(float(x), 2) for x in np.asarray(v).ravel()]

    def dossier_de_la_carte(k: int, cibles) -> dict:
        """Sauve la grille de l'instantané k et renvoie ce que la carte sait (panneaux, pistes, cibles, frontières,
        réservations), pour les bancs hors ligne ; renvoie {} si l'écriture échoue."""
        try:
            carte.sauve(SORTIE / "instantanes" / f"{k:03d}_carte")
            front = carte.frontieres(planning.Z_MIN, planning.Z_MAX)
            return {
                "resume": carte.resume(),
                "panneaux": [{"code": q.code, "position": _liste(q.position), "normale": _liste(q.normale),
                              "lectures": q.lectures, "vu_le": round(q.vu_le, 1)} for q in carte.panneaux],
                "pistes": [{"position": _liste(q.position),
                            "normale": None if q.normale is None else _liste(q.normale),
                            "vues": q.vues, "vu_le": round(q.vu_le, 1)} for q in carte.pistes],
                "cibles": [{"genre": c.genre, "position": _liste(c.position), "origine": _liste(c.origine),
                            "cote": c.cote, "utilite": round(float(c.utilite), 1)} for c in cibles],
                "frontieres": {"cases": int(len(front)),
                               "exemples": [_liste(f) for f in np.asarray(front)[:200]]},
                "reservations": [{"drone": r.drone, "cible": _liste(r.cible), "jusqu_a": round(r.jusqu_a, 1)}
                                 for r in carte.reservations.values()],
            }
        except Exception as e:
            print(f"  instantane {k} : dossier incomplet ({type(e).__name__}: {e})", flush=True)
            return {}

    def demande_avis(a: Agent) -> None:
        """Demande un avis au guide (étape 8), au plus toutes les 5 s ; le guide répond en arrière-plan et l'avis
        reçu sert à la décision suivante du drone."""
        if guide is None or clock.t - a.t_avis < AVIS_S:
            return
        from swarm_qr.guide import GuideEntraine
        # une demande déjà en cours : on attend sa réponse avant d'en lancer une autre
        if a.i in avis_en_cours:
            if not avis_en_cours[a.i].done():
                return
            avis = avis_en_cours.pop(a.i).result()
            if avis is not None:
                a.avis = avis
                evenement("avis", drone=a.i, zone=avis.phrase)
        cibles = a.cerveau.candidats(clock.t)
        zones = zones_candidates(cibles)
        if not zones:
            return
        # guide entraîné : la situation décrite en texte ; sinon, image de la caméra gauche + vue de dessus annotée
        if isinstance(guide, GuideEntraine):
            cas = {"t": round(clock.t, 1), "codes_lus": len(carte.codes), "drone": a.i,
                   "position": [float(v) for v in scene.position(a.i)],
                   "cap_deg": round(float(np.degrees(scene.yaw(a.i))), 1),
                   "coequipiers": [{"i": b.i, "position": [float(v) for v in scene.position(b.i)],
                                    "cap": float(scene.yaw(b.i)), "vivant": b.vivant} for b in agents if b.vivant],
                   "racks": racks_connus}
            avis_en_cours[a.i] = guide.demande(cas, zones)
            a.t_avis = clock.t
            return
        img = scene.cameras[a.i]["left"].get_rgb()
        if img is None or getattr(img, "ndim", 0) != 3 or not img.size:
            return
        vue = vue_annotee(carte, [[p[1:] for p in a.obs.trajectoire]], zones)
        avis_en_cours[a.i] = guide.demande(_img.to_bgr(img), vue, zones, scene.position(a.i))
        a.t_avis = clock.t

    print(f"mission : {args.drones} drones, entrepot {args.seed}, budget {args.budget:.0f} s\n")
    cycles_ms: list[float] = []     # durée de calcul de chaque cycle (ms, temps réel)
    try:
        # Boucle principale : un tour = un cycle de 0,2 s simulée ; le budget compte aussi le décollage (~95 s).
        while clock.t < args.budget:
            mur_cycle = time.monotonic()
            # 1. Commande : chaque drone envoie sa consigne de vol (un drone en panne continue de tenir sa place)
            for a in agents:
                if not a.vivant:
                    a.ctrl.tick()
                    continue
                if libre_de_passage(a):
                    if a.attente is not None:       # la voie est libre : il reprend l'ordre mis de côté
                        consigne, points = a.attente
                        a.attente = None
                        a.ctrl.assigne(consigne, points)
                    a.ctrl.tick()
                else:
                    # il doit céder le passage : il met son ordre de côté et tient sa place
                    if a.attente is None:
                        a.attente = (a.ctrl.consigne, list(a.ctrl.points[a.ctrl.i_point:]))
                        tient_sur_place(a)
                    a.ctrl.tick()
                    a.attentes += 1
            # 2. Physique : 160 pas de 1/800 s ; seul le dernier rend les images (caméras et lidar)
            for _ in range(PHYS_PAR_CYCLE - 1):
                scene.world.step(render=False)
            scene.world.step(render=True)
            # obstacle surprise (--obstacle), posé à l'instant prévu
            if obstacle_prevu and not obstacle_prevu["pose"] and clock.t >= obstacle_prevu["t"]:
                emprise = scene_mod.ajoute_obstacle("bloc_1", (obstacle_prevu["x"], obstacle_prevu["y"]))
                obstacle_prevu["pose"] = True
                journal["obstacle"] = {**emprise, "t": round(clock.t, 1)}
                evenement("obstacle", t_apparition=round(clock.t, 1), x=obstacle_prevu["x"], y=obstacle_prevu["y"])
                print(f"  t={clock.t:6.1f} s  OBSTACLE pose en ({obstacle_prevu['x']}, {obstacle_prevu['y']})")
            if args.video:
                enregistre_video()
            clock.t += PHYS_PAR_CYCLE * PHYS_DT     # pas faits sans clock.pump : l'horloge simulée avance à la main
            n_cycle += 1
            # 3. Observation : lidar, lectures et repérages versés dans la carte partagée ;
            #    le détecteur appris ne regarde qu'un drone par cycle, à tour de rôle
            for a in agents:
                if a.vivant:
                    a.obs.observe(clock.t, oeil=(n_cycle % len(agents) == a.i))
            carte.vieillit(clock.t)                 # réservations et liste noire expirées
            journal["codes_par_t"].append([round(clock.t, 1), len(carte.codes)])
            # panne simulée (--panne) : le drone ne décide plus, libère sa cible et tient sa position
            if panne and clock.t >= panne[1] and agents[panne[0]].vivant:
                a = agents[panne[0]]
                a.vivant = False
                carte.libere(a.i)
                a.cible = None
                tient_sur_place(a)
                evenement("panne", drone=a.i)
                print(f"  t={clock.t:6.1f} s  PANNE du drone {a.i}")
            # chute : sous 30 cm d'altitude ou incliné de plus de 70°, contrôlé après 120 s simulées
            # (le décollage en prend déjà environ 95)
            for a in agents:
                inclinaison = a.obs.inclinaisons[-1][1] if a.obs.inclinaisons else 0.0
                if a.vivant and clock.t > 120.0 and (scene.position(a.i)[2] < 0.3 or inclinaison > 70.0):
                    a.vivant = False
                    carte.libere(a.i)
                    a.cible = None
                    evenement("chute", drone=a.i, position=[round(float(v), 2) for v in scene.position(a.i)])
                    print(f"  t={clock.t:6.1f} s  CHUTE du drone {a.i}")
            # 4. Décision : au plus une nouvelle décision par cycle pour tout l'essaim, car un long calcul
            #    immobilise la simulation
            decide_fait = False
            for a in agents:
                if not a.vivant:
                    continue
                demande_avis(a)
                if a.cible is not None and a.ctrl.phase not in control.TERMINALES and bloque(a):
                    abandonne(a, "immobile")
                phase = a.ctrl.phase
                if a.cible is not None and phase not in control.TERMINALES:
                    # en route : le chemin restant est revérifié tous les 2 cycles (0,4 s)
                    if n_cycle % 2 == 0 and phase in (control.Phase.TRANSIT, control.Phase.APPROCHE):
                        replanifie(a)
                    continue
                if a.cible is not None and phase is control.Phase.ATTEINT:
                    # arrivé : 2 s de tenue devant une cible à lire ou à couvrir, le temps que le lecteur lise
                    if a.t_arrivee is None:
                        a.t_arrivee = clock.t
                    if a.cible.genre != "explorer" and clock.t - a.t_arrivee < LECTURE_S:
                        continue
                if a.cible is None and clock.t - a.t_decision < DECISION_REPOS_S:
                    continue
                if decide_fait:
                    continue
                a.t_decision = clock.t
                decide(a)
                decide_fait = True
            # 5. Arrêt : plus aucune cible pendant 30 s, plus aucun drone, part visée atteinte puis grâce écoulée,
            #    ou trop longtemps sans code nouveau
            vivants = [a for a in agents if a.vivant]
            if vivants and all(a.t_sans_cible is not None for a in vivants):
                if t_fin_candidats is None:
                    t_fin_candidats = clock.t
                elif clock.t - t_fin_candidats > FIN_S:
                    fin = "plus aucune cible"
                    break
            else:
                t_fin_candidats = None
            if not vivants:
                fin = "aucun drone vivant"
                break
            lus = len(carte.codes)
            if journal["codes_par_t"] and len(journal["codes_par_t"]) > 1 and lus > journal["codes_par_t"][-2][1]:
                t_dernier_code = clock.t        # au moins un code nouveau pendant ce cycle
            if args.part_arret > 0 and lus >= args.part_arret * codes_attendus:
                if t_part_atteinte is None:
                    t_part_atteinte = clock.t
                    evenement("part_atteinte", codes=lus, attendus=codes_attendus)
                elif clock.t - t_part_atteinte >= args.grace:
                    fin = f"inventaire a {lus / codes_attendus:.0%} et grace ecoulee"
                    break
            if args.sans_progres > 0 and lus > 0 and clock.t - t_dernier_code >= args.sans_progres:
                fin = f"sans code nouveau depuis {args.sans_progres:.0f} s"
                break
            # 6. Journal : instantané, durée du cycle, brouillon toutes les 60 s simulées, état affiché toutes les 30 s
            if args.instantanes and clock.t >= prochain_instantane:
                instantane()
                prochain_instantane = clock.t + args.instantanes
            cycles_ms.append((time.monotonic() - mur_cycle) * 1000)
            journal.setdefault("cycles_t", []).append([round(clock.t, 1), round(cycles_ms[-1])])
            if n_cycle % 300 == 0:
                brouillon = {"t_sim_s": round(clock.t, 1), "evenements": journal["evenements"],
                             "agents": [{"i": a.i, "vivant": a.vivant, "decisions": a.decisions,
                                         "trajectoire": a.obs.trajectoire, "inclinaisons": a.obs.inclinaisons}
                                        for a in agents]}
                (SORTIE / "brouillon.json").write_text(json.dumps(brouillon, default=_json_sur))
            if n_cycle % 150 == 0:
                r = carte.resume()
                etat = " ".join(f"d{a.i}:{'panne' if not a.vivant else (a.cible.genre if a.cible else 'libre')}"
                                for a in agents)
                print(f"  t={clock.t:6.1f} s  {r['part_connue']:.0%} connu  {r['codes_lus']:3d} codes  "
                      f"{r['pistes']:3d} pistes  {etat}  ({(time.monotonic() - mur0) / 60:.0f} min)")
        if fin is None:
            fin = "budget epuise"
    finally:
        # Fin, même après une erreur : index vidéo et journaux ArduPilot sauvés, drones mis en vol stationnaire,
        # puis carte, mission.json et image finale écrits
        if args.video:
            (SORTIE / "video" / "index.json").write_text(json.dumps(
                {"pas_s": 0.2, "codes_attendus": codes_attendus, "images": index_video}))
        sauve_journaux_ardupilot()
        for a in agents:
            a.pilot.hold()
        clock.pump(0.5)
        carte.sauve(SORTIE / "carte")
        cm = np.array(cycles_ms) if cycles_ms else np.zeros(1)
        journal.update({
            "fin": fin, "t_sim_s": round(clock.t, 1), "mur_min": round((time.monotonic() - mur0) / 60, 1),
            "cycles": {"n": int(len(cm)), "mediane_ms": round(float(np.median(cm)), 1),
                       "max_ms": round(float(cm.max()), 1), "lents_plus_de_500_ms": int((cm > 500).sum()),
                       "lents_plus_de_1_s": int((cm > 1000).sum())},
            "agents": [{"i": a.i, "vivant": a.vivant, "decisions": a.decisions, "attentes": a.attentes,
                        "compte": a.obs.compte,
                        "ms": {k: round(float(np.median(v)), 1) for k, v in a.obs.couts.items() if v},
                        "trajectoire": a.obs.trajectoire, "inclinaisons": a.obs.inclinaisons} for a in agents],
            "resume": carte.resume(),
            "verite": [{"code": t.tag_id, "position": list(map(float, t.position)),
                        "normale": list(map(float, t.normal)), "taille": round(float(t.size), 3)}
                       for t in tags],
            "racks": [{"prim": r.prim, "x": r.x_bounds, "y": r.y_bounds} for r in layout.racks],
        })
        (SORTIE / "mission.json").write_text(json.dumps(journal, indent=1, default=_json_sur))
        trajs = [[p[1:] for p in a.obs.trajectoire] for a in agents]
        cv2.imwrite(str(SORTIE / "carte_finale.png"), vue_annotee(carte, trajs, []))
        r = carte.resume()
        print(f"\nfin : {fin} a t={clock.t:.0f} s ; {r['codes_lus']} codes lus, {r['part_connue']:.0%} connu, "
              f"{r['pistes']} pistes restantes ; {(time.monotonic() - mur0) / 60:.0f} min de calcul")


# Lancement : une erreur est affichée en entier, et Isaac Sim est refermé dans tous les cas.
try:
    main()
    print("MISSION FINIE")
except Exception:
    traceback.print_exc()
finally:
    simulation_app.close()
