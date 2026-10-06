"""Fabrique les avis du guide dans une copie des journaux des 4 vols de l'évaluation (tests of system).

Les vols ont été faits sans guide ; ce script écrit, dans ce dossier, leurs mission.json et brouillon.json
comme si `mission.py --guide entraine --lam 1` avait été lancé, en suivant les règles du code :
- demande_avis (mission.py) : chaque drone vivant demande un avis au plus toutes les 5 s (AVIS_S) ; la
  réponse est relevée au premier cycle 5 s après la demande, ce qui écrit un événement « avis », et une
  nouvelle demande part aussitôt ;
- la réponse est la bonne zone de l'instantané le plus récent (instantanes[].verite : la zone qui contient
  le plus de QR non lus), son côté est celui des faces de la zone (GuideEntraine.conseille, guide.py) ;
- decide (mission.py) : le champ « avis » de chaque décision est la phrase du dernier avis reçu par le drone ;
- Cerveau.note (planning.py) : +λ × 10 si l'origine de la cible est dans la zone, +λ × 5 si elle est aussi
  du côté conseillé (cible autre que « explorer ») ; la note des cibles des instantanés suit.
Les originaux ne sont pas modifiés. Lancer : python fabrique.py
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "tests of system"
VOLS = ("eval_nominal", "eval_panne", "eval_9019", "eval_obstacle")
GUIDE, LAM = "entraine", 1.0
AVIS_S, BONUS_ZONE, BONUS_COTE = 5.0, 10.0, 5.0          # mêmes valeurs que mission.py et planning.py
NOMS_COTES = {0: "est", 1: "ouest", 2: "nord", 3: "sud"}  # planning.NOMS_COTES
INVERSE = {nom: k for k, nom in NOMS_COTES.items()}
TOLERANCE_S = 0.5           # s : l'instantané est écrit en fin de cycle, l'horloge a pu avancer de quelques dixièmes
MARQUE = ("Donnees fabriquees : avis du guide ajoutes apres coup a un vol fait sans guide. "
          "Voir README.md du dossier fake data.")
# ordre des événements d'un même cycle : panne, chute, obstacle (étapes 2-3), puis par drone avis,
# abandon, sans_cible (étape 4), puis part_atteinte (étape 5)
ORDRE = {"obstacle": (0, 0), "panne": (0, 1), "chute": (0, 2), "avis": (1, 0), "abandon": (1, 1),
         "sans_cible": (1, 2), "part_atteinte": (2, 0)}


def phrase(z: dict) -> str:
    """Phrase de l'avis, écrite comme GuideEntraine.conseille."""
    reponse = f"ANSWER: {z['numero']}"
    return f"zone {z['numero']}" + (f", cote {z['cote']}" if z.get("cote") else "") + f" ({reponse[:20]!r})"


def instantane_a(instantanes: list[dict], t: float) -> dict | None:
    """Instantané le plus récent au temps t (ce que la carte proposait alors), ou None."""
    avant = [x for x in instantanes if x["t"] <= t + TOLERANCE_S]
    return avant[-1] if avant else None


def avis_du_vol(j: dict) -> tuple[list[dict], dict[int, list[tuple[float, dict]]]]:
    """Rejoue demande_avis cycle par cycle ; renvoie les événements « avis » et, par drone, les avis reçus."""
    morts = {e["drone"]: e["t"] for e in j["evenements"] if e["genre"] in ("panne", "chute")}
    cycles = [t for t, _ in j["codes_par_t"]]
    t_avis = {i: -1e9 for i in range(j["drones"])}
    en_cours: dict[int, dict] = {}
    evenements, recus = [], {i: [] for i in range(j["drones"])}
    for tc in cycles:
        for i in range(j["drones"]):
            if i in morts and tc >= morts[i]:
                continue                    # drone en panne : plus de demande
            if tc - t_avis[i] < AVIS_S:
                continue
            if i in en_cours:
                av = en_cours.pop(i)
                recus[i].append((tc, av))
                evenements.append({"t": tc, "genre": "avis", "drone": i, "zone": av["phrase"]})
            inst = instantane_a(j["instantanes"], tc)
            if inst is None or not inst["zones"]:
                continue                    # pas de zone : pas de demande, nouvel essai au cycle suivant
            z = next(z for z in inst["zones"] if z["numero"] == inst["verite"]["zone"])
            en_cours[i] = {"phrase": phrase(z), "centre": z["centre"], "rayon": z["rayon"],
                           "cote": INVERSE.get(z.get("cote"))}
            t_avis[i] = tc
    return evenements, recus


def avec_avis(d: dict, recus: list[tuple[float, dict]]) -> dict:
    """Copie de la décision avec l'avis en cours et la note augmentée du bonus du guide."""
    d = dict(d)
    avant = [av for t, av in recus if t <= d["t"]]
    if not avant:
        return d
    av = avant[-1]
    d["avis"] = av["phrase"]
    if np.hypot(d["origine"][0] - av["centre"][0], d["origine"][1] - av["centre"][1]) <= av["rayon"]:
        bonus = LAM * BONUS_ZONE
        if av["cote"] is not None and d["cote"] == av["cote"] and d["genre"] != "explorer":
            bonus += LAM * BONUS_COTE
        d["note"] = round(d["note"] + bonus, 1)
    return d


def fusionne(reels: list[dict], avis: list[dict], t_max: float) -> list[dict]:
    """Événements réels et avis jusqu'à t_max, dans l'ordre où mission.py les écrit."""
    tous = [e for e in reels if e["t"] <= t_max] + [e for e in avis if e["t"] <= t_max]
    return sorted(tous, key=lambda e: (e["t"], ORDRE.get(e["genre"], (1, 3))[0], e.get("drone") or 0,
                                       ORDRE.get(e["genre"], (1, 3))[1]))


def fabrique(vol: str) -> dict:
    """Écrit mission.json et brouillon.json du vol avec les avis ; renvoie un petit bilan."""
    j = json.loads((SOURCE / vol / "mission.json").read_text(encoding="utf-8"))
    b = json.loads((SOURCE / vol / "brouillon.json").read_text(encoding="utf-8"))
    evts, recus = avis_du_vol(j)

    j["guide"], j["lam"] = GUIDE, LAM
    j["evenements"] = fusionne(j["evenements"], evts, float("inf"))
    avant = {(a["i"], d["t"]): d["note"] for a in j["agents"] for d in a["decisions"]}
    nouvelles = {}
    for a in j["agents"]:
        a["decisions"] = [avec_avis(d, recus[a["i"]]) for d in a["decisions"]]
        nouvelles.update({(a["i"], d["t"]): d for d in a["decisions"]})
    # note de la cible en cours dans les instantanés : celle de la décision qui l'a choisie
    for inst in j["instantanes"]:
        for dr in inst["drones"]:
            c = dr.get("cible")
            d = None if c is None else nouvelles.get((dr["i"], round(inst["t"] - c["depuis"], 1)))
            if d is not None and d["origine"] == c["origine"]:
                c["note"] = d["note"]
    # brouillon : mêmes règles, événements jusqu'à son heure d'écriture
    b["evenements"] = fusionne(b["evenements"], evts, b["t_sim_s"])
    for a in b["agents"]:
        a["decisions"] = [avec_avis(d, recus[a["i"]]) for d in a["decisions"]]

    (HERE / vol).mkdir(exist_ok=True)
    (HERE / vol / "mission.json").write_text(json.dumps({"_donnees_fabriquees": MARQUE, **j}, indent=1))
    (HERE / vol / "brouillon.json").write_text(json.dumps({"_donnees_fabriquees": MARQUE, **b}))
    decs = [(a["i"], d) for a in j["agents"] for d in a["decisions"]]
    return {"vol": vol, "evenements_avis": len(evts), "decisions": len(decs),
            "decisions_avec_avis": sum(d["avis"] is not None for _, d in decs),
            "decisions_avec_bonus": sum(d["note"] != avant[(i, d["t"])] for i, d in decs)}


if __name__ == "__main__":
    for v in VOLS:
        print(fabrique(v))
