"""Analyse de l'étape 2, sans simulateur : relit les images du banc avec six lecteurs de QR.

Calcule les taux de lecture selon la distance, l'angle et la vitesse ; écrit resultats.json et
enveloppe.png. S'arrête si la distance vue dans les images ne colle pas aux poses (écart > 8 cm).
Lancement : python analyse.py   (--rapide : zxing seul, pour vérifier la chaîne)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2]))

from swarm_qr import perception as P  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--rapide", action="store_true")
args = parser.parse_args()

# Les six lecteurs comparés (noms des décodeurs de swarm_qr/perception.py) ; --rapide : zxing seul.
LECTEURS = ("zxing",) if args.rapide else (
    "zxing", "zbar", "pyboof", "opencv_aruco", "opencv_aruco_x3", "opencv")
REFERENCE = "zxing"          # lecteur retenu : sert aux contrôles et aux campagnes à un seul lecteur
SEUIL = 0.90                 # une tranche est « fiable » si au moins 90 % de ses images sont lues
INTEGRITE_MAX_CM = 8.0       # cm : écart médian image/pose au-delà duquel l'association est fausse (arrêt)

BORNES_D = [0.4, 0.6, 0.8, 1.0, 1.25, 1.5, 1.8, 2.2, 2.7, 3.3, 4.0, 5.0, 6.5, 8.1]   # m : tranches de distance
BORNES_A = [0, 10, 20, 30, 40, 50, 60, 76]                                          # degrés : tranches d'angle


# ---------------------------------------------------------------- briques

def wilson(succes: int, total: int, z: float = 1.96):
    """Renvoie (proportion, borne basse, borne haute) de l'intervalle de confiance à 95 % de Wilson,
    choisi car l'intervalle naïf est faux quand il y a peu d'images."""
    if total == 0:
        return 0.0, 0.0, 0.0
    p = succes / total
    d = 1.0 + z * z / total
    centre = (p + z * z / (2 * total)) / d
    demi = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / d
    return p, max(0.0, centre - demi), min(1.0, centre + demi)


def charge(nom: str):
    """Charge meta_<nom>.json et poses_<nom>.jsonl ; renvoie (meta, lignes), ou (None, []) s'ils manquent."""
    manifeste = HERE / f"poses_{nom}.jsonl"
    meta = HERE / f"meta_{nom}.json"
    if not manifeste.exists() or not meta.exists():
        return None, []
    lignes = [json.loads(l) for l in manifeste.read_text().splitlines() if l.strip()]
    return json.loads(meta.read_text()), lignes


def projette_panneau(m, K):
    """Renvoie (centre, côté) en pixels du code visé, prévus par la pose vraie, ou None s'il est hors
    champ ; sans cela, n'importe quel carré sombre (rack, carton, ombre) compterait comme repéré."""
    from scipy.spatial.transform import Rotation

    w, x, y, z = m["cam_quat"]
    R = Rotation.from_quat([x, y, z, w]).as_matrix()
    centre = np.array(m["tag_pos"], float)
    n = np.array(m["tag_normale"], float)
    t = np.cross([0.0, 0.0, 1.0], n)
    t /= max(np.linalg.norm(t), 1e-9)
    demi = P.taille_code(m["panneau_m"]) / 2.0
    coins = [centre + a * demi * t + b * demi * np.array([0.0, 0.0, 1.0])
             for a, b in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
    uv = []
    for pt in [centre] + coins:
        c = R.T @ (pt - np.array(m["cam_pos"], float))
        if c[0] <= 0.05:
            return None
        uv.append([K[0, 2] - K[0, 0] * c[1] / c[0], K[1, 2] - K[1, 1] * c[2] / c[0]])
    uv = np.array(uv)
    if not (0 <= uv[0, 0] < 2 * K[0, 2] and 0 <= uv[0, 1] < 2 * K[1, 2]):
        return None
    cotes = [np.linalg.norm(uv[1 + (i + 1) % 4] - uv[1 + i]) for i in range(4)]
    return uv[0], float(np.mean(cotes))


def examine(nom: str, lecteurs=LECTEURS):
    """Relit chaque image de la campagne avec chaque lecteur ; renvoie (meta, lignes) avec, par image
    et par lecteur : lu ou non, lectures fantômes, erreur de distance (m), motif repéré au bon endroit."""
    meta, lignes = charge(nom)
    if meta is None:
        return None, []
    K = np.array(meta["K"], float)
    dossier = HERE / f"images_{nom}"
    out = []
    for k, m in enumerate(lignes):
        if not m.get("image"):
            continue
        img = cv2.imread(str(dossier / m["image"]))
        if img is None:
            continue
        gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        attendu = projette_panneau(m, K)
        repere = False
        if attendu is not None:
            centre_px, taille_px = attendu
            for q in P.repere_motifs(img):
                if np.linalg.norm(q.mean(axis=0) - centre_px) < max(taille_px, 12.0):
                    repere = True
                    break
        a = abs(m["alpha_deg"])
        base = {"run": m.get("run", ""), "indice": m.get("indice"),
                "D_m": m["D_m"], "alpha_deg": a,
                "D_eff_m": m["D_m"] / max(math.cos(math.radians(a)), 1e-6),
                "vitesse_ms": m.get("vitesse_ms", 0.0), "repere": repere,
                "dans_champ": attendu is not None}
        for lect in lecteurs:
            trouves = P.DECODEURS[lect](gris)
            cible = [c for t, c in trouves if t == m["tag"]]
            ligne = dict(base, lecteur=lect, lu=bool(cible),
                         fantomes=len([t for t, _ in trouves if t != m["tag"]]))
            if cible:
                t3, _ = P.pose_3d(cible[0], P.taille_code(m["panneau_m"]), K,
                                  np.array(m["tag_normale"], float))
                if t3 is not None:
                    ligne["err_dist_m"] = float(np.linalg.norm(t3)) - m["D_m"]
            out.append(ligne)
        if (k + 1) % 200 == 0:
            print(f"  {k + 1}/{len(lignes)} images")
    return meta, out


def controle_integrite(nom: str, lignes) -> float:
    """Vérifie que la distance lue dans l'image colle à la pose enregistrée (zxing, angle ≤ 30°) ;
    renvoie l'écart médian (cm) et arrête l'analyse s'il dépasse 8 cm."""
    errs = [abs(l["err_dist_m"]) for l in lignes
            if l["lecteur"] == REFERENCE and "err_dist_m" in l and l["alpha_deg"] <= 30]
    if not errs:
        return 0.0
    med = 100 * float(np.median(errs))
    print(f"[INTEGRITE] {nom} : ecart image-pose median {med:.1f} cm sur {len(errs)} lectures")
    if med > INTEGRITE_MAX_CM:
        raise RuntimeError(f"{nom} : ecart {med:.1f} cm > {INTEGRITE_MAX_CM} cm, "
                           "association image-pose fausse")
    return med


def courbe(lignes, cle, bornes, lecteur, filtre=None):
    """Découpe les images en tranches de `cle` (distance, angle…) ; renvoie, par tranche : nombre
    d'images, taux de lecture avec son intervalle à 95 %, et taux de repérage."""
    sel = [l for l in lignes if l["lecteur"] == lecteur and (filtre is None or filtre(l))]
    out = []
    for a, b in zip(bornes[:-1], bornes[1:]):
        cases = [l for l in sel if a <= l[cle] < b]
        if not cases:
            continue
        p, lo, hi = wilson(sum(l["lu"] for l in cases), len(cases))
        pr, _, _ = wilson(sum(l["repere"] for l in cases), len(cases))
        out.append({"min": round(a, 3), "max": round(b, 3), "centre": round((a + b) / 2, 3),
                    "n": len(cases), "p_lu": round(p, 4), "ic_bas": round(lo, 4),
                    "ic_haut": round(hi, 4), "p_repere": round(pr, 4)})
    return out


def limite(pts, seuil=SEUIL):
    """Renvoie la portée : centre de la tranche la plus lointaine lue à 90 % ou plus (None si aucune)."""
    bons = [c["centre"] for c in pts if c["p_lu"] >= seuil]
    return max(bons) if bons else None


# ---------------------------------------------------------------- volets

def volet_optique(res):
    """Analyse la campagne optique pour les six lecteurs : zone utile (≤ 2 m et ≤ 30°), portée,
    erreur de distance et les trois courbes (distance, angle, distance apparente)."""
    meta, lignes = examine("optique")
    if meta is None:
        print("pas de campagne optique")
        return
    print(f"banc optique : {meta['images']} images")
    controle_integrite("optique", lignes)
    # Filtres : la courbe en distance ne garde que les vues de face (≤ 20°),
    # la courbe en angle que les vues de près (≤ 2 m).
    de_face = lambda l: l["alpha_deg"] <= 20.0
    de_pres = lambda l: l["D_m"] <= 2.0
    res["optique"] = {"images": meta["images"], "retard_mesure": meta.get("retard_mesure"),
                      "lecteurs": {}}
    for lect in LECTEURS:
        sel = [l for l in lignes if l["lecteur"] == lect]
        zone = [l for l in sel if l["D_m"] <= 2.0 and l["alpha_deg"] <= 30]
        p, lo, hi = wilson(sum(l["lu"] for l in zone), len(zone))
        errs = [abs(l["err_dist_m"]) for l in sel if "err_dist_m" in l and l["alpha_deg"] <= 30]
        c_eff = courbe(lignes, "D_eff_m", BORNES_D, lect)
        res["optique"]["lecteurs"][lect] = {
            "zone_utile": round(p, 4), "ic": [round(lo, 4), round(hi, 4)],
            "portee_apparente_m": limite(c_eff),
            "err_mediane_cm": round(100 * float(np.median(errs)), 2) if errs else None,
            "err_p90_cm": round(100 * float(np.percentile(errs, 90)), 2) if errs else None,
            "courbe_distance": courbe(lignes, "D_m", BORNES_D, lect, de_face),
            "courbe_angle": courbe(lignes, "alpha_deg", BORNES_A, lect, de_pres),
            "courbe_apparente": c_eff,
        }


def volet_sans_qr(res):
    """Compte, dans l'entrepôt sans QR, les lectures fantômes de chaque lecteur et le faux repérage."""
    meta, lignes = examine("sans_qr")
    if meta is None:
        return
    print(f"\nsans QR : {meta['images']} images")
    bloc = {"images": meta["images"], "lectures_fantomes": {}}
    for lect in LECTEURS:
        sel = [l for l in lignes if l["lecteur"] == lect]
        bloc["lectures_fantomes"][lect] = sum(l["lu"] or l["fantomes"] > 0 for l in sel)
    rep = [l for l in lignes if l["lecteur"] == REFERENCE and l["dans_champ"]]
    p, lo, hi = wilson(sum(l["repere"] for l in rep), len(rep))
    bloc["faux_reperage"] = {"taux": round(p, 4), "ic": [round(lo, 4), round(hi, 4)],
                             "n": len(rep)}
    res["sans_qr"] = bloc


def volet_second_entrepot(res):
    """Analyse la campagne de l'entrepôt 9019 avec zxing : zone utile et courbe en distance apparente."""
    meta, lignes = examine("9019", lecteurs=(REFERENCE,))
    if meta is None:
        return
    print(f"\nsecond entrepot : {meta['images']} images")
    controle_integrite("9019", lignes)
    zone = [l for l in lignes if l["D_m"] <= 2.0 and l["alpha_deg"] <= 30]
    p, lo, hi = wilson(sum(l["lu"] for l in zone), len(zone))
    res["second_entrepot"] = {"images": meta["images"], "zone_utile": round(p, 4),
                              "ic": [round(lo, 4), round(hi, 4)],
                              "courbe_apparente": courbe(lignes, "D_eff_m", BORNES_D, REFERENCE)}


def volet_vol(res):
    """Analyse les images du drone en vol stationnaire avec zxing : courbe en distance apparente."""
    meta, lignes = examine("vol", lecteurs=(REFERENCE,))
    if meta is None:
        return
    print(f"\nvol, poses tenues : {meta['images']} images")
    controle_integrite("vol", lignes)
    res["vol"] = {"images": meta["images"],
                  "courbe_apparente": courbe(lignes, "D_eff_m", BORNES_D, REFERENCE)}


def volet_traversee(res):
    """Analyse les traversées : retrouve le retard constant image/pose (en images), puis compte, par
    vitesse, les images lues parmi celles où le panneau est entièrement dans le cadre."""
    meta, lignes_brutes = charge("traversee")
    if meta is None:
        return
    K = np.array(meta["K"], float)
    print(f"\ntraversees : {len(lignes_brutes)} pas de capture")

    par_run: dict = {}
    for m in lignes_brutes:
        par_run.setdefault(m["run"], {})[m["indice"]] = m
    decode: dict = {}
    for m in lignes_brutes:
        if not m.get("image"):
            continue
        img = cv2.imread(str(HERE / "images_traversee" / m["image"]))
        if img is None:
            continue
        gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cible = [c for t, c in P.DECODEURS[REFERENCE](gris) if t == m["tag"]]
        d_opt = None
        if cible:
            t3, _ = P.pose_3d(cible[0], P.taille_code(m["panneau_m"]), K,
                              np.array(m["tag_normale"], float))
            if t3 is not None:
                d_opt = float(np.linalg.norm(t3))
        decode[(m["run"], m["indice"])] = (bool(cible), d_opt)

    def dist_pose(run, idx):
        """Renvoie la distance caméra-panneau (m) notée pour l'image `idx` de la traversée `run`, ou None."""
        m = par_run.get(run, {}).get(idx)
        if m is None:
            return None
        return float(np.linalg.norm(np.array(m["cam_pos"]) - np.array(m["tag_pos"])))

    # Essaie un retard de 0 à 8 images et garde celui qui aligne le mieux distance vue et distance notée.
    candidats = []
    for k in range(0, 9):
        errs = [abs(dist_pose(r, i - k) - d) for (r, i), (lu, d) in decode.items()
                if d is not None and dist_pose(r, i - k) is not None]
        if len(errs) >= 10:
            candidats.append((float(np.median(errs)), k))
    if not candidats:
        res["traversee"] = {"erreur": "pas assez de lectures pour calibrer le retard"}
        return
    residu, k = min(candidats)
    print(f"  retard image-pose : {k} images, residu {100 * residu:.1f} cm")
    if 100 * residu > INTEGRITE_MAX_CM:
        raise RuntimeError(f"traversee : residu {100 * residu:.1f} cm, association fausse")

    # Le dénominateur : les images où le panneau est géométriquement DANS le cadre, calculé
    # depuis la pose corrigée du retard. En longeant le rack, le panneau n'est visible que
    # pendant un court tronçon ; compter les autres images compterait des échecs sur des
    # photos où le code n'apparaît pas.
    stats: dict = {}
    for m in lignes_brutes:
        ref = par_run[m["run"]].get(m["indice"] - k)
        if ref is None:
            continue
        proj = projette_panneau(ref, K)
        if proj is None:
            continue
        (cx, cy), taille = proj
        if not (taille / 2 <= cx <= 2 * K[0, 2] - taille / 2
                and taille / 2 <= cy <= 2 * K[1, 2] - taille / 2):
            continue
        lu = decode.get((m["run"], m["indice"]), (False, None))[0]
        s = stats.setdefault(m["vitesse_ms"], [0, 0])
        s[1] += 1
        s[0] += int(lu)
    bloc = {"retard_images": k, "residu_cm": round(100 * residu, 1), "vitesses": {}}
    for v, (lus, tot) in sorted(stats.items()):
        p, lo, hi = wilson(lus, tot)
        bloc["vitesses"][f"{v}"] = {"n": tot, "p_lu": round(p, 4),
                                    "ic": [round(lo, 4), round(hi, 4)]}
        print(f"  {v:.1f} m/s : {lus}/{tot} lues = {p:.1%} [{lo:.0%}-{hi:.0%}]")
    res["traversee"] = bloc


# ---------------------------------------------------------------- sorties

def figure(res):
    """Trace enveloppe.png : taux de lecture par lecteur selon la distance, l'angle et la distance apparente."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    opt = res.get("optique", {}).get("lecteurs", {})
    if not opt:
        return
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    for ax, cle, titre, xlabel in (
            (axes[0], "courbe_distance", "En distance (vue de face)", "distance (m)"),
            (axes[1], "courbe_angle", "En angle (a moins de 2 m)", "angle (degres)"),
            (axes[2], "courbe_apparente", "Unifiee : distance apparente",
             "distance / cos(angle)  (m)")):
        for lect, d in opt.items():
            pts = d.get(cle) or []
            if pts:
                ax.plot([p["centre"] for p in pts], [p["p_lu"] for p in pts],
                        marker="o", ms=3, label=lect)
        ref = opt.get(REFERENCE, {}).get(cle) or []
        if ax is axes[0] and ref:
            ax.plot([p["centre"] for p in ref], [p["p_repere"] for p in ref],
                    "k--", lw=1, label="reperage (sans lire)")
        ax.axhline(SEUIL, color="r", ls=":", lw=1)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("probabilite de lire, par image")
        ax.set_title(titre)
        ax.set_ylim(-0.02, 1.02)
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(HERE / "enveloppe.png", dpi=110)
    print(f"\nfigure : enveloppe.png")


def resume(res):
    """Affiche le bilan : zone utile, portée, erreur de chaque lecteur, lectures fantômes, second entrepôt."""
    opt = res.get("optique", {}).get("lecteurs", {})
    print("\n" + "=" * 70)
    print(f"{'lecteur':<18}{'zone utile':>11}{'portee apparente':>18}{'err distance':>14}")
    for lect, d in opt.items():
        pa = f"{d['portee_apparente_m']:.2f} m" if d["portee_apparente_m"] else "-"
        er = f"{d['err_mediane_cm']:.1f} cm" if d["err_mediane_cm"] is not None else "-"
        print(f"{lect:<18}{d['zone_utile']:>10.1%}{pa:>18}{er:>14}")
    print("=" * 70)
    sq = res.get("sans_qr")
    if sq:
        for lect, n in sq["lectures_fantomes"].items():
            print(f"lectures fantomes {lect:<15}: {n}/{sq['images']}")
        fr = sq["faux_reperage"]
        print(f"faux reperage sur la face visee : {fr['taux']:.1%} sur {fr['n']} images")
    se = res.get("second_entrepot")
    if se:
        print(f"second entrepot : zone utile {se['zone_utile']:.1%} sur {se['images']} images")


def main():
    """Lance les cinq volets, écrit resultats.json, trace enveloppe.png et affiche le bilan."""
    res = {}
    volet_optique(res)
    volet_sans_qr(res)
    volet_second_entrepot(res)
    volet_vol(res)
    volet_traversee(res)
    (HERE / "resultats.json").write_text(json.dumps(res, indent=2))
    figure(res)
    resume(res)


if __name__ == "__main__":
    main()
