"""Temps de décodage des six lecteurs, sur les images déjà enregistrées du banc.

Complète le tableau de comparaison des lecteurs avec une colonne « temps par image ».
Aucun simulateur : on relit les images de `images_optique/`.

  temps_lecteurs.py             400 images
  temps_lecteurs.py --n 2000    toutes les images de la campagne optique
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2]))

from swarm_qr import perception as P  # noqa: E402

LECTEURS = ("zxing", "zbar", "pyboof", "opencv", "opencv_aruco", "opencv_aruco_x3")

parser = argparse.ArgumentParser()
parser.add_argument("--n", type=int, default=400)
parser.add_argument("--campagne", default="optique")
args = parser.parse_args()

dossier = HERE / f"images_{args.campagne}"
fichiers = sorted(dossier.glob("*.jpg"))[: args.n]
if not fichiers:
    sys.exit(f"aucune image dans {dossier}")

# Les images sont chargées une fois pour toutes : on chronomètre le décodage, pas le disque.
images = []
for f in fichiers:
    img = cv2.imread(str(f))
    if img is not None:
        images.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
print(f"{len(images)} images, {len(LECTEURS)} lecteurs")

resultats = {}
for nom in LECTEURS:
    fn = P.DECODEURS[nom]
    try:
        fn(images[0])            # mise en route (chargement de la bibliothèque, JVM)
    except Exception as e:
        print(f"{nom:16s} indisponible ({type(e).__name__})")
        continue
    temps = []
    for g in images:
        t0 = time.perf_counter()
        try:
            fn(g)
        except Exception:
            pass
        temps.append((time.perf_counter() - t0) * 1000.0)
    resultats[nom] = {
        "images": len(temps),
        "median_ms": round(statistics.median(temps), 1),
        "moyen_ms": round(statistics.fmean(temps), 1),
        "p90_ms": round(sorted(temps)[int(0.9 * len(temps))], 1),
    }
    print(f"{nom:16s} médiane {resultats[nom]['median_ms']:6.1f} ms   "
          f"moyenne {resultats[nom]['moyen_ms']:6.1f} ms")

(HERE / "temps_lecteurs.json").write_text(json.dumps(resultats, indent=2))

print("\n| lecteur | temps par image |")
print("|---|---|")
for nom, r in sorted(resultats.items(), key=lambda kv: kv[1]["median_ms"]):
    print(f"| {nom} | {r['median_ms']:.0f} ms |")
