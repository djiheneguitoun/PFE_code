"""Chronomètre les six lecteurs de QR sur les images déjà enregistrées du banc (sans simulateur).

Ajoute la colonne « temps par image » au tableau des lecteurs ; écrit temps_lecteurs.json.
Lancement : python temps_lecteurs.py   (400 premières images de images_optique/)
Options : --n 2000 (toute la campagne optique), --campagne 9019 (lit images_9019/).
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

# Les six lecteurs chronométrés (décodeurs de swarm_qr/perception.py) ; un lecteur absent est sauté.
LECTEURS = ("zxing", "zbar", "pyboof", "opencv", "opencv_aruco", "opencv_aruco_x3")

# --n : nombre d'images chronométrées ; --campagne : suffixe du dossier lu, images_<campagne>/.
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
    # En millisecondes : médiane, moyenne et 90e centile (9 images sur 10 sont décodées plus vite).
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
