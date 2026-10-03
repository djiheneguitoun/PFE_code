"""Test 2 (étape 1) : des graines différentes donnent-elles des entrepôts vraiment différents ?

Une graine par lancement : construit l'entrepôt, enregistre sa vue de dessus et la position de
ses racks ; --board assemble ensuite la planche des graines 1 à 6 et rend le verdict.
Lancé par run_all.sh ; à la main, depuis la racine du projet :
  $PY swarm_qr/experiments/02_variation/run.py --seed N     (N = 1 à 6, un lancement chacun)
  $PY swarm_qr/experiments/02_variation/run.py --board      (sans Isaac Sim)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent  # dossier du test : toutes les sorties y sont écrites
ROOT = HERE.parents[2]                  # racine du projet, ajoutée au chemin d'import (swarm_qr)
sys.path.insert(0, str(ROOT))

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--board", action="store_true")

SEEDS = (1, 2, 3, 4, 5, 6)  # graines assemblées dans la planche (celles que lance run_all.sh)


def build_board() -> int:
    """Assemble les vues des graines 1 à 6 en planche et mesure l'écart entre entrepôts ; écrit
    resultat.json, renvoie 0 si la variation suffit, 2 sinon, 1 s'il manque des vues."""
    import cv2
    import numpy as np

    from swarm_qr.experiments import _img

    tiles, rows = [], []
    for s in SEEDS:
        img = cv2.imread(str(HERE / f"vue_{s}.jpg"))
        meta_path = HERE / f"layout_{s}.json"
        if img is None or not meta_path.exists():
            continue
        m = json.loads(meta_path.read_text())
        xs = [round(r[1], 1) for r in m["racks"]]
        tiles.append(_img.label(img, f"graine {s} - racks X {xs} - {m['boxes']} cartons"))
        rows.append({"seed": s, "x": [r[1] for r in m["racks"]], "y": [r[2] for r in m["racks"]],
                     "boxes": m["boxes"]})

    if len(tiles) < 2:
        print("il manque des vues")
        return 1

    _img.board(tiles, HERE / "planche_variation.jpg", cols=3, cell=440)

    xs = np.array([r["x"] for r in rows])
    ys = np.array([r["y"] for r in rows])
    boxes = np.array([r["boxes"] for r in rows])
    std_x = float(xs.std(axis=0).mean())
    std_y = float(ys.std(axis=0).mean())

    # Deux entrepots sont differents si LA DISPOSITION diffère, pas si aucun rack ne coincide :
    # avec 6 graines et 3 racks, une coincidence isolee est attendue et sans consequence.
    # On mesure donc, pour chaque paire de graines, le deplacement moyen des trois racks.
    pair_moves = []
    for i in range(len(rows)):
        for j in range(i + 1, len(rows)):
            d = (np.abs(xs[i] - xs[j]).mean() + np.abs(ys[i] - ys[j]).mean()) / 2.0
            pair_moves.append((float(d), rows[i]["seed"], rows[j]["seed"]))
    worst, s_a, s_b = min(pair_moves)

    # Verdict : la paire la plus proche diffère de plus de 1 m en moyenne, et le nombre de
    # cartons varie de plus de 20 d'un entrepôt à l'autre.
    ok = worst > 1.0 and int(boxes.max() - boxes.min()) > 20
    print(f"deplacement moyen le plus faible : {worst:.2f} m  (graines {s_a} et {s_b})")
    print(f"ecart-type des positions X       : {std_x:.2f} m")
    print(f"ecart-type des positions Y       : {std_y:.2f} m")
    print(f"cartons : de {boxes.min()} a {boxes.max()}")
    print("VERDICT :", "VARIATION SUFFISANTE" if ok else "VARIATION INSUFFISANTE")
    (HERE / "resultat.json").write_text(
        json.dumps({"paire_la_plus_proche": {"graines": [s_a, s_b], "deplacement_moyen_m": worst},
                    "std_x": std_x, "std_y": std_y,
                    "boxes_min": int(boxes.min()), "boxes_max": int(boxes.max()),
                    "ok": bool(ok)}, indent=2)
    )
    return 0 if ok else 2


# --board ne lit que les fichiers déjà écrits : on le traite avant de démarrer Isaac Sim.
args, _ = parser.parse_known_args()
if args.board:
    raise SystemExit(build_board())

args, _ = parser.parse_known_args()

import sys as _sys  # noqa: E402

_sys.stdout.reconfigure(line_buffering=True)

from isaacsim import SimulationApp  # noqa: E402

# Isaac Sim doit démarrer avant tout import de omni.* et de swarm_qr.env.
simulation_app = SimulationApp(
    {"headless": True, "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"]}
)

import traceback  # noqa: E402

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

from swarm_qr.env import scene as scene_mod  # noqa: E402
from swarm_qr.env.layout import make_layout  # noqa: E402
from swarm_qr.experiments import _viz  # noqa: E402


def main() -> None:
    """Construit l'entrepôt de la graine (drones posés, sans SITL) et, après 40 pas de simulation,
    enregistre vue_<graine>.jpg et layout_<graine>.json (racks, cartons, QR)."""
    layout = make_layout(args.seed)
    scene = scene_mod.build(layout, with_sitl=False)
    overview = _viz.overview_camera()
    _viz.hide_roof(omni.usd.get_context().get_stage())

    scene.world.reset()
    scene.finalize()
    _viz.overview_init(overview)
    omni.timeline.get_timeline_interface().play()

    for _ in range(40):
        scene.world.step(render=True)

    _viz.save(_viz.to_bgr(overview.get_rgb()), HERE / f"vue_{args.seed}.jpg")
    (HERE / f"layout_{args.seed}.json").write_text(
        json.dumps(
            {
                "seed": layout.seed,
                "racks": [[r.prim, round(r.x, 4), round(r.y_min, 4)] for r in layout.racks],
                "boxes": len(scene.kept_boxes),
                "tags": len(scene.tags),
            },
            indent=2,
        )
    )
    print(f"graine {args.seed} : {len(scene.kept_boxes)} cartons")


try:
    main()
except Exception:
    traceback.print_exc()
finally:
    simulation_app.close()
