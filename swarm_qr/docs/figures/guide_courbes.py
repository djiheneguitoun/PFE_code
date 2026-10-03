"""Fabrique fig_guide.png : part des 72 cas jamais vus de l'étape 8 où chaque méthode choisit la bonne
zone (zone la plus proche, planificateur géométrique, modèle avant et après l'entraînement LoRA).
Chiffres du planificateur et du modèle relus dans experiments/12_guide/resultats_entrainement.json.
    python guide_courbes.py
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

EXP = Path(__file__).resolve().parents[2] / "experiments"   # swarm_qr/experiments
ICI = Path(__file__).resolve().parent                        # la figure est écrite ici
ENCRE, DOUCE = "#16201C", "#5F6B66"                          # couleurs du texte
REFERENCE, AVANT, GUIDE = "#98A4A0", "#8FA0DB", "#364FC7"    # couleurs des barres
PLUS_PROCHE = 0.25                       # part juste de la règle « zone la plus proche » (étape 8)

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10.5, "axes.edgecolor": "#C9D3CF",
    "axes.labelcolor": ENCRE, "text.color": ENCRE, "xtick.color": DOUCE, "ytick.color": DOUCE,
    "axes.grid": True, "grid.color": "#E6EBE9", "grid.linewidth": 0.8, "figure.facecolor": "white",
})

r = json.loads((EXP / "12_guide" / "resultats_entrainement.json").read_text(encoding="utf-8"))
cas = r["apres"]["cas"]
barres = [("the nearest zone", PLUS_PROCHE, REFERENCE),
          ("the geometric planner", r["apres"]["geometrie"], REFERENCE),
          ("the model before specialisation", r["avant"]["justes"], AVANT),
          ("the specialised guide", r["apres"]["justes"], GUIDE)]

fig, ax = plt.subplots(figsize=(7.6, 3.2))
ax.xaxis.grid(True)
ax.yaxis.grid(False)
y = range(len(barres))
ax.barh(list(y), [b[1] for b in barres], color=[b[2] for b in barres], height=0.62, zorder=3)
for i, (_, v, c) in enumerate(barres):
    ax.text(v + 0.012, i, f"{v:.0%}".replace("%", " %"), va="center", fontsize=11, color=c, fontweight="semibold")

ax.set_yticks(list(y), [b[0] for b in barres])
ax.invert_yaxis()
ax.set_xlim(0, 1.12)
ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0], ["0", "25 %", "50 %", "75 %", "100 %"])
ax.set_xlabel(f"the most useful zone is chosen, on {cas} unseen cases")
ax.text(0.985, 0.94, f"specialisation wins {r['gagnes']} cases and loses {r['perdus']}",
        transform=ax.transAxes, ha="right", va="top", fontsize=9.5, color=ENCRE,
        bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="#C9D3CF", lw=0.9))
fig.tight_layout()
fig.savefig(ICI / "fig_guide.png", dpi=200)
print(f"fig_guide.png : {cas} cas, {len(barres)} barres")
