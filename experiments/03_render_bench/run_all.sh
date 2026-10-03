#!/usr/bin/env bash
# Mesure 03 : balayage du débit de rendu. Lance render_bench.py pour 7 configurations,
# un processus Isaac par configuration (le simulateur ne sait pas reconstruire une scène en cours de route).
#
#   bash experiments/03_render_bench/run_all.sh
#
# Résultats dans resultats.csv. Compter ~40 s de démarrage d'Isaac par configuration (~7 min en tout).
# `timeout -s KILL` est indispensable : Isaac ignore le signal d'arrêt normal.

# Erreur si une variable n'est pas définie (pas de -e : une configuration ratée n'arrête pas le balayage)
set -u
# Se place à la racine du projet (deux dossiers au-dessus de ce script)
cd "$(dirname "$0")/../.."
# Python de l'environnement Isaac Sim 5.1 + Isaac Lab
PY="$HOME/isaac5_env/bin/python"
SCRIPT="experiments/03_render_bench/render_bench.py"
CSV="experiments/03_render_bench/resultats.csv"
# Option d'Isaac qui fait taire l'avertissement de version du pilote NVIDIA (sans effet sur le rendu)
KIT="--kit_args=--/rtx/verifyDriverVersion/enabled=false"

# Repart d'un fichier de résultats vide
rm -f "$CSV"

# Une configuration par ligne : (environnements, caméras par environnement, côté de l'image en px)
CONFIGS="
1 1 64
1 2 64
3 2 64
8 2 64
16 2 64
32 2 64
8 2 128
"

# Pour chaque configuration : Isaac est tué au bout de 300 s ; seules les lignes [VÉRIF], [RESULTAT]
# et les erreurs sont affichées
echo "$CONFIGS" | while read -r envs cams res; do
  # saute les lignes vides de la liste
  [ -z "$envs" ] && continue
  echo "--- ${envs} envs x ${cams} cams @ ${res}px ---"
  PYTHONUNBUFFERED=1 timeout -s KILL 300 "$PY" "$SCRIPT" \
      --envs "$envs" --cams "$cams" --res "$res" --steps 60 "$KIT" 2>&1 \
    | grep --line-buffered -E "^\[VÉRIF\]|^\[RESULTAT\]|Traceback|Error:" \
    || echo "  BLOQUÉ ou échec (tué à 300 s)"
done

echo
echo "=== Récapitulatif ==="
# Le CSV mis en colonnes (ou affiché brut si la commande `column` manque)
column -s, -t < "$CSV" 2>/dev/null || cat "$CSV"
