#!/usr/bin/env bash
# Phase RL (rl_inventory) : compare la politique « aif » à un balayage « serpentine »
# (en lacets) sur les configurations de test, avec le seuil de lecture nominal (« gate ») et la graine 1000 (§ 7 de la conception).
# Chaque cas écrit son JSON dans results/baselines/ et peut être relancé seul.
# Usage : bash scripts/run_baseline_matrix.sh [aif|serpentine]   (sans argument : les deux politiques)
set -euo pipefail
# Se place à la racine du projet.
cd "$(dirname "$0")/.."

# Politiques à évaluer (1er argument, sinon aif et serpentine).
POLICIES=(${1:-aif serpentine})
# Script d'évaluation lancé via rl_inventory/launch.sh (Isaac Sim 5.1).
RUNNER="rl_inventory/swarmscan_map/baselines/run_baseline.py"
# Options communes : sans fenêtre, 16 environnements en parallèle, --rounds 2, graine 1000.
COMMON="--headless --num_envs 16 --rounds 2 --seed 1000"

# Pour chaque politique : nominal, pannes (failure1 à t = 0,25 / 0,5 / 0,75 ; failure2 à t = 0,5),
# bruit (noise, noise_hard), qr_minus / qr_plus, puis équipes de 2 et 4 drones.
for pol in "${POLICIES[@]}"; do
    bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario nominal
    for t in 0.25 0.5 0.75; do
        bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario failure1 --failure_t "$t" \
            --out "results/baselines/${pol}_failure1_t${t}_team3_seed1000.json"
    done
    bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario failure2 --failure_t 0.5
    bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario noise
    bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario noise_hard
    bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario qr_minus
    bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario qr_plus
    bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario nominal --team 2
    bash rl_inventory/launch.sh "$RUNNER" $COMMON --policy "$pol" --scenario nominal --team 4
done
echo "MATRICE TERMINÉE → results/baselines/"
