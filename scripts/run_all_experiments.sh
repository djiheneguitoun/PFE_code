#!/usr/bin/env bash
# Étape 12 : lance la série d'expériences AIF, un run Isaac Sim par configuration (via 12_launch_aif_isaac_sim.sh).
# 4 axes : AIF / heuristique, centralisé / distribué, réseau (WiFi / 5G, cloud lent), résilience (drone tué, cloud coupé,
# liens coupés, obstacle). Chaque run crée logs/runs/run_<AAAAMMJJ_HHMMSS>_<tag>/ : JSON, README et 11 graphiques PNG.
# Usage : bash scripts/run_all_experiments.sh   (variables : N_DRONES=3, MAX_STEPS=80, HEADLESS=1, ONLY="tag1,tag2")
set -euo pipefail

# Dossiers : scripts/, racine du projet, lanceur d'un run et dossier des résultats.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
LAUNCH="$SCRIPT_DIR/12_launch_aif_isaac_sim.sh"
RUNS_DIR="$WORKSPACE/logs/runs"
mkdir -p "$RUNS_DIR"

# Réglages modifiables par variables d'environnement, par exemple :
#   N_DRONES=3 MAX_STEPS=80 bash scripts/run_all_experiments.sh
#   ONLY="aif_cent_baseline,aif_dist_baseline" bash scripts/run_all_experiments.sh   (ne lance que ces tags)
#   HEADLESS=1 bash scripts/run_all_experiments.sh                                    (sans fenêtre)
# MAX_STEPS : nombre maximal de steps AIF par run (un step = une décision par drone).
N_DRONES=${N_DRONES:-3}
MAX_STEPS=${MAX_STEPS:-80}
ONLY="${ONLY:-}"

# Lance le run nommé $1 (tag) avec les options qui suivent ; l'ignore s'il n'est pas dans ONLY ; une erreur n'arrête pas la série.
run_one() {
    local TAG="$1"; shift
    if [[ -n "$ONLY" ]] && ! [[ ",$ONLY," == *",$TAG,"* ]]; then
        echo "  [SKIP] $TAG (not in ONLY=$ONLY)"
        return 0
    fi
    echo
    echo "════════════════════════════════════════════════════════════════"
    echo "  RUN  : $TAG"
    echo "  Time : $(date '+%Y-%m-%d %H:%M:%S')"
    echo "  Flags: $*"
    echo "════════════════════════════════════════════════════════════════"
    "$LAUNCH" "$N_DRONES" --max-steps "$MAX_STEPS" \
        --run-tag "$TAG" "$@" || {
        echo "  [WARN] run $TAG returned non-zero; continuing."
    }
}

# ════════════════════════════════════════════════════════════════════
# Références (4 runs) : AIF ou heuristique × centralisé ou distribué, réseau WiFi
# ════════════════════════════════════════════════════════════════════
run_one aif_cent_baseline       --planner aif       --arch centralized --ns3 wifi
run_one aif_dist_baseline       --planner aif       --arch distributed --ns3 wifi
#run_one heur_cent_baseline      --planner heuristic --arch centralized --ns3 wifi
#run_one heur_dist_baseline      --planner heuristic --arch distributed --ns3 wifi

# ════════════════════════════════════════════════════════════════════
# Cloud chargé : aller-retour cloud de 1 500 ms au lieu de 500 ms (× 3), soit 2 steps de retard au lieu de 1
# ════════════════════════════════════════════════════════════════════
#run_one aif_cent_cloud_loaded   --planner aif       --arch centralized --ns3 wifi \
#                                --cloud-round-trip-ms 1500

# ════════════════════════════════════════════════════════════════════
# Résilience — un seul stresseur par run, déclenché au step 20
# ════════════════════════════════════════════════════════════════════
# Drone 0 mis hors service (il se pose) au step 20
run_one aif_cent_kill_d0_s20    --planner aif       --arch centralized --ns3 wifi \
                                --kill-drone-at-step 20 --kill-drone-id 0
#run_one heur_cent_kill_d0_s20   --planner heuristic --arch centralized --ns3 wifi \
#                                --kill-drone-at-step 20 --kill-drone-id 0

# Lien cloud coupé au step 20 : le centralisé bascule seul en décision locale (mode distribué)
run_one aif_cent_cut_cloud_s20  --planner aif       --arch centralized --ns3 wifi \
                                --cut-cloud-at-step 20


# Tous les liens drone ↔ drone coupés au step 20 (distribué, rayon de voisinage 12 m)
run_one aif_dist_cut_links_s20  --planner aif       --arch distributed --ns3 wifi \
                                --neighbor-radius-m 12 \
                                --cut-drone-link all --cut-drone-link-at-step 20


# Obstacle (cube rouge de 3 × 3 × 4 m) lâché au step 20 en x = 4,0 m, y = 3,5 m (coordonnées du monde)
run_one aif_cent_obstacle_s20   --planner aif       --arch centralized --ns3 wifi \
                                --drop-obstacle-at-step 20 --drop-obstacle-xy "4.0,3.5"

# ════════════════════════════════════════════════════════════════════
# Plusieurs stresseurs enchaînés : cloud coupé au step 15 puis drone 1 tué au step 30
# ════════════════════════════════════════════════════════════════════
#run_one aif_cent_full_chain     --planner aif       --arch centralized --ns3 wifi \
#                                --cut-cloud-at-step 15 \
#                                --kill-drone-at-step 30 --kill-drone-id 1

# ════════════════════════════════════════════════════════════════════
# Couche réseau alternative : 5G (ns-3 5G-LENA) au lieu du WiFi
# ════════════════════════════════════════════════════════════════════
#run_one aif_cent_ns3_5g         --planner aif       --arch centralized --ns3 5g

echo
echo "  Runs terminés → $RUNS_DIR"
echo "  Open the dashboard, tab 'Runs', to browse runs interactively."
