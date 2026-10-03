#!/usr/bin/env bash
# Phase 11 : lance 11_isaac_sim_drones.py (N drones Iris posés dans un entrepôt Isaac Sim + Pegasus).
# Vérifie d'abord l'écran X11 (sinon mode sans fenêtre), le GPU NVIDIA et les imports isaacsim / pegasus.
# Usage : bash scripts/11_launch_isaac_sim_drones.sh [N_DRONES]   (3 par défaut ; HEADLESS=1 force le mode sans fenêtre)
# Prérequis : environnement ~/isaac_sim_env créé par install_isaac_sim.sh (racine du projet).

set -euo pipefail

# Nombre de drones (1er argument, 3 par défaut).
N_DRONES=${1:-3}
# Environnement Python d'Isaac Sim et son script d'activation (créés par install_isaac_sim.sh).
VENV_DIR="${HOME}/isaac_sim_env"
ACTIVATE_SCRIPT="${VENV_DIR}/activate_isaac.sh"
# Racine du projet (dossier parent de scripts/).
WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"

# Affiche un message d'information.
log()  { echo "[INFO] $*"; }
# Affiche un avertissement.
warn() { echo "[WARN] $*"; }
# Affiche une erreur puis arrête le script (code 1).
die()  { echo "[ERROR] $*"; exit 1; }

# --- Écran : si DISPLAY ne répond pas, essaie les écrans X11 existants ; sinon mode sans fenêtre ---
if [[ -z "${HEADLESS:-}" ]]; then
    if xdpyinfo -display "${DISPLAY:-}" >/dev/null 2>&1; then
        HEADLESS=0
    else
        FOUND_DISPLAY=""
        for sock in /tmp/.X11-unix/X*; do
            d=":${sock##*/tmp/.X11-unix/X}"
            if xdpyinfo -display "$d" >/dev/null 2>&1; then
                warn "DISPLAY invalide, basculement vers $d"
                export DISPLAY="$d"
                FOUND_DISPLAY="$d"
                break
            fi
        done
        if [[ -n "$FOUND_DISPLAY" ]]; then
            HEADLESS=0
        else
            HEADLESS=1
            warn "Aucun display X11 détecté, mode headless forcé."
        fi
    fi
fi

# --- 1. Environnement Isaac Sim ---
if [[ -f "${ACTIVATE_SCRIPT}" ]]; then
    # shellcheck disable=SC1090
    source "${ACTIVATE_SCRIPT}"
else
    die "Environnement Isaac Sim introuvable (${ACTIVATE_SCRIPT}). Lancez d'abord ./install_isaac_sim.sh"
fi

# --- 2. Vérifier GPU NVIDIA ---
command -v nvidia-smi &>/dev/null || die "nvidia-smi introuvable. Vérifiez les drivers NVIDIA."
log "GPU : $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"

# --- 3. Vérifier les imports ---
python -c "import isaacsim" 2>/dev/null || die "Isaac Sim non importable."
python -c "import pegasus"  2>/dev/null || die "Pegasus non importable."

# --- 4. Lancer la simulation ---
HEADLESS_FLAG=""
if [[ "${HEADLESS}" == "1" ]]; then
    HEADLESS_FLAG="--headless"
fi
log "Lancement : ${N_DRONES} drone(s), mode $([ "${HEADLESS}" == "1" ] && echo headless || echo GUI)"

# Force le pilote graphique NVIDIA et accepte la licence d'Omniverse sans question.
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export OMNI_KIT_ACCEPT_EULA=YES

# shellcheck disable=SC2086
python "${WORKSPACE}/scripts/11_isaac_sim_drones.py" \
    --num-drones "${N_DRONES}" \
    ${HEADLESS_FLAG}
