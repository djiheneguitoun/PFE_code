#!/usr/bin/env bash
# Lanceur des scripts Isaac Sim 5.1 de rl_inventory (même méthode que scripts/12_launch_aif_isaac_sim.sh).
# Active ~/isaac5_env, règle le rendu NVIDIA et l'affichage, puis lance le script avec le drapeau du pilote RTX.
#   avec fenêtre : bash launch.sh <script.py> [args]               (écran :1 par défaut, ou ISAAC_DISPLAY)
#   sans fenêtre : bash launch.sh <script.py> --headless [args]
# Le chemin du script est relatif au dossier courant (ex. depuis la racine : bash rl_inventory/launch.sh rl_inventory/tests/test_swarm.py --headless).
set -euo pipefail

# environnement Python d'Isaac Sim 5.1 ; 1er argument = script à lancer, le reste lui est transmis
VENV="${HOME}/isaac5_env"
SCRIPT="${1:?usage: bash launch.sh <script.py> [args...]}"
shift || true

# rendu par la carte NVIDIA (EGL), licence Omniverse acceptée, entrepôt par défaut
# (lu sur le serveur public de NVIDIA : accès Internet requis ; remplaçable en exportant AIF_FACTORY_USD)
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export OMNI_KIT_ACCEPT_EULA=YES
export AIF_FACTORY_USD="${AIF_FACTORY_USD:-http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd}"

# mode fenêtre (pas de --headless) : trouve l'autorisation X11 et l'écran, puis vérifie qu'il répond
if [[ "$*" != *--headless* ]]; then
    export XDG_RUNTIME_DIR="/run/user/$(id -u)"
    for xa in "${XDG_RUNTIME_DIR}/gdm/Xauthority" "${HOME}/.Xauthority"; do
        [[ -r "$xa" ]] && { export XAUTHORITY="$xa"; break; }
    done
    export DISPLAY="${ISAAC_DISPLAY:-:1}"
    echo "[launch] GUI DISPLAY=${DISPLAY}  XAUTHORITY=${XAUTHORITY:-?}"
    xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1 && echo "[launch] display ${DISPLAY} OK" || echo "[launch] ATTENTION: ${DISPLAY} ne répond pas"
fi

source "${VENV}/bin/activate"
# --kit_args désactive la vérification de version du pilote graphique (obligatoire sur la machine de simulation)
exec python "${SCRIPT}" --kit_args="--/rtx/verifyDriverVersion/enabled=false" "$@"
