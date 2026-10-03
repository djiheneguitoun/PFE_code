#!/usr/bin/env bash
# Campagne de l'étape 7 (détecteur YOLO), une phase par appel : bash campagne.sh <phase>
#   rendu (images + cadres de 6 entrepôts d'entraînement et 2 scellés, Isaac Sim) | controle (planches + comparaison à l'étape 2)
#   entraine [variantes] (défaut n1024,n640 ; s1024 seulement si demandé) | banc (jugement, sans simulateur)
#   vol (patrouille de l'étape 4 avec le détecteur branché, Isaac Sim + SITL)
# Jamais deux phases GPU en même temps : le rendu et l'entraînement se partagent les 8 Go de la carte.

set -u
# Python de l'environnement Isaac Sim (machine de simulation)
PY=/home/djihene_guitoun/isaac5_env/bin/python
ICI="$(cd "$(dirname "$0")" && pwd)"
# écran virtuel :1 pour Isaac Sim, sorties non tamponnées, ultralytics n'installe rien tout seul
export DISPLAY=:1 PYTHONUNBUFFERED=1 YOLO_AUTOINSTALL=false
cd "$ICI" || exit 1

# Rend un entrepôt (graine $1, options suivantes passées à rendu.py), coupé après 1 h ; filtre le bruit d'Isaac Sim.
rendu_un() {
  local seed="$1"; shift
  echo "=========== rendu entrepot $seed ==========="
  timeout -s KILL 3600 "$PY" rendu.py --seed "$seed" "$@" 2>&1 | grep -vE "gpu.foundation|PNG|ros2|omni.kit.app._impl\] \[py stderr\]: $" 
  echo "--- entrepot $seed : code de sortie ${PIPESTATUS[0]}"
  sleep 3
}

case "${1:-}" in
  rendu)
    # 6 entrepôts d'entraînement à 500 images, puis les 2 scellés à 400 images + ré-annotation des images de l'étape 2
    for s in 0 1 2 3 4 5; do rendu_un "$s" --images 500; done
    rendu_un 9033 --images 400 --relabel optique,sans_qr,vol,traversee
    rendu_un 9019 --images 400 --relabel 9019
    ;;
  controle)
    "$PY" controle.py --planche jeu/rendu_0 jeu/rendu_3 jeu/rendu_9033 jeu/rendu_9019 jeu/etape2_optique jeu/etape2_traversee --par-jeu 4
    "$PY" controle.py --verifie
    ;;
  entraine)
    "$PY" entraine.py --variantes "${2:-n1024,n640}"
    ;;
  banc)
    "$PY" banc.py
    ;;
  vol)
    # le détecteur en vol : même patrouille que l'étape 4 (09_carte), réduite à une étagère et deux
    # allées, coupée après 2 h, puis jugée par l'analyse de l'étape 4 (pistes, coûts, sécurité) ; sorties dans vol/
    pgrep -x arducopter >/dev/null && { echo "ATTENTION : un SITL tourne encore"; exit 1; }
    timeout -s KILL 7200 "$PY" ../09_carte/banc.py --mode vol --etages 1 --allees 2 --detecteur auto --sortie "$ICI/vol" 2>&1 | grep -vE "gpu.foundation|PNG|ros2"
    echo "--- vol : code de sortie ${PIPESTATUS[0]}"
    "$PY" ../09_carte/analyse.py --dossier "$ICI/vol"
    ;;
  *)
    echo "phase inconnue : rendu | controle | entraine | banc"; exit 1;;
esac
echo "PHASE ${1} FINIE"
