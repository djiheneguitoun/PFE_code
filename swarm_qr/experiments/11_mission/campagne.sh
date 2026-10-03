#!/usr/bin/env bash
# Campagne de mise au point de l'étape 5 (3 drones, 600 s simulées, détecteur branché) : bash campagne.sh <phase>
#   nominale (entrepôt 9033) | panne (idem, drone 1 en panne à 200 s) | autre (entrepôt 9019)
#   guidee (9019, guide smolvlm branché, λ = 1) | guide (banc hors ligne du guide sur les instantanés des 3 missions)
#   juge NOM (jugement seul, sans simulateur). Chaque mission écrit dans 11_mission/<phase>/.
set -u
# Python de l'environnement Isaac Sim (machine de simulation)
PY=/home/djihene_guitoun/isaac5_env/bin/python
ICI="$(cd "$(dirname "$0")" && pwd)"
# écran virtuel :1 (Isaac Sim, fenêtres SITL), sorties non tamponnées, ultralytics n'installe rien tout seul
export DISPLAY=:1 PYTHONUNBUFFERED=1 YOLO_AUTOINSTALL=false
cd "$ICI" || exit 1

# Vole une mission dans 11_mission/$1 (refus si un SITL tourne encore ; coupée après 3 h), puis la juge avec analyse.py.
vole() {
  local nom="$1"; shift
  pgrep -x arducopter >/dev/null && { echo "ATTENTION : un SITL tourne encore"; exit 1; }
  echo "=========== mission $nom ==========="
  timeout -s KILL 10800 "$PY" ../../mission.py --sortie "$ICI/$nom" "$@" 2>&1 | grep -vE "gpu.foundation|PNG|ros2"
  echo "--- mission $nom : code de sortie ${PIPESTATUS[0]}"
  sleep 5
  "$PY" analyse.py --dossier "$ICI/$nom"
}

case "${1:-}" in
  nominale) vole nominale --seed 9033 --drones 3 --budget 600 --detecteur auto ;;
  panne)    vole panne    --seed 9033 --drones 3 --budget 600 --detecteur auto --panne 1:200 ;;
  autre)    vole autre    --seed 9019 --drones 3 --budget 600 --detecteur auto ;;
  guidee)   vole guidee   --seed 9019 --drones 3 --budget 600 --detecteur auto --guide smolvlm --lam 1.0 ;;
  guide)    "$PY" ../12_guide/banc.py --missions "$ICI/nominale" "$ICI/panne" "$ICI/autre" --modeles smolvlm smolvlm-2b ;;
  juge)     "$PY" analyse.py --dossier "$ICI/${2:?nom}" ;;
  *) echo "phase inconnue : nominale | panne | autre | juge NOM"; exit 1 ;;
esac
echo "PHASE ${1} FINIE"
