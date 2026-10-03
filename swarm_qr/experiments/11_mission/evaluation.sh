#!/bin/bash
# Évaluation finale du système (étape 5) : 4 vols de 3 drones (600 s simulées au plus), chacun avec vidéo et bilan.
# Cas : nominal (9033), panne du drone 1 à 200 s, entrepôt 9019, obstacle à 200 s.  Lancement : bash evaluation.sh
# Sorties dans 11_mission/eval_<cas>/ (un cas déjà fait est sauté).
set -u
ICI="$(cd "$(dirname "$0")" && pwd)"
# dossier swarm_qr : mission.py y est lancé
SQ="$ICI/../.."
# Python de l'environnement Isaac Sim (machine de simulation)
PY=~/isaac5_env/bin/python
# écran virtuel :1 (Isaac Sim, fenêtres SITL), sorties non tamponnées
export DISPLAY=:1 PYTHONUNBUFFERED=1
# poids du guide λ = 1 ; sans option --guide, aucun guide n'est chargé
OPT="--lam 1.0"
# préfixe des dossiers de sortie : eval_<cas>
TAG="eval"
# Vole un cas (coupé après 2 h), puis écrit son bilan (analyse.py) et ses vidéos (video.py, images effacées ensuite).
vole() {
  local nom="$1"; shift
  local sortie="$ICI/${TAG}_$nom"
  if [ -f "$sortie/mission.json" ]; then echo "=== $nom deja fait"; return; fi
  echo "=== $TAG $nom : depart $(date +%H:%M)"
  ( cd "$SQ" && timeout -s KILL 7200 $PY mission.py --drones 3 --budget 600 --detecteur auto --video $OPT "$@" --sortie "$sortie" \
      2>&1 | grep -vE 'gpu.foundation|PNG|ros2|Duplicate input|enough inputs' )
  ( cd "$SQ" && $PY experiments/11_mission/analyse.py --dossier "$sortie" )
  ( cd "$SQ" && $PY experiments/11_mission/video.py --dossier "$sortie" --sans-images )
  echo "=== $TAG $nom : fin $(date +%H:%M)"
}
vole nominal  --seed 9033
vole panne    --seed 9033 --panne 1:200
vole 9019     --seed 9019
# bloc de 1 × 1 × 2 m qui apparaît en (−4,96 ; 4,0) à 200 s ; le « = » est nécessaire à cause du signe moins
vole obstacle --seed 9033 --obstacle=-4.96,4.0,200
echo "=== EVALUATION FINIE $(date +%H:%M)"
