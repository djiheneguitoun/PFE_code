#!/bin/bash
# Vole la méthode de référence (Pore et al., Symmetry 2026) sur les 4 cas de l'évaluation du système,
# même protocole (3 drones, 600 s simulées au plus), puis assemble les vidéos de chaque vol.
# Lancement : bash swarm_qr/experiments/13_pore/campagne.sh   (30 à 50 min de calcul par vol)
# Le jugement n'est pas inclus : lancer ensuite analyse.py sur chaque dossier pore_*.

# arrête le script si une variable n'est pas définie
set -u
# ICI : ce dossier (13_pore) ; SQ : le dossier swarm_qr, d'où partent les commandes
ICI="$(cd "$(dirname "$0")" && pwd)"
SQ="$ICI/../.."
# Python de l'environnement Isaac Sim (machine de simulation)
PY=~/isaac5_env/bin/python
# DISPLAY=:1 : écran virtuel utilisé par Isaac Sim ; PYTHONUNBUFFERED : journal affiché sans délai
export DISPLAY=:1 PYTHONUNBUFFERED=1
# Vole un cas (nom, puis options de pore_mission.py) dans pore_<nom>/ et fabrique ses vidéos ;
# saute le cas si pore_<nom>/mission.json existe déjà.
vole() {
  local nom="$1"; shift
  local sortie="$ICI/pore_$nom"
  if [ -f "$sortie/mission.json" ]; then echo "=== $nom deja fait"; return; fi
  echo "=== pore $nom : depart $(date +%H:%M)"
  # 7200 s au plus ; timeout -s KILL car Isaac Sim ignore l'arrêt normal ; grep masque les messages parasites
  ( cd "$SQ" && timeout -s KILL 7200 $PY pore_mission.py --drones 3 --budget 600 --video "$@" --sortie "$sortie" \
      2>&1 | grep -vE 'gpu.foundation|PNG|ros2|Duplicate input|enough inputs' )
  # assemble video/*.mp4 (mission, lecteur, caméras fixes) avec l'outil de l'étape 5, puis efface les images
  ( cd "$SQ" && $PY experiments/11_mission/video.py --dossier "$sortie" --sans-images )
  echo "=== pore $nom : fin $(date +%H:%M)"
}
# nominal : entrepôt 9033 ; panne : le drone 1 s'arrête à 200 s ; 9019 : entrepôt jamais vu en mise au point ;
# obstacle : un bloc de 1 x 1 x 2 m apparaît à 200 s en x = -4,96 m, y = 4,0 m
vole nominal  --seed 9033
vole panne    --seed 9033 --panne 1:200
vole 9019     --seed 9019
vole obstacle --seed 9033 --obstacle=-4.96,4.0,200
echo "=== CAMPAGNE PORE FINIE $(date +%H:%M)"
