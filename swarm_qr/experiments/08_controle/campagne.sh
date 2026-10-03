#!/usr/bin/env bash
# Campagne complète de l'étape 3 : les 3 passes de banc.py (Isaac Sim + ArduPilot SITL), puis analyse.py.
# Chaque passe écrit ses propres fichiers : un échec ne fait perdre qu'elle. Avant chaque passe,
# on vérifie qu'aucun SITL (pilote ArduPilot simulé) d'une passe précédente ne tourne encore.
#   bash campagne.sh 2>&1 | tee /tmp/campagne_controle.log

set -u
# Python de l'environnement Isaac Sim, sur la machine de simulation.
PY=/home/djihene_guitoun/isaac5_env/bin/python
# Dossier de ce script : on s'y place, et toutes les sorties y sont écrites.
ICI="$(cd "$(dirname "$0")" && pwd)"
# Écran graphique :1 (Pegasus ouvre une fenêtre par SITL) ; affichage immédiat des messages.
export DISPLAY=:1 PYTHONUNBUFFERED=1

# Lance une passe : passe "<nom>" <durée maximale en s> <script> <options> ; refuse si un SITL
# tourne encore, et tue Isaac Sim (signal KILL, il ignore l'arrêt normal) au-delà de la durée.
passe() {
  local nom="$1" duree="$2"; shift 2
  echo "=========== $nom ==========="
  pgrep -x arducopter >/dev/null && { echo "ATTENTION : un SITL tourne encore"; return 1; }
  timeout -s KILL "$duree" "$PY" "$@"
  echo "--- $nom : code de sortie $?"
  sleep 5
}

cd "$ICI" || exit 1
# Durées maximales : 1 h, 4 h, 1 h 30 (les 100 poses ont demandé 48 min de calcul).
passe "freinage : trois lois"        3600  banc.py --mode freinage
passe "cent poses"                  14400  banc.py --mode poses --poses 100
passe "essaim : trois drones"        5400  banc.py --mode essaim

echo "=========== analyse ==========="
# Analyse sans simulateur : réécrit resultats.json et les trois figures.
"$PY" analyse.py
echo "CAMPAGNE FINIE"
