#!/usr/bin/env bash
# Campagne complète de l'étape 2 : les 5 passes de banc.py dans Isaac Sim, puis analyse.py.
# Chaque passe écrit ses propres fichiers : un échec ne fait perdre qu'elle. Avant chaque passe,
# on vérifie qu'aucun SITL (pilote ArduPilot simulé) d'une passe précédente ne tourne encore.
#   bash campagne.sh 2>&1 | tee /tmp/campagne.log

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
# Durées maximales : 1 h 30, 30 min, 40 min, 1 h 20, 1 h 20.
passe "banc optique (2000 poses)"   5400 banc.py --mode optique   --poses 2000
passe "fausses alertes (300 poses)" 1800 banc.py --mode sans-qr   --poses 300
passe "second entrepot (400 poses)" 2400 banc.py --mode optique   --poses 400 --seed 9019 --nom 9019
passe "vol : poses tenues"          4800 banc.py --mode vol
passe "traversees a 4 vitesses"     4800 banc.py --mode traversee

echo "=========== analyse ==========="
# Analyse sans simulateur : réécrit resultats.json et enveloppe.png.
"$PY" analyse.py
echo "CAMPAGNE FINIE"
