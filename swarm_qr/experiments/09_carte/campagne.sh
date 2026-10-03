#!/usr/bin/env bash
# Lance toute l'étape 4 (carte partagée) : tests unitaires, contrôles du lidar, patrouille, analyse.
# Les tests et les contrôles passent d'abord : si le lidar ne dit pas la vérité, inutile de voler.
# Lancement (depuis n'importe où) :
#   bash campagne.sh 2>&1 | tee /tmp/campagne_carte.log

# arrête le script si une variable n'est pas définie
set -u
# Python de l'environnement Isaac Sim (machine de simulation)
PY=/home/djihene_guitoun/isaac5_env/bin/python
# dossier de ce script (09_carte)
ICI="$(cd "$(dirname "$0")" && pwd)"
# DISPLAY=:1 : écran virtuel utilisé par Isaac Sim ; PYTHONUNBUFFERED : journal affiché sans délai
export DISPLAY=:1 PYTHONUNBUFFERED=1

# Lance une passe du banc (nom, durée max en s, script et options) sous timeout -s KILL (Isaac Sim
# ignore l'arrêt normal) ; refuse de démarrer si un SITL (ArduPilot simulé) tourne encore.
passe() {
  local nom="$1" duree="$2"; shift 2
  echo "=========== $nom ==========="
  pgrep -x arducopter >/dev/null && { echo "ATTENTION : un SITL tourne encore"; return 1; }
  timeout -s KILL "$duree" "$PY" "$@"
  echo "--- $nom : code de sortie $?"
  sleep 5
}

cd "$ICI" || exit 1
# tests unitaires de swarm_qr/tests (sans simulateur) : la campagne s'arrête s'ils échouent
"$PY" -m pytest "$ICI/../../tests" -q || exit 1
# contrôles du lidar, sans vol : 1800 s = 30 min au plus (écrit verification.json, vue_un_tour.png)
passe "controles de la chaine lidar"  1800  banc.py --mode verifie
# patrouille sur les trois étagères : 10800 s = 3 h au plus (écrit carte.npz, vol.json, vue_NN.png, la vidéo)
passe "patrouille, trois etageres"   10800  banc.py --mode vol --etages 0,1,2
echo "=========== analyse ==========="
# jugement sans simulateur (écrit resultats.json et les figures)
"$PY" analyse.py
echo "CAMPAGNE FINIE"
