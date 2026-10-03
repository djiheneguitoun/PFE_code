# 08 — Contrôle de vol : rejoindre une pose et s'y tenir, seul et à trois (étape 3)

## En une phrase
Le drone sait-il rejoindre une pose de lecture devant un carton, s'y tenir le temps de lire et
dire s'il a réussi, seul puis à trois drones en même temps ?

## La tâche
Le contrôleur de vol (`swarm_qr/control.py`) reçoit une pose (position + cap) et, au besoin, des
points de passage. Il avance par phases (transit, approche, tenue) et finit par « atteint » ou
par « abandon » avec une raison. Il envoie des consignes de vitesse à ArduPilot SITL (le pilote
automatique simulé sur le PC), en bouclant sur la position vraie du drone.

On le teste dans Isaac Sim, sur le même entrepôt qu'à l'étape 2 (graine 9033), en trois vols :
1. freinage : trois façons de s'arrêter sur le même trajet de 4 m, trois fois chacune ;
2. 100 poses tirées au hasard devant les cartons, avec une vraie lecture du QR à chaque arrivée ;
3. essaim : trois drones décollent et volent en même temps vers trois cibles, puis vers trois autres.

La position vraie du drone est enregistrée à chaque pas, la calibration de la caméra est vérifiée
avant chaque vol, et le banc s'arrête si un drone ne décolle pas. On mesure aussi le temps d'un
cycle (aller, se placer, tenir), qui fixe combien de cartons une mission peut lire.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../banc.py` | le banc dans Isaac Sim + SITL, 3 modes : `freinage`, `poses`, `essaim` |
| `../campagne.sh` | relance les 3 vols puis l'analyse |
| `../analyse.py` | l'analyse sans simulateur : écrit `resultats.json` et les 3 figures |
| `../freinage.json` | les 9 essais de freinage (3 lois × 3) : trajectoires et bilans |
| `../poses.jsonl` | une ligne par pose (100) : cible, chemin, bilan du contrôleur, lecture du QR, trajectoire |
| `../meta_poses.json` | réglages du contrôleur pendant les 100 poses (vitesses, gain, tolérances, calibration K) |
| `../essaim.json` | les 2 manches à 3 drones : bilans, lectures, trajectoires |
| `../resultats.json` | tous les chiffres calculés par l'analyse |
| `../freinage.png` | distance à la cible et vitesse au cours du temps, pour les 3 lois |
| `../cycles.png` | temps de cycle des 100 poses : histogramme, et selon la longueur du trajet |
| `../essaim.png` | trajectoires des 3 drones sur le plan de l'entrepôt, une vue par manche |
| `../../../control.py` | le contrôleur testé (hors de ce dossier) |
| `../../../tests/test_control.py` | ses tests sans simulateur, sur un faux drone |
| `../../_vol.py` | la règle simple qui donne le chemin (même allée : ligne droite ; sinon, le couloir) |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus + ArduPilot SITL, carte graphique NVIDIA RTX et écran
`DISPLAY=:1` (Pegasus ouvre une fenêtre par SITL), pour `campagne.sh` et `banc.py`. Pour
`analyse.py`, Python avec numpy et matplotlib suffit ; pour les tests, numpy et pytest.
Durée : les 100 poses ont demandé 48 min de calcul ; `campagne.sh` accorde au plus 1 h
(freinage), 4 h (poses) et 1 h 30 (essaim).

Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    bash swarm_qr/experiments/08_controle/campagne.sh 2>&1 | tee /tmp/campagne_controle.log

Ce script utilise le Python de la machine de simulation (ligne `PY=`). Il se place dans son
dossier, refuse une passe si un SITL tourne encore, et tue Isaac Sim (`timeout -s KILL`) au-delà
de la durée permise, car Isaac ignore le signal d'arrêt normal. Les mêmes passes, une par une :

    E=swarm_qr/experiments/08_controle
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 3600 $PY $E/banc.py --mode freinage
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 14400 $PY $E/banc.py --mode poses --poses 100
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 5400 $PY $E/banc.py --mode essaim

Elles écrivent, dans le dossier de l'expérience, `freinage.json`, `poses.jsonl` +
`meta_poses.json` et `essaim.json`. Inutile d'ajouter `--kit_args` : `banc.py` désactive
lui-même la vérification du pilote graphique. Autres options : `--seed` (entrepôt, 9033 par
défaut), `--graine-tirage` (tirage des poses, 20260906 par défaut), `--v-transit` (vitesse de
transit en m/s ; 1,5 par défaut, valeur de `control.py`), `--nom` (nom des fichiers de sortie).

L'analyse (sans simulateur ; réécrit `resultats.json`, `freinage.png`, `cycles.png`,
`essaim.png`) et les tests du contrôleur (sans simulateur, en une fraction de seconde) :

    python swarm_qr/experiments/08_controle/analyse.py
    python -m pytest swarm_qr/tests/test_control.py -q

## Résultats en bref
- Loi retenue : vitesse proportionnelle à la distance restante, plafonnée à 1 m/s, sur la
  position vraie. Elle arrive en 4,7 s sur 4 m et tient la pose à 1 cm ; couper la vitesse fait
  glisser le drone de 79 cm, la consigne de position d'ArduPilot s'arrête à 25 cm.
- 100 poses sur 100 atteintes, sans abandon ; 8,9 cm d'erreur médiane à l'arrivée (tolérance
  15 cm) ; QR lu après 93 arrivées sur 100.
- Temps de cycle : 14,6 s en médiane, soit environ 2 s + 0,85 s par mètre de trajet.
- Trois drones ensemble : 6 cibles sur 6 et 6 QR sur 6, simulation à 0,23 fois le temps réel.
- Le contrôleur n'évite pas les autres drones : deux drones se sont croisés à 0,96 m.

→ détails dans `RESULTATS.md`.
