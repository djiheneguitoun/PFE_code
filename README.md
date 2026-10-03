# PFE — Inventaire de QR codes en entrepôt par un essaim de drones (simulation)

Des drones simulés font l'inventaire d'un entrepôt : ils trouvent et lisent les QR codes collés
sur les cartons. Le projet a connu trois phases ; **la contribution finale est le dossier
`swarm_qr/`**.

## Par où commencer

1. Ce fichier : la structure du dépôt et les commandes principales.
2. `swarm_qr/A_LIRE_POUR_LE_PROMOTEUR/README.md` puis `RESULTATS.md` : le système final et
   ses résultats.
3. Voir les drones voler :
   - le rejeu 3D des 8 vols de l'évaluation : `swarm_qr/experiments/14_rejeu/rejeu_hors_ligne.html`
     (double-clic, fonctionne sans Internet) ;
   - les vidéos : `swarm_qr/experiments/11_mission/tests of system/eval_*/video/mission.mp4`.

**Dans chaque dossier principal et dans chaque expérience, un sous-dossier
`A_LIRE_POUR_LE_PROMOTEUR/`** contient :
- `README.md` : la tâche, le contenu du dossier, les commandes pour le lancer ;
- `RESULTATS.md` (quand il y a des résultats) : les résultats expliqués.

## Les trois phases du projet

| Phase | Dossiers | En bref |
|---|---|---|
| 1. Premiers vols, réseau et inférence active | `scripts/`, `dashboard/`, `dashboard_aif/`, `logs/`, `scenarios/`, `models/` | Vols multi-drones sous Gazebo, communication simulée avec ns-3 (WiFi, 5G), puis un essaim piloté par inférence active (AIF) dans Isaac Sim, testé en architecture centralisée et distribuée et sous perturbations (mars à juin 2026). |
| 2. Apprentissage par renforcement | `rl_inventory/`, `experiments/` | Un environnement d'apprentissage (Isaac Lab, PPO) pour apprendre à 3 drones à lire les QR, puis une variante avec carte. Fin août, trois bancs rapides ont jugé une autre piste, DreamerV3 (résolution nécessaire, coût, débit du simulateur) (juin à août 2026). |
| 3. **Système final** | **`swarm_qr/`** | Un essaim de 3 drones pilotés par de vrais pilotes automatiques ArduPilot, qui construit une carte partagée et décide en vol où aller ; évalué contre une méthode publiée (Pore et al., 2026) (août à septembre 2026). |

## Structure du dépôt

    PFE_code/
    ├── README.md                  ce fichier
    ├── swarm_qr/                  LA CONTRIBUTION FINALE : le système multi-drones
    │   ├── A_LIRE_POUR_LE_PROMOTEUR/   le système et ses résultats finaux
    │   ├── mission.py             lance une mission (point d'entrée)
    │   ├── mapping.py             la carte partagée
    │   ├── planning.py            le choix des cibles
    │   ├── control.py             le contrôle de vol
    │   ├── observation.py, perception.py, detecteur.py   voir et lire les QR
    │   ├── guide.py               guide vision-langage (optionnel)
    │   ├── pore.py, pore_mission.py    méthode de référence (Pore et al.)
    │   ├── env/                   le simulateur (entrepôt, drones, QR)
    │   ├── tests/                 89 tests unitaires sans simulateur
    │   ├── experiments/           14 expériences, étape par étape
    │   ├── docs/figures/          figures du mémoire
    │   └── assets/                détecteur YOLO entraîné, images des QR
    ├── rl_inventory/              phase 2 : apprentissage par renforcement
    ├── experiments/               phase 2 : bancs préliminaires
    ├── scripts/                   phase 1 : Gazebo, ns-3, inférence active
    │   ├── aif_core/              le cœur de l'inférence active
    │   └── installation/          scripts d'installation (Gazebo, ArduPilot, ns-3)
    ├── dashboard/, dashboard_aif/ phase 1 : tableaux de bord web
    ├── logs/                      phase 1 : résultats bruts des expériences AIF
    ├── scenarios/                 phase 1 : scénarios réseau ns-3 (WiFi, 5G)
    ├── models/                    phase 1 : modèles Gazebo du drone Iris
    ├── install_isaac_sim.sh       phase 1 : installation d'Isaac Sim 4.5 et de Pegasus
    └── .gitignore                 fichiers lourds exclus de git (vidéos, images brutes…)

## Commandes principales (système final)

Depuis la racine du projet. Sur la machine de simulation, ce dossier s'appelle
`~/simulation_mc02`, et `PY` désigne le Python de l'environnement Isaac Sim 5.1 :
`PY=~/isaac5_env/bin/python`.

Vérifier le code sans simulateur (Python 3.10+, `numpy`, `opencv-python`, `pytest`) :

    python -m pytest swarm_qr/tests -q

Une mission complète (3 drones, entrepôt 9033, 600 s simulées), puis son bilan et ses vidéos :

    $PY swarm_qr/mission.py --seed 9033 --drones 3 --budget 600 --detecteur auto --video --sortie sorties/ma_mission
    $PY swarm_qr/experiments/11_mission/analyse.py --dossier sorties/ma_mission
    $PY swarm_qr/experiments/11_mission/video.py --dossier sorties/ma_mission --sans-images

Les 4 vols de l'évaluation finale (nominal, panne d'un drone, entrepôt jamais vu, obstacle) :

    bash swarm_qr/experiments/11_mission/evaluation.sh

La méthode de référence de Pore et al. sur les mêmes 4 cas :

    bash swarm_qr/experiments/13_pore/campagne.sh

Les commandes de chaque expérience et des phases précédentes sont dans le
`A_LIRE_POUR_LE_PROMOTEUR/README.md` de chaque dossier.

## Résultat principal

Sur 4 cas, le système lit plus de codes que la méthode de référence et atteint 90 % de
l'inventaire en environ 3 minutes simulées, contre plus de 8 minutes ou jamais pour la
référence, sans aucune chute ni contact :

| Cas | Système | Référence (Pore et al.) |
|---|---|---|
| Nominal (entrepôt 9033) | 110 / 114 codes | 106 / 114 |
| Panne d'un drone | 110 / 114 | 75 / 114 |
| Entrepôt jamais vu (9019) | 60 / 65 | 53 / 65 |
| Obstacle apparu en vol | 110 / 114 | 103 / 114 |

Détails et limites : `swarm_qr/A_LIRE_POUR_LE_PROMOTEUR/RESULTATS.md`.

## Outils utilisés (système final)

| Rôle | Outil |
|---|---|
| Simulateur | Isaac Sim 5.1 (NVIDIA), pas de physique 1/800 s |
| Drones | Pegasus Simulator (drone Iris) |
| Pilote automatique | ArduPilot SITL, un par drone, commandé par MAVLink (`pymavlink`) |
| Repérage des QR et cartons | YOLO11n (Ultralytics), entraîné sur des images du simulateur |
| Lecture des QR | zxing-cpp (ZBar et OpenCV en alternative) |
| Carte et chemins | `numpy` (grille de 25 cm), recherche de chemin A* |
| Guide optionnel | SmolVLM / Qwen2.5-VL, adaptateur LoRA |
| Machine | Linux, carte graphique NVIDIA RTX (8 Go) |
