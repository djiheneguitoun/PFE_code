# scripts — Premières phases : de Gazebo à l'inférence active (scripts 06 → 12)

## En une phrase

Ce dossier retrace les premières phases du projet : faire voler plusieurs drones simulés, mesurer le réseau
entre eux, passer à Isaac Sim, puis explorer l'entrepôt par inférence active (AIF).

## La tâche

Avant la contribution finale (`swarm_qr/`), il fallait une base : plusieurs drones pilotés par ArduPilot SITL
(le pilote automatique ArduPilot simulé sur le PC), un modèle du réseau radio entre eux, et une méthode pour
décider où aller. Les scripts sont numérotés dans l'ordre où ils ont été écrits :
- 01 à 05 : installation des outils (dossier `../installation/`) ;
- 06 à 09 : drones sous Gazebo (simulateur de robots) et mesure du réseau WiFi / 5G ;
- 11 : passage à Isaac Sim (simulateur 3D de NVIDIA) avec Pegasus (extension pour drones) ;
- 12 : exploration par inférence active dans Isaac Sim, avec réseau simulé et incidents (pannes, coupures).
Il n'y a pas de script 10. Seule la phase 12 a des résultats enregistrés (dans `../../logs/runs/`).

## Chronologie des scripts

| N° | Fichier | Ce qu'il fait |
|---|---|---|
| 06 | `../06_launch_multi_drones.sh` | Lance N drones Iris dans un entrepôt Gazebo de 30 × 20 m, un SITL par drone ; crée `../../models/iris_instance_<i>` |
| 07 | `../07_multi_drone_flight.py` | Vol simple via MAVLink (protocole de messages du pilote) : décollage, 15 s sur place, atterrissage |
| 07b | `../07b_dynamic_flight.py` | Patrouille : 6 points de passage par drone, 2 tours, entre 1 et 5,8 m d'altitude |
| 08 | `../08_wifi_bridge.py` | Qualité du lien WiFi entre chaque paire de drones : RSSI (puissance reçue) par Sionna, latence par ns-3 |
| 09 | `../09_5g_lena_bridge.py` | Même chose en 5G : RSSI drone ↔ antenne fixe (Sionna), latence entre drones (ns-3 5G-LENA) |
| 11 | `../11_launch_isaac_sim_drones.sh` + `../11_isaac_sim_drones.py` | Premier essai Isaac Sim + Pegasus : 3 drones posés dans un entrepôt, sans pilote |
| — | `../AIF_contoller.py` | Premier prototype du contrôleur AIF, sur une grille abstraite, sans simulateur |
| 12 | `../12_launch_aif_isaac_sim.sh` + `../12_aif_isaac_sim.py` | Exploration AIF : 3 drones SITL + LiDAR dans Isaac Sim, réseau ns-3, incidents programmés |
| 12 | `../12_ns3_bridge.py` | Lance ns-3 (WiFi ou 5G) en arrière-plan pendant un run AIF |
| 12 | `../qr_code_system.py` | Panneau QR + caméra par drone + décodage (utilisé par le script 12) |
| 12 | `../run_artifacts.py` | Enregistre les résultats d'un run dans `../../logs/runs/` (JSON, 11 graphiques, README) |
| 12 | `../run_all_experiments.sh` | Lance la série d'expériences AIF |

Glossaire : RSSI = puissance du signal reçu (dBm) ; Sionna = calcul de propagation radio par lancer de rayons
dans la scène 3D ; ns-3 = simulateur de réseau ; LiDAR = capteur laser qui mesure les distances ; AIF (inférence
active) = chaque drone choisit l'action qui réduit le plus son incertitude sur la carte.

## Contenu du dossier

| Fichier ou dossier | Rôle |
|---|---|
| `../aif_core/` | Cœur de l'AIF (carte, planificateur, architectures, réseau, incidents) → `../aif_core/A_LIRE_POUR_LE_PROMOTEUR/README.md` |
| `../installation/` | Scripts d'installation 01 à 05 (dépendances, Gazebo, ArduPilot SITL, ns-3, ns3-sionna) → `../installation/A_LIRE_POUR_LE_PROMOTEUR/README.md` |
| scripts 06 à 12 | voir la chronologie ci-dessus |
| `../run_baseline_matrix.sh` | Appartient à la phase RL (`rl_inventory/`) : évalue les politiques « aif » et « serpentine » sur une série de scénarios |
| `RESULTATS.md` | Les résultats des 8 runs AIF, expliqués |

Fichiers liés hors de ce dossier : `../../scenarios/` (sources des scénarios ns-3 WiFi et 5G),
`../../models/` (modèles Gazebo du drone, créés par 06), `../../logs/runs/` (résultats AIF),
`../../dashboard/` (tableau de bord des ponts 08/09), `../../dashboard_aif/` (tableau de bord AIF).

## Comment le lancer

Tout se lance sur la machine de simulation Linux. Les fichiers temporaires vont dans `/tmp`.

Phases 06 à 09 — prérequis : Gazebo Harmonic, ArduPilot dans `~/ardupilot`, plugin `~/ardupilot_gazebo`
(scripts `../installation/`), pymavlink ; pour 08/09 : ns-3.40 dans `~/ns-allinone-3.40` avec les scénarios
`drone-wifi-scenario` et `drone-5g-nr-scenario` compilés (sources dans `../../scenarios/`, à placer dans `scratch/`),
et un Python où Sionna RT est installé (l'installation 05 crée l'environnement `sionna-venv` de ns3-sionna).
Le script 06 travaille dans `~/simulation_mc02` (le dossier du projet sur la machine Linux).

    bash scripts/06_launch_multi_drones.sh 3            # terminal 1 : Gazebo + 3 SITL (répondre 1 ou 2 pour le rendu)
    python3 scripts/07_multi_drone_flight.py --drones 3  # terminal 2 : vol simple
    python3 scripts/07b_dynamic_flight.py --drones 3 --loops 2   # ou : patrouille
    python3 scripts/08_wifi_bridge.py                    # terminal 3 : pont WiFi (--test : drones fictifs, sans SITL)
    python3 scripts/09_5g_lena_bridge.py --interval 3    # ou pont 5G (--test possible aussi)

Les ponts écrivent leurs CSV dans `/tmp` (`drone_bridge_log.csv` pour le WiFi, `drone_5g_log.csv` pour la 5G),
lisibles avec le tableau de bord : depuis `dashboard/`, `python3 dashboard_server.py --port 8050`.

Phases 11 et 12 — prérequis : GPU NVIDIA, Isaac Sim + Pegasus dans `~/isaac_sim_env` (créé par
`../../install_isaac_sim.sh`, à la racine du projet) ; pour 12, ArduPilot SITL compilé
(`~/ardupilot/build/sitl/bin/arducopter`) et pymavlink ; ns-3 est facultatif (sans lui, latences = 0).
`HEADLESS=1` force le mode sans fenêtre.

    bash scripts/11_launch_isaac_sim_drones.sh 3
    bash scripts/12_launch_aif_isaac_sim.sh 3 --max-steps 80 --run-tag aif_cent_baseline --planner aif --arch centralized --ns3 wifi
    bash scripts/run_all_experiments.sh                  # série complète (ONLY="tag1,tag2" pour n'en lancer que certains)

Un run AIF dure environ 7 à 10 minutes. Il crée `logs/runs/run_<date>_<heure>_<tag>/` ; ces dossiers se parcourent
avec le tableau de bord AIF : depuis `dashboard_aif/`, `python3 server.py --port 8060`, puis http://localhost:8060
(onglet « Runs »). Le lanceur 12 modifie aussi `~/ardupilot/Tools/autotest/default_params/gazebo-iris.parm`
(désactive les contrôles d'armement et plusieurs sécurités du SITL : perte de radio, perte de station sol, détection de crash).

Le prototype `AIF_contoller.py` se lance seul, sans simulateur : `python3 scripts/AIF_contoller.py`
(2 agents sur une grille 10 × 10, un pas, affiche les actions).

## Résultats en bref

- 8 runs AIF (3 drones) : tous atteignent l'objectif de couverture de 93 %, même avec un incident.
- AIF centralisé : 45 steps ; AIF distribué : 53 ; heuristique : 69 (et 42 % de distance en plus).
- Cloud 3 fois plus lent : 44 steps (pas de ralentissement) ; drone perdu : 79 ; cloud coupé : 57 ;
  liens entre drones coupés : 52 ; obstacle soudain : 50.
→ détails dans `RESULTATS.md`.
