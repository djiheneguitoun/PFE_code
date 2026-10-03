# aif_core — Exploration d'un entrepôt par inférence active (phase 1 du projet)

## En une phrase
Des drones simulés peuvent-ils explorer un entrepôt inconnu en choisissant eux-mêmes leurs mouvements par
inférence active (AIF), en architecture centralisée ou distribuée, et continuer malgré des pannes ?

## La tâche
Première approche du projet, avant l'apprentissage par renforcement (`rl_inventory`) et la contribution
finale (`swarm_qr`). Trois drones Iris volent dans Isaac Sim (simulateur 3D de NVIDIA) avec Pegasus
(extension qui simule les drones) et ArduPilot SITL (le pilote automatique ArduPilot simulé sur le PC).
Chaque drone porte un lidar (capteur laser qui mesure les distances) et construit une carte de l'entrepôt.
À chaque pas de décision, il choisit où aller pour apprendre le plus possible. La mission s'arrête quand
93 % de la zone est « connue ». On compare les architectures centralisée et distribuée, un réseau simulé
par ns-3 (simulateur de réseau), et la réaction à des pannes (perte d'un drone, coupure réseau, obstacle).
Un panneau QR fixe est aussi lu par les caméras des drones, mais il ne guide pas l'exploration.

## L'inférence active en mots simples
- **Croyance (belief)** : la carte du drone. L'entrepôt est découpé en cases de 0,5 m ; pour chaque case,
  le drone garde la probabilité qu'elle soit occupée (0,5 au départ : « je ne sais pas »).
- **Perception** : chaque rayon lidar rend les cases traversées plus libres et la case touchée plus occupée.
- **Incertitude (entropie)** : 0,69 pour une case inconnue, 0 pour une case sûre. Explorer = la faire baisser.
- **Surprise (innovation)** : à quel point les obstacles vus n'étaient pas prévus par la carte.
  Un pic de surprise est traité comme une panne (« stress »).
- **Énergie libre attendue (G)** : la note de chaque mouvement possible. Elle baisse si le mouvement
  promet d'apprendre beaucoup (curiosité) ou d'approcher des zones encore incertaines ; elle monte si
  le mouvement rapproche d'un autre drone ou d'un obstacle. Le drone tire son mouvement au hasard en
  favorisant les G bas.
- **Fusion** : les drones mettent leurs cartes en commun (moyenne case par case).

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../config.py` | tous les réglages (classe `SimConfig`), avec unités |
| `../belief.py` | carte de croyance, mise à jour par le lidar, fusion de cartes |
| `../agent.py` | un drone : perception, surprise, envoi du point visé au pilote |
| `../planner.py` | choix du mouvement : AIF (note G) ou heuristique simple |
| `../swarm.py` | enchaîne les étapes d'un pas, calcule les métriques |
| `../architecture/centralized.py` | décision par un serveur central (« cloud ») |
| `../architecture/distributed.py` | décision locale de chaque drone |
| `../architecture/base.py` | interface commune aux deux architectures |
| `../network.py` | messages retardés, latences ns-3, liens coupés |
| `../resilience.py` | détection des stress, phases normal / recovery / durable |
| `../stressors.py` | pannes programmées par options |
| `../metrics.py` | vitesse de découverte, décisions par minute… |
| `../loggers.py` | fichiers JSON en direct et journal détaillé |
| `../math_utils.py` | probabilité ↔ log-odds, entropie, tirage softmax |

Scripts du dossier `scripts/` qui utilisent ce module :

| Fichier | Rôle |
|---|---|
| `../../12_aif_isaac_sim.py` | programme principal (scène, drones, boucle) |
| `../../12_launch_aif_isaac_sim.sh` | lanceur : vérifications puis programme principal |
| `../../12_ns3_bridge.py` | lance ns-3 et échange les fichiers avec lui |
| `../../run_all_experiments.sh` | enchaîne les runs de la campagne |
| `../../run_artifacts.py` | écrit le dossier du run (JSON + figures) |
| `../../qr_code_system.py` | panneau QR, caméras, décodage |

## Comment les modules s'enchaînent
`12_aif_isaac_sim.py` charge l'entrepôt, crée les drones, les fait décoller (pas de décision pendant
cette phase), puis répète : un pas de décision (`SwarmCoordinator.step()` dans `swarm.py`), puis 60 pas
physiques de vol (1 s simulée). Un pas de décision fait, dans l'ordre :
1. `stressors.py` : déclenche la panne prévue à ce pas, s'il y en a une.
2. `agent.py` : chaque drone lit son lidar, calcule sa surprise, met à jour sa carte (`belief.py`).
3. `resilience.py` : un pic de surprise déclenche un stress.
4. `swarm.py` : fusionne les cartes de tous les drones actifs (carte globale).
5. `resilience.py` : met à jour la phase (normal, recovery, durable).
6. Architecture : échange des cartes, puis livraison des messages arrivés (`network.py`).
7. Architecture : chaque drone reçoit son mouvement (`planner.py`) et l'envoie à son pilote.
8. `metrics.py` : métriques du pas, ajoutées à l'historique.

Fin : 93 % de couverture, ou plateau (gain < 0,1 point par pas pendant 15 pas), ou `--max-steps`.

## Centralisé ou distribué
| | Centralisé (`--arch centralized`) | Distribué (`--arch distributed`) |
|---|---|---|
| Qui décide | le cloud, pour tous | chaque drone, pour lui |
| Carte utilisée | carte globale de l'essaim | sa carte + cartes reçues des voisins |
| Délai | action reçue après 500 ms (1 pas) | carte des voisins retardée par ns-3 |
| Portée | tous les drones | voisins à moins de 5 m |
| Si coupure | 2 s d'attente, puis mode distribué | 2 s d'attente, puis mode « solo » |

Détails :
- Centralisé : tout l'aller-retour (envoi de la carte, calcul, retour de l'action) est compté sur
  l'action (`--cloud-round-trip-ms`, 500 ms par défaut). Au premier pas, aucune action n'est encore
  arrivée : les drones restent sur place.
- Le cloud choisit les mouvements drone par drone ; les cibles déjà choisies comptent comme des drones
  à éviter. En distribué, chaque drone décide seul.
- Pendant une bascule (2 s, soit 2 pas), aucun drone ne reçoit de nouvelle action.
- Dans les deux cas, la carte de planification mélange 70 % de la carte du drone et 30 % de la carte
  fusionnée (`fusion_mix`).

## Pannes et phases de résilience
Options de panne (pas = pas de décision, -1 = jamais) :
- `--kill-drone-at-step N --kill-drone-id K` : le drone K atterrit et ne participe plus.
- `--cut-cloud-at-step N` : coupure du cloud (en centralisé, bascule en distribué).
- `--cut-drone-link "0-1"` ou `"all"` avec `--cut-drone-link-at-step N` : coupure de liens drone-drone.
- `--drop-obstacle-at-step N --drop-obstacle-xy "x,y"` : cube rouge de 3 × 3 × 4 m (position en m, repère
  monde d'Isaac Sim).

Après un stress : phase « recovery » pendant 30 pas, puis « durable ». Le stress est levé après 60 pas
d'affilée avec une carte « rétablie » (entropie ≤ 0,44 et surprise ≤ 0,16).

## Comment le lancer
Prérequis : machine Linux avec carte NVIDIA et écran X11 (sinon mode sans fenêtre).
- Isaac Sim 4.5.0 et Pegasus, installés dans `~/isaac_sim_env` par `install_isaac_sim.sh`. Ce n'est
  pas l'environnement `~/isaac5_env` de `swarm_qr`. Le lanceur n'utilise pas `PY` : il active lui-même
  `~/isaac_sim_env/activate_isaac.sh`.
- ArduPilot compilé en SITL dans `~/ardupilot` (`./waf configure --board sitl && ./waf copter`).
- Paquets Python : pymavlink, qrcode, opencv, Pillow, matplotlib (pyzbar facultatif).
- Accès internet : l'entrepôt `warehouse_multiple_shelves.usd` est téléchargé depuis les serveurs NVIDIA.
- ns-3.40 dans `~/ns-allinone-3.40/ns-3.40`, facultatif (sans lui, latence 0). Les scénarios (sources
  dans `scenarios/` à la racine du projet) doivent déjà être compilés dans ns-3, car le pont le lance
  avec `--no-build`.

Durée : environ 7 à 10 minutes par run. Avant la première décision, il y a environ 3 300 pas physiques d'initialisation
et de décollage (jusqu'à 3 000 de plus si un drone tarde à décoller).

Depuis la racine du projet :

    bash scripts/12_launch_aif_isaac_sim.sh 3 --arch centralized --run-tag aif_cent_baseline
    bash scripts/12_launch_aif_isaac_sim.sh 3 --arch distributed --run-tag aif_dist_baseline
    bash scripts/12_launch_aif_isaac_sim.sh 3 --arch centralized --cut-cloud-at-step 20 --run-tag essai_coupure

- 1er argument : nombre de drones (3 par défaut). Les arguments suivants vont tels quels au script Python.
- `HEADLESS=1 bash scripts/12_launch_aif_isaac_sim.sh 3` force le mode sans fenêtre.
- Le lanceur tue les processus ArduPilot restants (`arducopter`, `sim_vehicle.py`, `mavproxy`) et
  modifie `~/ardupilot/Tools/autotest/default_params/gazebo-iris.parm` (contrôles d'armement et
  sécurités coupés).

Campagne de runs (80 pas maximum, en WiFi) :

    bash scripts/run_all_experiments.sh
    ONLY="aif_cent_baseline,aif_dist_baseline" MAX_STEPS=80 bash scripts/run_all_experiments.sh

`run_all_experiments.sh` appelle le lanceur directement. Après une copie depuis Google Drive, rendre
d'abord les scripts exécutables : `chmod +x scripts/*.sh`.

Sans le lanceur (après `source ~/isaac_sim_env/activate_isaac.sh`) :

    python scripts/12_aif_isaac_sim.py --num-drones 3 --arch distributed --ns3 none

Le drapeau `--/rtx/verifyDriverVersion/enabled=false` est déjà ajouté par le script lui-même.

Toutes les options de `12_aif_isaac_sim.py` (valeurs par défaut entre parenthèses) :
- `--num-drones` (3), `--max-steps` (500), `--headless`
- `--env-width` (30), `--env-height` (20) : taille de la zone sans entrepôt chargé
- `--planner aif|heuristic` (aif), `--arch centralized|distributed` (centralized)
- `--neighbor-radius-m` (5.0), `--ns3 none|wifi|5g` (wifi)
- `--cloud-round-trip-ms` (500), `--ns3-sim-time` (600 s)
- les options de panne ci-dessus
- `--run-tag` (default), `--runs-dir` (vide = `logs/runs`)

Pont ns-3 seul, pour le tester :

    python3 scripts/12_ns3_bridge.py --scenario wifi --n-drones 3 --sim-time 600

## Ce que produit un run
- Pendant le vol, dans `/tmp` : `aif_state.json`, `aif_history.json` et `aif_diagnostic.log` (journal
  détaillé). Ces fichiers sont lus par le tableau de bord : `python3 dashboard_aif/server.py`, puis
  ouvrir http://localhost:8060.
- À la fin, `logs/runs/run_<date>_<heure>_<tag>/` contient :
  - `config.json` (réglages), `history.json` (métriques de chaque pas), `state_final.json`, `qr_stats.json` ;
  - les figures (couverture, entropie, surprise, phases de résilience, carte finale, trajectoires,
    latences ns-3…) ;
  - un `README.md` généré automatiquement.

## Résultats en bref
D'après les `README.md` générés dans `../../../logs/runs/` :
- Les 8 runs enregistrés s'arrêtent tous au seuil de couverture (93,0 à 93,4 %).
- Pas de décision nécessaires : AIF centralisé 45, AIF distribué 53, heuristique centralisée 69,
  AIF centralisé avec cloud lent (1 500 ms) 44.
- Avec une panne au pas 20 : perte d'un drone 79 pas, coupure du cloud 57 (fin en distribué),
  coupure de tous les liens 52, obstacle soudain 50.

→ Analyse détaillée dans `../../A_LIRE_POUR_LE_PROMOTEUR/RESULTATS.md`.
