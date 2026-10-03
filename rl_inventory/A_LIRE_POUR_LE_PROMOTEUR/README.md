# rl_inventory — Essai d'apprentissage par renforcement (étape intermédiaire, avant swarm_qr)

## En une phrase

Peut-on apprendre à un essaim de 3 drones simulés, par renforcement, à parcourir le vrai
entrepôt et à lire tous les QR des cartons ?

## La tâche

- Historique : après l'inférence active (AIF) et avant la contribution finale `swarm_qr/`,
  on a essayé l'apprentissage par renforcement (RL : un réseau de neurones apprend par
  essais et erreurs, guidé par des points de récompense).
- Plan : comparer trois « cerveaux » dans la même arène, dans l'ordre PPO → Dreamer → Pore.
  Ce dossier contient le cerveau PPO (PPO : algorithme de RL très courant). Un banc de coût
  de Dreamer est dans `../../experiments/02_dreamer_bench/` ; la comparaison avec Pore et al.
  est faite dans `../../swarm_qr/experiments/13_pore/`.
- Simulateur : Isaac Sim 5.1 + Isaac Lab 2.3 (surcouche d'Isaac Sim qui simule des dizaines
  d'entrepôts en parallèle sur la carte graphique). Ni Pegasus ni ArduPilot ici : les
  drones (modèle Crazyflie, petit quadrirotor fourni par Isaac Lab) sont pilotés
  directement en vitesse, gravité coupée.
- Arène : l'entrepôt « Simple Warehouse » de NVIDIA (123 cartons), un LiDAR (capteur laser
  qui mesure les distances) de 360 × 5 rayons par drone, et une règle géométrique qui dit
  si un QR est « lu » (proche, de face, drone lent).
- Entraînement : skrl (bibliothèque de RL), algorithme IPPO (chaque drone décide seul, mais
  les 3 drones partagent le même réseau) ; MAPPO prêt en option.
- Ce dossier contient aussi la version 2 de l'essai, `../swarmscan_map/` : elle réutilise
  cette arène, ajoute une carte construite en vol et s'entraîne avec une autre bibliothèque
  (rsl_rl). Elle a son propre guide : `../swarmscan_map/A_LIRE_POUR_LE_PROMOTEUR/`.
- Le code met en œuvre le document de conception `../../CONCEPTION_controleur_RL_inventaire.md`
  (les renvois « doc §… » du code visent ses sections).

## Contenu du dossier

Le cœur :

| Fichier | Rôle |
|---|---|
| `../env.py` | l'arène : `QRInventoryEnv` (1 drone, tests) et `SwarmQREnv` (3 drones, entraînée) |
| `../config_rl.py` | tous les réglages (LiDAR, action, récompense, lecture QR, scène), en Python pur |
| `../actuator.py` | modèle d'actionneur : la vitesse réelle rejoint la consigne avec retard |
| `../qr_task.py` | trouve les cartons, colle un QR sur 2 faces, donne la position des QR |
| `../train_ppo.py` | entraînement IPPO / MAPPO (skrl) |
| `../eval.py` | évaluation d'un modèle entraîné (métriques de mission) |
| `../agents/*.yaml` | réglages skrl d'IPPO et de MAPPO (réseaux, PPO, durée) |
| `../__init__.py` | enregistre la tâche « Isaac-QR-Inventory-Swarm-Direct-v0 » |
| `../launch.sh` | lanceur Isaac (fenêtre ou non, rendu NVIDIA, drapeau du pilote) |
| `../assets/qr/` | les 123 images QR générées, une par carton (`CARTON_0000.png`…) |
| `RESULTATS.md` | ce qui a été vérifié et mesuré, expliqué |

Les tests `../tests/` (une vérification par étape) :

| Fichier | Vérifie |
|---|---|
| `test_drone.py` | T1.2 : le drone vole selon la commande |
| `test_lidar.py` | T1.3 : le LiDAR voit le vrai entrepôt |
| `test_qr_placement.py` | T1.5a : pose des QR, image ou fenêtre pour les voir |
| `test_qr_read.py` | T1.5b : la règle « QR lu » |
| `test_swarm.py` | l'arène à 3 drones se construit et tourne |
| `test_swarmscan_map_pure.py` | version 2, sans simulateur : carte, inventaires, curriculum |
| `test_swarmscan_map_env.py` | version 2, sous Isaac : test rapide de l'environnement |
| `test_baselines_pure.py` | méthodes de référence (Pore, AIF), sans simulateur |

Les diagnostics `../diag/` (ponctuels, gardés pour mémoire) :

| Fichier | Rôle |
|---|---|
| `inspect_usd.py` | structure de l'entrepôt (maillages, tailles) |
| `find_cartons.py` | liste des objets et des cartons de l'entrepôt |
| `render_view.py` | 3 images de l'entrepôt (cartons et bacs) |
| `test_isaac5.py` | Isaac Sim 5.1 démarre-t-il sur cette machine ? |
| `test_qr_attach.py` | pose des QR sans rendu |
| `bench_rtx_lidar.py` | mémoire et vitesse avec 1 à 16 lidars réalistes (RTX) |
| `bench_vram.py` | mémoire graphique de l'essaim selon le nombre d'entrepôts |
| `profile_step.py` | temps passé dans chaque partie d'un pas |
| `test_static_raycast.py` | LiDAR dynamique contre LiDAR statique (vitesse) |

## Comment le lancer

Prérequis : Isaac Sim 5.1 + Isaac Lab 2.3 dans `~/isaac5_env`, skrl, carte NVIDIA (la
machine a 8 Go), accès Internet (l'entrepôt est lu sur le serveur public de NVIDIA ;
autre fichier possible avec la variable `AIF_FACTORY_USD`).

`launch.sh` active lui-même `~/isaac5_env` et ajoute le drapeau obligatoire
`--kit_args="--/rtx/verifyDriverVersion/enabled=false"`. Sans `--headless`, il ouvre une
fenêtre sur l'écran `:1` (autre écran : variable `ISAAC_DISPLAY`). Le chemin du script est
relatif au dossier courant. Depuis la racine du projet :

    bash rl_inventory/launch.sh rl_inventory/tests/test_drone.py --headless --num_envs 2
    bash rl_inventory/launch.sh rl_inventory/tests/test_lidar.py --headless --num_envs 2
    bash rl_inventory/launch.sh rl_inventory/tests/test_qr_read.py --headless
    bash rl_inventory/launch.sh rl_inventory/tests/test_swarm.py --headless
    bash rl_inventory/launch.sh rl_inventory/tests/test_qr_placement.py

(Depuis `rl_inventory/`, la forme d'origine marche aussi : `bash launch.sh tests/test_qr_read.py --headless`.)
Chaque test affiche ses mesures et une ligne « RÉSULTAT : OK… » ou « À VÉRIFIER ».
`test_qr_placement.py` ouvre une fenêtre (clic droit maintenu + WASD pour voler) ; avec
`--headless`, il écrit `view_qr_closeup.png` dans son dossier de sortie (`OUT`, sur la
machine de simulation).

Entraînement puis évaluation (`PY=~/isaac5_env/bin/python`) :

    bash rl_inventory/launch.sh rl_inventory/train_ppo.py --headless --algorithm IPPO --num_envs 16
    $PY rl_inventory/eval.py --checkpoint <chemin>/best_agent.pt --headless --num_envs 16 --kit_args="--/rtx/verifyDriverVersion/enabled=false"

- `train_ppo.py` : options `--algorithm IPPO|MAPPO`, `--num_envs` (16 par défaut),
  `--timesteps` (100 000 par défaut), `--seed` (42). skrl écrit journaux et checkpoints
  (points de sauvegarde du réseau, dont `best_agent.pt`) dans `qr_inventory_swarm/`, sous le
  dossier de lancement.
- `eval.py` : rejoue la politique sans hasard ; options `--algorithm` (le même qu'à
  l'entraînement), `--max_steps`, `--out fichier.csv`. Affiche un tableau de métriques
  (couverture QR, temps de mission, succès, collisions, distance entre drones, sécurité,
  efficacité, effort).

Tests sans simulateur (Python seul, avec torch et gymnasium) :

    $PY rl_inventory/tests/test_swarmscan_map_pure.py
    $PY rl_inventory/tests/test_baselines_pure.py

Diagnostics : `bash rl_inventory/launch.sh rl_inventory/diag/<script>.py --headless`
(options dans l'en-tête de chaque script). Exceptions : `bench_rtx_lidar.py` demande aussi
`--enable_cameras` ; `render_view.py` se lance avec fenêtre.

## Décisions clés

- LiDAR 360 × 5 via `MultiMeshRayCaster` (voit l'entrepôt réel, plusieurs maillages, objets
  mobiles) ; pour la vitesse, `SwarmQREnv` lance ensuite ses rayons sur un maillage fixe partagé.
- Contrôle en vitesse (`write_root_velocity_to_sim`), gravité du drone coupée (altitude imposée).
- Un vrai QR par carton (texture sur 2 faces, avant et arrière) ; règle géométrique
  (distance, champ, angle, durée de visée, vitesse) à l'entraînement, vrai décodeur `pyzbar`
  à l'évaluation.
- Entrepôt chargé en référence dans une scène locale (sinon ses textures ne sont pas trouvées).
- Drapeau du pilote RTX obligatoire : `--kit_args="--/rtx/verifyDriverVersion/enabled=false"`
  (ajouté par `launch.sh`).

## Résultats en bref

- Arène construite et dotée de tests de réussite explicites : vol commandé, LiDAR qui voit
  l'entrepôt, règle de lecture, essaim de 3 drones (observation 1820 valeurs, état global 5440).
- Physique corrigée d'après des mesures : sans modèle d'actionneur, 45 m/s² (4,59 g) en un
  pas et 87,9 % des lectures d'une politique aléatoire juste après une survitesse ; descente
  bridée → drones collés au plafond, 36 des 90 QR invisibles.
- LiDAR par lancer de rayons statique sur un maillage partagé : environ 100 fois plus rapide
  que le LiDAR dynamique d'Isaac.
- Réglages d'entraînement : IPPO, 16 entrepôts en parallèle, 100 000 pas, épisodes de 45 s.

→ détails dans `RESULTATS.md`.
