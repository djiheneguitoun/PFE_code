# rl_inventory/swarmscan_map — Inventaire par apprentissage par renforcement, avec carte (essai RL, avant swarm_qr)

## En une phrase

Trois drones qui partagent un seul réseau de neurones peuvent-ils apprendre seuls à lire
tous les QR d'un entrepôt, sans connaître la position des QR, en ne voyant qu'une carte
qu'ils construisent en vol ?

## La tâche

- Méthode : apprentissage par renforcement (RL : le réseau apprend par essais et erreurs,
  guidé par des points de récompense). Algorithme PPO avec un seul réseau pour les 3 drones
  (PS-PPO, « paramètres partagés »), bibliothèque rsl_rl 3.0.
- Simulateur : Isaac Sim 5.1 / Isaac Lab 2.3. Entrepôt « Simple Warehouse » d'Omniverse,
  123 cartons portant chacun un QR sur deux faces. Drones Crazyflie d'Isaac Lab, pilotés
  directement en vitesse : ni Pegasus ni ArduPilot ici. Beaucoup d'entrepôts tournent en
  parallèle (64 par défaut), 3 drones dans chacun.
- Ce que voit un drone : des vues de carte centrées sur lui, à deux échelles (8 m et 32 m) :
  obstacles vus par le lidar (capteur laser qui mesure les distances), zone explorée, zones
  déjà balayées par les caméras, QR déjà lus, traces des drones. Plus un petit vecteur
  (lidar résumé en 72 secteurs, vitesse, cap, altitude, coéquipiers, seuils de lecture).
  Jamais la position d'un QR non lu.
- Lecture d'un QR : une règle géométrique (le « gate »), pas un vrai décodage d'image.
  Le QR doit être vu par l'une des 2 caméras latérales (champ de 60°), à 1,25 m au plus,
  sous 50° au plus, drone lent (≤ 0,6 m/s, ≤ 0,8 rad/s), pendant 2 images de suite.
  Distance et angle ont été mesurés avec `../calibrate_gate.py`.
- Curriculum (difficulté croissante) : 10 crans. Au cran 0 la règle est tolérante
  (4 m, 60°, 1,45 m/s). On passe au cran suivant quand la moyenne glissante du taux de
  lecture dépasse 0,88 pendant 25 épisodes ; on recule si elle passe sous 0,65.
- Récompense : couverture des façades de racks, 25 points par QR lu (plus aux derniers
  crans), bonus de posture de lecture, bonus d'équipe ; pénalités près des obstacles,
  pour les à-coups et pour les drones trop proches. Après le dernier cran, un drone peut
  tomber en panne (jusqu'à 30 % des épisodes).
- Épisode : 150 s au cran 0 (+15 s par cran, 295 s au plus), 30 décisions par seconde.
  Il s'arrête quand 95 % des QR lisibles sont lus.
- Document de conception cité par le code : `docs/conception_solution_finale.md`.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../config_map.py` | tous les réglages, avec la raison de chaque valeur |
| `../env_map.py` | l'environnement Isaac Lab (observation, récompense, lecture, départs, pannes) |
| `../mapping.py` | la carte partagée par l'essaim (sans Isaac) |
| `../curriculum.py` | le curriculum de la règle de lecture (sans Isaac) |
| `../layouts.py` | tirage des configurations ; ensembles entraînement / validation / test figés |
| `../models.py` | le réseau : CNN sur les cartes + MLP |
| `../flatten_wrapper.py` | adaptateur rsl_rl : 3 drones × B entrepôts = 3B environnements |
| `../train.py` | entraînement |
| `../eval_map.py` | évaluation d'un modèle à plusieurs niveaux de bruit |
| `../verify_env.py` | vérifie qu'une politique au hasard ne lit rien |
| `../record_traj.py` | enregistre les trajectoires d'un modèle (.npz) |
| `../calibrate_gate.py` | mesure les seuils de lecture (rendu caméra + décodeur OpenCV) |
| `../__init__.py` | marque le dossier comme paquet Python |
| `RESULTATS.md` | les résultats consignés |

Tests liés, rangés dans `../../tests/` :
- `test_swarmscan_map_pure.py` : carte, tirages, curriculum, équilibre de la récompense (sans Isaac) ;
- `test_swarmscan_map_env.py` : test rapide de l'environnement dans Isaac ;
- `test_baselines_pure.py` : méthodes de référence (Pore et al., inférence active), sans Isaac.

## Comment le lancer

Prérequis : PC Linux avec GPU NVIDIA, Isaac Sim 5.1 + Isaac Lab 2.3 dans `~/isaac5_env`,
rsl_rl 3.0 et tensordict. Ni Pegasus ni ArduPilot. L'entrepôt est téléchargé depuis le
serveur de contenu Omniverse : il faut un accès Internet (ou la variable `AIF_FACTORY_USD`
vers une copie locale).

Toutes les commandes se lancent depuis la racine du projet. `rl_inventory/launch.sh` active
`~/isaac5_env` et ajoute de lui-même `--kit_args=--/rtx/verifyDriverVersion/enabled=false`.
Sans `--headless`, il ouvre une fenêtre sur `DISPLAY=:1` (modifiable par `ISAAC_DISPLAY`).

1) Vérifications sans simulateur (`PY=~/isaac5_env/bin/python`) :

    $PY rl_inventory/tests/test_swarmscan_map_pure.py

Avec pytest installé, on peut aussi lancer `$PY -m pytest rl_inventory/tests/test_swarmscan_map_pure.py`.

2) Test rapide de l'environnement dans Isaac (options `--num_envs 2`, `--steps 40`) :

    bash rl_inventory/launch.sh rl_inventory/tests/test_swarmscan_map_env.py --headless

Affiche OK ou ÉCHEC pour chaque vérification (dimensions, carte, départs, lecture latérale, panne).

3) Vérifier que l'environnement ne se laisse pas tromper :

    bash rl_inventory/launch.sh rl_inventory/swarmscan_map/verify_env.py --headless --policy random

- `--policy` : `random`, `zero` (immobile), `forward` (plein gaz tout droit) ou
  `checkpoint` (avec `--checkpoint chemin.pt` et `--sigma`, −1 = σ du modèle, 0 = sans bruit).
- `--level` : −1 = règle nominale (défaut) ; l'exemple du script utilise `--level 7`.
- Autres : `--num_envs 16`, `--spawn_help 1` (0 = pas de départ aidé), `--seed 0`.
- Joue un épisode et affiche 3 verdicts PASSE / ÉCHOUE : rien lu au gate nominal (< 0,01)
  pour random/zero/forward ; accélération p99 < 0,5 g hors chocs ; moins de 20 % des
  lectures juste après une survitesse. Plus un tableau « bons contre mauvais épisodes ».

4) Entraîner (commande d'exemple du script) :

    bash rl_inventory/launch.sh rl_inventory/swarmscan_map/train.py --headless --num_envs 32 --max_iterations 2000

| Option | Défaut | Rôle |
|---|---|---|
| `--num_envs` | 64 | entrepôts en parallèle |
| `--num_steps` | 32 | pas collectés par entrepôt et par itération |
| `--mini_batches` | 1 | mini-lots par époque |
| `--max_iterations` | 5000 | itérations PPO |
| `--seed` | 42 | graine |
| `--run_name` | v2_map | suffixe du dossier de sortie |
| `--resume` | aucun | modèle .pt à reprendre |
| `--start_level` | 0 | cran de départ (reprise : remettre celui atteint) |
| `--freeze_level` | −1 | fige le curriculum à ce cran |
| `--entropy` | 0,01 | coefficient d'entropie de départ |
| `--entropy_final` | 0,001 | entropie après la bascule |
| `--converge_level` | 9 | cran qui déclenche la bascule |

Sorties : `swarmscan_runs/<aa-mm-jj_hh-mm-ss>_<run_name>/` à la racine du projet, avec
`model_<itération>.pt` toutes les 200 itérations et les journaux TensorBoard (cran du
curriculum, fraction lue, comportement, chaque poste de la récompense).

5) Évaluer un modèle à plusieurs niveaux de bruit :

    bash rl_inventory/launch.sh rl_inventory/swarmscan_map/eval_map.py --headless \
         --checkpoint swarmscan_runs/<run>/model_XXXX.pt --level 7 --sigmas 0,0.20,0.32,0.42

- `--level` −1 (défaut) = règle nominale ; `--waves 1` vague d'épisodes par σ ; `--num_envs 16`.
- `--split` : ensemble de configurations, `train` (défaut), `val` ou `test` (configurations
  jamais vues à l'entraînement ; ni départ aidé ni panne). `--spawn_help 0` coupe les départs aidés.
- Affiche un tableau à l'écran (fraction lue à la règle évaluée et à la règle nominale,
  récompense) et le meilleur σ.

6) Enregistrer des trajectoires :

    bash rl_inventory/launch.sh rl_inventory/swarmscan_map/record_traj.py --headless \
         --checkpoint swarmscan_runs/<run>/model_XXXX.pt --level 0 --num_envs 8

Écrit `/tmp/traj.npz` (option `--out`), 1 pas sur 5 (`--every`), à analyser hors ligne.

7) Recalibrer la règle de lecture (lancé sans `launch.sh`, d'où le drapeau `--kit_args` à donner soi-même) :

    PYTHONUNBUFFERED=1 ~/isaac5_env/bin/python rl_inventory/swarmscan_map/calibrate_gate.py \
        --headless --enable_cameras --kit_args="--/rtx/verifyDriverVersion/enabled=false"

Options : `--resolution 1280 960`, `--fov_deg 60`, `--settle_frames 12`. Écrit
`docs/calibration_gate.csv` à la racine et affiche les seuils conseillés, à recopier à la
main dans `../config_map.py` (classe `GateConfig`).

## Résultats en bref

- Règle de lecture calibrée le 2026-07-09 : décodage jusqu'à 1,5 m et 55° ; seuils retenus
  1,25 m et 50° ; vitesse : seuil déclaré 0,6 m/s (loi de Cristiani 2020).
- Caméra frontale abandonnée le 2026-07-24 : la lecture tombait de 0,91 (avec bruit) à 0,11
  (sans bruit) ; avec deux caméras latérales, tout est lisible géométriquement (1,00).
- Pendant les entraînements : moyenne de lecture mesurée 0,39 ; 34 entraînements bloqués au
  cran 0 ; un modèle promu à 0,62 ne lisait plus que 0,551 au cran 0.
- 26 problèmes de l'environnement et de la récompense repérés et corrigés (tableau dans RESULTATS.md).
- Critère de réussite final : 0,85 de QR lus à la règle nominale, mesuré par `../eval_map.py`.

→ détails dans `RESULTATS.md`.
