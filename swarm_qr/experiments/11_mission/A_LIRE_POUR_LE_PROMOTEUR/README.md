# 11 — Le cerveau et les missions complètes à 3 drones (étape 5)

## En une phrase

Les composants des étapes précédentes (lecture, carte partagée, détecteur, contrôle de vol)
forment-ils un système où 3 drones choisissent seuls leurs cibles, se partagent le travail,
absorbent une panne et terminent seuls la mission ? Ce dossier contient aussi
**l'évaluation finale du système** (4 vols, dans `tests of system/`).

## La tâche

- Chaque drone choisit ses cibles sur **sa carte**, jamais sur le plan de l'entrepôt : lire un
  QR repéré de loin, couvrir une surface jamais regardée, explorer une zone inconnue.
- Les 3 drones partagent une carte et se réservent des cibles ; une réservation non renouvelée
  expire seule. La panne d'un drone est donc gérée sans code spécial.
- La porte de validation est une mécanique, pas une performance : aucun blocage, aucun doublon,
  une dispersion au départ, une panne absorbée, une fin propre. `analyse.py` vérifie ces 5
  points après le vol, avec la vérité de la simulation (position et contenu de chaque QR).
- Tout se passe dans Isaac Sim 5.1 + Pegasus + ArduPilot SITL (le pilote automatique ArduPilot
  simulé sur le PC), 600 s simulées au plus, détecteur YOLO de l'étape 7 branché.
- Deux séries de vols : la **campagne de mise au point** (`campagne.sh`, résultats racontés
  dans `RESULTATS.md`) et l'**évaluation finale** (`evaluation.sh`, 4 vols dans
  `tests of system/`).

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../evaluation.sh` | l'évaluation finale : 4 vols avec vidéo, bilan et vidéos |
| `../campagne.sh` | les missions de mise au point et leur jugement |
| `../analyse.py` | juge une mission : 5 vérifications, lecture, sécurité, obstacle |
| `../video.py` | assemble les vidéos d'une mission |
| `../cycles.py` | durée des cycles de calcul face à l'inclinaison des drones |
| `../oscillation.py` | oscillation d'attitude lue dans les journaux ArduPilot |
| `../journaux_ardupilot.py` | modes, erreurs et messages des journaux ArduPilot |
| `../cameras_test.py` | vérifie l'orientation des caméras fixes (sans drone) |
| `../tests of system/` | l'évaluation finale : 4 vols (voir ci-dessous) |
| `RESULTATS.md` | les résultats expliqués |

## L'évaluation finale : `tests of system/`

4 vols, un par cas, 3 drones, guide vision-langage débranché :

| Dossier | Cas |
|---|---|
| `../tests of system/eval_nominal/` | entrepôt 9033 (114 codes) |
| `../tests of system/eval_panne/` | entrepôt 9033, le drone 1 tombe en panne à 200 s |
| `../tests of system/eval_9019/` | entrepôt 9019 (65 codes), jamais utilisé pendant la mise au point |
| `../tests of system/eval_obstacle/` | entrepôt 9033, un bloc de 1 × 1 × 2 m apparaît à 200 s |

Chaque dossier contient :

| Fichier | Contenu |
|---|---|
| `mission.json` | le journal complet du vol (réglages, événements, décisions, trajectoires, codes lus dans le temps) |
| `resultats.json` | le bilan écrit par analyse.py |
| `carte_finale.png` | la carte en fin de mission |
| `codes_dans_le_temps.png` | la courbe des codes lus |
| `carte.json`, `carte.npz` | la carte finale (données) |
| `brouillon.json` | sauvegarde du journal faite pendant le vol |
| `instantanes/` | toutes les 10 s simulées : vue annotée (`NNN_vue.png`), images des caméras gauche et droite de chaque drone (`NNN_camI.jpg`, `NNN_camId.jpg`), carte (`NNN_carte.*`) |
| `ardupilot_logs/drone_0/` à `drone_2/` | le journal de bord ArduPilot de chaque drone (`00000001.BIN`) |
| `video/mission.mp4` | la vidéo composée (caméras fixes, caméra du lecteur, temps et codes lus) |
| `video/sud_central.mp4` | la caméra fixe du couloir central |
| `video/sud_grande.mp4` | la caméra fixe de la grande zone |
| `video/lecteur.mp4` | la caméra du drone en train de lire |
| `video/index.json` et `video/lecteur/`, `video/sud_central/`, `video/sud_grande/` | l'index et les images des vidéos |

Pour voir un vol : `../tests of system/eval_nominal/video/mission.mp4`.

Ces 4 dossiers sont lus par `14_rejeu/extrait.py` (rejeu 3D des vols),
`swarm_qr/docs/figures/evaluation/analyse_evaluation.py` (figures du mémoire) et
`swarm_qr/tests/test_pore.py`.

## Comment le lancer

Prérequis : un PC Linux avec carte graphique NVIDIA, Isaac Sim 5.1 + Pegasus + ArduPilot SITL,
un affichage `DISPLAY=:1`. Durée : 25 à 33 min de calcul par vol de l'évaluation, 36 à 57 min
par mission de la campagne. Les outils d'analyse (`analyse.py`, `cycles.py`, `oscillation.py`,
`journaux_ardupilot.py`, `video.py`) tournent sans simulateur ; les deux lecteurs de journaux
demandent `pymavlink`, `video.py` utilise `ffmpeg` s'il est installé.

### L'évaluation finale

Depuis la racine du projet :

    bash swarm_qr/experiments/11_mission/evaluation.sh

Le script vole les 4 cas l'un après l'autre (chaque vol coupé après 2 h par `timeout -s KILL`,
car Isaac Sim ignore le signal d'arrêt normal), puis lance `analyse.py` et
`video.py --sans-images` sur chacun. Un cas dont le dossier contient déjà `mission.json` est
sauté. Il utilise `PY=~/isaac5_env/bin/python` (fixé dans le script).

Un seul cas à la main, exactement comme le fait evaluation.sh (depuis le dossier `swarm_qr`,
`PY` étant le Python de l'environnement Isaac Sim) :

    cd swarm_qr
    $PY mission.py --drones 3 --budget 600 --detecteur auto --video --lam 0.0 --seed 9033 --sortie experiments/11_mission/eval_nominal
    $PY experiments/11_mission/analyse.py --dossier experiments/11_mission/eval_nominal
    $PY experiments/11_mission/video.py --dossier experiments/11_mission/eval_nominal --sans-images

Pour les autres cas, remplacer `--seed 9033` par `--seed 9033 --panne 1:200`, `--seed 9019` ou
`--seed 9033 --obstacle=-4.96,4.0,200` (le `=` est nécessaire à cause du signe moins), et le
nom du dossier de sortie.

### La campagne de mise au point

    bash swarm_qr/experiments/11_mission/campagne.sh nominale
    bash swarm_qr/experiments/11_mission/campagne.sh panne
    bash swarm_qr/experiments/11_mission/campagne.sh autre
    bash swarm_qr/experiments/11_mission/campagne.sh guidee
    bash swarm_qr/experiments/11_mission/campagne.sh guide
    bash swarm_qr/experiments/11_mission/campagne.sh juge NOM

- `nominale`, `panne`, `autre` : 3 drones, 600 s, entrepôt 9033 (`panne` : drone 1 à 200 s)
  ou 9019 (`autre`) ; sorties dans `11_mission/<phase>/`, jugées par analyse.py.
- `guidee` : entrepôt 9019 avec le guide vision-langage smolvlm (poids λ = 1, étape 8).
- `guide` : banc hors ligne du guide (`12_guide/banc.py`) sur les instantanés de `nominale`,
  `panne` et `autre` (il faut donc ces trois dossiers).
- `juge NOM` : analyse.py seul, sur `11_mission/NOM`.
- Chaque mission est coupée après 3 h ; le script refuse de partir si un SITL tourne encore. Il
  utilise `PY=/home/djihene_guitoun/isaac5_env/bin/python` (chemin de la machine de simulation).

### Les outils d'analyse

Depuis la racine du projet (attention : `analyse.py` et `video.py` réécrivent leurs sorties
dans le dossier donné) :

    $PY swarm_qr/experiments/11_mission/analyse.py --dossier "swarm_qr/experiments/11_mission/tests of system/eval_nominal"
    $PY swarm_qr/experiments/11_mission/cycles.py --dossier "swarm_qr/experiments/11_mission/tests of system/eval_nominal"
    $PY swarm_qr/experiments/11_mission/oscillation.py --dossier "swarm_qr/experiments/11_mission/tests of system/eval_nominal"
    $PY swarm_qr/experiments/11_mission/journaux_ardupilot.py --dossier "swarm_qr/experiments/11_mission/tests of system/eval_nominal" --depuis 200
    $PY swarm_qr/experiments/11_mission/video.py --dossier "swarm_qr/experiments/11_mission/tests of system/eval_nominal"
    $PY swarm_qr/experiments/11_mission/cameras_test.py --sortie /tmp/cams

- `analyse.py` : affiche le bilan, écrit `resultats.json` et `codes_dans_le_temps.png`.
- `cycles.py` : durée des cycles de calcul et inclinaison par fenêtre de 20 s.
- `oscillation.py` : écart entre attitude commandée et réelle, verdict « sain » ou « OSCILLATION ».
- `journaux_ardupilot.py` : modes, erreurs et messages des pilotes depuis 200 s.
- `video.py` : refait les vidéos (`--sans-images` efface ensuite les images brutes).
- `cameras_test.py` : demande Isaac Sim ; une image par caméra dans `/tmp/cams`.

## Résultats en bref

- Évaluation finale : **110 / 114** codes en nominal, panne et obstacle ; **60 / 65** sur
  l'entrepôt 9019 ; aucun code inventé ; 90 % des codes lus entre 165 et 206 s simulées.
- Sécurité de l'évaluation : 0 point de trajectoire dans un rack (sur 2 873 à 4 215 par vol),
  1,60 à 3,82 m au plus près entre drones, l'obstacle jamais approché à moins de 0,57 m,
  aucune chute enregistrée.
- Vérifications : 5 sur 5 pour nominal, 9019 et obstacle ; 4 sur 5 pour la panne.
- Campagne de mise au point : nominale 110 / 114 et panne 114 / 114 (5 vérifications sur 5) ;
  sur 9019, 61 / 65 avec trois chutes vers 410 s, puis 63 / 65 sans chute au vol refait.

→ détails, historique des corrections et réserves dans `RESULTATS.md`.
