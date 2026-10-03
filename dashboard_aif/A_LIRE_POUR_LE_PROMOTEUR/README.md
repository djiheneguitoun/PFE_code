# dashboard_aif — Tableau de bord de l'inférence active (approche AIF, avant swarm_qr)

## En une phrase
Une page web pour suivre en direct un vol d'exploration AIF à 3 drones dans Isaac Sim, et pour
revoir les vols déjà enregistrés dans `../../logs/runs/`.

## La tâche
Avant la contribution finale (`swarm_qr/`), l'essaim explorait l'entrepôt par inférence active
(AIF : à chaque décision, chaque drone choisit le déplacement qui réduit le plus son incertitude
sur la carte ; le score utilisé s'appelle « énergie libre attendue »). Le code est dans
`../../scripts/aif_core/` et la simulation se lance avec `../../scripts/12_aif_isaac_sim.py`.

La carte partagée est une « carte de croyance » : une grille où chaque case a une probabilité
d'être occupée (0 = libre, 0,5 = inconnue, 1 = occupée), fusionnée entre les drones. La
simulation peut aussi provoquer des pannes (drone arrêté, lien vers le cloud coupé, liens entre
drones coupés, obstacle ajouté) pour tester la résilience : la phase passe alors de « normal » à
« recovery » (récupération), puis éventuellement « durable ».

Ce tableau de bord ne calcule rien : toutes les 0,5 s, il relit les fichiers que la simulation
écrit dans `/tmp`, et il liste les dossiers de vols de `logs/runs/`. Il n'a pas de résultats
propres ; les résultats des vols sont dans `../../logs/runs/`.

## Ce que montre la page (4 onglets)

| Onglet | Contenu |
|---|---|
| Live | le vol en cours : carte, chiffres, caméras et QR codes, 6 graphiques, tableau des drones |
| Resilience | phase actuelle, début du stress, cause ; courbes avec bandes de couleur par phase |
| NS-3 | réseau simulé : lien cloud, messages envoyés / livrés / perdus, latences entre drones |
| Runs | liste des vols enregistrés ; un clic affiche toutes leurs figures (PNG) |

Détail de l'onglet Live :
- carte de croyance recadrée sur les murs détectés (vert = libre, bleu nuit = inconnu,
  rouge = occupé), avec traces, position et cap des drones ;
- chiffres : step (numéro de décision), couverture (% de la carte connue), entropie moyenne
  (incertitude de la carte), gain d'information, innovation (écart entre les nouvelles mesures et
  la carte), drones actifs, phase de résilience ;
- une carte par caméra de drone : image brute ou annotée (QR encadré), état du décodage
  (DECODED, FAILED, CACHED = texte repris du cache), texte lu, compteurs ;
- chiffres QR : images traitées, décodées, échecs, réponses du cache, taux de réussite, dernier
  texte lu ;
- graphiques : entropie, couverture, énergie libre attendue par drone, gain d'information,
  innovation, taux de réussite QR.

## D'où viennent les données

| Fichier lu | Écrit par | Onglet |
|---|---|---|
| `/tmp/aif_state.json` | `scripts/12_aif_isaac_sim.py` (via `aif_core/loggers.py`) | Live, Resilience, NS-3 |
| `/tmp/aif_history.json` | idem (une ligne de métriques par step) | Live, Resilience |
| `/tmp/qr_state.json` | `scripts/qr_code_system.py`, lancé par le script 12 | Live |
| `/tmp/camera_frames/latest_drone_<id>.jpg`, `annotated_drone_<id>.jpg` | `scripts/qr_code_system.py` | Live |
| `/tmp/ns3_output.csv` (WiFi), `/tmp/drone_latency_ns3.csv`, `/tmp/drone_5g_metrics.csv` (5G) | NS-3, lancé par le script 12 quand `--ns3 wifi` ou `--ns3 5g` | NS-3 |
| `logs/runs/run_<date>_<tag>/` (`config.json` + PNG) | fin de chaque vol (`scripts/run_artifacts.py`) | Runs |

Routes du serveur (lues par la page) : `/api/state`, `/api/history`, `/api/qr_state`,
`/api/ns3`, `/api/runs`, `/api/frame/<image>` (images des caméras) et
`/api/run/<tag>/img/<nom>` (figures d'un vol).

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../server.py` | serveur web : envoie la page, les fichiers JSON de `/tmp`, les images et la liste des vols |
| `../index.html` | structure de la page (4 onglets) |
| `../app.js` | logique de la page : interroge le serveur toutes les 0,5 s et redessine |
| `../style.css` | apparence (thème sombre ; vert = normal, rouge = recovery, jaune = durable) |

(Le dossier `../__pycache__/` est un cache Python créé automatiquement, sans intérêt.)

## Comment le lancer
Prérequis : Python 3 seul (le serveur n'utilise que la bibliothèque standard), et un navigateur
qui a accès à Internet (la bibliothèque de graphiques Chart.js est chargée depuis Internet).

Depuis la racine du projet :

    python3 dashboard_aif/server.py

Options (tirées du code du serveur) :

    python3 dashboard_aif/server.py --port 8061                     (autre port ; défaut 8060)
    python3 dashboard_aif/server.py --runs-dir /autre/dossier/runs  (défaut : logs/runs du projet)

Le dossier des vols peut aussi être donné par la variable d'environnement `AIF_RUNS_DIR`.
Le serveur affiche les fichiers qu'il lit ; Ctrl+C l'arrête.

Pour revoir les vols enregistrés, aucun simulateur n'est nécessaire : lancer le serveur, ouvrir
la page, aller dans l'onglet « Runs ». Le dépôt en contient 8, chacun avec 11 figures :

| Vol (dossier de `logs/runs/`) | Essai |
|---|---|
| `run_20260531_041619_aif_cent_baseline` | AIF, centralisé, sans panne |
| `run_20260531_042411_aif_dist_baseline` | AIF, distribué, sans panne |
| `run_20260531_043247_heur_cent_baseline` | planificateur heuristique, centralisé |
| `run_20260531_043951_aif_cent_cloud_loaded` | AIF, cloud chargé (latence plus forte) |
| `run_20260531_044951_aif_cent_kill_d0_s20` | drone 0 arrêté au step 20 |
| `run_20260531_075736_aif_cent_cut_cloud_s20` | lien cloud coupé au step 20 |
| `run_20260531_100239_aif_dist_cut_links_s20` | liens entre drones coupés au step 20 |
| `run_20260608_042211_aif_cent_obstacle_s20` | obstacle ajouté au step 20 |

Pour des données en direct, lancer la simulation dans un autre terminal sur la machine de
simulation (Isaac Sim, Pegasus, ArduPilot SITL compilé dans `~/ardupilot`, GPU NVIDIA) :

    scripts/12_launch_aif_isaac_sim.sh 3

Le premier argument est le nombre de drones ; les options suivantes sont transmises à
`12_aif_isaac_sim.py` (par exemple `--planner aif|heuristic`, `--arch centralized|distributed`,
`--ns3 none|wifi|5g`, `--max-steps 500`, `--run-tag nom`). Le lanceur active l'environnement
`~/isaac_sim_env/activate_isaac.sh`. Toute la série d'essais se relance avec
`scripts/run_all_experiments.sh` (par défaut 3 drones, 80 steps).

## Comment l'ouvrir dans un navigateur
- Sur la même machine : `http://localhost:8060`
- Depuis un autre PC du même réseau : `http://<adresse IP de la machine>:8060` (le serveur
  écoute sur toutes les interfaces).
- Sans simulation en cours, le voyant reste « Offline » : c'est normal, seul l'onglet Runs a
  alors du contenu.

## À savoir
- Les fichiers en direct sont lus dans `/tmp` : le suivi en direct se fait sur la machine de
  simulation (Linux).
