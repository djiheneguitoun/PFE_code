# 13 — Le système face à la méthode de référence de Pore et al. (comparaison)

## En une phrase
Sur le même simulateur et dans les 4 mêmes cas que l'évaluation finale du système, une
méthode publiée récente (Pore, Patle et Thorat, Symmetry 2026, 18(4):548) lit-elle
l'inventaire mieux ou moins bien que le système ?

## La tâche
La méthode de l'article a été réécrite pour notre simulateur : `swarm_qr/pore.py` (ses
briques) et `swarm_qr/pore_mission.py` (son déroulé de mission, séparé de celui du système).

Ce qui distingue les deux méthodes :
- **La référence suit un plan figé.** Elle reçoit le plan de l'entrepôt et calcule, avant le
  décollage, un parcours en zigzag : un rack par drone, un arrêt tous les 1,5 m à chaque
  étage, 2 s d'arrêt devant chaque face. En vol, les drones déroulent ce plan dans l'ordre.
- **Le système décide à partir de la carte.** Il ne connaît pas le plan. Chaque drone construit
  la carte partagée en vol et choisit sa prochaine cible sur cette carte : lire un QR repéré de
  loin, regarder de près une surface jamais vue, explorer l'inconnu. Les cibles d'un drone en
  panne se libèrent seules.

Les 4 cas, avec les mêmes paramètres que pour le système (3 drones, 600 s simulées au plus) :
nominal (entrepôt 9033), panne du drone 1 à 200 s, entrepôt 9019 jamais vu pendant la mise au
point, obstacle de 1 × 1 × 2 m posé à 200 s. Chaque vol est jugé après coup par `../analyse.py`,
avec les mêmes mesures que le juge du système (`../../11_mission/analyse.py`).

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../campagne.sh` | vole les 4 cas de la référence et fabrique leurs vidéos |
| `../analyse.py` | juge un vol (sans simulateur) et trace ses deux figures |
| `../comparaison.md` | tableau récapitulatif système / référence |
| `../pore_nominal/`, `../pore_panne/`, `../pore_9019/`, `../pore_obstacle/` | un dossier par cas |
| `RESULTATS.md` | les résultats expliqués |

Dans chaque dossier `pore_*` :
- `mission.json` : le journal complet du vol ;
- `resultats.json` : le bilan chiffré du juge ;
- `codes_dans_le_temps.png` : la part des codes lus au fil du temps ;
- `plan_et_vol.png` : le plan connu, les arrêts prévus, le vol réel et les codes non lus ;
- `video/mission.mp4` : le film du vol, à vitesse réelle (caméras fixes de couloir, caméra de
  lecture, temps et compteur de codes en bandeau) ;
- `video/lecteur.mp4` : la caméra frontale du drone en train de lire ;
- `video/sud_central.mp4`, `video/sud_grande.mp4` : deux caméras fixes (couloir central et
  grande zone, vues du sud) ; `video/index.json` : l'index des images.

Les résultats du système sur les mêmes cas sont dans
`../../11_mission/tests of system/eval_*/` (mêmes fichiers, plus la carte). Les 8 vols se
revoient aussi en 3D : `../../14_rejeu/rejeu_hors_ligne.html`.

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus + ArduPilot SITL (le pilote automatique simulé sur le PC)
et un écran virtuel (`DISPLAY=:1`) pour les vols ; `analyse.py` n'a besoin que de Python avec
numpy, OpenCV et matplotlib. Durée : 33 à 50 min de calcul par vol (mesuré).

Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    bash swarm_qr/experiments/13_pore/campagne.sh
    $PY swarm_qr/experiments/13_pore/analyse.py --dossier swarm_qr/experiments/13_pore/pore_nominal

- `campagne.sh` utilise `~/isaac5_env/bin/python` (ligne `PY=`). Pour chaque cas, il
  lance depuis `swarm_qr/` :
  `timeout -s KILL 7200 $PY pore_mission.py --drones 3 --budget 600 --video <options> --sortie <dossier>`,
  puis `experiments/11_mission/video.py --dossier <dossier> --sans-images`, qui assemble les
  vidéos et efface les images. Options par cas : `--seed 9033` ; `--seed 9033 --panne 1:200` ;
  `--seed 9019` ; `--seed 9033 --obstacle=-4.96,4.0,200` (le `=` est nécessaire car la valeur
  commence par un signe moins).
- Un cas déjà fait (son `mission.json` existe) est sauté.
- Le jugement n'est pas dans la campagne : lancer `analyse.py` sur chaque dossier. Il lit
  `mission.json` et écrit `resultats.json`, `codes_dans_le_temps.png` et `plan_et_vol.png`.
- `pore_mission.py` démarre Isaac Sim lui-même avec `--/rtx/verifyDriverVersion/enabled=false`.
  `timeout -s KILL` est nécessaire car Isaac Sim ignore l'arrêt normal.

## Résultats en bref
- Codes lus, système contre référence : 110 contre 106 sur 114 (nominal), 110 contre 75
  (panne), 60 contre 53 sur 65 (entrepôt 9019), 110 contre 103 (obstacle).
- 90 % de l'inventaire : en 165 à 206 s pour le système ; en 515 et 534 s pour la référence,
  qui ne l'atteint jamais en cas de panne ni sur l'entrepôt 9019.
- Panne : les arrêts du drone en panne ne sont repris par personne (104 arrêts servis sur
  180) ; dans le système, les deux autres drones reprennent sa zone.
- Aucune chute ni aucun point dans un rack, pour les deux méthodes ; l'obstacle est évité par
  les deux. La référence garde ses drones plus loin les uns des autres (2,49 à 4,63 m contre
  1,60 à 3,82 m).

→ détails dans `RESULTATS.md`.
