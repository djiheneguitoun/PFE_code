# 06 — Où est exactement la caméra sur le drone ? (étape 1 : l'environnement de simulation)

## En une phrase
La position du drone, celle de sa caméra et la distance lue dans l'image concordent-elles,
et où la caméra est-elle montée exactement ?

## La tâche
- Au test 3, la taille du QR dans l'image indiquait une distance d'environ deux tiers de
  celle que le pilote croyait avoir : il fallait savoir qui avait raison.
- Un drone piloté par SITL (le pilote automatique ArduPilot simulé sur le PC), dans
  l'entrepôt de la graine 7, se place face à un panneau QR à 1, 2 puis 3 m (distance
  comptée depuis le centre du drone).
- À chaque point, trois estimations indépendantes de la distance au panneau :
  - le centre du drone, position lue dans le simulateur (celle qu'utilise le pilote) ;
  - la caméra gauche, position lue dans le simulateur ;
  - la distance « optique », déduite de la taille du code dans l'image (plus le code paraît
    petit, plus il est loin).
- La distance optique sert de référence : une caméra fixe placée à des distances connues la
  retrouve à 1 % près.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../run.py` | le test (un seul vol) |
| `../resultat.json` | les trois distances et les positions à chaque point |
| `../vue_1.0m.jpg`, `../vue_2.0m.jpg`, `../vue_3.0m.jpg` | l'image de la caméra gauche à chaque point |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus + ArduPilot SITL, carte graphique NVIDIA, et un
affichage graphique (`DISPLAY=:1`) : le lancement automatique d'ArduPilot ouvre un terminal.
`run_all.sh` ne lance pas ce test.
Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    DISPLAY=:1 $PY swarm_qr/experiments/06_position_vraie/run.py --seed 7

- Écrit `vue_1.0m.jpg`, `vue_2.0m.jpg`, `vue_3.0m.jpg` et `resultat.json` dans le dossier du
  test, et affiche un tableau : distance visée, pilote, caméra, optique, écart du pilote.
- Le script demande lui-même `--/rtx/verifyDriverVersion/enabled=false` au démarrage d'Isaac.
  Comme dans `run_all.sh`, on peut le préfixer par `timeout -s KILL <secondes>` : Isaac
  ignore le signal d'arrêt normal.

## Résultats en bref
- La caméra gauche est montée 10 cm sur le côté du drone (vers le panneau) et 11 cm sous son
  centre : mesuré en vol, aux trois points.
- Distance de la caméra et distance optique concordent : écart de 0,2 à 4,3 cm.
- La distance du centre du drone dépasse toujours celle de la caméra de 10 cm : c'est le
  montage de la caméra, pas une erreur de position.
- La distance optique vaut 89 à 96 % de celle du centre du drone : l'écart vient du montage
  de la caméra.
- Ces deux valeurs sont celles de `swarm_qr/env/config.py` (« vérifié en vol
  (06_position_vraie) »).

→ détails dans `RESULTATS.md`.
