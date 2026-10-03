# 04 — Combien de temps coûte une simulation ? (étape 1 : l'environnement de simulation)

## En une phrase
Combien de pas de simulation par seconde la machine tient-elle sur la scène complète, et
que coûtent les images des caméras ?

## La tâche
- Ce chiffre gouverne tout le projet : il dit combien d'heures coûtera l'évaluation finale.
- La physique avance par petits pas de 1/800 s, le réglage officiel d'ArduPilot : il faut
  800 pas pour une seconde simulée.
- Scène complète de la graine 7 : 3 drones et 9 caméras (3 par drone).
- Trois situations chronométrées dans un seul lancement d'Isaac Sim : sans aucune image,
  avec 5 images par seconde (le régime des missions), et avec une image à chaque pas.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../run.py` | la mesure, ou le tableau des résultats (`--plot`) |
| `../mesures.json` | les pas par seconde de chaque situation |
| `../courbe_debit.png` | la figure qui compare les trois situations |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus (extension d'Isaac Sim qui simule les drones), carte
graphique NVIDIA. ArduPilot n'est pas nécessaire.
Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    KIT=--kit_args=--/rtx/verifyDriverVersion/enabled=false
    timeout -s KILL 1200 $PY swarm_qr/experiments/04_debit/run.py $KIT
    $PY swarm_qr/experiments/04_debit/run.py --plot

- La première commande mesure les trois situations et écrit `mesures.json`. Option
  `--steps N` : nombre de pas chronométrés par situation (1600 par défaut ; 8 fois moins,
  200 au moins, pour l'image à chaque pas).
- `--plot` ne démarre pas Isaac Sim : il affiche le tableau (pas/s, vitesse par rapport au
  temps réel, durée de calcul d'une mission de 10 min).
- `timeout -s KILL 1200` tue Isaac au bout de 20 min : il ignore le signal d'arrêt normal.
- `--kit_args=--/rtx/verifyDriverVersion/enabled=false` désactive le contrôle de version du
  pilote de la carte graphique au démarrage d'Isaac (le script le demande aussi lui-même).

## Résultats en bref
- Sans image : 222 pas/s ; 5 images/s : 206 pas/s (environ 7 % de moins) ; une image à
  chaque pas : 6 pas/s.
- La simulation tourne environ 4 fois moins vite que le temps réel : une mission de
  10 min demande environ 40 min de calcul, 25 missions environ 16 h.
- Le coût vient de la physique du pilote automatique, calculée 800 fois par seconde, pas
  des caméras. Réglage retenu : 5 images/s.

→ détails dans `RESULTATS.md`.
