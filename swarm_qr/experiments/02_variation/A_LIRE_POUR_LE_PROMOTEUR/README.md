# 02 — Les entrepôts sont-ils vraiment différents entre eux ? (étape 1 : l'environnement de simulation)

## En une phrase
Des numéros (« graines ») différents donnent-ils des entrepôts vraiment différents ?

## La tâche
- Si tous les entrepôts se ressemblent, dire que le système « généralise » (fonctionne dans
  un entrepôt qu'il n'a jamais vu) ne veut rien dire : il aurait toujours vu la même chose.
- Six entrepôts (graines 1 à 6) sont construits avec Isaac Sim, un par lancement, et
  photographiés d'en haut. Les drones restent posés.
- Pour chaque paire d'entrepôts, on mesure de combien les trois racks ont bougé en moyenne ;
  on compare aussi le nombre de cartons.
- Verdict « variation suffisante » si la paire la plus proche diffère de plus de 1 m et si
  le nombre de cartons varie de plus de 20 d'un entrepôt à l'autre.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../run.py` | le test : une graine (`--seed N`) ou la planche et le verdict (`--board`) |
| `../layout_1.json` … `../layout_6.json` | racks, cartons et QR de chaque graine |
| `../vue_1.jpg` … `../vue_6.jpg` | la vue de dessus de chaque entrepôt |
| `../planche_variation.jpg` | les six vues côte à côte |
| `../resultat.json` | le verdict chiffré |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus (extension d'Isaac Sim qui simule les drones), carte
graphique NVIDIA. ArduPilot n'est pas nécessaire.
Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    KIT=--kit_args=--/rtx/verifyDriverVersion/enabled=false
    for s in 1 2 3 4 5 6; do timeout -s KILL 1200 $PY swarm_qr/experiments/02_variation/run.py --seed $s $KIT; done
    $PY swarm_qr/experiments/02_variation/run.py --board

- Chaque lancement `--seed N` écrit `vue_N.jpg` et `layout_N.json` dans le dossier du test.
- `--board` ne démarre pas Isaac Sim : il écrit `planche_variation.jpg` et `resultat.json`
  et affiche le verdict (VARIATION SUFFISANTE ou INSUFFISANTE).
- Ce sont les commandes de `../../run_all.sh` (tests 1 à 4) :
  `bash swarm_qr/experiments/run_all.sh`.
- `timeout -s KILL 1200` tue Isaac au bout de 20 min : il ignore le signal d'arrêt normal.
- `--kit_args=--/rtx/verifyDriverVersion/enabled=false` désactive le contrôle de version du
  pilote de la carte graphique au démarrage d'Isaac (le script le demande aussi lui-même).

## Résultats en bref
- Paire la plus proche (graines 1 et 6) : racks déplacés de 1,85 m en moyenne (seuil 1 m).
- Écart-type des positions des racks : 1,73 m en largeur, 3,27 m en profondeur.
- De 62 à 118 cartons selon l'entrepôt.
- Verdict : la variation est suffisante.

→ détails dans `RESULTATS.md`.
