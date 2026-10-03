# 01 — Le même numéro redonne-t-il le même entrepôt ? (étape 1 : l'environnement de simulation)

## En une phrase
Si l'on construit deux fois l'entrepôt d'un même numéro (la « graine »), obtient-on
exactement le même entrepôt ?

## La tâche
- Plus tard, on comparera deux versions du système sur le même terrain : il faut donc que le
  même numéro redonne exactement le même entrepôt.
- L'entrepôt de la graine 7 est construit deux fois, dans deux processus séparés (passages
  A et B), avec Isaac Sim (le simulateur 3D de NVIDIA). Les drones restent posés : pas de
  pilote automatique.
- À chaque passage, le script enregistre une photo prise d'en haut (toit masqué) et la
  description de l'entrepôt : position des racks, taux de remplissage, nombre de cartons et
  de QR, points de départ des drones.
- Une troisième commande compare les deux passages. La preuve est la description écrite ;
  l'image sert seulement à vérifier qu'aucun objet n'a bougé.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../run.py` | le test : un passage (`--pass A` ou `B`) ou la comparaison (`--compare`) |
| `../layout_A.json`, `../layout_B.json` | la description de l'entrepôt, une par passage |
| `../vue_A.jpg`, `../vue_B.jpg` | la vue de dessus de chaque passage |
| `../comparaison.jpg` | les deux vues côte à côte et la carte de leurs différences |
| `../resultat.json` | le verdict chiffré |
| `../vue_A.ancien.jpg`, `../vue_B.ancien.jpg` | anciennes vues de dessus |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus (extension d'Isaac Sim qui simule les drones), carte
graphique NVIDIA. ArduPilot n'est pas nécessaire.
Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    KIT=--kit_args=--/rtx/verifyDriverVersion/enabled=false
    timeout -s KILL 1200 $PY swarm_qr/experiments/01_reproductibilite/run.py --seed 7 --pass A $KIT
    timeout -s KILL 1200 $PY swarm_qr/experiments/01_reproductibilite/run.py --seed 7 --pass B $KIT
    $PY swarm_qr/experiments/01_reproductibilite/run.py --compare

- Les deux premières commandes écrivent, dans le dossier du test, `vue_A.jpg` et
  `layout_A.json`, puis `vue_B.jpg` et `layout_B.json`.
- La troisième ne démarre pas Isaac Sim : elle écrit `comparaison.jpg` et `resultat.json`
  et affiche le verdict (REPRODUCTIBLE ou NON REPRODUCTIBLE).
- Ce sont les commandes de `../../run_all.sh`, qui lance aussi les tests 2 à 4 :
  `bash swarm_qr/experiments/run_all.sh`.
- `timeout -s KILL 1200` tue Isaac au bout de 20 min : il ignore le signal d'arrêt normal.
- `--kit_args=--/rtx/verifyDriverVersion/enabled=false` désactive le contrôle de version du
  pilote de la carte graphique au démarrage d'Isaac (le script le demande aussi lui-même).

## Résultats en bref
- Description de l'entrepôt identique entre les deux passages, octet par octet.
- Écart de géométrie sur l'image : 0,000 % (seuil toléré : 0,1 %).
- 65 cartons et 130 QR dans les deux cas.
- Verdict : l'entrepôt est reproductible.

→ détails dans `RESULTATS.md`.
