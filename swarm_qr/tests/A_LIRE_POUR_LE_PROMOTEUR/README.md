# tests — Les tests unitaires du système final, sans simulateur (étapes 3, 4, 5, 7 et référence)

## En une phrase
Les pièces du système final (le contrôleur de vol, la carte partagée, le cerveau, le détecteur
et la méthode de référence de Pore et al.) font-elles bien ce qu'on attend d'elles ? 89 tests
le vérifient en moins de 30 secondes, sur un PC ordinaire, sans simulateur.

## La tâche
Un test unitaire est un petit programme qui place une fonction dans une situation dont on
connaît la bonne réponse, puis vérifie qu'elle donne cette réponse. L'outil `pytest` lance tous
les tests et compte ceux qui passent.

Pourquoi c'est important : une mission dans Isaac Sim demande 25 à 60 minutes de calcul. Une
erreur dans la carte ou dans le cerveau ne se verrait qu'après un vol entier, et serait
difficile à retrouver. Ici, chaque pièce est vérifiée seule, **sans simulateur**, sur de petits
cas construits à la main :
- le contrôleur pilote un faux drone (un point qui suit la vitesse commandée avec un retard) ;
- la carte est remplie par un lidar (capteur laser qui mesure les distances) simulé sans erreur
  dans un monde de boîtes : on connaît la vérité, donc on peut juger la carte case par case ;
- le cerveau choisit sa cible sur des cartes « jouets » écrites à la main (un couloir, une
  salle avec un mur), dont on connaît la bonne réponse ;
- la méthode de référence travaille sur les plans d'entrepôt, calculés en Python pur à partir
  de la graine (graines 9033 et 9019), et sur de vraies images de QR fabriquées par OpenCV.

Rien ne vole : ni Isaac Sim, ni ArduPilot, ni carte graphique. Les vols sont dans
`../../experiments/`.

Ces tests servent aussi de garde-fou : la campagne de l'étape 4
(`../../experiments/09_carte/campagne.sh`) les lance avant de voler et s'arrête s'ils échouent.

## Ce que vérifie chaque fichier
- `../test_control.py` (14 tests) — le contrôleur de vol (`../../control.py`, étape 3). Le
  drone rejoint une pose et s'y tient ; il abandonne s'il est bloqué contre un mur ou si son
  temps est écoulé ; il garde sa position après l'arrivée comme après un abandon ; deux drones
  volent dans la même boucle. Les trois façons de s'arrêter comparées à l'étape 3 sont testées.
- `../test_detecteur.py` (1 test) — le détecteur appris (`../../detecteur.py`, étape 7) : le
  centre et la taille d'un cadre de détection. Le réseau YOLO lui-même n'est pas chargé.
- `../test_mapping.py` (31 tests) — la carte partagée (`../../mapping.py`, étape 4). Elle note
  ce que voit le lidar sans rien inventer derrière un mur ; elle retient ce que les caméras ont
  vu d'assez près pour lire, et de quel côté ; elle range les QR lus et ceux seulement repérés ;
  elle gère les réservations entre drones et la panne d'un drone ; elle calcule des chemins qui
  contournent les obstacles.
- `../test_planning.py` (14 tests) — le cerveau (`../../planning.py`, étape 5), qui choisit la
  prochaine cible de chaque drone : lire un QR repéré avant d'explorer, laisser à un coéquipier
  ce qu'il a réservé, trois essais par QR puis abandon, aucune pose trop près d'un obstacle,
  aucun survol de rack.
- `../test_pore.py` (29 tests) — la méthode de référence de Pore et al. (`../../pore.py`,
  comparée à notre système dans `../../experiments/13_pore/`). La plupart des tests citent la
  phrase de l'article qu'ils contrôlent. Le premier vérifie que ce module n'emprunte rien à
  notre système : c'est la condition d'une comparaison honnête.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../test_control.py` | 14 tests du contrôleur |
| `../test_detecteur.py` | 1 test du détecteur |
| `../test_mapping.py` | 31 tests de la carte |
| `../test_planning.py` | 14 tests du cerveau |
| `../test_pore.py` | 29 tests de la référence |
| `RESULTATS.md` | le dernier résultat, expliqué fichier par fichier |

Chaque fichier commence par un en-tête qui dit ce qu'il vérifie, et chaque test porte une
phrase qui décrit la situation testée et la valeur attendue.

## Comment le lancer
Prérequis : Python 3.10 ou plus, avec `numpy`, `opencv-python` et `pytest`. Pas besoin
d'Isaac Sim, ni d'ArduPilot, ni de carte graphique : `test_detecteur.py` n'importe pas
`ultralytics` (la bibliothèque du réseau YOLO). Durée : environ 25 secondes.

Depuis la racine du projet :

    pip install numpy opencv-python pytest
    python -m pytest swarm_qr/tests -q

La dernière ligne affichée doit être `89 passed`, suivie de la durée. Rien n'est écrit dans
le projet, sauf les caches de Python et de pytest (`__pycache__`, `.pytest_cache`).

Un seul fichier : `python -m pytest swarm_qr/tests/test_mapping.py -q` (même principe pour
les quatre autres).

Sur la machine de simulation, le Python d'Isaac Sim convient aussi :
`~/isaac5_env/bin/python -m pytest swarm_qr/tests -q`.

Trois tests de `test_pore.py` lisent des fichiers du projet, qui doivent rester en place : le
code source de `../../pore.py`, `../../pore_mission.py`, `../../control.py`,
`../../env/pilot.py` et `../../env/scene.py`, et les journaux des vols 9033 et 9019 de
l'évaluation finale (`../../experiments/11_mission/tests of system/eval_nominal/mission.json`
et `.../eval_9019/mission.json`).

## Résultats en bref
- 89 tests sur 89 réussis, en 24,86 s (Windows, Python 3.13.3, le 3 octobre 2026).
- Répartition : contrôleur 14, détecteur 1, carte 31, cerveau 14, référence 29.
- Aucun échec, aucun test sauté.
→ détails dans `RESULTATS.md`.
