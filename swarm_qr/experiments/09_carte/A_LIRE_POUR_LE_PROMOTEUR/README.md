# 09 — La carte partagée, mémoire commune des drones (étape 4)

## En une phrase
Donner aux drones une carte de l'entrepôt construite en vol, partagée entre eux, et vérifier
qu'elle dit vrai et qu'elle suffit à guider un drone sans jamais consulter le plan.

## La tâche
Avant cette étape, le drone lisait un code puis l'oubliait. La carte (`swarm_qr/mapping.py`)
lui donne une mémoire à deux étages :
- une grille de cubes de 25 cm : chaque cube est occupé, libre ou inconnu, et retient de quel
  côté il a été vu d'assez près pour qu'un code y soit lisible ;
- une table des codes lus, avec leur position au centimètre.

Elle est remplie par le lidar (capteur laser qui mesure les distances) et par les deux caméras
latérales. Le banc tourne dans Isaac Sim (le simulateur 3D) avec ArduPilot SITL (le pilote
automatique simulé sur le PC). Il contrôle d'abord le lidar contre le moteur physique, puis
fait patrouiller un drone le long des racks : chaque trajet est calculé sur la carte qu'il
découvre. Un script d'analyse, sans simulateur, compare ensuite la carte à la vérité.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../campagne.sh` | lance tout : tests, contrôles du lidar, patrouille, analyse |
| `../banc.py` | le banc Isaac Sim : `--mode verifie` (contrôles) ou `--mode vol` (patrouille) |
| `../analyse.py` | le jugement, sans simulateur : carte et vol comparés à la vérité |
| `../vue3d.py` | fabrique la carte en 3D (`carte_3d.html`, `carte_3d.png`) |
| `../verification.json` | résultats des contrôles du lidar |
| `../vol.json` | journal de la patrouille : allers, trajectoire vraie, vérité des panneaux |
| `../carte.npz`, `../carte.json` | la carte finale : les grilles ; les codes lus et les pistes |
| `../resultats.json` | tous les chiffres du jugement |
| `RESULTATS.md` | les résultats expliqués |

Images et vidéo :
- `../carte_qui_se_remplit.mp4` : la carte vue de dessus qui se remplit, aller par aller ;
- `../carte_3d.html` : la carte en 3D, à tourner à la souris, avec le vol à rejouer et la
  vérité à superposer (double-clic pour l'ouvrir dans un navigateur) ;
- `../carte_3d.png` : la même carte en image fixe, deux points de vue ;
- `../comparaison.png` : vue de dessus, vérité en rouge, codes lus en vert, pistes en orange ;
- `../vue_00.png` à `../vue_14.png` : la carte après chacun des 15 allers ;
  `../carte_finale.png` est la dernière ;
- `../vue_un_tour.png` : la carte après un seul tour de lidar (mode `verifie`).

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus + ArduPilot SITL et un écran virtuel (`DISPLAY=:1`) pour
`campagne.sh` et `banc.py`. Rien de tout cela pour `analyse.py` et `vue3d.py` : Python avec
numpy, OpenCV et matplotlib suffit. Durée : au plus 30 min pour les contrôles et 3 h pour la
patrouille (limites fixées dans `campagne.sh`) ; la patrouille dure 597 s de temps simulé.

Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    bash swarm_qr/experiments/09_carte/campagne.sh 2>&1 | tee /tmp/campagne_carte.log

Ce script utilise le Python de la machine de simulation (ligne `PY=`). Il lance les tests
unitaires de `swarm_qr/tests` et s'arrête s'ils échouent ; puis les contrôles, la patrouille et
l'analyse. Il refuse une passe si un SITL tourne encore. Les mêmes étapes, une par une :

    cd swarm_qr/experiments/09_carte
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 1800 $PY banc.py --mode verifie
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 10800 $PY banc.py --mode vol --etages 0,1,2
    $PY analyse.py

- `--mode verifie` écrit `verification.json` et `vue_un_tour.png`. Il s'arrête sur une erreur
  si un contrôle échoue.
- `--mode vol` écrit `carte.npz`, `carte.json`, `vol.json`, `vue_NN.png`, `carte_finale.png`
  et `carte_qui_se_remplit.mp4`.
- `analyse.py` écrit `resultats.json`, `comparaison.png`, `carte_3d.html` et `carte_3d.png`.
  Option `--dossier` pour juger une carte rangée ailleurs.
- `vue3d.py` seul (`$PY vue3d.py`, depuis ce dossier) refait la page 3D et l'image fixe.
- Autres options de `banc.py` : `--seed` (entrepôt, 9033 par défaut), `--allees` (nombre
  d'allées, 0 = toutes), `--detecteur` (`auto` = détecteur appris de l'étape 7, ou un `.pt` ;
  vide = repérage classique), `--sortie` (dossier des sorties du mode vol).
- `banc.py` démarre Isaac Sim lui-même avec `--/rtx/verifyDriverVersion/enabled=false` :
  inutile de l'ajouter. `timeout -s KILL` est nécessaire car Isaac Sim ignore l'arrêt normal.

## Résultats en bref
- Le lidar dit vrai : 0,0 cm d'écart médian avec le moteur physique.
- 89,5 % de l'entrepôt connu ; 110 codes lus sur 114, aucun inventé.
- Position d'une face lue : 0,9 cm en médiane, 8,3 cm au pire.
- La carte tient sa promesse de lisibilité dans 95 % des cas ; 0,39 % de fausses cases
  occupées en pleine allée.
- Guidé par sa seule carte, le drone réussit ses 15 allers sur 15, et aucun des 2 738 points
  de sa trajectoire n'entre dans un rack.

Ces chiffres ont été mesurés avant les changements de l'étape 7 → détails dans `RESULTATS.md`.
