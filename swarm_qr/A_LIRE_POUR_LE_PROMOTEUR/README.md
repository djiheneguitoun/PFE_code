# swarm_qr — le système final (contribution principale du PFE)

## En une phrase

Trois drones simulés explorent un entrepôt qu'ils n'ont jamais vu, lisent les QR codes collés
sur les cartons et s'arrêtent seuls quand l'inventaire est fait, sans aucun plan de vol calculé
à l'avance.

## La tâche

- L'entrepôt est simulé dans Isaac Sim 5.1. Un numéro (la « graine ») fixe la position des
  racks et des cartons : chaque numéro donne un entrepôt différent, et le même numéro redonne
  exactement le même entrepôt.
- Un vrai QR code est collé sur les deux faces de chaque carton (`BOX_000`, `BOX_001`…).
- Chaque drone est un Iris (simulateur Pegasus) piloté par un vrai pilote automatique ArduPilot
  simulé sur le PC (SITL). Le système lui envoie des consignes de vitesse par MAVLink : rien
  n'est téléporté, le drone a son inertie et ses imprécisions.
- Capteurs de chaque drone : deux caméras latérales 1024 × 768 (pour lire les QR), un lidar
  360° (capteur laser qui mesure les distances), sa position.
- Les drones décident eux-mêmes où aller, à partir de ce qu'ils ont observé. Ils doivent aussi
  supporter la panne d'un drone, un obstacle qui apparaît pendant la mission, et un entrepôt
  jamais vu.

## Comment ça marche : une boucle en 5 temps

1. Les drones observent (lidar, caméras) et écrivent ce qu'ils voient dans une **carte partagée**.
2. La carte dit ce qui reste à faire : QR repérés mais pas encore lus, faces de rack jamais
   regardées, zones encore inconnues.
3. Chaque drone donne une note à chaque cible possible et **réserve** la meilleure. Une
   réservation expire seule si elle n'est pas renouvelée.
4. Le contrôleur de vol amène le drone devant sa cible, ou abandonne si c'est impossible.
5. Ce que le drone a appris repart dans la carte, et la boucle recommence.

Deux règles font tout le travail :
- rien n'est écrit dans la carte sans avoir été observé ;
- rien n'est réservé plus longtemps que la réservation n'est renouvelée.

Grâce à elles, la panne d'un drone (ses réservations expirent, les autres reprennent sa zone)
et l'obstacle imprévu (le lidar le voit, les chemins sont recalculés) sont gérés sans aucun code
spécial.

Figure de l'architecture : `../docs/figures/architecture.png`

## Contenu du dossier

| Fichier / dossier | Rôle |
|---|---|
| `../mission.py` | lance une mission complète : le chef d'orchestre (point d'entrée) |
| `../mapping.py` | la carte partagée (grille 3D de cases de 25 cm) et le calcul des chemins (A*) |
| `../planning.py` | le « cerveau » : choisit la prochaine cible de chaque drone |
| `../control.py` | le contrôleur de vol : transit, approche, tenue, abandon |
| `../observation.py` | verse dans la carte ce que voient les capteurs d'un drone |
| `../perception.py` | lit un QR dans une image et calcule sa position 3D |
| `../detecteur.py` | réseau YOLO qui repère QR et cartons de loin, sans les lire |
| `../guide.py` | guide vision-langage optionnel (poids λ ; non utilisé dans l'évaluation finale) |
| `../pore.py`, `../pore_mission.py` | méthode de référence (Pore et al., 2026), réécrite pour comparer |
| `../env/` | le simulateur : constantes (`config.py`), entrepôt selon la graine (`layout.py`), pilote ArduPilot (`pilot.py`), QR (`qr_tags.py`), scène Isaac Sim (`scene.py`) |
| `../assets/detecteur/` | le détecteur YOLO entraîné (`detecteur.pt`) et ses réglages |
| `../assets/qr/` | les images des QR (recréées à chaque construction de scène) |
| `../tests/` | 89 tests unitaires qui tournent sans simulateur |
| `../experiments/` | les 14 expériences, étape par étape |
| `../docs/figures/` | les figures du mémoire et leurs scripts |
| `RESULTATS.md` | les résultats finaux (ce dossier) |

## Ordre de lecture conseillé (environ 1 h)

1. Ce README, puis `RESULTATS.md`.
2. Voir les drones voler :
   - le rejeu 3D des 8 vols : `../experiments/14_rejeu/rejeu_hors_ligne.html` (double-clic, sans Internet) ;
   - la vidéo d'un vol : `../experiments/11_mission/tests of system/eval_nominal/video/mission.mp4`.
3. Le code, dans cet ordre : `mission.py` (fonction `main`), `mapping.py`, `planning.py`,
   `observation.py`, `control.py`, puis `pore.py` pour la méthode de référence.
4. Les expériences, étape par étape : `../experiments/A_LIRE_POUR_LE_PROMOTEUR/README.md`.

## Comment le lancer

### Prérequis

- **Tests unitaires** : Python 3.10 ou plus, `numpy`, `opencv-python`, `pytest`. Ni simulateur,
  ni carte graphique.
- **Une mission** : un PC Linux avec une carte graphique NVIDIA RTX, Isaac Sim 5.1, Pegasus
  Simulator et ArduPilot SITL. Sur la machine de simulation, tout cela vit dans l'environnement
  Python `~/isaac5_env`. ArduPilot SITL s'installe avec
  `scripts/installation/03_install_ardupilot_sitl.sh`.
  Paquets Python nécessaires dans cet environnement : `pymavlink`, `scipy`, `opencv-python`,
  `qrcode`, `zxing-cpp`, `ultralytics` ; en option `pyzbar` (autre décodeur), et pour le guide
  `torch`, `transformers`, `peft`, `bitsandbytes`.
- Il faut un affichage graphique (les scripts fixent `DISPLAY=:1`) : Pegasus ouvre une fenêtre
  de terminal pour chaque pilote ArduPilot.
- Durée : une mission de 600 s simulées demande environ 25 à 60 min de calcul (la simulation à
  3 drones tourne 5 à 8 fois plus lentement que le temps réel).

### Commandes

Depuis la racine du projet. `PY` est le Python de l'environnement Isaac Sim (sur la machine de
simulation : `PY=~/isaac5_env/bin/python`).

Les tests (sans simulateur) :

    python -m pytest swarm_qr/tests -q

Une mission : 3 drones, entrepôt 9033, 600 s simulées, détecteur appris, vidéo enregistrée.
Puis son bilan (5 vérifications, codes lus, sécurité) et ses vidéos :

    $PY swarm_qr/mission.py --seed 9033 --drones 3 --budget 600 --detecteur auto --video --sortie sorties/ma_mission
    $PY swarm_qr/experiments/11_mission/analyse.py --dossier sorties/ma_mission
    $PY swarm_qr/experiments/11_mission/video.py --dossier sorties/ma_mission --sans-images

Options utiles de `mission.py` :

| Option | Effet |
|---|---|
| `--seed 9019` | un autre entrepôt (jamais vu pendant la mise au point) |
| `--panne 1:200` | le drone 1 tombe en panne à 200 s simulées |
| `--obstacle=-4.96,4.0,200` | un bloc de 1 × 1 × 2 m apparaît en (−4,96 ; 4,0) à 200 s (le `=` est nécessaire à cause du signe moins) |
| `--drones 2` | le nombre de drones |
| `--guide smolvlm --lam 1.0` | branche le guide vision-langage, avec le poids λ = 1 |

Les 4 vols de l'évaluation finale (nominal, panne, entrepôt 9019, obstacle), avec bilan et vidéos :

    bash swarm_qr/experiments/11_mission/evaluation.sh

La méthode de référence de Pore et al. sur les mêmes 4 cas :

    bash swarm_qr/experiments/13_pore/campagne.sh

Ces scripts utilisent le Python de la machine de simulation (`~/isaac5_env/bin/python`, fixé
par la ligne `PY=` en tête de chaque script : à adapter sur une autre machine). Ils arrêtent
Isaac Sim avec `timeout -s KILL`, car il ignore le signal d'arrêt normal.

## Où sont les résultats et les vidéos

- Les résultats finaux expliqués : `RESULTATS.md` (ce dossier).
- Les 4 vols de l'évaluation finale : `../experiments/11_mission/tests of system/eval_nominal`,
  `eval_panne`, `eval_9019` et `eval_obstacle`. Dans chacun :
  - `video/mission.mp4` : la vidéo composée, à vitesse réelle (caméras des couloirs en
    mosaïque, caméra du drone qui lit, temps et nombre de codes lus en surimpression) ;
  - `video/sud_central.mp4`, `video/sud_grande.mp4` : les deux caméras fixes des couloirs ;
  - `video/lecteur.mp4` : la caméra du drone qui est en train de lire ;
  - `carte_finale.png`, `codes_dans_le_temps.png` : la carte en fin de mission et la courbe
    des codes lus ;
  - `resultats.json` : le bilan chiffré ; `mission.json` : le journal complet du vol.
- Les 4 vols de la référence : `../experiments/13_pore/pore_*/video/`.
- Le rejeu 3D des 8 vols : `../experiments/14_rejeu/rejeu_hors_ligne.html`.

## Limites connues

- Tout se passe en simulation : aucun vol réel.
- Le gain du guide vision-langage mesuré hors ligne a été obtenu sur le même entrepôt que son
  entraînement (voir l'étape 8).
- Lors d'un premier vol sur l'entrepôt 9019, les trois drones sont tombés presque ensemble
  vers 400 s, sans collision. Le vol refait avec le même code n'a pas reproduit le problème,
  qui reste inexpliqué (voir l'étape 5, `../experiments/11_mission/`).
