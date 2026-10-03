# 10 — Le détecteur appris : repérer QR et cartons de loin (étape 7)

## En une phrase

Un petit réseau YOLO (réseau de neurones qui encadre des objets dans une image), entraîné sur
des images annotées automatiquement, repère-t-il les QR nettement plus loin que le repérage
classique de l'étape 2, sans inventer d'objets ?

## La tâche

- Le décodeur classique ne lit un QR que de près et de face : l'enveloppe de lecture de
  l'étape 2 (`07_enveloppe`) s'arrête à 4 m.
- On ajoute un réseau qui ne lit pas mais **repère**. Un QR vu de loin devient une « piste »
  sur la carte (un endroit où aller lire). Un carton repéré remplit le canal sémantique de la
  carte, resté vide à l'étape 4 (`09_carte`).
- Les images d'entraînement sont rendues dans Isaac Sim et annotées sans aucun clic : la
  position de chaque objet est connue, et des rayons du moteur physique vérifient ce qui est
  réellement visible.
- 6 entrepôts pour apprendre, 2 entrepôts « scellés » (9033 et 9019) jamais vus pendant
  l'apprentissage, et les 3 577 images de l'étape 2 comme juge commun avec le repérage classique.
- Réseau : YOLO11 nano (2,6 millions de paramètres), à deux tailles d'entrée (1024 et 640 pixels).
- Enfin, un vol en simulation : la patrouille de l'étape 4 avec le détecteur branché.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../rendu.py` | rend les images dans Isaac Sim et calcule les cadres |
| `../controle.py` | contrôle les cadres avant l'entraînement |
| `../entraine.py` | entraîne les variantes YOLO |
| `../banc.py` | juge les variantes, choisit, installe le modèle retenu |
| `../campagne.sh` | enchaîne les phases (rendu, controle, entraine, banc, vol) |
| `../jeu.yaml` | description du jeu pour ultralytics (réécrite par entraine.py) |
| `../jeu/` | le jeu d'images annotées (environ 14 800 fichiers) |
| `../runs/` | les entraînements n1024 et n640 |
| `../entrainement.json` | bilan des entraînements |
| `../resultats.json` | bilan du banc, variante retenue, seuil |
| `../portee.png` | courbes de repérage selon la distance |
| `../controle/` | planches de contrôle et vérification des cadres |
| `../vol/` | le second vol (le vol de référence) |
| `../vol_avant_correction/` | le premier vol, gardé comme preuve |
| `../yolo11n.pt`, `../yolo11s.pt` | poids pré-entraînés téléchargés |
| `RESULTATS.md` | les résultats expliqués |

Détail des sous-dossiers :

- `../jeu/` : un dossier par jeu. `rendu_0` à `rendu_5` (apprentissage, 500 images chacun),
  `rendu_9033` et `rendu_9019` (entrepôts scellés, 400 images chacun), et `etape2_optique`,
  `etape2_9019`, `etape2_sans_qr`, `etape2_vol`, `etape2_traversee` (les 3 577 images de
  l'étape 2, ré-annotées ; ce sont des liens vers les images de `07_enveloppe`). Chaque dossier
  contient `images/`, `labels/` (un fichier de cadres par image, au format YOLO),
  `manifeste.jsonl` (pose de la caméra et objets de chaque image) et `meta.json` (calibration).
- `../runs/n1024/` et `../runs/n640/` : courbes d'apprentissage (`results.png`), matrices de
  confusion, exemples d'images, et les poids `weights/best.pt` et `weights/last.pt`.
  `n1024_val/` et `n640_val/` sont vides.
- `../controle/` : `planche_00.jpg` à `planche_03.jpg` (24 images avec leurs cadres dessinés)
  et `verification_cadres.json` (comparaison avec la projection de l'étape 2).
- `../vol/` et `../vol_avant_correction/` : `carte.json` et `carte.npz` (la carte finale),
  `carte_finale.png`, `carte_qui_se_remplit.mp4` (la carte qui se construit pendant le vol),
  `carte_3d.html` et `carte_3d.png` (vue 3D), `comparaison.png`, `vue_00.png` à `vue_03.png`,
  `vol.json` (le journal du vol) et `resultats.json` (le jugement de l'étape 4).
- `../yolo11n.pt` (nano) et `../yolo11s.pt` (small) : poids pré-entraînés publics
  d'ultralytics, point de départ de l'entraînement. Le modèle « small » n'a pas été entraîné.
- Le modèle retenu (n1024) est copié par banc.py dans
  `../../../assets/detecteur/detecteur.pt`, avec ses réglages dans
  `../../../assets/detecteur/detecteur.json` (taille 1024 px, seuil 0,5, classes qr et carton).
  C'est ce fichier que le système charge (`swarm_qr/detecteur.py`, option `--detecteur auto`).

## Comment le lancer

Prérequis :
- `rendu` : Isaac Sim 5.1 et une carte graphique ;
- `controle` : Python avec OpenCV et numpy, sans simulateur ;
- `entraine` et `banc` : une carte graphique NVIDIA et `ultralytics`, sans simulateur ;
- `vol` : Isaac Sim 5.1 + Pegasus + ArduPilot SITL (le pilote automatique ArduPilot simulé sur
  le PC).

Durée : entraînement 48 min (n1024) et 21 min (n640). Le rendu de chaque entrepôt est coupé
après 1 h au plus, le vol après 2 h au plus (`timeout -s KILL`, car Isaac Sim ignore le signal
d'arrêt normal). Ne jamais lancer deux phases GPU en même temps : le rendu et l'entraînement se
partagent les 8 Go de la carte.

Depuis la racine du projet (le script se place lui-même dans son dossier) :

    bash swarm_qr/experiments/10_detecteur/campagne.sh rendu
    bash swarm_qr/experiments/10_detecteur/campagne.sh controle
    bash swarm_qr/experiments/10_detecteur/campagne.sh entraine
    bash swarm_qr/experiments/10_detecteur/campagne.sh banc
    bash swarm_qr/experiments/10_detecteur/campagne.sh vol

- `rendu` : écrit `jeu/rendu_<graine>/` pour les entrepôts 0 à 5 (500 images) puis 9033 et
  9019 (400 images), et `jeu/etape2_*/` (cadres des images de l'étape 2).
- `controle` : écrit `controle/planche_*.jpg` et `controle/verification_cadres.json`.
- `entraine` : entraîne n1024 et n640 (`runs/`, `entrainement.json`). Pour ajouter le modèle
  « small » : `bash swarm_qr/experiments/10_detecteur/campagne.sh entraine n1024,n640,s1024`.
- `banc` : écrit `resultats.json`, `portee.png`, et copie le modèle retenu dans
  `swarm_qr/assets/detecteur/`.
- `vol` : lance `09_carte/banc.py --mode vol --etages 1 --allees 2 --detecteur auto` puis
  `09_carte/analyse.py` ; sorties dans `vol/`. Refuse de partir si un SITL tourne encore.

campagne.sh fixe lui-même `PY=/home/djihene_guitoun/isaac5_env/bin/python` (le Python de la
machine de simulation) et `DISPLAY=:1`.

Les scripts peuvent aussi être lancés un par un, `PY` étant le Python de l'environnement Isaac
Sim (sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    $PY swarm_qr/experiments/10_detecteur/rendu.py --seed 3 --images 500
    $PY swarm_qr/experiments/10_detecteur/rendu.py --seed 9033 --images 400 --relabel optique,sans_qr,vol,traversee
    $PY swarm_qr/experiments/10_detecteur/rendu.py --seed 9019 --images 400 --relabel 9019
    $PY swarm_qr/experiments/10_detecteur/controle.py --verifie
    $PY swarm_qr/experiments/10_detecteur/entraine.py --variantes n1024,n640
    $PY swarm_qr/experiments/10_detecteur/banc.py

À savoir :
- rendu.py passe lui-même l'option `--/rtx/verifyDriverVersion/enabled=false` à Isaac Sim.
- Les chemins donnés à `controle.py --planche` sont relatifs au dossier courant (campagne.sh
  les donne depuis `10_detecteur/`, par exemple `jeu/rendu_0`).
- banc.py lit les poids à juger dans `entrainement.json` (écrit par entraine.py).

## Résultats en bref

- Panneau visé repéré à 6–8 m, sur les mêmes images que l'étape 2 : **98 %** contre 47 % pour
  le repérage classique ; portée tenue à 90 % : **8,0 m** (la limite des images) contre 4,0 m.
- QR inventés sur les 300 images sans QR : **0,7 %** des images, contre 27,3 % pour le classique.
- Entrepôts jamais vus : **98,3 %** des QR visibles et **94,1 %** des cartons trouvés ;
  12 ms par image, 63 Mo de mémoire sur la carte graphique.
- Variante retenue : nano 1024 px, seuil de confiance 0,5.
- En vol : 1 piste fantôme sur 143, 77 % des pistes à moins de 1 m d'un vrai panneau,
  0 point de trajectoire sur 819 dans une structure de rack.

→ détails, réserves et chiffres complets dans `RESULTATS.md`.
