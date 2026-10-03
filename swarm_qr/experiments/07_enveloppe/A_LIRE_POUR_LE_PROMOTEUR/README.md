# 07 — Enveloppe de lecture : jusqu'où, sous quel angle et à quelle vitesse le drone lit un QR (étape 2)

## En une phrase
Jusqu'à quelle distance, sous quel angle et à quelle vitesse la caméra du drone lit-elle un QR
code, et quel lecteur de QR faut-il utiliser ?

## La tâche
Ces trois limites servent de base à tout le reste : le contrôleur de vol (étape 3) doit les
respecter, et la décision (étape 5) s'en sert pour choisir où envoyer chaque drone.

On photographie le QR d'un carton depuis des milliers de poses connues (une pose = la position
et l'orientation de la caméra), dans le simulateur Isaac Sim. Chaque image est enregistrée avec
la pose vraie de la caméra, relue dans le simulateur (jamais la pose commandée). Ensuite, sans
simulateur, six lecteurs de QR (bibliothèques qui décodent un QR dans une image : zxing, zbar,
pyboof, OpenCV…) relisent exactement les mêmes images, et on calcule le taux de lecture selon
la distance, l'angle et la vitesse.

Avant de mesurer, le banc vérifie sa propre chaîne (calibration de la caméra, retard du rendu,
lecture à une distance connue) et s'arrête si un contrôle échoue.

Cinq campagnes :
- optique : 2 000 poses tirées au hasard, caméra seule (sans drone), entrepôt de graine 9033
  (la graine est le numéro qui fixe la disposition de l'entrepôt) ;
- sans QR : les 300 premières de ces poses, avec les 228 panneaux QR cachés (fausses alertes) ;
- second entrepôt : 400 poses dans l'entrepôt de graine 9019 ;
- vol : 12 positions tenues par le vrai drone (piloté par ArduPilot SITL, le pilote
  automatique simulé sur le PC), 15 images chacune ;
- traversée : le drone longe le rack à 1,2 m du panneau, à 0,1, 0,3, 0,6 et 1 m/s.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../banc.py` | le banc dans Isaac Sim, 4 modes : `optique`, `sans-qr`, `vol`, `traversee` |
| `../campagne.sh` | relance les 5 campagnes puis l'analyse |
| `../analyse.py` | l'analyse sans simulateur : relit les images avec les 6 lecteurs |
| `../temps_lecteurs.py` | chronomètre les 6 lecteurs sur 400 images (sans simulateur) |
| `../resultats.json` | tous les chiffres de l'analyse (courbes par lecteur, intervalles de confiance) |
| `../temps_lecteurs.json` | temps de décodage par image de chaque lecteur |
| `../enveloppe.png` | la figure principale : taux de lecture selon la distance, l'angle et la distance apparente |
| `../poses_<campagne>.jsonl` | une ligne par image : nom du fichier, position et orientation vraies de la caméra, distance et angle au QR, vitesse |
| `../meta_<campagne>.json` | réglages de la campagne : mode, graine de l'entrepôt, graine du tirage, calibration K de la caméra, nombre d'images, retard du rendu mesuré |
| `RESULTATS.md` | les résultats expliqués |

Les images brutes (images JPEG de 1024 × 768 pixels, environ 510 Mo en tout, numérotées
`00000.jpg`, `00001.jpg`… ; la ligne correspondante de `poses_<campagne>.jsonl` donne la pose
de chaque image) :

| Dossier | Images | Contenu |
|---|---|---|
| `../images_optique/` | 2 000 | caméra seule, poses au hasard (0,45 à 8 m, jusqu'à 75°), entrepôt 9033 |
| `../images_sans_qr/` | 300 | les mêmes 300 premières poses, tous les QR cachés |
| `../images_9019/` | 400 | poses au hasard dans le second entrepôt (graine 9019) |
| `../images_vol/` | 180 | le drone en vol stationnaire : 12 poses × 15 images |
| `../images_traversee/` | 701 | le drone qui longe le rack : 427 à 0,1 m/s, 149 à 0,3, 77 à 0,6, 48 à 1 m/s |

Chaque campagne vise le panneau `BOX_000` (40 cm de côté) de son entrepôt.

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus + ArduPilot SITL, carte graphique NVIDIA RTX et écran
`DISPLAY=:1`, pour `campagne.sh` et `banc.py` (les modes `optique` et `sans-qr` n'utilisent pas
de drone, mais passent par le même script). Pour `analyse.py` et `temps_lecteurs.py`, pas de
simulateur : Python avec numpy, OpenCV, scipy, matplotlib et les lecteurs `zxing-cpp`, `pyzbar`,
`pyboof` (qui demande Java).
Durée : `campagne.sh` accorde au plus 1 h 30, 30 min, 40 min, 1 h 20 et 1 h 20 aux 5 passes
(5 h 20 au total).

Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    bash swarm_qr/experiments/07_enveloppe/campagne.sh 2>&1 | tee /tmp/campagne.log

Ce script utilise le Python de la machine de simulation (ligne `PY=`). Il se place dans son
dossier, refuse une passe si un SITL tourne encore, et tue Isaac Sim (`timeout -s KILL`) au-delà de la durée
permise, car Isaac ignore le signal d'arrêt normal. Les mêmes passes, une par une :

    E=swarm_qr/experiments/07_enveloppe
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 5400 $PY $E/banc.py --mode optique --poses 2000
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 1800 $PY $E/banc.py --mode sans-qr --poses 300
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 2400 $PY $E/banc.py --mode optique --poses 400 --seed 9019 --nom 9019
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 4800 $PY $E/banc.py --mode vol
    DISPLAY=:1 PYTHONUNBUFFERED=1 timeout -s KILL 4800 $PY $E/banc.py --mode traversee

Chaque passe écrit, dans le dossier de l'expérience, `images_<nom>/`, `poses_<nom>.jsonl` et
`meta_<nom>.json` (`<nom>` = le mode, ou la valeur de `--nom`). Inutile d'ajouter
`--kit_args` : `banc.py` désactive lui-même la vérification du pilote graphique. Autres options :
`--seed` (entrepôt, 9033 par défaut), `--graine-tirage` (tirage des poses, 20260901 par défaut).

L'analyse et le chronométrage, sans simulateur (ils réécrivent leurs fichiers de sortie) :

    python swarm_qr/experiments/07_enveloppe/analyse.py
    python swarm_qr/experiments/07_enveloppe/temps_lecteurs.py

- `analyse.py` écrit `resultats.json` et `enveloppe.png` ; avec `--rapide`, il n'utilise que
  zxing, pour vérifier la chaîne.
- `temps_lecteurs.py` écrit `temps_lecteurs.json` (400 images de `images_optique/` ; `--n 2000`
  pour toutes, `--campagne 9019` pour un autre dossier d'images).

## Résultats en bref
- Distance et angle ne font qu'une limite : la distance apparente (distance ÷ cosinus de
  l'angle). Zone fiable : 1,5 à 4 m de distance apparente, 91 à 99 % des images lues.
- La vitesse ne coûte rien jusqu'à 1 m/s : 100 % des images lues à 0,1, 0,3, 0,6 et 1 m/s
  (petits échantillons aux vitesses élevées, et pas de flou de bougé dans le simulateur).
- Aucun code inventé : 0 lecture sur 300 images sans QR, pour les 6 lecteurs ; mais le simple
  repérage d'un motif se trompe 27 % du temps.
- Lecteur retenu : zxing (90,2 % en zone utile, 17 ms par image), zbar en alternative
  (91,2 %) ; position d'un code lu à 0,8 cm près (médiane).
- Le drone en vol lit comme la caméra seule (100 % entre 1 et 4 m), et un second entrepôt
  donne le même résultat (90,4 % contre 90,2 %).

→ détails dans `RESULTATS.md`.
