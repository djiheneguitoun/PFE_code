# swarm_qr/experiments — les expériences, étape par étape

## En une phrase

Chaque dossier répond à une question précise. Ensemble, ils construisent le système pièce par
pièce, et chaque pièce est mesurée avant d'être utilisée par la suivante.

## Attention à la numérotation

Les numéros des dossiers (01 à 14) suivent l'ordre de création des dossiers, pas les étapes du
projet (1 à 8). Le tableau ci-dessous les remet dans l'ordre des étapes. Aucun dossier ne porte
l'étape 6.

## L'index

| Étape | Dossier | La question | La réponse courte |
|---|---|---|---|
| 1 | `01_reproductibilite` | Le même numéro redonne-t-il le même entrepôt ? | Oui : description identique octet par octet, aucun objet déplacé (65 cartons, 130 QR). |
| 1 | `02_variation` | Les entrepôts sont-ils vraiment différents ? | Oui : au pire, les racks bougent de 1,85 m ; de 62 à 118 cartons selon l'entrepôt. |
| 1 | `03_images_qr` | Le drone lit-il les QR en volant ? | Oui : lus de 0,5 à 3 m ; 6 codes en un seul passage à 0,5 m/s. |
| 1 | `04_debit` | Combien coûte la simulation ? | 5 images/s ne coûtent que 7 % ; c'est la physique du pilote automatique qui coûte. |
| 1 | `05_sitl` | La chaîne Isaac Sim + Pegasus + ArduPilot vole-t-elle ? | Oui, de bout en bout et sans intervention ; vol stationnaire stable à environ 1 cm (Y, Z). |
| 1 | `06_position_vraie` | Où sont exactement les caméras sur le drone ? | 10 cm sur le côté et 11 cm sous le centre du drone (mesuré en vol). |
| 2 | `07_enveloppe` | Jusqu'où, sous quel angle, à quelle vitesse lit-on un QR ? | Une seule limite, la distance apparente : fiable de 1,5 à 4 m ; aucun effet de la vitesse jusqu'à 1 m/s ; aucun code inventé. |
| 3 | `08_controle` | Le drone sait-il rejoindre une pose et s'y tenir ? | 100 arrivées sur 100, tenue à 1 cm, 93 lectures sur 100 ; à 3 drones : 6 sur 6. |
| 4 | `09_carte` | La carte partagée est-elle fidèle ? | Code lu placé à 0,9 cm près (médiane) ; 110 codes sur 114, aucun inventé ; aucun passage dans un rack. |
| 5 | `11_mission` | Le système complet à 3 drones fonctionne-t-il ? | Oui : 110/114 (nominal), 114/114 (avec une panne), 90 % des codes en environ 3 min. Contient l'évaluation finale (`tests of system/`). |
| 7 | `10_detecteur` | Un réseau YOLO repère-t-il les QR de loin ? | 98 % des panneaux repérés à 6–8 m (47 % sans lui) ; QR inventé sur 0,7 % des images (27 % sans lui). |
| 8 | `12_guide` | Un modèle vision-langage donne-t-il de bons conseils ? | Après un petit entraînement : 96 % de bonnes zones contre 83 % pour la géométrie — mais testé sur l'entrepôt de l'entraînement. |
| Évaluation | `13_pore` | Fait-on mieux qu'une méthode publiée (Pore et al., 2026) ? | Oui dans les 4 cas : 110 contre 106 codes, 110 contre 75 (panne), 60 contre 53 (entrepôt jamais vu), 110 contre 103 (obstacle). |
| Démonstration | `14_rejeu` | — | Rejeu 3D des 8 vols de l'évaluation, à ouvrir dans un navigateur. |

Les détails, les commandes et les limites de chaque mesure sont dans le dossier
`A_LIRE_POUR_LE_PROMOTEUR/` de chaque expérience.

## Ce qu'on trouve dans chaque dossier

- `A_LIRE_POUR_LE_PROMOTEUR/README.md` : la tâche, le contenu du dossier, les commandes ;
- `A_LIRE_POUR_LE_PROMOTEUR/RESULTATS.md` : les résultats expliqués, et ce que la mesure ne
  prouve pas ;
- les scripts (`run.py`, ou `banc.py` + `analyse.py` + `campagne.sh`) et les sorties brutes
  (json, images, vidéos).

## Les fichiers communs, à la racine de ce dossier

| Fichier | Rôle |
|---|---|
| `../_img.py` | écrire des images, des planches et des vidéos (sans simulateur) |
| `../_viz.py` | caméra de survol et toit masqué, pour les vues d'ensemble (avec Isaac Sim) |
| `../_vol.py` | chemins de vol simples par les allées, utilisés avant que la carte (étape 4) existe |
| `../run_all.sh` | relance les tests 01 à 04 de l'étape 1 |

## Une méthode commune

- À partir de l'étape 2, chaque campagne commence par des contrôles automatiques (calibration
  de la caméra, retard du rendu, lecture à distance connue…) et s'arrête seule si un contrôle
  échoue.
- Le jugement est fait après coup par un arbitre qui connaît la vérité du simulateur (position
  et contenu de chaque QR).
- Les entrepôts d'essai 9033 et 9019 font partie des numéros « scellés » (9000 à 9039), jamais
  utilisés pour entraîner le détecteur. Les vols de mise au point des missions ont tous été
  faits sur l'entrepôt 9033 : le 9019 sert de test sur un entrepôt « jamais vu ».
