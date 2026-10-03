# docs — Les figures du mémoire (toutes les étapes)

## En une phrase

Ce dossier contient les figures du mémoire (schémas et courbes, légendées en anglais) et les
scripts Python qui les fabriquent à partir des résultats des expériences.

## La tâche

- Les schémas (architecture, perception) sont dessinés en SVG par du code, sans logiciel de
  dessin : les positions sont calculées, rien n'est placé à la main.
- Les courbes relisent les fichiers de résultats des expériences (`../../experiments/...`).
- Aucun simulateur n'est nécessaire. Chaque script écrit ses figures à côté de lui.

## Contenu du dossier

| Script | Figures produites |
|---|---|
| `../figures/architecture.py` | `architecture.svg` |
| `../figures/perception_schemas.py` | `fig_perception_chaine.svg`, `fig_distance_apparente.svg`, `fig_regimes.svg` |
| `../figures/perception_courbes.py` | `fig_enveloppe.png`, `fig_portees.png` |
| `../figures/fig_annotation.py` | `fig_annotation.png` |
| `../figures/guide_courbes.py` | `fig_guide.png` |
| `../figures/evaluation/analyse_evaluation.py` | `evaluation/mesures.json` et les 9 `evaluation/ev_*.png` |
| `../figures/prompt_architecture.md` | description écrite de l'architecture (voir plus bas) |

Les PNG des schémas (`architecture.png`, `fig_perception_chaine.png`,
`fig_distance_apparente.png`, `fig_regimes.png`) ont été convertis depuis les SVG à l'échelle 2
(par exemple 1560 × 1132 → 3120 × 2264 pixels).

## Ce que montre chaque figure

Schémas (chapitre conception) :
- `../figures/architecture.svg` / `.png` : l'architecture du système (chapitre 3) : l'entrepôt,
  le drone (capteurs, perception, contrôleur de vol), la carte partagée (occupation, couverture
  orientée, table des QR, coéquipiers et réservations), la décision (candidats, note unique,
  faisabilité), le guide optionnel, la supervision, les échanges nommés et la boucle en 5 temps.
- `../figures/fig_perception_chaine.svg` / `.png` : l'intérieur de la perception d'un drone
  sur un cycle : capteurs → décodage et détection → placement 3D → carte partagée.
- `../figures/fig_distance_apparente.svg` / `.png` : pourquoi distance et angle sont une seule
  limite : un panneau incliné de α remplit le même cône qu'un panneau de face 1 / cos α fois
  plus loin, d'où la distance apparente d / cos α.
- `../figures/fig_regimes.svg` / `.png` : placer une détection : jusqu'à 8 m un rayon du lidar
  (capteur laser qui mesure les distances) tombe sur l'étiquette ; au-delà, deux rayons sont
  plus écartés qu'une case de la carte et c'est la carte qui donne la distance.

Courbes mesurées :
- `../figures/fig_enveloppe.png` (étape 2, `07_enveloppe/resultats.json`) : probabilité de
  lire un QR par image selon la distance apparente, avec l'enveloppe de lecture 1,5–4 m et le
  seuil de 90 %.
- `../figures/fig_portees.png` (étape 7, `10_detecteur/resultats.json`) : part des panneaux
  repérés selon la distance : détecteur appris 98 % au point le plus lointain, détecteur
  géométrique 47 % ; sur des images sans aucun QR, fausses alertes 27 % contre 0,7 %.
- `../figures/fig_annotation.png` (étape 7, `10_detecteur/controle/planche_00.jpg`) : deux
  vues de l'annotation automatique, boîtes vertes = QR, bleues = cartons.
- `../figures/fig_guide.png` (étape 8, `12_guide/resultats_entrainement.json`) : bonne zone
  choisie sur 72 cas jamais vus : zone la plus proche 25 %, planificateur 83 %, modèle avant
  entraînement 83 %, guide entraîné 96 % (11 cas gagnés, 2 perdus).

Évaluation finale (4 vols du système, `11_mission/tests of system/eval_*`, et 4 vols de Pore et
al., `13_pore/pore_*` ; scénarios : nominal, panne d'un drone, entrepôt 9019 jamais vu,
obstacle) — chiffres relus dans `../figures/evaluation/mesures.json` :
- `ev_methodes.png` : part de l'inventaire lue. Système 96 / 96 / 92 / 96 %, Pore 93 / 66 /
  82 / 90 %, inférence active 41 / 38 / 36 / 39 %, apprentissage par renforcement 19 / 15 /
  14 / 17 %.
- `ev_lecture_temps.png` : codes lus au fil du temps, un panneau par scénario. À la fin :
  système 110, 110, 60, 110 ; Pore 106, 75, 53, 103 (sur 114, 114, 65, 114 présents).
- `ev_jalons.png` : temps pour lire 50, 80 et 90 % du total. 90 % atteint par le système en
  196, 187, 165 et 206 s ; par Pore en 515 s (nominal) et 534 s (obstacle), jamais en panne et
  sur 9019.
- `ev_panne.png` : scénario panne (drone 1 arrêté à 200 s) : 110 codes lus au total.
- `ev_obstacle.png` : scénario obstacle (à 200 s) : 101 codes lus avant, 110 au total.
- `ev_effort.png` : distance volée par drone, et rendement : 37,7 / 41,1 / 16,6 / 28,7 codes
  lus pour 100 m volés.
- `ev_couts.png` : coût d'une observation, 160 à 173 ms, surtout la détection apprise puis le
  lidar.
- `ev_securite.png` : distance minimale entre drones (système 2,08 / 3,82 / 2,53 / 1,60 m, seuil
  d'arrêt 1,5 m), aucun point de trajectoire dans un rack, cessions de priorité 2 / 0 / 0 / 10.
- `ev_decisions.png` : cibles choisies par genre : lire 72 / 62 / 47 / 54, couvrir 10 / 3 /
  12 / 24, explorer 0 partout.

## `prompt_architecture.md`

Texte en anglais qui décrit le système à dessiner : composants, ce que chacun contient, ce qui
circule entre eux (tableau des échanges), boucle en 5 temps, principe à faire passer et pile
technique. Il fixe le contenu seulement ; la mise en page et le style sont laissés à qui dessine.
`architecture.py` en reprend le contenu. **Ne pas le modifier.**

## Comment le lancer

Prérequis : Python 3 avec `matplotlib` et `numpy` (courbes et évaluation), `Pillow` en plus
pour `fig_annotation.py` ; rien pour les deux scripts de schémas. Pas de simulateur, pas de
carte graphique. Toutes les données lues sont présentes dans cette copie. Durée : quelques
secondes. Depuis la racine du projet :

    python swarm_qr/docs/figures/architecture.py
    python swarm_qr/docs/figures/architecture.py --sans-techs
    python swarm_qr/docs/figures/perception_schemas.py
    python swarm_qr/docs/figures/perception_courbes.py
    python swarm_qr/docs/figures/fig_annotation.py
    python swarm_qr/docs/figures/guide_courbes.py
    python swarm_qr/docs/figures/evaluation/analyse_evaluation.py

- `--sans-techs` produit la version sans noms de technologies, dans le même `architecture.svg`.
- `analyse_evaluation.py` écrit `mesures.json` et les 9 figures `ev_*.png`.
- Les PNG des schémas se refont à la main depuis les SVG (outil de conversion au choix).

## Résultats en bref

- 4 schémas d'architecture et de perception, 4 courbes mesurées (étapes 2, 7, 8) et 9 figures
  d'évaluation finale.
- Évaluation : le système lit 92 à 96 % de l'inventaire dans les 4 scénarios, contre 66 à 93 %
  pour Pore et al.
- Détails des résultats : dans les dossiers `A_LIRE_POUR_LE_PROMOTEUR` des expériences.
