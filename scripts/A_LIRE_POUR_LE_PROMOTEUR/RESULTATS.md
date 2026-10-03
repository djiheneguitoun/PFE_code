# 12 — Exploration par inférence active (AIF) dans Isaac Sim : résultats

## La question

Trois drones qui explorent l'entrepôt par inférence active (AIF) :
- vont-ils plus vite qu'une méthode heuristique simple ?
- vaut-il mieux décider de façon centralisée (un « cloud » décide pour tous) ou distribuée (chaque drone décide seul) ?
- l'exploration résiste-t-elle à un incident : cloud lent, drone perdu, cloud coupé, liens radio coupés, obstacle soudain ?

## Méthode (en bref)

- Simulateur : Isaac Sim + Pegasus, 3 drones Iris pilotés chacun par ArduPilot SITL (le pilote automatique simulé
  sur le PC), dans l'entrepôt `warehouse_multiple_shelves.usd`. Zone étudiée : 26 × 41 m, découpée en cases de 0,5 m.
- Chaque drone vole à 2 m avec un LiDAR (capteur laser qui mesure les distances) : 360 rayons, portée 8 m.
  Il tient une carte d'occupation (probabilité qu'une case soit un obstacle) ; les cartes sont fusionnées.
- À chaque step AIF, chaque drone choisit un déplacement de 1 m parmi 9 (rester ou 8 directions).
  Un step = 60 pas de physique de 1/60 s (`physics_dt_s`), soit 1 s simulée.
- Planificateur AIF : minimise une « énergie libre attendue » G = − 2,5 × gain d'information attendu
  − 0,8 × attrait des zones inconnues proches + 0,1 × coût de mouvement + 5 × proximité des autres drones ;
  le choix est tiré au hasard, en favorisant les G bas (température 0,3).
- Planificateur heuristique : garde sa direction tant que la case suivante est libre ; sinon tire une nouvelle
  direction au hasard, de préférence vers des cases inconnues.
- Centralisé : le cloud calcule les actions sur la carte fusionnée ; une action arrive 1 step plus tard
  (aller-retour de 500 ms). Distribué : chaque drone fusionne sa carte avec celles des voisins à moins de 5 m.
- Réseau entre drones : ns-3 (simulateur de réseau) en WiFi ; latences mesurées en fin de run : 0,6 à 1,5 ms.
- Arrêt du run : couverture ≥ 93 % (ou 15 steps sans progrès, ou 80 steps au plus).
  Les 8 runs se sont tous arrêtés en atteignant 93 %.
- Incidents (« stresseurs ») : un seul par run, déclenché au step 20.
- Dates des runs : 31 mai 2026 (8 juin 2026 pour l'obstacle).

## Comment lire les chiffres

- **Steps jusqu'à 93 %** : nombre de décisions avant d'atteindre l'objectif. Plus c'est bas, plus c'est rapide.
- **Couverture** (« bbox USD » dans les journaux) : part des cases connues avec confiance (probabilité < 0,3 ou > 0,7),
  rapportée à l'aire intérieure de l'entrepôt (boîte réduite de 2,5 m de chaque côté).
- **Couverture intérieure** (« bbox murs ») : part des cases vues au moins une fois dans le rectangle des murs détectés.
- **Entropie** : incertitude moyenne de la carte (0,69 = rien n'est connu). Elle part de 0,65 et finit vers 0,32
  dans tous les runs (elle ne descend pas plus bas car la grille contient aussi l'extérieur, jamais vu).
- **Couverture connue des planificateurs** : ce que voient réellement ceux qui décident (cloud ou drones).
  L'écart avec la couverture globale vient du réseau.
- **Innovation** : écart moyen entre ce que voit le LiDAR et ce que prévoyait la carte.
- **Phases de résilience** : après un incident, phase « récupération » pendant 30 steps, puis « durable ».

## Résultats

### Vue d'ensemble des 8 runs

| Run (tag) | Steps → 93 % | Distance totale | Couv. intérieure finale |
|---|---|---|---|
| aif_cent_baseline | 45 | 81,8 m | 94,5 % |
| aif_dist_baseline | 53 | 102,1 m | 97,1 % |
| heur_cent_baseline | 69 | 116,0 m | 95,0 % |
| aif_cent_cloud_loaded | 44 | 77,2 m | 99,0 % |
| aif_cent_kill_d0_s20 | 79 | 103,1 m | 96,0 % |
| aif_cent_cut_cloud_s20 | 57 | 97,4 m | 99,0 % |
| aif_dist_cut_links_s20 | 52 | 76,0 m | 99,3 % |
| aif_cent_obstacle_s20 | 50 | 98,3 m | 99,1 % |

Couverture finale : 93,0 à 93,4 % partout ; entropie finale : 0,315 à 0,320 partout.
Distance totale = somme des trajets des 3 drones.

### 1. Références : AIF centralisé contre AIF distribué

| Indicateur | Centralisé | Distribué |
|---|---|---|
| Steps jusqu'à 93 % | 45 | 53 |
| Couverture au step 20 / 40 | 57,5 / 91,5 % | 52,5 / 83,2 % |
| Distance totale | 81,8 m | 102,1 m |
| Couv. connue des planificateurs (fin) | 93,0 % | 86,2 % |
| Écart max global − connu | 6,8 points (step 1) | 11,4 points (step 24) |
| Messages envoyés / perdus | 135 / 0 | 234 / 0 |

En centralisé, le cloud voit la carte fusionnée complète et les cibles déjà choisies pour les autres drones ;
en distribué, chaque drone ne voit que les cartes des voisins à moins de 5 m.

### 2. Heuristique contre AIF (centralisé, WiFi)

| Indicateur | AIF | Heuristique |
|---|---|---|
| Steps jusqu'à 93 % | 45 | 69 |
| Steps pour 50 / 75 / 90 % | 17 / 31 / 39 | 21 / 42 / 68 |
| Distance totale | 81,8 m | 116,0 m |
| Couverture intérieure finale | 94,5 % | 95,0 % |

L'AIF atteint l'objectif en 24 steps de moins (− 35 %) avec 30 % de distance en moins.

### 3. Cloud chargé : aller-retour de 1 500 ms au lieu de 500 ms

| Indicateur | 500 ms | 1 500 ms |
|---|---|---|
| Retard des actions | 1 step | 2 steps |
| Steps jusqu'à 93 % | 45 | 44 |
| Steps pour 50 / 75 / 90 % | 17 / 31 / 39 | 17 / 32 / 40 |
| Distance totale | 81,8 m | 77,2 m |
| Décisions fraîches par minute (moyenne) | 168,3 | 159,7 |
| Messages en attente en fin de run | 3 | 6 |

Aucun ralentissement visible sur ce run.

### 4. Drone 0 mis hors service au step 20 (centralisé)

| Indicateur | Référence | Drone 0 perdu |
|---|---|---|
| Steps jusqu'à 93 % | 45 | 79 |
| Couverture step 20 → 21 | 57,5 → 59,0 % | 56,5 → 45,1 % |
| Steps pour 75 / 90 % | 31 / 39 | 44 / 69 |
| Distance totale | 81,8 m | 103,1 m (dont drone 0 : 11,0 m) |
| Décisions par minute (fin) | 180 | 120 |
| Phases | normale | récupération 21–51, durable 52–79 |

La couverture chute de 11,4 points au step 21 : la carte fusionnée n'utilise que les drones actifs, donc
ce que le drone 0 avait vu est perdu. Les 2 drones restants terminent la mission, avec 34 steps de plus que la référence.

### 5. Cloud coupé au step 20 (centralisé)

| Indicateur | Référence | Cloud coupé |
|---|---|---|
| Steps jusqu'à 93 % | 45 | 57 |
| Architecture après le step 20 | centralisée | distribuée (bascule automatique) |
| Steps sans nouvelle décision | 0 | 2 (steps 21–22) |
| Actions en route perdues | 0 | 3 |
| Couv. connue des planificateurs, step 20 → 21 | 57,5 → 59,0 % | 55,7 → 40,9 % |
| Distance totale | 81,8 m | 97,4 m |

La bascule fonctionne : 2 steps de transition (2 000 ms prévus), puis chaque drone décide seul.
Coût : + 12 steps par rapport au centralisé, + 4 par rapport à la référence distribuée (53).

### 6. Tous les liens entre drones coupés au step 20 (distribué)

| Indicateur | Réf. distribuée | Liens coupés |
|---|---|---|
| Steps jusqu'à 93 % | 53 | 52 |
| Steps sans nouvelle décision | 0 | 2 (steps 21–22) |
| Cartes (messages) perdues | 0 | 180 |
| Couv. connue des planificateurs (fin) | 86,2 % | 70,5 % |
| Écart max global − connu | 11,4 points | 23,0 points (step 49) |
| Distance totale | 102,1 m | 76,0 m |

Après la coupure, chaque drone continue seul avec sa propre carte et la mission est terminée en 52 steps,
alors que chacun ne connaît que 70,5 % en moyenne à la fin.

### 7. Obstacle lâché au step 20 (centralisé)

Cube rouge de 3 × 3 × 4 m posé en x = 4,0 m, y = 3,5 m (coordonnées du monde).

| Indicateur | Référence | Obstacle |
|---|---|---|
| Steps jusqu'à 93 % | 45 | 50 |
| Couverture au step 30 / 40 | 74,3 / 91,5 % | 66,3 / 84,1 % |
| Distance totale | 81,8 m | 98,3 m |
| Phase après le step 20 | normale | récupération jusqu'à la fin (step 50) |

### Lecture des QR codes pendant les runs

Un seul panneau QR de test (texte « DRONE_WAREHOUSE_INSPECTION_001 », 2,5 m de côté, posé en x = 0, y = −5 m,
à 2 m de haut). Images décodées / analysées : 7 / 1 620 (AIF centralisé), 4 / 1 908 (distribué), 35 / 2 484
(heuristique), 11 / 1 584 (cloud chargé), 1 / 2 844 (drone perdu), 11 / 2 052 (cloud coupé), 5 / 1 872 (liens coupés),
7 / 1 800 (obstacle). Presque toutes les lectures viennent du drone 1. Ces runs n'évaluent pas l'inventaire.

## Conclusion

- Les 8 runs atteignent l'objectif de 93 %, y compris avec un incident.
- L'AIF centralisé est le plus rapide des références : 45 steps, contre 53 en distribué et 69 pour l'heuristique.
- Un cloud 3 fois plus lent (2 steps de retard) n'a pas ralenti l'exploration sur ce run (44 steps).
- L'incident le plus coûteux est la perte d'un drone (+ 34 steps) : il emporte aussi sa part de carte.
- La coupure du cloud est absorbée par la bascule automatique en distribué (+ 12 steps, 3 actions perdues).
- Sans aucun lien entre drones, l'essaim termine quand même (52 steps), chacun avec une carte incomplète.
- Un obstacle soudain coûte 5 steps.

## Limites : ce que ce test ne prouve pas

- Ces runs mesurent l'exploration de l'entrepôt ; ils n'évaluent pas l'inventaire des QR codes
  (au plus 1,4 % des images du panneau de test décodées).

## Fichiers de résultats et figures

Chaque run a son dossier `../../logs/runs/run_<date>_<heure>_<tag>/` (contenu détaillé dans
`../../logs/A_LIRE_POUR_LE_PROMOTEUR/README.md`) :
- `README.md` : résumé généré (steps, entropie et couvertures finales) ;
- `config.json`, `history.json` (une ligne par step), `state_final.json`, `qr_stats.json` ;
- figures utiles : `coverage.png`, `trajectories.png`, `resilience_phases.png`, `coverage_known_vs_global.png`.

Exemples :
- `../../logs/runs/run_20260531_041619_aif_cent_baseline/coverage.png` (référence centralisée) ;
- `../../logs/runs/run_20260531_044951_aif_cent_kill_d0_s20/coverage_known_vs_global.png` (chute au step 21) ;
- `../../logs/runs/run_20260531_100239_aif_dist_cut_links_s20/coverage_known_vs_global.png` (écart dû aux liens coupés).

Pour relancer une configuration (depuis la racine du projet) :

    bash scripts/12_launch_aif_isaac_sim.sh 3 --max-steps 80 --run-tag <tag> <options>

| Tag | Options |
|---|---|
| aif_cent_baseline | --planner aif --arch centralized --ns3 wifi |
| aif_dist_baseline | --planner aif --arch distributed --ns3 wifi |
| heur_cent_baseline | --planner heuristic --arch centralized --ns3 wifi |
| aif_cent_cloud_loaded | idem aif_cent_baseline + --cloud-round-trip-ms 1500 |
| aif_cent_kill_d0_s20 | idem + --kill-drone-at-step 20 --kill-drone-id 0 |
| aif_cent_cut_cloud_s20 | idem + --cut-cloud-at-step 20 |
| aif_dist_cut_links_s20 | --planner aif --arch distributed --ns3 wifi --neighbor-radius-m 12 --cut-drone-link all --cut-drone-link-at-step 20 |
| aif_cent_obstacle_s20 | idem aif_cent_baseline + --drop-obstacle-at-step 20 --drop-obstacle-xy "4.0,3.5" |

Ces options viennent de `../run_all_experiments.sh`.
