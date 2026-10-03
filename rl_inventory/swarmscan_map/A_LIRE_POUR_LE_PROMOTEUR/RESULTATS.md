# rl_inventory/swarmscan_map — Essai RL avec carte : résultats

## La question

Un réseau unique, partagé par 3 drones, peut-il apprendre par renforcement à lire tous les
QR d'un entrepôt sans connaître leur position, avec seulement une carte construite en vol ?

## Méthode (en bref)

- PPO à paramètres partagés (rsl_rl 3.0) dans Isaac Lab ; 3 drones par entrepôt, des dizaines
  d'entrepôts en parallèle ; observation = cartes centrées sur le drone + petit vecteur.
- Lecture simulée par une règle géométrique (distance, angle, vitesse, rotation, 2 images),
  calibrée sur le vrai décodeur OpenCV.
- Curriculum à 10 crans : règle tolérante au départ, resserrée quand les drones lisent bien.
- Les chiffres ci-dessous sont notés dans les commentaires du code (`../config_map.py`,
  `../env_map.py`, `../mapping.py`, `../models.py`, `../curriculum.py`, `../train.py`,
  `../eval_map.py`, `../verify_env.py`) et dans les docstrings de
  `../../tests/test_swarmscan_map_pure.py`.
- Détails et commandes : `README.md`.

## Résultats

### 1. Calibration de la règle de lecture (`../calibrate_gate.py`, 2026-07-09)

Conditions : décodeur OpenCV `QRCodeDetectorAruco`, image agrandie ×3, caméra 1280×960,
champ horizontal 60°.

| Grandeur | Mesuré | Retenu |
|---|---|---|
| Distance, de face | décode à 1,5 m, échoue à 1,75 m | 1,25 m |
| Angle avec la normale | 55° | 50° |
| Vitesse | non mesurable en rendu fixe | 0,6 m/s |
| Temps de visée | — | 2 images (33 ms chacune) |

- Vitesse : seuil déclaré d'après une loi publiée (Cristiani 2020, P = 1/(v+1)^kr).
- La mesure « décode à 1,5 m, échoue à 1,75 m » figure aussi dans
  `../../../experiments/01_resolution_qr/` (section B de ses résultats), qui en déduit un QR
  de 6,7 à 7,9 cm de côté (7,3 cm retenu).

### 2. Caméra frontale puis deux caméras latérales (choix du 2026-07-24)

- Caméra frontale : lecture 0,91 avec le bruit d'exploration, 0,11 sans bruit. La visée fine
  ne marchait que grâce au hasard des actions.
- Avec la caméra frontale, la meilleure lecture était vers σ ≈ 0,20-0,35 (σ = bruit des
  actions) et elle s'effondrait sous 0,15. `../eval_map.py` mesure la même courbe avec les
  caméras latérales.
- Deux caméras latérales (±90° du cap) : tous les QR des vraies configurations sont
  lisibles géométriquement (1,00) en longeant les racks.

### 3. Taux de lecture observés pendant les entraînements

| Observation | Valeur |
|---|---|
| Moyenne de lecture mesurée (34 entraînements bloqués au cran 0) | 0,39 |
| Lecture maximale tant que les façades à QR étaient indiscernables | ~0,4 |
| Modèle promu au cran suivant avec une moyenne de | 0,62 |
| Même modèle, remesuré au cran 0 | 0,551 |
| Sa courbe au moment de la promotion (sans plateau) | 0,28 → 0,60 |
| Un modèle joué sans bruit, puis avec bruit (raison de la phase de convergence) | 0,05 contre 0,66 |

Autres faits notés : 19 entraînements « sains » avec la boucle d'entraînement finale ;
3 entraînements perdus par des moyennes d'action hors bornes ; quatre semaines perdues à
cause d'un environnement où le hasard pouvait lire des QR (d'où `../verify_env.py`).

### 4. Problèmes observés → corrections

| Problème observé | Correction |
|---|---|
| Vitesse limitée dès le cran 0 (0,75 m/s) : 22 % des actions passaient la règle (11 % au nominal) ; aucun passage de cran en 34 entraînements | cran 0 à 1,45 m/s, au-dessus de la vitesse 3D max (1,414 m/s) |
| Barre de passage 0,88 tenue 25 épisodes : simulation Monte-Carlo sur les taux mesurés (moyenne 0,39) → 0,00 % de chances, encore 0,00 % à 0,75 | barre baissée |
| Modèle promu à 0,62 trop tôt : 0,551 au cran 0, courbe encore montante | barre remise à 0,88, identique à tous les crans (2026-07-27) |
| Temps de visée de 1 pas (33 ms) : irréaliste | 2 images à tous les crans |
| Règle de vitesse sans la montée : un drone montant à 1,5 m/s était vu « immobile » | vitesse comptée en 3D |
| Lectures gratuites au départ (vitesse nulle, drone posé près d'un QR) | aucune lecture pendant les 15 premiers pas (0,5 s) |
| Carte de couverture coupée 47,7 % du temps (seuil 1,0 m/s) | seuils 1,5 m/s et 1,6 rad/s, jamais bloquants |
| Pénalité d'à-coups qui punissait le bruit : à σ = 0,29, −454 par épisode dus au bruit seul pour −486 mesurés ; pente −3132 par unité de σ ; entropie 2,90 → 0,41 en 1600 itérations | pénalité divisée par 10 (0,005) : −45 par épisode |
| Pénalité de proximité « en marche » (1,0) : −1098 par épisode dans les bons épisodes contre −111 dans les mauvais ; annulait le revenu des lectures (+1142) | pénalité graduée de 0,25 sous 0,6 m |
| Couverture divisée par la taille de l'empreinte (105 cases en latéral contre ~13 en frontal) : revenu ÷ 8 ; couvrir toute l'arène = 124 points contre 111 QR × 25 = 2775 (entraînement « socle_v2 ») | unité d'aire fixe de 13 cases : une tranche complète vaut ~1000 points |
| Peu de cartons portant un QR : façades à QR indiscernables, lecture plafonnée à ~0,4 | 95 à 100 % des cartons avec QR, aucun QR « déjà lu » au départ |
| QR trop hauts payés 25 points sans compter dans le total | seuls les QR lisibles comptent (hauteur ≤ 4,55 m) |
| Épisode de 120 s infaisable au cran 0 (plan optimal 131 s, calculé avec 0,75 m/s) | 150 s au cran 0, +15 s par cran, plafond 295 s (plan optimal au nominal : 207 s) |
| Drones identiques qui se regroupaient : pénalité de proximité 10× plus forte dans les bons épisodes, couverture d'équipe négative | identité du drone ajoutée à l'observation |
| Aide visant la posture nominale à tous les crans : double travail infaisable en 120 s, curriculum bloqué | l'aide vise les QR non lus à la règle du cran courant |
| Aide « être lent » calculée par rapport à 0,6 m/s : nulle entre 0,6 et 1,5 m/s | calculée par rapport à la limite du cran courant |
| Couverture liée à la règle de lecture : ne pas lire devenait rentable, blocage aux crans 5-6 | couverture indépendante (portée 2,5 m, vitesses fixes) |
| Diagonale plus rapide que la limite (2,12 m/s quand elle était de 1,5) | vitesse horizontale ramenée à la limite |
| Drone en panne qui dérivait encore ~1 s | sa vitesse est remise à zéro |
| Lectures des départs aidés comptées sans être payées (lecture relancée après le reset) | lecture bloquée entre le reset et le pas suivant |
| Départ aidé nez vers le QR : aucune caméra latérale ne le voit | départ le flanc tourné vers le QR |
| Recouvrement compté contre sa propre trace : ~105 cases par pas, −225 par drone et par épisode ; la panne devenait rentable | recouvrement compté avec les coéquipiers seulement |
| Moyennes d'action éjectées hors de [−1, 1] puis coincées (3 entraînements perdus) | borne douce à ±3 |
| Écart-type σ des actions monté jusqu'à 9 | σ limité entre 0,05 et 1,2 |
| Validations faites à `--level 7` avec 6 crans : règle 37 % plus stricte (0,79 m au lieu de 1,25 m) ; les « zéro lecture » ont été pris pour un environnement sain | avancement du curriculum plafonné à 1 |
| Évaluations biaisées : fin anticipée (seuls les entrepôts rapides comptés), configurations non tirées à la première vague, bruit qui changeait les entrepôts | les trois corrigés dans `../eval_map.py` |

### 5. Vérifications et critère de réussite

- `../verify_env.py` : une politique au hasard, immobile ou « tout droit » doit lire moins
  de 1 % au gate nominal ; accélération p99 < 0,5 g ; moins de 20 % des lectures juste après
  une survitesse.
- Critère de réussite final : 0,85 de QR lus à la règle nominale, mesuré par `../eval_map.py`.
- Comparaison avec des lignes de base (inférence active, balayage en serpentin, méthode de
  Pore et al.) : `../../../scripts/run_baseline_matrix.sh`.

## Conclusion

- L'essentiel du travail consigné porte sur un environnement qui ne se laisse pas tromper
  et une récompense équilibrée : 26 corrections, souvent après des entraînements bloqués.
- La contribution finale du projet est `swarm_qr/`, qui suit une autre approche.

## Limites : ce que ce test ne prouve pas

- Pendant l'entraînement, la lecture est une règle géométrique calibrée sur le vrai
  décodeur, pas un décodage d'image ; la limite de vitesse (0,6 m/s) est un seuil déclaré,
  tiré d'une loi publiée (Cristiani 2020).

## Fichiers de résultats et figures

- Sorties des scripts : `swarmscan_runs/<date>_<nom>/` (modèles et TensorBoard),
  `docs/calibration_gate.csv` (calibration), `/tmp/traj.npz` (trajectoires).
- Sources des chiffres : `../config_map.py` (la plupart), `../env_map.py`, `../mapping.py`,
  `../models.py`, `../curriculum.py`, `../train.py`, `../eval_map.py`, `../verify_env.py`,
  `../../tests/test_swarmscan_map_pure.py`, et `../../../experiments/01_resolution_qr/` pour la calibration.
