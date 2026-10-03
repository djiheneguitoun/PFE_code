# rl_inventory — Essai d'apprentissage par renforcement : résultats

## La question

Peut-on entraîner par renforcement (RL) un essaim de 3 drones, dans le vrai entrepôt simulé,
à lire tous les QR des cartons ? Avant tout entraînement : l'arène est-elle juste (le drone
vole, le LiDAR voit l'entrepôt, la règle de lecture marche) et assez rapide pour une carte
graphique de 8 Go ?

## Méthode (en bref)

- Arène Isaac Lab (`../env.py`) : entrepôt de NVIDIA, 123 cartons, drones Crazyflie pilotés
  en vitesse, LiDAR (capteur laser de distances) de 1800 rayons, règle géométrique
  « QR lu » à l'entraînement (le vrai décodeur `pyzbar` est réservé à l'évaluation).
- Deux arènes : `QRInventoryEnv` (1 drone, sans récompense, pour les tests) et `SwarmQREnv`
  (3 drones, la tâche entraînée).
- Vérifications pas à pas (T1.2 à T1.5b) puis test de l'essaim à 3 drones (`../tests/`).
- Diagnostics de coût (`../diag/`) : mémoire graphique, temps par pas, vitesse du LiDAR.
- Entraînement : IPPO avec skrl, 16 entrepôts en parallèle, 100 000 pas (`../train_ppo.py`) ;
  évaluation sans hasard avec `../eval.py`.
- Les mesures ci-dessous sont celles notées dans les commentaires du code, avec leur fichier
  source.

## Résultats

### 1. Ce que vérifient les tests

| Test | Situation | Réussi si… |
|---|---|---|
| T1.2 `test_drone.py` | 120 pas plein avant | déplacement moyen en x > 0,3 m |
| T1.3 `test_lidar.py` | drone immobile, 5 pas | distance mini < 7,5 m et > 5 % de rayons qui touchent |
| T1.5a `test_qr_placement.py` | pose des QR | contrôle à l'œil (image ou fenêtre) |
| T1.5b `test_qr_read.py` | drone à 1,2 m face au QR 0, immobile | ce QR lu ; aucun QR lu par un drone resté au départ |
| `test_swarm.py` | 12 pas aléatoires, 3 drones | observation 1820, état 5440, récompenses finies, chaque drone a bougé > 0,1 m |

Tailles : observation d'un drone = 1800 distances LiDAR + 13 (position, orientation,
vitesses) + 6 (position des 2 voisins) + 1 (part de QR lus) = 1820. État global vu par le
critique de MAPPO = 3 × 13 + 3 × 1800 + 1 = 5440.

### 2. Problèmes observés → corrections (chiffres des commentaires du code)

| Problème observé | Correction |
|---|---|
| Sans modèle d'actionneur, le drone passait de 0 à 1,5 m/s en un pas : 45 m/s² = 4,59 g. Un Crazyflie plafonne à 3,57 m/s² (≈ 0,36 g, inclinaison 20°) ; même à poussée maximale (poussée/poids 1,82), 14,9 m/s². 45 m/s² exigeraient 77,7° d'inclinaison et un rapport poussée/poids de 4,69. | `../actuator.py` : la vitesse réelle suit la consigne avec retard (0,30 s ; 0,15 s en lacet) et accélération bornée (3,6 m/s² à plat, 3,0 m/s² en vertical) |
| Conséquence mesurée : le bruit d'exploration créait des creux de vitesse d'un seul pas qui validaient « drone lent » sans vrai ralentissement ; 87,9 % des lectures d'une politique aléatoire suivaient une survitesse. | même correction |
| Près du plafond, la butée d'altitude coupait la vitesse d'un coup : 0,71 g mesuré pour un modèle limité à 0,37 g. | butée appliquée à la vitesse réelle, correction limitée à 3,0 m/s² × durée du pas |
| Descente bridée à 0,5 m/s (montée 1,0) : un bruit de moyenne nulle donnait +0,060 m/s, soit +9 m par épisode ; drones collés au plafond, 36 des 90 QR invisibles. | vitesse verticale symétrique : 1,0 m/s |
| Vitesse horizontale de 1,5 m/s jugée « non tenable ». | 1,0 m/s (valeur du firmware Crazyflie) |
| LiDAR dynamique d'Isaac coûteux (`../diag/profile_step.py`). | lancer de rayons statique sur un maillage partagé : environ 100 fois plus rapide (`../diag/test_static_raycast.py`) |

Autres choix motivés dans le code :
- le filtre de vitesse travaille en repère monde : en repère drone, la vitesse tournerait
  avec le nez du drone (inversion de vitesse par simple pivot) ;
- la vitesse est remise à zéro au début d'un épisode : sinon le drone repartirait avec
  l'élan de l'épisode précédent ;
- la collision coûte des points à chaque pas mais n'arrête pas l'épisode : Isaac Lab ne
  réinitialise un entrepôt que si ses 3 drones ont fini.

### 3. Réglages retenus (`../config_rl.py`, `../agents/`)

| Règle « QR lu » | Valeur |
|---|---|
| distance | ≤ 3 m |
| champ de la caméra | ± 60° autour du nez |
| angle avec la face du QR | ≤ 35° |
| vitesse / lacet | ≤ 0,6 m/s et ≤ 0,8 rad/s |
| durée de visée | 2 pas (≈ 67 ms) |

| Récompense d'un drone | Points |
|---|---|
| QR lu (nouveau) | +10 |
| tous les QR lus | +100 |
| bien placé pour lire | +0,5 par pas |
| mètre gagné vers le QR non lu le plus proche | +1 |
| temps | −0,1 par pas |
| obstacle à moins de 0,25 m | −5 par pas |
| à-coups | −0,01 × ‖Δaction‖² |

| Simulation et entraînement | Valeur |
|---|---|
| physique / décision | 120 Hz / 30 Hz |
| épisode | 45 s (1350 décisions) |
| entrepôts en parallèle | 16 |
| vitesses max | 1,0 m/s par axe, 1,5 rad/s en lacet |
| altitude | 0,3 à 3,0 m (départ 1 m) |
| réseaux | 256-256-128 (critique MAPPO : 512-256-128) |
| PPO | 24 pas par mise à jour, 5 passes, γ = 0,99 |
| entropie | 0,05 (IPPO), 0,01 (MAPPO) |
| durée | 100 000 pas, graine 42 |

### 4. Chiffres de la version 2 notés dans `../tests/`

Ces chiffres concernent `../swarmscan_map/` (détails dans son propre guide) :
- 34 entraînements bloqués au premier cran du curriculum (difficulté qui monte avec la
  réussite) : à ce cran, seules 22 % des actions respectaient la limite de vitesse ;
- la barre de passage (moyenne glissante > 0,88 pendant 25 épisodes) était hors d'atteinte
  d'une performance qui plafonnait à 0,39 ;
- entraînement « socle_v2 » : couvrir toute l'arène rapportait 124 points, contre
  111 × 25 = 2775 pour les lectures.

## Conclusion

L'essai a établi une arène d'apprentissage complète et testée : vrai entrepôt, 123 QR
uniques, essaim de 3 drones au comportement physique réaliste (accélérations du Crazyflie),
LiDAR par lancer de rayons statique, règle de lecture, récompense, scripts d'entraînement et
d'évaluation. Les mesures notées dans le code ont surtout servi à corriger des failles de la
simulation que la politique pouvait exploiter (fausses lectures par creux de vitesse, dérive
vers le plafond).

## Limites : ce que ce test ne prouve pas

- À l'entraînement, la lecture d'un QR est une règle géométrique (proxy), pas un décodage
  d'image ; le vrai décodeur `pyzbar` sert à l'évaluation.
- Les poids de la récompense et la vitesse maximale de lecture (0,6 m/s) sont notés
  « à calibrer » dans `../config_rl.py`.

## Fichiers de résultats et figures

- `../assets/qr/` : les 123 images QR générées (`CARTON_0000.png` à `CARTON_0122.png`).
- Les images `view_*.png` de `../diag/render_view.py` et `view_qr_closeup.png` de
  `../tests/test_qr_placement.py` sont écrites dans le dossier de sortie de ces scripts, sur
  la machine de simulation.
