# 07 — Enveloppe de lecture : résultats

## La question
Jusqu'où, sous quel angle et à quelle vitesse le drone lit-il un QR ? Le contrôleur devra
respecter ces limites, et la décision s'en servira pour placer les drones.

## Méthode (en bref)
- 3 481 images, chacune avec la pose vraie de la caméra, relue dans le simulateur. Six lecteurs
  de QR lisent exactement les mêmes images.
- Cinq campagnes : 2 000 poses au hasard (caméra libre) ; 300 dans l'entrepôt vidé de ses codes ;
  400 dans un second entrepôt ; 12 positions tenues par le vrai drone ; 4 traversées du rack.
- Visée volontairement imparfaite (panneau décalé dans l'image), comme sur un vrai drone.
- Contrôles avant chaque campagne (calibration, retard du rendu mesuré : 5 rendus, lecture à
  distance connue), arrêt si l'un échoue. Distance vue et pose enregistrée concordent à
  0,7–1,1 cm près (médiane) selon les campagnes, 0,3 cm pour les traversées.

## Résultats

### Distance et angle : une seule limite
Vu de biais, le code paraît plus petit, comme plus loin. On utilise donc la distance apparente =
distance ÷ cosinus de l'angle (sa distance s'il était vu de face). zxing, 2 000 poses :

| distance apparente | lues | images |
|---|---|---|
| 1,0 à 1,25 m | 75 % | 84 |
| 1,25 à 1,5 m | 89 % | 115 |
| 1,5 à 4 m | 91 à 99 % | 925 |
| 4 à 5 m | 74 % | 167 |
| 5 à 6,5 m | 31 % | 194 |
| plus loin | 3 % | 185 |

Zone fiable : 1,5 à 4 m. La borne basse est une marge pour une visée réaliste : bien centré, le
drone lit à 0,5 m (test 3, `../../03_images_qr/`).

### La vitesse ne coûte rien jusqu'à 1 m/s
À 5 images par seconde, en ne comptant que les images où le panneau est dans le cadre :

| vitesse | images | lues | intervalle à 95 % |
|---|---|---|---|
| 0,1 m/s | 45 | 100 % | 92–100 % |
| 0,3 m/s | 18 | 100 % | 82–100 % |
| 0,6 m/s | 10 | 100 % | 72–100 % |
| 1,0 m/s | 5 | 100 % | 57–100 % |

### Le vrai drone lit comme la caméra idéale
- 12 positions tenues : 100 % entre 1 et 4 m de distance apparente, puis 91 % entre 4 et 5 m,
  comme la caméra libre le prévoit. Le vol n'ajoute aucune difficulté.
- Sous 1 m, il ne tient plus son angle : visant 90 cm de face, il s'est stabilisé à 74 cm et 29°
  (si près, un petit écart latéral fait un grand angle). Il faut alors demander une zone.

### Aucun code inventé, mais repérer n'est pas lire
- Entrepôt vidé de ses 228 panneaux : 0 lecture sur 300 images, pour les six lecteurs. Une
  lecture peut être crue sans vérification.
- Repérer (voir un carré qui ressemble à un QR) porte plus loin que lire (99 % des images
  repérées entre 5 et 6,5 m, 31 % lues), mais sans aucun code le repérage voit un panneau sur
  27 % des images (intervalle 23–33 %) : cartons, montants, ombres. D'où l'étape 7.

### Les lecteurs, sur les mêmes images
Zone utile : moins de 2 m et de 30°. Portée : tranche la plus lointaine lue à 90 % ou plus.
Erreur : sur la distance du code (médiane).

| lecteur | zone utile | portée | erreur |
|---|---|---|---|
| zbar | 91,2 % | 4,50 m | 0,4 cm |
| zxing (retenu) | 90,2 % | 3,65 m | 0,8 cm |
| pyboof | 87,2 % | 4,50 m | 1,0 cm |
| opencv | 80,8 % | 1,65 m | 1,0 cm |
| opencv aruco, image ×3 | 79,8 % | — | 1,6 cm |
| opencv aruco (ancien) | 79,3 % | — | 1,8 cm |

zxing et zbar dominent nettement, et les deux sont rapides : zxing reste le lecteur retenu
(17 ms par image) ; zbar, un peu plus loin ici, est l'alternative immédiate. L'ancien lecteur
perdait onze points.

### Position d'un code lu, autre entrepôt
- Position à 0,8 cm près (médiane), moins de 3 cm dans 9 cas sur 10 (90e centile : 3,04 cm) :
  ce qu'il faut à la carte de l'étape 4. Vaut pour un code lu, pas pour un motif repéré.
- Second entrepôt (autre panneau, autre éclairage) : 90,4 % contre 90,2 %. C'est une propriété
  de la caméra et du code, pas du lieu.
- L'allée fait 3,92 m : pas de recul au-delà de 3,6 m. C'est l'angle, presque gratuit, qui
  permet de couvrir plusieurs cartons d'une même position.

### Contrôle de la campagne précédente
Refaite entièrement, elle redonne 90,2 %, 3,65 m, 90,4 % et zéro lecture fantôme. Corrigé : la
« distance minimale d'un mètre » venait d'une visée décalée de 10 cm ; l'effet de la vitesse est
désormais mesuré.

## Conclusion
| à retenir | valeur |
|---|---|
| Zone fiable | distance apparente 1,5 à 4 m (≥ 90 % par image) |
| Vitesse | aucun effet jusqu'à 1 m/s |
| Recul maximal en allée | 3,6 m (limite physique, pas optique) |
| Lecteur | zxing (zbar en alternative) |
| Position d'un code lu | 0,8 cm en médiane |
| Codes inventés / motifs repérés à tort | aucun / 27 % |

## Limites : ce que ce test ne prouve pas
- Pas de flou de bougé dans le simulateur : le résultat sur la vitesse vaut pour des images nettes.
- Peu d'images rapides (5 à 1 m/s), mais aucun échec nulle part.

## Fichiers de résultats et figures
- `../resultats.json` : tous les chiffres par campagne et par lecteur ; `../temps_lecteurs.json`.
- `../enveloppe.png` : lecture selon la distance (de face), l'angle (à moins de 2 m) et la
  distance apparente ; pointillés : repérage ; ligne rouge : seuil de 90 %.
- `../images_*/`, `../poses_*.jsonl`, `../meta_*.json` : images brutes et poses (voir `README.md`).
