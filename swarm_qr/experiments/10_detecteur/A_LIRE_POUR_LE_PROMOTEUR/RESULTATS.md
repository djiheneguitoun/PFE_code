# 10 — Le détecteur appris (étape 7) : résultats

## La question

Le décodeur classique lit de près, de face et lentement : l'enveloppe de l'étape 2 s'arrête à
4 m. Un petit réseau qui ne lit pas mais **repère** QR et cartons voit-il nettement plus loin
que le repérage classique, **sans inventer d'objets** ? Un QR repéré de loin deviendra une
piste sur la carte ; un carton repéré remplira le canal sémantique laissé vide à l'étape 4.

## Méthode (en bref)

- **Cadres calculés sans un clic** : chaque objet connu est projeté dans l'image ; 25 points
  par panneau (et par face de carton) sont testés par un rayon du moteur physique, et le cadre
  entoure les points visibles. Visible à moins de 30 % (QR) ou 20 % (carton) : « ignoré » (ni
  exemple, ni fausse alerte). Tout le visible est annoté, dès 8 pixels de côté.
- **Deux classes** : le QR (code seul, sans marge blanche) et le carton ; les zones (allée,
  rack, mur) viennent de la géométrie de la carte.
- **Huit entrepôts** : apprentissage sur les graines 0 à 5 (500 images chacune) ; test sur les
  entrepôts scellés 9033 et 9019 (400 images chacun). Poses au hasard : 70 % vers le rack le
  plus proche, le reste n'importe où, pour apprendre aussi ce qui n'est pas un QR.
- **Juge commun** : les 3 577 images de l'étape 2, ré-annotées. Comparaison au repérage
  classique **sur les mêmes photos**, dont 300 sans QR où le classique inventait 27 % de motifs.
- **Réseau** : YOLO11 nano (2,6 M paramètres), demi-précision, carte graphique de 8 Go,
  entrées de 1024 et 640 pixels ; plus gros seulement si le nano ne suffit pas.
- **Seuil de mission** : le plus bas qui garde les fausses alertes sous 1 % des images sans
  QR ; 0,5 pour les deux variantes.

| Jeu | Images | QR | Cartons | Ignorés |
|---|---|---|---|---|
| Apprentissage (6 entrepôts) | 3 000 | 23 016 | 26 304 | 13 325 |
| Scellés (2 entrepôts) | 800 | 5 917 | 6 495 | 2 580 |
| Étape 2 ré-annotée (test) | 3 577 | 19 615 | 24 625 | 14 949 |

Les QR annotés vont jusqu'au fond de l'entrepôt : 9 % à moins de 4 m, 36 % au-delà de 12 m.

## Résultats

### 1. Contrôle des cadres avant l'entraînement

Un cadre décalé n'arrête pas l'entraînement : il le fausse en silence.

| Contrôle | Résultat |
|---|---|
| Calibration relue dans le simulateur | fx = 886,8, la valeur calculée |
| Panneau dégagé de face à 2 m | cadre de 149 px pour 149 attendus, 8 entrepôts |
| Centre du panneau visé (projection de l'étape 2) dans le cadre | **99,4 %** (2 000 images) ; **100 %** (9019, vol, traversée) |
| Planches regardées à l'œil | 24 images, cadres sur les codes et les cartons, gris sur les objets cachés |

### 2. Trois faits découverts en fabriquant les cadres

| Problème observé | Correction |
|---|---|
| Les cartons non retenus étaient seulement invisibles : 9 sur 9 arrêtaient encore les rayons, donc le lidar de l'étape 4 voyait des obstacles fantômes dans les racks. | `scene._hide` désactive l'objet (ni rendu, ni physique). Les chiffres de l'étape 4 datent d'avant. |
| Le collider (forme physique) d'un carton est plus petit que sa forme visible : un rayon oblique touche le carton 10 cm plus loin que prévu ; BOX_109 à 2,4 m était « caché » (5 points visibles sur 25). | On juge par l'identité du premier objet touché : si c'est le carton du QR, rien ne s'interpose. Même panneau : 23 sur 25. |
| Le contrôle d'aplomb prenait le panneau le plus proche de la hauteur de vol ; dans l'entrepôt 5 il était à moitié caché, d'où un arrêt à tort. | Témoin cherché parmi 40 candidats : caméra hors rack, panneau visible à 95 %. |

### 3. Le banc (seuil de mission 0,5)

| Mesure | Classique | Appris 1024 (retenu) | Appris 640 |
|---|---|---|---|
| Panneau visé repéré jusqu'à 6 m | 100 % jusqu'à 4 m, 75 % à 5–6,5 m | **100 %** | 100 % |
| Panneau visé repéré à 6–8 m | 47 % | **98 %** | 98 % |
| Portée tenue à 90 % | 4,0 m | **8,0 m** (limite des images) | 8,0 m |
| Images sans QR avec un QR inventé (300) | 27,3 % | **0,7 %** | 0,3 % |
| QR trouvés, scellés (5 917) | — | **98,3 %** | 94,4 % |
| … dont à 8–12 m (4 195) | — | **98,3 %** | 93,4 % |
| Cartons trouvés, scellés (6 495) | — | **94,1 %** | 91,4 % |
| Faux QR par image, scellés | — | 0,02 | 0,02 |
| Temps par image (demi-précision) | — | 12 ms | 12 ms |
| Mémoire GPU | — | 63 Mo | 49 Mo |
| Paramètres / entraînement | — | 2,6 M / 48 min | 2,6 M / 21 min |

- **La portée justifie le composant** : là où le classique perd un panneau sur deux, le réseau
  en garde 98 sur 100 ; sur les entrepôts scellés, il garde 98 % des QR jusqu'à 12 m, la limite
  de ce qui a été annoté. Un drone qui longe une allée voit les codes de toute l'allée, pas
  seulement ceux à 4 m.
- **Les fausses alertes le rendent utilisable** : chaque objet inventé coûte un trajet perdu.
  Classique : 27 % des images vides ; réseau : moins de 1 %. Scellés : 2 faux QR pour 100
  images, dont une partie est encore refusée par la carte (hors enveloppe ou hors entrepôt).
- **Choix de la 1024** : même portée et même temps que la 640 (à une image par observation, le
  coût vient du transfert, pas du calcul), mais quatre points de QR et cinq points de cartons de
  plus : elle est retenue. Un modèle plus gros n'a pas été entraîné : le nano tient la barre.
- **Effet du seuil** : à 0,25, la 1024 trouve 99,4 % des QR mais invente un QR sur 2,3 % des
  images vides ; à 0,5, 98,3 % et 0,7 %. On garde 0,5 : un point de rappel contre trois fois
  moins de fantômes.
- mAP50 de validation (`../entrainement.json`) : 0,986 (1024) et 0,974 (640).

### 4. En vol, dans la patrouille de l'étape 4

Un drone, une étagère, deux allées, détecteur branché : un QR repéré devient une piste, un
carton repéré marque le canal sémantique. Jugement de l'étape 4.

| Problème observé au premier vol (`../vol_avant_correction/`) | Cause et correction |
|---|---|
| Pistes dans les racks (1 sur 152 hors emprise) mais mal placées : 43 % à moins de 50 cm d'un vrai panneau, 71 % à moins de 1 m, 99 % à moins de 2 m. | À 8 m, deux rayons du lidar sont espacés de 28 cm : le rayon visé tombe sur le carton voisin ou au fond du rack. Les pistes lointaines sont désormais placées par la carte : premier cube occupé dans la direction du cadre. |
| 7 points de trajectoire « dans un rack » (0 à l'étape 4) : traversée du bout du rack du milieu à 3,17 m. | 1er diagnostic, **faux** : lidar trop clairsemé (un anneau tous les 4° ne touche une planche 24 cm sous le drone qu'à 1,4 m devant) et planificateur limité à ±0,6 m. Passage à 2° (3 420 rayons) et ±0,85 m (rayon du drone + oscillation verticale) : encore 4 points. Vraie cause, mesurée : aucune structure solide aux bouts des racks (0,70 m au sud, 0,77 m au nord ; seulement un panneau en haut et un pare-chocs bas). La carte avait raison ; l'arbitre juge désormais la structure solide. |

Réserve : les changements de lidar et de planificateur sont gardés (physiquement fondés, tests
passés) mais n'ont corrigé aucun défaut mesuré, et le lidar coûte 44 ms au lieu de 24.

Second vol, arbitre corrigé (`../vol/`), quatre allers :

| Mesure | Résultat |
|---|---|
| Allers atteints | 4 sur 4, 3 chemins recalculés |
| Points dans une structure de rack | **0 sur 819**, au plus près 0,55 m |
| Codes lus (1 étagère sur 3, 2 allées sur 4) | 45 sur 114, aucun inventé, 1,0 cm d'erreur médiane |
| Pistes du détecteur | 143 ; **77 %** à moins de 1 m, **99 %** à moins de 2 m |
| Pistes hors de toute emprise (fantômes) | **1 sur 143** |
| Repérage classique de l'étape 4 | 47 % / 86 %, 6 fantômes sur 182 |
| Cubes « carton » à moins de 60 cm / 1 m / 2 m d'un vrai | 826 ; 68 % / 81 % / 98 % |
| Coût par observation (2 caméras) | détecteur 33 ms, lidar 44 ms, carte 4 ms |

Le réseau ne fait pas courir les drones après des objets imaginaires. La position d'un objet
vu de loin reste grossière (direction par le cadre, distance par le lidar de près et par la
carte de loin, à un cube près), mais une piste à 1 m suffit pour décider où aller : la lecture
de près donne ensuite la position au centimètre.

### 5. Carte et tests
- Le canal sémantique porte maintenant les cartons repérés ; les zones se déduisent de la
  géométrie de la carte (le guide de l'étape 8 en aura besoin).
- 45 tests sans simulateur en 10 s : les 42 des étapes 3 et 4, plus le canal sémantique, la
  géométrie d'une détection et le premier obstacle le long d'un rayon de la carte.

## Conclusion

- Le détecteur double la portée de repérage (8 m contre 4 m ; 98 % contre 47 % à 6–8 m) et
  rend les fausses alertes rares (0,7 % contre 27 % des images sans QR), pour 12 ms et 63 Mo.
- Sur des entrepôts jamais vus, il trouve 98,3 % des QR et 94 % des cartons ; ses cadres
  d'apprentissage sont justes (99,4 à 100 % contre l'étape 2).
- En vol, il ne crée presque pas de fantômes (1 piste sur 143) et la patrouille reste sûre
  (0 point sur 819 dans une structure de rack).
- Il est retenu (nano, 1024 px, seuil 0,5) et installé dans le système.

## Limites : ce que ce test ne prouve pas

- La portée de 8,0 m est la limite des images de l'étape 2 ; sur les entrepôts scellés, la
  mesure va jusqu'à 12 m, la limite de ce qui a été annoté.
- La position d'un objet vu de loin reste grossière (à un cube près) : elle dit où aller, pas
  où est le code au centimètre.
- Les changements de lidar et de planificateur faits au premier vol n'ont corrigé aucun défaut
  mesuré, et le lidar coûte 44 ms par observation au lieu de 24.
- Les chiffres de l'étape 4 ont été mesurés avant la correction des cartons cachés.

## Fichiers de résultats et figures

- `../resultats.json` : bilan de chaque variante, variante retenue, seuil de mission.
- `../entrainement.json` : bilan des entraînements.
- `../portee.png` : repérage selon la distance (panneau visé ; objets des entrepôts scellés).
- `../controle/` : planches et `verification_cadres.json`.
- `../runs/n1024/results.png`, `../runs/n640/results.png` : courbes d'apprentissage.
- `../vol/` et `../vol_avant_correction/` : `resultats.json`, `carte_finale.png`,
  `carte_qui_se_remplit.mp4`, `carte_3d.html`.
- `../../../assets/detecteur/detecteur.pt` et `detecteur.json` : le modèle installé.
