# 09 — La carte partagée : résultats

## La question
La carte construite en vol dit-elle vrai, et suffit-elle à guider le drone dans un entrepôt
qu'il découvre, sans jamais consulter le plan ?

## Méthode (en bref)
- Grille de cubes de 25 cm (occupé, libre, inconnu, côté vu de près) + table des codes lus.
- Lidar (capteur laser qui mesure les distances) : 1 800 rayons par image ; les preuves
  s'additionnent (vue libre 10 fois puis occupée 1 fois, une case reste libre).
- Case « couverte » : un code posé là aurait été lisible (1,5 à 4 m, enveloppe de l'étape 2,
  rien devant, bon côté).
- Chemins sur la carte : l'inconnu coûte 3 fois le libre connu ; revérifiés toutes les 0,4 s.
  (Valeur portée ensuite à 20 à l'étape 5 : `COUT_INCONNU` dans `swarm_qr/mapping.py`.)
- Équipe : réservations qui expirent (un drone muet perd les siennes : panne gérée sans code
  dédié), liste noire, frontières prêtes pour l'étape 5.
- Patrouille : 1 drone, entrepôt 9033, 5 faces × 3 étagères = 15 allers, à 2,2 m des racks
  (0,96 m dans l'allée étroite ouest), 5 photos par seconde, 2 caméras.

## Résultats

### Contrôles, avant toute mesure (le banc s'arrête si l'un échoue)
| Contrôle | Résultat |
|---|---|
| Rayons du lidar refaits par le moteur physique, 3 caps | **0,0 cm** d'écart médian, 0 à 3 % de rayons faux |
| Le monde bouge-t-il quand le drone tourne ? | non : 0,0 cm entre les nuages à 0 et 90° |
| Obstacles inventés dans les allées | aucun : 100 % des points à hauteur de vol sur un rack ou un mur |
| Coût d'une observation | 18 ms de lidar, 2 ms de couverture |
| Mémoire | 1,5 Mo pour 258 048 cases |

Ces contrôles ont servi : le lidar compte ses angles verticaux vers le bas et ne se met à
jour qu'au rendu. Sans eux, la carte se serait remplie tête-bêche avec des chiffres
d'apparence normale.

### La patrouille
| Mesure | Résultat |
|---|---|
| Entrepôt connu | **89,5 %** des cases |
| Codes lus | **110 sur 114**, sur 171 faces, toutes confirmées par au moins 2 lectures |
| Codes inventés | **aucun** |
| Position d'une face lue | **0,9 cm** en médiane, 1,2 cm dans 9 cas sur 10, 8,3 cm au pire |
| Fausses cases occupées en pleine allée | 0,39 % |
| Cartons dont l'obstacle est posé sur la carte | 82 à 90 % selon l'étagère |
| Promesse de lisibilité | **95 %** des panneaux annoncés lisibles sont lus, contre 37 % des autres |
| Transits et traversées | 15 sur 15 atteints, aucun abandon, 3 chemins recalculés en route |
| Vol guidé par la seule carte | **aucun** des 2 738 points de trajectoire dans un rack |
| Chemins coin à coin sur la carte finale | 12 sur 12, **aucun sur un obstacle connu** |
| Coût par observation en vol | 22 ms de lidar + 2 ms de couverture + 19 ms de décodage |

- La position au centimètre est le chiffre clé : c'est la précision avec laquelle
  l'inventaire situera chaque carton.
- Promesse de lisibilité : quand la carte dit « un code posé là aurait été lu », c'est vrai
  95 fois sur 100. Les 37 % lus sans être annoncés sont des lectures plus proches que
  l'enveloppe (allée étroite, seconde caméra qui lit la face d'en face à 1 m).

### Cinq faits découverts en comparant à la vérité
Aucun ne provoquait d'erreur ; tous ont été trouvés parce que le jugement compare la carte à la
vérité, chiffre par chiffre.

| Fait observé | Correction |
|---|---|
| Étiquettes de tailles différentes (40 cm, 21 cm, moins) : un code 2 fois plus petit paraît 2 fois plus loin, placé à 1,3 m de sa place | la distance vient du lidar ; l'image ne donne que l'identité et l'orientation (les 5 000 positions de la patrouille) |
| Même code sur les 2 faces d'un carton : la moyenne le plaçait au milieu du carton | une lecture rejoint la face connue la plus proche ; une 3e face lointaine est une erreur de décodage, refusée (1 sur 5 000) |
| Les racks sont des cadres ouverts : une case vue à travers ne rend pas lisible la face opposée | la couverture retient le côté d'où chaque case a été vue |
| Le décodeur tous formats lisait les codes-barres des montants | décodeur restreint aux QR |
| Montants fins : à 8 m, deux rayons sont espacés de 28 cm ; le drone a frôlé un montant d'angle | chemin revérifié en vol : 3 recalculs, plus aucun point dans un rack |

### Fausses détections du repérage classique
182 motifs repérés sans être lus, dont 93 % ne correspondent à aucun vrai panneau : ce sont
les 27 % de faux repérages de l'étape 2, accumulés sur 2 800 images. La carte en refuse déjà
beaucoup (position hors de l'enveloppe ou de l'entrepôt) ; le reste est le travail de l'étape 7.

### Tests sans simulateur
28 tests sur un monde de boîtes à vérité connue, en 8 s : occupation (rien d'inventé derrière
un mur), couverture (enveloppe, côté), deux faces, refus des positions absurdes et d'une 3e
face, réservations, drone muet, frontières, chemins (contournent un mur, préfèrent le libre
connu, se déclarent coupés si un mur apparaît, sortent de la marge d'un obstacle).

## Conclusion
Le lidar dit vrai (0,0 cm) ; la carte connaît 89,5 % de l'entrepôt, n'invente ni obstacle
(0,39 %) ni code, place chaque code à 0,9 cm près et tient sa promesse de lisibilité à 95 %.
Elle suffit à guider le drone : 15 allers sur 15, aucun point dans un rack. Coût : 24 ms pour
la carte et 19 ms pour le décodage par observation, 1,5 Mo de mémoire.

## Limites : ce que ce test ne prouve pas
- La carte ne décide pas où aller : la patrouille suit un parcours fixe (c'est l'étape 5).
- Les 4 codes manquants sont les 4 cartons aux étiquettes de 12 cm, sur le rack du milieu,
  dans des allées accessibles. Trois fois plus petites, elles se lisent trois fois plus près
  (vers 1,2 m par règle de trois) ; la patrouille passait à plus de 2,5 m. La carte ne peut
  pas connaître la taille d'une étiquette avant de la lire : c'est à l'étape 5 de revenir
  plus près d'un carton vu de face sans code lu.
- Une face de rack sur six est inaccessible (contre le mur est, allée de 75 cm) ; ses codes
  ont été lus sur la face opposée.
- Mesuré avant trois changements de l'étape 7 (post-scriptum du 2026-09-07) :
  - les cartons non retenus gardaient leur volume physique : le lidar voyait des cartons
    fantômes dans les racks (corrigé à la source) ;
  - les racks n'ont aucune structure solide sur 0,70 m au sud et 0,77 m au nord de leur
    emprise : l'arbitre compte maintenant la structure, pas le rectangle du plan ;
  - le lidar a un anneau tous les 2 degrés, le planificateur une tranche de ±0,85 m
    (tranche portée ensuite à [−2 m ; +0,85 m] autour de l'altitude de vol à l'étape 5).

  La patrouille complète sera refaite avec la décision de l'étape 5.

## Fichiers de résultats et figures
- `../verification.json`, `../resultats.json` : les chiffres ; `../vol.json`, `../carte.npz`,
  `../carte.json` : vol et carte enregistrés.
- `../carte_qui_se_remplit.mp4` : la carte aller par aller ; `../carte_3d.html` : la carte en
  3D, vol à rejouer ; `../carte_3d.png`, `../comparaison.png` : images.
