# swarm_qr — résultats finaux : le système face à la méthode de Pore et al.

## La question

Sur un même simulateur et dans les mêmes conditions, le système lit-il l'inventaire mieux et
plus vite qu'une méthode publiée récente, et résiste-t-il mieux aux imprévus (panne d'un
drone, entrepôt jamais vu, obstacle qui apparaît) ?

La méthode de référence est celle de Pore, Patle et Thorat (Symmetry 2026, 18(4):548),
réécrite dans `../pore.py` et `../pore_mission.py`. Ses drones suivent un **plan en zigzag
calculé avant le vol** (un rack par drone, des arrêts tous les 1,5 m à chaque étage). Le
système, lui, **décide en vol** à partir de la carte partagée.

## Méthode (en bref)

- 4 cas, un vol par cas et par méthode, 3 drones, au plus 600 s simulées :
  - **nominal** : entrepôt 9033, 114 codes à lire ;
  - **panne** : le même, et le drone 1 s'arrête à 200 s ;
  - **entrepôt 9019** : 65 codes, jamais utilisé pendant la mise au point ;
  - **obstacle** : entrepôt 9033, et un bloc de 1 × 1 × 2 m apparaît à 200 s dans un couloir,
    en (−4,96 ; 4,0).
- Mêmes entrepôts, mêmes drones, même contrôleur de vol, mêmes durées maximales. Chaque
  méthode garde ses propres capteurs : la référence lit avec une caméra frontale et le décodeur
  d'OpenCV, comme dans l'article ; le système avec ses deux caméras latérales, le décodeur
  zxing et le détecteur YOLO.
- Règles d'arrêt : 95 % des codes lus suivis de 60 s de grâce, 120 s sans code nouveau, ou
  plan terminé (référence).
- Système : détecteur YOLO branché, guide vision-langage débranché (λ = 0).
- Le jugement est fait après coup par un arbitre qui connaît la vérité (position et contenu
  de chaque QR) : `../experiments/11_mission/analyse.py` et `../experiments/13_pore/analyse.py`.
- Les temps sont des temps simulés comptés depuis le lancement, décollage compris.

## Résultats

### Lecture de l'inventaire

| Cas | Système : codes lus | Référence : codes lus |
|---|---|---|
| Nominal | **110 / 114** (96,5 %) | 106 / 114 (93,0 %) |
| Panne d'un drone | **110 / 114** (96,5 %) | 75 / 114 (65,8 %) |
| Entrepôt 9019 | **60 / 65** (92,3 %) | 53 / 65 (81,5 %) |
| Obstacle | **110 / 114** (96,5 %) | 103 / 114 (90,4 %) |

Aucun code inventé, dans aucun vol, par aucune des deux méthodes.

### Vitesse : temps pour lire 50 %, 80 % et 90 % des codes

| Cas | Système | Référence |
|---|---|---|
| Nominal | 154 / 180 / **196 s** | 250 / 354 / 515 s |
| Panne d'un drone | 131 / 154 / **187 s** | 269 s / jamais / jamais |
| Entrepôt 9019 | 124 / 144 / **165 s** | 260 / 304 s / jamais |
| Obstacle | 135 / 192 / **206 s** | 250 / 354 / 534 s |

Le système atteint 90 % de l'inventaire environ 2,6 fois plus vite dans les deux cas où la
référence l'atteint ; la référence ne l'atteint pas du tout en cas de panne ni sur
l'entrepôt 9019.

### Sécurité

| Cas | Points de trajectoire dans un rack (système / réf.) | Distance minimale entre drones (système / réf.) |
|---|---|---|
| Nominal | 0 / 4 215 · 0 / 6 981 | 2,08 m · 4,61 m |
| Panne d'un drone | 0 / 2 873 · 0 / 3 935 | 3,82 m · 4,63 m |
| Entrepôt 9019 | 0 / 3 195 · 0 / 6 612 | 2,53 m · 2,49 m |
| Obstacle | 0 / 3 840 · 0 / 7 218 | 1,60 m · 4,62 m |

- Aucune chute, aucun passage dans un rack, pour les deux méthodes.
- Obstacle : le système le contourne et ne passe jamais à moins de 30 cm (au plus près
  0,57 m). La référence passe à 0,28 m au plus près à hauteur du bloc, et passe aussi
  au-dessus (75 points de trajectoire au-dessus du bloc, à 1,04 m au moins) ; son champ de
  risque l'a fait dévier 6 fois.

### Fin de mission et coût de calcul

| Cas | Fin du système | Fin de la référence | Calcul (système / réf.) |
|---|---|---|---|
| Nominal | 368 s, plus de code nouveau | 552 s, plan terminé | 33 / 48 min |
| Panne d'un drone | 317 s, plus de code nouveau | 423 s, plus de code nouveau | 26 / 33 min |
| Entrepôt 9019 | 300 s, plus de code nouveau | 527 s, plus de code nouveau | 25 / 44 min |
| Obstacle | 343 s, plus de code nouveau | 568 s, plan terminé | 30 / 50 min |

## Pourquoi ces écarts

- **Panne** : le plan de la référence est figé. Les arrêts du drone en panne ne sont repris
  par personne : 104 arrêts servis sur 180, 74 jamais servis, d'où 75 codes seulement. Dans le
  système, les réservations du drone en panne expirent seules : les deux autres drones
  continuent (24 décisions après la panne) et visitent sa zone dès 281,6 s.
- **Entrepôt 9019** : le plan de la référence sert 139 arrêts sur 144 mais ne lit que 53 codes
  sur 65. Le système en lit 60.
- **Nominal et obstacle** : la référence finit par passer partout (180 arrêts sur 180), mais
  elle parcourt tout son plan dans l'ordre, alors que le système va d'abord vers les QR que sa
  carte a repérés.

## Conclusion

Dans les quatre cas, le système lit plus de codes que la référence (110 contre 106, 110 contre
75, 60 contre 53, 110 contre 103) et atteint 90 % de l'inventaire en environ 3 minutes, contre
plus de 8 minutes ou jamais pour la référence. Il le fait sans aucune chute ni contact, et
tient mieux à distance l'obstacle imprévu.

## Limites : ce que ces résultats ne prouvent pas

- Simulation uniquement.
- La méthode de référence est notre réécriture d'après l'article, sur notre simulateur ;
  ses chiffres ne sont pas ceux publiés par ses auteurs.
- La référence garde ses drones plus loin les uns des autres (2,49 à 4,63 m, contre 1,60 à
  3,82 m pour le système) : à ce titre, elle est plus prudente.

## Fichiers de résultats et figures

- Le tableau d'origine : `../experiments/13_pore/comparaison.md`.
- Les bilans chiffrés : `../experiments/11_mission/tests of system/eval_*/resultats.json`
  (système) et `../experiments/13_pore/pore_*/resultats.json` (référence).
- Les courbes de lecture : `codes_dans_le_temps.png` dans chacun de ces dossiers ; les cartes
  finales : `carte_finale.png` (système) et `plan_et_vol.png` (référence).
- Les vidéos : `video/mission.mp4` dans chacun de ces dossiers.
- Le rejeu 3D des 8 vols : `../experiments/14_rejeu/rejeu_hors_ligne.html`.
- Les résultats de chaque étape de construction : `../experiments/A_LIRE_POUR_LE_PROMOTEUR/README.md`.
