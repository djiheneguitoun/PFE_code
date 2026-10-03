# 13 — Le système face à la méthode de Pore et al. : résultats

## La question
Dans les mêmes conditions, le système lit-il l'inventaire mieux et plus vite qu'une méthode
publiée récente, et résiste-t-il mieux aux imprévus : panne d'un drone, entrepôt jamais vu,
obstacle qui apparaît ?

## Les deux méthodes
| | Référence (Pore et al., réécrite) | Système |
|---|---|---|
| Au départ, le drone connaît | le plan de l'entrepôt | rien : il découvre l'entrepôt |
| Où aller | plan figé, calculé avant le vol : un rack par drone, un arrêt tous les 1,5 m à chaque étage, à 1,65 m de la face, 2 s par arrêt | décidé en vol sur la carte partagée : lire un QR repéré, couvrir une surface pas vue de près, explorer une frontière |
| Si un drone tombe en panne | ses arrêts restent à faire : personne ne les reprend | ses réservations expirent, les autres reprennent sa zone |
| Si un obstacle apparaît | un champ de risque tiré du lidar décale l'arrêt (1,2 m au plus) ou le remet à un 2e passage | l'obstacle entre dans la carte, le chemin est recalculé |
| Lecture | 1 caméra frontale, décodeur OpenCV après seuillage | 2 caméras latérales, décodeur zxing, détecteur YOLO |

L'article parle de « hover-and-scan stops » (arrêts où le drone plane et scanne) et d'un
« sector allocator » qui donne à chaque drone un rack à lui. Sa caméra frontale a ici les mêmes
réglages optiques que les caméras latérales du système.

## Méthode (en bref)
- 4 cas, un vol par cas et par méthode, 3 drones, 600 s simulées au plus :
  - **nominal** : entrepôt 9033, 114 codes ;
  - **panne** : le même ; le drone 1 s'arrête à 200,2 s ;
  - **9019** : un autre entrepôt, 65 codes, jamais utilisé pendant la mise au point ;
  - **obstacle** : entrepôt 9033 ; un bloc de 1 × 1 × 2 m (x de −5,46 à −4,46 m, y de 3,5 à
    4,5 m) apparaît à 200,2 s.
- Même règle d'arrêt : 120 s sans code nouveau, ou plan terminé (référence).
- Système : détecteur YOLO branché.
- Juges : `../analyse.py` (référence) et `../../11_mission/analyse.py` (système). Mêmes
  mesures : points dans la structure d'un rack (bouts vides exclus), distance entre drones au
  même instant.
- Les temps sont des secondes simulées depuis le lancement ; la mission commence vers 86 s,
  après le décollage.

## Résultats

### Lecture et vitesse
| Cas | Codes lus : système | Codes lus : référence | 50 / 80 / 90 % : système | 50 / 80 / 90 % : référence |
|---|---|---|---|---|
| Nominal | **110 / 114** (96,5 %) | 106 / 114 (93,0 %) | 154 / 180 / **196 s** | 250 / 354 / 515 s |
| Panne | **110 / 114** (96,5 %) | 75 / 114 (65,8 %) | 131 / 154 / **187 s** | 269 s / jamais / jamais |
| 9019 | **60 / 65** (92,3 %) | 53 / 65 (81,5 %) | 124 / 144 / **165 s** | 260 / 304 s / jamais |
| Obstacle | **110 / 114** (96,5 %) | 103 / 114 (90,4 %) | 135 / 192 / **206 s** | 250 / 354 / 534 s |

- Aucun code inventé, dans aucun vol.
- Quand la référence atteint 90 %, le système y arrive environ 2,6 fois plus vite.

### Fin de mission et coût de calcul
| Cas | Système : dernier code / fin | Référence : dernier code / fin | Calcul (système / réf.) |
|---|---|---|---|
| Nominal | 247 s / 368 s, 120 s sans code | 530 s / 552 s, plan terminé | 33 / 48 min |
| Panne | 197 s / 317 s, 120 s sans code | 303 s / 423 s, 120 s sans code | 26 / 33 min |
| 9019 | 179 s / 300 s, 120 s sans code | — / 527 s, 120 s sans code | 25 / 44 min |
| Obstacle | 222 s / 343 s, 120 s sans code | 534 s / 568 s, plan terminé | 30 / 50 min |

Un cycle de 0,2 s simulée coûte un peu moins à la référence (1,16 à 1,23 s de calcul en
médiane) qu'au système (1,34 à 1,36 s). Mais ses vols durent plus longtemps.

### Le plan de la référence
| Cas | Arrêts prévus | Servis | Non servis | Passages | Décalages d'arrêt |
|---|---|---|---|---|---|
| Nominal | 180 | 180 | 0 | 1 | 3 |
| Panne | 180 | 104 | 74 | 1 | 2 |
| 9019 | 144 | 139 | 4 | 1 | — |
| Obstacle | 180 | 180 | 0 | 2 | 6 |

- Entrepôt 9033 : le drone 0 a les 2 faces du rack du milieu (72 arrêts), le drone 1 les 2 faces
  du rack ouest (72), le drone 2 la seule face accessible du rack est (36).
- Inventaire : 1 860, 903, 990 et 1 951 lectures ; 2 116, 1 059, 1 120 et 2 230 doublons écartés ;
  0 arrêt à relire.

### Sécurité
| Cas | Points dans un rack (sys. / réf.) | Distance min. entre drones (sys. / réf.) | Attentes de priorité (sys. / réf.) |
|---|---|---|---|
| Nominal | 0 / 4 215 · 0 / 6 981 | 2,08 m · 4,61 m | 2 · 0 |
| Panne | 0 / 2 873 · 0 / 3 935 | 3,82 m · 4,63 m | 0 · 0 |
| 9019 | 0 / 3 195 · 0 / 6 612 | 2,53 m · 2,49 m | 0 · 0 |
| Obstacle | 0 / 3 840 · 0 / 7 218 | 1,60 m · 4,62 m | 10 · 0 |

- Aucune chute, pour les deux méthodes. La référence tient partout les 1,2 m de séparation
  qu'elle exige.
- Cibles abandonnées par le système (chemin coupé, drone immobile, trop de recalculs) : 22, 0,
  3 et 4. Aucun abandon d'arrêt pour la référence (nominal, panne, obstacle).
  Dans le vol nominal du système, 16 des 22 abandons ont lieu après le dernier code nouveau.

### L'obstacle
- **Système** : 0 point dans le bloc, 0 à moins de 30 cm, au plus près 0,57 m. Quatre cibles
  abandonnées pour « chemin coupé » (à 211, 220, 224 et 325 s).
- **Référence** : 0 point dans le bloc, au plus près 0,28 m. Son champ de risque a décalé
  6 arrêts ; 1 arrêt a été refait au 2e passage (drone 1, à 551 s).

### La panne
- **Système** : drone 1 arrêté à 200,2 s. Ensuite, les deux autres drones prennent
  24 décisions et visitent 3 fois la zone visée par le drone en panne, la première fois à
  281,6 s. Vérifications du juge : 4 sur 5.
- **Référence** : le drone 1 (rack ouest) s'arrête après 17 de ses 72 arrêts ; 54 ne seront
  jamais servis. La mission s'arrête à 423 s (120 s sans code nouveau). Des 39 codes non lus,
  29 sont sur le rack du drone en panne.

## Conclusion
- Dans les 4 cas, le système lit plus de codes que la référence (110 contre 106, 110 contre 75,
  60 contre 53, 110 contre 103). Il atteint 90 % de l'inventaire en 165 à 206 s, contre 515 à
  534 s ou jamais pour la référence.
- L'écart le plus net est la panne. Le plan figé perd le secteur du drone en panne. Dans le
  système, les réservations expirent seules et les autres drones reprennent sa zone.
- Les deux méthodes volent sans chute ni contact et évitent l'obstacle. La référence, plus
  prudente, garde ses drones plus loin les uns des autres.

## Limites : ce que ces résultats ne prouvent pas
- Simulation uniquement.
- La référence est notre réécriture de l'article, sur notre simulateur : ses chiffres ne sont pas
  ceux de ses auteurs.

## Fichiers de résultats et figures
- `../comparaison.md` : le tableau d'origine.
- `../pore_*/resultats.json` et `../pore_*/mission.json` (référence) ;
  `../../11_mission/tests of system/eval_*/resultats.json` et `mission.json` (système).
- `../pore_*/codes_dans_le_temps.png` : les codes lus au fil du temps, panne ou obstacle en
  pointillé.
- `../pore_*/plan_et_vol.png` : le plan connu, les arrêts prévus, le vol réel, les codes non lus
  (croix rouges).
- `../pore_*/video/mission.mp4` : le film de chaque vol ; `lecteur.mp4` (caméra du drone qui
  lit), `sud_central.mp4`, `sud_grande.mp4` (caméras fixes).
- `../../14_rejeu/rejeu_hors_ligne.html` : les 8 vols rejoués en 3D, côte à côte.
