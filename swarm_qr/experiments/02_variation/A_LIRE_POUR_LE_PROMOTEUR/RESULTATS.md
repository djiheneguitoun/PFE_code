# 02 — Variation entre entrepôts : résultats

## La question
Les entrepôts sont-ils vraiment différents entre eux ? Si tous se ressemblent, dire que le
système généralise ne veut rien dire, puisqu'il aura toujours vu la même chose.

## Méthode (en bref)
- Six entrepôts (graines 1 à 6), photographiés d'en haut.
- Pour chacune des 15 paires d'entrepôts : déplacement moyen des trois racks (moyenne des
  écarts en largeur X et en profondeur Y).
- On juge sur le pire cas, la paire la plus proche : elle doit différer de plus de 1 m. Le
  nombre de cartons doit aussi varier de plus de 20.

## Résultats
| Mesure | Valeur |
|---|---|
| Paire la plus proche | graines 1 et 6 : 1,85 m |
| Écart-type en largeur (X) | 1,73 m |
| Écart-type en profondeur (Y) | 3,27 m |
| Nombre de cartons | de 62 à 118 |

Détail par graine (position X des trois racks ; chaque carton porte 2 QR) :

| Graine | Racks X (m) | Cartons |
|---|---|---|
| 1 | −7,8 / 1,6 / 6,7 | 102 |
| 2 | −8,4 / 3,1 / 7,6 | 103 |
| 3 | −6,9 / −1,5 / 4,3 | 62 |
| 4 | −8,0 / −2,6 / 3,1 | 118 |
| 5 | −3,9 / 1,5 / 6,3 | 63 |
| 6 | −5,0 / 1,9 / 6,5 | 90 |

Verdict : **la variation est suffisante.**

## Conclusion
Même les deux entrepôts qui se ressemblent le plus ont leurs racks déplacés de 1,85 m en
moyenne, et le nombre de cartons change beaucoup. Les drones exploreront de vraies formes
nouvelles.

Point de méthode important : on juge chaque entrepôt dans son ensemble, et non rack par
rack. Avec six entrepôts et trois racks, cela fait 45 comparaisons de racks (15 paires × 3) :
que deux racks tombent au même endroit par hasard est attendu et sans conséquence.

## Fichiers de résultats et figures
- `../resultat.json` : paire la plus proche (graines 1 et 6, 1,845 m), écarts-types en X et
  Y, nombre de cartons minimal et maximal, verdict vrai.
- `../layout_1.json` … `../layout_6.json` : position des racks, nombre de cartons et de QR.
- `../planche_variation.jpg` : les six entrepôts vus d'en haut ; chaque vue porte la graine,
  la position X des racks et le nombre de cartons.
- `../vue_1.jpg` … `../vue_6.jpg` : les six vues séparées.
