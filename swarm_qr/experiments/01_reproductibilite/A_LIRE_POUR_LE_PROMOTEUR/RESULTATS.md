# 01 — Reproductibilité de l'entrepôt : résultats

## La question
Si l'on demande deux fois le même entrepôt, obtient-on exactement le même ? C'est
indispensable, parce qu'on voudra plus tard comparer deux versions du système sur le même
terrain.

## Méthode (en bref)
- L'entrepôt numéro 7 est construit deux fois, dans deux programmes séparés, puis
  photographié d'en haut (après 40 pas de simulation).
- On compare d'abord la description écrite (fichier JSON : racks, remplissage, nombre de
  cartons et de QR, points de départ des drones).
- On compare ensuite les deux images :
  - l'écart moyen des pixels, sur une échelle de 0 à 255 ;
  - l'« écart structurel » : la part des pixels qui changent de plus de 40 sur 255, signe
    qu'un objet a bougé. Seuil toléré : 0,1 %.
- Verdict « reproductible » si la description est identique ET l'écart structurel reste
  sous 0,1 %.

## Résultats
| Mesure | Valeur |
|---|---|
| Description (JSON) | identique octet par octet |
| Écart structurel | 0,000 % (seuil 0,1 %) |
| Écart moyen des pixels | 1,50 sur 255 |
| Cartons / QR | 65 / 130, dans les deux passages |

Verdict : **l'entrepôt est reproductible.**

## Conclusion
La même graine redonne les mêmes racks, le même nombre de cartons et de QR, et les mêmes
points de départ des drones. Aucun objet n'a bougé de façon visible.

Point de méthode important : la preuve est la description écrite, pas l'image. Le moteur
graphique dessine avec une part de hasard : deux images du même entrepôt ne sont jamais
identiques au pixel près (ici 1,50/255 d'écart moyen, du bruit de rendu et de
stabilisation physique). Les comparer directement mènerait à une fausse conclusion.
L'image sert uniquement à confirmer que rien n'a bougé.

## Fichiers de résultats et figures
- `../resultat.json` : description identique (`same_layout` vrai), écart moyen 1,50,
  écart structurel 0,0, verdict vrai.
- `../layout_A.json`, `../layout_B.json` : les deux descriptions (identiques).
- `../comparaison.jpg` : à gauche le passage A, au milieu le passage B, à droite la carte des
  différences (sombre = pas de différence) avec les deux écarts écrits en haut.
- `../vue_A.jpg`, `../vue_B.jpg` : les deux vues de dessus.
- `../vue_A.ancien.jpg`, `../vue_B.ancien.jpg` : anciennes vues de dessus.
