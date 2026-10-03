# 01 — Résolution nécessaire pour lire un QR : résultats

29 août 2026 · `../qr_resolution_test.py` · processeur seul, aucun simulateur.

## La question

Un réseau qui encode l'image en 64 × 64 px (entrée d'un « modèle du monde ») voit-il un QR ?
Sinon, quelle résolution faut-il ?

## Méthode (en bref)

- QR du projet (correction H, marge 2 modules ; module = petit carré du QR), rendu de face sur
  fond carton, caméra de champ 60°. Défauts : décalage sous-pixel, rotation ±3°, flou 0,6 px,
  bruit σ = 2.
- 30 essais par point (12 pour la portée). Décodeurs : OpenCV + OpenCV-Aruco + pyzbar.
- « Repéré » = motif trouvé ; « décodé » = contenu lu.

## Résultats

Le code demande `version=1`, mais `qr.make(fit=True)` relève la version : avec `Item#A14-R5` et
la correction H, c'est une version 2, soit 29 modules de large marge comprise (25 + 2 × 2).
16 % de finesse en plus : à corriger dans toute analyse.

| Seuil (pixels par module) | Valeur |
|---|---|
| Décodage fiable (≥ 90 %) | 2,07 |
| Décodage possible (≥ 50 %) | 1,72 |
| Repérage fiable (≥ 90 %) | 1,38 |
| Repérage possible (≥ 50 %) | 1,21 |

- Repérer est 1,42 (à 50 %) à 1,50 fois (à 90 %) plus facile que décoder, en distance. L'étude
  préalable (fiche D3) annonçait 1,75 à 3,5 d'après une source qui ne mesurait
  pas le repérage : le facteur est environ deux fois plus petit.
- Taille du QR : la calibration du projet (`docs/calibration_gate.csv`, vrai rendu Isaac
  1280 × 960, champ 60°) décode à 1,50 m et échoue à 1,75 m. Avec le seuil à 50 %, le QR
  mesure 6,7 à 7,9 cm : on retient 7,3 cm (et non les 10 cm supposés au départ).

| Largeur d'image | Portée de décodage (≥ 50 %) |
|---|---|
| 64 px (modèle du monde) | 7,5 cm |
| 96 / 128 / 256 px | 10 / 15 / 25 cm |
| 512 px | 60 cm |
| 1280 px | 1,50 m |

Validation croisée : le modèle prédit 1,50 m à 1280 px, et la calibration réelle sous Isaac Sim
mesure exactement 1,50 m : le test synthétique reproduit donc la mesure réelle.

## Conclusion

- Décoder à 1,25 m demande une image d'environ 985 px de large : un modèle du monde en 64 × 64
  est 15 fois en dessous (portée 7,5 cm).
- La condition 1 de l'architecture D1 de l'étude (« pixels purs ») est confirmée : un décodeur
  externe en pleine résolution est obligatoire, quelle que soit la direction retenue.
- Contrainte matérielle à figer en premier : au moins 1000 px de large à 60°. (La contribution
  finale utilise 1024 × 768 px à 60° : `swarm_qr/env/config.py`.)

## Limites : ce que ce test ne prouve pas

- Rendu synthétique et frontal (ni perspective forte, ni flou de bougé, ni éclairage variable,
  ni occlusion) : chiffres optimistes, portées réelles plus courtes.
- Repérage mesuré avec OpenCV ; un détecteur appris (type YOLO) ferait sans doute mieux et
  pourrait relever le facteur 1,4. À mesurer si la question devient décisive.
- Repérage bruité aux très petites tailles (faux positifs à 0,69 et 0,86 px/module) : seuls les
  seuils à 90 % sont solides.

## Fichiers de résultats et figures

- `../resultats_seuil.csv` : 15 tailles (12 à 220 px) → px/module, taux de décodage et de repérage.
- `../resultats_portee.csv` : 12 distances (5 cm à 3 m) × 6 largeurs (64 à 1280 px) → mêmes taux.
- Pas de figure.
