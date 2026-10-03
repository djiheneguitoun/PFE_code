# 01 — Résolution nécessaire pour lire un QR (bancs préliminaires)

## En une phrase

À partir de combien de pixels un QR du projet devient-il lisible, et un réseau qui ne voit que
des images de 64 × 64 pixels (cas d'un « modèle du monde » comme DreamerV3) pourrait-il lire les
QR lui-même ?

## La tâche

- Le script génère le même QR que le projet (même appel que `scripts/qr_code_system.py`) et le
  pose sur un fond couleur carton.
- Il le réduit à la taille voulue avec les défauts d'une caméra : flou, bruit, décalage de moins
  d'un pixel, petite rotation (±3°). 30 essais par taille.
- Il essaie ensuite de le repérer (trouver le motif du QR dans l'image) puis de le décoder (lire
  son contenu), avec les détecteurs d'OpenCV et, s'il est installé, pyzbar.
- La taille est comptée en pixels par module (un module = un des petits carrés noirs ou blancs
  du QR).
- Avec une calibration faite auparavant sous Isaac Sim, il en déduit la taille réelle du QR, puis
  la distance de lecture selon la largeur de l'image (caméra de champ horizontal 60°).
- Sans simulateur ni GPU : processeur seul.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../qr_resolution_test.py` | le banc : génère, dégrade et lit les QR ; imprime les parties A à D |
| `../resultats_seuil.csv` | taux de décodage et de repérage pour 15 tailles de QR (en pixels) |
| `../resultats_portee.csv` | les mêmes taux pour 12 distances × 6 largeurs d'image |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer

Prérequis : Python 3 avec `opencv-python`, `numpy` et `qrcode` ; `pyzbar` en option (utilisé
seulement s'il est installé). Ni simulateur ni GPU. Durée : ~10 min.

Depuis la racine du projet (sur la machine de simulation : `cd ~/simulation_mc02`) :

    python3 experiments/01_resolution_qr/qr_resolution_test.py

Le script peut être lancé de n'importe où : il écrit toujours à côté de lui. Il imprime quatre
tableaux (A : seuil de lisibilité ; B : taille du QR ; C : portée selon la résolution ;
D : conclusion) et écrit `../resultats_seuil.csv` et `../resultats_portee.csv`.

## Résultats en bref

- Le QR du projet est en réalité de version 2 (29 modules de large, marge comprise), et non de
  version 1 comme demandé dans le code.
- Décodage fiable (≥ 90 %) à partir de 2,07 pixels par module ; repérage fiable dès 1,38.
- Taille du QR déduite : environ 7,3 cm de côté.
- Portée de décodage : 7,5 cm avec une image de 64 px, 1,50 m avec 1280 px.
- Pour décoder à 1,25 m (champ de 60°), il faut une image d'environ 1000 px de large : un
  décodeur externe en pleine résolution est obligatoire.

→ détails dans `RESULTATS.md`.
