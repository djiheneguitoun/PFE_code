# 02 — Coût de DreamerV3 sur la machine du projet (bancs préliminaires)

## En une phrase

Combien de temps et de mémoire graphique coûte l'entraînement de DreamerV3 sur la carte
graphique du projet (RTX 2060 SUPER, 8 Go), et combien durerait un entraînement complet ?

## La tâche

- DreamerV3 (Hafner et al., 2023) apprend un « modèle du monde » (réseau qui prédit la suite
  des images), puis entraîne un acteur (qui choisit les actions) et un critique (qui note les
  situations) dans des trajectoires imaginées par ce modèle.
- Le script reconstruit son architecture : encodeur d'images (réseau convolutif à 4 étages),
  RSSM (mémoire récurrente GRU + 32 variables à 32 valeurs), décodeur, têtes récompense et fin
  d'épisode, acteur, critique.
- Il chronomètre un pas d'entraînement complet (modèle du monde, imagination sur 15 pas, acteur
  et critique) sur un lot de 16 séquences × 64 pas d'images aléatoires, et relève le pic de
  mémoire graphique.
- 8 configurations : 3 tailles (12M, 25M, 50M), 1 ou 2 caméras, 3 précisions de calcul (fp16,
  fp32, bf16), 3 tailles de lot.
- Il en déduit la place du tampon de rejeu (images gardées pour réapprendre) et la durée d'un
  entraînement. Sans simulateur.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../dreamer_bench.py` | le banc : modèle DreamerV3 + chronométrage + calculs (tableaux A, B, C) |
| `../resultats.csv` | temps et mémoire des 8 configurations |
| `../resultats.json` | les mêmes chiffres + le nom du GPU |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer

Prérequis : GPU NVIDIA avec CUDA et PyTorch (on prend le Python de l'environnement Isaac Sim,
qui contient PyTorch). Pas de simulateur. Durée : ~3 min.

Depuis la racine du projet (sur la machine de simulation : `cd ~/simulation_mc02`), avec
`PY=~/isaac5_env/bin/python` :

    $PY experiments/02_dreamer_bench/dreamer_bench.py
    # essai rapide : 5 itérations au lieu de 15, chiffres plus bruités
    $PY experiments/02_dreamer_bench/dreamer_bench.py --quick

Le script imprime les tableaux A (coût d'un pas), B (tampon de rejeu) et C (durée d'un
entraînement), puis écrit `../resultats.csv` et `../resultats.json`.

## Résultats en bref

- Réglage de référence (12M, 2 caméras, fp16, lot 16 × 64) : 296 ms par pas, 2,33 Go de mémoire
  graphique sur 7,8.
- bf16 est 5 fois plus lent que fp16 sur cette carte (1506 ms) : il faut forcer fp16.
- Un million de pas d'entraînement : de 2,6 h à 41 h selon le `train_ratio`, et non 1 à 2
  semaines.
- La vraie contrainte est le disque : le tampon de rejeu plafonne vers 250 000 pas (17,2 Go).

→ détails dans `RESULTATS.md`.
