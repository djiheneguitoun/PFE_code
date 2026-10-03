# 02 — Coût de DreamerV3 : résultats

29 août 2026 · `../dreamer_bench.py` · RTX 2060 SUPER 8 Go, torch 2.7 + CUDA 12.6.

## La question

Combien coûte DreamerV3 (temps, mémoire graphique) sur cette machine, et combien dure un
entraînement complet ? L'analyse initiale disait « 1 à 2 semaines » et craignait la mémoire.

## Méthode (en bref)

- Architecture DreamerV3 reconstruite : encodeur CNN 4 étages, RSSM (GRU + 32 × 32 variables
  catégorielles), décodeur, têtes récompense/fin, acteur, critique.
- Réglages officiels : lot 16 × 64, imagination sur 15 pas. 3 pas de chauffe, puis moyenne sur
  15 pas (5 avec `--quick`).

## Résultats

### A. Coût d'un pas d'entraînement

| Taille | Cam. | Lot | Précision | Param. | Mémoire pic | Temps/pas |
|---|---|---|---|---|---|---|
| 12M | 2 | 16×64 | fp16 | 8,9 M | 2,33 Go | 296 ms |
| 12M | 2 | 16×64 | fp32 | 8,9 M | 2,51 Go | 318 ms |
| 12M | 2 | 16×64 | bf16 | 8,9 M | 2,33 Go | 1506 ms |
| 12M | 1 | 16×64 | fp16 | 8,9 M | 2,18 Go | 281 ms |
| 25M | 2 | 16×64 | fp16 | 18,8 M | 3,38 Go | 335 ms |
| 50M | 2 | 16×64 | fp16 | 39,0 M | 5,36 Go | 486 ms |
| 12M | 2 | 32×64 | fp16 | 8,9 M | 4,45 Go | 404 ms |
| 12M | 2 | 8×64 | fp16 | 8,9 M | 1,26 Go | 264 ms |

- Piège matériel majeur : bf16 est 5 fois plus lent que fp16. La RTX 2060 SUPER (Turing,
  capacité 7.5) a des unités fp16 mais pas de bf16 natif, qu'elle émule. Or
  `torch.cuda.is_bf16_supported()` renvoie `True` et la plupart des dépôts DreamerV3 utilisent
  bf16 par défaut : il faut forcer fp16.
- La mémoire graphique n'est pas un problème : 2,33 Go sur 7,8 ; même le 50M passe (5,36 Go) ;
  on peut doubler le lot.

### B. Tampon de rejeu : la vraie contrainte

Images 64 × 64 RGB, 2 caméras, 3 drones, en `uint8` : 72 Ko par pas d'équipe.

| Pas stockés | Taille | Tient dans 19 Go libres ? |
|---|---|---|
| 100 000 | 6,9 Go | oui |
| 250 000 | 17,2 Go | oui, tout juste |
| 500 000 | 34,3 Go | non |
| 1 000 000 | 68,7 Go | non |

Disque plein à 90 % : plafond réaliste d'environ 250 000 pas. Pour le repousser, « par ordre
d'efficacité » : une seule caméra (×2), compression JPEG (×5 à ×10), images 48 × 48 (×1,8).

### C. Durée d'un entraînement complet

Le `train_ratio` dit combien de fois chaque pas collecté est rejoué. Un pas d'entraînement
consomme 16 × 64 = 1024 pas rejoués, donc pas d'environnement/s = 1024 ÷ (train_ratio × 0,296 s).

| `train_ratio` | Pas env/s | 1 M pas | 5 M pas |
|---|---|---|---|
| 32 | 108 | 2,6 h | 12,8 h |
| 64 | 54 | 5,1 h | 25,7 h |
| 128 | 27 | 10,3 h | 51,3 h |
| 256 | 14 | 20,5 h | 102,6 h |
| 512 (défaut DMC, banc d'essai DeepMind Control) | 7 | 41,1 h | 205,3 h |

## Conclusion

Ce que la mesure corrige :
- « 1 à 2 semaines par entraînement » était faux : avec fp16 et un `train_ratio` raisonnable, un
  million de pas prend 3 à 10 h ; au réglage le plus lourd (512), 41 h, moins de deux jours.
- La mémoire graphique n'est pas un risque (2,33 Go sur 7,8).
- Les deux arguments qui pesaient le plus contre Dreamer sont donc annulés par la mesure.

Ce qui reste vrai :
1. Le tampon de rejeu plafonne à ~250 000 pas faute de disque : contournable, mais du travail
   d'ingénierie à prévoir.
2. La mémoire récurrente ne tient pas l'épisode : le RSSM doit se souvenir des cartons lus sur
   ~1800 pas, sa portée utile est de quelques dizaines de pas (d'après la littérature, non
   mesuré). Une carte externe reste nécessaire, ce qui retire au modèle du monde une partie de
   son intérêt.
3. Le simulateur n'est pas mesuré ici. C'était la principale inconnue : à `train_ratio` 64,
   Isaac Sim doit produire 54 pas/s avec 6 caméras rendues → mesure 03 (`../../03_render_bench/`).

## Limites : ce que ce test ne prouve pas

- Seul l'apprentissage est chronométré ; le simulateur (Isaac Sim, 6 caméras) s'ajoute.

## Fichiers de résultats et figures

- `../resultats.csv` : une ligne par configuration (taille, caméras, lot, précision, millions de
  paramètres, mémoire pic en Go, secondes par pas, pas par seconde).
- `../resultats.json` : les mêmes lignes + le nom du GPU.
- Tableaux B et C : seulement affichés dans le terminal.
- Pas de figure.
