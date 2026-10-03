# 03 — Débit du simulateur avec caméras : résultats

29 août 2026 · `../render_bench.py` + `../run_all.sh` · Isaac Sim 5.1 / Isaac Lab 2.3.2 ·
RTX 2060 SUPER 8 Go · TiledCamera, images RGB, scène : sol, lumière, deux rangées de racks.

## La question

Dreamer alterne collecte et apprentissage, et la plus lente des deux impose son rythme. La
mesure 02 donne 296 ms par pas d'apprentissage pour 16 × 64 = 1024 pas rejoués. Avec un
`train_ratio` de 64, il faut 1024 ÷ 64 = 16 pas neufs par pas d'apprentissage, soit 54 pas neufs
par seconde. Le simulateur peut-il les fournir avec les caméras ?

## Méthode (en bref)

N copies de la scène (« environnements »), 1 ou 2 caméras chacune, images de 64 ou 128 px ; pas
physique de 1/120 s avec rendu à chaque pas ; 20 pas de chauffe puis 60 pas chronométrés ; un
processus Isaac par configuration.

## Résultats

| Env. | Résolution | Caméras | Pas env/s | Images/s |
|---|---|---|---|---|
| 1 | 64 px | 1 | 54,2 | 54,2 |
| 1 | 64 px | 2 | 34,0 | 68,0 |
| 3 (l'essaim) | 64 px | 2 | 93,4 | 186,8 |
| 8 | 64 px | 2 | 245,6 | 491,3 |
| 16 | 64 px | 2 | 409,8 | 819,7 |
| 32 | 64 px | 2 | 736,1 | 1472,2 |
| 8 | 128 px | 2 | 201,4 | 402,8 |

Toutes les images sont valides (100 % de pixels non noirs, valeur moyenne ≈ 197).

1. Le simulateur n'est pas le goulot, de très loin : 736 pas/s à 32 environnements pour 54
   nécessaires (marge ×13). Même 3 environnements (un par drone) donnent 93 pas/s, 1,7 fois le
   besoin.
2. Le rendu par tuiles monte presque linéairement : 31 pas/s par environnement à 3 et à 8, 26 à
   16, 23 à 32. Légère baisse dès 16, normale ; aucun effondrement.
3. La résolution coûte peu : 64 → 128 px (4 fois plus de pixels) = −18 % (246 → 201 pas/s). Le
   coût vient surtout du nombre d'appels de rendu, pas de la surface : mieux voir est presque
   gratuit.
4. On peut baisser le `train_ratio` : avec 8 environnements (246 pas/s), un `train_ratio` de 16
   est soutenable (il en faudrait 216). Cela ramène un entraînement d'un million de pas à
   environ 2,6 h.

## Conclusion

Le débit du simulateur ne bloque pas Dreamer : l'objection « le simulateur ne suivra pas »
tombe (voir le bilan dans `../../A_LIRE_POUR_LE_PROMOTEUR/README.md`).

## Limites : ce que ce test ne prouve pas

- Colonne mémoire (`vram_gb`) inutilisable (0,01 Go partout) : la mémoire de TiledCamera est
  allouée hors de PyTorch, que lit `torch.cuda.max_memory_allocated()`. Il faudrait `nvidia-smi`.
  À corriger si la question devient utile.
- Scène simple (un sol, une lumière, deux blocs, aucun drone) : un vrai entrepôt avec 123
  cartons texturés et un éclairage réaliste sera plus lent. Ces chiffres sont un plafond. La
  contribution finale mesure son propre débit, scène complète, dans
  `swarm_qr/experiments/04_debit/`.
- Forte variance : la même configuration (1 env., 1 caméra) a donné 86 puis 54 pas/s à deux
  lancements. Ordres de grandeur, pas mesures fines.
- Blocage passager au tout premier essai (8 env. × 2 caméras, bloqué 28 min), non reproduit,
  cause inconnue. D'où `timeout -s KILL`, obligatoire : Isaac ignore le signal d'arrêt normal.
- L'avertissement de pilote (535.32 contre 535.129 requis) est sans effet : les images sont
  correctes ; `verifyDriverVersion` sert seulement à le faire taire.

## Fichiers de résultats et figures

- `../resultats.csv` : par configuration, environnements, résolution, caméras, pas/s, images/s,
  `vram_gb` (inutilisable) et `render_ok` (part de pixels non noirs, 1.0 = 100 %).
- Pas de figure.
