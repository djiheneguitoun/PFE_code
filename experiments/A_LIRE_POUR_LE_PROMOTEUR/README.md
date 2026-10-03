# experiments — Bancs préliminaires (avant la contribution finale)

## En une phrase

Trois mesures rapides, faites le 29 août 2026, pour savoir si un apprentissage par « modèle du
monde » (DreamerV3) était réaliste sur la machine du projet : quelle résolution faut-il pour lire
un QR, combien coûte DreamerV3, et le simulateur suit-il avec les caméras allumées ?

## La tâche

- Contexte : après l'inférence active (`scripts/aif_core`) et pendant l'essai d'apprentissage par
  renforcement (`rl_inventory/`, cerveaux prévus : PPO, Dreamer, Pore), il fallait savoir si
  DreamerV3 était faisable ici.
- DreamerV3 : méthode d'apprentissage par renforcement qui apprend d'abord un « modèle du
  monde » (un réseau qui prédit la suite des images à partir de petites images de 64 × 64 px),
  puis entraîne le « cerveau » qui choisit les actions dans des trajectoires imaginées par ce
  modèle.
- Règle suivie : aucune mesure ne dépend d'Isaac Sim tant que ce n'est pas indispensable. Chaque
  banc tourne en quelques minutes et peut être relancé à volonté.
- Ne pas confondre avec `swarm_qr/experiments/` : les expériences de la contribution finale.

## L'index

| N° | Dossier | Question | Réponse courte |
|---|---|---|---|
| 01 | `../01_resolution_qr/` | Quelle résolution faut-il pour lire un QR ? | Décodage fiable à 2,07 px par module (petit carré du QR). En image de 64 px, la portée tombe à 7,5 cm. Il faut environ 1000 px de large pour lire à 1,25 m. |
| 02 | `../02_dreamer_bench/` | Combien coûte DreamerV3 sur cette machine ? | 296 ms par pas d'entraînement, 2,33 Go de mémoire graphique. Un entraînement dure 3 à 40 h, pas 2 semaines. fp16 obligatoire : bf16 est 5 fois plus lent. |
| 03 | `../03_render_bench/` | Le simulateur suit-il, caméras allumées ? | Oui, largement : 736 pas/s à 32 environnements pour 54 nécessaires (marge ×13). 4 fois plus de pixels ne coûtent que 18 % de débit. |

Détails de chaque banc : `README.md` (la tâche, les commandes) et `RESULTATS.md` (les chiffres et
les limites) dans son dossier `A_LIRE_POUR_LE_PROMOTEUR/`.

## Contenu du dossier

| Fichier / dossier | Rôle |
|---|---|
| `../01_resolution_qr/` | banc 01 : script, 2 CSV de résultats |
| `../02_dreamer_bench/` | banc 02 : script, résultats CSV et JSON |
| `../03_render_bench/` | banc 03 : script, lanceur `run_all.sh`, résultats CSV |
| `README.md` | ce fichier |

## Comment le lancer

Prérequis : 01 = Python 3 seul (processeur) ; 02 = GPU NVIDIA + PyTorch ; 03 = Isaac Sim 5.1 +
Isaac Lab (pas besoin de Pegasus ni d'ArduPilot). Durées : ~10 min, ~3 min, ~7 min.

Depuis la racine du projet (sur la machine de simulation : `cd ~/simulation_mc02`), `PY` étant le
Python de l'environnement Isaac Sim (`PY=~/isaac5_env/bin/python`) :

    # 01 — résolution et lisibilité des QR (processeur seul)
    python3 experiments/01_resolution_qr/qr_resolution_test.py
    # 02 — coût de DreamerV3 (GPU)
    $PY experiments/02_dreamer_bench/dreamer_bench.py
    # 02 en essai rapide (moins d'itérations, chiffres plus bruités)
    $PY experiments/02_dreamer_bench/dreamer_bench.py --quick
    # 03 — débit du simulateur avec caméras (Isaac Sim)
    bash experiments/03_render_bench/run_all.sh

Chaque banc affiche ses tableaux dans le terminal au fur et à mesure, et écrit ses fichiers de
résultats à côté de son script.

Pour lire les résultats : le terminal donne tout ; le `RESULTATS.md` de chaque banc donne
l'interprétation et surtout ce que la mesure ne prouve pas ; les `resultats*.csv` donnent les
chiffres bruts (pour refaire des graphiques ou vérifier).

## Résultats en bref : le bilan sur Dreamer après les mesures 01 à 03

L'analyse initiale donnait sept raisons d'écarter Dreamer. Quatre sont réfutées par la mesure.

| Objection initiale | Verdict après mesure |
|---|---|
| Le QR est invisible en 64 × 64 | Fausse : racks et cartons sont très visibles, et le décodage se fait de toute façon à part, en pleine résolution. |
| Un entraînement prend 1 à 2 semaines | Fausse : 3 à 40 h selon le réglage (l'estimation initiale se trompait d'un facteur 30 à 80). |
| La mémoire graphique ne suffit pas | Fausse : 2,33 Go sur 7,8. |
| Le simulateur ne suivra pas | Fausse : 736 pas/s pour 54 nécessaires. |
| Le disque limite le tampon de rejeu | Vraie, mais contournable : ~250 000 pas à 2 caméras, le double à 1 caméra, bien plus en JPEG. |
| La mémoire récurrente ne tient pas l'épisode | Non mesurée ici (vient de la littérature). Une carte externe la rend sans objet. |
| Trop de causes possibles pour déboguer | Un jugement, pas un fait. |

Dreamer était donc redevenu une option sérieuse : la position initiale reposait sur des
estimations, et les mesures les ont réfutées.

## La suite prévue à l'époque

Mesure 04 — la marge disponible : sur la géométrie de l'entrepôt, quel écart y a-t-il entre un
balayage fixe et un glouton omniscient (une stratégie qui connaît d'avance tous les QR) ? C'est ce
qui dit s'il y a quelque chose à apprendre, pour les quatre directions envisagées par l'étude
préalable, Dreamer compris. Elle était jugée la plus importante, car aucune des trois premières
mesures ne dit si la tâche a du contenu.

Le travail suivant est `swarm_qr/` (contribution finale), qui n'utilise pas Dreamer.
