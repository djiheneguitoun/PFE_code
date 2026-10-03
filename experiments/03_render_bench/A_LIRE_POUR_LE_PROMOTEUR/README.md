# 03 — Débit du simulateur avec caméras (bancs préliminaires)

## En une phrase

Isaac Sim peut-il produire assez d'images pour nourrir l'entraînement de DreamerV3, soit 54 pas
d'environnement par seconde avec 2 caméras par drone ?

## La tâche

- La mesure 02 montre qu'à `train_ratio` 64, l'apprentissage consomme 54 pas d'environnement
  neufs par seconde : le simulateur doit suivre.
- Le script construit une scène simple dans Isaac Sim 5.1 / Isaac Lab 2.3.2 : un sol, une
  lumière, deux blocs de 0,6 × 4 × 3 m couleur carton (les « racks ») et 1 ou 2 caméras
  latérales (champ 60°) entre eux. Aucun drone.
- La scène est copiée N fois (N « environnements » simulés ensemble) ; toutes les caméras sont
  rendues d'un coup par TiledCamera (rendu « par tuiles » : les petites images sont assemblées
  dans une grande).
- 20 pas de chauffe, vérification que l'image n'est pas noire, puis 60 pas chronométrés.
- Isaac Sim ne sait pas reconstruire une scène dans le même processus : une configuration par
  lancement ; `run_all.sh` enchaîne les 7 configurations.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../render_bench.py` | mesure une configuration et ajoute une ligne à `resultats.csv` |
| `../run_all.sh` | lance les 7 configurations, un processus Isaac chacune |
| `../resultats.csv` | les 7 mesures |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer

Prérequis : Isaac Sim 5.1 + Isaac Lab 2.3.2 (environnement `~/isaac5_env`), GPU NVIDIA ; ni
Pegasus ni ArduPilot. Durée : ~7 min pour le balayage (~40 s de démarrage d'Isaac par
configuration).

Depuis la racine du projet (sur la machine de simulation : `cd ~/simulation_mc02`) :

    bash experiments/03_render_bench/run_all.sh

`run_all.sh` se place lui-même à la racine, utilise `$HOME/isaac5_env/bin/python`, efface
`resultats.csv`, lance chaque configuration sous `timeout -s KILL 300` (obligatoire : Isaac
ignore le signal d'arrêt normal ; « BLOQUÉ ou échec » s'affiche si aucune ligne utile n'est
sortie), puis affiche le récapitulatif.

Une seule configuration, avec `PY=~/isaac5_env/bin/python` :

    PYTHONUNBUFFERED=1 $PY experiments/03_render_bench/render_bench.py \
        --envs 8 --cams 2 --res 64 --steps 60 \
        --kit_args="--/rtx/verifyDriverVersion/enabled=false"

Options : `--envs` (environnements, 8 par défaut), `--cams` (1 ou 2, défaut 2), `--res` (côté
de l'image en px, défaut 64), `--steps` (pas chronométrés, défaut 60). Le mode sans fenêtre et
les caméras sont forcés par le script. Chaque lancement ajoute une ligne à `../resultats.csv`.
Le drapeau `verifyDriverVersion` fait seulement taire l'avertissement de pilote (535.32 installé
contre 535.129 demandé), sans effet sur le rendu.

## Résultats en bref

- 736 pas/s à 32 environnements pour 54 nécessaires : marge ×13.
- 3 environnements (un par drone de l'essaim) : 93 pas/s, déjà 1,7 fois le besoin.
- Débit presque proportionnel au nombre d'environnements ; 4 fois plus de pixels (64 → 128 px)
  ne coûtent que 18 %.
- Chiffres optimistes (scène très simple) et forte variance d'un lancement à l'autre.

→ détails dans `RESULTATS.md`.
