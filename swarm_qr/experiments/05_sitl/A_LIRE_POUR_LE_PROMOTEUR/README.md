# 05 — SITL : un drone ArduPilot vole-t-il dans l'entrepôt ? (étape 1 : l'environnement de simulation)

## En une phrase
Le montage Isaac Sim 5.1 + Pegasus + ArduPilot SITL fonctionne-t-il, le drone vole-t-il,
et à quel prix ?

## La tâche
- Isaac Sim (simulateur 3D de NVIDIA) simule l'entrepôt ; Pegasus (extension d'Isaac Sim
  pour les drones) simule le drone Iris (un quadricoptère) ; SITL (le pilote automatique
  ArduPilot simulé sur le PC) le pilote, comme sur un vrai drone.
- La recette vient de ce qui avait déjà marché, rien n'est improvisé :
  - démarrage, boucle et arrêt propre : le pipeline déjà validé du projet
    (`scripts/11_isaac_sim_drones.py` et `scripts/12_aif_isaac_sim.py`, 8 vols archivés
    dans `logs/runs/`) ;
  - lien avec ArduPilot : l'exemple officiel de Pegasus (`examples/11_ardupilot_multi_vehicle.py`).
- Deux corrections issues de l'enquête :
  - le pas de physique est celui du réglage officiel d'ArduPilot (1/800 s), que l'exemple
    oublie ;
  - le décor est notre entrepôt, chargé par son adresse directe et déjà en cache sur le
    disque — jamais les décors en ligne de Pegasus, dont le téléchargement bloquait la
    fenêtre et déclenchait le message « l'application ne répond pas ».
- Deux sondes : `sonde_gui.py` (manuelle, avec fenêtre, pour voir) et `vol_auto.py`
  (automatique, sans fenêtre, qui fait les mesures).

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../vol_auto.py` | sonde automatique : décollage, stationnaire, arrêt, débit |
| `../resultat_vol.json` | le rapport chiffré de `vol_auto.py` |
| `../sonde_gui.py` | sonde manuelle avec fenêtre : on pilote soi-même |
| `../debug_cams.py` | petit script de débogage des caméras (vue de dessus blanche) |
| `../debug_cams/` | les images écrites par `debug_cams.py` |
| `RESULTATS.md` | les résultats expliqués |

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus + ArduPilot SITL, carte graphique NVIDIA, et un
affichage graphique (`DISPLAY=:1`) : le lancement automatique d'ArduPilot ouvre un terminal.
`run_all.sh` ne lance pas ce test.
Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

Sonde automatique (les mesures) :

    DISPLAY=:1 $PY swarm_qr/experiments/05_sitl/vol_auto.py

→ écrit `resultat_vol.json` dans le dossier du test, même en cas d'échec (il contient alors
les étapes franchies). Le script demande lui-même `--/rtx/verifyDriverVersion/enabled=false`
au démarrage d'Isaac. Comme dans `run_all.sh`, on peut le préfixer par
`timeout -s KILL <secondes>` : Isaac ignore le signal d'arrêt normal.

Sonde manuelle (avec fenêtre) :

    DISPLAY=:1 $PY swarm_qr/experiments/05_sitl/sonde_gui.py

1. la fenêtre Isaac s'ouvre (environ 10 s, jusqu'à 45 s au tout premier lancement, le temps
   de préparer les programmes de la carte graphique) ;
2. l'entrepôt apparaît, le drone Iris est posé dans l'allée ;
3. un terminal MAVProxy (console de commande d'ArduPilot) s'ouvre tout seul ;
4. y taper `mode guided`, puis `arm throttle`, puis `takeoff 3` ;
5. le drone monte à 3 m ; sa position s'affiche aussi dans le terminal de lancement,
   toutes les 2 secondes.

Arrêt : Ctrl+C dans le terminal de lancement, ou fermer la fenêtre Isaac ; Pegasus arrête
alors le SITL et MAVProxy. Si le bureau Linux (GNOME) affiche « ne répond pas » pendant un
chargement : cliquer « Attendre ».

Débogage des caméras (sans ArduPilot) :

    $PY swarm_qr/experiments/05_sitl/debug_cams.py

→ écrit ses images dans `debug_cams/`.

## Résultats en bref
- Chaîne validée de bout en bout, sans intervention : lien de commande, estimateur de
  position prêt en 36 s, mode guidé, armement, décollage à 3 m.
- Stationnaire 10 s : écart-type 0,9 cm (Y) et 1,2 cm (Z) ; 10,8 cm en X, encore pollué par
  la fin de la montée.
- Arrêt depuis 1 m/s : 6,8 s et 1,06 m.
- Débit sans image : 413 pas/s, soit 0,52× le temps réel.

→ détails dans `RESULTATS.md`.
