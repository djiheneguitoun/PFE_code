# 03 — Le drone lit-il les QR codes en volant ? (étape 1 : l'environnement de simulation)

## En une phrase
Un drone piloté par ArduPilot lit-il les QR codes des cartons, d'abord à l'arrêt devant un
panneau, puis en volant le long d'un rack ?

## La tâche
- C'est le cœur de l'inventaire : si le drone ne sait pas lire, le reste du système ne sert
  à rien.
- Le drone est piloté par SITL (le pilote automatique ArduPilot simulé sur le PC) : il a son
  inertie et ses imprécisions. Isaac Sim simule l'entrepôt et la caméra.
- Un seul vol (graine 7), deux essais :
  - la planche : le drone se place à six distances (0,5 à 3 m) face à un panneau de 40 cm,
    photographie, et on vérifie que c'est bien le code visé qui est lu, pas un voisin ;
  - la vidéo : il longe le rack à 0,5 m/s en lisant en continu.
- Pour rejoindre le rack, le drone passe par le couloir ouvert au bout des racks :
  l'évitement d'obstacles n'existe pas encore à ce stade du projet.

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../run.py` | le test complet (un seul lancement) |
| `../planche_qr.jpg` | les six photos, une par distance, résultat écrit dessus |
| `../vol_le_long_du_rack.mp4` | **la vidéo du vol le long du rack** |
| `../resultat.json` | les mesures chiffrées |
| `RESULTATS.md` | les résultats expliqués |

## La vidéo à regarder
`../vol_le_long_du_rack.mp4` (52 images, 8 images/s) : à gauche, la caméra gauche du drone
avec le nombre de codes différents lus depuis le début du passage (et, en bas, les codes lus
dans l'image) ; à droite, la vue de dessus avec la position du drone.

## Comment le lancer
Prérequis : Isaac Sim 5.1 + Pegasus + ArduPilot SITL, carte graphique NVIDIA, et un
affichage graphique (`DISPLAY=:1`) : le lancement automatique d'ArduPilot ouvre un terminal.
Depuis la racine du projet, `PY` étant le Python de l'environnement Isaac Sim
(sur la machine de simulation : `PY=~/isaac5_env/bin/python`) :

    KIT=--kit_args=--/rtx/verifyDriverVersion/enabled=false
    DISPLAY=:1 timeout -s KILL 1200 $PY swarm_qr/experiments/03_images_qr/run.py --seed 7 $KIT

- Le pilote automatique se lance tout seul.
- Sorties, dans le dossier du test : `planche_qr.jpg`, `vol_le_long_du_rack.mp4`,
  `resultat.json`.
- Options : `--speed` (vitesse le long du rack, 0,5 m/s par défaut) et `--standoff`
  (distance caméra–panneau pendant la vidéo, 1,2 m par défaut).
- `../../run_all.sh` lance la même commande (`bash swarm_qr/experiments/run_all.sh`), à
  lancer depuis une session qui a un affichage graphique.
- `timeout -s KILL 1200` tue Isaac au bout de 20 min : il ignore le signal d'arrêt normal.
- `--kit_args=--/rtx/verifyDriverVersion/enabled=false` désactive le contrôle de version du
  pilote de la carte graphique au démarrage d'Isaac (le script le demande aussi lui-même).

## Résultats en bref
- Les six distances sont lues, de 0,5 à 3 m, et c'est bien le code visé (BOX_000).
- Le drone arrive entre 4 et 11 cm du point demandé.
- À partir de 1,5 m, des codes voisins sont lus en même temps (trois codes à 3 m).
- Le long du rack : 6 codes différents lus en un seul passage.
- Une ancienne conclusion (« pas de lecture sous 1 m environ ») était fausse : elle venait
  d'une visée qui oubliait le décalage de la caméra sur le côté du drone.

→ détails dans `RESULTATS.md`.
