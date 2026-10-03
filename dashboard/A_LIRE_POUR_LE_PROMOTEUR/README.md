# dashboard — Tableau de bord web « WiFi / 5G + exploration » (première phase, avant swarm_qr)

## En une phrase
Une page web qui affiche en direct les mesures radio WiFi et 5G entre les drones simulés,
puis l'exploration de l'entrepôt par l'inférence active (AIF).

## La tâche
Dans la première phase du projet, on a comparé deux réseaux pour faire communiquer 3 drones
dans l'entrepôt :
- le WiFi 802.11n en mode « ad hoc » (les drones se parlent directement) ;
- la 5G NR, où chaque drone passe par une antenne-relais appelée gNB (placée en x = 0, y = 0, z = 6 m).

Pour chaque paire de drones, on mesure le RSSI (puissance du signal reçu, en dBm : plus c'est
proche de 0, meilleur c'est) et la latence (temps de transmission, en ms). Le RSSI est calculé
par Sionna (logiciel de « lancer de rayons » radio qui tient compte des murs et des étagères), la
latence par NS-3 (simulateur de réseau). Plus tard, un onglet « Exploration » a été ajouté pour
suivre l'exploration AIF (voir aussi `../../dashboard_aif/`).

Le tableau de bord ne calcule rien et n'enregistre rien : toutes les 2 s, il relit des fichiers
que d'autres scripts écrivent dans `/tmp` sur la machine de simulation, et les affiche.
Il n'a donc pas de résultats propres.

## Ce que montre la page (4 onglets en haut)

| Onglet | Contenu |
|---|---|
| Exploration (par défaut) | grande carte d'occupation, une carte par drone, 4 jauges, journal |
| Comparaison | WiFi et 5G côte à côte : chiffres, courbes, carte 2D, tableaux, barres |
| WiFi 802.11n | seulement les mesures WiFi |
| 5G NR | seulement les mesures 5G |

Détail :
- Exploration : grille de l'entrepôt (sombre = jamais vue, vert = libre, rouge = occupée),
  traces et cap de chaque drone, portée du lidar (capteur laser qui mesure les distances, 8 m par
  défaut) ; jauges du pourcentage exploré, des altitudes, des vitesses et des distances entre
  drones (en rouge sous 3 m).
- Onglets réseau : 6 chiffres (drones actifs, RSSI et latence moyens en WiFi et en 5G, nombre
  de « ticks » = cycles de mesure), courbes RSSI et latence sur les 120 derniers ticks, carte 2D
  des positions avec les liens, tableau par paire (WiFi) et par drone (5G), étiquette de qualité
  (Excellent au-dessus de -50 dBm, Good au-dessus de -60, Fair au-dessus de -70, sinon Weak ;
  BLOCKED si aucun trajet radio).
- En bas : un journal de la page (connexion, ticks reçus, erreurs).

## D'où viennent les données
Tous les fichiers sont dans `/tmp` de la machine de simulation (Linux). Le serveur les relit à
chaque demande de la page.

| Fichier lu | Écrit par | Sert à |
|---|---|---|
| `drone_positions.csv` | `scripts/07_multi_drone_flight.py`, `07b_dynamic_flight.py` ou `12_aif_isaac_sim.py` | positions des drones |
| `drone_rssi_latency.csv` | `scripts/08_wifi_bridge.py` | dernière mesure WiFi par paire |
| `drone_bridge_log.csv` | `scripts/08_wifi_bridge.py` | numéro du tick courant |
| `ns3_output.csv` | scénario NS-3 WiFi (`scenarios/drone-wifi-scenario.cc`) | WiFi de secours |
| `drone_rssi_sionna.csv` | `scripts/09_5g_lena_bridge.py` | RSSI 5G de chaque drone vers le gNB |
| `drone_latency_ns3.csv` | `scripts/09_5g_lena_bridge.py` | latence et gigue 5G par paire |
| `drone_5g_metrics.csv` | scénario NS-3 5G (`scenarios/drone-5g-nr-scenario.cc`) | 5G de secours |
| `aif_state.json` | `scripts/12_aif_isaac_sim.py` | onglet Exploration |
| `exploration_state.json` | ancien format de l'exploration | onglet Exploration, si `aif_state.json` manque |

La gigue est la variation de la latence d'un paquet à l'autre. Les fichiers « de secours » ne
sont lus que si le fichier principal manque.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../dashboard_server.py` | serveur web : envoie la page et rassemble les fichiers en un JSON (`/api/data`) |
| `../index.html` | structure de la page (onglets, cartes, graphiques, tableaux) |
| `../app.js` | logique de la page : interroge le serveur toutes les 2 s et redessine tout |
| `../style.css` | apparence (thème sombre, WiFi en cyan, 5G en violet) |

## Comment le lancer
Prérequis : Python 3 seul (le serveur n'utilise que la bibliothèque standard), et un navigateur
qui a accès à Internet (la bibliothèque de graphiques Chart.js et les polices sont chargées
depuis Internet).

Depuis la racine du projet :

    python3 dashboard/dashboard_server.py

Options (tirées du code du serveur) :

    python3 dashboard/dashboard_server.py --port 8051         (autre port ; défaut 8050)
    python3 dashboard/dashboard_server.py --host 127.0.0.1    (visible seulement depuis ce PC ; défaut 0.0.0.0 = tout le réseau)

Le serveur affiche les fichiers qu'il surveille ; Ctrl+C l'arrête. Il démarre même si aucun
fichier n'existe : la page reste alors « En attente... ».

Pour avoir des données, lancer dans d'autres terminaux, sur la même machine :
- un vol de drones qui écrit les positions, par exemple `scripts/06_launch_multi_drones.sh`
  (Gazebo + ArduPilot SITL, le pilote automatique simulé sur le PC) puis
  `scripts/07_multi_drone_flight.py` ;
- la mesure WiFi : `python3 scripts/08_wifi_bridge.py` (options : `--test` = faux drones qui
  bougent, sans SITL ; `--no-ns3` = latence estimée sans NS-3 ; `--no-render` ; `--cycles N`) ;
- la mesure 5G : `python3 scripts/09_5g_lena_bridge.py` (options : `--test`, `--interval 3`,
  `--duration 0`) ;
- ou l'exploration AIF : `scripts/12_launch_aif_isaac_sim.sh 3` (voir
  `../../dashboard_aif/A_LIRE_POUR_LE_PROMOTEUR/README.md`).

Les scripts 08 et 09 importent Sionna RT et NS-3 (installés par
`scripts/installation/04_install_ns3.sh` et `05_install_ns3sionna.sh`, dans `~/ns-allinone-3.40`) :
il faut les lancer avec un Python où Sionna RT est installé.

## Comment l'ouvrir dans un navigateur
- Sur la machine de simulation : `http://localhost:8050`
- Depuis un autre PC du même réseau : `http://<adresse IP de la machine>:8050`
- Pour vérifier que le serveur répond : `http://localhost:8050/api/health` ;
  les données brutes : `http://localhost:8050/api/data`.
