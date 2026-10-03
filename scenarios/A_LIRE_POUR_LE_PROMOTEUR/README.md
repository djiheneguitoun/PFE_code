# scenarios — Scénarios réseau ns-3 entre drones (première phase du projet)

## En une phrase

Deux programmes pour ns-3 (simulateur de réseaux) qui calculent, pour chaque paire de drones, la
qualité du lien radio (RSSI = puissance reçue, en dBm) et la latence, à partir des positions des
drones lues dans un fichier CSV : l'un en Wi-Fi direct, l'autre en 5G.

## La tâche

- Les drones de l'essaim échangent des messages ; ces scénarios simulent leur réseau radio dans
  ns-3.40 (installé par `scripts/installation/04` et `05`).
- Les positions arrivent par `/tmp/drone_positions.csv`, écrit par le script de vol (Gazebo +
  ArduPilot, ou Isaac Sim). Les résultats repartent dans un autre CSV, lu par les scripts « pont »
  Python et par le tableau de bord (`dashboard/dashboard_server.py`).
- Pas de simulateur de drones ici : ns-3 seul. Ces scénarios servent à la première phase et à la
  phase d'inférence active (AIF) ; la contribution finale `swarm_qr/` ne les utilise pas.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../drone-wifi-scenario.cc` | Wi-Fi entre drones, en temps réel, positions relues toutes les 0,5 s |
| `../drone-5g-nr-scenario.cc` | 5G avec une antenne fixe, instantané de 2 s pour des positions figées |
| `README.md` | ce fichier |

### Ce que simule `drone-wifi-scenario.cc`

- N drones (3 par défaut) en Wi-Fi 802.11n ad hoc (sans point d'accès), débit fixe (MCS 7),
  puissance 20 dBm, adresses 10.1.1.(i+1).
- Propagation : modèle « log-distance » par défaut (exposant 3, 40 dB de perte à la distance de
  référence) ou `sionna` (lancer de rayons par NS3-Sionna : ns-3 interroge un serveur Python
  Sionna par ZMQ, `tcp://localhost:5555`, scène `simple_room/simple_room.xml` par défaut).
- Trafic : chaque drone envoie un paquet UDP de 256 octets toutes les 0,1 s au drone suivant (en
  anneau), à partir de t = 1 s.
- Toutes les 0,5 s : relit les positions, déplace les nœuds et écrit pour chaque paire la
  distance, le RSSI (donné par le modèle de propagation) et la latence (délai moyen des paquets
  reçus depuis la mesure précédente, mesuré par FlowMonitor ; à défaut, temps de vol + 2 ms).
- Temps réel : 1 s simulée = 1 s d'horloge, pour suivre un vol en direct.
- Sortie (`/tmp/ns3_output.csv` par défaut) :
  `time_s,drone_i,drone_j,distance_m,rssi_dbm,latency_ms,xi,yi,zi,xj,yj,zj`.
- Options : `--nDrones`, `--posFile`, `--outFile`, `--simTime` (60 s), `--updateInterval` (0,5 s),
  `--channelModel` (`log-distance` ou `sionna`), `--sionnaEnv`, `--sionnaUrl`.
- Entrée : une ligne d'en-tête, puis `id,x,y,z` en mètres.

### Ce que simule `drone-5g-nr-scenario.cc`

- Une antenne 5G fixe (gNB) en (0, 0, 6) m ; chaque drone est un terminal 5G (UE). Module
  5G-LENA (`nr`) : bande de 20 MHz à 3,5 GHz, canal 3GPP « bureau ouvert en intérieur » avec
  ombrage, numérologie 1 (sous-porteuses de 30 kHz), gNB à 23 dBm, antennes 4 × 8 (gNB) et 2 × 4
  (drones), faisceaux idéaux, cœur de réseau EPC (2 ms sur le lien gNB–cœur).
- Trafic : pour chaque paire a < b, paquets UDP de 500 octets toutes les 10 ms de a vers b
  (a → gNB → cœur → gNB → b), de 0,4 s à la fin (2 s par défaut). Pas de temps réel : calculé
  aussi vite que possible, positions figées.
- Après la simulation : latence et gigue (variation du délai) moyennes par paire (FlowMonitor ;
  −1 si aucun paquet reçu), et RSSI de chaque drone recalculé à part (modèle 3GPP intérieur,
  ombrage tiré au hasard).
- Sortie (`/tmp/drone_5g_metrics.csv` par défaut) :
  `drone_a,drone_b,rssi_a_dBm,rssi_b_dBm,latency_ms,jitter_ms,dist_ab_m,rx_packets`, plus un
  tableau dans le terminal.
- Options : `--posFile`, `--outFile`, `--simTime` (2 s), `--txPower` (23 dBm), `--frequency`
  (3,5e9 Hz), `--bandwidth` (20e6 Hz), `--gnbX`, `--gnbY`, `--gnbZ` (m), `--seed` (0 = tirée de
  l'horloge : résultats différents à chaque lancement).
- Entrée : `id,x,y,z` ou `t,id,x,y,z` ; lignes vides, lignes « # » et en-tête `drone_id`
  ignorés ; au moins 2 drones, sinon arrêt.

## Comment les compiler et les lancer

Prérequis : ns-3.40 dans `~/ns-allinone-3.40/ns-3.40` avec NS3-Sionna (`contrib/sionna`) ;
pour la 5G, le module 5G-LENA dans `contrib/nr`.

Depuis la racine du projet (sur la machine de simulation : `cd ~/simulation_mc02`) :

    cp scenarios/drone-wifi-scenario.cc scenarios/drone-5g-nr-scenario.cc ~/ns-allinone-3.40/ns-3.40/scratch/
    cd ~/ns-allinone-3.40/ns-3.40
    ./ns3 build
    ./ns3 run "drone-wifi-scenario --nDrones=3 --simTime=60"
    ./ns3 run "drone-5g-nr-scenario --posFile=/tmp/drone_positions.csv"

ns-3 compile tout fichier placé dans `scratch/` ; `./ns3 run` le retrouve par son nom. Les
scripts Python lancent les scénarios avec `--no-build` : il faut donc avoir compilé une fois
avant. Sans script de vol, on peut écrire un fichier de positions à la main, lisible par les deux
scénarios :

    drone_id,x,y,z
    0,-3.0,0.0,4.0
    1,0.0,0.0,5.0
    2,5.0,2.0,3.5

Le scénario Wi-Fi n'affiche rien par défaut : ses messages (dont les statistiques finales par
flux) sont des journaux ns-3, visibles avec `NS_LOG=DroneWifiScenario=info` devant la commande.
Pour `--channelModel=sionna`, lancer d'abord le serveur Sionna (voir
`scripts/installation/A_LIRE_POUR_LE_PROMOTEUR/README.md`).

## Quels scripts Python les utilisent

| Script | Scénario | Comment |
|---|---|---|
| `../../scripts/08_wifi_bridge.py` | Wi-Fi | lance en arrière-plan, depuis le dossier de ns-3, `ns3 run "drone-wifi-scenario --nDrones=3 --posFile=/tmp/drone_positions.csv --outFile=/tmp/ns3_output.csv --simTime=600 --channelModel=log-distance" --no-build`, puis lit la latence dans `/tmp/ns3_output.csv` ; le RSSI est calculé à part, en Python, par Sionna (`--no-ns3` : sans ns-3) |
| `../../scripts/09_5g_lena_bridge.py` | 5G | toutes les `--interval` s (3 par défaut), lance `ns3 run drone-5g-nr-scenario --no-build` (options par défaut, 60 s au plus), lit latence et gigue dans `/tmp/drone_5g_metrics.csv` ; RSSI antenne → drone recalculé par Sionna |
| `../../scripts/12_ns3_bridge.py` | les deux | module chargé par `scripts/12_aif_isaac_sim.py` (option `--ns3 none/wifi/5g`, `wifi` par défaut) ; lance le scénario choisi ; sorties lues par `scripts/aif_core/network.py` |

Les deux ponts 08 et 09 utilisent la scène Sionna de l'entrepôt
`contrib/sionna/model/ns3sionna/models/warehouse/warehouse.xml`. `scripts/run_all_experiments.sh`
lance l'AIF avec `--ns3 wifi`.
