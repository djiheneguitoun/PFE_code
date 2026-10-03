# installation — Mise en place de la machine de simulation (début du projet)

## En une phrase

Cinq scripts qui installent, dans l'ordre, les outils de la première phase du projet sur une
machine Ubuntu (Gazebo, ArduPilot SITL, ns-3, NS3-Sionna), plus `install_isaac_sim.sh`, à la
racine du projet, qui installe Isaac Sim 4.5.0 et le simulateur de drones Pegasus.

## La tâche

- Première phase du projet : drones simulés dans Gazebo avec ArduPilot SITL (le pilote
  automatique ArduPilot simulé sur le PC), réseau entre drones simulé par ns-3 (simulateur de
  réseaux) — voir `scripts/06_launch_multi_drones.sh`, `scripts/08_wifi_bridge.py`,
  `scripts/09_5g_lena_bridge.py` et `scenarios/`.
- Phase suivante : Isaac Sim (simulateur 3D de NVIDIA) + Pegasus (extension d'Isaac Sim pour les
  drones), installés par `install_isaac_sim.sh`.
- Les scripts utilisent `apt-get` et `sudo` ; `install_isaac_sim.sh` demande Python 3.10.

## Contenu du dossier, dans l'ordre d'installation

| Ordre | Fichier | Ce qu'il installe |
|---|---|---|
| 1 | `../01_install_dependencies.sh` | paquets apt : outils de compilation (git, cmake, build-essential, ninja…), Python 3 et ses bibliothèques pour ArduPilot (jinja2, numpy, empy, lxml, matplotlib, opencv, wxgtk…), GStreamer (vidéo) ; par pip (`--user`) : `numpy<2`, pymavlink, MAVProxy, pexpect, dronekit, dronekit-sitl |
| 2 | `../02_install_gazebo.sh` | ajoute le dépôt apt d'OSRF et installe Gazebo Harmonic (`gz-harmonic`), puis teste `gz sim --version` |
| 3 | `../03_install_ardupilot_sitl.sh` | clone ArduPilot (branche par défaut) dans `~/ardupilot`, lance son script officiel `install-prereqs-ubuntu.sh`, compile ArduCopter SITL (`./waf configure --board sitl` puis `./waf copter`, ~5–10 min) ; clone et compile le greffon `ardupilot_gazebo` dans `~/ardupilot_gazebo` ; ajoute `GZ_SIM_SYSTEM_PLUGIN_PATH` et `GZ_SIM_RESOURCE_PATH` à `~/.bashrc` |
| 4 | `../04_install_ns3.sh` | dépendances de ns-3 et de NS3-Sionna (ZMQ, ProtoBuf, python3-venv) ; télécharge **ns-allinone-3.40** depuis nsnam.org dans `~`, puis `./ns3 configure --enable-examples --enable-tests` et `./ns3 build` (~5–10 min) |
| 5 | `../05_install_ns3sionna.sh` | clone NS3-Sionna (`github.com/tkn-tub/ns3sionna`, calcul de la propagation radio par lancer de rayons) dans `contrib/sionna` de ns-3, recompile ns-3, crée le venv `sionna-venv` du serveur Python Sionna (`requirements.txt`), teste les imports, ajoute `NS3_DIR` et `NS3_SIONNA_DIR` à `~/.bashrc` |
| 6 | `../../../install_isaac_sim.sh` (racine du projet) | voir ci-dessous |
| — | `README.md` | ce fichier |

### Ce que fait `install_isaac_sim.sh`

1. Vérifie les prérequis : pilote NVIDIA ≥ 535.129 (sinon arrêt), GPU, Python 3.10, au moins
   20 Go libres (simple avertissement). Avec `--dry-run`, il s'arrête là.
2. Installe par apt les outils de compilation, Python 3.10 (venv, dev) et des bibliothèques
   graphiques.
3. Crée le venv `~/isaac_sim_env` (Python 3.10).
4. Installe **Isaac Sim 4.5.0** par pip : `isaacsim[all]` et `isaacsim[extscache]` depuis l'index
   de NVIDIA (`pypi.nvidia.com`), en acceptant la licence (`OMNI_KIT_ACCEPT_EULA=YES`) ;
   10 à 20 min.
5. Installe **Pegasus Simulator** : clone `github.com/PegasusSimulator/PegasusSimulator` (branche
   `main`) dans `~/PegasusSimulator`, puis `pip install -e extensions/pegasus.simulator`.
6. Installe numpy, scipy, pyyaml, matplotlib, puis vérifie que `isaacsim` et `pegasus`
   s'importent.
7. Écrit `~/isaac_sim_env/activate_isaac.sh` (active le venv, accepte la licence, exporte
   `ISAACSIM_PATH`). Ce fichier est utilisé par `scripts/11_launch_isaac_sim_drones.sh` et
   `scripts/12_launch_aif_isaac_sim.sh`.

## Comment le lancer

Prérequis : Ubuntu, droits sudo, connexion internet ; carte NVIDIA pour Isaac Sim. Durée :
compter plusieurs dizaines de minutes au total (compilations d'ArduPilot et de ns-3 ~5–10 min
chacune, Isaac Sim 10–20 min).

Depuis la racine du projet (sur la machine de simulation : `cd ~/simulation_mc02`) :

    bash scripts/installation/01_install_dependencies.sh
    bash scripts/installation/02_install_gazebo.sh
    bash scripts/installation/03_install_ardupilot_sitl.sh
    bash scripts/installation/04_install_ns3.sh
    bash scripts/installation/05_install_ns3sionna.sh
    ./install_isaac_sim.sh --dry-run    # vérifie seulement les prérequis
    ./install_isaac_sim.sh
    source ~/.bashrc                    # recharge les chemins ajoutés par 03 et 05

Les scripts 03, 04 et 05 peuvent être relancés : ils mettent à jour les dépôts déjà clonés
(03, 05) ou sautent le téléchargement (04).

Après l'étape 5, pour utiliser les scénarios réseau : voir
`scenarios/A_LIRE_POUR_LE_PROMOTEUR/README.md` (copie dans `scratch/`, compilation). Le serveur
Sionna se lance depuis `contrib/sionna/model/ns3sionna` avec `source sionna-venv/bin/activate`
puis `./run.sh`.
