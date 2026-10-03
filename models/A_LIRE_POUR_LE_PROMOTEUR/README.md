# models — Modèles Gazebo du drone Iris (phase 06)

## En une phrase

Trois copies du modèle Gazebo du quadrirotor Iris, une par drone, utilisées par les vols multi-drones
sous Gazebo (phases 06 et 07).

## La tâche

Pour faire voler plusieurs drones dans Gazebo (simulateur de robots), chaque drone doit dialoguer avec sa propre
instance d'ArduPilot SITL (le pilote automatique ArduPilot simulé sur le PC). Il faut donc un modèle par drone,
avec ses propres ports réseau. Ces modèles ne servent pas aux phases Isaac Sim (11, 12) ni à `swarm_qr/` :
celles-ci utilisent le modèle Iris fourni par Pegasus.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../iris_instance_<i>/model.sdf` | description du drone au format SDF (XML de Gazebo) |
| `../iris_instance_<i>/model.config` | fiche d'identité : nom `iris_instance_<i>`, version, auteurs, dépendance à `iris_with_standoffs` |

`model.sdf` reprend `iris_with_standoffs` (châssis, 4 moteurs, centrale inertielle) et ajoute :
- la portance des hélices (plugins « lift-drag ») ;
- le plugin ArduPilotPlugin, qui échange capteurs et commandes moteurs avec le SITL (ports « FDM ») ;
- un LiDAR 2D (capteur laser qui mesure les distances) : 360 rayons sur 360°, portée 0,08 à 10 m, 5 mesures par
  seconde, fixé 5 cm au-dessus du châssis.

Les trois copies ne diffèrent que par leur nom et leur port :

| Dossier | Drone | Port FDM (`fdm_port_in`) | Adresse MAVLink du SITL |
|---|---|---|---|
| `../iris_instance_0/` | 0 | 9002 | tcp:127.0.0.1:5760 |
| `../iris_instance_1/` | 1 | 9012 | tcp:127.0.0.1:5770 |
| `../iris_instance_2/` | 2 | 9022 | tcp:127.0.0.1:5780 |

(MAVLink : protocole de messages utilisé par les scripts de vol 07 et 07b pour commander les drones.)

## Comment ils sont créés et utilisés

`../../scripts/06_launch_multi_drones.sh` les fabrique à chaque lancement : il efface `iris_instance_<i>`,
recopie `~/ardupilot_gazebo/models/iris_with_ardupilot`, change le port FDM et le nom, puis ajoute le LiDAR.
Le monde généré `worlds/warehouse_drones.sdf` les inclut ensuite (`model://iris_instance_<i>`).
Les fichiers présents ici sont donc la copie laissée par le dernier lancement.

    bash scripts/06_launch_multi_drones.sh 3

Prérequis : Gazebo Harmonic, ArduPilot et le plugin ardupilot_gazebo (`../../scripts/installation/`) ;
le script travaille dans `~/simulation_mc02` (le dossier du projet sur la machine Linux ; ces modèles vont
dans son sous-dossier `models/`).

## Remarques

- Toute modification faite à la main ici est écrasée au lancement suivant de 06.
- Pas de résultats associés : les vols Gazebo n'ont pas laissé de fichiers enregistrés.
