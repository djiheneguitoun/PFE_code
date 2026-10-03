# logs — Résultats bruts des runs d'exploration AIF (étape 12)

## En une phrase

`logs/runs/` garde un dossier par lancement de l'exploration par inférence active (AIF) dans Isaac Sim,
avec ses données et ses graphiques.

## La tâche

Chaque run fait explorer l'entrepôt par 3 drones simulés jusqu'à 93 % de couverture, avec une configuration donnée
(planificateur, architecture, réseau, incident éventuel). À la fin du run, `scripts/run_artifacts.py` (appelé par
`scripts/12_aif_isaac_sim.py`) crée automatiquement le dossier du run. La série a été lancée par
`scripts/run_all_experiments.sh`. Ces dossiers sont des données générées : on ne les modifie pas.

## Nom des dossiers

`run_<AAAAMMJJ>_<HHMMSS>_<tag>`, la date et l'heure étant celles de la fin du run. Le tag se lit
« planificateur _ architecture _ scénario » :
- planificateur : `aif` (inférence active) ou `heur` (heuristique) ;
- architecture : `cent` (centralisée : un cloud décide) ou `dist` (distribuée : chaque drone décide) ;
- scénario : `baseline` (référence), `cloud_loaded` (cloud lent), `kill_d0` (drone 0 perdu), `cut_cloud`
  (cloud coupé), `cut_links` (liens entre drones coupés), `obstacle` ; `s20` = incident au step 20.

## Contenu du dossier

| Dossier | Scénario |
|---|---|
| `../runs/run_20260531_041619_aif_cent_baseline/` | AIF centralisé, référence |
| `../runs/run_20260531_042411_aif_dist_baseline/` | AIF distribué, référence |
| `../runs/run_20260531_043247_heur_cent_baseline/` | heuristique centralisée, référence |
| `../runs/run_20260531_043951_aif_cent_cloud_loaded/` | cloud lent (aller-retour 1 500 ms) |
| `../runs/run_20260531_044951_aif_cent_kill_d0_s20/` | drone 0 mis hors service au step 20 |
| `../runs/run_20260531_075736_aif_cent_cut_cloud_s20/` | lien cloud coupé au step 20 |
| `../runs/run_20260531_100239_aif_dist_cut_links_s20/` | tous les liens entre drones coupés au step 20 |
| `../runs/run_20260608_042211_aif_cent_obstacle_s20/` | obstacle lâché au step 20 |

## Ce que contient chaque dossier de run

| Fichier | Contenu |
|---|---|
| `README.md` | résumé généré : tag, planificateur, architecture (prévue et effective), réseau, nombre de steps, entropie et couvertures finales |
| `config.json` | tous les paramètres du run (taille de la grille, LiDAR, poids du planificateur, incident…) |
| `history.json` | une entrée par step : entropie, couvertures, innovation, phase de résilience, drones actifs, messages envoyés / livrés / perdus, décisions par minute |
| `state_final.json` | état final : position et trajet des drones, carte fusionnée (82 × 52 cases de 0,5 m), événements de résilience, état du réseau et latences ns-3 |
| `qr_stats.json` | lecture du panneau QR de test : images analysées et décodées, par drone |
| `coverage.png`, `coverage_interior.png` | couverture au fil des steps (deux définitions) |
| `entropy.png` | incertitude moyenne de la carte |
| `innovation.png` | écart entre LiDAR et carte, avec les phases de résilience |
| `resilience_phases.png` | entropie, phases (récupération en rouge, durable en jaune) et incidents |
| `discovery_rate.png` | gain de couverture par step |
| `coverage_known_vs_global.png` | couverture globale contre couverture connue de ceux qui décident |
| `decisions_per_min.png` | décisions fraîches par minute |
| `belief_map_final.png`, `trajectories.png` | carte finale et trajets des drones |
| `ns3_latencies.png` | latences ns-3 entre paires de drones (ms) |

Pendant un run, l'état courant est aussi écrit dans `/tmp` (`aif_state.json`, `aif_history.json`,
`aif_diagnostic.log`, `qr_state.json`, images des caméras) : ces fichiers ne sont pas conservés ici.

## Comment les produire et les consulter

Depuis la racine du projet, sur la machine de simulation (prérequis : voir
`../../scripts/A_LIRE_POUR_LE_PROMOTEUR/README.md`) :

    bash scripts/run_all_experiments.sh                 # toute la série (environ 7 à 10 min par run)
    bash scripts/12_launch_aif_isaac_sim.sh 3 --max-steps 80 --run-tag aif_cent_baseline --planner aif --arch centralized --ns3 wifi

Pour parcourir les runs : depuis `dashboard_aif/`, `python3 server.py --port 8060`, puis http://localhost:8060
(onglet « Runs »).

## Résultats en bref

Les 8 runs atteignent 93 % de couverture :
45 steps en AIF centralisé, 53 en distribué, 69 en heuristique ; 79 avec un drone perdu.
→ résultats expliqués dans `../../scripts/A_LIRE_POUR_LE_PROMOTEUR/RESULTATS.md`.
