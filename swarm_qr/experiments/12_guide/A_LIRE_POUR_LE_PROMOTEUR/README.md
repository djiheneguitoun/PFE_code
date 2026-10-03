# 12 — Le guide vision-langage (étape 8)

## En une phrase

Un modèle vision-langage (une IA qui lit à la fois des images et du texte) peut-il conseiller
aux drones une meilleure zone à visiter que le planificateur géométrique ?

## La tâche

- Le « guide » reçoit la photo d'un drone, le plan vu de dessus et/ou un résumé texte de la
  carte, et désigne une des 6 zones candidates (et un côté d'abordage dans la 1re version).
- Il conseille seulement : son avis ajoute λ × 10 points (λ × 5 de plus pour le bon côté) à la
  note des cibles. λ = 0 le retire : c'est la valeur de l'évaluation finale.
- Tout se mesure hors ligne, sans simulateur, sur des « instantanés » écrits par `mission.py`
  pendant les vols de l'étape 5 (`<vol>/mission.json` et `<vol>/instantanes/`) : plan annoté,
  photos des drones, état de la carte, et la bonne réponse connue après coup (la zone qui
  contient le plus de QR non lus).
- Modèles : SmolVLM-500M, puis Qwen2.5-VL-7B et surtout Qwen2.5-VL-3B, compressé en 4 bits
  (bibliothèque bitsandbytes) ou non, puis un petit entraînement LoRA (petites couches ajoutées
  et apprises, le modèle restant gelé) du 3B.
- Comparaison avec le hasard, le planificateur géométrique et une règle de 3 lignes.

## Contenu du dossier

| Fichier | Rôle |
|---|---|
| `../banc.py` | 1er banc : photo + plan, 2 questions (zone, côté) ; références (hasard, géométrie, « toujours ouest ») |
| `../variantes.py` | banc des variantes de question (27, dictionnaire `VARIANTES`), numéros de zones mélangés |
| `../perception.py` | contrôle de perception : 6 questions à réponse calculée sur le plan, 4 compressions |
| `../introspection.py` | demande au modèle d'expliquer sa réponse (15 cas) |
| `../entraine.py` | entraînement LoRA du modèle 3B, évaluation avant / après |
| `../adaptateur_lora/` | l'adaptateur LoRA appris (15 Mo), voir plus bas |
| `../resultats_*.json` | résultats bruts, liste ci-dessous |
| `RESULTATS.md` | les résultats expliqués |

Code lié, hors de ce dossier : `../../../guide.py` (le guide en vol : classes `Guide` et
`GuideEntraine`, construction du dossier JSON des zones) et `../../../mission.py` (options
`--guide`, `--lam`, `--modele-guide`, `--adaptateur`, `--instantanes`).

## Les vols utilisés

Dossiers `experiments/11_mission/<vol>/` de la machine de simulation :
- jeu « ancien », 165 cas : `nominale` (96 cas), `panne` (38) et `autre` (31) ;
- jeu « complet », 102 cas, entrepôt 9033 : `fin_de_mission` (30), `fin_de_mission_v3` (36),
  `controle_code_restaure` (36) ;
- `dossier_complet` (36 cas), vol du 10 septembre qui enregistre aussi les listes brutes ;
- entraînement LoRA : `fin_de_mission` + `fin_de_mission_v3` (66 cas) ; test :
  `controle_code_restaure` + `dossier_complet` (72 cas).

## Les fichiers de résultats

Tous dans `..` ; modèle Qwen2.5-VL-3B sauf mention ; « jeu complet » = 102 cas.
- `resultats.json` : 1re campagne, SmolVLM-500M, 165 cas (`banc.py`).
- `resultats_description.json` : idem, zones décrites en phrases (`banc.py --description`).
- `resultats_qwen7b.json` : Qwen2.5-VL-7B, 165 cas, avant le mélange des numéros (sans valeur).
- `resultats_variantes.json` : 165 cas ; variantes comptage, image, lisible, description,
  raisonnement, notes.
- `resultats_variantes_faits.json` : jeu complet ; lisible, description.
- `resultats_perception.json` : contrôle de perception, 2 cas, 4 versions (`perception.py`).
- `resultats_fewshot.json` (30 cas) et `resultats_fewshot100.json` (100 cas) : description
  contre 2 exemples résolus (few_shot).
- `resultats_texte_seul.json` : jeu complet, non compressé ; texte_seul, texte_seul_indice.
- `resultats_melange.json` : jeu complet ; texte_melange (biais de position).
- `resultats_dossier.json` : jeu complet, non compressé ; dossier_phrases, dossier_json,
  dossier_json_note, dossier_sans_image.
- `resultats_complet.json` : vol `dossier_complet`, 36 cas ; dossier_json.
- `resultats_complet2.json` : 36 cas ; listes, listes_sans_image, tout.
- `resultats_introspection.json` : 15 cas (`introspection.py`).
- `resultats_leviers.json` : jeu complet, 4 bits ; dossier_json, extraire, question_avant.
- `resultats_epure.json` : jeu complet, 4 bits ; ordre, epure.
- `resultats_epure_nc.json` : epure sur 100 cas, 83 %, 5,8 s par question.
- `resultats_trois.json` : jeu complet, 4 bits ; epure_metier, epure_noms, epure_noms_metier.
- `resultats_entrainement.json` : entraînement LoRA, 72 cas de test avant / après (`entraine.py`).

## L'adaptateur LoRA (`../adaptateur_lora/`)

- `adapter_model.safetensors` (14,8 Mo) : les poids appris, de petites matrices de rang 8
  ajoutées aux projections d'attention (q, k, v, o) du modèle de langage de Qwen2.5-VL-3B
  (3,7 millions de paramètres, 0,1 % du modèle). Le modèle de base n'est pas inclus.
- `adapter_config.json` : réglages (rang 8, alpha 16, dropout 0,05, PEFT 0.20.0) et chemin du
  modèle de base.
- `README.md` : fiche générée automatiquement par la bibliothèque PEFT.
- Appris sur 66 cas (2 vols, entrepôt 9033) ; 96 % de bonnes zones sur 72 cas jamais vus du
  **même** entrepôt. En vol : `mission.py --guide entraine --lam <λ>` (classe `GuideEntraine`).

## Comment le lancer

Prérequis : carte graphique NVIDIA (celle du projet : 8 Go, génération Turing) ; Python avec
`torch`, `transformers`, `bitsandbytes`, `peft` (pour l'entraînement), `opencv-python`, `numpy`.
Pas de simulateur. SmolVLM se télécharge seul (`smolvlm`, `smolvlm-2b`) ; Qwen2.5-VL est un
dossier local (défaut de `mission.py` : `~/Documents/qwen2.5-vl-3b`). Il faut aussi les vols
avec leurs instantanés (dossiers `experiments/11_mission/<vol>/`).
Durée : de 0,7 à 8 s par question, soit quelques minutes à un quart d'heure par variante sur
102 cas.

Les json enregistrent des chemins `experiments/11_mission/...` : les scripts étaient lancés
depuis `swarm_qr/`. `PY` est le Python de l'environnement (sur la machine de simulation :
`PY=~/isaac5_env/bin/python`). Les résultats sont toujours écrits dans `12_guide/`.

    cd swarm_qr
    $PY experiments/12_guide/banc.py --missions experiments/11_mission/nominale experiments/11_mission/panne experiments/11_mission/autre --modeles smolvlm smolvlm-2b
    $PY experiments/12_guide/variantes.py --missions experiments/11_mission/fin_de_mission experiments/11_mission/fin_de_mission_v3 experiments/11_mission/controle_code_restaure --modele ~/Documents/qwen2.5-vl-3b --variantes ordre epure --sortie resultats_epure.json
    $PY experiments/12_guide/perception.py --missions fin_de_mission --modele ~/Documents/qwen2.5-vl-3b
    $PY experiments/12_guide/introspection.py --missions <vols> --modele ~/Documents/qwen2.5-vl-3b --cas 15
    $PY experiments/12_guide/entraine.py --entrainement experiments/11_mission/fin_de_mission experiments/11_mission/fin_de_mission_v3 --test experiments/11_mission/controle_code_restaure experiments/11_mission/dossier_complet --modele ~/Documents/qwen2.5-vl-3b

- `banc.py` (même commande que `bash experiments/11_mission/campagne.sh guide`) écrit
  `resultats.json` ; `--description` écrit `resultats_description.json` ; `--quant 4bit`
  compresse le modèle.
- `variantes.py` : l'exemple refait `resultats_epure.json`. `--quant` vaut `4bit` par défaut ;
  non compressé : `--quant aucune --device auto`. Sans `--sortie`, il écrit
  `resultats_variantes.json`. Variantes : clés de `VARIANTES` dans le script.
- `perception.py` attend des **noms** de vols (cherchés dans `experiments/11_mission/`).
- `entraine.py` reprend les vols notés dans `resultats_entrainement.json` et écrit
  `adaptateur_lora/` et ce json.
- Guide en vol (Isaac Sim nécessaire) : `$PY swarm_qr/mission.py --guide smolvlm --lam 1 --sortie <dossier>`
  (1re version) ou `--guide entraine --lam 1` (modèle 3B + adaptateur).

## Résultats en bref

- SmolVLM-500M (photo + plan) : 7 % de bonnes zones, sous le hasard (17 %) et la géométrie
  (42 %), sur 165 cas.
- Qwen2.5-VL-3B, 102 cas : 61 % avec images + faits + 2 exemples ; 79 % avec le dossier complet
  en JSON sans image ; 83 % avec le dossier « épuré », à égalité avec la géométrie (85 %).
- Une règle de 3 lignes (le plus de QR repérés non lus, sinon le plus proche) fait 94 %.
- Après entraînement LoRA : 96 % sur 72 cas jamais vus contre 83 % pour la géométrie
  (p = 0,035), mais sur le même entrepôt 9033 que l'entraînement, et la règle de 3 lignes y
  fait 100 %.
- Le guide n'est pas branché (λ = 0) → détails dans `RESULTATS.md`.
