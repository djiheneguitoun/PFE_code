# fake data — Journaux des 4 vols d'évaluation avec des avis du guide fabriqués

## En une phrase

Une copie des `mission.json` et `brouillon.json` des 4 vols de `../tests of system/`, auxquels on a
ajouté après coup les avis d'un guide, pour avoir des données d'entraînement d'un guide LLM au
format des journaux de `mission.py`.

**Ce ne sont pas des résultats.** Les 4 vols ont été faits **sans guide** (`evaluation.sh` ne
passe pas `--guide`). Les vrais journaux restent dans `../tests of system/`, qui n'a pas été modifié.
Chaque fichier d'ici commence par la clé `"_donnees_fabriquees"`, qui le rappelle.

## Contenu

| Fichier | Rôle |
|---|---|
| `eval_nominal/`, `eval_panne/`, `eval_9019/`, `eval_obstacle/` | `mission.json` et `brouillon.json` de chaque vol, avec les avis |
| `fabrique.py` | le script qui a fabriqué ces fichiers à partir de `../tests of system/` |
| `README.md` | ce fichier |

Les autres fichiers des vols (images, vidéos, carte, journaux ArduPilot) ne contiennent pas d'avis
du guide : ils ne sont pas copiés.

## Ce qui change par rapport aux vrais journaux

Seulement les champs que `mission.py` écrit quand le guide est actif. Tout le reste est identique,
octet pour octet, ce que `fabrique.py` a vérifié.

| Champ | Vrai vol | Ici |
|---|---|---|
| `"guide"` (en-tête) | `""` | `"entraine"` (`"lam"` reste à 1.0) |
| `evenements[]` | pas d'avis | un événement `{"t", "genre": "avis", "drone", "zone"}` à chaque réponse du guide |
| `agents[].decisions[].avis` | `null` | la phrase du dernier avis reçu par le drone, par ex. `"zone 1, cote est ('ANSWER: 1')"` |
| `agents[].decisions[].note` | note géométrique | + 10 si la cible est dans la zone conseillée, + 5 de plus si elle est aussi du côté conseillé |
| `instantanes[].drones[].cible.note` | note géométrique | la note de la décision ci-dessus |

## Comment les avis sont fabriqués

`fabrique.py` rejoue les règles du code, cycle par cycle, sur les vrais instants des cycles
(`codes_par_t`) :
- **Cadence** (`demande_avis`, `mission.py`) : chaque drone vivant demande un avis, la réponse est
  relevée 5 s plus tard, ce qui écrit l'événement, puis une nouvelle demande part. Un drone en
  panne ne demande plus rien.
- **Réponse** : la zone qui contient le plus de QR non lus, c'est-à-dire la « bonne réponse » que
  `mission.py` calcule lui-même dans `instantanes[].verite`, prise dans l'instantané le plus
  récent au moment de la demande. Le côté est celui des faces de la zone, comme dans
  `GuideEntraine.conseille` (`guide.py`).
- **Décision** (`decide`, `mission.py`) : chaque décision recopie le dernier avis reçu par son
  drone. Les 3 premières décisions de chaque vol (t ≈ 86 s) gardent `"avis": null`, car le guide
  n'a pas encore répondu.
- **Note** (`Cerveau.note`, `planning.py`) : bonus λ × 10 et λ × 5, avec λ = 1.

## Chiffres

| Vol | Événements « avis » | Décisions | dont avec avis | dont avec bonus |
|---|---|---|---|---|
| `eval_nominal` | 168 | 82 | 79 | 8 |
| `eval_panne` | 114 (drone 1 : 22, panne à 200,2 s) | 65 | 62 | 7 |
| `eval_9019` | 126 | 59 | 56 | 4 |
| `eval_obstacle` | 153 | 78 | 75 | 15 |

## Limites

- **Les décisions sont celles du vol sans guide.** Avec un vrai guide, le bonus aurait pu faire
  choisir au drone une autre cible dans la zone conseillée, donc d'autres trajets. Ici, seuls
  l'avis et la note changent.
- **Le guide a toujours raison.** Le guide entraîné de l'étape 12 trouvait la bonne zone dans
  96 % des cas.
- **Les zones viennent des instantanés**, écrits toutes les 10 s et calculés pour le drone 0. Elles
  servent pour les 3 drones, alors qu'en vol chaque drone calcule ses propres zones au moment de
  la demande.
- **Le guide est supposé répondre en moins de 5 s.** Les durées de calcul (`cycles_t`, `cycles`,
  `mur_min`) sont celles du vol sans guide.

## Utiliser ces données pour entraîner un guide

Pour chaque événement `"avis"` :
- **la question** est le texte que `guide.texte_pour_le_guide` construit à partir de l'instantané
  utilisé : le plus récent au moment de la demande, soit environ 5 s avant l'événement ;
- **la réponse attendue** est `ANSWER: <n>`, où `n` est le numéro dans `"zone"`.

Comme la réponse est toujours `instantanes[].verite`, le modèle apprendra cette règle : la zone
qui contient le plus de QR non lus.

## Refaire les fichiers

Depuis ce dossier (Python 3.10+, `numpy`) :

    python fabrique.py
