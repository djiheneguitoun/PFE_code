# 12 — Le guide vision-langage : résultats

## La question

Un modèle vision-langage (une IA qui lit images et texte) peut-il dire aux drones quelle zone
visiter mieux que le planificateur géométrique ? On ne le branche que s'il bat le hasard et,
surtout, la géométrie seule.

## Méthode (en bref)

- Le guide conseille seulement : les cibles de la zone conseillée gagnent λ × 10 points, plus
  λ × 5 si leur côté d'abordage est le bon. λ = 0 : système purement géométrique (repli
  garanti). Il tourne en arrière-plan (une demande toutes les 5 s par drone).
- Zones candidates : les cibles du planificateur groupées sur une grille de 3 m, les 6 plus utiles.
- Banc hors ligne : des instantanés enregistrés pendant les vols, toutes les 30 s (plan vu de
  dessus, photo de chaque drone, état de la carte) ; la bonne réponse est connue après coup :
  la zone qui contient le plus de QR non lus. Un cas = un drone vivant × un instantané.
- Score : % de bonnes zones. Comparaisons appariées (mêmes cas : gagnés / perdus) ; p =
  probabilité que l'écart soit dû au hasard, p < 0,05 = prouvé.

## Résultats

### 1. Première campagne : SmolVLM-500M (jeu « ancien », 165 cas)

SmolVLM-500M-Instruct (500 millions de paramètres, environ 1,8 Go de mémoire graphique) voit
la photo et le plan (zones entourées de rouge, numérotées) et reçoit deux questions courtes :
« quelle zone ? », puis « quel côté : nord, sud, est, ouest ? » (une seule longue question
donnait « 2. » sans côté).

| qui répond | bonne zone | bon côté |
|---|---|---|
| hasard | 17 % | 25 % |
| géométrie | 42 % | — |
| toujours « ouest » | — | 96 % |
| SmolVLM-500M | 7 % | 42 % |
| idem + zones en phrases | 13 % | 12 % |

- 0,7 s par question, 1,7 Go (avec phrases : 0,8 s, 1,9 Go). La version de 2,2 milliards n'a
  pas été mesurée : téléchargement (4,5 Go) arrêté à 1,7 Go après 3 h.
- Le côté est sans valeur ici : racks tous orientés pareil, QR restants presque tous à l'ouest.
- Bilan : pire que le hasard, six fois moins bien que la géométrie ; lire un plan à cercles
  numérotés est hors de sa portée. Non branché ; aucun modèle ne le sera sous 42 % ici.

### 2. Deuxième campagne (9-10 septembre) : Qwen2.5-VL, 7 puis 3 milliards

**Défaut du banc corrigé d'abord** : les zones étaient numérotées par utilité décroissante,
donc répondre « 1 » valait la géométrie. Numéros désormais tirés au hasard. Les 22 % du modèle
de 7 milliards, mesurés avant, ne veulent rien dire.

**Deux jeux de cas, à ne jamais comparer entre eux :**

| | ancien | complet |
|---|---|---|
| cas | 165 | 102 |
| vols | du 08-09 | du 09-09 |
| comptages par zone | absents | présents |
| instant médian | 197 s | 146 s |
| cas après 250 s | 44 % | 0 % |
| codes non lus (méd.) | 14 | 99 |
| dans la meilleure zone | 8 | 34 |
| géométrie | 42 % | 85–87 % |

C'est l'examen qui change : en fin de mission il reste une quinzaine de codes éparpillés et
tout s'effondre ; le jeu complet ne couvre que la phase de lecture.

**Variantes, jeu complet (3 milliards) :**

| qui décide | bonne zone | s/question |
|---|---|---|
| hasard | 17 % | — |
| les deux images seules | 19 % | 0,8 |
| plan redessiné, sans texte | 25 % | 0,9 |
| plan redessiné + faits en phrases | 40 % | 1,0 |
| photo + faits enrichis, non compressé | 58 % | 3,3 |
| idem + une phrase de métier | 55 % | 3,4 |
| idem, zones en ordre mélangé | 55 % | 3,4 |
| plan + faits + 2 exemples résolus | 61 % | 3,5 |
| noter chaque zone de 0 à 10 | 16 % | 6,0 |
| géométrie | 85–87 % | instantané |
| règle de 3 lignes | 94 % | instantané |

- Jeu ancien : 17, 25, 48, 36 et 16 % (images, plan redessiné, faits, raisonnement, notes),
  géométrie 42 % ; même classement.
- Exemples résolus : +21 points sur 100 cas (27 gagnés, 6 perdus, p = 0,0003) ; sur 30 cas,
  p = 0,45 : il fallait les 100.

**Cause 1 : il lit mal un plan, et la compression n'y est pour rien.** Six questions à réponse
calculée (compter, plus à gauche, à droite, en haut, en bas, voisin le plus proche) :

| 3 milliards | plan d'origine | redessiné | mémoire |
|---|---|---|---|
| 4 bits | 1/9 | 5/9 | 2 463 Mo |
| 4 bits, vision 16 bits | 1/9 | 5/9 | 4 264 Mo |
| 8 bits | 1/9 | 5/9 | 4 184 Mo |
| non compressé | 1/9 | 4/9 | 6 748 Mo |

Mêmes erreurs partout, non compressé le pire : AWQ, GPTQ ou GGUF n'y changeraient rien. Les
cercles fins de 20 px à chiffre de 8 px étaient illisibles ; le redessin donne 5/9 au lieu de 1/9.

**Cause 2 : il connaît la règle, se trompe sur les nombres** : toujours « la zone 5, parce
qu'elle a le plus de codes non lus », avec un nombre qui n'existe pas quand il a tort. Une
phrase de métier ne change rien (2 gagnés, 5 perdus).

**Cause 3 : biais de position** : jamais « 1 » sur 102 cas, alors que 1 était juste 15 fois.
En ordre mélangé, il répond 1 quatorze fois et trouve 6 de ces 15 cas. Biais prouvé, mais le
corriger ne monte pas le score (55 contre 58 %).

**Règle de 3 lignes, 94 % contre 85 %** : la zone qui a le plus de codes repérés non lus, la
plus proche en cas d'égalité. Un code repéré est une lecture presque certaine, une surface
jamais vue un espoir ; la somme des utilités mélange les deux (lecture : 30 points ; groupe de
surfaces plafonné à 30 : à égalité, la distance tranche). Comparaison de classements de zones,
pas de décisions en vol : formule inchangée, gain en vol non mesuré.

**Mémoire** : en vol, le simulateur prend 4,6 Go des 8 Go. 7 milliards en 4 bits : 6,3 Go,
impossible ; 3 milliards en 4 bits : 2,5 Go, possible ; en 16 bits, ne se charge pas.

**Jamais consulté au bon moment** : sur 612 zones proposées, aucune zone d'exploration, et
tous les cas tombent entre 96 et 208 s. Le guide n'est jamais interrogé quand on ignore encore
où sont les codes, là où il aurait un avantage.

### 3. Le dossier complet de la carte (102 cas)

Au lieu de 4 chiffres par zone, tout ce que l'enregistrement contient, sans une once de vérité
du simulateur (environ 4 100 caractères) : instant, codes lus, position et cap des 3 drones,
emprise des 5 racks, et par zone centre, rayon, 4 comptages, genres de cibles, côté des faces,
distance et direction. Photo jointe, plus de plan.

| ce que reçoit le modèle | bonne zone |
|---|---|
| plan + 4 chiffres par zone | 40 % |
| photo + faits enrichis | 58 % |
| photo + dossier en phrases | 76 % |
| photo + dossier JSON | 78 % |
| idem + note du planificateur | 78 % |
| dossier JSON, sans image | 79 % |
| géométrie / règle de 3 lignes | 85 % / 94 % |

- Le contenu fait tout : +38 points (la crainte d'un trop-plein était fausse).
- Le format ne compte pas : JSON contre phrases, +10 / −8, p = 0,81 (JSON gardé, automatique).
- La photo n'apporte rien : +6 / −5, p = 1,00. Il travaille comme un modèle de texte.
- Il ne sait pas prendre un maximum : avec la note du planificateur (85 % par construction), il
  reste à 78 % et ne suit la géométrie que 73 fois sur 102. Erreur type : « la zone 4, parce
  qu'elle a le plus de groupes de frontière inexplorés », un nombre nul partout.
- Biais de position atténué : 5 réponses « 1 » pour 15 attendues.
- Plafond : 61 → 79 %, à neuf points de la géométrie et quinze de la règle de 3 lignes :
  toujours au-dessous, le guide n'est pas branché. Pour le mémoire : un modèle de 3 milliards,
  nourri du dossier complet, retrouve 79 % des choix du planificateur sans connaître sa formule.

### 4. Listes brutes (vol `dossier_complet`, 10 septembre, 36 cas)

Vol sur 9033 (113 codes sur 114, 0 chute) enregistrant tout : grille, 149 faces lues, pistes,
512 cibles candidates, frontières, réservations, cible de chaque drone. Géométrie 92 %. La
carte graphique (Turing, sans attention rapide, mémoire en carré du texte) déborde ses 8 Go
dès 5 400 jetons : lignes compactes (une décimale, sans noms de codes), 25 cibles sur 512 au
niveau « tout » (dit au modèle).

| ce que reçoit le modèle | jetons | bonne zone |
|---|---|---|
| dossier des zones | 1 550 | 83 % |
| + listes brutes | 4 000 | 78 % |
| idem sans photo | 3 400 | 53 % |
| + 25 meilleures cibles | 4 500 | 72 % |

- Listes : +3 / −5 (p = 0,73) ; cibles : +1 / −3 (p = 0,62). Rien de gagné, tendance
  négative ; il cite « le plus de frontières inexplorées », nul partout.
- Sans photo avec un long texte : −25 points (0 gagné, 9 perdus, p = 0,004), alors qu'elle ne
  comptait pas avec un texte court.
- 36 cas : chiffres fragiles, mais il plafonne clairement avec les comptages par zone.

### 5. Introspection, puis leviers (10 septembre, 102 cas, géométrie 85 %)

Après sa réponse, on lui demande quels champs et nombres il a utilisés (15 cas). Juste, il
cite le bon champ et compare juste. Faux, il choisit une zone à zéro code repéré à cause d'un
gros nombre sans valeur (« le total des cibles candidates est plus grand, 33 contre 27 », ou
les frontières, nulles partout), et se contredit parfois, la réponse précédant le raisonnement.

| levier (dossier JSON + photo) | bonne zone |
|---|---|
| référence, 4 bits | 75 % |
| recopier les chiffres, puis choisir | 57 % |
| question avant les données | 55 % (10 sans réponse) |
| champ décisif en tête | 76 % |
| 4 distracteurs retirés (« épuré ») | 83 % |
| vote à cinq | en cours |
| géométrie | 85 % |

- Il recopie les 6 nombres sans erreur (« zone 1 : 15, zone 2 : 5, zone 3 : 12, zone 4 : 7,
  zone 5 : 6, zone 6 : 6 ») puis répond zone 6 : la faute est la comparaison, pas la lecture.
  Ce levier coûte 18 points (+3 / −21, p < 0,001).
- Retirer total de cibles, genres, rayon et frontières : +9 points (+13 / −4, p = 0,049).
  L'ordre seul ne fait rien (+9 / −7).
- L'épuré fait jeu égal avec la géométrie (14 gagnés, 16 perdus), en 3 s au lieu d'un instant.
- 4 bits contre non compressé : 74,5 contre 78,4 %, +6 / −10, p = 0,45 : non prouvé, 3 points
  d'écart restent possibles.

### 6. Trois derniers leviers (102 cas, 4 bits, référence épuré 83 %)

| ajouté à l'épuré | bonne zone | cas par cas |
|---|---|---|
| 2 phrases de métier | 71 % | +3 / −16, p < 0,01 |
| noms de champs parlants | 66 % | +2 / −20, p < 0,01 |
| les deux | 53 % | +3 / −34, p < 0,01 |

Tout ajout fait baisser. Les noms parlants créent de nouveaux distracteurs (« le plus de
surfaces non vérifiées, 23 ») ; les phrases de métier n'apprennent rien à un modèle qui connaît
déjà la règle et détournent son attention. Leçon : il décide sur le plus grand nombre qu'il
voit ; moins il voit de nombres, mieux il choisit.

### 7. Petit entraînement QLoRA (72 cas jamais vus)

Modèle en 4 bits gelé ; couches LoRA de rang 8 sur l'attention du modèle de langage (3,7
millions de paramètres, 0,1 %) ; réponse « ANSWER: n » apprise depuis le dossier épuré, sans
image. Entraînement : 2 vols du 9 septembre, 66 cas × 6 numérotations = 396 exemples,
2 époques, perte 0,032 → 0,004. Test : vol de contrôle et vol `dossier_complet`. Adaptateur
de 15 Mo, retirable sans toucher au modèle.

| 72 cas de test | bonne zone |
|---|---|
| zone la plus proche | 25 % |
| géométrie | 83 % |
| modèle avant | 83 % |
| modèle après | 96 % |
| règle de 3 lignes | 100 % |

Avant / après : +11 / −2 (p = 0,022) ; contre la géométrie : +12 / −3 (p = 0,035). Première
variante au-dessus du planificateur ; ses 3 erreurs portent sur la même zone vraie.

## Conclusion

- Guide non branché : λ = 0, formule de décision inchangée.
- Meilleurs scores successifs : 7 % (SmolVLM), 61 %, 79 % (dossier complet), 83 % (épuré, à
  égalité avec la géométrie), 96 % après entraînement, sur le même entrepôt.
- Le contenu compte, pas le format ; ajouter des nombres fait baisser ; le point faible est la
  comparaison des nombres. La littérature de 2026 le confirme : le raisonnement spatial reste
  un point faible des modèles vision-langage, ouverts comme fermés (OmniSpatial, 8 400
  questions ; revue « Spatial intelligence in vision-language models »).
- Pistes non faites : questions que la carte ne sait pas poser (étagère vide, allée bouchée,
  pancarte), avec une vérité du simulateur ; exemples résolus rendus permanents par prompt
  appris ou LoRA (la section 7 est un premier essai, sans exemples) ; plafond d'un modèle de
  pointe en nuage ; modèle de 2,2 milliards.

## Limites : ce que ce test ne prouve pas

- **Entraînement et test LoRA sur le même entrepôt, 9033** : le modèle a pu apprendre cet
  entrepôt autant que la règle. Le 96 % ne vaudra qu'après un test sur un entrepôt jamais vu
  (vols sur d'autres graines, avec l'enregistrement complet).
- **Une règle de 3 lignes fait 100 % sur ces cas** : le modèle a très probablement appris cette règle.
- La règle de 3 lignes est comparée sur le classement des zones, pas sur la décision en vol :
  la formule de décision est inchangée, et le gain éventuel en vol reste à mesurer par un vol.
- Phase de lecture seulement (96 à 208 s, aucune zone d'exploration).
- Sur 36 cas (section 4), les chiffres restent fragiles.
- Le côté n'a pas de sens dans cet entrepôt (« toujours ouest » : 96 %).

## Fichiers de résultats et figures

Correspondance complète fichier → expérience : `README.md` (ce dossier). Sections 1-2 :
`../resultats.json`, `../resultats_variantes.json`, `../resultats_perception.json`… ; section 3 :
`../resultats_dossier.json` ; section 4 : `../resultats_complet2.json` ; sections 5-6 :
`../resultats_leviers.json`, `../resultats_epure.json`, `../resultats_trois.json` ; section 7 :
`../resultats_entrainement.json`, `../adaptateur_lora/` et la figure
`../../../docs/figures/fig_guide.png`.
