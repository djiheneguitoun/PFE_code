# 11 — Le cerveau et les missions à 3 drones (étape 5) : résultats

## La question

Le drone sait lire, se souvenir et repérer de loin ; il lui manque de savoir **où aller**.
Chaque drone choisit ses cibles sur sa carte (jamais sur le plan), 3 drones se partagent le
travail, la mission se termine seule. La porte de validation est une mécanique : **aucun
blocage, aucun doublon, une dispersion au départ, une panne absorbée, une fin propre**.

## Méthode (en bref)

- **Trois sortes de cibles, tirées de la carte.** *Lire* : une piste (QR repéré de loin, pas
  encore lu) ; pose à 2 m devant, du côté où la carte montre de l'espace libre, pas du côté
  d'où le QR a été aperçu (souvent de biais : à 60°, le lecteur ne lit pas). *Couvrir* : une
  surface occupée, à hauteur de vol, jamais regardée d'assez près du côté libre ; un mètre de
  surface par cible ; un carton repéré par le détecteur y vaut 3 fois un mur nu (premier usage
  du canal sémantique). *Explorer* : une frontière connu/inconnu, groupée par 2 m.
- **Une note par cible** : utilité (30 pour une piste, 1 par cube de surface, 0,15 par case de
  frontière), moins 1 par mètre de trajet, moins 1 000 si un coéquipier l'a réservée (donc
  exclue), moins 40 si elle est à moins de 4 m d'un coéquipier en vol. Les 6 meilleures à vol
  d'oiseau reçoivent un vrai chemin ; une cible sans chemin est écartée. Une piste visitée
  3 fois sans lecture va en liste noire.
- **Trois rythmes** : le contrôleur à chaque pas de physique, l'observation 5 fois par
  seconde, la décision quand un drone n'a plus de cible. Devant une cible de lecture, le drone
  tient sa pose 2 s (10 images).
- **L'équipe** : une carte partagée ; chaque drone annonce sa cible. Une réservation non
  renouvelée expire, un drone muet perd les siennes : la panne est gérée sans une ligne de code
  qui la surveille. Croisements : à moins de 2,5 m, le drone au plus grand numéro cède le
  passage ; à moins de 1,5 m, tous s'arrêtent.
- **La fin** : plus personne n'a de cible pendant 30 s, ou budget de temps épuisé.
- **Le cerveau testé avant de voler** sur dix cartes jouets dont on connaît la réponse : une
  piste proche bat une frontière lointaine ; une zone réservée est évitée ; entre deux
  frontières égales, la plus proche gagne ; tout exploré, la liste est vide ; une surface jamais
  regardée attire puis disparaît une fois couverte ; un carton vaut plus qu'un mur nu ; trois
  visites sans lecture écartent la piste ; une pose dans la marge d'un obstacle se rapproche du
  panneau ; l'avis du guide fait pencher la balance.
- **Le jugement** : `analyse.py`, après le vol, avec la vérité de la simulation.

## Résultats

### 1. Vols d'essai (5 vols, 1 ou 2 drones, 3 à 5 min)

| Problème observé | Règle retenue |
|---|---|
| 23 cibles de lecture atteintes, aucune lue : pose dans la direction d'aperçu, souvent à 60°. Avec « le côté libre le plus net », le drone se mettait face au bout vide du rack. | Un panneau se lit perpendiculairement au grand axe de sa structure, du côté libre le plus proche. 7 codes à 78 s contre 1 ; 74 codes en 3 min avec un drone. |
| Un cadre vu de biais dont le rayon passe par un trou du rack donne un point trop profond. | Piste créée seulement si sa distance vaut 0,45 à 1,8 fois celle déduite de la taille du cadre. |
| Une piste pouvait être visitée sans fin. | 3 chances : à 2 m, à 1,2 m (petites étiquettes), puis de l'autre côté ; tentatives comptées par proximité (une piste bouge de quelques cm). |
| Un QR du décor, « 409518 », est entré dans l'inventaire. | Seuls les codes au format attendu y entrent ; les autres sont comptés à part. |

Dernier vol d'essai (2 drones, 5 min) : 5 vérifications passées, 89 codes sur 114, aucun point
dans une structure, 1,02 m au plus près entre drones.

### 2. Missions à 3 drones : historique des corrections

Huit missions nominales avant un code stable, chacune gardée (`nominale_v0` à `nominale_v8`,
plus `diagnostic_3_drones_400s`). Toutes lisent 110 à 114 codes sur 114, 90 % en 160 à 200 s ;
seule la sécurité change.

**Régime à 3 drones** : la simulation tourne sept fois plus lentement que le temps réel : 1,3 s
de calcul par cycle de 0,2 s, dont 0,75 s de physique (mesuré). Le pilote automatique suit ce
rythme, mais le contrôle en vitesse, validé à un drone avec un gain de 0,9, oscille à trois :
±0,5 à ±0,7 m après quelques minutes, inclinaisons de 20 à 35°, jusqu'au contact avec un rack.
Gain de mission 0,5, approche 0,6 m/s, transit 1 m/s.

| N° | Problème observé | Correction |
|---|---|---|
| 1 | Deux drones sur la même cible, à 47 cm : la pénalité de réservation (25) restait sous l'utilité d'une piste (30). | Pénalité portée à 1 000 (cible exclue) ; cible proche d'un coéquipier : d'abord 3 m et +20, puis 4 m et +40 ; arrêt de tous à 80 cm. |
| 2 | Drone coincé dans le rack du milieu, à 2 m : cible à 5,5 m, chemin au-dessus du rack, traversée commencée avant la prise d'altitude. | Plus de survol : un obstacle connu entre 2 m sous le drone et 0,85 m au-dessus bloque la colonne ; altitude prise sur place avant le trajet. |
| 3 | Les recalculs de chemin remettaient la patience à zéro : recalcul sans fin. | Immobile (moins de 30 cm) 15 s après l'affectation, ou 30 recalculs : la cible est abandonnée et écartée. |
| 4 | Un drone sans cible dérivait jusque dans un rack ; un autre s'est posé au sol. | Un drone sans cible, ou en panne, tient activement sa position. |
| 5 | Le détecteur de blocage jugeait « immobile » un drone qui venait de tenir sa place : 1 622 cibles abandonnées en une mission. | Compte depuis l'affectation ; un drone sans cible décide au plus toutes les 2 s. |
| 6 | Deux drones se sont touchés malgré la règle d'arrêt (l'arrêté dérive, l'autre continue). | Coéquipiers = obstacles mobiles de 2 m dans les chemins ; céder, c'est tenir sa place. |
| 7 | Un drone a heurté le panneau au sommet d'un rack (5 m) en volant à 4,8 m, s'est retourné, posé sur le rack sans être déclaré mort. | Plafond de vol 4,5 m ; incliné à plus de 70° = déclaré en chute. |
| 8 | Descente de 4,5 à 2 m à 1,2 m d'un rack en dérivant d'un mètre ; traversée d'une zone inconnue dans un rack pas encore vu. | Changement d'altitude à 1,5 m de tout obstacle, segment vérifié ; case inconnue = 20 fois une case libre ; tolérance d'arrivée 35 cm. |
| 9 | Oscillation du contrôleur : trois chutes tardives, sans autre drone à moins de 5 m. | Gain 0,5, transit 1 m/s. |

Le vol de diagnostic de 400 s (règles 6 à 8) est propre : 114 codes sur 114, aucune chute,
aucun point dans un rack, 1,70 m au plus près, inclinaison maximale 30°.

Deux détails du simulateur, sans effet sur les vols : Pegasus lance le pilote ArduPilot deux
fois par drone (6 fenêtres pour 3 drones, dont 3 mortes) ; une observation coûte 30 à 150 ms
par image selon le partage de la carte graphique entre rendu et détecteur.

### 3. Campagne de mise au point : trois missions, même code

3 drones, 600 s simulées dont 95 s de décollage, détecteur branché, sans guide.

| Mesure | Nominale 9033 | Panne drone 1 à 200 s | 9019, jamais vu |
|---|---|---|---|
| Vérifications | **5 sur 5** | **5 sur 5** | 3 sur 5 |
| Codes lus | 110 / 114 | **114 / 114** | 61 / 65 |
| Codes inventés | 0 | 0 | 0 |
| 50 / 80 / 90 % à | 137 / 169 / 185 s | 130 / 156 / 163 s | 150 / 169 / 178 s |
| Entrepôt connu | 89 % | 88 % | 89 % |
| Chutes | **aucune** | **aucune** | 3, à 407, 409, 413 s |
| Inclinaison max | 36° | 15° | 103° |
| Points dans un rack | **0** / 7 707 | **0** / 5 707 | 8 / 4 849, après les chutes |
| Distance min entre drones | 2,6 m | 1,96 m | 2,22 m |
| Pas d'attente de priorité | 1 | 32 | 27 |
| Calcul | 57 min | 53 min | 36 min |

- **Les deux premières** prouvent la mécanique : aucun blocage, jamais deux drones sur la même
  cible, trois départs dans trois directions, fin propre, aucun contact. Panne absorbée : 52
  décisions des autres ensuite, sa zone reprise 20 s après, codes 108 puis 114. Inventaire lu à
  96 % puis 100 %, 90 % en trois minutes après le décollage.
- **La troisième reste ouverte** : vers 300 s, les trois drones, à 5 à 12 m les uns des autres,
  passent ensemble d'une inclinaison sous 18° à des oscillations de 40 à 55°, et tombent 100 s
  plus tard. Ni collision, ni choix des cibles : trois pilotes indépendants qui se dérèglent
  ensemble, un événement de la simulation, **non compris**. Les 61 codes étaient lus avant.
- **Vol refait sur 9019** (`autre_v2`, même code, cycles datés) : 5 sur 5, **63 / 65**, aucune
  chute, inclinaison max 20°, 0 point dans un rack sur 7 692, 2,45 m au plus près, 90 % à
  166 s ; cycles stables (1,25 s en médiane). Le dérèglement est intermittent ; `../cycles.py`
  permettra de le dater s'il revient.

### 4. Évaluation finale : les 4 vols de `tests of system/`

Chiffres tirés de leurs `resultats.json`, sauf mention « mission.json ». 3 drones, 600 s au
plus, détecteur branché, vidéo.

| Mesure | Nominal 9033 | Panne drone 1 à 200 s | Entrepôt 9019 | Obstacle à 200 s |
|---|---|---|---|---|
| Vérifications | 5 sur 5 | 4 sur 5 | 5 sur 5 | 5 sur 5 + obstacle évité |
| Codes lus | 110 / 114 | 110 / 114 | 60 / 65 | 110 / 114 |
| Codes inventés | 0 | 0 | 0 | 0 |
| 50 / 80 / 90 % à | 154 / 180 / 196 s | 131 / 154 / 187 s | 124 / 144 / 165 s | 135 / 192 / 206 s |
| Entrepôt connu | 87,6 % | 86,0 % | 89,3 % | 88,0 % |
| Fin | 367,9 s | 317,3 s | 299,9 s | 342,9 s |
| Points dans un rack | 0 / 4 215 | 0 / 2 873 | 0 / 3 195 | 0 / 3 840 |
| Distance min entre drones | 2,08 m | 3,82 m | 2,53 m | 1,60 m |
| Pas d'attente de priorité | 2 | 0 | 0 | 10 |
| Écarts des premières cibles | 23,5 / 19,7 / 15,6 m | idem | 20,8 / 7,0 / 20,9 m | 26,3 / 19,7 / 17,7 m |
| « Lire » : choisies / atteintes / lues | 72 / 51 / 49 | 62 / 59 / 36 | 47 / 42 / 21 | 54 / 49 / 37 |
| « Couvrir » : choisies / atteintes | 10 / 4 | 3 / 3 | 12 / 11 | 24 / 22 |
| Cibles abandonnées | 22 | 0 | 3 | 4 |
| Calcul | 32,9 min | 25,8 min | 24,9 min | 29,8 min |

- **Panne** : drone 1 arrêté à 200,2 s ; les autres prennent 24 décisions ensuite et reviennent
  3 fois dans sa zone (à moins de 4 m de sa dernière cible), dès 281,6 s.
- **Obstacle** : bloc de 1 × 1 × 2 m (x de −5,46 à −4,46 m, y de 3,5 à 4,5 m, mission.json)
  apparu à 200,2 s ; sur 2 136 points ensuite, 0 dedans, 0 à moins de 30 cm, au plus près
  0,57 m. Abandons « chemin coupé » à 211,2, 219,6 et 224,4 s (un par drone), puis à 324,8 s.
- **Abandons** : nominal 7 « chemin coupé » et 15 « immobile » (entre 208,8 et 347 s) ;
  9019 : 2 « chemin coupé » (152,4 et 152,8 s) et 1 « trop de recalculs » (295,2 s).
- Aucune cible « explorer » choisie ; aucun événement « chute » (mission.json).
- Coût par observation : détecteur 22,9 à 176,2 ms selon le drone (le drone 0 toujours le plus
  lent, 163,9 à 176,2 ms), lidar 44,9 à 78,2 ms, décodage 1,6 à 5,5 ms, couverture 2,2 à 2,4 ms.
  Cycle de calcul (mission.json) : 1,34 à 1,36 s en médiane, 4,1 s au plus.

## Conclusion

- La mécanique tient dans la campagne comme dans l'évaluation : aucun blocage, aucun doublon,
  dispersion au départ, fin propre.
- La panne est absorbée sans code spécial (campagne : zone reprise en 20 s, 108 puis 114
  codes ; évaluation : les autres drones continuent et reviennent 3 fois dans sa zone).
- Inventaire lu à 92 à 100 %, aucun code inventé, 90 % des codes en 163 à 206 s simulées.
- Hors le premier vol sur 9019 (trois chutes) : aucun point dans un rack, 1,60 m au moins
  entre drones, obstacle imprévu contourné à 0,57 m au moins.

## Limites : ce que ces tests ne prouvent pas

- Les chutes simultanées du premier vol sur 9019 (trois pilotes qui se dérèglent ensemble) ne
  sont pas comprises ; le dérèglement est intermittent et ne s'est pas reproduit au vol refait.
- Simulation uniquement.

## Fichiers de résultats et figures

Dans chaque `../tests of system/eval_*/` : `resultats.json` (bilan), `mission.json`
(journal), `codes_dans_le_temps.png`, `carte_finale.png`, `video/mission.mp4` (et
`sud_central.mp4`, `sud_grande.mp4`, `lecteur.mp4`), `ardupilot_logs/` (lisibles avec
`../oscillation.py` et `../journaux_ardupilot.py`), `instantanes/` (carte et caméras toutes
les 10 s).
