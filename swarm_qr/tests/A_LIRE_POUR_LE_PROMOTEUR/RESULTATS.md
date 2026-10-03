# tests — Les tests unitaires du système final : résultats

## La question
Prises une à une, les pièces du système final se comportent-elles comme prévu, dans des
situations dont on connaît la bonne réponse ? Et la méthode de référence de Pore et al.
fait-elle bien ce que dit l'article, sans rien emprunter à notre système ?

## Méthode (en bref)
- Exécution par le coordinateur le 3 octobre 2026, sous Windows : Python 3.13.3, numpy 2.2.6,
  OpenCV 4.12.0, pytest 9.0.3.
- Commande, depuis la racine du projet : `python -m pytest swarm_qr/tests -q`.
- Aucun simulateur : un faux drone, un monde de boîtes vu par un lidar exact, des cartes jouets
  écrites à la main, les plans d'entrepôt calculés à partir de la graine, des images de QR
  fabriquées par OpenCV.

## Résultats
Sortie de pytest : **« 89 passed in 24.86s »** — 89 tests réussis sur 89, aucun échec, aucun
test sauté.

| Fichier | Pièce vérifiée | Tests | Réussis |
|---|---|---|---|
| `test_control.py` | contrôleur (étape 3) | 14 | 14 |
| `test_detecteur.py` | détecteur (étape 7) | 1 | 1 |
| `test_mapping.py` | carte (étape 4) | 31 | 31 |
| `test_planning.py` | cerveau (étape 5) | 14 | 14 |
| `test_pore.py` | référence Pore et al. | 29 | 29 |
| total | | 89 | 89 |

### Le contrôleur de vol — `../test_control.py` (14 tests)
Sur un faux drone (réponse en vitesse retardée, rotation limitée à 0,5 rad/s, mur
infranchissable possible), les tests prouvent que :
- **il arrive** : une pose à 3,2 m est atteinte avec une erreur sous 15 cm, un cap à moins de
  0,09 rad et une tenue d'au moins 0,5 s ; un trajet par deux points de passage est suivi
  (phase « transit », longueur comptée 13 m) ;
- **il abandonne à bon escient** : bloqué contre un mur, il abandonne pour « bloque » après 3 s
  sans progrès (en moins de 10 s, sans attendre son budget de 60 s) ; avec une grande patience,
  c'est le budget de 5 s qui le fait abandonner (« delai ») ; un virage lent de 3 rad sur une
  cible à 30 cm n'est pas pris pour un blocage ;
- **il tient sa position** : poussé de 50 cm après l'arrivée, il revient à moins de 15 cm en
  6 s ; après un abandon, il reste à moins de 15 cm de son point d'arrêt au lieu de dériver ;
- **les trois façons de s'arrêter comparées à l'étape 3** sont testées : « couper la vitesse »
  dépasse la cible et ne revient pas, comme en vol (le faux drone reçoit une inertie proche de
  celle mesurée dans `05_sitl` : 1,06 m de glissade depuis 1 m/s) ; « autopilote » envoie une
  consigne de position au pilote ; la loi retenue (vitesse proportionnelle à la distance) sert
  dans tous les autres tests ;
- **le cap** est commandé en vitesse de rotation sur le cap vrai (erreur finale sous 0,09 rad) ;
  sans mesure du cap vrai, le cap absolu est envoyé ;
- **plusieurs drones** volent dans la même boucle (`control.pas`) : deux cibles atteintes en
  moins de 60 s simulées ;
- un tick sans consigne ne fait rien, et le budget par défaut vaut deux fois le temps du trajet
  plus 20 s (51,3 s dans le cas testé).

### Le détecteur appris — `../test_detecteur.py` (1 test)
- Un cadre de (100, 200) à (160, 250) pixels donne le centre (130, 225) et un côté de 50 px,
  le plus petit des deux côtés. `../../observation.py` s'en sert en mission : le centre pour
  viser le point 3D du QR, le côté pour estimer sa distance.
- Le test prouve aussi que `detecteur.py` s'importe sans `ultralytics` ni carte graphique.

### La carte partagée — `../test_mapping.py` (31 tests)
- **Taille et vitesse** : grille de 84 × 128 × 24 cases de 25 cm, moins de 2 Mo ; ajouter un
  tour de lidar et un champ de caméra prend moins de 150 ms.
- **Occupation, exactitude** : devant un mur connu, plus de 95 % du vide devant est connu et
  tout est libre, plus de 95 % de la face est occupée, et aucune case n'est inventée derrière
  le mur ; un faux écho isolé ne fait pas basculer une case libre.
- **Couverture** (ce qu'une caméra a vu d'assez près pour lire un QR) : elle suit l'enveloppe
  de lecture de l'étape 2, de 1,5 à 4 m de distance apparente (couvert à 2,5 et 3,9 m de face,
  pas à 1,2 m, ni à 5 m, ni à 4,4 m apparents) ; elle retient de quel côté on a regardé ; elle
  s'arrête au mur.
- **Panneaux et pistes** (une piste est un QR repéré de loin, pas encore lu) : deux lectures à
  10 cm font un seul panneau ; les deux faces d'un carton restent deux panneaux ; une troisième
  « face » est refusée ; deux repérages à 20 cm font une piste, à 50 cm deux pistes ; une
  lecture efface la piste voisine ; les positions absurdes (1e6 m, NaN) sont refusées.
- **Équipe** : une réservation de 10 s expire seule ; un drone muet depuis plus de 5 s perd ses
  réservations (la panne se gère sans code spécial) ; la liste noire oublie après son délai.
- **Frontières** (limite entre le connu et l'inconnu) : elles sont dans le libre, au bord de la
  zone vue, entre 5 et 8,5 m du drone.
- **Chemins** : ligne droite quand l'espace est libre ; contournement d'un mur par 4 points au
  plus ; sortie possible de la marge de sécurité (60 cm) d'un obstacle ; aucun chemin vers
  l'intérieur d'un obstacle ; l'inconnu reste traversable, mais un couloir connu de 8 m est
  préféré à 4 m d'inconnu ; un segment déjà planifié est déclaré coupé quand un mur apparaît ;
  un coéquipier (rayon 1,5 m) est contourné.
- **Divers** : une carte sauvée puis rechargée est identique ; la vue de dessus est une image
  lisible ; l'étiquette « carton » se pose case par case ; le premier obstacle le long d'un
  rayon est trouvé à une case (30 cm) près.

### Le cerveau — `../test_planning.py` (14 tests)
- **Viser** : pour pointer la caméra gauche vers l'est, le cap vaut −π/2 (drone tourné vers le
  sud).
- **Priorités** : une piste proche passe avant des frontières lointaines (pose de lecture à 2 m
  devant le QR) ; entre deux frontières équivalentes, la plus proche gagne ; sur une carte
  entièrement vue, il ne reste aucune cible.
- **Couverture** : une face de mur jamais regardée attire le drone, puis disparaît des cibles
  une fois regardée ; une face où le détecteur a vu des cartons vaut plus (3 par case au lieu
  de 1).
- **Équipe** : une zone réservée par un coéquipier est laissée à celui-ci ; pour les autres,
  une cible réservée reçoit une note sous −900 (pénalité de 1 000) et n'est prise que s'il ne
  reste rien d'autre.
- **Pistes** : trois chances (à 2 m, puis plus près, puis de l'autre côté), puis la piste est
  écartée pour de bon ; la direction d'aperçu est moyennée ; le côté de lecture suit le grand
  axe du rack, même pour un QR au bout du rack aperçu à 70 degrés de biais.
- **Sécurité** : une pose de lecture prise dans la marge d'un obstacle est rapprochée du QR, ou
  abandonnée si aucune pose n'est praticable ; un vol à 5,3 m au-dessus d'un rack de 4,6 m est
  interdit (il faut 2 m de vide dessous).
- **Guide** : l'avis du guide vision-langage (poids λ = 1) fait choisir la piste de la zone
  conseillée.

### La méthode de référence de Pore et al. — `../test_pore.py` (29 tests)
- **Indépendance** : `pore.py` n'importe ni la carte, ni le cerveau, ni l'observateur, ni la
  perception, ni le détecteur appris.
- **Plan de couverture** fixé avant le vol : un rack par drone (cas III de l'article, sur les
  entrepôts 9033 et 9019) ; avec 4 drones, le rack le plus chargé est coupé en deux (cas II) ;
  avec 2 drones, des racks voisins sont regroupés ; le plan monte étage par étage en serpentin ;
  deux drones voisins partent de bouts opposés ; chaque arrêt est à 0,9–1,65 m de sa face,
  tourné vers elle, à la hauteur d'une étagère ; chaque drone reçoit le secteur le plus proche.
- **Champ de risque** (équation 9 de l'article) : risque au-dessus du seuil de 0,4 à 10 cm d'un
  rack, sous 0,05 à 2,2 m ; un écho lidar compte, puis s'efface après 3 s ; un coéquipier compte
  comme obstacle ; la déflexion rend un point sûr à 1,2 m au plus, ou renonce s'il n'y a pas
  d'issue.
- **Routage dans les allées** : aucun chemin ne coupe un rack ; deux arrêts voisins se
  rejoignent en ligne droite ; le couloir le plus court est choisi.
- **Lecture au sol** (décodeur OpenCV de l'article) : « BOX_042 » est lu sur un QR net ; la
  confiance baisse sur une image floue ; rien n'est lu sur une image vide.
- **Inventaire** : un code n'est compté qu'une fois ; une relecture à 0,5 s est écartée
  (fenêtre de 1,5 s de l'article) ; une lecture peu sûre (0,2 sous le seuil de 0,5) fait
  refaire l'arrêt.
- **Coordination** : deux drones face à face à 10 m seront en conflit (à moins de 1,2 m) dans
  4,4 s ; deux drones qui s'éloignent ne le seront jamais ; sous 3 s de conflit, le drone 1 cède
  le passage au drone 0 ; la vitesse baisse quand la confiance de lecture baisse.
- **Portes avant vol** : chacun des vrais codes des vols 9033 et 9019 est dans le champ et à
  1,0–2,7 m d'au moins un arrêt (la référence ne perd donc aucun code à cause de son plan) ; le
  recul de 1,65 m est dans la zone où OpenCV lit 92 % des codes ; `pore_mission.py` n'appelle
  que des méthodes qui existent (un `clock.tick()` inexistant avait déjà fait échouer un vol).

## Conclusion
- Les 89 tests passent : chaque pièce du système se comporte comme prévu sur des cas dont on
  connaît la réponse, sans simulateur et en moins de 30 secondes.
- Ils servent de garde-fou : après toute modification, le code se revérifie en une
  demi-minute, avant de relancer des vols qui coûtent 25 à 60 minutes de calcul.
- La méthode de référence est vérifiée avec le même soin que notre système, et elle n'emprunte
  rien à notre système : sur ce point, la comparaison de `13_pore` est équitable.

## Limites : ce que ces tests ne prouvent pas
- **Rien ne vole.** Le comportement réel dans Isaac Sim avec ArduPilot est mesuré dans les
  expériences (`08_controle`, `09_carte`, `11_mission`, `13_pore`), pas ici.
- **Le faux drone est simplifié** (réponse du premier ordre, mur plan) : l'inertie et le cap
  réels du drone piloté par ArduPilot sont mesurés à l'étape 3 (`08_controle`).
- **Le lidar des tests est exact**, sans bruit : la carte construite en vol est jugée dans
  `09_carte`.
- **Les cartes du cerveau sont des jouets** écrites à la main ; ses décisions en mission sont
  jugées dans `11_mission`.
- **Le détecteur** : seule la géométrie d'un cadre est testée ici ; la qualité du réseau est
  mesurée dans `10_detecteur`.

## Fichiers de résultats et figures
- Aucun fichier de résultats : pytest affiche le bilan dans le terminal.
- Fichiers lus par `test_pore.py` (à garder en place) :
  `../../experiments/11_mission/tests of system/eval_nominal/mission.json`,
  `../../experiments/11_mission/tests of system/eval_9019/mission.json`, et le code de
  `../../pore.py`, `../../pore_mission.py`, `../../control.py`, `../../env/pilot.py`,
  `../../env/scene.py`.
- Le test de sauvegarde de la carte écrit dans un dossier temporaire géré par pytest, hors du
  projet.
