# 14 — Le rejeu 3D des 8 vols (système et référence)

## En une phrase
Revoir en 3D, dans un navigateur, les 8 vols de l'évaluation finale : les 4 du système et les
4 de la méthode de référence de Pore et al., dans les mêmes 4 cas.

## La tâche
Les vidéos montrent ce que filmaient les caméras ; le rejeu montre tout l'entrepôt à la fois.
C'est une page web qui redessine l'entrepôt, les drones et l'inventaire à partir des journaux
de vol. Aucun simulateur n'est nécessaire, ni pour la voir ni pour la refaire. Les 4 cas :
nominal, panne d'un drone, entrepôt jamais vu, obstacle en cours de mission (détails et
chiffres dans `../../13_pore/A_LIRE_POUR_LE_PROMOTEUR/RESULTATS.md`).

## Contenu du dossier
| Fichier | Rôle |
|---|---|
| `../rejeu_hors_ligne.html` | la page de rejeu, qui fonctionne sans Internet |
| `../rejeu.html` | la même page, qui charge la bibliothèque 3D depuis Internet |
| `../three.min.js` | la bibliothèque 3D three.js (version r128), lue par `rejeu_hors_ligne.html` |
| `../vols.json` | les données des 8 vols (1,3 Mo), reprises dans les deux pages |
| `../extrait.py` | fabrique `vols.json` à partir des journaux de vol |

## Comment l'ouvrir
- **Double-cliquer sur `../rejeu_hors_ligne.html`.** La page s'ouvre dans le navigateur et
  fonctionne sans Internet, à condition que `three.min.js` soit dans le même dossier. Depuis
  Google Drive, télécharger le dossier `14_rejeu` entier, pas la page seule.
- `../rejeu.html` est identique, à une ligne près : elle charge three.js depuis Internet
  (cdnjs.cloudflare.com) au lieu du fichier local. Elle ne marche qu'avec une connexion.
- Les deux pages demandent aussi des polices à Google ; sans Internet, le navigateur en prend
  une autre, sans autre effet.
- Les données des vols et les figures sont incluses dans la page : `vols.json` n'est pas
  nécessaire pour la regarder.

## Ce que montre le rejeu
- **À gauche** : le choix du vol, en deux groupes (« Système proposé », « Pore et al. —
  référence ») de 4 cas, et « Ce vol en chiffres » : durée, codes lus, temps pour 50 % et 90 %
  de l'inventaire, arrêts du plan servis (référence), points dans un rack, écart minimal entre
  drones, cause de la fin. Les entrepôts sont nommés « Entrepôt 1 » (graine 9033) et
  « Entrepôt 2 » (graine 9019).
- **Au centre**, l'entrepôt en 3D :
  - les racks, les cartons et leurs QR, qui passent au vert quand le code est lu ;
  - les 3 drones avec leur traînée ; un drone en panne devient gris ;
  - le trait de décision, en pointillé, du drone vers ce qu'il vise ;
  - l'obstacle, en rouge, à partir de son apparition ;
  - le brouillard de découverte, en gris sur ce que la carte ne connaît pas encore. Il n'existe
    que pour le système : la référence ne construit pas de carte, elle reçoit le plan.
- **Boutons de vue** : caméra libre (glisser pour tourner, clic droit pour déplacer, molette
  pour zoomer), vue de dessus, suivre un drone (ou cliquer sa fiche à droite). Trois boutons
  affichent ou masquent le brouillard, le trait de décision et, en médaillon, la vue du drone
  suivi.
- **À droite** : pour chaque drone, son état (lecture, couverture, exploration, arrêt du plan,
  arrêté), sa cible et son altitude ; puis la grille de tous les codes. Une case verte est un
  code lu, une case cerclée de rouge un code jamais lu ; au survol, l'instant de lecture.
- **En bas** : lecture et pause. La courbe des codes lus sert de barre de temps (cliquer ou
  glisser) ; panne et obstacle y sont marqués en pointillé rouge. Des raccourcis (« Panne du
  drone 1 », « Obstacle posé », « Moitié lue », « 90 % lus ») et la vitesse 1×, 2×, 4× (1× =
  temps réel du vol simulé). Au clavier : espace = lecture/pause, flèches = ±5 s.
- **Bouton « Résultats »** : 7 figures, à parcourir avec les flèches (Échap pour revenir) :
  - 3 figures de comparaison : la part de l'inventaire lue par cas (avec aussi l'inférence
    active et l'apprentissage par renforcement), les codes lus au fil du temps, le temps pour
    atteindre 50, 80 et 90 % ;
  - 4 figures « plan et vol » de la référence, une par cas.

## Ce que le rejeu n'est pas
- Ce n'est pas une vidéo : la scène est un dessin simplifié. Les racks sont des montants et
  des plateaux, les cartons des boîtes ; les drones sont grossis 1,45 fois pour être visibles.
  La « vue du drone » est ce dessin vu du drone, pas l'image de sa caméra. Les vraies images
  sont dans les vidéos (`video/mission.mp4` de chaque vol).
- Les positions et les inclinaisons sont celles enregistrées pendant le vol. Le cap (la
  direction où regarde le drone) n'est pas dans les journaux : il est reconstitué (vers sa
  cible quand il en est tout près, sinon vers où il va).
- Pour le système, l'instant de lecture de chaque code est reconstitué. Le nombre de codes lus
  au fil du temps est exact, mais l'ordre des codes lus dans un même intervalle de 10 s est
  approché. Pour la référence, les instants viennent de son journal de lectures.
- Le brouillard avance par paliers de 10 s (un instantané de carte toutes les 10 s).

## Comment il est reconstruit
Rien à refaire pour regarder. Pour reconstruire après de nouveaux vols :
1. Les vols : `swarm_qr/experiments/11_mission/evaluation.sh` (système) et
   `swarm_qr/experiments/13_pore/campagne.sh` (référence). Chaque vol laisse `mission.json` et
   `resultats.json` ; ceux du système laissent aussi des instantanés de carte (`instantanes/`).
2. L'extraction, sans simulateur (Python avec numpy suffit), depuis la racine du projet :

       $PY swarm_qr/experiments/14_rejeu/extrait.py

   Elle lit `../../11_mission/tests of system/eval_{nominal,panne,9019,obstacle}/` et
   `../../13_pore/pore_{nominal,panne,9019,obstacle}/`, puis écrit `../vols.json` (option
   `--sortie` pour écrire ailleurs). Pour les vols de la référence, racks et panneaux viennent
   du vol du système sur le même entrepôt.
3. `vols.json` → page : le contenu de `vols.json` se trouve dans la ligne
   `<script>window.__VOLS__=…;</script>` des deux pages. La ligne suivante,
   `window.__FIGURES__`, contient les 7 figures en images intégrées. Les 3 figures de
   comparaison sont celles de `swarm_qr/docs/figures/evaluation/` (`ev_methodes.png`,
   `ev_lecture_temps.png`, `ev_jalons.png`) ; les 4 autres sont les figures « plan et vol » de
   la référence.
