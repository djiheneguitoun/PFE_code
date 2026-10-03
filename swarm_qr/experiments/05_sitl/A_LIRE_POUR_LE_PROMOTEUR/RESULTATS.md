# 05 — SITL : la chaîne vole, et voici ses chiffres : résultats

## La question
Isaac Sim 5.1 + Pegasus + ArduPilot SITL : est-ce que ça vole, et à quel prix ?

## Méthode (en bref)
`vol_auto.py` fait tout seul, sans fenêtre :
- construit la scène : l'entrepôt d'origine (racks non déplacés, sans QR), le sol, un drone
  Iris posé dans l'allée ouest ;
- parle à ArduPilot par MAVLink (le protocole de commande des drones) sur le port de secours
  du SITL (`tcp:5762`), pendant que Pegasus garde son propre lien (port 14550) ;
- enchaîne : attente de l'estimateur de position (EKF), mode guidé, armement, décollage à
  3 m ;
- puis trois mesures :
  1. stationnaire : 10 s immobile → écart-type et écart maximal de la position, en cm.
     C'est la mesure qui conditionne toute la lecture de QR ;
  2. arrêt : vitesse commandée à 1 m/s pendant 3 s, puis zéro → temps et distance pour
     descendre sous 5 cm/s. C'est l'inertie réelle du drone ;
  3. débit : 2 400 pas de simulation sans image, chronométrés.

## Résultats
**Oui, la chaîne est validée de bout en bout.** Lien de commande établi, estimateur prêt en
36 s, mode guidé confirmé, armement réussi, décollage accepté (acquittement 0 = accepté),
altitude de 2,99 m atteinte pour 3 m demandés — le tout automatiquement, sans fenêtre.

| Mesure | Valeur |
|---|---|
| Stationnaire 10 s, écart-type | X 10,8 / Y 0,9 / Z 1,2 cm |
| Stationnaire 10 s, écart maximal | 20,9 cm (à l'horizontale) |
| Arrêt depuis 1,01 m/s | 6,8 s et 1,06 m |
| Débit sans image | 413 pas/s = 0,52× le temps réel |

Lecture des chiffres :
- L'écart-type en X est pollué par la fin de la stabilisation après la montée : la mesure a
  commencé 4 s seulement après l'arrivée à 3 m. Y et Z, autour de 1 cm, montrent la vraie
  tenue.
- L'arrêt mesure l'inertie réelle du drone — celle que l'ancien cube à vitesse instantanée
  n'avait pas — avec un seuil volontairement strict (moins de 5 cm/s).
- Le débit sans image donne une mission de 10 min en environ 19 min réelles. Il sera
  re-mesuré avec les caméras (test 4, `04_debit`).

## Pièges résolus en route
| Problème observé | Correction |
|---|---|
| Sur un port MAVLink secondaire, ArduPilot n'envoie presque rien | S'annoncer comme station sol et demander les flux de données (MAVProxy le fait seul) |
| Un décollage demandé avant que l'estimateur ait son origine GPS est refusé en silence | Attendre le fixe GPS 3D plus 20 s, comme le pipeline Gazebo du projet (ou le message « is using GPS ») |
| Les décors en ligne de Pegasus bloquaient la fenêtre | Charger notre entrepôt par son adresse directe, déjà en cache |
| L'exemple Pegasus oublie le pas de physique d'ArduPilot | Monde réglé à 1/800 s |

## Conclusion
La chaîne Isaac Sim 5.1 + Pegasus + ArduPilot SITL vole, de bout en bout et sans
intervention. Le drone tient le stationnaire à environ 1 cm (Y, Z), et il a une vraie
inertie : il lui faut environ 1 m pour s'arrêter depuis 1 m/s. La séquence trouvée ici
(station sol, flux, GPS plus marge, mode guidé confirmé, armement avec réessais, décollage
avec acquittement) est celle que reprend le pilote du système, `swarm_qr/env/pilot.py`.

## Fichiers de résultats et figures
- `../resultat_vol.json` : lien, estimateur, mode, armement, acquittements du décollage,
  altitude atteinte (2,99 m), stationnaire, arrêt, débit (412,6 pas/s).
- `../debug_cams/` : 5 images de débogage — vue de dessus après 40 et 240 pas
  (`overview_40pas.jpg`, `overview_240pas.jpg`), caméra gauche du drone 0 après 40 et
  240 pas, caméra frontale après 240 pas.
