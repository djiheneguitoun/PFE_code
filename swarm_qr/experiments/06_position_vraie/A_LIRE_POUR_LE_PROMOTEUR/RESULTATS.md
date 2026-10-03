# 06 — Position vraie du drone et de sa caméra : résultats

## La question
Le drone sait-il où il est, et où est sa caméra ? Au test 3, la taille du QR dans l'image
indiquait une distance d'environ deux tiers de celle que le pilote croyait avoir. Ce test
tranche en comparant trois estimations indépendantes de la même distance.

## Méthode (en bref)
- Graine 7, un drone piloté par ArduPilot SITL, décollage à 1,6 m.
- Panneau visé : BOX_000, panneau de 0,40 m. Le code lui-même mesure 0,336 m : il occupe
  21 modules (petits carrés) sur les 25 du panneau, le reste est la marge blanche.
- Le drone se place à 1, 2 puis 3 m du panneau, distance comptée depuis son centre. La
  hauteur de la caméra est compensée (11 cm), pas son décalage sur le côté.
- À chaque point, distance horizontale au panneau :
  - « pilote » : centre du drone, position lue dans le simulateur (celle qu'utilise le
    pilote pour se diriger) ;
  - « caméra » : caméra gauche, position lue dans le simulateur ;
  - « optique » : focale (886,8 pixels) × taille du code (0,336 m) ÷ taille du code dans
    l'image (pixels).
- Référence : la distance optique (à 1 % près pour une caméra fixe à distance connue).

## Résultats
| Visé | Pilote | Caméra | Optique | Code (px) |
|---|---|---|---|---|
| 1,0 m | 0,926 m | 0,826 m | 0,828 m | 360,0 |
| 2,0 m | 1,920 m | 1,821 m | 1,778 m | 167,6 |
| 3,0 m | 2,956 m | 2,856 m | 2,844 m | 104,8 |

1. **La caméra et l'image sont d'accord** : écart caméra − optique de 0,2 cm, 4,3 cm et
   1,2 cm (0,2 %, 2,4 % et 0,4 %).
2. **Le centre du drone est toujours 10 cm plus loin que la caméra** (0,100 ; 0,099 ;
   0,100 m).
3. Écart pilote / optique (calcul affiché par le script) : +12 %, +8 %, +4 %. Il diminue
   avec la distance, car le décalage fixe de 10 cm pèse de moins en moins. La distance
   optique vaut donc 89 %, 93 % et 96 % de la distance « pilote ».
4. **Montage de la caméra**, d'après les positions du drone et de la caméra aux trois points :
   10,0 à 10,1 cm vers le panneau (sur le côté gauche du drone, là où regarde la caméra),
   10,9 à 11,0 cm plus bas, et moins de 1 cm dans la troisième direction.
5. Le centre du drone s'est arrêté 7,4 cm, 8,0 cm et 4,4 cm plus près que visé (le pilote
   accepte 15 cm d'écart à l'arrivée).

Les images montrent ce que voit la caméra gauche : à 1 m, le code visé occupe une grande
partie de l'image (360 px de côté) et un voisin apparaît en partie ; à 2 m, trois panneaux
sont visibles ; à 3 m, quatre.

## Conclusion
- Les trois mesures concordent une fois le montage de la caméra pris en compte : la position
  lue dans le simulateur, la pose de la caméra et l'image racontent la même chose, à
  quelques centimètres près.
- L'écart entre la distance « pilote » et la distance optique vient du montage de la
  caméra : 10 cm sur le côté du drone (vers le panneau) et 11 cm plus bas que son centre.
- Conséquence pratique : une visée qui place le centre du drone à la distance voulue met la
  caméra 10 cm trop près. Ces deux valeurs sont inscrites dans `swarm_qr/env/config.py`
  (`side_offset = 0.10`, `below = 0.11`, avec la mention « vérifié en vol
  (06_position_vraie) »), et le test 3 compense désormais les deux dans sa visée (voir
  `03_images_qr`).

## Fichiers de résultats et figures
- `../resultat.json` : panneau visé, tailles du panneau (0,4 m) et du code (0,336 m),
  focale en pixels (886,8), et pour chaque point les trois distances, la taille du code en
  pixels et les positions du drone et de la caméra.
- `../vue_1.0m.jpg`, `../vue_2.0m.jpg`, `../vue_3.0m.jpg` : l'image de la caméra gauche à
  chaque point visé.
