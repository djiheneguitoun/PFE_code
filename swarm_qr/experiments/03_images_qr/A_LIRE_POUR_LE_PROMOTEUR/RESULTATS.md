# 03 — Lecture des QR en vol : résultats

## La question
Le drone lit-il vraiment les QR codes en volant ? C'est le cœur de la tâche. Ici le drone
est un vrai appareil piloté par ArduPilot (simulé), avec son inertie et ses imprécisions.

## Méthode (en bref)
- Graine 7, un drone, décollage à 1,6 m.
- Panneau visé : BOX_000, 40 cm de côté.
- La planche : le drone rejoint six points face au panneau (caméra à 0,5 ; 0,8 ; 1,1 ; 1,5 ;
  2 et 3 m), s'arrête, photographie avec sa caméra gauche, et on vérifie que **le code visé**
  est lu — pas celui d'un carton voisin.
- La vidéo : le drone longe le rack à 0,5 m/s en lisant en continu.
- Le trajet vers le rack passe par le couloir ouvert au bout des racks : la ligne droite les
  traverserait, et l'évitement d'obstacles n'existe pas encore.
- Lecture par zxing (bibliothèque de lecture de codes), réglée pour ne lire que les QR.

## Résultats

### La planche : lecture à l'arrêt
| Distance | Cible lue | Voisins lus aussi | Écart au point |
|---|---|---|---|
| 0,5 m | oui | — | 10,6 cm |
| 0,8 m | oui | — | 3,7 cm |
| 1,1 m | oui | — | 8,1 cm |
| 1,5 m | oui | BOX_001 | 4,2 cm |
| 2,0 m | oui | BOX_001 | 4,4 cm |
| 3,0 m | oui | BOX_001, BOX_002 | 3,9 cm |

- **Les six distances sont lues, de 0,5 à 3 m.** Chaque point a été atteint, entre 4 et
  11 cm du point demandé (écart mesuré sur la position vraie du drone).
- À partir de 1,5 m, les cartons voisins se lisent en même temps que la cible : trois codes
  dans une seule image à 3 m.

### La vidéo : lecture en vol continu
- **Six codes différents lus en un seul passage** : BOX_000 à BOX_004 et BOX_018.
- La vidéo compte 52 images.

### Ce que cela corrige
| Problème observé | Correction |
|---|---|
| Anciennes versions : « pas de lecture sous 1 m environ » (faux) | Visée complète (côté et hauteur) : lu dès 0,5 m |

La cause : la caméra est montée 10 cm sur le côté du drone et 11 cm plus bas, et l'ancienne
visée ne compensait que la hauteur. Le drone se plaçait donc toujours 10 cm trop près, et à
courte distance ce décalage suffisait à couper la marge blanche du code. Ce décalage a été
mesuré en vol par le test 6 (`06_position_vraie`).

## Conclusion
**La chaîne complète est prouvée** — décollage, trajet sans collision, visée, capture,
lecture, identité du carton — sur toute la plage de 0,5 à 3 m.

## Fichiers de résultats et figures
- `../resultat.json` : le socle (Pegasus + ArduPilot SITL), le panneau visé (BOX_000,
  0,4 m), une ligne par distance (point atteint, cible lue, voisins lus, écart de position),
  la portée maximale (3 m), les codes lus en vol et le nombre d'images de la vidéo (52).
- `../planche_qr.jpg` : les six photos ; chaque titre donne la distance et
  « CIBLE LUE BOX_000 ».
- `../vol_le_long_du_rack.mp4` : la vidéo du vol (à gauche la caméra du drone et le compteur
  de codes lus, à droite la vue de dessus et la position du drone).
