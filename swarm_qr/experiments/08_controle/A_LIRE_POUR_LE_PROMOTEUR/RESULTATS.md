# 08 — Contrôle de vol : résultats

## La question
Le drone sait-il rejoindre une pose de lecture, s'y tenir le temps de lire, et dire s'il a
réussi ou abandonné ? Trois drones le peuvent-ils ensemble (le système final est un essaim) ?
Combien dure un cycle ? C'est lui qui fixe combien de cartons une mission peut lire.

## Méthode (en bref)
- Trois vols sur l'entrepôt de l'étape 2, avec ArduPilot SITL : freinage (trois façons de
  s'arrêter sur 4 m, trois fois chacune) ; 100 poses au hasard devant les cartons (1,5 à 3,2 m,
  jusqu'à 40°, hauteurs variées) avec lecture du QR à chaque arrivée ; trois drones en même temps.
- Position vraie enregistrée à chaque pas ; calibration de la caméra vérifiée avant chaque vol.
- Le contrôleur a 13 tests qui tournent sans simulateur, sur un faux drone : ils vérifient ses
  phases et ses abandons en une fraction de seconde (`swarm_qr/tests/test_control.py`).

## Résultats

### Freinage : il fallait bien un contrôleur
| façon de s'arrêter | glissade au-delà | arrivée | erreur finale | tient la pose |
|---|---|---|---|---|
| couper la vitesse | 0,79 m | — | 0,72 m | non, 0/3 |
| vitesse proportionnelle à la distance restante | 0,14 m | 4,7 s | 0,01 m | oui, 3/3 |
| consigne de position native d'ArduPilot | 0,20 m | jamais dans la tolérance | 0,25 m | non, 0/3 |

- Couper la vitesse : le drone glisse et s'immobilise à 72 cm de la cible, 3 fois sur 3 (inertie).
- Loi retenue : vitesse proportionnelle à la distance restante, plafond 1 m/s (le même pour les
  trois lois) ; loin, le drone va vite, près il ralentit seul. 4,7 s pour 4 m, puis tenue à 1 cm.
- La consigne d'ArduPilot s'arrête toujours à 25 cm : elle vit dans le repère que le pilote bâtit
  avec son GPS, décalé d'une vingtaine de centimètres du vrai. Notre loi boucle sur la position
  vraie, donc elle converge. Tolérance d'arrivée : 15 cm.

### Deux pièges trouvés en route
| problème observé | correction |
|---|---|
| Arrivé, le drone recevait « vitesse zéro » et dérivait de 1 à 3 cm/s (44 cm en 14 s). | La tenue garde la loi de position, même après un abandon (pas de dérive vers un rack). |
| Le pilote tient le cap qu'il croit avoir (compas simulé faux jusqu'à 8,7°) : avec un cap absolu, le drone restait à 8° et ne validait jamais son arrivée. | Vitesse de rotation proportionnelle à l'écart de cap vrai : erreur finale de cap tombée à 0°. |

### Cent poses : cent arrivées
| mesure | valeur |
|---|---|
| Poses atteintes | 100 sur 100, aucun abandon |
| Erreur à l'arrivée | 8,9 cm en médiane, de 3 à 13 cm (tolérance 15 cm) |
| Erreur de cap / vitesse résiduelle en tenue | 0° / 0,10 m/s |
| Temps de cycle, médiane | 14,6 s (transit 8,2 + approche 7,1 + tenue 0,6) |
| Temps de cycle, 9 poses sur 10 sous | 25 s |
| Même allée (46 poses) / par le couloir (54) | 6,7 s / 21,4 s |
| Vitesse moyenne effective, transit compris | 1,03 m/s |
| QR lu après l'arrivée | 93 sur 100 |

- Cycle ≈ 2 s + 0,85 s par mètre de trajet : le chiffre pour estimer ce qu'une mission peut lire.
- Les 7 lectures manquées ne sont pas des échecs du contrôleur (drone à moins de 12 cm de la
  pose). Elles correspondent au taux de lecture par image mesuré à l'étape 2 dans la même zone :
  48 sur 50 entre 1,5 et 2,5 m de distance apparente, 36 sur 40 entre 2,5 et 3,2, 9 sur 10
  au-delà. En mission, le drone prend 5 images par seconde en tenue : une seconde image
  rattrape presque toujours la première.
- Distance au QR vue dans l'image : juste à 1,2 cm près (médiane, `resultats.json`).
- 100 poses = 48 min de calcul (simulation à la moitié du temps réel avec un drone).

### Trois drones en même temps
Le plus gros risque, jamais testé : trois pilotes, trois liens, trois décollages, un contrôleur
par drone dans une seule boucle.

| manche | drone 0 | drone 1 | drone 2 |
|---|---|---|---|
| vers trois allées | atteint, 2 cm, lu | atteint, 10 cm, lu | atteint, 6 cm, lu |
| seconde pose, même allée | atteint, 4 cm, lu | atteint, 11 cm, lu | atteint, 1 cm, lu |

- 6 cibles sur 6, 6 QR sur 6, aucune interférence entre les liens. Simulation à 0,23 fois le
  temps réel : une mission de 10 min coûte environ 45 min de calcul.
- Manche 1 : deux drones ont pris le même couloir en sens opposés et se sont croisés à 0,96 m,
  protégés par 70 cm d'écart d'altitude. Le contrôleur n'évite pas les autres drones : la
  coordination de l'étape 5 devra empêcher deux drones de partager un couloir.

### Ce que fait le contrôleur
- Transit à vitesse constante de point en point, approche à vitesse proportionnelle, tenue 0,5 s,
  puis « atteint ». Abandon motivé : délai dépassé, ou aucun progrès pendant 8 s (drone qui racle
  un rack). Il n'a jamais eu à abandonner sur les 100 poses ; les abandons sont vérifiés sur le
  faux drone des tests.
- Il ne bloque jamais : à chaque tour, chaque drone envoie sa commande, puis le monde avance d'un
  pas. C'est ce qui permet à trois drones de voler ensemble.

## Conclusion
| à retenir | valeur |
|---|---|
| Loi retenue | vitesse proportionnelle à la distance restante, plafond 1 m/s, position vraie |
| Précision de tenue | 1 cm si on lui laisse le temps ; 9 cm quand il se déclare arrivé |
| Temps de cycle | 14,6 s en médiane ; environ 2 s + 0,85 s par mètre |
| Arrivées / lectures | 100 sur 100 sans abandon / 93 sur 100, conforme à l'étape 2 |
| Trois drones ensemble | 6 cibles et 6 lectures sur 6, 0,23 fois le temps réel |

## Limites : ce que ce test ne prouve pas
- Pas de calcul de chemin : ici, une règle simple sur le plan connu (même allée : ligne droite ;
  sinon, le couloir au bout des racks). Dans le vrai système, l'entrepôt est inconnu et le chemin
  viendra de la carte de l'étape 4.
- Pas d'évitement des autres drones.

## Fichiers de résultats et figures
- `../resultats.json` : tous les chiffres ; `../freinage.json`, `../poses.jsonl`,
  `../meta_poses.json`, `../essaim.json` : données brutes.
- `../freinage.png` : distance à la cible et vitesse dans le temps, 3 lois × 3 essais.
- `../cycles.png` : temps de cycle (même allée / couloir) et selon la longueur du trajet.
- `../essaim.png` : trajectoires des trois drones (rond : départ, croix : cible), par manche.
