# 04 — Débit de la simulation : résultats

## La question
Combien de temps coûte une simulation ? Ce chiffre gouverne tout le projet, puisqu'il dit
combien d'heures coûtera l'évaluation finale.

## Méthode (en bref)
- Scène complète de la graine 7 : 3 drones, 9 caméras.
- Pas de physique : 1/800 s (réglage officiel d'ArduPilot), donc temps réel = 800 pas/s.
- Trois situations chronométrées :
  - sans aucune image ;
  - 5 images par seconde simulée (une image tous les 160 pas) ;
  - une image à chaque pas de calcul.

## Résultats
| Situation | Pas/s | × temps réel |
|---|---|---|
| sans image | 222,4 | 0,28 |
| 5 images/s | 205,7 | 0,26 |
| image à chaque pas | 6,2 | 0,01 |

(× temps réel = pas/s ÷ 800, comme sur la figure `../courbe_debit.png`.)

- Avec 5 images par seconde, la machine perd à peine 7 % par rapport à « sans image ».
- Avec une image à chaque pas, elle s'effondre à 6 pas par seconde.

## Conclusion
- Le coût ne vient pas des caméras, contrairement à ce qu'on pouvait croire, mais de la
  physique du pilote automatique, qui doit être calculée 800 fois par seconde.
- La simulation tourne donc environ 4 fois moins vite que le temps réel : une mission de
  10 minutes demande à peu près 40 minutes de calcul, et l'évaluation finale de 25 missions
  environ 16 heures.
- **Réglage retenu : 5 images par seconde.** C'est largement suffisant pour un drone qui
  vole sous 1 m/s, et les caméras gardent leur pleine résolution, condition pour lire les
  QR codes.

## Fichiers de résultats et figures
- `../mesures.json` : pour chaque situation, les pas/s et le pas de physique (0,00125 s).
- `../courbe_debit.png` : à gauche les pas/s (222, 206, 6), à droite la vitesse par rapport
  au temps réel (0,28×, 0,26×, 0,01×) avec la ligne du temps réel.
