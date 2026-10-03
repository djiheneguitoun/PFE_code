"""Fabrique fig_annotation.png : deux vues de la planche de contrôle du détecteur (étape 7), boîtes
vertes = QR, bleues = cartons, recadrées et légendées en anglais pour le rapport. Source :
experiments/10_detecteur/controle/planche_00.jpg. Remplace l'ancien fig_annotation.sh (ImageMagick).
    python fig_annotation.py
"""

from pathlib import Path

import matplotlib
from PIL import Image, ImageDraw, ImageFont

ICI = Path(__file__).resolve().parent
SOURCE = ICI.parents[1] / "experiments" / "10_detecteur" / "controle" / "planche_00.jpg"   # planche de l'étape 7
TTF = Path(matplotlib.get_data_path()) / "fonts" / "ttf"   # polices DejaVu livrées avec matplotlib

ENCRE, ETIQUETTE, CARTON = "#16201C", "#2F9E44", "#364FC7"   # couleurs : texte, boîte QR, boîte carton
BAS, HAUT = 42, 54                      # hauteur (px) des bandeaux ajoutés sous les vues et au-dessus

normal = ImageFont.truetype(str(TTF / "DejaVuSans.ttf"), 20)      # police des légendes
gras = ImageFont.truetype(str(TTF / "DejaVuSans-Bold.ttf"), 22)   # police de la clé des couleurs

# légende sous chaque vue (gauche, droite)
LEGENDES = ("a rack from an aisle : every label and every carton is boxed",
            "close range : the label and the carton each receive their own box")
# clé des couleurs en haut : (x en px, texte, couleur)
CLES = ((28, "green : label", ETIQUETTE), (228, "blue : carton", CARTON))

planche = Image.open(SOURCE)
vues = []
for x, texte in zip((0, 760), LEGENDES):
    # deux vues de 760 x 536 px découpées dans la planche, à gauche puis à droite
    vue = planche.crop((x, 602, x + 760, 1138))
    avec = Image.new("RGB", (760, 536 + BAS), "white")
    avec.paste(vue, (0, 0))
    d = ImageDraw.Draw(avec)
    l, t, r, b = d.textbbox((0, 0), texte, font=normal)
    d.text(((760 - (r - l)) / 2, 536 + (BAS - (b - t)) / 2 - t), texte, font=normal, fill=ENCRE)
    vues.append(avec)

duo = Image.new("RGB", (1520, 536 + BAS + HAUT), "white")
duo.paste(vues[0], (0, HAUT))
duo.paste(vues[1], (760, HAUT))

d = ImageDraw.Draw(duo)
for x, texte, couleur in CLES:
    l, t, r, b = d.textbbox((0, 0), texte, font=gras)
    d.text((x, (HAUT - (b - t)) / 2 - t), texte, font=gras, fill=couleur)

duo.save(ICI / "fig_annotation.png")
print(f"fig_annotation.png : {duo.size[0]}x{duo.size[1]}")
