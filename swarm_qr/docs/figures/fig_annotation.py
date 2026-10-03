"""La figure d'annotation automatique : deux vues de la planche de contrôle de l'étape 7,
recadrées et légendées en anglais pour le rapport. Remplace fig_annotation.sh, qui demandait
ImageMagick.

    fig_annotation.py
"""

from pathlib import Path

import matplotlib
from PIL import Image, ImageDraw, ImageFont

ICI = Path(__file__).resolve().parent
SOURCE = ICI.parents[1] / "experiments" / "10_detecteur" / "controle" / "planche_00.jpg"
TTF = Path(matplotlib.get_data_path()) / "fonts" / "ttf"

ENCRE, ETIQUETTE, CARTON = "#16201C", "#2F9E44", "#364FC7"
BAS, HAUT = 42, 54                      # bandeaux ajoutés sous les vues et au-dessus

normal = ImageFont.truetype(str(TTF / "DejaVuSans.ttf"), 20)
gras = ImageFont.truetype(str(TTF / "DejaVuSans-Bold.ttf"), 22)

LEGENDES = ("a rack from an aisle : every label and every carton is boxed",
            "close range : the label and the carton each receive their own box")
CLES = ((28, "green : label", ETIQUETTE), (228, "blue : carton", CARTON))

planche = Image.open(SOURCE)
vues = []
for x, texte in zip((0, 760), LEGENDES):
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
