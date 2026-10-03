"""Introspection (étape 8) : après sa réponse, demande au modèle quels champs il a utilisés et quels
nombres il a lus (une loupe sur ses erreurs, pas une preuve). Modèle en 4 bits. Depuis swarm_qr/ :
    $PY experiments/12_guide/introspection.py --missions <dossiers de vol> --modele <dossier du modèle> --cas 15
Écrit resultats_introspection.json dans ce dossier (vrais chiffres, réponse, explication du modèle).
"""
import argparse, json, sys
from pathlib import Path
import cv2
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2])); sys.path.insert(0, str(HERE))
from swarm_qr.guide import Guide
from banc import instantanes
from variantes import dossier, melange, CONTEXTE_CAMERA, BUT, _reponse_zone

ap = argparse.ArgumentParser()
ap.add_argument("--missions", nargs="+", required=True)
ap.add_argument("--modele", required=True)
ap.add_argument("--cas", type=int, default=15)
a = ap.parse_args()
tous = instantanes([Path(m) for m in a.missions])
# --cas cas pris à intervalles réguliers dans la liste
pas = max(len(tous) // a.cas, 1)
cas = tous[::pas][:a.cas]
g = Guide(a.modele, max_tokens=160, quantisation="4bit")
sortie = []
for k, c in enumerate(cas):
    zones, bonne, geo = melange(c["zones"], c["verite"], graine=tous.index(c))
    dos = dossier(c, zones, False)
    q1 = (CONTEXTE_CAMERA + BUT + "Here is the map data as JSON:\n" + json.dumps(dos, indent=1)
          + "\nWhich zone should the drone go to next? 'ANSWER: <zone number>' then 'WHY: <one sentence>'.")
    cam = cv2.imread(c["cam"])
    r1 = g.repond_images([cam], q1)
    choix = _reponse_zone(r1, zones)
    # deuxième question : la même conversation, plus « explique-toi » sur la zone choisie et la bonne
    q2 = (q1 + "\nYour answer was: " + r1.strip()
          + f"\nNow explain: which fields of the data did you use to decide? Quote the exact values "
            f"you read for zone {choix} and for zone {bonne}, then say which is larger. Be brief.")
    r2 = g.repond_images([cam], q2)
    # vrais chiffres de chaque zone : (codes repérés non lus, distance au drone en m)
    vrai = {z["numero"]: (z.get("n_lire"), round(float(__import__('math').hypot(z['centre'][0]-c['position'][0], z['centre'][1]-c['position'][1])), 1)) for z in zones}
    ligne = {"cas": k, "choix": choix, "bonne": bonne, "juste": choix == bonne,
             "vrais_chiffres (zone: codes reperes non lus, distance m)": {choix: vrai.get(choix), bonne: vrai.get(bonne)},
             "reponse": r1.strip()[:120], "introspection": r2.strip()[:400]}
    sortie.append(ligne)
    print(f"\n=== cas {k} : choix {choix}, bonne {bonne} -> {'JUSTE' if choix == bonne else 'FAUX'}")
    print(f"   vrais chiffres : zone {choix} = {vrai.get(choix)} | zone {bonne} = {vrai.get(bonne)}")
    print("   il dit :", r2.strip().replace("\n", " | ")[:400])
(HERE / "resultats_introspection.json").write_text(json.dumps(sortie, indent=1))
print("\nINTROSPECTION FINI")
