"""Le guide vision-langage, optionnel (étape 8) : conseille au drone une zone où aller, et un côté.

`Guide` : un modèle généraliste (SmolVLM, Qwen2.5-VL) regarde la caméra et la carte vue de dessus.
`GuideEntraine` : Qwen2.5-VL 3B en 4 bits + adaptateur LoRA (petites couches entraînées sur des vols)
lit les faits de la carte en JSON. L'avis pèse dans la note des cibles avec un poids λ (planning.py).
Lancé par `mission.py --guide ... --lam ...` ; bancs : experiments/12_guide.
"""

from __future__ import annotations

import os
import re
import time
from concurrent.futures import Future, ThreadPoolExecutor

import cv2
import json
import numpy as np

from . import planning

# Noms courts → modèles Hugging Face (500 millions et 2,2 milliards de paramètres).
MODELES = {
    "smolvlm": "HuggingFaceTB/SmolVLM-500M-Instruct",
    "smolvlm-2b": "HuggingFaceTB/SmolVLM-Instruct",
}
# Mot de la réponse → numéro de côté (0 est, 1 ouest, 2 nord, 3 sud), en anglais ou en français.
COTES = {"north": 2, "nord": 2, "south": 3, "sud": 3, "east": 0, "est": 0, "west": 1, "ouest": 1}
# Consigne de `Guide` (en anglais) qui explique les deux images ; suivent les deux questions courtes.
CONTEXTE = (
    "You guide a drone that must read QR codes glued on cardboard boxes stored on warehouse "
    "shelves. The first image is the drone's side camera. The second image is the map seen from "
    "above: dark grey is a shelf or a wall, white has already been read, light grey is free, "
    "medium grey is unknown, orange dots are boxes seen but not read yet, and the red numbered "
    "circles are the candidate zones. "
)
QUESTION_ZONE = CONTEXTE + "{description}Which zone number should the drone go to next to read the most unread boxes? Answer with the number only."
QUESTION_COTE = CONTEXTE + "{description}The drone goes to zone {n}. From which side should it approach the shelf there: north, south, east or west? Answer with one word."


def decrit(zones: list[dict], position=None, codes_lus: int | None = None) -> str:
    """Renvoie, en phrases anglaises, les faits connus de chaque zone (jamais la note du cerveau)."""
    lignes = []
    for z in zones:
        faits = []
        # zones récentes : comptages par genre de cible ; anciens instantanés : seulement les genres
        if z.get("n_lire") is not None:
            if z["n_lire"]:
                faits.append(f"{z['n_lire']} QR code(s) spotted but not read yet")
            if z.get("n_couvrir_cartons"):
                faits.append(f"{z['n_couvrir_cartons']} shelf section(s) with boxes never looked at")
            elif z.get("n_couvrir"):
                faits.append(f"{z['n_couvrir']} surface(s) never looked at")
            if z.get("n_explorer"):
                faits.append("unexplored space")
        else:
            genres = set(z.get("genres", []))
            if "lire" in genres:
                faits.append("QR codes spotted but not read yet")
            if "couvrir" in genres:
                faits.append("surfaces never looked at")
            if "explorer" in genres:
                faits.append("unexplored space")
            faits.append(f"{z['cibles']} candidate target(s)")
        if z.get("cote"):
            faits.append(f"shelf faces looking {z['cote']}")
        d = ""
        if position is not None:
            d = f", {float(np.hypot(z['centre'][0] - position[0], z['centre'][1] - position[1])):.0f} m from the drone"
        lignes.append(f"Zone {z['numero']}: " + ", ".join(faits) + d + ".")
    tete = f"So far {codes_lus} codes have been read. " if codes_lus is not None else ""
    return tete + "The map says: " + " ".join(lignes) + " "


# Pixels par image au plus pour Qwen : environ 640 jetons d'image (un jeton couvre 28 × 28 pixels).
MAX_PIXELS = 640 * 28 * 28


# Texte de `GuideEntraine`, identique à celui de l'entraînement (sans image) : début, but, question.
TETE = ("You help a team of drones that must read QR codes glued on cardboard boxes in a warehouse. "
        "You get no image, only the data the map holds. ")
BUT = ("The drone must fly to the zone where it will read the largest number of QR codes that are "
       "still unknown. ")
QUESTION = "\nWhich zone should the drone go to next? Answer with 'ANSWER: <zone number>'."
# Champs retirés du dossier : ils égaraient le modèle (83 % de bonnes zones sans eux, 75 % avec, étape 8).
DISTRACTEURS = ("total_candidate_targets", "target_kinds", "radius_m", "unexplored_frontier_groups")


def boussole(zone, position) -> str:
    """Renvoie la direction de la zone vue du drone (« north-east »…), ou « right here » à moins de 1 m."""
    dx = zone["centre"][0] - position[0]
    dy = zone["centre"][1] - position[1]
    ns = "north" if dy > 1.0 else ("south" if dy < -1.0 else "")
    eo = "east" if dx > 1.0 else ("west" if dx < -1.0 else "")
    return (ns + ("-" if ns and eo else "") + eo) or "right here"


def dossier_zones(cas: dict, zones: list[dict], avec_note: bool = False) -> dict:
    """Renvoie le dossier complet (dict prêt pour JSON) : instant, codes lus, ce drone, coéquipiers, racks
    et zones candidates ; `avec_note` y ajoute la note du cerveau géométrique."""
    moi = cas.get("position") or [0.0, 0.0, 0.0]
    autres = [d for d in cas.get("coequipiers", []) if d["i"] != cas.get("drone")]
    out = {
        "mission_time_s": cas.get("t"),
        "qr_codes_read_so_far": cas.get("codes_lus"),
        "this_drone": {"id": cas.get("drone"), "position_xyz_m": [round(float(v), 2) for v in moi],
                       "heading_deg": cas.get("cap_deg"),
                       "note": "the image is one of its two side cameras, looking sideways "
                               "from the heading above"},
        "teammates": [{"id": d["i"], "position_xyz_m": [round(float(v), 2) for v in d["position"]],
                       "heading_deg": round(float(np.degrees(d.get("cap", 0.0))), 1),
                       "distance_from_this_drone_m": round(float(np.linalg.norm(
                           np.array(d["position"]) - np.array(moi))), 1)} for d in autres],
        "shelves_mapped_so_far": [{"name": r["prim"], "x_range_m": [round(float(r["x"][0]), 2), round(float(r["x"][1]), 2)],
                                   "y_range_m": [round(float(r["y"][0]), 2), round(float(r["y"][1]), 2)]}
                                  for r in cas.get("racks", [])],
        "candidate_zones": [],
    }
    for z in sorted(zones, key=lambda z: z["numero"]):
        d_moi = float(np.hypot(z["centre"][0] - moi[0], z["centre"][1] - moi[1]))
        d_eux = [float(np.hypot(z["centre"][0] - d["position"][0], z["centre"][1] - d["position"][1]))
                 for d in autres]
        e = {"number": z["numero"], "centre_xy_m": z["centre"], "radius_m": z["rayon"],
             "qr_codes_spotted_but_not_read": z.get("n_lire"),
             "shelf_sections_with_a_box_seen_but_face_never_looked_at": z.get("n_couvrir_cartons"),
             "surfaces_never_looked_at_total": z.get("n_couvrir"),
             "unexplored_frontier_groups": z.get("n_explorer"),
             "target_kinds": z.get("genres"),
             "total_candidate_targets": z.get("cibles"),
             "shelf_faces_look_towards": z.get("cote"),
             "distance_from_this_drone_m": round(d_moi, 1),
             "direction_from_this_drone": boussole(z, moi),
             "distance_from_nearest_teammate_m": round(min(d_eux), 1) if d_eux else None}
        if avec_note:
            e["geometric_score_of_our_planner"] = z.get("utilite")
        out["candidate_zones"].append(e)
    return out


def epure(dos: dict, distracteurs: bool = True) -> dict:
    """Renvoie une copie du dossier où chaque zone commence par le champ décisif (QR repérés non lus)
    et perd les champs DISTRACTEURS si `distracteurs` est vrai."""
    zones = []
    for z in dos["candidate_zones"]:
        tete = {"number": z["number"], "qr_codes_spotted_but_not_read": z["qr_codes_spotted_but_not_read"]}
        reste = {k: v for k, v in z.items() if k not in tete and not (distracteurs and k in DISTRACTEURS)}
        zones.append({**tete, **reste})
    return {**dos, "candidate_zones": zones}


def texte_pour_le_guide(cas: dict, zones: list[dict]) -> str:
    """Renvoie le texte exact donné au guide entraîné, comme à l'entraînement : consigne, dossier
    épuré en JSON et question, sans image."""
    return TETE + BUT + "Here is the map data as JSON:\n" + json.dumps(epure(dossier_zones(cas, zones)), indent=1) + QUESTION


class GuideEntraine:
    """Le guide entraîné : Qwen2.5-VL 3B compressé en 4 bits + adaptateur LoRA, qui lit du texte seul."""

    def __init__(self, modele: str, adaptateur: str, device: str = "cuda", max_tokens: int = 12):
        """Charge le modèle `modele` en 4 bits et y branche l'adaptateur LoRA `adaptateur` ; `max_tokens` =
        longueur maximale de la réponse, en jetons."""
        os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
        import torch
        from peft import PeftModel
        from transformers import AutoModelForImageTextToText, AutoProcessor, BitsAndBytesConfig

        # 4 bits (NF4) : le modèle 3B tient en 2,5 Go de mémoire graphique au lieu de 6,7 Go
        config = BitsAndBytesConfig(load_in_4bit=True, bnb_4bit_quant_type="nf4",
                                    bnb_4bit_compute_dtype=torch.float16, bnb_4bit_use_double_quant=True)
        self.processor = AutoProcessor.from_pretrained(modele)
        base = AutoModelForImageTextToText.from_pretrained(modele, quantization_config=config,
                                                            device_map={"": device}, dtype=torch.float16,
                                                            attn_implementation="sdpa")
        self.modele = PeftModel.from_pretrained(base, adaptateur).eval()   # le modèle reste gelé
        self.nom, self.device, self.max_tokens = f"{modele} + {adaptateur}", device, max_tokens
        self.latences: list[float] = []          # s, une par appel
        self.reponses: list[str] = []
        self._pool = ThreadPoolExecutor(max_workers=1)   # un fil d'arrière-plan : la mission n'attend jamais

    def repond_texte(self, texte: str) -> str:
        """Renvoie la réponse brute du modèle à un texte seul (sans hasard) et note sa latence."""
        import torch
        messages = [{"role": "user", "content": [{"type": "text", "text": texte}]}]
        prompt = self.processor.apply_chat_template(messages, add_generation_prompt=True)
        ids = self.processor.tokenizer(prompt, return_tensors="pt")["input_ids"].to(self.device)
        t0 = time.perf_counter()
        with torch.no_grad():
            sortie = self.modele.generate(input_ids=ids, attention_mask=torch.ones_like(ids),
                                          max_new_tokens=self.max_tokens, do_sample=False)
        self.latences.append(time.perf_counter() - t0)
        texte = self.processor.tokenizer.decode(sortie[0, ids.shape[1]:], skip_special_tokens=True).strip()
        self.reponses.append(texte)
        return texte

    def conseille(self, cas: dict, zones: list[dict]) -> planning.Avis | None:
        """Renvoie l'avis du guide : la zone lue dans « ANSWER: n », avec le côté de ses faces ; None si
        la réponse ne désigne aucune zone."""
        reponse = self.repond_texte(texte_pour_le_guide(cas, zones))
        m = re.search(r"ANSWER\s*[:=]?\s*(\d+)", reponse, re.IGNORECASE) or re.search(r"(\d+)", reponse)
        if not m:
            return None
        numero = int(m.group(1))
        z = next((z for z in zones if z["numero"] == numero), None)
        if z is None:
            return None
        # le côté n'est pas demandé au modèle : c'est celui des faces de la zone, lu sur la carte
        inverse = {nom: k for k, nom in planning.NOMS_COTES.items()}
        cote = inverse.get(z.get("cote"))
        phrase = f"zone {numero}" + (f", cote {z['cote']}" if z.get("cote") else "") + f" ({reponse[:20]!r})"
        return planning.Avis(centre=np.array([*z["centre"][:2], 0.0]), rayon=float(z["rayon"]), cote=cote, phrase=phrase)

    def demande(self, cas: dict, zones: list[dict]) -> Future:
        """Lance `conseille` en arrière-plan et renvoie aussitôt un Future (résultat à venir)."""
        return self._pool.submit(self.conseille, cas, list(zones))

    def bilan(self) -> dict:
        """Renvoie le nom du modèle, le nombre d'appels et la latence médiane (s)."""
        return {"modele": self.nom, "appels": len(self.latences),
                "latence_mediane_s": round(float(np.median(self.latences)), 2) if self.latences else None}


class Guide:
    """Le guide généraliste : un modèle vision-langage (SmolVLM ou Qwen) qui répond à partir d'images."""

    def __init__(self, nom: str = "smolvlm", max_tokens: int = 60, device: str = "cuda",
                 quantisation: str | None = None):
        """Charge le modèle `nom` (alias de MODELES ou chemin) ; `quantisation` : None (16 bits), "4bit",
        "8bit" ou "4bit-vision16" (encodeur d'images gardé en 16 bits)."""
        os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
        import torch
        from transformers import AutoModelForImageTextToText, AutoProcessor

        self.nom = MODELES.get(nom, nom)
        options = {"max_pixels": MAX_PIXELS} if "qwen" in self.nom.lower() else {}
        self.processor = AutoProcessor.from_pretrained(self.nom, **options)
        self.dtype = torch.bfloat16 if device == "cpu" else torch.float16
        self.device = "cuda" if device == "auto" else device
        if quantisation:
            from transformers import BitsAndBytesConfig
            if quantisation == "8bit":
                config = BitsAndBytesConfig(load_in_8bit=True)
            else:
                config = BitsAndBytesConfig(load_in_4bit=True, bnb_4bit_quant_type="nf4",
                                            bnb_4bit_compute_dtype=torch.float16,
                                            bnb_4bit_use_double_quant=True)
            if quantisation.endswith("vision16"):
                # l'encodeur d'images et la couche de sortie ne sont pas compressés
                config.llm_int8_skip_modules = ["visual", "lm_head"]
            self.modele = AutoModelForImageTextToText.from_pretrained(
                self.nom, quantization_config=config, device_map={"": self.device}, dtype=torch.float16,
                attn_implementation="sdpa")
        elif device == "auto":
            # « auto » : le modèle est réparti entre la carte graphique et la mémoire centrale
            self.modele = AutoModelForImageTextToText.from_pretrained(
                self.nom, dtype=torch.float16, device_map="auto", attn_implementation="sdpa")
        else:
            self.modele = AutoModelForImageTextToText.from_pretrained(self.nom, dtype=self.dtype,
                                                                    attn_implementation="sdpa").to(device)
        self.modele.eval()
        self.max_tokens = max_tokens
        self.latences: list[float] = []
        self.reponses: list[str] = []
        self._pool = ThreadPoolExecutor(max_workers=1)

    def repond(self, image_bgr: np.ndarray, vue_bgr: np.ndarray, question: str) -> str:
        """Renvoie la réponse brute du modèle à `question`, avec deux images : la caméra du drone et la
        carte vue de dessus."""
        return self.repond_images([image_bgr, vue_bgr], question)

    def repond_images(self, images_bgr: list, question: str, hasard: float = 0.0) -> str:
        """Renvoie la réponse brute du modèle avec un nombre libre d'images (même aucune) ; `hasard` > 0 :
        tirage avec cette température (pour les votes), sinon réponse toujours identique."""
        import torch
        from PIL import Image

        images = [Image.fromarray(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)) for im in images_bgr]
        messages = [{"role": "user", "content": [{"type": "image"} for _ in images]
                                               + [{"type": "text", "text": question}]}]
        prompt = self.processor.apply_chat_template(messages, add_generation_prompt=True)
        entrees = self.processor(text=prompt, images=images or None, return_tensors="pt")
        entrees = {k: (v.to(self.device, dtype=self.dtype) if v.dtype.is_floating_point else v.to(self.device))
                   for k, v in entrees.items()}
        t0 = time.perf_counter()
        with torch.no_grad():
            sortie = self.modele.generate(**entrees, max_new_tokens=self.max_tokens,
                                          **({"do_sample": True, "temperature": hasard, "top_k": 50,
                                              "renormalize_logits": True} if hasard > 0
                                             else {"do_sample": False}))
        self.latences.append(time.perf_counter() - t0)
        texte = self.processor.batch_decode(sortie[:, entrees["input_ids"].shape[1]:], skip_special_tokens=True)[0]
        texte = texte.strip()
        self.reponses.append(texte)
        return texte

    @staticmethod
    def zone_dans(texte: str, zones: list[dict]) -> int | None:
        """Renvoie le numéro de zone lu dans la réponse (« zone 3 », « 3. », « Zone: 3 »), ou None s'il
        ne correspond à aucune zone proposée."""
        numeros = {z["numero"] for z in zones}
        m = re.search(r"zone\s*[:=]?\s*(\d+)", texte, re.IGNORECASE) or re.search(r"(\d+)", texte)
        if not m:
            return None
        n = int(m.group(1))
        return n if n in numeros else None

    @staticmethod
    def cote_dans(texte: str) -> int | None:
        """Renvoie le numéro du premier côté cité dans la réponse (north, sud…, voir COTES), ou None."""
        for mot in re.findall(r"[a-zA-Zéè]+", texte.lower()):
            if mot in COTES:
                return COTES[mot]
        return None

    def conseille(self, image_bgr, vue_bgr, zones: list[dict], position=None,
                  description: str | None = None) -> planning.Avis | None:
        """Renvoie l'avis du guide (ou None) en deux questions courtes, la zone puis le côté : un petit
        modèle suit mieux une consigne à la fois."""
        desc = description or ""
        numero = self.zone_dans(self.repond(image_bgr, vue_bgr, QUESTION_ZONE.format(description=desc)), zones)
        if numero is None:
            return None
        texte_cote = self.repond(image_bgr, vue_bgr, QUESTION_COTE.format(description=desc, n=numero))
        cote = self.cote_dans(texte_cote)
        z = next(z for z in zones if z["numero"] == numero)
        phrase = f"zone {numero}" + (f", cote {planning.NOMS_COTES[cote]}" if cote is not None else "")
        return planning.Avis(centre=np.array([*z["centre"][:2], 0.0]), rayon=float(z["rayon"]), cote=cote,
                             phrase=phrase + f" ({self.reponses[-2][:40]!r} / {texte_cote[:40]!r})")

    def demande(self, image_bgr, vue_bgr, zones, position=None, description: str | None = None) -> Future:
        """Lance `conseille` en arrière-plan sur une copie des images et renvoie aussitôt un Future."""
        return self._pool.submit(self.conseille, image_bgr.copy(), vue_bgr.copy(), zones, position, description)

    def bilan(self) -> dict:
        """Renvoie le nom du modèle, le nombre d'appels et la latence médiane (s)."""
        return {"modele": self.nom, "appels": len(self.latences),
                "latence_mediane_s": round(float(np.median(self.latences)), 2) if self.latences else None}
