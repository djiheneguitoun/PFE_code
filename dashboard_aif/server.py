#!/usr/bin/env python3
"""Serveur du tableau de bord de l'inférence active (AIF) : sert la page web et une API JSON (port 8060).

Relit à chaque requête les fichiers écrits dans /tmp par scripts/12_aif_isaac_sim.py (état et
historique AIF, décodage des QR codes, images des caméras, latences NS-3) et liste les vols
enregistrés dans logs/runs/. Lancement depuis la racine du projet :
python3 dashboard_aif/server.py [--port 8060] [--runs-dir logs/runs], puis ouvrir http://localhost:8060
"""

import argparse
import csv
import json
import mimetypes
import os
import re
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import unquote

# Dossier servi au navigateur (index.html, app.js, style.css) : celui de ce fichier.
DASHBOARD_DIR = os.path.dirname(os.path.abspath(__file__))
# Écrits par scripts/12_aif_isaac_sim.py (dossier de sortie par défaut : /tmp) : état courant et
# historique AIF (aif_core/loggers.py), état du décodage QR et images des caméras (qr_code_system.py).
STATE_PATH = "/tmp/aif_state.json"
HISTORY_PATH = "/tmp/aif_history.json"
QR_STATE_PATH = "/tmp/qr_state.json"
CAMERA_FRAMES_DIR = "/tmp/camera_frames"

# ── Latences NS-3 (simulateur de réseau), essayées dans cet ordre : WiFi, 5G, 5G brut ──
NS3_WIFI_CSV = "/tmp/ns3_output.csv"
NS3_LTE_CSV  = "/tmp/drone_latency_ns3.csv"
NS3_5G_METRICS = "/tmp/drone_5g_metrics.csv"

# ── Dossier des vols enregistrés : <racine>/logs/runs, sauf variable AIF_RUNS_DIR ou option --runs-dir ──
_WORKSPACE_ROOT = os.path.dirname(DASHBOARD_DIR)  # racine du projet (parent de dashboard_aif/)
DEFAULT_RUNS_DIR = os.environ.get(
    "AIF_RUNS_DIR",
    os.path.join(_WORKSPACE_ROOT, "logs", "runs"),
)
RUNS_DIR = DEFAULT_RUNS_DIR  # remplacé par la valeur de --runs-dir dans main()

# Noms acceptés dans /api/run/<tag>/img/<nom> (lettres, chiffres, « _ . - ») : interdit de sortir de logs/runs
SAFE_IMG_RE = re.compile(r"^[a-zA-Z0-9_.\-]+\.(png|jpg|jpeg|svg)$")
SAFE_TAG_RE = re.compile(r"^[a-zA-Z0-9_.\-]+$")


def _read_ns3_csv(path: str):
    """Renvoie la dernière mesure de chaque paire d'un CSV NS-3 (WiFi ou 5G) : {a, b, latency_ms, jitter_ms, rx_packets}.
    Les lignes mal formées (fichier en cours d'écriture) sont ignorées ; liste vide si le fichier manque."""
    if not os.path.isfile(path):
        return []
    out = []
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    i_key = "drone_i" if "drone_i" in row else "drone_a"
                    j_key = "drone_j" if "drone_j" in row else "drone_b"
                    a = int(row[i_key])
                    b = int(row[j_key])
                    lat = float(row.get("latency_ms", 0.0))
                    jitter = float(row.get("jitter_ms", 0.0)) if "jitter_ms" in row else 0.0
                    rx = int(row.get("rx_packets", 0)) if "rx_packets" in row else 0
                    out.append({
                        "a": min(a, b), "b": max(a, b),
                        "latency_ms": round(lat, 3),
                        "jitter_ms": round(jitter, 3),
                        "rx_packets": rx,
                    })
                except (ValueError, KeyError):
                    continue
    except (IOError, OSError):
        return out
    # déduplication : garder la dernière mesure par paire
    by_pair = {}
    for row in out:
        by_pair[(row["a"], row["b"])] = row
    return list(by_pair.values())


def _ns3_state() -> dict:
    """Construit la réponse de /api/ns3 : latences par paire (CSV NS-3, sinon aif_state.json) et état du réseau simulé.
    L'état (lien cloud, file d'attente, messages envoyés / livrés / perdus, liens coupés) vient de aif_state.json."""
    pairs = []
    # essai wifi puis 5g
    if os.path.exists(NS3_WIFI_CSV):
        pairs = _read_ns3_csv(NS3_WIFI_CSV)
    if not pairs and os.path.exists(NS3_LTE_CSV):
        pairs = _read_ns3_csv(NS3_LTE_CSV)
    if not pairs and os.path.exists(NS3_5G_METRICS):
        pairs = _read_ns3_csv(NS3_5G_METRICS)

    network = {}
    ns3_mode = "none"
    cloud_link = True
    queue_size = 0
    dropped = 0
    sent = 0
    delivered = 0
    cut_pairs = []
    all_cut = False
    try:
        with open(STATE_PATH) as f:
            state = json.load(f)
        ns3_mode = state.get("ns3_mode", "none")
        net = state.get("network", {}) or {}
        cloud_link = bool(net.get("cloud_link_active", True))
        queue_size = int(net.get("queue_size", 0))
        dropped    = int(net.get("msg_dropped", 0))
        sent       = int(net.get("msg_sent", 0))
        delivered  = int(net.get("msg_delivered", 0))
        cut_pairs  = list(net.get("cut_pairs", []))
        all_cut    = bool(net.get("all_drone_links_cut", False))
        # si aif_state.json contient déjà les paires, on les utilise (plus à jour)
        if not pairs and isinstance(net.get("ns3_pairs"), list):
            pairs = net["ns3_pairs"]
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        pass

    return {
        "ns3_mode": ns3_mode,
        "cloud_link": "up" if cloud_link else "down",
        "pairs": pairs,
        "queue_size": queue_size,
        "dropped": dropped,
        "sent": sent,
        "delivered": delivered,
        "cut_pairs": cut_pairs,
        "all_drone_links_cut": all_cut,
    }


def _runs_list() -> dict:
    """Renvoie les vols enregistrés (dossiers run_* de RUNS_DIR, du plus récent au plus ancien) avec config.json et images."""
    runs = []
    if os.path.isdir(RUNS_DIR):
        for name in sorted(os.listdir(RUNS_DIR), reverse=True):
            run_path = os.path.join(RUNS_DIR, name)
            if not os.path.isdir(run_path) or not name.startswith("run_"):
                continue
            cfg = {}
            cfg_path = os.path.join(run_path, "config.json")
            try:
                if os.path.isfile(cfg_path):
                    with open(cfg_path) as f:
                        cfg = json.load(f)
            except (IOError, json.JSONDecodeError):
                cfg = {}
            # Images disponibles
            images = []
            try:
                for fn in sorted(os.listdir(run_path)):
                    if SAFE_IMG_RE.match(fn):
                        images.append(fn)
            except OSError:
                pass
            try:
                mtime = datetime.fromtimestamp(os.path.getmtime(run_path)).isoformat(timespec="seconds")
            except OSError:
                mtime = ""
            runs.append({
                "tag": name,
                "config": cfg,
                "images": images,
                "mtime": mtime,
            })
    return {"runs_dir": RUNS_DIR, "runs": runs}


class DashboardHandler(SimpleHTTPRequestHandler):
    """Gestionnaire HTTP : sert les fichiers de dashboard_aif/ et les routes JSON /api/... lues par la page."""

    def __init__(self, *args, **kwargs):
        """Prépare le serveur de fichiers en le limitant au dossier dashboard_aif/."""
        super().__init__(*args, directory=DASHBOARD_DIR, **kwargs)

    def do_GET(self):
        """Aiguille une requête GET vers la bonne route /api/..., ou sert un fichier statique (page, JS, CSS)."""
        # routes de l'API
        if self.path == "/api/state":
            self._serve_json_file(STATE_PATH)
        elif self.path == "/api/history":
            self._serve_json_file(HISTORY_PATH)
        elif self.path == "/api/qr_state":
            self._serve_json_file(QR_STATE_PATH)
        elif self.path == "/api/ns3":
            self._serve_json_obj(_ns3_state())
        elif self.path == "/api/runs":
            self._serve_json_obj(_runs_list())
        elif self.path.startswith("/api/frame/"):
            self._serve_frame(self.path)
        elif self.path.startswith("/api/run/"):
            self._serve_run_img(self.path)
        else:
            super().do_GET()

    # ── Envoi des réponses ──
    def _serve_json_obj(self, obj):
        """Envoie l'objet Python `obj` en JSON compact (code 200, sans cache)."""
        data = json.dumps(obj, separators=(",", ":")).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_json_file(self, filepath: str):
        """Envoie tel quel un fichier JSON de /tmp ; répond 404 « data not available yet » s'il n'existe pas encore."""
        try:
            with open(filepath, "r") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data.encode())
        except FileNotFoundError:
            self.send_response(404)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"error":"data not available yet"}')

    def _serve_frame(self, path: str):
        """Envoie une image caméra de /tmp/camera_frames : latest_drone_<id>.jpg (brute) ou annotated_drone_<id>.jpg (QR encadré).
        Refuse (400) un nom contenant « / » ou « .. » ; répond 404 si l'image n'existe pas encore."""
        filename = path.split("/api/frame/")[-1]
        if "?" in filename:
            filename = filename.split("?")[0]
        if "/" in filename or ".." in filename:
            self.send_response(400)
            self.end_headers()
            return

        filepath = os.path.join(CAMERA_FRAMES_DIR, filename)
        if not os.path.isfile(filepath):
            self.send_response(404)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"error":"frame not found"}')
            return

        mime, _ = mimetypes.guess_type(filepath)
        mime = mime or "image/jpeg"
        try:
            with open(filepath, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except Exception:
            self.send_response(500)
            self.end_headers()

    def _serve_run_img(self, path: str):
        """Envoie une image d'un vol enregistré (/api/run/<tag>/img/<nom>) après avoir vérifié que le chemin reste dans RUNS_DIR."""
        # On retire un éventuel « ?… » ajouté à l'URL (souvent pour contourner le cache du navigateur)
        clean = path.split("?", 1)[0]
        m = re.match(r"^/api/run/([^/]+)/img/(.+)$", clean)
        if not m:
            self.send_response(404)
            self.end_headers()
            return
        tag = unquote(m.group(1))
        name = unquote(m.group(2))
        if not SAFE_TAG_RE.match(tag) or not SAFE_IMG_RE.match(name):
            self.send_response(400)
            self.end_headers()
            return
        filepath = os.path.join(RUNS_DIR, tag, name)
        # Sécurité : le chemin réel (liens symboliques résolus) doit rester dans RUNS_DIR
        try:
            real = os.path.realpath(filepath)
            real_root = os.path.realpath(RUNS_DIR)
            if not real.startswith(real_root + os.sep):
                self.send_response(400)
                self.end_headers()
                return
        except OSError:
            self.send_response(500)
            self.end_headers()
            return
        if not os.path.isfile(filepath):
            self.send_response(404)
            self.end_headers()
            return
        mime, _ = mimetypes.guess_type(filepath)
        mime = mime or "image/png"
        try:
            with open(filepath, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except OSError:
            self.send_response(500)
            self.end_headers()

    def log_message(self, format, *args):
        """Allège le terminal : n'affiche pas les requêtes réussies (code 200), seulement les autres."""
        if args and "200" not in str(args[0]):
            super().log_message(format, *args)


def main():
    """Lit les options (--port, défaut 8060 ; --runs-dir, défaut <racine>/logs/runs) et sert jusqu'à Ctrl+C."""
    global RUNS_DIR
    parser = argparse.ArgumentParser(description="AIF Dashboard Server")
    parser.add_argument("--port", type=int, default=8060)
    parser.add_argument("--runs-dir", default=DEFAULT_RUNS_DIR,
                        help="Path to logs/runs/ (default: <workspace>/logs/runs)")
    args = parser.parse_args()
    RUNS_DIR = os.path.abspath(args.runs_dir)

    server = HTTPServer(("0.0.0.0", args.port), DashboardHandler)
    print(f"[Dashboard] http://localhost:{args.port}")
    print(f"[Dashboard] Reading {STATE_PATH}")
    print(f"[Dashboard] QR state: {QR_STATE_PATH}")
    print(f"[Dashboard] Camera frames: {CAMERA_FRAMES_DIR}")
    print(f"[Dashboard] Runs dir   : {RUNS_DIR}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Dashboard] Stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
