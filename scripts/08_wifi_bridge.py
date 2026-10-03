#!/usr/bin/env python3
"""Phase 08 : pont réseau WiFi — mesure en continu la qualité du lien entre chaque paire de drones.

RSSI (puissance reçue, en dBm) calculé par lancer de rayons Sionna dans la scène 3D de l'entrepôt ; latence (ms)
mesurée par ns-3 (simulateur de réseau, scénario drone-wifi-scenario), ou estimée si ns-3 est absent.
Lit /tmp/drone_positions.csv (écrit par 07 / 07b) ; écrit /tmp/drone_rssi_latency.csv et le journal /tmp/drone_bridge_log.csv.
Usage : python3 scripts/08_wifi_bridge.py [--test] [--cycles N] [--no-render] [--no-ns3] [--csv chemin]
"""

import argparse, csv, itertools, math, os, signal, subprocess, sys, time
from datetime import datetime
import numpy as np

# Fichiers : scène 3D de l'entrepôt pour Sionna (fournie par ns3-sionna), positions lues, résultats écrits
SCENE_XML = os.path.expanduser(
    "~/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna/"
    "models/warehouse/warehouse.xml"
)
POS_CSV    = "/tmp/drone_positions.csv"    # entrée : « id,x,y,z » (m) par drone
RSSI_CSV   = "/tmp/drone_rssi_latency.csv" # dernière mesure par paire (écrasé à chaque cycle)
LOG_CSV    = "/tmp/drone_bridge_log.csv"   # journal : une ligne par paire et par cycle
RENDER_DIR = "/tmp/sionna_renders"
RENDER_PNG = os.path.join(RENDER_DIR, "latest.png")  # image vue de dessus (rendu Sionna)

# ns-3 (scénario WiFi entre drones, sources dans scenarios/)
NS3_DIR      = os.path.expanduser("~/ns-allinone-3.40/ns-3.40")
NS3_BIN      = os.path.join(NS3_DIR, "ns3")
NS3_OUT_CSV  = "/tmp/ns3_output.csv"       # latences écrites par ns-3
NS3_SCENARIO = "drone-wifi-scenario"

# Paramètres radio WiFi 802.11ax utilisés par Sionna
TX_POWER_DBM  = 20.0   # dBm : puissance d'émission
FREQUENCY_GHZ = 5.18   # GHz : canal WiFi de la bande 5 GHz
BANDWIDTH_MHZ = 20     # MHz

# Couleurs (rouge, vert, bleu) des drones 0, 1, 2 dans le rendu
COLORS = [(1, 0, 0), (0, 0.8, 0), (0.2, 0.4, 1)]

running = True  # passe à False sur Ctrl+C


def signal_handler(sig, frame):
    """Demande l'arrêt de la boucle principale (Ctrl+C)."""
    global running
    running = False
    print("\n  Stopping...")




def read_positions_csv(path):
    """Lit le CSV des positions (« id,x,y,z » ou « temps,id,x,y,z ») et renvoie {id: (x, y, z)}, dernière ligne par drone."""
    latest = {}
    try:
        with open(path) as f:
            for row in csv.reader(f):
                if not row or row[0].startswith("#") or row[0].strip() in ("drone_id","timestamp"):
                    continue
                try:
                    if len(row) >= 5:
                        did, x, y, z = int(row[1]), float(row[2]), float(row[3]), float(row[4])
                    else:
                        did, x, y, z = int(row[0]), float(row[1]), float(row[2]), float(row[3])
                    latest[did] = (x, y, z)
                except (ValueError, IndexError):
                    continue
    except FileNotFoundError:
        pass
    return latest


def fake_positions(tick):
    """Renvoie 3 positions fictives qui tournent en rond dans l'entrepôt (mode --test, sans SITL)."""
    t = tick * 0.3
    return {
        0: (-3.0 + 1.5 * math.sin(t),       1.0 * math.cos(t),       4.0),
        1: ( 0.0 + 1.5 * math.sin(t + 2.1), 1.0 * math.cos(t + 2.1), 5.0),
        2: ( 3.0 + 1.5 * math.sin(t + 4.2), 1.0 * math.cos(t + 4.2), 6.0),
    }



_ns3_process = None  # processus ns-3 lancé en arrière-plan (ou None)


def launch_ns3(n_drones=3, sim_time=300):
    """Lance ns-3 en arrière-plan (WiFi ad hoc, canal « log-distance ») : il relit le CSV des positions, mesure la latence
    (FlowMonitor) et l'écrit dans NS3_OUT_CSV. Renvoie vrai si le processus tourne encore après 2 s."""
    global _ns3_process

    if not os.path.isfile(NS3_BIN):
        print(f"  ⚠ NS-3 non trouvé : {NS3_BIN}")
        print(f"    → Latence sera estimée (propagation uniquement)")
        return False

    if os.path.exists(NS3_OUT_CSV):
        os.remove(NS3_OUT_CSV)

    cmd = [
        NS3_BIN, "run",
        f"{NS3_SCENARIO} "
        f"--nDrones={n_drones} "
        f"--posFile={POS_CSV} "
        f"--outFile={NS3_OUT_CSV} "
        f"--simTime={sim_time} "
        f"--channelModel=log-distance",
        "--no-build",
    ]
    print(f"  ⏳ Lancement NS-3 WiFi ({NS3_SCENARIO})...")
    try:
        _ns3_process = subprocess.Popen(
            cmd, cwd=NS3_DIR,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
        )
        time.sleep(2)
        if _ns3_process.poll() is not None:
            err = _ns3_process.stderr.read().decode()[-300:]
            print(f"  ⚠ NS-3 a crashé : {err}")
            _ns3_process = None
            return False
        print(f"  ✓ NS-3 lancé en arrière-plan (PID {_ns3_process.pid})")
        return True
    except Exception as e:
        print(f"  ⚠ Erreur lancement NS-3 : {e}")
        _ns3_process = None
        return False


def read_ns3_latency():
    """Lit le CSV de ns-3 et renvoie {(i, j): latence en ms} avec i < j, dernière mesure par paire."""
    latest = {}
    try:
        with open(NS3_OUT_CSV) as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    i = int(row["drone_i"])
                    j = int(row["drone_j"])
                    lat = float(row["latency_ms"])
                    key = (min(i, j), max(i, j))
                    latest[key] = lat
                except (ValueError, KeyError):
                    continue
    except FileNotFoundError:
        pass
    return latest


def stop_ns3():
    """Arrête le processus ns-3 (terminate, puis kill s'il tourne encore après 5 s)."""
    global _ns3_process
    if _ns3_process and _ns3_process.poll() is None:
        _ns3_process.terminate()
        try:
            _ns3_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _ns3_process.kill()
        print(f"  NS-3 arrêté.")
    _ns3_process = None



def compute_rssi(scene, pos_a, pos_b):
    """Trace les rayons entre deux positions (Sionna) et renvoie (affaiblissement en dB, délai de propagation en ns), ou
    (None, None) sans trajet. Ce délai physique (~ns) n'est pas la latence réseau, fournie par ns-3 (~ms)."""
    from sionna.rt import Transmitter, Receiver, PlanarArray, PathSolver

    # Retire l'émetteur et le récepteur du calcul précédent
    for name in list(scene.transmitters.keys()):
        scene.remove(name)
    for name in list(scene.receivers.keys()):
        scene.remove(name)

    scene.add(Transmitter(name="tx", position=list(pos_a), orientation=[0, 0, 0]))
    scene.add(Receiver(name="rx", position=list(pos_b), orientation=[0, 0, 0]))

    scene.tx_array = PlanarArray(num_rows=1, num_cols=1,
                                  vertical_spacing=0.5, horizontal_spacing=0.5,
                                  pattern="dipole", polarization="V")
    scene.rx_array = PlanarArray(num_rows=1, num_cols=1,
                                  vertical_spacing=0.5, horizontal_spacing=0.5,
                                  pattern="dipole", polarization="V")

    solver = PathSolver()
    paths = solver(scene=scene, max_depth=3, samples_per_src=10**6,
                   los=True, specular_reflection=True, diffuse_reflection=False,
                   refraction=True, synthetic_array=False,
                   diffraction=False, edge_diffraction=False)

    a, tau = paths.cir(sampling_frequency=1e9, normalize_delays=False, out_type="numpy")
    tau_flat = np.squeeze(tau)
    valid = tau_flat[tau_flat >= 0]

    if len(valid) == 0:
        return None, None

    delay_ns = int(round(np.min(valid) * 1e9))

    # Puissance moyenne reçue sur 64 sous-porteuses du canal de 20 MHz → affaiblissement en dB
    num_sc = 64
    sc_spacing = BANDWIDTH_MHZ * 1e6 / num_sc
    freqs = np.arange(num_sc) * sc_spacing
    freqs = freqs - np.mean(freqs) + FREQUENCY_GHZ * 1e9

    h = np.squeeze(paths.cfr(frequencies=freqs, sampling_frequency=1.0,
                              num_time_steps=1, normalize_delays=True,
                              normalize=False, out_type="numpy"))
    mean_power = np.mean(np.abs(h) ** 2)
    if mean_power <= 0:
        return None, None

    return float(-10 * np.log10(mean_power)), delay_ns




def render_scene(scene, positions):
    """Enregistre dans RENDER_PNG une vue de dessus (caméra à 25 m) de l'entrepôt avec un point coloré par drone."""
    from sionna.rt import Transmitter, Camera

    for name in list(scene.transmitters.keys()):
        scene.remove(name)
    for name in list(scene.receivers.keys()):
        scene.remove(name)

    for did, (x, y, z) in sorted(positions.items()):
        color = COLORS[did % len(COLORS)]
        scene.add(Transmitter(
            name=f"drone_{did}", position=[x, y, z],
            color=color, power_dbm=20, display_radius=0.5
        ))

    cam = Camera(position=[0, 0, 25], look_at=[0, 0, 0])
    os.makedirs(RENDER_DIR, exist_ok=True)
    scene.render_to_file(
        camera=cam, filename=RENDER_PNG,
        resolution=(1280, 960), num_samples=64,
        show_devices=True, show_orientations=False,
        clip_at=5.9
    )




def write_fake_positions_csv(positions):
    """Écrit les positions fictives dans le CSV des positions (mode --test seulement)."""
    with open(POS_CSV, "w") as f:
        for did, (x, y, z) in sorted(positions.items()):
            f.write(f"{did},{x:.4f},{y:.4f},{z:.4f}\n")


def write_rssi_csv(results):
    """Réécrit RSSI_CSV : une ligne par paire avec RSSI (Sionna), latence (ns-3 ou estimée), distance et source."""
    with open(RSSI_CSV, "w") as f:
        f.write("drone_a,drone_b,rssi_dBm,latency_ms,distance_m,latency_source\n")
        for row in results:
            f.write(",".join(str(v) for v in row) + "\n")


def append_log(tick, positions, results):
    """Ajoute au journal LOG_CSV une ligne par paire : heure, n° de cycle, mesures et positions des deux drones."""
    ts = time.strftime("%H:%M:%S")
    write_header = not os.path.exists(LOG_CSV) or os.path.getsize(LOG_CSV) == 0
    with open(LOG_CSV, "a") as f:
        if write_header:
            f.write("time,tick,drone_a,drone_b,rssi_dBm,latency_ms,dist_m,"
                    "latency_source,ax,ay,az,bx,by,bz\n")
        for row in results:
            id_a, id_b, rssi, latency, dist = row[0], row[1], row[2], row[3], row[4]
            lat_src = row[5] if len(row) > 5 else ""
            pa = positions[id_a]
            pb = positions[id_b]
            f.write(f"{ts},{tick},{id_a},{id_b},{rssi},{latency},{dist},{lat_src},"
                    f"{pa[0]:.2f},{pa[1]:.2f},{pa[2]:.2f},"
                    f"{pb[0]:.2f},{pb[1]:.2f},{pb[2]:.2f}\n")


W = 72  # largeur d'affichage (caractères)


def rssi_bar(rssi_val):
    """Renvoie une barre de qualité selon le RSSI : > -50 dBm excellent, > -60 bon, > -70 moyen, sinon faible."""
    if rssi_val > -50:
        return "█████ Excellent"
    elif rssi_val > -60:
        return "████░ Good"
    elif rssi_val > -70:
        return "███░░ Fair"
    else:
        return "██░░░ Weak"


def print_table(tick, positions, results, elapsed):
    """Affiche un bloc par cycle : positions, puis distance, RSSI, latence et qualité de chaque paire (ou BLOCKED)."""
    now = datetime.now().strftime("%H:%M:%S")

    # ── séparateur et en-tête du cycle ──
    label = f"  Tick {tick}  ·  {now}  ·  {elapsed:.1f}s  "
    print(f"\n{'─' * 4}{label}{'─' * max(4, W - 4 - len(label))}")

    # ── positions sur une ligne ──
    pos_parts = []
    for did in sorted(positions.keys()):
        x, y, z = positions[did]
        pos_parts.append(f"D{did}({x:.1f},{y:.1f},{z:.1f})")
    print(f"  Pos: {'  '.join(pos_parts)}")

    # ── RSSI et latence par paire ──
    for i, row in enumerate(results):
        id_a, id_b, rssi, latency, dist = row[0], row[1], row[2], row[3], row[4]
        lat_src = row[5] if len(row) > 5 else ""
        if rssi == "blocked" or rssi == "error":
            line = f"  {id_a}↔{id_b}  {dist:>6}m   {'BLOCKED':>10}   {'---':>8}     ⛔ No path"
        else:
            rssi_f = float(rssi)
            bar = rssi_bar(rssi_f)
            src_tag = f"[{lat_src}]" if lat_src else ""
            line = f"  {id_a}↔{id_b}  {dist:>6}m  {rssi:>8} dBm  {latency:>6} ms  {bar} {src_tag}"
        print(line)



def main():
    """Charge la scène Sionna, lance ns-3, puis à chaque nouvelle position calcule et enregistre RSSI et latence de chaque paire."""
    parser = argparse.ArgumentParser(description="Live Bridge: RSSI (Sionna) + Latence (NS-3 WiFi)")
    parser.add_argument("--test", action="store_true",
                        help="Fake moving drones (no SITL needed)")
    parser.add_argument("--csv", default=POS_CSV,
                        help=f"Positions CSV to watch (default: {POS_CSV})")
    parser.add_argument("--cycles", type=int, default=0,
                        help="Number of cycles (0 = infinite)")
    parser.add_argument("--no-render", action="store_true",
                        help="Skip 3D render (faster)")
    parser.add_argument("--no-ns3", action="store_true",
                        help="Skip NS-3 (use propagation delay estimation only)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    print()

    # Charge la scène Sionna de l'entrepôt
    print("\n  Loading Sionna scene...")
    from sionna.rt import load_scene
    scene = load_scene(SCENE_XML, merge_shapes=False)
    scene.frequency = FREQUENCY_GHZ * 1e9
    scene.bandwidth = BANDWIDTH_MHZ * 1e6
    print(f"  ✓ Scene loaded — {len(scene.objects)} objects")

    # Lance ns-3 WiFi (en mode test, écrit d'abord des positions fictives pour qu'il démarre)
    ns3_ok = False
    if not args.no_ns3:
        if args.test:
            write_fake_positions_csv(fake_positions(0))
        n_drones = 3
        ns3_ok = launch_ns3(n_drones=n_drones, sim_time=600)
    else:
        print("   NS-3 désactivé (--no-ns3) — latence estimée uniquement")

    if os.path.exists(LOG_CSV):
        os.remove(LOG_CSV)

    print(f"   Ready — Ctrl+C to stop")
    if not args.test:
        print(f"   Start script 08 in another terminal to fly drones")

    tick = 0
    last_positions = {}
    while running:
        t0 = time.time()

        # 1. Positions : fictives (--test) ou lues dans le CSV ; il faut au moins 2 drones
        if args.test:
            positions = fake_positions(tick)
            write_fake_positions_csv(positions)
        else:
            positions = read_positions_csv(args.csv)
            if len(positions) < 2:
                if tick % 10 == 0: 
                    print(f"  Waiting for positions in {args.csv}... "
                          f"(found {len(positions)} drones)")
                time.sleep(1)
                tick += 1
                continue

        # 2. Rien à recalculer si les positions n'ont pas changé
        if positions == last_positions:
            time.sleep(0.5)
            tick += 1
            continue
        last_positions = dict(positions)

        # 3. Mesures pour chaque paire de drones
        drone_ids = sorted(positions.keys())
        pairs = list(itertools.combinations(drone_ids, 2))
        results = []

        # Dernières latences écrites par ns-3
        ns3_lat = read_ns3_latency() if ns3_ok else {}

        for id_a, id_b in pairs:
            pa, pb = positions[id_a], positions[id_b]
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(pa, pb)))

            # RSSI (dBm) = puissance émise - affaiblissement calculé par Sionna
            try:
                pl, _prop_delay = compute_rssi(scene, pa, pb)
                if pl is None:
                    rssi_str = "blocked"
                else:
                    rssi_str = f"{TX_POWER_DBM - pl:.2f}"
            except Exception:
                rssi_str = "error"

            # Latence : valeur ns-3 si disponible, sinon estimation = propagation (distance / c) + 2 ms fixes
            pair_key = (min(id_a, id_b), max(id_a, id_b))
            lat_ms = ns3_lat.get(pair_key)
            if lat_ms is not None:
                lat_str = f"{lat_ms:.2f}"
                lat_src = "ns3"
            elif rssi_str not in ("blocked", "error"):
                lat_str = f"{dist / 3e8 * 1e3 + 2.0:.2f}"
                lat_src = "est"
            else:
                lat_str = "---"
                lat_src = ""

            results.append((id_a, id_b, rssi_str, lat_str, f"{dist:.2f}", lat_src))

        # 4. Dernières mesures dans RSSI_CSV
        write_rssi_csv(results)

        # 5. Ajout au journal
        append_log(tick, positions, results)

        # 6. Image vue de dessus (une erreur de rendu est ignorée)
        if not args.no_render:
            try:
                render_scene(scene, positions)
            except Exception as e:
                pass  

        elapsed = time.time() - t0

        # 7. Affichage du cycle
        print_table(tick, positions, results, elapsed)

        tick += 1
        if args.cycles > 0 and tick >= args.cycles:
            break

    stop_ns3()
    print(f"\n{'─' * W}")
    print(f"  Bridge stopped.  Log → {LOG_CSV}")


if __name__ == "__main__":
    main()
