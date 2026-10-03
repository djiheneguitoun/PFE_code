"""Affiche ce que chaque pilote ArduPilot a écrit dans son journal (.BIN) depuis un instant : modes, erreurs, messages.

Sert à comprendre une chute vue de l'intérieur du drone (lit <dossier>/ardupilot_logs/drone_*/*.BIN, sans simulateur).
    journaux_ardupilot.py --dossier "experiments/11_mission/tests of system/eval_nominal" --depuis 200   (depuis swarm_qr/)
"""
import argparse
from pathlib import Path

from pymavlink import DFReader

# numéro de mode de vol ArduCopter → nom ; ROUTINE : bouts de texte des messages de routine (démarrage,
# calibration, paramètres…) : un message qui en contient un n'est pas affiché
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND", 16: "POSHOLD", 17: "BRAKE"}
ROUTINE = ("ArduCopter", "ChibiOS", "RCOut", "Frame", "IMU", "Calibrat", "GPS", "New mission", "New rally",
           "New fence", "RC Protocol", "Param", "Fence", "PreArm", "Ground", "yaw alignment", "is using", "origin set")


def lit(fichier: Path, depuis: float) -> None:
    """Affiche les changements de mode, les erreurs et les messages non routiniers d'un journal .BIN, à partir de `depuis` (s)."""
    log = DFReader.DFReader_binary(str(fichier))
    t0, t = None, 0.0
    while True:
        m = log.recv_msg()
        if m is None:
            break
        if hasattr(m, "TimeUS"):
            t0 = m.TimeUS if t0 is None else t0
            t = (m.TimeUS - t0) / 1e6
        genre = m.get_type()
        if genre == "MODE" and t >= depuis:
            print(f"  {t:7.1f}s MODE {MODES.get(m.Mode, m.Mode)} (raison {m.Rsn})")
        elif genre == "ERR" and t >= depuis:
            print(f"  {t:7.1f}s ERR sous-systeme {m.Subsys} code {m.ECode}")
        elif genre == "MSG" and t >= depuis and not any(k in m.Message for k in ROUTINE):
            print(f"  {t:7.1f}s MSG {m.Message}")
    print(f"  (duree du journal {t:.0f} s)")


def main() -> None:
    """Parcourt les journaux de chaque drone du dossier (ardupilot_logs/drone_*/) et les affiche."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--dossier", required=True)
    ap.add_argument("--depuis", type=float, default=0.0, help="secondes depuis le debut du journal")
    a = ap.parse_args()
    for d in sorted((Path(a.dossier) / "ardupilot_logs").glob("drone_*")):
        for f in sorted(d.glob("*.BIN")):
            print(f"=== {d.name} / {f.name}")
            lit(f, a.depuis)


if __name__ == "__main__":
    main()
