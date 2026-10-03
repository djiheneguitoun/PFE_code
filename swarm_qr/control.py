"""Contrôleur de vol d'un drone (étape 3) : l'amène à une pose (position + cap), l'y tient, et rend un bilan.

Phases : transit de point en point, approche à vitesse proportionnelle à la distance restante, tenue 0,5 s, puis
« atteint » ; ou « abandon » (délai dépassé, ou 8 s sans progrès). Ne bloque jamais : un appel à tick() = une commande.
Utilisé par mission.py (un contrôleur par drone) et par le banc experiments/08_controle ; tests : tests/test_control.py."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

V_TRANSIT = 1.5         # m/s, vitesse constante entre les points de passage (la mission prend 1,0)
V_APPROCHE = 1.0        # m/s, plafond de vitesse en approche (la mission prend 0,6)
GAIN = 0.9              # 1/s : vitesse = gain × distance restante ; validé à un drone, la mission prend 0,5
TOL = 0.15              # m, écart de position accepté pour dire la pose atteinte (la mission prend 0,35)
TOL_CAP = 0.09          # rad (environ 5°), écart de cap accepté
GAIN_CAP = 1.5          # 1/s : vitesse de rotation = 1,5 × écart de cap
VIT_CAP_MAX = 1.0       # rad/s, vitesse de rotation maximale
TOL_WP = 0.5            # m : un point de passage est franchi à moins de 50 cm
TENUE_S = 0.5           # s à tenir la pose dans la tolérance avant de la déclarer atteinte
PATIENCE_S = 8.0        # s sans progrès avant d'abandonner (drone qui racle un rack)
PROGRES_MIN = 0.10      # m : gain minimal sur la meilleure distance pour compter comme un progrès
MARGE_BUDGET_S = 20.0   # s ajoutées au délai par défaut d'un ordre

# Les trois façons de s'arrêter comparées à l'étape 3 : vitesse proportionnelle à la distance (retenue),
# vitesse coupée à l'arrivée (le drone glisse de 79 cm), consigne de position native d'ArduPilot (s'arrête à 25 cm).
LOIS = ("proportionnel", "coupe", "autopilote")


class Phase(Enum):
    """Où en est le drone dans l'ordre reçu."""
    REPOS = "repos"             # aucun ordre reçu
    TRANSIT = "transit"         # de point de passage en point de passage, à vitesse constante
    APPROCHE = "approche"       # vers la pose finale, vitesse proportionnelle à la distance
    TENUE = "tenue"             # dans la tolérance : compte TENUE_S avant de valider
    ATTEINT = "atteint"         # pose atteinte et tenue (fin de l'ordre)
    ABANDON = "abandon"         # délai dépassé ou plus de progrès (fin de l'ordre)


TERMINALES = (Phase.ATTEINT, Phase.ABANDON)     # les phases qui terminent un ordre


@dataclass(frozen=True)
class Consigne:
    """Une pose à atteindre : position (m, repère du monde) et cap (rad)."""
    position: np.ndarray
    cap: float


@dataclass
class Bilan:
    """Le bilan d'un ordre terminé : phase finale, raison, durées (s), longueur (m), erreurs et vitesse finales."""
    phase: Phase
    raison: str             # « delai », « bloque »… ; vide si atteint
    t_transit: float        # s
    t_approche: float       # s
    t_tenue: float          # s
    t_total: float          # s
    longueur: float         # m, longueur prévue du trajet
    err_finale: float       # m, distance à la pose à la fin
    cap_err_finale: float   # rad
    v_finale: float         # m/s
    n_points: int           # nombre de points de passage

    def as_dict(self) -> dict:
        """Renvoie le bilan sous forme de dictionnaire, la phase en texte (pour le JSON)."""
        d = dict(self.__dict__)
        d["phase"] = self.phase.value
        return d


def _ecart_cap_signe(a: float, b: float) -> float:
    """Renvoie l'écart de cap a - b ramené entre -π et π (rad)."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def _ecart_cap(a: float, b: float) -> float:
    """Renvoie l'écart entre deux caps en valeur absolue (rad, entre 0 et π)."""
    return abs(_ecart_cap_signe(a, b))


class Controleur:
    """Le contrôleur d'un drone : le mène à une pose par phases, l'y tient, et rend un bilan, sans jamais bloquer."""

    def __init__(self, pilot, get_pos, get_yaw=None, get_vel=None, *,
                 loi: str = "proportionnel", v_transit: float = V_TRANSIT,
                 v_approche: float = V_APPROCHE, gain: float = GAIN, tol: float = TOL,
                 tol_cap: float = TOL_CAP, tol_wp: float = TOL_WP, tenue_s: float = TENUE_S,
                 patience_s: float = PATIENCE_S):
        """Garde le pilote, les fonctions qui lisent position, cap et vitesse VRAIS du drone, et les réglages.
        Démarre au repos ; lève ValueError si la loi n'est pas l'une de LOIS."""
        if loi not in LOIS:
            raise ValueError(f"loi inconnue : {loi}")
        self.pilot = pilot
        self.get_pos = get_pos
        self.get_yaw = get_yaw
        self.get_vel = get_vel
        self.loi = loi
        self.v_transit = v_transit
        self.v_approche = v_approche
        self.gain = gain
        self.tol = tol
        self.tol_cap = tol_cap
        self.tol_wp = tol_wp
        self.tenue_s = tenue_s
        self.patience_s = patience_s
        self.phase = Phase.REPOS
        self.consigne: Consigne | None = None
        self.bilan: Bilan | None = None
        self._tenue: Consigne | None = None

    def assigne(self, consigne: Consigne, waypoints=(), budget_s: float | None = None) -> None:
        """Donne un nouvel ordre : pose finale et points de passage, délai par défaut = 2 × temps prévu + 20 s.
        Démarre en transit, ou directement en approche s'il n'y a aucun point de passage."""
        self.pilot.ensure_frame(self.get_pos, self.get_yaw)
        self.consigne = consigne
        self.points = [np.asarray(w, float) for w in waypoints]
        self.i_point = 0                    # indice du prochain point de passage à rejoindre
        p = np.asarray(self.get_pos(), float)
        legs = [np.linalg.norm(q - a) for a, q in zip([p] + self.points, self.points)]
        dernier = float(np.linalg.norm(consigne.position - (self.points[-1] if self.points else p)))
        self.longueur = float(sum(legs) + dernier)
        if budget_s is None:
            budget_s = 2.0 * (sum(legs) / self.v_transit + dernier / self.v_approche) + MARGE_BUDGET_S
        self.budget_s = budget_s
        self.t0 = self.pilot.sim_clock
        self.t_fin_transit = None
        self.t_debut_tenue = None
        self.t_premiere_tenue = None
        self._coupe = False
        self._tenue = None
        vers = consigne.position - (self.points[-1] if self.points else p)
        self._axe = vers / max(np.linalg.norm(vers), 1e-6)     # direction du dernier tronçon (loi « coupe »)
        self._reset_progres()
        self.bilan = None
        self.phase = Phase.TRANSIT if self.points else Phase.APPROCHE

    def tick(self) -> Phase:
        """Avance d'un pas : envoie une commande selon la phase et renvoie la phase (ordre fini : tient la pose)."""
        if self.phase in TERMINALES or self.consigne is None:
            if self._tenue is not None:
                self._tenir(self._tenue)
            return self.phase
        t = self.pilot.sim_clock
        p = np.asarray(self.get_pos(), float)
        if t - self.t0 > self.budget_s:
            return self._fin(Phase.ABANDON, "delai")

        # transit : vitesse constante vers le point de passage courant ; à 50 cm, on passe au suivant,
        # et après le dernier, à l'approche
        if self.phase is Phase.TRANSIT:
            goal = self.points[self.i_point]
            err = goal - p
            d = float(np.linalg.norm(err))
            if d < self.tol_wp:
                self.i_point += 1
                self._reset_progres()
                if self.i_point == len(self.points):
                    self.phase = Phase.APPROCHE
                    self.t_fin_transit = t
                else:
                    goal = self.points[self.i_point]
                    err = goal - p
                    d = float(np.linalg.norm(err))
            if self.phase is Phase.TRANSIT:
                if not self._progresse(d, t):
                    return self._fin(Phase.ABANDON, "bloque")
                self._envoie(err / max(d, 1e-6) * self.v_transit, self.consigne.cap)
                return self.phase

        err = self.consigne.position - p
        d = float(np.linalg.norm(err))
        cap_ok = self.get_yaw is None or _ecart_cap(self.consigne.cap, self.get_yaw()) < self.tol_cap
        # approche : le progrès n'est jugé que loin de la pose (au-delà de 2 × la tolérance) ;
        # dans la tolérance de position ET de cap, on passe en tenue
        if self.phase is Phase.APPROCHE:
            if d > 2.0 * self.tol and not self._progresse(d, t):
                return self._fin(Phase.ABANDON, "bloque")
            if d < self.tol and cap_ok:
                self.phase = Phase.TENUE
                self.t_debut_tenue = t
                if self.t_premiere_tenue is None:
                    self.t_premiere_tenue = t
        # tenue : retour en approche si le drone sort de 1,5 × la tolérance ou perd le cap ; atteint après TENUE_S
        elif self.phase is Phase.TENUE:
            if d > 1.5 * self.tol or not cap_ok:
                self.phase = Phase.APPROCHE
                self.t_debut_tenue = None
            elif t - self.t_debut_tenue >= self.tenue_s:
                return self._fin(Phase.ATTEINT, "")
        self._commande_approche(err, d)
        return self.phase

    def _envoie(self, v: np.ndarray, cap: float) -> None:
        """Envoie une vitesse (m/s, repère du monde) ; le cap est corrigé par une vitesse de rotation calculée sur
        le cap VRAI, car le cap estimé par ArduPilot se trompe jusqu'à 8,7° (étape 3)."""
        if self.get_yaw is None:
            self.pilot.velocity_world(v, cap)
            return
        rotation = GAIN_CAP * _ecart_cap_signe(cap, self.get_yaw())
        self.pilot.velocity_world(v, yaw_rate_world=float(np.clip(rotation, -VIT_CAP_MAX, VIT_CAP_MAX)))

    def _commande_approche(self, err: np.ndarray, d: float) -> None:
        """Envoie la commande d'approche selon la loi : proportionnelle (plafonnée), « coupe », ou position ArduPilot."""
        cap = self.consigne.cap
        if self.loi == "proportionnel":
            v = self.gain * err
            n = float(np.linalg.norm(v))
            if n > self.v_approche:
                v *= self.v_approche / n
            self._envoie(v, cap)
        elif self.loi == "coupe":
            # vitesse constante, puis vitesse nulle pour de bon dès la cible atteinte ou dépassée
            if self._coupe or d < self.tol or float(np.dot(err, self._axe)) <= 0.0:
                self._coupe = True
                self._envoie(np.zeros(3), cap)
            else:
                self._envoie(err / max(d, 1e-6) * self.v_approche, cap)
        else:
            self.pilot.position_world(self.consigne.position, cap)

    def _tenir(self, c: Consigne) -> None:
        """Tient une pose avec la loi proportionnelle : une simple vitesse nulle laisserait dériver le drone
        de 1 à 3 cm/s (mesuré à l'étape 3 : 44 cm en 14 s)."""
        if self.loi == "autopilote":
            self.pilot.position_world(c.position, c.cap)
            return
        v = self.gain * (c.position - np.asarray(self.get_pos(), float))
        n = float(np.linalg.norm(v))
        if n > self.v_approche:
            v *= self.v_approche / n
        self._envoie(v, c.cap)

    def _reset_progres(self) -> None:
        """Remet à zéro le suivi du progrès (meilleure distance et son heure), à chaque ordre et point de passage."""
        self._meilleure_d = math.inf
        self._t_meilleure = self.pilot.sim_clock

    def _progresse(self, d: float, t: float) -> bool:
        """Renvoie faux si la meilleure distance n'a pas gagné 10 cm depuis plus de `patience_s` (8 s) : drone bloqué."""
        if d < self._meilleure_d - PROGRES_MIN:
            self._meilleure_d = d
            self._t_meilleure = t
        return t - self._t_meilleure <= self.patience_s

    def _fin(self, phase: Phase, raison: str) -> Phase:
        """Termine l'ordre : remplit le bilan, passe en phase finale et tient la pose (la cible si elle est atteinte,
        sinon le point où le drone s'est arrêté) ; renvoie la phase."""
        t = self.pilot.sim_clock
        p = np.asarray(self.get_pos(), float)
        t_transit = (self.t_fin_transit - self.t0) if self.t_fin_transit is not None else 0.0
        if self.t_premiere_tenue is not None:
            t_approche = self.t_premiere_tenue - self.t0 - t_transit
            t_tenue = t - self.t_premiere_tenue
        else:
            t_approche = t - self.t0 - t_transit
            t_tenue = 0.0
        self.bilan = Bilan(
            phase=phase, raison=raison,
            t_transit=round(t_transit, 2), t_approche=round(t_approche, 2),
            t_tenue=round(t_tenue, 2), t_total=round(t - self.t0, 2),
            longueur=round(self.longueur, 2),
            err_finale=round(float(np.linalg.norm(self.consigne.position - p)), 3),
            cap_err_finale=round(
                _ecart_cap(self.consigne.cap, self.get_yaw()) if self.get_yaw else 0.0, 3),
            v_finale=round(float(np.linalg.norm(self.get_vel())) if self.get_vel else 0.0, 3),
            n_points=len(self.points),
        )
        self.phase = phase
        self._tenue = self.consigne if phase is Phase.ATTEINT else Consigne(p, self.consigne.cap)
        self._tenir(self._tenue)
        return phase


DT_TICK = 0.15      # s de temps simulé entre deux commandes dans `pas` et `rejoindre` (mission.py : 0,2 s)


def pas(controleurs, clock, dt: float = DT_TICK) -> list[Phase]:
    """Fait un pas pour plusieurs drones : chacun envoie sa commande, puis le monde avance une fois ; renvoie les phases."""
    phases = [c.tick() for c in controleurs]
    clock.pump(dt)
    return phases


def rejoindre(ctrl: Controleur, dt: float = DT_TICK, on_tick=None) -> Bilan:
    """Fait voler un seul drone jusqu'à ATTEINT ou ABANDON (appel bloquant) et renvoie le bilan."""
    while True:
        phase = ctrl.tick()
        if on_tick is not None:
            on_tick(ctrl)
        if phase in TERMINALES:
            return ctrl.bilan
        ctrl.pilot.pump(dt)
