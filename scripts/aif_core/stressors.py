"""Pannes programmées (« stresseurs ») pour tester la résilience de l'essaim.

Perte d'un drone, coupure du cloud, coupure de liens drone-drone, obstacle soudain : chacune se déclenche
une seule fois, au pas donné par les options --kill-drone-at-step, --cut-cloud-at-step, etc. (-1 = jamais),
et signale un stress au gestionnaire de résilience.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class StressContext:
    """Ce dont une panne a besoin pour agir : drones, état des liens, résilience, réglages, fonction d'ajout d'obstacle."""

    agents: list
    link_state: object
    resilience: object
    cfg: object
    inject_obstacle_fn: Optional[callable] = None


class Stressor:
    """Panne générique, déclenchée une seule fois au pas trigger_step (-1 = désactivée)."""

    name: str = "generic"

    def __init__(self, trigger_step: int):
        """Mémorise le pas de déclenchement ; la panne n'a pas encore eu lieu."""
        self.trigger_step = trigger_step
        self.fired: bool = False

    def ready(self, step: int) -> bool:
        """Vérifie que la panne doit se produire maintenant (activée, bon pas, pas encore faite)."""
        return (self.trigger_step >= 0
                and step == self.trigger_step
                and not self.fired)

    def apply(self, step: int, ctx: StressContext) -> None:
        """Produit la panne (à définir dans chaque type de panne)."""
        raise NotImplementedError

    def fire(self, step: int, ctx: StressContext) -> None:
        """Produit la panne si c'est le bon pas, puis la marque comme faite."""
        if self.ready(step):
            self.apply(step, ctx)
            self.fired = True


class KillDroneStressor(Stressor):
    """Panne « perte d'un drone » : le drone choisi atterrit et ne participe plus."""

    name = "kill_drone"

    def __init__(self, trigger_step: int, drone_id: int):
        """Mémorise le pas de la panne et le numéro du drone à perdre."""
        super().__init__(trigger_step)
        self.drone_id = drone_id

    def apply(self, step: int, ctx: StressContext) -> None:
        """Fait atterrir le drone visé et déclenche le stress "drone_<id>_lost" (rien s'il est déjà inactif)."""
        target = None
        for a in ctx.agents:
            if a.id == self.drone_id and a.active:
                target = a
                break
        if target is None:
            print(f"  [STRESS] kill_drone : drone {self.drone_id} déjà inactif ou inconnu")
            return
        target.land()
        ctx.resilience.trigger(step, f"drone_{self.drone_id}_lost")
        print(f"\n  [STRESS] ⚠ Drone {self.drone_id} KILLED at step {step}")


class CutCloudStressor(Stressor):
    """Panne « coupure du cloud » : le serveur central ne peut plus envoyer d'actions."""

    name = "cut_cloud"

    def apply(self, step: int, ctx: StressContext) -> None:
        """Coupe le lien cloud et déclenche le stress "cloud_link_lost" (en centralisé, les drones passent en décision locale)."""
        if not ctx.link_state.cloud_link_active:
            return
        ctx.link_state.cloud_link_active = False
        ctx.resilience.trigger(step, "cloud_link_lost")
        print(f"\n  [STRESS] ☁ Cloud link CUT at step {step}")
        if ctx.cfg.arch == "centralized":
            print(f"  [STRESS] → auto-failover centralisé vers décision locale")


class CutDroneLinkStressor(Stressor):
    """Panne « coupure de liens drone-drone » : une paire « i-j » ou tous les liens (« all »)."""

    name = "cut_drone_link"

    def __init__(self, trigger_step: int, spec: str):
        """Mémorise le pas de la panne et les liens à couper ("i-j" ou "all")."""
        super().__init__(trigger_step)
        self.spec = spec.strip()

    def apply(self, step: int, ctx: StressContext) -> None:
        """Coupe la paire ou tous les liens et déclenche le stress correspondant ; un texte invalide est signalé puis ignoré."""
        if self.spec == "all":
            if not ctx.link_state.all_drone_links_cut:
                ctx.link_state.all_drone_links_cut = True
                ctx.resilience.trigger(step, "all_drone_links_lost")
                print(f"\n  [STRESS] ✂ ALL drone↔drone links CUT at step {step}")
        elif "-" in self.spec:
            try:
                a, b = self.spec.split("-")
                i, j = int(a), int(b)
                pair = frozenset({i, j})
                if pair not in ctx.link_state.cut_pairs:
                    ctx.link_state.cut_pairs.add(pair)
                    ctx.resilience.trigger(step, f"drone_link_{i}-{j}_lost")
                    print(f"\n  [STRESS] ✂ Drone link {i}↔{j} CUT at step {step}")
            except ValueError:
                print(f"  [WARN] Invalid cut_drone_link spec: {self.spec!r}")


class DynamicObstacleStressor(Stressor):
    """Panne « obstacle soudain » : un cube de 3 × 3 × 4 m apparaît en (x, y), repère monde."""

    name = "dynamic_obstacle"

    def __init__(self, trigger_step: int, xy: str):
        """Mémorise le pas et lit la position "x,y" (m, repère monde) ; texte invalide → (0, 0) avec un avertissement."""
        super().__init__(trigger_step)
        self.xy = xy.strip()
        self.x: float = 0.0
        self.y: float = 0.0
        if self.xy:
            try:
                x, y = self.xy.split(",")
                self.x, self.y = float(x), float(y)
            except ValueError:
                print(f"  [WARN] Invalid drop_obstacle_xy spec: {self.xy!r}")

    def apply(self, step: int, ctx: StressContext) -> None:
        """Fait apparaître l'obstacle dans Isaac Sim (si la fonction est fournie) et déclenche le stress "dynamic_obstacle"."""
        print(f"\n  [STRESS] 🧱 Dynamic obstacle DROPPED at step {step} "
              f"({self.x:.2f}, {self.y:.2f})")
        if ctx.inject_obstacle_fn is not None:
            try:
                ctx.inject_obstacle_fn(self.x, self.y)
            except Exception as e:
                print(f"  [WARN] obstacle injection failed: {e}")
        else:
            print(f"  [STRESS]   (pas de callback Isaac Sim → no-op)")

        # Stress déclenché explicitement, pour être sûr de passer en phase "recovery"
        ctx.resilience.trigger(step, "dynamic_obstacle")


class StressorScheduler:
    """Liste des pannes programmées, construite à partir des réglages."""

    def __init__(self, cfg):
        """Crée la liste des pannes prévues par les réglages."""
        self.cfg = cfg
        self.stressors: List[Stressor] = []
        self._build_from_config()

    def _build_from_config(self):
        """Ajoute une panne pour chaque option activée (pas ≥ 0) dans les réglages."""
        cfg = self.cfg
        if cfg.kill_drone_at_step >= 0:
            self.stressors.append(KillDroneStressor(
                cfg.kill_drone_at_step, cfg.kill_drone_id,
            ))
        if cfg.cut_cloud_at_step >= 0:
            self.stressors.append(CutCloudStressor(cfg.cut_cloud_at_step))
        if cfg.cut_drone_link and cfg.cut_drone_link_at_step >= 0:
            self.stressors.append(CutDroneLinkStressor(
                cfg.cut_drone_link_at_step, cfg.cut_drone_link,
            ))
        if cfg.drop_obstacle_at_step >= 0:
            self.stressors.append(DynamicObstacleStressor(
                cfg.drop_obstacle_at_step, cfg.drop_obstacle_xy,
            ))

    def tick(self, step: int, ctx: StressContext) -> None:
        """Produit les pannes prévues à ce pas."""
        for s in self.stressors:
            s.fire(step, ctx)

    def summary(self) -> List[str]:
        """Renvoie une description texte de chaque panne programmée (affichée au démarrage)."""
        out = []
        for s in self.stressors:
            tag = f"{s.name} @ step {s.trigger_step}"
            if isinstance(s, KillDroneStressor):
                tag += f" (drone_id={s.drone_id})"
            elif isinstance(s, CutDroneLinkStressor):
                tag += f" (spec={s.spec!r})"
            elif isinstance(s, DynamicObstacleStressor):
                tag += f" (xy=({s.x:.1f},{s.y:.1f}))"
            out.append(tag)
        return out
