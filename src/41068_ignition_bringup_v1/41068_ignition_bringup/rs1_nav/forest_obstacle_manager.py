"""Dynamic forest-gap obstacle manager for the Status UI demo.

Places a real Gazebo box that blocks exactly one of the two known forest-wall
gaps at x≈8 in custom_world_1. Activation is spatial: the obstacle is only
spawned once the Husky is within ``trigger_distance_m`` of the selected gap
centre. Nav2 is never told which gap to use — lidar → costmap → replan does
the work.

State machine:
    IDLE → REQUESTED (gap selected) → WAITING (until ≤5 m) → ACTIVE → IDLE (clear)
"""

from __future__ import annotations

import math
import os
import random
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

import yaml

from .gazebo_world import GazeboWorld, ObstacleSpec

LogFn = Callable[[str], None]


class ObstacleState(str, Enum):
    IDLE = 'idle'
    REQUESTED = 'requested'
    WAITING = 'waiting'
    ACTIVE = 'active'
    CLEARED = 'cleared'


@dataclass(frozen=True)
class GapSpec:
    key: str
    label: str
    x: float
    y: float
    yaw: float = 0.0

    def distance_to(self, robot_xy: Tuple[float, float]) -> float:
        return math.hypot(robot_xy[0] - self.x, robot_xy[1] - self.y)


@dataclass
class ForestGapConfig:
    world_name: str = 'custom_world_1'
    trigger_distance_m: float = 5.0
    size_x: float = 0.5
    size_y: float = 6.2
    size_z: float = 1.5
    name_prefix: str = 'dynamic_obstacle_'
    gaps: Dict[str, GapSpec] = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path: str) -> 'ForestGapConfig':
        with open(path, 'r', encoding='utf-8') as handle:
            raw = yaml.safe_load(handle) or {}
        block = raw.get('forest_gaps', raw)
        obstacle = block.get('obstacle', {})
        gaps: Dict[str, GapSpec] = {}
        for key, data in (block.get('gaps') or {}).items():
            gaps[key] = GapSpec(
                key=key,
                label=str(data.get('label', key)),
                x=float(data['x']),
                y=float(data['y']),
                yaw=float(data.get('yaw', 0.0)),
            )
        if not gaps:
            gaps = default_custom_world_gaps()
        return cls(
            world_name=str(block.get('world_name', 'custom_world_1')),
            trigger_distance_m=float(block.get('trigger_distance_m', 5.0)),
            size_x=float(obstacle.get('size_x', 0.5)),
            size_y=float(obstacle.get('size_y', 6.2)),
            size_z=float(obstacle.get('size_z', 1.5)),
            name_prefix=str(obstacle.get('name_prefix', 'dynamic_obstacle_')),
            gaps=gaps,
        )


def default_custom_world_gaps() -> Dict[str, GapSpec]:
    """Hard-coded fallback matching custom_world_1 forest_wall geometry."""
    return {
        'gap_a': GapSpec('gap_a', 'Gap A (north)', 8.0, 5.5, 0.0),
        'gap_b': GapSpec('gap_b', 'Gap B (south)', 8.0, -9.5, 0.0),
    }


def default_config_path() -> Optional[str]:
    """Locate packaged forest_gaps YAML via ament, else source-tree relative."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('41068_ignition_bringup')
        candidate = os.path.join(share, 'config', 'forest_gaps_custom_world_1.yaml')
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidate = os.path.join(here, 'config', 'forest_gaps_custom_world_1.yaml')
    return candidate if os.path.isfile(candidate) else None


def load_forest_gap_config(path: Optional[str] = None) -> ForestGapConfig:
    resolved = path or default_config_path()
    if resolved and os.path.isfile(resolved):
        return ForestGapConfig.from_yaml(resolved)
    cfg = ForestGapConfig(gaps=default_custom_world_gaps())
    return cfg


class ForestObstacleManager:
    """Select a forest gap, wait for proximity, spawn/remove Gazebo obstacles."""

    def __init__(
        self,
        config: Optional[ForestGapConfig] = None,
        *,
        world: Optional[GazeboWorld] = None,
        rng: Optional[random.Random] = None,
        seed: Optional[int] = None,
        force_gap: Optional[str] = None,
        logger: Optional[LogFn] = None,
    ):
        self.config = config or load_forest_gap_config()
        self.world = world or GazeboWorld(self.config.world_name, logger=logger)
        self._log = logger or (lambda message: print(message, flush=True))
        if rng is not None:
            self._rng = rng
        elif seed is not None:
            self._rng = random.Random(seed)
        else:
            self._rng = random.Random()
        self.force_gap = force_gap

        self.state = ObstacleState.IDLE
        self.selected_gap: Optional[GapSpec] = None
        self.active_names: List[str] = []
        self._next_id = 1
        self.status_message = 'Obstacle: READY'
        self.last_distance_m: Optional[float] = None

    # -- public API -------------------------------------------------------

    def request_obstacle(self) -> Tuple[bool, str]:
        """Arm a pending obstacle (random gap). Rejects if already pending/active."""
        if self.state in (ObstacleState.REQUESTED, ObstacleState.WAITING, ObstacleState.ACTIVE):
            msg = (
                f'Obstacle already {self.state.value}'
                + (f' ({self.selected_gap.label})' if self.selected_gap else '')
            )
            self._log(msg)
            return False, msg

        gap = self._choose_gap()
        if gap is None:
            msg = 'No forest gaps configured'
            self.status_message = 'Obstacle: ERROR'
            return False, msg

        self.selected_gap = gap
        self.state = ObstacleState.WAITING
        self.last_distance_m = None
        self.status_message = f'Obstacle: SELECTED {gap.label} — waiting ≤{self.config.trigger_distance_m:.0f} m'
        self._log(
            f'Obstacle requested: selected {gap.key} ({gap.label}) at '
            f'({gap.x:.2f}, {gap.y:.2f}); waiting for robot within '
            f'{self.config.trigger_distance_m:.1f} m'
        )
        return True, self.status_message

    def clear_obstacles(self) -> Tuple[bool, str]:
        """Remove all demo obstacles and reset to idle."""
        removed = 0
        for name in list(self.active_names):
            if self.world.remove_model(name):
                removed += 1
            else:
                # Best-effort: still drop from tracking so UI does not stick.
                self._log(f'Failed to remove "{name}" (may already be gone)')
            if name in self.active_names:
                self.active_names.remove(name)

        self.selected_gap = None
        self.last_distance_m = None
        self.state = ObstacleState.IDLE
        self.status_message = 'Obstacle: CLEARED' if removed or True else 'Obstacle: READY'
        # Always present CLEARED then READY semantics for the UI.
        self.status_message = 'Obstacle: CLEARED'
        self._log(f'Cleared dynamic obstacles ({removed} remove calls)')
        return True, self.status_message

    def tick(self, robot_xy: Optional[Tuple[float, float]]) -> Optional[str]:
        """Advance WAITING → ACTIVE when the robot nears the forest-gap region.

        Proximity uses the nearest configured gap centre (the wall-gap *region*),
        not only the selected gap. Otherwise selecting the southern gap would
        never fire when the robot's preferred route is the northern opening.
        The obstacle is still spawned at the randomly selected gap.
        """
        if self.state != ObstacleState.WAITING or self.selected_gap is None:
            return None
        if robot_xy is None:
            return None

        distance = min(g.distance_to(robot_xy) for g in self.config.gaps.values())
        self.last_distance_m = distance
        if distance > self.config.trigger_distance_m:
            self.status_message = (
                f'Obstacle: WAITING {self.selected_gap.label} '
                f'(region {distance:.1f} m > {self.config.trigger_distance_m:.0f} m)'
            )
            return self.status_message

        return self._spawn_selected()

    def reset_for_sim_restart(self) -> None:
        """Drop in-memory tracking after a simulation restart (models are gone)."""
        self.active_names.clear()
        self.selected_gap = None
        self.last_distance_m = None
        self.state = ObstacleState.IDLE
        self.status_message = 'Obstacle: READY'
        self._next_id = 1

    # -- internals --------------------------------------------------------

    def _choose_gap(self) -> Optional[GapSpec]:
        gaps = list(self.config.gaps.values())
        if not gaps:
            return None
        if self.force_gap:
            chosen = self.config.gaps.get(self.force_gap)
            if chosen is None:
                self._log(f'Unknown force_gap "{self.force_gap}"; falling back to random')
            else:
                return chosen
        return self._rng.choice(gaps)

    def _spawn_selected(self) -> str:
        gap = self.selected_gap
        assert gap is not None
        name = f'{self.config.name_prefix}{self._next_id:03d}'
        self._next_id += 1
        spec = ObstacleSpec(
            name=name,
            x=gap.x,
            y=gap.y,
            size_x=self.config.size_x,
            size_y=self.config.size_y,
            size_z=self.config.size_z,
            yaw=gap.yaw,
        )
        if not self.world.is_available():
            self.status_message = 'Obstacle: Gazebo unavailable'
            self._log(self.status_message)
            return self.status_message

        # Replace any leftover model with the same name.
        self.world.remove_model(name)
        if not self.world.spawn_obstacle(spec):
            self.status_message = 'Obstacle: SPAWN FAILED'
            self._log(f'Failed to spawn {name} at gap {gap.key}')
            return self.status_message

        self.active_names.append(name)
        self.state = ObstacleState.ACTIVE
        self.status_message = f'Obstacle: ACTIVE at {gap.label} ({name})'
        self._log(
            f'Spawned {name} blocking {gap.key} at ({gap.x:.2f}, {gap.y:.2f}) '
            f'when robot was {self.last_distance_m:.2f} m away'
        )
        return self.status_message
