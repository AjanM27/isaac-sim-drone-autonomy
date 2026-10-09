#!/usr/bin/env python3
"""Compact probabilistic occupancy with stable three-state classification."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable

Voxel = tuple[int, int, int]

UNKNOWN = 0
FREE = -1
OCCUPIED = 1


def ray_voxels(
    origin: tuple[float, float, float],
    endpoint: tuple[float, float, float],
    resolution: float,
) -> list[Voxel]:
    """Traverse grid cells from origin up to, but excluding, the hit cell.

    This is the Amanatides-Woo grid traversal. Unlike repeated ``linspace``
    sampling, it allocates once per ray, cannot skip thin cells, and has no
    NumPy calls in the hot inner loop.
    """
    current = [math.floor(value / resolution) for value in origin]
    goal = [math.floor(value / resolution) for value in endpoint]
    if current == goal:
        return []
    delta = [endpoint[i] - origin[i] for i in range(3)]
    step = [1 if value > 0 else -1 if value < 0 else 0 for value in delta]
    t_delta = [resolution / abs(value) if value else math.inf for value in delta]
    t_max = []
    for axis in range(3):
        if not step[axis]:
            t_max.append(math.inf)
            continue
        boundary = (current[axis] + (1 if step[axis] > 0 else 0)) * resolution
        t_max.append((boundary - origin[axis]) / delta[axis])

    traversed: list[Voxel] = []
    # Crossing one cell boundary reduces Manhattan distance by one. The small
    # margin protects against floating-point ties at exact voxel corners.
    limit = sum(abs(goal[i] - current[i]) for i in range(3)) + 3
    for _ in range(limit):
        if current == goal:
            break
        traversed.append(tuple(current))
        axis = min(range(3), key=t_max.__getitem__)
        current[axis] += step[axis]
        t_max[axis] += t_delta[axis]
    return traversed


@dataclass(frozen=True)
class OccupancyConfig:
    """Integer log-odds parameters suitable for compact sparse maps."""

    hit_increment: int = 9
    miss_increment: int = -4
    minimum: int = -48
    maximum: int = 36
    occupied_enter: int = 18
    occupied_exit: int = 3
    free_enter: int = -10
    free_exit: int = -2


class ProbabilisticVoxelMap:
    """Sparse evidence map with hysteresis and reversible classifications.

    ``evidence`` stores quantized log-odds. ``states`` contains only stable
    FREE/OCCUPIED classifications; absent cells are UNKNOWN. Two consistent
    hits are required to create an obstacle, one contradictory scan cannot
    remove it, and repeated clear rays can eventually correct it to free.
    """

    def __init__(self, config: OccupancyConfig | None = None) -> None:
        self.config = config or OccupancyConfig()
        self.evidence: dict[Voxel, int] = {}
        self.states: dict[Voxel, int] = {}
        self.last_state_changes: set[Voxel] = set()
        self.state_transition_count = 0
        self.direct_flip_count = 0

    def state(self, cell: Voxel) -> int:
        return self.states.get(cell, UNKNOWN)

    def _classify(self, cell: Voxel, value: int) -> int:
        old = self.state(cell)
        cfg = self.config
        if old == OCCUPIED:
            if value <= cfg.free_enter:
                return FREE
            return OCCUPIED if value > cfg.occupied_exit else UNKNOWN
        if old == FREE:
            if value >= cfg.occupied_enter:
                return OCCUPIED
            return FREE if value < cfg.free_exit else UNKNOWN
        if value >= cfg.occupied_enter:
            return OCCUPIED
        if value <= cfg.free_enter:
            return FREE
        return UNKNOWN

    def _add(self, cell: Voxel, increment: int) -> bool:
        cfg = self.config
        old_value = self.evidence.get(cell, 0)
        value = max(cfg.minimum, min(cfg.maximum, old_value + increment))
        old_state = self.state(cell)
        state = self._classify(cell, value)
        self.evidence[cell] = value
        if state == UNKNOWN:
            self.states.pop(cell, None)
        else:
            self.states[cell] = state
        if state != old_state:
            self.last_state_changes.add(cell)
            self.state_transition_count += 1
            self.direct_flip_count += {old_state, state} == {FREE, OCCUPIED}
        return value != old_value or state != old_state

    def update(
        self,
        misses: Iterable[Voxel],
        hits: Iterable[Voxel],
        forced_free: Iterable[Voxel] = (),
    ) -> set[Voxel]:
        """Integrate one scan and return cells whose evidence/state changed."""
        self.last_state_changes.clear()
        hit_cells = set(hits)
        changed: set[Voxel] = set()
        for cell in set(misses) - hit_cells:
            if self._add(cell, self.config.miss_increment):
                changed.add(cell)
        for cell in hit_cells:
            if self._add(cell, self.config.hit_increment):
                changed.add(cell)
        for cell in set(forced_free) - hit_cells:
            old_value = self.evidence.get(cell, 0)
            old_state = self.state(cell)
            value = min(old_value, self.config.free_enter)
            self.evidence[cell] = value
            self.states[cell] = FREE
            if value != old_value or old_state != FREE:
                changed.add(cell)
            if old_state != FREE:
                self.last_state_changes.add(cell)
                self.state_transition_count += 1
                self.direct_flip_count += old_state == OCCUPIED
        return changed


def is_frontier(
    cell: Voxel,
    states: dict[Voxel, int],
    within_bounds: Callable[[Voxel], bool],
    neighbors: tuple[Voxel, ...],
    minimum_unknown_neighbors: int = 2,
    minimum_free_neighbors: int = 2,
) -> bool:
    """Return whether a stable free voxel borders enough true unknown space."""
    if states.get(cell, UNKNOWN) != FREE:
        return False
    unknown = 0
    free = 0
    for delta in neighbors:
        adjacent = tuple(cell[i] + delta[i] for i in range(3))
        if not within_bounds(adjacent):
            continue
        state = states.get(adjacent, UNKNOWN)
        unknown += state == UNKNOWN
        free += state == FREE
    return unknown >= minimum_unknown_neighbors and free >= minimum_free_neighbors
