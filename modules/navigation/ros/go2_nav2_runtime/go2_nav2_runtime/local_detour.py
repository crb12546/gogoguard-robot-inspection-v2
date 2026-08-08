"""Bounded A* over an already-inflated rolling Nav2 OccupancyGrid."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple


@dataclass(frozen=True)
class GridView:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    costs: Tuple[int, ...]

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def point(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        return (
            self.origin_x + (cell[0] + 0.5) * self.resolution,
            self.origin_y + (cell[1] + 0.5) * self.resolution,
        )

    def inside(self, cell: Tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def cost(self, cell: Tuple[int, int]) -> int:
        return self.costs[cell[1] * self.width + cell[0]]

    def traversable(self, cell: Tuple[int, int], threshold: int) -> bool:
        return self.inside(cell) and 0 <= self.cost(cell) < threshold


def _nearest_free(grid: GridView, cell: Tuple[int, int], threshold: int) -> Optional[Tuple[int, int]]:
    if grid.traversable(cell, threshold):
        return cell
    for radius in range(1, 7):
        candidates = []
        for dx in range(-radius, radius + 1):
            for dy in (-radius, radius):
                candidates.append((cell[0] + dx, cell[1] + dy))
        for dy in range(-radius + 1, radius):
            for dx in (-radius, radius):
                candidates.append((cell[0] + dx, cell[1] + dy))
        usable = [value for value in candidates if grid.traversable(value, threshold)]
        if usable:
            return min(usable, key=lambda value: math.hypot(value[0] - cell[0], value[1] - cell[1]))
    return None


def plan_detour(
    grid: GridView,
    start_xy: Sequence[float],
    goal_xy: Sequence[float],
    *,
    occupied_threshold: int = 65,
    maximum_expansions: int = 12000,
) -> Optional[Tuple[Tuple[float, float], ...]]:
    """Return a smoothed grid path or None within a strict compute budget."""
    if (
        grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0.0
        or len(grid.costs) != grid.width * grid.height
    ):
        return None
    start = _nearest_free(grid, grid.cell(float(start_xy[0]), float(start_xy[1])), occupied_threshold)
    goal = _nearest_free(grid, grid.cell(float(goal_xy[0]), float(goal_xy[1])), occupied_threshold)
    if start is None or goal is None:
        return None
    frontier = [(0.0, start)]
    came_from = {start: None}
    distance = {start: 0.0}
    neighbors = (
        (-1, -1, math.sqrt(2.0)), (0, -1, 1.0), (1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0), (1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)), (0, 1, 1.0), (1, 1, math.sqrt(2.0)),
    )
    expansions = 0
    while frontier and expansions < maximum_expansions:
        _, current = heapq.heappop(frontier)
        expansions += 1
        if current == goal:
            break
        for dx, dy, step in neighbors:
            following = (current[0] + dx, current[1] + dy)
            if not grid.traversable(following, occupied_threshold):
                continue
            # The grid is already footprint-inflated. Remaining cost is a soft
            # preference for the centre of the free corridor, not a second veto.
            penalty = 1.0 + max(0, grid.cost(following)) / 100.0
            candidate = distance[current] + step * penalty
            if candidate >= distance.get(following, float("inf")):
                continue
            distance[following] = candidate
            came_from[following] = current
            heuristic = math.hypot(following[0] - goal[0], following[1] - goal[1])
            heapq.heappush(frontier, (candidate + heuristic, following))
    if goal not in came_from:
        return None
    cells = []
    current = goal
    while current is not None:
        cells.append(current)
        current = came_from[current]
    cells.reverse()
    points = [grid.point(cell) for cell in cells]
    # Collapse collinear grid steps. RPP owns this path only until the route
    # rejoin anchor; MPPI resumes the recorded route after RPP succeeds.
    simplified = [points[0]]
    previous_direction = None
    for index in range(1, len(cells)):
        direction = (cells[index][0] - cells[index - 1][0], cells[index][1] - cells[index - 1][1])
        if previous_direction is not None and direction != previous_direction:
            simplified.append(points[index - 1])
        previous_direction = direction
    simplified.append(points[-1])
    return tuple(simplified)


def route_rejoin_index(
    route_xy: Iterable[Sequence[float]], current_index: int, lookahead_m: float
) -> int:
    points = list(route_xy)
    if len(points) < 2:
        return 0
    index = max(0, min(int(current_index), len(points) - 1))
    distance = 0.0
    while index + 1 < len(points) and distance < float(lookahead_m):
        distance += math.hypot(
            float(points[index + 1][0]) - float(points[index][0]),
            float(points[index + 1][1]) - float(points[index][1]),
        )
        index += 1
    return index
