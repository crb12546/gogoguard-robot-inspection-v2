"""Explain the bounded local A* result without changing its planning policy."""

from __future__ import annotations

import heapq
import math
from typing import Any, Optional, Sequence


def _nearest_free(grid, cell, threshold: int):
    if grid.traversable(cell, threshold):
        return cell, 0
    for radius in range(1, 7):
        candidates = []
        for dx in range(-radius, radius + 1):
            candidates.extend(((cell[0] + dx, cell[1] - radius), (cell[0] + dx, cell[1] + radius)))
        for dy in range(-radius + 1, radius):
            candidates.extend(((cell[0] - radius, cell[1] + dy), (cell[0] + radius, cell[1] + dy)))
        usable = [value for value in candidates if grid.traversable(value, threshold)]
        if usable:
            return min(usable, key=lambda value: math.hypot(value[0] - cell[0], value[1] - cell[1])), radius
    return None, None


def plan_detour_diagnostic(
    grid,
    start_xy: Sequence[float],
    goal_xy: Sequence[float],
    *,
    occupied_threshold: int = 65,
    maximum_expansions: int = 12000,
) -> tuple[Optional[tuple[tuple[float, float], ...]], dict[str, Any]]:
    """Run the same A* policy and return a bounded, renderable explanation."""
    diagnostic: dict[str, Any] = {
        "schema": "gogoguard.detour_diagnostic.v1", "outcome": "unknown",
        "occupiedThreshold": occupied_threshold, "maximumExpansions": maximum_expansions,
        "grid": {"width": int(getattr(grid, "width", 0)), "height": int(getattr(grid, "height", 0)),
                 "resolution": float(getattr(grid, "resolution", 0.0)),
                 "origin": [float(getattr(grid, "origin_x", 0.0)), float(getattr(grid, "origin_y", 0.0))]},
        "startPoint": [float(start_xy[0]), float(start_xy[1])],
        "goalPoint": [float(goal_xy[0]), float(goal_xy[1])],
        "expandedCells": [], "pathCells": [], "pathPoints": [],
    }
    if grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0.0 or len(grid.costs) != grid.width * grid.height:
        diagnostic["outcome"] = "invalid_grid"
        return None, diagnostic
    raw_start, raw_goal = grid.cell(*start_xy), grid.cell(*goal_xy)
    start, start_radius = _nearest_free(grid, raw_start, occupied_threshold)
    goal, goal_radius = _nearest_free(grid, raw_goal, occupied_threshold)
    diagnostic.update({
        "startCell": list(raw_start), "goalCell": list(raw_goal),
        "resolvedStartCell": list(start) if start else None,
        "resolvedGoalCell": list(goal) if goal else None,
        "startSearchRadiusCells": start_radius, "goalSearchRadiusCells": goal_radius,
    })
    if start is None or goal is None:
        diagnostic["outcome"] = "start_surrounded" if start is None else "goal_surrounded"
        return None, diagnostic
    frontier, came_from, distance = [(0.0, start)], {start: None}, {start: 0.0}
    neighbors = ((-1, -1, math.sqrt(2)), (0, -1, 1), (1, -1, math.sqrt(2)),
                 (-1, 0, 1), (1, 0, 1), (-1, 1, math.sqrt(2)), (0, 1, 1), (1, 1, math.sqrt(2)))
    expanded = []
    expansions = 0
    while frontier and expansions < maximum_expansions:
        _, current = heapq.heappop(frontier)
        expansions += 1
        if len(expanded) < 4000:
            expanded.append(list(current))
        if current == goal:
            break
        for dx, dy, step in neighbors:
            following = current[0] + dx, current[1] + dy
            if not grid.traversable(following, occupied_threshold):
                continue
            candidate = distance[current] + step * (1.0 + max(0, grid.cost(following)) / 100.0)
            if candidate >= distance.get(following, float("inf")):
                continue
            distance[following], came_from[following] = candidate, current
            heuristic = math.hypot(following[0] - goal[0], following[1] - goal[1])
            heapq.heappush(frontier, (candidate + heuristic, following))
    diagnostic.update({"expansionCount": expansions, "expandedCells": expanded, "frontierRemaining": len(frontier)})
    if goal not in came_from:
        diagnostic["outcome"] = "expansion_limit" if expansions >= maximum_expansions else "disconnected"
        return None, diagnostic
    cells, current = [], goal
    while current is not None:
        cells.append(current)
        current = came_from[current]
    cells.reverse()
    points = [grid.point(cell) for cell in cells]
    simplified, previous_direction = [points[0]], None
    for index in range(1, len(cells)):
        direction = cells[index][0] - cells[index - 1][0], cells[index][1] - cells[index - 1][1]
        if previous_direction is not None and direction != previous_direction:
            simplified.append(points[index - 1])
        previous_direction = direction
    simplified.append(points[-1])
    diagnostic.update({"outcome": "path_found", "pathCells": [list(value) for value in cells],
                       "pathPoints": [[round(x, 3), round(y, 3)] for x, y in simplified],
                       "pathCost": round(distance[goal], 3)})
    return tuple(simplified), diagnostic
