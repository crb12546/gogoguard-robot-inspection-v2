"""Pure geometry for the navigation-only obstacle cloud filter."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Iterator, Sequence


@dataclass(frozen=True)
class SelfExclusionBox:
    """The physical space occupied by the rigid Go2 body and backboard.

    This is intentionally smaller than the padded planning footprint.  Points
    inside the rigid body cannot be external obstacles; points in the 10 cm
    planning margin must remain visible to collision handling.
    """

    x_min: float = -0.35
    x_max: float = 0.42
    y_min: float = -0.22
    y_max: float = 0.22
    z_min: float = -0.10
    z_max: float = 1.05

    def validate(self) -> None:
        values = (
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.z_min,
            self.z_max,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("self exclusion bounds must be finite")
        if not (
            self.x_min < self.x_max
            and self.y_min < self.y_max
            and self.z_min < self.z_max
        ):
            raise ValueError("self exclusion bounds are invalid")

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )


def filtered_xyz_points(
    points: Iterable[Sequence[float]],
    *,
    self_box: SelfExclusionBox,
) -> Iterator[tuple[float, float, float]]:
    """Yield finite XYZ points outside the rigid self-occupied volume."""

    self_box.validate()
    for point in points:
        if len(point) < 3:
            continue
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if self_box.contains(x, y, z):
            continue
        yield x, y, z
