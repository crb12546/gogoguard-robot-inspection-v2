"""Small dependency-free SE(3) helpers using ROS quaternion conventions."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _finite_tuple(values: Iterable[Any], size: int, label: str) -> Tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size:
        raise ValueError("%s must contain %d values" % (label, size))
    if not all(math.isfinite(value) for value in result):
        raise ValueError("%s must contain only finite values" % label)
    return result


def quaternion_normalize(values: Iterable[Any]) -> Quaternion:
    """Return a normalized ROS xyzw quaternion."""
    x, y, z, w = _finite_tuple(values, 4, "rotation_xyzw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError("rotation_xyzw must not be a zero quaternion")
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Compose xyzw quaternions, applying ``right`` before ``left``."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return quaternion_normalize(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_conjugate(value: Quaternion) -> Quaternion:
    x, y, z, w = value
    return (-x, -y, -z, w)


def quaternion_rotate(value: Quaternion, vector: Vector3) -> Vector3:
    """Rotate a vector without constructing a temporary point quaternion."""
    x, y, z, w = value
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Create an xyzw quaternion from radians in fixed-axis ROS RPY order."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return quaternion_normalize(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


@dataclass(frozen=True)
class RigidTransform:
    """A transform ``T_parent_child`` that maps child points into parent."""

    parent_frame: str
    child_frame: str
    translation_xyz_m: Vector3
    rotation_xyzw: Quaternion

    def __post_init__(self) -> None:
        if not self.parent_frame or not self.child_frame:
            raise ValueError("transform frames must be non-empty")
        if self.parent_frame == self.child_frame:
            raise ValueError("transform parent and child must differ")
        translation = _finite_tuple(self.translation_xyz_m, 3, "translation_xyz_m")
        rotation = quaternion_normalize(self.rotation_xyzw)
        object.__setattr__(self, "translation_xyz_m", translation)
        object.__setattr__(self, "rotation_xyzw", rotation)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RigidTransform":
        return cls(
            parent_frame=str(data.get("parent_frame", "")),
            child_frame=str(data.get("child_frame", "")),
            translation_xyz_m=data.get("translation_xyz_m", ()),
            rotation_xyzw=data.get("rotation_xyzw", ()),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "translation_xyz_m": list(self.translation_xyz_m),
            "rotation_xyzw": list(self.rotation_xyzw),
        }

    def rotate_vector(self, vector: Iterable[Any]) -> Vector3:
        parsed = _finite_tuple(vector, 3, "vector")
        return quaternion_rotate(self.rotation_xyzw, parsed)

    def apply_point(self, point: Iterable[Any]) -> Vector3:
        rotated = self.rotate_vector(point)
        return tuple(
            rotated[index] + self.translation_xyz_m[index]
            for index in range(3)
        )

    def inverse(self) -> "RigidTransform":
        inverse_rotation = quaternion_conjugate(self.rotation_xyzw)
        inverse_translation = quaternion_rotate(
            inverse_rotation,
            tuple(-value for value in self.translation_xyz_m),
        )
        return RigidTransform(
            parent_frame=self.child_frame,
            child_frame=self.parent_frame,
            translation_xyz_m=inverse_translation,
            rotation_xyzw=inverse_rotation,
        )

    def compose(self, child_transform: "RigidTransform") -> "RigidTransform":
        """Return ``T_parent_grandchild`` from this and ``T_child_grandchild``."""
        if self.child_frame != child_transform.parent_frame:
            raise ValueError(
                "cannot compose %s->%s with %s->%s"
                % (
                    self.parent_frame,
                    self.child_frame,
                    child_transform.parent_frame,
                    child_transform.child_frame,
                )
            )
        rotated_translation = self.rotate_vector(child_transform.translation_xyz_m)
        translation = tuple(
            self.translation_xyz_m[index] + rotated_translation[index]
            for index in range(3)
        )
        return RigidTransform(
            parent_frame=self.parent_frame,
            child_frame=child_transform.child_frame,
            translation_xyz_m=translation,
            rotation_xyzw=quaternion_multiply(
                self.rotation_xyzw,
                child_transform.rotation_xyzw,
            ),
        )
