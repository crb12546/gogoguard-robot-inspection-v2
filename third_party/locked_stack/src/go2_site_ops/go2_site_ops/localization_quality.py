"""Sealed localization quality profiles shared by release and runtime code."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class LocalizationQualityProfile:
    """A reviewed set of ROS parameters selected by an immutable map package."""

    profile_id: str
    registration_type: str
    local_map_radius_m: float
    local_map_refresh_distance_m: float
    max_scan_range_m: float
    tracking_scan_leaf_m: float
    coarse_scan_leaf_m: float
    tracking_map_leaf_m: float
    coarse_map_leaf_m: float
    registration_threads: int
    coarse_correspondence_m: float
    screening_iterations: int
    screening_min_inlier_ratio: float
    coarse_iterations: int
    tracking_correspondence_m: float
    tracking_iterations: int
    minimum_input_points: int
    minimum_inlier_ratio: float
    maximum_fitness_mse: float
    minimum_hessian_eigenvalue: float
    maximum_hessian_condition: float
    maximum_correction_translation_m: float
    maximum_correction_rotation_deg: float
    maximum_input_age_s: float

    def ros_parameters(self) -> Dict[str, Any]:
        """Return the complete parameter override consumed by the C++ node."""

        return {
            "quality.profile_id": self.profile_id,
            "registration.type": self.registration_type,
            "local_map.radius_m": self.local_map_radius_m,
            "local_map.refresh_distance_m": self.local_map_refresh_distance_m,
            "max_scan_range": self.max_scan_range_m,
            "tracking_scan_leaf": self.tracking_scan_leaf_m,
            "coarse_scan_leaf": self.coarse_scan_leaf_m,
            "tracking_map_leaf": self.tracking_map_leaf_m,
            "coarse_map_leaf": self.coarse_map_leaf_m,
            "registration_threads": self.registration_threads,
            "initialization.max_correspondence_m": self.coarse_correspondence_m,
            "initialization.screening_iterations": self.screening_iterations,
            "initialization.screening_min_inlier_ratio": (
                self.screening_min_inlier_ratio
            ),
            "initialization.max_iterations": self.coarse_iterations,
            "tracking.max_correspondence_m": self.tracking_correspondence_m,
            "tracking.max_iterations": self.tracking_iterations,
            "quality.min_input_points": self.minimum_input_points,
            "quality.min_inlier_ratio": self.minimum_inlier_ratio,
            "quality.max_fitness_mse": self.maximum_fitness_mse,
            "quality.min_hessian_eigenvalue": self.minimum_hessian_eigenvalue,
            "quality.max_hessian_condition": self.maximum_hessian_condition,
            "quality.max_correction_translation_m": (
                self.maximum_correction_translation_m
            ),
            "quality.max_correction_rotation_deg": (
                self.maximum_correction_rotation_deg
            ),
            "quality.max_input_age_s": self.maximum_input_age_s,
        }


# Existing release profiles already reference this identifier. Its parameter
# set is now explicit and immutable instead of silently inherited from YAML.
GO2_VGICP_ORIN_V1 = LocalizationQualityProfile(
    profile_id="go2-vgicp-orin-v1",
    registration_type="VGICP",
    local_map_radius_m=45.0,
    local_map_refresh_distance_m=5.0,
    max_scan_range_m=35.0,
    tracking_scan_leaf_m=0.25,
    coarse_scan_leaf_m=0.65,
    tracking_map_leaf_m=0.30,
    coarse_map_leaf_m=0.80,
    registration_threads=4,
    coarse_correspondence_m=3.0,
    screening_iterations=5,
    screening_min_inlier_ratio=0.05,
    coarse_iterations=15,
    tracking_correspondence_m=1.2,
    tracking_iterations=30,
    minimum_input_points=250,
    minimum_inlier_ratio=0.20,
    maximum_fitness_mse=0.20,
    minimum_hessian_eigenvalue=1.0e-6,
    maximum_hessian_condition=1.0e8,
    maximum_correction_translation_m=0.75,
    maximum_correction_rotation_deg=15.0,
    maximum_input_age_s=0.35,
)

SUPPORTED_LOCALIZATION_QUALITY_PROFILES = {
    GO2_VGICP_ORIN_V1.profile_id: GO2_VGICP_ORIN_V1,
}


def resolve_localization_quality_profile(profile_id: str) -> LocalizationQualityProfile:
    """Resolve an exact reviewed profile or fail closed."""

    requested = str(profile_id).strip()
    try:
        return SUPPORTED_LOCALIZATION_QUALITY_PROFILES[requested]
    except KeyError as exc:
        raise ValueError("unsupported localization quality profile: %s" % requested) from exc
