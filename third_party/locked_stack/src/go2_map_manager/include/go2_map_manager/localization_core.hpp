// Copyright 2026 GoGoGuard

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

namespace go2_map_manager
{

enum class LocalizationState
{
  kUninitialized,
  kSearching,
  kTracking,
  kDegraded,
  kLost,
};

inline const char * to_string(LocalizationState state)
{
  switch (state) {
    case LocalizationState::kUninitialized:
      return "UNINITIALIZED";
    case LocalizationState::kSearching:
      return "SEARCHING";
    case LocalizationState::kTracking:
      return "TRACKING";
    case LocalizationState::kDegraded:
      return "DEGRADED";
    case LocalizationState::kLost:
      return "LOST";
  }
  return "LOST";
}

struct RegistrationMetrics
{
  bool converged = false;
  std::size_t input_points = 0;
  double inlier_ratio = 0.0;
  double fitness_mse = 0.0;
  double hessian_min_eigenvalue = 0.0;
  double hessian_condition = 0.0;
  double correction_translation_m = 0.0;
  double correction_rotation_deg = 0.0;
  double input_age_s = 0.0;
};

struct RegistrationThresholds
{
  std::size_t min_input_points = 250;
  double min_inlier_ratio = 0.20;
  double max_fitness_mse = 0.20;
  double min_hessian_eigenvalue = 1.0e-6;
  double max_hessian_condition = 1.0e8;
  double max_correction_translation_m = 0.75;
  double max_correction_rotation_deg = 15.0;
  double max_input_age_s = 0.35;
};

struct RegistrationAssessment
{
  bool accepted = false;
  double confidence = 0.0;
  std::string reason = "not_evaluated";
};

struct InitialEvidencePolicy
{
  std::size_t min_samples = 8;
  std::size_t min_points = 500;
  double min_duration_s = 1.5;
};

struct InitialEvidenceAssessment
{
  bool ready = false;
  std::string reason = "waiting_initial_scans";
};

struct InitialPoseSeed
{
  double map_x = 0.0;
  double map_y = 0.0;
  double map_yaw_rad = 0.0;
};

inline InitialPoseSeed initial_pose_seed(
  double center_map_x,
  double center_map_y,
  double center_map_yaw_rad,
  double offset_map_x,
  double offset_map_y,
  double offset_map_yaw_rad)
{
  // The sealed initialization zone describes base_link in map coordinates.
  // It is deliberately independent of the current odom origin: the robot may
  // have been carried or walked after boot, and that history must not shift
  // the physical search area.
  return {
    center_map_x + offset_map_x,
    center_map_y + offset_map_y,
    std::atan2(
      std::sin(center_map_yaw_rad + offset_map_yaw_rad),
      std::cos(center_map_yaw_rad + offset_map_yaw_rad)),
  };
}

inline InitialEvidenceAssessment assess_initial_evidence(
  std::size_t sample_count,
  std::size_t point_count,
  double duration_s,
  const InitialEvidencePolicy & policy)
{
  InitialEvidenceAssessment assessment;
  if (!std::isfinite(duration_s) || duration_s < 0.0) {
    assessment.reason = "invalid_initial_scan_timing";
    return assessment;
  }
  if (sample_count < policy.min_samples) {
    assessment.reason = "waiting_initial_scans";
    return assessment;
  }
  if (duration_s < policy.min_duration_s) {
    assessment.reason = "waiting_initial_scan_duration";
    return assessment;
  }
  if (point_count < policy.min_points) {
    assessment.reason = "too_few_initial_points";
    return assessment;
  }
  assessment.ready = true;
  assessment.reason = "initial_evidence_ready";
  return assessment;
}

inline bool finite_metrics(const RegistrationMetrics & metrics)
{
  return std::isfinite(metrics.inlier_ratio) &&
         std::isfinite(metrics.fitness_mse) &&
         std::isfinite(metrics.hessian_min_eigenvalue) &&
         std::isfinite(metrics.hessian_condition) &&
         std::isfinite(metrics.correction_translation_m) &&
         std::isfinite(metrics.correction_rotation_deg) &&
         std::isfinite(metrics.input_age_s);
}

inline RegistrationAssessment assess_registration(
  const RegistrationMetrics & metrics,
  const RegistrationThresholds & thresholds,
  bool enforce_correction_limit)
{
  RegistrationAssessment assessment;
  if (!finite_metrics(metrics)) {
    assessment.reason = "non_finite_metrics";
    return assessment;
  }
  if (!metrics.converged) {
    assessment.reason = "not_converged";
    return assessment;
  }
  if (metrics.input_points < thresholds.min_input_points) {
    assessment.reason = "too_few_input_points";
    return assessment;
  }
  if (metrics.input_age_s > thresholds.max_input_age_s) {
    assessment.reason = "stale_input";
    return assessment;
  }
  if (metrics.inlier_ratio < thresholds.min_inlier_ratio) {
    assessment.reason = "low_inlier_ratio";
    return assessment;
  }
  if (metrics.fitness_mse > thresholds.max_fitness_mse) {
    assessment.reason = "high_fitness_error";
    return assessment;
  }
  if (metrics.hessian_min_eigenvalue < thresholds.min_hessian_eigenvalue) {
    assessment.reason = "geometry_unobservable";
    return assessment;
  }
  if (metrics.hessian_condition > thresholds.max_hessian_condition) {
    assessment.reason = "geometry_ill_conditioned";
    return assessment;
  }
  if (enforce_correction_limit &&
    metrics.correction_translation_m > thresholds.max_correction_translation_m)
  {
    assessment.reason = "translation_jump";
    return assessment;
  }
  if (enforce_correction_limit &&
    metrics.correction_rotation_deg > thresholds.max_correction_rotation_deg)
  {
    assessment.reason = "rotation_jump";
    return assessment;
  }

  const double inlier_score = std::clamp(
    metrics.inlier_ratio / std::max(thresholds.min_inlier_ratio * 2.0, 1.0e-9),
    0.0, 1.0);
  const double fitness_score = std::clamp(
    1.0 - metrics.fitness_mse / std::max(thresholds.max_fitness_mse, 1.0e-9),
    0.0, 1.0);
  const double condition_score = std::clamp(
    1.0 - std::log10(std::max(metrics.hessian_condition, 1.0)) /
    std::log10(std::max(thresholds.max_hessian_condition, 10.0)),
    0.0, 1.0);
  assessment.accepted = true;
  assessment.confidence = std::clamp(
    0.45 * inlier_score + 0.40 * fitness_score + 0.15 * condition_score,
    0.0, 1.0);
  assessment.reason = "accepted";
  return assessment;
}

class LocalizationStateMachine
{
public:
  struct Policy
  {
    int good_searches_to_track = 2;
    int bad_tracks_to_degrade = 2;
    int bad_degraded_to_lose = 5;
    int good_degraded_to_recover = 3;
  };

  LocalizationStateMachine()
  : LocalizationStateMachine(Policy{})
  {
  }

  explicit LocalizationStateMachine(Policy policy)
  : policy_(policy)
  {
  }

  LocalizationState state() const {return state_;}
  int consecutive_good() const {return consecutive_good_;}
  int consecutive_bad() const {return consecutive_bad_;}

  void begin_search()
  {
    state_ = LocalizationState::kSearching;
    consecutive_good_ = 0;
    consecutive_bad_ = 0;
  }

  void mark_lost()
  {
    state_ = LocalizationState::kLost;
    consecutive_good_ = 0;
    consecutive_bad_ = 0;
  }

  LocalizationState observe(bool accepted)
  {
    if (accepted) {
      ++consecutive_good_;
      consecutive_bad_ = 0;
      const bool acquiring = state_ == LocalizationState::kSearching ||
        state_ == LocalizationState::kUninitialized ||
        state_ == LocalizationState::kLost;
      const bool recovering = state_ == LocalizationState::kDegraded &&
        consecutive_good_ >= policy_.good_degraded_to_recover;
      if (acquiring) {
        if (consecutive_good_ >= policy_.good_searches_to_track) {
          state_ = LocalizationState::kTracking;
        }
      } else if (recovering) {
        state_ = LocalizationState::kTracking;
      }
      return state_;
    }

    ++consecutive_bad_;
    consecutive_good_ = 0;
    const bool degrading = state_ == LocalizationState::kTracking &&
      consecutive_bad_ >= policy_.bad_tracks_to_degrade;
    const bool losing = state_ == LocalizationState::kDegraded &&
      consecutive_bad_ >= policy_.bad_degraded_to_lose;
    if (degrading) {
      state_ = LocalizationState::kDegraded;
      consecutive_bad_ = 0;
    } else if (losing) {
      state_ = LocalizationState::kLost;
    }
    return state_;
  }

private:
  Policy policy_;
  LocalizationState state_ = LocalizationState::kUninitialized;
  int consecutive_good_ = 0;
  int consecutive_bad_ = 0;
};

}  // namespace go2_map_manager
