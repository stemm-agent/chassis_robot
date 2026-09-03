#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{
double yaw_of(const tf2::Transform & transform)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  return yaw;
}

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double allowed_odom_step(
  const double dt,
  const double max_speed,
  const double max_acceleration,
  const double slack,
  const double discontinuity_floor)
{
  const double bounded_dt = std::max(0.0, dt);
  const double kinematic_limit =
    slack + max_speed * bounded_dt +
    0.5 * max_acceleration * bounded_dt * bounded_dt;
  // The legacy max_odom_step_* settings are retained as minimum evidence that
  // a discontinuity occurred, not as time-independent caps on legitimate
  // motion.  A jump must exceed both this floor and the dt-scaled kinematic
  // envelope.  Source gaps are rejected separately before this is evaluated.
  return std::max(discontinuity_floor, kinematic_limit);
}

enum class OdomTimingSeverity
{
  kHealthy,
  kSoft,
  kHard,
};

enum class QualityCovariancePolicy
{
  kStrictConverged,
  kStructuralOnly,
  kIgnored,
};

enum class TrustedOverrideQuality
{
  kGood,
  kSoftFailure,
  kCatastrophic,
};

enum class TrustedOverrideFailureEvidence
{
  kNone,
  kEmergencyDrift,
  kTemporalNotReady,
};

OdomTimingSeverity classify_odom_timing_value(
  const double value,
  const double soft_limit,
  const double hard_limit)
{
  if (soft_limit <= 0.0 || value <= soft_limit) {
    return OdomTimingSeverity::kHealthy;
  }
  if (hard_limit > 0.0 && value >= hard_limit) {
    return OdomTimingSeverity::kHard;
  }
  return OdomTimingSeverity::kSoft;
}

double finite_bounded_parameter(
  const double value,
  const double lower,
  const double upper,
  const double fallback)
{
  return std::isfinite(value) ? std::clamp(value, lower, upper) : fallback;
}

tf2::Transform transform_from_pose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation;
  tf2::fromMsg(pose.orientation, rotation);
  return tf2::Transform(rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

tf2::Transform transform_from_msg(const geometry_msgs::msg::Transform & transform)
{
  tf2::Quaternion rotation;
  tf2::fromMsg(transform.rotation, rotation);
  return tf2::Transform(
    rotation,
    tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));
}

bool transform_is_finite(const tf2::Transform & transform)
{
  const auto & origin = transform.getOrigin();
  const auto & rotation = transform.getRotation();
  const double norm = std::sqrt(
    rotation.x() * rotation.x() + rotation.y() * rotation.y() +
    rotation.z() * rotation.z() + rotation.w() * rotation.w());
  return std::isfinite(origin.x()) && std::isfinite(origin.y()) &&
         std::isfinite(origin.z()) && std::isfinite(rotation.x()) &&
         std::isfinite(rotation.y()) && std::isfinite(rotation.z()) &&
         std::isfinite(rotation.w()) && norm > 0.5 && norm < 1.5;
}

struct CandidateMetrics
{
  double dt{0.0};
  double translation{0.0};
  double yaw_delta{0.0};
  double kinematic_linear_limit{0.0};
  double kinematic_angular_limit{0.0};
  double linear_limit{0.0};
  double angular_limit{0.0};
  double correction_translation{0.0};
  double correction_yaw{0.0};
  double cumulative_translation{0.0};
  double cumulative_yaw{0.0};
};

struct ScanMatchMetrics
{
  bool covariance_valid{false};
  bool covariance_converged{false};
  bool scan_available{false};
  bool scan_motion_compensated{false};
  bool base_in_free_space{false};
  std::size_t considered_beams{0};
  std::int64_t scan_stamp_ns{0};
  double scan_time_delta_sec{std::numeric_limits<double>::infinity()};
  double covariance_xy_lambda_max{std::numeric_limits<double>::infinity()};
  double covariance_yaw{std::numeric_limits<double>::infinity()};
  double inlier_ratio{0.0};
  double likelihood_mean{0.0};
  double trimmed_mean_distance{std::numeric_limits<double>::infinity()};
  double unknown_offmap_ratio{1.0};
  double best_neighbor_likelihood_gain{0.0};
  std::string reason{"bootstrap_quality_unavailable"};
};

struct MapDistanceField
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  double resolution{0.0};
  tf2::Transform map_origin;
  std::vector<std::int8_t> occupancy;
  std::vector<float> distance_m;
};

struct BootstrapSample
{
  tf2::Transform map_odom;
  tf2::Transform map_base;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point receipt{};
};

struct AnchorAdjustment
{
  std::chrono::steady_clock::time_point receipt{};
  double translation_x{0.0};
  double translation_y{0.0};
  double yaw{0.0};
};

struct OdomMotionSegment
{
  std::chrono::steady_clock::time_point receipt{};
  double translation{0.0};
  double yaw{0.0};
};

double directional_vector_step_limit(
  const double net_x,
  const double net_y,
  const double direction_x,
  const double direction_y,
  const double budget,
  const double requested_step)
{
  if (budget <= 0.0 || requested_step <= 0.0) {
    return 0.0;
  }
  const double direction_norm = std::hypot(direction_x, direction_y);
  if (direction_norm <= 1.0e-12) {
    return 0.0;
  }
  const double unit_x = direction_x / direction_norm;
  const double unit_y = direction_y / direction_norm;
  const double projection = net_x * unit_x + net_y * unit_y;
  const double discriminant = projection * projection + budget * budget -
    (net_x * net_x + net_y * net_y);
  if (discriminant < -1.0e-9) {
    // A valid history should never be outside its own effective budget.  Fail
    // closed instead of spending more anchor budget if state is inconsistent.
    return 0.0;
  }
  const double boundary_step = -projection + std::sqrt(std::max(0.0, discriminant));
  return std::min(requested_step, std::max(0.0, boundary_step));
}
}  // namespace

class AmclTfGuard : public rclcpp::Node
{
public:
  AmclTfGuard()
  : Node("amcl_tf_guard")
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom_combined");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/amcl_pose");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/amcl_tf_guard/mode");
    max_linear_speed_ = std::max(
      0.0, declare_parameter<double>("max_linear_speed", 0.5));
    max_angular_speed_ = std::max(
      0.0, declare_parameter<double>("max_angular_speed", 2.0));
    max_linear_acceleration_ = std::max(
      0.0, declare_parameter<double>("max_linear_acceleration", 1.5));
    max_angular_acceleration_ = std::max(
      0.0, declare_parameter<double>("max_angular_acceleration", 1.5));
    linear_slack_ = std::max(
      0.0, declare_parameter<double>("linear_slack", 0.15));
    angular_slack_ = std::max(
      0.0, declare_parameter<double>("angular_slack", 0.15));
    max_odom_correction_ = declare_parameter<double>("max_odom_correction", 0.30);
    max_odom_yaw_correction_ = declare_parameter<double>("max_odom_yaw_correction", 0.35);
    max_cumulative_map_odom_translation_ = std::max(
      0.0, declare_parameter<double>("max_cumulative_map_odom_translation", 0.30));
    max_cumulative_map_odom_yaw_ = std::max(
      0.0, declare_parameter<double>("max_cumulative_map_odom_yaw", 0.20));
    recovery_cumulative_map_odom_translation_ = std::min(
      max_cumulative_map_odom_translation_,
      std::max(
        0.0,
        declare_parameter<double>(
          "recovery_cumulative_map_odom_translation",
          max_cumulative_map_odom_translation_)));
    recovery_cumulative_map_odom_yaw_ = std::min(
      max_cumulative_map_odom_yaw_,
      std::max(
        0.0,
        declare_parameter<double>(
          "recovery_cumulative_map_odom_yaw", max_cumulative_map_odom_yaw_)));
    rolling_anchor_enabled_ = declare_parameter<bool>("rolling_anchor_enabled", true);
    rolling_anchor_translation_deadband_ = std::min(
      max_cumulative_map_odom_translation_,
      std::max(0.0, declare_parameter<double>("rolling_anchor_translation_deadband", 0.12)));
    rolling_anchor_yaw_deadband_ = std::min(
      max_cumulative_map_odom_yaw_,
      std::max(0.0, declare_parameter<double>("rolling_anchor_yaw_deadband", 0.08)));
    rolling_anchor_max_step_translation_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_max_step_translation", 0.03));
    rolling_anchor_max_step_yaw_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_max_step_yaw", 0.025));
    rolling_anchor_catchup_max_gap_sec_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_catchup_max_gap_sec", 4.0));
    rolling_anchor_catchup_max_step_scale_ = std::max(
      1.0, declare_parameter<double>("rolling_anchor_catchup_max_step_scale", 4.0));
    rolling_anchor_catchup_max_likelihood_drop_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_catchup_max_likelihood_drop", 0.03));
    rolling_anchor_window_sec_ = std::max(
      1.0, declare_parameter<double>("rolling_anchor_window_sec", 30.0));
    rolling_anchor_window_translation_budget_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_window_translation_budget", 0.12));
    rolling_anchor_window_yaw_budget_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_window_yaw_budget", 0.10));
    rolling_anchor_translation_budget_per_odom_meter_ = std::max(
      0.0,
      declare_parameter<double>("rolling_anchor_translation_budget_per_odom_meter", 0.15));
    rolling_anchor_yaw_budget_per_odom_rad_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_yaw_budget_per_odom_rad", 0.15));
    rolling_anchor_window_translation_motion_cap_ = std::max(
      0.0,
      declare_parameter<double>("rolling_anchor_window_translation_motion_cap", 0.18));
    rolling_anchor_window_yaw_motion_cap_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_window_yaw_motion_cap", 0.10));
    rolling_anchor_odom_translation_epsilon_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_odom_translation_epsilon", 0.001));
    rolling_anchor_odom_yaw_epsilon_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_odom_yaw_epsilon", 0.002));
    rolling_anchor_session_translation_budget_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_session_translation_budget", 2.0));
    rolling_anchor_session_yaw_budget_ = std::max(
      0.0, declare_parameter<double>("rolling_anchor_session_yaw_budget", 1.5));
    cumulative_overrun_defer_enabled_ =
      declare_parameter<bool>("cumulative_overrun_defer_enabled", true);
    cumulative_overrun_defer_consecutive_samples_ = std::max(
      1, static_cast<int>(
        declare_parameter<int>("cumulative_overrun_defer_consecutive_samples", 3)));
    cumulative_overrun_defer_min_duration_sec_ = std::max(
      0.0, declare_parameter<double>("cumulative_overrun_defer_min_duration_sec", 0.50));
    cumulative_overrun_defer_max_duration_sec_ = std::max(
      cumulative_overrun_defer_min_duration_sec_,
      declare_parameter<double>("cumulative_overrun_defer_max_duration_sec", 1.50));
    cumulative_overrun_defer_max_translation_excess_ = std::max(
      0.0,
      declare_parameter<double>("cumulative_overrun_defer_max_translation_excess", 0.30));
    cumulative_overrun_defer_max_yaw_excess_ = std::max(
      0.0, declare_parameter<double>("cumulative_overrun_defer_max_yaw_excess", 0.15));
    cumulative_overrun_defer_max_correction_translation_ = std::max(
      0.0,
      declare_parameter<double>("cumulative_overrun_defer_max_correction_translation", 0.10));
    cumulative_overrun_defer_max_correction_yaw_ = std::max(
      0.0, declare_parameter<double>("cumulative_overrun_defer_max_correction_yaw", 0.08));
    cumulative_overrun_defer_max_source_gap_sec_ = std::max(
      0.0,
      declare_parameter<double>("cumulative_overrun_defer_max_source_gap_sec", 0.75));
    cumulative_overrun_defer_max_odom_receipt_age_sec_ = std::max(
      0.0,
      declare_parameter<double>("cumulative_overrun_defer_max_odom_receipt_age_sec", 0.25));
    cumulative_overrun_defer_pose_odom_stamp_tolerance_sec_ = std::max(
      0.0,
      declare_parameter<double>(
        "cumulative_overrun_defer_pose_odom_stamp_tolerance_sec", 0.01));
    trusted_override_enabled_ = declare_parameter<bool>("trusted_override_enabled", true);
    trusted_override_active_consecutive_samples_ = std::max(
      2, static_cast<int>(
        declare_parameter<int>("trusted_override_active_consecutive_samples", 2)));
    trusted_override_active_min_duration_sec_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_active_min_duration_sec", 0.25),
      0.25, std::numeric_limits<double>::max(), 0.25);
    trusted_override_active_evidence_timeout_sec_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_active_evidence_timeout_sec", 1.00),
      0.50, std::numeric_limits<double>::max(), 1.00);
    trusted_override_hold_consecutive_samples_ = std::max(
      3, static_cast<int>(
        declare_parameter<int>("trusted_override_hold_consecutive_samples", 3)));
    trusted_override_hold_min_duration_sec_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_hold_min_duration_sec", 0.50),
      0.50, std::numeric_limits<double>::max(), 0.50);
    trusted_override_hold_stationary_duration_sec_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_hold_stationary_duration_sec", 0.40),
      0.30, std::numeric_limits<double>::max(), 0.40);
    trusted_override_verify_max_duration_sec_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_verify_max_duration_sec", 1.50),
      trusted_override_active_min_duration_sec_, std::numeric_limits<double>::max(),
      std::max(trusted_override_active_min_duration_sec_, 1.50));
    trusted_override_return_consecutive_samples_ = std::max(
      3, static_cast<int>(
        declare_parameter<int>("trusted_override_return_consecutive_samples", 3)));
    trusted_override_return_min_duration_sec_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_return_min_duration_sec", 0.50),
      0.50, std::numeric_limits<double>::max(), 0.50);
    trusted_override_min_valid_beams_ = std::max(
      1, static_cast<int>(declare_parameter<int>("trusted_override_min_valid_beams", 25)));
    trusted_override_min_inlier_ratio_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_min_inlier_ratio", 0.60), 0.0, 1.0, 0.60);
    trusted_override_min_likelihood_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_min_likelihood", 0.55), 0.0, 1.0, 0.55);
    trusted_override_max_trimmed_mean_distance_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_max_trimmed_mean_distance", 0.20),
      0.0, std::numeric_limits<double>::max(), 0.20);
    trusted_override_max_unknown_offmap_ratio_ = finite_bounded_parameter(
      declare_parameter<double>("trusted_override_max_unknown_offmap_ratio", 0.20),
      0.0, 1.0, 0.20);
    trusted_override_catastrophic_min_valid_beams_ = std::min(
      trusted_override_min_valid_beams_,
      std::max(
        1,
        static_cast<int>(
          declare_parameter<int>("trusted_override_catastrophic_min_valid_beams", 10))));
    trusted_override_catastrophic_min_inlier_ratio_ = std::min(
      trusted_override_min_inlier_ratio_,
      finite_bounded_parameter(
        declare_parameter<double>("trusted_override_catastrophic_min_inlier_ratio", 0.35),
        0.0, 1.0, 0.35));
    trusted_override_catastrophic_min_likelihood_ = std::min(
      trusted_override_min_likelihood_,
      finite_bounded_parameter(
        declare_parameter<double>("trusted_override_catastrophic_min_likelihood", 0.30),
        0.0, 1.0, 0.30));
    trusted_override_catastrophic_max_trimmed_mean_distance_ = std::max(
      trusted_override_max_trimmed_mean_distance_,
      finite_bounded_parameter(
        declare_parameter<double>(
          "trusted_override_catastrophic_max_trimmed_mean_distance", 0.35),
        0.0, std::numeric_limits<double>::max(), 0.35));
    trusted_override_catastrophic_max_unknown_offmap_ratio_ = std::max(
      trusted_override_max_unknown_offmap_ratio_,
      finite_bounded_parameter(
        declare_parameter<double>(
          "trusted_override_catastrophic_max_unknown_offmap_ratio", 0.40),
        0.0, 1.0, 0.40));
    trusted_override_soft_failure_consecutive_samples_ = std::max(
      2,
      static_cast<int>(
        declare_parameter<int>("trusted_override_soft_failure_consecutive_samples", 2)));
    active_quality_scan_tolerance_sec_ = std::max(
      0.0, declare_parameter<double>("active_quality_scan_tolerance_sec", 0.20));
    max_amcl_pose_age_sec_ = std::max(
      0.0, declare_parameter<double>("max_amcl_pose_age_sec", 2.0));
    max_future_pose_skew_sec_ = std::max(
      0.0, declare_parameter<double>("max_future_pose_skew_sec", 0.25));
    max_amcl_validation_gap_sec_ = std::max(
      0.0, declare_parameter<double>("max_amcl_validation_gap_sec", 1.0));
    amcl_silence_timeout_while_moving_sec_ = std::max(
      0.0, declare_parameter<double>("amcl_silence_timeout_while_moving_sec", 3.0));
    amcl_silence_motion_translation_ = std::max(
      0.0, declare_parameter<double>("amcl_silence_motion_translation", 0.30));
    amcl_silence_motion_yaw_ = std::max(
      0.0, declare_parameter<double>("amcl_silence_motion_yaw", 0.25));
    amcl_silence_motion_grace_sec_ = std::max(
      0.0, declare_parameter<double>("amcl_silence_motion_grace_sec", 1.0));
    max_publish_timer_gap_sec_ = std::max(
      0.0, declare_parameter<double>("max_publish_timer_gap_sec", 0.25));
    timer_recovery_consecutive_ticks_ = std::max(
      1, static_cast<int>(declare_parameter<int>("timer_recovery_consecutive_ticks", 10)));
    max_odom_observation_gap_sec_ = std::max(
      0.0, declare_parameter<double>("max_odom_observation_gap_sec", 1.0));
    max_odom_tf_age_sec_ = std::max(
      0.0, declare_parameter<double>("max_odom_tf_age_sec", 0.25));
    hard_odom_observation_gap_sec_ = std::max(
      max_odom_observation_gap_sec_,
      declare_parameter<double>("hard_odom_observation_gap_sec", 1.5));
    hard_odom_tf_age_sec_ = std::max(
      max_odom_tf_age_sec_,
      declare_parameter<double>("hard_odom_tf_age_sec", 1.0));
    odom_timing_soft_fault_consecutive_samples_ = std::max(
      1,
      static_cast<int>(declare_parameter<int>(
          "odom_timing_soft_fault_consecutive_samples", 3)));
    odom_timing_soft_fault_min_duration_sec_ = std::max(
      0.0, declare_parameter<double>("odom_timing_soft_fault_min_duration_sec", 0.25));
    max_odom_step_translation_ = std::max(
      0.0, declare_parameter<double>("max_odom_step_translation", 0.35));
    max_odom_step_yaw_ = std::max(
      0.0, declare_parameter<double>("max_odom_step_yaw", 0.35));
    max_map_base_step_translation_ = std::max(
      0.0, declare_parameter<double>("max_map_base_step_translation", 1.0));
    max_map_base_step_yaw_ = std::max(
      0.0, declare_parameter<double>("max_map_base_step_yaw", 1.0));
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 30.0));
    bootstrap_consecutive_valid_samples_ = std::max(
      1, static_cast<int>(declare_parameter<int>("bootstrap_consecutive_valid_samples", 5)));
    bootstrap_requires_initialpose_ =
      declare_parameter<bool>("bootstrap_requires_initialpose", false);
    auto_global_localization_ =
      declare_parameter<bool>("auto_global_localization", false);
    allow_manual_rebase_ = declare_parameter<bool>("allow_manual_rebase", false);
    auto_initial_pose_enabled_ =
      declare_parameter<bool>("auto_initial_pose_enabled", true);
    auto_initial_pose_x_ = declare_parameter<double>("auto_initial_pose_x", 0.0);
    auto_initial_pose_y_ = declare_parameter<double>("auto_initial_pose_y", 0.0);
    auto_initial_pose_yaw_ = declare_parameter<double>("auto_initial_pose_yaw", 0.0);
    auto_initial_pose_variance_x_ = std::max(
      0.0, declare_parameter<double>("auto_initial_pose_variance_x", 0.25));
    auto_initial_pose_variance_y_ = std::max(
      0.0, declare_parameter<double>("auto_initial_pose_variance_y", 0.25));
    auto_initial_pose_variance_yaw_ = std::max(
      0.0, declare_parameter<double>(
        "auto_initial_pose_variance_yaw", 0.06853891945200942));
    // These values are only an automatic coarse AMCL prior.  They never become
    // the guard anchor directly; the anchor is authored from the medoid of the
    // later scan-map/covariance-qualified confirmation window.
    epoch_initial_pose_x_ = auto_initial_pose_x_;
    epoch_initial_pose_y_ = auto_initial_pose_y_;
    epoch_initial_pose_yaw_ = auto_initial_pose_yaw_;
    global_localization_service_ = declare_parameter<std::string>(
      "global_localization_service", "/reinitialize_global_localization");
    bootstrap_min_epoch_age_sec_ = std::max(
      0.0, declare_parameter<double>("bootstrap_min_epoch_age_sec", 5.0));
    bootstrap_min_post_global_updates_ = std::max(
      1, static_cast<int>(declare_parameter<int>("bootstrap_min_post_global_updates", 3)));
    bootstrap_good_samples_ = std::max(
      2, static_cast<int>(declare_parameter<int>("bootstrap_good_samples", 6)));
    bootstrap_min_confirm_duration_sec_ = std::max(
      0.0, declare_parameter<double>("bootstrap_min_confirm_duration_sec", 3.0));
    bootstrap_seed_max_translation_ = std::max(
      0.0, declare_parameter<double>("bootstrap_seed_max_translation", 0.08));
    bootstrap_seed_max_yaw_ = std::max(
      0.0, declare_parameter<double>("bootstrap_seed_max_yaw", 0.06));
    bootstrap_cov_max_xy_eigenvalue_ = std::max(
      0.0, declare_parameter<double>("bootstrap_cov_max_xy_eigenvalue", 0.04));
    bootstrap_cov_max_yaw_ = std::max(
      0.0, declare_parameter<double>("bootstrap_cov_max_yaw", 0.0225));
    bootstrap_zero_cov_epsilon_ = std::max(
      0.0, declare_parameter<double>("bootstrap_zero_cov_epsilon", 1.0e-10));
    bootstrap_timeout_sec_ = std::max(
      1.0, declare_parameter<double>("bootstrap_timeout_sec", 45.0));
    bootstrap_max_nomotion_requests_ = std::max(
      1, static_cast<int>(declare_parameter<int>("bootstrap_max_nomotion_requests", 20)));
    bootstrap_retry_cooldown_sec_ = std::max(
      1.0, declare_parameter<double>("bootstrap_retry_cooldown_sec", 15.0));
    score_occupied_threshold_ = std::max(
      1, std::min(100, static_cast<int>(
        declare_parameter<int>("score_occupied_threshold", 65))));
    score_max_beams_ = std::max(
      1, static_cast<int>(declare_parameter<int>("score_max_beams", 90)));
    score_min_valid_beams_ = std::max(
      1, static_cast<int>(declare_parameter<int>("score_min_valid_beams", 35)));
    score_sigma_m_ = std::max(
      0.01, declare_parameter<double>("score_sigma_m", 0.20));
    score_inlier_distance_m_ = std::max(
      0.0, declare_parameter<double>("score_inlier_distance_m", 0.20));
    score_min_inlier_ratio_ = std::max(
      0.0, std::min(1.0, declare_parameter<double>("score_min_inlier_ratio", 0.65)));
    score_min_likelihood_ = std::max(
      0.0, std::min(1.0, declare_parameter<double>("score_min_likelihood", 0.55)));
    score_max_trimmed_mean_distance_ = std::max(
      0.0, declare_parameter<double>("score_max_trimmed_mean_distance", 0.18));
    score_max_unknown_offmap_ratio_ = std::max(
      0.0, std::min(
        1.0, declare_parameter<double>("score_max_unknown_offmap_ratio", 0.10)));
    score_neighbor_translation_m_ = std::max(
      0.0, declare_parameter<double>("score_neighbor_translation_m", 0.10));
    score_neighbor_yaw_rad_ = std::max(
      0.0, declare_parameter<double>("score_neighbor_yaw_rad", 0.15));
    score_max_neighbor_likelihood_gain_ = std::max(
      0.0, declare_parameter<double>("score_max_neighbor_likelihood_gain", 0.04));
    recovery_consecutive_valid_samples_ = std::max(
      1, static_cast<int>(declare_parameter<int>("recovery_consecutive_valid_samples", 5)));
    initialpose_rebase_consecutive_valid_samples_ = std::max(
      1, static_cast<int>(declare_parameter<int>(
        "initialpose_rebase_consecutive_valid_samples", 5)));
    initialpose_rebase_position_tolerance_ = std::max(
      0.0, declare_parameter<double>("initialpose_rebase_position_tolerance", 0.35));
    initialpose_rebase_yaw_tolerance_ = std::max(
      0.0, declare_parameter<double>("initialpose_rebase_yaw_tolerance", 0.35));
    recovery_cluster_consecutive_samples_ = std::max(
      2, static_cast<int>(declare_parameter<int>("recovery_cluster_consecutive_samples", 5)));
    recovery_cluster_translation_tolerance_ = std::max(
      0.0, declare_parameter<double>("recovery_cluster_translation_tolerance", 0.10));
    recovery_cluster_yaw_tolerance_ = std::max(
      0.0, declare_parameter<double>("recovery_cluster_yaw_tolerance", 0.10));
    recovery_cluster_odom_translation_tolerance_ = std::max(
      0.0, declare_parameter<double>("recovery_cluster_odom_translation_tolerance", 0.025));
    recovery_cluster_odom_yaw_tolerance_ = std::max(
      0.0, declare_parameter<double>("recovery_cluster_odom_yaw_tolerance", 0.025));
    recovery_cluster_min_duration_sec_ = std::max(
      0.0, declare_parameter<double>("recovery_cluster_min_duration_sec", 3.0));
    recovery_stationary_duration_sec_ = std::max(
      0.0, declare_parameter<double>("recovery_stationary_duration_sec", 1.0));
    interlock_blocked_topic_ = declare_parameter<std::string>(
      "interlock_blocked_topic", "/nav2_guard_interlock/blocked");
    interlock_blocked_max_age_sec_ = std::max(
      0.05, declare_parameter<double>("interlock_blocked_max_age_sec", 0.25));
    auto_reanchor_enabled_ = declare_parameter<bool>("auto_reanchor_enabled", true);
    auto_reanchor_max_translation_ = std::max(
      max_cumulative_map_odom_translation_,
      declare_parameter<double>("auto_reanchor_max_translation", 1.0));
    auto_reanchor_max_yaw_ = std::max(
      max_cumulative_map_odom_yaw_,
      declare_parameter<double>("auto_reanchor_max_yaw", 0.75));
    auto_reanchor_min_likelihood_improvement_ = std::max(
      0.0, declare_parameter<double>("auto_reanchor_min_likelihood_improvement", 0.08));
    auto_reanchor_max_odom_receipt_age_sec_ = std::max(
      0.05, declare_parameter<double>("auto_reanchor_max_odom_receipt_age_sec", 0.25));
    auto_reanchor_cooldown_sec_ = std::max(
      0.0, declare_parameter<double>("auto_reanchor_cooldown_sec", 10.0));
    max_auto_reanchors_per_session_ = std::max(
      0, static_cast<int>(declare_parameter<int>("max_auto_reanchors_per_session", 1)));
    post_reanchor_consecutive_valid_samples_ = std::max(
      2,
      static_cast<int>(declare_parameter<int>(
          "post_reanchor_consecutive_valid_samples", 5)));
    post_reanchor_min_confirm_duration_sec_ = std::max(
      0.0, declare_parameter<double>("post_reanchor_min_confirm_duration_sec", 2.0));
    nomotion_update_service_ = declare_parameter<std::string>(
      "nomotion_update_service", "/request_nomotion_update");
    nomotion_update_period_sec_ = std::max(
      0.0, declare_parameter<double>("nomotion_update_period_sec", 1.0));
    recovery_nomotion_max_requests_ = std::max(
      1, static_cast<int>(declare_parameter<int>("recovery_nomotion_max_requests", 12)));
    recovery_nomotion_max_window_sec_ = std::max(
      0.5, declare_parameter<double>("recovery_nomotion_max_window_sec", 8.0));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    anomaly_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/amcl_tf_guard/anomaly", rclcpp::QoS(1).transient_local());
    auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    mode_qos.reliable().transient_local();
    mode_publisher_ = create_publisher<std_msgs::msg::String>(
      mode_topic_, mode_qos);
    auto interlock_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    interlock_qos.reliable().transient_local();
    interlock_blocked_subscription_ = create_subscription<std_msgs::msg::Bool>(
      interlock_blocked_topic_, interlock_qos,
      std::bind(&AmclTfGuard::on_interlock_blocked, this, std::placeholders::_1));

    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(20),
      std::bind(&AmclTfGuard::on_amcl_pose, this, std::placeholders::_1));
    if (allow_manual_rebase_) {
      initialpose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initialpose_topic_, rclcpp::QoS(10),
        std::bind(&AmclTfGuard::on_initialpose, this, std::placeholders::_1));
    }
    initialpose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(10));
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AmclTfGuard::on_map, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&AmclTfGuard::on_scan, this, std::placeholders::_1));
    nomotion_update_client_ = create_client<std_srvs::srv::Empty>(nomotion_update_service_);
    global_localization_client_ =
      create_client<std_srvs::srv::Empty>(global_localization_service_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&AmclTfGuard::on_publish_timer, this));
    nomotion_update_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.1, nomotion_update_period_sec_)),
      std::bind(&AmclTfGuard::request_nomotion_update_if_needed, this));

    // There is no safe map->odom transform before automatic AMCL acquisition,
    // scan-map scoring, covariance, and a fixed-seed window have
    // all been validated.  Starting in HOLD also blocks summon navigation.
    publish_anomaly(true);
    publish_guard_mode();
    RCLCPP_WARN(
      get_logger(),
      "AMCL TF guard owns %s -> %s and starts in HOLD. AMCL must run with "
      "tf_broadcast=false; automatic localization plus %d qualified AMCL/scan "
      "updates are required before publishing a trusted transform. Per-anchor "
      "cumulative limits are %.3f m / %.3f rad; rolling correction has bounded "
      "window/session budgets and automatic far re-anchor requires a fresh BLOCKED "
      "interlock heartbeat (manual_rebase=%s).",
      global_frame_.c_str(), odom_frame_.c_str(), bootstrap_good_samples_,
      max_cumulative_map_odom_translation_, max_cumulative_map_odom_yaw_,
      allow_manual_rebase_ ? "enabled" : "disabled");
    RCLCPP_INFO(
      get_logger(),
      "TRUSTED_OVERRIDE config: active=%d/%.3f s, verify_deadline=%.3f s, "
      "active_evidence_deadline=%.3f s, soft_failures=%d, soft_hold=%d/%.3f s "
      "while stationary for %.3f s, return=%d/%.3f s, scan_tolerance=%.3f s.",
      trusted_override_active_consecutive_samples_,
      trusted_override_active_min_duration_sec_,
      trusted_override_verify_max_duration_sec_,
      trusted_override_active_evidence_timeout_sec_,
      trusted_override_soft_failure_consecutive_samples_,
      trusted_override_hold_consecutive_samples_,
      trusted_override_hold_min_duration_sec_,
      trusted_override_hold_stationary_duration_sec_,
      trusted_override_return_consecutive_samples_,
      trusted_override_return_min_duration_sec_,
      active_quality_scan_tolerance_sec_);
    RCLCPP_INFO(
      get_logger(),
      "TRUSTED_OVERRIDE quality thresholds: normal beams=%d inlier=%.3f "
      "likelihood=%.3f trimmed=%.3f unknown/offmap=%.3f; catastrophic beams=%d "
      "inlier=%.3f likelihood=%.3f trimmed=%.3f unknown/offmap=%.3f.",
      trusted_override_min_valid_beams_, trusted_override_min_inlier_ratio_,
      trusted_override_min_likelihood_, trusted_override_max_trimmed_mean_distance_,
      trusted_override_max_unknown_offmap_ratio_,
      trusted_override_catastrophic_min_valid_beams_,
      trusted_override_catastrophic_min_inlier_ratio_,
      trusted_override_catastrophic_min_likelihood_,
      trusted_override_catastrophic_max_trimmed_mean_distance_,
      trusted_override_catastrophic_max_unknown_offmap_ratio_);
  }

private:
  void on_interlock_blocked(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool ended_blocked_epoch = have_interlock_blocked_receipt_ &&
      interlock_blocked_ && !message->data;
    interlock_blocked_ = message->data;
    have_interlock_blocked_receipt_ = true;
    last_interlock_blocked_receipt_ = std::chrono::steady_clock::now();
    if (ended_blocked_epoch) {
      // Never carry a 4/N far-candidate or post-reanchor confirmation window
      // across an interlock release/re-block epoch.
      reset_pending_locked();
    }
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (message->info.width == 0 || message->info.height == 0 ||
      !std::isfinite(message->info.resolution) || message->info.resolution <= 0.0 ||
      message->data.size() !=
      static_cast<std::size_t>(message->info.width) * message->info.height)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid occupancy map while automatic localization remains unarmed.");
      return;
    }

    auto field = std::make_shared<MapDistanceField>();
    field->width = message->info.width;
    field->height = message->info.height;
    field->resolution = message->info.resolution;
    field->map_origin = transform_from_pose(message->info.origin);
    field->occupancy = message->data;
    const std::size_t cell_count = field->occupancy.size();
    const float infinity = std::numeric_limits<float>::infinity();
    field->distance_m.assign(cell_count, infinity);
    for (std::size_t index = 0; index < cell_count; ++index) {
      if (field->occupancy[index] >= score_occupied_threshold_) {
        field->distance_m[index] = 0.0F;
      }
    }

    const float axial = static_cast<float>(field->resolution);
    const float diagonal = static_cast<float>(field->resolution * std::sqrt(2.0));
    const auto relax = [&field](
      const std::uint32_t x, const std::uint32_t y,
      const int nx, const int ny, const float cost)
      {
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(field->width) ||
          ny >= static_cast<int>(field->height))
        {
          return;
        }
        const std::size_t index = static_cast<std::size_t>(y) * field->width + x;
        const std::size_t neighbor =
          static_cast<std::size_t>(ny) * field->width + static_cast<std::uint32_t>(nx);
        field->distance_m[index] = std::min(
          field->distance_m[index], field->distance_m[neighbor] + cost);
      };

    for (std::uint32_t y = 0; y < field->height; ++y) {
      for (std::uint32_t x = 0; x < field->width; ++x) {
        relax(x, y, static_cast<int>(x) - 1, static_cast<int>(y), axial);
        relax(x, y, static_cast<int>(x), static_cast<int>(y) - 1, axial);
        relax(x, y, static_cast<int>(x) - 1, static_cast<int>(y) - 1, diagonal);
        relax(x, y, static_cast<int>(x) + 1, static_cast<int>(y) - 1, diagonal);
      }
    }
    for (int y = static_cast<int>(field->height) - 1; y >= 0; --y) {
      for (int x = static_cast<int>(field->width) - 1; x >= 0; --x) {
        relax(
          static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
          x + 1, y, axial);
        relax(
          static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
          x, y + 1, axial);
        relax(
          static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
          x + 1, y + 1, diagonal);
        relax(
          static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
          x - 1, y + 1, diagonal);
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_distance_field_ = field;
    }
    RCLCPP_INFO(
      get_logger(), "Automatic-localization map distance field ready (%u x %u, %.3f m/cell).",
      field->width, field->height, field->resolution);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    if (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0) {
      return;
    }
    const rclcpp::Time scan_stamp(message->header.stamp);
    const auto steady_now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_new_scan_observation_ || scan_stamp > last_new_scan_stamp_) {
      have_new_scan_observation_ = true;
      last_new_scan_stamp_ = scan_stamp;
      last_new_scan_observation_steady_ = steady_now;
    }
    scan_buffer_.push_back(message);
    while (scan_buffer_.size() > 64) {
      scan_buffer_.pop_front();
    }
  }

  sensor_msgs::msg::LaserScan::SharedPtr select_quality_scan(
    const rclcpp::Time & pose_stamp,
    const bool allow_nearest,
    ScanMatchMetrics * metrics)
  {
    sensor_msgs::msg::LaserScan::SharedPtr nearest_scan;
    double nearest_delta_sec = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto iterator = scan_buffer_.rbegin(); iterator != scan_buffer_.rend(); ++iterator) {
        const rclcpp::Time scan_stamp((*iterator)->header.stamp);
        if (scan_stamp == pose_stamp) {
          metrics->scan_available = true;
          metrics->scan_time_delta_sec = 0.0;
          return *iterator;
        }
        if (!allow_nearest) {
          continue;
        }
        const double delta_sec = std::abs((scan_stamp - pose_stamp).seconds());
        if (delta_sec < nearest_delta_sec) {
          nearest_delta_sec = delta_sec;
          nearest_scan = *iterator;
        }
      }
    }

    if (nearest_scan && nearest_delta_sec <= active_quality_scan_tolerance_sec_) {
      metrics->scan_available = true;
      metrics->scan_time_delta_sec = nearest_delta_sec;
      return nearest_scan;
    }
    metrics->reason = allow_nearest ?
      "active_quality_no_scan_within_tolerance" : "bootstrap_waiting_same_stamp_scan";
    return nullptr;
  }

  bool evaluate_bootstrap_quality(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message,
    const tf2::Transform & candidate_map_odom,
    const tf2::Transform & candidate_map_base,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    const QualityCovariancePolicy covariance_policy,
    ScanMatchMetrics * metrics)
  {
    const auto & covariance = message.pose.covariance;
    const double xx = covariance[0];
    const double xy = covariance[1];
    const double yx = covariance[6];
    const double yy = covariance[7];
    const double yaw_variance = covariance[35];
    const bool finite_covariance = std::isfinite(xx) && std::isfinite(xy) &&
      std::isfinite(yx) && std::isfinite(yy) && std::isfinite(yaw_variance);
    const double symmetric_xy = 0.5 * (xy + yx);
    const double determinant = xx * yy - symmetric_xy * symmetric_xy;
    const double discriminant = std::max(
      0.0, (xx - yy) * (xx - yy) + 4.0 * symmetric_xy * symmetric_xy);
    metrics->covariance_xy_lambda_max =
      0.5 * (xx + yy + std::sqrt(discriminant));
    metrics->covariance_yaw = yaw_variance;
    const bool zero_covariance = std::abs(xx) <= bootstrap_zero_cov_epsilon_ &&
      std::abs(yy) <= bootstrap_zero_cov_epsilon_ &&
      std::abs(yaw_variance) <= bootstrap_zero_cov_epsilon_;
    const bool covariance_structurally_valid = finite_covariance && !zero_covariance &&
      xx >= 0.0 && yy >= 0.0 && yaw_variance >= 0.0 && determinant >= -1.0e-8 &&
      std::abs(xy - yx) <= 1.0e-5;
    metrics->covariance_converged = covariance_structurally_valid &&
      metrics->covariance_xy_lambda_max <= bootstrap_cov_max_xy_eigenvalue_ &&
      yaw_variance <= bootstrap_cov_max_yaw_;
    metrics->covariance_valid = covariance_policy == QualityCovariancePolicy::kIgnored ||
      (covariance_structurally_valid &&
      (covariance_policy == QualityCovariancePolicy::kStructuralOnly ||
      metrics->covariance_converged));
    if (covariance_policy != QualityCovariancePolicy::kIgnored &&
      !covariance_structurally_valid)
    {
      metrics->reason = zero_covariance ?
        "bootstrap_zero_covariance" : "quality_covariance_structurally_invalid";
      return false;
    }
    if (covariance_policy == QualityCovariancePolicy::kStrictConverged &&
      !metrics->covariance_converged)
    {
      metrics->reason = "bootstrap_covariance_not_converged";
      return false;
    }

    std::shared_ptr<const MapDistanceField> field;
    const rclcpp::Time pose_stamp(message.header.stamp);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      field = map_distance_field_;
    }
    if (!field) {
      metrics->reason = "bootstrap_waiting_map";
      return false;
    }
    if (!scan) {
      if (metrics->reason == "bootstrap_quality_unavailable") {
        metrics->reason = "bootstrap_waiting_same_stamp_scan";
      }
      return false;
    }
    metrics->scan_available = true;

    const rclcpp::Time scan_stamp(scan->header.stamp);
    metrics->scan_stamp_ns = scan_stamp.nanoseconds();
    metrics->scan_time_delta_sec = std::abs((scan_stamp - pose_stamp).seconds());
    tf2::Transform scoring_map_base = candidate_map_base;
    if (scan_stamp != pose_stamp) {
      geometry_msgs::msg::TransformStamped odom_to_base_at_scan_message;
      try {
        odom_to_base_at_scan_message = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, scan_stamp,
          rclcpp::Duration::from_seconds(0.05));
      } catch (const tf2::TransformException &) {
        metrics->reason = "quality_missing_scan_stamp_odom_tf";
        return false;
      }
      scoring_map_base = candidate_map_odom *
        transform_from_msg(odom_to_base_at_scan_message.transform);
      metrics->scan_motion_compensated = true;
    }
    if (!transform_is_finite(scoring_map_base)) {
      metrics->reason = "quality_invalid_motion_compensated_pose";
      return false;
    }

    geometry_msgs::msg::TransformStamped base_to_scan_message;
    try {
      base_to_scan_message = tf_buffer_->lookupTransform(
        base_frame_, scan->header.frame_id, scan_stamp,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &) {
      metrics->reason = "quality_missing_base_to_scan_tf";
      return false;
    }
    const tf2::Transform map_to_grid = field->map_origin.inverse();
    const tf2::Transform candidate_map_scan =
      scoring_map_base * transform_from_msg(base_to_scan_message.transform);

    const tf2::Vector3 base_grid = map_to_grid * scoring_map_base.getOrigin();
    const int base_x = static_cast<int>(std::floor(base_grid.x() / field->resolution));
    const int base_y = static_cast<int>(std::floor(base_grid.y() / field->resolution));
    if (base_x >= 0 && base_y >= 0 && base_x < static_cast<int>(field->width) &&
      base_y < static_cast<int>(field->height))
    {
      const std::size_t base_index =
        static_cast<std::size_t>(base_y) * field->width + static_cast<std::uint32_t>(base_x);
      metrics->base_in_free_space = field->occupancy[base_index] >= 0 &&
        field->occupancy[base_index] < score_occupied_threshold_;
    }
    if (!metrics->base_in_free_space) {
      metrics->reason = "bootstrap_base_not_in_known_free_space";
      return false;
    }

    const std::size_t range_count = scan->ranges.size();
    const std::size_t stride = std::max<std::size_t>(
      1, (range_count + static_cast<std::size_t>(score_max_beams_) - 1) /
      static_cast<std::size_t>(score_max_beams_));
    std::vector<double> residuals;
    residuals.reserve(score_max_beams_);
    std::size_t unknown_or_offmap = 0;
    std::size_t inliers = 0;
    double likelihood_sum = 0.0;
    const double inverse_two_sigma_squared = 1.0 / (2.0 * score_sigma_m_ * score_sigma_m_);
    for (std::size_t index = 0; index < range_count; index += stride) {
      const double range = scan->ranges[index];
      if (!std::isfinite(range) || range <= scan->range_min || range >= scan->range_max) {
        continue;
      }
      ++metrics->considered_beams;
      const double angle = scan->angle_min + static_cast<double>(index) * scan->angle_increment;
      const tf2::Vector3 point_scan(range * std::cos(angle), range * std::sin(angle), 0.0);
      const tf2::Vector3 point_grid = map_to_grid * (candidate_map_scan * point_scan);
      const int grid_x = static_cast<int>(std::floor(point_grid.x() / field->resolution));
      const int grid_y = static_cast<int>(std::floor(point_grid.y() / field->resolution));
      if (grid_x < 0 || grid_y < 0 || grid_x >= static_cast<int>(field->width) ||
        grid_y >= static_cast<int>(field->height))
      {
        ++unknown_or_offmap;
        continue;
      }
      const std::size_t grid_index =
        static_cast<std::size_t>(grid_y) * field->width + static_cast<std::uint32_t>(grid_x);
      if (field->occupancy[grid_index] < 0 ||
        !std::isfinite(field->distance_m[grid_index]))
      {
        ++unknown_or_offmap;
        continue;
      }
      const double distance = field->distance_m[grid_index];
      residuals.push_back(distance);
      if (distance <= score_inlier_distance_m_) {
        ++inliers;
      }
      likelihood_sum += std::exp(-distance * distance * inverse_two_sigma_squared);
    }

    if (metrics->considered_beams > 0U) {
      metrics->unknown_offmap_ratio =
        static_cast<double>(unknown_or_offmap) / metrics->considered_beams;
      metrics->inlier_ratio = static_cast<double>(inliers) / metrics->considered_beams;
      metrics->likelihood_mean = likelihood_sum / metrics->considered_beams;
    }
    if (!residuals.empty()) {
      std::sort(residuals.begin(), residuals.end());
      const std::size_t trimmed_count = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(0.8 * residuals.size())));
      double trimmed_sum = 0.0;
      for (std::size_t index = 0; index < trimmed_count; ++index) {
        trimmed_sum += residuals[index];
      }
      metrics->trimmed_mean_distance = trimmed_sum / trimmed_count;
    }

    // Always populate the absolute scan metrics before applying the normal
    // AMCL-candidate beam threshold.  TRUSTED_OVERRIDE deliberately uses a
    // lower, explicit beam floor while validating the already-published
    // last-trusted transform; it never authorizes the raw AMCL candidate.
    // It also does not need the six-neighbor local-optimum search below.  Exit
    // after one beam pass so continuous override validation reduces rather than
    // increases executor load during navigation.
    if (covariance_policy == QualityCovariancePolicy::kIgnored) {
      metrics->reason = "absolute_scan_metrics_available";
      return true;
    }
    if (metrics->considered_beams < static_cast<std::size_t>(score_min_valid_beams_)) {
      metrics->reason = "bootstrap_too_few_valid_scan_beams";
      return false;
    }

    const auto likelihood_for_pose = [&](const tf2::Transform & map_base) {
        const tf2::Transform map_scan =
          map_base * transform_from_msg(base_to_scan_message.transform);
        double sum = 0.0;
        std::size_t considered = 0;
        for (std::size_t index = 0; index < range_count; index += stride) {
          const double range = scan->ranges[index];
          if (!std::isfinite(range) || range <= scan->range_min || range >= scan->range_max) {
            continue;
          }
          ++considered;
          const double angle =
            scan->angle_min + static_cast<double>(index) * scan->angle_increment;
          const tf2::Vector3 point_scan(
            range * std::cos(angle), range * std::sin(angle), 0.0);
          const tf2::Vector3 point_grid = map_to_grid * (map_scan * point_scan);
          const int grid_x = static_cast<int>(std::floor(point_grid.x() / field->resolution));
          const int grid_y = static_cast<int>(std::floor(point_grid.y() / field->resolution));
          if (grid_x < 0 || grid_y < 0 || grid_x >= static_cast<int>(field->width) ||
            grid_y >= static_cast<int>(field->height))
          {
            continue;
          }
          const std::size_t grid_index =
            static_cast<std::size_t>(grid_y) * field->width +
            static_cast<std::uint32_t>(grid_x);
          if (field->occupancy[grid_index] < 0 ||
            !std::isfinite(field->distance_m[grid_index]))
          {
            continue;
          }
          const double distance = field->distance_m[grid_index];
          sum += std::exp(-distance * distance * inverse_two_sigma_squared);
        }
        return considered > 0 ? sum / considered : 0.0;
      };
    std::vector<tf2::Transform> neighboring_poses;
    neighboring_poses.reserve(6);
    for (const double x_offset : {-score_neighbor_translation_m_, score_neighbor_translation_m_}) {
      tf2::Transform neighbor = scoring_map_base;
      neighbor.getOrigin().setX(neighbor.getOrigin().x() + x_offset);
      neighboring_poses.push_back(neighbor);
    }
    for (const double y_offset : {-score_neighbor_translation_m_, score_neighbor_translation_m_}) {
      tf2::Transform neighbor = scoring_map_base;
      neighbor.getOrigin().setY(neighbor.getOrigin().y() + y_offset);
      neighboring_poses.push_back(neighbor);
    }
    for (const double yaw_offset : {-score_neighbor_yaw_rad_, score_neighbor_yaw_rad_}) {
      tf2::Transform neighbor = scoring_map_base;
      tf2::Quaternion rotation;
      rotation.setRPY(0.0, 0.0, yaw_of(scoring_map_base) + yaw_offset);
      neighbor.setRotation(rotation);
      neighboring_poses.push_back(neighbor);
    }
    double best_neighbor_likelihood = metrics->likelihood_mean;
    for (const auto & neighbor : neighboring_poses) {
      best_neighbor_likelihood = std::max(
        best_neighbor_likelihood, likelihood_for_pose(neighbor));
    }
    metrics->best_neighbor_likelihood_gain =
      best_neighbor_likelihood - metrics->likelihood_mean;
    if (metrics->unknown_offmap_ratio > score_max_unknown_offmap_ratio_) {
      metrics->reason = "bootstrap_scan_unknown_or_offmap";
      return false;
    }
    if (metrics->inlier_ratio < score_min_inlier_ratio_ ||
      metrics->likelihood_mean < score_min_likelihood_ ||
      metrics->trimmed_mean_distance > score_max_trimmed_mean_distance_)
    {
      metrics->reason = "bootstrap_scan_map_mismatch";
      return false;
    }
    if (metrics->best_neighbor_likelihood_gain > score_max_neighbor_likelihood_gain_) {
      metrics->reason = "bootstrap_candidate_not_local_scan_optimum";
      return false;
    }
    metrics->reason = "bootstrap_quality_pass";
    return true;
  }

  bool is_spatial_fault_reason(const std::string & reason) const
  {
    return reason == "unexpected_amcl_frame" ||
           reason == "non_finite_amcl_pose" ||
           reason == "invalid_amcl_quaternion" ||
           reason == "invalid_spatial_transform";
  }

  bool candidate_from_amcl_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message,
    tf2::Transform * candidate_map_odom,
    tf2::Transform * candidate_map_base,
    tf2::Transform * current_odom_base,
    rclcpp::Time * current_odom_stamp,
    bool * used_latest_odom,
    std::string * reason)
  {
    *used_latest_odom = false;
    if (message.header.frame_id != global_frame_) {
      *reason = "unexpected_amcl_frame";
      return false;
    }
    const rclcpp::Time stamp(message.header.stamp);
    if (stamp.nanoseconds() == 0) {
      *reason = "zero_amcl_stamp";
      return false;
    }
    const auto & pose = message.pose.pose;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w))
    {
      *reason = "non_finite_amcl_pose";
      return false;
    }
    const double quaternion_norm =
      std::sqrt(pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
    if (quaternion_norm < 0.5 || quaternion_norm > 1.5) {
      *reason = "invalid_amcl_quaternion";
      return false;
    }

    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
      // Pair the pose with odom at the same source timestamp.  If that sample
      // is no longer buffered, discard it as a timing diagnostic; do not pair
      // an old AMCL pose with a newer odom pose and call it a spatial jump.
      odom_to_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &) {
      // Never combine an old AMCL pose with a newer odom sample, including
      // bootstrap.  Such a mixed-time pair can establish a permanently wrong
      // fixed anchor while still looking spatially self-consistent.
      *reason = "missing_same_stamp_odom_tf";
      return false;
    }

    *candidate_map_base = transform_from_pose(message.pose.pose);
    *current_odom_base = transform_from_msg(odom_to_base.transform);
    *current_odom_stamp = rclcpp::Time(odom_to_base.header.stamp);
    if (!transform_is_finite(*current_odom_base) || !transform_is_finite(*candidate_map_base)) {
      *reason = "invalid_spatial_transform";
      return false;
    }
    *candidate_map_odom = *candidate_map_base * current_odom_base->inverse();
    return true;
  }

  bool candidate_is_reasonable(
    const tf2::Transform & reference_map_odom,
    const tf2::Transform & reference_map_base,
    const rclcpp::Time & reference_stamp,
    const tf2::Transform & candidate_map_base,
    const tf2::Transform & current_odom_base,
    const rclcpp::Time & stamp,
    CandidateMetrics * metrics,
    std::string * reason) const
  {
    // Delayed or irregular timestamps are deliberately not a direct fault
    // criterion.  Preserve the real source interval for diagnostics and for
    // the separately quality-gated rolling-anchor catch-up path below; spatial
    // continuity still comes from the same-stamp odom prediction residual.
    metrics->dt = (stamp - reference_stamp).seconds();
    const tf2::Vector3 displacement =
      candidate_map_base.getOrigin() - reference_map_base.getOrigin();
    metrics->translation = std::hypot(displacement.x(), displacement.y());
    metrics->yaw_delta = std::abs(
      normalize_angle(yaw_of(candidate_map_base) - yaw_of(reference_map_base)));
    metrics->linear_limit = max_map_base_step_translation_;
    metrics->angular_limit = max_map_base_step_yaw_;

    const tf2::Transform predicted_map_base = reference_map_odom * current_odom_base;
    const tf2::Vector3 correction =
      candidate_map_base.getOrigin() - predicted_map_base.getOrigin();
    metrics->correction_translation = std::hypot(correction.x(), correction.y());
    metrics->correction_yaw = std::abs(
      normalize_angle(yaw_of(candidate_map_base) - yaw_of(predicted_map_base)));

    // Do not reject displacement from the previous AMCL map pose itself: that
    // includes legitimate odom motion between AMCL updates and previously made
    // a sufficiently long/sparse update look like a localization jump.  The
    // odom-compensated prediction residual below is the actual map->odom
    // correction, while monitor_odom_integrity() separately validates motion.
    if (metrics->correction_translation > max_odom_correction_ ||
      metrics->correction_yaw > max_odom_yaw_correction_)
    {
      *reason = "spatial_step_or_correction_limit";
      return false;
    }
    return true;
  }

  bool candidate_is_within_anchor_locked(
    const tf2::Transform & candidate_map_odom,
    const bool recovery_limits,
    CandidateMetrics * metrics,
    std::string * reason) const
  {
    if (!have_authorized_anchor_) {
      *reason = "missing_authorized_anchor";
      return false;
    }

    const tf2::Transform delta = authorized_anchor_map_odom_.inverse() * candidate_map_odom;
    metrics->cumulative_translation =
      std::hypot(delta.getOrigin().x(), delta.getOrigin().y());
    metrics->cumulative_yaw = std::abs(normalize_angle(yaw_of(delta)));
    const double translation_limit = recovery_limits ?
      recovery_cumulative_map_odom_translation_ : max_cumulative_map_odom_translation_;
    const double yaw_limit = recovery_limits ?
      recovery_cumulative_map_odom_yaw_ : max_cumulative_map_odom_yaw_;
    if (metrics->cumulative_translation > translation_limit ||
      metrics->cumulative_yaw > yaw_limit)
    {
      *reason = "cumulative_map_odom_drift_limit";
      return false;
    }
    return true;
  }

  bool candidate_matches_requested_rebase(
    const tf2::Transform & candidate_map_base,
    std::string * reason) const
  {
    const tf2::Vector3 displacement =
      candidate_map_base.getOrigin() - requested_rebase_map_base_.getOrigin();
    const double translation = std::hypot(displacement.x(), displacement.y());
    const double yaw_delta = std::abs(
      normalize_angle(yaw_of(candidate_map_base) - yaw_of(requested_rebase_map_base_)));
    if (translation > initialpose_rebase_position_tolerance_ ||
      yaw_delta > initialpose_rebase_yaw_tolerance_)
    {
      *reason = "candidate_does_not_match_initialpose";
      return false;
    }
    return true;
  }

  int mark_new_amcl_stamp(const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp.nanoseconds() == 0) {
      return -1;
    }
    if (have_seen_amcl_stamp_ && stamp == last_seen_amcl_stamp_) {
      return 0;
    }
    if (have_seen_amcl_stamp_ && stamp < last_seen_amcl_stamp_) {
      return -1;
    }
    last_seen_amcl_stamp_ = stamp;
    have_seen_amcl_stamp_ = true;
    if (waiting_for_nomotion_pose_ && stamp > nomotion_request_stamp_floor_) {
      waiting_for_nomotion_pose_ = false;
    }
    return 1;
  }

  void reset_cumulative_overrun_defer_locked()
  {
    cumulative_overrun_defer_active_ = false;
    cumulative_overrun_defer_samples_ = 0;
    cumulative_overrun_defer_last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    cumulative_overrun_defer_peak_translation_ = 0.0;
    cumulative_overrun_defer_peak_yaw_ = 0.0;
    cumulative_overrun_defer_peak_correction_translation_ = 0.0;
    cumulative_overrun_defer_peak_correction_yaw_ = 0.0;
  }

  void reset_pending_locked()
  {
    have_pending_candidate_ = false;
    pending_valid_samples_ = 0;
    have_recovery_cluster_ = false;
    recovery_cluster_valid_samples_ = 0;
    have_recovery_cluster_started_ = false;
    reset_cumulative_overrun_defer_locked();
  }

  void reset_recovery_nomotion_budget_locked()
  {
    recovery_nomotion_requests_sent_ = 0;
    have_recovery_nomotion_window_ = false;
    recovery_cluster_reported_ = false;
  }

  void reset_trusted_override_confirmation_locked()
  {
    trusted_override_confirmation_active_ = false;
    trusted_override_confirmation_samples_ = 0;
    trusted_override_confirmation_last_scan_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void reset_trusted_override_return_locked()
  {
    trusted_override_return_active_ = false;
    trusted_override_return_samples_ = 0;
    trusted_override_return_last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void reset_trusted_override_failures_locked()
  {
    trusted_override_soft_failure_samples_ = 0;
    trusted_override_last_failure_scan_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    trusted_override_failure_evidence_ = TrustedOverrideFailureEvidence::kNone;
  }

  void note_trusted_override_soft_failure_locked(
    const rclcpp::Time & scan_stamp,
    const TrustedOverrideFailureEvidence evidence)
  {
    // AMCL may publish faster than the lidar. Reusing one scan cannot turn one
    // structural or emergency-drift observation into multiple confirmations.
    if (trusted_override_failure_evidence_ != evidence) {
      trusted_override_soft_failure_samples_ = 0;
      trusted_override_last_failure_scan_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      trusted_override_failure_evidence_ = evidence;
    }
    if (scan_stamp.nanoseconds() != 0 &&
      (trusted_override_last_failure_scan_stamp_.nanoseconds() == 0 ||
      scan_stamp > trusted_override_last_failure_scan_stamp_))
    {
      trusted_override_last_failure_scan_stamp_ = scan_stamp;
      ++trusted_override_soft_failure_samples_;
    }
  }

  void clear_trusted_override_locked()
  {
    trusted_override_active_ = false;
    trusted_override_soft_hold_ = false;
    trusted_override_hard_hold_ = false;
    trusted_override_verify_epoch_active_ = false;
    reset_trusted_override_failures_locked();
    reset_trusted_override_confirmation_locked();
    reset_trusted_override_return_locked();
  }

  bool trusted_override_recovery_is_stationary_locked(
    const std::chrono::steady_clock::time_point & now) const
  {
    if (!have_odom_baseline_) {
      return false;
    }
    const auto stationary_reference =
      have_odom_motion_ ? last_odom_motion_ : odom_baseline_started_;
    return std::chrono::duration<double>(now - stationary_reference).count() >=
           trusted_override_hold_stationary_duration_sec_;
  }

  TrustedOverrideQuality trusted_override_scan_quality_locked(
    const ScanMatchMetrics & quality,
    std::string * reason) const
  {
    const bool scan_temporally_valid = quality.scan_available &&
      quality.scan_stamp_ns != 0 &&
      std::isfinite(quality.scan_time_delta_sec) &&
      quality.scan_time_delta_sec <= active_quality_scan_tolerance_sec_ &&
      (quality.scan_time_delta_sec <= 1.0e-6 || quality.scan_motion_compensated);
    if (!scan_temporally_valid) {
      // /scan and /amcl_pose are independent DDS paths.  One missing/late scan
      // is not a physical catastrophe; use the configured distinct-scan failure
      // window (or let the dedicated TF/input watchdog assert its hard deadline).
      // This avoids relocking on harmless delivery reordering.
      *reason = "trusted_scan_or_same_stamp_tf_not_ready";
      return TrustedOverrideQuality::kSoftFailure;
    }
    if (!std::isfinite(quality.inlier_ratio) ||
      !std::isfinite(quality.likelihood_mean) ||
      !std::isfinite(quality.trimmed_mean_distance) ||
      !std::isfinite(quality.unknown_offmap_ratio))
    {
      *reason = "trusted_scan_non_finite_metrics";
      return TrustedOverrideQuality::kCatastrophic;
    }
    if (!quality.base_in_free_space) {
      *reason = "trusted_base_not_in_known_free_space";
      return TrustedOverrideQuality::kCatastrophic;
    }
    if (quality.considered_beams <
      static_cast<std::size_t>(trusted_override_catastrophic_min_valid_beams_) ||
      quality.inlier_ratio < trusted_override_catastrophic_min_inlier_ratio_ ||
      quality.likelihood_mean < trusted_override_catastrophic_min_likelihood_ ||
      quality.trimmed_mean_distance >
      trusted_override_catastrophic_max_trimmed_mean_distance_ ||
      quality.unknown_offmap_ratio >
      trusted_override_catastrophic_max_unknown_offmap_ratio_)
    {
      // A finite numeric quality excursion can be transient while the robot is
      // moving. It remains advisory unless an independent raw map->odom
      // emergency-drift condition is also present and persistently confirmed.
      *reason = "trusted_scan_catastrophic_mismatch";
      return TrustedOverrideQuality::kSoftFailure;
    }
    if (quality.considered_beams <
      static_cast<std::size_t>(trusted_override_min_valid_beams_) ||
      quality.inlier_ratio < trusted_override_min_inlier_ratio_ ||
      quality.likelihood_mean < trusted_override_min_likelihood_ ||
      quality.trimmed_mean_distance > trusted_override_max_trimmed_mean_distance_ ||
      quality.unknown_offmap_ratio > trusted_override_max_unknown_offmap_ratio_)
    {
      *reason = "trusted_scan_below_override_floor";
      return TrustedOverrideQuality::kSoftFailure;
    }
    reason->clear();
    return TrustedOverrideQuality::kGood;
  }

  bool advance_trusted_override_confirmation_locked(
    const rclcpp::Time & scan_stamp,
    const std::chrono::steady_clock::time_point & now,
    const int required_samples,
    const double required_duration_sec,
    int * samples,
    double * duration_sec)
  {
    if (!trusted_override_confirmation_active_) {
      trusted_override_confirmation_active_ = true;
      trusted_override_confirmation_started_ = now;
      trusted_override_confirmation_samples_ = 0;
      trusted_override_confirmation_last_scan_stamp_ =
        rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    if (trusted_override_confirmation_last_scan_stamp_.nanoseconds() == 0 ||
      scan_stamp > trusted_override_confirmation_last_scan_stamp_)
    {
      trusted_override_confirmation_last_scan_stamp_ = scan_stamp;
      ++trusted_override_confirmation_samples_;
    }
    *samples = trusted_override_confirmation_samples_;
    *duration_sec = std::chrono::duration<double>(
      now - trusted_override_confirmation_started_).count();
    return trusted_override_confirmation_samples_ >= required_samples &&
           *duration_sec >= required_duration_sec;
  }

  bool advance_trusted_override_return_locked(
    const rclcpp::Time & stamp,
    const std::chrono::steady_clock::time_point & now,
    int * samples,
    double * duration_sec)
  {
    if (!trusted_override_return_active_) {
      trusted_override_return_active_ = true;
      trusted_override_return_started_ = now;
      trusted_override_return_samples_ = 0;
      trusted_override_return_last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    if (trusted_override_return_last_stamp_.nanoseconds() == 0 ||
      stamp > trusted_override_return_last_stamp_)
    {
      trusted_override_return_last_stamp_ = stamp;
      ++trusted_override_return_samples_;
    }
    *samples = trusted_override_return_samples_;
    *duration_sec = std::chrono::duration<double>(
      now - trusted_override_return_started_).count();
    return trusted_override_return_samples_ >= trusted_override_return_consecutive_samples_ &&
           *duration_sec >= trusted_override_return_min_duration_sec_;
  }

  bool cumulative_overrun_scan_quality_ok_locked(
    const ScanMatchMetrics & quality,
    std::string * reason) const
  {
    const bool scan_temporally_valid = quality.scan_available &&
      quality.scan_time_delta_sec <= active_quality_scan_tolerance_sec_ &&
      (quality.scan_time_delta_sec <= 1.0e-6 || quality.scan_motion_compensated);
    if (!quality.covariance_valid) {
      *reason = "defer_covariance_structurally_invalid";
      return false;
    }
    if (!scan_temporally_valid) {
      *reason = "defer_scan_not_same_stamp_or_odom_compensated";
      return false;
    }
    if (!quality.base_in_free_space) {
      *reason = "defer_base_not_in_known_free_space";
      return false;
    }
    if (quality.considered_beams < static_cast<std::size_t>(score_min_valid_beams_)) {
      *reason = "defer_too_few_valid_scan_beams";
      return false;
    }
    if (quality.inlier_ratio < score_min_inlier_ratio_ ||
      quality.likelihood_mean < score_min_likelihood_ ||
      quality.best_neighbor_likelihood_gain > score_max_neighbor_likelihood_gain_ ||
      quality.trimmed_mean_distance > score_max_trimmed_mean_distance_ ||
      quality.unknown_offmap_ratio > score_max_unknown_offmap_ratio_)
    {
      *reason = "defer_scan_map_quality_failed";
      return false;
    }
    return true;
  }

  bool cumulative_overrun_defer_eligible_locked(
    const CandidateMetrics & metrics,
    const ScanMatchMetrics & quality,
    const rclcpp::Time & pose_stamp,
    const rclcpp::Time & odom_stamp,
    const std::chrono::steady_clock::time_point & now,
    std::string * reason) const
  {
    if (!cumulative_overrun_defer_enabled_) {
      *reason = "cumulative_overrun_defer_disabled";
      return false;
    }
    if (anomaly_active_ || amcl_timing_hold_ || odom_timing_hold_ ||
      odom_soft_timing_fault_samples_ > 0 || timer_health_hold_ ||
      auto_reanchor_integrity_inhibit_ || explicit_rebase_required_ || rebase_requested_)
    {
      *reason = "cumulative_overrun_integrity_hold";
      return false;
    }
    if (!have_new_odom_observation_) {
      *reason = "cumulative_overrun_no_fresh_odom";
      return false;
    }
    const double odom_receipt_age_sec =
      std::chrono::duration<double>(now - last_new_odom_observation_steady_).count();
    if (cumulative_overrun_defer_max_odom_receipt_age_sec_ > 0.0 &&
      odom_receipt_age_sec > cumulative_overrun_defer_max_odom_receipt_age_sec_)
    {
      *reason = "cumulative_overrun_stale_odom_receipt";
      return false;
    }
    if (std::abs((odom_stamp - pose_stamp).seconds()) >
      cumulative_overrun_defer_pose_odom_stamp_tolerance_sec_)
    {
      *reason = "cumulative_overrun_pose_odom_stamp_mismatch";
      return false;
    }
    if (metrics.dt <= 0.0 ||
      (cumulative_overrun_defer_max_source_gap_sec_ > 0.0 &&
      metrics.dt > cumulative_overrun_defer_max_source_gap_sec_))
    {
      *reason = "cumulative_overrun_source_gap_out_of_bounds";
      return false;
    }
    if (metrics.correction_translation >
      cumulative_overrun_defer_max_correction_translation_ ||
      metrics.correction_yaw > cumulative_overrun_defer_max_correction_yaw_)
    {
      *reason = "cumulative_overrun_correction_too_large";
      return false;
    }
    if (metrics.cumulative_translation >
      max_cumulative_map_odom_translation_ +
      cumulative_overrun_defer_max_translation_excess_ ||
      metrics.cumulative_yaw >
      max_cumulative_map_odom_yaw_ + cumulative_overrun_defer_max_yaw_excess_)
    {
      *reason = "cumulative_overrun_beyond_defer_ceiling";
      return false;
    }
    if (rolling_anchor_session_translation_used_ >=
      rolling_anchor_session_translation_budget_ ||
      rolling_anchor_session_yaw_used_ >= rolling_anchor_session_yaw_budget_)
    {
      *reason = "cumulative_overrun_session_fuse_exhausted";
      return false;
    }
    return cumulative_overrun_scan_quality_ok_locked(quality, reason);
  }

  bool advance_cumulative_overrun_defer_locked(
    const CandidateMetrics & metrics,
    const rclcpp::Time & stamp,
    const std::chrono::steady_clock::time_point & now,
    int * samples,
    double * duration_sec)
  {
    if (!cumulative_overrun_defer_active_) {
      cumulative_overrun_defer_active_ = true;
      cumulative_overrun_defer_started_ = now;
      cumulative_overrun_defer_samples_ = 0;
      cumulative_overrun_defer_last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    if (cumulative_overrun_defer_last_stamp_.nanoseconds() != 0 &&
      stamp <= cumulative_overrun_defer_last_stamp_)
    {
      *samples = cumulative_overrun_defer_samples_;
      *duration_sec = std::chrono::duration<double>(
        now - cumulative_overrun_defer_started_).count();
      return true;
    }
    cumulative_overrun_defer_last_stamp_ = stamp;
    ++cumulative_overrun_defer_samples_;
    cumulative_overrun_defer_peak_translation_ = std::max(
      cumulative_overrun_defer_peak_translation_, metrics.cumulative_translation);
    cumulative_overrun_defer_peak_yaw_ = std::max(
      cumulative_overrun_defer_peak_yaw_, metrics.cumulative_yaw);
    cumulative_overrun_defer_peak_correction_translation_ = std::max(
      cumulative_overrun_defer_peak_correction_translation_, metrics.correction_translation);
    cumulative_overrun_defer_peak_correction_yaw_ = std::max(
      cumulative_overrun_defer_peak_correction_yaw_, metrics.correction_yaw);
    *samples = cumulative_overrun_defer_samples_;
    *duration_sec = std::chrono::duration<double>(
      now - cumulative_overrun_defer_started_).count();
    return cumulative_overrun_defer_samples_ <
           cumulative_overrun_defer_consecutive_samples_ ||
           *duration_sec < cumulative_overrun_defer_min_duration_sec_;
  }

  void monitor_cumulative_overrun_defer(
    const std::chrono::steady_clock::time_point & steady_now)
  {
    bool expired = false;
    int samples = 0;
    double elapsed_sec = 0.0;
    double peak_translation = 0.0;
    double peak_yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cumulative_overrun_defer_active_) {
        elapsed_sec = std::chrono::duration<double>(
          steady_now - cumulative_overrun_defer_started_).count();
        if (elapsed_sec >= cumulative_overrun_defer_max_duration_sec_) {
          expired = true;
          samples = cumulative_overrun_defer_samples_;
          peak_translation = cumulative_overrun_defer_peak_translation_;
          peak_yaw = cumulative_overrun_defer_peak_yaw_;
          anomaly_active_ = true;
          reset_pending_locked();
        }
      }
    }
    if (expired) {
      publish_current_anomaly();
      RCLCPP_ERROR(
        get_logger(),
        "Cumulative map->odom overrun confirmed by %.3f s steady-clock deadline "
        "(%d distinct AMCL source stamps, peak %.3f m / %.3f rad). Entering HOLD; "
        "the last trusted TF remains frozen.",
        elapsed_sec, samples, peak_translation, peak_yaw);
    }
  }

  void monitor_trusted_override_verification(
    const std::chrono::steady_clock::time_point & steady_now)
  {
    bool entered_frozen_override = false;
    double elapsed_sec = 0.0;
    double confirmation_elapsed_sec = 0.0;
    std::uint64_t verify_epoch = 0;
    int confirmation_samples = 0;
    int soft_failure_samples = 0;
    std::int64_t confirmation_scan_stamp_ns = 0;
    std::int64_t failure_scan_stamp_ns = 0;
    const char * previous_counter_reset_reason = "none";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (trusted_override_verify_epoch_active_ && !trusted_override_active_ &&
        !trusted_override_soft_hold_ && !anomaly_active_)
      {
        elapsed_sec = std::chrono::duration<double>(
          steady_now - trusted_override_verify_epoch_started_).count();
        if (elapsed_sec >= trusted_override_verify_max_duration_sec_) {
          verify_epoch = trusted_override_verify_epoch_id_;
          confirmation_samples = trusted_override_confirmation_samples_;
          soft_failure_samples = trusted_override_soft_failure_samples_;
          confirmation_scan_stamp_ns =
            trusted_override_confirmation_last_scan_stamp_.nanoseconds();
          failure_scan_stamp_ns = trusted_override_last_failure_scan_stamp_.nanoseconds();
          previous_counter_reset_reason = trusted_override_last_counter_reset_reason_;
          if (trusted_override_confirmation_active_) {
            confirmation_elapsed_sec = std::chrono::duration<double>(
              steady_now - trusted_override_confirmation_started_).count();
          }
          trusted_override_verify_epoch_active_ = false;
          trusted_override_active_ = true;
          trusted_override_soft_hold_ = false;
          trusted_override_hard_hold_ = false;
          anomaly_active_ = false;
          reset_trusted_override_confirmation_locked();
          reset_trusted_override_return_locked();
          reset_pending_locked();
          entered_frozen_override = true;
        }
      }
    }
    if (entered_frozen_override) {
      publish_current_anomaly();
      RCLCPP_WARN(
        get_logger(),
        "TRUSTED_OVERRIDE_DIAG branch=VERIFY_TIMEOUT internal_recovery_state=TRUSTED_OVERRIDE "
        "effective_mode=TRUSTED_OVERRIDE anomaly=false "
        "reason=verify_deadline_froze_last_trusted verify_epoch=%llu "
        "verify_elapsed_sec=%.3f verify_deadline_sec=%.3f confirmations=%d "
        "confirmation_elapsed_sec=%.3f confirmation_scan_stamp_ns=%lld "
        "soft_failures=%d/%d failure_scan_stamp_ns=%lld "
        "transition_reason=verify_timeout_froze_last_trusted "
        "last_counter_reset_reason=%s.",
        static_cast<unsigned long long>(verify_epoch), elapsed_sec,
        trusted_override_verify_max_duration_sec_, confirmation_samples,
        confirmation_elapsed_sec, static_cast<long long>(confirmation_scan_stamp_ns),
        soft_failure_samples, trusted_override_soft_failure_consecutive_samples_,
        static_cast<long long>(failure_scan_stamp_ns),
        previous_counter_reset_reason);
      RCLCPP_WARN(
        get_logger(),
        "TRUSTED_OVERRIDE verification did not collect enough good scan-map evidence in "
        "%.3f/%.3f s. Keeping the last trusted TF frozen without asserting anomaly; "
        "numeric scan-map quality alone is advisory, while hard integrity monitors remain active.",
        elapsed_sec, trusted_override_verify_max_duration_sec_);
    }
  }

  void monitor_trusted_override_active_evidence(
    const std::chrono::steady_clock::time_point & steady_now)
  {
    bool expired = false;
    double evidence_age_sec = std::numeric_limits<double>::infinity();
    std::int64_t last_scan_stamp_ns = 0;
    std::uint64_t verify_epoch = 0;
    const char * authority_state = "inactive";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (trusted_override_active_ || trusted_override_verify_epoch_active_) {
        authority_state = trusted_override_active_ ? "TRUSTED_OVERRIDE" : "VERIFY_TRUSTED";
        verify_epoch = trusted_override_verify_epoch_id_;
        if (have_new_scan_observation_) {
          evidence_age_sec = std::chrono::duration<double>(
            steady_now - last_new_scan_observation_steady_).count();
          last_scan_stamp_ns = last_new_scan_stamp_.nanoseconds();
        }
        if (!have_new_scan_observation_ ||
          evidence_age_sec >= trusted_override_active_evidence_timeout_sec_)
        {
          expired = true;
          anomaly_active_ = true;
          clear_trusted_override_locked();
          trusted_override_hard_hold_ = true;
          reset_pending_locked();
        }
      }
    }
    if (expired) {
      publish_current_anomaly();
      RCLCPP_ERROR(
        get_logger(),
        "TRUSTED_OVERRIDE_DIAG branch=HARD_HOLD reason=active_evidence_timeout "
        "effective_mode=HARD_HOLD anomaly=true verify_epoch=%llu "
        "authority_state=%s "
        "evidence_kind=source_monotonic_scan_input evidence_age_steady_sec=%.3f "
        "evidence_deadline_steady_sec=%.3f last_scan_source_stamp_ns=%lld.",
        static_cast<unsigned long long>(verify_epoch), authority_state, evidence_age_sec,
        trusted_override_active_evidence_timeout_sec_,
        static_cast<long long>(last_scan_stamp_ns));
      RCLCPP_ERROR(
        get_logger(),
        "TRUSTED_OVERRIDE entered HARD_HOLD from %s: no strictly newer scan input for %.3f s "
        "(deadline %.3f s, last_scan_stamp_ns=%lld). Cached, duplicate, or out-of-order "
        "scans do not refresh input liveness.",
        authority_state, evidence_age_sec, trusted_override_active_evidence_timeout_sec_,
        static_cast<long long>(last_scan_stamp_ns));
    }
  }

  void set_auto_reanchor_integrity_inhibit_locked(
    const std::string & reason,
    const rclcpp::Time & source_stamp)
  {
    // Timing/TF/odom integrity faults always dominate TRUSTED_OVERRIDE.  They
    // cannot be healed by scan matching alone.
    clear_trusted_override_locked();
    if (auto_reanchor_integrity_inhibit_) {
      return;
    }
    auto_reanchor_integrity_inhibit_ = true;
    auto_reanchor_integrity_inhibit_reason_ = reason;
    auto_reanchor_integrity_inhibit_source_stamp_ = source_stamp;
    auto_reanchor_integrity_inhibit_started_ = std::chrono::steady_clock::now();
    have_auto_reanchor_integrity_inhibit_started_ = true;
    RCLCPP_ERROR(
      get_logger(),
      "Automatic far re-anchor integrity inhibit asserted (first_reason=%s, "
      "source_stamp_ns=%lld). The latch cannot be cleared by a far candidate.",
      reason.c_str(), static_cast<long long>(source_stamp.nanoseconds()));
  }

  void clear_auto_reanchor_integrity_inhibit_locked(const std::string & clear_reason)
  {
    if (!auto_reanchor_integrity_inhibit_) {
      return;
    }
    const double held_sec = have_auto_reanchor_integrity_inhibit_started_ ?
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - auto_reanchor_integrity_inhibit_started_).count() : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "Automatic far re-anchor integrity inhibit cleared by %s after %.3f s "
      "(first_reason=%s, first_source_stamp_ns=%lld).",
      clear_reason.c_str(), held_sec, auto_reanchor_integrity_inhibit_reason_.c_str(),
      static_cast<long long>(auto_reanchor_integrity_inhibit_source_stamp_.nanoseconds()));
    auto_reanchor_integrity_inhibit_ = false;
    auto_reanchor_integrity_inhibit_reason_.clear();
    auto_reanchor_integrity_inhibit_source_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    have_auto_reanchor_integrity_inhibit_started_ = false;
  }

  bool recovery_is_stationary_locked(const std::chrono::steady_clock::time_point & now) const
  {
    if (!have_odom_baseline_) {
      return false;
    }
    const auto stationary_reference =
      have_odom_motion_ ? last_odom_motion_ : odom_baseline_started_;
    return std::chrono::duration<double>(now - stationary_reference).count() >=
           recovery_stationary_duration_sec_;
  }

  bool interlock_is_freshly_blocked_locked(
    const std::chrono::steady_clock::time_point & now) const
  {
    return have_interlock_blocked_receipt_ && interlock_blocked_ &&
           std::chrono::duration<double>(now - last_interlock_blocked_receipt_).count() <=
           interlock_blocked_max_age_sec_;
  }

  void reset_rolling_anchor_budget_locked(const bool reset_session_budget)
  {
    rolling_anchor_adjustments_.clear();
    rolling_anchor_odom_motion_.clear();
    if (reset_session_budget) {
      rolling_anchor_session_translation_used_ = 0.0;
      rolling_anchor_session_yaw_used_ = 0.0;
    }
  }

  void prune_rolling_anchor_window_locked(
    const std::chrono::steady_clock::time_point & now)
  {
    while (!rolling_anchor_adjustments_.empty() &&
      std::chrono::duration<double>(
        now - rolling_anchor_adjustments_.front().receipt).count() >
      rolling_anchor_window_sec_)
    {
      rolling_anchor_adjustments_.pop_front();
    }
    while (!rolling_anchor_odom_motion_.empty() &&
      std::chrono::duration<double>(
        now - rolling_anchor_odom_motion_.front().receipt).count() >
      rolling_anchor_window_sec_)
    {
      rolling_anchor_odom_motion_.pop_front();
    }
  }

  bool maybe_advance_rolling_anchor_locked(
    const tf2::Transform & candidate_map_odom,
    const rclcpp::Time & stamp,
    const std::chrono::steady_clock::time_point & now,
    const double step_scale,
    const bool require_candidate_inside_limits,
    std::string * skip_reason)
  {
    const auto skip = [skip_reason](const std::string & reason) {
        if (skip_reason) {
          *skip_reason = reason;
        }
        return false;
      };
    if (!rolling_anchor_enabled_) {
      return skip("disabled");
    }
    if (!have_authorized_anchor_) {
      return skip("no_authorized_anchor");
    }
    if (post_reanchor_verification_active_) {
      return skip("post_reanchor_verification_active");
    }
    if (auto_reanchor_integrity_inhibit_) {
      return skip("integrity_inhibit_active");
    }
    if (anomaly_active_) {
      return skip("anomaly_active");
    }
    if (amcl_timing_hold_) {
      return skip("amcl_timing_hold");
    }
    if (odom_timing_hold_) {
      return skip("odom_timing_hold");
    }
    if (odom_soft_timing_fault_samples_ > 0) {
      return skip("odom_timing_debounce");
    }
    if (timer_health_hold_) {
      return skip("publish_timer_hold");
    }
    if (explicit_rebase_required_) {
      return skip("explicit_rebase_required");
    }
    if (rebase_requested_) {
      return skip("manual_rebase_pending");
    }

    prune_rolling_anchor_window_locked(now);
    double window_translation_net_x = 0.0;
    double window_translation_net_y = 0.0;
    double window_yaw_net = 0.0;
    for (const auto & adjustment : rolling_anchor_adjustments_) {
      window_translation_net_x += adjustment.translation_x;
      window_translation_net_y += adjustment.translation_y;
      window_yaw_net += adjustment.yaw;
    }
    double odom_window_translation = 0.0;
    double odom_window_yaw = 0.0;
    for (const auto & motion : rolling_anchor_odom_motion_) {
      odom_window_translation += motion.translation;
      odom_window_yaw += motion.yaw;
    }
    const double effective_window_translation_budget =
      rolling_anchor_window_translation_budget_ + std::min(
      rolling_anchor_window_translation_motion_cap_,
      rolling_anchor_translation_budget_per_odom_meter_ * odom_window_translation);
    const double effective_window_yaw_budget = rolling_anchor_window_yaw_budget_ + std::min(
      rolling_anchor_window_yaw_motion_cap_,
      rolling_anchor_yaw_budget_per_odom_rad_ * odom_window_yaw);
    const tf2::Transform delta = authorized_anchor_map_odom_.inverse() * candidate_map_odom;
    const double drift_translation = std::hypot(delta.getOrigin().x(), delta.getOrigin().y());
    const double drift_yaw_signed = normalize_angle(yaw_of(delta));
    const double drift_yaw = std::abs(drift_yaw_signed);
    const double desired_translation = std::max(
      0.0, drift_translation - rolling_anchor_translation_deadband_);
    const double desired_yaw = std::max(0.0, drift_yaw - rolling_anchor_yaw_deadband_);
    if (desired_translation <= 0.0 && desired_yaw <= 0.0) {
      return skip("inside_deadband");
    }
    const double session_translation_remaining = std::max(
      0.0,
      rolling_anchor_session_translation_budget_ - rolling_anchor_session_translation_used_);
    const double session_yaw_remaining = std::max(
      0.0, rolling_anchor_session_yaw_budget_ - rolling_anchor_session_yaw_used_);
    const double bounded_step_scale = std::max(1.0, step_scale);
    double translation_direction_x = 0.0;
    double translation_direction_y = 0.0;
    if (drift_translation > 0.0) {
      const tf2::Vector3 local_direction(
        delta.getOrigin().x() / drift_translation,
        delta.getOrigin().y() / drift_translation, 0.0);
      const tf2::Vector3 map_direction =
        tf2::quatRotate(authorized_anchor_map_odom_.getRotation(), local_direction);
      translation_direction_x = map_direction.x();
      translation_direction_y = map_direction.y();
    }
    const double requested_translation_step = std::min(
      desired_translation, rolling_anchor_max_step_translation_ * bounded_step_scale);
    const double directional_translation_limit = directional_vector_step_limit(
      window_translation_net_x, window_translation_net_y,
      translation_direction_x, translation_direction_y,
      effective_window_translation_budget, requested_translation_step);
    const double requested_yaw_step = std::min(
      desired_yaw, rolling_anchor_max_step_yaw_ * bounded_step_scale);
    const double yaw_direction = drift_yaw_signed < 0.0 ? -1.0 : 1.0;
    const double directional_yaw_limit = directional_vector_step_limit(
      window_yaw_net, 0.0, yaw_direction, 0.0,
      effective_window_yaw_budget, requested_yaw_step);
    const double translation_step = std::min(
      directional_translation_limit, session_translation_remaining);
    const double yaw_step = std::min(directional_yaw_limit, session_yaw_remaining);
    if (translation_step <= 0.0 && yaw_step <= 0.0) {
      const bool window_blocks_all =
        (desired_translation <= 0.0 || directional_translation_limit <= 0.0) &&
        (desired_yaw <= 0.0 || directional_yaw_limit <= 0.0);
      const bool session_blocks_all =
        (desired_translation <= 0.0 || session_translation_remaining <= 0.0) &&
        (desired_yaw <= 0.0 || session_yaw_remaining <= 0.0);
      if (window_blocks_all) {
        return skip("sliding_window_budget_exhausted");
      }
      if (session_blocks_all) {
        return skip("session_budget_exhausted");
      }
      return skip("configured_step_limit_zero");
    }

    tf2::Transform adjustment;
    adjustment.setIdentity();
    if (translation_step > 0.0 && drift_translation > 0.0) {
      const double scale = translation_step / drift_translation;
      adjustment.setOrigin(tf2::Vector3(
          delta.getOrigin().x() * scale, delta.getOrigin().y() * scale, 0.0));
    }
    if (yaw_step > 0.0) {
      tf2::Quaternion rotation;
      rotation.setRPY(0.0, 0.0, std::copysign(yaw_step, drift_yaw_signed));
      adjustment.setRotation(rotation);
    }
    const tf2::Transform previous_anchor = authorized_anchor_map_odom_;
    const tf2::Transform proposed_anchor = previous_anchor * adjustment;
    if (require_candidate_inside_limits) {
      // A catch-up is transactional: never spend anchor budget unless the
      // bounded adjustment is sufficient to bring this already-continuous
      // candidate back inside the unchanged hard cumulative envelope.
      const tf2::Transform residual = proposed_anchor.inverse() * candidate_map_odom;
      const double residual_translation =
        std::hypot(residual.getOrigin().x(), residual.getOrigin().y());
      const double residual_yaw = std::abs(normalize_angle(yaw_of(residual)));
      if (residual_translation > max_cumulative_map_odom_translation_ ||
        residual_yaw > max_cumulative_map_odom_yaw_)
      {
        return skip("catchup_step_insufficient_for_hard_envelope");
      }
    }
    authorized_anchor_map_odom_ = proposed_anchor;
    authorized_anchor_stamp_ = stamp;

    AnchorAdjustment record;
    record.receipt = now;
    record.translation_x =
      proposed_anchor.getOrigin().x() - previous_anchor.getOrigin().x();
    record.translation_y =
      proposed_anchor.getOrigin().y() - previous_anchor.getOrigin().y();
    record.yaw = normalize_angle(yaw_of(proposed_anchor) - yaw_of(previous_anchor));
    rolling_anchor_adjustments_.push_back(record);
    rolling_anchor_session_translation_used_ += translation_step;
    rolling_anchor_session_yaw_used_ += yaw_step;
    ++rolling_anchor_update_count_;
    if (skip_reason) {
      skip_reason->clear();
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Advanced the authorized rolling anchor by %.3f m / %.3f rad after a trusted, "
      "scan-qualified correction (step_scale=%.2f, transactional_catchup=%s). "
      "Sliding-window net use %.3f/%.3f m and %.3f/%.3f rad; validated odom motion "
      "in the window %.3f m / %.3f rad; "
      "session use %.3f/%.3f m and %.3f/%.3f rad.",
      translation_step, yaw_step, bounded_step_scale,
      require_candidate_inside_limits ? "yes" : "no",
      std::hypot(
        window_translation_net_x + record.translation_x,
        window_translation_net_y + record.translation_y),
      effective_window_translation_budget, std::abs(window_yaw_net + record.yaw),
      effective_window_yaw_budget, odom_window_translation, odom_window_yaw,
      rolling_anchor_session_translation_used_,
      rolling_anchor_session_translation_budget_, rolling_anchor_session_yaw_used_,
      rolling_anchor_session_yaw_budget_);
    return true;
  }

  bool auto_reanchor_preconditions_locked(
    const tf2::Transform & candidate_map_odom,
    const std::chrono::steady_clock::time_point & now,
    CandidateMetrics * metrics,
    std::string * reason) const
  {
    if (!auto_reanchor_enabled_ || max_auto_reanchors_per_session_ <= 0) {
      *reason = "auto_reanchor_disabled";
      return false;
    }
    if (auto_reanchors_used_ >= max_auto_reanchors_per_session_) {
      *reason = "auto_reanchor_session_limit_reached";
      return false;
    }
    if (!anomaly_active_ || !have_trusted_transform_ || !have_authorized_anchor_) {
      *reason = "auto_reanchor_requires_trusted_hold";
      return false;
    }
    const tf2::Transform delta = authorized_anchor_map_odom_.inverse() * candidate_map_odom;
    metrics->cumulative_translation =
      std::hypot(delta.getOrigin().x(), delta.getOrigin().y());
    metrics->cumulative_yaw = std::abs(normalize_angle(yaw_of(delta)));
    const bool exceeds_absolute_limit =
      metrics->cumulative_translation > auto_reanchor_max_translation_ ||
      metrics->cumulative_yaw > auto_reanchor_max_yaw_;
    if (explicit_rebase_required_ || rebase_requested_ || amcl_timing_hold_ ||
      odom_timing_hold_ || odom_soft_timing_fault_samples_ > 0 || timer_health_hold_)
    {
      *reason = "auto_reanchor_blocked_by_integrity_hold";
      return false;
    }
    if (auto_reanchor_integrity_inhibit_) {
      *reason = "auto_reanchor_inhibited_after_" + auto_reanchor_integrity_inhibit_reason_;
      if (exceeds_absolute_limit) {
        *reason += "_and_absolute_limit";
      }
      return false;
    }
    if (!interlock_is_freshly_blocked_locked(now)) {
      *reason = "auto_reanchor_waiting_for_fresh_interlock_blocked";
      return false;
    }
    if (!have_new_odom_observation_ ||
      std::chrono::duration<double>(now - last_new_odom_observation_steady_).count() >
      auto_reanchor_max_odom_receipt_age_sec_)
    {
      *reason = "auto_reanchor_waiting_for_fresh_odom";
      return false;
    }
    if (!recovery_is_stationary_locked(now)) {
      *reason = "auto_reanchor_waiting_for_stationary";
      return false;
    }
    if (have_authorized_anchor_steady_ &&
      std::chrono::duration<double>(now - authorized_anchor_steady_).count() <
      auto_reanchor_cooldown_sec_)
    {
      *reason = "auto_reanchor_cooldown";
      return false;
    }

    if (exceeds_absolute_limit) {
      *reason = "auto_reanchor_absolute_limit";
      return false;
    }
    return true;
  }

  bool advance_recovery_cluster_locked(
    const tf2::Transform & candidate_map_odom,
    const tf2::Transform & candidate_map_base,
    const tf2::Transform & current_odom_base,
    const std::chrono::steady_clock::time_point & now,
    const int required_samples,
    const double required_duration_sec)
  {
    if (!have_recovery_cluster_) {
      recovery_cluster_map_odom_ = candidate_map_odom;
      recovery_cluster_map_base_ = candidate_map_base;
      recovery_cluster_odom_base_ = current_odom_base;
      recovery_cluster_valid_samples_ = 1;
      have_recovery_cluster_ = true;
      recovery_cluster_started_ = now;
      have_recovery_cluster_started_ = true;
      return required_samples <= 1 && required_duration_sec <= 0.0;
    }

    const tf2::Vector3 map_delta =
      candidate_map_base.getOrigin() - recovery_cluster_map_base_.getOrigin();
    const double map_translation = std::hypot(map_delta.x(), map_delta.y());
    const double map_yaw = std::abs(
      normalize_angle(yaw_of(candidate_map_base) - yaw_of(recovery_cluster_map_base_)));
    const tf2::Vector3 odom_delta =
      current_odom_base.getOrigin() - recovery_cluster_odom_base_.getOrigin();
    const double odom_translation = std::hypot(odom_delta.x(), odom_delta.y());
    const double odom_yaw = std::abs(
      normalize_angle(yaw_of(current_odom_base) - yaw_of(recovery_cluster_odom_base_)));
    const tf2::Transform map_odom_delta =
      recovery_cluster_map_odom_.inverse() * candidate_map_odom;
    const double map_odom_translation = std::hypot(
      map_odom_delta.getOrigin().x(), map_odom_delta.getOrigin().y());
    const double map_odom_yaw = std::abs(normalize_angle(yaw_of(map_odom_delta)));

    // A far candidate may be a legitimate, explicitly requested re-localization,
    // but it must first form a stable cluster.  Never accept a moving or drifting
    // cluster as a new global pose.
    if (map_translation > recovery_cluster_translation_tolerance_ ||
      map_yaw > recovery_cluster_yaw_tolerance_ ||
      map_odom_translation > recovery_cluster_translation_tolerance_ ||
      map_odom_yaw > recovery_cluster_yaw_tolerance_ ||
      odom_translation > recovery_cluster_odom_translation_tolerance_ ||
      odom_yaw > recovery_cluster_odom_yaw_tolerance_)
    {
      recovery_cluster_map_odom_ = candidate_map_odom;
      recovery_cluster_map_base_ = candidate_map_base;
      recovery_cluster_odom_base_ = current_odom_base;
      recovery_cluster_valid_samples_ = 1;
      recovery_cluster_started_ = now;
      have_recovery_cluster_started_ = true;
      return false;
    }

    // Keep the first sample as an immutable seed.  Updating the reference on
    // every frame would let a stable-looking sequence ratchet arbitrarily far.
    ++recovery_cluster_valid_samples_;
    const double cluster_duration_sec = have_recovery_cluster_started_ ?
      std::chrono::duration<double>(now - recovery_cluster_started_).count() : 0.0;
    return recovery_cluster_valid_samples_ >= required_samples &&
           cluster_duration_sec >= required_duration_sec;
  }

  void store_pending_locked(
    const tf2::Transform & map_odom,
    const tf2::Transform & map_base,
    const rclcpp::Time & stamp)
  {
    pending_map_odom_ = map_odom;
    pending_map_base_ = map_base;
    pending_stamp_ = stamp;
    have_pending_candidate_ = true;
    pending_valid_samples_ = 1;
  }

  bool advance_pending_locked(
    const tf2::Transform & candidate_map_odom,
    const tf2::Transform & candidate_map_base,
    const tf2::Transform & current_odom_base,
    const rclcpp::Time & stamp,
    const int required_samples,
    CandidateMetrics * metrics,
    std::string * reason)
  {
    if (!have_pending_candidate_) {
      store_pending_locked(candidate_map_odom, candidate_map_base, stamp);
      if (required_samples <= 1) {
        return true;
      }
      *reason = "collecting_consistent_candidates";
      return false;
    }

    if (!candidate_is_reasonable(
        pending_map_odom_, pending_map_base_, pending_stamp_, candidate_map_base,
        current_odom_base, stamp, metrics, reason))
    {
      // A new sequence may begin only after the previous sequence is discarded;
      // it still needs all required confirmations before it can influence TF.
      store_pending_locked(candidate_map_odom, candidate_map_base, stamp);
      return false;
    }

    // Keep the first sample as an immutable seed for the whole confirmation
    // sequence.  The newest candidate is committed only after it also passes
    // the fixed-anchor gate.
    ++pending_valid_samples_;
    if (pending_valid_samples_ < required_samples) {
      *reason = "collecting_consistent_candidates";
      return false;
    }
    return true;
  }

  bool advance_bootstrap_locked(
    const tf2::Transform & candidate_map_odom,
    const tf2::Transform & candidate_map_base,
    const rclcpp::Time & stamp,
    const std::chrono::steady_clock::time_point & receipt,
    BootstrapSample * selected,
    std::string * reason)
  {
    if (!bootstrap_samples_.empty() && max_amcl_validation_gap_sec_ > 0.0 &&
      std::chrono::duration<double>(receipt - bootstrap_samples_.back().receipt).count() >
      max_amcl_validation_gap_sec_)
    {
      bootstrap_samples_.clear();
      *reason = "bootstrap_confirmation_gap_exceeded";
    }
    if (!bootstrap_samples_.empty()) {
      const tf2::Transform delta =
        bootstrap_samples_.front().map_odom.inverse() * candidate_map_odom;
      const double translation = std::hypot(delta.getOrigin().x(), delta.getOrigin().y());
      const double yaw = std::abs(normalize_angle(yaw_of(delta)));
      if (translation > bootstrap_seed_max_translation_ ||
        yaw > bootstrap_seed_max_yaw_)
      {
        bootstrap_samples_.clear();
        *reason = "bootstrap_fixed_seed_span_exceeded";
      }
    }

    BootstrapSample sample;
    sample.map_odom = candidate_map_odom;
    sample.map_base = candidate_map_base;
    sample.stamp = stamp;
    sample.receipt = receipt;
    bootstrap_samples_.push_back(sample);
    if (bootstrap_samples_.size() < static_cast<std::size_t>(bootstrap_good_samples_) ||
      std::chrono::duration<double>(receipt - bootstrap_samples_.front().receipt).count() <
      bootstrap_min_confirm_duration_sec_)
    {
      if (*reason != "bootstrap_fixed_seed_span_exceeded") {
        *reason = "collecting_bootstrap_quality_window";
      }
      return false;
    }

    std::size_t best_index = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t left = 0; left < bootstrap_samples_.size(); ++left) {
      double cost = 0.0;
      for (std::size_t right = 0; right < bootstrap_samples_.size(); ++right) {
        const tf2::Transform delta =
          bootstrap_samples_[left].map_odom.inverse() * bootstrap_samples_[right].map_odom;
        cost += std::hypot(delta.getOrigin().x(), delta.getOrigin().y()) +
          0.5 * std::abs(normalize_angle(yaw_of(delta)));
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_index = left;
      }
    }
    *selected = bootstrap_samples_[best_index];
    bootstrap_samples_.clear();
    return true;
  }

  void on_initialpose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!allow_manual_rebase_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring /initialpose because this deployment uses automatic AMCL convergence as the "
        "only anchor-authoring path.");
      return;
    }
    // A manual localization hint is not authority to move map->odom abruptly.
    // It merely opens a separately audited rebase path: multiple following AMCL
    // poses must match this requested map pose and be mutually consistent.
    const auto & pose = message->pose.pose;
    if (message->header.frame_id != global_frame_ ||
      !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w))
    {
      enter_hold("invalid_initialpose", true);
      return;
    }
    const double quaternion_norm =
      std::sqrt(pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
    if (quaternion_norm < 0.5 || quaternion_norm > 1.5) {
      enter_hold("invalid_initialpose_quaternion", true);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      requested_rebase_map_base_ = transform_from_pose(message->pose.pose);
      rebase_requested_ = true;
      anomaly_active_ = true;
      reset_pending_locked();
    }
    publish_anomaly(true);
    RCLCPP_WARN(
      get_logger(),
      "Initial pose received; blind rebase is disabled. Retaining the last trusted %s -> %s "
      "until %d fresh AMCL samples both match the requested pose and pass continuity validation.",
      global_frame_.c_str(), odom_frame_.c_str(), initialpose_rebase_consecutive_valid_samples_);
  }

  void on_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    // Enforce the steady-clock deadline even if the next AMCL callback arrives
    // before the publish timer gets CPU time.
    monitor_cumulative_overrun_defer(steady_now);
    monitor_trusted_override_active_evidence(steady_now);
    monitor_trusted_override_verification(steady_now);
    const rclcpp::Time stamp(message->header.stamp);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_amcl_receipt_ = steady_now;
      last_received_amcl_stamp_ = stamp;
      have_amcl_receipt_ = true;
    }
    if (stamp.nanoseconds() == 0) {
      enter_amcl_timing_hold("zero_amcl_stamp", stamp);
      return;
    }
    bool historical_amcl_pose = false;
    double historical_lag_sec = 0.0;
    std::uint64_t historical_drop_count = 0;
    std::int64_t last_validated_stamp_ns = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_validated_amcl_receipt_ && stamp <= last_validated_amcl_stamp_) {
        historical_amcl_pose = true;
        historical_lag_sec = (last_validated_amcl_stamp_ - stamp).seconds();
        historical_drop_count = ++historical_amcl_drop_count_;
        last_validated_stamp_ns = last_validated_amcl_stamp_.nanoseconds();
      }
    }
    if (historical_amcl_pose) {
      // Reliable DDS queues may deliver one old pose after the current stream
      // has already advanced. It is neither new evidence nor a source-clock
      // failure. Drop it before age/TF/quality processing so it cannot mutate
      // validated/accepted state, the motion reference, HOLD, or the inhibit.
      // A stream containing only historical packets still trips the existing
      // moving-silence watchdog because its validated timestamp never advances.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping historical AMCL pose %.6f s behind the last validated source stamp "
        "(source_stamp_ns=%lld, last_validated_stamp_ns=%lld, drops=%llu).",
        historical_lag_sec, static_cast<long long>(stamp.nanoseconds()),
        static_cast<long long>(last_validated_stamp_ns),
        static_cast<unsigned long long>(historical_drop_count));
      return;
    }
    const double pose_age_sec = (get_clock()->now() - stamp).seconds();
    if (pose_age_sec > max_amcl_pose_age_sec_) {
      enter_amcl_timing_hold("stale_amcl_pose", stamp);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "AMCL pose age %.3f s exceeds %.3f s (source_stamp_ns=%lld); retaining the fixed "
        "trusted transform.",
        pose_age_sec, max_amcl_pose_age_sec_,
        static_cast<long long>(stamp.nanoseconds()));
      return;
    }
    if (pose_age_sec < -max_future_pose_skew_sec_) {
      enter_amcl_timing_hold("future_amcl_pose", stamp);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "AMCL pose is %.3f s in the future (limit %.3f s, source_stamp_ns=%lld); retaining "
        "the fixed trusted transform.",
        -pose_age_sec, max_future_pose_skew_sec_,
        static_cast<long long>(stamp.nanoseconds()));
      return;
    }

    tf2::Transform candidate_map_odom;
    tf2::Transform candidate_map_base;
    tf2::Transform current_odom_base;
    rclcpp::Time current_odom_stamp(0, 0, RCL_ROS_TIME);
    bool used_latest_odom = false;
    std::string reason;
    if (!candidate_from_amcl_pose(
        *message, &candidate_map_odom, &candidate_map_base, &current_odom_base,
        &current_odom_stamp, &used_latest_odom, &reason))
    {
      if (is_spatial_fault_reason(reason)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "AMCL candidate failed spatial validation (%s, source_stamp_ns=%lld).",
          reason.c_str(), static_cast<long long>(stamp.nanoseconds()));
        enter_hold(reason);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "AMCL candidate unavailable (%s, source_stamp_ns=%lld); retaining the last trusted "
          "transform.",
          reason.c_str(), static_cast<long long>(stamp.nanoseconds()));
      }
      return;
    }

    bool have_trusted_before_candidate = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_trusted_before_candidate = have_trusted_transform_;
    }
    if (used_latest_odom && have_trusted_before_candidate) {
      // Do not pair an old AMCL pose with a newer odom pose after bootstrap:
      // retaining the last trusted transform is the safe timing-only action.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "AMCL pose could not be paired at its source timestamp; retaining the last trusted "
        "transform (timing diagnostic only).");
      return;
    }

    const int stamp_classification = mark_new_amcl_stamp(stamp);
    if (stamp_classification == 0) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring duplicate AMCL source stamp; no confirmation counter was advanced.");
      return;
    }
    if (stamp_classification < 0) {
      // Defensive fallback for a source stamp that is older than a previously
      // seen (but not yet validated) message. As above, one old DDS sample is a
      // drop-only diagnostic; freshness/silence monitoring owns stream health.
      {
        std::lock_guard<std::mutex> lock(mutex_);
        historical_drop_count = ++historical_amcl_drop_count_;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping out-of-order AMCL source stamp %lld ns before validation (drops=%llu); "
        "no guard state or confirmation counter was changed.",
        static_cast<long long>(stamp.nanoseconds()),
        static_cast<unsigned long long>(historical_drop_count));
      return;
    }

    ScanMatchMetrics bootstrap_quality;
    bool bootstrap_epoch_ready = false;
    bool bootstrap_stationary = false;
    bool bootstrap_quality_ok = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      bootstrap_stationary = recovery_is_stationary_locked(steady_now);
      if (!have_trusted_transform_ && global_localization_completed_ &&
        stamp > localization_stamp_floor_ && bootstrap_stationary)
      {
        ++bootstrap_post_global_updates_;
        bootstrap_epoch_ready =
          bootstrap_post_global_updates_ >= bootstrap_min_post_global_updates_ &&
          std::chrono::duration<double>(steady_now - localization_epoch_steady_).count() >=
          bootstrap_min_epoch_age_sec_;
      }
    }
    if (!bootstrap_stationary) {
      bootstrap_quality.reason = "bootstrap_waiting_for_stationary";
    } else if (bootstrap_epoch_ready) {
      const auto bootstrap_scan = select_quality_scan(
        stamp, false, &bootstrap_quality);
      bootstrap_quality_ok = evaluate_bootstrap_quality(
        *message, candidate_map_odom, candidate_map_base, bootstrap_scan,
        QualityCovariancePolicy::kStrictConverged,
        &bootstrap_quality);
    } else {
      bootstrap_quality.reason = "bootstrap_waiting_for_post_global_epoch";
    }

    ScanMatchMetrics trusted_quality;
    ScanMatchMetrics trusted_reference_quality;
    ScanMatchMetrics raw_override_quality;
    bool trusted_quality_ok = false;
    bool trusted_reanchor_quality_ok = false;
    std::string trusted_reanchor_quality_reason;
    if (have_trusted_before_candidate) {
      // Normal continuity remains spatially gated.  ACTIVE quality may use the
      // nearest scan inside a small tolerance, but it is never projected from
      // the robot's current pose: odom at the selected scan stamp reconstructs
      // map->base before scoring.
      const auto trusted_scan = select_quality_scan(stamp, true, &trusted_quality);
      trusted_quality_ok = evaluate_bootstrap_quality(
        *message, candidate_map_odom, candidate_map_base, trusted_scan,
        QualityCovariancePolicy::kStructuralOnly,
        &trusted_quality);
      // TRUSTED_OVERRIDE never uses AMCL covariance or a relative comparison
      // to authorize the frozen transform.  This second absolute score is used
      // only to decide whether a now-near raw candidate may return to NORMAL.
      evaluate_bootstrap_quality(
        *message, candidate_map_odom, candidate_map_base, trusted_scan,
        QualityCovariancePolicy::kIgnored,
        &raw_override_quality);
      trusted_reanchor_quality_ok =
        trusted_quality_ok && trusted_quality.covariance_converged;
      trusted_reanchor_quality_reason =
        trusted_quality_ok && !trusted_quality.covariance_converged ?
        "bootstrap_covariance_not_converged" : trusted_quality.reason;
      tf2::Transform trusted_reference_map_odom;
      tf2::Transform trusted_reference_map_base;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        trusted_reference_map_odom = last_trusted_map_odom_;
        trusted_reference_map_base = trusted_reference_map_odom * current_odom_base;
      }
      // A corridor alias can be stable, low-covariance, and locally optimal.
      // Score the frozen trusted transform with the identical scan as an
      // explicit counterfactual; a far re-anchor must materially improve it.
      evaluate_bootstrap_quality(
        *message, trusted_reference_map_odom, trusted_reference_map_base, trusted_scan,
        QualityCovariancePolicy::kIgnored,
        &trusted_reference_quality);
    }

    bool accepted = false;
    bool bootstrap_accepted = false;
    bool recovery_accepted = false;
    bool rebase_accepted = false;
    bool auto_reanchor_started = false;
    bool auto_reanchor_verified = false;
    bool anomaly_was_active = false;
    bool rejected = false;
    bool stable_far_candidate_ready = false;
    bool cumulative_overrun_deferred = false;
    bool cumulative_overrun_cleared = false;
    bool cumulative_overrun_confirmed = false;
    bool trusted_override_handled = false;
    bool trusted_override_verifying = false;
    bool trusted_override_entered = false;
    bool trusted_override_retained = false;
    bool trusted_override_soft_hold_waiting = false;
    bool trusted_override_recovered = false;
    bool trusted_override_returned = false;
    bool trusted_override_hard_failed = false;
    bool trusted_override_verify_cancelled = false;
    bool trusted_override_verify_started = false;
    bool trusted_override_soft_failure_observed = false;
    bool trusted_override_counter_reset_observed = false;
    bool trusted_override_diagnostic_snapshot_valid = false;
    bool trusted_override_scan_disposition_needs_throttle = false;
    bool trusted_override_verify_epoch_was_active = false;
    int confirmation_count = 0;
    int required_confirmations = 0;
    int trusted_override_samples = 0;
    int trusted_override_soft_failures = 0;
    int trusted_override_confirmations_before = 0;
    int trusted_override_confirmations_after = 0;
    int trusted_override_soft_failures_before = 0;
    int trusted_override_soft_failures_after = 0;
    int trusted_override_returns_before = 0;
    int trusted_override_returns_after = 0;
    double cumulative_overrun_defer_duration_sec = 0.0;
    double cumulative_overrun_peak_translation = 0.0;
    double cumulative_overrun_peak_yaw = 0.0;
    double trusted_override_duration_sec = 0.0;
    double trusted_override_verify_elapsed_sec =
      std::numeric_limits<double>::quiet_NaN();
    std::uint64_t trusted_override_verify_epoch = 0;
    CandidateMetrics metrics;
    std::string rolling_anchor_skip_reason;
    std::string trusted_override_reason;
    const char * trusted_override_quality_class = "NOT_EVALUATED";
    const char * trusted_override_quality_reason_code = "not_evaluated";
    const char * trusted_override_scan_disposition = "not_evaluated";
    const char * trusted_override_counter_reset_reason = "none";
    const auto trusted_override_quality_label = [](TrustedOverrideQuality quality) {
        switch (quality) {
          case TrustedOverrideQuality::kGood:
            return "GOOD";
          case TrustedOverrideQuality::kSoftFailure:
            return "SOFT_FAILURE";
          case TrustedOverrideQuality::kCatastrophic:
            return "CATASTROPHIC_FAILURE";
        }
        return "UNKNOWN";
      };
    const auto trusted_override_quality_reason_label = [](const std::string & quality_reason) {
        if (quality_reason.empty()) {
          return "quality_ok";
        }
        if (quality_reason == "trusted_scan_or_same_stamp_tf_not_ready") {
          return "trusted_scan_or_same_stamp_tf_not_ready";
        }
        if (quality_reason == "trusted_scan_non_finite_metrics") {
          return "trusted_scan_non_finite_metrics";
        }
        if (quality_reason == "trusted_base_not_in_known_free_space") {
          return "trusted_base_not_in_known_free_space";
        }
        if (quality_reason == "trusted_scan_catastrophic_mismatch") {
          return "trusted_scan_catastrophic_mismatch";
        }
        if (quality_reason == "trusted_scan_below_override_floor") {
          return "trusted_scan_below_override_floor";
        }
        return "unmapped_quality_reason";
      };

    {
      std::lock_guard<std::mutex> lock(mutex_);
      anomaly_was_active = anomaly_active_;
      trusted_override_verify_epoch_was_active = trusted_override_verify_epoch_active_;
      trusted_override_confirmations_before = trusted_override_confirmation_samples_;
      trusted_override_soft_failures_before = trusted_override_soft_failure_samples_;
      trusted_override_returns_before = trusted_override_return_samples_;
      if (have_validated_amcl_receipt_ && max_amcl_validation_gap_sec_ > 0.0 &&
        std::chrono::duration<double>(steady_now - last_validated_amcl_receipt_).count() >
        max_amcl_validation_gap_sec_)
      {
        // A confirmation sequence must be temporally compact.  Normal ACTIVE
        // operation is not faulted by this gap; moving silence is monitored by
        // the publish timer below.
        reset_pending_locked();
        if (trusted_override_confirmation_samples_ > 0 ||
          trusted_override_return_samples_ > 0 ||
          trusted_override_soft_failure_samples_ > 0)
        {
          trusted_override_counter_reset_observed = true;
          trusted_override_last_counter_reset_reason_ = "amcl_validation_gap";
        }
        reset_trusted_override_confirmation_locked();
        reset_trusted_override_return_locked();
        reset_trusted_override_failures_locked();
      }
      last_validated_amcl_receipt_ = steady_now;
      last_validated_amcl_stamp_ = stamp;
      have_validated_amcl_receipt_ = true;
      amcl_timing_hold_ = false;
      motion_reference_odom_base_ = current_odom_base;
      have_motion_reference_ = true;
      odom_moved_since_last_amcl_ = false;
      have_amcl_motion_threshold_crossing_ = false;
      if (odom_timing_hold_ || odom_soft_timing_fault_samples_ > 0 ||
        timer_health_hold_)
      {
        // A fresh AMCL pose cannot update map->odom while another input or the
        // TF publisher is still unhealthy.  A soft odom timing violation is
        // deliberately not exposed as an anomaly until confirmed, but it
        // still freezes candidate commits during that bounded debounce window.
        // HOLD means bit-for-bit frozen.
        reset_pending_locked();
        rejected = true;
        reason = odom_timing_hold_ ?
          "waiting_for_healthy_odom_before_commit" :
          (odom_soft_timing_fault_samples_ > 0 ?
          "waiting_for_odom_timing_debounce_before_commit" :
          "waiting_for_healthy_publish_timer_before_commit");
      } else if (rebase_requested_) {
        required_confirmations = initialpose_rebase_consecutive_valid_samples_;
        if (!candidate_matches_requested_rebase(candidate_map_base, &reason)) {
          reset_pending_locked();
          anomaly_active_ = true;
          rejected = true;
        } else if (!recovery_is_stationary_locked(steady_now)) {
          // A large, explicitly requested map rebase is only safe while the
          // robot is stationary.  Restart the confirmation sequence after
          // motion has settled instead of moving map->odom underneath it.
          reset_pending_locked();
          anomaly_active_ = true;
          reason = "rebase_waiting_for_stationary";
          rejected = true;
        } else if (advance_recovery_cluster_locked(
            candidate_map_odom, candidate_map_base, current_odom_base,
            steady_now, required_confirmations, 0.0))
        {
          if (accept_candidate(
              candidate_map_odom, candidate_map_base, current_odom_base, stamp,
              current_odom_stamp, true))
          {
            reset_rolling_anchor_budget_locked(true);
            auto_reanchors_used_ = 0;
            clear_auto_reanchor_integrity_inhibit_locked("explicit_rebase_accepted");
            reset_recovery_nomotion_budget_locked();
            rebase_requested_ = false;
            explicit_rebase_required_ = false;
            anomaly_active_ = false;
            accepted = true;
            rebase_accepted = true;
          }
        } else {
          anomaly_active_ = true;
          confirmation_count = recovery_cluster_valid_samples_;
          reason = "collecting_rebase_cluster";
        }
      } else if (explicit_rebase_required_) {
        // An odom reset changes the meaning of the odom frame.  Never clear
        // that fault merely because AMCL later lands near the old anchor.
        anomaly_active_ = true;
        reset_pending_locked();
        rejected = true;
        reason = "odom_jump_waiting_for_explicit_initialpose";
      } else if (!have_trusted_transform_) {
        required_confirmations = bootstrap_good_samples_;
        if (bootstrap_requires_initialpose_) {
          anomaly_active_ = true;
          rejected = true;
          reason = "bootstrap_waiting_for_initialpose";
          reset_pending_locked();
        } else if (auto_global_localization_ && !global_localization_completed_) {
          anomaly_active_ = true;
          rejected = true;
          reason = "bootstrap_waiting_for_global_localization";
          bootstrap_samples_.clear();
        } else if (!bootstrap_epoch_ready || !bootstrap_quality_ok) {
          anomaly_active_ = true;
          rejected = true;
          reason = bootstrap_quality.reason;
          // /scan and /amcl_pose are separate DDS paths.  A pose may reach this
          // callback just before the matching scan under load.  Do not discard
          // an otherwise valid confirmation window for this transient ordering
          // miss; the next good sample still has to satisfy the fixed-seed and
          // maximum-gap rules in advance_bootstrap_locked().
          if (reason != "bootstrap_waiting_same_stamp_scan" &&
            reason != "quality_missing_base_to_scan_tf")
          {
            bootstrap_samples_.clear();
          }
        } else {
          BootstrapSample selected;
          if (advance_bootstrap_locked(
              candidate_map_odom, candidate_map_base, stamp, steady_now,
              &selected, &reason))
          {
            if (accept_candidate(
                selected.map_odom, selected.map_base, current_odom_base,
                selected.stamp, current_odom_stamp, true))
            {
              reset_rolling_anchor_budget_locked(true);
              auto_reanchors_used_ = 0;
              clear_auto_reanchor_integrity_inhibit_locked("bootstrap_accepted");
              reset_recovery_nomotion_budget_locked();
              explicit_rebase_required_ = false;
              force_global_localization_for_epoch_ = false;
              anomaly_active_ = false;
              accepted = true;
              bootstrap_accepted = true;
            }
          } else {
            anomaly_active_ = true;
            confirmation_count = static_cast<int>(bootstrap_samples_.size());
          }
        }
      } else {
        CandidateMetrics trusted_metrics;
        std::string trusted_reason;
        const bool use_recovery_anchor_limits = anomaly_active_;
        bool in_bounds_from_anchor = candidate_is_within_anchor_locked(
          candidate_map_odom, use_recovery_anchor_limits, &trusted_metrics, &trusted_reason);
        // Always evaluate local continuity, even when the older fixed anchor is
        // already out of bounds.  The old short-circuit left correction=0 in
        // diagnostics and made it impossible to distinguish a continuous
        // correction accumulated during a sparse callback interval from a real
        // AMCL discontinuity.
        const bool continuous_from_trusted = candidate_is_reasonable(
          last_trusted_map_odom_, last_trusted_map_base_, last_trusted_stamp_,
          candidate_map_base, current_odom_base, stamp, &trusted_metrics, &trusted_reason);
        bool in_bounds_from_trusted = in_bounds_from_anchor && continuous_from_trusted;
        bool rolling_anchor_catchup_applied = false;
        const bool candidate_score_not_worse =
          !trusted_reference_quality.scan_available ||
          trusted_quality.likelihood_mean + rolling_anchor_catchup_max_likelihood_drop_ >=
          trusted_reference_quality.likelihood_mean;

        const bool raw_candidate_anomalous = !in_bounds_from_trusted;
        const bool raw_candidate_emergency_drift =
          trusted_metrics.cumulative_translation > auto_reanchor_max_translation_ ||
          trusted_metrics.cumulative_yaw > auto_reanchor_max_yaw_;
        const rclcpp::Time trusted_scan_stamp(
          trusted_reference_quality.scan_stamp_ns, RCL_ROS_TIME);
        const rclcpp::Time raw_scan_stamp(raw_override_quality.scan_stamp_ns, RCL_ROS_TIME);
        const auto note_trusted_evidence = [&]() {
            // A scan-qualified frozen transform is fresh localization evidence
            // even though the ambiguous raw AMCL candidate is intentionally not
            // committed.  Keep the moving-silence watchdog from misclassifying
            // a healthy, continuously verified override as AMCL silence.
            last_accepted_amcl_receipt_ = steady_now;
            last_accepted_amcl_stamp_ = stamp;
            have_accepted_amcl_receipt_ = true;
          };
        const auto enter_trusted_override_hard_hold =
          [&](const std::string & hard_reason) {
            reason = hard_reason;
            trusted_override_reason = hard_reason;
            trusted_override_hard_failed = true;
            rejected = true;
            anomaly_active_ = true;
            metrics = trusted_metrics;
            trusted_override_soft_failures = trusted_override_soft_failure_samples_;
            trusted_override_counter_reset_observed = true;
            trusted_override_last_counter_reset_reason_ = "hard_hold";
            clear_trusted_override_locked();
            trusted_override_hard_hold_ = true;
            reset_pending_locked();
          };
        bool trusted_override_temporal_failure_confirmed = false;
        const auto persistent_override_fault_confirmed = [&]() {
            // Finite scan-map score excursions are advisory by themselves: a
            // person or close obstacle can temporarily occlude the static map.
            // Escalate only when that independent evidence coincides with a raw
            // map->odom candidate beyond the existing absolute re-anchor limit.
            const bool finite_numeric_scan_mismatch =
              trusted_override_reason == "trusted_scan_below_override_floor" ||
              trusted_override_reason == "trusted_scan_catastrophic_mismatch";
            const bool temporal_not_ready =
              trusted_override_reason == "trusted_scan_or_same_stamp_tf_not_ready";
            if (temporal_not_ready) {
              if (trusted_override_failure_evidence_ !=
                TrustedOverrideFailureEvidence::kTemporalNotReady &&
                trusted_override_soft_failure_samples_ > 0)
              {
                trusted_override_counter_reset_observed = true;
                trusted_override_last_counter_reset_reason_ =
                  "failure_evidence_category_changed";
              }
              const rclcpp::Time evidence_stamp = trusted_scan_stamp.nanoseconds() != 0 ?
                trusted_scan_stamp : last_new_scan_stamp_;
              note_trusted_override_soft_failure_locked(
                evidence_stamp, TrustedOverrideFailureEvidence::kTemporalNotReady);
              trusted_override_soft_failures = trusted_override_soft_failure_samples_;
              trusted_override_temporal_failure_confirmed =
                trusted_override_soft_failure_samples_ >=
                trusted_override_soft_failure_consecutive_samples_;
              return trusted_override_temporal_failure_confirmed;
            }
            if (!finite_numeric_scan_mismatch || !raw_candidate_emergency_drift) {
              if (trusted_override_soft_failure_samples_ > 0) {
                trusted_override_counter_reset_observed = true;
                trusted_override_last_counter_reset_reason_ =
                  finite_numeric_scan_mismatch ?
                  "scan_mismatch_without_emergency_drift" :
                  "non_numeric_soft_evidence";
              }
              reset_trusted_override_failures_locked();
              trusted_override_soft_failures = 0;
              return false;
            }
            if (trusted_override_failure_evidence_ !=
              TrustedOverrideFailureEvidence::kEmergencyDrift &&
              trusted_override_soft_failure_samples_ > 0)
            {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "failure_evidence_category_changed";
            }
            note_trusted_override_soft_failure_locked(
              trusted_scan_stamp, TrustedOverrideFailureEvidence::kEmergencyDrift);
            trusted_override_soft_failures = trusted_override_soft_failure_samples_;
            return trusted_override_soft_failure_samples_ >=
                   trusted_override_soft_failure_consecutive_samples_;
          };

        if (trusted_override_enabled_ && trusted_override_active_) {
          // Ordinary raw AMCL excursions cannot revoke this mode.  A HOLD needs
          // either a hard integrity fault or persistent scan-map mismatch while
          // the raw map->odom candidate also exceeds the absolute emergency
          // drift ceiling.
          trusted_override_handled = true;
          metrics = trusted_metrics;
          const auto trusted_state = trusted_override_scan_quality_locked(
            trusted_reference_quality, &trusted_override_reason);
          trusted_override_quality_class = trusted_override_quality_label(trusted_state);
          trusted_override_quality_reason_code =
            trusted_override_quality_reason_label(trusted_override_reason);
          if (trusted_state == TrustedOverrideQuality::kGood) {
            if (trusted_override_soft_failure_samples_ > 0) {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "active_good_scan_cleared_soft_failures";
            }
            reset_trusted_override_failures_locked();
            note_trusted_evidence();
            std::string raw_return_reason;
            const auto raw_return_state = trusted_override_scan_quality_locked(
              raw_override_quality, &raw_return_reason);
            if (!raw_candidate_anomalous &&
              raw_return_state == TrustedOverrideQuality::kGood)
            {
              if (advance_trusted_override_return_locked(
                  raw_scan_stamp, steady_now, &trusted_override_samples,
                  &trusted_override_duration_sec))
              {
                if (accept_candidate(
                    candidate_map_odom, candidate_map_base, current_odom_base, stamp,
                    current_odom_stamp, false))
                {
                  anomaly_active_ = false;
                  accepted = true;
                  trusted_override_returned = true;
                  trusted_override_counter_reset_observed = true;
                  trusted_override_last_counter_reset_reason_ =
                    "normal_return_complete";
                } else {
                  enter_trusted_override_hard_hold(
                    "trusted_override_normal_return_commit_refused");
                }
              } else {
                trusted_override_retained = true;
                rejected = true;
                trusted_override_reason = "normal_return_confirmation_pending";
              }
            } else {
              if (trusted_override_return_samples_ > 0) {
                trusted_override_counter_reset_observed = true;
                trusted_override_last_counter_reset_reason_ =
                  "active_raw_return_not_ready";
              }
              reset_trusted_override_return_locked();
              trusted_override_retained = true;
              rejected = true;
              trusted_override_reason = raw_candidate_anomalous ?
                "raw_amcl_outside_normal_envelope" :
                "raw_amcl_absolute_scan_quality_not_ready";
            }
          } else if (trusted_state == TrustedOverrideQuality::kSoftFailure) {
            trusted_override_soft_failure_observed = true;
            if (trusted_override_return_samples_ > 0) {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "active_soft_failure_cleared_return_confirmation";
            }
            reset_trusted_override_return_locked();
            if (persistent_override_fault_confirmed()) {
              enter_trusted_override_hard_hold(
                trusted_override_temporal_failure_confirmed ?
                "trusted_override_persistent_scan_tf_not_ready" :
                "trusted_override_confirmed_emergency_map_odom_drift");
            } else {
              trusted_override_retained = true;
              rejected = true;
            }
          } else {
            enter_trusted_override_hard_hold(trusted_override_reason);
          }
        } else if (trusted_override_enabled_ && trusted_override_soft_hold_) {
          // Preserve recovery for a legacy/integrity SOFT_HOLD, but require a
          // stationary independent scan window. Raw AMCL is never accepted as
          // evidence for this release.
          trusted_override_handled = true;
          trusted_override_soft_hold_waiting = true;
          metrics = trusted_metrics;
          const auto trusted_state = trusted_override_scan_quality_locked(
            trusted_reference_quality, &trusted_override_reason);
          trusted_override_quality_class = trusted_override_quality_label(trusted_state);
          trusted_override_quality_reason_code =
            trusted_override_quality_reason_label(trusted_override_reason);
          if (trusted_state == TrustedOverrideQuality::kGood) {
            if (trusted_override_soft_failure_samples_ > 0) {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "soft_hold_good_scan_cleared_soft_failures";
            }
            reset_trusted_override_failures_locked();
            note_trusted_evidence();
            if (!trusted_override_recovery_is_stationary_locked(steady_now)) {
              if (trusted_override_confirmation_samples_ > 0) {
                trusted_override_counter_reset_observed = true;
                trusted_override_last_counter_reset_reason_ =
                  "soft_hold_motion_cleared_confirmation";
              }
              reset_trusted_override_confirmation_locked();
              trusted_override_reason = "soft_hold_waiting_for_stationary";
            } else if (advance_trusted_override_confirmation_locked(
                trusted_scan_stamp, steady_now,
                trusted_override_hold_consecutive_samples_,
                trusted_override_hold_min_duration_sec_,
                &trusted_override_samples, &trusted_override_duration_sec))
            {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "soft_hold_recovery_complete";
              clear_trusted_override_locked();
              trusted_override_active_ = true;
              anomaly_active_ = false;
              trusted_override_soft_hold_waiting = false;
              trusted_override_recovered = true;
              rejected = true;
              trusted_override_reason = "stationary_trusted_scan_recovery_complete";
            } else {
              rejected = true;
              trusted_override_reason = "soft_hold_trusted_confirmation_pending";
            }
          } else if (trusted_state == TrustedOverrideQuality::kSoftFailure) {
            trusted_override_soft_failure_observed = true;
            if (trusted_override_confirmation_samples_ > 0) {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "soft_hold_soft_failure_cleared_confirmation";
            }
            reset_trusted_override_confirmation_locked();
            rejected = true;
            if (persistent_override_fault_confirmed()) {
              enter_trusted_override_hard_hold(
                trusted_override_temporal_failure_confirmed ?
                "soft_hold_persistent_scan_tf_not_ready" :
                "soft_hold_confirmed_emergency_map_odom_drift");
            }
          } else {
            enter_trusted_override_hard_hold(trusted_override_reason);
          }
        } else if (trusted_override_enabled_ && !post_reanchor_verification_active_ &&
          !anomaly_active_ && raw_candidate_anomalous)
        {
          // ACTIVE first freezes the last trusted transform and verifies that
          // transform before asserting anomaly.  The outlying raw candidate is
          // never committed and cannot influence the authorization score.
          trusted_override_handled = true;
          trusted_override_verifying = true;
          rejected = true;
          metrics = trusted_metrics;
          reset_pending_locked();
          if (!trusted_override_verify_epoch_active_) {
            trusted_override_verify_epoch_active_ = true;
            trusted_override_verify_epoch_was_active = true;
            trusted_override_verify_started = true;
            trusted_override_verify_epoch_started_ = steady_now;
            ++trusted_override_verify_epoch_id_;
            trusted_override_hard_hold_ = false;
            trusted_override_last_diagnostic_scan_stamp_ =
              rclcpp::Time(0, 0, RCL_ROS_TIME);
            trusted_override_last_counter_reset_reason_ = "verify_epoch_started";
            reset_trusted_override_confirmation_locked();
            reset_trusted_override_failures_locked();
          }
          const auto trusted_state = trusted_override_scan_quality_locked(
            trusted_reference_quality, &trusted_override_reason);
          trusted_override_quality_class = trusted_override_quality_label(trusted_state);
          trusted_override_quality_reason_code =
            trusted_override_quality_reason_label(trusted_override_reason);
          if (trusted_state == TrustedOverrideQuality::kGood) {
            if (trusted_override_soft_failure_samples_ > 0) {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "verify_good_scan_cleared_soft_failures";
            }
            reset_trusted_override_failures_locked();
            note_trusted_evidence();
            if (advance_trusted_override_confirmation_locked(
                trusted_scan_stamp, steady_now,
                trusted_override_active_consecutive_samples_,
                trusted_override_active_min_duration_sec_,
                &trusted_override_samples, &trusted_override_duration_sec))
            {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "verify_authorized_override";
              clear_trusted_override_locked();
              trusted_override_active_ = true;
              anomaly_active_ = false;
              trusted_override_verifying = false;
              trusted_override_entered = true;
              trusted_override_reason = "trusted_scan_override_confirmed";
            } else {
              trusted_override_reason = "trusted_scan_override_confirmation_pending";
            }
          } else if (trusted_state == TrustedOverrideQuality::kSoftFailure) {
            trusted_override_soft_failure_observed = true;
            if (trusted_override_confirmation_samples_ > 0) {
              trusted_override_counter_reset_observed = true;
              trusted_override_last_counter_reset_reason_ =
                "verify_soft_failure_cleared_confirmation";
            }
            reset_trusted_override_confirmation_locked();
            if (persistent_override_fault_confirmed()) {
              trusted_override_verifying = false;
              enter_trusted_override_hard_hold(
                trusted_override_temporal_failure_confirmed ?
                "verify_persistent_scan_tf_not_ready" :
                "verify_confirmed_emergency_map_odom_drift");
            }
          } else {
            trusted_override_verifying = false;
            enter_trusted_override_hard_hold(trusted_override_reason);
          }
        } else if (trusted_override_verify_epoch_active_ && !raw_candidate_anomalous) {
          // Raw AMCL returned to the unchanged normal 0.30 m / 0.20 rad
          // envelope before override authorization.  Resume the existing normal
          // commit path; no anomaly was asserted during VERIFY_TRUSTED.
          trusted_override_verify_epoch_active_ = false;
          trusted_override_verify_cancelled = true;
          metrics = trusted_metrics;
          if (trusted_override_confirmation_samples_ > 0 ||
            trusted_override_soft_failure_samples_ > 0)
          {
            trusted_override_counter_reset_observed = true;
          }
          trusted_override_last_counter_reset_reason_ =
            "verify_raw_candidate_returned_normal_envelope";
          reset_trusted_override_confirmation_locked();
          reset_trusted_override_failures_locked();
        }

        if (!trusted_override_handled && cumulative_overrun_defer_active_ &&
          in_bounds_from_trusted && !anomaly_active_)
        {
          cumulative_overrun_cleared = true;
          cumulative_overrun_defer_duration_sec = std::chrono::duration<double>(
            steady_now - cumulative_overrun_defer_started_).count();
          confirmation_count = cumulative_overrun_defer_samples_;
          reset_cumulative_overrun_defer_locked();
        }

        if (!trusted_override_handled && !post_reanchor_verification_active_ && !anomaly_active_ &&
          !cumulative_overrun_defer_active_ &&
          !in_bounds_from_anchor && continuous_from_trusted)
        {
          const double candidate_gap_sec = trusted_metrics.dt;
          const bool scan_temporally_valid = trusted_quality.scan_available &&
            trusted_quality.scan_time_delta_sec <= active_quality_scan_tolerance_sec_ &&
            (trusted_quality.scan_time_delta_sec <= 1.0e-6 ||
            trusted_quality.scan_motion_compensated);
          if (!trusted_quality_ok) {
            rolling_anchor_skip_reason = "catchup_quality_" + trusted_quality.reason;
          } else if (!scan_temporally_valid) {
            // A nearest scan is acceptable only when it was explicitly
            // reconstructed at its own odom stamp by evaluate_bootstrap_quality().
            rolling_anchor_skip_reason = "catchup_scan_not_time_compensated";
          } else if (!candidate_score_not_worse) {
            rolling_anchor_skip_reason = "catchup_candidate_score_worse_than_trusted";
          } else if (candidate_gap_sec <= 0.0 ||
            candidate_gap_sec > rolling_anchor_catchup_max_gap_sec_)
          {
            rolling_anchor_skip_reason = "catchup_source_gap_out_of_bounds";
          } else {
            // AMCL updates normally arrive at least once per validation-gap
            // interval.  If executor pressure coalesces several otherwise
            // continuous updates, allow the per-callback step to catch up by
            // the number of missed intervals.  Sliding-window and session
            // budgets remain unchanged, and the helper commits transactionally
            // only if the resulting candidate is inside the original 0.30/0.20
            // hard envelope.
            const double nominal_gap_sec = std::max(0.1, max_amcl_validation_gap_sec_);
            const double catchup_step_scale = std::min(
              rolling_anchor_catchup_max_step_scale_,
              std::max(1.0, candidate_gap_sec / nominal_gap_sec));
            rolling_anchor_catchup_applied = maybe_advance_rolling_anchor_locked(
              candidate_map_odom, stamp, steady_now, catchup_step_scale, true,
              &rolling_anchor_skip_reason);
            if (rolling_anchor_catchup_applied) {
              in_bounds_from_anchor = candidate_is_within_anchor_locked(
                candidate_map_odom, false, &trusted_metrics, &trusted_reason);
              in_bounds_from_trusted = in_bounds_from_anchor && continuous_from_trusted;
            }
          }
        }
        if (trusted_override_handled) {
          // The TRUSTED_OVERRIDE state machine above owns this sample.  In
          // particular, do not fall through to legacy far-cluster recovery or
          // commit the raw AMCL candidate while it remains ambiguous.
        } else if (post_reanchor_verification_active_) {
          required_confirmations = post_reanchor_consecutive_valid_samples_;
          metrics = trusted_metrics;
          if (!trusted_reanchor_quality_ok) {
            reason = "post_reanchor_quality_" + trusted_reanchor_quality_reason;
            reset_pending_locked();
            rejected = true;
          } else if (!interlock_is_freshly_blocked_locked(steady_now)) {
            reason = "post_reanchor_waiting_for_fresh_interlock_blocked";
            reset_pending_locked();
            rejected = true;
          } else if (!recovery_is_stationary_locked(steady_now)) {
            reason = "post_reanchor_waiting_for_stationary";
            reset_pending_locked();
            rejected = true;
          } else if (!in_bounds_from_trusted) {
            reason = "post_reanchor_continuity_" + trusted_reason;
            reset_pending_locked();
            rejected = true;
          } else if (advance_recovery_cluster_locked(
              candidate_map_odom, candidate_map_base, current_odom_base, steady_now,
              required_confirmations, post_reanchor_min_confirm_duration_sec_))
          {
            if (accept_candidate(
                candidate_map_odom, candidate_map_base, current_odom_base, stamp,
                current_odom_stamp, false))
            {
              post_reanchor_verification_active_ = false;
              anomaly_active_ = false;
              reset_recovery_nomotion_budget_locked();
              accepted = true;
              auto_reanchor_verified = true;
            }
          } else {
            reason = "collecting_post_reanchor_quality_continuity_window";
            confirmation_count = recovery_cluster_valid_samples_;
            anomaly_active_ = true;
            rejected = true;
          }
        } else if (!in_bounds_from_trusted && !anomaly_active_) {
          metrics = trusted_metrics;
          std::string defer_reason;
          const bool cumulative_only_overrun = !in_bounds_from_anchor &&
            continuous_from_trusted;
          bool defer_eligible = false;
          if (!cumulative_only_overrun) {
            defer_reason = trusted_reason;
          } else if (!trusted_quality_ok) {
            defer_reason = "cumulative_overrun_quality_" + trusted_quality.reason;
          } else if (!candidate_score_not_worse) {
            defer_reason = "cumulative_overrun_candidate_score_worse_than_trusted";
          } else {
            defer_eligible = cumulative_overrun_defer_eligible_locked(
              trusted_metrics, trusted_quality, stamp, current_odom_stamp,
              steady_now, &defer_reason);
          }
          if (defer_eligible) {
            required_confirmations = cumulative_overrun_defer_consecutive_samples_;
            cumulative_overrun_deferred = advance_cumulative_overrun_defer_locked(
              trusted_metrics, stamp, steady_now, &confirmation_count,
              &cumulative_overrun_defer_duration_sec);
            if (cumulative_overrun_deferred) {
              reason = "cumulative_overrun_confirmation_pending";
            } else {
              cumulative_overrun_confirmed = true;
              cumulative_overrun_peak_translation =
                cumulative_overrun_defer_peak_translation_;
              cumulative_overrun_peak_yaw = cumulative_overrun_defer_peak_yaw_;
              reason = "cumulative_overrun_confirmed_by_new_stamps";
              anomaly_active_ = true;
              reset_pending_locked();
            }
          } else {
            reason = cumulative_only_overrun ? defer_reason : trusted_reason;
            anomaly_active_ = true;
            reset_pending_locked();
          }
          // A deferred sample freezes the candidate without committing or
          // rolling the anchor.  Immediate/confirmed faults assert anomaly
          // first so the interlock reports BLOCKED before far recovery starts.
          rejected = true;
        } else if (!anomaly_active_) {
          accepted = accept_candidate(
            candidate_map_odom, candidate_map_base, current_odom_base, stamp,
            current_odom_stamp, false);
          if (accepted && trusted_quality_ok) {
            // A past timing fault may inhibit automatic far re-anchor after its
            // low-level timer clears.  Only a scan-qualified commit that is
            // already inside the trusted envelope may clear that latch; a far
            // candidate can never clear its own safety inhibit.
            clear_auto_reanchor_integrity_inhibit_locked("scan_qualified_active_commit");
            if (!rolling_anchor_catchup_applied) {
              maybe_advance_rolling_anchor_locked(
                candidate_map_odom, stamp, steady_now, 1.0, false,
                &rolling_anchor_skip_reason);
            }
          } else if (accepted) {
            rolling_anchor_skip_reason = "quality_" + trusted_quality.reason;
          }
        } else if (!in_bounds_from_trusted) {
          metrics = trusted_metrics;
          required_confirmations = recovery_cluster_consecutive_samples_;
          if (!trusted_reanchor_quality_ok) {
            reason = "auto_reanchor_quality_" + trusted_reanchor_quality_reason;
            reset_pending_locked();
            rejected = true;
          } else if (
            trusted_quality.likelihood_mean - trusted_reference_quality.likelihood_mean <
            auto_reanchor_min_likelihood_improvement_)
          {
            reason = "auto_reanchor_ambiguous_vs_trusted_pose";
            reset_pending_locked();
            rejected = true;
          } else if (!auto_reanchor_preconditions_locked(
              candidate_map_odom, steady_now, &metrics, &reason))
          {
            reset_pending_locked();
            rejected = true;
          } else if (advance_recovery_cluster_locked(
              candidate_map_odom, candidate_map_base, current_odom_base, steady_now,
              required_confirmations, recovery_cluster_min_duration_sec_))
          {
            stable_far_candidate_ready = true;
            const tf2::Transform selected_map_odom = recovery_cluster_map_odom_;
            const tf2::Transform selected_map_base = selected_map_odom * current_odom_base;
            if (accept_candidate(
                selected_map_odom, selected_map_base, current_odom_base, stamp,
                current_odom_stamp, true))
            {
              // A discontinuous TF correction is published only while the
              // independently reported interlock remains BLOCKED.  Keep anomaly
              // asserted after the reset; a second, fresh quality/continuity
              // window must pass before Nav2 begins its own release debounce.
              reset_rolling_anchor_budget_locked(false);
              ++auto_reanchors_used_;
              post_reanchor_verification_active_ = true;
              anomaly_active_ = true;
              accepted = true;
              auto_reanchor_started = true;
            } else {
              reason = "auto_reanchor_commit_refused";
              rejected = true;
            }
          } else {
            reason = "collecting_stationary_quality_far_cluster";
            confirmation_count = recovery_cluster_valid_samples_;
            anomaly_active_ = true;
            rejected = true;
          }
        } else {
          // Once the candidate returns to the trusted spatial envelope, the
          // far-cluster diagnostic is no longer relevant.  Confirm a fresh
          // near-trusted sequence before clearing HOLD.
          have_recovery_cluster_ = false;
          recovery_cluster_valid_samples_ = 0;
          required_confirmations = recovery_consecutive_valid_samples_;
          if (advance_pending_locked(
              candidate_map_odom, candidate_map_base, current_odom_base, stamp,
              required_confirmations, &metrics, &reason))
          {
            if (accept_candidate(
                candidate_map_odom, candidate_map_base, current_odom_base, stamp,
                current_odom_stamp, false))
            {
              anomaly_active_ = false;
              clear_auto_reanchor_integrity_inhibit_locked("near_anchor_recovery_accepted");
              reset_recovery_nomotion_budget_locked();
              accepted = true;
              recovery_accepted = true;
            }
          } else {
            anomaly_active_ = true;
            confirmation_count = pending_valid_samples_;
          }
        }
      }
      if (trusted_override_handled || trusted_override_verify_cancelled) {
        trusted_override_diagnostic_snapshot_valid = true;
        const rclcpp::Time diagnostic_scan_stamp(
          trusted_reference_quality.scan_stamp_ns, RCL_ROS_TIME);
        if (diagnostic_scan_stamp.nanoseconds() == 0) {
          trusted_override_scan_disposition = "zero";
          trusted_override_scan_disposition_needs_throttle = true;
        } else if (trusted_override_last_diagnostic_scan_stamp_.nanoseconds() == 0 ||
          diagnostic_scan_stamp > trusted_override_last_diagnostic_scan_stamp_)
        {
          trusted_override_scan_disposition = "new";
          trusted_override_last_diagnostic_scan_stamp_ = diagnostic_scan_stamp;
        } else if (diagnostic_scan_stamp == trusted_override_last_diagnostic_scan_stamp_) {
          trusted_override_scan_disposition = "duplicate";
          trusted_override_scan_disposition_needs_throttle = true;
        } else {
          trusted_override_scan_disposition = "out_of_order";
          trusted_override_scan_disposition_needs_throttle = true;
        }
        trusted_override_verify_epoch = trusted_override_verify_epoch_id_;
        if (trusted_override_verify_epoch_was_active) {
          trusted_override_verify_elapsed_sec = std::chrono::duration<double>(
            steady_now - trusted_override_verify_epoch_started_).count();
        }
        trusted_override_confirmations_after = trusted_override_confirmation_samples_;
        trusted_override_soft_failures_after = trusted_override_soft_failure_samples_;
        trusted_override_returns_after = trusted_override_return_samples_;
        trusted_override_counter_reset_reason = trusted_override_last_counter_reset_reason_;
        if (!trusted_override_hard_failed) {
          trusted_override_soft_failures = trusted_override_soft_failure_samples_;
        }
      }
      if (accepted) {
        last_accepted_amcl_receipt_ = steady_now;
        have_accepted_amcl_receipt_ = true;
      }
    }

    const bool trusted_override_counter_changed =
      trusted_override_diagnostic_snapshot_valid &&
      (trusted_override_confirmations_before != trusted_override_confirmations_after ||
      trusted_override_soft_failures_before != trusted_override_soft_failures_after ||
      trusted_override_returns_before != trusted_override_returns_after);
    const bool trusted_override_diagnostic_event =
      trusted_override_diagnostic_snapshot_valid &&
      (trusted_override_verify_started || trusted_override_verify_cancelled ||
      trusted_override_entered || trusted_override_recovered ||
      trusted_override_returned || trusted_override_hard_failed ||
      trusted_override_counter_reset_observed || trusted_override_counter_changed);
    const bool trusted_override_repeated_scan_event =
      trusted_override_diagnostic_snapshot_valid && !trusted_override_diagnostic_event &&
      trusted_override_scan_disposition_needs_throttle &&
      (trusted_override_verifying || trusted_override_soft_hold_waiting ||
      trusted_override_retained);
    const rclcpp::Time diagnostic_scan_stamp(
      trusted_reference_quality.scan_stamp_ns, RCL_ROS_TIME);
    const double trusted_override_scan_pose_offset_sec =
      diagnostic_scan_stamp.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() :
      (diagnostic_scan_stamp - stamp).seconds();
    const double trusted_override_scan_age_sec =
      diagnostic_scan_stamp.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() :
      pose_age_sec - trusted_override_scan_pose_offset_sec;
    const char * trusted_override_branch = "OTHER";
    if (trusted_override_hard_failed) {
      trusted_override_branch = "HARD_HOLD";
    } else if (trusted_override_verify_cancelled) {
      trusted_override_branch = "VERIFY_CANCELLED_RAW_RETURN";
    } else if (trusted_override_entered) {
      trusted_override_branch = "TRUSTED_OVERRIDE_ENTER";
    } else if (trusted_override_recovered) {
      trusted_override_branch = "SOFT_HOLD_RECOVERED";
    } else if (trusted_override_returned) {
      trusted_override_branch = "TRUSTED_OVERRIDE_RETURN_NORMAL";
    } else if (trusted_override_soft_hold_waiting) {
      trusted_override_branch = "SOFT_HOLD";
    } else if (trusted_override_verifying) {
      trusted_override_branch = "VERIFY_TRUSTED";
    } else if (trusted_override_retained) {
      trusted_override_branch = "TRUSTED_OVERRIDE_RETAINED";
    }
    const char * trusted_override_diagnostic_reason =
      trusted_override_verify_cancelled ? "raw_candidate_returned_normal_envelope" :
      (trusted_override_returned ? "normal_return_complete" :
      (trusted_override_reason.empty() ? "not_set" : trusted_override_reason.c_str()));
    const auto emit_trusted_override_diagnostic = [&]() {
        if (trusted_override_diagnostic_event) {
          RCLCPP_INFO(
            get_logger(),
            "TRUSTED_OVERRIDE_DIAG branch=%s reason=%s quality_class=%s "
            "quality_reason_code=%s verify_epoch=%llu "
            "verify_elapsed_sec=%.3f pose_source_stamp_ns=%lld pose_age_sec=%.3f "
            "scan_source_stamp_ns=%lld scan_ros_source_age_sec=%.3f "
            "scan_pose_offset_sec=%+.3f selected_scan_disposition=%s "
            "confirmations=%d->%d soft_failure_observed=%s soft_failures=%d/%d (%d->%d) "
            "returns=%d->%d last_counter_reset_reason=%s counter_reset_observed=%s.",
            trusted_override_branch, trusted_override_diagnostic_reason,
            trusted_override_quality_class, trusted_override_quality_reason_code,
            static_cast<unsigned long long>(trusted_override_verify_epoch),
            trusted_override_verify_elapsed_sec, static_cast<long long>(stamp.nanoseconds()),
            pose_age_sec, static_cast<long long>(trusted_reference_quality.scan_stamp_ns),
            trusted_override_scan_age_sec, trusted_override_scan_pose_offset_sec,
            trusted_override_scan_disposition, trusted_override_confirmations_before,
            trusted_override_confirmations_after,
            trusted_override_soft_failure_observed ? "yes" : "no",
            trusted_override_soft_failures,
            trusted_override_soft_failure_consecutive_samples_,
            trusted_override_soft_failures_before, trusted_override_soft_failures_after,
            trusted_override_returns_before, trusted_override_returns_after,
            trusted_override_counter_reset_reason,
            trusted_override_counter_reset_observed ? "yes" : "no");
          RCLCPP_INFO(
            get_logger(),
            "TRUSTED_OVERRIDE_QUALITY verify_epoch=%llu scan_source_stamp_ns=%lld "
            "scan_available=%s scan_motion_compensated=%s base_in_free_space=%s beams=%zu "
            "inlier=%.3f likelihood=%.3f trimmed=%.3f unknown/offmap=%.3f "
            "correction_translation=%.3f correction_yaw=%.3f "
            "cumulative_translation=%.3f cumulative_yaw=%.3f.",
            static_cast<unsigned long long>(trusted_override_verify_epoch),
            static_cast<long long>(trusted_reference_quality.scan_stamp_ns),
            trusted_reference_quality.scan_available ? "yes" : "no",
            trusted_reference_quality.scan_motion_compensated ? "yes" : "no",
            trusted_reference_quality.base_in_free_space ? "yes" : "no",
            trusted_reference_quality.considered_beams,
            trusted_reference_quality.inlier_ratio,
            trusted_reference_quality.likelihood_mean,
            trusted_reference_quality.trimmed_mean_distance,
            trusted_reference_quality.unknown_offmap_ratio,
            metrics.correction_translation, metrics.correction_yaw,
            metrics.cumulative_translation, metrics.cumulative_yaw);
          return;
        }
        if (!trusted_override_repeated_scan_event) {
          return;
        }
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "TRUSTED_OVERRIDE_SCAN_EVIDENCE branch=%s reason=%s quality_class=%s "
          "quality_reason_code=%s verify_epoch=%llu scan_source_stamp_ns=%lld "
          "scan_ros_source_age_sec=%.3f scan_pose_offset_sec=%+.3f "
          "selected_scan_disposition=%s scan_available=%s "
          "scan_motion_compensated=%s beams=%zu inlier=%.3f likelihood=%.3f "
          "trimmed=%.3f unknown/offmap=%.3f confirmations=%d soft_failures=%d/%d returns=%d.",
          trusted_override_branch, trusted_override_diagnostic_reason,
          trusted_override_quality_class, trusted_override_quality_reason_code,
          static_cast<unsigned long long>(trusted_override_verify_epoch),
          static_cast<long long>(trusted_reference_quality.scan_stamp_ns),
          trusted_override_scan_age_sec, trusted_override_scan_pose_offset_sec,
          trusted_override_scan_disposition,
          trusted_reference_quality.scan_available ? "yes" : "no",
          trusted_reference_quality.scan_motion_compensated ? "yes" : "no",
          trusted_reference_quality.considered_beams,
          trusted_reference_quality.inlier_ratio,
          trusted_reference_quality.likelihood_mean,
          trusted_reference_quality.trimmed_mean_distance,
          trusted_reference_quality.unknown_offmap_ratio,
          trusted_override_confirmations_after, trusted_override_soft_failures_after,
          trusted_override_soft_failure_consecutive_samples_,
          trusted_override_returns_after);
      };

    if (!rolling_anchor_skip_reason.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rolling anchor not advanced (%s): selected scan delta %.3f s, "
        "odom motion compensation=%s, scan=%s, beams=%zu, likelihood=%.3f, inlier=%.3f.",
        rolling_anchor_skip_reason.c_str(), trusted_quality.scan_time_delta_sec,
        trusted_quality.scan_motion_compensated ? "yes" : "no",
        trusted_quality.scan_available ? "available" : "unavailable",
        trusted_quality.considered_beams, trusted_quality.likelihood_mean,
        trusted_quality.inlier_ratio);
    }

    if (cumulative_overrun_cleared) {
      RCLCPP_INFO(
        get_logger(),
        "Cumulative map->odom overrun confirmation cleared after %d distinct source "
        "stamps / %.3f s because the candidate returned inside the unchanged "
        "%.3f m / %.3f rad envelope; anomaly was not asserted.",
        confirmation_count, cumulative_overrun_defer_duration_sec,
        max_cumulative_map_odom_translation_, max_cumulative_map_odom_yaw_);
    }
    if (cumulative_overrun_confirmed) {
      RCLCPP_ERROR(
        get_logger(),
        "Cumulative map->odom overrun confirmed after %d distinct AMCL source stamps "
        "spanning %.3f s (peak %.3f m / %.3f rad). Entering HOLD with the last "
        "trusted TF frozen.",
        confirmation_count, cumulative_overrun_defer_duration_sec,
        cumulative_overrun_peak_translation, cumulative_overrun_peak_yaw);
    }

    if (trusted_override_handled && !accepted) {
      publish_current_anomaly();
      publish_last_trusted_transform();
      emit_trusted_override_diagnostic();
      if (trusted_override_hard_failed) {
        RCLCPP_ERROR(
          get_logger(),
          "TRUSTED_OVERRIDE entered HARD_HOLD (%s): trusted scan stamp=%lld, beams=%zu, "
          "inlier=%.3f, likelihood=%.3f, trimmed=%.3f, unknown/offmap=%.3f, "
          "soft_failures=%d/%d, raw_cumulative=%.3f m / %.3f rad, "
          "emergency_limits=%.3f m / %.3f rad, raw_drift_role=%s.",
          trusted_override_reason.c_str(),
          static_cast<long long>(trusted_reference_quality.scan_stamp_ns),
          trusted_reference_quality.considered_beams,
          trusted_reference_quality.inlier_ratio,
          trusted_reference_quality.likelihood_mean,
          trusted_reference_quality.trimmed_mean_distance,
          trusted_reference_quality.unknown_offmap_ratio,
          trusted_override_soft_failures,
          trusted_override_soft_failure_consecutive_samples_,
          metrics.cumulative_translation, metrics.cumulative_yaw,
          auto_reanchor_max_translation_, auto_reanchor_max_yaw_,
          trusted_override_reason.find("emergency_map_odom_drift") != std::string::npos ?
          "joint_required_and_exceeded" :
          (trusted_override_reason.find("scan_tf_not_ready") != std::string::npos ?
          "not_required_for_temporal_tf_fault" : "not_required_for_categorical_fault"));
      } else if (trusted_override_entered) {
        RCLCPP_WARN(
          get_logger(),
          "VERIFY_TRUSTED passed %d fresh scan stamps over %.3f s; entering "
          "TRUSTED_OVERRIDE with anomaly healthy. Raw AMCL remains rejected and the last "
          "trusted map->odom is unchanged.",
          trusted_override_samples, trusted_override_duration_sec);
      } else if (trusted_override_recovered) {
        RCLCPP_INFO(
          get_logger(),
          "Recoverable TRUSTED_OVERRIDE SOFT_HOLD released after %d fresh trusted scans "
          "over %.3f s while stationary; anomaly is healthy and the frozen trusted TF "
          "remains authoritative.",
          trusted_override_samples, trusted_override_duration_sec);
      } else if (trusted_override_soft_hold_waiting) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TRUSTED_OVERRIDE SOFT_HOLD recovery pending (%s): trusted scans=%d/%d over "
          "%.3f/%.3f s, stationary requirement %.3f s, soft failures=%d/%d.",
          trusted_override_reason.c_str(), trusted_override_samples,
          trusted_override_hold_consecutive_samples_, trusted_override_duration_sec,
          trusted_override_hold_min_duration_sec_,
          trusted_override_hold_stationary_duration_sec_, trusted_override_soft_failures,
          trusted_override_soft_failure_consecutive_samples_);
      } else if (trusted_override_verifying) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "VERIFY_TRUSTED pending (%s): fresh trusted scans=%d/%d over %.3f/%.3f s, "
          "soft failures=%d/%d. Candidate is frozen; anomaly is not asserted.",
          trusted_override_reason.c_str(), trusted_override_samples,
          trusted_override_active_consecutive_samples_, trusted_override_duration_sec,
          trusted_override_active_min_duration_sec_, trusted_override_soft_failures,
          trusted_override_soft_failure_consecutive_samples_);
      } else if (trusted_override_retained) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TRUSTED_OVERRIDE retained (%s): trusted scan stamp=%lld, beams=%zu, "
          "inlier=%.3f, likelihood=%.3f, trimmed=%.3f, unknown/offmap=%.3f. "
          "Raw candidate is ignored; anomaly remains healthy.",
          trusted_override_reason.c_str(),
          static_cast<long long>(trusted_reference_quality.scan_stamp_ns),
          trusted_reference_quality.considered_beams,
          trusted_reference_quality.inlier_ratio,
          trusted_reference_quality.likelihood_mean,
          trusted_reference_quality.trimmed_mean_distance,
          trusted_reference_quality.unknown_offmap_ratio);
      }
      return;
    }

    if (!accepted && !have_trusted_before_candidate) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Automatic AMCL anchor remains HOLD (%s): covariance lambda/yaw=%.4f/%.4f, "
        "scan beams=%zu inlier=%.3f likelihood=%.3f neighbor_gain=%.3f trimmed=%.3f "
        "unknown/offmap=%.3f, "
        "confirmation=%d/%d.",
        reason.c_str(), bootstrap_quality.covariance_xy_lambda_max,
        bootstrap_quality.covariance_yaw, bootstrap_quality.considered_beams,
        bootstrap_quality.inlier_ratio, bootstrap_quality.likelihood_mean,
        bootstrap_quality.best_neighbor_likelihood_gain,
        bootstrap_quality.trimmed_mean_distance,
        bootstrap_quality.unknown_offmap_ratio, confirmation_count, required_confirmations);
    }

    if (!accepted) {
      if (cumulative_overrun_deferred) {
        // Keep the last trusted TF (which is republished with a fresh TF stamp)
        // and the existing healthy anomaly state.  This is a bounded evidence
        // window, not authorization to commit the out-of-envelope candidate.
        publish_current_anomaly();
        emit_trusted_override_diagnostic();
        RCLCPP_WARN(
          get_logger(),
          "Cumulative map->odom overrun pending confirmation %d/%d over %.3f/%.3f s "
          "(source_stamp_ns=%lld, cumulative %.3f m / %.3f rad, correction %.3f m / "
          "%.3f rad). Candidate and rolling anchor are frozen; anomaly is not newly "
          "asserted unless the overrun persists or the %.3f s deadline expires.",
          confirmation_count, required_confirmations,
          cumulative_overrun_defer_duration_sec,
          cumulative_overrun_defer_min_duration_sec_,
          static_cast<long long>(stamp.nanoseconds()), metrics.cumulative_translation,
          metrics.cumulative_yaw, metrics.correction_translation, metrics.correction_yaw,
          cumulative_overrun_defer_max_duration_sec_);
        return;
      }
      publish_anomaly(true);
      emit_trusted_override_diagnostic();
      if (!anomaly_was_active || rejected) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "AMCL spatial candidate held (%s, source_stamp_ns=%lld); keeping the last trusted "
          "%s -> %s. "
          "source candidate gap %.3f s, correction %.3f m / %.3f rad "
          "(limits %.3f / %.3f), "
          "cumulative anchor drift %.3f m / %.3f rad (limits %.3f / %.3f), "
          "confirmation=%d/%d",
          reason.c_str(), static_cast<long long>(stamp.nanoseconds()),
          global_frame_.c_str(), odom_frame_.c_str(),
          metrics.dt,
          metrics.correction_translation, metrics.correction_yaw,
          max_odom_correction_, max_odom_yaw_correction_,
          metrics.cumulative_translation, metrics.cumulative_yaw,
          anomaly_was_active ? recovery_cumulative_map_odom_translation_ :
          max_cumulative_map_odom_translation_,
          anomaly_was_active ? recovery_cumulative_map_odom_yaw_ :
          max_cumulative_map_odom_yaw_,
          confirmation_count, required_confirmations);
      }
      if (stable_far_candidate_ready) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Stable far AMCL candidate cluster confirmed (%d/%d), but the guarded re-anchor "
          "commit was refused. Keeping HOLD and the last trusted transform.",
          confirmation_count, required_confirmations);
      }
      return;
    }

    publish_current_anomaly();
    publish_last_trusted_transform();
    emit_trusted_override_diagnostic();
    if (bootstrap_accepted) {
      RCLCPP_INFO(
        get_logger(),
        "Automatic AMCL localization armed the fixed map->odom anchor after %d "
        "scan-map/covariance-qualified samples spanning at least %.1f s.",
        bootstrap_good_samples_, bootstrap_min_confirm_duration_sec_);
    } else if (rebase_accepted) {
      RCLCPP_INFO(
        get_logger(), "Explicit AMCL rebase accepted after %d stationary consistent samples.",
        initialpose_rebase_consecutive_valid_samples_);
    } else if (auto_reanchor_started) {
      RCLCPP_WARN(
        get_logger(),
        "Guarded in-session AMCL re-anchor %d/%d committed while interlock BLOCKED. "
        "Anomaly remains asserted until %d additional scan-qualified, stationary, "
        "TF-continuous samples span at least %.1f s.",
        auto_reanchors_used_, max_auto_reanchors_per_session_,
        post_reanchor_consecutive_valid_samples_, post_reanchor_min_confirm_duration_sec_);
    } else if (auto_reanchor_verified) {
      RCLCPP_INFO(
        get_logger(),
        "Guarded in-session AMCL re-anchor verified; anomaly is healthy and the interlock "
        "must still complete its own stability and post-activation cleanup windows.");
    } else if (recovery_accepted) {
      RCLCPP_INFO(
        get_logger(), "AMCL TF recovery accepted after %d consistent samples.",
        recovery_consecutive_valid_samples_);
    } else if (trusted_override_returned) {
      RCLCPP_INFO(
        get_logger(),
        "Raw AMCL returned inside the unchanged normal %.3f m / %.3f rad envelope and "
        "passed %d fresh absolute scan confirmations over %.3f s; TRUSTED_OVERRIDE "
        "returned to NORMAL.",
        max_cumulative_map_odom_translation_, max_cumulative_map_odom_yaw_,
        trusted_override_samples, trusted_override_duration_sec);
    }
  }

  void enter_hold(const std::string & reason, const bool suppress_tf_output = false)
  {
    (void)suppress_tf_output;
    bool newly_active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      newly_active = !anomaly_active_;
      anomaly_active_ = true;
      // A spatial HOLD retains the last trusted transform.  Timing or input
      // availability must never make the map frame disappear.
      suppress_tf_output_ = false;
      clear_trusted_override_locked();
      reset_pending_locked();
    }
    publish_anomaly(true);
    if (newly_active) {
      RCLCPP_ERROR(
        get_logger(),
        "AMCL TF guard entered spatial HOLD (%s); retaining the last trusted transform.",
        reason.c_str());
    }
  }

  void enter_amcl_timing_hold(const std::string & reason, const rclcpp::Time & source_stamp)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      amcl_timing_hold_ = true;
      set_auto_reanchor_integrity_inhibit_locked(reason, source_stamp);
    }
    enter_hold(reason);
  }

  bool note_soft_odom_timing_fault_locked(
    const std::string & reason,
    const double value,
    const rclcpp::Time & source_stamp,
    const std::chrono::steady_clock::time_point & now,
    int * sample_count,
    double * peak_value,
    double * duration_sec)
  {
    if (!have_odom_soft_timing_fault_window_ || odom_soft_timing_fault_reason_ != reason) {
      have_odom_soft_timing_fault_window_ = true;
      odom_soft_timing_fault_started_at_ = now;
      odom_soft_timing_fault_samples_ = 0;
      odom_soft_timing_fault_peak_ = 0.0;
      odom_soft_timing_fault_reason_ = reason;
      have_odom_soft_timing_fault_last_stamp_ = false;
    }
    if (!have_odom_soft_timing_fault_last_stamp_ ||
      source_stamp != odom_soft_timing_fault_last_stamp_)
    {
      ++odom_soft_timing_fault_samples_;
      odom_soft_timing_fault_last_stamp_ = source_stamp;
      have_odom_soft_timing_fault_last_stamp_ = true;
    }
    odom_soft_timing_fault_peak_ = std::max(odom_soft_timing_fault_peak_, value);
    *sample_count = odom_soft_timing_fault_samples_;
    *peak_value = odom_soft_timing_fault_peak_;
    *duration_sec = std::chrono::duration<double>(
      now - odom_soft_timing_fault_started_at_).count();
    if (*duration_sec < odom_timing_soft_fault_min_duration_sec_) {
      return false;
    }

    odom_timing_hold_ = true;
    set_auto_reanchor_integrity_inhibit_locked(
      "odom_soft_" + reason, source_stamp);
    odom_healthy_ticks_ = 0;
    reset_pending_locked();
    return true;
  }

  int clear_soft_odom_timing_fault_locked(
    std::string * reason,
    double * peak_value)
  {
    const int cleared_samples = odom_soft_timing_fault_samples_;
    *reason = odom_soft_timing_fault_reason_;
    *peak_value = odom_soft_timing_fault_peak_;
    odom_soft_timing_fault_samples_ = 0;
    odom_soft_timing_fault_peak_ = 0.0;
    odom_soft_timing_fault_reason_.clear();
    have_odom_soft_timing_fault_window_ = false;
    have_odom_soft_timing_fault_last_stamp_ = false;
    return cleared_samples;
  }

  void monitor_amcl_freshness(const std::chrono::steady_clock::time_point & steady_now)
  {
    bool silence_fault = false;
    double received_age_sec = std::numeric_limits<double>::infinity();
    double validated_age_sec = std::numeric_limits<double>::infinity();
    double accepted_age_sec = std::numeric_limits<double>::infinity();
    double since_motion_crossing_sec = std::numeric_limits<double>::infinity();
    double crossing_translation = 0.0;
    double crossing_yaw = 0.0;
    std::int64_t received_stamp_ns = 0;
    std::int64_t validated_stamp_ns = 0;
    std::int64_t accepted_stamp_ns = 0;
    std::int64_t crossing_odom_stamp_ns = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_amcl_receipt_) {
        received_age_sec = std::chrono::duration<double>(
          steady_now - last_amcl_receipt_).count();
        received_stamp_ns = last_received_amcl_stamp_.nanoseconds();
      }
      if (have_validated_amcl_receipt_) {
        validated_age_sec = std::chrono::duration<double>(
          steady_now - last_validated_amcl_receipt_).count();
        validated_stamp_ns = last_validated_amcl_stamp_.nanoseconds();
      }
      if (have_accepted_amcl_receipt_) {
        accepted_age_sec = std::chrono::duration<double>(
          steady_now - last_accepted_amcl_receipt_).count();
        accepted_stamp_ns = last_accepted_amcl_stamp_.nanoseconds();
      }
      if (have_amcl_motion_threshold_crossing_) {
        since_motion_crossing_sec = std::chrono::duration<double>(
          steady_now - amcl_motion_threshold_crossed_steady_).count();
        crossing_translation = amcl_motion_crossing_translation_;
        crossing_yaw = amcl_motion_crossing_yaw_;
        crossing_odom_stamp_ns = amcl_motion_crossing_odom_stamp_.nanoseconds();
      }

      if (have_trusted_transform_ && have_validated_amcl_receipt_ &&
        odom_moved_since_last_amcl_ && amcl_silence_timeout_while_moving_sec_ > 0.0)
      {
        const bool motion_grace_elapsed = !have_amcl_motion_threshold_crossing_ ||
          since_motion_crossing_sec > amcl_silence_motion_grace_sec_;
        if (validated_age_sec > amcl_silence_timeout_while_moving_sec_ &&
          motion_grace_elapsed)
        {
          silence_fault = !amcl_timing_hold_;
          amcl_timing_hold_ = true;
          set_auto_reanchor_integrity_inhibit_locked(
            "amcl_silence_while_moving", last_validated_amcl_stamp_);
          anomaly_active_ = true;
          reset_pending_locked();
        }
      }
    }
    if (silence_fault) {
      publish_current_anomaly();
      RCLCPP_ERROR(
        get_logger(),
        "AMCL silence HOLD: basic-valid AMCL age %.3f s exceeded %.3f s and odom first crossed "
        "the motion threshold %.3f s ago (grace %.3f s, crossing %.3f m / %.3f rad, "
        "odom_stamp_ns=%lld). Ages are distinct: last received %.3f s (stamp_ns=%lld), "
        "last basic timing/TF validated %.3f s (stamp_ns=%lld; fault clock), last trusted "
        "commit %.3f s (stamp_ns=%lld).",
        validated_age_sec, amcl_silence_timeout_while_moving_sec_,
        since_motion_crossing_sec, amcl_silence_motion_grace_sec_,
        crossing_translation, crossing_yaw,
        static_cast<long long>(crossing_odom_stamp_ns), received_age_sec,
        static_cast<long long>(received_stamp_ns), validated_age_sec,
        static_cast<long long>(validated_stamp_ns), accepted_age_sec,
        static_cast<long long>(accepted_stamp_ns));
    }
  }

  void request_nomotion_update_if_needed()
  {
    bool should_publish_auto_prior = false;
    bool should_global_localize = false;
    bool bootstrap_retry_scheduled = false;
    bool bootstrap_retry_started = false;
    double prior_x = 0.0;
    double prior_y = 0.0;
    double prior_yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto steady_now = std::chrono::steady_clock::now();
      const bool dependencies_ready = static_cast<bool>(map_distance_field_) &&
        !scan_buffer_.empty() && have_new_odom_observation_ && !odom_timing_hold_ &&
        !timer_health_hold_ && recovery_is_stationary_locked(steady_now);
      const bool bootstrap_cycle_exhausted = global_localization_completed_ &&
        !have_trusted_transform_ &&
        (std::chrono::duration<double>(
          steady_now - localization_epoch_steady_).count() > bootstrap_timeout_sec_ ||
        bootstrap_nomotion_requests_sent_ >= bootstrap_max_nomotion_requests_);
      if (bootstrap_cycle_exhausted && !bootstrap_retry_pending_) {
        bootstrap_retry_pending_ = true;
        bootstrap_retry_after_ = steady_now +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(bootstrap_retry_cooldown_sec_));
        bootstrap_retry_scheduled = true;
      }
      if (bootstrap_retry_pending_ && steady_now >= bootstrap_retry_after_ &&
        dependencies_ready)
      {
        // Start a fresh bounded acquisition epoch.  Failure remains fail-closed,
        // but a transient dropped prior/scan cannot leave the guard permanently
        // unarmed until the whole process is restarted.
        bootstrap_retry_pending_ = false;
        global_localization_completed_ = false;
        global_localization_in_flight_ = false;
        nomotion_request_in_flight_ = false;
        waiting_for_nomotion_pose_ = false;
        bootstrap_post_global_updates_ = 0;
        bootstrap_nomotion_requests_sent_ = 0;
        bootstrap_samples_.clear();
        reset_pending_locked();
        bootstrap_retry_started = true;
      }
      const bool use_global_localization =
        auto_global_localization_ || force_global_localization_for_epoch_;
      should_publish_auto_prior = auto_initial_pose_enabled_ &&
        !use_global_localization && !have_trusted_transform_ &&
        !global_localization_completed_ && dependencies_ready;
      should_global_localize = use_global_localization && !have_trusted_transform_ &&
        !global_localization_completed_ && !global_localization_in_flight_ &&
        dependencies_ready;
      prior_x = epoch_initial_pose_x_;
      prior_y = epoch_initial_pose_y_;
      prior_yaw = epoch_initial_pose_yaw_;
    }
    if (bootstrap_retry_scheduled) {
      RCLCPP_ERROR(
        get_logger(),
        "Automatic AMCL acquisition exhausted its bounded attempt; guard remains HOLD and will "
        "retry after a %.1f s cooldown.", bootstrap_retry_cooldown_sec_);
    }
    if (bootstrap_retry_started) {
      RCLCPP_WARN(
        get_logger(),
        "Starting a new bounded automatic AMCL acquisition epoch after cooldown; no prior "
        "candidate or transform was accepted during the failed epoch.");
    }
    if (should_publish_auto_prior) {
      const std::size_t required_initialpose_subscriptions = allow_manual_rebase_ ? 2U : 1U;
      if (!nomotion_update_client_->service_is_ready() ||
        initialpose_publisher_->get_subscription_count() < required_initialpose_subscriptions)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for AMCL /initialpose subscription and services before publishing the "
          "automatic coarse localization prior.");
        return;
      }
      geometry_msgs::msg::PoseWithCovarianceStamped prior;
      prior.header.stamp = get_clock()->now();
      prior.header.frame_id = global_frame_;
      prior.pose.pose.position.x = prior_x;
      prior.pose.pose.position.y = prior_y;
      prior.pose.pose.position.z = 0.0;
      tf2::Quaternion rotation;
      rotation.setRPY(0.0, 0.0, prior_yaw);
      prior.pose.pose.orientation = tf2::toMsg(rotation);
      prior.pose.covariance[0] = auto_initial_pose_variance_x_;
      prior.pose.covariance[7] = auto_initial_pose_variance_y_;
      prior.pose.covariance[35] = auto_initial_pose_variance_yaw_;
      initialpose_publisher_->publish(prior);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        global_localization_completed_ = true;
        localization_epoch_steady_ = std::chrono::steady_clock::now();
        localization_stamp_floor_ = have_seen_amcl_stamp_ ?
          last_seen_amcl_stamp_ : rclcpp::Time(0, 0, RCL_ROS_TIME);
        bootstrap_post_global_updates_ = 0;
        bootstrap_nomotion_requests_sent_ = 0;
        bootstrap_samples_.clear();
        reset_pending_locked();
      }
      RCLCPP_WARN(
        get_logger(),
        "Published automatic AMCL coarse prior (%.3f, %.3f, %.3f rad; variances "
        "%.3f, %.3f, %.4f). This is not the guard anchor; scan-map/covariance "
        "confirmation must still pass.",
        prior_x, prior_y, prior_yaw,
        auto_initial_pose_variance_x_, auto_initial_pose_variance_y_,
        auto_initial_pose_variance_yaw_);
      return;
    }
    if (should_global_localize) {
      if (!global_localization_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for %s before automatic AMCL acquisition can start.",
          global_localization_service_.c_str());
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (global_localization_in_flight_ || global_localization_completed_) {
          return;
        }
        global_localization_in_flight_ = true;
      }
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      global_localization_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
          bool success = true;
          try {
            (void)future.get();
          } catch (const std::exception & exception) {
            success = false;
            RCLCPP_ERROR(
              get_logger(), "Automatic global-localization request failed: %s",
              exception.what());
          }
          {
            std::lock_guard<std::mutex> lock(mutex_);
            global_localization_in_flight_ = false;
            if (success) {
              global_localization_completed_ = true;
              localization_epoch_steady_ = std::chrono::steady_clock::now();
              localization_stamp_floor_ = have_seen_amcl_stamp_ ?
                last_seen_amcl_stamp_ : rclcpp::Time(0, 0, RCL_ROS_TIME);
              bootstrap_post_global_updates_ = 0;
              bootstrap_nomotion_requests_sent_ = 0;
              bootstrap_samples_.clear();
              reset_pending_locked();
            }
          }
          if (success) {
            RCLCPP_WARN(
              get_logger(),
              "AMCL global localization seeded automatically; guard remains HOLD until scan-map, "
              "covariance, and fixed-window confirmation all pass.");
          }
        });
      return;
    }

    bool should_request = false;
    bool bootstrap_timed_out = false;
    bool recovery_nomotion_budget_exhausted = false;
    int recovery_nomotion_requests = 0;
    double recovery_nomotion_elapsed_sec = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (global_localization_completed_ && !have_trusted_transform_) {
        bootstrap_timed_out =
          std::chrono::duration<double>(now - localization_epoch_steady_).count() >
          bootstrap_timeout_sec_;
      }
      if (waiting_for_nomotion_pose_ &&
        std::chrono::duration<double>(now - last_nomotion_request_steady_).count() > 0.75)
      {
        // The Empty service only sets a boolean in AMCL.  If a pose was not
        // produced, permit a bounded retry rather than counting the response.
        waiting_for_nomotion_pose_ = false;
      }
      if (!anomaly_active_ && have_recovery_nomotion_window_) {
        reset_recovery_nomotion_budget_locked();
      }
      bool recovery_nomotion_allowed = have_trusted_transform_ && anomaly_active_ &&
        !explicit_rebase_required_ && !auto_reanchor_integrity_inhibit_ &&
        !recovery_cluster_reported_;
      if (recovery_nomotion_allowed) {
        if (!have_recovery_nomotion_window_) {
          recovery_nomotion_window_started_ = now;
          have_recovery_nomotion_window_ = true;
          recovery_nomotion_requests_sent_ = 0;
        }
        recovery_nomotion_elapsed_sec = std::chrono::duration<double>(
          now - recovery_nomotion_window_started_).count();
        if (recovery_nomotion_requests_sent_ >= recovery_nomotion_max_requests_ ||
          recovery_nomotion_elapsed_sec >= recovery_nomotion_max_window_sec_)
        {
          recovery_cluster_reported_ = true;
          recovery_nomotion_allowed = false;
          recovery_nomotion_budget_exhausted = true;
          recovery_nomotion_requests = recovery_nomotion_requests_sent_;
        }
      }
      should_request = nomotion_update_period_sec_ > 0.0 &&
        !nomotion_request_in_flight_ && !waiting_for_nomotion_pose_ &&
        (rebase_requested_ ||
        (!bootstrap_requires_initialpose_ && !have_trusted_transform_ &&
        global_localization_completed_ && !bootstrap_timed_out &&
        bootstrap_nomotion_requests_sent_ < bootstrap_max_nomotion_requests_) ||
        recovery_nomotion_allowed);
    }
    if (bootstrap_timed_out) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Automatic AMCL acquisition exceeded %.1f s; guard remains HOLD and will not guess an anchor.",
        bootstrap_timeout_sec_);
    }
    if (recovery_nomotion_budget_exhausted) {
      RCLCPP_ERROR(
        get_logger(),
        "Bounded AMCL anomaly-recovery no-motion window exhausted after %d requests / %.3f s "
        "(limits %d / %.3f s). Further forced updates are disabled for this HOLD epoch; "
        "the trusted transform remains frozen.",
        recovery_nomotion_requests, recovery_nomotion_elapsed_sec,
        recovery_nomotion_max_requests_, recovery_nomotion_max_window_sec_);
    }
    if (!should_request) {
      return;
    }
    if (!nomotion_update_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for %s while guard confirmation needs fresh AMCL observations.",
        nomotion_update_service_.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (nomotion_request_in_flight_) {
        return;
      }
      nomotion_request_in_flight_ = true;
      waiting_for_nomotion_pose_ = true;
      last_nomotion_request_steady_ = std::chrono::steady_clock::now();
      nomotion_request_stamp_floor_ = have_seen_amcl_stamp_ ?
        last_seen_amcl_stamp_ : rclcpp::Time(0, 0, RCL_ROS_TIME);
      if (!have_trusted_transform_) {
        ++bootstrap_nomotion_requests_sent_;
      } else if (anomaly_active_) {
        ++recovery_nomotion_requests_sent_;
      }
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    nomotion_update_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        std::lock_guard<std::mutex> lock(mutex_);
        nomotion_request_in_flight_ = false;
      });
  }

  void on_publish_timer()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    (void)monitor_publish_timer_health();
    (void)monitor_odom_integrity(steady_now);
    monitor_amcl_freshness(steady_now);
    monitor_cumulative_overrun_defer(steady_now);
    monitor_trusted_override_active_evidence(steady_now);
    monitor_trusted_override_verification(steady_now);
    publish_current_anomaly();
    publish_guard_mode();
    publish_last_trusted_transform();
  }

  bool monitor_publish_timer_health()
  {
    const auto now = std::chrono::steady_clock::now();
    double gap_sec = 0.0;
    bool entered_hold = false;
    bool recovered = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_timer_tick_) {
        have_timer_tick_ = true;
        last_timer_tick_ = now;
        timer_healthy_ticks_ = 1;
      } else {
        gap_sec = std::chrono::duration<double>(now - last_timer_tick_).count();
        last_timer_tick_ = now;
        if (max_publish_timer_gap_sec_ > 0.0 && gap_sec > max_publish_timer_gap_sec_) {
          entered_hold = !timer_health_hold_;
          timer_health_hold_ = true;
          set_auto_reanchor_integrity_inhibit_locked(
            "publish_timer_callback_gap", rclcpp::Time(0, 0, RCL_ROS_TIME));
          timer_healthy_ticks_ = 0;
        } else if (timer_health_hold_) {
          ++timer_healthy_ticks_;
          if (timer_healthy_ticks_ >= timer_recovery_consecutive_ticks_) {
            timer_health_hold_ = false;
            recovered = true;
          }
        }
      }
    }
    if (entered_hold) {
      RCLCPP_ERROR(
        get_logger(),
        "TF publish timer timing HOLD: callback gap %.3f s exceeded %.3f s. "
        "The last trusted transform will remain published while anomaly stays asserted.",
        gap_sec, max_publish_timer_gap_sec_);
    } else if (recovered) {
      RCLCPP_INFO(
        get_logger(), "TF publish timer recovered after %d consecutive healthy ticks.",
        timer_recovery_consecutive_ticks_);
    }
    return true;
  }

  bool monitor_odom_integrity(const std::chrono::steady_clock::time_point & steady_now)
  {
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
      odom_to_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Duration::from_seconds(0.02));
    } catch (const tf2::TransformException &) {
      bool hard_fault = false;
      bool confirmed_soft_fault = false;
      int soft_samples = 0;
      double soft_peak_sec = 0.0;
      double soft_duration_sec = 0.0;
      double unavailable_gap_sec = 0.0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_new_odom_observation_) {
          // There is no trusted timing baseline to debounce against.  Startup
          // already begins fail-closed, so keep that invariant explicit.
          hard_fault = true;
        } else {
          unavailable_gap_sec = std::chrono::duration<double>(
            steady_now - last_new_odom_observation_steady_).count();
          const auto severity = classify_odom_timing_value(
            unavailable_gap_sec, max_odom_observation_gap_sec_,
            hard_odom_observation_gap_sec_);
          hard_fault = severity == OdomTimingSeverity::kHard;
          if (severity == OdomTimingSeverity::kSoft) {
            confirmed_soft_fault = note_soft_odom_timing_fault_locked(
              "transform_unavailable", unavailable_gap_sec, last_new_odom_stamp_, steady_now,
              &soft_samples, &soft_peak_sec, &soft_duration_sec);
          }
        }
        if (hard_fault) {
          odom_timing_hold_ = true;
          set_auto_reanchor_integrity_inhibit_locked(
            "odom_transform_unavailable", have_new_odom_observation_ ?
            last_new_odom_stamp_ : rclcpp::Time(0, 0, RCL_ROS_TIME));
          odom_healthy_ticks_ = 0;
          reset_pending_locked();
        }
      }
      publish_current_anomaly();
      if (hard_fault || confirmed_soft_fault) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Odom timing HOLD (transform_unavailable): lookup unavailable for %.3f s; "
          "soft/hard limits %.3f/%.3f s, distinct stamps %d (legacy target %d), "
          "duration %.3f/%.3f s (peak %.3f s). "
          "Fault class=%s; retaining the last trusted map->odom.",
          unavailable_gap_sec, max_odom_observation_gap_sec_,
          hard_odom_observation_gap_sec_, soft_samples,
          odom_timing_soft_fault_consecutive_samples_, soft_duration_sec,
          odom_timing_soft_fault_min_duration_sec_, soft_peak_sec,
          hard_fault ? "hard-immediate" : "soft-confirmed");
      } else if (soft_samples > 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Odom timing debounce (transform_unavailable): gap %.3f s exceeded soft %.3f s "
          "but is below hard %.3f s; distinct stamps %d (legacy target %d), "
          "duration %.3f/%.3f s, peak %.3f s. Candidate commits are "
          "frozen; this soft sample does not newly assert HOLD, and any existing HOLD remains.",
          unavailable_gap_sec, max_odom_observation_gap_sec_,
          hard_odom_observation_gap_sec_, soft_samples,
          odom_timing_soft_fault_consecutive_samples_, soft_duration_sec,
          odom_timing_soft_fault_min_duration_sec_, soft_peak_sec);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Latest odom->base transform temporarily unavailable below the %.3f s soft gap; "
          "retaining the last trusted map->odom.",
          max_odom_observation_gap_sec_);
      }
      return true;
    }

    const tf2::Transform current_odom_base = transform_from_msg(odom_to_base.transform);
    if (!transform_is_finite(current_odom_base)) {
      enter_hold("invalid_odom_transform");
      return true;
    }

    const rclcpp::Time odom_stamp(odom_to_base.header.stamp);
    if (odom_stamp.nanoseconds() == 0) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_timing_hold_ = true;
        set_auto_reanchor_integrity_inhibit_locked("odom_zero_source_stamp", odom_stamp);
        odom_healthy_ticks_ = 0;
        reset_pending_locked();
      }
      publish_current_anomaly();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Odom timing HOLD (zero_source_stamp): the odom->base sample has no usable source "
        "time; retaining the last trusted map->odom.");
      return true;
    }
    const double odom_age_sec = (get_clock()->now() - odom_stamp).seconds();
    if (odom_age_sec < -max_future_pose_skew_sec_) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_timing_hold_ = true;
        set_auto_reanchor_integrity_inhibit_locked("odom_future_source_stamp", odom_stamp);
        odom_healthy_ticks_ = 0;
        reset_pending_locked();
      }
      publish_current_anomaly();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Odom timing HOLD (future_source_stamp): transform age %.3f s is below -%.3f s "
        "(source_stamp_ns=%lld); retaining the last trusted map->odom.",
        odom_age_sec, max_future_pose_skew_sec_,
        static_cast<long long>(odom_stamp.nanoseconds()));
      return true;
    }

    const auto source_age_severity = classify_odom_timing_value(
      odom_age_sec, max_odom_tf_age_sec_, hard_odom_tf_age_sec_);
    if (source_age_severity != OdomTimingSeverity::kHealthy) {
      bool confirmed_soft_fault = false;
      int soft_samples = 0;
      double soft_peak_sec = 0.0;
      double soft_duration_sec = 0.0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (source_age_severity == OdomTimingSeverity::kHard) {
          odom_timing_hold_ = true;
          set_auto_reanchor_integrity_inhibit_locked("odom_hard_source_age", odom_stamp);
          odom_healthy_ticks_ = 0;
          reset_pending_locked();
        } else {
          confirmed_soft_fault = note_soft_odom_timing_fault_locked(
            "source_age", odom_age_sec, odom_stamp, steady_now, &soft_samples,
            &soft_peak_sec, &soft_duration_sec);
        }
      }
      if (source_age_severity == OdomTimingSeverity::kHard || confirmed_soft_fault) {
        publish_current_anomaly();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Odom timing HOLD (source_age): age %.3f s; soft/hard limits %.3f/%.3f s, "
          "distinct stamps %d (legacy target %d), duration %.3f/%.3f s (peak %.3f s), "
          "source_stamp_ns=%lld. Fault class=%s; "
          "retaining the last trusted map->odom.",
          odom_age_sec, max_odom_tf_age_sec_, hard_odom_tf_age_sec_, soft_samples,
          odom_timing_soft_fault_consecutive_samples_, soft_duration_sec,
          odom_timing_soft_fault_min_duration_sec_, soft_peak_sec,
          static_cast<long long>(odom_stamp.nanoseconds()),
          source_age_severity == OdomTimingSeverity::kHard ?
          "hard-immediate" : "soft-confirmed");
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Odom timing debounce (source_age): age %.3f s exceeded soft %.3f s but is below "
          "hard %.3f s; distinct stamps %d (legacy target %d), duration %.3f/%.3f s, "
          "peak %.3f s, source_stamp_ns=%lld. Candidate "
          "commits are frozen; this soft sample does not newly assert HOLD, and any existing "
          "HOLD remains.",
          odom_age_sec, max_odom_tf_age_sec_, hard_odom_tf_age_sec_, soft_samples,
          odom_timing_soft_fault_consecutive_samples_, soft_duration_sec,
          odom_timing_soft_fault_min_duration_sec_, soft_peak_sec,
          static_cast<long long>(odom_stamp.nanoseconds()));
      }
      return true;
    }

    bool jump_detected = false;
    bool large_step_allowed_by_kinematics = false;
    bool timing_fault = false;
    bool timing_hard_fault = false;
    bool timing_soft_violation = false;
    bool timing_soft_confirmed = false;
    bool timing_recovered = false;
    bool new_observation = false;
    double observation_receipt_gap_sec = 0.0;
    double timing_value_sec = 0.0;
    double timing_soft_limit_sec = 0.0;
    double timing_hard_limit_sec = 0.0;
    double timing_soft_peak_sec = 0.0;
    double timing_soft_duration_sec = 0.0;
    double cleared_soft_peak_sec = 0.0;
    int timing_soft_samples = 0;
    int cleared_soft_samples = 0;
    std::string timing_reason;
    std::string cleared_soft_reason;
    rclcpp::Time previous_observation_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time previous_integrity_stamp(0, 0, RCL_ROS_TIME);
    CandidateMetrics metrics;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool recovering_from_odom_timing_hold = odom_timing_hold_;
      if (have_new_odom_observation_) {
        previous_observation_stamp = last_new_odom_stamp_;
        observation_receipt_gap_sec = std::chrono::duration<double>(
          steady_now - last_new_odom_observation_steady_).count();
        metrics.dt = (odom_stamp - last_new_odom_stamp_).seconds();
      }

      const auto consider_gap =
        [&](const std::string & reason, const double value) {
          const auto severity = classify_odom_timing_value(
            value, max_odom_observation_gap_sec_, hard_odom_observation_gap_sec_);
          if (severity == OdomTimingSeverity::kHard) {
            timing_fault = true;
            timing_hard_fault = true;
            timing_reason = reason;
            timing_value_sec = value;
            timing_soft_limit_sec = max_odom_observation_gap_sec_;
            timing_hard_limit_sec = hard_odom_observation_gap_sec_;
          } else if (severity == OdomTimingSeverity::kSoft && !timing_hard_fault) {
            timing_soft_violation = true;
            timing_reason = reason;
            timing_value_sec = value;
            timing_soft_limit_sec = max_odom_observation_gap_sec_;
            timing_hard_limit_sec = hard_odom_observation_gap_sec_;
          }
        };

      if (have_new_odom_observation_ && odom_stamp < last_new_odom_stamp_) {
        timing_fault = true;
        timing_hard_fault = true;
        timing_reason = "non_monotonic_source_stamp";
      } else if (have_new_odom_observation_ && odom_stamp == last_new_odom_stamp_) {
        consider_gap("duplicate_source_stamp_timeout", observation_receipt_gap_sec);
      } else {
        new_observation = true;
        if (have_new_odom_observation_) {
          consider_gap("observation_receipt_gap", observation_receipt_gap_sec);
        }
        if (have_odom_baseline_) {
          previous_integrity_stamp = last_odom_stamp_;
          metrics.dt = (odom_stamp - last_odom_stamp_).seconds();
          if (metrics.dt <= 0.0) {
            timing_fault = true;
            timing_hard_fault = true;
            timing_soft_violation = false;
            timing_reason = "non_positive_integrity_source_dt";
            timing_value_sec = metrics.dt;
            timing_soft_limit_sec = 0.0;
            timing_hard_limit_sec = 0.0;
          } else {
            consider_gap("source_stamp_gap", metrics.dt);
          }
        }
        last_new_odom_stamp_ = odom_stamp;
        last_new_odom_observation_steady_ = steady_now;
        have_new_odom_observation_ = true;
      }

      if (!timing_fault && timing_soft_violation) {
        timing_soft_confirmed = note_soft_odom_timing_fault_locked(
          timing_reason, timing_value_sec, odom_stamp, steady_now, &timing_soft_samples,
          &timing_soft_peak_sec, &timing_soft_duration_sec);
        timing_fault = timing_soft_confirmed;
      }
      if (timing_fault) {
        odom_timing_hold_ = true;
        set_auto_reanchor_integrity_inhibit_locked(
          "odom_" + timing_reason, odom_stamp);
        odom_healthy_ticks_ = 0;
        reset_pending_locked();
      } else if (new_observation && !timing_soft_violation &&
        (odom_stamp > last_integrity_odom_stamp_ || !have_integrity_odom_stamp_))
      {
        cleared_soft_samples = clear_soft_odom_timing_fault_locked(
          &cleared_soft_reason, &cleared_soft_peak_sec);
        last_integrity_odom_stamp_ = odom_stamp;
        have_integrity_odom_stamp_ = true;
        ++odom_healthy_ticks_;
        if (odom_timing_hold_ && odom_healthy_ticks_ >= timer_recovery_consecutive_ticks_) {
          odom_timing_hold_ = false;
          timing_recovered = true;
        }
      }

      if (new_observation && !have_odom_baseline_) {
        last_odom_base_ = current_odom_base;
        last_odom_stamp_ = odom_stamp;
        have_odom_baseline_ = true;
        odom_baseline_started_ = steady_now;
        motion_reference_odom_base_ = current_odom_base;
        have_motion_reference_ = true;
        stationary_reference_odom_base_ = current_odom_base;
        have_stationary_reference_ = true;
      } else if (new_observation) {
        const tf2::Transform previous_odom_base = last_odom_base_;
        const tf2::Vector3 displacement =
          current_odom_base.getOrigin() - previous_odom_base.getOrigin();
        metrics.translation = std::hypot(displacement.x(), displacement.y());
        metrics.yaw_delta = std::abs(
          normalize_angle(yaw_of(current_odom_base) - yaw_of(previous_odom_base)));
        metrics.kinematic_linear_limit =
          linear_slack_ + max_linear_speed_ * metrics.dt +
          0.5 * max_linear_acceleration_ * metrics.dt * metrics.dt;
        metrics.kinematic_angular_limit =
          angular_slack_ + max_angular_speed_ * metrics.dt +
          0.5 * max_angular_acceleration_ * metrics.dt * metrics.dt;
        metrics.linear_limit = allowed_odom_step(
          metrics.dt, max_linear_speed_, max_linear_acceleration_,
          linear_slack_, max_odom_step_translation_);
        metrics.angular_limit = allowed_odom_step(
          metrics.dt, max_angular_speed_, max_angular_acceleration_,
          angular_slack_, max_odom_step_yaw_);

        last_odom_base_ = current_odom_base;
        last_odom_stamp_ = odom_stamp;
        jump_detected =
          have_trusted_transform_ && !timing_fault &&
          !recovering_from_odom_timing_hold &&
          (metrics.translation > metrics.linear_limit ||
          metrics.yaw_delta > metrics.angular_limit);
        large_step_allowed_by_kinematics =
          have_trusted_transform_ && !timing_fault &&
          !recovering_from_odom_timing_hold && !jump_detected &&
          (metrics.translation > max_odom_step_translation_ ||
          metrics.yaw_delta > max_odom_step_yaw_);

        if (!timing_fault && !timing_soft_violation &&
          !recovering_from_odom_timing_hold && !jump_detected &&
          (metrics.translation >= rolling_anchor_odom_translation_epsilon_ ||
          metrics.yaw_delta >= rolling_anchor_odom_yaw_epsilon_))
        {
          // Only source-monotonic, timing-healthy, physically plausible odom
          // segments enlarge the dynamic rolling budget.  This path is never
          // fed by duplicate stamps, timer repeats, or rejected jumps.
          OdomMotionSegment motion;
          motion.receipt = steady_now;
          motion.translation = metrics.translation;
          motion.yaw = metrics.yaw_delta;
          rolling_anchor_odom_motion_.push_back(motion);
          prune_rolling_anchor_window_locked(steady_now);
        }

        if (!timing_fault && !timing_soft_violation && have_motion_reference_) {
          const tf2::Vector3 motion_delta =
            current_odom_base.getOrigin() - motion_reference_odom_base_.getOrigin();
          // AMCL's update_min_d gate is axis based.  Use the same max-axis norm
          // here so diagonal motion below AMCL's own x/y thresholds cannot be
          // misclassified as an AMCL-silence fault.
          const double motion_translation = std::max(
            std::abs(motion_delta.x()), std::abs(motion_delta.y()));
          const double motion_yaw = std::abs(normalize_angle(
            yaw_of(current_odom_base) - yaw_of(motion_reference_odom_base_)));
          // AMCL intentionally remains silent below update_min_d/update_min_a.
          // Only declare a missing-AMCL fault once odom has crossed a budget
          // larger than those configured update thresholds.
          if (motion_translation > amcl_silence_motion_translation_ ||
            motion_yaw > amcl_silence_motion_yaw_)
          {
            if (!odom_moved_since_last_amcl_) {
              odom_moved_since_last_amcl_ = true;
              have_amcl_motion_threshold_crossing_ = true;
              amcl_motion_threshold_crossed_steady_ = steady_now;
              amcl_motion_crossing_odom_stamp_ = odom_stamp;
              amcl_motion_crossing_translation_ = motion_translation;
              amcl_motion_crossing_yaw_ = motion_yaw;
            }
          }
        }

        if (!timing_fault && !timing_soft_violation && have_stationary_reference_) {
          const tf2::Vector3 stationary_delta =
            current_odom_base.getOrigin() - stationary_reference_odom_base_.getOrigin();
          const double stationary_translation =
            std::hypot(stationary_delta.x(), stationary_delta.y());
          const double stationary_yaw = std::abs(normalize_angle(
            yaw_of(current_odom_base) - yaw_of(stationary_reference_odom_base_)));
          if (stationary_translation > 0.01 || stationary_yaw > 0.01) {
            last_odom_motion_ = steady_now;
            have_odom_motion_ = true;
            stationary_reference_odom_base_ = current_odom_base;
          }
        }

        if (jump_detected) {
          // Never absorb an odom reset into map->odom.  The former full
          // compensation path could move the guarded transform without limit.
          // The old map->odom is no longer meaningful for the new odom epoch,
          // so stop publishing and require a complete automatic localization
          // quality cycle before authoring a new fixed anchor.
          // Preserve the last globally trusted robot pose as the automatic
          // coarse prior for the new odom epoch.  A full-map uniform reset is
          // deliberately avoided: with a static robot and a limited particle
          // budget it is both expensive and prone to map aliases.  This prior
          // still has non-zero covariance and is never committed directly.
          epoch_initial_pose_x_ = last_trusted_map_base_.getOrigin().x();
          epoch_initial_pose_y_ = last_trusted_map_base_.getOrigin().y();
          epoch_initial_pose_yaw_ = yaw_of(last_trusted_map_base_);
          anomaly_active_ = true;
          explicit_rebase_required_ = false;
          have_trusted_transform_ = false;
          have_authorized_anchor_ = false;
          have_last_published_safe_transform_ = false;
          suppress_tf_output_ = true;
          global_localization_completed_ = false;
          global_localization_in_flight_ = false;
          force_global_localization_for_epoch_ = false;
          bootstrap_post_global_updates_ = 0;
          bootstrap_nomotion_requests_sent_ = 0;
          bootstrap_samples_.clear();
          reset_pending_locked();
        }
      }
    }

    if (jump_detected) {
      publish_anomaly(true);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Odom integrity HOLD (jump_detected): source stamps %lld -> %lld ns, real dt %.6f s; "
        "odom->base step %.3f m / %.3f rad exceeded effective limits %.3f / %.3f "
        "(kinematic %.3f / %.3f, discontinuity floors %.3f / %.3f). TF publication is "
        "stopped and a full "
        "automatic coarse-prior localization quality cycle is required for the new odom epoch.",
        static_cast<long long>(previous_integrity_stamp.nanoseconds()),
        static_cast<long long>(odom_stamp.nanoseconds()), metrics.dt,
        metrics.translation, metrics.yaw_delta,
        metrics.linear_limit, metrics.angular_limit,
        metrics.kinematic_linear_limit, metrics.kinematic_angular_limit,
        max_odom_step_translation_, max_odom_step_yaw_);
    } else if (large_step_allowed_by_kinematics) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Odom large step accepted by the dt-scaled kinematic envelope: source stamps "
        "%lld -> %lld ns, real dt %.6f s, step %.3f m / %.3f rad, effective limits "
        "%.3f / %.3f (discontinuity floors %.3f / %.3f).",
        static_cast<long long>(previous_integrity_stamp.nanoseconds()),
        static_cast<long long>(odom_stamp.nanoseconds()), metrics.dt,
        metrics.translation, metrics.yaw_delta, metrics.linear_limit, metrics.angular_limit,
        max_odom_step_translation_, max_odom_step_yaw_);
    }
    if (timing_fault) {
      publish_current_anomaly();
      if (timing_hard_limit_sec > 0.0 || timing_soft_confirmed) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Odom timing HOLD (%s): source stamps %lld -> %lld ns, source dt %.6f s, "
          "receipt gap %.3f s; offending value %.3f s, soft/hard limits %.3f/%.3f s, "
          "distinct stamps %d (legacy target %d), duration %.3f/%.3f s (peak %.3f s). "
          "Fault class=%s; retaining the last "
          "trusted map->odom.",
          timing_reason.c_str(),
          static_cast<long long>(previous_observation_stamp.nanoseconds()),
          static_cast<long long>(odom_stamp.nanoseconds()), metrics.dt,
          observation_receipt_gap_sec, timing_value_sec, timing_soft_limit_sec,
          timing_hard_limit_sec, timing_soft_samples,
          odom_timing_soft_fault_consecutive_samples_, timing_soft_duration_sec,
          odom_timing_soft_fault_min_duration_sec_, timing_soft_peak_sec,
          timing_hard_fault ? "hard-immediate" : "soft-confirmed");
      } else {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Odom timing HOLD (%s): source stamps %lld -> %lld ns, source dt %.6f s, "
          "receipt gap %.3f s. Structural timestamp fault is hard-immediate; retaining "
          "the last trusted map->odom.",
          timing_reason.c_str(),
          static_cast<long long>(previous_observation_stamp.nanoseconds()),
          static_cast<long long>(odom_stamp.nanoseconds()), metrics.dt,
          observation_receipt_gap_sec);
      }
    } else if (timing_soft_violation) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Odom timing debounce (%s): source stamps %lld -> %lld ns, source dt %.6f s, "
        "receipt gap %.3f s; offending value %.3f s exceeded soft %.3f s but is below "
        "hard %.3f s, distinct stamps %d (legacy target %d), duration %.3f/%.3f s "
        "(peak %.3f s). Candidate commits and AMCL-silence/"
        "stationary state advancement are frozen while kinematic jump checking stays active; "
        "this soft sample does not newly assert HOLD, and any existing HOLD remains.",
        timing_reason.c_str(),
        static_cast<long long>(previous_observation_stamp.nanoseconds()),
        static_cast<long long>(odom_stamp.nanoseconds()), metrics.dt,
        observation_receipt_gap_sec, timing_value_sec, timing_soft_limit_sec,
        timing_hard_limit_sec, timing_soft_samples,
        odom_timing_soft_fault_consecutive_samples_, timing_soft_duration_sec,
        odom_timing_soft_fault_min_duration_sec_, timing_soft_peak_sec);
    }
    if (cleared_soft_samples > 0) {
      RCLCPP_INFO(
        get_logger(),
        "Odom timing debounce window cleared by a fresh monotonic sample after %d "
        "distinct source stamps (last reason=%s, peak %.3f s). Any confirmed timing "
        "HOLD remains fail-closed until %d healthy samples pass.",
        cleared_soft_samples, cleared_soft_reason.c_str(), cleared_soft_peak_sec,
        timer_recovery_consecutive_ticks_);
    }
    if (timing_recovered) {
      publish_current_anomaly();
      RCLCPP_INFO(
        get_logger(), "Odom timing input recovered after %d consecutive healthy samples.",
        timer_recovery_consecutive_ticks_);
    }
    return true;
  }

  bool accept_candidate(
    const tf2::Transform & map_odom,
    const tf2::Transform & map_base,
    const tf2::Transform & odom_base,
    const rclcpp::Time & stamp,
    const rclcpp::Time & odom_stamp,
    const bool reset_authorized_anchor)
  {
    (void)odom_base;
    (void)odom_stamp;
    if (odom_timing_hold_ || odom_soft_timing_fault_samples_ > 0 ||
      timer_health_hold_)
    {
      // This is a second, commit-local invariant.  Callers may collect
      // diagnostics while an input is unhealthy, but neither HOLD nor a
      // not-yet-confirmed timing debounce window may update TF.
      return false;
    }
    if (reset_authorized_anchor) {
      authorized_anchor_map_odom_ = map_odom;
      authorized_anchor_stamp_ = stamp;
      authorized_anchor_steady_ = std::chrono::steady_clock::now();
      have_authorized_anchor_steady_ = true;
      have_authorized_anchor_ = true;
      ++authorized_anchor_generation_;
      post_reanchor_verification_active_ = false;
    } else {
      CandidateMetrics anchor_metrics;
      std::string anchor_reason;
      if (!candidate_is_within_anchor_locked(
          map_odom, anomaly_active_, &anchor_metrics, &anchor_reason))
      {
        anomaly_active_ = true;
        reset_pending_locked();
        RCLCPP_ERROR(
          get_logger(),
          "Refused candidate commit (%s): cumulative map->odom drift %.3f m / %.3f rad "
          "from authorized anchor generation %llu.",
          anchor_reason.c_str(), anchor_metrics.cumulative_translation,
          anchor_metrics.cumulative_yaw,
          static_cast<unsigned long long>(authorized_anchor_generation_));
        return false;
      }
    }

    last_trusted_map_odom_ = map_odom;
    last_trusted_map_base_ = map_base;
    last_trusted_stamp_ = stamp;
    last_accepted_amcl_stamp_ = stamp;
    have_trusted_transform_ = true;
    // The integrity baseline is maintained exclusively from the newest odom
    // observations in monitor_odom_integrity().  Never rewind it to the
    // historical odom sample paired with a delayed AMCL pose.
    suppress_tf_output_ = false;
    clear_trusted_override_locked();
    reset_pending_locked();
    return true;
  }

  void publish_last_trusted_transform()
  {
    tf2::Transform trusted_transform;
    bool anchor_invariant_fault = false;
    bool can_publish = false;
    double invariant_translation = 0.0;
    double invariant_yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_trusted_transform_ || suppress_tf_output_) {
        return;
      }
      trusted_transform = last_trusted_map_odom_;
      can_publish = true;
      if (have_authorized_anchor_) {
        const tf2::Transform delta =
          authorized_anchor_map_odom_.inverse() * last_trusted_map_odom_;
        invariant_translation = std::hypot(delta.getOrigin().x(), delta.getOrigin().y());
        invariant_yaw = std::abs(normalize_angle(yaw_of(delta)));
        if (invariant_translation > max_cumulative_map_odom_translation_ ||
          invariant_yaw > max_cumulative_map_odom_yaw_)
        {
          // Last-resort invariant: even if a future call path forgets the
          // commit gate, never put an out-of-envelope transform on /tf.
          anchor_invariant_fault = true;
          anomaly_active_ = true;
          if (have_last_published_safe_transform_) {
            trusted_transform = last_published_safe_map_odom_;
          } else {
            can_publish = false;
          }
        }
      }
      if (!anchor_invariant_fault) {
        last_published_safe_map_odom_ = trusted_transform;
        have_last_published_safe_transform_ = true;
      }
    }

    if (anchor_invariant_fault) {
      publish_current_anomaly();
      RCLCPP_FATAL_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Fixed-anchor invariant prevented publishing cumulative map->odom drift "
        "%.3f m / %.3f rad; retaining the last transform that already passed the invariant.",
        invariant_translation, invariant_yaw);
    }

    if (!can_publish) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = global_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform = tf2::toMsg(trusted_transform);
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_anomaly(const bool anomaly)
  {
    std_msgs::msg::Bool message;
    message.data = anomaly;
    anomaly_publisher_->publish(message);
  }

  void publish_current_anomaly()
  {
    bool anomaly = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Every input/timer fault is fail-closed.  HOLD retains the last trusted
      // transform but keeps navigation consumers informed through anomaly.
      anomaly = anomaly_active_ || amcl_timing_hold_ || odom_timing_hold_ ||
        timer_health_hold_ || explicit_rebase_required_ ||
        !have_trusted_transform_ || rebase_requested_;
    }
    publish_anomaly(anomaly);
  }

  void publish_guard_mode()
  {
    std_msgs::msg::String message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool hard_hold = anomaly_active_ || amcl_timing_hold_ ||
        odom_timing_hold_ || timer_health_hold_ || explicit_rebase_required_ ||
        !have_trusted_transform_ || rebase_requested_ || trusted_override_hard_hold_;
      if (hard_hold) {
        message.data = "HARD_HOLD";
      } else if (trusted_override_verify_epoch_active_) {
        message.data = "VERIFY_TRUSTED";
      } else if (trusted_override_active_) {
        message.data = "TRUSTED_OVERRIDE";
      } else {
        message.data = "ACTIVE";
      }
    }
    // on_publish_timer calls this at publish_rate_hz_ as a heartbeat.  Reliable
    // transient-local QoS also gives late-joining status publishers the latest
    // state immediately.
    mode_publisher_->publish(message);
  }

  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string pose_topic_;
  std::string initialpose_topic_;
  std::string map_topic_;
  std::string scan_topic_;
  std::string mode_topic_;
  std::string interlock_blocked_topic_;
  std::string nomotion_update_service_;
  std::string global_localization_service_;
  double max_linear_speed_{};
  double max_angular_speed_{};
  double max_linear_acceleration_{};
  double max_angular_acceleration_{};
  double linear_slack_{};
  double angular_slack_{};
  double max_odom_correction_{};
  double max_odom_yaw_correction_{};
  double max_cumulative_map_odom_translation_{};
  double max_cumulative_map_odom_yaw_{};
  double recovery_cumulative_map_odom_translation_{};
  double recovery_cumulative_map_odom_yaw_{};
  bool rolling_anchor_enabled_{true};
  double rolling_anchor_translation_deadband_{};
  double rolling_anchor_yaw_deadband_{};
  double rolling_anchor_max_step_translation_{};
  double rolling_anchor_max_step_yaw_{};
  double rolling_anchor_catchup_max_gap_sec_{};
  double rolling_anchor_catchup_max_step_scale_{};
  double rolling_anchor_catchup_max_likelihood_drop_{};
  double rolling_anchor_window_sec_{};
  double rolling_anchor_window_translation_budget_{};
  double rolling_anchor_window_yaw_budget_{};
  double rolling_anchor_translation_budget_per_odom_meter_{};
  double rolling_anchor_yaw_budget_per_odom_rad_{};
  double rolling_anchor_window_translation_motion_cap_{};
  double rolling_anchor_window_yaw_motion_cap_{};
  double rolling_anchor_odom_translation_epsilon_{};
  double rolling_anchor_odom_yaw_epsilon_{};
  double rolling_anchor_session_translation_budget_{};
  double rolling_anchor_session_yaw_budget_{};
  bool cumulative_overrun_defer_enabled_{true};
  int cumulative_overrun_defer_consecutive_samples_{};
  double cumulative_overrun_defer_min_duration_sec_{};
  double cumulative_overrun_defer_max_duration_sec_{};
  double cumulative_overrun_defer_max_translation_excess_{};
  double cumulative_overrun_defer_max_yaw_excess_{};
  double cumulative_overrun_defer_max_correction_translation_{};
  double cumulative_overrun_defer_max_correction_yaw_{};
  double cumulative_overrun_defer_max_source_gap_sec_{};
  double cumulative_overrun_defer_max_odom_receipt_age_sec_{};
  double cumulative_overrun_defer_pose_odom_stamp_tolerance_sec_{};
  bool trusted_override_enabled_{true};
  int trusted_override_active_consecutive_samples_{};
  double trusted_override_active_min_duration_sec_{};
  double trusted_override_active_evidence_timeout_sec_{};
  int trusted_override_hold_consecutive_samples_{};
  double trusted_override_hold_min_duration_sec_{};
  double trusted_override_hold_stationary_duration_sec_{};
  double trusted_override_verify_max_duration_sec_{};
  int trusted_override_return_consecutive_samples_{};
  double trusted_override_return_min_duration_sec_{};
  int trusted_override_min_valid_beams_{};
  double trusted_override_min_inlier_ratio_{};
  double trusted_override_min_likelihood_{};
  double trusted_override_max_trimmed_mean_distance_{};
  double trusted_override_max_unknown_offmap_ratio_{};
  int trusted_override_catastrophic_min_valid_beams_{};
  double trusted_override_catastrophic_min_inlier_ratio_{};
  double trusted_override_catastrophic_min_likelihood_{};
  double trusted_override_catastrophic_max_trimmed_mean_distance_{};
  double trusted_override_catastrophic_max_unknown_offmap_ratio_{};
  int trusted_override_soft_failure_consecutive_samples_{};
  double active_quality_scan_tolerance_sec_{};
  double max_amcl_pose_age_sec_{};
  double max_future_pose_skew_sec_{};
  double max_amcl_validation_gap_sec_{};
  double amcl_silence_timeout_while_moving_sec_{};
  double amcl_silence_motion_translation_{};
  double amcl_silence_motion_yaw_{};
  double amcl_silence_motion_grace_sec_{};
  double max_publish_timer_gap_sec_{};
  double max_odom_observation_gap_sec_{};
  double max_odom_tf_age_sec_{};
  double hard_odom_observation_gap_sec_{};
  double hard_odom_tf_age_sec_{};
  double max_odom_step_translation_{};
  double max_odom_step_yaw_{};
  double max_map_base_step_translation_{};
  double max_map_base_step_yaw_{};
  double publish_rate_hz_{};
  int bootstrap_consecutive_valid_samples_{};
  bool bootstrap_requires_initialpose_{false};
  bool auto_global_localization_{true};
  bool allow_manual_rebase_{false};
  bool auto_initial_pose_enabled_{true};
  double auto_initial_pose_x_{};
  double auto_initial_pose_y_{};
  double auto_initial_pose_yaw_{};
  double auto_initial_pose_variance_x_{};
  double auto_initial_pose_variance_y_{};
  double auto_initial_pose_variance_yaw_{};
  double epoch_initial_pose_x_{};
  double epoch_initial_pose_y_{};
  double epoch_initial_pose_yaw_{};
  double bootstrap_min_epoch_age_sec_{};
  int bootstrap_min_post_global_updates_{};
  int bootstrap_good_samples_{};
  double bootstrap_min_confirm_duration_sec_{};
  double bootstrap_seed_max_translation_{};
  double bootstrap_seed_max_yaw_{};
  double bootstrap_cov_max_xy_eigenvalue_{};
  double bootstrap_cov_max_yaw_{};
  double bootstrap_zero_cov_epsilon_{};
  double bootstrap_timeout_sec_{};
  int bootstrap_max_nomotion_requests_{};
  double bootstrap_retry_cooldown_sec_{};
  int score_occupied_threshold_{};
  int score_max_beams_{};
  int score_min_valid_beams_{};
  double score_sigma_m_{};
  double score_inlier_distance_m_{};
  double score_min_inlier_ratio_{};
  double score_min_likelihood_{};
  double score_max_trimmed_mean_distance_{};
  double score_max_unknown_offmap_ratio_{};
  double score_neighbor_translation_m_{};
  double score_neighbor_yaw_rad_{};
  double score_max_neighbor_likelihood_gain_{};
  int recovery_consecutive_valid_samples_{};
  int initialpose_rebase_consecutive_valid_samples_{};
  int timer_recovery_consecutive_ticks_{};
  int odom_timing_soft_fault_consecutive_samples_{};
  double odom_timing_soft_fault_min_duration_sec_{};
  double initialpose_rebase_position_tolerance_{};
  double initialpose_rebase_yaw_tolerance_{};
  int recovery_cluster_consecutive_samples_{};
  double recovery_cluster_translation_tolerance_{};
  double recovery_cluster_yaw_tolerance_{};
  double recovery_cluster_odom_translation_tolerance_{};
  double recovery_cluster_odom_yaw_tolerance_{};
  double recovery_cluster_min_duration_sec_{};
  double recovery_stationary_duration_sec_{};
  double interlock_blocked_max_age_sec_{};
  bool auto_reanchor_enabled_{true};
  double auto_reanchor_max_translation_{};
  double auto_reanchor_max_yaw_{};
  double auto_reanchor_min_likelihood_improvement_{};
  double auto_reanchor_max_odom_receipt_age_sec_{};
  double auto_reanchor_cooldown_sec_{};
  int max_auto_reanchors_per_session_{};
  int post_reanchor_consecutive_valid_samples_{};
  double post_reanchor_min_confirm_duration_sec_{};
  double nomotion_update_period_sec_{};
  int recovery_nomotion_max_requests_{};
  double recovery_nomotion_max_window_sec_{};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr anomaly_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr interlock_blocked_subscription_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr nomotion_update_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_localization_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr nomotion_update_timer_;

  std::mutex mutex_;
  bool have_trusted_transform_{false};
  bool have_authorized_anchor_{false};
  bool anomaly_active_{true};
  bool amcl_timing_hold_{false};
  bool odom_timing_hold_{false};
  bool have_seen_amcl_stamp_{false};
  bool have_pending_candidate_{false};
  bool have_odom_baseline_{false};
  bool have_timer_tick_{false};
  bool timer_health_hold_{false};
  bool suppress_tf_output_{true};
  bool have_amcl_receipt_{false};
  bool have_validated_amcl_receipt_{false};
  bool have_accepted_amcl_receipt_{false};
  bool have_amcl_motion_threshold_crossing_{false};
  bool have_odom_motion_{false};
  bool have_motion_reference_{false};
  bool odom_moved_since_last_amcl_{false};
  bool have_new_odom_observation_{false};
  bool have_integrity_odom_stamp_{false};
  bool rebase_requested_{false};
  bool explicit_rebase_required_{false};
  bool have_recovery_cluster_{false};
  bool have_recovery_cluster_started_{false};
  bool recovery_cluster_reported_{false};
  bool have_interlock_blocked_receipt_{false};
  bool interlock_blocked_{true};
  bool have_authorized_anchor_steady_{false};
  bool post_reanchor_verification_active_{false};
  bool auto_reanchor_integrity_inhibit_{false};
  bool have_auto_reanchor_integrity_inhibit_started_{false};
  bool have_recovery_nomotion_window_{false};
  bool nomotion_request_in_flight_{false};
  bool have_stationary_reference_{false};
  bool have_last_published_safe_transform_{false};
  bool global_localization_completed_{false};
  bool global_localization_in_flight_{false};
  bool waiting_for_nomotion_pose_{false};
  bool force_global_localization_for_epoch_{false};
  bool bootstrap_retry_pending_{false};
  bool have_odom_soft_timing_fault_window_{false};
  bool have_odom_soft_timing_fault_last_stamp_{false};
  bool cumulative_overrun_defer_active_{false};
  bool trusted_override_active_{false};
  bool trusted_override_soft_hold_{false};
  bool trusted_override_hard_hold_{false};
  bool trusted_override_verify_epoch_active_{false};
  bool trusted_override_confirmation_active_{false};
  bool trusted_override_return_active_{false};
  bool have_new_scan_observation_{false};
  int pending_valid_samples_{0};
  int timer_healthy_ticks_{0};
  int recovery_cluster_valid_samples_{0};
  int odom_healthy_ticks_{0};
  int odom_soft_timing_fault_samples_{0};
  int cumulative_overrun_defer_samples_{0};
  int trusted_override_confirmation_samples_{0};
  int trusted_override_return_samples_{0};
  int trusted_override_soft_failure_samples_{0};
  TrustedOverrideFailureEvidence trusted_override_failure_evidence_{
    TrustedOverrideFailureEvidence::kNone};
  std::uint64_t trusted_override_verify_epoch_id_{0};
  int bootstrap_post_global_updates_{0};
  int bootstrap_nomotion_requests_sent_{0};
  int recovery_nomotion_requests_sent_{0};
  std::uint64_t historical_amcl_drop_count_{0};
  int auto_reanchors_used_{0};
  std::uint64_t authorized_anchor_generation_{0};
  std::uint64_t rolling_anchor_update_count_{0};
  tf2::Transform authorized_anchor_map_odom_;
  tf2::Transform last_trusted_map_odom_;
  tf2::Transform last_trusted_map_base_;
  tf2::Transform pending_map_odom_;
  tf2::Transform pending_map_base_;
  tf2::Transform last_odom_base_;
  tf2::Transform requested_rebase_map_base_;
  tf2::Transform recovery_cluster_map_odom_;
  tf2::Transform recovery_cluster_map_base_;
  tf2::Transform recovery_cluster_odom_base_;
  tf2::Transform motion_reference_odom_base_;
  tf2::Transform stationary_reference_odom_base_;
  tf2::Transform last_published_safe_map_odom_;
  std::shared_ptr<const MapDistanceField> map_distance_field_;
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scan_buffer_;
  std::deque<AnchorAdjustment> rolling_anchor_adjustments_;
  std::deque<OdomMotionSegment> rolling_anchor_odom_motion_;
  std::vector<BootstrapSample> bootstrap_samples_;
  rclcpp::Time authorized_anchor_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trusted_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen_amcl_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_received_amcl_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_validated_amcl_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_accepted_amcl_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time amcl_motion_crossing_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pending_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_new_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_new_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_integrity_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_soft_timing_fault_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cumulative_overrun_defer_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trusted_override_confirmation_last_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trusted_override_return_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trusted_override_last_failure_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trusted_override_last_diagnostic_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time localization_stamp_floor_{0, 0, RCL_ROS_TIME};
  rclcpp::Time nomotion_request_stamp_floor_{0, 0, RCL_ROS_TIME};
  rclcpp::Time auto_reanchor_integrity_inhibit_source_stamp_{0, 0, RCL_ROS_TIME};
  const char * trusted_override_last_counter_reset_reason_{"startup"};
  std::chrono::steady_clock::time_point last_timer_tick_{};
  std::chrono::steady_clock::time_point last_amcl_receipt_{};
  std::chrono::steady_clock::time_point last_validated_amcl_receipt_{};
  std::chrono::steady_clock::time_point last_accepted_amcl_receipt_{};
  std::chrono::steady_clock::time_point amcl_motion_threshold_crossed_steady_{};
  std::chrono::steady_clock::time_point last_odom_motion_{};
  std::chrono::steady_clock::time_point odom_baseline_started_{};
  std::chrono::steady_clock::time_point localization_epoch_steady_{};
  std::chrono::steady_clock::time_point bootstrap_retry_after_{};
  std::chrono::steady_clock::time_point last_nomotion_request_steady_{};
  std::chrono::steady_clock::time_point last_new_odom_observation_steady_{};
  std::chrono::steady_clock::time_point last_new_scan_observation_steady_{};
  std::chrono::steady_clock::time_point recovery_cluster_started_{};
  std::chrono::steady_clock::time_point last_interlock_blocked_receipt_{};
  std::chrono::steady_clock::time_point authorized_anchor_steady_{};
  std::chrono::steady_clock::time_point odom_soft_timing_fault_started_at_{};
  std::chrono::steady_clock::time_point auto_reanchor_integrity_inhibit_started_{};
  std::chrono::steady_clock::time_point recovery_nomotion_window_started_{};
  std::chrono::steady_clock::time_point cumulative_overrun_defer_started_{};
  std::chrono::steady_clock::time_point trusted_override_confirmation_started_{};
  std::chrono::steady_clock::time_point trusted_override_return_started_{};
  std::chrono::steady_clock::time_point trusted_override_verify_epoch_started_{};
  double amcl_motion_crossing_translation_{0.0};
  double amcl_motion_crossing_yaw_{0.0};
  double odom_soft_timing_fault_peak_{0.0};
  double rolling_anchor_session_translation_used_{0.0};
  double rolling_anchor_session_yaw_used_{0.0};
  double cumulative_overrun_defer_peak_translation_{0.0};
  double cumulative_overrun_defer_peak_yaw_{0.0};
  double cumulative_overrun_defer_peak_correction_translation_{0.0};
  double cumulative_overrun_defer_peak_correction_yaw_{0.0};
  std::string odom_soft_timing_fault_reason_;
  std::string auto_reanchor_integrity_inhibit_reason_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclTfGuard>());
  rclcpp::shutdown();
  return 0;
}
