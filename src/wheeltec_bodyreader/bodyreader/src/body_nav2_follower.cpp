#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "bodyreader_msg/msg/body_jump_event.hpp"
#include "bodyreader_msg/msg/bodyposture.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define BODY_NAV2_DEPTH_CAMERA_BEHIND_LIDAR_M 0.13
#define BODY_NAV2_LIDAR_FORWARD_OF_BASE_M 0.247
#define BODY_NAV2_PERSON_MAP_DEPTH_SCALE 1.03
#define BODY_NAV2_TARGET_HALF_WIDTH_M 0.22
#define BODY_NAV2_VISUAL_REVERSE_MAX_RATIO 1.00
#define BODY_NAV2_VISUAL_FULL_SPEED_ERROR_M 0.40
#define BODY_NAV2_VISUAL_STOP_EPSILON_MPS 0.005
#define BODY_NAV2_SAFETY_SLOW_ENTRY_SCALE 0.45
#define BODY_NAV2_SAFETY_PRETRIGGER_SCALE 0.18
#define BODY_NAV2_SAFETY_DECEL_MPS2 1.20
#define BODY_NAV2_SIDE_APPROACH_MIN_MPS 0.03
#define BODY_NAV2_ENABLE_OBSTACLE_CONFIRM_ESTOP 0
#define BODY_NAV2_AVOIDANCE_GOAL_ELASTIC_M 0.00
#define BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS 3
#define BODY_NAV2_VISUAL_REVERSE_TAKEOVER_CONFIRM_TICKS 3

namespace
{
volatile std::sig_atomic_t g_shutdown_requested = 0;
constexpr int64_t kExactObservationFlushNs = 200000000LL;

void handle_shutdown_signal(int)
{
  g_shutdown_requested = 1;
}
}  // namespace

class BodyNav2Follower : public rclcpp::Node
{
  enum class AvoidanceStage
  {
    IDLE,
    GATE
  };

  enum class NavGoalKind
  {
    NORMAL,
    AVOID_GATE
  };

  enum class VisualTakeoverPhase
  {
    NONE,
    CANCEL_PENDING_CLEAR,
    CANCEL_PENDING_REACQUIRE,
    NAV_DRAINING_CLEAR,
    NAV_DRAINING_REACQUIRE,
    SAFETY_GATE
  };

  enum class ObstacleSector
  {
    NONE,
    FRONT,
    LEFT,
    RIGHT
  };

  struct ObstacleObservation
  {
    ObstacleSector sector{ObstacleSector::NONE};
    double distance{std::numeric_limits<double>::infinity()};
    double angle{0.0};
    // Historical field name: positive infinity is also a valid open-space return.
    std::size_t finite_ray_count{0};
  };

  struct ObstacleScan
  {
    ObstacleObservation front;
    ObstacleObservation left;
    ObstacleObservation right;
  };

  using GoalUUID = std::array<uint8_t, 16>;

  struct NavigationActionTracker
  {
    std::string action_name;
    std::set<GoalUUID> episode_goal_ids;
    std::set<GoalUUID> active_goal_ids_in_latest_status;
    std::map<GoalUUID, int8_t> goal_statuses;
    std::map<GoalUUID, int64_t> last_goal_status_update_ns;
    std::map<GoalUUID, int64_t> last_cancel_request_ns;
    std::map<GoalUUID, uint64_t> cancel_request_sequences;
    std::set<GoalUUID> cancel_requests_in_flight;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub;
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_client;
  };

public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using Spin = nav2_msgs::action::Spin;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

  BodyNav2Follower()
  : Node("body_nav2_follower"),
    mode_(1),
    avoidance_trigger_count_(0),
    left_avoidance_trigger_count_(0),
    right_avoidance_trigger_count_(0),
    visual_reverse_takeover_count_(0),
    avoidance_stage_(AvoidanceStage::IDLE),
    last_sent_goal_kind_(NavGoalKind::NORMAL),
    has_body_(false),
    position_jump_hold_active_(false),
    position_jump_recovery_waiting_(false),
    has_last_person_map_(false),
    has_sent_goal_(false),
    goal_active_(false),
    cancel_sent_(false),
    spin_active_(false),
    spin_cancel_sent_(false),
    nav_retry_waiting_(false),
    scan_received_(false),
    has_cmd_(false),
    visual_filter_init_(false),
    visual_hold_active_(false),
    direct_cmd_init_(false),
    shutdown_stop_done_(false),
    paused_stop_published_(false),
    has_last_clear_person_pose_(false),
    has_side_trigger_target_(false),
    awaiting_visual_after_avoidance_(false),
    avoidance_rearm_required_(false),
    visual_reverse_takeover_pending_(false),
    has_avoidance_gate_goal_(false),
    avoidance_motion_logged_(false),
    has_calibration_reference_(false),
    motion_enabled_(true),
    motion_gate_stop_published_(false),
    goal_sequence_(0),
    spin_sequence_(0),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    nav_action_name_ = declare_parameter<std::string>("nav_action_name", "/navigate_to_pose");
    spin_action_name_ = declare_parameter<std::string>("spin_action_name", "/spin");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    visual_cmd_topic_ = declare_parameter<std::string>("visual_cmd_topic", "/cmd_vel");
    motion_gate_service_name_ = declare_parameter<std::string>(
      "motion_gate_service", "/body_nav2_follower/set_motion_enabled");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    body_frame_id_ = declare_parameter<std::string>("body_frame_id", "");
    behavior_tree_ = declare_parameter<std::string>("behavior_tree", "");

    follow_distance_m_ = declare_parameter<double>("follow_distance_m", 2.0);
    hold_distance_band_m_ = declare_parameter<double>("hold_distance_band_m", 0.08);
    min_goal_distance_m_ = declare_parameter<double>("min_goal_distance_m", 0.15);
    goal_update_period_s_ = declare_parameter<double>("goal_update_period_s", 1.5);
    goal_position_tolerance_m_ = declare_parameter<double>("goal_position_tolerance_m", 0.45);
    goal_yaw_tolerance_rad_ = declare_parameter<double>("goal_yaw_tolerance_rad", 0.45);
    lost_timeout_s_ = declare_parameter<double>("lost_timeout_s", 1.0);
    body_invalid_grace_s_ =
      std::max(0.0, declare_parameter<double>("body_invalid_grace_s", 0.45));
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.20);
    max_retreat_goal_m_ = declare_parameter<double>("max_retreat_goal_m", 0.8);
    target_memory_timeout_s_ = declare_parameter<double>("target_memory_timeout_s", 8.0);
    reacquire_timeout_s_ = declare_parameter<double>("reacquire_timeout_s", 18.0);
    nav2_retry_backoff_s_ = declare_parameter<double>("nav2_retry_backoff_s", 1.5);
    reacquire_spin_min_yaw_rad_ = declare_parameter<double>("reacquire_spin_min_yaw_rad", 0.35);
    reacquire_spin_max_yaw_rad_ = declare_parameter<double>("reacquire_spin_max_yaw_rad", 1.2);
    spin_time_allowance_s_ = declare_parameter<double>("spin_time_allowance_s", 8.0);
    spin_retry_period_s_ = declare_parameter<double>("spin_retry_period_s", 1.5);
    cmd_zero_timeout_s_ = declare_parameter<double>("cmd_zero_timeout_s", 3.0);
    cmd_zero_linear_epsilon_ = declare_parameter<double>("cmd_zero_linear_epsilon", 0.02);
    cmd_zero_angular_epsilon_ = declare_parameter<double>("cmd_zero_angular_epsilon", 0.05);
    scan_timeout_s_ = declare_parameter<double>("scan_timeout_s", 0.6);
    obstacle_front_angle_rad_ = declare_parameter<double>("obstacle_front_angle_rad", 0.45);
    side_obstacle_min_angle_rad_ =
      declare_parameter<double>("side_obstacle_min_angle_rad", 0.35);
    side_obstacle_max_angle_rad_ =
      declare_parameter<double>("side_obstacle_max_angle_rad", 1.57);
    obstacle_slow_distance_m_ = declare_parameter<double>("obstacle_slow_distance_m", 1.25);
    obstacle_nav_distance_m_ = declare_parameter<double>("obstacle_nav_distance_m", 0.75);
    obstacle_nav_release_distance_m_ =
      declare_parameter<double>("obstacle_nav_release_distance_m", 0.80);
    side_obstacle_nav_distance_m_ =
      declare_parameter<double>("side_obstacle_nav_distance_m", 0.40);
    side_obstacle_release_distance_m_ =
      declare_parameter<double>("side_obstacle_release_distance_m", 0.45);
    visual_min_linear_scale_ = declare_parameter<double>("visual_min_linear_scale", 0.25);
    target_exemption_angle_rad_ = declare_parameter<double>("target_exemption_angle_rad", 0.20);
    target_exemption_distance_margin_m_ =
      declare_parameter<double>("target_exemption_distance_margin_m", 0.12);
    avoidance_forward_margin_m_ = declare_parameter<double>("avoidance_forward_margin_m", 0.35);
    avoidance_min_forward_step_m_ =
      declare_parameter<double>("avoidance_min_forward_step_m", 0.45);
    avoidance_max_forward_step_m_ =
      declare_parameter<double>("avoidance_max_forward_step_m", 1.10);
    avoidance_occlusion_hold_s_ =
      declare_parameter<double>("avoidance_occlusion_hold_s", 6.0);
    visual_clear_takeover_front_clearance_m_ =
      declare_parameter<double>("visual_clear_takeover_front_clearance_m", 0.80);
    visual_clear_takeover_side_clearance_m_ =
      declare_parameter<double>("visual_clear_takeover_side_clearance_m", 0.45);
    visual_clear_takeover_min_avoidance_s_ =
      declare_parameter<double>("visual_clear_takeover_min_avoidance_s", 0.60);
    visual_clear_takeover_body_max_age_s_ =
      declare_parameter<double>("visual_clear_takeover_body_max_age_s", 0.30);
    visual_clear_takeover_confirm_frames_ = static_cast<int>(
      declare_parameter<int>("visual_clear_takeover_confirm_frames", 2));
    visual_clear_takeover_confirm_frames_ =
      std::max(1, visual_clear_takeover_confirm_frames_);
    visual_reacquire_takeover_body_max_age_s_ =
      std::max(
      0.05, declare_parameter<double>("visual_reacquire_takeover_body_max_age_s", 0.35));
    visual_reacquire_takeover_min_frame_interval_s_ =
      std::max(
      0.0, declare_parameter<double>("visual_reacquire_takeover_min_frame_interval_s", 0.05));
    visual_reacquire_takeover_max_frame_interval_s_ =
      std::max(
      visual_reacquire_takeover_min_frame_interval_s_,
      declare_parameter<double>("visual_reacquire_takeover_max_frame_interval_s", 0.45));
    visual_reacquire_takeover_confirm_frames_ = static_cast<int>(
      declare_parameter<int>("visual_reacquire_takeover_confirm_frames", 2));
    visual_reacquire_takeover_confirm_frames_ =
      std::max(1, visual_reacquire_takeover_confirm_frames_);
    visual_safety_release_confirm_frames_ = static_cast<int>(
      declare_parameter<int>("visual_safety_release_confirm_frames", 2));
    visual_safety_release_confirm_frames_ =
      std::max(1, visual_safety_release_confirm_frames_);
    visual_safety_min_finite_rays_per_sector_ = static_cast<int>(
      declare_parameter<int>("visual_safety_min_finite_rays_per_sector", 1));
    visual_safety_min_finite_rays_per_sector_ =
      std::max(1, visual_safety_min_finite_rays_per_sector_);
    nav_takeover_child_capture_slop_s_ = std::max(
      0.0, declare_parameter<double>("nav_takeover_child_capture_slop_s", 0.10));
    nav_takeover_cancel_retry_s_ = std::max(
      0.10, declare_parameter<double>("nav_takeover_cancel_retry_s", 0.35));
    nav_takeover_status_quiet_s_ = std::max(
      0.10, declare_parameter<double>("nav_takeover_status_quiet_s", 0.30));
    nav_takeover_cmd_quiet_s_ = std::max(
      0.10, declare_parameter<double>("nav_takeover_cmd_quiet_s", 0.30));
    nav_takeover_child_discovery_s_ = std::max(
      0.50, declare_parameter<double>("nav_takeover_child_discovery_s", 1.20));
    visual_x_p_ = declare_parameter<double>("visual_x_p", 0.5);
    visual_x_d_ = declare_parameter<double>("visual_x_d", 0.33);
    visual_z_p_ = declare_parameter<double>("visual_z_p", 1.2);
    visual_z_d_ = declare_parameter<double>("visual_z_d", 0.5);
    visual_filter_alpha_ = declare_parameter<double>("visual_filter_alpha", 0.35);
    visual_angle_deadband_ = declare_parameter<double>("visual_angle_deadband", 0.025);
    visual_distance_deadband_mm_ = declare_parameter<double>("visual_distance_deadband_mm", 80.0);
    visual_max_linear_mps_ = declare_parameter<double>("visual_max_linear_mps", 0.75);
    visual_max_angular_rps_ = declare_parameter<double>("visual_max_angular_rps", 1.0);
    visual_linear_accel_limit_ = declare_parameter<double>("visual_linear_accel_limit", 0.90);
    visual_reverse_accel_limit_ =
      declare_parameter<double>("visual_reverse_accel_limit", 1.80);
    visual_linear_decel_limit_ = declare_parameter<double>("visual_linear_decel_limit", 0.35);
    visual_angular_accel_limit_ = declare_parameter<double>("visual_angular_accel_limit", 0.9);
    visual_allow_reverse_ = declare_parameter<bool>("visual_allow_reverse", true);
    require_scan_for_visual_ = declare_parameter<bool>("require_scan_for_visual", true);

    camera_offset_x_m_ = declare_parameter<double>("camera_offset_x_m", 0.0);
    camera_offset_y_m_ = declare_parameter<double>("camera_offset_y_m", 0.0);
    camera_offset_z_m_ = declare_parameter<double>("camera_offset_z_m", 0.0);
    camera_yaw_offset_rad_ = declare_parameter<double>("camera_yaw_offset_rad", 0.0);

    respect_mode_topic_ = declare_parameter<bool>("respect_mode_topic", true);
    cancel_on_lost_ = declare_parameter<bool>("cancel_on_lost", true);
    enable_retreat_goal_ = declare_parameter<bool>("enable_retreat_goal", false);
    enable_avoidance_side_goal_ = declare_parameter<bool>("enable_avoidance_side_goal", true);
    keep_last_goal_when_occluded_ = declare_parameter<bool>("keep_last_goal_when_occluded", true);
    reacquire_yaw_goal_ = declare_parameter<bool>("reacquire_yaw_goal", true);
    enable_cmd_watchdog_ = declare_parameter<bool>("enable_cmd_watchdog", true);
    shutdown_stop_publish_count_ = declare_parameter<int>("shutdown_stop_publish_count", 8);
    shutdown_stop_publish_period_ms_ =
      declare_parameter<int>("shutdown_stop_publish_period_ms", 30);
    mode_required_ = declare_parameter<int>("mode_required", 2);
    mode_ = declare_parameter<int>("initial_mode", 1);
    motion_enabled_ = declare_parameter<bool>("motion_enabled", true);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin_action_name_);
    create_navigation_action_tracker("/compute_path_to_pose");
    create_navigation_action_tracker("/follow_path");
    create_navigation_action_tracker(spin_action_name_);
    create_navigation_action_tracker("/drive_on_heading");
    create_navigation_action_tracker("/wait");
    create_navigation_action_tracker("/backup");
    create_navigation_action_tracker("/assisted_teleop");

    body_sub_ = create_subscription<bodyreader_msg::msg::Bodyposture>(
      "/body_posture_yolo_validated", rclcpp::QoS(1).reliable(),
      std::bind(&BodyNav2Follower::body_callback, this, std::placeholders::_1));

    mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/mode", 10,
      std::bind(&BodyNav2Follower::mode_callback, this, std::placeholders::_1));

    motion_gate_service_ = create_service<std_srvs::srv::SetBool>(
      motion_gate_service_name_,
      std::bind(
        &BodyNav2Follower::motion_gate_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 20,
      std::bind(&BodyNav2Follower::cmd_callback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BodyNav2Follower::scan_callback, this, std::placeholders::_1));

    calibration_reference_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/body_nav2_calibration/reference_input", 10,
      std::bind(
        &BodyNav2Follower::calibration_reference_callback, this, std::placeholders::_1));

    visual_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(visual_cmd_topic_, 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/body_nav2_goal", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/body_nav2_state", 10);
    calibration_sample_pub_ =
      create_publisher<std_msgs::msg::String>("/body_nav2_calibration/sample", 10);
    calibration_person_base_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/body_nav2_calibration/person_base", 10);
    calibration_person_map_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/body_nav2_calibration/person_map", 10);
    calibration_robot_map_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/body_nav2_calibration/robot_map", 10);
    calibration_reference_map_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/body_nav2_calibration/reference_map", 10);
    calibration_obstacle_ray_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/body_nav2_calibration/obstacle_ray_map", 10);
    calibration_obstacle_assumed_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/body_nav2_calibration/obstacle_assumed_map", 10);
    calibration_latched_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/body_nav2_calibration/latched_goal", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BodyNav2Follower::control_loop, this));

    last_goal_send_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_failed_goal_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_spin_send_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_body_time_ = this->now();
    invalid_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_person_map_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_clear_person_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_cmd_time_ = this->now();
    last_cmd_motion_time_ = this->now();
    last_scan_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_direct_cmd_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_avoidance_goal_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    avoidance_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    avoidance_goal_accept_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_dual_presence_body_time_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    active_visual_body_time_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    filtered_x_angle_ = 0.0;
    last_visual_error_angle_ = 0.0;
    last_visual_error_distance_ = 0.0;
    RCLCPP_INFO(
      get_logger(),
      "BodyNav2Follower hybrid ready: action=%s spin=%s scan=%s visual_cmd=%s follow_distance=%.2fm motion_gate=%s",
      nav_action_name_.c_str(), spin_action_name_.c_str(), scan_topic_.c_str(),
      visual_cmd_topic_.c_str(), follow_distance_m_, motion_gate_service_name_.c_str());
  }

  void request_shutdown_stop(const std::string & reason)
  {
    if (shutdown_stop_done_) {
      return;
    }
    shutdown_stop_done_ = true;

    if (timer_) {
      timer_->cancel();
    }
    dual_handoff_active_ = false;
    dual_handoff_reacquire_armed_ = false;
    dual_handoff_retired_by_strict_ = false;
    dual_handoff_strict_confirm_count_ = 0;
    cancel_goal(reason);
    cancel_spin(reason);
    reset_avoidance_episode();
    reset_visual_controller();

    geometry_msgs::msg::Twist stop;
    const int publish_count = std::max(1, shutdown_stop_publish_count_);
    const int publish_period_ms = std::max(0, shutdown_stop_publish_period_ms_);
    last_direct_cmd_ = stop;
    direct_cmd_init_ = true;
    last_direct_cmd_time_ = this->now();
    for (int i = 0; i < publish_count; ++i) {
      visual_cmd_pub_->publish(stop);
      if (publish_period_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(publish_period_ms));
      }
    }
    publish_state(reason);
    RCLCPP_INFO(
      get_logger(), "Published %d stop command(s) before shutdown: %s",
      publish_count, reason.c_str());
  }

private:
  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double clamp_value(double value, double low, double high)
  {
    if (value < low) {
      return low;
    }
    if (value > high) {
      return high;
    }
    return value;
  }

  // Legacy skeleton-ID jump/dual callbacks are intentionally compiled out.
  // The active control path below consumes only exact observations emitted by
  // the YOLO identity bridge.  A skeleton ID is never a cross-frame key.
#if 0
  void reset_position_jump_recovery_confirmation()
  {
    position_jump_recovery_valid_count_ = 0;
    position_jump_recovery_body_id_ = 0;
    position_jump_recovery_last_valid_time_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void reset_position_jump_event_context(bool clear_strict_target = false)
  {
    position_jump_hold_active_ = false;
    position_jump_recovery_waiting_ = false;
    position_jump_source_epoch_ = 0;
    position_jump_event_id_ = 0;
    position_jump_event_update_seq_ = 0;
    position_jump_origin_body_id_ = 0;
    position_jump_expected_body_id_ = 0;
    position_jump_native_body_id_ = 0;
    position_jump_trusted_depth_mm_ = 0.0;
    position_jump_observed_depth_mm_ = 0.0;
    position_jump_confirmed_depth_mm_ = 0.0;
    position_jump_phase_ = 0;
    position_jump_outcome_ = 0;
    position_jump_bound_goal_sequence_ = 0;
    position_jump_source_reset_recovery_ = false;
    pending_position_jump_events_.clear();
    reset_position_jump_recovery_confirmation();
    if (clear_strict_target) {
      strict_target_body_id_ = 0;
    }
  }

  bool position_jump_recovery_sample_confirmed(
    const bodyreader_msg::msg::Bodyposture & sample,
    const rclcpp::Time & now)
  {
    constexpr int kRequiredFrames = 2;
    constexpr double kMinFrameIntervalS = 0.03;
    constexpr double kMaxFrameIntervalS = 0.45;
    constexpr double kConsistencyMm = 150.0;

    bool continue_confirmation = position_jump_recovery_valid_count_ > 0;
    double interval_s = 0.0;
    if (continue_confirmation) {
      interval_s = (now - position_jump_recovery_last_valid_time_).seconds();
      if (interval_s < kMinFrameIntervalS) {
        return false;
      }
      const double dx = static_cast<double>(sample.centerofmass_x) -
        static_cast<double>(position_jump_recovery_candidate_.centerofmass_x);
      const double dz = static_cast<double>(sample.centerofmass_z) -
        static_cast<double>(position_jump_recovery_candidate_.centerofmass_z);
      continue_confirmation = interval_s <= kMaxFrameIntervalS &&
        sample.bodyid == position_jump_recovery_body_id_ &&
        std::hypot(dx, dz) <= kConsistencyMm;
    }

    if (!continue_confirmation) {
      position_jump_recovery_valid_count_ = 1;
    } else {
      position_jump_recovery_valid_count_ = std::min(
        position_jump_recovery_valid_count_ + 1, kRequiredFrames);
    }
    position_jump_recovery_candidate_ = sample;
    position_jump_recovery_body_id_ = sample.bodyid;
    position_jump_recovery_last_valid_time_ = now;

    if (position_jump_recovery_valid_count_ < kRequiredFrames) {
      RCLCPP_INFO(
        get_logger(),
        "Position-jump recovery candidate: body=%d depth_mm=%.0f required_frames=%d",
        sample.bodyid, sample.centerofmass_z, kRequiredFrames);
      return false;
    }
    return true;
  }

  void body_callback(const bodyreader_msg::msg::Bodyposture::SharedPtr msg)
  {
    const auto now = this->now();
    const bool valid_lock = msg->lock_status == 2;
    const bool valid_depth =
      std::isfinite(msg->centerofmass_x) &&
      std::isfinite(msg->centerofmass_y) &&
      std::isfinite(msg->centerofmass_z) &&
      msg->centerofmass_z > 100.0f;
    bool position_jump_recovery_completed = false;
    bool position_jump_danger_cancel = false;

    if (valid_lock && valid_depth && position_jump_source_reset_recovery_ &&
        !position_jump_hold_active_)
    {
      const auto pending_event = pending_position_jump_events_.find(msg->bodyid);
      if (pending_event != pending_position_jump_events_.end()) {
        position_jump_expected_body_id_ = msg->bodyid;
        auto event = std::make_shared<bodyreader_msg::msg::BodyJumpEvent>(
          pending_event->second);
        pending_position_jump_events_.erase(pending_event);
        position_jump_event_callback(event);
      }
    }

    if (position_jump_hold_active_) {
      reset_position_jump_recovery_confirmation();
      return;
    }

    if (position_jump_recovery_waiting_) {
      if (!valid_lock || !valid_depth) {
        reset_position_jump_recovery_confirmation();
        return;
      }
      if (position_jump_expected_body_id_ <= 0) {
        // A body_main source reset restarts its persistent-ID allocator. The
        // first strict sample establishes the new epoch's target candidate;
        // the ordinary two-frame confirmation still gates motion.
        position_jump_expected_body_id_ = msg->bodyid;
      }
      if (msg->bodyid != position_jump_expected_body_id_)
      {
        reset_position_jump_recovery_confirmation();
        if (position_jump_source_reset_recovery_) {
          position_jump_expected_body_id_ = msg->bodyid;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Position-jump recovery waits for target body=%d; received body=%d",
            position_jump_expected_body_id_, msg->bodyid);
          return;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Body source-reset recovery candidate changed; restart two-frame "
          "confirmation with body=%d", msg->bodyid);
      }
      if (position_jump_confirmed_depth_mm_ > 100.0 &&
          std::fabs(
            static_cast<double>(msg->centerofmass_z) -
            position_jump_confirmed_depth_mm_) > 150.0)
      {
        reset_position_jump_recovery_confirmation();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Position-jump recovery depth mismatch: target=%d sample=%.0f confirmed=%.0f",
          msg->bodyid, msg->centerofmass_z,
          position_jump_confirmed_depth_mm_);
        return;
      }
      if (!position_jump_recovery_sample_confirmed(*msg, now)) {
        return;
      }
      position_jump_recovery_waiting_ = false;
      reset_position_jump_recovery_confirmation();
      strict_target_body_id_ = msg->bodyid;
      position_jump_recovery_completed = true;
      const bool confirmed_nearer =
        position_jump_outcome_ ==
        bodyreader_msg::msg::BodyJumpEvent::OUTCOME_REBASED &&
        position_jump_confirmed_depth_mm_ > 100.0 &&
        position_jump_trusted_depth_mm_ - position_jump_confirmed_depth_mm_ > 200.0;
      const bool dangerous_near =
        position_jump_confirmed_depth_mm_ * 0.001 <
        follow_distance_m_ - hold_distance_band_m_ &&
        static_cast<double>(msg->centerofmass_z) * 0.001 <
        follow_distance_m_ - hold_distance_band_m_;
      const bool same_avoidance_goal =
        position_jump_bound_goal_sequence_ != 0 &&
        position_jump_bound_goal_sequence_ == goal_sequence_ &&
        last_sent_goal_kind_ == NavGoalKind::AVOID_GATE &&
        avoidance_episode_active() && (goal_active_ || has_sent_goal_);
      position_jump_danger_cancel =
        confirmed_nearer && dangerous_near && same_avoidance_goal;
      RCLCPP_INFO(
        get_logger(),
        "Position-jump target recovery confirmed: body=%d event=%llu depth=%.0f "
        "confirmed=%.0f dangerous_cancel=%s",
        msg->bodyid,
        static_cast<unsigned long long>(position_jump_event_id_),
        msg->centerofmass_z, position_jump_confirmed_depth_mm_,
        position_jump_danger_cancel ? "true" : "false");
    } else if (valid_lock && valid_depth) {
      strict_target_body_id_ = msg->bodyid;
    }

    if (valid_lock && valid_depth) {
      if (has_body_ && invalid_body_since_.nanoseconds() != 0) {
        const double invalid_duration_s =
          (now - invalid_body_since_).seconds();
        RCLCPP_INFO(
          get_logger(), "Body observation recovered after %.3fs invalid interval",
          invalid_duration_s);
      }
      invalid_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      latest_body_ = *msg;
      has_body_ = true;
      last_body_time_ = now;
      ++body_generation_;
      if (dual_handoff_active_ || dual_handoff_reacquire_armed_) {
        ++dual_handoff_strict_confirm_count_;
        if (dual_handoff_strict_confirm_count_ >= 3) {
          dual_handoff_active_ = false;
          dual_handoff_reacquire_armed_ = false;
          dual_handoff_retired_by_strict_ = true;
          dual_handoff_strict_confirm_count_ = 0;
          RCLCPP_INFO(
            get_logger(),
            "Strict validated body stable for 3 frames; retire raw dual visual fallback");
          }
      }
      if (position_jump_recovery_completed) {
        const uint64_t recovered_event_id = position_jump_event_id_;
        reset_position_jump_event_context(false);
        if (position_jump_danger_cancel &&
            visual_takeover_phase_ == VisualTakeoverPhase::NONE)
        {
          visual_takeover_phase_ = VisualTakeoverPhase::CANCEL_PENDING_CLEAR;
          awaiting_visual_after_avoidance_ = false;
          RCLCPP_WARN(
            get_logger(),
            "Confirmed same-target dangerous-near jump event=%llu; "
            "cancel and drain only the bound Nav2 avoidance task",
            static_cast<unsigned long long>(recovered_event_id));
          request_visual_takeover_cancel(
            "confirmed_target_position_jump_near");
          cancel_tracked_navigation_children();
          reset_visual_controller();
          publish_safety_stop_cmd(
            "confirmed_target_position_jump_near_cancel");
        }
      }
    } else {
      dual_handoff_strict_confirm_count_ = 0;
      if (invalid_body_since_.nanoseconds() == 0) {
        invalid_body_since_ = now;
      }
      const double invalid_duration_s = (now - invalid_body_since_).seconds();

      if (has_body_ && invalid_duration_s < body_invalid_grace_s_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 250,
          "Transient invalid body interval ignored: age=%.3fs grace=%.3fs "
          "lock_status=%d depth_mm=%.0f",
          invalid_duration_s, body_invalid_grace_s_, msg->lock_status, msg->centerofmass_z);
        return;
      }

      if (has_body_) {
        if (avoidance_episode_active()) {
          avoidance_visual_loss_seen_ = true;
          if (!visual_takeover_cancel_pending()) {
            reset_visual_reacquire_takeover_confirmation();
          }
          RCLCPP_INFO(
            get_logger(),
            "Visual target loss observed during Nav2 avoidance; arm dual-presence reacquire");
        }
        has_body_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Body loss confirmed after %.3fs invalid interval: lock_status=%d depth_mm=%.0f",
          invalid_duration_s, msg->lock_status, msg->centerofmass_z);
      }
    }
  }

  void position_jump_event_callback(
    const bodyreader_msg::msg::BodyJumpEvent::SharedPtr msg)
  {
    using Event = bodyreader_msg::msg::BodyJumpEvent;
    if (msg->phase == Event::PHASE_RESET) {
      if (position_jump_source_epoch_ != 0 &&
          msg->source_epoch < position_jump_source_epoch_)
      {
        return;
      }
      const bool had_target_context =
        strict_target_body_id_ > 0 || has_body_ ||
        position_jump_hold_active_ || position_jump_recovery_waiting_;
      reset_position_jump_event_context(true);
      position_jump_source_epoch_ = msg->source_epoch;
      has_body_ = false;
      dual_handoff_active_ = false;
      dual_handoff_reacquire_armed_ = false;
      dual_handoff_strict_confirm_count_ = 0;
      reset_visual_reacquire_takeover_confirmation();
      reset_visual_controller();
      if (had_target_context) {
        position_jump_recovery_waiting_ = true;
        position_jump_expected_body_id_ = 0;
        position_jump_source_reset_recovery_ = true;
      }
      RCLCPP_INFO(
        get_logger(),
        "Body jump source reset: epoch=%llu old_target_cleared=true "
        "recovery_waiting=%s",
        static_cast<unsigned long long>(msg->source_epoch),
        position_jump_recovery_waiting_ ? "true" : "false");
      return;
    }

    if (position_jump_source_epoch_ != 0 &&
        msg->source_epoch < position_jump_source_epoch_)
    {
      return;
    }
    if (msg->source_epoch != position_jump_source_epoch_) {
      reset_position_jump_event_context(false);
      position_jump_source_epoch_ = msg->source_epoch;
    }

    if (msg->phase == Event::PHASE_PENDING) {
      const int current_target = strict_target_body_id_ > 0 ?
        strict_target_body_id_ :
        (has_body_ ? latest_body_.bodyid :
        (position_jump_source_reset_recovery_ ?
        position_jump_expected_body_id_ : 0));
      if (current_target <= 0 && position_jump_source_reset_recovery_ &&
          msg->origin_persistent_body_id > 0)
      {
        pending_position_jump_events_[msg->origin_persistent_body_id] = *msg;
        RCLCPP_INFO(
          get_logger(),
          "Cache body jump event=%llu origin=%d until the reset source "
          "produces a strict target candidate",
          static_cast<unsigned long long>(msg->event_id),
          msg->origin_persistent_body_id);
        return;
      }
      if (msg->origin_persistent_body_id <= 0 ||
          msg->origin_persistent_body_id != current_target)
      {
        RCLCPP_INFO(
          get_logger(),
          "Ignore non-target body jump: event=%llu native=%d origin=%d target=%d "
          "old_z=%.0f raw_z=%.0f",
          static_cast<unsigned long long>(msg->event_id),
          msg->native_body_id, msg->origin_persistent_body_id,
          current_target, msg->trusted_depth_mm, msg->observed_depth_mm);
        return;
      }
      if (position_jump_event_id_ == msg->event_id &&
          msg->update_seq <= position_jump_event_update_seq_)
      {
        return;
      }

      reset_position_jump_event_context(false);
      position_jump_source_epoch_ = msg->source_epoch;
      position_jump_event_id_ = msg->event_id;
      position_jump_event_update_seq_ = msg->update_seq;
      position_jump_origin_body_id_ = msg->origin_persistent_body_id;
      position_jump_expected_body_id_ = msg->origin_persistent_body_id;
      position_jump_native_body_id_ = msg->native_body_id;
      position_jump_trusted_depth_mm_ = msg->trusted_depth_mm;
      position_jump_observed_depth_mm_ = msg->observed_depth_mm;
      position_jump_confirmed_depth_mm_ = 0.0;
      position_jump_phase_ = msg->phase;
      position_jump_outcome_ = msg->outcome;
      position_jump_hold_active_ = true;
      position_jump_recovery_waiting_ = true;
      if ((goal_active_ || has_sent_goal_) &&
          last_sent_goal_kind_ == NavGoalKind::AVOID_GATE &&
          avoidance_episode_active())
      {
        position_jump_bound_goal_sequence_ = goal_sequence_;
      }
      reset_position_jump_recovery_confirmation();
      dual_handoff_active_ = false;
      dual_handoff_reacquire_armed_ = false;
      dual_handoff_strict_confirm_count_ = 0;
      reset_visual_reacquire_takeover_confirmation();
      reset_visual_controller();

      const bool nav_engaged = goal_active_ || has_sent_goal_;
      RCLCPP_WARN(
        get_logger(),
        "Target body jump pending: event=%llu native=%d target=%d old_z=%.0f "
        "raw_z=%.0f nav2_continue=%s goal_sequence=%llu",
        static_cast<unsigned long long>(msg->event_id),
        msg->native_body_id, msg->origin_persistent_body_id,
        msg->trusted_depth_mm, msg->observed_depth_mm,
        nav_engaged ? "true" : "false",
        static_cast<unsigned long long>(position_jump_bound_goal_sequence_));
      if (nav_engaged) {
        publish_state("target_position_jump_blocks_visual_nav2_continue");
      } else {
        cancel_spin("target_position_jump_direct_hold");
        if (motion_enabled_ && (!respect_mode_topic_ || mode_ == mode_required_) &&
            !visual_takeover_drain_active())
        {
          publish_safety_stop_cmd("target_position_jump_direct_hold_stop");
        }
      }
      return;
    }

    const auto cached_event = pending_position_jump_events_.find(
      msg->origin_persistent_body_id);
    if (cached_event != pending_position_jump_events_.end() &&
        cached_event->second.event_id == msg->event_id)
    {
      if (msg->phase == Event::PHASE_RELEASED ||
          msg->phase == Event::PHASE_EXPIRED)
      {
        pending_position_jump_events_.erase(cached_event);
      }
    }

    if (msg->event_id != position_jump_event_id_ ||
        msg->origin_persistent_body_id != position_jump_origin_body_id_ ||
        msg->update_seq <= position_jump_event_update_seq_)
    {
      return;
    }

    position_jump_event_update_seq_ = msg->update_seq;
    position_jump_native_body_id_ = msg->native_body_id;
    position_jump_observed_depth_mm_ = msg->observed_depth_mm;
    position_jump_phase_ = msg->phase;
    position_jump_outcome_ = msg->outcome;
    if (msg->current_persistent_body_id > 0) {
      position_jump_expected_body_id_ = msg->current_persistent_body_id;
    }
    if (msg->confirmed_depth_mm > 100.0f) {
      position_jump_confirmed_depth_mm_ = msg->confirmed_depth_mm;
    }

    RCLCPP_INFO(
      get_logger(),
      "Target body jump update: event=%llu phase=%u outcome=%u native=%d "
      "origin=%d current=%d old_z=%.0f raw_z=%.0f confirmed_z=%.0f good=%u",
      static_cast<unsigned long long>(msg->event_id),
      static_cast<unsigned int>(msg->phase),
      static_cast<unsigned int>(msg->outcome), msg->native_body_id,
      msg->origin_persistent_body_id, msg->current_persistent_body_id,
      msg->trusted_depth_mm, msg->observed_depth_mm,
      msg->confirmed_depth_mm, static_cast<unsigned int>(msg->good_count));

    if (msg->phase == Event::PHASE_RELEASED ||
        msg->phase == Event::PHASE_EXPIRED)
    {
      position_jump_hold_active_ = false;
      position_jump_recovery_waiting_ = true;
      reset_position_jump_recovery_confirmation();
      RCLCPP_INFO(
        get_logger(),
        "Target body jump source hold ended; event=%llu wait for two fresh "
        "validated frames of body=%d",
        static_cast<unsigned long long>(msg->event_id),
        position_jump_expected_body_id_);
    } else {
      position_jump_hold_active_ = true;
    }
  }

  void dual_presence_body_callback(
    const bodyreader_msg::msg::Bodyposture::SharedPtr msg)
  {
    const auto now = this->now();
    if (position_jump_hold_active_ || position_jump_recovery_waiting_) {
      dual_handoff_active_ = false;
      dual_handoff_reacquire_armed_ = false;
      dual_handoff_strict_confirm_count_ = 0;
      reset_visual_reacquire_takeover_confirmation();
      return;
    }
    const bool valid =
      msg->lock_status == 2 &&
      std::isfinite(msg->centerofmass_x) &&
      std::isfinite(msg->centerofmass_y) &&
      std::isfinite(msg->centerofmass_z) &&
      msg->centerofmass_z > 100.0f;
    if (!valid) {
      if (
        dual_handoff_active_ || dual_handoff_reacquire_armed_ ||
        visual_takeover_phase_is_reacquire(visual_takeover_phase_))
      {
        dual_handoff_active_ = false;
        dual_handoff_reacquire_armed_ = !dual_handoff_retired_by_strict_;
      }
      if (avoidance_episode_active() && !avoidance_visual_loss_seen_) {
        avoidance_visual_loss_seen_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Skeleton/YOLO co-presence lost during Nav2 avoidance; "
          "arm two-frame dual-presence reacquire");
      }
      reset_visual_reacquire_takeover_confirmation();
      return;
    }

    if (visual_reacquire_takeover_count_ > 0) {
      const double interval_s = (now - last_dual_presence_body_time_).seconds();
      if (interval_s > visual_reacquire_takeover_max_frame_interval_s_) {
        reset_visual_reacquire_takeover_confirmation();
      } else if (interval_s < visual_reacquire_takeover_min_frame_interval_s_) {
        return;
      }
    }

    visual_reacquire_body_id_ = msg->bodyid;
    latest_dual_presence_body_ = *msg;
    last_dual_presence_body_time_ = now;
    visual_reacquire_takeover_count_ = std::min(
      visual_reacquire_takeover_count_ + 1,
      visual_reacquire_takeover_confirm_frames_);
    if (dual_handoff_reacquire_armed_ &&
        visual_reacquire_takeover_count_ >= visual_reacquire_takeover_confirm_frames_)
    {
      dual_handoff_active_ = true;
      dual_handoff_reacquire_armed_ = false;
      dual_handoff_retired_by_strict_ = false;
      dual_handoff_strict_confirm_count_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "Raw dual visual fallback reacquired after %d fresh frames",
        visual_reacquire_takeover_count_);
    }
    if (dual_takeover_gate_open_ && visual_reacquire_takeover_count_ == 1) {
      RCLCPP_INFO(
        get_logger(),
        "Post-clearance dual-presence candidate: body=%d required_frames=%d",
        visual_reacquire_body_id_, visual_reacquire_takeover_confirm_frames_);
    }
  }

#endif

  void reset_position_jump_recovery_confirmation()
  {
  }

  void reset_position_jump_event_context(bool clear_strict_target = false)
  {
    position_jump_hold_active_ = false;
    position_jump_recovery_waiting_ = false;
    if (clear_strict_target) {
      strict_target_body_id_ = 0;
    }
  }

  void body_callback(const bodyreader_msg::msg::Bodyposture::SharedPtr msg)
  {
    const auto sample_time = this->now();
    const bool exact_observation =
      msg->lock_status == 2 &&
      std::isfinite(msg->centerofmass_x) &&
      std::isfinite(msg->centerofmass_y) &&
      std::isfinite(msg->centerofmass_z) &&
      msg->centerofmass_z > 100.0f;

    if (exact_observation &&
        sample_time.nanoseconds() < exact_observation_accept_after_ns_)
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Discard exact observation during mode/navigation handoff flush window");
      return;
    }

    if (exact_observation) {
      if (has_body_ && invalid_body_since_.nanoseconds() != 0) {
        RCLCPP_INFO(
          get_logger(), "Exact YOLO+skeleton observation recovered after %.3fs",
          (sample_time - invalid_body_since_).seconds());
      }
      invalid_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      latest_body_ = *msg;
      has_body_ = true;
      last_body_time_ = sample_time;
      ++body_generation_;
      RCLCPP_DEBUG(
        get_logger(),
        "Exact target observation: skeleton=%d x=%.0f z=%.0f generation=%llu "
        "(skeleton ID is diagnostic only)",
        msg->bodyid, msg->centerofmass_x, msg->centerofmass_z,
        static_cast<unsigned long long>(body_generation_));
      return;
    }

    if (invalid_body_since_.nanoseconds() == 0) {
      invalid_body_since_ = sample_time;
    }
    // A soft grace may keep the last exact geometry usable for ordinary visual
    // continuity, but an explicit loss must break body-dependent confirmation.
    visual_reverse_takeover_count_ = 0;
    visual_reverse_takeover_last_body_generation_ = body_generation_;
    visual_reverse_takeover_pending_ = false;
    const double invalid_duration_s =
      (sample_time - invalid_body_since_).seconds();
    if (has_body_ && invalid_duration_s < body_invalid_grace_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 250,
        "Transient exact-observation loss ignored: age=%.3fs grace=%.3fs",
        invalid_duration_s, body_invalid_grace_s_);
      return;
    }

    if (has_body_) {
      if (avoidance_episode_active()) {
        avoidance_visual_loss_seen_ = true;
      }
      has_body_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Exact YOLO+skeleton target loss confirmed after %.3fs; "
        "skeleton ID is not retained for reacquisition",
        invalid_duration_s);
    }
  }

  void mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    const int previous_mode = mode_;
    mode_ = msg->data;
    if (previous_mode != mode_) {
      ++follow_session_epoch_;
      awaiting_visual_after_avoidance_ = false;
      avoidance_rearm_required_ = false;
      has_last_person_map_ = false;
      has_last_clear_person_pose_ = false;
      has_side_trigger_target_ = false;
      last_person_map_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      last_clear_person_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      clear_exact_visual_observation();
      dual_handoff_active_ = false;
      dual_handoff_reacquire_armed_ = false;
      dual_handoff_retired_by_strict_ = false;
      dual_handoff_strict_confirm_count_ = 0;
      strict_target_body_id_ = 0;
      reset_position_jump_event_context(true);
    }
    if (respect_mode_topic_ && mode_ == mode_required_) {
      paused_stop_published_ = false;
    }
    if (respect_mode_topic_ && previous_mode == mode_required_ && mode_ != mode_required_) {
      cancel_goal("mode_paused");
      cancel_spin("mode_paused");
      reset_avoidance_episode();
      reset_visual_controller();
      paused_stop_published_ = publish_stop_cmd("paused_by_mode");
    }
  }

  // The voice bearing search needs current visual observations but must not
  // allow this follower to submit a competing Nav2 goal.  This service gates
  // only actuation; subscriptions and identity tracking remain live.  Its
  // default is enabled, so existing callers retain their exact behavior.
  void motion_gate_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const bool requested_enabled = request->data;
    motion_enabled_ = requested_enabled;
    if (!motion_enabled_) {
      dual_handoff_active_ = false;
      dual_handoff_reacquire_armed_ = false;
      dual_handoff_retired_by_strict_ = false;
      dual_handoff_strict_confirm_count_ = 0;
      cancel_goal("motion_gate_disabled");
      cancel_spin("motion_gate_disabled");
      reset_avoidance_episode();
      avoidance_rearm_required_ = false;
      reset_visual_controller();
      motion_gate_stop_published_ = publish_stop_cmd("motion_gate_disabled");
      publish_state("motion_gate_observe_only");
    } else {
      motion_gate_stop_published_ = false;
      publish_state("motion_gate_enabled");
    }
    response->success = true;
    response->message = requested_enabled ?
      "body follower motion enabled" : "body follower observation-only";
    RCLCPP_INFO(
      get_logger(), "Body follower motion gate: %s",
      requested_enabled ? "enabled" : "observation-only");
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    has_cmd_ = true;
    last_observed_cmd_ = *msg;
    const auto now = this->now();
    last_cmd_time_ = now;

    const double linear = std::hypot(msg->linear.x, msg->linear.y);
    const double angular = std::fabs(msg->angular.z);
    if (linear > cmd_zero_linear_epsilon_ || angular > cmd_zero_angular_epsilon_) {
      last_cmd_motion_time_ = now;
      if (!avoidance_motion_logged_ && goal_active_ &&
          last_sent_goal_kind_ == NavGoalKind::AVOID_GATE &&
          avoidance_goal_accept_time_.nanoseconds() != 0 &&
          avoidance_start_time_.nanoseconds() != 0)
      {
        const double trigger_to_motion_ms =
          std::max(0.0, (now - avoidance_start_time_).seconds()) * 1000.0;
        const double accept_to_motion_ms =
          std::max(0.0, (now - avoidance_goal_accept_time_).seconds()) * 1000.0;
        avoidance_motion_logged_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Avoidance first nonzero cmd_vel: trigger_to_motion=%.0fms "
          "accept_to_motion=%.0fms linear=%.3f angular=%.3f",
          trigger_to_motion_ms, accept_to_motion_ms, msg->linear.x, msg->angular.z);
      }
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = *msg;
    scan_received_ = true;
    ++scan_generation_;
    last_scan_time_ = this->now();
  }

  void calibration_reference_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped source = *msg;
    if (source.header.frame_id.empty()) {
      source.header.frame_id = global_frame_;
    }

    try {
      if (source.header.frame_id == global_frame_) {
        calibration_reference_map_ = source;
      } else {
        calibration_reference_map_ = tf_buffer_->transform(
          source, global_frame_, tf2::durationFromSec(tf_timeout_s_));
      }
      calibration_reference_map_.header.frame_id = global_frame_;
      calibration_reference_map_.header.stamp = this->now();
      has_calibration_reference_ = true;
      calibration_reference_map_pub_->publish(calibration_reference_map_);
      RCLCPP_INFO(
        get_logger(), "Calibration reference updated: map=(%.3f, %.3f, %.3f)",
        calibration_reference_map_.point.x,
        calibration_reference_map_.point.y,
        calibration_reference_map_.point.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "Failed to transform calibration reference from %s to %s: %s",
        source.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    }
  }

  bool scan_recent(const rclcpp::Time & now) const
  {
    return scan_received_ && (now - last_scan_time_).seconds() <= scan_timeout_s_;
  }

  bool nearest_front_scan_ray(double & distance, double & angle) const
  {
    distance = std::numeric_limits<double>::infinity();
    angle = 0.0;
    if (!scan_received_) {
      return false;
    }

    for (std::size_t i = 0; i < last_scan_.ranges.size(); ++i) {
      const double ray_angle =
        last_scan_.angle_min + static_cast<double>(i) * last_scan_.angle_increment;
      if (std::fabs(ray_angle) > obstacle_front_angle_rad_) {
        continue;
      }

      const double range = last_scan_.ranges[i];
      if (!std::isfinite(range) || range < last_scan_.range_min || range > last_scan_.range_max) {
        continue;
      }
      if (range < distance) {
        distance = range;
        angle = ray_angle;
      }
    }
    return std::isfinite(distance);
  }

  ObstacleScan scan_obstacles(bool visual_recent) const
  {
    ObstacleScan obstacles;
    obstacles.front.sector = ObstacleSector::FRONT;
    obstacles.left.sector = ObstacleSector::LEFT;
    obstacles.right.sector = ObstacleSector::RIGHT;

    if (!scan_received_) {
      return obstacles;
    }

    double person_angle = 0.0;
    double person_distance = 0.0;
    double person_angle_tolerance = target_exemption_angle_rad_;
    const bool person_exemption =
      visual_recent && active_visual_body_.centerofmass_z > 100.0f &&
      target_exemption_distance_margin_m_ > 0.0;

    if (person_exemption) {
      const double camera_forward =
        active_visual_body_.centerofmass_z * 0.001 * BODY_NAV2_PERSON_MAP_DEPTH_SCALE;
      const double camera_lateral =
        active_visual_body_.centerofmass_x * 0.001 * BODY_NAV2_PERSON_MAP_DEPTH_SCALE;
      const double laser_forward = camera_forward - BODY_NAV2_DEPTH_CAMERA_BEHIND_LIDAR_M;
      person_angle = std::atan2(camera_lateral, laser_forward);
      person_distance = std::hypot(laser_forward, camera_lateral);
      const double max_person_angle = std::max(0.06, target_exemption_angle_rad_);
      person_angle_tolerance = clamp_value(
        std::atan2(BODY_NAV2_TARGET_HALF_WIDTH_M, std::max(0.1, person_distance)),
        0.06, max_person_angle);
    }

    const double side_min = std::max(0.0, side_obstacle_min_angle_rad_);
    const double side_max = std::max(side_min, side_obstacle_max_angle_rad_);
    double angle = last_scan_.angle_min;
    for (const auto range : last_scan_.ranges) {
      const double normalized_angle = normalize_angle(angle);
      const bool in_front = std::fabs(normalized_angle) <= obstacle_front_angle_rad_;
      const bool in_left = normalized_angle >= side_min && normalized_angle <= side_max;
      const bool in_right = normalized_angle <= -side_min && normalized_angle >= -side_max;
      const bool finite_valid =
        std::isfinite(range) &&
        range >= last_scan_.range_min &&
        range <= last_scan_.range_max;
      const bool open_return = std::isinf(range) && range > 0.0;

      // LaserScan uses +inf for a valid ray with no return inside range_max.
      // Count it as observed open space, but never as a finite obstacle.
      if (finite_valid || open_return) {
        if (in_front) {
          ++obstacles.front.finite_ray_count;
        }
        if (in_left) {
          ++obstacles.left.finite_ray_count;
        }
        if (in_right) {
          ++obstacles.right.finite_ray_count;
        }
      }

      if (finite_valid) {
        const double target_range_error = static_cast<double>(range) - person_distance;
        const double distance_margin = clamp_value(
          target_exemption_distance_margin_m_, 0.0, 0.12);
        const bool outside_safety_zone =
          static_cast<double>(range) > obstacle_slow_distance_m_;
        const bool is_target =
          person_exemption &&
          outside_safety_zone &&
          std::fabs(normalize_angle(normalized_angle - person_angle)) <=
            person_angle_tolerance &&
          std::fabs(target_range_error) <= distance_margin;

        if (!is_target) {
          const double distance = static_cast<double>(range);
          if (in_front && distance < obstacles.front.distance) {
            obstacles.front.distance = distance;
            obstacles.front.angle = normalized_angle;
          }

          if (in_left && distance < obstacles.left.distance) {
            obstacles.left.distance = distance;
            obstacles.left.angle = normalized_angle;
          }
          if (in_right && distance < obstacles.right.distance) {
            obstacles.right.distance = distance;
            obstacles.right.angle = normalized_angle;
          }
        }
      }
      angle += last_scan_.angle_increment;
    }

    return obstacles;
  }

  double front_obstacle_distance(bool visual_recent) const
  {
    return scan_obstacles(visual_recent).front.distance;
  }

  static const char * obstacle_sector_name(ObstacleSector sector)
  {
    switch (sector) {
      case ObstacleSector::FRONT:
        return "front";
      case ObstacleSector::LEFT:
        return "left";
      case ObstacleSector::RIGHT:
        return "right";
      default:
        return "none";
    }
  }

  bool obstacles_released(const ObstacleScan & obstacles) const
  {
    const bool front_released =
      !std::isfinite(obstacles.front.distance) ||
      obstacles.front.distance >= obstacle_nav_release_distance_m_;
    const bool left_released =
      !std::isfinite(obstacles.left.distance) ||
      obstacles.left.distance >= side_obstacle_release_distance_m_;
    const bool right_released =
      !std::isfinite(obstacles.right.distance) ||
      obstacles.right.distance >= side_obstacle_release_distance_m_;
    return front_released && left_released && right_released;
  }

  bool scan_sectors_valid(const ObstacleScan & obstacles) const
  {
    const auto minimum =
      static_cast<std::size_t>(visual_safety_min_finite_rays_per_sector_);
    return
      obstacles.front.finite_ray_count >= minimum &&
      obstacles.left.finite_ray_count >= minimum &&
      obstacles.right.finite_ray_count >= minimum;
  }

  void estimate_obstacle_point(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const ObstacleObservation & obstacle,
    double & obstacle_x,
    double & obstacle_y) const
  {
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double obstacle_yaw = robot_yaw + obstacle.angle;
    obstacle_x = robot_pose.pose.position.x + std::cos(obstacle_yaw) * obstacle.distance;
    obstacle_y = robot_pose.pose.position.y + std::sin(obstacle_yaw) * obstacle.distance;
  }

  void publish_calibration_sample(
    const geometry_msgs::msg::PointStamped & map_person,
    double base_x,
    double base_y,
    double base_z)
  {
    const auto sample_time = this->now();

    geometry_msgs::msg::PointStamped person_base;
    person_base.header.stamp = sample_time;
    person_base.header.frame_id = base_frame_;
    person_base.point.x = base_x;
    person_base.point.y = base_y;
    person_base.point.z = base_z;
    calibration_person_base_pub_->publish(person_base);

    geometry_msgs::msg::PoseStamped person_map;
    person_map.header.stamp = sample_time;
    person_map.header.frame_id = global_frame_;
    person_map.pose.position = map_person.point;
    tf2::Quaternion identity;
    identity.setRPY(0.0, 0.0, 0.0);
    person_map.pose.orientation = tf2::toMsg(identity);
    calibration_person_map_pub_->publish(person_map);

    geometry_msgs::msg::PoseStamped robot_pose;
    const bool robot_valid = lookup_robot_pose(robot_pose);
    double robot_yaw = 0.0;
    if (robot_valid) {
      robot_pose.header.stamp = sample_time;
      robot_pose.header.frame_id = global_frame_;
      robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
      calibration_robot_map_pub_->publish(robot_pose);
    }

    double ray_distance = std::numeric_limits<double>::infinity();
    double ray_angle = 0.0;
    const bool ray_valid = scan_recent(sample_time) &&
      nearest_front_scan_ray(ray_distance, ray_angle);
    const double raw_front_distance = scan_recent(sample_time) ?
      front_obstacle_distance(false) : std::numeric_limits<double>::infinity();
    const double effective_front_distance = scan_recent(sample_time) ?
      front_obstacle_distance(true) : std::numeric_limits<double>::infinity();

    geometry_msgs::msg::PointStamped obstacle_ray;
    geometry_msgs::msg::PointStamped obstacle_assumed;
    const bool obstacle_ray_valid = robot_valid && ray_valid;
    const bool obstacle_assumed_valid = robot_valid && std::isfinite(effective_front_distance);
    if (obstacle_ray_valid) {
      obstacle_ray.header.stamp = sample_time;
      obstacle_ray.header.frame_id = global_frame_;
      obstacle_ray.point.x = robot_pose.pose.position.x +
        std::cos(robot_yaw + ray_angle) * ray_distance;
      obstacle_ray.point.y = robot_pose.pose.position.y +
        std::sin(robot_yaw + ray_angle) * ray_distance;
      obstacle_ray.point.z = 0.0;
      calibration_obstacle_ray_pub_->publish(obstacle_ray);
    }
    if (obstacle_assumed_valid) {
      obstacle_assumed.header.stamp = sample_time;
      obstacle_assumed.header.frame_id = global_frame_;
      obstacle_assumed.point.x = robot_pose.pose.position.x +
        std::cos(robot_yaw) * effective_front_distance;
      obstacle_assumed.point.y = robot_pose.pose.position.y +
        std::sin(robot_yaw) * effective_front_distance;
      obstacle_assumed.point.z = 0.0;
      calibration_obstacle_assumed_pub_->publish(obstacle_assumed);
    }

    double reference_dx = std::numeric_limits<double>::quiet_NaN();
    double reference_dy = std::numeric_limits<double>::quiet_NaN();
    double reference_error = std::numeric_limits<double>::quiet_NaN();
    if (has_calibration_reference_) {
      calibration_reference_map_.header.stamp = sample_time;
      calibration_reference_map_pub_->publish(calibration_reference_map_);
      reference_dx = map_person.point.x - calibration_reference_map_.point.x;
      reference_dy = map_person.point.y - calibration_reference_map_.point.y;
      reference_error = std::hypot(reference_dx, reference_dy);
    }

    const double body_age = (sample_time - active_visual_body_time_).seconds();
    const double scan_age = scan_received_ ?
      (sample_time - last_scan_time_).seconds() : -1.0;
    const double clear_pose_age = has_last_clear_person_pose_ ?
      (sample_time - last_clear_person_time_).seconds() : -1.0;

    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    const auto append_number = [&out](double value) {
        if (std::isfinite(value)) {
          out << value;
        } else {
          out << "null";
        }
      };

    out << "{\"stamp_ns\":" << sample_time.nanoseconds()
        << ",\"state\":\"" << last_state_ << "\""
        << ",\"mode\":" << mode_
        << ",\"avoidance_active\":" << (avoidance_episode_active() ? "true" : "false")
        << ",\"body\":{\"id\":" << active_visual_body_.bodyid
        << ",\"lock\":" << static_cast<int>(active_visual_body_.lock_status)
        << ",\"age_s\":" << body_age
        << ",\"raw_mm\":[" << active_visual_body_.centerofmass_x << ","
        << active_visual_body_.centerofmass_y << "," << active_visual_body_.centerofmass_z << "]}"
        << ",\"person_base_m\":[" << base_x << "," << base_y << "," << base_z << "]"
        << ",\"person_map_m\":[" << map_person.point.x << ","
        << map_person.point.y << "," << map_person.point.z << "]"
        << ",\"robot_map\":{\"valid\":" << (robot_valid ? "true" : "false");
    if (robot_valid) {
      out << ",\"x\":" << robot_pose.pose.position.x
          << ",\"y\":" << robot_pose.pose.position.y
          << ",\"yaw\":" << robot_yaw;
    }
    out << "}"
        << ",\"scan\":{\"age_s\":" << scan_age
        << ",\"raw_front_m\":";
    append_number(raw_front_distance);
    out << ",\"effective_front_m\":";
    append_number(effective_front_distance);
    out << ",\"nearest_ray_valid\":" << (ray_valid ? "true" : "false")
        << ",\"nearest_ray_m\":";
    append_number(ray_distance);
    out << ",\"nearest_ray_angle_rad\":";
    append_number(ray_valid ? ray_angle : std::numeric_limits<double>::quiet_NaN());
    out << "}"
        << ",\"obstacle_ray_map\":{\"valid\":"
        << (obstacle_ray_valid ? "true" : "false");
    if (obstacle_ray_valid) {
      out << ",\"x\":" << obstacle_ray.point.x << ",\"y\":" << obstacle_ray.point.y;
    }
    out << "}"
        << ",\"obstacle_assumed_map\":{\"valid\":"
        << (obstacle_assumed_valid ? "true" : "false");
    if (obstacle_assumed_valid) {
      out << ",\"x\":" << obstacle_assumed.point.x
          << ",\"y\":" << obstacle_assumed.point.y;
    }
    out << "}"
        << ",\"reference\":{\"valid\":"
        << (has_calibration_reference_ ? "true" : "false");
    if (has_calibration_reference_) {
      out << ",\"x\":" << calibration_reference_map_.point.x
          << ",\"y\":" << calibration_reference_map_.point.y
          << ",\"dx\":" << reference_dx
          << ",\"dy\":" << reference_dy
          << ",\"error_m\":" << reference_error;
    }
    out << "}"
        << ",\"last_clear_person\":{\"valid\":"
        << (has_last_clear_person_pose_ ? "true" : "false")
        << ",\"age_s\":" << clear_pose_age;
    if (has_last_clear_person_pose_) {
      out << ",\"x\":" << last_clear_person_pose_.pose.position.x
          << ",\"y\":" << last_clear_person_pose_.pose.position.y;
    }
    out << "}"
        << ",\"latched_goal\":{\"valid\":"
        << (has_avoidance_gate_goal_ ? "true" : "false");
    if (has_avoidance_gate_goal_) {
      out << ",\"x\":" << avoidance_gate_goal_.pose.position.x
          << ",\"y\":" << avoidance_gate_goal_.pose.position.y
          << ",\"yaw\":" << tf2::getYaw(avoidance_gate_goal_.pose.orientation);
    }
    out << "}"
        << ",\"cmd_vel\":{\"linear_x\":" << last_observed_cmd_.linear.x
        << ",\"angular_z\":" << last_observed_cmd_.angular.z << "}}";

    std_msgs::msg::String sample;
    sample.data = out.str();
    calibration_sample_pub_->publish(sample);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "CALIB body=%d raw_mm=(%.0f,%.0f,%.0f) base=(%.3f,%.3f) map=(%.3f,%.3f) ref_err=%.3f scan=(%.3f@%.3f) effective=%.3f",
      active_visual_body_.bodyid,
      active_visual_body_.centerofmass_x, active_visual_body_.centerofmass_y,
      active_visual_body_.centerofmass_z,
      base_x, base_y, map_person.point.x, map_person.point.y, reference_error,
      ray_distance, ray_angle, effective_front_distance);
  }

  void publish_paused_calibration(const rclcpp::Time & now)
  {
    if (!has_body_ || (now - last_body_time_).seconds() > lost_timeout_s_) {
      return;
    }

    active_visual_body_ = latest_body_;
    active_visual_body_time_ = last_body_time_;

    double person_x = 0.0;
    double person_y = 0.0;
    double person_z = 0.0;
    if (!person_in_base(person_x, person_y, person_z)) {
      return;
    }

    geometry_msgs::msg::PointStamped base_person;
    base_person.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    base_person.header.frame_id = base_frame_;
    base_person.point.x = person_x;
    base_person.point.y = person_y;
    base_person.point.z = person_z;

    try {
      const auto map_person = tf_buffer_->transform(
        base_person, global_frame_, tf2::durationFromSec(tf_timeout_s_));
      publish_calibration_sample(map_person, person_x, person_y, person_z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform paused calibration point from %s to %s: %s",
        base_frame_.c_str(), global_frame_.c_str(), ex.what());
    }
  }

  bool extend_goal_from_obstacle(
    double obstacle_x,
    double obstacle_y,
    double & target_x,
    double & target_y) const
  {
    const double dx = target_x - obstacle_x;
    const double dy = target_y - obstacle_y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1e-3) {
      return false;
    }

    target_x += dx / distance * BODY_NAV2_AVOIDANCE_GOAL_ELASTIC_M;
    target_y += dy / distance * BODY_NAV2_AVOIDANCE_GOAL_ELASTIC_M;
    return true;
  }

  void fill_map_goal(
    geometry_msgs::msg::PoseStamped & goal,
    double x,
    double y,
    double yaw)
  {
    goal.header.stamp = this->now();
    goal.header.frame_id = global_frame_;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    goal.pose.orientation = tf2::toMsg(q);
  }

  void remember_last_clear_person_pose()
  {
    last_clear_person_pose_ = last_person_map_pose_;
    last_clear_person_time_ = this->now();
    has_last_clear_person_pose_ = true;
  }

  bool make_avoidance_gate_goal(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const ObstacleObservation & obstacle,
    geometry_msgs::msg::PoseStamped & gate_goal)
  {
    if (!enable_avoidance_side_goal_ ||
        obstacle.sector == ObstacleSector::NONE ||
        !std::isfinite(obstacle.distance)) {
      return false;
    }

    const auto now = this->now();
    double obstacle_x = 0.0;
    double obstacle_y = 0.0;
    estimate_obstacle_point(robot_pose, obstacle, obstacle_x, obstacle_y);

    const bool side_obstacle =
      obstacle.sector == ObstacleSector::LEFT ||
      obstacle.sector == ObstacleSector::RIGHT;
    double target_x = 0.0;
    double target_y = 0.0;
    const char * source = nullptr;

    if (side_obstacle) {
      if (!has_side_trigger_target_) {
        publish_state("nav2_side_avoid_wait_latched_person_pose");
        return false;
      }
      target_x = side_trigger_target_pose_.pose.position.x;
      target_y = side_trigger_target_pose_.pose.position.y;
      source = "visual_person_latest_side_latched";
    } else {
      const bool has_recent_clear_person =
        has_last_clear_person_pose_ &&
        (now - last_clear_person_time_).seconds() <= target_memory_timeout_s_;
      if (!has_recent_clear_person) {
        publish_state("nav2_avoid_wait_clear_person_pose");
        return false;
      }

      target_x = last_clear_person_pose_.pose.position.x;
      target_y = last_clear_person_pose_.pose.position.y;
      source = "visual_person_history";
      if (!extend_goal_from_obstacle(obstacle_x, obstacle_y, target_x, target_y)) {
        publish_state("nav2_avoid_invalid_clear_person_pose");
        return false;
      }
    }

    const double yaw_from_obstacle = std::atan2(target_y - obstacle_y, target_x - obstacle_x);

    fill_map_goal(gate_goal, target_x, target_y, yaw_from_obstacle);
    calibration_latched_goal_pub_->publish(gate_goal);
    last_avoidance_goal_time_ = now;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Avoidance gate goal: source=%s sector=%s range=%.2f angle=%.2f obstacle=(%.2f, %.2f) goal=(%.2f, %.2f) elastic=%.2f yaw=%.2f",
      source, obstacle_sector_name(obstacle.sector), obstacle.distance, obstacle.angle,
      obstacle_x, obstacle_y, target_x, target_y,
      BODY_NAV2_AVOIDANCE_GOAL_ELASTIC_M, yaw_from_obstacle);
    return true;
  }

  bool avoidance_episode_active() const
  {
    return avoidance_stage_ != AvoidanceStage::IDLE ||
      ((goal_active_ || has_sent_goal_) && last_sent_goal_kind_ == NavGoalKind::AVOID_GATE);
  }

  bool avoidance_trigger_confirmed(
    const ObstacleScan & obstacles,
    ObstacleObservation & triggered_obstacle)
  {
    if (avoidance_episode_active()) {
      triggered_obstacle = active_avoidance_obstacle_;
      return true;
    }

    const bool front_close =
      std::isfinite(obstacles.front.distance) &&
      obstacles.front.distance <= obstacle_nav_distance_m_;
    const bool left_close =
      std::isfinite(obstacles.left.distance) &&
      obstacles.left.distance <= side_obstacle_nav_distance_m_;
    const bool right_close =
      std::isfinite(obstacles.right.distance) &&
      obstacles.right.distance <= side_obstacle_nav_distance_m_;

    const bool any_close = front_close || left_close || right_close;
    const bool first_close_sample =
      any_close && avoidance_trigger_count_ == 0 &&
      left_avoidance_trigger_count_ == 0 &&
      right_avoidance_trigger_count_ == 0;
    if (first_close_sample) {
      avoidance_visual_loss_seen_ = false;
      reset_visual_reacquire_takeover_confirmation();
      avoidance_start_time_ = this->now();
      avoidance_goal_accept_time_ =
        rclcpp::Time(0, 0, get_clock()->get_clock_type());
      avoidance_motion_logged_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Avoidance threshold entered: front=%.3fm left=%.3fm right=%.3fm confirm=%d ticks",
        obstacles.front.distance, obstacles.left.distance, obstacles.right.distance,
        BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS);
    } else if (!any_close) {
      avoidance_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    const bool side_close = left_close || right_close;
    if (side_close && !has_side_trigger_target_ && has_last_person_map_) {
      side_trigger_target_pose_ = last_person_map_pose_;
      has_side_trigger_target_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Latched latest side-avoidance person target at (%.2f, %.2f)",
        side_trigger_target_pose_.pose.position.x,
        side_trigger_target_pose_.pose.position.y);
    }

    const bool side_released =
      (!std::isfinite(obstacles.left.distance) ||
      obstacles.left.distance >= side_obstacle_release_distance_m_) &&
      (!std::isfinite(obstacles.right.distance) ||
      obstacles.right.distance >= side_obstacle_release_distance_m_);
    if (!avoidance_episode_active() && side_released) {
      has_side_trigger_target_ = false;
    }

    avoidance_trigger_count_ = front_close ? std::min(
      avoidance_trigger_count_ + 1, BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS) : 0;
    left_avoidance_trigger_count_ = left_close ? std::min(
      left_avoidance_trigger_count_ + 1, BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS) : 0;
    right_avoidance_trigger_count_ = right_close ? std::min(
      right_avoidance_trigger_count_ + 1, BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS) : 0;

    double best_score = std::numeric_limits<double>::infinity();
    if (avoidance_trigger_count_ >= BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS) {
      best_score = obstacles.front.distance / std::max(0.01, obstacle_nav_distance_m_);
      triggered_obstacle = obstacles.front;
    }
    if (left_avoidance_trigger_count_ >= BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS) {
      const double score =
        obstacles.left.distance / std::max(0.01, side_obstacle_nav_distance_m_);
      if (score < best_score) {
        best_score = score;
        triggered_obstacle = obstacles.left;
      }
    }
    if (right_avoidance_trigger_count_ >= BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS) {
      const double score =
        obstacles.right.distance / std::max(0.01, side_obstacle_nav_distance_m_);
      if (score < best_score) {
        triggered_obstacle = obstacles.right;
      }
    }

    const bool confirmed = triggered_obstacle.sector != ObstacleSector::NONE;
    if (confirmed) {
      const double confirm_ms =
        avoidance_start_time_.nanoseconds() == 0 ? -1.0 :
        std::max(0.0, (this->now() - avoidance_start_time_).seconds()) * 1000.0;
      RCLCPP_INFO(
        get_logger(),
        "Avoidance trigger confirmed: sector=%s distance=%.3fm elapsed=%.0fms",
        obstacle_sector_name(triggered_obstacle.sector),
        triggered_obstacle.distance, confirm_ms);
    }
    return confirmed;
  }

  bool make_avoidance_nav_goal(
    geometry_msgs::msg::PoseStamped & map_goal,
    const ObstacleObservation & obstacle,
    NavGoalKind & goal_kind)
  {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!lookup_robot_pose(robot_pose)) {
      publish_state("tf_failed");
      return false;
    }

    const auto now = this->now();
    if (avoidance_stage_ == AvoidanceStage::IDLE) {
      if (!make_avoidance_gate_goal(robot_pose, obstacle, avoidance_gate_goal_)) {
        goal_kind = NavGoalKind::NORMAL;
        return false;
      }

      avoidance_stage_ = AvoidanceStage::GATE;
      has_avoidance_gate_goal_ = true;
      active_avoidance_obstacle_ = obstacle;
      avoidance_trigger_count_ = BODY_NAV2_AVOIDANCE_TRIGGER_CONFIRM_TICKS;
      if (avoidance_start_time_.nanoseconds() == 0) {
        avoidance_start_time_ = now;
      }
    }

    if (!has_avoidance_gate_goal_) {
      return false;
    }

    map_goal = avoidance_gate_goal_;
    map_goal.header.stamp = now;
    goal_kind = NavGoalKind::AVOID_GATE;
    last_avoidance_goal_time_ = now;
    publish_state("nav2_avoid_gate_goal_ready");
    return true;
  }

  void reset_avoidance_episode(bool preserve_takeover_phase = false)
  {
    avoidance_stage_ = AvoidanceStage::IDLE;
    last_sent_goal_kind_ = NavGoalKind::NORMAL;
    has_avoidance_gate_goal_ = false;
    avoidance_trigger_count_ = 0;
    left_avoidance_trigger_count_ = 0;
    right_avoidance_trigger_count_ = 0;
    active_avoidance_obstacle_ = ObstacleObservation();
    visual_reverse_takeover_count_ = 0;
    visual_reverse_takeover_last_body_generation_ = body_generation_;
    visual_reverse_takeover_pending_ = false;
    avoidance_visual_loss_seen_ = false;
    dual_takeover_gate_open_ = false;
    dual_handoff_retired_by_strict_ = false;
    reset_visual_clear_takeover_confirmation();
    reset_visual_reacquire_takeover_confirmation(dual_handoff_active_);
    reset_visual_safety_release_confirmation();
    if (!preserve_takeover_phase) {
      visual_takeover_phase_ = VisualTakeoverPhase::NONE;
      reset_navigation_episode_tracking();
    }
    has_side_trigger_target_ = false;
    avoidance_motion_logged_ = false;
    avoidance_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    avoidance_goal_accept_time_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  bool recent_avoidance_goal(const rclcpp::Time & now) const
  {
    if (avoidance_occlusion_hold_s_ <= 0.0) {
      return false;
    }

    return (now - last_avoidance_goal_time_).seconds() <= avoidance_occlusion_hold_s_;
  }

  double safety_scale_within_band(
    double distance, double trigger_distance, double release_distance) const
  {
    if (!std::isfinite(distance) || distance >= release_distance) {
      return 1.0;
    }
    if (distance <= trigger_distance) {
      return 0.0;
    }

    const double span = release_distance - trigger_distance;
    if (span <= 1e-3) {
      return 0.0;
    }

    const double slow_entry_scale =
      clamp_value(BODY_NAV2_SAFETY_SLOW_ENTRY_SCALE, 0.0, 1.0);
    const double pretrigger_scale =
      clamp_value(BODY_NAV2_SAFETY_PRETRIGGER_SCALE, 0.0, slow_entry_scale);
    const double ratio =
      clamp_value((distance - trigger_distance) / span, 0.0, 1.0);
    return pretrigger_scale +
      (slow_entry_scale - pretrigger_scale) * ratio;
  }

  double visual_scale_from_obstacle(double front_distance) const
  {
    return safety_scale_within_band(
      front_distance, obstacle_nav_distance_m_, obstacle_slow_distance_m_);
  }

  double side_visual_scale_from_obstacle(double side_distance) const
  {
    return safety_scale_within_band(
      side_distance, side_obstacle_nav_distance_m_,
      side_obstacle_release_distance_m_);
  }

  double visual_scale_from_obstacles(const ObstacleScan & obstacles) const
  {
    return std::min({
      visual_scale_from_obstacle(obstacles.front.distance),
      side_visual_scale_from_obstacle(obstacles.left.distance),
      side_visual_scale_from_obstacle(obstacles.right.distance)});
  }

  geometry_msgs::msg::Twist smooth_direct_cmd(
    const geometry_msgs::msg::Twist & desired,
    const rclcpp::Time & now,
    double decel_limit_override = -1.0)
  {
    if (!direct_cmd_init_) {
      direct_cmd_init_ = true;
      last_direct_cmd_time_ = now;
      last_direct_cmd_ = geometry_msgs::msg::Twist();
    }

    double dt = (now - last_direct_cmd_time_).seconds();
    if (dt <= 1e-3 || dt > 0.5) {
      dt = 0.1;
    }

    geometry_msgs::msg::Twist cmd = desired;
    double linear_target = desired.linear.x;
    const bool reversing_direction =
      last_direct_cmd_.linear.x * desired.linear.x < 0.0;
    if (reversing_direction) {
      linear_target = 0.0;
    }

    const bool speeding_up =
      !reversing_direction &&
      std::fabs(linear_target) > std::fabs(last_direct_cmd_.linear.x);
    const double accel_limit = desired.linear.x < 0.0 ?
      visual_reverse_accel_limit_ : visual_linear_accel_limit_;
    const double decel_limit = decel_limit_override >= 0.0 ?
      decel_limit_override : visual_linear_decel_limit_;
    const double linear_limit = speeding_up ? accel_limit : decel_limit;
    const double max_linear_delta = std::max(0.0, linear_limit) * dt;
    const double max_angular_delta = std::max(0.0, visual_angular_accel_limit_) * dt;

    cmd.linear.x = last_direct_cmd_.linear.x + clamp_value(
      linear_target - last_direct_cmd_.linear.x,
      -max_linear_delta, max_linear_delta);
    if (std::fabs(cmd.linear.x) < BODY_NAV2_VISUAL_STOP_EPSILON_MPS) {
      cmd.linear.x = 0.0;
    }
    cmd.angular.z = last_direct_cmd_.angular.z + clamp_value(
      desired.angular.z - last_direct_cmd_.angular.z,
      -max_angular_delta, max_angular_delta);

    last_direct_cmd_ = cmd;
    last_direct_cmd_time_ = now;
    return cmd;
  }

  bool publish_stop_cmd(const std::string & state)
  {
    geometry_msgs::msg::Twist stop;
    const auto cmd = smooth_direct_cmd(stop, this->now());
    visual_cmd_pub_->publish(cmd);
    publish_state(state);
    return cmd.linear.x == 0.0 && cmd.angular.z == 0.0;
  }

  void publish_safety_stop_cmd(const std::string & state)
  {
    geometry_msgs::msg::Twist stop;
    const auto now = this->now();
    direct_cmd_init_ = true;
    last_direct_cmd_ = stop;
    last_direct_cmd_time_ = now;
    visual_cmd_pub_->publish(stop);
    publish_state(state);
  }

  bool publish_visual_follow_cmd(
    double linear_scale,
    const std::string & state,
    double decel_limit_override = -1.0,
    double minimum_forward_mps = 0.0,
    bool scale_angular_fully = false)
  {
    const auto now = this->now();
    const double raw_x_angle =
      active_visual_body_.centerofmass_x / active_visual_body_.centerofmass_z;
    const double raw_distance_mm = active_visual_body_.centerofmass_z;

    if (!visual_filter_init_) {
      filtered_x_angle_ = raw_x_angle;
      filtered_distance_mm_ = raw_distance_mm;
      visual_filter_init_ = true;
    } else {
      const double alpha = clamp_value(visual_filter_alpha_, 0.0, 1.0);
      filtered_x_angle_ = alpha * raw_x_angle + (1.0 - alpha) * filtered_x_angle_;
      filtered_distance_mm_ =
        alpha * raw_distance_mm + (1.0 - alpha) * filtered_distance_mm_;
    }

    double error_x_angle = filtered_x_angle_;
    double error_distance = filtered_distance_mm_ - follow_distance_m_ * 1000.0;
    const double raw_distance_error_m = error_distance * 0.001;
    const double distance_deadband_m =
      std::max(0.0, visual_distance_deadband_mm_) * 0.001;
    const double hold_exit_distance_m =
      std::max(distance_deadband_m, std::max(0.0, hold_distance_band_m_));

    if (visual_hold_active_) {
      if (std::fabs(raw_distance_error_m) > hold_exit_distance_m) {
        visual_hold_active_ = false;
      }
    } else if (std::fabs(raw_distance_error_m) <= distance_deadband_m) {
      visual_hold_active_ = true;
    }

    if (std::fabs(error_x_angle) < visual_angle_deadband_) {
      error_x_angle = 0.0;
    }
    if (visual_hold_active_) {
      error_distance = 0.0;
    }

    geometry_msgs::msg::Twist desired;
    const double full_speed_error_m = std::max(
      distance_deadband_m + 1e-3, BODY_NAV2_VISUAL_FULL_SPEED_ERROR_M);
    const double effective_distance_error_m = std::max(
      0.0, std::fabs(raw_distance_error_m) - distance_deadband_m);
    const double distance_speed_ratio = clamp_value(
      effective_distance_error_m / (full_speed_error_m - distance_deadband_m),
      0.0, 1.0);
    const double max_reverse_mps =
      visual_max_linear_mps_ * BODY_NAV2_VISUAL_REVERSE_MAX_RATIO;

    if (!visual_hold_active_ && raw_distance_error_m > distance_deadband_m) {
      desired.linear.x = visual_max_linear_mps_ * distance_speed_ratio;
    } else if (!visual_hold_active_ && visual_allow_reverse_ &&
      raw_distance_error_m < -distance_deadband_m)
    {
      desired.linear.x = -max_reverse_mps * distance_speed_ratio;
    }

    if (!visual_hold_active_ && desired.linear.x != 0.0) {
      const double braking_distance_m = std::max(
        0.0, std::fabs(raw_distance_error_m) - distance_deadband_m);
      const double braking_accel_mps2 = std::max(
        1e-3, desired.linear.x < 0.0 ?
        visual_reverse_accel_limit_ : visual_linear_accel_limit_);
      const double braking_speed_mps =
        std::sqrt(2.0 * braking_accel_mps2 * braking_distance_m);
      desired.linear.x = std::copysign(
        std::min(std::fabs(desired.linear.x), braking_speed_mps), desired.linear.x);
    }
    desired.angular.z =
      error_x_angle * visual_z_p_ +
      (error_x_angle - last_visual_error_angle_) * visual_z_d_;

    const double safe_linear_scale = clamp_value(linear_scale, 0.0, 1.0);
    desired.linear.x *= safe_linear_scale;
    desired.angular.z *= scale_angular_fully ?
      safe_linear_scale : std::max(safe_linear_scale, 0.55);

    if (desired.linear.x > 0.0 && minimum_forward_mps > 0.0) {
      desired.linear.x = std::max(desired.linear.x, minimum_forward_mps);
    }

    if (!visual_allow_reverse_ && desired.linear.x < 0.0) {
      desired.linear.x = 0.0;
    }

    desired.linear.x = clamp_value(
      desired.linear.x, visual_allow_reverse_ ? -max_reverse_mps : 0.0,
      visual_max_linear_mps_);
    desired.angular.z = clamp_value(
      desired.angular.z, -visual_max_angular_rps_, visual_max_angular_rps_);

    if (std::fabs(desired.linear.x) < 0.02) {
      desired.linear.x = 0.0;
    }
    if (std::fabs(desired.angular.z) < 0.02) {
      desired.angular.z = 0.0;
    }

    const double decel_limit = decel_limit_override >= 0.0 ?
      decel_limit_override :
      (desired.linear.x < 0.0 ? visual_reverse_accel_limit_ : visual_linear_accel_limit_);
    const auto cmd = smooth_direct_cmd(desired, now, decel_limit);
    visual_cmd_pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Visual cmd source=%s state=%s body=%d raw_x=%.0fmm raw_z=%.0fmm "
      "scale=%.2f linear=%.3f angular=%.3f",
      active_visual_body_is_dual_ ? "raw_dual" : "strict_validated",
      state.c_str(), active_visual_body_.bodyid,
      active_visual_body_.centerofmass_x, active_visual_body_.centerofmass_z,
      safe_linear_scale, cmd.linear.x, cmd.angular.z);
    last_visual_error_angle_ = error_x_angle;
    last_visual_error_distance_ = error_distance;
    publish_state(visual_hold_active_ ? "visual_distance_hold" : state);
    return true;
  }

  void reset_visual_controller()
  {
    visual_filter_init_ = false;
    visual_hold_active_ = false;
    filtered_distance_mm_ = 0.0;
    last_visual_error_angle_ = 0.0;
    last_visual_error_distance_ = 0.0;
  }

  void clear_exact_visual_observation()
  {
    has_body_ = false;
    latest_body_ = bodyreader_msg::msg::Bodyposture();
    active_visual_body_ = bodyreader_msg::msg::Bodyposture();
    active_visual_body_is_dual_ = false;
    last_body_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    active_visual_body_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    invalid_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    exact_observation_accept_after_ns_ =
      now().nanoseconds() + kExactObservationFlushNs;
    reset_visual_clear_takeover_confirmation();
    reset_visual_reacquire_takeover_confirmation();
    reset_visual_safety_release_confirmation();
    visual_reverse_takeover_count_ = 0;
    visual_reverse_takeover_last_body_generation_ = body_generation_;
    visual_reverse_takeover_pending_ = false;
  }

  void reset_visual_clear_takeover_confirmation()
  {
    visual_clear_takeover_count_ = 0;
    visual_clear_takeover_last_scan_generation_ = scan_generation_;
  }

  void reset_visual_reacquire_takeover_confirmation(
    bool preserve_latest_sample = false)
  {
    visual_reacquire_takeover_count_ = 0;
    visual_reacquire_body_id_ = 0;
    if (!preserve_latest_sample) {
      last_dual_presence_body_time_ =
        rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
  }

  double dual_handoff_freshness_s() const
  {
    return std::max(
      visual_reacquire_takeover_body_max_age_s_,
      visual_reacquire_takeover_max_frame_interval_s_);
  }

  bool visual_takeover_progress_clearance_ready(
    const rclcpp::Time & now,
    const ObstacleScan & obstacles,
    bool avoidance_active,
    bool nav_engaged) const
  {
    if (!avoidance_active || !nav_engaged || !scan_recent(now) ||
        !scan_sectors_valid(obstacles) ||
        avoidance_start_time_.nanoseconds() == 0 ||
        (now - avoidance_start_time_).seconds() <
        visual_clear_takeover_min_avoidance_s_)
    {
      return false;
    }

    const bool front_clear =
      !std::isfinite(obstacles.front.distance) ||
      obstacles.front.distance >= visual_clear_takeover_front_clearance_m_;
    const bool left_clear =
      !std::isfinite(obstacles.left.distance) ||
      obstacles.left.distance >= visual_clear_takeover_side_clearance_m_;
    const bool right_clear =
      !std::isfinite(obstacles.right.distance) ||
      obstacles.right.distance >= visual_clear_takeover_side_clearance_m_;
    return front_clear && left_clear && right_clear;
  }

  bool visual_reacquire_takeover_confirmed(
    const rclcpp::Time & now,
    bool avoidance_active,
    bool nav_engaged) const
  {
    const bool body_fresh =
      last_dual_presence_body_time_.nanoseconds() != 0 &&
      (now - last_dual_presence_body_time_).seconds() <=
      dual_handoff_freshness_s();
    return avoidance_active && nav_engaged && body_fresh &&
      visual_reacquire_takeover_count_ >= visual_reacquire_takeover_confirm_frames_;
  }

  void reset_visual_safety_release_confirmation()
  {
    visual_safety_release_count_ = 0;
    visual_safety_release_last_scan_generation_ = scan_generation_;
  }

  bool visual_safety_release_confirmed(const ObstacleScan & obstacles)
  {
    if (!scan_sectors_valid(obstacles) || !obstacles_released(obstacles)) {
      reset_visual_safety_release_confirmation();
      return false;
    }
    if (visual_safety_release_last_scan_generation_ == scan_generation_) {
      return false;
    }

    visual_safety_release_last_scan_generation_ = scan_generation_;
    visual_safety_release_count_ = std::min(
      visual_safety_release_count_ + 1, visual_safety_release_confirm_frames_);
    if (visual_safety_release_count_ < visual_safety_release_confirm_frames_) {
      publish_state("visual_takeover_safety_release_confirming");
      return false;
    }
    return true;
  }

  static bool navigation_goal_status_active(int8_t status)
  {
    using GoalStatus = action_msgs::msg::GoalStatus;
    return status == GoalStatus::STATUS_ACCEPTED ||
      status == GoalStatus::STATUS_EXECUTING ||
      status == GoalStatus::STATUS_CANCELING;
  }

  static bool navigation_goal_status_cancelable(int8_t status)
  {
    using GoalStatus = action_msgs::msg::GoalStatus;
    return status == GoalStatus::STATUS_ACCEPTED ||
      status == GoalStatus::STATUS_EXECUTING;
  }

  static bool navigation_goal_status_terminal(int8_t status)
  {
    using GoalStatus = action_msgs::msg::GoalStatus;
    return status == GoalStatus::STATUS_SUCCEEDED ||
      status == GoalStatus::STATUS_CANCELED ||
      status == GoalStatus::STATUS_ABORTED;
  }

  static std::string goal_uuid_string(const GoalUUID & uuid)
  {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte : uuid) {
      stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
  }

  void attach_navigation_status_subscription(
    const std::shared_ptr<NavigationActionTracker> & tracker)
  {
    const std::weak_ptr<NavigationActionTracker> weak_tracker = tracker;
    tracker->status_sub = create_subscription<action_msgs::msg::GoalStatusArray>(
      tracker->action_name + "/_action/status",
      rclcpp_action::DefaultActionStatusQoS(),
      [this, weak_tracker](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        const auto tracker = weak_tracker.lock();
        if (!tracker) {
          return;
        }
        const int64_t receipt_ns = this->now().nanoseconds();
        tracker->active_goal_ids_in_latest_status.clear();
        for (const auto & status : msg->status_list) {
          if (navigation_goal_status_active(status.status)) {
            tracker->active_goal_ids_in_latest_status.insert(
              status.goal_info.goal_id.uuid);
          }
        }
        if (!navigation_episode_tracking_) {
          return;
        }

        std::set<GoalUUID> ids_in_snapshot;
        for (const auto & status : msg->status_list) {
          const GoalUUID uuid = status.goal_info.goal_id.uuid;
          ids_in_snapshot.insert(uuid);
          const int64_t accepted_ns = rclcpp::Time(
            status.goal_info.stamp, get_clock()->get_clock_type()).nanoseconds();
          const int64_t capture_floor_ns = navigation_episode_start_ns_ -
            static_cast<int64_t>(nav_takeover_child_capture_slop_s_ * 1e9);
          const int64_t capture_ceiling_ns = navigation_parent_terminal_ ?
            navigation_parent_terminal_ns_ +
            static_cast<int64_t>(nav_takeover_child_capture_slop_s_ * 1e9) :
            std::numeric_limits<int64_t>::max();
          const bool eligible_for_episode =
            accepted_ns > 0 && accepted_ns >= capture_floor_ns &&
            accepted_ns <= capture_ceiling_ns;
          if (eligible_for_episode &&
              status.status != action_msgs::msg::GoalStatus::STATUS_UNKNOWN)
          {
            const bool inserted = tracker->episode_goal_ids.insert(uuid).second;
            if (inserted) {
              RCLCPP_INFO(
                get_logger(),
                "Captured navigation child action: action=%s uuid=%s status=%d",
                tracker->action_name.c_str(), goal_uuid_string(uuid).c_str(),
                static_cast<int>(status.status));
            }
          }

          if (tracker->episode_goal_ids.count(uuid) != 0U) {
            tracker->goal_statuses[uuid] = status.status;
            tracker->last_goal_status_update_ns[uuid] = receipt_ns;
          }
        }

        for (const auto & owned_uuid : tracker->episode_goal_ids) {
          if (ids_in_snapshot.count(owned_uuid) == 0U) {
            tracker->goal_statuses[owned_uuid] =
              action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
            tracker->last_goal_status_update_ns[owned_uuid] = receipt_ns;
          }
        }

        if (!tracker->active_goal_ids_in_latest_status.empty()) {
          navigation_children_idle_since_ns_ = 0;
        }
      });
  }

  void create_navigation_action_tracker(const std::string & action_name)
  {
    auto tracker = std::make_shared<NavigationActionTracker>();
    tracker->action_name = action_name;
    tracker->cancel_client = create_client<action_msgs::srv::CancelGoal>(
      action_name + "/_action/cancel_goal");
    attach_navigation_status_subscription(tracker);
    navigation_action_trackers_.push_back(tracker);
  }

  void start_navigation_episode_tracking()
  {
    navigation_episode_tracking_ = true;
    navigation_parent_terminal_ = false;
    navigation_episode_start_ns_ = this->now().nanoseconds();
    navigation_parent_terminal_ns_ = 0;
    navigation_drain_start_ns_ = 0;
    navigation_children_idle_since_ns_ = 0;
    navigation_parent_outcome_.clear();
    ++navigation_episode_sequence_;
    for (const auto & tracker : navigation_action_trackers_) {
      tracker->episode_goal_ids.clear();
      tracker->goal_statuses.clear();
      tracker->last_goal_status_update_ns.clear();
      tracker->last_cancel_request_ns.clear();
      tracker->cancel_request_sequences.clear();
      tracker->cancel_requests_in_flight.clear();
    }
    RCLCPP_INFO(
      get_logger(), "Started navigation child tracking: episode=%llu",
      static_cast<unsigned long long>(navigation_episode_sequence_));
  }

  void reset_navigation_episode_tracking()
  {
    navigation_episode_tracking_ = false;
    navigation_parent_terminal_ = false;
    navigation_episode_start_ns_ = 0;
    navigation_parent_terminal_ns_ = 0;
    navigation_drain_start_ns_ = 0;
    navigation_children_idle_since_ns_ = 0;
    navigation_parent_outcome_.clear();
    ++navigation_episode_sequence_;
    for (const auto & tracker : navigation_action_trackers_) {
      tracker->episode_goal_ids.clear();
      tracker->goal_statuses.clear();
      tracker->last_goal_status_update_ns.clear();
      tracker->last_cancel_request_ns.clear();
      tracker->cancel_request_sequences.clear();
      tracker->cancel_requests_in_flight.clear();
    }
  }

  void cancel_tracked_navigation_children()
  {
    if (!navigation_episode_tracking_) {
      return;
    }
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t retry_ns = static_cast<int64_t>(nav_takeover_cancel_retry_s_ * 1e9);
    const auto episode_sequence = navigation_episode_sequence_;
    for (const auto & tracker : navigation_action_trackers_) {
      for (const auto & uuid : tracker->episode_goal_ids) {
        const auto status_it = tracker->goal_statuses.find(uuid);
        if (status_it == tracker->goal_statuses.end() ||
            !navigation_goal_status_cancelable(status_it->second))
        {
          continue;
        }
        const auto cancel_it = tracker->last_cancel_request_ns.find(uuid);
        if (tracker->cancel_requests_in_flight.count(uuid) != 0U) {
          if (cancel_it != tracker->last_cancel_request_ns.end() &&
              now_ns - cancel_it->second < retry_ns)
          {
            continue;
          }
          tracker->cancel_requests_in_flight.erase(uuid);
          RCLCPP_WARN(
            get_logger(),
            "Navigation child cancel response timed out; retry exact UUID: action=%s uuid=%s",
            tracker->action_name.c_str(), goal_uuid_string(uuid).c_str());
        } else if (cancel_it != tracker->last_cancel_request_ns.end() &&
          now_ns - cancel_it->second < retry_ns)
        {
          continue;
        }
        if (!tracker->cancel_client->service_is_ready()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Navigation child cancel service unavailable: %s/_action/cancel_goal",
            tracker->action_name.c_str());
          navigation_children_idle_since_ns_ = 0;
          continue;
        }

        auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
        request->goal_info.goal_id.uuid = uuid;
        request->goal_info.stamp.sec = 0;
        request->goal_info.stamp.nanosec = 0;
        tracker->last_cancel_request_ns[uuid] = now_ns;
        const uint64_t request_sequence = ++tracker->cancel_request_sequences[uuid];
        tracker->cancel_requests_in_flight.insert(uuid);
        navigation_children_idle_since_ns_ = 0;
        tracker->cancel_client->async_send_request(
          request,
          [this, tracker, uuid, episode_sequence, request_sequence](
            rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future) {
            if (episode_sequence != navigation_episode_sequence_) {
              return;
            }
            const auto request_it = tracker->cancel_request_sequences.find(uuid);
            if (request_it == tracker->cancel_request_sequences.end() ||
                request_it->second != request_sequence)
            {
              return;
            }
            tracker->cancel_requests_in_flight.erase(uuid);
            try {
              const auto response = future.get();
              RCLCPP_INFO(
                get_logger(),
                "Navigation child cancel response: action=%s uuid=%s return_code=%d",
                tracker->action_name.c_str(), goal_uuid_string(uuid).c_str(),
                static_cast<int>(response->return_code));
            } catch (const std::exception & ex) {
              RCLCPP_ERROR(
                get_logger(), "Navigation child cancel failed: action=%s error=%s",
                tracker->action_name.c_str(), ex.what());
            }
          });
      }
    }
  }

  bool navigation_children_drained() const
  {
    if (!navigation_parent_terminal_) {
      return false;
    }
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t discovery_ns =
      static_cast<int64_t>(nav_takeover_child_discovery_s_ * 1e9);
    if (now_ns - navigation_parent_terminal_ns_ < discovery_ns) {
      return false;
    }
    for (const auto & tracker : navigation_action_trackers_) {
      if (!tracker->active_goal_ids_in_latest_status.empty()) {
        return false;
      }
      for (const auto & uuid : tracker->episode_goal_ids) {
        const auto status_it = tracker->goal_statuses.find(uuid);
        const bool explicit_terminal =
          status_it != tracker->goal_statuses.end() &&
          navigation_goal_status_terminal(status_it->second);
        if (explicit_terminal) {
          continue;
        }

        int64_t required_status_after_ns = navigation_parent_terminal_ns_;
        const auto cancel_it = tracker->last_cancel_request_ns.find(uuid);
        if (cancel_it != tracker->last_cancel_request_ns.end()) {
          required_status_after_ns = std::max(
            required_status_after_ns, cancel_it->second);
        }
        const auto update_it = tracker->last_goal_status_update_ns.find(uuid);
        if (update_it == tracker->last_goal_status_update_ns.end() ||
            update_it->second <= required_status_after_ns)
        {
          return false;
        }
        const bool confirmed_absent_in_fresh_full_snapshot =
          status_it != tracker->goal_statuses.end() &&
          status_it->second == action_msgs::msg::GoalStatus::STATUS_UNKNOWN &&
          tracker->active_goal_ids_in_latest_status.count(uuid) == 0U;
        if (!confirmed_absent_in_fresh_full_snapshot) {
          return false;
        }
      }
    }
    return !spin_active_;
  }

  std::string active_navigation_children_summary() const
  {
    std::ostringstream stream;
    bool first = true;
    for (const auto & tracker : navigation_action_trackers_) {
      for (const auto & uuid : tracker->active_goal_ids_in_latest_status) {
        const auto status_it = tracker->goal_statuses.find(uuid);
        if (!first) {
          stream << ',';
        }
        first = false;
        stream << tracker->action_name << ':';
        if (status_it != tracker->goal_statuses.end()) {
          stream << static_cast<int>(status_it->second);
        } else {
          stream << "unowned";
        }
        stream << ':' << goal_uuid_string(uuid).substr(0, 8);
      }
    }
    if (first) {
      return "none";
    }
    return stream.str();
  }

  bool visual_takeover_cancel_pending() const
  {
    return
      visual_takeover_phase_ == VisualTakeoverPhase::CANCEL_PENDING_CLEAR ||
      visual_takeover_phase_ == VisualTakeoverPhase::CANCEL_PENDING_REACQUIRE;
  }

  bool visual_takeover_drain_active() const
  {
    return
      visual_takeover_phase_ == VisualTakeoverPhase::NAV_DRAINING_CLEAR ||
      visual_takeover_phase_ == VisualTakeoverPhase::NAV_DRAINING_REACQUIRE;
  }

  static bool visual_takeover_phase_is_reacquire(VisualTakeoverPhase phase)
  {
    return
      phase == VisualTakeoverPhase::CANCEL_PENDING_REACQUIRE ||
      phase == VisualTakeoverPhase::NAV_DRAINING_REACQUIRE;
  }

  void reset_parent_cancel_tracking()
  {
    ++parent_cancel_request_sequence_;
    parent_cancel_in_flight_ = false;
    parent_cancel_acknowledged_ = false;
    parent_cancel_last_request_ns_ = 0;
    active_parent_goal_sequence_ = 0;
    cancel_sent_ = false;
  }

  void request_current_navigation_goal_cancel(const std::string & reason)
  {
    if (!goal_handle_) {
      if (goal_active_ || has_sent_goal_) {
        publish_state("wait_goal_handle_to_cancel_" + reason);
      }
      return;
    }
    if (parent_cancel_acknowledged_) {
      return;
    }

    const int64_t now_ns = this->now().nanoseconds();
    const int64_t retry_ns = static_cast<int64_t>(nav_takeover_cancel_retry_s_ * 1e9);
    if (parent_cancel_last_request_ns_ != 0 &&
        now_ns - parent_cancel_last_request_ns_ < retry_ns)
    {
      return;
    }
    if (parent_cancel_in_flight_) {
      RCLCPP_WARN(
        get_logger(),
        "NavigateToPose cancel response timed out; retry exact current goal");
      parent_cancel_in_flight_ = false;
    }

    const auto goal_handle = goal_handle_;
    const auto goal_uuid = goal_handle->get_goal_id();
    const uint64_t goal_sequence = active_parent_goal_sequence_;
    const uint64_t request_sequence = ++parent_cancel_request_sequence_;
    parent_cancel_in_flight_ = true;
    parent_cancel_last_request_ns_ = now_ns;
    cancel_sent_ = true;
    try {
      nav_client_->async_cancel_goal(
        goal_handle,
        [this, goal_sequence, request_sequence, goal_uuid](auto response) {
          if (goal_sequence == 0 || goal_sequence != active_parent_goal_sequence_ ||
              request_sequence != parent_cancel_request_sequence_ ||
              !goal_handle_ || goal_handle_->get_goal_id() != goal_uuid)
          {
            return;
          }
          parent_cancel_in_flight_ = false;
          if (response && response->return_code == 0) {
            parent_cancel_acknowledged_ = true;
            RCLCPP_INFO(
              get_logger(),
              "NavigateToPose exact cancel accepted: uuid=%s; wait for terminal result",
              goal_uuid_string(goal_uuid).c_str());
          } else {
            parent_cancel_acknowledged_ = false;
            cancel_sent_ = false;
            RCLCPP_WARN(
              get_logger(),
              "NavigateToPose exact cancel rejected: uuid=%s return_code=%d; retry",
              goal_uuid_string(goal_uuid).c_str(),
              response ? static_cast<int>(response->return_code) : -1);
          }
        });
      publish_state("cancel_" + reason);
    } catch (const std::exception & ex) {
      parent_cancel_in_flight_ = false;
      parent_cancel_acknowledged_ = false;
      cancel_sent_ = false;
      RCLCPP_ERROR(
        get_logger(), "NavigateToPose exact cancel failed: %s", ex.what());
    }
  }

  void request_visual_takeover_cancel(const std::string & reason)
  {
    request_current_navigation_goal_cancel(reason);
  }

  void complete_visual_takeover_transition(
    VisualTakeoverPhase requested_phase,
    const std::string & nav_outcome)
  {
    if (!takeover_phase_is_pending(requested_phase)) {
      return;
    }

    if (!navigation_episode_tracking_) {
      start_navigation_episode_tracking();
      if (last_goal_send_time_.nanoseconds() != 0) {
        navigation_episode_start_ns_ = last_goal_send_time_.nanoseconds();
      }
    }
    navigation_parent_terminal_ = true;
    navigation_parent_terminal_ns_ = this->now().nanoseconds();
    navigation_drain_start_ns_ = navigation_parent_terminal_ns_;
    navigation_children_idle_since_ns_ = 0;
    navigation_parent_outcome_ = nav_outcome;
    visual_takeover_phase_ =
      requested_phase == VisualTakeoverPhase::CANCEL_PENDING_REACQUIRE ?
      VisualTakeoverPhase::NAV_DRAINING_REACQUIRE :
      VisualTakeoverPhase::NAV_DRAINING_CLEAR;
    cancel_tracked_navigation_children();
    reset_visual_controller();
    RCLCPP_INFO(
      get_logger(),
      "NavigateToPose %s; enter navigation drain before visual takeover",
      nav_outcome.c_str());
    publish_state("visual_takeover_navigation_draining");
  }

  void finish_visual_takeover_transition(VisualTakeoverPhase drain_phase)
  {
    if (drain_phase != VisualTakeoverPhase::NAV_DRAINING_CLEAR &&
        drain_phase != VisualTakeoverPhase::NAV_DRAINING_REACQUIRE)
    {
      return;
    }

    nav_retry_waiting_ = false;
    reset_avoidance_episode(true);
    visual_takeover_phase_ = VisualTakeoverPhase::SAFETY_GATE;
    reset_navigation_episode_tracking();
    avoidance_rearm_required_ = true;
    awaiting_visual_after_avoidance_ = true;
    clear_exact_visual_observation();
    reset_visual_controller();

    RCLCPP_INFO(
      get_logger(),
      "Navigation actions drained for exact YOLO+skeleton visual takeover; "
      "hold zero until a fresh exact observation and two safe scans");
    publish_state("visual_takeover_safety_gate");
  }

  static bool takeover_phase_is_pending(VisualTakeoverPhase phase)
  {
    return
      phase == VisualTakeoverPhase::CANCEL_PENDING_CLEAR ||
      phase == VisualTakeoverPhase::CANCEL_PENDING_REACQUIRE;
  }

  bool visual_clear_takeover_confirmed(
    const rclcpp::Time & now,
    const ObstacleScan & obstacles,
    bool avoidance_active,
    bool nav_engaged)
  {
    const double avoidance_elapsed_s =
      avoidance_start_time_.nanoseconds() == 0 ? 0.0 :
      std::max(0.0, (now - avoidance_start_time_).seconds());
    const bool candidate =
      visual_takeover_progress_clearance_ready(
      now, obstacles, avoidance_active, nav_engaged);

    if (!candidate) {
      reset_visual_clear_takeover_confirmation();
      return false;
    }
    if (visual_clear_takeover_last_scan_generation_ == scan_generation_) {
      return false;
    }

    visual_clear_takeover_last_scan_generation_ = scan_generation_;
    ++visual_clear_takeover_count_;
    if (visual_clear_takeover_count_ == 1) {
      RCLCPP_INFO(
        get_logger(),
        "Physical-clear takeover candidate: front=%.2f left=%.2f right=%.2f "
        "avoidance_elapsed=%.2f required_scans=%d",
        obstacles.front.distance, obstacles.left.distance, obstacles.right.distance,
        avoidance_elapsed_s, visual_clear_takeover_confirm_frames_);
    }

    if (visual_clear_takeover_count_ < visual_clear_takeover_confirm_frames_) {
      publish_state("physical_clear_takeover_confirming");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Physical-clear takeover confirmed: front=%.2f left=%.2f right=%.2f scans=%d",
      obstacles.front.distance, obstacles.left.distance, obstacles.right.distance,
      visual_clear_takeover_count_);
    return true;
  }

  void control_loop()
  {
    const auto now = this->now();

    if (g_shutdown_requested != 0) {
      g_shutdown_requested = 0;
      request_shutdown_stop("shutdown_stop");
      rclcpp::shutdown();
      return;
    }

    if (respect_mode_topic_ && mode_ != mode_required_) {
        publish_paused_calibration(now);
        cancel_goal("mode_paused");
        cancel_spin("mode_paused");
        reset_avoidance_episode();
        avoidance_rearm_required_ = false;
        reset_visual_controller();
        if (!paused_stop_published_) {
          paused_stop_published_ = publish_stop_cmd("paused_by_mode");
        }
        return;
    }

    if (!motion_enabled_) {
      // Keep consuming body/scan callbacks for the voice coordinator, while
      // guaranteeing that this node sends neither visual cmd_vel nor Nav2
      // goals until the coordinator explicitly releases it.
      cancel_goal("motion_gate_observe_only");
      cancel_spin("motion_gate_observe_only");
      reset_avoidance_episode();
      avoidance_rearm_required_ = false;
      reset_visual_controller();
      if (!motion_gate_stop_published_) {
        motion_gate_stop_published_ = publish_stop_cmd("motion_gate_observe_only");
      }
      publish_state("motion_gate_observe_only");
      return;
    }
    motion_gate_stop_published_ = false;

    if (has_body_ && invalid_body_since_.nanoseconds() != 0 &&
        (now - invalid_body_since_).seconds() >= body_invalid_grace_s_)
    {
      if (avoidance_episode_active()) {
        avoidance_visual_loss_seen_ = true;
      }
      has_body_ = false;
      active_visual_body_ = bodyreader_msg::msg::Bodyposture();
      active_visual_body_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      visual_reverse_takeover_count_ = 0;
      visual_reverse_takeover_last_body_generation_ = body_generation_;
      visual_reverse_takeover_pending_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Exact YOLO+skeleton observation grace expired after %.3fs; stop using cached geometry",
        (now - invalid_body_since_).seconds());
    }

    geometry_msgs::msg::PoseStamped goal;
    NavGoalKind goal_kind = NavGoalKind::NORMAL;
    const bool visual_recent =
      has_body_ && (now - last_body_time_).seconds() <= lost_timeout_s_;
    if (visual_recent) {
      active_visual_body_ = latest_body_;
      active_visual_body_time_ = last_body_time_;
      active_visual_body_is_dual_ = false;
    }
    const bool has_memory = has_last_person_map_;
    const double memory_age = has_memory ? (now - last_person_map_time_).seconds() : 0.0;
    const bool avoidance_active = avoidance_episode_active();
    const bool nav_engaged = goal_active_ || has_sent_goal_;
    if (!visual_recent) {
      visual_reverse_takeover_count_ = 0;
      visual_reverse_takeover_last_body_generation_ = body_generation_;
    }

    if (visual_takeover_cancel_pending()) {
      const auto requested_phase = visual_takeover_phase_;
      cancel_spin("visual_takeover_pending");
      request_visual_takeover_cancel("visual_takeover_during_avoidance");
      cancel_tracked_navigation_children();
      if (!goal_active_ && !has_sent_goal_) {
        complete_visual_takeover_transition(requested_phase, "not_active");
      }
      reset_visual_controller();
      publish_safety_stop_cmd("cancel_nav2_visual_takeover_pending");
      return;
    }

    if (visual_takeover_drain_active()) {
      const auto drain_phase = visual_takeover_phase_;
      cancel_spin("visual_takeover_navigation_draining");
      cancel_tracked_navigation_children();

      const bool children_drained = navigation_children_drained();
      const bool cmd_quiet =
        (now - last_cmd_motion_time_).seconds() >= nav_takeover_cmd_quiet_s_;
      if (!children_drained || !cmd_quiet) {
        navigation_children_idle_since_ns_ = 0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Navigation drain holds visual takeover: children=%s active=%s "
          "cmd_quiet=%s age=%.3f/%.3f",
          children_drained ? "drained" : "pending",
          active_navigation_children_summary().c_str(),
          cmd_quiet ? "true" : "false",
          (now - last_cmd_motion_time_).seconds(), nav_takeover_cmd_quiet_s_);
        reset_visual_controller();
        publish_safety_stop_cmd("visual_takeover_navigation_draining_stop");
        return;
      }

      if (navigation_children_idle_since_ns_ == 0) {
        navigation_children_idle_since_ns_ = now.nanoseconds();
      }
      const double idle_s =
        static_cast<double>(now.nanoseconds() - navigation_children_idle_since_ns_) * 1e-9;
      if (idle_s < nav_takeover_status_quiet_s_) {
        reset_visual_controller();
        publish_safety_stop_cmd("visual_takeover_navigation_quiet_confirming");
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "Navigation drain complete: parent=%s child_idle=%.3fs cmd_idle=%.3fs",
        navigation_parent_outcome_.c_str(), idle_s,
        (now - last_cmd_motion_time_).seconds());
      finish_visual_takeover_transition(drain_phase);
      publish_safety_stop_cmd("visual_takeover_navigation_drained_stop");
      return;
    }

    if (visual_takeover_phase_ == VisualTakeoverPhase::NONE && !nav_engaged &&
        now.nanoseconds() < exact_observation_accept_after_ns_)
    {
      cancel_spin("exact_stream_handoff_flush");
      reset_visual_controller();
      publish_safety_stop_cmd("exact_stream_handoff_flush_stop");
      return;
    }

#if 0
    if (position_jump_hold_active_ || position_jump_recovery_waiting_) {
      reset_visual_controller();
      if (nav_engaged) {
        // An unconfirmed target-depth jump invalidates visual evidence, not
        // the already-running obstacle-avoidance task. Do not publish a
        // competing zero cmd_vel and do not cancel any Nav2 action here.
        publish_state(position_jump_hold_active_ ?
          "target_position_jump_pending_nav2_continue" :
          "target_position_jump_wait_fresh_nav2_continue");
        return;
      }
      if (spin_active_) {
        cancel_spin("target_position_jump_direct_hold");
      }
      if (avoidance_active && !nav_engaged && !spin_active_ &&
          visual_takeover_phase_ == VisualTakeoverPhase::NONE)
      {
        reset_avoidance_episode();
        avoidance_rearm_required_ = false;
      }
      publish_safety_stop_cmd(position_jump_hold_active_ ?
        "target_position_jump_direct_hold_stop" :
        "target_position_jump_wait_fresh_stop");
      return;
    }
#endif

    if (visual_takeover_phase_ == VisualTakeoverPhase::SAFETY_GATE) {
      cancel_spin("visual_takeover_safety_gate");
      if (spin_active_) {
        reset_visual_safety_release_confirmation();
        reset_visual_controller();
        publish_safety_stop_cmd("visual_takeover_spin_cancel_pending");
        return;
      }
      const bool strict_visual_recent =
        visual_recent &&
        (now - last_body_time_).seconds() <= visual_clear_takeover_body_max_age_s_;
      if (!strict_visual_recent) {
        reset_visual_safety_release_confirmation();
        reset_visual_controller();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Visual safety gate holds: fresh exact body is stale or absent");
        publish_safety_stop_cmd("visual_takeover_body_stale_hold");
        return;
      }
      if (!scan_recent(now)) {
        reset_visual_safety_release_confirmation();
        reset_visual_controller();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Visual safety gate holds: laser scan is stale");
        publish_safety_stop_cmd("visual_takeover_scan_stale_hold");
        return;
      }

      const ObstacleScan gate_obstacles = scan_obstacles(false);
      if (!scan_sectors_valid(gate_obstacles)) {
        reset_visual_safety_release_confirmation();
        reset_visual_controller();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Visual safety gate holds: invalid scan sectors front=%zu left=%zu right=%zu",
          gate_obstacles.front.finite_ray_count,
          gate_obstacles.left.finite_ray_count,
          gate_obstacles.right.finite_ray_count);
        publish_safety_stop_cmd("visual_takeover_scan_invalid_hold");
        return;
      }
      const bool front_blocked =
        std::isfinite(gate_obstacles.front.distance) &&
        gate_obstacles.front.distance <= obstacle_nav_distance_m_;
      const bool left_not_released =
        std::isfinite(gate_obstacles.left.distance) &&
        gate_obstacles.left.distance < side_obstacle_release_distance_m_;
      const bool right_not_released =
        std::isfinite(gate_obstacles.right.distance) &&
        gate_obstacles.right.distance < side_obstacle_release_distance_m_;
      const bool visual_too_close =
        active_visual_body_.centerofmass_z * 0.001 <
        follow_distance_m_ - hold_distance_band_m_;

      if (front_blocked || left_not_released || right_not_released || visual_too_close) {
        reset_visual_safety_release_confirmation();
        reset_visual_controller();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Visual safety gate holds zero velocity: front=%.2f left=%.2f right=%.2f "
          "too_close=%s",
          gate_obstacles.front.distance, gate_obstacles.left.distance,
          gate_obstacles.right.distance, visual_too_close ? "true" : "false");
        publish_safety_stop_cmd("visual_takeover_obstacle_hold");
        return;
      }

      if (!obstacles_released(gate_obstacles)) {
        reset_visual_safety_release_confirmation();
        reset_visual_controller();
        publish_safety_stop_cmd("visual_takeover_obstacle_release_hold");
        return;
      }

      if (!visual_safety_release_confirmed(gate_obstacles)) {
        reset_visual_controller();
        publish_safety_stop_cmd("visual_takeover_release_confirming_stop");
        return;
      }

      visual_takeover_phase_ = VisualTakeoverPhase::NONE;
      avoidance_rearm_required_ = false;
      reset_visual_safety_release_confirmation();
      RCLCPP_INFO(
        get_logger(),
        "Visual safety gate released after fresh scan confirmation: "
        "front=%.2f left=%.2f right=%.2f",
        gate_obstacles.front.distance, gate_obstacles.left.distance,
        gate_obstacles.right.distance);
      publish_state("visual_takeover_safety_released");
    }

#if 0
    const bool dual_gate_scan_fresh = scan_recent(now);
    const ObstacleScan dual_gate_obstacles =
      dual_gate_scan_fresh ? scan_obstacles(visual_recent) : ObstacleScan();
    const bool dual_sample_fresh =
      last_dual_presence_body_time_.nanoseconds() != 0 &&
      (now - last_dual_presence_body_time_).seconds() <=
      dual_handoff_freshness_s();
    const bool dual_visual_too_close =
      dual_sample_fresh &&
      latest_dual_presence_body_.centerofmass_z * 0.001 <
      follow_distance_m_ - hold_distance_band_m_;
    const bool dual_takeover_gate_ready =
      dual_gate_scan_fresh &&
      scan_sectors_valid(dual_gate_obstacles) &&
      !dual_visual_too_close &&
      visual_takeover_progress_clearance_ready(
      now, dual_gate_obstacles, avoidance_active, nav_engaged);

    if (avoidance_active && nav_engaged &&
        visual_takeover_phase_ == VisualTakeoverPhase::NONE)
    {
      if (!dual_takeover_gate_ready) {
        const double scan_age = last_scan_time_.nanoseconds() == 0 ? -1.0 :
          (now - last_scan_time_).seconds();
        const double avoidance_elapsed = avoidance_start_time_.nanoseconds() == 0 ? -1.0 :
          (now - avoidance_start_time_).seconds();
        const double dual_age = last_dual_presence_body_time_.nanoseconds() == 0 ? -1.0 :
          (now - last_dual_presence_body_time_).seconds();
        const bool sectors_valid = scan_sectors_valid(dual_gate_obstacles);
        const bool elapsed_ready = avoidance_elapsed >= visual_clear_takeover_min_avoidance_s_;
        const bool front_clear =
          !std::isfinite(dual_gate_obstacles.front.distance) ||
          dual_gate_obstacles.front.distance >= visual_clear_takeover_front_clearance_m_;
        const bool left_clear =
          !std::isfinite(dual_gate_obstacles.left.distance) ||
          dual_gate_obstacles.left.distance >= visual_clear_takeover_side_clearance_m_;
        const bool right_clear =
          !std::isfinite(dual_gate_obstacles.right.distance) ||
          dual_gate_obstacles.right.distance >= visual_clear_takeover_side_clearance_m_;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Dual takeover gate closed: scan_fresh=%s age=%.3f/%.3f "
          "sectors_valid=%s rays=%zu/%zu/%zu elapsed_ready=%s elapsed=%.2f/%.2f "
          "too_close=%s "
          "clear=%s/%s/%s dist=%.2f/%.2f/%.2f threshold=%.2f/%.2f "
          "dual_fresh=%s age=%.3f count=%d/%d goal_active=%s handle=%s cancel_sent=%s",
          dual_gate_scan_fresh ? "true" : "false", scan_age, scan_timeout_s_,
          sectors_valid ? "true" : "false",
          dual_gate_obstacles.front.finite_ray_count,
          dual_gate_obstacles.left.finite_ray_count,
          dual_gate_obstacles.right.finite_ray_count,
          elapsed_ready ? "true" : "false",
          avoidance_elapsed, visual_clear_takeover_min_avoidance_s_,
          dual_visual_too_close ? "true" : "false",
          front_clear ? "true" : "false",
          left_clear ? "true" : "false",
          right_clear ? "true" : "false",
          dual_gate_obstacles.front.distance,
          dual_gate_obstacles.left.distance,
          dual_gate_obstacles.right.distance,
          visual_clear_takeover_front_clearance_m_,
          visual_clear_takeover_side_clearance_m_,
          dual_sample_fresh ? "true" : "false", dual_age,
          visual_reacquire_takeover_count_,
          visual_reacquire_takeover_confirm_frames_,
          goal_active_ ? "true" : "false",
          goal_handle_ ? "true" : "false",
          cancel_sent_ ? "true" : "false");
        dual_takeover_gate_open_ = false;
        reset_visual_reacquire_takeover_confirmation();
      } else if (!dual_takeover_gate_open_) {
        dual_takeover_gate_open_ = true;
        reset_visual_reacquire_takeover_confirmation();
        RCLCPP_INFO(
          get_logger(),
          "Avoidance progress/clearance satisfied: front=%.2f left=%.2f right=%.2f; "
          "wait for %d fresh skeleton+YOLO frames",
          dual_gate_obstacles.front.distance,
          dual_gate_obstacles.left.distance,
          dual_gate_obstacles.right.distance,
          visual_reacquire_takeover_confirm_frames_);
      }
    } else if (!avoidance_active || !nav_engaged) {
      dual_takeover_gate_open_ = false;
    }

    if (visual_takeover_phase_ == VisualTakeoverPhase::NONE &&
        dual_takeover_gate_open_ &&
        visual_reacquire_takeover_confirmed(now, avoidance_active, nav_engaged))
    {
      dual_handoff_retired_by_strict_ = false;
      visual_takeover_phase_ = VisualTakeoverPhase::CANCEL_PENDING_REACQUIRE;
      awaiting_visual_after_avoidance_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Post-clearance skeleton+YOLO confirmed after %d frames; cancel current Nav2 goal",
        visual_reacquire_takeover_count_);
      request_visual_takeover_cancel("post_clearance_dual_presence");
      reset_visual_controller();
      publish_safety_stop_cmd("cancel_nav2_post_clearance_dual_presence");
      return;
    }
#endif

    if (avoidance_active && has_sent_goal_ && !avoidance_motion_logged_ &&
        avoidance_start_time_.nanoseconds() != 0 &&
        (now - avoidance_start_time_).seconds() >= 1.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Avoidance handoff waiting for nonzero cmd_vel: trigger_elapsed=%.2fs "
        "goal_accepted=%s",
        (now - avoidance_start_time_).seconds(),
        avoidance_goal_accept_time_.nanoseconds() == 0 ? "false" : "true");
    }

    if (visual_reverse_takeover_pending_) {
      if (visual_takeover_phase_ == VisualTakeoverPhase::NONE) {
        visual_takeover_phase_ = VisualTakeoverPhase::CANCEL_PENDING_CLEAR;
      }
      request_visual_takeover_cancel("visual_reverse_takeover_during_avoidance");
      reset_visual_controller();
      publish_safety_stop_cmd("cancel_nav2_visual_reverse_during_avoidance");
      return;
    }

    if (visual_takeover_phase_ == VisualTakeoverPhase::NONE &&
        avoidance_active && nav_engaged)
    {
      const ObstacleScan takeover_obstacles =
        scan_recent(now) ? scan_obstacles(false) : ObstacleScan();
      if (visual_clear_takeover_confirmed(
          now, takeover_obstacles, avoidance_active, nav_engaged))
      {
        visual_takeover_phase_ = VisualTakeoverPhase::CANCEL_PENDING_CLEAR;
        awaiting_visual_after_avoidance_ = true;
        request_visual_takeover_cancel("physical_clear_takeover");
        reset_visual_controller();
        publish_safety_stop_cmd("cancel_nav2_physical_clear_takeover");
        return;
      }
    } else if (!avoidance_active || !nav_engaged) {
      reset_visual_clear_takeover_confirmation();
    }

    if (visual_recent) {
      if (spin_active_ && !avoidance_active) {
        cancel_spin("target_reacquired");
        publish_state("target_reacquired_cancel_spin");
        return;
      }

      double person_x = 0.0;
      double person_y = 0.0;
      double person_z = 0.0;
      if (!person_in_base(person_x, person_y, person_z) ||
          !update_person_memory_from_base(person_x, person_y, person_z)) {
        publish_state("tf_failed");
        return;
      }

      if (awaiting_visual_after_avoidance_ && !avoidance_active) {
        awaiting_visual_after_avoidance_ = false;
        publish_state("visual_reacquired_after_avoidance");
      }

      if (require_scan_for_visual_ && !scan_recent(now)) {
        cancel_goal("scan_stale");
        cancel_spin("scan_stale");
        reset_visual_clear_takeover_confirmation();
        reset_visual_controller();
        publish_safety_stop_cmd("scan_stale_stop");
        return;
      }

      const ObstacleScan obstacles =
        scan_recent(now) ? scan_obstacles(true) : ObstacleScan();
      const double front_distance = obstacles.front.distance;
      const bool side_obstacle_in_trigger =
        (std::isfinite(obstacles.left.distance) &&
        obstacles.left.distance <= side_obstacle_nav_distance_m_) ||
        (std::isfinite(obstacles.right.distance) &&
        obstacles.right.distance <= side_obstacle_nav_distance_m_);
      const bool side_obstacle_in_slow_band =
        (std::isfinite(obstacles.left.distance) &&
        obstacles.left.distance > side_obstacle_nav_distance_m_ &&
        obstacles.left.distance < side_obstacle_release_distance_m_) ||
        (std::isfinite(obstacles.right.distance) &&
        obstacles.right.distance > side_obstacle_nav_distance_m_ &&
        obstacles.right.distance < side_obstacle_release_distance_m_);
      const bool front_clear_for_side_approach =
        !std::isfinite(front_distance) || front_distance >= obstacle_slow_distance_m_;
      const bool visual_too_close =
        active_visual_body_.centerofmass_z * 0.001 <
        follow_distance_m_ - hold_distance_band_m_;

      const bool reverse_takeover_candidate =
        avoidance_active && visual_allow_reverse_ && visual_too_close;
      if (!reverse_takeover_candidate) {
        visual_reverse_takeover_count_ = 0;
        visual_reverse_takeover_last_body_generation_ = body_generation_;
      } else if (visual_reverse_takeover_last_body_generation_ != body_generation_) {
        visual_reverse_takeover_last_body_generation_ = body_generation_;
        visual_reverse_takeover_count_ = std::min(
          visual_reverse_takeover_count_ + 1,
          BODY_NAV2_VISUAL_REVERSE_TAKEOVER_CONFIRM_TICKS);
      }
      if (visual_reverse_takeover_count_ >=
          BODY_NAV2_VISUAL_REVERSE_TAKEOVER_CONFIRM_TICKS)
      {
        visual_reverse_takeover_pending_ = true;
        visual_takeover_phase_ = VisualTakeoverPhase::CANCEL_PENDING_CLEAR;
        request_visual_takeover_cancel("visual_reverse_takeover_during_avoidance");
        reset_visual_controller();
        publish_safety_stop_cmd("cancel_nav2_visual_reverse_during_avoidance");
        return;
      }

      if (avoidance_active) {
        reset_visual_controller();
        if (spin_active_) {
          cancel_spin("avoidance_keep_goal");
        }

        if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Nav2 action server %s is not available", nav_action_name_.c_str());
          publish_safety_stop_cmd("nav2_unavailable_stop");
          return;
        }

        if (!make_avoidance_nav_goal(goal, active_avoidance_obstacle_, goal_kind)) {
          publish_safety_stop_cmd("nav2_avoid_locked_waiting_stop");
          return;
        }
      } else {
          if (visual_allow_reverse_ && visual_too_close) {
          if (nav_engaged) {
            visual_takeover_phase_ = VisualTakeoverPhase::CANCEL_PENDING_CLEAR;
            request_visual_takeover_cancel("visual_reverse_takeover");
            publish_safety_stop_cmd("cancel_nav2_visual_reverse");
            return;
          }

          reset_avoidance_episode();
          publish_visual_follow_cmd(1.0, "visual_reverse_follow");
            return;
          }

          if (avoidance_rearm_required_ && side_obstacle_in_trigger) {
            avoidance_rearm_required_ = false;
            publish_state("nav2_side_avoid_rearm_override");
          }

          if (avoidance_rearm_required_) {
            const bool obstacle_released = obstacles_released(obstacles);
            avoidance_trigger_count_ = 0;
            left_avoidance_trigger_count_ = 0;
            right_avoidance_trigger_count_ = 0;

            if (!obstacle_released) {
              const bool side_hold =
                (std::isfinite(obstacles.left.distance) &&
                obstacles.left.distance < side_obstacle_release_distance_m_) ||
                (std::isfinite(obstacles.right.distance) &&
                obstacles.right.distance < side_obstacle_release_distance_m_);
              const double visual_scale =
                side_hold ? 0.0 : visual_scale_from_obstacle(front_distance);
              publish_visual_follow_cmd(
                visual_scale, "visual_post_avoidance_holdoff",
                BODY_NAV2_SAFETY_DECEL_MPS2);
              return;
            }

            avoidance_rearm_required_ = false;
            publish_state("visual_avoidance_rearmed");
          }

          ObstacleObservation triggered_obstacle;
          if (!avoidance_trigger_confirmed(obstacles, triggered_obstacle)) {
          const bool front_confirming =
            std::isfinite(obstacles.front.distance) &&
            obstacles.front.distance <= obstacle_nav_distance_m_;
          const bool side_confirming =
            (std::isfinite(obstacles.left.distance) &&
            obstacles.left.distance <= side_obstacle_nav_distance_m_) ||
            (std::isfinite(obstacles.right.distance) &&
            obstacles.right.distance <= side_obstacle_nav_distance_m_);
          if (front_confirming || side_confirming) {
            reset_visual_controller();
            publish_safety_stop_cmd(side_confirming ?
              "nav2_side_avoid_confirming_stop" : "nav2_avoid_confirming_stop");
            return;
          }

          remember_last_clear_person_pose();

          if (nav_engaged) {
            visual_takeover_phase_ = VisualTakeoverPhase::CANCEL_PENDING_CLEAR;
            request_visual_takeover_cancel("visual_direct_takeover");
            publish_safety_stop_cmd("cancel_nav2_visual_direct");
            return;
          }

          reset_avoidance_episode();
          const double visual_scale = visual_scale_from_obstacles(obstacles);
          const std::string state =
            visual_scale < 0.99 ? "visual_weighted_follow" : "visual_direct_follow";
          publish_visual_follow_cmd(
            visual_scale, state,
            visual_scale < 0.99 ? BODY_NAV2_SAFETY_DECEL_MPS2 : -1.0,
            side_obstacle_in_slow_band && front_clear_for_side_approach ?
            BODY_NAV2_SIDE_APPROACH_MIN_MPS : 0.0);
          return;
        }

        reset_visual_controller();
        if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Nav2 action server %s is not available", nav_action_name_.c_str());
          publish_safety_stop_cmd("nav2_unavailable_stop");
          return;
        }

        if (!make_avoidance_nav_goal(goal, triggered_obstacle, goal_kind)) {
          publish_safety_stop_cmd("nav2_avoid_goal_not_ready_stop");
          return;
        }
      }
    } else if (awaiting_visual_after_avoidance_) {
      reset_visual_controller();
      cancel_goal("awaiting_visual_after_avoidance");

      if (now.nanoseconds() < exact_observation_accept_after_ns_) {
        cancel_spin("post_avoidance_exact_stream_flush");
        publish_safety_stop_cmd("post_avoidance_exact_stream_flush_stop");
        return;
      }

      if (!scan_recent(now)) {
        cancel_spin("post_avoidance_scan_stale");
        publish_safety_stop_cmd("post_avoidance_scan_stale_stop");
        return;
      }

      if (!has_memory || memory_age > reacquire_timeout_s_) {
        cancel_spin("post_avoidance_memory_expired");
        publish_stop_cmd(
          has_memory ? "post_avoidance_memory_expired" : "post_avoidance_no_memory");
        return;
      }

      if (spin_active_) {
        publish_state("post_avoidance_reacquire_active");
        return;
      }

      if (!reacquire_yaw_goal_) {
        publish_stop_cmd("post_avoidance_wait_exact_target");
        return;
      }

      start_reacquire_spin("post_avoidance_reacquire");
      if (!spin_active_) {
        publish_stop_cmd("post_avoidance_reacquire_wait");
      }
      return;
    } else if (avoidance_active) {
      reset_visual_controller();
      if (spin_active_) {
        cancel_spin("avoidance_keep_goal");
      }

      if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Nav2 action server %s is not available", nav_action_name_.c_str());
        publish_safety_stop_cmd("nav2_unavailable_stop");
        return;
      }

      if (!make_avoidance_nav_goal(goal, active_avoidance_obstacle_, goal_kind)) {
        publish_safety_stop_cmd("nav2_avoid_occluded_waiting_stop");
        return;
      }
    } else if (spin_active_) {
      reset_visual_controller();
      if (!has_memory || memory_age > reacquire_timeout_s_) {
        cancel_spin("body_lost");
        if (cancel_on_lost_) {
          cancel_goal("body_lost");
        }
        publish_state(has_memory ? "body_lost_memory_expired" : "body_lost");
        return;
      }

      publish_state("spin_reacquire_active");
      return;
    } else if (reacquire_yaw_goal_ && has_memory && memory_age <= reacquire_timeout_s_) {
      reset_visual_controller();
      start_reacquire_spin("reacquire_spin_to_last_target");
      return;
    } else {
      reset_visual_controller();
      if (cancel_on_lost_) {
        cancel_goal("body_lost");
      }
        cancel_spin("body_lost");
        reset_avoidance_episode();
        avoidance_rearm_required_ = false;
        publish_stop_cmd(has_memory ? "body_lost_memory_expired" : "body_lost");
        return;
    }

    const bool first_avoidance_goal =
      goal_kind == NavGoalKind::AVOID_GATE &&
      last_sent_goal_kind_ != NavGoalKind::AVOID_GATE;
    const bool avoidance_goal_pending =
      goal_kind == NavGoalKind::AVOID_GATE && !goal_active_;

    if (!first_avoidance_goal && nav_retry_waiting_ && nav2_retry_backoff_s_ > 0.0 &&
        (now - last_failed_goal_time_).seconds() < nav2_retry_backoff_s_) {
      if (avoidance_goal_pending) {
        publish_safety_stop_cmd("nav2_avoid_retry_backoff_stop");
      } else {
        publish_state("nav2_retry_backoff");
      }
      return;
    }

    if (!first_avoidance_goal &&
        (now - last_goal_send_time_).seconds() < goal_update_period_s_) {
      if (avoidance_goal_pending) {
        publish_safety_stop_cmd("nav2_avoid_goal_throttle_stop");
      }
      return;
    }

    if (has_sent_goal_ && !goal_changed(goal, goal_kind)) {
      if (avoidance_goal_pending) {
        publish_safety_stop_cmd("nav2_avoid_wait_accept_stop");
      }
      return;
    }

    send_goal(goal, goal_kind);
  }

  bool make_goal_from_visible_body(geometry_msgs::msg::PoseStamped & map_goal)
  {
    double person_x = 0.0;
    double person_y = 0.0;
    double person_z = 0.0;
    if (!person_in_base(person_x, person_y, person_z)) {
      publish_state("tf_failed");
      return false;
    }

    if (!update_person_memory_from_base(person_x, person_y, person_z)) {
      publish_state("tf_failed");
      return false;
    }

    return make_goal_from_person_map(map_goal, "follow_goal_ready",
                                     "retreat_goal_ready", "holding_distance", true);
  }

  bool make_goal_from_memory(geometry_msgs::msg::PoseStamped & map_goal)
  {
    return make_goal_from_person_map(map_goal, "occluded_keep_last_goal",
                                     "occluded_retreat_goal_ready",
                                     "occluded_holding_distance", false);
  }

  bool make_goal_from_person_map(
    geometry_msgs::msg::PoseStamped & map_goal,
    const std::string & follow_state,
    const std::string & retreat_state,
    const std::string & hold_state,
    bool allow_hold)
  {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!lookup_robot_pose(robot_pose)) {
      publish_state("tf_failed");
      return false;
    }

    const double person_x = last_person_map_pose_.pose.position.x - robot_pose.pose.position.x;
    const double person_y = last_person_map_pose_.pose.position.y - robot_pose.pose.position.y;
    const double person_dist = std::hypot(person_x, person_y);
    if (person_dist < 1e-3) {
      cancel_goal("invalid_body_distance");
      publish_state("invalid_body_distance");
      return false;
    }

    const double distance_error = person_dist - follow_distance_m_;
    const bool inside_hold_band = std::fabs(distance_error) <= hold_distance_band_m_;
    const bool too_close = distance_error < -hold_distance_band_m_;

    if (inside_hold_band) {
      if (!allow_hold && reacquire_yaw_goal_) {
        start_reacquire_spin("occluded_holding_spin_reacquire");
        return false;
      }
      cancel_goal("inside_follow_band");
      publish_state(hold_state);
      return false;
    }

    if (too_close && !enable_retreat_goal_) {
      cancel_goal("too_close_retreat_disabled");
      publish_state("too_close");
      return false;
    }

    const double unit_x = person_x / person_dist;
    const double unit_y = person_y / person_dist;
    double target_x = last_person_map_pose_.pose.position.x - unit_x * follow_distance_m_;
    double target_y = last_person_map_pose_.pose.position.y - unit_y * follow_distance_m_;
    double target_dx = target_x - robot_pose.pose.position.x;
    double target_dy = target_y - robot_pose.pose.position.y;
    double target_dist = std::hypot(target_dx, target_dy);

    if (too_close && max_retreat_goal_m_ > 0.0 && target_dist > max_retreat_goal_m_) {
      const double scale = max_retreat_goal_m_ / target_dist;
      target_dx *= scale;
      target_dy *= scale;
      target_x = robot_pose.pose.position.x + target_dx;
      target_y = robot_pose.pose.position.y + target_dy;
      target_dist = max_retreat_goal_m_;
    }

    if (target_dist < min_goal_distance_m_) {
      if (target_dist < 1e-6) {
        cancel_goal("goal_too_close");
        publish_state("goal_too_close");
        return false;
      }
      const double scale = min_goal_distance_m_ / target_dist;
      target_dx *= scale;
      target_dy *= scale;
      target_x = robot_pose.pose.position.x + target_dx;
      target_y = robot_pose.pose.position.y + target_dy;
      target_dist = min_goal_distance_m_;
    }

    const double yaw_to_person =
      std::atan2(last_person_map_pose_.pose.position.y - target_y,
                 last_person_map_pose_.pose.position.x - target_x);

    map_goal.header.stamp = this->now();
    map_goal.header.frame_id = global_frame_;
    map_goal.pose.position.x = target_x;
    map_goal.pose.position.y = target_y;
    map_goal.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_to_person);
    map_goal.pose.orientation = tf2::toMsg(q);

    goal_pub_->publish(map_goal);
    publish_state(too_close ? retreat_state : follow_state);
    return true;
  }

  bool should_force_spin_reacquire(const rclcpp::Time & now) const
  {
    if (!enable_cmd_watchdog_ || !reacquire_yaw_goal_ || !goal_active_ || !has_cmd_) {
      return false;
    }

    if (recent_avoidance_goal(now)) {
      return false;
    }

    return (now - last_cmd_motion_time_).seconds() >= cmd_zero_timeout_s_;
  }

  bool start_reacquire_spin(
    const std::string & state,
    bool force_post_avoidance = false)
  {
    if (!reacquire_yaw_goal_ && !force_post_avoidance) {
      return false;
    }

    const auto now = this->now();
    if (spin_retry_period_s_ > 0.0 &&
        (now - last_spin_send_time_).seconds() < spin_retry_period_s_) {
      publish_state("spin_retry_wait");
      return true;
    }

    if (!spin_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Nav2 spin action server %s is not available", spin_action_name_.c_str());
      publish_state("spin_unavailable");
      return false;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!lookup_robot_pose(robot_pose)) {
      publish_state("tf_failed");
      return false;
    }

    const double dx = last_person_map_pose_.pose.position.x - robot_pose.pose.position.x;
    const double dy = last_person_map_pose_.pose.position.y - robot_pose.pose.position.y;
    if (std::hypot(dx, dy) < 1e-3) {
      publish_state("reacquire_goal_too_close");
      return false;
    }

    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    double yaw_delta = normalize_angle(std::atan2(dy, dx) - robot_yaw);
    if (std::fabs(yaw_delta) < reacquire_spin_min_yaw_rad_) {
      yaw_delta = yaw_delta >= 0.0 ? reacquire_spin_min_yaw_rad_ : -reacquire_spin_min_yaw_rad_;
    }
    if (yaw_delta > reacquire_spin_max_yaw_rad_) {
      yaw_delta = reacquire_spin_max_yaw_rad_;
    } else if (yaw_delta < -reacquire_spin_max_yaw_rad_) {
      yaw_delta = -reacquire_spin_max_yaw_rad_;
    }

    cancel_goal("spin_reacquire");

    Spin::Goal spin_goal;
    spin_goal.target_yaw = static_cast<float>(yaw_delta);
    const double allowance_s = spin_time_allowance_s_ > 0.0 ? spin_time_allowance_s_ : 0.0;
    spin_goal.time_allowance.sec = static_cast<int32_t>(allowance_s);
    spin_goal.time_allowance.nanosec =
      static_cast<uint32_t>((allowance_s - spin_goal.time_allowance.sec) * 1e9);

    const auto spin_sequence = ++spin_sequence_;
    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.goal_response_callback =
      [this, spin_sequence](GoalHandleSpin::SharedPtr goal_handle) {
        if (spin_sequence != spin_sequence_) {
          if (goal_handle) {
            spin_client_->async_cancel_goal(goal_handle);
            RCLCPP_WARN(
              get_logger(),
              "Cancel accepted stale spin response: seq=%llu current=%llu",
              static_cast<unsigned long long>(spin_sequence),
              static_cast<unsigned long long>(spin_sequence_));
          }
          return;
        }

        if (!goal_handle) {
          spin_active_ = false;
          spin_handle_.reset();
          if (spin_cancel_requested_sequence_ == spin_sequence) {
            spin_cancel_requested_sequence_ = 0;
          }
          publish_state("spin_rejected");
          RCLCPP_WARN(get_logger(), "Nav2 rejected reacquire spin goal");
          return;
        }

        spin_active_ = true;
        spin_cancel_sent_ = false;
        spin_handle_ = goal_handle;
        publish_state("spin_accepted");
        if (spin_cancel_requested_sequence_ == spin_sequence ||
            visual_takeover_phase_ != VisualTakeoverPhase::NONE ||
            position_jump_hold_active_ || position_jump_recovery_waiting_)
        {
          cancel_spin("visual_or_position_hold_after_spin_accept");
        }
      };

    options.result_callback =
      [this, spin_sequence](const GoalHandleSpin::WrappedResult & result) {
        if (spin_sequence != spin_sequence_) {
          return;
        }

        spin_active_ = false;
        spin_cancel_sent_ = false;
        spin_handle_.reset();
        if (spin_cancel_requested_sequence_ == spin_sequence) {
          spin_cancel_requested_sequence_ = 0;
        }

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            publish_state("spin_succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            publish_state("spin_aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            publish_state("spin_canceled");
            break;
          default:
            publish_state("spin_unknown_result");
            break;
        }
      };

    spin_active_ = true;
    spin_cancel_sent_ = false;
    last_spin_send_time_ = now;
    spin_client_->async_send_goal(spin_goal, options);
    publish_state(state);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Sending reacquire spin %.2f rad toward remembered target", yaw_delta);
    return true;
  }

  bool lookup_robot_pose(geometry_msgs::msg::PoseStamped & robot_pose)
  {
    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    base_pose.header.frame_id = base_frame_;
    base_pose.pose.position.x = 0.0;
    base_pose.pose.position.y = 0.0;
    base_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    base_pose.pose.orientation = tf2::toMsg(q);

    try {
      robot_pose = tf_buffer_->transform(
        base_pose, global_frame_, tf2::durationFromSec(tf_timeout_s_));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform robot pose from %s to %s: %s",
        base_frame_.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool update_person_memory_from_base(double x, double y, double z)
  {
    geometry_msgs::msg::PointStamped base_person;
    base_person.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    base_person.header.frame_id = base_frame_;
    base_person.point.x = x;
    base_person.point.y = y;
    base_person.point.z = z;

    try {
      const auto map_person = tf_buffer_->transform(
        base_person, global_frame_, tf2::durationFromSec(tf_timeout_s_));
      last_person_map_pose_.header = map_person.header;
      last_person_map_pose_.pose.position.x = map_person.point.x;
      last_person_map_pose_.pose.position.y = map_person.point.y;
      last_person_map_pose_.pose.position.z = map_person.point.z;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      last_person_map_pose_.pose.orientation = tf2::toMsg(q);
      last_person_map_time_ = this->now();
      has_last_person_map_ = true;
      publish_calibration_sample(map_person, x, y, z);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform remembered body target from %s to %s: %s",
        base_frame_.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool person_in_base(double & x, double & y, double & z)
  {
    const double camera_forward =
      active_visual_body_.centerofmass_z * 0.001 * BODY_NAV2_PERSON_MAP_DEPTH_SCALE;
    const double camera_left =
      active_visual_body_.centerofmass_x * 0.001 * BODY_NAV2_PERSON_MAP_DEPTH_SCALE;
    const double camera_up = active_visual_body_.centerofmass_y * 0.001;

    if (!body_frame_id_.empty()) {
      geometry_msgs::msg::PointStamped body_point;
      body_point.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      body_point.header.frame_id = body_frame_id_;
      body_point.point.x = camera_forward;
      body_point.point.y = camera_left;
      body_point.point.z = camera_up;

      try {
        const auto base_point = tf_buffer_->transform(
          body_point, base_frame_, tf2::durationFromSec(tf_timeout_s_));
        x = base_point.point.x;
        y = base_point.point.y;
        z = base_point.point.z;
        return true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to transform body point from %s to %s: %s",
          body_frame_id_.c_str(), base_frame_.c_str(), ex.what());
        return false;
      }
    }

    const double c = std::cos(camera_yaw_offset_rad_);
    const double s = std::sin(camera_yaw_offset_rad_);
    x = camera_offset_x_m_ + BODY_NAV2_LIDAR_FORWARD_OF_BASE_M -
      BODY_NAV2_DEPTH_CAMERA_BEHIND_LIDAR_M +
      c * camera_forward - s * camera_left;
    y = camera_offset_y_m_ + s * camera_forward + c * camera_left;
    z = camera_offset_z_m_ + camera_up;
    return true;
  }

  bool goal_changed(
    const geometry_msgs::msg::PoseStamped & goal,
    NavGoalKind goal_kind) const
  {
    if (goal_kind != last_sent_goal_kind_) {
      return true;
    }

    const double dx = goal.pose.position.x - last_sent_goal_.pose.position.x;
    const double dy = goal.pose.position.y - last_sent_goal_.pose.position.y;
    const double dist = std::hypot(dx, dy);
    const double yaw = tf2::getYaw(goal.pose.orientation);
    const double last_yaw = tf2::getYaw(last_sent_goal_.pose.orientation);
    const double yaw_delta = std::fabs(normalize_angle(yaw - last_yaw));
    return dist >= goal_position_tolerance_m_ || yaw_delta >= goal_yaw_tolerance_rad_;
  }

  void send_goal(
    const geometry_msgs::msg::PoseStamped & goal_pose,
    NavGoalKind goal_kind = NavGoalKind::NORMAL)
  {
    if (goal_kind == NavGoalKind::AVOID_GATE &&
        last_sent_goal_kind_ != NavGoalKind::AVOID_GATE)
    {
      start_navigation_episode_tracking();
    }
    NavigateToPose::Goal goal;
    goal.pose = goal_pose;
    goal.behavior_tree = behavior_tree_;

    const auto goal_sequence = ++goal_sequence_;
    const auto follow_session_epoch = follow_session_epoch_;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this, goal_sequence, goal_kind, follow_session_epoch](
        GoalHandleNavigate::SharedPtr goal_handle) {
        if (follow_session_epoch != follow_session_epoch_) {
          if (goal_handle) {
            nav_client_->async_cancel_goal(goal_handle);
          }
          if (goal_sequence == goal_sequence_) {
            goal_active_ = false;
            goal_handle_.reset();
            has_sent_goal_ = false;
            reset_parent_cancel_tracking();
          }
          return;
        }
        if (goal_sequence != goal_sequence_) {
          if (goal_handle) {
            nav_client_->async_cancel_goal(goal_handle);
            RCLCPP_WARN(
              get_logger(),
              "Cancel accepted stale Nav2 goal response: seq=%llu current=%llu",
              static_cast<unsigned long long>(goal_sequence),
              static_cast<unsigned long long>(goal_sequence_));
          }
          return;
        }

        if (!goal_handle) {
          const auto requested_phase = visual_takeover_phase_;
          goal_active_ = false;
          goal_handle_.reset();
          has_sent_goal_ = false;
          reset_parent_cancel_tracking();
          nav_retry_waiting_ = true;
          last_failed_goal_time_ = this->now();
          if (goal_kind == NavGoalKind::AVOID_GATE &&
              takeover_phase_is_pending(requested_phase))
          {
            complete_visual_takeover_transition(requested_phase, "rejected");
            publish_safety_stop_cmd("nav2_avoid_rejected_visual_takeover");
            return;
          }
          if (goal_kind == NavGoalKind::AVOID_GATE) {
            publish_safety_stop_cmd("nav2_avoid_goal_rejected_stop");
          } else {
            publish_state("goal_rejected");
          }
          RCLCPP_WARN(get_logger(), "Nav2 rejected body follow goal");
          return;
        }

        goal_active_ = true;
        reset_parent_cancel_tracking();
        active_parent_goal_sequence_ = goal_sequence;
        goal_handle_ = goal_handle;
        if (goal_kind == NavGoalKind::AVOID_GATE) {
          avoidance_goal_accept_time_ = this->now();
          const double send_to_accept_ms =
            std::max(
            0.0, (avoidance_goal_accept_time_ - last_goal_send_time_).seconds()) * 1000.0;
          const double trigger_to_accept_ms =
            avoidance_start_time_.nanoseconds() == 0 ? -1.0 :
            std::max(
            0.0, (avoidance_goal_accept_time_ - avoidance_start_time_).seconds()) * 1000.0;
          RCLCPP_INFO(
            get_logger(),
            "Avoidance goal accepted: trigger_to_accept=%.0fms send_to_accept=%.0fms",
            trigger_to_accept_ms, send_to_accept_ms);
        }
        publish_state("goal_accepted");
        if (goal_kind == NavGoalKind::AVOID_GATE && visual_takeover_cancel_pending()) {
          request_visual_takeover_cancel("visual_takeover_after_accept");
          publish_safety_stop_cmd("cancel_nav2_visual_takeover_after_accept");
        }
      };

    options.result_callback =
      [this, goal_sequence, goal_kind, follow_session_epoch](
        const GoalHandleNavigate::WrappedResult & result) {
        if (goal_sequence != goal_sequence_) {
          return;
        }

        if (follow_session_epoch != follow_session_epoch_) {
          goal_active_ = false;
          goal_handle_.reset();
          has_sent_goal_ = false;
          reset_parent_cancel_tracking();
          awaiting_visual_after_avoidance_ = false;
          avoidance_rearm_required_ = false;
          return;
        }

        const auto requested_phase = visual_takeover_phase_;
        goal_active_ = false;
        goal_handle_.reset();
        has_sent_goal_ = false;
        reset_parent_cancel_tracking();

        if (goal_kind == NavGoalKind::AVOID_GATE &&
            takeover_phase_is_pending(requested_phase))
        {
          const char * outcome = "unknown";
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              outcome = "completed";
              break;
            case rclcpp_action::ResultCode::ABORTED:
              outcome = "aborted";
              break;
            case rclcpp_action::ResultCode::CANCELED:
              outcome = "canceled";
              break;
            default:
              break;
          }
          nav_retry_waiting_ = false;
          complete_visual_takeover_transition(requested_phase, outcome);
          return;
        }

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            nav_retry_waiting_ = false;
            if (goal_kind == NavGoalKind::AVOID_GATE) {
              reset_avoidance_episode();
              avoidance_rearm_required_ = true;
              clear_exact_visual_observation();
              awaiting_visual_after_avoidance_ = true;
              publish_state("nav2_avoid_gate_succeeded_wait_visual");
            } else {
              publish_state("goal_succeeded");
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            nav_retry_waiting_ = true;
            last_failed_goal_time_ = this->now();
            if (goal_kind == NavGoalKind::AVOID_GATE) {
              publish_safety_stop_cmd("nav2_avoid_goal_aborted_stop");
            } else {
              publish_state("goal_aborted");
            }
            break;
          case rclcpp_action::ResultCode::CANCELED:
            nav_retry_waiting_ = false;
            if (goal_kind == NavGoalKind::AVOID_GATE) {
              if (visual_reverse_takeover_pending_) {
                reset_avoidance_episode();
                avoidance_rearm_required_ = true;
                publish_state("nav2_avoid_canceled_for_visual_reverse");
              } else {
                publish_safety_stop_cmd("nav2_avoid_goal_canceled_stop");
              }
            } else {
              publish_state("goal_canceled");
            }
            break;
          default:
            nav_retry_waiting_ = true;
            last_failed_goal_time_ = this->now();
            if (goal_kind == NavGoalKind::AVOID_GATE) {
              publish_safety_stop_cmd("nav2_avoid_goal_unknown_stop");
            } else {
              publish_state("goal_unknown_result");
            }
            break;
        }
      };

    nav_retry_waiting_ = false;
    goal_pub_->publish(goal_pose);
    nav_client_->async_send_goal(goal, options);
    last_sent_goal_ = goal_pose;
    last_sent_goal_kind_ = goal_kind;
    has_sent_goal_ = true;
    last_goal_send_time_ = this->now();
    if (goal_kind == NavGoalKind::AVOID_GATE) {
      avoidance_motion_logged_ = false;
      const double trigger_to_send_ms =
        avoidance_start_time_.nanoseconds() == 0 ? -1.0 :
        std::max(0.0, (last_goal_send_time_ - avoidance_start_time_).seconds()) * 1000.0;
      RCLCPP_INFO(
        get_logger(),
        "Avoidance goal sent: trigger_to_send=%.0fms target=(%.2f, %.2f)",
        trigger_to_send_ms, goal_pose.pose.position.x, goal_pose.pose.position.y);
    }
    publish_state("goal_sent");
  }

  void cancel_goal(const std::string & reason)
  {
    request_current_navigation_goal_cancel(reason);
  }

  void cancel_spin(const std::string & reason)
  {
    if (spin_active_) {
      spin_cancel_requested_sequence_ = spin_sequence_;
    }
    if (spin_cancel_sent_) {
      return;
    }

    if (spin_handle_) {
      spin_client_->async_cancel_goal(spin_handle_);
      spin_cancel_sent_ = true;
      publish_state("cancel_spin_" + reason);
    } else if (spin_active_) {
      publish_state("wait_spin_handle_to_cancel_" + reason);
    }
  }

  void publish_state(const std::string & state)
  {
    if (state == last_state_) {
      return;
    }
    last_state_ = state;

    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
  }

  std::string nav_action_name_;
  std::string spin_action_name_;
  std::string scan_topic_;
  std::string visual_cmd_topic_;
  std::string motion_gate_service_name_;
  std::string global_frame_;
  std::string base_frame_;
  std::string body_frame_id_;
  std::string dual_presence_body_topic_;
  std::string position_jump_event_topic_;
  std::string behavior_tree_;
  std::string last_state_;

  double follow_distance_m_;
  double hold_distance_band_m_;
  double min_goal_distance_m_;
  double goal_update_period_s_;
  double goal_position_tolerance_m_;
  double goal_yaw_tolerance_rad_;
  double lost_timeout_s_;
  double body_invalid_grace_s_;
  double tf_timeout_s_;
  double max_retreat_goal_m_;
  double target_memory_timeout_s_;
  double reacquire_timeout_s_;
  double nav2_retry_backoff_s_;
  double reacquire_spin_min_yaw_rad_;
  double reacquire_spin_max_yaw_rad_;
  double spin_time_allowance_s_;
  double spin_retry_period_s_;
  double cmd_zero_timeout_s_;
  double cmd_zero_linear_epsilon_;
  double cmd_zero_angular_epsilon_;
  double scan_timeout_s_;
  double obstacle_front_angle_rad_;
  double side_obstacle_min_angle_rad_;
  double side_obstacle_max_angle_rad_;
  double obstacle_slow_distance_m_;
  double obstacle_nav_distance_m_;
  double obstacle_nav_release_distance_m_;
  double side_obstacle_nav_distance_m_;
  double side_obstacle_release_distance_m_;
  double visual_min_linear_scale_;
  double target_exemption_angle_rad_;
  double target_exemption_distance_margin_m_;
  double avoidance_forward_margin_m_;
  double avoidance_min_forward_step_m_;
  double avoidance_max_forward_step_m_;
  double avoidance_occlusion_hold_s_;
  double visual_clear_takeover_front_clearance_m_;
  double visual_clear_takeover_side_clearance_m_;
  double visual_clear_takeover_min_avoidance_s_;
  double visual_clear_takeover_body_max_age_s_;
  double visual_reacquire_takeover_body_max_age_s_;
  double visual_reacquire_takeover_min_frame_interval_s_;
  double visual_reacquire_takeover_max_frame_interval_s_;
  double nav_takeover_child_capture_slop_s_;
  double nav_takeover_cancel_retry_s_;
  double nav_takeover_status_quiet_s_;
  double nav_takeover_cmd_quiet_s_;
  double nav_takeover_child_discovery_s_;
  double visual_x_p_;
  double visual_x_d_;
  double visual_z_p_;
  double visual_z_d_;
  double visual_filter_alpha_;
  double visual_angle_deadband_;
  double visual_distance_deadband_mm_;
  double visual_max_linear_mps_;
  double visual_max_angular_rps_;
  double visual_linear_accel_limit_;
  double visual_reverse_accel_limit_;
  double visual_linear_decel_limit_;
  double visual_angular_accel_limit_;
  double camera_offset_x_m_;
  double camera_offset_y_m_;
  double camera_offset_z_m_;
  double camera_yaw_offset_rad_;
  bool visual_allow_reverse_;
  bool require_scan_for_visual_;
  bool respect_mode_topic_;
  bool cancel_on_lost_;
  bool enable_retreat_goal_;
  bool enable_avoidance_side_goal_;
  bool keep_last_goal_when_occluded_;
  bool reacquire_yaw_goal_;
  bool enable_cmd_watchdog_;
  int shutdown_stop_publish_count_;
  int shutdown_stop_publish_period_ms_;
  int mode_required_;
  int mode_;
  int avoidance_trigger_count_;
  int left_avoidance_trigger_count_;
  int right_avoidance_trigger_count_;
  int visual_reverse_takeover_count_;
  int visual_clear_takeover_confirm_frames_;
  int visual_reacquire_takeover_confirm_frames_;
  int visual_safety_release_confirm_frames_;
  int visual_safety_min_finite_rays_per_sector_;
  int visual_clear_takeover_count_{0};
  int visual_reacquire_takeover_count_{0};
  int visual_safety_release_count_{0};
  int dual_handoff_strict_confirm_count_{0};
  AvoidanceStage avoidance_stage_;
  NavGoalKind last_sent_goal_kind_;
  VisualTakeoverPhase visual_takeover_phase_{VisualTakeoverPhase::NONE};

  bodyreader_msg::msg::Bodyposture latest_body_;
  bodyreader_msg::msg::Bodyposture latest_dual_presence_body_;
  bodyreader_msg::msg::Bodyposture active_visual_body_;
  bodyreader_msg::msg::Bodyposture position_jump_recovery_candidate_;
  std::map<int, bodyreader_msg::msg::BodyJumpEvent>
    pending_position_jump_events_;
  bool active_visual_body_is_dual_{false};
  bool has_body_;
  bool position_jump_hold_active_;
  bool position_jump_recovery_waiting_;
  bool position_jump_source_reset_recovery_{false};
  int strict_target_body_id_{0};
  int position_jump_origin_body_id_{0};
  int position_jump_expected_body_id_{0};
  int position_jump_native_body_id_{0};
  int position_jump_recovery_valid_count_{0};
  int position_jump_recovery_body_id_{0};
  double position_jump_trusted_depth_mm_{0.0};
  double position_jump_observed_depth_mm_{0.0};
  double position_jump_confirmed_depth_mm_{0.0};
  uint8_t position_jump_phase_{0};
  uint8_t position_jump_outcome_{0};
  bool has_last_person_map_;
  bool has_sent_goal_;
  bool goal_active_;
  bool cancel_sent_;
  bool spin_active_;
  bool spin_cancel_sent_;
  bool nav_retry_waiting_;
  bool scan_received_;
  bool has_cmd_;
  bool visual_filter_init_;
  bool visual_hold_active_;
  bool direct_cmd_init_;
  bool shutdown_stop_done_;
  bool paused_stop_published_;
  bool has_last_clear_person_pose_;
  bool has_side_trigger_target_;
  bool awaiting_visual_after_avoidance_;
  bool avoidance_rearm_required_;
  bool visual_reverse_takeover_pending_;
  bool avoidance_visual_loss_seen_{false};
  bool dual_takeover_gate_open_{false};
  bool dual_handoff_active_{false};
  bool dual_handoff_reacquire_armed_{false};
  bool dual_handoff_retired_by_strict_{false};
  bool has_avoidance_gate_goal_;
  bool avoidance_motion_logged_;
  bool has_calibration_reference_;
  bool motion_enabled_;
  bool motion_gate_stop_published_;
  uint64_t goal_sequence_;
  uint64_t follow_session_epoch_{0};
  uint64_t spin_sequence_;
  uint64_t position_jump_source_epoch_{0};
  uint64_t position_jump_event_id_{0};
  uint64_t position_jump_bound_goal_sequence_{0};
  uint32_t position_jump_event_update_seq_{0};
  uint64_t spin_cancel_requested_sequence_{0};
  uint64_t body_generation_{0};
  int64_t exact_observation_accept_after_ns_{0};
  uint64_t visual_reverse_takeover_last_body_generation_{0};
  uint64_t visual_clear_takeover_last_scan_generation_{0};
  uint64_t scan_generation_{0};
  uint64_t visual_safety_release_last_scan_generation_{0};
  uint64_t navigation_episode_sequence_{0};
  uint64_t active_parent_goal_sequence_{0};
  uint64_t parent_cancel_request_sequence_{0};
  int visual_reacquire_body_id_{0};
  bool parent_cancel_in_flight_{false};
  bool parent_cancel_acknowledged_{false};
  int64_t parent_cancel_last_request_ns_{0};
  bool navigation_episode_tracking_{false};
  bool navigation_parent_terminal_{false};
  int64_t navigation_episode_start_ns_{0};
  int64_t navigation_parent_terminal_ns_{0};
  int64_t navigation_drain_start_ns_{0};
  int64_t navigation_children_idle_since_ns_{0};
  std::string navigation_parent_outcome_;
  rclcpp::Time last_dual_presence_body_time_;
  rclcpp::Time active_visual_body_time_;
  rclcpp::Time position_jump_recovery_last_valid_time_;

  rclcpp::Time last_body_time_;
  rclcpp::Time invalid_body_since_;
  rclcpp::Time last_person_map_time_;
  rclcpp::Time last_clear_person_time_;
  rclcpp::Time last_goal_send_time_;
  rclcpp::Time last_failed_goal_time_;
  rclcpp::Time last_spin_send_time_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_cmd_motion_time_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time last_direct_cmd_time_;
  rclcpp::Time last_avoidance_goal_time_;
  rclcpp::Time avoidance_start_time_;
  rclcpp::Time avoidance_goal_accept_time_;
  double filtered_x_angle_;
  double filtered_distance_mm_;
  double last_visual_error_angle_;
  double last_visual_error_distance_;
  sensor_msgs::msg::LaserScan last_scan_;
  geometry_msgs::msg::Twist last_direct_cmd_;
  geometry_msgs::msg::Twist last_observed_cmd_;
  geometry_msgs::msg::PointStamped calibration_reference_map_;
  geometry_msgs::msg::PoseStamped last_person_map_pose_;
  geometry_msgs::msg::PoseStamped last_clear_person_pose_;
  geometry_msgs::msg::PoseStamped side_trigger_target_pose_;
  geometry_msgs::msg::PoseStamped avoidance_gate_goal_;
  geometry_msgs::msg::PoseStamped last_sent_goal_;
  ObstacleObservation active_avoidance_obstacle_;
  GoalHandleNavigate::SharedPtr goal_handle_;
  GoalHandleSpin::SharedPtr spin_handle_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  std::vector<std::shared_ptr<NavigationActionTracker>> navigation_action_trackers_;
  rclcpp::Subscription<bodyreader_msg::msg::Bodyposture>::SharedPtr body_sub_;
  rclcpp::Subscription<bodyreader_msg::msg::Bodyposture>::SharedPtr
    dual_presence_body_sub_;
  rclcpp::Subscription<bodyreader_msg::msg::BodyJumpEvent>::SharedPtr
    position_jump_event_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motion_gate_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    calibration_reference_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr visual_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calibration_sample_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
    calibration_person_base_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    calibration_person_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    calibration_robot_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
    calibration_reference_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
    calibration_obstacle_ray_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
    calibration_obstacle_assumed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    calibration_latched_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, handle_shutdown_signal);
  std::signal(SIGTERM, handle_shutdown_signal);
  auto node = std::make_shared<BodyNav2Follower>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    node->request_shutdown_stop("spin_return_stop");
    rclcpp::shutdown();
  }
  return 0;
}
