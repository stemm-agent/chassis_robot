#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "bodyreader_msg/msg/bodyposture.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
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
  };

  struct ObstacleScan
  {
    ObstacleObservation front;
    ObstacleObservation left;
    ObstacleObservation right;
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
    goal_sequence_(0),
    spin_sequence_(0),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    nav_action_name_ = declare_parameter<std::string>("nav_action_name", "/navigate_to_pose");
    spin_action_name_ = declare_parameter<std::string>("spin_action_name", "/spin");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    visual_cmd_topic_ = declare_parameter<std::string>("visual_cmd_topic", "/cmd_vel");
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
      declare_parameter<double>("obstacle_nav_release_distance_m", 1.35);
    side_obstacle_nav_distance_m_ =
      declare_parameter<double>("side_obstacle_nav_distance_m", 0.40);
    side_obstacle_release_distance_m_ =
      declare_parameter<double>("side_obstacle_release_distance_m", 0.50);
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
      declare_parameter<double>("visual_clear_takeover_side_clearance_m", 0.50);
    visual_clear_takeover_min_avoidance_s_ =
      declare_parameter<double>("visual_clear_takeover_min_avoidance_s", 0.60);
    visual_clear_takeover_body_max_age_s_ =
      declare_parameter<double>("visual_clear_takeover_body_max_age_s", 0.30);
    visual_clear_takeover_confirm_frames_ = static_cast<int>(
      declare_parameter<int>("visual_clear_takeover_confirm_frames", 3));
    visual_clear_takeover_confirm_frames_ =
      std::max(1, visual_clear_takeover_confirm_frames_);

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

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin_action_name_);

    body_sub_ = create_subscription<bodyreader_msg::msg::Bodyposture>(
      "/body_posture_yolo_validated", 10,
      std::bind(&BodyNav2Follower::body_callback, this, std::placeholders::_1));

    mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/mode", 10,
      std::bind(&BodyNav2Follower::mode_callback, this, std::placeholders::_1));

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
    last_person_map_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_clear_person_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_cmd_time_ = this->now();
    last_cmd_motion_time_ = this->now();
    last_scan_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_direct_cmd_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_avoidance_goal_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    avoidance_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    avoidance_goal_accept_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    filtered_x_angle_ = 0.0;
    last_visual_error_angle_ = 0.0;
    last_visual_error_distance_ = 0.0;
    RCLCPP_INFO(
      get_logger(),
      "BodyNav2Follower hybrid ready: action=%s spin=%s scan=%s visual_cmd=%s follow_distance=%.2fm",
      nav_action_name_.c_str(), spin_action_name_.c_str(), scan_topic_.c_str(),
      visual_cmd_topic_.c_str(), follow_distance_m_);
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

  void body_callback(const bodyreader_msg::msg::Bodyposture::SharedPtr msg)
  {
    const bool valid_lock = msg->lock_status == 2;
    const bool valid_depth = msg->centerofmass_z > 100.0f;
    if (valid_lock && valid_depth) {
      latest_body_ = *msg;
      has_body_ = true;
      last_body_time_ = this->now();
      ++body_generation_;
    } else {
      has_body_ = false;
      reset_visual_clear_takeover_confirmation();
    }
  }

  void mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    const int previous_mode = mode_;
    mode_ = msg->data;
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
      visual_recent && latest_body_.centerofmass_z > 100.0f &&
      target_exemption_distance_margin_m_ > 0.0;

    if (person_exemption) {
      const double camera_forward =
        latest_body_.centerofmass_z * 0.001 * BODY_NAV2_PERSON_MAP_DEPTH_SCALE;
      const double camera_lateral =
        latest_body_.centerofmass_x * 0.001 * BODY_NAV2_PERSON_MAP_DEPTH_SCALE;
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
      if (std::isfinite(range) &&
          range >= last_scan_.range_min &&
          range <= last_scan_.range_max) {
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
          if (std::fabs(normalized_angle) <= obstacle_front_angle_rad_ &&
              distance < obstacles.front.distance) {
            obstacles.front.distance = distance;
            obstacles.front.angle = normalized_angle;
          }

          if (normalized_angle >= side_min && normalized_angle <= side_max &&
              distance < obstacles.left.distance) {
            obstacles.left.distance = distance;
            obstacles.left.angle = normalized_angle;
          }
          if (normalized_angle <= -side_min && normalized_angle >= -side_max &&
              distance < obstacles.right.distance) {
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

    const double body_age = (sample_time - last_body_time_).seconds();
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
        << ",\"body\":{\"id\":" << latest_body_.bodyid
        << ",\"lock\":" << static_cast<int>(latest_body_.lock_status)
        << ",\"age_s\":" << body_age
        << ",\"raw_mm\":[" << latest_body_.centerofmass_x << ","
        << latest_body_.centerofmass_y << "," << latest_body_.centerofmass_z << "]}"
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
      latest_body_.bodyid,
      latest_body_.centerofmass_x, latest_body_.centerofmass_y, latest_body_.centerofmass_z,
      base_x, base_y, map_person.point.x, map_person.point.y, reference_error,
      ray_distance, ray_angle, effective_front_distance);
  }

  void publish_paused_calibration(const rclcpp::Time & now)
  {
    if (!has_body_ || (now - last_body_time_).seconds() > lost_timeout_s_) {
      return;
    }

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

  void reset_avoidance_episode()
  {
    avoidance_stage_ = AvoidanceStage::IDLE;
    last_sent_goal_kind_ = NavGoalKind::NORMAL;
    has_avoidance_gate_goal_ = false;
    avoidance_trigger_count_ = 0;
    left_avoidance_trigger_count_ = 0;
    right_avoidance_trigger_count_ = 0;
    active_avoidance_obstacle_ = ObstacleObservation();
    visual_reverse_takeover_count_ = 0;
    visual_reverse_takeover_pending_ = false;
    visual_clear_takeover_pending_ = false;
    reset_visual_clear_takeover_confirmation();
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
    const double decel_limit = decel_limit_override >= 0.0 ?
      decel_limit_override : visual_linear_decel_limit_;
    const double linear_limit = speeding_up ? visual_linear_accel_limit_ : decel_limit;
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
    double minimum_forward_mps = 0.0)
  {
    const auto now = this->now();
    const double raw_x_angle = latest_body_.centerofmass_x / latest_body_.centerofmass_z;
    const double raw_distance_mm = latest_body_.centerofmass_z;

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
      const double braking_accel_mps2 = std::max(1e-3, visual_linear_accel_limit_);
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
    desired.angular.z *= std::max(safe_linear_scale, 0.55);

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
      decel_limit_override : visual_linear_accel_limit_;
    const auto cmd = smooth_direct_cmd(desired, now, decel_limit);
    visual_cmd_pub_->publish(cmd);
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

  void reset_visual_clear_takeover_confirmation()
  {
    visual_clear_takeover_count_ = 0;
    visual_clear_takeover_last_body_generation_ = body_generation_;
  }

  bool visual_clear_takeover_confirmed(
    const rclcpp::Time & now,
    const ObstacleScan & obstacles,
    bool avoidance_active,
    bool nav_engaged,
    bool visual_too_close)
  {
    const bool body_fresh =
      (now - last_body_time_).seconds() <= visual_clear_takeover_body_max_age_s_;
    const double avoidance_elapsed_s =
      avoidance_start_time_.nanoseconds() == 0 ? 0.0 :
      std::max(0.0, (now - avoidance_start_time_).seconds());
    const bool front_clear =
      !std::isfinite(obstacles.front.distance) ||
      obstacles.front.distance >= visual_clear_takeover_front_clearance_m_;
    const bool left_clear =
      !std::isfinite(obstacles.left.distance) ||
      obstacles.left.distance >= visual_clear_takeover_side_clearance_m_;
    const bool right_clear =
      !std::isfinite(obstacles.right.distance) ||
      obstacles.right.distance >= visual_clear_takeover_side_clearance_m_;
    const bool candidate =
      avoidance_active && nav_engaged && !visual_too_close && body_fresh &&
      avoidance_elapsed_s >= visual_clear_takeover_min_avoidance_s_ &&
      front_clear && left_clear && right_clear;

    if (!candidate) {
      reset_visual_clear_takeover_confirmation();
      return false;
    }
    if (visual_clear_takeover_last_body_generation_ == body_generation_) {
      return false;
    }

    visual_clear_takeover_last_body_generation_ = body_generation_;
    ++visual_clear_takeover_count_;
    if (visual_clear_takeover_count_ == 1) {
      RCLCPP_INFO(
        get_logger(),
        "Visual clear takeover candidate: front=%.2f left=%.2f right=%.2f "
        "body_age=%.3f avoidance_elapsed=%.2f required_frames=%d",
        obstacles.front.distance, obstacles.left.distance, obstacles.right.distance,
        (now - last_body_time_).seconds(), avoidance_elapsed_s,
        visual_clear_takeover_confirm_frames_);
    }

    if (visual_clear_takeover_count_ < visual_clear_takeover_confirm_frames_) {
      publish_state("visual_clear_takeover_confirming");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Visual clear takeover confirmed: front=%.2f left=%.2f right=%.2f frames=%d",
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

    geometry_msgs::msg::PoseStamped goal;
    NavGoalKind goal_kind = NavGoalKind::NORMAL;
    const bool visual_recent = has_body_ && (now - last_body_time_).seconds() <= lost_timeout_s_;
    const bool has_memory = has_last_person_map_;
    const double memory_age = has_memory ? (now - last_person_map_time_).seconds() : 0.0;
    const bool avoidance_active = avoidance_episode_active();
    if (!visual_recent) {
      reset_visual_clear_takeover_confirmation();
    }

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

    if (visual_clear_takeover_pending_) {
      cancel_goal("visual_clear_takeover_during_avoidance");
      if (!goal_active_ && !has_sent_goal_) {
        reset_avoidance_episode();
        avoidance_rearm_required_ = true;
        awaiting_visual_after_avoidance_ = false;
      }
      reset_visual_controller();
      publish_safety_stop_cmd("cancel_nav2_visual_clear_during_avoidance");
      return;
    }

    if (visual_reverse_takeover_pending_) {
      cancel_goal("visual_reverse_takeover_during_avoidance");
      if (!goal_active_ && !has_sent_goal_) {
        reset_avoidance_episode();
        avoidance_rearm_required_ = true;
      }
      reset_visual_controller();
      publish_safety_stop_cmd("cancel_nav2_visual_reverse_during_avoidance");
      return;
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
      const bool nav_engaged = goal_active_ || has_sent_goal_;
      const bool visual_too_close =
        latest_body_.centerofmass_z * 0.001 < follow_distance_m_ - hold_distance_band_m_;

      visual_reverse_takeover_count_ =
        avoidance_active && visual_allow_reverse_ && visual_too_close ?
        std::min(
          visual_reverse_takeover_count_ + 1,
          BODY_NAV2_VISUAL_REVERSE_TAKEOVER_CONFIRM_TICKS) : 0;
      if (visual_reverse_takeover_count_ >=
          BODY_NAV2_VISUAL_REVERSE_TAKEOVER_CONFIRM_TICKS)
      {
        visual_reverse_takeover_pending_ = true;
        cancel_goal("visual_reverse_takeover_during_avoidance");
        if (!goal_active_ && !has_sent_goal_) {
          reset_avoidance_episode();
          avoidance_rearm_required_ = true;
        }
        reset_visual_controller();
        publish_safety_stop_cmd("cancel_nav2_visual_reverse_during_avoidance");
        return;
      }

      if (visual_clear_takeover_confirmed(
          now, obstacles, avoidance_active, nav_engaged, visual_too_close))
      {
        visual_clear_takeover_pending_ = true;
        awaiting_visual_after_avoidance_ = false;
        cancel_goal("visual_clear_takeover_during_avoidance");
        if (!goal_active_ && !has_sent_goal_) {
          reset_avoidance_episode();
          avoidance_rearm_required_ = true;
        }
        reset_visual_controller();
        publish_safety_stop_cmd("cancel_nav2_visual_clear_during_avoidance");
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
            cancel_goal("visual_reverse_takeover");
            reset_avoidance_episode();
            publish_stop_cmd("cancel_nav2_visual_reverse");
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
            cancel_goal("visual_direct_takeover");
            reset_avoidance_episode();
            publish_stop_cmd("cancel_nav2_visual_direct");
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

      start_reacquire_spin("post_avoidance_reacquire", true);
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
          return;
        }

        if (!goal_handle) {
          spin_active_ = false;
          spin_handle_.reset();
          publish_state("spin_rejected");
          RCLCPP_WARN(get_logger(), "Nav2 rejected reacquire spin goal");
          return;
        }

        spin_active_ = true;
        spin_cancel_sent_ = false;
        spin_handle_ = goal_handle;
        publish_state("spin_accepted");
      };

    options.result_callback =
      [this, spin_sequence](const GoalHandleSpin::WrappedResult & result) {
        if (spin_sequence != spin_sequence_) {
          return;
        }

        spin_active_ = false;
        spin_cancel_sent_ = false;
        spin_handle_.reset();

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
      latest_body_.centerofmass_z * 0.001 * BODY_NAV2_PERSON_MAP_DEPTH_SCALE;
    const double camera_left =
      latest_body_.centerofmass_x * 0.001 * BODY_NAV2_PERSON_MAP_DEPTH_SCALE;
    const double camera_up = latest_body_.centerofmass_y * 0.001;

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
    NavigateToPose::Goal goal;
    goal.pose = goal_pose;
    goal.behavior_tree = behavior_tree_;

    const auto goal_sequence = ++goal_sequence_;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this, goal_sequence, goal_kind](GoalHandleNavigate::SharedPtr goal_handle) {
        if (goal_sequence != goal_sequence_) {
          return;
        }

        if (!goal_handle) {
          goal_active_ = false;
          goal_handle_.reset();
          has_sent_goal_ = false;
          nav_retry_waiting_ = true;
          last_failed_goal_time_ = this->now();
          if (goal_kind == NavGoalKind::AVOID_GATE) {
            publish_safety_stop_cmd("nav2_avoid_goal_rejected_stop");
          } else {
            publish_state("goal_rejected");
          }
          RCLCPP_WARN(get_logger(), "Nav2 rejected body follow goal");
          return;
        }

        goal_active_ = true;
        cancel_sent_ = false;
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
      };

    options.result_callback =
      [this, goal_sequence, goal_kind](const GoalHandleNavigate::WrappedResult & result) {
        if (goal_sequence != goal_sequence_) {
          return;
        }

        goal_active_ = false;
        cancel_sent_ = false;
        goal_handle_.reset();
        has_sent_goal_ = false;
        const bool visual_clear_takeover_requested = visual_clear_takeover_pending_;

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            nav_retry_waiting_ = false;
            if (goal_kind == NavGoalKind::AVOID_GATE) {
              if (visual_clear_takeover_requested) {
                reset_avoidance_episode();
                avoidance_rearm_required_ = true;
                awaiting_visual_after_avoidance_ = false;
                RCLCPP_INFO(
                  get_logger(),
                  "Avoidance goal completed while visual clear takeover was pending");
                publish_state("nav2_avoid_completed_for_visual_clear");
              } else {
                reset_avoidance_episode();
                avoidance_rearm_required_ = true;
                awaiting_visual_after_avoidance_ = true;
                publish_state("nav2_avoid_gate_succeeded_wait_visual");
              }
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
              if (visual_clear_takeover_requested) {
                reset_avoidance_episode();
                avoidance_rearm_required_ = true;
                awaiting_visual_after_avoidance_ = false;
                RCLCPP_INFO(
                  get_logger(), "Nav2 avoidance canceled for visual clear takeover");
                publish_state("nav2_avoid_canceled_for_visual_clear");
              } else if (visual_reverse_takeover_pending_) {
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
    if (cancel_sent_) {
      return;
    }

    if (goal_handle_) {
      nav_client_->async_cancel_goal(goal_handle_);
      cancel_sent_ = true;
      has_sent_goal_ = false;
      publish_state("cancel_" + reason);
    } else if (goal_active_) {
      nav_client_->async_cancel_all_goals();
      cancel_sent_ = true;
      has_sent_goal_ = false;
      publish_state("cancel_all_" + reason);
    }
  }

  void cancel_spin(const std::string & reason)
  {
    if (spin_cancel_sent_) {
      return;
    }

    if (spin_handle_) {
      spin_client_->async_cancel_goal(spin_handle_);
      spin_cancel_sent_ = true;
      publish_state("cancel_spin_" + reason);
    } else if (spin_active_) {
      spin_client_->async_cancel_all_goals();
      spin_cancel_sent_ = true;
      publish_state("cancel_all_spin_" + reason);
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
  std::string global_frame_;
  std::string base_frame_;
  std::string body_frame_id_;
  std::string behavior_tree_;
  std::string last_state_;

  double follow_distance_m_;
  double hold_distance_band_m_;
  double min_goal_distance_m_;
  double goal_update_period_s_;
  double goal_position_tolerance_m_;
  double goal_yaw_tolerance_rad_;
  double lost_timeout_s_;
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
  int visual_clear_takeover_count_{0};
  AvoidanceStage avoidance_stage_;
  NavGoalKind last_sent_goal_kind_;

  bodyreader_msg::msg::Bodyposture latest_body_;
  bool has_body_;
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
  bool visual_clear_takeover_pending_{false};
  bool has_avoidance_gate_goal_;
  bool avoidance_motion_logged_;
  bool has_calibration_reference_;
  uint64_t goal_sequence_;
  uint64_t spin_sequence_;
  uint64_t body_generation_{0};
  uint64_t visual_clear_takeover_last_body_generation_{0};

  rclcpp::Time last_body_time_;
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
  rclcpp::Subscription<bodyreader_msg::msg::Bodyposture>::SharedPtr body_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
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
