#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

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

class BodyNav2Follower : public rclcpp::Node
{
  enum class AvoidanceStage
  {
    IDLE,
    GATE,
    ANCHOR
  };

  enum class NavGoalKind
  {
    NORMAL,
    AVOID_GATE,
    AVOID_ANCHOR
  };

public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using Spin = nav2_msgs::action::Spin;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

  BodyNav2Follower()
  : Node("body_nav2_follower"),
    mode_(2),
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
    direct_cmd_init_(false),
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
    obstacle_slow_distance_m_ = declare_parameter<double>("obstacle_slow_distance_m", 1.25);
    obstacle_nav_distance_m_ = declare_parameter<double>("obstacle_nav_distance_m", 0.75);
    obstacle_nav_release_distance_m_ =
      declare_parameter<double>("obstacle_nav_release_distance_m", 1.35);
    visual_min_linear_scale_ = declare_parameter<double>("visual_min_linear_scale", 0.25);
    target_exemption_angle_rad_ = declare_parameter<double>("target_exemption_angle_rad", 0.30);
    target_exemption_distance_margin_m_ =
      declare_parameter<double>("target_exemption_distance_margin_m", 0.45);
    avoidance_lateral_offset_m_ = declare_parameter<double>("avoidance_lateral_offset_m", 0.65);
    avoidance_forward_margin_m_ = declare_parameter<double>("avoidance_forward_margin_m", 0.35);
    avoidance_min_forward_step_m_ =
      declare_parameter<double>("avoidance_min_forward_step_m", 0.45);
    avoidance_max_forward_step_m_ =
      declare_parameter<double>("avoidance_max_forward_step_m", 1.10);
    avoidance_side_angle_rad_ = declare_parameter<double>("avoidance_side_angle_rad", 1.35);
    avoidance_side_clearance_min_m_ =
      declare_parameter<double>("avoidance_side_clearance_min_m", 0.45);
    avoidance_target_side_bias_m_ =
      declare_parameter<double>("avoidance_target_side_bias_m", 0.20);
    avoidance_side_hold_s_ = declare_parameter<double>("avoidance_side_hold_s", 2.0);
    avoidance_occlusion_hold_s_ =
      declare_parameter<double>("avoidance_occlusion_hold_s", 6.0);
    avoidance_gate_switch_distance_m_ =
      declare_parameter<double>("avoidance_gate_switch_distance_m", 0.45);
    avoidance_gate_timeout_s_ =
      declare_parameter<double>("avoidance_gate_timeout_s", 5.0);
    avoidance_anchor_blend_alpha_ =
      declare_parameter<double>("avoidance_anchor_blend_alpha", 0.20);

    visual_x_p_ = declare_parameter<double>("visual_x_p", 0.5);
    visual_x_d_ = declare_parameter<double>("visual_x_d", 0.33);
    visual_z_p_ = declare_parameter<double>("visual_z_p", 1.2);
    visual_z_d_ = declare_parameter<double>("visual_z_d", 0.5);
    visual_filter_alpha_ = declare_parameter<double>("visual_filter_alpha", 0.35);
    visual_angle_deadband_ = declare_parameter<double>("visual_angle_deadband", 0.025);
    visual_distance_deadband_mm_ = declare_parameter<double>("visual_distance_deadband_mm", 80.0);
    visual_max_linear_mps_ = declare_parameter<double>("visual_max_linear_mps", 0.5);
    visual_max_angular_rps_ = declare_parameter<double>("visual_max_angular_rps", 1.0);
    visual_linear_accel_limit_ = declare_parameter<double>("visual_linear_accel_limit", 0.45);
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
    mode_required_ = declare_parameter<int>("mode_required", 2);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin_action_name_);

    body_sub_ = create_subscription<bodyreader_msg::msg::Bodyposture>(
      "/body_posture", 10,
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

    visual_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(visual_cmd_topic_, 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/body_nav2_goal", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/body_nav2_state", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BodyNav2Follower::control_loop, this));

    last_goal_send_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_failed_goal_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_spin_send_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_body_time_ = this->now();
    last_person_map_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_cmd_time_ = this->now();
    last_cmd_motion_time_ = this->now();
    last_scan_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_direct_cmd_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_avoidance_side_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_avoidance_goal_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_clear_anchor_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    avoidance_stage_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    filtered_x_angle_ = 0.0;
    last_visual_error_angle_ = 0.0;
    last_visual_error_distance_ = 0.0;
    last_avoidance_side_ = 0;
    avoidance_stage_ = AvoidanceStage::IDLE;
    last_sent_goal_kind_ = NavGoalKind::NORMAL;
    has_last_clear_anchor_ = false;
    has_active_avoidance_anchor_ = false;
    has_avoidance_gate_goal_ = false;

    RCLCPP_INFO(
      get_logger(),
      "BodyNav2Follower hybrid ready: action=%s spin=%s scan=%s visual_cmd=%s follow_distance=%.2fm",
      nav_action_name_.c_str(), spin_action_name_.c_str(), scan_topic_.c_str(),
      visual_cmd_topic_.c_str(), follow_distance_m_);
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
    }
  }

  void mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    mode_ = msg->data;
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    has_cmd_ = true;
    const auto now = this->now();
    last_cmd_time_ = now;

    const double linear = std::hypot(msg->linear.x, msg->linear.y);
    const double angular = std::fabs(msg->angular.z);
    if (linear > cmd_zero_linear_epsilon_ || angular > cmd_zero_angular_epsilon_) {
      last_cmd_motion_time_ = now;
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = *msg;
    scan_received_ = true;
    last_scan_time_ = this->now();
  }

  bool scan_recent(const rclcpp::Time & now) const
  {
    return scan_received_ && (now - last_scan_time_).seconds() <= scan_timeout_s_;
  }

  double front_obstacle_distance(bool visual_recent) const
  {
    if (!scan_received_) {
      return std::numeric_limits<double>::infinity();
    }

    double person_angle = 0.0;
    double person_distance = 0.0;
    const bool person_exemption =
      visual_recent && latest_body_.centerofmass_z > 100.0f &&
      target_exemption_distance_margin_m_ > 0.0;

    if (person_exemption) {
      person_angle = std::atan2(latest_body_.centerofmass_x, latest_body_.centerofmass_z);
      person_distance = latest_body_.centerofmass_z * 0.001;
    }

    double min_distance = std::numeric_limits<double>::infinity();
    double angle = last_scan_.angle_min;
    for (const auto range : last_scan_.ranges) {
      const double normalized_angle = normalize_angle(angle);
      if (std::fabs(normalized_angle) <= obstacle_front_angle_rad_ &&
          std::isfinite(range) &&
          range >= last_scan_.range_min &&
          range <= last_scan_.range_max) {
        const bool is_target =
          person_exemption &&
          std::fabs(normalize_angle(normalized_angle - person_angle)) <=
            target_exemption_angle_rad_ &&
          std::fabs(static_cast<double>(range) - person_distance) <=
            target_exemption_distance_margin_m_;

        if (!is_target) {
          min_distance = std::min(min_distance, static_cast<double>(range));
        }
      }
      angle += last_scan_.angle_increment;
    }

    return min_distance;
  }

  double scan_sector_clearance(double min_angle, double max_angle) const
  {
    if (!scan_received_) {
      return std::numeric_limits<double>::infinity();
    }

    double clearance = std::numeric_limits<double>::infinity();
    double angle = last_scan_.angle_min;
    for (const auto range : last_scan_.ranges) {
      const double normalized_angle = normalize_angle(angle);
      if (normalized_angle >= min_angle &&
          normalized_angle <= max_angle &&
          std::isfinite(range) &&
          range >= last_scan_.range_min &&
          range <= last_scan_.range_max) {
        clearance = std::min(clearance, static_cast<double>(range));
      }
      angle += last_scan_.angle_increment;
    }

    return clearance;
  }

  int choose_avoidance_side(const rclcpp::Time & now)
  {
    if (last_avoidance_side_ != 0 && avoidance_side_hold_s_ > 0.0 &&
        (now - last_avoidance_side_time_).seconds() <= avoidance_side_hold_s_) {
      return last_avoidance_side_;
    }

    const double side_angle =
      std::max(obstacle_front_angle_rad_ + 0.05, avoidance_side_angle_rad_);
    const double left_clearance =
      scan_sector_clearance(obstacle_front_angle_rad_, side_angle);
    const double right_clearance =
      scan_sector_clearance(-side_angle, -obstacle_front_angle_rad_);

    int side = left_clearance >= right_clearance ? 1 : -1;
    const int target_side = latest_body_.centerofmass_x >= 0.0f ? 1 : -1;
    const double target_side_clearance = target_side > 0 ? left_clearance : right_clearance;
    const double other_side_clearance = target_side > 0 ? right_clearance : left_clearance;

    if (target_side_clearance >= avoidance_side_clearance_min_m_ &&
        target_side_clearance + avoidance_target_side_bias_m_ >= other_side_clearance) {
      side = target_side;
    }

    last_avoidance_side_ = side;
    last_avoidance_side_time_ = now;
    return side;
  }

  bool make_avoidance_side_goal(
    geometry_msgs::msg::PoseStamped & map_goal,
    double front_distance)
  {
    if (!enable_avoidance_side_goal_ || !std::isfinite(front_distance)) {
      return false;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!lookup_robot_pose(robot_pose)) {
      publish_state("tf_failed");
      return false;
    }

    const auto now = this->now();
    const int side = choose_avoidance_side(now);
    const double forward = clamp_value(
      front_distance + avoidance_forward_margin_m_,
      avoidance_min_forward_step_m_, avoidance_max_forward_step_m_);
    const double lateral = static_cast<double>(side) * std::fabs(avoidance_lateral_offset_m_);
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double c = std::cos(robot_yaw);
    const double s = std::sin(robot_yaw);

    const double target_x = robot_pose.pose.position.x + c * forward - s * lateral;
    const double target_y = robot_pose.pose.position.y + s * forward + c * lateral;
    const double yaw_to_person =
      std::atan2(last_person_map_pose_.pose.position.y - target_y,
                 last_person_map_pose_.pose.position.x - target_x);

    map_goal.header.stamp = now;
    map_goal.header.frame_id = global_frame_;
    map_goal.pose.position.x = target_x;
    map_goal.pose.position.y = target_y;
    map_goal.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_to_person);
    map_goal.pose.orientation = tf2::toMsg(q);

    goal_pub_->publish(map_goal);
    last_avoidance_goal_time_ = now;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Avoidance side goal: side=%d front=%.2f forward=%.2f lateral=%.2f",
      side, front_distance, forward, lateral);
    publish_state(side > 0 ? "nav2_avoid_side_left_goal_ready" :
                             "nav2_avoid_side_right_goal_ready");
    return true;
  }

  static double pose_distance_xy(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b)
  {
    return std::hypot(
      a.pose.position.x - b.pose.position.x,
      a.pose.position.y - b.pose.position.y);
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

  bool compute_follow_anchor_pose(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & person_pose,
    geometry_msgs::msg::PoseStamped & anchor)
  {
    const double person_x = person_pose.pose.position.x - robot_pose.pose.position.x;
    const double person_y = person_pose.pose.position.y - robot_pose.pose.position.y;
    const double person_dist = std::hypot(person_x, person_y);
    if (person_dist < 1e-3) {
      return false;
    }

    const double unit_x = person_x / person_dist;
    const double unit_y = person_y / person_dist;
    double target_x = person_pose.pose.position.x - unit_x * follow_distance_m_;
    double target_y = person_pose.pose.position.y - unit_y * follow_distance_m_;
    double target_dx = target_x - robot_pose.pose.position.x;
    double target_dy = target_y - robot_pose.pose.position.y;
    double target_dist = std::hypot(target_dx, target_dy);

    if (target_dist < min_goal_distance_m_) {
      if (target_dist < 1e-6) {
        return false;
      }
      const double scale = min_goal_distance_m_ / target_dist;
      target_dx *= scale;
      target_dy *= scale;
      target_x = robot_pose.pose.position.x + target_dx;
      target_y = robot_pose.pose.position.y + target_dy;
    }

    const double yaw_to_person =
      std::atan2(person_pose.pose.position.y - target_y,
                 person_pose.pose.position.x - target_x);
    fill_map_goal(anchor, target_x, target_y, yaw_to_person);
    return true;
  }

  void remember_last_clear_anchor(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    geometry_msgs::msg::PoseStamped anchor;
    if (!compute_follow_anchor_pose(robot_pose, last_person_map_pose_, anchor)) {
      return;
    }

    last_clear_follow_anchor_ = anchor;
    last_clear_anchor_time_ = this->now();
    has_last_clear_anchor_ = true;
  }

  bool init_active_avoidance_anchor(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    const auto now = this->now();
    if (has_last_clear_anchor_ &&
        (now - last_clear_anchor_time_).seconds() <= target_memory_timeout_s_) {
      active_avoidance_anchor_ = last_clear_follow_anchor_;
      has_active_avoidance_anchor_ = true;
      return true;
    }

    if (!has_last_person_map_) {
      return false;
    }

    if (!compute_follow_anchor_pose(robot_pose, last_person_map_pose_, active_avoidance_anchor_)) {
      return false;
    }

    has_active_avoidance_anchor_ = true;
    return true;
  }

  void update_active_avoidance_anchor_from_visual(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double front_distance)
  {
    if (!has_active_avoidance_anchor_) {
      return;
    }

    geometry_msgs::msg::PoseStamped visual_anchor;
    if (!compute_follow_anchor_pose(robot_pose, last_person_map_pose_, visual_anchor)) {
      return;
    }

    const double span =
      std::max(1e-3, obstacle_nav_release_distance_m_ - obstacle_nav_distance_m_);
    const double clearance_ratio = clamp_value(
      (front_distance - obstacle_nav_distance_m_) / span, 0.0, 1.0);
    const double danger_gain = 0.20 + 0.80 * clearance_ratio;
    const double gain = clamp_value(
      avoidance_anchor_blend_alpha_ * danger_gain, 0.0, 1.0);

    const double blended_x =
      (1.0 - gain) * active_avoidance_anchor_.pose.position.x +
      gain * visual_anchor.pose.position.x;
    const double blended_y =
      (1.0 - gain) * active_avoidance_anchor_.pose.position.y +
      gain * visual_anchor.pose.position.y;
    const double yaw_to_person =
      std::atan2(last_person_map_pose_.pose.position.y - blended_y,
                 last_person_map_pose_.pose.position.x - blended_x);

    fill_map_goal(active_avoidance_anchor_, blended_x, blended_y, yaw_to_person);
  }

  bool make_avoidance_gate_goal(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double front_distance,
    geometry_msgs::msg::PoseStamped & gate_goal)
  {
    if (!enable_avoidance_side_goal_ || !std::isfinite(front_distance)) {
      return false;
    }

    const auto now = this->now();
    const int side = choose_avoidance_side(now);
    const double forward = clamp_value(
      front_distance + avoidance_forward_margin_m_,
      avoidance_min_forward_step_m_, avoidance_max_forward_step_m_);
    const double lateral = static_cast<double>(side) * std::fabs(avoidance_lateral_offset_m_);
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double c = std::cos(robot_yaw);
    const double s = std::sin(robot_yaw);

    const double target_x = robot_pose.pose.position.x + c * forward - s * lateral;
    const double target_y = robot_pose.pose.position.y + s * forward + c * lateral;
    const double yaw_to_gate =
      std::atan2(target_y - robot_pose.pose.position.y,
                 target_x - robot_pose.pose.position.x);

    fill_map_goal(gate_goal, target_x, target_y, yaw_to_gate);
    last_avoidance_goal_time_ = now;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Avoidance gate goal: side=%d front=%.2f forward=%.2f lateral=%.2f",
      side, front_distance, forward, lateral);
    return true;
  }

  bool should_switch_avoidance_to_anchor(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double front_distance) const
  {
    if (avoidance_stage_ != AvoidanceStage::GATE || !has_avoidance_gate_goal_) {
      return false;
    }

    if (pose_distance_xy(robot_pose, avoidance_gate_goal_) <= avoidance_gate_switch_distance_m_) {
      return true;
    }

    if (std::isfinite(front_distance) && front_distance >= obstacle_nav_release_distance_m_) {
      return true;
    }

    return avoidance_gate_timeout_s_ > 0.0 &&
      (this->now() - avoidance_stage_time_).seconds() >= avoidance_gate_timeout_s_;
  }

  bool make_avoidance_nav_goal(
    geometry_msgs::msg::PoseStamped & map_goal,
    double front_distance,
    bool visual_recent,
    NavGoalKind & goal_kind)
  {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!lookup_robot_pose(robot_pose)) {
      publish_state("tf_failed");
      return false;
    }

    const auto now = this->now();
    if (avoidance_stage_ == AvoidanceStage::IDLE) {
      if (!init_active_avoidance_anchor(robot_pose)) {
        goal_kind = NavGoalKind::NORMAL;
        return make_avoidance_side_goal(map_goal, front_distance);
      }

      if (!make_avoidance_gate_goal(robot_pose, front_distance, avoidance_gate_goal_)) {
        goal_kind = NavGoalKind::NORMAL;
        return make_avoidance_side_goal(map_goal, front_distance);
      }

      avoidance_stage_ = AvoidanceStage::GATE;
      avoidance_stage_time_ = now;
      has_avoidance_gate_goal_ = true;
    }

    if (visual_recent) {
      update_active_avoidance_anchor_from_visual(robot_pose, front_distance);
    }

    if (should_switch_avoidance_to_anchor(robot_pose, front_distance)) {
      avoidance_stage_ = AvoidanceStage::ANCHOR;
      avoidance_stage_time_ = now;
      has_sent_goal_ = false;
      publish_state("nav2_avoid_switch_anchor");
    }

    if (avoidance_stage_ == AvoidanceStage::GATE && has_avoidance_gate_goal_) {
      map_goal = avoidance_gate_goal_;
      map_goal.header.stamp = now;
      goal_pub_->publish(map_goal);
      goal_kind = NavGoalKind::AVOID_GATE;
      last_avoidance_goal_time_ = now;
      publish_state("nav2_avoid_gate_goal_ready");
      return true;
    }

    if (has_active_avoidance_anchor_) {
      map_goal = active_avoidance_anchor_;
      map_goal.header.stamp = now;
      goal_pub_->publish(map_goal);
      goal_kind = NavGoalKind::AVOID_ANCHOR;
      last_avoidance_goal_time_ = now;
      publish_state("nav2_avoid_anchor_goal_ready");
      return true;
    }

    return false;
  }

  void reset_avoidance_episode()
  {
    avoidance_stage_ = AvoidanceStage::IDLE;
    last_sent_goal_kind_ = NavGoalKind::NORMAL;
    has_active_avoidance_anchor_ = false;
    has_avoidance_gate_goal_ = false;
  }

  bool recent_avoidance_goal(const rclcpp::Time & now) const
  {
    if (avoidance_occlusion_hold_s_ <= 0.0) {
      return false;
    }

    return (now - last_avoidance_goal_time_).seconds() <= avoidance_occlusion_hold_s_;
  }

  double visual_scale_from_obstacle(double front_distance) const
  {
    if (!std::isfinite(front_distance) || front_distance >= obstacle_slow_distance_m_) {
      return 1.0;
    }
    if (front_distance <= obstacle_nav_distance_m_) {
      return 0.0;
    }

    const double span = obstacle_slow_distance_m_ - obstacle_nav_distance_m_;
    if (span <= 1e-3) {
      return visual_min_linear_scale_;
    }

    const double ratio = (front_distance - obstacle_nav_distance_m_) / span;
    return visual_min_linear_scale_ + ratio * (1.0 - visual_min_linear_scale_);
  }

  geometry_msgs::msg::Twist smooth_direct_cmd(
    const geometry_msgs::msg::Twist & desired,
    const rclcpp::Time & now)
  {
    if (!direct_cmd_init_) {
      direct_cmd_init_ = true;
      last_direct_cmd_time_ = now;
      last_direct_cmd_ = desired;
      return desired;
    }

    double dt = (now - last_direct_cmd_time_).seconds();
    if (dt <= 1e-3 || dt > 0.5) {
      dt = 0.1;
    }

    geometry_msgs::msg::Twist cmd = desired;
    const double max_linear_delta = std::max(0.0, visual_linear_accel_limit_) * dt;
    const double max_angular_delta = std::max(0.0, visual_angular_accel_limit_) * dt;

    cmd.linear.x = last_direct_cmd_.linear.x + clamp_value(
      desired.linear.x - last_direct_cmd_.linear.x,
      -max_linear_delta, max_linear_delta);
    cmd.angular.z = last_direct_cmd_.angular.z + clamp_value(
      desired.angular.z - last_direct_cmd_.angular.z,
      -max_angular_delta, max_angular_delta);

    last_direct_cmd_ = cmd;
    last_direct_cmd_time_ = now;
    return cmd;
  }

  void publish_stop_cmd(const std::string & state)
  {
    geometry_msgs::msg::Twist stop;
    last_direct_cmd_ = stop;
    direct_cmd_init_ = true;
    last_direct_cmd_time_ = this->now();
    visual_cmd_pub_->publish(stop);
    publish_state(state);
  }

  bool publish_visual_follow_cmd(double linear_scale, const std::string & state)
  {
    const auto now = this->now();
    const double raw_x_angle = latest_body_.centerofmass_x / latest_body_.centerofmass_z;
    const double distance_mm = latest_body_.centerofmass_z;

    if (!visual_filter_init_) {
      filtered_x_angle_ = raw_x_angle;
      visual_filter_init_ = true;
    } else {
      const double alpha = clamp_value(visual_filter_alpha_, 0.0, 1.0);
      filtered_x_angle_ = alpha * raw_x_angle + (1.0 - alpha) * filtered_x_angle_;
    }

    double error_x_angle = filtered_x_angle_;
    double error_distance = distance_mm - follow_distance_m_ * 1000.0;

    if (std::fabs(error_x_angle) < visual_angle_deadband_) {
      error_x_angle = 0.0;
    }
    if (std::fabs(error_distance) < visual_distance_deadband_mm_) {
      error_distance = 0.0;
    }

    geometry_msgs::msg::Twist desired;
    desired.linear.x =
      error_distance * visual_x_p_ / 1000.0 +
      (error_distance - last_visual_error_distance_) * visual_x_d_ / 1000.0;
    desired.angular.z =
      error_x_angle * visual_z_p_ +
      (error_x_angle - last_visual_error_angle_) * visual_z_d_;

    const double safe_linear_scale = clamp_value(linear_scale, 0.0, 1.0);
    desired.linear.x *= safe_linear_scale;
    desired.angular.z *= std::max(safe_linear_scale, 0.55);

    if (!visual_allow_reverse_ && desired.linear.x < 0.0) {
      desired.linear.x = 0.0;
    }

    desired.linear.x = clamp_value(
      desired.linear.x, visual_allow_reverse_ ? -visual_max_linear_mps_ : 0.0,
      visual_max_linear_mps_);
    desired.angular.z = clamp_value(
      desired.angular.z, -visual_max_angular_rps_, visual_max_angular_rps_);

    if (std::fabs(desired.linear.x) < 0.02) {
      desired.linear.x = 0.0;
    }
    if (std::fabs(desired.angular.z) < 0.02) {
      desired.angular.z = 0.0;
    }

    const auto cmd = smooth_direct_cmd(desired, now);
    visual_cmd_pub_->publish(cmd);
    last_visual_error_angle_ = error_x_angle;
    last_visual_error_distance_ = error_distance;
    publish_state(state);
    return true;
  }

  void reset_visual_controller()
  {
    visual_filter_init_ = false;
    last_visual_error_angle_ = 0.0;
    last_visual_error_distance_ = 0.0;
  }

  void control_loop()
  {
    const auto now = this->now();

    if (respect_mode_topic_ && mode_ != mode_required_) {
      cancel_goal("mode_paused");
      cancel_spin("mode_paused");
      reset_avoidance_episode();
      reset_visual_controller();
      publish_stop_cmd("paused_by_mode");
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    NavGoalKind goal_kind = NavGoalKind::NORMAL;
    const bool visual_recent = has_body_ && (now - last_body_time_).seconds() <= lost_timeout_s_;
    const bool has_memory = has_last_person_map_;
    const double memory_age = has_memory ? (now - last_person_map_time_).seconds() : 0.0;

    if (visual_recent) {
      if (spin_active_) {
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

      if (require_scan_for_visual_ && !scan_recent(now)) {
        cancel_goal("scan_stale");
        cancel_spin("scan_stale");
        reset_visual_controller();
        publish_stop_cmd("scan_stale_stop");
        return;
      }

      const double front_distance =
        scan_recent(now) ? front_obstacle_distance(true) : std::numeric_limits<double>::infinity();
      const bool nav_engaged = goal_active_ || has_sent_goal_;
      const bool visual_too_close =
        latest_body_.centerofmass_z * 0.001 < follow_distance_m_ - hold_distance_band_m_;

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

      const bool nav_takeover =
        front_distance <= obstacle_nav_distance_m_ ||
        (nav_engaged && front_distance <= obstacle_nav_release_distance_m_);

      if (!nav_takeover) {
        geometry_msgs::msg::PoseStamped robot_pose;
        if (front_distance > obstacle_nav_distance_m_ && lookup_robot_pose(robot_pose)) {
          remember_last_clear_anchor(robot_pose);
        }

        if (nav_engaged) {
          cancel_goal("visual_direct_takeover");
          reset_avoidance_episode();
          publish_stop_cmd("cancel_nav2_visual_direct");
          return;
        }

        reset_avoidance_episode();
        const double visual_scale = visual_scale_from_obstacle(front_distance);
        const std::string state =
          visual_scale < 0.99 ? "visual_weighted_follow" : "visual_direct_follow";
        publish_visual_follow_cmd(visual_scale, state);
        return;
      }

      reset_visual_controller();
      if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Nav2 action server %s is not available", nav_action_name_.c_str());
        publish_stop_cmd("nav2_unavailable_stop");
        return;
      }

      if (!make_avoidance_nav_goal(goal, front_distance, true, goal_kind)) {
        if (!make_goal_from_person_map(
            goal, "nav2_avoid_goal_ready", "nav2_avoid_retreat_blocked",
            "nav2_avoid_holding_distance", true)) {
          return;
        }
      }
    } else if (avoidance_stage_ != AvoidanceStage::IDLE && recent_avoidance_goal(now)) {
      reset_visual_controller();
      if (spin_active_) {
        cancel_spin("avoidance_keep_goal");
      }

      if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Nav2 action server %s is not available", nav_action_name_.c_str());
        publish_state("nav2_unavailable");
        return;
      }

      const double front_distance =
        scan_recent(now) ? front_obstacle_distance(false) : std::numeric_limits<double>::infinity();
      if (!make_avoidance_nav_goal(goal, front_distance, false, goal_kind)) {
        publish_state("nav2_avoid_occluded_waiting");
        return;
      }
    } else if ((goal_active_ || has_sent_goal_) && recent_avoidance_goal(now)) {
      reset_visual_controller();
      if (spin_active_) {
        cancel_spin("avoidance_keep_goal");
      }
      publish_state("nav2_avoid_occluded_keep_side_goal");
      return;
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
    } else if (keep_last_goal_when_occluded_ && has_memory &&
               memory_age <= target_memory_timeout_s_) {
      reset_visual_controller();
      if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Nav2 action server %s is not available", nav_action_name_.c_str());
        publish_state("nav2_unavailable");
        return;
      }

      if (should_force_spin_reacquire(now)) {
        start_reacquire_spin("cmd_zero_spin_reacquire");
        return;
      }
      if (!make_goal_from_memory(goal)) {
        return;
      }
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
      publish_stop_cmd(has_memory ? "body_lost_memory_expired" : "body_lost");
      return;
    }

    if (nav_retry_waiting_ && nav2_retry_backoff_s_ > 0.0 &&
        (now - last_failed_goal_time_).seconds() < nav2_retry_backoff_s_) {
      publish_state("nav2_retry_backoff");
      return;
    }

    if ((now - last_goal_send_time_).seconds() < goal_update_period_s_) {
      return;
    }

    if (has_sent_goal_ && !goal_changed(goal, goal_kind)) {
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

  bool start_reacquire_spin(const std::string & state)
  {
    if (!reacquire_yaw_goal_) {
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
    const double camera_forward = latest_body_.centerofmass_z * 0.001;
    const double camera_left = latest_body_.centerofmass_x * 0.001;
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
    x = camera_offset_x_m_ + c * camera_forward - s * camera_left;
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
      [this, goal_sequence](GoalHandleNavigate::SharedPtr goal_handle) {
        if (goal_sequence != goal_sequence_) {
          return;
        }

        if (!goal_handle) {
          goal_active_ = false;
          goal_handle_.reset();
          has_sent_goal_ = false;
          nav_retry_waiting_ = true;
          last_failed_goal_time_ = this->now();
          publish_state("goal_rejected");
          RCLCPP_WARN(get_logger(), "Nav2 rejected body follow goal");
          return;
        }

        goal_active_ = true;
        cancel_sent_ = false;
        goal_handle_ = goal_handle;
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

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            nav_retry_waiting_ = false;
            if (goal_kind == NavGoalKind::AVOID_GATE) {
              avoidance_stage_ = AvoidanceStage::ANCHOR;
              avoidance_stage_time_ = this->now();
              publish_state("nav2_avoid_gate_succeeded");
            } else if (goal_kind == NavGoalKind::AVOID_ANCHOR) {
              reset_avoidance_episode();
              publish_state("nav2_avoid_anchor_succeeded");
            } else {
              publish_state("goal_succeeded");
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            nav_retry_waiting_ = true;
            last_failed_goal_time_ = this->now();
            publish_state(goal_kind == NavGoalKind::NORMAL ?
              "goal_aborted" : "nav2_avoid_goal_aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            nav_retry_waiting_ = false;
            publish_state("goal_canceled");
            break;
          default:
            nav_retry_waiting_ = true;
            last_failed_goal_time_ = this->now();
            publish_state("goal_unknown_result");
            break;
        }
      };

    nav_retry_waiting_ = false;
    nav_client_->async_send_goal(goal, options);
    last_sent_goal_ = goal_pose;
    last_sent_goal_kind_ = goal_kind;
    has_sent_goal_ = true;
    last_goal_send_time_ = this->now();
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
  double obstacle_slow_distance_m_;
  double obstacle_nav_distance_m_;
  double obstacle_nav_release_distance_m_;
  double visual_min_linear_scale_;
  double target_exemption_angle_rad_;
  double target_exemption_distance_margin_m_;
  double avoidance_lateral_offset_m_;
  double avoidance_forward_margin_m_;
  double avoidance_min_forward_step_m_;
  double avoidance_max_forward_step_m_;
  double avoidance_side_angle_rad_;
  double avoidance_side_clearance_min_m_;
  double avoidance_target_side_bias_m_;
  double avoidance_side_hold_s_;
  double avoidance_occlusion_hold_s_;
  double avoidance_gate_switch_distance_m_;
  double avoidance_gate_timeout_s_;
  double avoidance_anchor_blend_alpha_;
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
  int mode_required_;
  int mode_;
  int last_avoidance_side_;
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
  bool direct_cmd_init_;
  bool has_last_clear_anchor_;
  bool has_active_avoidance_anchor_;
  bool has_avoidance_gate_goal_;
  uint64_t goal_sequence_;
  uint64_t spin_sequence_;

  rclcpp::Time last_body_time_;
  rclcpp::Time last_person_map_time_;
  rclcpp::Time last_goal_send_time_;
  rclcpp::Time last_failed_goal_time_;
  rclcpp::Time last_spin_send_time_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_cmd_motion_time_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time last_direct_cmd_time_;
  rclcpp::Time last_avoidance_side_time_;
  rclcpp::Time last_avoidance_goal_time_;
  rclcpp::Time last_clear_anchor_time_;
  rclcpp::Time avoidance_stage_time_;
  double filtered_x_angle_;
  double last_visual_error_angle_;
  double last_visual_error_distance_;
  sensor_msgs::msg::LaserScan last_scan_;
  geometry_msgs::msg::Twist last_direct_cmd_;
  geometry_msgs::msg::PoseStamped last_person_map_pose_;
  geometry_msgs::msg::PoseStamped last_clear_follow_anchor_;
  geometry_msgs::msg::PoseStamped active_avoidance_anchor_;
  geometry_msgs::msg::PoseStamped avoidance_gate_goal_;
  geometry_msgs::msg::PoseStamped last_sent_goal_;
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr visual_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyNav2Follower>());
  rclcpp::shutdown();
  return 0;
}
