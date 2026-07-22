#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <string>

class CollisionGuard : public rclcpp::Node
{
public:
  CollisionGuard()
  : Node("collision_guard"),
    cmd_received_(false),
    imu_received_(false),
    odom_received_(false),
    odom_filter_ready_(false),
    collision_latched_(false),
    reverse_active_(false),
    reverse_motion_started_(false),
    stall_candidate_active_(false),
    reverse_start_time_(this->now()),
    stall_candidate_time_(this->now()),
    release_candidate_time_(this->now()),
    last_imu_time_(this->now()),
    last_odom_time_(this->now()),
    last_impact_time_(this->now()),
    accel_x_(0.0),
    planar_accel_(0.0),
    accel_jerk_(0.0),
    yaw_rate_(0.0),
    odom_linear_x_(0.0),
    odom_linear_accel_(0.0),
    odom_yaw_(0.0),
    odom_yaw_rate_(0.0),
    odom_yaw_accel_(0.0),
    wheel_push_counter_(0)
  {
    declare_parameter("reverse_speed_threshold", -0.05);
    declare_parameter("reverse_confirm_time", 0.20);
    declare_parameter("soft_planar_accel_threshold", 1.2);
    declare_parameter("hard_planar_accel_threshold", 2.5);
    declare_parameter("soft_jerk_threshold", 8.0);
    declare_parameter("hard_jerk_threshold", 15.0);
    declare_parameter("forward_decel_threshold", 1.2);
    declare_parameter("yaw_rate_threshold", 0.8);
    declare_parameter("odom_speed_threshold", 0.03);
    declare_parameter("reverse_stall_confirm_time", 0.50);
    declare_parameter("reverse_motion_ratio_threshold", 0.35);
    declare_parameter("impact_memory_time", 0.35);
    declare_parameter("odom_reverse_speed_threshold", 0.04);
    declare_parameter("odom_linear_accel_quiet_threshold", 0.20);
    declare_parameter("odom_yaw_rate_quiet_threshold", 0.12);
    declare_parameter("odom_yaw_accel_quiet_threshold", 0.60);
    declare_parameter("wheel_push_confirm_cycles", 2);
    declare_parameter("release_hold_time", 1.0);

    get_parameter("reverse_speed_threshold", reverse_speed_threshold_);
    get_parameter("reverse_confirm_time", reverse_confirm_time_);
    get_parameter("soft_planar_accel_threshold", soft_planar_accel_threshold_);
    get_parameter("hard_planar_accel_threshold", hard_planar_accel_threshold_);
    get_parameter("soft_jerk_threshold", soft_jerk_threshold_);
    get_parameter("hard_jerk_threshold", hard_jerk_threshold_);
    get_parameter("forward_decel_threshold", forward_decel_threshold_);
    get_parameter("yaw_rate_threshold", yaw_rate_threshold_);
    get_parameter("odom_speed_threshold", odom_speed_threshold_);
    get_parameter("reverse_stall_confirm_time", reverse_stall_confirm_time_);
    get_parameter("reverse_motion_ratio_threshold", reverse_motion_ratio_threshold_);
    get_parameter("impact_memory_time", impact_memory_time_);
    get_parameter("odom_reverse_speed_threshold", odom_reverse_speed_threshold_);
    get_parameter("odom_linear_accel_quiet_threshold", odom_linear_accel_quiet_threshold_);
    get_parameter("odom_yaw_rate_quiet_threshold", odom_yaw_rate_quiet_threshold_);
    get_parameter("odom_yaw_accel_quiet_threshold", odom_yaw_accel_quiet_threshold_);
    get_parameter("wheel_push_confirm_cycles", wheel_push_confirm_cycles_);
    get_parameter("release_hold_time", release_hold_time_);

    cmd_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw", 10,
      std::bind(&CollisionGuard::cmd_raw_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", rclcpp::SensorDataQoS(),
      std::bind(&CollisionGuard::imu_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&CollisionGuard::odom_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    collision_state_pub_ = create_publisher<std_msgs::msg::String>("/collision_state", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CollisionGuard::control_loop, this));

    publish_collision_state("normal");

    RCLCPP_INFO(
      get_logger(),
      "CollisionGuard ready: reverse<%.2f m/s, stall_confirm=%.2fs, odom_stop<%.2f m/s",
      reverse_speed_threshold_, reverse_stall_confirm_time_, odom_speed_threshold_);
  }

private:
  static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

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

  void cmd_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;
    cmd_received_ = true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto now = this->now();
    const double dt = std::max((now - last_imu_time_).seconds(), 1e-3);

    accel_x_ = msg->linear_acceleration.x;
    const double accel_y = msg->linear_acceleration.y;
    planar_accel_ = std::sqrt(accel_x_ * accel_x_ + accel_y * accel_y);
    accel_jerk_ = std::fabs((accel_x_ - last_accel_x_) / dt);
    yaw_rate_ = msg->angular_velocity.z;

    last_accel_x_ = accel_x_;
    last_imu_time_ = now;
    imu_received_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto now = this->now();
    const double new_odom_linear_x = msg->twist.twist.linear.x;
    const double new_odom_yaw = quaternion_to_yaw(msg->pose.pose.orientation);

    if (!odom_filter_ready_) {
      odom_linear_x_ = new_odom_linear_x;
      odom_yaw_ = new_odom_yaw;
      odom_yaw_rate_ = 0.0;
      odom_linear_accel_ = 0.0;
      odom_yaw_accel_ = 0.0;
      last_odom_time_ = now;
      odom_filter_ready_ = true;
      odom_received_ = true;
      return;
    }

    const double dt = std::max((now - last_odom_time_).seconds(), 1e-3);

    odom_linear_accel_ = (new_odom_linear_x - odom_linear_x_) / dt;

    const double yaw_delta = normalize_angle(new_odom_yaw - odom_yaw_);
    const double new_odom_yaw_rate = yaw_delta / dt;
    odom_yaw_accel_ = (new_odom_yaw_rate - odom_yaw_rate_) / dt;

    odom_linear_x_ = new_odom_linear_x;
    odom_yaw_ = new_odom_yaw;
    odom_yaw_rate_ = new_odom_yaw_rate;
    last_odom_time_ = now;
    odom_received_ = true;
  }

  void control_loop()
  {
    geometry_msgs::msg::Twist safe_cmd;
    const auto now = this->now();

    if (!cmd_received_) {
      cmd_vel_pub_->publish(safe_cmd);
      return;
    }

    const bool reversing = latest_cmd_.linear.x < reverse_speed_threshold_;
    if (reversing && !reverse_active_) {
      reverse_start_time_ = now;
    }
    if (!reversing) {
      reverse_motion_started_ = false;
      stall_candidate_active_ = false;
      wheel_push_counter_ = 0;
    }
    reverse_active_ = reversing;

    if (collision_latched_) {
      if (!reversing) {
        if ((now - release_candidate_time_).seconds() >= release_hold_time_) {
          collision_latched_ = false;
          publish_collision_state("normal");
          RCLCPP_INFO(get_logger(), "Rear collision latch released");
        }
      } else {
        release_candidate_time_ = now;
      }

      cmd_vel_pub_->publish(safe_cmd);
      return;
    }

    safe_cmd = latest_cmd_;

    const bool reverse_confirmed = reverse_active_ &&
      (now - reverse_start_time_).seconds() >= reverse_confirm_time_;

    if (reverse_confirmed && odom_received_) {
      if (odom_linear_x_ <= -odom_reverse_speed_threshold_) {
        reverse_motion_started_ = true;
      }

      const bool hard_impact = planar_accel_ >= hard_planar_accel_threshold_ ||
        accel_jerk_ >= hard_jerk_threshold_;

      const bool helper_forward_decel = accel_x_ >= forward_decel_threshold_;
      const bool helper_yaw_spike = std::fabs(yaw_rate_) >= yaw_rate_threshold_;
      const double required_reverse_speed = std::max(
        odom_speed_threshold_,
        std::fabs(latest_cmd_.linear.x) * reverse_motion_ratio_threshold_);
      const bool helper_odom_stall = odom_linear_x_ > -odom_speed_threshold_;
      const bool helper_odom_reverse_lost = odom_linear_x_ > -required_reverse_speed;

      const bool soft_impact = planar_accel_ >= soft_planar_accel_threshold_ ||
        accel_jerk_ >= soft_jerk_threshold_;

      if (soft_impact || hard_impact) {
        last_impact_time_ = now;
      }

      const bool impact_recent = (now - last_impact_time_).seconds() <= impact_memory_time_;
      const bool odom_motion_quiet =
        std::fabs(odom_linear_accel_) <= odom_linear_accel_quiet_threshold_ &&
        std::fabs(odom_yaw_rate_) <= odom_yaw_rate_quiet_threshold_ &&
        std::fabs(odom_yaw_accel_) <= odom_yaw_accel_quiet_threshold_;

      if (impact_recent && helper_odom_stall && odom_motion_quiet) {
        ++wheel_push_counter_;
      } else {
        wheel_push_counter_ = 0;
      }

      const bool helper_wheel_push = wheel_push_counter_ >= wheel_push_confirm_cycles_;
      const bool reverse_stall_armed = reverse_motion_started_ || impact_recent || helper_wheel_push;
      const bool reverse_stalled = reverse_stall_armed &&
        (helper_odom_stall || helper_odom_reverse_lost);
      const bool reverse_contact_hint = impact_recent || helper_wheel_push || odom_motion_quiet;

      if (soft_impact && !hard_impact) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Rear stall candidate: cmd=%.3f odom=%.3f req=%.3f odom_acc=%.3f odom_yaw_rate=%.3f imu_acc=%.3f planar=%.3f jerk=%.3f push_count=%d",
          latest_cmd_.linear.x,
          odom_linear_x_,
          -required_reverse_speed,
          odom_linear_accel_,
          odom_yaw_rate_,
          accel_x_,
          planar_accel_,
          accel_jerk_,
          wheel_push_counter_);
      }

      if (reverse_stalled && reverse_contact_hint) {
        if (!stall_candidate_active_) {
          stall_candidate_active_ = true;
          stall_candidate_time_ = now;
        }
      } else {
        stall_candidate_active_ = false;
      }

      const bool stall_confirmed = stall_candidate_active_ &&
        (now - stall_candidate_time_).seconds() >= reverse_stall_confirm_time_;

      if (stall_confirmed &&
        (hard_impact || helper_wheel_push ||
        (soft_impact && (helper_forward_decel || helper_yaw_spike || helper_odom_reverse_lost)) ||
        odom_motion_quiet)) {
        collision_latched_ = true;
        const double stall_duration = (now - stall_candidate_time_).seconds();
        stall_candidate_active_ = false;
        wheel_push_counter_ = 0;
        release_candidate_time_ = now;
        publish_collision_state("rear_collision_stop");
        RCLCPP_WARN(
          get_logger(),
          "Rear collision stop: cmd=%.3f odom=%.3f req=%.3f stall=%.2fs odom_acc=%.3f odom_yaw_rate=%.3f odom_yaw_acc=%.3f accel_x=%.3f planar=%.3f jerk=%.3f imu_yaw=%.3f",
          latest_cmd_.linear.x,
          odom_linear_x_,
          -required_reverse_speed,
          stall_duration,
          odom_linear_accel_,
          odom_yaw_rate_,
          odom_yaw_accel_,
          accel_x_,
          planar_accel_,
          accel_jerk_,
          yaw_rate_);
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        return;
      }
    } else {
      stall_candidate_active_ = false;
      wheel_push_counter_ = 0;
    }

    cmd_vel_pub_->publish(safe_cmd);
  }

  void publish_collision_state(const std::string &state)
  {
    if (state == last_collision_state_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = state;
    collision_state_pub_->publish(msg);
    last_collision_state_ = state;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr collision_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist latest_cmd_;

  bool cmd_received_;
  bool imu_received_;
  bool odom_received_;
  bool odom_filter_ready_;
  bool collision_latched_;
  bool reverse_active_;
  bool reverse_motion_started_;
  bool stall_candidate_active_;

  rclcpp::Time reverse_start_time_;
  rclcpp::Time stall_candidate_time_;
  rclcpp::Time release_candidate_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_impact_time_;

  double reverse_speed_threshold_;
  double reverse_confirm_time_;
  double soft_planar_accel_threshold_;
  double hard_planar_accel_threshold_;
  double soft_jerk_threshold_;
  double hard_jerk_threshold_;
  double forward_decel_threshold_;
  double yaw_rate_threshold_;
  double odom_speed_threshold_;
  double reverse_stall_confirm_time_;
  double reverse_motion_ratio_threshold_;
  double impact_memory_time_;
  double odom_reverse_speed_threshold_;
  double odom_linear_accel_quiet_threshold_;
  double odom_yaw_rate_quiet_threshold_;
  double odom_yaw_accel_quiet_threshold_;
  int wheel_push_confirm_cycles_;
  double release_hold_time_;

  double last_accel_x_ {0.0};
  double accel_x_;
  double planar_accel_;
  double accel_jerk_;
  double yaw_rate_;
  double odom_linear_x_;
  double odom_linear_accel_;
  double odom_yaw_;
  double odom_yaw_rate_;
  double odom_yaw_accel_;
  int wheel_push_counter_;

  std::string last_collision_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionGuard>());
  rclcpp::shutdown();
  return 0;
}