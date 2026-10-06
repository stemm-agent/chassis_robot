#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class AmclNoiseScheduler : public rclcpp::Node
{
public:
  AmclNoiseScheduler()
  : Node("amcl_noise_scheduler")
  {
    amcl_node_ = declare_parameter<std::string>("amcl_node", "/amcl");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_combined");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.5);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.0);
    scan_quality_threshold_ = declare_parameter<double>("scan_quality_threshold", 0.45);
    min_factor_delta_ = declare_parameter<double>("min_factor_delta", 0.10);
    update_period_sec_ = declare_parameter<double>("update_period_sec", 2.0);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&AmclNoiseScheduler::on_odom, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AmclNoiseScheduler::on_scan, this, std::placeholders::_1));
    parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, amcl_node_);

    const auto period = std::chrono::duration<double>(std::max(update_period_sec_, 0.5));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&AmclNoiseScheduler::update_parameters, this));

    RCLCPP_INFO(
      get_logger(), "AMCL noise scheduler ready for %s; it changes parameters only after a stable profile change.",
      amcl_node_.c_str());
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    linear_speed_ = message->twist.twist.linear.x;
    angular_speed_ = message->twist.twist.angular.z;
    have_odom_ = true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    if (message->ranges.empty()) {
      return;
    }

    size_t valid_count = 0;
    for (const auto range : message->ranges) {
      if (std::isfinite(range) && range >= message->range_min && range <= message->range_max) {
        ++valid_count;
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    scan_quality_ = static_cast<double>(valid_count) / static_cast<double>(message->ranges.size());
    have_scan_ = true;
  }

  void update_parameters()
  {
    double linear_speed = 0.0;
    double angular_speed = 0.0;
    double scan_quality = 1.0;
    bool have_odom = false;
    bool have_scan = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear_speed = linear_speed_;
      angular_speed = angular_speed_;
      scan_quality = scan_quality_;
      have_odom = have_odom_;
      have_scan = have_scan_;
    }

    if (!have_odom || !parameter_client_->service_is_ready()) {
      return;
    }

    const double speed_factor = std::min(
      1.0,
      std::max(
        std::abs(linear_speed) / std::max(max_linear_speed_, 0.01),
        std::abs(angular_speed) / std::max(max_angular_speed_, 0.01)));
    const double scan_factor = have_scan ? std::max(0.0, std::min(1.0,
      (scan_quality_threshold_ - scan_quality) / std::max(scan_quality_threshold_, 0.01))) : 0.0;
    const double factor = std::max(speed_factor, scan_factor);

    if (last_factor_ >= 0.0 && std::abs(factor - last_factor_) < min_factor_delta_) {
      return;
    }

    // Low motion uses a tighter, odometry-led particle proposal.  At speed or
    // poor scan coverage, the proposal and measurement model become gradually
    // less confident; z_hit + z_max + z_rand + z_short always remains one.
    const std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("alpha1", 0.20 + 0.20 * factor),
      rclcpp::Parameter("alpha2", 0.10 + 0.10 * factor),
      rclcpp::Parameter("alpha3", 0.15 + 0.10 * factor),
      rclcpp::Parameter("alpha4", 0.25 + 0.15 * factor),
      rclcpp::Parameter("z_hit", 0.70 - 0.15 * factor),
      rclcpp::Parameter("z_rand", 0.20 + 0.15 * factor),
      rclcpp::Parameter("sigma_hit", 0.20 + 0.10 * factor),
    };
    parameter_client_->set_parameters(parameters);
    last_factor_ = factor;
    RCLCPP_INFO(
      get_logger(),
      "Applied AMCL noise factor %.2f (speed %.2f m/s, %.2f rad/s; scan quality %.2f)",
      factor, linear_speed, angular_speed, scan_quality);
  }

  std::string amcl_node_;
  std::string odom_topic_;
  std::string scan_topic_;
  double max_linear_speed_{};
  double max_angular_speed_{};
  double scan_quality_threshold_{};
  double min_factor_delta_{};
  double update_period_sec_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  bool have_odom_{false};
  bool have_scan_{false};
  double linear_speed_{0.0};
  double angular_speed_{0.0};
  double scan_quality_{1.0};
  double last_factor_{-1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclNoiseScheduler>());
  rclcpp::shutdown();
  return 0;
}
