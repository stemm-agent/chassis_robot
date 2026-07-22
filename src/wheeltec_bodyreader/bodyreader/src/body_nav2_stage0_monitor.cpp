#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

double yaw_from_quaternion(double x, double y, double z, double w)
{
  tf2::Quaternion quaternion(x, y, z, w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

double angle_difference(double lhs, double rhs)
{
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

std::string csv_text(std::string value)
{
  std::replace(value.begin(), value.end(), ',', ';');
  std::replace(value.begin(), value.end(), '\n', ' ');
  std::replace(value.begin(), value.end(), '\r', ' ');
  return value;
}

std::string local_timestamp(const char * format)
{
  const std::time_t now = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&now, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, format);
  return stream.str();
}
}  // namespace

class BodyNav2Stage0Monitor : public rclcpp::Node
{
public:
  BodyNav2Stage0Monitor()
  : Node("body_nav2_stage0_monitor"),
    output_directory_(declare_parameter<std::string>("output_directory", "/home/wheeltec")),
    sample_period_s_(declare_parameter<double>("sample_period_s", 1.0)),
    lifecycle_poll_period_s_(declare_parameter<double>("lifecycle_poll_period_s", 5.0)),
    log_period_s_(declare_parameter<double>("log_period_s", 5.0)),
    mode_(-1),
    navigate_status_(action_msgs::msg::GoalStatus::STATUS_UNKNOWN),
    compute_path_status_(action_msgs::msg::GoalStatus::STATUS_UNKNOWN),
    follow_path_status_(action_msgs::msg::GoalStatus::STATUS_UNKNOWN),
    last_sample_time_(now()),
    sample_count_(0)
  {
    sample_period_s_ = std::max(0.2, sample_period_s_);
    lifecycle_poll_period_s_ = std::max(2.0, lifecycle_poll_period_s_);
    log_period_s_ = std::max(sample_period_s_, log_period_s_);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(20);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        update_odom_stats(*message, odom_);
      });
    odom_combined_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_combined", sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        update_odom_stats(*message, odom_combined_);
      });
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", sensor_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        update_amcl_stats(*message);
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", sensor_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        update_scan_stats(*message);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_bias_compensated", sensor_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {
        update_imu_stats(*message);
      });
    mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/mode", rclcpp::QoS(10),
      [this](const std_msgs::msg::Int8::SharedPtr message) {mode_ = message->data;});
    marker_sub_ = create_subscription<std_msgs::msg::String>(
      "/body_nav2_stage0/marker", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr message) {
        pending_marker_ = csv_text(message->data);
        RCLCPP_INFO(get_logger(), "Stage0 marker: %s", pending_marker_.c_str());
      });

    navigate_status_sub_ = create_status_subscription(
      "/navigate_to_pose/_action/status", navigate_status_);
    compute_path_status_sub_ = create_status_subscription(
      "/compute_path_to_pose/_action/status", compute_path_status_);
    follow_path_status_sub_ = create_status_subscription(
      "/follow_path/_action/status", follow_path_status_);

    summary_pub_ = create_publisher<std_msgs::msg::String>(
      "/body_nav2_stage0/summary", rclcpp::QoS(1).transient_local());
    file_path_pub_ = create_publisher<std_msgs::msg::String>(
      "/body_nav2_stage0/file_path", rclcpp::QoS(1).transient_local());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    lifecycle_nodes_ = {
      "/amcl",
      "/behavior_server",
      "/bt_navigator",
      "/controller_server",
      "/global_costmap/global_costmap",
      "/local_costmap/local_costmap",
      "/map_server",
      "/planner_server",
      "/smoother_server",
      "/waypoint_follower",
    };
    for (const auto & node_name : lifecycle_nodes_) {
      lifecycle_clients_[node_name] = create_client<lifecycle_msgs::srv::GetState>(
        node_name + "/get_state");
      lifecycle_states_[node_name] = "unknown";
      lifecycle_request_pending_[node_name] = false;
      lifecycle_request_generation_[node_name] = 0;
    }

    open_output_file();
    read_cpu_ticks(previous_cpu_ticks_);

    sample_timer_ = create_wall_timer(
      std::chrono::duration<double>(sample_period_s_),
      std::bind(&BodyNav2Stage0Monitor::sample, this));
    lifecycle_timer_ = create_wall_timer(
      std::chrono::duration<double>(lifecycle_poll_period_s_),
      std::bind(&BodyNav2Stage0Monitor::poll_lifecycle_states, this));
    poll_lifecycle_states();

    publish_file_path();
    RCLCPP_INFO(
      get_logger(),
      "Stage0 monitor ready: csv=%s sample=%.2fs lifecycle=%.2fs (observation only)",
      output_path_.c_str(), sample_period_s_, lifecycle_poll_period_s_);
  }

private:
  struct OdomStats
  {
    bool seen = false;
    uint64_t interval_count = 0;
    uint64_t stamp_backtracks = 0;
    rclcpp::Time last_stamp{0, 0, RCL_SYSTEM_TIME};
    rclcpp::Time last_receipt{0, 0, RCL_SYSTEM_TIME};
    double x = kNan;
    double y = kNan;
    double yaw = kNan;
    double max_position_step_m = 0.0;
    double max_yaw_step_rad = 0.0;
  };

  struct TransformStats
  {
    bool valid = false;
    bool previous_valid = false;
    double age_ms = kNan;
    double x = kNan;
    double y = kNan;
    double yaw = kNan;
    double position_step_m = kNan;
    double yaw_step_rad = kNan;
  };

  struct AmclStats : OdomStats
  {
    double covariance_x = kNan;
    double covariance_y = kNan;
    double covariance_yaw = kNan;
  };

  struct ScanStats
  {
    bool seen = false;
    uint64_t interval_count = 0;
    uint64_t stamp_backtracks = 0;
    rclcpp::Time last_stamp{0, 0, RCL_SYSTEM_TIME};
    std::string frame_id;
    double valid_ratio = kNan;
    double min_valid_m = kNan;
    double scan_time_s = kNan;
  };

  struct ImuStats
  {
    bool seen = false;
    uint64_t interval_count = 0;
    uint64_t stamp_backtracks = 0;
    rclcpp::Time last_stamp{0, 0, RCL_SYSTEM_TIME};
    double yaw = kNan;
    double max_yaw_step_rad = 0.0;
    double angular_velocity_z = kNan;
  };

  struct CpuTicks
  {
    bool valid = false;
    uint64_t idle = 0;
    uint64_t total = 0;
  };

  using GoalStatusArray = action_msgs::msg::GoalStatusArray;

  rclcpp::Subscription<GoalStatusArray>::SharedPtr create_status_subscription(
    const std::string & topic, int8_t & destination)
  {
    int8_t * destination_ptr = &destination;
    return create_subscription<GoalStatusArray>(
      topic, rclcpp::QoS(10),
      [destination_ptr](const GoalStatusArray::SharedPtr message) {
        int8_t latest = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        int64_t latest_stamp_ns = -1;
        for (const auto & status : message->status_list) {
          const int64_t stamp_ns =
            static_cast<int64_t>(status.goal_info.stamp.sec) * 1000000000LL +
            static_cast<int64_t>(status.goal_info.stamp.nanosec);
          if (stamp_ns >= latest_stamp_ns) {
            latest_stamp_ns = stamp_ns;
            latest = status.status;
          }
        }
        *destination_ptr = latest;
      });
  }

  void update_odom_stats(const nav_msgs::msg::Odometry & message, OdomStats & stats)
  {
    const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
    const auto & position = message.pose.pose.position;
    const auto & orientation = message.pose.pose.orientation;
    const double yaw = yaw_from_quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);

    if (stats.seen) {
      if (stamp < stats.last_stamp) {
        ++stats.stamp_backtracks;
      }
      stats.max_position_step_m = std::max(
        stats.max_position_step_m,
        std::hypot(position.x - stats.x, position.y - stats.y));
      stats.max_yaw_step_rad = std::max(
        stats.max_yaw_step_rad, angle_difference(yaw, stats.yaw));
    }

    stats.seen = true;
    ++stats.interval_count;
    stats.last_stamp = stamp;
    stats.last_receipt = now();
    stats.x = position.x;
    stats.y = position.y;
    stats.yaw = yaw;
  }

  void update_amcl_stats(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message)
  {
    const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
    const auto & position = message.pose.pose.position;
    const auto & orientation = message.pose.pose.orientation;
    const double yaw = yaw_from_quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);

    if (amcl_.seen) {
      if (stamp < amcl_.last_stamp) {
        ++amcl_.stamp_backtracks;
      }
      amcl_.max_position_step_m = std::max(
        amcl_.max_position_step_m,
        std::hypot(position.x - amcl_.x, position.y - amcl_.y));
      amcl_.max_yaw_step_rad = std::max(
        amcl_.max_yaw_step_rad, angle_difference(yaw, amcl_.yaw));
    }

    amcl_.seen = true;
    ++amcl_.interval_count;
    amcl_.last_stamp = stamp;
    amcl_.last_receipt = now();
    amcl_.x = position.x;
    amcl_.y = position.y;
    amcl_.yaw = yaw;
    amcl_.covariance_x = message.pose.covariance[0];
    amcl_.covariance_y = message.pose.covariance[7];
    amcl_.covariance_yaw = message.pose.covariance[35];
  }

  void update_scan_stats(const sensor_msgs::msg::LaserScan & message)
  {
    const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
    if (scan_.seen && stamp < scan_.last_stamp) {
      ++scan_.stamp_backtracks;
    }
    size_t valid_count = 0;
    double min_valid = std::numeric_limits<double>::infinity();
    for (const float range : message.ranges) {
      if (std::isfinite(range) && range >= message.range_min && range <= message.range_max) {
        ++valid_count;
        min_valid = std::min(min_valid, static_cast<double>(range));
      }
    }
    scan_.seen = true;
    ++scan_.interval_count;
    scan_.last_stamp = stamp;
    scan_.frame_id = csv_text(message.header.frame_id);
    scan_.valid_ratio = message.ranges.empty() ?
      kNan : static_cast<double>(valid_count) / static_cast<double>(message.ranges.size());
    scan_.min_valid_m = std::isfinite(min_valid) ? min_valid : kNan;
    scan_.scan_time_s = message.scan_time;
  }

  void update_imu_stats(const sensor_msgs::msg::Imu & message)
  {
    const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
    const auto & orientation = message.orientation;
    const double yaw = yaw_from_quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);
    if (imu_.seen) {
      if (stamp < imu_.last_stamp) {
        ++imu_.stamp_backtracks;
      }
      imu_.max_yaw_step_rad = std::max(
        imu_.max_yaw_step_rad, angle_difference(yaw, imu_.yaw));
    }
    imu_.seen = true;
    ++imu_.interval_count;
    imu_.last_stamp = stamp;
    imu_.yaw = yaw;
    imu_.angular_velocity_z = message.angular_velocity.z;
  }

  void open_output_file()
  {
    output_path_ = output_directory_ + "/body_nav2_stage0_" +
      local_timestamp("%Y%m%d_%H%M%S") + ".csv";
    csv_.open(output_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error("Unable to open Stage0 CSV: " + output_path_);
    }
    csv_ <<
      "ros_time_ns,wall_time,marker,mode,cpu_busy_pct,"
      "odom_rate_hz,odom_age_ms,odom_x,odom_y,odom_yaw,odom_step_max_m,"
      "odom_yaw_step_max_rad,odom_stamp_backtracks,"
      "odom_combined_rate_hz,odom_combined_age_ms,odom_combined_x,"
      "odom_combined_y,odom_combined_yaw,odom_combined_step_max_m,"
      "odom_combined_yaw_step_max_rad,odom_combined_stamp_backtracks,"
      "amcl_rate_hz,amcl_age_ms,amcl_x,amcl_y,amcl_yaw,amcl_step_max_m,"
      "amcl_yaw_step_max_rad,amcl_cov_x,amcl_cov_y,amcl_cov_yaw,"
      "amcl_stamp_backtracks,"
      "scan_rate_hz,scan_age_ms,scan_frame,scan_valid_ratio,scan_min_valid_m,"
      "scan_time_s,scan_stamp_backtracks,"
      "imu_rate_hz,imu_age_ms,imu_yaw,imu_yaw_step_max_rad,imu_angular_z,"
      "imu_stamp_backtracks,"
      "tf_map_odom_ok,tf_map_odom_age_ms,tf_map_odom_x,tf_map_odom_y,"
      "tf_map_odom_yaw,tf_map_odom_step_m,tf_map_odom_yaw_step_rad,"
      "tf_odom_base_ok,tf_odom_base_age_ms,tf_odom_base_x,tf_odom_base_y,"
      "tf_odom_base_yaw,tf_odom_base_step_m,tf_odom_base_yaw_step_rad,"
      "tf_map_base_ok,tf_map_base_age_ms,tf_map_base_x,tf_map_base_y,"
      "tf_map_base_yaw,tf_map_base_step_m,tf_map_base_yaw_step_rad,"
      "navigate_status,compute_path_status,follow_path_status,lifecycle_states\n";
    csv_.flush();
  }

  bool read_cpu_ticks(CpuTicks & ticks) const
  {
    std::ifstream proc_stat("/proc/stat");
    std::string cpu_label;
    std::array<uint64_t, 10> values{};
    if (!(proc_stat >> cpu_label) || cpu_label != "cpu") {
      return false;
    }
    for (auto & value : values) {
      if (!(proc_stat >> value)) {
        value = 0;
      }
    }
    ticks.idle = values[3] + values[4];
    ticks.total = 0;
    for (const auto value : values) {
      ticks.total += value;
    }
    ticks.valid = true;
    return true;
  }

  double cpu_busy_percent()
  {
    CpuTicks current;
    if (!read_cpu_ticks(current) || !previous_cpu_ticks_.valid) {
      previous_cpu_ticks_ = current;
      return kNan;
    }
    const uint64_t total_delta = current.total - previous_cpu_ticks_.total;
    const uint64_t idle_delta = current.idle - previous_cpu_ticks_.idle;
    previous_cpu_ticks_ = current;
    if (total_delta == 0) {
      return kNan;
    }
    return 100.0 * static_cast<double>(total_delta - idle_delta) /
           static_cast<double>(total_delta);
  }

  TransformStats lookup_transform(
    const std::string & target, const std::string & source,
    TransformStats & previous)
  {
    TransformStats result;
    try {
      const auto transform = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;
      result.valid = true;
      result.age_ms = (
        now() - rclcpp::Time(transform.header.stamp, get_clock()->get_clock_type())
      ).seconds() * 1000.0;
      result.x = translation.x;
      result.y = translation.y;
      result.yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
      if (previous.valid) {
        result.position_step_m = std::hypot(result.x - previous.x, result.y - previous.y);
        result.yaw_step_rad = angle_difference(result.yaw, previous.yaw);
      }
    } catch (const tf2::TransformException &) {
      result.valid = false;
    }
    previous = result;
    return result;
  }

  std::string lifecycle_summary() const
  {
    std::ostringstream stream;
    bool first = true;
    for (const auto & node_name : lifecycle_nodes_) {
      if (!first) {
        stream << '|';
      }
      first = false;
      const auto state = lifecycle_states_.find(node_name);
      stream << node_name << '=' <<
        (state == lifecycle_states_.end() ? "unknown" : state->second);
    }
    return csv_text(stream.str());
  }

  void poll_lifecycle_states()
  {
    const auto poll_time = now();
    for (const auto & node_name : lifecycle_nodes_) {
      auto client = lifecycle_clients_.at(node_name);
      if (lifecycle_request_pending_[node_name]) {
        const double pending_s =
          (poll_time - lifecycle_request_started_[node_name]).seconds();
        if (pending_s > 10.0) {
          lifecycle_request_pending_[node_name] = false;
          lifecycle_states_[node_name] = "timeout";
        }
        continue;
      }
      if (!client->service_is_ready()) {
        lifecycle_states_[node_name] = "unavailable";
        continue;
      }
      lifecycle_request_pending_[node_name] = true;
      lifecycle_request_started_[node_name] = poll_time;
      lifecycle_states_[node_name] = "requesting";
      const uint64_t generation = ++lifecycle_request_generation_[node_name];
      auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      client->async_send_request(
        request,
        [this, node_name, generation](
          rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
          if (lifecycle_request_generation_[node_name] != generation) {
            return;
          }
          try {
            lifecycle_states_[node_name] = future.get()->current_state.label;
          } catch (const std::exception & error) {
            lifecycle_states_[node_name] = std::string("error:") + error.what();
          }
          lifecycle_request_pending_[node_name] = false;
        });
    }
  }

  void publish_file_path()
  {
    std_msgs::msg::String message;
    message.data = output_path_;
    file_path_pub_->publish(message);
  }

  void sample()
  {
    const auto sample_time = now();
    const double interval_s = std::max(1e-6, (sample_time - last_sample_time_).seconds());
    last_sample_time_ = sample_time;
    ++sample_count_;

    const double cpu_busy = cpu_busy_percent();
    const double odom_rate = static_cast<double>(odom_.interval_count) / interval_s;
    const double odom_combined_rate =
      static_cast<double>(odom_combined_.interval_count) / interval_s;
    const double amcl_rate = static_cast<double>(amcl_.interval_count) / interval_s;
    const double scan_rate = static_cast<double>(scan_.interval_count) / interval_s;
    const double imu_rate = static_cast<double>(imu_.interval_count) / interval_s;
    const double odom_age_ms = odom_.seen ?
      (sample_time - odom_.last_stamp).seconds() * 1000.0 : kNan;
    const double odom_combined_age_ms = odom_combined_.seen ?
      (sample_time - odom_combined_.last_stamp).seconds() * 1000.0 : kNan;
    const double amcl_age_ms = amcl_.seen ?
      (sample_time - amcl_.last_stamp).seconds() * 1000.0 : kNan;
    const double scan_age_ms = scan_.seen ?
      (sample_time - scan_.last_stamp).seconds() * 1000.0 : kNan;
    const double imu_age_ms = imu_.seen ?
      (sample_time - imu_.last_stamp).seconds() * 1000.0 : kNan;

    const auto map_odom = lookup_transform(
      "map", "odom_combined", previous_map_odom_tf_);
    const auto odom_base = lookup_transform(
      "odom_combined", "base_footprint", previous_odom_base_tf_);
    const auto map_base = lookup_transform(
      "map", "base_footprint", previous_map_base_tf_);

    const std::string marker = pending_marker_;
    pending_marker_.clear();
    const std::string lifecycle = lifecycle_summary();

    csv_ << sample_time.nanoseconds() << ',' << local_timestamp("%F %T") << ',' <<
      marker << ',' << static_cast<int>(mode_) << ',' << cpu_busy << ',' <<
      odom_rate << ',' << odom_age_ms << ',' << odom_.x << ',' << odom_.y << ',' <<
      odom_.yaw << ',' << odom_.max_position_step_m << ',' <<
      odom_.max_yaw_step_rad << ',' << odom_.stamp_backtracks << ',' <<
      odom_combined_rate << ',' << odom_combined_age_ms << ',' <<
      odom_combined_.x << ',' << odom_combined_.y << ',' << odom_combined_.yaw << ',' <<
      odom_combined_.max_position_step_m << ',' << odom_combined_.max_yaw_step_rad << ',' <<
      odom_combined_.stamp_backtracks << ',' <<
      amcl_rate << ',' << amcl_age_ms << ',' << amcl_.x << ',' << amcl_.y << ',' <<
      amcl_.yaw << ',' << amcl_.max_position_step_m << ',' <<
      amcl_.max_yaw_step_rad << ',' << amcl_.covariance_x << ',' <<
      amcl_.covariance_y << ',' << amcl_.covariance_yaw << ',' <<
      amcl_.stamp_backtracks << ',' <<
      scan_rate << ',' << scan_age_ms << ',' << scan_.frame_id << ',' <<
      scan_.valid_ratio << ',' << scan_.min_valid_m << ',' << scan_.scan_time_s << ',' <<
      scan_.stamp_backtracks << ',' <<
      imu_rate << ',' << imu_age_ms << ',' << imu_.yaw << ',' <<
      imu_.max_yaw_step_rad << ',' << imu_.angular_velocity_z << ',' <<
      imu_.stamp_backtracks << ',' <<
      static_cast<int>(map_odom.valid) << ',' << map_odom.age_ms << ',' <<
      map_odom.x << ',' << map_odom.y << ',' << map_odom.yaw << ',' <<
      map_odom.position_step_m << ',' << map_odom.yaw_step_rad << ',' <<
      static_cast<int>(odom_base.valid) << ',' << odom_base.age_ms << ',' <<
      odom_base.x << ',' << odom_base.y << ',' << odom_base.yaw << ',' <<
      odom_base.position_step_m << ',' << odom_base.yaw_step_rad << ',' <<
      static_cast<int>(map_base.valid) << ',' << map_base.age_ms << ',' <<
      map_base.x << ',' << map_base.y << ',' << map_base.yaw << ',' <<
      map_base.position_step_m << ',' << map_base.yaw_step_rad << ',' <<
      static_cast<int>(navigate_status_) << ',' <<
      static_cast<int>(compute_path_status_) << ',' <<
      static_cast<int>(follow_path_status_) << ',' << lifecycle << '\n';
    csv_.flush();

    if (amcl_.max_position_step_m > 0.5 || amcl_.max_yaw_step_rad > 0.5 ||
      (map_odom.valid &&
      (map_odom.position_step_m > 0.5 || map_odom.yaw_step_rad > 0.5)))
    {
      RCLCPP_WARN(
        get_logger(),
        "Stage0 localization jump: amcl_step=%.3fm/%.3frad cov=%.3f/%.3f/%.3f "
        "map_odom_step=%.3fm/%.3frad ages_ms(amcl=%.1f scan=%.1f combined=%.1f)",
        amcl_.max_position_step_m, amcl_.max_yaw_step_rad,
        amcl_.covariance_x, amcl_.covariance_y, amcl_.covariance_yaw,
        map_odom.position_step_m, map_odom.yaw_step_rad,
        amcl_age_ms, scan_age_ms, odom_combined_age_ms);
    }

    std_msgs::msg::String summary;
    std::ostringstream text;
    text << std::fixed << std::setprecision(1) <<
      "mode=" << static_cast<int>(mode_) <<
      " cpu=" << cpu_busy << "%" <<
      " odom=" << odom_rate << "Hz/" << odom_age_ms << "ms" <<
      " combined=" << odom_combined_rate << "Hz/" << odom_combined_age_ms << "ms" <<
      " tf_age_ms(map_odom=" << map_odom.age_ms <<
      " odom_base=" << odom_base.age_ms <<
      " map_base=" << map_base.age_ms << ")" <<
      " actions=" << static_cast<int>(navigate_status_) << '/' <<
      static_cast<int>(compute_path_status_) << '/' <<
      static_cast<int>(follow_path_status_);
    summary.data = text.str();
    summary_pub_->publish(summary);
    publish_file_path();

    const uint64_t log_every = static_cast<uint64_t>(
      std::max(1.0, std::round(log_period_s_ / sample_period_s_)));
    if ((sample_count_ % log_every) == 0) {
      RCLCPP_INFO(get_logger(), "%s", summary.data.c_str());
    }

    odom_.interval_count = 0;
    odom_.stamp_backtracks = 0;
    odom_.max_position_step_m = 0.0;
    odom_.max_yaw_step_rad = 0.0;
    odom_combined_.interval_count = 0;
    odom_combined_.stamp_backtracks = 0;
    odom_combined_.max_position_step_m = 0.0;
    odom_combined_.max_yaw_step_rad = 0.0;
    amcl_.interval_count = 0;
    amcl_.stamp_backtracks = 0;
    amcl_.max_position_step_m = 0.0;
    amcl_.max_yaw_step_rad = 0.0;
    scan_.interval_count = 0;
    scan_.stamp_backtracks = 0;
    imu_.interval_count = 0;
    imu_.stamp_backtracks = 0;
    imu_.max_yaw_step_rad = 0.0;
  }

  std::string output_directory_;
  double sample_period_s_;
  double lifecycle_poll_period_s_;
  double log_period_s_;
  int8_t mode_;
  int8_t navigate_status_;
  int8_t compute_path_status_;
  int8_t follow_path_status_;
  rclcpp::Time last_sample_time_;
  uint64_t sample_count_;
  std::string pending_marker_;
  std::string output_path_;
  std::ofstream csv_;

  OdomStats odom_;
  OdomStats odom_combined_;
  AmclStats amcl_;
  ScanStats scan_;
  ImuStats imu_;
  CpuTicks previous_cpu_ticks_;
  TransformStats previous_map_odom_tf_;
  TransformStats previous_odom_base_tf_;
  TransformStats previous_map_base_tf_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::string> lifecycle_nodes_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>
    lifecycle_clients_;
  std::map<std::string, std::string> lifecycle_states_;
  std::map<std::string, bool> lifecycle_request_pending_;
  std::map<std::string, rclcpp::Time> lifecycle_request_started_;
  std::map<std::string, uint64_t> lifecycle_request_generation_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_combined_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    amcl_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr marker_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr navigate_status_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr compute_path_status_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr follow_path_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr file_path_pub_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr lifecycle_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyNav2Stage0Monitor>());
  rclcpp::shutdown();
  return 0;
}
