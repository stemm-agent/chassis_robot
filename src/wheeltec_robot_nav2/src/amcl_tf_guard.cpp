#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
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
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.5);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.0);
    max_linear_acceleration_ = declare_parameter<double>("max_linear_acceleration", 1.5);
    max_angular_acceleration_ = declare_parameter<double>("max_angular_acceleration", 1.5);
    linear_slack_ = declare_parameter<double>("linear_slack", 0.15);
    angular_slack_ = declare_parameter<double>("angular_slack", 0.15);
    max_odom_correction_ = declare_parameter<double>("max_odom_correction", 0.30);
    max_odom_yaw_correction_ = declare_parameter<double>("max_odom_yaw_correction", 0.35);
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(20),
      std::bind(&AmclTfGuard::on_amcl_pose, this, std::placeholders::_1));
    initialpose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(10),
      std::bind(&AmclTfGuard::on_initialpose, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&AmclTfGuard::publish_last_transform, this));

    RCLCPP_INFO(
      get_logger(), "Guarding %s -> %s from %s (v=%.2f m/s, w=%.2f rad/s)",
      global_frame_.c_str(), odom_frame_.c_str(), pose_topic_.c_str(),
      max_linear_speed_, max_angular_speed_);
  }

private:
  void on_initialpose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_transform_ = false;
    have_last_pose_ = false;
    RCLCPP_INFO(get_logger(), "Initial pose received; next AMCL pose will seed the guarded transform.");
  }

  void on_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    const rclcpp::Time stamp(message->header.stamp);
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
      odom_to_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN(get_logger(), "AMCL pose ignored: no %s -> %s TF at its stamp: %s",
        odom_frame_.c_str(), base_frame_.c_str(), exception.what());
      return;
    }

    const tf2::Transform candidate_map_base = transform_from_pose(message->pose.pose);
    const tf2::Transform current_odom_base = transform_from_msg(odom_to_base.transform);
    const tf2::Transform candidate_map_odom = candidate_map_base * current_odom_base.inverse();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_last_pose_) {
      accept(candidate_map_odom, candidate_map_base, stamp);
      RCLCPP_INFO(get_logger(), "Accepted initial AMCL pose for guarded map transform.");
      return;
    }

    const double dt = std::max(0.0, (stamp - last_pose_stamp_).seconds());
    const tf2::Vector3 displacement = candidate_map_base.getOrigin() - last_map_base_.getOrigin();
    const double translation = std::hypot(displacement.x(), displacement.y());
    const double yaw_delta = std::abs(normalize_angle(yaw_of(candidate_map_base) - yaw_of(last_map_base_)));
    const double linear_limit =
      max_linear_speed_ * dt + 0.5 * max_linear_acceleration_ * dt * dt + linear_slack_;
    const double angular_limit =
      max_angular_speed_ * dt + 0.5 * max_angular_acceleration_ * dt * dt + angular_slack_;

    const tf2::Transform predicted_map_base = last_map_odom_ * current_odom_base;
    const tf2::Vector3 correction = candidate_map_base.getOrigin() - predicted_map_base.getOrigin();
    const double correction_translation = std::hypot(correction.x(), correction.y());
    const double correction_yaw = std::abs(
      normalize_angle(yaw_of(candidate_map_base) - yaw_of(predicted_map_base)));

    if (translation > linear_limit || yaw_delta > angular_limit ||
      correction_translation > max_odom_correction_ || correction_yaw > max_odom_yaw_correction_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Rejected AMCL jump: map motion %.3f m / %.3f rad (limits %.3f / %.3f), "
        "odom correction %.3f m / %.3f rad (limits %.3f / %.3f)",
        translation, yaw_delta, linear_limit, angular_limit,
        correction_translation, correction_yaw, max_odom_correction_, max_odom_yaw_correction_);
      return;
    }

    accept(candidate_map_odom, candidate_map_base, stamp);
  }

  void accept(
    const tf2::Transform & map_to_odom,
    const tf2::Transform & map_to_base,
    const rclcpp::Time & stamp)
  {
    last_map_odom_ = map_to_odom;
    last_map_base_ = map_to_base;
    last_pose_stamp_ = stamp;
    have_transform_ = true;
    have_last_pose_ = true;
  }

  void publish_last_transform()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_transform_) {
      return;
    }

    geometry_msgs::msg::TransformStamped guarded_transform;
    guarded_transform.header.stamp = now() + rclcpp::Duration::from_seconds(transform_tolerance_);
    guarded_transform.header.frame_id = global_frame_;
    guarded_transform.child_frame_id = odom_frame_;
    guarded_transform.transform.translation.x = last_map_odom_.getOrigin().x();
    guarded_transform.transform.translation.y = last_map_odom_.getOrigin().y();
    guarded_transform.transform.translation.z = last_map_odom_.getOrigin().z();
    guarded_transform.transform.rotation = tf2::toMsg(last_map_odom_.getRotation());
    tf_broadcaster_->sendTransform(guarded_transform);
  }

  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string pose_topic_;
  std::string initialpose_topic_;
  double max_linear_speed_{};
  double max_angular_speed_{};
  double max_linear_acceleration_{};
  double max_angular_acceleration_{};
  double linear_slack_{};
  double angular_slack_{};
  double max_odom_correction_{};
  double max_odom_yaw_correction_{};
  double transform_tolerance_{};
  double publish_rate_hz_{};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex mutex_;
  bool have_transform_{false};
  bool have_last_pose_{false};
  tf2::Transform last_map_odom_;
  tf2::Transform last_map_base_;
  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclTfGuard>());
  rclcpp::shutdown();
  return 0;
}
