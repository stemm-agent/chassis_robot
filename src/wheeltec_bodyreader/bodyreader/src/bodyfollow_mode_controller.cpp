#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
constexpr int8_t kInactiveMode = 1;
constexpr int8_t kFollowMode = 2;

std::string json_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += character; break;
    }
  }
  return escaped;
}
}  // namespace

// A service is point-to-point, whereas the follow pipeline has several
// independent consumers of the mode.  This node is the single service
// endpoint and publishes its accepted state through a reliable, transient
// local topic.  New/restarted consumers therefore receive the last state
// without relying on callers to repeat a command.
class BodyfollowModeController : public rclcpp::Node
{
public:
  BodyfollowModeController()
  : Node("bodyfollow_mode_controller")
  {
    service_name_ = declare_parameter<std::string>(
      "service_name", "/voice_summon/set_body_follow_enabled");
    legacy_service_name_ = declare_parameter<std::string>(
      "legacy_service_name", "/bodyfollow/set_enabled");
    status_service_name_ = declare_parameter<std::string>(
      "status_service_name", "/voice_summon/get_body_follow_status");
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/mode");

    mode_pub_ = create_publisher<std_msgs::msg::Int8>(
      mode_topic_, rclcpp::QoS(1).reliable().transient_local());
    service_ = create_service<std_srvs::srv::SetBool>(
      service_name_,
      std::bind(
        &BodyfollowModeController::set_enabled_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    if (!legacy_service_name_.empty() && legacy_service_name_ != service_name_) {
      legacy_service_ = create_service<std_srvs::srv::SetBool>(
        legacy_service_name_,
        std::bind(
          &BodyfollowModeController::set_enabled_callback, this,
          std::placeholders::_1, std::placeholders::_2));
    }
    status_service_ = create_service<std_srvs::srv::Trigger>(
      status_service_name_,
      std::bind(
        &BodyfollowModeController::get_status_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Service process restart must be safe: it always returns the system to
    // inactive instead of retaining an old follow command.
    publish_mode(kInactiveMode, "startup_safe_inactive");
    RCLCPP_INFO(
      get_logger(),
      "Mode services ready: set=%s legacy=%s status=%s state topic=%s",
      service_name_.c_str(), legacy_service_name_.c_str(),
      status_service_name_.c_str(), mode_topic_.c_str());
  }

private:
  void set_enabled_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const int8_t requested_mode = request->data ? kFollowMode : kInactiveMode;
    const bool published = publish_mode(
      requested_mode, request->data ? "service_enable" : "service_disable");
    if (published) {
      follow_enabled_ = request->data;
    }
    response->success = published;
    response->message = published
      ? (request->data ? "body follow enabled (mode=2)" : "body follow stopped (mode=1)")
      : (last_error_.empty() ? "failed to publish follow mode" : last_error_);
  }

  void get_status_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto subscriber_count = mode_pub_->get_subscription_count();
    // The controller owns the reliable transient-local mode value.  A live
    // subscriber is additionally required before an enabled follow command is
    // reported as applied to the downstream follow pipeline.
    const bool controller_applied = last_publish_succeeded_ &&
      (!follow_enabled_ || subscriber_count > 0U);
    std::ostringstream payload;
    payload << "{\"enabled\":" << (follow_enabled_ ? "true" : "false")
            << ",\"state\":\"" << (follow_enabled_ ? "enabled" : "disabled")
            << "\",\"controllerApplied\":"
            << (controller_applied ? "true" : "false")
            << ",\"pending\":false"
            << ",\"modeSubscriberCount\":" << subscriber_count
            << ",\"lastError\":\"" << json_escape(last_error_) << "\"}";
    response->success = true;
    response->message = payload.str();
  }

  bool publish_mode(int8_t mode, const char * reason)
  {
    try {
      std_msgs::msg::Int8 message;
      message.data = mode;
      mode_pub_->publish(message);
      last_publish_succeeded_ = true;
      last_error_.clear();
      RCLCPP_INFO(get_logger(), "Mode published: %d reason=%s", mode, reason);
      return true;
    } catch (const std::exception & error) {
      last_publish_succeeded_ = false;
      last_error_ = error.what();
      RCLCPP_ERROR(get_logger(), "Failed to publish follow mode: %s", error.what());
      return false;
    }
  }

  std::string service_name_;
  std::string legacy_service_name_;
  std::string status_service_name_;
  std::string mode_topic_;
  bool follow_enabled_{false};
  bool last_publish_succeeded_{false};
  std::string last_error_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr legacy_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyfollowModeController>());
  rclcpp::shutdown();
  return 0;
}
