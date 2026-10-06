#include <atomic>
#include <chrono>
#include <csignal>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
std::atomic_bool g_shutdown_requested(false);

void handle_signal(int)
{
  g_shutdown_requested.store(true);
}
}  // namespace

class BodyNav2ProfileGuard : public rclcpp::Node
{
public:
  BodyNav2ProfileGuard()
  : Node("body_nav2_profile_guard"),
    restore_done_(false)
  {
    rclcpp::NodeOptions client_options;
    client_options.use_global_arguments(false);
    parameter_client_node_ =
      std::make_shared<rclcpp::Node>(
        "body_nav2_profile_guard_param_client", client_options);
    parameter_executor_.add_node(parameter_client_node_);

    enabled_ = declare_parameter<bool>("enabled", true);
    restore_on_shutdown_ = declare_parameter<bool>("restore_on_shutdown", true);
    mode_control_enabled_ = declare_parameter<bool>("mode_control_enabled", true);
    mode_required_ = declare_parameter<int>("mode_required", 2);
    controller_profile_enabled_ = declare_parameter<bool>("controller_profile_enabled", true);
    behavior_profile_enabled_ = declare_parameter<bool>("behavior_profile_enabled", true);
    amcl_profile_enabled_ = declare_parameter<bool>("amcl_profile_enabled", true);
    controller_server_name_ =
      declare_parameter<std::string>("controller_server_name", "/controller_server");
    behavior_server_name_ =
      declare_parameter<std::string>("behavior_server_name", "/behavior_server");
    amcl_node_name_ = declare_parameter<std::string>("amcl_node_name", "/amcl");
    wait_timeout_s_ = declare_parameter<double>("wait_timeout_s", 6.0);

    controller_frequency_ = declare_parameter<double>("controller_frequency", 15.0);
    controller_vx_min_ = declare_parameter<double>("controller_vx_min", 0.0);
    controller_vx_max_ = declare_parameter<double>("controller_vx_max", 0.45);
    controller_vx_std_ = declare_parameter<double>("controller_vx_std", 0.16);
    controller_wz_max_ = declare_parameter<double>("controller_wz_max", 1.00);
    controller_wz_std_ = declare_parameter<double>("controller_wz_std", 0.25);
    controller_retry_attempt_limit_ =
      declare_parameter<int>("controller_retry_attempt_limit", 2);
    controller_failure_tolerance_ =
      declare_parameter<double>("controller_failure_tolerance", 4.0);
    controller_yaw_goal_tolerance_ =
      declare_parameter<double>("controller_yaw_goal_tolerance", 0.20);
    prefer_forward_enabled_ =
      declare_parameter<bool>("prefer_forward_enabled", true);
    path_angle_forward_preference_ =
      declare_parameter<bool>("path_angle_forward_preference", true);
    progress_required_movement_radius_ =
      declare_parameter<double>("progress_required_movement_radius", 0.05);
    progress_movement_time_allowance_ =
      declare_parameter<double>("progress_movement_time_allowance", 60.0);

    behavior_max_rotational_vel_ =
      declare_parameter<double>("behavior_max_rotational_vel", 0.45);
    behavior_min_rotational_vel_ =
      declare_parameter<double>("behavior_min_rotational_vel", 0.03);
    behavior_rotational_acc_lim_ =
      declare_parameter<double>("behavior_rotational_acc_lim", 0.22);
    behavior_simulate_ahead_time_ =
      declare_parameter<double>("behavior_simulate_ahead_time", 1.20);

    amcl_alpha1_ = declare_parameter<double>("amcl_alpha1", 0.10);
    amcl_alpha2_ = declare_parameter<double>("amcl_alpha2", 0.10);
    amcl_alpha3_ = declare_parameter<double>("amcl_alpha3", 0.10);
    amcl_alpha4_ = declare_parameter<double>("amcl_alpha4", 0.10);
    amcl_update_min_d_ = declare_parameter<double>("amcl_update_min_d", 0.15);
    amcl_update_min_a_ = declare_parameter<double>("amcl_update_min_a", 0.10);
    amcl_laser_model_type_ =
      declare_parameter<std::string>("amcl_laser_model_type", "likelihood_field_prob");
    amcl_do_beamskip_ = declare_parameter<bool>("amcl_do_beamskip", true);

    state_pub_ = create_publisher<std_msgs::msg::String>("/body_nav2_profile_state", 10);
    mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/mode", rclcpp::QoS(10),
      std::bind(&BodyNav2ProfileGuard::mode_callback, this, std::placeholders::_1));
    signal_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BodyNav2ProfileGuard::signal_check_loop, this));
  }

  void start()
  {
    if (mode_control_enabled_) {
      publish_state("profile_waiting_for_follow_mode");
      RCLCPP_INFO(
        get_logger(), "Waiting for /mode=%d before applying the Body Nav2 profile",
        mode_required_);
      return;
    }
    apply_profile();
  }

  void apply_profile()
  {
    if (g_shutdown_requested.load()) {
      return;
    }

    if (!enabled_) {
      publish_state("profile_disabled");
      RCLCPP_INFO(get_logger(), "Body Nav2 profile guard disabled");
      return;
    }

    publish_state("profile_applying");
    saved_parameters_.clear();
    restore_done_ = false;

    std::map<std::string, std::vector<rclcpp::Parameter>> targets_by_node;
    if (controller_profile_enabled_) {
      targets_by_node[controller_server_name_] = controller_targets();
    }
    if (behavior_profile_enabled_) {
      targets_by_node[behavior_server_name_] = behavior_targets();
    }
    if (amcl_profile_enabled_) {
      targets_by_node[amcl_node_name_] = amcl_targets();
    }

    bool all_ok = true;
    for (const auto & item : targets_by_node) {
      if (g_shutdown_requested.load()) {
        all_ok = false;
        break;
      }
      const bool ok = snapshot_and_apply(item.first, item.second);
      all_ok = all_ok && ok;
    }

    publish_state(all_ok ? "profile_applied" : "profile_partial");
    RCLCPP_INFO(
      get_logger(),
      "Body Nav2 profile %s; saved %zu node profile(s)",
      all_ok ? "applied" : "partially applied", saved_parameters_.size());
  }

  void restore_profile(const std::string & reason)
  {
    if (!restore_on_shutdown_ || restore_done_ || saved_parameters_.empty()) {
      restore_done_ = true;
      return;
    }

    publish_state("profile_restoring");
    bool all_ok = true;
    for (const auto & item : saved_parameters_) {
      const bool ok = set_parameters(item.first, item.second, true);
      all_ok = all_ok && ok;
    }

    restore_done_ = true;
    publish_state(all_ok ? "profile_restored" : "profile_restore_partial");
    RCLCPP_INFO(
      get_logger(), "Body Nav2 profile restore %s after %s",
      all_ok ? "completed" : "partially failed", reason.c_str());
  }

private:
  std::vector<rclcpp::Parameter> controller_targets() const
  {
    return {
      rclcpp::Parameter("controller_frequency", controller_frequency_),
      rclcpp::Parameter("FollowPath.vx_min", controller_vx_min_),
      rclcpp::Parameter("FollowPath.vx_max", controller_vx_max_),
      rclcpp::Parameter("FollowPath.vx_std", controller_vx_std_),
      rclcpp::Parameter("FollowPath.wz_max", controller_wz_max_),
      rclcpp::Parameter("FollowPath.wz_std", controller_wz_std_),
      rclcpp::Parameter("FollowPath.retry_attempt_limit", controller_retry_attempt_limit_),
      rclcpp::Parameter("failure_tolerance", controller_failure_tolerance_),
      rclcpp::Parameter("goal_checker.yaw_goal_tolerance", controller_yaw_goal_tolerance_),
      rclcpp::Parameter("FollowPath.PreferForwardCritic.enabled", prefer_forward_enabled_),
      rclcpp::Parameter("FollowPath.PathAngleCritic.forward_preference",
        path_angle_forward_preference_),
      rclcpp::Parameter("progress_checker.required_movement_radius",
        progress_required_movement_radius_),
      rclcpp::Parameter("progress_checker.movement_time_allowance",
        progress_movement_time_allowance_)
    };
  }

  std::vector<rclcpp::Parameter> behavior_targets() const
  {
    return {
      rclcpp::Parameter("max_rotational_vel", behavior_max_rotational_vel_),
      rclcpp::Parameter("min_rotational_vel", behavior_min_rotational_vel_),
      rclcpp::Parameter("rotational_acc_lim", behavior_rotational_acc_lim_),
      rclcpp::Parameter("simulate_ahead_time", behavior_simulate_ahead_time_)
    };
  }

  std::vector<rclcpp::Parameter> amcl_targets() const
  {
    return {
      rclcpp::Parameter("alpha1", amcl_alpha1_),
      rclcpp::Parameter("alpha2", amcl_alpha2_),
      rclcpp::Parameter("alpha3", amcl_alpha3_),
      rclcpp::Parameter("alpha4", amcl_alpha4_),
      rclcpp::Parameter("update_min_d", amcl_update_min_d_),
      rclcpp::Parameter("update_min_a", amcl_update_min_a_),
      rclcpp::Parameter("laser_model_type", amcl_laser_model_type_),
      rclcpp::Parameter("do_beamskip", amcl_do_beamskip_)
    };
  }

  bool snapshot_and_apply(
    const std::string & node_name,
    const std::vector<rclcpp::Parameter> & targets)
  {
    auto client = make_client(node_name, false);
    if (!client) {
      return false;
    }

    std::vector<std::string> names;
    names.reserve(targets.size());
    for (const auto & parameter : targets) {
      names.push_back(parameter.get_name());
    }

    std::vector<rclcpp::Parameter> current;
    try {
      auto future = client->get_parameters(names);
      if (!wait_for_future(future, false)) {
        return false;
      }
      current = future.get();
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(), "Failed to read current parameters from %s: %s",
        node_name.c_str(), ex.what());
      return false;
    }

    std::vector<rclcpp::Parameter> saved;
    std::vector<rclcpp::Parameter> filtered_targets;
    for (std::size_t i = 0; i < current.size() && i < targets.size(); ++i) {
      if (current[i].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        RCLCPP_WARN(
          get_logger(), "Skipping undeclared parameter %s on %s",
          targets[i].get_name().c_str(), node_name.c_str());
        continue;
      }

      saved.push_back(current[i]);
      filtered_targets.push_back(targets[i]);
      RCLCPP_INFO(
        get_logger(), "Profile %s.%s: %s -> %s",
        node_name.c_str(), targets[i].get_name().c_str(),
        current[i].value_to_string().c_str(), targets[i].value_to_string().c_str());
    }

    if (filtered_targets.empty()) {
      RCLCPP_WARN(get_logger(), "No parameters can be profiled on %s", node_name.c_str());
      return false;
    }

    if (!set_parameters(node_name, filtered_targets, false)) {
      return false;
    }

    saved_parameters_[node_name] = saved;
    return saved.size() == targets.size();
  }

  bool set_parameters(
    const std::string & node_name,
    const std::vector<rclcpp::Parameter> & parameters,
    bool restoring)
  {
    auto client = make_client(node_name, restoring);
    if (!client) {
      return false;
    }

    rcl_interfaces::msg::SetParametersResult result;
    try {
      auto future = client->set_parameters_atomically(parameters);
      if (!wait_for_future(future, restoring)) {
        return false;
      }
      result = future.get();
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(), "Failed to atomically set parameters on %s: %s",
        node_name.c_str(), ex.what());
      return false;
    }

    if (!result.successful) {
      RCLCPP_WARN(
        get_logger(), "%s parameter batch for %s rejected: %s",
        restoring ? "Restore" : "Profile", node_name.c_str(), result.reason.c_str());
    }
    return result.successful;
  }

  void mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    const bool was_following = mode_ == mode_required_;
    mode_ = msg->data;
    const bool is_following = mode_ == mode_required_;
    if (!mode_control_enabled_ || was_following == is_following) {
      return;
    }
    if (is_following) {
      apply_profile();
    } else {
      restore_profile("follow_mode_exit");
    }
  }

  template<typename FutureT>
  bool wait_for_future(FutureT & future, bool allow_during_shutdown)
  {
    const auto timeout = allow_during_shutdown ?
      std::chrono::milliseconds(1000) :
      std::chrono::milliseconds(static_cast<int>(wait_timeout_s_ * 1000.0));
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() &&
      (allow_during_shutdown || !g_shutdown_requested.load()) &&
      std::chrono::steady_clock::now() < deadline)
    {
      const auto result = parameter_executor_.spin_until_future_complete(
        future, std::chrono::milliseconds(100));
      if (result == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
      }
      if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
        return false;
      }
    }
    return false;
  }

  std::shared_ptr<rclcpp::AsyncParametersClient> make_client(
    const std::string & node_name,
    bool allow_during_shutdown)
  {
    if (!allow_during_shutdown && g_shutdown_requested.load()) {
      return nullptr;
    }

    auto client =
      std::make_shared<rclcpp::AsyncParametersClient>(parameter_client_node_, node_name);
    const auto service_timeout = allow_during_shutdown ?
      std::chrono::milliseconds(1000) :
      std::chrono::milliseconds(static_cast<int>(wait_timeout_s_ * 1000.0));
    const auto deadline = std::chrono::steady_clock::now() +
      service_timeout;
    while (rclcpp::ok() &&
      (allow_during_shutdown || !g_shutdown_requested.load()) &&
      std::chrono::steady_clock::now() < deadline)
    {
      if (client->wait_for_service(std::chrono::milliseconds(100))) {
        return client;
      }
    }

    if (allow_during_shutdown || !g_shutdown_requested.load()) {
      RCLCPP_WARN(
        get_logger(), "Parameter service for %s is not available", node_name.c_str());
    }
    return nullptr;
  }

  void signal_check_loop()
  {
    if (!g_shutdown_requested.exchange(false)) {
      return;
    }

    publish_state("profile_shutdown_signal");
    restore_profile("shutdown_signal");
    rclcpp::shutdown();
  }

  void publish_state(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
  }

  bool enabled_;
  bool restore_on_shutdown_;
  bool mode_control_enabled_;
  int mode_required_;
  int mode_{-1};
  bool controller_profile_enabled_;
  bool behavior_profile_enabled_;
  bool amcl_profile_enabled_;
  bool restore_done_;
  std::string controller_server_name_;
  std::string behavior_server_name_;
  std::string amcl_node_name_;
  double wait_timeout_s_;

  double controller_frequency_;
  double controller_vx_min_;
  double controller_vx_max_;
  double controller_vx_std_;
  double controller_wz_max_;
  double controller_wz_std_;
  int controller_retry_attempt_limit_;
  double controller_failure_tolerance_;
  double controller_yaw_goal_tolerance_;
  bool prefer_forward_enabled_;
  bool path_angle_forward_preference_;
  double progress_required_movement_radius_;
  double progress_movement_time_allowance_;

  double behavior_max_rotational_vel_;
  double behavior_min_rotational_vel_;
  double behavior_rotational_acc_lim_;
  double behavior_simulate_ahead_time_;

  double amcl_alpha1_;
  double amcl_alpha2_;
  double amcl_alpha3_;
  double amcl_alpha4_;
  double amcl_update_min_d_;
  double amcl_update_min_a_;
  std::string amcl_laser_model_type_;
  bool amcl_do_beamskip_;

  std::map<std::string, std::vector<rclcpp::Parameter>> saved_parameters_;
  rclcpp::Node::SharedPtr parameter_client_node_;
  rclcpp::executors::SingleThreadedExecutor parameter_executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr signal_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  auto node = std::make_shared<BodyNav2ProfileGuard>();
  node->start();

  rclcpp::spin(node);
  node->restore_profile("spin_exit");

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
