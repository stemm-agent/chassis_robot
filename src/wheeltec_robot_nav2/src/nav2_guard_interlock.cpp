#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

using SteadyClock = std::chrono::steady_clock;

class Nav2GuardInterlock : public rclcpp::Node
{
public:
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
  using GetState = lifecycle_msgs::srv::GetState;

  Nav2GuardInterlock()
  : Node("nav2_guard_interlock")
  {
    anomaly_topic_ = declare_parameter<std::string>(
      "anomaly_topic", "/amcl_tf_guard/anomaly");
    lifecycle_manager_service_ = declare_parameter<std::string>(
      "lifecycle_manager_service", "/lifecycle_manager_navigation/manage_nodes");
    raw_cmd_vel_topic_ = declare_parameter<std::string>(
      "raw_cmd_vel_topic", "/nav2_cmd_vel");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    blocked_topic_ = declare_parameter<std::string>(
      "blocked_topic", "/nav2_guard_interlock/blocked");

    guard_message_timeout_sec_ = positive_parameter("guard_message_timeout_sec", 1.0, 0.05);
    healthy_stability_sec_ = positive_parameter("healthy_stability_sec", 2.0, 0.0);
    post_pause_settle_sec_ = positive_parameter("post_pause_settle_sec", 0.25, 0.0);
    post_activation_cancel_sec_ = positive_parameter("post_activation_cancel_sec", 0.5, 0.1);
    manager_retry_period_sec_ = positive_parameter("manager_retry_period_sec", 0.5, 0.05);
    manager_startup_timeout_sec_ = positive_parameter(
      "manager_startup_timeout_sec", 30.0, 1.0);
    manager_transition_timeout_sec_ = positive_parameter(
      "manager_transition_timeout_sec", 10.0, 1.0);
    manager_state_verification_timeout_sec_ = positive_parameter(
      "manager_state_verification_timeout_sec", 5.0, 0.5);
    cancel_retry_period_sec_ = positive_parameter("cancel_retry_period_sec", 0.2, 0.02);
    blocked_zero_burst_sec_ = positive_parameter("blocked_zero_burst_sec", 0.20, 0.0);
    control_period_ms_ =
      std::max<int64_t>(10, declare_parameter<int>("control_period_ms", 20));
    zero_burst_period_ms_ = std::max<int64_t>(
      control_period_ms_, declare_parameter<int>("zero_burst_period_ms", 20));
    blocked_heartbeat_period_sec_ = 1.0 / std::max(
      1.0, declare_parameter<double>("blocked_heartbeat_hz", 10.0));
    manage_initial_startup_ = declare_parameter<bool>("manage_initial_startup", true);
    navigation_started_ = !manage_initial_startup_;
    managed_node_names_ = declare_parameter<std::vector<std::string>>(
      "managed_node_names",
      {
        "/controller_server", "/smoother_server", "/planner_server",
        "/behavior_server", "/bt_navigator", "/waypoint_follower"
      });

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latched_qos.reliable().transient_local();
    blocked_publisher_ = create_publisher<std_msgs::msg::Bool>(blocked_topic_, latched_qos);
    anomaly_subscription_ = create_subscription<std_msgs::msg::Bool>(
      anomaly_topic_, latched_qos,
      std::bind(&Nav2GuardInterlock::on_anomaly, this, std::placeholders::_1));

    zero_velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10).reliable());
    raw_velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      raw_cmd_vel_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&Nav2GuardInterlock::on_nav2_velocity, this, std::placeholders::_1));
    lifecycle_manager_client_ = create_client<ManageLifecycleNodes>(lifecycle_manager_service_);
    for (const auto & node_name : managed_node_names_) {
      managed_node_state_clients_.push_back(create_client<GetState>(node_name + "/get_state"));
    }
    manager_state_request_ids_.assign(managed_node_state_clients_.size(), -1);

    navigate_to_pose_client_ = make_action_client<nav2_msgs::action::NavigateToPose>(
      "navigate_to_pose");
    navigate_through_poses_client_ =
      make_action_client<nav2_msgs::action::NavigateThroughPoses>("navigate_through_poses");
    follow_waypoints_client_ = make_action_client<nav2_msgs::action::FollowWaypoints>(
      "follow_waypoints");
    follow_path_client_ = make_action_client<nav2_msgs::action::FollowPath>("follow_path");
    compute_path_to_pose_client_ = make_action_client<nav2_msgs::action::ComputePathToPose>(
      "compute_path_to_pose");
    compute_path_through_poses_client_ =
      make_action_client<nav2_msgs::action::ComputePathThroughPoses>(
      "compute_path_through_poses");
    smooth_path_client_ = make_action_client<nav2_msgs::action::SmoothPath>("smooth_path");
    spin_client_ = make_action_client<nav2_msgs::action::Spin>("spin");
    backup_client_ = make_action_client<nav2_msgs::action::BackUp>("backup");
    drive_on_heading_client_ = make_action_client<nav2_msgs::action::DriveOnHeading>(
      "drive_on_heading");
    assisted_teleop_client_ = make_action_client<nav2_msgs::action::AssistedTeleop>(
      "assisted_teleop");
    wait_client_ = make_action_client<nav2_msgs::action::Wait>("wait");

    // Unknown guard state is unsafe. Publish the latched state immediately and
    // emit only a bounded zero burst. After that burst the final /cmd_vel bus is
    // intentionally silent so manual and other independently owned modes are not
    // continuously overwritten while Nav2 remains fail-closed.
    const auto startup_now = SteadyClock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      start_zero_burst_locked(startup_now);
      last_blocked_heartbeat_ = startup_now;
      have_last_blocked_heartbeat_ = true;
    }
    publish_blocked(true);
    service_zero_burst(startup_now);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&Nav2GuardInterlock::control_tick, this));

    if (manage_initial_startup_) {
      RCLCPP_WARN(
        get_logger(),
        "Nav2 guard interlock starts BLOCKED with navigation autostart disabled. "
        "Waiting for a fresh, stable localization guard before STARTUP while dropping Nav2 "
        "velocity, issuing a bounded zero burst, and cancelling goals. STARTUP may open the "
        "gate only when its guard epoch remains current; otherwise PAUSE is forced before "
        "any later RESUME. Guard: %s, "
        "stability: %.2f s. Velocity gate: %s -> %s. Zero burst: %.2f s / %d ms.",
        anomaly_topic_.c_str(), healthy_stability_sec_, raw_cmd_vel_topic_.c_str(),
        cmd_vel_topic_.c_str(), blocked_zero_burst_sec_, zero_burst_period_ms_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Nav2 guard interlock starts BLOCKED in compatibility mode. Waiting for a "
        "successful navigation lifecycle PAUSE and %.2f s of stable health before RESUME.",
        healthy_stability_sec_);
    }
  }

private:
  enum class ManagerCommand
  {
    kNone,
    kStartup,
    kPause,
    kResume,
  };

  double positive_parameter(
    const std::string & name, const double default_value, const double minimum)
  {
    const double configured = declare_parameter<double>(name, default_value);
    if (configured >= minimum) {
      return configured;
    }
    RCLCPP_WARN(
      get_logger(), "Parameter %s=%.3f is below its safe minimum %.3f; clamping it.",
      name.c_str(), configured, minimum);
    return minimum;
  }

  template<typename ActionT>
  typename rclcpp_action::Client<ActionT>::SharedPtr make_action_client(
    const std::string & action_name)
  {
    return rclcpp_action::create_client<ActionT>(this, action_name);
  }

  void on_anomaly(const std_msgs::msg::Bool::SharedPtr message)
  {
    const auto now = SteadyClock::now();
    bool newly_blocked = false;
    bool newly_healthy = false;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      have_guard_receipt_ = true;
      last_guard_receipt_ = now;

      if (message->data) {
        if (!guard_sample_received_ || guard_healthy_) {
          ++state_epoch_;
        }
        guard_sample_received_ = true;
        guard_healthy_ = false;
        have_healthy_since_ = false;
        newly_blocked = !blocked_;
        blocked_ = true;
        if (newly_blocked) {
          start_zero_burst_locked(now);
        }

        // A RESUME request may already be executing in the lifecycle manager.
        // Its eventual response must not be allowed to unlock this newer epoch.
        if (manager_request_in_flight_ &&
          (manager_command_in_flight_ == ManagerCommand::kStartup ||
          manager_command_in_flight_ == ManagerCommand::kResume))
        {
          pause_confirmed_ = false;
        }
      } else {
        if (!guard_sample_received_ || !guard_healthy_) {
          ++state_epoch_;
          healthy_since_ = now;
          have_healthy_since_ = true;
          newly_healthy = true;
        }
        guard_sample_received_ = true;
        guard_healthy_ = true;
      }
    }

    if (message->data) {
      if (newly_blocked) {
        publish_blocked(true);
        service_zero_burst(now);
        cancel_all_navigation_goals();
        RCLCPP_ERROR(
          get_logger(),
          "AMCL TF guard asserted anomaly: blocking goals, issuing a bounded zero burst, "
          "and pausing the Nav2 navigation lifecycle.");
      }
    } else if (newly_healthy) {
      RCLCPP_INFO(
        get_logger(),
        "AMCL TF guard is healthy; remaining BLOCKED during the %.2f s stability window.",
        healthy_stability_sec_);
    }

    // React in the subscription callback instead of waiting for the next timer
    // period.  The timer remains the watchdog and retry path.
    control_tick();
  }

  void on_nav2_velocity(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    // Keep the state check and forwarding publication in one critical section.
    // A transition into BLOCKED starts one bounded zero burst under the same
    // lock, so a command which wins this race is cleared immediately afterwards.
    // Once the burst expires, rejected Nav2 commands are dropped silently: this
    // interlock must not continuously contend with manual/follow publishers on
    // the legacy shared final /cmd_vel bus.
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!blocked_) {
      zero_velocity_publisher_->publish(*message);
    }
  }

  void control_tick()
  {
    const auto now = SteadyClock::now();
    bool guard_timed_out = false;
    bool entered_blocked = false;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (guard_healthy_ &&
        (!have_guard_receipt_ ||
        seconds_between(last_guard_receipt_, now) > guard_message_timeout_sec_))
      {
        guard_timed_out = true;
        ++state_epoch_;
        guard_healthy_ = false;
        have_healthy_since_ = false;
        entered_blocked = !blocked_;
        blocked_ = true;
        if (entered_blocked) {
          start_zero_burst_locked(now);
        }
        if (manager_request_in_flight_ &&
          (manager_command_in_flight_ == ManagerCommand::kStartup ||
          manager_command_in_flight_ == ManagerCommand::kResume))
        {
          pause_confirmed_ = false;
        }
      }
    }

    if (guard_timed_out) {
      publish_blocked(true);
      if (entered_blocked) {
        service_zero_burst(now);
      }
      RCLCPP_ERROR(
        get_logger(),
        "Guard heartbeat timed out after %.2f s; re-entering BLOCKED fail-closed state.",
        guard_message_timeout_sec_);
    }

    handle_manager_request_timeout(now);
    handle_manager_state_verification_timeout(now);
    service_zero_burst(now);

    bool blocked = true;
    bool cancel_due = false;
    bool heartbeat_due = false;
    bool release_after_activation_cleanup = false;
    std::uint64_t release_epoch = 0;
    ManagerCommand desired_command = ManagerCommand::kNone;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (activation_cleanup_pending_) {
        if (!guard_healthy_ || activation_cleanup_epoch_ != state_epoch_) {
          activation_cleanup_pending_ = false;
          pause_confirmed_ = false;
        } else if (
          seconds_between(activation_cleanup_started_at_, now) >= post_activation_cancel_sec_)
        {
          activation_cleanup_pending_ = false;
          release_after_activation_cleanup = true;
          release_epoch = state_epoch_;
        }
      }

      blocked = blocked_;
      if (blocked_ && !release_after_activation_cleanup) {
        if (!have_last_cancel_attempt_ ||
          seconds_between(last_cancel_attempt_, now) >= cancel_retry_period_sec_)
        {
          last_cancel_attempt_ = now;
          have_last_cancel_attempt_ = true;
          cancel_due = true;
        }

        if (!manager_request_in_flight_ && !manager_state_verification_in_flight_ &&
          !activation_cleanup_pending_)
        {
          if (!navigation_started_) {
            const bool guard_is_fresh = have_guard_receipt_ &&
              seconds_between(last_guard_receipt_, now) <= guard_message_timeout_sec_;
            if (guard_sample_received_ && guard_healthy_ && guard_is_fresh &&
              have_healthy_since_ &&
              seconds_between(healthy_since_, now) >= healthy_stability_sec_)
            {
              // Localization owns startup ordering: a trusted map transform must
              // exist before Nav2 costmaps are allowed to activate.
              desired_command = ManagerCommand::kStartup;
            }
          } else if (!pause_confirmed_) {
            desired_command = ManagerCommand::kPause;
          } else if (guard_sample_received_ && guard_healthy_ && have_healthy_since_ &&
            seconds_between(healthy_since_, now) >= healthy_stability_sec_ &&
            seconds_between(pause_confirmed_at_, now) >= post_pause_settle_sec_)
          {
            desired_command = ManagerCommand::kResume;
          }
        }
      }

      if (!have_last_blocked_heartbeat_ ||
        seconds_between(last_blocked_heartbeat_, now) >= blocked_heartbeat_period_sec_)
      {
        last_blocked_heartbeat_ = now;
        have_last_blocked_heartbeat_ = true;
        heartbeat_due = true;
      }
    }

    if (release_after_activation_cleanup) {
      // Cancel once more immediately before opening the gate.  The preceding
      // hold covered the period in which action servers became active while the
      // lifecycle RESUME response was still in flight.
      cancel_all_navigation_goals();
      bool released = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (blocked_ && guard_healthy_ && state_epoch_ == release_epoch) {
          blocked_ = false;
          pause_confirmed_ = false;
          zero_burst_active_ = false;
          released = true;
          // Publish while holding the gate lock so a raw velocity callback
          // cannot pass a command before consumers observe the open state.
          publish_blocked(false);
        }
      }
      if (released) {
        RCLCPP_INFO(
          get_logger(),
          "Post-activation cancellation hold completed; accepting only new Nav2 goals.");
      } else {
        publish_blocked(true);
        RCLCPP_ERROR(
          get_logger(),
          "Guard state changed at the activation gate boundary; remaining BLOCKED.");
      }
      return;
    }

    // This is the interlock liveness heartbeat. Consumers must treat a stale
    // value as blocked even if the last received sample was false. Keep it at a
    // bounded rate independent of the 50 Hz control/cancellation timer.
    if (heartbeat_due) {
      publish_blocked(blocked);
    }
    if (!blocked) {
      return;
    }

    if (cancel_due) {
      cancel_all_navigation_goals();
    }
    if (desired_command != ManagerCommand::kNone) {
      try_manager_command(desired_command, now);
    }
  }

  void try_manager_command(const ManagerCommand command, const SteadyClock::time_point now)
  {
    std::uint64_t request_epoch = 0;
    std::uint64_t request_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (manager_request_in_flight_ || manager_state_verification_in_flight_ || !blocked_) {
        return;
      }
      if (command == ManagerCommand::kStartup &&
        (navigation_started_ || !guard_sample_received_ || !guard_healthy_ ||
        !have_guard_receipt_ ||
        seconds_between(last_guard_receipt_, now) > guard_message_timeout_sec_ ||
        !have_healthy_since_ ||
        seconds_between(healthy_since_, now) < healthy_stability_sec_))
      {
        return;
      }
      if (command == ManagerCommand::kPause && pause_confirmed_) {
        return;
      }
      if (command == ManagerCommand::kResume &&
        (!pause_confirmed_ || !guard_sample_received_ || !guard_healthy_ ||
        !have_healthy_since_ ||
        seconds_between(healthy_since_, now) < healthy_stability_sec_ ||
        seconds_between(pause_confirmed_at_, now) < post_pause_settle_sec_))
      {
        return;
      }
      if (have_last_manager_attempt_ &&
        seconds_between(last_manager_attempt_, now) < manager_retry_period_sec_)
      {
        return;
      }
      last_manager_attempt_ = now;
      have_last_manager_attempt_ = true;
      request_epoch = state_epoch_;
    }

    if (!lifecycle_manager_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Navigation lifecycle manager service %s is unavailable; the Nav2 velocity gate "
        "and action cancellation remain active.", lifecycle_manager_service_.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      // Recheck after graph discovery without weakening the fail-closed state.
      if (manager_request_in_flight_ || manager_state_verification_in_flight_ || !blocked_ ||
        (command == ManagerCommand::kStartup &&
        (navigation_started_ || state_epoch_ != request_epoch ||
        !guard_sample_received_ || !guard_healthy_ || !have_guard_receipt_ ||
        seconds_between(last_guard_receipt_, now) > guard_message_timeout_sec_ ||
        !have_healthy_since_ ||
        seconds_between(healthy_since_, now) < healthy_stability_sec_)) ||
        (command == ManagerCommand::kPause && pause_confirmed_) ||
        (command == ManagerCommand::kResume &&
        (!pause_confirmed_ || !guard_healthy_ || state_epoch_ != request_epoch)))
      {
        return;
      }
      manager_request_in_flight_ = true;
      manager_command_in_flight_ = command;
      manager_request_started_at_ = now;
      manager_request_epoch_ = request_epoch;
      request_sequence = ++manager_request_sequence_counter_;
      active_manager_request_sequence_ = request_sequence;
      manager_request_id_ = -1;
    }

    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    switch (command) {
      case ManagerCommand::kStartup:
        request->command = ManageLifecycleNodes::Request::STARTUP;
        break;
      case ManagerCommand::kPause:
        request->command = ManageLifecycleNodes::Request::PAUSE;
        break;
      case ManagerCommand::kResume:
        request->command = ManageLifecycleNodes::Request::RESUME;
        break;
      case ManagerCommand::kNone:
        return;
    }

    try {
      const auto future_and_request_id = lifecycle_manager_client_->async_send_request(
        request,
        [this, command, request_epoch, request_sequence](
          rclcpp::Client<ManageLifecycleNodes>::SharedFuture future)
        {
          on_manager_response(command, request_epoch, request_sequence, future);
        });
      bool response_was_already_invalidated = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (manager_request_in_flight_ &&
          active_manager_request_sequence_ == request_sequence)
        {
          manager_request_id_ = future_and_request_id.request_id;
        } else {
          response_was_already_invalidated = true;
        }
      }
      if (response_was_already_invalidated) {
        (void)lifecycle_manager_client_->remove_pending_request(
          future_and_request_id.request_id);
      }
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (manager_request_in_flight_ &&
          active_manager_request_sequence_ == request_sequence)
        {
          manager_request_in_flight_ = false;
          manager_command_in_flight_ = ManagerCommand::kNone;
          active_manager_request_sequence_ = 0;
          manager_request_id_ = -1;
        }
      }
      RCLCPP_ERROR(
        get_logger(), "Failed to send Nav2 lifecycle command: %s", error.what());
    }
  }

  void on_manager_response(
    const ManagerCommand command,
    const std::uint64_t request_epoch,
    const std::uint64_t request_sequence,
    rclcpp::Client<ManageLifecycleNodes>::SharedFuture future)
  {
    bool success = false;
    try {
      const auto response = future.get();
      success = response && response->success;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Nav2 lifecycle command response failed: %s", error.what());
    }

    const auto now = SteadyClock::now();
    bool activation_cleanup_started = false;
    bool startup_requires_pause = false;
    bool stale_resume = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!manager_request_in_flight_ ||
        manager_command_in_flight_ != command ||
        active_manager_request_sequence_ != request_sequence)
      {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring late Nav2 lifecycle %s response from invalidated request sequence %llu.",
          manager_command_name(command),
          static_cast<unsigned long long>(request_sequence));
        return;
      }
      manager_request_in_flight_ = false;
      manager_command_in_flight_ = ManagerCommand::kNone;
      active_manager_request_sequence_ = 0;
      manager_request_id_ = -1;

      if (command == ManagerCommand::kStartup) {
        const bool guard_is_fresh = have_guard_receipt_ &&
          seconds_between(last_guard_receipt_, now) <= guard_message_timeout_sec_;
        const bool guard_is_stable = guard_sample_received_ && guard_healthy_ &&
          guard_is_fresh && have_healthy_since_ &&
          seconds_between(healthy_since_, now) >= healthy_stability_sec_;
        const bool can_skip_pause = success && blocked_ && request_epoch == state_epoch_ &&
          !activation_cleanup_pending_ && guard_is_stable;

        // Even a failed STARTUP may have configured or activated a prefix of
        // the managed-node list. Never retry STARTUP blindly. If the gate stayed
        // blocked and the guard is already fresh and stable when a successful
        // STARTUP returns, its active nodes can enter the existing cancellation
        // cleanup directly. Otherwise force PAUSE so partial or unsafe activation
        // cannot command motion. Direct cleanup requires both current guard health
        // and the original STARTUP request epoch; the gate remained closed for the
        // whole request.
        navigation_started_ = true;
        blocked_ = true;
        pause_confirmed_ = false;
        have_last_manager_attempt_ = false;
        if (can_skip_pause) {
          activation_cleanup_pending_ = true;
          activation_cleanup_epoch_ = state_epoch_;
          activation_cleanup_started_at_ = now;
          activation_cleanup_started = true;
        } else {
          activation_cleanup_pending_ = false;
          startup_requires_pause = true;
        }
      } else if (command == ManagerCommand::kPause) {
        activation_cleanup_pending_ = false;
        pause_confirmed_ = success;
        if (success) {
          pause_confirmed_at_ = now;
        }
      } else {
        const bool guard_is_fresh = have_guard_receipt_ &&
          seconds_between(last_guard_receipt_, now) <= guard_message_timeout_sec_;
        const bool response_matches_current_safe_epoch =
          success && request_epoch == state_epoch_ && guard_sample_received_ &&
          guard_healthy_ && guard_is_fresh && have_healthy_since_ &&
          seconds_between(healthy_since_, now) >= healthy_stability_sec_;

        if (response_matches_current_safe_epoch) {
          blocked_ = true;
          pause_confirmed_ = false;
          activation_cleanup_pending_ = true;
          activation_cleanup_epoch_ = state_epoch_;
          activation_cleanup_started_at_ = now;
          activation_cleanup_started = true;
        } else {
          // A failed RESUME may have activated only a subset of managed nodes;
          // a stale successful RESUME definitely may have activated all of them.
          // In both cases, require a fresh PAUSE before any later recovery.
          blocked_ = true;
          pause_confirmed_ = false;
          stale_resume = success;
        }
      }
    }

    if (command == ManagerCommand::kStartup && startup_requires_pause) {
      publish_blocked(true);
      cancel_all_navigation_goals();
      if (success) {
        RCLCPP_WARN(
          get_logger(),
          "Blocked Nav2 prewarm STARTUP completed, but direct-release guard/gate "
          "prerequisites were not all satisfied; immediately forcing PAUSE before any "
          "later RESUME.");
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Nav2 prewarm STARTUP failed and may have partially activated nodes; remaining "
          "BLOCKED and immediately forcing PAUSE.");
      }
    } else if (command == ManagerCommand::kPause && success) {
      cancel_all_navigation_goals();
      RCLCPP_WARN(
        get_logger(),
        "Nav2 navigation lifecycle is PAUSED; all pre-HOLD goals are invalidated.");
    } else if (activation_cleanup_started) {
      cancel_all_navigation_goals();
      publish_blocked(true);
      RCLCPP_INFO(
        get_logger(),
        "Guard health remained stable and Nav2 %s succeeded; %s PAUSE and keeping the "
        "velocity gate closed while cancelling goals for %.2f s before opening it.",
        command == ManagerCommand::kStartup ? "STARTUP" : "RESUME",
        command == ManagerCommand::kStartup ? "skipping" : "leaving",
        post_activation_cancel_sec_);
    } else {
      publish_blocked(true);
      cancel_all_navigation_goals();
      if (command == ManagerCommand::kResume && stale_resume)
      {
        RCLCPP_ERROR(
          get_logger(),
          "A stale Nav2 RESUME completed after guard state changed; immediately re-pausing.");
      } else {
        RCLCPP_ERROR(
          get_logger(), "Nav2 lifecycle %s failed; remaining BLOCKED and retrying safely.",
          command == ManagerCommand::kPause ? "PAUSE" : "RESUME");
      }
    }

    control_tick();
  }

  static const char * manager_command_name(const ManagerCommand command)
  {
    switch (command) {
      case ManagerCommand::kStartup:
        return "STARTUP";
      case ManagerCommand::kPause:
        return "PAUSE";
      case ManagerCommand::kResume:
        return "RESUME";
      case ManagerCommand::kNone:
      default:
        return "NONE";
    }
  }

  double manager_timeout_for(const ManagerCommand command) const
  {
    return command == ManagerCommand::kStartup ?
           manager_startup_timeout_sec_ : manager_transition_timeout_sec_;
  }

  void handle_manager_request_timeout(const SteadyClock::time_point now)
  {
    ManagerCommand timed_out_command = ManagerCommand::kNone;
    std::uint64_t timed_out_epoch = 0;
    std::uint64_t timed_out_sequence = 0;
    int64_t timed_out_request_id = -1;
    double timeout_sec = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!manager_request_in_flight_) {
        return;
      }
      timeout_sec = manager_timeout_for(manager_command_in_flight_);
      if (seconds_between(manager_request_started_at_, now) < timeout_sec) {
        return;
      }

      timed_out_command = manager_command_in_flight_;
      timed_out_epoch = manager_request_epoch_;
      timed_out_sequence = active_manager_request_sequence_;
      timed_out_request_id = manager_request_id_;
      manager_request_in_flight_ = false;
      manager_command_in_flight_ = ManagerCommand::kNone;
      active_manager_request_sequence_ = 0;
      manager_request_id_ = -1;
    }

    if (timed_out_request_id >= 0) {
      (void)lifecycle_manager_client_->remove_pending_request(timed_out_request_id);
    }
    RCLCPP_ERROR(
      get_logger(),
      "Nav2 lifecycle %s request sequence %llu timed out after %.2f s; keeping BLOCKED "
      "and verifying every managed node state before retrying.",
      manager_command_name(timed_out_command),
      static_cast<unsigned long long>(timed_out_sequence), timeout_sec);
    start_manager_state_verification(timed_out_command, timed_out_epoch, now);
  }

  void start_manager_state_verification(
    const ManagerCommand command,
    const std::uint64_t request_epoch,
    const SteadyClock::time_point now)
  {
    bool every_service_ready = !managed_node_state_clients_.empty();
    for (const auto & client : managed_node_state_clients_) {
      every_service_ready = every_service_ready && client && client->service_is_ready();
    }
    if (!every_service_ready) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot verify timed-out Nav2 %s: at least one managed get_state service is "
        "unavailable. Remaining BLOCKED and forcing a PAUSE retry.",
        manager_command_name(command));
      reconcile_manager_state_after_timeout(
        command, request_epoch, false, false, false, now);
      return;
    }

    std::uint64_t verification_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (manager_state_verification_in_flight_) {
        return;
      }
      manager_state_verification_in_flight_ = true;
      manager_state_verification_command_ = command;
      manager_state_verification_epoch_ = request_epoch;
      manager_state_verification_started_at_ = now;
      verification_sequence = ++manager_state_verification_sequence_counter_;
      active_manager_state_verification_sequence_ = verification_sequence;
      manager_state_response_count_ = 0;
      manager_state_all_known_ = true;
      manager_state_all_active_ = true;
      manager_state_all_inactive_ = true;
      manager_state_response_received_.assign(managed_node_state_clients_.size(), false);
      manager_state_request_ids_.assign(managed_node_state_clients_.size(), -1);
    }

    for (std::size_t index = 0; index < managed_node_state_clients_.size(); ++index) {
      auto request = std::make_shared<GetState::Request>();
      try {
        const auto future_and_request_id = managed_node_state_clients_[index]->async_send_request(
          request,
          [this, index, verification_sequence](rclcpp::Client<GetState>::SharedFuture future)
          {
            on_manager_state_response(index, verification_sequence, future);
          });
        bool response_was_already_invalidated = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (manager_state_verification_in_flight_ &&
            active_manager_state_verification_sequence_ == verification_sequence &&
            !manager_state_response_received_[index])
          {
            manager_state_request_ids_[index] = future_and_request_id.request_id;
          } else {
            response_was_already_invalidated = true;
          }
        }
        if (response_was_already_invalidated) {
          (void)managed_node_state_clients_[index]->remove_pending_request(
            future_and_request_id.request_id);
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Failed to query lifecycle state for %s: %s",
          managed_node_names_[index].c_str(), error.what());
        record_manager_state_response(index, verification_sequence, false, 0, now);
      }
    }
  }

  void on_manager_state_response(
    const std::size_t index,
    const std::uint64_t verification_sequence,
    rclcpp::Client<GetState>::SharedFuture future)
  {
    bool known = false;
    std::uint8_t state_id = 0;
    try {
      const auto response = future.get();
      if (response) {
        known = true;
        state_id = response->current_state.id;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Managed lifecycle get_state response failed: %s", error.what());
    }
    record_manager_state_response(
      index, verification_sequence, known, state_id, SteadyClock::now());
  }

  void record_manager_state_response(
    const std::size_t index,
    const std::uint64_t verification_sequence,
    const bool known,
    const std::uint8_t state_id,
    const SteadyClock::time_point now)
  {
    bool verification_complete = false;
    bool all_known = false;
    bool all_active = false;
    bool all_inactive = false;
    ManagerCommand command = ManagerCommand::kNone;
    std::uint64_t request_epoch = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!manager_state_verification_in_flight_ ||
        active_manager_state_verification_sequence_ != verification_sequence ||
        index >= manager_state_response_received_.size() ||
        manager_state_response_received_[index])
      {
        return;
      }

      manager_state_response_received_[index] = true;
      manager_state_request_ids_[index] = -1;
      ++manager_state_response_count_;
      manager_state_all_known_ = manager_state_all_known_ && known;
      manager_state_all_active_ = manager_state_all_active_ && known &&
        state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      manager_state_all_inactive_ = manager_state_all_inactive_ && known &&
        state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

      if (manager_state_response_count_ == managed_node_state_clients_.size()) {
        verification_complete = true;
        all_known = manager_state_all_known_;
        all_active = manager_state_all_active_;
        all_inactive = manager_state_all_inactive_;
        command = manager_state_verification_command_;
        request_epoch = manager_state_verification_epoch_;
        manager_state_verification_in_flight_ = false;
        manager_state_verification_command_ = ManagerCommand::kNone;
        active_manager_state_verification_sequence_ = 0;
      }
    }

    if (verification_complete) {
      reconcile_manager_state_after_timeout(
        command, request_epoch, all_known, all_active, all_inactive, now);
      control_tick();
    }
  }

  void handle_manager_state_verification_timeout(const SteadyClock::time_point now)
  {
    bool timed_out = false;
    ManagerCommand command = ManagerCommand::kNone;
    std::uint64_t request_epoch = 0;
    std::vector<int64_t> request_ids;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!manager_state_verification_in_flight_ ||
        seconds_between(manager_state_verification_started_at_, now) <
        manager_state_verification_timeout_sec_)
      {
        return;
      }
      timed_out = true;
      command = manager_state_verification_command_;
      request_epoch = manager_state_verification_epoch_;
      request_ids = manager_state_request_ids_;
      manager_state_verification_in_flight_ = false;
      manager_state_verification_command_ = ManagerCommand::kNone;
      active_manager_state_verification_sequence_ = 0;
    }

    for (std::size_t index = 0; index < request_ids.size(); ++index) {
      if (request_ids[index] >= 0) {
        (void)managed_node_state_clients_[index]->remove_pending_request(request_ids[index]);
      }
    }
    if (timed_out) {
      RCLCPP_ERROR(
        get_logger(),
        "Lifecycle state verification after timed-out %s exceeded %.2f s; keeping "
        "BLOCKED and forcing a PAUSE retry.",
        manager_command_name(command), manager_state_verification_timeout_sec_);
      reconcile_manager_state_after_timeout(
        command, request_epoch, false, false, false, now);
    }
  }

  void reconcile_manager_state_after_timeout(
    const ManagerCommand command,
    const std::uint64_t request_epoch,
    const bool all_known,
    const bool all_active,
    const bool all_inactive,
    const SteadyClock::time_point now)
  {
    bool activation_cleanup_started = false;
    bool pause_verified = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      blocked_ = true;
      activation_cleanup_pending_ = false;
      have_last_manager_attempt_ = false;
      if (command == ManagerCommand::kStartup) {
        // STARTUP must never be retried blindly after an ambiguous timeout: it
        // may already have activated an arbitrary prefix of the managed nodes.
        navigation_started_ = true;
      }

      const bool guard_is_fresh = have_guard_receipt_ &&
        seconds_between(last_guard_receipt_, now) <= guard_message_timeout_sec_;
      const bool guard_is_stable = guard_sample_received_ && guard_healthy_ &&
        guard_is_fresh && have_healthy_since_ &&
        seconds_between(healthy_since_, now) >= healthy_stability_sec_;
      const bool safe_epoch = request_epoch == state_epoch_;

      if (all_known && all_inactive) {
        pause_confirmed_ = true;
        pause_confirmed_at_ = now;
        pause_verified = true;
      } else if (
        all_known && all_active && command != ManagerCommand::kPause &&
        guard_is_stable && safe_epoch)
      {
        pause_confirmed_ = false;
        activation_cleanup_pending_ = true;
        activation_cleanup_epoch_ = state_epoch_;
        activation_cleanup_started_at_ = now;
        activation_cleanup_started = true;
      } else {
        // Mixed, unknown, unsafe-active, or a PAUSE which left anything active:
        // invalidate any former PAUSE proof and retry PAUSE fail-closed.
        pause_confirmed_ = false;
      }
    }

    publish_blocked(true);
    cancel_all_navigation_goals();
    if (activation_cleanup_started) {
      RCLCPP_WARN(
        get_logger(),
        "Lifecycle verification found all managed nodes ACTIVE after timed-out %s; "
        "the guard/epoch are safe, so keeping the gate closed for the %.2f s "
        "post-activation cancellation window.",
        manager_command_name(command), post_activation_cancel_sec_);
    } else if (pause_verified) {
      RCLCPP_WARN(
        get_logger(),
        "Lifecycle verification found all managed nodes INACTIVE after timed-out %s; "
        "PAUSE is confirmed and recovery may retry from the guarded state.",
        manager_command_name(command));
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Lifecycle state after timed-out %s is mixed, unknown, or unsafe; a fresh "
        "PAUSE will be retried while the gate remains BLOCKED.",
        manager_command_name(command));
    }
  }

  template<typename ActionT>
  void cancel_action_goals(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const char * action_name)
  {
    if (!client || !client->action_server_is_ready()) {
      return;
    }
    try {
      (void)client->async_cancel_all_goals();
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not cancel goals on %s yet: %s", action_name, error.what());
    }
  }

  void cancel_all_navigation_goals()
  {
    // Cancel both orchestration actions and their lower-level children.  This
    // closes races where a waypoint/BT goal starts a controller or behavior
    // action while the lifecycle PAUSE request is still in flight.
    cancel_action_goals<nav2_msgs::action::FollowWaypoints>(
      follow_waypoints_client_, "follow_waypoints");
    cancel_action_goals<nav2_msgs::action::NavigateThroughPoses>(
      navigate_through_poses_client_, "navigate_through_poses");
    cancel_action_goals<nav2_msgs::action::NavigateToPose>(
      navigate_to_pose_client_, "navigate_to_pose");
    cancel_action_goals<nav2_msgs::action::FollowPath>(follow_path_client_, "follow_path");
    cancel_action_goals<nav2_msgs::action::ComputePathToPose>(
      compute_path_to_pose_client_, "compute_path_to_pose");
    cancel_action_goals<nav2_msgs::action::ComputePathThroughPoses>(
      compute_path_through_poses_client_, "compute_path_through_poses");
    cancel_action_goals<nav2_msgs::action::SmoothPath>(smooth_path_client_, "smooth_path");
    cancel_action_goals<nav2_msgs::action::Spin>(spin_client_, "spin");
    cancel_action_goals<nav2_msgs::action::BackUp>(backup_client_, "backup");
    cancel_action_goals<nav2_msgs::action::DriveOnHeading>(
      drive_on_heading_client_, "drive_on_heading");
    cancel_action_goals<nav2_msgs::action::AssistedTeleop>(
      assisted_teleop_client_, "assisted_teleop");
    cancel_action_goals<nav2_msgs::action::Wait>(wait_client_, "wait");
  }

  void start_zero_burst_locked(const SteadyClock::time_point now)
  {
    have_last_zero_burst_publish_ = false;
    if (blocked_zero_burst_sec_ <= 0.0) {
      zero_burst_active_ = false;
      return;
    }
    zero_burst_active_ = true;
    zero_burst_until_ = now + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(blocked_zero_burst_sec_));
  }

  void service_zero_burst(const SteadyClock::time_point now)
  {
    bool publish_zero = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!zero_burst_active_) {
        return;
      }
      if (now >= zero_burst_until_) {
        zero_burst_active_ = false;
        return;
      }
      const double zero_burst_period_sec =
        static_cast<double>(zero_burst_period_ms_) / 1000.0;
      if (!have_last_zero_burst_publish_ ||
        seconds_between(last_zero_burst_publish_, now) >= zero_burst_period_sec)
      {
        last_zero_burst_publish_ = now;
        have_last_zero_burst_publish_ = true;
        publish_zero = true;
      }
    }
    if (publish_zero) {
      publish_zero_velocity();
    }
  }

  void publish_zero_velocity()
  {
    zero_velocity_publisher_->publish(geometry_msgs::msg::Twist());
  }

  void publish_blocked(const bool blocked)
  {
    std_msgs::msg::Bool message;
    message.data = blocked;
    blocked_publisher_->publish(message);
  }

  static double seconds_between(
    const SteadyClock::time_point start, const SteadyClock::time_point end)
  {
    return std::chrono::duration<double>(end - start).count();
  }

  std::string anomaly_topic_;
  std::string lifecycle_manager_service_;
  std::string raw_cmd_vel_topic_;
  std::string cmd_vel_topic_;
  std::string blocked_topic_;
  double guard_message_timeout_sec_{1.0};
  double healthy_stability_sec_{2.0};
  double post_pause_settle_sec_{0.25};
  double post_activation_cancel_sec_{0.5};
  double manager_retry_period_sec_{0.5};
  double manager_startup_timeout_sec_{30.0};
  double manager_transition_timeout_sec_{10.0};
  double manager_state_verification_timeout_sec_{5.0};
  double cancel_retry_period_sec_{0.2};
  double blocked_zero_burst_sec_{0.2};
  double blocked_heartbeat_period_sec_{0.1};
  int control_period_ms_{20};
  int zero_burst_period_ms_{20};
  bool manage_initial_startup_{true};
  std::vector<std::string> managed_node_names_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr zero_velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blocked_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr anomaly_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_velocity_subscription_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr lifecycle_manager_client_;
  std::vector<rclcpp::Client<GetState>::SharedPtr> managed_node_state_clients_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
    navigate_to_pose_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    navigate_through_poses_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    follow_waypoints_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr
    compute_path_to_pose_client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr
    compute_path_through_poses_client_;
  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_path_client_;
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;
  rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<nav2_msgs::action::DriveOnHeading>::SharedPtr
    drive_on_heading_client_;
  rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>::SharedPtr
    assisted_teleop_client_;
  rclcpp_action::Client<nav2_msgs::action::Wait>::SharedPtr wait_client_;

  std::mutex state_mutex_;
  bool blocked_{true};
  bool guard_sample_received_{false};
  bool guard_healthy_{false};
  bool have_guard_receipt_{false};
  bool have_healthy_since_{false};
  bool navigation_started_{false};
  bool pause_confirmed_{false};
  bool activation_cleanup_pending_{false};
  bool manager_request_in_flight_{false};
  bool manager_state_verification_in_flight_{false};
  bool have_last_manager_attempt_{false};
  bool have_last_cancel_attempt_{false};
  bool have_last_blocked_heartbeat_{false};
  bool zero_burst_active_{false};
  bool have_last_zero_burst_publish_{false};
  bool manager_state_all_known_{false};
  bool manager_state_all_active_{false};
  bool manager_state_all_inactive_{false};
  ManagerCommand manager_command_in_flight_{ManagerCommand::kNone};
  ManagerCommand manager_state_verification_command_{ManagerCommand::kNone};
  std::uint64_t state_epoch_{0};
  std::uint64_t activation_cleanup_epoch_{0};
  std::uint64_t manager_request_epoch_{0};
  std::uint64_t manager_request_sequence_counter_{0};
  std::uint64_t active_manager_request_sequence_{0};
  std::uint64_t manager_state_verification_epoch_{0};
  std::uint64_t manager_state_verification_sequence_counter_{0};
  std::uint64_t active_manager_state_verification_sequence_{0};
  std::size_t manager_state_response_count_{0};
  int64_t manager_request_id_{-1};
  std::vector<int64_t> manager_state_request_ids_;
  std::vector<bool> manager_state_response_received_;
  SteadyClock::time_point last_guard_receipt_;
  SteadyClock::time_point healthy_since_;
  SteadyClock::time_point pause_confirmed_at_;
  SteadyClock::time_point activation_cleanup_started_at_;
  SteadyClock::time_point last_manager_attempt_;
  SteadyClock::time_point last_cancel_attempt_;
  SteadyClock::time_point last_blocked_heartbeat_;
  SteadyClock::time_point zero_burst_until_;
  SteadyClock::time_point last_zero_burst_publish_;
  SteadyClock::time_point manager_request_started_at_;
  SteadyClock::time_point manager_state_verification_started_at_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2GuardInterlock>());
  rclcpp::shutdown();
  return 0;
}
