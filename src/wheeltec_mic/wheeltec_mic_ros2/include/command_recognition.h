#ifndef __CALL_COMMAND_RECOGNITION_H_
#define __CALL_COMMAND_RECOGNITION_H_

#include <iostream>
#include <vector>
#include <array>
#include <cstdint>
#include <unistd.h>
#include <inttypes.h>
#include <play_path.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "wheeltec_mic_msg/msg/motion_control.hpp"
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
enum class Car_Status {FRONT, BACK, LEFT, RIGHT, STOP};
class Command : public rclcpp::Node{
public:
	using ClientT = nav2_msgs::action::NavigateToPose;
	using GoalHandle_ = rclcpp_action::ClientGoalHandle<ClientT>;
	Command(const std::string &node_name,
         const rclcpp::NodeOptions &options);
	~Command();
	void run();

private:
	int voice_flag = 0;
	bool if_akm;
	bool mapping_motion_blocked_{false};
	float line_vel_x,ang_vel_z,turn_line_vel_x;
	float I_position_x,I_position_y,I_orientation_z,I_orientation_w;
	float J_position_x,J_position_y,J_orientation_z,J_orientation_w;
	float K_position_x,K_position_y,K_orientation_z,K_orientation_w;
	vector<float> point;
	string sw = "on";
	
	wheeltec_mic_msg::msg::MotionControl motion_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr awake_flag_pub,laser_follow_flag_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_audio_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_navigation_status_pub;
    rclcpp::Publisher<wheeltec_mic_msg::msg::MotionControl>::SharedPtr motion_msg_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_nav_pub;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr voice_flag_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_words_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mapping_motion_blocked_sub;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr voice_navigation_goal_status_sub;

    rclcpp_action::Client<ClientT>::SharedPtr goal_client;
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr voice_navigation_cancel_client_;
    rclcpp::TimerBase::SharedPtr voice_navigation_status_timer_;
    std::uint64_t voice_navigation_generation_{0};
    std::uint64_t active_voice_navigation_generation_{0};
    bool voice_navigation_active_{false};
    bool voice_navigation_cancel_requested_{false};
    bool voice_navigation_recovery_required_{false};
    bool voice_navigation_recovery_cancel_requested_{false};
    bool voice_navigation_action_uuid_valid_{false};
    GoalHandle_::SharedPtr voice_navigation_goal_handle_;
    rclcpp_action::GoalUUID voice_navigation_action_uuid_{};
    std::string voice_navigation_goal_id_;
    std::string voice_navigation_goal_label_;
    float voice_navigation_goal_x_{0.0F};
    float voice_navigation_goal_y_{0.0F};
    float voice_navigation_goal_orientation_z_{0.0F};
    float voice_navigation_goal_orientation_w_{1.0F};

    int node_kill(const char* progress);

    void voice_flag_Callback(const std_msgs::msg::Int8::SharedPtr msg);
    void voice_words_Callback(const std_msgs::msg::String::SharedPtr msg);
    void mapping_motion_blocked_Callback(const std_msgs::msg::Bool::SharedPtr msg);
	void voice_navigation_goal_status_Callback(
        const action_msgs::msg::GoalStatusArray::SharedPtr msg);
	void car_move(Car_Status status);
    bool send_goal(const vector<float>& msg, const std::string& label);
    void cancel_voice_navigation_for_mapping();
    void try_recover_persisted_voice_navigation();
    bool load_voice_navigation_recovery_session();
    bool persisted_voice_navigation_session_is_active() const;
    void goal_response_callback(
        GoalHandle_::SharedPtr goal_handle,
        std::uint64_t generation);
    void feedback_callback(
        GoalHandle_::SharedPtr,
        const std::shared_ptr<const ClientT::Feedback> feedback,
        std::uint64_t generation);
    void result_callback(
        const GoalHandle_::WrappedResult &result,
        std::uint64_t generation);
    bool publish_voice_navigation_status(
        const std::string& state,
        const std::string& message,
        const std::string& last_error,
        bool persist_session = true);
    void publish_voice_navigation_heartbeat();
	void request_audio_feedback(const std::string& audio_file);
    void play_audio_feedback(const std::string& audio_file);
    void play_audio_feedback_with_flag(const std::string& audio_file);
	
};

#endif
