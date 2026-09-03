/************************************************************************************************/
/* Copyright (c) 2023 WHEELTEC Technology, Inc   												*/
/* function:Command controller, command word recognition results into the corresponding action	*/
/* 功能：命令控制器，命令词识别结果转化为对应的执行动作													*/
/************************************************************************************************/
#include "command_recognition.h"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
using namespace std;
using std::placeholders::_1;

namespace
{
constexpr const char* VOICE_NAVIGATION_SESSION_PATH =
    "/home/wheeltec/.local/state/stemm-voice-target-navigation-session.json";

std::string json_escape(const std::string& value)
{
    std::ostringstream escaped;
    for (const char character : value)
    {
        switch (character)
        {
            case '\\': escaped << "\\\\"; break;
            case '"': escaped << "\\\""; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default: escaped << character; break;
        }
    }
    return escaped.str();
}

std::int64_t current_epoch_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

bool is_voice_navigation_active_state(const std::string& state)
{
    return state == "starting" || state == "navigating";
}

bool is_voice_navigation_terminal_state(const std::string& state)
{
    return state == "succeeded" || state == "failed" || state == "canceled";
}

std::string goal_uuid_to_hex(const rclcpp_action::GoalUUID& goal_uuid)
{
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const auto byte : goal_uuid)
    {
        output << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return output.str();
}

bool goal_uuid_from_hex(
    const std::string& text,
    rclcpp_action::GoalUUID* goal_uuid)
{
    if (goal_uuid == nullptr || text.size() != goal_uuid->size() * 2)
    {
        return false;
    }
    for (std::size_t index = 0; index < goal_uuid->size(); ++index)
    {
        const char high = text[index * 2];
        const char low = text[index * 2 + 1];
        if (!std::isxdigit(static_cast<unsigned char>(high)) ||
            !std::isxdigit(static_cast<unsigned char>(low)))
        {
            return false;
        }
        const std::string byte_text = text.substr(index * 2, 2);
        (*goal_uuid)[index] = static_cast<std::uint8_t>(
            std::stoul(byte_text, nullptr, 16));
    }
    return true;
}

std::string json_string_value(const std::string& body, const std::string& field_name)
{
    const std::string exact_prefix =
        std::string(1, '"') + field_name + std::string(1, '"') + ":" +
        std::string(1, '"');
    const std::string prefix = "\\\"" + field_name + "\\\":\\\"";
    const std::size_t begin = body.find(exact_prefix);
    if (begin == std::string::npos)
    {
        return "";
    }
    const std::size_t value_begin = begin + exact_prefix.size();
    const std::size_t value_end = body.find('"', value_begin);
    if (value_end == std::string::npos)
    {
        return "";
    }
    return body.substr(value_begin, value_end - value_begin);
}

bool json_boolean_value(
    const std::string& body,
    const std::string& field_name,
    bool* value)
{
    if (value == nullptr)
    {
        return false;
    }
    const std::string key = std::string(1, '"') + field_name + std::string(1, '"');
    const std::size_t key_begin = body.find(key);
    if (key_begin == std::string::npos)
    {
        return false;
    }
    const std::size_t colon = body.find(':', key_begin + key.size());
    if (colon == std::string::npos)
    {
        return false;
    }
    std::size_t value_begin = colon + 1;
    while (value_begin < body.size() &&
           std::isspace(static_cast<unsigned char>(body[value_begin])))
    {
        ++value_begin;
    }
    if (body.compare(value_begin, 4, "true") == 0)
    {
        *value = true;
        return true;
    }
    if (body.compare(value_begin, 5, "false") == 0)
    {
        *value = false;
        return true;
    }
    return false;
}

bool json_integer_value(
    const std::string& body,
    const std::string& field_name,
    int* value)
{
    if (value == nullptr)
    {
        return false;
    }
    const std::string key = std::string(1, '"') + field_name + std::string(1, '"');
    const std::size_t key_begin = body.find(key);
    if (key_begin == std::string::npos)
    {
        return false;
    }
    const std::size_t colon = body.find(':', key_begin + key.size());
    if (colon == std::string::npos)
    {
        return false;
    }
    std::size_t value_begin = colon + 1;
    while (value_begin < body.size() &&
           std::isspace(static_cast<unsigned char>(body[value_begin])))
    {
        ++value_begin;
    }
    std::size_t value_end = value_begin;
    while (value_end < body.size() &&
           std::isdigit(static_cast<unsigned char>(body[value_end])))
    {
        ++value_end;
    }
    if (value_end == value_begin)
    {
        return false;
    }
    try
    {
        *value = std::stoi(body.substr(value_begin, value_end - value_begin));
    }
    catch (const std::exception&)
    {
        return false;
    }
    return true;
}

bool write_voice_navigation_session_atomically(const std::string& payload)
{
    const std::string temporary_path = std::string(VOICE_NAVIGATION_SESSION_PATH) +
        ".tmp." + std::to_string(getpid());
    {
        std::ofstream output(temporary_path, std::ios::out | std::ios::trunc);
        if (!output.is_open())
        {
            return false;
        }
        output << payload;
        output.flush();
        if (!output.good())
        {
            output.close();
            std::remove(temporary_path.c_str());
            return false;
        }
    }
    if (std::rename(temporary_path.c_str(), VOICE_NAVIGATION_SESSION_PATH) != 0)
    {
        std::remove(temporary_path.c_str());
        return false;
    }
    return true;
}
}  // namespace

/**************************************************************************
函数功能：获取节点进程pid
返回  值：int pid
**************************************************************************/
int Command::node_kill(const char*  progress)
{
	char get_pid[32] = "pgrep -f ";
	strcat(get_pid,progress);
	FILE *fp = popen(get_pid,"r");
	if (fp == NULL)
	{
		printf("popen failed,get_pid = %s",get_pid);
		return -1;
	}
	
	char pid[16] = {0};
	fgets(pid,16,fp);
	if (strlen(pid) == 0)
	{
		pclose(fp);
		return -1;
	}
	pclose(fp);
	
	char cmd[32] = "kill -9 ";
	strcat(cmd,pid);
	system(cmd);
	return 0;
}

/**************************************************************************
函数功能：语音播放函数（带播报完成标志）
**************************************************************************/
void Command::play_audio_feedback_with_flag(const std::string& audio_file) {
    // 先重置状态，确保之前的完成标志被清除
    g_play_state::reset();
    
    // 发布开始消息
    g_play_state::start();

    // 执行播放
    std::string full_path = audio_path + audio_file;
    std::string command = "aplay " + full_path;
    int ret = system(command.c_str());

    // 发布完成消息
    g_play_state::finish();
    
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "播放音频失败: %s, 返回值: %d", 
                     full_path.c_str(), ret);
    }
}

/**************************************************************************
函数功能：语音播放函数
**************************************************************************/
void Command::play_audio_feedback(const std::string& audio_file) {
    if (device_type_ == "M07") {
        play_audio_feedback_with_flag(audio_file);
    } else {
        std::string command = "aplay " + audio_path + audio_file;
        system(command.c_str());
    }
}

/**************************************************************************
函数功能：请求反馈节点播放语音
**************************************************************************/
void Command::request_audio_feedback(const std::string& audio_file) {
	std_msgs::msg::String feedback_msg;
	feedback_msg.data = audio_file;
	feedback_audio_pub->publish(feedback_msg);
}

/**************************************************************************
函数功能：发送导航目标点
返回  值：无
**************************************************************************/
bool Command::publish_voice_navigation_status(
    const std::string& state,
    const std::string& message,
    const std::string& last_error,
    bool persist_session)
{
    const bool active = is_voice_navigation_active_state(state);
    const bool terminal = (
        !voice_navigation_recovery_required_ &&
        is_voice_navigation_terminal_state(state)
    );
    const bool interlock_active = active || voice_navigation_recovery_required_;
    std_msgs::msg::String status_msg;
    std::ostringstream payload;
    payload << "{\"schemaVersion\":1"
            << ",\"source\":\"voice_command_recognition\""
            << ",\"state\":\"" << json_escape(state) << "\""
            << ",\"active\":" << (active ? "true" : "false")
            << ",\"terminal\":" << (terminal ? "true" : "false")
            << ",\"interlockActive\":" << (interlock_active ? "true" : "false")
            << ",\"recoveryRequired\":"
            << (voice_navigation_recovery_required_ ? "true" : "false")
            << ",\"goalId\":\"" << json_escape(voice_navigation_goal_id_) << "\"";
    if (voice_navigation_goal_id_.empty())
    {
        payload << ",\"goal\":null";
    }
    else
    {
        payload << ",\"goal\":{\"frameId\":\"map\""
                << ",\"x\":" << voice_navigation_goal_x_
                << ",\"y\":" << voice_navigation_goal_y_
                << ",\"orientationZ\":" << voice_navigation_goal_orientation_z_
                << ",\"orientationW\":" << voice_navigation_goal_orientation_w_
                << ",\"label\":\"" << json_escape(voice_navigation_goal_label_) << "\"}";
    }
    payload
            // The mini-program does not expose a cancellation command yet.  A
            // later Nav2 cancellation is still reported by result_callback.
            << ",\"canCancel\":false"
            << ",\"actionUuid\":\""
            << (
                voice_navigation_action_uuid_valid_
                ? goal_uuid_to_hex(voice_navigation_action_uuid_)
                : ""
            )
            << "\""
            << ",\"message\":\"" << json_escape(message) << "\""
            << ",\"lastError\":\"" << json_escape(last_error) << "\""
            << ",\"updatedAtEpochMs\":" << current_epoch_ms()
            << "}";
    status_msg.data = payload.str();
    bool persisted = true;
    if (persist_session && !write_voice_navigation_session_atomically(status_msg.data))
    {
        persisted = false;
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to atomically persist voice target-navigation session");
    }
    if (voice_navigation_status_pub)
    {
        voice_navigation_status_pub->publish(status_msg);
    }
    return persisted;
}

bool Command::load_voice_navigation_recovery_session()
{
    std::ifstream input(VOICE_NAVIGATION_SESSION_PATH);
    if (!input.is_open())
    {
        return false;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    const std::string session = contents.str();
    bool active = false;
    bool terminal = false;
    int schema_version = 0;
    const bool valid_session =
        json_integer_value(session, "schemaVersion", &schema_version) &&
        schema_version == 1 &&
        json_string_value(session, "source") == "voice_command_recognition" &&
        json_boolean_value(session, "active", &active) &&
        json_boolean_value(session, "terminal", &terminal);
    if (!valid_session || (!active && !terminal))
    {
        // A corrupt or internally inconsistent non-terminal file is not proof
        // that a historical Nav2 goal stopped.  Hold the voice target interlock
        // until navigation is explicitly and safely shut down.
        voice_navigation_recovery_required_ = true;
        voice_navigation_recovery_cancel_requested_ = false;
        voice_navigation_action_uuid_valid_ = false;
        RCLCPP_ERROR(
            this->get_logger(),
            "Persisted voice target-navigation session is invalid; recovery is locked");
        return true;
    }
    if (!active)
    {
        return false;
    }
    const std::string action_uuid_text = json_string_value(session, "actionUuid");
    voice_navigation_action_uuid_valid_ = goal_uuid_from_hex(
        action_uuid_text, &voice_navigation_action_uuid_);
    voice_navigation_recovery_required_ = true;
    voice_navigation_recovery_cancel_requested_ = false;
    return true;
}

bool Command::persisted_voice_navigation_session_is_active() const
{
    std::ifstream input(VOICE_NAVIGATION_SESSION_PATH);
    if (!input.is_open())
    {
        // A missing file is not proof that a previously recovered action ended.
        return true;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    const std::string session = contents.str();
    bool active = false;
    bool terminal = false;
    int schema_version = 0;
    const bool valid_session =
        json_integer_value(session, "schemaVersion", &schema_version) &&
        schema_version == 1 &&
        json_string_value(session, "source") == "voice_command_recognition" &&
        json_boolean_value(session, "active", &active) &&
        json_boolean_value(session, "terminal", &terminal);
    // Treat malformed or non-terminal-but-inactive content as unresolved.
    return !valid_session || active || !terminal;
}

void Command::try_recover_persisted_voice_navigation()
{
    if (!voice_navigation_recovery_required_)
    {
        return;
    }
    if (!persisted_voice_navigation_session_is_active())
    {
        voice_navigation_recovery_required_ = false;
        voice_navigation_recovery_cancel_requested_ = false;
        voice_navigation_action_uuid_valid_ = false;
        voice_navigation_goal_id_.clear();
        voice_navigation_goal_label_.clear();
        publish_voice_navigation_status(
            "canceled",
            "Voice target-navigation recovery was cleared after confirmed navigation shutdown.",
            "",
            false);
        return;
    }
    if (
        voice_navigation_recovery_cancel_requested_ ||
        !voice_navigation_action_uuid_valid_ ||
        !voice_navigation_cancel_client_ ||
        !voice_navigation_cancel_client_->service_is_ready()
    )
    {
        return;
    }
    try
    {
        auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
        request->goal_info.goal_id.uuid = voice_navigation_action_uuid_;
        voice_navigation_cancel_client_->async_send_request(request);
        voice_navigation_recovery_cancel_requested_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "Requested cancellation for the exact persisted voice target goal");
    }
    catch (const std::exception& exception)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not request cancellation for persisted voice target goal: %s",
            exception.what());
    }
}

void Command::voice_navigation_goal_status_Callback(
    const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    if (!voice_navigation_recovery_required_ || !voice_navigation_action_uuid_valid_)
    {
        return;
    }
    for (const auto& entry : msg->status_list)
    {
        if (!std::equal(
                entry.goal_info.goal_id.uuid.begin(),
                entry.goal_info.goal_id.uuid.end(),
                voice_navigation_action_uuid_.begin()))
        {
            continue;
        }
        std::string terminal_state;
        if (entry.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
        {
            terminal_state = "succeeded";
        }
        else if (entry.status == action_msgs::msg::GoalStatus::STATUS_ABORTED)
        {
            terminal_state = "failed";
        }
        else if (entry.status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
        {
            terminal_state = "canceled";
        }
        if (terminal_state.empty())
        {
            return;
        }
        voice_navigation_recovery_required_ = false;
        voice_navigation_recovery_cancel_requested_ = false;
        voice_navigation_active_ = false;
        publish_voice_navigation_status(
            terminal_state,
            "Persisted voice target-navigation recovery reached a terminal action state.",
            terminal_state == "failed"
                ? "Recovered voice target-navigation action was aborted."
                : "");
        // Persist terminal provenance before releasing the in-memory identity.
        voice_navigation_action_uuid_valid_ = false;
        voice_navigation_goal_handle_.reset();
        voice_navigation_goal_id_.clear();
        voice_navigation_goal_label_.clear();
        return;
    }
}

void Command::cancel_voice_navigation_for_mapping()
{
    if (!voice_navigation_active_)
    {
        return;
    }
    voice_navigation_cancel_requested_ = true;
    publish_voice_navigation_status(
        "navigating",
        "Cartographer mapping took motion ownership; voice target cancellation was requested.",
        "");
    if (!goal_client || !voice_navigation_goal_handle_)
    {
        return;
    }
    try
    {
        goal_client->async_cancel_goal(voice_navigation_goal_handle_);
    }
    catch (const std::exception& exception)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to request cancellation for voice target during mapping takeover: %s",
            exception.what());
    }
}

void Command::publish_voice_navigation_heartbeat()
{
    if (voice_navigation_recovery_required_)
    {
        try_recover_persisted_voice_navigation();
        if (voice_navigation_recovery_required_)
        {
            // Do not overwrite the unresolved atomic session while its exact
            // action UUID is being cancelled or awaits manual Nav2 shutdown.
            publish_voice_navigation_status(
                "unknown",
                "A prior voice target-navigation session is being recovered.",
                "Voice target navigation is locked until the persisted session reaches a terminal state.",
                false);
        }
        return;
    }
    if (voice_navigation_active_)
    {
        publish_voice_navigation_status(
            "navigating", "Voice target navigation is in progress.", "", false);
    }
    else if (voice_navigation_goal_id_.empty() && voice_navigation_goal_label_.empty())
    {
        // Keep the retained idle snapshot fresh without overwriting a
        // succeeded/failed/canceled target-navigation terminal state.
        publish_voice_navigation_status(
            "unknown", "Voice target navigation has not been requested in this node session.", "", false);
    }
}

bool Command::send_goal(const vector<float>& msg, const std::string& label)
{
    if (voice_navigation_recovery_required_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Reject voice target goal while a persisted prior target is unresolved");
        publish_voice_navigation_status(
            "unknown",
            "Voice target navigation is locked pending recovery of a prior goal.",
            "A prior voice target-navigation session has not reached a terminal state.",
            false);
        return false;
    }
    if (voice_navigation_active_ || voice_navigation_cancel_requested_)
    {
        publish_voice_navigation_status(
            voice_navigation_goal_handle_ ? "navigating" : "starting",
            "A prior voice target-navigation request is still being resolved.",
            "A new voice target cannot replace an unfinished target-navigation request.",
            false);
        return false;
    }
    if (msg.size() < 4)
    {
        RCLCPP_ERROR(this->get_logger(), "Voice target goal is incomplete");
        voice_navigation_active_ = false;
        voice_navigation_goal_id_.clear();
        voice_navigation_goal_label_ = label;
        publish_voice_navigation_status(
            "failed", "Voice target navigation request is invalid.",
            "Voice target goal is incomplete.");
        return false;
    }

    const std::uint64_t generation = ++voice_navigation_generation_;
    active_voice_navigation_generation_ = generation;
    voice_navigation_active_ = true;
    voice_navigation_cancel_requested_ = false;
    voice_navigation_goal_handle_.reset();
    voice_navigation_action_uuid_valid_ = false;
    voice_navigation_goal_id_ = "voice-target-" +
        std::to_string(current_epoch_ms()) + "-" + std::to_string(generation);
    voice_navigation_goal_label_ = label;
    voice_navigation_goal_x_ = msg.at(0);
    voice_navigation_goal_y_ = msg.at(1);
    voice_navigation_goal_orientation_z_ = msg.at(2);
    voice_navigation_goal_orientation_w_ = msg.at(3);
    if (!publish_voice_navigation_status(
            "starting", "Voice target navigation request received.", ""))
    {
        // Never send a Nav2 goal which cannot survive a process restart as a
        // safely attributable, cancelable voice-target session.
        voice_navigation_active_ = false;
        publish_voice_navigation_status(
            "failed",
            "Voice target navigation was not started.",
            "The target-navigation recovery session could not be persisted.",
            false);
        return false;
    }

	if (mapping_motion_blocked_)
	{
		RCLCPP_WARN(this->get_logger(),
			"Reject normal navigation goal while Cartographer mapping owns motion");
        voice_navigation_active_ = false;
        publish_voice_navigation_status(
            "failed", "Voice target navigation was rejected.",
            "Cartographer mapping currently owns vehicle motion.");
		return false;
	}
	if (!goal_client || !goal_client->wait_for_action_server(std::chrono::seconds(3)))
	{
		RCLCPP_ERROR(this->get_logger(), "Action server not available!");
        voice_navigation_active_ = false;
        publish_voice_navigation_status(
            "failed", "Voice target navigation was not started.",
            "Navigation action server is not available.");
        return false;
    }

	auto goal = ClientT::Goal();
	goal.pose.header.stamp = this->now();
	goal.pose.header.frame_id = "map";
	goal.pose.pose.position.x = msg.at(0);
	goal.pose.pose.position.y = msg.at(1);
	goal.pose.pose.orientation.z = msg.at(2);
	goal.pose.pose.orientation.w = msg.at(3);

	RCLCPP_INFO(this->get_logger(), "Sending goal");

	auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
	send_goal_options.goal_response_callback =
        [this, generation](GoalHandle_::SharedPtr goal_handle)
        {
            this->goal_response_callback(goal_handle, generation);
        };
	send_goal_options.feedback_callback =
        [this, generation](GoalHandle_::SharedPtr goal_handle,
                           const std::shared_ptr<const ClientT::Feedback> feedback)
        {
            this->feedback_callback(goal_handle, feedback, generation);
        };
	send_goal_options.result_callback =
        [this, generation](const GoalHandle_::WrappedResult& result)
        {
            this->result_callback(result, generation);
        };

    try
    {
	    goal_client->async_send_goal(goal,send_goal_options);
    }
    catch (const std::exception& exception)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send voice target goal: %s",
            exception.what());
        if (generation == active_voice_navigation_generation_)
        {
            voice_navigation_active_ = false;
            publish_voice_navigation_status(
                "failed", "Voice target navigation was not started.",
                "Sending the navigation goal failed.");
        }
        return false;
    }
    return true;
}
/**************************************************************************
函数功能：动作目标响应回调函数
返回  值：无
**************************************************************************/
void Command::goal_response_callback(
    GoalHandle_::SharedPtr goal_handle,
    std::uint64_t generation)
{
	if (generation != active_voice_navigation_generation_)
	{
		return;
	}
	if (!goal_handle)
	{
		RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        voice_navigation_active_ = false;
        voice_navigation_goal_handle_.reset();
        publish_voice_navigation_status(
            "failed", "Voice target navigation was rejected.",
            "Navigation goal was rejected by the action server.");
	}
	else
    {
        voice_navigation_goal_handle_ = goal_handle;
        voice_navigation_action_uuid_ = goal_handle->get_goal_id();
        voice_navigation_action_uuid_valid_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        if (mapping_motion_blocked_ || voice_navigation_cancel_requested_)
        {
            cancel_voice_navigation_for_mapping();
            return;
        }
        publish_voice_navigation_status(
            "navigating", "Voice target navigation was accepted.", "");
    }
}

/**************************************************************************
函数功能：动作反馈信息处理回调函数
返回  值：无
**************************************************************************/
void Command::feedback_callback(
    GoalHandle_::SharedPtr,
    const std::shared_ptr<const ClientT::Feedback> feedback,
    std::uint64_t generation)
{
	// RCLCPP_INFO(this->get_logger(),"Remaining Distance from Destination: %f",feedback->distance_remaining);
    if (generation == active_voice_navigation_generation_ && voice_navigation_active_)
    {
        publish_voice_navigation_status(
            "navigating", "Voice target navigation feedback received.", "");
    }
}

/**************************************************************************
函数功能：动作结果回调函数
返回  值：无
**************************************************************************/
void Command::result_callback(
    const GoalHandle_::WrappedResult &result,
    std::uint64_t generation)
{
	if (generation != active_voice_navigation_generation_)
	{
		return;
    }
    voice_navigation_active_ = false;
    voice_navigation_cancel_requested_ = false;
    voice_navigation_goal_handle_.reset();
	switch(result.code)
	{
		case rclcpp_action::ResultCode::SUCCEEDED:
			request_audio_feedback("/reach_goal.wav");
			publish_voice_navigation_status(
                "succeeded", "Voice target navigation succeeded.", "");
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
			publish_voice_navigation_status(
                "failed", "Voice target navigation failed.",
                "Navigation goal was aborted.");
			return;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
			publish_voice_navigation_status(
                "canceled", "Voice target navigation was canceled.",
                "");
			return;
		default:
        	RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			publish_voice_navigation_status(
                "failed", "Voice target navigation failed.",
                "Navigation goal returned an unknown result.");
        	return;
	}
	RCLCPP_INFO(this->get_logger(), "Result received");
}

/**************************************************************************
函数功能：寻找语音开启成功标志位sub回调函数
入口参数：voice_flag_msg  voice_control.cpp
返回  值：无
**************************************************************************/
void Command::voice_flag_Callback(std_msgs::msg::Int8::SharedPtr msg){
	voice_flag = msg->data;
    if (voice_flag){
		request_audio_feedback("/voice_control.wav");
        cout<<"语音打开成功"<<endl;
    }
}

void Command::mapping_motion_blocked_Callback(std_msgs::msg::Bool::SharedPtr msg)
{
	const bool blocked = msg->data;
	if (blocked && !mapping_motion_blocked_)
	{
		mapping_motion_blocked_ = true;
		car_move(Car_Status::STOP);
		cancel_voice_navigation_for_mapping();
		RCLCPP_WARN(this->get_logger(),
			"Cartographer mapping gate active; manual motion and normal voice targets are disabled");
		return;
	}
	mapping_motion_blocked_ = blocked;
}

/**************************************************************************
函数功能：寻找语音开启成功标志位sub回调函数
入口参数：status 五个小车基础运动状态
返回  值：无
**************************************************************************/
void Command::car_move(Car_Status status){
	motion_.linear_x = 0;
	motion_.angular_z = 0;
	motion_.cmd_vel_flag =1;
	motion_.follow_flag = 0;

	switch (status) {
		case Car_Status::FRONT:
			motion_.linear_x = line_vel_x;
			break;
		case Car_Status::BACK:
			motion_.linear_x = -line_vel_x;
			break;
		case Car_Status::LEFT:
			motion_.linear_x = turn_line_vel_x;
			motion_.angular_z = ang_vel_z;
			break;
		case Car_Status::RIGHT:
			motion_.linear_x = turn_line_vel_x;
			motion_.angular_z = -ang_vel_z;
			break;
		case Car_Status::STOP:
			break;
    }

    motion_msg_pub->publish(motion_);
}


/**************************************************************************
函数功能：离线命令词识别结果sub回调函数
入口参数：命令词字符串
返回  值：无
**************************************************************************/
void Command::voice_words_Callback(std_msgs::msg::String::SharedPtr msg){
	/***语音指令***/
	string str1 = msg->data;    //取传入数据
	string str2 = "小车前进";
	string str3 = "小车后退"; 
	string str4 = "小车左转";
	string str5 = "小车右转";
	string str6 = "小车停";
	string str7 = "小车休眠";
	string str8 = "小车过来";
	string str9 = "小车去I点";
	string str10 = "小车去J点";
	string str11 = "小车去K点";
	string str12 = "失败5次";
	string str13 = "失败10次";
	string str14 = "遇到障碍物";
	string str15 = "小车唤醒";
	string str16 = "小车雷达跟随";
	string str17 = "关闭雷达跟随";
	const bool requests_normal_target =
		str1 == str9 || str1 == str10 || str1 == str11;
	const bool requests_motion =
		str1 == str2 || str1 == str3 || str1 == str4 || str1 == str5 ||
		str1 == str8 || str1 == str9 || str1 == str10 || str1 == str11 ||
		str1 == str16;
	if (mapping_motion_blocked_ && requests_motion && !requests_normal_target)
	{
		RCLCPP_WARN(this->get_logger(),
			"Reject voice motion command during Cartographer mapping: %s",
			str1.c_str());
		request_audio_feedback("/stop.wav");
		return;
	}
/***********************************
指令：小车前进
动作：底盘运动控制器使能，发布速度指令
***********************************/
	if (str1 == str2){
		car_move(Car_Status::FRONT);
		request_audio_feedback("/car_front.wav");
		cout<<"好的：小车前进"<<endl;
	}
/***********************************
指令：小车后退
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if (str1 == str3){
		car_move(Car_Status::BACK);
		request_audio_feedback("/car_back.wav");
		cout<<"好的：小车后退"<<endl;
	}
/***********************************
指令：小车左转
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if (str1 == str4){
		car_move(Car_Status::LEFT);
		request_audio_feedback("/turn_left.wav");
		cout<<"好的：小车左转"<<endl;
	}
/***********************************
指令：小车右转
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if (str1 == str5){
		car_move(Car_Status::RIGHT);
		request_audio_feedback("/turn_right.wav");
		cout<<"好的：小车右转"<<endl;
	}
/***********************************
指令：小车停
动作：底盘运动控制器失能，发布速度空指令
***********************************/
	else if (str1 == str6){
		car_move(Car_Status::STOP);
		request_audio_feedback("/stop.wav");
		cout<<"好的：小车停"<<endl;
	}
/***********************************************
指令：小车休眠
动作：底盘运动控制器失能，发布速度空指令，唤醒标志位置零
***********************************************/
	else if (str1 == str7){
		std_msgs::msg::Int8 awake_flag_msg;
		awake_flag_msg.data = 0;
		awake_flag_pub->publish(awake_flag_msg);

		request_audio_feedback("/sleep.wav");
		cout<<"小车休眠，等待下一次唤醒"<<endl;
	}
/***********************************
指令：小车过来
动作：交由 voice_summon_coordinator 处理，禁用旧声源跟随入口
***********************************/
        else if (str1 == str8){
                // Relinquish any command left active in motion_control. The visual
                // summon flow itself remains owned by voice_summon_coordinator.
                motion_.linear_x = 0;
                motion_.angular_z = 0;
                motion_.cmd_vel_flag = 0;
                motion_.follow_flag = 0;
                motion_msg_pub->publish(motion_);
                /*
                 * Legacy voice-follow path disabled.
                 * The new summon flow is handled only by voice_summon_coordinator:
                 *   1. turn toward awake_angle
                 *   2. publish /mode=2
                 *   3. let bodyfollow_nav2 keep visual recognition/following
                 *
                 * Old logic kept here as a comment to make the mutual exclusion explicit:
                 *   motion_.linear_x = 0;
                 *   motion_.angular_z = 0;
                 *   motion_.cmd_vel_flag = 0;
                 *   motion_.follow_flag = 1;
                 *   motion_msg_pub->publish(motion_);
                 *   request_audio_feedback("/search_voice.wav");
                 */
                cout<<"好的：语音召唤交由voice_summon_coordinator处理"<<endl;
        }
/***********************************
指令：小车去I点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if (str1 == str9){
		geometry_msgs::msg::PoseStamped point_I;
		if (!point.empty()) point.clear();
		point.push_back(I_position_x);
		point.push_back(I_position_y);
		point.push_back(I_orientation_z);
		point.push_back(I_orientation_w);
		if (send_goal(point, "I"))
		{
			request_audio_feedback("/OK.wav");
			cout<<"好的：小车自主导航至I点"<<endl;
		}
		else
		{
			request_audio_feedback("/stop.wav");
			cout<<"小车未能启动至I点的自主导航"<<endl;
		}
	}
/***********************************
指令：小车去J点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if (str1 == str10){
		if (!point.empty()) point.clear();
		point.push_back(J_position_x);
		point.push_back(J_position_y);
		point.push_back(J_orientation_z);
		point.push_back(J_orientation_w);
		if (send_goal(point, "J"))
		{
			request_audio_feedback("/OK.wav");
			cout<<"好的：小车自主导航至J点"<<endl;
		}
		else
		{
			request_audio_feedback("/stop.wav");
			cout<<"小车未能启动至J点的自主导航"<<endl;
		}
	}
/***********************************
指令：小车去K点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if (str1 == str11){
		if (!point.empty()) point.clear();
		point.push_back(K_position_x);
		point.push_back(K_position_y);
		point.push_back(K_orientation_z);
		point.push_back(K_orientation_w);
		if (send_goal(point, "K"))
		{
			request_audio_feedback("/OK.wav");
			cout<<"好的：小车自主导航至K点"<<endl;
		}
		else
		{
			request_audio_feedback("/stop.wav");
			cout<<"小车未能启动至K点的自主导航"<<endl;
		}
	}
	else if (str1 == str12){
		cout<<"您已经连续【输入空指令or识别失败】5次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
	else if (str1 == str13){
		cout<<"您已经连续【输入空指令or识别失败】10次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
/***********************************
辅助指令：遇到障碍物
动作：用户界面打印提醒
***********************************/
	else if (str1 == str14){
		request_audio_feedback("/Tracker.wav");
		cout<<"小车遇到障碍物，已停止运动"<<endl;
	}
/***********************************
辅助指令：小车唤醒
动作：用户界面打印提醒
***********************************/
	else if (str1 == str15){
		// call_recognition owns the generation-tagged awake feedback handshake.
		// Keeping playback here would create a second awake tone and reopen the
		// microphone before the correct tone has completed.
		cout<<"小车已被唤醒，等待唤醒反馈音完成"<<endl;
	}
/***********************************
辅助指令：小车雷达跟随
动作：用户界面打印提醒并开启节点
***********************************/
	else if (str1 == str16 && sw == "on"){
		sw = "off";
		request_audio_feedback("/OK.wav");

		std_msgs::msg::Int8 laser_follow_flag_msg;
		laser_follow_flag_msg.data = 0;
		laser_follow_flag_pub->publish(laser_follow_flag_msg);

		Launch = gnome_terminal + wheeltec_mic_ros2 + "laserfollower.launch.py";
		system(Launch.c_str());
		cout<<"好的：小车雷达跟随"<<endl;
	}
/***********************************
辅助指令：关闭雷达跟随
动作：用户界面打印提醒并关闭节点
***********************************/
	else if (str1 == str17 && sw == "off"){
		sw = "on";
		request_audio_feedback("/OK.wav");
		cout<<"好的：关闭雷达跟随"<<endl;
	}
}

Command::Command(const std::string &node_name,
	const rclcpp::NodeOptions &options)
: rclcpp::Node(node_name,options){
	RCLCPP_INFO(this->get_logger(),"%s node init!\n",node_name.c_str());
	/***声明参数并获取***/
	this->declare_parameter<string>("audio_path","");
	this->declare_parameter<std::string>("device_type", "default");
	this->declare_parameter<bool>("if_akm_yes_or_no",false);
	this->declare_parameter<float>("line_vel_x",0.2);
	this->declare_parameter<float>("ang_vel_z",0.2);
	this->declare_parameter<float>("I_position_x",1);
	this->declare_parameter<float>("I_position_y",0);
	this->declare_parameter<float>("I_orientation_z",0);
	this->declare_parameter<float>("I_orientation_w",1);
	this->declare_parameter<float>("J_position_x",1);
	this->declare_parameter<float>("J_position_y",0);
	this->declare_parameter<float>("J_orientation_z",0);
	this->declare_parameter<float>("J_orientation_w",1);
	this->declare_parameter<float>("K_position_x",1);
	this->declare_parameter<float>("K_position_y",0);
	this->declare_parameter<float>("K_orientation_z",0);
	this->declare_parameter<float>("K_orientation_w",1);
	this->get_parameter("audio_path",audio_path);
	this->get_parameter("device_type", device_type_);
	this->get_parameter("line_vel_x",line_vel_x);
	this->get_parameter("ang_vel_z",ang_vel_z);
	this->get_parameter("if_akm_yes_or_no",if_akm);
	this->get_parameter("I_position_x",I_position_x);
	this->get_parameter("I_position_y",I_position_y);
	this->get_parameter("I_orientation_z",I_orientation_z);
	this->get_parameter("I_orientation_w",I_orientation_w);
	this->get_parameter("J_position_x",J_position_x);
	this->get_parameter("J_position_y",J_position_y);
	this->get_parameter("J_orientation_z",J_orientation_z);
	this->get_parameter("J_orientation_w",J_orientation_w);
	this->get_parameter("K_position_x",K_position_x);
	this->get_parameter("K_position_y",K_position_y);
	this->get_parameter("K_orientation_z",K_orientation_z);
	this->get_parameter("K_orientation_w",K_orientation_w);

	/***唤醒标志位话题发布者创建***/
	awake_flag_pub = this->create_publisher<std_msgs::msg::Int8>("awake_flag",10); 
	/***雷达跟随标志位话题发布者创建***/
	laser_follow_flag_pub = this->create_publisher<std_msgs::msg::Int8>("laser_follow_flag",10); 
	/***反馈播报请求话题发布者创建***/
	feedback_audio_pub = this->create_publisher<std_msgs::msg::String>("feedback_audio",10);
	/***普通语音 I/J/K 目标导航状态发布者（非通用 Nav2 观测）***/
	rclcpp::QoS voice_navigation_status_qos(rclcpp::KeepLast(1));
	voice_navigation_status_qos.reliable();
	voice_navigation_status_qos.transient_local();
	voice_navigation_status_pub = this->create_publisher<std_msgs::msg::String>(
		"/stemm_voice_navigation/status", voice_navigation_status_qos);
	/***底盘运动信息话题发布者创建***/
	motion_msg_pub = this->create_publisher<wheeltec_mic_msg::msg::MotionControl>("motion_msg",10);
	/***导航点位置话题发布者创建***/
	pose_nav_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose",10);
	/***导航点动作客户端创建***/
	goal_client =  rclcpp_action::create_client<ClientT>(this,"navigate_to_pose");
	voice_navigation_cancel_client_ =
		this->create_client<action_msgs::srv::CancelGoal>(
			"navigate_to_pose/_action/cancel_goal");
	// Observe only the exact persisted I/J/K action UUID during restart
	// recovery; other NavigateToPose users must never be attributed to voice.
	voice_navigation_goal_status_sub =
		this->create_subscription<action_msgs::msg::GoalStatusArray>(
			"/navigate_to_pose/_action/status", 10,
			std::bind(&Command::voice_navigation_goal_status_Callback, this, _1));
	const bool recovering_voice_target = load_voice_navigation_recovery_session();
	voice_navigation_status_timer_ = this->create_wall_timer(
		std::chrono::seconds(1),
		std::bind(&Command::publish_voice_navigation_heartbeat, this));
	// Do not overwrite a non-terminal atomic session with a fabricated idle
	// state: its exact action may still be executing after this node restarts.
	if (recovering_voice_target)
	{
		publish_voice_navigation_status(
			"unknown",
			"A prior voice target-navigation session is being recovered.",
			"Voice target navigation is locked until the persisted session reaches a terminal state.",
			false);
	}
	else
	{
		publish_voice_navigation_status(
			"unknown", "Voice target navigation has not been requested in this node session.", "", false);
	}
	/***离线命令词识别结果话题订阅者创建***/
	voice_words_sub = this->create_subscription<std_msgs::msg::String>(
		"voice_words",10,std::bind(&Command::voice_words_Callback,this,_1));
	rclcpp::QoS mapping_gate_qos(rclcpp::KeepLast(1));
	mapping_gate_qos.reliable();
	mapping_gate_qos.transient_local();
	mapping_motion_blocked_sub = this->create_subscription<std_msgs::msg::Bool>(
		"/stemm_cartographer/motion_blocked", mapping_gate_qos,
		std::bind(&Command::mapping_motion_blocked_Callback,this,_1));
	/***寻找语音开启标志位话题订阅者创建***/
	voice_flag_sub = this->create_subscription<std_msgs::msg::Int8>(
		"voice_flag",10,std::bind(&Command::voice_flag_Callback,this,_1));

	g_play_state::init();

	if (if_akm) turn_line_vel_x = 0.2;
	else turn_line_vel_x = 0;

	sleep(8);
	cout<<"您可以语音控制啦!"<<endl;
	cout<<"小车前进———————————>向前"<<endl;
	cout<<"小车后退———————————>后退"<<endl;
	cout<<"小车左转———————————>左转"<<endl;
	cout<<"小车右转———————————>右转"<<endl;
	cout<<"小车停———————————>停止"<<endl;
	cout<<"小车休眠———————————>休眠，等待下一次唤醒"<<endl;
	cout<<"小车过来———————————>语音召唤"<<endl;
	cout<<"小车去I点———————————>小车自主导航至I点"<<endl;
	cout<<"小车去J点———————————>小车自主导航至J点"<<endl;
	cout<<"小车去K点———————————>小车自主导航至K点"<<endl;
	cout<<"小车雷达跟随———————————>小车打开雷达跟随"<<endl;
	cout<<"关闭雷达跟随———————————>小车关闭雷达跟随"<<endl;
}

void Command::run(){
	rclcpp::spin(shared_from_this());
}

Command::~Command(){
	RCLCPP_INFO(this->get_logger(),"command_recognition_node over!\n");
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc,argv);
	auto node = std::make_shared<Command>("command_recognition",rclcpp::NodeOptions());
	rclcpp::spin(node);  
	rclcpp::shutdown();
	return 0;
}
