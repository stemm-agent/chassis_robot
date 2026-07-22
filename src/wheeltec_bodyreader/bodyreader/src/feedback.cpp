#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdlib.h>
#include <ctime>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdio.h>
#include <bodyreader_msg/msg/bodyposture.hpp>

using namespace std;
int temp1 = 0;
int temp2 = 0;
int interaction = 0;
bool voice_feedback ;
bool gesture_voice_feedback = false;
static bool rear_collision_alarm_played = false;

// 标定完成音"只播一次"标志位
static bool cal_finish_played = false;   // akimibo 的标定完成音已播过

// 标定广播控制 (2026-06-11)
// first_cal_done: 首次标定完成后=true, 之后不再重复播完成音
// post_avoidance_cal_needed: 避障结束后需重新标定时=true, 允许再播一次完成音
static bool first_cal_done = false;
static bool post_avoidance_cal_needed = false;

void bodyposture_Callback(bodyreader_msg::msg::Bodyposture msg)
{
	if (voice_feedback)
	{
		if(msg.akimibo)
		{
			// 只在首次标定或避障后重标定时播放
			if ((!first_cal_done || post_avoidance_cal_needed) && !cal_finish_played) {
				system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/cal_finish.wav");
				cal_finish_played = true;
				first_cal_done = true;
				post_avoidance_cal_needed = false;
			}
		} else {
			cal_finish_played = false;  // akimibo 消失后重置，下次出现再播
		}

		if (msg.tips == 2)
		{
			system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/recovery.wav");
		}

		// Direction gesture prompts are disabled by default to avoid noisy body-follow audio.
                if(interaction==1 && gesture_voice_feedback){

			if(msg.left_foot_up == 1 && msg.right_foot_up == 0)
			{
				temp1++;
				temp2 = 0;
				if(temp1 > 5)
					{system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/backward.wav");
					temp1 = 0;}
			}
			else if(msg.left_foot_up == 0 && msg.right_foot_up == 1)
			{
				temp1++;
				temp2 = 0;
				if(temp1 > 5)
					{system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/forward.wav");
					temp1 = 0;}
			}
			else if(msg.left_hand_raised == 0 && msg.right_hand_raised == 1)
			{
				temp2++;
				temp1 = 0;
				if(temp2 > 5)
					{system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/turn_right.wav");
					temp2 = 0;}
			}
			else if(msg.left_hand_raised == 1 && msg.right_hand_raised == 0)
			{
				temp2++;
				temp1 = 0;
				if(temp2 > 5)
					{system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/turn_left.wav");
					temp2 = 0;}
			}
			else if(msg.left_arm_out == 1 && msg.right_arm_out == 0)
			{
					system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/left_translation.wav");
					temp1 = 0;
					temp2 = 0;
			}
			else if(msg.left_arm_out == 0 && msg.right_arm_out == 1)
			{
					system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/right_translation.wav");
					temp1 = 0;
					temp2 = 0;
			}
			else if (msg.fall == 1)
			{
					system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/audio/fall.wav");
					temp1 = 0;
					temp2 = 0;
			}

		}
	}
	
}


void mode_Callback(std_msgs::msg::Int8 msg)
{
	interaction = msg.data;
}

void avoidance_state_Callback(std_msgs::msg::String msg)
{
	if (msg.data == "recalibrating") {
		post_avoidance_cal_needed = true;
		cal_finish_played = false;
		RCLCPP_INFO(rclcpp::get_logger("bodyreader_feedback"),
			"Avoidance recalibrate: prompt voice disabled");
	}
}

void collision_state_Callback(std_msgs::msg::String msg)
{
	if (msg.data == "rear_collision_stop") {
		if (voice_feedback && !rear_collision_alarm_played) {
			system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/wheeltec_mic/wheeltec_mic_ros2/feedback_voice/stop.wav");
			rear_collision_alarm_played = true;
		}
		RCLCPP_WARN(rclcpp::get_logger("bodyreader_feedback"),
			"Rear collision detected, emergency stop latched");
	} else if (msg.data == "normal") {
		rear_collision_alarm_played = false;
	}
}



int main(int argc, char** argv)
{

  	rclcpp::init(argc, argv);
  	auto node = rclcpp::Node::make_shared("bodyreader_feedback");
  	node->declare_parameter<bool>("voice_feedback", true);
	node->declare_parameter<bool>("gesture_voice_feedback", false);
  	node->declare_parameter<int>("interaction", 0);
  	node->get_parameter("voice_feedback", voice_feedback);
	node->get_parameter("gesture_voice_feedback", gesture_voice_feedback);
  	node->get_parameter("interaction", interaction);
	
	//ros::Subscriber laser_follow_flag_sub = nd.subscribe("laser_follow_flag", 1, laser_follow_flagCallback);//雷达跟随开启标志位订阅


	//printf("interaction = %d\n",interaction);

	auto bodyposture_sub = node->create_subscription<bodyreader_msg::msg::Bodyposture>("/body_posture", 1, bodyposture_Callback);

	auto mode_sub = node->create_subscription<std_msgs::msg::Int8>("/mode", 1, mode_Callback);

	auto avoidance_state_sub = node->create_subscription<std_msgs::msg::String>(
		"/avoidance_state", 10, avoidance_state_Callback);

	auto collision_state_sub = node->create_subscription<std_msgs::msg::String>(
		"/collision_state", 10, collision_state_Callback);

	rclcpp::spin(node);



}
