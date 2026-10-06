/****************************************************************/
/* Copyright (c) 2023 WHEELTEC Technology, Inc   				*/
/* function:Functional node feedback							*/
/* 功能：功能节点反馈												*/
/****************************************************************/
#include "node_feedback.h"
/**************************************************************************
函数功能：获取节点进程pid
返回  值：int pid
**************************************************************************/
int Feedback::node_kill(const char*  progress)
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
函数功能：语音播放函数（带播报完成标志)
**************************************************************************/
void play_audio_feedback_with_flag(const char* audio_file) {
    // 先重置状态
    g_play_state::reset();
    
    // 开始播报
    g_play_state::start();
    
    // 执行播放
    std::unique_ptr<char[]> command(
        audio_utils::join_smart((head + audio_path), audio_file));
    if (command) {
        system(command.get());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("node_feedback"), 
                    "Failed to create audio command for: %s", audio_file);
    }
    
    // 播报完成
    g_play_state::finish();
}

/**************************************************************************
函数功能：语音播放函数
**************************************************************************/
void play_audio_feedback(const char* audio_file) {
    if (device_type_ == "M07") {
        play_audio_feedback_with_flag(audio_file);
    } else {
        std::unique_ptr<char[]> command(
            audio_utils::join_smart((head + audio_path), audio_file));
        if (command) {
            system(command.get());
        }
    }
}

/**************************************************************************
函数功能：反馈播报请求回调函数
入口参数：音频文件名
返回  值：无
**************************************************************************/
void Feedback::feedback_audio_Callback(const std_msgs::msg::String::SharedPtr msg){
	if (!msg->data.empty()) {
		auto now = this->now();
		if (msg->data == last_feedback_audio_ &&
			(now - last_feedback_time_).seconds() < 1.0) {
			RCLCPP_WARN(this->get_logger(), "Ignore duplicate feedback audio: %s", msg->data.c_str());
			return;
		}
		last_feedback_audio_ = msg->data;
		last_feedback_time_ = now;
		play_audio_feedback(msg->data.c_str());
	}
}

void Feedback::wake_feedback_request_Callback(
	const std_msgs::msg::UInt64::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(),
		"Play awake feedback for generation %llu",
		static_cast<unsigned long long>(msg->data));
	play_audio_feedback("/awake.wav");

	// aplay returning is the explicit playback completion signal. Keep a
	// short tail guard so the speaker/routing path settles before capture.
	std::this_thread::sleep_for(std::chrono::milliseconds(200));

	std_msgs::msg::UInt64 done_msg;
	done_msg.data = msg->data;
	wake_feedback_done_pub_->publish(done_msg);
	RCLCPP_INFO(this->get_logger(),
		"Awake feedback completed for generation %llu (tail_guard_ms=200)",
		static_cast<unsigned long long>(msg->data));
}

/**************************************************************************
函数功能：雷达跟随开启成功标志位sub回调函数
入口参数：laser_follow_flag.msg  
返回  值：无
**************************************************************************/
void Feedback::laser_follow_Callback(const std_msgs::msg::Int8::SharedPtr msg){
	laser_follow_flag = msg->data;
	if (laser_follow_flag){
		play_audio_feedback("/rplidar_open.wav");
		cout<<"雷达跟随打开成功"<<endl;
	}
}

/**************************************************************************
函数功能：离线命令词识别结果sub回调函数
入口参数：命令词字符串
返回  值：无
**************************************************************************/
void Feedback::voice_words_Callback(const std_msgs::msg::String::SharedPtr msg){
	string str1 = msg->data;    //取传入数据
	string str2 = "关闭雷达跟随";

	if (str1 == str2){
		node_kill("/laserfollower");
		cmd_vel_pub->publish(geometry_msgs::msg::Twist());
		sleep(1);
		cout<<"已关闭雷达跟随"<<endl;
	}
}

Feedback::Feedback(const std::string &node_name)
: rclcpp::Node(node_name){
	RCLCPP_INFO(this->get_logger(),"%s node init!\n",node_name.c_str());
	last_feedback_time_ = this->now();
	/***声明参数并获取***/
	this->declare_parameter<string>("audio_path","");
	this->declare_parameter<std::string>("device_type", "default");
	this->get_parameter("audio_path",audio_path);
	this->get_parameter("device_type", device_type_);
	/***速度话题发布者创建***/
	cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
	/***雷达跟随话题订阅者创建***/
	laser_follow_sub = this->create_subscription<std_msgs::msg::Int8>(
		"laser_follow_flag",10,std::bind(&Feedback::laser_follow_Callback,this,_1));
	/***反馈播报请求话题订阅者创建***/
	feedback_audio_sub = this->create_subscription<std_msgs::msg::String>(
		"feedback_audio",10,std::bind(&Feedback::feedback_audio_Callback,this,_1));
	/***离线命令词识别结果话题订阅者创建***/
	voice_words_sub = this->create_subscription<std_msgs::msg::String>(
		"voice_words",10,std::bind(&Feedback::voice_words_Callback,this,_1));
	wake_feedback_done_pub_ =
		this->create_publisher<std_msgs::msg::UInt64>("wake_feedback_done", 10);
	wake_feedback_request_sub_ = this->create_subscription<std_msgs::msg::UInt64>(
		"wake_feedback_request", 10,
		std::bind(&Feedback::wake_feedback_request_Callback, this, _1));
}

Feedback::~Feedback(){
	RCLCPP_INFO(this->get_logger(),"node_feedback over!\n");
}

void Feedback::run(){
	rclcpp::spin(shared_from_this());
}

int main(int argc, char const *argv[])
{
	rclcpp::init(argc,argv);
	auto node = std::make_shared<Feedback>("node_feedback");
	rclcpp::spin(node);  
	rclcpp::shutdown();
	return 0;
}
