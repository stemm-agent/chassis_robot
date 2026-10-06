/****************************************************************/
/* Copyright (c) 2023 WHEELTEC Technology, Inc   				*/
/* function:Recording call controller, including sleep function	*/
/* 功能：录音调用控制器，包含休眠功能									*/
/****************************************************************/
#include <signal.h>
#include "call_recognition.h"

using namespace std;
using std::placeholders::_1;

void sighandler(int);

/********************************************************
Function:Wake flag result processing 
功能: 唤醒标志位处理回调函数
*********************************************************/
void Call::awake_flag_Callback(std_msgs::msg::Int8::SharedPtr msg)
{
	awake_flag = msg->data;
	if (awake_flag) {
		recognize_fail_count = 0;
		++latest_wake_generation_;
		feedback_pending_ = true;
		call_ = false;

		std_msgs::msg::UInt64 feedback_request;
		feedback_request.data = latest_wake_generation_;
		wake_feedback_request_pub_->publish(feedback_request);

		if (request_in_flight_) {
			std_msgs::msg::UInt64 cancel_msg;
			cancel_msg.data = latest_wake_generation_;
			recognition_cancel_pub_->publish(cancel_msg);
			RCLCPP_INFO(this->get_logger(),
				"Wake generation %llu cancels recognition generation %llu",
				static_cast<unsigned long long>(latest_wake_generation_),
				static_cast<unsigned long long>(in_flight_generation_));
		}
		RCLCPP_INFO(this->get_logger(),
			"Requested awake feedback for generation %llu",
			static_cast<unsigned long long>(latest_wake_generation_));
	} else {
		call_ = false;
		feedback_pending_ = false;
	}
}

void Call::wake_feedback_done_Callback(const std_msgs::msg::UInt64::SharedPtr msg)
{
	if (!awake_flag || msg->data != latest_wake_generation_) {
		RCLCPP_INFO(this->get_logger(),
			"Ignore stale awake feedback completion generation %llu (latest %llu)",
			static_cast<unsigned long long>(msg->data),
			static_cast<unsigned long long>(latest_wake_generation_));
		return;
	}

	completed_feedback_generation_ = msg->data;
	feedback_pending_ = false;
	if (!request_in_flight_) {
		call_ = true;
	}
	RCLCPP_INFO(this->get_logger(),
		"Awake feedback and speaker tail completed for generation %llu",
		static_cast<unsigned long long>(msg->data));
}

/********************************************************
Function: Service result processing 
功能: 服务结果处理回调函数
*********************************************************/
void Call::server_callback_(
	rclcpp::Client<wheeltec_mic_msg::srv::GetOfflineResult>::SharedFuture result,
	uint64_t request_generation)
{
	auto response = result.get();
	request_in_flight_ = false;
	if (request_generation != latest_wake_generation_) {
		RCLCPP_INFO(this->get_logger(),
			"Discard stale recognition response generation %llu (latest %llu)",
			static_cast<unsigned long long>(request_generation),
			static_cast<unsigned long long>(latest_wake_generation_));
		if (awake_flag && !feedback_pending_ &&
			completed_feedback_generation_ == latest_wake_generation_) {
			call_ = true;
		}
		return;
	}

	string str1 = "ok";				//语音识别相关字符串
	string str2 = "fail";						
	string str3 = "小车休眠";					
	string str4 = "失败5次";					
	string str5 = "失败10次";

	if (str3 == response->text){	
		awake_flag=0;
		recognize_fail_count = 0;}
	else if (str1 == response->result){
		awake_flag = 0;
		call_ = false;
		recognize_fail_count = 0;
		RCLCPP_INFO(this->get_logger(),
			"Recognition succeeded; ending the current wake session");
	}
	else if (str2 == response->result){
		recognize_fail_count++;
		if (recognize_fail_count == 5){
			std_msgs::msg::String count_msg;
			count_msg.data = str4;
			voice_words_pub->publish(count_msg);
		}
		else if (recognize_fail_count == 10){
			std_msgs::msg::String count_msg;
			count_msg.data = str5;
			voice_words_pub->publish(count_msg);
		}
		else if (recognize_fail_count == recognize_fail_count_threshold){
			awake_flag = 0;
			recognize_fail_count = 0;
			std_msgs::msg::String off_msg;
			off_msg.data = str3;
			voice_words_pub->publish(off_msg);
		}
		awake_flag = 0;
		call_ = false;
		RCLCPP_INFO(this->get_logger(),
			"Recognition failed; wait for a new wake instead of recording again blindly");
	}
	else{
		RCLCPP_INFO(this->get_logger(),"failed to call service \"get_offline_recognise_result_srv\"!");
	}

}

/********************************************************
Function: Client requests service
功能: 客户端请求服务
*********************************************************/
int Call::request_()
{
	while(!Call_Client_->wait_for_service(std::chrono::seconds(2))){
		// while(!rclcpp::ok()){
		// RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
		// return 1;
		// }
		RCLCPP_INFO(this->get_logger(), "waiting the service...");
	}
	RCLCPP_INFO(this->get_logger(), "Offline_Recognise_Result Service has found.");
	auto request = std::make_shared<wheeltec_mic_msg::srv::GetOfflineResult::Request>();
	/***离线命令词识别服务参数设置***/
	request->offline_recognise_start = 1;
	request->confidence_threshold = confidence_threshold;
	request->time_per_order = seconds_per_order;
	/***循环频率Hz***/
	rclcpp::Rate loop_rate(10);

	while(rclcpp::ok()){
		if (awake_flag && call_ && !request_in_flight_){
			const uint64_t request_generation = latest_wake_generation_;
			request_in_flight_ = true;
			in_flight_generation_ = request_generation;
			Call_Client_->async_send_request(
				request,
				[this, request_generation](
					rclcpp::Client<wheeltec_mic_msg::srv::GetOfflineResult>::SharedFuture result) {
					server_callback_(result, request_generation);
				});
			call_ = false;
			RCLCPP_INFO(this->get_logger(),
				"Start recognition request generation %llu",
				static_cast<unsigned long long>(request_generation));
		}
		rclcpp::spin_some(this->get_node_base_interface());
		loop_rate.sleep();
	}
	return 0;
}

Call::Call(const std::string &node_name,const rclcpp::NodeOptions &options) 
: rclcpp::Node(node_name,options){
	RCLCPP_INFO(this->get_logger(),"%s node init!\n",node_name.c_str());

	this->declare_parameter<int>("confidence_threshold",20);
	this->declare_parameter<int>("time_per_order",10);

	this->get_parameter("confidence_threshold",confidence_threshold);
	this->get_parameter("time_per_order",seconds_per_order);

	/***唤醒标志位话题订阅者创建***/
	awake_flag_sub = this->create_subscription<std_msgs::msg::Int8>(
		"awake_flag",10,std::bind(&Call::awake_flag_Callback,this,_1));
	/***唤醒标志位话题发布者创建***/
	awake_flag_pub = this->create_publisher<std_msgs::msg::Int8>("awake_flag",10);
	/***离线命令词识别结果话题发布者创建***/
	voice_words_pub = this->create_publisher<std_msgs::msg::String>("voice_words",10);
	wake_feedback_request_pub_ =
		this->create_publisher<std_msgs::msg::UInt64>("wake_feedback_request", 10);
	recognition_cancel_pub_ =
		this->create_publisher<std_msgs::msg::UInt64>("recognition_cancel", 10);
	wake_feedback_done_sub_ = this->create_subscription<std_msgs::msg::UInt64>(
		"wake_feedback_done", 10,
		std::bind(&Call::wake_feedback_done_Callback, this, _1));
	/***离线命令词识别服务客服端创建***/
	Call_Client_ = this->create_client<wheeltec_mic_msg::srv::GetOfflineResult>(
		"/get_offline_result_srv");
}

Call::~Call(){
	RCLCPP_INFO(this->get_logger(),"call_recognition_node over!\n");
} 

int main(int argc,char **argv)
{
	rclcpp::init(argc,argv);
	signal(SIGINT, sighandler);
	Call call_node("call_recognition",rclcpp::NodeOptions());
	if (call_node.request_() == 0){
	RCLCPP_INFO(rclcpp::get_logger("call_recognition"),
			"call_recognition done!");
	}
	else{
	RCLCPP_INFO(rclcpp::get_logger("call_recognition"),
			"call_recognition Interrupted!");
	}
	rclcpp::shutdown();
	return 0;
}

void sighandler(int signum)
{
   	printf("capture signal：%d，Interrupt...\n", signum);
   	exit(1);
}
